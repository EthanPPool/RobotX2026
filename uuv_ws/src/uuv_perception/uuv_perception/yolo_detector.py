#!/usr/bin/env python3
import json
import os
import time
from typing import Any

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String


class UuvYoloDetector(Node):
    """Turns the UUV camera into a ROS sensor.

    Always publishes normalized JSON detections. The inference backend can be
    local Ultralytics/TensorRT or a self-hosted Roboflow Inference server.
    """

    def __init__(self):
        super().__init__('uuv_yolo_detector')
        self.backend = str(self.declare_parameter('backend', 'ultralytics').value).lower()
        self.model_path = str(self.declare_parameter('model_path', 'models/uuv_yolo.pt').value)
        self.model_id = str(self.declare_parameter('roboflow_model_id', '').value)
        self.api_url = str(self.declare_parameter('roboflow_api_url', 'http://127.0.0.1:9001').value)
        self.api_key_env = str(self.declare_parameter('roboflow_api_key_env', 'ROBOFLOW_API_KEY').value)
        self.confidence = float(self.declare_parameter('confidence', 0.45).value)
        self.imgsz = int(self.declare_parameter('imgsz', 640).value)
        self.device = str(self.declare_parameter('device', '0').value)
        self.max_fps = float(self.declare_parameter('max_fps', 12.0).value)
        self.image_topic = str(self.declare_parameter('image_topic', '/uuv/camera/image_raw').value)
        self.detections_topic = str(self.declare_parameter('detections_topic', '/uuv/perception/detections').value)
        self.status_topic = str(self.declare_parameter('status_topic', '/uuv/perception/status').value)
        self.annotated_topic = str(self.declare_parameter('annotated_topic', '/uuv/perception/annotated').value)
        self.publish_annotated = bool(self.declare_parameter('publish_annotated', True).value)

        self.bridge = CvBridge()
        self.det_pub = self.create_publisher(String, self.detections_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.annotated_pub = self.create_publisher(Image, self.annotated_topic, 2)
        self.sub = self.create_subscription(Image, self.image_topic, self._image_cb, 5)

        self.engine: Any = None
        self.last_infer_start = 0.0
        self.last_success = None
        self.fps_ema = 0.0
        self.last_error = ''
        self.last_count = 0
        self.last_top_label = None
        self.last_top_confidence = None
        self._load_backend()
        self.status_timer = self.create_timer(1.0, self._publish_status)

    def _load_backend(self):
        try:
            if self.backend == 'ultralytics':
                from ultralytics import YOLO
                self.engine = YOLO(self.model_path)
                self.get_logger().info(f'Loaded Ultralytics model: {self.model_path}')
            elif self.backend == 'roboflow':
                from inference_sdk import InferenceHTTPClient
                key = os.environ.get(self.api_key_env, '')
                self.engine = InferenceHTTPClient(api_url=self.api_url, api_key=key)
                if not self.model_id:
                    raise RuntimeError('roboflow_model_id is empty')
                self.get_logger().info(f'Using Roboflow Inference: {self.api_url} model={self.model_id}')
            else:
                raise RuntimeError(f'unsupported perception backend: {self.backend}')
            self.last_error = ''
        except Exception as exc:
            self.engine = None
            self.last_error = str(exc)
            self.get_logger().error(f'perception backend unavailable: {exc}')

    @staticmethod
    def _color_for_class(class_id: int):
        palette = [(255, 180, 0), (0, 220, 0), (0, 180, 255), (255, 0, 180), (180, 255, 0)]
        return palette[class_id % len(palette)]

    def _run_ultralytics(self, frame):
        result = self.engine.predict(
            source=frame,
            conf=self.confidence,
            imgsz=self.imgsz,
            device=self.device,
            verbose=False,
        )[0]
        names = result.names or {}
        detections = []
        if result.boxes is not None:
            for box in result.boxes:
                xyxy = box.xyxy[0].tolist()
                class_id = int(box.cls[0].item())
                score = float(box.conf[0].item())
                detections.append({
                    'class_id': class_id,
                    'label': str(names.get(class_id, class_id)),
                    'confidence': score,
                    'x1': float(xyxy[0]), 'y1': float(xyxy[1]),
                    'x2': float(xyxy[2]), 'y2': float(xyxy[3]),
                })
        return detections

    def _run_roboflow(self, frame):
        result = self.engine.infer(frame, model_id=self.model_id)
        if isinstance(result, list) and result:
            result = result[0]
        predictions = (result or {}).get('predictions', []) if isinstance(result, dict) else []
        detections = []
        for pred in predictions:
            score = float(pred.get('confidence', 0.0))
            if score < self.confidence:
                continue
            x = float(pred.get('x', 0.0)); y = float(pred.get('y', 0.0))
            w = float(pred.get('width', 0.0)); h = float(pred.get('height', 0.0))
            detections.append({
                'class_id': int(pred.get('class_id', -1)),
                'label': str(pred.get('class', pred.get('class_name', 'unknown'))),
                'confidence': score,
                'x1': x - w / 2.0, 'y1': y - h / 2.0,
                'x2': x + w / 2.0, 'y2': y + h / 2.0,
            })
        return detections

    def _image_cb(self, msg: Image):
        now = time.monotonic()
        if self.max_fps > 0.0 and now - self.last_infer_start < 1.0 / self.max_fps:
            return
        self.last_infer_start = now
        if self.engine is None:
            self._load_backend()
            if self.engine is None:
                return
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            t0 = time.monotonic()
            detections = self._run_ultralytics(frame) if self.backend == 'ultralytics' else self._run_roboflow(frame)
            elapsed = time.monotonic() - t0
            instant_fps = 1.0 / max(elapsed, 1e-6)
            self.fps_ema = instant_fps if self.fps_ema <= 0.0 else 0.9 * self.fps_ema + 0.1 * instant_fps
            self.last_success = time.monotonic()
            self.last_error = ''
            self.last_count = len(detections)
            top = max(detections, key=lambda d: d['confidence']) if detections else None
            self.last_top_label = top['label'] if top else None
            self.last_top_confidence = top['confidence'] if top else None

            payload = {
                'stamp_sec': msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9,
                'frame_id': msg.header.frame_id,
                'backend': self.backend,
                'model': self.model_path if self.backend == 'ultralytics' else self.model_id,
                'image_width': int(frame.shape[1]),
                'image_height': int(frame.shape[0]),
                'detections': detections,
            }
            out = String(); out.data = json.dumps(payload, separators=(',', ':'))
            self.det_pub.publish(out)

            if self.publish_annotated:
                annotated = frame.copy()
                for det in detections:
                    x1, y1, x2, y2 = map(int, (det['x1'], det['y1'], det['x2'], det['y2']))
                    color = self._color_for_class(int(det['class_id']))
                    cv2.rectangle(annotated, (x1, y1), (x2, y2), color, 2)
                    cv2.putText(annotated, f"{det['label']} {det['confidence']:.2f}", (x1, max(15, y1 - 5)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)
                ann = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
                ann.header = msg.header
                self.annotated_pub.publish(ann)
        except Exception as exc:
            self.last_error = str(exc)
            self.get_logger().warn(f'inference failed: {exc}')

    def _publish_status(self):
        age = None if self.last_success is None else time.monotonic() - self.last_success
        alive = bool(age is not None and age < 2.0)
        data = {
            'perception_alive': alive,
            'perception_backend': self.backend,
            'perception_model': self.model_path if self.backend == 'ultralytics' else self.model_id,
            'perception_fps': round(self.fps_ema, 2),
            'perception_age_sec': None if age is None else round(age, 3),
            'detection_count': self.last_count,
            'top_detection_label': self.last_top_label,
            'top_detection_confidence': self.last_top_confidence,
            'perception_error': self.last_error or None,
        }
        msg = String(); msg.data = json.dumps(data, separators=(',', ':'))
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = UuvYoloDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
