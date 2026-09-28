#!/usr/bin/env python3
import json
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String


class UuvCameraNode(Node):
    def __init__(self):
        super().__init__('uuv_camera')
        self.source = str(self.declare_parameter('source', '/dev/video0').value)
        self.width = int(self.declare_parameter('width', 1280).value)
        self.height = int(self.declare_parameter('height', 720).value)
        self.target_fps = float(self.declare_parameter('fps', 15.0).value)
        self.frame_id = str(self.declare_parameter('frame_id', 'uuv_camera_optical_frame').value)
        self.reconnect_sec = float(self.declare_parameter('reconnect_sec', 2.0).value)
        self.image_topic = str(self.declare_parameter('image_topic', '/uuv/camera/image_raw').value)
        self.status_topic = str(self.declare_parameter('status_topic', '/uuv/camera/status').value)

        self.bridge = CvBridge()
        self.pub = self.create_publisher(Image, self.image_topic, 5)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.capture = None
        self.last_open_attempt = 0.0
        self.last_frame_time = None
        self.fps_ema = 0.0
        self.last_error = ''
        self.frames = 0

        self._open_capture()
        self.timer = self.create_timer(max(0.01, 1.0 / max(self.target_fps, 1.0)), self._tick)
        self.status_timer = self.create_timer(1.0, self._publish_status)

    def _open_capture(self):
        self.last_open_attempt = time.monotonic()
        if self.capture is not None:
            self.capture.release()
            self.capture = None
        try:
            if self.source.startswith('gst:'):
                self.capture = cv2.VideoCapture(self.source[4:], cv2.CAP_GSTREAMER)
            elif self.source.isdigit():
                self.capture = cv2.VideoCapture(int(self.source))
            else:
                self.capture = cv2.VideoCapture(self.source)
            if self.capture is not None:
                self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
                self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
                self.capture.set(cv2.CAP_PROP_FPS, self.target_fps)
            if self.capture is None or not self.capture.isOpened():
                self.last_error = f'camera open failed: {self.source}'
                self.get_logger().warn(self.last_error)
            else:
                self.last_error = ''
                self.get_logger().info(f'UUV camera opened: {self.source}')
        except Exception as exc:
            self.last_error = str(exc)
            self.get_logger().error(f'camera open error: {exc}')

    def _tick(self):
        now = time.monotonic()
        if self.capture is None or not self.capture.isOpened():
            if now - self.last_open_attempt >= self.reconnect_sec:
                self._open_capture()
            return

        ok, frame = self.capture.read()
        if not ok or frame is None:
            self.last_error = 'camera frame read failed'
            if now - self.last_open_attempt >= self.reconnect_sec:
                self._open_capture()
            return

        if self.last_frame_time is not None:
            instant = 1.0 / max(now - self.last_frame_time, 1e-6)
            self.fps_ema = instant if self.fps_ema <= 0.0 else (0.9 * self.fps_ema + 0.1 * instant)
        self.last_frame_time = now
        self.frames += 1
        self.last_error = ''

        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        self.pub.publish(msg)

    def _publish_status(self):
        age = None if self.last_frame_time is None else time.monotonic() - self.last_frame_time
        alive = bool(age is not None and age < max(1.0, 3.0 / max(self.target_fps, 1.0)))
        status = {
            'camera_alive': alive,
            'camera_source': self.source,
            'camera_fps': round(self.fps_ema, 2),
            'camera_frame_age_sec': None if age is None else round(age, 3),
            'camera_frames': self.frames,
            'camera_width': self.width,
            'camera_height': self.height,
            'camera_error': self.last_error or None,
        }
        msg = String()
        msg.data = json.dumps(status, separators=(',', ':'))
        self.status_pub.publish(msg)

    def destroy_node(self):
        if self.capture is not None:
            self.capture.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UuvCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
