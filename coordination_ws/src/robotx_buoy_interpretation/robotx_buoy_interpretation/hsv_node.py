"""Capture-time HSV producer for the existing USV interpretation interface."""
from collections import deque
from copy import deepcopy
import math

import cv2
from cv_bridge import CvBridge, CvBridgeError
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from robotx_coordination_interfaces.msg import BuoyColorDetection, BuoyColorDetectionArray

from .hsv_clusters import DEFAULT_RANGES, Limits, Segmenter, rectify, capture_valid
from .ros_support import camera_from_info, stamp_seconds
from .uav import select_capture_calibration


def stamp_ns(stamp):
    return int(stamp.sec)*1_000_000_000 + int(stamp.nanosec)


def detection_array(image, clusters):
    out = BuoyColorDetectionArray()
    out.header = deepcopy(image.header)
    out.image_width, out.image_height = image.width, image.height
    out.image_rectified = True
    for index, cluster in enumerate(clusters):
        item = BuoyColorDetection()
        item.detection_id = index  # Capture-local, not a persistent tracking ID.
        for name in ('u', 'v', 'width', 'height', 'color', 'confidence'):
            setattr(item, name, cluster[name])
        item.light_state = 'unknown'  # No temporal flashing inference.
        out.detections.append(item)
    return out


class HsvColorClusters(Node):
    def __init__(self):
        super().__init__('usv_hsv_clusters')
        def p(name, default):
            return self.declare_parameter(name, default).value
        image_topic = str(p('image_topic', '')).strip()
        if not image_topic:
            raise ValueError('Set image_topic to the actual USV camera Image stream')
        self.input_rectified = p('input_rectified', False)
        self.timeout = float(p('max_image_age', .5))
        self.calibration_timeout = float(p('calibration_timeout', .1))
        self.allow_static = p('allow_static_calibration', False)
        if not all(math.isfinite(v) and v > 0 for v in (self.timeout, self.calibration_timeout)):
            raise ValueError('Invalid capture deadlines')
        defaults = Limits()
        limits = Limits(**{n: p(n, getattr(defaults, n)) for n in defaults.__dataclass_fields__})
        ranges = {c: p(c+'_hsv', v) for c, v in DEFAULT_RANGES.items()}
        self.segmenter = Segmenter(limits, ranges)
        self.bridge = CvBridge()
        self.calibrations = deque(maxlen=60)
        self.pending = None
        self.last_stamp = 0
        self.publisher = self.create_publisher(
            BuoyColorDetectionArray, p('detections_topic', '/perception/usv/color_detections'), 10)
        self.create_subscription(CameraInfo,
            p('camera_info_topic', '/usv/camera/color/camera_info'),
            self.on_info, qos_profile_sensor_data)
        self.create_subscription(Image, image_topic, self.on_image, qos_profile_sensor_data)
        # Retry a bounded latest image when CameraInfo arrives after Image.
        self.create_timer(.02, self.process_pending)

    def warn(self, message):
        self.get_logger().warning(message, throttle_duration_sec=5.)

    def on_info(self, info):
        self.calibrations.append((stamp_seconds(info.header.stamp), info.header.frame_id, info))
        self.process_pending()

    def on_image(self, image):
        stamp = stamp_ns(image.header.stamp)
        now = self.get_clock().now().nanoseconds
        if not image.header.frame_id or not capture_valid(stamp, now, self.last_stamp, self.timeout):
            return
        if self.pending is None or stamp > stamp_ns(self.pending.header.stamp):
            self.pending = image
        self.process_pending()

    def process_pending(self):
        image = self.pending
        if image is None:
            return
        stamp = stamp_ns(image.header.stamp)
        if not capture_valid(stamp, self.get_clock().now().nanoseconds, self.last_stamp, self.timeout):
            self.pending = None
            return
        # Use the same conversion as CameraInfo and the existing interpreter.
        # Dividing epoch nanoseconds can round differently from sec + nsec*1e-9
        # and misclassify an equal-time calibration as a future measurement.
        info = select_capture_calibration(self.calibrations, stamp_seconds(image.header.stamp),
            image.header.frame_id, self.calibration_timeout, self.allow_static)
        if info is None:
            self.warn('Waiting for matching capture-time camera calibration')
            return
        try:
            # Validate exactly the downstream interpreter's camera conventions.
            camera_from_info(info, image.width, image.height, image.header.frame_id)
            bgr = self.bridge.imgmsg_to_cv2(image, desired_encoding='bgr8')
            if bgr.shape[:2] != (image.height, image.width):
                raise ValueError('Image payload dimensions mismatch')
            pixels = rectify(bgr, info, self.input_rectified)
            clusters, _ = self.segmenter.detect(pixels)
            if capture_valid(stamp, self.get_clock().now().nanoseconds, self.last_stamp, self.timeout):
                self.publisher.publish(detection_array(image, clusters))
                self.last_stamp = stamp
        except (CvBridgeError, cv2.error, ValueError, TypeError) as exc:
            self.warn('HSV image rejected: ' + str(exc))
        finally:
            self.pending = None


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = HsvColorClusters()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
