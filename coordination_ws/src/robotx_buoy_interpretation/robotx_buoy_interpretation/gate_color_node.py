"""Two-gate color adapter; no shared datum, beacon tracker, or new mission law."""
from collections import deque
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformException, TransformListener
from boat_interfaces.msg import DetectedObjectArray
from boat_perception.pose_history import PoseHistory, to_map, to_body
from robotx_coordination.geometry import rotation
from robotx_coordination_interfaces.msg import BuoyColorDetectionArray

from .gate_colors import color_objects, unknown_objects
from .ros_support import camera_from_info, detections_from_msg, stamp_seconds, transform_parts
from .uav import select_capture_calibration


class GateColorAdapter(Node):
    def __init__(self):
        super().__init__('two_gate_colors')
        def p(name, default):
            return self.declare_parameter(name, default).value
        self.timeout = float(p('capture_timeout', .5))
        self.max_sync = float(p('max_sync', .1))
        self.wait = float(p('association_wait', .2))
        self.calibration_timeout = float(p('calibration_timeout', .1))
        self.allow_static = p('allow_static_calibration', False)
        self.min_confidence = float(p('min_color_confidence', .6))
        self.padding = float(p('bbox_padding', 8.))
        tolerance = float(p('pose_history_tolerance', .15))
        if (not all(math.isfinite(v) for v in (self.timeout, self.max_sync,
                self.wait, self.calibration_timeout, self.min_confidence, self.padding, tolerance))
                or not 0 < self.wait < self.timeout or self.max_sync < 0
                or min(self.calibration_timeout, tolerance) <= 0
                or not 0 <= self.min_confidence <= 1 or self.padding < 0):
            raise ValueError('Invalid two-gate color association limits')
        self.pose_history = PoseHistory(tolerance=tolerance)
        self.pose_frame = None
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.images, self.calibrations, self.pending = deque(maxlen=30), deque(maxlen=60), deque(maxlen=20)
        self.last_geometry = 0.
        input_topic = p('objects_topic', '/perception/objects')
        output_topic = p('output_topic', '/perception/colored_objects')
        if input_topic == output_topic:
            raise ValueError('Color adapter must not overwrite the raw LiDAR topic')
        self.publisher = self.create_publisher(DetectedObjectArray, output_topic, 10)
        self.create_subscription(DetectedObjectArray, input_topic, self.on_objects, qos_profile_sensor_data)
        self.create_subscription(BuoyColorDetectionArray,
            p('detections_topic', '/perception/usv/color_detections'), self.on_image, qos_profile_sensor_data)
        self.create_subscription(CameraInfo,
            p('camera_info_topic', '/usv/camera/color/camera_info'), self.on_info, qos_profile_sensor_data)
        self.create_subscription(PoseStamped,
            p('local_position_topic', '/mavros/local_position/pose'), self.on_pose, qos_profile_sensor_data)
        self.create_timer(.02, self.process)

    def now(self):
        return self.get_clock().now().nanoseconds/1e9

    def fresh(self, stamp):
        return stamp > 0 and 0 <= self.now()-stamp <= self.timeout

    def on_pose(self, msg):
        frame = msg.header.frame_id
        if not frame or self.pose_frame is not None and frame != self.pose_frame:
            return
        p, q = msg.pose.position, msg.pose.orientation
        try:
            r = rotation((q.x, q.y, q.z, q.w))
            yaw = math.atan2(r[1, 0], r[0, 0])
            if self.pose_history.add(stamp_seconds(msg.header.stamp), p.x, p.y, yaw):
                self.pose_frame = frame
        except ValueError:
            return
        self.process()

    def on_info(self, msg):
        self.calibrations.append((stamp_seconds(msg.header.stamp), msg.header.frame_id, msg))
        self.process()

    def on_image(self, msg):
        if msg.image_rectified and msg.header.frame_id and self.fresh(stamp_seconds(msg.header.stamp)):
            self.images.append(msg)
            self.process()

    def on_objects(self, msg):
        stamp = stamp_seconds(msg.header.stamp)
        if msg.header.frame_id != 'base_link' or not self.fresh(stamp) or stamp <= self.last_geometry:
            return
        self.last_geometry = stamp
        self.pending.append(msg)
        self.process()

    def associate(self, objects):
        stamp = stamp_seconds(objects.header.stamp)
        images = [m for m in self.images if self.fresh(stamp_seconds(m.header.stamp))
                  and abs(stamp_seconds(m.header.stamp)-stamp) <= self.max_sync]
        if not images:
            return None
        image = min(images, key=lambda m: abs(stamp_seconds(m.header.stamp)-stamp))
        image_stamp = stamp_seconds(image.header.stamp)
        info = select_capture_calibration(self.calibrations, image_stamp,
            image.header.frame_id, self.calibration_timeout, self.allow_static)
        if info is None:
            return None
        camera = camera_from_info(info, image.image_width, image.image_height, image.header.frame_id)
        poses = None
        if image.header.stamp != objects.header.stamp:
            poses = (self.pose_history.at(stamp), self.pose_history.at(image_stamp))
            if None in poses:
                return None
        tf = self.buffer.lookup_transform(image.header.frame_id, 'base_link',
                                          Time.from_msg(image.header.stamp))
        translation, quaternion = transform_parts(tf)
        matrix, translation = rotation(quaternion), np.asarray(translation)
        def optical(point):
            x, y, z = point
            if poses is not None:
                x, y = to_body(to_map((x, y), poses[0]), poses[1])
            return matrix @ np.asarray((x, y, z)) + translation
        return color_objects(objects, detections_from_msg(image), camera, optical,
                             self.min_confidence, self.padding)

    def process(self):
        while self.pending:
            objects = self.pending[0]
            stamp = stamp_seconds(objects.header.stamp)
            if not self.fresh(stamp):
                self.pending.popleft()
                continue
            result = None
            try:
                result = self.associate(objects)
            except (TransformException, ValueError, TypeError) as exc:
                self.get_logger().warning('Color association unavailable: '+str(exc), throttle_duration_sec=5.)
            if result is None and self.now()-stamp < self.wait:
                return
            self.pending.popleft()
            # Publish once per distinct original scan stamp. Late camera data
            # cannot create a second confirmation hit or refresh old geometry.
            self.publisher.publish(result if result is not None else unknown_objects(objects))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = GateColorAdapter()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
