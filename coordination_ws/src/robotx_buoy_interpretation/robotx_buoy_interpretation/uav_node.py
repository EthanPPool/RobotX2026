"""ROS wrapper for capture-time UAV semantic buoy projection.

Consumes the teammate model's normalized detections, never a second classifier.
This node neither launches nor requires a flight/autonomy stack.
"""
import json
import math
from collections import deque

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from robotx_coordination.geometry import reference_id
from robotx_coordination_interfaces.msg import BuoyColorDetectionArray, BuoyObservationArray

from .ros_support import (camera_from_info, detections_from_msg, observation_array,
                          stamp_seconds, transform_parts)
from .uav import interpret_uav, select_capture_calibration


class UavInterpretation(Node):
    def __init__(self):
        super().__init__('uav_buoy_interpretation')
        p = lambda name, default: self.declare_parameter(name, default).value
        self.frame = str(p('map_frame', 'robotx_map'))
        self.enabled = bool(p('datum_configured', False))
        self.origin = (float(p('origin_latitude', 0.)),
                       float(p('origin_longitude', 0.)), float(p('water_altitude', 0.)))
        self.ref = reference_id(*self.origin)
        self.timeout = float(p('detection_timeout', 0.5))
        self.max_range = float(p('max_mapping_range', 60.))
        self.min_confidence = float(p('min_confidence', 0.6))
        self.uncertainty = float(p('position_uncertainty', 0.75))
        self.pixel_uncertainty = float(p('pixel_uncertainty', 2.))
        self.anchor_fraction = float(p('anchor_fraction', 0.5))
        self.radius = float(p('buoy_radius', 0.2))
        self.height_bound = float(p('max_buoy_height', 1.5))
        self.calibration_timeout = float(p('calibration_timeout', 0.1))
        self.allow_static_calibration = bool(p('allow_static_calibration', False))
        limits = (self.timeout, self.max_range, self.min_confidence, self.uncertainty,
                  self.pixel_uncertainty, self.anchor_fraction, self.radius, self.height_bound,
                  self.calibration_timeout)
        if (not all(math.isfinite(v) for v in limits) or not self.frame
                or min(self.timeout, self.max_range, self.radius, self.calibration_timeout) <= 0
                or min(self.uncertainty, self.pixel_uncertainty, self.height_bound) < 0
                or not 0 <= self.min_confidence <= 1 or not -.5 <= self.anchor_fraction <= .5):
            raise ValueError('Invalid UAV interpretation parameters')
        self.calibrations = deque(maxlen=60)
        self.pending = None
        self.last_received_stamp = 0.
        self.buffer = Buffer(cache_time=Duration(seconds=5.))
        self.listener = TransformListener(self.buffer, self)
        self.publisher = self.create_publisher(BuoyObservationArray,
            str(p('output_topic', '/perception/buoys/uav')), 10)
        self.status_publisher = self.create_publisher(String, '/perception/buoys/uav/status', 10)
        self.create_subscription(CameraInfo, str(p('camera_info_topic', '/uav/downward/camera_info')),
                                 self.on_info, qos_profile_sensor_data)
        self.create_subscription(BuoyColorDetectionArray,
            str(p('detections_topic', '/perception/uav/color_detections')),
            self.on_detections, qos_profile_sensor_data)
        self.create_timer(0.05, self.process_pending)

    def status(self, state):
        msg = String()
        msg.data = json.dumps({'state': state, 'source': 'uav', 'reference_id': self.ref})
        self.status_publisher.publish(msg)

    def on_info(self, msg):
        self.calibrations.append((stamp_seconds(msg.header.stamp), msg.header.frame_id, msg))

    def calibration_at(self, stamp, frame):
        # CameraInfo is normally stamped with its image capture time. Never use
        # a calibration from after that capture, or an arbitrarily old stream.
        return select_capture_calibration(self.calibrations, stamp, frame,
                                         self.calibration_timeout, self.allow_static_calibration)

    def on_detections(self, msg):
        stamp = stamp_seconds(msg.header.stamp)
        now = self.get_clock().now().nanoseconds / 1e9
        if stamp <= 0 or stamp > now or now-stamp > self.timeout:
            self.status('CAPTURE_STALE_OR_FUTURE')
            return
        if stamp <= self.last_received_stamp:
            self.status('REPLAY_OR_OUT_OF_ORDER')
            return
        self.last_received_stamp = stamp
        self.pending = msg  # Bounded queue: newest capture wins.

    def process_pending(self):
        if not self.enabled:
            self.pending = None
            self.status('DATUM_NOT_CONFIGURED')
            return
        msg = self.pending
        if msg is None:
            return
        stamp = stamp_seconds(msg.header.stamp)
        now = self.get_clock().now().nanoseconds / 1e9
        if stamp <= 0 or stamp > now or now-stamp > self.timeout:
            self.pending = None
            self.status('CAPTURE_STALE_OR_FUTURE')
            return
        info = self.calibration_at(stamp, msg.header.frame_id)
        if info is None:
            self.status('WAITING_FOR_CAPTURE_CALIBRATION')
            return
        try:
            camera = camera_from_info(info, msg.image_width, msg.image_height,
                                      msg.header.frame_id)
            transform = self.buffer.lookup_transform(self.frame, msg.header.frame_id,
                                                      Time.from_msg(msg.header.stamp))
            position, quaternion = transform_parts(transform)
            observations = interpret_uav(detections_from_msg(msg), camera, position,
                quaternion, stamp, now, timeout=self.timeout, max_range=self.max_range,
                min_confidence=self.min_confidence, position_uncertainty=self.uncertainty,
                pixel_uncertainty=self.pixel_uncertainty, anchor_fraction=self.anchor_fraction,
                min_radius=self.radius, max_buoy_height=self.height_bound)
            self.publisher.publish(observation_array(observations, self.frame, self.ref,
                                                      msg.header.stamp))
        except TransformException:
            # Retry this capture until its original acquisition stamp expires.
            self.status('WAITING_FOR_CAPTURE_TF')
            return
        except (ValueError, TypeError, OverflowError) as exc:
            self.pending = None
            self.status('REJECTED: '+str(exc))
            return
        self.pending = None
        self.status('PROJECTED' if observations else 'NO_VALID_OBSERVATIONS')


def main(args=None):
    rclpy.init(args=args)
    node = UavInterpretation()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
