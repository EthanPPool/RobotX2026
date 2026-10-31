"""ROS 2 adapter for source-independent USV buoy observations.

No commands are sent to propulsion. The detector preserves the original image
stamp/optical frame. LiDAR positions are transformed to map at LiDAR capture,
then into the camera at image capture: TF latest is never used.
"""
from collections import deque
import json
import math

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from boat_interfaces.msg import DetectedObjectArray
from robotx_coordination.geometry import reference_id, rotation
from robotx_coordination_interfaces.msg import BuoyColorDetectionArray, BuoyObservationArray

from .ros_support import (camera_from_info, detections_from_msg, observation_array,
                          stamp_seconds, transform_parts)
from .usv import GeometryCandidate, geometry_observations, interpret_usv
from .uav import select_capture_calibration


class UsvInterpreter(Node):
    def __init__(self):
        super().__init__('usv_buoy_interpretation')
        p = lambda name, default: self.declare_parameter(name, default).value
        self.frame = str(p('map_frame', 'robotx_map'))
        self.enabled = bool(p('datum_configured', False))
        self.ref = reference_id(float(p('origin_latitude', 0.0)),
                                float(p('origin_longitude', 0.0)),
                                float(p('water_altitude', 0.0)))
        self.timeout = float(p('detection_timeout', 0.5))
        self.max_sync = float(p('max_sync', 0.1))
        self.min_confidence = float(p('min_confidence', 0.6))
        self.pixel_gate = float(p('max_pixel_distance', 80.0))
        self.padding = float(p('bbox_padding', 8.0))
        self.uncertainty = float(p('geometry_uncertainty', 0.25))
        self.calibration_timeout = float(p('calibration_timeout', 0.1))
        self.allow_static_calibration = bool(p('allow_static_calibration', False))
        if (not all(math.isfinite(v) for v in (self.timeout, self.max_sync,
                self.min_confidence, self.pixel_gate, self.padding, self.uncertainty,
                self.calibration_timeout))
                or self.timeout <= 0 or self.max_sync < 0 or self.pixel_gate <= 0
                or self.padding < 0 or self.uncertainty < 0
                or self.calibration_timeout <= 0
                or not 0 <= self.min_confidence <= 1):
            raise ValueError('Invalid USV interpretation limits')
        self.buffer = Buffer(cache_time=Duration(seconds=5.0))
        self.listener = TransformListener(self.buffer, self)
        self.geometries = deque(maxlen=20)
        self.pending = None
        self.calibrations = deque(maxlen=60)
        self.last_image_stamp = 0.0
        self.last_geometry_stamp = 0.0
        self.pending_geometry = None
        self.publisher = self.create_publisher(BuoyObservationArray,
            p('output_topic', '/perception/buoys/usv'), 10)
        self.status_publisher = self.create_publisher(String,
            p('status_topic', '/perception/buoys/usv/status'), 10)
        self.create_subscription(CameraInfo,
            p('camera_info_topic', '/usv/camera/color/camera_info'),
            self.on_info, qos_profile_sensor_data)
        self.create_subscription(BuoyColorDetectionArray,
            p('detections_topic', '/perception/usv/color_detections'),
            self.on_detection, qos_profile_sensor_data)
        self.create_subscription(DetectedObjectArray,
            p('objects_topic', '/perception/objects'),
            self.on_geometry, qos_profile_sensor_data)
        self.create_timer(0.05, self.process_pending)

    def status(self, reason):
        msg = String()
        msg.data = json.dumps({'source': 'usv', 'state': reason,
                               'reference_id': self.ref})
        self.status_publisher.publish(msg)

    def now_seconds(self):
        return self.get_clock().now().nanoseconds/1e9

    def on_info(self, msg):
        self.calibrations.append((stamp_seconds(msg.header.stamp),
                                  msg.header.frame_id, msg))

    def calibration_at(self, stamp, frame):
        return select_capture_calibration(self.calibrations, stamp, frame,
            self.calibration_timeout, self.allow_static_calibration)

    def on_detection(self, msg):
        stamp = stamp_seconds(msg.header.stamp)
        try:
            geometry_observations([], stamp, self.now_seconds(), self.timeout)
        except ValueError as exc:
            self.status('IMAGE_REJECTED: ' + str(exc))
            return
        if stamp <= self.last_image_stamp:
            self.status('REPLAYED_IMAGE')
            return
        if self.pending is None or stamp > stamp_seconds(self.pending.header.stamp):
            self.pending = msg

    def on_geometry(self, msg):
        stamp = stamp_seconds(msg.header.stamp)
        try:
            geometry_observations([], stamp, self.now_seconds(), self.timeout)
        except ValueError as exc:
            self.status('GEOMETRY_REJECTED: ' + str(exc))
            return
        if stamp <= self.last_geometry_stamp:
            self.status('REPLAYED_GEOMETRY')
            return
        self.last_geometry_stamp = stamp
        self.geometries.append(msg)
        self.pending_geometry = msg
        self.process_pending()

    def _mapped(self, msg):
        if not msg.header.frame_id:
            raise ValueError('Missing geometry frame')
        if msg.header.frame_id == self.frame:
            matrix, translation = np.eye(3), np.zeros(3)
        else:
            tf = self.buffer.lookup_transform(self.frame, msg.header.frame_id,
                                              Time.from_msg(msg.header.stamp))
            translation, quaternion = transform_parts(tf)
            matrix = rotation(quaternion)
            translation = np.asarray(translation)
        candidates = []
        for obj in msg.objects:
            if obj.object_type not in (0, 1):
                continue
            point = matrix @ np.array((obj.position.x, obj.position.y, obj.position.z)) + translation
            radius = max(0.05, float(obj.size.x)/2, float(obj.size.y)/2)
            candidate = GeometryCandidate(int(obj.id), tuple(point), radius=radius,
                uncertainty=self.uncertainty, confidence=float(obj.confidence))
            candidate.validate()
            candidates.append(candidate)
        return candidates

    def _publish(self, observations, stamp_msg):
        self.publisher.publish(observation_array(observations, self.frame,
                                                 self.ref, stamp_msg))

    def process_pending(self):
        if not self.enabled:
            self.status('DATUM_NOT_CONFIGURED')
            return
        now = self.now_seconds()
        if self.pending_geometry is not None:
            msg = self.pending_geometry
            try:
                stamp = stamp_seconds(msg.header.stamp)
                # Validate freshness before attempting TF or emitting observations.
                geometry_observations([], stamp, now, self.timeout)
                self._publish(geometry_observations(self._mapped(msg), stamp, now,
                                                   self.timeout), msg.header.stamp)
                self.pending_geometry = None
            except TransformException:
                self.status('WAITING_FOR_GEOMETRY_TF')
            except (ValueError, TypeError) as exc:
                self.pending_geometry = None
                self.status('GEOMETRY_REJECTED: ' + str(exc))
        if self.pending is None:
            return
        image = self.pending
        stamp = stamp_seconds(image.header.stamp)
        try:
            geometry_observations([], stamp, now, self.timeout)
            info = self.calibration_at(stamp, image.header.frame_id)
            if info is None:
                self.status('WAITING_FOR_CAPTURE_CALIBRATION')
                return
            camera = camera_from_info(info, image.image_width,
                                      image.image_height, image.header.frame_id)
            available = [g for g in self.geometries
                         if abs(stamp_seconds(g.header.stamp)-stamp) <= self.max_sync]
            if not available:
                self.status('WAITING_FOR_SYNCHRONIZED_GEOMETRY')
                return
            geometry = min(available,
                key=lambda g: abs(stamp_seconds(g.header.stamp)-stamp))
            candidates = self._mapped(geometry)
            tf = self.buffer.lookup_transform(image.header.frame_id, self.frame,
                                              Time.from_msg(image.header.stamp))
            translation, quaternion = transform_parts(tf)
            matrix = rotation(quaternion)
            positioned = [GeometryCandidate(c.candidate_id, c.map_position,
                tuple(matrix @ np.asarray(c.map_position) + np.asarray(translation)),
                c.radius, c.uncertainty, c.confidence) for c in candidates]
            observations = interpret_usv(detections_from_msg(image), positioned,
                camera, stamp, stamp_seconds(geometry.header.stamp), now,
                self.timeout, self.max_sync, self.min_confidence,
                self.pixel_gate, self.padding)
            self._publish(observations, image.header.stamp)
            self.last_image_stamp = stamp
            self.pending = None
            self.status('INTERPRETING')
        except TransformException:
            self.status('WAITING_FOR_CAPTURE_TF')
        except (ValueError, TypeError) as exc:
            self.last_image_stamp = max(self.last_image_stamp, stamp)
            self.pending = None
            self.status('IMAGE_REJECTED: ' + str(exc))


def main(args=None):
    rclpy.init(args=args)
    node = UsvInterpreter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
