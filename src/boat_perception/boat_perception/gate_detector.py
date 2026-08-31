#!/usr/bin/env python3

import itertools
import math

import rclpy
from rclpy.node import Node

from visualization_msgs.msg import Marker, MarkerArray

from boat_interfaces.msg import DetectedObject, DetectedObjectArray, Gate


class GateDetector(Node):

    def __init__(self):
        super().__init__('gate_detector')

        self.objects_topic = self.declare_parameter(
            'objects_topic', '/perception/objects'
        ).value
        self.gate_topic = self.declare_parameter(
            'gate_topic', '/perception/gate'
        ).value
        self.marker_topic = self.declare_parameter(
            'marker_topic', '/perception/gate_markers'
        ).value

        self.min_buoy_confidence = float(
            self.declare_parameter(
                'min_buoy_confidence', 0.40
            ).value
        )

        self.min_gate_width = float(
            self.declare_parameter('min_gate_width', 0.92).value
        )
        self.max_gate_width = float(
            self.declare_parameter('max_gate_width', 5.50).value
        )
        self.nominal_gate_width = float(
            self.declare_parameter('nominal_gate_width', 3.0).value
        )
        self.gate_width_tolerance = float(
            self.declare_parameter(
                'gate_width_tolerance', 1.5
            ).value
        )

        # Maximum angle between a square-on gate and the
        # gate as observed from the drifting/yawing boat.
        #
        # 0 deg = perfectly square to gate
        # 55 deg = substantial yaw/drift still accepted
        self.max_gate_skew_deg = float(
            self.declare_parameter(
                'max_gate_skew_deg', 55.0
            ).value
        )
        self.max_gate_skew_rad = math.radians(
            self.max_gate_skew_deg
        )

        self.max_depth_difference = float(
            self.declare_parameter(
                'max_depth_difference', 1.20
            ).value
        )

        self.min_lateral_fraction = float(
            self.declare_parameter(
                'min_lateral_fraction', 0.75
            ).value
        )

        self.min_center_x = float(
            self.declare_parameter('min_center_x', 0.60).value
        )
        self.max_center_x = float(
            self.declare_parameter('max_center_x', 15.0).value
        )

        self.min_gate_confidence = float(
            self.declare_parameter(
                'min_gate_confidence', 0.60
            ).value
        )

        # Temporal gate confirmation.
        self.confirm_hits = int(
            self.declare_parameter('confirm_hits', 3).value
        )
        self.max_misses = int(
            self.declare_parameter('max_misses', 2).value
        )

        # Publish the latest confirmed measurement at a steady rate so
        # downstream control does not stutter when 3D perception is slower
        # than the controller.  Publication stops after hold_timeout if no
        # new real buoy-pair measurement arrives.
        self.publish_rate = float(
            self.declare_parameter('publish_rate', 10.0).value
        )
        self.hold_timeout = float(
            self.declare_parameter('hold_timeout', 1.80).value
        )
        self.center_association_distance = float(
            self.declare_parameter(
                'center_association_distance', 0.80
            ).value
        )

        # Add association allowance as range increases.
        # This helps compensate for boat motion and angular
        # jitter in the body-frame LiDAR measurements.
        self.center_association_per_meter = float(
            self.declare_parameter(
                'center_association_per_meter', 0.05
            ).value
        )
        self.width_association_tolerance = float(
            self.declare_parameter(
                'width_association_tolerance', 0.90
            ).value
        )
        self.track_alpha = float(
            self.declare_parameter('track_alpha', 0.65).value
        )

        self.tracked_center_x = None
        self.tracked_center_y = None
        self.tracked_width = None
        self.tracked_confidence = 0.0
        self.tracked_left = None
        self.tracked_right = None

        self.hits = 0
        self.misses = 0
        self.last_measurement_header = None
        self.last_measurement_time = None
        self.output_active = False

        self.gate_pub = self.create_publisher(
            Gate,
            self.gate_topic,
            10
        )
        self.marker_pub = self.create_publisher(
            MarkerArray,
            self.marker_topic,
            10
        )

        self.objects_sub = self.create_subscription(
            DetectedObjectArray,
            self.objects_topic,
            self.objects_callback,
            10
        )

        self.publish_timer = self.create_timer(
            1.0 / max(self.publish_rate, 1.0),
            self.publish_tracked_gate
        )

        self.get_logger().info(
            'Robust gate detector started: '
            f'width={self.min_gate_width:.1f}-'
            f'{self.max_gate_width:.1f} m, '
            f'confirm_hits={self.confirm_hits}'
        )

    def objects_callback(self, msg):
        buoys = [
            obj
            for obj in msg.objects
            if obj.object_type == DetectedObject.TYPE_BUOY
            and obj.confidence >= self.min_buoy_confidence
            and obj.position.x >= self.min_center_x
            and obj.position.x <= self.max_center_x
        ]

        candidate = self.find_best_pair(buoys)

        if candidate is None:
            self.misses += 1

            if self.misses > self.max_misses:
                self.reset_track()
            return

        self.update_gate_track(candidate)

        if self.hits < self.confirm_hits:
            self.last_measurement_header = None
            self.last_measurement_time = None
            return

        if self.tracked_confidence < self.min_gate_confidence:
            self.last_measurement_header = None
            self.last_measurement_time = None
            return

        # This timestamp identifies one genuine perception measurement.
        # Timer republishes retain it so the controller can distinguish
        # held output from new passage evidence.
        self.last_measurement_header = msg.header
        self.last_measurement_time = self.get_clock().now()

    def measurement_age(self):
        if self.last_measurement_time is None:
            return None

        return (
            self.get_clock().now()
            - self.last_measurement_time
        ).nanoseconds / 1e9

    def publish_tracked_gate(self):
        age = self.measurement_age()

        publishable = (
            age is not None
            and age <= self.hold_timeout
            and self.last_measurement_header is not None
            and self.tracked_center_x is not None
            and self.hits >= self.confirm_hits
            and self.tracked_confidence
                >= self.min_gate_confidence
        )

        if not publishable:
            if self.output_active:
                self.publish_empty_markers()
                self.output_active = False
            return

        gate = Gate()
        gate.header = self.last_measurement_header
        gate.header.frame_id = 'base_link'

        gate.left_marker = self.tracked_left
        gate.right_marker = self.tracked_right

        gate.center.x = float(self.tracked_center_x)
        gate.center.y = float(self.tracked_center_y)
        gate.center.z = 0.0

        gate.width = float(self.tracked_width)
        gate.confidence = float(self.tracked_confidence)

        self.gate_pub.publish(gate)
        self.publish_gate_markers(gate)
        self.output_active = True

    def find_best_pair(self, buoys):
        best = None

        for a, b in itertools.combinations(buoys, 2):
            dx = a.position.x - b.position.x
            dy = a.position.y - b.position.y

            width = math.hypot(dx, dy)

            if width < self.min_gate_width:
                continue

            if width > self.max_gate_width:
                continue

            # ------------------------------------------------
            # DRIFT / YAW TOLERANT GATE GEOMETRY
            # ------------------------------------------------
            #
            # The previous detector rejected a gate whenever
            # one buoy was > max_depth_difference farther ahead
            # than the other. That is not rotation invariant:
            # simply yawing the boat can make a real gate fail.
            #
            # Instead, calculate the observed skew of the gate.
            # A square-on gate has dx ~= 0. A yawed/drifting
            # boat produces increasing dx while gate width
            # itself remains approximately unchanged.
            depth_difference = abs(dx)

            lateral_fraction = (
                abs(dy) / width
                if width > 1e-6
                else 0.0
            )

            gate_skew = math.atan2(
                depth_difference,
                max(abs(dy), 1e-6)
            )

            if gate_skew > self.max_gate_skew_rad:
                continue

            center_x = 0.5 * (
                a.position.x + b.position.x
            )
            center_y = 0.5 * (
                a.position.y + b.position.y
            )

            if center_x < self.min_center_x:
                continue

            if center_x > self.max_center_x:
                continue

            object_confidence = 0.5 * (
                float(a.confidence)
                + float(b.confidence)
            )

            # Smoothly reduce confidence as the boat becomes
            # less square to the gate instead of abruptly
            # rejecting a valid gate.
            alignment_score = max(
                0.0,
                math.cos(gate_skew)
            )

            width_error = (
                abs(width - self.nominal_gate_width)
                / max(self.gate_width_tolerance, 1e-6)
            )
            width_score = math.exp(
                -0.5 * width_error * width_error
            )

            confidence = (
                0.45 * object_confidence
                + 0.25 * alignment_score
                + 0.20 * lateral_fraction
                + 0.10 * width_score
            )

            if best is not None:
                if confidence <= best['confidence']:
                    continue

            if a.position.y >= b.position.y:
                left = a.position
                right = b.position
            else:
                left = b.position
                right = a.position

            best = {
                'left': left,
                'right': right,
                'center_x': center_x,
                'center_y': center_y,
                'width': width,
                'confidence': confidence,
            }

        return best

    def update_gate_track(self, candidate):
        if self.tracked_center_x is None:
            self.start_track(candidate)
            return

        center_distance = math.hypot(
            candidate['center_x'] - self.tracked_center_x,
            candidate['center_y'] - self.tracked_center_y,
        )

        width_difference = abs(
            candidate['width'] - self.tracked_width
        )

        # A fixed body-frame association radius is too brittle
        # on a moving USV. Permit a modestly larger displacement
        # for gates farther from the LiDAR.
        track_range = math.hypot(
            self.tracked_center_x,
            self.tracked_center_y
        )

        association_limit = (
            self.center_association_distance
            + self.center_association_per_meter
            * track_range
        )

        same_gate = (
            center_distance <= association_limit
            and width_difference
            <= self.width_association_tolerance
        )

        if not same_gate:
            self.start_track(candidate)
            return

        a = self.track_alpha
        b = 1.0 - a

        self.tracked_center_x = (
            a * candidate['center_x']
            + b * self.tracked_center_x
        )
        self.tracked_center_y = (
            a * candidate['center_y']
            + b * self.tracked_center_y
        )
        self.tracked_width = (
            a * candidate['width']
            + b * self.tracked_width
        )
        self.tracked_confidence = (
            a * candidate['confidence']
            + b * self.tracked_confidence
        )

        self.tracked_left = candidate['left']
        self.tracked_right = candidate['right']

        self.hits += 1
        self.misses = 0

    def start_track(self, candidate):
        self.tracked_center_x = candidate['center_x']
        self.tracked_center_y = candidate['center_y']
        self.tracked_width = candidate['width']
        self.tracked_confidence = candidate['confidence']
        self.tracked_left = candidate['left']
        self.tracked_right = candidate['right']

        self.hits = 1
        self.misses = 0

    def reset_track(self):
        self.tracked_center_x = None
        self.tracked_center_y = None
        self.tracked_width = None
        self.tracked_confidence = 0.0
        self.tracked_left = None
        self.tracked_right = None

        self.hits = 0
        self.misses = 0
        self.last_measurement_header = None
        self.last_measurement_time = None

    def publish_empty_markers(self):
        markers = MarkerArray()

        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = 'base_link'
        marker.action = Marker.DELETEALL

        markers.markers.append(marker)
        self.marker_pub.publish(markers)

    def publish_gate_markers(self, gate):
        markers = MarkerArray()

        delete_all = Marker()
        delete_all.header = gate.header
        delete_all.action = Marker.DELETEALL
        markers.markers.append(delete_all)

        line = Marker()
        line.header = gate.header
        line.ns = 'confirmed_gate'
        line.id = 0
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.pose.orientation.w = 1.0

        line.scale.x = 0.08

        line.color.r = 0.0
        line.color.g = 1.0
        line.color.b = 0.0
        line.color.a = 0.9

        line.points.append(gate.left_marker)
        line.points.append(gate.right_marker)

        line.lifetime.sec = 0
        line.lifetime.nanosec = 300000000

        markers.markers.append(line)

        center = Marker()
        center.header = gate.header
        center.ns = 'confirmed_gate'
        center.id = 1
        center.type = Marker.SPHERE
        center.action = Marker.ADD

        center.pose.position = gate.center
        center.pose.orientation.w = 1.0

        center.scale.x = 0.30
        center.scale.y = 0.30
        center.scale.z = 0.30

        center.color.r = 0.0
        center.color.g = 1.0
        center.color.b = 1.0
        center.color.a = 1.0

        center.lifetime.sec = 0
        center.lifetime.nanosec = 300000000

        markers.markers.append(center)

        self.marker_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = GateDetector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
