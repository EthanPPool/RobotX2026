#!/usr/bin/env python3

import copy
import json
import math

import rclpy
from rclpy.node import Node

from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String

from boat_interfaces.msg import DetectedObjectArray, Gate, GateArray
from boat_perception.gate_candidates import find_pairs
from boat_perception.pose_history import stamp_seconds


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
                'min_buoy_confidence', 0.60
            ).value
        )

        self.min_gate_width = float(
            self.declare_parameter('min_gate_width', 1.83).value
        )
        self.max_gate_width = float(
            self.declare_parameter('max_gate_width', 3.05).value
        )
        self.nominal_gate_width = float(
            self.declare_parameter('nominal_gate_width', 2.44).value
        )
        self.gate_width_tolerance = float(
            self.declare_parameter(
                'gate_width_tolerance', 0.75
            ).value
        )

        # Accepted for older parameter files; max_gate_skew_deg now controls
        # orientation instead of these heading-dependent limits.
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
            self.declare_parameter('min_center_x', 0.75).value
        )
        self.max_center_x = float(
            self.declare_parameter('max_center_x', 10.0).value
        )

        self.min_gate_confidence = float(
            self.declare_parameter(
                'min_gate_confidence', 0.72
            ).value
        )

        self.confirm_hits = int(self.declare_parameter('confirm_hits', 2).value)
        self.gates_topic = self.declare_parameter('gates_topic', '/perception/gates').value
        self.publish_rate = float(self.declare_parameter('publish_rate', 10.0).value)
        self.hold_timeout = float(self.declare_parameter('hold_timeout', 1.8).value)
        self.pair_parameters = dict(
            min_confidence=self.min_buoy_confidence, min_width=self.min_gate_width,
            max_width=self.max_gate_width, nominal_width=self.nominal_gate_width,
            width_tolerance=self.gate_width_tolerance, min_x=self.min_center_x,
            max_x=self.max_center_x,
            max_skew_deg=float(self.declare_parameter('max_gate_skew_deg', 55.0).value),
            max_buoys=int(self.declare_parameter('max_buoys', 64).value),
            max_candidates=int(self.declare_parameter('max_gate_candidates', 64).value),
            require_red_green=bool(self.declare_parameter('require_red_green', False).value),
            min_pair_confidence=self.min_gate_confidence,
        )
        p = self.pair_parameters
        if (not all(math.isfinite(v) for v in p.values()) or
                not 0 < p['min_width'] <= p['max_width'] or p['width_tolerance'] <= 0 or
                not 0 <= p['min_x'] < p['max_x'] or not 0 < p['max_skew_deg'] < 90 or
                not 0 <= p['min_confidence'] <= 1 or not 0 <= self.min_gate_confidence <= 1 or
                not 2 <= p['max_buoys'] <= 256 or not 1 <= p['max_candidates'] <= 256 or
                self.confirm_hits < 1 or not 0 < self.publish_rate <= 100 or
                not math.isfinite(self.hold_timeout) or self.hold_timeout <= 0):
            raise ValueError('Invalid gate confirmation, geometry or workload parameters')
        self.pair_hits = {}
        self.last_input_stamp = None
        self.last_measurement_time = None
        self.confirmed = None
        self.gates_pub = self.create_publisher(GateArray, self.gates_topic, 10)
        self.diagnostics_pub = self.create_publisher(String, '/perception/gate_diagnostics', 10)
        self.create_timer(1.0 / self.publish_rate, self.publish_tracked_gate)

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

        self.get_logger().info(
            'Robust gate detector started: '
            f'width={self.min_gate_width:.1f}-'
            f'{self.max_gate_width:.1f} m, '
            f'confirm_hits={self.confirm_hits}'
        )

    def objects_callback(self, msg):
        stamp = stamp_seconds(msg.header.stamp)
        if msg.header.frame_id != 'base_link' or stamp <= 0:
            self.confirmed = None
            self.pair_hits.clear()
            return
        if self.last_input_stamp is not None and stamp <= self.last_input_stamp:
            return  # Republishing a scan is not another confirmation hit.
        self.last_input_stamp = stamp
        candidates = find_pairs(msg.objects, **self.pair_parameters)
        hits, confirmed = {}, []
        for candidate in candidates:
            key = candidate['key']
            hits[key] = self.pair_hits.get(key, 0) + 1
            if hits[key] < self.confirm_hits or candidate['confidence'] < self.min_gate_confidence:
                continue
            gate = Gate()
            gate.header = copy.deepcopy(msg.header)
            gate.left_marker, gate.right_marker = candidate['left'], candidate['right']
            gate.center.x, gate.center.y = candidate['center_x'], candidate['center_y']
            gate.width, gate.confidence = candidate['width'], candidate['confidence']
            confirmed.append(gate)
        self.pair_hits = hits
        output = GateArray()
        output.header = copy.deepcopy(msg.header)
        output.gates = confirmed
        self.confirmed = output
        self.last_measurement_time = self.get_clock().now()
        info = String()
        info.data = json.dumps(dict(input_objects=len(msg.objects),
            plausible_pairs=len(candidates), confirmed_pairs=len(confirmed),
            max_buoys=self.pair_parameters['max_buoys'],
            max_candidates=self.pair_parameters['max_candidates']))
        self.diagnostics_pub.publish(info)

    def publish_tracked_gate(self):
        age = (None if self.last_measurement_time is None else
               (self.get_clock().now() - self.last_measurement_time).nanoseconds / 1e9)
        if self.confirmed is None or age is None or age < 0 or age > self.hold_timeout:
            empty = GateArray()
            empty.header.stamp = self.get_clock().now().to_msg()
            empty.header.frame_id = 'base_link'
            self.gates_pub.publish(empty)
            self.publish_empty_markers(empty.header)
            self.pair_hits.clear()
            return
        self.gates_pub.publish(self.confirmed)
        if self.confirmed.gates:
            # Compatibility output. The mission consumes the full array.
            nearest = self.confirmed.gates[0]
            self.gate_pub.publish(nearest)
            self.publish_gate_markers(nearest)
        else:
            self.publish_empty_markers(self.confirmed.header)

    def publish_empty_markers(self, header):
        markers = MarkerArray()

        marker = Marker()
        marker.header = header
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
