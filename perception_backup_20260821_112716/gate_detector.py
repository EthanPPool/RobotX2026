#!/usr/bin/env python3
import math
from itertools import combinations

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray

from boat_interfaces.msg import DetectedObject, DetectedObjectArray, Gate


class GateDetector(Node):
    """Pair buoy-like candidates into the most plausible forward gate."""

    def __init__(self):
        super().__init__('gate_detector')

        self.objects_topic = self.declare_parameter('objects_topic', '/perception/objects').value
        self.gate_topic = self.declare_parameter('gate_topic', '/perception/gate').value
        self.markers_topic = self.declare_parameter('markers_topic', '/perception/gate_markers').value

        self.width_min = float(self.declare_parameter('gate_width_min', 1.65).value)
        self.width_max = float(self.declare_parameter('gate_width_max', 3.25).value)
        self.depth_tolerance = float(self.declare_parameter('gate_depth_tolerance', 1.20).value)
        self.min_gate_x = float(self.declare_parameter('min_gate_x', 0.60).value)
        self.max_gate_x = float(self.declare_parameter('max_gate_x', 25.0).value)
        self.centerline_weight = float(self.declare_parameter('centerline_weight', 1.50).value)
        self.depth_weight = float(self.declare_parameter('depth_weight', 0.50).value)

        self.gate_pub = self.create_publisher(Gate, self.gate_topic, 10)
        self.marker_pub = self.create_publisher(MarkerArray, self.markers_topic, 10)
        self.objects_sub = self.create_subscription(
            DetectedObjectArray, self.objects_topic, self.objects_callback, 10
        )

    def objects_callback(self, msg: DetectedObjectArray) -> None:
        candidates = [
            o for o in msg.objects
            if o.object_type in (DetectedObject.TYPE_UNKNOWN, DetectedObject.TYPE_BUOY)
            and self.min_gate_x <= o.position.x <= self.max_gate_x
        ]

        best = None
        best_score = float('inf')

        for a, b in combinations(candidates, 2):
            dx = a.position.x - b.position.x
            dy = a.position.y - b.position.y
            width = math.hypot(dx, dy)

            if not (self.width_min <= width <= self.width_max):
                continue
            if abs(dx) > self.depth_tolerance:
                continue

            cx = 0.5 * (a.position.x + b.position.x)
            cy = 0.5 * (a.position.y + b.position.y)
            if cx <= self.min_gate_x:
                continue

            # Prefer the nearest plausible gate centered in front of the vessel.
            score = cx + self.centerline_weight * abs(cy) + self.depth_weight * abs(dx)
            if score < best_score:
                best_score = score
                best = (a, b, width, cx, cy)

        if best is None:
            return

        a, b, width, cx, cy = best
        left, right = (a, b) if a.position.y >= b.position.y else (b, a)

        gate = Gate()
        gate.header = msg.header
        gate.left_marker = left.position
        gate.right_marker = right.position
        gate.center.x = cx
        gate.center.y = cy
        gate.center.z = 0.0
        gate.width = float(width)

        width_mid = 0.5 * (self.width_min + self.width_max)
        width_halfspan = max(0.5 * (self.width_max - self.width_min), 0.01)
        width_conf = max(0.0, 1.0 - abs(width - width_mid) / width_halfspan)
        pair_conf = 0.5 * (float(left.confidence) + float(right.confidence))
        gate.confidence = float(max(0.0, min(1.0, 0.5 * width_conf + 0.5 * pair_conf)))

        self.gate_pub.publish(gate)
        self.publish_markers(gate)

    def publish_markers(self, gate: Gate) -> None:
        markers = MarkerArray()

        line = Marker()
        line.header = gate.header
        line.ns = 'gate'
        line.id = 0
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.scale.x = 0.08
        line.color.r = 0.0
        line.color.g = 1.0
        line.color.b = 1.0
        line.color.a = 0.9
        line.points = [gate.left_marker, gate.right_marker]
        line.lifetime.nanosec = 300_000_000
        markers.markers.append(line)

        center = Marker()
        center.header = gate.header
        center.ns = 'gate'
        center.id = 1
        center.type = Marker.SPHERE
        center.action = Marker.ADD
        center.pose.position = gate.center
        center.pose.position.z = 0.25
        center.pose.orientation.w = 1.0
        center.scale.x = 0.35
        center.scale.y = 0.35
        center.scale.z = 0.35
        center.color.r = 0.0
        center.color.g = 1.0
        center.color.b = 0.0
        center.color.a = 0.9
        center.lifetime.nanosec = 300_000_000
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
