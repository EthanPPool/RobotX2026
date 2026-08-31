#!/usr/bin/env python3
import math
from dataclasses import dataclass
from typing import List

import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray

from boat_interfaces.msg import DetectedObject, DetectedObjectArray


@dataclass
class Point2D:
    x: float
    y: float
    r: float


def distance(a: Point2D, b: Point2D) -> float:
    return math.hypot(a.x - b.x, a.y - b.y)


class BuoyDetector(Node):
    """Extract compact LiDAR clusters and publish them as buoy-like candidates.

    v0.2 is intentionally geometry-only. It does not claim color classification.
    """

    def __init__(self):
        super().__init__('buoy_detector')

        self.scan_topic = self.declare_parameter('scan_topic', '/perception/scan').value
        self.objects_topic = self.declare_parameter('objects_topic', '/perception/objects').value
        self.markers_topic = self.declare_parameter('markers_topic', '/perception/object_markers').value

        self.min_range = float(self.declare_parameter('min_range', 0.75).value)
        self.max_range = float(self.declare_parameter('max_range', 25.0).value)
        self.join_distance = float(self.declare_parameter('cluster_join_distance', 0.45).value)
        self.min_points = int(self.declare_parameter('min_cluster_points', 2).value)
        self.min_width = float(self.declare_parameter('min_cluster_width', 0.03).value)
        self.max_width = float(self.declare_parameter('max_cluster_width', 0.90).value)
        self.max_depth = float(self.declare_parameter('max_cluster_depth', 0.90).value)
        self.nominal_width = float(self.declare_parameter('nominal_buoy_width', 0.35).value)

        self.objects_pub = self.create_publisher(DetectedObjectArray, self.objects_topic, 10)
        self.marker_pub = self.create_publisher(MarkerArray, self.markers_topic, 10)
        self.scan_sub = self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, qos_profile_sensor_data)

        self.get_logger().info(f'Buoy detector listening on {self.scan_topic}')

    def scan_callback(self, scan: LaserScan) -> None:
        clusters: List[List[Point2D]] = []
        current: List[Point2D] = []

        for i, rng in enumerate(scan.ranges):
            if not math.isfinite(rng) or rng < self.min_range or rng > self.max_range:
                if current:
                    clusters.append(current)
                    current = []
                continue

            angle = scan.angle_min + i * scan.angle_increment
            point = Point2D(rng * math.cos(angle), rng * math.sin(angle), rng)

            if current and distance(current[-1], point) > self.join_distance:
                clusters.append(current)
                current = []

            current.append(point)

        if current:
            clusters.append(current)

        output = DetectedObjectArray()
        output.header = scan.header
        output.objects = []

        marker_array = MarkerArray()
        marker_id = 0

        for cluster in clusters:
            if len(cluster) < self.min_points:
                continue

            xs = [p.x for p in cluster]
            ys = [p.y for p in cluster]
            width = math.hypot(cluster[-1].x - cluster[0].x, cluster[-1].y - cluster[0].y)
            depth = max(p.r for p in cluster) - min(p.r for p in cluster)

            if width < self.min_width or width > self.max_width or depth > self.max_depth:
                continue

            cx = sum(xs) / len(xs)
            cy = sum(ys) / len(ys)
            if cx <= 0.0:
                continue

            width_error = abs(width - self.nominal_width)
            width_scale = max(self.nominal_width, 0.05)
            confidence = max(0.05, min(1.0, 1.0 - 0.65 * width_error / width_scale))

            obj = DetectedObject()
            obj.id = marker_id
            obj.object_type = DetectedObject.TYPE_BUOY
            obj.color = DetectedObject.COLOR_UNKNOWN
            obj.position.x = cx
            obj.position.y = cy
            obj.position.z = 0.0
            obj.size.x = width
            obj.size.y = max(depth, 0.05)
            obj.size.z = 0.0
            obj.confidence = float(confidence)
            output.objects.append(obj)

            marker = Marker()
            marker.header = scan.header
            marker.ns = 'buoy_candidates'
            marker.id = marker_id
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = cx
            marker.pose.position.y = cy
            marker.pose.position.z = 0.25
            marker.pose.orientation.w = 1.0
            marker.scale.x = max(width, 0.15)
            marker.scale.y = max(width, 0.15)
            marker.scale.z = 0.50
            marker.color.r = 1.0
            marker.color.g = 0.55
            marker.color.b = 0.0
            marker.color.a = 0.85
            marker.lifetime.sec = 0
            marker.lifetime.nanosec = 300_000_000
            marker_array.markers.append(marker)
            marker_id += 1

        self.objects_pub.publish(output)
        self.marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = BuoyDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
