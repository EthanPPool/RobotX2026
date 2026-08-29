#!/usr/bin/env python3

import math
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray

from boat_interfaces.msg import DetectedObject, DetectedObjectArray


@dataclass
class Track:
    track_id: int
    x: float
    y: float
    size_x: float
    size_y: float
    confidence: float
    hits: int = 1
    misses: int = 0


class BuoyDetector(Node):
    """
    Conservative 2D buoy detector.

    Pipeline:
      LaserScan
        -> contiguous point clusters
        -> geometric candidate filtering
        -> temporal tracking
        -> confirmed buoy objects

    Only confirmed tracks are published as TYPE_BUOY.
    """

    def __init__(self):
        super().__init__('buoy_detector')

        self.scan_topic = self.declare_parameter(
            'scan_topic', '/perception/scan'
        ).value
        self.objects_topic = self.declare_parameter(
            'objects_topic', '/perception/objects'
        ).value
        self.markers_topic = self.declare_parameter(
            'markers_topic', '/perception/object_markers'
        ).value

        # Region of interest.
        self.min_range = float(
            self.declare_parameter('min_range', 0.70).value
        )
        self.max_range = float(
            self.declare_parameter('max_range', 15.0).value
        )
        self.min_forward_x = float(
            self.declare_parameter('min_forward_x', 0.50).value
        )
        self.max_lateral = float(
            self.declare_parameter('max_lateral', 8.0).value
        )
        self.max_bearing_deg = float(
            self.declare_parameter('max_bearing_deg', 55.0).value
        )
        self.max_bearing_rad = math.radians(self.max_bearing_deg)

        # Cluster segmentation.
        self.cluster_gap_base = float(
            self.declare_parameter('cluster_gap_base', 0.10).value
        )
        self.cluster_gap_per_meter = float(
            self.declare_parameter('cluster_gap_per_meter', 0.012).value
        )

        # Buoy geometry.
        self.min_cluster_points = int(
            self.declare_parameter('min_cluster_points', 3).value
        )
        self.good_cluster_points = int(
            self.declare_parameter('good_cluster_points', 7).value
        )

        self.min_width = float(
            self.declare_parameter('min_cluster_width', 0.08).value
        )
        self.max_width = float(
            self.declare_parameter('max_cluster_width', 0.75).value
        )
        self.max_depth = float(
            self.declare_parameter('max_cluster_depth', 0.55).value
        )
        self.max_radial_span = float(
            self.declare_parameter('max_radial_span', 0.45).value
        )

        self.nominal_width = float(
            self.declare_parameter('nominal_buoy_width', 0.35).value
        )
        self.size_tolerance = float(
            self.declare_parameter('size_tolerance', 0.30).value
        )

        # Shape / compactness.
        self.min_pca_ratio = float(
            self.declare_parameter('min_pca_ratio', 0.002).value
        )
        self.good_pca_ratio = float(
            self.declare_parameter('good_pca_ratio', 0.050).value
        )
        self.good_convexity = float(
            self.declare_parameter('good_convexity', 0.025).value
        )
        self.min_candidate_confidence = float(
            self.declare_parameter(
                'min_candidate_confidence', 0.30
            ).value
        )

        # Temporal tracking.
        self.confirm_hits = int(
            self.declare_parameter('confirm_hits', 3).value
        )
        self.max_track_misses = int(
            self.declare_parameter('max_track_misses', 3).value
        )
        self.publish_misses = int(
            self.declare_parameter('publish_misses', 0).value
        )
        self.association_distance = float(
            self.declare_parameter('association_distance', 0.55).value
        )
        self.track_alpha = float(
            self.declare_parameter('track_alpha', 0.65).value
        )

        self.next_track_id = 0
        self.tracks = {}

        self.objects_pub = self.create_publisher(
            DetectedObjectArray,
            self.objects_topic,
            10
        )
        self.marker_pub = self.create_publisher(
            MarkerArray,
            self.markers_topic,
            10
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_callback,
            qos_profile_sensor_data
        )

        self.get_logger().info(
            'Robust buoy detector started: '
            f'ROI x>{self.min_forward_x:.2f} m, '
            f'range={self.min_range:.2f}-{self.max_range:.1f} m, '
            f'confirm_hits={self.confirm_hits}'
        )

    def scan_callback(self, scan: LaserScan):
        clusters = self.build_clusters(scan)

        detections = []
        for cluster in clusters:
            candidate = self.evaluate_cluster(cluster)
            if candidate is not None:
                detections.append(candidate)

        self.update_tracks(detections)

        output = DetectedObjectArray()
        output.header = scan.header

        for track in self.tracks.values():
            if track.hits < self.confirm_hits:
                continue
            if track.misses > self.publish_misses:
                continue

            obj = DetectedObject()
            obj.id = int(track.track_id)
            obj.object_type = DetectedObject.TYPE_BUOY
            obj.color = DetectedObject.COLOR_UNKNOWN

            obj.position.x = float(track.x)
            obj.position.y = float(track.y)
            obj.position.z = 0.0

            obj.size.x = float(track.size_x)
            obj.size.y = float(track.size_y)
            obj.size.z = 0.0

            confirmation = min(
                1.0,
                track.hits / max(float(self.confirm_hits), 1.0)
            )
            obj.confidence = float(
                min(1.0, track.confidence * confirmation)
            )

            output.objects.append(obj)

        self.objects_pub.publish(output)
        self.publish_markers(output)

    def build_clusters(self, scan):
        clusters = []
        current = []
        previous = None

        angle = scan.angle_min

        for r in scan.ranges:
            valid = (
                math.isfinite(r)
                and r >= max(scan.range_min, self.min_range)
                and r <= min(scan.range_max, self.max_range)
            )

            if valid:
                x = r * math.cos(angle)
                y = r * math.sin(angle)

                bearing = abs(math.atan2(y, x))

                valid = (
                    x >= self.min_forward_x
                    and abs(y) <= self.max_lateral
                    and bearing <= self.max_bearing_rad
                )

            if not valid:
                if current:
                    clusters.append(current)
                    current = []

                previous = None
                angle += scan.angle_increment
                continue

            point = (x, y, r)

            if previous is not None:
                px, py, pr = previous

                point_gap = math.hypot(x - px, y - py)

                allowed_gap = (
                    self.cluster_gap_base
                    + self.cluster_gap_per_meter * min(r, pr)
                )

                if point_gap > allowed_gap:
                    if current:
                        clusters.append(current)
                    current = []

            current.append(point)
            previous = point

            angle += scan.angle_increment

        if current:
            clusters.append(current)

        return clusters

    def evaluate_cluster(self, cluster):
        n = len(cluster)

        if n < self.min_cluster_points:
            return None

        xs = [p[0] for p in cluster]
        ys = [p[1] for p in cluster]
        rs = [p[2] for p in cluster]

        x_mean = sum(xs) / n
        y_mean = sum(ys) / n

        depth = max(xs) - min(xs)
        width = max(ys) - min(ys)
        span = math.hypot(depth, width)
        radial_span = max(rs) - min(rs)

        # Hard geometry rejection.
        if span < self.min_width:
            return None

        if span > self.max_width:
            return None

        if depth > self.max_depth:
            return None

        if radial_span > self.max_radial_span:
            return None

        # 2D PCA compactness.
        cxx = sum((x - x_mean) ** 2 for x in xs) / n
        cyy = sum((y - y_mean) ** 2 for y in ys) / n
        cxy = sum(
            (x - x_mean) * (y - y_mean)
            for x, y in zip(xs, ys)
        ) / n

        trace = cxx + cyy
        disc = math.sqrt(
            max(0.0, (cxx - cyy) ** 2 + 4.0 * cxy * cxy)
        )

        lambda_major = 0.5 * (trace + disc)
        lambda_minor = 0.5 * (trace - disc)

        if lambda_major <= 1e-9:
            return None

        pca_ratio = max(
            0.0,
            lambda_minor / lambda_major
        )

        if pca_ratio < self.min_pca_ratio:
            return None

        # Convex surfaces tend to have their middle points slightly
        # closer to the lidar than their edges.
        edge_count = max(1, n // 4)

        edge_mean = (
            sum(rs[:edge_count])
            + sum(rs[-edge_count:])
        ) / (2.0 * edge_count)

        middle_start = n // 3
        middle_end = max(middle_start + 1, (2 * n) // 3)

        middle_values = rs[middle_start:middle_end]
        middle_mean = sum(middle_values) / len(middle_values)

        convexity = edge_mean - middle_mean

        # Confidence scoring.
        size_error = (
            abs(span - self.nominal_width)
            / max(self.size_tolerance, 1e-3)
        )
        size_score = math.exp(-0.5 * size_error * size_error)

        point_score = min(
            1.0,
            n / max(float(self.good_cluster_points), 1.0)
        )

        shape_score = min(
            1.0,
            pca_ratio / max(self.good_pca_ratio, 1e-6)
        )

        convexity_score = min(
            1.0,
            max(0.0, convexity)
            / max(self.good_convexity, 1e-6)
        )

        confidence = (
            0.35 * size_score
            + 0.25 * point_score
            + 0.25 * shape_score
            + 0.15 * convexity_score
        )

        if confidence < self.min_candidate_confidence:
            return None

        return {
            'x': x_mean,
            'y': y_mean,
            'size_x': depth,
            'size_y': width,
            'confidence': confidence,
        }

    def update_tracks(self, detections):
        unmatched = set(range(len(detections)))

        # Greedy nearest-neighbor association.
        for track in list(self.tracks.values()):
            best_idx = None
            best_distance = self.association_distance

            for idx in unmatched:
                detection = detections[idx]

                distance = math.hypot(
                    detection['x'] - track.x,
                    detection['y'] - track.y,
                )

                if distance < best_distance:
                    best_distance = distance
                    best_idx = idx

            if best_idx is None:
                track.misses += 1
                continue

            detection = detections[best_idx]
            unmatched.remove(best_idx)

            a = self.track_alpha
            b = 1.0 - a

            track.x = a * detection['x'] + b * track.x
            track.y = a * detection['y'] + b * track.y
            track.size_x = (
                a * detection['size_x']
                + b * track.size_x
            )
            track.size_y = (
                a * detection['size_y']
                + b * track.size_y
            )
            track.confidence = (
                a * detection['confidence']
                + b * track.confidence
            )

            track.hits += 1
            track.misses = 0

        # Create new tracks.
        for idx in unmatched:
            detection = detections[idx]

            track = Track(
                track_id=self.next_track_id,
                x=detection['x'],
                y=detection['y'],
                size_x=detection['size_x'],
                size_y=detection['size_y'],
                confidence=detection['confidence'],
            )

            self.tracks[self.next_track_id] = track
            self.next_track_id += 1

        # Remove stale tracks.
        stale = [
            track_id
            for track_id, track in self.tracks.items()
            if track.misses > self.max_track_misses
        ]

        for track_id in stale:
            del self.tracks[track_id]

    def publish_markers(self, objects):
        markers = MarkerArray()

        delete_all = Marker()
        delete_all.action = Marker.DELETEALL
        markers.markers.append(delete_all)

        for obj in objects.objects:
            marker = Marker()
            marker.header = objects.header
            marker.ns = 'confirmed_buoys'
            marker.id = int(obj.id)
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD

            marker.pose.position.x = obj.position.x
            marker.pose.position.y = obj.position.y
            marker.pose.position.z = 0.35
            marker.pose.orientation.w = 1.0

            diameter = max(
                0.15,
                obj.size.x,
                obj.size.y
            )

            marker.scale.x = diameter
            marker.scale.y = diameter
            marker.scale.z = 0.70

            marker.color.r = 1.0
            marker.color.g = 0.65
            marker.color.b = 0.0
            marker.color.a = 0.85

            marker.lifetime.sec = 0
            marker.lifetime.nanosec = 300000000

            markers.markers.append(marker)

        self.marker_pub.publish(markers)


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
