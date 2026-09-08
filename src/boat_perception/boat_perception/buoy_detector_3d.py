#!/usr/bin/env python3

import math
from dataclasses import dataclass

import numpy as np
from scipy.spatial import cKDTree

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from boat_interfaces.msg import DetectedObject, DetectedObjectArray


@dataclass
class Track3D:
    track_id: int
    x: float
    y: float
    z: float
    size_x: float
    size_y: float
    size_z: float
    confidence: float
    step_score: float
    step_evidence: int
    hits: int = 1
    misses: int = 0


class BuoyDetector3D(Node):
    """Conservative 3D buoy candidate detector.

    The node deliberately publishes on separate test topics by default so it
    can run beside the validated 2D detector.  A candidate must be a compact,
    upright 3D cluster with dimensions compatible with the RobotX buoys and
    must persist over multiple cloud frames before it is published.
    """

    def __init__(self):
        super().__init__('buoy_detector_3d')

        self.cloud_topic = self.declare_parameter(
            'cloud_topic', '/unilidar/cloud'
        ).value
        self.objects_topic = self.declare_parameter(
            'objects_topic', '/perception/objects_3d'
        ).value
        self.markers_topic = self.declare_parameter(
            'markers_topic', '/perception/object_markers_3d'
        ).value
        self.target_frame = self.declare_parameter(
            'target_frame', 'base_link'
        ).value
        self.transform_timeout = float(
            self.declare_parameter('transform_timeout', 0.08).value
        )

        # Search region in the leveled boat body frame.  The z limits are
        # intentionally broad for the first test because the exact waterline
        # relative to base_link still needs to be measured.
        self.min_range = float(
            self.declare_parameter('min_range', 0.75).value
        )
        self.max_range = float(
            self.declare_parameter('max_range', 10.0).value
        )
        self.min_forward_x = float(
            self.declare_parameter('min_forward_x', 0.60).value
        )
        self.max_lateral = float(
            self.declare_parameter('max_lateral', 5.0).value
        )
        self.max_bearing_deg = float(
            self.declare_parameter('max_bearing_deg', 70.0).value
        )
        self.max_bearing_rad = math.radians(self.max_bearing_deg)
        self.min_z = float(
            self.declare_parameter('min_z', -1.00).value
        )
        self.max_z = float(
            self.declare_parameter('max_z', 1.50).value
        )

        # Downsampling and range-adaptive footprint clustering.  The Unitree
        # scan pattern can leave vertical gaps larger than the neighbor
        # tolerance, splitting one tall buoy into several short horizontal
        # bands.  Neighboring returns are therefore connected in projected
        # XY space after the floor/waterline height crop.  The later height,
        # verticality, depth, profile, and temporal checks still operate on
        # the complete 3D points and reject non-buoy columns.
        self.voxel_size = float(
            self.declare_parameter('voxel_size', 0.03).value
        )
        self.cluster_tolerance_base = float(
            self.declare_parameter(
                'cluster_tolerance_base', 0.10
            ).value
        )
        self.cluster_tolerance_per_meter = float(
            self.declare_parameter(
                'cluster_tolerance_per_meter', 0.008
            ).value
        )
        self.min_cluster_points = int(
            self.declare_parameter('min_cluster_points', 6).value
        )
        self.good_cluster_points = int(
            self.declare_parameter('good_cluster_points', 18).value
        )
        self.max_cluster_points = int(
            self.declare_parameter('max_cluster_points', 1500).value
        )

        # The physical buoys are approximately 0.254-0.305 m in diameter
        # and 0.889-1.016 m exposed above water.  Hard limits are wider than
        # those measurements because LiDAR sees only part of the surface.
        self.min_diameter = float(
            self.declare_parameter('min_diameter', 0.12).value
        )
        self.max_diameter = float(
            self.declare_parameter('max_diameter', 0.65).value
        )
        self.nominal_diameter = float(
            self.declare_parameter('nominal_diameter', 0.28).value
        )
        self.diameter_tolerance = float(
            self.declare_parameter('diameter_tolerance', 0.16).value
        )

        self.min_object_height = float(
            self.declare_parameter('min_object_height', 0.35).value
        )
        self.max_object_height = float(
            self.declare_parameter('max_object_height', 1.40).value
        )
        self.nominal_object_height = float(
            self.declare_parameter('nominal_object_height', 0.95).value
        )
        self.height_tolerance = float(
            self.declare_parameter('height_tolerance', 0.45).value
        )
        self.max_radial_depth = float(
            self.declare_parameter('max_radial_depth', 0.65).value
        )
        self.min_height_to_diameter = float(
            self.declare_parameter(
                'min_height_to_diameter', 1.0
            ).value
        )
        self.min_verticality = float(
            self.declare_parameter('min_verticality', 0.55).value
        )
        self.good_verticality = float(
            self.declare_parameter('good_verticality', 0.85).value
        )
        self.min_candidate_confidence = float(
            self.declare_parameter(
                'min_candidate_confidence', 0.55
            ).value
        )

        # Distinctive buoy profile measured from the real 2.5 m capture:
        # approximately 0.40 m lower disk, 0.26 m upper cylinder, and a
        # lower/upper width ratio near 1.55.  The signature is accumulated
        # temporally because the rotating LiDAR does not illuminate the
        # lower disk completely in every individual frame.
        self.require_step_signature = bool(
            self.declare_parameter(
                'require_step_signature', True
            ).value
        )
        self.step_lower_start_fraction = float(
            self.declare_parameter(
                'step_lower_start_fraction', 0.02
            ).value
        )
        self.step_lower_end_fraction = float(
            self.declare_parameter(
                'step_lower_end_fraction', 0.28
            ).value
        )
        self.step_upper_start_fraction = float(
            self.declare_parameter(
                'step_upper_start_fraction', 0.38
            ).value
        )
        self.step_upper_end_fraction = float(
            self.declare_parameter(
                'step_upper_end_fraction', 0.88
            ).value
        )
        self.step_min_points_per_band = int(
            self.declare_parameter(
                'step_min_points_per_band', 6
            ).value
        )
        self.step_min_upper_width = float(
            self.declare_parameter(
                'step_min_upper_width', 0.16
            ).value
        )
        self.step_max_upper_width = float(
            self.declare_parameter(
                'step_max_upper_width', 0.36
            ).value
        )
        self.step_min_lower_width = float(
            self.declare_parameter(
                'step_min_lower_width', 0.30
            ).value
        )
        self.step_max_lower_width = float(
            self.declare_parameter(
                'step_max_lower_width', 0.60
            ).value
        )
        self.step_min_ratio = float(
            self.declare_parameter('step_min_ratio', 1.15).value
        )
        self.step_good_ratio = float(
            self.declare_parameter('step_good_ratio', 1.45).value
        )
        self.step_confirm_hits = int(
            self.declare_parameter('step_confirm_hits', 2).value
        )
        self.step_evidence_limit = int(
            self.declare_parameter('step_evidence_limit', 4).value
        )

        # Temporal confirmation.  Tracks live in base_link, so the
        # association allowance grows modestly with target range.
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
            self.declare_parameter('association_distance', 0.45).value
        )
        self.association_distance_per_meter = float(
            self.declare_parameter(
                'association_distance_per_meter', 0.03
            ).value
        )
        self.track_alpha = float(
            self.declare_parameter('track_alpha', 0.60).value
        )
        self.diagnostic_period = float(
            self.declare_parameter('diagnostic_period', 2.0).value
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.next_track_id = 0
        self.tracks = {}
        self.last_diagnostic_ns = 0

        self.objects_pub = self.create_publisher(
            DetectedObjectArray,
            self.objects_topic,
            10,
        )
        self.marker_pub = self.create_publisher(
            MarkerArray,
            self.markers_topic,
            10,
        )
        self.cloud_sub = self.create_subscription(
            PointCloud2,
            self.cloud_topic,
            self.cloud_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            '3D buoy detector started in side-by-side mode: '
            f'{self.cloud_topic} -> {self.objects_topic}; '
            f'diameter={self.min_diameter:.2f}-'
            f'{self.max_diameter:.2f} m; '
            f'height={self.min_object_height:.2f}-'
            f'{self.max_object_height:.2f} m'
        )

    def cloud_callback(self, cloud: PointCloud2):
        points = self.read_xyz(cloud)
        input_count = len(points)

        if input_count == 0:
            self.publish_empty(cloud)
            return

        points = self.transform_points(points, cloud)
        if points is None:
            return

        points = self.crop_roi(points)
        roi_count = len(points)

        if self.voxel_size > 0.0 and roi_count:
            points = self.voxel_downsample(points, self.voxel_size)

        clusters = self.build_clusters(points)

        detections = []
        for indices in clusters:
            candidate = self.evaluate_cluster(points[indices])
            if candidate is not None:
                detections.append(candidate)

        step_matches = sum(
            1 for detection in detections
            if detection['step_match']
        )

        self.update_tracks(detections)
        output = self.build_output(cloud)
        self.objects_pub.publish(output)
        self.publish_markers(output)
        self.publish_diagnostics(
            input_count,
            roi_count,
            len(points),
            len(clusters),
            len(detections),
            step_matches,
            len(output.objects),
        )

    def read_xyz(self, cloud):
        try:
            raw = point_cloud2.read_points_numpy(
                cloud,
                field_names=('x', 'y', 'z'),
                skip_nans=True,
            )
        except (AssertionError, ValueError) as exc:
            self.get_logger().error(
                f'Unable to decode PointCloud2 xyz fields: {exc}'
            )
            return np.empty((0, 3), dtype=np.float64)

        points = np.asarray(raw)

        # read_points_numpy normally returns an unstructured N x 3 array,
        # but retain compatibility with structured arrays as a safeguard.
        if points.dtype.names:
            points = np.column_stack(
                (points['x'], points['y'], points['z'])
            )

        points = np.asarray(points, dtype=np.float64).reshape((-1, 3))
        return points[np.isfinite(points).all(axis=1)]

    def transform_points(self, points, cloud):
        source_frame = cloud.header.frame_id

        if not source_frame:
            self.get_logger().warning('Point cloud has no frame_id')
            return None

        if source_frame == self.target_frame:
            return points

        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                source_frame,
                Time.from_msg(cloud.header.stamp),
                timeout=Duration(seconds=self.transform_timeout),
            )
        except TransformException as exc:
            self.get_logger().warning(
                f'Waiting for {source_frame} -> '
                f'{self.target_frame} transform: {exc}',
                throttle_duration_sec=2.0,
            )
            return None

        translation = transform.transform.translation
        rotation = transform.transform.rotation

        matrix = self.quaternion_matrix(
            rotation.x,
            rotation.y,
            rotation.z,
            rotation.w,
        )
        offset = np.array(
            [translation.x, translation.y, translation.z],
            dtype=np.float64,
        )

        return points @ matrix.T + offset

    @staticmethod
    def quaternion_matrix(x, y, z, w):
        norm = x * x + y * y + z * z + w * w
        if norm <= 1e-12:
            return np.eye(3, dtype=np.float64)

        scale = 2.0 / norm
        xx = x * x * scale
        yy = y * y * scale
        zz = z * z * scale
        xy = x * y * scale
        xz = x * z * scale
        yz = y * z * scale
        wx = w * x * scale
        wy = w * y * scale
        wz = w * z * scale

        return np.array(
            [
                [1.0 - yy - zz, xy - wz, xz + wy],
                [xy + wz, 1.0 - xx - zz, yz - wx],
                [xz - wy, yz + wx, 1.0 - xx - yy],
            ],
            dtype=np.float64,
        )

    def crop_roi(self, points):
        x = points[:, 0]
        y = points[:, 1]
        z = points[:, 2]
        ranges = np.hypot(x, y)
        bearings = np.abs(np.arctan2(y, x))

        keep = (
            (ranges >= self.min_range)
            & (ranges <= self.max_range)
            & (x >= self.min_forward_x)
            & (np.abs(y) <= self.max_lateral)
            & (bearings <= self.max_bearing_rad)
            & (z >= self.min_z)
            & (z <= self.max_z)
        )
        return points[keep]

    @staticmethod
    def voxel_downsample(points, voxel_size):
        keys = np.floor(points / voxel_size).astype(np.int64)
        _, inverse = np.unique(keys, axis=0, return_inverse=True)

        counts = np.bincount(inverse)
        sums = np.zeros((len(counts), 3), dtype=np.float64)
        np.add.at(sums, inverse, points)
        return sums / counts[:, None]

    def build_clusters(self, points):
        count = len(points)
        if count < self.min_cluster_points:
            return []

        ranges = np.hypot(points[:, 0], points[:, 1])
        max_tolerance = (
            self.cluster_tolerance_base
            + self.cluster_tolerance_per_meter * self.max_range
        )

        tree = cKDTree(points[:, :2])
        pairs = tree.query_pairs(max_tolerance, output_type='ndarray')

        parent = np.arange(count, dtype=np.int32)
        rank = np.zeros(count, dtype=np.uint8)

        def find(index):
            while parent[index] != index:
                parent[index] = parent[parent[index]]
                index = parent[index]
            return index

        def union(first, second):
            first_root = find(first)
            second_root = find(second)

            if first_root == second_root:
                return

            if rank[first_root] < rank[second_root]:
                first_root, second_root = second_root, first_root

            parent[second_root] = first_root
            if rank[first_root] == rank[second_root]:
                rank[first_root] += 1

        for first, second in pairs:
            allowed = (
                self.cluster_tolerance_base
                + self.cluster_tolerance_per_meter
                * min(ranges[first], ranges[second])
            )

            horizontal_distance = np.linalg.norm(
                points[first, :2] - points[second, :2]
            )

            if horizontal_distance <= allowed:
                union(int(first), int(second))

        groups = {}
        for index in range(count):
            root = find(index)
            groups.setdefault(root, []).append(index)

        clusters = []
        for indices in groups.values():
            size = len(indices)
            if size < self.min_cluster_points:
                continue
            if size > self.max_cluster_points:
                continue
            clusters.append(np.asarray(indices, dtype=np.int32))

        return clusters

    def evaluate_cluster(self, points):
        count = len(points)
        minimum = points.min(axis=0)
        maximum = points.max(axis=0)
        center = points.mean(axis=0)
        dimensions = maximum - minimum

        size_x = float(dimensions[0])
        size_y = float(dimensions[1])
        size_z = float(dimensions[2])
        diameter = max(size_x, size_y)

        ranges = np.hypot(points[:, 0], points[:, 1])
        radial_depth = float(np.ptp(ranges))

        if diameter < self.min_diameter:
            return None
        if diameter > self.max_diameter:
            return None
        if size_z < self.min_object_height:
            return None
        if size_z > self.max_object_height:
            return None
        if radial_depth > self.max_radial_depth:
            return None

        aspect = size_z / max(diameter, 1e-6)
        if aspect < self.min_height_to_diameter:
            return None

        centered = points - center
        covariance = centered.T @ centered / max(float(count), 1.0)
        eigenvalues, eigenvectors = np.linalg.eigh(covariance)
        major_axis = eigenvectors[:, int(np.argmax(eigenvalues))]
        verticality = abs(float(major_axis[2]))

        if verticality < self.min_verticality:
            return None

        diameter_score = self.gaussian_score(
            diameter,
            self.nominal_diameter,
            self.diameter_tolerance,
        )
        height_score = self.gaussian_score(
            size_z,
            self.nominal_object_height,
            self.height_tolerance,
        )
        verticality_score = min(
            1.0,
            verticality / max(self.good_verticality, 1e-6),
        )
        point_score = min(
            1.0,
            count / max(float(self.good_cluster_points), 1.0),
        )

        # A partial cylindrical return is commonly narrower radially than
        # laterally.  This score is intentionally weak and is not a hard
        # rejection criterion.
        horizontal_minor = min(size_x, size_y)
        horizontal_ratio = horizontal_minor / max(diameter, 1e-6)
        compactness_score = min(1.0, horizontal_ratio / 0.35)

        base_confidence = (
            0.25 * diameter_score
            + 0.25 * height_score
            + 0.20 * verticality_score
            + 0.20 * point_score
            + 0.10 * compactness_score
        )

        step_profile = self.evaluate_step_profile(
            points,
            center,
        )

        # The profile can raise confidence but cannot rescue a cluster that
        # completely fails the basic upright-object geometry.  Publication
        # separately requires accumulated step evidence when enabled.
        confidence = (
            0.80 * base_confidence
            + 0.20 * step_profile['score']
        )

        if confidence < self.min_candidate_confidence:
            return None

        return {
            'x': float(center[0]),
            'y': float(center[1]),
            'z': float(center[2]),
            'size_x': size_x,
            'size_y': size_y,
            'size_z': size_z,
            'confidence': float(confidence),
            'step_score': step_profile['score'],
            'step_match': step_profile['match'],
        }

    def evaluate_step_profile(self, points, center):
        """Measure the lower-disk/upper-cylinder width transition.

        Width is measured tangentially to the LiDAR viewing direction so the
        calculation works for buoys anywhere in the forward field of view.
        Percentiles suppress isolated spray, floor, and multipath points.
        """
        z_low = float(np.percentile(points[:, 2], 2.0))
        z_high = float(np.percentile(points[:, 2], 98.0))
        height = z_high - z_low

        if height <= 1e-6:
            return {'score': 0.0, 'match': False}

        center_xy = np.asarray(center[:2], dtype=np.float64)
        target_range = float(np.linalg.norm(center_xy))
        if target_range <= 1e-6:
            return {'score': 0.0, 'match': False}

        radial = center_xy / target_range
        tangent = np.array([-radial[1], radial[0]], dtype=np.float64)
        tangential = points[:, :2] @ tangent

        lower_width = self.profile_band_width(
            points[:, 2],
            tangential,
            z_low + self.step_lower_start_fraction * height,
            z_low + self.step_lower_end_fraction * height,
        )
        upper_width = self.profile_band_width(
            points[:, 2],
            tangential,
            z_low + self.step_upper_start_fraction * height,
            z_low + self.step_upper_end_fraction * height,
        )

        if lower_width is None or upper_width is None:
            return {'score': 0.0, 'match': False}

        ratio = lower_width / max(upper_width, 1e-6)

        upper_match = (
            self.step_min_upper_width
            <= upper_width
            <= self.step_max_upper_width
        )
        lower_match = (
            self.step_min_lower_width
            <= lower_width
            <= self.step_max_lower_width
        )
        ratio_match = ratio >= self.step_min_ratio

        ratio_score = self.ramp_score(
            ratio,
            self.step_min_ratio,
            self.step_good_ratio,
        )
        upper_score = self.range_score(
            upper_width,
            self.step_min_upper_width,
            self.step_max_upper_width,
        )
        lower_score = self.range_score(
            lower_width,
            self.step_min_lower_width,
            self.step_max_lower_width,
        )

        score = (
            0.50 * ratio_score
            + 0.25 * upper_score
            + 0.25 * lower_score
        )

        return {
            'score': float(score),
            'match': bool(upper_match and lower_match and ratio_match),
        }

    def profile_band_width(
        self,
        z_values,
        tangential_values,
        minimum_z,
        maximum_z,
    ):
        mask = (z_values >= minimum_z) & (z_values <= maximum_z)
        values = tangential_values[mask]

        if len(values) < self.step_min_points_per_band:
            return None

        low, high = np.percentile(values, [5.0, 95.0])
        return float(high - low)

    @staticmethod
    def ramp_score(value, minimum, good):
        if value <= minimum:
            return 0.0
        if value >= good:
            return 1.0
        return (value - minimum) / max(good - minimum, 1e-6)

    @staticmethod
    def range_score(value, minimum, maximum):
        if value < minimum or value > maximum:
            return 0.0

        midpoint = 0.5 * (minimum + maximum)
        half_width = 0.5 * (maximum - minimum)
        return max(
            0.0,
            1.0 - abs(value - midpoint) / max(half_width, 1e-6),
        )

    @staticmethod
    def gaussian_score(value, nominal, tolerance):
        error = abs(value - nominal) / max(tolerance, 1e-6)
        return math.exp(-0.5 * error * error)

    def update_tracks(self, detections):
        unmatched = set(range(len(detections)))

        for track in list(self.tracks.values()):
            best_index = None
            track_range = math.hypot(track.x, track.y)
            best_distance = (
                self.association_distance
                + self.association_distance_per_meter * track_range
            )

            for index in unmatched:
                detection = detections[index]
                distance = math.hypot(
                    detection['x'] - track.x,
                    detection['y'] - track.y,
                )

                if distance < best_distance:
                    best_distance = distance
                    best_index = index

            if best_index is None:
                track.misses += 1
                continue

            detection = detections[best_index]
            unmatched.remove(best_index)

            alpha = self.track_alpha
            beta = 1.0 - alpha

            track.x = alpha * detection['x'] + beta * track.x
            track.y = alpha * detection['y'] + beta * track.y
            track.z = alpha * detection['z'] + beta * track.z
            track.size_x = (
                alpha * detection['size_x'] + beta * track.size_x
            )
            track.size_y = (
                alpha * detection['size_y'] + beta * track.size_y
            )
            track.size_z = (
                alpha * detection['size_z'] + beta * track.size_z
            )
            track.confidence = (
                alpha * detection['confidence']
                + beta * track.confidence
            )
            track.step_score = (
                alpha * detection['step_score']
                + beta * track.step_score
            )

            if detection['step_match']:
                track.step_evidence = min(
                    self.step_evidence_limit,
                    track.step_evidence + 1,
                )

            # A missing step in one frame is not negative evidence: the
            # rotating LiDAR often illuminates the upper cylinder without
            # sampling enough of the partially submerged disk.  Positive
            # evidence remains valid until normal track expiry removes the
            # object entirely.

            track.hits += 1
            track.misses = 0

        for index in unmatched:
            detection = detections[index]
            self.tracks[self.next_track_id] = Track3D(
                track_id=self.next_track_id,
                x=detection['x'],
                y=detection['y'],
                z=detection['z'],
                size_x=detection['size_x'],
                size_y=detection['size_y'],
                size_z=detection['size_z'],
                confidence=detection['confidence'],
                step_score=detection['step_score'],
                step_evidence=(1 if detection['step_match'] else 0),
            )
            self.next_track_id += 1

        stale = [
            track_id
            for track_id, track in self.tracks.items()
            if track.misses > self.max_track_misses
        ]
        for track_id in stale:
            del self.tracks[track_id]

    def build_output(self, cloud):
        output = DetectedObjectArray()
        output.header = cloud.header
        output.header.frame_id = self.target_frame

        for track in self.tracks.values():
            if track.hits < self.confirm_hits:
                continue
            if track.misses > self.publish_misses:
                continue
            if (
                self.require_step_signature
                and track.step_evidence < self.step_confirm_hits
            ):
                continue

            obj = DetectedObject()
            obj.id = int(track.track_id)
            obj.object_type = DetectedObject.TYPE_BUOY
            obj.color = DetectedObject.COLOR_UNKNOWN
            obj.position.x = float(track.x)
            obj.position.y = float(track.y)
            obj.position.z = float(track.z)
            obj.size.x = float(track.size_x)
            obj.size.y = float(track.size_y)
            obj.size.z = float(track.size_z)

            confirmation = min(
                1.0,
                track.hits / max(float(self.confirm_hits), 1.0),
            )
            obj.confidence = float(
                min(1.0, track.confidence * confirmation)
            )
            output.objects.append(obj)

        return output

    def publish_empty(self, cloud):
        output = DetectedObjectArray()
        output.header = cloud.header
        output.header.frame_id = self.target_frame
        self.objects_pub.publish(output)
        self.publish_markers(output)

    def publish_markers(self, objects):
        markers = MarkerArray()

        delete_all = Marker()
        delete_all.header = objects.header
        delete_all.action = Marker.DELETEALL
        markers.markers.append(delete_all)

        for obj in objects.objects:
            marker = Marker()
            marker.header = objects.header
            marker.ns = 'confirmed_buoys_3d'
            marker.id = int(obj.id)
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position = obj.position
            marker.pose.orientation.w = 1.0

            diameter = max(0.05, obj.size.x, obj.size.y)
            marker.scale.x = float(diameter)
            marker.scale.y = float(diameter)
            marker.scale.z = float(max(0.05, obj.size.z))

            marker.color.r = 0.1
            marker.color.g = 0.4
            marker.color.b = 1.0
            marker.color.a = 0.90
            marker.lifetime.sec = 0
            marker.lifetime.nanosec = 300000000
            markers.markers.append(marker)

        self.marker_pub.publish(markers)

    def publish_diagnostics(
        self,
        input_count,
        roi_count,
        downsampled_count,
        cluster_count,
        candidate_count,
        step_match_count,
        published_count,
    ):
        now_ns = self.get_clock().now().nanoseconds
        period_ns = int(max(0.1, self.diagnostic_period) * 1e9)

        if now_ns - self.last_diagnostic_ns < period_ns:
            return

        self.last_diagnostic_ns = now_ns
        self.get_logger().info(
            '3D pipeline: '
            f'input={input_count}, roi={roi_count}, '
            f'voxel={downsampled_count}, clusters={cluster_count}, '
            f'candidates={candidate_count}, '
            f'step_matches={step_match_count}, '
            f'published={published_count}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = BuoyDetector3D()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
