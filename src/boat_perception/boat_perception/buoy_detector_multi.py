#!/usr/bin/env python3

# =============================================================================
# MULTI-TYPE 3D BUOY DETECTOR - GENERIC BUOY + SUBTYPE VERSION
# =============================================================================
# This ROS 2 node detects physical LiDAR clusters first, tracks those clusters
# over time, and only then decides whether a tracked object matches one of the
# globally-defined buoy types in buoy_types.yaml.
#
# This revision separates two decisions:
#   1. Is the tracked physical object a buoy at all?
#   2. If it is a buoy, which configured subtype best matches it?
#
# A confirmed generic buoy remains published through short LiDAR dropouts and
# temporary subtype ambiguity. Subtype uncertainty therefore cannot make a
# stable gate marker disappear from /perception/objects.
#
# Why this version exists:
#   The first multi-type detector classified each individual LiDAR frame before
#   tracking it. A real buoy could therefore disappear whenever one partial
#   scan failed a single geometry threshold. This revision keeps broad physical
#   clusters alive long enough to measure what the LiDAR is actually seeing.
#
# Processing pipeline:
#   PointCloud2
#       -> transform to base_link
#       -> crop search ROI
#       -> voxel downsample
#       -> XY clustering
#       -> extract raw geometric features for EVERY cluster
#       -> score EVERY cluster against EVERY buoy model for diagnostics
#       -> track broad physical clusters across frames
#       -> re-score the SMOOTHED TRACK against every buoy model
#       -> publish confirmed buoy tracks
#
# Important behavior:
#   * Most model limits are SOFT scoring limits, not instant rejection limits.
#   * Only explicit hard_min / hard_max limits can immediately reject a model.
#   * Rejected clusters are still reported on the diagnostics topic.
#   * Ambiguous buoy subtypes can be published as "unknown_buoy" while still
#     remaining TYPE_BUOY for compatibility with the existing gate detector.
# =============================================================================

import json
import math
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import yaml
from scipy.spatial import cKDTree

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from boat_interfaces.msg import DetectedObject, DetectedObjectArray


# =============================================================================
# DATA CLASSES
# =============================================================================

# -----------------------------------------------------------------------------
# One buoy type loaded from the global YAML configuration.
# -----------------------------------------------------------------------------
@dataclass
class BuoyTypeSpec:
    name: str
    enabled: bool
    min_points: int
    good_points: int
    min_score: float
    features: dict


# -----------------------------------------------------------------------------
# Result of evaluating one feature vector against one buoy type.
# -----------------------------------------------------------------------------
@dataclass
class ModelEvaluation:
    model_name: str
    score: float
    accepted: bool
    reasons: list = field(default_factory=list)
    feature_scores: dict = field(default_factory=dict)


# -----------------------------------------------------------------------------
# One physical cluster measured in the current LiDAR frame.
# This exists whether or not the cluster looks like a buoy.
# -----------------------------------------------------------------------------
@dataclass
class Candidate:
    frame_cluster_id: int
    features: dict
    evaluations: dict
    track_id: int = 0

    @property
    def x(self):
        return float(self.features['x'])

    @property
    def y(self):
        return float(self.features['y'])

    @property
    def z(self):
        return float(self.features['z'])


# -----------------------------------------------------------------------------
# One broad physical object tracked across multiple LiDAR frames.
# Classification is performed from these smoothed track features.
# -----------------------------------------------------------------------------
@dataclass
class Track:
    track_id: int
    features: dict
    hits: int = 1
    misses: int = 0

    # Generic buoy state is intentionally independent of subtype.
    # Once confirmed, a buoy can survive a few weak/missing scans without
    # disappearing from the gate detector input topic.
    buoy_hits: int = 0
    buoy_bad_frames: int = 0
    confirmed_buoy: bool = False
    generic_confidence: float = 0.0

    @property
    def x(self):
        return float(self.features.get('x', 0.0))

    @property
    def y(self):
        return float(self.features.get('y', 0.0))

    @property
    def z(self):
        return float(self.features.get('z', 0.0))


# =============================================================================
# MAIN ROS 2 NODE
# =============================================================================
class MultiTypeBuoyDetector(Node):

    # -------------------------------------------------------------------------
    # NODE STARTUP
    # Loads detector parameters, global buoy definitions, ROS publishers,
    # TF, and temporal tracking state.
    # -------------------------------------------------------------------------
    def __init__(self):
        super().__init__('buoy_detector_multi')

        # ---------------------------------------------------------------------
        # ROS topics and reference frame.
        # ---------------------------------------------------------------------
        self.cloud_topic = self.declare_parameter(
            'cloud_topic', '/unilidar/cloud'
        ).value

        self.objects_topic = self.declare_parameter(
            'objects_topic', '/perception/objects_multi'
        ).value

        self.markers_topic = self.declare_parameter(
            'markers_topic', '/perception/object_markers_multi'
        ).value

        self.classification_topic = self.declare_parameter(
            'classification_topic', '/perception/buoy_classifications'
        ).value

        self.diagnostics_topic = self.declare_parameter(
            'diagnostics_topic', '/perception/buoy_cluster_diagnostics'
        ).value

        self.target_frame = self.declare_parameter(
            'target_frame', 'base_link'
        ).value

        self.transform_timeout = float(
            self.declare_parameter('transform_timeout', 0.08).value
        )

        # ---------------------------------------------------------------------
        # Global buoy-type configuration file.
        # ---------------------------------------------------------------------
        default_type_file = str(
            Path(get_package_share_directory('boat_perception'))
            / 'config'
            / 'buoy_types.yaml'
        )

        self.buoy_types_file = self.declare_parameter(
            'buoy_types_file', default_type_file
        ).value

        (
            self.generic_buoy_spec,
            self.buoy_types,
        ) = self.load_buoy_types(
            self.buoy_types_file
        )

        # ---------------------------------------------------------------------
        # Search region.
        # Keep these broad in code. During calibration we can override them on
        # the command line, for example max_range:=3.0.
        # ---------------------------------------------------------------------
        self.min_range = float(
            self.declare_parameter('min_range', 0.75).value
        )

        self.max_range = float(
            self.declare_parameter('max_range', 12.0).value
        )

        self.min_forward_x = float(
            self.declare_parameter('min_forward_x', 0.50).value
        )

        self.max_lateral = float(
            self.declare_parameter('max_lateral', 6.0).value
        )

        self.max_bearing_deg = float(
            self.declare_parameter('max_bearing_deg', 75.0).value
        )
        self.max_bearing_rad = math.radians(
            self.max_bearing_deg
        )

        self.min_z = float(
            self.declare_parameter('min_z', -1.00).value
        )

        self.max_z = float(
            self.declare_parameter('max_z', 1.50).value
        )

        # ---------------------------------------------------------------------
        # Point-cloud reduction and object clustering.
        # These limits describe whether something is a physical cluster, not
        # whether it is a buoy.
        # ---------------------------------------------------------------------
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
                'cluster_tolerance_per_meter', 0.010
            ).value
        )

        self.absolute_min_cluster_points = int(
            self.declare_parameter(
                'absolute_min_cluster_points', 3
            ).value
        )

        self.max_cluster_points = int(
            self.declare_parameter(
                'max_cluster_points', 2500
            ).value
        )

        # ---------------------------------------------------------------------
        # Dominant horizontal-surface rejection.
        #
        # The LiDAR can return hundreds of points from the water surface,
        # pavement, a dock, or another broad horizontal surface.  Because the
        # object clusterer intentionally connects points in XY, those returns
        # can create a long chain that absorbs a nearby buoy into one enormous
        # cluster.  We remove the strongest near-horizontal plane before
        # clustering so upright buoy returns remain independent objects.
        # ---------------------------------------------------------------------
        self.surface_filter_enabled = bool(
            self.declare_parameter(
                'surface_filter_enabled', True
            ).value
        )

        self.surface_filter_min_z = float(
            self.declare_parameter(
                'surface_filter_min_z', -0.90
            ).value
        )

        self.surface_filter_max_z = float(
            self.declare_parameter(
                'surface_filter_max_z', 0.35
            ).value
        )

        self.surface_filter_distance = float(
            self.declare_parameter(
                'surface_filter_distance', 0.055
            ).value
        )

        self.surface_filter_max_tilt_deg = float(
            self.declare_parameter(
                'surface_filter_max_tilt_deg', 25.0
            ).value
        )

        self.surface_filter_ransac_iterations = int(
            self.declare_parameter(
                'surface_filter_ransac_iterations', 80
            ).value
        )

        self.surface_filter_min_points = int(
            self.declare_parameter(
                'surface_filter_min_points', 80
            ).value
        )

        self.surface_filter_min_fraction = float(
            self.declare_parameter(
                'surface_filter_min_fraction', 0.08
            ).value
        )

        # Diagnostics from the most recent plane-removal pass.
        self.last_surface_filter_removed = 0
        self.last_surface_plane = None
        self.last_surface_filtered_count = 0

        # ---------------------------------------------------------------------
        # Vertical-profile feature extraction.
        # Each cluster is divided into lower/middle/upper height bands so we
        # can later distinguish stepped/cylindrical shapes from flat cutouts.
        # ---------------------------------------------------------------------
        self.profile_min_points_per_band = int(
            self.declare_parameter(
                'profile_min_points_per_band', 3
            ).value
        )

        # ---------------------------------------------------------------------
        # Temporal tracking.
        # Broad clusters are tracked before final buoy classification.
        # ---------------------------------------------------------------------
        self.confirm_hits = int(
            self.declare_parameter('confirm_hits', 3).value
        )

        self.max_track_misses = int(
            self.declare_parameter('max_track_misses', 7).value
        )

        self.publish_misses = int(
            self.declare_parameter('publish_misses', 4).value
        )

        self.association_distance = float(
            self.declare_parameter(
                'association_distance', 0.45
            ).value
        )

        self.association_distance_per_meter = float(
            self.declare_parameter(
                'association_distance_per_meter', 0.03
            ).value
        )

        self.track_alpha = float(
            self.declare_parameter('track_alpha', 0.55).value
        )

        # ---------------------------------------------------------------------
        # Generic buoy confirmation / hysteresis.
        # ---------------------------------------------------------------------
        # A new track must look buoy-like for several measured frames before it
        # is promoted to a confirmed buoy. Once promoted, several temporarily
        # weak frames are tolerated before the buoy state is released.
        self.generic_confirm_hits = int(
            self.declare_parameter(
                'generic_confirm_hits', 3
            ).value
        )

        self.generic_release_bad_frames = int(
            self.declare_parameter(
                'generic_release_bad_frames', 6
            ).value
        )

        self.generic_confidence_alpha = float(
            self.declare_parameter(
                'generic_confidence_alpha', 0.60
            ).value
        )

        self.generic_confidence_decay = float(
            self.declare_parameter(
                'generic_confidence_decay', 0.94
            ).value
        )

        # ---------------------------------------------------------------------
        # Subtype classification policy.
        # If the best two accepted model scores are too close, the object is
        # still a buoy but the subtype is reported as unknown_buoy.
        # ---------------------------------------------------------------------
        self.ambiguity_margin = float(
            self.declare_parameter(
                'ambiguity_margin', 0.12
            ).value
        )

        # ---------------------------------------------------------------------
        # Diagnostics limits.
        # Prevent a cluttered scene from producing an unbounded JSON message.
        # ---------------------------------------------------------------------
        self.max_diagnostic_clusters = int(
            self.declare_parameter(
                'max_diagnostic_clusters', 40
            ).value
        )

        # ---------------------------------------------------------------------
        # TF setup.
        # ---------------------------------------------------------------------
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self
        )

        # ---------------------------------------------------------------------
        # ROS publishers and LiDAR subscription.
        # ---------------------------------------------------------------------
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

        self.classification_pub = self.create_publisher(
            String,
            self.classification_topic,
            10
        )

        self.diagnostics_pub = self.create_publisher(
            String,
            self.diagnostics_topic,
            10
        )

        self.cloud_sub = self.create_subscription(
            PointCloud2,
            self.cloud_topic,
            self.cloud_callback,
            qos_profile_sensor_data
        )

        # ---------------------------------------------------------------------
        # Temporal track storage.
        # ---------------------------------------------------------------------
        self.next_track_id = 1
        self.tracks = {}

        loaded_names = ', '.join(
            sorted(self.buoy_types.keys())
        )

        self.get_logger().info(
            'Multi-type buoy detector v4 started. '
            f'Generic model: {self.generic_buoy_spec.name}; subtypes: {loaded_names}. '
            f'Diagnostics: {self.diagnostics_topic}'
        )

    # =========================================================================
    # GLOBAL BUOY CONFIGURATION
    # =========================================================================

    # -------------------------------------------------------------------------
    # Loads buoy_types.yaml into BuoyTypeSpec objects.
    # -------------------------------------------------------------------------
    def load_buoy_types(self, filename):
        path = Path(filename)

        if not path.exists():
            raise FileNotFoundError(
                f'Buoy type configuration does not exist: {path}'
            )

        with path.open('r', encoding='utf-8') as stream:
            raw = yaml.safe_load(stream) or {}

        # The generic model answers only "is this a buoy?". It should stay
        # broad enough to cover every subtype used by the task.
        generic_config = raw.get('generic_buoy')

        if not isinstance(generic_config, dict):
            raise ValueError(
                f'{path} must contain a "generic_buoy" mapping'
            )

        generic_spec = BuoyTypeSpec(
            name='generic_buoy',
            enabled=True,
            min_points=int(generic_config.get('min_points', 3)),
            good_points=int(generic_config.get('good_points', 14)),
            min_score=float(generic_config.get('min_score', 0.52)),
            features=dict(generic_config.get('features', {})),
        )

        # Subtype models answer "which buoy type is this?" only after the
        # generic model has confirmed that the track is a buoy.
        raw_types = raw.get('buoy_types', {})

        if not isinstance(raw_types, dict) or not raw_types:
            raise ValueError(
                f'{path} must contain a non-empty "buoy_types" mapping'
            )

        specs = {}

        for name, config in raw_types.items():
            config = config or {}

            spec = BuoyTypeSpec(
                name=str(name),
                enabled=bool(config.get('enabled', True)),
                min_points=int(config.get('min_points', 3)),
                good_points=int(config.get('good_points', 12)),
                min_score=float(config.get('min_score', 0.55)),
                features=dict(config.get('features', {})),
            )

            if spec.enabled:
                specs[spec.name] = spec

        if not specs:
            raise ValueError(
                f'No enabled buoy subtypes were found in {path}'
            )

        return generic_spec, specs

    # =========================================================================
    # TOP-LEVEL PERCEPTION PIPELINE
    # =========================================================================

    # -------------------------------------------------------------------------
    # Runs once for every incoming Unitree LiDAR point cloud.
    # -------------------------------------------------------------------------
    def cloud_callback(self, cloud):
        # Decode raw XYZ points.
        input_points = self.read_xyz(cloud)
        input_count = len(input_points)

        if input_count == 0:
            self.last_surface_filter_removed = 0
            self.last_surface_plane = None
            self.last_surface_filtered_count = 0
            self.age_tracks_without_detections()
            self.publish_results(cloud)
            self.publish_diagnostics(
                cloud,
                input_count=0,
                roi_count=0,
                downsampled_count=0,
                candidates=[],
            )
            return

        # Transform the cloud into base_link.
        points = self.transform_points(
            input_points,
            cloud
        )

        if points is None:
            return

        # Crop to the configured search region.
        points = self.crop_roi(points)
        roi_count = len(points)

        # Downsample while preserving approximate geometry.
        if self.voxel_size > 0.0 and roi_count:
            points = self.voxel_downsample(
                points,
                self.voxel_size
            )

        downsampled_count = len(points)

        # Remove the dominant near-horizontal surface before clustering.
        # This prevents water/ground returns from chaining across the scene
        # and merging an otherwise valid buoy into a giant object cluster.
        points = self.remove_dominant_horizontal_surface(
            points
        )

        filtered_count = len(points)
        self.last_surface_filtered_count = filtered_count

        # Build broad physical clusters from the surface-filtered cloud.
        clusters = self.build_clusters(points)

        # Measure EVERY cluster and evaluate it against EVERY model.
        # Importantly, a model rejection does not remove the physical cluster.
        candidates = []

        for cluster_id, indices in enumerate(clusters):
            features = self.extract_features(
                points[indices]
            )

            if features is None:
                continue

            evaluations = self.evaluate_all_models(
                features
            )

            candidates.append(
                Candidate(
                    frame_cluster_id=cluster_id,
                    features=features,
                    evaluations=evaluations,
                )
            )

        # Track broad physical objects before final classification.
        self.update_tracks(candidates)

        # Update generic buoy state independently of subtype classification.
        # This is what prevents a confirmed buoy from disappearing merely
        # because one scan is ambiguous between cylinder and cardboard.
        self.update_generic_buoy_states()

        # Publish confirmed buoy tracks and full engineering diagnostics.
        self.publish_results(cloud)
        self.publish_diagnostics(
            cloud,
            input_count=input_count,
            roi_count=roi_count,
            downsampled_count=downsampled_count,
            candidates=candidates,
        )

    # =========================================================================
    # POINT-CLOUD INPUT AND TRANSFORM
    # =========================================================================

    # -------------------------------------------------------------------------
    # Converts a ROS PointCloud2 into an N x 3 NumPy XYZ array.
    # -------------------------------------------------------------------------
    def read_xyz(self, cloud):
        try:
            raw = point_cloud2.read_points_numpy(
                cloud,
                field_names=('x', 'y', 'z'),
                skip_nans=True,
            )
        except (AssertionError, ValueError) as exc:
            self.get_logger().error(
                f'Unable to decode PointCloud2 XYZ fields: {exc}'
            )
            return np.empty((0, 3), dtype=np.float64)

        points = np.asarray(raw)

        # Keep compatibility with structured NumPy arrays.
        if points.dtype.names:
            points = np.column_stack(
                (points['x'], points['y'], points['z'])
            )

        points = np.asarray(
            points,
            dtype=np.float64
        ).reshape((-1, 3))

        return points[
            np.isfinite(points).all(axis=1)
        ]

    # -------------------------------------------------------------------------
    # Transforms all points into base_link so the geometry has one consistent
    # boat-relative coordinate system.
    # -------------------------------------------------------------------------
    def transform_points(self, points, cloud):
        source_frame = cloud.header.frame_id

        if not source_frame:
            self.get_logger().warning(
                'Point cloud has no frame_id'
            )
            return None

        if source_frame == self.target_frame:
            return points

        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                source_frame,
                Time.from_msg(cloud.header.stamp),
                timeout=Duration(
                    seconds=self.transform_timeout
                ),
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

        rotation_matrix = self.quaternion_matrix(
            rotation.x,
            rotation.y,
            rotation.z,
            rotation.w,
        )

        offset = np.array(
            [
                translation.x,
                translation.y,
                translation.z,
            ],
            dtype=np.float64,
        )

        return points @ rotation_matrix.T + offset

    # -------------------------------------------------------------------------
    # Converts a quaternion into a 3x3 rotation matrix.
    # -------------------------------------------------------------------------
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

    # =========================================================================
    # ROI AND CLUSTERING
    # =========================================================================

    # -------------------------------------------------------------------------
    # Removes points outside the region where we want to search for markers.
    # -------------------------------------------------------------------------
    def crop_roi(self, points):
        x = points[:, 0]
        y = points[:, 1]
        z = points[:, 2]

        ranges = np.hypot(x, y)
        bearings = np.abs(
            np.arctan2(y, x)
        )

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

    # -------------------------------------------------------------------------
    # Reduces point count by replacing points in the same voxel with their
    # centroid.
    # -------------------------------------------------------------------------
    @staticmethod
    def voxel_downsample(points, voxel_size):
        keys = np.floor(
            points / voxel_size
        ).astype(np.int64)

        _, inverse = np.unique(
            keys,
            axis=0,
            return_inverse=True
        )

        counts = np.bincount(inverse)

        sums = np.zeros(
            (len(counts), 3),
            dtype=np.float64
        )

        np.add.at(
            sums,
            inverse,
            points
        )

        return sums / counts[:, None]

    # -------------------------------------------------------------------------
    # Removes the largest near-horizontal plane in the configured Z region.
    #
    # Why this is needed:
    #   - water/ground/dock returns are dense and spatially continuous;
    #   - XY connected-component clustering can therefore "walk" across the
    #     surface and absorb a buoy into a multi-meter cluster;
    #   - a buoy is primarily vertical, so removing a thin horizontal plane
    #     costs only a small band of buoy points while preserving its body.
    #
    # A small RANSAC fit is used instead of assuming one fixed waterline Z so
    # modest boat pitch/roll does not break the filter.
    # -------------------------------------------------------------------------
    def remove_dominant_horizontal_surface(self, points):
        self.last_surface_filter_removed = 0
        self.last_surface_plane = None

        if not self.surface_filter_enabled:
            return points

        if len(points) < self.surface_filter_min_points:
            return points

        # Only use points in the expected water/ground vertical region when
        # searching for the plane.  Points outside this band are still kept.
        search_mask = (
            (points[:, 2] >= self.surface_filter_min_z)
            & (points[:, 2] <= self.surface_filter_max_z)
        )

        search_points = points[search_mask]

        if len(search_points) < self.surface_filter_min_points:
            return points

        rng = np.random.default_rng(7)
        best_normal = None
        best_offset = None
        best_count = 0

        # A horizontal plane has a normal close to the Z axis.
        min_abs_normal_z = math.cos(
            math.radians(
                self.surface_filter_max_tilt_deg
            )
        )

        iterations = max(
            1,
            self.surface_filter_ransac_iterations
        )

        for _ in range(iterations):
            try:
                sample_indices = rng.choice(
                    len(search_points),
                    size=3,
                    replace=False
                )
            except ValueError:
                break

            p0, p1, p2 = search_points[sample_indices]

            normal = np.cross(
                p1 - p0,
                p2 - p0
            )

            norm = float(
                np.linalg.norm(normal)
            )

            if norm <= 1e-9:
                continue

            normal = normal / norm

            if abs(float(normal[2])) < min_abs_normal_z:
                continue

            # Keep the plane normal pointed upward so diagnostic coefficients
            # have a consistent sign.
            if normal[2] < 0.0:
                normal = -normal

            offset = -float(
                np.dot(normal, p0)
            )

            distances = np.abs(
                search_points @ normal
                + offset
            )

            count = int(
                np.count_nonzero(
                    distances
                    <= self.surface_filter_distance
                )
            )

            if count > best_count:
                best_count = count
                best_normal = normal
                best_offset = offset

        required = max(
            self.surface_filter_min_points,
            int(
                math.ceil(
                    self.surface_filter_min_fraction
                    * len(search_points)
                )
            )
        )

        if (
            best_normal is None
            or best_count < required
        ):
            return points

        all_distances = np.abs(
            points @ best_normal
            + best_offset
        )

        # Only remove plane-adjacent points inside the configured vertical
        # search band.  This avoids accidentally deleting an unrelated high
        # horizontal object elsewhere in the cloud.
        remove_mask = (
            (points[:, 2] >= self.surface_filter_min_z)
            & (points[:, 2] <= self.surface_filter_max_z)
            & (all_distances <= self.surface_filter_distance)
        )

        self.last_surface_filter_removed = int(
            np.count_nonzero(remove_mask)
        )

        self.last_surface_plane = {
            'a': float(best_normal[0]),
            'b': float(best_normal[1]),
            'c': float(best_normal[2]),
            'd': float(best_offset),
            'inliers': int(best_count),
        }

        return points[~remove_mask]

    # -------------------------------------------------------------------------
    # Groups nearby XY points into broad physical objects. The gap tolerance
    # increases with range because angular LiDAR spacing becomes wider farther
    # away from the sensor.
    # -------------------------------------------------------------------------
    def build_clusters(self, points):
        count = len(points)

        if count < self.absolute_min_cluster_points:
            return []

        ranges = np.hypot(
            points[:, 0],
            points[:, 1]
        )

        max_tolerance = (
            self.cluster_tolerance_base
            + self.cluster_tolerance_per_meter
            * self.max_range
        )

        tree = cKDTree(
            points[:, :2]
        )

        pairs = tree.query_pairs(
            max_tolerance,
            output_type='ndarray'
        )

        # Union-find groups points that are mutually connected.
        parent = np.arange(
            count,
            dtype=np.int32
        )
        rank = np.zeros(
            count,
            dtype=np.int8
        )

        def find(index):
            while parent[index] != index:
                parent[index] = parent[parent[index]]
                index = parent[index]
            return index

        def union(a, b):
            root_a = find(a)
            root_b = find(b)

            if root_a == root_b:
                return

            local_range = 0.5 * (
                ranges[a] + ranges[b]
            )

            allowed = (
                self.cluster_tolerance_base
                + self.cluster_tolerance_per_meter
                * local_range
            )

            separation = np.linalg.norm(
                points[a, :2]
                - points[b, :2]
            )

            if separation > allowed:
                return

            if rank[root_a] < rank[root_b]:
                parent[root_a] = root_b
            elif rank[root_a] > rank[root_b]:
                parent[root_b] = root_a
            else:
                parent[root_b] = root_a
                rank[root_a] += 1

        for a, b in pairs:
            union(
                int(a),
                int(b)
            )

        groups = {}

        for index in range(count):
            root = find(index)
            groups.setdefault(root, []).append(index)

        clusters = []

        for members in groups.values():
            cluster_count = len(members)

            if cluster_count < self.absolute_min_cluster_points:
                continue

            if cluster_count > self.max_cluster_points:
                continue

            clusters.append(
                np.asarray(
                    members,
                    dtype=np.int32
                )
            )

        return clusters

    # =========================================================================
    # FEATURE EXTRACTION
    # =========================================================================

    # -------------------------------------------------------------------------
    # Measures one broad physical cluster.
    #
    # Coordinate convention used for classification:
    #   width  = tangential extent across the LiDAR line of sight
    #   depth  = radial extent toward/away from the LiDAR
    #   height = vertical extent
    # -------------------------------------------------------------------------
    def extract_features(self, cluster):
        center = np.median(
            cluster,
            axis=0
        )

        x = float(center[0])
        y = float(center[1])
        z = float(center[2])

        target_range = math.hypot(
            x,
            y
        )

        if target_range <= 1e-6:
            return None

        radial = np.array(
            [x, y],
            dtype=np.float64
        ) / target_range

        tangent = np.array(
            [-radial[1], radial[0]],
            dtype=np.float64
        )

        xy_offsets = (
            cluster[:, :2]
            - center[:2]
        )

        radial_values = (
            xy_offsets @ radial
        )

        tangent_values = (
            xy_offsets @ tangent
        )

        width = float(
            np.ptp(tangent_values)
        )

        depth = float(
            np.ptp(radial_values)
        )

        height = float(
            np.ptp(cluster[:, 2])
        )

        # ---------------------------------------------------------------------
        # PCA shape descriptors.
        # ---------------------------------------------------------------------
        centered = (
            cluster
            - np.mean(cluster, axis=0)
        )

        verticality = 0.0
        linearity = 0.0
        planarity = 0.0

        if len(cluster) >= 3:
            covariance = np.cov(
                centered,
                rowvar=False
            )

            eigenvalues, eigenvectors = np.linalg.eigh(
                covariance
            )

            order = np.argsort(
                eigenvalues
            )[::-1]

            eigenvalues = np.maximum(
                eigenvalues[order],
                0.0
            )

            eigenvectors = eigenvectors[:, order]

            largest = max(
                float(eigenvalues[0]),
                1e-12
            )

            verticality = abs(
                float(eigenvectors[2, 0])
            )

            linearity = float(
                (eigenvalues[0] - eigenvalues[1])
                / largest
            )

            planarity = float(
                (eigenvalues[1] - eigenvalues[2])
                / largest
            )

        # ---------------------------------------------------------------------
        # Derived whole-object geometry.
        # ---------------------------------------------------------------------
        aspect_ratio = (
            height / max(width, 1e-4)
        )

        depth_to_width = (
            depth / max(width, 1e-4)
        )

        projected_area = max(
            width * height,
            1e-4
        )

        point_density = (
            len(cluster) / projected_area
        )

        # ---------------------------------------------------------------------
        # Vertical profile.
        # Measure the apparent tangential width in lower/middle/upper bands.
        # Missing bands remain None and are not automatically rejected.
        # ---------------------------------------------------------------------
        profile = self.extract_vertical_profile(
            cluster,
            tangent,
        )

        features = {
            'x': x,
            'y': y,
            'z': z,
            'range': target_range,
            'point_count': float(len(cluster)),
            'width': width,
            'depth': depth,
            'height': height,
            'verticality': verticality,
            'linearity': linearity,
            'planarity': planarity,
            'aspect_ratio': aspect_ratio,
            'depth_to_width': depth_to_width,
            'point_density': point_density,
        }

        features.update(profile)

        return features

    # -------------------------------------------------------------------------
    # Divides a cluster into vertical thirds and measures the tangential width
    # of each populated band.
    # -------------------------------------------------------------------------
    def extract_vertical_profile(self, cluster, tangent):
        z_values = cluster[:, 2]

        z_min = float(np.min(z_values))
        z_max = float(np.max(z_values))
        height = z_max - z_min

        result = {
            'lower_width': None,
            'middle_width': None,
            'upper_width': None,
            'lower_upper_ratio': None,
            'vertical_width_variation': None,
        }

        if height <= 1e-6:
            return result

        normalized_z = (
            (z_values - z_min)
            / height
        )

        # Use the cluster XY centroid as the common tangential origin.
        center_xy = np.mean(
            cluster[:, :2],
            axis=0
        )

        tangent_values = (
            (cluster[:, :2] - center_xy)
            @ tangent
        )

        bands = {
            'lower_width': (0.00, 0.34),
            'middle_width': (0.33, 0.67),
            'upper_width': (0.66, 1.01),
        }

        valid_widths = []

        for name, (low, high) in bands.items():
            mask = (
                (normalized_z >= low)
                & (normalized_z < high)
            )

            count = int(
                np.count_nonzero(mask)
            )

            if count < self.profile_min_points_per_band:
                continue

            band_width = float(
                np.ptp(tangent_values[mask])
            )

            result[name] = band_width
            valid_widths.append(band_width)

        lower = result['lower_width']
        upper = result['upper_width']

        if (
            lower is not None
            and upper is not None
            and upper > 1e-4
        ):
            result['lower_upper_ratio'] = (
                lower / upper
            )

        if len(valid_widths) >= 2:
            mean_width = max(
                float(np.mean(valid_widths)),
                1e-4
            )

            result['vertical_width_variation'] = (
                float(np.std(valid_widths))
                / mean_width
            )

        return result

    # =========================================================================
    # GLOBAL BUOY MODEL SCORING
    # =========================================================================

    # -------------------------------------------------------------------------
    # Evaluates one feature vector against every enabled global buoy type.
    # -------------------------------------------------------------------------
    def evaluate_generic_buoy(self, features):
        """Score one feature vector against the broad generic buoy model."""
        return self.score_model(
            features,
            self.generic_buoy_spec
        )

    def evaluate_all_models(self, features):
        """Score one feature vector against every configured buoy subtype."""
        evaluations = {}

        for name, spec in self.buoy_types.items():
            evaluations[name] = self.score_model(
                features,
                spec
            )

        return evaluations

    # -------------------------------------------------------------------------
    # Scores one feature vector against one buoy model.
    #
    # Rule semantics:
    #   hard_min / hard_max : immediate physical impossibility rejection
    #   soft_min / soft_max : expected range used for confidence scoring
    #   ideal               : best expected value inside the soft range
    #   weight              : contribution to geometry confidence
    #   required            : if true, missing feature rejects the model
    #   mode                : "ideal" or "range"
    #
    # Backward compatibility:
    #   old YAML min/max keys are interpreted as SOFT limits, not hard limits.
    # -------------------------------------------------------------------------
    def score_model(self, features, spec):
        reasons = []
        feature_scores = {}

        point_count = int(
            features.get('point_count', 0)
        )

        # Point count is still a hard minimum because an extremely sparse
        # cluster does not contain enough geometry to classify reliably.
        if point_count < spec.min_points:
            reasons.append(
                f'point_count {point_count} < min_points {spec.min_points}'
            )

            return ModelEvaluation(
                model_name=spec.name,
                score=0.0,
                accepted=False,
                reasons=reasons,
                feature_scores=feature_scores,
            )

        weighted_sum = 0.0
        total_weight = 0.0
        hard_failed = False

        for feature_name, raw_rule in spec.features.items():
            rule = raw_rule or {}

            if feature_name not in features:
                reasons.append(
                    f'unknown feature in config: {feature_name}'
                )
                hard_failed = True
                continue

            value = features.get(feature_name)
            required = bool(
                rule.get('required', False)
            )

            # Missing vertical-profile features are normal on partial scans.
            if value is None or not math.isfinite(float(value)):
                if required:
                    reasons.append(
                        f'{feature_name} missing but required'
                    )
                    hard_failed = True
                else:
                    feature_scores[feature_name] = None
                continue

            value = float(value)
            weight = float(
                rule.get('weight', 1.0)
            )

            hard_min = rule.get('hard_min')
            hard_max = rule.get('hard_max')

            # Old min/max syntax becomes soft_min/soft_max.
            soft_min = rule.get(
                'soft_min',
                rule.get('min')
            )
            soft_max = rule.get(
                'soft_max',
                rule.get('max')
            )

            ideal = rule.get('ideal')
            mode = str(
                rule.get(
                    'mode',
                    'ideal' if ideal is not None else 'range'
                )
            ).lower()

            if hard_min is not None:
                hard_min = float(hard_min)

                if value < hard_min:
                    reasons.append(
                        f'{feature_name}={value:.3f} '
                        f'< hard_min={hard_min:.3f}'
                    )
                    hard_failed = True

            if hard_max is not None:
                hard_max = float(hard_max)

                if value > hard_max:
                    reasons.append(
                        f'{feature_name}={value:.3f} '
                        f'> hard_max={hard_max:.3f}'
                    )
                    hard_failed = True

            # Hard failures are recorded, but we still calculate the rest of
            # the feature scores so diagnostics remain informative.
            score = self.score_feature(
                value=value,
                hard_min=hard_min,
                hard_max=hard_max,
                soft_min=soft_min,
                soft_max=soft_max,
                ideal=ideal,
                mode=mode,
            )

            feature_scores[feature_name] = float(score)

            if weight > 0.0:
                weighted_sum += weight * score
                total_weight += weight

        geometry_score = (
            weighted_sum / total_weight
            if total_weight > 0.0
            else 0.0
        )

        # Point support is deliberately only a modest part of the confidence.
        point_support = min(
            1.0,
            point_count / max(spec.good_points, 1)
        )

        final_score = (
            0.85 * geometry_score
            + 0.15 * point_support
        )

        final_score = float(
            np.clip(final_score, 0.0, 1.0)
        )

        if final_score < spec.min_score:
            reasons.append(
                f'score {final_score:.3f} '
                f'< min_score {spec.min_score:.3f}'
            )

        accepted = (
            not hard_failed
            and final_score >= spec.min_score
        )

        return ModelEvaluation(
            model_name=spec.name,
            score=final_score,
            accepted=accepted,
            reasons=reasons,
            feature_scores=feature_scores,
        )

    # -------------------------------------------------------------------------
    # Converts one feature measurement into a 0..1 confidence contribution.
    # -------------------------------------------------------------------------
    def score_feature(
        self,
        value,
        hard_min,
        hard_max,
        soft_min,
        soft_max,
        ideal,
        mode,
    ):
        soft_min = (
            float(soft_min)
            if soft_min is not None
            else None
        )

        soft_max = (
            float(soft_max)
            if soft_max is not None
            else None
        )

        ideal = (
            float(ideal)
            if ideal is not None
            else None
        )

        # ---------------------------------------------------------------------
        # RANGE MODE
        # Full score anywhere inside the expected soft range. Outside the soft
        # range, confidence fades toward the explicit hard limit. If no hard
        # limit exists on that side, it falls to zero outside the soft range.
        # ---------------------------------------------------------------------
        if mode == 'range' or ideal is None:
            if (
                soft_min is not None
                and value < soft_min
            ):
                if hard_min is None:
                    return 0.0

                denominator = (
                    soft_min - hard_min
                )

                if denominator <= 1e-12:
                    return 0.0

                return float(
                    np.clip(
                        (value - hard_min)
                        / denominator,
                        0.0,
                        1.0,
                    )
                )

            if (
                soft_max is not None
                and value > soft_max
            ):
                if hard_max is None:
                    return 0.0

                denominator = (
                    hard_max - soft_max
                )

                if denominator <= 1e-12:
                    return 0.0

                return float(
                    np.clip(
                        (hard_max - value)
                        / denominator,
                        0.0,
                        1.0,
                    )
                )

            return 1.0

        # ---------------------------------------------------------------------
        # IDEAL MODE
        # Score rises from the soft minimum to the ideal and falls from the
        # ideal to the soft maximum. Beyond the soft bounds, the score is zero.
        # ---------------------------------------------------------------------
        if value == ideal:
            return 1.0

        if value < ideal:
            if soft_min is None:
                return 1.0

            denominator = (
                ideal - soft_min
            )

            if denominator <= 1e-12:
                return 1.0

            return float(
                np.clip(
                    (value - soft_min)
                    / denominator,
                    0.0,
                    1.0,
                )
            )

        if soft_max is None:
            return 1.0

        denominator = (
            soft_max - ideal
        )

        if denominator <= 1e-12:
            return 1.0

        return float(
            np.clip(
                (soft_max - value)
                / denominator,
                0.0,
                1.0,
            )
        )

    # =========================================================================
    # TEMPORAL TRACKING BEFORE CLASSIFICATION
    # =========================================================================

    # -------------------------------------------------------------------------
    # Associates every broad current-frame cluster with an existing physical
    # track. Classification does not participate in association.
    # -------------------------------------------------------------------------
    def update_tracks(self, candidates):
        unmatched_tracks = set(
            self.tracks.keys()
        )

        # Nearest objects first gives deterministic association in clutter.
        ordered = sorted(
            candidates,
            key=lambda candidate: candidate.features['range']
        )

        for candidate in ordered:
            best_track_id = None
            best_distance = float('inf')

            for track_id in unmatched_tracks:
                track = self.tracks[track_id]

                distance = math.hypot(
                    candidate.x - track.x,
                    candidate.y - track.y,
                )

                track_range = math.hypot(
                    track.x,
                    track.y
                )

                allowed = (
                    self.association_distance
                    + self.association_distance_per_meter
                    * track_range
                )

                if (
                    distance <= allowed
                    and distance < best_distance
                ):
                    best_track_id = track_id
                    best_distance = distance

            if best_track_id is None:
                track = self.create_track(
                    candidate
                )
            else:
                track = self.tracks[
                    best_track_id
                ]

                self.update_track(
                    track,
                    candidate
                )

                unmatched_tracks.remove(
                    best_track_id
                )

            candidate.track_id = int(
                track.track_id
            )

        # Age tracks that received no measurement this frame.
        for track_id in unmatched_tracks:
            self.tracks[track_id].misses += 1

        # Remove tracks that have disappeared for too long.
        expired = [
            track_id
            for track_id, track in self.tracks.items()
            if track.misses > self.max_track_misses
        ]

        for track_id in expired:
            del self.tracks[track_id]

    # -------------------------------------------------------------------------
    # Starts a new track from a broad physical cluster.
    # -------------------------------------------------------------------------
    def create_track(self, candidate):
        track = Track(
            track_id=self.next_track_id,
            features=dict(candidate.features),
        )

        self.tracks[track.track_id] = track
        self.next_track_id += 1

        return track

    # -------------------------------------------------------------------------
    # Smooths the measurements of an existing physical track.
    # Missing optional features do not overwrite previously observed values.
    # -------------------------------------------------------------------------
    def update_track(self, track, candidate):
        alpha = self.track_alpha
        beta = 1.0 - alpha

        all_feature_names = set(
            track.features.keys()
        ) | set(
            candidate.features.keys()
        )

        for name in all_feature_names:
            new_value = candidate.features.get(name)
            old_value = track.features.get(name)

            if new_value is None:
                continue

            try:
                new_value = float(new_value)
            except (TypeError, ValueError):
                continue

            if not math.isfinite(new_value):
                continue

            if old_value is None:
                track.features[name] = new_value
                continue

            try:
                old_value = float(old_value)
            except (TypeError, ValueError):
                track.features[name] = new_value
                continue

            if not math.isfinite(old_value):
                track.features[name] = new_value
                continue

            track.features[name] = (
                alpha * new_value
                + beta * old_value
            )

        track.hits += 1
        track.misses = 0

    # -------------------------------------------------------------------------
    # Ages all tracks when an empty cloud arrives.
    # -------------------------------------------------------------------------
    def age_tracks_without_detections(self):
        for track in self.tracks.values():
            track.misses += 1

        expired = [
            track_id
            for track_id, track in self.tracks.items()
            if track.misses > self.max_track_misses
        ]

        for track_id in expired:
            del self.tracks[track_id]

    # =========================================================================
    # GENERIC BUOY CONFIRMATION / HYSTERESIS
    # =========================================================================

    def update_generic_buoy_states(self):
        """Update persistent BUOY / NOT-BUOY state for every physical track.

        A track is promoted only after repeated generic-buoy evidence. Once
        promoted, temporary weak geometry or short LiDAR dropouts do not
        immediately revoke it. Physical track expiry still bounds how long a
        disappeared object can survive.
        """
        for track in self.tracks.values():
            # No measurement this frame: preserve the confirmed state and
            # gently decay confidence. publish_misses controls how long the
            # held position is allowed onto /perception/objects.
            if track.misses > 0:
                if track.confirmed_buoy:
                    track.generic_confidence *= (
                        self.generic_confidence_decay
                    )
                continue

            evaluation = self.evaluate_generic_buoy(
                track.features
            )

            if evaluation.accepted:
                track.buoy_hits += 1
                track.buoy_bad_frames = 0

                alpha = self.generic_confidence_alpha

                if track.generic_confidence <= 0.0:
                    track.generic_confidence = evaluation.score
                else:
                    track.generic_confidence = (
                        alpha * evaluation.score
                        + (1.0 - alpha)
                        * track.generic_confidence
                    )

                if (
                    not track.confirmed_buoy
                    and track.buoy_hits
                    >= self.generic_confirm_hits
                ):
                    track.confirmed_buoy = True

            else:
                # Before confirmation, require a coherent run of buoy-like
                # measurements. After confirmation, use a slower release
                # counter instead of immediately dropping the track.
                if not track.confirmed_buoy:
                    track.buoy_hits = 0
                    track.generic_confidence = evaluation.score
                else:
                    track.buoy_bad_frames += 1
                    track.generic_confidence *= (
                        self.generic_confidence_decay
                    )

                    if (
                        track.buoy_bad_frames
                        > self.generic_release_bad_frames
                    ):
                        track.confirmed_buoy = False
                        track.buoy_hits = 0
                        track.buoy_bad_frames = 0

    # =========================================================================
    # SUBTYPE CLASSIFICATION DECISION
    # =========================================================================

    # -------------------------------------------------------------------------
    # Chooses a subtype from model evaluations.
    # Returns:
    #   subtype, confidence, accepted_evaluations
    # -------------------------------------------------------------------------
    def choose_classification(self, evaluations):
        accepted = [
            evaluation
            for evaluation in evaluations.values()
            if evaluation.accepted
        ]

        accepted.sort(
            key=lambda evaluation: evaluation.score,
            reverse=True
        )

        if not accepted:
            return None, 0.0, []

        best = accepted[0]

        # If two models are nearly tied, keep the general buoy detection but
        # avoid pretending we know the exact subtype.
        if len(accepted) >= 2:
            second = accepted[1]

            if (
                best.score - second.score
                < self.ambiguity_margin
            ):
                return (
                    'unknown_buoy',
                    best.score,
                    accepted,
                )

        return (
            best.model_name,
            best.score,
            accepted,
        )

    # =========================================================================
    # STANDARD PERCEPTION OUTPUT
    # =========================================================================

    # -------------------------------------------------------------------------
    # Publishes confirmed buoy tracks in the existing DetectedObjectArray
    # format so the current gate detector can consume them unchanged.
    # -------------------------------------------------------------------------
    def publish_results(self, cloud):
        output = DetectedObjectArray()
        output.header = cloud.header
        output.header.frame_id = self.target_frame

        classifications = []
        markers = MarkerArray()

        # Clear stale RViz markers every frame before adding current tracks.
        delete_all = Marker()
        delete_all.header = output.header
        delete_all.action = Marker.DELETEALL
        markers.markers.append(delete_all)

        for track_id in sorted(self.tracks):
            track = self.tracks[track_id]

            # Publish only tracks that have passed the independent generic
            # buoy confirmation state machine. Subtype ambiguity can no longer
            # remove an otherwise confirmed buoy from this output.
            if not track.confirmed_buoy:
                continue

            # Hold a confirmed buoy through a bounded number of missed scans.
            if track.misses > self.publish_misses:
                continue

            # Subtype classification is advisory. If none of the subtype
            # models cleanly wins, publish the object as unknown_buoy.
            evaluations = self.evaluate_all_models(
                track.features
            )

            subtype, subtype_confidence, accepted = (
                self.choose_classification(
                    evaluations
                )
            )

            if subtype is None:
                subtype = 'unknown_buoy'
                subtype_confidence = 0.0

            confidence = float(
                np.clip(
                    track.generic_confidence,
                    0.0,
                    1.0,
                )
            )

            obj = DetectedObject()
            obj.id = int(track.track_id)
            obj.object_type = DetectedObject.TYPE_BUOY
            obj.color = DetectedObject.COLOR_UNKNOWN

            obj.position.x = float(track.x)
            obj.position.y = float(track.y)
            obj.position.z = float(track.z)

            # Existing convention:
            #   size.x = radial depth
            #   size.y = tangential width
            #   size.z = vertical height
            obj.size.x = float(
                track.features.get('depth', 0.0)
            )
            obj.size.y = float(
                track.features.get('width', 0.0)
            )
            obj.size.z = float(
                track.features.get('height', 0.0)
            )

            obj.confidence = float(confidence)

            output.objects.append(obj)

            classifications.append(
                {
                    'id': int(track.track_id),
                    'type': subtype,
                    'confidence': round(
                        float(confidence),
                        3
                    ),
                    'subtype_confidence': round(
                        float(subtype_confidence),
                        3
                    ),
                    'hits': int(track.hits),
                    'misses': int(track.misses),
                    'buoy_hits': int(track.buoy_hits),
                    'buoy_bad_frames': int(track.buoy_bad_frames),
                    'confirmed_buoy': bool(track.confirmed_buoy),
                    'scores': {
                        name: round(
                            float(evaluation.score),
                            3
                        )
                        for name, evaluation
                        in sorted(evaluations.items())
                    },
                    'accepted_models': [
                        evaluation.model_name
                        for evaluation in accepted
                    ],
                }
            )

            markers.markers.extend(
                self.make_track_markers(
                    output.header,
                    track,
                    subtype,
                    confidence,
                )
            )

        self.objects_pub.publish(output)

        classification_msg = String()
        classification_msg.data = json.dumps(
            classifications,
            separators=(',', ':')
        )
        self.classification_pub.publish(
            classification_msg
        )

        self.marker_pub.publish(markers)

    # =========================================================================
    # ENGINEERING DIAGNOSTICS
    # =========================================================================

    # -------------------------------------------------------------------------
    # Publishes raw features and exact model rejection reasons for EVERY broad
    # current-frame cluster. This is the primary calibration output.
    # -------------------------------------------------------------------------
    def publish_diagnostics(
        self,
        cloud,
        input_count,
        roi_count,
        downsampled_count,
        candidates,
    ):
        diagnostic_candidates = []

        # Nearest clusters first are usually the most useful during bench tests.
        ordered = sorted(
            candidates,
            key=lambda candidate: candidate.features['range']
        )

        for candidate in ordered[
            :self.max_diagnostic_clusters
        ]:
            evaluations = candidate.evaluations
            generic_evaluation = self.evaluate_generic_buoy(
                candidate.features
            )

            subtype, confidence, accepted = (
                self.choose_classification(
                    evaluations
                )
            )

            diagnostic_candidates.append(
                {
                    'cluster': int(
                        candidate.frame_cluster_id
                    ),
                    'track_id': int(
                        candidate.track_id
                    ),
                    'position': {
                        'x': self.round_or_none(
                            candidate.features.get('x')
                        ),
                        'y': self.round_or_none(
                            candidate.features.get('y')
                        ),
                        'z': self.round_or_none(
                            candidate.features.get('z')
                        ),
                        'range': self.round_or_none(
                            candidate.features.get('range')
                        ),
                    },
                    'features': {
                        name: self.round_or_none(
                            candidate.features.get(name)
                        )
                        for name in (
                            'point_count',
                            'width',
                            'depth',
                            'height',
                            'verticality',
                            'linearity',
                            'planarity',
                            'aspect_ratio',
                            'depth_to_width',
                            'point_density',
                            'lower_width',
                            'middle_width',
                            'upper_width',
                            'lower_upper_ratio',
                            'vertical_width_variation',
                        )
                    },
                    'generic_buoy': {
                        'score': round(
                            float(generic_evaluation.score),
                            3
                        ),
                        'accepted': bool(
                            generic_evaluation.accepted
                        ),
                        'reasons': list(
                            generic_evaluation.reasons
                        ),
                    },
                    'classification': {
                        'type': subtype,
                        'confidence': round(
                            float(confidence),
                            3
                        ),
                        'accepted_models': [
                            evaluation.model_name
                            for evaluation in accepted
                        ],
                    },
                    'models': {
                        name: {
                            'score': round(
                                float(evaluation.score),
                                3
                            ),
                            'accepted': bool(
                                evaluation.accepted
                            ),
                            'reasons': list(
                                evaluation.reasons
                            ),
                            'feature_scores': {
                                feature_name: (
                                    None
                                    if score is None
                                    else round(
                                        float(score),
                                        3
                                    )
                                )
                                for feature_name, score
                                in sorted(
                                    evaluation.feature_scores.items()
                                )
                            },
                        }
                        for name, evaluation
                        in sorted(evaluations.items())
                    },
                }
            )

        # Include smoothed track features even when they do not classify as a
        # buoy yet. This reveals whether the real object is being tracked but
        # failing final model scoring.
        track_diagnostics = []

        for track_id in sorted(self.tracks):
            track = self.tracks[track_id]

            generic_evaluation = self.evaluate_generic_buoy(
                track.features
            )

            evaluations = self.evaluate_all_models(
                track.features
            )

            subtype, confidence, accepted = (
                self.choose_classification(
                    evaluations
                )
            )

            track_diagnostics.append(
                {
                    'id': int(track.track_id),
                    'hits': int(track.hits),
                    'misses': int(track.misses),
                    'buoy_hits': int(track.buoy_hits),
                    'buoy_bad_frames': int(track.buoy_bad_frames),
                    'confirmed_buoy': bool(track.confirmed_buoy),
                    'generic_score': round(
                        float(generic_evaluation.score),
                        3
                    ),
                    'generic_accepted': bool(
                        generic_evaluation.accepted
                    ),
                    'generic_confidence': round(
                        float(track.generic_confidence),
                        3
                    ),
                    'x': self.round_or_none(track.x),
                    'y': self.round_or_none(track.y),
                    'range': self.round_or_none(
                        math.hypot(track.x, track.y)
                    ),
                    'width': self.round_or_none(
                        track.features.get('width')
                    ),
                    'depth': self.round_or_none(
                        track.features.get('depth')
                    ),
                    'height': self.round_or_none(
                        track.features.get('height')
                    ),
                    'classification': subtype,
                    'confidence': round(
                        float(confidence),
                        3
                    ),
                    'accepted_models': [
                        evaluation.model_name
                        for evaluation in accepted
                    ],
                    'scores': {
                        name: round(
                            float(evaluation.score),
                            3
                        )
                        for name, evaluation
                        in sorted(evaluations.items())
                    },
                }
            )

        payload = {
            'stamp': {
                'sec': int(cloud.header.stamp.sec),
                'nanosec': int(
                    cloud.header.stamp.nanosec
                ),
            },
            'frame_id': self.target_frame,
            'counts': {
                'input_points': int(input_count),
                'roi_points': int(roi_count),
                'downsampled_points': int(
                    downsampled_count
                ),
                'surface_removed_points': int(
                    self.last_surface_filter_removed
                ),
                'surface_filtered_points': int(
                    getattr(
                        self,
                        'last_surface_filtered_count',
                        downsampled_count
                    )
                ),
                'clusters': int(len(candidates)),
                'tracks': int(len(self.tracks)),
            },
            'surface_plane': (
                None
                if self.last_surface_plane is None
                else {
                    key: (
                        int(value)
                        if key == 'inliers'
                        else round(float(value), 5)
                    )
                    for key, value
                    in self.last_surface_plane.items()
                }
            ),
            'clusters': diagnostic_candidates,
            'tracks': track_diagnostics,
        }

        msg = String()
        msg.data = json.dumps(
            payload,
            separators=(',', ':')
        )

        self.diagnostics_pub.publish(msg)

    # -------------------------------------------------------------------------
    # Rounds a numeric diagnostic value while preserving missing values as null.
    # -------------------------------------------------------------------------
    @staticmethod
    def round_or_none(value, digits=4):
        if value is None:
            return None

        try:
            value = float(value)
        except (TypeError, ValueError):
            return None

        if not math.isfinite(value):
            return None

        return round(value, digits)

    # =========================================================================
    # RVIZ VISUALIZATION
    # =========================================================================

    # -------------------------------------------------------------------------
    # Creates one box and one text label for a confirmed buoy track.
    # -------------------------------------------------------------------------
    def make_track_markers(
        self,
        header,
        track,
        subtype,
        confidence,
    ):
        box = Marker()
        box.header = header
        box.ns = 'buoy_detector_multi_boxes'
        box.id = int(track.track_id)
        box.type = Marker.CUBE
        box.action = Marker.ADD

        box.pose.position.x = float(track.x)
        box.pose.position.y = float(track.y)
        box.pose.position.z = float(track.z)
        box.pose.orientation.w = 1.0

        box.scale.x = max(
            float(track.features.get('depth', 0.0)),
            0.05
        )
        box.scale.y = max(
            float(track.features.get('width', 0.0)),
            0.05
        )
        box.scale.z = max(
            float(track.features.get('height', 0.0)),
            0.05
        )

        # RViz color encodes the advisory subtype while every marker remains
        # a generic TYPE_BUOY to downstream autonomy.
        if subtype == 'real_cylindrical_buoy':
            box.color.r = 0.1
            box.color.g = 0.9
            box.color.b = 0.9
        elif subtype == 'cardboard_cutout':
            box.color.r = 1.0
            box.color.g = 0.55
            box.color.b = 0.1
        else:
            box.color.r = 1.0
            box.color.g = 0.9
            box.color.b = 0.1

        box.color.a = 0.50
        box.lifetime.sec = 1

        text = Marker()
        text.header = header
        text.ns = 'buoy_detector_multi_labels'
        text.id = int(
            100000 + track.track_id
        )
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD

        text.pose.position.x = float(track.x)
        text.pose.position.y = float(track.y)
        text.pose.position.z = float(
            track.z
            + 0.5
            * float(
                track.features.get('height', 0.0)
            )
            + 0.20
        )
        text.pose.orientation.w = 1.0

        text.scale.z = 0.22
        text.color.r = 1.0
        text.color.g = 1.0
        text.color.b = 1.0
        text.color.a = 1.0

        text.text = (
            f'#{track.track_id} '
            f'{subtype} '
            f'buoy={confidence:.2f}'
        )
        text.lifetime.sec = 1

        return [box, text]


# =============================================================================
# ROS 2 ENTRY POINT
# =============================================================================
def main(args=None):
    rclpy.init(args=args)

    node = MultiTypeBuoyDetector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()