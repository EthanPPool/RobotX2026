from dataclasses import dataclass
import math


def finite(*values):
    return all(math.isfinite(v) for v in values)


def angle_difference(a, b):
    return math.atan2(math.sin(a-b), math.cos(a-b))


@dataclass(frozen=True)
class Marker:
    track_id: int
    x: float
    y: float
    radius: float = 0.3
    uncertainty: float = 0.25
    color: str = 'unknown'
    light_state: str = 'unknown'
    confidence: float = 0.0
    semantic_stamp: float = 0.0
    color_source: str = ''

    @property
    def point(self):
        return self.x, self.y

    @property
    def role(self):
        if self.color == 'blue' and self.light_state == 'flashing':
            return 'entry'
        if self.color == 'blue' and self.light_state == 'steady':
            return 'exit'
        if self.color in ('red', 'green') and self.light_state == 'flashing':
            return self.color
        return 'obstacle'

    def validate(self):
        if (not isinstance(self.track_id, int) or self.track_id < 0
                or not finite(self.x, self.y, self.radius, self.uncertainty,
                              self.confidence, self.semantic_stamp)
                or self.radius <= 0 or self.uncertainty < 0
                or not 0 <= self.confidence <= 1):
            raise ValueError('Invalid marker geometry or metadata')


@dataclass(frozen=True)
class Pose:
    x: float
    y: float
    yaw: float
    stamp: float

    @property
    def point(self):
        return self.x, self.y

    def validate(self):
        if not finite(self.x, self.y, self.yaw, self.stamp) or self.stamp <= 0:
            raise ValueError('Invalid measured pose')


@dataclass(frozen=True)
class PlannerConfig:
    hull_radius: float = 0.9
    clearance: float = 0.35
    orbit_radius: float = 3.0
    max_orbit_radius: float = 6.0
    passage_range: float = 6.0
    abeam_tolerance: float = 0.2
    graph_samples: int = 16
    max_search_states: int = 40000
    max_markers: int = 20
    max_semantic_markers: int = 8
    speed: float = 0.25
    acceleration: float = 0.15
    yaw_rate: float = 0.2
    yaw_acceleration: float = 0.15
    preview_dt: float = 0.1
    arrival_tolerance: float = 0.25
    heading_tolerance: float = 0.2
    tracking_soft_limit: float = 0.4
    tracking_hard_limit: float = 1.2
    min_confidence: float = 0.6

    def validate(self):
        scalars = (self.hull_radius, self.clearance, self.orbit_radius,
            self.max_orbit_radius, self.passage_range, self.abeam_tolerance,
            self.speed, self.acceleration, self.yaw_rate, self.yaw_acceleration,
            self.preview_dt, self.arrival_tolerance, self.heading_tolerance,
            self.tracking_soft_limit, self.tracking_hard_limit)
        if (not all(math.isfinite(v) and v > 0 for v in scalars)
                or self.max_orbit_radius < self.orbit_radius
                or not 8 <= self.graph_samples <= 64
                or not 1 <= self.max_search_states <= 200000
                or not 2 <= self.max_markers <= 100
                or not 1 <= self.max_semantic_markers <= 12
                or self.tracking_soft_limit >= self.tracking_hard_limit
                or not math.isfinite(self.min_confidence)
                or not 0 <= self.min_confidence <= 1):
            raise ValueError('Invalid planner configuration')
