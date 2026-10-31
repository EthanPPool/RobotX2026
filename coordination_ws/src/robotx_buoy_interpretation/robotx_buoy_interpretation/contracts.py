"""Pure Python contracts. SI units, ROS optical axes, shared ENU map frame.

Detection pixels are in the original rectified image, never network input space.
Positions and capture timestamps describe observations, not publish time.
"""
from dataclasses import dataclass
import math

COLORS = frozenset(('unknown', 'red', 'green', 'blue', 'black', 'yellow', 'white'))
LIGHT_STATES = frozenset(('unknown', 'steady', 'flashing', 'inactive'))


@dataclass(frozen=True)
class Camera:
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float

    def validate(self):
        if self.width <= 0 or self.height <= 0 or not all(math.isfinite(v) for v in
                (self.fx, self.fy, self.cx, self.cy)) or min(self.fx, self.fy) <= 0:
            raise ValueError('Invalid rectified camera intrinsics')


@dataclass(frozen=True)
class Detection:
    detection_id: int
    u: float
    v: float
    width: float
    height: float
    color: str
    confidence: float
    light_state: str = 'unknown'

    def validate(self, camera):
        camera.validate()
        if (not all(math.isfinite(v) for v in (self.u, self.v, self.width,
                self.height, self.confidence)) or self.width <= 0 or self.height <= 0
                or not 0 <= self.u < camera.width or not 0 <= self.v < camera.height
                or not 0 <= self.confidence <= 1 or self.color not in COLORS
                or self.light_state not in LIGHT_STATES):
            raise ValueError('Invalid image detection')


@dataclass(frozen=True)
class Observation:
    observation_id: str
    source: str
    stamp: float
    x: float
    y: float
    z: float
    color: str
    confidence: float
    uncertainty: float
    radius: float = 0.3
    light_state: str = 'unknown'

    def validate(self):
        if (self.source not in ('usv', 'uav') or not self.observation_id
                or self.color not in COLORS or self.light_state not in LIGHT_STATES
                or not all(math.isfinite(v) for v in (self.stamp, self.x, self.y,
                    self.z, self.confidence, self.uncertainty, self.radius))
                or self.stamp <= 0 or not 0 <= self.confidence <= 1
                or self.uncertainty < 0 or self.radius <= 0):
            raise ValueError('Invalid buoy observation')
