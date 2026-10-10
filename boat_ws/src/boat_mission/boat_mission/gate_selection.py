"""Mission gate identity in the fixed local frame, independent of ROS."""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class GateGeometry:
    port: tuple
    starboard: tuple
    confidence: float
    measurement_stamp: float

    @property
    def center(self):
        return tuple((a + b) / 2 for a, b in zip(self.port, self.starboard))

    @property
    def width(self):
        return math.dist(self.port, self.starboard)


def same_gate(a, b, center_limit=0.8, marker_limit=1.0, width_limit=0.4):
    # Marker ordering can reverse with heading; compare both assignments.
    direct = max(math.dist(a.port, b.port), math.dist(a.starboard, b.starboard))
    swapped = max(math.dist(a.port, b.starboard), math.dist(a.starboard, b.port))
    return (math.dist(a.center, b.center) <= center_limit and
            min(direct, swapped) <= marker_limit and abs(a.width - b.width) <= width_limit)


class GateSelector:
    def __init__(self, center_limit=0.8, marker_limit=1.0, width_limit=0.4,
                 max_bearing_deg=45.0):
        self.center_limit, self.marker_limit, self.width_limit = center_limit, marker_limit, width_limit
        self.max_bearing = math.radians(max_bearing_deg)
        self.anchor = None
        self.passed = []

    def matches(self, a, b):
        return same_gate(a, b, self.center_limit, self.marker_limit, self.width_limit)

    def choose(self, candidates, pose):
        if self.anchor is not None:
            matches = [g for g in candidates if self.matches(self.anchor, g)]
            return min(matches, key=lambda g: (math.dist(g.center, self.anchor.center),
                                               -g.confidence)) if matches else None
        x, y, yaw = pose
        eligible = []
        for gate in candidates:
            if any(self.matches(old, gate) for old in self.passed):
                continue
            dx, dy = gate.center[0] - x, gate.center[1] - y
            bearing = math.atan2(math.sin(math.atan2(dy, dx) - yaw),
                                 math.cos(math.atan2(dy, dx) - yaw))
            if abs(bearing) <= self.max_bearing:
                eligible.append(gate)
        if not eligible:
            return None
        chosen = min(eligible, key=lambda g: (math.dist(g.center, (x, y)),
                                              -g.confidence, g.center))
        self.anchor = chosen  # Fixed acquisition anchor prevents gradual hopping.
        return chosen

    def complete(self, geometry):
        if geometry is not None:
            self.passed.append(geometry)
        self.anchor = None

    def release(self):
        self.anchor = None

    def reset(self):
        self.anchor = None
        self.passed.clear()
