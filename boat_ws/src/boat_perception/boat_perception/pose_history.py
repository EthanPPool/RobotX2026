"""Bounded planar pose interpolation for timestamped perception geometry."""
from collections import deque
import math


def stamp_seconds(stamp):
    return int(stamp.sec) + int(stamp.nanosec) * 1e-9


def to_map(point, pose):
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    return x + c * point[0] - s * point[1], y + s * point[0] + c * point[1]


def to_body(point, pose):
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    dx, dy = point[0] - x, point[1] - y
    return c * dx + s * dy, -s * dx + c * dy


class PoseHistory:
    def __init__(self, duration=10.0, tolerance=0.15):
        self.duration, self.tolerance = duration, tolerance
        self.samples = deque(maxlen=2000)

    def add(self, stamp, x, y, yaw):
        if stamp <= 0 or not all(math.isfinite(v) for v in (stamp, x, y, yaw)):
            return False
        if self.samples and stamp < self.samples[-1][0]:
            return False  # Out-of-order poses must not rewind the history.
        item = (stamp, (x, y, yaw))
        if self.samples and stamp == self.samples[-1][0]:
            self.samples[-1] = item
        else:
            self.samples.append(item)
        while self.samples and stamp - self.samples[0][0] > self.duration:
            self.samples.popleft()
        return True

    def at(self, stamp):
        if not self.samples or not math.isfinite(stamp) or stamp <= 0:
            return None
        first, last = self.samples[0], self.samples[-1]
        if stamp <= first[0]:
            return first[1] if first[0] - stamp <= self.tolerance else None
        if stamp >= last[0]:
            return last[1] if stamp - last[0] <= self.tolerance else None
        for previous, following in zip(self.samples, list(self.samples)[1:]):
            if previous[0] <= stamp <= following[0]:
                if following[0] - previous[0] > 2 * self.tolerance:
                    return None
                fraction = (stamp - previous[0]) / (following[0] - previous[0])
                a, b = previous[1], following[1]
                dyaw = math.atan2(math.sin(b[2] - a[2]), math.cos(b[2] - a[2]))
                return (a[0] + fraction * (b[0] - a[0]),
                        a[1] + fraction * (b[1] - a[1]), a[2] + fraction * dyaw)
        return None
