"""Short-horizon LiDAR coverage test, independent of the course survey.

A planned route is not proof that its water is currently observed clear. A valid
controller preview requires its swept hull disk inside fresh measured scan range.
"""
from dataclasses import dataclass
import math

from .models import angle_difference, finite


@dataclass(frozen=True)
class Scan:
    ranges: tuple
    angle_min: float
    angle_increment: float
    range_min: float
    range_max: float
    x: float
    y: float
    yaw: float
    stamp: float

    def validate(self):
        if (not finite(self.angle_min, self.angle_increment, self.range_min,
                       self.range_max, self.x, self.y, self.yaw, self.stamp)
                or not 8 <= len(self.ranges) <= 20000 or self.stamp <= 0
                or not 0 < self.angle_increment <= math.radians(2)
                or not 0 <= self.range_min < self.range_max):
            raise ValueError('Invalid or insufficiently resolved LiDAR scan')

    def disk_clear(self, point, radius):
        self.validate()
        dx, dy = point[0]-self.x, point[1]-self.y
        distance = math.hypot(dx, dy)
        bearing = math.atan2(dy, dx)-self.yaw
        support = math.pi if distance <= radius else math.asin(radius/distance)
        span = len(self.ranges)*self.angle_increment
        full = span >= 2*math.pi-self.angle_increment
        relative = angle_difference(bearing, self.angle_min+(len(self.ranges)-1)*self.angle_increment/2)
        if not full and abs(relative)+support > span/2-self.angle_increment:
            return False
        selected = [r for i, r in enumerate(self.ranges) if support == math.pi or
                    abs(angle_difference(self.angle_min+i*self.angle_increment, bearing))
                    <= support+self.angle_increment]
        if not selected:
            return False
        # Angular discretization margin accounts for the nearest measured ray.
        required = distance+radius+distance*math.sin(self.angle_increment/2)
        for r in selected:
            if math.isinf(r) and r > 0:
                r = self.range_max
            if not math.isfinite(r) or not self.range_min <= r <= self.range_max or r <= required:
                return False
        return True

    def preview_clear(self, samples, radius, now, timeout=0.5):
        if not finite(now, timeout, radius) or radius <= 0 or timeout <= 0:
            return False
        if not 0 <= now-self.stamp <= timeout:
            return False
        try:
            points = [(s.x, s.y) for s in samples]
            if not points:
                return False
            for a, b in zip(points, points[1:]):
                n = max(1, math.ceil(math.dist(a, b)/(radius/4)))
                for i in range(n+1):
                    p = (a[0]+(b[0]-a[0])*i/n, a[1]+(b[1]-a[1])*i/n)
                    if not self.disk_clear(p, radius):
                        return False
            return all(self.disk_clear(p, radius) for p in points)
        except ValueError:
            return False
