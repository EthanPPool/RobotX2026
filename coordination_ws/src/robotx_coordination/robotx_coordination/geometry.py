"""Coordinate contracts shared by mapping, navigation and underwater following.

Map axes are WGS84 ENU (east, north, up); z=0 is the surveyed water surface.
Quaternions are ROS xyzw. Optical axes are right/down/forward.
"""
import hashlib
import math
from dataclasses import dataclass

import numpy as np


def finite(*values):
    return all(math.isfinite(float(v)) for v in values)


def fresh(stamp, now, timeout, future_tolerance=0.05):
    return finite(stamp, now, timeout) and -future_tolerance <= now - stamp <= timeout


def rotation(q):
    a = np.asarray(q, dtype=float)
    if a.shape != (4,) or not np.isfinite(a).all() or np.linalg.norm(a) < 1e-6:
        raise ValueError('Invalid orientation quaternion')
    x, y, z, w = a / np.linalg.norm(a)
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ])


def yaw(q):
    r = rotation(q)
    return math.atan2(r[1, 0], r[0, 0])


def yaw_quaternion(angle):
    return (0., 0., math.sin(angle/2), math.cos(angle/2))


def reference_id(latitude, longitude, altitude):
    if not finite(latitude, longitude, altitude) or not -90 < latitude < 90 or not -180 <= longitude <= 180:
        raise ValueError('Invalid WGS84 water datum')
    text = f'{latitude:.9f},{longitude:.9f},{altitude:.4f},ENU'
    return hashlib.sha256(text.encode()).hexdigest()[:16]


def ecef(lat, lon, alt):
    lat, lon = math.radians(lat), math.radians(lon)
    a, e2 = 6378137., 6.69437999014e-3
    n = a / math.sqrt(1-e2*math.sin(lat)**2)
    return np.array([(n+alt)*math.cos(lat)*math.cos(lon),
                     (n+alt)*math.cos(lat)*math.sin(lon),
                     (n*(1-e2)+alt)*math.sin(lat)])


def geodetic_to_enu(lat, lon, alt, origin):
    reference_id(lat, lon, alt)
    reference_id(*origin)
    p, l = math.radians(origin[0]), math.radians(origin[1])
    r = np.array([[-math.sin(l), math.cos(l), 0.],
                  [-math.sin(p)*math.cos(l), -math.sin(p)*math.sin(l), math.cos(p)],
                  [math.cos(p)*math.cos(l), math.cos(p)*math.sin(l), math.sin(p)]])
    return r @ (ecef(lat, lon, alt)-ecef(*origin))


def ray_to_water(ray_optical, position, quaternion, max_range=100., min_down=0.15):
    """Intersect a calibrated optical ray with water z=0; reject horizon rays."""
    pos = np.asarray(position, dtype=float)
    ray = rotation(quaternion) @ np.asarray(ray_optical, dtype=float)
    if not np.isfinite(pos).all() or not np.isfinite(ray).all() or pos[2] <= 0:
        raise ValueError('Camera must be above water with finite pose')
    ray /= np.linalg.norm(ray)
    if ray[2] >= -min_down:
        raise ValueError('Camera ray does not reliably intersect water')
    distance = -pos[2]/ray[2]
    if distance > max_range:
        raise ValueError('Intersection beyond mapping range')
    return pos + distance*ray


def point_in_polygon(p, vertices):
    x, y = p
    inside = False
    for a, b in zip(vertices, vertices[1:]+vertices[:1]):
        if (a[1] > y) != (b[1] > y):
            cross = (b[0]-a[0])*(y-a[1])/(b[1]-a[1])+a[0]
            if x < cross:
                inside = not inside
    return inside


def segment_distance(p, a, b):
    dx,dy=float(b[0]-a[0]),float(b[1]-a[1])
    norm=dx*dx+dy*dy
    t=max(0.,min(1.,((p[0]-a[0])*dx+(p[1]-a[1])*dy)/norm)) if norm>1e-12 else 0.
    return math.hypot(p[0]-(a[0]+t*dx),p[1]-(a[1]+t*dy))


@dataclass(frozen=True)
class Obstacle:
    x: float
    y: float
    radius: float = 0.3
    uncertainty: float = 0.5
    color: str = 'unknown'
    stamp: float = 0.


@dataclass(frozen=True)
class Footprint:
    points: tuple
    stamp: float


@dataclass(frozen=True)
class Pose:
    position: tuple
    quaternion: tuple
    stamp: float
    frame: str = 'robotx_map'
    variance: float = 0.

    def valid(self, frame, now, timeout, max_variance=1.):
        try:
            rotation(self.quaternion)
            return (self.frame == frame and finite(*self.position, self.variance)
                    and 0 <= self.variance <= max_variance and fresh(self.stamp, now, timeout))
        except ValueError:
            return False
