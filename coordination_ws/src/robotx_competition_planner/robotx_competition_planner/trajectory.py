"""C2 timed references for a two-thruster vessel: stop, turn, advance or orbit.

Each primitive starts/ends at zero velocity and acceleration. Heading is unwrapped
continuously; no instantaneous corner turn or artificial first-order plant is used.
"""
from dataclasses import dataclass, replace
import math

from .models import angle_difference


@dataclass(frozen=True)
class Sample:
    x: float
    y: float
    yaw: float
    vx: float = 0.
    vy: float = 0.
    ax: float = 0.
    ay: float = 0.
    yaw_rate: float = 0.
    yaw_acceleration: float = 0.
    offset: float = 0.


def profile(time, duration):
    u = max(0., min(1., time/duration))
    return (10*u**3-15*u**4+6*u**5,
            (30*u**2-60*u**3+30*u**4)/duration,
            (60*u-180*u**2+120*u**3)/duration**2)


@dataclass(frozen=True)
class Primitive:
    kind: str
    start: tuple
    end: tuple
    yaw: float
    amount: float
    duration: float
    center: tuple = (0., 0.)
    radius: float = 0.
    theta: float = 0.

    def sample(self, time):
        s, sd, sdd = profile(time, self.duration)
        if self.kind == 'turn':
            return Sample(*self.start, self.yaw+self.amount*s,
                          yaw_rate=self.amount*sd, yaw_acceleration=self.amount*sdd)
        if self.kind == 'line':
            dx, dy = self.end[0]-self.start[0], self.end[1]-self.start[1]
            return Sample(self.start[0]+s*dx, self.start[1]+s*dy, self.yaw,
                          sd*dx, sd*dy, sdd*dx, sdd*dy)
        theta = self.theta+self.amount*s
        w, alpha = self.amount*sd, self.amount*sdd
        c, sn, r = math.cos(theta), math.sin(theta), self.radius
        return Sample(self.center[0]+r*c, self.center[1]+r*sn,
            self.yaw+self.amount*s, -r*sn*w, r*c*w,
            -r*c*w*w-r*sn*alpha, -r*sn*w*w+r*c*alpha, w, alpha)


class Trajectory:
    def __init__(self, primitives, start_sample):
        self.primitives, self.initial = tuple(primitives), start_sample
        self.duration = sum(p.duration for p in self.primitives)

    @property
    def end(self):
        return self.sample(self.duration)

    def sample(self, time):
        if not math.isfinite(time):
            raise ValueError('Nonfinite trajectory time')
        remaining = max(0., time)
        for p in self.primitives:
            if remaining <= p.duration:
                return p.sample(remaining)
            remaining -= p.duration
        return self.primitives[-1].sample(self.primitives[-1].duration) if self.primitives else self.initial

    def preview(self, elapsed, dt):
        if not math.isfinite(dt) or dt <= 0:
            raise ValueError('Preview dt must be finite and positive')
        return tuple(replace(self.sample(elapsed+i*dt), offset=i*dt) for i in range(3))


def duration(amount, rate, acceleration):
    return max(0.01, 1.875*abs(amount)/rate,
               math.sqrt(5.773503*abs(amount)/acceleration))


def polyline(path, initial_yaw, config):
    if not path:
        raise ValueError('Empty reference path')
    primitives, yaw = [], initial_yaw
    for a, b in zip(path, path[1:]):
        distance = math.dist(a, b)
        if distance < 1e-9:
            continue
        change = angle_difference(math.atan2(b[1]-a[1], b[0]-a[0]), yaw)
        if abs(change) > 1e-9:
            primitives.append(Primitive('turn', a, a, yaw, change,
                duration(change, config.yaw_rate, config.yaw_acceleration)))
            yaw += change
        primitives.append(Primitive('line', a, b, yaw, distance,
            duration(distance, config.speed, config.acceleration)))
    return Trajectory(primitives, Sample(*path[0], initial_yaw))


def orbit(arc, initial_yaw, config):
    tangent = arc.start_angle+arc.direction*math.pi/2
    change = angle_difference(tangent, initial_yaw)
    primitives = []
    if abs(change) > 1e-9:
        primitives.append(Primitive('turn', arc.start, arc.start, initial_yaw, change,
            duration(change, config.yaw_rate, config.yaw_acceleration)))
    amount = arc.direction*2*math.pi
    # Reserve half the linear acceleration for centripetal acceleration.
    speed = min(config.speed, math.sqrt(config.acceleration*arc.radius/2))
    seconds = max(duration(abs(amount)*arc.radius, speed, config.acceleration/2),
                  duration(amount, config.yaw_rate, config.yaw_acceleration))
    primitives.append(Primitive('arc', arc.start, arc.start, initial_yaw+change,
        amount, seconds, arc.center, arc.radius, arc.start_angle))
    return Trajectory(primitives, Sample(*arc.start, initial_yaw))
