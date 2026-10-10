"""Follow the USV's observed trail with measured underwater localization."""
import math
from collections import deque

import numpy as np

from .geometry import finite, fresh, yaw


class TrailFollower:
    def __init__(self, distance=3., depth=1.5, max_depth=5., max_xy=0.4, max_z=0.2,
                 kp=0.5, pose_timeout=0.5, frame='robotx_map', max_variance=1., max_jump=2.):
        if not finite(distance, depth, max_depth, max_xy, max_z, kp, pose_timeout, max_variance, max_jump):
            raise ValueError('Nonfinite follower parameter')
        if min(distance, depth, max_depth, max_xy, max_z, kp, pose_timeout, max_variance, max_jump) <= 0 or depth > max_depth:
            raise ValueError('Invalid follower limits')
        self.distance, self.depth, self.max_depth = distance, depth, max_depth
        self.max_xy, self.max_z, self.kp = max_xy, max_z, kp
        self.timeout, self.frame, self.max_variance, self.max_jump = pose_timeout, frame, max_variance, max_jump
        self.trail = deque(maxlen=2000)
        self.last_boat = None
        self.last_rx_stamp = None
        self.jump_latched = False

    def reset(self):
        self.trail.clear(); self.last_boat = None; self.last_rx_stamp = None; self.jump_latched = False

    def observe(self, boat, now):
        if not boat.valid(self.frame, now, self.timeout, self.max_variance):
            return False
        if self.last_rx_stamp is not None and boat.stamp <= self.last_rx_stamp:
            return False  # A replay cannot extend telemetry freshness.
        if self.last_boat is not None:
            discontinuity = math.dist(boat.position[:2], self.last_boat.position[:2]) > self.max_jump
            gap = boat.stamp-self.last_boat.stamp > self.timeout
            if discontinuity or gap:
                self.jump_latched = True
                return False  # Require operator reset after a localization jump or gap.
        self.last_rx_stamp = boat.stamp
        self.last_boat = boat
        if not self.trail or math.dist(boat.position[:2], self.trail[-1]) > 0.05:
            self.trail.append(tuple(boat.position[:2]))
        return True

    def command(self, own, now, enabled=False, authorized=False):
        zero = (0., 0., 0., 0.)
        if not enabled or not authorized:
            return zero, 'DISABLED_OR_UNAUTHORIZED', None
        if self.jump_latched:
            return zero, 'LOCALIZATION_DISCONTINUITY', None
        if self.last_boat is None or not self.last_boat.valid(self.frame, now, self.timeout, self.max_variance):
            return zero, 'USV_POSE_STALE', None
        if own is None or not own.valid(self.frame, now, self.timeout, self.max_variance):
            return zero, 'UUV_LOCALIZATION_UNAVAILABLE', None
        if own.position[2] > 0.2 or own.position[2] < -self.max_depth:
            return zero, 'DEPTH_ENVELOPE', None
        if math.dist(own.position[:2], self.last_boat.position[:2]) < 1.:
            return zero, 'USV_SEPARATION_STOP', None
        # Follow actual trail geometry; refuse to invent an unobserved path behind the boat.
        remaining = self.distance
        target = np.asarray(self.last_boat.position[:2])
        found = False
        points = list(self.trail)
        lag_index = 0
        for index in range(len(points)-2,-1,-1):
            point = points[index]
            delta = np.asarray(point)-target
            length = float(np.linalg.norm(delta))
            if length >= remaining and length > 0:
                target = target+remaining*delta/length; found = True; lag_index = index; break
            remaining -= length; target = np.asarray(point)
        if not found:
            return zero, 'WAIT_FOR_USV_TRAIL', None
        # Track only the trail before the lag point; advance at most one segment
        # at a time so a corner cannot be replaced by a long diagonal shortcut.
        nearest_candidate = None
        trail_to_target = points[:lag_index+1]+[tuple(target)]
        for index,(a, b) in enumerate(zip(trail_to_target, trail_to_target[1:])):
            dx,dy=b[0]-a[0],b[1]-a[1]
            norm2=dx*dx+dy*dy
            t=max(0.,min(1.,((own.position[0]-a[0])*dx+(own.position[1]-a[1])*dy)/norm2)) if norm2>0 else 0.
            q=(a[0]+t*dx,a[1]+t*dy)
            candidate=(math.hypot(q[0]-own.position[0],q[1]-own.position[1]),index,q)
            if nearest_candidate is None or candidate[0]<nearest_candidate[0]:
                nearest_candidate=candidate
        if nearest_candidate is not None:
            distance, index, nearest = nearest_candidate
            nearest = np.asarray(nearest)
            if distance > 0.5:
                target = nearest
            else:
                endpoint = np.asarray(trail_to_target[index+1])
                if math.dist(own.position[:2],endpoint)<0.15 and index+2<len(trail_to_target):
                    endpoint = np.asarray(trail_to_target[index+2])
                delta = endpoint-nearest
                length = float(np.linalg.norm(delta))
                target = nearest+min(0.7,length)*delta/length if length>0 else endpoint
        delta = target-np.asarray(own.position[:2])
        velocity = self.kp*delta
        length = float(np.linalg.norm(velocity))
        if length > self.max_xy:
            velocity *= self.max_xy/length
        vz = max(-self.max_z, min(self.max_z, self.kp*(-self.depth-own.position[2])))
        heading = math.atan2(delta[1], delta[0]) if np.linalg.norm(delta) > 0.15 else yaw(own.quaternion)
        error = math.atan2(math.sin(heading-yaw(own.quaternion)), math.cos(heading-yaw(own.quaternion)))
        return (float(velocity[0]), float(velocity[1]), vz, max(-0.25,min(0.25,error))), 'FOLLOWING', (float(target[0]),float(target[1]),-self.depth)
