"""Small ROS adapters; core algorithms can run without ROS installed."""
import math
import time

from geometry_msgs.msg import Point

from .geometry import Pose, fresh


def seconds(stamp):
    return stamp.sec+stamp.nanosec/1e9


def now_sec(node):
    return node.get_clock().now().nanoseconds/1e9


def point(values):
    p = Point(); p.x,p.y,p.z = map(float,values); return p


def quaternion(q):
    return (q.x,q.y,q.z,q.w)


def odometry_pose(msg):
    p = msg.pose.pose.position
    c = msg.pose.covariance
    # Negative/NaN variance must not silently pass as trustworthy localization.
    values = [float(c[i]) for i in (0,7,14)]
    variance = max(values) if all(math.isfinite(v) and v >= 0 for v in values) else math.inf
    return Pose((p.x,p.y,p.z),quaternion(msg.pose.pose.orientation),seconds(msg.header.stamp),msg.header.frame_id,variance)


class Received:
    """Both receipt age and source stamp are checked; replays never refresh state."""
    def __init__(self):
        self.message = None; self.stamp = None; self.receipt = None

    def update(self, msg, stamp):
        if self.stamp is not None and stamp <= self.stamp:
            return False
        self.message, self.stamp, self.receipt = msg, stamp, time.monotonic()
        return True

    def valid(self, now, timeout):
        return self.message is not None and fresh(self.stamp,now,timeout) and time.monotonic()-self.receipt <= timeout
