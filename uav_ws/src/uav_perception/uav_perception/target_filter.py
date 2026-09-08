#!/usr/bin/env python3
import math

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node


class TargetFilter(Node):
    """Validate a target pose before exposing it to mission logic.

    The normalized target contract is PoseStamped in base_link (ROS FLU):
      x forward, y left, z up.
    """

    def __init__(self):
        super().__init__('target_filter')
        self.declare_parameter('input_topic', '/perception/target_raw')
        self.declare_parameter('output_topic', '/perception/target')
        self.declare_parameter('required_frame', 'base_link')
        self.declare_parameter('max_distance', 30.0)

        self.required_frame = str(self.get_parameter('required_frame').value)
        self.max_distance = float(self.get_parameter('max_distance').value)
        self.pub = self.create_publisher(
            PoseStamped, str(self.get_parameter('output_topic').value), 10)
        self.sub = self.create_subscription(
            PoseStamped, str(self.get_parameter('input_topic').value), self._cb, 10)

    def _cb(self, msg: PoseStamped):
        if msg.header.frame_id != self.required_frame:
            self.get_logger().warn(
                f'Rejecting target frame {msg.header.frame_id!r}; '
                f'expected {self.required_frame!r}.', throttle_duration_sec=2.0)
            return

        p = msg.pose.position
        values = (float(p.x), float(p.y), float(p.z))
        if not all(math.isfinite(v) for v in values):
            self.get_logger().warn('Rejecting non-finite target.', throttle_duration_sec=2.0)
            return
        if math.sqrt(sum(v * v for v in values)) > self.max_distance:
            self.get_logger().warn('Rejecting target outside max_distance.', throttle_duration_sec=2.0)
            return

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TargetFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
