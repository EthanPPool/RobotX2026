#!/usr/bin/env python3
import math

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node

from boat_interfaces.msg import NavigationTarget


class TargetController(Node):
    """Convert a base_link-relative point target into body-frame velocity commands."""

    def __init__(self):
        super().__init__('target_controller')

        self.target_topic = self.declare_parameter('target_topic', '/mission/target').value
        self.cmd_topic = self.declare_parameter('cmd_topic', '/control/cmd_vel').value
        self.yaw_kp = float(self.declare_parameter('yaw_kp', 0.90).value)
        self.max_yaw_rate = float(self.declare_parameter('max_yaw_rate', 0.45).value)
        self.max_forward_speed = float(self.declare_parameter('max_forward_speed', 0.55).value)
        self.target_timeout = float(self.declare_parameter('target_timeout', 0.50).value)
        self.stop_forward_angle = float(self.declare_parameter('stop_forward_angle', 1.20).value)

        self.last_target = None
        self.last_target_time = None

        self.cmd_pub = self.create_publisher(TwistStamped, self.cmd_topic, 10)
        self.target_sub = self.create_subscription(
            NavigationTarget, self.target_topic, self.target_callback, 10
        )
        self.timer = self.create_timer(0.05, self.update)

    def target_callback(self, msg: NavigationTarget) -> None:
        self.last_target = msg
        self.last_target_time = self.get_clock().now()

    def update(self) -> None:
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'

        if self.last_target is None or self.last_target_time is None:
            self.cmd_pub.publish(cmd)
            return

        age = (self.get_clock().now() - self.last_target_time).nanoseconds / 1e9
        if age > self.target_timeout or self.last_target.stop:
            self.cmd_pub.publish(cmd)
            return

        x = float(self.last_target.target.x)
        y = float(self.last_target.target.y)
        if x <= 0.0:
            self.cmd_pub.publish(cmd)
            return

        heading_error = math.atan2(y, x)
        yaw_rate = max(-self.max_yaw_rate, min(self.max_yaw_rate, self.yaw_kp * heading_error))

        desired_speed = max(0.0, min(self.max_forward_speed, float(self.last_target.desired_speed)))
        if abs(heading_error) >= self.stop_forward_angle:
            forward = 0.0
        else:
            forward = desired_speed * max(0.0, math.cos(heading_error))

        cmd.twist.linear.x = float(forward)
        cmd.twist.angular.z = float(yaw_rate)
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = TargetController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
