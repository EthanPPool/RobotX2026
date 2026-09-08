#!/usr/bin/env python3
import math

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from uav_interfaces.msg import MissionCommand


class VelocityGuidance(Node):
    def __init__(self):
        super().__init__('velocity_guidance')
        self.declare_parameter('input_topic', '/mission/command')
        self.declare_parameter('output_topic', '/control/cmd_vel')
        self.declare_parameter('max_horizontal_speed', 1.0)
        self.declare_parameter('max_vertical_speed', 0.5)
        self.declare_parameter('max_yaw_rate', 0.5)

        self.max_horizontal = float(self.get_parameter('max_horizontal_speed').value)
        self.max_vertical = float(self.get_parameter('max_vertical_speed').value)
        self.max_yaw = float(self.get_parameter('max_yaw_rate').value)
        self.pub = self.create_publisher(
            TwistStamped, str(self.get_parameter('output_topic').value), 10)
        self.sub = self.create_subscription(
            MissionCommand,
            str(self.get_parameter('input_topic').value),
            self._command_cb,
            10)

    @staticmethod
    def _finite_or_zero(value):
        return float(value) if math.isfinite(float(value)) else 0.0

    @staticmethod
    def _clamp(value, limit):
        return max(-limit, min(limit, value))

    def _command_cb(self, cmd: MissionCommand):
        out = TwistStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'base_link'

        if cmd.command == MissionCommand.VELOCITY:
            x = self._finite_or_zero(cmd.linear.x)
            y = self._finite_or_zero(cmd.linear.y)
            norm = math.hypot(x, y)
            if norm > self.max_horizontal and norm > 0.0:
                scale = self.max_horizontal / norm
                x *= scale
                y *= scale
            out.twist.linear.x = x
            out.twist.linear.y = y
            out.twist.linear.z = self._clamp(
                self._finite_or_zero(cmd.linear.z), self.max_vertical)
            out.twist.angular.z = self._clamp(
                self._finite_or_zero(cmd.yaw_rate), self.max_yaw)
        elif cmd.command in (MissionCommand.HOLD, MissionCommand.ABORT, MissionCommand.LAND):
            # LAND is intentionally not implemented as a velocity descent here.
            # Landing should use the autopilot's dedicated LAND mode/service later.
            pass
        else:
            self.get_logger().warn(f'Unknown mission command {cmd.command}; commanding HOLD.')

        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = VelocityGuidance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
