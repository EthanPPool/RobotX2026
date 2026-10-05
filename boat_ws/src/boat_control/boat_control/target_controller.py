#!/usr/bin/env python3

"""Generic NavigationTarget-to-body-velocity controller for the BlueBoat."""

import json
import math

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from std_msgs.msg import String

from boat_interfaces.msg import NavigationTarget


class TargetController(Node):
    """Convert base_link-relative navigation targets into velocity commands."""

    def __init__(self):
        super().__init__('target_controller')

        self.target_topic = self.declare_parameter(
            'target_topic', '/mission/target'
        ).value
        self.cmd_topic = self.declare_parameter(
            'cmd_topic', '/control/cmd_vel'
        ).value
        self.diagnostics_topic = self.declare_parameter(
            'diagnostics_topic', '/control/diagnostics'
        ).value

        self.yaw_kp = float(self.declare_parameter('yaw_kp', 0.60).value)
        self.max_yaw_rate = float(
            self.declare_parameter('max_yaw_rate', 0.12).value
        )
        self.max_forward_speed = float(
            self.declare_parameter('max_forward_speed', 0.15).value
        )
        self.target_timeout = float(
            self.declare_parameter('target_timeout', 0.30).value
        )
        self.forward_angle_limit_deg = float(
            self.declare_parameter('forward_angle_limit_deg', 20.0).value
        )

        if self.max_yaw_rate < 0.0:
            raise ValueError('max_yaw_rate cannot be negative')
        if self.max_forward_speed < 0.0:
            raise ValueError('max_forward_speed cannot be negative')
        if self.target_timeout <= 0.0:
            raise ValueError('target_timeout must be positive')
        if not 0.0 < self.forward_angle_limit_deg <= 180.0:
            raise ValueError('forward_angle_limit_deg must be in (0, 180]')

        self.last_target = None
        self.last_target_time = None

        self.cmd_pub = self.create_publisher(TwistStamped, self.cmd_topic, 10)
        self.diagnostics_pub = self.create_publisher(
            String, self.diagnostics_topic, 10
        )
        self.create_subscription(
            NavigationTarget,
            self.target_topic,
            self.target_callback,
            10,
        )
        self.timer = self.create_timer(0.05, self.update)

    def target_callback(self, msg):
        self.last_target = msg
        self.last_target_time = self.get_clock().now()

    def target_age(self):
        if self.last_target_time is None:
            return None
        return (
            self.get_clock().now() - self.last_target_time
        ).nanoseconds / 1e9

    def publish_command(
        self,
        linear_x=0.0,
        angular_z=0.0,
        reason='ZERO_REQUESTED',
        heading_error=None,
        forward_allowed=False,
    ):
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'
        cmd.twist.linear.x = float(linear_x)
        cmd.twist.angular.z = float(angular_z)
        self.cmd_pub.publish(cmd)

        target_x = None
        target_y = None
        desired_speed = None
        target_stop = None
        if self.last_target is not None:
            target_x = float(self.last_target.target.x)
            target_y = float(self.last_target.target.y)
            desired_speed = float(self.last_target.desired_speed)
            target_stop = bool(self.last_target.stop)

            if not math.isfinite(target_x):
                target_x = None
            if not math.isfinite(target_y):
                target_y = None
            if not math.isfinite(desired_speed):
                desired_speed = None

        data = {
            'controller_reason': str(reason),
            'target_age_s': self.target_age(),
            'target_x': target_x,
            'target_y': target_y,
            'target_stop': target_stop,
            'desired_speed': desired_speed,
            'heading_error_deg': (
                None if heading_error is None else math.degrees(heading_error)
            ),
            'forward_angle_limit_deg': self.forward_angle_limit_deg,
            'forward_allowed': bool(forward_allowed),
            'command_linear_x': float(linear_x),
            'command_angular_z': float(angular_z),
        }

        msg = String()
        msg.data = json.dumps(data, separators=(',', ':'), allow_nan=False)
        self.diagnostics_pub.publish(msg)

    def update(self):
        if self.last_target is None or self.last_target_time is None:
            self.publish_command(reason='NO_TARGET')
            return

        age = self.target_age()
        if age is None or age > self.target_timeout:
            self.publish_command(reason='TARGET_STALE')
            return

        if bool(self.last_target.stop):
            self.publish_command(reason='MISSION_STOP_TARGET')
            return

        x = float(self.last_target.target.x)
        y = float(self.last_target.target.y)
        desired_speed = float(self.last_target.desired_speed)

        if not all(math.isfinite(value) for value in (x, y, desired_speed)):
            self.publish_command(reason='TARGET_NONFINITE')
            return

        if x <= 0.0:
            self.publish_command(reason='TARGET_NOT_AHEAD')
            return

        heading_error = math.atan2(y, x)
        yaw_rate = max(
            -self.max_yaw_rate,
            min(self.max_yaw_rate, self.yaw_kp * heading_error),
        )

        forward_limit = math.radians(self.forward_angle_limit_deg)
        forward_allowed = abs(heading_error) <= forward_limit
        forward = (
            max(0.0, min(self.max_forward_speed, desired_speed))
            if forward_allowed
            else 0.0
        )

        self.publish_command(
            linear_x=forward,
            angular_z=yaw_rate,
            reason=(
                'TARGET_TRACKING'
                if forward_allowed
                else 'TARGET_OUTSIDE_FORWARD_ANGLE'
            ),
            heading_error=heading_error,
            forward_allowed=forward_allowed,
        )


def main(args=None):
    rclpy.init(args=args)
    node = TargetController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_command(reason='SHUTDOWN')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
