#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_srvs.srv import SetBool

from boat_interfaces.msg import Gate


class SimpleGateFollower(Node):
    """Fail-stop controller that slowly drives toward a confirmed gate center."""

    def __init__(self):
        super().__init__('simple_gate_follower')

        self.gate_topic = self.declare_parameter(
            'gate_topic',
            '/perception/gate'
        ).value

        self.cmd_topic = self.declare_parameter(
            'cmd_topic',
            '/control/cmd_vel'
        ).value

        # Motion is DISABLED by default.
        self.declare_parameter('enabled', False)

        self.min_gate_confidence = float(
            self.declare_parameter(
                'min_gate_confidence',
                0.75
            ).value
        )

        self.gate_timeout = float(
            self.declare_parameter(
                'gate_timeout',
                0.30
            ).value
        )

        self.forward_speed = float(
            self.declare_parameter(
                'forward_speed',
                0.12
            ).value
        )

        self.yaw_kp = float(
            self.declare_parameter(
                'yaw_kp',
                0.60
            ).value
        )

        self.max_yaw_rate = float(
            self.declare_parameter(
                'max_yaw_rate',
                0.12
            ).value
        )

        self.forward_enable_angle_deg = float(
            self.declare_parameter(
                'forward_enable_angle_deg',
                20.0
            ).value
        )

        self.forward_enable_angle = math.radians(
            self.forward_enable_angle_deg
        )

        self.min_gate_x = float(
            self.declare_parameter(
                'min_gate_x',
                0.50
            ).value
        )

        self.last_gate = None
        self.last_gate_time = None

        self.cmd_pub = self.create_publisher(
            TwistStamped,
            self.cmd_topic,
            10
        )

        self.gate_sub = self.create_subscription(
            Gate,
            self.gate_topic,
            self.gate_callback,
            10
        )

        self.enable_srv = self.create_service(
            SetBool,
            '/control/set_enabled',
            self.enable_callback
        )

        # 20 Hz command/deadman loop.
        self.timer = self.create_timer(
            0.05,
            self.update
        )

        self.get_logger().info(
            'Simple gate follower started DISABLED. '
            'No valid fresh gate = zero motion.'
        )

    def gate_callback(self, msg: Gate) -> None:
        """Accept only a finite, sufficiently confident gate in front of the boat."""

        x = float(msg.center.x)
        y = float(msg.center.y)
        confidence = float(msg.confidence)

        valid = (
            math.isfinite(x)
            and math.isfinite(y)
            and math.isfinite(confidence)
            and x >= self.min_gate_x
            and confidence >= self.min_gate_confidence
        )

        if not valid:
            self.last_gate = None
            self.last_gate_time = None
            return

        self.last_gate = msg
        self.last_gate_time = self.get_clock().now()

    def enable_callback(self, request, response):
        enabled = bool(request.data)

        self.set_parameters([
            Parameter(
                'enabled',
                Parameter.Type.BOOL,
                enabled
            )
        ])

        # Every enable/disable transition starts from zero.
        # This prevents an old gate detection from immediately
        # producing motion after enabling.
        self.last_gate = None
        self.last_gate_time = None

        self.publish_zero()

        if enabled:
            response.success = True
            response.message = (
                'Simple gate follower enabled; '
                'waiting for fresh valid gate'
            )

            self.get_logger().warn(
                'GATE FOLLOWER ENABLED: '
                'fresh valid gate required before motion'
            )

        else:
            response.success = True
            response.message = (
                'Simple gate follower disabled; '
                'command forced to zero'
            )

            self.get_logger().warn(
                'GATE FOLLOWER DISABLED'
            )

        return response

    def publish_zero(self) -> None:
        """Publish an explicit zero velocity command."""

        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'

        cmd.twist.linear.x = 0.0
        cmd.twist.linear.y = 0.0
        cmd.twist.linear.z = 0.0

        cmd.twist.angular.x = 0.0
        cmd.twist.angular.y = 0.0
        cmd.twist.angular.z = 0.0

        self.cmd_pub.publish(cmd)

    def update(self) -> None:
        # Read enabled every cycle so it can be changed
        # dynamically through /control/set_enabled.
        enabled = bool(
            self.get_parameter('enabled').value
        )

        # Rule 1:
        # Explicit enable is required before ANY motion is possible.
        if not enabled:
            self.publish_zero()
            return

        # Rule 2:
        # No valid gate = no movement.
        if (
            self.last_gate is None
            or self.last_gate_time is None
        ):
            self.publish_zero()
            return

        # Rule 3:
        # Stale gate = immediate stop.
        age = (
            self.get_clock().now()
            - self.last_gate_time
        ).nanoseconds / 1e9

        if age > self.gate_timeout:
            self.last_gate = None
            self.last_gate_time = None
            self.publish_zero()
            return

        x = float(self.last_gate.center.x)
        y = float(self.last_gate.center.y)

        # Extra defensive validation.
        if (
            not math.isfinite(x)
            or not math.isfinite(y)
            or x < self.min_gate_x
        ):
            self.publish_zero()
            return

        # Calculate angle from boat bow to gate center.
        #
        # x = forward
        # y = left/right
        heading_error = math.atan2(y, x)

        # Simple proportional steering.
        yaw_rate = self.yaw_kp * heading_error

        # Clamp steering rate.
        yaw_rate = max(
            -self.max_yaw_rate,
            min(
                self.max_yaw_rate,
                yaw_rate
            )
        )

        # Default is always zero forward speed.
        forward = 0.0

        # Only move forward when the gate is reasonably
        # centered in front of the boat.
        if abs(heading_error) <= self.forward_enable_angle:
            forward = self.forward_speed

        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'

        cmd.twist.linear.x = float(forward)
        cmd.twist.linear.y = 0.0
        cmd.twist.linear.z = 0.0

        cmd.twist.angular.x = 0.0
        cmd.twist.angular.y = 0.0
        cmd.twist.angular.z = float(yaw_rate)

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)

    node = SimpleGateFollower()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()