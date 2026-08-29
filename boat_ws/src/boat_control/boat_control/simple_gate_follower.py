#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_srvs.srv import SetBool, Trigger

from boat_interfaces.msg import Gate


class SimpleGateFollower(Node):
    """Simple single-gate mission controller.

    The node remains running continuously.

    Mission states:

        SEARCH / APPROACH
            Track the confirmed gate and drive toward its center.

        PASSING
            Entered once the gate center becomes sufficiently close.

        COMPLETE
            Latched once the previously-close gate disappears or the
            detector jumps to a substantially farther gate.

    COMPLETE remains latched until /control/reset_mission is called.
    """

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

        # Keep the controller running by default.
        self.declare_parameter('enabled', True)

        # Exposed for diagnostics/dashboard.
        self.declare_parameter('mission_complete', False)

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

        # Once the gate has been this close, we know that we are
        # actually entering/passing the intended gate.
        self.pass_arm_distance = float(
            self.declare_parameter(
                'pass_arm_distance',
                1.50
            ).value
        )

        # Once passage has been armed, losing the gate for this long
        # is interpreted as having passed through it.
        self.pass_loss_timeout = float(
            self.declare_parameter(
                'pass_loss_timeout',
                0.40
            ).value
        )

        # If we were close to the gate and the detector suddenly
        # chooses a much farther pair of objects, treat that as the
        # original gate having passed behind the boat.
        self.pass_jump_distance = float(
            self.declare_parameter(
                'pass_jump_distance',
                1.00
            ).value
        )

        self.last_gate = None
        self.last_gate_time = None

        self.passage_armed = False
        self.closest_gate_x = math.inf

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

        # Kept for compatibility, but normal operation should leave
        # this node enabled continuously.
        self.enable_srv = self.create_service(
            SetBool,
            '/control/set_enabled',
            self.enable_callback
        )

        self.reset_srv = self.create_service(
            Trigger,
            '/control/reset_mission',
            self.reset_callback
        )

        self.timer = self.create_timer(
            0.05,
            self.update
        )

        self.get_logger().info(
            'Simple gate follower started ENABLED. '
            'Waiting for a confirmed gate.'
        )

    def mission_is_complete(self):
        return bool(
            self.get_parameter(
                'mission_complete'
            ).value
        )

    def set_mission_complete(self, complete):
        self.set_parameters([
            Parameter(
                'mission_complete',
                Parameter.Type.BOOL,
                bool(complete)
            )
        ])

    def gate_callback(self, msg: Gate) -> None:
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

        # Do not destroy the previous valid detection immediately.
        # The update loop handles freshness/timeouts.
        if not valid:
            return

        if self.mission_is_complete():
            return

        # If passage has already been armed and we suddenly detect a
        # gate substantially farther away, it is probably a new pair
        # after the original gate moved behind the boat.
        if self.passage_armed:
            jump_threshold = max(
                self.pass_arm_distance + 0.25,
                self.closest_gate_x
                + self.pass_jump_distance
            )

            if x >= jump_threshold:
                self.complete_mission(
                    'gate passed: detector jumped '
                    f'from {self.closest_gate_x:.2f} m '
                    f'to {x:.2f} m'
                )
                return

        self.last_gate = msg
        self.last_gate_time = self.get_clock().now()

        if x < self.closest_gate_x:
            self.closest_gate_x = x

        if (
            not self.passage_armed
            and x <= self.pass_arm_distance
        ):
            self.passage_armed = True
            self.closest_gate_x = x

            self.get_logger().warn(
                'GATE PASSAGE ARMED: '
                f'gate center is {x:.2f} m ahead'
            )

    def enable_callback(self, request, response):
        requested = bool(request.data)

        currently_enabled = bool(
            self.get_parameter('enabled').value
        )

        # Repeating "enable" must not wipe a valid gate.
        if requested == currently_enabled:
            response.success = True
            response.message = (
                'Gate follower already '
                + ('enabled' if requested else 'disabled')
            )
            return response

        self.set_parameters([
            Parameter(
                'enabled',
                Parameter.Type.BOOL,
                requested
            )
        ])

        if requested:
            # Do not reset mission_complete here.
            # A completed mission requires an explicit reset.
            self.last_gate = None
            self.last_gate_time = None

            response.success = True

            if self.mission_is_complete():
                response.message = (
                    'Follower enabled, but mission COMPLETE '
                    'remains latched; reset required'
                )
            else:
                response.message = (
                    'Gate follower enabled'
                )

            self.get_logger().warn(
                'GATE FOLLOWER ENABLED'
            )

        else:
            self.last_gate = None
            self.last_gate_time = None
            self.publish_zero()

            response.success = True
            response.message = (
                'Gate follower disabled'
            )

            self.get_logger().warn(
                'GATE FOLLOWER DISABLED'
            )

        return response

    def reset_callback(self, request, response):
        self.set_mission_complete(False)

        self.last_gate = None
        self.last_gate_time = None

        self.passage_armed = False
        self.closest_gate_x = math.inf

        self.publish_zero()

        response.success = True
        response.message = (
            'Gate mission reset; waiting for fresh gate'
        )

        self.get_logger().warn(
            'GATE MISSION RESET'
        )

        return response

    def complete_mission(self, reason):
        if self.mission_is_complete():
            return

        self.set_mission_complete(True)

        self.last_gate = None
        self.last_gate_time = None

        self.passage_armed = False

        # A zero command is intentional here. The command bridge
        # interprets all-zero as STOP and transitions ArduRover to HOLD.
        self.publish_zero()

        self.get_logger().error(
            'GATE MISSION COMPLETE: '
            f'{reason}. STOP requested.'
        )

    def publish_zero(self):
        cmd = TwistStamped()

        cmd.header.stamp = (
            self.get_clock().now().to_msg()
        )

        cmd.header.frame_id = 'base_link'

        cmd.twist.linear.x = 0.0
        cmd.twist.linear.y = 0.0
        cmd.twist.linear.z = 0.0

        cmd.twist.angular.x = 0.0
        cmd.twist.angular.y = 0.0
        cmd.twist.angular.z = 0.0

        self.cmd_pub.publish(cmd)

    def update(self):
        enabled = bool(
            self.get_parameter('enabled').value
        )

        if not enabled:
            self.publish_zero()
            return

        # Completed missions remain stopped indefinitely until
        # /control/reset_mission is called.
        if self.mission_is_complete():
            self.publish_zero()
            return

        if (
            self.last_gate is None
            or self.last_gate_time is None
        ):
            self.publish_zero()
            return

        age = (
            self.get_clock().now()
            - self.last_gate_time
        ).nanoseconds / 1e9

        if age > self.gate_timeout:
            # If we were already very close to the gate and then lost
            # it, that is the expected signature of passing through it.
            if (
                self.passage_armed
                and age >= self.pass_loss_timeout
            ):
                self.complete_mission(
                    'previously-close gate '
                    f'lost for {age:.2f} s'
                )
                return

            self.publish_zero()
            return

        x = float(self.last_gate.center.x)
        y = float(self.last_gate.center.y)

        if (
            not math.isfinite(x)
            or not math.isfinite(y)
            or x < self.min_gate_x
        ):
            self.publish_zero()
            return

        if x < self.closest_gate_x:
            self.closest_gate_x = x

        if (
            not self.passage_armed
            and x <= self.pass_arm_distance
        ):
            self.passage_armed = True
            self.closest_gate_x = x

            self.get_logger().warn(
                'GATE PASSAGE ARMED: '
                f'gate center is {x:.2f} m ahead'
            )

        heading_error = math.atan2(y, x)

        yaw_rate = (
            self.yaw_kp
            * heading_error
        )

        yaw_rate = max(
            -self.max_yaw_rate,
            min(
                self.max_yaw_rate,
                yaw_rate
            )
        )

        forward = 0.0

        if (
            abs(heading_error)
            <= self.forward_enable_angle
        ):
            forward = self.forward_speed

        cmd = TwistStamped()

        cmd.header.stamp = (
            self.get_clock().now().to_msg()
        )

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
