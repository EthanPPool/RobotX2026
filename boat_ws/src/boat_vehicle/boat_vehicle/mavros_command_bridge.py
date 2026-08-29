#!/usr/bin/env python3

import copy
import signal
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from mavros_msgs.msg import State
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.signals import SignalHandlerOptions
from std_srvs.srv import SetBool


class MavrosCommandBridge(Node):
    """Safety boundary between autonomy commands and MAVROS.

    Propulsion commands are forwarded only when:
      - software E-stop is cleared
      - autonomy is explicitly enabled
      - MAVROS is connected
      - vehicle is armed
      - ArduRover is in an allowed mode
      - a fresh command exists

    Otherwise a zero velocity command is published.
    """

    def __init__(self):
        super().__init__('mavros_command_bridge')

        self.input_topic = self.declare_parameter(
            'input_topic', '/control/cmd_vel'
        ).value

        self.output_topic = self.declare_parameter(
            'output_topic',
            '/mavros/setpoint_velocity/cmd_vel'
        ).value

        self.state_topic = self.declare_parameter(
            'state_topic', '/mavros/state'
        ).value

        # BOOT-SAFE DEFAULTS
        self.declare_parameter('autonomy_enabled', False)
        self.declare_parameter('software_estop', True)

        self.allowed_modes = list(
            self.declare_parameter(
                'allowed_modes', ['GUIDED']
            ).value
        )

        self.deadman_timeout = float(
            self.declare_parameter(
                'deadman_timeout', 0.35
            ).value
        )

        self.publish_rate = float(
            self.declare_parameter(
                'publish_rate', 20.0
            ).value
        )

        self.max_forward_speed = float(
            self.declare_parameter(
                'max_forward_speed', 0.60
            ).value
        )

        self.max_yaw_rate = float(
            self.declare_parameter(
                'max_yaw_rate', 0.50
            ).value
        )

        self.shutdown_zero_duration = float(
            self.declare_parameter(
                'shutdown_zero_duration', 0.50
            ).value
        )

        self.last_command = None
        self.last_command_time = None
        self.vehicle_state = None

        self.cmd_pub = self.create_publisher(
            TwistStamped,
            self.output_topic,
            10
        )

        self.cmd_sub = self.create_subscription(
            TwistStamped,
            self.input_topic,
            self.command_callback,
            10
        )

        self.state_sub = self.create_subscription(
            State,
            self.state_topic,
            self.state_callback,
            10
        )

        self.estop_srv = self.create_service(
            SetBool,
            '/vehicle/software_estop',
            self.estop_callback
        )

        self.timer = self.create_timer(
            1.0 / max(self.publish_rate, 1.0),
            self.update
        )

        self.get_logger().warn(
            'BOOT INHIBIT ACTIVE: software_estop=True, '
            'autonomy_enabled=False. Propulsion output is ZERO.'
        )

    def command_callback(self, msg: TwistStamped) -> None:
        self.last_command = msg
        self.last_command_time = self.get_clock().now()

    def state_callback(self, msg: State) -> None:
        self.vehicle_state = msg

    def estop_callback(self, request, response):
        active = bool(request.data)

        self.set_parameters([
            Parameter(
                'software_estop',
                Parameter.Type.BOOL,
                active
            )
        ])

        if active:
            # Prevent an old command from resuming after E-stop reset.
            self.last_command = None
            self.last_command_time = None

            # Send zero immediately instead of waiting for timer.
            self.publish_zero()

            response.success = True
            response.message = (
                'SOFTWARE E-STOP ENGAGED: propulsion command forced to zero'
            )

            self.get_logger().error(
                'SOFTWARE E-STOP ENGAGED'
            )

        else:
            # Still requires autonomy_enabled + fresh command + all other
            # authorization conditions before anything can move.
            self.last_command = None
            self.last_command_time = None

            response.success = True
            response.message = (
                'Software E-stop cleared; waiting for fresh authorized command'
            )

            self.get_logger().warn(
                'Software E-stop cleared. '
                'Fresh command still required.'
            )

        return response

    def authorized(self) -> bool:
        if bool(self.get_parameter('software_estop').value):
            return False

        if not bool(self.get_parameter('autonomy_enabled').value):
            return False

        if self.vehicle_state is None:
            return False

        if not self.vehicle_state.connected:
            return False

        if not self.vehicle_state.armed:
            return False

        if self.vehicle_state.mode not in self.allowed_modes:
            return False

        if self.last_command is None or self.last_command_time is None:
            return False

        age = (
            self.get_clock().now() - self.last_command_time
        ).nanoseconds / 1e9

        return age <= self.deadman_timeout

    def make_zero(self) -> TwistStamped:
        out = TwistStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'base_link'

        out.twist.linear.x = 0.0
        out.twist.linear.y = 0.0
        out.twist.linear.z = 0.0

        out.twist.angular.x = 0.0
        out.twist.angular.y = 0.0
        out.twist.angular.z = 0.0

        return out

    def publish_zero(self) -> None:
        self.cmd_pub.publish(self.make_zero())

    def publish_zero_burst(self) -> None:
        """Send repeated zero commands immediately before shutdown."""
        self.last_command = None
        self.last_command_time = None

        duration = max(self.shutdown_zero_duration, 0.0)
        period = 1.0 / max(self.publish_rate, 20.0)

        self.get_logger().warn(
            f'SHUTDOWN: publishing ZERO velocity for {duration:.2f} s'
        )

        end_time = time.monotonic() + duration

        while time.monotonic() < end_time:
            self.publish_zero()
            time.sleep(period)

        # One final zero.
        self.publish_zero()

    def update(self) -> None:
        if not self.authorized():
            self.publish_zero()
            return

        out = copy.deepcopy(self.last_command)
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'base_link'

        out.twist.linear.x = max(
            -self.max_forward_speed,
            min(
                self.max_forward_speed,
                float(out.twist.linear.x)
            )
        )

        out.twist.linear.y = 0.0
        out.twist.linear.z = 0.0

        out.twist.angular.x = 0.0
        out.twist.angular.y = 0.0

        out.twist.angular.z = max(
            -self.max_yaw_rate,
            min(
                self.max_yaw_rate,
                float(out.twist.angular.z)
            )
        )

        self.cmd_pub.publish(out)


def main(args=None):
    stop_requested = False

    # We handle SIGINT/SIGTERM ourselves so the ROS context remains alive
    # long enough to transmit the shutdown-zero burst.
    rclpy.init(
        args=args,
        signal_handler_options=SignalHandlerOptions.NO
    )

    node = MavrosCommandBridge()

    def request_shutdown(signum, frame):
        nonlocal stop_requested
        stop_requested = True

    signal.signal(signal.SIGINT, request_shutdown)
    signal.signal(signal.SIGTERM, request_shutdown)

    try:
        while rclpy.ok() and not stop_requested:
            rclpy.spin_once(node, timeout_sec=0.10)

    finally:
        try:
            node.publish_zero_burst()
        except Exception as exc:
            node.get_logger().error(
                f'Failed while publishing shutdown zero burst: {exc}'
            )

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
