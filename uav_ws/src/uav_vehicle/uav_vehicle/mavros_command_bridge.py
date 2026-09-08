#!/usr/bin/env python3
import math
from typing import Optional

import rclpy
from geometry_msgs.msg import TwistStamped
from mavros_msgs.msg import State
from rclpy.node import Node
from uav_interfaces.msg import AutonomyStatus, SafetyStatus
from uav_interfaces.srv import SetAutonomy


class MavrosCommandBridge(Node):
    """Final software authorization gate before MAVROS velocity setpoints."""

    def __init__(self) -> None:
        super().__init__('mavros_command_bridge')

        self.declare_parameter('input_topic', '/control/cmd_vel')
        self.declare_parameter('output_topic', '/mavros/setpoint_velocity/cmd_vel')
        self.declare_parameter('state_topic', '/mavros/state')
        self.declare_parameter('status_topic', '/vehicle/autonomy_status')
        self.declare_parameter('safety_status_topic', '/vehicle/safety_status')
        self.declare_parameter('autonomy_enabled', False)
        self.declare_parameter('allowed_modes', ['GUIDED'])
        self.declare_parameter('require_safety_supervisor', True)
        self.declare_parameter('safety_timeout', 0.75)
        self.declare_parameter('deadman_timeout', 0.30)
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('max_horizontal_speed', 1.50)
        self.declare_parameter('max_vertical_speed', 0.75)
        self.declare_parameter('max_yaw_rate', 0.75)

        self.input_topic = str(self.get_parameter('input_topic').value)
        self.output_topic = str(self.get_parameter('output_topic').value)
        self.state_topic = str(self.get_parameter('state_topic').value)
        self.status_topic = str(self.get_parameter('status_topic').value)
        self.safety_status_topic = str(self.get_parameter('safety_status_topic').value)
        self.autonomy_enabled = bool(self.get_parameter('autonomy_enabled').value)
        self.allowed_modes = [str(m).upper() for m in self.get_parameter('allowed_modes').value]
        self.require_safety_supervisor = bool(
            self.get_parameter('require_safety_supervisor').value)
        self.safety_timeout = float(self.get_parameter('safety_timeout').value)
        self.deadman_timeout = float(self.get_parameter('deadman_timeout').value)
        self.max_horizontal_speed = float(self.get_parameter('max_horizontal_speed').value)
        self.max_vertical_speed = float(self.get_parameter('max_vertical_speed').value)
        self.max_yaw_rate = float(self.get_parameter('max_yaw_rate').value)

        publish_rate = max(1.0, float(self.get_parameter('publish_rate').value))

        self.have_state = False
        self.state = State()
        self.have_safety = False
        self.safety = SafetyStatus()
        self.last_safety_rx_time = None
        self.last_cmd: Optional[TwistStamped] = None
        self.last_cmd_rx_time = None
        self.was_authorized = False
        self.zero_hold_active = False

        self.cmd_sub = self.create_subscription(
            TwistStamped, self.input_topic, self._cmd_cb, 10)
        self.state_sub = self.create_subscription(
            State, self.state_topic, self._state_cb, 10)
        self.safety_sub = self.create_subscription(
            SafetyStatus, self.safety_status_topic, self._safety_cb, 10)
        self.cmd_pub = self.create_publisher(TwistStamped, self.output_topic, 10)
        self.status_pub = self.create_publisher(AutonomyStatus, self.status_topic, 10)
        self.autonomy_srv = self.create_service(
            SetAutonomy, '/vehicle/set_autonomy', self._set_autonomy_cb)

        self.timer = self.create_timer(1.0 / publish_rate, self._timer_cb)
        self.get_logger().info(
            f'Bridge ready: {self.input_topic} -> {self.output_topic}; '
            f'autonomy_enabled={self.autonomy_enabled}; '
            f'require_safety_supervisor={self.require_safety_supervisor}')

    def _cmd_cb(self, msg: TwistStamped) -> None:
        self.last_cmd = msg
        self.last_cmd_rx_time = self.get_clock().now()

    def _state_cb(self, msg: State) -> None:
        self.have_state = True
        self.state = msg

    def _safety_cb(self, msg: SafetyStatus) -> None:
        self.have_safety = True
        self.safety = msg
        self.last_safety_rx_time = self.get_clock().now()

    def _command_fresh(self) -> bool:
        if self.last_cmd_rx_time is None:
            return False
        age = (self.get_clock().now() - self.last_cmd_rx_time).nanoseconds / 1e9
        return age <= self.deadman_timeout

    def _safety_fresh(self) -> bool:
        if self.last_safety_rx_time is None:
            return False
        age = (self.get_clock().now() - self.last_safety_rx_time).nanoseconds / 1e9
        return age <= self.safety_timeout

    def _safety_ok(self):
        if not self.require_safety_supervisor:
            return True, 'safety supervisor bypassed by configuration'
        if not self.have_safety:
            return False, 'no safety status received'
        if not self._safety_fresh():
            return False, 'safety status stale'
        if self.safety.failsafe_latched:
            return False, f'safety FAILSAFE: {self.safety.reason}'
        if not self.safety.ready:
            return False, f'safety not ready: {self.safety.reason}'
        return True, 'safety ready'

    def _authorization(self):
        fresh = self._command_fresh()
        if not self.autonomy_enabled:
            return False, fresh, 'autonomy disabled'
        if not self.have_state:
            return False, fresh, 'no MAVROS state received'
        if not self.state.connected:
            return False, fresh, 'MAVROS disconnected'
        if not self.state.armed:
            return False, fresh, 'vehicle not armed'
        if str(self.state.mode).upper() not in self.allowed_modes:
            return False, fresh, f'mode {self.state.mode!r} not allowed'
        safety_ok, safety_reason = self._safety_ok()
        if not safety_ok:
            return False, fresh, safety_reason
        if not fresh:
            return False, fresh, 'command stale / deadman active'
        return True, fresh, 'authorized'

    def _set_autonomy_cb(self, request, response):
        if not request.enabled:
            self.autonomy_enabled = False
            self.zero_hold_active = False
            response.success = True
            response.message = 'Autonomy disabled.'
            self.get_logger().warn(response.message)
            return response

        if not self.have_state:
            response.success = False
            response.message = 'Cannot enable autonomy: no MAVROS state received.'
            return response
        if not self.state.connected:
            response.success = False
            response.message = 'Cannot enable autonomy: MAVROS is disconnected.'
            return response
        if not self.state.armed:
            response.success = False
            response.message = 'Cannot enable autonomy: vehicle is not armed.'
            return response
        if str(self.state.mode).upper() not in self.allowed_modes:
            response.success = False
            response.message = (
                f'Cannot enable autonomy: mode {self.state.mode!r} is not in '
                f'{self.allowed_modes}.')
            return response

        safety_ok, safety_reason = self._safety_ok()
        if not safety_ok:
            response.success = False
            response.message = f'Cannot enable autonomy: {safety_reason}.'
            return response

        self.autonomy_enabled = True
        response.success = True
        response.message = (
            'Autonomy enabled; safety supervisor and deadman remain mandatory.')
        self.get_logger().warn(response.message)
        return response

    @staticmethod
    def _clamp(value: float, limit: float) -> float:
        if not math.isfinite(value):
            return 0.0
        return max(-limit, min(limit, value))

    def _safe_command(self) -> TwistStamped:
        out = TwistStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'base_link'
        if self.last_cmd is None:
            return out

        x = float(self.last_cmd.twist.linear.x)
        y = float(self.last_cmd.twist.linear.y)
        z = float(self.last_cmd.twist.linear.z)
        norm_xy = math.hypot(x, y)
        if norm_xy > self.max_horizontal_speed and norm_xy > 0.0:
            scale = self.max_horizontal_speed / norm_xy
            x *= scale
            y *= scale

        out.twist.linear.x = self._clamp(x, self.max_horizontal_speed)
        out.twist.linear.y = self._clamp(y, self.max_horizontal_speed)
        out.twist.linear.z = self._clamp(z, self.max_vertical_speed)
        out.twist.angular.z = self._clamp(
            float(self.last_cmd.twist.angular.z), self.max_yaw_rate)
        return out

    def _zero_command(self) -> TwistStamped:
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        return msg

    def _publish_status(self, authorized: bool, fresh: bool, reason: str) -> None:
        msg = AutonomyStatus()
        msg.stamp = self.get_clock().now().to_msg()
        msg.autonomy_enabled = self.autonomy_enabled
        msg.mavros_state_received = self.have_state
        msg.mavros_connected = bool(self.state.connected) if self.have_state else False
        msg.armed = bool(self.state.armed) if self.have_state else False
        msg.mode = str(self.state.mode) if self.have_state else ''
        msg.command_fresh = fresh
        msg.authorized = authorized
        msg.reason = reason
        self.status_pub.publish(msg)

    def _timer_cb(self) -> None:
        authorized, fresh, reason = self._authorization()
        self._publish_status(authorized, fresh, reason)

        if authorized:
            self.zero_hold_active = False
            self.cmd_pub.publish(self._safe_command())
        else:
            if self.was_authorized:
                self.zero_hold_active = True
            if self.zero_hold_active and self.autonomy_enabled and self.have_state and self.state.connected:
                # Continue a zero-velocity stream after authorization is lost.
                # A later version will replace selected faults with explicit
                # ArduCopter HOLD/RTL/LAND actions.
                self.cmd_pub.publish(self._zero_command())

        self.was_authorized = authorized


def main(args=None):
    rclpy.init(args=args)
    node = MavrosCommandBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
