#!/usr/bin/env python3
import math

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import BatteryState, NavSatFix, NavSatStatus


class MockMavros(Node):
    """Software-only MAVROS stand-in with telemetry and command services."""

    def __init__(self):
        super().__init__('mock_mavros')
        self.declare_parameter('state_topic', '/mock_mavros/state')
        self.declare_parameter('setpoint_topic', '/mock_mavros/setpoint_velocity/cmd_vel')
        self.declare_parameter('battery_topic', '/mock_mavros/battery')
        self.declare_parameter('gps_topic', '/mock_mavros/global_position/raw/fix')
        self.declare_parameter('local_position_topic', '/mock_mavros/local_position/pose')
        self.declare_parameter('arming_service', '/mock_mavros/cmd/arming')
        self.declare_parameter('set_mode_service', '/mock_mavros/set_mode')
        self.declare_parameter('takeoff_service', '/mock_mavros/cmd/takeoff')
        self.declare_parameter('land_service', '/mock_mavros/cmd/land')

        self.declare_parameter('connected', True)
        self.declare_parameter('armed', False)
        self.declare_parameter('mode', 'GUIDED')
        self.declare_parameter('battery_percentage', 0.80)
        self.declare_parameter('battery_voltage', 15.2)
        self.declare_parameter('gps_valid', True)
        self.declare_parameter('local_position_valid', True)
        self.declare_parameter('latitude', 30.2241)
        self.declare_parameter('longitude', -92.0198)
        self.declare_parameter('altitude', 0.0)
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter(
            'supported_modes', ['GUIDED', 'LAND', 'RTL', 'LOITER', 'STABILIZE'])

        self.state_pub = self.create_publisher(
            State, str(self.get_parameter('state_topic').value), 10)
        self.battery_pub = self.create_publisher(
            BatteryState, str(self.get_parameter('battery_topic').value), 10)
        self.gps_pub = self.create_publisher(
            NavSatFix, str(self.get_parameter('gps_topic').value), 10)
        self.local_position_pub = self.create_publisher(
            PoseStamped, str(self.get_parameter('local_position_topic').value), 10)
        self.setpoint_sub = self.create_subscription(
            TwistStamped, str(self.get_parameter('setpoint_topic').value),
            self._setpoint_cb, 10)

        self.create_service(
            CommandBool, str(self.get_parameter('arming_service').value), self._arming_cb)
        self.create_service(
            SetMode, str(self.get_parameter('set_mode_service').value), self._set_mode_cb)
        self.create_service(
            CommandTOL, str(self.get_parameter('takeoff_service').value), self._takeoff_cb)
        self.create_service(
            CommandTOL, str(self.get_parameter('land_service').value), self._land_cb)

        self.last_log_ns = 0
        rate = max(1.0, float(self.get_parameter('publish_rate').value))
        self.timer = self.create_timer(1.0 / rate, self._publish_mock_telemetry)
        self.get_logger().warn(
            'MOCK MAVROS v0.3 active: command services affect only simulated parameters.')

    def _set_param(self, name: str, value) -> None:
        current = self.get_parameter(name)
        self.set_parameters([Parameter(name, current.type_, value)])

    def _publish_mock_telemetry(self):
        now = self.get_clock().now().to_msg()

        state = State()
        state.connected = bool(self.get_parameter('connected').value)
        state.armed = bool(self.get_parameter('armed').value)
        state.mode = str(self.get_parameter('mode').value)
        state.guided = state.mode.upper() == 'GUIDED'
        self.state_pub.publish(state)

        battery = BatteryState()
        battery.header.stamp = now
        battery.voltage = float(self.get_parameter('battery_voltage').value)
        battery.percentage = float(self.get_parameter('battery_percentage').value)
        self.battery_pub.publish(battery)

        gps = NavSatFix()
        gps.header.stamp = now
        gps.header.frame_id = 'gps_link'
        gps_valid = bool(self.get_parameter('gps_valid').value)
        gps.status.status = (
            NavSatStatus.STATUS_FIX if gps_valid else NavSatStatus.STATUS_NO_FIX)
        gps.status.service = NavSatStatus.SERVICE_GPS
        gps.latitude = float(self.get_parameter('latitude').value)
        gps.longitude = float(self.get_parameter('longitude').value)
        gps.altitude = float(self.get_parameter('altitude').value)
        self.gps_pub.publish(gps)

        if bool(self.get_parameter('local_position_valid').value):
            pose = PoseStamped()
            pose.header.stamp = now
            pose.header.frame_id = 'map'
            pose.pose.position.z = float(self.get_parameter('altitude').value)
            pose.pose.orientation.w = 1.0
            self.local_position_pub.publish(pose)

    def _arming_cb(self, request, response):
        if not bool(self.get_parameter('connected').value):
            response.success = False; response.result = 1; return response
        self._set_param('armed', bool(request.value))
        response.success = True
        response.result = 0
        self.get_logger().warn(f'MOCK armed set to {bool(request.value)}')
        return response

    def _set_mode_cb(self, request, response):
        if not bool(self.get_parameter('connected').value):
            response.mode_sent = False; return response
        mode = str(request.custom_mode).strip().upper()
        supported = {str(m).upper() for m in self.get_parameter('supported_modes').value}
        if mode not in supported:
            response.mode_sent = False; return response
        self._set_param('mode', mode)
        response.mode_sent = True
        self.get_logger().warn(f'MOCK mode set to {mode}')
        return response

    def _takeoff_cb(self, request, response):
        connected = bool(self.get_parameter('connected').value)
        armed = bool(self.get_parameter('armed').value)
        mode = str(self.get_parameter('mode').value).upper()
        nav_ok = bool(self.get_parameter('gps_valid').value) and bool(
            self.get_parameter('local_position_valid').value)
        altitude = float(request.altitude)
        if not (connected and armed and mode == 'GUIDED' and nav_ok and altitude > 0.0):
            response.success = False; response.result = 4; return response
        self._set_param('altitude', altitude)
        response.success = True; response.result = 0
        self.get_logger().warn(f'MOCK takeoff accepted to {altitude:.1f} m')
        return response

    def _land_cb(self, request, response):
        del request
        if not bool(self.get_parameter('connected').value):
            response.success = False; response.result = 1; return response
        self._set_param('mode', 'LAND')
        self._set_param('altitude', 0.0)
        response.success = True; response.result = 0
        return response

    def _setpoint_cb(self, msg: TwistStamped):
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self.last_log_ns >= 1_000_000_000:
            self.last_log_ns = now_ns
            values = (
                msg.twist.linear.x, msg.twist.linear.y,
                msg.twist.linear.z, msg.twist.angular.z)
            if all(math.isfinite(float(v)) for v in values):
                self.get_logger().info(
                    'Mock received setpoint: '
                    f'vx={msg.twist.linear.x:.2f}, vy={msg.twist.linear.y:.2f}, '
                    f'vz={msg.twist.linear.z:.2f}, yaw_rate={msg.twist.angular.z:.2f}')


def main(args=None):
    rclpy.init(args=args)
    node = MockMavros()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
