#!/usr/bin/env python3
"""
RobotX 2026 UAV Proof-of-Readiness route runner.

Safety behavior:
- Launching this node DOES NOT arm or move the aircraft.
- Flight starts only after an explicit /por/start Trigger call.
- The aircraft must start DISARMED with MAVROS connected and a valid local pose.
- /por/abort requests LAND.
- Route geometry is expressed relative to the aircraft heading at /por/start.
  Point the aircraft AWAY from the pilot before starting.

This node is intentionally limited to the POR flight patterns. It is not a
replacement for the team's normal autonomy/safety stack.
"""

import math
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandLong, CommandTOL, SetMode
from std_srvs.srv import Trigger


class PorRouteRunner(Node):
    def __init__(self):
        super().__init__('por_route_runner')

        # Route / flight parameters.
        self.declare_parameter('route_name', 'element1_square')
        self.declare_parameter('route_points', [5.0, 5.0, 15.0, 5.0, 15.0, -5.0, 5.0, -5.0])
        self.declare_parameter('takeoff_altitude_m', 3.5)
        self.declare_parameter('waypoint_tolerance_m', 0.75)
        self.declare_parameter('altitude_tolerance_m', 0.50)
        self.declare_parameter('waypoint_hold_s', 1.0)
        self.declare_parameter('takeoff_timeout_s', 25.0)
        self.declare_parameter('abort_mode', 'LAND')
        self.declare_parameter('force_arm', False)

        # Current MAVROS ROS 2 namespace seen on this aircraft. These remain
        # parameters so they can be overridden without changing code.
        self.declare_parameter('state_topic', '/mavros/mavros/state')
        self.declare_parameter('local_pose_topic', '/mavros/mavros/local_position/pose')
        self.declare_parameter('setpoint_topic', '/mavros/mavros/setpoint_position/local')
        self.declare_parameter('arming_service', '/mavros/mavros/cmd/arming')
        self.declare_parameter('set_mode_service', '/mavros/mavros/set_mode')
        self.declare_parameter('takeoff_service', '/mavros/mavros/cmd/takeoff')
        self.declare_parameter('command_service', '/mavros/mavros/cmd/command')

        self.route_name = str(self.get_parameter('route_name').value)
        raw_points = list(self.get_parameter('route_points').value)
        if len(raw_points) < 2 or len(raw_points) % 2 != 0:
            raise ValueError('route_points must be [forward_m, right_m, ...].')
        self.route_body = [
            (float(raw_points[i]), float(raw_points[i + 1]))
            for i in range(0, len(raw_points), 2)
        ]

        self.altitude = float(self.get_parameter('takeoff_altitude_m').value)
        self.wp_tol = float(self.get_parameter('waypoint_tolerance_m').value)
        self.alt_tol = float(self.get_parameter('altitude_tolerance_m').value)
        self.hold_s = float(self.get_parameter('waypoint_hold_s').value)
        self.takeoff_timeout_s = float(self.get_parameter('takeoff_timeout_s').value)
        self.abort_mode = str(self.get_parameter('abort_mode').value).upper()
        self.force_arm = bool(self.get_parameter('force_arm').value)

        state_topic = str(self.get_parameter('state_topic').value)
        local_pose_topic = str(self.get_parameter('local_pose_topic').value)
        setpoint_topic = str(self.get_parameter('setpoint_topic').value)
        arming_service = str(self.get_parameter('arming_service').value)
        set_mode_service = str(self.get_parameter('set_mode_service').value)
        takeoff_service = str(self.get_parameter('takeoff_service').value)
        command_service = str(self.get_parameter('command_service').value)

        self.state: Optional[State] = None
        self.pose: Optional[PoseStamped] = None

        self.create_subscription(State, state_topic, self._state_cb, 10)
        self.create_subscription(PoseStamped, local_pose_topic, self._pose_cb, qos_profile_sensor_data)
        self.setpoint_pub = self.create_publisher(PoseStamped, setpoint_topic, 10)

        self.arm_client = self.create_client(CommandBool, arming_service)
        self.mode_client = self.create_client(SetMode, set_mode_service)
        self.takeoff_client = self.create_client(CommandTOL, takeoff_service)
        self.command_client = self.create_client(CommandLong, command_service)

        self.create_service(Trigger, '/por/start', self._start_cb)
        self.create_service(Trigger, '/por/abort', self._abort_cb)

        self.timer = self.create_timer(0.10, self._tick)

        self.stage = 'IDLE'
        self.origin: Optional[PoseStamped] = None
        self.initial_yaw = 0.0
        self.route_world = []
        self.wp_index = 0
        self.target: Optional[PoseStamped] = None
        self.arrival_since_ns: Optional[int] = None
        self.takeoff_start_ns: Optional[int] = None

        self.pending_future = None
        self.pending_kind = None
        self.pending_next = None
        self.force_arm_start_ns = None
        self.force_arm_next_stage = None

        self.get_logger().info(
            f'POR runner ready: {self.route_name}. '
            'Launch is SAFE/IDLE; call /por/start to begin flight.'
        )

    def _state_cb(self, msg: State):
        self.state = msg

    def _pose_cb(self, msg: PoseStamped):
        self.pose = msg

    @staticmethod
    def _yaw_from_quaternion(q):
        # Standard ENU yaw from quaternion.
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _start_cb(self, request, response):
        del request

        if self.stage not in ('IDLE', 'DONE', 'FAILED'):
            response.success = False
            response.message = f'Route already active: stage={self.stage}'
            return response

        if self.state is None or not self.state.connected:
            response.success = False
            response.message = 'MAVROS/FCU is not connected.'
            return response

        if self.pose is None:
            response.success = False
            response.message = 'No local-position pose has been received.'
            return response

        if self.state.armed:
            response.success = False
            response.message = 'Start rejected: aircraft must be DISARMED.'
            return response

        self.origin = PoseStamped()
        self.origin.header = self.pose.header
        self.origin.pose = self.pose.pose
        self.initial_yaw = self._yaw_from_quaternion(self.pose.pose.orientation)

        # Convert body-relative route points (forward, right) into MAVROS ENU.
        # ENU yaw=0: forward=east; body-right=south.
        c = math.cos(self.initial_yaw)
        s = math.sin(self.initial_yaw)
        self.route_world = []
        for forward_m, right_m in self.route_body:
            dx = forward_m * c + right_m * s
            dy = forward_m * s - right_m * c
            self.route_world.append((
                self.origin.pose.position.x + dx,
                self.origin.pose.position.y + dy,
            ))

        self.wp_index = 0
        self.target = None
        self.arrival_since_ns = None
        self.pending_future = None
        self.pending_kind = None
        self.pending_next = None
        self.stage = 'SET_GUIDED'

        response.success = True
        response.message = (
            f'{self.route_name} accepted. Runner will request GUIDED, arm, '
            f'take off to {self.altitude:.1f} m, fly the route, return, and LAND.'
        )
        self.get_logger().warn(response.message)
        return response

    def _abort_cb(self, request, response):
        del request
        if self.stage in ('IDLE', 'DONE'):
            response.success = False
            response.message = f'No active route to abort (stage={self.stage}).'
            return response

        self.target = None
        self.pending_future = None
        self.pending_kind = None
        self.pending_next = None
        self.stage = 'ABORT'
        response.success = True
        response.message = f'Abort accepted; requesting {self.abort_mode}.'
        self.get_logger().error(response.message)
        return response

    def _fail(self, reason: str):
        self.get_logger().error(f'POR route FAILED: {reason}')
        self.target = None
        self.pending_future = None
        self.pending_kind = None
        self.pending_next = None
        self.stage = 'FAILED'

    def _service_ready(self, client, name: str):
        if client.service_is_ready():
            return True
        self.get_logger().warn(f'Waiting for service: {name}', throttle_duration_sec=2.0)
        return False

    def _call_mode(self, mode: str, next_stage: str):
        if not self._service_ready(self.mode_client, 'set_mode'):
            return
        req = SetMode.Request()
        req.base_mode = 0
        req.custom_mode = mode
        self.pending_future = self.mode_client.call_async(req)
        self.pending_kind = 'mode'
        self.pending_next = next_stage
        self.get_logger().info(f'Requesting flight mode {mode}')

    def _call_arm(self, arm: bool, next_stage: str):
        if arm and self.force_arm:
            if not self._service_ready(self.command_client, 'command'):
                return

            req = CommandLong.Request()
            req.broadcast = False
            req.command = 400
            req.confirmation = 0
            req.param1 = 1.0
            req.param2 = 2989.0
            req.param3 = 0.0
            req.param4 = 0.0
            req.param5 = 0.0
            req.param6 = 0.0
            req.param7 = 0.0

            self.pending_future = self.command_client.call_async(req)
            self.pending_kind = 'force_arm'
            self.pending_next = 'WAIT_FORCE_ARM'
            self.force_arm_next_stage = next_stage
            self.force_arm_start_ns = self.get_clock().now().nanoseconds
            self.get_logger().warn('Requesting FORCE ARM (SITL mode)')
            return

        if not self._service_ready(self.arm_client, 'arming'):
            return

        req = CommandBool.Request()
        req.value = bool(arm)
        self.pending_future = self.arm_client.call_async(req)
        self.pending_kind = 'arm'
        self.pending_next = next_stage
        self.get_logger().info('Requesting ARM' if arm else 'Requesting DISARM')

    def _call_takeoff(self, next_stage: str):
        if not self._service_ready(self.takeoff_client, 'takeoff'):
            return
        req = CommandTOL.Request()
        req.min_pitch = 0.0
        req.yaw = float('nan')
        req.latitude = float('nan')
        req.longitude = float('nan')
        req.altitude = self.altitude
        self.pending_future = self.takeoff_client.call_async(req)
        self.pending_kind = 'takeoff'
        self.pending_next = next_stage
        self.takeoff_start_ns = self.get_clock().now().nanoseconds
        self.get_logger().info(f'Requesting takeoff to {self.altitude:.1f} m AGL')

    def _check_pending(self):
        if self.pending_future is None:
            return False

        if not self.pending_future.done():
            return True

        try:
            result = self.pending_future.result()
        except Exception as exc:
            self._fail(f'service call exception: {exc}')
            return True

        ok = False
        if self.pending_kind == 'mode':
            ok = bool(result.mode_sent)
        elif self.pending_kind == 'force_arm':
            # Confirm forced arming from /state instead of trusting
            # the COMMAND_LONG service response.
            ok = True
        elif self.pending_kind in ('arm', 'takeoff'):
            ok = bool(result.success)

        next_stage = self.pending_next
        kind = self.pending_kind
        self.pending_future = None
        self.pending_kind = None
        self.pending_next = None

        if not ok:
            self._fail(f'{kind} service rejected the request.')
            return True

        if kind == 'force_arm':
            # Start the arm-confirmation timeout only AFTER MAVROS/ArduPilot
            # has returned from the COMMAND_LONG request.
            self.force_arm_start_ns = self.get_clock().now().nanoseconds

        self.stage = next_stage
        self.get_logger().info(f'{kind} accepted -> stage {self.stage}')
        return True

    def _make_target(self, x: float, y: float):
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(self.origin.pose.position.z + self.altitude)
        msg.pose.orientation = self.origin.pose.orientation
        return msg

    def _home_target(self):
        return self._make_target(
            self.origin.pose.position.x,
            self.origin.pose.position.y,
        )

    def _publish_target(self):
        if self.target is None:
            return
        self.target.header.stamp = self.get_clock().now().to_msg()
        self.setpoint_pub.publish(self.target)

    def _at_target(self):
        if self.pose is None or self.target is None:
            return False

        dx = self.pose.pose.position.x - self.target.pose.position.x
        dy = self.pose.pose.position.y - self.target.pose.position.y
        dz = self.pose.pose.position.z - self.target.pose.position.z
        return math.hypot(dx, dy) <= self.wp_tol and abs(dz) <= self.alt_tol

    def _held_at_target(self):
        if not self._at_target():
            self.arrival_since_ns = None
            return False

        now_ns = self.get_clock().now().nanoseconds
        if self.arrival_since_ns is None:
            self.arrival_since_ns = now_ns
            return False

        return (now_ns - self.arrival_since_ns) / 1e9 >= self.hold_s

    def _tick(self):
        if self.stage in ('IDLE', 'DONE', 'FAILED'):
            return

        if self.state is None or not self.state.connected:
            self._fail('lost MAVROS/FCU connection.')
            return

        if self._check_pending():
            return

        if self.stage == 'SET_GUIDED':
            self._call_mode('GUIDED', 'ARM')
            return

        if self.stage == 'ARM':
            self._call_arm(True, 'TAKEOFF')
            return

        if self.stage == 'WAIT_FORCE_ARM':
            if self.state is not None and self.state.armed:
                self.stage = self.force_arm_next_stage
                self.get_logger().info(
                    f'Forced arm confirmed -> stage {self.stage}'
                )
                return

            if self.force_arm_start_ns is not None:
                elapsed = (
                    self.get_clock().now().nanoseconds -
                    self.force_arm_start_ns
                ) / 1e9
                if elapsed > 5.0:
                    self._fail('force-arm command sent but armed state was not confirmed.')
            return

        if self.stage == 'TAKEOFF':
            self._call_takeoff('WAIT_TAKEOFF')
            return

        if self.stage == 'WAIT_TAKEOFF':
            if self.pose is None:
                return

            rel_z = self.pose.pose.position.z - self.origin.pose.position.z
            if rel_z >= self.altitude - self.alt_tol:
                self.target = self._make_target(*self.route_world[0])
                self.arrival_since_ns = None
                self.stage = 'ROUTE'
                self.get_logger().info('Takeoff altitude reached; starting route.')
                return

            if self.takeoff_start_ns is not None:
                elapsed = (self.get_clock().now().nanoseconds - self.takeoff_start_ns) / 1e9
                if elapsed > self.takeoff_timeout_s:
                    self.stage = 'ABORT'
                    self.get_logger().error('Takeoff timeout; requesting LAND.')
            return

        if self.stage == 'ROUTE':
            self._publish_target()
            if self._held_at_target():
                self.get_logger().info(
                    f'Waypoint {self.wp_index + 1}/{len(self.route_world)} reached.'
                )
                self.wp_index += 1
                self.arrival_since_ns = None

                if self.wp_index >= len(self.route_world):
                    self.target = self._home_target()
                    self.stage = 'RETURN'
                    self.get_logger().info('Route complete; returning to takeoff position.')
                else:
                    self.target = self._make_target(*self.route_world[self.wp_index])
            return

        if self.stage == 'RETURN':
            self._publish_target()
            if self._held_at_target():
                self.target = None
                self.arrival_since_ns = None
                self.stage = 'LAND'
                self.get_logger().info('Takeoff position reached; requesting LAND.')
            return

        if self.stage == 'LAND':
            self._call_mode('LAND', 'WAIT_LAND')
            return

        if self.stage == 'ABORT':
            self._call_mode(self.abort_mode, 'WAIT_LAND')
            return

        if self.stage == 'WAIT_LAND':
            # ArduCopter normally auto-disarms after landing.
            if self.state is not None and not self.state.armed:
                self.stage = 'DONE'
                self.get_logger().info(
                    f'{self.route_name} complete; aircraft is disarmed.'
                )
            return


def main(args=None):
    rclpy.init(args=args)
    node = PorRouteRunner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
