#!/usr/bin/env python3
"""
RobotX 2026 UAV Proof-of-Readiness Element #3 runner.

Element #3 (August 7, 2026 Team Handbook):
- Take off / establish low hover.
- Climb out to Point 2, 30-40 m out, at 30-40 m altitude.
- Begin toward Point 3.
- Perform 3-4 stationary pirouettes, re-orient, continue to Point 3.
- Descend approximately 45 degrees toward Point 4 / landing area.
- Establish low hover and land.

Safety behavior:
- Launching this node DOES NOT arm or move the aircraft.
- Flight starts only after explicit /por/start.
- /por/abort requests LAND.
- By default pirouettes start automatically after reaching the midpoint.
- If pirouette_wait_for_trigger=true, the aircraft holds at the midpoint
  until /por/pirouette is called.

Geometry is relative to aircraft heading at /por/start.
Point the aircraft AWAY from the pilot/flight line before starting.
"""

import math
from typing import Optional, List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandLong, CommandTOL, SetMode
from std_srvs.srv import Trigger


class PorElement3Runner(Node):
    def __init__(self):
        super().__init__('por_element3_runner')

        self.declare_parameter('route_name', 'por_element3_pirouette')

        # Handbook ranges are 30-40 m out and 30-40 m altitude.
        # Defaults intentionally sit near the middle of those ranges.
        self.declare_parameter('initial_hover_altitude_m', 3.5)
        self.declare_parameter('high_altitude_m', 35.0)
        self.declare_parameter('final_hover_altitude_m', 2.5)

        # P2/P3 defaults:
        # radial distance from START ~= 32.5 m
        # P2-P3 separation = 30 m
        # P3 -> START horizontal distance ~= 32.5 m
        # high_altitude - final_hover ~= 32.5 m, approximating a 45 deg descent.
        self.declare_parameter('p2_forward_m', 28.8)
        self.declare_parameter('p2_right_m', 15.0)
        self.declare_parameter('p3_forward_m', 28.8)
        self.declare_parameter('p3_right_m', -15.0)

        self.declare_parameter('pirouette_count', 4)
        self.declare_parameter('pirouette_yaw_rate_dps', 30.0)
        self.declare_parameter('pirouette_wait_for_trigger', False)

        self.declare_parameter('descent_steps', 12)
        self.declare_parameter('waypoint_tolerance_m', 1.0)
        self.declare_parameter('altitude_tolerance_m', 0.75)
        self.declare_parameter('waypoint_hold_s', 1.5)
        self.declare_parameter('takeoff_timeout_s', 30.0)
        self.declare_parameter('abort_mode', 'LAND')
        self.declare_parameter('force_arm', False)

        self.declare_parameter('state_topic', '/mavros/state')
        self.declare_parameter('local_pose_topic', '/mavros/local_position/pose')
        self.declare_parameter('setpoint_topic', '/mavros/setpoint_position/local')
        self.declare_parameter('arming_service', '/mavros/cmd/arming')
        self.declare_parameter('set_mode_service', '/mavros/set_mode')
        self.declare_parameter('takeoff_service', '/mavros/cmd/takeoff')
        self.declare_parameter('command_service', '/mavros/cmd/command')

        self.route_name = str(self.get_parameter('route_name').value)
        self.initial_hover = float(self.get_parameter('initial_hover_altitude_m').value)
        self.high_alt = float(self.get_parameter('high_altitude_m').value)
        self.final_hover = float(self.get_parameter('final_hover_altitude_m').value)

        self.p2_body = (
            float(self.get_parameter('p2_forward_m').value),
            float(self.get_parameter('p2_right_m').value),
        )
        self.p3_body = (
            float(self.get_parameter('p3_forward_m').value),
            float(self.get_parameter('p3_right_m').value),
        )

        self.pirouette_count = int(self.get_parameter('pirouette_count').value)
        self.pirouette_rate = math.radians(
            float(self.get_parameter('pirouette_yaw_rate_dps').value)
        )
        self.wait_for_pirouette_trigger = bool(
            self.get_parameter('pirouette_wait_for_trigger').value
        )

        self.descent_steps = max(2, int(self.get_parameter('descent_steps').value))
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
        self.create_service(Trigger, '/por/pirouette', self._pirouette_cb)

        self.timer = self.create_timer(0.10, self._tick)

        self.stage = 'IDLE'
        self.origin: Optional[PoseStamped] = None
        self.initial_yaw = 0.0
        self.p2_world = None
        self.mid_world = None
        self.p3_world = None
        self.descent_targets: List[PoseStamped] = []
        self.descent_index = 0

        self.target: Optional[PoseStamped] = None
        self.arrival_since_ns: Optional[int] = None
        self.takeoff_start_ns: Optional[int] = None

        self.pending_future = None
        self.pending_kind = None
        self.pending_next = None
        self.force_arm_start_ns = None
        self.force_arm_next_stage = None

        self.pirouette_triggered = False
        self.pirouette_start_ns: Optional[int] = None

        self.get_logger().info(
            'POR Element #3 runner ready. SAFE/IDLE; call /por/start to begin.'
        )

    def _state_cb(self, msg: State):
        self.state = msg

    def _pose_cb(self, msg: PoseStamped):
        self.pose = msg

    @staticmethod
    def _yaw_from_quaternion(q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _quat_from_yaw(yaw):
        # Roll=pitch=0 quaternion.
        qz = math.sin(yaw * 0.5)
        qw = math.cos(yaw * 0.5)
        return (0.0, 0.0, qz, qw)

    def _body_to_world(self, forward_m: float, right_m: float) -> Tuple[float, float]:
        c = math.cos(self.initial_yaw)
        s = math.sin(self.initial_yaw)
        dx = forward_m * c + right_m * s
        dy = forward_m * s - right_m * c
        return (
            self.origin.pose.position.x + dx,
            self.origin.pose.position.y + dy,
        )

    def _make_target(self, x: float, y: float, rel_z: float, yaw: Optional[float] = None):
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(self.origin.pose.position.z + rel_z)

        if yaw is None:
            yaw = self.initial_yaw
        qx, qy, qz, qw = self._quat_from_yaw(yaw)
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        return msg

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

        self.p2_world = self._body_to_world(*self.p2_body)
        self.p3_world = self._body_to_world(*self.p3_body)
        self.mid_world = (
            0.5 * (self.p2_world[0] + self.p3_world[0]),
            0.5 * (self.p2_world[1] + self.p3_world[1]),
        )

        # Approximate the prescribed 45-degree descent with a series of local
        # position targets from P3 to the START/END landing area.
        self.descent_targets = []
        x0, y0 = self.p3_world
        x1 = self.origin.pose.position.x
        y1 = self.origin.pose.position.y
        for i in range(1, self.descent_steps + 1):
            a = i / self.descent_steps
            x = x0 + a * (x1 - x0)
            y = y0 + a * (y1 - y0)
            z = self.high_alt + a * (self.final_hover - self.high_alt)
            self.descent_targets.append(
                self._make_target(x, y, z, self.initial_yaw)
            )

        self.target = None
        self.arrival_since_ns = None
        self.takeoff_start_ns = None
        self.pending_future = None
        self.pending_kind = None
        self.pending_next = None
        self.pirouette_triggered = False
        self.pirouette_start_ns = None
        self.descent_index = 0
        self.stage = 'SET_GUIDED'

        response.success = True
        response.message = (
            f'{self.route_name} accepted: GUIDED -> ARM -> takeoff -> climb -> '
            'pirouettes -> descent -> LAND.'
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

    def _pirouette_cb(self, request, response):
        del request
        self.pirouette_triggered = True
        response.success = True
        if self.stage == 'WAIT_PIROUETTE':
            response.message = 'Pirouette trigger accepted.'
        else:
            response.message = (
                f'Pirouette trigger latched; current stage={self.stage}. '
                'It will be used when the midpoint is reached.'
            )
        self.get_logger().warn(response.message)
        return response

    def _fail(self, reason: str):
        self.get_logger().error(f'POR Element #3 FAILED: {reason}')
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
        req.altitude = self.initial_hover
        self.pending_future = self.takeoff_client.call_async(req)
        self.pending_kind = 'takeoff'
        self.pending_next = next_stage
        self.takeoff_start_ns = self.get_clock().now().nanoseconds
        self.get_logger().info(
            f'Requesting takeoff to initial hover {self.initial_hover:.1f} m AGL'
        )

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

    def _start_pirouettes(self):
        self.pirouette_start_ns = self.get_clock().now().nanoseconds
        self.stage = 'PIROUETTE'
        self.get_logger().warn(
            f'Starting {self.pirouette_count} stationary pirouettes at '
            f'{math.degrees(self.pirouette_rate):.1f} deg/s.'
        )

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
            if rel_z >= self.initial_hover - self.alt_tol:
                self.target = self._make_target(
                    self.p2_world[0], self.p2_world[1], self.high_alt, self.initial_yaw
                )
                self.arrival_since_ns = None
                self.stage = 'CLIMB_TO_P2'
                self.get_logger().info('Initial hover reached; climbing/outbound to P2.')
                return

            if self.takeoff_start_ns is not None:
                elapsed = (self.get_clock().now().nanoseconds - self.takeoff_start_ns) / 1e9
                if elapsed > self.takeoff_timeout_s:
                    self.stage = 'ABORT'
                    self.get_logger().error('Takeoff timeout; requesting LAND.')
            return

        if self.stage == 'CLIMB_TO_P2':
            self._publish_target()
            if self._held_at_target():
                self.target = self._make_target(
                    self.mid_world[0], self.mid_world[1], self.high_alt, self.initial_yaw
                )
                self.arrival_since_ns = None
                self.stage = 'TO_PIROUETTE_POINT'
                self.get_logger().info('P2 reached; proceeding toward P3.')
            return

        if self.stage == 'TO_PIROUETTE_POINT':
            self._publish_target()
            if self._held_at_target():
                if self.wait_for_pirouette_trigger and not self.pirouette_triggered:
                    self.stage = 'WAIT_PIROUETTE'
                    self.get_logger().warn(
                        'Holding stationary for assessor call; '
                        'call /por/pirouette to begin.'
                    )
                else:
                    self._start_pirouettes()
            return

        if self.stage == 'WAIT_PIROUETTE':
            self._publish_target()
            if self.pirouette_triggered:
                self._start_pirouettes()
            return

        if self.stage == 'PIROUETTE':
            if self.pirouette_start_ns is None:
                self._start_pirouettes()
                return

            elapsed = (
                self.get_clock().now().nanoseconds - self.pirouette_start_ns
            ) / 1e9
            total_angle = 2.0 * math.pi * self.pirouette_count
            commanded_angle = min(self.pirouette_rate * elapsed, total_angle)
            yaw = self.initial_yaw + commanded_angle

            self.target = self._make_target(
                self.mid_world[0], self.mid_world[1], self.high_alt, yaw
            )
            self._publish_target()

            if commanded_angle >= total_angle:
                self.target = self._make_target(
                    self.mid_world[0], self.mid_world[1], self.high_alt, self.initial_yaw
                )
                self.arrival_since_ns = None
                self.stage = 'REORIENT'
                self.get_logger().info('Pirouettes complete; re-orienting.')
            return

        if self.stage == 'REORIENT':
            self._publish_target()
            # Position remains fixed; hold briefly after commanded yaw returns.
            if self._held_at_target():
                self.target = self._make_target(
                    self.p3_world[0], self.p3_world[1], self.high_alt, self.initial_yaw
                )
                self.arrival_since_ns = None
                self.stage = 'TO_P3'
                self.get_logger().info('Re-oriented; continuing to P3.')
            return

        if self.stage == 'TO_P3':
            self._publish_target()
            if self._held_at_target():
                self.descent_index = 0
                self.target = self.descent_targets[0]
                self.arrival_since_ns = None
                self.stage = 'DESCEND'
                self.get_logger().info(
                    'P3 reached; beginning approximately 45-degree descent to P4.'
                )
            return

        if self.stage == 'DESCEND':
            self._publish_target()
            if self._held_at_target():
                self.descent_index += 1
                self.arrival_since_ns = None
                if self.descent_index >= len(self.descent_targets):
                    self.target = self._make_target(
                        self.origin.pose.position.x,
                        self.origin.pose.position.y,
                        self.final_hover,
                        self.initial_yaw,
                    )
                    self.stage = 'FINAL_HOVER'
                    self.get_logger().info(
                        f'P4 reached; holding {self.final_hover:.1f} m before landing.'
                    )
                else:
                    self.target = self.descent_targets[self.descent_index]
            return

        if self.stage == 'FINAL_HOVER':
            self._publish_target()
            if self._held_at_target():
                self.target = None
                self.arrival_since_ns = None
                self.stage = 'LAND'
                self.get_logger().info('Final hover complete; requesting LAND.')
            return

        if self.stage == 'LAND':
            self._call_mode('LAND', 'WAIT_LAND')
            return

        if self.stage == 'ABORT':
            self._call_mode(self.abort_mode, 'WAIT_LAND')
            return

        if self.stage == 'WAIT_LAND':
            if self.state is not None and not self.state.armed:
                self.stage = 'DONE'
                self.get_logger().info(
                    f'{self.route_name} complete; aircraft is disarmed.'
                )
            return


def main(args=None):
    rclpy.init(args=args)
    node = PorElement3Runner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
