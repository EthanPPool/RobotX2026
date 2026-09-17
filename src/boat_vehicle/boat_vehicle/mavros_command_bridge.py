#!/usr/bin/env python3

import copy
import json
import math
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from mavros_msgs.msg import ManualControl, State
from mavros_msgs.srv import SetMode
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, Trigger


class MavrosCommandBridge(Node):
    """Safety boundary between autonomy commands and MAVROS.

    Normal running:
        fresh non-zero Twist
        + software stop cleared
        + autonomy enabled
        + MAVROS connected
        + GUIDED
        -> MAVROS velocity stream

    The velocity stream is intentionally allowed while DISARMED.
    This lets the current autonomous setpoint be established BEFORE
    the vehicle is armed.

    Safety stop:
        stop publishing
        disable autonomy
        request HOLD
    """

    def __init__(self):
        super().__init__('mavros_command_bridge')

        self.input_topic = self.declare_parameter(
            'input_topic',
            '/control/cmd_vel'
        ).value

        self.output_topic = self.declare_parameter(
            'output_topic',
            '/mavros/setpoint_velocity/cmd_vel'
        ).value

        self.state_topic = self.declare_parameter(
            'state_topic',
            '/mavros/state'
        ).value

        self.mission_state_topic = self.declare_parameter(
            'mission_state_topic',
            '/mission/state'
        ).value

        self.operator_input_topic = self.declare_parameter(
            'operator_input_topic',
            '/operator/cmd_vel'
        ).value

        self.manual_control_topic = self.declare_parameter(
            'manual_control_topic',
            '/mavros/manual_control/send'
        ).value

        self.operator_mode = str(
            self.declare_parameter(
                'operator_mode',
                'MANUAL'
            ).value
        ).upper()

        self.operator_timeout = float(
            self.declare_parameter(
                'operator_timeout',
                0.30
            ).value
        )

        self.operator_manual_axis_max = float(
            self.declare_parameter(
                'operator_manual_axis_max',
                500.0
            ).value
        )

        self.operator_mode_retry_period = float(
            self.declare_parameter(
                'operator_mode_retry_period',
                0.25
            ).value
        )

        self.battery_topic = self.declare_parameter(
            'battery_topic',
            '/mavros/battery'
        ).value

        self.battery_warning_voltage = float(
            self.declare_parameter(
                'battery_warning_voltage',
                13.6
            ).value
        )

        self.battery_critical_voltage = float(
            self.declare_parameter(
                'battery_critical_voltage',
                13.2
            ).value
        )

        self.battery_critical_duration = float(
            self.declare_parameter(
                'battery_critical_duration',
                1.0
            ).value
        )

        self.battery_timeout = float(
            self.declare_parameter(
                'battery_timeout',
                3.0
            ).value
        )

        self.battery_required_for_propulsion = bool(
            self.declare_parameter(
                'battery_required_for_propulsion',
                True
            ).value
        )

        self.declare_parameter(
            'low_voltage_latched',
            False
        )

        self.set_mode_service = self.declare_parameter(
            'set_mode_service',
            '/mavros/set_mode'
        ).value

        # Human-readable mode reported by /mavros/state.
        self.stop_mode = str(
            self.declare_parameter(
                'stop_mode',
                'HOLD'
            ).value
        ).upper()

        self.declare_parameter(
            'autonomy_enabled',
            False
        )

        self.declare_parameter(
            'software_estop',
            True
        )

        self.allowed_modes = [
            str(mode).upper()
            for mode in self.declare_parameter(
                'allowed_modes',
                ['GUIDED']
            ).value
        ]

        self.deadman_timeout = float(
            self.declare_parameter(
                'deadman_timeout',
                0.25
            ).value
        )

        # Gives the continuously-running follower time to publish
        # a new command after autonomy is enabled.
        self.initial_command_timeout = float(
            self.declare_parameter(
                'initial_command_timeout',
                0.75
            ).value
        )

        self.publish_rate = float(
            self.declare_parameter(
                'publish_rate',
                20.0
            ).value
        )

        self.max_forward_speed = float(
            self.declare_parameter(
                'max_forward_speed',
                0.15
            ).value
        )

        self.max_yaw_rate = float(
            self.declare_parameter(
                'max_yaw_rate',
                0.15
            ).value
        )

        self.zero_command_epsilon = float(
            self.declare_parameter(
                'zero_command_epsilon',
                1.0e-4
            ).value
        )

        self.hold_retry_period = float(
            self.declare_parameter(
                'hold_retry_period',
                0.50
            ).value
        )

        self.shutdown_hold_timeout = float(
            self.declare_parameter(
                'shutdown_hold_timeout',
                1.00
            ).value
        )

        self.last_command = None
        self.last_command_time = None
        self.autonomy_enable_time = None
        self.vehicle_state = None

        # Operator deadman is passive in Phase 1.
        # It does NOT change control authority yet.
        self.operator_deadman = False
        self.operator_deadman_time = None

        self.operator_command = None
        self.operator_command_time = None

        self.operator_requested = False
        self.operator_active = False
        self.operator_session_owned = False

        # After a stale/fault stop while LB remains physically
        # held, require an actual LB release before takeover
        # can occur again.
        self.operator_rearm_required = False

        self.operator_enable_time = None

        self.operator_mode_future = None
        self.last_operator_mode_request_monotonic = 0.0

        self.battery_voltage = None
        self.last_battery_time = None
        self.battery_critical_since = None
        self.battery_safety_status = 'UNKNOWN'

        self.hold_required = True
        self.hold_future = None
        self.last_hold_request_monotonic = 0.0
        self.last_stop_reason = None

        # Intentional Task 1 pause between Gate 1 and Gate 2.
        # Unlike a safety HOLD, this does NOT revoke autonomy.
        self.intergate_hold_active = False
        self.intergate_resume_requested = False

        self.autonomy_mode_future = None
        self.last_autonomy_mode_request_monotonic = 0.0

        self.cmd_pub = self.create_publisher(
            TwistStamped,
            self.output_topic,
            10
        )

        self.manual_control_pub = self.create_publisher(
            ManualControl,
            self.manual_control_topic,
            10
        )

        self.create_subscription(
            TwistStamped,
            self.input_topic,
            self.command_callback,
            10
        )

        self.create_subscription(
            TwistStamped,
            self.operator_input_topic,
            self.operator_command_callback,
            10
        )

        self.create_subscription(
            State,
            self.state_topic,
            self.state_callback,
            10
        )

        self.create_subscription(
            String,
            self.mission_state_topic,
            self.mission_state_callback,
            10
        )

        self.create_subscription(
            BatteryState,
            self.battery_topic,
            self.battery_callback,
            qos_profile_sensor_data
        )

        self.create_subscription(
            Bool,
            '/operator/deadman',
            self.operator_deadman_callback,
            10
        )

        self.battery_status_pub = self.create_publisher(
            String,
            '/vehicle/battery_safety_status',
            10
        )

        self.software_stop_pub = self.create_publisher(
            Bool,
            '/vehicle/software_stop_state',
            10
        )

        self.control_diagnostics_pub = self.create_publisher(
            String,
            '/vehicle/control_diagnostics',
            10
        )

        self.mode_client = self.create_client(
            SetMode,
            self.set_mode_service
        )

        self.create_service(
            SetBool,
            '/vehicle/software_estop',
            self.estop_callback
        )

        self.create_service(
            SetBool,
            '/vehicle/set_autonomy',
            self.autonomy_callback
        )

        self.create_service(
            Trigger,
            '/vehicle/reset_low_voltage',
            self.reset_low_voltage_callback
        )

        self.timer = self.create_timer(
            1.0 / max(self.publish_rate, 1.0),
            self.update
        )

        self.get_logger().warn(
            'BOOT INHIBIT ACTIVE: '
            'software_estop=True, autonomy_enabled=False.'
        )

    def operator_command_callback(self, msg):
        self.operator_command = msg
        self.operator_command_time = (
            self.get_clock().now()
        )

    def operator_deadman_is_fresh(self):
        if self.operator_deadman_time is None:
            return False

        age = (
            self.get_clock().now()
            - self.operator_deadman_time
        ).nanoseconds / 1e9

        return age <= self.operator_timeout

    def operator_command_is_fresh(self):
        if self.operator_command_time is None:
            return False

        age = (
            self.get_clock().now()
            - self.operator_command_time
        ).nanoseconds / 1e9

        return age <= self.operator_timeout

    def publish_manual_neutral(self):
        msg = ManualControl()

        msg.x = 0.0
        msg.y = 0.0
        msg.z = 0.0
        msg.r = 0.0
        msg.buttons = 0

        self.manual_control_pub.publish(msg)

    def publish_operator_manual(self):
        if self.operator_command is None:
            self.publish_manual_neutral()
            return

        forward = max(
            -self.max_forward_speed,
            min(
                self.max_forward_speed,
                float(
                    self.operator_command.twist.linear.x
                )
            )
        )

        yaw = max(
            -self.max_yaw_rate,
            min(
                self.max_yaw_rate,
                float(
                    self.operator_command.twist.angular.z
                )
            )
        )

        axis_max = max(
            0.0,
            min(
                1000.0,
                self.operator_manual_axis_max
            )
        )

        if self.max_forward_speed > 0.0:
            throttle = (
                forward
                / self.max_forward_speed
                * axis_max
            )
        else:
            throttle = 0.0

        if self.max_yaw_rate > 0.0:
            steering = (
                yaw
                / self.max_yaw_rate
                * axis_max
            )
        else:
            steering = 0.0

        msg = ManualControl()

        # ArduRover MANUAL_CONTROL:
        #
        # y = steering
        # z = throttle
        #
        # Our validated neutral is y=0, z=0.
        msg.x = 0.0
        msg.y = float(steering)
        msg.z = float(throttle)
        msg.r = 0.0
        msg.buttons = 0

        self.manual_control_pub.publish(msg)

    def operator_mode_done(self, future):
        try:
            result = future.result()

            if (
                result is None
                or not result.mode_sent
            ):
                self.get_logger().error(
                    'MANUAL mode request rejected'
                )

        except Exception as exc:
            self.get_logger().error(
                f'MANUAL mode request error: {exc}'
            )

        finally:
            self.operator_mode_future = None

        # Important race protection:
        # LB may have been released while MANUAL request was
        # still in flight.
        if not self.operator_requested:
            self.hold_required = True

            self.request_hold(
                'operator released during MANUAL transition',
                force=True
            )

    def request_operator_mode(self, force=False):
        if (
            not self.operator_requested
            or not self.operator_deadman
            or not self.operator_deadman_is_fresh()
            or self.operator_rearm_required
        ):
            return

        state = self.vehicle_state

        if (
            state is None
            or not state.connected
        ):
            return

        mode = str(state.mode).upper()

        if mode == self.operator_mode:
            self.operator_active = True
            return

        if (
            self.operator_mode_future is not None
            and not self.operator_mode_future.done()
        ):
            return

        now = time.monotonic()

        if (
            not force
            and (
                now
                - self.last_operator_mode_request_monotonic
                < self.operator_mode_retry_period
            )
        ):
            return

        if not self.mode_client.service_is_ready():
            return

        self.last_operator_mode_request_monotonic = now

        request = SetMode.Request()
        request.base_mode = 0
        request.custom_mode = self.operator_mode

        self.operator_mode_future = (
            self.mode_client.call_async(request)
        )

        self.operator_mode_future.add_done_callback(
            self.operator_mode_done
        )

    def begin_operator_takeover(self):
        # A fresh LB press is the highest-priority software
        # control-authority request.
        #
        # It explicitly clears the software stop, revokes
        # autonomy, neutralizes GUIDED velocity, and requests
        # MANUAL regardless of the current ArduRover mode.
        self.operator_rearm_required = False

        if bool(
            self.get_parameter(
                'software_estop'
            ).value
        ):
            self.set_parameters([
                Parameter(
                    'software_estop',
                    Parameter.Type.BOOL,
                    False
                )
            ])

            self.publish_software_stop_state()

            self.get_logger().warn(
                'LB TAKEOVER: software stop cleared'
            )

        self.set_autonomy(False)

        self.publish_velocity_neutral()
        self.clear_stored_command()

        state = self.vehicle_state

        if (
            state is None
            or not state.connected
        ):
            self.get_logger().error(
                'LB TAKEOVER: MAVROS disconnected; '
                'cannot command ArduRover MANUAL'
            )
            return

        # Never carry stale browser stick data into a new
        # operator session.
        self.operator_command = None
        self.operator_command_time = None

        self.operator_requested = True
        self.operator_active = False
        self.operator_session_owned = True

        self.operator_enable_time = (
            self.get_clock().now()
        )

        self.hold_required = False
        self.last_stop_reason = None

        # MANUAL transition always begins at neutral.
        self.publish_manual_neutral()

        self.get_logger().warn(
            'LB TAKEOVER: autonomy revoked; '
            'requesting MANUAL from current mode '
            + str(state.mode)
        )

        self.request_operator_mode(
            force=True
        )


    def operator_stop_to_hold(
        self,
        reason,
        require_release=False
    ):
        # Neutral first. HOLD second.
        self.publish_manual_neutral()
        self.publish_velocity_neutral()

        self.set_autonomy(False)
        self.clear_stored_command()

        self.operator_requested = False
        self.operator_active = False
        self.operator_enable_time = None

        if require_release:
            self.operator_rearm_required = True

        self.hold_required = True
        self.last_stop_reason = reason

        self.get_logger().warn(
            f'OPERATOR STOP: {reason}; '
            f'neutral sent; requesting {self.stop_mode}'
        )

        self.request_hold(
            reason,
            force=True
        )

    def operator_deadman_callback(self, msg):
        previous = bool(
            self.operator_deadman
        )

        active = bool(
            msg.data
        )

        self.operator_deadman = active

        self.operator_deadman_time = (
            self.get_clock().now()
        )

        if not active:
            # A real physical release re-arms takeover after
            # any stale/fault latch.
            self.operator_rearm_required = False

            if (
                previous
                or self.operator_requested
                or self.operator_active
                or self.operator_session_owned
            ):
                self.operator_stop_to_hold(
                    'operator deadman released',
                    require_release=False
                )

            return

        # Every fresh LB press is a takeover request.
        if active and not previous:
            self.operator_rearm_required = False
            self.begin_operator_takeover()

    def battery_is_fresh(self):
        if (
            self.battery_voltage is None
            or self.last_battery_time is None
        ):
            return False

        age = (
            self.get_clock().now()
            - self.last_battery_time
        ).nanoseconds / 1e9

        return age <= self.battery_timeout

    def low_voltage_is_latched(self):
        return bool(
            self.get_parameter(
                'low_voltage_latched'
            ).value
        )

    def set_low_voltage_latched(self, value):
        self.set_parameters([
            Parameter(
                'low_voltage_latched',
                Parameter.Type.BOOL,
                bool(value)
            )
        ])

    def publish_battery_safety_status(self):
        msg = String()
        msg.data = self.battery_safety_status
        self.battery_status_pub.publish(msg)

    def battery_callback(self, msg):
        voltage = float(msg.voltage)

        if (
            not math.isfinite(voltage)
            or voltage <= 0.0
        ):
            return

        self.battery_voltage = voltage
        self.last_battery_time = (
            self.get_clock().now()
        )

        if self.low_voltage_is_latched():
            self.battery_safety_status = (
                'CRITICAL_LATCHED'
            )
            self.publish_battery_safety_status()
            return

        if voltage <= self.battery_critical_voltage:
            if self.battery_critical_since is None:
                self.battery_critical_since = (
                    self.get_clock().now()
                )

            self.battery_safety_status = (
                'CRITICAL_PENDING'
            )

        elif voltage <= self.battery_warning_voltage:
            self.battery_critical_since = None
            self.battery_safety_status = 'LOW'

        else:
            self.battery_critical_since = None
            self.battery_safety_status = 'NORMAL'

        self.publish_battery_safety_status()

    def evaluate_battery_safety(self):
        if not self.battery_is_fresh():
            if self.battery_required_for_propulsion:
                self.battery_safety_status = (
                    'TELEMETRY_STALE'
                )

                if bool(
                    self.get_parameter(
                        'autonomy_enabled'
                    ).value
                ):
                    self.trip_to_hold(
                        'battery telemetry stale'
                    )

            self.publish_battery_safety_status()
            return

        if self.low_voltage_is_latched():
            self.battery_safety_status = (
                'CRITICAL_LATCHED'
            )

            if bool(
                self.get_parameter(
                    'autonomy_enabled'
                ).value
            ):
                self.trip_to_hold(
                    'critical battery latch active'
                )

            self.publish_battery_safety_status()
            return

        voltage = float(self.battery_voltage)

        if voltage <= self.battery_critical_voltage:
            if self.battery_critical_since is None:
                self.battery_critical_since = (
                    self.get_clock().now()
                )

            elapsed = (
                self.get_clock().now()
                - self.battery_critical_since
            ).nanoseconds / 1e9

            if elapsed >= self.battery_critical_duration:
                self.set_low_voltage_latched(True)

                self.battery_safety_status = (
                    'CRITICAL_LATCHED'
                )

                self.get_logger().error(
                    'CRITICAL BATTERY: '
                    f'{voltage:.2f} V <= '
                    f'{self.battery_critical_voltage:.2f} V '
                    f'for {elapsed:.2f} s. '
                    'Propulsion inhibited.'
                )

                self.trip_to_hold(
                    'critical battery voltage'
                )

        elif voltage <= self.battery_warning_voltage:
            self.battery_critical_since = None
            self.battery_safety_status = 'LOW'

        else:
            self.battery_critical_since = None
            self.battery_safety_status = 'NORMAL'

        self.publish_battery_safety_status()

    def battery_propulsion_allowed(self):
        if self.low_voltage_is_latched():
            return (
                False,
                'critical low-voltage latch is active'
            )

        if (
            self.battery_required_for_propulsion
            and not self.battery_is_fresh()
        ):
            return (
                False,
                'battery telemetry unavailable or stale'
            )

        return True, ''

    def reset_low_voltage_callback(
        self,
        request,
        response
    ):
        if not self.battery_is_fresh():
            response.success = False
            response.message = (
                'Cannot reset low-voltage latch: '
                'battery telemetry is stale'
            )
            return response

        if (
            self.battery_voltage
            <= self.battery_warning_voltage
        ):
            response.success = False
            response.message = (
                'Cannot reset low-voltage latch: '
                f'battery is {self.battery_voltage:.2f} V; '
                f'must be above '
                f'{self.battery_warning_voltage:.2f} V'
            )
            return response

        self.set_low_voltage_latched(False)

        self.battery_critical_since = None
        self.battery_safety_status = 'NORMAL'

        self.publish_battery_safety_status()

        response.success = True
        response.message = (
            'Low-voltage latch reset. '
            'Propulsion remains disabled until '
            'explicitly enabled.'
        )

        self.get_logger().warn(
            'LOW-VOLTAGE LATCH RESET'
        )

        return response

    def mission_state_callback(self, msg):
        text = str(msg.data).strip()

        autonomy_enabled = bool(
            self.get_parameter(
                'autonomy_enabled'
            ).value
        )

        software_estop = bool(
            self.get_parameter(
                'software_estop'
            ).value
        )

        # -------------------------------------------------------------
        # INTENTIONAL INTER-GATE HOLD
        # -------------------------------------------------------------
        if text.startswith('WAIT_GATE_2'):
            if (
                autonomy_enabled
                and not software_estop
            ):
                if not self.intergate_hold_active:
                    self.get_logger().warning(
                        'INTER-GATE PAUSE: '
                        'Gate 1 complete; requesting HOLD '
                        'while waiting for Gate 2'
                    )

                self.intergate_hold_active = True
                self.intergate_resume_requested = False

                # Overwrite the final PASS_GATE velocity immediately.
                self.publish_velocity_neutral()

                # Never replay a Gate 1 command after entering HOLD.
                self.clear_stored_command()

            return

        # -------------------------------------------------------------
        # GATE 2 ACQUIRED: RESUME GUIDED
        # -------------------------------------------------------------
        if text.startswith('TRACK_GATE_2'):
            if (
                autonomy_enabled
                and self.intergate_hold_active
                and not software_estop
            ):
                self.intergate_hold_active = False
                self.intergate_resume_requested = True

                # Stay neutral until GUIDED is positively confirmed.
                self.publish_velocity_neutral()
                self.clear_stored_command()

                self.get_logger().warning(
                    'INTER-GATE RESUME: '
                    'Gate 2 acquired; requesting GUIDED'
                )

            return

        # -------------------------------------------------------------
        # FINAL TASK COMPLETION
        # -------------------------------------------------------------
        if not text.startswith('MISSION_COMPLETE'):
            return

        if self.last_stop_reason == 'mission complete':
            return

        self.get_logger().warning(
            'MISSION COMPLETE: '
            'stopping GUIDED velocity authority; '
            'follower will request LOITER'
        )

        self.publish_velocity_neutral()
        self.clear_stored_command()

        self.set_autonomy(False)

        # Do not fight the follower's final LOITER request with HOLD.
        self.hold_required = False

        self.last_stop_reason = 'mission complete'

    def command_callback(self, msg):
        self.last_command = msg
        self.last_command_time = (
            self.get_clock().now()
        )

    def state_callback(self, msg):
        self.vehicle_state = msg

        mode = str(
            msg.mode
        ).upper()

        if (
            self.hold_required
            and msg.connected
            and mode == self.stop_mode
        ):
            self.hold_required = False

            mode_request_pending = (
                self.operator_mode_future is not None
                and not self.operator_mode_future.done()
            )

            if (
                self.operator_session_owned
                and not self.operator_requested
                and not mode_request_pending
            ):
                self.operator_session_owned = False

        if not msg.connected:
            self.operator_active = False
            return

        if (
            self.operator_session_owned
            and self.operator_requested
            and mode == self.operator_mode
            and self.operator_deadman
            and self.operator_deadman_is_fresh()
            and not self.operator_rearm_required
        ):
            self.operator_active = True

        elif mode != self.operator_mode:
            self.operator_active = False

    def publish_velocity_neutral(self):
        # Explicitly overwrite any previously commanded GUIDED
        # velocity. A mode-change request must never be the only
        # mechanism stopping propulsion.
        out = TwistStamped()

        out.header.stamp = (
            self.get_clock().now().to_msg()
        )

        out.header.frame_id = 'base_link'

        out.twist.linear.x = 0.0
        out.twist.linear.y = 0.0
        out.twist.linear.z = 0.0

        out.twist.angular.x = 0.0
        out.twist.angular.y = 0.0
        out.twist.angular.z = 0.0

        self.cmd_pub.publish(out)

    def clear_stored_command(self):
        self.last_command = None
        self.last_command_time = None

    def set_autonomy(self, enabled):
        self.set_parameters([
            Parameter(
                'autonomy_enabled',
                Parameter.Type.BOOL,
                bool(enabled)
            )
        ])

        if not enabled:
            self.autonomy_enable_time = None

            # A real autonomy revocation cancels any intentional
            # inter-gate pause/resume sequence.
            self.intergate_hold_active = False
            self.intergate_resume_requested = False

    def hold_done(self, future, reason):
        try:
            result = future.result()

            if (
                result is None
                or not result.mode_sent
            ):
                self.get_logger().error(
                    f'{self.stop_mode} REQUEST FAILED: '
                    f'{reason}'
                )
            else:
                self.get_logger().warn(
                    f'{self.stop_mode} REQUEST SENT: '
                    f'{reason}'
                )

        except Exception as exc:
            self.get_logger().error(
                f'{self.stop_mode} REQUEST ERROR: '
                f'{exc}'
            )

        finally:
            self.hold_future = None

    def autonomy_mode_done(self, future):
        try:
            result = future.result()

            if (
                result is None
                or not result.mode_sent
            ):
                self.get_logger().error(
                    'GUIDED mode request rejected '
                    'during inter-gate resume'
                )
            else:
                self.get_logger().warning(
                    'GUIDED mode request sent '
                    'during inter-gate resume'
                )

        except Exception as exc:
            self.get_logger().error(
                f'GUIDED mode request error: {exc}'
            )

        finally:
            self.autonomy_mode_future = None


    def request_autonomy_mode(self, force=False):
        if (
            not self.intergate_resume_requested
            or self.intergate_hold_active
            or not bool(
                self.get_parameter(
                    'autonomy_enabled'
                ).value
            )
        ):
            return

        state = self.vehicle_state

        if (
            state is None
            or not state.connected
        ):
            return

        mode = str(state.mode).upper()

        if mode in self.allowed_modes:
            self.intergate_resume_requested = False
            return

        # The bridge is only allowed to resume automatically
        # from the intentional HOLD it created.
        if mode != self.stop_mode:
            self.trip_to_hold(
                'unexpected mode during inter-gate resume: '
                f'{state.mode}'
            )
            return

        if (
            self.autonomy_mode_future is not None
            and not self.autonomy_mode_future.done()
        ):
            return

        # Do not overlap another mode transition.
        if (
            self.hold_future is not None
            and not self.hold_future.done()
        ):
            return

        if (
            self.operator_mode_future is not None
            and not self.operator_mode_future.done()
        ):
            return

        now = time.monotonic()

        if (
            not force
            and (
                now
                - self.last_autonomy_mode_request_monotonic
                < self.hold_retry_period
            )
        ):
            return

        if not self.mode_client.service_is_ready():
            return

        self.last_autonomy_mode_request_monotonic = now

        request = SetMode.Request()
        request.base_mode = 0
        request.custom_mode = self.allowed_modes[0]

        self.autonomy_mode_future = (
            self.mode_client.call_async(request)
        )

        self.autonomy_mode_future.add_done_callback(
            self.autonomy_mode_done
        )

    def request_hold(
        self,
        reason,
        force=False
    ):
        state = self.vehicle_state

        if (
            state is None
            or not state.connected
        ):
            return

        mode = str(state.mode).upper()

        if mode == self.stop_mode:
            self.hold_required = False
            return

        autonomy_owned_mode = (
            mode in self.allowed_modes
        )

        operator_owned_mode = (
            self.operator_session_owned
            and mode == self.operator_mode
        )

        # Do not fight unrelated AUTO/RTL/etc.
        # MANUAL is owned only when this bridge created the
        # operator session.
        if not (
            autonomy_owned_mode
            or operator_owned_mode
        ):
            self.hold_required = False
            return

        if (
            self.hold_future is not None
            and not self.hold_future.done()
        ):
            return

        now = time.monotonic()

        if (
            not force
            and (
                now
                - self.last_hold_request_monotonic
                < self.hold_retry_period
            )
        ):
            return

        self.last_hold_request_monotonic = now

        if not self.mode_client.service_is_ready():
            return

        request = SetMode.Request()
        request.base_mode = 0
        request.custom_mode = self.stop_mode

        self.hold_future = (
            self.mode_client.call_async(request)
        )

        self.hold_future.add_done_callback(
            lambda future: self.hold_done(
                future,
                reason
            )
        )

    def trip_to_hold(self, reason):
        self.set_autonomy(False)

        # Stop propulsion immediately while Rover is still in
        # the current mode. Do not wait for HOLD to be accepted.
        self.publish_velocity_neutral()

        self.clear_stored_command()

        if self.operator_session_owned:
            self.publish_manual_neutral()

            if self.operator_deadman:
                self.operator_rearm_required = True

        self.operator_requested = False
        self.operator_active = False
        self.operator_enable_time = None

        self.hold_required = True

        if reason != self.last_stop_reason:
            self.get_logger().error(
                f'PROPULSION STOP: {reason}; '
                f'requesting {self.stop_mode}'
            )

            self.last_stop_reason = reason

        self.request_hold(
            reason,
            force=True
        )

    def command_age(self):
        """Return age of the latest autonomy command in seconds."""

        if self.last_command_time is None:
            return None

        return (
            self.get_clock().now()
            - self.last_command_time
        ).nanoseconds / 1e9

    def diagnostic_bridge_reason(self):
        """Describe the control-authority branch update() will take.

        This mirrors the existing update() priority order but does not
        change state, publish propulsion commands, or request a mode.
        """

        software_estop = bool(
            self.get_parameter(
                'software_estop'
            ).value
        )

        autonomy_enabled = bool(
            self.get_parameter(
                'autonomy_enabled'
            ).value
        )

        state = self.vehicle_state
        connected = bool(
            state is not None
            and state.connected
        )

        mode = (
            str(state.mode).upper()
            if state is not None
            else 'UNKNOWN'
        )

        # HOLD handling is the highest-priority update() branch.
        if self.hold_required:
            return 'HOLD_REQUIRED'

        # ---------------------------------------------------------
        # OPERATOR AUTHORITY
        # ---------------------------------------------------------
        if (
            self.operator_requested
            or self.operator_active
            or self.operator_session_owned
        ):
            if software_estop:
                return 'OPERATOR_BLOCKED_SOFTWARE_ESTOP'

            if (
                not self.operator_deadman
                or not self.operator_deadman_is_fresh()
            ):
                return 'OPERATOR_DEADMAN_STALE'

            if self.operator_rearm_required:
                return 'OPERATOR_REARM_REQUIRED'

            if not connected:
                return 'OPERATOR_MAVROS_DISCONNECTED'

            if not self.operator_requested:
                return 'OPERATOR_NEUTRAL_NOT_REQUESTED'

            if mode != self.operator_mode:
                return 'OPERATOR_WAITING_FOR_MANUAL'

            if (
                self.operator_command is None
                or self.operator_command_time is None
            ):
                if self.operator_enable_time is not None:
                    age = (
                        self.get_clock().now()
                        - self.operator_enable_time
                    ).nanoseconds / 1e9

                    if age <= self.operator_timeout:
                        return 'OPERATOR_INITIAL_COMMAND_GRACE'

                return 'OPERATOR_NO_FRESH_COMMAND'

            if not self.operator_command_is_fresh():
                return 'OPERATOR_COMMAND_STALE'

            if not self.command_is_finite(
                self.operator_command
            ):
                return 'OPERATOR_COMMAND_NONFINITE'

            return 'OPERATOR_AUTHORIZED'

        # ---------------------------------------------------------
        # NON-OPERATOR SAFETY
        # ---------------------------------------------------------
        if software_estop:
            return 'SOFTWARE_ESTOP'

        if not autonomy_enabled:
            return 'AUTONOMY_DISABLED'

        if not connected:
            return 'MAVROS_DISCONNECTED'

        # ---------------------------------------------------------
        # AUTONOMY AUTHORITY
        # ---------------------------------------------------------
        if self.intergate_hold_active:
            if mode == self.stop_mode:
                return 'INTERGATE_HOLD'

            if mode not in self.allowed_modes:
                return 'INTERGATE_HOLD_UNEXPECTED_MODE'

            return 'INTERGATE_HOLD_REQUESTING_HOLD'

        if self.intergate_resume_requested:
            if mode in self.allowed_modes:
                return 'INTERGATE_RESUME_GUIDED_CONFIRMED'

            if mode != self.stop_mode:
                return 'INTERGATE_RESUME_UNEXPECTED_MODE'

            return 'INTERGATE_RESUME_REQUESTING_GUIDED'

        if mode not in self.allowed_modes:
            return 'MODE_NOT_ALLOWED'

        if (
            self.last_command is None
            or self.last_command_time is None
        ):
            if self.autonomy_enable_time is not None:
                since_enable = (
                    self.get_clock().now()
                    - self.autonomy_enable_time
                ).nanoseconds / 1e9

                if (
                    since_enable
                    <= self.initial_command_timeout
                ):
                    return 'INITIAL_COMMAND_GRACE'

            return 'NO_FRESH_COMMAND'

        age = self.command_age()

        if (
            age is None
            or age > self.deadman_timeout
        ):
            return 'COMMAND_STALE'

        if not self.command_is_finite(
            self.last_command
        ):
            return 'COMMAND_NONFINITE'

        if self.command_is_zero(
            self.last_command
        ):
            return 'AUTHORIZED_ZERO'

        return 'AUTHORIZED'

    def publish_control_diagnostics(self):
        """Publish a read-only snapshot of bridge authorization state."""

        try:
            reason = self.diagnostic_bridge_reason()

            software_estop = bool(
                self.get_parameter(
                    'software_estop'
                ).value
            )

            autonomy_enabled = bool(
                self.get_parameter(
                    'autonomy_enabled'
                ).value
            )

            state = self.vehicle_state

            connected = bool(
                state is not None
                and state.connected
            )

            armed = bool(
                state is not None
                and state.armed
            )

            mode = (
                str(state.mode).upper()
                if state is not None
                else 'UNKNOWN'
            )

            mode_allowed = bool(
                connected
                and mode in self.allowed_modes
            )

            input_age = self.command_age()

            command_fresh = bool(
                self.last_command is not None
                and input_age is not None
                and input_age <= self.deadman_timeout
            )

            command_zero = (
                None
                if self.last_command is None
                else bool(
                    self.command_is_zero(
                        self.last_command
                    )
                )
            )

            input_linear = (
                None
                if self.last_command is None
                else float(
                    self.last_command.twist.linear.x
                )
            )

            input_yaw = (
                None
                if self.last_command is None
                else float(
                    self.last_command.twist.angular.z
                )
            )

            output_authorized = bool(
                reason in (
                    'AUTHORIZED',
                    'AUTHORIZED_ZERO',
                )
            )

            data = {
                'bridge_reason': reason,
                'bridge_autonomy_enabled': autonomy_enabled,
                'software_estop': software_estop,
                'bridge_mode_allowed': mode_allowed,
                'bridge_command_fresh': command_fresh,
                'bridge_output_authorized': output_authorized,
                'bridge_input_age_s': input_age,

                'bridge_connected': connected,
                'bridge_armed': armed,
                'bridge_mode': mode,
                'bridge_command_zero': command_zero,
                'bridge_input_linear_x': input_linear,
                'bridge_input_angular_z': input_yaw,

                'hold_required': bool(
                    self.hold_required
                ),
                'last_stop_reason': self.last_stop_reason,

                'intergate_hold_active': bool(
                    self.intergate_hold_active
                ),
                'intergate_resume_requested': bool(
                    self.intergate_resume_requested
                ),

                'operator_requested': bool(
                    self.operator_requested
                ),
                'operator_active': bool(
                    self.operator_active
                ),
                'operator_session_owned': bool(
                    self.operator_session_owned
                ),
                'operator_deadman': bool(
                    self.operator_deadman
                ),
                'operator_deadman_fresh': bool(
                    self.operator_deadman_is_fresh()
                ),

                'battery_safety_status': str(
                    self.battery_safety_status
                ),
            }

            msg = String()
            msg.data = json.dumps(
                data,
                separators=(',', ':'),
            )

            self.control_diagnostics_pub.publish(msg)

        except Exception as exc:
            # Diagnostics must never alter propulsion behavior.
            self.get_logger().warning(
                f'Control diagnostics publish failed: {exc}'
            )

    def publish_software_stop_state(self):
        msg = Bool()

        msg.data = bool(
            self.get_parameter(
                'software_estop'
            ).value
        )

        self.software_stop_pub.publish(msg)

    def estop_callback(
        self,
        request,
        response
    ):
        active = bool(
            request.data
        )

        self.set_parameters([
            Parameter(
                'software_estop',
                Parameter.Type.BOOL,
                active
            ),
            Parameter(
                'autonomy_enabled',
                Parameter.Type.BOOL,
                False
            ),
        ])

        self.autonomy_enable_time = None
        self.clear_stored_command()

        self.publish_software_stop_state()

        if active:
            # Overwrite any previously accepted GUIDED velocity before
            # waiting for the next timer cycle or a HOLD acknowledgement.
            self.publish_velocity_neutral()

            if self.operator_session_owned:
                self.publish_manual_neutral()

                if self.operator_deadman:
                    self.operator_rearm_required = True

            self.operator_requested = False
            self.operator_active = False
            self.operator_enable_time = None

            self.hold_required = True
            self.last_stop_reason = (
                'software stop engaged'
            )

            self.request_hold(
                self.last_stop_reason,
                force=True
            )

            response.success = True
            response.message = (
                'SOFTWARE STOP ENGAGED: '
                'all propulsion authority revoked; '
                'HOLD requested'
            )

        else:
            response.success = True
            response.message = (
                'Software stop cleared. '
                'Autonomy and operator authority remain DISABLED.'
            )

        return response

    def autonomy_callback(
        self,
        request,
        response
    ):
        enable = bool(request.data)

        if not enable:
            self.set_autonomy(False)
            self.publish_velocity_neutral()
            self.clear_stored_command()

            self.hold_required = True

            self.request_hold(
                'autonomy disabled',
                force=True
            )

            response.success = True
            response.message = (
                'Autonomy disabled; HOLD requested'
            )

            return response

        # Operator authority always wins over an autonomy-enable
        # request.
        if (
            self.operator_requested
            or self.operator_active
            or self.operator_session_owned
            or (
                self.operator_deadman
                and self.operator_deadman_is_fresh()
            )
        ):
            self.set_autonomy(False)

            response.success = False
            response.message = (
                'Cannot enable autonomy while '
                'operator deadman/authority is active'
            )

            return response

        if bool(
            self.get_parameter(
                'software_estop'
            ).value
        ):
            self.set_autonomy(False)

            response.success = False
            response.message = (
                'Cannot enable autonomy while '
                'software stop is active'
            )

            return response

        state = self.vehicle_state

        if (
            state is None
            or not state.connected
        ):
            self.set_autonomy(False)

            response.success = False
            response.message = (
                'Cannot enable autonomy: '
                'MAVROS not connected'
            )

            return response

        mode = str(state.mode).upper()

        if mode not in self.allowed_modes:
            self.set_autonomy(False)

            response.success = False
            response.message = (
                'Cannot enable autonomy: '
                f'vehicle mode {state.mode} '
                'is not GUIDED'
            )

            return response

        # Gate detection is intentionally NOT a prerequisite.
        # A zero follower command is also valid. The bridge can
        # remain enabled while waiting for perception.
        self.clear_stored_command()

        self.set_autonomy(True)

        self.autonomy_enable_time = (
            self.get_clock().now()
        )

        self.hold_required = False
        self.last_stop_reason = None

        response.success = True

        if state.armed:
            response.message = (
                'Autonomy enabled; vehicle ARMED'
            )
        else:
            response.message = (
                'Autonomy enabled while DISARMED; '
                'waiting for controller commands'
            )

        return response


    def command_is_zero(
        self,
        msg
    ):
        return (
            abs(float(msg.twist.linear.x))
                <= self.zero_command_epsilon
            and
            abs(float(msg.twist.angular.z))
                <= self.zero_command_epsilon
        )

    def command_is_finite(
        self,
        msg
    ):
        values = (
            msg.twist.linear.x,
            msg.twist.linear.y,
            msg.twist.linear.z,
            msg.twist.angular.x,
            msg.twist.angular.y,
            msg.twist.angular.z,
        )

        return all(
            math.isfinite(float(v))
            for v in values
        )

    def update(self):
        # Battery telemetry remains informational/warning-only.
        # It no longer owns propulsion authority.
        self.publish_software_stop_state()

        # Read-only 20 Hz diagnostic snapshot. This must never
        # participate in authority decisions or propulsion output.
        self.publish_control_diagnostics()

        # HOLD retries always win, including after a
        # low-voltage latch.
        if self.hold_required:
            # Continuously overwrite any previous GUIDED
            # velocity command until HOLD is positively confirmed.
            self.publish_velocity_neutral()

            if self.operator_session_owned:
                self.publish_manual_neutral()

            self.request_hold(
                self.last_stop_reason
                or 'stop latched'
            )
            return

        # ====================================================
        # OPERATOR AUTHORITY
        # ====================================================

        if (
            self.operator_requested
            or self.operator_active
            or self.operator_session_owned
        ):
            if bool(
                self.get_parameter(
                    'software_estop'
                ).value
            ):
                self.operator_stop_to_hold(
                    'software stop active',
                    require_release=True
                )
                return

            if (
                not self.operator_deadman
                or not self.operator_deadman_is_fresh()
            ):
                self.operator_stop_to_hold(
                    'operator deadman stale/released',
                    require_release=True
                )
                return

            if self.operator_rearm_required:
                self.operator_stop_to_hold(
                    'operator release/repress required',
                    require_release=True
                )
                return

            state = self.vehicle_state

            if (
                state is None
                or not state.connected
            ):
                self.operator_stop_to_hold(
                    'MAVROS disconnected',
                    require_release=True
                )
                return

            mode = str(
                state.mode
            ).upper()

            if not self.operator_requested:
                self.publish_manual_neutral()
                return

            if mode != self.operator_mode:
                # Do not release stick commands until MANUAL
                # is positively confirmed.
                self.operator_active = False
                self.publish_manual_neutral()

                # LB takeover may request MANUAL from any
                # current ArduRover mode.
                self.request_operator_mode()
                return

            self.operator_active = True

            if (
                self.operator_command is None
                or self.operator_command_time is None
            ):
                if self.operator_enable_time is not None:
                    age = (
                        self.get_clock().now()
                        - self.operator_enable_time
                    ).nanoseconds / 1e9

                    if age <= self.operator_timeout:
                        self.publish_manual_neutral()
                        return

                self.operator_stop_to_hold(
                    'no fresh operator command',
                    require_release=True
                )
                return

            if not self.operator_command_is_fresh():
                self.operator_stop_to_hold(
                    'operator command stale',
                    require_release=True
                )
                return

            if not self.command_is_finite(
                self.operator_command
            ):
                self.operator_stop_to_hold(
                    'non-finite operator command',
                    require_release=True
                )
                return

            # Centered stick is valid here.
            # LB held + zero stick = MANUAL neutral.
            self.publish_operator_manual()
            return

        # ====================================================
        # NON-OPERATOR SAFETY
        # ====================================================

        if bool(
            self.get_parameter(
                'software_estop'
            ).value
        ):
            self.hold_required = True
            self.last_stop_reason = (
                'software stop active'
            )

            self.request_hold(
                self.last_stop_reason
            )
            return

        # ====================================================
        # AUTONOMY AUTHORITY
        # ====================================================

        if not bool(
            self.get_parameter(
                'autonomy_enabled'
            ).value
        ):
            return

        state = self.vehicle_state

        if (
            state is None
            or not state.connected
        ):
            self.trip_to_hold(
                'MAVROS disconnected'
            )
            return

        mode = str(
            state.mode
        ).upper()

        # ====================================================
        # INTENTIONAL INTER-GATE HOLD
        # ====================================================

        if self.intergate_hold_active:
            # Never allow an old GUIDED velocity to remain active
            # while transitioning into or sitting in HOLD.
            self.publish_velocity_neutral()

            if mode == self.stop_mode:
                # This is the expected inter-gate state.
                # Keep autonomy authorization but do nothing.
                return

            if mode not in self.allowed_modes:
                self.trip_to_hold(
                    'unexpected mode during inter-gate HOLD: '
                    f'{state.mode}'
                )
                return

            # Still in GUIDED: keep requesting HOLD until confirmed.
            self.request_hold(
                'waiting for Gate 2'
            )
            return

        # ====================================================
        # INTER-GATE RESUME TO GUIDED
        # ====================================================

        if self.intergate_resume_requested:
            # HOLD must remain neutral until GUIDED is actually confirmed.
            self.publish_velocity_neutral()

            if mode in self.allowed_modes:
                self.intergate_resume_requested = False

                # Give the follower a fresh-command grace period after
                # returning from HOLD.
                self.autonomy_enable_time = (
                    self.get_clock().now()
                )

                self.get_logger().warning(
                    'INTER-GATE RESUME COMPLETE: '
                    f'{state.mode} confirmed'
                )

                return

            if mode != self.stop_mode:
                self.trip_to_hold(
                    'unexpected mode during inter-gate resume: '
                    f'{state.mode}'
                )
                return

            self.request_autonomy_mode()
            return

        if mode not in self.allowed_modes:
            self.trip_to_hold(
                f'vehicle left allowed mode: '
                f'{state.mode}'
            )
            return

        if (
            self.last_command is None
            or self.last_command_time is None
        ):
            if self.autonomy_enable_time is not None:
                since_enable = (
                    self.get_clock().now()
                    - self.autonomy_enable_time
                ).nanoseconds / 1e9

                if (
                    since_enable
                    <= self.initial_command_timeout
                ):
                    return

            self.trip_to_hold(
                'no fresh command after enable'
            )
            return

        age = (
            self.get_clock().now()
            - self.last_command_time
        ).nanoseconds / 1e9

        if age > self.deadman_timeout:
            self.trip_to_hold(
                f'command stale ({age:.3f} s)'
            )
            return

        if not self.command_is_finite(
            self.last_command
        ):
            self.trip_to_hold(
                'non-finite command'
            )
            return

        if self.command_is_zero(
            self.last_command
        ):
            # A zero controller command is a valid autonomous
            # neutral command. This occurs while waiting between
            # gate 1 and gate 2 and must NOT leave GUIDED mode.
            #
            # Actual safety faults still use trip_to_hold(), and
            # MISSION_COMPLETE is handled by mission_state_callback().
            out = TwistStamped()

            out.header.stamp = (
                self.get_clock().now().to_msg()
            )

            out.header.frame_id = 'base_link'

            out.twist.linear.x = 0.0
            out.twist.linear.y = 0.0
            out.twist.linear.z = 0.0

            out.twist.angular.x = 0.0
            out.twist.angular.y = 0.0
            out.twist.angular.z = 0.0

            self.cmd_pub.publish(out)
            return

        out = copy.deepcopy(
            self.last_command
        )

        out.header.stamp = (
            self.get_clock().now().to_msg()
        )

        out.header.frame_id = 'base_link'

        out.twist.linear.x = max(
            -self.max_forward_speed,
            min(
                self.max_forward_speed,
                float(
                    self.last_command.twist.linear.x
                )
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
                float(
                    self.last_command.twist.angular.z
                )
            )
        )

        self.cmd_pub.publish(out)

    def shutdown_to_hold(self):
        self.set_autonomy(False)
        self.publish_velocity_neutral()
        self.clear_stored_command()

        if self.operator_session_owned:
            self.publish_manual_neutral()

        self.operator_requested = False
        self.operator_active = False

        self.hold_required = True

        state = self.vehicle_state

        if (
            state is None
            or not state.connected
        ):
            return

        mode = str(
            state.mode
        ).upper()

        owned = (
            mode in self.allowed_modes
            or (
                self.operator_session_owned
                and mode == self.operator_mode
            )
        )

        if not owned:
            return

        if not self.mode_client.wait_for_service(
            timeout_sec=0.25
        ):
            return

        request = SetMode.Request()
        request.base_mode = 0
        request.custom_mode = self.stop_mode

        future = self.mode_client.call_async(
            request
        )

        try:
            rclpy.spin_until_future_complete(
                self,
                future,
                timeout_sec=self.shutdown_hold_timeout
            )
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)

    node = MavrosCommandBridge()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        try:
            node.shutdown_to_hold()
        except Exception:
            pass

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
