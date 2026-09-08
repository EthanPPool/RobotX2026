#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger

from boat_interfaces.msg import Gate


class TwoGateFollower(Node):
    """Conservative two-gate controller based on the validated
    single-gate follower behavior.

    Gate passage is recognized only after the gate has first been
    close enough to arm passage detection.

    Passage is then recognized by either:
      1. close gate disappearing long enough, or
      2. detector jumping to a substantially farther gate.

    No blind CLEAR_GATE driving is performed.
    """

    def __init__(self):
        super().__init__('two_gate_follower')

        self.gate_topic = self.declare_parameter(
            'gate_topic',
            '/perception/gate'
        ).value

        self.output_topic = self.declare_parameter(
            'output_topic',
            '/control/cmd_vel'
        ).value

        self.state_topic = self.declare_parameter(
            'state_topic',
            '/mission/state'
        ).value

        self.vehicle_state_topic = self.declare_parameter(
            'vehicle_state_topic',
            '/mavros/state'
        ).value

        self.local_position_topic = self.declare_parameter(
            'local_position_topic',
            '/mavros/local_position/pose'
        ).value

        self.enabled = bool(
            self.declare_parameter(
                'enabled',
                True
            ).value
        )

        self.gates_required = int(
            self.declare_parameter(
                'gates_required',
                2
            ).value
        )

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

        self.forward_angle_limit_deg = float(
            self.declare_parameter(
                'forward_angle_limit_deg',
                20.0
            ).value
        )

        self.passage_arm_distance = float(
            self.declare_parameter(
                'passage_arm_distance',
                2.00
            ).value
        )

        # A pass cannot be inferred from GPS/EKF displacement alone.  The
        # tracked gate must also move substantially closer in the LiDAR body
        # frame and remain inside this close range for several observations.
        self.pass_close_distance = float(
            self.declare_parameter(
                'pass_close_distance',
                1.00
            ).value
        )

        self.pass_min_gate_approach = float(
            self.declare_parameter(
                'pass_min_gate_approach',
                0.50
            ).value
        )

        self.pass_close_confirm_hits = int(
            self.declare_parameter(
                'pass_close_confirm_hits',
                3
            ).value
        )

        self.pass_loss_timeout = float(
            self.declare_parameter(
                'pass_loss_timeout',
                0.40
            ).value
        )

        self.pass_jump_distance = float(
            self.declare_parameter(
                'pass_jump_distance',
                0.40
            ).value
        )

        # Real vehicle displacement required before a gate
        # can ever be counted as passed.
        self.pass_min_travel = float(
            self.declare_parameter(
                'pass_min_travel',
                0.75
            ).value
        )

        self.vehicle_state = None

        self.local_x = None
        self.local_y = None

        self.pass_arm_local_x = None
        self.pass_arm_local_y = None
        self.pass_arm_gate_x = None
        self.close_gate_hits = 0
        self.lidar_approach_confirmed = False

        self.cmd_pub = self.create_publisher(
            TwistStamped,
            self.output_topic,
            10
        )

        self.state_pub = self.create_publisher(
            String,
            self.state_topic,
            10
        )

        self.create_subscription(
            Gate,
            self.gate_topic,
            self.gate_callback,
            10
        )

        # Mission passage logic must not advance while the
        # vehicle is disarmed.  We still keep the gate fresh
        # so pre-arm GUIDED setpoints can be streamed.
        self.vehicle_state = None

        self.create_subscription(
            State,
            self.vehicle_state_topic,
            self.vehicle_state_callback,
            10
        )

        self.create_subscription(
            PoseStamped,
            self.local_position_topic,
            self.local_position_callback,
            qos_profile_sensor_data
        )

        self.create_service(
            SetBool,
            '/control/set_enabled',
            self.set_enabled_callback
        )

        self.create_service(
            Trigger,
            '/control/reset_mission',
            self.reset_callback
        )

        self.timer = self.create_timer(
            0.05,
            self.update
        )

        self.last_state_text = None

        self.reset_mission()

        self.get_logger().warn(
            'Two-gate follower started: '
            '2 gates, 0.12 m/s conservative control, '
            'no blind gate clearing.'
        )

    def reset_mission(self):
        self.gates_passed = 0
        self.current_gate = 1

        self.mission_complete = False

        self.last_gate = None
        self.last_gate_time = None
        self.last_gate_measurement_stamp = None

        self.passage_armed = False
        self.closest_gate_x = None
        self.pass_arm_local_x = None
        self.pass_arm_local_y = None
        self.pass_arm_gate_x = None
        self.close_gate_hits = 0
        self.lidar_approach_confirmed = False

        self.publish_state(
            'WAIT_GATE_1: waiting for confirmed gate 1'
        )

    def set_enabled_callback(
        self,
        request,
        response
    ):
        self.enabled = bool(request.data)

        if not self.enabled:
            self.publish_zero()
            self.publish_state(
                'DISABLED: controller stopped'
            )

        else:
            self.publish_state(
                f'WAIT_GATE_{self.current_gate}: controller enabled'
            )

        response.success = True
        response.message = (
            'two-gate follower enabled'
            if self.enabled
            else 'two-gate follower disabled'
        )

        return response

    def reset_callback(
        self,
        request,
        response
    ):
        self.reset_mission()
        self.publish_zero()

        response.success = True
        response.message = (
            'Two-gate mission reset to gate 1'
        )

        return response

    def gate_is_valid(self, msg):
        confidence = float(
            msg.confidence
        )

        x = float(
            msg.center.x
        )

        y = float(
            msg.center.y
        )

        return (
            math.isfinite(confidence)
            and math.isfinite(x)
            and math.isfinite(y)
            and confidence >= self.min_gate_confidence
            and x > 0.0
        )



    def vehicle_state_callback(self, msg):
        self.vehicle_state = msg

        # If we lose actual propulsion authority, cancel a
        # partially armed gate-passage event.
        if not self.vehicle_motion_ready():
            self.clear_passage_state()

    def local_position_callback(self, msg):
        self.local_x = float(msg.pose.position.x)
        self.local_y = float(msg.pose.position.y)

    def vehicle_motion_ready(self):
        state = self.vehicle_state

        return (
            state is not None
            and bool(state.connected)
            and bool(state.armed)
            and str(state.mode).upper() == 'GUIDED'
        )

    def travel_since_pass_arm(self):
        if (
            self.local_x is None
            or self.local_y is None
            or self.pass_arm_local_x is None
            or self.pass_arm_local_y is None
        ):
            return 0.0

        return math.hypot(
            self.local_x - self.pass_arm_local_x,
            self.local_y - self.pass_arm_local_y
        )

    def gate_approach_since_pass_arm(self):
        if (
            self.pass_arm_gate_x is None
            or self.closest_gate_x is None
        ):
            return 0.0

        return max(
            0.0,
            self.pass_arm_gate_x - self.closest_gate_x
        )

    def clear_passage_state(self):
        self.passage_armed = False
        self.closest_gate_x = None
        self.pass_arm_local_x = None
        self.pass_arm_local_y = None
        self.pass_arm_gate_x = None
        self.close_gate_hits = 0
        self.lidar_approach_confirmed = False

    def gate_callback(self, msg):
        if self.mission_complete:
            return

        if not self.gate_is_valid(msg):
            return

        now = self.get_clock().now()

        x = float(msg.center.x)

        measurement_stamp = (
            int(msg.header.stamp.sec),
            int(msg.header.stamp.nanosec),
        )

        new_measurement = (
            measurement_stamp
            != self.last_gate_measurement_stamp
        )

        # Always retain the current valid detection so guidance
        # can steer continuously using the gate detector's bounded
        # sample-and-hold output.
        self.last_gate = msg
        self.last_gate_time = now

        # Repeated gate messages with the same source timestamp are held
        # control output, not additional LiDAR evidence. They refresh
        # guidance but must not increment close-hit or passage logic.
        if not new_measurement:
            return

        self.last_gate_measurement_stamp = measurement_stamp

        # ----------------------------------------------------
        # PASSAGE ARMING
        # ----------------------------------------------------
        #
        # Merely seeing a gate close to the boat is NOT enough.
        # We only arm passage detection after:
        #
        #   connected + ARMED + GUIDED + valid local position
        #
        just_confirmed_lidar_approach = False

        if (
            x <= self.passage_arm_distance
            and not self.passage_armed
            and self.vehicle_motion_ready()
            and self.local_x is not None
            and self.local_y is not None
        ):
            self.passage_armed = True
            self.closest_gate_x = x
            self.pass_arm_gate_x = x
            self.close_gate_hits = (
                1 if x <= self.pass_close_distance else 0
            )
            self.lidar_approach_confirmed = False

            self.pass_arm_local_x = self.local_x
            self.pass_arm_local_y = self.local_y

            self.publish_state(
                f'PASSAGE_ARMED_GATE_{self.current_gate}: '
                f'gate at {x:.2f} m; '
                f'waiting for close LiDAR approach and boat travel'
            )

        elif self.passage_armed:
            self.closest_gate_x = min(
                self.closest_gate_x,
                x
            )

            if not self.lidar_approach_confirmed:
                if x <= self.pass_close_distance:
                    self.close_gate_hits += 1
                else:
                    self.close_gate_hits = 0

                approach = self.gate_approach_since_pass_arm()

                if (
                    self.close_gate_hits
                    >= self.pass_close_confirm_hits
                    and approach >= self.pass_min_gate_approach
                ):
                    self.lidar_approach_confirmed = True
                    just_confirmed_lidar_approach = True

        # ----------------------------------------------------
        # FAR-GATE TRANSITION
        # ----------------------------------------------------
        #
        # Perception is allowed to jump to a farther candidate,
        # but that CANNOT count as passing the gate until the
        # boat physically moved at least pass_min_travel.
        #
        if (
            self.passage_armed
            and self.closest_gate_x is not None
            and self.lidar_approach_confirmed
            and self.vehicle_motion_ready()
            and self.travel_since_pass_arm()
                >= self.pass_min_travel
            and hasattr(self, 'pass_jump_distance')
            and x >= (
                self.closest_gate_x
                + self.pass_jump_distance
            )
        ):
            old_gate = self.current_gate

            self.finish_current_gate(
                'farther gate acquired after real vehicle travel'
            )

            if self.mission_complete:
                return

            # Use this farther detection as the beginning of the
            # next gate's tracking, but DO NOT arm passage yet.
            self.last_gate = msg
            self.last_gate_time = now

            self.publish_state(
                f'TRACK_GATE_{self.current_gate}: '
                f'gate {old_gate} passed after vehicle travel'
            )

            return

        if just_confirmed_lidar_approach:
            self.publish_state(
                f'LIDAR_APPROACH_CONFIRMED_GATE_{self.current_gate}: '
                f'closest={self.closest_gate_x:.2f} m; '
                f'approach={self.gate_approach_since_pass_arm():.2f} m'
            )
        else:
            self.publish_state(
                f'TRACK_GATE_{self.current_gate}: '
                f'x={x:.2f} m'
            )

    def gate_age(self):
        if self.last_gate_time is None:
            return None

        return (
            self.get_clock().now()
            - self.last_gate_time
        ).nanoseconds / 1e9

    def gate_is_fresh(self):
        age = self.gate_age()

        return (
            self.last_gate is not None
            and age is not None
            and age <= self.gate_timeout
        )

    def finish_current_gate(self, reason):
        finished_gate = self.current_gate

        self.gates_passed += 1

        self.last_gate = None
        self.last_gate_time = None

        self.clear_passage_state()

        if self.gates_passed >= self.gates_required:
            self.mission_complete = True

            self.publish_state(
                f'MISSION_COMPLETE: passed '
                f'{self.gates_passed}/{self.gates_required} gates; '
                f'{reason}'
            )

            return

        self.current_gate = (
            self.gates_passed + 1
        )

        self.publish_state(
            f'WAIT_GATE_{self.current_gate}: '
            f'gate {finished_gate} passed; '
            f'{reason}'
        )

    def publish_state(self, text):
        # Avoid spamming the console with an identical state
        # on every 20 Hz control iteration.
        if text == self.last_state_text:
            return

        self.last_state_text = text

        msg = String()
        msg.data = text

        self.state_pub.publish(msg)
        self.get_logger().info(text)

    def publish_zero(self):
        msg = TwistStamped()

        msg.header.stamp = (
            self.get_clock().now().to_msg()
        )

        msg.header.frame_id = 'base_link'

        msg.twist.linear.x = 0.0
        msg.twist.angular.z = 0.0

        self.cmd_pub.publish(msg)

    def publish_gate_command(self):
        gate = self.last_gate

        # Navigation target is ALWAYS the geometric midpoint
        # of the two currently published gate posts.
        #
        # Do not trust a separately tracked/smoothed center for
        # steering.
        left_x = float(gate.left_marker.x)
        left_y = float(gate.left_marker.y)

        right_x = float(gate.right_marker.x)
        right_y = float(gate.right_marker.y)

        x = 0.5 * (
            left_x + right_x
        )

        y = 0.5 * (
            left_y + right_y
        )

        heading_error = math.atan2(
            y,
            x
        )

        yaw = (
            self.yaw_kp
            * heading_error
        )

        yaw = max(
            -self.max_yaw_rate,
            min(
                self.max_yaw_rate,
                yaw
            )
        )

        forward_limit = math.radians(
            self.forward_angle_limit_deg
        )

        forward = (
            self.forward_speed
            if abs(heading_error)
                <= forward_limit
            else 0.0
        )

        msg = TwistStamped()

        msg.header.stamp = (
            self.get_clock().now().to_msg()
        )

        msg.header.frame_id = 'base_link'

        msg.twist.linear.x = float(
            forward
        )

        msg.twist.linear.y = 0.0
        msg.twist.linear.z = 0.0

        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0

        msg.twist.angular.z = float(
            yaw
        )

        self.cmd_pub.publish(msg)

    def update(self):
        if not self.enabled:
            self.publish_zero()
            return

        if self.mission_complete:
            self.publish_zero()
            return

        if self.gate_is_fresh():
            self.publish_gate_command()
            return

        age = self.gate_age()

        # A disappearing gate can only count as passed after:
        #
        # 1. passage was armed,
        # 2. vehicle is actually ARMED + GUIDED,
        # 3. boat physically travelled >= pass_min_travel.
        if (
            self.passage_armed
            and age is not None
            and age >= self.pass_loss_timeout
        ):
            travel = self.travel_since_pass_arm()
            approach = self.gate_approach_since_pass_arm()

            if (
                self.vehicle_motion_ready()
                and travel >= self.pass_min_travel
                and self.lidar_approach_confirmed
            ):
                self.finish_current_gate(
                    f'gate disappeared after '
                    f'{travel:.2f} m vehicle travel and '
                    f'{approach:.2f} m LiDAR approach'
                )

                self.publish_zero()
                return

            if not self.lidar_approach_confirmed:
                self.publish_state(
                    f'PASS_BLOCKED_GATE_{self.current_gate}: '
                    f'no confirmed close LiDAR approach; '
                    f'approach={approach:.2f} m, '
                    f'close_hits={self.close_gate_hits}/'
                    f'{self.pass_close_confirm_hits}'
                )
            elif travel < self.pass_min_travel:
                self.publish_state(
                    f'PASS_BLOCKED_GATE_{self.current_gate}: '
                    f'vehicle travel={travel:.2f}/'
                    f'{self.pass_min_travel:.2f} m'
                )
            else:
                self.publish_state(
                    f'PASS_BLOCKED_GATE_{self.current_gate}: '
                    f'vehicle not ARMED + GUIDED'
                )

        # If perception disappears but the boat has not moved
        # enough, this is NOT a gate pass.
        self.publish_zero()


def main(args=None):
    rclpy.init(args=args)

    node = TwoGateFollower()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.publish_zero()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
