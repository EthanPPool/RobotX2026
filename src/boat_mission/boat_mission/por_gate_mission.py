#!/usr/bin/env python3
from enum import Enum, auto
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from boat_interfaces.msg import Gate, NavigationTarget


class MissionState(Enum):
    SEEK_GATE = auto()
    APPROACH_GATE = auto()
    CLEAR_GATE = auto()
    COMPLETE = auto()


class PorGateMission(Node):
    """Two-gate Proof-of-Readiness state machine.

    The mission layer publishes navigation objectives only. It never publishes
    MAVROS setpoints directly.
    """

    def __init__(self):
        super().__init__('por_gate_mission')

        self.gate_topic = self.declare_parameter('gate_topic', '/perception/gate').value
        self.target_topic = self.declare_parameter('target_topic', '/mission/target').value
        self.state_topic = self.declare_parameter('state_topic', '/mission/state').value

        self.gate_timeout = float(self.declare_parameter('gate_timeout', 0.60).value)
        self.gate_confirm_duration = float(
            self.declare_parameter('gate_confirm_duration', 0.75).value
        )
        self.min_gate_confidence = float(
            self.declare_parameter('min_gate_confidence', 0.60).value
        )
        self.gate_pass_x = float(self.declare_parameter('gate_pass_x', 1.10).value)
        self.clear_duration = float(self.declare_parameter('clear_duration', 5.0).value)
        self.clear_target_x = float(self.declare_parameter('clear_target_x', 3.0).value)
        self.approach_speed = float(self.declare_parameter('approach_speed', 0.40).value)
        self.near_gate_speed = float(self.declare_parameter('near_gate_speed', 0.28).value)
        self.near_gate_x = float(self.declare_parameter('near_gate_x', 3.0).value)
        self.clear_speed = float(self.declare_parameter('clear_speed', 0.35).value)
        self.gates_required = int(self.declare_parameter('gates_required', 2).value)

        self.state = MissionState.SEEK_GATE
        self.gates_passed = 0
        self.last_gate = None
        self.last_gate_time = None
        self.gate_candidate_since = None
        self.clear_start_time = None

        self.target_pub = self.create_publisher(NavigationTarget, self.target_topic, 10)
        self.state_pub = self.create_publisher(String, self.state_topic, 10)
        self.gate_sub = self.create_subscription(Gate, self.gate_topic, self.gate_callback, 10)
        self.timer = self.create_timer(0.10, self.update)

        self.publish_state('SEEK_GATE: waiting for gate 1')

    def gate_callback(self, msg: Gate) -> None:
        now = self.get_clock().now()

        # Reject weak detections and reset acquisition confirmation.
        if float(msg.confidence) < self.min_gate_confidence:
            if self.state == MissionState.SEEK_GATE:
                self.last_gate = None
                self.last_gate_time = None
                self.gate_candidate_since = None
            return

        # A usable gate center must be finite and in front of the boat.
        if (
            not math.isfinite(float(msg.center.x))
            or not math.isfinite(float(msg.center.y))
            or float(msg.center.x) <= 0.0
        ):
            if self.state == MissionState.SEEK_GATE:
                self.last_gate = None
                self.last_gate_time = None
                self.gate_candidate_since = None
            return

        self.last_gate = msg
        self.last_gate_time = now

        # Do NOT move on the first detection. Start confirmation timer.
        if self.state == MissionState.SEEK_GATE and self.gate_candidate_since is None:
            self.gate_candidate_since = now
            self.publish_state(
                f'SEEK_GATE: confirming gate {self.gates_passed + 1}'
            )

    def gate_is_fresh(self) -> bool:
        if self.last_gate is None or self.last_gate_time is None:
            return False
        age = (self.get_clock().now() - self.last_gate_time).nanoseconds / 1e9
        return age <= self.gate_timeout

    def update(self) -> None:
        if self.state == MissionState.COMPLETE:
            self.publish_stop()
            return

        if self.state in (MissionState.SEEK_GATE, MissionState.APPROACH_GATE):
            if not self.gate_is_fresh():
                self.state = MissionState.SEEK_GATE
                self.gate_candidate_since = None
                self.publish_stop()
                self.publish_state(
                    f'SEEK_GATE: no fresh valid gate; FAIL-STOP; passed {self.gates_passed}/{self.gates_required}'
                )
                return

            gate = self.last_gate

            # Require a stable gate before generating any movement target.
            if self.state == MissionState.SEEK_GATE:
                if self.gate_candidate_since is None:
                    self.publish_stop()
                    return

                confirm_age = (
                    self.get_clock().now() - self.gate_candidate_since
                ).nanoseconds / 1e9

                if confirm_age < self.gate_confirm_duration:
                    self.publish_stop()
                    return

                self.state = MissionState.APPROACH_GATE
                self.publish_state(
                    f'APPROACH_GATE: confirmed gate {self.gates_passed + 1}'
                )

            if gate.center.x <= self.gate_pass_x:
                self.state = MissionState.CLEAR_GATE
                self.clear_start_time = self.get_clock().now()
                self.last_gate = None
                self.last_gate_time = None
                self.gate_candidate_since = None
                self.publish_state(f'CLEAR_GATE: crossing gate {self.gates_passed + 1}')
                self.publish_clear_target()
                return

            speed = self.near_gate_speed if gate.center.x <= self.near_gate_x else self.approach_speed
            target = NavigationTarget()
            target.header = gate.header
            target.header.frame_id = 'base_link'
            target.target = gate.center
            target.desired_speed = float(speed)
            target.stop = False
            self.target_pub.publish(target)
            return

        if self.state == MissionState.CLEAR_GATE:
            elapsed = (self.get_clock().now() - self.clear_start_time).nanoseconds / 1e9
            if elapsed < self.clear_duration:
                self.publish_clear_target()
                return

            self.gates_passed += 1
            if self.gates_passed >= self.gates_required:
                self.state = MissionState.COMPLETE
                self.publish_state(f'COMPLETE: passed {self.gates_passed}/{self.gates_required} gates')
                self.publish_stop()
            else:
                self.state = MissionState.SEEK_GATE
                self.gate_candidate_since = None
                self.publish_state(f'SEEK_GATE: waiting for gate {self.gates_passed + 1}')
                self.publish_stop()

    def publish_clear_target(self) -> None:
        target = NavigationTarget()
        target.header.stamp = self.get_clock().now().to_msg()
        target.header.frame_id = 'base_link'
        target.target.x = self.clear_target_x
        target.target.y = 0.0
        target.target.z = 0.0
        target.desired_speed = self.clear_speed
        target.stop = False
        self.target_pub.publish(target)

    def publish_stop(self) -> None:
        target = NavigationTarget()
        target.header.stamp = self.get_clock().now().to_msg()
        target.header.frame_id = 'base_link'
        target.stop = True
        target.desired_speed = 0.0
        self.target_pub.publish(target)

    def publish_state(self, text: str) -> None:
        msg = String()
        msg.data = text
        self.state_pub.publish(msg)
        self.get_logger().info(text)


def main(args=None):
    rclpy.init(args=args)
    node = PorGateMission()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
