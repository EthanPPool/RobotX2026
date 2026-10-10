#!/usr/bin/env python3

"""Task 1 mission executive for the RobotX BlueBoat.

This node owns mission sequencing and gate-passage geometry only. It publishes
base_link-relative NavigationTarget objectives and never publishes velocity
setpoints or changes autopilot modes directly.
"""

import json
import math
from enum import Enum, auto

import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped, Vector3Stamped
from mavros_msgs.msg import State
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float64, String
from std_srvs.srv import SetBool, Trigger

from boat_interfaces.msg import Gate, GateArray, NavigationTarget
from boat_perception.pose_history import PoseHistory, stamp_seconds, to_map
from boat_mission.gate_selection import GateGeometry, GateSelector


class MissionPhase(Enum):
    WAIT_GATE = auto()
    TRACK_GATE = auto()
    PASS_GATE = auto()
    COMPLETE = auto()


class Task1GateMission(Node):
    """Two-gate Task 1 mission state machine.

    TRACK_GATE stores each fresh LiDAR gate measurement in the fixed MAVROS
    local frame. PASS_GATE freezes that geometry and publishes a target beyond
    the saved gate line until the boat has crossed inside the valid corridor
    and cleared the line by ``pass_clear_distance``.
    """

    def __init__(self):
        super().__init__('task1_gate_mission')

        self.gate_topic = self.declare_parameter(
            'gate_topic', '/perception/gate'
        ).value
        self.gates_topic = self.declare_parameter('gates_topic', '/perception/gates').value
        self.use_gate_candidates = bool(self.declare_parameter('use_gate_candidates', True).value)
        self.gate_measurement_timeout = float(self.declare_parameter('gate_measurement_timeout', 2.5).value)
        center_limit = float(self.declare_parameter('gate_association_distance', 0.8).value)
        marker_limit = float(self.declare_parameter('gate_marker_association_distance', 1.0).value)
        width_limit = float(self.declare_parameter('gate_width_association_tolerance', 0.4).value)
        bearing_limit = float(self.declare_parameter('gate_acquisition_angle_deg', 45.0).value)
        if (not all(math.isfinite(v) for v in (self.gate_measurement_timeout,
                center_limit, marker_limit, width_limit, bearing_limit)) or
                min(self.gate_measurement_timeout, center_limit, marker_limit, width_limit) <= 0 or
                not 0 < bearing_limit < 90):
            raise ValueError('Invalid gate selection or freshness parameters')
        self.gate_selector = GateSelector(center_limit, marker_limit, width_limit, bearing_limit)
        self.pose_history = PoseHistory()
        self.selected_measurement_stamp = None
        self.gate_candidate_count = 0
        self.gate_selection_reason = 'NO_CANDIDATES'
        self.target_topic = self.declare_parameter(
            'target_topic', '/mission/target'
        ).value
        self.state_topic = self.declare_parameter(
            'state_topic', '/mission/state'
        ).value
        self.diagnostics_topic = self.declare_parameter(
            'diagnostics_topic', '/mission/diagnostics'
        ).value
        self.vehicle_state_topic = self.declare_parameter(
            'vehicle_state_topic', '/mavros/state'
        ).value
        self.local_position_topic = self.declare_parameter(
            'local_position_topic', '/mavros/local_position/pose'
        ).value

        self.enabled = bool(self.declare_parameter('enabled', True).value)
        self.gates_required = int(
            self.declare_parameter('gates_required', 2).value
        )
        self.min_gate_confidence = float(
            self.declare_parameter('min_gate_confidence', 0.75).value
        )

        self.track_speed = float(
            self.declare_parameter('track_speed', 0.12).value
        )
        self.pass_speed = float(
            self.declare_parameter('pass_speed', 0.12).value
        )

        self.pass_commit_distance = float(
            self.declare_parameter('pass_commit_distance', 2.0).value
        )
        self.pass_target_distance = float(
            self.declare_parameter('pass_target_distance', 1.5).value
        )
        self.pass_clear_distance = float(
            self.declare_parameter('pass_clear_distance', 1.0).value
        )
        self.local_pose_timeout = float(
            self.declare_parameter('local_pose_timeout', 0.50).value
        )
        self.pass_edge_margin = float(
            self.declare_parameter('pass_edge_margin', 0.0).value
        )

        if self.gates_required < 1:
            raise ValueError('gates_required must be at least 1')
        if self.track_speed < 0.0 or self.pass_speed < 0.0:
            raise ValueError('track_speed and pass_speed cannot be negative')
        if self.pass_commit_distance <= 0.0:
            raise ValueError('pass_commit_distance must be positive')
        if self.pass_target_distance <= self.pass_clear_distance:
            raise ValueError(
                'pass_target_distance must be greater than pass_clear_distance'
            )
        if self.pass_edge_margin < 0.0:
            raise ValueError('pass_edge_margin cannot be negative')
        if self.local_pose_timeout <= 0.0:
            raise ValueError('local_pose_timeout must be positive')

        self.vehicle_state = None
        self.local_x = None
        self.local_y = None
        self.local_yaw = None
        self.local_pose_time = None

        self.last_gate = None
        self.last_gate_time = None
        self.last_gate_measurement_stamp = None

        self.tracked_port = None
        self.tracked_starboard = None
        self.tracked_midpoint = None
        self.tracked_gate_measurement_time = None

        self.saved_port = None
        self.saved_starboard = None
        self.saved_midpoint = None
        self.saved_gate_tangent = None
        self.saved_gate_normal = None
        self.saved_gate_width = None
        self.saved_usable_half_width = None
        self.saved_pass_target = None
        self.gate_entry_side = None
        self.previous_pass_signed_distance = None
        self.previous_pass_lateral_offset = None
        self.gate_crossed = False
        self.crossing_lateral_offset = None

        self.phase = MissionPhase.WAIT_GATE
        self.gates_passed = 0
        self.current_gate = 1
        self.mission_complete = False
        self.last_state_text = None
        self.last_reason = 'BOOT'
        self.last_target_map = None
        self.last_target_body = None

        self.target_pub = self.create_publisher(
            NavigationTarget, self.target_topic, 10
        )
        self.state_pub = self.create_publisher(String, self.state_topic, 10)
        self.diagnostics_pub = self.create_publisher(
            String, self.diagnostics_topic, 10
        )

        self.debug_target_point_pub = self.create_publisher(
            PointStamped, '/task1/debug/target_point_map', 10
        )
        self.debug_target_vector_pub = self.create_publisher(
            Vector3Stamped, '/task1/debug/target_vector_body', 10
        )
        self.debug_gate_port_pub = self.create_publisher(
            PointStamped, '/task1/debug/gate_port_map', 10
        )
        self.debug_gate_starboard_pub = self.create_publisher(
            PointStamped, '/task1/debug/gate_starboard_map', 10
        )
        self.debug_gate_range_pub = self.create_publisher(
            Float64, '/task1/debug/gate_map_range', 10
        )
        self.debug_signed_distance_pub = self.create_publisher(
            Float64, '/task1/debug/gate_signed_distance', 10
        )

        if self.use_gate_candidates:
            self.create_subscription(GateArray, self.gates_topic, self.gates_callback, 10)
        else:
            self.create_subscription(Gate, self.gate_topic, self.gate_callback, 10)
        self.create_subscription(
            State,
            self.vehicle_state_topic,
            self.vehicle_state_callback,
            10,
        )
        self.create_subscription(
            PoseStamped,
            self.local_position_topic,
            self.local_position_callback,
            qos_profile_sensor_data,
        )

        self.create_service(
            SetBool, '/mission/set_enabled', self.set_enabled_callback
        )
        self.create_service(Trigger, '/mission/reset', self.reset_callback)

        self.timer = self.create_timer(0.05, self.update)
        self.reset_mission()

        self.get_logger().warning(
            'Task 1 gate mission started: mission owns gate sequencing and '
            'passage geometry; boat_control owns velocity generation.'
        )

    def reset_mission(self):
        self.gate_selector.reset()
        self.selected_measurement_stamp = None
        self.gates_passed = 0
        self.current_gate = 1
        self.mission_complete = False
        self.phase = MissionPhase.WAIT_GATE
        self.last_gate = None
        self.last_gate_time = None
        self.last_gate_measurement_stamp = None
        self.clear_tracked_gate_geometry()
        self.clear_saved_gate_geometry()
        self.last_target_map = None
        self.last_target_body = None
        self.publish_state('WAIT_GATE_1: waiting for confirmed gate 1')

    def set_enabled_callback(self, request, response):
        self.enabled = bool(request.data)

        if not self.enabled:
            if self.phase != MissionPhase.COMPLETE:
                self.last_gate = None
                self.last_gate_time = None
                self.last_gate_measurement_stamp = None
                self.clear_tracked_gate_geometry()
                self.clear_saved_gate_geometry()
                self.phase = MissionPhase.WAIT_GATE
            self.publish_stop('MISSION_DISABLED')
            self.publish_state(
                'DISABLED: mission stopped; saved passage discarded'
            )
        elif self.phase == MissionPhase.COMPLETE:
            self.publish_state('MISSION_COMPLETE: mission enabled; target stopped')
        else:
            self.last_gate = None
            self.last_gate_time = None
            self.last_gate_measurement_stamp = None
            self.clear_tracked_gate_geometry()
            self.clear_saved_gate_geometry()
            self.phase = MissionPhase.WAIT_GATE
            self.publish_state(
                f'WAIT_GATE_{self.current_gate}: mission enabled; reacquiring gate'
            )

        response.success = True
        response.message = (
            'Task 1 mission enabled' if self.enabled else 'Task 1 mission disabled'
        )
        return response

    def reset_callback(self, _request, response):
        self.reset_mission()
        self.publish_stop('RESET_MISSION')
        response.success = True
        response.message = 'Task 1 mission reset to gate 1'
        return response

    def gate_is_valid(self, msg):
        confidence = float(msg.confidence)
        values = (
            confidence,
            float(msg.center.x),
            float(msg.center.y),
            float(msg.left_marker.x),
            float(msg.left_marker.y),
            float(msg.right_marker.x),
            float(msg.right_marker.y),
        )
        return (
            all(math.isfinite(value) for value in values)
            and self.min_gate_confidence <= confidence <= 1.0
            and float(msg.center.x) > 0.0
        )

    def vehicle_state_callback(self, msg):
        self.vehicle_state = msg

    def local_position_callback(self, msg):
        q = msg.pose.orientation
        if not all(math.isfinite(v) for v in (msg.pose.position.x, msg.pose.position.y,
                                              q.x, q.y, q.z, q.w)):
            return
        self.local_x = float(msg.pose.position.x)
        self.local_y = float(msg.pose.position.y)

        q = msg.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.local_yaw = math.atan2(siny_cosp, cosy_cosp)
        self.local_pose_time = self.get_clock().now()
        self.pose_history.add(stamp_seconds(msg.header.stamp), self.local_x,
                              self.local_y, self.local_yaw)

    def local_pose_is_available(self):
        return (
            self.local_x is not None
            and self.local_y is not None
            and self.local_yaw is not None
            and self.local_pose_time is not None
        )

    def local_pose_age(self):
        if self.local_pose_time is None:
            return None
        return (
            self.get_clock().now() - self.local_pose_time
        ).nanoseconds / 1e9

    def local_pose_is_fresh(self, timeout=None):
        if timeout is None:
            timeout = self.local_pose_timeout
        age = self.local_pose_age()
        return (
            self.local_pose_is_available()
            and age is not None
            and age <= timeout
        )

    def body_point_to_map(self, x_body, y_body):
        if not self.local_pose_is_available():
            return None
        c = math.cos(self.local_yaw)
        s = math.sin(self.local_yaw)
        return (
            self.local_x + c * x_body - s * y_body,
            self.local_y + s * x_body + c * y_body,
        )

    def map_point_to_body(self, x_map, y_map):
        if not self.local_pose_is_available():
            return None
        dx = x_map - self.local_x
        dy = y_map - self.local_y
        c = math.cos(self.local_yaw)
        s = math.sin(self.local_yaw)
        return (
            c * dx + s * dy,
            -s * dx + c * dy,
        )

    def clear_tracked_gate_geometry(self):
        self.gate_selector.release()
        self.selected_measurement_stamp = None
        self.tracked_port = None
        self.tracked_starboard = None
        self.tracked_midpoint = None
        self.tracked_gate_measurement_time = None

    def update_tracked_gate_geometry(self, gate, measurement_pose=None):
        if not self.local_pose_is_fresh():
            return False

        if measurement_pose is None:
            measurement_pose = self.pose_history.at(stamp_seconds(gate.header.stamp))
        if measurement_pose is None:
            return False
        port_map = to_map((float(gate.left_marker.x), float(gate.left_marker.y)), measurement_pose)
        starboard_map = to_map((float(gate.right_marker.x), float(gate.right_marker.y)), measurement_pose)
        if port_map is None or starboard_map is None:
            return False

        px, py = port_map
        sx, sy = starboard_map
        if math.hypot(sx - px, sy - py) <= 1e-6:
            return False

        self.tracked_port = (px, py)
        self.tracked_starboard = (sx, sy)
        self.tracked_midpoint = (0.5 * (px + sx), 0.5 * (py + sy))
        self.tracked_gate_measurement_time = self.get_clock().now()
        return True

    def tracked_gate_distance(self):
        if not self.local_pose_is_available() or self.tracked_midpoint is None:
            return None
        midpoint_x, midpoint_y = self.tracked_midpoint
        return math.hypot(
            midpoint_x - self.local_x,
            midpoint_y - self.local_y,
        )

    def vehicle_motion_ready(self):
        state = self.vehicle_state
        return (
            state is not None
            and bool(state.connected)
            and bool(state.armed)
            and str(state.mode).upper() == 'GUIDED'
        )

    def commit_gate(self):
        if self.phase != MissionPhase.TRACK_GATE:
            return False
        if not self.local_pose_is_fresh() or not self.vehicle_motion_ready():
            return False
        if (
            self.tracked_port is None
            or self.tracked_starboard is None
            or self.tracked_midpoint is None
        ):
            return False

        px, py = self.tracked_port
        sx, sy = self.tracked_starboard
        midpoint_x, midpoint_y = self.tracked_midpoint

        gate_dx = sx - px
        gate_dy = sy - py
        gate_width = math.hypot(gate_dx, gate_dy)
        if gate_width <= 1e-6:
            return False

        usable_half_width = 0.5 * gate_width - self.pass_edge_margin
        if usable_half_width <= 0.0:
            self.get_logger().warning(
                f'Refusing gate commit: width={gate_width:.2f} m is too narrow '
                f'for edge margin={self.pass_edge_margin:.2f} m'
            )
            return False

        tangent_x = gate_dx / gate_width
        tangent_y = gate_dy / gate_width
        normal_x = -tangent_y
        normal_y = tangent_x

        boat_to_mid_x = midpoint_x - self.local_x
        boat_to_mid_y = midpoint_y - self.local_y
        if normal_x * boat_to_mid_x + normal_y * boat_to_mid_y < 0.0:
            normal_x *= -1.0
            normal_y *= -1.0

        target_x = midpoint_x + self.pass_target_distance * normal_x
        target_y = midpoint_y + self.pass_target_distance * normal_y

        entry_distance = (
            (self.local_x - midpoint_x) * normal_x
            + (self.local_y - midpoint_y) * normal_y
        )
        entry_lateral = (
            (self.local_x - midpoint_x) * tangent_x
            + (self.local_y - midpoint_y) * tangent_y
        )

        self.saved_port = (px, py)
        self.saved_starboard = (sx, sy)
        self.saved_midpoint = (midpoint_x, midpoint_y)
        self.saved_gate_tangent = (tangent_x, tangent_y)
        self.saved_gate_normal = (normal_x, normal_y)
        self.saved_gate_width = gate_width
        self.saved_usable_half_width = usable_half_width
        self.saved_pass_target = (target_x, target_y)
        self.gate_entry_side = entry_distance
        self.previous_pass_signed_distance = entry_distance
        self.previous_pass_lateral_offset = entry_lateral
        self.gate_crossed = False
        self.crossing_lateral_offset = None

        self.clear_tracked_gate_geometry()
        self.last_gate = None
        self.last_gate_time = None
        self.phase = MissionPhase.PASS_GATE

        self.publish_state(
            f'PASS_GATE_{self.current_gate}: committed from map memory; '
            f'width={gate_width:.2f} m, '
            f'midpoint=({midpoint_x:.2f}, {midpoint_y:.2f}), '
            f'target=({target_x:.2f}, {target_y:.2f})'
        )
        self.publish_diagnostics('PASS_GATE_COMMITTED')
        return True

    def signed_gate_distance(self):
        if (
            not self.local_pose_is_available()
            or self.saved_midpoint is None
            or self.saved_gate_normal is None
        ):
            return None
        midpoint_x, midpoint_y = self.saved_midpoint
        normal_x, normal_y = self.saved_gate_normal
        return (
            (self.local_x - midpoint_x) * normal_x
            + (self.local_y - midpoint_y) * normal_y
        )

    def gate_lateral_offset(self):
        if (
            not self.local_pose_is_available()
            or self.saved_midpoint is None
            or self.saved_gate_tangent is None
        ):
            return None
        midpoint_x, midpoint_y = self.saved_midpoint
        tangent_x, tangent_y = self.saved_gate_tangent
        return (
            (self.local_x - midpoint_x) * tangent_x
            + (self.local_y - midpoint_y) * tangent_y
        )

    def clear_saved_gate_geometry(self):
        self.saved_port = None
        self.saved_starboard = None
        self.saved_midpoint = None
        self.saved_gate_tangent = None
        self.saved_gate_normal = None
        self.saved_gate_width = None
        self.saved_usable_half_width = None
        self.saved_pass_target = None
        self.gate_entry_side = None
        self.previous_pass_signed_distance = None
        self.previous_pass_lateral_offset = None
        self.gate_crossed = False
        self.crossing_lateral_offset = None

    def abort_to_wait_gate(self, reason):
        self.clear_saved_gate_geometry()
        self.clear_tracked_gate_geometry()
        self.last_gate = None
        self.last_gate_time = None
        self.last_gate_measurement_stamp = None
        self.phase = MissionPhase.WAIT_GATE
        self.publish_state(
            f'WAIT_GATE_{self.current_gate}: passage aborted; {reason}'
        )

    def gate_callback(self, msg):
        # Legacy input is explicit opt-in; both paths enforce mission identity.
        array = GateArray()
        array.header = msg.header
        array.gates = [msg]
        self.gates_callback(array)

    def gates_callback(self, array):
        if not self.enabled or self.mission_complete:
            return
        if self.phase == MissionPhase.PASS_GATE:
            return
        self.gate_candidate_count = len(array.gates)
        if not self.local_pose_is_fresh() or array.header.frame_id != 'base_link':
            self.gate_selection_reason = 'POSE_OR_FRAME_UNAVAILABLE'
            return
        stamp = stamp_seconds(array.header.stamp)
        now = self.get_clock().now().nanoseconds / 1e9
        if stamp <= 0 or not 0 <= now - stamp <= self.gate_measurement_timeout:
            self.gate_selection_reason = 'STALE_MEASUREMENT'
            return
        if self.selected_measurement_stamp is not None and stamp <= self.selected_measurement_stamp:
            return
        pose = self.pose_history.at(stamp)
        if pose is None:
            self.gate_selection_reason = 'MEASUREMENT_POSE_UNAVAILABLE'
            return
        geometries, messages = [], {}
        for msg in array.gates:
            if (not self.gate_is_valid(msg) or msg.header.frame_id != 'base_link' or
                    stamp_seconds(msg.header.stamp) != stamp):
                continue
            port = to_map((float(msg.left_marker.x), float(msg.left_marker.y)), pose)
            starboard = to_map((float(msg.right_marker.x), float(msg.right_marker.y)), pose)
            geometry = GateGeometry(port, starboard, float(msg.confidence), stamp)
            if (not all(math.isfinite(v) for v in (*port, *starboard, msg.width)) or
                    geometry.width <= 0 or abs(geometry.width - msg.width) > 0.25):
                continue
            geometries.append(geometry)
            messages[id(geometry)] = msg
        chosen = self.gate_selector.choose(geometries, (self.local_x, self.local_y, self.local_yaw))
        if chosen is None:
            self.gate_selection_reason = ('LOCKED_GATE_NOT_OBSERVED' if
                self.gate_selector.anchor is not None else 'NO_ELIGIBLE_GATE')
            return
        msg = messages[id(chosen)]
        self.gate_selection_reason = 'MATCHED_LOCKED_GATE'
        self.last_gate_measurement_stamp = (int(array.header.stamp.sec), int(array.header.stamp.nanosec))
        self.selected_measurement_stamp = stamp

        self.last_gate = msg
        self.last_gate_time = self.get_clock().now()

        if not self.update_tracked_gate_geometry(msg, pose):
            return

        if self.phase == MissionPhase.WAIT_GATE:
            self.phase = MissionPhase.TRACK_GATE

        gate_range = self.tracked_gate_distance()
        if self.phase == MissionPhase.TRACK_GATE and gate_range is not None:
            self.publish_state(
                f'TRACK_GATE_{self.current_gate}: map_range={gate_range:.2f} m'
            )

    def finish_current_gate(self, reason):
        finished_gate = self.current_gate
        geometry = (None if self.saved_port is None or self.saved_starboard is None else
                    GateGeometry(self.saved_port, self.saved_starboard, 1.0,
                                 self.selected_measurement_stamp or 0.0))
        self.gate_selector.complete(geometry)
        self.gates_passed += 1
        self.last_gate = None
        self.last_gate_time = None
        self.last_gate_measurement_stamp = None
        self.clear_tracked_gate_geometry()
        self.clear_saved_gate_geometry()

        if self.gates_passed >= self.gates_required:
            self.mission_complete = True
            self.phase = MissionPhase.COMPLETE
            self.publish_state(
                f'MISSION_COMPLETE: passed '
                f'{self.gates_passed}/{self.gates_required} gates; {reason}'
            )
            return

        self.current_gate = self.gates_passed + 1
        self.phase = MissionPhase.WAIT_GATE
        self.publish_state(
            f'WAIT_GATE_{self.current_gate}: gate {finished_gate} passed; {reason}'
        )

    def publish_state(self, text):
        if text == self.last_state_text:
            return
        self.last_state_text = text
        msg = String()
        msg.data = text
        self.state_pub.publish(msg)
        self.get_logger().info(text)

    def publish_stop(self, reason='STOP_REQUESTED'):
        target = NavigationTarget()
        target.header.stamp = self.get_clock().now().to_msg()
        target.header.frame_id = 'base_link'
        target.desired_speed = 0.0
        target.stop = True
        self.target_pub.publish(target)
        self.last_target_map = None
        self.last_target_body = None
        self.publish_diagnostics(reason)

    def publish_map_target(self, target_map, speed, reason):
        if target_map is None or not self.local_pose_is_fresh():
            self.publish_stop(f'{reason}_TARGET_UNAVAILABLE')
            return False

        target_body = self.map_point_to_body(*target_map)
        if target_body is None:
            self.publish_stop(f'{reason}_TRANSFORM_FAILED')
            return False

        x_body, y_body = target_body
        if not all(math.isfinite(v) for v in (x_body, y_body, speed)):
            self.publish_stop(f'{reason}_NONFINITE_TARGET')
            return False

        target = NavigationTarget()
        target.header.stamp = self.get_clock().now().to_msg()
        target.header.frame_id = 'base_link'
        target.target.x = float(x_body)
        target.target.y = float(y_body)
        target.target.z = 0.0
        target.desired_speed = float(max(0.0, speed))
        target.stop = False
        self.target_pub.publish(target)

        self.last_target_map = tuple(target_map)
        self.last_target_body = tuple(target_body)
        self.publish_diagnostics(reason)
        return True

    def publish_debug_telemetry(self):
        stamp = self.get_clock().now().to_msg()

        port = self.tracked_port or self.saved_port
        starboard = self.tracked_starboard or self.saved_starboard
        target_map = (
            self.saved_pass_target
            if self.phase == MissionPhase.PASS_GATE
            else self.tracked_midpoint
        )

        if port is not None:
            msg = PointStamped()
            msg.header.stamp = stamp
            msg.header.frame_id = 'map'
            msg.point.x, msg.point.y = port
            self.debug_gate_port_pub.publish(msg)

        if starboard is not None:
            msg = PointStamped()
            msg.header.stamp = stamp
            msg.header.frame_id = 'map'
            msg.point.x, msg.point.y = starboard
            self.debug_gate_starboard_pub.publish(msg)

        if target_map is not None:
            msg = PointStamped()
            msg.header.stamp = stamp
            msg.header.frame_id = 'map'
            msg.point.x, msg.point.y = target_map
            self.debug_target_point_pub.publish(msg)

            if self.local_pose_is_available():
                target_body = self.map_point_to_body(*target_map)
                if target_body is not None:
                    vec = Vector3Stamped()
                    vec.header.stamp = stamp
                    vec.header.frame_id = 'base_link'
                    vec.vector.x, vec.vector.y = target_body
                    self.debug_target_vector_pub.publish(vec)

        gate_range = None
        if self.phase == MissionPhase.TRACK_GATE:
            gate_range = self.tracked_gate_distance()
        elif (
            self.phase == MissionPhase.PASS_GATE
            and self.saved_midpoint is not None
            and self.local_pose_is_available()
        ):
            gate_range = math.hypot(
                self.saved_midpoint[0] - self.local_x,
                self.saved_midpoint[1] - self.local_y,
            )
        if gate_range is not None:
            msg = Float64()
            msg.data = float(gate_range)
            self.debug_gate_range_pub.publish(msg)

        signed_distance = self.signed_gate_distance()
        if signed_distance is not None:
            msg = Float64()
            msg.data = float(signed_distance)
            self.debug_signed_distance_pub.publish(msg)

    @staticmethod
    def xy_values(point):
        if point is None:
            return None, None
        return float(point[0]), float(point[1])

    def publish_diagnostics(self, reason):
        try:
            self.last_reason = str(reason)

            gate_map_range = None
            if self.phase == MissionPhase.TRACK_GATE:
                gate_map_range = self.tracked_gate_distance()
            elif (
                self.phase == MissionPhase.PASS_GATE
                and self.saved_midpoint is not None
                and self.local_pose_is_available()
            ):
                gate_map_range = math.hypot(
                    self.saved_midpoint[0] - self.local_x,
                    self.saved_midpoint[1] - self.local_y,
                )

            signed_distance = self.signed_gate_distance()
            lateral_offset = self.gate_lateral_offset()
            distance_beyond_gate = (
                None if signed_distance is None else max(0.0, signed_distance)
            )

            tracked_age = None
            if self.tracked_gate_measurement_time is not None:
                tracked_age = (
                    self.get_clock().now() - self.tracked_gate_measurement_time
                ).nanoseconds / 1e9

            tracked_port_x, tracked_port_y = self.xy_values(self.tracked_port)
            tracked_starboard_x, tracked_starboard_y = self.xy_values(
                self.tracked_starboard
            )
            tracked_midpoint_x, tracked_midpoint_y = self.xy_values(
                self.tracked_midpoint
            )
            saved_port_x, saved_port_y = self.xy_values(self.saved_port)
            saved_starboard_x, saved_starboard_y = self.xy_values(
                self.saved_starboard
            )
            saved_midpoint_x, saved_midpoint_y = self.xy_values(
                self.saved_midpoint
            )
            tangent_x, tangent_y = self.xy_values(self.saved_gate_tangent)
            normal_x, normal_y = self.xy_values(self.saved_gate_normal)
            pass_target_x, pass_target_y = self.xy_values(self.saved_pass_target)
            target_map_x, target_map_y = self.xy_values(self.last_target_map)
            target_body_x, target_body_y = self.xy_values(self.last_target_body)

            target_distance = None
            if self.last_target_body is not None:
                target_distance = math.hypot(*self.last_target_body)

            data = {
                'mission_reason': self.last_reason,
                'mission_phase': self.phase.name,
                'current_gate': int(self.current_gate),
                'gates_passed': int(self.gates_passed),
                'mission_complete': bool(self.mission_complete),
                'mission_enabled': bool(self.enabled),
                'local_pose_age_s': self.local_pose_age(),
                'local_pose_fresh': bool(self.local_pose_is_fresh()),
                'tracked_gate_measurement_age_s': tracked_age,
                'gate_candidate_count': self.gate_candidate_count,
                'gate_selection_reason': self.gate_selection_reason,
                'gate_identity_locked': (self.gate_selector.anchor is not None or
                                         self.phase == MissionPhase.PASS_GATE),
                'completed_gate_exclusions': len(self.gate_selector.passed),
                'gate_map_range_m': gate_map_range,
                'gate_signed_distance_m': signed_distance,
                'gate_lateral_offset_m': lateral_offset,
                'distance_beyond_gate_m': distance_beyond_gate,
                'tracked_port_x': tracked_port_x,
                'tracked_port_y': tracked_port_y,
                'tracked_starboard_x': tracked_starboard_x,
                'tracked_starboard_y': tracked_starboard_y,
                'tracked_midpoint_x': tracked_midpoint_x,
                'tracked_midpoint_y': tracked_midpoint_y,
                'saved_port_x': saved_port_x,
                'saved_port_y': saved_port_y,
                'saved_starboard_x': saved_starboard_x,
                'saved_starboard_y': saved_starboard_y,
                'saved_midpoint_x': saved_midpoint_x,
                'saved_midpoint_y': saved_midpoint_y,
                'pass_tangent_x': tangent_x,
                'pass_tangent_y': tangent_y,
                'pass_normal_x': normal_x,
                'pass_normal_y': normal_y,
                'saved_gate_width_m': self.saved_gate_width,
                'usable_half_width': self.saved_usable_half_width,
                'pass_target_x': pass_target_x,
                'pass_target_y': pass_target_y,
                'gate_entry_side_m': self.gate_entry_side,
                'previous_pass_signed_distance_m': (
                    self.previous_pass_signed_distance
                ),
                'previous_pass_lateral_offset_m': (
                    self.previous_pass_lateral_offset
                ),
                'gate_crossed': bool(self.gate_crossed),
                'crossing_lateral_offset_m': self.crossing_lateral_offset,
                'target_map_x': target_map_x,
                'target_map_y': target_map_y,
                'target_body_x': target_body_x,
                'target_body_y': target_body_y,
                'target_distance': target_distance,
            }

            msg = String()
            msg.data = json.dumps(data, separators=(',', ':'), allow_nan=False)
            self.diagnostics_pub.publish(msg)
        except (TypeError, ValueError) as exc:
            self.get_logger().warning(f'Mission diagnostics publish failed: {exc}')

    def update(self):
        self.publish_debug_telemetry()

        if not self.enabled:
            self.publish_stop('MISSION_DISABLED')
            return

        if self.phase == MissionPhase.COMPLETE:
            self.publish_stop('MISSION_COMPLETE')
            return

        if self.phase == MissionPhase.PASS_GATE:
            if not self.vehicle_motion_ready():
                self.publish_state(
                    f'PASS_GATE_{self.current_gate}_BLOCKED: '
                    'vehicle not ARMED + GUIDED'
                )
                self.publish_stop('PASS_VEHICLE_NOT_READY')
                return

            if not self.local_pose_is_fresh():
                age = self.local_pose_age()
                age_text = 'none' if age is None else f'{age:.2f}s'
                self.publish_state(
                    f'PASS_GATE_{self.current_gate}_BLOCKED: '
                    f'local pose stale ({age_text})'
                )
                self.publish_stop('PASS_LOCAL_POSE_STALE')
                return

            signed_distance = self.signed_gate_distance()
            lateral_offset = self.gate_lateral_offset()
            if signed_distance is None or lateral_offset is None:
                self.publish_stop('PASS_GEOMETRY_UNAVAILABLE')
                return

            if not self.gate_crossed:
                previous_signed = self.previous_pass_signed_distance
                previous_lateral = self.previous_pass_lateral_offset

                if (
                    previous_signed is not None
                    and previous_signed < 0.0
                    and signed_distance >= 0.0
                ):
                    denominator = signed_distance - previous_signed
                    fraction = (
                        -previous_signed / denominator
                        if abs(denominator) > 1e-9
                        else 1.0
                    )
                    crossing_lateral = (
                        lateral_offset
                        if previous_lateral is None
                        else previous_lateral
                        + fraction * (lateral_offset - previous_lateral)
                    )

                    if (
                        self.saved_usable_half_width is None
                        or abs(crossing_lateral) > self.saved_usable_half_width
                    ):
                        self.abort_to_wait_gate(
                            'crossed outside safe gate corridor; '
                            f'lateral={crossing_lateral:.2f} m'
                        )
                        self.publish_stop('PASS_CROSSED_OUTSIDE_CORRIDOR')
                        return

                    self.gate_crossed = True
                    self.crossing_lateral_offset = crossing_lateral
                    self.publish_state(
                        f'CROSSED_GATE_{self.current_gate}: '
                        f'lateral_offset={crossing_lateral:.2f} m'
                    )

                self.previous_pass_signed_distance = signed_distance
                self.previous_pass_lateral_offset = lateral_offset

            if self.gate_crossed and signed_distance >= self.pass_clear_distance:
                self.finish_current_gate(
                    'crossed saved gate line and cleared '
                    f'{signed_distance:.2f} m'
                )
                self.publish_stop('PASS_GATE_CLEARED')
                return

            self.publish_map_target(
                self.saved_pass_target,
                self.pass_speed,
                'PASSAGE_TARGET',
            )
            return

        if self.phase == MissionPhase.WAIT_GATE:
            self.publish_stop('WAIT_GATE')
            return

        if self.phase == MissionPhase.TRACK_GATE:
            if self.tracked_midpoint is None:
                self.phase = MissionPhase.WAIT_GATE
                self.publish_state(
                    f'WAIT_GATE_{self.current_gate}: no remembered gate geometry'
                )
                self.publish_stop('TRACK_GEOMETRY_MISSING')
                return

            # Preserve identity during a brief gap, but do not keep approaching
            # a remembered gate indefinitely when its measurements disappear.
            age = (None if self.selected_measurement_stamp is None else
                   self.get_clock().now().nanoseconds / 1e9 - self.selected_measurement_stamp)
            if age is None or not 0 <= age <= self.gate_measurement_timeout:
                self.publish_stop('TRACK_GATE_MEASUREMENT_STALE')
                return

            if not self.local_pose_is_fresh():
                age = self.local_pose_age()
                age_text = 'none' if age is None else f'{age:.2f}s'
                self.publish_state(
                    f'TRACK_GATE_{self.current_gate}_BLOCKED: '
                    f'local pose stale ({age_text})'
                )
                self.publish_stop('TRACK_LOCAL_POSE_STALE')
                return

            gate_range = self.tracked_gate_distance()
            if gate_range is None:
                self.publish_stop('TRACK_RANGE_UNAVAILABLE')
                return

            self.publish_state(
                f'TRACK_GATE_{self.current_gate}: map_range={gate_range:.2f} m'
            )

            if gate_range <= self.pass_commit_distance and self.commit_gate():
                self.publish_stop('PASS_GATE_COMMITTED')
                return

            self.publish_map_target(
                self.tracked_midpoint,
                self.track_speed,
                'TRACKING_GATE',
            )
            return

        self.publish_stop('UNEXPECTED_STATE')


def main(args=None):
    rclpy.init(args=args)
    node = Task1GateMission()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_stop('SHUTDOWN')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
