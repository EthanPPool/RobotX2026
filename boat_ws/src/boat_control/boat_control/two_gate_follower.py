#!/usr/bin/env python3

import json
import math
from enum import Enum, auto
import rclpy
from geometry_msgs.msg import (
    PointStamped,
    PoseStamped,
    TwistStamped,
    Vector3Stamped,
)

from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import (
    Float64,
    String,
)

from std_srvs.srv import SetBool, Trigger

from boat_interfaces.msg import Gate

# =============================================================================
# TASK 1 MISSION STATES
# =============================================================================
# These are explicit controller states rather than inferring mission state
# from several independent Boolean variables.
#
# WAIT_GATE:
#     No usable gate is currently available. Boat commands zero velocity.
#
# TRACK_GATE:
#     A gate is visible. Steering uses the live LiDAR-derived gate midpoint.
#
# PASS_GATE:
#     The gate geometry has been saved into the fixed MAVROS "map" frame.
#     The boat drives toward a fixed target beyond the gate and no longer
#     depends on continued LiDAR visibility of that gate.
#
# COMPLETE:
#     Required gates have been passed. Boat commands zero velocity.
# =============================================================================
class MissionPhase(Enum):
    WAIT_GATE = auto()
    TRACK_GATE = auto()
    PASS_GATE = auto()
    COMPLETE = auto()

class TwoGateFollower(Node):
    """RobotX Task 1 two-gate controller.

    TRACK_GATE uses the live LiDAR-derived midpoint of a detected gate.

    PASS_GATE will snapshot the gate geometry into the fixed MAVROS
    local "map" frame, calculate a target beyond the gate, and use
    local vehicle pose to verify that the boat physically crosses
    and clears the saved gate line.

    Gate passage is never inferred solely from loss of perception
    or acquisition of a farther gate.
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

        # ---------------------------------------------------------------------
        # FIXED-FRAME GATE PASSAGE PARAMETERS
        # ---------------------------------------------------------------------

        # Distance from the live gate midpoint at which we stop relying
        # exclusively on perception and snapshot the gate into the map frame.
        self.pass_commit_distance = float(
            self.declare_parameter(
                'pass_commit_distance',
                2.0
            ).value
        )

        # Fixed target distance beyond the saved gate line.
        self.pass_target_distance = float(
            self.declare_parameter(
                'pass_target_distance',
                1.5
            ).value
        )

        # Boat must travel at least this far beyond the saved gate line
        # before the gate is declared completely cleared.
        self.pass_clear_distance = float(
            self.declare_parameter(
                'pass_clear_distance',
                1.0
            ).value
        )

        # Maximum allowed age of local odometry during PASS_GATE.
        self.local_pose_timeout = float(
            self.declare_parameter(
                'local_pose_timeout',
                0.50
            ).value
        )
        # Minimum clearance from either gate post when the boat center
        # crosses the saved gate line.
        #
        # IMPORTANT:
        # Before water testing this should be set to approximately:
        #
        #     half the boat beam + desired safety buffer
        #
        # Leave at 0.0 for the initial synthetic/cart geometry tests if
        # the physical boat clearance has not yet been measured.
        self.pass_edge_margin = float(
            self.declare_parameter(
                'pass_edge_margin',
                0.0
            ).value
        )

        # Mode requested after the final gate has been crossed and cleared.
        # LOITER allows ArduRover to hold position instead of drifting away.
        self.complete_mode = str(
            self.declare_parameter(
                'complete_mode',
                'LOITER'
            ).value
        ).upper()

        # Do not flood MAVROS with repeated mode requests if the FC takes
        # some time to actually enter LOITER.
        self.complete_mode_retry_period = float(
            self.declare_parameter(
                'complete_mode_retry_period',
                0.50
            ).value
        )

        # ---------------------------------------------------------------------
        # PARAMETER SANITY CHECKS
        # ---------------------------------------------------------------------
        if (
            self.pass_target_distance
            <= self.pass_clear_distance
        ):
            raise ValueError(
                'pass_target_distance must be greater than '
                'pass_clear_distance'
            )

        if self.pass_edge_margin < 0.0:
            raise ValueError(
                'pass_edge_margin cannot be negative'
            )

        # ---------------------------------------------------------------------
        # BOAT POSE IN MAVROS LOCAL "map" FRAME
        # ---------------------------------------------------------------------
        # /mavros/local_position/pose is published in the fixed MAVROS map
        # frame. Position is in meters and orientation is a quaternion.
        #
        # These values let us transform LiDAR gate coordinates from base_link
        # into a fixed frame so a gate remains geometrically defined after it
        # leaves the LiDAR field of view.
        self.local_x = None
        self.local_y = None
        self.local_yaw = None
        self.local_pose_time = None
        self.tracked_port = None
        self.tracked_starboard = None
        self.tracked_midpoint = None
        self.tracked_gate_measurement_time = None

        # ---------------------------------------------------------------------
        # SAVED GATE GEOMETRY IN MAVROS "map" FRAME
        # ---------------------------------------------------------------------
        self.phase = MissionPhase.WAIT_GATE

        self.saved_port = None
        self.saved_starboard = None
        self.saved_midpoint = None

        # Tangent runs along the gate from port to starboard.
        self.saved_gate_tangent = None

        # Normal points from the approach side through the gate.
        self.saved_gate_normal = None

        self.saved_gate_width = None
        self.saved_usable_half_width = None
        self.complete_mode_request_in_flight = False
        self.last_complete_mode_request_time = None
        self.saved_pass_target = None

        # Passage progress relative to the saved gate plane.
        self.gate_entry_side = None
        self.previous_pass_signed_distance = None
        self.previous_pass_lateral_offset = None

        self.gate_crossed = False
        self.crossing_lateral_offset = None

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

        self.diagnostics_pub = self.create_publisher(
            String,
            '/control/diagnostics',
            10
        )

        self.last_diagnostics_error = None

        self.create_subscription(
            Gate,
            self.gate_topic,
            self.gate_callback,
            10
        )

        # ---------------------------------------------------------------------
        # TASK 1 DEBUG TELEMETRY
        # ---------------------------------------------------------------------
        self.debug_target_point_pub = self.create_publisher(
            PointStamped,
            '/task1/debug/target_point_map',
            10
        )

        self.debug_target_vector_pub = self.create_publisher(
            Vector3Stamped,
            '/task1/debug/target_vector_body',
            10
        )

        self.debug_gate_port_pub = self.create_publisher(
            PointStamped,
            '/task1/debug/gate_port_map',
            10
        )

        self.debug_gate_starboard_pub = self.create_publisher(
            PointStamped,
            '/task1/debug/gate_starboard_map',
            10
        )

        self.debug_gate_range_pub = self.create_publisher(
            Float64,
            '/task1/debug/gate_map_range',
            10
        )

        self.debug_signed_distance_pub = self.create_publisher(
            Float64,
            '/task1/debug/gate_signed_distance',
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

        self.set_mode_client = self.create_client(
            SetMode,
            '/mavros/set_mode'
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
        self.clear_tracked_gate_geometry()

        self.clear_saved_gate_geometry()

        self.complete_mode_request_in_flight = False
        self.last_complete_mode_request_time = None

        self.phase = MissionPhase.WAIT_GATE

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

            # -------------------------------------------------------------
            # DISABLE IS A HARD MISSION-EXECUTION INTERRUPTION
            # -------------------------------------------------------------
            # If manual takeover or software disable occurs during PASS_GATE,
            # do NOT resume an old map target when autonomy is re-enabled.
            #
            # The boat may have been moved manually in the meantime.
            # Re-enable therefore requires fresh perception of the same gate.
            # -------------------------------------------------------------
            if self.phase != MissionPhase.COMPLETE:
                self.clear_saved_gate_geometry()

                self.last_gate = None
                self.last_gate_time = None
                self.clear_tracked_gate_geometry()
                self.last_gate_measurement_stamp = None

                self.phase = MissionPhase.WAIT_GATE

            self.publish_zero(
                'FOLLOWER_DISABLED'
            )

            self.publish_state(
                'DISABLED: controller stopped; '
                'saved passage discarded'
            )

        else:

            if self.phase == MissionPhase.COMPLETE:
                self.publish_state(
                    'MISSION_COMPLETE: controller enabled; '
                    f'holding with {self.complete_mode}'
                )

            else:
                # Always require a fresh gate after re-enabling autonomy.
                self.last_gate = None
                self.last_gate_time = None
                self.last_gate_measurement_stamp = None

                self.phase = MissionPhase.WAIT_GATE

                self.publish_state(
                    f'WAIT_GATE_{self.current_gate}: '
                    f'controller enabled; reacquiring gate'
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
        self.publish_zero(
            'RESET_MISSION'
        )

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

    # =========================================================================
    # MAVROS LOCAL POSE CALLBACK
    # =========================================================================
    def local_position_callback(self, msg):
        """Store the boat's position and heading in the fixed map frame."""

        self.local_x = float(
            msg.pose.position.x
        )
        self.local_y = float(
            msg.pose.position.y
        )

        # ---------------------------------------------------------------------
        # Convert quaternion orientation to planar yaw.
        #
        # ROS quaternion:
        #   q = (x, y, z, w)
        #
        # For the surface vehicle we only need rotation about the vertical
        # axis. The resulting yaw is in radians in the MAVROS map/ENU frame.
        # ---------------------------------------------------------------------
        q = msg.pose.orientation

        siny_cosp = 2.0 * (
            q.w * q.z
            + q.x * q.y
        )

        cosy_cosp = 1.0 - 2.0 * (
            q.y * q.y
            + q.z * q.z
        )

        self.local_yaw = math.atan2(
            siny_cosp,
            cosy_cosp
        )

        # Save receive time so PASS_GATE will never continue navigating using
        # an old/stale position estimate.
        self.local_pose_time = (
            self.get_clock().now()
        )

    # =========================================================================
    # LOCAL POSE VALIDITY
    # =========================================================================
    def local_pose_is_available(self):
        """Return True when a complete local map pose has been received."""

        return (
            self.local_x is not None
            and self.local_y is not None
            and self.local_yaw is not None
            and self.local_pose_time is not None
        )


    def local_pose_age(self):
        """Return age of the latest MAVROS local pose in seconds."""

        if self.local_pose_time is None:
            return None

        return (
            self.get_clock().now()
            - self.local_pose_time
        ).nanoseconds / 1e9


    def local_pose_is_fresh(self, timeout=None):
        """Reject stale odometry before fixed-frame gate navigation."""

        if timeout is None:
            timeout = self.local_pose_timeout

        age = self.local_pose_age()

        return (
            self.local_pose_is_available()
            and age is not None
            and age <= timeout
        )


    # =========================================================================
    # 2D COORDINATE TRANSFORMS
    # =========================================================================
    def body_point_to_map(self, x_body, y_body):
        """Transform a base_link XY point into the fixed MAVROS map frame."""

        if not self.local_pose_is_available():
            return None

        c = math.cos(self.local_yaw)
        s = math.sin(self.local_yaw)

        x_map = (
            self.local_x
            + c * x_body
            - s * y_body
        )

        y_map = (
            self.local_y
            + s * x_body
            + c * y_body
        )

        return x_map, y_map

    def map_point_to_body(self, x_map, y_map):
        """Transform a fixed map-frame XY point back into base_link."""

        if not self.local_pose_is_available():
            return None

        dx = x_map - self.local_x
        dy = y_map - self.local_y

        c = math.cos(self.local_yaw)
        s = math.sin(self.local_yaw)

        # Inverse of the body -> map planar rotation.
        x_body = (
            c * dx
            + s * dy
        )

        y_body = (
            -s * dx
            + c * dy
        )

        return x_body, y_body
    # =========================================================================
    # TRACK_GATE MAP-FRAME GEOMETRY
    # =========================================================================
    def clear_tracked_gate_geometry(self):
        """Discard the currently remembered TRACK_GATE geometry."""

        self.tracked_port = None
        self.tracked_starboard = None
        self.tracked_midpoint = None
        self.tracked_gate_measurement_time = None


    def update_tracked_gate_geometry(self, gate):
        """Transform a new LiDAR gate observation into the MAVROS map frame."""

        # A perception measurement cannot be placed reliably into the map
        # unless the boat pose used for the transform is current.
        if not self.local_pose_is_fresh():
            return False

        port_body_x = float(
            gate.left_marker.x
        )
        port_body_y = float(
            gate.left_marker.y
        )

        starboard_body_x = float(
            gate.right_marker.x
        )
        starboard_body_y = float(
            gate.right_marker.y
        )

        values = (
            port_body_x,
            port_body_y,
            starboard_body_x,
            starboard_body_y,
        )

        if not all(
            math.isfinite(v)
            for v in values
        ):
            return False

        port_map = self.body_point_to_map(
            port_body_x,
            port_body_y
        )

        starboard_map = self.body_point_to_map(
            starboard_body_x,
            starboard_body_y
        )

        if (
            port_map is None
            or starboard_map is None
        ):
            return False

        px, py = port_map
        sx, sy = starboard_map

        gate_width = math.hypot(
            sx - px,
            sy - py
        )

        if gate_width <= 1e-6:
            return False

        self.tracked_port = (
            px,
            py
        )

        self.tracked_starboard = (
            sx,
            sy
        )

        self.tracked_midpoint = (
            0.5 * (px + sx),
            0.5 * (py + sy),
        )

        self.tracked_gate_measurement_time = (
            self.get_clock().now()
        )

        return True


    def tracked_gate_distance(self):
        """Current map-frame distance from the boat to remembered gate."""

        if (
            not self.local_pose_is_available()
            or self.tracked_midpoint is None
        ):
            return None

        midpoint_x, midpoint_y = (
            self.tracked_midpoint
        )

        return math.hypot(
            midpoint_x - self.local_x,
            midpoint_y - self.local_y
        )

    def commit_gate(self):
        """Freeze remembered TRACK_GATE geometry for passage."""

        if self.phase != MissionPhase.TRACK_GATE:
            return False

        # PASS_GATE requires trustworthy current odometry.
        if not self.local_pose_is_fresh():
            return False

        # Never advance the mission during a bench detection or while
        # autonomous propulsion authority is unavailable.
        if not self.vehicle_motion_ready():
            return False

        if (
            self.tracked_port is None
            or self.tracked_starboard is None
            or self.tracked_midpoint is None
        ):
            return False

        px, py = self.tracked_port
        sx, sy = self.tracked_starboard

        midpoint_x, midpoint_y = (
            self.tracked_midpoint
        )

        # -------------------------------------------------------------
        # Gate tangent
        # -------------------------------------------------------------
        gate_dx = sx - px
        gate_dy = sy - py

        gate_width = math.hypot(
            gate_dx,
            gate_dy
        )

        if gate_width <= 1e-6:
            return False

        # -------------------------------------------------------------
        # Physically usable crossing corridor
        # -------------------------------------------------------------
        usable_half_width = (
            0.5 * gate_width
            - self.pass_edge_margin
        )

        if usable_half_width <= 0.0:
            self.get_logger().warn(
                f'Refusing gate commit: '
                f'width={gate_width:.2f} m is too narrow for '
                f'edge margin={self.pass_edge_margin:.2f} m'
            )
            return False

        tangent_x = (
            gate_dx / gate_width
        )

        tangent_y = (
            gate_dy / gate_width
        )

        # -------------------------------------------------------------
        # Gate normal
        # -------------------------------------------------------------
        normal_x = -tangent_y
        normal_y = tangent_x

        # Select the normal pointing from the current approach side,
        # through the remembered gate.
        boat_to_mid_x = (
            midpoint_x - self.local_x
        )

        boat_to_mid_y = (
            midpoint_y - self.local_y
        )

        if (
            normal_x * boat_to_mid_x
            + normal_y * boat_to_mid_y
            < 0.0
        ):
            normal_x *= -1.0
            normal_y *= -1.0

        # -------------------------------------------------------------
        # Fixed target beyond gate
        # -------------------------------------------------------------
        target_x = (
            midpoint_x
            + self.pass_target_distance
            * normal_x
        )

        target_y = (
            midpoint_y
            + self.pass_target_distance
            * normal_y
        )

        # Boat should begin PASS_GATE on the negative side.
        entry_distance = (
            (self.local_x - midpoint_x)
            * normal_x
            + (self.local_y - midpoint_y)
            * normal_y
        )

        entry_lateral = (
            (self.local_x - midpoint_x)
            * tangent_x
            + (self.local_y - midpoint_y)
            * tangent_y
        )

        # -------------------------------------------------------------
        # Freeze geometry for PASS_GATE
        # -------------------------------------------------------------
        self.saved_port = (
            px,
            py
        )

        self.saved_starboard = (
            sx,
            sy
        )

        self.saved_midpoint = (
            midpoint_x,
            midpoint_y
        )

        self.saved_gate_tangent = (
            tangent_x,
            tangent_y
        )

        self.saved_gate_normal = (
            normal_x,
            normal_y
        )

        self.saved_gate_width = (
            gate_width
        )

        self.saved_usable_half_width = (
            usable_half_width
        )

        self.saved_pass_target = (
            target_x,
            target_y
        )

        self.gate_entry_side = (
            entry_distance
        )

        self.previous_pass_signed_distance = (
            entry_distance
        )

        self.previous_pass_lateral_offset = (
            entry_lateral
        )

        self.gate_crossed = False
        self.crossing_lateral_offset = None

        # From this point forward the current gate is frozen.
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

        self.publish_control_diagnostics(
            reason='PASS_GATE_COMMITTED',
            target_map=self.saved_pass_target,
            forward_allowed=False,
            command_forward=0.0,
            command_yaw=0.0,
        )

        return True

    def signed_gate_distance(self):
        """Signed boat distance from saved gate plane."""

        if (
            not self.local_pose_is_available()
            or self.saved_midpoint is None
            or self.saved_gate_normal is None
        ):
            return None

        midpoint_x, midpoint_y = (
            self.saved_midpoint
        )

        normal_x, normal_y = (
            self.saved_gate_normal
        )

        return (
            (self.local_x - midpoint_x) * normal_x
            + (self.local_y - midpoint_y) * normal_y
        )


    def gate_lateral_offset(self):
        """Boat offset along the saved gate line."""

        if (
            not self.local_pose_is_available()
            or self.saved_midpoint is None
            or self.saved_gate_tangent is None
        ):
            return None

        midpoint_x, midpoint_y = (
            self.saved_midpoint
        )

        tangent_x, tangent_y = (
            self.saved_gate_tangent
        )

        return (
            (self.local_x - midpoint_x) * tangent_x
            + (self.local_y - midpoint_y) * tangent_y
        )

    def vehicle_motion_ready(self):
        state = self.vehicle_state

        return (
            state is not None
            and bool(state.connected)
            and bool(state.armed)
            and str(state.mode).upper() == 'GUIDED'
        )

    # =========================================================================
    # MISSION-COMPLETE HOLD MODE
    # =========================================================================
    def request_complete_mode(self):
        """Request LOITER (or configured complete_mode) after Task 1."""

        state = self.vehicle_state

        if (
            state is None
            or not bool(state.connected)
        ):
            return

        # Actual MAVROS state is authoritative.
        if str(state.mode).upper() == self.complete_mode:
            return

        if self.complete_mode_request_in_flight:
            return

        now = self.get_clock().now()

        if self.last_complete_mode_request_time is not None:
            elapsed = (
                now
                - self.last_complete_mode_request_time
            ).nanoseconds / 1e9

            if elapsed < self.complete_mode_retry_period:
                return

        if not self.set_mode_client.service_is_ready():
            return

        request = SetMode.Request()
        request.base_mode = 0
        request.custom_mode = self.complete_mode

        self.last_complete_mode_request_time = now
        self.complete_mode_request_in_flight = True

        future = self.set_mode_client.call_async(
            request
        )

        future.add_done_callback(
            self.complete_mode_response
        )


    def complete_mode_response(self, future):
        """Handle MAVROS mode response without blocking the control timer."""

        self.complete_mode_request_in_flight = False

        try:
            response = future.result()

        except Exception as exc:
            self.get_logger().warning(
                f'{self.complete_mode} mode request failed: '
                f'{exc}'
            )
            return

        if not response.mode_sent:
            self.get_logger().warning(
                f'MAVROS did not accept '
                f'{self.complete_mode} mode request'
            )

    # =========================================================================
    # SAVED PASSAGE STATE MANAGEMENT
    # =========================================================================
    def clear_saved_gate_geometry(self):
        """Discard all geometry belonging to the current committed passage."""

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
        """Safely abandon a committed passage and reacquire the same gate."""

        self.clear_saved_gate_geometry()
        self.clear_tracked_gate_geometry()

        self.last_gate = None
        self.last_gate_time = None
        self.last_gate_measurement_stamp = None

        self.phase = MissionPhase.WAIT_GATE

        self.publish_state(
            f'WAIT_GATE_{self.current_gate}: '
            f'passage aborted; {reason}'
        )

    def gate_callback(self, msg):
        if not self.enabled:
            return

        if self.mission_complete:
            return

        # Once passage begins, current-gate perception is deliberately ignored.
        if self.phase == MissionPhase.PASS_GATE:
            return

        if not self.gate_is_valid(msg):
            return

        measurement_stamp = (
            int(msg.header.stamp.sec),
            int(msg.header.stamp.nanosec),
        )

        new_measurement = (
            measurement_stamp
            != self.last_gate_measurement_stamp
        )

        # Held/sample-and-hold detector output is NOT new geometric evidence.
        if not new_measurement:
            return

        self.last_gate_measurement_stamp = (
            measurement_stamp
        )

        # Save the raw message only for diagnostics/backward compatibility.
        self.last_gate = msg
        self.last_gate_time = (
            self.get_clock().now()
        )

        # Immediately establish/update the gate in the fixed map frame.
        if not self.update_tracked_gate_geometry(
            msg
        ):
            return

        if self.phase == MissionPhase.WAIT_GATE:
            self.phase = MissionPhase.TRACK_GATE

        if self.phase == MissionPhase.TRACK_GATE:
            gate_range = (
                self.tracked_gate_distance()
            )

            if gate_range is not None:
                self.publish_state(
                    f'TRACK_GATE_{self.current_gate}: '
                    f'map_range={gate_range:.2f} m'
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
        self.clear_tracked_gate_geometry()
        self.last_gate_measurement_stamp = None

        self.clear_saved_gate_geometry()

        if self.gates_passed >= self.gates_required:
            self.mission_complete = True
            self.phase = MissionPhase.COMPLETE

            self.publish_state(
                f'MISSION_COMPLETE: passed '
                f'{self.gates_passed}/{self.gates_required} gates; '
                f'{reason}'
            )

            return

        self.current_gate = (
            self.gates_passed + 1
        )
        self.phase = MissionPhase.WAIT_GATE
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

    def publish_debug_telemetry(self):
        """Publish Task 1 internal navigation geometry for logging."""

        now = self.get_clock().now().to_msg()

        target = None
        port = None
        starboard = None

        gate_range = math.nan
        signed_distance = math.nan

        # -------------------------------------------------------------
        # TRACK_GATE
        # -------------------------------------------------------------
        if self.phase == MissionPhase.TRACK_GATE:
            target = self.tracked_midpoint
            port = self.tracked_port
            starboard = self.tracked_starboard

            value = self.tracked_gate_distance()

            if value is not None:
                gate_range = float(value)

        # -------------------------------------------------------------
        # PASS_GATE
        # -------------------------------------------------------------
        elif self.phase == MissionPhase.PASS_GATE:
            target = self.saved_pass_target
            port = self.saved_port
            starboard = self.saved_starboard

            if (
                self.saved_midpoint is not None
                and self.local_x is not None
                and self.local_y is not None
            ):
                midpoint_x, midpoint_y = (
                    self.saved_midpoint
                )

                gate_range = math.hypot(
                    midpoint_x - self.local_x,
                    midpoint_y - self.local_y
                )

            value = self.signed_gate_distance()

            if value is not None:
                signed_distance = float(value)

        # -------------------------------------------------------------
        # Scalar diagnostics
        # -------------------------------------------------------------
        range_msg = Float64()
        range_msg.data = float(gate_range)

        self.debug_gate_range_pub.publish(
            range_msg
        )

        signed_msg = Float64()
        signed_msg.data = float(
            signed_distance
        )

        self.debug_signed_distance_pub.publish(
            signed_msg
        )

        # -------------------------------------------------------------
        # Gate geometry
        # -------------------------------------------------------------
        if port is not None:
            msg = PointStamped()

            msg.header.stamp = now
            msg.header.frame_id = 'map'

            msg.point.x = float(port[0])
            msg.point.y = float(port[1])
            msg.point.z = 0.0

            self.debug_gate_port_pub.publish(
                msg
            )

        if starboard is not None:
            msg = PointStamped()

            msg.header.stamp = now
            msg.header.frame_id = 'map'

            msg.point.x = float(
                starboard[0]
            )

            msg.point.y = float(
                starboard[1]
            )

            msg.point.z = 0.0

            self.debug_gate_starboard_pub.publish(
                msg
            )

        # -------------------------------------------------------------
        # Navigation target
        # -------------------------------------------------------------
        if target is not None:
            target_msg = PointStamped()

            target_msg.header.stamp = now
            target_msg.header.frame_id = 'map'

            target_msg.point.x = float(
                target[0]
            )

            target_msg.point.y = float(
                target[1]
            )

            target_msg.point.z = 0.0

            self.debug_target_point_pub.publish(
                target_msg
            )

            # This is the exact target direction as seen from
            # the boat's current base_link frame.
            if self.local_pose_is_fresh():

                target_body = (
                    self.map_point_to_body(
                        target[0],
                        target[1]
                    )
                )

                if target_body is not None:
                    vector_msg = Vector3Stamped()

                    vector_msg.header.stamp = now
                    vector_msg.header.frame_id = (
                        'base_link'
                    )

                    vector_msg.vector.x = float(
                        target_body[0]
                    )

                    vector_msg.vector.y = float(
                        target_body[1]
                    )

                    vector_msg.vector.z = 0.0

                    self.debug_target_vector_pub.publish(
                        vector_msg
                    )

    def publish_control_diagnostics(
        self,
        reason,
        target_map=None,
        target_body=None,
        heading_error=None,
        forward_allowed=False,
        command_forward=0.0,
        command_yaw=0.0,
    ):
        """Publish diagnostics without ever interrupting control."""

        try:
            self._publish_control_diagnostics(
                reason=reason,
                target_map=target_map,
                target_body=target_body,
                heading_error=heading_error,
                forward_allowed=forward_allowed,
                command_forward=command_forward,
                command_yaw=command_yaw,
            )
            self.last_diagnostics_error = None

        except Exception as exc:
            error_text = str(exc)

            if error_text != self.last_diagnostics_error:
                self.get_logger().warning(
                    f'Control diagnostics publish failed: '
                    f'{error_text}'
                )
                self.last_diagnostics_error = error_text

    def _publish_control_diagnostics(
        self,
        reason,
        target_map=None,
        target_body=None,
        heading_error=None,
        forward_allowed=False,
        command_forward=0.0,
        command_yaw=0.0,
    ):
        """Publish controller-internal geometry and decision state as JSON."""

        def xy_values(point):
            if point is None:
                return None, None

            return (
                float(point[0]),
                float(point[1]),
            )

        tracked_port_x, tracked_port_y = (
            xy_values(self.tracked_port)
        )
        tracked_starboard_x, tracked_starboard_y = (
            xy_values(self.tracked_starboard)
        )
        tracked_midpoint_x, tracked_midpoint_y = (
            xy_values(self.tracked_midpoint)
        )

        saved_port_x, saved_port_y = (
            xy_values(self.saved_port)
        )
        saved_starboard_x, saved_starboard_y = (
            xy_values(self.saved_starboard)
        )
        saved_midpoint_x, saved_midpoint_y = (
            xy_values(self.saved_midpoint)
        )

        tangent_x, tangent_y = (
            xy_values(self.saved_gate_tangent)
        )
        normal_x, normal_y = (
            xy_values(self.saved_gate_normal)
        )
        pass_target_x, pass_target_y = (
            xy_values(self.saved_pass_target)
        )

        if target_map is None:
            if self.phase == MissionPhase.TRACK_GATE:
                target_map = self.tracked_midpoint

            elif self.phase == MissionPhase.PASS_GATE:
                target_map = self.saved_pass_target

        if (
            target_body is None
            and target_map is not None
            and self.local_pose_is_available()
        ):
            target_body = self.map_point_to_body(
                target_map[0],
                target_map[1],
            )

        target_map_x, target_map_y = (
            xy_values(target_map)
        )
        target_body_x, target_body_y = (
            xy_values(target_body)
        )

        target_distance = None

        if target_body is not None:
            target_distance = math.hypot(
                target_body[0],
                target_body[1],
            )

        gate_map_range = None

        if self.phase == MissionPhase.TRACK_GATE:
            gate_map_range = (
                self.tracked_gate_distance()
            )

        elif (
            self.phase == MissionPhase.PASS_GATE
            and self.saved_midpoint is not None
            and self.local_pose_is_available()
        ):
            gate_map_range = math.hypot(
                self.saved_midpoint[0] - self.local_x,
                self.saved_midpoint[1] - self.local_y,
            )

        signed_distance = None
        lateral_offset = None

        if self.phase == MissionPhase.PASS_GATE:
            signed_distance = (
                self.signed_gate_distance()
            )
            lateral_offset = (
                self.gate_lateral_offset()
            )

        distance_beyond_gate = None

        if signed_distance is not None:
            distance_beyond_gate = max(
                0.0,
                float(signed_distance),
            )

        local_pose_age = self.local_pose_age()

        tracked_gate_measurement_age = None

        if self.tracked_gate_measurement_time is not None:
            tracked_gate_measurement_age = (
                self.get_clock().now()
                - self.tracked_gate_measurement_time
            ).nanoseconds / 1e9

        data = {
            'controller_reason': str(reason),
            'mission_phase': self.phase.name,
            'current_gate': int(self.current_gate),
            'gates_passed': int(self.gates_passed),
            'mission_complete': bool(self.mission_complete),
            'follower_enabled': bool(self.enabled),

            'local_pose_age_s': (
                None
                if local_pose_age is None
                else float(local_pose_age)
            ),
            'local_pose_fresh': bool(
                self.local_pose_is_fresh()
            ),
            'tracked_gate_measurement_age_s': (
                None
                if tracked_gate_measurement_age is None
                else float(tracked_gate_measurement_age)
            ),

            'gate_map_range_m': (
                None
                if gate_map_range is None
                else float(gate_map_range)
            ),
            'gate_signed_distance_m': (
                None
                if signed_distance is None
                else float(signed_distance)
            ),
            'gate_lateral_offset_m': (
                None
                if lateral_offset is None
                else float(lateral_offset)
            ),
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
            'saved_gate_width_m': (
                None
                if self.saved_gate_width is None
                else float(self.saved_gate_width)
            ),
            'usable_half_width': (
                None
                if self.saved_usable_half_width is None
                else float(self.saved_usable_half_width)
            ),
            'pass_target_x': pass_target_x,
            'pass_target_y': pass_target_y,

            'gate_entry_side_m': (
                None
                if self.gate_entry_side is None
                else float(self.gate_entry_side)
            ),
            'previous_pass_signed_distance_m': (
                None
                if self.previous_pass_signed_distance is None
                else float(self.previous_pass_signed_distance)
            ),
            'previous_pass_lateral_offset_m': (
                None
                if self.previous_pass_lateral_offset is None
                else float(self.previous_pass_lateral_offset)
            ),
            'gate_crossed': bool(self.gate_crossed),
            'crossing_lateral_offset_m': (
                None
                if self.crossing_lateral_offset is None
                else float(self.crossing_lateral_offset)
            ),

            'target_map_x': target_map_x,
            'target_map_y': target_map_y,
            'target_body_x': target_body_x,
            'target_body_y': target_body_y,
            'target_distance': (
                None
                if target_distance is None
                else float(target_distance)
            ),
            'heading_error_deg': (
                None
                if heading_error is None
                else math.degrees(
                    float(heading_error)
                )
            ),
            'forward_angle_limit_deg': float(
                self.forward_angle_limit_deg
            ),
            'forward_allowed': bool(
                forward_allowed
            ),
            'follower_linear_x': float(
                command_forward
            ),
            'follower_angular_z': float(
                command_yaw
            ),
        }

        msg = String()
        msg.data = json.dumps(
            data,
            separators=(',', ':'),
            allow_nan=False,
        )

        self.diagnostics_pub.publish(msg)

    def publish_zero(
        self,
        reason='ZERO_REQUESTED',
    ):
        msg = TwistStamped()

        msg.header.stamp = (
            self.get_clock().now().to_msg()
        )

        msg.header.frame_id = 'base_link'

        msg.twist.linear.x = 0.0
        msg.twist.angular.z = 0.0

        self.cmd_pub.publish(msg)

        self.publish_control_diagnostics(
            reason=reason,
            forward_allowed=False,
            command_forward=0.0,
            command_yaw=0.0,
        )

    def publish_gate_command(self):
        """Steer toward the remembered map-frame gate midpoint."""

        if self.tracked_midpoint is None:
            self.publish_zero(
                'TRACK_TARGET_MISSING'
            )
            return

        if not self.local_pose_is_fresh():
            self.publish_zero(
                'TRACK_LOCAL_POSE_STALE'
            )
            return

        midpoint_x, midpoint_y = (
            self.tracked_midpoint
        )

        target_body = self.map_point_to_body(
            midpoint_x,
            midpoint_y
        )

        if target_body is None:
            self.publish_zero(
                'TRACK_TARGET_TRANSFORM_FAILED'
            )
            return

        x, y = target_body

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

        forward_allowed = bool(
            forward != 0.0
        )

        msg = TwistStamped()

        msg.header.stamp = (
            self.get_clock().now().to_msg()
        )

        msg.header.frame_id = (
            'base_link'
        )

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

        self.publish_control_diagnostics(
            reason=(
                'TRACKING_GATE'
                if forward_allowed
                else 'TRACK_TARGET_OUTSIDE_FORWARD_ANGLE'
            ),
            target_map=self.tracked_midpoint,
            target_body=target_body,
            heading_error=heading_error,
            forward_allowed=forward_allowed,
            command_forward=forward,
            command_yaw=yaw,
        )

    def publish_pass_command(self):
        """Steer toward the fixed map-frame target beyond the gate."""

        if self.saved_pass_target is None:
            self.publish_zero(
                'PASS_TARGET_MISSING'
            )
            return

        target_x, target_y = (
            self.saved_pass_target
        )

        target_body = self.map_point_to_body(
            target_x,
            target_y
        )

        if target_body is None:
            self.publish_zero(
                'PASS_TARGET_TRANSFORM_FAILED'
            )
            return

        x, y = target_body

        heading_error = math.atan2(
            y,
            x
        )

        yaw = self.yaw_kp * heading_error

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
            if abs(heading_error) <= forward_limit
            else 0.0
        )

        forward_allowed = bool(
            forward != 0.0
        )

        msg = TwistStamped()

        msg.header.stamp = (
            self.get_clock().now().to_msg()
        )

        msg.header.frame_id = 'base_link'

        msg.twist.linear.x = float(
            forward
        )

        msg.twist.angular.z = float(
            yaw
        )

        self.cmd_pub.publish(msg)

        self.publish_control_diagnostics(
            reason=(
                'PASSAGE_TARGET_AHEAD'
                if forward_allowed
                else 'PASS_TARGET_OUTSIDE_FORWARD_ANGLE'
            ),
            target_map=self.saved_pass_target,
            target_body=target_body,
            heading_error=heading_error,
            forward_allowed=forward_allowed,
            command_forward=forward,
            command_yaw=yaw,
        )

    def update(self):
        self.publish_debug_telemetry()

        # ---------------------------------------------------------------------
        # GLOBAL SAFETY
        # ---------------------------------------------------------------------
        if not self.enabled:
            self.publish_zero(
                'FOLLOWER_DISABLED'
            )
            return

        # ---------------------------------------------------------------------
        # MISSION COMPLETE
        # ---------------------------------------------------------------------
        if self.phase == MissionPhase.COMPLETE:
            # Stop mission velocity commands before handing position holding
            # to the autopilot.
            self.publish_zero(
                'MISSION_COMPLETE'
            )

            self.request_complete_mode()

            if (
                self.vehicle_state is not None
                and str(
                    self.vehicle_state.mode
                ).upper() == self.complete_mode
            ):
                self.publish_state(
                    f'MISSION_COMPLETE: '
                    f'{self.complete_mode} active; '
                    f'holding position'
                )

            return

        # ---------------------------------------------------------------------
        # PASS_GATE
        # ---------------------------------------------------------------------
        if self.phase == MissionPhase.PASS_GATE:

            # Never continue a committed passage unless MAVROS remains
            # connected, ARMED, and GUIDED.
            if not self.vehicle_motion_ready():
                self.publish_state(
                    f'PASS_GATE_{self.current_gate}_BLOCKED: '
                    f'vehicle not ARMED + GUIDED'
                )

                self.publish_zero(
                    'PASS_VEHICLE_NOT_READY'
                )
                return

            # Fixed-frame navigation must never use stale odometry.
            if not self.local_pose_is_fresh():
                age = self.local_pose_age()

                age_text = (
                    'none'
                    if age is None
                    else f'{age:.2f}s'
                )

                self.publish_state(
                    f'PASS_GATE_{self.current_gate}_BLOCKED: '
                    f'local pose stale ({age_text})'
                )

                self.publish_zero(
                    'PASS_LOCAL_POSE_STALE'
                )
                return

            signed_distance = (
                self.signed_gate_distance()
            )

            lateral_offset = (
                self.gate_lateral_offset()
            )

            if (
                signed_distance is None
                or lateral_offset is None
            ):
                self.publish_zero(
                    'PASS_GEOMETRY_UNAVAILABLE'
                )
                return

            # -------------------------------------------------------------
            # Detect actual crossing of the saved gate line.
            # -------------------------------------------------------------
            if not self.gate_crossed:

                previous_signed = (
                    self.previous_pass_signed_distance
                )

                previous_lateral = (
                    self.previous_pass_lateral_offset
                )

                if (
                    previous_signed is not None
                    and previous_signed < 0.0
                    and signed_distance >= 0.0
                ):

                    denominator = (
                        signed_distance
                        - previous_signed
                    )

                    fraction = (
                        -previous_signed / denominator
                        if abs(denominator) > 1e-9
                        else 1.0
                    )

                    if previous_lateral is None:
                        crossing_lateral = (
                            lateral_offset
                        )
                    else:
                        crossing_lateral = (
                            previous_lateral
                            + fraction
                            * (
                                lateral_offset
                                - previous_lateral
                            )
                        )

                    usable_half_width = (
                        self.saved_usable_half_width
                    )

                    # Crossing the infinite gate plane beside the buoys
                    # does not count as passing through the gate.
                    if (
                        usable_half_width is None
                        or abs(crossing_lateral)
                        > usable_half_width
                    ):
                        self.abort_to_wait_gate(
                            f'crossed outside safe gate corridor; '
                            f'lateral={crossing_lateral:.2f} m'
                        )

                        self.publish_zero(
                            'PASS_CROSSED_OUTSIDE_CORRIDOR'
                        )
                        return

                    self.gate_crossed = True

                    self.crossing_lateral_offset = (
                        crossing_lateral
                    )

                    self.publish_state(
                        f'CROSSED_GATE_{self.current_gate}: '
                        f'lateral_offset='
                        f'{crossing_lateral:.2f} m'
                    )

                self.previous_pass_signed_distance = (
                    signed_distance
                )

                self.previous_pass_lateral_offset = (
                    lateral_offset
                )

            # -------------------------------------------------------------
            # Require clearance beyond the saved gate line.
            # -------------------------------------------------------------
            if (
                self.gate_crossed
                and signed_distance
                >= self.pass_clear_distance
            ):
                self.finish_current_gate(
                    f'crossed saved gate line and cleared '
                    f'{signed_distance:.2f} m'
                )

                self.publish_zero(
                    'PASS_GATE_CLEARED'
                )
                return

            self.publish_pass_command()
            return

        # ---------------------------------------------------------------------
        # WAIT_GATE
        # ---------------------------------------------------------------------
        if self.phase == MissionPhase.WAIT_GATE:
            self.publish_zero(
                'WAIT_GATE'
            )
            return

        # ---------------------------------------------------------------------
        # TRACK_GATE
        # ---------------------------------------------------------------------
        if self.phase == MissionPhase.TRACK_GATE:

            # Once a gate has been established in map coordinates, live
            # perception is no longer required every control cycle.
            if self.tracked_midpoint is None:
                self.phase = MissionPhase.WAIT_GATE

                self.publish_state(
                    f'WAIT_GATE_{self.current_gate}: '
                    f'no remembered gate geometry'
                )

                self.publish_zero(
                    'TRACK_GEOMETRY_MISSING'
                )
                return

            # Map-based tracking and commit both require current odometry.
            if not self.local_pose_is_fresh():
                age = self.local_pose_age()

                age_text = (
                    'none'
                    if age is None
                    else f'{age:.2f}s'
                )

                self.publish_state(
                    f'TRACK_GATE_{self.current_gate}_BLOCKED: '
                    f'local pose stale ({age_text})'
                )

                self.publish_zero(
                    'TRACK_LOCAL_POSE_STALE'
                )
                return

            gate_range = (
                self.tracked_gate_distance()
            )

            if gate_range is None:
                self.publish_zero(
                    'TRACK_RANGE_UNAVAILABLE'
                )
                return

            self.publish_state(
                f'TRACK_GATE_{self.current_gate}: '
                f'map_range={gate_range:.2f} m'
            )

            # This is now a LOCAL-POSE decision, not a perception-callback
            # decision. The detector does not need to produce another frame
            # exactly as the boat crosses the commit radius.
            if (
                gate_range
                <= self.pass_commit_distance
            ):
                if self.commit_gate():
                    return

            self.publish_gate_command()
            return

        # Defensive fallback for any unexpected state.
        self.publish_zero(
            'UNEXPECTED_STATE'
        )


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
