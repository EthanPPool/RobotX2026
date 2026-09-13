#!/usr/bin/env python3

"""Fault-tolerant 20 Hz diagnostic recorder for RobotX USV missions."""

import json
import math
import re
import threading
import time
from collections import deque

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State, SysStatus
from rclpy.node import Node
from rclpy.parameter import parameter_value_to_python
from rclpy.parameter_client import AsyncParameterClient
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import BatteryState, Imu, MagneticField, NavSatFix
from std_msgs.msg import Bool, String

from boat_dashboard_bridge.logger_core import MissionLogCore
from boat_interfaces.msg import Gate
from boat_interfaces.srv import ResetMissionLog

try:
    from mavros_msgs.msg import EstimatorStatus
except ImportError:  # pragma: no cover - depends on installed MAVROS plugins
    EstimatorStatus = None

try:
    from mavros_msgs.msg import GPSRAW
except ImportError:  # pragma: no cover - depends on installed MAVROS plugins
    GPSRAW = None

try:
    from mavros_msgs.msg import RCOut
except ImportError:  # pragma: no cover - depends on installed MAVROS plugins
    RCOut = None

try:
    from mavros_msgs.msg import TimesyncStatus
except ImportError:  # pragma: no cover - depends on installed MAVROS plugins
    TimesyncStatus = None

try:
    from mavros_msgs.srv import ParamGet
except ImportError:  # pragma: no cover - depends on installed MAVROS version
    ParamGet = None


SAMPLE_FIELDS = {
    "mavros_state": [
        "connected", "armed", "guided", "mode", "system_status",
    ],
    "pose": [
        "pose_x_m", "pose_y_m", "pose_z_m",
        "pose_frame_id", "rolling_pose_hz",
        "roll_deg", "pitch_deg", "yaw_deg",
        "quaternion_x", "quaternion_y", "quaternion_z", "quaternion_w",
    ],
    "velocity": [
        "velocity_map_x", "velocity_map_y", "velocity_map_z",
        "velocity_body_forward", "velocity_body_lateral",
    ],
    "imu": [
        "gyro_x", "gyro_y", "gyro_z",
        "accel_x", "accel_y", "accel_z",
    ],
    "mag": ["mag_x", "mag_y", "mag_z"],
    "gps": [
        "gps_status", "gps_latitude", "gps_longitude", "gps_altitude",
    ],
    "gps_raw": [
        "gps_fix_type", "gps_satellites", "gps_eph", "gps_epv",
    ],
    "ekf": [
        "ekf_attitude_healthy", "ekf_horiz_velocity_healthy",
        "ekf_relative_position_healthy", "ekf_absolute_position_healthy",
        "ekf_gps_glitch", "ekf_accel_error",
    ],
    "gate": [
        "gate_confidence", "gate_left_x", "gate_left_y",
        "gate_right_x", "gate_right_y", "gate_midpoint_x",
        "gate_midpoint_y", "gate_width", "gate_new_measurement",
        "gate_held_measurement", "gate_frame_id",
    ],
    "mission": [
        "mission_phase", "mission_state_text", "current_gate",
        "gates_passed", "mission_complete", "follower_enabled",
    ],
    "follower_diag": [
        "controller_reason", "passage_armed", "lidar_approach_confirmed",
        "close_gate_hits", "gate_map_range_m", "gate_signed_distance_m",
        "gate_lateral_offset_m", "distance_beyond_gate_m",
        "tracked_port_x", "tracked_port_y", "tracked_starboard_x",
        "tracked_starboard_y", "tracked_midpoint_x", "tracked_midpoint_y",
        "saved_port_x", "saved_port_y", "saved_starboard_x",
        "saved_starboard_y", "saved_midpoint_x", "saved_midpoint_y",
        "pass_tangent_x", "pass_tangent_y", "pass_normal_x",
        "pass_normal_y", "pass_target_x", "pass_target_y",
        "usable_half_width", "target_map_x", "target_map_y",
        "target_body_x", "target_body_y", "target_distance",
        "heading_error_deg", "forward_angle_limit_deg", "forward_allowed",
        "intergate_hold_active", "intergate_resume_requested",
        "travel_since_pass_arm", "gate_approach_since_pass_arm",
    ],
    "follower_cmd": ["follower_linear_x", "follower_angular_z"],
    "bridge_diag": [
        "bridge_reason", "bridge_autonomy_enabled", "software_estop",
        "bridge_mode_allowed", "bridge_command_fresh",
        "bridge_output_authorized", "bridge_input_age_s",
    ],
    "bridge_input": ["bridge_input_linear_x", "bridge_input_angular_z"],
    "bridge_output": ["bridge_output_linear_x", "bridge_output_angular_z"],
    "rc_out": [f"rc_out_{index}" for index in range(1, 19)],
    "operator": ["operator_linear_x", "operator_angular_z"],
    "operator_deadman": ["operator_deadman"],
    "battery": ["battery_voltage", "battery_current", "battery_percent"],
    "sys_status": ["sys_voltage", "sys_current", "sys_remaining"],
    "timesync": [
        "timesync_rtt_ms", "timesync_observed_offset_ns",
        "timesync_estimated_offset_ns",
    ],
}

FIELDS = [
    "mission_id", "ros_time", "elapsed_s", "row_index",
    "event_type", "event_text", "end_reason",
]
for sample_name, names in SAMPLE_FIELDS.items():
    FIELDS.extend([f"{sample_name}_stamp", f"{sample_name}_age_s", f"{sample_name}_new"])
    FIELDS.extend(names)


FOLLOWER_PARAMETERS = [
    "forward_speed", "yaw_kp", "max_yaw_rate", "forward_angle_limit_deg",
    "passage_arm_distance", "pass_close_distance",
    "pass_min_gate_approach", "pass_close_confirm_hits",
    "pass_loss_timeout", "pass_jump_distance", "pass_min_travel",
    "gate_timeout", "min_gate_confidence",
]
BRIDGE_PARAMETERS = [
    "max_forward_speed", "max_yaw_rate", "deadman_timeout",
    "publish_rate", "allowed_modes",
]
AUTOPILOT_PARAMETERS = [
    "AHRS_ORIENTATION",
    "COMPASS_OFS_X", "COMPASS_OFS_Y", "COMPASS_OFS_Z",
    "COMPASS_DIA_X", "COMPASS_DIA_Y", "COMPASS_DIA_Z",
    "COMPASS_ODI_X", "COMPASS_ODI_Y", "COMPASS_ODI_Z",
    "COMPASS_SCALE",
    "INS_ACCOFFS_X", "INS_ACCOFFS_Y", "INS_ACCOFFS_Z",
    "INS_ACCSCAL_X", "INS_ACCSCAL_Y", "INS_ACCSCAL_Z",
    "EK3_ENABLE", "EK3_SRC1_POSXY", "EK3_SRC1_VELXY",
    "EK3_SRC1_POSZ", "EK3_SRC1_VELZ", "EK3_SRC1_YAW",
]


def message_stamp(msg, fallback):
    """Format a message header stamp, falling back to receipt ROS time."""
    header = getattr(msg, "header", None)
    stamp = getattr(header, "stamp", None)
    if stamp is None:
        return fallback
    seconds = int(getattr(stamp, "sec", 0))
    nanoseconds = int(getattr(stamp, "nanosec", 0))
    if seconds == 0 and nanoseconds == 0:
        return fallback
    return f"{seconds}.{nanoseconds:09d}"


def quaternion_euler(q):
    """Return roll, pitch, yaw in radians from a quaternion."""
    sinr = 2.0 * (q.w * q.x + q.y * q.z)
    cosr = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr, cosr)
    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return roll, pitch, math.atan2(siny, cosy)


class MissionLogger(Node):
    """Cache ROS sources and write synchronized snapshots at 20 Hz."""

    def __init__(self):
        super().__init__("mission_logger")
        log_directory = self.declare_parameter(
            "log_directory", "~/robotx_logs"
        ).value
        self.record_rate_hz = float(self.declare_parameter("record_rate_hz", 20.0).value)
        prearm_seconds = float(self.declare_parameter("prearm_seconds", 5.0).value)
        disarm_seconds = float(self.declare_parameter("disarm_grace_seconds", 5.0).value)

        self.lock = threading.RLock()
        self.samples = {}
        self.written_sequences = {}
        self.sequence = 0
        self.last_gate_source_stamp = None
        self.last_mission_state = None
        self.last_state = None
        self.last_bridge_reason = None
        self.yaw_rad = None
        self.pose_receipts = deque(maxlen=50)

        self.core = MissionLogCore(
            log_directory,
            FIELDS,
            prearm_seconds=prearm_seconds,
            disarm_grace_seconds=disarm_seconds,
        )

        self.status_pub = self.create_publisher(String, "/mission_logger/status", 10)
        self.create_service(
            ResetMissionLog,
            "/mission_logger/reset",
            self.reset_callback,
        )

        self._subscribe(State, "/mavros/state", self.state_callback)
        self._subscribe(PoseStamped, "/mavros/local_position/pose", self.pose_callback, True)
        self._subscribe(
            TwistStamped,
            "/mavros/local_position/velocity_local",
            self.velocity_callback,
            True,
        )
        self._subscribe(Imu, "/mavros/imu/data", self.imu_callback, True)
        self._subscribe(MagneticField, "/mavros/imu/mag", self.mag_callback, True)
        self._subscribe(NavSatFix, "/mavros/global_position/global", self.gps_callback, True)
        self._subscribe(Gate, "/perception/gate", self.gate_callback)
        self._subscribe(String, "/mission/state", self.mission_callback)
        self._subscribe(String, "/control/diagnostics", self.follower_diag_callback)
        self._subscribe(TwistStamped, "/control/cmd_vel", self.follower_cmd_callback)
        self._subscribe(String, "/vehicle/control_diagnostics", self.bridge_diag_callback)
        self._subscribe(
            TwistStamped,
            "/mavros/setpoint_velocity/cmd_vel",
            self.bridge_output_callback,
        )
        self._subscribe(TwistStamped, "/operator/cmd_vel", self.operator_callback)
        self._subscribe(Bool, "/operator/deadman", self.operator_deadman_callback)
        self._subscribe(BatteryState, "/mavros/battery", self.battery_callback, True)
        self._subscribe(SysStatus, "/mavros/sys_status", self.sys_status_callback)

        self._optional_subscription(GPSRAW, "/mavros/gpsstatus/gps1/raw", self.gps_raw_callback)
        self._optional_subscription(EstimatorStatus, "/mavros/estimator_status", self.ekf_callback)
        self._optional_subscription(RCOut, "/mavros/rc/out", self.rc_out_callback)
        self._optional_subscription(
            TimesyncStatus,
            "/mavros/timesync_status",
            self.timesync_callback,
        )

        self.follower_parameter_client = AsyncParameterClient(self, "two_gate_follower")
        self.bridge_parameter_client = AsyncParameterClient(self, "mavros_command_bridge")
        self.autopilot_param_client = (
            self.create_client(ParamGet, "/mavros/param/get")
            if ParamGet is not None else None
        )

        self.create_timer(1.0 / max(self.record_rate_hz, 1.0), self.record_snapshot)
        self.create_timer(0.2, self.publish_status)
        self.get_logger().info(
            f"Mission logger ready: pending sessions buffer {prearm_seconds:.1f} s; "
            f"logs write to {self.core.log_directory}"
        )

    def _subscribe(self, message_type, topic, callback, sensor_qos=False):
        qos = qos_profile_sensor_data if sensor_qos else 10
        self.create_subscription(message_type, topic, callback, qos)

    def _optional_subscription(self, message_type, topic, callback):
        if message_type is not None:
            self.create_subscription(message_type, topic, callback, qos_profile_sensor_data)

    def _ros_time(self):
        now = self.get_clock().now().nanoseconds
        return f"{now // 1000000000}.{now % 1000000000:09d}"

    def cache(self, name, values, msg=None):
        now_mono = time.monotonic()
        now_ros = self._ros_time()
        clean_values = {
            key: (
                ""
                if isinstance(value, float)
                and not math.isfinite(value)
                else value
            )
            for key, value in values.items()
        }
        with self.lock:
            self.sequence += 1
            self.samples[name] = {
                "values": clean_values,
                "stamp": message_stamp(msg, now_ros) if msg is not None else now_ros,
                "receipt": now_mono,
                "sequence": self.sequence,
            }

    def state_callback(self, msg):
        values = {
            "connected": bool(msg.connected),
            "armed": bool(msg.armed),
            "guided": bool(msg.guided),
            "mode": str(msg.mode),
            "system_status": int(msg.system_status),
        }
        previous = self.last_state
        self.last_state = values
        if previous is not None:
            if previous["connected"] != values["connected"]:
                self.core.queue_event(
                    "MAVROS_CONNECTED" if values["connected"] else "MAVROS_DISCONNECTED",
                    f"connected={values['connected']}",
                )
            if previous["mode"] != values["mode"]:
                self.core.queue_event("MODE_CHANGE", f"{previous['mode']} -> {values['mode']}")
        self.cache("mavros_state", values, msg)
        try:
            self.core.observe_armed(bool(msg.armed), time.monotonic(), self._ros_time())
        except Exception as exc:
            self.get_logger().error(f"Could not start mission log: {exc}")
        self.publish_status()

    def pose_callback(self, msg):
        q = msg.pose.orientation
        roll, pitch, yaw = quaternion_euler(q)
        self.yaw_rad = yaw
        now = time.monotonic()
        self.pose_receipts.append(now)
        rolling_hz = ""
        if len(self.pose_receipts) >= 2:
            duration = self.pose_receipts[-1] - self.pose_receipts[0]
            if duration > 0.0:
                rolling_hz = (len(self.pose_receipts) - 1) / duration
        self.cache("pose", {
            "pose_x_m": float(msg.pose.position.x),
            "pose_y_m": float(msg.pose.position.y),
            "pose_z_m": float(msg.pose.position.z),
            "pose_frame_id": str(msg.header.frame_id),
            "rolling_pose_hz": rolling_hz,
            "roll_deg": math.degrees(roll),
            "pitch_deg": math.degrees(pitch),
            "yaw_deg": math.degrees(yaw),
            "quaternion_x": float(q.x), "quaternion_y": float(q.y),
            "quaternion_z": float(q.z), "quaternion_w": float(q.w),
        }, msg)

    def velocity_callback(self, msg):
        vx = float(msg.twist.linear.x)
        vy = float(msg.twist.linear.y)
        yaw = self.yaw_rad
        forward = lateral = ""
        if yaw is not None:
            forward = math.cos(yaw) * vx + math.sin(yaw) * vy
            lateral = -math.sin(yaw) * vx + math.cos(yaw) * vy
        self.cache("velocity", {
            "velocity_map_x": vx, "velocity_map_y": vy,
            "velocity_map_z": float(msg.twist.linear.z),
            "velocity_body_forward": forward,
            "velocity_body_lateral": lateral,
        }, msg)

    def imu_callback(self, msg):
        self.cache("imu", {
            "gyro_x": float(msg.angular_velocity.x),
            "gyro_y": float(msg.angular_velocity.y),
            "gyro_z": float(msg.angular_velocity.z),
            "accel_x": float(msg.linear_acceleration.x),
            "accel_y": float(msg.linear_acceleration.y),
            "accel_z": float(msg.linear_acceleration.z),
        }, msg)

    def mag_callback(self, msg):
        self.cache("mag", {
            "mag_x": float(msg.magnetic_field.x),
            "mag_y": float(msg.magnetic_field.y),
            "mag_z": float(msg.magnetic_field.z),
        }, msg)

    def gps_callback(self, msg):
        self.cache("gps", {
            "gps_status": int(msg.status.status),
            "gps_latitude": float(msg.latitude),
            "gps_longitude": float(msg.longitude),
            "gps_altitude": float(msg.altitude),
        }, msg)

    def gps_raw_callback(self, msg):
        self.cache("gps_raw", {
            "gps_fix_type": getattr(msg, "fix_type", ""),
            "gps_satellites": getattr(msg, "satellites_visible", ""),
            "gps_eph": getattr(msg, "eph", ""),
            "gps_epv": getattr(msg, "epv", ""),
        }, msg)

    def ekf_callback(self, msg):
        self.cache("ekf", {
            "ekf_attitude_healthy": getattr(msg, "attitude_status_flag", ""),
            "ekf_horiz_velocity_healthy": getattr(msg, "velocity_horiz_status_flag", ""),
            "ekf_relative_position_healthy": getattr(msg, "pos_horiz_rel_status_flag", ""),
            "ekf_absolute_position_healthy": getattr(msg, "pos_horiz_abs_status_flag", ""),
            "ekf_gps_glitch": getattr(msg, "gps_glitch_status_flag", ""),
            "ekf_accel_error": getattr(msg, "accel_error_status_flag", ""),
        }, msg)

    def gate_callback(self, msg):
        stamp = message_stamp(msg, self._ros_time())
        new_measurement = stamp != self.last_gate_source_stamp
        self.last_gate_source_stamp = stamp
        self.cache("gate", {
            "gate_confidence": float(msg.confidence),
            "gate_left_x": float(msg.left_marker.x),
            "gate_left_y": float(msg.left_marker.y),
            "gate_right_x": float(msg.right_marker.x),
            "gate_right_y": float(msg.right_marker.y),
            "gate_midpoint_x": 0.5 * (float(msg.left_marker.x) + float(msg.right_marker.x)),
            "gate_midpoint_y": 0.5 * (float(msg.left_marker.y) + float(msg.right_marker.y)),
            "gate_width": float(msg.width),
            "gate_new_measurement": new_measurement,
            "gate_held_measurement": not new_measurement,
            "gate_frame_id": str(msg.header.frame_id),
        }, msg)

    def mission_callback(self, msg):
        state = str(msg.data).strip()
        phase = state.split(":", 1)[0] if state else ""
        gate_match = re.search(r"GATE_(\d+)", phase)
        if state != self.last_mission_state:
            self.core.queue_event(phase or "MISSION_STATE", state)
            self.last_mission_state = state
        self.cache("mission", {
            "mission_phase": phase,
            "mission_state_text": state,
            "current_gate": int(gate_match.group(1)) if gate_match else "",
            "mission_complete": phase == "MISSION_COMPLETE",
        }, msg)

    def json_callback(self, sample_name, msg):
        try:
            data = json.loads(msg.data)
        except (TypeError, json.JSONDecodeError):
            return
        if isinstance(data, dict):
            self.cache(sample_name, data, msg)

    def follower_diag_callback(self, msg):
        self.json_callback("follower_diag", msg)

    def bridge_diag_callback(self, msg):
        try:
            data = json.loads(msg.data)
        except (TypeError, json.JSONDecodeError):
            return
        reason = data.get("bridge_reason")
        if reason != self.last_bridge_reason:
            if reason == "SOFTWARE_ESTOP":
                self.core.queue_event(
                    "SOFTWARE_ESTOP_ON",
                    "bridge output inhibited",
                )
            elif self.last_bridge_reason == "SOFTWARE_ESTOP":
                self.core.queue_event(
                    "SOFTWARE_ESTOP_OFF",
                    f"new bridge reason={reason}",
                )
            elif reason == "AUTHORIZED":
                self.core.queue_event(
                    "BRIDGE_AUTHORIZED",
                    "propulsion output authorized",
                )
            self.last_bridge_reason = reason
        self.cache("bridge_diag", data, msg)

    def twist_callback(self, name, linear_name, angular_name, msg):
        self.cache(name, {
            linear_name: float(msg.twist.linear.x),
            angular_name: float(msg.twist.angular.z),
        }, msg)

    def follower_cmd_callback(self, msg):
        self.twist_callback("follower_cmd", "follower_linear_x", "follower_angular_z", msg)
        self.cache("bridge_input", {
            "bridge_input_linear_x": float(msg.twist.linear.x),
            "bridge_input_angular_z": float(msg.twist.angular.z),
        }, msg)

    def bridge_output_callback(self, msg):
        self.twist_callback(
            "bridge_output",
            "bridge_output_linear_x",
            "bridge_output_angular_z",
            msg,
        )

    def operator_callback(self, msg):
        self.twist_callback("operator", "operator_linear_x", "operator_angular_z", msg)

    def operator_deadman_callback(self, msg):
        self.cache("operator_deadman", {"operator_deadman": bool(msg.data)}, msg)

    def rc_out_callback(self, msg):
        channels = list(getattr(msg, "channels", []))
        self.cache("rc_out", {
            f"rc_out_{index + 1}": channels[index]
            for index in range(min(len(channels), 18))
        }, msg)

    def battery_callback(self, msg):
        percentage = float(msg.percentage)
        self.cache("battery", {
            "battery_voltage": float(msg.voltage),
            "battery_current": float(msg.current),
            "battery_percent": percentage * 100.0 if percentage >= 0.0 else "",
        }, msg)

    def sys_status_callback(self, msg):
        self.cache("sys_status", {
            "sys_voltage": (
                float(msg.voltage_battery) / 1000.0
                if msg.voltage_battery not in (0, 65535)
                else ""
            ),
            "sys_current": float(msg.current_battery) / 100.0 if msg.current_battery >= 0 else "",
            "sys_remaining": int(msg.battery_remaining) if msg.battery_remaining >= 0 else "",
        }, msg)

    def timesync_callback(self, msg):
        self.cache("timesync", {
            "timesync_rtt_ms": getattr(msg, "round_trip_time_ms", ""),
            "timesync_observed_offset_ns": getattr(msg, "observed_offset_ns", ""),
            "timesync_estimated_offset_ns": getattr(msg, "estimated_offset_ns", ""),
        }, msg)

    def reset_callback(self, request, response):
        try:
            status = self.core.reset(
                request.label,
                metadata={
                    "logger_version": 1,
                    "record_rate_hz": self.record_rate_hz,
                    "missing_value_policy": "blank (never synthetic zero)",
                    "coordinate_note": (
                        "Controller uses live body-frame gate geometry. "
                        "TRACK map fields are diagnostic transforms; "
                        "frozen PASS fields are unavailable and remain blank."
                    ),
                },
                now_monotonic=time.monotonic(),
                ros_time=self._ros_time(),
            )
            response.success = True
            response.message = (
                "Diagnostic log pending; file will be created on first arm"
            )
            # Capture configuration while disarmed/pending so parameter
            # traffic cannot add work at the instant propulsion is armed.
            self.capture_parameters()
        except Exception as exc:
            status = self.core.status()
            response.success = False
            response.message = f"Could not prepare diagnostic log: {exc}"
        response.state = str(status["state"])
        response.mission_id = int(status["mission_id"] or 0)
        response.file_path = str(status["file_path"] or "")
        self.publish_status()
        return response

    def record_snapshot(self):
        now = time.monotonic()
        row = {"ros_time": self._ros_time()}
        with self.lock:
            samples = {key: dict(value) for key, value in self.samples.items()}
        for name, sample in samples.items():
            row.update(sample["values"])
            row[f"{name}_stamp"] = sample["stamp"]
            row[f"{name}_age_s"] = max(0.0, now - sample["receipt"])
            sequence = sample["sequence"]
            row[f"{name}_new"] = sequence != self.written_sequences.get(name)
            self.written_sequences[name] = sequence
        try:
            self.core.add_snapshot(row, now, row["ros_time"])
        except Exception as exc:
            self.get_logger().error(f"Mission log write failed: {exc}")

    def capture_parameters(self):
        self._capture_ros_parameters(
            "follower",
            self.follower_parameter_client,
            FOLLOWER_PARAMETERS,
        )
        self._capture_ros_parameters(
            "bridge",
            self.bridge_parameter_client,
            BRIDGE_PARAMETERS,
        )
        if (
            self.autopilot_param_client is not None
            and self.autopilot_param_client.service_is_ready()
        ):
            for name in AUTOPILOT_PARAMETERS:
                request = ParamGet.Request()
                request.param_id = name
                future = self.autopilot_param_client.call_async(request)
                future.add_done_callback(
                    lambda item, key=name:
                    self._autopilot_parameter_done(key, item)
                )

    def _capture_ros_parameters(self, prefix, client, names):
        try:
            future = client.get_parameters(names)
            future.add_done_callback(
                lambda item, key=prefix, requested=tuple(names):
                self._ros_parameters_done(key, requested, item)
            )
        except Exception as exc:
            self.core.update_metadata({f"{prefix}_parameter_error": str(exc)})

    def _ros_parameters_done(self, prefix, names, future):
        try:
            response = future.result()
            result = {}
            for name, parameter_value in zip(names, response.values):
                value = parameter_value_to_python(parameter_value)
                result[f"{prefix}.{name}"] = value
            self.core.update_metadata(result)
        except Exception as exc:
            self.core.update_metadata({f"{prefix}_parameter_error": str(exc)})

    def _autopilot_parameter_done(self, name, future):
        try:
            response = future.result()
            if response is None or not response.success:
                return
            value = response.value
            real = float(getattr(value, "real", 0.0))
            integer = int(getattr(value, "integer", 0))
            selected = real if real != 0.0 or integer == 0 else integer
            self.core.update_metadata({f"ardupilot.{name}": selected})
        except Exception:
            return

    def publish_status(self):
        msg = String()
        msg.data = json.dumps(self.core.status(), separators=(",", ":"))
        self.status_pub.publish(msg)

    def destroy_node(self):
        try:
            self.core.close("NODE_SHUTDOWN", time.monotonic(), self._ros_time())
        finally:
            return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MissionLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
