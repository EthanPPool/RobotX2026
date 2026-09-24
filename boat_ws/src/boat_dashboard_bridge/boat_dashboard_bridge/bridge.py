#!/usr/bin/env python3

import base64
import json
import math
import socket
import struct
import subprocess
import threading
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State, SysStatus
from mavros_msgs.srv import CommandBool, SetMode
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import (
    BatteryState,
    Imu,
    NavSatFix,
    PointCloud2,
    PointField,
)
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, Trigger
from tf2_ros import Buffer, TransformListener

from boat_interfaces.msg import DetectedObjectArray, Gate
from boat_interfaces.srv import ResetMissionLog


class BoatDashboardBridge(Node):

    def __init__(self):
        super().__init__("boat_dashboard_bridge")

        self.lock = threading.RLock()
        self.client_lock = threading.RLock()
        self.send_lock = threading.RLock()
        self.action_lock = threading.RLock()
        self.client_socket = None

        self.mission_service = (
            "robotx-usv-autonomy.service"
        )
        self.mission_process_state = "unknown"
        self.mission_process_checked_at = None

        self.software_stop_state = "UNKNOWN"
        self.control_state = "BOOT SAFE"

        # Local Jetson monotonic timestamps for ROS source
        # freshness. Only ages, never absolute monotonic times,
        # are sent across machines.
        self.state_last_rx = None
        self.battery_last_rx = None
        self.gate_last_rx = None
        self.bridge_last_rx = None
        self.logger_last_rx = None

        # USV_VISUALIZATION_V1
        # Display-only state. No control or safety decisions use it.
        self.cloud_last_rx = None
        self.local_pose_last_rx = None
        self.objects_last_rx = None
        self.trajectory_last_append = None

        self.visualization = {
            "version": 1,
            "cloud_frame": None,
            "cloud_points": [],
            "buoys": [],
            "gate": None,
            "local_pose": None,
            "trajectory": [],
            "attitude": {
                "roll_deg": None,
                "pitch_deg": None,
                "yaw_deg": None,
            },
        }

        self.visualization_max_points = 650
        self.visualization_max_trajectory = 400

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self,
            spin_thread=False,
        )

        # Browser/Xbox operator input. Input arrives from
        # Beeptop over TCP and is republished locally on ROS.
        self.operator_connected = False
        self.operator_deadman = False
        self.operator_forward = 0.0
        self.operator_yaw = 0.0
        self.operator_last_rx = None

        self.operator_timeout = 0.30
        self.operator_max_forward = 0.15
        self.operator_max_yaw = 0.15

        # A fresh LB press requests MANUAL through the
        # mavros_command_bridge. Once MANUAL is confirmed,
        # arm the vehicle asynchronously.
        self.operator_arm_lock = threading.Lock()
        self.operator_arm_thread = None

        self.telemetry = {
            "connected": False,
            "armed": False,
            "mode": "UNKNOWN",

            "latitude": None,
            "longitude": None,
            "altitude": None,
            "heading_deg": None,

            "voltage": None,
            "battery_percent": None,
            "battery_current": None,
            "battery_remaining": None,

            "buoy_count": 0,

            "gate_confidence": None,
            "gate_x": None,
            "gate_y": None,

            "control_forward": 0.0,
            "control_yaw": 0.0,

            "mission_state": None,

            "bridge_forward": 0.0,
            "bridge_yaw": 0.0,

            "log_state": "UNAVAILABLE",
            "log_pending": False,
            "log_recording": False,
            "log_mission_id": None,
            "log_label": "",
            "log_file_path": None,
            "log_row_count": 0,
            "log_buffer_rows": 0,
            "log_last_end_reason": None,
            "log_last_error": None,
        }

        # MAVROS telemetry
        self.create_subscription(
            State,
            "/mavros/state",
            self.state_callback,
            10,
        )

        self.create_subscription(
            NavSatFix,
            "/mavros/global_position/global",
            self.gps_callback,
            10,
        )

        self.create_subscription(
            Imu,
            "/mavros/imu/data",
            self.imu_callback,
            10,
        )

        self.create_subscription(
            BatteryState,
            "/mavros/battery",
            self.battery_callback,
            10,
        )

        self.create_subscription(
            SysStatus,
            "/mavros/sys_status",
            self.sys_status_callback,
            10,
        )

        # RobotX boat telemetry
        self.create_subscription(
            DetectedObjectArray,
            "/perception/objects",
            self.objects_callback,
            10,
        )

        self.create_subscription(
            Gate,
            "/perception/gate",
            self.gate_callback,
            10,
        )

        self.create_subscription(
            TwistStamped,
            "/control/cmd_vel",
            self.control_callback,
            10,
        )

        self.create_subscription(
            String,
            "/mission/state",
            self.mission_callback,
            10,
        )

        self.create_subscription(
            TwistStamped,
            "/mavros/setpoint_velocity/cmd_vel",
            self.bridge_callback,
            10,
        )

        self.create_subscription(
            String,
            "/mission_logger/status",
            self.logger_status_callback,
            10,
        )

        self.create_subscription(
            PoseStamped,
            "/mavros/local_position/pose",
            self.local_pose_callback,
            qos_profile_sensor_data,
        )

        self.create_subscription(
            PointCloud2,
            "/unilidar/cloud",
            self.pointcloud_callback,
            qos_profile_sensor_data,
        )

        # Local control services. These are intentionally
        # executed on the Jetson so safety transactions do not
        # depend on the ground-station TCP link remaining alive.
        self.estop_client = self.create_client(
            SetBool,
            "/vehicle/software_estop",
        )

        self.autonomy_client = self.create_client(
            SetBool,
            "/vehicle/set_autonomy",
        )

        self.arm_client = self.create_client(
            CommandBool,
            "/mavros/cmd/arming",
        )

        self.mode_client = self.create_client(
            SetMode,
            "/mavros/set_mode",
        )

        self.reset_client = self.create_client(
            Trigger,
            "/control/reset_mission",
        )

        self.logger_reset_client = self.create_client(
            ResetMissionLog,
            "/mission_logger/reset",
        )

        self.operator_pub = self.create_publisher(
            TwistStamped,
            "/operator/cmd_vel",
            10,
        )

        self.operator_deadman_pub = self.create_publisher(
            Bool,
            "/operator/deadman",
            10,
        )

        # Keep publishing operator state locally at 20 Hz.
        # If TCP/browser input goes stale, publish neutral and
        # release the deadman automatically.
        self.create_timer(
            0.05,
            self.publish_operator_command,
        )

        # 5 Hz output to ground station.
        #
        # Keep this independent of the ROS executor so heavy
        # MAVROS/perception callback traffic cannot starve the
        # Beeptop telemetry stream.
        self.telemetry_thread = threading.Thread(
            target=self.telemetry_loop,
            daemon=True,
            name="dashboard-telemetry",
        )
        self.telemetry_thread.start()

        # Separate 2 Hz visualization stream. This prevents large
        # point-cloud payloads from changing the normal 5 Hz telemetry
        # behavior used by control/status displays.
        self.visualization_thread = threading.Thread(
            target=self.visualization_loop,
            daemon=True,
            name="dashboard-visualization",
        )
        self.visualization_thread.start()

        self.server_thread = threading.Thread(
            target=self.server_loop,
            daemon=True,
            name="dashboard-tcp-server",
        )
        self.server_thread.start()

        self.get_logger().info(
            "Boat dashboard bridge listening on TCP 0.0.0.0:8765"
        )

    # ========================================================
    # ROS CALLBACKS
    # ========================================================

    def state_callback(self, msg):
        with self.lock:
            self.state_last_rx = time.monotonic()
            self.telemetry["connected"] = bool(msg.connected)
            self.telemetry["armed"] = bool(msg.armed)
            self.telemetry["mode"] = (
                str(msg.mode)
                if msg.mode
                else "UNKNOWN"
            )

    def gps_callback(self, msg):
        with self.lock:
            if math.isfinite(msg.latitude):
                self.telemetry["latitude"] = float(msg.latitude)

            if math.isfinite(msg.longitude):
                self.telemetry["longitude"] = float(msg.longitude)

            if math.isfinite(msg.altitude):
                self.telemetry["altitude"] = float(msg.altitude)

    def imu_callback(self, msg):
        q = msg.orientation

        sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        if abs(sinp) >= 1.0:
            pitch = math.copysign(math.pi / 2.0, sinp)
        else:
            pitch = math.asin(sinp)

        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        roll_deg = math.degrees(roll)
        pitch_deg = math.degrees(pitch)
        yaw_deg = math.degrees(yaw)
        heading = (yaw_deg + 360.0) % 360.0

        with self.lock:
            self.telemetry["heading_deg"] = heading
            self.visualization["attitude"] = {
                "roll_deg": round(roll_deg, 3),
                "pitch_deg": round(pitch_deg, 3),
                "yaw_deg": round(yaw_deg, 3),
            }

    def battery_callback(self, msg):
        with self.lock:
            self.battery_last_rx = time.monotonic()
            if math.isfinite(msg.voltage):
                self.telemetry["voltage"] = float(msg.voltage)

            if (
                math.isfinite(msg.percentage)
                and msg.percentage >= 0.0
            ):
                self.telemetry["battery_percent"] = (
                    float(msg.percentage)
                    * 100.0
                )

    def sys_status_callback(self, msg):
        voltage_raw = int(msg.voltage_battery)
        current_raw = int(msg.current_battery)
        remaining_raw = int(msg.battery_remaining)

        with self.lock:
            self.battery_last_rx = time.monotonic()

            if (
                voltage_raw > 0
                and voltage_raw != 65535
            ):
                self.telemetry["voltage"] = (
                    voltage_raw / 1000.0
                )

            if current_raw >= 0:
                self.telemetry["battery_current"] = (
                    current_raw / 100.0
                )
            else:
                self.telemetry["battery_current"] = None

            if remaining_raw >= 0:
                remaining = float(remaining_raw)

                self.telemetry["battery_remaining"] = remaining
                self.telemetry["battery_percent"] = remaining

            else:
                self.telemetry["battery_remaining"] = None

    def objects_callback(self, msg):
        color_names = {
            0: "unknown",
            1: "red",
            2: "green",
            3: "yellow",
            4: "black",
            5: "white",
        }

        objects = []
        for item in msg.objects[:32]:
            p = item.position
            if not all(math.isfinite(v) for v in (p.x, p.y, p.z)):
                continue
            objects.append({
                "id": int(item.id),
                "type": int(item.object_type),
                "color": color_names.get(int(item.color), "unknown"),
                "x": round(float(p.x), 3),
                "y": round(float(p.y), 3),
                "z": round(float(p.z), 3),
                "confidence": round(float(item.confidence), 3),
            })

        with self.lock:
            self.objects_last_rx = time.monotonic()
            self.telemetry["buoy_count"] = len(msg.objects)
            self.visualization["buoys"] = objects

    def gate_callback(self, msg):
        gate = {
            "left": [
                round(float(msg.left_marker.x), 3),
                round(float(msg.left_marker.y), 3),
                round(float(msg.left_marker.z), 3),
            ],
            "right": [
                round(float(msg.right_marker.x), 3),
                round(float(msg.right_marker.y), 3),
                round(float(msg.right_marker.z), 3),
            ],
            "center": [
                round(float(msg.center.x), 3),
                round(float(msg.center.y), 3),
                round(float(msg.center.z), 3),
            ],
            "width": round(float(msg.width), 3),
            "confidence": round(float(msg.confidence), 3),
        }

        with self.lock:
            self.gate_last_rx = time.monotonic()
            self.telemetry["gate_confidence"] = float(msg.confidence)
            self.telemetry["gate_x"] = float(msg.center.x)
            self.telemetry["gate_y"] = float(msg.center.y)
            self.visualization["gate"] = gate

    def local_pose_callback(self, msg):
        p = msg.pose.position
        if not all(math.isfinite(v) for v in (p.x, p.y, p.z)):
            return

        now = time.monotonic()
        point = [round(float(p.x), 3), round(float(p.y), 3), round(float(p.z), 3)]

        with self.lock:
            self.local_pose_last_rx = now
            self.visualization["local_pose"] = point
            trajectory = self.visualization["trajectory"]

            append = not trajectory
            if trajectory:
                last = trajectory[-1]
                distance = math.hypot(point[0] - last[0], point[1] - last[1])
                if distance >= 0.10:
                    append = True
                elif self.trajectory_last_append is None or (now - self.trajectory_last_append) >= 1.0:
                    append = True

            if append:
                trajectory.append(point)
                if len(trajectory) > self.visualization_max_trajectory:
                    del trajectory[:len(trajectory) - self.visualization_max_trajectory]
                self.trajectory_last_append = now

    @staticmethod
    def _point_field_value(data, offset, field, endian):
        if field.datatype == PointField.FLOAT32:
            return struct.unpack_from(endian + "f", data, offset)[0]
        if field.datatype == PointField.FLOAT64:
            return struct.unpack_from(endian + "d", data, offset)[0]
        return None

    @staticmethod
    def _apply_transform(point, transform):
        x, y, z = point
        q = transform.transform.rotation
        t = transform.transform.translation

        tx = 2.0 * (q.y * z - q.z * y)
        ty = 2.0 * (q.z * x - q.x * z)
        tz = 2.0 * (q.x * y - q.y * x)

        rx = x + q.w * tx + (q.y * tz - q.z * ty)
        ry = y + q.w * ty + (q.z * tx - q.x * tz)
        rz = z + q.w * tz + (q.x * ty - q.y * tx)

        return (rx + t.x, ry + t.y, rz + t.z)

    def pointcloud_callback(self, msg):
        fields = {field.name: field for field in msg.fields}
        if not all(name in fields for name in ("x", "y", "z")):
            return

        total = int(msg.width) * int(msg.height)
        if total <= 0 or msg.point_step <= 0 or msg.width <= 0:
            return

        stride = max(1, int(math.ceil(total / self.visualization_max_points)))
        endian = ">" if msg.is_bigendian else "<"
        points = []

        source_frame = str(msg.header.frame_id or "lidar_link")
        cloud_transform = None
        cloud_frame = source_frame
        cloud_transform_ok = source_frame == "base_link"

        if not cloud_transform_ok:
            try:
                cloud_transform = self.tf_buffer.lookup_transform(
                    "base_link",
                    source_frame,
                    Time(),
                )
                cloud_frame = "base_link"
                cloud_transform_ok = True
            except Exception:
                cloud_transform = None

        for index in range(0, total, stride):
            row = index // int(msg.width)
            col = index % int(msg.width)
            base = row * int(msg.row_step) + col * int(msg.point_step)
            try:
                x = self._point_field_value(msg.data, base + fields["x"].offset, fields["x"], endian)
                y = self._point_field_value(msg.data, base + fields["y"].offset, fields["y"], endian)
                z = self._point_field_value(msg.data, base + fields["z"].offset, fields["z"], endian)
            except (struct.error, IndexError, TypeError):
                continue

            if x is None or y is None or z is None:
                continue
            if not all(math.isfinite(v) for v in (x, y, z)):
                continue
            if (x * x + y * y + z * z) > 3600.0:
                continue

            if cloud_transform is not None:
                x, y, z = self._apply_transform(
                    (x, y, z),
                    cloud_transform,
                )

            points.append([round(float(x), 3), round(float(y), 3), round(float(z), 3)])
            if len(points) >= self.visualization_max_points:
                break

        with self.lock:
            self.cloud_last_rx = time.monotonic()
            self.visualization["cloud_frame"] = cloud_frame
            self.visualization["cloud_source_frame"] = source_frame
            self.visualization["cloud_transform_ok"] = cloud_transform_ok
            self.visualization["cloud_points"] = points

    def control_callback(self, msg):
        with self.lock:
            self.telemetry["control_forward"] = float(
                msg.twist.linear.x
            )
            self.telemetry["control_yaw"] = float(
                msg.twist.angular.z
            )

    def mission_callback(self, msg):
        state = str(msg.data).strip()

        with self.lock:
            self.telemetry["mission_state"] = (
                state if state else None
            )

    def bridge_callback(self, msg):
        with self.lock:
            self.bridge_last_rx = time.monotonic()

            self.telemetry["bridge_forward"] = float(
                msg.twist.linear.x
            )
            self.telemetry["bridge_yaw"] = float(
                msg.twist.angular.z
            )

    def logger_status_callback(self, msg):
        try:
            status = json.loads(msg.data)
        except (TypeError, json.JSONDecodeError):
            return

        if not isinstance(status, dict):
            return

        mapping = {
            "state": "log_state",
            "pending": "log_pending",
            "recording": "log_recording",
            "mission_id": "log_mission_id",
            "label": "log_label",
            "file_path": "log_file_path",
            "row_count": "log_row_count",
            "buffer_rows": "log_buffer_rows",
            "last_end_reason": "log_last_end_reason",
            "last_error": "log_last_error",
        }

        with self.lock:
            self.logger_last_rx = time.monotonic()

            for source, destination in mapping.items():
                if source in status:
                    self.telemetry[destination] = status[source]

    # ========================================================
    # LOCAL STATUS / SERVICE HELPERS
    # ========================================================

    def local_status(self):
        now = time.monotonic()

        with self.lock:
            data = dict(self.telemetry)

            state_age = (
                None
                if self.state_last_rx is None
                else now - self.state_last_rx
            )

            gate_age = (
                None
                if self.gate_last_rx is None
                else now - self.gate_last_rx
            )

            bridge_age = (
                None
                if self.bridge_last_rx is None
                else now - self.bridge_last_rx
            )

        online = bool(
            data["connected"]
            and state_age is not None
            and state_age < 2.0
        )

        gate_fresh = bool(
            gate_age is not None
            and gate_age <= 0.50
            and data["gate_confidence"] is not None
            and data["gate_confidence"] >= 0.75
            and data["gate_x"] is not None
            and data["gate_x"] > 0.0
        )

        control_ready = bool(
            abs(data["control_forward"]) > 0.0001
            or
            abs(data["control_yaw"]) > 0.0001
        )

        bridge_command_active = bool(
            bridge_age is not None
            and bridge_age <= 0.50
        )

        bridge_alive = bool(
            self.estop_client.service_is_ready()
            and self.autonomy_client.service_is_ready()
        )

        mode = str(data["mode"]).upper()

        return {
            "online": online,
            "armed": bool(data["armed"]),
            "mode": mode,
            "gate_fresh": gate_fresh,
            "control_ready": control_ready,
            "bridge_command_active": bridge_command_active,
            "bridge_alive": bridge_alive,
        }

    def mission_process_status(self, force=False):
        now = time.monotonic()

        with self.lock:
            if (
                not force
                and self.mission_process_checked_at
                is not None
                and (
                    now
                    - self.mission_process_checked_at
                ) < 1.0
            ):
                return self.mission_process_state

        try:
            result = subprocess.run(
                [
                    "/usr/bin/systemctl",
                    "is-active",
                    self.mission_service,
                ],
                capture_output=True,
                text=True,
                timeout=2.0,
                check=False,
            )

            state = (
                result.stdout.strip()
                or "unknown"
            )

        except Exception as exc:
            self.get_logger().warn(
                "Mission service status failed: "
                + str(exc)
            )
            state = "unknown"

        with self.lock:
            self.mission_process_state = state
            self.mission_process_checked_at = now

        return state


    def execute_mission_process(self, action):
        action = str(
            action or "status"
        ).strip().lower()

        if action == "status":
            state = self.mission_process_status(
                force=True
            )

            return (
                True,
                f"Mission process is {state}",
            )

        if action not in ("start", "stop"):
            return (
                False,
                "mission_process action must be "
                "start, stop, or status",
            )

        # STOP also revokes autonomous authority.
        # START intentionally does NOT enable autonomy,
        # clear software stop, arm, or change mode.
        if action == "stop":
            self.call_bool_service(
                self.autonomy_client,
                False,
            )

        try:
            result = subprocess.run(
                [
                    "sudo",
                    "-n",
                    "/usr/bin/systemctl",
                    action,
                    self.mission_service,
                ],
                capture_output=True,
                text=True,
                timeout=8.0,
                check=False,
            )

        except Exception as exc:
            return (
                False,
                "Mission process command failed: "
                + str(exc),
            )

        state = self.mission_process_status(
            force=True
        )

        desired = (
            "active"
            if action == "start"
            else "inactive"
        )

        success = state == desired

        detail = (
            result.stderr.strip()
            or result.stdout.strip()
        )

        message = (
            f"Mission process {action}: "
            f"{state}"
        )

        if detail and not success:
            message += f" ({detail})"

        return success, message


    def get_mission_log_file(self, kind):
        kind = str(kind or "csv").strip().lower()
        if kind not in ("csv", "metadata"):
            return False, "Log kind must be csv or metadata", {}

        with self.lock:
            raw_path = self.telemetry.get("log_file_path")

        if not raw_path:
            return False, "No mission log file is available yet", {}

        root = (Path.home() / "robotx_logs").resolve()
        csv_path = Path(str(raw_path)).expanduser().resolve()
        if csv_path != root and root not in csv_path.parents:
            return False, "Mission log path is outside ~/robotx_logs", {}

        if kind == "metadata":
            path = csv_path.with_name(csv_path.stem + "_metadata.json")
            mime_type = "application/json"
        else:
            path = csv_path
            mime_type = "text/csv"

        if not path.is_file():
            return False, f"{kind} log file does not exist", {}

        size = path.stat().st_size
        if size > 50 * 1024 * 1024:
            return False, "Mission log exceeds 50 MiB transfer limit", {}

        payload = base64.b64encode(path.read_bytes()).decode("ascii")
        return True, f"{kind} log ready", {
            "filename": path.name,
            "mime_type": mime_type,
            "file_size": size,
            "file_content_b64": payload,
        }


    def wait_future(self, future, timeout=2.0):
        deadline = time.monotonic() + timeout

        while (
            not future.done()
            and time.monotonic() < deadline
        ):
            time.sleep(0.02)

        return future.done()

    def call_bool_service(
        self,
        client,
        value,
        timeout=2.0,
    ):
        if not client.wait_for_service(
            timeout_sec=0.50
        ):
            return (
                False,
                "service unavailable",
            )

        request = SetBool.Request()
        request.data = bool(value)

        future = client.call_async(request)

        if not self.wait_future(
            future,
            timeout,
        ):
            return (
                False,
                "service timed out",
            )

        response = future.result()

        if response is None:
            return (
                False,
                "service returned no response",
            )

        return (
            bool(response.success),
            str(response.message),
        )

    def call_arm_service(
        self,
        value,
        timeout=2.0,
    ):
        if not self.arm_client.wait_for_service(
            timeout_sec=0.50
        ):
            return (
                False,
                "Arming service unavailable",
            )

        request = CommandBool.Request()
        request.value = bool(value)

        future = self.arm_client.call_async(
            request
        )

        if not self.wait_future(
            future,
            timeout,
        ):
            return (
                False,
                "Arming service timed out",
            )

        response = future.result()

        if (
            response is None
            or not response.success
        ):
            return (
                False,
                "Arming request rejected",
            )

        return (
            True,
            "ARM requested"
            if value
            else "DISARM requested",
        )

    def call_mode_service(
        self,
        mode,
        timeout=2.0,
    ):
        if not self.mode_client.wait_for_service(
            timeout_sec=0.50
        ):
            return (
                False,
                "Mode service unavailable",
            )

        request = SetMode.Request()
        request.base_mode = 0
        request.custom_mode = str(mode)

        future = self.mode_client.call_async(
            request
        )

        if not self.wait_future(
            future,
            timeout,
        ):
            return (
                False,
                "Mode service timed out",
            )

        response = future.result()

        if (
            response is None
            or not response.mode_sent
        ):
            return (
                False,
                f"Mode {mode} rejected",
            )

        return (
            True,
            f"Mode request sent: {mode}",
        )

    def call_reset_service(
        self,
        timeout=2.0,
    ):
        if not self.reset_client.wait_for_service(
            timeout_sec=0.50
        ):
            return (
                False,
                "Mission reset service unavailable",
            )

        request = Trigger.Request()

        future = self.reset_client.call_async(
            request
        )

        if not self.wait_future(
            future,
            timeout,
        ):
            return (
                False,
                "Mission reset timed out",
            )

        response = future.result()

        if response is None:
            return (
                False,
                "Mission reset returned no response",
            )

        return (
            bool(response.success),
            str(response.message),
        )

    def call_logger_reset_service(
        self,
        label="",
        timeout=2.0,
    ):
        details = {
            "state": "UNAVAILABLE",
            "mission_id": 0,
            "file_path": "",
        }

        if not self.logger_reset_client.wait_for_service(
            timeout_sec=0.50
        ):
            return (
                False,
                "Diagnostic logger reset service unavailable",
                details,
            )

        request = ResetMissionLog.Request()
        request.label = str(label or "")

        future = self.logger_reset_client.call_async(
            request
        )

        if not self.wait_future(
            future,
            timeout,
        ):
            return (
                False,
                "Diagnostic logger reset timed out",
                details,
            )

        response = future.result()

        if response is None:
            return (
                False,
                "Diagnostic logger returned no response",
                details,
            )

        details = {
            "state": str(response.state),
            "mission_id": int(response.mission_id),
            "file_path": str(response.file_path),
        }

        return (
            bool(response.success),
            str(response.message),
            details,
        )

    # ========================================================
    # JETSON-LOCAL USV CONTROL ACTIONS
    # ========================================================

    def execute_fail_safe_stop(self):
        self.call_bool_service(
            self.estop_client,
            True,
        )

        self.call_bool_service(
            self.autonomy_client,
            False,
        )

        self.software_stop_state = "ENGAGED"
        self.control_state = "STOPPED / HOLD"

    def execute_stop(self):
        with self.action_lock:
            messages = []
            success = True

            ok, msg = self.call_bool_service(
                self.estop_client,
                True,
            )

            messages.append(
                "software_stop: " + msg
            )

            if ok:
                self.software_stop_state = "ENGAGED"
            else:
                success = False

            ok, msg = self.call_bool_service(
                self.autonomy_client,
                False,
            )

            messages.append(
                "autonomy: " + msg
            )

            if not ok:
                success = False

            self.control_state = (
                "STOPPED / HOLD"
                if success
                else "STOP COMMAND FAILED"
            )

            return (
                success,
                " | ".join(messages),
            )

    def execute_clear_stop(self):
        with self.action_lock:
            status = self.local_status()

            if not status["online"]:
                return (
                    False,
                    "Clear stop rejected: MAVROS "
                    "is not connected",
                )

            if status["armed"]:
                return (
                    False,
                    "Clear stop rejected: "
                    "vehicle is armed",
                )

            ok, autonomy_msg = (
                self.call_bool_service(
                    self.autonomy_client,
                    False,
                )
            )

            if not ok:
                return (
                    False,
                    "Could not verify autonomy OFF: "
                    + autonomy_msg,
                )

            ok, stop_msg = (
                self.call_bool_service(
                    self.estop_client,
                    False,
                )
            )

            if not ok:
                return (
                    False,
                    "Could not clear software stop: "
                    + stop_msg,
                )

            self.software_stop_state = "CLEARED"
            self.control_state = (
                "STOP CLEARED / AUTONOMY OFF"
            )

            return (
                True,
                "Software stop CLEARED. "
                "Autonomy remains OFF and "
                "vehicle remains DISARMED.",
            )

    def execute_enable(self):
        with self.action_lock:
            status = self.local_status()

            if not status["online"]:
                return (
                    False,
                    "Enable rejected: MAVROS "
                    "is not connected",
                )

            # Gate perception and follower motion are NOT
            # prerequisites for entering autonomy.
            #
            # The vehicle may enter GUIDED/autonomy while
            # DISARMED and wait safely for perception.
            #
            # Gate visibility is enforced at ARM instead.
            if not status["bridge_alive"]:
                return (
                    False,
                    "Enable rejected: bridge "
                    "services unavailable",
                )

            messages = []

            # Start from a known safe state.
            ok, msg = self.call_bool_service(
                self.estop_client,
                True,
            )

            messages.append(
                "safety: " + msg
            )

            if not ok:
                return (
                    False,
                    " | ".join(messages),
                )

            self.software_stop_state = "ENGAGED"

            status = self.local_status()

            # Never transition an armed USV into GUIDED.
            if status["armed"]:
                ok, msg = self.call_arm_service(
                    False
                )

                messages.append(
                    "disarm: " + msg
                )

                if not ok:
                    return (
                        False,
                        " | ".join(messages),
                    )

                deadline = (
                    time.monotonic() + 1.5
                )

                while (
                    time.monotonic()
                    < deadline
                ):
                    if not self.local_status()["armed"]:
                        break

                    time.sleep(0.02)

                if self.local_status()["armed"]:
                    return (
                        False,
                        "Enable aborted: USV did "
                        "not confirm DISARM",
                    )

            # Clear software stop while still disarmed.
            ok, msg = self.call_bool_service(
                self.estop_client,
                False,
            )

            messages.append(
                "software_stop: " + msg
            )

            if not ok:
                self.execute_fail_safe_stop()

                return (
                    False,
                    " | ".join(messages),
                )

            self.software_stop_state = "CLEARED"

            # Enter GUIDED while still disarmed.
            ok, msg = self.call_mode_service(
                "GUIDED"
            )

            messages.append(
                "mode: " + msg
            )

            if not ok:
                self.execute_fail_safe_stop()

                return (
                    False,
                    " | ".join(messages),
                )

            deadline = (
                time.monotonic() + 1.5
            )

            while (
                time.monotonic() < deadline
            ):
                if (
                    self.local_status()["mode"]
                    == "GUIDED"
                ):
                    break

                time.sleep(0.02)

            status = self.local_status()

            if status["mode"] != "GUIDED":
                self.execute_fail_safe_stop()

                return (
                    False,
                    "Enable aborted: GUIDED "
                    "was not confirmed",
                )

            if status["armed"]:
                self.execute_fail_safe_stop()

                return (
                    False,
                    "Enable aborted: vehicle "
                    "unexpectedly armed",
                )

            with self.lock:
                previous_bridge_time = (
                    self.bridge_last_rx
                )

            ok, msg = self.call_bool_service(
                self.autonomy_client,
                True,
            )

            messages.append(
                "autonomy: " + msg
            )

            if not ok:
                self.execute_fail_safe_stop()

                return (
                    False,
                    " | ".join(messages),
                )

            deadline = (
                time.monotonic() + 1.25
            )

            fresh_bridge = False

            while (
                time.monotonic() < deadline
            ):
                with self.lock:
                    current = self.bridge_last_rx

                if (
                    current is not None
                    and (
                        previous_bridge_time is None
                        or
                        current > previous_bridge_time
                    )
                ):
                    fresh_bridge = True
                    break

                time.sleep(0.02)

            if not fresh_bridge:
                self.execute_fail_safe_stop()

                return (
                    False,
                    "Enable aborted: bridge "
                    "did not produce a fresh "
                    "autonomous setpoint",
                )

            status = self.local_status()

            if (
                status["armed"]
                or status["mode"] != "GUIDED"
            ):
                self.execute_fail_safe_stop()

                return (
                    False,
                    "Enable aborted: vehicle "
                    "state changed during preparation",
                )

            self.control_state = (
                "AUTONOMY READY / DISARMED"
            )

            return (
                True,
                "Autonomy READY: GUIDED entered "
                "while DISARMED and fresh autonomous "
                "setpoints are streaming. "
                "Press ARM to start propulsion.",
            )

    def execute_arm(self):
        with self.action_lock:
            status = self.local_status()

            if not status["online"]:
                return (
                    False,
                    "ARM rejected: MAVROS "
                    "is not connected",
                )

            # Dashboard ARM is the transition that can
            # actually permit propulsion. Require current,
            # high-confidence gate perception here rather
            # than when autonomy is merely enabled.
            if not status["gate_fresh"]:
                return (
                    False,
                    "ARM rejected: no fresh "
                    "high-confidence gate visible",
                )

            if status["mode"] == "GUIDED":
                if (
                    self.control_state
                    != "AUTONOMY READY / DISARMED"
                    or
                    not status[
                        "bridge_command_active"
                    ]
                ):
                    return (
                        False,
                        "ARM rejected: GUIDED "
                        "requires ENABLE AUTONOMY first",
                    )

            ok, msg = self.call_arm_service(
                True
            )

            if (
                ok
                and status["mode"] == "GUIDED"
            ):
                self.control_state = "ENABLED"

            return (
                ok,
                msg,
            )

    def execute_disarm(self):
        with self.action_lock:
            stop_ok, stop_msg = (
                self.execute_stop()
            )

            arm_ok, arm_msg = (
                self.call_arm_service(
                    False
                )
            )

            self.control_state = (
                "DISARMED"
                if arm_ok
                else "DISARM FAILED"
            )

            return (
                stop_ok and arm_ok,
                stop_msg
                + " | disarm: "
                + arm_msg,
            )

    def execute_reset_mission(self, label=""):
        with self.action_lock:
            stop_ok, stop_msg = (
                self.execute_stop()
            )

            reset_ok, reset_msg = (
                self.call_reset_service()
            )

            (
                logger_ok,
                logger_msg,
                logger_details,
            ) = self.call_logger_reset_service(
                label
            )

            if reset_ok:
                self.control_state = (
                    "MISSION RESET / STOPPED"
                )

            return (
                stop_ok and reset_ok and logger_ok,
                stop_msg
                + " | reset: "
                + reset_msg
                + " | logger: "
                + logger_msg,
                logger_details,
            )

    # ========================================================
    # REMOTE OPERATOR INPUT
    # ========================================================

    def receive_operator_input(self, data):
        if not isinstance(data, dict):
            return

        connected = bool(
            data.get("connected", False)
        )

        deadman = bool(
            data.get("deadman", False)
        )

        try:
            forward = float(
                data.get("forward", 0.0)
            )
            yaw = float(
                data.get("yaw", 0.0)
            )
        except (TypeError, ValueError):
            connected = False
            deadman = False
            forward = 0.0
            yaw = 0.0

        if not math.isfinite(forward):
            forward = 0.0

        if not math.isfinite(yaw):
            yaw = 0.0

        forward = max(
            -1.0,
            min(1.0, forward),
        )

        yaw = max(
            -1.0,
            min(1.0, yaw),
        )

        deadman_rising = False

        with self.lock:
            previous_deadman = bool(
                self.operator_deadman
            )

            self.operator_connected = connected

            self.operator_deadman = bool(
                connected and deadman
            )

            deadman_rising = bool(
                self.operator_deadman
                and not previous_deadman
            )

            if self.operator_deadman:
                self.operator_forward = (
                    forward
                    * self.operator_max_forward
                )

                self.operator_yaw = (
                    yaw
                    * self.operator_max_yaw
                )
            else:
                self.operator_forward = 0.0
                self.operator_yaw = 0.0

            self.operator_last_rx = (
                time.monotonic()
            )

        if deadman_rising:
            self.start_operator_auto_arm()

    def start_operator_auto_arm(self):
        with self.operator_arm_lock:

            if (
                self.operator_arm_thread is not None
                and self.operator_arm_thread.is_alive()
            ):
                return

            self.operator_arm_thread = threading.Thread(
                target=self.operator_auto_arm_worker,
                daemon=True,
                name="operator-auto-arm",
            )

            self.operator_arm_thread.start()

    def operator_auto_arm_worker(self):
        try:
            deadline = time.monotonic() + 3.0

            self.get_logger().warn(
                "LB AUTO ARM: waiting for MANUAL"
            )

            while (
                rclpy.ok()
                and time.monotonic() < deadline
            ):
                now = time.monotonic()

                with self.lock:
                    fresh = bool(
                        self.operator_connected
                        and self.operator_deadman
                        and self.operator_last_rx is not None
                        and (
                            now - self.operator_last_rx
                        ) <= self.operator_timeout
                    )

                    mode = str(
                        self.telemetry.get(
                            "mode",
                            "UNKNOWN",
                        )
                    ).upper()

                    armed = bool(
                        self.telemetry.get(
                            "armed",
                            False,
                        )
                    )

                if not fresh:
                    self.get_logger().warn(
                        "LB AUTO ARM cancelled: "
                        "deadman released or stale"
                    )
                    return

                if armed:
                    self.get_logger().info(
                        "LB AUTO ARM: already armed"
                    )
                    return

                if mode == "MANUAL":
                    self.get_logger().warn(
                        "LB AUTO ARM: MANUAL confirmed; "
                        "requesting ARM"
                    )

                    ok, message = (
                        self.call_arm_service(True)
                    )

                    if not ok:
                        self.get_logger().error(
                            "LB AUTO ARM failed: "
                            + message
                        )
                        return

                    confirm_deadline = (
                        time.monotonic() + 1.5
                    )

                    while (
                        rclpy.ok()
                        and time.monotonic()
                        < confirm_deadline
                    ):
                        with self.lock:
                            confirmed = bool(
                                self.telemetry.get(
                                    "armed",
                                    False,
                                )
                            )

                        if confirmed:
                            self.get_logger().warn(
                                "LB AUTO ARM: "
                                "ARMED in MANUAL"
                            )
                            return

                        time.sleep(0.05)

                    self.get_logger().error(
                        "LB AUTO ARM: arm request "
                        "sent but ARM was not confirmed"
                    )
                    return

                time.sleep(0.05)

            self.get_logger().error(
                "LB AUTO ARM timed out "
                "waiting for MANUAL"
            )

        finally:
            with self.operator_arm_lock:
                self.operator_arm_thread = None

    def publish_operator_command(self):
        now = time.monotonic()

        with self.lock:
            fresh = bool(
                self.operator_last_rx is not None
                and
                (
                    now - self.operator_last_rx
                ) <= self.operator_timeout
            )

            active = bool(
                fresh
                and self.operator_connected
                and self.operator_deadman
            )

            if active:
                forward = self.operator_forward
                yaw = self.operator_yaw
            else:
                forward = 0.0
                yaw = 0.0

                if not fresh:
                    self.operator_deadman = False
                    self.operator_forward = 0.0
                    self.operator_yaw = 0.0

        cmd = TwistStamped()
        cmd.header.stamp = (
            self.get_clock().now().to_msg()
        )
        cmd.header.frame_id = "base_link"
        cmd.twist.linear.x = float(forward)
        cmd.twist.angular.z = float(yaw)

        deadman = Bool()
        deadman.data = bool(active)

        self.operator_deadman_pub.publish(
            deadman
        )

        self.operator_pub.publish(cmd)

    def release_operator(self):
        with self.lock:
            self.operator_connected = False
            self.operator_deadman = False
            self.operator_forward = 0.0
            self.operator_yaw = 0.0
            self.operator_last_rx = None

        # Immediate neutral/release instead of waiting for
        # the next watchdog timer tick.
        self.publish_operator_command()


    # ========================================================
    # TCP SERVER
    # ========================================================

    def server_loop(self):
        server = socket.socket(
            socket.AF_INET,
            socket.SOCK_STREAM,
        )

        server.setsockopt(
            socket.SOL_SOCKET,
            socket.SO_REUSEADDR,
            1,
        )

        server.bind(
            ("0.0.0.0", 8765)
        )

        server.listen(1)

        while rclpy.ok():
            try:
                client, address = server.accept()

                self.get_logger().info(
                    f"Ground station connected: "
                    f"{address[0]}:{address[1]}"
                )

                with self.client_lock:
                    old_client = self.client_socket
                    self.client_socket = client

                if old_client is not None:
                    try:
                        old_client.close()
                    except OSError:
                        pass

                threading.Thread(
                    target=self.client_reader,
                    args=(client,),
                    daemon=True,
                ).start()

            except OSError:
                if not rclpy.ok():
                    break

    def client_reader(self, client):
        try:
            file_obj = client.makefile(
                "r",
                encoding="utf-8",
            )

            for line in file_obj:
                line = line.strip()

                if not line:
                    continue

                try:
                    message = json.loads(line)
                except json.JSONDecodeError:
                    continue

                self.handle_message(
                    client,
                    message,
                )

        except (
            ConnectionResetError,
            ConnectionAbortedError,
            OSError,
        ):
            pass

        finally:
            with self.client_lock:
                if self.client_socket is client:
                    self.client_socket = None

            try:
                client.close()
            except OSError:
                pass

            self.release_operator()

            self.get_logger().info(
                "Ground station disconnected"
            )

    def handle_message(self, client, message):
        if not isinstance(message, dict):
            return

        if message.get("type") == "operator_input":
            self.receive_operator_input(
                message.get("data", {})
            )
            return

        if message.get("type") != "command":
            return

        command = str(
            message.get("command", "")
        ).strip().lower()

        command_data = message.get("data", {})

        if not isinstance(command_data, dict):
            command_data = {}

        handlers = {
            "stop": self.execute_stop,
            "clear_stop": self.execute_clear_stop,
            "enable": self.execute_enable,
            "arm": self.execute_arm,
            "disarm": self.execute_disarm,
            "reset_mission": lambda: self.execute_reset_mission(
                command_data.get("label", "")
            ),
            "mission_process": lambda: self.execute_mission_process(
                command_data.get("action", "status")
            ),
            "get_log_file": lambda: self.get_mission_log_file(
                command_data.get("kind", "csv")
            ),
        }

        handler = handlers.get(command)
        response_extra = {}

        if handler is None:
            success = False
            result_message = (
                f"Unknown command: {command}"
            )
        else:
            self.get_logger().info(
                f"Remote command received: {command}"
            )

            try:
                result = handler()

                if (
                    isinstance(result, tuple)
                    and len(result) == 3
                ):
                    (
                        success,
                        result_message,
                        response_extra,
                    ) = result
                else:
                    (
                        success,
                        result_message,
                    ) = result

            except Exception as exc:
                self.get_logger().error(
                    f"Remote command {command} failed: "
                    f"{exc}"
                )

                success = False
                result_message = (
                    f"Internal command error: {exc}"
                )

        response = {
            "type": "response",
            "request_id": message.get("request_id"),
            "success": bool(success),
            "message": str(result_message),
        }

        if response_extra:
            response.update(response_extra)

        try:
            self.send_to_socket(
                client,
                response,
            )
        except OSError:
            pass

    # ========================================================
    # VISUALIZATION TELEMETRY
    # ========================================================

    def visualization_loop(self):
        period = 0.50
        while rclpy.ok():
            start = time.monotonic()
            try:
                self.publish_visualization()
            except Exception as exc:
                self.get_logger().warn("Visualization send loop error: " + str(exc))
            elapsed = time.monotonic() - start
            time.sleep(max(0.02, period - elapsed))

    def publish_visualization(self):
        now = time.monotonic()
        with self.lock:
            src = self.visualization
            data = {
                "version": 1,
                "cloud_frame": src.get("cloud_frame"),
                "cloud_source_frame": src.get("cloud_source_frame"),
                "cloud_transform_ok": bool(src.get("cloud_transform_ok", False)),
                "cloud_points": [list(p) for p in src.get("cloud_points", [])],
                "buoys": [dict(x) for x in src.get("buoys", [])],
                "gate": None if src.get("gate") is None else dict(src["gate"]),
                "local_pose": None if src.get("local_pose") is None else list(src["local_pose"]),
                "trajectory": [list(p) for p in src.get("trajectory", [])],
                "attitude": dict(src.get("attitude", {})),
                "cloud_age_sec": None if self.cloud_last_rx is None else now - self.cloud_last_rx,
                "local_pose_age_sec": None if self.local_pose_last_rx is None else now - self.local_pose_last_rx,
                "objects_age_sec": None if self.objects_last_rx is None else now - self.objects_last_rx,
                "gate_age_sec": None if self.gate_last_rx is None else now - self.gate_last_rx,
            }

        with self.client_lock:
            client = self.client_socket
        if client is None:
            return

        try:
            self.send_to_socket(client, {"type": "visualization", "data": data})
        except OSError:
            with self.client_lock:
                if self.client_socket is client:
                    self.client_socket = None

    # ========================================================
    # TELEMETRY
    # ========================================================

    def telemetry_loop(self):
        period = 0.2

        while rclpy.ok():
            start = time.monotonic()

            try:
                self.publish_telemetry()

            except Exception as exc:
                self.get_logger().warn(
                    f"Telemetry send loop error: {exc}"
                )

            elapsed = time.monotonic() - start

            time.sleep(
                max(
                    0.01,
                    period - elapsed,
                )
            )

    def publish_telemetry(self):
        now = time.monotonic()

        with self.lock:
            data = dict(self.telemetry)

            data["battery_age_sec"] = (
                None
                if self.battery_last_rx is None
                else now - self.battery_last_rx
            )

            data["gate_age_sec"] = (
                None
                if self.gate_last_rx is None
                else now - self.gate_last_rx
            )

            data["bridge_age_sec"] = (
                None
                if self.bridge_last_rx is None
                else now - self.bridge_last_rx
            )

            data["state_age_sec"] = (
                None
                if self.state_last_rx is None
                else now - self.state_last_rx
            )

            data["logger_age_sec"] = (
                None
                if self.logger_last_rx is None
                else now - self.logger_last_rx
            )

        status = self.local_status()

        mission_process_state = (
            self.mission_process_status()
        )

        data["mission_process_state"] = (
            mission_process_state
        )

        data["mission_process_running"] = (
            mission_process_state == "active"
        )

        data["bridge_alive"] = status["bridge_alive"]
        data["control_ready"] = status["control_ready"]
        data["bridge_command_active"] = (
            status["bridge_command_active"]
        )
        data["control_state"] = (
            self.control_state
            if status["online"]
            else "OFFLINE"
        )
        data["software_stop"] = (
            self.software_stop_state
        )
        data["autonomy_enabled"] = bool(
            status["mode"] == "GUIDED"
            and
            self.control_state
            in (
                "ENABLED",
                "AUTONOMY READY / DISARMED",
            )
        )
        data["can_enable"] = bool(
            status["online"]
            and status["bridge_alive"]
            and
            self.control_state
            not in (
                "ENABLED",
                "AUTONOMY READY / DISARMED",
            )
        )

        with self.client_lock:
            client = self.client_socket

        if client is None:
            return

        message = {
            "type": "telemetry",
            "data": data,
        }

        try:
            self.send_to_socket(
                client,
                message,
            )

        except OSError:
            with self.client_lock:
                if self.client_socket is client:
                    self.client_socket = None

    def send_to_socket(self, sock, message):
        payload = (
            json.dumps(
                message,
                separators=(",", ":"),
            )
            + "\n"
        ).encode("utf-8")

        with self.send_lock:
            sock.sendall(payload)


def main(args=None):
    rclpy.init(args=args)

    node = BoatDashboardBridge()

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
