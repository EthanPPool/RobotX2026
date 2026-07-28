#!/usr/bin/env python3

import json
import math
import socket
import threading
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from mavros_msgs.msg import State, SysStatus
from mavros_msgs.srv import CommandBool, SetMode
from rclpy.node import Node
from sensor_msgs.msg import BatteryState, Imu, NavSatFix
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger

from boat_interfaces.msg import DetectedObjectArray, Gate


class BoatDashboardBridge(Node):

    def __init__(self):
        super().__init__("boat_dashboard_bridge")

        self.lock = threading.RLock()
        self.client_lock = threading.RLock()
        self.send_lock = threading.RLock()
        self.action_lock = threading.RLock()
        self.client_socket = None

        self.software_stop_state = "UNKNOWN"
        self.control_state = "BOOT SAFE"

        # Local Jetson monotonic timestamps for ROS source
        # freshness. Only ages, never absolute monotonic times,
        # are sent across machines.
        self.state_last_rx = None
        self.battery_last_rx = None
        self.gate_last_rx = None
        self.bridge_last_rx = None

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

        # 5 Hz output to ground station
        self.create_timer(
            0.2,
            self.publish_telemetry,
        )

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

        siny_cosp = 2.0 * (
            q.w * q.z
            + q.x * q.y
        )

        cosy_cosp = 1.0 - 2.0 * (
            q.y * q.y
            + q.z * q.z
        )

        yaw = math.atan2(
            siny_cosp,
            cosy_cosp,
        )

        heading = (
            math.degrees(yaw)
            + 360.0
        ) % 360.0

        with self.lock:
            self.telemetry["heading_deg"] = heading

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
        with self.lock:
            self.telemetry["buoy_count"] = len(msg.objects)

    def gate_callback(self, msg):
        with self.lock:
            self.gate_last_rx = time.monotonic()

            self.telemetry["gate_confidence"] = float(
                msg.confidence
            )
            self.telemetry["gate_x"] = float(
                msg.center.x
            )
            self.telemetry["gate_y"] = float(
                msg.center.y
            )

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

            if not status["gate_fresh"]:
                return (
                    False,
                    "Enable rejected: no fresh "
                    "high-confidence gate",
                )

            if not status["control_ready"]:
                return (
                    False,
                    "Enable rejected: follower "
                    "is commanding STOP",
                )

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

    def execute_reset_mission(self):
        with self.action_lock:
            stop_ok, stop_msg = (
                self.execute_stop()
            )

            reset_ok, reset_msg = (
                self.call_reset_service()
            )

            if reset_ok:
                self.control_state = (
                    "MISSION RESET / STOPPED"
                )

            return (
                stop_ok and reset_ok,
                stop_msg
                + " | reset: "
                + reset_msg,
            )

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

            self.get_logger().info(
                "Ground station disconnected"
            )

    def handle_message(self, client, message):
        if not isinstance(message, dict):
            return

        if message.get("type") != "command":
            return

        command = str(
            message.get("command", "")
        ).strip().lower()

        handlers = {
            "stop": self.execute_stop,
            "clear_stop": self.execute_clear_stop,
            "enable": self.execute_enable,
            "arm": self.execute_arm,
            "disarm": self.execute_disarm,
            "reset_mission": self.execute_reset_mission,
        }

        handler = handlers.get(command)

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
                success, result_message = handler()

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

        try:
            self.send_to_socket(
                client,
                response,
            )
        except OSError:
            pass

    # ========================================================
    # TELEMETRY
    # ========================================================

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

        status = self.local_status()

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
