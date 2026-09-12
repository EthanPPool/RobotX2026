#!/usr/bin/env python3

import json
import math
import socket
import threading
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from mavros_msgs.msg import State
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import BatteryState, Imu, NavSatFix
from std_msgs.msg import Float64

from uav_interfaces.msg import AutonomyStatus, SafetyStatus


class UavDashboardBridge(Node):

    def __init__(self):
        super().__init__("uav_dashboard_bridge")

        self.lock = threading.RLock()
        self.client_lock = threading.RLock()
        self.send_lock = threading.RLock()

        self.client_socket = None

        self.state_last_rx = None
        self.gps_last_rx = None
        self.imu_last_rx = None
        self.battery_last_rx = None
        self.velocity_last_rx = None
        self.safety_last_rx = None
        self.autonomy_last_rx = None

        self.telemetry = {
            "connected": False,
            "armed": False,
            "guided": False,
            "manual_input": False,
            "mode": "UNKNOWN",
            "system_status": None,

            "latitude": None,
            "longitude": None,
            "altitude_msl": None,
            "relative_altitude": None,
            "gps_sigma_m": None,

            "roll_deg": None,
            "pitch_deg": None,
            "yaw_deg": None,
            "heading_deg": None,

            "ground_speed": None,
            "vertical_speed": None,
            "velocity_north": None,
            "velocity_east": None,
            "velocity_up": None,

            "voltage": None,
            "current": None,
            "battery_percent": None,

            "safety_state": "UNKNOWN",
            "safety_reason": None,
            "prearm_ready": False,
            "flight_ready": False,
            "failsafe_latched": False,
            "gps_valid": False,
            "local_position_valid": False,

            "autonomy_enabled": False,
            "command_fresh": False,
            "authorized": False,
            "autonomy_reason": None,
        }

        # ----------------------------------------------------
        # MAVROS telemetry
        # ----------------------------------------------------

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
            qos_profile_sensor_data,
        )

        self.create_subscription(
            Float64,
            "/mavros/global_position/rel_alt",
            self.relative_altitude_callback,
            qos_profile_sensor_data,
        )

        self.create_subscription(
            Float64,
            "/mavros/global_position/compass_hdg",
            self.heading_callback,
            qos_profile_sensor_data,
        )

        self.create_subscription(
            Imu,
            "/mavros/imu/data",
            self.imu_callback,
            qos_profile_sensor_data,
        )

        self.create_subscription(
            BatteryState,
            "/mavros/battery",
            self.battery_callback,
            qos_profile_sensor_data,
        )

        self.create_subscription(
            TwistStamped,
            "/mavros/local_position/velocity_local",
            self.velocity_callback,
            qos_profile_sensor_data,
        )

        # ----------------------------------------------------
        # UAV autonomy/safety telemetry
        # ----------------------------------------------------

        self.create_subscription(
            SafetyStatus,
            "/vehicle/safety_status",
            self.safety_callback,
            10,
        )

        self.create_subscription(
            AutonomyStatus,
            "/vehicle/autonomy_status",
            self.autonomy_callback,
            10,
        )

        # 5 Hz telemetry stream to Beeptop.
        self.create_timer(
            0.2,
            self.publish_telemetry,
        )

        self.server_thread = threading.Thread(
            target=self.server_loop,
            daemon=True,
            name="uav-dashboard-tcp-server",
        )
        self.server_thread.start()

        self.get_logger().info(
            "UAV dashboard bridge listening on TCP 0.0.0.0:8766"
        )

    # ========================================================
    # ROS CALLBACKS
    # ========================================================

    def state_callback(self, msg):
        with self.lock:
            self.state_last_rx = time.monotonic()

            self.telemetry["connected"] = bool(msg.connected)
            self.telemetry["armed"] = bool(msg.armed)
            self.telemetry["guided"] = bool(msg.guided)
            self.telemetry["manual_input"] = bool(msg.manual_input)
            self.telemetry["mode"] = (
                str(msg.mode) if msg.mode else "UNKNOWN"
            )
            self.telemetry["system_status"] = int(msg.system_status)

    def gps_callback(self, msg):
        with self.lock:
            self.gps_last_rx = time.monotonic()

            if math.isfinite(msg.latitude):
                self.telemetry["latitude"] = float(msg.latitude)

            if math.isfinite(msg.longitude):
                self.telemetry["longitude"] = float(msg.longitude)

            if math.isfinite(msg.altitude):
                self.telemetry["altitude_msl"] = float(msg.altitude)

            try:
                covariance = msg.position_covariance

                if (
                    len(covariance) >= 5
                    and covariance[0] >= 0.0
                    and covariance[4] >= 0.0
                ):
                    sigma = math.sqrt(
                        max(
                            float(covariance[0]),
                            float(covariance[4]),
                        )
                    )

                    if math.isfinite(sigma):
                        self.telemetry["gps_sigma_m"] = sigma

            except Exception:
                pass

    def relative_altitude_callback(self, msg):
        if math.isfinite(msg.data):
            with self.lock:
                self.telemetry["relative_altitude"] = float(msg.data)

    def heading_callback(self, msg):
        if math.isfinite(msg.data):
            with self.lock:
                self.telemetry["heading_deg"] = (
                    float(msg.data) % 360.0
                )

    def imu_callback(self, msg):
        q = msg.orientation

        # Quaternion -> Euler
        sinr_cosp = 2.0 * (
            q.w * q.x
            + q.y * q.z
        )

        cosr_cosp = 1.0 - 2.0 * (
            q.x * q.x
            + q.y * q.y
        )

        roll = math.atan2(
            sinr_cosp,
            cosr_cosp,
        )

        sinp = 2.0 * (
            q.w * q.y
            - q.z * q.x
        )

        if abs(sinp) >= 1.0:
            pitch = math.copysign(
                math.pi / 2.0,
                sinp,
            )
        else:
            pitch = math.asin(sinp)

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

        with self.lock:
            self.imu_last_rx = time.monotonic()

            self.telemetry["roll_deg"] = math.degrees(roll)
            self.telemetry["pitch_deg"] = math.degrees(pitch)
            self.telemetry["yaw_deg"] = (
                math.degrees(yaw) + 360.0
            ) % 360.0

            # Compass heading is preferred when available.
            if self.telemetry["heading_deg"] is None:
                self.telemetry["heading_deg"] = (
                    math.degrees(yaw) + 360.0
                ) % 360.0

    def velocity_callback(self, msg):
        vx = float(msg.twist.linear.x)
        vy = float(msg.twist.linear.y)
        vz = float(msg.twist.linear.z)

        with self.lock:
            self.velocity_last_rx = time.monotonic()

            self.telemetry["velocity_east"] = vx
            self.telemetry["velocity_north"] = vy
            self.telemetry["velocity_up"] = vz

            self.telemetry["ground_speed"] = math.hypot(
                vx,
                vy,
            )

            self.telemetry["vertical_speed"] = vz

    def battery_callback(self, msg):
        with self.lock:
            self.battery_last_rx = time.monotonic()

            if math.isfinite(msg.voltage):
                self.telemetry["voltage"] = float(msg.voltage)

            if math.isfinite(msg.current):
                self.telemetry["current"] = float(msg.current)

            if (
                math.isfinite(msg.percentage)
                and msg.percentage >= 0.0
            ):
                self.telemetry["battery_percent"] = (
                    float(msg.percentage) * 100.0
                )

    def safety_callback(self, msg):
        with self.lock:
            self.safety_last_rx = time.monotonic()

            self.telemetry["safety_state"] = str(msg.state_label)
            self.telemetry["safety_reason"] = str(msg.reason)

            self.telemetry["prearm_ready"] = bool(msg.prearm_ready)
            self.telemetry["flight_ready"] = bool(msg.flight_ready)
            self.telemetry["failsafe_latched"] = bool(
                msg.failsafe_latched
            )

            self.telemetry["gps_valid"] = bool(msg.gps_valid)
            self.telemetry["local_position_valid"] = bool(
                msg.local_position_valid
            )

    def autonomy_callback(self, msg):
        with self.lock:
            self.autonomy_last_rx = time.monotonic()

            self.telemetry["autonomy_enabled"] = bool(
                msg.autonomy_enabled
            )

            self.telemetry["command_fresh"] = bool(
                msg.command_fresh
            )

            self.telemetry["authorized"] = bool(
                msg.authorized
            )

            self.telemetry["autonomy_reason"] = str(msg.reason)

    # ========================================================
    # TELEMETRY
    # ========================================================

    @staticmethod
    def age(now, stamp):
        if stamp is None:
            return None

        return max(
            0.0,
            now - stamp,
        )

    def telemetry_snapshot(self):
        now = time.monotonic()

        with self.lock:
            data = dict(self.telemetry)

            data["state_age_sec"] = self.age(
                now,
                self.state_last_rx,
            )

            data["gps_age_sec"] = self.age(
                now,
                self.gps_last_rx,
            )

            data["imu_age_sec"] = self.age(
                now,
                self.imu_last_rx,
            )

            data["battery_age_sec"] = self.age(
                now,
                self.battery_last_rx,
            )

            data["velocity_age_sec"] = self.age(
                now,
                self.velocity_last_rx,
            )

            data["safety_age_sec"] = self.age(
                now,
                self.safety_last_rx,
            )

            data["autonomy_age_sec"] = self.age(
                now,
                self.autonomy_last_rx,
            )

        return data

    def publish_telemetry(self):
        payload = {
            "type": "telemetry",
            "vehicle": "uav",
            "data": self.telemetry_snapshot(),
        }

        self.send_json(payload)

    # ========================================================
    # TCP
    # ========================================================

    def send_json(self, payload):
        encoded = (
            json.dumps(
                payload,
                separators=(",", ":"),
            )
            + "\n"
        ).encode("utf-8")

        with self.client_lock:
            client = self.client_socket

        if client is None:
            return

        try:
            with self.send_lock:
                client.sendall(encoded)

        except OSError:
            self.drop_client(client)

    def drop_client(self, client=None):
        with self.client_lock:
            current = self.client_socket

            if current is None:
                return

            if (
                client is not None
                and current is not client
            ):
                return

            self.client_socket = None

        try:
            current.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass

        try:
            current.close()
        except OSError:
            pass

        self.get_logger().warn(
            "UAV ground-station TCP client disconnected"
        )

    def client_loop(self, client):
        buffer = b""

        try:
            while rclpy.ok():

                chunk = client.recv(4096)

                if not chunk:
                    break

                buffer += chunk

                while b"\n" in buffer:
                    line, buffer = buffer.split(
                        b"\n",
                        1,
                    )

                    if not line.strip():
                        continue

                    # Telemetry-only phase.
                    # Parse valid JSON so malformed clients can
                    # be diagnosed, but execute no commands.
                    try:
                        message = json.loads(
                            line.decode("utf-8")
                        )

                        if message.get("type") == "command":
                            self.send_json({
                                "type": "response",
                                "request_id": message.get(
                                    "request_id"
                                ),
                                "success": False,
                                "message": (
                                    "UAV bridge is telemetry-only; "
                                    "remote flight commands disabled"
                                ),
                            })

                    except Exception as exc:
                        self.get_logger().warn(
                            f"Invalid TCP message: {exc}"
                        )

        except OSError:
            pass

        finally:
            self.drop_client(client)

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
            ("0.0.0.0", 8766)
        )

        server.listen(2)
        server.settimeout(1.0)

        while rclpy.ok():

            try:
                client, address = server.accept()

            except socket.timeout:
                continue

            except OSError:
                break

            client.setsockopt(
                socket.IPPROTO_TCP,
                socket.TCP_NODELAY,
                1,
            )

            with self.client_lock:
                old_client = self.client_socket
                self.client_socket = client

            if old_client is not None:
                try:
                    old_client.close()
                except OSError:
                    pass

            self.get_logger().info(
                "UAV ground station connected from "
                f"{address[0]}:{address[1]}"
            )

            thread = threading.Thread(
                target=self.client_loop,
                args=(client,),
                daemon=True,
            )
            thread.start()


def main(args=None):
    rclpy.init(args=args)

    node = UavDashboardBridge()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.drop_client()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
