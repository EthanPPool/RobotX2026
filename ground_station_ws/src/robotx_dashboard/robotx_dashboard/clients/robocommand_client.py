#!/usr/bin/env python3

import json
import math
import os
import threading
import time
from collections import deque

import paho.mqtt.client as mqtt

from google.protobuf.json_format import MessageToDict
from google.protobuf.timestamp_pb2 import Timestamp

import common_pb2

from robotx import (
    rx_commands_pb2,
    rx_common_pb2,
    rx_course_pb2,
    rx_reports_pb2,
    rx_requests_pb2,
)


TOPIC_ROOT = "robocommand/robotx"


class RoboCommandClient:

    HEARTBEAT_PERIOD = 0.5

    def __init__(
        self,
        vehicle_snapshot,
        host=None,
        port=None,
        team_id=None,
        uav_geofence_snapshot=None,
    ):
        self.vehicle_snapshot = vehicle_snapshot
        self.uav_geofence_snapshot = (
            uav_geofence_snapshot
        )

        self.host = (
            host
            or os.environ.get(
                "ROBOCOMMAND_BROKER",
                "127.0.0.1",
            )
        )

        self.port = int(
            port
            or os.environ.get(
                "ROBOCOMMAND_PORT",
                "1883",
            )
        )

        self.team_id = (
            team_id
            or os.environ.get(
                "ROBOCOMMAND_TEAM_ID",
                "ULLY",
            )
        )

        self.lock = threading.RLock()
        self.stop_event = threading.Event()

        self.connected = False
        self.started = False

        self.course_id = None
        self.pinger_freq_hz = None
        self.course_corners = []

        self.run_state = "WAITING"
        self.run_id = None
        self.declaration_seq = None
        self.last_command = None

        self.last_rx = None
        self.last_tx = None

        self.request_seq = 0
        self.report_seq = {
            "USV1": 0,
            "UAV1": 0,
        }

        self.vehicle_reports = {
            "USV1": {
                "state": "UNKNOWN",
                "last_tx": None,
            },
            "UAV1": {
                "state": "UNKNOWN",
                "last_tx": None,
            },
        }

        self.rx_history = deque(maxlen=100)
        self.tx_history = deque(maxlen=100)

        self.client = mqtt.Client(
            mqtt.CallbackAPIVersion.VERSION2,
            client_id=f"{self.team_id}-Beeptop-OCS",
        )

        self.client.on_connect = self._on_connect
        self.client.on_disconnect = self._on_disconnect
        self.client.on_message = self._on_message

        self.client.reconnect_delay_set(
            min_delay=1,
            max_delay=5,
        )

        self.heartbeat_thread = None

    # ========================================================
    # Lifecycle
    # ========================================================

    def start(self):
        if self.started:
            return

        self.started = True
        self.stop_event.clear()

        self.client.connect_async(
            self.host,
            self.port,
            keepalive=30,
        )

        self.client.loop_start()

        self.heartbeat_thread = threading.Thread(
            target=self._heartbeat_loop,
            name="robocommand-heartbeats",
            daemon=True,
        )

        self.heartbeat_thread.start()

    def stop(self):
        self.stop_event.set()

        if self.heartbeat_thread is not None:
            self.heartbeat_thread.join(
                timeout=2.0
            )

        try:
            self.client.disconnect()
        except Exception:
            pass

        try:
            self.client.loop_stop()
        except Exception:
            pass

        with self.lock:
            self.connected = False

    def reconnect(self):
        try:
            self.client.disconnect()
        except Exception:
            pass

        with self.lock:
            self.connected = False

        try:
            self.client.connect_async(
                self.host,
                self.port,
                keepalive=30,
            )

            return {
                "success": True,
                "message": (
                    "RoboCommand reconnect requested."
                ),
            }

        except Exception as exc:
            return {
                "success": False,
                "message": str(exc),
            }

    # ========================================================
    # MQTT
    # ========================================================

    def _on_connect(
        self,
        client,
        userdata,
        flags,
        reason_code,
        properties,
    ):
        if getattr(
            reason_code,
            "is_failure",
            False,
        ):
            with self.lock:
                self.connected = False
            return

        with self.lock:
            self.connected = True

        client.subscribe(
            f"{TOPIC_ROOT}/course"
        )

        client.subscribe(
            f"{TOPIC_ROOT}/"
            f"{self.team_id}/command"
        )

    def _on_disconnect(
        self,
        client,
        userdata,
        disconnect_flags,
        reason_code,
        properties,
    ):
        with self.lock:
            self.connected = False

    def _on_message(
        self,
        client,
        userdata,
        msg,
    ):
        now = time.monotonic()

        try:
            if msg.topic == f"{TOPIC_ROOT}/course":
                course = rx_course_pb2.RxCourse()
                course.ParseFromString(msg.payload)

                corners = [
                    {
                        "latitude": float(p.latitude),
                        "longitude": float(p.longitude),
                    }
                    for p in course.corners
                ]

                with self.lock:
                    self.course_id = course.course_id
                    self.pinger_freq_hz = int(
                        course.pinger_freq_hz
                    )
                    self.course_corners = corners
                    self.last_rx = now

                self._record(
                    "rx",
                    "RxCourse",
                    msg.topic,
                    (
                        f"course={course.course_id}, "
                        f"pinger={course.pinger_freq_hz} Hz"
                    ),
                    course,
                )

                return

            if (
                msg.topic
                == f"{TOPIC_ROOT}/"
                   f"{self.team_id}/command"
            ):
                command = (
                    rx_commands_pb2.RxCommand()
                )

                command.ParseFromString(
                    msg.payload
                )

                body = (
                    command.WhichOneof("body")
                    or "unknown"
                )

                summary = body

                with self.lock:
                    self.last_rx = now
                    self.last_command = body

                    if body == "run_start":
                        self.run_state = "STARTED"
                        self.run_id = int(
                            command.run_start.run_id
                        )

                        summary = (
                            "RunStart "
                            f"run_id={self.run_id}, "
                            "declaration_seq="
                            f"{command.run_start.declaration_seq}"
                        )

                self._record(
                    "rx",
                    "RxCommand",
                    msg.topic,
                    summary,
                    command,
                )

        except Exception as exc:
            self._record_text(
                "rx",
                "DECODE_ERROR",
                msg.topic,
                str(exc),
            )

    # ========================================================
    # Logging / status
    # ========================================================

    @staticmethod
    def _message_dict(message):
        try:
            return MessageToDict(
                message,
                preserving_proto_field_name=True,
            )
        except Exception:
            return {
                "text": str(message)
            }

    def _record(
        self,
        direction,
        message_type,
        topic,
        summary,
        message,
        vehicle=None,
    ):
        entry = {
            "time": time.strftime("%H:%M:%S"),
            "type": message_type,
            "topic": topic,
            "summary": summary,
            "vehicle": vehicle,
            "detail": self._message_dict(
                message
            ),
        }

        with self.lock:
            if direction == "rx":
                self.rx_history.append(entry)
            else:
                self.tx_history.append(entry)

    def _record_text(
        self,
        direction,
        message_type,
        topic,
        summary,
    ):
        entry = {
            "time": time.strftime("%H:%M:%S"),
            "type": message_type,
            "topic": topic,
            "summary": summary,
            "vehicle": None,
            "detail": {
                "message": summary,
            },
        }

        with self.lock:
            if direction == "rx":
                self.rx_history.append(entry)
            else:
                self.tx_history.append(entry)

    @staticmethod
    def _age(now, stamp):
        if stamp is None:
            return None

        return max(
            0.0,
            now - stamp,
        )

    def snapshot(self):
        now = time.monotonic()

        with self.lock:
            return {
                "connected": bool(
                    self.connected
                ),
                "broker": (
                    f"{self.host}:{self.port}"
                ),
                "team_id": self.team_id,

                "course_id": self.course_id,
                "pinger_freq_hz":
                    self.pinger_freq_hz,
                "course_corners":
                    list(self.course_corners),

                "run_state": self.run_state,
                "run_id": self.run_id,
                "declaration_seq":
                    self.declaration_seq,
                "last_command":
                    self.last_command,

                "last_rx_age_sec":
                    self._age(
                        now,
                        self.last_rx,
                    ),

                "last_tx_age_sec":
                    self._age(
                        now,
                        self.last_tx,
                    ),

                "vehicle_reports": {
                    vehicle_id: {
                        "state":
                            report["state"],
                        "last_tx_age_sec":
                            self._age(
                                now,
                                report["last_tx"],
                            ),
                    }
                    for (
                        vehicle_id,
                        report,
                    ) in self.vehicle_reports.items()
                },

                "rx_history":
                    list(
                        reversed(
                            self.rx_history
                        )
                    ),

                "tx_history":
                    list(
                        reversed(
                            self.tx_history
                        )
                    ),
            }

    def clear_history(self):
        with self.lock:
            self.rx_history.clear()
            self.tx_history.clear()

        return {
            "success": True,
            "message": "RoboCommand display log cleared.",
        }

    # ========================================================
    # Publish helpers
    # ========================================================

    def _publish(
        self,
        topic,
        message,
        message_type,
        summary,
        vehicle=None,
    ):
        with self.lock:
            if not self.connected:
                return False

        info = self.client.publish(
            topic,
            message.SerializeToString(),
            qos=0,
            retain=False,
        )

        if info.rc != mqtt.MQTT_ERR_SUCCESS:
            return False

        now = time.monotonic()

        with self.lock:
            self.last_tx = now

            if (
                vehicle is not None
                and vehicle
                in self.vehicle_reports
            ):
                self.vehicle_reports[
                    vehicle
                ]["last_tx"] = now

        self._record(
            "tx",
            message_type,
            topic,
            summary,
            message,
            vehicle=vehicle,
        )

        return True

    @staticmethod
    def _timestamp():
        stamp = Timestamp()
        stamp.GetCurrentTime()
        return stamp

    @staticmethod
    def _finite(value, default=0.0):
        try:
            value = float(value)

            if math.isfinite(value):
                return value

        except (
            TypeError,
            ValueError,
        ):
            pass

        return float(default)

    # ========================================================
    # Heartbeats
    # ========================================================

    def _heartbeat_loop(self):
        while not self.stop_event.is_set():

            try:
                self._send_vehicle_heartbeats()

            except Exception as exc:
                self._record_text(
                    "tx",
                    "HEARTBEAT_ERROR",
                    "",
                    str(exc),
                )

            self.stop_event.wait(
                self.HEARTBEAT_PERIOD
            )

    @staticmethod
    def _robot_state(
        vehicle_type,
        vehicle,
    ):
        if not vehicle.get(
            "online",
            False,
        ):
            return (
                common_pb2.STATE_UNKNOWN,
                "UNKNOWN",
            )

        if (
            vehicle_type == "USV"
            and str(
                vehicle.get(
                    "software_stop",
                    "",
                )
            ).upper() == "ENGAGED"
        ):
            return (
                common_pb2.STATE_KILLED,
                "KILLED",
            )

        if vehicle.get(
            "autonomy_enabled",
            False,
        ):
            return (
                common_pb2.STATE_AUTO,
                "AUTO",
            )

        return (
            common_pb2.STATE_MANUAL,
            "MANUAL",
        )

    def _send_vehicle_heartbeats(self):

        with self.lock:
            if not self.connected:
                return

        vehicles = self.vehicle_snapshot()

        definitions = (
            (
                "USV1",
                "boat",
                "USV",
                rx_common_pb2.TYPE_USV,
            ),
            (
                "UAV1",
                "uav",
                "UAV",
                rx_common_pb2.TYPE_UAV,
            ),
        )

        for (
            vehicle_id,
            local_id,
            vehicle_type,
            proto_type,
        ) in definitions:

            vehicle = vehicles.get(
                local_id,
                {},
            )

            state, state_name = (
                self._robot_state(
                    vehicle_type,
                    vehicle,
                )
            )

            self.report_seq[
                vehicle_id
            ] += 1

            lat = self._finite(
                vehicle.get("latitude")
            )

            lng = self._finite(
                vehicle.get("longitude")
            )

            heading = self._finite(
                vehicle.get("heading_deg")
            )

            speed = vehicle.get(
                "ground_speed"
            )

            if speed is None:
                speed = abs(
                    self._finite(
                        vehicle.get(
                            "bridge_forward"
                        )
                    )
                )

            speed = self._finite(speed)

            report = rx_reports_pb2.RxReport(
                team_id=self.team_id,
                vehicle_id=vehicle_id,
                seq=self.report_seq[
                    vehicle_id
                ],
                sent_at=self._timestamp(),
                heartbeat=(
                    rx_reports_pb2.Heartbeat(
                        state=state,
                        position=(
                            common_pb2.LatLng(
                                latitude=lat,
                                longitude=lng,
                            )
                        ),
                        spd_mps=speed,
                        heading_deg=heading,
                        vehicle_type=proto_type,
                    )
                ),
            )

            topic = (
                f"{TOPIC_ROOT}/"
                f"{self.team_id}/"
                f"{vehicle_id}/report"
            )

            if self._publish(
                topic,
                report,
                "RxReport/Heartbeat",
                (
                    f"{vehicle_id} "
                    f"{state_name} "
                    f"lat={lat:.6f} "
                    f"lon={lng:.6f} "
                    f"spd={speed:.2f} "
                    f"hdg={heading:.1f}"
                ),
                vehicle=vehicle_id,
            ):
                with self.lock:
                    self.vehicle_reports[
                        vehicle_id
                    ]["state"] = state_name

    # ========================================================
    # Run Declaration
    # ========================================================

    def send_run_declaration(self):

        with self.lock:
            if not self.connected:
                return {
                    "success": False,
                    "message": (
                        "RoboCommand is disconnected."
                    ),
                }

            corners = list(
                self.course_corners
            )

        if len(corners) < 4:
            return {
                "success": False,
                "message": (
                    "No valid RoboCommand course "
                    "boundary has been received yet."
                ),
            }


        if self.uav_geofence_snapshot is None:
            return {
                "success": False,
                "message": (
                    "Native UAV geofence source "
                    "is unavailable."
                ),
            }


        fence = (
            self.uav_geofence_snapshot()
        )


        if not fence.get(
            "complete",
            False,
        ):
            return {
                "success": False,
                "message": (
                    "Native UAV geofence has not "
                    "been downloaded completely."
                ),
            }


        zones = list(
            fence.get(
                "zones",
                []
            )
        )


        inclusion_polygons = [
            zone
            for zone in zones
            if zone.get("type")
            == "inclusion_polygon"
        ]


        if inclusion_polygons:

            vertices = list(
                inclusion_polygons[
                    0
                ].get(
                    "points",
                    []
                )
            )

        else:

            # Compatibility fallback for an older
            # cached fence representation.
            vertices = list(
                fence.get(
                    "inclusion_vertices",
                    []
                )
            )


        if len(vertices) < 3:
            return {
                "success": False,
                "message": (
                    "Native UAV geofence does not "
                    "contain a valid inclusion polygon."
                ),
            }


        # RobotX RunDeclaration uses a closed polygon.
        if (
            vertices[0]["latitude"]
            != vertices[-1]["latitude"]
            or
            vertices[0]["longitude"]
            != vertices[-1]["longitude"]
        ):
            vertices.append(
                dict(vertices[0])
            )


        geofence = [
            common_pb2.LatLng(
                latitude=float(
                    p["latitude"]
                ),
                longitude=float(
                    p["longitude"]
                ),
            )
            for p in vertices
        ]

        with self.lock:
            self.request_seq += 1
            seq = self.request_seq

        declaration = (
            rx_requests_pb2.RunDeclaration(
                vehicle_ids=[
                    "USV1",
                    "UAV1",
                ],
                task1_tier=(
                    common_pb2.TIER_NONE
                ),
                task2_tier=(
                    common_pb2.TIER_NONE
                ),
                task3_tier=(
                    common_pb2.TIER_NONE
                ),
                task4_tier=(
                    common_pb2.TIER_NONE
                ),
                uav_geofence=geofence,
            )
        )

        request = rx_requests_pb2.RxRequest(
            team_id=self.team_id,
            seq=seq,
            sent_at=self._timestamp(),
            run_declaration=declaration,
        )

        topic = (
            f"{TOPIC_ROOT}/"
            f"{self.team_id}/request"
        )

        success = self._publish(
            topic,
            request,
            "RxRequest/RunDeclaration",
            (
                "USV1,UAV1 "
                f"declaration_seq={seq}"
            ),
            vehicle="TEAM",
        )

        if not success:
            return {
                "success": False,
                "message": (
                    "Failed to publish "
                    "RunDeclaration."
                ),
            }

        with self.lock:
            self.declaration_seq = seq
            self.run_state = "DECLARED"
            self.run_id = None

        return {
            "success": True,
            "message": (
                "RunDeclaration published "
                f"for USV1 and UAV1 (seq={seq})."
            ),
            "declaration_seq": seq,
        }
