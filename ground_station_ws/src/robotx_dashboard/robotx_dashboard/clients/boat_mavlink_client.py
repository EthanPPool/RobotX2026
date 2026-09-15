#!/usr/bin/env python3

import math
import threading
import time

from pymavlink import mavutil

from robotx_dashboard.clients.base_client import BaseVehicleClient


class BoatMavlinkClient(BaseVehicleClient):
    """
    Receive-only direct MAVLink client for the BlueBoat autopilot.

    BlueOS/ArduPilot sends UDP MAVLink to Beeptop:14551.
    This client does not send vehicle commands. Jetson TCP remains
    the command/control transport.
    """

    def __init__(
        self,
        vehicle_id,
        state_update_callback,
        link_offline_callback=None,
        endpoint="udpin:0.0.0.0:14551",
        rx_timeout=3.0,
    ):
        super().__init__(
            vehicle_id,
            state_update_callback,
        )

        self.endpoint = endpoint
        self.rx_timeout = float(rx_timeout)

        self.link_offline_callback = (
            link_offline_callback
        )

        self.stop_event = threading.Event()
        self.thread = None
        self.connection = None

        self.target_system = None
        self.target_component = (
            mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1
        )
        self.last_rx = None

    def start(self):
        if (
            self.thread is not None
            and self.thread.is_alive()
        ):
            return

        self.stop_event.clear()

        self.thread = threading.Thread(
            target=self._run,
            name="boat-direct-mavlink",
            daemon=True,
        )

        self.thread.start()

    def stop(self):
        self.stop_event.set()

        connection = self.connection

        if connection is not None:
            try:
                connection.close()
            except Exception:
                pass

        if self.thread is not None:
            self.thread.join(timeout=2.0)

        self.connected = False

    def command(self, command, data=None):
        return {
            "success": False,
            "message": (
                "Direct USV MAVLink transport is "
                "receive-only. Commands use Jetson TCP."
            ),
        }

    def _run(self):
        while not self.stop_event.is_set():

            try:
                self.connection = (
                    mavutil.mavlink_connection(
                        self.endpoint,
                        dialect="ardupilotmega",
                    )
                )

                self._receive_loop()

            except Exception:
                self._mark_offline()

                if not self.stop_event.is_set():
                    self.stop_event.wait(1.0)

            finally:
                connection = self.connection
                self.connection = None

                if connection is not None:
                    try:
                        connection.close()
                    except Exception:
                        pass

    def _receive_loop(self):
        while not self.stop_event.is_set():

            message = self.connection.recv_match(
                blocking=True,
                timeout=0.5,
            )

            now = time.monotonic()

            if message is None:
                if (
                    self.last_rx is not None
                    and now - self.last_rx
                    > self.rx_timeout
                ):
                    self._mark_offline()

                continue

            message_type = message.get_type()

            if message_type == "BAD_DATA":
                continue

            system_id = int(
                message.get_srcSystem()
            )

            component_id = int(
                message.get_srcComponent()
            )

            # Only the primary ArduPilot autopilot component
            # is authoritative for USV state. Other MAVLink
            # components may share the same system ID.
            if (
                component_id
                != self.target_component
            ):
                continue

            if self.target_system is None:

                if message_type != "HEARTBEAT":
                    continue

                if (
                    int(message.autopilot)
                    != mavutil.mavlink
                    .MAV_AUTOPILOT_ARDUPILOTMEGA
                ):
                    continue

                self.target_system = system_id

            if system_id != self.target_system:
                continue

            self.last_rx = now
            self.connected = True

            self._handle_message(
                message_type,
                message,
            )

    def _mark_offline(self):
        was_connected = self.connected

        self.connected = False
        self.target_system = None
        self.last_rx = None

        if (
            was_connected
            and self.link_offline_callback
            is not None
        ):
            try:
                self.link_offline_callback(
                    self.vehicle_id,
                    "mavlink",
                )
            except Exception:
                pass

    def _handle_message(
        self,
        message_type,
        message,
    ):

        if message_type == "HEARTBEAT":

            base_mode = int(
                message.base_mode
            )

            armed = bool(
                base_mode
                & mavutil.mavlink
                .MAV_MODE_FLAG_SAFETY_ARMED
            )

            guided = bool(
                base_mode
                & mavutil.mavlink
                .MAV_MODE_FLAG_GUIDED_ENABLED
            )

            manual_input = bool(
                base_mode
                & mavutil.mavlink
                .MAV_MODE_FLAG_MANUAL_INPUT_ENABLED
            )

            mode = mavutil.mode_string_v10(
                message
            )

            self.update_state(
                connected=True,
                armed=armed,
                guided=guided,
                manual_input=manual_input,
                mode=str(mode),
                system_status=int(
                    message.system_status
                ),
            )

            return

        if message_type == "GLOBAL_POSITION_INT":

            fields = {
                "latitude":
                    float(message.lat) / 1.0e7,
                "longitude":
                    float(message.lon) / 1.0e7,
                "altitude_msl":
                    float(message.alt) / 1000.0,
                "relative_altitude":
                    float(message.relative_alt)
                    / 1000.0,
                "velocity_north":
                    float(message.vx) / 100.0,
                "velocity_east":
                    float(message.vy) / 100.0,
                "velocity_up":
                    -float(message.vz) / 100.0,
            }

            fields["altitude"] = (
                fields["altitude_msl"]
            )

            fields["ground_speed"] = math.hypot(
                fields["velocity_north"],
                fields["velocity_east"],
            )

            if int(message.hdg) != 65535:
                fields["heading_deg"] = (
                    float(message.hdg) / 100.0
                )

            self.update_state(**fields)
            return

        if message_type == "GPS_RAW_INT":

            # Only use GPS_RAW_INT position when the receiver
            # reports at least a 2D fix.
            if int(message.fix_type) >= 2:
                self.update_state(
                    latitude=(
                        float(message.lat)
                        / 1.0e7
                    ),
                    longitude=(
                        float(message.lon)
                        / 1.0e7
                    ),
                    altitude_msl=(
                        float(message.alt)
                        / 1000.0
                    ),
                    altitude=(
                        float(message.alt)
                        / 1000.0
                    ),
                )

            return

        if message_type == "ATTITUDE":

            roll_deg = math.degrees(
                float(message.roll)
            )

            pitch_deg = math.degrees(
                float(message.pitch)
            )

            yaw_deg = math.degrees(
                float(message.yaw)
            )

            heading_deg = (
                yaw_deg + 360.0
            ) % 360.0

            self.update_state(
                roll_deg=roll_deg,
                pitch_deg=pitch_deg,
                yaw_deg=yaw_deg,
                heading_deg=heading_deg,
            )

            return

        if message_type == "VFR_HUD":

            fields = {
                "ground_speed":
                    float(message.groundspeed),
            }

            heading = int(message.heading)

            if heading >= 0:
                fields["heading_deg"] = float(
                    heading % 360
                )

            self.update_state(**fields)
            return

        if message_type == "SYS_STATUS":

            fields = {}

            voltage_mv = int(
                message.voltage_battery
            )

            current_ca = int(
                message.current_battery
            )

            remaining = int(
                message.battery_remaining
            )

            if voltage_mv != 65535:
                fields["voltage"] = (
                    voltage_mv / 1000.0
                )

            if current_ca != -1:
                fields["battery_current"] = (
                    current_ca / 100.0
                )

            if remaining >= 0:
                fields["battery_percent"] = (
                    remaining / 100.0
                )
                fields["battery_remaining"] = (
                    remaining
                )

            if fields:
                self.update_state(**fields)

            return
