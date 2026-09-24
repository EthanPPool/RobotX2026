#!/usr/bin/env python3

import copy
import threading
import time


class VehicleManager:

    def __init__(self, vehicle_specs, lock=None):

        self.lock = (
            lock
            if lock is not None
            else threading.RLock()
        )

        self.clients = {}
        self.vehicles = {}

        for vehicle_id, spec in vehicle_specs.items():

            self.vehicles[vehicle_id] = {
                "id": vehicle_id,
                "name": spec["name"],
                "type": spec["type"],

                # Overall ground-station <-> vehicle transport.
                # Kept for compatibility with the existing UI/API.
                "vehicle_link": False,
                "vehicle_link_last_rx": None,

                # Source-specific USV links.
                #
                # jetson_link:
                #   TCP dashboard/control bridge on the Jetson.
                #
                # mavlink_link:
                #   Direct BlueOS/ArduPilot MAVLink stream.
                "jetson_link": False,
                "jetson_link_last_rx": None,
                "mavlink_link": False,
                "mavlink_link_last_rx": None,

                # Autopilot / MAVROS state
                "connected": False,
                "armed": False,
                "mode": "UNKNOWN",

                "latitude": None,
                "longitude": None,
                "altitude": None,
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

                "guided": False,
                "manual_input": False,
                "system_status": None,

                "voltage": None,
                "battery_percent": None,

                "battery_current": None,
                "battery_remaining": None,
                "battery_last_rx": None,

                # Direct ArduPilot actuator telemetry.
                "servo_outputs": {},
                "servo_config": {},
                "servo_output_last_rx": None,

                "buoy_count": 0,

                "visualization": {
                    "version": 1,
                    "cloud_points": [],
                    "buoys": [],
                    "gate": None,
                    "local_pose": None,
                    "trajectory": [],
                    "attitude": {},
                },

                "gate_confidence": None,
                "gate_x": None,
                "gate_y": None,
                "gate_last_rx": None,

                "control_forward": 0.0,
                "control_yaw": 0.0,

                "mission_state": None,

                # Vehicle-local mission/autonomy process.
                # This is independent of autopilot mode,
                # arming and autonomy authorization.
                "mission_process_state": "unknown",
                "mission_process_running": False,

                "bridge_forward": 0.0,
                "bridge_yaw": 0.0,
                "bridge_last_rx": None,

                # Jetson-local mission diagnostic logger.
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
                "logger_last_rx": None,

                # Jetson-side control/safety authority.
                "bridge_alive": False,
                "control_ready": False,
                "bridge_command_active": False,
                "control_state": "BOOT SAFE",
                "software_stop": "UNKNOWN",
                "autonomy_enabled": False,
                "can_enable": False,

                # UAV safety/autonomy telemetry.
                "safety_state": "UNKNOWN",
                "safety_reason": None,
                "prearm_ready": False,
                "flight_ready": False,
                "failsafe_latched": False,

                "mavros_state_fresh": False,
                "mavros_connected": False,
                "mode_allowed": False,
                "gps_valid": False,
                "local_position_valid": False,
                "battery_valid": False,
                "safety_battery_percentage": None,
                "mission_healthy": False,
                "autonomy_status_fresh": False,

                "command_fresh": False,
                "authorized": False,
                "autonomy_reason": None,

                "last_rx": None,
            }

    def register_client(
        self,
        vehicle_id,
        client,
    ):

        with self.lock:

            if vehicle_id not in self.vehicles:
                raise KeyError(
                    f"Unknown vehicle: {vehicle_id}"
                )

            self.clients[vehicle_id] = client

    def get_client(self, vehicle_id):

        with self.lock:
            return self.clients.get(vehicle_id)

    def update_vehicle(
        self,
        vehicle_id,
        fields,
    ):
        self._update_vehicle_source(
            vehicle_id,
            fields,
            source="vehicle",
        )

    def update_jetson_vehicle(
        self,
        vehicle_id,
        fields,
    ):
        self._update_vehicle_source(
            vehicle_id,
            fields,
            source="jetson",
        )

    def update_mavlink_vehicle(
        self,
        vehicle_id,
        fields,
    ):
        self._update_vehicle_source(
            vehicle_id,
            fields,
            source="mavlink",
        )

    def _update_vehicle_source(
        self,
        vehicle_id,
        fields,
        source,
    ):

        with self.lock:

            if vehicle_id not in self.vehicles:
                raise KeyError(
                    f"Unknown vehicle: {vehicle_id}"
                )

            state = self.vehicles[vehicle_id]
            now = time.monotonic()

            # ------------------------------------------------
            # Normal telemetry fields
            # ------------------------------------------------

            # Direct MAVLink owns the USV autopilot state.
            # Jetson TCP owns mission/control/perception/safety
            # state and must not overwrite these fields.
            mavlink_owned_fields = {
                "connected",
                "armed",
                "mode",
                "latitude",
                "longitude",
                "altitude",
                "altitude_msl",
                "relative_altitude",
                "roll_deg",
                "pitch_deg",
                "yaw_deg",
                "heading_deg",
                "ground_speed",
                "vertical_speed",
                "velocity_north",
                "velocity_east",
                "velocity_up",
                "guided",
                "manual_input",
                "system_status",
                "voltage",
                "battery_percent",
                "battery_current",
                "battery_remaining",
                "servo_outputs",
                "servo_config",
            }

            for key, value in fields.items():

                if (
                    vehicle_id == "boat"
                    and source == "jetson"
                    and key in mavlink_owned_fields
                ):
                    continue

                if key in (
                    "battery_age_sec",
                    "gate_age_sec",
                    "bridge_age_sec",
                    "state_age_sec",
                    "gps_age_sec",
                    "imu_age_sec",
                    "velocity_age_sec",
                    "safety_age_sec",
                    "autonomy_age_sec",
                    "logger_age_sec",
                ):
                    continue

                if key in state:
                    state[key] = value

            # UAV bridge calls absolute GPS altitude
            # altitude_msl. Keep the old generic altitude
            # field populated for the existing map/UI.
            if (
                fields.get("altitude_msl") is not None
                and not (
                    vehicle_id == "boat"
                    and source == "jetson"
                )
            ):
                state["altitude"] = fields[
                    "altitude_msl"
                ]

            # UAV bridge calls battery current simply
            # "current". Normalize it into the existing
            # shared state field.
            if (
                fields.get("current") is not None
                and not (
                    vehicle_id == "boat"
                    and source == "jetson"
                )
            ):
                state["battery_current"] = fields[
                    "current"
                ]

            # ------------------------------------------------
            # Vehicle transport freshness
            # ------------------------------------------------

            if (
                vehicle_id == "boat"
                and source == "jetson"
            ):
                state["jetson_link"] = True
                state["jetson_link_last_rx"] = now

            elif (
                vehicle_id == "boat"
                and source == "mavlink"
            ):
                state["mavlink_link"] = True
                state["mavlink_link_last_rx"] = now

            # Existing aggregate link remains populated for
            # backwards compatibility. Snapshot logic will
            # later derive it from the source-specific links.
            state["vehicle_link"] = True
            state["vehicle_link_last_rx"] = now
            state["last_rx"] = now

            # ------------------------------------------------
            # Source-specific ROS freshness
            #
            # Jetson sends source ages, not its monotonic clock.
            # Reconstruct equivalent local timestamps here.
            # ------------------------------------------------

            if source == "mavlink":

                # Direct battery messages do not carry a
                # dashboard-relative age; receiving one now
                # establishes freshness.
                if any(
                    key in fields
                    for key in (
                        "voltage",
                        "battery_percent",
                        "battery_current",
                        "battery_remaining",
                    )
                ):
                    state["battery_last_rx"] = now

                if "servo_outputs" in fields:
                    state["servo_output_last_rx"] = now

            else:

                freshness_fields = (
                    (
                        "battery_age_sec",
                        "battery_last_rx",
                    ),
                    (
                        "gate_age_sec",
                        "gate_last_rx",
                    ),
                    (
                        "bridge_age_sec",
                        "bridge_last_rx",
                    ),
                    (
                        "logger_age_sec",
                        "logger_last_rx",
                    ),
                )

                for age_key, stamp_key in freshness_fields:

                    # Do not erase an unrelated source timestamp
                    # merely because this packet omitted its age.
                    if age_key not in fields:
                        continue

                    age = fields.get(age_key)

                    if age is None:
                        state[stamp_key] = None
                    else:
                        state[stamp_key] = (
                            now
                            - max(
                                0.0,
                                float(age),
                            )
                        )

    def mark_link_offline(
        self,
        vehicle_id,
        source=None,
    ):

        with self.lock:

            if vehicle_id not in self.vehicles:
                return

            state = self.vehicles[vehicle_id]

            # Existing BoatClient calls this without a source.
            # For the USV that means the Jetson TCP transport.
            if source is None:
                source = (
                    "jetson"
                    if vehicle_id == "boat"
                    else "vehicle"
                )

            if (
                vehicle_id == "boat"
                and source == "jetson"
            ):
                state["jetson_link"] = False

            elif (
                vehicle_id == "boat"
                and source == "mavlink"
            ):
                state["mavlink_link"] = False

                # For the USV, direct MAVLink is the
                # authoritative autopilot connection.
                state["connected"] = False

            if vehicle_id == "boat":
                state["vehicle_link"] = bool(
                    state["jetson_link"]
                    or state["mavlink_link"]
                )
            else:
                state["vehicle_link"] = False

    def snapshot_raw(self):

        with self.lock:
            return copy.deepcopy(
                self.vehicles
            )
