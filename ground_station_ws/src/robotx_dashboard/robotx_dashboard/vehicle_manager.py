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

                # Ground-station <-> vehicle transport
                "vehicle_link": False,
                "vehicle_link_last_rx": None,

                # Autopilot / MAVROS state
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
                "battery_last_rx": None,

                "buoy_count": 0,

                "gate_confidence": None,
                "gate_x": None,
                "gate_y": None,
                "gate_last_rx": None,

                "control_forward": 0.0,
                "control_yaw": 0.0,

                "mission_state": None,

                "bridge_forward": 0.0,
                "bridge_yaw": 0.0,
                "bridge_last_rx": None,

                # Jetson-side control/safety authority.
                "bridge_alive": False,
                "control_ready": False,
                "bridge_command_active": False,
                "control_state": "BOOT SAFE",
                "software_stop": "UNKNOWN",
                "autonomy_enabled": False,
                "can_enable": False,

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

            for key, value in fields.items():

                if key in (
                    "battery_age_sec",
                    "gate_age_sec",
                    "bridge_age_sec",
                ):
                    continue

                if key in state:
                    state[key] = value

            # ------------------------------------------------
            # Vehicle transport freshness
            # ------------------------------------------------

            state["vehicle_link"] = True
            state["vehicle_link_last_rx"] = now
            state["last_rx"] = now

            # ------------------------------------------------
            # Source-specific ROS freshness
            #
            # Jetson sends source ages, not its monotonic clock.
            # Reconstruct equivalent local timestamps here.
            # ------------------------------------------------

            battery_age = fields.get(
                "battery_age_sec"
            )

            if battery_age is None:
                state["battery_last_rx"] = None
            else:
                state["battery_last_rx"] = (
                    now - max(
                        0.0,
                        float(battery_age),
                    )
                )

            gate_age = fields.get(
                "gate_age_sec"
            )

            if gate_age is None:
                state["gate_last_rx"] = None
            else:
                state["gate_last_rx"] = (
                    now - max(
                        0.0,
                        float(gate_age),
                    )
                )

            bridge_age = fields.get(
                "bridge_age_sec"
            )

            if bridge_age is None:
                state["bridge_last_rx"] = None
            else:
                state["bridge_last_rx"] = (
                    now - max(
                        0.0,
                        float(bridge_age),
                    )
                )

    def mark_link_offline(
        self,
        vehicle_id,
    ):

        with self.lock:

            if vehicle_id not in self.vehicles:
                return

            self.vehicles[
                vehicle_id
            ]["vehicle_link"] = False

    def snapshot_raw(self):

        with self.lock:
            return copy.deepcopy(
                self.vehicles
            )
