#!/usr/bin/env python3

import copy
import threading


class VehicleManager:
    """
    Central vehicle-state store for the RobotX ground station.

    During the transport refactor, dashboard.py and the future
    vehicle clients share this same state dictionary.
    """

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

            for key, value in fields.items():

                if key in state:
                    state[key] = value

    def snapshot_raw(self):

        with self.lock:
            return copy.deepcopy(
                self.vehicles
            )
