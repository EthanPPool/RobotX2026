#!/usr/bin/env python3

from abc import ABC, abstractmethod


class BaseVehicleClient(ABC):
    """
    Common interface used by the ground-station dashboard.

    Vehicle-specific transports such as WebSocket, MAVLink,
    or another protocol implement this interface.
    """

    def __init__(self, vehicle_id, state_update_callback):
        self.vehicle_id = vehicle_id
        self.state_update_callback = state_update_callback
        self.connected = False

    @abstractmethod
    def start(self):
        """Start the vehicle connection/client."""
        raise NotImplementedError

    @abstractmethod
    def stop(self):
        """Stop the vehicle connection/client."""
        raise NotImplementedError

    @abstractmethod
    def command(self, command, data=None):
        """Send a command to the vehicle."""
        raise NotImplementedError

    def update_state(self, **fields):
        """
        Pass new telemetry into VehicleManager without the
        dashboard needing to know which transport produced it.
        """
        self.state_update_callback(
            self.vehicle_id,
            fields,
        )
