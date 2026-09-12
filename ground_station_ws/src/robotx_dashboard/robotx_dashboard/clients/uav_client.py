#!/usr/bin/env python3

import json
import socket
import threading

from robotx_dashboard.clients.base_client import BaseVehicleClient


class UavClient(BaseVehicleClient):

    def __init__(
        self,
        vehicle_id,
        state_update_callback,
        host="192.168.2.104",
        port=8766,
        reconnect_delay=1.0,
    ):
        super().__init__(
            vehicle_id,
            state_update_callback,
        )

        self.host = host
        self.port = int(port)
        self.reconnect_delay = float(reconnect_delay)

        self._stop_event = threading.Event()
        self._thread = None
        self._socket = None
        self._socket_lock = threading.RLock()

    def start(self):
        if self._thread and self._thread.is_alive():
            return

        self._stop_event.clear()

        self._thread = threading.Thread(
            target=self._run,
            name="uav-client",
            daemon=True,
        )

        self._thread.start()

    def stop(self):
        self._stop_event.set()
        self._close_socket()

        if self._thread:
            self._thread.join(timeout=2.0)

        self.connected = False

    def command(self, command, data=None):
        # First integration phase is intentionally read-only.
        return {
            "success": False,
            "message": (
                "UAV remote commands are disabled; "
                "telemetry-only integration is active"
            ),
        }

    def _run(self):
        while not self._stop_event.is_set():

            try:
                sock = socket.create_connection(
                    (self.host, self.port),
                    timeout=3.0,
                )

                sock.settimeout(None)

                with self._socket_lock:
                    self._socket = sock

                self.connected = True

                file_obj = sock.makefile(
                    "r",
                    encoding="utf-8",
                )

                for line in file_obj:

                    if self._stop_event.is_set():
                        break

                    line = line.strip()

                    if not line:
                        continue

                    try:
                        message = json.loads(line)

                    except json.JSONDecodeError:
                        continue

                    self._handle_message(message)

            except (
                ConnectionRefusedError,
                ConnectionResetError,
                ConnectionAbortedError,
                TimeoutError,
                OSError,
            ):
                pass

            finally:
                self.connected = False
                self._close_socket()

                try:
                    owner = getattr(
                        self.state_update_callback,
                        "__self__",
                        None,
                    )

                    if (
                        owner is not None
                        and hasattr(
                            owner,
                            "mark_link_offline",
                        )
                    ):
                        owner.mark_link_offline(
                            self.vehicle_id
                        )

                except Exception:
                    pass

            self._stop_event.wait(
                self.reconnect_delay
            )

    def _handle_message(self, message):
        if not isinstance(message, dict):
            return

        if message.get("type") != "telemetry":
            return

        if message.get("vehicle") not in (
            None,
            "uav",
        ):
            return

        data = message.get(
            "data",
            {},
        )

        if isinstance(data, dict):
            self.update_state(**data)

    def _close_socket(self):
        with self._socket_lock:
            sock = self._socket
            self._socket = None

        if sock is None:
            return

        try:
            sock.shutdown(
                socket.SHUT_RDWR
            )
        except OSError:
            pass

        try:
            sock.close()
        except OSError:
            pass
