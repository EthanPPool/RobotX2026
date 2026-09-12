#!/usr/bin/env python3

import json
import socket
import threading
import uuid

from robotx_dashboard.clients.base_client import BaseVehicleClient


class UavClient(BaseVehicleClient):

    def __init__(
        self,
        vehicle_id,
        state_update_callback,
        host="192.168.2.104",
        port=8766,
        reconnect_delay=1.0,
        command_timeout=5.0,
    ):
        super().__init__(
            vehicle_id,
            state_update_callback,
        )

        self.host = host
        self.port = int(port)
        self.reconnect_delay = float(reconnect_delay)
        self.command_timeout = float(command_timeout)

        self._stop_event = threading.Event()
        self._thread = None
        self._socket = None

        self._socket_lock = threading.RLock()
        self._pending_lock = threading.RLock()
        self._pending = {}

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
        if not self.connected:
            return {
                "success": False,
                "message": "UAV transport is offline",
            }

        request_id = uuid.uuid4().hex

        waiter = {
            "event": threading.Event(),
            "response": None,
        }

        with self._pending_lock:
            self._pending[request_id] = waiter

        message = {
            "type": "command",
            "request_id": request_id,
            "command": str(command),
            "data": dict(data or {}),
        }

        try:
            self._send(message)

        except Exception as exc:
            with self._pending_lock:
                self._pending.pop(
                    request_id,
                    None,
                )

            return {
                "success": False,
                "message": (
                    f"UAV command send failed: {exc}"
                ),
            }

        if not waiter["event"].wait(
            self.command_timeout
        ):
            with self._pending_lock:
                self._pending.pop(
                    request_id,
                    None,
                )

            return {
                "success": False,
                "message": "UAV command timed out",
            }

        response = waiter["response"]

        if not isinstance(response, dict):
            return {
                "success": False,
                "message": "Invalid UAV response",
            }

        return {
            "success": bool(
                response.get(
                    "success",
                    False,
                )
            ),
            "message": str(
                response.get(
                    "message",
                    "",
                )
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

                    self._handle_message(
                        message
                    )

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

                self._fail_pending(
                    "UAV connection lost"
                )

            self._stop_event.wait(
                self.reconnect_delay
            )

    def _handle_message(self, message):
        if not isinstance(message, dict):
            return

        message_type = message.get(
            "type"
        )

        if message_type == "telemetry":

            if message.get(
                "vehicle"
            ) not in (
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

            return

        if message_type == "response":

            request_id = message.get(
                "request_id"
            )

            if not request_id:
                return

            with self._pending_lock:
                waiter = self._pending.pop(
                    request_id,
                    None,
                )

            if waiter is None:
                return

            waiter["response"] = message
            waiter["event"].set()

    def _send(self, message):
        payload = (
            json.dumps(
                message,
                separators=(",", ":"),
            )
            + "\n"
        ).encode("utf-8")

        with self._socket_lock:

            if self._socket is None:
                raise ConnectionError(
                    "UAV socket unavailable"
                )

            self._socket.sendall(
                payload
            )

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

    def _fail_pending(self, message):
        with self._pending_lock:
            pending = list(
                self._pending.values()
            )

            self._pending.clear()

        for waiter in pending:
            waiter["response"] = {
                "success": False,
                "message": message,
            }

            waiter["event"].set()
