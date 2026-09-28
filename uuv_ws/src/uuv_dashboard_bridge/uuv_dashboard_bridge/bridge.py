#!/usr/bin/env python3
import json
import socket
import threading
import time

import rclpy
from mavros_msgs.srv import CommandBool
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger


class UuvDashboardBridge(Node):
    """Newline-delimited JSON TCP bridge matching the existing RobotX vehicle protocol."""

    def __init__(self):
        super().__init__('uuv_dashboard_bridge')
        self.host = str(self.declare_parameter('host', '0.0.0.0').value)
        self.port = int(self.declare_parameter('port', 8770).value)
        self.telemetry_hz = float(self.declare_parameter('telemetry_hz', 5.0).value)
        self.telemetry = {'id':'uuv','name':'UUV','type':'UUV'}
        self.lock = threading.RLock()
        self.client_lock = threading.RLock()
        self.client_socket = None
        self.software_stop = True
        self.stop_event = threading.Event()

        self.create_subscription(String, '/uuv/vehicle/status', self._status_cb, 10)
        self.enable_client = self.create_client(SetBool, '/uuv/control/set_enabled')
        self.reset_client = self.create_client(Trigger, '/uuv/control/reset_mission')
        self.arm_client = self.create_client(CommandBool, '/uuv/mavros/cmd/arming')

        self.server_thread = threading.Thread(target=self._server_loop, daemon=True, name='uuv-dashboard-server')
        self.server_thread.start()
        self.timer = self.create_timer(1.0 / max(self.telemetry_hz, 1.0), self._publish_telemetry)
        self.get_logger().info(f'UUV dashboard bridge listening on TCP {self.host}:{self.port}')

    def _status_cb(self, msg):
        try:
            obj = json.loads(msg.data)
            if isinstance(obj, dict):
                with self.lock: self.telemetry.update(obj)
        except Exception:
            pass

    def _server_loop(self):
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind((self.host, self.port)); server.listen(1); server.settimeout(1.0)
        while rclpy.ok() and not self.stop_event.is_set():
            try:
                client, addr = server.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            client.settimeout(1.0)
            with self.client_lock:
                old = self.client_socket; self.client_socket = client
            if old:
                try: old.close()
                except OSError: pass
            self.get_logger().info(f'Ground station connected to UUV bridge: {addr[0]}:{addr[1]}')
            threading.Thread(target=self._client_loop, args=(client,), daemon=True).start()
        try: server.close()
        except OSError: pass

    def _client_loop(self, client):
        buf = b''
        try:
            while rclpy.ok() and not self.stop_event.is_set():
                try: chunk = client.recv(4096)
                except socket.timeout: continue
                if not chunk: break
                buf += chunk
                while b'\n' in buf:
                    line, buf = buf.split(b'\n', 1)
                    if not line.strip(): continue
                    try: message = json.loads(line.decode('utf-8'))
                    except Exception: continue
                    self._handle_message(client, message)
        finally:
            with self.client_lock:
                if self.client_socket is client: self.client_socket = None
            try: client.close()
            except OSError: pass

    def _call(self, client, request, timeout=3.0):
        if not client.wait_for_service(timeout_sec=0.5):
            return False, 'ROS service unavailable'
        future = client.call_async(request)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if future.done():
                try:
                    response = future.result()
                    success = bool(getattr(response, 'success', True))
                    message = str(getattr(response, 'message', 'ok'))
                    return success, message
                except Exception as exc:
                    return False, str(exc)
            time.sleep(0.02)
        return False, 'ROS service timeout'

    def _handle_message(self, sock, message):
        if not isinstance(message, dict) or message.get('type') != 'command': return
        command = str(message.get('command', '')).strip().lower()
        success, result = False, f'Unknown command: {command}'
        if command == 'stop':
            self.software_stop = True
            req = SetBool.Request(); req.data = False
            success, result = self._call(self.enable_client, req)
            if success: result = 'UUV software stop latched; autonomy disabled'
        elif command == 'clear_stop':
            self.software_stop = False; success, result = True, 'UUV software stop cleared; autonomy remains disabled'
        elif command == 'enable':
            if self.software_stop:
                success, result = False, 'clear software stop before enabling autonomy'
            else:
                req = SetBool.Request(); req.data = True
                success, result = self._call(self.enable_client, req)
        elif command in ('disable',):
            req = SetBool.Request(); req.data = False
            success, result = self._call(self.enable_client, req)
        elif command in ('arm','disarm'):
            req = CommandBool.Request(); req.value = command == 'arm'
            success, result = self._call(self.arm_client, req)
        elif command == 'reset_mission':
            success, result = self._call(self.reset_client, Trigger.Request())
        self._send(sock, {'type':'response','request_id':message.get('request_id'),'success':bool(success),'message':str(result)})

    def _publish_telemetry(self):
        with self.lock: data = dict(self.telemetry)
        data['bridge_alive'] = True
        data['software_stop'] = 'ACTIVE' if self.software_stop else 'CLEAR'
        data['control_state'] = 'STOPPED' if self.software_stop else ('ENABLED' if data.get('autonomy_enabled') else 'READY')
        data['can_enable'] = bool(data.get('mavros_connected') and not self.software_stop)
        with self.client_lock: client = self.client_socket
        if client is None: return
        try: self._send(client, {'type':'telemetry','data':data})
        except OSError:
            with self.client_lock:
                if self.client_socket is client: self.client_socket = None

    @staticmethod
    def _send(sock, message):
        payload = (json.dumps(message, separators=(',', ':')) + '\n').encode('utf-8')
        sock.sendall(payload)

    def destroy_node(self):
        self.stop_event.set()
        with self.client_lock:
            client = self.client_socket; self.client_socket = None
        if client:
            try: client.close()
            except OSError: pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args); node = UuvDashboardBridge()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__': main()
