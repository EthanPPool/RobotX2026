#!/usr/bin/env python3

import json
import math
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from geometry_msgs.msg import TwistStamped
from mavros_msgs.msg import State
from rclpy.node import Node
from std_srvs.srv import SetBool

from boat_interfaces.msg import DetectedObjectArray, Gate


PAGE = r"""<!DOCTYPE html>
<html>
<head>
    <meta charset="utf-8">
    <title>RobotX Safe Gate Test</title>

    <style>
        body {
            font-family: Arial, sans-serif;
            margin: 0;
            background: #101418;
            color: #eeeeee;
        }

        .page {
            max-width: 1050px;
            margin: auto;
            padding: 24px;
        }

        .warning {
            background: #4a1616;
            border: 2px solid #d64545;
            padding: 14px;
            margin: 18px 0;
            font-weight: bold;
        }

        .grid {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(280px, 1fr));
            gap: 16px;
        }

        .card {
            background: #1c2329;
            border: 1px solid #3b444c;
            border-radius: 8px;
            padding: 18px;
        }

        .row {
            display: flex;
            justify-content: space-between;
            margin: 10px 0;
        }

        .value {
            font-weight: bold;
        }

        .good {
            color: #62d26f;
        }

        .bad {
            color: #ff6363;
        }

        .neutral {
            color: #f5d76e;
        }

        button {
            width: 100%;
            min-height: 62px;
            margin-top: 12px;
            font-size: 18px;
            font-weight: bold;
            border: none;
            border-radius: 8px;
            cursor: pointer;
        }

        #enableButton {
            background: #2f8f46;
            color: white;
        }

        #enableButton:disabled {
            background: #4b555b;
            cursor: not-allowed;
        }

        #stopButton {
            background: #c62828;
            color: white;
            font-size: 23px;
        }

        #message {
            min-height: 28px;
            margin-top: 16px;
            font-family: monospace;
        }

        .small {
            color: #abb4bb;
            font-size: 13px;
        }
    </style>
</head>

<body>
<div class="page">

    <h1>RobotX Safe Gate Test</h1>
    <div class="small">
        Simplified single-gate autonomy controller
    </div>

    <div class="warning">
        SOFTWARE STOP IS NOT THE PHYSICAL MOTOR E-STOP.
        The physical E-stop must independently remove motor power.
    </div>

    <div class="grid">

        <div class="card">
            <h2>Vehicle</h2>

            <div class="row">
                <span>MAVROS</span>
                <span id="connected" class="value">---</span>
            </div>

            <div class="row">
                <span>Armed</span>
                <span id="armed" class="value">---</span>
            </div>

            <div class="row">
                <span>Mode</span>
                <span id="mode" class="value">---</span>
            </div>
        </div>

        <div class="card">
            <h2>Perception</h2>

            <div class="row">
                <span>Buoys</span>
                <span id="buoys" class="value">0</span>
            </div>

            <div class="row">
                <span>Gate</span>
                <span id="gate" class="value">---</span>
            </div>

            <div class="row">
                <span>Confidence</span>
                <span id="confidence" class="value">---</span>
            </div>

            <div class="row">
                <span>Center X</span>
                <span id="gateX" class="value">---</span>
            </div>

            <div class="row">
                <span>Center Y</span>
                <span id="gateY" class="value">---</span>
            </div>
        </div>

        <div class="card">
            <h2>Follower Command</h2>

            <div class="row">
                <span>Forward</span>
                <span id="controlX" class="value">0.000 m/s</span>
            </div>

            <div class="row">
                <span>Yaw</span>
                <span id="controlYaw" class="value">0.000 rad/s</span>
            </div>
        </div>

        <div class="card">
            <h2>Bridge Output</h2>

            <div class="row">
                <span>Forward</span>
                <span id="bridgeX" class="value">0.000 m/s</span>
            </div>

            <div class="row">
                <span>Yaw</span>
                <span id="bridgeYaw" class="value">0.000 rad/s</span>
            </div>

            <div class="row">
                <span>Control State</span>
                <span id="controlState" class="value">BOOT SAFE</span>
            </div>
        </div>

    </div>

    <div class="card" style="margin-top:16px;">
        <h2>Autonomy Control</h2>

        <button
            id="enableButton"
            onclick="enableAutonomy()"
            disabled>
            ENABLE AUTONOMY
        </button>

        <button
            id="stopButton"
            onclick="softwareStop()">
            SOFTWARE STOP
        </button>

        <div id="message"></div>
    </div>

</div>

<script>

function setState(id, text, state) {
    const e = document.getElementById(id);

    e.textContent = text;
    e.className = "value";

    if (state === true) {
        e.classList.add("good");
    } else if (state === false) {
        e.classList.add("bad");
    } else {
        e.classList.add("neutral");
    }
}


async function refreshStatus() {
    try {
        const response = await fetch(
            "/api/status",
            {cache: "no-store"}
        );

        const s = await response.json();

        setState(
            "connected",
            s.connected ? "CONNECTED" : "DISCONNECTED",
            s.connected
        );

        setState(
            "armed",
            s.armed ? "ARMED" : "DISARMED",
            s.armed
        );

        setState(
            "mode",
            s.mode || "---",
            s.mode === "GUIDED"
        );

        document.getElementById("buoys").textContent =
            s.buoy_count;

        setState(
            "gate",
            s.gate_fresh ? "DETECTED" : "NONE",
            s.gate_fresh
        );

        document.getElementById("confidence").textContent =
            s.gate_confidence.toFixed(3);

        document.getElementById("gateX").textContent =
            s.gate_x.toFixed(2) + " m";

        document.getElementById("gateY").textContent =
            s.gate_y.toFixed(2) + " m";

        document.getElementById("controlX").textContent =
            s.control_forward.toFixed(3) + " m/s";

        document.getElementById("controlYaw").textContent =
            s.control_yaw.toFixed(3) + " rad/s";

        document.getElementById("bridgeX").textContent =
            s.bridge_forward.toFixed(3) + " m/s";

        document.getElementById("bridgeYaw").textContent =
            s.bridge_yaw.toFixed(3) + " rad/s";

        setState(
            "controlState",
            s.control_state,
            s.control_state === "ENABLED"
                ? true
                : s.control_state === "STOPPED"
                    ? false
                    : null
        );

        document.getElementById(
            "enableButton"
        ).disabled = !s.can_enable;

    } catch (err) {
        document.getElementById("message").textContent =
            "Dashboard connection error: " + err;
    }
}


async function postAction(path) {
    const response = await fetch(
        path,
        {
            method: "POST",
            cache: "no-store"
        }
    );

    const result = await response.json();

    document.getElementById("message").textContent =
        result.message;

    await refreshStatus();
}


async function enableAutonomy() {
    document.getElementById("message").textContent =
        "Requesting autonomy enable...";

    await postAction("/api/enable");
}


async function softwareStop() {
    document.getElementById("message").textContent =
        "Sending SOFTWARE STOP...";

    await postAction("/api/stop");
}


setInterval(refreshStatus, 500);
refreshStatus();

</script>

</body>
</html>
"""


class DashboardNode(Node):

    def __init__(self):
        super().__init__('boat_dashboard')

        self.port = int(
            self.declare_parameter(
                'port',
                8080
            ).value
        )

        self.gate_timeout = float(
            self.declare_parameter(
                'gate_timeout',
                0.50
            ).value
        )

        self.min_gate_confidence = float(
            self.declare_parameter(
                'min_gate_confidence',
                0.75
            ).value
        )

        self.vehicle_state = None

        self.buoy_count = 0

        self.last_gate = None
        self.last_gate_time = None

        self.control_forward = 0.0
        self.control_yaw = 0.0

        self.bridge_forward = 0.0
        self.bridge_yaw = 0.0
        self.last_bridge_time = None

        self.control_state = 'BOOT SAFE'

        self.action_lock = threading.Lock()

        self.create_subscription(
            State,
            '/mavros/state',
            self.state_callback,
            10
        )

        self.create_subscription(
            DetectedObjectArray,
            '/perception/objects',
            self.objects_callback,
            10
        )

        self.create_subscription(
            Gate,
            '/perception/gate',
            self.gate_callback,
            10
        )

        self.create_subscription(
            TwistStamped,
            '/control/cmd_vel',
            self.control_callback,
            10
        )

        self.create_subscription(
            TwistStamped,
            '/mavros/setpoint_velocity/cmd_vel',
            self.bridge_callback,
            10
        )

        self.estop_client = self.create_client(
            SetBool,
            '/vehicle/software_estop'
        )

        self.autonomy_client = self.create_client(
            SetBool,
            '/vehicle/set_autonomy'
        )

        self.follower_client = self.create_client(
            SetBool,
            '/control/set_enabled'
        )

        handler = self.make_handler()

        self.http_server = ThreadingHTTPServer(
            ('0.0.0.0', self.port),
            handler
        )

        self.http_thread = threading.Thread(
            target=self.http_server.serve_forever,
            daemon=True
        )

        self.http_thread.start()

        self.get_logger().warn(
            f'Dashboard listening on port {self.port}. '
            'Control starts BOOT SAFE.'
        )

    def state_callback(self, msg):
        self.vehicle_state = msg

    def objects_callback(self, msg):
        self.buoy_count = len(msg.objects)

    def gate_callback(self, msg):
        self.last_gate = msg
        self.last_gate_time = time.monotonic()

    def control_callback(self, msg):
        self.control_forward = float(
            msg.twist.linear.x
        )

        self.control_yaw = float(
            msg.twist.angular.z
        )

    def bridge_callback(self, msg):
        self.bridge_forward = float(
            msg.twist.linear.x
        )

        self.bridge_yaw = float(
            msg.twist.angular.z
        )

        self.last_bridge_time = time.monotonic()

    def gate_is_fresh(self):
        if (
            self.last_gate is None
            or self.last_gate_time is None
        ):
            return False

        age = (
            time.monotonic()
            - self.last_gate_time
        )

        if age > self.gate_timeout:
            return False

        confidence = float(
            self.last_gate.confidence
        )

        x = float(
            self.last_gate.center.x
        )

        y = float(
            self.last_gate.center.y
        )

        return (
            math.isfinite(confidence)
            and math.isfinite(x)
            and math.isfinite(y)
            and confidence >= self.min_gate_confidence
            and x > 0.0
        )

    def bridge_is_zero(self):
        return (
            abs(self.bridge_forward) <= 0.01
            and abs(self.bridge_yaw) <= 0.01
        )

    def get_status(self):
        connected = False
        armed = False
        mode = ''

        if self.vehicle_state is not None:
            connected = bool(
                self.vehicle_state.connected
            )

            armed = bool(
                self.vehicle_state.armed
            )

            mode = str(
                self.vehicle_state.mode
            )

        gate_fresh = self.gate_is_fresh()

        gate_confidence = 0.0
        gate_x = 0.0
        gate_y = 0.0

        if self.last_gate is not None:
            gate_confidence = float(
                self.last_gate.confidence
            )

            gate_x = float(
                self.last_gate.center.x
            )

            gate_y = float(
                self.last_gate.center.y
            )

        bridge_alive = False

        if self.last_bridge_time is not None:
            bridge_alive = (
                time.monotonic()
                - self.last_bridge_time
            ) <= 0.50

        can_enable = (
            connected
            and armed
            and mode == 'GUIDED'
            and gate_fresh
            and bridge_alive
            and self.bridge_is_zero()
            and self.control_state != 'ENABLED'
        )

        return {
            'connected': connected,
            'armed': armed,
            'mode': mode,

            'buoy_count': self.buoy_count,

            'gate_fresh': gate_fresh,
            'gate_confidence': gate_confidence,
            'gate_x': gate_x,
            'gate_y': gate_y,

            'control_forward': self.control_forward,
            'control_yaw': self.control_yaw,

            'bridge_forward': self.bridge_forward,
            'bridge_yaw': self.bridge_yaw,

            'bridge_alive': bridge_alive,

            'control_state': self.control_state,

            'can_enable': can_enable,
        }

    def call_bool_service(
        self,
        client,
        value,
        timeout=2.0
    ):
        if not client.wait_for_service(
            timeout_sec=0.25
        ):
            return (
                False,
                'ROS service unavailable'
            )

        request = SetBool.Request()
        request.data = bool(value)

        future = client.call_async(request)

        deadline = time.monotonic() + timeout

        while (
            not future.done()
            and time.monotonic() < deadline
        ):
            time.sleep(0.02)

        if not future.done():
            return (
                False,
                'ROS service call timed out'
            )

        try:
            response = future.result()

        except Exception as exc:
            return (
                False,
                f'ROS service exception: {exc}'
            )

        if response is None:
            return (
                False,
                'ROS service returned no response'
            )

        return (
            bool(response.success),
            str(response.message)
        )

    def execute_stop(self):
        with self.action_lock:

            messages = []
            success = True

            ok, msg = self.call_bool_service(
                self.estop_client,
                True
            )

            messages.append(
                'software_stop: ' + msg
            )

            if not ok:
                success = False

            ok, msg = self.call_bool_service(
                self.autonomy_client,
                False
            )

            messages.append(
                'autonomy: ' + msg
            )

            if not ok:
                success = False

            ok, msg = self.call_bool_service(
                self.follower_client,
                False
            )

            messages.append(
                'follower: ' + msg
            )

            if not ok:
                success = False

            if success:
                self.control_state = 'STOPPED'
            else:
                self.control_state = 'STOP COMMAND FAILED'

            return (
                success,
                ' | '.join(messages)
            )

    def execute_enable(self):
        with self.action_lock:

            status = self.get_status()

            if not status['connected']:
                return (
                    False,
                    'Enable rejected: MAVROS is not connected'
                )

            if not status['armed']:
                return (
                    False,
                    'Enable rejected: vehicle is not armed'
                )

            if status['mode'] != 'GUIDED':
                return (
                    False,
                    'Enable rejected: vehicle is not in GUIDED'
                )

            if not status['gate_fresh']:
                return (
                    False,
                    'Enable rejected: no fresh high-confidence gate'
                )

            if not status['bridge_alive']:
                return (
                    False,
                    'Enable rejected: bridge output is not alive'
                )

            if not self.bridge_is_zero():
                return (
                    False,
                    'Enable rejected: bridge output is not zero'
                )

            messages = []

            ok, msg = self.call_bool_service(
                self.estop_client,
                False
            )

            messages.append(
                'software_stop: ' + msg
            )

            if not ok:
                self.control_state = 'STOPPED'
                return (
                    False,
                    ' | '.join(messages)
                )

            ok, msg = self.call_bool_service(
                self.follower_client,
                True
            )

            messages.append(
                'follower: ' + msg
            )

            if not ok:
                self.execute_fail_safe_stop()
                return (
                    False,
                    ' | '.join(messages)
                )

            ok, msg = self.call_bool_service(
                self.autonomy_client,
                True
            )

            messages.append(
                'autonomy: ' + msg
            )

            if not ok:
                self.execute_fail_safe_stop()
                return (
                    False,
                    ' | '.join(messages)
                )

            self.control_state = 'ENABLED'

            return (
                True,
                ' | '.join(messages)
            )

    def execute_fail_safe_stop(self):

        self.call_bool_service(
            self.estop_client,
            True
        )

        self.call_bool_service(
            self.autonomy_client,
            False
        )

        self.call_bool_service(
            self.follower_client,
            False
        )

        self.control_state = 'STOPPED'

    def make_handler(self):
        node = self

        class Handler(BaseHTTPRequestHandler):

            def send_json(
                self,
                payload,
                status=200
            ):
                data = json.dumps(
                    payload
                ).encode('utf-8')

                self.send_response(status)

                self.send_header(
                    'Content-Type',
                    'application/json'
                )

                self.send_header(
                    'Content-Length',
                    str(len(data))
                )

                self.send_header(
                    'Cache-Control',
                    'no-store'
                )

                self.end_headers()

                self.wfile.write(data)

            def do_GET(self):

                if self.path == '/':
                    data = PAGE.encode('utf-8')

                    self.send_response(200)

                    self.send_header(
                        'Content-Type',
                        'text/html; charset=utf-8'
                    )

                    self.send_header(
                        'Content-Length',
                        str(len(data))
                    )

                    self.send_header(
                        'Cache-Control',
                        'no-store'
                    )

                    self.end_headers()

                    self.wfile.write(data)
                    return

                if self.path == '/api/status':
                    self.send_json(
                        node.get_status()
                    )
                    return

                self.send_json(
                    {
                        'success': False,
                        'message': 'Not found'
                    },
                    status=404
                )

            def do_POST(self):

                if self.path == '/api/stop':

                    success, message = (
                        node.execute_stop()
                    )

                    self.send_json({
                        'success': success,
                        'message': message
                    })

                    return

                if self.path == '/api/enable':

                    success, message = (
                        node.execute_enable()
                    )

                    self.send_json({
                        'success': success,
                        'message': message
                    })

                    return

                self.send_json(
                    {
                        'success': False,
                        'message': 'Not found'
                    },
                    status=404
                )

            def log_message(
                self,
                format,
                *args
            ):
                return

        return Handler

    def destroy_node(self):

        try:
            self.http_server.shutdown()
            self.http_server.server_close()

        except Exception:
            pass

        return super().destroy_node()


def main(args=None):

    rclpy.init(args=args)

    node = DashboardNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        try:
            node.execute_stop()
        except Exception:
            pass

        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
