#!/usr/bin/env python3

import glob
import time

import rclpy
from rclpy.node import Node

from mavros_msgs.msg import State
from rcl_interfaces.srv import GetParameters

import serial


class Esp32StatusBridge(Node):
    def __init__(self):
        super().__init__('esp32_status_bridge')

        self.declare_parameter('serial_port', 'auto')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('send_period', 0.25)

        self.serial_port_setting = (
            self.get_parameter('serial_port').get_parameter_value().string_value
        )
        self.baud_rate = (
            self.get_parameter('baud_rate').get_parameter_value().integer_value
        )
        self.send_period = (
            self.get_parameter('send_period').get_parameter_value().double_value
        )

        # Default to stopped until the real parameter is received.
        self.software_estop = True

        self.mavros_connected = False
        self.flight_mode = 'UNKNOWN'

        self.serial_connection = None
        self.connected_serial_port = None
        self.serial_ready_time = 0.0
        self.last_serial_warning_time = 0.0

        self.parameter_request_pending = False
        self.last_reported_status = None

        self.state_subscription = self.create_subscription(
            State,
            '/mavros/state',
            self.state_callback,
            10,
        )

        self.parameter_client = self.create_client(
            GetParameters,
            '/mavros_command_bridge/get_parameters',
        )

        self.timer = self.create_timer(
            self.send_period,
            self.timer_callback,
        )

        self.get_logger().info(
            'ESP32 status bridge started. '
            'Software E-stop defaults to engaged until verified.'
        )

    def state_callback(self, message):
        self.mavros_connected = message.connected
        self.flight_mode = message.mode.upper()

    def request_software_estop(self):
        if self.parameter_request_pending:
            return

        if not self.parameter_client.service_is_ready():
            return

        request = GetParameters.Request()
        request.names = ['software_estop']

        self.parameter_request_pending = True
        future = self.parameter_client.call_async(request)
        future.add_done_callback(self.software_estop_response)

    def software_estop_response(self, future):
        self.parameter_request_pending = False

        try:
            response = future.result()

            if response is None or len(response.values) == 0:
                self.software_estop = True
                return

            self.software_estop = response.values[0].bool_value

        except Exception as error:
            self.software_estop = True
            self.get_logger().error(
                f'Could not read software_estop parameter: {error}'
            )

    def select_esp32_command(self):
        # Explicit software stop has highest Jetson-side priority.
        if self.software_estop:
            return 'E_STOP'

        # Loss of MAVROS communication is treated as a communication stop.
        if not self.mavros_connected:
            return 'E_STOP'

        if self.flight_mode == 'GUIDED':
            return 'AUTONOMOUS'

        if self.flight_mode == 'MANUAL':
            return 'RC_MODE'

        # The current ESP32 firmware has no all-lights-off command.
        # Map ordinary HOLD to yellow so HOLD alone never produces red.
        if self.flight_mode == 'HOLD':
            return 'RC_MODE'

        # Unknown non-stopped modes are shown as yellow rather than red.
        return 'RC_MODE'

    def find_serial_ports(self):
        if self.serial_port_setting != 'auto':
            return [self.serial_port_setting]

        candidates = []

        candidates.extend(sorted(glob.glob('/dev/serial/by-id/*')))
        candidates.extend(sorted(glob.glob('/dev/ttyUSB*')))
        candidates.extend(sorted(glob.glob('/dev/ttyACM*')))

        # Remove duplicates while keeping the original order.
        return list(dict.fromkeys(candidates))

    def open_serial_connection(self):
        if self.serial_connection is not None:
            return True

        candidate_ports = self.find_serial_ports()

        if not candidate_ports:
            current_time = time.monotonic()

            if current_time - self.last_serial_warning_time >= 5.0:
                self.get_logger().warning(
                    'No ESP32 serial port found. Waiting for '
                    '/dev/ttyUSB*, /dev/ttyACM*, or /dev/serial/by-id/*'
                )
                self.last_serial_warning_time = current_time

            return False

        for port in candidate_ports:
            try:
                self.serial_connection = serial.Serial(
                    port=port,
                    baudrate=self.baud_rate,
                    timeout=0,
                    write_timeout=0.1,
                )

                self.connected_serial_port = port

                # Many ESP32 boards reset when serial is opened.
                self.serial_ready_time = time.monotonic() + 2.0

                self.get_logger().info(
                    f'Connected to ESP32 on {port} at {self.baud_rate} baud'
                )

                return True

            except (serial.SerialException, OSError) as error:
                current_time = time.monotonic()

                if current_time - self.last_serial_warning_time >= 5.0:
                    self.get_logger().warning(
                        f'Could not open serial port {port}: {error}'
                    )
                    self.last_serial_warning_time = current_time

        return False

    def close_serial_connection(self):
        if self.serial_connection is not None:
            try:
                self.serial_connection.close()
            except Exception:
                pass

        self.serial_connection = None
        self.connected_serial_port = None
        self.serial_ready_time = 0.0

    def report_status_change(self, command):
        status = (
            command,
            self.flight_mode,
            self.software_estop,
            self.mavros_connected,
        )

        if status == self.last_reported_status:
            return

        self.last_reported_status = status

        self.get_logger().info(
            f'ESP32 command={command}, '
            f'mode={self.flight_mode}, '
            f'software_estop={self.software_estop}, '
            f'mavros_connected={self.mavros_connected}'
        )

    def send_command(self, command):
        if not self.open_serial_connection():
            return

        if time.monotonic() < self.serial_ready_time:
            return

        try:
            message = f'{command}\n'.encode('ascii')

            self.serial_connection.write(message)
            self.serial_connection.flush()

        except (serial.SerialException, serial.SerialTimeoutException, OSError) as error:
            self.get_logger().error(
                f'ESP32 serial connection failed: {error}'
            )
            self.close_serial_connection()

    def timer_callback(self):
        self.request_software_estop()

        command = self.select_esp32_command()

        self.report_status_change(command)
        self.send_command(command)

    def destroy_node(self):
        # Attempt an explicit stop before closing serial.
        if self.serial_connection is not None:
            try:
                self.serial_connection.write(b'E_STOP\n')
                self.serial_connection.flush()
            except Exception:
                pass

        self.close_serial_connection()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = Esp32StatusBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
