#!/usr/bin/env python3

import math

import rclpy
from mavros_msgs.msg import State
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_srvs.srv import Trigger


EARTH_RADIUS_M = 6371000.0


class UAVGeofence(Node):
    """
    Software geofence supervisor.

    Monitors a GPS inclusion polygon. If the armed UAV moves outside
    the polygon for a configurable number of consecutive samples,
    request RTL through /vehicle/rtl.

    This is a supervisory layer. The flight controller's native
    ArduPilot geofence should remain the hard containment layer.
    """

    def __init__(self):
        super().__init__('uav_geofence')

        self.declare_parameter('enabled', False)

        self.declare_parameter(
            'state_topic',
            '/mavros/mavros/state'
        )

        self.declare_parameter(
            'gps_topic',
            '/mavros/mavros/global_position/raw/fix'
        )

        self.declare_parameter(
            'rtl_service',
            '/vehicle/rtl'
        )

        # Overwrite these with the actual polygon.
        self.declare_parameter(
            'fence_latitudes',
            [0.0]
        )

        self.declare_parameter(
            'fence_longitudes',
            [0.0]
        )

        self.declare_parameter(
            'warning_margin_m',
            5.0
        )

        self.declare_parameter(
            'gps_timeout',
            2.0
        )

        self.declare_parameter(
            'breach_confirmations',
            3
        )

        self.declare_parameter(
            'check_rate',
            10.0
        )

        self.enabled = bool(
            self.get_parameter('enabled').value
        )

        self.latitudes = [
            float(v)
            for v in self.get_parameter(
                'fence_latitudes'
            ).value
        ]

        self.longitudes = [
            float(v)
            for v in self.get_parameter(
                'fence_longitudes'
            ).value
        ]

        self.warning_margin = float(
            self.get_parameter(
                'warning_margin_m'
            ).value
        )

        self.gps_timeout = float(
            self.get_parameter(
                'gps_timeout'
            ).value
        )

        self.breach_confirmations = max(
            1,
            int(
                self.get_parameter(
                    'breach_confirmations'
                ).value
            )
        )

        self.state = None
        self.gps = None
        self.last_gps_time = None

        self.breach_count = 0
        self.rtl_requested = False
        self.rtl_pending = False

        self.last_status = None

        self.configured = self._validate_polygon()

        if self.configured:
            self.reference_lat = math.radians(
                sum(self.latitudes)
                / len(self.latitudes)
            )

            self.reference_lon = math.radians(
                sum(self.longitudes)
                / len(self.longitudes)
            )

            self.polygon_xy = [
                self._latlon_to_xy(lat, lon)
                for lat, lon in zip(
                    self.latitudes,
                    self.longitudes
                )
            ]

        self.create_subscription(
            State,
            str(
                self.get_parameter(
                    'state_topic'
                ).value
            ),
            self._state_callback,
            10
        )

        self.create_subscription(
            NavSatFix,
            str(
                self.get_parameter(
                    'gps_topic'
                ).value
            ),
            self._gps_callback,
            qos_profile_sensor_data
        )

        self.rtl_client = self.create_client(
            Trigger,
            str(
                self.get_parameter(
                    'rtl_service'
                ).value
            )
        )

        rate = max(
            1.0,
            float(
                self.get_parameter(
                    'check_rate'
                ).value
            )
        )

        self.timer = self.create_timer(
            1.0 / rate,
            self._tick
        )

        if not self.enabled:
            self.get_logger().warn(
                'UAV geofence loaded but DISABLED.'
            )

        elif not self.configured:
            self.get_logger().error(
                'UAV geofence enabled but polygon '
                'configuration is invalid.'
            )

        else:
            self.get_logger().warn(
                f'UAV geofence ACTIVE with '
                f'{len(self.latitudes)} vertices.'
            )

    def _validate_polygon(self):
        if len(self.latitudes) < 3:
            return False

        if len(self.latitudes) != len(
            self.longitudes
        ):
            return False

        for lat, lon in zip(
            self.latitudes,
            self.longitudes
        ):
            if not (
                math.isfinite(lat)
                and math.isfinite(lon)
                and -90.0 <= lat <= 90.0
                and -180.0 <= lon <= 180.0
            ):
                return False

        return True

    def _state_callback(self, msg):
        previously_armed = (
            self.state is not None
            and bool(self.state.armed)
        )

        self.state = msg

        # Reset the latch between flights.
        if previously_armed and not msg.armed:
            self.breach_count = 0
            self.rtl_requested = False
            self.rtl_pending = False

            self._log_status(
                'Geofence reset after disarm.',
                'info'
            )

    def _gps_callback(self, msg):
        self.gps = msg
        self.last_gps_time = (
            self.get_clock().now()
        )

    def _gps_is_valid(self):
        if (
            self.gps is None
            or self.last_gps_time is None
        ):
            return False

        age = (
            self.get_clock().now()
            - self.last_gps_time
        ).nanoseconds / 1e9

        if age > self.gps_timeout:
            return False

        if (
            int(self.gps.status.status)
            < int(NavSatStatus.STATUS_FIX)
        ):
            return False

        return (
            math.isfinite(self.gps.latitude)
            and math.isfinite(self.gps.longitude)
        )

    def _latlon_to_xy(self, latitude, longitude):
        lat = math.radians(latitude)
        lon = math.radians(longitude)

        x = (
            EARTH_RADIUS_M
            * (lon - self.reference_lon)
            * math.cos(self.reference_lat)
        )

        y = (
            EARTH_RADIUS_M
            * (lat - self.reference_lat)
        )

        return x, y

    def _inside_polygon(self, x, y):
        inside = False

        count = len(self.polygon_xy)
        j = count - 1

        for i in range(count):
            xi, yi = self.polygon_xy[i]
            xj, yj = self.polygon_xy[j]

            crosses = (
                (yi > y) != (yj > y)
            )

            if crosses:
                intersection_x = (
                    (xj - xi)
                    * (y - yi)
                    / (yj - yi)
                    + xi
                )

                if x < intersection_x:
                    inside = not inside

            j = i

        return inside

    @staticmethod
    def _distance_to_segment(
        px,
        py,
        ax,
        ay,
        bx,
        by
    ):
        dx = bx - ax
        dy = by - ay

        length_sq = (
            dx * dx
            + dy * dy
        )

        if length_sq <= 1e-12:
            return math.hypot(
                px - ax,
                py - ay
            )

        t = (
            (px - ax) * dx
            + (py - ay) * dy
        ) / length_sq

        t = max(
            0.0,
            min(1.0, t)
        )

        closest_x = ax + t * dx
        closest_y = ay + t * dy

        return math.hypot(
            px - closest_x,
            py - closest_y
        )

    def _distance_to_boundary(self, x, y):
        minimum = float('inf')

        count = len(self.polygon_xy)

        for i in range(count):
            ax, ay = self.polygon_xy[i]

            bx, by = self.polygon_xy[
                (i + 1) % count
            ]

            minimum = min(
                minimum,
                self._distance_to_segment(
                    x,
                    y,
                    ax,
                    ay,
                    bx,
                    by
                )
            )

        return minimum

    def _request_rtl(self):
        if self.rtl_requested:
            return

        if self.rtl_pending:
            return

        if (
            self.state is None
            or not self.state.connected
        ):
            self._log_status(
                'GEOFENCE BREACH: MAVROS disconnected; '
                'ROS cannot request RTL.',
                'error'
            )
            return

        if not self.state.armed:
            self._log_status(
                'Outside geofence while disarmed; '
                'RTL not requested.',
                'warn'
            )
            return

        if str(self.state.mode).upper() == 'RTL':
            self.rtl_requested = True

            self._log_status(
                'Geofence breached; vehicle already RTL.',
                'warn'
            )
            return

        if not self.rtl_client.service_is_ready():
            self._log_status(
                'GEOFENCE BREACH: /vehicle/rtl '
                'service unavailable.',
                'error'
            )
            return

        self.rtl_pending = True

        future = self.rtl_client.call_async(
            Trigger.Request()
        )

        future.add_done_callback(
            self._rtl_done
        )

        self.get_logger().error(
            'GEOFENCE BREACH CONFIRMED: '
            'requesting RTL.'
        )

    def _rtl_done(self, future):
        self.rtl_pending = False

        try:
            response = future.result()

        except Exception as exc:
            self.get_logger().error(
                f'RTL service failure: {exc}'
            )
            return

        if response.success:
            self.rtl_requested = True

            self.get_logger().error(
                'RTL accepted due to '
                'geofence breach.'
            )

        else:
            self.get_logger().error(
                'RTL rejected after geofence '
                f'breach: {response.message}'
            )

    def _log_status(
        self,
        message,
        level='info'
    ):
        if message == self.last_status:
            return

        self.last_status = message

        logger = self.get_logger()

        if level == 'error':
            logger.error(message)

        elif level == 'warn':
            logger.warn(message)

        else:
            logger.info(message)

    def _tick(self):
        if not self.enabled:
            return

        if not self.configured:
            return

        if not self._gps_is_valid():
            self.breach_count = 0

            self._log_status(
                'Geofence cannot evaluate: '
                'GPS invalid or stale.',
                'warn'
            )
            return

        x, y = self._latlon_to_xy(
            self.gps.latitude,
            self.gps.longitude
        )

        inside = self._inside_polygon(
            x,
            y
        )

        distance = (
            self._distance_to_boundary(
                x,
                y
            )
        )

        if inside:
            self.breach_count = 0

            if (
                distance
                <= self.warning_margin
            ):
                self._log_status(
                    'GEOFENCE WARNING: '
                    f'{distance:.1f} m from boundary.',
                    'warn'
                )

            else:
                self._log_status(
                    'Geofence OK: '
                    f'{distance:.1f} m from boundary.',
                    'info'
                )

            return

        self.breach_count += 1

        self._log_status(
            'GEOFENCE BREACH: '
            f'outside boundary '
            f'({self.breach_count}/'
            f'{self.breach_confirmations}).',
            'error'
        )

        if (
            self.breach_count
            >= self.breach_confirmations
        ):
            self._request_rtl()


def main(args=None):
    rclpy.init(args=args)

    node = UAVGeofence()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
