#!/usr/bin/env python3

import math
import os
import threading
import time

# Fence, rally, and other mission-type transfers use MAVLink 2
# extension fields. Direct GCS management links must therefore
# always encode MAVLink 2 packets.
os.environ.setdefault("MAVLINK20", "1")

from pymavlink import mavutil

from robotx_dashboard.clients.base_client import BaseVehicleClient


class BoatMavlinkClient(BaseVehicleClient):
    """
    Direct Beeptop <-> BlueOS/ArduPilot MAVLink GCS transport.

    BlueOS sends MAVLink to Beeptop UDP 14550.

    This transport is permitted to perform GCS-management operations:
      - telemetry
      - parameter read/write
      - servo-output monitoring
      - later: missions and fences

    Propulsion/autonomy commands remain on the guarded Jetson path.
    """

    PARAM_TYPE_NAMES = {
        1: "UINT8",
        2: "INT8",
        3: "UINT16",
        4: "INT16",
        5: "UINT32",
        6: "INT32",
        7: "UINT64",
        8: "INT64",
        9: "REAL32",
        10: "REAL64",
    }

    SERVO_FUNCTION_NAMES = {
        0: "Disabled",
        26: "Ground Steering",
        70: "Throttle",
        73: "Port Thruster",
        74: "Starboard Thruster",
    }

    def __init__(
        self,
        vehicle_id,
        state_update_callback,
        link_offline_callback=None,
        endpoint="udpin:0.0.0.0:14550",
        rx_timeout=3.0,
    ):
        super().__init__(
            vehicle_id,
            state_update_callback,
        )

        self.endpoint = endpoint
        self.rx_timeout = float(rx_timeout)

        self.link_offline_callback = (
            link_offline_callback
        )

        self.stop_event = threading.Event()
        self.thread = None
        self.connection = None

        self.send_lock = threading.RLock()

        self.target_system = None
        self.target_component = (
            mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1
        )

        self.last_rx = None
        self.armed = False

        self.management_requested = False

        # ----------------------------------------------------
        # Parameter cache
        # ----------------------------------------------------

        self.param_condition = threading.Condition(
            threading.RLock()
        )

        self.param_cache = {}
        self.param_versions = {}
        self.param_expected = None

        self.param_refresh_active = False
        self.param_refresh_seen = set()

        # ----------------------------------------------------
        # Native ArduPilot geofence cache
        # ----------------------------------------------------

        self.fence_condition = threading.Condition(
            threading.RLock()
        )

        self.fence_download_active = False
        self.fence_upload_active = False
        self.fence_expected = None
        self.fence_items = {}
        self.fence_upload_items = {}
        self.fence_last_ack = None

    # ========================================================
    # LIFECYCLE
    # ========================================================

    def start(self):
        if (
            self.thread is not None
            and self.thread.is_alive()
        ):
            return

        self.stop_event.clear()

        self.thread = threading.Thread(
            target=self._run,
            name="boat-direct-mavlink",
            daemon=True,
        )

        self.thread.start()

    def stop(self):
        self.stop_event.set()

        connection = self.connection

        if connection is not None:
            try:
                connection.close()
            except Exception:
                pass

        if self.thread is not None:
            self.thread.join(timeout=2.0)

        self.connected = False

    def command(self, command, data=None):
        return {
            "success": False,
            "message": (
                "Direct USV MAVLink is a GCS-management "
                "transport. Propulsion/autonomy commands "
                "use the guarded Jetson transport."
            ),
        }

    # ========================================================
    # PUBLIC PARAMETER API
    # ========================================================

    def parameter_snapshot(self):
        with self.param_condition:

            parameters = []

            for name in sorted(self.param_cache):

                entry = self.param_cache[name]

                parameters.append({
                    "name": name,
                    "value": entry["value"],
                    "type": entry["type"],
                    "type_name": self.PARAM_TYPE_NAMES.get(
                        entry["type"],
                        f"TYPE_{entry['type']}",
                    ),
                    "index": entry["index"],
                })

            expected = self.param_expected

        available = bool(
            self.connected
            and self.target_system is not None
        )

        return {
            "available": available,
            "write_allowed": bool(
                available and not self.armed
            ),
            "armed": bool(self.armed),
            "count": len(parameters),
            "expected": expected,
            "complete": bool(
                expected is not None
                and len(parameters) >= expected
            ),
            "parameters": parameters,
        }

    def refresh_parameters(
        self,
        timeout=90.0,
    ):
        """
        Synchronize the FCU parameter cache without flooding a
        slow MAVLink transport.

        Important for the UAV:

            Pixhawk -> companion MAVROS -> Beeptop

        includes a 57600-baud serial hop. A 1023-parameter
        transfer can remain active for tens of seconds.

        Strategy:

          1. Preserve the persistent cache.
          2. If a bulk transfer is already progressing, let it
             finish.
          3. Consider the stream stalled only after five
             seconds without any new indexed parameter.
          4. Recover remaining gaps one index at a time.
          5. Never start competing bulk transfers during gap
             recovery.
        """

        if (
            not self.connected
            or self.connection is None
            or self.target_system is None
        ):
            return {
                "success": False,
                "message": (
                    "Autopilot parameter backend offline"
                ),
                "count": len(self.param_cache),
                "expected": self.param_expected,
                "complete": False,
                "missing_count": None,
            }


        deadline = (
            time.monotonic()
            + float(timeout)
        )


        # ----------------------------------------------------
        # Cache inspection
        # ----------------------------------------------------

        def cache_state():

            with self.param_condition:

                expected = (
                    int(self.param_expected)
                    if self.param_expected is not None
                    else None
                )

                present = set()

                for item in self.param_cache.values():

                    try:
                        index = int(
                            item.get("index")
                        )

                    except (
                        TypeError,
                        ValueError,
                        AttributeError,
                    ):
                        continue

                    if index >= 0:
                        present.add(index)


                if (
                    expected is None
                    or expected <= 0
                ):
                    missing = []

                else:
                    missing = [
                        index
                        for index in range(expected)
                        if index not in present
                    ]


                return (
                    expected,
                    present,
                    missing,
                )


        def result():

            expected, present, missing = (
                cache_state()
            )

            count = len(present)

            complete = bool(
                expected is not None
                and expected > 0
                and not missing
                and count >= expected
            )

            if complete:

                message = (
                    "Parameter refresh complete: "
                    f"{count}/{expected}"
                )

            else:

                message = (
                    "Parameter refresh ended: "
                    f"{count}/"
                    f"{expected if expected is not None else '?'}"
                )

                if missing:
                    message += (
                        f" ({len(missing)} missing)"
                    )


            return {
                "success": complete,
                "message": message,
                "count": count,
                "expected": expected,
                "complete": complete,
                "missing_count": len(missing),
            }


        def complete_now():

            expected, present, missing = (
                cache_state()
            )

            return bool(
                expected is not None
                and expected > 0
                and len(present) >= expected
                and not missing
            )


        # ----------------------------------------------------
        # Wait while a bulk stream is actively progressing.
        # ----------------------------------------------------

        def wait_for_stream_to_settle(
            max_duration,
        ):

            local_deadline = min(
                deadline,
                time.monotonic()
                + float(max_duration),
            )

            expected, present, missing = (
                cache_state()
            )

            last_count = len(present)
            last_progress = time.monotonic()


            while (
                time.monotonic()
                < local_deadline
            ):

                if complete_now():
                    return True


                with self.param_condition:

                    remaining = (
                        local_deadline
                        - time.monotonic()
                    )

                    if remaining <= 0.0:
                        break

                    self.param_condition.wait(
                        timeout=min(
                            0.5,
                            remaining,
                        )
                    )


                expected, present, missing = (
                    cache_state()
                )

                count = len(present)


                if count > last_count:

                    last_count = count
                    last_progress = (
                        time.monotonic()
                    )


                #
                # No new indexed parameter for five seconds:
                # bulk stream is now considered stalled.
                #
                if (
                    time.monotonic()
                    - last_progress
                    >= 5.0
                ):
                    break


            return complete_now()


        # ----------------------------------------------------
        # Mark refresh active
        # ----------------------------------------------------

        with self.param_condition:

            self.param_refresh_active = True

            self.param_refresh_seen.clear()


        try:

            expected, present, missing = (
                cache_state()
            )


            # ------------------------------------------------
            # If cache is already complete, no reason to force
            # another 1000-message transfer just to establish
            # completeness.
            # ------------------------------------------------

            if complete_now():
                return result()


            # ------------------------------------------------
            # Existing startup transfer may already be active.
            #
            # If we have any parameter data, first let that
            # transfer continue without transmitting anything.
            # ------------------------------------------------

            if present:

                if wait_for_stream_to_settle(
                    max_duration=60.0
                ):
                    return result()


            # ------------------------------------------------
            # If no list has started, request ONE bulk transfer.
            # ------------------------------------------------

            expected, present, missing = (
                cache_state()
            )

            if (
                expected is None
                or expected <= 0
                or not present
            ):

                if not self._send_parameter_list_request():

                    return {
                        "success": False,
                        "message": (
                            "Failed to request parameter list"
                        ),
                        "count": len(self.param_cache),
                        "expected": self.param_expected,
                        "complete": False,
                        "missing_count": None,
                    }


                if wait_for_stream_to_settle(
                    max_duration=60.0
                ):
                    return result()


            # ------------------------------------------------
            # Targeted gap recovery.
            #
            # Only one missing index is requested at a time.
            # Wait for that exact index before requesting the
            # next one.
            # ------------------------------------------------

            recovery_pass = 0

            while (
                time.monotonic()
                < deadline
            ):

                expected, present, missing = (
                    cache_state()
                )


                if (
                    expected is not None
                    and expected > 0
                    and not missing
                ):
                    return result()


                if (
                    expected is None
                    or expected <= 0
                ):
                    break


                recovery_pass += 1


                for index in missing:

                    if (
                        time.monotonic()
                        >= deadline
                    ):
                        break


                    # It may have arrived while processing an
                    # earlier recovery request.
                    expected_now, present_now, _ = (
                        cache_state()
                    )

                    if index in present_now:
                        continue


                    if not self._send_parameter_index_request(
                        index
                    ):
                        continue


                    #
                    # Wait up to 1.25 s for this exact index.
                    #
                    index_deadline = min(
                        deadline,
                        time.monotonic() + 1.25,
                    )


                    while (
                        time.monotonic()
                        < index_deadline
                    ):

                        _, present_now, _ = (
                            cache_state()
                        )

                        if index in present_now:
                            break


                        remaining = (
                            index_deadline
                            - time.monotonic()
                        )

                        if remaining <= 0.0:
                            break


                        with self.param_condition:

                            self.param_condition.wait(
                                timeout=min(
                                    0.20,
                                    remaining,
                                )
                            )


                if complete_now():
                    return result()


                #
                # Allow any responses still crossing the serial
                # link to drain before another recovery pass.
                #
                drain_deadline = min(
                    deadline,
                    time.monotonic() + 2.0,
                )

                while (
                    time.monotonic()
                    < drain_deadline
                ):

                    if complete_now():
                        return result()

                    with self.param_condition:

                        remaining = (
                            drain_deadline
                            - time.monotonic()
                        )

                        if remaining <= 0.0:
                            break

                        self.param_condition.wait(
                            timeout=min(
                                0.25,
                                remaining,
                            )
                        )


                #
                # Three serialized passes is enough. If a
                # parameter still cannot be retrieved after
                # that, report the real missing count.
                #
                if recovery_pass >= 3:
                    break


            return result()


        finally:

            with self.param_condition:

                self.param_refresh_active = False

                self.param_condition.notify_all()


    def set_parameter(
        self,
        name,
        value,
        timeout=4.0,
    ):
        name = str(name).strip().upper()

        if not name:
            return {
                "success": False,
                "message": "Parameter name is empty",
            }

        if not self.connected:
            return {
                "success": False,
                "message": "USV MAVLink is offline",
            }

        if self.armed:
            return {
                "success": False,
                "message": (
                    "Parameter write rejected: "
                    "vehicle is ARMED"
                ),
            }

        try:
            requested_value = float(value)
        except (TypeError, ValueError):
            return {
                "success": False,
                "message": (
                    "Parameter value must be numeric"
                ),
            }

        if not math.isfinite(requested_value):
            return {
                "success": False,
                "message": (
                    "Parameter value must be finite"
                ),
            }

        with self.param_condition:

            existing = self.param_cache.get(name)

            if existing is None:
                return {
                    "success": False,
                    "message": (
                        f"Unknown parameter: {name}. "
                        "Refresh parameters first."
                    ),
                }

            param_type = int(
                existing["type"]
            )

            previous_version = int(
                self.param_versions.get(
                    name,
                    0,
                )
            )

        if not self._send_parameter_set(
            name,
            requested_value,
            param_type,
        ):
            return {
                "success": False,
                "message": (
                    f"Could not send PARAM_SET "
                    f"for {name}"
                ),
            }

        deadline = (
            time.monotonic()
            + float(timeout)
        )

        read_request_sent = False
        start = time.monotonic()

        while time.monotonic() < deadline:

            with self.param_condition:

                current = (
                    self.param_cache.get(name)
                )

                current_version = int(
                    self.param_versions.get(
                        name,
                        0,
                    )
                )

                if (
                    current is not None
                    and
                    current_version
                    > previous_version
                ):
                    actual = float(
                        current["value"]
                    )

                    if math.isclose(
                        actual,
                        requested_value,
                        rel_tol=1.0e-6,
                        abs_tol=1.0e-5,
                    ):
                        return {
                            "success": True,
                            "message": (
                                f"{name} verified "
                                f"at {actual}"
                            ),
                            "name": name,
                            "requested_value":
                                requested_value,
                            "value": actual,
                            "type": int(
                                current["type"]
                            ),
                        }

                remaining = (
                    deadline
                    - time.monotonic()
                )

                if remaining <= 0.0:
                    break

                self.param_condition.wait(
                    timeout=min(
                        0.25,
                        remaining,
                    )
                )

            # If PARAM_SET did not produce a matching
            # PARAM_VALUE quickly, explicitly request
            # read-back while continuing to wait.
            if (
                not read_request_sent
                and
                time.monotonic() - start
                >= 0.75
            ):
                self._send_parameter_read_request(
                    name
                )

                read_request_sent = True

        with self.param_condition:
            current = self.param_cache.get(name)

        return {
            "success": False,
            "message": (
                f"{name} write was not verified "
                f"before timeout"
            ),
            "name": name,
            "requested_value": requested_value,
            "value": (
                None
                if current is None
                else current["value"]
            ),
        }

    # ========================================================
    # NATIVE ARDUPILOT GEOFENCE
    # ========================================================

    def fence_snapshot(self):
        """
        Return the cached native ArduPilot fence as both raw
        MAVLink mission items and editable logical zones.
        """

        with self.fence_condition:

            expected = self.fence_expected

            raw_items = [
                dict(self.fence_items[seq])
                for seq in sorted(
                    self.fence_items
                )
            ]


        polygon_in = int(
            mavutil.mavlink
            .MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION
        )

        polygon_out = int(
            mavutil.mavlink
            .MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION
        )

        circle_in = int(
            mavutil.mavlink
            .MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION
        )

        circle_out = int(
            mavutil.mavlink
            .MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION
        )


        zones = []
        unknown_items = []

        i = 0

        while i < len(raw_items):

            item = raw_items[i]

            command = int(
                item["command"]
            )


            # -----------------------------------------------
            # Polygon zone
            # -----------------------------------------------

            if command in {
                polygon_in,
                polygon_out,
            }:

                vertex_count = int(
                    round(
                        float(
                            item.get(
                                "param1",
                                0.0,
                            )
                        )
                    )
                )

                points = []
                valid = (
                    vertex_count >= 3
                )


                if valid:

                    for offset in range(
                        vertex_count
                    ):

                        position = i + offset

                        if position >= len(
                            raw_items
                        ):
                            valid = False
                            break

                        candidate = (
                            raw_items[position]
                        )

                        if (
                            int(
                                candidate["command"]
                            )
                            != command
                        ):
                            valid = False
                            break

                        if (
                            int(
                                round(
                                    float(
                                        candidate.get(
                                            "param1",
                                            0.0,
                                        )
                                    )
                                )
                            )
                            != vertex_count
                        ):
                            valid = False
                            break

                        points.append({
                            "latitude": float(
                                candidate[
                                    "latitude"
                                ]
                            ),
                            "longitude": float(
                                candidate[
                                    "longitude"
                                ]
                            ),
                        })


                if valid:

                    zones.append({
                        "type": (
                            "inclusion_polygon"
                            if command
                            == polygon_in
                            else
                            "exclusion_polygon"
                        ),
                        "points": points,
                    })

                    i += vertex_count

                    continue


                unknown_items.append(
                    dict(item)
                )

                i += 1

                continue


            # -----------------------------------------------
            # Circle zone
            # -----------------------------------------------

            if command in {
                circle_in,
                circle_out,
            }:

                zones.append({
                    "type": (
                        "inclusion_circle"
                        if command
                        == circle_in
                        else
                        "exclusion_circle"
                    ),
                    "center": {
                        "latitude": float(
                            item["latitude"]
                        ),
                        "longitude": float(
                            item["longitude"]
                        ),
                    },
                    "radius_m": float(
                        item.get(
                            "param1",
                            0.0,
                        )
                    ),
                })

                i += 1

                continue


            unknown_items.append(
                dict(item)
            )

            i += 1


        # Legacy flattened fields are retained for compatibility
        # with existing dashboard/RoboCommand code.
        inclusion_vertices = []
        exclusion_vertices = []
        circles = []


        for zone in zones:

            if (
                zone["type"]
                == "inclusion_polygon"
            ):

                for index, point in enumerate(
                    zone["points"]
                ):

                    inclusion_vertices.append({
                        "seq": index,
                        "polygon_vertex_count":
                            len(zone["points"]),
                        "latitude":
                            point["latitude"],
                        "longitude":
                            point["longitude"],
                    })


            elif (
                zone["type"]
                == "exclusion_polygon"
            ):

                for index, point in enumerate(
                    zone["points"]
                ):

                    exclusion_vertices.append({
                        "seq": index,
                        "polygon_vertex_count":
                            len(zone["points"]),
                        "latitude":
                            point["latitude"],
                        "longitude":
                            point["longitude"],
                    })


            elif zone["type"] in {
                "inclusion_circle",
                "exclusion_circle",
            }:

                circles.append({
                    "type": (
                        "inclusion"
                        if zone["type"]
                        == "inclusion_circle"
                        else "exclusion"
                    ),
                    "latitude":
                        zone["center"][
                            "latitude"
                        ],
                    "longitude":
                        zone["center"][
                            "longitude"
                        ],
                    "radius_m":
                        zone["radius_m"],
                })


        complete = bool(
            expected is not None
            and expected >= 0
            and len(raw_items)
            == expected
        )


        return {
            "available": bool(
                self.connected
                and self.target_system
                is not None
            ),
            "armed": bool(
                self.armed
            ),
            "count": len(
                raw_items
            ),
            "expected": expected,
            "complete": complete,

            "zones": zones,

            "inclusion_vertices":
                inclusion_vertices,

            "exclusion_vertices":
                exclusion_vertices,

            "circles": circles,

            "unknown_items":
                unknown_items,

            "items":
                raw_items,
        }


    def download_fence(
        self,
        timeout=20.0,
    ):
        """
        Download the native ArduPilot fence using the MAVLink
        mission protocol with MAV_MISSION_TYPE_FENCE.
        """

        if (
            not self.connected
            or self.connection is None
            or self.target_system is None
        ):
            result = self.fence_snapshot()

            result.update({
                "success": False,
                "message": (
                    "Autopilot geofence backend offline"
                ),
            })

            return result


        fence_type = int(
            mavutil.mavlink
            .MAV_MISSION_TYPE_FENCE
        )

        deadline = (
            time.monotonic()
            + float(timeout)
        )


        with self.fence_condition:

            self.fence_download_active = True
            self.fence_expected = None
            self.fence_items = {}
            self.fence_last_ack = None


        try:

            # -----------------------------------------------
            # Ask the FCU how many fence items are stored.
            # -----------------------------------------------

            if not self._send_with_connection(
                lambda connection, system, component:
                    connection.mav
                    .mission_request_list_send(
                        system,
                        component,
                        fence_type,
                    )
            ):
                result = self.fence_snapshot()

                result.update({
                    "success": False,
                    "message": (
                        "Failed to request geofence list"
                    ),
                })

                return result


            with self.fence_condition:

                while (
                    self.fence_expected is None
                    and time.monotonic()
                    < deadline
                ):

                    remaining = (
                        deadline
                        - time.monotonic()
                    )

                    if remaining <= 0.0:
                        break

                    self.fence_condition.wait(
                        timeout=min(
                            0.5,
                            remaining,
                        )
                    )


                expected = (
                    self.fence_expected
                )


            if expected is None:

                result = self.fence_snapshot()

                result.update({
                    "success": False,
                    "message": (
                        "Timed out waiting for "
                        "MISSION_COUNT for geofence"
                    ),
                })

                return result


            # Empty native fence is valid.
            if expected == 0:

                result = self.fence_snapshot()

                result.update({
                    "success": True,
                    "message": (
                        "Native geofence is empty"
                    ),
                })

                return result


            # -----------------------------------------------
            # Retrieve each fence item deterministically.
            # -----------------------------------------------

            for seq in range(
                int(expected)
            ):

                received = False


                for attempt in range(3):

                    if (
                        time.monotonic()
                        >= deadline
                    ):
                        break


                    with self.fence_condition:

                        if seq in self.fence_items:
                            received = True
                            break


                    sent = self._send_with_connection(
                        lambda connection,
                               system,
                               component,
                               seq=seq:
                            connection.mav
                            .mission_request_int_send(
                                system,
                                component,
                                int(seq),
                                fence_type,
                            )
                    )


                    if not sent:
                        continue


                    item_deadline = min(
                        deadline,
                        time.monotonic()
                        + 2.0,
                    )


                    with self.fence_condition:

                        while (
                            seq
                            not in self.fence_items
                            and time.monotonic()
                            < item_deadline
                        ):

                            remaining = (
                                item_deadline
                                - time.monotonic()
                            )

                            if remaining <= 0.0:
                                break

                            self.fence_condition.wait(
                                timeout=min(
                                    0.25,
                                    remaining,
                                )
                            )


                        if seq in self.fence_items:
                            received = True
                            break


                if not received:

                    result = self.fence_snapshot()

                    result.update({
                        "success": False,
                        "message": (
                            "Geofence download stopped: "
                            f"missing item {seq}"
                        ),
                    })

                    return result


            # Download complete. Acknowledge receipt.
            self._send_with_connection(
                lambda connection, system, component:
                    connection.mav
                    .mission_ack_send(
                        system,
                        component,
                        mavutil.mavlink
                        .MAV_MISSION_ACCEPTED,
                        fence_type,
                    )
            )


            result = self.fence_snapshot()

            result.update({
                "success": bool(
                    result["complete"]
                ),
                "message": (
                    "Native geofence downloaded: "
                    f"{result['count']}/"
                    f"{result['expected']} items"
                ),
            })

            return result


        finally:

            with self.fence_condition:

                self.fence_download_active = False

                self.fence_condition.notify_all()


    def upload_fence_zones(
        self,
        zones,
        timeout=30.0,
    ):
        """
        Replace the complete native ArduPilot fence mission
        with a list of logical zones.

        Supported zone types:
          - inclusion_polygon
          - exclusion_polygon
          - inclusion_circle
          - exclusion_circle

        After upload, the complete fence is downloaded again
        and compared against the requested geometry.

        This DOES NOT modify FENCE_ENABLE.
        """

        if (
            not self.connected
            or self.connection is None
            or self.target_system is None
        ):
            return {
                "success": False,
                "message": (
                    "Autopilot geofence backend offline"
                ),
            }


        if self.armed:
            return {
                "success": False,
                "message": (
                    "Geofence writes are blocked while armed"
                ),
            }


        if not isinstance(zones, list):
            return {
                "success": False,
                "message": (
                    "zones must be a list"
                ),
            }


        fence_type = int(
            mavutil.mavlink
            .MAV_MISSION_TYPE_FENCE
        )

        frame = int(
            mavutil.mavlink
            .MAV_FRAME_GLOBAL_INT
        )

        polygon_in = int(
            mavutil.mavlink
            .MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION
        )

        polygon_out = int(
            mavutil.mavlink
            .MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION
        )

        circle_in = int(
            mavutil.mavlink
            .MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION
        )

        circle_out = int(
            mavutil.mavlink
            .MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION
        )


        def clean_coordinate(
            latitude,
            longitude,
        ):

            latitude = float(latitude)
            longitude = float(longitude)

            if (
                not math.isfinite(latitude)
                or not math.isfinite(longitude)
                or latitude < -90.0
                or latitude > 90.0
                or longitude < -180.0
                or longitude > 180.0
            ):
                raise ValueError(
                    "latitude/longitude outside valid range"
                )

            return latitude, longitude


        normalized = []


        try:

            for zone_number, zone in enumerate(
                zones,
                start=1,
            ):

                if not isinstance(zone, dict):
                    raise ValueError(
                        f"Zone {zone_number} must be an object"
                    )


                zone_type = str(
                    zone.get(
                        "type",
                        "",
                    )
                ).strip()


                # ==========================================
                # POLYGON
                # ==========================================

                if zone_type in {
                    "inclusion_polygon",
                    "exclusion_polygon",
                }:

                    raw_points = zone.get(
                        "points"
                    )

                    if not isinstance(
                        raw_points,
                        list,
                    ):
                        raise ValueError(
                            f"Zone {zone_number}: "
                            "points must be a list"
                        )


                    points = []

                    for point_number, point in enumerate(
                        raw_points,
                        start=1,
                    ):

                        if not isinstance(
                            point,
                            dict,
                        ):
                            raise ValueError(
                                f"Zone {zone_number}, "
                                f"vertex {point_number}: "
                                "invalid point"
                            )


                        latitude, longitude = (
                            clean_coordinate(
                                point["latitude"],
                                point["longitude"],
                            )
                        )


                        points.append({
                            "latitude": latitude,
                            "longitude": longitude,
                        })


                    # UI may explicitly close the polygon.
                    if (
                        len(points) >= 4
                        and math.isclose(
                            points[0]["latitude"],
                            points[-1]["latitude"],
                            abs_tol=1.0e-9,
                        )
                        and math.isclose(
                            points[0]["longitude"],
                            points[-1]["longitude"],
                            abs_tol=1.0e-9,
                        )
                    ):
                        points.pop()


                    if len(points) < 3:
                        raise ValueError(
                            f"Zone {zone_number}: "
                            "polygon requires at least "
                            "3 unique vertices"
                        )


                    normalized.append({
                        "type": zone_type,
                        "points": points,
                    })

                    continue


                # ==========================================
                # CIRCLE
                # ==========================================

                if zone_type in {
                    "inclusion_circle",
                    "exclusion_circle",
                }:

                    center = zone.get(
                        "center"
                    )

                    if not isinstance(
                        center,
                        dict,
                    ):
                        raise ValueError(
                            f"Zone {zone_number}: "
                            "circle center is missing"
                        )


                    latitude, longitude = (
                        clean_coordinate(
                            center["latitude"],
                            center["longitude"],
                        )
                    )


                    radius = float(
                        zone.get(
                            "radius_m",
                            0.0,
                        )
                    )


                    if (
                        not math.isfinite(radius)
                        or radius <= 0.0
                    ):
                        raise ValueError(
                            f"Zone {zone_number}: "
                            "circle radius must be > 0"
                        )


                    normalized.append({
                        "type": zone_type,
                        "center": {
                            "latitude": latitude,
                            "longitude": longitude,
                        },
                        "radius_m": radius,
                    })

                    continue


                raise ValueError(
                    f"Zone {zone_number}: "
                    f"unsupported type {zone_type!r}"
                )


        except (
            KeyError,
            TypeError,
            ValueError,
        ) as exc:

            return {
                "success": False,
                "message": str(exc),
            }


        # ==============================================
        # FLATTEN LOGICAL ZONES INTO MAVLINK ITEMS
        # ==============================================

        upload_items = {}

        seq = 0


        for zone in normalized:

            zone_type = zone["type"]


            if zone_type in {
                "inclusion_polygon",
                "exclusion_polygon",
            }:

                command = (
                    polygon_in
                    if zone_type
                    == "inclusion_polygon"
                    else polygon_out
                )

                points = zone["points"]

                vertex_count = len(
                    points
                )


                for point in points:

                    upload_items[seq] = {
                        "seq": seq,
                        "frame": frame,
                        "command": command,
                        "current": 0,
                        "autocontinue": 0,

                        # For polygon fence items,
                        # param1 is vertex count.
                        "param1":
                            float(vertex_count),

                        "param2": 0.0,
                        "param3": 0.0,
                        "param4": 0.0,

                        "x": int(round(
                            point["latitude"]
                            * 1.0e7
                        )),

                        "y": int(round(
                            point["longitude"]
                            * 1.0e7
                        )),

                        "z": 0.0,
                    }

                    seq += 1


            else:

                command = (
                    circle_in
                    if zone_type
                    == "inclusion_circle"
                    else circle_out
                )

                center = zone[
                    "center"
                ]

                upload_items[seq] = {
                    "seq": seq,
                    "frame": frame,
                    "command": command,
                    "current": 0,
                    "autocontinue": 0,

                    # For circle fence items,
                    # param1 is radius in meters.
                    "param1":
                        float(
                            zone["radius_m"]
                        ),

                    "param2": 0.0,
                    "param3": 0.0,
                    "param4": 0.0,

                    "x": int(round(
                        center["latitude"]
                        * 1.0e7
                    )),

                    "y": int(round(
                        center["longitude"]
                        * 1.0e7
                    )),

                    "z": 0.0,
                }

                seq += 1


        count = len(
            upload_items
        )


        # ==============================================
        # EMPTY LIST = CLEAR NATIVE FENCE
        # ==============================================

        if count == 0:

            with self.fence_condition:

                if (
                    self.fence_upload_active
                    or self.fence_download_active
                ):
                    return {
                        "success": False,
                        "message": (
                            "Another geofence transfer "
                            "is already active"
                        ),
                    }

                self.fence_upload_active = True
                self.fence_upload_items = {}
                self.fence_last_ack = None


            try:

                sent = self._send_with_connection(
                    lambda connection,
                           system,
                           component:
                        connection.mav
                        .mission_clear_all_send(
                            system,
                            component,
                            fence_type,
                        )
                )


                if not sent:
                    return {
                        "success": False,
                        "message": (
                            "Failed to send fence clear request"
                        ),
                    }


                deadline = (
                    time.monotonic()
                    + float(timeout)
                )


                with self.fence_condition:

                    while (
                        self.fence_last_ack is None
                        and time.monotonic()
                        < deadline
                    ):

                        remaining = (
                            deadline
                            - time.monotonic()
                        )

                        if remaining <= 0.0:
                            break

                        self.fence_condition.wait(
                            timeout=min(
                                0.5,
                                remaining,
                            )
                        )


                    ack = self.fence_last_ack


                if ack is None:
                    return {
                        "success": False,
                        "message": (
                            "Timed out waiting for "
                            "fence-clear ACK"
                        ),
                    }


                if int(ack) != int(
                    mavutil.mavlink
                    .MAV_MISSION_ACCEPTED
                ):
                    return {
                        "success": False,
                        "message": (
                            "Autopilot rejected "
                            "fence clear "
                            f"(MAV_MISSION_RESULT={ack})"
                        ),
                    }


            finally:

                with self.fence_condition:

                    self.fence_upload_active = False
                    self.fence_upload_items = {}

                    self.fence_condition.notify_all()


            readback = self.download_fence(
                timeout=15.0
            )


            if (
                not readback.get(
                    "success",
                    False,
                )
                or not readback.get(
                    "complete",
                    False,
                )
                or int(
                    readback.get(
                        "count",
                        -1,
                    )
                ) != 0
            ):
                return {
                    "success": False,
                    "message": (
                        "Fence clear ACK received, "
                        "but empty read-back "
                        "verification failed"
                    ),
                    "readback": readback,
                }


            return {
                "success": True,
                "message": (
                    "Native geofence cleared "
                    "and verified."
                ),
                "zones": [],
                "readback": readback,
            }


        # ==============================================
        # BEGIN NORMAL MISSION UPLOAD
        # ==============================================

        with self.fence_condition:

            if (
                self.fence_upload_active
                or self.fence_download_active
            ):
                return {
                    "success": False,
                    "message": (
                        "Another geofence transfer "
                        "is already active"
                    ),
                }


            self.fence_upload_active = True

            self.fence_upload_items = (
                upload_items
            )

            self.fence_last_ack = None


        try:

            sent = self._send_with_connection(
                lambda connection,
                       system,
                       component:
                    connection.mav
                    .mission_count_send(
                        system,
                        component,
                        count,
                        fence_type,
                    )
            )


            if not sent:
                return {
                    "success": False,
                    "message": (
                        "Failed to start "
                        "geofence upload"
                    ),
                }


            deadline = (
                time.monotonic()
                + float(timeout)
            )


            with self.fence_condition:

                while (
                    self.fence_last_ack is None
                    and time.monotonic()
                    < deadline
                ):

                    remaining = (
                        deadline
                        - time.monotonic()
                    )

                    if remaining <= 0.0:
                        break


                    self.fence_condition.wait(
                        timeout=min(
                            0.5,
                            remaining,
                        )
                    )


                ack = self.fence_last_ack


            if ack is None:
                return {
                    "success": False,
                    "message": (
                        "Timed out waiting for "
                        "geofence upload ACK"
                    ),
                }


            if int(ack) != int(
                mavutil.mavlink
                .MAV_MISSION_ACCEPTED
            ):
                return {
                    "success": False,
                    "message": (
                        "Autopilot rejected "
                        "geofence upload "
                        f"(MAV_MISSION_RESULT={ack})"
                    ),
                    "ack": int(ack),
                }


        finally:

            with self.fence_condition:

                self.fence_upload_active = False

                self.fence_upload_items = {}

                self.fence_condition.notify_all()


        # ==============================================
        # DOWNLOAD COMPLETE FENCE AGAIN
        # ==============================================

        readback = self.download_fence(
            timeout=20.0
        )


        if not readback.get(
            "success",
            False,
        ):
            return {
                "success": False,
                "message": (
                    "Upload was accepted, "
                    "but fence read-back failed: "
                    + str(
                        readback.get(
                            "message",
                            "unknown error",
                        )
                    )
                ),
                "readback": readback,
            }


        if not readback.get(
            "complete",
            False,
        ):
            return {
                "success": False,
                "message": (
                    "Upload was accepted, "
                    "but fence read-back "
                    "was incomplete"
                ),
                "readback": readback,
            }


        returned_zones = (
            readback.get(
                "zones",
                []
            )
        )


        # ==============================================
        # COMPARE REQUESTED VS READ-BACK
        # ==============================================

        if len(returned_zones) != len(
            normalized
        ):
            return {
                "success": False,
                "message": (
                    "Fence upload was accepted, "
                    "but zone count does not match "
                    "read-back "
                    f"({len(normalized)} requested, "
                    f"{len(returned_zones)} returned)"
                ),
                "readback": readback,
            }


        coordinate_tolerance = (
            2.0e-7
        )

        radius_tolerance = (
            0.10
        )


        for index, (
            requested,
            returned,
        ) in enumerate(
            zip(
                normalized,
                returned_zones,
            ),
            start=1,
        ):

            if (
                requested["type"]
                != returned.get(
                    "type"
                )
            ):
                return {
                    "success": False,
                    "message": (
                        f"Zone {index} type "
                        "does not match read-back"
                    ),
                    "readback": readback,
                }


            if requested[
                "type"
            ].endswith(
                "_polygon"
            ):

                requested_points = (
                    requested["points"]
                )

                returned_points = (
                    returned.get(
                        "points",
                        []
                    )
                )


                if len(
                    requested_points
                ) != len(
                    returned_points
                ):
                    return {
                        "success": False,
                        "message": (
                            f"Zone {index} vertex count "
                            "does not match read-back"
                        ),
                        "readback": readback,
                    }


                for point_index, (
                    requested_point,
                    returned_point,
                ) in enumerate(
                    zip(
                        requested_points,
                        returned_points,
                    ),
                    start=1,
                ):

                    if (
                        not math.isclose(
                            float(
                                requested_point[
                                    "latitude"
                                ]
                            ),
                            float(
                                returned_point[
                                    "latitude"
                                ]
                            ),
                            abs_tol=(
                                coordinate_tolerance
                            ),
                        )
                        or
                        not math.isclose(
                            float(
                                requested_point[
                                    "longitude"
                                ]
                            ),
                            float(
                                returned_point[
                                    "longitude"
                                ]
                            ),
                            abs_tol=(
                                coordinate_tolerance
                            ),
                        )
                    ):
                        return {
                            "success": False,
                            "message": (
                                f"Zone {index}, "
                                f"vertex {point_index} "
                                "does not match read-back"
                            ),
                            "readback": readback,
                        }


            else:

                requested_center = (
                    requested["center"]
                )

                returned_center = (
                    returned.get(
                        "center",
                        {}
                    )
                )


                if (
                    not math.isclose(
                        float(
                            requested_center[
                                "latitude"
                            ]
                        ),
                        float(
                            returned_center.get(
                                "latitude",
                                999.0,
                            )
                        ),
                        abs_tol=(
                            coordinate_tolerance
                        ),
                    )
                    or
                    not math.isclose(
                        float(
                            requested_center[
                                "longitude"
                            ]
                        ),
                        float(
                            returned_center.get(
                                "longitude",
                                999.0,
                            )
                        ),
                        abs_tol=(
                            coordinate_tolerance
                        ),
                    )
                    or
                    not math.isclose(
                        float(
                            requested[
                                "radius_m"
                            ]
                        ),
                        float(
                            returned.get(
                                "radius_m",
                                -1.0,
                            )
                        ),
                        abs_tol=(
                            radius_tolerance
                        ),
                    )
                ):
                    return {
                        "success": False,
                        "message": (
                            f"Zone {index} circle "
                            "does not match read-back"
                        ),
                        "readback": readback,
                    }


        return {
            "success": True,
            "message": (
                "Native geofence verified: "
                f"{len(normalized)} zone(s), "
                f"{count} MAVLink item(s)."
            ),
            "zones": normalized,
            "readback": readback,
        }


    def upload_fence(
        self,
        vertices,
        timeout=25.0,
    ):
        """
        Replace the native ArduPilot fence with one inclusion
        polygon, then download it again and verify read-back.

        This function deliberately DOES NOT modify FENCE_ENABLE.
        Fence activation remains a separate operator action.
        """

        if (
            not self.connected
            or self.connection is None
            or self.target_system is None
        ):
            return {
                "success": False,
                "message": (
                    "Autopilot geofence backend offline"
                ),
            }


        if self.armed:
            return {
                "success": False,
                "message": (
                    "Geofence writes are blocked while armed"
                ),
            }


        if not isinstance(vertices, list):
            return {
                "success": False,
                "message": (
                    "vertices must be a list"
                ),
            }


        if len(vertices) < 3:
            return {
                "success": False,
                "message": (
                    "An inclusion polygon requires "
                    "at least 3 vertices"
                ),
            }


        cleaned = []

        for index, point in enumerate(vertices):

            if not isinstance(point, dict):
                return {
                    "success": False,
                    "message": (
                        f"Vertex {index} must be an object"
                    ),
                }

            try:
                latitude = float(
                    point["latitude"]
                )
                longitude = float(
                    point["longitude"]
                )

            except (
                KeyError,
                TypeError,
                ValueError,
            ):
                return {
                    "success": False,
                    "message": (
                        f"Vertex {index} has invalid "
                        "latitude/longitude"
                    ),
                }


            if (
                not math.isfinite(latitude)
                or not math.isfinite(longitude)
                or latitude < -90.0
                or latitude > 90.0
                or longitude < -180.0
                or longitude > 180.0
            ):
                return {
                    "success": False,
                    "message": (
                        f"Vertex {index} is outside "
                        "valid latitude/longitude range"
                    ),
                }


            cleaned.append({
                "latitude": latitude,
                "longitude": longitude,
            })


        # A UI may send the polygon explicitly closed.
        # MAVLink fence storage wants the unique vertices only.
        if (
            len(cleaned) >= 4
            and math.isclose(
                cleaned[0]["latitude"],
                cleaned[-1]["latitude"],
                abs_tol=1.0e-9,
            )
            and math.isclose(
                cleaned[0]["longitude"],
                cleaned[-1]["longitude"],
                abs_tol=1.0e-9,
            )
        ):
            cleaned.pop()


        if len(cleaned) < 3:
            return {
                "success": False,
                "message": (
                    "An inclusion polygon requires "
                    "at least 3 unique vertices"
                ),
            }


        fence_type = int(
            mavutil.mavlink
            .MAV_MISSION_TYPE_FENCE
        )

        command = int(
            mavutil.mavlink
            .MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION
        )

        frame = int(
            mavutil.mavlink
            .MAV_FRAME_GLOBAL_INT
        )

        count = len(cleaned)


        upload_items = {}

        for seq, point in enumerate(cleaned):

            upload_items[seq] = {
                "seq": seq,
                "frame": frame,
                "command": command,
                "current": 0,
                "autocontinue": 0,
                "param1": float(count),
                "param2": 0.0,
                "param3": 0.0,
                "param4": 0.0,
                "x": int(round(
                    point["latitude"]
                    * 1.0e7
                )),
                "y": int(round(
                    point["longitude"]
                    * 1.0e7
                )),
                "z": 0.0,
            }


        with self.fence_condition:

            if (
                self.fence_upload_active
                or self.fence_download_active
            ):
                return {
                    "success": False,
                    "message": (
                        "Another geofence transfer "
                        "is already active"
                    ),
                }

            self.fence_upload_active = True
            self.fence_upload_items = (
                upload_items
            )
            self.fence_last_ack = None


        try:

            sent = self._send_with_connection(
                lambda connection,
                       system,
                       component:
                    connection.mav
                    .mission_count_send(
                        system,
                        component,
                        count,
                        fence_type,
                    )
            )


            if not sent:
                return {
                    "success": False,
                    "message": (
                        "Failed to start geofence upload"
                    ),
                }


            deadline = (
                time.monotonic()
                + float(timeout)
            )


            with self.fence_condition:

                while (
                    self.fence_last_ack is None
                    and time.monotonic()
                    < deadline
                ):

                    remaining = (
                        deadline
                        - time.monotonic()
                    )

                    if remaining <= 0.0:
                        break

                    self.fence_condition.wait(
                        timeout=min(
                            0.5,
                            remaining,
                        )
                    )


                ack = self.fence_last_ack


            if ack is None:
                return {
                    "success": False,
                    "message": (
                        "Timed out waiting for "
                        "geofence upload acknowledgement"
                    ),
                }


            if (
                int(ack)
                != int(
                    mavutil.mavlink
                    .MAV_MISSION_ACCEPTED
                )
            ):
                return {
                    "success": False,
                    "message": (
                        "Autopilot rejected geofence "
                        f"upload with MAV_MISSION_RESULT "
                        f"{ack}"
                    ),
                    "ack": int(ack),
                }


        finally:

            with self.fence_condition:

                self.fence_upload_active = False
                self.fence_upload_items = {}

                self.fence_condition.notify_all()


        # ----------------------------------------------------
        # Mandatory read-back verification.
        # ----------------------------------------------------

        readback = self.download_fence(
            timeout=20.0
        )


        if not readback.get(
            "success",
            False,
        ):
            return {
                "success": False,
                "message": (
                    "Upload was accepted but "
                    "read-back failed: "
                    + str(
                        readback.get(
                            "message",
                            "unknown error",
                        )
                    )
                ),
                "readback": readback,
            }


        returned = (
            readback.get(
                "inclusion_vertices",
                []
            )
        )


        if len(returned) != len(cleaned):
            return {
                "success": False,
                "message": (
                    "Geofence read-back count mismatch: "
                    f"requested {len(cleaned)}, "
                    f"received {len(returned)}"
                ),
                "readback": readback,
            }


        tolerance = 2.0e-7

        for index, (
            requested,
            actual,
        ) in enumerate(
            zip(cleaned, returned)
        ):

            if (
                not math.isclose(
                    requested["latitude"],
                    float(actual["latitude"]),
                    abs_tol=tolerance,
                )
                or
                not math.isclose(
                    requested["longitude"],
                    float(actual["longitude"]),
                    abs_tol=tolerance,
                )
            ):
                return {
                    "success": False,
                    "message": (
                        "Geofence read-back mismatch "
                        f"at vertex {index}"
                    ),
                    "readback": readback,
                }


        return {
            "success": True,
            "message": (
                "Native geofence upload verified "
                f"({len(cleaned)} vertices). "
                "FENCE_ENABLE was not changed."
            ),
            "vertices": cleaned,
            "readback": readback,
        }


    # ========================================================
    # MAVLINK TX HELPERS
    # ========================================================

    def _send_with_connection(
        self,
        func,
    ):
        connection = self.connection
        system = self.target_system

        if (
            connection is None
            or system is None
        ):
            return False

        try:
            with self.send_lock:
                func(
                    connection,
                    int(system),
                    int(self.target_component),
                )

            return True

        except Exception:
            return False

    def _send_parameter_list_request(self):
        return self._send_with_connection(
            lambda connection, system, component:
                connection.mav
                .param_request_list_send(
                    system,
                    component,
                )
        )

    def _send_parameter_read_request(
        self,
        name,
    ):
        encoded = (
            str(name)
            .encode(
                "ascii",
                errors="ignore",
            )
        )

        return self._send_with_connection(
            lambda connection, system, component:
                connection.mav
                .param_request_read_send(
                    system,
                    component,
                    encoded,
                    -1,
                )
        )

    def _send_parameter_index_request(
        self,
        index,
    ):
        """
        Request one FCU parameter by MAVLink param_index.

        This is used to recover entries lost during a bulk
        PARAM_REQUEST_LIST transfer.
        """

        connection = self.connection

        if (
            connection is None
            or self.target_system is None
        ):
            return False

        try:

            with self.send_lock:

                connection.mav.param_request_read_send(
                    int(self.target_system),
                    int(self.target_component),
                    b"",
                    int(index),
                )

            return True

        except Exception:
            return False


    def _send_parameter_set(
        self,
        name,
        value,
        param_type,
    ):
        encoded = (
            str(name)
            .encode(
                "ascii",
                errors="ignore",
            )
        )

        return self._send_with_connection(
            lambda connection, system, component:
                connection.mav
                .param_set_send(
                    system,
                    component,
                    encoded,
                    float(value),
                    int(param_type),
                )
        )

    def _request_servo_output_stream(self):
        return self._send_with_connection(
            lambda connection, system, component:
                connection.mav
                .command_long_send(
                    system,
                    component,
                    mavutil.mavlink
                    .MAV_CMD_SET_MESSAGE_INTERVAL,
                    0,
                    mavutil.mavlink
                    .MAVLINK_MSG_ID_SERVO_OUTPUT_RAW,
                    200000,  # 5 Hz
                    0,
                    0,
                    0,
                    0,
                    0,
                )
        )

    def _request_management_data(self):
        """
        Automatically initialize GCS management whenever an
        ArduPilot connection is established.

        Both USV and UAV use this exact same path:
          - request actuator/PWM telemetry
          - synchronize the complete FCU parameter cache
          - recover missing parameter indices automatically

        Parameter synchronization runs in its own thread so the
        MAVLink receive loop remains responsive.
        """

        if self.management_requested:
            return

        self.management_requested = True

        # Start actuator telemetry.
        self._request_servo_output_stream()

        # Complete parameter synchronization in the background.
        thread = threading.Thread(
            target=self.refresh_parameters,
            kwargs={
                "timeout": 90.0,
            },
            name=(
                f"{self.vehicle_id}-parameter-sync"
            ),
            daemon=True,
        )

        thread.start()


    def _run(self):
        while not self.stop_event.is_set():

            try:
                self.connection = (
                    mavutil.mavlink_connection(
                        self.endpoint,
                        dialect="ardupilotmega",
                        source_system=255,
                        source_component=(
                            mavutil.mavlink
                            .MAV_COMP_ID_MISSIONPLANNER
                        ),
                    )
                )

                self._receive_loop()

            except Exception:
                self._mark_offline()

                if not self.stop_event.is_set():
                    self.stop_event.wait(1.0)

            finally:
                connection = self.connection
                self.connection = None

                if connection is not None:
                    try:
                        connection.close()
                    except Exception:
                        pass

    def _receive_loop(self):
        while not self.stop_event.is_set():

            message = self.connection.recv_match(
                blocking=True,
                timeout=0.5,
            )

            now = time.monotonic()

            if message is None:
                if (
                    self.last_rx is not None
                    and now - self.last_rx
                    > self.rx_timeout
                ):
                    self._mark_offline()

                continue

            message_type = message.get_type()

            if message_type == "BAD_DATA":
                continue

            system_id = int(
                message.get_srcSystem()
            )

            component_id = int(
                message.get_srcComponent()
            )

            # Only the primary ArduPilot component owns
            # authoritative autopilot/GCS management state.
            if (
                component_id
                != self.target_component
            ):
                continue

            if self.target_system is None:

                if message_type != "HEARTBEAT":
                    continue

                if (
                    int(message.autopilot)
                    != mavutil.mavlink
                    .MAV_AUTOPILOT_ARDUPILOTMEGA
                ):
                    continue

                self.target_system = system_id

            if system_id != self.target_system:
                continue

            self.last_rx = now
            self.connected = True

            self._handle_message(
                message_type,
                message,
            )

    def _mark_offline(self):
        was_connected = self.connected

        self.connected = False
        self.target_system = None
        self.last_rx = None
        self.armed = False
        self.management_requested = False

        if (
            was_connected
            and self.link_offline_callback
            is not None
        ):
            try:
                self.link_offline_callback(
                    self.vehicle_id,
                    "mavlink",
                )
            except Exception:
                pass

    # ========================================================
    # PARAMETER CACHE
    # ========================================================

    @staticmethod
    def _clean_param_name(value):
        if isinstance(value, bytes):
            value = value.decode(
                "utf-8",
                errors="ignore",
            )

        return str(value).rstrip("\x00")

    @staticmethod
    def _is_servo_config_parameter(name):
        if not name.startswith("SERVO"):
            return False

        head, separator, suffix = (
            name.partition("_")
        )

        if not separator:
            return False

        channel = head[5:]

        if not channel.isdigit():
            return False

        return suffix in {
            "FUNCTION",
            "MIN",
            "MAX",
            "TRIM",
            "REVERSED",
        }

    def _cache_parameter(self, message):
        name = self._clean_param_name(
            message.param_id
        )

        entry = {
            "value": float(
                message.param_value
            ),
            "type": int(
                message.param_type
            ),
            "index": int(
                message.param_index
            ),
        }

        with self.param_condition:

            self.param_cache[name] = entry

            self.param_versions[name] = (
                int(
                    self.param_versions.get(
                        name,
                        0,
                    )
                )
                + 1
            )

            count = int(
                message.param_count
            )

            if count > 0:
                self.param_expected = count

            if self.param_refresh_active:
                self.param_refresh_seen.add(
                    name
                )

            self.param_condition.notify_all()

        if self._is_servo_config_parameter(
            name
        ):
            self.update_state(
                servo_config=(
                    self.servo_configuration()
                )
            )

    def _cached_value(
        self,
        cache,
        name,
        default=None,
    ):
        entry = cache.get(name)

        if entry is None:
            return default

        return entry.get(
            "value",
            default,
        )

    def servo_configuration(self):
        with self.param_condition:
            cache = {
                key: dict(value)
                for key, value
                in self.param_cache.items()
            }

        output = {}

        for channel in range(1, 17):

            prefix = f"SERVO{channel}"

            function_value = self._cached_value(
                cache,
                f"{prefix}_FUNCTION",
            )

            if function_value is None:
                continue

            function = int(
                round(function_value)
            )

            label = (
                self.SERVO_FUNCTION_NAMES.get(
                    function
                )
            )

            if (
                label is None
                and 33 <= function <= 44
            ):
                label = (
                    f"Motor {function - 32}"
                )

            if label is None:
                label = (
                    f"Function {function}"
                )

            output[str(channel)] = {
                "channel": channel,
                "function": function,
                "label": label,
                "min": int(round(
                    self._cached_value(
                        cache,
                        f"{prefix}_MIN",
                        1000.0,
                    )
                )),
                "trim": int(round(
                    self._cached_value(
                        cache,
                        f"{prefix}_TRIM",
                        1500.0,
                    )
                )),
                "max": int(round(
                    self._cached_value(
                        cache,
                        f"{prefix}_MAX",
                        2000.0,
                    )
                )),
                "reversed": bool(round(
                    self._cached_value(
                        cache,
                        f"{prefix}_REVERSED",
                        0.0,
                    )
                )),
            }

        return output

    # ========================================================
    # MESSAGE HANDLING
    # ========================================================

    def _handle_message(
        self,
        message_type,
        message,
    ):

        if message_type == "MISSION_COUNT":

            mission_type = int(
                getattr(
                    message,
                    "mission_type",
                    0,
                )
            )

            if (
                mission_type
                == int(
                    mavutil.mavlink
                    .MAV_MISSION_TYPE_FENCE
                )
            ):

                with self.fence_condition:

                    if self.fence_download_active:

                        self.fence_expected = int(
                            message.count
                        )

                        self.fence_condition.notify_all()

                return


        if message_type == "MISSION_ITEM_INT":

            mission_type = int(
                getattr(
                    message,
                    "mission_type",
                    0,
                )
            )

            if (
                mission_type
                == int(
                    mavutil.mavlink
                    .MAV_MISSION_TYPE_FENCE
                )
            ):

                with self.fence_condition:

                    if self.fence_download_active:

                        seq = int(
                            message.seq
                        )

                        self.fence_items[
                            seq
                        ] = {
                            "seq": seq,
                            "frame": int(
                                message.frame
                            ),
                            "command": int(
                                message.command
                            ),
                            "param1": float(
                                message.param1
                            ),
                            "param2": float(
                                message.param2
                            ),
                            "param3": float(
                                message.param3
                            ),
                            "param4": float(
                                message.param4
                            ),
                            "latitude": (
                                float(message.x)
                                / 1.0e7
                            ),
                            "longitude": (
                                float(message.y)
                                / 1.0e7
                            ),
                            "altitude": float(
                                message.z
                            ),
                        }

                        self.fence_condition.notify_all()

                return


        if message_type in {
            "MISSION_REQUEST_INT",
            "MISSION_REQUEST",
        }:

            mission_type = int(
                getattr(
                    message,
                    "mission_type",
                    0,
                )
            )

            if (
                mission_type
                == int(
                    mavutil.mavlink
                    .MAV_MISSION_TYPE_FENCE
                )
            ):

                seq = int(
                    message.seq
                )

                with self.fence_condition:

                    if not self.fence_upload_active:
                        return

                    item = (
                        self.fence_upload_items
                        .get(seq)
                    )


                if item is None:
                    return


                fence_type = int(
                    mavutil.mavlink
                    .MAV_MISSION_TYPE_FENCE
                )


                self._send_with_connection(
                    lambda connection,
                           system,
                           component,
                           item=dict(item):
                        connection.mav
                        .mission_item_int_send(
                            system,
                            component,
                            int(item["seq"]),
                            int(item["frame"]),
                            int(item["command"]),
                            int(item["current"]),
                            int(item["autocontinue"]),
                            float(item["param1"]),
                            float(item["param2"]),
                            float(item["param3"]),
                            float(item["param4"]),
                            int(item["x"]),
                            int(item["y"]),
                            float(item["z"]),
                            fence_type,
                        )
                )

                return


        if message_type == "MISSION_ACK":

            mission_type = int(
                getattr(
                    message,
                    "mission_type",
                    0,
                )
            )

            if (
                mission_type
                == int(
                    mavutil.mavlink
                    .MAV_MISSION_TYPE_FENCE
                )
            ):

                with self.fence_condition:

                    self.fence_last_ack = int(
                        message.type
                    )

                    self.fence_condition.notify_all()

                return


        if message_type == "PARAM_VALUE":
            self._cache_parameter(
                message
            )
            return

        if message_type == "SERVO_OUTPUT_RAW":

            outputs = {}

            for channel in range(1, 17):

                field = (
                    f"servo{channel}_raw"
                )

                if not hasattr(
                    message,
                    field,
                ):
                    continue

                value = int(
                    getattr(
                        message,
                        field,
                    )
                )

                if value != 0:
                    outputs[
                        str(channel)
                    ] = value

            self.update_state(
                servo_outputs=outputs,
            )

            return

        if message_type == "HEARTBEAT":

            base_mode = int(
                message.base_mode
            )

            armed = bool(
                base_mode
                & mavutil.mavlink
                .MAV_MODE_FLAG_SAFETY_ARMED
            )

            self.armed = armed

            guided = bool(
                base_mode
                & mavutil.mavlink
                .MAV_MODE_FLAG_GUIDED_ENABLED
            )

            manual_input = bool(
                base_mode
                & mavutil.mavlink
                .MAV_MODE_FLAG_MANUAL_INPUT_ENABLED
            )

            mode = mavutil.mode_string_v10(
                message
            )

            self.update_state(
                connected=True,
                armed=armed,
                guided=guided,
                manual_input=manual_input,
                mode=str(mode),
                system_status=int(
                    message.system_status
                ),
            )

            self._request_management_data()

            return

        if message_type == "GLOBAL_POSITION_INT":

            fields = {
                "latitude":
                    float(message.lat) / 1.0e7,
                "longitude":
                    float(message.lon) / 1.0e7,
                "altitude_msl":
                    float(message.alt) / 1000.0,
                "relative_altitude":
                    float(message.relative_alt)
                    / 1000.0,
                "velocity_north":
                    float(message.vx) / 100.0,
                "velocity_east":
                    float(message.vy) / 100.0,
                "velocity_up":
                    -float(message.vz) / 100.0,
            }

            fields["altitude"] = (
                fields["altitude_msl"]
            )

            fields["ground_speed"] = math.hypot(
                fields["velocity_north"],
                fields["velocity_east"],
            )

            if int(message.hdg) != 65535:
                fields["heading_deg"] = (
                    float(message.hdg) / 100.0
                )

            self.update_state(**fields)
            return

        if message_type == "GPS_RAW_INT":

            if int(message.fix_type) >= 2:
                self.update_state(
                    latitude=(
                        float(message.lat)
                        / 1.0e7
                    ),
                    longitude=(
                        float(message.lon)
                        / 1.0e7
                    ),
                    altitude_msl=(
                        float(message.alt)
                        / 1000.0
                    ),
                    altitude=(
                        float(message.alt)
                        / 1000.0
                    ),
                )

            return

        if message_type == "ATTITUDE":

            roll_deg = math.degrees(
                float(message.roll)
            )

            pitch_deg = math.degrees(
                float(message.pitch)
            )

            yaw_deg = math.degrees(
                float(message.yaw)
            )

            heading_deg = (
                yaw_deg + 360.0
            ) % 360.0

            self.update_state(
                roll_deg=roll_deg,
                pitch_deg=pitch_deg,
                yaw_deg=yaw_deg,
                heading_deg=heading_deg,
            )

            return

        if message_type == "VFR_HUD":

            fields = {
                "ground_speed":
                    float(message.groundspeed),
            }

            heading = int(message.heading)

            if heading >= 0:
                fields["heading_deg"] = float(
                    heading % 360
                )

            self.update_state(**fields)
            return

        if message_type == "SYS_STATUS":

            fields = {}

            voltage_mv = int(
                message.voltage_battery
            )

            current_ca = int(
                message.current_battery
            )

            remaining = int(
                message.battery_remaining
            )

            if voltage_mv != 65535:
                fields["voltage"] = (
                    voltage_mv / 1000.0
                )

            if current_ca != -1:
                fields["battery_current"] = (
                    current_ca / 100.0
                )

            if remaining >= 0:
                fields["battery_percent"] = (
                    remaining / 100.0
                )
                fields["battery_remaining"] = (
                    remaining
                )

            if fields:
                self.update_state(**fields)

            return
