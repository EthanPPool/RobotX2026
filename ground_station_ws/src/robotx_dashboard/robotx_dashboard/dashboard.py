#!/usr/bin/env python3

import math
import threading
import time

from flask import Flask, jsonify, Response

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TwistStamped
from mavros_msgs.msg import State, SysStatus
from mavros_msgs.srv import CommandBool, SetMode
from sensor_msgs.msg import BatteryState, Imu, NavSatFix
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger

from boat_interfaces.msg import DetectedObjectArray, Gate
from robotx_dashboard.vehicle_manager import VehicleManager
from robotx_dashboard.clients.boat_client import BoatClient


# ============================================================
# VEHICLE DEFINITIONS
#
# BlueBoat currently uses the standard /mavros namespace.
# UAV namespaces are placeholders for the multi-UAV setup.
# ============================================================

VEHICLES = {
    "boat": {
        "name": "USV",
        "type": "USV",
        "mavros": "/mavros",
    },
    "uav": {
        "name": "UAV",
        "type": "UAV",
        "mavros": "/uav/mavros",
    },
    "uuv": {
        "name": "UUV",
        "type": "UUV",
        "mavros": "/uuv/mavros",
    },
}


HTML = r"""
<!DOCTYPE html>
<html>
<head>
    <meta charset="utf-8">
    <title>RobotX Ground Station</title>

    <meta name="viewport"
          content="width=device-width, initial-scale=1">

    <link rel="stylesheet"
          href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css">

    <script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js">
    </script>

    <style>
        * {
            box-sizing: border-box;
        }

        body {
            margin: 0;
            background: #111820;
            color: #e8edf2;
            font-family: Arial, Helvetica, sans-serif;
        }

        header {
            height: 64px;
            display: flex;
            align-items: center;
            justify-content: space-between;
            padding: 0 22px;
            background: #18222d;
            border-bottom: 1px solid #344250;
        }

        header h1 {
            margin: 0;
            font-size: 22px;
        }

        #tabs {
            display: flex;
            background: #18222d;
            border-bottom: 1px solid #344250;
        }

        .tab {
            padding: 14px 24px;
            cursor: pointer;
            border: 0;
            background: transparent;
            color: #aebbc7;
            font-size: 15px;
        }

        .tab.active {
            color: white;
            background: #263443;
        }

        .page {
            display: none;
            padding: 20px;
        }

        .page.active {
            display: block;
        }

        #map {
            width: 100%;
            height: calc(100vh - 155px);
            min-height: 500px;
            border: 1px solid #344250;
            border-radius: 8px;
        }

        .vehicle-layout {
            display: grid;
            grid-template-columns:
                repeat(auto-fit, minmax(260px, 1fr));
            gap: 18px;
        }

        .card {
            background: #18222d;
            border: 1px solid #344250;
            border-radius: 8px;
            padding: 18px;
        }

        .card h2 {
            margin-top: 0;
            font-size: 18px;
        }

        .row {
            display: flex;
            justify-content: space-between;
            padding: 8px 0;
            border-bottom: 1px solid #283744;
        }

        .row:last-child {
            border-bottom: 0;
        }

        .value {
            font-family: monospace;
        }

        .connected {
            color: #61d095;
        }

        .disconnected {
            color: #ee6c6c;
        }

        .vehicle-arrow {
            width: 36px;
            height: 36px;
            font-size: 32px;
            text-align: center;
            line-height: 36px;
            color: #ffcc4d;
            text-shadow: 0 0 4px #000;
            transform-origin: center center;
        }

        #summary {
            position: absolute;
            z-index: 1000;
            left: 32px;
            bottom: 35px;
            background: rgba(15, 22, 30, 0.90);
            padding: 10px 14px;
            border-radius: 6px;
            pointer-events: none;
        }
    
        .control-button {
            width: 100%;
            min-height: 46px;
            margin-top: 10px;
            border: 1px solid #46596b;
            border-radius: 6px;
            background: #26384a;
            color: #eeeeee;
            font-size: 15px;
            font-weight: bold;
            cursor: pointer;
        }

        .control-button:hover {
            background: #324a60;
        }

        .control-button:disabled {
            opacity: 0.45;
            cursor: not-allowed;
        }

        .arm-button {
            background: #7b531d;
        }

        .disarm-button {
            background: #3b4c59;
        }

        .enable-button {
            background: #27663b;
        }

        .reset-button {
            background: #554272;
        }

        .switch-row {
            display: flex;
            justify-content: space-between;
            align-items: center;
            margin-top: 16px;
            gap: 14px;
        }

        .switch {
            position: relative;
            display: inline-block;
            width: 62px;
            height: 32px;
            flex-shrink: 0;
        }

        .switch input {
            opacity: 0;
            width: 0;
            height: 0;
        }

        .slider {
            position: absolute;
            cursor: pointer;
            inset: 0;
            background: #4d5d68;
            border-radius: 32px;
            transition: 0.15s;
        }

        .slider:before {
            position: absolute;
            content: "";
            height: 24px;
            width: 24px;
            left: 4px;
            bottom: 4px;
            background: white;
            border-radius: 50%;
            transition: 0.15s;
        }

        .switch input:checked + .slider {
            background: #b72f2f;
        }

        .switch input:checked + .slider:before {
            transform: translateX(30px);
        }

        .control-message {
            margin-top: 14px;
            padding: 10px;
            min-height: 20px;
            border: 1px solid #344250;
            border-radius: 5px;
            font-family: monospace;
            font-size: 13px;
            overflow-wrap: anywhere;
        }

        .mission-state-display {
            font-family: monospace;
            font-weight: bold;
            overflow-wrap: anywhere;
        }

</style>
</head>

<body>

<header>
    <h1>RobotX Ground Station</h1>
    <div id="host-status">
        Ground Station: STARTING
    </div>
</header>

<div id="tabs">
    <button class="tab active"
            data-page="map-page">
        Overview Map
    </button>
</div>

<div id="map-page"
     class="page active">

    <div id="map"></div>

    <div id="summary">
        Waiting for telemetry...
    </div>
</div>

<script>

const map = L.map("map").setView(
    [30.22, -92.02],
    13
);

L.tileLayer(
    "https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png",
    {
        maxZoom: 20,
        attribution: "&copy; OpenStreetMap"
    }
).addTo(map);


const markers = {};
const vehiclePages = {};

let mapCentered = false;


function installTabHandlers() {

    document.querySelectorAll(".tab").forEach(button => {

        button.onclick = () => {

            document.querySelectorAll(".tab")
                .forEach(
                    x => x.classList.remove("active")
                );

            document.querySelectorAll(".page")
                .forEach(
                    x => x.classList.remove("active")
                );

            button.classList.add("active");

            document
                .getElementById(button.dataset.page)
                .classList.add("active");

            if (
                button.dataset.page === "map-page"
            ) {
                setTimeout(
                    () => map.invalidateSize(),
                    50
                );
            }
        };
    });
}


function makeVehiclePage(id, vehicle) {

    const tabs =
        document.getElementById("tabs");

    const button =
        document.createElement("button");

    button.className = "tab";
    button.dataset.page = `${id}-page`;
    button.textContent = vehicle.name;

    tabs.appendChild(button);


    const page =
        document.createElement("div");

    page.id = `${id}-page`;
    page.className = "page";


    let usvCards = "";

    if (vehicle.type === "USV") {

        usvCards = `

            <div class="card">
                <h2>Battery Detail</h2>

                <div class="row">
                    <span>Current</span>
                    <span class="value"
                          id="${id}-battery-current">
                        --
                    </span>
                </div>

                <div class="row">
                    <span>Remaining</span>
                    <span class="value"
                          id="${id}-battery-remaining">
                        --
                    </span>
                </div>

                <div class="row">
                    <span>Telemetry</span>
                    <span class="value"
                          id="${id}-battery-telemetry">
                        UNAVAILABLE
                    </span>
                </div>
            </div>


            <div class="card">
                <h2>Perception</h2>

                <div class="row">
                    <span>Buoys</span>
                    <span class="value"
                          id="${id}-buoys">
                        0
                    </span>
                </div>

                <div class="row">
                    <span>Gate</span>
                    <span class="value"
                          id="${id}-gate">
                        NONE
                    </span>
                </div>

                <div class="row">
                    <span>Confidence</span>
                    <span class="value"
                          id="${id}-gate-confidence">
                        --
                    </span>
                </div>

                <div class="row">
                    <span>Center X</span>
                    <span class="value"
                          id="${id}-gate-x">
                        --
                    </span>
                </div>

                <div class="row">
                    <span>Center Y</span>
                    <span class="value"
                          id="${id}-gate-y">
                        --
                    </span>
                </div>
            </div>


            <div class="card">
                <h2>Follower Command</h2>

                <div class="row">
                    <span>Forward</span>
                    <span class="value"
                          id="${id}-control-forward">
                        0.000 m/s
                    </span>
                </div>

                <div class="row">
                    <span>Yaw</span>
                    <span class="value"
                          id="${id}-control-yaw">
                        0.000 rad/s
                    </span>
                </div>

                <div class="row">
                    <span>Command Ready</span>
                    <span class="value"
                          id="${id}-control-ready">
                        STOP
                    </span>
                </div>
            </div>


            <div class="card">
                <h2>State Machine</h2>

                <div class="row">
                    <span>Mission State</span>
                    <span class="value"
                          id="${id}-mission-state">
                        OFF
                    </span>
                </div>

                <div class="row">
                    <span>Autonomy</span>
                    <span class="value"
                          id="${id}-autonomy">
                        OFF
                    </span>
                </div>
            </div>


            <div class="card">
                <h2>Bridge Output</h2>

                <div class="row">
                    <span>Forward</span>
                    <span class="value"
                          id="${id}-bridge-forward">
                        0.000 m/s
                    </span>
                </div>

                <div class="row">
                    <span>Yaw</span>
                    <span class="value"
                          id="${id}-bridge-yaw">
                        0.000 rad/s
                    </span>
                </div>

                <div class="row">
                    <span>Velocity Output</span>
                    <span class="value"
                          id="${id}-bridge-active">
                        INACTIVE
                    </span>
                </div>

                <div class="row">
                    <span>Control State</span>
                    <span class="value"
                          id="${id}-control-state">
                        OFFLINE
                    </span>
                </div>
            </div>


            <div class="card">
                <h2>Xbox Operator Controller</h2>

                <div class="row">
                    <span>Controller</span>
                    <span class="value"
                          id="${id}-gamepad">
                        DISCONNECTED
                    </span>
                </div>

                <div class="row">
                    <span>Deadman (LB)</span>
                    <span class="value"
                          id="${id}-deadman">
                        RELEASED
                    </span>
                </div>

                <div class="row">
                    <span>Forward</span>
                    <span class="value"
                          id="${id}-operator-forward">
                        0.000 m/s
                    </span>
                </div>

                <div class="row">
                    <span>Yaw</span>
                    <span class="value"
                          id="${id}-operator-yaw">
                        0.000 rad/s
                    </span>
                </div>

                <div class="row">
                    <span>Backend</span>
                    <span class="value">
                        READ ONLY
                    </span>
                </div>
            </div>


            <div class="card">
                <h2>Vehicle Control</h2>

                <button
                    class="control-button arm-button"
                    id="${id}-arm-button"
                    onclick="usvControlAction('arm')">
                    ARM
                </button>

                <button
                    class="control-button disarm-button"
                    id="${id}-disarm-button"
                    onclick="usvControlAction('disarm')">
                    DISARM
                </button>
            </div>


            <div class="card">
                <h2>Mission Control</h2>

                <button
                    class="control-button enable-button"
                    id="${id}-enable-button"
                    onclick="usvControlAction('enable')">
                    ENABLE AUTONOMY
                </button>

                <button
                    class="control-button reset-button"
                    id="${id}-reset-button"
                    onclick="usvControlAction('reset_mission')">
                    RESET MISSION
                </button>


                <div class="switch-row">

                    <div>
                        <div style="font-weight:bold;">
                            Software Stop
                        </div>

                        <div class="value"
                             id="${id}-stop-state">
                            UNKNOWN
                        </div>
                    </div>


                    <label class="switch">

                        <input
                            type="checkbox"
                            id="${id}-stop-switch"
                            onchange="toggleUsvStop(this)">

                        <span class="slider"></span>

                    </label>

                </div>


                <div
                    class="control-message"
                    id="${id}-control-message">
                    No control command sent.
                </div>

            </div>


        `;
    }


    page.innerHTML = `
        <div class="vehicle-layout">

            <div class="card">
                <h2>Connection</h2>

                <div class="row">
                    <span>Vehicle</span>
                    <span class="value">
                        ${vehicle.name}
                    </span>
                </div>

                <div class="row">
                    <span>Type</span>
                    <span class="value">
                        ${vehicle.type}
                    </span>
                </div>

                <div class="row">
                    <span>MAVROS</span>
                    <span class="value"
                          id="${id}-connection">
                        DISCONNECTED
                    </span>
                </div>

                <div class="row">
                    <span>Telemetry Age</span>
                    <span class="value"
                          id="${id}-age">
                        --
                    </span>
                </div>
            </div>


            <div class="card">
                <h2>Autopilot</h2>

                <div class="row">
                    <span>Mode</span>
                    <span class="value"
                          id="${id}-mode">
                        UNKNOWN
                    </span>
                </div>

                <div class="row">
                    <span>Armed</span>
                    <span class="value"
                          id="${id}-armed">
                        NO
                    </span>
                </div>
            </div>


            <div class="card">
                <h2>Position</h2>

                <div class="row">
                    <span>Latitude</span>
                    <span class="value"
                          id="${id}-lat">
                        --
                    </span>
                </div>

                <div class="row">
                    <span>Longitude</span>
                    <span class="value"
                          id="${id}-lon">
                        --
                    </span>
                </div>

                <div class="row">
                    <span>Altitude</span>
                    <span class="value"
                          id="${id}-alt">
                        --
                    </span>
                </div>

                <div class="row">
                    <span>Heading</span>
                    <span class="value"
                          id="${id}-heading">
                        --
                    </span>
                </div>
            </div>


            <div class="card">
                <h2>Power</h2>

                <div class="row">
                    <span>Voltage</span>
                    <span class="value"
                          id="${id}-voltage">
                        --
                    </span>
                </div>

                <div class="row">
                    <span>Battery</span>
                    <span class="value"
                          id="${id}-battery">
                        --
                    </span>
                </div>
            </div>


            ${usvCards}

        </div>
    `;


    document.body.appendChild(page);

    vehiclePages[id] = true;

    installTabHandlers();
}


function setText(id, value) {

    const element =
        document.getElementById(id);

    if (element) {
        element.textContent = value;
    }
}


function updateVehicle(id, vehicle) {

    if (!vehiclePages[id]) {
        makeVehiclePage(id, vehicle);
    }


    if (vehicle.type === "USV") {

        setText(
            `${id}-battery-current`,
            vehicle.battery_current === null
                ? "--"
                : `${vehicle.battery_current.toFixed(2)} A`
        );

        setText(
            `${id}-battery-remaining`,
            vehicle.battery_remaining === null
                ? "--"
                : `${vehicle.battery_remaining.toFixed(0)} %`
        );

        setText(
            `${id}-battery-telemetry`,
            vehicle.battery_fresh
                ? "RECEIVING"
                : "UNAVAILABLE"
        );


        setText(
            `${id}-buoys`,
            vehicle.buoy_count ?? 0
        );

        setText(
            `${id}-gate`,
            vehicle.gate_fresh
                ? "DETECTED"
                : "NONE"
        );

        setText(
            `${id}-gate-confidence`,
            vehicle.gate_fresh
                ? vehicle.gate_confidence.toFixed(3)
                : "--"
        );

        setText(
            `${id}-gate-x`,
            vehicle.gate_fresh
                ? `${vehicle.gate_x.toFixed(2)} m`
                : "--"
        );

        setText(
            `${id}-gate-y`,
            vehicle.gate_fresh
                ? `${vehicle.gate_y.toFixed(2)} m`
                : "--"
        );


        setText(
            `${id}-control-forward`,
            `${vehicle.control_forward.toFixed(3)} m/s`
        );

        setText(
            `${id}-control-yaw`,
            `${vehicle.control_yaw.toFixed(3)} rad/s`
        );

        setText(
            `${id}-control-ready`,
            vehicle.control_ready
                ? "READY"
                : "STOP"
        );


        setText(
            `${id}-mission-state`,
            vehicle.mission_state
                ?? (
                    vehicle.online
                        ? "WAITING FOR STATE"
                        : "OFFLINE"
                )
        );

        setText(
            `${id}-autonomy`,
            vehicle.autonomy_enabled
                ? "ENABLED"
                : "OFF"
        );


        setText(
            `${id}-bridge-forward`,
            `${vehicle.bridge_forward.toFixed(3)} m/s`
        );

        setText(
            `${id}-bridge-yaw`,
            `${vehicle.bridge_yaw.toFixed(3)} rad/s`
        );

        setText(
            `${id}-bridge-active`,
            vehicle.bridge_command_active
                ? "ACTIVE"
                : "INACTIVE"
        );

        setText(
            `${id}-control-state`,
            vehicle.control_state ?? "--"
        );
    }


    
    if (vehicle.type === "USV") {

        const armButton =
            document.getElementById(
                `${id}-arm-button`
            );

        const disarmButton =
            document.getElementById(
                `${id}-disarm-button`
            );

        const enableButton =
            document.getElementById(
                `${id}-enable-button`
            );

        const resetButton =
            document.getElementById(
                `${id}-reset-button`
            );

        const stopSwitch =
            document.getElementById(
                `${id}-stop-switch`
            );


        if (armButton) {
            armButton.disabled =
                !vehicle.online;
        }

        if (disarmButton) {
            disarmButton.disabled =
                !vehicle.online;
        }

        if (enableButton) {
            enableButton.disabled =
                !vehicle.can_enable;
        }

        if (resetButton) {
            resetButton.disabled =
                !vehicle.online;
        }


        if (stopSwitch) {

            stopSwitch.checked =
                vehicle.software_stop
                    === "ENGAGED";
        }


        setText(
            `${id}-stop-state`,
            vehicle.software_stop
                ?? "UNKNOWN"
        );
    }


    const connection =
        document.getElementById(
            `${id}-connection`
        );

    connection.textContent =
        vehicle.online
            ? "CONNECTED"
            : "DISCONNECTED";

    connection.className =
        "value " +
        (
            vehicle.online
                ? "connected"
                : "disconnected"
        );


    setText(
        `${id}-age`,
        vehicle.age_sec === null
            ? "--"
            : `${vehicle.age_sec.toFixed(2)} s`
    );

    setText(
        `${id}-mode`,
        vehicle.mode ?? "--"
    );

    setText(
        `${id}-armed`,
        vehicle.armed ? "YES" : "NO"
    );

    setText(
        `${id}-lat`,
        vehicle.latitude === null
            ? "--"
            : vehicle.latitude.toFixed(7)
    );

    setText(
        `${id}-lon`,
        vehicle.longitude === null
            ? "--"
            : vehicle.longitude.toFixed(7)
    );

    setText(
        `${id}-alt`,
        vehicle.altitude === null
            ? "--"
            : `${vehicle.altitude.toFixed(1)} m`
    );

    setText(
        `${id}-heading`,
        vehicle.heading_deg === null
            ? "--"
            : `${vehicle.heading_deg.toFixed(1)}°`
    );

    setText(
        `${id}-voltage`,
        vehicle.voltage === null
            ? "--"
            : `${vehicle.voltage.toFixed(2)} V`
    );

    setText(
        `${id}-battery`,
        vehicle.battery_percent === null
            ? "--"
            : `${vehicle.battery_percent.toFixed(0)} %`
    );


    if (
        vehicle.online &&
        vehicle.latitude !== null &&
        vehicle.longitude !== null
    ) {

        const heading =
            vehicle.heading_deg === null
                ? 0
                : vehicle.heading_deg;

        const icon = L.divIcon({
            className: "",
            html:
                `<div
                    class="vehicle-arrow"
                    style="transform:
                    rotate(${heading}deg)">
                    ▲
                </div>`,
            iconSize: [36, 36],
            iconAnchor: [18, 18]
        });


        if (!markers[id]) {

            markers[id] = L.marker(
                [
                    vehicle.latitude,
                    vehicle.longitude
                ],
                {
                    icon: icon
                }
            )
            .addTo(map)
            .bindTooltip(
                `${vehicle.name} (${vehicle.type})`,
                {
                    permanent: true,
                    direction: "right"
                }
            );


            markers[id].on(
                "click",
                () => {

                    const tab =
                        Array.from(
                            document.querySelectorAll(
                                ".tab"
                            )
                        )
                        .find(
                            x =>
                                x.dataset.page ===
                                `${id}-page`
                        );

                    if (tab) {
                        tab.click();
                    }
                }
            );

        } else {

            markers[id].setLatLng(
                [
                    vehicle.latitude,
                    vehicle.longitude
                ]
            );

            markers[id].setIcon(icon);
        }


        if (!mapCentered) {

            map.setView(
                [
                    vehicle.latitude,
                    vehicle.longitude
                ],
                18
            );

            mapCentered = true;
        }

    } else if (markers[id]) {

        map.removeLayer(markers[id]);

        delete markers[id];
    }
}



async function postUsvControl(action) {

    const messageBox =
        document.getElementById(
            "boat-control-message"
        );

    if (messageBox) {
        messageBox.textContent =
            "Sending " + action + "...";
    }


    try {

        const response = await fetch(
            `/api/usv/${action}`,
            {
                method: "POST",
                cache: "no-store"
            }
        );

        const result =
            await response.json();


        if (messageBox) {
            messageBox.textContent =
                result.message
                ?? "No response message.";
        }


        if (
            typeof refresh
            === "function"
        ) {
            await refresh();
        }


        return result;

    } catch (error) {

        if (messageBox) {
            messageBox.textContent =
                "Control request failed: "
                + error;
        }

        return {
            success: false,
            message: String(error)
        };
    }
}


async function usvControlAction(action) {

    if (action === "arm") {

        if (!confirm(
            "ARM the USV? "
            + "Keep the physical E-stop accessible."
        )) {
            return;
        }
    }


    if (action === "enable") {

        if (!confirm(
            "Prepare autonomous control? "
            + "The USV will enter GUIDED while "
            + "DISARMED and wait for ARM."
        )) {
            return;
        }
    }


    if (action === "disarm") {

        if (!confirm(
            "DISARM the USV and revoke "
            + "autonomy?"
        )) {
            return;
        }
    }


    await postUsvControl(action);
}


async function toggleUsvStop(input) {

    const requestedEngaged =
        input.checked;


    if (!requestedEngaged) {

        const confirmed = confirm(
            "Clear the SOFTWARE STOP?\n\n"
            + "This does NOT arm the USV and "
            + "does NOT enable autonomy."
        );

        if (!confirmed) {

            input.checked = true;

            return;
        }
    }


    input.disabled = true;


    const action =
        requestedEngaged
            ? "stop"
            : "clear_stop";


    const result =
        await postUsvControl(action);


    if (!result.success) {

        // Restore the previous visual state.
        input.checked =
            !requestedEngaged;
    }


    input.disabled = false;
}


async function refresh() {

    try {

        const response =
            await fetch("/api/vehicles");

        const vehicles =
            await response.json();

        let onlineCount = 0;
        let totalCount = 0;


        Object.entries(vehicles).forEach(
            ([id, vehicle]) => {

                totalCount += 1;

                if (vehicle.online) {
                    onlineCount += 1;
                }

                updateVehicle(
                    id,
                    vehicle
                );
            }
        );


        setText(
            "host-status",
            "Ground Station: ONLINE"
        );

        setText(
            "summary",
            `${onlineCount}/${totalCount} vehicles connected`
        );

    } catch (error) {

        setText(
            "host-status",
            "Ground Station: API ERROR"
        );
    }
}


installTabHandlers();

refresh();

setInterval(
    refresh,
    500
);

</script>

</body>
</html>
"""


class RobotXDashboard(Node):

    VALID_MODES = {
        "MANUAL",
        "HOLD",
        "LOITER",
        "AUTO",
        "RTL",
    }

    def __init__(self):

        super().__init__("robotx_dashboard")

        self.lock = threading.RLock()
        self.action_lock = threading.RLock()

        # Dashboard-side record of the control sequence.
        # The Jetson bridge remains the actual propulsion
        # authorization boundary.
        self.control_state = "BOOT SAFE"
        self.software_stop_state = "UNKNOWN"

        self.vehicle_manager = VehicleManager(
            VEHICLES,
            lock=self.lock,
        )

        # Compatibility alias.
        #
        # All existing callbacks, snapshot logic, API routes,
        # and frontend behavior continue using self.vehicles.
        self.vehicles = self.vehicle_manager.vehicles

        # ----------------------------------------------------
        # Remote vehicle transports
        # ----------------------------------------------------
        #
        # BlueBoat telemetry is received from the Jetson over
        # TCP instead of requiring this ground station to join
        # the Jetson's ROS 2 DDS graph.
        self.boat_client = BoatClient(
            "boat",
            self.vehicle_manager.update_vehicle,
            host="192.168.2.20",
            port=8765,
        )

        self.vehicle_manager.register_client(
            "boat",
            self.boat_client,
        )

        self.boat_client.start()

        for vehicle_id, spec in VEHICLES.items():

            # BlueBoat telemetry now arrives through BoatClient
            # over TCP from the Jetson-side dashboard bridge.
            # Do not join the BlueBoat ROS 2 graph from Beeptop.
            if vehicle_id == "boat":
                self.get_logger().info(
                    "Monitoring USV via TCP bridge "
                    "at 192.168.2.20:8765"
                )
                continue

            prefix = spec["mavros"].rstrip("/")

            self.create_subscription(
                State,
                f"{prefix}/state",
                lambda msg, vid=vehicle_id:
                    self.state_callback(vid, msg),
                10
            )

            self.create_subscription(
                NavSatFix,
                f"{prefix}/global_position/global",
                lambda msg, vid=vehicle_id:
                    self.gps_callback(vid, msg),
                10
            )

            self.create_subscription(
                Imu,
                f"{prefix}/imu/data",
                lambda msg, vid=vehicle_id:
                    self.imu_callback(vid, msg),
                10
            )

            self.create_subscription(
                BatteryState,
                f"{prefix}/battery",
                lambda msg, vid=vehicle_id:
                    self.battery_callback(vid, msg),
                10
            )

            if vehicle_id == "boat":

                self.create_subscription(
                    SysStatus,
                    "/mavros/sys_status",
                    self.usv_sys_status_callback,
                    10
                )

                self.create_subscription(
                    DetectedObjectArray,
                    "/perception/objects",
                    self.usv_objects_callback,
                    10
                )

                self.create_subscription(
                    Gate,
                    "/perception/gate",
                    self.usv_gate_callback,
                    10
                )

                self.create_subscription(
                    TwistStamped,
                    "/control/cmd_vel",
                    self.usv_control_callback,
                    10
                )

                self.create_subscription(
                    String,
                    "/mission/state",
                    self.usv_mission_callback,
                    10
                )

                self.create_subscription(
                    TwistStamped,
                    "/mavros/setpoint_velocity/cmd_vel",
                    self.usv_bridge_callback,
                    10
                )

            self.get_logger().info(
                f"Monitoring {spec['name']} at {prefix}"
            )

        # ----------------------------------------------------
        # USV control service clients
        # ----------------------------------------------------

        self.estop_client = self.create_client(
            SetBool,
            "/vehicle/software_estop"
        )

        self.autonomy_client = self.create_client(
            SetBool,
            "/vehicle/set_autonomy"
        )

        self.arm_client = self.create_client(
            CommandBool,
            "/mavros/cmd/arming"
        )

        self.mode_client = self.create_client(
            SetMode,
            "/mavros/set_mode"
        )

        self.reset_client = self.create_client(
            Trigger,
            "/control/reset_mission"
        )

        self.register_control_routes()


    def destroy_node(self):

        if hasattr(self, "boat_client"):
            self.boat_client.stop()

        return super().destroy_node()


    # ========================================================
    # HTTP CONTROL ROUTES
    # ========================================================

    def register_control_routes(self):

        def result_response(func):

            success, message = func()

            return jsonify({
                "success": bool(success),
                "message": str(message),
                "control_state": self.control_state,
                "software_stop":
                    self.software_stop_state,
            })

        app.add_url_rule(
            "/api/usv/arm",
            endpoint="usv_arm",
            view_func=lambda:
                result_response(self.execute_arm),
            methods=["POST"]
        )

        app.add_url_rule(
            "/api/usv/disarm",
            endpoint="usv_disarm",
            view_func=lambda:
                result_response(self.execute_disarm),
            methods=["POST"]
        )

        app.add_url_rule(
            "/api/usv/enable",
            endpoint="usv_enable",
            view_func=lambda:
                result_response(self.execute_enable),
            methods=["POST"]
        )

        app.add_url_rule(
            "/api/usv/stop",
            endpoint="usv_stop",
            view_func=lambda:
                result_response(self.execute_stop),
            methods=["POST"]
        )

        app.add_url_rule(
            "/api/usv/clear_stop",
            endpoint="usv_clear_stop",
            view_func=lambda:
                result_response(
                    self.execute_clear_stop
                ),
            methods=["POST"]
        )

        app.add_url_rule(
            "/api/usv/reset_mission",
            endpoint="usv_reset_mission",
            view_func=lambda:
                result_response(
                    self.execute_reset_mission
                ),
            methods=["POST"]
        )


    # ========================================================
    # TELEMETRY CALLBACKS
    # ========================================================

    def touch(self, vehicle_id):

        self.vehicles[
            vehicle_id
        ]["last_rx"] = time.monotonic()


    def state_callback(self, vehicle_id, msg):

        with self.lock:

            data = self.vehicles[vehicle_id]

            data["connected"] = bool(
                msg.connected
            )

            data["armed"] = bool(
                msg.armed
            )

            data["mode"] = (
                str(msg.mode)
                if msg.mode
                else "UNKNOWN"
            )

            self.touch(vehicle_id)

        if (
            vehicle_id == "boat"
            and bool(msg.armed)
            and str(msg.mode).upper() == "GUIDED"
            and self.control_state
                == "AUTONOMY READY / DISARMED"
        ):
            self.control_state = "ENABLED"


    def gps_callback(self, vehicle_id, msg):

        with self.lock:

            data = self.vehicles[vehicle_id]

            if math.isfinite(msg.latitude):
                data["latitude"] = float(
                    msg.latitude
                )

            if math.isfinite(msg.longitude):
                data["longitude"] = float(
                    msg.longitude
                )

            if math.isfinite(msg.altitude):
                data["altitude"] = float(
                    msg.altitude
                )

            self.touch(vehicle_id)


    def imu_callback(self, vehicle_id, msg):

        q = msg.orientation

        siny_cosp = 2.0 * (
            q.w * q.z
            + q.x * q.y
        )

        cosy_cosp = 1.0 - 2.0 * (
            q.y * q.y
            + q.z * q.z
        )

        yaw = math.atan2(
            siny_cosp,
            cosy_cosp
        )

        heading = (
            math.degrees(yaw) + 360.0
        ) % 360.0

        with self.lock:

            self.vehicles[
                vehicle_id
            ]["heading_deg"] = heading

            self.touch(vehicle_id)


    def battery_callback(self, vehicle_id, msg):

        with self.lock:

            data = self.vehicles[vehicle_id]

            if math.isfinite(msg.voltage):
                data["voltage"] = float(
                    msg.voltage
                )

            if (
                math.isfinite(msg.percentage)
                and msg.percentage >= 0.0
            ):
                data["battery_percent"] = (
                    float(msg.percentage)
                    * 100.0
                )

            self.touch(vehicle_id)


    def usv_sys_status_callback(self, msg):

        with self.lock:

            data = self.vehicles["boat"]

            voltage_raw = int(
                msg.voltage_battery
            )

            current_raw = int(
                msg.current_battery
            )

            remaining_raw = int(
                msg.battery_remaining
            )

            if (
                voltage_raw > 0
                and voltage_raw != 65535
            ):
                data["voltage"] = (
                    voltage_raw / 1000.0
                )
            else:
                data["voltage"] = None

            if current_raw >= 0:
                data["battery_current"] = (
                    current_raw / 100.0
                )
            else:
                data["battery_current"] = None

            if remaining_raw >= 0:

                data[
                    "battery_remaining"
                ] = float(remaining_raw)

                data[
                    "battery_percent"
                ] = float(remaining_raw)

            else:
                data["battery_remaining"] = None

            data["battery_last_rx"] = (
                time.monotonic()
            )


    def usv_objects_callback(self, msg):

        with self.lock:

            self.vehicles["boat"][
                "buoy_count"
            ] = len(msg.objects)


    def usv_gate_callback(self, msg):

        with self.lock:

            data = self.vehicles["boat"]

            data["gate_confidence"] = float(
                msg.confidence
            )

            data["gate_x"] = float(
                msg.center.x
            )

            data["gate_y"] = float(
                msg.center.y
            )

            data["gate_last_rx"] = (
                time.monotonic()
            )


    def usv_control_callback(self, msg):

        with self.lock:

            data = self.vehicles["boat"]

            data["control_forward"] = float(
                msg.twist.linear.x
            )

            data["control_yaw"] = float(
                msg.twist.angular.z
            )


    def usv_mission_callback(self, msg):

        with self.lock:

            state = str(msg.data).strip()

            self.vehicles["boat"][
                "mission_state"
            ] = (
                state if state else None
            )


    def usv_bridge_callback(self, msg):

        with self.lock:

            data = self.vehicles["boat"]

            data["bridge_forward"] = float(
                msg.twist.linear.x
            )

            data["bridge_yaw"] = float(
                msg.twist.angular.z
            )

            data["bridge_last_rx"] = (
                time.monotonic()
            )


    # ========================================================
    # STATUS SNAPSHOT
    # ========================================================

    def snapshot(self):

        now = time.monotonic()
        output = {}

        with self.lock:

            for vehicle_id, source in (
                self.vehicles.items()
            ):

                data = dict(source)

                last_rx = data.pop(
                    "last_rx"
                )

                age = (
                    None
                    if last_rx is None
                    else now - last_rx
                )

                data["age_sec"] = age

                # Existing "online" continues to mean that the
                # autopilot/MAVROS state is connected and fresh.
                data["online"] = bool(
                    data["connected"]
                    and age is not None
                    and age < 2.0
                )

                vehicle_link_rx = data.pop(
                    "vehicle_link_last_rx",
                    None
                )

                vehicle_link_age = (
                    None
                    if vehicle_link_rx is None
                    else now - vehicle_link_rx
                )

                data[
                    "vehicle_link_age_sec"
                ] = vehicle_link_age

                data["vehicle_link"] = bool(
                    data.get(
                        "vehicle_link",
                        False,
                    )
                    and
                    vehicle_link_age is not None
                    and
                    vehicle_link_age < 2.0
                )

                battery_rx = data.pop(
                    "battery_last_rx",
                    None
                )

                gate_rx = data.pop(
                    "gate_last_rx",
                    None
                )

                bridge_rx = data.pop(
                    "bridge_last_rx",
                    None
                )

                data["battery_fresh"] = bool(
                    battery_rx is not None
                    and now - battery_rx <= 3.0
                )

                data["gate_fresh"] = bool(
                    gate_rx is not None
                    and now - gate_rx <= 0.50
                    and
                    data["gate_confidence"]
                        is not None
                    and
                    data["gate_confidence"]
                        >= 0.75
                    and
                    data["gate_x"] is not None
                    and data["gate_x"] > 0.0
                )

                data["control_ready"] = bool(
                    abs(
                        data["control_forward"]
                    ) > 0.0001
                    or
                    abs(
                        data["control_yaw"]
                    ) > 0.0001
                )

                data[
                    "bridge_command_active"
                ] = bool(
                    bridge_rx is not None
                    and now - bridge_rx <= 0.50
                )

                if not data[
                    "bridge_command_active"
                ]:
                    data["bridge_forward"] = 0.0
                    data["bridge_yaw"] = 0.0

                if vehicle_id == "boat":

                    mode = str(
                        data["mode"]
                    ).upper()

                    bridge_alive = bool(
                        self.estop_client
                            .service_is_ready()
                        and
                        self.autonomy_client
                            .service_is_ready()
                    )

                    data["bridge_alive"] = (
                        bridge_alive
                    )

                    data["control_state"] = (
                        self.control_state
                        if data["online"]
                        else "OFFLINE"
                    )

                    data[
                        "software_stop"
                    ] = self.software_stop_state

                    data[
                        "autonomy_enabled"
                    ] = bool(
                        mode == "GUIDED"
                        and
                        self.control_state
                        in (
                            "ENABLED",
                            "AUTONOMY READY / DISARMED",
                        )
                    )

                    data["can_enable"] = bool(
                        data["online"]
                        and bridge_alive
                        and self.control_state
                        not in (
                            "ENABLED",
                            "AUTONOMY READY / DISARMED",
                        )
                    )

                else:

                    data["bridge_alive"] = False
                    data["control_state"] = "---"
                    data["software_stop"] = "---"
                    data["autonomy_enabled"] = False
                    data["can_enable"] = False

                output[vehicle_id] = data

        return output


    def usv_status(self):

        return self.snapshot()["boat"]


    # ========================================================
    # ROS SERVICE HELPERS
    # ========================================================

    def wait_future(
        self,
        future,
        timeout=2.0
    ):

        deadline = (
            time.monotonic() + timeout
        )

        while (
            not future.done()
            and time.monotonic() < deadline
        ):
            time.sleep(0.02)

        return future.done()


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
                "ROS service unavailable"
            )

        request = SetBool.Request()
        request.data = bool(value)

        future = client.call_async(
            request
        )

        if not self.wait_future(
            future,
            timeout
        ):
            return (
                False,
                "ROS service call timed out"
            )

        try:
            response = future.result()

        except Exception as exc:
            return (
                False,
                f"ROS service exception: {exc}"
            )

        if response is None:
            return (
                False,
                "ROS service returned no response"
            )

        return (
            bool(response.success),
            str(response.message)
        )


    def call_arm_service(
        self,
        arm,
        timeout=3.0
    ):

        if not self.arm_client.wait_for_service(
            timeout_sec=0.50
        ):
            return (
                False,
                "MAVROS arming service unavailable"
            )

        request = CommandBool.Request()
        request.value = bool(arm)

        future = self.arm_client.call_async(
            request
        )

        if not self.wait_future(
            future,
            timeout
        ):
            return (
                False,
                "Arming service timed out"
            )

        try:
            response = future.result()

        except Exception as exc:
            return (
                False,
                f"Arming exception: {exc}"
            )

        if (
            response is None
            or not response.success
        ):
            return (
                False,
                "Arming rejected"
            )

        return (
            True,
            "ARMED" if arm else "DISARMED"
        )


    def call_mode_service(
        self,
        mode,
        timeout=3.0
    ):

        if not self.mode_client.wait_for_service(
            timeout_sec=0.50
        ):
            return (
                False,
                "MAVROS mode service unavailable"
            )

        request = SetMode.Request()
        request.base_mode = 0
        request.custom_mode = str(mode)

        future = self.mode_client.call_async(
            request
        )

        if not self.wait_future(
            future,
            timeout
        ):
            return (
                False,
                "Mode service timed out"
            )

        response = future.result()

        if (
            response is None
            or not response.mode_sent
        ):
            return (
                False,
                f"Mode {mode} rejected"
            )

        return (
            True,
            f"Mode request sent: {mode}"
        )


    def call_reset_service(
        self,
        timeout=2.0
    ):

        if not self.reset_client.wait_for_service(
            timeout_sec=0.50
        ):
            return (
                False,
                "Mission reset service unavailable"
            )

        request = Trigger.Request()

        future = self.reset_client.call_async(
            request
        )

        if not self.wait_future(
            future,
            timeout
        ):
            return (
                False,
                "Mission reset timed out"
            )

        response = future.result()

        if response is None:
            return (
                False,
                "Mission reset returned no response"
            )

        return (
            bool(response.success),
            str(response.message)
        )


    # ========================================================
    # USV CONTROL ACTIONS
    # ========================================================

    def execute_fail_safe_stop(self):

        self.call_bool_service(
            self.estop_client,
            True
        )

        self.call_bool_service(
            self.autonomy_client,
            False
        )

        self.software_stop_state = "ENGAGED"
        self.control_state = "STOPPED / HOLD"


    def execute_stop(self):

        with self.action_lock:

            messages = []
            success = True

            ok, msg = self.call_bool_service(
                self.estop_client,
                True
            )

            messages.append(
                "software_stop: " + msg
            )

            if ok:
                self.software_stop_state = (
                    "ENGAGED"
                )
            else:
                success = False

            ok, msg = self.call_bool_service(
                self.autonomy_client,
                False
            )

            messages.append(
                "autonomy: " + msg
            )

            if not ok:
                success = False

            self.control_state = (
                "STOPPED / HOLD"
                if success
                else "STOP COMMAND FAILED"
            )

            return (
                success,
                " | ".join(messages)
            )


    def execute_clear_stop(self):

        with self.action_lock:

            status = self.usv_status()

            if not status["online"]:
                return (
                    False,
                    "Clear stop rejected: MAVROS "
                    "is not connected"
                )

            if status["armed"]:
                return (
                    False,
                    "Clear stop rejected: "
                    "vehicle is armed"
                )

            # Autonomy must remain revoked when the
            # software stop is manually cleared.
            ok, autonomy_msg = (
                self.call_bool_service(
                    self.autonomy_client,
                    False
                )
            )

            if not ok:
                return (
                    False,
                    "Could not verify autonomy OFF: "
                    + autonomy_msg
                )

            ok, stop_msg = (
                self.call_bool_service(
                    self.estop_client,
                    False
                )
            )

            if not ok:
                return (
                    False,
                    "Could not clear software stop: "
                    + stop_msg
                )

            self.software_stop_state = "CLEARED"

            self.control_state = (
                "STOP CLEARED / AUTONOMY OFF"
            )

            return (
                True,
                "Software stop CLEARED. "
                "Autonomy remains OFF and "
                "vehicle remains DISARMED."
            )


    def execute_enable(self):

        with self.action_lock:

            status = self.usv_status()

            if not status["online"]:
                return (
                    False,
                    "Enable rejected: MAVROS is not connected"
                )

            if not status["gate_fresh"]:
                return (
                    False,
                    "Enable rejected: no fresh "
                    "high-confidence gate"
                )

            if not status["control_ready"]:
                return (
                    False,
                    "Enable rejected: follower "
                    "is commanding STOP"
                )

            if not status["bridge_alive"]:
                return (
                    False,
                    "Enable rejected: bridge "
                    "services unavailable"
                )

            messages = []

            # Start from a known safe state.
            ok, msg = self.call_bool_service(
                self.estop_client,
                True
            )

            messages.append(
                "safety: " + msg
            )

            if not ok:
                return (
                    False,
                    " | ".join(messages)
                )

            self.software_stop_state = "ENGAGED"

            status = self.usv_status()

            # Never transition an armed USV into GUIDED.
            if status["armed"]:

                ok, msg = self.call_arm_service(
                    False
                )

                messages.append(
                    "disarm: " + msg
                )

                if not ok:
                    return (
                        False,
                        " | ".join(messages)
                    )

                deadline = (
                    time.monotonic() + 1.5
                )

                while (
                    time.monotonic()
                    < deadline
                ):

                    state = self.vehicles[
                        "boat"
                    ]

                    if not state["armed"]:
                        break

                    time.sleep(0.02)

                if self.vehicles[
                    "boat"
                ]["armed"]:

                    return (
                        False,
                        "Enable aborted: USV did "
                        "not confirm DISARM"
                    )

            # Clear software stop while still disarmed.
            ok, msg = self.call_bool_service(
                self.estop_client,
                False
            )

            messages.append(
                "software_stop: " + msg
            )

            if not ok:
                self.execute_fail_safe_stop()

                return (
                    False,
                    " | ".join(messages)
                )

            self.software_stop_state = "CLEARED"

            # Enter GUIDED while still disarmed.
            ok, msg = self.call_mode_service(
                "GUIDED"
            )

            messages.append(
                "mode: " + msg
            )

            if not ok:
                self.execute_fail_safe_stop()

                return (
                    False,
                    " | ".join(messages)
                )

            deadline = (
                time.monotonic() + 1.5
            )

            while (
                time.monotonic() < deadline
            ):

                state = self.vehicles["boat"]

                if (
                    str(state["mode"]).upper()
                    == "GUIDED"
                ):
                    break

                time.sleep(0.02)

            state = self.vehicles["boat"]

            if (
                str(state["mode"]).upper()
                != "GUIDED"
            ):
                self.execute_fail_safe_stop()

                return (
                    False,
                    "Enable aborted: GUIDED "
                    "was not confirmed"
                )

            if state["armed"]:
                self.execute_fail_safe_stop()

                return (
                    False,
                    "Enable aborted: vehicle "
                    "unexpectedly armed"
                )

            previous_bridge_time = (
                self.vehicles[
                    "boat"
                ]["bridge_last_rx"]
            )

            ok, msg = self.call_bool_service(
                self.autonomy_client,
                True
            )

            messages.append(
                "autonomy: " + msg
            )

            if not ok:
                self.execute_fail_safe_stop()

                return (
                    False,
                    " | ".join(messages)
                )

            deadline = (
                time.monotonic() + 1.25
            )

            fresh_bridge = False

            while (
                time.monotonic() < deadline
            ):

                current = self.vehicles[
                    "boat"
                ]["bridge_last_rx"]

                if (
                    current is not None
                    and (
                        previous_bridge_time
                            is None
                        or
                        current
                            > previous_bridge_time
                    )
                ):
                    fresh_bridge = True
                    break

                time.sleep(0.02)

            if not fresh_bridge:
                self.execute_fail_safe_stop()

                return (
                    False,
                    "Enable aborted: bridge "
                    "did not produce a fresh "
                    "autonomous setpoint"
                )

            state = self.vehicles["boat"]

            if (
                state["armed"]
                or
                str(state["mode"]).upper()
                    != "GUIDED"
            ):
                self.execute_fail_safe_stop()

                return (
                    False,
                    "Enable aborted: vehicle "
                    "state changed during preparation"
                )

            self.control_state = (
                "AUTONOMY READY / DISARMED"
            )

            return (
                True,
                "Autonomy READY: GUIDED entered "
                "while DISARMED and fresh autonomous "
                "setpoints are streaming. "
                "Press ARM to start propulsion."
            )


    def execute_arm(self):

        with self.action_lock:

            status = self.usv_status()

            if not status["online"]:
                return (
                    False,
                    "ARM rejected: MAVROS "
                    "is not connected"
                )

            if (
                str(status["mode"]).upper()
                == "GUIDED"
            ):

                if (
                    self.control_state
                        != "AUTONOMY READY / DISARMED"
                    or
                    not status[
                        "bridge_command_active"
                    ]
                ):
                    return (
                        False,
                        "ARM rejected: GUIDED requires "
                        "ENABLE AUTONOMY first"
                    )

            ok, msg = self.call_arm_service(
                True
            )

            if (
                ok
                and
                str(status["mode"]).upper()
                    == "GUIDED"
            ):
                self.control_state = "ENABLED"

            return (
                ok,
                msg
            )


    def execute_disarm(self):

        with self.action_lock:

            stop_ok, stop_msg = (
                self.execute_stop()
            )

            arm_ok, arm_msg = (
                self.call_arm_service(
                    False
                )
            )

            self.control_state = (
                "DISARMED"
                if arm_ok
                else "DISARM FAILED"
            )

            return (
                stop_ok and arm_ok,
                stop_msg
                + " | disarm: "
                + arm_msg
            )


    def execute_reset_mission(self):

        with self.action_lock:

            stop_ok, stop_msg = (
                self.execute_stop()
            )

            reset_ok, reset_msg = (
                self.call_reset_service()
            )

            if reset_ok:
                self.control_state = (
                    "MISSION RESET / STOPPED"
                )

            return (
                stop_ok and reset_ok,
                stop_msg
                + " | reset: "
                + reset_msg
            )


app = Flask(__name__)

dashboard_node = None


@app.route("/")
def index():

    return Response(
        HTML,
        mimetype="text/html"
    )


@app.route("/api/vehicles")
def api_vehicles():

    if dashboard_node is None:
        return jsonify({})

    return jsonify(
        dashboard_node.snapshot()
    )


def main(args=None):

    global dashboard_node

    rclpy.init(args=args)

    dashboard_node = (
        RobotXDashboard()
    )


    ros_thread = threading.Thread(
        target=rclpy.spin,
        args=(dashboard_node,),
        daemon=True
    )

    ros_thread.start()


    dashboard_node.get_logger().info(
        "RobotX Ground Station running"
    )

    dashboard_node.get_logger().info(
        "Open http://localhost:8080"
    )


    try:

        app.run(
            host="0.0.0.0",
            port=8080,
            debug=False,
            use_reloader=False
        )

    finally:

        dashboard_node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
