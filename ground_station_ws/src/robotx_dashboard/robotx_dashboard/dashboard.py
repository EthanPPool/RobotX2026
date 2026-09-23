#!/usr/bin/env python3

import math
import re
import subprocess
import threading
import time
from pathlib import Path

from flask import (
    Flask,
    jsonify,
    Response,
    request,
    send_file,
)

from waitress import serve

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TwistStamped
from mavros_msgs.msg import State, SysStatus
from sensor_msgs.msg import BatteryState, Imu, NavSatFix
from std_msgs.msg import String

from boat_interfaces.msg import DetectedObjectArray, Gate
from robotx_dashboard.vehicle_manager import VehicleManager
from robotx_dashboard.clients.boat_client import BoatClient
from robotx_dashboard.clients.boat_mavlink_client import BoatMavlinkClient
from robotx_dashboard.clients.uav_client import UavClient
from robotx_dashboard.clients.robocommand_client import RoboCommandClient


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

        #map-page {
            position: relative;
        }

        #map {
            width: 100%;
            height: calc(100vh - 155px);
            min-height: 500px;
            border: 1px solid #344250;
            border-radius: 8px;
        }

        /* ==================================================
           OVERVIEW MAP GEOFENCE CONTROL
           ================================================== */

        .map-geofence-card {
            position: absolute;
            z-index: 1100;
            top: 36px;
            right: 36px;
            width: 320px;
            max-height: calc(100vh - 210px);
            overflow-y: auto;

            background: rgba(24, 34, 45, 0.96);
            border: 1px solid #52677a;
            border-radius: 8px;
            padding: 14px;

            box-shadow:
                0 4px 18px rgba(0, 0, 0, 0.45);
        }

        .map-geofence-card h2 {
            margin: 0 0 10px 0;
            font-size: 17px;
        }

        .map-geofence-buttons {
            display: grid;
            grid-template-columns: 1fr;
            gap: 8px;
            margin-top: 12px;
        }

        .map-geofence-message {
            margin-top: 10px;
            padding: 8px;
            min-height: 34px;

            background: #10171e;
            border: 1px solid #344250;
            border-radius: 5px;

            font-family: monospace;
            font-size: 12px;
            overflow-wrap: anywhere;
        }

        .map-geofence-drawing {
            color: #ffd84d;
            font-weight: bold;
        }

        .map-geofence-enabled {
            color: #61d095;
            font-weight: bold;
        }

        .map-geofence-disabled {
            color: #ee6c6c;
            font-weight: bold;
        }

        @media (max-width: 700px) {
            .map-geofence-card {
                width: calc(100% - 70px);
                right: 35px;
            }
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

        .log-label-input {
            width: 100%;
            margin-top: 8px;
            padding: 10px;
            border: 1px solid #46596b;
            border-radius: 5px;
            background: #10171e;
            color: #e8edf2;
            font-family: monospace;
        }

        .log-path {
            max-width: 62%;
            text-align: right;
            overflow-wrap: anywhere;
        }

        .mission-state-display {
            font-family: monospace;
            font-weight: bold;
            overflow-wrap: anywhere;
        }



        /* ==================================================
           VEHICLE OPERATOR CONSOLE
           ================================================== */

        .console-status-panel {
            overflow: hidden;
            border: 1px solid #2d3d4b;
            border-radius: 10px;
            background: #0d141b;
        }

        .vehicle-console-header {
            display: grid;
            grid-template-columns:
                minmax(180px, 1.3fr)
                repeat(6, minmax(110px, 1fr));
            align-items: stretch;
            background: #101820;
            border-bottom: 1px solid #344250;
        }

        .console-identity {
            padding: 18px 20px;
            display: flex;
            flex-direction: column;
            justify-content: center;
            border-right: 1px solid #344250;
        }

        .console-kicker {
            color: #7f91a1;
            font-size: 10px;
            font-weight: bold;
            letter-spacing: 0.16em;
            text-transform: uppercase;
            margin-bottom: 4px;
        }

        .console-vehicle-name {
            font-size: 25px;
            font-weight: 700;
            letter-spacing: 0.03em;
        }

        .console-readout {
            min-height: 72px;
            padding: 12px 14px;
            border-right: 1px solid #283744;
            display: flex;
            flex-direction: column;
            justify-content: center;
        }

        .console-readout:last-child {
            border-right: 0;
        }

        .console-readout-label {
            color: #778999;
            font-size: 10px;
            letter-spacing: 0.11em;
            text-transform: uppercase;
            margin-bottom: 6px;
        }

        .console-state {
            font-family: monospace;
            font-weight: 700;
            font-size: 14px;
            overflow-wrap: anywhere;
        }

        .console-state.good {
            color: #61d095;
        }

        .console-state.bad {
            color: #ee6c6c;
        }

        .console-state.warn {
            color: #ffd166;
        }

        .console-state.neutral {
            color: #dce5ec;
        }


        /*
         * Live data-flow strip.
         *
         * USV:
         * perception -> follower -> bridge -> thrusters
         *
         * UAV:
         * safety -> GPS -> pre-arm -> command authorization
         */

        .console-flow {
            display: grid;
            grid-template-columns:
                repeat(4, minmax(130px, 1fr));
            background: #111b24;
            border-bottom: 1px solid #344250;
        }

        .console-flow-stage {
            position: relative;
            padding: 12px 18px;
            border-right: 1px solid #283744;
        }

        .console-flow-stage:last-child {
            border-right: 0;
        }

        .console-flow-stage:not(:last-child)::after {
            content: "›";
            position: absolute;
            z-index: 2;
            right: -7px;
            top: 50%;
            transform: translateY(-50%);
            color: #607587;
            font-size: 22px;
            font-weight: bold;
        }

        .console-flow-label {
            display: block;
            margin-bottom: 4px;
            color: #718493;
            font-size: 10px;
            letter-spacing: 0.11em;
            text-transform: uppercase;
        }


        /*
         * Main cockpit geometry:
         *
         * ┌─────────────────────────────┬──────────────┐
         * │ live vehicle information    │ controls     │
         * │                             │              │
         * └─────────────────────────────┴──────────────┘
         */

        .console-layout {
            display: grid;
            grid-template-columns:
                minmax(0, 1fr)
                350px;
            gap: 0;
            align-items: start;
        }

        .console-main-grid {
            min-width: 0;
            display: grid;
            grid-template-columns:
                repeat(2, minmax(0, 1fr));
            background: #0e161d;
        }

        .console-control-rail {
            min-width: 0;
            align-self: stretch;
            background: #121c25;
            border-left: 1px solid #344250;
        }


        /*
         * These are still called ".card" internally so none
         * of the old element IDs or update code have to change.
         * Visually they are now integrated console modules.
         */

        .console-module.card {
            margin: 0;
            border: 0;
            border-right: 1px solid #283744;
            border-bottom: 1px solid #283744;
            border-radius: 0;
            background: transparent;
            padding: 17px 19px;
            min-width: 0;
        }

        .console-module.card h2 {
            margin: 0 0 12px 0;
            color: #8ea0ae;
            font-size: 11px;
            font-weight: 700;
            text-transform: uppercase;
            letter-spacing: 0.11em;
        }

        .console-module .row {
            padding: 7px 0;
        }

        .console-module.console-wide {
            grid-column: 1 / -1;
        }

        .console-control-rail
        > .console-module.card {
            border-right: 0;
            background: transparent;
            padding: 18px;
        }

        .console-control-rail
        > .console-module.card h2 {
            color: #b8c4ce;
            font-size: 12px;
        }

        .console-control-rail
        .control-button {
            min-height: 48px;
        }


        /*
         * Primary modules get slightly stronger emphasis.
         */

        .console-flight-hud {
            background: #101a22 !important;
        }

        .console-perception {
            background: #101a22 !important;
        }

        .console-pre-arm-checks {
            background: #0e171e !important;
        }

        .console-thruster-outputs {
            background: #0e171e !important;
        }

        .console-autonomy {
            background: #0e171e !important;
        }


        /*
         * Keep the attitude display dominant on the UAV.
         */

        .console-status-panel
        .uav-hud-card {
            grid-column: 1 / -1;
        }


        /*
         * Parameters and geofence remain their own functional
         * subtabs and deliberately retain their existing card
         * presentation.
         */


        /* ==================================================
           COMPACT USV OPERATOR CONSOLE
           ================================================== */

        /*
         * The USV has many small telemetry groups.
         * Use three columns instead of two.
         */

        .usv-console-status
        .console-layout {
            grid-template-columns:
                minmax(0, 1fr)
                300px;
        }

        .usv-console-status
        .console-main-grid {
            grid-template-columns:
                repeat(3, minmax(0, 1fr));
        }


        /*
         * USV modules no longer need forced full-width rows.
         */

        .usv-console-status
        .console-module.console-wide {
            grid-column: auto;
        }


        /*
         * These generic modules duplicate information already
         * visible in the persistent operator header:
         *
         * Connection -> LINK
         * Autopilot  -> MODE / ARM
         * Power      -> POWER
         *
         * Keep their elements in the DOM so existing telemetry
         * Javascript still functions, but do not waste screen
         * space displaying them twice.
         */

        .usv-console-status
        .console-connection,

        .usv-console-status
        .console-autopilot,

        .usv-console-status
        .console-power {
            display: none;
        }


        /*
         * Tighter telemetry density.
         */

        .usv-console-status
        .console-module.card {
            padding: 12px 15px;
        }

        .usv-console-status
        .console-module.card h2 {
            margin-bottom: 8px;
        }

        .usv-console-status
        .console-module .row {
            padding: 5px 0;
        }


        /*
         * Do not stretch the control rail to match the entire
         * telemetry area. This removes the large empty vertical
         * block beneath the controls.
         */

        .usv-console-status
        .console-control-rail {
            align-self: start;
        }


        /*
         * Slightly compress the USV data-flow strip.
         */

        .usv-console-status
        .console-flow-stage {
            padding: 9px 15px;
        }


        /*
         * Compact the top status bar slightly as well.
         */

        .usv-console-status
        .console-readout {
            min-height: 60px;
            padding: 9px 11px;
        }


        @media (max-width: 1180px) {

            .vehicle-console-header {
                grid-template-columns:
                    repeat(3, 1fr);
            }

            .console-identity {
                grid-column: 1 / -1;
                border-right: 0;
                border-bottom: 1px solid #344250;
            }

            .console-layout {
                grid-template-columns: 1fr;
            }

            .console-control-rail {
                border-left: 0;
                border-top: 1px solid #344250;
            }

            .console-flow {
                grid-template-columns:
                    repeat(2, 1fr);
            }
        }


        @media (max-width: 700px) {

            .vehicle-console-header {
                grid-template-columns:
                    repeat(2, 1fr);
            }

            .console-main-grid {
                grid-template-columns: 1fr;
            }

            .console-module.console-wide {
                grid-column: auto;
            }

            .console-flow {
                grid-template-columns: 1fr;
            }

            .console-flow-stage {
                border-right: 0;
                border-bottom: 1px solid #283744;
            }

            .console-flow-stage::after {
                display: none;
            }
        }


        /* ==================================================
           UAV FLIGHT HUD
           ================================================== */

        .uav-hud-card {
            grid-column: span 2;
        }

        .uav-hud-grid {
            display: grid;
            grid-template-columns:
                minmax(280px, 1.25fr)
                minmax(220px, 1fr);
            gap: 20px;
            align-items: center;
        }

        .uav-horizon {
            position: relative;
            width: min(100%, 360px);
            aspect-ratio: 1 / 1;
            margin: 0 auto;
            overflow: hidden;
            border-radius: 50%;
            border: 4px solid #52677a;
            background: #397caf;
            box-shadow:
                inset 0 0 20px rgba(0, 0, 0, 0.7),
                0 0 12px rgba(0, 0, 0, 0.35);
        }

        .uav-horizon-world {
            position: absolute;
            width: 180%;
            height: 180%;
            left: -40%;
            top: -40%;
            transform-origin: 50% 50%;
            will-change: transform;
        }

        .uav-horizon-sky {
            position: absolute;
            left: 0;
            top: 0;
            width: 100%;
            height: 50%;
            background: #397caf;
        }

        .uav-horizon-ground {
            position: absolute;
            left: 0;
            top: 50%;
            width: 100%;
            height: 50%;
            background: #8a5b32;
        }

        .uav-horizon-line {
            position: absolute;
            left: 0;
            top: calc(50% - 2px);
            width: 100%;
            height: 4px;
            background: white;
            box-shadow: 0 0 3px black;
        }

        .uav-aircraft-symbol {
            position: absolute;
            z-index: 10;
            left: 50%;
            top: 50%;
            width: 58%;
            height: 4px;
            transform: translate(-50%, -50%);
            background:
                linear-gradient(
                    to right,
                    #ffd84d 0 36%,
                    transparent 36% 64%,
                    #ffd84d 64% 100%
                );
            box-shadow: 0 0 2px #000;
        }

        .uav-aircraft-symbol:after {
            content: "";
            position: absolute;
            width: 8px;
            height: 8px;
            border: 3px solid #ffd84d;
            border-radius: 50%;
            left: 50%;
            top: 50%;
            transform: translate(-50%, -50%);
        }

        .uav-hud-big {
            font-family: monospace;
            font-size: 24px;
            font-weight: bold;
        }

        .uav-hud-label {
            color: #9eacb9;
            font-size: 12px;
            text-transform: uppercase;
            letter-spacing: 0.08em;
        }

        .uav-hud-readout {
            margin-bottom: 16px;
        }

        .uav-warning {
            color: #ee6c6c;
        }

        .uav-good {
            color: #61d095;
        }

        @media (max-width: 800px) {
            .uav-hud-card {
                grid-column: span 1;
            }

            .uav-hud-grid {
                grid-template-columns: 1fr;
            }
        }

        /* ==================================================
           ROBOCOMMAND
           ================================================== */

        .rc-grid {
            display: grid;
            grid-template-columns:
                repeat(auto-fit, minmax(300px, 1fr));
            gap: 18px;
        }

        .rc-wide {
            grid-column: 1 / -1;
        }

        .rc-traffic-grid {
            display: grid;
            grid-template-columns:
                repeat(auto-fit, minmax(420px, 1fr));
            gap: 18px;
        }

        .rc-log {
            max-height: 55vh;
            overflow-y: auto;
            font-family: monospace;
            font-size: 12px;
        }

        .rc-entry {
            padding: 8px 0;
            border-bottom: 1px solid #283744;
        }

        .rc-entry summary {
            cursor: pointer;
            line-height: 1.4;
        }

        .rc-entry pre {
            white-space: pre-wrap;
            overflow-wrap: anywhere;
            background: #10171e;
            padding: 10px;
            border-radius: 5px;
        }

        .rc-actions {
            display: grid;
            grid-template-columns:
                repeat(auto-fit, minmax(180px, 1fr));
            gap: 10px;
        }


        /* ==================================================
           VEHICLE SUBTABS / PARAMETER EDITOR
           ================================================== */

        .vehicle-subtabs {
            display: flex;
            gap: 8px;
            margin-bottom: 18px;
            border-bottom: 1px solid #344250;
        }

        .vehicle-subtab {
            padding: 10px 18px;
            border: 0;
            border-bottom: 3px solid transparent;
            background: transparent;
            color: #9eacb9;
            cursor: pointer;
            font-size: 14px;
            font-weight: bold;
        }

        .vehicle-subtab.active {
            color: #ffffff;
            border-bottom-color: #61d095;
        }

        .vehicle-panel {
            display: none;
        }

        .vehicle-panel.active {
            display: block;
        }

        .parameter-toolbar {
            display: grid;
            grid-template-columns:
                minmax(240px, 1fr)
                auto;
            gap: 12px;
            margin-bottom: 14px;
        }

        .parameter-search,
        .parameter-value-input {
            padding: 9px 10px;
            border: 1px solid #46596b;
            border-radius: 5px;
            background: #10171e;
            color: #e8edf2;
            font-family: monospace;
        }

        .parameter-value-input {
            width: 150px;
        }

        .parameter-table-wrap {
            width: 100%;
            max-height: calc(100vh - 300px);
            overflow: auto;
            border: 1px solid #344250;
            border-radius: 6px;
        }

        .parameter-table {
            width: 100%;
            border-collapse: collapse;
            font-family: monospace;
            font-size: 13px;
        }

        .parameter-table th,
        .parameter-table td {
            padding: 9px 10px;
            border-bottom: 1px solid #283744;
            text-align: left;
            white-space: nowrap;
        }

        .parameter-table th {
            position: sticky;
            top: 0;
            z-index: 2;
            background: #202d39;
        }

        .parameter-table tr:hover {
            background: #202a34;
        }

        .parameter-name {
            font-weight: bold;
        }

        .parameter-write-button {
            padding: 7px 12px;
            border: 1px solid #46596b;
            border-radius: 5px;
            background: #26384a;
            color: #eeeeee;
            cursor: pointer;
            font-weight: bold;
        }

        .parameter-write-button:disabled {
            opacity: 0.40;
            cursor: not-allowed;
        }

        .parameter-status {
            margin-bottom: 12px;
            padding: 10px;
            border: 1px solid #344250;
            border-radius: 5px;
            background: #10171e;
            font-family: monospace;
        }


        /* ==================================================
           UAV GEOFENCE
           ================================================== */

        .geofence-map {
            width: 100%;
            height: 430px;
            margin-top: 14px;
            border: 1px solid #344250;
            border-radius: 6px;
        }

        .geofence-table {
            width: 100%;
            border-collapse: collapse;
            margin-top: 14px;
            font-family: monospace;
            font-size: 13px;
        }

        .geofence-table th,
        .geofence-table td {
            padding: 9px 10px;
            border-bottom: 1px solid #283744;
            text-align: left;
        }

        .geofence-status {
            margin-bottom: 12px;
            padding: 10px;
            border: 1px solid #344250;
            border-radius: 5px;
            background: #10171e;
            font-family: monospace;
        }


        /* LEGACY UAV GEOFENCE TAB HIDDEN */
        #uav-geofence-tab,
        #uav-geofence-panel {
            display: none !important;
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

    <button class="tab"
            data-page="robocommand-page">
        RoboCommand
    </button>
</div>

<div id="map-page"
     class="page active">

    <div id="map"></div>


    <div
        class="map-geofence-card"
        id="overview-geofence-card">

        <h2>UAV Geofence</h2>


        <div class="row">
            <span>Autopilot</span>
            <span
                class="value"
                id="full-fence-status">
                NOT LOADED
            </span>
        </div>

        <div class="row">
            <span>Zones</span>
            <span
                class="value"
                id="full-fence-zone-count">
                0
            </span>
        </div>

        <div class="row">
            <span>Hard Fence</span>
            <span
                class="value"
                id="full-fence-enabled">
                --
            </span>
        </div>


        <button
            class="control-button"
            onclick="fullFenceLoadFromAutopilot()">
            LOAD FROM PIXHAWK
        </button>


        <hr style="
            border:0;
            border-top:1px solid #344250;
            margin:14px 0;
        ">


        <div style="font-weight:bold;">
            Add Zone
        </div>

        <select
            id="full-fence-new-type"
            style="
                width:100%;
                margin-top:8px;
                padding:8px;
                background:#10171e;
                color:#e8edf2;
                border:1px solid #46596b;
                border-radius:5px;
            ">

            <option value="inclusion_polygon">
                Inclusion Polygon
            </option>

            <option value="exclusion_polygon">
                Exclusion Polygon
            </option>

            <option value="inclusion_circle">
                Inclusion Circle
            </option>

            <option value="exclusion_circle">
                Exclusion Circle
            </option>

        </select>


        <button
            class="control-button enable-button"
            onclick="fullFenceAddZone()">
            ADD ZONE
        </button>


        <div
            id="full-fence-zones"
            style="
                margin-top:12px;
                max-height:190px;
                overflow:auto;
            ">
            No zones loaded.
        </div>


        <div
            id="full-fence-active-controls"
            style="margin-top:10px;">

            <button
                class="control-button"
                onclick="fullFenceFinishDrawing()">
                FINISH DRAWING
            </button>

            <button
                class="control-button"
                onclick="fullFenceUndo()">
                UNDO VERTEX
            </button>

            <button
                class="control-button disarm-button"
                onclick="fullFenceDeleteActive()">
                DELETE SELECTED ZONE
            </button>

        </div>


        <button
            class="control-button enable-button"
            onclick="fullFenceUploadAll()">
            UPLOAD ALL & VERIFY
        </button>


        <hr style="
            border:0;
            border-top:1px solid #344250;
            margin:14px 0;
        ">


        <details>
            <summary
                style="
                    cursor:pointer;
                    font-weight:bold;
                    padding:6px 0;
                ">
                Fence Settings
            </summary>


            <div style="margin-top:10px;">

                <label>
                    Breach Action
                </label>

                <select
                    id="full-fence-action"
                    style="
                        width:100%;
                        padding:7px;
                        margin:4px 0 10px;
                        background:#10171e;
                        color:#e8edf2;
                    ">
                    <option value="0">Report Only</option>
                    <option value="1">RTL / Land</option>
                    <option value="2">Land</option>
                    <option value="3">SmartRTL / RTL / Land</option>
                    <option value="4">Brake / Land</option>
                    <option value="5">SmartRTL / Land</option>
                </select>


                <label>Maximum Altitude (m)</label>
                <input
                    id="full-fence-alt-max"
                    type="number"
                    step="1"
                    class="parameter-search"
                    style="width:100%;margin:4px 0 10px;">


                <label>Minimum Altitude (m)</label>
                <input
                    id="full-fence-alt-min"
                    type="number"
                    step="1"
                    class="parameter-search"
                    style="width:100%;margin:4px 0 10px;">


                <label>Fence Margin (m)</label>
                <input
                    id="full-fence-margin"
                    type="number"
                    step="0.5"
                    class="parameter-search"
                    style="width:100%;margin:4px 0 10px;">


                <label>Home Circle Radius (m)</label>
                <input
                    id="full-fence-radius"
                    type="number"
                    step="1"
                    class="parameter-search"
                    style="width:100%;margin:4px 0 10px;">


                <div style="margin-top:8px;font-weight:bold;">
                    Enabled Fence Types
                </div>

                <label style="display:block;margin-top:6px;">
                    <input
                        id="full-type-max"
                        type="checkbox">
                    Maximum Altitude
                </label>

                <label style="display:block;margin-top:6px;">
                    <input
                        id="full-type-circle"
                        type="checkbox">
                    Home-Centered Circle
                </label>

                <label style="display:block;margin-top:6px;">
                    <input
                        id="full-type-zones"
                        type="checkbox">
                    Inclusion / Exclusion Zones
                </label>

                <label style="display:block;margin-top:6px;">
                    <input
                        id="full-type-min"
                        type="checkbox">
                    Minimum Altitude
                </label>


                <label
                    style="
                        display:block;
                        margin-top:12px;
                    ">
                    <input
                        id="full-fence-union"
                        type="checkbox">
                    Use UNION of inclusion areas
                </label>


                <button
                    class="control-button"
                    onclick="fullFenceApplySettings()">
                    APPLY SETTINGS
                </button>

            </div>

        </details>


        <button
            class="control-button enable-button"
            onclick="fullFenceSetEnabled(true)">
            ENABLE HARD FENCE
        </button>

        <button
            class="control-button disarm-button"
            onclick="fullFenceSetEnabled(false)">
            DISABLE HARD FENCE
        </button>


        <div
            class="map-geofence-message"
            id="full-fence-message">
            Loading native fence...
        </div>

    </div>


    <div id="summary">
        Waiting for telemetry...
    </div>
</div>


<div id="robocommand-page"
     class="page">

    <div class="rc-grid">

        <div class="card">
            <h2>RoboCommand Connection</h2>

            <div class="row">
                <span>Status</span>
                <span class="value"
                      id="rc-status">
                    DISCONNECTED
                </span>
            </div>

            <div class="row">
                <span>Broker</span>
                <span class="value"
                      id="rc-broker">
                    --
                </span>
            </div>

            <div class="row">
                <span>Team ID</span>
                <span class="value"
                      id="rc-team">
                    --
                </span>
            </div>

            <div class="row">
                <span>Last RX</span>
                <span class="value"
                      id="rc-last-rx">
                    --
                </span>
            </div>

            <div class="row">
                <span>Last TX</span>
                <span class="value"
                      id="rc-last-tx">
                    --
                </span>
            </div>
        </div>


        <div class="card">
            <h2>Course / Run</h2>

            <div class="row">
                <span>Course ID</span>
                <span class="value"
                      id="rc-course">
                    --
                </span>
            </div>

            <div class="row">
                <span>Pinger</span>
                <span class="value"
                      id="rc-pinger">
                    --
                </span>
            </div>

            <div class="row">
                <span>Run State</span>
                <span class="value"
                      id="rc-run-state">
                    WAITING
                </span>
            </div>

            <div class="row">
                <span>Declaration Seq</span>
                <span class="value"
                      id="rc-declaration-seq">
                    --
                </span>
            </div>

            <div class="row">
                <span>Run ID</span>
                <span class="value"
                      id="rc-run-id">
                    --
                </span>
            </div>

            <div class="row">
                <span>Last Command</span>
                <span class="value"
                      id="rc-last-command">
                    --
                </span>
            </div>
        </div>


        <div class="card">
            <h2>Vehicle Reports</h2>

            <div class="row">
                <span>USV1 State</span>
                <span class="value"
                      id="rc-usv-state">
                    UNKNOWN
                </span>
            </div>

            <div class="row">
                <span>USV1 Last Heartbeat</span>
                <span class="value"
                      id="rc-usv-age">
                    --
                </span>
            </div>

            <div class="row">
                <span>UAV1 State</span>
                <span class="value"
                      id="rc-uav-state">
                    UNKNOWN
                </span>
            </div>

            <div class="row">
                <span>UAV1 Last Heartbeat</span>
                <span class="value"
                      id="rc-uav-age">
                    --
                </span>
            </div>
        </div>


        <div class="card">
            <h2>PoR Technical Readiness</h2>

            <div class="row">
                <span>Overall</span>
                <span class="value"
                      id="rc-por-overall">
                    NOT READY
                </span>
            </div>

            <div class="row">
                <span>RoboCommand Broker</span>
                <span class="value"
                      id="rc-por-broker">
                    NOT READY
                </span>
            </div>

            <div class="row">
                <span>Course Definition</span>
                <span class="value"
                      id="rc-por-course">
                    NOT READY
                </span>
            </div>

            <div class="row">
                <span>USV Communications</span>
                <span class="value"
                      id="rc-por-usv-comms">
                    NOT READY
                </span>
            </div>

            <div class="row">
                <span>UAV Communications</span>
                <span class="value"
                      id="rc-por-uav-comms">
                    NOT READY
                </span>
            </div>

            <div class="row">
                <span>USV Mission Process</span>
                <span class="value"
                      id="rc-por-usv-mission">
                    STOPPED
                </span>
            </div>

            <div class="row">
                <span>UAV Mission Process</span>
                <span class="value"
                      id="rc-por-uav-mission">
                    STOPPED
                </span>
            </div>

            <div class="row">
                <span>Native UAV Geofence</span>
                <span class="value"
                      id="rc-por-geofence">
                    NOT VERIFIED
                </span>
            </div>

            <div
                class="control-message"
                id="rc-por-reason">
                Waiting for readiness data.
            </div>
        </div>



        <div class="card">
            <h2>RoboCommand Run Control</h2>

            <div class="row">
                <span>Declaration</span>
                <span
                    class="value"
                    id="rc-control-declaration">
                    NOT SENT
                </span>
            </div>

            <div class="row">
                <span>Declaration Seq</span>
                <span
                    class="value"
                    id="rc-control-declaration-seq">
                    --
                </span>
            </div>

            <div class="row">
                <span>USV1 Heartbeat</span>
                <span
                    class="value"
                    id="rc-control-usv">
                    UNKNOWN
                </span>
            </div>

            <div class="row">
                <span>UAV1 Heartbeat</span>
                <span
                    class="value"
                    id="rc-control-uav">
                    UNKNOWN
                </span>
            </div>

            <div class="row">
                <span>RunStart</span>
                <span
                    class="value"
                    id="rc-control-runstart">
                    WAITING
                </span>
            </div>

            <div class="row">
                <span>Sequence Verification</span>
                <span
                    class="value"
                    id="rc-control-seq-check">
                    WAITING
                </span>
            </div>

            <div class="row">
                <span>Run ID</span>
                <span
                    class="value"
                    id="rc-control-run-id">
                    --
                </span>
            </div>

            <div
                class="control-message"
                id="rc-control-guidance">
                Waiting for run state.
            </div>

            <div class="control-message">
                Vehicle AUTO status comes from the real
                vehicle autonomy state. This panel does
                not arm, change mode, or enable autonomy.
            </div>
        </div>


        <div class="card">
            <h2>Official Communications PoR Log</h2>

            <div class="row">
                <span>rc-test Server</span>
                <span
                    class="value"
                    id="rc-evidence-server">
                    UNKNOWN
                </span>
            </div>

            <div class="row">
                <span>Current Log</span>
                <span
                    class="value"
                    id="rc-evidence-file">
                    --
                </span>
            </div>

            <div class="row">
                <span>Declared Vehicles</span>
                <span
                    class="value"
                    id="rc-evidence-vehicles">
                    --
                </span>
            </div>

            <div class="row">
                <span>RunStart Seen</span>
                <span
                    class="value"
                    id="rc-evidence-runstart">
                    NO
                </span>
            </div>

            <div class="row">
                <span>Declaration Seq</span>
                <span
                    class="value"
                    id="rc-evidence-declaration-seq">
                    --
                </span>
            </div>

            <div class="row">
                <span>RunStart Seq</span>
                <span
                    class="value"
                    id="rc-evidence-runstart-seq">
                    --
                </span>
            </div>

            <div class="row">
                <span>Sequence Match</span>
                <span
                    class="value"
                    id="rc-evidence-sequence">
                    WAITING
                </span>
            </div>

            <div class="row">
                <span>Run ID</span>
                <span
                    class="value"
                    id="rc-evidence-run-id">
                    --
                </span>
            </div>

            <div class="row">
                <span>Communications PoR</span>
                <span
                    class="value"
                    id="rc-evidence-result">
                    INCOMPLETE
                </span>
            </div>

            <div
                class="control-message"
                id="rc-evidence-message">
                Waiting for official rc-test evidence.
            </div>

            <button
                class="control-button"
                onclick="rcViewOfficialLog()">
                VIEW LOG
            </button>

            <button
                class="control-button enable-button"
                id="rc-download-log-button"
                onclick="rcDownloadOfficialLog()">
                DOWNLOAD OFFICIAL LOG
            </button>

            <button
                class="control-button reset-button"
                onclick="rcStartNewPorTest()">
                START NEW PoR TEST
            </button>

            <pre
                id="rc-official-log-view"
                style="
                    display:none;
                    max-height:500px;
                    overflow:auto;
                    white-space:pre-wrap;
                    margin-top:12px;
                    background:#0b1016;
                    padding:12px;
                    border-radius:6px;
                ">
            </pre>
        </div>


        <div class="card">
            <h2>OCS Actions</h2>

            <div class="rc-actions">

                <button
                    class="control-button enable-button"
                    id="rc-declaration-button"
                    onclick="rcSendDeclaration()">
                    SEND RUN DECLARATION
                </button>

                <button
                    class="control-button reset-button"
                    onclick="rcReconnect()">
                    RECONNECT
                </button>

                <button
                    class="control-button"
                    onclick="rcClearHistory()">
                    CLEAR DISPLAY LOG
                </button>

            </div>

            <div class="control-message"
                 id="rc-action-message">
                No RoboCommand action sent.
            </div>
        </div>


        <div class="rc-wide rc-traffic-grid">

            <div class="card">
                <h2>Incoming from RoboCommand</h2>

                <div class="rc-log"
                     id="rc-rx-log">
                    No messages received.
                </div>
            </div>


            <div class="card">
                <h2>Outgoing to RoboCommand</h2>

                <div class="rc-log"
                     id="rc-tx-log">
                    No messages sent.
                </div>
            </div>

        </div>

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
const vehicleParameterState = {};
const vehicleGeofenceState = {};
const vehicleGeofenceMaps = {};
const vehicleGeofenceLayers = {};

let mapCentered = false;


/* ==========================================================
   OVERVIEW MAP GEOFENCE STATE
   ========================================================== */

let overviewFenceState = null;

let overviewFenceEditor = [];

let overviewFencePolygon = null;

let overviewFenceMarkers = [];

let overviewFenceDrawing = false;



/* ==========================================================
   FULL OVERVIEW MAP UAV GEOFENCE EDITOR
   ========================================================== */

let fullFenceState = null;
let fullFenceZones = [];
let fullFenceActive = -1;
let fullFenceDrawing = false;

let fullFenceLayers = [];
let fullFenceEditMarkers = [];


function fullFenceMessage(text) {

    setText(
        "full-fence-message",
        String(text)
    );
}


function fullFenceClearMap() {

    fullFenceLayers.forEach(
        layer => {
            map.removeLayer(layer);
        }
    );

    fullFenceEditMarkers.forEach(
        layer => {
            map.removeLayer(layer);
        }
    );

    fullFenceLayers = [];
    fullFenceEditMarkers = [];
}


function fullFenceTypeLabel(type) {

    const labels = {
        inclusion_polygon:
            "Inclusion Polygon",

        exclusion_polygon:
            "Exclusion Polygon",

        inclusion_circle:
            "Inclusion Circle",

        exclusion_circle:
            "Exclusion Circle"
    };

    return labels[type] ?? type;
}


function fullFenceRender() {

    fullFenceClearMap();


    fullFenceZones.forEach(
        (zone, zoneIndex) => {

            const active =
                zoneIndex === fullFenceActive;


            if (
                zone.type === "inclusion_polygon"
                ||
                zone.type === "exclusion_polygon"
            ) {

                const points =
                    Array.isArray(
                        zone.points
                    )
                        ? zone.points
                        : [];


                const latLngs =
                    points.map(
                        point => [
                            Number(
                                point.latitude
                            ),
                            Number(
                                point.longitude
                            )
                        ]
                    );


                if (latLngs.length >= 3) {

                    const polygon =
                        L.polygon(
                            latLngs,
                            {
                                weight:
                                    active ? 5 : 3,

                                fillOpacity:
                                    zone.type
                                    === "inclusion_polygon"
                                        ? 0.13
                                        : 0.05,

                                dashArray:
                                    zone.type
                                    === "exclusion_polygon"
                                        ? "8 6"
                                        : null
                            }
                        )
                        .addTo(map);

                    polygon.on(
                        "click",
                        event => {
                            L.DomEvent
                                .stopPropagation(
                                    event
                                );

                            fullFenceSelect(
                                zoneIndex
                            );
                        }
                    );

                    fullFenceLayers.push(
                        polygon
                    );


                } else if (
                    latLngs.length >= 2
                ) {

                    const line =
                        L.polyline(
                            latLngs,
                            {
                                weight: 4
                            }
                        )
                        .addTo(map);

                    fullFenceLayers.push(
                        line
                    );
                }


                if (active) {

                    points.forEach(
                        (point, pointIndex) => {

                            const marker =
                                L.marker(
                                    [
                                        Number(
                                            point.latitude
                                        ),
                                        Number(
                                            point.longitude
                                        )
                                    ],
                                    {
                                        draggable: true
                                    }
                                )
                                .addTo(map);


                            marker.bindTooltip(
                                `Vertex ${pointIndex + 1}`
                            );


                            marker.on(
                                "dragend",
                                event => {

                                    const pos =
                                        event.target
                                        .getLatLng();

                                    zone.points[
                                        pointIndex
                                    ] = {
                                        latitude:
                                            pos.lat,

                                        longitude:
                                            pos.lng
                                    };

                                    fullFenceRender();
                                }
                            );


                            marker.on(
                                "contextmenu",
                                event => {

                                    if (
                                        event.originalEvent
                                    ) {
                                        event.originalEvent
                                        .preventDefault();
                                    }

                                    zone.points.splice(
                                        pointIndex,
                                        1
                                    );

                                    fullFenceRender();
                                }
                            );


                            fullFenceEditMarkers.push(
                                marker
                            );
                        }
                    );
                }
            }


            if (
                zone.type === "inclusion_circle"
                ||
                zone.type === "exclusion_circle"
            ) {

                if (!zone.center) {
                    return;
                }


                const radius =
                    Number(
                        zone.radius_m
                        ?? 20
                    );


                const circle =
                    L.circle(
                        [
                            Number(
                                zone.center.latitude
                            ),
                            Number(
                                zone.center.longitude
                            )
                        ],
                        {
                            radius: radius,
                            weight:
                                active ? 5 : 3,

                            fillOpacity:
                                zone.type
                                === "inclusion_circle"
                                    ? 0.13
                                    : 0.05,

                            dashArray:
                                zone.type
                                === "exclusion_circle"
                                    ? "8 6"
                                    : null
                        }
                    )
                    .addTo(map);


                circle.on(
                    "click",
                    event => {

                        L.DomEvent
                            .stopPropagation(
                                event
                            );

                        fullFenceSelect(
                            zoneIndex
                        );
                    }
                );


                fullFenceLayers.push(
                    circle
                );


                if (active) {

                    const marker =
                        L.marker(
                            [
                                Number(
                                    zone.center.latitude
                                ),
                                Number(
                                    zone.center.longitude
                                )
                            ],
                            {
                                draggable: true
                            }
                        )
                        .addTo(map);


                    marker.bindTooltip(
                        "Circle center"
                    );


                    marker.on(
                        "dragend",
                        event => {

                            const pos =
                                event.target
                                .getLatLng();

                            zone.center = {
                                latitude:
                                    pos.lat,

                                longitude:
                                    pos.lng
                            };

                            fullFenceRender();
                        }
                    );


                    fullFenceEditMarkers.push(
                        marker
                    );
                }
            }
        }
    );


    fullFenceRenderList();


    setText(
        "full-fence-zone-count",
        String(
            fullFenceZones.length
        )
    );
}


function fullFenceRenderList() {

    const container =
        document.getElementById(
            "full-fence-zones"
        );


    if (!container) {
        return;
    }


    if (
        fullFenceZones.length === 0
    ) {

        container.innerHTML =
            "<div class='value'>"
            + "No fence zones."
            + "</div>";

        return;
    }


    container.innerHTML =
        fullFenceZones
        .map(
            (zone, index) => {

                const selected =
                    index === fullFenceActive;


                let detail = "";


                if (
                    zone.type.endsWith(
                        "_polygon"
                    )
                ) {

                    detail =
                        `${
                            zone.points
                            ?.length ?? 0
                        } vertices`;

                } else {

                    detail =
                        `${Number(
                            zone.radius_m
                            ?? 0
                        ).toFixed(1)} m radius`;
                }


                const radiusEditor =
                    (
                        selected
                        &&
                        zone.type.endsWith(
                            "_circle"
                        )
                    )
                    ? `
                        <div
                            style="
                                margin-top:7px;
                                display:flex;
                                gap:6px;
                                align-items:center;
                            ">

                            <span>
                                Radius:
                            </span>

                            <input
                                type="number"
                                min="1"
                                step="1"
                                value="${
                                    Number(
                                        zone.radius_m
                                        ?? 20
                                    )
                                }"
                                style="
                                    width:90px;
                                    background:#10171e;
                                    color:white;
                                    border:1px solid #46596b;
                                    padding:5px;
                                "
                                onchange="
                                    fullFenceSetRadius(
                                        ${index},
                                        this.value
                                    )
                                ">

                            <span>m</span>

                        </div>
                    `
                    : "";


                return `
                    <div
                        style="
                            border:
                                1px solid ${
                                    selected
                                        ? "#b5c7d8"
                                        : "#344250"
                                };
                            border-radius:5px;
                            padding:8px;
                            margin-bottom:7px;
                            cursor:pointer;
                        "
                        onclick="
                            fullFenceSelect(
                                ${index}
                            )
                        ">

                        <div
                            style="
                                font-weight:bold;
                            ">
                            ${index + 1}.
                            ${
                                fullFenceTypeLabel(
                                    zone.type
                                )
                            }
                        </div>

                        <div
                            class="value"
                            style="
                                margin-top:4px;
                            ">
                            ${detail}
                        </div>

                        ${radiusEditor}

                        ${
                            selected
                            ? `
                                <div
                                    style="
                                        display:flex;
                                        gap:5px;
                                        margin-top:7px;
                                    ">

                                    <button
                                        onclick="
                                            event.stopPropagation();
                                            fullFenceMove(
                                                ${index},
                                                -1
                                            );
                                        ">
                                        ↑
                                    </button>

                                    <button
                                        onclick="
                                            event.stopPropagation();
                                            fullFenceMove(
                                                ${index},
                                                1
                                            );
                                        ">
                                        ↓
                                    </button>

                                </div>
                            `
                            : ""
                        }

                    </div>
                `;
            }
        )
        .join("");
}


function fullFenceSelect(index) {

    if (
        index < 0
        ||
        index >= fullFenceZones.length
    ) {
        return;
    }

    fullFenceActive = index;

    fullFenceDrawing = false;

    fullFenceRender();


    const zone =
        fullFenceZones[index];


    fullFenceMessage(
        "Selected "
        + fullFenceTypeLabel(
            zone.type
        )
        + ". Drag its map handles "
        + "to edit it."
    );
}


function fullFenceAddZone() {

    const select =
        document.getElementById(
            "full-fence-new-type"
        );

    const type =
        select?.value
        ?? "inclusion_polygon";


    let zone;


    if (
        type.endsWith(
            "_polygon"
        )
    ) {

        zone = {
            type: type,
            points: []
        };

    } else {

        zone = {
            type: type,
            center: null,
            radius_m: 20.0
        };
    }


    fullFenceZones.push(
        zone
    );

    fullFenceActive =
        fullFenceZones.length - 1;

    fullFenceDrawing = true;


    fullFenceRender();


    if (
        type.endsWith(
            "_polygon"
        )
    ) {

        fullFenceMessage(
            "DRAWING "
            + fullFenceTypeLabel(type)
            + ": click the map for each vertex."
        );

    } else {

        fullFenceMessage(
            "Click the map once "
            + "to place the circle center."
        );
    }
}


function fullFenceFinishDrawing() {

    fullFenceDrawing = false;

    fullFenceRender();


    if (
        fullFenceActive >= 0
    ) {

        fullFenceMessage(
            "Zone editing finished. "
            + "UPLOAD ALL & VERIFY when ready."
        );
    }
}


function fullFenceUndo() {

    if (
        fullFenceActive < 0
        ||
        fullFenceActive
        >= fullFenceZones.length
    ) {
        return;
    }


    const zone =
        fullFenceZones[
            fullFenceActive
        ];


    if (
        zone.type.endsWith(
            "_polygon"
        )
        &&
        zone.points?.length
    ) {

        zone.points.pop();

        fullFenceRender();
    }
}


function fullFenceDeleteActive() {

    if (
        fullFenceActive < 0
        ||
        fullFenceActive
        >= fullFenceZones.length
    ) {
        return;
    }


    const zone =
        fullFenceZones[
            fullFenceActive
        ];


    if (
        !confirm(
            "Delete "
            + fullFenceTypeLabel(
                zone.type
            )
            + " from the local editor?\\n\\n"
            + "The Pixhawk is not modified "
            + "until UPLOAD ALL & VERIFY."
        )
    ) {
        return;
    }


    fullFenceZones.splice(
        fullFenceActive,
        1
    );


    if (
        fullFenceZones.length === 0
    ) {

        fullFenceActive = -1;

    } else {

        fullFenceActive =
            Math.min(
                fullFenceActive,
                fullFenceZones.length - 1
            );
    }


    fullFenceDrawing = false;

    fullFenceRender();
}


function fullFenceMove(
    index,
    delta
) {

    const target =
        index + delta;


    if (
        target < 0
        ||
        target >= fullFenceZones.length
    ) {
        return;
    }


    const temp =
        fullFenceZones[index];

    fullFenceZones[index] =
        fullFenceZones[target];

    fullFenceZones[target] =
        temp;


    fullFenceActive =
        target;


    fullFenceRender();
}


function fullFenceSetRadius(
    index,
    value
) {

    const radius =
        Number(value);


    if (
        !Number.isFinite(radius)
        ||
        radius <= 0
    ) {
        return;
    }


    if (
        index < 0
        ||
        index >= fullFenceZones.length
    ) {
        return;
    }


    fullFenceZones[index]
        .radius_m = radius;


    fullFenceRender();
}


function fullFenceFitBounds() {

    const positions = [];


    fullFenceZones.forEach(
        zone => {

            if (
                zone.type.endsWith(
                    "_polygon"
                )
            ) {

                (
                    zone.points
                    ?? []
                )
                .forEach(
                    point => {
                        positions.push([
                            Number(
                                point.latitude
                            ),
                            Number(
                                point.longitude
                            )
                        ]);
                    }
                );

            } else if (
                zone.center
            ) {

                positions.push([
                    Number(
                        zone.center.latitude
                    ),
                    Number(
                        zone.center.longitude
                    )
                ]);
            }
        }
    );


    if (
        positions.length > 0
    ) {

        const bounds =
            L.latLngBounds(
                positions
            );


        if (bounds.isValid()) {

            map.fitBounds(
                bounds,
                {
                    padding:
                        [50, 380],
                    maxZoom: 19
                }
            );
        }
    }
}


function fullFencePopulateSettings(
    settings
) {

    settings =
        settings ?? {};


    const setValue = (
        id,
        value
    ) => {

        const element =
            document.getElementById(id);

        if (
            element
            &&
            value !== null
            &&
            value !== undefined
        ) {
            element.value =
                String(value);
        }
    };


    setValue(
        "full-fence-action",
        settings.FENCE_ACTION
    );

    setValue(
        "full-fence-alt-max",
        settings.FENCE_ALT_MAX
    );

    setValue(
        "full-fence-alt-min",
        settings.FENCE_ALT_MIN
    );

    setValue(
        "full-fence-margin",
        settings.FENCE_MARGIN
    );

    setValue(
        "full-fence-radius",
        settings.FENCE_RADIUS
    );


    const fenceType =
        Number(
            settings.FENCE_TYPE
            ?? 0
        );


    const typeBits = [
        ["full-type-max", 1],
        ["full-type-circle", 2],
        ["full-type-zones", 4],
        ["full-type-min", 8],
    ];


    typeBits.forEach(
        ([id, bit]) => {

            const element =
                document.getElementById(id);

            if (element) {

                element.checked =
                    (
                        fenceType & bit
                    ) !== 0;
            }
        }
    );


    const options =
        Number(
            settings.FENCE_OPTIONS
            ?? 0
        );


    const union =
        document.getElementById(
            "full-fence-union"
        );


    if (union) {

        // FENCE_OPTIONS value 2:
        // combine inclusion regions as a union.
        union.checked =
            (
                options & 2
            ) !== 0;
    }


    const enabled =
        Number(
            settings.FENCE_ENABLE
            ?? 0
        ) === 1;


    const enabledText =
        document.getElementById(
            "full-fence-enabled"
        );


    if (enabledText) {

        enabledText.textContent =
            enabled
                ? "ENABLED"
                : "DISABLED";

        enabledText.className =
            "value "
            + (
                enabled
                    ? "map-geofence-enabled"
                    : "map-geofence-disabled"
            );
    }
}


async function fullFenceLoadFromAutopilot(
    refresh = true
) {

    fullFenceMessage(
        "Downloading native fence "
        + "from Pixhawk..."
    );


    try {

        const response =
            await fetch(
                refresh
                    ? (
                        "/api/vehicles/uav/"
                        + "geofence/refresh"
                    )
                    : (
                        "/api/vehicles/uav/"
                        + "geofence"
                    ),
                {
                    method:
                        refresh
                            ? "POST"
                            : "GET",

                    cache:
                        "no-store"
                }
            );


        const data =
            await response.json();


        fullFenceState =
            data;


        if (!data.available) {

            setText(
                "full-fence-status",
                "OFFLINE"
            );

            fullFenceMessage(
                data.message
                ?? "UAV autopilot offline."
            );

            return data;
        }


        if (!data.complete) {

            setText(
                "full-fence-status",
                "INCOMPLETE"
            );

            fullFenceMessage(
                data.message
                ?? "Fence download incomplete."
            );

            return data;
        }


        fullFenceZones =
            JSON.parse(
                JSON.stringify(
                    data.zones
                    ?? []
                )
            );


        fullFenceActive =
            fullFenceZones.length
                ? 0
                : -1;


        fullFenceDrawing =
            false;


        setText(
            "full-fence-status",
            "LOADED"
        );


        fullFencePopulateSettings(
            data.settings
        );


        fullFenceRender();

        fullFenceFitBounds();


        fullFenceMessage(
            `Loaded ${
                fullFenceZones.length
            } zone(s), ${
                data.count
            } MAVLink item(s).`
        );


        return data;


    } catch (error) {

        setText(
            "full-fence-status",
            "ERROR"
        );


        fullFenceMessage(
            "Fence load failed: "
            + String(error)
        );


        return null;
    }
}


async function fullFenceUploadAll() {

    fullFenceDrawing = false;


    for (
        let i = 0;
        i < fullFenceZones.length;
        i += 1
    ) {

        const zone =
            fullFenceZones[i];


        if (
            zone.type.endsWith(
                "_polygon"
            )
            &&
            (
                !Array.isArray(
                    zone.points
                )
                ||
                zone.points.length < 3
            )
        ) {

            fullFenceMessage(
                `Zone ${i + 1} requires `
                + "at least 3 vertices."
            );

            return;
        }


        if (
            zone.type.endsWith(
                "_circle"
            )
            &&
            !zone.center
        ) {

            fullFenceMessage(
                `Zone ${i + 1} requires `
                + "a circle center."
            );

            return;
        }
    }


    const clearing =
        fullFenceZones.length === 0;


    const prompt =
        clearing
            ? (
                "CLEAR ALL native UAV "
                + "geofence zones from "
                + "the Pixhawk?"
            )
            : (
                `Upload all ${
                    fullFenceZones.length
                } UAV fence zone(s) `
                + "to the Pixhawk and "
                + "verify the complete read-back?"
            );


    if (!confirm(prompt)) {
        return;
    }


    fullFenceMessage(
        clearing
            ? "Clearing native fence..."
            : (
                "Uploading complete fence "
                + "and verifying read-back..."
            )
    );


    try {

        const response =
            await fetch(
                "/api/vehicles/uav/geofence",
                {
                    method: "POST",
                    cache: "no-store",

                    headers: {
                        "Content-Type":
                            "application/json"
                    },

                    body: JSON.stringify({
                        zones:
                            fullFenceZones
                    })
                }
            );


        const responseText =
            await response.text();

        let result;

        try {

            result =
                JSON.parse(
                    responseText
                );

        } catch (parseError) {

            const preview =
                responseText
                .replace(
                    /\\s+/g,
                    " "
                )
                .slice(
                    0,
                    300
                );

            throw new Error(
                "HTTP "
                + response.status
                + ": "
                + preview
            );
        }


        fullFenceMessage(
            result.message
            ?? (
                result.success
                    ? "Fence verified."
                    : "Fence upload failed."
            )
        );


        if (result.success) {

            await fullFenceLoadFromAutopilot(
                true
            );
        }


    } catch (error) {

        fullFenceMessage(
            "Fence upload failed: "
            + String(error)
        );
    }
}


async function fullFenceWriteSettings(
    payload
) {

    const response =
        await fetch(
            (
                "/api/vehicles/uav/"
                + "geofence/settings"
            ),
            {
                method: "POST",
                cache: "no-store",

                headers: {
                    "Content-Type":
                        "application/json"
                },

                body:
                    JSON.stringify(
                        payload
                    )
            }
        );


    return await response.json();
}


async function fullFenceApplySettings() {

    const numericValue = id => {

        const element =
            document.getElementById(id);

        if (!element) {
            return null;
        }

        const value =
            Number(
                element.value
            );

        return Number.isFinite(value)
            ? value
            : null;
    };


    let fenceType = 0;


    if (
        document.getElementById(
            "full-type-max"
        )?.checked
    ) {
        fenceType |= 1;
    }


    if (
        document.getElementById(
            "full-type-circle"
        )?.checked
    ) {
        fenceType |= 2;
    }


    if (
        document.getElementById(
            "full-type-zones"
        )?.checked
    ) {
        fenceType |= 4;
    }


    if (
        document.getElementById(
            "full-type-min"
        )?.checked
    ) {
        fenceType |= 8;
    }


    const previousOptions =
        Number(
            fullFenceState
            ?.settings
            ?.FENCE_OPTIONS
            ?? 0
        );


    let fenceOptions =
        previousOptions;


    if (
        document.getElementById(
            "full-fence-union"
        )?.checked
    ) {

        fenceOptions |= 2;

    } else {

        fenceOptions &= ~2;
    }


    const payload = {
        FENCE_TYPE:
            fenceType,

        FENCE_ACTION:
            numericValue(
                "full-fence-action"
            ),

        FENCE_ALT_MAX:
            numericValue(
                "full-fence-alt-max"
            ),

        FENCE_ALT_MIN:
            numericValue(
                "full-fence-alt-min"
            ),

        FENCE_MARGIN:
            numericValue(
                "full-fence-margin"
            ),

        FENCE_RADIUS:
            numericValue(
                "full-fence-radius"
            ),

        FENCE_OPTIONS:
            fenceOptions
    };


    fullFenceMessage(
        "Writing and verifying "
        + "fence settings..."
    );


    try {

        const result =
            await fullFenceWriteSettings(
                payload
            );


        fullFenceMessage(
            result.message
            ?? (
                result.success
                    ? "Settings verified."
                    : "Settings write failed."
            )
        );


        if (result.success) {

            await fullFenceLoadFromAutopilot(
                false
            );
        }


    } catch (error) {

        fullFenceMessage(
            "Settings write failed: "
            + String(error)
        );
    }
}


async function fullFenceSetEnabled(
    enabled
) {

    if (enabled) {

        if (
            !fullFenceState
            ||
            !fullFenceState.complete
            ||
            fullFenceZones.length === 0
        ) {

            fullFenceMessage(
                "Cannot enable: no "
                + "verified native fence "
                + "is loaded."
            );

            return;
        }


        if (
            !confirm(
                "ENABLE the UAV hard fence?\\n\\n"
                + "Confirm the intended zones, "
                + "GPS and altitude reference "
                + "are valid."
            )
        ) {
            return;
        }

    } else {

        if (
            !confirm(
                "DISABLE the UAV hard fence?"
            )
        ) {
            return;
        }
    }


    fullFenceMessage(
        enabled
            ? "Enabling hard fence..."
            : "Disabling hard fence..."
    );


    try {

        const result =
            await fullFenceWriteSettings({
                FENCE_ENABLE:
                    enabled ? 1 : 0
            });


        fullFenceMessage(
            result.message
            ?? (
                result.success
                    ? "Fence state verified."
                    : "Fence state write failed."
            )
        );


        await fullFenceLoadFromAutopilot(
            false
        );


    } catch (error) {

        fullFenceMessage(
            "Fence state write failed: "
            + String(error)
        );
    }
}


/*
 * Main Overview Map drawing input.
 */
map.on(
    "click",
    event => {

        if (
            !fullFenceDrawing
            ||
            fullFenceActive < 0
            ||
            fullFenceActive
                >= fullFenceZones.length
        ) {
            return;
        }


        const zone =
            fullFenceZones[
                fullFenceActive
            ];


        if (
            zone.type.endsWith(
                "_polygon"
            )
        ) {

            zone.points.push({
                latitude:
                    event.latlng.lat,

                longitude:
                    event.latlng.lng
            });


            fullFenceRender();


            fullFenceMessage(
                `${
                    zone.points.length
                } vertices. Continue `
                + "clicking or press "
                + "FINISH DRAWING."
            );

            return;
        }


        if (
            zone.type.endsWith(
                "_circle"
            )
        ) {

            zone.center = {
                latitude:
                    event.latlng.lat,

                longitude:
                    event.latlng.lng
            };


            fullFenceDrawing = false;


            fullFenceRender();


            fullFenceMessage(
                "Circle center placed. "
                + "Set its radius in the "
                + "zone list."
            );
        }
    }
);


/*
 * Compatibility wrapper for the previous startup loader.
 */
async function overviewLoadFence(
    refresh = true
) {

    return await fullFenceLoadFromAutopilot(
        refresh
    );
}


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



function consoleSlug(title) {

    return String(title)
        .trim()
        .toLowerCase()
        .replace(/[^a-z0-9]+/g, "-")
        .replace(/^-+|-+$/g, "");
}


function setConsoleState(
    id,
    text,
    state = "neutral"
) {

    const element =
        document.getElementById(id);

    if (!element) {
        return;
    }

    element.textContent =
        text ?? "--";

    element.className =
        `console-state ${state}`;
}


function finiteConsoleNumber(value) {

    if (
        value === null
        || value === undefined
    ) {
        return null;
    }

    const number = Number(value);

    return Number.isFinite(number)
        ? number
        : null;
}


function decorateVehicleConsole(
    id,
    vehicle
) {

    const statusPanel =
        document.getElementById(
            `${id}-status-panel`
        );

    if (!statusPanel) {
        return;
    }

    const layout =
        statusPanel.querySelector(
            ".vehicle-layout"
        );

    if (!layout) {
        return;
    }


    statusPanel.classList.add(
        "console-status-panel"
    );

    if (vehicle.type === "USV") {
        statusPanel.classList.add(
            "usv-console-status"
        );
    }

    layout.classList.add(
        "console-layout"
    );


    const header =
        document.createElement("div");

    header.className =
        "vehicle-console-header";

    header.innerHTML = `

        <div class="console-identity">
            <div class="console-kicker">
                VEHICLE OPERATOR CONSOLE
            </div>

            <div class="console-vehicle-name">
                ${vehicle.name}
            </div>
        </div>


        <div class="console-readout">
            <span class="console-readout-label">
                Link
            </span>

            <span
                class="console-state neutral"
                id="${id}-console-link">
                --
            </span>
        </div>


        <div class="console-readout">
            <span class="console-readout-label">
                Mode
            </span>

            <span
                class="console-state neutral"
                id="${id}-console-mode">
                --
            </span>
        </div>


        <div class="console-readout">
            <span class="console-readout-label">
                Arm
            </span>

            <span
                class="console-state neutral"
                id="${id}-console-arm">
                --
            </span>
        </div>


        <div class="console-readout">
            <span class="console-readout-label">
                Autonomy
            </span>

            <span
                class="console-state neutral"
                id="${id}-console-auto">
                --
            </span>
        </div>


        <div class="console-readout">
            <span class="console-readout-label">
                Power
            </span>

            <span
                class="console-state neutral"
                id="${id}-console-power">
                --
            </span>
        </div>


        <div class="console-readout">
            <span class="console-readout-label">
                Mission Process
            </span>

            <span
                class="console-state neutral"
                id="${id}-console-process">
                --
            </span>
        </div>
    `;


    const flow =
        document.createElement("div");

    flow.className =
        "console-flow";


    if (vehicle.type === "USV") {

        flow.innerHTML = `

            <div class="console-flow-stage">
                <span class="console-flow-label">
                    Perception
                </span>

                <span
                    class="console-state neutral"
                    id="${id}-console-flow-1">
                    WAITING
                </span>
            </div>


            <div class="console-flow-stage">
                <span class="console-flow-label">
                    Follower
                </span>

                <span
                    class="console-state neutral"
                    id="${id}-console-flow-2">
                    STOP
                </span>
            </div>


            <div class="console-flow-stage">
                <span class="console-flow-label">
                    Command Bridge
                </span>

                <span
                    class="console-state neutral"
                    id="${id}-console-flow-3">
                    IDLE
                </span>
            </div>


            <div class="console-flow-stage">
                <span class="console-flow-label">
                    Thruster Telemetry
                </span>

                <span
                    class="console-state neutral"
                    id="${id}-console-flow-4">
                    --
                </span>
            </div>
        `;

    } else if (vehicle.type === "UAV") {

        flow.innerHTML = `

            <div class="console-flow-stage">
                <span class="console-flow-label">
                    Safety
                </span>

                <span
                    class="console-state neutral"
                    id="${id}-console-flow-1">
                    --
                </span>
            </div>


            <div class="console-flow-stage">
                <span class="console-flow-label">
                    GPS
                </span>

                <span
                    class="console-state neutral"
                    id="${id}-console-flow-2">
                    --
                </span>
            </div>


            <div class="console-flow-stage">
                <span class="console-flow-label">
                    Pre-Arm
                </span>

                <span
                    class="console-state neutral"
                    id="${id}-console-flow-3">
                    --
                </span>
            </div>


            <div class="console-flow-stage">
                <span class="console-flow-label">
                    Command Authorization
                </span>

                <span
                    class="console-state neutral"
                    id="${id}-console-flow-4">
                    --
                </span>
            </div>
        `;
    }


    statusPanel.insertBefore(
        header,
        layout
    );

    statusPanel.insertBefore(
        flow,
        layout
    );


    /*
     * Split existing modules into:
     *
     *   main telemetry workspace
     *   right-side operator control rail
     *
     * Moving the DOM nodes preserves all existing IDs and
     * inline control actions.
     */

    const cards =
        Array.from(
            layout.children
        ).filter(
            element =>
                element.classList
                    .contains("card")
        );


    const main =
        document.createElement("main");

    main.className =
        "console-main-grid";


    const rail =
        document.createElement("aside");

    rail.className =
        "console-control-rail";


    const controlTitles =
        vehicle.type === "USV"
            ? [
                "Vehicle Control",
                "Mission Control",
                "Mission Process",
            ]
            : [
                "Remote Flight Control",
                "Mission Process",
            ];


    const wideTitles =
        vehicle.type === "USV"
            ? [
                "Perception",
                "Thruster Outputs",
                "Xbox Operator Controller",
            ]
            : [
                "Flight HUD",
                "Pre-Arm Checks",
            ];


    const mainOrder =
        vehicle.type === "USV"
            ? [
                "Perception",
                "State Machine",
                "Follower Command",
                "Bridge Output",
                "Thruster Outputs",
                "Position",
                "Battery Detail",
                "Xbox Operator Controller",
                "Connection",
                "Autopilot",
                "Power",
            ]
            : [
                "Flight HUD",
                "GPS / Navigation",
                "Flight Safety",
                "Pre-Arm Checks",
                "Autonomy",
                "Position",
                "Connection",
                "Autopilot",
                "Power",
            ];


    const cardInfo =
        cards.map(card => {

            const heading =
                card.querySelector("h2");

            const title =
                heading
                    ? heading.textContent.trim()
                    : "";

            card.classList.add(
                "console-module"
            );

            if (title) {

                card.classList.add(
                    `console-${consoleSlug(title)}`
                );
            }

            if (
                wideTitles.includes(title)
            ) {

                card.classList.add(
                    "console-wide"
                );
            }

            return {
                card,
                title,
            };
        });


    const controlCards =
        cardInfo
            .filter(
                item =>
                    controlTitles.includes(
                        item.title
                    )
            )
            .sort(
                (a, b) =>
                    controlTitles.indexOf(
                        a.title
                    )
                    -
                    controlTitles.indexOf(
                        b.title
                    )
            );


    const mainCards =
        cardInfo
            .filter(
                item =>
                    !controlTitles.includes(
                        item.title
                    )
            )
            .sort(
                (a, b) => {

                    const ai =
                        mainOrder.indexOf(
                            a.title
                        );

                    const bi =
                        mainOrder.indexOf(
                            b.title
                        );

                    const aRank =
                        ai < 0
                            ? 999
                            : ai;

                    const bRank =
                        bi < 0
                            ? 999
                            : bi;

                    return aRank - bRank;
                }
            );


    mainCards.forEach(
        item =>
            main.appendChild(
                item.card
            )
    );


    controlCards.forEach(
        item =>
            rail.appendChild(
                item.card
            )
    );


    layout.appendChild(main);
    layout.appendChild(rail);
}


function updateVehicleConsoleHeader(
    id,
    vehicle
) {

    setConsoleState(
        `${id}-console-link`,
        vehicle.online
            ? "ONLINE"
            : "OFFLINE",
        vehicle.online
            ? "good"
            : "bad"
    );


    setConsoleState(
        `${id}-console-mode`,
        vehicle.mode
            ?? "UNKNOWN",
        "neutral"
    );


    setConsoleState(
        `${id}-console-arm`,
        vehicle.armed
            ? "ARMED"
            : "DISARMED",
        vehicle.armed
            ? "warn"
            : "neutral"
    );


    setConsoleState(
        `${id}-console-auto`,
        vehicle.autonomy_enabled
            ? "AUTO"
            : "MANUAL",
        vehicle.autonomy_enabled
            ? "good"
            : "neutral"
    );


    let battery =
        finiteConsoleNumber(
            vehicle.battery_percent
        );

    if (battery === null) {

        battery =
            finiteConsoleNumber(
                vehicle.battery_remaining
            );
    }


    const voltage =
        finiteConsoleNumber(
            vehicle.voltage
        );


    let powerText = "--";

    if (battery !== null) {

        powerText =
            `${battery.toFixed(0)}%`;

        if (voltage !== null) {

            powerText +=
                ` · ${voltage.toFixed(1)} V`;
        }

    } else if (voltage !== null) {

        powerText =
            `${voltage.toFixed(1)} V`;
    }


    setConsoleState(
        `${id}-console-power`,
        powerText,
        "neutral"
    );


    const processRunning =
        Boolean(
            vehicle.mission_process_running
        );


    setConsoleState(
        `${id}-console-process`,
        processRunning
            ? "RUNNING"
            : String(
                vehicle.mission_process_state
                ?? "STOPPED"
            ).toUpperCase(),
        processRunning
            ? "good"
            : "neutral"
    );


    if (vehicle.type === "USV") {

        setConsoleState(
            `${id}-console-flow-1`,
            vehicle.gate_fresh
                ? "GATE LOCK"
                : "WAITING",
            vehicle.gate_fresh
                ? "good"
                : "neutral"
        );


        setConsoleState(
            `${id}-console-flow-2`,
            vehicle.control_ready
                ? "READY"
                : "STOP",
            vehicle.control_ready
                ? "good"
                : "neutral"
        );


        setConsoleState(
            `${id}-console-flow-3`,
            vehicle.bridge_command_active
                ? "ACTIVE"
                : "IDLE",
            vehicle.bridge_command_active
                ? "good"
                : "neutral"
        );


        setConsoleState(
            `${id}-console-flow-4`,
            vehicle.servo_output_fresh
                ? "LIVE"
                : "STALE",
            vehicle.servo_output_fresh
                ? "good"
                : "warn"
        );

    } else if (vehicle.type === "UAV") {

        const safetyState =
            String(
                vehicle.safety_state
                ?? "UNKNOWN"
            ).toUpperCase();


        setConsoleState(
            `${id}-console-flow-1`,
            safetyState,
            safetyState === "READY"
                || safetyState === "ACTIVE"
                    ? "good"
                    : (
                        safetyState === "FAILSAFE"
                            ? "bad"
                            : "warn"
                    )
        );


        setConsoleState(
            `${id}-console-flow-2`,
            vehicle.gps_valid
                ? "VALID"
                : "INVALID",
            vehicle.gps_valid
                ? "good"
                : "bad"
        );


        setConsoleState(
            `${id}-console-flow-3`,
            vehicle.prearm_ready
                ? "READY"
                : "BLOCKED",
            vehicle.prearm_ready
                ? "good"
                : "warn"
        );


        setConsoleState(
            `${id}-console-flow-4`,
            vehicle.authorized
                ? "AUTHORIZED"
                : "INHIBITED",
            vehicle.authorized
                ? "good"
                : "neutral"
        );
    }
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
    let uavCards = "";
    let missionProcessCard = "";

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
                <h2>Thruster Outputs</h2>

                <div class="row">
                    <span>Port Thruster</span>
                    <span class="value"
                          id="${id}-port-thruster">
                        --
                    </span>
                </div>

                <div class="row">
                    <span>Starboard Thruster</span>
                    <span class="value"
                          id="${id}-starboard-thruster">
                        --
                    </span>
                </div>

                <div class="row">
                    <span>PWM Telemetry</span>
                    <span class="value"
                          id="${id}-pwm-status">
                        UNAVAILABLE
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
                    <span class="value"
                          id="${id}-operator-backend">
                        WAITING
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

                <label for="${id}-log-label"
                       style="display:block; margin-top:14px;">
                    Test label (optional)
                </label>

                <input
                    class="log-label-input"
                    id="${id}-log-label"
                    maxlength="40"
                    placeholder="precal, water, parkinglot">

                <button
                    class="control-button reset-button"
                    id="${id}-reset-button"
                    onclick="usvControlAction('reset_mission')">
                    RESET MISSION
                </button>

                <div class="row" style="margin-top:12px;">
                    <span>Diagnostic Log</span>
                    <span class="value"
                          id="${id}-log-state">
                        UNAVAILABLE
                    </span>
                </div>

                <div class="row">
                    <span>Mission / Rows</span>
                    <span class="value"
                          id="${id}-log-progress">
                        --
                    </span>
                </div>

                <div class="row">
                    <span>Jetson File</span>
                    <span class="value log-path"
                          id="${id}-log-file">
                        --
                    </span>
                </div>


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


    if (vehicle.type === "UAV") {

        uavCards = `

            <div class="card uav-hud-card">
                <h2>Flight HUD</h2>

                <div class="uav-hud-grid">

                    <div
                        class="uav-horizon"
                        id="${id}-horizon">

                        <div
                            class="uav-horizon-world"
                            id="${id}-horizon-world">

                            <div class="uav-horizon-sky">
                            </div>

                            <div class="uav-horizon-ground">
                            </div>

                            <div class="uav-horizon-line">
                            </div>

                        </div>

                        <div class="uav-aircraft-symbol">
                        </div>

                    </div>


                    <div>

                        <div class="uav-hud-readout">
                            <div class="uav-hud-label">
                                Heading
                            </div>

                            <div
                                class="uav-hud-big"
                                id="${id}-hud-heading">
                                --
                            </div>
                        </div>


                        <div class="uav-hud-readout">
                            <div class="uav-hud-label">
                                Relative Altitude
                            </div>

                            <div
                                class="uav-hud-big"
                                id="${id}-relative-altitude">
                                --
                            </div>
                        </div>


                        <div class="uav-hud-readout">
                            <div class="uav-hud-label">
                                Ground Speed
                            </div>

                            <div
                                class="uav-hud-big"
                                id="${id}-ground-speed">
                                --
                            </div>
                        </div>


                        <div class="uav-hud-readout">
                            <div class="uav-hud-label">
                                Vertical Speed
                            </div>

                            <div
                                class="uav-hud-big"
                                id="${id}-vertical-speed">
                                --
                            </div>
                        </div>

                    </div>

                </div>


                <div class="row">
                    <span>Roll</span>
                    <span
                        class="value"
                        id="${id}-roll">
                        --
                    </span>
                </div>

                <div class="row">
                    <span>Pitch</span>
                    <span
                        class="value"
                        id="${id}-pitch">
                        --
                    </span>
                </div>

            </div>


            <div class="card">
                <h2>GPS / Navigation</h2>

                <div class="row">
                    <span>GPS Valid</span>
                    <span
                        class="value"
                        id="${id}-gps-valid">
                        NO
                    </span>
                </div>

                <div class="row">
                    <span>GPS Position σ</span>
                    <span
                        class="value"
                        id="${id}-gps-sigma">
                        --
                    </span>
                </div>

                <div class="row">
                    <span>Local Position</span>
                    <span
                        class="value"
                        id="${id}-local-position">
                        INVALID
                    </span>
                </div>

                <div class="row">
                    <span>MSL Altitude</span>
                    <span
                        class="value"
                        id="${id}-altitude-msl">
                        --
                    </span>
                </div>

            </div>


            <div class="card">
                <h2>Flight Safety</h2>

                <div class="row">
                    <span>Safety State</span>
                    <span
                        class="value"
                        id="${id}-safety-state">
                        UNKNOWN
                    </span>
                </div>

                <div class="row">
                    <span>Pre-arm Ready</span>
                    <span
                        class="value"
                        id="${id}-prearm-ready">
                        NO
                    </span>
                </div>

                <div class="row">
                    <span>Flight Ready</span>
                    <span
                        class="value"
                        id="${id}-flight-ready">
                        NO
                    </span>
                </div>

                <div class="row">
                    <span>Failsafe</span>
                    <span
                        class="value"
                        id="${id}-failsafe">
                        CLEAR
                    </span>
                </div>

                <div class="row">
                    <span>Reason</span>
                    <span
                        class="value"
                        id="${id}-safety-reason">
                        --
                    </span>
                </div>

            </div>


            <div class="card">
                <h2>Pre-Arm Checks</h2>

                <div class="row">
                    <span>MAVROS State Fresh</span>
                    <span class="value"
                          id="${id}-check-state-fresh">
                        FALSE
                    </span>
                </div>

                <div class="row">
                    <span>MAVROS Connected</span>
                    <span class="value"
                          id="${id}-check-mavros-connected">
                        FALSE
                    </span>
                </div>

                <div class="row">
                    <span>Flight Mode Allowed</span>
                    <span class="value"
                          id="${id}-check-mode-allowed">
                        FALSE
                    </span>
                </div>

                <div class="row">
                    <span>GPS Valid</span>
                    <span class="value"
                          id="${id}-check-gps-valid">
                        FALSE
                    </span>
                </div>

                <div class="row">
                    <span>Local Position Fresh</span>
                    <span class="value"
                          id="${id}-check-local-position">
                        FALSE
                    </span>
                </div>

                <div class="row">
                    <span>Battery Valid</span>
                    <span class="value"
                          id="${id}-check-battery-valid">
                        FALSE
                    </span>
                </div>

                <div class="row">
                    <span>Safety Battery Level</span>
                    <span class="value"
                          id="${id}-check-battery-percent">
                        --
                    </span>
                </div>

                <div class="row">
                    <span>Autonomy Status Fresh</span>
                    <span class="value"
                          id="${id}-check-autonomy-fresh">
                        FALSE
                    </span>
                </div>

                <div class="row">
                    <span>Mission Command Fresh</span>
                    <span class="value"
                          id="${id}-check-mission-healthy">
                        FALSE
                    </span>
                </div>

                <div class="row">
                    <span>Failsafe Clear</span>
                    <span class="value"
                          id="${id}-check-failsafe-clear">
                        TRUE
                    </span>
                </div>

                <div class="row">
                    <span>Pre-Arm Ready</span>
                    <span class="value"
                          id="${id}-check-prearm-ready">
                        FALSE
                    </span>
                </div>

                <div class="row">
                    <span>RC / Manual Input (info)</span>
                    <span class="value"
                          id="${id}-check-manual-input">
                        FALSE
                    </span>
                </div>

                <div class="row">
                    <span>Blocking Reason</span>
                    <span class="value"
                          id="${id}-check-reason"
                          style="
                              max-width:60%;
                              text-align:right;
                              overflow-wrap:anywhere;
                          ">
                        --
                    </span>
                </div>

            </div>


            <div class="card">
                <h2>Autonomy</h2>

                <div class="row">
                    <span>Enabled</span>
                    <span
                        class="value"
                        id="${id}-uav-autonomy">
                        OFF
                    </span>
                </div>

                <div class="row">
                    <span>Authorized</span>
                    <span
                        class="value"
                        id="${id}-authorized">
                        NO
                    </span>
                </div>

                <div class="row">
                    <span>Command Fresh</span>
                    <span
                        class="value"
                        id="${id}-command-fresh">
                        NO
                    </span>
                </div>

                <div class="row">
                    <span>Status</span>
                    <span
                        class="value"
                        id="${id}-autonomy-reason">
                        --
                    </span>
                </div>

                <div class="control-message">
                    Remote commands use the UAV vehicle-side
                    safety services. Commands may still be
                    rejected by vehicle safety gates.
                </div>

            </div>


            <div class="card">
                <h2>Remote Flight Control</h2>

                <div class="row">
                    <span>Requested Mode</span>

                    <select
                        id="${id}-mode-select"
                        style="
                            background:#18222c;
                            color:white;
                            border:1px solid #52677a;
                            border-radius:4px;
                            padding:6px;
                        ">

                        <option value="STABILIZE">
                            STABILIZE
                        </option>

                        <option value="GUIDED">
                            GUIDED
                        </option>

                        <option value="LOITER">
                            LOITER
                        </option>

                        <option value="RTL">
                            RTL
                        </option>

                        <option value="LAND">
                            LAND
                        </option>

                    </select>
                </div>

                <button
                    class="control-button reset-button"
                    id="${id}-set-mode-button"
                    onclick="uavSetMode()">
                    SET MODE
                </button>


                <button
                    class="control-button arm-button"
                    id="${id}-arm-button"
                    onclick="uavSimpleAction('arm')">
                    ARM
                </button>

                <button
                    class="control-button disarm-button"
                    id="${id}-disarm-button"
                    onclick="uavSimpleAction('disarm')">
                    DISARM
                </button>


                <div class="row"
                     style="margin-top:16px;">

                    <span>Takeoff Altitude</span>

                    <input
                        id="${id}-takeoff-altitude"
                        type="number"
                        value="2.0"
                        min="1.0"
                        max="5.0"
                        step="0.5"
                        style="
                            width:80px;
                            background:#18222c;
                            color:white;
                            border:1px solid #52677a;
                            border-radius:4px;
                            padding:6px;
                        ">
                </div>

                <button
                    class="control-button enable-button"
                    id="${id}-takeoff-button"
                    onclick="uavTakeoff()">
                    TAKEOFF
                </button>


                <button
                    class="control-button reset-button"
                    id="${id}-rtl-button"
                    onclick="uavSimpleAction('rtl')">
                    RTL
                </button>

                <button
                    class="control-button disarm-button"
                    id="${id}-land-button"
                    onclick="uavSimpleAction('land')">
                    LAND
                </button>


                <button
                    class="control-button enable-button"
                    id="${id}-autonomy-enable-button"
                    onclick="uavSetAutonomy(true)">
                    ENABLE AUTONOMY
                </button>

                <button
                    class="control-button reset-button"
                    id="${id}-autonomy-disable-button"
                    onclick="uavSetAutonomy(false)">
                    DISABLE AUTONOMY
                </button>


                <button
                    class="control-button reset-button"
                    id="${id}-reset-failsafe-button"
                    onclick="uavSimpleAction('reset_failsafe')">
                    RESET FAILSAFE
                </button>


                <div
                    class="control-message"
                    id="${id}-control-message">
                    No UAV control command sent.
                </div>

            </div>

        `;
    }


    if (
        vehicle.type === "USV"
        || vehicle.type === "UAV"
    ) {

        missionProcessCard = `

            <div class="card">
                <h2>Mission Process</h2>

                <div class="row">
                    <span>Service State</span>
                    <span
                        class="value"
                        id="${id}-mission-process-state">
                        UNKNOWN
                    </span>
                </div>

                <div class="row">
                    <span>Running</span>
                    <span
                        class="value"
                        id="${id}-mission-process-running">
                        NO
                    </span>
                </div>

                <button
                    class="control-button enable-button"
                    id="${id}-mission-start-button"
                    onclick="missionProcessAction(
                        '${id}',
                        'start'
                    )">
                    START MISSION PROCESS
                </button>

                <button
                    class="control-button reset-button"
                    id="${id}-mission-stop-button"
                    onclick="missionProcessAction(
                        '${id}',
                        'stop'
                    )">
                    STOP MISSION PROCESS
                </button>

                <div
                    class="control-message"
                    id="${id}-mission-process-message">
                    Mission process control ready.
                </div>

                <div class="control-message">
                    Starting the process does not arm the
                    vehicle, change flight mode, clear a
                    software stop, or enable autonomy.
                </div>

            </div>
        `;
    }


    page.innerHTML = `
        <div class="vehicle-subtabs">

            <button
                class="vehicle-subtab active"
                id="${id}-status-tab"
                onclick="selectVehicleSubtab('${id}', 'status')">
                STATUS
            </button>

            <button
                class="vehicle-subtab"
                id="${id}-parameters-tab"
                onclick="selectVehicleSubtab('${id}', 'parameters')">
                PARAMETERS
            </button>

            ${
                vehicle.type === "UAV"
                    ? `
            <button
                class="vehicle-subtab"
                id="${id}-geofence-tab"
                onclick="selectVehicleSubtab('${id}', 'geofence')">
                GEOFENCE
            </button>
                    `
                    : ""
            }

        </div>


        <div
            class="vehicle-panel active"
            id="${id}-status-panel">

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


            ${missionProcessCard}
            ${usvCards}
            ${uavCards}

            </div>
        </div>


        <div
            class="vehicle-panel"
            id="${id}-parameters-panel">

            <div class="card">

                <h2>${vehicle.name} Parameters</h2>

                <div
                    class="parameter-status"
                    id="${id}-parameter-status">
                    Open this tab to load parameters.
                </div>

                <div class="parameter-toolbar">

                    <input
                        class="parameter-search"
                        id="${id}-parameter-search"
                        placeholder="Search parameters..."
                        oninput="renderVehicleParameters('${id}')">

                    <button
                        class="control-button"
                        style="margin-top:0;"
                        onclick="refreshVehicleParameters('${id}')">
                        REFRESH ALL
                    </button>

                </div>

                <div class="parameter-table-wrap">

                    <table class="parameter-table">

                        <thead>
                            <tr>
                                <th>Parameter</th>
                                <th>Current</th>
                                <th>Type</th>
                                <th>New Value</th>
                                <th>Action</th>
                            </tr>
                        </thead>

                        <tbody
                            id="${id}-parameter-table-body">

                            <tr>
                                <td colspan="5">
                                    Parameters not loaded.
                                </td>
                            </tr>

                        </tbody>

                    </table>

                </div>

            </div>

        </div>

        ${
            vehicle.type === "UAV"
                ? `
        <div
            class="vehicle-panel"
            id="${id}-geofence-panel">

            <div class="card">

                <h2>Native ArduPilot Geofence</h2>

                <div
                    class="geofence-status"
                    id="${id}-geofence-status">
                    Open this tab to download the Pixhawk fence.
                </div>

                <button
                    class="control-button"
                    style="margin-top:0;"
                    onclick="refreshVehicleGeofence('${id}')">
                    DOWNLOAD FROM AUTOPILOT
                </button>

                <div
                    class="geofence-map"
                    id="${id}-geofence-map">
                </div>

                <table class="geofence-table">

                    <thead>
                        <tr>
                            <th>Seq</th>
                            <th>Latitude</th>
                            <th>Longitude</th>
                            <th>Vertices</th>
                        </tr>
                    </thead>

                    <tbody
                        id="${id}-geofence-table-body">

                        <tr>
                            <td colspan="4">
                                Fence not loaded.
                            </td>
                        </tr>

                    </tbody>

                </table>

            </div>

        </div>
                `
                : ""
        }

    `;


    document.body.appendChild(page);


    // The page must be attached to the document before
    // decorateVehicleConsole() uses document.getElementById().
    decorateVehicleConsole(
        id,
        vehicle
    );


    vehiclePages[id] = true;

    installTabHandlers();
}



function selectVehicleSubtab(
    vehicleId,
    tabName
) {

    const page =
        document.getElementById(
            `${vehicleId}-page`
        );

    if (!page) {
        return;
    }


    [
        "status",
        "parameters",
        "geofence"
    ].forEach(name => {

        const tab =
            document.getElementById(
                `${vehicleId}-${name}-tab`
            );

        const panel =
            document.getElementById(
                `${vehicleId}-${name}-panel`
            );


        if (tab) {
            tab.classList.toggle(
                "active",
                name === tabName
            );
        }

        if (panel) {
            panel.classList.toggle(
                "active",
                name === tabName
            );
        }
    });


    if (tabName === "parameters") {

        if (
            !vehicleParameterState[
                vehicleId
            ]
        ) {
            loadVehicleParameters(
                vehicleId
            );
        }
    }


    if (tabName === "geofence") {

        loadVehicleGeofence(
            vehicleId,
            false
        );
    }
}


function parameterEscape(value) {

    return String(value ?? "")
        .replaceAll("&", "&amp;")
        .replaceAll("<", "&lt;")
        .replaceAll(">", "&gt;")
        .replaceAll('"', "&quot;");
}


function parameterValueText(value) {

    if (
        value === null
        || value === undefined
    ) {
        return "--";
    }

    const number = Number(value);

    if (!Number.isFinite(number)) {
        return String(value);
    }

    return String(number);
}


async function loadVehicleParameters(
    vehicleId
) {

    const status =
        document.getElementById(
            `${vehicleId}-parameter-status`
        );

    if (status) {
        status.textContent =
            "Loading parameter cache...";
    }


    try {

        const response = await fetch(
            `/api/vehicles/${vehicleId}/parameters`,
            {
                cache: "no-store"
            }
        );

        const data =
            await response.json();

        vehicleParameterState[
            vehicleId
        ] = data;


        if (status) {

            if (!data.available) {

                status.textContent =
                    data.message
                    ?? "Parameter backend unavailable.";

            } else {

                status.textContent =
                    `${data.count} / `
                    + `${data.expected ?? "?"} parameters`
                    + (
                        data.write_allowed
                            ? " | WRITES ENABLED"
                            : " | WRITES LOCKED"
                    );
            }
        }

        renderVehicleParameters(
            vehicleId
        );

    } catch (error) {

        if (status) {
            status.textContent =
                "Parameter request failed: "
                + error;
        }
    }
}


function renderVehicleParameters(
    vehicleId
) {

    const body =
        document.getElementById(
            `${vehicleId}-parameter-table-body`
        );

    if (!body) {
        return;
    }


    const state =
        vehicleParameterState[
            vehicleId
        ];

    if (!state) {

        body.innerHTML = `
            <tr>
                <td colspan="5">
                    Parameters not loaded.
                </td>
            </tr>
        `;

        return;
    }


    if (!state.available) {

        body.innerHTML = `
            <tr>
                <td colspan="5">
                    ${parameterEscape(
                        state.message
                        ?? "Backend unavailable."
                    )}
                </td>
            </tr>
        `;

        return;
    }


    const search =
        (
            document.getElementById(
                `${vehicleId}-parameter-search`
            )?.value
            ?? ""
        )
        .trim()
        .toUpperCase();


    const parameters =
        Array.isArray(
            state.parameters
        )
            ? state.parameters
            : [];


    const filtered =
        parameters.filter(param => {

            if (!search) {
                return true;
            }

            return (
                String(param.name)
                    .toUpperCase()
                    .includes(search)
                ||
                String(param.type_name ?? "")
                    .toUpperCase()
                    .includes(search)
            );
        });


    if (filtered.length === 0) {

        body.innerHTML = `
            <tr>
                <td colspan="5">
                    No matching parameters.
                </td>
            </tr>
        `;

        return;
    }


    body.innerHTML =
        filtered.map(param => {

            const name =
                String(param.name);

            const value =
                parameterValueText(
                    param.value
                );

            const disabled =
                state.write_allowed
                    ? ""
                    : "disabled";

            return `
                <tr>

                    <td class="parameter-name">
                        ${parameterEscape(name)}
                    </td>

                    <td>
                        ${parameterEscape(value)}
                    </td>

                    <td>
                        ${parameterEscape(
                            param.type_name
                            ?? param.type
                            ?? "--"
                        )}
                    </td>

                    <td>
                        <input
                            class="parameter-value-input"
                            id="param-${vehicleId}-${name}"
                            value="${parameterEscape(value)}"
                            ${disabled}>
                    </td>

                    <td>
                        <button
                            class="parameter-write-button"
                            ${disabled}
                            onclick="
                                writeVehicleParameter(
                                    '${vehicleId}',
                                    '${name}'
                                )
                            ">
                            WRITE
                        </button>
                    </td>

                </tr>
            `;
        }).join("");
}


async function refreshVehicleParameters(
    vehicleId
) {

    const status =
        document.getElementById(
            `${vehicleId}-parameter-status`
        );

    if (status) {
        status.textContent =
            "Refreshing all parameters...";
    }


    try {

        const response = await fetch(
            `/api/vehicles/${vehicleId}/parameters/refresh`,
            {
                method: "POST",
                cache: "no-store"
            }
        );

        const result =
            await response.json();

        if (status) {
            status.textContent =
                result.message
                ?? "Refresh complete.";
        }

        await loadVehicleParameters(
            vehicleId
        );

    } catch (error) {

        if (status) {
            status.textContent =
                "Refresh failed: "
                + error;
        }
    }
}


async function writeVehicleParameter(
    vehicleId,
    parameterName
) {

    const state =
        vehicleParameterState[
            vehicleId
        ];

    const status =
        document.getElementById(
            `${vehicleId}-parameter-status`
        );


    if (
        !state
        || !state.write_allowed
    ) {

        if (status) {
            status.textContent =
                "Parameter writes are currently locked.";
        }

        return;
    }


    const input =
        document.getElementById(
            `param-${vehicleId}-${parameterName}`
        );

    if (!input) {
        return;
    }


    const value =
        Number(input.value);

    if (!Number.isFinite(value)) {

        if (status) {
            status.textContent =
                `${parameterName}: `
                + "value must be numeric.";
        }

        return;
    }


    if (!confirm(
        `Write ${parameterName} = ${value}?`
        + "\n\n"
        + "The value will be sent to the autopilot "
        + "and verified by read-back."
    )) {
        return;
    }


    if (status) {
        status.textContent =
            `Writing ${parameterName}...`;
    }


    try {

        const response = await fetch(
            (
                `/api/vehicles/${vehicleId}/`
                + `parameters/${parameterName}`
            ),
            {
                method: "POST",
                cache: "no-store",
                headers: {
                    "Content-Type":
                        "application/json"
                },
                body: JSON.stringify({
                    value: value
                })
            }
        );

        const result =
            await response.json();


        if (status) {
            status.textContent =
                result.message
                ?? (
                    result.success
                        ? "Parameter verified."
                        : "Parameter write failed."
                );
        }


        if (result.success) {

            await loadVehicleParameters(
                vehicleId
            );
        }

    } catch (error) {

        if (status) {
            status.textContent =
                "Parameter write failed: "
                + error;
        }
    }
}



function ensureVehicleGeofenceMap(
    vehicleId
) {

    if (
        vehicleGeofenceMaps[
            vehicleId
        ]
    ) {
        return vehicleGeofenceMaps[
            vehicleId
        ];
    }


    const element =
        document.getElementById(
            `${vehicleId}-geofence-map`
        );

    if (!element) {
        return null;
    }


    const fenceMap =
        L.map(element).setView(
            [30.21, -92.02],
            16
        );


    L.tileLayer(
        "https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png",
        {
            maxZoom: 20,
            attribution:
                "&copy; OpenStreetMap"
        }
    ).addTo(fenceMap);


    vehicleGeofenceMaps[
        vehicleId
    ] = fenceMap;


    setTimeout(
        () => fenceMap.invalidateSize(),
        50
    );


    return fenceMap;
}


function renderVehicleGeofence(
    vehicleId
) {

    const state =
        vehicleGeofenceState[
            vehicleId
        ];

    const status =
        document.getElementById(
            `${vehicleId}-geofence-status`
        );

    const body =
        document.getElementById(
            `${vehicleId}-geofence-table-body`
        );


    if (!state) {
        return;
    }


    if (status) {

        if (!state.available) {

            status.textContent =
                "Autopilot geofence backend offline.";

        } else {

            status.textContent =
                `Native fence: `
                + `${state.count}/`
                + `${state.expected ?? "?"}`
                + (
                    state.complete
                        ? " | COMPLETE"
                        : " | INCOMPLETE"
                )
                + (
                    state.armed
                        ? " | VEHICLE ARMED"
                        : " | VEHICLE DISARMED"
                );
        }
    }


    const vertices =
        Array.isArray(
            state.inclusion_vertices
        )
            ? state.inclusion_vertices
            : [];


    if (body) {

        if (vertices.length === 0) {

            body.innerHTML = `
                <tr>
                    <td colspan="4">
                        No inclusion polygon loaded.
                    </td>
                </tr>
            `;

        } else {

            body.innerHTML =
                vertices.map(point => `
                    <tr>
                        <td>
                            ${point.seq}
                        </td>

                        <td>
                            ${Number(
                                point.latitude
                            ).toFixed(7)}
                        </td>

                        <td>
                            ${Number(
                                point.longitude
                            ).toFixed(7)}
                        </td>

                        <td>
                            ${
                                point.polygon_vertex_count
                                ?? "--"
                            }
                        </td>
                    </tr>
                `).join("");
        }
    }


    const fenceMap =
        ensureVehicleGeofenceMap(
            vehicleId
        );

    if (!fenceMap) {
        return;
    }


    if (
        vehicleGeofenceLayers[
            vehicleId
        ]
    ) {

        fenceMap.removeLayer(
            vehicleGeofenceLayers[
                vehicleId
            ]
        );

        delete vehicleGeofenceLayers[
            vehicleId
        ];
    }


    if (vertices.length >= 3) {

        const points =
            vertices.map(point => [
                Number(point.latitude),
                Number(point.longitude)
            ]);


        const polygon =
            L.polygon(points);

        polygon.addTo(
            fenceMap
        );


        vehicleGeofenceLayers[
            vehicleId
        ] = polygon;


        fenceMap.fitBounds(
            polygon.getBounds(),
            {
                padding: [25, 25]
            }
        );
    }


    setTimeout(
        () => fenceMap.invalidateSize(),
        50
    );
}


async function loadVehicleGeofence(
    vehicleId,
    forceRefresh = false
) {

    const status =
        document.getElementById(
            `${vehicleId}-geofence-status`
        );


    if (status) {
        status.textContent =
            forceRefresh
                ? "Downloading native fence..."
                : "Loading geofence cache...";
    }


    try {

        const path =
            forceRefresh
                ? (
                    `/api/vehicles/`
                    + `${vehicleId}/`
                    + `geofence/refresh`
                )
                : (
                    `/api/vehicles/`
                    + `${vehicleId}/`
                    + `geofence`
                );


        const options = {
            cache: "no-store"
        };


        if (forceRefresh) {
            options.method = "POST";
        }


        const response =
            await fetch(
                path,
                options
            );


        const data =
            await response.json();


        vehicleGeofenceState[
            vehicleId
        ] = data;


        renderVehicleGeofence(
            vehicleId
        );


    } catch (error) {

        if (status) {
            status.textContent =
                "Geofence request failed: "
                + error;
        }
    }
}


async function refreshVehicleGeofence(
    vehicleId
) {

    await loadVehicleGeofence(
        vehicleId,
        true
    );
}


function formatThrusterOutput(
    vehicle,
    functionId
) {

    const config =
        vehicle.servo_config
        ?? {};

    const outputs =
        vehicle.servo_outputs
        ?? {};


    for (
        const [channel, item]
        of Object.entries(config)
    ) {

        if (
            Number(item.function)
            !== Number(functionId)
        ) {
            continue;
        }


        const pwm =
            outputs[channel];

        const pwmText =
            pwm === null
            || pwm === undefined
                ? "--"
                : `${pwm} µs`;


        return (
            `SERVO${channel} | `
            + `${pwmText} | `
            + `MIN ${item.min ?? "--"} / `
            + `TRIM ${item.trim ?? "--"} / `
            + `MAX ${item.max ?? "--"}`
        );
    }


    return "NOT MAPPED";
}



function setText(id, value) {

    const element =
        document.getElementById(id);

    if (element) {
        element.textContent = value;
    }
}


function setBooleanCheck(
    id,
    value,
    invert = false
) {
    const element =
        document.getElementById(id);

    if (!element) {
        return;
    }

    const result =
        invert
            ? !Boolean(value)
            : Boolean(value);

    element.textContent =
        result ? "TRUE" : "FALSE";

    element.className =
        "value "
        + (
            result
                ? "connected"
                : "disconnected"
        );
}


function formatQuadrantalHeading(value) {

    if (
        value === null
        || value === undefined
        || !Number.isFinite(Number(value))
    ) {
        return "--";
    }

    let heading =
        ((Number(value) % 360) + 360) % 360;

    // Keep one decimal when useful, otherwise suppress .0.
    function angleText(angle) {
        const rounded =
            Math.round(angle * 10) / 10;

        return Number.isInteger(rounded)
            ? rounded.toFixed(0)
            : rounded.toFixed(1);
    }

    // Exact cardinal directions.
    if (heading < 0.05 || heading >= 359.95) {
        return "N";
    }

    if (Math.abs(heading - 90) < 0.05) {
        return "E";
    }

    if (Math.abs(heading - 180) < 0.05) {
        return "S";
    }

    if (Math.abs(heading - 270) < 0.05) {
        return "W";
    }

    // Quadrantal bearing notation:
    //
    //   0..90     N angle E
    //   90..180   S angle E
    //   180..270  S angle W
    //   270..360  N angle W

    if (heading < 90) {
        return `N ${angleText(heading)}° E`;
    }

    if (heading < 180) {
        return `S ${angleText(180 - heading)}° E`;
    }

    if (heading < 270) {
        return `S ${angleText(heading - 180)}° W`;
    }

    return `N ${angleText(360 - heading)}° W`;
}


function updateVehicle(id, vehicle) {

    if (!vehiclePages[id]) {
        makeVehiclePage(id, vehicle);
    }


    updateVehicleConsoleHeader(
        id,
        vehicle
    );


    const missionState =
        String(
            vehicle.mission_process_state
            ?? "unknown"
        ).toLowerCase();

    const missionRunning =
        Boolean(
            vehicle.mission_process_running
        );

    setText(
        `${id}-mission-process-state`,
        missionState.toUpperCase()
    );

    setText(
        `${id}-mission-process-running`,
        missionRunning
            ? "YES"
            : "NO"
    );


    const missionStartButton =
        document.getElementById(
            `${id}-mission-start-button`
        );

    const missionStopButton =
        document.getElementById(
            `${id}-mission-stop-button`
        );


    const missionTransportReady =
        vehicle.type === "USV"
            ? Boolean(
                vehicle.jetson_link
            )
            : Boolean(
                vehicle.vehicle_link
            );


    if (missionStartButton) {

        missionStartButton.disabled =
            !missionTransportReady
            || missionRunning;
    }


    if (missionStopButton) {

        missionStopButton.disabled =
            !missionTransportReady
            || !missionRunning;
    }


    if (vehicle.type === "USV") {

        setText(
            `${id}-port-thruster`,
            formatThrusterOutput(
                vehicle,
                73
            )
        );

        setText(
            `${id}-starboard-thruster`,
            formatThrusterOutput(
                vehicle,
                74
            )
        );

        setText(
            `${id}-pwm-status`,
            vehicle.servo_output_fresh
                ? "RECEIVING"
                : "STALE / UNAVAILABLE"
        );


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
            `${id}-log-state`,
            vehicle.logger_fresh
                ? (vehicle.log_state ?? "IDLE")
                : "UNAVAILABLE"
        );

        const missionNumber =
            vehicle.log_mission_id === null
                ? "pending"
                : `#${vehicle.log_mission_id}`;

        setText(
            `${id}-log-progress`,
            vehicle.log_pending
                ? `pending / ${vehicle.log_buffer_rows ?? 0} buffered`
                : `${missionNumber} / ${vehicle.log_row_count ?? 0} rows`
        );

        const logPath = vehicle.log_file_path;
        setText(
            `${id}-log-file`,
            logPath
                ? logPath.split("/").pop()
                : (
                    vehicle.log_last_end_reason
                        ? `saved: ${vehicle.log_last_end_reason}`
                        : "--"
                )
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




    if (vehicle.type === "UAV") {

        const roll =
            vehicle.roll_deg === null
                ? 0.0
                : vehicle.roll_deg;

        const pitch =
            vehicle.pitch_deg === null
                ? 0.0
                : vehicle.pitch_deg;

        const pitchOffset =
            Math.max(
                -80,
                Math.min(
                    80,
                    pitch * 3.0
                )
            );

        const horizonWorld =
            document.getElementById(
                `${id}-horizon-world`
            );

        if (horizonWorld) {

            horizonWorld.style.transform =
                `translateY(${pitchOffset}px) `
                + `rotate(${-roll}deg)`;
        }


        setText(
            `${id}-roll`,
            vehicle.roll_deg === null
                ? "--"
                : `${vehicle.roll_deg.toFixed(1)}°`
        );

        setText(
            `${id}-pitch`,
            vehicle.pitch_deg === null
                ? "--"
                : `${vehicle.pitch_deg.toFixed(1)}°`
        );

        setText(
            `${id}-hud-heading`,
            formatQuadrantalHeading(
                vehicle.heading_deg
            )
        );

        setText(
            `${id}-relative-altitude`,
            vehicle.relative_altitude === null
                ? "--"
                : `${vehicle.relative_altitude.toFixed(1)} m`
        );

        setText(
            `${id}-ground-speed`,
            vehicle.ground_speed === null
                ? "--"
                : `${vehicle.ground_speed.toFixed(1)} m/s`
        );

        setText(
            `${id}-vertical-speed`,
            vehicle.vertical_speed === null
                ? "--"
                : `${vehicle.vertical_speed.toFixed(1)} m/s`
        );


        setText(
            `${id}-gps-valid`,
            vehicle.gps_valid
                ? "VALID"
                : "INVALID"
        );

        setText(
            `${id}-gps-sigma`,
            vehicle.gps_sigma_m === null
                ? "--"
                : `${vehicle.gps_sigma_m.toFixed(1)} m`
        );

        setText(
            `${id}-local-position`,
            vehicle.local_position_valid
                ? "VALID"
                : "INVALID"
        );

        setText(
            `${id}-altitude-msl`,
            vehicle.altitude_msl === null
                ? "--"
                : `${vehicle.altitude_msl.toFixed(1)} m`
        );


        setBooleanCheck(
            `${id}-check-state-fresh`,
            vehicle.mavros_state_fresh
        );

        setBooleanCheck(
            `${id}-check-mavros-connected`,
            vehicle.mavros_connected
        );

        setBooleanCheck(
            `${id}-check-mode-allowed`,
            vehicle.mode_allowed
        );

        setBooleanCheck(
            `${id}-check-gps-valid`,
            vehicle.gps_valid
        );

        setBooleanCheck(
            `${id}-check-local-position`,
            vehicle.local_position_valid
        );

        setBooleanCheck(
            `${id}-check-battery-valid`,
            vehicle.battery_valid
        );

        setBooleanCheck(
            `${id}-check-autonomy-fresh`,
            vehicle.autonomy_status_fresh
        );

        setBooleanCheck(
            `${id}-check-mission-healthy`,
            vehicle.mission_healthy
        );

        setBooleanCheck(
            `${id}-check-failsafe-clear`,
            vehicle.failsafe_latched,
            true
        );

        setBooleanCheck(
            `${id}-check-prearm-ready`,
            vehicle.prearm_ready
        );

        setText(
            `${id}-check-manual-input`,
            vehicle.manual_input
                ? "TRUE"
                : "FALSE"
        );

        setText(
            `${id}-check-battery-percent`,
            vehicle.safety_battery_percentage == null
                ? "--"
                : `${vehicle.safety_battery_percentage.toFixed(0)} %`
        );

        setText(
            `${id}-check-reason`,
            vehicle.safety_reason ?? "--"
        );


        setText(
            `${id}-safety-state`,
            vehicle.safety_state ?? "UNKNOWN"
        );

        setText(
            `${id}-prearm-ready`,
            vehicle.prearm_ready
                ? "YES"
                : "NO"
        );

        setText(
            `${id}-flight-ready`,
            vehicle.flight_ready
                ? "YES"
                : "NO"
        );

        setText(
            `${id}-failsafe`,
            vehicle.failsafe_latched
                ? "LATCHED"
                : "CLEAR"
        );

        setText(
            `${id}-safety-reason`,
            vehicle.safety_reason ?? "--"
        );


        setText(
            `${id}-uav-autonomy`,
            vehicle.autonomy_enabled
                ? "ENABLED"
                : "OFF"
        );

        setText(
            `${id}-authorized`,
            vehicle.authorized
                ? "YES"
                : "NO"
        );

        setText(
            `${id}-command-fresh`,
            vehicle.command_fresh
                ? "YES"
                : "NO"
        );

        setText(
            `${id}-autonomy-reason`,
            vehicle.autonomy_reason ?? "--"
        );


        const modeButton =
            document.getElementById(
                `${id}-set-mode-button`
            );

        const armButton =
            document.getElementById(
                `${id}-arm-button`
            );

        const disarmButton =
            document.getElementById(
                `${id}-disarm-button`
            );

        const takeoffButton =
            document.getElementById(
                `${id}-takeoff-button`
            );

        const rtlButton =
            document.getElementById(
                `${id}-rtl-button`
            );

        const landButton =
            document.getElementById(
                `${id}-land-button`
            );

        const autonomyEnableButton =
            document.getElementById(
                `${id}-autonomy-enable-button`
            );

        const autonomyDisableButton =
            document.getElementById(
                `${id}-autonomy-disable-button`
            );

        const resetFailsafeButton =
            document.getElementById(
                `${id}-reset-failsafe-button`
            );


        // The browser only gates on transport availability.
        //
        // Vehicle-side ROS services remain the authoritative
        // safety/authorization boundary. This lets an operator
        // request a command and receive the actual rejection
        // reason instead of hiding the command in the GUI.
        //
        // The physical RC remains completely independent.

        if (modeButton) {
            modeButton.disabled =
                !vehicle.online;
        }

        if (armButton) {
            armButton.disabled =
                !vehicle.online;
        }

        if (disarmButton) {
            disarmButton.disabled =
                !vehicle.online;
        }

        if (takeoffButton) {
            takeoffButton.disabled =
                !vehicle.online;
        }

        if (rtlButton) {
            rtlButton.disabled =
                !vehicle.online;
        }

        if (landButton) {
            landButton.disabled =
                !vehicle.online;
        }

        if (autonomyEnableButton) {
            autonomyEnableButton.disabled =
                !vehicle.online;
        }

        if (autonomyDisableButton) {
            autonomyDisableButton.disabled =
                !vehicle.online;
        }

        if (resetFailsafeButton) {
            resetFailsafeButton.disabled =
                !vehicle.online;
        }
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
        formatQuadrantalHeading(
            vehicle.heading_deg
        )
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



async function missionProcessAction(
    vehicleId,
    action
) {

    const messageBox =
        document.getElementById(
            `${vehicleId}-mission-process-message`
        );


    const verb =
        action === "start"
            ? "START"
            : "STOP";


    const warning =
        action === "start"
            ? (
                `Start the ${vehicleId.toUpperCase()} `
                + "mission process?\n\n"
                + "This will NOT arm the vehicle, "
                + "change mode, or enable autonomy."
            )
            : (
                `Stop the ${vehicleId.toUpperCase()} `
                + "mission process?\n\n"
                + "Stopping also revokes autonomy "
                + "authorization."
            );


    // USV mission-process controls execute immediately.
    // Other vehicle types retain their existing confirmation.
    if (
        vehicleId !== "boat"
        && !confirm(warning)
    ) {
        return;
    }


    if (messageBox) {

        messageBox.textContent =
            `${verb} request in progress...`;
    }


    try {

        const response =
            await fetch(
                `/api/vehicles/${vehicleId}/mission_process`,
                {
                    method: "POST",
                    cache: "no-store",
                    headers: {
                        "Content-Type":
                            "application/json"
                    },
                    body: JSON.stringify({
                        action: action
                    })
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
                "Mission process request failed: "
                + error;
        }


        return {
            success: false,
            message: String(error)
        };
    }
}



async function postUsvControl(action, data = null) {

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
                cache: "no-store",
                headers: data === null
                    ? {}
                    : {"Content-Type": "application/json"},
                body: data === null
                    ? null
                    : JSON.stringify(data)
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

    let requestData = null;

    // ARM is the only USV action requiring an
    // additional operator confirmation.
    if (action === "arm") {

        if (!confirm(
            "ARM the USV? "
            + "Keep the physical E-stop accessible."
        )) {
            return;
        }
    }


    if (action === "reset_mission") {

        const labelInput =
            document.getElementById(
                "boat-log-label"
            );

        const label = labelInput
            ? labelInput.value.trim()
            : "";

        requestData = {
            label: label
        };
    }


    await postUsvControl(
        action,
        requestData
    );
}


async function toggleUsvStop(input) {

    const requestedEngaged =
        input.checked;


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




async function postUavControl(
    action,
    payload = null
) {

    const messageBox =
        document.getElementById(
            "uav-control-message"
        );

    if (messageBox) {
        messageBox.textContent =
            "Sending " + action + "...";
    }

    const options = {
        method: "POST",
        cache: "no-store"
    };

    if (payload !== null) {
        options.headers = {
            "Content-Type":
                "application/json"
        };

        options.body =
            JSON.stringify(payload);
    }


    try {

        const response = await fetch(
            `/api/uav/${action}`,
            options
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
                "UAV control request failed: "
                + error;
        }

        return {
            success: false,
            message: String(error)
        };
    }
}


async function uavSimpleAction(action) {

    // ARM is the only UAV action requiring
    // an additional operator confirmation.
    if (action === "arm") {

        if (!confirm(
            "ARM the UAV?\\n\\n"
            + "Propellers may become active if "
            + "vehicle-side execution is enabled."
        )) {
            return;
        }
    }

    await postUavControl(action);
}


async function uavSetMode() {

    const select =
        document.getElementById(
            "uav-mode-select"
        );

    if (!select) {
        return;
    }

    const mode =
        String(select.value).toUpperCase();

    await postUavControl(
        "set_mode",
        {
            mode: mode
        }
    );
}


async function uavTakeoff() {

    const input =
        document.getElementById(
            "uav-takeoff-altitude"
        );

    if (!input) {
        return;
    }

    const altitude =
        Number(input.value);

    if (
        !Number.isFinite(altitude)
        || altitude < 1.0
        || altitude > 5.0
    ) {
        alert(
            "Takeoff altitude must be "
            + "between 1.0 and 5.0 meters."
        );

        return;
    }

    await postUavControl(
        "takeoff",
        {
            altitude: altitude
        }
    );
}


async function uavSetAutonomy(enabled) {

    await postUavControl(
        "set_autonomy",
        {
            enabled: enabled
        }
    );
}



const GAMEPAD_DEADZONE = 0.12;
const GAMEPAD_PERIOD_MS = 50;

let operatorPostInFlight = false;
let operatorPendingState = null;
let operatorLastBackendSuccess = 0;

const OPERATOR_POST_TIMEOUT_MS = 200;
const OPERATOR_BACKEND_LOST_MS = 500;


function applyGamepadDeadzone(value) {

    value = Number(value) || 0.0;

    const magnitude = Math.abs(value);

    if (magnitude <= GAMEPAD_DEADZONE) {
        return 0.0;
    }

    const scaled =
        (magnitude - GAMEPAD_DEADZONE)
        / (1.0 - GAMEPAD_DEADZONE);

    return Math.sign(value) * scaled;
}


function findOperatorGamepad() {

    if (!navigator.getGamepads) {
        return null;
    }

    const pads = navigator.getGamepads();

    for (const pad of pads) {

        if (
            pad
            && pad.connected
            && pad.mapping === "standard"
        ) {
            return pad;
        }
    }

    for (const pad of pads) {

        if (pad && pad.connected) {
            return pad;
        }
    }

    return null;
}


function setControllerValue(
    id,
    text,
    state
) {

    const element =
        document.getElementById(id);

    if (!element) {
        return;
    }

    element.textContent = text;

    element.className =
        "value "
        + (
            state === true
                ? "connected"
                : state === false
                    ? "disconnected"
                    : ""
        );
}



async function flushOperatorInput() {

    if (
        operatorPostInFlight
        || operatorPendingState === null
    ) {
        return;
    }

    const payload =
        operatorPendingState;

    operatorPendingState = null;
    operatorPostInFlight = true;

    const controller =
        new AbortController();

    const timeoutId =
        setTimeout(
            () => controller.abort(),
            OPERATOR_POST_TIMEOUT_MS
        );

    try {

        const response = await fetch(
            "/api/operator_input",
            {
                method: "POST",
                cache: "no-store",
                signal: controller.signal,
                headers: {
                    "Content-Type":
                        "application/json"
                },
                body: JSON.stringify(
                    payload
                )
            }
        );

        const result =
            await response.json();

        if (result.success) {

            operatorLastBackendSuccess =
                performance.now();

            setControllerValue(
                "boat-operator-backend",
                "RECEIVING",
                true
            );

        } else {

            setControllerValue(
                "boat-operator-backend",
                "REJECTED",
                false
            );
        }

    } catch (error) {

        const age =
            performance.now()
            - operatorLastBackendSuccess;

        if (
            operatorLastBackendSuccess === 0
            || age >= OPERATOR_BACKEND_LOST_MS
        ) {
            setControllerValue(
                "boat-operator-backend",
                "LOST",
                false
            );
        }

    } finally {

        clearTimeout(timeoutId);

        operatorPostInFlight = false;

        if (operatorPendingState !== null) {
            setTimeout(
                flushOperatorInput,
                0
            );
        }
    }
}


function updateGamepad() {

    const pad =
        findOperatorGamepad();

    const connected = !!pad;

    let deadman = false;
    let forward = 0.0;
    let yaw = 0.0;


    if (pad) {

        const leftX =
            applyGamepadDeadzone(
                pad.axes[0] || 0.0
            );

        const leftY =
            applyGamepadDeadzone(
                pad.axes[1] || 0.0
            );

        deadman = !!(
            pad.buttons[4]
            && pad.buttons[4].pressed
        );

        if (deadman) {
            forward = -leftY;
            yaw = leftX;
        }
    }


    setControllerValue(
        "boat-gamepad",
        connected
            ? "CONNECTED"
            : "DISCONNECTED",
        connected
    );

    setControllerValue(
        "boat-deadman",
        deadman
            ? "HELD"
            : "RELEASED",
        deadman ? true : null
    );

    setText(
        "boat-operator-forward",
        `${(forward * 0.15).toFixed(3)} m/s`
    );

    setText(
        "boat-operator-yaw",
        `${(yaw * 0.15).toFixed(3)} rad/s`
    );


    // Keep only the newest controller state.
    // Never build a backlog of stale stick commands.
    operatorPendingState = {
        connected: connected,
        deadman: deadman,
        forward: forward,
        yaw: yaw
    };

    flushOperatorInput();
}



setInterval(
    updateGamepad,
    GAMEPAD_PERIOD_MS
);

updateGamepad();



function startAsyncPoll(callback, periodMs) {

    const run = async () => {

        try {
            await callback();
        } catch (error) {
            // Individual refresh functions already handle
            // their own UI error states.
        } finally {
            window.setTimeout(
                run,
                periodMs
            );
        }
    };

    run();
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


function rcEscape(value) {
    return String(value ?? "")
        .replaceAll("&", "&amp;")
        .replaceAll("<", "&lt;")
        .replaceAll(">", "&gt;")
        .replaceAll('"', "&quot;");
}


function rcAge(value) {

    if (
        value === null
        || value === undefined
        || !Number.isFinite(Number(value))
    ) {
        return "--";
    }

    return `${Number(value).toFixed(1)} s`;
}


function rcTrafficHtml(entries) {

    if (!Array.isArray(entries) || entries.length === 0) {
        return "No messages.";
    }

    return entries.map(entry => {

        const vehicle =
            entry.vehicle
                ? ` [${rcEscape(entry.vehicle)}]`
                : "";

        const detail =
            rcEscape(
                JSON.stringify(
                    entry.detail ?? {},
                    null,
                    2
                )
            );

        return `
            <details class="rc-entry">
                <summary>
                    ${rcEscape(entry.time)}
                    ${vehicle}
                    ${rcEscape(entry.type)}
                    —
                    ${rcEscape(entry.summary)}
                </summary>

                <pre>${detail}</pre>
            </details>
        `;
    }).join("");
}


function rcSetStatus(connected) {

    const element =
        document.getElementById(
            "rc-status"
        );

    if (!element) {
        return;
    }

    element.textContent =
        connected
            ? "CONNECTED"
            : "DISCONNECTED";

    element.className =
        "value "
        + (
            connected
                ? "connected"
                : "disconnected"
        );
}


function porSet(
    id,
    ok,
    goodText="READY",
    badText="NOT READY"
) {

    const element =
        document.getElementById(id);

    if (!element) {
        return;
    }

    element.textContent =
        ok
            ? goodText
            : badText;

    element.className =
        "value "
        + (
            ok
                ? "connected"
                : "disconnected"
        );
}


async function refreshPorReadiness() {

    const reasonElement =
        document.getElementById(
            "rc-por-reason"
        );

    try {

        const [
            vehiclesResponse,
            rcResponse,
            fenceResponse
        ] = await Promise.all([
            fetch(
                "/api/vehicles",
                {
                    cache: "no-store"
                }
            ),
            fetch(
                "/api/robocommand",
                {
                    cache: "no-store"
                }
            ),
            fetch(
                "/api/vehicles/uav/geofence",
                {
                    cache: "no-store"
                }
            )
        ]);


        const vehicles =
            await vehiclesResponse.json();

        const rc =
            await rcResponse.json();

        const fence =
            await fenceResponse.json();


        const boat =
            vehicles.boat ?? {};

        const uav =
            vehicles.uav ?? {};


        const brokerReady =
            Boolean(
                rc.connected
            );


        const courseCorners =
            Array.isArray(
                rc.course_corners
            )
                ? rc.course_corners
                : [];


        const courseReady =
            Boolean(
                rc.course_id
                && courseCorners.length >= 4
            );


        const usvCommsReady =
            Boolean(
                boat.online
                && boat.jetson_link
                && boat.mavlink_link
                && boat.mavros_connected
                && boat.mavros_state_fresh
            );


        const uavCommsReady =
            Boolean(
                uav.online
                && uav.vehicle_link
                && uav.mavros_connected
                && uav.mavros_state_fresh
            );


        const usvMissionReady =
            Boolean(
                boat.mission_process_running
            );


        const uavMissionReady =
            Boolean(
                uav.mission_process_running
            );


        const zones =
            Array.isArray(
                fence.zones
            )
                ? fence.zones
                : [];


        const inclusionPolygon =
            zones.find(
                zone =>
                    zone.type
                        === "inclusion_polygon"
                    &&
                    Array.isArray(
                        zone.points
                    )
                    &&
                    zone.points.length >= 3
            );


        const fenceReady =
            Boolean(
                fence.complete
                && inclusionPolygon
            );


        const checks = [
            [
                "RoboCommand broker",
                brokerReady
            ],
            [
                "course definition",
                courseReady
            ],
            [
                "USV communications",
                usvCommsReady
            ],
            [
                "UAV communications",
                uavCommsReady
            ],
            [
                "USV mission process",
                usvMissionReady
            ],
            [
                "UAV mission process",
                uavMissionReady
            ],
            [
                "native UAV geofence",
                fenceReady
            ]
        ];


        const prerequisitesReady =
            checks.every(
                item => item[1]
            );

        const runState =
            String(
                rc.run_state
                ?? "WAITING"
            ).toUpperCase();

        const reports =
            rc.vehicle_reports
            ?? {};

        const usvReport =
            reports.USV1
            ?? {};

        const uavReport =
            reports.UAV1
            ?? {};


        porSet(
            "rc-por-broker",
            brokerReady
        );

        porSet(
            "rc-por-course",
            courseReady
        );

        porSet(
            "rc-por-usv-comms",
            usvCommsReady
        );

        porSet(
            "rc-por-uav-comms",
            uavCommsReady
        );

        porSet(
            "rc-por-usv-mission",
            usvMissionReady,
            "RUNNING",
            "STOPPED"
        );

        porSet(
            "rc-por-uav-mission",
            uavMissionReady,
            "RUNNING",
            "STOPPED"
        );

        porSet(
            "rc-por-geofence",
            fenceReady,
            "VERIFIED",
            "NOT VERIFIED"
        );

        const overall =
            document.getElementById(
                "rc-por-overall"
            );

        if (overall) {

            if (
                runState === "STARTED"
                && rc.run_start_validated
            ) {

                overall.textContent =
                    "RUNSTART VERIFIED";

                overall.className =
                    "value connected";

            } else if (
                runState === "DECLARED"
            ) {

                overall.textContent =
                    "DECLARED — WAITING FOR AUTO";

                overall.className =
                    "value connected";

            } else if (
                prerequisitesReady
            ) {

                overall.textContent =
                    "READY TO DECLARE";

                overall.className =
                    "value connected";

            } else {

                overall.textContent =
                    "NOT READY";

                overall.className =
                    "value disconnected";
            }
        }


        const declarationButton =
            document.getElementById(
                "rc-declaration-button"
            );


        if (declarationButton) {

            declarationButton.disabled =
                !prerequisitesReady
                || runState !== "WAITING";
        }


        const failed =
            checks
            .filter(
                item => !item[1]
            )
            .map(
                item => item[0]
            );


        if (reasonElement) {

            if (
                runState === "STARTED"
                && rc.run_start_validated
            ) {

                reasonElement.textContent =
                    "RunStart verified. "
                    + `Run ID ${rc.run_id}.`;

            } else if (
                runState === "DECLARED"
            ) {

                const autoWaiting = [];

                if (
                    String(
                        usvReport.state
                        ?? ""
                    ).toUpperCase()
                    !== "AUTO"
                ) {
                    autoWaiting.push("USV1 AUTO");
                }

                if (
                    String(
                        uavReport.state
                        ?? ""
                    ).toUpperCase()
                    !== "AUTO"
                ) {
                    autoWaiting.push("UAV1 AUTO");
                }

                reasonElement.textContent =
                    autoWaiting.length
                        ? (
                            "Declaration sent. Waiting for: "
                            + autoWaiting.join(", ")
                        )
                        : (
                            "Both vehicles report AUTO. "
                            + "Waiting for RunStart."
                        );

            } else {

                reasonElement.textContent =
                    prerequisitesReady
                        ? (
                            "All local technical prerequisites "
                            + "are satisfied. Run Declaration "
                            + "is enabled."
                        )
                        : (
                            "Waiting for: "
                            + failed.join(", ")
                        );
            }
        }


    } catch (error) {

        porSet(
            "rc-por-overall",
            false,
            "READY TO DECLARE",
            "READINESS ERROR"
        );


        if (reasonElement) {

            reasonElement.textContent =
                "Readiness check failed: "
                + error;
        }
    }
}




function rcRunValue(
    id,
    text,
    good=null
) {

    const element =
        document.getElementById(id);

    if (!element) {
        return;
    }

    element.textContent = text;

    if (good === true) {
        element.className =
            "value connected";
    } else if (good === false) {
        element.className =
            "value disconnected";
    } else {
        element.className =
            "value";
    }
}


function updateRcRunControl(rc) {

    const runState =
        String(
            rc.run_state
            ?? "WAITING"
        ).toUpperCase();

    const reports =
        rc.vehicle_reports
        ?? {};

    const usv =
        reports.USV1
        ?? {};

    const uav =
        reports.UAV1
        ?? {};

    const usvState =
        String(
            usv.state
            ?? "UNKNOWN"
        ).toUpperCase();

    const uavState =
        String(
            uav.state
            ?? "UNKNOWN"
        ).toUpperCase();

    const declarationSent =
        rc.declaration_seq !== null
        && rc.declaration_seq !== undefined;

    const runStartValidated =
        Boolean(
            rc.run_start_validated
        );

    const validationMessage =
        String(
            rc.run_start_validation_message
            ?? ""
        );

    const runStartRejected =
        validationMessage
            .toLowerCase()
            .includes("rejected");


    rcRunValue(
        "rc-control-declaration",
        declarationSent
            ? "SENT"
            : "NOT SENT",
        declarationSent
            ? true
            : null
    );

    rcRunValue(
        "rc-control-declaration-seq",
        declarationSent
            ? String(rc.declaration_seq)
            : "--"
    );

    rcRunValue(
        "rc-control-usv",
        usvState,
        usvState === "AUTO"
            ? true
            : (
                usvState === "KILLED"
                    ? false
                    : null
            )
    );

    rcRunValue(
        "rc-control-uav",
        uavState,
        uavState === "AUTO"
            ? true
            : (
                uavState === "KILLED"
                    ? false
                    : null
            )
    );

    if (runStartValidated) {

        rcRunValue(
            "rc-control-runstart",
            "VERIFIED",
            true
        );

        rcRunValue(
            "rc-control-seq-check",
            "PASS",
            true
        );

    } else if (runStartRejected) {

        rcRunValue(
            "rc-control-runstart",
            "REJECTED",
            false
        );

        rcRunValue(
            "rc-control-seq-check",
            "FAIL",
            false
        );

    } else {

        rcRunValue(
            "rc-control-runstart",
            "WAITING",
            null
        );

        rcRunValue(
            "rc-control-seq-check",
            "WAITING",
            null
        );
    }


    rcRunValue(
        "rc-control-run-id",
        rc.run_id
            ?? "--"
    );


    const guidance =
        document.getElementById(
            "rc-control-guidance"
        );


    if (guidance) {

        if (
            runState === "STARTED"
            && runStartValidated
        ) {

            guidance.textContent =
                "RunStart verified. "
                + `Declaration seq ${rc.declaration_seq} `
                + `matches. Run ID ${rc.run_id}.`;

        } else if (runStartRejected) {

            guidance.textContent =
                validationMessage;

        } else if (runState === "DECLARED") {

            const waiting = [];

            if (usvState !== "AUTO") {
                waiting.push("USV1 AUTO");
            }

            if (uavState !== "AUTO") {
                waiting.push("UAV1 AUTO");
            }

            if (waiting.length > 0) {

                guidance.textContent =
                    "Declaration accepted locally. "
                    + "Waiting for: "
                    + waiting.join(", ")
                    + ".";

            } else {

                guidance.textContent =
                    "Both declared vehicles report AUTO. "
                    + "Waiting for RoboCommand RunStart.";
            }

        } else {

            guidance.textContent =
                "Ready for a RunDeclaration when "
                + "all declaration prerequisites pass.";
        }
    }


    const declarationButton =
        document.getElementById(
            "rc-declaration-button"
        );

    if (
        declarationButton
        && runState !== "WAITING"
    ) {
        declarationButton.disabled = true;
    }
}



async function refreshRoboCommand() {

    try {

        const response =
            await fetch(
                "/api/robocommand",
                {
                    cache: "no-store"
                }
            );

        const rc =
            await response.json();

        updateRcRunControl(rc);

        rcSetStatus(
            Boolean(rc.connected)
        );

        setText(
            "rc-broker",
            rc.broker ?? "--"
        );

        setText(
            "rc-team",
            rc.team_id ?? "--"
        );

        setText(
            "rc-course",
            rc.course_id ?? "--"
        );

        setText(
            "rc-pinger",
            rc.pinger_freq_hz == null
                ? "--"
                : `${rc.pinger_freq_hz} Hz`
        );

        setText(
            "rc-run-state",
            rc.run_state ?? "WAITING"
        );

        setText(
            "rc-declaration-seq",
            rc.declaration_seq ?? "--"
        );

        setText(
            "rc-run-id",
            rc.run_id ?? "--"
        );

        setText(
            "rc-last-command",
            rc.last_command ?? "--"
        );

        setText(
            "rc-last-rx",
            rcAge(rc.last_rx_age_sec)
        );

        setText(
            "rc-last-tx",
            rcAge(rc.last_tx_age_sec)
        );

        const reports =
            rc.vehicle_reports ?? {};

        const usv =
            reports.USV1 ?? {};

        const uav =
            reports.UAV1 ?? {};

        setText(
            "rc-usv-state",
            usv.state ?? "UNKNOWN"
        );

        setText(
            "rc-usv-age",
            rcAge(
                usv.last_tx_age_sec
            )
        );

        setText(
            "rc-uav-state",
            uav.state ?? "UNKNOWN"
        );

        setText(
            "rc-uav-age",
            rcAge(
                uav.last_tx_age_sec
            )
        );

        const rx =
            document.getElementById(
                "rc-rx-log"
            );

        if (rx) {
            rx.innerHTML =
                rcTrafficHtml(
                    rc.rx_history
                );
        }

        const tx =
            document.getElementById(
                "rc-tx-log"
            );

        if (tx) {
            tx.innerHTML =
                rcTrafficHtml(
                    rc.tx_history
                );
        }

    } catch (error) {

        rcSetStatus(false);

    }
}



function rcEvidenceValue(
    id,
    text,
    status=null
) {

    const element =
        document.getElementById(id);

    if (!element) {
        return;
    }

    element.textContent =
        text ?? "--";

    if (status === true) {

        element.className =
            "value connected";

    } else if (status === false) {

        element.className =
            "value disconnected";

    } else {

        element.className =
            "value";
    }
}


async function refreshPorEvidence() {

    try {

        const response =
            await fetch(
                "/api/robocommand/por/evidence",
                {
                    cache: "no-store"
                }
            );

        const evidence =
            await response.json();


        rcEvidenceValue(
            "rc-evidence-server",
            evidence.test_server_running
                ? "RUNNING"
                : "STOPPED",
            Boolean(
                evidence.test_server_running
            )
        );


        rcEvidenceValue(
            "rc-evidence-file",
            evidence.log_name
                ?? "--"
        );


        const vehicles =
            Array.isArray(
                evidence.declared_vehicles
            )
                ? evidence.declared_vehicles
                : [];


        rcEvidenceValue(
            "rc-evidence-vehicles",
            vehicles.length
                ? vehicles.join(", ")
                : "--",
            vehicles.length >= 2
                ? true
                : null
        );


        rcEvidenceValue(
            "rc-evidence-runstart",
            evidence.run_start_seen
                ? "YES"
                : "NO",
            evidence.run_start_seen
                ? true
                : null
        );


        rcEvidenceValue(
            "rc-evidence-declaration-seq",
            evidence.declaration_seq
                ?? "--"
        );


        rcEvidenceValue(
            "rc-evidence-runstart-seq",
            evidence.run_start_seq
                ?? "--"
        );


        rcEvidenceValue(
            "rc-evidence-sequence",
            evidence.sequence_match
                ? "PASS"
                : (
                    evidence.run_start_seen
                        ? "FAIL"
                        : "WAITING"
                ),
            evidence.sequence_match
                ? true
                : (
                    evidence.run_start_seen
                        ? false
                        : null
                )
        );


        rcEvidenceValue(
            "rc-evidence-run-id",
            evidence.run_id
                ?? "--"
        );


        rcEvidenceValue(
            "rc-evidence-result",
            evidence.pass
                ? "PASS"
                : "INCOMPLETE",
            Boolean(evidence.pass)
        );


        const message =
            document.getElementById(
                "rc-evidence-message"
            );


        if (message) {

            message.textContent =
                evidence.message
                ?? (
                    evidence.pass
                        ? (
                            "Official rc-test evidence "
                            + "is ready for submission."
                        )
                        : (
                            "Waiting for complete "
                            + "run-start evidence."
                        )
                );
        }


        const downloadButton =
            document.getElementById(
                "rc-download-log-button"
            );

        if (downloadButton) {

            downloadButton.disabled =
                !evidence.log_available;
        }

    } catch (error) {

        rcEvidenceValue(
            "rc-evidence-result",
            "ERROR",
            false
        );

        const message =
            document.getElementById(
                "rc-evidence-message"
            );

        if (message) {
            message.textContent =
                "Evidence status error: "
                + error;
        }
    }
}


async function rcViewOfficialLog() {

    const view =
        document.getElementById(
            "rc-official-log-view"
        );

    if (!view) {
        return;
    }


    if (
        view.style.display !== "none"
    ) {

        view.style.display = "none";
        return;
    }


    try {

        const response =
            await fetch(
                "/api/robocommand/por/log?tail=500",
                {
                    cache: "no-store"
                }
            );


        if (!response.ok) {

            const detail =
                await response.text();

            view.textContent =
                detail || "Log unavailable";

        } else {

            view.textContent =
                await response.text();
        }


        view.style.display = "block";


    } catch (error) {

        view.textContent =
            "Could not load log: "
            + error;

        view.style.display = "block";
    }
}


function rcDownloadOfficialLog() {

    window.location.href =
        "/api/robocommand/por/download";
}


async function rcStartNewPorTest() {

    const proceed =
        confirm(
            "Start a new official Communications PoR test?\n\n"
            + "This restarts only the RoboNation rc-test "
            + "server and resets the OCS run session.\n\n"
            + "It does NOT arm either vehicle, disable "
            + "autonomy, change flight modes, or command motion."
        );

    if (!proceed) {
        return;
    }


    const message =
        document.getElementById(
            "rc-evidence-message"
        );

    if (message) {
        message.textContent =
            "Starting new official PoR session...";
    }


    try {

        const response =
            await fetch(
                "/api/robocommand/por/new_session",
                {
                    method: "POST",
                    cache: "no-store"
                }
            );

        const result =
            await response.json();


        if (message) {

            message.textContent =
                result.message
                ?? "New session request completed.";
        }


        await refreshRoboCommand();
        await refreshPorReadiness();
        await refreshPorEvidence();


    } catch (error) {

        if (message) {

            message.textContent =
                "Could not start new PoR session: "
                + error;
        }
    }
}



async function rcPost(path) {

    const messageBox =
        document.getElementById(
            "rc-action-message"
        );

    try {

        const response =
            await fetch(
                path,
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
                ?? "No response.";
        }

        await refreshRoboCommand();

        return result;

    } catch (error) {

        if (messageBox) {
            messageBox.textContent =
                String(error);
        }

        return {
            success: false,
            message: String(error)
        };
    }
}


async function rcSendDeclaration() {
    await rcPost(
        "/api/robocommand/run_declaration"
    );
}


async function rcReconnect() {
    await rcPost(
        "/api/robocommand/reconnect"
    );
}


async function rcClearHistory() {
    await rcPost(
        "/api/robocommand/clear_history"
    );
}


startAsyncPoll(refreshRoboCommand, 750);

startAsyncPoll(refreshPorReadiness, 1500);
startAsyncPoll(refreshPorEvidence, 1500);

refreshPorEvidence();



installTabHandlers();

startAsyncPoll(refresh, 500);


/*
 * Automatically synchronize the Overview Map with the UAV's
 * native ArduPilot fence. No operator button is required.
 *
 * The dashboard may start before the UAV, so retry briefly
 * until a complete native fence is available.
 */
let overviewFenceStartupAttempts = 0;

async function overviewAutoLoadFence() {

    overviewFenceStartupAttempts += 1;

    const data =
        await overviewLoadFence(
            true
        );

    if (
        data
        && data.available
        && data.complete
    ) {
        return;
    }

    if (
        overviewFenceStartupAttempts
        < 10
    ) {

        setTimeout(
            overviewAutoLoadFence,
            2000
        );
    }
}

setTimeout(
    overviewAutoLoadFence,
    1500
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
            self.vehicle_manager.update_jetson_vehicle,
            host="192.168.2.20",
            port=8765,
            command_timeout=10.0,
        )

        self.vehicle_manager.register_client(
            "boat",
            self.boat_client,
        )

        self.boat_client.start()

        # Direct autopilot telemetry path. This is deliberately
        # receive-only; USV commands continue through the Jetson
        # TCP bridge and its existing safety authorization.
        self.boat_mavlink_client = BoatMavlinkClient(
            "boat",
            self.vehicle_manager.update_mavlink_vehicle,
            link_offline_callback=(
                self.vehicle_manager.mark_link_offline
            ),
            endpoint="udpin:0.0.0.0:14550",
            rx_timeout=3.0,
        )

        self.boat_mavlink_client.start()

        self.get_logger().info(
            "Monitoring USV autopilot directly via "
            "MAVLink UDP 0.0.0.0:14550"
        )

        self.uav_client = UavClient(
            "uav",
            self.vehicle_manager.update_vehicle,
            host="192.168.2.104",
            port=8766,
        )

        self.vehicle_manager.register_client(
            "uav",
            self.uav_client,
        )

        self.uav_client.start()

        # Direct UAV autopilot management channel.
        #
        # MAVROS on the UAV companion forwards the Pixhawk
        # MAVLink stream to Beeptop UDP 14552.
        #
        # This channel is used for GCS administration:
        # parameters now, then fence/mission management.
        #
        # The normal UAV telemetry/control UI continues to use
        # the guarded TCP bridge at port 8766.
        self.uav_mavlink_client = BoatMavlinkClient(
            "uav",
            lambda vehicle_id, fields: None,
            endpoint="udpin:0.0.0.0:14552",
            rx_timeout=3.0,
        )

        self.uav_mavlink_client.start()

        self.get_logger().info(
            "Monitoring UAV autopilot management via "
            "MAVLink UDP 0.0.0.0:14552"
        )

        # ----------------------------------------------------
        # RoboCommand OCS interface
        # ----------------------------------------------------
        #
        # Beeptop is the OCS. Only Beeptop connects to the
        # RoboCommand MQTT network. USV/UAV remain on our
        # internal vehicle TCP transports.

        self.robocommand_client = RoboCommandClient(
            vehicle_snapshot=self.snapshot,
            uav_geofence_snapshot=(
                self.uav_mavlink_client
                .fence_snapshot
            ),
        )

        self.robocommand_client.start()

        self.get_logger().info(
            "RoboCommand OCS client configured for "
            f"{self.robocommand_client.host}:"
            f"{self.robocommand_client.port} "
            f"team={self.robocommand_client.team_id}"
        )

        for vehicle_id, spec in VEHICLES.items():

            # USV and UAV telemetry arrive over their
            # vehicle-local TCP dashboard bridges.
            # Beeptop must not join either vehicle's ROS 2
            # DDS graph directly.
            if vehicle_id == "boat":
                self.get_logger().info(
                    "Monitoring USV via TCP bridge "
                    "at 192.168.2.20:8765"
                )
                continue

            if vehicle_id == "uav":
                self.get_logger().info(
                    "Monitoring UAV via TCP bridge "
                    "at 192.168.2.104:8766"
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






        self.register_control_routes()


    def destroy_node(self):

        if hasattr(self, "boat_client"):
            self.boat_client.stop()

        if hasattr(self, "boat_mavlink_client"):
            self.boat_mavlink_client.stop()

        if hasattr(self, "uav_client"):
            self.uav_client.stop()

        if hasattr(self, "uav_mavlink_client"):
            self.uav_mavlink_client.stop()

        if hasattr(self, "robocommand_client"):
            self.robocommand_client.stop()

        return super().destroy_node()


    # ========================================================
    # HTTP CONTROL ROUTES
    # ========================================================

    def register_control_routes(self):

        def result_response(func):

            success, message = func()

            status = self.usv_status()

            return jsonify({
                "success": bool(success),
                "message": str(message),
                "control_state":
                    status.get(
                        "control_state",
                        "UNKNOWN",
                    ),
                "software_stop":
                    status.get(
                        "software_stop",
                        "UNKNOWN",
                    ),
            })

        def operator_input():

            data = request.get_json(
                silent=True
            ) or {}

            if not isinstance(data, dict):
                return jsonify({
                    "success": False,
                    "message": "Invalid operator input",
                })

            current_deadman = bool(
                data.get("deadman", False)
            )

            previous_deadman = getattr(
                self,
                "_debug_last_browser_deadman",
                None,
            )

            if current_deadman != previous_deadman:

                self._debug_last_browser_deadman = (
                    current_deadman
                )

                self.get_logger().warning(
                    "BROWSER_OPERATOR_INPUT "
                    f"connected={bool(data.get('connected', False))} "
                    f"deadman={current_deadman} "
                    f"forward={data.get('forward', 0.0)} "
                    f"yaw={data.get('yaw', 0.0)}"
                )

            result = (
                self.boat_client
                .send_operator_input(data)
            )

            return jsonify(result)

        def reset_mission_response():
            data = request.get_json(
                silent=True
            ) or {}

            if not isinstance(data, dict):
                data = {}

            return result_response(
                lambda: self.execute_reset_mission(
                    data.get("label", "")
                )
            )

        app.add_url_rule(
            "/api/operator_input",
            endpoint="operator_input",
            view_func=operator_input,
            methods=["POST"],
        )

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
            view_func=reset_mission_response,
            methods=["POST"]
        )

        # ----------------------------------------------------
        # Vehicle parameter management
        #
        # Phase 1:
        #   USV -> direct ArduPilot MAVLink
        #
        # UAV/UUV will implement the same API contract when
        # their direct management transports are attached.
        # ----------------------------------------------------

        def parameter_client(vehicle_id):

            if vehicle_id == "boat":
                return self.boat_mavlink_client

            if vehicle_id == "uav":
                return self.uav_mavlink_client

            return None


        def vehicle_parameters(vehicle_id):

            client = parameter_client(
                vehicle_id
            )

            if client is None:

                return jsonify({
                    "vehicle_id": vehicle_id,
                    "available": False,
                    "write_allowed": False,
                    "armed": False,
                    "count": 0,
                    "expected": None,
                    "complete": False,
                    "parameters": [],
                    "message": (
                        "Direct parameter backend is "
                        "not attached to this vehicle yet"
                    ),
                })

            result = (
                client.parameter_snapshot()
            )

            result["vehicle_id"] = (
                vehicle_id
            )

            return jsonify(result)


        def refresh_vehicle_parameters(
            vehicle_id
        ):

            client = parameter_client(
                vehicle_id
            )

            if client is None:

                return jsonify({
                    "success": False,
                    "vehicle_id": vehicle_id,
                    "message": (
                        "Direct parameter backend is "
                        "not attached to this vehicle yet"
                    ),
                })

            result = (
                client.refresh_parameters()
            )

            result["vehicle_id"] = (
                vehicle_id
            )

            return jsonify(result)


        def set_vehicle_parameter(
            vehicle_id,
            param_name,
        ):

            client = parameter_client(
                vehicle_id
            )

            if client is None:

                return jsonify({
                    "success": False,
                    "message": (
                        "Direct parameter backend is "
                        "not attached to this vehicle yet"
                    ),
                })

            data = request.get_json(
                silent=True
            ) or {}

            if "value" not in data:

                return jsonify({
                    "success": False,
                    "message": (
                        "Parameter write requires "
                        "a value"
                    ),
                }), 400

            result = (
                client.set_parameter(
                    param_name,
                    data["value"],
                )
            )

            return jsonify(result)


        app.add_url_rule(
            "/api/vehicles/<vehicle_id>/parameters",
            endpoint="vehicle_parameters",
            view_func=vehicle_parameters,
            methods=["GET"],
        )

        app.add_url_rule(
            (
                "/api/vehicles/<vehicle_id>/"
                "parameters/refresh"
            ),
            endpoint="refresh_vehicle_parameters",
            view_func=refresh_vehicle_parameters,
            methods=["POST"],
        )

        app.add_url_rule(
            (
                "/api/vehicles/<vehicle_id>/"
                "parameters/<param_name>"
            ),
            endpoint="set_vehicle_parameter",
            view_func=set_vehicle_parameter,
            methods=["POST"],
        )


        # ----------------------------------------------------
        # Native ArduPilot geofence management
        # ----------------------------------------------------

        FENCE_PARAMETERS = (
            "FENCE_ENABLE",
            "FENCE_TYPE",
            "FENCE_ACTION",
            "FENCE_ALT_MAX",
            "FENCE_ALT_MIN",
            "FENCE_MARGIN",
            "FENCE_RADIUS",
            "FENCE_OPTIONS",
            "FENCE_ALT_MAX_TP",
            "FENCE_ALT_MIN_TP",
            "FENCE_TOTAL",
        )


        def geofence_settings(
            client
        ):

            snapshot = (
                client.parameter_snapshot()
            )

            values = {
                item["name"]:
                    item["value"]
                for item in snapshot.get(
                    "parameters",
                    []
                )
            }

            return {
                name: values.get(name)
                for name
                in FENCE_PARAMETERS
            }


        def enrich_geofence(
            client,
            result,
        ):

            output = dict(result)

            parameters = (
                client.parameter_snapshot()
            )

            output["settings"] = (
                geofence_settings(
                    client
                )
            )

            output["write_allowed"] = bool(
                parameters.get(
                    "write_allowed",
                    False,
                )
            )

            return output


        def vehicle_geofence(
            vehicle_id
        ):

            client = parameter_client(
                vehicle_id
            )

            if client is None:

                return jsonify({
                    "success": False,
                    "vehicle_id":
                        vehicle_id,
                    "available": False,
                    "write_allowed": False,
                    "message": (
                        "Direct MAVLink "
                        "geofence backend "
                        "is not attached"
                    ),
                })


            result = enrich_geofence(
                client,
                client.fence_snapshot(),
            )

            result["vehicle_id"] = (
                vehicle_id
            )

            return jsonify(result)


        def refresh_vehicle_geofence(
            vehicle_id
        ):

            client = parameter_client(
                vehicle_id
            )

            if client is None:

                return jsonify({
                    "success": False,
                    "vehicle_id":
                        vehicle_id,
                    "message": (
                        "Direct MAVLink "
                        "geofence backend "
                        "is not attached"
                    ),
                })


            result = enrich_geofence(
                client,
                client.download_fence(),
            )

            result["vehicle_id"] = (
                vehicle_id
            )

            return jsonify(result)


        def write_vehicle_geofence(
            vehicle_id
        ):

            if vehicle_id != "uav":

                return jsonify({
                    "success": False,
                    "message": (
                        "Native fence editing "
                        "is currently enabled "
                        "only for the UAV"
                    ),
                }), 400


            client = parameter_client(
                vehicle_id
            )


            if client is None:

                return jsonify({
                    "success": False,
                    "message": (
                        "Direct MAVLink "
                        "geofence backend "
                        "is not attached"
                    ),
                }), 503


            data = request.get_json(
                silent=True
            ) or {}


            if "zones" in data:

                result = (
                    client
                    .upload_fence_zones(
                        data["zones"]
                    )
                )

            else:

                result = (
                    client.upload_fence(
                        data.get(
                            "vertices",
                            [],
                        )
                    )
                )


            result["vehicle_id"] = (
                vehicle_id
            )

            return jsonify(result)


        def write_geofence_settings(
            vehicle_id
        ):

            if vehicle_id != "uav":

                return jsonify({
                    "success": False,
                    "message": (
                        "Fence settings are "
                        "currently enabled only "
                        "for the UAV"
                    ),
                }), 400


            client = parameter_client(
                vehicle_id
            )


            if client is None:

                return jsonify({
                    "success": False,
                    "message": (
                        "Direct MAVLink "
                        "backend unavailable"
                    ),
                }), 503


            data = request.get_json(
                silent=True
            ) or {}


            if not isinstance(
                data,
                dict,
            ):

                return jsonify({
                    "success": False,
                    "message":
                        "Invalid settings payload",
                }), 400


            allowed = set(
                FENCE_PARAMETERS
            )

            allowed.discard(
                "FENCE_TOTAL"
            )


            results = {}


            for name, value in data.items():

                if name not in allowed:
                    continue


                if value is None:
                    continue


                result = (
                    client.set_parameter(
                        name,
                        value,
                    )
                )

                results[name] = result


                if not result.get(
                    "success",
                    False,
                ):

                    return jsonify({
                        "success": False,
                        "message": (
                            f"{name}: "
                            + str(
                                result.get(
                                    "message",
                                    "write failed",
                                )
                            )
                        ),
                        "results": results,
                    })


            return jsonify({
                "success": True,
                "message": (
                    "Fence settings written "
                    "and verified."
                ),
                "results": results,
                "settings":
                    geofence_settings(
                        client
                    ),
            })


        app.add_url_rule(
            (
                "/api/vehicles/"
                "<vehicle_id>/geofence"
            ),
            endpoint="vehicle_geofence",
            view_func=vehicle_geofence,
            methods=["GET"],
        )


        app.add_url_rule(
            (
                "/api/vehicles/"
                "<vehicle_id>/"
                "geofence/refresh"
            ),
            endpoint=(
                "refresh_vehicle_geofence"
            ),
            view_func=(
                refresh_vehicle_geofence
            ),
            methods=["POST"],
        )


        app.add_url_rule(
            (
                "/api/vehicles/"
                "<vehicle_id>/geofence"
            ),
            endpoint=(
                "write_vehicle_geofence"
            ),
            view_func=(
                write_vehicle_geofence
            ),
            methods=["POST"],
        )


        app.add_url_rule(
            (
                "/api/vehicles/"
                "<vehicle_id>/"
                "geofence/settings"
            ),
            endpoint=(
                "write_geofence_settings"
            ),
            view_func=(
                write_geofence_settings
            ),
            methods=["POST"],
        )


        # ----------------------------------------------------
        # UAV remote command routes
        # ----------------------------------------------------

        def uav_command_response(
            command,
            data=None,
        ):
            result = self.uav_client.command(
                command,
                data or {},
            )

            return jsonify({
                "success": bool(
                    result.get(
                        "success",
                        False,
                    )
                ),
                "message": str(
                    result.get(
                        "message",
                        "",
                    )
                ),
            })

        def uav_set_mode():
            data = request.get_json(
                silent=True
            ) or {}

            mode = data.get("mode")

            if (
                not isinstance(mode, str)
                or not mode.strip()
            ):
                return jsonify({
                    "success": False,
                    "message": (
                        "set_mode requires a "
                        "non-empty mode"
                    ),
                }), 400

            return uav_command_response(
                "set_mode",
                {
                    "mode": mode.strip().upper(),
                },
            )

        def uav_takeoff():
            data = request.get_json(
                silent=True
            ) or {}

            altitude = data.get("altitude")

            try:
                altitude = float(altitude)

            except (
                TypeError,
                ValueError,
            ):
                return jsonify({
                    "success": False,
                    "message": (
                        "takeoff requires a "
                        "numeric altitude"
                    ),
                }), 400

            return uav_command_response(
                "takeoff",
                {
                    "altitude": altitude,
                },
            )

        def uav_set_autonomy():
            data = request.get_json(
                silent=True
            ) or {}

            enabled = data.get("enabled")

            if not isinstance(enabled, bool):
                return jsonify({
                    "success": False,
                    "message": (
                        "set_autonomy requires "
                        "boolean enabled"
                    ),
                }), 400

            return uav_command_response(
                "set_autonomy",
                {
                    "enabled": enabled,
                },
            )

        app.add_url_rule(
            "/api/uav/arm",
            endpoint="uav_arm",
            view_func=lambda:
                uav_command_response("arm"),
            methods=["POST"],
        )

        app.add_url_rule(
            "/api/uav/disarm",
            endpoint="uav_disarm",
            view_func=lambda:
                uav_command_response("disarm"),
            methods=["POST"],
        )

        app.add_url_rule(
            "/api/uav/set_mode",
            endpoint="uav_set_mode",
            view_func=uav_set_mode,
            methods=["POST"],
        )

        app.add_url_rule(
            "/api/uav/takeoff",
            endpoint="uav_takeoff",
            view_func=uav_takeoff,
            methods=["POST"],
        )

        app.add_url_rule(
            "/api/uav/land",
            endpoint="uav_land",
            view_func=lambda:
                uav_command_response("land"),
            methods=["POST"],
        )

        app.add_url_rule(
            "/api/uav/rtl",
            endpoint="uav_rtl",
            view_func=lambda:
                uav_command_response("rtl"),
            methods=["POST"],
        )

        app.add_url_rule(
            "/api/uav/set_autonomy",
            endpoint="uav_set_autonomy",
            view_func=uav_set_autonomy,
            methods=["POST"],
        )

        app.add_url_rule(
            "/api/uav/reset_failsafe",
            endpoint="uav_reset_failsafe",
            view_func=lambda:
                uav_command_response(
                    "reset_failsafe"
                ),
            methods=["POST"],
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

                vehicle_link_rx = data.pop(
                    "vehicle_link_last_rx",
                    None
                )

                vehicle_link_age = (
                    None
                    if vehicle_link_rx is None
                    else now - vehicle_link_rx
                )

                jetson_link_rx = data.pop(
                    "jetson_link_last_rx",
                    None,
                )

                mavlink_link_rx = data.pop(
                    "mavlink_link_last_rx",
                    None,
                )

                jetson_link_age = (
                    None
                    if jetson_link_rx is None
                    else now - jetson_link_rx
                )

                mavlink_link_age = (
                    None
                    if mavlink_link_rx is None
                    else now - mavlink_link_rx
                )

                data[
                    "jetson_link_age_sec"
                ] = jetson_link_age

                data[
                    "mavlink_link_age_sec"
                ] = mavlink_link_age

                if vehicle_id == "boat":

                    # TCP bridge normally publishes at 5 Hz.
                    data["jetson_link"] = bool(
                        data.get(
                            "jetson_link",
                            False,
                        )
                        and
                        jetson_link_age is not None
                        and
                        jetson_link_age < 2.0
                    )

                    # Direct MAVLink is authoritative for
                    # autopilot presence. Any accepted MAVLink
                    # traffic refreshes this timestamp.
                    data["mavlink_link"] = bool(
                        data.get(
                            "mavlink_link",
                            False,
                        )
                        and
                        mavlink_link_age is not None
                        and
                        mavlink_link_age < 3.0
                    )

                    data["vehicle_link"] = bool(
                        data["jetson_link"]
                        or data["mavlink_link"]
                    )

                    valid_link_ages = [
                        value
                        for value in (
                            jetson_link_age,
                            mavlink_link_age,
                        )
                        if value is not None
                    ]

                    data[
                        "vehicle_link_age_sec"
                    ] = (
                        min(valid_link_ages)
                        if valid_link_ages
                        else None
                    )

                    # Communications PoR definition:
                    # USV online = direct autopilot MAVLink alive.
                    data["online"] = bool(
                        data["mavlink_link"]
                        and data["connected"]
                    )

                    # For the USV, age_sec represents the
                    # authoritative autopilot transport age.
                    data["age_sec"] = mavlink_link_age

                else:

                    data["online"] = bool(
                        data["connected"]
                        and age is not None
                        and age < 2.0
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

                logger_rx = data.pop(
                    "logger_last_rx",
                    None
                )

                servo_output_rx = data.pop(
                    "servo_output_last_rx",
                    None
                )

                data[
                    "servo_output_age_sec"
                ] = (
                    None
                    if servo_output_rx is None
                    else now - servo_output_rx
                )

                data[
                    "servo_output_fresh"
                ] = bool(
                    servo_output_rx is not None
                    and
                    now - servo_output_rx
                    <= 1.0
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

                    # The Jetson bridge owns USV control and
                    # safety state. Beeptop only presents it.
                    data["bridge_alive"] = bool(
                        data.get(
                            "bridge_alive",
                            False,
                        )
                    )

                    if not data["online"]:
                        data["control_state"] = (
                            "AUTOPILOT OFFLINE"
                        )

                    elif not data["jetson_link"]:
                        # Vehicle remains visible through direct
                        # MAVLink even if Jetson ROS/TCP is down.
                        data["bridge_alive"] = False
                        data["control_state"] = (
                            "JETSON OFFLINE"
                        )

                    data["software_stop"] = str(
                        data.get(
                            "software_stop",
                            "UNKNOWN",
                        )
                    )

                    data["autonomy_enabled"] = bool(
                        data.get(
                            "autonomy_enabled",
                            False,
                        )
                    )

                    data["can_enable"] = bool(
                        data.get(
                            "can_enable",
                            False,
                        )
                        and data["online"]
                        and data["jetson_link"]
                    )

                    data["logger_fresh"] = bool(
                        data["jetson_link"]
                        and logger_rx is not None
                        and now - logger_rx <= 1.0
                    )

                    if not data["logger_fresh"]:
                        data["log_state"] = "UNAVAILABLE"
                        data["log_pending"] = False
                        data["log_recording"] = False

                else:

                    data["bridge_alive"] = False
                    data["control_state"] = "---"
                    data["software_stop"] = "---"

                    # UAV autonomy state comes from the
                    # authoritative vehicle-local
                    # /vehicle/autonomy_status telemetry.
                    #
                    # Do not overwrite it here merely because
                    # the vehicle is not the USV.
                    if vehicle_id == "uav":
                        data["autonomy_enabled"] = bool(
                            data.get(
                                "autonomy_enabled",
                                False,
                            )
                        )
                    else:
                        data["autonomy_enabled"] = False

                    data["can_enable"] = False
                    data["logger_fresh"] = False

                output[vehicle_id] = data

        return output


    def usv_status(self):

        return self.snapshot()["boat"]


    # ========================================================
    # ROS SERVICE HELPERS
    # ========================================================











    # ========================================================
    # USV CONTROL ACTIONS
    # ========================================================

    def execute_remote_usv_command(
        self,
        command,
        data=None,
    ):

        with self.action_lock:

            if not self.boat_client.connected:
                return (
                    False,
                    "USV command rejected: "
                    "Jetson TCP link is offline",
                )

            try:
                result = self.boat_client.command(
                    command,
                    data or {},
                )

            except Exception as exc:
                return (
                    False,
                    "USV command transport error: "
                    + str(exc),
                )

            if not isinstance(result, dict):
                return (
                    False,
                    "Invalid response from Jetson bridge",
                )

            return (
                bool(
                    result.get(
                        "success",
                        False,
                    )
                ),
                str(
                    result.get(
                        "message",
                        "No response message",
                    )
                ),
            )


    def execute_fail_safe_stop(self):

        return self.execute_remote_usv_command(
            "stop"
        )


    def execute_stop(self):

        return self.execute_remote_usv_command(
            "stop"
        )


    def execute_clear_stop(self):

        return self.execute_remote_usv_command(
            "clear_stop"
        )


    def execute_enable(self):

        return self.execute_remote_usv_command(
            "enable"
        )


    def execute_arm(self):

        return self.execute_remote_usv_command(
            "arm"
        )


    def execute_disarm(self):

        return self.execute_remote_usv_command(
            "disarm"
        )


    def execute_reset_mission(self, label=""):

        return self.execute_remote_usv_command(
            "reset_mission",
            {"label": str(label or "")},
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




@app.route(
    "/api/vehicles/<vehicle_id>/mission_process",
    methods=["GET", "POST"],
)
def api_vehicle_mission_process(vehicle_id):

    if dashboard_node is None:
        return jsonify({
            "success": False,
            "message": "Dashboard node unavailable",
        }), 503

    if vehicle_id not in ("boat", "uav"):
        return jsonify({
            "success": False,
            "message": (
                "Mission process control is only "
                "available for boat and uav"
            ),
        }), 404

    if request.method == "GET":
        action = "status"
    else:
        data = request.get_json(
            silent=True
        ) or {}

        action = str(
            data.get("action", "status")
        ).strip().lower()

    if action not in (
        "start",
        "stop",
        "status",
    ):
        return jsonify({
            "success": False,
            "message": (
                "action must be start, stop, or status"
            ),
        }), 400

    client = (
        dashboard_node
        .vehicle_manager
        .get_client(vehicle_id)
    )

    if client is None:
        return jsonify({
            "success": False,
            "message": "Vehicle client unavailable",
        }), 503

    result = client.command(
        "mission_process",
        {
            "action": action,
        },
    )

    return jsonify(result)



RC_TEST_ROOT = Path(
    "/home/epeop/"
    "RoboCommand_Official/"
    "RobotX_2026"
)

RC_TEST_LOG_DIR = (
    RC_TEST_ROOT
    / "logs"
)


def latest_rc_test_log():

    if not RC_TEST_LOG_DIR.exists():
        return None

    logs = list(
        RC_TEST_LOG_DIR.glob(
            "robocommand_*.log"
        )
    )

    if not logs:
        return None

    return max(
        logs,
        key=lambda path:
            path.stat().st_mtime,
    )


def rc_test_server_running():

    if not RC_TEST_ROOT.exists():
        return False

    try:

        result = subprocess.run(
            [
                "/usr/bin/docker",
                "compose",
                "ps",
                "--status",
                "running",
                "--services",
            ],
            cwd=str(RC_TEST_ROOT),
            capture_output=True,
            text=True,
            timeout=3.0,
            check=False,
        )

        if result.returncode != 0:
            return False

        services = {
            line.strip()
            for line
            in result.stdout.splitlines()
            if line.strip()
        }

        return "rc-test" in services

    except Exception:
        return False


def parse_rc_test_evidence(
    team_id="ULLY",
):

    path = latest_rc_test_log()

    result = {
        "test_server_running":
            rc_test_server_running(),
        "log_available": False,
        "log_name": None,
        "log_size_bytes": None,
        "declared_vehicles": [],
        "declaration_seq": None,
        "run_start_seen": False,
        "run_start_seq": None,
        "run_id": None,
        "sequence_match": False,
        "pass": False,
        "message": (
            "No official rc-test log "
            "has been found yet."
        ),
    }


    if path is None:
        return result


    result["log_available"] = True
    result["log_name"] = path.name
    result["log_size_bytes"] = (
        path.stat().st_size
    )


    try:
        text = path.read_text(
            errors="replace"
        )

    except Exception as exc:

        result["message"] = (
            "Could not read rc-test log: "
            + str(exc)
        )

        return result


    # --------------------------------------------------------
    # Find the latest RunDeclaration acknowledged by the
    # official rc-test server for our team.
    #
    # The official server emits:
    #
    # [auto-start] Tracking ULLY vehicles [...]
    # --------------------------------------------------------

    tracking = None

    pattern = re.compile(
        r"\[auto-start\]\s+Tracking\s+"
        + re.escape(team_id)
        + r"\s+vehicles\s+\[(.*?)\]"
    )


    for match in pattern.finditer(text):
        tracking = match


    if tracking is None:

        result["message"] = (
            "Official log exists, but no "
            "RunDeclaration for "
            + team_id
            + " has been observed yet."
        )

        return result


    vehicle_text = tracking.group(1)

    vehicles = re.findall(
        r"""['"]([^'"]+)['"]""",
        vehicle_text,
    )

    result["declared_vehicles"] = (
        vehicles
    )


    # The full decoded RxRequest is logged immediately before
    # the tracking line. Pull the declaration sequence from
    # that decoded message.

    request_start = text.rfind(
        "[RxRequest]",
        0,
        tracking.start(),
    )


    if request_start >= 0:

        request_block = text[
            request_start:
            tracking.start()
        ]

        if (
            "run_declaration {" in request_block
            and
            f'team_id: "{team_id}"'
            in request_block
        ):

            seq_match = re.search(
                r"(?m)^seq:\s*(\d+)\s*$",
                request_block,
            )

            if seq_match:

                result["declaration_seq"] = int(
                    seq_match.group(1)
                )


    # --------------------------------------------------------
    # Find RunStart produced after this declaration.
    #
    # The official rc-test server emits:
    #
    # All declared vehicles autonomous ... publishing
    # RunStart (declaration_seq=N, run_id=M)
    # --------------------------------------------------------

    start_pattern = re.compile(
        r"\[auto-start\]\s+"
        r"All declared vehicles autonomous for\s+"
        + re.escape(team_id)
        + r"[^\n]*"
        r"publishing RunStart\s+"
        r"\(declaration_seq=(\d+),\s*"
        r"run_id=(\d+)\)"
    )


    run_start = start_pattern.search(
        text,
        tracking.end(),
    )


    if run_start is not None:

        result["run_start_seen"] = True

        result["run_start_seq"] = int(
            run_start.group(1)
        )

        result["run_id"] = int(
            run_start.group(2)
        )


    result["sequence_match"] = bool(
        result["declaration_seq"]
        is not None
        and
        result["run_start_seq"]
        is not None
        and
        result["declaration_seq"]
        == result["run_start_seq"]
    )


    result["pass"] = bool(
        len(
            result["declared_vehicles"]
        ) >= 2
        and
        result["run_start_seen"]
        and
        result["sequence_match"]
        and
        result["run_id"]
        is not None
    )


    if result["pass"]:

        result["message"] = (
            "PASS: official rc-test log contains "
            "a two-vehicle RunDeclaration and "
            "matching RunStart. "
            "Download this log for the "
            "Communications PoR submission."
        )

    else:

        waiting = []

        if len(
            result["declared_vehicles"]
        ) < 2:
            waiting.append(
                "RunDeclaration with >=2 vehicles"
            )

        if not result[
            "run_start_seen"
        ]:
            waiting.append(
                "RunStart"
            )

        elif not result[
            "sequence_match"
        ]:
            waiting.append(
                "matching declaration sequence"
            )

        result["message"] = (
            "INCOMPLETE: waiting for "
            + ", ".join(waiting)
        )


    return result


@app.route(
    "/api/robocommand/por/evidence"
)
def api_robocommand_por_evidence():

    return jsonify(
        parse_rc_test_evidence()
    )


@app.route(
    "/api/robocommand/por/log"
)
def api_robocommand_por_log():

    path = latest_rc_test_log()

    if path is None:
        return Response(
            "No official rc-test log found.",
            status=404,
            mimetype="text/plain",
        )


    try:
        requested_tail = int(
            request.args.get(
                "tail",
                "500",
            )
        )

    except ValueError:
        requested_tail = 500


    requested_tail = max(
        1,
        min(
            requested_tail,
            2000,
        ),
    )


    text = path.read_text(
        errors="replace"
    )

    lines = text.splitlines()

    return Response(
        "\n".join(
            lines[-requested_tail:]
        ),
        mimetype="text/plain",
    )


@app.route(
    "/api/robocommand/por/download"
)
def api_robocommand_por_download():

    path = latest_rc_test_log()

    if path is None:
        return jsonify({
            "success": False,
            "message": (
                "No official rc-test log found."
            ),
        }), 404


    return send_file(
        path,
        mimetype="text/plain",
        as_attachment=True,
        download_name=path.name,
    )


@app.route(
    "/api/robocommand/por/new_session",
    methods=["POST"],
)
def api_robocommand_por_new_session():

    if dashboard_node is None:
        return jsonify({
            "success": False,
            "message": (
                "Dashboard node unavailable."
            ),
        }), 503


    if not RC_TEST_ROOT.exists():
        return jsonify({
            "success": False,
            "message": (
                "Official RoboCommand test "
                "installation was not found."
            ),
        }), 404


    old_log = latest_rc_test_log()


    # Reset only local OCS run/session state.
    #
    # This does NOT send any vehicle command.
    reset_result = (
        dashboard_node
        .robocommand_client
        .reset_run_session()
    )


    try:

        restart = subprocess.run(
            [
                "/usr/bin/docker",
                "compose",
                "restart",
                "rc-test",
            ],
            cwd=str(RC_TEST_ROOT),
            capture_output=True,
            text=True,
            timeout=20.0,
            check=False,
        )

    except Exception as exc:

        return jsonify({
            "success": False,
            "message": (
                "Could not restart official "
                "rc-test server: "
                + str(exc)
            ),
        }), 500


    if restart.returncode != 0:

        detail = (
            restart.stderr.strip()
            or restart.stdout.strip()
            or "unknown docker error"
        )

        return jsonify({
            "success": False,
            "message": (
                "rc-test restart failed: "
                + detail
            ),
        }), 500


    # Give rc-test a moment to create its new timestamped log.

    deadline = (
        time.monotonic()
        + 8.0
    )

    new_log = None

    while (
        time.monotonic()
        < deadline
    ):

        candidate = (
            latest_rc_test_log()
        )

        if (
            candidate is not None
            and candidate != old_log
        ):
            new_log = candidate
            break

        time.sleep(0.25)


    return jsonify({
        "success": True,
        "message": (
            "New official Communications PoR "
            "session started. "
            "Vehicle autonomy, modes and arming "
            "were not changed. "
            "Send one new Run Declaration "
            "from the RoboCommand page."
        ),
        "new_log": (
            None
            if new_log is None
            else new_log.name
        ),
        "ocs_reset": bool(
            reset_result.get(
                "success",
                False,
            )
        ),
    })


@app.route("/api/robocommand")
def api_robocommand():

    if (
        dashboard_node is None
        or not hasattr(
            dashboard_node,
            "robocommand_client",
        )
    ):
        return jsonify({
            "connected": False,
        })

    return jsonify(
        dashboard_node
        .robocommand_client
        .snapshot()
    )


@app.route(
    "/api/robocommand/run_declaration",
    methods=["POST"],
)
def api_robocommand_run_declaration():

    if dashboard_node is None:
        return jsonify({
            "success": False,
            "message": (
                "Dashboard node unavailable."
            ),
        }), 503

    result = (
        dashboard_node
        .robocommand_client
        .send_run_declaration()
    )

    return jsonify(result)


@app.route(
    "/api/robocommand/reconnect",
    methods=["POST"],
)
def api_robocommand_reconnect():

    if dashboard_node is None:
        return jsonify({
            "success": False,
            "message": (
                "Dashboard node unavailable."
            ),
        }), 503

    return jsonify(
        dashboard_node
        .robocommand_client
        .reconnect()
    )


@app.route(
    "/api/robocommand/clear_history",
    methods=["POST"],
)
def api_robocommand_clear_history():

    if dashboard_node is None:
        return jsonify({
            "success": False,
            "message": (
                "Dashboard node unavailable."
            ),
        }), 503

    return jsonify(
        dashboard_node
        .robocommand_client
        .clear_history()
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

        serve(
            app,
            host="0.0.0.0",
            port=8080,
            threads=8,
        )

    finally:

        dashboard_node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
