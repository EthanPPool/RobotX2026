# RobotX UAV ROS 2 Workspace — v0.3

ROS 2 Kilted autonomy stack for the RobotX UAV companion computer.

## v0.3 scope

v0.3 adds a safety-aware ArduCopter command-management layer while preserving
the v0.2 perception → mission → control → safety-gated velocity pipeline. The
new `vehicle_manager` exposes stable `/vehicle/*` services for flight mode,
arm/disarm, takeoff, LAND, and RTL.

Real-hardware command execution is **disabled by default** in `autonomy.yaml`.
The software-only launch explicitly enables command execution because all
MAVROS endpoints are provided by `mock_mavros`.

## Packages

- `uav_interfaces` — shared messages/services.
- `uav_vehicle` — velocity gate, ArduCopter command manager, mock MAVROS.
- `uav_safety` — pre-arm/flight readiness and latched FAILSAFE.
- `uav_control` — bounded velocity setpoints.
- `uav_perception` — target normalization/mock target.
- `uav_mission` — mission state machine.
- `uav_bringup` — configuration and launches.

## Safety semantics changed in v0.3

`READY` now means **safe to arm**, not “already armed.” This removes the
circular dependency that would otherwise prevent ROS from issuing the arm
command. `SafetyStatus` now includes:

- `prearm_ready` — navigation, battery, telemetry, mode, mission and command path healthy.
- `flight_ready` — `prearm_ready` plus `armed=true`.
- `ready` — retained as an alias of `prearm_ready` for compatibility.

The state machine remains:

- `INIT` — required streams not all observed.
- `NOT_READY` — one or more pre-arm prerequisites invalid.
- `READY` — safe to arm / issue guarded vehicle commands.
- `ACTIVE` — autonomy enabled and all flight prerequisites healthy.
- `FAILSAFE` — a flight prerequisite failed while autonomy was enabled.

FAILSAFE remains latched and requires autonomy disabled plus explicit reset.

## Vehicle command services

```text
/vehicle/arm       std_srvs/srv/Trigger
/vehicle/disarm    std_srvs/srv/Trigger
/vehicle/set_mode  uav_interfaces/srv/SetFlightMode
/vehicle/takeoff   uav_interfaces/srv/Takeoff
/vehicle/land      std_srvs/srv/Trigger
/vehicle/rtl       std_srvs/srv/Trigger
```

Internally, the manager uses MAVROS `/mavros/cmd/arming`, `/mavros/set_mode`,
and `/mavros/cmd/takeoff`. LAND and RTL are requested as ArduCopter modes.

## Hardware lock

`autonomy.yaml` sets:

```yaml
command_execution_enabled: false
```

Therefore a real MAVROS connection cannot be armed, take off, or change flight
mode through `vehicle_manager` until this parameter is deliberately enabled.
Do not enable it for physical bench testing until the propulsion installation is
complete and propellers are removed.

## Build

```bash
cd ~/uav_ws
source /opt/ros/kilted/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --executor sequential
source install/setup.bash
```

## Software-only test

```bash
ros2 launch uav_bringup software_test.launch.py
```

The mock starts connected, `GUIDED`, healthy, and **disarmed**. The expected
safety state is `READY` with `prearm_ready=true` and `flight_ready=false`.

Arm the mock:

```bash
ros2 service call /vehicle/arm std_srvs/srv/Trigger "{}"
```

Then `flight_ready` should become true. Test takeoff:

```bash
ros2 service call /vehicle/takeoff uav_interfaces/srv/Takeoff "{altitude: 2.0}"
```

Test LAND and RTL:

```bash
ros2 service call /vehicle/land std_srvs/srv/Trigger "{}"
ros2 service call /vehicle/set_mode uav_interfaces/srv/SetFlightMode "{mode: 'GUIDED'}"
ros2 service call /vehicle/rtl std_srvs/srv/Trigger "{}"
```

Because the autonomy supervisor only considers `GUIDED` an allowed autonomy
mode, LAND/RTL intentionally make the stack `NOT_READY` when autonomy is off; if
autonomy is active, leaving GUIDED causes FAILSAFE and the velocity bridge is
zeroed while ArduCopter owns the LAND/RTL behavior.

## Backups

All backups remain under `~/uav_ws/backups/`. Use:

```bash
cd ~/uav_ws
./scripts/backup_workspace.sh
```

The v0.3 updater creates a complete `uav_ws_pre_v0.3_<timestamp>.tar.gz` before
overlaying any v0.3 files.
