# RobotX Task 1 Runtime Interfaces

Status: canonical split architecture after Task 1 controller refactor  
Canonical launch: `ros2 launch boat_bringup task1_bringup.launch.py`

## Source-of-truth architecture

```text
Unitree L2
  -> buoy_detector_multi
  -> /perception/objects
  -> gate_detector
  -> /perception/gate
  -> task1_gate_mission
       -> /mission/target
       -> /mission/state
       -> /mission/diagnostics
  -> target_controller
       -> /control/cmd_vel
       -> /control/diagnostics
  -> mavros_command_bridge
       -> /mavros/setpoint_velocity/cmd_vel
  -> MAVROS / ArduRover
```

## Ownership rules

| Interface | Sole writer | Role |
|---|---|---|
| `/perception/objects` | `buoy_detector_multi` | Production buoy observations |
| `/perception/gate` | `gate_detector` | Production gate estimate |
| `/mission/target` | `task1_gate_mission` | Body-frame navigation objective |
| `/mission/state` | `task1_gate_mission` | Mission state contract |
| `/mission/diagnostics` | `task1_gate_mission` | Gate/pass geometry diagnostics |
| `/control/cmd_vel` | `target_controller` | Desired body velocity |
| `/control/diagnostics` | `target_controller` | Guidance diagnostics |
| `/mavros/setpoint_velocity/cmd_vel` | `mavros_command_bridge` | Authorized MAVROS velocity setpoint |

No other Task 1 node may publish any of the interfaces above.

## Mission state contract

The vehicle bridge intentionally consumes three mission-state prefixes:

- `WAIT_GATE_2`: Gate 1 is complete; enter intentional HOLD while waiting for Gate 2.
- `TRACK_GATE_2`: Gate 2 has been acquired; request GUIDED and remain neutral until GUIDED is confirmed.
- `MISSION_COMPLETE`: revoke autonomous velocity authority and request the configured mission-complete mode (`LOITER`).

Other mission states are informational to the bridge.

## Mission services

- `/mission/reset` (`std_srvs/srv/Trigger`)
- `/mission/set_enabled` (`std_srvs/srv/SetBool`)

The old `/control/reset_mission` interface is removed because reset is a mission-layer operation.

## Current Task 1 passage behavior

`task1_gate_mission` preserves the latest fixed-frame passage implementation formerly embedded in `two_gate_follower`:

1. A fresh gate measurement is transformed from `base_link` into the fixed MAVROS local frame.
2. Repeated sample-and-hold detector messages with the same source timestamp are ignored as new geometric evidence.
3. The remembered map-frame midpoint is tracked while approaching the gate.
4. At `pass_commit_distance`, and only while local pose is fresh and the vehicle is ARMED + GUIDED, both gate posts are frozen in the local frame.
5. A gate tangent, traversal normal, usable crossing corridor, and fixed target beyond the gate are computed.
6. Passage is confirmed by an actual sign change through the saved gate plane.
7. The interpolated crossing location must lie inside the usable corridor.
8. The boat must reach `pass_clear_distance` beyond the saved gate plane before the gate count increments.

Loss of perception alone never counts as a passage.

## QoS

`/mavros/local_position/pose` is consumed using `qos_profile_sensor_data` so its best-effort MAVROS publisher is compatible with the mission subscriber.

## Runtime source-of-truth rule

Use one runtime workspace. Do not source multiple RobotX `install/setup.bash` overlays in the same shell. After this refactor, a clean rebuild should install only the current Task 1 mission/control executables; legacy executables must not remain in an old `install/` tree.
