# Ground-station USV mission selection

The USV Mission Process panel now supports two mission profiles. The onboard
USV dashboard bridge owns one child ROS launch group, supervises its heartbeat,
and forwards the existing enable/reset/arm actions to the selected mission.

| Profile | Target producer | Enable/reset |
| --- | --- | --- |
| Two-gate mission | `task1_gate_mission` | `/mission/set_enabled`, `/mission/reset` |
| UAV-mapped USV route | `survey_route_mission` | `/boat/route/set_enabled`, `/boat/route/reset` |

These profiles select the **USV target producer**. UAV survey flight/cameras and
UUV following retain their own launch, readiness and authority controls. Loading
the route does not launch or arm those vehicles. Configure them as described in
[UAV_UUV_COORDINATION.md](UAV_UUV_COORDINATION.md). The route mission checks all
reported local obstacles and does not implement Task 1 gate counting. The
legacy `two_gate_follower.py` is untouched.

## Operator flow

1. STOP and DISARM the USV before changing missions.
2. Choose a profile in the USV Mission Process panel and click **LOAD MISSION**.
   The manager confirms software stop, revoked autonomy, and newly observed
   neutral bridge output, stops/reaps the old mission launch group, then records
   the new selection. No new mission is started by LOAD.
3. Click **START MISSION PROCESS**. Each mission starts disabled. Readiness may
   say `MISSION_DISABLED` or `ROUTE_DISABLED` until enabling it.
4. RESET MISSION to reset sequencing/path and begin a new labeled mission log.
5. ENABLE AUTONOMY activates the selected mission, retaining the existing
   disarmed GUIDED preparation. Check its readiness/status, then ARM explicitly.
6. STOP MISSION PROCESS revokes authority and stops the owned mission group.
   DISARM separately before loading another mission.

A new route requires a calibrated `route_config` file on the Jetson. Use an
absolute path passed to `managed_bringup.launch.py`. The defaults intentionally
have no configured route file. The onboard manager rejects route selection if
that file is missing. A present but uncalibrated map datum, missing UAV map,
missing shared USV odometry, stale local perception, invalid map, or no safe path
is reported by the route mission and prevents route ARM readiness. A valid,
fresh route target stream is required; gate visibility is not required for
route arming. Two-gate arming still requires gate perception and a locked target.

The ground station retains its existing mission process endpoint and TCP
transport. `select` adds a whitelisted `profile` field; status exposes the loaded
profile, process state, readiness, reason, and child log path. Unsupported/older
bridges show the selector disabled and retain legacy service start/stop behavior.
The mission label still names the log, not the executable.

## Onboard ownership and failure behavior

`manage_missions` is opt-in (false by default). The dedicated managed bringup
launch enables it, runs the common perception/controller/vehicle/logger stack,
and excludes the fixed Task 1 mission. Do not run it alongside the old stack.
No arbitrary command, launch filename, or configuration path is accepted from
the network. The route configuration path is a local launch parameter.

Start requires a freshly connected, disarmed USV, no existing `/mission/target`
publisher, and exactly one root `target_controller` publishing `/control/cmd_vel`.
The selected process must become the sole expected root mission publisher and
provide its enable service before startup is acknowledged. The common stack
continues running when the selected mission is stopped.

An independent onboard telemetry loop supervises process exit, target/control
publisher conflicts and mission diagnostic heartbeat loss (2 s, after a 3 s
startup grace). It requests software stop and revokes authority, terminates the
owned mission group, and reports a fault. This also runs with no ground station
connected. Existing controller target freshness and vehicle deadman checks are
still in force. A STOP/LOAD action is needed to clear a fault. Logs are saved
under `~/robotx_logs/mission_process_<profile>_<timestamp>.log`.

## Build in a separate checkout

Do not pull/reset the dirty working `~/boat_ws`. Use the new review checkout.
The helper builds a separate overlay without replacing its existing installs,
restarting services or launching nodes. ROS 2 Kilted and existing runtime
prerequisites must already be installed.

On the laptop/WSL, after publishing/importing the branch:

```bash
cd ~/RobotX2026-mission-selection
bash scripts/build_mission_selection.sh groundstation
```

On the Jetson:

```bash
git clone --branch feature/groundstation-mission-selection \
  https://github.com/EthanPPool/RobotX2026.git ~/RobotX2026-mission-selection
cd ~/RobotX2026-mission-selection
bash scripts/build_mission_selection.sh jetson
```

The Jetson build uses `~/boat_ws/install/setup.bash` and, when present, the
validated `~/boat_ws_gate_selection_test/install/setup.bash` as underlays. This
preserves the existing working controller/vehicle implementation, including the
separately tested bridge fix. It overlays current main's interfaces/perception/
mission and the new selector. Set `USV_BASE_SETUP` or `GROUND_STATION_BASE_SETUP`
when the existing base setup file is elsewhere.

## First host validation and launch

Keep propulsion disconnected for the first host validation. The code checks
below have no ROS installation, so actual colcon builds, DDS, process launching,
service timing, and UI operation must be verified on the hosts.

Before changing systemd units, record their current ExecStart, WorkingDirectory
and ROS environment. The old mission service must remain stopped in managed
mode. Replace the existing stack/dashboard launch with the managed launch;
starting both produces duplicated nodes or a TCP port conflict. Do not blindly
replace service files or ROS networking values. Existing vehicle-specific FCU
URL, LiDAR network settings, domain, RMW and sensor configurations must carry
forward unchanged.

With the old stack/dashboard processes stopped, source the Jetson overlay and
launch the managed stack **in their place**:

```bash
source /opt/ros/kilted/setup.bash
source ~/RobotX2026-mission-selection/install-selection-jetson/setup.bash
ros2 launch boat_bringup managed_bringup.launch.py \
  fcu_url:=YOUR_EXISTING_FCU_URL \
  route_config:=/absolute/path/to/calibrated_boat_route_navigation.yaml
```

For gate-only validation, omit `route_config`. Pass the same sensor/MAVROS/ESP32
launch choices used by the existing working stack. The default profile is
Two-gate, stopped; no mission resumes automatically after a bridge restart.

Start the ground station from its new overlay instead of its existing process:

```bash
source /opt/ros/kilted/setup.bash
source ~/RobotX2026-mission-selection/install-selection-groundstation/setup.bash
ros2 run robotx_dashboard dashboard
```

Refresh the browser. Confirm LOAD, START, RESET and STOP acknowledgements while
disarmed, verify exactly one mission publisher, and confirm missing route inputs
are reported. Then verify a sensor-loss or process-loss test leaves authority
off and neutral output. Convert the verified launch/environment to the existing
systemd deployment only after the first host checks. The build helper does not
edit systemd units or enable vehicle authority.

## Local validation

```bash
python3 -m unittest discover -s tests/mission_selection -v
python3 -m unittest discover -s boat_ws/tests -v
```

The selection tests exercise the actual bridge methods and API function using
service/process adapters: disarmed loading, failed stop acknowledgement,
whitelisted profiles, competing publishers, process death, group termination,
mission-specific enable/reset/arm readiness and legacy status transport.
They do not constitute a ROS runtime or water test.
