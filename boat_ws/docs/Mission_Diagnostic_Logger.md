# Mission Diagnostic Logger

The mission diagnostic logger runs on the USV Jetson. The ground-station
dashboard only prepares a labeled logging session and displays its status, so
loss of the Beeptop TCP connection does not interrupt an active log.

## Lifecycle

1. Enter an optional test label and press **RESET MISSION**.
2. The existing mission reset runs and the logger enters `PENDING`.
3. While pending, the logger keeps the newest five seconds of 20 Hz snapshots
   in memory. It does not create a file or consume a mission number.
4. The first `armed: false -> true` transition allocates the next mission
   number, creates the CSV and metadata JSON, writes the pre-arm buffer, and
   enters `RECORDING`.
5. Mission completion, software E-stop, and MAVROS loss are recorded but do
   not stop the recorder.
6. On disarm, the logger enters `CLOSING` and records for five more seconds. A
   re-arm during that interval cancels the close and continues the same file.
7. Five seconds continuously disarmed closes the log as `DISARMED_5S`.
8. Pressing **RESET MISSION** during an active log closes it immediately as
   `RESET_MISSION`, then creates a new pending session.

Logs default to:

```text
~/robotx_logs/mission_004_2026-09-12_231845_precal.csv
~/robotx_logs/mission_004_2026-09-12_231845_precal_metadata.json
```

The files stay on the Jetson. Mission IDs persist because the logger scans the
existing log directory before allocating the next number.

## Recorded data

Every CSV row is a synchronized cached-value snapshot. Each source includes
its source timestamp, value age, and a `*_new` flag. Missing values are blank,
never synthetic zeroes.

The logger records:

- MAVROS state, mode, arm state, local pose, attitude, local/body velocity,
  IMU, magnetometer, GPS, estimator health, battery, timesync, and RC outputs;
- raw left/right gate posts, geometric midpoint, confidence, width, and
  detector-new versus detector-held status;
- mission state, gate counters, passage evidence, target-in-body coordinates,
  heading error, forward-angle decision, follower request, and an explicit
  `controller_reason`;
- bridge input, every authorization input, explicit `bridge_reason`, bridge
  output, operator command, and deadman state;
- state-change events including arm, disarm, re-arm, mode, MAVROS connection,
  mission phase, software E-stop reason changes, reset, and file close.

The sidecar metadata snapshots actual runtime follower/bridge parameters and
requests relevant ArduPilot AHRS, compass, accelerometer, and EKF parameters.

The current `two_gate_follower` steers from the live gate midpoint in the body
frame. For diagnosis, it also transforms each live gate post and midpoint into
the current local map frame; these TRACK coordinates expose yaw/transform
errors without changing control behavior. The controller does not yet freeze a
PASS gate line or target. Frozen PASS columns are deliberately blank so
downstream analysis cannot confuse missing geometry with `(0, 0)`. They are
already present for a future controller that publishes those values.

## Build and launch on the Jetson

```bash
cd ~/boat_ws
source /opt/ros/kilted/setup.bash
colcon build --symlink-install --packages-up-to \
  boat_interfaces boat_control boat_vehicle boat_dashboard_bridge boat_bringup
source install/setup.bash
ros2 launch boat_bringup robotx_task1.launch.py
```

The unified launch starts `/mission_logger` by default. It can be disabled for
diagnosis with `start_mission_logger:=false`.

Useful live checks:

```bash
ros2 node info /mission_logger
ros2 topic echo /mission_logger/status --once
ros2 topic echo /control/diagnostics --once
ros2 topic echo /vehicle/control_diagnostics --once
ros2 service type /mission_logger/reset
```

## Bench verification

Keep the USV physically safe and disarmed. Press **RESET MISSION**, then verify
the dashboard shows `PENDING` and a growing buffered-row count. Confirm that no
new CSV exists yet. Only perform the arm/disarm lifecycle test under the team's
normal physical E-stop and propulsion-safety procedure.
