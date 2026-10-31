# Task 1 competition planner and timed references

Base: RobotX2026 main `fee7f822d3f2f8ea53a2d15f60221bc6e798f458`, with the merged independent-USV/UAV-assisted interpretation package.

Rules verified against RoboNation's **23 October 2026 handbook**, section 3.3.2, printed pages 71–74:
https://robonation.org/app/uploads/sites/2/2026/10/RobotX-2026_Team-Handbook-20261023.pdf

## What is implemented

- The Jetson consumes the common `/mission/buoys` input from either interpretation mode.
- Core tier works with the USV color model plus local LiDAR geometry. Advanced tier requires an aerial-classified survey and the integration's acknowledgement that the survey was actually reported to RoboCommand.
- Confirmed labels are collected across multiple images, then retained for a single attempt. The UAV may depart after the Advanced survey; local LiDAR clearance and shared-frame pose must remain fresh.
- The planner finds an approach, a complete clockwise entry orbit, a constrained transit, and a complete counterclockwise exit orbit.
- Passage checks use signed port/starboard geometry and the nearest buoy on that side at abeam. Black/off markers remain obstacles and may be passed on either side.
- A bounded visibility-graph search checks hull radius, clearance, buoy physical radius, uncertainty and the configured convex course boundary.
- The current conservative routing policy visits every recognized red/green marker on its correct side at least once. This is an implementation objective, not a claim that the handbook mandates an individual visit to every marker. It can reject tightly arranged layouts where a less restrictive task strategy might score partial credit.
- Timed references contain desired map position, continuous heading, velocity and acceleration, including yaw derivatives, at exactly `t`, `t+dt`, `t+2dt`.
- Measured motion determines progress. Time alone cannot finish an orbit, advance a stationary boat through the route, or certify that the required passages occurred.

This package implements Core and Advanced planning. Disruptive-tier live semantic retasking is not implemented; selecting that tier is rejected. A confirmed mid-attempt route-label change or a newly blocked route invalidates the reference and requires a new plan/attempt. It is never silently treated as a successful passage.

## Data flow

| Stage | Input | Output |
| --- | --- | --- |
| Color assignment model | USV/UAV camera | Rectified-image color detections |
| Interpretation and tracking | Color detections, camera pose/calibration, local LiDAR positions | `/mission/buoys` |
| Survey collection | Timestamped buoy tracks and selected estimate provenance | Attempt-scoped classified map |
| Safe Passage planner | Classified map, measured pose, course boundary | `/mission/task1/path` |
| Trajectory generator | Collision-checked route and nominal motion limits | `/mission/trajectory_preview` |
| Professor-controller adapter, subsequent hookup | Timed references and measured model state | Force/thrust and ESC commands |

The controller handoff is implemented as a typed ROS message stream. A live consumer translating that stream into the professor's model coordinates and evaluating the supplied force law is **not** included. No force, PWM, velocity setpoint, arming, or flight-mode command is published by this package. Existing PoR control, boot services, and the professor's notebook are unchanged.

## Beacon interpretation

`BuoyTrack` now includes `color_source` and `position_source` so an Advanced aerial classification cannot be claimed merely because a UAV once contributed geometry.

If `light_state` is already supplied by the teammate's temporal model, that value is used directly. Otherwise the tracker's `BeaconHistory` uses high-confidence explicit color observations:

- Two alternating one-second active/black cycles establish flashing; transition intervals from 0.5 to 1.5 seconds allow sample timing uncertainty.
- At least 2.5 seconds of contiguous blue establishes steady blue.
- At least 2.5 seconds of contiguous black establishes inactive.
- Missing/unknown detections do not establish OFF. Gaps over 0.75 seconds reset history; inference must preserve capture timestamps.
- The active color is retained while a classified flashing beacon is temporarily dark.

These thresholds are engineering defaults, requiring validation against the teammate model's frame rate and label behavior. The model must detect/locate the buoy through its off phase and explicitly label its inactive beacon black; if it only reports visible bright pixels, provide a dedicated temporal beacon detector instead. Single-frame blue never identifies entry vs exit by itself.

## Survey lifetime and mission lifecycle

The planner starts in `IDLE`. A survey collector gathers confirmed semantic observations for up to `survey_semantic_age` (default 90 seconds), joining them with updated geometry. This window is independent of the live tracker's short freshness timeout.

At start, the current survey must have the configured number of distinct physical buoys (10 for this task), exactly one flashing-blue entry and one steady-blue exit, valid states for the remaining markers, and confidence above threshold. Unknown or conflicting labels do not count. The complete-map requirement is deliberate: this version plans from a survey; it does not implement an autonomous reconnaissance/search maneuver to discover unseen buoys.

On a successful start, semantics are pinned for that attempt. A later unknown/off observation does not erase the accepted route. Updated LiDAR geometry still checks whether the nominal route remains clear. Stop/reset clears survey history and acknowledgement, preventing pre-reset classification timestamps from silently becoming a new randomized attempt's survey. After completion, reset is required before a new start.

Phases:

1. `APPROACH_ENTRY`
2. `CIRCLE_ENTRY`
3. `TRANSIT`
4. `CIRCLE_EXIT`
5. `COMPLETE`

Full circle progress is measured from bounded, direction-sensitive movement around the marker. Transit records actual abeam passages using measured vessel heading, rejects a wrong-side passage, and checks all required passage evidence before completion.

## Motion and clearance

The two-thruster vessel advances along collision-checked straight segments and rotates in place at corners. Complete orbits are analytic circular trajectories. Quintic time profiles bring velocity and acceleration to zero at each primitive boundary, so the time-domain reference is C2 even though the geometric route has corners. This is deliberately slower than a continuous-curvature cruising planner, and avoids imposing sideways translation on a two-parallel-thruster hull.

Heading is unwrapped continuously; there is no `+pi` to `-pi` command jump. Cruise speed, linear acceleration, yaw rate and yaw acceleration are independently bounded. Orbit timing includes centripetal acceleration.

The nominal route is not evidence of currently clear water. A valid controller preview also requires fresh pose, a current map stream, and a current `/scan` transformed into the shared map at scan acquisition time. The swept hull disk must fit within measured scan coverage over the nominal braking interval plus scan latency and the preview duration. NaN, negative, invalid finite ranges, missing TF, unknown coverage or a close obstacle invalidates the preview. Positive infinite scan ranges are interpreted as no obstacle up to that sensor's declared maximum range.

The scan checker currently supports positive angular increments up to 2 degrees and a near-planar LiDAR frame. These assumptions must match the actual scan generator. A course-boundary polygon is not a substitute for scan coverage.

`hull_radius` is the circumscribed radius of the complete vessel, including protrusions, not half its beam. Default 0.9 m is a starting value to replace with a measurement. Clearance, source uncertainty and local-scan calibration must be measured for the real setup. Tracking lag pauses the reference clock; excessive error invalidates the reference and calls for replanning.

## ROS interfaces

| Interface | Type | Meaning |
| --- | --- | --- |
| `/mission/buoys` | `robotx_coordination_interfaces/BuoyTrackArray` | Shared classified tracks |
| `/usv/coordination/odometry` | `nav_msgs/Odometry` | USV pose in the same surveyed ENU map |
| `/scan` | `sensor_msgs/LaserScan` | Fresh local obstacle/coverage observations |
| `/mission/task1/path` | `nav_msgs/Path` | Nominal geometric route for inspection |
| `/mission/trajectory_preview` | `robotx_coordination_interfaces/TrajectoryPreview` | Three timed desired samples, validity and run identity |
| `/mission/task1/status` | `std_msgs/String` JSON | Phase, validity, reason and measured progress |
| `/mission/task1/events` | `std_msgs/String` JSON | Planning and phase events for an OCS/report adapter |
| `/mission/task1/start` | `std_srvs/Trigger` | Validate/latch survey, plan and start reference progression |
| `/mission/task1/stop` | `std_srvs/Trigger` | Invalidate references and clear survey collection |
| `/mission/task1/reset` | `std_srvs/Trigger` | Clear attempt, survey and measured progress |
| `/mission/task1/survey_reported` | `std_srvs/SetBool` | OCS acknowledgement of an actual successful survey report |

The package does not directly send RoboCommand reports. The OCS integration must submit the task/marker reports and call `survey_reported` after successful reporting. The acknowledgement is bound to the classified survey signature, is consumed on start, and is cleared on stop/reset. Do not use it to claim a report that was not sent.

## Controller reference contract

`TrajectoryPreview.header.stamp` is the reference time `t`; `header.frame_id` and `reference_id` identify the shared surveyed ENU map. Exactly three `TrajectorySample` entries have offsets `0`, `dt`, `2*dt`. Their fields are:

- `x`, `y`: meters in the map.
- `yaw`: continuous radians in the ENU convention, positive counterclockwise.
- `vx`, `vy`: desired map velocities, m/s.
- `ax`, `ay`: desired map accelerations, m/s².
- `yaw_rate`, `yaw_acceleration`: rad/s and rad/s².

Consumers must reject `valid=false`, future/expired previews, unexpected frames/datums, nonfinite fields and old `run_id` references. They must not continue an old route after stop/reset or a new attempt. No future actual measurement is invented; actual derivatives required by the professor's law must come from the state estimator or measurement history.

The notebook's meaning of longitudinal `x` and `q3`, force sign conventions, nominal bias/integration state and actuator calibration still need an explicit adapter contract. These references preserve full planar information so that connection can be implemented without rewriting the professor's control law or silently guessing model coordinates.

## Build and run

Rebuild the changed coordination messages, interpretation package and planner on all participating hosts using the same definitions. Do not mix the previous generated `BuoyTrack` type with the extended one.

This patch also carries the prior offline build correction from `robotx@localhost` to `robotx@example.com` in the two shared package manifests and coordination setup metadata, so a clean checkout does not reintroduce that known maintainer-address build error.

From the real repository, with ROS Kilted and existing dependencies available:

```bash
bash scripts/test_competition_planner.sh
bash scripts/build_competition_planner.sh
source coordination_ws/install-competition-planner/setup.bash
```

Copy `coordination_ws/src/robotx_competition_planner/config/task1.yaml` to a site configuration. Set the actual shared datum, measured hull dimensions/limits, topic names and an ordered convex course boundary in shared map meters. No fabricated boundary or datum is enabled by default.

Run the interpretation stack in `source_mode:=usv_only` for Core or `source_mode:=assisted` for Advanced. Select the matching `tier` in the planner configuration. Then:

```bash
ros2 launch robotx_competition_planner task1_planner.launch.py \
  config:=/absolute/path/to/site_task1.yaml
```

Inspect the map and survey states; the existing shared-pose publisher must be configured for the USV and the same `robotx_map` datum. Independent MAVROS local origins are not interchangeable. The scan must have capture-time TF into that map.

For Advanced, have the OCS report the actual accepted survey, then acknowledge:

```bash
ros2 service call /mission/task1/survey_reported std_srvs/srv/SetBool '{data: true}'
ros2 service call /mission/task1/start std_srvs/srv/Trigger '{}'
```

Core uses only the start call. Stop/reset are available through their Trigger services. Successful planning does not arm or move the vehicle; inspect `/mission/task1/path` and `/mission/trajectory_preview` before integrating the controller consumer.

## Verification

Tests use a ten-buoy synthetic field, rotated geometry, a full measured-reference mission, camera-label-to-survey integration, ROS callback transport stubs, and missing/conflicting/stale-input cases. A perfect-reference tracking test validates planner/state-machine logic; it is not a hydrodynamic vessel simulation, PID validation or proof of real-world tracking.

The authoring environment has no ROS Kilted installation, so colcon generation, actual DDS transport, launch execution, field sensor calibration, the teammate's model, the professor's controller and hardware remain untested here.
