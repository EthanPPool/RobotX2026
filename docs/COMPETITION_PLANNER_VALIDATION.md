# Validation — Task 1 planner

Base repository: `fee7f822d3f2f8ea53a2d15f60221bc6e798f458`.
Rule reference: RoboNation RobotX handbook dated 23 October 2026, section 3.3.2.

## Passed

- 53 new planner/temporal/survey/ROS-callback tests.
- 56 existing interpretation tests (109 total).
- A ten-buoy synthetic course with both blue orbit roles, four red/green markers and four inactive obstacles.
- A separate course with all eight red/green beacons; no fixed two-gate assumption.
- Correct-side/nearest-abeam passage constraints, rotation invariance, obstacle/hull/uncertainty clearance and bounded search failure.
- Quintic profile speed and acceleration limits, analytic circular references, continuous heading and desired derivatives checked against finite differences.
- Complete mission with measured perfect-reference tracking, plus stationary-boat, skipped-circle, wrong-side, pose-jump and missing-clearance rejection.
- Color frames through temporal interpretation, shared tracking, aerial survey collection and mission planning.
- Attempt-scoped UAV semantics, report-signature binding, local-geometry refinement and stopped/reset reference behavior.
- Controller-preview callback tests with exact three sample offsets, matching datum/frame, fresh scan/pose/map requirements, and no propulsion publication.
- Python syntax compilation, shell syntax checks, package XML parsing, and Git whitespace checks.

## Scope of evidence

The full-mission test uses a perfect reference follower to exercise planning and measured-progress logic. It is not a hydrodynamic simulation, closed-loop PID validation or hardware test.

ROS 2 Kilted/colcon interface generation, real DDS transport/TF, launch execution, actual cameras/LiDAR, field localization, the teammate's trained model and a live professor-controller adapter have not been tested in this environment.

The patch installer preflights a clean checkout with `git apply --check`. A separate packaging check applies it to an isolated copy of the reference files and reruns the full test suite.
