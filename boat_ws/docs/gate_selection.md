# Gate selection in a crowded buoy field

Previously, perception published its highest-confidence pair and mission tracking accepted each new pair as the current gate. A distant gate could replace the near gate while the boat approached it. Body-frame tracks also accumulated position errors during turns because their old coordinates were reused without compensating for boat motion.

Perception now publishes confirmed geometric hypotheses in `boat_interfaces/msg/GateArray` on `/perception/gates`. Mission control selects the nearest eligible gate within the acquisition bearing, then matches subsequent observations to a fixed acquisition anchor in the MAVROS local frame. A competing pair cannot replace the chosen gate solely because its confidence rises. Completed gates are excluded until mission reset. During PASS_GATE, the existing frozen crossing geometry remains authoritative.

## Candidate filtering

Pairs require two finite, confident buoy detections, a configured width, a permitted skew relative to the boat, and an unobstructed opening. Near-coincident returns are deduplicated. Two consecutive, distinct input timestamps confirm each pair. Defaults consider the nearest 64 eligible buoy tracks and publish at most 64 candidates; increasing these limits increases processing work. Diagnostics report input, plausible-pair and confirmed-pair counts.

This is a geometric selection policy. Unrelated competition buoys can still form a plausible pair. `require_red_green: true` accepts only pairs explicitly classified red and green upstream; the current LiDAR detector publishes unknown colors, so enabling it alone produces no gates. Color recognition or task-specific course information is needed to distinguish geometrically similar gates belonging to different tasks. Width and bearing defaults are tuning values, not a statement about competition rules.

## Measurement timing and loss

The production perception launch enables motion compensation using `/mavros/local_position/pose`. Tracks are transformed from the previous scan's body frame through the fixed local frame into the current scan's body frame before association. Missing pose history clears positional tracks rather than reusing uncompensated coordinates. Pose interpolation is bounded to a 0.15-second endpoint tolerance and a 0.30-second maximum bracket gap.

The production launch sets `publish_misses: 0`: tracks survive internally for reacquisition, but an unobserved predicted buoy is not published with a new measurement timestamp. Standalone detector defaults retain their bench-test behavior; use the production launch for these settings.

Gate arrays retain the original input timestamp when republished. Mission control converts their endpoints using the pose at that timestamp, checks `base_link`, and rejects repeated, old or inconsistent measurements. After 2.5 seconds without a matching measurement, TRACK_GATE publishes a stopped target while retaining its gate identity. A fresh matching observation resumes tracking. PASS_GATE retains its existing pose-based crossing behavior and frozen geometry. Sensor and pose timestamps must use the same ROS clock; large clock offsets or local-frame resets require correcting the source and resetting the mission.

## Topics and parameters

| Topic | Meaning |
| --- | --- |
| `/perception/gates` | All confirmed hypotheses from one measurement; mission input |
| `/perception/gate` | Nearest confirmed pair, retained for compatibility; not necessarily the mission's locked pair |
| `/perception/gate_diagnostics` | JSON candidate counts and workload limits |
| `/mission/diagnostics` | Selected geometry, candidate count, selection reason, lock and completed exclusions |

`boat_perception/config/gate_detector.yaml` controls widths (1.83–3.05 m), maximum skew (55 degrees), confidence, confirmation and workload limits. `boat_mission/config/task1_gate_mission.yaml` controls the acquisition bearing (45 degrees), center association (0.8 m), endpoint association (1.0 m), width association (0.4 m), and measurement timeout (2.5 seconds). Set association tolerances from measured localization and detection errors; overly loose tolerances allow similar nearby gates to alias. If the locked gate is permanently unavailable, stop and reset the mission rather than silently selecting another gate.

## Build and checks

Use a separate checkout for initial testing: the working Jetson source contains local changes and should not be overwritten by a pull or patch. From the repository root:

```bash
python3 -m unittest discover -s boat_ws/tests -v
source /opt/ros/kilted/setup.bash
cd boat_ws
colcon build --symlink-install --packages-up-to boat_mission boat_perception
source install/setup.bash
ros2 interface show boat_interfaces/msg/GateArray
```

The interface, perception and mission packages must be rebuilt together. The unit tests and CI use message/clock adapters for real mission and detector callbacks and pure geometry tests; they do not initialize ROS or validate DDS, ROS interface generation, hardware, or actuator behavior.

Before physical trials, run sensor injection in an isolated ROS domain with propulsion disconnected. Verify nearest-gate acquisition with a more confident far pair, distractor pairs on both sides, varying headings, measurement loss/reacquisition, and two correctly ordered truth crossings. Inspect the selected mission geometry as well as the compatibility gate topic. Confirm stale TRACK_GATE targets stop, PASS_GATE follows its committed corridor, and mission completion produces neutral output and revokes autonomy.

The older perception simulator snapshots only standalone node files and validates four original interfaces. It must be updated to copy `gate_candidates.py`, `pose_history.py` and `gate_selection.py`, add their source package directories to child-process PYTHONPATH, validate `GateArray`, and observe `/perception/gates`. A success from that older harness without these updates does not validate the new candidate path. Its prior bridge transition fix remains a separate change; this branch does not replace the vehicle bridge.
