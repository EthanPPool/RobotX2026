# Incremental Safe Passage planner

The new `incremental_planner` executable removes complete-survey gating. It learns
from sensor-derived map markers, seeks observation viewpoints, approaches/circles
a confirmed entry, plans short passages while the exit may still be unknown, and
replans when new information invalidates its current route. Entry and measured
passage progress survive local replans. Unknown closest-side passages are withheld
until classified; unknown geometry is always an obstacle. The engine never consumes
scene counts or hidden colors. Static attempt labels are retained until conflicting
evidence; dynamic Disruptive-tier retasking is unsupported.

Launch, instead of the original snapshot planner:

```bash
ros2 launch robotx_competition_planner task1_incremental.launch.py \
  config:=/absolute/path/to/configured-task1.yaml
```

Use the existing shared-map, fresh odometry and capture-time planar scan interfaces.
The same reference topic and start/stop/reset services are used; run one publisher
at a time. `extra_planning_margin` defaults to 0.5 m; `observation_distance` defaults
to 8 m and must be set for actual camera/model performance. Camera colors may remain
unknown; they are not grounds to treat a buoy as inactive. Geometry inside only the
extra planning margin permits an outward retreat with physical clearance preserved.

Advanced inputs additionally consume `/perception/buoys/uav`. Direct projected
aerial evidence enriches semantic provenance while precise local positions remain.
Run the USV interpretation path geometry-only in Advanced (`detections_topic`
unconnected), avoiding semantic conflicts between inactive side and active top
beacons. The real adapter never fabricates reporting success: the OCS calls
`/mission/task1/survey_reported` after its successful actual report; identified entry
and exit and a fresh map are required to record acknowledgment. Browser simulation
uses explicit simulated reporting. This is not a full competition referee.

A fresh local map and pose and a scan covering the braking horizon are required
for valid references. The module outputs the current local path and t/t+dt/t+2dt
position, velocity, acceleration and yaw references. It does not send motor output.
Reference transitions, braking and model-based control require testing with the
actual force/PID consumer. Default placeholder boundary/datum are not deployment
configuration. No live ROS/DDS or hardware testing is claimed; transport is tested
using message stubs.
