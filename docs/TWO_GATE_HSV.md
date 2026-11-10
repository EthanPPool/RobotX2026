# HSV colors for the existing two-gate mission

This path uses task1_gate_mission and target_controller, not the incremental
competition planner. No flashing/steady classification, surveyed map datum,
entry/exit orbits, BuoyTrackArray, or professor reference generator is required.
The existing two-gate counting, gate identity, passage and control code is unchanged.

## Build and run

Use Ubuntu 24.04 / ROS 2 Kilted in a clean shell, with the existing boat hardware
dependencies installed. Install the new image dependencies (cv_bridge/OpenCV) and
the existing package dependencies using rosdep during development:

```bash
source /opt/ros/kilted/setup.bash
rosdep install --from-paths boat_ws/src \
  coordination_ws/src/robotx_coordination \
  coordination_ws/src/robotx_coordination_interfaces \
  coordination_ws/src/robotx_buoy_interpretation --ignore-src -r -y
bash scripts/build_two_gate_hsv.sh
source install-two-gate-hsv/setup.bash
ros2 launch boat_bringup task1_bringup.launch.py \
  use_hsv_colors:=true image_topic:=/ACTUAL_USV_IMAGE_TOPIC
```

The image source remains the actual camera driver, which must already publish
Image and matching CameraInfo. Supply its real topic. Existing camera optical TF
to base_link and calibration are required; no extrinsics are invented.
camera_info_topic defaults to /usv/camera/color/camera_info.
Use input_rectified:=true only for a verified rectified stream; otherwise HSV
rectifies the raw image using matching capture-time calibration.
Zero-stamped CameraInfo requires allow_static_calibration:=true explicitly.

If hsv_camera.launch.py is already running, add start_hsv_camera:=false to avoid
a duplicate HSV publisher. If LiDAR drivers already run, retain the existing
start_sensors/start_perception flags as appropriate; do not duplicate drivers.
use_hsv_colors defaults false to retain the existing geometry-only deployment.
When enabled, the gate detector requires an observed red/green pair. No automatic
fallback to uncolored pairs occurs if the camera stops.

## Exact handoff

* Camera -> hsv_color_clusters -> /perception/usv/color_detections.
* Independent LiDAR buoy_detector_multi -> /perception/objects.
* two_gate_colors -> /perception/colored_objects (DetectedObjectArray).
* Existing gate_detector reads colored_objects with require_red_green=true.
* Existing /perception/gates -> task1_gate_mission -> /mission/target
  -> target_controller -> the existing vehicle bridge.

Only the raw LiDAR detector writes /perception/objects. The adapter copies that
message, preserving its IDs, positions, sizes, base_link frame and measurement
stamp, and attaches only uniquely associated red/green enums. Blue remains an
independent HSV image detection; DetectedObject has no blue enum and blue cannot
qualify a red/green gate. No pixel displacement becomes a physical position.

The adapter projects each measured buoy's 3-D bounding cuboid into the calibrated
image. This supports a light on the buoy away from its centroid, without inventing
a lamp height. A center must lie inside the projected bounds plus bbox_padding.
Multiple possible objects or multiple color candidates withhold the assignment.
Validate that the real LiDAR bounds encompass the lamp; adjust measured geometry
or calibrated association limits if they do not.

Image and geometry captures must differ by at most max_sync (0.1 s). For distinct
capture times, the adapter uses the existing planar PoseHistory from
/mavros/local_position/pose to account for boat movement, then applies the optical
extrinsic at image capture. No global surveyed datum is needed for this two-gate
path. Without the required pose/TF/calibration, no color is guessed.

Camera and LiDAR processing run independently. Missing camera data causes the
adapter to publish unknown-colored copies after a bounded wait, leaving all raw
LiDAR measurements available. Missing LiDAR data leaves camera detections running,
but produces no positioned colored objects. Gate confirmation uses distinct
original scan timestamps; delayed camera data never refreshes an old scan.

## Configuration and tests

The tested red thresholds and the same initial green/blue S/V/area thresholds
remain in config/hsv_clusters.yaml. Association limits are in
config/two_gate_colors.yaml in robotx_buoy_interpretation. To run the adapter alone
with a site-specific copy, use two_gate_colors.launch.py start_hsv_camera:=false
color_config:=/ABSOLUTE/PATH/TO/site_two_gate_colors.yaml, and run the existing
gate_detector on /perception/colored_objects with require_red_green:=true.

Offline tests pass HSV pixels through the real association, gate detector and
two-gate mission callbacks. The Kilted workflow also tests real generated ROS
messages and DDS delivery from HSV through the adapter into gate_detector, using
a synthetic calibrated transform. These are not camera/extrinsic hardware tests.
