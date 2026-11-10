# USV HSV RGB light detection

The existing robotx_buoy_interpretation package now supplies hsv_color_clusters.
It publishes robotx_coordination_interfaces/BuoyColorDetectionArray on
/perception/usv/color_detections. The existing USV interpreter combines these
image-only boxes with /perception/objects at capture time, then emits physical
observations on /perception/buoys/usv for the existing tracker and planner.
No control algorithm, motor interface, physical distance estimate, or TF is added.

## Independent camera operation and the existing mission handoff

Run the camera producer separately from the existing LiDAR detector:
```bash
ros2 launch robotx_buoy_interpretation hsv_camera.launch.py \
  image_topic:=/ACTUAL_USV_IMAGE_TOPIC
```
This starts only HSV processing. The camera driver must already provide images
and CameraInfo. It does not subscribe to LiDAR objects or start the interpreter,
tracker, planner, or controller. Camera results continue without LiDAR.
The existing boat_perception/perception.launch.py continues owning the independent
LiDAR object detector and /perception/objects. LiDAR results continue without HSV.

Start the existing association/tracking path separately:
```bash
ros2 launch robotx_buoy_interpretation interpretation.launch.py \
  start_hsv:=false config:=/ABSOLUTE/PATH/TO/site_interpretation.yaml
```
For this separate-launch form, configure the interpreter's CameraInfo/detection
topics and calibration policy in site_interpretation.yaml to match the camera
launch. Do not also start a second interpreter/tracker through usv_shadow.
Alternatively, run hsv_camera separately and use the existing usv_shadow launch
with start_hsv:=false to start association, tracking, and mission processing once.
The combined start_hsv:=true launch includes the same standalone camera launch.

The mission logic already accepts this exact contract:

Camera Image/CameraInfo -> HSV -> /perception/usv/color_detections

LiDAR -> /perception/objects

Both -> existing USV association -> /perception/buoys/usv -> existing buoy_tracker
-> /mission/buoys -> existing incremental planner -> timed professor references.

The existing tracker owns temporal beacon classification and persistent buoy
identity. The existing planner owns entry/exit selection, passage side, search,
orbits, and replanning. HSV is an interchangeable upstream color producer;
none of those algorithms is replaced. No camera pixel is treated as map geometry.
The Advanced tier's existing aerial semantic provenance policy is also unchanged.

RGB-only HSV supplies visible colors, not explicit OFF measurements. Existing
temporal classification still requires its documented evidence: repeated blue
can establish steady blue; missing color cannot establish inactive or flashing.
Explicit observed OFF/temporal-model evidence must arrive through the established
camera model contract when needed. This separation does not claim that bright-only
HSV alone measures every mission beacon state.

## Build and launch (Ubuntu 24.04 / ROS 2 Kilted)

Install system dependencies with rosdep on the development machine, including
cv_bridge and python3-opencv, before the existing field build script:
```bash
source /opt/ros/kilted/setup.bash
rosdep install --from-paths coordination_ws/src/robotx_coordination \
  coordination_ws/src/robotx_coordination_interfaces \
  coordination_ws/src/robotx_buoy_interpretation boat_ws/src/boat_interfaces --ignore-src -r -y
bash scripts/build_buoy_interpretation.sh
source coordination_ws/install-buoy-interpretation/setup.bash
ros2 launch robotx_buoy_interpretation interpretation.launch.py \
  start_hsv:=true image_topic:=/ACTUAL_USV_IMAGE_TOPIC \
  camera_info_topic:=/usv/camera/color/camera_info \
  config:=/ABSOLUTE/PATH/TO/site_interpretation.yaml
```

The actual USV Image stream is a required deployment input when enabling HSV.
It was not established in the inspected sensor launch. Supply its real topic;
do not substitute the UAV camera or assume its frame is the boat's optical frame.
The camera driver must publish acquisition stamps and matching CameraInfo.
The site configuration must supply the surveyed datum; geometry and capture-time
TF must already be available as required by the existing interpreter.
Only run one producer on the detection topic and one interpreter/tracker pair.

The existing integrated shadow launch forwards the same options:
```bash
ros2 launch robotx_professor_control usv_shadow.launch.py \
  start_hsv:=true image_topic:=/ACTUAL_USV_IMAGE_TOPIC \
  interpretation_config:=/ABSOLUTE/PATH/TO/site_interpretation.yaml \
  pathing_config:=/ABSOLUTE/PATH/TO/site_pathing.yaml
```
The shadow launch still does not start a propulsion bridge.
start_hsv defaults false for compatibility with deployments using another detector.

## Tuned values

Red hue: 0..15 OR 165..179; saturation >=230; value >=120.
Green hue: 35..85; blue hue: 95..135. All use saturation >=230 and value >=120.
Area >=400 pixels, maximum 40% of image, fill >=10%, solidity >=30%.
Green/blue share the tested red saturation/area settings as starting values;
they have not been independently field validated. Area is in the actual input
resolution, so a different camera/resolution/distance needs retuning.
The kernel defaults 1 to preserve LED rims; a larger odd kernel re-enables cleanup.
These classify colored regions, not buoy identity or illuminated-vs-reflected light.

Override hsv_config with a copy of the installed config/hsv_clusters.yaml.
Each color has its own HSV array, while size/shape limits apply to all colors.
Confidence is threshold-supported pixel fraction after morphology, not a calibrated
probability. Shape filters do not masquerade as confidence. light_state remains
unknown: neither steady nor flashing is inferred from one image.

## Coordinates and calibration

Boxes use centers and sizes in the full rectified image. There is no resizing.
The original acquisition timestamp and optical frame are copied unchanged.
Raw input is rectified using K/D and P; only identity R/reference optical cameras
are supported, matching the existing interpreter. Stereo offsets, nonidentity R,
ROI/binning and unsupported distortion are rejected.
input_rectified:=true requires an already-rectified stream with matching P.
Zero-stamped calibration requires allow_static_calibration:=true, forwarded to
both producer and interpreter. calibration_timeout is likewise shared.
Late-arriving CameraInfo can satisfy a bounded latest pending image.
Stale/future/replayed images are dropped. Empty valid images publish empty arrays.
Missing calibration or unsupported images produce no detections, never fake geometry.

## Validation

```bash
bash scripts/test_buoy_interpretation.sh
```
The HSV workflow also builds the real Kilted interfaces/package and exercises
real generated messages and the ROS producer with synthetic capture-time images.
Synthetic tests do not establish on-water performance or camera calibration.
