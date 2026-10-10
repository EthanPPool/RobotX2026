# UAV mapping, USV maze navigation and UUV following

This is the first implementation of the flow described in **Virtual USV Simulation**: the UAV takes downward images of buoys, sends georeferenced photographs and a buoy map to the USV Jetson, the USV plans and follows a route through observed water while checking onboard object detections, and the UUV follows the USV's recorded trail underwater.

Developed against RobotX2026 main commit `ffa6047867c3725bff58803955ab1385d6decd51`, Ubuntu 24.04 / ROS 2 Kilted. The existing Task 1 mission, boat target controller, boat command bridge, and legacy uploaded `two_gate_follower.py` are preserved. UAV camera selection and two missing Python entry-point registrations are updated for the paired OAK-D deployment.

## Packages

| Package | Location | Responsibility |
|---|---|---|
| `uav_mapping` | `uav_ws/src` | Second OAK-D RGB camera, calibrated image-to-water projection, buoy-map memory, georeferenced JPEG publication |
| `boat_route_navigation` | `boat_ws/src` | Receive/save UAV photos, bounded A* route planning, onboard obstacle checks, body-relative `NavigationTarget` publication |
| `uuv_follow` | `uuv_ws/src` | Recorded-trail following at configured depth, source freshness checks, world-velocity guard |
| `robotx_coordination` | `coordination_ws/src` | Shared geometry/planning/following algorithms; GPS + attitude + downward range adapter for UAV/USV |
| `robotx_coordination_interfaces` | `coordination_ws/src` | Typed `BuoyMap`, `GeoImage`, buoy and observation-footprint messages |
| `robotx_coordination_sim` | `coordination_ws/src` | Isolated ROS integration fixture with synthetic images, world poses and vehicle state |

The route mission is an alternative producer on `/mission/target`; run it with the existing `target_controller` and `mavros_command_bridge`. Do not run `task1_gate_mission` at the same time. The perception stack, including gate detection, can keep running. The route mission uses **all** reported local obstacles; it does not steer exclusively toward the selected gate. It does not implement Task 1 gate counting.

## Deployment: one Jetson and one Raspberry Pi

| Computer | Processes | Camera / vehicle connections |
|---|---|---|
| Shared USV/UUV Jetson | Existing USV stack, route planner, photo recorder, existing UUV stack, trail follower and velocity guard | USV autopilot and sensors; separate UUV autopilot and UUV sensors/tether |
| UAV Raspberry Pi | Existing UAV flight stack, first OAK-D RGB driver, **second downward OAK-D** driver, shared pose adapter and buoy mapper | UAV autopilot; two OAK-D devices selected by distinct DeviceID/MXID |

USV poses travel locally from USV nodes to UUV nodes on the same Jetson. There is no second UUV computer or USV-to-UUV ROS network hop. The UUV still requires its own autopilot connection and measured underwater pose source; sharing a Jetson does not supply underwater position. Pi-to-Jetson DDS transports `/uav/mapping/buoys` and `/uav/mapping/georeferenced_image`. Raw downward frames and UAV capture-time TF are consumed on the Pi.

Clone this GitHub branch on both hosts. ROS 2 Kilted must be installed and rosdep initialized. Build the appropriate profile:

```bash
# Shared Jetson: source the existing boat sensor/bringup overlay first as needed.
bash scripts/build_coordination.sh jetson
source coordination_ws/install-jetson/setup.bash

# Separate Raspberry Pi (64-bit Ubuntu 24.04 / ROS 2 Kilted):
bash scripts/build_coordination.sh uav-pi
source coordination_ws/install-uav-pi/setup.bash

# Isolated software fixture on a development Kilted host:
bash scripts/build_coordination.sh sim
source coordination_ws/install-sim/setup.bash
```

The three profiles use separate build/install/log directories. The Jetson profile includes the existing UUV bringup stack; it does not rebuild the USV LiDAR/camera SDK stack. The Pi profile includes UAV flight/bringup and mapping, with no boat packages. The simulation profile includes the existing boat controller/bridge and new adapters, with no real camera or vehicle processes. Install any existing UUV detector/model dependencies according to `uuv_ws/models/README.md`.

On the **Pi**, install the pinned DepthAI v3 driver in a ROS-compatible Python environment before building. Avoid replacing the system NumPy used by cv_bridge:

```bash
python3 -m venv --system-site-packages ~/robotx-oak-env
source ~/robotx-oak-env/bin/activate
python3 -m pip install colcon-common-extensions
python3 -m pip install --no-deps -r scripts/requirements_uav_oak.txt
python3 -c 'import depthai, numpy, cv_bridge; print(depthai.__version__)'
```

Build and run the Pi overlay with this environment activated; the venv colcon entry point ensures ROS node scripts use the Python environment containing DepthAI. DepthAI itself runs the camera pipeline on each OAK-D. The driver uses v3 frame transformation metadata for calibration and has been API-tested against 3.11.0; ARM64 installation and two-device streaming remain host checks. Set up Luxonis USB permissions on the Pi, use distinct IDs, and verify simultaneous camera power/bandwidth on the actual Pi.

Use the same `ROS_DOMAIN_ID` (for example `26`) and RMW implementation on both computers, permit DDS traffic on their link, and synchronize host clocks. Use the supplied `coordinated_autonomy.launch.py` for the UAV flight stack when joining the boat's domain:

```bash
ros2 launch uav_bringup coordinated_autonomy.launch.py \
  fcu_url:=/dev/ttyAMA0:57600 target_system:=2
```

This launch puts UAV nodes under `/uav` and remaps their absolute telemetry, mission, control, vehicle services and dashboard subscriptions. UAV MAVROS uses `/uav/mavros/...`; existing USV `/mavros/...` and UUV `/uuv/mavros/...` remain distinct. UAV TF uses `/uav/tf` and `/uav/tf_static` so local MAVROS frames cannot collide with USV frames. Use it in place of the standalone UAV `real_autonomy.launch.py`, never alongside it. Match all MAVLink target system IDs, serial devices and UDP ports to the real autopilots. Startup preserves the existing disabled autonomy controls.

Do not source multiple copies of a package from different overlays in the same process.

## Local algorithm verification

The recorded 100-trial synthetic results are in `docs/validation/coordination_100_trials.json`; all 100 completed and passed. The minimum simulated buoy-edge clearance was 1.260 m and the maximum goal error was 0.350 m. These results exercise the algorithms/controller shim, not the new camera hardware driver. GitHub CI runs the unit/adapter tests and repeats the 100-trial harness, uploading its logs as an artifact.

To repeat the tests independently of ROS:

```bash
cd ~/RobotX2026
python3 -m venv /tmp/robotx-coordination-tests
source /tmp/robotx-coordination-tests/bin/activate
python3 -m pip install numpy opencv-python-headless pytest -r scripts/requirements_uav_oak.txt
PYTHONPATH="$PWD/coordination_ws/src/robotx_coordination" \
  python3 -m pytest tests/coordination -q
python3 tests/coordination/run_coordinated.py --runs 100 \
  --output coordination_test_results
```

The repeated harness renders five colored buoys into a downward-camera image, detects/projects them, plans a route, executes the repository's **actual** controller and bridge Python methods through a small ROS API shim, and integrates concurrent USV/UUV motion. It checks the executed segments, goal arrival, LOITER completion, revoked USV authority, and zero output on selected faults. The trials vary buoy positions, starting lateral offset, and initial heading. They are not trials of measured camera data, DDS, MAVROS serialization, Gazebo, ArduPilot SITL, current/waves, or underwater localization.

## Full ROS software fixture

Run this in a new terminal on the Kilted host:

```bash
cd ~/RobotX2026
source /opt/ros/kilted/setup.bash
source coordination_ws/install-sim/setup.bash
export ROS_DOMAIN_ID=126
ros2 launch robotx_coordination_sim coordinated_sim.launch.py
```

All fixture topics/services are under `/sim`; TF is also remapped. No MAVROS hardware process, MAVLink connection, real camera, or vehicle service is launched. Synthetic arming/mode state is only published inside the fixture. The fixture enables the route and simulated bridge through `/sim` services after initialization. It uses a stationary aerial camera overlooking the course; it does not simulate UAV flight dynamics.

In a second terminal with the same overlay and `ROS_DOMAIN_ID=126`:

```bash
ros2 topic echo /sim/test/status
ros2 topic echo /sim/boat/route/status
ros2 topic echo /sim/uuv/follow/status
```

At 0.12 m/s, the approximately 30 m route takes about five minutes in real time. Expected completion: `MISSION_COMPLETE`, zero boat velocity, `LOITER`, and zero UUV output. Use a new simulation launch for a clean run.

Fault injection examples:

```bash
ros2 service call /sim/test/drop_camera std_srvs/srv/SetBool '{data: true}'
ros2 service call /sim/test/drop_local_perception std_srvs/srv/SetBool '{data: true}'
ros2 service call /sim/test/drop_usv_pose std_srvs/srv/SetBool '{data: true}'
ros2 service call /sim/test/drop_uuv_pose std_srvs/srv/SetBool '{data: true}'
ros2 service call /sim/test/drop_uuv_safety std_srvs/srv/SetBool '{data: true}'
```

Set `data: false` to restore a source. Camera loss stops the USV after `map_timeout` (3 s); local perception or measured USV position loss stops its route within 0.5 s. Missing USV/UUV pose or UUV safety stops UUV commands within the configured freshness limits. A USV trail discontinuity/gap latches the UUV follower stopped; disable the mission and call `/sim/uuv/follow/reset` before a new trial. No stop command is substituted for missing source data in the fixture; these services actually stop publishing the source.

The full ROS launch was supplied but **not executed in the development environment**, which has no ROS installation. Its first on-host smoke run remains necessary.

## Shared-coordinate protocol

`robotx_map` is WGS84 ENU: **x east, y north, z up**, with z=0 at the surveyed water surface. Set identical `origin_latitude`, `origin_longitude`, and **WGS84 ellipsoid water altitude** on every participating node, then set `datum_configured: true`. The default false setting inhibits mapping and following until configured. The datum hash in map/image messages must match the receiver's configured datum. Changing the datum requires restarting the survey; never relabel an unrelated local pose as `robotx_map`.

| Interface | Type | Contract |
|---|---|---|
| `/uav/downward/image_raw` | `sensor_msgs/Image` | BGR/RGB image, acquisition stamp, optical frame |
| `/uav/downward/camera_info` | `sensor_msgs/CameraInfo` | Measured intrinsics/distortion at the exact image size, same optical frame |
| TF `robotx_map` → camera optical frame | TF | Camera's capture-time translation and full orientation, including measured body-to-camera mounting transform |
| `/uav/mapping/buoys` | `robotx_coordination_interfaces/BuoyMap` | Absolute ENU buoy positions, physical radius, position uncertainty, observation times, surveyed footprints and datum hash |
| `/uav/mapping/georeferenced_image` | `robotx_coordination_interfaces/GeoImage` | JPEG plus exact capture pose, calibration, stamp and water datum |
| `/boat/coordination/odometry` | `nav_msgs/Odometry` | USV measured pose in shared map with valid covariance; body attitude is ROS FLU relative to ENU |
| `/uuv/coordination/odometry` | `nav_msgs/Odometry` | UUV measured pose in the **same** map, including depth and covariance |
| `/perception/objects` | `boat_interfaces/DetectedObjectArray` | Current onboard obstacles in `base_link`; empty current array is a valid heartbeat |
| `/mission/target` | `boat_interfaces/NavigationTarget` | Existing USV controller contract in `base_link` |
| `/uuv/follow/cmd_vel` | `geometry_msgs/TwistStamped` | ENU world velocity, then validated by `velocity_guard` |
| `/uuv/control/cmd_vel` | `geometry_msgs/TwistStamped` | Guard output consumed by the existing UUV command bridge |

The typed topics are the on-vehicle communications protocol. UAV and USV must share a ROS domain and functioning DDS networking, or use a configured ROS relay for these exact types. The existing dashboard TCP bridges do not forward these new topics. Synchronize host clocks: source timestamps and receipt times are both checked where commands depend on them. USV/UUV ROS pose exchange stays local on their shared Jetson. The separate UUV autopilot and sensors still require their actual tether/telemetry connections.

The Jetson receiver saves JPEGs plus JSON sidecars under `~/robotx_survey`, up to the configured `max_images`. Each sidecar contains the original camera pose, calibration and datum; this is georeferenced imagery plus a semantic buoy map, rather than a stitched orthomosaic.

## UAV configuration

The `uav_mapping` config lives at `uav_ws/src/uav_mapping/config/uav_mapping.yaml`. Set the shared datum and the downward OAK-D DeviceID/MXID. The driver reads calibrated RGB intrinsics/distortion from the frame metadata, including the actual crop/resize. Check that stored factory calibration matches the lens and verify it at the operating focus/altitude; missing or unsupported calibration is rejected. The camera pose must be provided by TF at image time. The mapper waits briefly for capture-time TF if it arrives after the image; it never substitutes the latest orientation for an old exposure.

The included `gps_pose` adapter can produce shared UAV odometry/TF from MAVROS GPS, ROS ENU body attitude, and a **downward calibrated range to water**. It rejects unknown/high GPS covariance, invalid range, excess tilt, and unsynchronized or stale sources. Set the range reference/lever arms so the resulting height represents the body origin. GPS altitude alone and relative takeoff altitude are not used as UAV water height. If a fused estimator already publishes accurately aligned shared TF, use that instead.

```bash
ros2 run robotx_coordination gps_pose --ros-args \
  --params-file /absolute/path/to/calibrated_uav_pose.yaml \
  -r /tf:=/uav/tf -r /tf_static:=/uav/tf_static
ros2 launch uav_mapping uav_mapping.launch.py \
  config:=/absolute/path/to/calibrated_uav_mapping.yaml
```

The downward camera is the **second OAK-D device**, not the stereo right camera on the first OAK-D. This mapper uses the second device's RGB sensor (`CAM_A`); stereo depth is not used as aerial water height. List connected DeviceIDs with both camera drivers stopped:

```bash
python3 -c 'import depthai as dai; print([d.getDeviceId() for d in dai.Device.getAllAvailableDevices()])'
ros2 launch uav_bringup dual_oak_mapping.launch.py \
  primary_device_id:=FIRST_OAK_MXID downward_device_id:=SECOND_OAK_MXID \
  config:=/absolute/path/to/calibrated_uav_mapping.yaml
```

The paired launch rejects missing or identical IDs, keeps the first RGB stream on `/uav/oak/rgb/image_raw`, and publishes the second on `/uav/downward/image_raw` with matching CameraInfo. It also starts the mapper; do not separately launch another mapper or either camera driver. For only the second camera, use `uav_mapping.launch.py start_camera:=true` with `camera_device_id` in its YAML. The existing first-camera executable now also requires `device_id` when launched directly.

The downward driver converts DepthAI's host-synchronized acquisition timestamp into the Pi's ROS system-clock epoch by subtracting measured frame age. It rejects stale/future frames and hardware use with simulation time. It does not stamp delayed captures as fresh frames or use the raw device-boot clock. The Pi/Jetson clocks still need synchronization for cross-host freshness checks.

Publish the measured camera mounting TF on `/uav/tf_static` and shared UAV pose TF on `/uav/tf`; the mapper listens there. For example, with YOUR measured translation/quaternion:

```bash
ros2 run tf2_ros static_transform_publisher --x X --y Y --z Z \
  --qx QX --qy QY --qz QZ --qw QW \
  --frame-id uav/base_link --child-frame-id uav/downward_optical \
  --ros-args -r /tf_static:=/uav/tf_static
```

A downward mounting example is optical x → body -y, optical y → body -x, optical z → body -z. Its xyzw quaternion is approximately `(0.7071, -0.7071, 0, 0)`; **measure your actual mount** and publish that static body-to-optical TF. The simulation uses this transform. The mapper handles vehicle yaw/roll/pitch through full TF, not an assumption that the camera always faces north.

The baseline image detector recognizes red, green and yellow connected components using HSV thresholds. Field validation/replacement with a trained detector is still needed for glare, shadows, missed detections and other buoy colors. Set `georef_uncertainty` from measured GPS, attitude, altitude, timing and pixel error at survey altitude; the simulation's 0.25 m is not a field accuracy claim. The mapping package runs alongside the existing UAV flight control; autonomous survey takeoff/flight/landing is not added here.

## USV configuration and operation

Set the map datum, goal (ENU meters), course bounds, boat clearance, speed and timeouts in `boat_ws/src/boat_route_navigation/config/boat_route_navigation.yaml`.

The optional GPS adapter supports `vehicle: boat`, with `/mavros/global_position/global` and `/mavros/imu/data`, publishing `/boat/coordination/odometry`. It fixes the boat water-plane reference to z=0. A validated shared localization estimator may supply this topic instead.

Start the existing sensors/perception/control/vehicle stack with the Task 1 mission excluded, then launch the route package in the same overlay/domain:

```bash
ros2 launch boat_bringup task1_bringup.launch.py start_mission:=false
ros2 launch boat_route_navigation boat_route_navigation.launch.py \
  config:=/absolute/path/to/calibrated_boat_route_navigation.yaml
ros2 service call /boat/route/set_enabled std_srvs/srv/SetBool '{data: true}'
```

The route service only enables target generation. The existing bridge's software stop, explicit autonomy enable, vehicle mode and arm procedure still control real propulsion. It is not enabled by this launch. Completing the route publishes the existing `MISSION_COMPLETE` state, so the unchanged bridge requests LOITER and revokes autonomy. Reset a completed route with `/boat/route/reset` before re-enabling it.

A* uses only observed camera footprints, inflated by the vehicle clearance. It inflates buoy circles by physical radius + mapping uncertainty + boat clearance. It checks every planned segment, prevents diagonal grid corner-cutting, and rechecks the current steering segment against live onboard detections. Unsafe local observations result in zero targets while replanning. The route is published as `/boat/route/path` for visualization.

Observed footprints expire after `survey_ttl`; old water becomes unknown. Previously observed buoys are conservatively retained, even if later images miss them. A hovering camera updates its footprint rather than accumulating duplicates. A fixed survey has bounded memory; restart the mapper to begin a fresh survey after repositioning buoys or changing the datum. This first implementation does not perform a camera-derived free-space certification or full local avoidance maneuver planning.

## UUV configuration and operation

Set the same datum and a measured UUV pose source in `uuv_ws/src/uuv_follow/config/uuv_follow.yaml`. Defaults: 3 m trail lag, 1.5 m depth, 0.4 m/s maximum horizontal speed, 0.2 m/s vertical speed and 5 m maximum configured depth. Choose limits for the actual course before deployment.

The UUV cannot locate itself horizontally underwater from ordinary GPS or depth alone. Provide a validated estimator (for example externally referenced DVL/acoustic/visual localization) and align it to the shared water datum. Its covariance and timestamps must reflect the actual measurements. The GPS adapter explicitly does not provide underwater localization. Publishing simulated or integrated command motion as measured odometry on the real vehicle would invalidate this controller's assumptions.

With the existing UUV bringup/safety/mission/bridge running:

```bash
ros2 launch uuv_follow uuv_follow.launch.py \
  config:=/absolute/path/to/calibrated_uuv_follow.yaml
```

The follower reads the existing `/uuv/mission/state` enable flag and `/uuv/safety/authorized` heartbeat; it starts without motion authority. It also checks fresh connected/armed/GUIDED MAVROS state. The existing mission manager's `/uuv/control/set_enabled` remains the enable service. The default safety supervisor's camera/perception requirements remain in force.

The guard verifies `mav_frame == LOCAL_NED` on `/uuv/mavros/setpoint_velocity` through its parameter service before passing motion. For that MAVROS frame, the ROS input is world ENU; MAVROS converts it to NED. Do not configure BODY_NED for these commands. ArduSub GUIDED velocity control also requires a working navigation estimate in its EKF. Verify external navigation is supplied to the autopilot as well as the ROS follower.

The UUV follows the observed USV trail rather than inventing a waypoint behind an instantaneous heading. It waits until enough trail has been observed, reacquires the nearest trail segment, tracks corners without a long diagonal shortcut, and maintains negative-z depth. A boat-pose jump or telemetry gap latches it stopped until the mission is disabled and `/uuv/follow/reset` is called. Missing own position, stale safety authorization, wrong frame, bad covariance, bad mode, invalid commands or parameter-check failure give zero output. No automatic arm/mode request is added.

This follower does not sense underwater obstacles, prove overhead/thruster separation, or manage a physical tether. Its safe path and depth depend on measured localization and the actual operating environment; the software fixture validates only its stated kinematic assumptions.

## Sources checked for velocity semantics

- MAVROS ROS 2 `setpoint_velocity` implementation: https://github.com/mavlink/mavros/blob/ros2/mavros/src/plugins/setpoint_velocity.cpp
- ArduSub GUIDED velocity message handling: https://github.com/ArduPilot/ardupilot/blob/master/ArduSub/GCS_MAVLink_Sub.cpp

## Sources checked for OAK-D and ROS deployment

- Luxonis DepthAI v3 device selection and clocks: https://docs.luxonis.com/software-v3/depthai/depthai-components/device
- Camera output/resize API: https://docs.luxonis.com/software-v3/depthai/depthai-components/nodes/camera/
- Frame intrinsics/transformation metadata: https://docs.luxonis.com/software-v3/depthai/depthai-components/messages/img_transformations/
- ROS 2 wildcard topic/service remapping: https://design.ros2.org/articles/static_remapping.html
- MAVROS ROS 2 namespace launch: https://github.com/mavlink/mavros/blob/ros2/mavros/launch/node.launch
