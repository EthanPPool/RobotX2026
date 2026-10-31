# Competition buoy interpretation: USV independent, UAV optional

This addition builds both interpretation branches around the teammate's color-assignment model. It does not substitute an HSV classifier, assume two PoR gates, modify the professor's controller, or connect a new source to propulsion. It prepares a common competition-planner input. Automatic competition task decisions and the PID/ESC connection remain subsequent integration work.

Reference source: `https://github.com/EthanPPool/RobotX2026`, main commit `50855604c97658b4221bf921d66927ee9dd902fa`. Existing device workspaces may contain newer offline edits; use the additive installer rather than overwrite their existing packages.

## Runtime placement and modes

| Process | Host | Input | Output |
| --- | --- | --- | --- |
| Teammate model plus output adapter | Host processing that camera feed | Camera image | `BuoyColorDetectionArray` |
| `usv_buoy_interpretation` | USV Jetson | USV detections, camera calibration, LiDAR objects, acquisition-time TF | `/perception/buoys/usv` |
| `uav_buoy_interpretation` | UAV Pi or processing host with UAV calibration/TF | UAV detections, calibration, acquisition-time TF | `/perception/buoys/uav` |
| `buoy_tracker` | USV Jetson | Local observations and optional UAV observations | `/mission/buoys` |

`source_mode:=usv_only` is the default. The tracker does not subscribe to UAV observations in this mode. The USV camera/model pipeline is sufficient for color interpretation; LiDAR geometry still publishes as unknown obstacles when the camera or model is absent, but unknown obstacles cannot supply color-based mission decisions.

`source_mode:=assisted` accepts both streams. The UAV has no startup-ready requirement, heartbeat prerequisite, or authority to suspend the USV input. If it disappears, its semantics age out using original capture times. USV observations continue. Retained UAV geometry stays as a conservative obstacle until retention expiry; it is not automatically relabeled as free water. Expired or never observed areas remain unknown to the future planner.

UAV flight readiness is a team operational decision; this package does not arm, launch, or inspect flight approval. Select assisted mode only when that infrastructure is available. The aerial camera can alternatively feed a ground processing host if the original timestamps, calibration, and TF are transported with it.

## Model output contract

New messages live in the existing `robotx_coordination_interfaces` package. No dependency on the teammate's ML framework is imposed.

Publish `BuoyColorDetectionArray` on:

- `/perception/usv/color_detections` for the USV feed.
- `/perception/uav/color_detections` for the UAV feed.

The array header must contain the ORIGINAL IMAGE ACQUISITION timestamp and optical frame. Image width and height must match the rectified camera image and `CameraInfo`. Set `image_rectified: true` only when those coordinates actually refer to a rectified image; false is rejected. Each detection contains:

| Field | Meaning |
| --- | --- |
| `detection_id` | Unique within this image; persistent ML tracking is not required |
| `u`, `v` | Bounding-box center, in original rectified image pixels |
| `width`, `height` | Bounding-box size, pixels |
| `color` | `red`, `green`, `blue`, `black`, `yellow`, `white`, or `unknown` |
| `confidence` | Finite normalized value in [0,1] |
| `light_state` | `unknown`, `steady`, `flashing`, or `inactive` |

The model adapter must undo inference resizing, cropping and letterboxing. Raw distorted-image coordinates must first be rectified, or the team must extend the geometric adapter for the actual distortion model. Calibration uses CameraInfo's rectified `P`, not a guessed focal length. Nonzero stereo projection offsets and nonidentity rectification rotations `R` are rejected; use monocular/reference-optical rectification with identity `R`, so the pixels and TF describe the same optical axes. Supporting a rotated stereo rectification would require the corresponding rectified optical TF or explicit rotation in both adapters.

The current repository's UAV `downward_camera.py` explicitly uses `enableUndistortion=False` and publishes `/uav/downward/image_raw`. Feed a properly rectified version of that image to the model, preserving capture time and matching CameraInfo, before setting the assertion. This package does not silently reinterpret the existing raw feed as rectified or change the camera's startup process.

An example member is `{detection_id: 4, u: 320.0, v: 240.0, width: 30.0, height: 50.0, color: blue, confidence: 0.92, light_state: unknown}`. This is a contract example, not a built-in detection.

Color assignment alone does not establish flashing vs steady beacon state. The teammate may provide that field directly; otherwise the shared tracker now estimates it from explicit tracked color observations. It requires two measured on/off cycles for flashing, or 2.5 seconds of contiguous observations for steady/inactive. A gap longer than 0.75 seconds resets the temporal history. Black must be an explicit observation of that buoy's inactive beacon; an unknown or missed detection is never OFF. The learned active color is retained through an observed off phase. The planner waits if temporal evidence is insufficient.

## Coordinate and timestamp requirements

All map output is SI, WGS84 ENU: x east, y north, z up, water surface z=0. ROS optical axes are x right, y down, z forward. Both vehicles must use the SAME configured surveyed latitude, longitude, and water altitude. Existing `robotx_coordination` reference hashing enforces that origin. A matching frame name alone is insufficient; datum hashes must also match.

Set `datum_configured: true` in a copy of `interpretation.yaml` after configuring the same actual datum and TF tree on the participating hosts. Default false intentionally publishes no actionable observations. Independent MAVROS local origins are not a shared map. Camera-to-body extrinsics and vehicle pose must establish `robotx_map <- camera_optical` at capture time.

Calibration is selected from a bounded history at or before image capture, within `calibration_timeout`. Zero-stamped, truly static calibration can be enabled explicitly with `allow_static_calibration: true`; matching frame and dimensions still apply. A newly resized or different camera's calibration cannot be used for an older image.

The USV adapter transforms LiDAR positions at their capture timestamp into the map, then into the camera using the image capture pose. LiDAR and image captures must fall within `max_sync`. Ambiguous projected bounding boxes receive no color assignment. Geometry remains present as unknown obstacles.

The UAV adapter intersects calibrated pixel rays with the water plane using the UAV optical pose at image capture, including attitude and camera mounting. No GPS/image pixel shortcut is used. The default anchor is bbox bottom-center for an oblique water-contact approximation; for nadir imagery a calibrated center anchor (`anchor_fraction: 0.0`) may be more appropriate. Buoy height, bounding-box extent, pixel error, and configured pose uncertainty inflate position uncertainty. This is an estimate requiring field validation. Excessively uncertain, near-horizon, upward-facing, stale or out-of-range detections are rejected.

Synchronize host clocks. Model compute/network latency counts toward the capture timeout; re-stamping inference output as current time is not allowed. No source uses latest TF for an older capture. Missing TF/calibration retries are bounded by that capture's age.

## Shared tracking and mission-planner interface

`/mission/buoys` uses `BuoyTrackArray`. Track IDs are created here; USV detector IDs and UAV detection indices are not shared buoy identities. Spatial association is distance-gated. The USV's active local geometry IDs can disambiguate already tracked close obstacles; UAV frame-local IDs cannot. Ambiguous cross-source matches are rejected, not assigned by color or image order. Set association distance below local buoy separation and validate on the actual course.

Each track includes position, physical radius, uncertainty, color confidence, optional light state, original geometry/color times, all contributing sources, currently active sources, `semantic_conflict`, and `actionable`.

- Fresh observations from both vehicles of the same buoy merge without averaging a precise local obstacle away.
- Contradictory active color/light evidence produces unknown semantics and `actionable=false`.
- Geometry-only updates do not refresh old color evidence. Delayed inference may enrich an earlier geometry capture without rewinding current position or refreshing age.
- Unknown/stale-color tracks remain obstacles, not selectable route markers.
- Source/dropout age and obstacle retention are separate. Republished map headers do not renew original observation times.
- Uncertainty is a conservative distance bound, not a covariance. Future planning clearance must include BOTH radius and uncertainty, plus the USV hull clearance.
- No map entry or empty array proves a region is navigable. Local free-space assessment and collision checking remain required before generating an executable path.

The competition mission planner should subscribe only to this common track interface, choose maneuvers from the actual competition rules, and pass a feasible spatial route to trajectory generation. No two-gate count or universal red/green pairing is embedded here. The controller-facing trajectory should use smooth timed samples at t, t+dt, t+2dt in the professor's defined coordinates. The control law is accepted as supplied and has not been rewritten in this addition.

## Build and run

The package is added under `coordination_ws/src`. Build dependencies include existing `boat_interfaces` (needed by the USV adapter), `robotx_coordination`, and the expanded coordination message package. Use the same interface definitions on both hosts. No Internet dependency installation is attempted by the build script.

From the real repository root, after dependencies and ROS 2 Kilted are available:

```bash
bash scripts/build_buoy_interpretation.sh
source coordination_ws/install-buoy-interpretation/setup.bash
```

The reference repository contains a legacy `robotx@localhost` maintainer address in `robotx_coordination` metadata. The previous offline build work already corrected it on the devices. If the build reports this exact metadata error, retain that existing correction (`robotx@example.com`); this installer does not overwrite existing coordination package files.

Copy the configuration to a site-specific file, set the common datum, set camera topic/extrinsics, and validate calibration/uncertainty first. On Jetson:

```bash
ros2 launch robotx_buoy_interpretation interpretation.launch.py \
  config:=/absolute/path/to/site_interpretation.yaml source_mode:=usv_only
```

To accept UAV input, change only `source_mode:=assisted`. On the UAV Pi or its processing host:

```bash
ros2 launch robotx_buoy_interpretation uav_interpretation.launch.py \
  config:=/absolute/path/to/site_interpretation.yaml
```

Run these as additional interpretation processes. They are not inserted into boot services or the current PoR launch. They never publish `/control/cmd_vel`, PWM, or arming/mode requests.

Inspect `/perception/buoys/usv/status`, `/perception/buoys/uav/status`, `/mission/buoy_interpretation/status`, and `/mission/buoys`. Verify the same physical buoy in both streams has overlapping map position uncertainty, an unambiguous color, and the same datum reference before accepting assisted observations.

## Verification and remaining integration

Run `bash scripts/test_buoy_interpretation.sh` with Python 3 and numpy. No ROS installation, model weights, camera or network is required for these tests. They cover real geometry/tracker algorithms and ROS adapter callbacks with transport stubs. They are not hardware or DDS tests.

The teammate must connect their actual model output to the two detection topics. Camera rectification, TF extrinsics, shared datum, field uncertainty bounds, and camera/model latency must be measured on the devices. The separate `robotx_competition_planner` package now implements Core/Advanced Safe Passage routing and timed trajectory previews; see `docs/COMPETITION_PLANNER.md`. Professor-controller/ESC integration is a separate consumer of those references.
