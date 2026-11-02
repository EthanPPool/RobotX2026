# RobotX ROS pathing + professor two-thruster control

This upgrade adds the incremental planner from Boat Lab, the notebook's right/left
force-rate equations, a causal ROS controller, and an optional calibrated force
backend for the existing Rover bridge. It targets the two stern-thruster model.
It preserves the previous velocity-control entry point and the supplied legacy
follower file. No boot service is changed and no hardware controller starts on
installation.

The source baseline is EthanPPool/RobotX2026 main at
`d2e5237e4c3d2b6b733e51a69b4e6e475fe59701`.
The installer accepts the original planner or the already-applied v4 fixes. It
preflights every file and refuses unrecognized local edits before writing any file.

## Install on a review branch

Download robotx_ros_pathing_control.zip, then in WSL:

```bash
unzip -oq /mnt/c/Users/epeop/Downloads/robotx_ros_pathing_control.zip -d "$HOME/Downloads"
repo="$HOME/RobotX2026"
review="$HOME/RobotX2026-professor-pathing"
git -C "$repo" fetch origin main
git -C "$repo" worktree add -b feat/professor-pathing "$review" origin/main
python3 "$HOME/Downloads/robotx_ros_pathing_control/install_into_repo.py" --repo "$review"
python3 "$HOME/Downloads/robotx_ros_pathing_control/install_into_repo.py" --repo "$review" --apply
cd "$review"
bash scripts/test_competition_planner.sh
bash scripts/test_professor_control.sh
```

No GitHub push/merge is performed. Inspect `git diff` and `git status` in the
review checkout before committing. Keep local device-specific edits separate;
the installer does not overwrite an unknown vehicle bridge version.

## Build with ROS 2 Kilted

On the ROS-equipped machine, from the installed review checkout:

```bash
bash scripts/build_professor_pathing.sh
source coordination_ws/install-professor-pathing/setup.bash
```

The build includes boat_vehicle from this checkout so the new output hook cannot
silently resolve to an old installed version. Dependencies must already be available
on an offline machine. Build/test execution here used Python transport stubs; a
real colcon build, generated message import, TF timing and DDS need verification
on your ROS machine.

## Configure and run without propulsion output

```bash
cp coordination_ws/src/robotx_professor_control/config/pathing.yaml "$HOME/task1-pathing.yaml"
```

Edit that file with the actual shared geographic datum, convex course boundary,
pose/scan topics, hull dimensions and agreed local-model limits. The default datum
is disabled and the boundary is deliberately unset. All perception/localization
processes must use the same ENU frame/datum and original capture timestamps.
Set datum_configured=true only once that site configuration is supplied. Leave
the three model-confirmation flags false during the first shadow review.

Run existing camera/LiDAR interpretation and localization processes. Select
source_mode=usv_only for independent USV interpretation or source_mode=assisted
for duo, using the existing interpretation.launch.py/config interfaces. Advanced
local input should be geometry-only where its side-facing beacon semantics are
not the aerial competition assignment; see docs/INCREMENTAL_PLANNER.md.

```bash
ros2 launch robotx_professor_control task1_professor.launch.py \
  mode:=usv config:="$HOME/task1-pathing.yaml"
# Use mode:=duo for the aerial-assisted Advanced planner.
```

In another terminal with the same overlay:

```bash
ros2 topic echo /control/professor_status
ros2 topic echo /control/thruster_forces
# With live pose, observed map and scan/TF available:
ros2 service call /mission/task1/start std_srvs/srv/Trigger '{}'
```

Shadow launch sends no propulsion commands. The new launch does not include
legacy mission/control/vehicle nodes. When reusing task1_bringup for its sensors,
turn its start_mission/start_control/start_vehicle off and arrange MAVROS separately;
avoid two active planners/controllers/bridges. Existing systemd instances must
also be accounted for before selecting a hardware control path.

## What remains before hardware activation

Read docs/PROFESSOR_CONTROL_LAW.md for the exact transcription and the proposed
local-tangent/cross-track binding. Confirm with the professor: x/q3 meanings and
units, force-axis directions, constant nominal-trim applicability, local operating
domain, and the causal derivative realization. The coefficients themselves are
unchanged. No unit claim is inferred from a number's magnitude.

Populate config/calibration.template.json using measured force-versus-command
data for EACH thruster at the actual supply voltage range. Verify the FCU mixing
matrix, reversals and endpoints. Optional RPM/torque/voltage/PWM columns require
measurements; the code does not manufacture them. Set calibration_file to that
absolute file path and the calibration/contract flags only after supplying those
facts. The template cannot produce calibrated commands as shipped.

The optional bridge is selected with start_force_bridge:=true. Its hardware
output remains disabled by default; its deployment flag, the controller contract
flags, calibration, observed MANUAL mode and existing operator authority conditions
all apply. This document does not arm or enable propulsion automatically.

See docs/ROS_PATHING_PROTOCOL.md for interfaces and transport behavior, and
VALIDATION.md for checks and remaining limits. The simulator's successful kinematic
runs are not closed-loop validation of the professor controller or this hull.
