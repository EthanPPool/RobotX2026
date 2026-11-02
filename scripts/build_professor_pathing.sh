#!/usr/bin/env bash
set -eo pipefail
repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/kilted/setup.bash
colcon --log-base "$repo_dir/coordination_ws/log-professor-pathing" build --symlink-install \
  --build-base "$repo_dir/coordination_ws/build-professor-pathing" \
  --install-base "$repo_dir/coordination_ws/install-professor-pathing" \
  --base-paths \
    "$repo_dir/boat_ws/src/boat_interfaces" \
    "$repo_dir/boat_ws/src/boat_vehicle" \
    "$repo_dir/coordination_ws/src/robotx_coordination_interfaces" \
    "$repo_dir/coordination_ws/src/robotx_coordination" \
    "$repo_dir/coordination_ws/src/robotx_buoy_interpretation" \
    "$repo_dir/coordination_ws/src/robotx_competition_planner" \
    "$repo_dir/coordination_ws/src/robotx_control_interfaces" \
    "$repo_dir/coordination_ws/src/robotx_professor_control" \
  --packages-up-to robotx_professor_control
printf 'Source %s/coordination_ws/install-professor-pathing/setup.bash\n' "$repo_dir"
