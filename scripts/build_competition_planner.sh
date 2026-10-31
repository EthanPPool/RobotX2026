#!/usr/bin/env bash
set -eo pipefail
repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/kilted/setup.bash
cd "$repo_dir/coordination_ws"
colcon --log-base log-competition-planner build --symlink-install \
  --build-base build-competition-planner --install-base install-competition-planner \
  --base-paths src/robotx_coordination src/robotx_coordination_interfaces \
    src/robotx_buoy_interpretation src/robotx_competition_planner \
    ../boat_ws/src/boat_interfaces \
  --packages-up-to robotx_competition_planner
echo "Source $repo_dir/coordination_ws/install-competition-planner/setup.bash"
