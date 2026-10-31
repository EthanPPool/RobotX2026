#!/usr/bin/env bash
set -eo pipefail
repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/kilted/setup.bash
cd "$repo_dir/coordination_ws"
# Existing coordination overlays may be sourced before running this script.
# No network or rosdep installation is attempted on the field devices.
colcon --log-base log-buoy-interpretation build --symlink-install \
  --build-base build-buoy-interpretation --install-base install-buoy-interpretation \
  --base-paths src/robotx_coordination src/robotx_coordination_interfaces \
    src/robotx_buoy_interpretation ../boat_ws/src/boat_interfaces \
  --packages-up-to robotx_buoy_interpretation
echo "Source $repo_dir/coordination_ws/install-buoy-interpretation/setup.bash"
