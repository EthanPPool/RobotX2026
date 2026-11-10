#!/usr/bin/env bash
set -eo pipefail
repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/kilted/setup.bash
cd "$repo_dir"
# One runtime install contains the existing two-gate stack and HSV adapter.
# Run in a clean shell; do not source an older RobotX workspace first.
colcon --log-base log-two-gate-hsv build --symlink-install \
  --build-base build-two-gate-hsv --install-base install-two-gate-hsv \
  --base-paths boat_ws/src coordination_ws/src/robotx_coordination \
    coordination_ws/src/robotx_coordination_interfaces \
    coordination_ws/src/robotx_buoy_interpretation \
  --packages-up-to boat_bringup
echo "Source $repo_dir/install-two-gate-hsv/setup.bash"
