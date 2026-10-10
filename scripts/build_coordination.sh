#!/usr/bin/env bash
set -eo pipefail
repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
profile="${1:-sim}"
common=(src/robotx_coordination src/robotx_coordination_interfaces)
boat=(../boat_ws/src/boat_interfaces ../boat_ws/src/boat_control
      ../boat_ws/src/boat_vehicle ../boat_ws/src/boat_route_navigation)
case "$profile" in
  sim)
    paths=("${common[@]}" src/robotx_coordination_sim "${boat[@]}"
           ../uav_ws/src/uav_mapping ../uuv_ws/src/uuv_follow)
    packages=(robotx_coordination_sim)
    ;;
  jetson)
    paths=("${common[@]}" "${boat[@]}" ../uuv_ws/src)
    packages=(boat_route_navigation boat_control boat_vehicle uuv_follow uuv_bringup)
    ;;
  uav-pi)
    paths=("${common[@]}" ../uav_ws/src)
    packages=(uav_mapping uav_bringup)
    ;;
  *)
    echo 'Usage: bash scripts/build_coordination.sh [sim|jetson|uav-pi]' >&2
    exit 2
    ;;
esac
source /opt/ros/kilted/setup.bash
cd "$repo_dir/coordination_ws"
rosdep install --from-paths "${paths[@]}" --ignore-src -r -y
colcon --log-base "log-$profile" build --symlink-install \
  --build-base "build-$profile" --install-base "install-$profile" \
  --base-paths "${paths[@]}" --packages-up-to "${packages[@]}"
echo "Built $profile overlay: source $repo_dir/coordination_ws/install-$profile/setup.bash"
