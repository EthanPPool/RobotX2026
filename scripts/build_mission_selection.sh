#!/usr/bin/env bash
# Build only the selection overlay; keep the existing working install intact.
set -eo pipefail
selection_repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/kilted/setup.bash
case "${1:-}" in
  jetson)
    source "${USV_BASE_SETUP:-$HOME/boat_ws/install/setup.bash}"
    if [[ -f "$HOME/boat_ws_gate_selection_test/install/setup.bash" ]]; then
      source "$HOME/boat_ws_gate_selection_test/install/setup.bash"
    fi
    selection_sources=(boat_ws/src/boat_interfaces boat_ws/src/boat_perception
      boat_ws/src/boat_mission boat_ws/src/boat_dashboard_bridge boat_ws/src/boat_bringup
      boat_ws/src/boat_route_navigation coordination_ws/src/robotx_coordination
      coordination_ws/src/robotx_coordination_interfaces)
    selection_packages=(boat_interfaces boat_perception boat_mission boat_dashboard_bridge
      boat_bringup boat_route_navigation robotx_coordination robotx_coordination_interfaces)
    selection_profile=jetson
    ;;
  groundstation)
    source "${GROUND_STATION_BASE_SETUP:-$HOME/RobotX2026/ground_station_ws/install/setup.bash}"
    selection_sources=(ground_station_ws/src/robotx_dashboard)
    selection_packages=(robotx_dashboard)
    selection_profile=groundstation
    ;;
  *) echo 'Usage: bash scripts/build_mission_selection.sh [jetson|groundstation]' >&2; exit 2 ;;
esac
cd "$selection_repo"
colcon --log-base "log-selection-$selection_profile" build --symlink-install \
  --build-base "build-selection-$selection_profile" --install-base "install-selection-$selection_profile" \
  --base-paths "${selection_sources[@]}" --packages-select "${selection_packages[@]}" \
  --allow-overriding "${selection_packages[@]}"
echo "Overlay ready: source $selection_repo/install-selection-$selection_profile/setup.bash"
echo 'No services restarted and no ROS nodes launched.'
