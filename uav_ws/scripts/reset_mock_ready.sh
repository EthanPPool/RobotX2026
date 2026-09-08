#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/kilted/setup.bash
source "${HOME}/uav_ws/install/setup.bash"

ros2 param set /mock_mavros connected true
ros2 param set /mock_mavros gps_valid true
ros2 param set /mock_mavros local_position_valid true
ros2 param set /mock_mavros battery_percentage 0.80
ros2 param set /mock_mavros mode GUIDED
ros2 service call /vehicle/set_autonomy uav_interfaces/srv/SetAutonomy "{enabled: false}"
ros2 service call /safety/reset_failsafe std_srvs/srv/Trigger "{}" || true
ros2 topic echo --once /vehicle/safety_status
