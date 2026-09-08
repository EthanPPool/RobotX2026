# UAV Workspace Commands — v0.3

## Build
```bash
cd ~/uav_ws
source /opt/ros/kilted/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --executor sequential
source install/setup.bash
```

## Launch software test
```bash
ros2 launch uav_bringup software_test.launch.py
```

## Return a v0.2-style FAILSAFE test to READY
```bash
ros2 param set /mock_mavros connected true
ros2 param set /mock_mavros gps_valid true
ros2 param set /mock_mavros local_position_valid true
ros2 param set /mock_mavros battery_percentage 0.80
ros2 service call /vehicle/set_autonomy uav_interfaces/srv/SetAutonomy "{enabled: false}"
ros2 service call /safety/reset_failsafe std_srvs/srv/Trigger "{}"
ros2 topic echo --once /vehicle/safety_status
```

## Vehicle management — mock test

### Safety status
```bash
ros2 topic echo --once /vehicle/safety_status
```

### Arm
```bash
ros2 service call /vehicle/arm std_srvs/srv/Trigger "{}"
```

### Disarm
```bash
ros2 service call /vehicle/disarm std_srvs/srv/Trigger "{}"
```

### Set GUIDED
```bash
ros2 service call /vehicle/set_mode uav_interfaces/srv/SetFlightMode "{mode: 'GUIDED'}"
```

### Takeoff to 2 m
```bash
ros2 service call /vehicle/takeoff uav_interfaces/srv/Takeoff "{altitude: 2.0}"
```

### LAND
```bash
ros2 service call /vehicle/land std_srvs/srv/Trigger "{}"
```

### RTL
```bash
ros2 service call /vehicle/rtl std_srvs/srv/Trigger "{}"
```

### Mock state
```bash
ros2 topic echo --once /mock_mavros/state
```

### Mock local pose / altitude
```bash
ros2 topic echo --once /mock_mavros/local_position/pose
```

## Mission/autonomy pipeline
```bash
ros2 service call /mission/start std_srvs/srv/Trigger "{}"
ros2 service call /vehicle/set_autonomy uav_interfaces/srv/SetAutonomy "{enabled: true}"
ros2 topic echo --once /vehicle/autonomy_status
```

## Abort / disable
```bash
ros2 service call /mission/abort std_srvs/srv/Trigger "{}"
ros2 service call /vehicle/set_autonomy uav_interfaces/srv/SetAutonomy "{enabled: false}"
```

## Reset FAILSAFE
```bash
ros2 service call /safety/reset_failsafe std_srvs/srv/Trigger "{}"
```

## Real-hardware lock
`autonomy.yaml` defaults vehicle command execution to false. Only for a later,
controlled propeller-off bench test should it be deliberately enabled:
```bash
ros2 param set /vehicle_manager command_execution_enabled true
```
