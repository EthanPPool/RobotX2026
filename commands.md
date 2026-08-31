

## Add an IP Address
sudo ip addr add 192.168.1.2/24 dev enP8p1s0

## Start/Stop LiDAR
source /opt/ros/kilted/setup.bash
source ~/unilidar_l2_ws/install/setup.bash
ros2 service call unitree_lidar_ros2_node/<start/stop>_rotation std_srvs/srv/Trigger "{}"

###################################################
ROS2 Cheat Sheet
###################################################
1: Set ROS Domain
export ROS_DOMAIN_ID=0
2: Run a Node
ros2 run <package> <executable>
3: Launch Multiple Nodes
ros2 launch <package> <launch_file>.launch.py
4: List Running Nodes
ros2 node list
5: Inspect Node
ros2 node info /node_name
6: List Topics
ros2 topic list
7: Inspect Topic
ros2 topic info /topic_name
8: Echo Topic
ros2 topic echo /topic_name
9: Topic Publish Rate
ros2 topic hz /topic_name
10: Publish Topic
ros2 topic pub /topic_name <msg_type> '<message>'
11: List Services
ros2 service list
12: Inspect Service
ros2 service info /service_name
13: Call Service
ros2 service call /service_name <srv_type> '<request>'
14: List Actions
ros2 action list
15: Inspect Action
ros2 action info /action_name
16: Send Action Goal
ros2 action send_goal /action_name <action_type> '<goal>'
17: Restart ROS Daemon
ros2 daemon stop
###################################################
Workspace
###################################################
18: Create Workspace
mkdir -p ~/colcon_ws/src
cd ~/colcon_ws && colcon build
19: Source Workspace
source install/setup.bash
20: Clone Package
cd ~/colcon_ws/src
git clone <repo_url>
21: Install Dependencies
rosdep update
rosdep install --from-paths src --ignore-src -r -y
###################################################
Packages
###################################################
22: Create C++ Package
ros2 pkg create --build-type ament_cmake <package_name>
23: Create Python Package
ros2 pkg create --build-type ament_python <package_name>
###################################################
Your Common Commands
###################################################
24: Build Whole Workspace
cd ~/rx26_ws
colcon build
25: Build One Package
colcon build --packages-select rx26_localization
26: View TF Tree
ros2 run tf2_tools view_frames
27: Launch MAVROS
ros2 launch mavros apm.launch fcu_url:=udp://0.0.0.0:14550@
28: Echo GPS
ros2 topic echo /mavros/global_position/global
29: Echo IMU
ros2 topic echo /mavros/imu/data
30: List Packages
ros2 pkg list
31: List Parameters
ros2 param list
32: Get Parameter
ros2 param get /node parameter_name
33: Set Parameter
ros2 param set /node parameter_name value
34: List Interfaces
ros2 interface list
35: Show Message Definition
ros2 interface show sensor_msgs/msg/NavSatFix
36: Monitor Topic Frequency
ros2 topic hz /topic_name
37: Show Topic Bandwidth
ros2 topic bw /topic_name
38: View Node Graph
rqt_graph
39: RViz
rviz2
40: Build + Source
colcon build && source install/setup.bash