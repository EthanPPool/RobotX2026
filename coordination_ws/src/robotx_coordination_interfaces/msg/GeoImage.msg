# Timestamp is image acquisition time. Pose transforms optical coordinates to map.
std_msgs/Header header
string reference_id
float64 origin_latitude
float64 origin_longitude
float64 water_altitude
sensor_msgs/CompressedImage image
sensor_msgs/CameraInfo calibration
geometry_msgs/Pose camera_pose
