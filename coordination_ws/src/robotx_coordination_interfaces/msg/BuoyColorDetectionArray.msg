# Original image acquisition stamp and optical frame, preserved through inference.
std_msgs/Header header
uint32 image_width
uint32 image_height
# Explicit producer assertion; raw image detections require rectification first.
bool image_rectified
BuoyColorDetection[] detections
