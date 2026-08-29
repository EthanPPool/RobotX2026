Notes for buoy detector

Imports:

  - Loads ROS 2, math utilities, LiDAR LaserScan, RViz markers, and your custom detected-object messages.

Track dataclass:

  - Stores the state of one potential buoy over time: position, size, confidence, number of detections (hits), and missed detections (misses).

BuoyDetector.__init__():

Sets up the ROS node and loads all tunable parameters. These parameters define:

  - where the detector should look
  
  - how LiDAR points are clustered

  - acceptable buoy size/shape

  - confidence thresholds

  - tracking behavior

Publishers and subscriber:

  - Subs to --> /perception/scan

  - Pubs to --> /perception/objects and /perception/object_markers

scan_callback():

  - processing pipeline, when a new LiDAR scan arrives it:
  
  - scan --> build clusters --> evaluate clusters --> update tracks --> publish confirmed buoys

build_clusters():

  - Converts valid LiDAR ranges into (x, y) points and groups nearby consecutive points into clusters. It also removes points that are too far away, behind the boat, too far sideways, or outside the permitted viewing angle.

evaluate_cluster():

Decides if a cluster is a buoy checking:

  - number of Lidar points

  - width

  - depth

  - radial variation

  - compactness using PCA

  - curvature/convexity

all posted with a confidence score, bad candidates are discarded

PCA compactness block:

  - Examines the geometric shape of the cluster. It helps reject objects that are essentially straight lines, such as walls or long flat surfaces.

Convexity block:

  - Checks whether the middle of the detected object is slightly closer to the LiDAR than its edges. This helps identify rounded objects such as buoys.

Confidence scoring:

  - 35% expected buoy size

  - 25% number of LiDAR points

  - 25% geometric shape

  - 15% convexity

update_tracks():

  - Tracks candidates across multiple LiDAR scans. A new detection is matched to an existing track if it is nearby. This prevents a single noisy LiDAR reading from immediately being called a buoy.

confirm_hits:

  - A track must normally be detected several times before being published. With confirm_hits=3; the detector needs three successful observations before declaring it a real buoy.

Miss handling:

  - If a tracked object disappears temporarily, its misses count increases. If it disappears for too long, its track is deleted.

Track smoothing:

  - If a tracked object disappears temporarily, its misses count increases. If it disappears for too long, its track is deleted.

DetectedObject publishing:

  - Confirmed tracks are converted into your custom DetectedObject messages and identified as TYPE_BODY with position, size, ID, and confidence

publish_markers():

  - Creates orange cylindrical RViz markers at the locations of confirmed buoys so you can visually inspect what the detector believes is a buoy.

main():

  - Starts ROS 2, creates the BuoyDetector, keeps it running with rclpy.spin(), and cleans up when you press Ctrl+C.
