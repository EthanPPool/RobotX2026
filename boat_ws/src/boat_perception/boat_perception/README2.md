Notes for Gates

Imports:
  
  - Loads ROS 2, math, pair-generation utilities, RViz markers, and your custom DetectedObject, DetectedObjectArray, and Gate messages.

GateDetector.__init__():

  - Initializes the node and loads all gate-detection parameters, including:
  - acceptable buoy confidence
  - minimum/maximum gate width
  - expected gate width
  - allowable front/back mismatch between buoys
  - acceptable gate distance
  - temporal confirmation settings

Publishers and subscriber:

  - Subs --> /perception/objects

  - Pubs --> /perception/gate and /perception/gate_markers

objects_callback():

Main processing loop. Each time detected objects arrive, it:

detected objects --> keep valid buoys --> find best buoy pair --> track that gate --> confirm over several frames --> publish gate

Buoy filtering:

  - Removes objects that:

  - are not classified as buoys

  - have low confidence

  - are too close

  - are too far away

find_best_pair():

  - Tests every possible pair of buoys to determine whether two buoys plausibly form a gate.

Gate width check:

  - Rejects buoy pairs that are too close together or too far apart.

Default acceptable range:

  - 1.2 m to 5.5 m

Depth difference check:

  - Checks whether the two buoys are approximately the same distance forward from the boat.

Lateral fraction check:

  - Ensures the pair is separated mostly left-to-right rather than front-to-back.

Gate center calculation:

Finds the midpoint between the two buoys:

left buoy ---- center ---- right buoy

That center becomes the point the boat can navigate toward.

Gate confidence scoring:

Scores each buoy pair based on:

  - 45% buoy detection confidence
    
  - 25% front/back alignment

  - 20% lateral alignment
  - 10% expected gate width

Best-pair selection:

  - If several buoy pairs could form gates, the pair with the highest confidence score is selected.

Left/right assignment:

  - Uses the buoy y positions to determine which buoy is on the left and which is on the right.

update_gate_track():

  - Tracks the selected gate over time. It checks whether the new candidate is close enough in:

  - center position

  - gate width

to be considered the same gate.

Track smoothing:

  - Uses track_alpha to smooth gate center, width, and confidence across multiple detections, reducing LiDAR jitter.

confirm_hits logic:

  - Requires the same gate to be detected repeatedly before publishing it.

Miss handling:
  - If the gate temporarily disappears, misses increases. If it is missing for too many updates, the current track is deleted.

start_track():

  - Creates a new gate track when a new candidate appears or when the detector decides it is looking at a different gate.

reset_track():

  - Clears all stored gate information after the gate has been lost for too long.

Gate publishing:

Once confirmed, publishes a Gate containing:

  - left buoy position
    
  - right buoy position
    
  - gate center
    
  - gate width
    
  - confidence
    
  - publish_empty_markers()
    
  - Removes old gate markers from RViz when no confirmed gate exists.
    
  - publish_gate_markers()
    
  - Draws the detected gate in RViz:
    
  - green line between the two buoys
    
  - cyan sphere at the gate center

main()

  - Starts the ROS node, spins it continuously, and performs clean shutdown on Ctrl+C.

  - Overall, this node sits immediately after your buoy detector:

LiDAR --> Buoy Detector --> /perception/objects --> Gate Detector --> find pairs of buoys --> check width + alignment --> score candidate gates --> track over multiple frames --> /perception/gate

