Notes for POR Gate Mission

Imports:

- Loads ROS 2, math utilities, the mission-state enum tools, String messages, and your custom Gate and NavigationTarget messages.

MissionState enum:

- Defines the four possible mission states:
- SEEK_GATE
- APPROACH_GATE
- CLEAR_GATE
- COMPLETE

PorGateMission.init():

Sets up the ROS node and loads all tunable mission parameters. These parameters define:

-gate confidence requirements
-how long a gate must remain confirmed before moving
-how long a gate detection can go without updating before being considered stale
-approach and near-gate speeds
-how close the boat gets before considering itself at the gate
-how long and how fast the boat drives forward to clear a gate
-how many gates must be passed

Mission state variables:

-Stores the current mission state
-Tracks how many gates have been passed
-Stores the most recently detected gate
-Stores when that gate was last seen
-Stores when gate confirmation began
-Stores when the CLEAR_GATE phase began

Publishers and subscriber:

-Subs to --> /perception/gate
-Pubs to --> /mission/target and /mission/state

gate_callback():

-Runs whenever a new Gate message arrives
-Checks whether the gate detection is usable before allowing the mission to react to it

Gate confidence check:

-Rejects gates whose confidence is below min_gate_confidence
-Default minimum confidence is 0.60

Gate validity check:

-Rejects gates if the center coordinates are invalid
-Rejects gates if the gate center is behind the boat or directly at x <= 0

Gate storage:

-Saves the most recent valid gate
-Records the time it was received so the mission can check whether the detection becomes stale

Gate confirmation:

-The boat does not move immediately after the first valid detection
-When a gate is first detected in SEEK_GATE, a confirmation timer starts
-The gate must remain valid for gate_confirm_duration
-Default is 0.75 seconds

gate_is_fresh():

-Checks how long it has been since the most recent valid gate detection
-If the gate is older than gate_timeout, it is considered lost
-Default timeout is 0.60 seconds

update():

-Main mission state-machine loop
-Runs every 0.10 seconds, or 10 Hz
-Decides what navigation target should be published based on the current state

SEEK_GATE:

-Waits for a valid and fresh gate
-Publishes stop commands while waiting
-Requires the gate to remain detected for the confirmation period before moving
-Once confirmed --> switches to APPROACH_GATE

Fail-stop behavior:

-If the gate becomes stale while SEEK_GATE or APPROACH_GATE is active, the mission immediately:
-returns to SEEK_GATE
-clears the gate-confirmation timer
-publishes a stop command
-waits for another valid gate

APPROACH_GATE:

-Uses the detected gate center as the NavigationTarget
-The target is published in the base_link frame
-This tells the downstream controller to steer the boat toward the middle of the gate

Approach speed:

-Far from gate --> approach_speed = 0.40 m/s
-Near gate --> near_gate_speed = 0.28 m/s
-The boat switches to the slower speed when the gate center is within near_gate_x = 3.0 m

Gate pass threshold:

-When the detected gate center gets within gate_pass_x = 1.10 m
-The mission stops relying on the gate detector
-Changes state from APPROACH_GATE --> CLEAR_GATE

CLEAR_GATE:

-Commands the boat to continue straight forward so it physically passes completely through the gate
-Uses a target located straight ahead of the boat
-Default target --> x = 3.0 m, y = 0.0 m
-Default speed --> 0.35 m/s

Clear duration:

-Keeps commanding the forward CLEAR_GATE target for 5.0 seconds
-After 5 seconds, the current gate is counted as passed

Gate counting:

-Adds 1 to gates_passed after each CLEAR_GATE phase
-If more gates are required --> returns to SEEK_GATE
-If all required gates have been passed --> switches to COMPLETE

gates_required:

-Defines how many gates must be passed before the mission finishes
-Default --> 2

COMPLETE:

-Mission is finished
-Continuously publishes stop commands
-Prevents the boat from receiving another mission movement target from this node

publish_clear_target():

-Publishes a NavigationTarget directly in front of the boat during CLEAR_GATE
-Default:
-x = 3.0
-y = 0.0
-desired_speed = 0.35
-stop = False

publish_stop():

-Publishes a NavigationTarget telling the downstream controller not to move
-Sets:
-stop = True
-desired_speed = 0.0

publish_state():

-Publishes readable mission-status messages to /mission/state
-Also prints those messages to the ROS logger
-Examples:
-SEEK_GATE: waiting for gate 1
-APPROACH_GATE: confirmed gate 1
-CLEAR_GATE: crossing gate 1
-COMPLETE: passed 2/2 gates

main():

-Starts ROS 2, creates the PorGateMission node, keeps it running with rclpy.spin(), and cleans up when you press Ctrl+C.

Overall pipeline:

-Gate detector --> /perception/gate --> mission state machine --> /mission/target --> target controller

Mission flow:

-SEEK_GATE --> confirm gate --> APPROACH_GATE --> get close to gate --> CLEAR_GATE --> count gate --> SEEK_GATE again --> second gate --> COMPLETE
