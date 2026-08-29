This node is a simple closed-loop guidance controller. Its job is to take a target point expressed relative to the boat and convert that target into two commands:

forward velocity
yaw rate

It does not directly control thrusters. It publishes a desired body-motion command on /control/cmd_vel

The pipeline can be visualized:

NavigationTarget ->
 target_controller ->
[TwistStamped
linear.x  = forward speed
angular.z = turning rate] ->
/control/cmd_vel

The controller is always asking:
1. Where is the target relative to the boat?
2. How fast should I move forward and turn to point toward it?

x-axis: Longitudinal axis, with +x being the space in front of the boat

y-axis: Transversal axis, +y being Portside and, -y being the Starboard side

z-axis: Axial axis, +z being up

class TargetController(Node):

  """Convert a base_link-relative point target into body-frame velocity commands."""

aka 

Input: point target relative to the boat

Output: forward speed + yaw rate
