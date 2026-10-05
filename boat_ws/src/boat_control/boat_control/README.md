# boat_control

`boat_control` is intentionally mission-agnostic.

The production Task 1 interface is:

```text
/mission/target (boat_interfaces/msg/NavigationTarget)
        -> target_controller
        -> /control/cmd_vel (geometry_msgs/msg/TwistStamped)
```

The controller owns only body-frame guidance conversion:

- validate target freshness and finiteness;
- compute heading error from the base_link target;
- apply proportional yaw control with a configured yaw-rate limit;
- allow forward motion only inside `forward_angle_limit_deg`;
- clamp requested forward speed to `max_forward_speed`;
- fail to zero velocity for stale, stopped, invalid, or behind-vehicle targets.

Mission sequencing, gate geometry, gate counting, and gate-passage detection belong to `boat_mission`. MAVROS modes, propulsion authority, E-stop, operator takeover, and command deadman behavior belong to `boat_vehicle`.
