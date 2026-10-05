# boat_mission

`boat_mission` owns mission decisions. It does not generate MAVROS velocity setpoints and it does not change ArduRover modes.

Task 1 production flow:

```text
/perception/gate
/mavros/local_position/pose
/mavros/state
        -> task1_gate_mission
        -> /mission/target
        -> /mission/state
        -> /mission/diagnostics
```

The current Task 1 mission preserves the latest fixed-frame logic previously embedded in `two_gate_follower`:

- rejects weak/non-finite gate detections;
- ignores repeated sample-and-hold gate messages by source timestamp;
- transforms each new gate observation from `base_link` into the fixed MAVROS local frame;
- tracks the gate midpoint from map-frame memory rather than requiring continuous LiDAR output;
- commits the gate only with fresh local pose and an ARMED + GUIDED vehicle;
- freezes both gate posts and constructs a fixed target beyond the gate;
- detects actual sign change through the saved gate plane;
- verifies the crossing occurred inside the usable gate corridor;
- requires `pass_clear_distance` beyond the gate before incrementing the gate count;
- publishes `WAIT_GATE_2`, `TRACK_GATE_2`, and `MISSION_COMPLETE` states expected by `boat_vehicle`.

The mission publishes `boat_interfaces/msg/NavigationTarget` in `base_link`. `boat_control/target_controller` is the sole producer of `/control/cmd_vel`, and `boat_vehicle/mavros_command_bridge` is the sole producer of the MAVROS propulsion setpoint.
