# boat_mission

`boat_mission` owns mission decisions. It does not generate MAVROS velocity setpoints and it does not change ArduRover modes.

Task 1 production flow:

```text
/perception/gates
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
- transforms observations using the local pose at their original measurement timestamp;
- selects the nearest eligible gate ahead and associates further observations with its fixed acquisition geometry;
- retains the gate identity when it temporarily disappears, stopping in TRACK_GATE after the measurement timeout;
- excludes completed gates from subsequent selection;
- commits the gate only with fresh local pose and an ARMED + GUIDED vehicle;
- freezes both gate posts and constructs a fixed target beyond the gate;
- detects actual sign change through the saved gate plane;
- verifies the crossing occurred inside the usable gate corridor;
- requires `pass_clear_distance` beyond the gate before incrementing the gate count;
- publishes `WAIT_GATE_2`, `TRACK_GATE_2`, and `MISSION_COMPLETE` states expected by `boat_vehicle`.

The mission publishes `boat_interfaces/msg/NavigationTarget` in `base_link`. `boat_control/target_controller` is the sole producer of `/control/cmd_vel`, and `boat_vehicle/mavros_command_bridge` is the sole producer of the MAVROS propulsion setpoint.

See [gate selection and validation](../../../docs/gate_selection.md) for candidate topics, tuning, timestamp requirements, and rebuild instructions. The legacy `/perception/gate` subscription requires `use_gate_candidates: false`; it still enforces gate identity and measurement freshness.
