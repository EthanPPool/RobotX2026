# Mission Diagnostic Logger

The Jetson-local `mission_logger` records Task 1 state, perception, navigation, control, vehicle-bridge, and MAVROS telemetry.

After the split architecture, diagnostics are sourced from two independent layers:

- `/mission/diagnostics`: mission phase, gate geometry, map-frame passage geometry, crossing status, and mission target geometry.
- `/control/diagnostics`: target freshness, heading error, forward-angle decision, and the generated `/control/cmd_vel` values.

The logger merges those two streams into the existing controller/follower diagnostic CSV fields for backward compatibility with existing logs and analysis tools. `/control/cmd_vel` is still logged independently as the actual controller output, and `/vehicle/control_diagnostics` remains the authoritative bridge/propulsion-authorization diagnostic stream.

The logger also records `/mission/state`, `/perception/gate`, MAVROS state/pose/velocity/IMU/GPS/battery data, operator commands, and authorized MAVROS velocity output.

The architecture is intentionally layered:

```text
mission diagnostics  -> mission decision / geometry
control diagnostics  -> NavigationTarget -> Twist conversion
bridge diagnostics   -> propulsion authorization / vehicle mode
```

A nonzero mission target does not prove that the controller produced forward velocity, and a nonzero `/control/cmd_vel` does not prove that the vehicle bridge authorized propulsion. Inspect all three layers when diagnosing a water test.
