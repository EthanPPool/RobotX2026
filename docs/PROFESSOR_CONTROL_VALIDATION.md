# Validation

32 new Python unittest cases pass. They cover:

- Continuous coefficient order and actual-minus-desired signs, exact forward
  recurrence versus independent expanded notebook right/left expressions, nominal
  equilibrium, finite/time validation, output limits and no hidden windup.
- Causal pose derivatives with unequal capture intervals, measured-twist mode,
  yaw wrapping, path-frame rotation invariance, cross-track guidance and local
  model-domain rejection.
- Three-reference validation, C2 interpolation, timestamp skew/freshness, source-bounded force expiry, datum
  mismatch, HOLD/invalid references, pose-only shared odometry, mission resets and
  invalid zero-force output.
- Independent motor calibrations, axis-sign mapping, verified mixer inversion,
  reverse/neutral behavior, optional measured RPM/torque fields and refusal of
  uncalibrated, out-of-range or malformed data.
- Real vehicle-bridge methods with small ROS transport stubs: software stop,
  autonomy stop, wrong mode, disconnection, operator takeover, battery/state/force
  freshness, invalid commands, source replay/restart, dashboard authority fields,
  neutral/HOLD behavior and the existing Rover y/z manual mapping.
- A ROS callback pipeline from three preview samples and measured pose history
  through the professor force requests to calibrated per-motor/mixer output.

The legacy vehicle refactor was checked structurally against the exact current
GitHub blob. All non-update methods are unchanged. The old autonomous velocity
output block was moved intact into one overridable hook, after unchanged authority
checks. No preflight, arming, FCU or operator safety check was removed.

The combined install is checked against original-main, v3 and v4 planner fixtures:
preview-only behavior, application, repeat/idempotent application, conflict refusal
with all existing bytes preserved, and git apply --check of the original-baseline
patch. The installed-fixture regression suites pass 69 planner and 56 perception
checks in addition to the 32 new control checks (157 total).

Python compilation, shell syntax, package XML parsing, package entry-point/source
presence and payload hashes pass. The calibration template is intentionally
unusable until measured data and verified conventions are supplied.

This environment has no ROS 2 installation. No actual colcon build, generated
message/DDS communication, launch execution, autopilot mixer identification,
physical thrust calibration, closed-loop stability or on-water performance is
claimed. The professor's notebook does not establish a unit system or map-state
binding by itself; the proposed local-tangent guidance and causal derivative
realization remain explicit review items. Those flags default false and the
hardware bridge is absent from the default launch. No boot service was changed.
