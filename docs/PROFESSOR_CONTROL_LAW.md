# Notebook transcription and coordinate contract

Source: Yellow_Robot_Boat_Control_Law.nb, supplied by the user. Its title states
"two Stern drive variable thrust fixed axes thrusters". Ft3 is right/starboard;
Ft4 is left/port. The continuous equations and discrete outputs are preserved.
The source notebook does not contain the upstream physical derivation, a unit
system declaration, motor curves, force-axis directions, or a definition binding
x and q3 to a particular map frame. Those cannot be proved from coefficient values.

Let ex = x - xd and eq = q3 - qd3, exactly the notebook's error signs:

    Ft3_dot = 3409.834796259714 ex
            + 2238.596695925194 ex_dot
            + 719.8053330781249 ex_ddot
            + 9.519074990274795 eq
            + 29.856690797946126 eq_dot
            + 17.040946843853824 eq_ddot

    Ft4_dot = 3409.834796259714 ex
            + 2238.596695925194 ex_dot
            + 719.8053330781249 ex_ddot
            - 9.519074990274795 eq
            - 29.856690797946126 eq_dot
            - 17.040946843853824 eq_ddot

These return force RATE. The live integrator advances Ft(t+dt)=Ft(t)+dt*Ft_dot.
It initializes at the configured nominal trim and stores the limited force state,
so a saturated request does not accumulate hidden windup. Force/rate limits are
engineering additions outside the unchanged equation kernel. Default limits and
trim are provisional shadow-review values, not measured boat parameters.

If x is meters, q3 radians, time seconds and Ft newtons, coefficient dimensions
must respectively be N/(m*s), N/m, N*s/m and N/(rad*s), N/rad, N*s/rad. Check that
against the professor's derivation, including any earlier nondimensionalization.
Confirm positive Ft directions separately; do not reverse a feedback error sign
as a substitute for confirming the model's thruster axes.

## Exact discrete law versus live causal realization

law.forward_step implements the notebook's exact forward finite differences:

    D1 f = (f(t+h)-f(t))/h
    D2 f = (f(t)-2*f(t+h)+f(t+2h))/h^2

It accepts THREE actual samples and THREE desired samples of x and q3. Tests
compare it against an independent transcription of expanded Out[1124]/Out[1127]
for both motors and several step sizes. It is useful for replay or a validated
state predictor. Future desired points do not supply future actual measurements.

The live ROS node instead evaluates the continuous force-rate equations using
current measured position, causal estimated velocity/acceleration, and the timed
reference's velocity/acceleration. This is an explicitly different derivative
realization of the same continuous law, not a fabricated future actual state or
a claim that only current position suffices for the exact forward formula.

The default shared GPS adapter is pose-only and leaves Odometry.twist unset.
velocity_source=pose_history therefore uses three distinct past capture times,
quadratic differentiation and a configurable causal filter. Timestamp gaps reset
warmup. A verified fused estimator can use velocity_source=odometry_twist; its
linear velocity must be in the configured Odometry.child_frame_id, and the node
rotates it into the map. Yaw rate is body z for the supported near-level attitude.
Noise/filter delay must be assessed using real localization logs.

## Proposed path binding: separate from the professor's equations

The notebook has x and q3, not independent x/y force actuators. A two-fixed-axis
thruster boat cannot directly command sideways thrust. A separate guidance layer
projects map translation onto the nominal path tangent, frozen for each local
linearization, and treats q3 as yaw. This avoids applying global east-only error
to a northbound path. It preserves the notebook coefficient/sign pattern.

Cross-track error contributes a line-of-sight desired-heading correction:

    delta_yaw = -atan2(cross_track_error, lookahead)

Its first and second derivatives are included in desired yaw rate/acceleration
using the same frozen tangent. This guidance and coordinate binding are proposed
engineering choices; they are NOT present in the notebook. Confirm them against
the professor's definition of x/q3 and nominal path before marking the binding
confirmed. The local error limits withhold commands outside the intended small
perturbation domain. They do not establish closed-loop stability for this hull.

Nominal trim is currently a constant right/left force pair. Speed-dependent drag
feedforward and a time-varying nominal-thrust schedule are not inferred. If the
professor's nominal model requires such a schedule, supply it explicitly before
hardware use. Planner kinematics are not a hydrodynamic plant validation.
