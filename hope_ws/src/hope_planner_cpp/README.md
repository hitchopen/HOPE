# hope_planner_cpp

Deterministic C++17 Planner for the `model_21800` hardware contract. This is
the only supported ROS Planner runtime for production, Foxglove and Gate 3.
The ROS production path contains a non-recursive, bounce-aware batch physics estimator, the
Stage 2 trajectory predictor, the complete Stage 3 racket solve, and revisioned
Schema-2/Schema-3 publishers. Schema 2 remains exactly 19 doubles; Schema 3
preserves fields 1..18 and appends `reach_level`/`swing_foot_sign`. It contains
no EKF, covariance, Kalman gain, or chi-square admission logic.

In production, `hope_ball_flight_packetizer` consumes `/poses/planner_full`
from either the VRPN or NatNet relay. Every original ball exposure enters the
rolling 180 ms window (about 54 samples at 300 Hz); the 10 ms preview cadence
does not decimate that input. At least 24 samples spanning 80 ms are required.
The exposure stamps are preserved; a gap over 20 ms starts a fresh fit segment.
The expected 300 Hz rate is an input contract, not proof of the camera's live rate.

A detector-only pre-roll identifies incoming motion and the opponent turnaround.
After net crossing, the packetizer maintains the current ball state and previews
the future hit. Unobserved table contact uses flight/contact/flight propagation;
a measured post-contact state predicts the remaining flight. The production
`batch_single_flight_only` fit never includes a table impulse: a pending reversal
is temporarily unfit, then samples through contact+8 ms are excluded and only
the latest post-contact segment is fitted. Insufficient post-contact history
never falls back to a mixed pre/post-contact fit. Only the raw samples actually
fitted are put in the immutable packet, so the HDU independently reproduces a
single free-flight fit.

Submission is scheduled by `predicted_hit - policy_lead - transport_budget -
solve_budget - policy_tick`, with production defaults 350+30+5+20 = 405 ms. There is no
net+50 ms baseline, post-bounce early send, or net+500 ms cap in this mode.
The latest valid prediction can move the pending deadline in either direction;
invalid partial-contact fits and source stalls cannot retime it.
Immediately before submission the latest exposure is
re-evaluated even if it is between preview ticks. A still-fresh last valid
pure-flight candidate can bridge contact warmup; a source age over 50 ms cannot
be emitted on a timer. Confirmed outgoing reversal cancels the pending flight.

The operator-selected transport budget is 30 ms, an allowance rather than a
Wi-Fi latency measurement. `config/transport_30ms_candidate.yaml` retains the
same value as an explicit override. Packet-to-actor tail latency has not yet
been field-qualified for this budget.
The policy lead remains explicit and must match the loaded model's trained
actor-entry support; `step_dt=20 ms` supplies the separate policy tick allowance.
These changes do not enable mid-swing target revisions or alter Runner admission.

`hope_planner_cpp_node` runs one estimator/Stage-2/Stage-3 solve per immutable
flight. The packet is retransmitted at 0/5/15 ms and deduplicated on the HDU.
Both packet endpoints use RELIABLE, KEEP_LAST(8), VOLATILE, and a 100 ms
lifespan. The laptop hands writes to a bounded worker so DDS backpressure does
not stall mocap callbacks. Both original freeze and final observation times
must remain within 100 ms at send, receive, and dequeue; retransmitting does
not renew them. Rejected late transport is audited as `stale_transport` and
does not change the ball play state. The managed packet endpoints additionally
use asynchronous writes and 10 ms heartbeats via
`with_fastdds_unicast.sh --flight-packet-qos-topic /ball/flight_packet`.
Transport message size remains at the established default so discovery and
traffic from existing participants retain compatible receive buffers.
The model14600 configuration mirrors the production scheduling profile.
`adaptive_commit_enabled: false` retains the explicit legacy fixed-net-delay
comparison path. Direct `/poses` Planner input remains a legacy A/B mode.

The net crossing is task-phase bookkeeping. A mathematically invalid one-shot
is logged as that flight's sole result; it is not retried. Expired strike
deadlines are rejected. The horizontal angle is measured from the robot's
current heading to the strike point. The shared production profile now uses
`max_target_turn_deg: 180.0`, disabling angular rejection at the operator's
request. The earlier 40 degree limit can be restored explicitly in the profile;
the C++ default remains 40 degrees. Unavailable or stale base poses are still
rejected. This parameter is read at startup, so changing the YAML requires a
controlled Planner restart.
There is no additional confidence, stability-frame, READY, or balance gate.

Audit CSV creation is exclusive and refuses to truncate an existing attempt.
There is no Planner process-lock gate; operations should still start one
publisher per field attempt so its evidence has unambiguous ownership.

The Ball quaternion supplies a causal angular-velocity estimate. The
`model14600_spin_contact.yaml` overlay enables a shared future-contact
predictor on both Laptop and HDU: valid spin with coherence >= 0.4 uses
`tangential_gain=0.348260`, effective `omega_scale=2/3`, and the existing
vertical restitution 0.96. Missing or incoherent spin retains the original
0.20/0.96 effective zero-spin model. Magnus remains shadow-only. This changes
only the single pre-engagement prediction, never an active swing's target or
clock. Audit columns record whether the spin model was selected and its
parameters; the original spin measurement remains unscaled in the log.

Selected on three older packet datasets, this contact model improved the
held-out 2026-10-10 21:21 PDT recording's 20 measurable crossings from
33.70 to 27.21 ms median time error, 2.34 to 1.82 cm lateral error, and
4.02 to 3.85 cm height error. The latest continuous callback replay retained
all 48 flight commits exactly once. These are prediction measurements, not
a physical hit-rate certification. Evidence, original packet inputs, and the
shared-core replay source are in `artifacts/planner_repair_20261010_212143/`.

The optional-reach classifier is a deterministic mirror of the screened
training-bank labels. It consumes only the frozen target world Y, swing side,
producer TTS, and the first valid solve-time base Y latched as immutable
session HOME. Startup base packets do not establish HOME. It never reads foot
pose, contact, load, or outcome. Disabled mode is an explicit zero-only
contract; invalid packets always carry `(0, 0)`.

## Policy support and swing-side hysteresis

Trajectory prediction no longer rejects a descending interception solely because
its table-relative height is below 5 cm. The legacy and spin-aware predictors
retain contact ordering and bounce physics; valid low crossings remain available
to both the adaptive packetizer and the Planner. Deploy both executables when
updating this behavior.

Set `swing_side_fh_y_range` and `swing_side_bh_y_range` to the selected
policy's `[min, max]` lateral support bounds in the configured reference frame.
If only one side supports an intercept, that side overrides the previous swing;
hysteresis applies only inside overlapping support. Targets outside both bands
produce an invalid command. Both bands default to empty for legacy selection.
With zero hysteresis, the split itself always selects backhand.

For schema 4, retain the native Runner-owned session HOME protocol and
`swing_side_reference_mode=session_home_v1`. The support bands use that HOME.
This change does not enable the optional bands automatically or modify policy
metadata, Runner reach checks, or the hardware profile.

For legacy schema-2/3 integrations, `swing_side_reference_mode=runner_home_v1`
accepts `std_msgs/Float64MultiArray` on `/hope/runner/home_anchor_flat` with
`[version=1, valid=1, home_x, home_y]`. The integration must supply this publisher
from the Runner's immutable MOTION-entry anchor and publish `valid=0` outside
that session. Missing or invalid HOME prevents a valid strike. This optional
adapter avoids using a drifted first-solve base as the policy's HOME; it is not
a replacement for the schema-4 session protocol, and combining them is rejected.

The legacy adapter also accepts `x_hit_home_support_range=[min, max]` for a
shared policy X interval. The requested fixed interception plane is projected
inside the interval about Runner HOME with a 1 micrometre margin; interior
planes are unchanged. The same supported plane is published to the adaptive
packetizer. This requires `x_hit_follow_robot=false` and zero `x_hit_bh_delta`.
It changes the interception plane, not the policy's support bounds. HOME and
the resulting plane are logged. No default plane or wire schema is changed.

Gate3 accepts fixed-delay and adaptive Stage-2 timing without requiring a
50 ms commit delay or a single calibrated bounce gain. It still checks commit
ordering, finite values, gain bounds, flight identities, packet integrity,
freshness, logger/input loss, and resolved physical-spin parameters.

## Local build and tests

The commands below assume the ROS-equipped `hope` container already exists.
On a new machine, create it using
[`docs/DISTROBOX_SETUP.md`](../../../docs/DISTROBOX_SETUP.md) first.

```bash
distrobox enter hope
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install \
  --packages-select hope_msgs hope_planner_cpp \
  --cmake-args \
    -DPython3_EXECUTABLE=/usr/bin/python3 \
    -DPYTHON_EXECUTABLE=/usr/bin/python3
colcon test --packages-select hope_planner_cpp
colcon test-result --verbose
```

After sourcing the workspace, the supported Planner launch loads the hardware
settings, full return-contact settings, and selected PPQ004 ball profile:

```bash
source install/setup.bash
ros2 launch hope_planner_cpp model21800_hardware.xml
```

For a direct node invocation, supply all three files in that order:

```bash
PLANNER_CONFIG="$(ros2 pkg prefix --share hope_planner_cpp)/config"
ros2 run hope_planner_cpp hope_planner_cpp_node --ros-args \
  --params-file "$PLANNER_CONFIG/model21800_hardware.yaml" \
  --params-file "$PLANNER_CONFIG/return_physics.yaml" \
  --params-file "$PLANNER_CONFIG/ball_physics_ppq004_20261020.yaml"
```

`return_physics.yaml` supplies the ball radius, gravity, inertia, paddle contact
coefficients, and return solver settings. A base hardware file alone does not
configure the full return model. Select another measured ball profile explicitly
with the launch argument `ball_profile:=/absolute/path/to/profile.yaml`.

The shared hardware configuration is `config/model21800_hardware.yaml`; it
deliberately remains Schema 2 with optional reach disabled so legacy
model21800/V17 consumers are not silently upgraded. A Schema27 112-D artifact
must explicitly layer `config/schema28_optional_reach.yaml` after the shared
profile. The overlay enables deterministic training-screened labels; it is not
trajectory, deployment, HDU ARM, or field qualification evidence. The current
profile values alone do not establish which parameters a deployed process uses
or qualify its prediction accuracy.

The default selected ball profile is
`config/ball_physics_ppq004_20261020.yaml`; its YAML values supply the drag,
Magnus, and table-contact coefficients. The field hardware file also contains the PPQ004 values; explicit profile
overrides retain precedence. Estimation and future
prediction use Magnus plus the same capped slip/angular-impulse table model.
Both Laptop packetizer and HDU Planner configurations must be deployed; Gate3
resolves the Planner YAML and selected overlay for its nodes and MuJoCo plant.
`config/ball_physics_real_20261010.receipt.json` records the historical adoption,
not a live status check. The older C3D receipt remains historical provenance.

See [ball-physics calibration](../../../docs/operations/ball_physics_calibration.md)
for required measurements, spin validity, fixed/unidentified parameters and
consumer verification. Stage 3 now carries the predicted strike spin through
the full paddle contact law, including exponential normal restitution, capped
tangential impulse, and angular impulse. It integrates drag, Magnus, and gravity
with RK4 to the first descending ball-center crossing at one radius above the
table surface. The solve must converge in both landing XY and flight duration;
configured alternative flight times are checked against the same model and the
net height plus ball radius and clearance margin. Paddle coefficients remain the
existing venue fit, loaded from `return_physics.yaml`. Adoption does not establish
improved real hit rate or a passing Gate3 certification.

The full return solver is ported from [HOPE PR #86](https://github.com/hitchopen/HOPE/pull/86)
at `d3d3fd094ca741d64ae77dc6fc5696290c17eea3`. Its training/reward changes are
outside this runtime port. A usable orientation estimate supplies incoming spin;
missing orientation remains a zero-spin prior, not a measured zero-spin ball.
The runtime hit-speed multiplier still applies after the inverse solve; predicted
landing is the model target for the unscaled command, not a guarantee for a
non-unit multiplier or imperfect policy tracking.

## Offline replay

Replay defaults load the installed PPQ004 profile, `return_physics.yaml`, and
`model21800_hardware.yaml`. Override their locations with
`--ball-physics-yaml`, `--return-physics-yaml`, and `--planner-settings-yaml`.
Without explicit contact overrides, both observed and predicted incoming table
contacts use the selected ball profile. Missing orientation uses a zero-spin
prior; available spin and subsequent contact-generated spin reach Stage 3.

```bash
hope_planner_cpp_replay \
  --input /path/to/laptop/mocap_raw.csv \
  --output /tmp/model21800_cpp_replay.csv \
  --window 0.18 \
  --min-span 0.08 \
  --huber-delta 0.003 \
  --recency-half-life 0.0 \
  --drag-k 0.119006 \
  --restitution-h 0.651740 \
  --restitution-v 0.916598 \
  --table-tangential-gain 0.369 \
  --spin-mode venue-grip \
  --control-zero-spin \
  --post-net-one-shot \
  --post-net-delay 0.05 \
  --post-net-future-bounce-tangential-gain 0.20 \
  --post-net-future-bounce-restitution-v 0.96 \
  --incoming-opponent-side-margin 0.05 \
  --incoming-speed-threshold 0.25 \
  --outgoing-speed-threshold 0.25 \
  --incoming-direction-fit-samples 4 \
  --incoming-direction-confirmations 2 \
  --incoming-pre-roll-samples 24 \
  --incoming-source-gap-reset 0.25 \
  --adaptive-horizon \
  --net-x 1.37
```

This CLI example deliberately retains the **historical** fixed-50-ms, zero-spin
comparison coefficients for incoming prediction. Stage 3 still uses the full
return solver, so this command does not reproduce the historical Stage 3
numerics. The adaptive deployment
validation, including recorded callback arrival times and timer polling, is
retained in `artifacts/adaptive_commit_model14600_20261010/run_replay.py` at the
repository root.

`--control-zero-spin` reproduces the earlier zero-spin hardware profile: it keeps
the venue table-contact law while preventing orientation-derived spin from
changing the strike. The spin-contact profile is instead replayed with the shared
`FutureContactPredictor` in the artifact replay harness above; this CLI alone
does not reproduce its conditional fallback or adaptive commit scheduling.

In that historical comparison, the estimator retains the venue contact coefficient `0.369` for a bounce that
has actually been observed. The one-shot predictor uses the separately audited
effective coefficient `0.20`, with vertical restitution `0.96`, when it must predict a future table contact
from pre-bounce data. This split avoids changing post-bounce state fitting to
compensate for the zero-spin causal prediction model. These effective values
are not a refit of material friction. The hardware candidate
also used the adaptive Stage-2 horizon, capped at `3.0 s`. The current profiles
instead use the unified fitted table law described above; do not copy these
historical overrides into a current physics qualification run.

`scripts/compare_cpp_python_planner.py` retains the historical simplified Python
Stage 3 oracle and does not qualify the full return model. The C++ `test_trajectory_and_stage3` suite checks the full return solve,
spin compensation, invalid coefficients, and failed convergence. Python remains
an offline reference and is not part of the C++ ROS runtime.

`scripts/audit_model21800_replay.py` associates causal revisions with measured
crossings and scores fixed prefix candidates. Its pass/fail result is an
offline deployment decision, never a runtime gate.

`scripts/audit_bounce_transition.py` uses the existing canonical C3D exports
and reconstructed contact labels to compare post-bounce recovery and velocity
error with the old reset implementation. Contact segmentation diagnostics are
offline evidence only.

`scripts/audit_spin_observability.py` checks Ball quaternion coverage, sign
equivalence, possible marker relocks, gaps, and incoming-flight angular-rate
statistics.  Its run definitions and all quality values are audit-only.

## Qualification boundary

### Contact-observation experiment (model14600)

`config/model14600_contact_observation_candidate.yaml` is an **inactive,
experimental overlay**. Apply it after the normal configuration to the
OptiTrack relay, packetizer, and Planner together. The relay publishes fresh
ball exposures on `/poses/planner_full` before its existing 150 Hz output
gate. It preserves source timestamps and the standard `/poses` output. The
packetizer retains a full raw window in its immutable packet; both Laptop and
HDU independently fit the measured post-contact arc after at least 40 ms,
12 samples, and an 8 ms contact exclusion. The nominal bounce coefficient no
longer constrains that outgoing fit. Before it is mature, the historical fit
and one-command-per-flight scheduling remain available. Setting
`batch_post_bounce_min_span_s=0` retains the previous estimator behavior.
No target or clock is revised after Runner engages.

The October 10 three-recording replay has mixed results: the full-rate
candidate improves lateral/height medians in the latest record but regresses
on earlier recordings. It is **not the selected hardware configuration**.
Replays, alternatives, native tests and isolated continuous Gate3 runs are in
`artifacts/contact_observation_fix_model14600_20261010/`. Complete-sampling
replays approximate new callback times with raw bag receipt plus 1 ms; they do
not establish actual live callback latency or hardware hit rate.

Gate3 receipts now include `prediction_vs_simulated_contact`. The deployed
effective future gain of 0.20 means 0.80 horizontal retention (zero spin,
inactive friction cap), whereas the measured MuJoCo material uses 0.651740.
Their vertical values also differ: 0.96 versus 0.916598. Sharing one YAML does
not make those laws identical. The explicit
`config/model14600_mujoco_material_match_diagnostic.yaml` profile removes
that compensation for a simulation-only comparison while leaving the plant
unchanged. This profile is not a new real-ball fit and must not replace the
hardware profile based on a simulation score.

Local x86 timing does not establish HDU ARM timing, DDS callback retention, P1
correctness, or physical robot behavior. The field runbook documents how to
build, run, and collect evidence for this candidate; that documentation is not
a qualification claim. Treat the candidate as unqualified until the planned
HDU ARM replay and supported-robot field test have passed.

### PPP3/PPP4 ball y filter and Foxglove Ready controls

`model21800_hardware.yaml` enables the filter by default with the P1 interval
`[-1.30, -0.5543593239924284]` metres. The operator adjusted the right boundary
to -1.30 on 2026-10-22; the left boundary retains its measured PPP3 value.
`config/ball_y_filter_ppp3_ppp4_20261015.receipt.json` records the raw topics,
sample counts, measurement time and mm-to-m conversion. These are venue
coordinates, not subscriptions that follow moved markers. Remeasure after
moving markers or changing the table frame; P2 requires transforming both y
bounds to its canonical frame.

The Foxglove Ready card offers enabled, left/maximum y, right/minimum y, Apply,
and Restore defaults. Draft edits update only a local 3D preview, not admission.
The displayed applied range comes from fresh Planner receipts. Apply calls only
`/hope/planner/set_ball_y_filter` (`rcl_interfaces/srv/SetParametersAtomically`)
with exactly `enabled` (bool), `min_m` (double), and `max_m` (double). Unknown,
duplicate, missing, nonfinite or reversed fields reject the entire request.
Acknowledgement waits for previous publication to finish. Subsequent commands
use the new range, without cancelling an already engaged swing.

Hardware saves accepted settings atomically to
`/var/lib/hope-lifecycle/planner_y_filter.state` and reloads them at startup.
The file is versioned text: `1 enabled min_y_m max_y_m`. A failed save leaves
live settings unchanged; corrupt saved state prevents startup. Empty
`ball_y_filter_state_file` makes changes session-only for tests/simulation.
Startup defaults are read-only ROS parameters; runtime state is authoritative
in `/planner/diagnostics`, including persistence and configuration support.

Both the latest observed ball y in the consumed trajectory/flight packet and
the predicted contact y must be inside the inclusive interval. Rejections
publish `valid=false` on flat and typed outputs, without clamping targets.
CSV audits include the applied bounds and rejection reason.

`/hope/planner/y_filter_markers` publishes five world-frame ROS markers: two
thin orange bars along the tabletop x direction, two numeric y labels, and
a faint green lane clipped to the tabletop. Disabled bounds are gray with
OFF labels and no fill. Markers expire after three seconds without Planner
updates. The shipped console/control layouts enable this topic; existing
custom 3D layouts must enable it once. Draft and disconnected UI states never
claim to be the applied lane. The filter limits lateral strike admission and
is not a physical balance guarantee.

### Body pose timing audit

`planner body health` is written every second even with recording OFF and no
ball input. It separates monotonic receipt age/gap from wall-clock source age,
source age at receipt, and source timestamp gap; absent timestamps are `nan`.
Wall-clock source age includes host clock offset and only covers the portion
of the pipeline represented by the incoming timestamp (the current VRPN client
stamps on Laptop reception, not camera exposure). Diagnostics and solve CSVs
also include body counts, invalid counts, maximum gap, and timing counters.

`target_turn_reject_stale_pose` defaults to `true`. The field hardware profile
explicitly sets it to `false` at the operator's request: crossing
`target_turn_max_pose_age_s` (200 ms) is then logged and counted, but does not
itself reject a target. Pose validity, availability and the other target checks
still apply. `base_pose_age_exceeded_commands_total` counts otherwise
geometry-admissible commands crossing either age threshold, once per command;
`base_receipt_stale_rejections_total` and `base_source_stale_rejections_total`
count actual rejections and stay zero in audit mode. Compare these with
`target_turn_evaluations_total`; they are not per-frame or time-fraction metrics.
The setting is read at startup. Do not restart only Planner inside a surviving
schema-2 Runner session: its sequence counter resets and Runner can reject the
new sequence as reordered. Use a complete managed lifecycle restart when the
robot session is active.

### Runtime hit speed scale

`hit_speed_scale` defaults to `1.0` at startup. The dedicated
`/hope/planner/set_hit_speed_scale` (`rcl_interfaces/srv/SetParametersAtomically`)
accepts exactly one `scale` double, finite and nonnegative, while Planner is running.
It scales all three components of the planned racket velocity before command
admission, flat/typed publication, and CSV logging. Position, normal, deadline,
flight identity, and the original Stage 3 landing solution are unchanged.
The outgoing-ball/net/landing predictions still describe the unscaled solution.
Already published one-shot flights and their retransmissions remain immutable.
Exact question-bank fixtures lock the scale to 1 and ignore saved settings. When
`hit_speed_scale_state_file` is configured, updates are saved atomically before
acknowledgement and restored on restart. An invalid saved file fails startup;
a failed write leaves the live value unchanged. Without a saved file, the startup
parameter (default 1) applies. Hardware config uses
`/var/lib/hope-lifecycle/planner_hit_speed_scale.state`. Diagnostics report
`hit_speed_scale_persistent`, `hit_speed_scale`,
`hit_speed_scale_configurable`, and `hit_speed_scale_last_command`; the last value
is NaN until a valid target is published. CSV rows include their applied scale.

### Planner restart and wire identity

Hardware sets `wire_identity_state_file` to a persistent local file. Before
publishing, each Planner process reserves a disjoint range of command and flight
IDs under an exclusive lock and atomically persists its upper bound. Restarting
only the Planner therefore does not reset the IDs observed by the live Runner.
The reservation also survives wall-clock rollback; no disk work is added to the
per-ball solve. Keep this file when updating or restarting the Planner. A missing
or corrupt reservation cannot be repaired by resetting the live Runner's replay
protection. Duplicate/reordered commands and mismatched HOME frames remain invalid.

### Unknown spin before a future table contact

The hardware Planner and Laptop packetizer explicitly set
`future_contact_require_spin=false`: uncertain spin must not suppress a shot or
consume extra preparation time waiting for a more coherent orientation estimate.
The library default remains strict for callers that explicitly need that mode.

Position and velocity still come from the latest continuous 180 ms window with
three Huber-weighted physical-fit iterations. This combines many observations
while reducing outlier influence; it does not average across a bounce or outage.
Reliable spin is used as measured. With missing or incoherent spin, prediction
uses the estimator's physical prior (zero incoming spin when none is observed,
retaining spin generated by an observed contact). This is an assumption for
best-effort execution, not evidence that the ball has no spin. Coherence and
`future_contact_spin_used` remain in the audit so the approximation is visible.
Table friction, restitution, spin impulse, Magnus and racket-contact physics
remain unchanged. No extra candidate search or waiting is introduced, and the
405 ms preparation reserve is unchanged.

Setting `future_contact_require_spin=true` restores strict rejection of an
unobserved future contact as `future_contact_spin_unobserved`. In strict mode the
adaptive packetizer waits for usable spin or a clean measured post-bounce arc,
and invalidates an unresolved saved deadline. Free flight remains available.
Both producer and receiver must use the intended setting; this parameter is
read at startup, so changing its ROS parameter value alone does not update the
constructed predictor. The normal finite-data, deadline, frame and policy
reachability checks still apply. Neither mode establishes physical hit accuracy.

## Operator ball-state gate (PR #119, Console 1.9.3)

The operator-side C++ packetizer classifies normalized P1/P2 ball tracks and
filters both ball channels. Local Ready arms OPPONENT_SERVE; local Serve arms
SERVE; local Stand/Stop holds the gate closed until explicit Ready/Serve.
Runner feedback does not control this classifier. Pelvis/base traffic remains
independent. Forecasts are advisory and cannot reject a measured trajectory.
A held ego serve requires explicit local Ready before a fresh opponent serve.

Install the matching packetizer, bringup launch files and Console 1.9.3.
Foxglove Desktop requires one-time pairing with the runtime's private
`hope-ball-controls.key`; status is at `http://127.0.0.1:8768/ball-state`.
See [state rules, installation, tests and rollback](../../../docs/operations/laptop_ball_states.md).
This operator-only integration preserves the field scheduling and physics
settings and changes no HDU/MDU service. It cannot recall a previously sent target.

### Runtime return flight time

`/hope/planner/set_flight_time` (`rcl_interfaces/srv/SetParametersAtomically`) accepts
exactly one `flight_time_s` double in `(0, max_predict_time]`. It overrides the
Stage 3 desired strike-to-landing duration for both sides on subsequent new targets.
Before the first override, `delta_t_flight` and finite `delta_t_flight_fh/bh` retain
their startup semantics. An override is serialized from Stage 3 input selection
through publication: after acknowledgement no old-duration solve can publish.
Already committed one-shot flights and duplicate packets remain unchanged.

The solver retains `return_alternative_flight_times`; the requested duration is
not guaranteed. `flight_time_s`, `flight_time_fh_s`, `flight_time_bh_s`,
`flight_time_max_s`, `flight_time_configurable`, and `flight_time_persistent`
diagnostics describe the current settings. `flight_time_last_requested_s` and
`flight_time_last_command_s` describe the last valid published target's request
and predicted duration respectively (NaN before a valid target). The prediction
is before hit-speed scaling, not measured flight time. CSV adds
`flight_time_requested_s` and `return_flight_time_s` without changing wire schemas.

`flight_time_state_file` enables atomic save-before-acknowledgement and restart
restoration; hardware config uses `/var/lib/hope-lifecycle/planner_flight_time.state`.
Invalid saved state fails startup; write failure leaves the active value unchanged.
Exact question-bank fixtures ignore saved overrides and reject live updates.
