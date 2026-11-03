# hope_planner_cpp

The opt-in `prediction_backend: v2` now embeds the six-state CKF and shared
physics in this same node. It predicts post-bounce contacts even with unknown
game history, solves and reconstructs the paddle action, and checks its first
outgoing landing before emitting schema-2 **shadow** previews. Legacy remains
the default. See [integration contracts and migration](docs/integrated_planner_input_migration.md)
and [current verification and remaining gates](docs/integrated_planner_acceptance.md).
For the shared V2 incoming path, see the [local simulation, accuracy graphs and ROS/MuJoCo guide](docs/v2_local_simulation_guide.md).
The production description below describes the preserved legacy backend.

Deterministic C++17 Planner for the `model_21800` hardware contract. This is
the only supported ROS Planner runtime for production, Foxglove and Gate 3.
The ROS production path contains a non-recursive, bounce-aware batch physics estimator, the
Stage 2 trajectory predictor, the complete Stage 3 racket solve, and the exact
19-double schema-2 publisher. It contains no EKF, covariance, Kalman gain, or
chi-square admission logic.

In production, the separate `hope_ball_flight_packetizer` executable owns the
high-rate `/poses` callback and incoming-flight state machine. A small
detector-only pre-roll fits X velocity,
ignores the robot's outgoing flight, and backtracks a confirmed opponent return
to its X turnaround. Only that epoch's incoming samples enter the 180 ms
estimator history. The model14600 profile uses adaptive scheduling around the
`net crossing + 50 ms` baseline: short-lead balls commit earlier, while a ball
with sufficient preparation time can use at least 80 ms of clean post-bounce
observations. The scheduler reserves 350 ms for the policy and 100 ms for
transport; the latter is an allowance, not a bound on Wi-Fi latency. It uses
the calibrated foremost hit plane published by the HDU, with a conservative
fallback when that reference is unavailable. A wall timer closes the snapshot
even if mocap stops producing frames. A confirmed outgoing reversal cancels
an uncommitted incoming candidate. The packetizer freezes one immutable
snapshot, clears the active epoch, and publishes an immutable
`/ball/flight_packet`. `hope_planner_cpp_node` runs one
estimator/Stage-2/Stage-3 solve and publishes once. Direct `/poses` input remains
an explicit legacy A/B mode and is disabled by the production config.

`config/model21800_flight_packetizer.yaml` now enables the model14600 adaptive
profile (also retained as `config/model14600_adaptive_flight_packetizer.yaml`).
Set `adaptive_commit_enabled: false` in an isolated comparison profile to
restore fixed 50 ms scheduling. The immutable packet is retransmitted at
0/5/15 ms and deduplicated on the HDU; the target is never revised mid-swing.

The net crossing is task-phase bookkeeping. A mathematically invalid one-shot
is logged as that flight's sole result; it is not retried. Expired strike
deadlines are rejected. The horizontal angle is measured from the robot's
current heading to the strike point. The shared production profile now uses
`max_target_turn_deg: 180.0`, disabling angular rejection at the operator's
request. The earlier 40 degree limit can be restored explicitly in the profile;
the C++ default remains 40 degrees. Unavailable/invalid base poses are still
rejected. Pose age is a separate site option: `target_turn_reject_stale_pose`
defaults to `true`, preserving the Berkeley/main receipt and source age checks.
The local PPQ field setup uses `false`: measured ages remain in the audit CSV,
and exceeded ages emit a warning but do not veto a shot. Do not infer this option
from the ball model, operating system, or robot model. Use
`config/field_body_timing_audit.yaml` after the base parameter file, or retain
its single parameter in the installed site's `model21800_hardware.yaml` when
using the lifecycle helper. Both policies retain invalid-pose, strike-deadline,
and configured turn-angle checks. Parameters are read at startup; changing them
requires a controlled Planner restart. A source push does not change a running
Planner or a robot's installed site configuration.
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

## Local build and tests

The commands below use the Ubuntu 24.04/ROS 2 Jazzy `hope` container. On a new
machine, reproduce it using
[`docs/DISTROBOX_SETUP.md`](../../../docs/DISTROBOX_SETUP.md) first.

```bash
distrobox enter hope
source /opt/ros/jazzy/setup.bash
cd "$HOME/workspace/HOPE/hope_ws"
colcon build --symlink-install \
  --packages-select hope_msgs hope_planner_cpp \
  --cmake-args \
    -DPython3_EXECUTABLE=/usr/bin/python3 \
    -DPYTHON_EXECUTABLE=/usr/bin/python3
colcon test --packages-select hope_planner_cpp
colcon test-result --verbose
```

The shared hardware configuration is `config/model21800_hardware.yaml`; it
deliberately remains Schema 2 with optional reach disabled so legacy
model21800/V17 consumers are not silently upgraded. A Schema27 112-D artifact
must explicitly layer `config/schema28_optional_reach.yaml` after the shared
profile. The overlay enables deterministic training-screened labels; it is not
trajectory, deployment, HDU ARM, or field qualification evidence. The current
profile values alone do not establish which parameters a deployed process uses
or qualify its prediction accuracy.

The shared deployment profiles now contain the 2026-10-10 multi-batch real-ball
fit: `drag_k=0.12861635416914574`, `magnus_k=0.00379260504480258`,
`restitution_v=0.9313775047019153`, `table_tangential_gain=0.38744443855065164`
and `table_friction_cap_mu=0.16706204822993975`. Estimation and future prediction
use Magnus plus the same capped slip/angular-impulse table model. Both Laptop
packetizer and HDU Planner configurations must be deployed; Gate3 resolves the
Planner YAML and selected overlay for its nodes and MuJoCo plant.
`config/ball_physics_real_20261010.receipt.json` records the historical adoption,
not a live status check. The older C3D receipt remains historical provenance.

See [ball-physics calibration](../../../docs/operations/ball_physics_calibration.md)
for required measurements, spin validity, fixed/unidentified parameters and
consumer verification. Stage 3 outgoing-flight optimization still omits outgoing
Magnus; paddle parameters were not refitted. Adoption does not establish improved
real hit rate or a passing Gate3 certification.

## Offline replay

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
comparison coefficients; it is not the current Magnus/capped-contact deployment
replay. The adaptive deployment
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

`scripts/compare_cpp_python_planner.py` checks the migrated Stage 2 and Stage 3
numerics against the retained Python implementation. Python is an offline
oracle only and is not part of the C++ ROS runtime.

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
