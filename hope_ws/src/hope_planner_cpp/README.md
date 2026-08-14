# hope_planner_cpp

Deterministic C++17 planner candidate for the `model_21800` hardware contract.
The ROS production path contains a non-recursive, bounce-aware batch physics estimator, the
Stage 2 trajectory predictor, the complete Stage 3 racket solve, and the exact
19-double schema-2 publisher. It contains no EKF, covariance, Kalman gain, or
chi-square admission logic.

The high-rate `/poses` callback validates monotonic samples and owns the
incoming-flight state machine. A small detector-only pre-roll fits X velocity,
ignores the robot's outgoing flight, and backtracks a confirmed opponent return
to its X turnaround. Only that epoch's incoming samples enter the 180 ms
estimator history. At `net crossing + 50 ms`, the callback freezes one immutable
snapshot, clears the active epoch, and sends it through a one-slot latest-wins
mailbox. The solver runs one estimator/Stage-2/Stage-3 solve and publishes once.
The fixed SPSC ring remains only for the explicitly disabled one-shot legacy
mode. DDS QoS remains `KeepLast(64)` (about 178 ms at 360 Hz).

The net crossing is fixed task-phase bookkeeping, not a safety or quality gate.
There is no confidence, stability-frame, READY, source-age, calibration, or
balance admission check. A mathematically invalid one-shot is still logged as
that flight's sole result; it is not retried. Source age, diagnostics, residuals,
calibration status, queue depth, and deadline misses are audit-only.

Audit CSV creation is exclusive and refuses to truncate an existing attempt.
There is no Planner process-lock gate; operations should still start one
publisher per field attempt so its evidence has unambiguous ownership.

The hardware candidate also computes a robust angular velocity from the Ball
quaternion and runs a spin-aware Stage-2 shadow. Its spin estimate, predicted
crossing, and delta from the control predictor are written to the audit CSV
only. The 19-double command uses the venue table-contact law with zero spin;
measured spin and Magnus remain shadow-only until a later cross-session causal
replay and HDU/field qualification explicitly promote them.

## Local build and tests

The commands below assume the ROS-equipped `hope` container already exists.
On a new machine, create it using
[`docs/DISTROBOX_SETUP.md`](../../../docs/DISTROBOX_SETUP.md) first.

```bash
distrobox enter hope
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to hope_planner_cpp
colcon test --packages-select hope_planner_cpp
colcon test-result --verbose
```

The hardware configuration is
`config/model21800_hardware.yaml`. Its current estimator constants are an
offline candidate, not an HDU ARM or field qualification result.

## Offline replay

```bash
hope_planner_cpp_replay \
  --input /path/to/laptop/mocap_raw.csv \
  --output /tmp/model21800_cpp_replay.csv \
  --window 0.18 \
  --min-span 0.08 \
  --huber-delta 0.003 \
  --recency-half-life 0.0 \
  --restitution-h 0.64 \
  --restitution-v 0.9215 \
  --table-tangential-gain 0.369 \
  --spin-mode venue-grip \
  --control-zero-spin \
  --post-net-one-shot \
  --post-net-delay 0.05 \
  --post-net-future-bounce-tangential-gain 0.075 \
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

`--control-zero-spin` is required for exact hardware-control replay: it keeps
the venue table-contact law while preventing orientation-derived spin from
changing the strike. Omit it only for a separately named spin-shadow study.

The estimator retains the venue contact coefficient `0.369` for a bounce that
has actually been observed. The one-shot predictor uses the separately audited
effective coefficient `0.075` only when it must predict a future table contact
from pre-bounce data. This split avoids changing post-bounce state fitting to
compensate for the zero-spin causal prediction model. The hardware candidate
also uses the adaptive Stage-2 horizon, capped at `3.0 s`.

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

Local x86 timing does not establish HDU ARM timing, DDS callback retention, P1
correctness, or physical robot behavior. The field runbook documents how to
build, run, and collect evidence for this candidate; that documentation is not
a qualification claim. Treat the candidate as unqualified until the planned
HDU ARM replay and supported-robot field test have passed.
