# hope_planner_cpp

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
estimator history. Once the estimator has at least 80 ms and 12 samples, the
callback freezes the complete retained history (capped at 180 ms) into
immutable revisions at about 30 Hz. Each revision
contains one internally consistent position, velocity, TTS, side and station
solve. A final revision is still frozen at `net crossing + 50 ms` for audit.
`hope_planner_cpp_node` deduplicates retries and uses a latest-only mailbox, so
an older pending revision cannot accumulate behind newer data. Direct `/poses`
input remains an explicit legacy A/B mode and is disabled by production config.

The net crossing is fixed task-phase bookkeeping, not a safety or quality gate.
There is no confidence, stability-frame, READY, source-age, calibration, or
balance admission check. A mathematically invalid revision is logged and a
newer complete revision may supersede it before Runner engage. Source age,
diagnostics, residuals, calibration status, mailbox depth, and deadline misses
are audit-only. Runner atomically freezes the latest complete tuple at its
existing engage boundary and ignores later revisions during the swing.

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
colcon build --symlink-install \
  --packages-select hope_msgs hope_planner_cpp \
  --cmake-args \
    -DPython3_EXECUTABLE=/usr/bin/python3 \
    -DPYTHON_EXECUTABLE=/usr/bin/python3
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
  --post-net-future-bounce-tangential-gain 0.369 \
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

The estimator and the future-contact predictor both use the venue contact
coefficient `0.369`. This removes the old one-shot-only `0.075` model split.
The retained Python predictor still uses its legacy point-ball contact geometry,
so the offline comparison reports that residual explicitly instead of claiming
bit parity. The hardware candidate also uses the adaptive Stage-2 horizon,
capped at `3.0 s`.

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
