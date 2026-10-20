# Validation and reproduction

The executed `flight_prediction_acceptance.md` / JSON is the result of record.
Source dates, final tested commit, effective configuration hashes and actual
metrics are pinned there. Historical failures are retained with their fixes;
no numerical/freshness/availability thresholds are disabled.

## Separate gates

- **S-F:** every mandatory forecast/physics assertion, clean public build/install,
  source/packet/covariance/causality and legacy no-regression checks.
- **S-R:** complete affected repository gate, including inherited bringup lint.
  A scoped functional pass is not a claim that all repository lint passes.
- **RT:** selected output profile freshness/coverage and each measured cadence,
  with actual input rate, rejected/missing stamps, full/compact channels and payload.
- **P:** independent matched real-ball accuracy; NOT_RUN without independent data.
- **D:** deployment/adoption OUT_OF_SCOPE, not attempted.

## Clean ROS test

Use the existing `hope` distrobox (Ubuntu 24.04 / ROS Jazzy). In environments where
Snap changes the Podman data store, enter with:

```bash
env -u XDG_DATA_HOME -u XDG_CONFIG_HOME distrobox enter hope
source /opt/ros/jazzy/setup.bash
hope_ws/src/hope_planner_cpp/scripts/flight_prediction_validation/verify.sh /tmp/hope-v2-scoped-new-clean
```

Run from the checkout root. The output directory must not exist. The script clears
private/stale overlay variables, builds this public checkout's mocap interfaces,
messages, planner and bringup, then runs the entire affected test gate in a local
ROS domain. Its final exit remains nonzero for existing lint failures. Read the
individual CTest/group results and scoped acceptance receipt rather than replacing
that exit with a false FULL_GATE_PASS. Generated bulky logs belong in ignored
artifacts; portable summaries and source pins are committed.

The installed V2 executable is
`install/hope_planner_cpp/lib/hope_planner_cpp/hope_flight_prediction_v2_node`.
No runner is launched. `flight_v2_binary_scope` checks that it lacks inverse and
command-packing symbols; source and ROS graph tests independently audit the boundary.
`cmake --build <planner-build-dir> --target check_flight_prediction_v2` runs the
15 labelled forecast/physics/tooling groups without building the inverse planner.
Existing production/PR49 compatibility and baseline-trigger tests remain in the
full gate; passing them does not reinstate a deferred downstream feature.

## Installed-executable checks

After the clean build, source its install and run from the checkout root. These
are the installed checks used for the acceptance receipt; choose another output
directory when reproducing. Keep throughput measurements separate from compilers
or other test workloads.

```bash
source /tmp/hope-v2-scoped-final/install/setup.bash
export ROS_DOMAIN_ID=188 ROS_LOCALHOST_ONLY=1
scoped_tools=hope_ws/src/hope_planner_cpp/scripts/flight_prediction_validation
scoped_tests=hope_ws/src/hope_planner_cpp/test
scoped_bin=/tmp/hope-v2-scoped-final/install/hope_planner_cpp/lib/hope_planner_cpp
python3 "$scoped_tools/numerical_gate.py" --executable "$scoped_bin/hope_flight_prediction_v2_replay" --output /tmp/scoped-numerical.json
python3 "$scoped_tests/flight_prediction_v2_scenarios.py" --executable "$scoped_bin/hope_flight_prediction_v2_node" --output /tmp/scoped-raw.json
python3 "$scoped_tests/flight_prediction_v2_smoke.py" --executable "$scoped_bin/hope_flight_prediction_v2_node" --worker-delay-ms 12
python3 "$scoped_tests/matched_replay_gate.py" --baseline "$scoped_bin/hope_ball_flight_baseline_replay" --v2 "$scoped_bin/hope_flight_prediction_v2_replay" --output /tmp/scoped-synthetic-tooling
python3 "$scoped_tools/benchmark_cadence.py" --executable "$scoped_bin/hope_flight_prediction_v2_node" --smoke "$scoped_tests/flight_prediction_v2_smoke.py" --output /tmp/scoped-cadence
```

`benchmark_cadence.py` retains all eight independent results, even when an optional
profile fails. The complete sweep's exit reports the selected 50 Hz profile;
individual statuses and raw logs remain part of the result. The synthetic plotting
fixture deliberately includes unavailable/invalid method/event epochs; it exercises
the scoring/coverage path and is not a coherent real-flight accuracy comparison.

## Actual scenario coverage

Native APIs and installed ROS cover both directions and 150/300 Hz, actual
acquisition-time positions, automatic acquisition, SERVE context before B1,
B1/B2 confirmation/identity, context/history preservation, spin measured→held→
contact-propagated→reacquired, absent spin, q/−q/long gaps, live synthetic registered
FOLLOW_POSE, ON_REFRESH freeze, stale/fresh plane recovery, world/clock rejection,
new same-direction shot, delayed worker and stale generation rejection. Synthetic
registration remains synthetic: it is not a live frame/paddle qualification.

A raw plane pose may be newer than the ball's acquisition stamp while already
known when preparing the forecast. Pose age is checked against the preparation
clock, then geometry is frozen in the job. An unavailable pose cannot emit a new
predicted receiving-plane event using a default/frozen X; physical waypoints remain.

Contact detection uses five exposures, descending/rising source slopes and an
estimated low-point occurrence with bracket uncertainty. Current engineering
gates include low z≤R+0.03 m, total bracket<80 ms and excursion≥max(1 mm,2σ).
The non-ballistic regression excludes plausible unresolved contacts in
[R−0.01,R+0.03] m over the active window. These are explicit provisional thresholds,
not physically calibrated contact confidence.

## Numerical and sensitivity gate

`numerical_gate.py` runs the installed C++ replay against independently implemented
SciPy DOP853 at 1/0.5/0.25 ms, both directions, speed/spin variants and multiple
contacts. Separate analytic plane/end/apex fixtures, finite-difference free-flight
and event derivatives, energy/gain limits, PSD/symmetry and branch-quality tests
apply. Budgets are 1 mm and 0.5 ms solver discrepancy, not physical prediction error.
Net/edge/tangent/tie fixtures are separate from the smooth reference envelope.
A net/edge/ground stop terminates the modeled path; no uncalibrated net rebound or
future motion beyond such a terminal event is invented.

## Cadence and latency benchmark

```bash
python3 hope_ws/src/hope_planner_cpp/scripts/flight_prediction_validation/benchmark_cadence.py \
  --executable /path/to/install/hope_planner_cpp/lib/hope_planner_cpp/hope_flight_prediction_v2_node \
  --smoke hope_ws/src/hope_planner_cpp/test/flight_prediction_v2_smoke.py \
  --output /tmp/hope-v2-scoped-cadence
```

The tool runs 150/300 Hz nominal inputs separately at 50/100/150 Hz and
latest-completed-result full-output settings. It records failures as well as
successes. Zero is event-driven; fixed 300 Hz is not adopted when the unconstrained
profile already exceeds the measured consumer/resource budget. The default remains
50 Hz as the conservative observed tradeoff, not a claim of universal optimality.

The unchanged applicable compact-availability gate is ≥95% of published eligible
source stamps, allowing four startup samples per flight. Rejected/unreceived
outputs stay in that denominator. Full source coverage is separately reported and
is expected to reflect decimation, not silently relabelled compact availability.
Reports include P50/P95/P99/max source→generation/publication/subscriber receipt,
full received rate, input accounting, bounded one-running/one-pending work,
superseded snapshots, CPU/RSS, sampled CDR size/bandwidth and first useful synthetic
physical/target lead. CDR estimates exclude DDS/network headers. The analytic
fixture evaluates position at **actual** acquisition time under jitter/pauses.

## Sanitizers and original behavior

```bash
cmake -S hope_ws/src/hope_planner_cpp/test/native -B /tmp/hope-scoped-asan \
  -DCMAKE_BUILD_TYPE=Debug \
  '-DCMAKE_CXX_FLAGS=-O1 -g1 -fsanitize=address,undefined -fno-omit-frame-pointer'
cmake --build /tmp/hope-scoped-asan -j2
ASAN_OPTIONS=detect_leaks=1 UBSAN_OPTIONS=halt_on_error=1 \
  ctest --test-dir /tmp/hope-scoped-asan --output-on-failure
PYTHONPATH=hope_ws/src/hope_planner python3 -m pytest -q \
  hope_ws/src/hope_planner/test/test_spin_estimator.py
python3 hope_ws/src/hope_planner_cpp/scripts/flight_prediction_validation/production_parity.py \
  --repo . --output /tmp/scoped-production-parity.json
```

The parity tool measures unchanged inversion on fixed ball states and separately
reports changes from the receiving forward model. It does not substitute a
command golden rewrite for evidence. Original fixed-input scalar normal/velocity
assertions and snapshot/reversal/turn/reach/one-shot/receiver regressions remain.

## Independent physical evaluation

See `flight_prediction_accuracy/README.md` and `NOT_RUN.json`. To run real evidence:

```bash
python3 hope_ws/src/hope_planner_cpp/scripts/flight_prediction_validation/compare_recordings.py \
  --dataset /private/registered_held_out_flights.json \
  --v2-executable /path/to/hope_flight_prediction_v2_replay \
  --baseline-executable /path/to/hope_ball_flight_baseline_replay \
  --output /private/scored_flight_report
```

The working scorer retains rejection/missing-truth denominators, exact cutoffs,
actual one-shot epochs, spin/event/phase strata and common-valid-pair comparisons.
It resamples whole physical shots, not adjacent frames. Source lead and offline
compute are labelled separately from measured ROS publication/receipt latency.
Each figure family is generated separately by mode/event/shot type; synthetic
unit-test figures are watermarked and never count as Gate P. No raw private data
or fabricated physical superiority result is committed.
