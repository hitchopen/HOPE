# Flight prediction acceptance receipt

Tested implementation: `7f663138f2d044c44e07004bb0e9360b7a98d131` on `feature/flight-state-prediction-v2`, based on
upstream `nightly_built` `8d599e7d21e196275cf426488555e2648e8fd789`. Later receipt-only commits do
not change the tested implementation. The original HOPE/nohope worktrees remain unchanged.

Functional software checks pass. **The complete Gate S is not claimed passed**:
the full affected gate retains the existing bringup flake8/pep257 failures. Their
assertion counts match the imported scaffold baseline. Gate P is NOT RUN because
independent recorded-flight truth is missing. Gate D is NOT ATTEMPTED: v2 remains
shadow-only, with hardware frame/paddle/policy/cancellation qualification pending.

## Actual execution

Environment: Ubuntu 24.04 in the existing `hope` distrobox; ROS Jazzy, GCC 13.3.0,
CMake 3.28.3 and Python 3.12.3. A fresh build/install/message-generation directory
was used, followed by the full gate after the final relevant changes.

| Package | CTest groups passed | Failed groups |
|---|---:|---|
| hope_msgs | 2/2 | none |
| hope_planner_cpp | 27/27 | none |
| hope_bringup | 24/26 | flake8, pep257 |

The baseline had 21/21 planner groups and 2/2 message groups passing. Its separate
relay dependency failure is resolved by building this checkout's mocap interfaces.
The unchanged lint debt is 3,086 flake8 assertions and 37 pep257 assertions; colcon's
aggregate count also includes failing test-group wrappers and must not be confused
with this count. No thresholds or test groups were disabled to report success.

Numerical solver agreement (not real-ball accuracy):
`{"events": 42, "max_position_error_m": 7.22413413829901e-14, "max_time_error_s": 5.0182080713057076e-14, "reference": "independent scipy DOP853 rtol=2e-12 atol=2e-14", "status": "PASS"}`.

The numerical envelope is both directions, speeds 1.5/3 m/s, raw spin absent or
Y spin +/-30 rad/s with fixed X/Z components, with 1/0.5/0.25 ms refinement versus
independent DOP853 (rtol 2e-12, atol 2e-14). Contact positions/times use 1 mm/0.5 ms
budgets. Separate independent analytic tests cover receiving/outgoing plane roots
and the vertical-velocity apex root; derivative, energy and covariance checks are
separate. Edge/net/ordering fixtures cover geometry outside the smooth envelope.

Paced ROS evidence:

```json
{
  "status": "PASS",
  "streams": [
    {
      "rate_hz": 150,
      "source_span_s": 2.4479716280000003,
      "published_input_rate_hz": 143.38401474316433,
      "observed_input_rate_hz": 143.38401474316433,
      "observed_input_count": 352,
      "unaccounted_input_count": 0,
      "admitted_decisions": 4,
      "publication_latency_ms": {
        "p50": 1.0549404999999998,
        "p95": 2.225052,
        "p99": 2.339035,
        "maximum": 2.429273
      },
      "input_count": 352,
      "physical_outputs": 344,
      "denominator": 336,
      "physical_publication_coverage": 1.0,
      "target_outputs": 212,
      "planner_decisions": 367,
      "superseded": 0,
      "latency_ms": {
        "p50": 1.539964,
        "p95": 5.073451,
        "p99": 6.071994,
        "maximum": 16.510978
      }
    },
    {
      "rate_hz": 300,
      "source_span_s": 2.4610037360000003,
      "published_input_rate_hz": 288.9065098111659,
      "observed_input_rate_hz": 288.9065098111659,
      "observed_input_count": 712,
      "unaccounted_input_count": 0,
      "admitted_decisions": 4,
      "publication_latency_ms": {
        "p50": 1.034162,
        "p95": 3.23903,
        "p99": 3.513303,
        "maximum": 3.729396
      },
      "input_count": 712,
      "physical_outputs": 701,
      "denominator": 696,
      "physical_publication_coverage": 0.9956896551724138,
      "target_outputs": 444,
      "planner_decisions": 724,
      "superseded": 3,
      "latency_ms": {
        "p50": 1.499595,
        "p95": 4.765933,
        "p99": 5.493765,
        "maximum": 24.515941
      }
    }
  ]
}
```

Coverage denominator is actual published eligible source stamps after a four-sample
startup allowance per physical flight, including rejected outputs. Physical forecast
coverage and usable targets/planner decisions are separate. Full trajectories and
packets are decimated to 50 Hz; continuous compact revisions retain original source
references. Input accounting, observed rates, superseded jobs and publication/receipt
latency are reported. Earlier development and installed runs failed the 95%
threshold (79.17% and 78.57%). The old fixture paired nominal frame-index positions
with actual wall-clock acquisition stamps. A forced scheduler pause reproduced
the same contiguous lost-flight pattern and `qualified_non_ballistic_motion`
rejection. The fixture now computes analytic position at its actual source stamp;
a deterministic pause/catch-up regression covers both rates. Original failure
logs and the forced probe are retained. The 95% threshold is unchanged, and the
final checks below use the corrected source-time-consistent fixture.
One running solve and one replaceable pending snapshot bound
work. Jitter, dropout and a separate paced deliberately slow-worker regression are
included. The 95% physical publication threshold remains active.

Supplementary Python spin: PASS: 6 tests. Native sanitizers:
PASS: 4 focused native groups; ASan+UBSan+leak detection. The installed replay also passes the independent numerical gate. Three installed
ROS repeats pass with 100% coverage at 150 Hz and 98.1–99.3% at 300 Hz; P95
publication latency stays below 3.3 ms. Full per-run receipts are included in JSON.
Sanitizer coverage applies to the native shared core at `73298b1`; later changes
add ROS diagnostics, the corrected fixture and a deterministic native test.

## Case traceability

Each status below refers to the executed deterministic software assertions. X08 is
PARTIAL because its complete clean test clause includes the inherited lint failures. B11's
real-data clause is NOT RUN. X07's hardware clause remains unqualified. Full assertion
names and commands are in `flight_prediction_acceptance.json`.

| Case | Software | Evidence |
|---|---|---|
| A01 | PASS | test_flight_prediction_v2 |
| A02 | PASS | test_flight_prediction_v2 |
| A03 | PASS | test_flight_prediction_v2 |
| A04 | PASS | test_flight_prediction_v2 |
| A05 | PASS | test_flight_prediction_v2 |
| A06 | PASS | test_flight_prediction_v2 |
| A07 | PASS | test_flight_prediction_v2 |
| A08 | PASS | test_flight_prediction_v2 |
| A09 | PASS | test_flight_prediction_v2, test_strike_target_selector |
| A10 | PASS | test_flight_prediction_v2 |
| A11 | PASS | test_flight_prediction_v2 |
| A12 | PASS | flight_v2_ros_transport, test_flight_transport_v2, test_schema2, test_shadow_runner_input |
| A13 | PASS | flight_v2_causal_replay, flight_v2_ros_transport |
| A14 | PASS | test_flight_prediction_v2, test_future_contact_predictor, test_strike_target_selector |
| B01 | PASS | test_flight_prediction_v2 |
| B02 | PASS | test_flight_prediction_v2 |
| B03 | PASS | test_flight_prediction_v2, test_future_contact_predictor |
| B04 | PASS | test_flight_prediction_v2, test_flight_transport_v2 |
| B05 | PASS | hope_bringup/planner_full_rate_relay_test, test_flight_prediction_v2, test_flight_transport_v2 |
| B06 | PASS | test_flight_prediction_v2 |
| B07 | PASS | test_flight_prediction_v2, test_flight_transport_v2 |
| B08 | PASS | test_flight_prediction_v2 |
| B09 | PASS | test_flight_prediction_v2 |
| B10 | PASS | flight_v2_numerical, test_flight_prediction_v2 |
| B11 | PASS | flight_v2_causal_replay, flight_v2_scoring |
| B12 | PASS | test_adaptive_flight_commit, test_future_contact_predictor |
| C01 | PASS | test_flight_prediction_v2 |
| C02 | PASS | test_flight_prediction_v2 |
| C03 | PASS | test_flight_prediction_v2 |
| C04 | PASS | test_flight_prediction_v2, test_flight_transport_v2 |
| C05 | PASS | test_flight_prediction_v2 |
| C06 | PASS | test_flight_prediction_v2 |
| C07 | PASS | test_flight_prediction_v2 |
| C08 | PASS | test_flight_prediction_v2 |
| T01 | PASS | flight_v2_causal_replay, test_flight_prediction_v2, test_flight_transport_v2 |
| T02 | PASS | test_flight_prediction_v2 |
| T03 | PASS | flight_v2_ros_transport, test_flight_prediction_v2, test_flight_transport_v2 |
| T04 | PASS | test_flight_prediction_v2 |
| T05 | PASS | test_flight_prediction_v2 |
| T06 | PASS | test_flight_prediction_v2, test_flight_transport_v2, test_strike_target_selector |
| X01 | PASS | test_flight_prediction_v2 |
| X02 | PASS | test_flight_prediction_v2 |
| X03 | PASS | test_flight_prediction_v2 |
| X04 | PASS | flight_v2_ros_transport, test_flight_prediction_v2 |
| X05 | PASS | ckf_strike_shadow_ros, test_planner_node_turn_guard |
| X06 | PASS | test_flight_packet_pipeline, test_shadow_runner_input |
| X07 | PASS | flight_v2_ros_transport, test_schema2, test_shadow_runner_input |
| X08 | PARTIAL | flight_v2_causal_replay, flight_v2_ros_transport, hope_bringup/flake8, hope_bringup/pep257 |
| X09 | PASS | flight_v2_numerical, test_flight_prediction_v2, test_flight_transport_v2 |
| X10 | PASS | ckf_strike_shadow_ros, flight_v2_ros_transport |

## Reproduction

Run inside the HOPE distrobox, from a shell with no private workspace overlay:

```bash
source /opt/ros/jazzy/setup.bash
hope_ws/src/hope_planner_cpp/scripts/flight_prediction_validation/verify.sh /tmp/hope-v2-new-clean-run
```

The output directory must not exist. The command builds mocap interfaces from the
same public checkout, then messages, planner and bringup. Its full-gate exit status
remains nonzero for the documented existing lint failures. Generated evidence lives
under that explicit directory, not in source control. The delivered checkout also
retains raw final logs under ignored `artifacts/flight_prediction_v2/`. CTest discovers the numerical,
causal replay, transport, receiver and scoring checks normally. Run the parked
COLCON_IGNORE Python helper directly using the command in the JSON receipt.

Additional executed checks can be reproduced inside the same distrobox:

```bash
cmake -S hope_ws/src/hope_planner_cpp/test/native -B /tmp/hope-v2-sanitizers \
  -DCMAKE_BUILD_TYPE=Debug \
  '-DCMAKE_CXX_FLAGS=-O1 -g1 -fsanitize=address,undefined -fno-omit-frame-pointer'
cmake --build /tmp/hope-v2-sanitizers -j1 --target test_flight_prediction_v2 \
  test_batch_physics_estimator test_future_contact_predictor test_prediction_event_order
ASAN_OPTIONS=detect_leaks=1 UBSAN_OPTIONS=halt_on_error=1 \
  ctest --test-dir /tmp/hope-v2-sanitizers --output-on-failure \
  -R '^(flight_prediction_v2|batch_physics_estimator|future_contact_predictor|prediction_event_order)$'
PYTHONPATH=hope_ws/src/hope_planner python3 -m pytest -q \
  hope_ws/src/hope_planner/test/test_spin_estimator.py
```

For independent recordings, use `evaluate_recordings.py --records matched.jsonl
--output scored.json`; records must already be explicitly associated by physical
flight/event and source-information cutoff. It retains rejected denominators,
separates spin groups/methods and conditional position/velocity covariance marginals,
and resamples whole physical flights. It rejects missing physical provenance and
non-independent truth. No physical-data score is fabricated here.

## Review notes

The old numerical goldens and known-illegal advisory assertion were intentionally
revised to the shared model and revocation contract. A legacy smoke timeout exposed
an inherited reset race: a delayed `hit_plane_refresh` invalidation from the retired
session could halt a newly reset selector before its first flight. An independent
probe reproduced `halted=1 reason=forecast_invalidated:hit_plane_refresh` against
the untouched scaffold. The source-floor fence and deterministic A09/T06 regression
fix that race. Separately, the smoke fixture sends additional distinct post-net
exposures while still stopping within 25 ms of the net and requiring timer emission
at least 10 ms after final delivery. The timer-only assertion remains active.

C04 additionally covers receiver-side acquisition with missing earlier events in
both directions, attachment of independently verified pre-acquisition history,
preservation of queue/spin/generation, and qualified-boundary rejection of prior
shot events. Spin event IDs no longer imply chronological order; late applicable
out-of-order impulses are ambiguous, and acquisition replay cannot backfill later
spin into earlier intervals.

Physical qualification requires the recordings, annotations and clock/frame evidence
listed in the JSON receipt. The calibrated coefficient receipt itself is unchanged.
ON_REFRESH remains the existing production profile; FOLLOW_POSE is explicitly
selected in v2 shadow configuration. No push, PR publication, deployment or robot
motion was performed.
