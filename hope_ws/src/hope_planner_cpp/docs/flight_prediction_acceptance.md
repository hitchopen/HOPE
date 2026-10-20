# Minimal V2 acceptance

Tested code: `dcd17786f5b32afa56da63713c5cd32c3dffdd78`. Upstream base: `8d599e7d21e196275cf426488555e2648e8fd789`. Source: `9118a491df658fa7e68f042581af48890ec8b0b8`. Later commits update review documents/evidence only.

All **50 forecast/physics software cases** A01–A14, B01–B12, C01–C08, T01–T06 and X01–X10 PASS. The JSON names each executed group/assertion, configuration hash, command and evidence path. Seventeen downstream clauses remain explicitly deferred; the independent physical-data clause is NOT_RUN.

Clean HOPE-only Ubuntu 24.04/Jazzy build passed with no private checkout mounted. CTest groups: hope_msgs **2/2**; hope_planner_cpp **30/30** (including 35 V2 core assertions, 8 transport assertions and independent installed API consumer); hope_bringup **24/26**, with only inherited flake8/pep257 failures. The original baseline has planner 17/17 and bringup 23/25. Both branches have the same 3,086 flake8 and 37 pep257 file/code issues. One underlying test skips the standalone adapter's unsupported replay-launch mode on both branches; it is not counted as new coverage. ASan/UBSan **13/13** native groups passed.

Strict independently compiled production parity: 18 racket inversion fixtures, 3 receiving-flight cases, 540 predictor cases across all five legacy modes, and 140 spin/batch samples have exactly zero difference. Nakashima and venue contact modes remain distinct. Existing production sources/defaults/command types are pinned; BallSample grows and dependent C++ packages must be rebuilt.

Numerical gate: 42 DOP853 contact comparisons pass the 1 mm / 0.5 ms budgets, plus native analytic plane/apex and geometry/covariance checks. Matched synthetic tooling produced 270 rows, one actual one-shot epoch and six visibly synthetic plots; historical unsupported event paths are not fabricated.

Selected full/packet cadence: **50 Hz**. Compact physical-forecast coverage must remain >=95%; every published compact forecast source age <=30 ms and receipt P95 <=30 ms. Repeated isolated measurements:

| Nominal input Hz | Actual observed Hz | Compact coverage | Receipt P95 ms | Full received Hz |
|---:|---:|---:|---:|---:|
| 150 | 143.42 | 100.00% | 1.97 | 49.95 |
| 300 | 288.55 | 99.14% | 1.69 | 48.75 |

The clean CTest run independently passed coverage 99.70%/97.41% at nominal 150/300 Hz. A 12 ms delayed worker every 100 solves also passed (99.40%/96.98%). Both directions and both plane policies passed at 150/300 Hz, including automatic serve acquisition, two contacts, spin dropout, context/clock/frame resets and exact source stamps. Full payload/coverage, latency quantiles, CPU/RSS, superseded work and queue bounds are reported separately in the JSON.

Optional unrestricted full output at nominal 300 Hz **fails** compact coverage (91.52%). Optional 100/150 Hz profiles passed this run; they do not replace the conservative default. Earlier broad tests with concurrent builds failed coverage/target timing and remain archived as contention diagnostics; isolated final validation did not waive any thresholds.

**Physical accuracy = NOT_RUN.** No independent held-out annotated ball flights with clock/frame/registered-plane receipts were supplied. No live calibration, physical superiority, deployment readiness or GitHub Actions result is claimed. Legacy replay comparison is explicitly limited to fixed x_hit=.15 sessions.

Raw current-instance artifacts: `/workspace/.cloud-setup/v2-evidence`. Committed summaries are under `docs/flight_prediction_evidence/`; exact commands are in `flight_prediction_validation.md`.

## Software case traceability

The complete assertion names, split clauses, config/code hashes and commands are in the JSON. These are executed software results; physical qualification is separate.

| Case | Status | Executed groups |
|---|---|---|
| A01 | PASS | test_flight_prediction_v2 |
| A02 | PASS | test_flight_prediction_v2 |
| A03 | PASS | test_flight_prediction_v2 |
| A04 | PASS | test_flight_prediction_v2 |
| A05 | PASS | test_flight_prediction_v2 |
| A06 | PASS | test_flight_prediction_v2 |
| A07 | PASS | test_flight_prediction_v2 |
| A08 | PASS | test_flight_prediction_v2 |
| A09 | PASS | test_flight_prediction_v2, flight_v2_ros_transport, flight_v2_raw_scenarios, test_adaptive_flight_commit, test_incoming_trajectory, test_planner_node_turn_guard |
| A10 | PASS | test_flight_prediction_v2 |
| A11 | PASS | test_flight_prediction_v2, flight_v2_raw_scenarios |
| A12 | PASS | test_flight_prediction_v2, test_flight_transport_v2, flight_v2_ros_transport |
| A13 | PASS | flight_v2_ros_transport, flight_v2_causal_replay, flight_v2_raw_scenarios |
| A14 | PASS | test_flight_prediction_v2 |
| B01 | PASS | test_flight_prediction_v2, flight_v2_raw_scenarios |
| B02 | PASS | test_flight_prediction_v2 |
| B03 | PASS | test_flight_prediction_v2, flight_v2_baseline_epochs |
| B04 | PASS | test_flight_prediction_v2, test_flight_transport_v2 |
| B05 | PASS | test_flight_prediction_v2, test_flight_transport_v2, planner_full_rate_relay_test, flight_v2_raw_scenarios |
| B06 | PASS | test_flight_prediction_v2 |
| B07 | PASS | test_flight_prediction_v2, test_flight_transport_v2 |
| B08 | PASS | test_flight_prediction_v2 |
| B09 | PASS | test_flight_prediction_v2 |
| B10 | PASS | test_flight_prediction_v2, flight_v2_numerical, flight_v2_baseline_epochs |
| B11 | PASS | flight_v2_causal_replay, flight_v2_scoring, flight_v2_accuracy_tooling, flight_v2_matched_replay_tooling |
| B12 | PASS | test_flight_transport_v2, test_prediction_event_order |
| C01 | PASS | test_flight_prediction_v2 |
| C02 | PASS | test_flight_prediction_v2 |
| C03 | PASS | test_flight_prediction_v2 |
| C04 | PASS | test_flight_prediction_v2, test_flight_transport_v2 |
| C05 | PASS | test_flight_prediction_v2 |
| C06 | PASS | test_flight_prediction_v2 |
| C07 | PASS | test_flight_prediction_v2 |
| C08 | PASS | test_flight_prediction_v2 |
| T01 | PASS | test_flight_prediction_v2, test_flight_transport_v2, flight_v2_causal_replay |
| T02 | PASS | test_flight_prediction_v2 |
| T03 | PASS | test_flight_prediction_v2, test_flight_transport_v2, flight_v2_ros_transport |
| T04 | PASS | test_flight_prediction_v2 |
| T05 | PASS | test_flight_prediction_v2 |
| T06 | PASS | test_flight_prediction_v2, test_flight_transport_v2 |
| X01 | PASS | test_flight_prediction_v2 |
| X02 | PASS | test_flight_prediction_v2 |
| X03 | PASS | test_flight_prediction_v2 |
| X04 | PASS | test_flight_prediction_v2, flight_v2_ros_transport, flight_v2_raw_scenarios |
| X05 | PASS | flight_v2_raw_scenarios, flight_v2_scope |
| X06 | PASS | test_flight_transport_v2 |
| X07 | PASS | test_flight_transport_v2, flight_v2_ros_transport |
| X08 | PASS | test_flight_prediction_v2, flight_v2_scope, flight_v2_baseline_epochs, flight_v2_binary_scope, flight_v2_installed_api |
| X09 | PASS | test_flight_prediction_v2, test_flight_transport_v2, flight_v2_numerical |
| X10 | PASS | flight_v2_scope, flight_v2_binary_scope, flight_v2_ros_transport |
