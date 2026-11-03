# V2 minimal-branch engineering report

This report replaces scaffold-era descriptions of 9118a491/2bb6cc56 for the new minimal branch. The uploaded specification supplies requirements, not final passing results. The separately referenced original review PDF was not supplied, so exact page citations are unavailable.

Architecture: qualified observation -> ShotContext / six-state CKF / TrustedSpin -> common forward physics -> nominal physical events / registered X-plane -> physical prediction, trajectory and causal packet. No inverse planning, command packing, runner change or robot execution is added.

Scope decisions: retain only essential physical modules and small history/config hash helpers; isolate strict spin qualification in FlightSpinEstimator; preserve every original production predictor/estimator/control implementation; exclude the 24 historical PR49 paths; expose a forward-only API. Preserve sparse actual historical one-shot epochs and disclose effective model/configuration differences.

Report refresh checklist:

- [x] Refresh both fork/upstream histories and source/base pins.
- [x] Preserve source branches, original production code/defaults and mode distinctions.
- [x] Complete every-file inventory and excluded-path register.
- [x] Run fresh public-HOPE-only build and final software, numerical, scope, parity, ROS, load and sanitizer checks.
- [x] Record all 50 software IDs with separate downstream and physical-data clauses.
- [x] Diagnose the source-gap defect with a failing-before/passing-after regression.
- [x] Record independent source pacing and optional profile failures transparently.
- [x] Refresh this Markdown/PDF, PR body and machine-readable receipts.
- [ ] Obtain independent held-out real flights and qualified clock/frame/plane receipts before physical qualification.


Validated code/tests: `38e31927373d90c538984f6174b6ff3525edf35e`. Fresh runtime build: `4a495ec1e1ba407c6c19bfd95afd8ad04e5859b5`. Upstream: `8d599e7d21e196275cf426488555e2648e8fd789`. Scoped source: `9118a491df658fa7e68f042581af48890ec8b0b8`. The later validation commit changes the synthetic ROS fixture; runtime sources, headers, messages and CMake are byte-identical to the fresh build. Later review commits change documentation/evidence only.

All **50 forecast/physics software cases** A01–A14, B01–B12, C01–C08, T01–T06 and X01–X10 PASS. The JSON maps each case to executed groups/named native tests, config/code hashes, commands and evidence. Seventeen downstream clauses remain deferred. The separate independent physical-data clause is NOT_RUN.

Clean HOPE-only Ubuntu 24.04/Jazzy build passed with no private checkout mounted. CTest: hope_msgs **2/2**; hope_planner_cpp **30/30**, including 36 core and 8 transport native test cases, a separately compiled installed-API consumer, numerical/causal/scoring/matched replay, binary/source scope and all eight ROS direction/rate/plane-policy combinations. Native ASan/UBSan: **13/13** groups.

Bringup: **24/26**, with only inherited flake8/pep257 failures. Baseline: planner 17/17, bringup 23/25. Both branches retain exactly the same 3,086 flake8 and 37 pep257 file/code issues. One underlying standalone adapter test skips its unsupported replay-launch mode on both branches. Full test-result exit is 1 because of the disclosed lint failures; no failure is suppressed.

Strict independently compiled production parity: 18 racket inversion fixtures, 3 receiving-flight cases, 540 predictor fixtures across all five modes, and 140 spin/batch samples have zero difference. Nakashima and venue modes remain distinct. Original production source/default/command types and all nine original YAML files are pinned. BallSample grows; rebuild C++ dependents rather than mixing older objects.

Revalidation found a V2 defect: a 128 ms observation gap hid a bounce, but the qualifier treated the interval as smooth acceleration and invalidated a physical ball. A deterministic regression fails before the fix at t=.44 s and now passes in both directions at 150/300 Hz. Gap-crossing acceleration evidence is withheld; history stays incomplete, no contact is fabricated, held checks remain active, and continuous non-ballistic rejection resumes after the gap leaves the window. Physical coefficients and legacy algorithms are unchanged.

The ROS fixture now paces its camera in an independent process, so decoding a large received packet cannot pause source generation. Actual functional source rates are about 148/298 Hz with deliberate two-frame dropout; exact rates/gaps are recorded per case. The independent synthetic authority renews complete-history evidence after gaps; both contacts must still be observed automatically. No incomplete-history target is forced. The load fixture awaits START acknowledgement before eligible input.

Numerical gate: 42 independent DOP853 contact comparisons meet the 1 mm / 0.5 ms budgets; native tests additionally check analytic plane/apex roots, geometry and covariance. Matched synthetic tooling produced 270 rows, one actual one-shot epoch and six visibly synthetic plots. Unsupported historical events are not generated using V2 physics.

Selected full/packet cadence: **50 Hz**. Required compact eligible coverage remains >=95%; every published compact source age <=30 ms, receipt P95 <=30 ms. Final isolated measurements:

| Nominal input Hz | Actual observed Hz | Compact coverage | Receipt P95 ms | Full received Hz |
|---:|---:|---:|---:|---:|
| 150 | 144.89 | 100.00% | 1.93 | 49.91 |
| 300 | 289.49 | 97.99% | 3.21 | 49.94 |

A delayed worker (12 ms every 100 solves) also passes at both nominal rates: 99.11%/95.83% coverage, receipt P95 2.03/2.72 ms. Full coverage/cadence, latency quantiles, CPU/RSS, payload sizes, superseded work and queue bounds are separate in the JSON. Optional 150 Hz full output at nominal 300 Hz fails coverage (93.39%); unrestricted full output at 300 Hz also fails (89.51%). Other optional profiles have individual outcomes. The default remains 50 Hz; thresholds were not lowered.

**Physical accuracy = NOT_RUN.** Independent held-out annotated real flights and clock/frame/registered-plane receipts are unavailable. No physical superiority, live calibration, deployment readiness or GitHub Actions result is claimed. Historical comparison supports only registered fixed x_hit=.15 sessions. The retired Python planner, production fallback migration and inverse/control adoption remain outside this branch.

Raw logs/XML/current-instance outputs: `/workspace/.cloud-setup/v2-evidence`. Committed summaries: `docs/flight_prediction_evidence/`. Reproduction: `flight_prediction_validation.md`. Earlier failing setup/load/fixture runs remain archived and are not substituted for final results.

## Software case traceability

Full named tests, split clauses and provenance are in the JSON. Physical qualification is separate.

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
| A13 | PASS | flight_v2_ros_transport, flight_v2_causal_replay, flight_v2_raw_scenarios, test_flight_prediction_v2 |
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
