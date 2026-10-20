# Executed scoped ball-flight acceptance

Tested source: `2bb6cc56b0aaa8747c99e20de8354bd20b204cbc` on `feature/flight-state-prediction-v2-scoped`.
Upstream freshly checked: `8d599e7d21e196275cf426488555e2648e8fd789`; scoped parent: `e73b3123bb881a10b562f1ba74a5173d667f31d8`.
Documentation-only receipt commits follow the tested source. All runtime and test sources remain those tested.

**This receipt replaces the broader pre-cleanup V2 PASS claims.** The deliverable ends at physical ball forecasts. No V2 command construction, inverse solve, schema packing, runner, merge or robot motion is enabled.

## Gate results

| Gate | Executed result |
|---|---|
| S-F | PASS |
| S-R | FAIL_KNOWN_PRE_EXISTING_BRINGUP_LINT |
| RT | PASS_SELECTED_50_HZ_FULL_PROFILE; OPTIONAL_PROFILES_INDIVIDUAL_RESULTS |
| P | NOT_RUN_INDEPENDENT_DATA_UNAVAILABLE |
| D | OUT_OF_SCOPE_NOT_ATTEMPTED |

S-F covers every software forecast/physics subassertion of A01–A14, B01–B12, C01–C08, T01–T06 and X01–X10, plus the new classification regressions. B11 physical error/uncertainty is a separate NOT_RUN subassertion. Seventeen explicit downstream/adoption/lint-cleanup clauses remain deferred; none is counted as a passed downstream implementation.

## Environment and clean gate

Executed in `hope` distrobox: Ubuntu 24.04, ROS Jazzy, GCC 13.3.0, CMake 3.28.3, Python 3.12.3. The clean script clears stale overlays and builds public HOPE messages, planner, bringup and public mocap interfaces into a fresh external install.

| Package | CTest groups passed | Failed |
|---|---:|---|
| hope_msgs | 2/2 | none |
| hope_planner_cpp | 33/33 | none |
| hope_bringup | 24/26 | flake8, pep257 |

Build exit 0 (four packages, 1 min 35 s). Full verification exit 1 preserves inherited lint: flake8 3,086 failing assertions; pep257 37. The pre-change baseline reproduced the same counts. Colcon aggregate: 3,593 tests, 0 errors, 3,125 failures (including wrapper accounting), one existing skip: standalone NatNet adapter has no bag-replay launch mode. No prediction assertion is skipped.

Native ASan + UBSan + leak detection: **13/13 groups passed** (40.02 s). Instrumented native code is unchanged between sanitizer source `9b5a00d` and the final tested commit; later changes affect ROS transport/node, replay/export and Python tooling. This is not an ASan-instrumented ROS-node claim. Supported Python spin checks: **6 passed**. Source, binary-symbol and actual ROS graph audits all passed.

## Configuration and output contract

- Shadow YAML SHA-256: `c3ca86f15a140d82bdb4964a965fe1570ded7681d4a924903ee1881b090c214d`.
- Physics profile: `thin-shell-forward-v3-receipt-20261010`.
- Effective default model ID: `flight-model-v3:sha256:824cb7ac358b163a26df34b73ec05d424b6955bcf7e5135aed547d051120a024`.
- Observation/context schema 2; forecast/packet schema 3. Consumers must regenerate/rebuild together. Old packet schema rejected.
- Physical outputs: `/hope/shadow/flight_v2/prediction`, `/hope/shadow/flight_v2/trajectory`, `/hope/shadow/flight_v2/packet`; invalidation/revocation is carried by the forecast stream.
- Current p/v, optional spin, 6×6 covariance; full waypoints; stable physical events with pre/post states; selected ball crossing with absolute arrival and conditional 7×7 event covariance; registered plane geometry/pose source time.
- `shot_id == shot_generation` within a session. Reference, occurrence, confirmation, generation, publication and receipt times remain distinct. Absolute arrival equals reference plus integrated duration using checked signed nanoseconds.
- Covariance is conditional on fixed auxiliary spin/model/geometry/clock assumptions; it is not a complete physical error bound. No racket normal, velocity, motion start or robot feasibility output exists.

## Runtime and cadence

Each row is an independent installed ROS run: four 0.6 s synthetic incoming flights, acquisition-time positions, jitter/two-frame dropout, with four startup samples per flight excluded. Eligible published input stamps remain in the denominator even if not received/forecast. Full coverage is intentionally lower under decimation. Compact publication age must be ≤30 ms for every emitted physical result; receipt P95 must be ≤30 ms; compact availability must be ≥95%. Packet cadence stays 50 Hz during this full-trajectory sweep.

| Full setting Hz | Nominal input Hz | Published / observed Hz | Compact coverage | Full source coverage | Full received Hz | Status |
|---:|---:|---:|---:|---:|---:|---|
| 50 | 150 | 143.40 / 143.40 | 100.00% | 34.23% | 49.93 | PASS |
| 50 | 300 | 288.88 / 288.47 | 96.98% | 16.38% | 47.50 | PASS |
| 100 | 150 | 143.47 / 143.06 | 98.81% | 66.37% | 95.16 | PASS |
| 100 | 300 | 288.91 / 288.91 | 98.71% | 33.33% | 96.63 | PASS |
| 150 | 150 | 143.52 / 143.11 | 98.81% | 98.21% | 140.55 | PASS |
| 150 | 300 | 288.92 / 288.10 | 97.27% | 48.28% | 140.48 | PASS |
| latest | 150 | 143.52 / 143.11 | 99.11% | 99.11% | 142.16 | PASS |
| latest | 300 | 288.61 / 284.96 | 93.82% | 85.78% | 248.18 | FAIL |

Retain 50 Hz as the conservative full/packet default. Final 100/150 Hz profiles passed this run; earlier development runs failed availability at those settings, so they are not universally qualified. Unrestricted latest at 300 Hz failed 95% availability (93.82%). Fixed 300 Hz full output was not adopted because latest already exceeded the measured availability budget. No threshold was relaxed.

Selected 50 Hz distributions below are source-to-stage age in milliseconds, P50 / P95 / P99 / max. Subscriber receipt includes local DDS scheduling. Full trajectories retain their own original source timestamp.

| Nominal input | Stage/channel | P50 | P95 | P99 | Max |
|---:|---|---:|---:|---:|---:|
| 150 | compact generation | 0.939 | 1.237 | 1.349 | 1.661 |
| 150 | compact publication | 1.021 | 1.344 | 1.501 | 1.855 |
| 150 | compact receipt | 1.578 | 1.988 | 2.106 | 2.692 |
| 150 | full publication | 1.083 | 1.439 | 1.566 | 1.625 |
| 150 | full receipt | 4.364 | 5.939 | 6.553 | 17.037 |
| 300 | compact generation | 0.937 | 1.275 | 1.868 | 2.937 |
| 300 | compact publication | 1.020 | 1.381 | 1.952 | 3.024 |
| 300 | compact receipt | 1.593 | 2.049 | 2.720 | 7.285 |
| 300 | full publication | 1.077 | 1.421 | 1.554 | 1.840 |
| 300 | full receipt | 4.505 | 6.214 | 7.627 | 18.215 |

| Nominal input | Compact eligible / denominator | Inputs unaccounted | Node CPU / source span s | Peak RSS MiB | Mean full CDR bytes | Sampled full CDR MB/s |
|---:|---:|---:|---:|---:|---:|---:|
| 150 | 336/336 | 0 | 0.60/2.45 | 30.91 | 55735 | 2.80 |
| 300 | 675/696 | 1 | 1.05/2.46 | 30.46 | 61563 | 2.95 |

CDR estimates exclude DDS/network headers and packet-channel bandwidth. The worker is bounded to one running and one pending snapshot. Per-profile input counts, supersession, lifecycle reasons, generations, payloads, all age distributions and first synthetic useful physical/target lead are retained in JSON; they are not real-ball accuracy or robot lead measurements.

Occasional 12 ms worker delay, every 100th solve at default 50 Hz full output:

| Nominal input | Compact coverage | Receipt P95 / max ms |
|---:|---:|---:|
| 150 | 98.81% | 2.205 / 14.148 |
| 300 | 96.26% | 2.199 / 13.799 |

All six installed raw scenarios passed: FOLLOW_POSE both directions at 150/300 Hz; ON_REFRESH both directions at 150 Hz. They jointly exercise automatic acquisition, provisional SERVE context, same-ID confirmed B1, B2/ordered usable target, measured→held→contact-propagated→reacquired spin, missing/rejected angular values, plane +0.10 X, Y outside the old robot gate, new-shot spin clearing, revocation, mapping loss, incompatible clock, stale/fresh plane and world-frame invalidation. Synthetic table registration is explicitly not live hardware qualification.

## Numerical physics and production differential

Independent SciPy DOP853 compared 42 event instances at 1/0.5/0.25 ms in both directions: max position difference 7.22e-14 m; max event-time difference 5.02e-14 s. Both are below 1 mm / 0.5 ms budgets. These measure solver consistency, not physical prediction error. Separate analytic plane/apex/end, finite-difference derivative, covariance PSD/symmetry, edge/net/tie and dissipative contact tests passed.

The inverse racket solver and adaptive scheduler match upstream source bytes. All 18 fixed-ball-input command fixtures match exactly (max difference 0), including original Stage 3 scalar goldens. Receiving forward physics necessarily changes downstream inputs: in two mutually valid probe cases max arrival delta 49.671 ms, position delta 10.487 mm, velocity delta 0.675141 m/s; a third case changes invalid→valid. Full deltas are disclosed in JSON. No strict end-to-end production equivalence or real-world improvement is claimed.

## Complete requirement traceability

All entries below mean **scoped software PASS**, with actual executed assertion names, commands, source/config hashes and XML paths in the adjacent JSON. Deferred clauses and physical NOT_RUN are listed separately. Legacy command compatibility tests remain regression evidence only.

| Case | Scoped software assertion | Result |
|---|---|---|
| A01 | Exactly five public states; startup INVALID; qualified ±X creates provisional rally family; held/toss rejects; rejected same flight does not re-arm. | PASS |
| A02 | Confirmed first striker B1 reclassifies without losing state; known NORMAL_RALLY forbids it; forbidden order/count revokes ball target. | PASS |
| A03 | Outgoing RALLY B2 before opponent-end crossing retains RALLY and selects exact B1–B2 apex (state/time/uncertainty). | PASS |
| A04 | OPPONENT_RALLY B2 before receiving plane is forbidden; OPPONENT_SERVE needs ordered B2 then plane before B3. | PASS |
| A05 | A later predicted bounce after an already valid plane event does not invalidate the earlier event. | PASS |
| A06 | Both serve routes: new shot→provisional rally→striker B1/serve→net→receiver B2→selected ball event→**new observed racket shot boundary**; correct queue preservation/retirement. | PASS |
| A07 | Qualified held/non-ballistic motion, below-table states, net collision and bad clocks produce correct physical/data-quality reasons; no fabricated bounce, no apex-as-held error. | PASS |
| A08 | Hard new shot/INVALID/provenance change clears dependent fit/events/spin/jobs; same-shot label change preserves raw data and supersedes forecast only. | PASS |
| A09 | Delayed work, expired callbacks and reversal cannot republish retired **ball forecasts**. Preserve legacy precommit reversal protections by regression. | PASS |
| A10 | Predicted B1→confirmed B1 retains event ID, distinct occurrence/confirmation time and one contact impulse/count. | PASS |
| A11 | `FOLLOW_POSE`: base +0.10 X→next prediction plane +0.10; +0.58 offset, age/frame/range checked. Also test `ON_REFRESH` preservation. | PASS |
| A12 | Full *ball* target p/v/optional omega, reference/arrival, frame, covariance, historical status; physical Z never receives policy height offset. | PASS |
| A13 | 150/300 Hz, jitter/dropout, exact source delta-t, causality, no cross-shot spin increment, latency/availability report. | PASS |
| A14 | Nominal and fallback model/config agree; distinguish warmup, horizon, uncertainty, forbidden route and revoked/stale ball events. | PASS |
| B01 | Valid `(0,30,0)`→missing/all-zero-sentinel/NaN/rejected→new valid; preserve complete held vector and original source time, XYZ and causal recovery at both rates. | PASS |
| B02 | No trusted spin→POSITION_ONLY from start through every leg/two serve contacts; no synthetic omega/postbounce Magnus. | PASS |
| B03 | Mutating rejected nonzero raw angular vector cannot alter result when trusted state/XYZ fixed. | PASS |
| B04 | Accept individual zero components; distinguish genuine identity quaternion from synthetic placeholder through authoritative source quality. | PASS |
| B05 | Mocap relay→v2 observation/packetizer/forecast receiver/replay preserves orientation validity, exact time and XYZ. Preserve legacy consumer behavior rather than expanding its command duties. | PASS |
| B06 | Measured/held spin through one/two bounces produces once-only contact-propagated spin; absent-spin remains absent. | PASS |
| B07 | Trusted checkpoint survives window trim/refit/transfer/reclassification and clears on hard boundary; duplicate event doesn't apply impulse twice. | PASS |
| B08 | q/−q, axis/frame signs, ambiguous long-gap rejection, original held-spin age and source-time delta tests. | PASS |
| B09 | Below-table, radius-aware finite net, real top-table footprint, edge ambiguity, target/contact earliest-event and tie ordering. | PASS |
| B10 | Common profile/mode in fit/nominal/fallback/V2 replay; parameter bounds, dissipative behavior, independent step/time/position convergence. | PASS |
| B11 | Valid spin reacquisition is prospective, not retroactive; separately score position-only real-flight error/uncertainty when independent truth exists. | PASS |
| B12 | Publish exactly the rollout's contact/target arrival time and optional spin provenance; no fixed-rate, double-age or separate physics timing approximation in **our output**. | PASS |
| C01 | Qualified ±X shot begins correct provisional rally; complete evidence plus no striker bounce before net confirms rally; deadband does not change owner. | PASS |
| C02 | Confirmed striker-side B1→same-generation serve; preserve raw timestamps/queue/spin/B1 ID, increment semantic forecast revision and propagate contact once. | PASS |
| C03 | Second striker-side pre-net bounce invalidates, clears active shot; a merely predicted bounce never counts as observed; supported deterministic early invalidation is conservative. | PASS |
| C04 | First observation after net has unknown/incomplete prehistory; physical forecast may exist, but route target must wait for reliable history/evidence. | PASS |
| C05 | Predicted striker B1 selects provisional serve route without public confirmation/observed count; deleting hypothesis does not reset queue. | PASS |
| C06 | NORMAL_RALLY context forbids striker bounce, SERVE constrains order, receiver B2 doesn't revert serve, genuinely new racket return resets ownership. | PASS |
| C07 | Reclassification while a solve is pending: old semantic output is fenced, preserved samples remain usable; hard reset fences all retired work. | PASS |
| C08 | Table/net-induced reversal is not a new racket shot; qualified same-direction contact is; label change is not a shot reset. | PASS |
| T01 | Every retained XYZ sample has exact original ns through both rates, trimming, queue and transport; integration uses actual delta-t. | PASS |
| T02 | Bounce at 10.000 s confirmed at 10.020 s keeps those two distinct timestamps plus evidence uncertainty. | PASS |
| T03 | Reference 20.000 s + integrated duration 0.300 s→arrival 20.300 s independent of later publish/receipt, for B1/B2/plane/end/apex. | PASS |
| T04 | Revision preserves event identity/history; predicted elapsed time never becomes an observation or racket contact automatically. | PASS |
| T05 | Expose valid source-clock mapping/uncertainty; no `strike_start` field, no timing rejection due solely to robot lead, valid sim epoch zero. | PASS |
| T06 | Hard boundary retires generation/forecasts, same-shot reclass preserves data and supersedes revision; no old output restores revoked ball target. | PASS |
| X01 | Both ON_REFRESH (frozen until refresh) and FOLLOW_POSE (fresh per forecast) are explicit and never silently substituted. | PASS |
| X02 | Valid moving-plane geometry is not treated as shot/ball provenance fault; real frame/calibration change still invalidates incompatible state. | PASS |
| X03 | Uncertain early forbidden prediction can recover with later causal observations; incomplete conditional covariance cannot justify hard invalidation; well-supported violation and confirmed forbidden event can. | PASS |
| X04 | Pace 150/300 Hz with occasional slower solves; monotone source-stamped publications and bounded latest-work queue, no starvation. | PASS |
| X05 | Ball forecasts continue and preserve physical validity despite target Y being outside robot reach range; a later flight remains qualifiable. | PASS |
| X06 | Duplicate/retransmitted V2 forecast packets preserve identity/hash/dedupe; producer restart and equal-counter conditions are documented without fabricating accepted events. | PASS |
| X07 | Ball frame/clock/reference point, p/v/optional omega, waypoint/event/plane geometry, no physical-policy Z conversion, installed v2 receiver/serializer parity. | PASS |
| X08 | Clean HOPE-only configure/build/install/test/replay from no private checkout or stale overlays. Report inherited lint separately from scoped functional pass; do not hide it. | PASS |
| X09 | One common forward/event kernel and coefficients for mean, covariance, event, fallback, V2 replay, and **forward-only** hypothetical-outgoing physics; spin after unmodelled racket impulse must not inherit incoming trusted state. | PASS |
| X10 | No V2 publication to production command topics, including remapping; no shadow adapter constructs racket commands. | PASS |

Deferred clauses (17; downstream/adoption/lint cleanup, not software passes):

| Case | Deferred subassertion |
|---|---|
| A02 | No actual robot command cancellation. |
| A03 | No outgoing racket command. |
| A06 | Do not generate the racket return command. |
| A09 | Hardware command cancellation or new scheduling behavior. |
| A11 | No robot target recomputation. |
| A12 | Racket target/normal, command frame conversion or strike start. |
| A14 | No stale command use in actuator. |
| B05 | Retooling live robot command receiver for V2 forecast. |
| B10 | Adoption by downstream inverse optimizer deferred unless physics-only integration is safe. |
| B12 | Adaptive commit/readiness/strike scheduling changes in `adaptive_flight_commit`. |
| T05 | Planner's conversion, scheduling and motion rejection. |
| T06 | Cancelling/retiming physical robot motion. |
| X05 | Y admission / Stage 3 / flat-command behavior. |
| X06 | One-shot robot command/action deduplication. |
| X07 | Typed/flat racket command, swing normal and policy Z support qualification. |
| X08 | Unrelated repository lint cleanup, unless separately authorized. |
| X09 | Inverse landing-point optimization, surface-to-center *Stage 3 target* conversion, candidate-choice or command-validity changes. |

New classification cases N01–N03 passed: SERVE context cannot confirm an unobserved B1; history loss recomputes confirmed rally; installed raw acquisition→two-contact serve→new physical boundary preserves/fences ownership correctly. F01–F11 are resolved for the scoped software contracts, with explicit case mapping in JSON; none claims physical calibration or downstream adoption.

## Physical accuracy and earlier failures

**Gate P NOT_RUN. Dataset IDs: none; qualified physical flight IDs: none.** Independent truth, bounce annotations/occurrence uncertainty, acquisition-clock mapping and table/plane registration are unavailable. Requested paths are not supplied at this receipt. B11 real-flight error/uncertainty and superiority to baselines remain unmeasured.

The complete replay/scoring path passed on explicitly synthetic input: actual legacy one-shot trigger (one epoch near 202 ms), progressive batch (45 epochs), V2 at matched information cutoffs, 270 method/event rows, six watermarked figures (three families for each of plane and B1). The report retains missing/rejected rows and separates common-valid-pair error comparisons from availability. One-shot is not interpolated into an invented continuous curve. Whole-shot bootstrap and conditional 68/95% covariance scoring are tooling checks; synthetic figures do not qualify Gate P.

Prior failures are retained in JSON and local history logs. Fixed issues included premature context classification, retired B1 identity, ambiguity among successive retired hypotheses, MEASURED labels on propagated spin, unregistered fallback plane events and pose age evaluated against old ball time. Fixture corrections were independently checked legal serve geometry and enough fresh observations after an expired fit window. Optional high-cadence failures remain disclosed; baseline lint is unchanged.

## Reproduction and artifacts

Run from this checkout inside the HOPE distrobox; use a fresh output directory:

```bash
env -u XDG_DATA_HOME -u XDG_CONFIG_HOME distrobox enter hope
hope_ws/src/hope_planner_cpp/scripts/flight_prediction_validation/verify.sh /tmp/hope-v2-scoped-reproduce
```

See [validation commands](flight_prediction_validation.md) for sanitizer, supported Python spin, installed numerical/raw/latency/cadence, production parity and independent physical evaluation. The executable source is pinned above. Public scripts and source are sufficient; no private dependency or stale overlay is required. Raw XML/logs and synthetic figures are retained locally under ignored `artifacts/scoped/`; portable executed metrics and the complete case matrix are committed in [JSON](flight_prediction_acceptance.json). A content-hash artifact index accompanies the local bundle.

Provisional parameters to revisit are listed in [architecture](flight_prediction_implementation.md#provisional-values-and-adoption-evidence) and [physics/config](flight_prediction_physics_and_config.md): physical coefficients, position-only profile, source/pose/clock budgets, contact and acquisition thresholds, registered plane and venue geometry, integrator/horizon/window/capacity, and output cadence. No new robot Y admission exists in V2.

Publication is a review branch. PR body: [flight_prediction_pr_description.md](flight_prediction_pr_description.md). No PR, merge, deployment, policy change or robot action is performed. Independent real-flight and live registration evidence remain required before claiming full research validation.
