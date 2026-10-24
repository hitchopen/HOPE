# Integrated planner PR64 review corrections

**Review regression-complete; shadow only.** RV01–RV19 pass with inherited lint
reported separately. RV20 awaits a publication choice. This does not complete
broader V3 simulation or physical qualification.

Historical cloud-tested source: `dc914c8e985d8aaef87cbc22bb91b2deabc3c82e` on
`feature/integrated-racket-planner-v2-review-fixes`, descended from exact
`3346e03002b47611e3531d7147392faec9e9268f`. Production core last changed at
`912d456`; the standalone context guard last changed at `65adda5`. Later commits
change fixtures. Final receipt/document changes follow the tested source.
The JSON companion pins source/config/interface/binary hashes and case artifacts.
The local distrobox verification below independently checks the preserved
corrections and two additional regression assertions at `be642ba`.

## Repository audit and scope

Both fork histories and corresponding upstreams were inspected. The original
HOPE minimal worktree (`4dc0959`) and `nohope` (`96e71c1`) remain unchanged.
`nohope` has three parent-only and nine local-only commits against
`dongc1/HOPE` main `c2a627f`; its research estimator is not replaced.
HOPE fork main is `5810849`. Upstream `hitchopen/HOPE` main observed here is
`3b3c402`, which includes newer calibration and runner contracts. These were
reviewed without rebasing or substituting coefficients. PR64 remains open at
`9118a491`, targeting **nightly_built** (`8d599e7`). No upstream PR was changed.

The original reviewer traces/full initial conditions were not supplied. Tests
are fully specified independent equivalent counterexamples, not claims of
byte-identical reviewer reproduction. The actual integration baseline was
compiled and executed with injected tests, without applying source fixes.

## Per-finding disposition

| Finding | Baseline at 3346e030 | Correction and measured result |
| --- | --- | --- |
| R01 aerodynamic acquisition | Fast physically modeled launches fail the gravity-only qualifier. | Compare against configured drag/Magnus acceleration using causal qualified spin; use a continuous recent launch suffix after held samples. All 32 rate/speed/direction/spin/noise combinations pass, with held/impossible/provenance negatives retained. |
| R02 duplicate bounce | The specified -20 m/s equivalent confirms two events at 150 Hz for one bounce (300 Hz already has one). | Stable source reversal/bracket identity, intervening motion to rearm, and unique physical predicted association. Both rates retain one ID/revision/impulse, including packet resume and authoritative corroboration. Distinct close bounces remain distinct. |
| R03 mixed angular window | Delayed confirmation permits pre/post-contact quaternion blending. | Bounded causal history/checkpoints and post-contact-only refit. A 10→54.2 rad/s step at .12 s, confirmed at .14 s, stays propagated/held until the post-only .05 s span and sample-count gates qualify. |
| R04 finite net | A 27.53 mm corner clearance is falsely colliding for a 20 mm sphere. | Swept box is broad phase; continuous sphere-to-finite-rectangle distance supplies collision/time/normal. Clear corner passes; true 19 mm contact and 20 mm tangency collide; top/side/corner, direction and event-priority controls pass. |
| R05 historical contact spin | Causal pre-spin unavailable, producing post-impact vx=2.354680 m/s in the zero-slip fixture. | Resolve pre-bracket spin and descending-side impact state; replay bounded CKF checkpoints after one impulse. Pre-spin wy=150 rad/s and vx=3 produce corrected vx=3, even with a newer post-contact measurement. |
| R06 same-step covariance | Bounce .499 ms then plane .750 ms produces vx variance about 1e-10 at 1 ms, omitting bounce noise. | Propagate ordered free-flight segments, saltation/reset and one bounce-noise addition. Variance .4225 at 1/.5/.25 ms agrees with an independent analytic finite-difference oracle. Branch/tangent quality and PSD controls pass. |
| R07 context revocation | No-op/rejected contexts already protected. | Preserve that behavior; add semantic context-revision worker fence, rejected enum validation, stale generation/session/authority and delayed-worker coverage. |
| R08 legacy mode dispatch | Already restored on integrated branch. | Original predictor/planner sources and five modes unchanged; µ=0/.5 sensitivity and strict 701-case legacy parity pass. The return solver does not replace legacy research modes. |
| R09 explicit zero | Legacy numeric zero works; V2 trusted stationary zero fails qualification. | Authoritative finite zero and sufficiently sampled stationary qualified quaternions remain available. Placeholder/invalid/non-authoritative zero stays unavailable. Synthetic fixtures now mark unavailable zero sentinels explicitly. |

Additional direct tests exposed and fixed serialized reversal-identity loss,
held-prefix launch delay, unsupported source gaps retaining a prior shot ledger,
and repeated corroboration of an authoritative contact. Wire restore rebuilds
private reversal metadata from existing retained samples/confirmation time;
message definitions do not change.

## Code map

- [Acquisition, physical reversal identities and causal tracker replay](../src/flight_state_v2.cpp).
- [Spin checkpoints, contact segmentation and explicit zero availability](../include/hope_planner_cpp/trusted_spin.hpp), with the [quaternion estimator](../src/flight_spin_estimator.cpp).
- [Continuous sphere contact with the finite net](../src/common_ball_physics.cpp).
- [Ordered contact resets and event covariance](../src/nominal_event_forecast.cpp).
- [Focused review regressions](../test/test_v2_pr64_review_regressions.cpp) and [ROS context/contact fences](../test/integrated_context_noop.py).

## Historical cloud validation and accounting

| Check | Result |
| --- | --- |
| Actual baseline focused suite | 20 cases: 15 expected failures, 5 passes; failures and logs retained. R07/R08 are already-fixed dispositions. |
| Final focused suite | 25/25 pass, including the five added boundary/resume controls. |
| Native / independent ASan+UBSan | 15/15 CTest groups each, rebuilt and rerun at the tested source. |
| Independent outgoing DOP853 reference | 144 candidates checked; valid/rejected outcomes retained. |
| Legacy strict parity | 701 fixtures, all five modes, no exception/tolerance weakening. |
| ROS messages / planner / bringup | 2/2; cumulative 67/67; 24/26 respectively. |
| Bringup inherited lint | 3,086 flake8 and 37 pep257 failures; exact failed-issue multisets match baseline. Aggregate verification remains nonzero. |
| Installed producer chain | 30 cases: original, 10/12/20 m/s × 150/300 Hz × unknown/zero/known spin, plus physical geometry/held negatives. Actual installed relays and real C++ schema-2 decoder; no COMPLETE-history publisher. |
| Additional plane/expiry cases | Four fast ON_REFRESH cases pass; two 40 ms delayed-worker cases correctly emit zero valid commands with one-running/one-pending bound. |
| Raw camera scenarios | Eight direction/rate/policy cases pass: automatic acquisition, two distinct bounces, causal spin modes, plane policies and intentional frame dropout. |
| Identical source cutoff | Four installed-vs-library cases at both rates, known/unknown spin; current state/covariance and event mean/time/covariance agree within declared 1e-9/1e-8 tolerances. |
| Runner compatibility | Original decoder/builder tests and 15 tests compiled against observed upstream 3b3c402 headers pass. Two compatible ONNX artifacts, 110-D metadata and six finite CPU inferences pass separately. |

The final full ROS invocation passed 66/67 planner groups. Its raw-camera failure
was diagnosed as fixture-side forecast decoding pausing the camera across a
bounce. The corrected publish-only independently paced fixture and focused C++
group passed a targeted two-group rerun. The receipt merges the most recent
result **per test name** and retains every XML; it does not describe this as one
67/67 invocation. Earlier pilots/aborted stages remain archived, including an
old profile-string smoke expectation and small-sample coverage failures.

Fast positive 10/12 m/s cases were frozen at **ten scored flights** after pilots
with as few as eight eligible epochs. The original 5 m/s two-flight fixture is
retained. Conditions remain phase >100 ms, lead >130 ms, coverage >=95%, every
published source age <=30 ms and actual C++ decoder p95 <=30 ms. No slow/rejected
row is discarded. Across the 12 final FOLLOW_POSE fast profiles, coverage is
97.5–100% and worst decoder p95 is 4.882 ms. The four ON_REFRESH supplements
achieve 98.75–100% coverage. The full-trajectory default remains 50 Hz.
Synthetic source cyclic GC is disabled only during pacing; all sent intervals
are recorded. Evidence-reader history is 128 to retain callbacks; the production
worker remains bounded at one running/one pending.

20 m/s physical support is reported separately from command feasibility.
True net collision, uncertain contact order, held input, missing base and frame
faults cannot produce a valid command. Corrected incoming flyover may still
fail the runner impact/velocity contract. A native representable outgoing
edge-clearance case validates reconstructed impact, net passage and legal first
landing; no execution guard is overridden to manufacture validity.

## Local verification on 21 October 2026

Worktree: `/home/tpark/workspace/HOPE/integrated-v2-review-fixes`, on the same
corrections branch. The existing remote head `1e379f5` was inspected and
preserved. The local follow-ups add regression coverage, without changing
production sources, interfaces, configuration defaults or model coefficients:

- `6961cf1`: accepted ROS contact evidence revokes once, its duplicate does not
  revoke, and the contact fences a pending solve. Both normal and 12 ms delayed
  worker variants pass, alongside the existing rejected/no-op context checks.
- `be642ba`: an incoming plane four milliseconds after a bounce is rejected as
  `CONTACT_ORDER_UNCERTAIN` with configured bounce noise. The identical
  physical branch is `RETURN_ELIGIBLE` when that noise is explicitly disabled.
  The test covers 5 ms and 0.25 ms integration; it fails on the pinned baseline.

| Local gate | Executed result |
| --- | --- |
| Final focused fixture on actual baseline `3346e030` | 6/26 pass; 20 expected review failures retained. |
| Final corrected focused fixture | 26/26 pass in release and ASan/UBSan builds. |
| Corrected native and sanitizer CTest suites | 15/15 groups each. |
| Strict legacy numerical parity | 701/701 fixtures; maximum difference zero. |
| Independent outgoing reference | 144/144 accepted candidates; maximum position difference 3.294e-13 m and event-time difference 7.883e-14 s. |
| Installed ROS | `hope_msgs` 2/2, `hope_planner_cpp` 67/67 in one invocation, `hope_bringup` 24/26. |
| Inherited bringup lint | 3,086 flake8 and 37 pep257 failures; normalized failure multisets exactly match the local baseline. Aggregate verification exits 1. |
| Installed source/decoder matrix | All 30 cases pass: 18 fast speed/spin cases, eight geometry/held controls, four original plane-policy cases. |
| Fast FOLLOW_POSE command coverage | All twelve 10/12 m/s profiles achieve 100%; worst actual decoder P95 2.845 ms. All emitted command source ages remain within 30 ms. |
| 20 m/s execution separation | All six cases acquire and confirm one physical bounce; zero valid commands. Execution limits remain unchanged. |
| Source cutoffs and raw scenarios | Four library/installed cutoff comparisons and all eight updated raw-camera scenarios pass. |
| Compatible policies | Two distinct existing artifacts pass metadata checks and six finite CPU inferences; actual paddle tracking remains NOT_RUN. |
| Supplemental plane/expiry checks | Four fast ON_REFRESH cases pass at 150/300 Hz with 100% coverage; both 40 ms delayed-worker cases emit zero valid commands. |

The clean entry point started at `6961cf1`. After the initial 25-case native and
sanitizer runs, the additional uncertainty/admission test was compiled and the
complete 26-case focused suite rerun against baseline, corrected release and
corrected sanitizers. The installed ROS build includes that expanded fixture.
Production/configuration/interface bytes are identical between `6961cf1` and
`be642ba`; the receipt records both source pins, initial artifacts and follow-up
artifacts rather than overwriting an earlier execution.

The historical baseline's installed raw-camera fixture timed out at 120 seconds
on this host, yielding 36/37 planner groups. This remains a baseline failure in
the evidence. The corrected publish-only, bounded fixture passed; no baseline
timeout was relabeled as a pass. The original baseline native/sanitizer groups
pass apart from the intentionally injected review regression group.

Local raw evidence is under `/tmp/hope-pr64-local-20261021`; the JSON companion's
`local_verification` records exact source/config/binary hashes, commands, case
results and artifact locations. A curated 3.99 MB raw-evidence archive is retained
locally at `artifacts/pr64-local-20261021-evidence.tar.gz` (ignored generated
artifact; its SHA-256 is in the JSON). The optional policy check used ONNX Runtime
1.31.0 installed only under `/tmp/hope-pr64-policy-packages`; system Python was
unchanged. Historical cloud results above remain intact.
This verification closes the review regression endpoint, not the broader V3
simulation/physical gates or the separate upstream publication decision.

## Model and compatibility limits

CKF6 plus deterministic RK4/event rollout is retained. Covariance is conditional
PV uncertainty; uncalibrated spin/model/plane/clock uncertainty is not invented.
The net is a finite zero-thickness rectangle without poles. Near-tangent or
branch-ambiguous linearizations report their quality honestly.

Effective model: `thin-shell-forward-v4-pr64-causal-contact-finite-net-20261021`.
Dependent binaries must be rebuilt. Runner schema 2, observation/context schema 2
and forecast/packet schema 3 remain unchanged. Legacy YAML/defaults/goldens and
source files are unchanged. The standalone forecast executable remains forecast
only; the integrated opt-in path stays in the same single planner node and
publishes shadow commands. Live output is rejected at startup.

## Reproduction and artifacts

In ROS Jazzy with Eigen/GTest/SciPy, from this worktree:

```bash
source /opt/ros/jazzy/setup.bash
bash hope_ws/src/hope_planner_cpp/scripts/flight_prediction_validation/verify_pr64_review.sh \
  /tmp/hope-pr64-new-evidence
```

Use a new absolute evidence directory. This archives exact baseline 3346e030,
injects the current focused tests with the baseline compatibility define, then
runs clean baseline and corrected native/sanitizer/numerical/ROS verification.
The current fixture has 25 cases; the archived initial baseline run had 20.
Expected baseline regressions and inherited corrected lint yield nonzero exits;
inspect case results rather than treating aggregate nonzero as unexplained.
The policy-artifact and extra ON_REFRESH/expiry commands are recorded in the
cloud closure script and receipt as supplementary executions.

Evidence root: `/workspace/.cloud-setup/pr64-review`. The JSON records full
paths/hashes. `qualification-85d22fd/baseline` is the clean baseline;
`retest-912d456` contains full-suite results using the shared clean-build cache;
`closure` contains final native/sanitizer, targeted ROS, raw and supplementary
results. Cache directory names describe origin, not the final source hash.
Earlier stage receipts are historical and excluded from final acceptance.

[Publication options and proposed reviewer responses](integrated_planner_pr64_publication_options.md)
cover the unchanged old PR64. [Remaining qualification](integrated_planner_review_remaining_qualification.md)
tracks broader V3 gates. `software_complete`, `simulation_qualified`,
`physically_qualified` and `deployment_authorized` all remain false.
