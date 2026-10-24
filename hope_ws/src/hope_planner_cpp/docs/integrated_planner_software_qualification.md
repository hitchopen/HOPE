# PR85 software qualification — not qualified

Tested source: `dc012fe970d6bb2169853dd8e985705193ac9148`; tree `0a57e50a2508717421673d16904a1dcac3d62b20`.

**Step 1 passed its integration checks. Step 2 did not pass software qualification.** The full clean run failed `base_yaw_chain_150_0`: 39/43 independently eligible updates (90.6977%, required ≥95%). The original verifier aggregate remains 1. No passing retry replaces this failure.

The separately continued, predeclared 160-trial campaign completed 157 passing and 3 failing trials on the same frozen source and binaries. Its result does not erase the failed full invocation.

## Integration

Ordinary merge `e7947b9` reconciled existing PR85 with `hitchopen/HOPE:nightly_built` at `91aecabfd44cb258ac998158e724be9017d883a7`. Native Git found 18 conflicts: 12 content and 6 add/add. The separate integration receipt includes every path, original index/stage blobs, chosen resolution and auto-merge compatibility fix. The current upstream runtime-plane/session-home behavior, dependencies, receiver semantics and deployment changes were preserved. V2 remains opt-in and shadow-only. Final target-relative scope contains 175 paths.

## Measured corrections and remaining uncertainty

Source state and acquisition stamp now describe one epoch, correcting a measured construction gap up to 2.106 ms. Default-off, bounded ledgers record actual relay/planner/worker/publication/receiver stages. Publication freshness uses completion time. Missing recorder data alone is never called a production drop.

A controlled actual-DDS callback pause demonstrated physical-rejection loss with depth-one ingress. The integrated profile now uses bounded depth eight at raw and typed ingress; legacy relay default remains one. Regressions cover both rates and invalid history bounds. Consumed-prefix parity replays the actual ordered pipeline ledger, including rejections; independent offered coverage retains every eligible exposure.

Closed-context C++ graph-query and Python subscription-take shutdown failures have deterministic before/after regressions. Active-context errors still propagate. Failure and partial-startup paths retain receipts and reap launched children.

V2 uses one executor for its mutually exclusive callback group plus its bounded worker; legacy retains three executor threads. Paired diagnostics measured 23–28% lower median planner CPU. Both variants passed those diagnostics, which do not uniquely explain historical loss. Admission reserves 5 ms for publication after a measured 4.431818 ms late-admission completion delay. The 30 ms completion and 95% coverage limits remain unchanged, and rejection still counts against coverage. This reserve is empirical, not a guarantee under arbitrary scheduling suspension.

In the final 150 Hz base-yaw failure, three missing exposures occurred in a catch-up burst after a source delay, and one was paced normally. Cyclic GC was already disabled. Actual-stage accounting was off, so the first lost production stage and OS/DDS/planner attribution remain unresolved. All four losses remain in the denominator. There is insufficient evidence for another source correction; the candidate is delivered as unqualified.

## Exact-source verification

| Gate | Outcome |
|---|---|
| native-test | exit 0 |
| sanitized-test | exit 0 |
| outgoing-reference | exit 0 |
| legacy-parity | exit 0 |
| historical-legacy-parity | exit 0 |
| premerge-v2-parity | exit 0 |
| ros-build | exit 0 |
| ros-result-audit | exit 1 |
| diagnostic-comparison | exit 0 |
| repeated-campaign | exit 77 |
| aggregate | exit 1 |
| hope_msgs ROS | 2/2 passed; failures: none |
| hope_planner_cpp ROS | 106/107 passed; failures: base_yaw_chain_150_0 |
| hope_bringup ROS | 25/27 passed; failures: flake8, pep257 |

Native and ASan/UBSan each passed 20/20 groups. Both historical/current-upstream legacy comparisons passed 701 cases; DOP853 passed 144 candidates; premerge V2 parity passed 789 cutoffs. Installed receiver/identity, one-node/shadow, spin/base semantics, consumed-prefix, geometry/fault/expiry and actual 110-D policy-builder checks ran. Policy verification includes metadata and finite CPU inference, not robot motion.

All 136 registered ROS groups executed. Bringup has two inherited failing lint categories: flake8 3222 findings versus upstream 3232, pep257 37 versus 37, with no added normalized findings. The new planner coverage failure is separate. Eight diagnostic on/off trials passed. The verifier skipped its repeated campaign with exit 77 because the ROS audit failed; an explicitly recorded separate continuation then ran all 160 trials on identical source/contract/binaries. This is not a coherent passing full invocation.

## Every declared repeated trial

Five fresh processes per profile, no retries or pooled pass rates. Every trial requires ≥95% independent eligible coverage, every valid publication ≤30 ms and actual decoder P95 ≤30 ms. Eligibility remains after 100 ms and with >130 ms lead, with original predeclared fault exclusions. Established ten-flight/round profiles retain ten. The table includes failures; the JSON companion retains all exceptions, denominators, resources and raw-evidence paths. Latency statistics describe retained observations; a complete-publication claim also requires command counters, unique trace identities and wire identities to agree, as recorded separately for every trial.

| Profile | Rep 1 | Rep 2 | Rep 3 | Rep 4 | Rep 5 | Max decoder P95 ms | Max publication ms |
|---|---:|---:|---:|---:|---:|---:|---:|
| reacquisition_chain_150_FOLLOW_POSE_unknown | 80/80 PASS | 80/80 PASS | 80/80 PASS | 80/80 PASS | 80/80 PASS | 3.182 | 7.531 |
| reacquisition_chain_150_FOLLOW_POSE_quaternion_only | 80/80 PASS | 80/80 PASS | 79/80 PASS | 80/80 PASS | 80/80 PASS | 3.171 | 16.616 |
| reacquisition_chain_150_FOLLOW_POSE_zero | 80/80 PASS | 80/80 PASS | 80/80 PASS | 80/80 PASS | 80/80 PASS | 3.797 | 8.423 |
| reacquisition_chain_150_ON_REFRESH_unknown | 80/80 PASS | 80/80 PASS | 70/70 PASS | 80/80 PASS | 80/80 PASS | 3.044 | 5.757 |
| reacquisition_chain_150_ON_REFRESH_quaternion_only | 80/80 PASS | 80/80 PASS | 80/80 PASS | 80/80 PASS | 80/80 PASS | 3.403 | 10.182 |
| reacquisition_chain_150_ON_REFRESH_zero | 80/80 PASS | 80/80 PASS | 80/80 PASS | 80/80 PASS | 80/80 PASS | 3.268 | 8.036 |
| review_refresh_150_10_unknown | 80/80 PASS | 80/80 PASS | 80/80 PASS | 80/80 PASS | 80/80 PASS | 3.782 | 10.163 |
| review_refresh_150_12_known | 20/20 PASS | 20/20 PASS | 20/20 PASS | 20/20 PASS | 20/20 PASS | 3.854 | 5.951 |
| reacquisition_chain_300_FOLLOW_POSE_unknown | 160/160 PASS | 160/160 PASS | 160/160 PASS | 160/160 PASS | 160/160 PASS | 2.693 | 11.189 |
| reacquisition_chain_300_FOLLOW_POSE_quaternion_only | 150/150 PASS | 150/150 PASS | 150/150 PASS | 150/150 PASS | 150/150 PASS | 3.299 | 15.663 |
| reacquisition_chain_300_FOLLOW_POSE_zero | 160/160 PASS | 160/160 PASS | 160/160 PASS | 159/160 PASS | 159/159 PASS | 3.258 | 8.518 |
| reacquisition_chain_300_ON_REFRESH_unknown | 160/160 PASS | 160/160 PASS | 160/160 PASS | 160/160 PASS | 160/160 PASS | 2.583 | 6.830 |
| reacquisition_chain_300_ON_REFRESH_quaternion_only | 150/150 PASS | 150/150 PASS | 150/150 PASS | 150/150 PASS | 149/149 PASS | 2.848 | 7.869 |
| reacquisition_chain_300_ON_REFRESH_zero | 160/160 PASS | 160/160 PASS | 160/160 PASS | 160/160 PASS | 160/160 PASS | 3.056 | 9.195 |
| review_refresh_300_10_unknown | 160/160 PASS | 160/160 PASS | 160/160 PASS | 160/160 PASS | 149/160 FAIL | 3.784 | 23.065 |
| review_refresh_300_12_known | 40/40 PASS | 40/40 PASS | 40/40 PASS | 40/40 PASS | 40/40 PASS | 3.991 | 7.596 |
| review_chain_150_10_unknown | 80/80 PASS | 80/80 PASS | 80/80 PASS | 80/80 PASS | 80/80 PASS | 4.143 | 8.256 |
| review_chain_150_10_zero | 80/80 PASS | 80/80 PASS | 80/80 PASS | 80/80 PASS | 80/80 PASS | 4.726 | 8.814 |
| review_chain_150_10_known | 80/80 PASS | 80/80 PASS | 80/80 PASS | 80/80 PASS | 80/80 PASS | 4.186 | 9.047 |
| review_chain_150_12_unknown | 20/20 PASS | 20/20 PASS | 20/20 PASS | 20/20 PASS | 20/20 PASS | 4.477 | 6.674 |
| review_chain_150_12_zero | 20/20 PASS | 20/20 PASS | 20/20 PASS | 18/20 FAIL | 20/20 PASS | 5.577 | 20.782 |
| review_chain_150_12_known | 20/20 PASS | 20/20 PASS | 20/20 PASS | 20/20 PASS | 20/20 PASS | 3.554 | 4.527 |
| integrated_chain_150_FOLLOW_POSE | 40/41 PASS | 40/41 PASS | 40/41 PASS | 40/41 PASS | 40/41 PASS | 3.578 | 4.964 |
| integrated_chain_150_ON_REFRESH | 40/41 PASS | 40/41 PASS | 40/41 PASS | 40/41 PASS | 40/41 PASS | 3.945 | 7.576 |
| review_chain_300_10_unknown | 160/160 PASS | 160/160 PASS | 160/160 PASS | 160/160 PASS | 160/160 PASS | 2.642 | 5.153 |
| review_chain_300_10_zero | 160/160 PASS | 160/160 PASS | 160/160 PASS | 160/160 PASS | 160/160 PASS | 3.053 | 7.290 |
| review_chain_300_10_known | 150/150 PASS | 150/150 PASS | 150/150 PASS | 150/150 PASS | 150/150 PASS | 2.953 | 8.568 |
| review_chain_300_12_unknown | 37/40 FAIL | 40/40 PASS | 40/40 PASS | 40/40 PASS | 40/40 PASS | 3.512 | 20.012 |
| review_chain_300_12_zero | 40/40 PASS | 40/40 PASS | 40/40 PASS | 40/40 PASS | 40/40 PASS | 3.402 | 6.835 |
| review_chain_300_12_known | 40/40 PASS | 40/40 PASS | 40/40 PASS | 40/40 PASS | 40/40 PASS | 3.241 | 9.845 |
| integrated_chain_300_FOLLOW_POSE | 81/82 PASS | 81/82 PASS | 81/82 PASS | 81/82 PASS | 81/82 PASS | 4.330 | 16.327 |
| integrated_chain_300_ON_REFRESH | 80/82 PASS | 81/82 PASS | 81/82 PASS | 81/82 PASS | 81/82 PASS | 3.286 | 5.657 |

## Retained history

- Original 92.5%/91.25% failures and retries remain historical; complete actual-stage attribution is unavailable.
- `63790a3`: all 134 ROS groups ran, planner 101/105; four failures retained; campaign not started.
- `909b268`: native passed; cancelled before ROS/scored trials to add complete-publication integrity checks.
- `3f0f7fc`: native/sanitizer/ROS passed except inherited lint; campaign stopped after 63 recorded outcomes (62 pass and one relay shutdown failure), one further completed raw trial and 96 not run.
- `4065cb5`: complete ladder, planner 105/107; quaternion 300 Hz coverage 138/150 and one T2 publication at 32.525202 ms. Campaign not started.
- Initial contaminated pilots, setup errors, partial traces and all later diagnostics are retained and labeled. No historical failure is replaced by a later pass.

## Acceptance and delivery

| ID | Status | Evidence |
|---|---|---|
| M01 | PASS | Pinned original head/base/common ancestor; original worktrees preserved |
| M02 | PASS | 18 conflicts: 12 content and 6 add/add; original index/stage blobs retained |
| M03 | PASS | Per-path conflict ledger records intent, risk, resolution and verification |
| M04 | PASS | Auto-merge runtime-plane linkage, dependencies, source/API/lifecycle reviewed |
| M05 | PASS_WITH_INHERITED_LINT | Clean upstream/integration builds and functional checks |
| M06 | PASS | 701 selected-upstream legacy cases; 789 V2 cutoffs; 144 outgoing cases |
| M07 | PASS | 175-path final scope inventory and protected source guard |
| M08 | PASS | Ordinary checkpoint pushed to existing PR85; final live status in final-delivery.json |
| CL01 | PASS | Immutable contract, source/base/tree, source and binary hashes |
| CL02 | PASS_WITH_LIMITS | Original archive byte-identical; all available failed runs retained; old missing ledgers disclosed |
| CL03 | PASS_WITH_LIMITS | Actual bounded stage ledgers and eight on/off trials; accounting-disabled losses explicitly unresolved |
| CL04 | PASS | Independent offered denominator, per-ball recovery and consumed-prefix parity remain separate |
| CL05 | PASS | Failure/partial-startup regressions pass; final failure retained raw/resources/exception |
| CL06 | PASS_WITH_LIMITS | Measured bounded corrections and before/after tests; residual scheduling loss not uniquely diagnosed |
| CL07 | PASS | All five declared T1 outcomes below |
| CL08 | PASS | All five declared T2 outcomes below |
| CL09 | FAIL | All 160 declared outcomes; no retries credited |
| CL10 | PASS | Every repeated reacquisition profile plus native counterexamples and delayed-generation cases; per-ball recovery remains separate from epoch coverage |
| CL11 | PASS | Installed geometry/turn, stale/reordered, missing-base, 40 ms expiry and 750 ms retired-work gates pass |
| CL12 | FAIL | SP/BP semantic and policy-builder checks pass; base_yaw_chain_150_0 coverage fails 39/43 |
| CL13 | PASS | RV, 144 DOP853 candidates, impact/covariance, two 701-case parity comparisons and 789 V2 cutoffs pass |
| CL14 | PASS_WITH_LIMITS | Parent quota/cpuset, environment and on/off overhead pinned; no cross-host or hard-real-time claim |
| CL15 | PASS | Target-relative scope manifest; no deployment rollback; protected source guard passes |
| CL16 | FAIL | Full clean invocation failed ROS coverage; its campaign was skipped 77, then separately completed on unchanged binaries |
| CL17 | PASS_WITH_DISCLOSED_FAILURES | Raw aggregate 1 retained; new ROS failure and exact inherited lint delta disclosed |
| CL18 | PASS | Evidence branch remote commit verified; archive/part/member hashes pass. Submitted documentation-only head equivalence is recorded in final-delivery.json. |
| CL19 | PASS | Software qualification false; simulation, physical qualification and deployment false |
| CL20 | PARTIAL_PR_BODY_WRITE_BLOCKED | Existing PR85 source branch updated; final submitted head and live API status recorded in final-delivery.json. GitHub rejects PR body PATCH with 403 Resource not accessible by integration; PR85_BODY.md published for manual application. |

[Portable evidence, checksum manifests and final delivery proof](https://github.com/timothyypark/HOPE/tree/evidence/pr85-software-qualification-20261024). Archive parts reconstruct byte-identical tar files, with per-member hashes; the original historical archive retains SHA-256 `fb23fd72ccdff4bb38e5c9820937ad1ee22421f53c15421cbeecb067c42ea3c8`.

ROS Jazzy, Ubuntu 24.04, GCC 13.3 and Fast DDS on localhost. The parent cgroup has four CPU equivalents and cpuset 0–4; the child container’s unlimited local quota does not override it. Source/config/test/interface hashes, tested binary hashes, exact commands, dependencies and continuous resource samples accompany the evidence. No build or compression ran concurrently with scored trials.

All software qualification flags remain false because the complete gate set did not pass. Simulation qualification, physical qualification and deployment authorization also remain false. No robot actuation or upstream merge was performed. PR body write access currently returns GitHub 403; the evidence branch includes a ready-to-apply body and records the final API result.
