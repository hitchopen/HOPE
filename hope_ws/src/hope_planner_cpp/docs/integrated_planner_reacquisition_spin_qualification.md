# Reacquisition, spin readiness and base-contract qualification

The P1/P2/base-contract corrections are implemented. **Full software qualification
is still open:** two nominal 300 Hz runs missed the frozen 95% command-coverage
threshold. Both exact retries passed, but the original failures remain failures.
`software_complete`, `simulation_qualified`, `physically_qualified` and
`deployment_authorized` are **false**. No robot was actuated.

## Source and scope

- Exact continuation base: `e9bf01579123eca35ce315b0f605814f8c952255`.
- Clean full-suite source: `a1f2f7a7e654b9ccd247f247cd50b6bc2910ad7a`.
- Final code/test correction source: `f089137eec49e05fd2b29cbc4f0f75dc76f580c0`.
  This adds explicit-zero ROS decoding and corrects verification instrumentation.
  The affected transport/cutoff/graph tests were rerun; the entire full suite was
  not rerun after these changes. The native physics/CKF/rearm code is unchanged.
- Local branch: `feature/integrated-planner-reacquisition-spin-readiness`.
- Original HOPE, earlier review-fixes and nohope worktrees remain untouched.
  Upstream changes were fetched and audited, not silently merged into the pinned base.
- PR64 remains **closed, unmerged**, at historical head `9118a491`; no PR was
  reopened, retargeted or published. The new PR description is a local draft.

Audited HOPE upstream main is `8d24642474205f6cfb48263cd22896306feeac47`,
upstream nightly is `91aecabfd44cb258ac998158e724be9017d883a7`, and fork nightly
is `6616b1df795b6b5db242435fafc394295e0b45cf`. New upstream asset/serve/home/model
changes are separate from this implementation. nohope remains
`96e71c1adb217d14b925681173cebe00d8427f21`; its parent main is
`c2a627f26c957a079b5cc2586ab6cf49aae4a221` (3 parent-only / 9 fork-only commits).

The existing CKF6, deterministic physics coefficients, residual/uncertainty
thresholds, legacy solver/goldens/default YAMLs, policy artifacts and schemas
remain intact: runner schema 2, observation/context schema 2, forecast/packet
schema 3. The integrated profile enables automatic rearm; standalone defaults
it off. Source-qualified gap or held-to-launch evidence is required after a
rejection. Continuous rejected motion and wall-clock cooldown cannot rearm.
Candidate angular history is bounded independently of the short dynamic window
and queried no later than its midpoint. A supported position-only forecast may
start before measured spin. The actual base consumer now reads WXYZ. Explicit
valid producer zero survives ROS decoding; absent/nonfinite input stays invalid.

## Gates and results

| Gate | Result |
| --- | --- |
| G0 | PASS: clean baseline, named E01/E02/E03 failures, controls, frozen criteria |
| G1 | PASS: RA01–RA08, fresh generation/origin/CKF/events/spin and old-worker fences |
| G2 | PASS: SP01–SP09, causal quaternion-only spin, early position-only, true zero |
| G3 | PASS: BP01–BP05, real producer, rotated bases, actual decoder/110-D builder |
| G4 | Functional recovery PASS; timing NOT_FULLY_QUALIFIED (one new matrix case) |
| G5 | NOT_FULLY_QUALIFIED: two first-run coverage failures; corrections/retries below |
| G6 | NOT_RUN: native Runner and instrumented plant absent; evaluation-only Arm B outstanding |
| G7 | NOT_RUN: real recordings inventoried, independent robot/clock/impact evidence outstanding |

Before production edits, the exact `e9bf015` baseline passed 15 native groups,
15 ASan/UBSan groups, 26 historical review cases and 701 exact legacy parity
cases. ROS had 2/2 message groups, 66/67 planner groups, and 24/26 bringup groups.
The raw-scenario fixture failed twice: cold first publication delayed its source
by about 283 ms and omitted the first bounce. Its correction warms an explicitly
invalid observation before scored exposure; existing physics/assertions remain.
The corrected raw-scenario test passed the full run.

E01/E02 fail-before counterexamples reproduce rejection latching and in-range
quaternion-only acquisition starvation. E02 uses 30 m/s and 100 rad/s, with the
unchanged 20 rev/s limit, and a direct-omega positive control. An exploratory
300 rad/s case exceeded that limit and is **excluded**, with logs retained.
E03 compiles the actual baseline consumer expression against values emitted by
`pose_to_base_flat`: identity passes while ±30° and +50° reveal the defect.
`test/reacquisition_baseline_probe.py` reproduces these failures from the pinned
Git source and requires the named positive controls to pass.

The corrected full run passed **18/18 native groups**, **18/18 ASan/UBSan groups**,
all **26 historical review cases**, **17 RA/SP cases**, **5 BP cases**, and the
four before/after counterexample cases. The actual Python base producer passed
8 sign/yaw combinations. Two distinct policy artifacts passed 110-D metadata,
world-frame/term semantics and six finite CPU inference cases; this is not a
native Runner inspection or achieved paddle tracking claim.

All **701 legacy parity cases** matched with maximum absolute difference zero.
All **144 outgoing candidates** passed the independently integrated reference,
now explicitly `method='DOP853'`, rtol=2e-12, atol=2e-14. Maximum landing-position
error was 1.346568145197644e-12 m, event-time
error 8.141265439576273e-13 s, and analytic
impact closure error zero. The old reference label said DOP853 while invoking
SciPy's default RK45; that historical result is not relabeled as a DOP853 run.

The initial full ROS run had **96/100 planner groups pass**. Four failures are
retained:

1. `reacquisition_chain_300_FOLLOW_POSE_unknown`: 148/160 eligible epochs, 92.5%.
2. `review_chain_300_10_zero`: 146/160, 91.25%.
3. `review_source_cutoff_parity`: a 23.44 ms source pause followed by a 0.38 ms
   interval exposed a different consumed input prefix at the depth-one receiver.
   The corrected comparator reconstructs the actual prefix from checkpoint
   observation ledgers, retains the complete published stream and lists missing
   rows/uncovered tail rows. State/covariance/event tolerances are unchanged.
4. `flight_v2_ros_transport`: the exact output allowlist omitted the newly added
   shadow readiness diagnostic. It now names that exact topic; control-topic
   prohibitions remain.

The two corrected tests and the full ROS transport unit group passed together
(3/3). The older cutoff comparator now runs six cases, including direct valid
zero at both rates. The four rejection/rearm quaternion-only cutoff cases and
scope check then passed (2/2). All matched current state/covariance/event errors
were zero. These checks also validate generation, source origin and angular
readiness at identical source cutoffs.

Bringup's two failures are **unchanged baseline lint**: flake8 has 3,086 findings
and pep257 has 37. Their exact finding multisets match the baseline. The full
verification command keeps its nonzero aggregate exit; no lint or new failure
was suppressed.

## Installed repeated-flight matrix

Each process uses the installed raw MCT and base relays, one selected integrated
planner, the real schema-2 packer and the actual C++ mailbox decoder. No context
publisher or live command publisher is present. All registration and plant truth
are synthetic. Ten A/B rounds mix genuine rejection, gap-based recovery and
held-to-launch recovery, with source/frame/order faults retained in raw records.
The 750 ms first-worker delay forces an A solve to finish after B starts.

A post-run raw-ledger audit proves **140 A acquisitions and under-table
rejections**, **140 B acquisitions with decoded commands**, and fresh B origins
and spin measurement sources across all initial attempts, including the failed
coverage case. Both delayed cases observe retired-generation work discarded.
Candidate/retained histories and one-running/one-pending work stay bounded.
The failed case has no resource snapshot because its coverage assertion fired
before that snapshot; it is not assigned an invented memory result.

| Hz | Plane policy | Spin | First-worker delay ms | Covered / eligible | Decoder p95 ms | Initial result |
| --- | --- | --- | --- | --- | --- | --- |
| 150 | FOLLOW_POSE | quaternion_only | 0 | 79/79 (100.00%) | 3.429 | PASS |
| 150 | FOLLOW_POSE | unknown | 0 | 80/80 (100.00%) | 4.006 | PASS |
| 150 | FOLLOW_POSE | zero | 0 | 80/80 (100.00%) | 3.388 | PASS |
| 150 | ON_REFRESH | quaternion_only | 0 | 80/80 (100.00%) | 4.017 | PASS |
| 150 | ON_REFRESH | unknown | 0 | 80/80 (100.00%) | 3.708 | PASS |
| 150 | ON_REFRESH | zero | 0 | 80/80 (100.00%) | 3.418 | PASS |
| 150 | FOLLOW_POSE | unknown | 750 | 80/80 (100.00%) | 4.384 | PASS |
| 300 | FOLLOW_POSE | quaternion_only | 0 | 150/150 (100.00%) | 2.840 | PASS |
| 300 | FOLLOW_POSE | unknown | 0 | 148/160 (92.50%) | 6.946 | FAIL_COVERAGE |
| 300 | FOLLOW_POSE | zero | 0 | 160/160 (100.00%) | 3.612 | PASS |
| 300 | ON_REFRESH | quaternion_only | 0 | 147/150 (98.00%) | 3.896 | PASS |
| 300 | ON_REFRESH | unknown | 0 | 159/160 (99.38%) | 3.537 | PASS |
| 300 | ON_REFRESH | zero | 0 | 158/160 (98.75%) | 4.919 | PASS |
| 300 | FOLLOW_POSE | unknown | 750 | 158/160 (98.75%) | 4.229 | PASS |

The frozen valid-publication source-age ceiling and decoder p95 ceiling are
30 ms; coverage is at least 95%. Eligibility uses independently generated source
opportunities with >100 ms elapsed and >130 ms lead. Full source intervals,
exposures, faults, all commands, decoder receipts and denominators are retained.
Failures were not removed from denominators or recast as positive tests.

The new failed timing case recorded a 20.79 ms source interval followed by a
catch-up burst; nine of its twelve missing command epochs were also missing at
the typed-observation recorder. The older failed case recorded a 22.91 ms source
interval and an observed expired input age of 42.53 ms. This demonstrates timing
variability and correct expiry rejection, not proof that every loss was outside
the planner. One exact retry per failed case, using the original full-suite
binaries and scripts, achieved 160/160 (100%). Both first failures remain open
for timing qualification; retries do not establish a hard real-time guarantee.

## Named correction coverage

| ID | Executed regression | Result |
| --- | --- | --- |
| RA01 | AlreadyRejectedThenGapNeedsMultipleNewSamples | PASS |
| RA02 | QuietHeldReleaseWithoutSourceGap | PASS |
| RA03 | ContinuousRejectedMotionAndDirectionChangeStayQuarantined | PASS |
| RA04 | ProvenanceAndClockFaultCannotAuthorizeNewEpoch | PASS |
| RA05 | NewPosteriorMatchesFreshOwnerWithNoOldSpinOrContacts | PASS |
| RA06 | RetiredSnapshotCannotPublishIntoNewGeneration | PASS |
| RA07 | TenRejectRearmCyclesRemainBounded | PASS |
| RA08 | BothRatesDirectionsSpeedsAndJitterHavePhysicalForecasts | PASS |
| SP01 | FullPositionOnlyShotNeverClaimsMeasuredZero | PASS |
| SP02 | QuaternionOnlyCollectsBeforeMeasuredAtBothRates | PASS |
| SP03 | MagnusAcquisitionActuallyUsesCausalQuaternionSpin | PASS |
| SP04 | PositionOnlyStartsPromptlyThenSameGenerationAdoptsSpin | PASS |
| SP05 | HeldOrientationExcludedFromLaunchAngularWindow | PASS |
| SP06 | UntrustedInvalidReorderedAndSignAliases | PASS |
| SP07 | ValidZeroAndMissingRemainDistinct | PASS |
| SP08 | DelayedBounceDropoutAndPostOnlyRecovery | PASS |
| SP09 | BoundedAttemptsAndUnsupportedUnknownSpin | PASS |
| BP01 | WxyzRoundTripAndHeading | PASS |
| BP02 | AnalyticTurnGateAtRotatedYaw | PASS |
| BP03 | PitchRollAndSignAreEquivalent | PASS |
| BP04 | InvalidStaleWorldAndReorderedInputsFailClosed | PASS |
| BP05 | ActualPackerDecoderAnd110DHeightOnceAtRotatedBase | PASS |

SP07 additionally passes the real ROS encode/decode path with explicit zero,
invalid flags and nonfinite vectors. Base yaw 0°, ±30° and +50° runs at both
rates establish allowed rotations and nonvacuous turn rejection. The original
net, contact uncertainty, held, 10/12/20 m/s, context races, startup guards,
receiver lifecycle and rollback tests remain present.

## Remaining qualification and evidence

Native Gate3 `--check` exited 2 because the configured
`a3_deploy_onnx_ref_pingpong` package is absent. The instrumented AimRT MuJoCo
install is also absent. Python MuJoCo/ONNX packages are not those runtime assets.
G6 still needs the non-live V2-incoming/legacy-outgoing evaluation adapter and
three-arm actual contact, achieved face-normal/velocity, landing, net clearance,
admission/miss/duplicate and stability/recovery scoring. No force-every-flight
or known-answer target substitution was used.

Real data is available upstream: all nine compressed/uncompressed PPQ004
C3D/CSV/NPZ hashes and lengths matched its manifest at upstream main `8d246424`.
It is 300 Hz table-corner data; parts 1+2 train and part 3 holds out 27 arcs,
24 causally scoreable arcs and only three qualified spin/contact holdouts.
Those are available inputs, not a continuation-planner accuracy result.
Relative capture times do not certify cross-host clock mapping or surveyed
pelvis/table/tool geometry. No newer coefficients, optional runner schemas 4/5,
physical model qualification or operator promotion was silently adopted.

The machine-readable companion records all pins, thresholds, case names,
initial/retry outcomes, file hashes, environment and explicit false qualification
flags. Raw evidence is under `/workspace/.cloud-setup/reacquisition-spin/`:
`baseline-e9bf015/`, `qualification-verified/`, `development/`, the two timing
retry directories, baseline probes and failed exploratory runs. The interrupted
`qualification-final/` run contains a test-constructor compile error fixed before
the clean full run; it is retained, not counted as a passing run.

Reproduce the full ladder inside Ubuntu 24.04 / ROS Jazzy with Eigen/GTest,
NumPy/SciPy and CPU ONNX Runtime:

```bash
bash hope_ws/src/hope_planner_cpp/scripts/flight_prediction_validation/verify_integrated.sh   /new/absolute/evidence-directory
```

`HOPE_POLICY_PYTHONPATH` may name an external ONNX Runtime dependency directory.
Every gate preserves its log and exit code. No existing evidence directory is
overwritten. The environment here used GCC 13.3, CMake 3.28.3, Python 3.12.3,
NumPy 1.26.4, SciPy 1.11.4, ONNX Runtime 1.31.0 and isolated localhost ROS domain
193 (specialized tests use their declared isolated domains).

The verified cloud evidence bundle is 15579488 bytes and contains 1575 files:
`/workspace/.cloud-setup/reacquisition-spin/reacquisition-spin-evidence.tar.gz`. Its SHA-256 is
`fb23fd72ccdff4bb38e5c9820937ad1ee22421f53c15421cbeecb067c42ea3c8`. The internal manifest hashes every evidence file.
The archive identifies compiled binaries by SHA-256 and retains build commands;
it does not bundle executable binaries. The archived receipt predates only this
packaging metadata, avoiding a self-referential archive hash.
