# HOPE V2: simulation verification and evidence-driven corrections

Date: October 30, 2026

## Objective and current baseline

Qualify the existing shared V2 incoming prediction, conversion to the existing racket planner, continuous forecast, and pre-engagement revocation using software tests and the supplied simulations. Hardware, physical deployment, and measured physical-flight accuracy are outside this iteration.

Start from `feature/v2-incoming-validity-continuous-dropin`, currently `0fb3542d77129d42ad04099bbb5296f816841e8b`, after checking remote changes and the worktree. The recorded complete qualification used `369dd6c`; subsequent changes were a source fixture and documentation. Preserve the existing evidence branch and all original results.

This is a plan; no runtime or test implementation has been changed by its preparation.

## Revised acceptance policy

- The user accepts the observed latency result. Record the 54.731 ms continuous-forecast publication as an accepted timing deviation; do not undertake performance optimization solely to satisfy the previous 30 ms publication limit.
- Keep measuring publication age, receiver latency, source cadence, and contact lead time. 300 Hz means approximately 3.33 ms between observations; the previous 30 ms gate measured observation-to-publication age. Neither is a fixed delay after net crossing.
- Timing acceptance does not extend an individual message's expiry or authorize a command from invalid evidence. Preserve runtime freshness, identity, ordering, and revocation checks. If those checks cause a missed useful opportunity, investigate its effect under the coverage assessment below.
- Coverage below the old per-observation threshold is a finding to investigate, not sufficient evidence by itself that runtime code needs changing. Every original failure profile must receive an evidenced disposition; the previous requirement to fix all four by forcing the old numerical gate to pass is superseded by this assessment.
- A demonstrated harmless missed update requires no runtime correction. A demonstrated lost usable command or incorrect conversion requires a focused fix. Inconclusive cases remain open.
- Preserve original raw verdicts. Publish a separate current assessment with statuses such as PASS, ACCEPTED_TIMING_DEVIATION, VERIFIED_NO_FUNCTIONAL_IMPACT, CONFIRMED_DEFECT, and INCONCLUSIVE. Do not relabel all historical tests as passing.

## Phase 1: freeze scenarios and define useful output

1. Check current Git/GitHub heads, retain the baseline, and inventory available simulation entry points, models, dependencies, and reference trajectories. Identify which supplied simulations cover ball truth, racket-command interpretation, and actual simulated racket contact. Do not claim a simulator ran merely because its files exist.
2. Before scored runs, freeze scenario IDs, seeds, configurations, repetition counts, expected outcomes, and numeric tolerances. Obtain tolerances from existing simulator contracts/reference cases where available; document proposed values and resolve any materially ambiguous requirement before scoring.
3. Define an opportunity from independent simulation/source evidence, not from successful planner receipts. Retain offered observations and losses at every stage.
4. Keep distinct measures: eligible observation coverage; coverage of physical flights; consumer-visible usable commands; correct abstention on invalid flights; and numerical/visual correctness of the resulting racket target.
5. Specify each backend's output contract separately. The new incoming path permits one immutable command per committed flight. The existing repeated-update backend must be assessed against its own update contract; its per-observation losses cannot automatically be equated to missing one-shot commands.

Deliverable: a versioned scenario/acceptance manifest, including the approved timing disposition and a definition of useful command coverage.

## Phase 2: determine whether missing outputs matter

Analyze every retained failure from the complete ROS run, the 160-trial existing-backend campaign, and the 90-trial incoming campaign. Include all four original profiles explicitly, particularly `review_chain_300_12_unknown` (36/39 eligible epochs), plus the six missing incoming commands. Do not sample only the easiest failures.

For each case, join source acquisition, relay delivery, packet formation, typed observation history, coordinator state, pending work, worker execution, publication, and actual receiver decoding using source timestamps and flight/HOME identities.

Answer these questions per flight/opportunity:

1. Did the simulator establish a physically valid and reachable contact under the declared plane and command contract?
2. Was that opportunity represented by qualified input, and where was any input lost or rejected?
3. Was a usable command for that same flight already available to the consumer? Did it remain valid and sufficiently early for the simulator's contact contract?
4. Did the missing update change contact, violate a required update contract, conceal invalidation, or leave the flight without a usable command?

Use deterministic replay for causal correctness and separate real-time ROS runs for delivery behavior. Build focused reproductions for observed missing typed history, missing packet formation, source expiry, and pending-job supersession. Use controlled disturbances where needed, with both undisturbed controls and unchanged opportunity accounting.

Classification rules:

| Finding | Action |
|---|---|
| A replaceable observation update was skipped, while the same flight retained a valid command and met the declared simulated outcome/update contract | Record VERIFIED_NO_FUNCTIONAL_IMPACT; no runtime change |
| Invalid or deliberately unqualified input correctly caused abstention | Verify the scenario's expected abstention; no runtime change |
| A valid flight lost its only usable command because qualified information was dropped, expired in transit, or never processed | Confirm a functional defect and fix the first demonstrated failure stage |
| A skipped update concealed revocation or permitted obsolete authority | Confirm a correctness defect and add a focused regression |
| Evidence cannot establish the consumer-visible effect | Keep INCONCLUSIVE and improve the evidence |

A rejection reason alone is not proof of harmlessness. In particular, correctly rejecting a mismatched packet can still reveal a real upstream delivery defect. Do not remove those flights from the denominator. Do not make queues unbounded, weaken prefix matching, or increase expiry merely to improve counts.

Only confirmed defects proceed to runtime changes. Each fix must have a causal explanation, a failing-before reproduction, and a passing-after test with the same independent expected outcome. For benign results, change only the assessment/reporting necessary to distinguish diagnostic metrics from acceptance gates.

Deliverable: a per-failure disposition table with raw evidence links, simulator consequences, and either a justified no-change decision or a small targeted fix.

## Phase 3: correct and validate the lifecycle observer

The current `reacquisition_producer_chain.py` checks acquisition and rejection using periodically observed latest diagnostics. Actual pipeline traces show acquisitions missed by that observer. Audit all assertions that depend on transient status, including rejection and quarantine, rather than changing only the first failing assertion.

1. Define an acquisition/rejection transition check using authoritative pipeline transition records tied to source timestamps and physical generation. Match records to independently generated A/B scenario windows. A new generation alone, or absence of an error, is insufficient proof of successful acquisition.
2. Retain transition evidence for the test duration. Verify trace completeness and reject truncated or missing evidence. If existing traces lack a necessary state distinction, add narrowly scoped test instrumentation rather than changing control behavior.
3. Keep diagnostic-topic delivery checks separate from proof that a short-lived internal transition occurred. Preserve any actual public-interface requirement to deliver retained revocations.
4. Exercise explicit positive and negative controls:
   - A acquires normally, is rejected, and B acquires a distinct generation.
   - A acquires and is rejected between two observer polls: the lifecycle test passes.
   - A truly never acquires: the lifecycle test fails.
   - Evidence belongs to an earlier round or another flight: the test fails.
   - Rejection or revocation is absent, or an obsolete command appears after rejection: the test fails.
   - B fails to reacquire or no required B command reaches the decoder: the test fails.
5. Replay the two recorded observer failures and run all 14 registered reacquisition consumers with the corrected observer. Validate instrumentation on/off so extra tracing does not silently determine outcomes.

Deliverable: a small observer/test change, negative-control results, preserved before/after evidence, and a clear distinction between an observer correction and any actual runtime defect.

## Phase 4: verify ball-flight-to-racket conversion in simulation

Use the supplied simulations to evaluate the actual production conversion path, not a separate reimplementation of the adapter. Pin model and physics configuration IDs. Keep reference truth independent of the V2 estimator and adapter being checked.

For each declared valid scenario, retain:

- Ground-truth trajectory and selected plane; predicted contact XYZ, incoming velocity, and absolute contact timestamp.
- The converted `StrikeTarget`, including frame, units, plane identity, bounce count, and source cutoff.
- The existing racket planner's position, normal/orientation, velocity, outgoing-ball velocity, and contact time.
- The schema-4/5 serialized command and what the real software receiver decodes, including coordinate offsets and HOME identity.
- Where supported by the supplied simulator, the simulated racket/ball contact and outgoing trajectory. Label target-level tests distinctly from full simulated contact tests.

Cover fixed/per-ball/stable-per-ball planes; lateral placements; declared speeds and supported spin modes; valid and invalid bounce order; net/ground events; and revocation before engagement. Continue to demonstrate that an engaged swing's cancellation is outside this contract.

Provide synchronized plots or replay videos showing the true and predicted ball paths, selected plane, intended contact, and racket target/normal. Pair these with position, velocity, timing, and decoder-conversion error measurements. Numeric tolerances must be declared before scoring; visual plausibility is supplementary.

Reuse the existing 27 adapter/downstream parity combinations and independent outgoing numerical reference tests as baseline checks. Add end-to-end simulation coverage only where those checks do not already establish the required behavior.

Deliverable: an inspectable simulation report with per-case metrics, images/replays, and an explicit statement of which levels of simulation actually ran.

## Phase 5: complete qualification and publish the decision

After focused corrections pass, freeze one clean source commit and run the native C++ suites, relevant sanitizer suites, legacy/V2 parity and outgoing reference checks, actual Runner/ONNX lifecycle comparison, full registered ROS suite, and the declared repeated campaigns. Retain the existing 160/90 matrices and all four original profiles so comparison remains possible. Any justified scenario change must be documented before execution.

Run timed trials separately from builds and archival. Keep all repetitions, original exit codes, failed attempts, source/configuration/binary hashes, and instrumentation settings. New results must distinguish raw legacy gate outcomes from the user's revised acceptance assessment. Disclose inherited lint separately; do not treat it as a new functional defect.

Completion requires:

- Every observed coverage failure is either corrected and reverified, or demonstrated to have no functional impact under the declared simulator/consumer contract. No unexplained missing usable flight remains.
- Lifecycle positive and negative controls pass, and the registered reacquisition scenarios satisfy their intended behavior.
- Ball-to-contact-to-racket-to-decoder conversion meets the declared simulation tolerances, with reviewable visual evidence.
- Invalid or revoked evidence does not authorize a new command in the tested cases; one-shot identity and plane immutability remain intact.
- The accepted timing result is explicitly recorded without claiming the old 30 ms gate passed. New timing excursions are reported with their effects on usability, expiry, and simulated contact; they are not automatically hidden by the acceptance of the prior result.
- Protected downstream behavior and legacy defaults remain covered by regressions.

## Commit and review sequence

Use small commits for: (1) revised acceptance/scenario definitions, (2) coverage diagnostics and outcome accounting, (3) the lifecycle observer and controls, (4) each individually confirmed runtime fix if any, (5) simulation conversion checks/artifacts, and (6) final qualification documentation. Omit runtime-fix commits entirely when investigation demonstrates no defect.

Update the existing draft PR with the final scope, current verdict, and no-change dispositions. Keep bulk evidence on a separate evidence branch. Historical failures remain accessible even after a new version passes or an old metric receives an evidence-backed non-blocking disposition.
