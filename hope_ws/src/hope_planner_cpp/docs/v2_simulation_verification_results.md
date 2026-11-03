# V2 simulation verification — current assessment

The observer correction and simulation verification are implemented. This is not yet a claim of complete incoming-flight coverage. Historical lost incoming commands remain distinct from harmless skipped updates and the accepted timing deviation.

Qualified source: `9603464` on `feature/v2-incoming-validity-continuous-dropin`; original iteration baseline: `0fb3542`. The new raw run is `qualification-01`. Its raw verdicts are preserved, including failures. The earlier evidence branch is unchanged.

## What this iteration changed

- Added an explicit acceptance manifest that records the user's acceptance of the historical 54.731069 ms forecast publication, keeps expiry/identity/invalidation checks, and evaluates physical-flight usefulness separately from per-observation coverage.
- Replaced latest-status polling as the lifecycle oracle with a complete, source- and generation-matched transition trace. Instrumentation is default-off; its bounded buffer is flushed only at shutdown. Negative controls must reject missing acquisition, rejection, quarantine, B acquisition/decode, obsolete commands, and incomplete evidence.
- Added a reproducible per-flight audit and controls that detect an absent flight, incorrect decoded target, insufficient lead, and incorrect contact time. New packet receipts retain every sample timestamp for future prefix diagnosis.
- Added independent ball-truth simulation through the actual V2 coordinator/adapter, existing racket planner and actual schema-4/5 decoder, plus a replay in the supplied A3 MuJoCo scene. Production racket-planning behavior is unchanged.

## Verification on the frozen source

| Check | Observed result |
|---|---|
| Native C++ | 25/25 groups pass |
| ASan + UBSan | 25/25 groups pass |
| Independent outgoing numerical reference | pass |
| Legacy and premerge V2 parity | pass; 701 exact legacy cutoffs, pinned V2 differences only |
| Fresh focused reacquisition consumers | 14/14 pass |
| Observer, outcome-audit and scope unit controls | 23/23 pass |
| Native conversion simulation | 162/162 checks pass; 81/81 physical scenarios have a usable command; 159 commands and 3 justified early abstentions |
| Supplied A3 MuJoCo scene | 159/159 outgoing-ball replays pass |
| Full ROS | planner 131/132, messages 2/2, bringup 25/27 |
| Instrumentation on/off comparison | 4/4 comparisons pass; enabled raw tests pass; disabled raw tests correctly fail missing-trace proof |
| Exact historical lifecycle replay | 2/2 pass; every recorded generation/reason matches |
| Existing repeated campaign | 155/160 raw passes; 1,440/1,440 flights receive usable decoded targets |
| Incoming repeated campaign | 88/90 raw passes; 899/900 flights receive usable decoded targets |
| Actual Runner/ONNX lifecycle comparison | 24/24 expected comparisons pass, including baseline/disabled negative controls |
| Actual recorded-command Runner replay | 4/4 pass: two affected flights with recorded revocation and valid-only controls |

The sole planner-suite raw failure is `integrated_chain_150_ON_REFRESH`: global source cadence was 139.656 Hz versus requested 150 Hz. The largest source gap was 331.480 ms immediately after the first startup sample, before the first declared ball flight at 0.8 s. Both scored flights had independently checked usable decoded targets (25 and 36 commands). This does not establish the requested source rate; it establishes that this particular startup failure did not lose either scored flight. Bringup flake8/pep257 failures are inherited, with no added lint signatures.

## Simulation scope and measurements

The frozen grid covers speeds 5/10/12 m/s, unknown/zero/known spin, three lateral positions, fixed/per-ball/stable-per-ball planes and two declared source cutoffs. The native probe supplies HOME and selected plane to production components; separate actual ROS tests exercise runtime selection, packet transport and decoding.

Maximum committed-contact error was 0.4933 mm and 0.05491 ms. Adapter position/velocity/time errors were zero; decoder position error was at most floating-point roundoff (1.12e-16 m). Independently reconstructed legacy-impact outgoing landing error was 0.6239 mm; the supplied MuJoCo scene's maximum landing error was 3.6477 mm.

The existing managed racket planner uses speed-dependent restitution, unlike the separate validated-return backend. The checker uses that declared law and independently integrates the reconstructed outgoing velocity. The original stricter validated-return reference check remains in qualification. Three early known-spin prefixes correctly abstained because their event-order uncertainty overlapped; each physical scenario also produced a valid command at its declared later cutoff. This is not used to excuse a missing committed ROS flight.

MuJoCo ran the supplied A3 scene with the robot pinned, starting the ball at the converted contact with independently reconstructed outgoing velocity. It tested ball flight, net clearance and landing. It did not execute a policy-driven robot swing or a racket collision. The original serve scene's drag fit was explicitly overlaid with the pinned incoming-planner fit before scoring. Unknown-spin reference cases use a zero-spin plant with withheld orientation; these checks do not establish accuracy for arbitrary unobserved spin or physical measurement noise.

## Historical coverage dispositions

All eight retained existing-backend campaign failures and the relevant full-ROS failures were audited using independent offered flights, actual decoder receipts, >=80 ms receiver lead, <=20 mm contact-position error and <=10 ms contact-time error. Each offered flight in these cases had a usable command. The historical review/reacquisition targets remain valid in the actual decoder through contact. In the base-yaw case, the first flight deliberately receives wrong-frame input at elapsed 1.0–1.08 s and its final invalidation arrives at 1.00242734 s; the second flight retains its target through contact. The observer, simulation and retention evidence support no runtime change for these skipped updates. The original per-observation verdicts remain intact.

The four original profiles remain explicit: `base_yaw_chain_150_0`, `review_chain_300_12_unknown`, `review_chain_150_12_zero`, and `review_refresh_300_10_unknown`. In the fresh full ROS run they passed raw coverage at 39/41, 40/40, 20/20 and 149/149 epochs respectively. The historical 36/39 result is preserved and separately assessed at ten useful flights out of ten.

The seven historical incoming campaign failures consist of six missed-command cases and the accepted forecast-latency deviation. The full ROS baseline has one additional missed-command case. Each missed-command case remains nine usable flights out of ten, with no removal from the denominator:

- Two missing packets are reproduced by replaying the actual offered source gaps (36.783 ms and 48.084 ms). Independent gap-free controls produce ten packets.
- The full-ROS missing packet is reproduced using the actual relay-delivered history; offered-source and gap-free controls produce ten packets. This establishes an effect of the retained delivery loss without guessing its OS/transport cause.
- Three prefix rejections include earlier observations received after the 30 ms qualification deadline. Packet endpoint/count evidence and typed receipt histories are retained. This does not authorize relaxing history qualification or packet matching.
- One immutable packet was rejected 46.229 ms after its source cutoff, including 40.846 ms after freeze. Its expiry rejection is correct, but the flight still lost its only useful command.

A recovery extension remains a scope question: making a fresh command attempt before acceptance must preserve one accepted command per flight, exact causal cutoffs for each attempt, HOME/plane identity, invalidation, and expiry. No such recovery has been silently enabled in this verification change.

## Preserved development failures

The first lifecycle-instrumented run failed because the initial oracle expected rejection to retain the acquisition generation; actual invalidation retires that generation. A second oracle correction accounts for deliberately altered fault timestamps. Original failed receipts remain, followed by corrected replay and a fresh 14/14 run.

The first conversion run used the wrong (constant) restitution law for the managed legacy racket planner. Its failed receipts remain; the corrected law and early-prefix uncertainty criteria were declared before the successful new run. Missing MuJoCo Python dependencies and diagnostic replay-parser mistakes are retained as development attempts, not presented as qualification failures or hidden retries.

## Fresh campaign findings and consumer lifetime

The four new per-observation coverage failures are repetition 1 `review_chain_300_12_known` (33/35), repetition 3 `review_chain_300_12_unknown` (36/40), repetition 3 `review_refresh_300_12_known` (32/34), and repetition 4 `review_chain_300_12_known` (32/34). Every scored flight received a usable decoded command and retained a valid target through contact. These skipped updates have no demonstrated functional effect in the declared scenarios, so no production correction was made for them.

The fifth existing-backend campaign failure is repetition 2 `integrated_chain_300_ON_REFRESH`: requested 300 Hz, measured 279.064 Hz, with a 328.278 ms gap from source elapsed 0.000054 to 0.328332 s. This occurs before the first flight at 0.8 s. Both scored flights received useful commands; the deliberate frame-fault withdrawal remains expected. The failed global-rate verdict remains recorded.

Incoming repetition 4 `incoming_chain_360_stable_per_ball_unknown` failed only the old publication-age gate at **41.784759 ms**. The delayed publication was a forecast for flight 8, reason `ONE_SHOT_ALREADY_COMMITTED`; its candidate was command-ineligible, followed by explicit `SOURCE_EXPIRED` evidence. All ten commands decoded. A separate replay of the actual flight-8 command and receiver intervals into production `PpPolicy`/ONNX engaged before its revocation and remained active through the recorded contact deadline. The valid-only control behaved likewise. This establishes no command-authority effect for that timing excursion; it does not claim the raw 30 ms gate passed or extend runtime expiry.

Incoming repetition 5 `incoming_chain_360_fixed_unknown` has a **confirmed missing flight-9 command**. All ten packets formed. Packet 9 contains 44 exact sample timestamps, including three observations the relay typed-published after 0.612, 0.585 and 0.661 ms respectively, but which never appear in the planner's receipt trace. The packet was rejected for `PACKET_OBSERVATION_PREFIX_MISMATCH` 15.648 ms after its cutoff. This identifies the lost delivery boundary without attributing an unproven DDS/OS cause. Its raw sample lists and complete stage traces are retained.

Consumer lifetime is reported separately from first useful receipt. The lifetime audit joins all wire identities belonging to the same independent physical flight and checks the actual decoder timeline; it does not infer policy engagement. It also found an early `SOURCE_EXPIRED` revocation in repetition 2 `incoming_chain_300_per_ball_zero`, flight 3, 86.545 ms before contact. Replaying the actual command and receipt intervals into production `PpPolicy`/ONNX showed engagement about 11 ms after command receipt, before the roughly 80 ms revocation interval. The recorded-revocation and valid-only runs both stayed active through contact, as required by the existing pre-engagement-only contract.

The four recorded-policy runs rebase absolute clocks and the local HOME epoch while preserving target geometry, velocities, flight/revision identity and receiver intervals. They use nominal 50 Hz steps with stand-state feedback, not dynamic robot feedback. They prove the tested software engagement boundary, not physical reach or racket contact.

The complete raw verifier returns **exit 1**. Its failed gates are the ROS-result checks, existing campaign and incoming campaign. Source commit/worktree checks pass. All 250 declared repetitions ran once with their original exits retained. No passing retry replaces a failed qualification trial.

## What remains

The observer correction, conversion checks, supplied ball simulation, timing assessment and failure accounting are implemented and verified. **The full plan's delivery-completion criterion is not satisfied.** Seven historical lost-flight cases and one fresh lost-flight case remain confirmed; they must not be relabeled as harmless.

Closing these cases requires a tested remedy for packet formation/delivery and, where an immutable packet has already expired, a fresh attempt before command publication. The recovery contract must preserve bounded memory, exact causal prefixes, HOME/plane identity, runtime freshness, invalidation, and at most one published command per physical flight. A published command must latch that boundary even when Runner engagement is unknown. No new racket planner or mid-swing update is required, and neither has been introduced.

The pending scope choice is whether to implement that V2 pre-command recovery extension in this iteration. No recovery or expiry relaxation has been enabled by this verification work. Hardware deployment is not a completion requirement here.

## Review and reproduce

- [Code and draft PR](https://github.com/timothyypark/HOPE/pull/1)
- [Raw evidence, per-failure dispositions, manifests and plots](https://github.com/timothyypark/HOPE/tree/evidence/v2-simulation-verification-20261030)
- [Preserved original evidence](https://github.com/timothyypark/HOPE/tree/evidence/v2-incoming-qualification-20261029)

The evidence branch README maps archive paths to the original evidence roots and includes SHA-256 inventories. `reports/failure-dispositions.json` lists all 29 assessed historical/current failure or original-profile receipts, with eight unresolved lost-flight cases. The continuous truth plots show actual V2 waypoints; separate conversion plots show contact and racket normal. Build commands, test exits, exact replay inputs, failed development attempts and configuration declarations are retained.

The reusable qualification entry point is `hope_ws/src/hope_planner_cpp/scripts/flight_prediction_validation/verify_integrated.sh`. The supplied-scene gate requires MuJoCo and the declared Python dependencies; this run used an isolated dependency directory through `HOPE_SIMULATION_PYTHONPATH`. The evidence wrapper declares its sole storage hook: hash-verified compression of sanitizer binaries after their tests and before ROS builds/timing runs. Qualification used clean source `96034645f26b1597c093281a398fa0e62d5c93b5`; subsequent report-only changes do not replace that source receipt.

`nohope` remains unchanged at `96e71c1adb217d14b925681173cebe00d8427f21`. Its upstream was unavailable during inspection (API 404/Git 403); no upstream contents are inferred.
