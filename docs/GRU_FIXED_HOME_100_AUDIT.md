# Fixed-home-100 reference audit for the recurrent GRU evaluator

Status: **AUDIT COMPLETE; REFERENCE EVALUATION NOT RUN; FORMAL GATE3 NOT PROVEN.**
The earlier custom-cadence draft is superseded. Do not use its launcher to
produce a reference score. No training, ROS runtime, native Runner or scored
simulation was started or stopped during this audit.

## Revision and inspected evidence

Evaluation worktree: `feature/stationary-gru-mujoco-eval`, HEAD
`33c609022efbb4e85fe6a58a5a8947b0fbd46c76`, based on public HOPE
`756918a8cfa7d496f6ea88018362be187985187e` plus the GRU changes.

Live public upstream `hitchopen/HOPE` main was resolved through GitHub to
**7990f18c2c1ae560c9e0708dca10228b9f2cc610** (#119). The four requested files
were fetched at that immutable revision and their Git blob hashes verified:

| Path | Git blob SHA-1 |
| --- | --- |
| `docs/operations/gate3_two_file_runner.md` | `2a4234b6217c385379f540ab969297d886836fe7` |
| `docs/operations/deploy_and_gate3.md` | `ae25068269a60ddf1ee5f21cd0c5bde915565b99` |
| `a3_deploy/a3_deploy_example/config/gate3/scenarios/fixed-home-100.yaml` | `a5a26591af6f8b379f80eb7b87707053d146523d` |
| `a3_deploy/a3_deploy_example/scripts/tests/test_run_gate3.py` | `aee53bf07ab678c1a115fe2c288f0752db1d3066` |

All four are byte-identical to this evaluation checkout. The official resolver,
fixed-HOME wrapper, common scenario definitions, conductor, backend script and
recovery-dwell helper were also fetched and match this checkout. These reference
files were not modified. The protocol is therefore pinned independently of the
GRU feature commits.

The original professor fork remote in `~/workspace/nohope` is
`https://github.com/dongc1/nohope.git`, fetch-only. Its **cached** upstream/main is
`c7986ee5b8e0da8276ca43e22b27664fcd256d55` (2026-11-02). A live Git request failed
DNS, and the connected GitHub API returned 404 for that repository. Its current
live HEAD is **not verified**. Three requested files match that cached revision;
`deploy_and_gate3.md` has a different cached blob. Do not describe the cached
professor fork as freshly fetched. The verified live public reference above
matches all four files actually inspected here.

External evidence is in
`~/workspace/hope-stationary-gru-evaluations/20261105-gate3-reference-audit-rkea78z8/`:
upstream source copies, revision/blob receipts, resolved environment, all 100
serve rows in CSV/JSON, inherited threshold composition and test output.

## Exact serve sequence

Coordinates are planner **table-surface coordinates**, SI units. Every row has
`x=2.4`, `z=0.49`, `vx=-3.0`, `vy=0.0`, `vz=2.2`. Default initial spin is zero.
Only the lateral coordinate differs:

| Label | Complete `[x,y,z,vx,vy,vz]` | Count in 100 |
| --- | --- | ---: |
| A | `[2.4,-1.0225,0.49,-3.0,0.0,2.2]` | 20 |
| B | `[2.4,-0.7125,0.49,-3.0,0.0,2.2]` | 20 |
| C | `[2.4,-1.0575,0.49,-3.0,0.0,2.2]` | 15 |
| D | `[2.4,-0.6575,0.49,-3.0,0.0,2.2]` | 15 |
| E | `[2.4,-1.0925,0.49,-3.0,0.0,2.2]` | 15 |
| F | `[2.4,-0.6125,0.49,-3.0,0.0,2.2]` | 15 |

The exact composition, without randomization, is:

1. Serves **1–6**: `A B C D E F`.
2. Serves **7–86**: four repetitions of the 20-row cycle
   `A C B D E F A B C D E F A B C D E F A B`.
3. Serves **87–100**: the first 14 rows of that cycle:
   `A C B D E F A B C D E F A B`.

The reference rows contain **no commanded FH/BH labels**. With nominal immutable
HOME and its intended split, A/C/E and B/D/F give 50 nominal lanes per side.
Actual sides are selected by the live Planner and must be reported from its
commands, not assigned from these labels. No seeded shuffle is part of this
reference. A seed may identify additional stochastic settings, but must not
permute or resample its 100 serve states.

## Timing and reset/lifecycle rules

- After initial stand and one transition into MOTION, preserve the **20 s
  policy-native idle**. Minimum accepted observed idle is **15 s**.
- Flights/slots: **[2.5 s ×6, 1.15 s ×93, 2.5 s ×1]**.
- Pauses: **[1.0 s ×6, 0.0 s ×94]**. Only the final serve gets the final 2.5 s
  rapid-flight tail; there is no extra tail between repeated 20-row cycles.
- Ideal launch times relative to the first launch: clean serves at
  0, 3.5, 7, 10.5, 14, 17.5 s; serve 7 at 21 s; rapid serve k at
  `21 + (k-7)*1.15` s; serve 100 at **127.95 s**. Scheduled slots/pauses finish
  at **130.45 s** after the first launch. This is not the total wall duration:
  launch/release acknowledgements, slot availability, final native completion,
  >=3 s post-completion recovery tail and full flight draining also apply.
- The actual launcher waits from a successful launch acknowledgement, then
  releases the incoming slot and waits for its release acknowledgement. The
  launch-cadence receipt compares actual versus scheduled launch gaps; default
  tolerance is **0.10 s**.
- **A slot ending does not kill the ball.** The native plant transfers the prior
  ball's position/velocity/spin/counters to independent return slots. Every old
  ball continues its measured flight to its natural ground endpoint, across
  subsequent incoming launches. Only the current incoming ball goes to Planner.
  If slots are full, wait for a free slot; record actual cadence rather than
  truncating a flight. Missing endpoints/incomplete trajectories fail.
- One continuous MOTION session, **no inter-ball reset, rescue, forced recovery,
  abort/re-arm, side selection or reference-target replay**. Initial stand/reset
  setup may occur before MOTION. A physical fall ends/fails the reference rally;
  continuing after a reset would be a separately labelled diagnostic adaptation.
- Native lifecycle expects all **100 engage/complete** events. The rapid 94
  completion identities are **93 preempt + one final native completion**;
  preemption is not a plant reset or a hidden-state reset.
- Drain waits while real telemetry progresses; it has no total-time cap merely
  for a long live flight. Producer-death grace is **2 s**, stale-progress failure
  is **10 s**; managed cleanup grants another 10 s after drain failure. An
  evaluation-only outer timeout must mark interruption/failure, never success.

## Applicable pass/fail requirements

The scene YAML alone is not the full gate. Resolving the fixed-home wrapper
at 100 balls yields these requirements; values below are not relaxed for GRU:

| Area | Reference requirement |
| --- | --- |
| Lifecycle | 100 engaged, 100 completed; 94 rapid engaged/completed; proxy rate 1.0; unique identities/no multiple engagement; both clean sides >=3 |
| Physical count floors | >=84 global qualified contacts, >=84 global legal landings, >=84 global eligible landing observations |
| Per-side floors | >=49 assigned physical samples, >=41 qualified contacts, >=41 legal landings, >=41 eligible landing observations |
| Per-side rates | qualified contacts / assigned serves >=5/6; legal landings / eligible observed landings >=5/6. These are different denominators. If exactly 50 assigned serves, contact rate requires >=42 contacts; do not replace all floors/rates with one count. |
| Recovery | HOME XY error <=0.05 m; planar speed <=0.12 m/s (EMA alpha 0.20); pelvis tilt <=12 degrees; HOME yaw error <=5 degrees; pelvis z>=0.80 m; continuous dwell >=0.30 s |
| Recovery semantics | Dwell is reversible: any later violation revokes qualification; a new uninterrupted dwell is required. The next engagement is not proof of recovery. Final rapid serve must recover. |
| Rapid timing | max engagement gap <=1.40 s; contact-to-next-commit in [0.40,0.55] s for each nonfinal rapid serve; all FF/FB/BB/BF transitions exercised |
| Fixed HOME/stability | command station invariant <=0.0015 m; x-lock <=0.05 m plus fixed 1 micrometre arithmetic tolerance; XY peak/end drift <=0.50 m; yaw peak <=12 degrees/end <=5 degrees; zero falls, actual-q faults, command-safety faults and rescues |
| Support-frame ends | support midpoint HOME error <=0.03 m; foot anchor HOME error <=0.03 m; signed stance width in [0.26,0.31] m |
| Support-frame additional defaults | base/support yaw peak/end 12/5 degrees; midpoint peak <=0.10 m; foot anchor peak <=0.12 m; foot heading peak/end 15/3 degrees; foot tilt peak/end 12/5 degrees; level-2 width allowance <=0.08 m; fore/aft stagger <=0.10 m |
| Torso HOLD | required; pitch median/p95/peak <=3/5/8 degrees, minimum HOLD window 0.20 s, settle trim 0.12 s; extra roll/yaw checks only if the corresponding three-axis contract is enabled |
| Idle/plant | observed idle >=15 s; qdes step peak <=0.08 rad, step RMS <=0.005 rad; reversals <=8 Hz; joint velocity RMS <=0.50 rad/s; normalized control step RMS/peak <=0.03/0.20; control saturation fraction 0; tracking-error RMS <=0.15 rad is enforced by the actual plant report despite the wrapper comment calling it diagnostic. |
| Physical evidence | >=20 samples per flight, <=0.050 s telemetry gap, valid active/inactive/natural terminal evidence, monotonic contact counters with no jumps; legal table-contact edge observation lag <=0.010 s; no rapid-lane censor exemption |
| Additional required reports | actual-q full-tail ledger, localization freshness, C++ Planner one-shot/flight joins, launch-cadence receipt, plant trace, idle smoothness, HOME support-frame, torso HOLD, collision ledger; optional-reach checks when the runtime enables them; ABI-aware obs/phase report |

Raw mesh contact is not the native `contact_pass`: it additionally requires one
legal incoming own-half bounce, no incoming net contact, exactly one racket
contact, positive outgoing vx, and complete physical telemetry. Native legal
landing requires the first post-racket real table bounce on the opponent half,
no post-racket net events, and complete flight/terminal evidence. Ball-center
bounds are inset by radius 0.02 m: table-frame x in [1.39,2.72] and y in
[-1.505,-0.02], positive post-bounce vz and vx, with a measured-step-dependent
height tolerance. The host evaluator's broader center bounds and first-plane
approximation do not implement this exact rule.

The wrapper/report tracking-error discrepancy is substantive: the shell comment
calls 0.15 rad diagnostic, but `pp_mujoco_plant_report.py` appends a failure above
that value and returns `pass=false`. This audit follows executable behavior and
does not weaken the gate to match the comment.

There are additional ABI-mode-dependent observation/phase checks. Historical
ball-clock reports include idle wrist limits (0.35 rad outside V15's 0.45 rad
exception) and mode-specific HOLD/plane assumptions. A missing GRU report mode
must be **NOT_PROVEN**, not silently routed to an old feed-forward report.
Final success needs a successful full process/archival exit and
`run.json.certification_pass=true`, with lifecycle, Planner, physical-ball,
plant/collision/idle and drain reports passing. `--check`, contact count or zero
falls alone cannot certify Gate3.

## Compatibility audit and adaptations

| Component | Current GRU evaluator | Required disposition |
| --- | --- | --- |
| Exact serve rows and sequence | Can read the official resolver and translate frames | Preserve all 100 rows/order and official timing; resolved ledger is already saved |
| GRU recurrent inference | Host supports `obs[1,110],h_in[1,1,110] -> actions[1,31],h_out[1,1,110]` | Preserve normalization, joint/action/control semantics and hidden state across every serve/preemption |
| Native two-file Runner ABI | Not compatible as-is | `PpOnnxPolicy::Run` feeds `obs,time_step`, not `h_in`, and never carries `h_out`; generic native ORT also requires one `obs_dict` input and one `action` output. Existing bundle is actor.onnx+manifest, not the native deploy.yaml export/inspection ABI. Same 110 observation width does not make these protocols equivalent. |
| Continuous ball handoff | Host manager controls one ball and parks it; vendor return bodies are disabled | Implement/instrument multiple physical return slots and continuous full-flight drain, or use the native instrumented plant with a genuine recurrent runner adapter. A 1.15 s forced park or waiting for every prior ball to finish is not reference cadence. |
| Scheduling | Draft used long per-serve windows, two-second warmup and shuffled lanes | Superseded. Preserve 20 s native idle, six clean slots, 94 rapid slots and final completion/recovery tail. No stock 12 s task timeout. |
| Reset policy | Draft resets and continues after a fall/numerical event | Formal reference fails/ends without rescue/reset; any continue-after-failure mode must be separately labelled and cannot PASS reference certification. |
| Planner | Draft selects direct-pose schema-2 mode in an isolated domain | Actual C++ solve can be used, but native shared-HOME/flight-packet/lifecycle mapping is different. Save chosen mode and do not claim production transport parity. |
| Frames/HOME | Host table starts x=.5, centered y=0; native world/table station differs | A documented constant translation is supported. Preserve the same relative robot/table geometry and only one z=.76 offset; HOME/side/plane must come from the actual runtime contract, not a 110-D shape guess. |
| Physics | Host nominal 3.4 g PPQ004 ball, MuJoCo soft contacts and custom aero stepping | Preserve and record established GRU plant for adapted diagnostics; do not call it native-plant parity. Native profile/solver/contact/drain settings and all active balls must be audited explicitly. |
| Scoring/READY | Host raw force events, first-landing rule and position/speed-only proxy | Add exact reference evidence and reversible yaw/tilt/speed/dwell/support/lifecycle checks. Keep primary all-100 statistics separately; do not relabel the partial proxy as Gate3 READY or native completion. |
| Reporting | Host has no native engage/preempt/native-completion or complete plant ledger | These reports are unavailable, not implicitly passing. Formal equivalence requires a new supported recurrent ABI and matched evidence pipeline. |

**Decision:** do not run the old custom test and do not pass the GRU bundle to
`run_gate3.sh` under a fabricated feed-forward ABI. A future run retaining the
host GRU runner, host physics or adapted scoring must be titled **fixed-home-100
reference-sequence GRU diagnostic**, with every adaptation and NOT_PROVEN gate
listed. Formal Gate3 equivalence is not claimed. The single-ball handoff and
native recurrent ABI/reporting gaps are substantive blockers, in addition to
the existing sandbox container-access restriction.

## Verification performed

- Live immutable upstream revision and source blob checks, including all four
  requested files and six resolver/scoring helpers: match the evaluation checkout.
- Official `scene_environment` used to resolve all 100 rows and timing lists.
- Composition-only wrapper inspection used the upstream tests' mock backend
  technique; it started no actual Runner, Planner, ROS or MuJoCo evaluation.
- **32 existing reference tests passed** in `test_run_gate3.py` and
  `test_gate3_continuous_flights.py`. These are configuration/lifecycle unit tests,
  not recurrent native-policy compatibility or a live 100-serve result.
- Training checkout was read-only; no dependency/hardware/runtime change.

## Authorized host diagnostic implementation update

The subsequent user-authorized reference-sequence diagnostic implements the concurrent-ball gap in the separate host evaluator. Focused tests pass; native recurrent ABI and native reporting remain unsupported. See [implementation, frozen identity and execution status](GRU_FIXED_HOME_DIAGNOSTIC.md). The earlier decision above continues to prohibit the retired custom protocol and fabricated native ABI, not this explicitly labelled adapted diagnostic. Actual export/preflight/100-serve execution remains blocked by sandbox Podman access; no Gate3 result is claimed.
