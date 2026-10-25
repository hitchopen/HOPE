# CompleteHold and StationaryRally training review

Source: `dongc1/nohope`, branch `main_improve_2`, commit
`9c4d6366ab7bd58b73baf98c06159e1ac51688d2`, including the newly published
revision-21 generalization bank. Training strategy changes were reviewed from
`54952d0f11252cbef7b156c1d46d49389b75dc95` before that asset publication.
Integration base: `hitchopen/HOPE:nightly_built` at
`91aecabfd44cb258ac998158e724be9017d883a7`.

## Scope and compatibility

This is a selective Isaac Lab RL training port with a shared full-PPQ004
first-bounce model in Python and the C++ simulation/runtime planner. HOPE already contained the
flattened model_14600 recipe from the fork's `2b14b76d1`; the source integration
applies the subsequent training changes without restoring retired Gym tasks.
The original CompleteHold and StationaryRally task/algorithm pairs preserve
the source strategy at `54952d0f1`; the later fork publication changes the
StationaryRally bank/physics binding, which is integrated through the separate
full-model PPQ004 task here. New training defaults to
`HOPEPingPongStationaryPPQ004` / `ppo_balanced_hold`, using the revision-23
sweet-spot refit and enabled placement reward described below. The revision-22
general-return refit remains available through `HOPEPingPongStationaryPPQ004General`.
`HOPEPingPong` / `ppo` remain available explicitly and
remain the playback defaults for existing checkpoints.

The port includes the active recipe inheritance, actual-pose and balance
rewards, fixed-station sampling, interruption-aware recovery, PPO diagnostics,
training metadata, required train/dev/test banks, offline bank preparation and
regression tests. Historical stationary v1-v3 banks and inactive four-clip /
joint-clock recipe assets and scripts are omitted. Shared source helpers remain
where the imported command implementation references them.

The actor/action dimensions remain 110/31 at 50 Hz. StationaryRally explicitly
changes the reset pose and affine action zero to `B_32_5_K20`; equal dimensions
do not make its action offsets interchangeable with older exports. Use the
resolved training/export metadata for any later deployment work. The full-physics follow-up updates C++ return planning, its YAML parameter loading,
and the PPQ004 launch/profile defaults alongside the training paths. Native
Runner control code, published checkpoints and ONNX assets remain unchanged.
No hardware deployment or hardware evaluation was performed.

## What improves

- CompleteHold combines measured full-body posture, station accuracy, support,
  velocity and continuous ready dwell. A bad limb cannot be hidden by averaging
  a large pose reward across the rest of the body.
- Balanced PPO removes the direct temporal mean-policy smoothing objective.
  Physical smoothness still contributes to the return, while feedback remains
  free to issue different commands in adjacent states to correct balance.
- Recovery can be interrupted by the next ball; standing objectives release
  during the strike window.
- StationaryRally holds an immutable session station instead of sampling lateral
  walking targets. Wide contacts are answered by arm/torso reach, with stronger
  capture-point, lean, post-swing momentum and loaded-foot friction penalties.
- The stationary recipe excludes the racket arm from swing imitation, avoiding
  competition between the reference wrist position and the commanded contact.
- Exact-contact position supervision and a bounded reward tail provide a useful
  signal for large misses. Fixed-station sampling preserves seeded outputs and
  subsequent random-number state while reducing redundant distance checks.

These are confirmed implementation changes. They support the standing objective;
unit tests do not establish learned-policy convergence.

## Evidence and limits

The user reports firm standing in a MuJoCo simulation test, with no real-world
evaluation. No matching run log, checkpoint identity or measured evaluation
report was supplied with that statement. This review therefore records the
MuJoCo result as user-reported and does not claim independent reproduction,
Isaac training convergence, hardware stability or measured success improvement.

The initial source snapshot included a PPQ004 profile and offline refitter,
but still pinned the recipe-19 venue bank. The later published commit
`9c4d6366a` changes its StationaryRally task to the PPQ004 revision-21 bank and
pins the physics profile before Isaac starts. This integration's pre-launch
validation additionally requires a generated full-model manifest,
verifies the bank/receipt, model/loader code and six referenced YAML hashes, binds
the matching table placement and physics environment, and records the resolved
bank and PPQ004 distribution. Missing preparation fails before Isaac starts.
HOLD remains 1.7; defaults are 6144 environments, 30000 iterations, seed 0 and no
resume or actor warm-start.

The developer's screenshot bank is now available: its training SHA-256 is
`02619e747333980a6f1d48aa3a9c7896e15727cf95cd3f51eb387d7687c4b419`.
The immutable revision-21 source includes 502,053 train, 61,815 dev and 62,103
test rows, with disjoint parent identities and zero stations. Training includes
183,274 retained questions and 318,779 new synthetic questions. All NPZ hashes
match their published receipts. The full trajectory diagnostic sidecars were
not published; the payload supplies at-strike states and source provenance.

Its receipt uses a 5 ms integration step and an outgoing table-surface plane,
so those labels cannot be used directly under the corrected first-bounce model.
The general-return baseline therefore refits this published revision-21 source
into its own revision-22 bank. Its qualification is `full_ppq004_first_bounce_v1` and training
distribution is `stationary_reach_ppq004_full_v1`. Old recipe20, unrefitted21 and
the superseded simplified-return experiment are rejected by model identity
checks. The published source assets remain unchanged in the PR.

The selected model includes fitted exponential racket restitution, capped
frictional/tangential impulse and spin transfer, followed by RK4 quadratic drag,
gravity and Magnus flight. Free-flight spin stays constant under the selected
fit. Its first bounce is the first descending ball-center crossing of the table
surface plus radius. The inverse solves that actual first-bounce position and
flight time, then the stored float32 command is evaluated through full contact
and flight again. C++ and Python share the parameter YAML and have native
numerical parity tests. Refitting/runtime/fine evaluation use the selected
0.5 ms step; the vectorized reward retains its 10 ms, 200-step integration,
with less than 1 mm landing/net discrepancy in nine tested cases against the
fine model. No GPU throughput result is claimed. Incoming reverse qualification uses the same flight law
and contact-center plane; table coefficients remain available for runtime
incoming bounce prediction.

The general-return refit retains at-strike incoming velocity/spin, contact position, intended
landing, parent identity, split membership and `generation_kind`. The original
source-row index is retained as `input_source_row_index`; `source_row_index`
maps each output row into the newly published input bank. It does not infer an observed
pre-bounce state. The PPQ004 data did not identify a new paddle fit; the shared
paddle YAML retains the venue coefficients. Existing reach/velocity boxes,
speed/spin/approach limits, incoming-bounce areas and net/landing margins remain
mandatory. A row is rejected when it fails them; commands are not clipped to
pretend they were qualified. Bank geometry checks do not evaluate robot balance.

## Reproduce the selected recipes

Follow the [training README](../README.md) for the default PPQ004 preparation,
receipt check and scratch-training command. The default offline refit generates
revision-23 train/dev/test banks under `outputs/stationary_ppq004_sweet_spot`; the
training task reads the resulting `training_overrides.yaml`. Its matching
profile is `configs/ball_physics_ppq004_training.yaml`.

For comparison with the original venue-physics recipes, inside the supported
Isaac environment and from `hope_training/whole_body_tracking`:

```bash
source setup_train_env.sh
unset HOPE_BALL_PHYSICS_YAML HOPE_BALL_PHYSICS_CONFIG
hope_isaac_py scripts/train.py task=HOPEPingPongCompleteHold algo=ppo_balanced_hold \
  headless=true max_iterations=30000 seed=1
hope_isaac_py scripts/train.py task=HOPEPingPongStationaryRally algo=ppo_balanced_hold \
  headless=true max_iterations=30000 seed=1
```

The explicit PPQ004 launcher remains available for a generated bundle at a
custom location:

```bash
hope_isaac_py scripts/train_refitted_stationary.py \
  --overrides /absolute/path/to/training_overrides.yaml --dry-run \
  max_iterations=30000 seed=0 \
  checkpoint_path=null warm_start_actor_checkpoint=null
```

Remove `--dry-run` to start. Both entrypoints use the same manifest validation.
They bind the physics profile and all referenced YAML, model/loader code, bank
and receipt hashes, stationary task, balanced PPO, and the full-physics
training-distribution identity. Old-physics
optimizer resumes are rejected. Explicit actor warm-start remains available
through `warm_start_actor_checkpoint=/absolute/path/model.pt`; it is not the
default scratch-training recipe. A geometry-qualified bank is not a qualified
policy.

For learned-policy confirmation, retain the checkpoint hash, resolved task and
physics/bank hashes, seed, evaluation duration, fall count, ready-pose/dwell
metrics and per-side contact/return rates. A later hardware evaluation is separate
from this simulation-training contribution.

## Comparison with PR 87

[PR 87](https://github.com/hitchopen/HOPE/pull/87) at
`3a19636a0dadc79cd985a45d0890007c8d0d2222` packages the stationary strategy for
`main`; this PR targets `nightly_built`. Its patch was compared against this
integration without merging unrelated base-branch changes.

All nine published revision-21 asset files are byte-identical. Its flattened
balanced PPO configuration resolves identically to this PR's inherited version.
The resolved StationaryRally strategy also matches after excluding only its
physics-profile binding and five bank path/revision/hash fields. There are no
additional reward, curriculum or PPO dynamics to import.

The selective update adopts PR 87's explicit `-filter` attribute for the published
NPZ files, ensuring they remain ordinary Git blobs even when global attributes
configure LFS. It adapts PR 87's pinned recipe-parity check to protect the shared
training strategy while allowing this PR's separate full-model bank binding.

The integration retains the full PPQ004 contact/spin/Magnus first-bounce model,
full-model qualification, YAML dependency and model hashes, checkpoint guards,
sampler frame/convergence/net checks, and PPO diagnostics fix. PR 87's direct
revision-21 default and single-profile hash check do not replace these checks.
Explicit legacy task selections remain available.

## Full-model baseline validation

The focused refit/default/launcher regression checks pass, including rejection of
old and simplified-model banks, dependency-hash changes, missing manifests before
Isaac import, and incompatible optimizer resumes. Native C++ parity tests cover
full contact/spin, drag/Magnus integration, the radius-height first bounce,
nonconvergence, fallback timing and net/post handling. Independent analytic drag
and Magnus cases are included so agreement between two implementations is not
the only numerical check.

The complete CPU suite passed **479 tests**, with 11 skipped for unavailable
Isaac/CUDA or a second GPU. The native C++ suite passed 84 tests. Real MuJoCo
scene smoke checks verified YAML shell inertia, drag/Magnus force and three
physical control ticks; these checks did not run a policy.

The revision-22 full-model refit of the published revision-21 source retained:

| Split | Source rows | Full-model qualified rows |
| --- | ---: | ---: |
| Train | 502,053 | 384,767 |
| Dev | 61,815 | 47,174 |
| Test | 62,103 | 47,443 |

Training retains 151,591 forehand and 233,176 backhand questions, exceeding the
50,000-per-side requirement. The unchanged reach/velocity boxes account for
117,215 overlapping train-row failures; no boxes or other bounds were relaxed.
The retained train first-bounce position error is 0.04819 mm at the 95th
percentile and 0.10003 mm maximum after float32 command storage. All source
input fields and synthetic provenance were verified against their mapped source
rows, all stations remain zero, and split parents remain disjoint. The generated
manifest passed the standalone dry-run, default prelaunch, task contract and
production training-bank loader.

A local arm64 C++ solver benchmark used 500 sampled source-bank cases: all
converged with net clearance, with median 0.969 ms, 95th percentile 1.357 ms and
maximum 1.884 ms. This measures the local solver, not hardware latency or policy
performance. The source bank for that timing sample was the original recipe-19
bank; the full qualification counts above use the newly published recipe 21.

Exact retained-row errors, commands, source/output hashes and complete test
outcomes are recorded in the
[validation record](nohope_main_improve_2_validation.json). No Isaac training run
or learned-policy evaluation ran on this macOS host. Refitted datasets remain
reproducible local outputs; the PR includes the original source banks. The
user-reported MuJoCo standing result is separate from these numerical checks.

## Sweet-spot recipe 23 update

New PPQ004 training now enables a placement reward and uses recipe 23, with
distribution `stationary_reach_ppq004_sweet_spot_v1` and reward revision
`actual_complete_hold_v11_stationary_sweet_spot_v1`. The objective comes from
page 2 of the supplied **HOPE Table Tennis Sweet Spot Map**, dated 19 October
2026; page 1 is an own-side reference patch. The canonical objective YAML defines
an inclusive 20 × 15 cm opponent-side rectangle: table-frame X `[2.24, 2.44]` m,
Y `[-0.575, -0.425]` m, centre `(2.34, -0.50)` m. The configured table placement
maps that centre to `(2.84, 0.2625)` m in the training environment. The landing
event remains the first descending ball-centre contact at surface plus radius.

Recipe 23 preserves source incoming states, contact positions, parent/split
identity and synthetic provenance, while retargeting commands to the rectangle
centre. Original targets remain in `input_intended_landing_xy`. The full PPQ004
model, physical bounds and qualification identity are unchanged. The production
loader checks the objective hash/descriptor, intended centres and stored landing
positions, so relabelling a general-return receipt is insufficient.

The shaping adapts the exponent 3/4 and bounded distance-penalty idea from
Dieter Büchler, Simon Guist, Roberto Calandra, Vincent Berenz, Bernhard Schölkopf
and Jan Peters, **“Learning to Play Table Tennis From Scratch Using Muscular
Robots,” IEEE Transactions on Robotics 38(6), 3850–3860 (2022)**
([DOI](https://doi.org/10.1109/TRO.2022.3176207),
[preprint Section II-B / Eq. (3)](https://arxiv.org/html/2006.05935v1#S2.SS2)).
Here distance is measured to the rectangle, normalized by 2.5 m, and shaped as
`max(0, 1 - (d / 2.5)^0.75)`, with a 1.0 bonus inside the rectangle and weight
30.0. These are initial settings for this implementation, not the publication's
robot-specific weights or an assertion of its experimental results. The reward
uses achieved racket contact and full spin-sensitive first-bounce prediction,
is consumed once per strike, and requires captured contact plus a legal return.
HOLD remains 1.7; no contact-centre or flight-height reward is added.

`task.sweet_spot.reward_enabled=false` disables only this reward, preserving the
same recipe-23 bank, objective, motions, action convention, PPO settings and
existing reward weights. This is the controlled ON/OFF comparison. Selecting
`HOPEPingPongStationaryPPQ004General` instead uses the separate recipe-22
general-return baseline. Checkpoint and export metadata bind the objective,
bank, switch and shaping parameters; incompatible optimizer resumes fail.
The independent evaluator restores the saved task/algorithm and reports
placement metrics for both ON and OFF runs, including per-side unconditional
hit rates, contacts/legal returns, interruptions/falls and finite-contact
landing-distance percentiles.

The standalone evaluator preserves its geometric 0.10 m contact gate and does
not apply the live reward's forward-approach gate. Live reward capture uses the
saved 0.095 m radius and 0.3 m/s minimum approach speed in the current recipe.
Matched ON/OFF evaluations must use identical evaluator settings; their rates
are distinct from the live reward diagnostics.

The generated recipe-23 bundle retained 393,930 training, 48,318 development
and 48,555 test rows, including 152,409 forehand and 241,521 backhand training
questions. All stored targets equal the canonical centre and all stored
landings lie inside the rectangle; unchanged physical constraints and split
provenance were checked. Both reward settings passed prelaunch, standalone
dry-run and the production bank loader. The CPU suite passed **543 tests**,
with 11 skipped for unavailable Isaac/CUDA or a second GPU. Exact hashes,
commands and audit limits are recorded in the
[sweet-spot validation record](sweet_spot_reward_validation.json).

These are implementation, numerical and data-integrity results. No new policy
training, matched ON/OFF policy evaluation, Isaac/GPU training run or hardware
test was performed. The earlier user-reported MuJoCo standing result does not
establish an improvement from the new placement reward.
