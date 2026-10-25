# HOPE Isaac Lab training

## Default: PPQ004 stationary rally with sweet-spot placement

The default training recipe uses **PPQ004 / 3-star ball physics**, fixed-station
rally training, balanced-hold PPO and an enabled sweet-spot landing reward. It retains **HOLD weight 1.7**, uses
**6144 environments**, **30000 iterations**, **seed 0**, and starts from scratch:
both `checkpoint_path` and `warm_start_actor_checkpoint` are `null`. It does not
load model_12600 or the bundled model_14600.

`cfg/train.yaml` selects `task=HOPEPingPongStationaryPPQ004` and
`algo=ppo_balanced_hold`. This task extends the imported StationaryRally strategy
and requires a matching PPQ004 bank, receipt and physics profile before Isaac
starts. The actor/action dimensions remain 110/31 at 50 Hz, with the stationary
recipe's `B_32_5_K20` ready pose and action zero.

**Published source bank:** the developer has now uploaded the screenshot's
**revision-21** generalization bank at fork commit
`9c4d6366ab7bd58b73baf98c06159e1ac51688d2`. Its train SHA-256 is
`02619e747333980a6f1d48aa3a9c7896e15727cf95cd3f51eb387d7687c4b419`.
The immutable input bank, receipts and generation report are included under
`assets/stationary_reach_v4_ppq004_generalization_20261022`: 502,053 training,
61,815 development and 62,103 test questions. Of the training rows, 318,779 are
synthetic additions; they are not new measured recordings.

The reproducible default below generates a **revision-23 sweet-spot bank**
from that published revision 21. It preserves incoming states, contact positions,
parent IDs, split membership and synthetic-source provenance. The original
intended landing is retained as `input_intended_landing_xy`; the new
`intended_landing_xy` is the sweet-spot centre. Racket commands are recomputed
and qualified under the full model and unchanged reach/velocity bounds. The upstream revision-21 labels use a table-surface landing plane;
they are not interchangeable with the physical center-contact convention here.
The source bank's README describes its upstream default, while the commands
below select this integration's full-model default.

### Full model and first-bounce convention

The Python training/refitter/evaluation paths and C++ simulation/runtime planner
use the full paddle-to-first-bounce calculation: exponential normal restitution,
capped tangential impulse and spin transfer at the racket, followed by RK4 flight
with quadratic drag, gravity and Magnus. Spin is constant during each free-flight
segment. The landing target is the **first descending ball-center contact at
`table surface + ball radius`**, not a later bounce or the table-surface plane.
The full inverse solves that first-bounce position and flight time and rejects
nonconvergence. Actual contact is recomputed from the stored racket command
before bank acceptance.

`../../configs/ball_physics_ppq004_training.yaml` selects PPQ004 and references
its runtime profile, shared `return_physics.yaml`, geometry, and runtime settings.
Fitted coefficients are read from YAML rather than duplicated in Python/C++.
The refitter, runtime planner and fine evaluator use the YAML-selected 0.5 ms
integration step. The vectorized training reward uses its configured 10 ms step
and 200-step horizon under the same full equations; nine numerical regression
cases keep landing/net differences below 1 mm versus the fine model. GPU
throughput has not been measured. The profile also binds the stationary table
placement. Receipts hash all six YAML sources plus
the shared full-model and profile-loader implementations. A changed dependency
requires regeneration before the default task can launch.

### Prepare the environment and PPQ004 bank

Run inside the existing `grasping` container; from the host, enter it with
`distrobox enter grasping`. Adjust the checkout path below for your machine.

```bash
cd "$HOME/workspace/HOPE/hope_training/whole_body_tracking"
if [[ -n "${CONDA_PREFIX:-}" ]]; then
  conda deactivate
fi
source setup_train_env.sh
```

Generate the matching bank once with the offline refitter (Numba is required in
this interpreter). Use a new output directory; the tool refuses to overwrite an
existing one.

```bash
hope_isaac_py scripts/refit_stationary_reach_bank.py \
  --source assets/stationary_reach_v4_ppq004_generalization_20261022 \
  --output outputs/stationary_ppq004_sweet_spot \
  --physics ../../configs/ball_physics_ppq004_training.yaml \
  --landing-objective ../../configs/return_placement_sweet_spot.yaml \
  --threads 6
```

This produces train/dev/test banks, receipts and
`outputs/stationary_ppq004_sweet_spot/training_overrides.yaml`. Keep the bundle together.
The manifest records absolute paths. If you move an identical checkout or
bundle, update its profile, bank, receipt and `task.landing_objective.path`
fields; keep the checksums and
relative YAML references intact. Validation compares dependency names and hashes
across locations and rejects ambiguous names or changed content. To use another output directory, pass
`task.ppq004_refit_manifest=/absolute/path/training_overrides.yaml` when training.

### Start the default training run

Check the generated bundle without starting Isaac:

```bash
hope_isaac_py scripts/train_refitted_stationary.py \
  --overrides outputs/stationary_ppq004_sweet_spot/training_overrides.yaml --dry-run \
  num_envs=6144 max_iterations=30000 seed=0 \
  checkpoint_path=null warm_start_actor_checkpoint=null
```

Then start training with the default task and algorithm:

```bash
hope_isaac_py scripts/train.py \
  headless=true \
  device=cuda:0 \
  num_envs=6144 \
  max_iterations=30000 \
  seed=0 \
  checkpoint_path=null \
  warm_start_actor_checkpoint=null \
  run_name=stationary_reach_ppq004_sweet_spot_scratch
```

Training verifies the bank, receipt, shared model/loader code, and every referenced
physics YAML hash and landing-objective identity, applies the bank fields and table placement,
and sets `HOPE_BALL_PHYSICS_YAML` to the verified PPQ004 profile before Isaac
starts. A stale shell value cannot silently select a different profile. A missing
or mismatched bundle stops the run; it does not fall back to the older bank.
The resolved task records `stationary_reach_ppq004_sweet_spot_v1`, the landing
objective and its hash, reward settings, and the bank identity.
Old-physics and superseded simplified-model optimizer resumes are rejected.
The required qualification is `full_ppq004_first_bounce_v1`; changing a receipt
revision number does not convert an older bank. Selecting only
`task=HOPEPingPongStationaryRally` selects the original venue-bank recipe;
use the PPQ004 default above for new training.

For a short training-path check, override `num_envs=64 max_iterations=3` and use
a separate `run_name`. This checks execution, not convergence. To play a
checkpoint from the new recipe, select its matching task and algorithm explicitly:

```bash
hope_isaac_py scripts/play.py \
  task=HOPEPingPongStationaryPPQ004 algo=ppo_balanced_hold \
  checkpoint=/absolute/path/to/model.pt
```

Playback uses the selected configuration. Repeat any saved manifest, reward or
motion overrides for that checkpoint; use the evaluator below to restore its
saved task and algorithm automatically.

### Sweet-spot region, reward and publication credit

The fixed target comes from page 2 of the supplied **HOPE Table Tennis Sweet Spot
Map**, dated 19 October 2026. Page 1 is an own-side reference patch and is not the
return target. The bounds are stored once in
[`configs/return_placement_sweet_spot.yaml`](../../configs/return_placement_sweet_spot.yaml):

| Coordinates | X bounds (m) | Y bounds (m) | Centre (m) |
| --- | --- | --- | --- |
| Table near-left corner, +X toward opponent, +Y left | [2.240, 2.440] | [-0.575, -0.425] | (2.340, -0.500) |
| Current training environment | [2.740, 2.940] | [0.1875, 0.3375] | (2.840, 0.2625) |

The 20 x 15 cm rectangle has inclusive boundaries. Environment coordinates are
derived from the YAML table placement; ball-centre bounce height is derived from
surface height plus radius. Rectangle geometry is on the tabletop, so do not add
ball radius to the XY bounds or add it twice to the landing plane. All qualified
bank commands aim at the rectangle centre. The reward gives equal placement
credit throughout the rectangle, using distance to the nearest point in it.

The reward is an adaptation of **Dieter Büchler, Simon Guist, Roberto Calandra,
Vincent Berenz, Bernhard Schölkopf and Jan Peters, “Learning to Play Table Tennis
From Scratch Using Muscular Robots,” IEEE Transactions on Robotics 38(6),
3850-3860 (2022)**. [Publication DOI](https://doi.org/10.1109/TRO.2022.3176207),
[open preprint, Section II-B / Eq. (3)](https://arxiv.org/html/2006.05935v1#S2.SS2).
Their return reward uses a distance penalty with exponent 3/4 and a lower bound.
This implementation credits that exponent and shaping idea; it does not copy
their robot-specific normalization or claim their experimental results.

Our adaptation uses dimensionless rectangle distance, a nonnegative floor,
and a legal-region bonus:

```text
d = Euclidean distance from predicted first-bounce XY to the rectangle
shape = max(0, 1 - (d / distance_scale_m) ** 0.75)
raw_reward = legal_captured_strike * (shape + inside_bonus * inside_rectangle)
```

Defaults are `distance_scale_m=2.5`, `inside_bonus=1.0` and
`reward_weight=30.0`. Isaac Lab applies the configured weight and control-step
duration to the raw one-shot reward. These are initial experimental settings,
not transferred publication weights. The event uses achieved racket state and
incoming spin with the full PPQ004 contact/flight model. Failed contact, invalid
prediction, net failure and off-table returns receive no placement reward.
Existing tracking rewards supply pre-contact learning; HOLD remains 1.7. The
reward is consumed once per strike at the reward phase, independently of the
later command update. No blade-contact-centre reward or flight-height penalty is
added.

### Turn the reward off for a controlled comparison

The PPQ004 default has `task.sweet_spot.reward_enabled=true`. Disable only the
new reward while keeping exactly the same qualified target bank:

```bash
hope_isaac_py scripts/train.py \
  task.sweet_spot.reward_enabled=false \
  checkpoint_path=null warm_start_actor_checkpoint=null \
  run_name=stationary_ppq004_sweet_spot_reward_off
```

The switch, reward parameters, target objective and bank identity are recorded in
checkpoint/export metadata. Incompatible optimizer resumes are rejected; use
matched scratch runs for the reward-on/off comparison, or an explicit actor-only
warm start with a fresh critic/optimizer. The independent evaluator reports
sweet-spot metrics even when its reward is off. `scripts/evaluate.py` retains the existing `success_rate`
and adds a separate `sweet_spot` summary for these checkpoints: unconditional
region-hit rates by side, contacts/legal returns, pre-strike interruptions/falls,
post-strike falls, and distance percentiles over finite captured first bounces.
A miss or pre-strike fall cannot disappear from the placement denominator.

Evaluate a trained checkpoint using its adjacent `params/task_resolved.yaml`:

```bash
hope_isaac_py scripts/evaluate.py \
  --checkpoint /absolute/path/to/model.pt \
  --num-envs 256 --num-steps 4000 \
  --json-out /absolute/path/to/evaluation.json
```

If the saved configuration is stored elsewhere, supply
`--resolved-config /absolute/path/to/task_resolved.yaml`. The evaluator verifies
the saved bank, physics, objective and reward identity against the bundle and
checkpoint; changing the saved reward switch cannot relabel an existing run.

The evaluator retains its geometric contact gate, `--contact-radius=0.10` m by
default, without the training reward's forward-approach gate. The live reward
uses the saved `racket.vb_capture_radius` (currently 0.095 m) and
`racket.vb_min_approach_speed` (currently 0.3 m/s). Keep the same evaluator
settings for ON/OFF comparisons; its contact and region-hit rates need not
equal the live reward diagnostics.

The original full-PPQ004 general-return bank remains available separately. Omit
`--landing-objective` when refitting to a new directory to produce recipe 22,
then explicitly select `task=HOPEPingPongStationaryPPQ004General` and its manifest.
Turning off the sweet-spot reward does not silently select that different bank.

## Evidence and required assets

The user reports firm standing in MuJoCo simulation, with no real-world
evaluation. No matching checkpoint or measured evaluation report was supplied.
The PPQ004 refit has passed numerical geometry checks; those checks do not
establish learned-policy balance or success rates. See the
[strategy review](docs/nohope_main_improve_2_review.md) and
[full-model baseline validation](docs/nohope_main_improve_2_validation.json),
plus the [sweet-spot validation record](docs/sweet_spot_reward_validation.json).

The required forehand/backhand motion files are included under
`../motions/preprocessed/`. The prepared A3 Isaac asset remains local; prepare
it from this checkout's vendor package with
`hope_isaac_py scripts/prepare_a3_isaac_asset.py`, then pass `--check` to verify.
One A3 Gym task, `HOPE-HitterPingPong-AgibotA3-v0`, is registered; Hydra selects
the training recipe. Local logs, generated banks and outputs stay separate from
source.

## Legacy recipes and model_14600

The previous `HOPEPingPong` / `ppo` recipe remains available explicitly. It comes
from `origin/main_improve@2b14b76d184643320c158ad938ab3cf6ba8a1624`; `ppo` was named
`ppo_quiet` there. Its flattened task exactly matches the resolved task embedded
in the supplied model_14600 ONNX. The recipe is
`continuous_rally_sim2real_v3`, revision 3, with reward
`actual_task_pose_v6_1_hard_stop_clearance`.

For legacy venue-physics training, clear both shell physics overrides and select
both groups. The prelaunch helper then explicitly binds the venue profile:

```bash
unset HOPE_BALL_PHYSICS_YAML HOPE_BALL_PHYSICS_CONFIG
hope_isaac_py scripts/train.py task=HOPEPingPong algo=ppo \
  num_envs=6144 max_iterations=30000 headless=true device=cuda:0 seed=1 \
  run_name=model14600_task_reference \
  checkpoint_path=null warm_start_actor_checkpoint=null
```

The imported `HOPEPingPongCompleteHold` and `HOPEPingPongStationaryRally` recipes
also remain available with `algo=ppo_balanced_hold`; both use their original
venue-physics banks. The separate PPQ004 task preserves those source recipes.

`cfg/play.yaml` retains the legacy `HOPEPingPong` / `ppo` defaults for existing
checkpoints. The `checkpoints/` directory still contains the unchanged model_14600
[ONNX](checkpoints/policy.onnx), [deploy metadata](checkpoints/deploy.yaml) and
[actor parity samples](checkpoints/actor_parity.txt). Their hashes and task
identity are recorded in the [provenance receipt](docs/model_14600_training_provenance.json).
These files are not a resumable training checkpoint.

A local legacy smoke test on 2026-10-13 used 64 environments and three PPO
updates (12,288 transitions). It exited successfully and changed actor weights
with finite model and optimizer tensors. That result applies to the legacy
training path, not the PPQ004 default or a converged policy.
