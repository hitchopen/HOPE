# HOPE Isaac Lab training

## Default: PPQ004 stationary rally from scratch

The default training recipe uses **PPQ004 / 3-star ball physics**, fixed-station
rally training and balanced-hold PPO. It retains **HOLD weight 1.7**, uses
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

The reproducible default below generates a new **revision-22 full-PPQ004
first-bounce bank** from that published revision 21. It preserves incoming
states, contact positions, intended landings, parent IDs, split membership and
synthetic-source provenance, while recomputing racket commands under the shared
full model. The upstream revision-21 labels use a table-surface landing plane;
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
  --output outputs/stationary_ppq004_full \
  --physics ../../configs/ball_physics_ppq004_training.yaml \
  --threads 6
```

This produces train/dev/test banks, receipts and
`outputs/stationary_ppq004_full/training_overrides.yaml`. Keep the bundle together.
The manifest records absolute paths. If you move an identical checkout or
bundle, update its profile, bank and receipt path fields; keep the checksums and
relative YAML references intact. Validation compares dependency names and hashes
across locations and rejects ambiguous names or changed content. To use another output directory, pass
`task.ppq004_refit_manifest=/absolute/path/training_overrides.yaml` when training.

### Start the default training run

Check the generated bundle without starting Isaac:

```bash
hope_isaac_py scripts/train_refitted_stationary.py \
  --overrides outputs/stationary_ppq004_full/training_overrides.yaml --dry-run \
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
  run_name=stationary_reach_ppq004_full_scratch
```

Training verifies the bank, receipt, shared model/loader code, and every referenced
physics YAML hash, applies the bank fields and table placement,
and sets `HOPE_BALL_PHYSICS_YAML` to the verified PPQ004 profile before Isaac
starts. A stale shell value cannot silently select a different profile. A missing
or mismatched bundle stops the run; it does not fall back to the older bank.
The resolved task records `stationary_reach_ppq004_full_v1` and the bank identity.
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

## Evidence and required assets

The user reports firm standing in MuJoCo simulation, with no real-world
evaluation. No matching checkpoint or measured evaluation report was supplied.
The PPQ004 refit has passed numerical geometry checks; those checks do not
establish learned-policy balance or success rates. See the
[strategy review](docs/nohope_main_improve_2_review.md) and
[validation record](docs/nohope_main_improve_2_validation.json).

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
