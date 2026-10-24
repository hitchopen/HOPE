# CompleteHold and StationaryRally training review

Source: `dongc1/nohope`, branch `main_improve_2`, commit
`54952d0f11252cbef7b156c1d46d49389b75dc95` (verified against the remote).
Integration base: `hitchopen/HOPE:nightly_built` at
`91aecabfd44cb258ac998158e724be9017d883a7`.

## Scope and compatibility

This is a selective Isaac Lab RL training port. HOPE already contained the
flattened model_14600 recipe from the fork's `2b14b76d1`; the source integration
applies the subsequent training changes without restoring retired Gym tasks.
The CompleteHold and StationaryRally task/algorithm pairs resolve identically
to the source fork. `HOPEPingPong` / `ppo` remain the default train/play pair.

The port includes the active recipe inheritance, actual-pose and balance
rewards, fixed-station sampling, interruption-aware recovery, PPO diagnostics,
training metadata, required train/dev/test banks, offline bank preparation and
regression tests. Historical stationary v1-v3 banks and inactive four-clip /
joint-clock recipe assets and scripts are omitted. Shared source helpers remain
where the imported command implementation references them.

The actor/action dimensions remain 110/31 at 50 Hz. StationaryRally explicitly
changes the reset pose and affine action zero to `B_32_5_K20`; equal dimensions
do not make its action offsets interchangeable with older exports. Use the
resolved training/export metadata for any later deployment work. This port
changes no native Runner, planner, ROS, mocap, hardware configuration, published
checkpoint, ONNX model or deployment asset.

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

The source branch includes a PPQ004 profile and offline bank refitter, but its
StationaryRally YAML still pins the original recipe-19 bank. The virtual-ball
loader also defaults to `ball_physics_venue.yaml` unless
`HOPE_BALL_PHYSICS_YAML` is set. Merely selecting StationaryRally does not select
the 3-star profile. No PPQ004-refitted bank or newly qualified policy is silently
substituted for the existing defaults in this PR.

The PPQ004 profile updates the fitted flight coefficients and provides the
updated table coefficients. Refitting retains at-strike incoming velocity/spin,
contact position, intended landing, parent identity and split membership, and
adjusts racket velocity before coarse/fine qualification. It cannot identify
pre-bounce states from this bank. The paddle fit remains the older venue fit;
the table surface convention remains 0.76 m. This is not full C++ planner or
incoming table-bounce parity. Bank geometry checks do not evaluate robot balance.

## Reproduce the selected recipes

Inside the supported Isaac environment, from `hope_training/whole_body_tracking`:

```bash
source setup_train_env.sh
hope_isaac_py scripts/train.py task=HOPEPingPongCompleteHold algo=ppo_balanced_hold \
  headless=true max_iterations=30000 seed=1
hope_isaac_py scripts/train.py task=HOPEPingPongStationaryRally algo=ppo_balanced_hold \
  headless=true max_iterations=30000 seed=1
```

Both commands above use the checked-in venue-physics banks. To explicitly use
the PPQ004/3-star at-strike refit, install Numba in the offline preparation
interpreter if needed, then run:

```bash
hope_isaac_py scripts/refit_stationary_reach_bank.py \
  --source assets/stationary_reach_v4 \
  --output outputs/stationary_ppq004 \
  --physics ../../configs/ball_physics_ppq004_training.yaml --threads 6
hope_isaac_py scripts/train_refitted_stationary.py \
  --overrides outputs/stationary_ppq004/training_overrides.yaml --dry-run \
  max_iterations=30000 seed=1
hope_isaac_py scripts/train_refitted_stationary.py \
  --overrides outputs/stationary_ppq004/training_overrides.yaml \
  max_iterations=30000 seed=1
```

Use a new output directory for each refit. The launcher binds the physics profile,
bank and receipt hashes, the stationary task, balanced PPO, and a distinct PPQ004
training-distribution identity. Old-physics optimizer resumes are rejected;
explicit actor warm-start remains available through
`warm_start_actor_checkpoint=/absolute/path/model.pt`. The launcher does not
turn a geometry-qualified bank into a qualified policy.

For learned-policy confirmation, retain the checkpoint hash, resolved task and
physics/bank hashes, seed, evaluation duration, fall count, ready-pose/dwell
metrics and per-side contact/return rates. A later hardware evaluation is separate
from this simulation-training contribution.

## Integration validation

CPU suite: **415 passed, 11 skipped**. Skips require Isaac Lab or CUDA; no
Isaac rollout ran on this macOS host. The suite includes real RSL-RL PPO
updates, fixed-station/pose and dataset contracts, and numerical Torch/Numba
physics parity. A diagnostics-variable collision in the fork's PPO update was
fixed and tested with diagnostics both enabled and disabled.

A full offline PPQ004 refit retained 183,274/183,315 training rows,
21,852/21,859 development rows and 22,452/22,457 test rows, with at least
50,000 training rows per side and disjoint parent IDs. The 1 ms recheck gave
a training landing-fit error of 0.0376 mm P95 and 0.900 mm maximum. These are
numerical fitting errors, not measured policy returns or robot stability.
The generated bundle passed the verified launcher's dry run.

See [the validation record](nohope_main_improve_2_validation.json) for commands,
runtime versions, source/output hashes and per-split results. Refitted datasets
remain reproducible local outputs; the PR includes the original source banks.
