# HOPE Isaac Lab training recipes

The default task, `HOPEPingPong`, and its algorithm
`ppo` (named `ppo_quiet` in the source branch), come from `origin/main_improve@2b14b76d184643320c158ad938ab3cf6ba8a1624`.
The task YAML is the flattened `HOPEPingPong -> HOPEPingPongQuiet ->
HOPEPingPongTaskReference` inheritance chain, with all values unchanged.
It exactly matches the resolved task embedded in the supplied model_14600 ONNX.

- Gym ID: `HOPE-HitterPingPong-AgibotA3-v0`.
- Recipe: `continuous_rally_sim2real_v3`, revision 3.
- Reward: `actual_task_pose_v6_1_hard_stop_clearance`.
- Actor/action: 110 observations, 31 actions, 50 Hz.
- Recipe source: [task YAML](cfg/task/HOPEPingPong.yaml).
- Algorithm: [PPO YAML](cfg/algo/ppo.yaml).
- Identity evidence: [policy provenance](docs/model_14600_training_provenance.json).
- Baseline export: [model_14600 policy](checkpoints/policy.onnx),
  [deploy.yaml](checkpoints/deploy.yaml) and
  [actor parity samples](checkpoints/actor_parity.txt).

The `checkpoints/` directory holds **model_14600** as the
baseline. These three files are byte-identical
to the supplied model_14600 download. Their hashes are recorded in the provenance
receipt above.

Both `cfg/train.yaml` and `cfg/play.yaml` select this task and algorithm by
default. The shared ancestor configurations, MDP functions and robot/table
utilities remain dependencies. One A3 Gym task is registered; additional training
recipes below select different rewards and sampling through Hydra.

Enter the existing `grasping` environment, deactivate Conda base if active,
and source the local setup before training. To use the selected task with the
recorded run's 6144 environments, 30000-iteration limit and seed 1:

```bash
cd "$HOME/workspace/HOPE/hope_training/whole_body_tracking"
source setup_train_env.sh
hope_isaac_py scripts/train.py task=HOPEPingPong algo=ppo num_envs=6144 max_iterations=30000 headless=true device=cuda:0 seed=1 run_name=model14600_task_reference checkpoint_path=null warm_start_actor_checkpoint=null
```

The command starts a fresh training run. A local smoke test on 2026-10-13 used
`task=HOPEPingPong algo=ppo num_envs=64 max_iterations=3`: all three PPO updates
completed (12,288 transitions), the process exited successfully, and the saved
actor weights changed with finite model and optimizer tensors. This verifies
the short training path, not convergence or the full 6144-environment run.
The supplied download contains ONNX/deploy/parity files, not a resumable training
checkpoint. Task equality does not prove an identical optimizer, simulator
state, or reproducible final policy weights.

The required forehand/backhand motion files are included under
`../motions/preprocessed/`. The prepared A3 Isaac asset remains local; to
prepare it from this checkout's vendor package, run
`hope_isaac_py scripts/prepare_a3_isaac_asset.py`, then pass `--check` to verify.
Existing local training logs and outputs are retained separately from source.

## Improved standing and stationary rally

The training-only port of `nohope/main_improve_2@54952d0f1` adds explicit
`HOPEPingPongCompleteHold` and `HOPEPingPongStationaryRally` recipes, paired with
`algo=ppo_balanced_hold`. The existing defaults and model_14600 bundle remain.

See [the strategy review and PPQ004 training instructions](docs/nohope_main_improve_2_review.md)
for the changes, required datasets and evidence limits. The reported firm-standing
result was tested by the user in MuJoCo; no real-world evaluation is claimed.
The checked-in banks use the prior venue physics: PPQ004 requires explicit refitting
and the receipt-bound launcher documented there.
