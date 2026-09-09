# Training the policy

The training package (`hope_training/whole_body_tracking/`) is an Isaac Lab extension that trains one
feed-forward actor — shared by forehand and backhand — with PPO and a privileged critic. This page
covers the design; exact commands and the package layout are in the
[package README](../hope_training/whole_body_tracking/README.md) and
[QUICKSTART_A3_ISAAC.md](../QUICKSTART_A3_ISAAC.md).

## The task

The shipped task is a Hydra YAML, `cfg/task/HOPEPingPong.yaml`, composed on top of the shared
`cfg/base/*.yaml` defaults:

- **`HitterPingPong`** (gym id `HOPE-HitterPingPong-AgibotA3-v0`) — the deploy-grade recipe
  validated on real A3 hardware, and the only task shipped. It trains the 110-D
  `hitter_pure` actor contract (HITTER-style proprioception + goal only, no `swing_side` term — the
  side is inferred outside the policy).

A task selects its observation layout via the `actor_obs_contract` key; the named contracts
(`full`=180, `deploy_parity`=175, `hitter_footwork`=177, `hitter_pure`=110,
`hitter_pure_v15`=118) are defined in one place,
`tasks/tracking/actor_observation_contract.py` — see
[POLICY_INTERFACE.md](POLICY_INTERFACE.md).

Common properties of the rally line:

- **Two motion clips** — clip 0 forehand, clip 1 backhand — imitated by the upper body. A new side is
  chosen per swing, so all four adjacent transitions (FH→FH, FH→BH, BH→FH, BH→BH) appear across a
  batch.
- **A racket-target goal**: a sampled racket target position, target velocity, and time-to-strike fed
  to the actor. The recipe draws 25% of its per-side question bank from a physical tuple bank
  fitted to the real venue (the `racket.venue_tuple_*` keys in the task YAML).
- **A base station**: the observation carries a base-target delta so the policy recentres in place as
  it drifts over a rally (Δ = 0 on mocap dropout, matching deploy).
- **Continuous rallies**: robot state, joint state, and `last_action` carry across swings; the
  environment only resets on the episode timeout or a physical fall.
- **50 Hz** control (decimation 4 over a 200 Hz physics step), **no observation normalization**.

The critic additionally sees privileged, simulation-only signals (the reference joint stream,
motion-anchor errors, and the true racket state) for its value estimate. Those never enter the
deployed policy.

## Launching a run

On a new machine, first reproduce the pinned workstation environment in
[`DISTROBOX_SETUP.md`](DISTROBOX_SETUP.md), then complete steps 0–4 of
[QUICKSTART_A3_ISAAC.md](../QUICKSTART_A3_ISAAC.md): materialize Git LFS,
enter `grasping`, deactivate Conda, source `setup_train_env.sh`, prepare the A3
asset, and run the bounded smoke checks. The pinned launcher uses the
working-tree source directly; no editable install is required.

```bash
distrobox enter grasping
if [[ -n "${CONDA_PREFIX:-}" ]]; then conda deactivate; fi
cd "$HOME/workspace/HOPE/hope_training/whole_body_tracking"
source setup_train_env.sh        # defines the hope_isaac_py launcher
hope_isaac_py scripts/train.py task=HOPEPingPong algo=ppo headless=true \
    motion_file=../motions/preprocessed/hope_forehand.npz \
    motion_file_2=../motions/preprocessed/hope_backhand.npz
```

Everything is a Hydra override: `num_envs=4096`, `max_iterations=20000`, `seed=1`,
`checkpoint_path=<run>/model_<N>.pt` (resume), or any dotted config path. Local `.npz` clips via
`motion_file=` / `motion_file_2=` are first-class and require no external artifact registry. The
shipped clips are the complete validated Build pair. See
[REPLACE_MOTIONS.md](REPLACE_MOTIONS.md) only when substituting your own motions.

## Reward terms

Reward term functions live in `tasks/tracking/mdp/hope_rewards.py`; the task YAML composes and
weights them. The recipe is deliberately documented in its own YAML
(`cfg/task/HOPEPingPong.yaml`): the complete question bank, rewards, and
domain randomization, plus a behavior-identical affine q_des clamp, a joint-acceleration
regularizer, the 25% physical tuple bank, and deploy-faithful mocap observations. Tune weights by
editing the task YAML — **the YAML overrides Python config defaults**, so edit the YAML, not the
env-cfg dataclass, when both define a value.

## Evaluation

The evaluation story has three layers, cheap to expensive:

1. **Isaac evaluation** — `scripts/evaluate.py` rolls the checkpoint out across many parallel
   environments and reports the in-Isaac `success_rate` estimate (racket contact, net crossing,
   opponent-half first bounce, judged with the shared no-spin ball model).

   ```bash
   hope_isaac_py scripts/evaluate.py --checkpoint <run>/model_<N>.pt \
       --motion-file /abs/fh.npz --motion-file-2 /abs/bh.npz
   ```

2. **MuJoCo sim-to-sim** — `scripts/mujoco_eval_onnx.py --onnx <run>/exported/policy.onnx` runs the
   exported ONNX against a real MuJoCo ball that physically bounces off the racket, table, and net
   (no GPU, no Isaac). This is the first cross-simulator check and the authoritative simulated
   number.

3. **Closed-loop planner rehearsal** — the reference harness's `--planner` mode exercises the
   exported policy through the real planner over the same flat wire the hardware runner
   subscribes ([RUN_ON_AGIBOT.md](RUN_ON_AGIBOT.md)).

Isaac metrics do not perfectly predict downstream behavior — treat each layer as a gate, not a
ranking oracle.

## Export

```bash
hope_isaac_py scripts/export_onnx.py --checkpoint <run_dir>/model_<N>.pt
```

writes the single-output actor ONNX (110-D observation in, 31-D raw action out, raw
observations) and its deploy manifest to `<run_dir>/exported/`. The exported metadata is what
the deploy-side loaders validate; hardware promotion additionally goes through the closed-loop
rehearsal and gate sweeps above. See [POLICY_INTERFACE.md](POLICY_INTERFACE.md).

## Checkpoints and logging

Checkpoints are written locally to `logs/rsl_rl/<experiment_name>/<timestamp>/` (a periodic
checkpoint every `save_interval` iterations and a final one). The public launcher uses local
TensorBoard logging and does not upload runs or checkpoints. Checkpoint selection is yours to do
with the evaluators above — nothing auto-promotes a checkpoint.

## Configuration

- `cfg/task/HOPEPingPong.yaml` — the task recipe: motion clip sources, contract selection
  (`actor_obs_contract`), rewards/DR recipe, and episode length.
- `cfg/algo/ppo.yaml` — PPO (`empirical_normalization: false`, i.e. raw observations), iteration and
  save intervals.
- `cfg/base/*.yaml` — shared env / sim / randomization defaults.

Base env-cfg dataclasses live in `tasks/tracking/config/agibot_a3/hope_env_cfg.py`; remember the
task YAML wins over Python where both set a value.

## The user runs training

Training needs your GPU; the validated reference pair is included, or you can supply your own
motion clips. Run it yourself with the commands above.
Launch from `hope_training/whole_body_tracking/` so the relative motion paths resolve.
