# HOPE PingPong — whole-body training (Agibot A3)

This Isaac Lab extension trains the **HOPE PingPong** whole-body policy for the
[Agibot A3](https://www.zhiyuan-robot.com/) humanoid (31 actuated DOF): a single feed-forward actor,
shared by forehand and backhand, that runs at **50 Hz** and drives a table-tennis swing.

- **Observation:** `float32[111]` (raw — no normalization). See [`docs/POLICY_INTERFACE.md`](../../docs/POLICY_INTERFACE.md).
- **Action:** `float32[31]` raw joint-position residual (joint order in
  [`hope_training/config/joint_order_agibot_a3.yaml`](../config/joint_order_agibot_a3.yaml)).
- **Task:** one Gym task, `HOPE-PingPong-AgibotA3-v0`, selected with `task=HOPEPingPong`.

The metric reported by the evaluators is a single number, **`success_rate`** (a returned ball must be
contacted, cross the net, and land its first bounce on the opponent half).

## Install

Requires Isaac Sim + Isaac Lab (with `rsl_rl`), Python 3.10, and an NVIDIA CUDA GPU. Follow the
[Isaac Lab installation guide](https://isaac-sim.github.io/IsaacLab/main/source/setup/installation/index.html)
first, then install this extension into the Isaac Lab Python:

```bash
python -m pip install -e source/whole_body_tracking
# extra deps used by the Hydra entry points (import them in the Isaac Lab python):
python -m pip install hydra-core omegaconf
```

Optionally `source setup_train_env.sh` (in the GPU/Isaac shell) to put the working-tree source first on
`PYTHONPATH` and get an `isaac_py` launcher.

## Robot asset (bundled URDF)

The starter ships the Agibot-provided A3 ping-pong URDF package under
`agibot/URDF/A3T2.5-URDF-std-pingpang/` (vendor material, no OSS license — see
[`A3_ASSETS.md`](../../A3_ASSETS.md)); the asset-prep step uses it by default. To use your own
vendor-supplied copy, place it under `a3_deploy/URDF/` (see
[`a3_deploy/URDF/README.md`](../../a3_deploy/URDF/README.md)) and pass `--source-root`:

```bash
python scripts/prepare_a3_isaac_asset.py --force
python scripts/prepare_a3_isaac_asset.py --source-root a3_deploy/URDF/<your_a3_package> --force
python scripts/prepare_a3_isaac_asset.py --check   # verify the prepared asset
```

## Motion clips — reference examples only

Training imitates two reference clips (clip 0 = forehand, clip 1 = backhand). The clips shipped under
`hope_training/motions/preprocessed/` (`hope_forehand.npz` + `hope_backhand.npz`) are **reference
examples only** — short, smooth, physically-neutral placeholder trajectories that let imports and
shape checks pass. **They are not performance-tuned; replace them with your own recorded clips**
before training a real policy (see `docs/REPLACE_MOTIONS.md`).

## Train

The user runs training. Pick the task/algo and override any field on the CLI:

```bash
python scripts/train.py task=HOPEPingPong algo=ppo headless=true

# common overrides
python scripts/train.py task=HOPEPingPong num_envs=2048 max_iterations=20000 seed=1 \
    motion_file=hope_training/motions/preprocessed/hope_forehand.npz \
    motion_file_2=hope_training/motions/preprocessed/hope_backhand.npz
```

Checkpoints are written locally to `logs/rsl_rl/hope_pingpong/<timestamp>/` (a periodic checkpoint
every `save_interval` iterations and a final one). Resume with `checkpoint_path=<...>/model_<N>.pt`.
Tune training by editing `cfg/task/HOPEPingPong.yaml` (env / motion / overrides) and
`cfg/algo/ppo.yaml` (PPO). Launch from the repository root so the relative motion paths resolve.

## Play a checkpoint

```bash
python scripts/play.py task=HOPEPingPong num_envs=4 \
    checkpoint=logs/rsl_rl/hope_pingpong/<run>/model_<iter>.pt
```

## Export the deployable policy

```bash
python scripts/export_onnx.py --checkpoint logs/rsl_rl/hope_pingpong/<run>/model_<iter>.pt
```

Writes `hope_pingpong.onnx` (observation[1, 111] -> raw_action[1, 31], single output) and
`policy_manifest.json` (contract name, dims, control rate, joint order, `observation_normalization:
none`, and the ActionAdapter config path) to `<run>/exported/`.

## Evaluate (success_rate only)

```bash
# in Isaac (runs the torch policy):
python scripts/evaluate.py --checkpoint logs/rsl_rl/hope_pingpong/<run>/model_<iter>.pt --num-envs 256

# MuJoCo sim-to-sim (runs the exported ONNX; needs `mujoco` + `onnxruntime`):
python scripts/mujoco_eval_onnx.py --onnx logs/rsl_rl/hope_pingpong/<run>/exported/hope_pingpong.onnx
```

Both print only `{"success_rate": <float>}`. There is no threshold, best-checkpoint selection, or
exit-code change — the number is descriptive.

## Tests

Pure-Python unit tests (no Isaac / torch needed):

```bash
python tests/test_policy_contract.py        # obs 111 / action 31 / manifest schema
python tests/test_success_metric.py         # no-spin return-success logic
python tests/test_racket_command_msg.py     # RacketCommand.msg field ABI
python tests/test_table_tennis_geometry.py  # ITTF table geometry
```

## Code structure

- `scripts/` — `train.py`, `play.py`, `export_onnx.py`, `evaluate.py`, `mujoco_eval_onnx.py`,
  `prepare_a3_isaac_asset.py`.
- `cfg/` — Hydra configs: `train.yaml` / `play.yaml`, `algo/ppo.yaml`, `base/*`, `task/HOPEPingPong.yaml`.
- `source/whole_body_tracking/whole_body_tracking/`
  - `tasks/tracking/` — the HOPE PingPong task, actor observation contract, and MDP terms.
  - `tasks/table_tennis/` — the no-spin ball / ITTF table world and its geometry.
  - `robots/` — the Agibot A3 articulation configuration.
  - `utils/` — `exporter.py` (ONNX + manifest), `success_metric.py` (the `success_rate` core),
    `my_on_policy_runner.py` / `ppo_cfg.py` (rsl_rl glue).

## License

Apache-2.0. Copyright Intelligent Racing Inc. (dba Hitch Interactive). See `LICENSE`.
