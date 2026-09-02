# HOPE Agibot A3 whole-body training

This package is the Isaac Lab training and playback extension for the HOPE
Agibot A3 table-tennis policy. It provides one public Hydra task,
`HOPEPingPong`, backed by the Gym environment
`HOPE-HitterPingPong-AgibotA3-v0`.

The task trains one feed-forward policy shared by forehand and backhand:

- actor observation: 110-D `hitter_pure`;
- action: 31-D joint-position command;
- control rate: 50 Hz;
- clip 0: forehand, `hope_forehand.npz`;
- clip 1: backhand, `hope_backhand.npz`;
- local checkpoints and TensorBoard logs only; no W&B registry is required.

The task YAML is the training-recipe source of truth:
[`cfg/task/HOPEPingPong.yaml`](cfg/task/HOPEPingPong.yaml). The public motion
files use stable names without private revision suffixes or SHA admission
checks.

For the full workflow, see
[`QUICKSTART_A3_ISAAC.md`](../../QUICKSTART_A3_ISAAC.md). For the recipe and
evaluation design, see [`docs/TRAIN_POLICY.md`](../../docs/TRAIN_POLICY.md).

## Requirements

The supported reproduction environment is the pinned `grasping` Distrobox in
[`docs/DISTROBOX_SETUP.md`](../../docs/DISTROBOX_SETUP.md). The working-machine
baseline is Isaac Sim 5.1.0, Python 3.11.13, Isaac Lab 0.54.2, PyTorch
2.7.0+cu128, Hydra 1.3.2 and `rsl-rl-lib` 3.1.2 on an NVIDIA GPU. Git LFS is
also required when playing the published `model_21800.pt` checkpoint.

Do not use an arbitrary host or Conda Python for Isaac commands. The included
`setup_train_env.sh` finds common Isaac installations and defines
`hope_isaac_py`. A machine-specific override can be placed in the git-ignored
`setup_train_env.local.sh`:

```bash
export ISAAC_PYTHON=/absolute/path/to/isaacsim/python.sh
export ISAACLAB_ROOT=/absolute/path/to/IsaacLab
```

On a new machine, create the pinned container through
[`docs/DISTROBOX_SETUP.md`](../../docs/DISTROBOX_SETUP.md), then enter it before
sourcing the setup script. Its shell may activate Conda `base`; deactivate it
first. Bare `python` in the image is not the supported Isaac interpreter.

```bash
distrobox enter grasping
if [[ -n "${CONDA_PREFIX:-}" ]]; then conda deactivate; fi
cd "$HOME/workspace/HOPE/hope_training/whole_body_tracking"
source setup_train_env.sh
```

## First-time setup on a new machine

First complete the host and `grasping` sections of
[`docs/DISTROBOX_SETUP.md`](../../docs/DISTROBOX_SETUP.md). From the repository
root, materialize the published checkpoint if you intend to run it in Isaac:

```bash
git lfs install
git lfs pull --include=hope_training/whole_body_tracking/checkpoints/model_21800.pt
test "$(stat -c %s hope_training/whole_body_tracking/checkpoints/model_21800.pt)" -gt 1000000
```

Training from scratch does not require `model_21800.pt`.

Enter `grasping` and initialize the launcher:

```bash
distrobox enter grasping
if [[ -n "${CONDA_PREFIX:-}" ]]; then conda deactivate; fi
cd "$HOME/workspace/HOPE/hope_training/whole_body_tracking"
source setup_train_env.sh
hope_isaac_py -c "import hydra, omegaconf, torch, importlib.util; assert torch.cuda.is_available(); assert importlib.util.find_spec('isaaclab'); assert importlib.util.find_spec('whole_body_tracking'); print('HOPE package path and runtime OK')"
```

`setup_train_env.sh` places the checked-out source first on `PYTHONPATH`, so the
pinned image does not need an editable package install. Directly importing the
task package before Kit starts can fail on `omni.*`; use the `find_spec` probe
above or one of the shipped Isaac entrypoints.

Prepare the bundled racket-equipped Agibot A3 URDF:

```bash
hope_isaac_py scripts/prepare_a3_isaac_asset.py --force
hope_isaac_py scripts/prepare_a3_isaac_asset.py --check
```

The source package is
`../../agibot/URDF/A3T2.5-URDF-std-pingpang/`. The prepared, git-ignored asset
is written to
`source/whole_body_tracking/whole_body_tracking/assets/agibot_a3/`.

Run the regression tests through the pinned interpreter without starting Kit:

```bash
PYTHONDONTWRITEBYTECODE=1 hope_isaac_py -m pytest -q -p no:cacheprovider tests
```

## Train

From this directory, after sourcing `setup_train_env.sh`:

```bash
hope_isaac_py scripts/train.py task=HOPEPingPong algo=ppo headless=true
```

The task defaults already select the published forehand and backhand clips.
Common run overrides are ordinary Hydra arguments:

```bash
hope_isaac_py scripts/train.py task=HOPEPingPong algo=ppo headless=true \
    num_envs=4096 max_iterations=20000 seed=1
```

To use a different motion pair:

```bash
hope_isaac_py scripts/train.py task=HOPEPingPong algo=ppo headless=true \
    motion_file=/absolute/path/to/forehand.npz \
    motion_file_2=/absolute/path/to/backhand.npz
```

The replacement schema is documented in
[`docs/REPLACE_MOTIONS.md`](../../docs/REPLACE_MOTIONS.md). Keep the clip order
fixed: forehand first, backhand second.

Checkpoints and resolved configuration are written under:

```text
logs/rsl_rl/agibot_a3_hitter_pingpong/<timestamp>/
```

Resume a local run with
`checkpoint_path=<run>/model_<iteration>.pt`. No checkpoint is automatically
promoted.

## Play

Open the published checkpoint in Isaac:

```bash
test "$(stat -c %s checkpoints/model_21800.pt)" -gt 1000000
hope_isaac_py scripts/play.py task=HOPEPingPong algo=ppo num_envs=4 \
    checkpoint=checkpoints/model_21800.pt
```

For a bounded headless smoke test:

```bash
hope_isaac_py scripts/play.py task=HOPEPingPong algo=ppo \
    headless=true device=cpu num_envs=1 num_steps=2 \
    checkpoint=checkpoints/model_21800.pt
```

`play.py` loads the supplied checkpoint directly; it does not add SHA,
provenance, or policy-admission gates. Git LFS materialization and a valid Isaac
environment remain required dependencies.

## Evaluate and export

Run the in-Isaac evaluator:

```bash
hope_isaac_py scripts/evaluate.py \
    --checkpoint logs/rsl_rl/agibot_a3_hitter_pingpong/<run>/model_<iteration>.pt \
    --motion-file ../motions/preprocessed/hope_forehand.npz \
    --motion-file-2 ../motions/preprocessed/hope_backhand.npz
```

Export a checkpoint to ONNX:

```bash
hope_isaac_py scripts/export_onnx.py \
    --checkpoint logs/rsl_rl/agibot_a3_hitter_pingpong/<run>/model_<iteration>.pt
```

The exported actor consumes the same 110-D observation and produces the same
31-D action used by the native Runner. Continue with
[`docs/POLICY_INTERFACE.md`](../../docs/POLICY_INTERFACE.md) and
[`docs/RUN_ON_AGIBOT.md`](../../docs/RUN_ON_AGIBOT.md) before deployment.

## Troubleshooting

- `training env NOT ready`: enter the Isaac/Distrobox environment and source
  `setup_train_env.sh` again, or set `ISAAC_PYTHON` and `ISAACLAB_ROOT` in the
  local override file.
- `ModuleNotFoundError: hydra` or an Isaac warning about active Conda: enter
  `grasping`, deactivate Conda, re-source `setup_train_env.sh`, and use
  `hope_isaac_py` instead of bare Python or `python.sh`.
- `ModuleNotFoundError: omni.timeline` during a direct package import: the task
  was imported before Kit startup; use the documented `find_spec` check or a
  shipped entrypoint.
- `torch.load` fails on a tiny checkpoint: run the Git LFS commands above and
  confirm `model_21800.pt` is larger than 1 MB.
- A3 URDF or mesh error: rerun asset preparation with `--force`, followed by
  `--check`.
- Motion file not found: launch from this directory or pass absolute
  `motion_file` and `motion_file_2` paths.

## Upstream basis

This extension derives from the MIT-licensed
[BeyondMimic whole-body tracking project](https://github.com/HybridRobotics/whole_body_tracking).
Its original Unitree G1/W&B workflow is intentionally not repeated here because
those commands are not the HOPE A3 entrypoints. See `LICENCE`, the repository
root `THIRD_PARTY_NOTICES.md`, and the upstream project for attribution and
upstream-specific usage.
