# A3 Isaac Lab Quickstart

This is the shortest public path from a fresh clone to an Isaac Lab smoke run.
It is meant to prove that the A3 asset, task registration, scene, and PPO entry
point are wired correctly. It is not a trained policy quality benchmark.

You do not need to set up ROS, mocap, MuJoCo, WandB, or the preserved reference
documents before running this path. Those materials are useful later, but the
commands below are the required v1 starter route.

## 0. Clone

```bash
git clone https://github.com/hitchopen/HOPE.git
cd HOPE
```

## 1. Isaac Lab Environment

Install NVIDIA Isaac Sim and Isaac Lab in your own GPU environment, then edit
the paths in `hope_training/whole_body_tracking/setup_train_env.sh` or create a
git-ignored local override:

```bash
cp hope_training/whole_body_tracking/setup_train_env.local.example.sh \
   hope_training/whole_body_tracking/setup_train_env.local.sh
```

Set at least:

```bash
export HOPE_ISAAC_PYTHON=/absolute/path/to/isaacsim/python.sh
export HOPE_ISAACLAB_ROOT=/absolute/path/to/IsaacLab
```

Then source the helper in every new training shell:

```bash
cd hope_training/whole_body_tracking
source setup_train_env.sh
hope_isaac_py -c "import whole_body_tracking.tasks; print('HOPE tasks import ok')"
```

WandB is optional. The smoke path below uses a local motion file and
TensorBoard.

## 2. Prepare the A3 Isaac Asset

The source A3 ping-pong URDF lives in `agibot/URDF/A3T2.5-URDF-std-pingpang`.
Isaac Lab loads a prepared copy under the Python package asset directory.

From the repository root:

```bash
python3 scripts/prepare_a3_isaac_asset.py --force
python3 scripts/prepare_a3_isaac_asset.py --check
```

The check verifies that
`hope_training/whole_body_tracking/source/whole_body_tracking/whole_body_tracking/assets/agibot_a3/urdf/model.urdf`
exists and no stale `package://.../meshes` references remain.

## 3. Run the Table-Tennis Scene Smoke Test

```bash
cd hope_training/whole_body_tracking
hope_isaac_py scripts/play_table_tennis.py --headless --steps 300
```

This launches the A3/table/net/ball scene without training.

## 4. Generate a Tiny Local Motion Clip

The public starter branch does not require a private motion registry. Generate a
small stand-still motion clip from the prepared A3 articulation:

```bash
hope_isaac_py scripts/create_smoke_motion.py \
  --headless \
  --frames 120 \
  --output sample_motions/agibot_a3_smoke_stand.npz
```

This clip is for smoke training only. Replace it with your own retargeted
forehand/backhand motion for meaningful learning.

## 5. Smoke Train with TensorBoard

```bash
hope_isaac_py scripts/train.py task=TrackingFlat algo=ppo \
  headless=true logger=tensorboard \
  motion_file=sample_motions/agibot_a3_smoke_stand.npz \
  num_envs=32 max_iterations=3
```

Expected result: Isaac Lab starts, the motion file is loaded from the local path,
PPO runs a few iterations, and logs appear under
`hope_training/whole_body_tracking/logs/rsl_rl/`.

## Optional WandB Registry Path

If your team uses a WandB motion registry, you can omit `motion_file=...` and
pass `registry_name=<org>/wandb-registry-motions/<motion_name>`. That path is
optional and is not required for the public smoke run.
