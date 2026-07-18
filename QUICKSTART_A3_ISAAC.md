# A3 Isaac Lab Quickstart

This is the shortest public path from a fresh clone to the full HOPE loop on the Agibot A3:
prepare the robot asset, smoke-test the table-tennis scene, train the unified forehand/backhand
policy in Isaac Lab, export it to ONNX, evaluate `success_rate`, and run the exported policy in
the reference simulation. Each step lists only what it needs — you can stop after any step.

| Step | Needs |
|------|-------|
| Train / play / Isaac eval | Isaac Sim + Isaac Lab (with `rsl_rl`), Python 3.10, CUDA GPU |
| Export ONNX | the training install (torch + onnx) |
| MuJoCo eval / reference runner | `mujoco`, `onnxruntime`, `numpy` (no GPU needed) |
| Planner (full loop) | ROS 2 (rclpy), `numpy`, `pyyaml` |

For the same loop with more depth per step, see [`docs/QUICKSTART.md`](docs/QUICKSTART.md); the
document index is [`REFERENCE_DOCS.md`](REFERENCE_DOCS.md).

## 0. Clone

```bash
git clone https://github.com/hitchopen/HOPE.git
cd HOPE
```

## 1. Install the training extension

Follow the [Isaac Lab install guide](https://isaac-sim.github.io/IsaacLab/), then, in the Isaac
Lab Python environment:

```bash
cd hope_training/whole_body_tracking
python -m pip install -e source/whole_body_tracking
python -m pip install hydra-core omegaconf     # used by the Hydra entry points
```

## 2. Prepare the A3 Isaac Asset

The starter ships the Agibot-provided A3 ping-pong URDF package under
`agibot/URDF/A3T2.5-URDF-std-pingpang/` (vendor material, no OSS license — see
[`A3_ASSETS.md`](A3_ASSETS.md)); the asset-prep step uses it by default. From the repository
root:

```bash
python3 hope_training/whole_body_tracking/scripts/prepare_a3_isaac_asset.py --force
python3 hope_training/whole_body_tracking/scripts/prepare_a3_isaac_asset.py --check
```

This copies the meshes and rewrites `package://.../meshes/*.STL` references so Isaac Lab can
load the model without ROS package resolution; the prepared asset lands under the training
package's git-ignored `assets/agibot_a3/` directory. `--check` verifies the prepared
`urdf/model.urdf` exists, no stale `package://` mesh references remain, and every referenced
mesh resolves.

To use your own vendor-supplied copy instead, place it under `a3_deploy/URDF/` (see
[`a3_deploy/URDF/README.md`](a3_deploy/URDF/README.md)) and add
`--source-root a3_deploy/URDF/<your_a3_package>`.

## 3. Set Up the Training Shell (`isaac_py`)

`setup_train_env.sh` puts the working-tree package source first on `PYTHONPATH` and defines an
`isaac_py` launcher that runs your Isaac Sim Python with that path. Source it (do not execute)
in every training shell:

```bash
cd hope_training/whole_body_tracking
source setup_train_env.sh
```

It probes for a usable Isaac Sim Python on its own. If the probe picks the wrong interpreter,
or your Isaac Lab is a source checkout, create the git-ignored local override:

```bash
cp setup_train_env.local.example.sh setup_train_env.local.sh
# edit: ISAAC_PYTHON=/absolute/path/to/isaacsim/python.sh
#       ISAACLAB_ROOT=/absolute/path/to/IsaacLab   (source checkouts only)
```

## 4. Smoke Checks

From `hope_training/whole_body_tracking/`, confirm the task package imports and the scene runs:

```bash
isaac_py -c "import whole_body_tracking.tasks; print('HOPE tasks import ok')"
isaac_py scripts/play_table_tennis.py --headless --steps 300
```

The scene smoke builds the full court (floor, table, net, ball, Agibot A3) and steps the physics
with no policy and no checkpoint — use it to verify the asset, ball flight, and layout before
training. Drop `--headless` for a window; other options: `--num_envs 9`, `--fix_base`,
`--disable_aero`.

Pure-Python unit tests need no GPU or Isaac install:

```bash
python -m pytest tests/ -q
```

## 5. Train

> The clips shipped under `hope_training/motions/preprocessed/` (`hope_forehand.npz` +
> `hope_backhand.npz`) are **reference-only placeholders** so imports and shape checks pass.
> Replace them with your own recorded forehand/backhand clips
> ([`docs/REPLACE_MOTIONS.md`](docs/REPLACE_MOTIONS.md)) before training a policy you intend to
> deploy.

```bash
isaac_py scripts/train.py task=HOPEPingPong algo=ppo headless=true

# common overrides
isaac_py scripts/train.py task=HOPEPingPong num_envs=2048 max_iterations=20000 seed=1 \
    motion_file=hope_training/motions/preprocessed/hope_forehand.npz \
    motion_file_2=hope_training/motions/preprocessed/hope_backhand.npz
```

This trains the single Gym task `HOPE-PingPong-AgibotA3-v0` (111-D observation, 31-D action,
50 Hz, continuous rallies — no teleport between swings). Checkpoints are written locally to
`logs/rsl_rl/hope_pingpong/<timestamp>/` (a periodic checkpoint every `save_interval`
iterations and a final one); resume with `checkpoint_path=<...>/model_<N>.pt`. Tune via
`cfg/task/HOPEPingPong.yaml` and `cfg/algo/ppo.yaml`. The only metric is `success_rate`,
reported by the evaluators in step 7 — see [`docs/TRAIN_POLICY.md`](docs/TRAIN_POLICY.md) for
the task design and reward terms.

## 6. Export the Deployable Policy

```bash
isaac_py scripts/export_onnx.py --checkpoint logs/rsl_rl/hope_pingpong/<run>/model_<iter>.pt
```

Writes `hope_pingpong.onnx` (single output, `observation[1,111] -> raw_action[1,31]`, no
observation normalization) and `policy_manifest.json` to `<run>/exported/`. Export hard-checks
the articulation's joint order against the canonical deploy order
(`hope_training/config/joint_order_agibot_a3.yaml`) and refuses to export on a mismatch, so a
mis-prepared asset cannot silently produce a policy with scrambled action columns. See
[`docs/POLICY_INTERFACE.md`](docs/POLICY_INTERFACE.md) for the full contract.

## 7. Evaluate — `success_rate`

`scripts/mujoco_eval_onnx.py` is the **authoritative** evaluator: it runs the exported ONNX
against a real MuJoCo ball that physically bounces off the racket, table, and net. It needs
only `mujoco`, `onnxruntime`, and `numpy` (no GPU, no Isaac):

```bash
python scripts/mujoco_eval_onnx.py --onnx logs/rsl_rl/hope_pingpong/<run>/exported/hope_pingpong.onnx
```

By default it evaluates a **continuous rally** (robot/policy state persist across serves and
all four adjacent forehand/backhand transitions are exercised); pass `--eval-mode independent`
for the isolated per-serve variant. A fast in-Isaac **estimate** (analytic no-spin ball model,
no MuJoCo) is also available:

```bash
isaac_py scripts/evaluate.py --checkpoint logs/rsl_rl/hope_pingpong/<run>/model_<iter>.pt --num-envs 256
```

Both print only `{"success_rate": <float>}` — trust the MuJoCo number.

## 8. Run the Full Loop (Reference Runner + Planner)

Copy the exported policy to the reference deploy runner and drive the MuJoCo simulation:

```bash
cp logs/rsl_rl/hope_pingpong/<run>/exported/hope_pingpong.onnx \
   a3_deploy/a3_deploy_example/models/
cd a3_deploy/a3_deploy_example
bash scripts/run_pingpong_sim.sh --view --realtime
```

By default the runner uses a built-in demo command feed. To close the loop with the planner,
build the ROS 2 workspace, launch the planner with a fake ball (mocap-free smoke test) or real
mocap, then start the runner in `--planner` mode so it consumes the planner's
`hope_msgs/RacketCommand` on `/racket/command`:

```bash
# Terminal 1: planner + ball source
cd hope_ws && colcon build && source install/setup.bash
ros2 launch hope_bringup hope_bringup.launch.py use_fake_ball:=true
# real mocap instead:
#   ros2 launch hope_bringup hope_bringup.launch.py mocap_server:=<host> \
#       ball_pose_topic:=/vrpn_mocap/ball/pose_id_0

# Terminal 2 (same ROS env sourced): the runner consuming /racket/command
cd a3_deploy/a3_deploy_example/reference
python -m a3_deploy_onnx_ref_pingpong --planner --view --realtime
```

The runner selects forehand/backhand from `swing_side` and executes the
`ready → swing → follow-through → recovery` lifecycle continuously, without resetting robot
state between balls. See [`docs/RUN_ON_AGIBOT.md`](docs/RUN_ON_AGIBOT.md) for the deploy stack
(including the real-robot integration seam) and
[`docs/PLANNER_INTERFACE.md`](docs/PLANNER_INTERFACE.md) for the command contract.
