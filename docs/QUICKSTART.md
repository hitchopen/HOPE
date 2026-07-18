# Quickstart

This walks the full loop: install → train → export → evaluate → run in simulation. It assumes a
Linux workstation with an NVIDIA GPU for training, and a shell at the repository root.

The pieces have different dependencies, so install only what you need for the step you are on:

| Step | Needs |
|------|-------|
| Train / play / Isaac eval | Isaac Sim + Isaac Lab (with `rsl_rl`), Python 3.10, CUDA GPU |
| Export ONNX | the training install (torch + onnx) |
| MuJoCo eval / reference runner | `mujoco`, `onnxruntime`, `numpy` (no GPU needed) |
| Planner | ROS 2 (rclpy), `numpy`, `pyyaml` |

## 1. Install the training extension

Follow the [Isaac Lab install guide](https://isaac-sim.github.io/IsaacLab/), then, in the Isaac Lab
Python environment:

```bash
cd hope_training/whole_body_tracking
python -m pip install -e source/whole_body_tracking
python -m pip install hydra-core omegaconf     # used by the Hydra entry points
```

## 2. Prepare the A3 robot asset

The starter ships the Agibot-provided A3 ping-pong URDF package under
`agibot/URDF/A3T2.5-URDF-std-pingpang/` (vendor material, no OSS license — see
[`A3_ASSETS.md`](../A3_ASSETS.md)); the asset-prep step uses it by default:

```bash
python hope_training/whole_body_tracking/scripts/prepare_a3_isaac_asset.py --force
```

To use your own vendor-supplied copy instead, place it under `a3_deploy/URDF/`
(see [`a3_deploy/URDF/README.md`](../a3_deploy/URDF/README.md)) and add
`--source-root a3_deploy/URDF/<your_a3_package>`.

## 3. Train the unified policy

> The sample motions are **reference-only placeholders**. Replace them with real forehand/backhand
> clips ([REPLACE_MOTIONS.md](REPLACE_MOTIONS.md)) before training a policy you intend to deploy.

```bash
cd hope_training/whole_body_tracking
python scripts/train.py task=HOPEPingPong algo=ppo headless=true
```

Checkpoints are written locally under `logs/rsl_rl/hope_pingpong/<run>/`. See
[TRAIN_POLICY.md](TRAIN_POLICY.md) for overrides, the reward terms, and the continuous-rally design.

## 4. Export the deployable policy

```bash
python scripts/export_onnx.py --checkpoint logs/rsl_rl/hope_pingpong/<run>/model_<iter>.pt
```

Produces `hope_pingpong.onnx` (`observation[1,111] -> raw_action[1,31]`) and `policy_manifest.json`
under `<run>/exported/`. See [POLICY_INTERFACE.md](POLICY_INTERFACE.md).

## 5. Evaluate — `success_rate`

```bash
# MuJoCo sim-to-sim on the exported ONNX (actual simulated ball physics):
python scripts/mujoco_eval_onnx.py --onnx logs/rsl_rl/hope_pingpong/<run>/exported/hope_pingpong.onnx
```

Prints only `{"success_rate": <float>}`. (An in-Isaac estimate is also available via
`scripts/evaluate.py --checkpoint <...>`; see [TRAIN_POLICY.md](TRAIN_POLICY.md#evaluation).)

## 6. Run in simulation (continuous rally)

Copy the exported policy to the runner and drive the MuJoCo simulation:

```bash
cp logs/rsl_rl/hope_pingpong/<run>/exported/hope_pingpong.onnx \
   a3_deploy/a3_deploy_example/models/
cd a3_deploy/a3_deploy_example
bash scripts/run_pingpong_sim.sh
```

To close the loop with the planner (ROS 2), build the workspace, launch the planner + a synthetic
or real mocap source, then start the runner in `--planner` mode so it consumes the planner's
`/racket/command`:

```bash
cd hope_ws && colcon build && source install/setup.bash
ros2 launch hope_bringup hope_bringup.launch.py use_fake_ball:=true   # mocap-free smoke test
# (real mocap: hope_bringup.launch.py mocap_server:=<host> ball_pose_topic:=/vrpn_mocap/<tracker>/pose_id_0
#  — the pose_to_posearray adapter turns the per-tracker PoseStamped into the planner's /poses)

# in another terminal (same ROS env sourced):
cd a3_deploy/a3_deploy_example/reference
python -m a3_deploy_onnx_ref_pingpong --planner --view --realtime
```

The runner consumes `RacketCommand`, selects forehand/backhand from `swing_side`, and runs the
`ready → swing → follow-through → recovery` lifecycle continuously without resetting robot state
between balls. See [RUN_ON_AGIBOT.md](RUN_ON_AGIBOT.md) and [PLANNER_INTERFACE.md](PLANNER_INTERFACE.md).

## Software smoke tests (no GPU)

```bash
cd hope_training/whole_body_tracking && python -m pytest tests/ -q
```
