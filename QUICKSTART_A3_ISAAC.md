# A3 Isaac Lab Quickstart

This is the shortest public path from a fresh clone to the full HOPE loop on the Agibot A3:
prepare the robot asset, smoke-test the table-tennis scene, train the deploy-grade rally policy
in Isaac Lab, evaluate it deterministically, export it to ONNX, verify it in MuJoCo sim-to-sim,
and rehearse the deploy stack closed-loop. Each step lists only what it needs — you can stop
after any step.

| Step | Needs |
|------|-------|
| Train / play / Isaac eval | Isaac Sim + Isaac Lab (with `rsl_rl`), Python 3.10, CUDA GPU |
| Export ONNX | the training install (torch + onnx) |
| MuJoCo sim-to-sim eval | `mujoco`, `onnxruntime`, `numpy` (no GPU needed) |
| Deploy build + closed-loop rehearsal | C++ toolchain (CMake), ROS 2, `onnxruntime` (bundled under `a3_deploy/`) |
| Planner (full loop) | ROS 2 (rclpy), `numpy`, `pyyaml` |

For the same loop with more depth per step, see [`docs/TRAIN_POLICY.md`](docs/TRAIN_POLICY.md)
(training/evaluation/export) and [`docs/RUN_ON_AGIBOT.md`](docs/RUN_ON_AGIBOT.md) (deploy); the
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

The repository ships the Agibot-provided A3 ping-pong URDF package under
`agibot/URDF/A3T2.5-URDF-std-pingpang/` (vendor material, no OSS license — see
[`A3_ASSETS.md`](A3_ASSETS.md)); the asset-prep step uses it by default. From
`hope_training/whole_body_tracking/`:

```bash
python3 scripts/prepare_a3_isaac_asset.py --force
python3 scripts/prepare_a3_isaac_asset.py --check
```

This copies the meshes and rewrites `package://.../meshes/*.STL` references so Isaac Lab can
load the model without ROS package resolution; the prepared asset lands under the training
package's git-ignored `assets/agibot_a3/` directory. `--check` verifies the prepared
`urdf/model.urdf` exists, no stale `package://` mesh references remain, and every referenced
mesh resolves.

To use your own vendor-supplied copy instead, place it under `a3_deploy/URDF/` (see
[`a3_deploy/URDF/README.md`](a3_deploy/URDF/README.md)) and add
`--source-root a3_deploy/URDF/<your_a3_package>`.

## 3. Set Up the Training Shell (`hope_isaac_py`)

`setup_train_env.sh` puts the working-tree package source first on `PYTHONPATH` and defines a
`hope_isaac_py` launcher that runs your Isaac Sim Python with that path. Source it (do not
execute) in every training shell:

```bash
cd hope_training/whole_body_tracking
source setup_train_env.sh
```

It probes for a usable Isaac Sim Python on its own. If the probe picks the wrong interpreter,
or your Isaac Lab is a source checkout, create the git-ignored local override
`setup_train_env.local.sh` (auto-sourced) next to it:

```bash
# setup_train_env.local.sh
export ISAAC_PYTHON=/absolute/path/to/isaacsim/python.sh
export ISAACLAB_ROOT=/absolute/path/to/IsaacLab   # source checkouts only
```

The public setup does not configure external logging. New runs remain local; the
published `model_21800` checkpoint is documented in
[`docs/MODEL_21800.md`](docs/MODEL_21800.md).

## 4. Smoke Checks

From `hope_training/whole_body_tracking/`, confirm the task package imports and the scene runs:

```bash
hope_isaac_py -c "import whole_body_tracking.tasks; print('HOPE tasks import ok')"
hope_isaac_py scripts/play_table_tennis.py --headless --steps 300
```

The scene smoke builds the full court (floor, table, net, ball, Agibot A3) and steps the physics
with no policy and no checkpoint — use it to verify the asset, ball flight, and layout before
training. Drop `--headless` for a window; other options: `--num_envs 9`, `--fix_base`,
`--enable_aero`.

Pure-Python unit tests need no GPU or Isaac install:

```bash
python -m pytest tests/ -q
```

Before training, you can run the published deployment actor in plain MuJoCo from
the repository root:

```bash
python3 -m venv .venv-mujoco && source .venv-mujoco/bin/activate
python -m pip install -r a3_deploy/a3_deploy_example/reference/requirements.txt
a3_deploy/a3_deploy_example/scripts/run_pingpong_sim.sh --duration 10
```

## 5. Train

> The clips shipped under `hope_training/motions/preprocessed/` (`hope_forehand.npz` +
> `hope_backhand.npz`) are **schema-valid placeholders** so imports and shape checks pass.
> Replace them with your own recorded forehand/backhand clips
> ([`docs/REPLACE_MOTIONS.md`](docs/REPLACE_MOTIONS.md)) before training a policy you intend to
> deploy — the proven internal line trained on real `*_v12fix` clips that are not shipped.

```bash
hope_isaac_py scripts/train.py task=HOPEPingPong algo=ppo headless=true \
    motion_file=../motions/preprocessed/hope_forehand.npz \
    motion_file_2=../motions/preprocessed/hope_backhand.npz

# common overrides
hope_isaac_py scripts/train.py task=HOPEPingPong algo=ppo headless=true \
    num_envs=4096 max_iterations=20000 seed=1 \
    motion_file=../motions/preprocessed/hope_forehand.npz \
    motion_file_2=../motions/preprocessed/hope_backhand.npz
```

`HitterPingPong` (gym id `HOPE-HitterPingPong-AgibotA3-v0`) is the deploy-grade recipe
validated on real A3 hardware: a **110-D `hitter_pure` observation**, 31-D raw action, 50 Hz,
continuous rallies — no teleport between swings. It is the only shipped task; its full recipe
lives in `cfg/task/HOPEPingPong.yaml` — see
[`docs/TRAIN_POLICY.md`](docs/TRAIN_POLICY.md). Checkpoints are written locally to
`logs/rsl_rl/<experiment_name>/<timestamp>/`; resume with `checkpoint_path=<...>/model_<N>.pt`.
Motion clips are selected through the local `motion_file=` / `motion_file_2=` overrides.

## 6. Evaluate in Isaac

```bash
hope_isaac_py scripts/evaluate.py \
    --checkpoint logs/rsl_rl/<run>/model_<iter>.pt \
    --motion-file ../motions/preprocessed/hope_forehand.npz \
    --motion-file-2 ../motions/preprocessed/hope_backhand.npz
```

This rolls the policy out across many parallel environments and reports the in-Isaac
`success_rate` estimate; the authoritative physical number comes from the MuJoCo sim-to-sim
check below.

## 7. Export the Deployable Policy

```bash
cd hope_training/whole_body_tracking
hope_isaac_py scripts/export_onnx.py --checkpoint logs/rsl_rl/<run>/model_<iter>.pt
```

Writes the single-output actor ONNX (110-D observation in, 31-D raw action out, no observation
normalization) and its deploy manifest to `<run>/exported/`. The exported metadata is what the
downstream loaders validate — a mis-matched or mis-prepared export cannot silently run. See
[`docs/POLICY_INTERFACE.md`](docs/POLICY_INTERFACE.md) for the full contract.

## 8. Evaluate in MuJoCo — sim-to-sim

`scripts/mujoco_eval_onnx.py` runs the exported ONNX against a real MuJoCo ball that physically
bounces off the racket, table, and net. It needs only `mujoco`, `onnxruntime`, and `numpy`
(no GPU, no Isaac):

```bash
python3 scripts/mujoco_eval_onnx.py --onnx logs/rsl_rl/<run>/exported/policy.onnx
```

Omit `--onnx` to evaluate the published `model_21800`. Add
`--video-out /tmp/model_21800.mp4` for an H.264 recording; the exact committed
example is [`docs/assets/model_21800_mujoco.mp4`](docs/assets/model_21800_mujoco.mp4).

Isaac metrics (step 6), this MuJoCo sim-to-sim check, and the closed-loop planner rehearsal
(step 9) together form the staged evaluation story — each layer is a gate before the next
([`docs/TRAIN_POLICY.md`](docs/TRAIN_POLICY.md#evaluation)).

## 9. Run the Full Loop (C++ Runner + Planner)

Build the ROS 2 planner workspace, then rehearse the chain closed-loop with the Python
reference harness in plain MuJoCo:

```bash
cd hope_ws && colcon build && source install/setup.bash
ros2 launch hope_bringup hope_bringup.launch.py use_fake_ball:=true

# in another terminal (same ROS env sourced):
cd a3_deploy/a3_deploy_example
PYTHONPATH=reference python3 -m a3_deploy_onnx_ref_pingpong --planner --view --realtime
```

The rehearsal wires a fake ball into the **real** Python planner
(`hope_ws/src/hope_planner`), which publishes `/racket/command_flat` and `/a3/base_pose_flat`;
the runner consumes the flats in `--planner` mode — the same wire the native C++ runner
(`a3_pingpong`, sources under `a3_deploy/a3_deploy_example/src/a3/a3_deploy_onnx_ref/`)
subscribes on hardware. Building and driving the C++ runner is covered in
[docs/RUN_ON_AGIBOT.md](docs/RUN_ON_AGIBOT.md).

To drive the planner from real mocap instead of the fake ball, additionally build one mocap
workspace (`NatNet2ROS2/` for OptiTrack, `VRPN2ROS2/` for VRPN — see
[`docs/OPTITRACK.md`](docs/OPTITRACK.md) and [`hope_ws/README.md`](hope_ws/README.md)):

```bash
cd hope_ws && source install/setup.bash
ros2 launch hope_bringup hope_bringup.launch.py use_fake_ball:=true   # mocap-free smoke test
```

The runner executes the `ready → swing → follow-through → recovery` lifecycle continuously,
without resetting robot state between balls; forehand/backhand is inferred by the planner and
carried on the wire (`swing_sign`) — the policy never observes it. See
[`docs/RUN_ON_AGIBOT.md`](docs/RUN_ON_AGIBOT.md) for the deploy stack (including the
real-robot path) and [`docs/PLANNER_INTERFACE.md`](docs/PLANNER_INTERFACE.md) for the command
contract.
