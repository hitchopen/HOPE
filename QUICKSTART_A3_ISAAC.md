# A3 Isaac Lab Quickstart

This is the shortest public path from a fresh clone to the full HOPE loop on the Agibot A3:
prepare the robot asset, smoke-test the table-tennis scene, train the deploy-grade rally policy
in Isaac Lab, evaluate it deterministically, export it to ONNX, verify it in MuJoCo sim-to-sim,
and rehearse the deploy stack closed-loop. Each step lists only what it needs — you can stop
after any step.

| Step | Needs |
|------|-------|
| Train / play / Isaac eval | the pinned GPU-enabled `grasping` Distrobox from `docs/DISTROBOX_SETUP.md` |
| Export ONNX | the training install (torch + onnx) |
| MuJoCo sim-to-sim eval | `mujoco`, `onnxruntime`, `numpy` (no GPU needed) |
| Deploy build + closed-loop rehearsal | C++ toolchain (CMake), ROS 2, `onnxruntime` (bundled under `a3_deploy/`) |
| Planner (full loop) | ROS 2 (rclpy), `numpy`, `pyyaml` |

For the same loop with more depth per step, see [`docs/TRAIN_POLICY.md`](docs/TRAIN_POLICY.md)
(training/evaluation/export) and [`docs/RUN_ON_AGIBOT.md`](docs/RUN_ON_AGIBOT.md) (deploy); the
document index is [`REFERENCE_DOCS.md`](REFERENCE_DOCS.md).

On a new workstation, start with
[`docs/DISTROBOX_SETUP.md`](docs/DISTROBOX_SETUP.md). It records the exact
working machine, pins the complete Isaac/PyTorch image, and separately creates
the ROS 2 `hope` environment. The generic upstream Isaac Lab installer is an
alternate migration path, not the reproducible HOPE setup.

## 0. Set up the workstation and clone

On a fresh Ubuntu host, complete sections 1 and 2 of
[`docs/DISTROBOX_SETUP.md`](docs/DISTROBOX_SETUP.md). Those steps install and
verify the NVIDIA driver, Podman CDI, Distrobox and Git LFS; clone HOPE under
`$HOME`; and create the pinned `grasping` environment.

If the workstation is already prepared and only the clone is missing:

```bash
mkdir -p "$HOME/workspace"
git clone https://github.com/hitchopen/HOPE.git "$HOME/workspace/HOPE"
cd "$HOME/workspace/HOPE"
git lfs install
git lfs pull
```

## 1. Enter and verify the training environment

Enter the pinned container. Its interactive shell may activate Conda `base`;
deactivate it and source the repository launcher before any Isaac command:

```bash
distrobox enter grasping

if [[ -n "${CONDA_PREFIX:-}" ]]; then
  conda deactivate
fi

cd "$HOME/workspace/HOPE/hope_training/whole_body_tracking"
source setup_train_env.sh
nvidia-smi -L
cat /workspace/isaacsim/VERSION
hope_isaac_py -c "import hydra, omegaconf, torch, importlib.util; assert torch.cuda.is_available(); assert importlib.util.find_spec('isaaclab'); assert importlib.util.find_spec('whole_body_tracking'); print('HOPE training environment OK')"
```

The verified launcher resolves to `/workspace/isaacsim/python.sh` and adds the
working-tree package before installed packages on `PYTHONPATH`; no editable
`pip install` is required for the pinned environment. Do not replace
`hope_isaac_py` with bare `python` or call `import whole_body_tracking` before
Kit starts. Isaac entrypoints import task modules after `AppLauncher` starts.

If you intend to play the published `model_21800.pt`, install Git LFS on the
host before cloning, or materialize the checkpoint after cloning. Run these
commands from the repository root:

```bash
cd "$HOME/workspace/HOPE"
git lfs install
git lfs pull --include=hope_training/whole_body_tracking/checkpoints/model_21800.pt
test "$(stat -c %s hope_training/whole_body_tracking/checkpoints/model_21800.pt)" -gt 1000000
```

Without this step the checkout contains a small text pointer instead of the
checkpoint, so `play.py` cannot load it. This is dependency materialization,
not a policy-admission or SHA gate.

## 2. Prepare the A3 Isaac Asset

The repository ships the Agibot-provided A3 ping-pong URDF package under
`agibot/URDF/A3T2.5-URDF-std-pingpang/` (vendor material, no OSS license — see
[`A3_ASSETS.md`](A3_ASSETS.md)); the asset-prep step uses it by default. From
`hope_training/whole_body_tracking/`:

```bash
hope_isaac_py scripts/prepare_a3_isaac_asset.py --force
hope_isaac_py scripts/prepare_a3_isaac_asset.py --check
```

This copies the meshes and rewrites `package://.../meshes/*.STL` references so Isaac Lab can
load the model without ROS package resolution; the prepared asset lands under the training
package's git-ignored `assets/agibot_a3/` directory. `--check` verifies the prepared
`urdf/model.urdf` exists, no stale `package://` mesh references remain, and every referenced
mesh resolves.

To use your own vendor-supplied copy instead, place it under `a3_deploy/URDF/` (see
[`a3_deploy/URDF/README.md`](a3_deploy/URDF/README.md)) and add
`--source-root a3_deploy/URDF/<your_a3_package>`.

## 3. Understand the training launcher (`hope_isaac_py`)

`setup_train_env.sh` puts the working-tree package source first on `PYTHONPATH` and defines a
`hope_isaac_py` launcher that runs your Isaac Sim Python with that path. Source it (do not
execute) after entering `grasping` in every training shell:

```bash
distrobox enter grasping
if [[ -n "${CONDA_PREFIX:-}" ]]; then conda deactivate; fi
cd "$HOME/workspace/HOPE/hope_training/whole_body_tracking"
source setup_train_env.sh
```

The pinned image is auto-detected. Only a deliberately custom Isaac install
should need the git-ignored local override
`setup_train_env.local.sh` (auto-sourced) next to it:

```bash
# setup_train_env.local.sh
export ISAAC_PYTHON=/absolute/path/to/isaacsim/python.sh
export ISAACLAB_ROOT=/absolute/path/to/IsaacLab   # source checkouts only
```

Do not run this step from ordinary host, ROS, or Conda Python. A successful
source prints `training env ready` and the selected Isaac paths; otherwise
`hope_isaac_py` exits immediately with a setup error. The complete version
probe and custom-install boundary are in `docs/DISTROBOX_SETUP.md`.

The public setup does not configure external logging. New runs remain local; the
published `model_21800` checkpoint is documented in
[`docs/MODEL_21800.md`](docs/MODEL_21800.md).

## 4. Smoke Checks

From `hope_training/whole_body_tracking/`, confirm the installed package is
discoverable and the scene runs. Task modules are imported by the entrypoints
after `AppLauncher` starts Isaac Kit:

```bash
hope_isaac_py -c "import importlib.util; assert importlib.util.find_spec('whole_body_tracking'); print('HOPE package found')"
hope_isaac_py scripts/play_table_tennis.py --headless --steps 300
```

The scene smoke builds the full court (floor, table, net, ball, Agibot A3) and steps the physics
with no policy and no checkpoint — use it to verify the asset, ball flight, and layout before
training. Drop `--headless` for a window; other options: `--num_envs 9`, `--fix_base`,
`--enable_aero`.

Run the package tests through the same pinned interpreter:

```bash
hope_isaac_py -m pytest tests/ -q
```

Before training, you can run the published deployment actor in plain MuJoCo from
the repository root:

```bash
python3 -m venv .venv && source .venv/bin/activate
python -m pip install -r a3_deploy/a3_deploy_example/reference/requirements.txt
a3_deploy/a3_deploy_example/scripts/run_pingpong_sim.sh --duration 10
```

## 5. Train

The repository ships the complete validated Build forehand/backhand pair under the stable public
filenames `hope_forehand.npz` and `hope_backhand.npz`. They are the default inputs used by the
recipe; no private-version suffix is part of the public interface. To train on a different motion
pair, follow [`docs/REPLACE_MOTIONS.md`](docs/REPLACE_MOTIONS.md) and override both paths.

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

To open the published checkpoint directly:

```bash
test "$(stat -c %s checkpoints/model_21800.pt)" -gt 1000000
hope_isaac_py scripts/play.py task=HOPEPingPong algo=ppo num_envs=4 \
    checkpoint=checkpoints/model_21800.pt
```

Use `headless=true num_steps=2` for a bounded non-interactive smoke; the default
opens the Isaac viewer and runs until the window closes.

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

Omit `--onnx` to evaluate the published `model_21800`. The committed
[`Gate 3 MuJoCo video`](docs/assets/model_21800_gate3_mujoco.mp4) comes from the
build_1 planner + policy-native runner closed loop, not this standalone evaluator.

Isaac metrics (step 6), this MuJoCo sim-to-sim check, and the closed-loop planner rehearsal
(step 9) together form the staged evaluation story — each layer is a gate before the next
([`docs/TRAIN_POLICY.md`](docs/TRAIN_POLICY.md#evaluation)).

## 9. Run the Full Loop (C++ Runner + Planner)

Build the ROS 2 workspace, then rehearse the C++ Planner chain closed-loop with
the Python MuJoCo reference Runner harness:

```bash
cd hope_ws && colcon build && source install/setup.bash
ros2 launch hope_bringup hope_bringup.launch.py use_fake_ball:=true

# in another terminal (same ROS env sourced):
cd a3_deploy/a3_deploy_example
PYTHONPATH=reference python3 -m a3_deploy_onnx_ref_pingpong --planner --view --realtime
```

The rehearsal wires a fake ball through `hope_ball_flight_packetizer` into the
production **C++ Planner** (`hope_ws/src/hope_planner_cpp`), which publishes
`/racket/command_flat`; `hope_bringup` owns the independent base-pose relay;
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
