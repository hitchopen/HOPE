# Running on the Agibot A3

The deploy side of HOPE lives under [`a3_deploy/`](../a3_deploy).
**`a3_deploy/`** is a revised fork of the official AgiBot A3 deploy stack. It ships:

1. the **native C++ runner `a3_deploy_onnx_ref_pingpong`** (sources under
   `a3_deploy/a3_deploy_example/src/a3/a3_deploy_onnx_ref/`) — the hardware control path, used
   both against the bundled MuJoCo simulation and on the real robot, with its CMake project
   (`CMakeLists.txt` + `cmake/`), `setup_a3_env.sh`, and `docker/` cross-build images for the
   robot's MDU; and
2. the **Python reference harness** (`a3_deploy_example/reference/`, launched by
   `scripts/run_pingpong_sim.sh`) that exercises the identical 110-D contract in plain MuJoCo —
   including `--planner` mode against the live ROS 2 planner — before any hardware session.

Nothing under `a3_deploy/` executes real-robot control on your behalf — hardware sessions are
run deliberately by an operator, only after the closed-loop rehearsal below
passes. A separate, self-contained
[MuJoCo-to-A3 serve application](../apps/a3_mujoco_serve/README.md) uses the
installed high-level motion-control stack. Its exact PR #18 reference artifact
was fully tested, executable, and safe on A3; newly generated motions require their own
qualification.

## What you need to supply

| Item | Why | Where it goes |
|------|-----|---------------|
| A policy bundle | `model_21800` is already published under `a3_deploy/a3_deploy_example/models/model_21800/policy/`; train/export another only if needed | the Python harness selects model_21800 by default; the C++ runner accepts its directory through `--policy-dir` — see [MODEL_21800](MODEL_21800.md) |
| The A3 URDF/meshes (`A3T2.5-URDF-std-pingpang`) | shipped under `agibot/URDF/` (Agibot-provided vendor material, **no OSS license** — see `A3_ASSETS.md`); or supply your own copy under `a3_deploy/URDF/` (see its `README.md`) | used as-is by the asset-prep step |
| The AgiBot vendor deploy payload | required for the **real robot** path only (~1.7 GB, vendor-gated, not in git) | `vendor_assets/agibot/a3_deploy_example_full/` (obtain it from Agibot; it is never committed) |
| Agibot vendor environment on the MDU | required only for the high-level arm serve application | see the [self-contained application](../apps/a3_mujoco_serve/README.md) |

The MuJoCo simulation ships with a runnable `a3_pingpong` scene, so the **simulation path needs
no extra assets**.

## Build the C++ runner

On a new workstation, create the Ubuntu 24.04/ROS 2 Jazzy `hope` container
through [`DISTROBOX_SETUP.md`](DISTROBOX_SETUP.md). Keep this native build out
of the Isaac `grasping` environment and out of the Ubuntu 26.04 host Python.

```bash
distrobox enter hope
source /opt/ros/jazzy/setup.bash

cd "$HOME/workspace/HOPE/a3_deploy/a3_deploy_example"
source setup_a3_env.sh            # ROS 2 env + public ONNX Runtime and Unitree SDK2
cmake -S . -B build
cmake --build build --target a3_deploy_onnx_ref_pingpong -j4
```

`cmake/` provides the AimRT fetch/patch modules. For the robot's MDU, use the
`docker/` cross-build images (rockchip / thor). The vendor's own build flow remains
documented under `agibot/code_deployment/`.

## Simulation path (runnable)

The runner drives the AgiBot MuJoCo simulation over the same **iceoryx body-drive** interface as
the real robot: it reads joint state / base orientation / base angular velocity, builds the
110-D `hitter_pure` observation ([POLICY_INTERFACE.md](POLICY_INTERFACE.md)), runs the ONNX
policy, and publishes the 31 joint-position targets to the backend PD loop. Runtime modes are
keyboard-driven (`p` passive, `s` PD-stand, `h` shadow/no-publish, `m` motion).

The quickest closed loop is the **Python reference harness** in plain MuJoCo:

```bash
cd a3_deploy/a3_deploy_example
scripts/run_pingpong_sim.sh --view --realtime            # synthetic serves
# with the ROS 2 planner + fake ball running (hope_ws):
PYTHONPATH=reference python3 -m a3_deploy_onnx_ref_pingpong --planner --view --realtime
```

For the full C++ rehearsal, start the AimRT MuJoCo sim (`a3_deploy/A3_MuJoCo_Sim`), the
built runner in `--planner` mode, and the planner + fake-ball publisher from `hope_ws`.
In `--planner` mode both runners subscribe the planner's flat wire topics
(`/racket/command_flat`, schema 2, 19 doubles including `swing_sign`/`flight_id`/`revision`; and
`/a3/base_pose_flat`, schema 2, 16 doubles) — see
[PLANNER_INTERFACE.md](PLANNER_INTERFACE.md) and
[interfaces/ros_topics.md](interfaces/ros_topics.md). Scripted serve batches can be driven
through the fake-ball publisher's `serves` parameter (a flat N×6 list cycled in order).

The runner executes the full per-strike lifecycle — `ready → swing → follow-through → recovery`
— continuously; robot state and `last_action` are never reset between balls. Forehand/backhand
comes from the planner's `swing_sign` on the wire; the policy itself never observes the side.

## Real-robot path

The same runner, cross-built through the vendor packaging script and its `docker/` image for the
robot's MDU, runs against the vendor body-drive backend instead of the simulator. Supply the
licensed AgiBot payload outside this repository, then build the complete Rockchip package from
the repository root:

```bash
A3_VENDOR_PAYLOAD_ROOT=vendor_assets/agibot/a3_deploy_example_full \
  agibot/code_deployment/a3_deploy_example/scripts/build_a3_deploy_pkg.sh \
  --arch rockchip --jobs 4

agibot/code_deployment/a3_deploy_example/dist/a3_deploy_rockchip/run_a3_pingpong.sh \
  --planner --policy-native --start passive --official-stand
```

The wrapper selects the ping-pong runtime/AimRT configs and the packaged
model_21800 bundle; it starts PASSIVE and does not enter motion on behalf of the
operator. Everything between an exported `policy.onnx` and a robot returning a
ball — machine layout, clock sync, mocap bring-up, planner host, e-stop
discipline, and the order of checks — is your operators' responsibility:
rehearse the identical chain in simulation first and advance to hardware only
on a clean pass.

Vendor hard joint limits, motor protection, communication timeouts, and physical e-stop remain
entirely your robot backend's responsibility. HOPE does not probe, score, certify, or
bypass those mechanisms.

The separate [`apps/a3_mujoco_serve/`](../apps/a3_mujoco_serve/README.md)
workflow implements deterministic MuJoCo planning, DLS IK and high-level
14-arm CSV replay. It does not implement the 110-D observation / 31-D learned
policy contract described in this section.

## Action realization (shared with training)

The policy emits a 31-D raw action; both training and the runner realize it as

```
q_des = default_q + raw_action * action_scale   # then a deterministic joint clamp
```

with the backend PD loop tracking `q_des`. The joint order and contract name travel **with the
export** (embedded ONNX metadata + `policy_manifest.json`), and the loaders fail closed on a
mismatch; for model_21800 both runners read the exported
`models/model_21800/policy/params/deploy.yaml`. See
[POLICY_INTERFACE.md](POLICY_INTERFACE.md) for the full action contract.
