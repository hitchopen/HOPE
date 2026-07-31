# Running on the Agibot A3

The deploy side of HOPE lives under [`a3_deploy/`](../a3_deploy).
**`a3_deploy/`** is a revised fork of the official AgiBot A3 deploy stack. The
proprietary full vendor policy runner is **not** part of this clean-room area;
instead you get:

1. a **clean-room Python reference runner** that implements the public policy contract and runs
   against the bundled MuJoCo simulation, and
2. an **integration seam** for wiring the same contract into your own licensed AgiBot vendor
   deploy package on the real robot.

Nothing under `a3_deploy/` executes real-robot control on your behalf. The
separate Agibot-provided reference tree under `agibot/code_deployment/` also
contains an optional, experimental
[fixed vendor-arm serve](../agibot/code_deployment/a3_deploy_example/SERVE_SCRIPT_DEPLOY.md).
That route uses Agibot's installed high-level motion-control stack and is not
part of the clean-room policy runner or a certified hardware path.

## What you need to supply

| Item | Why | Where it goes |
|------|-----|---------------|
| A trained policy `hope_pingpong.onnx` | the actor network | `a3_deploy/a3_deploy_example/models/` (see [export](TRAIN_POLICY.md)) |
| The A3 URDF/meshes (`A3T2.5-URDF-std-pingpang`) | shipped with the starter under `agibot/URDF/` (Agibot-provided vendor material, **no OSS license** — see `A3_ASSETS.md`); or supply your own copy under `a3_deploy/URDF/` (see its `README.md`) | used as-is by the asset-prep step |
| Your own AgiBot vendor deploy package | required for the **real robot** path only | referenced by `run_pingpong_real.sh` |
| Agibot vendor environment on the MDU | required only for the optional fixed vendor-arm serve | see the [isolated serve package](../agibot/code_deployment/a3_deploy_example/SERVE_SCRIPT_DEPLOY.md) |

The MuJoCo simulation ships with a runnable `a3_pingpong` model, so the **simulation path needs
no extra assets**.

## Simulation path (runnable)

The reference runner drives MuJoCo in-process via `MujocoDirectBridge`: it loads the shipped
`a3_pingpong.xml`, reads joint state / base orientation / base angular velocity, builds the
111-D observation ([POLICY_INTERFACE.md](POLICY_INTERFACE.md)), runs the ONNX policy, and
realizes the 31 joint-position targets with an explicit PD controller.

```bash
cd a3_deploy/a3_deploy_example
# place your exported hope_pingpong.onnx under models/ first
bash scripts/run_pingpong_sim.sh
```

The runner executes the full per-strike lifecycle — `ready → swing → follow-through → recovery`
— one swing per `task_id`, with `swing_side` locked for the strike and pre-contact
`task_revision` updates applied to the target. Robot state and `last_action` are never reset
between balls.

Racket commands come from one of three sources: the built-in demo feed (default, standalone
smoke test), `--idle` (no commands), or `--planner` — the **full planner → runner path**,
which subscribes the live planner's `hope_msgs/RacketCommand` over ROS 2:

```bash
# Terminal 1: mocap (or fake ball) + planner
cd hope_ws && colcon build && source install/setup.bash
ros2 launch hope_bringup hope_bringup.launch.py use_fake_ball:=true

# Terminal 2: the reference runner consuming /racket/command
cd a3_deploy/a3_deploy_example/reference
python -m a3_deploy_onnx_ref_pingpong --planner --view --realtime
```

`--planner` needs a sourced ROS 2 environment with the built `hope_msgs` package (see
[PLANNER_INTERFACE.md](PLANNER_INTERFACE.md) for the message contract; the bridge lives in
`reference/a3_deploy_onnx_ref_pingpong/ros_command_source.py`).

> The bundled MuJoCo model is a robot-only scene (no ball/table physics in MJCF yet), so the
> simulation path validates policy execution and joint control. Full rally physics and
> `success_rate` evaluation run in the Isaac training environment via
> `hope_training/whole_body_tracking/scripts/mujoco_eval_onnx.py` and `evaluate.py`.

## Real-robot path (you wire this)

`scripts/run_pingpong_real.sh` is a documented, non-executing template. It describes how to
hand the 111-D observation / 31-D action contract to your own AgiBot vendor backend. In the
reference runner this is the `AimrtSimBridge` seam, which documents the exact
`/body_drive/*` `joint_msgs` / `sensor_msgs/Imu` wiring required (it raises
`NotImplementedError` until you connect your vendor AimRT typesupport — it never fakes success).

Vendor hard joint limits, motor protection, communication timeouts, and physical e-stop remain
entirely your robot backend's responsibility. HOPE does not probe, score, certify, or
bypass those mechanisms.

The optional fixed vendor-arm serve is a separate integration example in the
Agibot-provided tree. It does not implement the 111-D observation / 31-D
policy contract described in this section.

## ActionAdapter (shared with training)

Both the reference runner and training read the same
`a3_deploy/a3_deploy_example/config/action_adapter.yaml`:

```
q_des = default_q + raw_action * action_scale   # then a deterministic joint clamp
```

The shipped `default_q` (neutral stand), `action_scale` (uniform 0.25), and clamp limits are
**example values — tune them for your robot**. Editing this one file keeps training and deploy
in sync. See [POLICY_INTERFACE.md](POLICY_INTERFACE.md) for the full action contract.

## Runtime config

`a3_deploy/a3_deploy_example/config/hope_pingpong_runtime.yaml` holds the clean 111-D
runtime settings (control rate 50 Hz, observation normalization none, ONNX path, joint-order
file, ActionAdapter path). It contains no model version numbers, recipes, or internal
references.
