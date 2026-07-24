# Clean-room reference runner (`a3_deploy_onnx_ref_pingpong`)

A from-scratch Python implementation of the public HOPE deploy contract.
It exists to document the contract **executably** and to run the exported policy
against the shipped MuJoCo sim. It contains none of the vendor runner's source,
tuned constants, or gates.

## Install & run

```bash
pip install -r requirements.txt
export PYTHONPATH="$PWD"          # or use ../scripts/run_pingpong_sim.sh
python -m a3_deploy_onnx_ref_pingpong \
    --config ../config/hope_pingpong_runtime.yaml \
    --onnx /path/to/hope_pingpong.onnx \
    --view --realtime
```

Flags: `--backend {mujoco,aimrt}`, `--onnx`, `--model-xml`, `--view`, `--realtime`,
`--duration N` / `--max-ticks N`, `--idle` (no command feed, robot just holds).

## Module layout

| Module | Responsibility |
| --- | --- |
| `joint_order.py` | The 31-DOF Agibot A3 joint order (the single order used everywhere). |
| `quaternion.py` | `(w,x,y,z)` quaternion helpers (projected gravity, base forward). |
| `observation.py` | `build_observation(...) -> float32[111]` — the exact 111-D layout. |
| `action_adapter.py` | Shared ActionAdapter: `q_des = default_q + raw*scale`, then clamp. |
| `racket_command.py` | `RacketCommand` + command sources (queue seam, example feed). |
| `lifecycle.py` | `ready -> swing -> follow-through -> recovery` state machine. |
| `onnx_policy.py` | onnxruntime actor wrapper `obs[1,111] -> raw_action[1,31]`. |
| `sim_bridge.py` | `MujocoDirectBridge` (default) + `AimrtSimBridge` (seam). |
| `config.py` | Runtime config loader. |
| `runner.py` | The 50 Hz control loop. |
| `__main__.py` | CLI entrypoint. |

## Per-tick control loop (`runner.py`)

1. read robot state from the sim bridge;
2. poll the latest `RacketCommand`; advance the swing lifecycle;
3. assemble the 111-D observation (raw, no normalization);
4. run the ONNX actor → `raw_action[31]`;
5. zero the passive head columns (idx 3, 4) to form the **applied action** and feed
   that back as the next `last_action` (matching training's zeroed feedback);
6. map the applied action → 31 joint targets via the shared ActionAdapter (holding the
   passive neck at its default);
7. write the targets and step the sim.

No gates, failure checks, rejections, reference playback, or state resets between
tasks — a single continuous 111-D path. `task_id`/`task_revision` semantics: a new
(strictly increasing) `task_id` engages exactly one swing and locks `swing_side`;
a higher `task_revision` refines the target/time-to-strike **before** contact only.

## How it drives MuJoCo

`MujocoDirectBridge` (the default, fully runnable path) loads the same
`a3_pingpong` MJCF that the AimRT MuJoCo sim wraps and steps MuJoCo in-process:

- joint state (`q`, `qd`) is read from the mapped `qpos`/`qvel` addresses;
- base orientation comes from the pelvis free-joint quaternion and base angular
  velocity from the pelvis gyro sensor;
- joint-position targets are realized with an explicit PD law
  (`tau = kp*(q_des - q) - kd*qd`) written to the model's torque actuators — the
  same implicit-PD shape the AimRT backend uses — and clamped to each actuator's
  control range;
- one 50 Hz control tick advances several physics substeps (20 at the model's
  1 kHz timestep), recomputing the PD each substep.

The PD gains are **example** simulation gains from the runtime config, not vendor
deploy gains.

## Live planner input (`--planner`)

`RosRacketCommandSource` (`ros_command_source.py`) is the wired planner → runner
path: it subscribes the planner's `hope_msgs/RacketCommand` topic (default
`/racket/command`) on a background rclpy executor and feeds the 50 Hz loop through
the same `QueueRacketCommandSource` mailbox the other sources use:

```bash
# needs a sourced ROS 2 env + built hope_msgs (cd hope_ws && colcon build && source install/setup.bash)
python -m a3_deploy_onnx_ref_pingpong --planner --view --realtime
```

The included `ExampleCommandFeed` (the default source) is a planner-less
demonstration feed so the sim is runnable without a planner; it is **not** part of
the deploy contract and is not a scripted swing (the swing trajectory is always
produced by the learned policy). `--idle` runs with no commands at all.

## Integration seams (explicitly not wired)

- **`AimrtSimBridge`** — driving the live AimRT MuJoCo sim *process* over its
  `/body_drive/*` channels. Wiring it needs the AimRT Python runtime plus the
  `joint_msgs` typesupport (a vendor build). It raises `NotImplementedError` with
  the exact channel/message mapping rather than faking state. Use
  `MujocoDirectBridge` to actually run.

## Notes

- Observation normalization is `none` (raw observation) by contract.
- `head_yaw` / `head_pitch` are passive at deploy (held at their default) but still
  occupy their action columns, so every vector stays length 31.
- The sample motion clips used in training are reference examples only, not
  performance-tuned; replace them with your own.
