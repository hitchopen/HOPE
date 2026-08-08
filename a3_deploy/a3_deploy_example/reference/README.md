# Clean-room reference runner (`a3_deploy_onnx_ref_pingpong`)

A from-scratch Python implementation of the public HOPE deploy contract
(the 110-D `hitter_pure` actor observation).
It exists to document the contract **executably** and to run the exported policy
against the shipped MuJoCo sim. It contains none of the vendor runner's source,
tuned constants, or gates.

## Install & run

```bash
pip install -r requirements.txt
export PYTHONPATH="$PWD"          # or use ../scripts/run_pingpong_sim.sh
python -m a3_deploy_onnx_ref_pingpong \
    --config ../config/hope_pingpong_runtime.yaml \
    --onnx /path/to/policy.onnx \
    --view --realtime
```

Flags: `--backend {mujoco,aimrt}`, `--onnx`, `--model-xml`, `--view`, `--realtime`,
`--duration N` / `--max-ticks N`, `--idle` (no command feed, robot just holds).

## Module layout

| Module | Responsibility |
| --- | --- |
| `joint_order.py` | The 31-DOF Agibot A3 joint order (the single order used everywhere). |
| `quaternion.py` | `(w,x,y,z)` quaternion helpers (projected gravity, base forward). |
| `observation.py` | `build_observation(...) -> float32[110]` — the exact `hitter_pure` layout. |
| `action_adapter.py` | Shared ActionAdapter: `q_des = default_q + raw*scale`, then clamp. |
| `racket_command.py` | `RacketCommand` + command sources (queue seam, example feed). |
| `lifecycle.py` | `ready -> swing -> follow-through -> recovery` state machine. |
| `onnx_policy.py` | onnxruntime actor wrapper `obs[1,110] -> raw_action[1,31]`. |
| `sim_bridge.py` | `MujocoDirectBridge` (default) + `AimrtSimBridge` (seam). |
| `config.py` | Runtime config loader. |
| `runner.py` | The 50 Hz control loop. |
| `__main__.py` | CLI entrypoint. |

## The 110-D observation (`hitter_pure`)

| slice | term | dim |
| --- | --- | --- |
| `[0:3]` | `base_ang_vel` (pelvis body frame) | 3 |
| `[3:34]` | `joint_pos` (`q - default_q`) | 31 |
| `[34:65]` | `joint_vel` | 31 |
| `[65:96]` | `actions` — previous APPLIED action (head columns zeroed) | 31 |
| `[96:99]` | `projected_gravity` (base frame) | 3 |
| `[99:101]` | `base_forward_xy` (world xy unit vector) | 2 |
| `[101:103]` | `base_target_delta_xy` — target base minus current base, world xy | 2 |
| `[103:106]` | `racket_target_rel_base` (world) | 3 |
| `[106:109]` | `racket_target_vel_w` | 3 |
| `[109:110]` | `time_to_strike` | 1 |

No observation normalization; no swing-side slot (deploy infers forehand/backhand
outside the policy). In this sim harness the base target is the fixed station
captured at spawn, so `base_target_delta_xy` is the in-place recentring feedback;
on hardware the delta is 0 on mocap dropout.

## Per-tick control loop (`runner.py`)

1. read robot state from the sim bridge;
2. poll the latest `RacketCommand`; advance the swing lifecycle;
3. assemble the 110-D observation (raw, no normalization);
4. run the ONNX actor → `raw_action[31]`;
5. zero the passive head columns (idx 3, 4) to form the **applied action** and feed
   that back as the next `last_action` (matching training's zeroed feedback);
6. map the applied action → 31 joint targets via the shared ActionAdapter (holding the
   passive neck at its default);
7. write the targets and step the sim.

No gates, failure checks, rejections, reference playback, or state resets between
tasks — a single continuous 110-D path. `task_id`/`task_revision` semantics: a new
(strictly increasing) `task_id` engages exactly one swing and locks the swing sign;
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
path: it subscribes the planner's flat command topic (default
`/racket/command_flat`, a `std_msgs/Float64MultiArray` with a schema tag at
element `[0]`; reliable QoS) on a background rclpy executor and feeds the 50 Hz
loop through the same `QueueRacketCommandSource` mailbox the other sources use.
Because the wire type is core `std_msgs`, **no `hope_msgs` build or rosidl
typesupport overlay is needed** — a sourced ROS 2 environment is enough:

```bash
# needs only a sourced ROS 2 env (rclpy + std_msgs)
python -m a3_deploy_onnx_ref_pingpong --planner --view --realtime
```

Wire schemas (`parse_flat_racket_command` accepts either; `valid == 0` packets are
skipped; extra transport/audit fields are ignored):

- **schema 1** (>= 11 doubles): `[0]=1`, `[1]=valid`, `[2]=swing_sign` (+1 FH / -1 BH),
  `[3..5]=pos_w`, `[6..8]=vel_w`, `[9]=time_to_strike`, `[10]=strike_time`;
- **schema 2** (19 doubles): the same head plus `[11]=frame_code`,
  `[14]=command_seq`, `[15]=flight_id`, `[16]=revision_id` (mapped onto the
  runner's `task_id`/`task_revision`) and producer/estimator audit fields.

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
