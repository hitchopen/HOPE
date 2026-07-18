# Policy interface

The HOPE policy is a single feed-forward actor network shared by forehand and
backhand. It runs at **50 Hz**. This document is the authoritative contract for its inputs,
outputs, and the joint ordering — training, the exported ONNX, and any deployment backend
must all agree with it.

## Summary

| Property | Value |
|----------|-------|
| Observation | `float32[111]` |
| Action | `float32[31]` (raw joint-position residual) |
| Control rate | 50 Hz |
| Observation normalization | none (raw observation fed directly) |
| ONNX signature | `observation[1, 111] -> raw_action[1, 31]` |
| Joint order | 31 DOF, see [joint order](#joint-order) |

The policy is exported by `hope_training/whole_body_tracking/scripts/export_onnx.py` as
`hope_pingpong.onnx` plus a `policy_manifest.json` describing this contract.

## Observation (111 dims)

Assembled in this exact order every tick:

| Slice | Term | Dim | Frame / units | Meaning |
|-------|------|----:|---------------|---------|
| `[0:3]`     | `base_ang_vel`           | 3  | pelvis body, rad/s | Pelvis angular velocity (IMU). |
| `[3:34]`    | `joint_pos`              | 31 | rad | `q - default_q`, joint order below. |
| `[34:65]`   | `joint_vel`              | 31 | rad/s | Encoder joint velocities. |
| `[65:96]`   | `last_action`            | 31 | raw | The action **applied** on the previous tick: `raw_action` with the passive head columns (idx 3, 4) zeroed. |
| `[96:99]`   | `projected_gravity`      | 3  | base frame, unit | Gravity direction in the base frame (IMU). |
| `[99:101]`  | `base_forward_xy`        | 2  | world xy, unit | Base forward unit vector projected to world XY (from IMU yaw). |
| `[101:103]` | `fixed_station_error_xy` | 2  | world xy, m | Fixed station position (a startup constant) minus current base XY. |
| `[103:106]` | `racket_target_rel_base` | 3  | world, m | Target racket position minus base position. |
| `[106:109]` | `racket_target_vel_w`    | 3  | world, m/s | Target racket velocity. |
| `[109:110]` | `time_to_strike`         | 1  | s | Time remaining until the strike. |
| `[110:111]` | `swing_side`             | 1  | ±1 | Forehand `+1`, backhand `-1` (locked for the whole strike). |

Total: `3 + 31 + 31 + 31 + 3 + 2 + 2 + 3 + 3 + 1 + 1 = 111`.

Notes:
- `fixed_station_error_xy` is **not** constant zero. The station target is fixed at startup,
  but the robot base drifts over a rally; these two dims give the policy in-place recentring
  feedback. This is balance feedback, not station planning — the station never moves.
- `swing_side` is a formal input, chosen once per incoming ball by the planner and held for
  the whole strike. The planner also carries it in `RacketCommand`
  (see [PLANNER_INTERFACE.md](PLANNER_INTERFACE.md)).
- Target vectors are expressed in the world frame relative to the base; the policy has no
  reference-motion stream, no ball state, and no spin inputs.

## Action (31 dims)

Each tick the actor emits `raw_action[31]` in the joint order below. The two passive head
columns (idx 3, 4) are zeroed to form the **applied action**, which is:
1. fed back as next tick's `last_action` (so those two columns are always 0 — exactly as
   training zeroes them in its applied-action feedback), and
2. passed through the **ActionAdapter** to produce 31 joint-position targets (the head is
   held at its default angle).

### ActionAdapter

The public example adapter is a joint-position residual:

```
q_des = default_q + raw_action * action_scale
q_des = clamp(q_des, q_min, q_max)      # deterministic numeric transform, not a gate
```

The example constants (`default_q`, `action_scale`, clamp limits) live in a single shared
config, `a3_deploy/a3_deploy_example/config/action_adapter.yaml`, read by **both**
training and the reference runner so you edit them in one place. The shipped values are a
neutral starting point — **tune them for your robot**. The deterministic clamp is a numeric
transform; it never emits a failure/rejection status.

Vendor hard limits, motor protection, communication timeouts, and physical e-stop are the
robot backend's responsibility; the policy code does not probe, score, certify, or bypass
them.

## Joint order

31 controllable DOF, from `hope_training/config/joint_order_agibot_a3.yaml`:

```
 0 waist_yaw_joint            11 left_wrist_yaw_joint       22 left_knee_joint
 1 waist_roll_joint           12 right_shoulder_pitch_joint 23 left_ankle_pitch_joint
 2 waist_pitch_joint          13 right_shoulder_roll_joint  24 left_ankle_roll_joint
 3 head_yaw_joint    (passive) 14 right_shoulder_yaw_joint  25 right_hip_pitch_joint
 4 head_pitch_joint  (passive) 15 right_elbow_joint         26 right_hip_roll_joint
 5 left_shoulder_pitch_joint  16 right_wrist_roll_joint     27 right_hip_yaw_joint
 6 left_shoulder_roll_joint   17 right_wrist_pitch_joint    28 right_knee_joint
 7 left_shoulder_yaw_joint    18 right_wrist_yaw_joint      29 right_ankle_pitch_joint
 8 left_elbow_joint           19 left_hip_pitch_joint       30 right_ankle_roll_joint
 9 left_wrist_roll_joint      20 left_hip_roll_joint
10 left_wrist_pitch_joint     21 left_hip_yaw_joint
```

`head_yaw_joint` and `head_pitch_joint` (indices 3–4) are held at their defaults on the real
robot but still occupy action columns. The racket is mounted on the right wrist.

## Continuous operation

The policy is designed for continuous rallies. Between incoming balls the robot state, joint
state, and `last_action` are **not** reset — no teleport, no history clear, no return to a
default pose. The lifecycle per strike is:

```
ready -> swing -> follow-through -> recovery -> ready -> (next ball)
```

Recovery is in-place recentring and balance only.

## policy_manifest.json

Emitted alongside the ONNX. It records the contract name (`hope_pingpong`), observation and
action dimensions, control rate, joint order, observation normalization (`none`), and the
ActionAdapter config path. It does **not** contain version numbers, training recipes, reward
definitions, checkpoint lineage, or metrics.
