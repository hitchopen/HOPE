# Policy interface

The HOPE policy is a single feed-forward actor network shared by forehand and
backhand. It runs at **50 Hz**. This document is the authoritative contract for its inputs,
outputs, and the joint ordering — training, the exported ONNX, and any deployment backend
must all agree with it.

## Summary

| Property | Value |
|----------|-------|
| Observation | `float32[110]` — the `hitter_pure` contract (deploy default) |
| Action | `float32[31]` (raw joint-position residual) |
| Control rate | 50 Hz |
| Observation normalization | none (raw observation fed directly) |
| ONNX signature | `observation[1, 110] -> raw_action[1, 31]` |
| Joint order | 31 DOF, see [joint order](#joint-order) |

Named observation contracts are defined in one place —
`hope_training/whole_body_tracking/source/whole_body_tracking/whole_body_tracking/tasks/tracking/actor_observation_contract.py`.
Each task config selects one via its `actor_obs_contract` key, and the exported ONNX
carries the contract name in its metadata so loaders can fail closed on a mismatch:

| Contract | Dim | Used by |
|----------|----:|---------|
| `hitter_pure` | 110 | **The deploy-grade rally line** — the shipped `HitterPingPong` task (gym id `HOPE-HitterPingPong-AgibotA3-v0`). |
| `full` | 180 | Historical full-observation line (no shipped task). |
| `deploy_parity` | 175 | Historical sim-to-real line. |
| `hitter_footwork` | 177 | Historical separate base/racket experiment. |
| `hitter_pure_v15` | 118 | Historical executed-q_des experiment. |

The rest of this document describes the deploy contract, `hitter_pure`.

## Observation (110 dims)

Assembled in this exact order every tick. Structure follows HITTER
(arXiv:2508.21043) Table I — proprioception + goal only — sized for the A3's 31 joints:

| Slice | Term | Dim | Frame / units | Meaning |
|-------|------|----:|---------------|---------|
| `[0:3]`     | `base_ang_vel`           | 3  | pelvis body, rad/s | Pelvis angular velocity (IMU/mocap-anchored per the task's contract). |
| `[3:34]`    | `joint_pos`              | 31 | rad | `q - default_q`, joint order below. |
| `[34:65]`   | `joint_vel`              | 31 | rad/s | Encoder joint velocities. |
| `[65:96]`   | `actions`                | 31 | raw | Previous policy action as applied by the runtime. |
| `[96:99]`   | `projected_gravity`      | 3  | base frame, unit | Gravity direction in the base frame. |
| `[99:101]`  | `base_forward_xy`        | 2  | world xy, unit | Base forward unit vector `e_base,x` projected to world XY. |
| `[101:103]` | `base_target_delta_xy`   | 2  | world xy, m | Target base position minus current base position (Δ=0 on mocap dropout). |
| `[103:106]` | `racket_target_rel_base` | 3  | world, m | Target racket position minus base position. |
| `[106:109]` | `racket_target_vel_w`    | 3  | world, m/s | Target racket velocity. |
| `[109:110]` | `time_to_strike`         | 1  | s | Time remaining until the strike. |

Total: `3 + 31 + 31 + 31 + 3 + 2 + 2 + 3 + 3 + 1 = 110`.

Notes:
- There is **no `swing_side` observation**. Forehand/backhand is inferred outside the policy
  (HITTER §V-B-3): the planner publishes the side on the wire
  (`swing_sign` in `/racket/command_flat`, see
  [PLANNER_INTERFACE.md](PLANNER_INTERFACE.md)) and the runner's engage machine uses it, but
  the actor never sees it.
- `base_target_delta_xy` gives the policy in-place recentring feedback toward its station;
  on mocap dropout the runtime holds Δ=0 so a stale pose can never command a chase.
- The 62-D reference joint stream from the paper is **critic-only** — it exists during
  training and never enters the actor observation or the deploy wire.
- The policy has no ball state and no spin inputs.

## Action (31 dims)

Each tick the actor emits `raw_action[31]` in the joint order below, mapped to
joint-position targets:

```
q_des = default_q + raw_action * action_scale
q_des = clamp(q_des, official A3 hard limits)   # deterministic numeric transform, not a gate
```

- `default_q`, `action_scale`, and the joint-limit table are recorded in the export
  sidecar `policy.deploy.json` (written next to the ONNX) — training and deployment read
  the same values. The C++ loader cross-checks them against its own
  `pp_joint_limits.hpp` table, which is verified against the official A3 URDF limits.
- The two head columns (idx 3, 4) are **passive**: the runtime holds the neck at its
  nominal pose regardless of the actor's output for those columns.
- Every pre-clamp `q_des` column is measured against the official A3 hard joint range
  during training (a range-normalized barrier reward term),
  so the runtime clamp is a numeric formality, not a behavior patch.

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
state, and action history are **not** reset — no teleport, no history clear, no return to a
default pose. The lifecycle per strike is:

```
ready -> swing -> follow-through -> recovery -> ready -> (next ball)
```

Recovery is in-place recentring and balance only.

## Export, metadata, and qualification

The export path is
`hope_training/whole_body_tracking/scripts/export_onnx.py --checkpoint RUN_DIR/model_XXXX.pt`.
It writes the single-output actor ONNX and its deploy manifest to `RUN_DIR/exported/`.

The export carries the contract (dims, control rate, joint order, observation
normalization = none) as metadata; the Python MuJoCo evaluator and the C++ hardware
loader validate these fields exactly and **fail closed** on a missing or mismatched value.
Exports are first proven in simulation ([TRAIN_POLICY.md](TRAIN_POLICY.md#evaluation));
hardware acceptance additionally goes through the closed-loop rehearsal and gate sweeps
described in [RUN_ON_AGIBOT.md](RUN_ON_AGIBOT.md).
