# Policy IO

Compact summary of the policy input/output contract. The authoritative document
is [POLICY_INTERFACE.md](../POLICY_INTERFACE.md) — training, the exported ONNX,
and any deployment backend must agree with it.

| Property | Value |
|----------|-------|
| Observation | `float32[111]` |
| Action | `float32[31]` raw joint-position residual |
| Control rate | 50 Hz |
| Observation normalization | **none** (raw observation fed directly) |
| ONNX signature | `observation[1, 111] -> raw_action[1, 31]` |
| Joint order | 31 DOF, see [joint_order.md](joint_order.md) |
| Export | `hope_pingpong.onnx` + `policy_manifest.json` (contract name `hope_pingpong`), via `scripts/export_onnx.py` |

## Observation (111 dims)

Assembled in this exact order every tick (full per-slice table in
[POLICY_INTERFACE.md](../POLICY_INTERFACE.md#observation-111-dims)):

| Slice | Terms |
|-------|-------|
| `[0:96]` | Proprioception: `base_ang_vel` (3), `joint_pos` (31), `joint_vel` (31), `last_action` (31). |
| `[96:103]` | `projected_gravity` (3), `base_forward_xy` (2), `fixed_station_error_xy` (2). |
| `[103:111]` | Racket target: `racket_target_rel_base` (3), `racket_target_vel_w` (3), `time_to_strike` (1), `swing_side` (1, forehand `+1` / backhand `-1`). |

The policy has no reference-motion stream, no ball state, and no spin inputs.

## Action (31 dims) and applied-action head zeroing

Each tick the actor emits `raw_action[31]`. The two passive head columns
(indices 3, 4) are **zeroed** to form the *applied action*, which is:

1. fed back as next tick's `last_action` (so those two dims are always 0 —
   exactly as training zeroes them), and
2. passed through the **ActionAdapter** to produce 31 joint-position targets
   (the head is held at its default angle):

```text
q_des = default_q + raw_action * action_scale
q_des = clamp(q_des, q_min, q_max)     # deterministic numeric transform
```

The adapter constants live in **one** shared config,
[`a3_deploy/a3_deploy_example/config/action_adapter.yaml`](../../a3_deploy/a3_deploy_example/config/action_adapter.yaml),
read by both training and the reference deploy runner — edit it in one place.
The shipped values are neutral examples; tune them for your robot.

## Continuous operation

The policy runs continuous rallies: between incoming balls the robot state and
`last_action` are never reset — no teleport, no history clear. Recovery between
strikes is in-place recentring and balance only.
