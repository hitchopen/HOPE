# Policy IO

Compact summary of the policy input/output contract. The authoritative document
is [POLICY_INTERFACE.md](../POLICY_INTERFACE.md) — training, the exported ONNX,
and any deployment backend must agree with it.

| Property | Value |
|----------|-------|
| Observation | `float32[110]` — the `hitter_pure` contract (deploy default) |
| Action | `float32[31]` raw joint-position residual |
| Control rate | 50 Hz |
| Observation normalization | **none** (raw observation fed directly) |
| ONNX signature | `observation[1, 110] -> raw_action[1, 31]` |
| Joint order | 31 DOF, see [joint_order.md](joint_order.md) |
| Export | the actor ONNX + its deploy manifest under `<run>/exported/`, via `scripts/export_onnx.py` |

Named contracts (`full` 180 / `deploy_parity` 175 / `hitter_footwork` 177 /
`hitter_pure` 110 / `hitter_pure_v15` 118) are defined in
`tasks/tracking/actor_observation_contract.py`; each task config picks one via
`actor_obs_contract`, and ONNX metadata carries the name so loaders fail closed
on a mismatch. The deploy-grade rally line uses `hitter_pure`.

## Observation (110 dims)

Assembled in this exact order every tick (full per-slice table in
[POLICY_INTERFACE.md](../POLICY_INTERFACE.md#observation-110-dims)):

| Slice | Terms |
|-------|-------|
| `[0:96]` | Proprioception: `base_ang_vel` (3), `joint_pos` (31), `joint_vel` (31), `actions` (31, previous applied action). |
| `[96:103]` | `projected_gravity` (3), `base_forward_xy` (2), `base_target_delta_xy` (2, Δ=0 on mocap dropout). |
| `[103:110]` | Racket target: `racket_target_rel_base` (3), `racket_target_vel_w` (3), `time_to_strike` (1). |

No `swing_side` observation — forehand/backhand is inferred outside the policy
(the wire carries `swing_sign`, the actor never sees it). No reference-motion
stream, no ball state, no spin inputs.

## Action (31 dims)

Each tick the actor emits `raw_action[31]`, mapped to joint-position targets:

```text
q_des = default_q + raw_action * action_scale
q_des = clamp(q_des, official A3 hard limits)   # deterministic numeric transform
```

`default_q` / `action_scale` / limits are recorded in the `policy.deploy.json`
export sidecar; the C++ loader cross-checks them against its
`pp_joint_limits.hpp` table (verified against the official A3 URDF). The two
head columns (indices 3, 4) are passive — the runtime holds the neck at nominal
regardless of the actor output.

## Continuous operation

The policy runs continuous rallies: between incoming balls the robot state and
action history are never reset — no teleport, no history clear. Recovery between
strikes is in-place recentring and balance only.
