# Table-Tennis Match Scene (Isaac Lab)

A clean, modular Isaac Lab task that simulates a **table-tennis scene** — floor, table, net (+ posts), a
dynamic 40 mm ball, and the **Agibot A3** humanoid — with realistic ball flight, bounce, and racket/table
contact. The table/net are invisible cuboid colliders overlaid with a **realistic USD mesh** for visuals
(see [Table visuals](#table-visuals-usd-overlay)). Built as a manager-based `ManagerBasedRLEnv` so a
returner policy can be layered on; this package owns the **physics + scene**.

Gym id: **`HOPE-TableTennis-AgibotA3-v0`**

## Coordinate frame (world frame, used everywhere)

The simulation world frame is the shared world frame (ROS 2 REP-103), identical to the planner / mocap
reference:

| Axis | Direction | Range on the table |
|------|-----------|--------------------|
| **X** | toward Player Two (P2), along the table length | `0 → +2.74 m` |
| **Y** | left, from P1's perspective, along the table width | `0 → −1.525 m` |
| **Z** | up; **z = 0 is the table surface** | floor at `z = −0.76 m` |

Origin = the **near-side left corner of the table surface** (P1 perspective). Each parallel environment
is an independent court whose local origin coincides with this origin, so an asset's environment-local
position *is* its world-frame position. All landmarks (net center `(1.37, −0.7625, 0)`, P1/P2 half
centers, floor at `−0.76`) come from [`geometry.py`](geometry.py), the single source of truth for scene
geometry.

## Physics model (no spin)

* **Gravity + all rigid-body contacts** (ball↔table / net / floor / racket) are handled natively by
  PhysX, with per-surface contact materials defined in `geometry.BounceMaterials`. Materials use
  multiplicative combine with a **neutral ball material** (restitution/friction = 1.0), so each surface's
  restitution is the effective ball↔surface value directly: table ≈ `0.92`, floor `0.40`, and a
  deliberately low net `0.10` so the ball dies on a net touch (match play).
* **Aerodynamic drag** is the one thing PhysX cannot model for a 40 mm ball. It is a **no-spin quadratic
  drag** `a_drag = −k·|v|·v`, added every physics substep by `TableTennisEnv` via a physics-step callback
  (see [`ball.py`](ball.py)). There is **no spin, angular velocity, or Magnus/lift term** anywhere in the
  model — the ball state is fully described by position and linear velocity.
* Physics runs at **400 Hz** (`sim.dt = 0.0025`), control at **100 Hz** (`decimation = 4`). The high
  physics rate + PhysX CCD keep the small, fast ball from tunnelling through the thin racket blade / net.

### Physics config — single source of truth

Every physical constant (ball mass/radius, drag coefficient, gravity, table/net geometry, and bounce
materials) is read from the repo-level **`configs/ball_physics.yaml`** via
[`physics_config.py`](physics_config.py), so training, the planner, and the MuJoCo evaluator share one
world. If that file is absent the package falls back to documented no-spin defaults (which mirror the
shipped config). Set the `HOPE_BALL_PHYSICS_CONFIG` environment variable to point at an alternate file.

## Running

This needs Isaac Sim / Isaac Lab. The environment is a standard `ManagerBasedRLEnv` registered as
`HOPE-TableTennis-AgibotA3-v0`; launch it with your Isaac Lab runner (the robot articulation is provided
by `whole_body_tracking.robots.agibot_a3`). Each reset serves a ball from over the P2 half toward the
P1-side robot; you should see it arc, bounce on the table, and continue toward the robot.

Notes:
* The A3 URDF → USD conversion runs once on first launch, then caches.
* There is no balance/return policy in this package, so a free-standing robot may drift over several
  seconds — that is the RL follow-up. Pin the base for a stable view of the ball physics.

## Modularity / extension points

| Concern | Where |
|---|---|
| Table / net / ball dimensions, landmarks, materials, serve, bounds | [`geometry.py`](geometry.py) |
| Ball drag math + config | [`ball.py`](ball.py) |
| Physics-config loader (`configs/ball_physics.yaml`) | [`physics_config.py`](physics_config.py) |
| Scene assets (one `build_*` helper per prim) + MDP managers | [`table_tennis_env_cfg.py`](table_tennis_env_cfg.py) |
| Per-substep force application / env class | [`table_tennis_env.py`](table_tennis_env.py) |
| Realistic table+net USD mesh (visual overlay) | [`table_usd/`](table_usd/) |
| Ball/robot observations, serve event, rewards, terminations | [`mdp/`](mdp/) |
| Robot choice, stand pose, action scale, Gym registration | [`config/agibot_a3/`](config/agibot_a3/) |

### Table visuals (USD overlay)

The table / net / posts / center line are kept as **invisible cuboid colliders** (`visible=False`) that
own all bounce physics, and a realistic **USD mesh** is overlaid for looks via `build_table_usd_visual_cfg`
(`scene.table_visual`). Only the USD's *base* geometry layer is referenced, which carries **no PhysX
colliders**, so physics is unchanged. The USD frame is aligned to the world frame by translating its
local origin to `(TABLE_LENGTH/2, −TABLE_WIDTH/2, FLOOR_Z)` (floor at its local z = 0, surface at 0.76).
The mesh is slightly wider than the ITTF/cuboid table — cosmetic only. To go back to plain boxes, drop
`table_visual` and flip the cuboids' `visible=False` back to `True`; for memory-tight headless training
set `scene.table_visual = None`.

The table USD mesh is a **third-party, MIT-licensed** asset; its license is preserved verbatim at
[`table_usd/LICENSE-PACE-ICRA2026-MIT.txt`](table_usd/LICENSE-PACE-ICRA2026-MIT.txt) and must be listed in
the repository `THIRD_PARTY_NOTICES.md`.

To add a second robot (P2), add an articulation to the scene cfg. To add real match rewards (racket-to-ball
tracking, net crossing, landing on the opponent half), extend `RewardsCfg` / `mdp/rewards.py`.
