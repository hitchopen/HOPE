# `planner_imitate` — fake HITTER planner for sim-to-real bring-up

> **TEMPORARY.** This is a fake planner used to bring up the deployment pipeline
> **before** motion capture and real-ball tracking are ready. It imitates the
> HITTER planner *output interface* (`hope_msgs/RacketCommand`) with safe,
> repeatable, staged strike presets. **It does not predict real ball
> trajectories and is not the final ball planner.** When mocap + the real
> `hope_planner` are ready, stop launching this and launch `hope_planner` instead
> — the real path is untouched by this node.

## What it's for

Bring up, in order, with the validated checkpoint **`model_15200`**:
1. ONNX policy inference on the robot,
2. **deterministic, no-dither** execution (MuJoCo showed dither is unnecessary
   for stability and *hurts* the backhand — run the policy deterministically),
3. standing stability,
4. forehand / backhand swing motion,
5. target-command interface correctness,
6. safety-stop behaviour,
7. logging for later comparison with MuJoCo.

## Interface (unchanged from the real planner)

Publishes `hope_msgs/RacketCommand` on **`/racket/command`** (QoS best-effort,
depth 1) — the exact topic/type/QoS the real `hope_planner` uses, so it is a
drop-in command source. The real mocap planner (`hope_planner_node`) is **not**
modified; pick one source at a time.

`RacketCommand` has no `swing_type` or `strike_phase` field, so (matching the
real planner) **swing type is encoded by the target Y sign** (forehand −y,
backhand +y) and timing rides on `time_to_strike` / `strike_time`. The strike
phase is a **controller-side** constant (forehand `0.36`, backhand `0.50`); this
node logs it for reference but does not put it on the wire.

## Frames (read this)

`model_15200` was validated on **robot-relative** targets, so that is the
**default** output frame:

| frame | `frame_id` | meaning |
|---|---|---|
| **base_link (default)** | `base_link` | +x robot-forward, +y robot-left, +z up (height above floor); origin at the robot's ground point. Validated strike plane **x = 0.40 m**, \|y\| ∈ [0.05, 0.45], z ∈ [0.70, 1.05]. Needs no robot world pose. |
| world (optional) | `world` | HOPE canonical table frame (faithful drop-in for the real planner). Requires the robot's **measured** world pose (`robot_world_xyz`, `robot_world_yaw`); `hope_world_frame.yaml: mocap_to_base_link` is currently TODO/zero, so world output prints a loud warning and is **not** trustworthy until measured. Transform: `world = Rz(yaw) @ base + robot_world_xyz` (yaw-only; see `to_world()`). |

**Every command logs its `frame_id`** (console line, CSV column, RViz marker
frame, and `header.frame_id` on the published message).

> The near-body **x = 0.40** target is intentional for this checkpoint — do **not**
> move it to 0.70; the policy was trained at x = 0.40.

## Bring-up levels (`level:=N`)

| level | name | behaviour |
|---|---|---|
| 0 | stand | no swing; publishes `valid=False` → controller just stands |
| 1 | forehand_slow | slow / low-amplitude forehand (≈1.0 m/s) |
| 2 | forehand | normal forehand (≈2.5 m/s) |
| 3 | backhand_slow | slow / low-amplitude backhand, conservative-lower z |
| 4 | backhand | normal backhand |
| 5 | alternate | alternating forehand / backhand |

**Start at level 0/1 before level 5.**

## Run commands

Dry-run (publishes **nothing**, logs + RViz markers only) — always do this first:
```bash
ros2 launch hope_planner planner_imitate.launch.py dry_run:=true level:=1
# or the node directly:
ros2 run hope_planner planner_imitate_node --ros-args -p dry_run:=true -p level:=1
```

Robot bring-up, Level 0/1 (publishes to `/racket/command`; controller must have
its own enable/safety active before anything moves):
```bash
ros2 launch hope_planner planner_imitate.launch.py dry_run:=false level:=0   # stand
ros2 launch hope_planner planner_imitate.launch.py dry_run:=false level:=1   # slow forehand
```

Alternating forehand/backhand test:
```bash
ros2 launch hope_planner planner_imitate.launch.py dry_run:=false level:=5
```

Emergency stop (any terminal) → node commands STAND:
```bash
ros2 topic pub -1 /hope/estop std_msgs/Bool "{data: true}"
```

CSV logging for MuJoCo comparison:
```bash
ros2 run hope_planner planner_imitate_node --ros-args -p dry_run:=true -p level:=5 \
  -p csv_path:=/tmp/planner_imitate.csv
```

## Safety

* `dry_run` defaults to **true** (publishes nothing).
* This node never drives hardware directly — it only publishes planner targets;
  the downstream WBC controller gates hardware behind its own enable.
* Clamps: target position box (x∈[0.30,0.50], |y|∈[0.05,0.45], z∈[0.65,1.30]),
  `max_speed` ceiling, `max_pos_step` per-cycle slew limit.
* `backhand_disabled:=true` hard-disables backhand swings.
* `/hope/estop` (`std_msgs/Bool`) → STAND.
* Warns (and clamps) if a target leaves the `model_15200` training range.

## Safe first-hardware-test checklist

1. [ ] Controller runs `model_15200` **deterministically (no dither)**.
2. [ ] `ros2 topic echo /racket/command` looks right in **dry-run** first.
3. [ ] Confirm `frame_id` in the logs is what the controller expects (`base_link`).
4. [ ] E-stop wired: `/hope/estop true` → robot stands; verify before enabling.
5. [ ] Start `level:=0` (stand), confirm standing stability.
6. [ ] `level:=1` (slow forehand), low speed, `strike_period_s:=4.0`.
7. [ ] Watch for `CLAMPED` / `OUT-OF-TRAINING-RANGE` warnings.
8. [ ] Only after 0→1→2 look good: `level:=3/4` (backhand), then `level:=5`.
9. [ ] Keep `backhand_disabled:=true` until forehand is confirmed on hardware.
10. [ ] Save the CSV each run for MuJoCo comparison.
