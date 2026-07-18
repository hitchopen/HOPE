# What the rewrite dropped, and why

The HOPE PingPong rewrite narrowed the repository to one robot (Agibot A3) and
one unified forehand/backhand policy. That narrowing removed files that existed
in the earlier starter. This page records **what each one was, why it is not
here, and what replaced it**, so nothing is silently lost.

Every file below is still retrievable from the pre-rewrite tree:

```bash
git show 3fb0054:<path>          # e.g. git show 3fb0054:hope_ws/src/hope_planner/hope_planner/calibration.py
```

`3fb0054` is the last pre-rewrite commit ("Rename agi/ → agibot/").

## Superseded — the capability still exists, under a new name

| Dropped | Replaced by |
|---------|-------------|
| `hope_planner/calibration.py`, `test/test_calibration.py` (fit drag `k`, restitution `C_h`/`C_v` from CSV; CLI `hope_calibrate`) | `hope_training/ball_physics_fit/` — `stage1_segments.py` + `stage2_fits.py` fit the same constants and write [`configs/ball_physics.yaml`](../../configs/ball_physics.yaml), with `falsify/` checks. Note the old fallback defaults (`k=0.5, C_h=0.75, C_v=0.85`) **contradict** the current fitted values, and its CLI printed `drag_k`/`restitution_h`/`restitution_v` keys that no longer exist. |
| `hope_planner/split_calibration_csv.py` + test (split a mixed CSV into per-trajectory CSVs, mm→m inference) | `hope_training/ball_physics_fit/extract_canonical.py` — same three jobs (time rebase, mm→m autodetect, gap split) plus `--scale`/`--z-offset`/`--gap-s`/`--min-rows`. |
| `hope_bringup/config/avatar_pro_vrpn.yaml`, `launch/avatar_pro_hope_bridge.launch.py`, `launch/avatar_pro_vrpn_relay.launch.py`, `scripts/avatar_pro_vrpn_relay` | The vendored [`hope_ws/src/vrpn_mocap/`](../../hope_ws/src/vrpn_mocap) driver plus `hope_bringup/launch/hope_bringup.launch.py`, which runs `scripts/pose_to_posearray` to build the same `/poses` PoseArray (ball at index 0, matching `ball_pose_index: 0`). |
| `scripts/probe_metric.py` (probe raw strike pass rates vs the logged curriculum metric) | `scripts/evaluate.py` + `utils/success_metric.py`. The entire metric vocabulary it probed (`strike_composite_success_exact`, the ref-perturb curriculum, `strike_success_*_thresh`) was removed; `success_rate` is now the single reported number. |
| `scripts/rsl_rl/cli_args.py`, `scripts/rsl_rl/{train,play}.py` (pre-Hydra argparse plumbing) | `cfg/algo/ppo.yaml` + `utils/ppo_cfg.py::runner_kwargs`, driven by `scripts/train.py` / `scripts/play.py` via Hydra. |
| `cfg/task/TrackingFlat.yaml` (selected `Tracking-Flat-AgibotA3-v0`) | `cfg/task/HOPEPingPong.yaml` (`HOPE-PingPong-AgibotA3-v0`), the one registered tracking task. |
| `scripts/create_smoke_motion.py`, `sample_motions/README.md` (generate a stand-still clip so the pipeline runs) | The committed placeholder clips `hope_training/motions/preprocessed/hope_{forehand,backhand}.npz` with their YAML sidecars, plus [`docs/REPLACE_MOTIONS.md`](../REPLACE_MOTIONS.md). |

## `docs/interfaces/` — folded into the current docs

The four short contract files were **merged rather than restored**, so the
repository does not carry two contracts that can drift apart:

| Dropped | Folded into |
|---------|-------------|
| `docs/interfaces/policy_io.md` | [`docs/POLICY_INTERFACE.md`](../POLICY_INTERFACE.md) — the 111-D observation / 31-D action contract. |
| `docs/interfaces/ros_topics.md` | [`docs/PLANNER_INTERFACE.md`](../PLANNER_INTERFACE.md) and [`mocap/README.md`](../../mocap/README.md). Note the old file described the removed VRPN *relay*, not the current `pose_to_posearray` path. |
| `docs/interfaces/joint_order.md` | [`A3_ASSETS.md`](../../A3_ASSETS.md), which points at the canonical `hope_training/config/joint_order_agibot_a3.yaml`. |
| `docs/interfaces/frames.md` | [`mocap/README.md`](../../mocap/README.md). Its two facts that had survived only in code — the world origin (near-side left corner of the table *surface*) and `z = 0` being the playing surface with the floor at `z = -0.76 m` — were written back into that contract, sourced from `tasks/table_tennis/geometry.py`. |

## Removed with the Weights & Biases dependency

The rewrite removed external logging entirely (`scripts/train.py`: "no Weights &
Biases, no external logging service"). These went with it:

- `scripts/upload_npz.py` — 8-line W&B artifact uploader with hardcoded paths.
- `scripts/replay_npz.py` — downloaded a clip from a W&B registry and replayed it
  in Isaac. It also imported the now-deleted `robots/g1.py`. If you want a clip
  viewer, write a fresh one against a local `.npz` and `MotionLoader(path, 14, device)`.

## Out of scope — multi-robot support

The rewrite is Agibot A3 only. Restoring any of these means first re-creating the
deleted `TrackingEnvCfg` base class (only `MySceneCfg` remains in
`tasks/tracking/tracking_env_cfg.py`) **and** the robot assets, which this
repository never shipped:

- `robots/g1.py` (`G1_CYLINDER_CFG`) — pointed at a `unitree_description` URDF that is not distributed.
- `robots/smpl.py` (`SMPL_HUMANOID`) — pointed at `smpl/smpl_humanoid.usda`, not distributed.
- `tasks/tracking/config/g1/**` — registered three G1 tracking tasks.
- `tasks/tracking/config/humanoid/**` — registered three SMPL-humanoid walk tasks. It also set `MotionCommandCfg.joint_names`, a field that no longer exists.
- `tasks/tracking/config/agibot_a3/flat_env_cfg.py` — the plain-tracking A3 baseline, superseded by `config/agibot_a3/hope_env_cfg.py::HOPEPingPongEnvCfg`. It depended on both `TrackingEnvCfg` and a removed `AGIBOT_A3_ACTION_SCALE`.

Beware when porting G1 configs: `G1FlatWoStateEstimationEnvCfg` disabled
`motion_anchor_pos_b` and `base_lin_vel` on the *policy* observation group. In the
current `HOPEPingPongEnvCfg` those terms live on the privileged/critic group, so a
naive port would mis-shape the 111-D actor contract that
`validate_actor_observation_contract` enforces.

## Known gap — not yet replaced

**`scripts/csv_to_npz.py`** (retargeted motion CSV → the `.npz` the tracking task
consumes) has **no replacement**. [`docs/REPLACE_MOTIONS.md`](../REPLACE_MOTIONS.md)
asks you to bring your own retargeted motions, but the repository currently ships
no converter for them.

It was deliberately **not** restored: it needs non-trivial, unverified changes and
shipping it half-fixed risks silently producing bad training data. To revive it
(`git show 3fb0054:hope_training/whole_body_tracking/scripts/csv_to_npz.py`):

1. Drop `from whole_body_tracking.robots.g1 import G1_CYLINDER_CFG`, the `_ROBOTS`
   dict, and the `--robot` choice; make it A3-only.
2. Delete `--upload_wandb` / `--wandb_registry` and the `import wandb` block.
3. **Index body logging to `A3_TRACKED_BODIES`.** It logs `body_pos_w[0, :]` (all
   articulation bodies), but `MotionLoader` raises `ValueError` unless the file
   stores exactly the 14 tracked bodies. Use
   `robot.find_bodies(A3_TRACKED_BODIES, preserve_order=True)`.
4. Verify the CSV's joint-column count and order against the 31-entry
   `AGIBOT_A3_JOINT_NAMES`. This was **not** verifiable without a sample CSV.

## Restored instead of dropped

For the record, these *were* brought back and adapted to the current tree:

| File | Adaptation |
|------|-----------|
| `setup_train_env.local.example.sh` | `HOPE_ISAAC_PYTHON`/`HOPE_ISAACLAB_ROOT` → `ISAAC_PYTHON`/`ISAACLAB_ROOT`; W&B lines dropped. `setup_train_env.sh` auto-sources the copied file, so the example was a documented gap. |
| `assets/README.md` | URDF source path updated to `a3_deploy/URDF/`. |
| `scripts/play_table_tennis.py` | `--magnus` removed (`BallAerodynamicsCfg` has no Magnus term and `ServeConfig` is explicitly no-spin). The only Isaac scene-visualization entry point that needs no checkpoint. |
| `hope_planner/bag_to_csv.py` | Registered as `hope_bag_to_csv` in `setup.py`; `rosbag2_py` / `rosidl_runtime_py` declared in `package.xml`. It is the capture front end whose `t,x,y,z` output feeds `ball_physics_fit`. |
