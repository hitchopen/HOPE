# What the rewrite dropped, and why

The first HOPE rewrite narrowed the repository to one robot (Agibot A3) and one
unified forehand/backhand policy, and removed files that existed in the earlier
starter. The **build1 open-source port has since replaced that pared-down
"starter" stack with the proven internal training/deploy line**, and most of
what this page once recorded as dropped is shipped again — usually in a
stronger form. This page keeps the ledger honest in both directions: what came
back, what superseded what, and what is still deliberately **not** shipped.

Every pre-rewrite file remains retrievable from the pre-rewrite tree:

```bash
git show 3fb0054:<path>          # e.g. git show 3fb0054:hope_ws/src/hope_planner/hope_planner/calibration.py
```

`3fb0054` is the last pre-rewrite commit ("Rename agi/ → agibot/").

## Formerly dropped — now shipped again (build1 port)

| Once dropped | Status now |
|--------------|------------|
| `hope_planner/calibration.py` + `split_calibration_csv.py` (fit drag/restitution from CSV) | **Restored** as the `hope_calibrate` / `hope_split_calibration_csv` console scripts, alongside the fuller `hope_training/ball_physics_fit/` pipeline. The shipped constants are now **real venue fits**: [`configs/ball_physics.yaml`](../configs/ball_physics.yaml) plus [`configs/ball_physics_venue.yaml`](../configs/ball_physics_venue.yaml) (`drag_k = 0.1261`, …) and [`configs/incoming_ball_venue.yaml`](../configs/incoming_ball_venue.yaml) (measured serve envelope). `ball_physics_fit/` no longer carries a sample capture — it fits your own recorded CSVs (`hope_bag_to_csv` is the capture front end). |
| `scripts/evaluate.py` + `utils/success_metric.py` (the analytic in-Isaac `success_rate`) | **Restored** — `scripts/evaluate.py` reports the shared-`success_metric` `success_rate` as the fast in-Isaac estimate, layered with `scripts/mujoco_eval_onnx.py` sim-to-sim and the closed-loop rehearsal. The internal exact-strike metric suite (`probe_metric.py` and friends) stays internal. See [TRAIN_POLICY.md](TRAIN_POLICY.md#evaluation). |
| `docs/interfaces/{policy_io,ros_topics,joint_order,frames}.md` | **Restored** as compact per-topic summaries that defer to the authoritative contracts ([POLICY_INTERFACE.md](POLICY_INTERFACE.md), [PLANNER_INTERFACE.md](PLANNER_INTERFACE.md)). |
| Multi-robot scaffolding (`robots/g1.py`, `robots/smpl.py`, `tasks/tracking/config/{g1,humanoid}/**`, `config/agibot_a3/flat_env_cfg.py`) | **Restored** with the internal line's tracking stack. The caveat that mattered still applies: the G1/SMPL **robot assets are not distributed**, so those configs need your own assets to run; the A3 ping-pong line is the supported path. |

Shipped for the first time with the port (never in the starter at all):

- the **native C++ deploy runner** `a3_pingpong` with its CMake project, cross-build docker
  images, and MuJoCo rehearsal path ([RUN_ON_AGIBOT.md](RUN_ON_AGIBOT.md));
- the **C++ planner** `hope_ws/src/hope_planner_cpp` (hardware line) next to the Python planner
  and its venue **presets** (`hope_planner.yaml`, `.hitter_pure`, `.rally_v17_r10`, `.sim`,
  `planner_imitate`);
- the **world-frame contract** `table_p1_to_p2_v1` with fail-closed calibration receipts
  (`hope_ws/calibration_receipts/`, [interfaces/frames.md](interfaces/frames.md)).

## Superseded — the capability exists under a new shape

| Dropped | Replaced by |
|---------|-------------|
| `hope_bringup/config/avatar_pro_vrpn.yaml`, `launch/avatar_pro_hope_bridge.launch.py`, `launch/avatar_pro_vrpn_relay.launch.py`, `scripts/avatar_pro_vrpn_relay` | The independent [`VRPN2ROS2/`](../VRPN2ROS2) and [`NatNet2ROS2/`](../NatNet2ROS2) driver workspaces plus `hope_bringup/launch/hope_bringup.launch.py` (`mocap_backend:=vrpn|optitrack`), which builds the same `/poses` PoseArray (ball at index 0). |
| `cfg/task/TrackingFlat.yaml` (selected `Tracking-Flat-AgibotA3-v0`) | The deploy-grade **`HitterPingPong`** task (`cfg/task/HitterPingPong.yaml`, 110-D `hitter_pure` contract) — the only shipped task. |
| `scripts/create_smoke_motion.py`, `sample_motions/README.md` (generate a stand-still clip so the pipeline runs) | The committed placeholder clips `hope_training/motions/preprocessed/hope_{forehand,backhand}.npz` with their YAML sidecars, plus [`docs/REPLACE_MOTIONS.md`](REPLACE_MOTIONS.md). |
| `hope_planner/side_selection.py` (pure lateral-split function) | Side selection now lives **inside both planners** with hysteresis (`swing_side_split_y` / `swing_side_hysteresis_y`); the side travels on the wire as `swing_sign` and the policy never observes it. |
| The starter's single control path (the Python reference runner as the only runner) | **Two roles, two runners**: the Python reference runner (`a3_deploy/a3_deploy_example/reference/`, package `a3_deploy_onnx_ref_pingpong`, with `config/action_adapter.yaml` / `config/hope_pingpong_runtime.yaml`) ships as the MuJoCo evaluation/simulation reference harness, while the **native C++ runner** `a3_pingpong` (`src/a3/a3_deploy_onnx_ref/`, built with the example's CMake project, consuming the planner's flat topics in `--planner` mode) is the hardware deploy path. |
| `scripts/rsl_rl/{cli_args,train,play}.py` (pre-Hydra argparse plumbing) | The **Hydra** entry points only: `scripts/train.py` / `scripts/play.py` + the `cfg/` tree. |
| `scripts/csv_to_npz.py` (the starter's motion converter) and the internal acceptance validators | The documented motion **`.npz` + `.yaml` schema** ([REPLACE_MOTIONS.md](REPLACE_MOTIONS.md)) — produce clips with your own retargeting/FK tooling; local `.npz` files are first-class inputs (`motion_file=` / `motion_file_2=`), the W&B registry stays opt-in (`registry_name=…`, `WANDB_*` in `setup_train_env.sh`). |

## Restored earlier and still current

| File | Adaptation |
|------|-----------|
| `scripts/play_table_tennis.py` | The Isaac scene-visualization entry point that needs no checkpoint; aero drag is off by default (`--enable_aero` opts in). |
| `hope_planner/bag_to_csv.py` | Registered as `hope_bag_to_csv`; it is the capture front end whose `t,x,y,z` output feeds `ball_physics_fit`. |
| `assets/README.md` | Asset-prep instructions; the prepare script lives in the training package (`hope_training/whole_body_tracking/scripts/prepare_a3_isaac_asset.py`). |

(`setup_train_env.local.example.sh` came and went again — `setup_train_env.sh` now documents the
override inline and auto-sources a git-ignored `setup_train_env.local.sh` if you create one.)

## Still not shipped, deliberately

These are the honest gaps a fresh clone must plan around:

- **Real motion clips.** The proven line trained on the *v12fix*-generation clips
  (`hope_forehand_v12fix.npz` / `hope_backhand_v12fix.npz`); the committed clips are schema-valid
  placeholders ([REPLACE_MOTIONS.md](REPLACE_MOTIONS.md)).
- **Trained checkpoints and exported ONNX weights.** You train and export your own; the loaders'
  fail-closed metadata checks are shipped, the weights are not.
- **The AgiBot vendor deploy payload** (~1.7 GB) — vendor-gated, lives under the git-ignored
  `vendor_assets/` (see [RUN_ON_AGIBOT.md](RUN_ON_AGIBOT.md)).
- **W&B registry contents** (internal motion artifacts and run history).
- **The internal PROGRESS journal** (day-by-day engineering log).
- **Qualification CSV evidence** (`artifacts/`, git-ignored) — hardware-session receipts stay in
  the team artifact store.
