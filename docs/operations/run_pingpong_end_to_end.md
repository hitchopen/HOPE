# Ping-Pong End-to-End Runbook: fresh machine → train → gates → MuJoCo → robot

For the current checkout, begin with [START_HERE](../START_HERE.md),
[environment setup](../DISTROBOX_SETUP.md), [training](../../QUICKSTART_A3_ISAAC.md) and
[the two-file native Gate 3 entry](gate3_two_file_runner.md). The dated
V11–V17 commands below retain historical procedures; removed task/wrapper
names and legacy Python bridges are not current launch targets.

## RallyV13 pre-export workflow (2026-07-18; code ready, no checkpoint)

RallyV13 is fresh-only. The operator-owned training command is:

```bash
cd /home/dongc1/workspace/Hope_v11/hope_training/whole_body_tracking
hope_isaac_py scripts/train.py task=HOPEPingPongHitterPureRallyV13 algo=ppo \
  headless=true num_envs=4096 checkpoint_path=null \
  motion_file=/home/dongc1/workspace/HOPE/hope_training/motions/preprocessed/hope_forehand_v12fix.npz \
  motion_file_2=/home/dongc1/workspace/HOPE/hope_training/motions/preprocessed/hope_backhand_v12fix.npz
```

Do not export from WandB. For each candidate, run deterministic eval with 512 envs, seed 0,
1200 steps/tail 400 and zero noise on each of `core`, `planner`, `mix`, writing `+json_out`.
Run a fourth eval with the same settings plus:

```text
+vel_eval_distribution=mix
+gate3_sequence_spec=/home/dongc1/workspace/Hope_v11/hope_training/whole_body_tracking/analysis/rally_v13_gate3_sequence.json
+json_out=/tmp/v13_gate3_sequence.json
```

Then run `python3 scripts/check_rally_v13_acceptance.py` on all four JSON files. Only exit code zero
permits export. Preflight the selected checkpoint against its saved resolved config, then export the
same file with the exact clips:

```bash
cd /home/dongc1/workspace/Hope_v11/hope_training/whole_body_tracking
PP_EXPORT_PREFLIGHT_ONLY=1 TASK=HOPEPingPongHitterPureRallyV13 \
FH=/home/dongc1/workspace/HOPE/hope_training/motions/preprocessed/hope_forehand_v12fix.npz \
BH=/home/dongc1/workspace/HOPE/hope_training/motions/preprocessed/hope_backhand_v12fix.npz \
bash scripts/export_onnx_hitter_pure.sh /absolute/path/to/selected_run model_1000.pt

TASK=HOPEPingPongHitterPureRallyV13 \
FH=/home/dongc1/workspace/HOPE/hope_training/motions/preprocessed/hope_forehand_v12fix.npz \
BH=/home/dongc1/workspace/HOPE/hope_training/motions/preprocessed/hope_backhand_v12fix.npz \
bash scripts/export_onnx_hitter_pure.sh /absolute/path/to/selected_run model_1000.pt
```

Stage that exact ONNX only as
`a3_deploy/a3_deploy_example/assets/a3_runtime/models/model_v13_candidate.onnx`, build with
`--runtime-cfg src/a3/a3_deploy_onnx_ref/config/a3_runtime_config.pingpong.v13_candidate.yaml`,
then run only `scripts/pp_gate3_rally_v13.sh` in the Jazzy `hope` distrobox. Do not change the
generic runtime. Raw planner commands still fail the trained velocity envelope and the fake ROS
ball still cannot measure MuJoCo collision/landing; neither gap is waived by a passing Isaac
selector or runner proxy.

## RallyV12 result (2026-07-18; `model_12700` is not promoted)

Use `HOPEPingPongHitterPureRallyV12` only as a fresh run (`checkpoint_path=null`). It inherits V11
and adds three narrow debts for the independently reproduced `model_11800` residuals: contact-local
station-x margin, idle left-wrist settling, and waist-only pre-clamp q-des saturation. The actor,
clips, command distribution, READY behavior, serve table and Gate thresholds are unchanged.

The required workflow is strictly:

1. Run the fresh training command handed to the operator; do not resume V11.
2. Evaluate candidate checkpoints at 512 envs, fixed seed 0, 1200 steps/tail 400, zero noise,
   separately on `core`, `planner` and `mix`. Use the new strike-x/waist/wrist residual metrics as
   well as strike, READY and fall metrics. WandB peaks alone cannot select a checkpoint.
3. Select one checkpoint from deterministic evidence, then preflight and export that exact file
   with task V12 and the exact local FH/BH clips.
4. Validate ONNX metadata/parity and build only
   `a3_runtime_config.pingpong.v12_candidate.yaml`; do not replace the generic runtime.
5. Run only `scripts/pp_gate3_rally_v12.sh`. A 12/12 lifecycle proxy is insufficient without the
   unchanged clamp/wrist/station/plant report and physical-contact/landing evidence.

The fresh run was stopped at `model_12700`. Checkpoints `8000/10700/10900/12100/12700` were evaluated
in the required 512-env core/planner/mix matrix; `model_12700` was selected and formally exported.
Its strict V12 Gate3 result is `141 pass / 18 fail`: 12/12 lifecycle proxy and the synchronized plant
pass, but q-des clipping, forehand elbow, station-x, post-heading, station-y and pre-speed checks do
not. Physical ball contact/landing remain unmeasured. The formal artifact is simulation-only and
must not replace the generic V10 runtime or enter an MDU/robot package. Full evidence is recorded in
`docs/gates/G06_isaac_to_mujoco.md` and `/home/dongc1/.cache/hope_v12_eval/`.

Run the V12 Gate only inside the Jazzy `hope` distrobox. The host ROS install is Lyrical, while the
instrumented MuJoCo plugin was built against Jazzy; mixing them fails during plugin load and is not
policy evidence:

```bash
distrobox enter hope -- bash -lc \
  'source /opt/ros/jazzy/setup.bash && cd /home/dongc1/workspace/Hope_v11/a3_deploy/a3_deploy_example && bash scripts/pp_gate3_rally_v12.sh'
```

## RallyV11 candidate rule (2026-07-18)

For V11 the mandatory order is: deterministic checkpoint evaluation, select a checkpoint from those
eval results, export that exact checkpoint, validate metadata, then run
`scripts/pp_gate3_rally_v11.sh`. Never select or promote directly from training/WandB curves, and
never use the generic Gate3 engine directly. The current `model_11800_v11_diag.onnx` was selected
after core/planner/mix evaluation, is isolated by `a3_runtime_config.pingpong.v11_candidate.yaml`,
and failed strict Gate3; it is not a hardware candidate. The older `model_11400_v11.onnx` remains a
baseline/rollback asset. Closed-loop paired replay clears the actual Gate3 plant, so do not attribute
this failure to Isaac-versus-MuJoCo dynamics: current blockers are policy q-des/wrist/station tails,
raw-planner velocity contract mismatch, and absent physical contact/landing evidence.

Use `num_envs=512` for V11 qualification. At 256 envs the aggregate conditional metric is valid,
but each FH/BH decayed exact-sample count may hover around the guard of 50. The evaluator reports
the three counts and `global-vs-clips rate delta`; do not rank a per-clip value unless its count is
at least 50, and reject any nonzero partition delta. Run the selected checkpoint separately with
`+vel_eval_distribution=core`, `planner`, and `mix`, fixed `+seed=0`, 1200 steps, tail 400 and
`'+noise_scales=[0.0]'` before export.

The V11 serve profile starts at `x0=3.7 m` (about `2.58 s` to strike), not the old `3.2 m` profile.
That horizon covers the trained maximum READY exposure (`1.2 s`) plus the `0.96 s` backhand windup.
Gate3 forwards base localization in a separate `hope_base_pose_flat_relay` process and may release a
previously READY pending shot from the most recent supported target only within the existing `0.25 s`
invalid-command grace. New shot/station/clip identity, an old latch, or a not-yet-READY station still
fails closed.

Gate evidence is layered: planner raw command support, runner/demo command support, lifecycle/READY
proxy, external ball-racket geometry, MuJoCo physical contact, and landing are distinct. The fake ROS
ball is not a MuJoCo body, so geometry cannot certify collision or landing.

**Audience:** you have a new robot (or a new machine), a full clone of this repo, and
you want the WHOLE process — which distrobox, which directory, which command, in order.
Written 2026-07-03; C++-only control path since 2026-07-04 (§0 retirement note).

> **CURRENT V10 DIRECT HARDWARE COMMISSIONING MODEL (2026-07-15): `model_10500_v10.onnx`.**
> The operator selected `model_10500.pt` from the completed
> `2026-07-15_02-11-37_rally_v10_planner_q05q95_fixed_x058` run and explicitly chose to proceed
> directly to staged real-A3 commissioning rather than make another sim gate a prerequisite.
> Source sync, x86 build and C++/Python parity are complete; the remaining direct path is fresh
> Rockchip build, ship, MDU identity/no-command check, SHADOW, then one low-gain backhand at a time.
> This is an operator-directed commissioning run, not a claim that G06/G07 promotion is complete.
> The model uses the v13 FH/BH clips,
> one shared station-relative strike plane `x=0.58 m`, and a fixed deploy station
> `station_x = x_hit 1.02 - 0.58 = 0.44 m`; footwork is lateral in y. Its velocity curriculum
> converges to `75% planner central-q05/q95 box + 25% v13 core box`. Selection must evaluate
> core-only, planner-only, and the final 75/25 mixture separately. A promoted export must be
> `training_recipe=rally_v10`, recipe version **7**, and
> `hitter_pure_runtime_contract=rally_final_v2`. Recipe-6 artifacts, V10-on-runtime-v1, and an old
> runner binary are not certifiable. For any later Gate3 record, use only
> `scripts/pp_gate3_rally_v10.sh`; the bare
> `pp_gate3_rally.sh` is an internal engine and now fails closed without a versioned profile.
> The only current model-10500 promotion artifact is `<run>/exported/policy.onnx`, SHA256
> `fa4277801bc6bb605e0312881173c06cbb7545b403533d6226c7db11a501ec5f`. **Do not stage** the
> run-root `<run>/2026-07-15_02-11-37_rally_v10_planner_q05q95_fixed_x058.onnx`: it is the old
> recipe-6/runtime-v1 auto-export (SHA256 `d7385d2d55effdf6c2075a243ecd50296dd6cb6610bab66870b490177210d706`).

> **NEXT PRODUCTION ATTEMPT — RALLYFINALV3 (2026-07-11; code ready, no checkpoint yet):**
> use `HOPEPingPongHitterPureRallyFinalV3` with the exact local V7 pair and no checkpoint warm start.
> V3 preserves the 110-D deploy wire and x-lock/plane `0.70`. The repaired command contract uses
> venue contact height `[0.98,1.26] m` and HITTER center-return velocity envelopes rather than
> synthetic Gate3 side-line aims, while adding
> fixed 40-60 ready exposure, same/small/main lateral station transitions, torso-relative arm
> velocity imitation, early-windup-only torso-world imitation, negative settle/recovery/height
> debts, passive head and physical-only terminations. After the first venuefix fresh run exposed a
> 0.5-1.0 m dead position basin, V3 retains V7 wrist **position only** as a reach scaffold and adds
> a non-dead strike-position debt; wrist face/twist remain planner-owned. The `v7_approved` receipt is a deliberate
> compatibility acceptance, not proof of strict quiet cyclic-ready clips. Run the mechanics check
> and the one uninterrupted command in `run_training.md`; training is operator-owned. Do not resume
> model18400, FinalV1 or FinalV2 into this task. After champion selection, export with exact FH/BH,
> rebuild, then require strict no-rescue Gate3 repeated-rally success before hardware.

> **RALLYFINALV2 CANDIDATE RECIPE (2026-07-10; 40-60 timer correction ready, no checkpoint promoted):**
> `HOPEPingPongHitterPureRallyFinalV2` is the clean-base successor recipe built from
> `HOPEPingPongHitterPure` and is architecturally compatible with a strict clean `model_18400.pt`
> warm start. The audited `model_13000` did **not** use that lineage: it descends from a fresh
> RallyFinal V1 `model_7300`. V2 preserves the
> 110-D deploy-honest observation contract, fixed station x and 0.51 m racket plane, while adding
> bounded y-only station steps, pre/post settle shaping, surgical backhand left-arm clearance,
> front-facing torso stability, and mild contact-foot slip shaping.  It deliberately excludes the
> V5 arrival-controlled hold stack, mid-swing resampling, heading curriculum, and lower-body plant
> imitation. It adds a fixed external 40-60 tick (`0.8-1.2 s`) ready-clamp distribution, legal
> bilateral hip-yaw shaping, and pre-clamp ankle safety debt. The
> implementation and host-static tests are complete. The initial fresh 25-50 `model_700` is diagnostic
> only: a seed-0 zero-weight-update deterministic override to 40-60 improved ready release
> `0.7205 -> 0.8665`, but exposed unsafe clearance/foot/q-des tails and remains below the `0.95` gate. Training/resume on
> the corrected timer and AGI closed-loop gates
> remain required before this candidate may replace the historical baseline below.  Use the Final
> commands in §§3–5 and the strict no-rescue Gate 3 command in §7.  Detailed recipe and thresholds:
> [run_training.md](../../QUICKSTART_A3_ISAAC.md) and [G06](../gates/G06_isaac_to_mujoco.md).

> **CURRENT BASELINE (2026-07-08): `model_13200_footfix08` — the 110-D `hitter_pure` generation**
> (task `HOPEPingPongHitterPure`, run `2026-07-07_22-31-59_footfix`, iter 13200). Direct descendant of
> `model_12200_hitterpure` (identical faithful-HITTER MDP: independent station + station-relative fixed
> 0.70 m plane + learned velocity-normal), warm-resumed with `foot_orientation_weight=-0.8` to cure the
> pigeon-toe (hip_yaw internal rotation was reward-free with no lower-body imitation → toe-in while
> stepping). Verified: `eval_deterministic` composite **0.9925** (fh 0.996 / bh 0.978); C++⇔Python
> parity **9.5e-07**; Gate 2.5 **oracle 10/10** (incl. 20 s hold, post-swing hold, P7 cycles — needs
> the 110 idle-anchor fix in pp_policy.hpp, §6; pre-fix Δ=0 idle was 8/9 P7 creep, a runner artifact);
> **landing MC 97.8% on-table** (§7.5); **pigeon-toe FIXED** — moving-frame hip_yaw p95 **0.27 rad**
> (12200 was 0.94; envelope 0.41). Cost of the −0.8 penalty: pre-strike fall 0.4%→1.3% (still <2%).
> Lineage: model_12200 (foundation, strike 0.994) → 12900 (−0.3 footfix, toe p95 0.85, partial) →
> **13200 (−0.8 footfix, toe FIXED) = current**. 12200 remains a valid fallback (0.4% fall, toe-in).
> ⚠ **HARD REQUIRES `external_base` localization (live mocap in the arena / `--oracle-pelvis` in sim)
> — even to STRIKE**: its obs is world-frame + station-relative, so under `perfect_tracking` (the Δ=0
> dropout fallback) the swing itself diverges (Gate 2.5 perfect_tracking only **2/5** — stand + hold
> pass, swings fall). This is a STRONGER mocap requirement than the prior 177-D baseline (which could
> at least strike-in-place). NOT yet run for this checkpoint: Gate 1 MuJoCo, Gate 3 planner
> closed-loop, the rockchip build, and the hardware ship —
> those steps below still show the prior baseline's results, flagged `[110-D: PENDING]`.
> Prior baseline (177-D `model_17400_hitter177`, 2026-07-06) is retained below as the last
> hardware-shipped generation and for the gate history.
Companion docs: run_sim2real_bridge.md (historical document; not included in this checkout) (the RETIRED python
ROS control chain — historical), `a3_deploy/a3_deploy_example/PINGPONG_NEW_CHECKPOINT_TUTORIAL.md`
(AGI-side checkpoint sync in depth), [run_training.md](../../QUICKSTART_A3_ISAAC.md).

## 0. The big picture — ONE control path (C++); ROS only feeds it inputs

```
                       TRAINING (grasping box, Isaac)
                                   │ .pt
                                   ▼  export (grasping box)
                       policy.onnx (110-D hitter_pure — baseline model_12200)
                                   │
                                   ▼
        C++ deploy runner (a3_deploy_onnx_ref_pingpong) — THE control path
          scripted mode:  keys f/b/1/0 (built-in test targets; gates + bring-up)
          --planner mode: LIVE racket targets from hope_planner over two
                          low-rate ros2 topics (/racket/command_flat,
                          /a3/base_pose_flat) — the autonomous path
          /body_drive stays iceoryx in BOTH modes
                                   │ same binary
                       ┌───────────┴───────────┐
                       ▼                       ▼
                 AGI MuJoCo sim            robot MDU

        input side (plain ROS 2, laptop for sim / HDU for arena — carries NO control code):
          fake_ball_publisher (sim) / mocap vrpn relay (arena) → hope_planner
          [arena: on the HDU, ROS_DOMAIN_ID=232 to reach the MDU runner — §9.-1]
```

- **Scripted mode** (built-in test targets) is for policy validation in sim and first
  hardware swings (§6 Gate 2, §8).
- **`--planner` mode (2026-07-04) is the official autonomous path** for BOTH the sim
  closed loop (§7 Gate 3) and the hardware demo (§9): the SAME binary subscribes the
  live racket target (`/racket/command_flat`) + base pose (`/a3/base_pose_flat`) over
  the AimRT ros2 backend and drives the proven swing machinery through the C++ engage
  state machine (§9.6). This follows AGI's RobotIOBackend adaptation guide (own
  executable + backend subscription) — no ROS workspace, no python onnxruntime, no
  ros2-on-/body_drive needed on the MDU.
- The ROS side is INPUT ONLY: mocap / fake ball + `hope_planner` (pure python; the
  flat topics are `std_msgs`). It contains no policy runner.
- The ONNX auto-detects the obs contract by input dim (110 hitter_pure / 175 deploy_parity /
  177 hitter_footwork / 180) and the clip layout + hitter_pure geometry boxes come from ONNX
  metadata. The baseline `model_13200_footfix08` is 110-D (as is its foundation `model_12200_hitterpure`).

> **RETIRED 2026-07-04 — the python control chain.** The old "Path B" (hope_wbc_runner
> + agibot_hardware_bridge driving `/body_drive` over ros2) is ABANDONED: closed-loop
> testing exposed a post-swing hold runaway (base-anchored hold carrot, no hold
> timeout → the robot chased its own hold target 12 m downrange) that the C++ runner's
> bounded-hold design (box-center hold seed + `hold_recover_s` → sticky static stand)
> does not have, and maintaining two engage state machines double-spends every fix.
> This runbook documents ONLY the C++ path. The python chain survives in git history
> (`git log -- hope_ws/src/hope_wbc_runner`) and in
> run_sim2real_bridge.md (historical document; not included in this checkout) (historical).

## 1. One-time machine setup

Three distroboxes (create once):

| box | for | inside it |
| --- | --- | --- |
| `grasping` | Isaac training + ONNX export | `/workspace/isaacsim` (Isaac Sim install) |
| `hope` | ROS 2 Jazzy: hope_ws, AGI sim, x86 deploy build | ⚠ broken `.bashrc`: run `source /opt/ros/jazzy/setup.bash` in EVERY shell |
| host (no box) | rockchip cross-build (Docker) | do NOT source ROS on the host |

**Not in git — copy from the team bundle / an existing machine** (a fresh clone alone
is NOT sufficient):

- `a3_deploy/a3_deploy_example/thirdparty/` big bundles: `onnxruntime/` (118 MB — parity +
  runner builds), `unitree_sdk2/`, `rknn_runtime/`, the `.deb`. (`joint_msgs/` IS now
  tracked in git — .gitignore fixed 2026-07-03.)
- `a3_deploy/a3_deploy_example/assets/a3_runtime/models/` — the deployed `.onnx` models.
- `a3_deploy/A3_MuJoCo_Sim/` sim **binaries** (`aimrt_main`, `.so` plugins next to
  `src/models/bin/`) — configs are tracked, binaries are not.
- `hope_training/.venv-motion/` (python env with mujoco+onnxruntime) — or rebuild it.
- `/workspace/isaacsim` inside `grasping`, plus
  `hope_training/whole_body_tracking/setup_train_env.local.sh` (machine-local paths,
  intentionally untracked).
- wandb login inside `grasping` (registry access for motion clips).

**Build the ROS workspace** (hope box):

```bash
distrobox enter hope
source /opt/ros/jazzy/setup.bash
cd ~/workspace/HOPE/hope_ws
colcon build --packages-up-to hope_bringup hope_planner
source install/local_setup.bash
# sanity: pure-python tests need no ROS
cd src/hope_planner && python3 -m pytest test/ -q   # 73 passed
```

**Build the deploy package** (hope box):

```bash
distrobox enter hope
source /opt/ros/jazzy/setup.bash
cd ~/workspace/HOPE/a3_deploy/a3_deploy_example
bash scripts/build_a3_deploy_pkg.sh --arch x86_64 \
  --runtime-cfg src/a3/a3_deploy_onnx_ref/config/a3_runtime_config.pingpong.yaml
# -> dist/a3_deploy_x86_64/  (runner binary, models/, config/, setup_ros2_msgs.bash)
```

## 2. ⚠ TODO / manual-calibration master list

Fill these before the corresponding stage; the chain tells you when one is missing
(planner diagnostics, the runner's `PLANNER: <status>` line, `[pp gate] REJECT` values).

**Planner ball physics + hit plane**

- [x] ~~zeroed placeholders~~ → **VENUE FIT LANDED (2026-07-03 recordings; flight
  drag refreshed from the 2026-08-23 new-ball Motive take)** in
  `hope_ws/src/hope_planner/config/hope_planner.yaml`:
  `drag_k: 0.1220`, `restitution_h: 0.64` (no-spin grip equiv), `restitution_v: 0.9215`,
  `restitution_racket: 0.654` — matching node.py defaults; consistency-guard tests
  police drift. Re-fit ONLY if the venue/ball changes:
  `ros2 run hope_planner hope_calibrate traj1.csv ... traj15.csv` → paste values.
  (The old `drag_k 0.8781` fit was ~8× physical — do NOT restore it; it makes the
  planner demand 4.6–10 m/s racket speeds and the runner gate stands on every ball.)
- [x] `x_hit` is now **ADAPTIVE** (2026-07-04): tracks the live robot pose from
  `robot_pose_topic` (`/UCB_P1/pose` arena, `/sim/a3/pelvis_pose` sim) as
  `clamp(robot_x + x_hit_offset 0.67, [x_hit_min 0.0, x_hit_max 0.35])`. The static
  `x_hit` is only the fallback while no robot pose arrived. The [0.0,0.35] clamp is
  the table-collision protection (§9.3). Retune the window if robot placement changes.
- **RallyV10 Gate3 is a deliberate fixed-plane certification profile, not the adaptive arena
  default:** its wrapper forces `x_hit_follow_robot=false`, `x_hit=1.02`, both-side
  `x_hit_bh_delta=0`, and station x `0.44`, proving the trained station-relative plane
  `1.02 - 0.44 = 0.58 m`. Do not transfer the V8/V9 `-0.15` backhand delta into V10.

**Mocap / arena — the VENUE FILL-IN SHEET (exact file : key for every manual value)**

All paths below are the **src** trees — edit src, then `colcon build --packages-select
hope_planner hope_bringup` (NEVER hand-edit `install/`, it is overwritten).
✎ = must fill at the venue; ✓ = correct default, verify only.

*(1) `hope_ws/src/hope_planner/config/hope_planner.yaml`* — ball physics + hit plane
+ the C++ runner's flat feed:

| key (line) | now | fill with |
|---|---|---|
| ✓ `drag_k` (~59) | `0.1220` new-ball fit | re-fit only if venue/ball changes: ≥15 trajectories → CSV (t,x,y,z) → `ros2 run hope_planner hope_calibrate traj*.csv` → paste. ⚠ if the fit comes out ≥0.5 it is WRONG (the rejected 0.8781 class) — keep 0.11-0.15 |
| ✓ `restitution_h` / `restitution_v` | `0.64`/`0.9215` venue fit | same calibrate run prints both (h = no-spin grip equivalent) |
| `restitution_racket` | `0.654` venue fit (paddle e const) | re-fit only with a new racket-bounce recording |
| ✎ `marker_to_base_xyz` (bottom, flat block) | `[0,0,0]` | SAME number as (2)'s `mocap_to_base_link.p1_xyz` — the planner applies it to `/UCB_P1/pose` before publishing `/a3/base_pose_flat` for the C++ `--planner` runner |
| ✓ `policy_z_offset` (bottom, flat block) | `0.76` (field 2026-07-07) | mocap/table-surface frame → policy/floor frame on ALL planner outputs; 0.76 = table height whenever the G5 calibration puts z=0 ON the surface (it does). Sim yaml keeps 0.0 |
| ✓ `position_scale` (avatar_pro_vrpn.yaml, relay) | `0.001` (field 2026-07-07) | this venue's CMTracker streams MILLIMETRES over VRPN; relay converts to metres. Set 1.0 if the export is switched to metres |
| ✓ `robot_pose_topic` | `/UCB_P1/pose` | arena default; sim overlay overrides |
| ✓ `fit_window` | `31` (= 300 Hz ChingMu rig) | samples in the velocity fit; keep ≥100 ms of window → `round(31·rate/300)`. `--mocap-backend optitrack` derives and passes it from `--update-freq` automatically (360→37) |
| ✓ `x_hit` static fallback (~18) | `0.17` | only matters before the first `/UCB_P1/pose`; MUST equal `robot_start_x + 0.67` — if you place the robot at −0.8 (recommended, §9.3) set this to **−0.13** |
| ✓ `x_hit_offset/min/max` | `0.67 / 0.0 / 0.35` | retune the clamp only if robot placement changes (table-collision protection) |

*(2) `hope_ws/src/hope_bringup/config/hope_world_frame.yaml`* — the shared frame
definitions:

| key (line) | now | fill with |
|---|---|---|
| ✎ `mocap_to_base_link.p1_xyz` (~41) | `[0,0,0]` | the SAME marker→base_link translation as (1)'s `marker_to_base_xyz`; measurement procedure is in the file's comment block (CAD or tape-measure + level, refine by FK-vs-mocap while standing still) |
| `mocap_to_base_link.p1_rpy` (~42) | `[0,0,0]` | rotation only if the marker plate is mounted tilted |
| ✓ `landmarks_m` / `table_m` | ITTF geometry | fixed; used for the calibration checks below |

(The old item (2), `wbc_runner.yaml`, is RETIRED with the python runner (§0). Its
load-bearing values moved: robot placement → the physical convention in §9.3 + item
(1)'s `x_hit` fallback; the reachability gate → compile-time in `pp_policy.hpp`,
item (5).)

*(3) NOT yaml — launch args of `avatar_pro_hope_bridge.launch.py`* (defaults in the
launch file, override per run; used when `--mocap-backend avatar_pro`, the default):

| arg | default | fill with |
|---|---|---|
| ✎ `server:=` | `192.168.10.100` (confirmed IP, but 2026-07-07 **UNROUTABLE** from HDU+MDU — 100% loss, no `192.168.10.x` route; G3/§9.-1) | the MCServer/CMTracker PC IP; the HDU (planner host) needs a route/NIC onto `192.168.10.x` before VRPN connects |
| ✓ `port:=` | `3883` | Chingmu VRPN default |
| ✓ `update_freq:=` | `300.0` | match the camera rate (ball needs ≥240) |
| ✓ `ball_tracking_mode:=` / `ball_object:=` | `rigid_body` / `Ball` | matches the standardized naming |

*(3-OT) OptiTrack venue instead? — `run_rally_v10_hdu.sh --mocap-backend optitrack`*
(same HOPE topics out of `optitrack_hope_bridge.launch.py`; full procedure in
[run_hardware_session.md](run_hardware_session.md) "OptiTrack (Motive/NatNet) Backend"):

| value | default | fill with |
|---|---|---|
| ✎ `--mocap-ip` (`hostname:=`) | NO default — REQUIRED, explicit per run (operator policy) | the Motive PC IP; current venue value updated 2026-07-21: `192.168.100.122` (Motive 3.1.0/NatNet 4.1, unicast). The HDU still needs its own route to `192.168.100.x` (same trap as (3)'s `server:=` — verify from the HDU before the first session) |
| ✓ `--update-freq` | `360.0` — venue rate CONFIRMED 2026-07-18 by NatNet frame-number span (drop-proof; naive packet/`hz` counting under-reads ~270-320 and is normal) | the TRUE camera rate — it sets the post-launch `/UCB_P1/pose` probe threshold AND the planner `fit_window` (`round(31·rate/300)`: 360→37; ≥100 ms rule) |
| ✓ `--position-scale` | `1.0` | Motive streams metres (0.001 only for a mm feed) |
| ✓ ball | Motive rigid-body ASSET named `Ball` (MODE A, venue 2026-07-18) | >=3 markers, **pivot at the sphere center**; the yaml librigidbodytracker block stays commented out. Fallback MODE B (fully-coated single-marker ball) documented in run_mocap.md + optitrack_mct.yaml |
| ✓ Motive streaming pane | — | Z-up, Rigid Bodies ON; assets named exactly `UCB_P1`/`Ball` (+`PPT`/`UCB_P2`); Multicast and Unicast are both supported; model changes self-heal in ~1-2 s |

*(4) NOT ours to edit — CMTracker/AvatarPro side (procedures, no files):*

- Rigid bodies named EXACTLY `Ball`, `UCB_P1`, `UCB_P2` (+ `PPT` if the table is tracked) —
  these are the defaults in `hope_bringup/config/avatar_pro_vrpn.yaml`
  (`p1_object/p2_object/ball_object`), so nothing to edit if the names match; verify
  `ros2 topic list | grep vrpn_mocap`.
- World calibration IN CMTracker: metres, Z-up, origin = P1 near-side LEFT table
  corner ON the surface, +x toward the opponent. VERIFY with a marker at the net
  center → `/poses` must read ≈ `(1.37, −0.7625, 0)`.
- Boot ritual (no file): robot stands STILL, square, FACING +x, for ~2 s at every
  MOTION entry (`m`) — the C++ runner yaw-aligns the IMU there; entering MOTION
  while the robot moves/leans rotates every target transform (Gate 3 bug #9).

*(5) C++ `--planner` runner — CLI flags, not yaml* (defaults are the verified values;
touch only if the venue forces it): `--engage-min-tts 1.0`, `--cmd-timeout 0.5`,
`--invalid-grace 0.25`, `--swing-rest` (0.5 default). **Planner velocity contract (2026-07-14):**
C0-B is the default: with no velocity-mode flag, execute the planner's solved racket velocity
subject to the ONNX metadata gates. Append **`--demo`** only for the quick-demo C0-A: command
the trained per-clip box-center velocity while the planner keeps WHERE+WHEN; this gives up the
planner's requested landing velocity. `--vel-box-center` remains a deprecated compatibility alias
for existing harnesses. **Other 110-D flags:** `--vel-gate-margin 0.30` (per-clip trained
vel-box gate slack; REJECT(110)+OUT-OF-BAND prints the demand vs box), `--stream-target`
(mid-swing streaming, default OFF — enable ONLY for a model trained with
midswing_resample_prob > 0; 13200 trained 0.0). Compile-time in PpPolicyConfig:
`engage_yaw_max_deg 20` (engage heading gate, status `yawed`), `static_handoff_yaw_max_deg 10`
+ near-station 0.3 m (static-stand handoff guards), `engage_settle_s 1.0` (MOTION-entry settle). **For every 110-D `hitter_pure` model the
per-clip engage gate (z-band, y-band, station geometry) is METADATA-DRIVEN — read
from the ONNX `hitter_pure_pos_range_per_clip` / `_vel_range_per_clip` / `_base_target_range` at
boot**. A Final export prints `plane_x=0.51`, fixed reach-y `fh [-0.40,-0.40]` /
`bh [0.20,0.20]`, followed by the `RallyFinal station contract` readiness line; an old 0.70-plane
model prints its own historical boxes. A correctly exported checkpoint
needs NO hpp edit for its trained envelope. Only the WIDE outer safety gate (`gate_x_lo/hi 0.20/0.90`,
`gate_y_abs 0.85`, `gate_z_lo/hi 0.55/1.40`, `gate_speed_max 3.5`) and the SCRIPTED Gate-2/2.5 test
target (`racket_pos_w_clip` / `racket_vel_w_clip`, now aligned to the clean/Final 0.51 plane centers
fh (0.51,-0.40,0.82) / bh (0.51,0.20,1.03)) is compile-time in `pp_policy.hpp`. (Legacy
175/177 models still use the compile-time scalar box on the non-110 branch.) Rebuild rockchip if the
hpp changes or the model generation changes.

**Training / eval**

- [ ] A1 latency/jitter flag values (`racket.target_delay_steps` etc.) from timestamped
  venue recordings — currently default-off.
- [ ] Per-model reachability-gate box (compile-time `gate_*` in `pp_policy.hpp`, sheet
  item (5)) when the training boxes change — rebuild BOTH dists.
- [ ] **Next-retrain riders (2026-07-06, from the model_17400 gate campaign — mostly SUPERSEDED
  for the `hitter_pure` baseline, which REMOVED the hold phase + all hold/foot/arm stability shaping
  by design; the runner's static official stand covers idle instead of a trained hold)**:
  (a) hold arm twist (left-down/right-up, into the q_des clamp) — **term IMPLEMENTED
  same day**: `arm_hold_discipline` (hold-gated L1 to the ready-stand arm pose;
  `rewards.arm_hold_weight: -0.5` in the hitter YAML, default-off in code). Root
  cause: upper-body imitation is `swing_only` and `hold_ready` std 1.5 only pins the
  racket point → hold arm posture was reward-free. Pending: the resume run + gate
  re-verification (stage-blocker for the demo aesthetics, not for safety);
  (b) hold-gated knee soft-limit (chronic `right_knee` clamp at ~86% of ticks, max
  viol 1.29 rad — safe but habitual; NOT yet implemented);
  (c) if station range is widened toward HITTER ±0.75 m, revisit entropy 0.01 /
  hold_steps_range [50,200] / base_position 1.5 / episode 16 s (the 2026-07-06
  audited knob set — the current YAML already carries them for the arm-fix resume).

**Robot / deploy**

- [ ] Rockchip sysroot tarball (`scripts/export_rockchip_sysroot.sh` output) present in
  `thirdparty/` for the cross-build.

## 3. Train (grasping box)

```bash
distrobox enter grasping
cd ~/workspace/HOPE/hope_training/whole_body_tracking
source setup_train_env.sh        # defines hope_isaac_py

# RallyV10 fresh run (operator-owned; do not add checkpoint_path or resume it as another recipe):
bash scripts/launch_rally_v10_recovery.sh
# Canonical run started 2026-07-15 02:11 and stopped after iteration 10568:
# logs/rsl_rl/agibot_a3_hope_hitter_pure_rally_v10/
#   2026-07-15_02-11-37_rally_v10_planner_q05q95_fixed_x058
# Hardware qualification candidate selected by the operator: model_10500.pt.

# BASELINE generation = HOPEPingPongHitterPure (110-D faithful HITTER repro):
hope_isaac_py scripts/train.py task=HOPEPingPongHitterPure algo=ppo headless=true
# resume from a checkpoint (the 2026-07-07 backhand-normal-sign fix was applied as a warm-resume):
hope_isaac_py scripts/train.py task=HOPEPingPongHitterPure algo=ppo headless=true \
  checkpoint_path=logs/rsl_rl/agibot_a3_hope_hitter_pure/<run>/model_XXXX.pt

# Current primary timer-only strict resume from the actual fresh seed-0 model700. Preserve its exact
# forehand:v2/backhand:v1 artifacts; the operator launches this command, never the agent.
TIMER_BASE="$PWD/logs/rsl_rl/agibot_a3_hope_hitter_pure_rally_final_v2/2026-07-10_16-50-45_hitter_rally_final_v2_fresh_s1/model_700.pt"
TIMER_FH="$PWD/artifacts/hope_forehand:v2/motion.npz"
TIMER_BH="$PWD/artifacts/hope_backhand:v1/motion.npz"
hope_isaac_py scripts/train.py \
  task=HOPEPingPongHitterPureRallyFinalV2 algo=ppo headless=true \
  motion_file="$TIMER_FH" motion_file_2="$TIMER_BH" \
  checkpoint_path="$TIMER_BASE" checkpoint_tolerant=false \
  num_envs=8000 max_iterations=12301 seed=0 algo.algorithm.entropy_coef=0.006 \
  algo.runner.save_interval=100 \
  logger=wandb run_name=hitter_rally_final_v2_long4060_safe_from700_to13000_s0
```

This is one uninterrupted run from label 700 through `model_13000` (about nine hours at the measured
throughput). Intermediate checkpoints are for later offline champion selection, not stop/restart
gates. The older `model_18400` arm proposal remains superseded.

The same ownership rule applies to RallyV10: the original run is complete and `model_10500.pt` is
the operator-selected hardware candidate. Do not resume or restart it merely to pick up
eval/export/runner tooling changes; those changes do not alter the saved V10 training configuration.

FinalV2 runs land in `logs/rsl_rl/agibot_a3_hope_hitter_pure_rally_final_v2/<date_time>/`. Health signals (tensorboard):
`strike_composite_success_exact` climbing with fh/bh balanced (⚠ the DETERMINISTIC eval is far
higher than the stochastic W&B rollout — model_12200 read 0.75 rollout but 0.994 deterministic;
the deployed ONNX is the mean), `pre/post_strike_fall_rate` < a few %, and WATCH
`Policy/mean_noise_std` — late-run std inflation makes reward curves lie; judge checkpoints by
`eval_deterministic` + the gates below, not W&B. **hitter_pure gotcha:** a single global
`mount_normal_sign` pins the BACKHAND face-normal ~137° off velocity (composite 0 for bh) — the
baseline uses per-clip `mount_normal_sign_per_clip=(1.0,-1.0)` so both faces strike (2026-07-07).

## 4. Export ONNX (grasping box)

```bash
distrobox enter grasping
cd ~/workspace/HOPE/hope_training/whole_body_tracking
RUN="$PWD/logs/rsl_rl/agibot_a3_hope_hitter_pure_rally_final_v2/DATE_TIME_RUN_NAME"
CKPT="$RUN/model_N.pt"

# RallyV10 recipe-7 / runtime-v2. Always bind the exact saved run config and v13 clip bytes.
V10_RUN="$PWD/logs/rsl_rl/agibot_a3_hope_hitter_pure_rally_v10/2026-07-15_02-11-37_rally_v10_planner_q05q95_fixed_x058"
V10_CKPT="$V10_RUN/model_10500.pt"
V10_FH="$HOME/workspace/HOPE/hope_training/motions/preprocessed/hope_forehand_v12fix.npz"
V10_BH="$HOME/workspace/HOPE/hope_training/motions/preprocessed/hope_backhand_v12fix.npz"

# Read-only provenance check; does not launch Isaac and does not create/delete the ONNX.
PP_EXPORT_PREFLIGHT_ONLY=1 TASK=HOPEPingPongHitterPureRallyV10 \
  FH="$V10_FH" BH="$V10_BH" \
  bash scripts/export_onnx_hitter_pure.sh "$V10_RUN" "$(basename "$V10_CKPT")"

# Full export only after checkpoint selection:
TASK=HOPEPingPongHitterPureRallyV10 FH="$V10_FH" BH="$V10_BH" \
  bash scripts/export_onnx_hitter_pure.sh "$V10_RUN" "$(basename "$V10_CKPT")"

# 2026-07-15 model_10500 identities. Recompute after any re-export and require all later copies
# to equal the exported policy; the run-root auto-export is deliberately included as a negative
# identity check because it is recipe-6/runtime-v1 and must never be staged.
sha256sum "$V10_CKPT" "$V10_RUN/exported/policy.onnx" \
  "$V10_RUN/2026-07-15_02-11-37_rally_v10_planner_q05q95_fixed_x058.onnx"
# expected checkpoint: 4d4a1ab90e41a75c113f607fab99836a56d833ee65c38516f3a0252a906d28cc
# expected promoted ONNX: fa4277801bc6bb605e0312881173c06cbb7545b403533d6226c7db11a501ec5f
# forbidden run-root ONNX: d7385d2d55effdf6c2075a243ecd50296dd6cb6610bab66870b490177210d706

# 110-D hitter_pure generation (HOPEPingPongHitterPure, CURRENT BASELINE):
bash ~/workspace/HOPE/hope_training/whole_body_tracking/scripts/export_onnx_hitter_pure.sh \
  logs/rsl_rl/agibot_a3_hope_hitter_pure/<run>  [model_XXXX.pt]     # omit ckpt = newest
# RallyFinalV2 MUST export with its own task so geometry plus recipe=v2/hold=40,60 metadata
# fail closed; exporting it as V1 is an operator error:
TASK=HOPEPingPongHitterPureRallyFinalV2 FH="$FH" BH="$BH" \
  bash scripts/export_onnx_hitter_pure.sh "$RUN" "$(basename "$CKPT")"
#   e.g. the baseline: ... logs/rsl_rl/agibot_a3_hope_hitter_pure/2026-07-07_13-28-13 model_12200.pt
# 177-D hitter_footwork generation (HOPEPingPongHitter, prior baseline):
bash ~/workspace/HOPE/hope_training/whole_body_tracking/scripts/export_onnx_hitter.sh \
  logs/rsl_rl/agibot_a3_hope_hitter/<run>  [model_XXXX.pt]
# 175-D deploy_parity generation (legacy lineage):
bash ~/workspace/HOPE/hope_training/whole_body_tracking/scripts/export_onnx_deploy_parity.sh \
  logs/rsl_rl/agibot_a3_hope_deploy_parity/<run>  [model_XXXX.pt]
# -> <run>/exported/policy.onnx  (auto-kills the hung Isaac after the file appears)
# 110 exports MUST carry actor_obs_contract=hitter_pure + actor_obs_total_dim=110 +
# clip_seg_lengths/clip_strike_phases +
# hitter_pure_pos_range_per_clip / _vel_range_per_clip / _base_target_range metadata. Final also
# requires _station_y_step_range + a supported hitter_pure_runtime_contract; export and runner
# both fail nonzero if that set is incomplete or inconsistent. RallyV10 additionally requires:
#   training_recipe=rally_v10, training_recipe_version=7, runtime_contract=rally_final_v2
#   hitter_pure_vel_core_range_per_clip / hitter_pure_vel_planner_range_per_clip
#   hitter_pure_vel_planner_mix_prob=0.75 / hitter_pure_vel_range_ramp_steps=96000
# and the exact shared-plane/x-drift/recovery metadata. The exporter reads and verifies these
# values from this run's saved params/env.yaml; it must not infer them from the current source tree.
# (177 exports similarly MUST carry actor_obs_contract=hitter_footwork + ref_reach_offset_xy.)
```

⚠ **The export must bake the SAME motion clips the run trained on.** The script pins
the hopex re-grounded clips (== registry sources `hope_forehand:v2`/`hope_backhand:v1`)
that the 2026-07-03 lineage uses. For any other lineage, verify first:

```bash
grep -oE "'motion_file': \[[^]]*\]" \
  ~/workspace/HOPE/hope_training/wandb/wandb/run-*<wandb_id>/logs/debug.log | sort -u
# then override:  FH=/path/fh.npz BH=/path/bh.npz bash scripts/export_onnx_deploy_parity.sh ...
```

(A v4-clip export of a hopex-trained checkpoint failed the deploy gate with 2.6 m/s
velocity errors — same weights, wrong baked references.)

## 5. Gate 1 — training-side MuJoCo (host, no distrobox needed)

The deploy-faithful behavioral gate: nominal-stand start, no teleports, fall-only
termination. **PASS = falls 0, completion 1.0 per clip, high composite.**

### RallyV10 checkpoint selection: three velocity distributions are mandatory

RallyV10 deterministic eval defaults to the converged curriculum, not step-zero core-only
sampling. The log must print `progress=1.0` and the effective planner probability. For the first
champion selection, run all three distributions against every candidate being compared:

```bash
cd ~/workspace/HOPE/hope_training/whole_body_tracking
V10_RUN="$PWD/logs/rsl_rl/agibot_a3_hope_hitter_pure_rally_v10/2026-07-15_02-11-37_rally_v10_planner_q05q95_fixed_x058"
V10_CKPT="$V10_RUN/model_10500.pt"
V10_FH="$HOME/workspace/HOPE/hope_training/motions/preprocessed/hope_forehand_v12fix.npz"
V10_BH="$HOME/workspace/HOPE/hope_training/motions/preprocessed/hope_backhand_v12fix.npz"
COMMON=(task=HOPEPingPongHitterPureRallyV10 algo=ppo headless=true num_envs=256
        +steps=2400 +tail=1200 '+noise_scales=[0.0]'
        checkpoint="$V10_CKPT" motion_file="$V10_FH" motion_file_2="$V10_BH")

# 100% core component
hope_isaac_py scripts/eval_deterministic.py "${COMMON[@]}" +vel_eval_distribution=core
# 100% planner component
hope_isaac_py scripts/eval_deterministic.py "${COMMON[@]}" +vel_eval_distribution=planner
# production final mixture: 75% planner + 25% core
hope_isaac_py scripts/eval_deterministic.py "${COMMON[@]}" +vel_eval_distribution=mix
```

The earlier six-checkpoint sweep used only the default converged mixture and is ranking evidence,
not the required three-distribution qualification. For `model_10500`, that sweep reported at
noise `0.0/0.05`: composite `0.9960/0.9967`, FH `0.9930/0.9935`, BH `0.9990/0.9997`, pre-strike
fall `0.0078/0.0072`, post-strike fall `0/0`, ready release `0.9725/0.9777`, left-wrist reference
mean error `0.1280/0.1271 rad`, and base drift `0.1010/0.0996 m/swing`. Do not promote from those
numbers alone; record the three commands above separately before Gate 2.5/Gate3.

`+allow_vel_ramp=true` is a debugging-only escape hatch that restores natural step-counter ramping;
do not use it for checkpoint ranking, and do not combine it with `core` or `planner`. RallyV10 yaw
settle rate metrics are reduced only over their ready/post masks, so compare the printed values
directly with the `0.20 rad/s` Gate3 envelope rather than averaging in out-of-window zeros.

> **[110-D historical baseline]** For `model_12200`, the deterministic training-side validation was
> run in Isaac:
> ```bash
> hope_isaac_py scripts/eval_deterministic.py task=HOPEPingPongHitterPure algo=ppo headless=true \
>   num_envs=256 +steps=1200 +tail=400 '+noise_scales=[0.0,0.05,0.1]' \
>   checkpoint=logs/rsl_rl/agibot_a3_hope_hitter_pure/2026-07-07_13-28-13/model_12200.pt
> ```
> `model_12200` (baseline, 2026-07-07): composite **0.9936** (fh 0.996 / bh 0.992), pos err 2.4 cm,
> vel err 0.18, pos/vel/normal pass all > 0.99, pre-strike fall 0.5 %, post 0.1 % — the best of the
> 12000–12400 window (all clustered 0.982–0.994; it won on composite AND lowest fall rate).
>
> **[2026-07-10] The 110-D MuJoCo evaluator is now operational.** It reconstructs the
> deploy-faithful HitterPure observation and reports x/y station error, pre/post base speed,
> post leg speed/slip, backhand hand/forearm clearance, and base/torso heading.  RallyFinal must
> pass both Isaac deterministic eval and the two MuJoCo protocols below; these are kinematic
> strike-target gates, not proof of physical ball contact or return landing:
> ```bash
> RUN="$PWD/logs/rsl_rl/agibot_a3_hope_hitter_pure_rally_final_v2/DATE_TIME_RUN_NAME"
> CKPT="$RUN/model_N.pt"
> hope_isaac_py scripts/eval_deterministic.py \
>   task=HOPEPingPongHitterPureRallyFinalV2 algo=ppo headless=true \
>   num_envs=256 +steps=2400 +tail=1200 '+noise_scales=[0.0]' \
>   checkpoint="$CKPT" "motion_file=[$FH,$BH]"
>
> ../.venv-motion/bin/python scripts/scoreboard_eval.py \
>   --onnx "$RUN/exported/policy.onnx" --label rally_final_model_N \
>   --motion-files "$FH" "$BH" \
>   --task-yaml cfg/task/HOPEPingPongHitterPureRallyFinalV2.yaml \
>   --protocols deploy_faithful deploy_faithful_implicit --steps 6000
> ```
> `scoreboard_eval.py` returns nonzero when a child protocol crashes and records the RallyFinal
> full-state fields in `scoreboard.csv`; compare those fields manually with the provisional thresholds
> below/G06 until a checkpoint-calibrated automatic threshold gate is promoted.
>
> After the 2026-07-10 `model_13000` audit, contact heading is no longer sufficient. Also record
> `ready_base_heading_error_deg` / max, `ready_base_yaw_rate_abs`,
> `post_swing_base_heading_error_deg` / max, and `post_swing_base_yaw_rate_abs`. Promotion requires
> ready/post heading <10 deg mean and <15 deg max, in addition to contact heading <15 deg. The task's
> `rally_heading_debt` covers runner-ready `tts 0.45..1.40` and post `tts -0.35..-1.80`; do not replace
> this gate by increasing the contact-only torso reward.
>
> **[x-LOCKED generation, 2026-07-08] The task itself is now x-plane-locked** —
> `HOPEPingPongHitterPure.yaml` (+ the mirrored env-cfg default) pins `base_target_x_range: [0,0]`,
> so the station x is FIXED at spawn and the absolute striking plane no longer wobbles ±0.10 (it now
> equals the deploy planner's fixed `x_hit=1.03` exactly — a train↔deploy alignment fix, not just a
> constraint). The G1 command above is UNCHANGED (it inherits the lock), but two things change:
> * baseline sanity: `model_13200_footfix08` scores **0.9935** on the locked task with zero
>   retraining (fh 0.998 / bh 0.990) — the lunge was never load-bearing for striking.
> * NEW x-lock criteria for x-locked candidates (report rows already in `eval_deterministic`):
>   `base_drift_fwd_per_swing` ≤ **0.03 m** (was ~0.10 tolerance in the x-free world) and
>   `base_dist_from_origin` tail-mean ≤ **0.20 m**; watch `base_pos_error_x` ≪ `base_pos_error_y`
>   (the x-locked / y-footwork signature). Footwork budget lives in y ONLY (station y ±0.40).
>
> The 177/175 MuJoCo gate below is retained for those generations:

```bash
cd ~/workspace/HOPE/hope_training/whole_body_tracking
../.venv-motion/bin/python scripts/mujoco_eval_onnx.py \
  --onnx logs/rsl_rl/agibot_a3_hope_hitter/2026-07-06_00-56-46/exported/model_17400_hitter177.onnx \
  --motion-files artifacts/hope_forehand_hopex/motion.npz artifacts/hope_backhand_hopex/motion.npz \
  --noise-scales 0.0 --pd-mode implicit --deploy-faithful --df-clips both --steps 6000
# cross-checkpoint comparison on fixed protocols (appends to logs/scoreboard/scoreboard.csv):
../.venv-motion/bin/python scripts/scoreboard_eval.py --onnx <...> --label <name> \
  --motion-files artifacts/hope_forehand_hopex/motion.npz artifacts/hope_backhand_hopex/motion.npz
```

Reference: `model_17400_hitter177` (baseline, 2026-07-06) scored 0 falls × 3 seeds at
6000 steps, completion 1.0, composite 0.966, pos err ~6-7 cm; the prior 175-D
`model_7500_hopex` reference was composite 1.0 both clips, 7/7 swings, 0 falls at
1500 steps. ⚠ 177 models: the harness feeds a LIVE station through holds (training
semantics). `PP_DF_HOLD_DZERO=1` restores the old Δ=0-during-hold pinning for A/B
only — with it the policy free-wanders meters during holds and falls off-station
(that is the localization-DROPOUT behavior, not the nominal deploy path).

## 6. Sync to the AGI side + Gate 2 (policy-only in the official sim)

```bash
distrobox enter hope; source /opt/ros/jazzy/setup.bash
cd ~/workspace/HOPE/a3_deploy/a3_deploy_example

# RallyV10: stage the selected recipe-7/runtime-v2 ONNX as a durable source asset and run parity.
# Do this with the checkout containing runtime-contract-v2 support; never pair V10 with an old dist.
V10_ONNX="$HOME/workspace/HOPE/hope_training/whole_body_tracking/logs/rsl_rl/agibot_a3_hope_hitter_pure_rally_v10/2026-07-15_02-11-37_rally_v10_planner_q05q95_fixed_x058/exported/policy.onnx"
PYBIN=~/workspace/HOPE/hope_training/.venv-motion/bin/python \
  bash scripts/sync_pingpong_model.sh "$V10_ONNX" model_10500_v10

# The sync command runs C++<->Python parity and rewrites the durable source runtime config.
# It must finish with PARITY: PASS. Prove that source and durable staged bytes are identical:
sha256sum "$V10_ONNX" assets/a3_runtime/models/model_10500_v10.onnx
# both must be fa4277801bc6bb605e0312881173c06cbb7545b403533d6226c7db11a501ec5f

# Build AFTER sync: sync updates the durable asset and source runtime cfg; the build repopulates
# dist from those sources. A hand-copy into dist is lost on rebuild and is not certifiable.
bash scripts/build_a3_deploy_pkg.sh --arch x86_64 \
  --runtime-cfg src/a3/a3_deploy_onnx_ref/config/a3_runtime_config.pingpong.yaml

# The first V10 load must print both of these before Gate3 evidence is accepted:
#   [pp] hitter_pure runtime_contract=rally_final_v2
#   [pp] hitter_pure training_recipe=rally_v10 ...
# An old runner does not understand rally_final_v2; the new loader also rejects recipe-6,
# rally_v10-on-v1, missing velocity components, or component values that drift from recipe-7.

# Historical baseline example follows; do not run both sync examples in one promotion.
# stage asset + rewrite runtime cfg + C++<->Python parity (obs-contract-agnostic; expect PASS ~1e-6).
# The sync script auto-repoints config/a3_runtime_config.pingpong.yaml model_path to the new onnx:
PYBIN=~/workspace/HOPE/hope_training/.venv-motion/bin/python \
  bash scripts/sync_pingpong_model.sh \
   ~/workspace/HOPE/hope_training/whole_body_tracking/logs/rsl_rl/agibot_a3_hope_hitter_pure/2026-07-07_22-31-59_footfix/exported/model_13200_footfix08.onnx \
    model_13200_footfix08
# (baseline model_13200_footfix08 = model_12200 warm-resumed with foot_orientation_weight=-0.8 for the
#  gait fix; run 2026-07-07_22-31-59_footfix iter 13200. parity: max|delta| seed 9.5e-07 -> PASS.)
# 110-D note: the engage gate boxes come from ONNX metadata — no hpp box edit needed for the trained
# envelope. The current SCRIPTED Gate-2/2.5 target is the clean/Final 0.51-plane generation; the
# historical model_13200 command above is retained as provenance and must not be used to certify Final.
# Then rebuild:
bash scripts/build_a3_deploy_pkg.sh --arch x86_64 \
  --runtime-cfg src/a3/a3_deploy_onnx_ref/config/a3_runtime_config.pingpong.yaml

# Gate 2: free-base swings in the OFFICIAL AGI MuJoCo sim (C++ runner, scripted targets).
# ⚠ 110-D hitter_pure HARD-REQUIRES external_base — it FALLS on the swing under perfect_tracking
# (world-frame/station obs degrades), so use the --oracle-pelvis (real base feedback) path:
bash scripts/pp_freebase_watch.sh --single-swing --oracle-pelvis  # + run scripts/run_oracle.sh in a 2nd terminal
# (perfect_tracking `bash scripts/pp_freebase_watch.sh --single-swing` will stand + hold but fall on
#  the swing for 110-D — that is the mocap hard-requirement, NOT a regression.)
# The HEADLESS automated qualifier is Gate 2.5 (§6 blockquote below) — the recommended way to
# qualify a new checkpoint (no viewer/keyboard). For model_12200: pp_gate25.sh --oracle = 10/10
# (with the 2026-07-07 110 idle-anchor fix; 8/9 with the legacy Δ=0 idle — see the blockquote below).
```

PASS per swing direction: ≥5 clean cycles, 0 falls, no guard trips, sync_miss 0.
Boot log must show `obs_dim=110 act_dim=31` and `[pp] clip layout from ONNX metadata:
seg_len={139,132} strike_phase={0.470,0.333}` — the legacy `{95,105}` (or `obs_dim=175/177`)
means a stale binary or wrong model. A Final model must ALSO print
`plane_x=0.51`, `fh y[-0.40,-0.40]`, `bh y[0.20,0.20]`, and
`RallyFinal station contract: step_y=[0.20,0.35]`; a missing/wrong runtime marker or incomplete
box set is now a startup error. Re-export with `TASK=HOPEPingPongHitterPureRallyFinal`; do not edit
the hpp to make a mismatched artifact run.
⚠ This generation walks to its station AND its whole obs is world-frame/station-relative, so
run `--oracle-pelvis`: `perfect_tracking` runs the loops open (station obs → Δ=0 dropout) and
the 110-D swing DIVERGES (stand + hold still pass). (A 177 model instead prints `[pp] 177 hitter:
reach offsets from ONNX metadata: fh=(+0.700,-0.409) bh=(+0.706,+0.185)` and can strike-in-place
under perfect_tracking.)

> ⚠ **Gate 2 coverage limits (2026-07-04 audit)** — know what a PASS does NOT prove:
> the AUTOMATED portion drives exactly ONE path (PD_STAND warmup → MOTION at level 1
> → one swing → level-0 hold); every repeat swing, every f/b switch, and every hold
> longer than the operator's pause is HUMAN-KEYED — coverage depends on what the
> operator happened to press. Known holes, confirmed by the automated transition
> matrix (Gate 2.5 below):
> 1. **Long level-0 hold is NOT covered and DOES fall**: parked at level 0 the policy
>    hold sinks and falls at **~5 s** (measured, scripted mode, perfect_tracking) —
>    Gate 2 never sees it because '1' is pressed within seconds. Do not park at level
>    0; drop to 's' (PD_STAND) when idling.
> 2. **f/b switches are manual-only**: the mid-swing QUEUE latch and the at-hold
>    switch have no systematic coverage in Gate 2.
> 3. **Walked-position re-swings are masked under perfect_tracking** (base obs pinned
>    to the reference → the footwork loop runs open); only `--oracle-pelvis` runs
>    exercise the real-base version — manually.
>
> **Gate 2.5 — automated transition matrix** (headless, deterministic; no viewer, no
> keyboard):
>
> ```bash
> distrobox enter hope -- bash ~/workspace/HOPE/a3_deploy/a3_deploy_example/scripts/pp_gate25.sh            # perfect_tracking
> distrobox enter hope -- bash ~/workspace/HOPE/a3_deploy/a3_deploy_example/scripts/pp_gate25.sh --oracle   # + real base feedback
> ```
>
> It owns the runner on a pty (headless keys), stands the robot itself
> (reset-into-armed-PD_STAND), then drives: stand→m0 entry, 20 s m0 hold, m0→m1 fh
> swing+complete, 8 s post-swing hold, f→b at hold, bh swing from the walked
> position, mid-swing queued dir switch, latched swing, alternating cycles. Each
> phase prints PASS/FAIL individually (`[g25] ... MATRIX`) so the fragile transition
> is IDENTIFIED, not averaged away. Run BOTH loc modes when qualifying a new
> checkpoint.
>
> **Expected with `model_13200_footfix08` (CURRENT baseline; = 12200 lineage + gait fix)**: `--oracle` = **10/10 phases PASS**
> (2026-07-07 evening, WITH the 110 idle-anchor fix below) — including both historic hold-killers
> (P2 m0 hold 20 s, P3b post-swing hold 8 s) AND the P7 continuous alternating cycles, z≈1.05
> throughout.
> **⚠ The 110 IDLE-ANCHOR fix (pp_policy.hpp, 2026-07-07 evening) is LOAD-BEARING for this score.**
> The first build fed station Δ=0 at 110-D level-0 idle ("hitter_pure trains no hold") and scored
> **8/9 — P7.1 fell**. That fall was MISDIAGNOSED as a training gap (continuous-rally retrain was
> attempted: model_18000 — strike regressed 0.994→0.866 AND it diverged outright in the Δ=0 idle,
> walking +0.94 m through the world-fixed target). Actual cause: Δ=0 idle has NO pull-back, so
> follow-through displacement accumulates across swings against world-fixed scripted targets until
> a swing starts from untrained geometry (hitter_pure trains NO forward locomotion — station
> x ±0.10, drift 0.01-0.02 m/swing; the observed forward pigeon-toed creep is OOD behavior, not a
> trained gait). Fix: 110 level-0 now uses the 177-style FIXED-WORLD hold anchor (idle actively
> station-keeps; `idle_station_dzero_110` keeps the legacy Δ=0 as a compile-time A/B). Zero
> retraining needed. If a 110 model creeps forward between swings in this gate, check that flag /
> rebuild before blaming the checkpoint.
> `perfect_tracking` = **2/5** (P1 stand + P2 20 s hold PASS; **P3a fh swing FALLS** at z=0.15, and the
> later swing phases with it): for 110-D this is NOT the mild 177 P3b-only failure — the whole obs is
> world-frame/station-relative, so WITHOUT real base localization the SWING itself diverges. Consequence:
> **qualify 110-D checkpoints on `--oracle` ONLY; on HARDWARE live mocap (`external_base`) is a HARD
> requirement for the strike, not just the footwork.** Serve/point discipline (§9.3) is unchanged.
>
> **Prior baseline — expected with model_17400_hitter177 (177-D, verified 2026-07-06)**:
> `--oracle` = **10/10 phases PASS** — including the two historic killers: P2 m0
> hold 20 s (the 175-era models fell at ~5 s; the ARM-A hold training + the 177
> station anchor fixed it) and P3b post-swing hold 8 s. perfect_tracking = 3/4
> (aborts at `P3b post-swing hold`): **that FAIL is BY DESIGN**, not a regression —
> without real base localization the 177 station channel degrades to the Δ=0
> dropout fallback, and the post-swing recovery hold has no world anchor to lean
> on (the policy free-wanders and tips; same mechanism, measured in the training
> harness: falls at |torso| 1–2 m from ±0.1 m stations). Consequences: (a) qualify
> 177 checkpoints on the `--oracle` matrix; perfect_tracking checks only the
> strike-in-place + dropout-degradation path; (b) on HARDWARE, live mocap
> (`external_base`) is REQUIRED for the 177 generation — it feeds both the footwork
> loop and the hold anchor. Known cosmetic behavior: during the policy hold the
> ARMS drift/twist (left down / right up, occasionally kissing joint limits —
> q_des clamp protects): training gates upper-body imitation OFF during holds
> (`motion_body_pos_swing_only`) and `hold_ready` (std 1.5) constrains only the
> racket point, so hold arm posture is reward-free. Deploy-side cosmetics, TWO
> pieces: (1) `--arm-hold-nominal` ramps the arms to nominal during PRE-SWING
> holds only (sustained-quiet floor 1 s + 2.5 s ramp; g25 oracle 10/10 with the
> flag); (2) `--hold-recover 1.2` (planner mode) hands the post-swing recovery to
> the STATIC official stand ~1.2 s after the swing (quiescence still gates the
> switch) — Gate 3 4/4 × 2 runs verified, and the official stand nominalizes the
> arms with proven gains. ⚠ CLOSED LINE OF INQUIRY — do NOT retry a runner-side
> arm override during the POST-SWING policy hold: the policy balances out of the
> follow-through WITH its arms, and every variant tested topples it (2026-07-06,
> five falls across three variants: instant-quiet 1 s ramp z=0.26/0.23;
> sustained-quiet floor + slow ramp + instant-release z=0.38/0.35, 1-of-3 pass).
> In scripted mode the operator's 's' → PD_STAND is the post-swing arm reset.
> Proper fix: the hold-gated arm-default-pose retrain rider (see §2 TODO).
>
> *(2026-07-06 harness fix: P6 "latched fh swing" used to read `applied=False`
> on a HEALTHY latch — the queued flip legally applies while the swing clock still
> sits at the windup clamp, i.e. BEFORE "swing complete", and the old scan baseline
> missed it. The 175-era "6/7" scores had the same false FAIL. pp_gate25.py now
> scans from the pre-`f` offset.)*
>
> **Historic note (175-era, model_11400, 2026-07-04)**: `P2 m0 hold 20s` failed at
> ~5 s in BOTH loc modes while the SAME onnx held 20 s in the training-side harness
> — the AGI sim's actuation model is stricter than Isaac's on marginal holds, and
> AGI calls their sim predictive of hardware. That gap drove the ARM-A hold
> retrain; it is CLOSED in the 177 baseline (oracle P2/P3b PASS above). The
> mitigations remain shipped and still apply to any future marginal model:
> `--planner` mode never parks on the policy hold (static stand except the bounded
> recovery window, §9.6); in scripted mode drop to `s` when idling.

## 7. Gate 3 — planner + control closed-loop in MuJoCo (C++ `--planner`, sim)

The closed loop runs the DEPLOY binary itself: fake_ball → REAL hope_planner (sim
profile, publishes the flat topics) → C++ runner in `--planner` mode
(`external_base` localization from `/a3/base_pose_flat`) → AGI MuJoCo sim (iceoryx
body-drive). This is the exact hardware wiring — the only sim-specific pieces are
the fake ball and the ground-truth base pose source. Input contract + engage
semantics: §9.6.

> **▶ VERSIONED ENTRYPOINTS (2026-07-15): never run bare `pp_gate3_rally.sh`.** It is now an
> internal engine and exits before cleanup unless a wrapper supplies every geometry/scoring field.
> The model generation chooses the wrapper:
>
> - **RallyV10 recipe-7/runtime-v2:** `pp_gate3_rally_v10.sh`. Shared plane `0.58/0.58`,
>   `x_hit=1.02`, `x_hit_bh_delta=0`, station x `0.44`, x-lock `0.05`. The wrapper owns an exact
>   physics-solved 12-serve table (6 FH/6 BH; 5 stay/3 small/4 STEP) and checks its bytes before
>   starting ROS/sim. It also runs the host-only planner-envelope audit. The raw planner lob
>   velocities are intentionally below the learned boxes; certification uses `--demo`, whose union
>   center has already been proven to lie inside the planner component.
> - **RallyV11 recipe-1/runtime-v2 diagnostic:** `pp_gate3_rally_v11.sh`. It preserves V10's
>   shared plane/station and thresholds, owns the `x0=3.7 m` 12-serve profile, requires the explicit
>   V11 candidate config/model, and launches process-isolated base localization. It is not a
>   promotion shortcut: current `model_11800_v11_diag.onnx` produces 12/12 proxy completion and
>   clears heading/elbow checks, but still fails strict q-des, idle-wrist, station-x and
>   physical-evidence layers.
> - **RallyV8/V9:** `pp_gate3_rally_v8.sh`. This keeps the older per-side geometry
>   (`0.65/0.50`, `x_hit_bh_delta=-0.15`, station x `0.33`) and its separately solved serve table.
>   Do not use this wrapper for V10: it moves the backhand plane/station contract by about 15 cm.
>
> Both wrappers enforce no rescue, READY evidence, the minimum transition count, `--demo`, and
> completed/recovered proxy scoring. V10 additionally requires the runner banner
> `runtime_contract=rally_final_v2`; a recipe-name line alone cannot certify that the component
> velocity gate is active.
> ```bash
> # V10 certification:
> distrobox enter hope -- bash /home/dongc1/workspace/HOPE/a3_deploy/a3_deploy_example/scripts/pp_gate3_rally_v10.sh
>
> # V11 candidate diagnosis from its isolated worktree:
> distrobox enter hope -- bash -lc \
>   'cd /home/dongc1/workspace/Hope_v11/a3_deploy/a3_deploy_example && \
>    bash scripts/pp_gate3_rally_v11.sh'
>
> # V8/V9 certification only:
> distrobox enter hope -- bash /home/dongc1/workspace/HOPE/a3_deploy/a3_deploy_example/scripts/pp_gate3_rally_v8.sh
> ```
> PREREQ: the selected export must be staged through `sync_pingpong_model.sh`, parity must pass, and
> the x86 dist must then be rebuilt WITH `ENABLE_A3_AIMRT_BACKEND`. Verify V10 prints one shared
> `fh/bh plane_x=0.58`; V8/V9 print `fh=0.65/bh=0.50`. Gate3 uses
> `/tmp/pp_runner.log`, `/tmp/pp_runner_trace.csv`, `/tmp/pp_obs.csv`,
> `/tmp/pp_mujoco_plant_report.json`, and `/tmp/pp_rally_report.json` as evidence; a checkpoint
> filename or report prose is not model identity. `clamp_viol_<joint>` columns and final
> `[clamp-audit]` counters identify discarded q-des per joint.
>
> **Scoring changed (2026-07-14):** the conductor now scores a **completed/recovered PROXY**, gated by
> `PP_MIN_PROXY_RATE` (0.8; `PP_MIN_RETURN_RATE` is a back-compat alias) — "RETURNED" never proved ball
> contact (no ball physics). It also FAILs a serve with more than one engage event (protocol
> violation). Deploy head-zeroing is now metadata-driven (`hitter_pure_last_action_head=zero`), so
> RallyV9/V10 exports zero the two head last-action slots automatically. RallyV9 = RallyV8 + causal
> repairs for the model_8500 forensics. RallyV10 changes to the shared x=0.58 plane and the new
> 12-serve profile, adds the model_10000 recovery repairs, and uses recipe **7** plus runtime v2.
> Its report checks all three idle left-wrist joint ranges and the right-elbow contact envelope;
> missing ONNX joint defaults fail once at model scope rather than producing per-serve NaN failures.

> **Gate3 is a continuous-rally deploy rehearsal.** The internal engine is
> `pp_gate3_rally.sh`, but operators invoke it only through the versioned wrappers above. The
> per-point harness below (`pp_planner_closedloop.sh`,
> serve → ONE return → operator `p` → sim reset) was designed around the 177 walk-and-strike
> contract and structurally cannot test what the station-keeping hitter_pure generation must do
> in the demo: return serve after serve from its station with NO reset — post-swing recovery
> INTO the next engage, station-drift accumulation, fh↔bh alternation from the walked (not
> reset) pose. The rally gate runs the same exact hardware wiring, stands the robot ONCE, then
> counts PP_SERVES (default 12) consecutive serves with per-serve verdicts:
>
> ```bash
> # Choose exactly one generation-matched entrypoint:
> distrobox enter hope -- bash ~/workspace/HOPE/a3_deploy/a3_deploy_example/scripts/pp_gate3_rally_v10.sh
> # or, for V8/V9 only: .../scripts/pp_gate3_rally_v8.sh
> # PP_VIEWER=1 may be supplied for visual diagnostics. Do not override certification geometry,
> # serve bytes, PP_EXTRA_ARGS, or rescue/READY knobs in the V10 wrapper.
> # outputs: per-serve table + /tmp/pp_rally_report.json + per-tick 110-D obs /tmp/pp_obs.csv
> # deep-dive: python3 scripts/pp_rally_report.py /tmp/pp_obs.csv /tmp/pp_rally_report.json
> ```
>
> **RallyFinal certification is stricter than the historical debug protocol.** It disables
> operator rescue, requires at least 80% of 20 serves to complete the harness return/recovery
> proxy, and keeps the fixed x plane. A rescue is a failure, not a way to continue the score.
> Run Gate3 inside `distrobox enter hope -- bash`; the host ROS installation may mix Lyrical and
> Jazzy plugin ABIs and is not valid. The historical hand-written bare command is intentionally
> retired: it could combine one model's serve/geometry table with another model's report gates.
> Put a new certification profile in a reviewed wrapper instead of exporting ad-hoc environment
> variables around the internal engine.
>
> `model_13000` diagnostic warning (2026-07-10): the saved 12-serve run was **not** this strict
> protocol (`readiness_required=false`, transition requirement/observed `0/0`). It proved zero falls
> and tight x lock, but its one swing went from 5.9 deg at contact to a 21-23 deg rest, after which the
> correct 20-degree safety gate blocked all later engages. The task now trains ready+post pelvis
> recovery, but `model_13000` predates that term. Keep the 20-degree gate.
>
> The strict command was subsequently run twice from the `hope` distrobox (viewer and headless),
> after verifying that the staged ONNX exactly matches the run export. Both repeats failed before
> any swing release on the first approximately 24.5 cm station command: `engaged=0`, `returned=0`,
> one fall, `0/8` required station transitions; min pelvis z was `0.092/0.094 m`. In the headless run
> the base did move laterally from y near zero to `+0.217 m` toward station `+0.249 m`, with pre-fall
> x error no more than about `0.048 m`, but could not settle safely. The terminal approximately 1.1 m
> x error is post-fall sliding, not a forward-lunge measurement. Artifacts:
> `/home/dongc1/hope_eval_final/g3_strict_2026-07-10_153x_model13000` and
> `/home/dongc1/hope_eval_final/g3_strict_headless_2026-07-10_154356_model13000`.
>
> This makes the current verdict unambiguous: `model_13000` is **not** a continuous-rally candidate.
> It learned lateral station pursuit under an advancing Isaac clip clock, but not the runner's
> frozen-ready `move -> settle -> release` state. Do not launch a heading-only long fork next. First
> train an external fixed-duration windup clamp on every wrap (the historical first A/B used 25-50
> ticks; the corrected current FinalV2 contract is 40-60 ticks at 50 Hz),
> with no arrival-gated extension/income/curriculum, and run a station-only frozen-clock gate before
> re-enabling strikes. See G05 and `run_training.md` for the exact distribution diagnosis.
>
> The viewer's foot concern is also real. Frozen-ready raw ankle-roll q-des exceeded the deploy clamp
> on approximately `83%/97%` of left/right ticks; hip-yaw reference error p95 was `0.593 rad`, and
> right-sole tilt p95/max was `16.1/18.4 deg`. A low slip number does not detect in-place toe-in or
> ankle inversion. Gate3A now includes raw q-des clamp rate, an absolute `+/-0.20 rad` ankle q-des
> envelope, hip-yaw/ankle-roll envelopes, loaded-sole tilt/yaw, and ready-dwell slip—not only foot
> translation.
>
> **Gate3A must run before any V2 strike gate.** It sends twelve metadata-consistent lateral
> transitions (20/27.5/35 cm, FH and BH, repeated), executes the native readiness state machine, and
> hard-inhibits level-1/swing release inside the runner:
>
> ```bash
> distrobox enter hope -- bash -lc '
> cd /home/dongc1/workspace/HOPE/a3_deploy/a3_deploy_example
> PP_VIEWER=0 bash scripts/pp_gate3a_station_only.sh
> '
> ```
>
> Certification requires a V2 recipe ONNX, 12/12 READY, latency p95 <=0.80 s, zero falls/swings,
> x error <=0.04 m, heading <=15 deg, pelvis z >=0.95 m, and all fail-closed joint/sole/raw-q-des
> checks true. The final-code V1 diagnostic at
> `/home/dongc1/hope_eval_final/g3a_station_only_2026-07-10_162504/pp_gate3a_report.json` correctly
> fails after 1/12 READY (the second BH station times out) despite zero swing/fall and tight x lock.
> It also fails foot yaw/tilt, hip/ankle and raw q-des gates. `PP_G3A_ALLOW_V1=1` is diagnostic-only
> and can never turn that old recipe into a PASS.
>
> Reject-log parsing: a generic `REJECT(110)` line always prints `vel=(...)`. Count a velocity-box
> failure only when the line contains literal `OUT-OF-BAND`. The preserved `model_13000` run had zero
> such events; its one generic rejection was an out-of-band forehand z target. Do not widen velocity
> boxes or blame 119 velocity rejects from a grep of `vel=`.
>
> In this fake-ball harness, `RETURNED` means the runner engaged, completed the swing, and
> recovered.  It does **not** prove racket–ball contact or an on-table landing.  Final promotion
> therefore also requires a ball-contact/landing-instrumented MuJoCo or hardware closed loop;
> the no-ball gate certifies repeated locomotion/control stability only.
>
> **What its first runs on `model_13200_footfix08` found (2026-07-08 debug campaign — each fixed
> in pp_policy.hpp / the harness same night):**
> 1. **Backhand engage was mathematically impossible in planner mode** — the pre-side late gate
>    used the windup MAX (cutoff 1.0 s) which sat above the whole bh engage window [0.78, 0.87] s.
>    Fixed (windup MIN). The legacy per-point gate never caught it because its 177-era serves
>    arrived 20 cm below the 110 bh z-band and were rejected before the timing gate.
> 2. **The 110 idle hold target rode the base** (0.70 m ahead of wherever the robot is, re-anchored
>    per tick) — a moving carrot with no positional feedback; the untrained-hold policy charged
>    +0.83 m off-station in ~1 s on it. Fixed: hold target is WORLD-FIXED at the hold-station
>    anchor, per-side geometry, trained box-center velocity (the Gate-2.5-proven scripted-hold
>    obs family).
> 3. **The planner demanded out-of-band racket velocities** (vy +0.18 vs the trained fh box
>    [0.96, 1.96]; only |v|≤3.5 was gated) — the swing executed OOD commands. Fixed twice over:
>    engage/stream now gate against the per-clip `hitter_pure_vel_range_per_clip` ONNX boxes
>    (±`--vel-gate-margin` 0.30), and the physically-solvable planner velocities barely intersect
>    the trained boxes (they meet only near the low-vz corner), so the DEMO config commands the
>    trained box-center velocity instead: **`--vel-box-center`** (now **`--demo`**; planner keeps WHERE + WHEN;
>    aim precision of the return is given up — returns go where the human demo's returns went).
>    A single `target_land_y` also cannot satisfy both sides (fh returns cross-court LEFT,
>    bh RIGHT by training); hope_planner now supports per-side aim
>    (`target_land_y_fh/_bh`, `delta_t_flight_fh/_bh`) — the rally harness sets
>    fh +0.70/0.40 s, bh −0.30/0.35 s (offline-solved 10/10 in-band for the sweep serves).
> 4. **Mid-swing target streaming defaulted ON but the model trained with
>    `midswing_resample_prob: 0.0`** — every stream update was an untrained obs transition (and
>    moved the derived STATION mid-swing, which even training's resample contract forbids).
>    stream_target now defaults OFF (`--stream-target` re-enables it, only for a model actually
>    trained with resample > 0).
> 5. **Swing execution in the AGI sim leaves the robot yawed 30–55° after some follow-throughs**
>    (the reference clips yaw ≤ ±21° mid-swing and END at ~0–6°; this is execution divergence,
>    fh and bh both, stochastic). Engaging from a yawed stand is far outside the trained start
>    distribution (measured: engage at −30° → 2 m sprint, violent fall) and the STATIC official
>    stand tips ~3 s after freezing onto a yawed/staggered stance (even +17°). Fixed with three
>    guards: an engage HEADING gate (reject while >20° off the engage heading, status `yawed`),
>    a MOTION-entry settle (1 s, no engage), and the static handoff now requires near-station
>    (<0.3 m) AND near-heading (<10°) — off those bounds the POLICY hold keeps balancing (never
>    fell in any run; 20 s g25-proven) until the operator re-stands. ⚠ hardware discipline:
>    re-enter MOTION only with the robot physically re-squared to table +x — EVERY 'm', not just
>    the first (yaw-align re-captures there, and the 110 world-frame obs make heading load-bearing).
> 6. **SHADOW→MOTION clock-domain corruption** (a swing engaged during a SHADOW preview latched
>    a phantom level-1 into MOTION) — planner-mode swing state is now fully reset at every
>    SHADOW/MOTION entry; also: guard-tripped mid-swing aborts now restart the recovery clock
>    (no more instant stiff-stand on a tilted robot), swing-speed keys are ignored in planner
>    mode, and the last-action obs is zeroed while the static stand owns the robot.
>
> **Result progression (12-serve protocol, PP_MIN_RETURN_RATE=0.5):** run 1 (pre-fix) fell on
> serve 1's follow-through charge; run 4 (idle anchor + vel gates) returned fh+bh back-to-back
> with no reset — the first-ever 110-D closed-loop rally pair — then tipped on a yawed static
> handoff; run 6 (`--hold-recover 9999`, policy hold forever) had **ZERO falls across all 12
> serves** (min z 1.011) but starved engages (the untrained policy-hold idle wobbles through the
> narrow engage windows) — confirming the architecture: static stand for engage-readiness,
> policy hold for recovery, 10° handoff gate between them.
> **FINAL (runs 7+8, `--vel-box-center`, now `--demo`, all gates, reproducible): 12 serves, 0 falls, min z
> ≈1.00, station drift 0.15 m, 3 engages (fh/bh/fh) all swings completed safely — 2 fully
> returned+recovered, the 3rd still balancing on the policy hold at run end; 2 operator
> rescues.** That is the CERTIFIED demo behavior for `model_13200_footfix08`: ~1 clean return
> per stand at a 5.7 s serve cadence, `yawed`-rejects (not falls) after divergent
> follow-throughs, operator re-stand to continue. Nothing fell in 4 consecutive
> gated-configuration runs (~45 rally-minutes equivalent).
>
> **The model-level finding this gate surfaces (retrain rider):** hitter_pure never trained the
> post-swing regime (episodes end after the strike), so follow-through heading/stance quality in
> the stricter AGI sim is chaotic even at box-center targets. The demo protocol that is safe
> TODAY: serve → return → if the next serves are `yawed`-rejected, operator re-stand ('p', square
> the robot to +x, 's', 'm') → continue. Expect roughly one clean return per stand, sometimes
> two-plus. The durable fix is the rally/hold retrain done RIGHT (station-anchored hold + heading
> restoration + post-strike brake — model_18000's regression shows it must be a careful warm
> resume, not a from-scratch rerun).
>
> **[x-LOCKED generation, 2026-07-08 — the retrain rider above is being executed, and this gate's
> PASS bar changes with it.]** Progress so far: (a) HEADING solved in training — `hold_heading=1.0`
> (hold-gated, GAE-safe; NOT post_strike_brake, which is the proven precision killer) taught
> self-squaring; `model_15500_rallyv3` ran this gate with **0 rescues** (engages self-cleared, no
> `yawed` re-stands). (b) That unmasked FORWARD DRIFT as the next blocker: with nobody re-standing
> the robot, base-x creep accumulates — 3-run evidence: drift 2.37 m → FELL, 1.49 m → FELL,
> 0.38 m → survived. Root cause: the task never locked the x-plane (station x wobbled ±0.10, soft
> symmetric base_position, no x termination). Now fixed at the SOURCE (x-locked task yamls) and the
> x-locked RallyV3 retrain (warm from 13200) is the candidate pipeline.
> **PASS bar for x-locked candidates on this gate:** falls == 0 **AND rescues == 0** (no operator
> re-stands — heading recovery is trained now, so a rescue is a regression) **AND station drift ≤
> ~0.4 m over 12 serves** (the measured fall threshold sits ≥ 1.5 m; 0.38 m survived; the certified
> 13200 run drifted 0.15 m) **AND every engaged swing completes + recovers**. Return rate remains
> reported-not-gated while `--vel-box-center` (now `--demo`) trades aim for in-band velocities.

Closed-loop VERIFIED 2026-07-04 (§9.6 status); re-verified
2026-07-06 with the PRIOR 177-D baseline `model_17400_hitter177` on the full serve sweep:
`PP_POINTS=10` → **10/10 points PASS** (every point engage → swing complete →
recovery done, fall_guard=0, min pelvis z 1.010) across 10 distinct placements
(fh arrivals y −0.65..−0.20, bh y +0.02..+0.34 — the trained-box sweep).

**ONE COMMAND, headless (the standard way to run this gate):**

```bash
distrobox enter hope -- bash ~/workspace/HOPE/a3_deploy/a3_deploy_example/scripts/pp_planner_closedloop.sh
# add PP_VIEWER=1 before `bash` to WATCH it (opens the MuJoCo viewer, same test)
# Runs the WHOLE chain: AGI sim (iceoryx body-drive) + REAL hope_planner (sim profile,
# publishes the flats) + fake_ball + the C++ runner in --planner mode, conducted by
# scripts/pp_planner_conductor.py (owns the runner pty, stands the robot via
# reset-into-armed-PD_STAND, presses m only when verifiably standing).
# The conductor plays PER-POINT, mirroring the §9.3 hardware demo discipline: serve →
# return → post-swing recovery → 'p' (operator abort, point over) → reset to the start
# spot → 's' → 'm' → next point, for 3 points. The model holds ONE clean return per
# placement (known margin: the post-lunge static stand tips after ~5-7 s, and the
# walked-forward robot puts later serves out of reach) — parking it after the return
# is NOT part of the contract, on hardware OR in sim.
# PASS: conductor SUMMARY "points_ok=3/3 ... -> PASS" (each point: engage → swing
#   complete → recovery done, fall_guard=0; the limp collapse after each deliberate
#   'p' is the operator catch, NOT a fall). Logs: /tmp/pp_runner.log (runner),
#   /tmp/pp_planner.log, /tmp/pp_ball.log, /tmp/pp_sim.log.
# The [obs] line shows "PLANNER: <status>" (no_command/stale/planner_invalid/too_late/
# base_low/target_gate/rest/engage/swinging); "[pp gate] REJECT ..." prints the values
# behind every reachability rejection (z_w≈0.00 = the ball died before the plane —
# EXPECTED for serves arriving after the robot has lunged forward within a point).
```

Verification physics inside that harness (arena values DON'T work here — expected):
drag_k **0.05** on BOTH planner and fake_ball (the arena 0.15 quadratic drag kills
every slow lob at floor height before the plane — physically correct, and exactly why
§9.3 wants close-bounce high-pop serves); robot reset to x=0.33 so the adaptive x_hit
(robot_x+0.67) sits at the hop apex; serve cadence `pause_s 4.0` (the 4-6 s demo
cadence). **Serves (2026-07-06 coverage sweep, replaces the old fixed pair)**: 10
placements (5 fh + 5 bh, alternating) with per-placement vy solved against the exact
publisher physics — fh `[3.2,-0.24,0.5,-2.0,vy,3.6]` arrives y −0.65..−0.20 at z=0.725
(trained fh box y [−0.74,−0.14], z [0.72,0.92]); bh **retuned** to
`[3.2,0.24,0.5,-1.6,vy,5.0]` (slower vx + higher toss) so it arrives at z=1.008 —
the old vz=3.6 backhand serve arrived at z=0.725, **20 cm below** the trained bh box
[0.93,1.13]. The old fh `vy=+0.10` (arrival y=−0.13 = box edge; the −0.10 fix flagged
2026-07-03 had never landed here) is gone with the sweep. bh arrivals stay ≥ +0.02:
the runner classifies fh/bh by base-rel target-y sign, so negative-y backhands would
engage as forehands. The sim overlay also sets `target_land_y: 0.0` (centered sim
table) — the arena aim (−0.7625) pointed every return at the sim table's right edge
and skewed the demanded forehand vy to the WRONG SIGN vs the trained velocity box.
Default conductor = 3 points (3 distinct placements); `PP_POINTS=10` sweeps all 10,
and `PP_RESET_Y=<m>` offsets the start spot so the 177-D station command is nonzero
from the first serve. NOTE: the sim overlay's x_hit clamp is deliberately wide
([0.55,2.20]) so the plane follows the robot's walk-and-strike drift across serves —
expected in sim; the ARENA clamp ([0.0,0.35], hope_planner.yaml) is the
table-collision protection and must stay tight in real life.

**Manual / raw three-terminal variant** — ⚠ prefer `PP_VIEWER=1` above: the manual
stand dance below has a **~0.5 s** reset→`s` window that is genuinely hard to hit by
hand across two terminals, and missing it produces a state that LOOKS completely dead:
`s` on a lying robot trips the fall guard → PASSIVE, after which every reset collapses
limp and no key seems to do anything (recovery: press `s` again within 0.5 s of the
NEXT reset — or just use the conductor, which retries this loop for you). Keep this
variant for debugging individual pieces:

Terminal A — the sim, with viewer:

```bash
distrobox enter hope; source /opt/ros/jazzy/setup.bash
cd ~/workspace/HOPE/a3_deploy/a3_deploy_example
A3_SIM_CFG=a3_pingpong_iceoryx_cfg.yaml ./scripts/run_sim.sh   # no MUJOCO_GL=egl → viewer opens
```

Terminal B — planner + fake ball (the drag-0.05 verification physics):

```bash
distrobox enter hope; source /opt/ros/jazzy/setup.bash
source ~/workspace/HOPE/hope_ws/install/local_setup.bash
ros2 run hope_planner hope_planner_node --ros-args \
  --params-file ~/workspace/HOPE/hope_ws/src/hope_planner/config/hope_planner.yaml \
  --params-file ~/workspace/HOPE/hope_ws/src/hope_planner/config/hope_planner.sim.yaml \
  -p drag_k:=0.05 &
ros2 run hope_bringup fake_ball_publisher --ros-args \
  -p serve_forehand:="[3.2, -0.24, 0.5, -2.0, -0.10, 3.6]" \
  -p serve_backhand:="[3.2, 0.24, 0.5, -1.6, -0.10, 5.0]" \
  -p drag_k:=0.05 -p pause_s:=4.0
# fh vy=-0.10 (NOT the old +0.10: that arrived y=-0.13 = box edge; -0.10 -> y=-0.35 mid-box).
# bh retuned vx=-1.6 vz=5.0 -> arrives z=1.008 inside the trained bh box (old: 20 cm low).
# For the full 10-placement coverage sweep use pp_planner_closedloop.sh (serves:=... list).
# sanity: ros2 topic hz /racket/command_flat /a3/base_pose_flat   # both alive
```

Terminal C — the C++ runner (read the STAND ORDER WARNING first):

```bash
distrobox enter hope; source /opt/ros/jazzy/setup.bash
cd ~/workspace/HOPE/a3_deploy/a3_deploy_example/dist/a3_deploy_x86_64
A3_SOURCE_ROBOT_ENV=0 ./run_a3_pingpong.sh --planner --start passive --official-stand
# boot log MUST show: target_src = PLANNER, localization mode = external_base,
# "racket target subscriber enabled" + "base pose subscriber enabled", and the
# ONNX-metadata clip-layout line (seg_len={139,132}; {95,105} = stale binary).
# then: run  bash ~/workspace/HOPE/a3_deploy/a3_deploy_example/scripts/reset_sim.sh  in
# terminal B and press 's' HERE within ~0.5 s (official gains catch the keyframe
# stand inside the limp-buckle window); wait until visibly stable, press 'm'.
# Keys: p = PASSIVE (operator abort), h = SHADOW (full engage+swing pipeline,
# publishes nothing), q = quit. 0/1/f/b are IGNORED in --planner mode (the planner
# drives engage + side).
```

**STAND ORDER WARNING (the "falls immediately" trap):** the sim robot is PASSIVE
until commanded — if you reset and wait, it collapses limp within ~0.5 s and no
later key picks it up off the floor. Reset INTO the catch: `reset_sim.sh`, then `s`
immediately. And NEVER reset after MOTION entry — it teleports the robot mid-hold
(the reset-during-MOTION trap). The one-command conductor handles all of this,
which is why it is the standard way to run this gate.

Each POINT looks like: `[pp engage] ... locked ... (clock tts0=...)` → full swing
with a forward lunge (~0.7 m, the trained walk-and-strike) → `swing complete` →
bounded post-swing policy hold → `post-swing recovery done -> STATIC official
stand` → conductor `p` + reset (point over). PASS = 3/3 points with `fall_guard=0`.
`PLANNER:` status cycling no_command/stale/planner_invalid/too_late between serves
is NORMAL, as are `z_w≈0.00` gate REJECTs late in a point (the robot lunged forward,
later serves die before the walked plane). Constant REJECTs from the FIRST serve of
a point = real geometry mismatch — read the printed values.

> **Why per-point (2026-07-04 evening finding):** a 65 s "park and rally" run fell
> ~6 s after the first return — the post-lunge stance is feet-staggered, and the
> STATIC official stand cannot actively rebalance it (the policy hold that could is
> itself only good for ~5 s, Gate 2.5). Not an input-chain bug: engage, swing,
> recovery and the sticky-stand handoff all executed cleanly, and every later serve
> was correctly gate-rejected (adaptive plane ahead of the dead ball). One clean
> return per placement IS the model's contract (§9.3); the durable fix is the
> post-strike homing / long-hold TRAINING item.

## 7.5 Landing analysis — does the strike actually put the ball ON THE TABLE? (host, CPU, no sim)

⚠ **A high `strike_composite_success_exact` does NOT by itself mean the return lands.** That metric
scores how well the racket matches the COMMANDED pos/vel/normal at contact; whether the ball then
clears the net and lands in the opponent half is the PLANNER's job (it solves the racket command
from a drag+impact model aimed at `target_land`). Nothing in Gates 1–3 closes that loop — Gate 2/2.5
feed scripted targets with no ball, and Gate 3's fake_ball only drives the planner (the MuJoCo sim
has no ball to hit back). This gate closes it in expectation.

`hope_planner.landing_mc` (2026-07-07) Monte-Carlos the EXACT deploy chain per sample: sample a lob
arrival in the trained per-clip bands → `RacketTargetPlanner.plan()` (the deploy drag shooting-solve
→ racket command) → **perturb the command by the policy's MEASURED execution error** (vel-norm σ,
normal-angle σ, strike-point σ from its `eval_deterministic` exact-error rows) → forward
`predict_paddle_contact` (the SAME venue-fitted impact law as training's virtual_ball, bit-for-bit)
→ drag free-flight → net-clearance + landing classification. Flight drag uses the 2026-08-23
new-ball fit (k=0.1220); paddle e(u_n) remains from 150 real strikes and table restitution from
101 bounces in the accepted 2026-07-03 venue fit.

```bash
# host, no distrobox; pure-python + the planner's own physics
cd ~/workspace/HOPE/hope_ws/src/hope_planner
python3 -m hope_planner.landing_mc        # (venv-motion python if numpy is missing)
# to score a NEW checkpoint: edit the "champion" run() line to its eval_deterministic exact errors
#   run(vel_err, normal_err_deg, pos_err, label="model_XXXX")
```

**Results (per-model with measured errors + reference levels; arena robot placement, target_land
(2.055,−0.7625)). The four footfix-lineage checkpoints all land ~97–98% — the foot/stance gait
differences are cosmetic and don't touch the ball; on-table is set by strike accuracy, which is
near-identical (pos 0.024–0.026 m / vel 0.17 / normal ~4°) across them:**

| model (foot_orientation weight) | measured vel/nrm°/pos | ON-TABLE | dominant failure |
|---|---|---:|---|
| perfect execution (planner-solve sanity) | 0 / 0 / 0 | **100.0%** | — (lands at target, σ 8–10 cm) |
| `model_12200_hitterpure` (0, foundation) | 0.177 / 4 / 0.024 | 98.1% | net-clip 1.9% |
| `model_12900` (−0.3, toe partial) | 0.177 / 4 / 0.026 | 97.2% | net-clip 2.8% |
| `model_13000` (−0.4, mid) | 0.177 / 4 / 0.024 | 97.2% | net-clip 2.7% |
| **`model_13200_footfix08` (−0.8, BASELINE)** | 0.172 / 4 / 0.025 | **98.3%** | net-clip 1.7% |
| at the TRAINING pass thresholds (landing-insufficient) | 0.5 / 15 / 0.075 | **68.7%** | net 13% + short 14% |

**Takeaways:**
1. Champion-accuracy strikes land **~98%** — comparable to the HITTER paper's 92.3% hardware return
   rate; current `eval_deterministic` accuracy DOES translate to on-table.
2. **The planner solve is sound** (perfect execution → 100%, centered on the opponent half).
3. **⚠ The training pass thresholds are landing-INSUFFICIENT**: a policy scoring 100% "success"
   right at `strike_success_vel_thresh 0.5` / `normal 15°` lands only **67%**. So **rank checkpoints
   by the exact-error MEANS (pos/vel/normal), never by pass rates.** A landing-calibrated threshold
   set is ≈ vel 0.25 / normal 6° (→94%); tightening the thresholds is a safe METRIC recalibration
   (reward uses stds, not thresholds — no training impact).
4. **Net-clip is the dominant failure at every level** — planner-side levers (higher net margin,
   longer `delta_t_flight`) come before any training change if on-table rate is short.

**Honest bounds (not a substitute for the paper's 26-ball hardware protocol):** the arrival
distribution is synthesized (plausible descending lobs, not venue-recorded); spin is mild (0–30
rad/s); contact is assumed (pos err 2.5 cm ≪ paddle 7.5 cm radius); outgoing flight omits Magnus
(the planner's own model does too). Ground truth still needs a contact-ball sim or on-robot balls.

## 8. Deploy to the robot

> **[CURRENT DIRECT PATH: RallyV10 `model_10500_v10.onnx`.]** The recipe-7/runtime-v2 export has
> already been synced to the durable asset, the x86 package has been rebuilt, and model-specific
> C++/Python parity passes. Do not rerun training or create another launcher. Build and ship a fresh
> Rockchip package, prove the MDU loaded the same SHA, then use the policy-native wrapper in §9.9.
> V10 is 110-D and has no localization-free swing mode, so live
> calibrated mocap and `/a3/base_pose_flat` remain part of this direct path. Keep a rollback package
> on the MDU; this commissioning path does not mark G06/G07 complete.

Build the rockchip package — **on the HOST (Docker), not in a distrobox**:

```bash
cd ~/workspace/HOPE/a3_deploy/a3_deploy_example
bash scripts/build_a3_deploy_pkg.sh --arch rockchip \
  --runtime-cfg src/a3/a3_deploy_onnx_ref/config/a3_runtime_config.pingpong.yaml
# rebuild rockchip after ANY pp_*.hpp / model change — dist is never auto-rebuilt

grep model_path dist/a3_deploy_rockchip/config/a3_runtime_config.pingpong.yaml
sha256sum assets/a3_runtime/models/model_10500_v10.onnx \
  dist/a3_deploy_rockchip/models/model_10500_v10.onnx
file dist/a3_deploy_rockchip/a3_deploy_onnx_ref_pingpong
# require: models/model_10500_v10.onnx; both hashes fa427780...501ec5f; ELF aarch64
```

Ship, then perform the no-command identity/plugin check before stopping the normal robot controller:

```bash
HDU="HDU_WIFI_IP"  # replace with the current HDU address
rsync -azP -e "ssh -J agi@$HDU" \
  "$HOME/workspace/HOPE/a3_deploy/a3_deploy_example/dist/a3_deploy_rockchip/" \
  agi@10.42.10.12:/agibot/a3_deploy/
ssh -J agi@$HDU agi@10.42.10.12
# on the MDU:
source /agibot/software/v0/entry/env/env.sh
export A3_TRANSPORT=iceoryx
cd /agibot/a3_deploy
grep model_path config/a3_runtime_config.pingpong.yaml
sha256sum models/model_10500_v10.onnx
file a3_deploy_onnx_ref_pingpong
./run_rally_v10_shadow.sh
# No commands are published. Require recipe=rally_v10, runtime=rally_final_v2, shared plane 0.58,
# target_src=PLANNER, localization=external_base, both planner subscribers, all six robot-state
# groups, fresh base samples, torso IMU present, and no watchdog/sync/finite error.
```

Do not use plain scripted `--backhand` as the model-10500 hardware path. It selects clip 1 but does
not provide the live external-base contract that V10 needs. The §9.9 HDU wrapper publishes the
explicit side and the MDU wrapper runs policy-native planner mode.

**Planner-driven variant (`--planner`, 2026-07-04):** same binary, same staged bring-up,
plus live inputs. The run script auto-switches the AimRT cfg to
`a3_aimrt_config.pingpong_ros2body.yaml` (body-drive iceoryx + the two planner topics on
ros2). Boot log must show `target_src = PLANNER`, `localization mode = external_base`,
and BOTH `racket target subscriber enabled` + `base pose subscriber enabled` lines:

```bash
./run_rally_v10_real.sh
# Expands to taskset -c 4-7 run_a3_pingpong.sh --planner --policy-native
# --start passive --official-stand. There is no --demo/gain/arm/recovery override.
# 0/1/f/b keys are IGNORED (the planner drives engage + side); p/s/h/m still work — 'p'
# (PASSIVE) is the operator abort. Full field procedure: §9.9.
```

## 9. Planner + control in REAL LIFE (hardware) — the demo runbook

> **★★ Current field procedure: §9.9.** §9.8 is retired. The subsections below (§9.-1 topology,
> §9.0 gap checklist, §9.1b builds, §9.2b/c tests + body-drive) are historical/reference detail.

Scenario: HUMAN serves, robot returns. The robot does not serve. The current model is RallyV10
`model_10500_v10.onnx` (110-D, recipe 7/runtime v2). The operator chose direct staged hardware
commissioning. The supported path is §9.9 `--side both` plus the policy-native MDU wrapper; no
`--demo`, gain split, arm override, or timed static recovery is part of the field command.

⚠ **110-D HARD REQUIREMENT: live mocap** (`external_base`)
— STRONGER than the 177 generation: the whole obs is world-frame + station-relative, so without a
fresh base pose the SWING ITSELF diverges (Gate 2.5 perfect_tracking only 2/5 — stand + hold pass,
swings fall), not merely the hold anchor. Do NOT arm MOTION on hardware without the mocap relay
alive — there is NO mocap-less strike-in-place mode for this generation. **Prior baselines:**
`model_17400_hitter177` (177-D, 2026-07-06 — last HARDWARE-shipped + full Gate 1/2.5/3 chain; can
strike-in-place under perfect_tracking); `model_11400_hopex` (175-D, 2026-07-04 — the mocap-less
strike-in-place fallback). Until the 110-D rockchip ship + hardware G2 land, `model_17400_hitter177`
remains the on-robot baseline (§9.-1); `model_13200_footfix08` is the sim-verified successor.

### 9.-1 Verified robot topology + first hardware session (2026-07-07)

First on-robot bring-up (read-only recon + the G2 dry-run). **Two-box robot, both aarch64:**

| box | hostname | interfaces | reaches | runs |
|-----|----------|-----------|---------|------|
| **HDU** (head unit) | `hdu` | `eth_hdu` 10.42.10.10/24 · `wifi_hdu` 192.168.120.249/24 | the MDU (10.42.10.x, 0-loss L2) + the laptop (wifi). **NOT** the mocap LAN. | mocap relay + `hope_planner` (the flats' publisher) |
| **MDU** (motion unit) | `mdu` | `eth_mdu` 10.42.10.12/24 (gw 10.42.10.10 = HDU) | the HDU only. | the C++ `--planner` control runner (iceoryx body-drive + the two ros2 flats) |

Laptop→HDU = `ssh agi@192.168.120.249`; laptop→MDU = `ssh -J agi@192.168.120.249 agi@10.42.10.12` (password `1`, both hops). (An alternate jump `172.16.238.4` routes via **ProtonVPN** and may time out from some hosts — the wifi jump `192.168.120.249` is the reliable path.)

**Session results:**
- ✅ **Runner fully shipped + fresh on the MDU** — `/agibot/a3_deploy` has `model_17400_hitter177.onnx`
  (+ `model_p4_deployparity.onnx` 175 fallback), the aarch64 binary + run script (Jul 7 00:19), runtime
  cfg → the 177 model, and `a3_aimrt_config.pingpong_ros2body.yaml`.
- ✅ **G2 PASSES on the real MDU** (`--planner --dry-run`, no-publish, 25 s clean, no lingering proc). Boot log:
  `clip layout seg_len={139,132}`, `177 hitter reach offsets fh=(+0.700,-0.409) bh=(+0.706,+0.185)` (from ONNX
  metadata, no "computed from baked refs" warn), BOTH `racket target subscriber enabled` + `base pose subscriber
  enabled`, `backend started`, `target_src = PLANNER`, `localization mode = external_base(mocap)`. **The aarch64
  AimRT ros2 plugin loads on hardware — the historical G2 blocker is CLOSED.**
- ✅ **HDU→MDU DDS interop VERIFIED** — the runner runs on **`ROS_DOMAIN_ID=232`** (set by the MDU's
  `/agibot/software/v0/entry/env/env.sh`; both boxes `rmw_fastrtps_cpp`, `ROS_LOCALHOST_ONLY=0`). From an
  HDU shell with `export ROS_DOMAIN_ID=232`, `ros2 topic info /racket/command_flat` + `/a3/base_pose_flat`
  each show `Subscription count: 1` (the runner) across 10.42.10.x. ⚠ A fresh HDU jazzy shell is domain 0 =
  INVISIBLE to the runner — the planner shell MUST set 232. The hand-fed-flats path (§9.2b) is fully wired.
- ✅ **Planner builds on the HDU** — `colcon build --packages-up-to hope_planner` (69 passed / 4 skipped);
  `--packages-select` fails on the missing `hope_msgs` build-dep.
- ⛔ **hope_ws (mocap relay + planner) is on NEITHER box.** Build tooling IS present on both
  (colcon/cmake/g++, python3-numpy 1.24.2) → build it on the **HDU** (§9.1b). `hope_planner` is pure-python
  (builds now via `--packages-up-to hope_planner`; `hope_msgs` is a build-time dep, the runtime import is optional); **`vrpn_mocap` is C++ and needs libVRPN, installed
  nowhere and absent from the apt cache** — the one real build dep to source.
- 🟡 **Live-mocap ingress via the LAPTOP (2026-07-07 — corrects the earlier "network-blocked" claim):** the
  HDU/MDU can't route to the MCServer `192.168.10.100` (100% loss), BUT the **laptop is ON the mocap LAN**
  (`enx…` = `192.168.10.82/24`, reaches `192.168.10.100` 0-loss) AND on the HDU's wifi
  (`192.168.120.133` ↔ HDU `192.168.120.249`, 0-loss). So the mocap chain is: **laptop** runs `vrpn_mocap`+relay
  (already built in the `hope` box; ingests `192.168.10.100`) → publishes `/UCB_P1/pose`+`/ball` on **domain 232**
  over wifi → **HDU** runs `hope_planner` (subscribes those, publishes the flats to the MDU over `10.42.10.x`;
  the HDU bridges its two NICs on one domain) → **MDU** runner. **Laptop→HDU DDS on domain 232 VERIFIED
  2026-07-07** (HDU discovered + read a laptop-published probe; the laptop's ProtonVPN did NOT break it) — and
  HDU→MDU was already verified, so the full mocap transport is proven end-to-end. Remaining are venue items only:
  the `vrpn_mocap`↔MCServer VRPN connection (rigid bodies named `Ball`/`UCB_P1`/`UCB_P2`, world calibration) + the
  hope_planner params (G4/G5/G8). (The earlier "mocap blocked" conclusion only checked HDU/MDU — not the laptop,
which is the natural ingress.)

**Corrected host assignment:** the **laptop** runs the mocap relay because it alone reaches the
MCServer; the **HDU** runs `hope_planner` and publishes the two flat topics; the **MDU** runs the C++
policy runner and motor I/O. The laptop's mocap topics reach the HDU on domain 232; the HDU reaches
the MDU on `10.42.10.0/24`. Do not run the planner or mocap relay on the MDU.

**What is unblocked right now** (no mocap needed): drive the runner from a **hand-crafted flat publisher on the
HDU** (decision-tree item 2) — publish `/racket/command_flat` + `/a3/base_pose_flat` (std_msgs/Float64MultiArray)
and exercise the real engage→swing pipeline on hardware. The 177 hold anchor still wants a live base pose, so
keep such tests to single strikes + `s`-to-stand between them (the perfect_tracking/Δ=0 caveat, §6).

### 9.0 Gap checklist — fill EVERY row before arming on hardware

Legend: ✅ = filled in the repo (2026-07-04), 🏟 = venue-day measurement/procedure,
⛔ = BLOCKING engineering gap (not a venue measurement — needs real work).
**For every 🏟 row, the EXACT file : key : measurement procedure is in the §2
VENUE FILL-IN SHEET** — G3/G4/G5 → sheet items (3)+(4), G6/G7 → §9.3 placement +
(1)'s `x_hit` fallback, G8 → (1)+(2) (the `--planner` flat feed needs
`marker_to_base_xyz` too), G9 → (1).

| # | item | where | status |
|---|------|-------|--------|
| G1 | **Planner inputs DDS-visible to the MDU** *(2026-07-04 REFRAMED — was: full hope_ws chain on the MDU)*. The `--planner` C++ runner (§9.6) eliminates wbc_runner, the hw bridge, and the python-onnxruntime wheel from the MDU entirely; the control loop is the native aarch64 binary. What remains: the two flat input topics must reach the MDU's DDS, and the laptop cannot see the MDU's DDS → run **mocap relay + hope_planner ON the MDU**. hope_planner is pure python (rclpy/numpy/core msgs — the AGI robot env already ships a ros2+rclpy runtime; the flats are `std_msgs`, no custom typesupport needed on the receive side). Remaining build items: **numpy on the MDU** (pip/apt, aarch64) and **vrpn_mocap** (one small C++ pkg: 3 .cpp + libVRPN — cross-compile with the rockchip sysroot or build on the MDU), plus the IP route MDU → MCServer LAN (VRPN is plain TCP/UDP — works if routable). **[2026-07-07 SUPERSEDED: build on the HDU, not the MDU — the MDU has no route to the mocap LAN and lacks libVRPN; the HDU sees the MDU's DDS on 10.42.10.0/24. Tooling present on both boxes; hope_planner builds now (pure-python), vrpn_mocap awaits libVRPN. See §9.-1 + §9.1b.]** | HDU | 🟡 |
| G2 | **aarch64 AimRT ros2_plugin loads on the MDU** *(2026-07-04 SCOPED DOWN — was: ros2-enabled `/body_drive`)*. `/body_drive` **stays iceoryx** in `--planner` mode; only the two low-rate flat topics ride the ros2 backend (`a3_aimrt_config.pingpong_ros2body.yaml`, same dual-plugin pattern as AGI's own teleop cfg). The aarch64 `libaimrt_ros2_plugin.so` ships in the rockchip dist; it loads clean on x86-in-box but has NEVER been exercised on the MDU (the x86 HOST hits an rclcpp ABI break — box works, host doesn't; the MDU is a third environment). Verify FIRST THING: `./run_a3_pingpong.sh --planner --dry-run` must reach "backend started" with both `subscriber enabled` lines and no undefined-symbol abort; then `ros2 topic hz /racket/command_flat` (after `setup_ros2_msgs.bash`) sees the planner. If the plugin won't load → fallback §9.5. **[2026-07-07 ✅ VERIFIED on the real MDU — plugin loads, both subscribers enabled, 177 metadata parsed, `backend started`, 25 s no-publish clean. Boot log in §9.-1.]** | MDU | ✅ |
| G3 | MCServer (CMTracker PC) IP on the arena LAN → `server:=` launch arg. No working default. **[2026-07-07: IP is `192.168.10.100`, but NEITHER robot box can route to `192.168.10.x` (100% loss from HDU + MDU) — this is now a real routing/interface gap, not just a value to paste. The HDU (the planner host) needs a route or NIC onto the mocap LAN before vrpn_mocap can connect. See §9.-1.]** | venue | 🏟 ⛔route |
| G4 | CMTracker rigid bodies named EXACTLY `Ball`, `UCB_P1` (P1-side robot), `UCB_P2` (P2-side robot), `PPT`. Verify: `ros2 topic list \| grep vrpn_mocap`. | venue | 🏟 |
| G5 | Mocap world calibration: meters, Z-up, origin at P1 near-side LEFT table corner ON the surface, +x toward the opponent. VERIFY with a marker at the net center → must read ≈ (1.37, −0.7625, 0). | venue | 🏟 |
| G6 | RallyV10 robot placement: mark the desired base station on the forehand half, facing +x, then set the fixed world hit plane to `x_hit = measured_station_x + 0.58`. Example only: station `x=-0.50` → `x_hit=0.08`. Gate3's `1.02/0.44` are sim-world coordinates, not arena constants. | venue | 🏟 |
| G7 | **Placement convention**: the robot FACES +x (the opponent), square. The C++ runner yaw-aligns the IMU at every MOTION entry (`m`) — keep it still ~2 s there. There is no yaml yaw override anymore: place it square. | venue | 🏟 |
| G8 | `marker_to_base_xyz` + `mocap_to_base_link.p1.xyz`: UCB_P1 marker-cluster → base_link offset. Measure per `hope_world_frame.yaml`; `[0,0,0]` is valid only if the cluster origin is the pelvis. | venue | 🏟 |
| G9 | Ball physics: **VENUE FIT IN THE YAML** (2026-07-03 contact values plus 2026-08-23 new-ball drag: drag_k 0.1220, restitution 0.64/0.9215/0.654; consistency-guard tests police drift vs node defaults). Spot-check on venue day; full re-fit only if venue/ball changed. The salvaged Jun-23 fit (0.8781) stays REJECTED (8× physical). | venue | ✅ (spot-check 🏟) |
| G10 | RallyV10 planner geometry: `robot_pose_topic:=/UCB_P1/pose`, `x_hit_follow_robot=false`, venue `x_hit=station_x+0.58`, and `x_hit_bh_delta=0.0`. The tracked `hope_planner.hitter_pure.yaml` still documents the V8 `0.65/0.50` per-side geometry, so the V10 CLI overrides in §9.2/§9.7 are load-bearing until a dedicated arena profile is committed. | HDU | 🟡 override required |
| G11 | Runner engage-safety set (active-swing lock + frozen target, bounded post-swing hold → sticky static stand, engage-tts clock seed + clamp, invalid-flutter grace, base_low guard, **MOTION-entry yaw align**) — in the C++ runner (§9.6). | — | ✅ |
| G12 | Current direct-commissioning model is `model_10500_v10.onnx`, sourced only from the recipe-7/runtime-v2 `<run>/exported/policy.onnx` with SHA256 `fa427780...501ec5f`. Durable source sync, x86 build and model-specific parity are complete. Fresh Rockchip build, ship, MDU hash/dry-run and real backhand evidence are still to be recorded. The run-root recipe-6/runtime-v1 ONNX is forbidden. Keep `model_17400_hitter177.onnx` plus `model_p4_deployparity.onnx` as rollback assets. | laptop→MDU | 🟡 direct commissioning selected; MDU steps pending |

**G1/G2 were the blockers; the 2026-07-04 `--planner` port (§9.6) shrank both** from
"build a full aarch64 ROS chain + re-plumb /body_drive" to "get two topics onto the MDU
+ confirm one plugin loads". **2026-07-07: G2 is now CLOSED (verified on the real MDU,
§9.-1); G1 remains open and reframed — the relay+planner host is the HDU, and the current
hard sub-blocker is the mocap-LAN route (G3) + libVRPN, not the plugin.** The demo decision
tree:

1. ~~`--planner --dry-run` on the MDU loads the ros2 plugin (G2 ✓)~~ **DONE** AND relay+planner
   run **on the HDU** (G1 — pending libVRPN + mocap route) → **autonomous demo via §9.6** (official path).
2. G2 ✓ but no live mocap yet → no live ball, but §9.6 can still be driven by a hand-crafted
   `/racket/command_flat` (+ `/a3/base_pose_flat`) publisher **on the HDU** (scripted serves,
   real engage logic). **← this path is UNBLOCKED today** (G2 passed; the HDU sees the MDU's DDS).
3. ~~ros2 plugin fails on the MDU~~ (didn't) → **fallback demo §9.5** (scripted keys) — still
   available on the proven scripted-mode binary.

### 9.1 One-time prep (laptop)

```bash
# HOST (no distrobox, no ROS sourced): rebuild rockchip after C++ runner changes
cd ~/workspace/HOPE/a3_deploy/a3_deploy_example
bash scripts/build_a3_deploy_pkg.sh --arch rockchip \
  --runtime-cfg src/a3/a3_deploy_onnx_ref/config/a3_runtime_config.pingpong.yaml

# ship (HOST):
HDU="HDU_WIFI_IP"  # replace with the current HDU address
rsync -azP -e "ssh -J agi@$HDU" dist/a3_deploy_rockchip/ agi@10.42.10.12:/agibot/a3_deploy/
```

### 9.1b RETIRED build snapshot (2026-07-07)

Do not use the old `192.168.120.249` planner-only procedure or its warning against building
`hope_bringup`. The current HDU is `192.168.10.150`, owns `vrpn_mocap + hope_bringup +
hope_planner`, and has a pinned workspace VRPN source for Debian Bookworm. Use §9.8
"Mandatory build ownership" for the exact `colcon build`, then §9.9.2 for deployment.

### 9.2 Staged bring-up (each stage safe by construction)

⚠ **2026-07-07 host correction:** the mocap + planner steps (1, 2) run **on the HDU**
(`ssh agi@192.168.120.249`), NOT the MDU — see §9.-1. Only the runner steps (3, 4, 5)
run on the MDU. Until G1 (hope_ws on the HDU + libVRPN + mocap route) is closed, rehearse
on the laptop against the AGI sim, §7 (identical wiring).

Runner shells (MDU) first:

```bash
ssh -J agi@$HDU agi@10.42.10.12        # $HDU = 192.168.120.249
source /agibot/software/v0/entry/env/env.sh
```

Planner/mocap shells (HDU) first:

```bash
ssh agi@192.168.120.249
source /opt/ros/jazzy/setup.bash
source ~/hope_ws/install/local_setup.bash   # after §9.1b builds the overlay
export ROS_DOMAIN_ID=232                     # ⚠ MUST match the MDU runner (env.sh sets 232); domain 0 = invisible
export ROS_LOCALHOST_ONLY=0                  # ⚠ the HDU login env sets 1 → localhost-trapped even on 232
```

1. **Mocap sanity** — the bridge runs on the **LAPTOP**, inside the `hope` distrobox (⚠ NOT the HDU): the
   laptop is the only box that reaches the MCServer `192.168.10.100` (via its `192.168.10.82` mocap-LAN NIC; the
   HDU/MDU have no route), and its `hope_ws` already has `hope_bringup`+`vrpn_mocap` built. Laptop→HDU DDS on
   domain 232 VERIFIED 2026-07-07 (§9.-1), so the mocap topics reach the HDU planner.
   ```bash
   distrobox enter hope
   source /opt/ros/jazzy/setup.bash
   cd ~/workspace/HOPE/hope_ws && source install/local_setup.bash
   export ROS_DOMAIN_ID=232                     # MUST — so /UCB_P1/pose+/ball reach the HDU planner + MDU
   ros2 launch hope_bringup avatar_pro_hope_bridge.launch.py server:=192.168.10.100
   # sanity (laptop): ros2 topic list | grep vrpn_mocap ; ros2 topic hz /poses  (ball ≥240 Hz)
   #                  ros2 topic echo /UCB_P1/pose --once   # net-center marker ≈ (1.37, −0.7625, 0)
   ```
2. **Planner sanity** — on the **HDU** (`ROS_DOMAIN_ID=232`, no robot motion); it receives `/UCB_P1/pose`+`/ball`
   from the laptop bridge over domain 232 and republishes the flats to the MDU:
   ```bash
   cd ~/hope_ws
   V10_X_HIT=0.08  # example: measured desired base station -0.50 + trained reach 0.58
   ros2 run hope_planner hope_planner_node --ros-args \
     --params-file src/hope_planner/config/hope_planner.yaml \
     --params-file src/hope_planner/config/hope_planner.hitter_pure.yaml \
     -p robot_pose_topic:=/UCB_P1/pose \
     -p marker_to_base_xyz:="[<G8 values>]" \
     -p x_hit_follow_robot:=false \
     -p x_hit:=$V10_X_HIT \
     -p x_hit_bh_delta:=0.0 \
     -p swing_side_split_y:=-10.0 \
     -p swing_side_hysteresis_y:=0.0
   # The last two parameters make the CURRENT first hardware session backhand-only. They are a
   # commissioning override, not a replacement for both-side Gate3. The tracked hitter_pure.yaml
   # still carries V8 0.65/0.50 geometry, so all V10 geometry overrides above are load-bearing.
   # Startup must say FIXED, never FOLLOW-MODE. Recompute V10_X_HIT from the measured station.
   ros2 topic hz /racket/command_flat /a3/base_pose_flat   # both alive
   ros2 topic echo /racket/command_flat  # data[1]=1.0 (valid) on good serves,
                                         # data[2] MUST remain -1.0 (backhand-only),
                                         # data[3] (px) ≈ constant V10_X_HIT,
                                         # data[9] (tts) counting down 2.0 → 0
   ```
3. **G2 gate — the ros2 plugin loads** (first run of the binary; publishes nothing):
   ```bash
   cd /agibot/a3_deploy && export A3_TRANSPORT=iceoryx
   taskset -c 4-7 ./run_a3_pingpong.sh --planner --dry-run
   # must reach "backend started" with BOTH "subscriber enabled" lines and no
   # undefined-symbol abort; boot log shows target_src = PLANNER,
   # localization mode = external_base. Plugin won't load → fallback demo §9.5.
   ```
4. **Shadow** — staged start, then `h`:
   ```bash
   taskset -c 4-7 ./run_a3_pingpong.sh --planner --start passive --official-stand \
     --gain-scale 0.4 --leg-gain-scale 1.0
   # --official-stand: the 's' stand needs the official knee gains to bear weight (§9.7 STEP 5).
   # passive → hoist checks → 's' (stand). Then 'h' (SHADOW): the FULL engage+swing
   # pipeline runs on real serves, publishing nothing. Verify [pp engage] fires on
   # good serves, the PLANNER status cycles sanely, [pp gate] REJECT only on
   # genuinely bad serves, projected gravity ≈ [0,0,−1] on the [obs] line.
   ```
5. **Hardware** — the ARM ritual below, then `m` (MOTION); gain-scale 0.4 first,
   raise with `]` once stable.

### 9.2b Hand-fed-flats hardware test (no mocap) — the unblocked path (wiring VERIFIED 2026-07-07)

Decision-tree item 2: with the mocap chain still blocked (G3 route to `192.168.10.100` + libVRPN),
drive the MDU runner from a hand-crafted flat publisher on the HDU. This exercises the REAL
engage→swing pipeline on hardware as a **strike-in-place** (the fed base pose is static → no
footwork; that needs live mocap). Transport is verified end-to-end (§9.-1).

⚠ **Every HDU shell must `export ROS_DOMAIN_ID=232`** (the MDU `env.sh` sets it; a fresh jazzy shell
is domain 0 = invisible to the runner). ⚠ **`--dry-run` and SHADOW publish NOTHING** (the robot is NOT
driven), so keep it HOISTED/supported until the MOTION step.

**Step 1 — iterate the target with ZERO motion (`--dry-run`, robot hoisted/safe):**

MDU:
```bash
ssh -J agi@192.168.120.249 agi@10.42.10.12
source /agibot/software/v0/entry/env/env.sh          # sets ROS_DOMAIN_ID=232
cd /agibot/a3_deploy && export A3_TRANSPORT=iceoryx   # ⚠ EXACTLY iceoryx — a typo like 'iceory' STICKS in the shell (re-export or unset)
taskset -c 4-7 ./run_a3_pingpong.sh --planner --dry-run --start shadow
# --start shadow runs the policy + engage machine from t=0 with NO key-press and NO motion
# (--dry-run never publishes). Equivalent: plain --dry-run then press 'h'.
```
HDU term 1 — base pose, ≥5 Hz continuous (engage drops if base stale >0.2 s):
```bash
ssh agi@192.168.120.249
source /opt/ros/jazzy/setup.bash; export ROS_DOMAIN_ID=232
ros2 topic pub -r 30 /a3/base_pose_flat std_msgs/msg/Float64MultiArray \
  "{data: [1, 1, 0.0, 0.0, 0.95, 1.0, 0.0, 0.0, 0.0]}"   # [schema,valid,x,y,z,qw,qx,qy,qz]: robot at origin, standing
```
HDU term 2 — fire ONE forehand engage (20 msgs @4Hz ≈ 5 s window, then exits):
```bash
ssh agi@192.168.120.249
source /opt/ros/jazzy/setup.bash; export ROS_DOMAIN_ID=232
ros2 topic pub -t 20 -r 4 /racket/command_flat std_msgs/msg/Float64MultiArray \
  "{data: [1, 1, 0, 0.70, -0.41, 0.82, 2.0, 0.0, 0.5, 1.5, 0.0, 0]}"
#  [schema,valid,swing_sign,px,py,pz,vx,vy,vz,tts,strike_time,frame_code=0(world)]
#  fh target base-rel x0.70 y-0.41 z0.82 (station≈base → strike in place, no walk), tts1.5
```
Expect on the MDU (**VERIFIED on hardware 2026-07-07**, both MDU-loopback and HDU→MDU):
`[pp engage] forehand locked: tgt base-rel (+0.70,-0.41,-0.13) tts=1.45s (clock tts0=1.30s)` → PLANNER
`racket`→`swinging` → `[pp] swing complete -> level 0 (held stand) (auto re-arm after rest)` → re-engage,
all with ZERO motion. (base-rel z −0.13 = world 0.82 − pelvis 0.95; tts0 clamps to the ~1.30 s clip windup.)
If instead `[pp gate] REJECT ...`, read the printed values and adjust the target
(gate: base-rel x∈[0.20,0.90], |y|≤0.85, z∈[0.55,1.40], speed≤3.5; base z≥0.7; tts≥1.0). Side is by
base-rel y sign (y<0 = forehand). `PLANNER: no_command/stale` between bursts is normal.

**Step 2 — a real strike (MOTION), operator hands-on.** ⚠ FIRST bring up the body-drive backend
per **§9.2c** (stop the default `agibot_pm` controller, keep the HAL) — otherwise the AGI default
controller fights our runner over the motors. Robot supported per §9.2c's safety gate.
```bash
# MDU: restart WITHOUT --dry-run (AFTER the §9.2c body-drive bring-up).
# DEFAULT C0-B: execute the planner's solved velocity. Out-of-metadata-box demands are
# intentionally rejected; tune/retrain the policy-planner velocity contract instead of silently
# replacing them. Re-square the robot to +x before EVERY 'm'; `PLANNER: yawed` rejects until square.
taskset -c 4-7 ./run_a3_pingpong.sh --planner --policy-native \
  --start passive --official-stand
# Production field path: use ./run_rally_v10_real.sh, which owns this exact command.
# stage: passive -> hoist checks -> 's' (STAND, real) -> keep still ~2 s (IMU yaw-align) -> 'm' (MOTION)
# HDU: keep term-1 base stream running; fire term-2's racket burst -> ONE real forehand strike.
# hand on 'p' (PASSIVE = abort). 's' to re-stand between strikes — the static fed base pose gives no
# walk-forward recovery anchor, so single strikes only until live mocap (the §9 177 mocap requirement).
```

### 9.2c Real-motion body-drive bring-up — stop the default stack, keep the HAL (⚠ 2026-07-07)

The dry-run/engage checks (§9.2b) publish nothing → no body-drive backend needed. A REAL strike
(`m` = MOTION) publishes body-drive over iceoryx, which needs the low-level HAL EtherCat up AND the
AGI default controller stopped — otherwise TWO controllers fight the motors. Recon 2026-07-07 on the
demo robot (model **A3_P1D0**): the boot service `agibot_pm` runs the default stack that OWNS the
motors — `process_manager` → `motion_player` + `start_motion_control` + `hal_ethercat`. Good news:
`iox-roudi` (the iceoryx broker our runner needs) runs INDEPENDENTLY (ppid 1) and SURVIVES stopping
agibot_pm; the HAL publishes joint + IMU state over iceoryx (no separate estimator process seen).
`start_hal_ethercat.sh` launches `aimrt_main_hal` with the model config, refuses if one is already
running, and its RouDi launch is commented out (assumes RouDi already up — it is).

⚠⚠ **SAFETY — stopping agibot_pm can DROP the robot.** It kills the controller currently HOLDING the
robot; until our runner takes over in MOTION the robot is uncommanded. **HOIST / support the robot
(feet off-load) BEFORE stopping agibot_pm.** Never on a free-standing, weight-bearing robot. These
are `sudo` + motion-enabling commands — operator-run (the automated harness never runs them).

Sequence (robot SUPPORTED; two MDU terminals):
```bash
# --- terminal HAL (MDU) ---
sudo systemctl stop agibot_pm                 # stops the default controller (RouDi survives)
pgrep -a iox-roudi                            # MUST still be running (our runner's iceoryx broker)
pgrep -a hal_ethercat && sudo pkill -TERM aimrt_main_hal   # drop the OLD HAL so the next line owns EtherCat
source /agibot/software/v0/entry/env/env.sh
cd /agibot/software/v0
bash scripts/hal_ethercat/start_hal_ethercat.sh   # A3_P1D0 -> hal_ethercat_a3_p1d0_cfg.yaml; BLOCKS (own terminal)

# --- terminal RUNNER (MDU) --- our runner is now the SOLE controller (then §9.2b Step 2 s->m + flats):
cd /agibot/a3_deploy && export A3_TRANSPORT=iceoryx
./run_rally_v10_real.sh
# Policy-native planner velocity; no demo/gain/arm/recovery override.
```
Restore the robot's normal stack afterwards: `sudo systemctl start agibot_pm`.

### 9.3 The ARM ritual + demo constraints (READ BEFORE ENABLING)

The C++ runner has NO ROS estop — the operator's hand stays on the runner keyboard;
**`p` (PASSIVE) is the abort** (robot goes limp: hoist/catch ready), and the
always-on fall guard drops to PASSIVE automatically if gravity-z says fallen for
>0.5 s.

```text
ARM sequence (runner keyboard, robot at its marked spot):
  1. robot standing STILL at the start spot, FACING the opponent (+x), square;
  2. 's'  — PD_STAND (official gains, pose-blended entry); verify weight-bearing;
  3. 'm'  — MOTION. Keep the robot untouched for ~2 s: the runner yaw-aligns the
            IMU at MOTION entry; entering MOTION while it moves/leans rotates
            EVERY target transform (Gate 3 bug #9).
  abort = 'p' at ANY moment (PASSIVE, zero gains).
```

- **Robot placement**: 0.8 m behind the table edge (`robot_start_x = −0.8`). The
  trained swing is a WALK-AND-STRIKE that lunges ~0.5–0.8 m forward — from −0.8 the
  lunge ends ~0.15–0.35 m short of the table. Starting at −0.5 puts the lunge INTO
  the table.
- **One clean return per rally point.** After the return the robot recovers on the
  bounded policy hold, then parks on the STATIC stand at its NEW position; the
  adaptive x_hit clamp ([0.0,0.35]) rejects serves it would have to lunge into the
  table for. Walk the robot back to the start spot between points (`p` → reposition
  → `s` → `m`; yaw re-aligns at each MOTION entry). Continuous rally without
  repositioning needs the post-strike homing behavior — a TRAINING item, not
  deployed.
- **Serving instructions for the human** (the policy needs ≥1.0 s of warning):
  - slow, HIGH lobs — apex ≥ 1.3 m, total flight to the robot ≥ 1.3 s. Hard/flat
    serves are stood out as `too_late` by design.
  - serve into the FOREHAND half first (robot's right); backhand works in the gates
    but has less closed-loop mileage.
  - aim the bounce mid-table; balls that would cross the robot's plane outside the
    planner window x∈[0.0,0.35] or the runner's reachability gate (base-rel
    x∈[0.20,0.90], |y|≤0.85, z∈[0.55,1.40], speed≤3.5) are gate-rejected (the
    robot just stands — that is the SAFE outcome, not a bug).
- **Failure = stand.** Every rejection path (late ball, unreachable target, planner
  flutter, fallen-robot guard) converges to a held stand, not a wild swing.

### 9.4 Honest status on hardware

The `--planner` C++ path is code-complete and closed-loop-verified in the AGI sim
(2026-07-04, §7/§9.6: engage → full swing → bounded recovery → sticky static stand
→ re-engage, zero falls). Still not exercised anywhere: the ros2 plugin on the
actual MDU (G2) and real mocap noise/latency through the engage logic — treat the
first hardware session as shadow-heavy (`h` before `m`, §9.2). The python
wbc_runner chain that used to back this section was RETIRED 2026-07-04 (§0); the
C++ runner reuses its planner and engage semantics.

### 9.5 Fallback demo (runs TODAY): scripted-mode choreography

If G1/G2 aren't closed by demo day: the proven C++ runner in SCRIPTED mode with an
operator on the keys, human serving for THEATER (ball not tracked):

```bash
# MDU (after 9.1 ship):
cd /agibot/a3_deploy
export A3_TRANSPORT=iceoryx
taskset -c 4-7 ./run_a3_pingpong.sh --start passive --gain-scale 0.4 --swing-rest 1.5
# stage up: p -> s (stand) -> m (MOTION); raise gain [ / ] once stable.
# operator presses f/b to match the server's side, 1 to swing; the 2026-07-04 fixes
# make the keys safe: f/b mid-swing is QUEUED to the next windup (no more OOD snap),
# a fall drops to PASSIVE automatically (fall guard), 's' mid-swing blends.
```

Honest framing if asked: scripted-target swings, not ball tracking. The mocap →
planner stack can run alongside on the laptop (stages 1–2 above work laptop-side —
only the ROBOT link needs the MDU) to show live `/racket/command` on a screen.

### 9.6 The official C++ `--planner` path (2026-07-04) — autonomous demo without the ROS chain

**What it is.** The deploy binary (`a3_deploy_onnx_ref_pingpong`) gained a live-input
mode built exactly per AGI's RobotIOBackend adaptation guide: two extra AimRT
subscriptions in `A3AimrtBackend` (registered only when `--planner` sets the callbacks,
mirroring the stock teleop hook) feed a C++ port of the (retired) python wbc_runner engage state machine
inside `PpPolicy`. The swing itself is executed by the SAME clip-clock/completion/latch
machinery Gate 2 proved. Everything below the target source is unchanged — obs build,
ONNX, decode, joint map, PD, watchdog, safety guards.

**Input contract** (both `std_msgs/Float64MultiArray`, RELIABLE, over the AimRT ros2
backend; hope_planner publishes both when `publish_flat_cmd:=true`, the default):

```text
/racket/command_flat  [schema=1, valid(0/1), swing_sign(+1 fh / -1 bh / 0 fallback),
                       px,py,pz, vx,vy,vz, time_to_strike, strike_time,
                       frame_code(0=world/table, 1=base_link)]        (≥11 doubles)
/a3/base_pose_flat    [schema=1, valid(0/1), x,y,z, qw,qx,qy,qz]      (≥9 doubles)
                       ← the robot base in the SAME frame as the racket target
                       (arena: /UCB_P1/pose + marker_to_base_xyz; sim: /sim/a3/pelvis_pose)
```

**Engage semantics** (defaults, all CLI-tunable): a fresh
VALID command engages only if it passes `--cmd-timeout 0.5` staleness,
`--invalid-grace 0.25` planner-flutter, `--engage-min-tts 1.0`, base-standing
(`base z ≥ 0.7`), and the reachability gate (base-rel x∈[0.20,0.90], |y|≤0.85,
z∈[0.55,1.40], speed≤3.5). On engage: side locks, target freezes, the ENGAGE tts
seeds the swing clock (`clock tts0=min(tts,max)` in the log — the strike meets the
ball), the clip runs once to the end (single-swing). Planner flutter/invalid
mid-swing is IGNORED (frozen target); the aborts are the safety guards and the
operator's `p` key. Localization defaults to `external_base` (live mocap base
position + yaw-aligned IMU attitude); a stale base stream (>0.2 s) blocks every new engage and
holds the last measured base in the observation with a loud warning. A swing already committed is
allowed to finish rather than snapping the clip mid-motion; strict Final certification permits no
unrequested stale-base warning.

For ONNX files carrying `hitter_pure_runtime_contract=rally_final_v1`, engage is two-stage. A valid
prediction first becomes a pending station while the clip clock stays at windup; the static stand
blends to the level-0 policy in 0.15 s, which receives the fixed-reach racket target and station delta.
The runner releases the swing only after `|dx|<=0.10 m`, `|dy|<=0.10 m`, filtered planar speed
`<=0.20 m/s`, and a 0.12 s continuous dwell. It also enforces the baked absolute station box and
station-step maximum (`metadata hi + 0.05 m`). If readiness arrives too late for the clip windup, the
ball is missed safely; the robot never arms while still moving. These values are adjustable with
`--ready-x-max`, `--ready-y-max`, `--ready-speed-max`, `--ready-dwell`,
`--station-takeover-blend`, and `--station-step-margin`; `--no-station-ready` is diagnostic-only and
cannot be used for Final certification.

**Level-0 policy: static stand except a bounded recovery window.** The model's
level-0 policy hold has only **~5 s of margin** (Gate 2.5, §6) — the runner therefore
NEVER parks on it: before the first engage it publishes the STATIC official stand
(the Python `_stand` design); after a completed swing it runs the policy hold just
long enough to actively balance out of the follow-through (`hold_recover_s` 2.5 s —
a static stand cannot do this part), then, quiescence-gated (upright + still, forced
by +3 s), blends to the STATIC official stand and stays there (sticky) until the
next engage. Verified: two full cycles incl. a re-engage FROM the static stand.
**Demo cadence**: serve every ~4-6 s (the policy is most stable swinging or briefly
holding), or estop (`p`) + reposition after each return per §9.3 — never rely on
more than ~3 s of autonomous policy hold.

**Rehearsal in the AGI sim (do this before hardware):** this is exactly Gate 3 —
§7 (one-command headless conductor + manual viewer variant, incl. the drag-0.05
verification physics and the reset-into-armed-stand mechanics).

**Hardware bring-up (MDU)** — after §9.1 ship: follow §9.2 (mocap → planner+flats
→ `--planner --dry-run` G2 gate → staged start + shadow `h` → ARM ritual §9.3 →
`m`). The G1 items (numpy [present] + vrpn_mocap [needs libVRPN] on the HDU, MCServer IP-routable) are in
§9.0.

**Status (2026-07-04, evening — closed-loop VERIFIED headless):** the full chain
fake_ball → REAL hope_planner (sim profile + flats) → C++ `--planner` runner
(external_base from the flats) → AGI MuJoCo sim ran end-to-end, repeatedly:
`[pp engage] forehand locked ... tts=1.25s (clock tts0=1.25s)` → full swing →
`swing complete` → post-swing policy hold → (in the faster-cadence run) a SECOND
engage from the walked position. Verified: both flat subscriptions, external_base
localization, every engage gate (no_command/stale/planner_invalid/too_late/base_low/
target_gate — the `[pp gate] REJECT` line prints the offending values), engage-time
tts transfer into the swing clock, side selection, single-swing completion, rest.
Bugs found+fixed during verification: engage originally ignored the planner tts
(strike always fired 1.30 s after engage — now `clock tts0=min(tts,max)`); the
pre-FIRST-engage hold now publishes the STATIC official stand (running the policy's
level-0 hold cold was never a validated regime and knelt the robot within ~2 s; the
retired python runner's `_stand` did the same); base_link-frame commands rotate velocity too;
stale-oracle engage is blocked. Residual (model-margin, NOT input-path): the
post-swing policy hold degrades after ~5-10 s and a second swing from the walked
position can fall — consistent with §9.3's demo spec (ONE clean return per placement,
reposition between points; continuous rally = the known post-strike-homing TRAINING
item). Still pending: ros2_plugin load on the actual MDU (the G2 gate above) and real
mocap noise/latency.

**Headless-harness mechanics** (baked into `scripts/pp_planner_conductor.py` /
`scripts/pp_gate25.py` — read this only when modifying them): the robot must be stood
by resetting INTO an armed PD_STAND — per attempt: publish a `mode:1` SimReset
(keyframe + `set_base` to place the robot), wait ~0.15 s, press `s` (official gains
catch it inside the ~0.5 s limp-buckle window), verify standing ~6 s, then `m`.
Arming `s` on a LYING robot instead trips the fall guard → PASSIVE → every later
reset collapses limp. The runner's keys need a real pty headless (`isatty` gate) —
`pty.openpty()` + `Popen(stdin=slave)`. Never let a reset fire after MOTION entry
(teleports the robot mid-hold = the reset-during-MOTION trap; use ONE persistent
rclpy publisher — per-call `ros2 topic pub --once` cold discovery can delay a reset
into the MOTION window, observed).

### 9.7 ★ FULL MOCAP POLICY TEST — copy-paste walkthrough (2026-07-07)

The end-to-end autonomous test: a human serves → mocap tracks the ball + robot base → the planner
computes a racket target → the C++ runner engages and swings → the robot returns. **Four roles, all
on `ROS_DOMAIN_ID=232`.** The whole DDS transport (laptop→HDU→MDU) was verified 2026-07-07; the
runner engage/swing was verified via hand-fed flats (§9.2b). What remains is the venue mocap config
and the physical bring-up.

```
  MCServer 192.168.10.100                LAPTOP (hope box)          HDU                    MDU
  (Ball/UCB_P1/UCB_P2 rigid bodies) --VRPN--> relay --DDS--> planner --DDS--> runner
                                       (only box that            (flats over 10.42.10.x)  (--planner)
                                        reaches the MCServer)                              --iceoryx--> HAL --> motors
```

**Pre-flight** (one-time; ✅ = verified this session, 🏟 = venue-day):
- ✅ MDU: `model_17400_hitter177.onnx` runner shipped; G2 ros2-plugin load verified (§9.-1).
- ✅ HDU: `hope_planner` built (`colcon build --packages-up-to hope_planner`, §9.1b).
- ✅ Laptop: `hope_bringup`+`vrpn_mocap` built in the `hope` box; laptop reaches the MCServer + HDU.
- 🏟 MCServer streaming rigid bodies named **`Ball`/`UCB_P1`/`UCB_P2`**, world-calibrated (net center ≈ (1.37,−0.7625,0)) — G4/G5.
- 🏟 Robot **0.8 m behind the table on the forehand half, facing +x, square** (§9.3); `marker_to_base_xyz` measured — G8.

---

**STEP 1 — Mocap bridge (LAPTOP, in the `hope` distrobox — ⚠ NOT the HDU)**
```bash
distrobox enter hope
source /opt/ros/jazzy/setup.bash
cd ~/workspace/HOPE/hope_ws && source install/local_setup.bash
export ROS_DOMAIN_ID=232
ros2 launch hope_bringup avatar_pro_hope_bridge.launch.py server:=192.168.10.100
```
Verify in a 2nd laptop shell (`distrobox enter hope`; source; `export ROS_DOMAIN_ID=232`):
```bash
ros2 topic list | grep vrpn_mocap        # vrpn topics present = VRPN connected
ros2 topic hz /poses                      # ball ≥240 Hz (publishes ONLY while ball data flows)
ros2 topic echo /UCB_P1/pose --once           # MUST be METRES + mocap frame: |x|,|y| < 3 and z ≈ 0.15
                                          #   (marker height ABOVE THE TABLE SURFACE — the G5 mocap
                                          #   origin is ON the surface); net-center marker ≈ (1.37,−0.7625,0)
# Position values in the HUNDREDS = the mocap streams MILLIMETRES (this venue does) — covered by
# avatar_pro_vrpn.yaml `position_scale: 0.001` (relay-side mm→m, field 2026-07-07). Without it the
# planner's solver diverges on the first ball (FloatingPointError) — now crash-guarded, but blind.
> `/vrpn_mocap/...` present but `/UCB_P1/pose` empty → the MCServer isn't streaming rigid bodies named
> `Ball`/`UCB_P1`/`UCB_P2` (fix on the mocap software — G4).
> Notes (field 2026-07-07): (a) run ONE bridge instance — a duplicate launch doubles every topic
> (two publishers on /UCB_P1/pose) and muddies rates; `pgrep -f avatar_pro_hope_bridge` before launching.
> (b) vrpn topics carry SENSOR-SUFFIXED names here (`/vrpn_mocap/UCB_P1/pose8`, `/vrpn_mocap/Ball/pose5`,
> plus a huge `/vrpn_mocap/MCAvatar/pose#####` flood) — that is normal, the relay discovers suffixed
> topics. (c) THE BALL CHECK: with the ball VISIBLY on the table,
> `ros2 topic hz /vrpn_mocap/Ball/pose5` and `/ball/point` must both tick. `/poses` is published ONLY
> when ball data flows (relay `_emit_ball` → `_publish_poses`) and the planner takes the ball FROM
> `/poses` — no ball stream ⇒ planner NEVER emits a racket command (runner `ts=-1` forever), even
> though `/UCB_P1/pose` is fine. Silent `Ball/pose5` while the ball is visible = CMTracker not tracking
> the Ball rigid body (re-enable it / check markers) — G4.

**STEP 2 — Planner (HDU)**
```bash
ssh agi@192.168.120.249                    # password 1 (wifi jump; NOT the ProtonVPN 172.16.238.4)
source /opt/ros/jazzy/setup.bash
source ~/hope_ws/install/local_setup.bash
export ROS_DOMAIN_ID=232
export ROS_LOCALHOST_ONLY=0                # ⚠ the HDU LOGIN env (aima load-env) sets this to 1 —
                                           # localhost-only traps the planner on the HDU even with
                                           # the right domain (field 2026-07-07, planner log said
                                           # "'localhost_only' is enabled")
echo $ROS_DOMAIN_ID $ROS_LOCALHOST_ONLY    # ⚠ MUST print "232 0" BEFORE starting the planner — wrong
                                           # domain OR localhost_only=1 = INVISIBLE to laptop+MDU and
                                           # the runner spams "[pp EXT-BASE] NO FRESH mocap base sample"
V10_X_HIT=0.08  # example only: measured desired station -0.50 + shared V10 reach 0.58
ros2 run hope_planner hope_planner_node --ros-args \
  --params-file ~/hope_ws/src/hope_planner/config/hope_planner.yaml \
  --params-file ~/hope_ws/src/hope_planner/config/hope_planner.hitter_pure.yaml \
  -p robot_pose_topic:=/UCB_P1/pose \
  -p marker_to_base_xyz:="[0.0, 0.0, 0.0]" \
  -p x_hit_follow_robot:=false \
  -p x_hit:=$V10_X_HIT \
  -p x_hit_bh_delta:=0.0 \
  -p swing_side_split_y:=-10.0 \
  -p swing_side_hysteresis_y:=0.0
# Replace marker_to_base_xyz with the G8 measurement unless the cluster is on the pelvis.
# V10 uses one shared 0.58 plane; the tracked overlay still contains V8's -0.15 BH delta, so the
# explicit 0.0 override is mandatory. The split override makes this first session backhand-only.
# Startup must say FIXED; /racket/command_flat data[2] must remain -1.0 before MOTION.
```
**MANDATORY verify** in a 2nd HDU shell (also `export ROS_DOMAIN_ID=232` + `export ROS_LOCALHOST_ONLY=0`)
— do NOT arm the robot until both pass:
```bash
ros2 topic hz /a3/base_pose_flat          # continuous ~15-40 Hz (robot pose alone drives it) — silence
                                          #   = planner not on 232 OR /UCB_P1/pose not reaching the HDU
ros2 topic echo --once /a3/base_pose_flat # data[4] (z) MUST read ≈0.91 (mocap 0.15 + policy_z_offset
                                          #   0.76 → floor frame; <0.7 = the runner base_low-blocks ALL
                                          #   engage). x,y in metres. Verified 2026-07-07: z=0.9125 ✓
ros2 topic info /poses                    # Subscription count ≥1 = the planner is really subscribed
# then toss a real ball THROUGH the air:
ros2 topic hz /racket/command_flat        # bursts while the ball flies
ros2 topic echo /racket/command_flat --once   # data[1]=1.0 valid, data[3] px∈[0.0,0.35], data[9] tts→0
# cross-check the planner's own env if in doubt:
#   tr "\0" "\n" < /proc/$(pgrep -f hope_planner_node | head -1)/environ | grep ROS_DOMAIN_ID
```
> **Frame contract (2026-07-07 fixes)**: the planner INGESTS the mocap frame (z=0 at the TABLE
> SURFACE — its bounce plane is z=0) and PUBLISHES the policy/floor frame (`policy_z_offset: 0.76`
> in hope_planner.yaml shifts both flats + /racket/command; sim yaml keeps 0.0 — sim feeds are
> already floor-origin). The planner is also crash-guarded: a diverging solve (garbage/mm feed)
> logs a throttled warning and degrades to no-command instead of killing the node.

**STEP 3 — Runner in SHADOW: verify engage on REAL serves, ZERO motion (MDU)**
```bash
ssh -J agi@192.168.120.249 agi@10.42.10.12
source /agibot/software/v0/entry/env/env.sh          # sets ROS_DOMAIN_ID=232
cd /agibot/a3_deploy && export A3_TRANSPORT=iceoryx   # ⚠ EXACTLY iceoryx
taskset -c 4-7 ./run_a3_pingpong.sh --planner --policy-native --dry-run --start shadow
```
Serve balls. Expect `[pp engage] forehand/backhand locked …` on good serves and `[pp gate] REJECT …`
(with values) on unreachable ones — the WHOLE pipeline minus motion. Tune serve style / robot
placement here until engage fires reliably. This is the safe gate before any motion.

**STEP 4 — Body-drive backend for real motion (⚠ robot HOISTED — see §9.2c)**
```bash
# MDU terminal HAL (robot SUPPORTED — stopping agibot_pm drops the controller holding it):
sudo systemctl stop agibot_pm
pgrep -a iox-roudi                                   # must stay alive (our iceoryx broker)
pgrep -a hal_ethercat && sudo pkill -TERM aimrt_main_hal
source /agibot/software/v0/entry/env/env.sh
cd /agibot/software/v0 && bash scripts/hal_ethercat/start_hal_ethercat.sh   # A3_P1D0; BLOCKS here
```

**STEP 5 — Runner in MOTION: real returns (MDU, 2nd terminal)**
```bash
cd /agibot/a3_deploy && export A3_TRANSPORT=iceoryx
./run_rally_v10_real.sh
# ONNX policy gains/actions + planner velocity; READY/yaw are telemetry, not release gates.
# ⚠ --official-stand is MANDATORY for free-standing: without it 's' uses --stand-kp 60
#   (vs the official knee ~2000) → the KNEES BUCKLE and the robot cannot stand
#   (field-confirmed 2026-07-07; main.cpp:709 picks the gain set on this flag).
# ARM ritual (§9.3): 's' → keep still ~2 s (IMU yaw-align) → 'm'. 'p' = PASSIVE abort.
```
**Human serving** (§9.3): slow HIGH lobs (apex ≥1.3 m, flight ≥1.3 s), into the **forehand half first**,
bounce mid-table. Hard/flat serves are stood out as `too_late` by design. **One clean return per
placement**, then `p` → walk the robot back to the start spot → `s` → `m` for the next point (no
post-strike homing yet). Restore the normal robot afterwards: `sudo systemctl start agibot_pm`.

> ⚠ **A STATIC ball does nothing — BY DESIGN.** Holding/placing a ball in front of the robot will
> NEVER trigger a swing: the planner fits a FLIGHT trajectory and predicts its crossing of the hit
> plane; a ball with ~zero velocity never crosses it → `valid=0` → the runner stands
> (`PLANNER: planner_invalid`/`no_command`). Engage also needs ≥1.0 s time-to-strike and a standing
> robot (`base z ≥ 0.7`). The ONLY trigger is a real served lob per the rules above.

**If it's not working:**
| symptom | cause → fix |
|---|---|
| STEP 2 `/racket/command_flat` never valid | mocap not flowing to the HDU → recheck STEP 1 `/UCB_P1/pose`, and `ROS_DOMAIN_ID=232` on BOTH laptop + HDU |
| **runner spams `[pp EXT-BASE] NO FRESH mocap base sample`** (field 2026-07-07) | the PLANNER shell missed `export ROS_DOMAIN_ID=232` → planner on domain 0, invisible to the runner. Decisive check: `tr "\0" "\n" < /proc/$(pgrep -f hope_planner_node\|head -1)/environ \| grep ROS_DOMAIN` — no line = domain 0. Restart the planner after the export. Engage is BLOCKED (by design) until the base stream returns |
| planner on 232 but STILL invisible / base flat silent (field 2026-07-07 #2) | `ROS_LOCALHOST_ONLY=1` from the HDU login env (`aima em load-env`) — planner boot log prints `'localhost_only' is enabled`. Fix: `export ROS_LOCALHOST_ONLY=0` before starting (STEP 2). The correct boot log says `'localhost_only' is disabled` |
| **runner `ts=-1` forever, robot stands through serves** (field 2026-07-07) | no racket command ever received → the BALL isn't streaming: `/vrpn_mocap/Ball/pose*` silent → `/ball/point`+`/poses` never publish → planner has no ball. Verify with the ball VISIBLE on the table (STEP 1 ball check); fix = CMTracker Ball rigid body tracking (G4) |
| **planner crashes `FloatingPointError: outgoing velocity solve diverged`** (field 2026-07-07) | garbage-scale measurements — the venue mocap streams MILLIMETRES. Fix = `position_scale: 0.001` in avatar_pro_vrpn.yaml (relay mm→m). The node is now crash-guarded (degrades to no-command), but the feed must still be fixed for real solutions |
| every engage `base_low`-rejected though the robot stands | base flat z < 0.7 → the mocap-frame z (surface origin, robot ≈0.15) reached the runner unshifted. Fix = `policy_z_offset: 0.76` in hope_planner.yaml (mocap/table frame → policy/floor frame); verify `data[4]≈0.91` per STEP 2 |
| STEP 3 no `[pp engage]`, only `no_command` | HDU planner not publishing / domain mismatch → `ros2 topic echo /racket/command_flat` on the HDU; confirm 232 |
| STEP 3 constant `[pp gate] REJECT z_w≈0.00` | ball dies before the plane → higher/slower serves (§9.3); or robot placement vs `x_hit` |
| STEP 3 engage only forehand, never backhand | runner splits fh/bh by base-rel target-y **sign** (y<0=fh); backhand needs the ball to arrive y≥0 |
| STEP 5 robot sags when you stop agibot_pm | it wasn't hoisted — that stop removes the holding controller (§9.2c safety) |
| **'s' but KNEES BUCKLE, cannot stand** (field 2026-07-07) | `--official-stand` missing → PD_STAND uses `--stand-kp 60` instead of the official knee ~2000. Re-run with `--official-stand`. If it buckled, fall guard likely tripped → PASSIVE (looks totally dead): support robot upright, then `s` again |
| stands, but legs SINK during the swing / tiny swing | `--gain-scale 0.4` also softened the policy LEG kp → add `--leg-gain-scale 1.0` (arms stay gentle, legs full); raise `--gain-scale` with `]` for full swings |
| ball placed/held in front → nothing | **by design** — static ball never crosses the hit plane, planner never emits valid. Serve a real lob (apex ≥1.3 m, flight ≥1.3 s, bounce mid-table) |
| boot log `seg_len={95,105}` or no 177 line | stale binary → re-ship the rockchip dist (§9.1) |

### 9.8 RETIRED — do not use the old four-terminal/manual planner recipe

The old §9.8 path was retired after the 2026-07-16 field session. It had three coupled defects:

- HDU-side `ros2 topic echo/hz` created extra DDS participants while the Python planner was already
  trying to solve at the full mocap rate. This could stall the HDU/group-control link.
- The MDU command changed the trained execution with `--demo`, split 0.4/1.0 gains,
  `--arm-hold-nominal`, and a 1.2 s static recovery handoff.
- The runner's hard READY/yaw admission and per-swing 0.5 s command blend could cancel or delay an
  otherwise valid ball-clock swing.

Use §9.9 only. In particular, do **not** run `ros2 topic echo`, `ros2 topic hz`, or a separate ROS
health subscriber on the HDU during the field session. The onboard planner now prints a 1 Hz
`HDU HEALTH` line from inside the existing process.

#### Mandatory build ownership

`ros2 run` executes the installed overlay, not the Python file under `src`. After any HDU source
sync/pull, rebuild and resource before starting §9.9.3:

```bash
ssh agi@192.168.10.150
source /opt/ros/jazzy/setup.bash
cd ~/hope_ws
colcon build --symlink-install \
  --packages-up-to vrpn_mocap hope_bringup hope_planner \
  --cmake-args -DBUILD_TESTING=OFF
source install/local_setup.bash
```

If only `hope_planner` changed and the bridge package was not touched, the minimum rebuild is
`colcon build --symlink-install --packages-up-to hope_planner --cmake-args -DBUILD_TESTING=OFF`,
followed by `source install/local_setup.bash` in the runtime shell.

An MDU C++ runner change is different: `colcon build` on the HDU does not update it. Re-run the
Laptop-host deployment in §9.9.2 so the Rockchip binary and wrappers are rebuilt and shipped.

### 9.9 ★ RECOMMENDED — onboard VRPN/planner, one command per Laptop terminal

This section is the only supported V10 field path. §9.8 is retained only as a retirement/build
notice; do not reconstruct or run its old manual Terminal A/B recipe. The Laptop is only an SSH
terminal and keyboard. In particular,
`taskset ... run_a3_pingpong.sh` executes on the MDU
(`ssh -tt -J agi@192.168.10.150 agi@10.42.10.12`)—not on the Laptop.

```text
CMTracker/VRPN ── group-control Wi-Fi ──> HDU (ssh -tt agi@192.168.10.150)
                                             ├─ vrpn_mocap
                                             ├─ avatar_pro_vrpn_relay
                                             └─ hope_planner (ROS_DOMAIN_ID=232, solve ≈30 Hz)
                                                        │
                                      Fast DDS only on 10.42.10.10 Ethernet
                                                        ▼
Laptop ── SSH/TTY only ──> MDU (ssh -tt -J agi@192.168.10.150 agi@10.42.10.12)
                              └─ C++ runner ── iceoryx ──> HAL ──> motors
```

**OptiTrack venue?** The same wrapper drives the alternative backend chain
`Motive ──NatNet UDP──> motion_capture_tracking ──> optitrack_mct_relay` with
`--mocap-backend optitrack --mocap-ip <MOTIVE_PC_IP> --update-freq <camera Hz>`
(defaults shift to `position_scale 1.0` and a derived planner `fit_window`; the
VRPN TCP preflight becomes ping + a post-launch `/UCB_P1/pose` rate probe). HOPE
topics, planner, and every MDU step below are identical. Details:
[run_hardware_session.md](run_hardware_session.md) "OptiTrack (Motive/NatNet) Backend" and the venue
fill-in sheet item (3-OT).

#### 9.9.1 Exact execution environments

| role | distrobox | directory | complete login from the Laptop |
| --- | --- | --- | --- |
| One-time build/deploy | **Laptop HOST; no distrobox**. The tracked builder manages its own Rockchip container. | `~/workspace/HOPE/a3_deploy/a3_deploy_example` | local shell; no SSH |
| Onboard mocap + planner | **HDU native ROS Jazzy; no distrobox** | `~/hope_ws` | HDU (`ssh -tt agi@192.168.10.150`) |
| EtherCAT HAL | **MDU native vendor environment; no distrobox** | `/agibot/software/v0` | MDU (`ssh -tt -J agi@192.168.10.150 agi@10.42.10.12`) |
| SHADOW / real runner | **MDU native vendor environment; no distrobox** | `/agibot/a3_deploy` | MDU (`ssh -tt -J agi@192.168.10.150 agi@10.42.10.12`) |

The `hope` distrobox remains useful for Laptop-side development/simulation, but it is not part of
this real-hardware runtime. The current group-control addresses and scale are:

- HDU: `agi@192.168.10.150`.
- CMTracker/VRPN: `192.168.10.100:3883`.
- MDU behind the HDU: `agi@10.42.10.12`.
- The V10 station-relative plane offset is `0.58 m`. Do **not** measure the initial absolute
  `V10_X_HIT` while the real runner is still PASSIVE: pressing `s` changes the pelvis/marker pose.
  The HDU wrapper freezes the final plane from the settled standing base after `s` without
  starting a `ros2 topic` or `ros2 service` CLI participant.
- The current site uses `position_scale=0.001` because CMTracker emits millimetres. Still confirm
  `Ball`, `UCB_P1`, `UCB_P2`, and metre-scale output on the first live session; use `1.0` only if a future
  CMTracker configuration already emits metres.

#### 9.9.2 One-time deployment — Laptop HOST (local shell; no SSH; no distrobox)

From one Laptop terminal, run one command:

```bash
cd ~/workspace/HOPE/a3_deploy/a3_deploy_example && \
  bash scripts/deploy_rally_v10_real.sh \
    --hdu agi@192.168.10.150 \
    --vrpn-ip 192.168.10.100 \
    --install-hdu-deps
```

The explicit `--install-hdu-deps` is for first setup only; omit it on later deploys. The detected
HDU is Debian Bookworm/aarch64 with vendor ROS Jazzy: `rclcpp`, `std_srvs`, `colcon`, CMake, and
NumPy are already usable, while its apt sources expose no `ros-jazzy-vrpn` or `libvrpn-dev`.
Therefore this path does **not** run apt or modify `/opt/ros/jazzy`. It fetches the official ROS
Jazzy `vrpn` release source at pinned commit
[`0b687036...`](https://github.com/ros2-gbp/vrpn-release/tree/0b68703610a8b017be5d3b8a773b7dfaa63103ab)
on the Laptop, syncs it to `~/hope_ws/src/_vendor/vrpn`, and lets `colcon` build it before
`vrpn_mocap`. Ubuntu Noble remains supported through the checked apt path.

For this Debian HDU, rerun the same command above; keeping `--install-hdu-deps` is harmless and
does not invoke `sudo apt`. Require this sequence in the output before continuing:

```text
[deploy-rally-v10-hdu] Debian Bookworm detected; using pinned workspace VRPN source
Starting >>> vrpn
Starting >>> vrpn_mocap
[HDU] native workspace verification PASS
```

Upstream VRPN CMake policy/deprecation warnings are non-fatal. Any `Failed <<<`,
`client_node is missing`, `not found`, or missing `native workspace verification PASS` is a hard
stop; do not proceed to the runtime terminals.

The script also checks HDU arm64/Jazzy and VRPN TCP/3883, syncs `hope_ws` without Laptop
`build/install/log`, builds the custom `vrpn_mocap + hope_bringup + hope_planner` natively on the
HDU, builds the exact Rockchip package, verifies
`model_10500_v10.onnx` SHA `fa427780...501ec5f` and the aarch64 ELF, then ships the package through
the full HDU jump host. It does not start ROS nodes, stop `agibot_pm`, or enter robot MOTION.

Both mocap backends are built into the HDU overlay by default: the colcon list
also includes the vendored `motion_capture_tracking` (+`_interfaces`) OptiTrack
driver, verified like `vrpn_mocap` (executable ldd + `NamedPoseArray`
typesupport + python import). `--skip-optitrack` drops it (escape hatch if the
OptiTrack driver ever fails to build — the AvatarPro deployment must never be
blocked by it); its build deps (`libpcl-dev libfmt-dev libboost-program-options-dev
libeigen3-dev python3-empy python3-lark`) are preflighted with an actionable
error on both platforms. For an OptiTrack venue run the same deploy with
`--mocap-backend optitrack --mocap-ip <MOTIVE_PC_IP>` so the network preflight
pings the Motive PC instead of requiring VRPN TCP/3883.

#### 9.9.3 Terminal A — HDU (`ssh -tt agi@192.168.10.150`); onboard bridge + locked planner

Run exactly one Laptop command and leave it running:

```bash
ssh -tt agi@192.168.10.150 \
  'cd ~/hope_ws && exec ./install/hope_bringup/lib/hope_bringup/run_rally_v10_hdu.sh \
    --vrpn-ip 192.168.10.100 --side both \
    --marker-to-base 0.0,0.0,0.0 --position-scale 0.001'
```

OptiTrack venue variant (same supervisor; scale/rate/fit_window defaults follow
the backend, NatNet liveness is proven by the post-launch `/UCB_P1/pose` probe):

```bash
ssh -tt agi@192.168.10.150 \
  'cd ~/hope_ws && exec ./install/hope_bringup/lib/hope_bringup/run_rally_v10_hdu.sh \
    --mocap-backend optitrack --mocap-ip <MOTIVE_PC_IP> --update-freq 360.0 \
    --side both --marker-to-base 0.0,0.0,0.0'
```

The script sources native Jazzy plus `~/hope_ws/install/local_setup.bash`, exports domain 232,
checks VRPN TCP and duplicate bridge/planner processes, then supervises the bridge, relay, and
planner together. Fast DDS is explicitly confined to the HDU-MDU Ethernet address `10.42.10.10`;
VRPN remains direct TCP to `192.168.10.100`. The estimator receives the full mocap stream while
Stage 2/3 solve is rate-limited to `solve_period_s=0.033` (about 30 Hz). If either managed process
exits, the other is stopped. It never starts the MDU.

Do not open another HDU shell for `ros2 topic echo/hz`. Read the planner's in-process line instead:

```text
HDU HEALTH poses=...Hz base=...Hz(age=...s) solves=...Hz valid=...Hz ...
```

Require ball/pose counters to keep increasing, `base age` to remain small, and solve rate to stay
near 30 Hz. `valid=0 Hz` while no ball is flying is normal. This startup has two independent outputs:

- `/UCB_P1/pose` comes from the bridge/relay and is available before planner calibration; running
  `hope_planner` is **not** what creates `/UCB_P1/pose`.
- `/a3/base_pose_flat` remains live so the MDU external-base input can be checked and the robot can
  enter PD_STAND. `/racket/command` and `/racket/command_flat` are fail-closed until Terminal A's
  `c` calibration succeeds.

`--side both` is the B1 automatic forehand/backhand selection. The planner publishes an explicit
`+1/-1` side selected from the intercept; the runner does not infer the side from racket velocity.

#### 9.9.4 Terminal B — MDU (`ssh -tt -J agi@192.168.10.150 agi@10.42.10.12`); SHADOW

Before changing controller ownership, run exactly one Laptop command:

```bash
ssh -tt -J agi@192.168.10.150 agi@10.42.10.12 \
  'cd /agibot/a3_deploy && exec ./run_rally_v10_shadow.sh'
```

The wrapper verifies the exact model/config/architecture, sets domain 232 and iceoryx, and executes
`--planner --policy-native --dry-run --start shadow`. It uses planner velocity and the same release
semantics as the real wrapper; it does not apply `--demo` or gain/action overrides. This SHADOW run
is an optional wiring diagnostic, not a Gate 3 battery. Then Ctrl-C this terminal. The SHADOW script
has no path that invokes the real script.

#### 9.9.5 Terminal C — MDU (`ssh -tt -J agi@192.168.10.150 agi@10.42.10.12`); HAL

Only after SHADOW passes and the robot is physically hoisted/supported, run one Laptop command and
leave it running:

```bash
ssh -tt -J agi@192.168.10.150 agi@10.42.10.12 \
  'cd /agibot/software/v0 && exec /agibot/a3_deploy/run_rally_v10_hal.sh --robot-supported'
```

The wrapper checks the MDU/vendor paths before stopping anything, requires the operator to type
`SUPPORTED`, stops `agibot_pm`, sends SIGTERM only to exact `aimrt_main_hal` PIDs, and starts the
vendor EtherCAT HAL. It never sends SIGKILL and never starts the policy runner.

#### 9.9.6 Terminal D — MDU (`ssh -tt -J agi@192.168.10.150 agi@10.42.10.12`); real runner

Run exactly one Laptop command:

```bash
ssh -tt -J agi@192.168.10.150 agi@10.42.10.12 \
  'cd /agibot/a3_deploy && exec ./run_rally_v10_real.sh'
```

This is a thin, interactive wrapper around the exact operator-selected command:

```bash
taskset -c 4-7 ./run_a3_pingpong.sh --planner --start passive --official-stand \
  --policy-native
```

The wrapper uses ONNX policy gains/actions, planner-solved velocity, and the ball clock. It does not
use `--demo`, gain scaling, arm nominal override, timed static recovery, or a per-swing motion blend.
READY/yaw/base-height/trained-support checks remain telemetry; they do not cancel a fresh strike.
The runner fall cutoff is also disabled in policy-native mode; the field safety operator owns the
physical cutoff. Fresh/finite/late-command checks and one-shot strike consumption remain, because
they prevent an expired command from being replayed after the ball has landed.

It starts in PASSIVE and cannot press keys itself:

1. Put the supported robot on the marked station, square to the table, then press `s` in Terminal D.
2. Leave it untouched for **2 seconds** in PD_STAND. The static stand path does not depend on
   `x_hit`; Terminal A continues streaming the corrected base pose during this step.
3. Switch to the already-running Terminal A and press lowercase `c` once. This writes a local
   `/tmp` request consumed by the existing planner process; it does not create another DDS
   participant. The planner takes the median of the latest `0.5 s` corrected-base window,
   rejects stale/fewer-than-10 samples or more than `0.01 m` X span, then atomically freezes
   `x_hit=base_x+0.58`. Require `success=1` and an `X-HIT FROZEN` line. If it reports
   `NOT CALIBRATED`, keep the robot still for 2 seconds and press `c` again.
4. Continue watching `HDU HEALTH`. No extra flat gate or `ros2 topic echo` process is started.
5. Return to Terminal D and press `m`. The field safety operator owns the physical cut-off.

The absolute `V10_X_HIT` is therefore measured once in the pose that matters—after `s`—and written
into the already-running planner; no ROS process or shell variable is restarted. Press `c` again
before `m` if the marked station or marker-to-base calibration was deliberately changed. A known
post-stand absolute plane can still be supplied using `--x-hit METRES`, but that bypasses the
startup calibration path and is not the recommended first-session path.

At steady state there are three Laptop windows: Terminal A (HDU stack), Terminal C (MDU HAL), and
Terminal D (MDU runner). Terminal B is a completed SHADOW gate, not a fourth live process.

#### 9.9.7 Stop and restore

Quit Terminal D with `q`, Ctrl-C Terminal C, and restore the default controller with one Laptop
command:

```bash
ssh -tt -J agi@192.168.10.150 agi@10.42.10.12 \
  'sudo systemctl start agibot_pm'
```

Then Ctrl-C Terminal A. Only after this complete manual path has fresh field evidence may the HDU
bridge/planner supervisor be considered for a boot service. Never configure the MDU runner to
auto-enter STAND or MOTION, and never auto-chain SHADOW to the real runner.

### 9.10 ★ V17-r1 `model_16600` fixed-station hardware trial

The full copy-paste field procedure is maintained in the standalone runbook:

- V17-r1 `model_16600` fixed-station qualification hardware trial (historical document; not included in this checkout)

Use that document as the sole procedure for this trial. It contains the frozen
model identity, venue addresses, NatNet/MODELDEF preflight, Laptop/HDU/MDU
build and launch steps, operator abort contract, and evidence collection.

Qualification means three successful returns observed by the field team, not
three attempts. The retired inline RallyV14 `model_19900` walkthrough has been
removed so its model and commands cannot be mixed with the V17-r1 trial.

## 10. Quick reference

| task | box | directory | command |
| --- | --- | --- | --- |
| train RallyV10 (fresh) | grasping | `hope_training/whole_body_tracking` | `bash scripts/launch_rally_v10_recovery.sh` — operator-owned; do not add `checkpoint_path` or restart the active run for eval/export tooling changes. |
| export RallyV10 recipe-7/v2 | grasping | same | `TASK=HOPEPingPongHitterPureRallyV10 FH=<v13-fh> BH=<v13-bh> bash scripts/export_onnx_hitter_pure.sh <run> <ckpt>`; first run the same command with `PP_EXPORT_PREFLIGHT_ONLY=1`. |
| Gate 1 Isaac (RallyV10) | grasping | same | Run `eval_deterministic.py` three times with `+vel_eval_distribution=core`, `planner`, and `mix`; production mix must print progress `1.0` and planner probability `0.75`. |
| Gate 1 MuJoCo (110-D Final) | host | same | `../.venv-motion/bin/python scripts/scoreboard_eval.py --onnx <policy> --label <name> ... --protocols deploy_faithful deploy_faithful_implicit` |
| sync + parity (`model_10500_v10`) | hope | `a3_deploy/a3_deploy_example` | `PYBIN=~/workspace/HOPE/hope_training/.venv-motion/bin/python bash scripts/sync_pingpong_model.sh "$V10_ONNX" model_10500_v10`; promoted/durable SHA must be `fa427780...501ec5f`. |
| build x86 / rockchip | hope / HOST | same | `bash scripts/build_a3_deploy_pkg.sh --arch x86_64\|rockchip --runtime-cfg ...pingpong.yaml` |
| AGI sim policy-only | hope | same | `bash scripts/pp_freebase_watch.sh --single-swing [--oracle-pelvis]` |
| AGI sim station-only **Gate3A (FinalV2)** | hope | same | `PP_VIEWER=0 bash scripts/pp_gate3a_station_only.sh` — 12 FH/BH lateral moves, production readiness, no ball/swing; V2 metadata and full-state foot/q-des checks fail closed. |
| AGI sim rally **Gate 3 (RallyV10 recipe-7/v2)** | hope | same | `bash scripts/pp_gate3_rally_v10.sh` — shared plane `0.58`, station x `0.44`, exact solver-checked 12 serves, component-aware velocity gate, `--demo`, no rescue, READY required. Requires the `rally_final_v2` runner banner. |
| AGI sim rally **Gate 3 (RallyV8/V9 only)** | hope | same | `bash scripts/pp_gate3_rally_v8.sh` — per-side planes `0.65/0.50`, station x `0.33`, its own solver-checked table. Never use it for V10. Bare `pp_gate3_rally.sh` is rejected. |
| AGI sim + planner loop (**LEGACY follow, 175/177 only**) | hope | same | `bash scripts/pp_planner_closedloop.sh` — FOLLOW-mode plane (adaptive). ⚠ Do NOT run an x-locked ONNX through this: follow-mode deletes the x anchor → drift-fall (the planner logs a `FOLLOW-MODE … UNSAFE for x-LOCKED` warning). Viewer variant in §7. |
| landing analysis (§7.5) | host | `hope_ws/src/hope_planner` | `python3 -m hope_planner.landing_mc` — on-table % from a checkpoint's exact errors (champion ≈ 98%; training thresholds only 67%) |
| onboard deploy: VRPN/planner + robot package | Laptop HOST, no distrobox | `a3_deploy/a3_deploy_example` | `bash scripts/deploy_rally_v10_real.sh --hdu agi@192.168.10.150 --vrpn-ip 192.168.10.100 [--install-hdu-deps]` (§9.9.2). |
| mocap + planner runtime (arena) | **HDU (`ssh -tt agi@192.168.10.150`)**, native Jazzy, domain 232 | `~/hope_ws` | `run_rally_v10_hdu.sh --vrpn-ip 192.168.10.100 --side both`; DDS stays on `10.42.10.10`, solve ≈30 Hz, and the in-process `HDU HEALTH` line replaces HDU `ros2 topic echo/hz` (§9.9.3/§9.9.6). |
| ship to robot | Laptop HOST → MDU (`ssh -tt -J agi@192.168.10.150 agi@10.42.10.12`) | `a3_deploy/a3_deploy_example` | Included in `deploy_rally_v10_real.sh`; the MDU destination is `/agibot/a3_deploy/`. |
| run on robot (scripted) | MDU (`ssh -tt -J agi@192.168.10.150 agi@10.42.10.12`) | `/agibot/a3_deploy` | `taskset -c 4-7 ./run_a3_pingpong.sh --start passive --legs-passive --gain-scale 0.4 --single-swing --arm-hold-nominal` (arm flag = 17400 pre-swing-hold twist cosmetics; g25-verified 10/10) |
| run `model_10500_v10` policy-native | MDU (`ssh -tt -J agi@192.168.10.150 agi@10.42.10.12`) | `/agibot/a3_deploy` | `./run_rally_v10_real.sh`; it executes `taskset ... --planner --policy-native --start passive --official-stand` with no demo/gain/arm/recovery overrides (§9.9.6). |
| transition matrix (Gate 2.5) | hope | `a3_deploy/a3_deploy_example` | `bash scripts/pp_gate25.sh [--oracle]` (§6: per-phase PASS/FAIL; **model_13200_footfix08 = 10/10 oracle** [= 12200+gait; needs the 110 idle-anchor fix; legacy Δ=0 idle = 8/9 P7 creep], **2/5 perfect_tracking** [110-D swings need external_base]; prior model_17400_hitter177 = 10/10 oracle / 3/4 perfect_tracking) |
| hardware planner+control | HDU (`ssh -tt agi@192.168.10.150`) planner + MDU (`ssh -tt -J agi@192.168.10.150 agi@10.42.10.12`) runner | §9.2 | domain 232 mocap → planner+flats → `--planner --dry-run` (G2 ✅) → shadow `h` → ARM ritual + `m` (§9.2-9.3) |
| hand-fed flats test (no mocap) | HDU (`ssh -tt agi@192.168.10.150`) publisher + MDU (`ssh -tt -J agi@192.168.10.150 agi@10.42.10.12`) runner | §9.2b | domain 232 `ros2 topic pub` for both flats → `--planner --dry-run`+`h` iterate → real `s`→`m` strike; wiring verified 2026-07-07 |
