# Ping-Pong: New Checkpoint → ONNX → AGI Deploy → MuJoCo → Hardware

> Current workflow checked on 2026-10-11: use
> [deploy_and_gate3.md](../../docs/operations/deploy_and_gate3.md) for complete
> packaging, current native simulator installation, policy checks, Gate 3
> execution/report reading and immutable-slot staging. Use
> [runtime_xbox.md](../../docs/operations/runtime_xbox.md) and the
> [first-deployment guide](../../docs/operations/foxglove_first_hardware_test.md)
> for managed hardware operation. The dated model-specific examples below are
> historical text, including their old paths, cloud aliases, manual HAL startup,
> reset/perfect-tracking commands and claimed results. They are not current
> launch instructions or results from this documentation review.

> **Historical HitterPingPong workflow (2026-08-03):** use the Unitree-style,
> two-file policy directory documented in
> [HITTER_PINGPONG_UNITREE_DEPLOY.md](HITTER_PINGPONG_UNITREE_DEPLOY.md).
> It replaces the checkpoint-specific copy/`onnx.model_path`/C++ editing steps
> below for the Build task.  The remaining sections are historical guidance for
> older 175/180-D deploy-parity policies.

Historical workflow for taking a freshly trained HOPE ping-pong checkpoint (`model_*.pt`)
to the AGI A3 robot. Every command block says **which box** and **which directory**.
Detail lives in the sibling docs — this file is the spine:
[PINGPONG_DEPLOY_ALIGNMENT.md](PINGPONG_DEPLOY_ALIGNMENT.md) (contract),
[MUJOCO_VALIDATION_RUNBOOK.md](MUJOCO_VALIDATION_RUNBOOK.md) (sim harness),
[SIM_DEPLOY_REHEARSAL.md](SIM_DEPLOY_REHEARSAL.md) (loc-mode A/B/C),
[HARDWARE_BRINGUP_CHECKLIST.md](HARDWARE_BRINGUP_CHECKLIST.md) (robot).

---

## 🤖 AGENT PROMPT — read this first if you are an AI agent (or a human in a hurry)

> **For current HitterPingPong, a policy is `params/deploy.yaml` plus
> `exported/policy.onnx`.** Follow
> [HITTER_PINGPONG_UNITREE_DEPLOY.md](HITTER_PINGPONG_UNITREE_DEPLOY.md).
> The old checklist below applies only to archived 175/180-D policies whose
> checkpoint-dependent values were hardcoded in C++/configs. For those policies:
>
> 1. **Stage the model**: `cp <run>/exported/policy.onnx assets/a3_runtime/models/<name>.onnx`
>    (never copy into `dist/` by hand — builds wipe it).
> 2. **Point the config at it**: `src/a3/a3_deploy_onnx_ref/config/a3_runtime_config.pingpong.yaml`
>    → `onnx.model_path`. This is the ONE key that names the deployed model.
> 3. **Re-sync the scripted targets**: `src/a3/a3_deploy_onnx_ref/include/a3_pingpong/pp_policy.hpp`
>    → `racket_pos_w_clip` / `racket_vel_w_clip` must sit INSIDE the new model's training
>    boxes (`cfg/task/HOPEPingPongDeployParity.yaml` `pos_range_per_clip`/`vel_range_per_clip`;
>    mirrored in `mujoco_eval_onnx.py` POS/VEL_RANGE_PER_CLIP). These are compile-time
>    constants with no CLI override — an out-of-box target is an OOD command obs every tick.
> 4. **Check what the runner reads from metadata vs hardcodes**: kp/kd, `clip_seg_lengths`,
>    `clip_strike_phases`, obs dim (175/180) all come from the ONNX at load (fail-fast on
>    bad gains; loud WARN + legacy-v1 fallback on missing clip layout). `strike_period` /
>    `strike_lead_frac` / `dt` are hardcoded — verify the periodic-wrap warning at startup.
> 5. **Rebuild BOTH dists** (x86_64 now, rockchip before the robot) — the packager stages the
>    model + rewrites the packaged yaml automatically from the `--runtime-cfg` you pass.
> 6. **Re-run the gates**: `run_parity.sh` (C++⇔Python ONNX parity), then the AGI-MuJoCo
>    free-base test (Stage 4). AGI's rule: **falls in MuJoCo = falls on the real robot.**
> 7. **Per-checkpoint verdicts do not carry over**: swing-direction clearance (forehand /
>    backhand), loc-mode requirements, and guard thresholds are re-decided per model from
>    the gates — do not inherit "forehand-only" or "never press b" from an older model.
>
> Also re-sync when the TRAINING boxes change without a new ONNX name (re-plane), and
> update `hope_ws/src/hope_planner/hope_planner/imitate_presets.py` if the ROS fake-planner
> path will be used (its SafetyLimits clamp is per-model).

---

## 🔴 HISTORICAL DEPLOY (2026-07-03)

**Staged policy: `model_9000_replane.onnx`** = `model_9000.pt` from run
`hope_training/whole_body_tracking/logs/rsl_rl/agibot_a3_hope_deploy_parity/2026-07-03_02-01-17`
(audit config: multi-swing episodes + HER replay + blade re-plane boxes).
175-D deploy_parity / 31-act, all-implicit training, v4 clips (seg `139,132`, strike phases
`0.470,0.333` in metadata).

* Training-side deploy-faithful MuJoCo gate: **7/7 swings incl. backhand 3/3, 0 falls** —
  the first model to pass backhand. Swing-direction clearance for hardware still comes
  from the Stage-4 AGI-MuJoCo gate below.
* 🔴 **2026-07-03 AGI-sim verdict: model_9000 FAILS under `perfect_tracking`** (repeated
  backward stepping → fall). Root cause (triaged, numerically confirmed): the run trained
  on **raw v4 clips that were never re-grounded** (baked frame-0 pelvis yaw +82°/+86°;
  registry `:latest` silently replaced the re-grounded hopex lineage) while the target
  boxes assume +X grounding → the policy learned to TURN ~84° and WALK 0.4–0.65 m to its
  target. That behavior is in-distribution and passes the training gate (oracle poses),
  but under `perfect_tracking` the base obs is pinned to the reference pelvis → the
  footwork is invisible → open strike loop → falls. **The ~84° `motion_anchor_ori_b` at
  engage is NOT a bug** (training's actor obs uses the raw clip quat too) — do not
  "re-anchor" the refs in the runner.
* ⚠ **The `--oracle-pelvis` A/B is only real if the oracle bridge is running**
  (`scripts/run_oracle.sh` in a second terminal). Without it the runner silently degraded
  to perfect_tracking (fixed 2026-07-03: it now repeats a loud `[pp ORACLE] NO FRESH
  SAMPLE` warning and shows `fresh=0`). A model_9000 oracle test that shows `fresh=1` and
  passes proves the policy; hardware still needs mocap for that footwork.
* **Recommended path: retrain with the re-grounded clips** (pin `registry_name` /
  `motion_file` to the hopex/+X lineage — frame-0 yaw exactly 0) → an in-place striker
  with the audit-config benefits, deployable under perfect_tracking like p4. Pre-flight
  warnings now fire in train/gate/runner when a clip is not re-grounded.
  `model_p4_deployparity.onnx` (in-place, forehand-only verdict) stays the hardware
  fallback; run p4 with `--no-imu-yaw`.
* Defaults changed 2026-07-03: `--use-imu-yaw` is now ON by default (engage-relative yaw
  via yaw-align = what training saw; revert `--no-imu-yaw`), and the runner warns at
  startup when periodic mode would wrap mid-follow-through — **always run v4-clip models
  with `--single-swing` or `--swing-rest S`**.

---

## 0. The three environments

| where | what | stages |
|---|---|---|
| distrobox **`grasping`** | Isaac Lab GPU box (training env) | 1 export · 2 training-side gate |
| distrobox **`hope`** | ROS2 Jazzy + g++13 build box | 3 sync/build/parity · 4 AGI MuJoCo |
| **host** shell | has Docker (the `hope` box does not) | 5 rockchip cross-build |
| robot **MDU** (aarch64, via HDU jump host) | onboard compute | 6 hardware |

Two gotchas that bite every time:
* the `hope` box has a broken `.bashrc` → **every new shell**: `source /opt/ros/jazzy/setup.bash`
* the `grasping` training env must be sourced **once per terminal**: `source setup_train_env.sh`

---

## Stage 1 — Export `.pt` → ONNX (box: `grasping`)

```text
distrobox enter grasping
cd ~/workspace/HOPE/hope_training/whole_body_tracking
source setup_train_env.sh
RUN=logs/rsl_rl/agibot_a3_hope_deploy_parity/2026-07-03_02-01-17   # your run dir

hope_isaac_py scripts/play.py task=HOPEPingPongDeployParity algo=ppo headless=true num_envs=2 \
  checkpoint=$RUN/model_9000.pt
# -> $RUN/exported/policy.onnx (~1.3 MB). play.py may not exit after the export —
#    once the file exists it is complete; Ctrl-C is safe (or use scripts/export_onnx_p4.sh's
#    setsid+poll+kill pattern for unattended runs).
# optional, only needed for dither (noise>0) evals:
#   hope_isaac_py scripts/make_std_sidecar.py --checkpoint $RUN/model_9000.pt
```

**Rules:**
* `task=HOPEPingPongDeployParity` — the task config supplies BOTH clips
  (`registry_name`+`registry_name_2`). **A single-clip export bakes a forehand-only
  reference and the backhand freezes at deploy.** Never override the task to one clip.
* Export ONLY through `play.py` (contract validation + metadata baking). The exporter
  refuses to bake non-positive kp/kd (the 07-02 limp-joint bug class).

**Contract check (PASS criteria):**

```text
python scripts/inspect_a3_deploy_contract.py --onnx $RUN/exported/policy.onnx
```
* inputs `obs[1,175]` + `time_step[1,1]`; output `actions[1,31]` (+ ref tensors)
* metadata: `actor_obs_contract=deploy_parity`, `clip_seg_lengths=139,132`,
  `clip_strike_phases=0.4700,0.3330`, 31 `joint_names`, **strictly positive** kp/kd
* missing clip metadata = the runner silently drives the LEGACY v1 clock → the 07-02
  stale-clock bug. Do not proceed.
* **v7-clip generation (registry `:latest` since 2026-07-11)**: checkpoints trained on the
  v7 clips (104 frames @ 50 Hz each) must bake `clip_seg_lengths=104,104` and
  `clip_strike_phases=0.3689,0.3592`. Seeing `139,132` on a v7-trained export means the
  exporter was fed the STALE hopex npz (CLIP-BAKE gotcha) — re-export with the v7 motion
  files (`motions/preprocessed/hope_{forehand,backhand}_v7.npz`). NOTE: the RallyFinalV3
  recipe is fail-closed (`motion_contract.approved: false`) and its validator currently
  REJECTS the v7 clips — see `V7_CLIP_MIGRATION_TODO.md`. (The v6 generation, 55-frame
  clips, was abandoned the same day without training.)

---

## Stage 2 — Training-side deploy-faithful gate (box: `grasping`, no Isaac needed)

The binding pre-deploy behavioral check, run against the exported ONNX with the deployed
episode protocol (nominal-stand start, hold → full clip → rest, no teleports, fall-only
termination):

```text
cd ~/workspace/HOPE/hope_training/whole_body_tracking
.venv-motion/bin/python  scripts/mujoco_eval_onnx.py \
  --onnx $RUN/exported/policy.onnx \
  --motion-files "artifacts/hope_forehand:v4/motion.npz" "artifacts/hope_backhand:v4/motion.npz" \
  --noise-scales 0.0 --pd-mode implicit --deploy-faithful --steps 1500
```

**PASS:** `falls = 0`, per-clip `completion_rate = 1.0`, per-clip `composite_succ_exact`
high (model_9000: 1.0/1.0). The script's default strike phases / target boxes already
match the current generation — for OLDER models pass `--strike-phase-per-clip` explicitly.

---

## Stage 3 — Sync into the deploy tree + build + parity (box: `hope`)

The agent-prompt checklist, as commands (steps 3–4 only when boxes/targets changed):

```text
distrobox enter hope
source /opt/ros/jazzy/setup.bash
cd ~/workspace/HOPE/a3_deploy/a3_deploy_example

# 1. stage the model (durable asset; never hand-copy into dist/)
cp ~/workspace/HOPE/hope_training/whole_body_tracking/$RUN/exported/policy.onnx \
   assets/a3_runtime/models/model_9000_replane.onnx

# 2. point the runtime config at it (already done for model_9000_replane):
#    src/a3/a3_deploy_onnx_ref/config/a3_runtime_config.pingpong.yaml -> onnx.model_path
#    (keep `backend: no_rknn` — the pingpong runner is CPU-onnxruntime on every arch)

# 3. scripted targets in pp_policy.hpp racket_{pos,vel}_w_clip: must be inside the model's
#    training boxes (already re-synced to the 2026-07-02 blade re-plane for model_9000).

# 4. build x86_64 — auto-stages model+config into dist/a3_deploy_x86_64/
bash scripts/build_a3_deploy_pkg.sh --arch x86_64 \
  --runtime-cfg src/a3/a3_deploy_onnx_ref/config/a3_runtime_config.pingpong.yaml

# 5. verify the staging + fresh binary
grep model_path dist/a3_deploy_x86_64/config/a3_runtime_config.pingpong.yaml  # -> models/model_9000_replane.onnx
ls -la dist/a3_deploy_x86_64/models/ dist/a3_deploy_x86_64/a3_deploy_onnx_ref_pingpong

# 6. C++ <-> Python ONNX parity (dim-aware since 2026-07-03; MODEL defaults to the new onnx)
bash scripts/pingpong_parity/run_parity.sh          # expect: PARITY: PASS (max|delta| ~1e-6)
```

**Stale-binary / stale-model check (one canonical marker):** at runner startup you MUST see
`[pp] clip layout from ONNX metadata: seg_len={139,132} strike_phase={0.470,0.333}` —
that one line proves both a fresh binary (metadata path exists) and the fresh model
(v4 seg lengths). Absence, or `seg_len={95,105}`, means stale binary or wrong ONNX.
(v7-trained checkpoints, 2026-07-11+: the marker must read `seg_len={104,104}
strike_phase={0.369,0.359}` instead.)

---

## Stage 4 — AGI official MuJoCo test (box: `hope`) ← **the pre-hardware gate**

Never modify AGI's sim. iceoryx gotcha: a stale RouDi from another user leaves a
`/dev/shm/iox1_0_u_<group>` segment your apps can't join — clean between runs (the watch
script does it for you).

**One-command free-base gate** (sim + auto warmup resets + interactive runner):

```text
distrobox enter hope
# forehand, hardware-realistic localization (perfect_tracking is the runner default):
bash ~/workspace/HOPE/a3_deploy/a3_deploy_example/scripts/pp_freebase_watch.sh --single-swing

# REQUIRED SECOND RUN for model_9000 — oracle base pose (the mode its footwork needs):
bash ~/workspace/HOPE/a3_deploy/a3_deploy_example/scripts/pp_freebase_watch.sh --single-swing --oracle-pelvis
```

* Keys once it swings: `1`=swing · `0`=hold · `f`/`b`=forehand/backhand · `,`/`.`=slower/faster · `q`=quit.
* `--single-swing` is REQUIRED for v4-clip models: the periodic clock wraps 0.9 s after the
  strike but v4 follow-throughs are 1.46/1.74 s — the wrap SNAPS the reference
  mid-follow-through and topples the free base (the runner now warns at startup).
* Manual 3-terminal flow, obs-CSV capture and the A/B/C loc-mode rehearsal:
  see [MUJOCO_VALIDATION_RUNBOOK.md](MUJOCO_VALIDATION_RUNBOOK.md) and
  [SIM_DEPLOY_REHEARSAL.md](SIM_DEPLOY_REHEARSAL.md). Obs health: the live `[obs]` block, or
  `python3 scripts/analyze_obs_log.py /tmp/pp_obs.csv` (175-D aware since 2026-07-03).

**PASS per swing direction** (re-decided for EVERY checkpoint):
* N≥5 clean swing cycles, no falls, no guard trips (`[pp] squat guard` / tilt), sync_miss 0.
* **model_9000 decision matrix**: `--oracle-pelvis` PASS + `perfect_tracking` PASS → clear
  for hardware as-is. Oracle PASS but perfect_tracking FAIL → the walk-and-strike footwork
  needs real localization → hardware BLOCKED until mocap; fall back to p4 (add
  `--no-imu-yaw` for p4). Oracle FAIL → back to training, do not ship.

---

## Stage 5 — Rockchip cross-compile (HOST shell — needs Docker; the `hope` box has none)

```text
cd ~/workspace/HOPE/a3_deploy/a3_deploy_example
# one-time: scripts/export_rockchip_sysroot.sh  (tarball already in thirdparty/)

bash scripts/build_a3_deploy_pkg.sh --arch rockchip \
  --runtime-cfg src/a3/a3_deploy_onnx_ref/config/a3_runtime_config.pingpong.yaml

# verify before shipping:
file dist/a3_deploy_rockchip/a3_deploy_onnx_ref_pingpong          # -> aarch64
grep model_path dist/a3_deploy_rockchip/config/a3_runtime_config.pingpong.yaml
strings dist/a3_deploy_rockchip/a3_deploy_onnx_ref_pingpong | grep -c "clip layout from ONNX"  # >=1
```

Since 2026-07-03 the standard rockchip path works for pingpong configs: the runtime cfg's
`onnx.backend: no_rknn` stops the packager from auto-promoting to the (nonexistent) RKNN/NPU
path. AGI-native configs keep their auto-RKNN behavior. **Rebuild rockchip after ANY
`pp_*.hpp` edit** — dist is never auto-rebuilt (the 07-02 stale-binary lesson).

Ship to the MDU (via the HDU jump host) and bring up per
[HARDWARE_BRINGUP_CHECKLIST.md](HARDWARE_BRINGUP_CHECKLIST.md).

---

## Stage 6 — Hardware bring-up (robot MDU)

Full staged procedure: [HARDWARE_BRINGUP_CHECKLIST.md](HARDWARE_BRINGUP_CHECKLIST.md).
The non-negotiables:

* **Only after Stage 4 passed** in the loc mode you will actually run on hardware.
* Stage gains up: start `--gain-scale 0.4` (small swing is expected at 0.4), `--single-swing`,
  one swing direction at a time, hands on the e-stop.
* yaw-align captures the heading at policy engage — the robot must be standing and facing
  its operational forward at that moment (re-armed on every SHADOW/MOTION engage).
* The runner's guards: squat 1.4 rad, tilt 0.35 rad — do NOT re-tighten for a model whose
  own swing crouches (v2/v4 clips reach knee ~1.14).
* Startup marker line (stale check) as in Stage 3.
* model_9000 ONLY: hardware requires the localization decision from Stage 4 (see the
  decision matrix). Without mocap, `perfect_tracking` must have passed.

---

## Quick reference (new checkpoint, all stages)

```text
# 1 export (grasping, env sourced)
hope_isaac_py scripts/play.py task=HOPEPingPongDeployParity algo=ppo headless=true num_envs=2 checkpoint=$RUN/model_N.pt
python scripts/inspect_a3_deploy_contract.py --onnx $RUN/exported/policy.onnx
# 2 training-side gate (grasping)
.venv-motion/bin/python scripts/mujoco_eval_onnx.py --onnx $RUN/exported/policy.onnx \
  --motion-files "artifacts/hope_forehand:v4/motion.npz" "artifacts/hope_backhand:v4/motion.npz" \
  --noise-scales 0.0 --pd-mode implicit --deploy-faithful --steps 1500
# 3 sync+build+parity (hope)
cp $RUN_ABS/exported/policy.onnx assets/a3_runtime/models/<name>.onnx   # + edit runtime cfg model_path
bash scripts/build_a3_deploy_pkg.sh --arch x86_64 --runtime-cfg src/a3/a3_deploy_onnx_ref/config/a3_runtime_config.pingpong.yaml
bash scripts/pingpong_parity/run_parity.sh
# 4 AGI MuJoCo gate (hope) — BOTH loc modes for walk-and-strike models
bash scripts/pp_freebase_watch.sh --single-swing
bash scripts/pp_freebase_watch.sh --single-swing --oracle-pelvis
# 5 rockchip (HOST, docker)
bash scripts/build_a3_deploy_pkg.sh --arch rockchip --runtime-cfg src/a3/a3_deploy_onnx_ref/config/a3_runtime_config.pingpong.yaml
```

---

## Appendix — how the swing works (10 lines, so nobody re-derives it)

The policy outputs a learned 31-DOF action every 20 ms; there is no hardcoded swing
trajectory. The C++ front-end scripts only the COMMAND: a per-clip racket target
(`pp_policy.hpp racket_{pos,vel}_w_clip`, must sit inside the model's training boxes) plus
the reference clock (seg lengths / strike phases from ONNX metadata; `time_to_strike`
clamped to the in-training max). `f`/`b` selects clip + target. The 175-D deploy_parity
obs drops the world base-position terms and expresses the racket target relative to the
live racket FK. Localization mode decides what the base obs sees: `perfect_tracking`
(reference pelvis — fine for strike-in-place models), `--oracle-pelvis` (sim truth), or a
future mocap source. Gains/action-scale/default pose all come from ONNX metadata and are
fail-fast validated at load. Source of truth: [PINGPONG_DEPLOY_ALIGNMENT.md](PINGPONG_DEPLOY_ALIGNMENT.md).

Historical model_15200 ground-contact experiment log (G0–G9.1) and the 180-D worked
example were removed in the 2026-07-03 rewrite — recover them from git history if needed
(`git log --follow PINGPONG_NEW_CHECKPOINT_TUTORIAL.md`).
