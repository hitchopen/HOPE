# Ping-pong policy ↔ AGI deploy alignment

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

> **CURRENT HitterPingPong contract (2026-08-03):** see
> [HITTER_PINGPONG_UNITREE_DEPLOY.md](HITTER_PINGPONG_UNITREE_DEPLOY.md).  The
> current 110-D/31-D policy is loaded from one Unitree-style policy directory;
> this document below remains the historical alignment record for older
> checkpoint-specific 175/180-D deployments.

**Goal (per AGI staff direction, 2026-06-30):** do **not** modify AGI's MuJoCo
simulation. Align *our* ONNX policy runner (`model_15200`, HOPE/HITTER ping-pong)
to AGI's official robot-I/O backend and deploy example. The real robot uses
**implicit PD in the body-drive backend**, so the `kp/kd` we publish *are* the PD
gains executed on hardware — they must equal the training gains.

This document is the consolidated contract + verification + bring-up package. It
sits alongside (does not replace) `PINGPONG_RUN.md`, `MUJOCO_VALIDATION_RUNBOOK.md`,
`SIM_DEPLOY_REHEARSAL.md`, and `HARDWARE_BRINGUP_CHECKLIST.md`.

> **New here / starting from a freshly trained checkpoint?** Read
> [`PINGPONG_NEW_CHECKPOINT_TUTORIAL.md`](PINGPONG_NEW_CHECKPOINT_TUTORIAL.md) first —
> the full copy-paste path `.pt` → ONNX export → AGI parity → MuJoCo → Rockchip →
> hardware, with the exact distrobox for every command.

---

## 0. HISTORICAL DEPLOY STATE (2026-07-03) — read this first, copy-paste to go live

> This section supersedes any `model_15200`/`p4` reference below. The rest of the doc
> (§1–§12) is the historical alignment/verification record; the facts here are the
> current ones.

**Staged policy:** `model_9000_replane.onnx` — **175-D obs / 31-act**, all-implicit-PD
training, v4 clips (seg `139,132` / strike phases `0.470,0.333` in metadata), from run
`logs/rsl_rl/agibot_a3_hope_deploy_parity/2026-07-03_02-01-17` (`model_9000.pt`).
Selected in
[`config/a3_runtime_config.pingpong.yaml`](src/a3/a3_deploy_onnx_ref/config/a3_runtime_config.pingpong.yaml)
(`onnx.model_path`). Scripted targets in `pp_policy.hpp` re-synced to its blade re-plane
boxes; C++⇔Python parity PASS (5.4e-7); `--use-imu-yaw` now default ON (engage-relative
yaw via yaw-align; revert `--no-imu-yaw` for p4).

**Swing clearance is PENDING the AGI-MuJoCo gate for this model** — do not inherit the
p4-era "forehand only / never `b`" verdict. Training-side deploy-faithful gate: 7/7 swings
incl. backhand 3/3, 0 falls (oracle base pose). ⚠ **model_9000 is a WALK-AND-STRIKE
policy** (turns ~84°, steps 0.4–0.65 m to its target): the AGI-MuJoCo gate MUST be run in
BOTH loc modes (`perfect_tracking` default AND `--oracle-pelvis`); if perfect_tracking
fails, hardware requires mocap localization — fall back to
`model_p4_deployparity.onnx` (+`--no-imu-yaw`), whose forehand-only 2026-07-02 verdict
(10 clean cycles, never `b`) still stands for p4 itself.

**Package state:** both dists rebuilt 2026-07-03 with `model_9000_replane.onnx` + the
07-03 runner changes (targets re-sync, imu-yaw default, periodic-wrap startup warning).
Rebuild BOTH dists after ANY `pp_*.hpp` edit — dist is never auto-rebuilt. Startup marker
that proves fresh binary AND fresh model:
`[pp] clip layout from ONNX metadata: seg_len={139,132} strike_phase={0.470,0.333}`.

### A. Rebuild the rockchip package (**HOST shell, NOT `hope`** — needs Docker)
> ⚠️ `--arch rockchip` re-invokes itself inside the `a3-rockchip-builder` Docker
> container. Docker lives on the **host**; the `hope` box has ROS but no Docker
> (`docker: command not found`). So run this from a **host terminal** and do **not**
> `source /opt/ros/jazzy/setup.bash` (the container carries ROS). This is the opposite
> of the x86 build.
```text
cd ~/workspace/HOPE/a3_deploy/a3_deploy_example
bash scripts/build_a3_deploy_pkg.sh --arch rockchip \
  --runtime-cfg src/a3/a3_deploy_onnx_ref/config/a3_runtime_config.pingpong.yaml \
  --jobs "$(nproc)"
```

### B. Verify the fresh package (x86 host)
```text
cd /home/dongc1/workspace/HOPE/a3_deploy/a3_deploy_example
file   dist/a3_deploy_rockchip/a3_deploy_onnx_ref_pingpong                     # -> ELF aarch64
grep   model_path dist/a3_deploy_rockchip/config/a3_runtime_config.pingpong.yaml  # -> models/model_p4_deployparity.onnx
ls     dist/a3_deploy_rockchip/models/model_p4_deployparity.onnx
strings dist/a3_deploy_rockchip/a3_deploy_onnx_ref_pingpong | grep -i "swing complete"  # non-empty = new binary
```

### C. Ship to the MDU (via the on-site HDU jump host)
```text
cd /home/dongc1/workspace/HOPE/a3_deploy/a3_deploy_example
HDU=<hdu_wifi_ip>          # on-site HDU Wi-Fi address
ssh -J agi@$HDU agi@10.42.10.12 'mkdir -p /agibot/a3_deploy'
rsync -azP -e "ssh -J agi@$HDU" dist/a3_deploy_rockchip/ agi@10.42.10.12:/agibot/a3_deploy/
```

### D. Run on the MDU — HOISTED, low gain, e-stop in hand, forehand only
```text
ssh -J agi@<hdu_wifi_ip> agi@10.42.10.12          # onto the MDU
source /agibot/software/v0/entry/env/env.sh        # robot env (iceoryx/DDS)
cd /agibot/a3_deploy
export A3_TRANSPORT=iceoryx
file ./a3_deploy_onnx_ref_pingpong                 # MUST say aarch64, not x86-64

# 6a) receive-only: 6 state topics + sync stable, NOTHING published
taskset -c 4-7 ./run_a3_pingpong.sh --dry-run

# 6b) MOTION, hoisted, forehand single-swing, low gain (recommended first live run)
taskset -c 4-7 ./run_a3_pingpong.sh \
  --start passive --legs-passive --gain-scale 0.4 --single-swing
```
Keys in the deploy terminal: `p` → `s` (wait ~3 s, confirm stable stand) → `h`
(SHADOW, no publish, watch tracking) → `m` (MOTION) → `1` (one forehand per press,
because `--single-swing`). `[`/`]` = gain ∓0.1, `,`/`.` = slower/faster, `q` = quit.
**Do NOT press `b`.** Do NOT use `--auto-start`.

**New runner flags & guard defaults (all in the fresh binary):**
| flag | default | meaning |
|---|---|---|
| `--single-swing` | off | play the clip once → auto-hold stand (no end→windup snap); press `1` to swing again. **Recommended for robot bring-up.** |
| `--swing-rest S` | off | like single-swing but auto re-arm after `S` s (continuous demo). A guard trip / manual `0` cancels the re-arm. |
| `--gain-scale F` | 1.0 | overall PD scale; start **0.4** on the robot |
| `--legs-passive` | off | hold legs at nominal (HOISTED demo; not a full-body test) |
| `--squat-guard-rad R` | **1.4** | trip to safe-hold if the policy commands a deep squat (the "catapult" fix) |
| `--tilt-guard G` | **0.35** | trip on excessive base tilt |
| `--warmup-sec S` | 0 | hold PD_STAND S s before auto-entering `--start` mode |
| `--perfect-tracking` | (default `loc_mode`) | hardware-safe localization (no fabricated world-pose error) |

---

## 1. Task summary

* AGI's runner `a3_deploy_onnx_ref` runs AGI's own teleop/tokenizer policy
  (`model_step_098000_a3`, obs **1570** → act **29**). Our HOPE policy is
  `model_15200` (obs **180** → act **31**), so it is **not** drop-in.
* Following `README_robot_io_backend.md` route **A** ("only reuse `RobotIOBackend`,
  write your own policy executable"), a **separate** runner already exists:
  **`a3_deploy_onnx_ref_pingpong`** (entry `src/.../a3_deploy/a3_pingpong_main.cpp`,
  front-end `include/a3_pingpong/*`). AGI's original `a3_deploy_onnx_ref` +
  `main.cpp` + `A3AimrtBackend` + `A3PolicyDriver` are **untouched** (separate
  CMake target).
* The runner reuses AGI's backend, 100 Hz sync, 50 Hz `A3PolicyDriver`,
  watchdog and safe-halt **unchanged**; only the front-end (obs build, ONNX
  inference, action decode, joint map, gains) is ours.
* The full obs/action contract is **embedded in the ONNX metadata** and consumed
  by the runner at load time (joint_names, action_scale, default_joint_pos,
  joint_stiffness=kp, joint_damping=kd, body_names, anchor_body_name). This makes
  the contract self-describing and removes stale hard-coded copies.

### What was verified in this pass (x86, no robot)

| Check | Result |
|---|---|
| ONNX IO | `obs[1,180]`, `time_step[1,1]` → `actions[1,31]` (+ ref side-outputs). ✅ |
| Action joint map is a bijection onto the 31 backend slots | ✅ (all 31 filled once) |
| `pp_joint_map::backend_joint_order()` == AGI `robot_io::MakeA3Layout31()` | ✅ slot-for-slot |
| `kp/kd` published == training `joint_stiffness/joint_damping` (implicit PD) | ✅ kp∈[20,250], kd∈[2,8] |
| Neck (head_yaw/head_pitch) forced passive q=0, kp=40, kd=2 | ✅ matches AGI `ExpandToBackend` |
| **C++ ONNX inference == Python ONNX inference** (same obs) | ✅ max\|Δ\|=9.5e-7 (action), 4.1e-7 (target_q) |
| **End-to-end C++ `ComputeCommand`** on nominal state | ✅ finite, bounded (max\|q_des\|≈0.88 rad) |
| First-tick debug dump (joint pos/vel, IMU/gravity, per-block obs stats, action stats, q_des/kp/kd) | ✅ added + compiles + runs |

> Verification harnesses are durable + reproducible in
> [`tests/`](tests/) — run
> `PYBIN=<py-with-onnxruntime> bash tests/run_parity.sh`
> (no ROS/AimRT). It contains `pp_parity_harness.cpp` (C++↔Py ONNX parity),
> `pp_e2e_harness.cpp` (end-to-end ComputeCommand + first-tick dump),
> `gen_python_ref.py`, and `first_tick_sample.txt` (sample log for AGI).

### Not verifiable in this environment (needs the AGI build / a robot)

* x86 package **rebuild** via `scripts/build_a3_deploy_pkg.sh` — needs ROS 2
  Jazzy/Humble + AimRT (FetchContent) + iceoryx; **not installed here**. The
  bundled `dist/a3_deploy_x86_64/` was built on a machine that had them.
* AGI **MuJoCo** closed-loop verification — needs ROS 2 Humble + the standalone
  sim. Commands below; run on the build box.
* Real-robot bring-up — see §9 safety checklist.

---

## 2. AGI backend contract (from `README_robot_io_backend.md`)

**State in (`robot_io::RobotState`):** `q`, `dq`, `tau_est` (31, backend order),
pelvis IMU `imu_quat_wxyz` / `imu_gyro` / `imu_accel`, optional torso (secondary)
IMU, plus `sync_complete` / `sync_aligned` / `tick`.

**Command out (`robot_io::RobotCommand`):** `q_des`, `dq_des`, `tau_ff`, `kp`,
`kd`, **each length 31** (= `GetLayout().dof()`). Backend splits into
`/body_drive/{waist,leg,arm,neck}_joint_command`.

**Implicit PD:** runs inside the body-drive backend / `hal_ethercat`, using the
`q_des, kp, kd` (and `dq_des, tau_ff`) we send. There is **no explicit torque PD
in our runner** — this matches Isaac's `ImplicitActuator`. ⇒ the published gains
must be the training gains.

**31-DOF backend layout** (`MakeA3Layout31`, == our `backend_joint_order()`):
```
[0..2]  waist: waist_yaw, waist_roll, waist_pitch
[3..4]  neck:  head_yaw, head_pitch
[5..11] left arm   [12..18] right arm
[19..24] left leg  [25..30] right leg
```

**Control freq:** policy **50 Hz**, sync **100 Hz** (`sync_mode=min_skew_pair`).
**Safety:** `--dry-run` / `--probe` / `publish_enabled=false` (no command pub);
state-age + chronic-unaligned watchdog → safe-halt; manual state machine
`PASSIVE → PD_STAND → MOTION`.

---

## 3. Observation contract — 180-D (verified order/spans)

ONNX `observation_names` (auto-exported from training) == C++ `build_obs_180`
order. World-pose-dependent terms are sourced via the **localization mode**
(`fabricated` / `perfect_tracking` / `oracle`); the obs *layout* is identical in
all modes, only values differ. **No global localizer exists on the robot**, so
the hardware-safe default is **perfect_tracking (B)**.

| # | obs block (span) | physical quantity | AGI backend source | frame | unit | transform | verified |
|---|---|---|---|---|---|---|---|
| 1 | command ref `joint_pos` [0:31] | reference clip joint pos @ time_step | ONNX side-output (clip baked in) | Isaac joint order | rad | reference clock: time_to_strike→time_step | ✅ |
| 2 | command ref `joint_vel` [31:62] | reference clip joint vel | ONNX side-output | Isaac | rad/s | — | ✅ |
| 3 | `motion_anchor_pos_b` [62:65] | torso anchor pos error (ref vs robot) | torso world pose (loc-mode) + ONNX `body_pos_w[7]` | robot torso frame | m | `subtract_frame_transforms` | ✅ (=0 in mode B) |
| 4 | `motion_anchor_ori_b` [65:71] | torso anchor ori (6D) | torso IMU + ONNX `body_quat_w[7]` | robot torso frame | unit | first **two columns** of R, row-major | ✅ |
| 5 | `base_ang_vel` [71:74] | pelvis angular velocity | `RobotState.imu_gyro` | pelvis body | rad/s | direct | ✅ |
| 6 | `joint_pos_rel` [74:105] | q − default_q | `RobotState.q` (reorder SDK→Isaac) − `default_joint_pos` | Isaac | rad | gather + subtract | ✅ |
| 7 | `joint_vel` [105:136] | joint velocity | `RobotState.dq` (reorder SDK→Isaac) | Isaac | rad/s | gather | ✅ |
| 8 | `last_action` [136:167] | previous **raw** policy action | runner state (`last_action_`) | Isaac | — | held from prev tick; 0 on first | ✅ |
| 9 | `projected_gravity` [167:170] | gravity dir in body | `RobotState.imu_quat_wxyz` (pelvis) | pelvis body | unit | `Rᵀ·[0,0,-1]`, wxyz quat | ✅ |
| 10 | `base_target_pos_b` [170:172] | base XY goal | scripted (0 ⇒ stay) + base world XY (loc-mode) | yaw-heading base | m | `quat_rotate_inverse(yaw_quat,·)` | ✅ |
| 11 | `racket_target_pos_b` [172:175] | racket target pos | scripted target + base world pose (loc-mode) | yaw-heading base | m | `quat_rotate_inverse(yaw_quat,·)` | ✅ |
| 12 | `racket_target_vel_w` [175:178] | racket target vel | scripted target | **world** | m/s | direct (world, not body) | ✅ |
| 13 | `time_to_strike` [178] | seconds to strike | strike cycle (`strike_period`, `strike_lead_frac`) | — | s | — | ✅ |
| 14 | `swing_type` [179] | +1 fore / −1 back | sign of target y | — | — | — | ✅ |

> Frame math (`pp_frame_math.hpp`) is a **verbatim** port of the training/sim2sim
> `frame_math.py`: scalar-first quaternions (w,x,y,z), world←body convention, 6D
> rotation = first two **columns** of R. Do not "improve" these.

**Localization-dependent terms (3, 4, 10, 11)** are the sim2real gap. In
`perfect_tracking` (B): `torso_pos := ref anchor` (so `motion_anchor_pos_b == 0`,
confirmed in the first-tick dump) and `base_pos := ref pelvis` (racket/base
targets relative to where the pelvis *should* be). Orientation always uses the
**real IMU**. `oracle` (C) is **simulation only** (reads true MuJoCo pelvis pose
from `/dev/shm`); on hardware the shm is absent → auto-fallback to B with a loud
warning.

---

## 4. Action contract — 31-DOF (verified from ONNX metadata, bijection ✅)

Decode: **`q_des_isaac = default_joint_pos + raw_action ⊙ action_scale`**, then
name-scatter to backend slots. `kp/kd` published = `joint_stiffness/joint_damping`
scattered to backend order (this is the **implicit-PD** gain set). Neck slots
[3,4] overridden to q=0, kp=40, kd=2 (model neck output dropped). `dq_des=0`,
`tau_ff=0`.

| policy idx | policy joint (Isaac) | AGI backend slot | scale | default (rad) | kp | kd | verified |
|---|---|---|---|---|---|---|---|
| 0 | left_hip_pitch | 19 | 0.688 | −0.131 | 80 | 3 | ✅ |
| 1 | right_hip_pitch | 25 | 0.688 | −0.131 | 80 | 3 | ✅ |
| 2 | waist_yaw | 0 | 0.647 | 0.000 | 85 | 3 | ✅ |
| 3 | left_hip_roll | 20 | 0.458 | 0.006 | 120 | 4 | ✅ |
| 4 | right_hip_roll | 26 | 0.458 | −0.006 | 120 | 4 | ✅ |
| 5 | waist_roll | 1 | 0.230 | 0.000 | 50 | 2 | ✅ |
| 6 | left_hip_yaw | 21 | 0.688 | −0.035 | 80 | 3 | ✅ |
| 7 | right_hip_yaw | 27 | 0.688 | 0.035 | 80 | 3 | ✅ |
| 8 | waist_pitch | 2 | 0.590 | 0.000 | 50 | 2 | ✅ |
| 9 | left_knee | 22 | 0.320 | 0.247 | 250 | 8 | ✅ |
| 10 | right_knee | 28 | 0.320 | 0.247 | 250 | 8 | ✅ |
| 11 | **head_yaw** | 3 | 0.038 | 0.000 | **40→passive** | **2** | ✅ (q forced 0) |
| 12 | left_shoulder_pitch | 5 | 0.375 | 0.300 | 40 | 3 | ✅ |
| 13 | right_shoulder_pitch | 12 | 0.375 | 0.300 | 40 | 3 | ✅ |
| 14 | left_ankle_pitch | 23 | 0.591 | −0.120 | 50 | 2 | ✅ |
| 15 | right_ankle_pitch | 29 | 0.591 | −0.120 | 50 | 2 | ✅ |
| 16 | **head_pitch** | 4 | 0.038 | 0.000 | **40→passive** | **2** | ✅ (q forced 0) |
| 17 | left_shoulder_roll | 6 | 0.375 | 0.120 | 40 | 3 | ✅ |
| 18 | right_shoulder_roll | 13 | 0.375 | −0.120 | 40 | 3 | ✅ |
| 19 | left_ankle_roll | 24 | 0.274 | −0.008 | 50 | 2 | ✅ |
| 20 | right_ankle_roll | 30 | 0.274 | 0.008 | 50 | 2 | ✅ |
| 21 | left_shoulder_yaw | 7 | 0.200 | 0.000 | 30 | 2 | ✅ |
| 22 | right_shoulder_yaw | 14 | 0.200 | 0.000 | 30 | 2 | ✅ |
| 23 | left_elbow | 8 | 0.200 | 0.800 | 30 | 2 | ✅ |
| 24 | right_elbow | 15 | 0.200 | 0.800 | 30 | 2 | ✅ |
| 25 | left_wrist_roll | 9 | 0.200 | 0.000 | 30 | 2 | ✅ |
| 26 | right_wrist_roll | 16 | 0.200 | 0.000 | 30 | 2 | ✅ |
| 27 | left_wrist_pitch | 10 | 0.075 | 0.000 | 20 | 2 | ✅ |
| 28 | right_wrist_pitch | 17 | 0.075 | 0.000 | 20 | 2 | ✅ |
| 29 | left_wrist_yaw | 11 | 0.075 | 0.000 | 20 | 2 | ✅ |
| 30 | right_wrist_yaw | 18 | 0.075 | 0.000 | 20 | 2 | ✅ |

> ✅ **Clip (FIXED this pass):** Isaac trained `JointPositionActionCfg` with **no
> clip** (`use_default_offset=True`), so `q_des` was previously unbounded. The
> runner now clamps `q_des` to the **MJCF joint position limits**
> (`pp_joint_limits.hpp`, backend order, defaults validated strictly in-range)
> right before publish — a no-op for in-distribution actions, and it warns + counts
> any joint it has to clamp (surfaced in the first-tick dump and 1 Hz status).

---

## 5. The runner (what is ours vs AGI's)

**Ours** (`include/a3_pingpong/*`, `src/a3_deploy/a3_pingpong_main.cpp`): obs
builder, ONNX session + metadata-driven decode, joint map, frame math, base
estimator, scripted target + reference clock, localization modes, staged mode
machine, diagnostics. **Header-only, depends only on Eigen + onnxruntime + the
plain `robot_io` structs** → unit-testable off-robot (this is how the parity/e2e
harnesses build with no ROS).

**AGI's, reused unchanged:** `A3AimrtBackend` (iceoryx/ros2, 6-topic sync,
command split), `A3PolicyDriver` (50 Hz RT loop, state cache, watchdog,
safe-halt), `ExpandToBackend`, `safe_halt`.

**Change made this pass:** consolidated **one-shot first-tick debug dump** in
`pp_policy.hpp` (`LogFirstTick`) — prints loc_mode/time_step, IMU quat +
projected_gravity + gyro, SDK joint pos/vel stats, **per-named-obs-block stats
for all 13 blocks**, raw ONNX action stats, decoded q_des/kp/kd stats, and the
worst q_des−q_meas joint. Fires once on the first policy tick. (Replaces the older
partial `[pp dbg]` line; the 1 Hz status block with per-joint tracking + obs
slices is unchanged.) Sample output saved for AGI: see `first_tick_sample.txt`.

Runtime modes: `p`=PASSIVE (limp, zero gain) · `s`=PD_STAND · `h`=SHADOW
(compute, **no publish**) · `m`=MOTION (publish) · `0/1`=swing level ·
`[`/`]`=gain_scale · `--dry-run`/`--no-publish`. Hardware-safe loc default:
`--perfect-tracking`.

---

## 6. x86 verification commands

```text
cd a3_deploy/a3_deploy_example

# (a) Build the x86 package (needs ROS 2 Jazzy/Humble + AimRT toolchain).
source /opt/ros/jazzy/setup.bash
bash scripts/build_a3_deploy_pkg.sh \
  --arch x86_64 \
  --runtime-cfg src/a3/a3_deploy_onnx_ref/config/a3_runtime_config.pingpong.yaml \
  --jobs 20
# -> dist/a3_deploy_x86_64/{a3_deploy_onnx_ref_pingpong,run_a3_pingpong.sh,config/,models/model_15200.onnx}

# (b) Standalone C++<->Python ONNX parity (NO ROS needed; reproduces this pass).
#     PREFERRED: the wrapper builds both harnesses, generates the python ref,
#     and runs the diff for you. Run from the a3_deploy_example repo root.
bash tests/run_parity.sh
# Result this pass: action max|Δ|=9.5e-7, target_q max|Δ|=4.1e-7  => PASS.
#
#     Manual build (only if iterating on the harness source). NOTE the harnesses
#     live in tests/ — compiling a bare `pp_parity_harness.cpp`
#     from the repo root fails with "No such file or directory".
ORT=thirdparty/onnxruntime/onnxruntime-linux-x64-1.19.2
g++ -std=c++20 -O2 tests/pp_parity_harness.cpp \
  -Isrc/a3/a3_deploy_onnx_ref/include -I/usr/include/eigen3 -I"$ORT/include" \
  -L"$ORT/lib" -lonnxruntime -Wl,-rpath,"$PWD/$ORT/lib" -o /tmp/pp_parity
# The harness takes args: /tmp/pp_parity <model.onnx> <obs.txt> <time_step>
# (run_parity.sh wraps it with generated obs_*.txt / py_act_*.txt files).

# (c) End-to-end ComputeCommand smoke (NO ROS): finite/bounded + first-tick dump.
g++ -std=c++20 -O2 tests/pp_e2e_harness.cpp \
  -Isrc/a3/a3_deploy_onnx_ref/include -I/usr/include/eigen3 -I"$ORT/include" \
  -L"$ORT/lib" -lonnxruntime -Wl,-rpath,"$PWD/$ORT/lib" -o /tmp/pp_e2e
/tmp/pp_e2e assets/a3_runtime/models/model_15200.onnx 1
# NOTE: model_15200.onnx lives durably at assets/a3_runtime/models/, and when the
# package is built with the pingpong --runtime-cfg it is also staged into
# dist/a3_deploy_x86_64/models/model_15200.onnx.
# Result this pass: dims=31, q_des finite max≈0.88 rad, kp∈[20,250], kd∈[2,8] => PASS.

# (d) Unit tests (built by the package): joint-map bijection, obs builder, parity.
#     pp_jointmap_test, pp_obs_builder, pp_parity_test, pp_e2e_test, pp_policy_test.
```

---

## 7. MuJoCo deploy verification (AGI sim **as-is** — do not patch it)

```text
# Terminal 1 — AGI MuJoCo standalone (starts iox-roudi + MuJoCo GUI):
cd a3_deploy/a3_deploy_example/mujoco_sim_standalone
./run.sh a3_t2d5_cfg.yaml          # pick an ICEORYX joint-topic cfg (a3_t2d5 = our variant)

# Terminal 2 — our runner (transport must match the sim; default iceoryx):
cd a3_deploy/a3_deploy_example/dist/a3_deploy_x86_64
RT=config/a3_runtime_config.pingpong.yaml
# 1) receive-only: confirm 6 state topics + sync stable, no publish
A3_SOURCE_ROBOT_ENV=0 A3_TRANSPORT=iceoryx ./run_a3.sh --dry-run    # (or: a3_deploy_onnx_ref_pingpong --runtime-cfg $RT --dry-run)
# 2) shadow: compute, NO publish, watch rate≈50Hz, |act| bounded, first-tick dump, obs blocks sane
./a3_deploy_onnx_ref_pingpong --runtime-cfg $RT --perfect-tracking --start shadow
# 3) PD_STAND: press 's' -> robot holds nominal; in MuJoCo GUI click load-key; confirm stable stand
# 4) closed loop: press 'h'->'m' (MOTION), level 0 (press '0') first, then level 1 (press '1')
```

Purpose: prove **deploy-path closure only** (state in → obs → ONNX → decode →
command out, no immediate divergence from interface mismatch). It is **not** a
claim the policy is safe on the real robot. Watch `sync_complete`/`sync_aligned`
stable and `infer_ms` < control period before any MOTION.

---

## 8. Cross-compilation commands (Rockchip/MDU = iceoryx)

```text
cd a3_deploy/a3_deploy_example
# x86_64 Docker builder + aarch64 sysroot (tarballs already in thirdparty/*_sysroot/)
bash scripts/build_a3_deploy_pkg.sh --arch rockchip --jobs 20
# -> dist/a3_deploy_rockchip/  (RKNN enabled; packaged onnx.backend rewritten to rknn)
#    builds a3_deploy_onnx_ref_pingpong too; stage model_15200.onnx + a runtime cfg.

# Verify the package contents:
file dist/a3_deploy_rockchip/a3_deploy_onnx_ref_pingpong          # ELF aarch64
ls dist/a3_deploy_rockchip/{models,config,lib}                    # onnx/rknn + cfg + runtime libs

# Ship to MDU via the HDU jump host (<hdu_wifi_ip> = on-site):
rsync -azP -e "ssh -J agi@<hdu_wifi_ip>" dist/a3_deploy_rockchip/ agi@10.42.10.12:/agibot/a3_deploy/
```

The **same runner + config** are used for MuJoCo verification and the robot —
only the transport YAML and the ONNX/RKNN backend differ. (Local laptop was
missing zmq dev headers → vendored to `thirdparty/zmq_shim/`; if the rockchip
toolchain also lacks them, point `ZMQ_INCLUDE_DIR/CPPZMQ_INCLUDE_DIR` there.)

---

## 9. Real-robot safety checklist (HOISTED first, staged)

Pre-flight:
- [ ] Robot **hoisted / on safety rope**; physical **e-stop in hand**.
- [ ] `--perfect-tracking` (no fabricated world-pose error); neck passive.
- [ ] Confirm `dist/.../config/a3_runtime_config.pingpong.yaml` → **`models/model_p4_deployparity.onnx`** (175-D; see §0), iceoryx, policy 50 Hz. And `file …_pingpong` == aarch64.

Bring-up order (do **not** `--auto-start`):
1. [ ] **Log-only / receive:** `taskset -c 4-7 ./run_a3.sh --dry-run` → 6 state topics ready, sync stable.
2. [ ] **Probe:** `A3_LATENCY_LOG=verbose ./run_a3_probe.sh` → infer_ms < 20 ms, no command published.
3. [ ] **Real joint states enter the runner:** start runner, **PASSIVE**; read the **first-tick dump** — confirm `STATE(SDK) q` ≈ real pose, `qd` ≈ 0.
4. [ ] **Observation sanity** (first-tick dump): `projected_gravity` ≈ [0,0,−1] upright; `motion_anchor_pos_b` ≈ 0 (mode B); `base_ang_vel` ≈ gyro; no NaN; obs block stats bounded.
5. [ ] **Action / target sanity:** `|action|` bounded; `Q_DES(SDK)` within joint limits; `KP/KD` == training (kp∈[20,250], kd∈[2,8]); **neck q=0,kp=40,kd=2**.
6. [ ] **PD_STAND** (`s`): robot holds nominal; tune `--stand-kp/--stand-kd` (hoist) or `--official-stand` (ground).
7. [ ] **SHADOW** (`h`): policy runs, **no publish**; watch the 1 Hz tracking block; confirm continuity (no q_des jumps).
8. [ ] **Watchdog:** induce a dropped state (pause sim / unplug a topic) → confirm safe-halt triggers; `halts` counter increments.
9. [ ] **E-stop / safe-halt:** confirm e-stop cuts power and `q` SIGINT (`q` key) exits cleanly to PASSIVE.
10. [ ] **MOTION** (`m`) at **low gain** (`--gain-scale 0.4`), **`--single-swing`**, press `1` for one **forehand** at a time (never `b` — backhand not deploy-ready); short, small first.
11. [ ] **Save the first-tick log + a `--trace-csv` / `--obs-csv` capture for AGI staff review.**

Now enforced in code (verify in the first-tick dump):
- [x] **`q_des` joint-limit clamp** before publish (`pp_joint_limits.hpp`; "q_des clamped on N/31" line — N should be 0).
- [x] **`loc_mode=perfect_tracking`** is the default (not `fabricated`); confirm the dump shows `loc_mode=perfect_tracking(B)`.
- [x] **Secondary-IMU guard**: if absent, a loud `[pp WARN]` fires and the dump shows `IDENTITY-FALLBACK(!)` — do **not** run MOTION in that state.

Still to confirm on the real sync rate:
- [ ] AGI's watchdog `max_frame_age_ms` / `max_unaligned_frames` thresholds vs the measured sync rate.

---

## 10. TODO (ordered for safe bring-up)

1. **(build)** On the AGI toolbox: `build_a3_deploy_pkg.sh --arch x86_64`; confirm
   `a3_deploy_onnx_ref_pingpong` links `model_15200.onnx` and the pp unit tests pass.
2. **(parity)** Re-run the C++↔Python ONNX parity + e2e harness on the built tree
   (this pass: PASS, ≤1e-6). Keep `golden.txt` in sync with the shipped model.
3. **(safety code)** Add `q_des` joint-limit clamp + optional per-tick Δq rate
   limit before publish; keep the existing SHADOW + gain_scale path.
4. **(sim)** AGI MuJoCo closed-loop (§7) with the sim **unmodified**: dry-run →
   probe → shadow → PD_STAND → MOTION L0 → L1. Confirm no interface-mismatch
   divergence; save obs-CSV.
5. **(cross)** `build_a3_deploy_pkg.sh --arch rockchip`; verify ELF aarch64 +
   bundled libs/config/model; rsync to MDU.
6. **(robot)** Hoisted, staged bring-up per §9; capture first-tick + trace logs
   for AGI staff.
7. **(planner)** Replace scripted racket target with the live planner output on
   `/racket/command` once §6 is clean (targets currently scripted).

---

## 11. Verification artifacts (this pass)

* [`tests/`](tests/) — durable, reproducible
  no-ROS bundle: `run_parity.sh`, `gen_python_ref.py`, `pp_parity_harness.cpp`,
  `pp_e2e_harness.cpp`, `first_tick_sample.txt`, `README.md`.
* C++↔Py ONNX parity: action max|Δ|=9.5e-7, target_q max|Δ|=4.1e-7. **PASS.**
* e2e ComputeCommand: dims=31, finite, max|q_des|≈0.88 rad, kp∈[20,250], kd∈[2,8]. **PASS.**
* New durable assets: `assets/a3_runtime/models/model_15200.onnx`,
  `src/a3/a3_deploy_onnx_ref/config/a3_runtime_config.pingpong.yaml` +
  `a3_aimrt_config.pingpong_iceoryx.yaml`; x86 dist restored.
* New code: `src/a3/a3_deploy_onnx_ref/include/a3_pingpong/pp_joint_limits.hpp`;
  edits to `pp_policy.hpp` (first-tick dump, q_des clamp, IMU guard, safe loc default)
  and `a3_pingpong_main.cpp` (safe loc default).

---

## 12. Adversarial audit — results, fixes applied, remaining gaps

A 20-agent adversarial audit cross-checked the C++ runner against the Python
`wbc_runner`, the training env cfg, and the ONNX metadata, and skeptically
verified each obs block.

**Confirmed aligned (4-source byte match unless noted):**
* Training env cfg **== ONNX metadata** (0 real mismatches): 13 obs terms / 180-D,
  31 joints Isaac order, `action_scale = 0.25·effort_limit/stiffness`
  (`use_default_offset=True`, **no clip**), default pose, `kp/kd` (ImplicitActuator).
* `dt=0.005`, `decimation=4` ⇒ **50 Hz** (== AGI's policy 50 / sync 100).
* `base_lin_vel` is **intentionally dropped** from the deployed actor (not
  measurable on a floating base) — correctly absent from the 180-D obs (critic-only).
* C++ obs builder / decode / frame math are a **faithful** port of the Python
  reference (the only intentional C++ divergence is being a *superset* on
  localization — adds perfect-tracking/oracle + waist-FK torso).
* Action decode, implicit-PD `kp/kd`, neck-passive, joint-map bijection: **PASS**.

**Fixes applied this pass (all compile + run; verified via the e2e/parity harnesses):**
| Fix | Severity | Where |
|---|---|---|
| `q_des` clamp to MJCF joint limits before publish | HIGH | `pp_joint_limits.hpp` (new) + `pp_policy.hpp` |
| Hardware-safe default `loc_mode = perfect_tracking` (was `fabricated`, the buzz mode) | HIGH | `PpPolicyConfig`, `a3_pingpong_main.cpp` default, pingpong YAML `obs_debug.loc_mode` |
| Loud one-shot guard when secondary/torso IMU absent (else anchor-ori silently wrong) | HIGH | `pp_policy.hpp` |
| Consolidated one-shot first-tick debug dump (obs blocks + IMU + action + q_des/kp/kd + clamp/IMU status) | — | `pp_policy.hpp` `LogFirstTick` |
| Restored the pingpong runtime assets as durable source files and taught the packager to preserve the pingpong runtime-config basename + wrapper script | MEDIUM | repo |

> `build_a3_deploy_pkg.sh` still wipes `dist/` on every rebuild, but the ping-pong
> path is now staged correctly when you pass
> `--runtime-cfg src/a3/a3_deploy_onnx_ref/config/a3_runtime_config.pingpong.yaml`.
> That build preserves `config/a3_runtime_config.pingpong.yaml`, stages
> `model_15200.onnx`, and emits `run_a3_pingpong.sh`.

**Remaining gaps / TODO (not blocking the contract; mostly need the build/sim/robot):**
* **[BUILD]** Wire `pp_jointmap_test` / `pp_policy_test` / `pp_parity_test` into a
  CMake/CTest target (currently only one-off `g++` recipes) + commit a golden.
* **[LOW]** Zero `last_action_` on PASSIVE/PD_STAND / MOTION re-entry to match the
  Python reset (first post-idle swing tick currently carries a stale last_action).
* **[LOW]** `time_step_for` rounding: C++ `std::lround` (half-away-from-zero) vs
  Python `round` (banker's) ⇒ ≤1 reference frame at exact .5 ties; negligible at 50 Hz.
* **[PRE-LIVE]** Replace scripted racket target with the live planner on
  `/racket/command`; the policy expects **world-frame** `racket_target_vel_w` —
  resolve the `base_link`→world rotation (planner_imitate currently warns, not rotates).
* **[UNVERIFIED — needs build/sim/robot]** AGI MuJoCo closed-loop acceptance run;
  A/B/C obs-CSV rehearsal; rockchip/thor cross-builds; hardware bring-up; the
  hardware **ros2** AimRT plugin (prebuilt `.so` is Humble-ABI, breaks on Jazzy →
  rebuild in-box). The known AGI free-base sim ~0.1 s divergence is the
  explicit-PD-vs-ImplicitActuator fidelity gap (per AGI), **not** a port bug — use
  the HOIST cfg for a clean swing.
