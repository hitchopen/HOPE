# MuJoCo deploy-path rehearsal + fabricated-pose fix (sim-to-real)

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

> **SUPERSEDED for current checkpoints (2026-07-03):** the 180-D obs-layout
> assertions in this doc (the obs indices below, the "180-D obs layout …
> unchanged" claim, the Step-5 "obs still 180-D" check) and the loc_mode A/B/C
> rehearsal apply to **legacy 180-D checkpoints only** (model_15200 lineage).
> Current 175-D deploy-parity policies (model_p4_deployparity / explicitpd_ft)
> remove `motion_anchor_pos_b` and `base_target_pos_b` and reframe
> `racket_target_pos_b` racket-FK-relative, so they have **no world-base-position
> obs terms** — loc_mode is irrelevant for them. See
> `hope_training/whole_body_tracking/scripts/realsensor_obs_reference.py`.

Goal: reproduce the deployed buzz/instability **in MuJoCo, off-hardware**, prove
it is the **fabricated world-pose observation** (not the ONNX policy), and validate
the **perfect-tracking** fix before any hardware staging.

> **Safety:** every step here is SHADOW / non-publishing or sim-only. No hardware
> motion. Oracle localization is **simulation only** and cannot run on hardware
> (the shm file is absent → it falls back). Do not proceed to hardware until
> Step 5 passes (see the gate at the bottom).

## Why the buzz happens (recap)

The A3 backend `RobotState` carries **only** joint state + pelvis/torso IMU — **no
base/world pose**. Three obs terms need world pose:
`motion_anchor_pos_b` (obs[62:65]), `racket_target_pos_b` (obs[172:175]),
`base_target_pos_b` (obs[170:172]). The deploy runner was fabricating them from a
**frozen nominal pose**, so `motion_anchor_pos_b` became a **fictional, time-varying
"you are mis-tracking by ~0.2 m" signal** → the policy fights it → buzz. In sim and
in the Python eval this never showed because both had ground-truth pose.

## Three localization modes (obs LAYOUT identical; only the values change)

| mode | flag | what it does | use |
|---|---|---|---|
| **A fabricated** | (default) | nominal frozen pose → fake anchor error | reproduce the bug |
| **B perfect-tracking** | `--perfect-tracking` | `torso=ref anchor` (anchor_pos_b≈0), `base=ref pelvis`; real IMU for orientation | **the hardware-safe fix** |
| **C oracle** | `--oracle-pelvis` | true MuJoCo pelvis pose via shm bridge | **sim only**, proves policy is correct |

## Files changed / added

- `src/a3/a3_deploy_onnx_ref/include/a3_pingpong/pp_obs_builder.hpp` — add `ref_pelvis_pos_w` to `PpRefs`.
- `.../pp_onnx_policy.hpp` — fill `ref_pelvis_pos_w` from reference body[0].
- `.../pp_policy.hpp` — `LocMode` enum (A/B/C); 3-mode localization in `ComputeCommand`; obs capture + `take_obs_debug()`; sync-miss counter. **180-D obs layout, 31-DOF order, ONNX, and command topics all unchanged.**
- `.../pp_oracle_pose.hpp` — **new**, sim-only shm reader (no ROS 2 linkage).
- `src/a3/a3_deploy_onnx_ref/src/a3_deploy/a3_pingpong_main.cpp` — `--loc-mode/--perfect-tracking/--oracle-pelvis/--oracle-shm/--oracle-max-age/--obs-csv`; oracle wiring + SIM-ONLY warning; obs CSV (obs+action); periodic obs-debug print.
- `dist/a3_deploy_x86_64/config/a3_runtime_config.pingpong.yaml` — `obs_debug:` block.
- `scripts/oracle_pose_bridge.py` — **new**, rclpy `/sim/a3/pelvis_pose` → shm (sim only).
- `scripts/analyze_obs_log.py` — **new**, A/B/C obs-CSV comparison.

---

## Step 1 — build the sim and the runner

The **source-built sim** (`a3_deploy/A3_MuJoCo_Sim/aimrt_mujoco_sim/build/install`) is the rehearsal sim: it
drives `/body_drive/*` on **iceoryx** *and* publishes the ground-truth
`/sim/a3/pelvis_pose`, `/sim/a3/right_racket_pose`, `/tf` on ROS 2 (needed for
oracle mode). The prebuilt `mujoco_sim_standalone` does **not** publish `/sim/a3/*`,
so it can only serve modes A/B.

```text
# (inside the box that has the AGI toolchain + ROS 2 + mujoco)
# 1a. build the sim (ROS 2 ON so /sim/a3/* publish):
cd ~/workspace/HOPE/a3_deploy/A3_MuJoCo_Sim/aimrt_mujoco_sim
./build.sh

# 1b. rebuild the ping-pong runner (picks up the new source):
cd ~/workspace/HOPE/a3_deploy/a3_deploy_example
bash scripts/build_a3_deploy_pkg.sh --arch x86_64     # produces dist/a3_deploy_x86_64/a3_deploy_onnx_ref_pingpong
# sanity: the runner needs onnxruntime on its lib path
cd dist/a3_deploy_x86_64 && source env.sh 2>/dev/null || true
ldd a3_deploy_onnx_ref_pingpong | grep -i onnxruntime    # must resolve
```

## Step 2 — reproduce the bug (mode A, current default)

```text
# Terminal 1 — sim (starts iox-roudi + MuJoCo, GUI):
# IMPORTANT: run from the INSTALLED package dir (build/install/bin), which is
# self-contained (aimrt_main, cfg/, model/, iox-roudi, ../lib, ../share). The
# source src/models/bin/ has only the script+cfg -> "./aimrt_main: No such file".
cd ~/workspace/HOPE/a3_deploy/A3_MuJoCo_Sim/aimrt_mujoco_sim/build/install/bin
./start_a3_pingpong_iceoryx.sh
# if a stale roudi is running and iceoryx won't connect:
#   pkill -x iox-roudi; sleep 1; ./start_a3_pingpong_iceoryx.sh

# Terminal 2 — runner in SHADOW (computes from the real backend path, NEVER publishes):
cd ~/workspace/HOPE/a3_deploy/a3_deploy_example/dist/a3_deploy_x86_64
RT=config/a3_runtime_config.pingpong.yaml
./a3_deploy_onnx_ref_pingpong --runtime-cfg $RT --start shadow --level 1 \
    --loc-mode fabricated --obs-csv /tmp/obs_A_fabricated.csv
```
Watch the `[obs]` block: in mode A `motion_anchor_pos_b |.|` is **large and drifts**.
To *see* the buzz physically, switch the runner to MOTION (press `m`) so it drives
the **sim** (sim only — still no hardware): the robot should buzz/lurch. Save
`/tmp/obs_A_fabricated.csv`.

## Step 3 — oracle localization (mode C, sim only) — proves the policy is fine

```text
# Terminal 3 — the SIM-ONLY ground-truth bridge (ROS 2 env sourced):
cd ~/workspace/HOPE/a3_deploy/a3_deploy_example
python3 scripts/oracle_pose_bridge.py --topic /sim/a3/pelvis_pose --shm /dev/shm/pp_oracle_pelvis

# Terminal 2 — runner with oracle:
./a3_deploy_onnx_ref_pingpong --runtime-cfg $RT --start shadow --level 1 \
    --oracle-pelvis --obs-csv /tmp/obs_C_oracle.csv
```
Expect the big SIM-ONLY warning, then `[obs] oracle(en=1 fresh=1 age=~0.01s)` and
`motion_anchor_pos_b |.|` becoming the *true* (small) tracking error. In MOTION the
sim swing should be clean. Save `/tmp/obs_C_oracle.csv`.

## Step 4 — the fix (mode B, hardware-safe) — no oracle, no fake error

```text
./a3_deploy_onnx_ref_pingpong --runtime-cfg $RT --start shadow --level 1 \
    --perfect-tracking --obs-csv /tmp/obs_B_perfect.csv
```
Expect `motion_anchor_pos_b ≈ 0`. Save `/tmp/obs_B_perfect.csv`.

## Step 5 — compare and decide

```text
python3 scripts/analyze_obs_log.py \
    A=/tmp/obs_A_fabricated.csv B=/tmp/obs_B_perfect.csv C=/tmp/obs_C_oracle.csv
```
Read the **`motion_anchor_pos_b |.|`** and **`|Δaction| (jerk/buzz)`** rows:

- **A buzzes** ⇒ A has large anchor `|.|` **and** high `|Δaction|`.
- **C clean** ⇒ low jerk ⇒ the **policy/ONNX is correct**; the gap is localization.
- **B ≈ C** ⇒ the perfect-tracking approximation is good enough for hardware staging.

Confirm in the fixed (B) run: `motion_anchor_pos_b ≈ 0`; obs still 180-D; action
still 31; joint order still `MakeA3Layout31()`; command topics unchanged; the sim
`/sim/a3/right_racket_pose` follows the intended swing; buzz removed/strongly reduced.

If B does **not** match C closely, the residual is real localization sensitivity →
a base-pose estimator is needed before hardware (do not ship B alone).

---

## Hardware gate (do NOT skip)

Only after Step 5 passes in sim, and with `loc_mode: perfect_tracking` set for
deploy (`obs_debug.loc_mode: perfect_tracking`, or `--perfect-tracking`):
`--dry-run` → `run_a3_probe.sh` → zero-gain → low-gain (`--gain-scale 0.4`) →
level 0 → level 1, hoisted, e-stop in hand. **Never** pass `--oracle-pelvis` or run
`oracle_pose_bridge.py` on hardware.

## Remaining risk before hardware

- **B is an approximation.** It assumes perfect *position* tracking; large real base
  drift is invisible to it. The sim B-vs-C gap quantifies this. If non-trivial, add a
  base-pose estimator.
- **Actuator tracking** (a separate buzz source): if the swing is too fast for the
  real actuators, use `--swing-speed <1.0` and `--gain-scale`. The sim uses ideal
  actuators, so it under-tests this — re-check at low gain on the hoist.
- **Transport**: rehearsal uses the pure-iceoryx cfg; hardware uses the rebuilt
  ros2 cfg. Re-confirm `/body_drive/*` rate ≈ expected (no `rate=0`/safe-halt).
