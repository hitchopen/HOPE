# G07 MuJoCo-To-Real Deployment

> For commands checked against the current checkout on 2026-10-11, use
> [deploy_and_gate3.md](../operations/deploy_and_gate3.md). Dated reports and
> legacy wrappers below retain their original scope; they do not replace the
> current two-file scene workflow or establish a new run's verdict.

Status: Partial (first real-A3 deployment demonstrated 2026-07-02, forehand only; acceptance evidence and the mocap-in-the-loop chain still open)

## 2026-07-18 RallyV13 isolation rule

V13 now has fail-closed exporter/Python/C++ metadata checks plus an isolated candidate config and
`pp_gate3_rally_v13.sh`. This is preparation only: no V13 checkpoint has passed the four-report
Isaac selector, no V13 ONNX has been exported, and no V13 Gate3 or real-robot command has run. A
future V13 artifact must stay at `model_v13_candidate.onnx` under the candidate config until strict
Gate3 includes measured physical contact and landing evidence. It must not replace the generic V10
runtime or enter an MDU/hardware package merely because the 12-serve runner proxy passes.

## 2026-07-18 RallyV12 disposition

Fresh V12 training was stopped at `model_12700`; that checkpoint was selected from five checkpoints
using deterministic core/planner/mix evaluation and exported through the isolated
`a3_runtime_config.pingpong.v12_candidate.yaml`. Its strict Gate3 result is **FAIL (`141/18`)**.
The run stays upright and completes 12/12 lifecycle proxies with a passing synchronized plant, and
V12 fixes V11's idle-wrist tail while materially reducing waist clipping and station-x failures.
However, aggregate q-des still clips (now dominated by ankles/left shoulder), all six forehand
elbow checks fail, three station-x checks fail, four post-heading checks fail, and physical ball
contact/landing remain unmeasured. The model is a simulation-only failed diagnostic, not an MDU or
hardware candidate. Do not copy it into the generic runtime or robot package; G07 remains `Partial`
and the current generic V10 runtime stays unchanged. No real-robot command was executed.

## 2026-07-18 RallyV11 disposition

`model_11800_v11_diag.onnx` is the current simulation-only failed Gate3 diagnostic. It was selected
only after deterministic core/planner/mix evaluation and is loaded through
`a3_runtime_config.pingpong.v11_candidate.yaml`; it is not approved for MDU or hardware staging.
The prior `model_11400_v11.onnx` is preserved as a rollback/baseline asset.
The generic `a3_runtime_config.pingpong.yaml` remains on the V10 artifact. Closed-loop paired
dynamics now clears the actual Gate3 plant, and a process-isolated base relay removed the planner
callback starvation (latest run `12613/12613` poses forwarded, zero rejected/stale warnings).
Nevertheless, the strict run remains unpromotable: q-des clipping, idle left-wrist roll/yaw motion,
six station-x strike tails and missing physical contact/landing proof remain. `model_11800` cleared
the previous heading/elbow failures and improved the strict count from `147/12` to `150/9`, but the
12/12 proxy result is not a hardware or ball-return certification. No real-robot command was
executed during this work.

## Goal

Move from MuJoCo/deploy dry-run to safe real A3 execution.

This gate is about safety, timing, message layout, and controlled rollout before performance.

## Inputs

- Active deploy tree `a3_deploy/a3_deploy_example` (tracked): C++ ping-pong runner
  `a3_deploy_onnx_ref_pingpong` reusing AGI `A3AimrtBackend`/`A3PolicyDriver` (Route A), build and
  packaging scripts, runbooks.
- Older vendor reference subset under `agi/code_deployment/a3_deploy_example`; full local runtime
  assets under `vendor_assets/agibot/a3_deploy_example_full`.
- Policy artifact from G05/G06. Current direct-commissioning candidate:
  `model_10500_v10.onnx` (110-D / 31-act, RallyV10 recipe 7/runtime v2, SHA256
  `fa4277801bc6bb605e0312881173c06cbb7545b403533d6226c7db11a501ec5f`). Preserve
  `model_17400_hitter177.onnx` and `model_p4_deployparity.onnx` as rollback assets.
- A3 runtime and hardware access (MDU).

## Outputs

- No-command backend/sync verification, inference latency probe, low-gain command test, safe halt.
- Real-run logs and MDU captures.

## Related Directories

- `a3_deploy/a3_deploy_example/` — runner source (`src/a3/a3_deploy_onnx_ref/include/a3_pingpong/`),
  `PINGPONG_RUN.md`, `PINGPONG_DEPLOY_ALIGNMENT.md`, `PINGPONG_NEW_CHECKPOINT_TUTORIAL.md`,
  `HARDWARE_BRINGUP_CHECKLIST.md`.
- `docs/operations/run_deploy_dryrun.md`, `docs/operations/run_pingpong_recovery_audit.md`,
  `docs/interfaces/joint_order_and_robot_state.md`, `docs/interfaces/policy_observation_action.md`.

## Operation Docs

- [../operations/build_and_test.md](../operations/build_and_test.md)
- [../operations/run_pingpong_end_to_end.md](../operations/run_pingpong_end_to_end.md)
- ../operations/run_v17_r1_fixed3_hardware_trial.md (historical document; not included in this checkout)
- [../../a3_deploy/a3_deploy_example/README.md#验证流程](../../a3_deploy/a3_deploy_example/README.md#验证流程)
- [../operations/gate3_two_file_runner.md#结果与进程](../operations/gate3_two_file_runner.md#结果与进程)
- [../operations/setup_local_sync.md](../operations/setup_local_sync.md)

## Acceptance Criteria

- Backend and sync can run without publishing commands; inference can run without command output.
- Command layout is verified against the A3 SDK/runtime.
- Safe halt works before any dynamic motion; first real command is low-gain and bounded.
- A real swing run is recorded with its checkpoint, config, and behavior notes.

## Current State

### 2026-07-30 V17-r1 `model_16600` fixed-station qualification procedure

- The field procedure is now maintained in the standalone
  V17-r1 fixed-station hardware-trial runbook (historical document; not included in this checkout);
  §9.10 of the end-to-end runbook is only its short entry point.
- The runbook fails closed to `model_v17_candidate.onnx`, SHA256
  `019fc86c8d045d95ac0054054e0fc91b845ce712feadfeb450f9a04b04556eaa`, and includes the
  NatNet 4.1 masked-MODELDEF preflight needed before ROS publishers can be created.
- Qualification means three successful returns observed by the field team, not three attempts or
  an automatic runner count. The procedure retains operator abort, freshness, fall-guard, package,
  planner, and evidence checks.
- This documentation update does **not** advance G07: the isolated fixed3 runtime package and the
  corresponding real-session evidence are not present in this checkout, so no hardware PASS is
  claimed here.

### 2026-07-15 `model_10500_v10.onnx` direct hardware commissioning selected

- The exact recipe-7/runtime-v2 export is synced to the durable deploy asset, the x86 package is
  rebuilt, and checkpoint-specific C++/Python parity passes with max deltas `4.77e-07` (zero) and
  `1.07e-06` (seeded). The run-root recipe-6/runtime-v1 auto-export is forbidden.
- The operator chose to proceed directly to staged real-A3 commissioning. The complete current
  procedure is [§9.9 of the end-to-end runbook](../operations/run_pingpong_end_to_end.md): one
  Laptop-host deploy command, HDU-native VRPN/relay/planner, an MDU no-command SHADOW entry, then
  separate HAL ownership and PASSIVE → STAND → one low-gain backhand MOTION entries. §9.8 remains
  the expanded manual/fallback procedure.
- This is a commissioning decision, not a completed G07 claim. Fresh Rockchip/MDU identities and
  real-run logs must be pasted here after the operator executes them. The first session is
  backhand-only and requires `swing_sign=-1`; the tracked planner overlay's V8 geometry is replaced
  by fixed V10 `x_hit=settled_post-stand_base_x+0.58`, zero BH delta and the side override. The HDU
  node publishes the base flat but suppresses racket commands until the operator freezes that plane
  after `s`/PD_STAND; no planner restart or Laptop-local ROS process is involved.
- Runbook §9.8 now records all three explicit planner modes for this shared-plane model: normal
  FH/BH auto-selection (`split=-0.25`, hysteresis `0.04`), forehand-only (`split=+10`, sign `+1`),
  and backhand-only (`split=-10`, sign `-1`). Exactly one planner process may run at a time.
- The Rockchip package builder now stages separate `run_rally_v10_shadow.sh` and
  `run_rally_v10_real.sh` entrypoints plus a guarded HAL entrypoint. No script chains SHADOW into
  MOTION; the real entry preserves the exact `taskset -c 4-7 ... --planner --demo` command and
  requires an interactive SSH TTY. Shell/command checks and a fresh full Rockchip package build
  pass with the wrappers staged executable, exact model SHA, and ARM aarch64 runner; the wrappers
  are not yet field-run.
- HDU dependency bootstrap now supports the observed Debian Bookworm/aarch64 vendor image without
  mixing Ubuntu packages into `/opt/ros/jazzy`: the official ROS Jazzy VRPN release is pinned and
  built in the HDU workspace before `vrpn_mocap`. The operator's original one-command deploy entry
  remains unchanged; a fresh onboard build/verification result is pending.

Done (2026-06-30 → 2026-07-02, recorded 2026-07-03):

- **First sim-to-real transfer succeeded (2026-07-02).** The unified swing policy ran on the real
  A3 through the Route A runner with AGI's backend/`A3PolicyDriver`/100 Hz sync/watchdog/safe-halt
  reused unmodified, and real behavior matched MuJoCo. Deployed artifact:
  `model_p4_deployparity.onnx` (175-D deploy-parity obs / 31 actions / 50 Hz). See
  `a3_deploy/a3_deploy_example/SIM_FIDELITY_NOTE_FOR_AGI.md` and `a3_deploy/a3_deploy_example/PINGPONG_DEPLOY_ALIGNMENT.md` §0.
- **Forehand only.** Backhand is NOT deploy-ready on the current models (training gap:
  teleport-entry clips, no stand-entry coverage) — do not trigger backhand on hardware until
  stand-entry backhand training closes the gap (`PINGPONG_DEPLOY_ALIGNMENT.md` §0).
- The runner validates the whole contract from ONNX metadata at load (joint_names bijection onto
  the 31 backend slots, default_joint_pos, action_scale, kp/kd with a zero-gain guard, obs input
  [1,175] or [1,180], optional `clip_seg_lengths`/`clip_strike_phases`). Actions are 31-DOF
  (`MakeA3Layout31`); neck outputs are overridden post-decode to q=0/kp=40/kd=2; q_des is clamped
  to MJCF joint limits; commands publish `{q_des, kp, kd}` to the implicit-PD backend.
- Hardware-derived fixes are in code (`d17cd57`/`dd32a29`): exporter zero-kp/kd guard; clip clock
  taken from ONNX metadata (the v1-layout default fired the forehand ~0.6 s early on v2-baked
  models); squat guard raised 0.6 → 1.4 rad plus a 0.35 tilt guard (the old threshold sat inside
  the trained swing envelope and its kp-2000 snap-back catapulted the robot); IMU yaw-align capture
  at every SHADOW/MOTION engage (boot-to-boot yaw drift 130-165°).
- Staged bring-up used throughout: PASSIVE → PD_STAND → SHADOW (no publish) → MOTION, with
  `run_a3_pingpong.sh --dry-run` / probe first (`PINGPONG_RUN.md`).
- The x86_64 package build is reproduced from the tracked tree
  (`scripts/build_a3_deploy_pkg.sh --arch x86_64 --runtime-cfg
  src/a3/a3_deploy_onnx_ref/config/a3_runtime_config.pingpong.yaml`); earlier fingerprints for the
  model_15200 era are in
  [../operations/gate3_two_file_runner.md#结果与进程](../operations/gate3_two_file_runner.md#结果与进程).

Not done:

- The `model_10500_v10.onnx` Rockchip dist has not yet been built/shipped in the documented state;
  its MDU SHA, recipe/runtime boot banner, no-command dry-run, SHADOW and real backhand capture are
  pending operator execution.
- The current C++ runner does consume the planner's racket/base flat topics and fails closed on
  stale external-base data. The venue chain still has to be proven for this session:
  CMTracker/VRPN → HDU bridge/relay/planner on ROS domain 232 → MDU runner, with measured marker
  offset and fixed V10 plane. The Laptop is only SSH/TTY. This replaces both the historical
  scripted/perfect-tracking limitation and the temporary Laptop bridge layout.
- Acceptance evidence for the successful runs (dates, checkpoint SHAs, MDU capture paths, observed
  behavior vs MuJoCo) is not yet recorded in this gate.
- No quality baseline on hardware (hit-rate style metrics need the mocap/ball loop).
- Quantified `model_10500_v10` hardware backhand behavior and repeatability.

## Risks

- Joint order or command scaling mistakes are dangerous — mitigated by the metadata bijection check
  and staged bring-up; keep both mandatory for new checkpoints
  (`PINGPONG_NEW_CHECKPOINT_TUTORIAL.md`).
- Safety guards themselves can cause incidents when their thresholds intersect the trained motion
  envelope (the 0.6 rad squat-guard lesson). Derive guard thresholds from the trained state
  envelope, and prefer damped take-over behaviors to stiff position snaps.
- IMU yaw drift silently corrupts any world-referenced quantity; the engage-time yaw-align must be
  re-verified whenever the boot sequence changes.

## Next Steps

1. Execute the exact `model_10500_v10.onnx` one-command-per-terminal sequence in runbook §9.9;
   use §9.8 only for expanded diagnosis.
2. Record Rockchip and MDU hashes, boot banner, SHADOW output and the first low-gain backhand log.
3. Restore `agibot_pm` after the session and retain the previous package as rollback.
4. Define the repeatability/hit-rate protocol after the first commissioning capture.
