# Ping-pong deployment and Gate 3 entry points

Updated 2026-10-11. Start with the
[complete command sequence](../../docs/operations/deploy_and_gate3.md): select
policy → package x86 Runner → build/install ROS and native simulator → configure
paths → inspect → explicitly run → read retained reports → cross-build/stage.

From the repository root after the native environment is installed:

```bash
./run_gate3.sh --help
./run_gate3.sh --policy-dir /absolute/path/to/policy --check
./run_gate3.sh --policy-dir /absolute/path/to/policy --scenario fixed-home --headless
```

The last command runs a 26-ball native physical-ball closed loop. `--check`
creates inspection artifacts only. The two-file ABI and scene restrictions,
viewer/custom-scene examples, output files and flight-drain behavior are in
[gate3_two_file_runner.md](../../docs/operations/gate3_two_file_runner.md).
The independent Python evaluator and shared-interface localization A/B/C tests
are different experiments and do not replace this run.

For hardware, use the complete package in a new immutable slot and follow
[runtime_xbox.md](../../docs/operations/runtime_xbox.md) and the
[first-deployment procedure](../../docs/operations/foxglove_first_hardware_test.md).
Do not run a second manual HAL/Runner alongside lifecycle-owned processes.
No robot installation or motion is performed by a documentation update.

## Historical shared-interface examples

The following commands describe earlier localization/standalone experiments
and keyboard layouts. Their old defaults are not the current policy-packaging
or Gate 3 entry; they are retained as text for dated reproduction.

# model_15200 ping-pong runner — run & validation guide

Separate native runner `a3_deploy_onnx_ref_pingpong` (AGI's `a3_deploy_onnx_ref`
is untouched). Front-end = our verified C++ port of model_15200 (180-obs/31-act,
neck PASSIVE). Reuses AGI's A3AimrtBackend + A3PolicyDriver + watchdog + safe-halt.

Binary: `dist/a3_deploy_x86_64/a3_deploy_onnx_ref_pingpong`
Config: `dist/a3_deploy_x86_64/config/a3_runtime_config.pingpong.yaml`
Wrapper: `dist/a3_deploy_x86_64/run_a3_pingpong.sh`

## Modes (keyboard at runtime, or `--start MODE`)
`p`=PASSIVE (limp, zero gain) · `s`=PD_STAND (hold nominal, modest PD) ·
`h`=SHADOW (compute, **no publish**) · `m`=MOTION (publish) ·
`0`/`1`=swing level (0 hold-windup, 1 forehand) · `[`/`]`=gain_scale −/＋ · `q`=quit.
Flags: `--start`, `--level`, `--gain-scale F`, `--stand-kp`, `--stand-kd`.

## A. Shared-interface MuJoCo validation (do BEFORE hardware)

Treat MuJoCo as a **shared `/body_drive/*` rehearsal**, not as a separate code
path. The same deploy runner should be exercised against sim first, then the
real robot.

`scripts/run_sim.sh` now auto-detects the locally available sim:

- default `auto`: prefer `a3_deploy/A3_MuJoCo_Sim/.../build/install` if present
  (shared `/body_drive/*` + `/sim/a3/*` oracle/reset)
- `A3_SIM_FLAVOR=standalone`: use `mujoco_sim_standalone/`
  (shared `/body_drive/*` only; no oracle/reset)

**Terminal 1 — sim**:
```text
cd a3_deploy/a3_deploy_example
./scripts/run_sim.sh --print-paths

# recommended: source sim, same shared interface plus oracle/reset topics
A3_SIM_CFG=a3_pingpong_iceoryx_cfg.yaml ./scripts/run_sim.sh

# optional: standalone fallback
A3_SIM_FLAVOR=standalone ./scripts/run_sim.sh a3_t2d5_cfg.yaml
```

Transport must still match: if the runner shows `rate=0`/safe-halt, the sim's
`/body_drive/*` topics are probably on `ros2` while the runner is on `iceoryx`
(or the reverse). Switch `A3_TRANSPORT` or pick the matching sim cfg first.

**Terminal 2 — runner**, staged exactly like your plan:
```text
cd a3_deploy/a3_deploy_example
source /opt/ros/jazzy/setup.bash
bash scripts/build_a3_deploy_pkg.sh \
  --arch x86_64 \
  --runtime-cfg src/a3/a3_deploy_onnx_ref/config/a3_runtime_config.pingpong.yaml

cd dist/a3_deploy_x86_64
./run_a3_pingpong.sh --dry-run
./run_a3_pingpong.sh --reference-playback
./run_a3_pingpong.sh --start shadow --perfect-tracking --level 1
```

**Optional closed-loop motion** on the same shared interface:
```text
cd a3_deploy/a3_deploy_example
./scripts/run_mode.sh B 10 0.4

# source sim only:
./scripts/run_oracle.sh
./scripts/run_mode.sh C 10 0.4
```

`run_mode.sh` also passes through extra runner flags, so ground-support tuning
can be rehearsed without changing entrypoints:
```text
./scripts/run_mode.sh B 10 0.4 \
  --auto-leg-hold --leg-gain-scale 0.5 --ankle-gain-scale 1.0
```

## B. Compare against Python wbc_runner / sim2sim
- **stand-hold:** PD_STAND pose ≈ nominal (hip −0.131, knee 0.247, …).
- **q_des continuity:** no jumps tick-to-tick (status `|act|` smooth).
- **neck stability:** head must NOT buzz (neck is passive: q=0, kp=40, kd=2).
- **wrist stability:** wrists steady (low-inertia; if buzz, lower `--gain-scale`).
- **recognizable forehand:** level 1 produces a forehand swing matching the clip.

## C. Cross-build for rockchip (only after A passes)
```text
cd a3_deploy/a3_deploy_example
bash scripts/build_a3_deploy_pkg.sh --arch rockchip   # builds a3_deploy_onnx_ref_pingpong too
# stage model_15200.onnx into dist/a3_deploy_rockchip/models/ + a runtime cfg
```
NOTE: this laptop was missing the zmq dev headers; vendored to
`thirdparty/zmq_shim/include/` (zmq.h v4.3.5 + cppzmq zmq.hpp). If the rockchip
toolchain also lacks them, point its `ZMQ_INCLUDE_DIR/CPPZMQ_INCLUDE_DIR` there.

## D. Hardware — HOISTED only, staged (your sequence)
PASSIVE → PD_STAND → SHADOW(no-command) → low-gain MOTION (`--gain-scale 0.4`)
→ level 0 → level 1. Physical e-stop in hand. Neck passive. Scripted targets
only (no live planner yet). If `level 1` is fine on the hoist but ground support
fails, keep treating that as a released-leg support problem rather than a
shared-interface transport failure.
