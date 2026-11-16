# model_19000 native GRU candidate v2

Implemented native integration candidate; **full Runner build/inspection and formal
Gate3 have not run in the Codex sandbox**. Podman cannot chmod its read-only
`/run/user/1001/libpod`. Native component parity is not a loading or certification
claim. No real-robot compatibility claim is made.

Run `gru-formal-2b8c09938a`, iteration 19000, training source
`33c609022efbb4e85fe6a58a5a8947b0fbd46c76`.

| File | SHA-256 |
|---|---|
| Original local checkpoint | `744526d2e7a9fdabe990070054ec3bd8126b286bc79041f94a930b0078c257c0` |
| Original actor (unchanged, sibling model_19000) | `f3f56fe7e0c31f94bea1645fae8d2f776dca01033f2c895d850bf5c708801dda` |
| [v2 policy ONNX](policy/exported/policy.onnx) | `2b2a029b450fb8601ecf34eefdbd3df6ab9f8597a33593dc9b7d6419fcd34d27` |
| [v2 deploy.yaml](policy/params/deploy.yaml) | `160f282fdb95d4ca5d0df5222897a203150152248fcc364f8ba723086357f3fc` |

The original model_19000 and metadata-only model_19000_native are preserved.
The checkpoint remains local. [manifest.json](manifest.json) records frozen
configuration hashes, pinned motion hashes, software versions and export checks.

## Implemented graph and runtime contract

Inputs: `obs float32[1,110]`, `h_in float32[1,1,110]`,
`time_step float32[1,1]`. Outputs: `actions float32[1,31]`,
`h_out float32[1,1,110]`, plus genuine `joint_pos`, `joint_vel`,
`body_pos_w`, `body_quat_w`, `body_lin_vel_w`, `body_ang_vel_w` reference streams.
The original actor graph fields and learned initializers are byte-preserved;
the extra clock affects only reference lookup. Observation normalization and
passive-head masking remain graph-owned. No second normalization or masking
layer is added to the actor.

`PpExplicitGru` is shared by `PpOnnxPolicy` and the native parity executable.
It validates names/shapes/dtypes/finite values, owns hidden state, and caches an
action per monotonically increasing policy tick. Repeated identical action
queries return the cache; changed duplicate queries fail. Reference queries
cannot update hidden state. The main Runner zeroes state when entering a policy
session from non-policy modes. SHADOW-to-MOTION, serves, HOLD and recovery carry
state. A real session restart clears it. GRU requires `--policy-native`; the
legacy static-standing bypass is rejected. Feed-forward inference is unchanged.

The normal native observation/action adapters remain authoritative: raw
110-D observations, existing world/base transforms and target/TTS construction,
previous-action feedback, nominal-default subtraction, 31-joint SDK permutation,
safe affine action decoding, limits and passive-head handling. The ONNX contains
the frozen actor's normalization. The strict deployment loader binds YAML values
and joint order to metadata and the exact ONNX hash. No actor inputs are added
beyond explicit state and the independent reference clock.

### Genuine references and clock

The reference provider embeds all six arrays from:

* `hope_training/motions/preprocessed/hope_forehand.npz`, 107 frames,
  SHA-256 `a6c68513720b12b168379cd6fa13f8b77607b4fa0bf7e828c4e1d81eda6f2094`.
* `hope_training/motions/preprocessed/hope_backhand.npz`, 109 frames,
  SHA-256 `67d04e13deeed068bdb003e379e18330dcd29210d280188fab7af26c0764eaac`.

The frozen training loader's body selector uses checkpoint-captured indices
`0,4,10,20,5,11,21,9,18,24,30,19,25,31`; torso anchor is tracked index 7.
Arrays retain training joint order, raw clip-world positions and wxyz
quaternions. Clips concatenate FH then BH, 50 Hz. Frozen strike phases
`0.3868,0.4444` select absolute frames 41 and 155. Native `ClipLayout` maps
time-to-strike to the nearest bounded clip frame; it does not interpolate.
The graph matches the training exporter: float clock truncates to int64, clamps
only the maximum to 215, then indexes arrays. Negative -1 selects the last frame;
native clocks stay inside the selected segment. No dummy helper outputs or
model_21800 reference data are used. All runtime reference data is embedded in
the ONNX; original NPZ files are needed only to reproduce the export.

Native HOLD reference joint positions/velocities retain the existing nominal
pose/zero-velocity override. For this 110-D policy, reference joint arrays do not
become new actor inputs; anchor orientation and motion-clock/phase semantics
remain in the existing observation builder. The native planner/lifecycle is
not claimed identical to the training command scheduler.

Deployment uses nominal defaults and wire damping. Training total damping
includes separate passive joint damping; the historical host MuJoCo bundle
also used sampled env-0 defaults. This v2 YAML preserves the nominal deployment
settings, not that historical randomized scene. Control is 0.02 s (training
physics 0.005 s, decimation 4); native latency/physics remain deployment settings.

## Verified here

* Original vs appended graph: 320 sequential CPU ORT steps, initial nonzero
  hidden state, resets at steps 32 and 192: action and hidden max errors **0**.
* Actual production-shared C++ state component, compiled with ORT 1.19.2:
  max action error **1.90735e-6**, hidden **0**, reference **0**;
  tolerance `atol=2e-5, rtol=2e-4`.
* Duplicate calls, read-only reference queries, genuine reset repeat,
  HOLD/recovery/clip-clock handoff carry, invalid calls and all 216 native
  reference-clock frames pass. Direct reference-array checks include clip
  boundaries, clamp and negative-index behavior.

This is component evidence. The full `PpOnnxPolicy` loader, feed-forward
regression, freshly built Runner inspection and formal simulation remain
guarded desktop stages; their result must be read from fresh receipts.
The feed-forward regression uses model_21800 only as an independent legacy
fixture; it is never selected for the model19000 simulation.

## Resume the full native build and formal test

From a normal desktop terminal, using the existing hope Distrobox:

```bash
python3 "$HOME/workspace/HOPE-stationary-gru-eval/a3_deploy/a3_deploy_example/models/model_19000_native_v2/run_native_gate3.py"
```

The launcher hashes the candidate, checkpoint, pinned assets, preserved originals,
native sources and parity fixtures before execution. It sources ROS Jazzy,
builds the ROS package closure, new x86 Runner package and current instrumented
MuJoCo into separate external directories. It reuses existing Unitree/ORT
dependencies and copies compatible cached source downloads; it performs no
apt/rosdep installation. Default dependency root is the existing sibling
`nohope/a3_deploy/a3_deploy_example/thirdparty`; override only its location with
`HOPE_NATIVE_THIRDPARTY` if necessary. Missing dependencies fail with logs.

Then it runs full-loader sequential/reset/duplicate/reference parity and
feed-forward regression, the fresh actual Runner `--inspect-policy`, and the
documented `run_gate3.sh --check`. These checks exit before backend startup.
Only if all pass does it invoke real native Gate3 headlessly with the unchanged
`fixed-home-100.yaml`. No custom host evaluator is used. The reference's
20-second idle, six clean shots, 1.15-second rapid slots, no-rescue rule,
outgoing-flight draining and thresholds are unchanged. Gate3 owns the global
lock/conflict checks and cleanup of its own PID ledger. An outer 900-second
wall timeout sends SIGINT to allow its cleanup; timeout is a failure, not a pass.

Evidence/builds/logs/commands/receipts are under
`$HOME/workspace/hope-stationary-gru-evaluations/20261108-model19000-native-v2`.
Verified stage outputs, including installed shared libraries, are hash-checked
before reuse. A changed source binding requires a fresh `--output` directory;
historical stages remain intact. Failed preflights get fresh run subdirectories.
A finalized formal result, including a failed certification, is retained and
not automatically rerun. `formal-gate3.receipt.json` points to the exact
`run.json`; read its `certification_pass` and underlying rally/plant/ball/planner/
drain reports. Launch count, process exit and zero falls alone are insufficient.

To reproduce graph export only (requires the existing frozen local archive):

```bash
"$HOME/workspace/nohope/.venv/bin/python" "$HOME/workspace/HOPE-stationary-gru-eval/a3_deploy/a3_deploy_example/models/model_19000_native_v2/build_candidate.py"
```

The builder executes the real metadata exporter function against saved managers,
omitting its ONNX I/O, and appends standard ONNX records without importing Isaac
or constructing a training environment. It refuses to overwrite a different v2
graph. Original-file inventories and external verification traces are retained.
