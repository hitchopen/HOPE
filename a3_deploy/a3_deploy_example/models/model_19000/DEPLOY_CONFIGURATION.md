# model_19000 deployment configuration

**Configuration exported and checked against saved training settings.**
**Receiving hardware runner compatibility: NOT YET VERIFIED.**

[deploy.yaml](policy/params/deploy.yaml) · [value-by-value verification and sources](deploy-config-verification.json) · [configuration-only exporter](config_export/export_config.py)

Run `gru-formal-2b8c09938a`, checkpoint `model_19000.pt`, source commit `33c609022efbb4e85fe6a58a5a8947b0fbd46c76`.

- Checkpoint SHA256: `744526d2e7a9fdabe990070054ec3bd8126b286bc79041f94a930b0078c257c0`.
- Unchanged actor ONNX SHA256: `f3f56fe7e0c31f94bea1645fae8d2f776dca01033f2c895d850bf5c708801dda`.
- deploy.yaml SHA256: `fd7e5361e336791f734e24dcc2f8837319665d5f88748a6bf2c04b6c639f3e51`.

This is a companion file export; the original actor ZIP, runtime.json, manifest and ONNX are unchanged. Download deploy.yaml and these notes separately from the original actor ZIP. No new model weights, inference run, simulation, training or hardware operation was performed. Model21800 supplies no parameter values.

## Configuration source and conventions

The document uses the repository's existing `unitree_rl_lab.deploy` format_version1, `a3_pingpong_hitter_pure` task plugin, and existing field names. The documented standalone `export_deploy_cfg.py` is absent from this feature checkout. Its existing configuration-builder source was recovered read-only from cached Git history at commit `51a9f098dd990a2d59f0c1fdd831d5103fae18fa`; a verbatim reference is retained in `config_export/reference_export_deploy_cfg.py`. The configuration body and its validation/output logic run independently of its ONNX-metadata reader. Only the three initial ONNX-read/metadata/dimension statements are replaced with verified saved values and actual graph dimensions. This adaptation is configuration export, not certification of a two-file runtime ABI.

The GRU graph lacks the legacy metadata required by that standalone script; the separate stateless `scripts/export_onnx.py` rejects recurrent actors. The configuration-only adapter therefore reconstructs metadata **in memory** from this exact checkpoint plus frozen YAML and the training source's contract functions; it never adds metadata to or rewrites ONNX. Torch/Isaac/container access is unnecessary: a restricted, lazy ZIP/pickle storage reader extracts only needed numeric rows after checking the full checkpoint hash. No arbitrary Python pickle globals or CUDA loading are permitted.

- Joint policy order is the saved checkpoint action order. `joint_ids_map[i]` maps that policy joint to the repository SDK order. Default positions, action scales/offsets and limits are policy-ordered; `stiffness`/`damping` are SDK-ordered, as the native parser requires.
- **Defaults and action offsets are nominal**, from frozen `env.yaml: scene.robot.init_state.joint_pos`, converted to float32 as training stores them. The existing recurrent evaluation manifest captured randomized environment0 defaults; those differ by up to the saved startup±0.01rad distribution. Per-joint differences are recorded in the report. That historical evaluation bundle is deliberately unchanged. Do not assume its runtime.json defaults equal this nominal deployment YAML.
- Kp uses checkpoint nominal `default_joint_stiffness`, checked against frozen actuator configuration. Exported Kd is the training source's **wire damping**; the saved passive-inclusive nominal damping equals wire+passive on every joint. Sending total training Kd would double-count the deployment plant's passive damping. All three arrays are recorded separately. This is an exporter convention, not measurement of the receiving robot's damping.
- Action decoding is `clip(nominal_offset + scale * raw_action, saved_safe_lower, saved_safe_upper)`, using saved V11 affine-safe contract. The graph masks passive head action columns11/16 to zero. Previous-action observation feedback is the applied raw action with those passive columns zeroed, not q_des. Hard limits, safe limits and actual-q tolerance come directly from checkpoint action fields and saved action configuration.
- The actor period is20ms (50Hz); original training physics is5ms×4, not the physical evaluation override of1ms×20. Training delay cohorts0–1control tick (nominal fraction0.5) are recorded in the report; the existing sim2real contract specifies direct deployment with no artificially added delay. No unsupported delay keys are invented in YAML.
- Observation terms preserve saved order, dimensions and history: gyro3, relative q31, relative qd31, last raw action31, projected gravity3, base forwardXY2, base target deltaXY2, racket target relative-base3, target velocity3, time-to-strike1: total110. Training history_length0 exports as one current sample, per the standard exporter. Saved term scale/clip are null, exported identity/no clip; graph normalization is identity. Training corruption/mocap stochastic distributions are not injected by this file.

The existing YAML schema does not carry recurrent state or actuator effort/armature/friction/default-velocity arrays. Those verified checkpoint values are retained as evidence in `deploy-config-verification.json`, **not invented loader keys**. The full receiving runtime's treatment of these quantities still needs verification. No required position/PD/mapping/control/observation field was unavailable.

## Explicit recurrent graph interface

| Direction | Name | Type | Shape |
|---|---|---|---|
| Input | obs | float32 | [1,110] |
| Input | h_in | float32 | [1,1,110] |
| Output | actions | float32 | [1,31] |
| Output | h_out | float32 | [1,1,110] |

Zero hidden state once at genuine episode/session start, then feed every h_out into the next h_in across control steps, serves, HOLD and recovery. Reset only at a genuine new episode/session reset. Never load training-batch memory or reset per control step/ball. ONNX Runtime inspected these names/shapes/types read-only; no inference was required for this file export.

The checked-in native policy runner expects its own feed-forward obs/time_step interface and does not implement this GRU carry/reset contract. Having a valid deploy.yaml or the same110-D observation size does not make that runner compatible. This document is not a new loader schema, a hardware authorization, or a claim about the Shanghai runner. Receiving hardware runner compatibility remains **not yet verified**.

## Reproduce this file export

From the evaluation repository root, using the existing host Python:

```bash
"$HOME/workspace/nohope/.venv/bin/python" \
  a3_deploy/a3_deploy_example/models/model_19000/config_export/export_config.py \
  --checkpoint "$HOME/workspace/hope-stationary-gru-evaluations/20261106-eight-checkpoint-comparison/model_19000/model_19000.pt" \
  --params "$HOME/workspace/hope-stationary-gru-evaluations/20261106-eight-checkpoint-comparison/model_19000/params"
```

The script verifies exact PT/ONNX/config hashes before export, frozen/training-source values, all31 joint mappings and arrays,110 observation dimensions, YAML round-trip and graph interface/hash afterward. Paths can point to the same frozen checkpoint/config copied elsewhere. No simulator or receiving-hardware confirmation is needed.
