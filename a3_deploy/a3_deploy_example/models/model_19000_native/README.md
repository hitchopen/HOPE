# model_19000_native — metadata candidate; native ABI unresolved

**NOT NATIVE READY. Formal Gate3 NOT RUN.** This sibling directory preserves model19000's actor computation while adding authoritative saved metadata and rebinding its nominal deployment YAML. It is a diagnostic candidate, not a corrected/qualified native policy or a fix consisting of joint_names alone.

- Run: `gru-formal-2b8c09938a`; training source `33c609022efbb4e85fe6a58a5a8947b0fbd46c76`.
- Original PT SHA256: `744526d2e7a9fdabe990070054ec3bd8126b286bc79041f94a930b0078c257c0`.
- Original ONNX: `f3f56fe7e0c31f94bea1645fae8d2f776dca01033f2c895d850bf5c708801dda`.
- Original deploy.yaml: `fd7e5361e336791f734e24dcc2f8837319665d5f88748a6bf2c04b6c639f3e51`.
- Candidate [policy/exported/policy.onnx](policy/exported/policy.onnx): `1a2d468cd4c063345b6c725687ffaa5dfb5f9365c54807668c562f5c87a8f1da`.
- Candidate [policy/params/deploy.yaml](policy/params/deploy.yaml): `1ceacf5caf2e9b43e3478c69d8dbb913b62505fafdc62cf6736dc2f3472dd2b7`.

The original directory, deployment files, actor ZIP and checkpoint remain unchanged. Large evidence is outside Git under `~/workspace/hope-stationary-gru-evaluations/20261108-model19000-native-gate3/`. The before/after inventory covers every original model19000 file, including existing bytecode; the PT is rehashed separately. No prior result, Google Drive handoff or training checkout is changed.

## What was verified

Only genuine values from model19000's frozen config and its checked checkpoint-value report are added: joint/body names, nominal defaults, affine decode, PD, hard/safe intervals, feedback, ordered observations, timing, passive-head and training recipe/runtime metadata. The existing pure `sim2real_metadata` function validates and exports its own saved-config fields. Clip lengths come from the actual hash-bound forehand/backhand motion NPZ assets, and strike phases/position/velocity envelopes use the saved values and existing exporter formatting. These assets are available; genuine native reference integration is unimplemented, not an assertion that the assets are missing. Metadata does not fabricate clock/reference outputs, change recipe identity to another ABI, or claim runtime support.

The original protobuf field7 GraphProto and every original nonmetadata ModelProto byte are identical. Learned initializers, operations, graph inputs/outputs and opset declarations are unchanged; only valid ModelProto metadata_props records are appended. ONNX Runtime loads both and exposes the expected metadata and `obs[1,110],h_in[1,1,110] -> actions[1,31],h_out[1,1,110]` float32 signature. This is ORT structural validation, not a claim that unavailable host `onnx.checker` was run.

Bounded original/candidate inference parity:320 observations (64 saved PyTorch-export fixtures plus256 archived real rally observations spanning the run), nonzero hidden start, zero resets at32/192; maximum action/hidden error **0/0** at atol2e−5,rtol2e−4; exact zero-reset repeat passed. No new physical evaluation was performed. This verifies the metadata edit, **not** native session memory handling. The native carry/reset implementation remains absent and untested.

YAML differs from the original only in provenance/hash binding. Its nominal saved defaults and wire damping are retained. The historical host bundle uses randomized environment0 defaults and passive-inclusive total damping; those are neither copied into the candidate YAML nor silently used for native control. Same-observation actor parity does not erase this intentional configuration distinction.

## Native contract audit and unresolved integration

The actual policy loader is `PpOnnxPolicy` in `pp_onnx_policy.hpp`, production strict profile, invoked by `a3_pingpong_main.cpp --inspect-policy`. It requires joint_names, body_names, default_joint_pos, action_scale and PD metadata before later action/clock/recipe checks. The full audit inventories mandatory literal lookup sites across all supported branches; not every key belongs to model19000, and dynamic branch checks also exist. Missing optional keys may activate or reject contracts; they must not be filled with arbitrary strings to advance loading.

More important than metadata:

1. `PpOnnxPolicy::Run` currently supplies **obs,time_step**, not h_in. It has no h_out storage, carried GRU memory or reset integration. A genuine native recurrent path must validate the explicit state shape/type, zero at real session start, update once per actor tick, carry across serves/HOLD/recovery, and preserve feed-forward behavior.
2. `PpPolicy::ComputeCommand` calls `onnx_.refs(time_step)` unconditionally; construction/geometry also uses `reach_offset_from_refs`. The native contract requests genuine **joint_pos, joint_vel, body_pos_w, body_quat_w** reference outputs. The frozen actor has none. The training reference exporter obtains these from the actual MotionLoader. Returning zeros, default poses or fabricated successful trajectories is forbidden. A real wrapper/reference integration must use the exact saved motion assets/clock semantics and preserve identical actor/h updates.
3. Reference-only calls must not advance GRU memory. The real session handoff is `rearm_yaw_align()` from PASSIVE/PD_STAND into policy modes, distinct from per-ball swing-clock resets. A new native recurrent reset hook must be attached to the former, never the latter. Existing previous-action passive-head-zero feedback and direct affine-safe decode must remain intact.
4. Inspection receipt alone does not test actor calls: current `--inspect-policy` constructs the policy and prints a receipt. Actual native sequential/reset parity and helper-output integration must pass before a formal simulation, even if metadata inspection eventually succeeds. A label such as ball_clock_110_v1 must not be used to conceal an unsupported actor-state ABI.

No native validation/scoring checks were weakened and no uncompiled partial change to the feed-forward Runner was presented as GRU support. These integration items remain **unimplemented**, not merely awaiting a passing test.

## Actual Linux loading / Gate3 availability

The documented default package in this evaluation checkout is absent, as is its expected `hope_ws/install/local_setup.bash`. The documented Gate3 Python `--check` implementation was actually called and exited2 with `installed runtime asset missing: .../dist/a3_deploy_x86_64/a3_deploy_onnx_ref_pingpong`. It never reached native policy loading or simulation.

The only installed Runner located was the sibling checkout's old package, binary SHA256 `f4e2da91fbaf19d9f90c2dc03b8b77e1d91015203501a44e96885a5dfd6d703c`. It lacks both `--inspect-policy` and `POLICY_INSPECTION_FAILED` binary markers. It was **not invoked** with an unsupported flag, to avoid falling through into normal transport/hardware initialization. A loader-only `ld-linux --list` check (never enters main) exited127 for missing `librcutils.so`. The CMake cache identifies the sibling source path, but no source-bound build receipt proves its current Git revision; source and binary identity are recorded separately. Its current native header also differs from this checkout's header.

The found sibling simulator binary SHA256 is `80cc137bd7108ef50eb66e050bf78d7ad8f40a04a20dc3c18bdc2ce457fccc64`; that does not prove it is the current instrumented Gate3 plant. Existing `hope` container access fails with `chmod /run/user/1001/libpod: read-only file system`. No permissions, dependencies or containers were changed. The sandbox process list cannot establish whole-machine idle; no unrelated processes were killed. No ROS graph/hardware bridge was launched.

The WSL joint_names inspection failure supplied by the user is historical evidence; this task did not reproduce a Linux native policy-inspection exit2. The Linux exit2 above is a missing-runtime setup failure. **Actual Linux native policy loading: NOT EXECUTED/NOT PROVEN. Formal Gate3 certification: NOT RUN.** No launches, contacts, falls or certification metrics are invented.

## Reproduction / continuation

`prepare_candidate.py` reuses completed files only if bytes match, never overwrites a different candidate, checks original hashes, and performs the bounded parity checks. `audit_and_check.py` preserves each failed preflight in a new directory, hashes sources/binaries, guards unsafe old binaries, and invokes the actual documented check implementation. Neither runs a custom host physical evaluator.

One normal desktop command retries completed-stage-aware preflight:

```bash
bash "$HOME/workspace/hope-stationary-gru-evaluations/20261108-model19000-native-gate3/resume.sh"
```

It uses existing environments and stops on access/loading errors. **It does not bypass the unresolved native integration**, even if a newer inspection binary is supplied. The current launcher deliberately holds formal simulation until genuine GRU/reference integration and native sequential/reset verification exist; it is not an automatic certification launcher for this partial candidate.

Once those real implementation/verification requirements are satisfied, the formal command must remain the documented native chain with the unmodified scenario:

```bash
./run_gate3.sh --policy-dir "$PWD/a3_deploy/a3_deploy_example/models/model_19000_native/policy" \
  --scenario "$PWD/a3_deploy/a3_deploy_example/config/gate3/scenarios/fixed-home-100.yaml" \
  --headless --output /absolute/path/to/a/new/formal-result
```

The audited protocol keeps20s initial idle, six clean serves, native rapid1.15s slots, reference thresholds, no inter-ball rescue/reset and complete flight draining. Use the actual native runtime's process/channel guards. Preserve exact two-file/config/source/binary snapshots and all individual native reports; only native run.json.certification_pass plus successful exit/report evidence can establish this scenario's simulator certification, never real-robot performance.
