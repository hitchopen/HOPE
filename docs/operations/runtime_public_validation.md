# Public runtime port validation

This report covers the public runtime branch based on `origin/main` at
`a1472d3af`, evaluated on 2026-09-20. It is a source/package and local simulation
review, not a robot deployment or physical rally qualification.

## Publication scope

The branch contains selected Runner, Foxglove, Planner, localization, simulator,
build and operating-documentation updates. It does not merge private branch
history. Existing public `hope_training/`, `experiments/` and
`a3_deploy/a3_deploy_example/models/` are unchanged from the main baseline.
No new training configuration, checkpoint, private experiment output, vendor
HumanLike weight, hardware calibration receipt or generated binary is added.
The serve CSVs are controller playback assets needed by the published runtime.
Runtime source retains metadata-driven support for multiple observation contracts;
that source is not a distribution of the corresponding trained policies.

The packaged default remains `model_21800`. Both packaged files were compared
byte-for-byte with the public source:

| File | SHA-256 |
| --- | --- |
| `exported/policy.onnx` | `6bf1a2418f8538e23577a0153f2fe6a1e78dee91f41650a232259432a84a4dc8` |
| `params/deploy.yaml` | `1c463c536211933e00b4393937add4a9fc0e60a8b682ca6486cde72dfb565cb0` |

## Build and automated checks

| Check | Result |
| --- | --- |
| Native x86 Runner, AimRT and ROS interfaces | Built successfully |
| Public package staging and default policy comparison | Passed |
| Foxglove, bringup and package Python tests | 292 passed, 1 skipped; 392 subtests passed |
| Foxglove extension build | Passed |
| Xbox status tests | 6 passed |
| Native C++ suite | 395 passed across suite and path-corrected rerun, 8 skipped; 1 unavailable fixture |
| Planner CTest targets | 11/11 passed |
| NatNet/marker association/clock CTest targets | 7/7 passed |
| Isolated ROS Xbox relay integration | Passed |
| Patch whitespace check | Passed |

The native suite initially had two AimRT configuration-path failures when run
from `a3_deploy_example/`. Both pass when run from the expected `a3_deploy/`
working directory. `FK.TestFKAndGlobalVelocities` requires the absent legacy
`reference/bones_072925_test` data and was not validated; the full suite is not
reported as entirely passing. Eight tests require optional fixtures or a
particular AimRT build/runtime mode and skip explicitly.

ROS checks used a separate localhost-only domain. No robot command, service
restart, training job or Foxglove extension installation was performed.
The x86 build used a locally available dependency cache; a fresh machine still
needs the documented dependencies. An ARM package and real hardware behavior
were not revalidated as part of this port.

## Repeated serve / Ready simulation

The harness uses the public model, the default forward-hit CSV, native action
admission, source-command capture, all five command fields, the production
handoff at source frame 110, a 1.0 s stand return and a 0.5 s receive blend.
It feeds both IMUs and the measured plant state. Normal Play supplies calibrated
base-pose packets; Kernel Mode uses its local odometry. It does not simulate a
physical ball, network delays, gripper transport or the complete Runner process.

| Case | Result | Evidence |
| --- | --- | --- |
| kernel_0_wait0 | PASS | 5 completed cycles; peak tilt 0.0541 rad |
| kernel_0_wait0.1 | PASS | 5 completed cycles; peak tilt 0.0672 rad |
| kernel_0_wait3 | PASS | 5 completed cycles; peak tilt 0.0576 rad |
| kernel_90_wait0 | PASS | 5 completed cycles; peak tilt 0.0538 rad |
| kernel_90_wait0.1 | PASS | 5 completed cycles; peak tilt 0.1007 rad |
| kernel_90_wait3 | PASS | 5 completed cycles; peak tilt 0.0581 rad |
| kernel_169.2_wait0 | PASS | 5 completed cycles; peak tilt 0.0536 rad |
| kernel_169.2_wait0.1 | PASS | 5 completed cycles; peak tilt 0.0817 rad |
| kernel_169.2_wait3 | PASS | 5 completed cycles; peak tilt 0.0609 rad |
| normal_0_wait0 | PASS | 5 completed cycles; peak tilt 0.0541 rad |
| normal_0_wait0.1 | FAIL | plant fell at 24.680s |
| normal_0_wait3 | PASS | 5 completed cycles; peak tilt 0.0611 rad |

The 11 passing cases complete 55 serves. Every checked entry preserves
`q_des`, `dq_des`, feed-forward torque, `kp` and `kd` exactly on the first tick.
**The failing Normal Play 0.1 s Ready case remains unresolved.** It loses balance
while preparing a later serve. Do not interpret continuous commands, other
passing dwell times, or the Kernel results as proof that every interrupted loop
is stable. A comparison against main's receive policy produced nearly identical
commands on identical input; the cause is not established by that comparison.

Reproduce the complete matrix from the repository root (install MuJoCo and
NumPy in the selected Python environment; provide ONNX Runtime for the C++ build):

```bash
export ORT_ROOT=/path/to/onnxruntime-linux-x64-1.19.2
bash a3_deploy/a3_deploy_example/scripts/build_runner_transition_probe.sh /tmp/hope-transition.so
python3 a3_deploy/a3_deploy_example/scripts/validate_serve_loop_mujoco.py \
  --library /tmp/hope-transition.so \
  --policy-dir a3_deploy/a3_deploy_example/models/model_21800/policy \
  --output /tmp/hope-serve-loop.json
```

The harness writes a result for every case and exits nonzero when any case
fails. Keep that failure visible when reviewing this branch.

## Public/main compatibility decisions

- Preserve main's 24-sticker optical-center layout and receipt validation; add
  distinct legacy V2/V3 services and P1/P2 transforms. A generated 24-marker
  capture/receipt/decoder test exercises the round trip and wrong-layout rejection.
- Preserve main's supervised PTP/chrony baseline and bounded NatNet parser;
  integrate marker aliases, association, unicast preflight and UI time controls.
  Some servers advertising NatNet 4.5 emit an older unsized wire layout. The
  preflight can identify it, but the main backend's sized-4.5 decoder has not been
  extended or qualified for that variant in this port.
- Gate3 creates a clearly labelled simulation-only calibration receipt rather
  than copying a field robot's calibration into the public repository.
- Teleop control code is included. The optional vendor HumanLike models must be
  supplied separately by the operator; the default package cannot enter learned
  Teleop without them. No vendor policy redistribution is implied.

See [Runtime operation](runtime_xbox.md) for installation and controls.
