# 2026-10-20 batch 1: Stand, serve CSV and deployment consistency

This batch restores the field Stand entry and carries the v23 serve bundle from
the field checkout into the public source layout. It does not include the
shared-HOME, runtime-plane, ball prediction or packetizer changes.

## Stand

PD Stand again uses `--motion-blend-sec` (default 0.5 seconds) for both pose/gain
blending and entry-velocity decay. The separate 5 second PD Stand ramp and its
0.25 second velocity decay are removed. The field operator reported insufficient
support with that ramp. Policy-entry and authored serve-transition timing remain
separate. This restores the field behavior; it does not qualify physical Stand
on another robot.

## CSV and control contract

The default is
`a3_deploy/a3_deploy_example/assets/a3_runtime/serve/motions/a3p_op3_serve025_v23_branch110_model14600.csv`.

SHA-256: `9ba0112eb721c571610bf9f42ac1d9b1ea2c441a1d882d133907fd76fa54d044`.

The original 468-frame v20 joint position, velocity and acceleration table is
retained; frame 110 gains the `SERVE_TO_POLICY` event. The parser rejects missing,
duplicate or displaced branch events and inconsistent bundle markers. Older
five-field v21/v22 markers and legacy CSVs remain readable.

| Segment | Samples | Duration | Destination |
| --- | ---: | ---: | --- |
| STAND_TO_SERVE | 501 | 5.00 s | Both modes |
| SERVE_TO_STAND | 358 | 3.57 s | Kernel, original frames 110–467 |
| SERVE_TO_POLICY | 87 | 0.86 s | Normal, 0.76 s movement + 0.10 s hold |

Normal requires the measured-ready gate before its 0.15 second impedance blend.
The CSV owns branch and transition timing; conflicting command-line overrides
are rejected.

**Normal v23 requires the exact model14600 policy/configuration below.** Keeping
the repository's default model21800 package selection does not satisfy this
binding. Supply the matching `--policy-dir` when building a v23 Normal package,
or explicitly select a compatible legacy CSV. No policy weights, model default,
hardware profile, network setting or deployment destination is changed by this
source batch. A mismatched v23 Normal start fails preflight; it must not silently
fall back to another CSV or actor.

| Input | SHA-256 |
| --- | --- |
| `policy/exported/policy.onnx` | `a987240610bb9a49ea42da8eafcdc84c3cfbc8d5470bf7ff1a2d7f1f624cdd28` |
| `policy/params/deploy.yaml` | `c6f60914d30c688ec1f262bdb048eba234aa032ed2c094998ce8d4dcf46a3632` |

The adjacent recipe reproduces the CSV using the retained v20 archive input.
v22 and its recipe are included for comparison; v21 is a parser regression
fixture. Existing legacy motions remain available for compatibility.

## Deployment consistency

Previously Normal discarded the uploaded selection and could load an older
built-in CSV. Normal and Kernel now validate and pass the same selected path.
The package builder, wrapper, native default, lifecycle and Console agree on
v23. Previously uploaded files are revalidated against the currently installed
Runner. Lifecycle preflight checks parser support, Stand compatibility and
Normal policy binding before HAL starts. Session receipts and native logs
identify the actual CSV path/hash and selected return segment.

Install the matching Runner, assets, helper, field backend and Console together
during a separately authorized stopped upgrade. Older Runners reject v23.
Creating the PR does not deploy or restart any robot.

## Evidence and limits

Historical field receipt, 2026-10-20: binary
`cd5621e2fc35218f9dcbbea7e0c545702fe439b30e286450cff4c12a9ca596fc`
loaded v23 with
`legacy_pose_gain_blend_sec=0.500 entry_velocity_decay_sec=0.500`
and started to PASSIVE. Physical Stand was not retested by the agent.

The earlier field simulation used native controllers and a simulation clock.
Three Normal cycles, three Kernel cycles and yaw/pitch perturbations passed
on the base MuJoCo plant. The augmented OP3 plant failed the ready gate.
The base same-ball rally used scripted jaw clearance and an opposing paddle;
receive-policy torque requests exceeded actuator limits and were clipped.
Those historical results do not establish ROS/DDS or physical serve/receive
qualification, nor do they validate the separate Planner no-target problem.

PR validation is rerun on the extracted source: native controller/transition/
runner-control/parser regressions, field-operation and asset tests, Console
tests/type checking, shell syntax, and byte-identical recipe regeneration.
It is distinct from the historical simulation and installed field binary.

## Reproduction

From the repository root, in an environment with Python/NumPy:

```bash
python3 a3_deploy/a3_deploy_example/scripts/generate_serve_segments.py \
  --recipe a3_deploy/a3_deploy_example/assets/a3_runtime/serve/motions/a3p_op3_serve025_v23_branch110_model14600.recipe.json \
  --output /tmp/hope-v23.csv
sha256sum /tmp/hope-v23.csv
python3 -m unittest discover -s foxglove/tests -p test_field_operations.py
bash a3_deploy/a3_deploy_example/scripts/test_serve_segments.sh
```

The native test script requires a C++20 compiler, Eigen3, GoogleTest and yaml-cpp.
The optional MuJoCo validator uses the native probe ABI and reports whether
its clock is simulation time or wall time; wall-time fast-loop sweeps must not
be treated as repeated-cycle timing qualification.
