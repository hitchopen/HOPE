# Public A3 runtime assets

The supported public package uses the model_21800 receive policy at
`../../models/model_21800/policy/` (relative to this directory). Its ONNX and
`params/deploy.yaml` are published together and remain unchanged.

This directory supplies the runtime assets required by the shared Runner:

- `serve/motions/`: six published SDK-order playback CSVs, including the default
  `a3p_op3_serve025_photo_right30_advance20_v12.csv`.
- `teleop_humanlike/`: `humanlike.yaml`, actor and velocity encoder used by the
  native Xbox locomotion adapter; included by default in public packages.
- `motions/pp_serve_v1_fixed.csv` and its manifest: retained public legacy clip.

Build with `scripts/build_a3_deploy_pkg.sh` from `a3_deploy_example/`. It selects
the public ping-pong YAML and model_21800 by default. `--without-teleop` explicitly
omits the HumanLike assets. The older generic A3 configs refer to additional
vendor models/RKNN/reference datasets; those are not dependencies of this public
ping-pong package and are not the default public workflow.

## Current attended serving assets

- `serve/motions/a3p_op3_serve025_smooth_center_v14.csv`: current
  Kernel-mode serve; smooth right-arm spline with a centered simulated return.
  All 468 frames play before the return to Stand. Non-right-arm fields and
  release timing are preserved. v12 remains a historical reference; v13 was
  rejected after hardware jitter feedback.
- `robots/A3PingPong-with-gripper/`: reviewed URDF and all referenced meshes;
  see its README for the 20 visual-mesh collision fallbacks.
- Timing and validation limits are in
  [Runner and serving](../../../../docs/operations/runtime_xbox.md).

See [Runtime operation](../../../../docs/operations/runtime_xbox.md) for installation
and package build instructions.
