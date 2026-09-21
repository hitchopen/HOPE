# Public A3 runtime assets

The supported public package uses the model_21800 receive policy at
`../../models/model_21800/policy/` (relative to this directory). Its ONNX and
`params/deploy.yaml` are published together and remain unchanged.

This directory supplies the runtime assets required by the shared Runner:

- `serve/motions/`: six published SDK-order playback CSVs, including the default
  `a3p_op3_serve025_new_build4_deep_1p07_compact50_lowdrop35_strikewindow180_full31_balanced_face20deg_forwardhit_v4.csv`.
- `teleop_humanlike/`: `humanlike.yaml`, actor and velocity encoder used by the
  native Xbox locomotion adapter; included by default in public packages.
- `motions/pp_serve_v1_fixed.csv` and its manifest: retained public legacy clip.

Build with `scripts/build_a3_deploy_pkg.sh` from `a3_deploy_example/`. It selects
the public ping-pong YAML and model_21800 by default. `--without-teleop` explicitly
omits the HumanLike assets. The older generic A3 configs refer to additional
vendor models/RKNN/reference datasets; those are not dependencies of this public
ping-pong package and are not the default public workflow.

See [Runtime operation](../../../../docs/operations/runtime_xbox.md) for installation
and package build instructions.
