# A3 Runtime Assets

This folder keeps the default A3 deploy runtime assets inside
`a3_deploy_example`, so a tracked-only archive of this directory can build and
package the reference deploy binary without depending on the original training
workspace.

Contents:

- `models/model_step_098000_a3.onnx`: default ORT CPU policy model.
- `models/model_step_026000_smpl.onnx`: SMPL teleop policy model.
- `models/model_step_098000_a3_fast.onnx`: A3-fast teleop policy model.
- `rknn_models/`: matching Rockchip RKNN models for the three ONNX policies.
- `motions/*.csv`: default flat A3 reference motions used by
  `reference_motion.motion_dir`.
- `remote_motions/*.csv`: independent direction-key clips used by
  `reference_motion.remote_motion_dir`; these are not appended to the normal
  playback list.
- `teleop_motions/BMD_0319_a3_filtered_20260319_164305__stand_Skeleton0.csv`:
  standing reference for MOTION idle and TELEOP startup.

The source config at
`src/a3/a3_deploy_onnx_ref/config/a3_runtime_config.yaml` refers to these files
with paths relative to the `a3_deploy_example` root.
