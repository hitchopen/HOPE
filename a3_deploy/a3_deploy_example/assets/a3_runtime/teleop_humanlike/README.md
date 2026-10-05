# HumanLike runtime assets

This directory supplies the AgiBot HumanLike runtime configuration and inference
models consumed by HOPE's native `PpHumanLikePolicy` adapter:

- `humanlike.yaml`: joint ordering, gains, scaling and gait configuration.
- `lin_vel_encoder.onnx`: `[1,1424]` observation history to `[1,3]` velocity latent.
- `policy.onnx`: `[1,1427]` actor input to `[1,25]` actions.

These vendor-origin files are preserved byte-for-byte as supplied for the A3
integration. They are separate from HOPE's published model_21800 receive policy
and do not include unpublished HOPE checkpoints or training code. Vendor-origin
materials retain their applicable vendor terms; see the repository's A3 asset
and vendor software notices. No vendor `motion_control` binary is required by
the native adapter.

The YAML also contains a vendor `dataset_file` entry. The HOPE native adapter
does not read that dataset; only these three runtime files are opened. No
`walk_data_all.csv` dependency is required for this adapter.

The public package builder includes this directory by default as
`teleop_humanlike/`. Use `--teleop-policy-dir PATH` to select another compatible
bundle or `--without-teleop` for a package that deliberately omits locomotion.
