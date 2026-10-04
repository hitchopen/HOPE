# Runtime calibration receipts

`p1_to_pelvis.json` is generated on the operator laptop by the attended
**12-station v3 sticker calibration**. It contains installation-specific
geometry, timestamps, sample counts and audit evidence, so it is intentionally
Git-ignored. The robot receives `/a3/base_pose_flat`, not this JSON.

## Current shell and OptiTrack asset

Use [A3_v3_12_stickers.motive](../agibot/mocap_sticker_shell_v3/motive_asset/A3_v3_12_stickers.motive),
internally named **P1**, for the retained stations **S04, S05, S11, S12, S13,
S14, S16, S18, S19, S20, S21, S22**. Do not renumber the physical station IDs.
The [sticker optical-centre table](../agibot/mocap_sticker_shell_v3/documents/marker_transforms_pelvis_link_stickers.csv)
is already in ROS `pelvis_link` metres: X forward, Y left, Z up, including
0.20 mm sticker-plus-adhesive thickness. Ball mode and mount-seat coordinates
are not substitutes for these sticker centres.

The native asset is rigidly aligned to the nominal CAD pelvis origin and axes.
With Motive Streaming **Up Axis = Z Up** and the bridge's existing
`topics.rigid_body_markers.modeldef_y_up_to_z_up: true`, the expected fixed
`P1 → pelvis_link` correction is translation `[0,0,0]` m and quaternion
`[0,0,0,1]` in xyzw order. Do not add another pose rotation or old pivot offset.
The supplied [identity-default JSON](../agibot/mocap_sticker_shell_v3/motive_asset/P1_to_pelvis_link_default.json)
is **unapproved reference data**, not a runtime receipt: never copy it here
or manually set `approved=true`. Offline asset checks have passed; Motive
import/re-export and installed live verification remain required.

## Generate and use a fresh receipt

Follow the [v3 setup and calibration procedure](../agibot/README.md#v3-sticker-p1-to-pelvis-calibration)
for asset import, rebuilding, physical attestation and live capture. Restart
NatNet after importing/changing P1 so its MODELDEF is refreshed. All 12 markers
must be defined and individually observed sufficiently during capture; they
need not be visible simultaneously. The solver estimates the actual correction
and checks residuals; it does not force an identity result.

For the attended Foxglove console, use the
[operator runbook](../docs/operations/foxglove_first_hardware_test.md).
`Calibration` calls `/hope/calibrate` while the Runner remains in fresh,
stationary `PD_STAND`; `Refresh x_hit` is a separate action, not calibration.
Complete the selected procedure before entering policy mode.

- An accepted fit atomically replaces `calibration/p1_to_pelvis.json` on the
  laptop. It records approval, the fixed P1-to-pelvis transform, marker
  correspondence, quality evidence and layout metadata. The service also
  records a stationary world-pelvis snapshot for audit; that snapshot must
  not be published as the static world pose of a moving robot.
- The runtime validates layout ID `A3_marker_shell_v3_stickers_12mm`, canonical
  table SHA-256, retained names and asset-frame revision
  `pelvis_link_aligned_native_y_up_20261004` through the shared
  [layout contract](../hope_ws/src/hope_bringup/scripts/p1_marker_layout.py).
  Pre-alignment 12-point, old 24-station, ten-marker and ball-mode receipts
  are rejected, even if they contain `approved=true`.
- A failed fit leaves the active file untouched; completed rejected analyses
  write `p1_to_pelvis.rejected.<UTC timestamp>.json` for local diagnosis.
  Preserving an old file does not make an obsolete receipt valid.
- Recalibrate after changes to the installed marker geometry or asset
  pivot/axes. The relay composes the accepted fixed correction with live
  `world → P1`; the expected local identity does not replace world/table calibration.

Do not commit runtime receipts, rejected fits, captures or machine-local
diagnostics. Distributable asset references and evidence belong in the
[v3 shell folder](../agibot/mocap_sticker_shell_v3/README.md), not here.
