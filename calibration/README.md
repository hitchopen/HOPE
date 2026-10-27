# Optional measured calibration receipts

Default `motive_live` localization accepts the selected body's current ModelDef
and assumes its imported pelvis pivot/axes. It requires no calibration JSON,
fixed file hash or fixed marker layout. Console 1.8.13 reports live asset status
and provides Refresh x_hit separately. The workflow below applies only when
explicitly selecting `localization_source:=calibration start_calibration:=true`.

`ucb_robot_to_pelvis.json` is generated on the operator laptop by the attended
**17-station v3 sticker calibration**. It contains installation-specific
geometry, timestamps, sample counts and audit evidence, so it is intentionally
Git-ignored. The robot receives `/a3/base_pose_flat`, not this JSON.

## Current P1/P2 patterns: measured-subset calibration not supported

The current [OptiTrack P1/P2 pair](../agibot/mocap_sticker_shell_v3/motive_asset/README.md)
has **12 stickers per player** on the unchanged 17-station shell, matching the
ChingMu pair. Default `motive_live` remains available; optional `motive_asset`
now checks the correct side's nominal 12-point reference. Neither proves
physical pelvis alignment. Import/re-export and installed checks are required.
The measured `stickers_v3` procedure below still requires all 17 stations and
**cannot generate a receipt for either new pattern**. Do not reuse or edit a
legacy receipt to bypass this limitation.

## Legacy all-17 measured mode

The unchanged [sticker optical-centre table](../agibot/mocap_sticker_shell_v3/documents/marker_transforms_pelvis_link_stickers.csv)
has 17 station rows in ROS `pelvis_link` metres, including 0.20 mm film/adhesive.
The `stickers_v3` measured solver still requires all those stations. It is not
a profile for either current 12-marker player installation. Legacy `v2` and
`v3` ball profiles remain separate.

The following procedure applies only to a separately validated all-17 physical
installation and matching native body. Historical exports/identity defaults
are source evidence and regression fixtures, **not deployment assets or approved
receipts**. The release import folders contain only the current P1/P2 pairs.
Do not add points, alter marker lists or approve a saved JSON merely to satisfy
the legacy solver.

## Generate and use a fresh receipt in calibration mode

Follow the [v3 setup and calibration procedure](../agibot/README.md#v3-sticker-p1-to-pelvis-calibration)
for asset import, rebuilding, physical attestation and live capture. Restart
NatNet after importing/changing the selected UCB asset so its MODELDEF is refreshed. All 17 markers
must be defined and individually observed sufficiently during capture; they
need not be visible simultaneously. The solver estimates the actual correction
and checks residuals; it does not force an identity result.

Use the manual/service workflow in the [calibration procedure](../agibot/README.md#v3-sticker-p1-to-pelvis-calibration) for measured mode while Runner remains in stationary `PD_STAND`. The current console shows live asset status; Refresh x_hit remains a separate action.

- An accepted fit atomically replaces `calibration/ucb_robot_to_pelvis.json` on the
  laptop. It records approval, the fixed P1-to-pelvis transform, marker
  correspondence, quality evidence and layout metadata. The service also
  records a stationary world-pelvis snapshot for audit; that snapshot must
  not be published as the static world pose of a moving robot.
- The runtime validates layout ID `A3_marker_shell_v3_stickers_12mm`, canonical
  table SHA-256, retained names and asset-frame revision
  `pelvis_link_aligned_native_y_up_rear17_20261011` through the shared
  [layout contract](../hope_ws/src/hope_bringup/scripts/p1_marker_layout.py).
  All earlier 12-point (including pelvis-aligned), old 24-station, ten-marker and ball-mode receipts
  are rejected, even if they contain `approved=true`.
- A failed fit leaves the active file untouched; completed rejected analyses
  write `ucb_robot_to_pelvis.rejected.<UTC timestamp>.json` for local diagnosis.
  Preserving an old file does not make an obsolete receipt valid.
- Recalibrate after changes to the installed marker geometry or asset
  pivot/axes. The relay composes the accepted fixed correction with live
  `world → P1`; the expected local identity does not replace world/table calibration.

Do not commit runtime receipts, rejected fits, captures or machine-local
diagnostics. Distributable asset references and evidence belong in the
[v3 shell folder](../agibot/mocap_sticker_shell_v3/README.md), not here.
