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

## Current shell and OptiTrack asset

Test-import [A3_v3_17_stickers.motive](../agibot/mocap_sticker_shell_v3/motive_asset/A3_v3_17_stickers.motive),
internally named **UCB_P1**, for stations **S04-S08 and S11-S22**. The seven front
markers are unchanged; rear rows S04-S08 and S14-S18 now have five each.
Do not renumber the physical station IDs.
The [sticker optical-centre table](../agibot/mocap_sticker_shell_v3/documents/marker_transforms_pelvis_link_stickers.csv)
is already in ROS `pelvis_link` metres: X forward, Y left, Z up, including
0.20 mm sticker-plus-adhesive thickness. Ball mode and mount-seat coordinates
are not substitutes for these sticker centres.

The supplied body already uses `UCB_P1`; use `UCB_P2` for table side P2.
For optional measured calibration, select `stickers_v3`.
The legacy `v2` and `v3` ball profiles remain separate supported layouts;
the sticker receipt restrictions below apply to `stickers_v3`.

The native asset is rigidly aligned to the nominal CAD pelvis origin and axes.
With Motive Streaming **Up Axis = Z Up** and the bridge's existing
`topics.rigid_body_markers.modeldef_y_up_to_z_up: true`, the expected fixed
`UCB_P1 → pelvis_link` correction is translation `[0,0,0]` m and quaternion
`[0,0,0,1]` in xyzw order. Do not add another pose rotation or old pivot offset.
The supplied [identity-default JSON](../agibot/mocap_sticker_shell_v3/motive_asset/P1_to_pelvis_link_default.json)
is **unapproved reference data**, not a runtime receipt: never copy it here
or manually set `approved=true`. Offline asset checks have passed; Motive
import/re-export and installed live verification remain required.
The original 12 measured positions are retained. Restored S06/S07/S08/S15/S17
are CAD-nominal additions with no observations in the source export. The learned
graph cache is omitted; native acceptance is unverified. Follow the
[asset validation/fallback procedure](../agibot/mocap_sticker_shell_v3/README.md#supplied-optitrack-asset-and-station-evidence)
before using it. Do not interpret nominal coordinates as physical samples.

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
