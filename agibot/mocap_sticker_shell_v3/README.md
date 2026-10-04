# A3 dual-mode marker shell v3

**12-station v3 revision, 2026-10-04. Printable mesh geometry checked;
physical fit, hardware retention and crash testing remain required.**
The v3 model name and printable filenames are unchanged.

## What changed

- All ten obsolete external marker bosses **and their screw bores** are removed,
  including f1 and b1-b5. Their locations are restored to smooth local shell skin.
- Removed stations **S01, S02, S03, S06, S07, S08, S09, S10, S15, S17, S23,
  S24**, including their mounting holes; these locations now use the smooth
  base-shell skin. No holes were cut through the pelvis-fitting interior.
- Retained **S04, S05, S11, S12, S13, S14, S16, S18, S19, S20, S21, S22**.
  Original station IDs are preserved, not renumbered.
- The 12 retained asymmetric stations have **flat 12 mm tops**, with no raised rims or
  recessed sticker pockets. Each retains its **3.4 mm central blind screw bore**
  and side-access captive-nut pocket; these are not through-holes.
- Structural shell fastener holes, seams and surrounding ribs are retained.
  Upper internal reliefs remain; only the old cavities near the restored skin
  are filled. Small, measured mesh cleanup was also needed.
- Retained marker positions and axes are unchanged. The `pelvis_link` tables
  contain only these 12 stations, with their original numeric values.

## Printable files

| File | Use |
|---|---|
| [Half A](printable/a3_marker_shell_v3_half_a.stl) | Front half, one fused solid; 74,536 triangles |
| [Half B](printable/a3_marker_shell_v3_half_b.stl) | Rear half, one fused solid; 90,316 triangles |
| [Flush pin](printable/a3_marker_shell_v3_flush_pin.stl) | Unchanged central-hole pin; print up to 12 for sticker mode |
| [OpenSCAD model](a3_marker_shell_v3.scad) | Assembly preview and individual-part print placement |
| [Instructions](output/pdf/A3_marker_shell_v3_instructions.pdf) | Installation, printing checks, station drawings and translation table |
| [Geometry verification](documents/geometry_repair_status.md) | Methods, measured limits and remaining physical checks |
| [Machine-readable checks](documents/printability_validation.json) | Mesh hashes and validation results |

The STL masters use **millimetres** in the original shell CAD frame (CAD Y up).
Import at **100% scale**. Their footprints are approximately 176 x 194 mm;
allow additional room for brim and supports.

In OpenSCAD, use F5 for `part="assembly"`. Select `half_a`, `half_b` or
`flush_pin` for F6/export. With `bed_oriented=true`, each half is placed
open-side down at Z=0; supports may still be required. Do not export the
assembled pair as one print. Print placement does not alter the marker table.

These are fused STL manufacturing masters, not the earlier overlapping STEP
assembly or partial previews. A repaired analytic STEP is not supplied.

## Marker installation

For sticker mode, install the **3.45 mm diameter x 1.30 mm long** central pin
flush, then centre a **12 mm reflective sticker, 0.20 mm total thickness**,
across the whole top. The pin's nominal interference fit is provisional:
test it first and do not force it. The 20 mm tapered foot sits below the
12 mm optical support plane.

For ball mode, remove the sticker and pin. The design retains a provisional
**M3** captive-nut pocket and concentric bore. Confirm the actual marker-ball
thread, nut retention and stud reach; 12 mm ball diameter does not specify
the thread. The bore is 4.6 mm deep below the seat; candidate stud penetration
must not exceed 4.0 mm without a new hardware check. Do not drill through
the shell's inner surface.

**Current release boundary:** the through-hole/internal-nut proposal has not
been applied to these v3 files. A nut is inserted through the existing station's
side slot, not fitted against the pelvis-facing interior. Ball hardware remains
provisional; neither an inside bolt/nut installation nor arbitrary 12 mm ball
studs are qualified. Any future through-hole revision must also address the
old nut-pocket void, supported internal bearing surfaces, ribs and pelvis clearance.

Neither mode has a validated impact-survival rating. Removing raised guards
improves local visibility but does not guarantee tracking at grazing angles.

## Verification and limits

Both exported halves pass independent triangle-level and FreeCAD checks:
closed, manifold, consistently oriented, one connected part, no duplicate or
degenerate triangles, and **zero detected self-intersections**. Their assembled
mesh intersection volume is zero. All 12 retained stations pass planar-top,
12 mm outside-diameter and 3.4 mm bore checks. At the 12 removed stations,
588 ray probes agree with the archived smooth base surfaces within 0.001 mm.

A 57,370-point comparison outside the altered old-mount regions and new-pad
footprints found maximum increases in distance versus the previous v3 meshes
of **0.00000103 mm front / 0.00000171 mm rear** (serialization-level changes).
Earlier base repairs and original CAD tessellation errors remain documented;
this is not a new claim of exact CAD accuracy. These are sampled
geometric comparisons, not a physical fit certificate.

Most restored sites sample near 2.5 mm wall thickness. The f1 restoration has
an approximately **1.50 mm local edge gap** in a vertical-ray audit; configure
and inspect the sliced layers so that region is retained. This is not a
certified global minimum wall thickness. Check supports, thin ribs, nut
pockets, screw access, seams and full joint motion on a secured robot before
operation. No printer/material-specific slicing or physical fit test has run.

## Marker transforms to pelvis_link

- [Sticker optical centres](documents/marker_transforms_pelvis_link_stickers.csv)
- [Mount-seat transforms](documents/marker_transforms_pelvis_link_mount_seats.csv)
- [Auditable workbook](documents/A3_v3_marker_transforms_pelvis_link.xlsx)
- [CAD registration and frame conventions](documents/CAD_registration.md)

Each transform maps the marker-local frame **into** `pelvis_link`. Translations
are metres; quaternions are `qx,qy,qz,qw`; local +z points outward.
Round-marker roll is a CAD convention, not a measured optical orientation.

```text
p_sticker_CAD = p_seat + 0.20 * normal           # CAD millimetres
p_pelvis_link = [CAD_Z, CAD_X, CAD_Y - 145.800095037096] / 1000
```

The mount seat is **not** the ball's sphere centre. Measure the installed
ball/stud/spacer offset before producing a ball-centre table and matching
Motive calibration. The recovered 6 mm stand-off of the obsolete registration
markers must not be reused for the new ball hardware.

`nightly_built` calibration uses the **sticker optical-centre CSV directly**
for the 12 retained stations and installs the same table with `hope_bringup`.
After changing the shell/Motive asset, restart NatNet and generate a fresh live
P1-to-pelvis receipt; pre-alignment 12-point, old 24-station and ten-marker receipts
are rejected. Follow the
[v3 calibration procedure](../README.md#v3-sticker-p1-to-pelvis-calibration).
The [calibration README](../../calibration/README.md) explains the laptop-local
runtime receipt; do not use the supplied identity-default JSON as that receipt.
This software integration does not make the CAD table or authoring asset a
physically verified calibration.

## Supplied OptiTrack asset and station evidence

- [Original A3.motive](motive_asset/source/A3.motive) and
  [A3.csv](motive_asset/source/A3.csv) are byte-for-byte copies of the user's
  16-point export, retained for traceability. **Do not use the unfiltered
  16-point asset for the revised 12-station shell.**
- [A3_v3_12_stickers.motive](motive_asset/A3_v3_12_stickers.motive) and its
  [CSV](motive_asset/A3_v3_12_stickers.csv) now define the **pelvis-aligned `P1`**.
  Import the native `.motive` file; the CSV is its coordinate companion.
  Original Points **3, 9, 13** (removed S23, S03, S02) and unmatched **Point 15**
  are excluded. The measured constellation is rigidly reframed to the nominal
  CAD `pelvis_link` origin and axes; **measured pair distances are preserved**,
  rather than replacing the observations with ideal CAD points. Diameters,
  constraint UUIDs and streaming ID `9` are unchanged. The body is renamed
  `P1`; member IDs remain 1–12 in the original retained order. All 66 graph/stick
  pairs retain their source values and filtered references.
- [Station mapping](documents/optitrack_station_mapping.csv) records the
  original Point number, filtered member ID and physical Sxx identity.
  All 12 retained stations have geometric matches in both supplied files:
  rigid-fit RMS **1.395 mm**, maximum **1.853 mm**. This confirms presence in
  the asset, **not continuous visibility or an approved runtime calibration**.
- The filtered native file passes XML/reference/coordinate checks but has
  **not been imported and re-exported in Motive**. Import into an isolated
  project first; the candidate preserves the source body UUID and can replace
  that body if imported into the same project. Confirm body name `P1`, 12
  markers and a unique streaming ID, save a fresh native export, restart
  NatNet and perform the live verification described below.

### Pelvis origin and ROS axes: no extra default correction

The aligned asset targets **`pelvis_link` at its origin, X forward / Y left /
Z up in the ROS stream**. The default `P1 → pelvis_link` is therefore:

```text
translation (m):    [0, 0, 0]
quaternion (xyzw):  [0, 0, 0, 1]
```

See [identity default](motive_asset/P1_to_pelvis_link_default.json). It is an
explicit offline expectation, **not an approved runtime receipt**.

Motive's native asset storage and the ROS streaming convention are different:
the file stores `[pelvis_x, pelvis_z, -pelvis_y]` (X forward, Y up, Z right).
Set **Motive Streaming → Up Axis = Z Up**, and retain the bridge's
`topics.rigid_body_markers.modeldef_y_up_to_z_up: true`. Its existing
`(x,y,z) → (x,-z,y)` MODELDEF conversion then yields the aligned ROS frame.
**Do not add another 90-degree pose rotation or reapply the old pivot offset.**
Simply placing raw ROS xyz values in this native file would apply the basis
change twice in the configured pipeline. The frame convention is covered by
the [Motive streaming settings](https://docs.optitrack.com/motive/data-streaming#up-axis)
and the repository's recorded MODELDEF field verification; this particular
Motive 3.5 asset still needs an import and stream check.

The production CAD solver recovers identity from the serialized, converted
asset to numerical precision (translation below 1e-10 m; quaternion-vector
components below 1e-9). That is an offline consistency test, not physical
accuracy: marker-to-CAD mismatch remains 1.395 mm RMS / 1.853 mm maximum.
The robot/world pose still rotates as the robot moves; only the **fixed local
P1-to-pelvis correction** is intended to be identity. World/table calibration
is unaffected.

After importing, do not use Motive **Reset Location / Reset Orientation**:
those reset the pivot to the marker centroid or re-align it to the world axes,
undoing this registration. See [Motive pivot controls](https://docs.optitrack.com/motive/rigid-body-tracking#adjusting-a-rigid-body-pivot-point).
Run the existing live calibration as verification; expect an approximately
identity result. A significant correction is a setup discrepancy to investigate,
not a reason to force the saved values to identity. Receipts from the earlier
12-point arbitrary pivot are rejected by the new asset-frame revision, even
though the CAD table hash is unchanged. Never manually mark the default approved.

The untouched `source/A3.motive` retains its old arbitrary frame. The separate
`P1_stickers_definition.json` and header remain **ideal CAD** authoring references
in CAD station order, not copies of the measured aligned asset. Their proposed
member IDs differ from that asset's order. Resolve the actual MODELDEF by verified
names or geometric matching.
See [asset correspondence report](documents/optitrack_asset_correspondence.json).

## Source records and engineering archive

The repository [license](../../LICENSE) and original source records are
unchanged. Neutral product naming does not remove provenance or applicable
attribution requirements.

Testing scripts, failed conversions, rejected repairs and superseded models
remain recoverably in the local Git-ignored `.engineering_archive/`; they are
not distributed release files. The release folder contains the checked parts,
OpenSCAD entry point, instructions/TF tables, asset and provenance files, and
geometry-validation reports. The unchanged original A3 exports are source
evidence, not disposable test output. Maintained regression tests live under
`hope_ws/src/hope_bringup/test/` and are not installed as robot runtime programs.
