# A3 dual-mode marker shell v3

**2026-10-27: one unchanged 17-station v3 shell, two distinct 12-sticker
installations for ChingMu and OptiTrack P1/P2. Seven front + ten rear mounting stations remain.**
Printable mesh geometry is checked; physical fit, retention, tracking and crash
testing remain required.
The v3 model name and printable filenames are unchanged.

Imported from [HOPE commit 10b1bd710c074cf9851a3ca2f63494dc23a39e06](https://github.com/hitchopen/HOPE/commit/10b1bd710c074cf9851a3ca2f63494dc23a39e06).
Meshes, coordinate tables and workbook retain that commit's bytes. The PDF now
documents the two player installations and both vendors' import steps.
The new all-nominal asset pair has distinct identities; original measured
exports remain unchanged as historical evidence, not current imports.

The default localization mode remains **`motive_live`**: it uses the selected
body's current NatNet ModelDef without binding to a file SHA-256, marker count
or reference layout. A fresh calibration JSON is not required by that mode.
The marker monitor derives its expected count from the current definition.

For **ChingMu / CMTracker over VRPN**, use [P1.json](vrpn_asset/P1.json) or
[P2.json](vrpn_asset/P2.json) with the corresponding installation below.
Both preserve nominal `pelvis_link` and ROS axes; neither uses Motive's Y-up
mapping. Their internal names are `UCB_P1` and `UCB_P2` for HOPE routing.
The updated **V4** runtime profile carries the two different 12-marker definitions;
the CAD folder remains v3. VRPN has no live ModelDef. Matching
[OptiTrack P1/P2 assets](motive_asset/README.md) now use the same selected points
with Motive's native unit/axis encoding.

## What changed

- All ten obsolete external marker bosses **and their screw bores** are removed,
  including f1 and b1-b5. Their locations are restored to smooth local shell skin.
- Stations **S01, S02, S03, S09, S10, S23, S24** remain removed,
  including their mounting holes; these locations retain the smooth
  base-shell skin. No holes were cut through the pelvis-fitting interior.
- The **front seven are unchanged: S11, S12, S13, S19, S20, S21, S22**.
  Half A is byte-for-byte identical to the preceding release.
- Restored **S06, S07, S08, S15, S17**, completing rear upper row
  **S04-S08** and rear middle row **S14-S18** (five each). These are the
  original asymmetric station locations, not newly regularized rings.
  Original station IDs are preserved, not renumbered.
- All 17 asymmetric stations have **flat 12 mm tops**, with no raised rims or
  recessed sticker pockets. Each retains its **3.4 mm central blind screw bore**
  and side-access captive-nut pocket; these are not through-holes.
- Structural shell fastener holes, seams and surrounding ribs are retained.
  Upper internal reliefs remain; only the old cavities near the restored skin
  are filled. Small, measured mesh cleanup was also needed.
- The previous 12 marker positions and axes are unchanged. The `pelvis_link`
  tables add the five restored stations from the archived CAD definitions;
  neither the pelvis datum nor the seven front marker coordinates moved.

## Printable files

| File | Use |
|---|---|
| [Half A](printable/a3_marker_shell_v3_half_a.stl) | Front half, one fused solid; 74,536 triangles |
| [Half B](printable/a3_marker_shell_v3_half_b.stl) | Rear half, one fused solid; 99,132 triangles |
| [Flush pin](printable/a3_marker_shell_v3_flush_pin.stl) | Unchanged central-hole pin; print up to 17 for sticker mode |
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

### P1 / P2: one print, two optical patterns

| Installation | Apply 12 reflective stickers | Leave these five nonreflective |
|---|---|---|
| P1, user-specified order | S19, S20, S11, S12, S22, S21, S13, S14, S04, S05, S16, S18 | S06, S07, S08, S15, S17 |
| P2, calculated selection | S04, S05, S06, S07, S08, S11, S12, S15, S17, S18, S20, S22 | S13, S14, S16, S19, S21 |

P1 has seven front and five rear stickers; P2 has four front and eight rear.
No mounting station, bore, STL, OpenSCAD geometry or CAD transform changed.
Use the [updated illustrated instructions](output/pdf/A3_marker_shell_v3_instructions.pdf)
for front/rear views, a 360-degree installation map and per-asset point indices.
Keep unused sites nonreflective under the actual cameras' IR illumination.

The shared stations are **S04, S05, S11, S12, S18, S20, S22**. Seven is the
minimum possible overlap: 12 + 12 - 17 = 7. Thus **no such pair can guarantee
distinct identity from every partial view**. In particular all five P2-only
sites are rear sites; a front-only P2 observation is a subset of P1. Six or
seven visible shared points can still be ambiguous despite `minMarker: 6`.

P2 is the optimum of the documented CAD screening criterion over all 792
maximum-separation candidates: minimize nonshared four-point, then three-point
distance-signature similarities at 2 mm, followed by angular coverage criteria.
The selected pattern has zero additional four-point candidates at that tolerance,
but the shared subsets remain exactly identical. This is not a measured vendor
recognition threshold or an all-view guarantee. See the
[analysis and assumptions](documents/player_pattern_analysis.json) and
[asset guide](vrpn_asset/README.md). Verify both bodies together under actual
occlusion and reacquisition before relying on player identity.

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

Both serialized halves pass current Trimesh/MeshLab checks: closed, manifold,
consistently oriented, one connected part, no duplicate or degenerate triangles,
and **zero detected self-intersections**. Their assembled mesh intersection
volume is zero. All ten rear stations pass 49 probes each for flat 12 mm tops,
3.4 mm bores and 4.6 mm blind depth, within 0.001 mm. The previously checked
front geometry is unchanged. FreeCAD was not rerun on the new rear mesh.

A comparison of **84,425 rear surface points outside the five restored
footprints** found maximum distance to the new mesh of **0.00000296 mm**;
none exceeded 0.001 mm. This includes the unchanged interior and structural
features within the sampled surface, not a certified global distance bound.
Earlier base repairs and original CAD tessellation errors remain documented.
These are geometric checks, not a physical fit certificate.

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

The **sticker optical-centre CSV** contains all 17 stations and is installed
with `hope_bringup`. Its layout metadata supports offline comparisons and
optional measured calibration. Old 12-point, 24-station and ten-marker
receipts do not match this table. The supplied identity-default JSON remains
a reference, not an approved measured receipt. Default `motive_live` operation
does not require this table or receipt; see the
[localization console notes](../../foxglove/extensions/hope-a3-console/README.md).

## Supplied OptiTrack asset and station evidence

Use [P1.motive](motive_asset/P1.motive) and [P2.motive](motive_asset/P2.motive):
12 markers each, internally named **UCB_P1 / UCB_P2**, streaming IDs **9 / 10**,
with distinct body and marker UUIDs. They use the same station selections and
nominal CAD optical centres as the ChingMu pair. See the
[import guide](motive_asset/README.md) and [coordinate/hash manifest](motive_asset/asset_manifest.json).

These native XML candidates pass offline checks; **Motive import/re-export and
live tracking are not verified**. Resolve existing name/streaming-ID collisions
in a backed-up project. The undocumented learned graph is omitted rather than
invented. Do not use the historical all-17 import for either 12-sticker installation.

### Pelvis origin and ROS axes: no extra default correction

Both new bodies target nominal **pelvis_link**, X forward / Y left / Z up.
The intended fixed correction is translation **[0,0,0] m**, quaternion
**[0,0,0,1] xyzw**, not a measured physical-calibration result.

Native offsets store `[pelvis_x, pelvis_z, -pelvis_y]` in metres. Set Motive
Streaming **Up Axis = Z Up** and retain
`topics.rigid_body_markers.modeldef_y_up_to_z_up: true`. Do not convert twice,
subtract either subset's centroid, reset pivot/orientation or rotate P2 locally.
The moving world-to-body pose and world/table calibration remain separate.

Default `motive_live` accepts the selected live definition without a file pin.
Optional `motive_asset` now pins the selected player's **12-marker** geometry.
Rebuild/install the contract, relay and both assets together. The older measured
`stickers_v3` calibration tool requires all 17 sites and does not support this
pair; do not reuse its receipts or manually approve identity defaults.

### Historical measurement evidence (not current imports)

Original user exports and correspondence evidence are preserved under
[documents/source_evidence/optitrack/](documents/source_evidence/optitrack/README.md),
outside both import folders. The old generated 17-point asset/CSV and unapproved
identity reference are maintained regression fixtures, not deployable imports.
Obsolete all-17 authoring helpers and their stale validation summary were removed;
Git history retains them.

The original measured subset fit was **1.395 mm RMS / 1.853 mm maximum**;
five restored rear sites had no original observations. This does not validate
the current all-nominal P1/P2 installations or continuous visibility. Use the
current [manifest](motive_asset/asset_manifest.json) for player coordinates.
The operator's 2026-10-13 report concerned the historical asset, not this pair.

## Supplied ChingMu / VRPN asset

- [P1.json](vrpn_asset/P1.json), 12 stickers, body `UCB_P1`, numeric ID 1.
- [P2.json](vrpn_asset/P2.json), 12 stickers, body `UCB_P2`, numeric ID 2.
- [Import, origin/axis checks and runtime selection](vrpn_asset/README.md).

These replace the former single 17-point ChingMu import. Each release file is
byte-identical to its updated V4 runtime counterpart; deploy the matching
catalog and profile reader together. They follow the supplied CMTracker schema,
retaining `minMarker: 6` and other tracking settings. Each contains 12 nominal
sticker centres in `pelvis_link` millimetres, **X forward / Y left / Z up**,
with identity `Rbc`. No centroid subtraction, old offset or P2-local rotation
is applied. Both use selected rows of the same unchanged CAD table, not the
historical mixed measured/nominal constellation. The new Motive P1/P2 pair uses
these same CAD points after basis/unit conversion.

Import this definition into CMTracker, not into the ROS VRPN client. Confirm
the importer preserves the local pivot/axes and separately verify the server's
world frame and stream units before enabling localization. Native import/
re-export and physical calibration are not established by these offline files.
Use `pants_profile:=v4` for these installations; the legacy ten-ball `v3`
calibration layout is different. V2 remains unchanged.
Do not use the 17-marker Motive definition with a 12-sticker installation.

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
