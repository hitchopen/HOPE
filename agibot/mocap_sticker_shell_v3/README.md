# A3 dual-mode marker shell v3

**Printable mesh geometry checked. Physical fit, hardware retention and crash
testing remain required.** The files are prepared locally on `nightly_built`;
nothing has been committed or pushed.

## What changed

- All ten obsolete external marker bosses **and their screw bores** are removed,
  including f1 and b1-b5. Their locations are restored to smooth local shell skin.
- The 24 asymmetric stations have **flat 12 mm tops**, with no raised rims or
  recessed sticker pockets. Each retains its **3.4 mm central screw bore**.
- Structural shell fastener holes, seams and surrounding ribs are retained.
  Upper internal reliefs remain; only the old cavities near the restored skin
  are filled. Small, measured mesh cleanup was also needed.
- Marker positions and axes are unchanged. The existing `pelvis_link` tables
  remain the nominal CAD transforms for this revision.

## Printable files

| File | Use |
|---|---|
| [Half A](printable/a3_marker_shell_v3_half_a.stl) | Front half, one fused solid; 83,340 triangles |
| [Half B](printable/a3_marker_shell_v3_half_b.stl) | Rear half, one fused solid; 102,652 triangles |
| [Flush pin](printable/a3_marker_shell_v3_flush_pin.stl) | One central-hole pin; print up to 24 for sticker mode |
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

Neither mode has a validated impact-survival rating. Removing raised guards
improves local visibility but does not guarantee tracking at grazing angles.

## Verification and limits

Both exported halves pass independent triangle-level and FreeCAD checks:
closed, manifold, consistently oriented, one connected part, no duplicate or
degenerate triangles, and **zero detected self-intersections**. Their assembled
mesh intersection volume is zero. All 250 probes at the old holes hit closed
skin; checks on all 24 new stations confirm planar tops and open screw bores.

A 57,370-point comparison outside the altered old-mount regions and new-pad
footprints found maximum increases in distance from the original CAD of
**0.0145 mm front / 0.0165 mm rear**. The archived tessellation already had
larger absolute errors; see the verification report. These are sampled
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

## Source records and engineering archive

The repository [license](../../LICENSE) and original source records are
unchanged. Neutral product naming does not remove provenance or applicable
attribution requirements.

Testing scripts, failed conversions, rejected repairs and superseded models
remain recoverably in the local Git-ignored `.engineering_archive/`; they are
not printable release files. Only the checked parts, OpenSCAD entry point,
README, required notices and documents belong in the release.
