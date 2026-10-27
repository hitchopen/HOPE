# OptiTrack P1/P2 sticker assets

Use **[P1.motive](P1.motive)** and **[P2.motive](P2.motive)** with the same
12-sticker installations as the [ChingMu pair](../vrpn_asset/README.md).
The 17-station shell, STL files and CAD transform tables are unchanged.

| File | Body name | Streaming ID | Reflective stations, member IDs 1–12 in this order |
|---|---|---|---|
| P1.motive | UCB_P1 | 9 | S19, S20, S11, S12, S22, S21, S13, S14, S04, S05, S16, S18 |
| P2.motive | UCB_P2 | 10 | S04, S05, S06, S07, S08, S11, S12, S15, S17, S18, S20, S22 |

Both use **nominal CAD optical centres**, including 0.20 mm sticker/adhesive
thickness. Unlike the historical mixed measured/nominal asset, all points in
this pair come from the canonical sticker table, matching ChingMu exactly
after unit and axis conversion. This is not a new physical calibration.
The [manifest](asset_manifest.json) records per-point coordinates in both
bases, station/member mapping, body/constraint UUIDs, file hashes and provenance.

## Import and verify

1. Back up the Motive project. Import the two `.motive` files in an isolated
   project first; they are **native XML candidates, not yet import/re-export
   verified in Motive**. Do not import the manifest or historical all-17 files.
2. Confirm `UCB_P1` and `UCB_P2`, each with exactly 12 constraints. The pair has
   distinct UUIDs, also distinct from the source export, so do not expect it
   to replace an existing body automatically. Resolve existing name and
   streaming-ID 9/10 collisions before deployment. These IDs are not the
   ChingMu IDs or VRPN sensor indices.
3. Confirm the pivot is the nominal **pelvis_link origin**, not the marker
   centroid. Keep the same robot-local frame for both players; **do not mirror
   or rotate P2** to face the table. The world/table-side transform is separate.
4. Set **Streaming Up Axis = Z Up** and retain
   `topics.rigid_body_markers.modeldef_y_up_to_z_up: true` in the NatNet adapter.
   Native asset offsets use metres and `[ROS X, ROS Z, -ROS Y]`; the MODELDEF
   conversion `[x, -z, y]` recovers ROS **X forward, Y left, Z up**. Do not apply
   another 90-degree pose rotation or add sticker thickness again.
5. Re-export the bodies and compare offsets/pivot/axes with the manifest,
   allowing marker reordering. Each candidate has 66 valid visualization
   sticks and omits the undocumented learned `constraint_graph`; no tracking
   statistics were fabricated. If the importer rejects the candidate, author
   the body using supported Motive tooling and the manifest's **native Y-up
   coordinates**, then repeat the export check. An export may regenerate UUIDs
   or labels; validate the resulting stream, not just the filename.
6. Restart the adapter to refresh MODELDEF. Check measured displacements and
   rotations against physical pelvis datums and check table/world alignment
   separately. Test each robot individually, then both together under front,
   rear and side occlusion and after tracking loss. Native import, physical fit
   and live recognition remain deployment acceptance checks.

The intended fixed `UCB_P1/UCB_P2 → pelvis_link` transform is translation
`[0,0,0]` m and quaternion `[0,0,0,1]` (xyzw). The robot's world pose remains
dynamic. Do not use Reset Location/Reset Orientation to recenter the body.
See the vendor's [rigid-body guide](https://docs.optitrack.com/motive/rigid-body-tracking)
and [streaming settings](https://docs.optitrack.com/motive/data-streaming).

## HOPE runtime

- Default `localization_source:=motive_live` is unchanged. It accepts the
  selected live ModelDef, with no pinned marker count or file; correct imported
  pivot/axes must be verified by the operator.
- Optional `localization_source:=motive_asset` now pins **the selected side's
  12-marker reference**. Rebuild/install `hope_bringup` with both files and the
  updated contract/relay. It compares MODELDEF after axis conversion, independent
  of marker order, within 0.05 mm serialization tolerance. This is not a camera
  residual tolerance. A renamed wrong-side pattern, recentered body or old
  17-marker asset fails this optional check. No new default admission gate is added.
- The older **measured `stickers_v3` calibration workflow still requires all
  17 sites**. It cannot issue a receipt for either new 12-marker pattern. Do not
  reuse/hand-approve old receipts or select that workflow for these installations.
  Live/pinned nominal modes do not require a calibration JSON; they do not prove
  the physical pelvis alignment. Extending measured subset calibration is separate.

## Occlusion limit

Seven stations are shared: **S04, S05, S11, S12, S18, S20, S22**. All five
P2-only stations are rear sites. Shared-only observations can be identical;
names, streaming IDs and UUIDs do not make the optical patterns unique from
every vantage point. The same [CAD screening analysis](../documents/player_pattern_analysis.json)
applies to both vendors, but does not model either vendor's tracking solver.
No marker-count/boot/continuation threshold is imposed by these Motive files;
verify project settings and both bodies under actual occlusion before play.

## Reference records and reproduction

This import folder contains only **P1.motive, P2.motive, asset_manifest.json
and README.md**. Every current player definition has 12 markers. There is no
all-17 P1 alias or obsolete authoring header here.

Original user exports and correspondence evidence are preserved separately in
[documents/source_evidence/optitrack](../documents/source_evidence/optitrack/README.md).
Historical generated 17-point assets and the unapproved identity reference
exist only as maintained regression fixtures outside the shell release.
Obsolete all-17 authoring helpers and their stale validation summary were
removed (recoverable from Git history). The current manifest supplies complete
P1/P2 authoring coordinates.

From the repository root, regenerate both vendors' assets and the
[installation PDF](../output/pdf/A3_marker_shell_v3_instructions.pdf):

```bash
python agibot/mocap_sticker_shell_v3/tools/generate_player_patterns.py --pdf
```

This offline authoring tool requires NumPy; PDF generation also needs
Matplotlib and ReportLab. When intentionally changing a Motive file, update
the matching per-side SHA pin in `motive_asset_contract.py`, both-vendor
geometry tests and installed files together. The generator does not silently
change runtime pins. Keep original measurements and geometry evidence intact.
