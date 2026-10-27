# Shell v3 - P1 / P2 ChingMu assets

One unchanged 17-station shell supports **two distinct 12-sticker installations**.
Import [P1.json](P1.json) and [P2.json](P2.json) into ChingMu CMTracker/MCServer.
The internal body names are **UCB_P1 / UCB_P2** for existing HOPE routing; the
numeric IDs are 1 / 2. These files follow the supplied CMTracker native JSON
schema, not a universal VRPN import format. Import and physical accuracy remain
unverified; the checks described below are offline CAD checks.

## Install exactly the selected twelve

| Body | Reflective stations, in asset array order | Five nonreflective stations |
|---|---|---|
| P1 / UCB_P1 | S19, S20, S11, S12, S22, S21, S13, S14, S04, S05, S16, S18 | S06, S07, S08, S15, S17 |
| P2 / UCB_P2 | S04, S05, S06, S07, S08, S11, S12, S15, S17, S18, S20, S22 | S13, S14, S16, S19, S21 |

Use **12 mm round reflective stickers, 0.20 mm including adhesive**, centred on
the flat station faces with trial-fitted flush pins. Leave the other five sites
bare or nonreflectively capped; verify under IR that they do not appear as markers.
The [illustrated PDF](../output/pdf/A3_marker_shell_v3_instructions.pdf) gives
front/rear views, a 360-degree map, sticker coordinates and each asset's indices.
The print, screw bores, 17-station tables and workbook are unchanged. These
sticker assets are not suitable for ball-marker centres without measuring the
installed ball/stud/spacer offsets.

## Why this P2 pattern, and what it cannot guarantee

P1 is exactly the user-specified set. Any two 12-of-17 sets share at least
**seven stations**. P2 includes all five P1-absent rear sites, reaching that
minimum overlap. The shared stations are **S04, S05, S11, S12, S18, S20, S22**.

The calculation considers all 6,188 possible 12-of-17 choices analytically for
overlap, then evaluates all **792** candidates with minimum overlap. The
lexicographic objective is:

1. Minimize additional nonshared four-point distance-signature candidates at 2 mm.
2. Minimize additional nonshared three-point candidates at the same threshold.
3. Maximize the minimum P2 normal-facing count over the angular grid.
4. Maximize grid directions containing a P1-only facing point.
5. Maximize directions with at least six P2-facing points, then total P2-facing
   count and directions containing two P1-only points. Station order breaks ties.

A distance signature is the sorted list of all inter-point distances in a
subset. Its mismatch is the largest component difference, minimized over P1
subsets. This screen is conservative: even matching signatures need not be a
valid proper-rotation fit. **2 mm is an engineering assumption, not a measured
CMTracker matching tolerance.** The selected layout has zero nonshared
four-point candidates and six nonshared three-point candidates at 2 mm.
At 3 mm the four-/three-point counts are 1/15; at 5 mm they are 24/70.

The angular grid has 288 directions: azimuth 0-355 degrees in steps of 5,
elevation 0/15/30/45 degrees, and normal incidence at most 60 degrees. P2's
minimum facing count is two and its mean is 3.934. A P1-only point faces 286
directions; a P2-only point faces 178. These are **one-direction upper bounds**,
not reconstructed marker counts. Shell/torso/arm occlusion and camera
triangulation are not simulated. The report also records 45/70-degree incidence
sensitivity. There is no camera-layout-independent universal best pattern.

**Shared-only observations remain indistinguishable.** The shared seven include
35 identical triples, 35 identical quadruples and seven identical six-point
subsets. All five P2-only points are at the rear; front-only P2 observations
are a subset of P1's front geometry. The retained `minMarker: 6` does not remove
this problem. Eight genuinely observed matched points would exclude a
shared-only subset in nominal geometry, but this is not a recognition guarantee
or a new runtime threshold. Neither color nor a JSON ID makes passive stickers
optically distinguishable. Verify camera coverage and two-body reacquisition.

See [machine-readable calculation](../documents/player_pattern_analysis.json)
and [reproducible authoring tool](../tools/generate_player_patterns.py). The
selection is best under that declared criterion, not certified for every view.

## Origin, axes and units

Each file contains **36 markerPose values = 12 XYZ triplets**, selected from the
unchanged [optical-centre CSV](../documents/marker_transforms_pelvis_link_stickers.csv):

```text
markerPose[3*i : 3*i+3] = 1000 * selected_station.[x_m, y_m, z_m]
Rbc = [1,0,0, 0,1,0, 0,0,1]
intended fixed body -> pelvis_link: xyz [0,0,0] m; xyzw [0,0,0,1]
```

The origin is nominal `pelvis_link`, **X forward, Y left, Z up**. Do not subtract
the subset centroid, add sticker thickness again, apply Motive Y-up conversion,
reuse the old ten-ball pivot offset or rotate/mirror P2's local frame. The robot's
physical heading is tracked; HOPE handles the table-side world conversion.
The matrix is inherited from the supplied format; confirm its interpretation
with the installed vendor version. Nominal CAD registration is not a measured
physical calibration, and the changing world-to-body pose is not identity.

## Import and commission

1. Back up the CMTracker project. Import both files into an isolated project,
   confirming distinct names/IDs, exactly 12 points and the intended subsets.
   Resolve project ID collisions; IDs 1/2 do not declare VRPN sensor indices.
   Do not import the manifest or analysis JSON.
2. Preserve the pelvis pivot and axes. Export fresh native JSON and compare
   geometry and frame definitions. A point reorder alone can be equivalent.
   Check installed centring, pivot and local axes against physical datums with
   the robot secured. Record measured errors across poses.
3. Test each body separately and both together, including front/rear occlusion,
   arms and torso, shared-only subsets, complete loss and reacquisition. Record
   swaps, erroneous matches and dropouts. Ensure actual rear coverage of the
   P2-only points; do not infer identity from absent markers.
4. Verify server world/table axes and streaming units independently. JSON
   millimetres do not prove VRPN's units: use measured displacement and set
   `position_scale:=0.001` for mm or `1.0` for m. The HOPE table frame is Z-up,
   tabletop Z=0, X=[0,2.74], Y=[-1.525,0] m.
5. Deploy the updated V4 catalog, its two runtime assets and profile reader
   together. Select **ChingMu/VRPN**, **V4**, and the correct table side while
   stopped. V4 remains the software alias for the v3 shell; V2 is unchanged.
   The former identical 17-point V4 profiles do not describe these installations.
   Importing in CMTracker and selecting in Foxglove are separate operations.

For a pose-only check in a sourced ROS workspace:

```bash
ros2 launch hope_bringup avatar_pro_hope_bridge.launch.py \
  server:=<CMTracker_IP> follow_lifecycle_config:=false \
  pants_profile:=v4 table_side:=P1 position_scale:=0.001 \
  start_flight_packetizer:=false
```

For P2 use `table_side:=P2` with the imported `UCB_P2`. Choose the measured stream
scale. This leaves `pelvis_asset_confirmed` and `world_frame_confirmed` false;
enable them only after their checks pass. See the
[VRPN operator procedure](../../../docs/operations/vrpn_pants_profiles.md).
VRPN supplies poses, not ModelDef or a remotely measured geometry hash. A local
profile hash cannot certify the server project or the physical installation.

The old single ChingMu import has been replaced by the two files. Matching
[OptiTrack P1/P2 files](../motive_asset/README.md) use the same subsets and CAD
optical centres with native Y-up/metre encoding. Both vendor import folders
contain only the current P1/P2 pair, manifest and README. Historical all-17
assets are regression fixtures outside the shell release, not player imports.

## Reproduce and maintain

[asset_manifest.json](asset_manifest.json) records per-player station order,
file hashes, source table, frame contract and unchanged printable hashes. The
two release JSONs are byte-identical to their respective files under
`hope_ws/src/hope_bringup/config/pants_profiles/`. Their catalog now keeps V4
marker names and positions per side. Update these together when a pattern changes.

From the repository root, with NumPy, Matplotlib and ReportLab installed:

```bash
python3 agibot/mocap_sticker_shell_v3/tools/generate_player_patterns.py --pdf
python3 -m pytest -q \
  hope_ws/src/hope_bringup/test/test_v3_vrpn_asset.py \
  hope_ws/src/hope_bringup/test/test_vrpn_pants_contract.py \
  hope_ws/src/hope_bringup/test/test_vrpn_pants_relay.py
```

`--analyze-only` recomputes the search without writing files. The authoring tool
is not deployed on the robot. Native import/re-export and live occlusion tests
remain required; offline numerical agreement is not a tracking certification.
