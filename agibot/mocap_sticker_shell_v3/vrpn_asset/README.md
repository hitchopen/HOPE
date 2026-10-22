# Shell v3 — ChingMu / CMTracker rigid-body asset

This JSON file describes the **17-sticker shell**, with its local rigid-body
origin at nominal A3 **`pelvis_link`** and right-handed ROS axes:
**X forward, Y left, Z up**.

Import it into **ChingMu CMTracker/MCServer**, which then streams the tracked
body pose over VRPN. It is a CMTracker rigid-body definition following the
format of the supplied `UCB_P1(1).json`, **not a universal VRPN import format**
and not a ROS calibration receipt.

**Status:** coordinates and file consistency are checked offline. Import,
re-export, installed-shell accuracy and live tracking still require venue
verification. No software confirmation flag certifies those checks by itself.

## Import file

Use [A3_v3_17_stickers.json](A3_v3_17_stickers.json), matching the filename
stem of the supplied Motive asset, `A3_v3_17_stickers.motive`. This is the
**only import asset supplied here**; no separate P1/P2 variations are provided.
Its internal body name remains **`UCB_P1`**, also matching the Motive asset;
its numeric asset ID is **1**.

The [asset manifest](asset_manifest.json) records provenance, the file hash, station
order and the coordinate contract. **Do not import the manifest as a body.**
Configure the required body name (`UCB_P1` or `UCB_P2`) and a unique numeric ID
in CMTracker for the site, preserving the geometry, pelvis pivot and axes.
Check for existing project collisions before import. The asset ID is **not**
a declared VRPN sensor index.

### Naming: CAD v3 = runtime V4

The hardware folder is called `mocap_sticker_shell_v3`, but the existing
Foxglove/VRPN **MARKER PANTS** selection calls this same 17-sticker layout
**V4** (`pants_profile:=v4`). It is not the legacy ten-ball `v3` calibration
layout. The single import file is byte-identical to the existing
[V4 P1 runtime definition](../../../hope_ws/src/hope_bringup/config/pants_profiles/a3_v4_P1_ground_truth.json);
its geometry also matches the existing P2 runtime definition.
No runtime profile, hash, geometry or safety gate is changed by publishing it
beside the CAD release.

## Origin, axes and units

The source is the canonical
[sticker optical-centre table](../documents/marker_transforms_pelvis_link_stickers.csv),
already registered to `pelvis_link`. For each station:

```text
markerPose[3*i : 3*i+3] = 1000 * [x_m, y_m, z_m]
                         # pelvis_link coordinates, in millimetres

Rbc = [1,0,0, 0,1,0, 0,0,1]   # identity, retained from the supplied format

intended local body → pelvis_link:
translation (m)   = [0, 0, 0]
quaternion (xyzw) = [0, 0, 0, 1]
```

The JSON has **51 numbers = 17 XYZ triplets**, in this station order:

```text
S04 S05 S06 S07 S08 S11 S12 S13 S14 S15 S16 S17 S18 S19 S20 S21 S22
```

This is an authoring order, not a promise about live marker labels or VRPN
sensor ordering. The point cloud centroid is approximately
`[-7.841290, +0.454181, -92.255730] mm`. **Do not subtract this centroid**:
zero is the pelvis datum, not the centre of the markers.

- Do not apply Motive's native Y-up coordinate mapping to these XYZ values.
- Do not apply the old ten-ball centroid/pivot offset or add sticker thickness
  again. The table includes the nominal **0.20 mm** sticker-plus-adhesive layer.
- Do not copy the Motive native asset's point array: it uses a different basis
  and combines 12 measured points with five nominal additions. This asset
  intentionally uses **all 17 nominal CAD sticker centres**.
- The same **local** coordinates and `Rbc` apply on either side; do not mirror
  or rotate the asset by 180 degrees for P2. Physical robot heading is tracked.
  HOPE's P2 table-side transformation belongs to the runtime world conversion.
- This is sticker geometry, not a ball-centre or mount-seat definition.
  Ball/stud/spacer installations need their own measured centre offsets.

The identity matrix preserves the supplied CMTracker format; it is not a
substitute for checking how the installed vendor version interprets that file.
The remaining vendor fields, including `minMarker: 6`, filtering and flags,
follow the existing V4 runtime definitions. The full model still contains 17
points; `minMarker` is not its point count. Compared with the uploaded
ten-marker example, geometry is replaced and the existing V4 P1 runtime ID
and colour are retained. No undocumented vendor options are invented.

## Import and verify before use

1. Install and inspect the matching shell and **all 17 stickers**. Back up the
   CMTracker project; import `A3_v3_17_stickers.json` into an isolated test project
   first. Do not import the old ten-marker example as this shell.
2. Confirm the body name, unique ID and 17-point layout. Preserve the authored
   pivot and axes: do not recenter the model on its markers or reset its
   orientation to the current world heading. Export a fresh native JSON and
   check that the point geometry and pivot/axis definition remain equivalent.
   A reordered point list alone is not a geometry change.
3. With the robot secured and motion control disabled, check the tracked
   origin against the physical pelvis datum. Check that local +X points
   forward, +Y left and +Z up across several observed poses. The dynamic
   `world → UCB_P1/UCB_P2` pose must follow the robot; it is **not** identity.
   The intended fixed **body → pelvis_link** correction is identity.
4. Independently verify the server's **world** frame and streaming units.
   A pelvis-aligned body does not calibrate the table origin, world yaw or
   table height. JSON millimetres do not prove that VRPN streams millimetres:
   confirm with a measured displacement, then use `position_scale:=0.001`
   for a millimetre stream or `1.0` for metres. The canonical HOPE table frame
   is right-handed/Z-up, table surface z=0, x=[0, 2.74], y=[-1.525, 0] metres.
5. While the lifecycle is stopped, select **青瞳 / CMTracker · VRPN**, the
   correct table side and **V4** in Foxglove, then confirm configuration.
   Profile selection does not import a file into CMTracker or verify its
   currently loaded geometry.

For an initial standalone pose-only check in a sourced ROS workspace:

```bash
ros2 launch hope_bringup avatar_pro_hope_bridge.launch.py \
  server:=<CMTracker_IP> follow_lifecycle_config:=false \
  pants_profile:=v4 table_side:=P1 position_scale:=0.001 \
  start_flight_packetizer:=false
```

For P2, configure the body name as `UCB_P2` in CMTracker and use
`table_side:=P2`, without changing the local geometry. Choose the actual
stream scale where appropriate. This command leaves
`pelvis_asset_confirmed` and `world_frame_confirmed` false, so the guarded
base-pose output is not admitted as valid. Only after the corresponding checks
pass should those confirmations be enabled using the
[VRPN setup procedure](../../../docs/operations/vrpn_pants_profiles.md).
This launch does not start Runner. Do not run parallel NatNet and VRPN input
stacks publishing the same robot/ball outputs.

VRPN carries body poses, not a NatNet ModelDef. The receiver cannot infer or
certify the 17-point model, its pivot, or physical mounting accuracy from that
stream. Any advertised local asset hash identifies the selected repository
profile, **not a remotely measured hash of the CMTracker project**.

## PR approval gate: real-machine validation

Submit this asset as an **offline-verified candidate**. Approval for merging
requires a CMTracker import and secured-robot validation; passing the offline
tests alone is not approval for live motion control. Keep motion control
disabled during the initial origin, axis and pose checks.

Attach the following evidence to the PR before approval:

- CMTracker version, imported filename/hash and a fresh native re-export;
  confirm all 17 points and that the pelvis pivot/axes survived import.
- Installed-shell photos, sticker size/thickness and a recorded comparison
  with the physical pelvis datum at several positions/headings. Record the
  measured error; do not treat the nominal CAD precision as physical accuracy.
- Measured VRPN units and table/world alignment, with sample raw poses and
  `/a3/mocap/pelvis_pose`. Verify that the latter retains physical pelvis
  height; the extra 0.76 m belongs only to the policy's flat-message convention.
- The selected V4 profile/body/side and tests for each side intended for use.
  Check P2's world transformation without changing the local marker geometry.
- Results for interrupted tracking, stale timestamps and unconfirmed setup:
  these must not leave usable stale localization at the controller.

If the importer changes the pivot/axes or the installed pose disagrees with
the physical pelvis, investigate and revise the asset/setup before approval.
Do not enable confirmation flags merely to bypass a failed check. A PR review
or a software profile selection cannot substitute for this recorded validation.

## Maintenance and offline checks

The existing runtime V4 profile files remain the distributed ROS configuration
source and are unchanged by this release packaging. If geometry changes, update
those files, their catalog hashes and this single release asset together; do
not edit only one copy. The regression test checks that only one import asset
is supplied, byte parity with the existing P1 runtime definition, geometry
parity with both runtime sides, station coordinates/order, scale, axes,
identity matrix and provenance.
From the repository root:

```bash
python3 -m pytest -q \
  hope_ws/src/hope_bringup/test/test_v3_vrpn_asset.py \
  hope_ws/src/hope_bringup/test/test_vrpn_pants_contract.py
```

See also the [shell README](../README.md), [CAD registration](../documents/CAD_registration.md)
and [shared mocap README](../../../mocap/README.md). Existing STL files,
Motive assets and runtime configuration are unchanged.
