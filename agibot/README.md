# Agibot A3 assets, hardware and reference software

This folder contains the A3 source robot models, vendor deployment/simulation
references, robot clock-sync package, and contributed mounting hardware used
by HOPE. It is not a single buildable workspace. Use this page as the folder
index; component READMEs retain their installation and safety procedures.

For the supported HOPE policy runner, use [a3_deploy/](../a3_deploy/) and
[RUN_ON_AGIBOT](../docs/RUN_ON_AGIBOT.md). Training and generated Isaac assets
are documented in [A3_ASSETS.md](../A3_ASSETS.md), not built in this folder.

## Folder map

| Path | Contents and entry point |
| --- | --- |
| [URDF/A3T2.5-URDF-std-pingpang/](URDF/A3T2.5-URDF-std-pingpang/) | Racket-equipped A3 source URDF, meshes, joint configuration, metadata and authoring helpers; the default source for HOPE's Isaac asset preparation. |
| [URDF/a3_t2d5/](URDF/a3_t2d5/) | Non-racket A3 reference variant, with `urdf/model.urdf` and its own meshes. |
| [code_deployment/](code_deployment/) | Vendor C++ deployment example, body-drive I/O, runtime configuration and packaging. Start with the [deployment README](code_deployment/a3_deploy_example/README.md) and [RobotIOBackend guide](code_deployment/a3_deploy_example/README_robot_io_backend.md). |
| [A3_MuJoCo_Sim/](A3_MuJoCo_Sim/README.md) | Vendor AimRT/MuJoCo simulation reference; implementation under `aimrt_mujoco_sim/`. |
| [ntp_sync/](ntp_sync/README.md) | A3 HDU Chrony and supervised HDU-to-MDU PTP clock synchronization; includes installation, verification and recovery procedures. |
| [mocap_sticker_shell_v3/](mocap_sticker_shell_v3/README.md) | Dual-mode pelvis marker shell: printable halves, OpenSCAD preview, instructions, marker-to-`pelvis_link` tables and geometry validation. |
| [pku/](pku/README.md) | Original contributed mounting references: the ten-marker hip shell and wrist racket adapter, with their source records and mounting guidance. |
| [agibot_left_gripper_serve_addon.STL](agibot_left_gripper_serve_addon.STL) | Developer-supplied left-gripper serving fixture. **Metre-scale coordinates; see the printing scale warning below.** |

## Robot models and software boundaries

The racket-equipped source model is
[`URDF-JOINT-LINK.urdf`](URDF/A3T2.5-URDF-std-pingpang/urdf/URDF-JOINT-LINK.urdf).
It includes `right_hand_pingpang_Link` and the fixed `pingpang_red_Link` /
`pingpang_black_Link` racket bodies. Keep meshes, configuration and metadata
with their matching URDF; do not substitute the non-racket variant without
updating and validating the consuming application.

The canonical policy joint order remains
[`joint_order_agibot_a3.yaml`](../hope_training/config/joint_order_agibot_a3.yaml).
Use [A3_ASSETS.md](../A3_ASSETS.md) for the generated Isaac copy and
[POLICY_INTERFACE](../docs/POLICY_INTERFACE.md) for the policy I/O contract.

The vendor deployment example is useful for studying state/command topics,
runtime configuration and packaging, but is separate from HOPE's native
runner in [`a3_deploy/a3_deploy_example/`](../a3_deploy/a3_deploy_example/README.md).
Likewise, HOPE's closed-loop rehearsal uses
[`a3_deploy/A3_MuJoCo_Sim/`](../a3_deploy/A3_MuJoCo_Sim/README.md), not this
folder's vendor simulation copy. Training runs in Isaac Lab; MuJoCo is used
for evaluation and rehearsal. The high-level serving application has its own
[guide](../apps/a3_mujoco_serve/README.md).

## Serving gripper addon

The release asset is
[`agibot_left_gripper_serve_addon.STL`](agibot_left_gripper_serve_addon.STL).
This developer-supplied mesh replaces the earlier gripper STEP on
`nightly_built`; the earlier locally repaired STL is not the release master.

### Printing scale

STL does not encode units. This file's coordinates are **metre-scale**, unlike
the millimetre-scale v3 shell STLs:

- Import as **metres** if the application supports an explicit source unit.
- If the slicer assumes millimetres, multiply all axes by **1,000**
  (**100,000%**, not 1,000%). Do not apply this conversion twice.
- Verify the resulting dimensions are approximately
  **28.188 × 41.978 × 36.429 mm** in the original CAD X/Y/Z directions before
  orienting the part on the print bed.

The supplied file has 9,556 triangles and passes closed/manifold,
consistent-winding, duplicate/degenerate-facet and self-intersection checks.
Those checks establish mesh integrity, not attachment fit, print strength,
ball retention or serving-release clearance. Check the mating gripper, jaw
travel and ball release on a secured robot before operation. No verified
CAD-to-gripper link transform is supplied with this STL; its presence here
does not add a collision model or attachment to the serving application.

The reviewed file's SHA-256 is
`6db32e9cc54bd407206b9645c8176d9633288c48097d572d14b17b62a9abb9fe`.
Recheck dimensions and geometry when replacing the asset.

## Pelvis marker hardware

There are two distinct layouts; their marker names and calibration data are
not interchangeable:

- **Dual-mode shell v3:** 12 retained asymmetric stations with flat 12 mm sticker
  seats and central **blind** marker-ball mounting bores with side-access
  captive-nut pockets. The proposed through-hole/internal-nut redesign is
  not included. Use the
  [v3 README](mocap_sticker_shell_v3/README.md) for printable parts,
  installation, hardware limits and the sticker/mount-seat transform tables.
  Ball diameter alone does not determine the installed sphere centre; the
  ball/stud/spacer offset still needs measurement.
- **Original ten-marker shell:** five front (`f1`–`f5`) and five rear
  (`b1`–`b5`) markers. Its source geometry and coordinate reference remain
  under [pku/](pku/README.md) for historical reference. Its calibration
  receipts and ball-centre coordinates are not valid for the v3 stickers.

For A3, `pelvis_link` uses ROS axes **X forward, Y left, Z up**. CAD print
orientation is not a robot-frame registration. Use the v3
[CAD registration record](mocap_sticker_shell_v3/documents/CAD_registration.md)
and its metre-valued transform tables; rotating a part for printing does not
change those tables. Mesh validation does not certify physical fit or crash
survival for either shell.

### V3 sticker P1-to-pelvis calibration

`nightly_built` uses **12 round 12 mm stickers**, retaining station IDs
**S04, S05, S11, S12, S13, S14, S16, S18, S19, S20, S21, S22**, and the canonical
[sticker optical-centre table](mocap_sticker_shell_v3/documents/marker_transforms_pelvis_link_stickers.csv).
Both calibration tools read this single table; `colcon` installs it with
`hope_bringup`. It is already in `pelvis_link` metres, X forward / Y left /
Z up, including the accepted **0.20 mm** sticker-plus-adhesive thickness.
Do not add another thickness, CAD-axis rotation, centroid offset, or the old
ball stand-off. The mounting quaternions in the table are CAD conventions;
the optical fit uses marker positions, not a round sticker's unobservable roll.

The setup tool solves `p_P1 = R_P1_pelvis * p_pelvis + t_P1_pelvis` from the
live Motive MODELDEF and these CAD centres. The supplied aligned asset now targets
an **identity P1-to-pelvis correction**; the solver verifies that expectation
instead of forcing it. Motive's pivot and axes remain configurable. The table is nominal
CAD geometry, not proof of the installed shell's fit or a physical calibration.

Migration prerequisites:

1. Fit and inspect the v3 shell and all 12 retained stickers. Use sticker mode
   only; 12 mm balls require a separately measured ball-centre table/profile.
2. Import [A3_v3_12_stickers.motive](mocap_sticker_shell_v3/motive_asset/A3_v3_12_stickers.motive)
   into an isolated Motive project first. This native candidate already contains
   all 12 retained stations, is named `P1` internally, and targets the CAD
   `pelvis_link` origin. Confirm the marker count and unique streaming ID `9`;
   the retained source UUID can replace an existing body on import.
   Set Motive Streaming Up Axis to **Z Up**, keep the bridge's MODELDEF
   Y-up-to-Z-up conversion enabled, and do not add a second pose rotation.
   Import/re-export and live validation are still required. Enable rigid bodies
   and labeled markers; do not reset the aligned pivot or orientation.
   The companion CSV is not the import file. The separate
   [ideal CAD authoring definition](mocap_sticker_shell_v3/motive_asset/P1_stickers_definition.json)
   and [identity default](mocap_sticker_shell_v3/motive_asset/P1_to_pelvis_link_default.json)
   are references, not approved receipts. Do not import the unfiltered
   `motive_asset/source/A3.motive` export. See the
   [shell asset section](mocap_sticker_shell_v3/README.md#supplied-optitrack-asset-and-station-evidence)
   for provenance and the measured-point correspondence.
3. Restart the NatNet adapter after changing the asset so MODELDEF is fresh.
   Use verified retained Sxx names when available; otherwise the calibrator matches
   geometry with ambiguity/residual gates. Stream order and numeric member
   IDs alone do not identify a station. An explicit `--mapping` is only for
   physically verified member-ID-to-station correspondence.
4. Rebuild/source `hope_ws` and generate a **new live receipt**. Pre-alignment
   12-point, old 24-station and ten-marker
   receipts are rejected even when marked approved. The checked-in P1 YAML
   default remains uncalibrated; zero/identity is the intended aligned result,
   not permission to operate without verification or hand-approve a receipt.

Only run this procedure in the approved setup session with the robot safely
supported as required and the installed hardware matching the table. Do not assume
that a PREPARE/Ready button invokes it: the legacy TTY orchestration and the
integrated Runner console have different control paths. Follow the selected
[Foxglove/operator procedure](../foxglove/README.md).

On the external computer, first build/source the independent NatNet adapter
and `hope_ws`, then enable the marker topic with `publish_p1_markers:=true`;
normal named-pose output alone is insufficient. Follow the
[NatNet startup instructions](../NatNet2ROS2/README.md#launch-the-adapter), including
the explicit wired `interface_ip` and Motive labeled-marker prerequisites.
After PD_STAND has been reached and settled through the approved robot
procedure, run from the external computer's HOPE repository root:

```bash
source hope_ws/install/setup.bash
ros2 run hope_bringup p1_marker_cad_calibrator \
  --topic /optitrack/rigid_body_markers \
  --asset-name P1 \
  --minimum-frames 200 \
  --capture-duration 4 \
  --stationary-prepare \
  --attest-installed-layout \
  --output calibration/p1_to_pelvis.json
```

The 12 retained stations are the default; no marker-list override is needed. All 12 must be
**defined** in MODELDEF, but they need not be visible in the same frame. The
collector accumulates at least 30 non-occluded, point-cloud-solved samples for
each sticker across the capture. If a station stays hidden, correct the camera
coverage in the approved setup procedure; do not bypass the gate or move an
active robot to satisfy it. Default quality limits remain 3 mm registration
RMS / 6 mm maximum, 5 mm pairwise RMS, and 4 mm live RMS / 5 mm maximum.

`--attest-installed-layout` confirms the physical installation, not a measured
pelvis reference. The service uses the same v3 defaults and treats an operator
calibration request as that attestation; verify the installation before invoking
it. An accepted fit atomically
replaces `calibration/p1_to_pelvis.json` on that external computer. The relay
then reads the fixed `P1 → pelvis_link` transform for the run, composes it with
live `world → P1`, and publishes `/a3/base_pose_flat` plus the unshifted
diagnostic `/a3/mocap/pelvis_pose`. No recalculation occurs during play. The
robot receives `/a3/base_pose_flat`, not the JSON.

The receipt records the layout ID `A3_marker_shell_v3_stickers_12mm`, table
SHA-256, asset-frame revision `pelvis_link_aligned_native_y_up_20261004`,
per-member S-name correspondence, CAD transforms and live residuals.
Both the relay and optional static-TF publisher check the current layout/table/
asset-frame identity and explicit approval. A failed capture leaves the previous file
untouched and writes a rejected diagnostic receipt when analysis completes;
an old-layout file remains unusable. Historical receipts are retained for audit.
If the canonical table changes, review its coordinates, update the pinned
hash/profile in `hope_bringup/scripts/p1_marker_layout.py` and matching tests,
and rebuild both the installed table and code in the same change. Never bypass
a hash mismatch; a changed table requires fresh live calibration. Changing the
asset pivot/axes also invalidates the previous receipt even when the table is
unchanged. See [calibration/README.md](../calibration/README.md) for the runtime
file lifecycle and the distinction between an identity reference and an approved receipt.

Do not confuse this result with `/a3/calibration/pelvis_pose`, the independent
input to the older pose-pair calibrator. No checked-in hardware node produces
that independent topic, and supplying a P1-derived result would make the
calibration circular. See the
[independent pose-pair route](../mocap/README.md#legacy-independent-pose-pair-route)
and [OptiTrack setup guide](../docs/OPTITRACK.md#optional-marker-cad-calibration-p1-to-an-a3-pelvis_link)
for those separate workflows and the frame contract.

## Clock synchronization

[`ntp_sync/README.md`](ntp_sync/README.md) owns the A3 installation and safety
procedure: Chrony disciplines HDU system time, and supervised PTP services
distribute it to MDU. The adapter computer and robot must share an approved
NTP source or equivalent common epoch before external-mocap control; a
healthy ROS topic is not evidence of synchronized clocks.

Ordinary offline A3 applications can run without Internet/NTP, but external
mocap timing remains unqualified until the clock-health gates pass. Never
step the clock while the robot is standing or executing a policy; follow the
secured-robot procedure in the clock-sync runbook. The system-wide design is in the
[clock synchronization plan](../docs/HOPE_A3_Clock_Synchronization_Improvement_Plan.pdf).

## Documentation and source records

Keep folder inventory, hardware selection and gripper units on this page.
Keep detailed component installation/printing procedures in their linked
READMEs, and keep shared ROS/frame contracts in [mocap/](../mocap/README.md)
and [docs/interfaces/](../docs/interfaces/). Other repository overview pages
should link here rather than duplicate these A3-specific procedures.

Preserve the [repository license](../LICENSE), component licenses and original
contributor notices when redistributing or modifying these assets. This
index does not change the licensing or provenance of bundled materials.
