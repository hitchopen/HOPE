# Frames

HOPE uses a single right-handed, ROS 2 REP-103 world frame shared by mocap, the
planner, training, evaluation, and deploy. The frame is a named contract —
**`table_p1_to_p2_v1`** — pinned, together with the frame names, landmarks, and
calibrated transforms, in
[`hope_ws/src/hope_bringup/config/hope_world_frame.yaml`](../../hope_ws/src/hope_bringup/config/hope_world_frame.yaml).
Its provenance is the real ball-capture fit recorded in
[`configs/ball_physics.yaml`](../../configs/ball_physics.yaml); the geometry
module
[`tasks/table_tennis/geometry.py`](../../hope_training/whole_body_tracking/source/whole_body_tracking/whole_body_tracking/tasks/table_tennis/geometry.py)
derives every dimension and landmark from that file, so nothing below is
duplicated by hand.

## World / table frame (`table_p1_to_p2_v1`)

| Axis | Direction | Range over the table |
|------|-----------|----------------------|
| +x | forward, from P1 toward the opponent (P2) | `[0, 2.74]` m |
| +y | left, from the robot's (P1) perspective | table occupies `[-1.525, 0]` m |
| +z | up | `0` **is the table surface** |

- **Origin**: the near-side **left** corner of the table *surface*, from P1's
  perspective (at the venue this is Motive's manually aligned table-surface
  origin; the `PPT` table rigid body is not a runtime dependency).
- The floor is at `z = -0.76` m; the net plane is at `x = 1.37` m.
- Landmarks (net center `(1.37, -0.7625, 0)`, `p1_half_center`
  `(0.685, -0.7625, 0)`, `p2_half_center` `(2.055, -0.7625, 0)`, …) are also
  published as named static-transform frames. The frame names are the
  contract's P1/P2 vocabulary — `p1_half_center` / `p2_half_center`, not
  "robot half" / "opponent half".
- Units are SI (metres, seconds) throughout.

## `policy_z_offset` — table z vs policy z

Mocap streams in the table frame (`z = 0` at the table **surface**); the
policy/training world puts `z = 0` on the **floor**. The bridge is the contract
value `planner.policy_z_offset: 0.76` in `hope_world_frame.yaml`: the deploy
chain adds it **exactly once** (in the planner/relay layer) when converting
table-frame mocap into policy-frame quantities. Applying it twice — or not at
all — shifts every racket target by a table height; treat it as part of the
frame contract, not a tunable.

## Mocap frame and calibrated transforms

The arena motion-capture stream carries the named rigid bodies — `Ball`, `P1`,
and `P2` — in the world frame during competition (a `Table` asset exists for
calibration only and is not streamed). P1/P2 are marker-cluster frames; the
calibrated constant transform mapping each one to its robot's URDF root
(`pelvis_link` on the Agibot A3) is the `mocap_to_base_link` block of
`hope_world_frame.yaml`. Discipline:

- The values are **never** hand-typed identities: each calibrated entry is
  pinned to a fail-closed receipt (SHA-256) under
  [`hope_ws/calibration_receipts/`](../../hope_ws/calibration_receipts) —
  uncalibrated entries say so (`calibrated: false`) and zero values there are
  inert placeholders, never an identity assertion.
- Two calibration routes exist; **use one, never both** (running both applies
  the correction twice): the upstream pose-pair route
  (`p1_pelvis_calibrator` + `p1_pelvis_tf_publisher`) and the venue-proven
  CAD-registration route (`p1_marker_cad_calibrator`, which registers the live
  Motive marker layout against the A3 hip-shell CAD at
  [`agibot/pku/hip_marker_shell/`](../../agibot/pku/hip_marker_shell)). See
  [`docs/OPTITRACK.md`](../OPTITRACK.md) for the operational walkthrough.

Each ROS 2 pose contains position `(x, y, z)` and quaternion orientation
`(qx, qy, qz, qw)`; the no-spin planner consumes only the Ball position, while
the base-pose relay forwards the full calibrated P1 pose (position + quaternion,
with validity flags) to the runner — which signal feeds which observation term
is pinned per contract in [POLICY_INTERFACE.md](../POLICY_INTERFACE.md). The
authoritative mocap topic contract is [`mocap/README.md`](../../mocap/README.md);
the preserved arena design document
[`mocap/HOPE_Motion_Capture_System_and_Coordinates_Reference_Setup.md`](../../mocap/HOPE_Motion_Capture_System_and_Coordinates_Reference_Setup.md)
covers rig setup (camera layout, marker placement, vendor frame conversions) and
predates the current stack — where the two differ, this contract wins.

## Training / evaluation placement

- Each Isaac Lab environment's local origin sits at the table-surface height, so
  an asset's environment-local position **is** its world-frame position; with
  multiple environments, every environment is an independent court anchored at
  its own origin.
- The robot stands on the P1 side, on the floor (`z = -0.76` m), centered on the
  table width (`y = -0.7625` m) at `x = -0.5` m (behind the near table end),
  facing +x toward P2. The root body is `pelvis_link`.
- The racket is mounted on the right wrist (`right_wrist_yaw_Link`); the
  dedicated racket body is `pingpang_red_Link` where the asset keeps it (URDF
  import usually merges fixed joints into the wrist body — the code falls back
  accordingly). See [`A3_ASSETS.md`](../../A3_ASSETS.md).
- All planner quantities (`RacketCommand` position/velocity and the flat wire)
  and all policy racket-target observation terms are expressed in this world
  frame — see [POLICY_INTERFACE.md](../POLICY_INTERFACE.md) and
  [PLANNER_INTERFACE.md](../PLANNER_INTERFACE.md).
