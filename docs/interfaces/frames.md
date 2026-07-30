# Frames

HOPE uses a single right-handed, ROS 2 REP-103 world frame shared by mocap, the
planner, training, evaluation, and deploy. Its provenance is the real
ball-capture fit recorded in
[`configs/ball_physics.yaml`](../../configs/ball_physics.yaml); the geometry
module
[`tasks/table_tennis/geometry.py`](../../hope_training/whole_body_tracking/source/whole_body_tracking/whole_body_tracking/tasks/table_tennis/geometry.py)
derives every dimension and landmark from that file, so nothing below is
duplicated by hand.

## World / table frame

| Axis | Direction | Range over the table |
|------|-----------|----------------------|
| +x | forward, toward the opponent (P2) | `[0, 2.74]` m |
| +y | left, from the robot's (P1) perspective | table occupies `[-1.525, 0]` m |
| +z | up | `0` **is the table surface** |

- **Origin**: the near-side **left** corner of the table *surface*, from P1's
  perspective.
- The floor is at `z = -0.76` m; the net plane is at `x = 1.37` m.
- Landmarks (net center `(1.37, -0.7625, 0)`, opponent-half center
  `(2.055, -0.7625, 0)`, …) are also published as named static-transform frames —
  see [`hope_ws/src/hope_bringup/config/hope_world_frame.yaml`](../../hope_ws/src/hope_bringup/config/hope_world_frame.yaml).
- Units are SI (metres, seconds) throughout.

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
- All planner quantities (`RacketCommand` position/velocity) and all policy
  racket-target observation terms are expressed in this world frame — see
  [POLICY_INTERFACE.md](../POLICY_INTERFACE.md) and
  [PLANNER_INTERFACE.md](../PLANNER_INTERFACE.md).

## Mocap frame

The arena motion-capture stream carries the named rigid bodies — `Ball`, `P1`,
and `P2` — in the same world frame during competition (a `Table` asset exists
for calibration only and is not streamed). P1/P2 are marker-cluster frames; a
calibrated static transform maps each one to its robot's declared URDF root
(`pelvis` on Unitree G1; `pelvis_link` on Agibot A3). Each ROS 2 pose contains position `(x, y, z)` and
quaternion orientation `(qx, qy, qz, qw)`; the current no-spin planner consumes
only the Ball position. The robot's control-facing root yaw comes from the
robot IMU, not from mocap. The authoritative mocap frame
and topic contract is [`mocap/README.md`](../../mocap/README.md); the preserved
arena design document
[`mocap/HOPE_Motion_Capture_System_and_Coordinates_Reference_Setup.md`](../../mocap/HOPE_Motion_Capture_System_and_Coordinates_Reference_Setup.md)
covers rig setup (camera layout, marker placement, vendor frame conversions) and
predates the current stack — where the two differ, the README contract wins.
