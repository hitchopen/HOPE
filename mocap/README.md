# Motion capture interface

HOPE PingPong drives its planner from an external motion-capture system that streams
**ball** (and optionally **robot base**) positions into ROS 2. This document defines the
generic frame and topic contract the rest of the stack expects. It is deliberately
vendor-neutral — any optical/VRPN motion-capture rig that can publish the topics below will
work. Configure your own rig's network address in the launch files (see `hope_ws/`).

## Coordinate frame

A single right-handed world frame is shared by mocap, planner, training, and the ball
physics model:

| Axis | Direction | Range over the table |
|------|-----------|----------------------|
| +x   | forward (toward the opponent half of the table) | `[0, length]` |
| +y   | left      | `[-width, 0]` |
| +z   | up        | `0` **is the table surface** |

The **origin is the near-side left corner of the table _surface_**, from the robot's
(P1's) perspective. Because `z = 0` is the playing surface, the floor sits at
`z = -0.76 m`.

Units are SI: metres and seconds. Timestamps are on a single shared clock.

These dimensions and landmarks are not duplicated by hand anywhere: the single source
of truth is
`hope_training/whole_body_tracking/source/whole_body_tracking/whole_body_tracking/tasks/table_tennis/geometry.py`,
which derives everything from [`configs/ball_physics.yaml`](../configs/ball_physics.yaml)
so the simulator, planner, and evaluator share one world.

The motion-capture system provides **positions only**. The robot's base orientation (yaw)
is taken from the robot IMU, not from mocap — this is why the policy observation includes an
IMU-derived `base_forward_xy` term (see [POLICY_INTERFACE.md](../docs/POLICY_INTERFACE.md)).
If your rig also produces a base yaw estimate, treat it as advisory only.

## Topics

| Topic | Type | Rate (typical) | Meaning |
|-------|------|----------------|---------|
| `/poses` | `geometry_msgs/PoseArray` | ~300 Hz | Tracked ball position(s) in the world frame. Only `position` is used; orientation is ignored. |
| `<robot_base_pose>` | `geometry_msgs/PoseStamped` | ~300 Hz (optional) | Robot base position in the world frame, used for the fixed-station recentring term. Topic name is a launch parameter. |

The planner consumes every incoming mocap sample for its estimator but runs its (more
expensive) trajectory solve at **at most 50 Hz**. Source timestamps are propagated so the
planner can extrapolate for capture latency.

## Bringing up mocap

The repository vendors a third-party VRPN ROS 2 client (`hope_ws/src/vrpn_mocap`, MIT
licensed) that bridges a VRPN motion-capture server to ROS 2. Point it at your server and
map your tracked rigid bodies to the topics above. A generic bringup that wires mocap into
the planner lives in `hope_ws/src/hope_bringup/`.

For testing without a physical rig, `hope_ws/src/hope_bringup/scripts/fake_ball_publisher`
publishes synthetic `/poses` trajectories.

## What is intentionally not here

This is a generic interface description, not a venue setup guide. Rig-specific hardware,
camera counts, network addresses, and calibration recordings are deployment details you
supply for your own environment.

For a worked example of one such environment, see the preserved arena design document —
[MOCAP_SYSTEM_DESIGN.md](MOCAP_SYSTEM_DESIGN.md) ([中文](MOCAP_SYSTEM_DESIGN_ZH.md)). It
covers the OptiTrack/Motive configuration, camera layout, tracked-object taxonomy,
`base_link` marker placement, and ball-tracking choices used for the original HOPE arena.
It predates this stack, so treat the contract above as authoritative where the two differ.
