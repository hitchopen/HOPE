# Motion capture interface

HOPE drives its planner from an external motion-capture system that streams rigid-body
poses into ROS 2. During competition the arena streams the named rigid bodies `Ball`,
`P1`, and `P2` (`Ball` is first in `/poses`; the default VRPN bringup aggregates only it). A
`Table` asset is used for setup/calibration only and appears only in training-data
recordings — it is not streamed during competition. This document defines the generic
frame and topic contract
the rest of the stack expects. It is deliberately vendor-neutral — any optical
motion-capture rig that can publish the topics below will work. Configure your own rig's
network address in the launch files (see `hope_ws/`).

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

Units are SI: metres and seconds. Cross-sensor calibration requires acquisition
timestamps expressed in one shared ROS clock epoch; matching numeric fields
that actually represent receipt time is not sufficient.

These dimensions and landmarks are not duplicated by hand anywhere: the single source
of truth is
`hope_training/whole_body_tracking/source/whole_body_tracking/whole_body_tracking/tasks/table_tennis/geometry.py`,
which derives everything from [`configs/ball_physics.yaml`](../configs/ball_physics.yaml)
so the simulator, planner, and evaluator share one world.

The competition stream contains vendor-defined **6-DOF rigid bodies** (`Ball`, plus `P1`/`P2`
where used). Conceptually, the ball
pose may be inspected as `(x, y, z, pitch, yaw, roll)`, but the ROS 2 wire contract uses
`geometry_msgs/Pose`: position `(x, y, z)` plus quaternion orientation
`(qx, qy, qz, qw)`. Euler angles are derived using an explicitly documented axis and rotation
order; never write pitch/yaw/roll values directly into `Pose.orientation`.

The current no-spin planner consumes only the ball position, so preserving orientation does
not change its input behavior. The orientation remains available for validation and future
spin-aware estimation. The robot's control-facing root orientation (yaw) is taken from the
robot IMU, not from mocap — this is why the policy observation includes an IMU-derived
`base_forward_xy` term (see [POLICY_INTERFACE.md](../docs/POLICY_INTERFACE.md)). Treat a
mocap root-yaw estimate as advisory unless a robot integration contract says otherwise.

## Topics

| Topic | Type | Rate (typical) | Meaning |
|-------|------|----------------|---------|
| `/poses` | `geometry_msgs/PoseArray` | ~300 Hz | Full tracked pose(s) in the world frame. `Ball` is always first; `P1` and `P2` marker-cluster poses may follow when configured. The planner reads `Ball` at `ball_pose_index` and currently consumes only its position. |
| `/tf` (optional) | `tf2_msgs/TFMessage` | ~300 Hz | Named transforms for `world → Ball` (and `world → P1`, `world → P2` where streamed). The OptiTrack relay publishes these; the shipped Chingmu/VRPN path does **not**, so add a `tf2_ros` broadcaster if that deployment needs named transforms. |
| `<robot_root_pose>` | `geometry_msgs/PoseStamped` | ~300 Hz (optional) | Full declared robot-root pose in the world frame (`pelvis` on Unitree G1; `pelvis_link` on Agibot A3), obtained after applying the marker-to-root calibration and used for fixed-station recentring. Topic name is deployment-specific. |

The planner consumes every incoming mocap sample for its estimator but runs its
(more expensive) trajectory solve at **at most 50 Hz**. For OptiTrack, HOPE
configures `topics.header_time: ros_latency_compensated`: the driver maps each
frame into the local ROS epoch as receipt time minus NatNet's per-frame Camera
and Motive processing latencies and the deployment-measured
`topics.network_latency_ms`. Bare `ros` is receipt time and creates a
velocity-proportional spatial bias; bare `camera` is Motive's high-resolution
clock in a different epoch and must not be mixed directly with ROS stamps. The
independent pelvis source must likewise stamp at acquisition in the same ROS
epoch. For VRPN, enable vendor timestamps only with a verified NTP/PTP or
equivalent clock mapping.

## Bringing up mocap

HOPE ships two source-specific paths that converge at the identical planner interface:

| Venue system | Vendor transport | Raw ROS 2 message | HOPE adapter |
|---|---|---|---|
| **OptiTrack Motive** | **NatNet UDP** (not VRPN) | `/optitrack/poses`, `motion_capture_tracking_interfaces/NamedPoseArray` | `optitrack_mct_relay` → `/poses` |
| **Chingmu CMTracker/MCServer** | **VRPN** | `/vrpn_mocap/<sender>/pose_id_<sensor_id>`, `geometry_msgs/PoseStamped` | `pose_to_posearray` → `/poses` |

### OptiTrack / Motive: NatNet

Use the `optitrack` backend for Motive. Enable NatNet, set **Up Axis = Z**, prefer unicast,
and stream rigid bodies named exactly `Ball`, `P1`, and `P2`. NatNet uses the Motive command
port (normally UDP 1510); the driver obtains the data-port and unicast/multicast details from
the server response. Motive's legacy VRPN stream on port 3883 is **not used** by this backend.

```text
Motive NatNet → motion_capture_tracking_node (namespace /optitrack)
             → /optitrack/poses (NamedPoseArray)
             → optitrack_mct_relay → /poses (PoseArray, Ball at index 0)
```

`NamedPoseArray` carries one header plus entries of the form `{name, Pose}`. The relay maps
the case-sensitive Motive asset names into the HOPE topics, preserves the position and
quaternion, and only publishes `/poses` on a frame that contains `Ball`; it never repeats a
stale ball pose during an occlusion. The raw topic is intentionally namespaced because its
message type differs from the HOPE `/poses` `PoseArray` contract.

### Calibrating a humanoid P1 body to `pelvis_link`

This is a required **one-time bringup calibration** for each installed marker
shell or whenever its Motive rigid-body definition changes.

Motive independently solves the P1 rigid body from the physical marker
constellation and publishes `world → P1`. A separate calibration-only
measurement source must publish the A3 pelvis as
`geometry_msgs/PoseStamped`, also in `world`. Because the marker shell is
rigidly attached to the robot pelvis, the following relative transform is
constant even while A3 moves:

```text
P1 → pelvis_link
  = inverse(world → P1 at time t)
    ×       (world → pelvis_link at the matching time)
```

The second input must be an **independent, real-time, full-6DOF** A3 pose; it
is neither hard-coded nor derived from `/P1/pose`. The repository's A3
hardware interface publishes `/body_drive/pelvis_imu/data`, which has no
absolute translation and cannot supply this input. For real-hardware
calibration, the robot integration must first publish an external tracker or
state-estimator result such as `/a3/calibration/pelvis_pose`, with
`header.frame_id: world` and the same clock domain as `/P1/pose`. The existing
`/sim/a3/pelvis_pose` is a MuJoCo-only producer in `odom`, suitable only when
the simulated P1 input is expressed in that same frame. The calibrator checks
that both publishers exist, synchronizes their messages, rejects outliers, and
robustly averages the constant `P1 → pelvis_link` result. It consumes the
solved 6-DOF `/P1/pose`, not individual marker topics, so marker topic names
and ordering are irrelevant. It consumes no Table topic or TF. Do not run
`p1_pelvis_tf_publisher` during collection, because that would make the target
transform circular.

A common `Table` transform would cancel algebraically from every relative-pose
sample, so enabling that asset would add setup/competition divergence without
providing any information to this calibration.

Residual RMS is only a **consistency** metric. The tool separately requires
both accepted trajectories to span at least 0.10 m translation, 10 degrees
rotation, and 1 second by default, with strictly increasing and at least 90%
unique timestamps and at least a 50 Hz accepted rate. A stationary capture
therefore fails even if its residual RMS is zero. These
measured excitation and pair-skew statistics are saved in JSON. They are
necessary checks, not proof that the pelvis producer is independent or that
all systematic latency has been removed; source independence remains an
operator/integration precondition.

The installed executable
[`p1_pelvis_calibrator`](../hope_ws/src/hope_bringup/scripts/p1_pelvis_calibrator)
is a thin ROS 2 wrapper around the implementation
[`p1_pelvis_calibration_impl.py`](../hope_ws/src/hope_bringup/scripts/p1_pelvis_calibration_impl.py).
After building and sourcing `hope_ws`, run:

```bash
ros2 run hope_bringup p1_pelvis_calibrator \
  --p1-topic /P1/pose \
  --pelvis-topic /a3/calibration/pelvis_pose \
  --reference-frame world \
  --pelvis-frame pelvis_link \
  --p1-frame P1 \
  --output calibration/p1_to_pelvis.json
```

The required JSON contains the measured constant transform, quality metrics,
the optional Motive pivot-axis rotation and local translation, and the CAD
centroid cross-check. Keep this JSON as the setup record and load it at normal
runtime:

```bash
ros2 run hope_bringup p1_pelvis_tf_publisher \
  --calibration-file calibration/p1_to_pelvis.json
```

This produces the TF chain:

```text
world ── dynamic mocap ──> P1 ── static calibrated TF ──> pelvis_link
```

As an optional alternative, apply the reported rotation and translation to
the P1 rigid-body pivot in Motive, save the asset/profile, restart streaming,
and rerun the calibrator. The second result should be approximately identity.
If the Motive pivot is corrected, do **not** run the static publisher, because
that would apply the offset twice. See the complete
[OptiTrack setup procedure](../docs/OPTITRACK.md#calibrating-p1-to-an-a3-pelvis_link).

Launch it with:

```bash
ros2 launch hope_bringup hope_bringup.launch.py \
  mocap_backend:=optitrack \
  mocap_server:=<MOTIVE_PC_IP> \
  mocap_network_latency_ms:=<MEASURED_ONE_WAY_MS>
```

The `Table` asset may be recorded through this relay during a separate setup/calibration
session, but must be disabled or omitted from Motive's competition stream. Consequently no
live `/table/pose`, table TF, or table entry reaches the competition `/poses` stream. See the
full operational guide in [`docs/OPTITRACK.md`](../docs/OPTITRACK.md).

### Chingmu / CMTracker: VRPN

CMTracker/MCServer serves the named rigid bodies as VRPN trackers directly. Configure it to
stream Z-up so no software frame conversion is needed. The vendored ROS 2 client
(`hope_ws/src/vrpn_mocap`, MIT licensed) publishes one `PoseStamped` topic per tracker (with
`multi_sensor: true`), and `hope_bringup/pose_to_posearray` copies the complete pose —
including its quaternion — into `/poses`.

```text
CMTracker/MCServer VRPN → /vrpn_mocap/<sender>/pose_id_<sensor_id> (PoseStamped)
                         → pose_to_posearray → /poses (PoseArray, Ball at index 0)
```

Topic and asset names are case-sensitive. Configure them for the actual name shown by Motive
or CMTracker instead of assuming that `Ball` and `ball` are interchangeable.

For testing without a physical rig, `hope_ws/src/hope_bringup/scripts/fake_ball_publisher`
publishes synthetic `/poses` trajectories (`fake_optitrack_publisher` does the same at the
OptiTrack driver level).

## What is intentionally not here

This is a generic interface description, not a venue setup guide. Rig-specific hardware,
camera counts, network addresses, and calibration recordings are deployment details you
supply for your own environment.

For a worked example of one such environment, see the preserved arena design document —
[HOPE_Motion_Capture_System_and_Coordinates_Reference_Setup.md](HOPE_Motion_Capture_System_and_Coordinates_Reference_Setup.md) ([中文](HOPE_Motion_Capture_System_and_Coordinates_Reference_Setup_ZH.md)). It
covers OptiTrack/Motive and Chingmu/CMTracker configuration, camera layout,
tracked-object taxonomy, robot root-frame registration, and 6-DOF ball tracking.
For the general frame and topic contract, treat this README as authoritative.
