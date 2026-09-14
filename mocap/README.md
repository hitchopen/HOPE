# Motion capture interface

Laptop ROS commands in this document run inside the Ubuntu 24.04/ROS 2 Jazzy
`hope` Distrobox. A new Laptop must first complete
[`docs/DISTROBOX_SETUP.md`](../docs/DISTROBOX_SETUP.md); do not build these
workspaces with host or Conda Python.

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
| `/poses` | `geometry_msgs/PoseArray` | 200 Hz adapter default | Full tracked pose(s) in the world frame. `Ball` is always first; `P1` and `P2` marker-cluster poses may follow when available. The planner reads `Ball` at `ball_pose_index` and currently consumes only its position. |
| `/tf` (optional) | `tf2_msgs/TFMessage` | 200 Hz OptiTrack relay output | Named transforms for `world → Ball` (and `world → P1`, `world → P2` when available). The HOPE OptiTrack relay publishes these from the filtered named-pose array; the raw NatNet adapter does not publish duplicate TF. The shipped Chingmu/VRPN path does **not** publish TF, so add a `tf2_ros` broadcaster if that deployment needs named transforms. |
| `<robot_root_pose>` | `geometry_msgs/PoseStamped` | source-dependent (optional) | Full declared robot-root pose in the world frame (`pelvis` on Unitree G1; `pelvis_link` on Agibot A3), obtained after applying the marker-to-root calibration and used for fixed-station recentring. Topic name is deployment-specific. |

The planner consumes every incoming mocap sample for its estimator but runs its
(more expensive) trajectory solve at **at most 50 Hz**. For OptiTrack, HOPE
configures `topics.header_time: camera_utc`. The driver uses NatNet echo clock
synchronization to map Motive's `CameraMidExposureTimestamp` QPC tick into the
adapter's monotonic clock, then subtracts that measured age from
`RCL_SYSTEM_TIME`—the Unix epoch disciplined by Chrony on the Linux adapter.
Bare `ros` is receipt time and creates a velocity-proportional spatial bias;
bare `camera` is Motive's high-resolution clock in a different epoch and must
not be mixed directly with ROS stamps. The independent pelvis source must
likewise stamp at acquisition in the same ROS epoch. The independent
`VRPN2ROS2` deployment preserves the VRPN server report `timeval` and rejects
samples that do not agree with the adapter's NTP-disciplined system clock
within configured bounds. It also monitors the sliding minimum of total age for
runtime shifts and can compare it with a commissioned expected minimum. Those
checks validate an operating regime; they do not synchronize the server clock
or distinguish its offset from one-way transport delay. The proprietary
server's camera-exposure timestamp semantics still require vendor documentation
or a hardware-trigger comparison.

Both raw adapters receive and validate every source report before reducing ROS
traffic. NatNet2ROS2 publishes only `/optitrack/poses`, capped at 200 Hz and
strictly filtered to the available exact-name rigid bodies `Ball`, `P1`, and
`P2` in that order. A valid selected frame with none of those bodies is an
empty-array heartbeat, allowing operators to distinguish competition-body
tracking loss from NatNet/adapter transport loss. It publishes no marker point
cloud, raw TF, Table, skeleton, or arbitrary Motive asset. VRPN2ROS2 independently caps each pose, velocity,
and acceleration topic per sensor at 200 Hz. Configure
`output_rate_hz:=<Hz>` on either launch command, or use `0.0` for every accepted
source report. Downsampling preserves the selected source header timestamp; it
is not a timestamp resampler. The production C++ packetizer retains a
time-based window (`flight_window_s`, default 0.18 s), so adapter-rate changes
do not require a sample-count conversion. Keep enough output samples to satisfy
the packetizer's minimum sample/span requirements.

## Bringing up mocap

HOPE ships two source-specific paths that converge at the identical planner interface:

| Venue system | Vendor transport | Raw ROS 2 message | HOPE adapter | Timestamp trust model |
|---|---|---|---|---|
| **OptiTrack Motive** | **NatNet UDP** (not VRPN) | `/optitrack/poses`, `motion_capture_tracking_interfaces/NamedPoseArray` | `optitrack_mct_relay` → `/poses` | Motive `CameraMidExposureTimestamp` mapped to adapter time by measured echo clock synchronization, with mapping uncertainty; acquisition-event semantics. |
| **Chingmu CMTracker/MCServer** | **VRPN** | `/vrpn_mocap/<sender>/pose_id_<sensor_id>`, `geometry_msgs/PoseStamped` | `pose_to_posearray` → `/poses` | Server report `timeval` trusted only after absolute-age, sliding-minimum, NTP, and optional commissioned-baseline checks; camera exposure → report delay remains unknown. |

Both backends can publish numerically compatible Unix/ROS timestamps, but they
do not yet represent a proven identical physical event. Switching between
NatNet camera-mid-exposure time and VRPN server-report time is therefore not
timing-neutral; preserve the unknown exposure-to-report interval in estimator
and strike-time error budgets until Chingmu supplies vendor evidence or a
hardware-trigger measurement.

### OptiTrack / Motive: NatNet

Use the `optitrack` backend for Motive. Enable NatNet, set **Up Axis = Z**, prefer unicast,
and stream rigid bodies named exactly `Ball`, `P1`, and `P2`. NatNet uses the Motive command
port (normally UDP 1510); the driver obtains the data-port and unicast/multicast details from
the server response. Motive's legacy VRPN stream on port 3883 is **not used** by this backend.

```text
Motive NatNet → NatNet2ROS2 workspace (namespace /optitrack)
             → /optitrack/poses (NamedPoseArray)
             → optitrack_mct_relay → /poses (PoseArray, Ball at index 0)
```

`NamedPoseArray` carries one header plus entries of the form `{name, Pose}`. The relay maps
the case-sensitive Motive asset names into the HOPE topics, preserves the position and
quaternion, and only publishes `/poses` on a frame that contains `Ball`; it never repeats a
stale ball pose during an occlusion. NatNet2ROS2 admits only `Ball`, `P1`, and `P2`, in that
order when available; absent bodies are silently omitted, and a frame with none is an empty
array heartbeat. The raw topic is intentionally
namespaced because its message type differs from the HOPE `/poses` `PoseArray` contract.

### Calibrating a humanoid P1 body to `pelvis_link`

The A3-specific marker-CAD procedure, installed-layout requirements and
receipt lifecycle are maintained in
[agibot/README.md](../agibot/README.md#v3-sticker-p1-to-pelvis-calibration).
The current profile is the 24-station v3 sticker shell, using the optical-centre
TF table in ROS `pelvis_link`; old ten-marker receipts cannot be reused.
Whether capture is automated depends on the selected operator integration;
do not assume every PREPARE/Ready action runs it. The independent pose-pair
method below is an audit route, not a runtime receipt generator.

#### Legacy independent pose-pair route

The older `p1_pelvis_calibrator` can compare synchronized P1 and independently
measured full-6DOF pelvis poses, or run a simulation check. It now uses the
same v3 sticker table for its nominal CAD cross-check, but writes an
**unapproved audit record**, not a production runtime receipt. The complete
[pose-pair audit procedure](../docs/OPTITRACK.md#legacy-independent-pose-pair-method)
covers the required independent source and timestamp/excitation checks.

No checked-in real-robot node produces the required
`/a3/calibration/pelvis_pose`. Never supply `/a3/mocap/pelvis_pose` or any
other P1-derived result as that input: the calibration would be circular.
Use the v3 marker-CAD procedure above for the production receipt, and never
stack two corrections or competing TF publishers.

Build and launch the raw adapter independently, then launch the HOPE relay and
planner:

```bash
source NatNet2ROS2/install/setup.bash
ros2 launch motion_capture_tracking natnet2ros2.launch.py \
  hostname:=<MOTIVE_PC_IP> interface_ip:=<ADAPTER_WIRED_IP>

source NatNet2ROS2/install/setup.bash
source hope_ws/install/setup.bash
ros2 launch hope_bringup hope_bringup.launch.py mocap_backend:=optitrack
```

The competition NatNet adapter never exports `Table`. Record or inspect that setup asset in
Motive (or with dedicated calibration tooling) during a separate setup/training-data session;
do not route it through the competition ROS adapter. Consequently no live `/table/pose`, table
TF, or table entry can reach the competition `/poses` stream. See the full operational guide
in [`docs/OPTITRACK.md`](../docs/OPTITRACK.md).

### Chingmu / CMTracker: VRPN

CMTracker/MCServer serves the named rigid bodies as VRPN trackers directly. Configure it to
stream Z-up so no software frame conversion is needed. The independent ROS 2 client
([`VRPN2ROS2`](../VRPN2ROS2/README.md), MIT licensed) publishes one `PoseStamped` topic per
tracker (with `multi_sensor: true`), and `hope_bringup/pose_to_posearray` copies the complete
pose—including its quaternion and source header—into `/poses`.

```text
CMTracker/MCServer VRPN → /vrpn_mocap/<sender>/pose_id_<sensor_id> (PoseStamped)
                         → pose_to_posearray → /poses (PoseArray, Ball at index 0)
```

The checked-in VRPN client polls at 500 Hz, above the typical 300–360 Hz
source stream. Keep its `update_freq` at or above the measured venue stream
rate so client socket/polling delay does not consume the tightened timestamp
age budget. The separate `output_rate_hz` parameter defaults to 200 Hz per ROS
topic/sensor and reduces DDS traffic only after every report has passed the
timestamp checks.

Build and launch the adapter separately, then start HOPE from a second terminal:

```bash
cd VRPN2ROS2
colcon build --symlink-install
source install/setup.bash
ros2 launch vrpn_mocap client.launch.yaml \
  server:=<CHINGMU_SERVER_IP> port:=3883

# Separate terminal
source VRPN2ROS2/install/setup.bash
source hope_ws/install/setup.bash
ros2 launch hope_bringup hope_bringup.launch.py \
  mocap_backend:=vrpn \
  ball_pose_topic:=/vrpn_mocap/Ball/pose_id_0
```

Before play, run the `vrpn_timestamp_probe.py` acceptance gate documented in
[`VRPN2ROS2/README.md`](../VRPN2ROS2/README.md). The 100 ms old-age default is
only for bring-up: tune it to the wired venue measurements and enable the
commissioned expected-minimum gate before competition.

Topic and asset names are case-sensitive. Configure them for the actual name shown by Motive
or CMTracker instead of assuming that `Ball` and `ball` are interchangeable.
VRPN sensor indices must be in the adapter's supported 0–255 range; normal
single-sensor rigid bodies use index 0.

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
