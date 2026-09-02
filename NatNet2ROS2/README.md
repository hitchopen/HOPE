# NatNet2ROS2

`NatNet2ROS2` is the standalone OptiTrack Motive/NatNet adapter workspace.
It receives NatNet frames, maps Motive capture timestamps into the adapter
host's Chrony/NTP-disciplined ROS system-time epoch, and publishes a filtered
named-rigid-body stream. It does not contain the HOPE planner or HOPE topic
relay.

## Workspace contents

- `motion_capture_tracking`: NatNet client, timestamp mapping, ROS 2 node,
  hardened launch/configuration, and driver tests.
- `motion_capture_tracking_interfaces`: `NamedPoseArray` and related messages.

The package and executable names remain compatible with the original driver.

## Build

On a new Laptop, first create the Ubuntu 24.04/ROS 2 Jazzy `hope` environment
through [`docs/DISTROBOX_SETUP.md`](../docs/DISTROBOX_SETUP.md). Run the build
inside that container so ROS message generation uses `/usr/bin/python3` rather
than host or Conda Python:

```bash
distrobox enter hope
source /opt/ros/jazzy/setup.bash
cd "$HOME/workspace/HOPE/NatNet2ROS2"
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
```

If ROS message generation accidentally selects a uv/conda Python interpreter,
add `--cmake-args -DPython3_EXECUTABLE=/usr/bin/python3` to `colcon build`.

## Launch the adapter

`natnet2ros2.launch.py` is the only supported launch entry point. The two
upstream legacy launch files were removed because they bypassed the hardened
`hope_optitrack.yaml` configuration.

```bash
source NatNet2ROS2/install/setup.bash
ros2 launch motion_capture_tracking natnet2ros2.launch.py \
  hostname:=<MOTIVE_PC_IP>
```

In normal operation the adapter publishes exactly one ROS 2 topic:

- `/optitrack/poses`
  (`motion_capture_tracking_interfaces/msg/NamedPoseArray`)

Each array contains only the available, exact case-sensitive rigid-body names
`Ball`, `P1`, and `P2`, always in that order. Missing bodies are silently
omitted. A selected, timestamp-valid NatNet frame containing none of those
bodies is published as an empty array heartbeat. This exposes no additional
body data, but distinguishes a live NatNet/adapter path with lost or misnamed
competition assets from a stopped source, rejected timestamp path, adapter
process, or network path. The adapter and downstream relay each emit a
throttled diagnostic while empty frames continue. All other Motive
rigid bodies—including `Table`—as well as marker coordinates, skeletons, raw
TF, and arbitrary assets are excluded from ROS 2.
The downstream `optitrack_mct_relay` owns the per-body topics and TF output.

The only exception is P1 initialization on the external computer. Start the
adapter with marker output enabled before each PREPARE that begins a new run:

```bash
ros2 launch motion_capture_tracking natnet2ros2.launch.py \
  hostname:=<MOTIVE_PC_IP> publish_p1_markers:=true
```

This adds `/optitrack/rigid_body_markers` for the ten-marker capture. Every
PREPARE recomputes the transform and atomically replaces the external
computer's repository-relative `calibration/p1_to_pelvis.json`, even if the
file already exists. The computer then only reads that JSON and publishes
`/a3/base_pose_flat` for the rest of the run; no recalculation occurs while the
robot is playing. The robot receives `/a3/base_pose_flat`, never the JSON.

## ROS 2 output downsampling

The adapter receives and validates every NatNet source frame but publishes the
filtered named-pose array at no more than `topics.output_rate_hz`. The default
is **200 Hz**. The selected frame keeps its original acquisition timestamp; the
limiter does not average, interpolate, replay, or re-stamp data.

Set the maximum rate at launch:

```bash
ros2 launch motion_capture_tracking natnet2ros2.launch.py \
  hostname:=<MOTIVE_PC_IP> output_rate_hz:=200.0
```

Use `output_rate_hz:=0.0` to disable downsampling and publish every valid
source frame. The observed rate can still be lower than the requested cap when
the source is slower, timestamps fail their safety gates, or the host cannot
keep up. Changing the launch argument requires restarting the adapter.

The shipped sensor QoS uses a 100 Hz publisher deadline. Keep
`topics.poses.qos.deadline` below the selected output rate, lower it when using
an output rate of 100 Hz or less, or select QoS mode `none`; otherwise DDS can
correctly report expected deadline misses.

## ROS 2 NTP timestamp estimation

The supported `camera_utc` path converts Motive's capture clock into the
adapter host's NTP-disciplined absolute ROS clock in two stages:

1. Each NatNet frame carries `CameraMidExposureTimestamp`, a tick count in
   Motive's high-resolution QPC domain, plus the server's advertised
   `HighResClockFrequency`. A raw QPC tick is relative to Motive uptime and is
   never interpreted as Unix time.
2. The adapter brackets a NatNet echo request with its local
   `std::chrono::steady_clock`. Motive returns the QPC tick at which it received
   the request. Cristian midpoint estimation maps Motive QPC seconds into the
   adapter steady-clock domain; the lowest-RTT samples minimize network-delay
   bias and periodic echoes update the mapping.
3. For each frame, the mapped QPC value yields
   `capture_age = adapter_steady_now - mapped_capture_time`. The ROS node
   brackets that query with two `RCL_SYSTEM_TIME` reads and computes
   `header_stamp = midpoint(adapter_system_time) - capture_age`.
4. The result is therefore a ROS/Unix timestamp in the adapter host's
   `CLOCK_REALTIME` epoch, which Chrony must discipline to the deployment NTP
   source. Frames are dropped when capture age, clock-read bracket, mapping
   uncertainty, or mapping staleness exceeds the configured limits.

This method does **not** read or translate the Motive Windows wall clock. It
estimates the relation between two monotonic high-resolution clocks and only
then anchors the measured capture age to the adapter's absolute system clock.
The adapter computer and humanoid robot computer must consequently use the
same approved NTP source. For Agibot A3, the robot-side Chrony and internal PTP
implementation is documented in
[`../agibot/ntp_sync/README.md`](../agibot/ntp_sync/README.md).

The default `camera_utc` timestamp mode uses NatNet echo synchronization to
map `CameraMidExposureTimestamp` from Motive's QPC domain into the adapter
host's absolute ROS system-time epoch. The adapter host must therefore pass
the deployment Chrony/NTP qualification.

Run the adapter on a wired arena network. NatNet echo reports Motive's receive
tick but no transmit tick, so a persistent request/response path asymmetry can
bias the midpoint mapping by up to the minimum echo RTT divided by two. That
quantity is included in the mapping uncertainty, but it is still a systematic
bias and Wi-Fi can consume most or all of the 2 ms mapping budget.

Runtime echoes normally accept only samples within 0.25 ms of the measured RTT
floor. If the network establishes a permanently higher RTT floor, ten
consecutive valid higher-RTT replies (about five seconds) rebase the floor to
the best sample in the new regime. The new RTT/2 and offset correction remain
fully charged to uncertainty, so unsafe frames continue to be dropped.

The 2 ms gate covers the Motive-to-adapter clock mapping and local clock-read
bracket only. It is not the total mocap-to-A3 alignment error; adapter-host NTP
error and A3 clock-distribution error must be added separately. If each host
is qualified only to 10 ms, the conservative layered bound is 10 + 2 + 10 =
22 ms.

If Motive does not implement NatNet echo, `camera_utc` deliberately fails at
startup. `header_time:=ros` is an explicit receipt-time fallback only for uses
that do not require moving cross-sensor alignment; it is not an equivalent
timestamp mode:

```bash
ros2 launch motion_capture_tracking natnet2ros2.launch.py \
  hostname:=<MOTIVE_PC_IP> header_time:=ros
```

## Connect to HOPE

Build `hope_ws` independently. On every machine that runs the HOPE relay, the
`motion_capture_tracking_interfaces` package must be installed and sourced so
ROS 2 has local type support for `NamedPoseArray`.

Same-host example:

```bash
# Terminal 1: raw NatNet adapter
source NatNet2ROS2/install/setup.bash
ros2 launch motion_capture_tracking natnet2ros2.launch.py \
  hostname:=<MOTIVE_PC_IP>

# Terminal 2: HOPE relay and planner
source NatNet2ROS2/install/setup.bash
source hope_ws/install/setup.bash
ros2 launch hope_bringup hope_bringup.launch.py mocap_backend:=optitrack
```

When adapter and HOPE run on different computers, build/source this workspace
on the adapter host. On the HOPE host, installing only the interface package is
sufficient:

```bash
cd NatNet2ROS2
colcon build --packages-select motion_capture_tracking_interfaces
source install/setup.bash
```

Both hosts must use a compatible `ROS_DOMAIN_ID`, RMW/DDS configuration, and
network discovery setup.

Driver provenance and the local patch inventory are recorded in
`src/motion_capture_tracking/PIN.md`.

Current clock-code anchors are
`deps/libmotioncapture/include/libmotioncapture/natnet_clock_sync.h`,
`deps/libmotioncapture/src/optitrack.cpp`, and
`src/motion_capture_tracking_node.cpp`, all below
`NatNet2ROS2/src/motion_capture_tracking/`. Historical clock-plan PDFs that
show the former `hope_ws/src/...` location are archival; use these paths for
code review and line references. Before regenerating any of those PDFs, update
their code-anchor appendices to the standalone `NatNet2ROS2/` paths.
