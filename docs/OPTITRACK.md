# Running with OptiTrack (Motive / NatNet)

HOPE supports two motion-capture backends that feed the planner the **same
`/poses` contract** (`geometry_msgs/PoseArray`, ball at index 0 — see
[interfaces/ros_topics.md](interfaces/ros_topics.md)); everything downstream is
backend-agnostic:

| Backend | Venue software | Driver → adapter |
|---|---|---|
| `vrpn` (default) | Any VRPN server (e.g. ChingMu/Avatar Pro CMTracker, VRPN TCP 3883) | vendored [`vrpn_mocap`](../hope_ws/src/vrpn_mocap/README.md) → `pose_to_posearray` |
| `optitrack` | OptiTrack Motive (NatNet UDP, cmd port 1510) | vendored [`motion_capture_tracking`](../hope_ws/src/motion_capture_tracking/PIN.md) → `optitrack_mct_relay` |

This document covers the `optitrack` backend. The VRPN path is unchanged and
documented in [interfaces/ros_topics.md](interfaces/ros_topics.md) and
[mocap/README.md](../mocap/README.md).

## How the OptiTrack path works

```text
Motive (NatNet UDP)
      |  motion_capture_tracking_node        (vendored driver, namespace /optitrack)
      v
/optitrack/poses                             motion_capture_tracking_interfaces/NamedPoseArray
      |  optitrack_mct_relay (hope_bringup)  (one message per camera frame, objects by name)
      v
/poses (ball at index 0), /ball/point, /P1/pose, /P2/pose, TF
      |  hope_planner
      v
/racket/command
```

The driver's raw topics stay under `/optitrack/*` **on purpose**: its `poses`
topic is a `NamedPoseArray` — on the bare `/poses` name it would collide with
the HOPE `/poses` contract (`geometry_msgs/PoseArray`) as a DDS type mismatch —
and its raw `/tf` (body names verbatim) would fight the relay's transforms.
`optitrack_hope_bridge.launch.py` enforces both remaps; the relay is the only
`/poses`/TF authority.

The relay parameter `publish_table` defaults to `false`. Therefore the normal
competition launch creates no `/table/pose` publisher and emits no Table TF or
Table entry in `/poses`, even if the Motive asset was accidentally left active.
For a separate arena setup or data-recording session only, the standalone
bridge may be launched with `publish_table:=true`; stop it and return to the
default before competition.

The vendored driver is [IMRCLab
motion_capture_tracking](https://github.com/IMRCLab/motion_capture_tracking)
pinned at v1.0.9 with its `libmotioncapture`/`librigidbodytracker` submodules
materialized, non-OptiTrack vendor SDKs removed, and NatNet unicast fixes
applied — the complete provenance and patch list is in
[`hope_ws/src/motion_capture_tracking/PIN.md`](../hope_ws/src/motion_capture_tracking/PIN.md).
It uses the **open-source NatNet depacketizer**, so it runs on any platform
(including aarch64) with no closed-source NatNet SDK.

## Build

The two vendored packages build with the workspace:

```bash
cd hope_ws
rosdep install --from-paths src --ignore-src -r -y   # PCL, Eigen, fmt, Boost, ...
colcon build
source install/setup.bash
```

Additional system dependencies beyond the VRPN-only build: the vendored
manifests declare PCL and Eigen, so the `rosdep install` line resolves those.
Two build requirements are **not** declared upstream — `libfmt-dev` (the
driver package's own CMake does `find_package(fmt REQUIRED)`) and
`libboost-program-options-dev` (a `librigidbodytracker` CMake requirement) —
install them via apt if the build reports a missing `fmt` or
`Boost::program_options`. Message generation also needs the rosidl generator
pythons (`python3-empy`, `python3-lark`), part of a standard ROS 2 dev
install.

Building with a uv/conda Python shim earlier on `PATH` can make CMake's
`FindPython3` pick a non-system interpreter, failing the
`motion_capture_tracking_interfaces` message generation with
`No module named 'em'`; in that case build with
`--cmake-args -DPython3_EXECUTABLE=/usr/bin/python3`.

VRPN-only builds stay possible: `hope_bringup` intentionally declares **no**
manifest dependency on the vendored driver (the OptiTrack scripts import its
message lazily and fail with an actionable error), so
`colcon build --packages-skip motion_capture_tracking motion_capture_tracking_interfaces`
builds everything else unchanged.

## Motive-side checklist

In Motive's Data Streaming pane:

| Setting | Required value | Notes |
|---------|----------------|-------|
| Enable NatNet | ✅ Enabled | This backend consumes NatNet (cmd port 1510) |
| Up Axis | **Z Axis** | Critical — aligns with the HOPE REP 103 Z-up frame; the relay applies no frame conversion |
| Transmission | Unicast preferred | Auto-negotiated from the server response; unicast keeps venue switches happy |
| Rigid Bodies | **ON** | `Ball`, `P1` (+ `P2`; `Table` in calibration sessions only) |
| Labeled/Unlabeled Markers | OFF (optional) | Not consumed — the ball is a rigid-body asset |
| Skeletons | OFF | Not used |
| Command/Data ports | 1510 / 1511 (defaults) | Must match firewall rules |
- Rigid Bodies **ON**; assets named exactly `P1` (+ `Table`, `P2` if used —
  the table asset is setup/calibration-only; older notes call it `PPT`) — the
  driver streams Motive asset names verbatim and the relay maps by name.
  Assets created/renamed while the bridge runs self-heal in ~1–2 s (the
  vendored driver re-requests the model definition when an unnamed body
  streams, PIN.md patch #6); restart the bridge only as a fallback.
- **Ball — strict 6-DOF rigid-body asset (the only supported mode):** the
  HOPE standard requires the ball to be tracked as a strict 6-DOF rigid body
  ([mocap reference §1 minimum spec / §5.1](../mocap/HOPE_Motion_Capture_System_and_Coordinates_Reference_Setup.md)).
  The verified preparation is **retroreflective marker dots added to a
  standard ping-pong ball** — confirmed workable on both OptiTrack and
  Chingmu. In Motive, define the ball as a rigid-body asset named exactly
  `Ball` and set the asset **pivot to the sphere center** (the planner's
  bounce geometry assumes ball-center positions). Occlusion clears the
  asset's tracking-valid bit and the driver drops it from the frame, so
  `/poses` pauses exactly like the VRPN path; validate high-speed tracking
  and re-acquisition per the mocap reference §5.4 acceptance checks.
  Single-marker / unlabeled-point ball tracking (the retired ≤ v0.4 design)
  does **not** meet the spec and is not supported by this bringup — the
  `librigidbodytracker` machinery in the vendored driver is deliberately left
  unconfigured.
- Units stream in **metres** → `position_scale:=1.0` (default). Sanity check:
  `/P1/pose` reading hundreds means a millimetre feed → `0.001`.

Note: the "VRPN port 3883" Motive also exposes is its legacy VRPN broadcast —
NOT used by this backend (NatNet cmd 1510; the data port and
unicast-vs-multicast are auto-negotiated from the server response).

### Acquisition timestamps

The supplied OptiTrack config uses
`topics.header_time: ros_latency_compensated`. For every frame the driver
computes a local-ROS-epoch exposure estimate:

```text
ROS receipt time
  - NatNet Camera latency
  - NatNet Motive processing latency
  - measured one-way network/host latency
```

Set `topics.network_latency_ms` from the deployed isolated LAN measurement;
half of a demonstrably symmetric RTT is only an initial estimate, while packet
capture or hardware timestamping is preferable. Do not use `header_time: ros`
for moving cross-sensor calibration: a fixed 5 ms path delay becomes 2.5 mm
of invisible position bias at 0.5 m/s. Do not substitute
`header_time: camera`: it preserves the Motive interval but uses the Motive
host's unrelated high-resolution-clock epoch. The pelvis reference producer
must independently map its own acquisition time into the same ROS epoch.

### Calibrating P1 to an A3 `pelvis_link`

Do this only in a **setup/calibration session**. The calibrator does not require
the `Table` rigid body; `Table` remains disabled in competition. The normal
deployment keeps Motive's dynamic `world → P1` rigid-body pose and adds the
calibrated constant `P1 → pelvis_link` static TF at ROS 2 bringup.
Introducing `Table` as a common intermediate would cancel exactly in every
relative-pose sample and add a setup/competition asset difference without
adding calibration information.

The tool below observes the raw `/P1/pose` and an **independent, real-time**
full-6DOF pelvis `PoseStamped` at matching timestamps. Both message headers
must name the same reference frame (`world` by default), and both poses may
change while A3 moves. The pelvis message must not be derived from `/P1/pose`
or any descendant of it, or the calibration would be circular. In particular,
do not run `p1_pelvis_tf_publisher` during calibration. At each synchronized
pair the tool estimates the same constant transform and then robustly averages
it:

```text
^P1 T_pelvis_link = (^world T_P1)^-1 · ^world T_pelvis_link
```

There is deliberately no default pelvis topic. The checked-in A3 hardware
bridge publishes `/body_drive/pelvis_imu/data`, but that IMU has no absolute
translation and is not a full-pose producer. For a real A3, first bridge an
independent external 6-DOF tracker or state estimator to a topic such as
`/a3/calibration/pelvis_pose`, using `geometry_msgs/PoseStamped`,
`header.frame_id: world`, and the same clock domain as `/P1/pose`. The existing
`/sim/a3/pelvis_pose` producer is MuJoCo-only and uses `odom`; it is valid for a
simulation check only if the P1 input is also expressed in `odom`.

Build and source the workspace, start both pose producers, then run:

```bash
ros2 run hope_bringup p1_pelvis_calibrator \
  --p1-topic /P1/pose \
  --pelvis-topic /a3/calibration/pelvis_pose \
  --reference-frame world \
  --pelvis-frame pelvis_link \
  --p1-frame P1 \
  --output calibration/p1_to_pelvis.json
```

It verifies that both topics have publishers, collects 200 synchronized samples
by default and accepts pairs within 2 ms. It rejects skewed/outlier pairs and
fails if either header frame is wrong or if the accepted residual exceeds
3 mm RMS or 1 degree RMS. Residual RMS measures consistency only, so a separate
excitation gate requires both accepted trajectories to cover at least 0.10 m,
10 degrees, and 1 second, with strictly increasing and at least 90% unique
timestamps and at least a 50 Hz accepted rate. A stationary capture therefore
cannot pass merely because every repeated sample agrees.
The JSON records measured pair skew, timestamp coverage, and motion spans; it
does not claim to verify source independence, which cannot be inferred from
two pose topics. A missing pelvis producer fails after the discovery timeout
instead of being counted as 200 TF misses. Writing the JSON record is required.
It also contains the constant `p1_to_pelvis` transform, residual quality
metrics, optional Motive-pivot registration values, and the CAD cross-check.
Load it during normal bringup:

```bash
ros2 run hope_bringup p1_pelvis_tf_publisher \
  --calibration-file calibration/p1_to_pelvis.json
```

This creates `P1 → pelvis_link`; together with the live NatNet result, the TF
chain is `world → P1 → pelvis_link`. Check it with:

```bash
ros2 run tf2_ros tf2_echo world P1
ros2 run tf2_ros tf2_echo P1 pelvis_link
```

The first transform must move with the robot; the second must stay constant.
As an optional alternative, absorb the correction into Motive: rotate the P1
pivot axes to the reported `pelvis_link` axes, enter the reported local
Translation Offset, save the asset profile, restart streaming, and rerun the
calibrator. The measured correction should then be approximately identity. In
that configuration, do not run `p1_pelvis_tf_publisher`; doing so would apply
the correction twice.

The v2 CAD table and the current
`a3_hip_marker_shell_p1_mocap_balls_0702.x_t` shell define all ten markers
(`f1`–`f5`, `b1`–`b5`), and a physical mocap experiment confirmed that all ten
points are visible. The default tool configuration therefore uses the complete
ten-marker set, whose centroid is `[-0.0024, 0, -0.1490] m` in `pelvis_link`.
If its axes are already aligned, the current-shell CAD cross-check is a Motive
pivot translation of `[+2.4, 0, +149.0] mm`. The live calibration result
remains authoritative because it captures the installed marker plate and its
actual orientation.
Marker stream order and per-marker topic names do not affect this tool: it
consumes the solved 6-DOF `/P1/pose`, while its CAD centroid calculation is
order-independent. Only an offline reconstruction directly from individual
marker coordinates would require a verified marker-ID-to-CAD correspondence.

## Bringup

### Preflight (before launch, no ROS required)

```bash
ros2 run hope_bringup natnet_preflight.py --hostname <MOTIVE_PC_IP>
# or directly: ./hope_ws/src/hope_bringup/scripts/natnet_preflight.py --hostname <MOTIVE_PC_IP>
```

`natnet_preflight.py` speaks the NatNet command protocol itself and separates
the failure modes that all look identical from the ROS side (every HOPE topic
silent): Motive unreachable / streaming disabled, Motive ignoring the
model-definition request (see the vendored driver's PIN.md patch #9 — on
Motive 3.1 / NatNet 4.1 this used to hang the driver's constructor before it
created any publisher), and frames not reaching this host (wrong interface,
multicast routed out a VPN tunnel, firewall). It also verifies the `Ball` and
`P1` assets are in the model definition and gates on the measured frame rate
(`--min-hz`, default 250). Exit code 0 means the bridge should come up.

### Launch

One command for mocap + planner (`mocap_server` = the Motive PC IP):

```bash
ros2 launch hope_bringup hope_bringup.launch.py \
  mocap_backend:=optitrack \
  mocap_server:=<MOTIVE_PC_IP> \
  mocap_network_latency_ms:=<MEASURED_ONE_WAY_MS>
```

Or start the mocap side alone (also publishes the static HOPE world frame):

```bash
ros2 launch hope_bringup optitrack_hope_bridge.launch.py \
  hostname:=<MOTIVE_PC_IP> \
  position_scale:=1.0 \
  network_latency_ms:=<MEASURED_ONE_WAY_MS>
```

`hostname` is a REQUIRED argument with no default — venue values are passed
explicitly, never baked in. Driver config (ball tracker modes, body names):
[`config/optitrack_mct.yaml`](../hope_ws/src/hope_bringup/config/optitrack_mct.yaml).
Relay config (name → topic mapping, scale):
[`config/optitrack_relay.yaml`](../hope_ws/src/hope_bringup/config/optitrack_relay.yaml).

### Verify

```bash
ros2 topic echo --once /optitrack/poses   # raw driver frames (names must match config)
ros2 topic echo --once /poses             # HOPE contract (ball at index 0)
ros2 run hope_bringup mocap_rate_probe.py --topic /P1/pose --min-hz 180
```

`mocap_rate_probe.py` is a one-shot pass/fail rate gate (NatNet is UDP — unlike
the VRPN TCP port there is nothing to `connect()` to before launch, so mocap
liveness can only be proven by data). Counting published messages can read
lower than the camera rate under receive-side drops; that is normal for a
best-effort sensor stream.

## No-hardware smoke test

`fake_optitrack_publisher` replaces Motive + driver with a synthetic
`/optitrack/poses` feed (alternating serves; the ball entry is omitted between
serves, exercising the relay's `/poses` gating):

```bash
ros2 run hope_bringup fake_optitrack_publisher
ros2 launch hope_bringup optitrack_mct_relay.launch.py   # relay under test
ros2 topic echo /poses
```

A driver-level no-hardware test also exists: `mocap_type:=mock` on
`optitrack_hope_bridge.launch.py` runs the real driver code with a static mock
backend instead of NatNet. Caveat: mock streams only bodies defined in a
`rigid_bodies` block, which the shipped config deliberately has none of (the
ball is a Motive rigid-body asset, not a tracker body) — with the default
config mock emits empty frames. For bench work just use
`fake_optitrack_publisher` above, or add a throwaway `rigid_bodies` entry
locally (mock uses only each body's `initial_position`).

For bag replay, record `/optitrack/poses` at a live session
(`ros2 bag record /optitrack/poses`) and replay it against
`optitrack_mct_relay.launch.py` (`start_mocap_node:=false` on the full bridge).

## Camera rate and the planner's `fit_window`

The planner's velocity fit uses `fit_window` **samples** (default 31 ≈ 103 ms
at a 300 Hz rig). The window is rate-coupled: keep it at ≥ ~100 ms of samples,
i.e. `round(31 × rate / 300)`. OptiTrack rigs commonly stream **360 Hz** →
set `fit_window: 37` in
[`hope_planner.yaml`](../hope_ws/src/hope_planner/config/hope_planner.yaml)
(or pass `-p fit_window:=37`). The camera rate is a venue fact — read it from
Motive, don't infer it from `ros2 topic hz` (receive-side drops read low).

## Multi-machine DDS (laptop bridge topology)

A common venue topology puts the NatNet driver on a laptop that bridges the
Motive LAN to the robot's network. Where DDS multicast discovery does not work
(venue Wi-Fi, segmented LANs), wrap each side's command with
`with_fastdds_unicast.sh` to use explicit unicast peers:

```bash
# Laptop (runs the bridge; peers with the robot host):
./hope_ws/src/hope_bringup/scripts/with_fastdds_unicast.sh --peer <ROBOT_HOST_IP> -- \
  ros2 launch hope_bringup optitrack_hope_bridge.launch.py \
    hostname:=<MOTIVE_PC_IP> \
    network_latency_ms:=<MEASURED_ONE_WAY_MS>

# Robot host (peers with the laptop):
./hope_ws/src/hope_bringup/scripts/with_fastdds_unicast.sh --peer <LAPTOP_IP> -- \
  ros2 launch hope_bringup hope_bringup.launch.py mocap_backend:=optitrack ...
```

The wrapper generates a Fast DDS profile (unicast-only transport, interface
whitelist derived from the route to each peer) and sets
`ROS_STATIC_PEERS`/`RMW_IMPLEMENTATION` for the wrapped command only.

## Troubleshooting

| Symptom | Cause / fix |
|---|---|
| Driver starts but 0 Hz on `/optitrack/poses` | Wrong `hostname`, firewall on UDP 1510/1511, or not on the Motive LAN. `ping` the Motive PC first. Run `natnet_preflight.py --hostname <MOTIVE_PC_IP>` to pinpoint the failing stage. |
| `/optitrack/poses` exists with `Publisher count: 0`, nothing logged | Pre-patch-#9 driver hung in its constructor: Motive 3.1 / NatNet 4.1 silently drops payload-less model-definition requests. Fixed in the vendored driver (type-mask request + bounded handshake — it now retries and then exits with an error instead of hanging); `natnet_preflight.py` reports this Motive behavior explicitly. |
| Objects stream but nothing relayed | Motive asset names don't match `optitrack_relay.yaml` (`P1`/`P2`/`Table`/`Ball`, case-sensitive). Check `ros2 topic echo --once /optitrack/poses`. |
| Rigid bodies stream with empty names | Fixed by vendored patch #6 (self-heals in ~1–2 s); if persistent, restart the bridge. |
| `/P1/pose` positions in the hundreds | Millimetre feed → `position_scale:=0.001`. |
| `/poses` pauses while `/P1/pose` keeps updating | By design: the ball left the volume / lost tracking; the relay never re-emits a stale ball (protects the planner's velocity fit). |
| `optitrack_mct_relay` exits with an import error | The vendored interfaces package isn't built/sourced — build the workspace, or use the VRPN backend. |
| Planner predictions lag/noisy at 360 Hz | Scale `fit_window` with the camera rate (see above). |
