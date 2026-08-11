# Running with OptiTrack (Motive / NatNet)

HOPE supports two motion-capture backends that feed the planner the **same
`/poses` contract** (`geometry_msgs/PoseArray`, ball at index 0 — see
[interfaces/ros_topics.md](interfaces/ros_topics.md)); everything downstream is
backend-agnostic:

| Backend | Venue software | Driver → adapter |
|---|---|---|
| `vrpn` (default) | Any VRPN server (e.g. ChingMu/Avatar Pro CMTracker, VRPN TCP 3883) | independent [`VRPN2ROS2`](../VRPN2ROS2/README.md) → `pose_to_posearray` |
| `optitrack` | OptiTrack Motive (NatNet UDP, cmd port 1510) | independent [`NatNet2ROS2`](../NatNet2ROS2/README.md) → `optitrack_mct_relay` |

This document covers the `optitrack` backend. The VRPN path is independent and
documented in [interfaces/ros_topics.md](interfaces/ros_topics.md) and
[mocap/README.md](../mocap/README.md).

## How the OptiTrack path works

```text
Motive (NatNet UDP)
      |  NatNet2ROS2 workspace               (independent driver, namespace /optitrack)
      v
/optitrack/poses                             motion_capture_tracking_interfaces/NamedPoseArray
      |  optitrack_mct_relay (hope_bringup)  (rate-capped arrays, objects by name)
      v
/poses (ball at index 0), /ball/point, /P1/pose, /P2/pose, TF
      |  hope_planner (or hope_planner_cpp)
      v
/racket/command + /racket/command_flat
```

The raw driver publishes exactly `/optitrack/poses`. It is namespaced **on
purpose**: its type is `NamedPoseArray`; using the bare `/poses` name would
collide with HOPE's `geometry_msgs/PoseArray` contract. Each raw array is a
strict, case-sensitive allowlist of the available `Ball`, `P1`, and `P2`
assets in that order. Missing assets are silently omitted. The adapter exports
no marker point cloud, raw TF, `Table`, skeleton, or arbitrary Motive body; the
HOPE relay is the only `/poses` and HOPE-TF authority. If a selected,
timestamp-valid frame contains none of the three allowed assets, the adapter
publishes an empty array heartbeat. Empty arrays prove that NatNet frames and
the adapter are live while competition-body tracking is not; the relay emits
no placeholder pose or TF from them. Both the adapter and relay issue a
throttled diagnostic while this condition persists.

Record or inspect `Table` in Motive or dedicated calibration tooling during a
separate setup/training-data session. The competition adapter never exports it,
and the HOPE competition relay has no Table publisher.

The driver in the independent `NatNet2ROS2` workspace is [IMRCLab
motion_capture_tracking](https://github.com/IMRCLab/motion_capture_tracking)
pinned at v1.0.9 with its `libmotioncapture` sources materialized,
non-OptiTrack vendor SDKs removed, and NatNet unicast fixes
applied — the complete provenance and patch list is in
[`NatNet2ROS2/src/motion_capture_tracking/PIN.md`](../NatNet2ROS2/src/motion_capture_tracking/PIN.md).
It uses the **open-source NatNet depacketizer**, so it runs on any platform
(including aarch64) with no closed-source NatNet SDK.

## Build

Build and source the adapter workspace independently:

```bash
cd NatNet2ROS2
rosdep install --from-paths src --ignore-src -r -y   # Eigen, Boost, ROS 2 interfaces, ...
colcon build --symlink-install
source install/setup.bash
```

The competition adapter no longer builds the legacy unlabeled-marker tracker,
PCL, or fmt path. Message generation still needs the rosidl generator pythons
(`python3-empy`, `python3-lark`), part of a standard ROS 2 dev install.

Building with a uv/conda Python shim earlier on `PATH` can make CMake's
`FindPython3` pick a non-system interpreter, failing the
`motion_capture_tracking_interfaces` message generation with
`No module named 'em'`; in that case build with
`--cmake-args -DPython3_EXECUTABLE=/usr/bin/python3`.

Build `hope_ws` separately. It intentionally has **no build dependency** on
the NatNet driver; the OptiTrack scripts import its message lazily. A machine
that runs `optitrack_mct_relay` must nevertheless have the independent
`motion_capture_tracking_interfaces` package installed and sourced at runtime.

## Motive-side checklist

In Motive's Data Streaming pane:

| Setting | Required value | Notes |
|---------|----------------|-------|
| Enable NatNet | ✅ Enabled | This backend consumes NatNet (cmd port 1510) |
| Up Axis | **Z Axis** | Critical — aligns with the HOPE REP 103 Z-up frame; the relay applies no frame conversion |
| Transmission | Unicast preferred | Auto-negotiated from the server response; unicast keeps venue switches happy |
| Rigid Bodies | **ON** | Competition assets named exactly `Ball`, `P1`, and `P2` |
| Labeled/Unlabeled Markers | OFF (optional) | Not consumed — the ball is a rigid-body asset |
| Skeletons | OFF | Not used |
| Command/Data ports | 1510 / 1511 (defaults) | Must match firewall rules |
- Rigid Bodies **ON**; competition assets named exactly `Ball`, `P1`, and
  `P2`. The adapter passes only these exact names and orders them
  `Ball`, `P1`, `P2`; all other assets are ignored.
  Assets created/renamed while the bridge runs self-heal in ~1–2 s (the
  pinned adapter driver re-requests the model definition when an unnamed body
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
  former unlabeled-marker tracker path is not built by the competition adapter.
- Units stream in **metres** → `position_scale:=1.0` (default). Sanity check:
  `/P1/pose` reading hundreds means a millimetre feed → `0.001`.

Note: the "VRPN port 3883" Motive also exposes is its legacy VRPN broadcast —
NOT used by this backend (NatNet cmd 1510; the data port and
unicast-vs-multicast are auto-negotiated from the server response).

### Acquisition timestamps

The supplied OptiTrack config uses `topics.header_time: camera_utc`. At startup
the driver exchanges NatNet `NAT_ECHOREQUEST`/`NAT_ECHORESPONSE` packets and
selects the minimum-RTT samples to map Motive's high-resolution QPC ticks into
the adapter host's monotonic clock. For every frame it then computes:

```text
capture age = adapter monotonic now
              - map(CameraMidExposureTimestamp)
capture UTC = adapter RCL_SYSTEM_TIME now - capture age
```

Run the adapter on a **wired** arena network. NatNet's echo response supplies
Motive's request-receive tick but no server-transmit tick. Consequently,
systematic request/response path asymmetry can bias the midpoint estimate by
up to `minimum echo RTT / 2`. The implementation includes that amount in its
uncertainty rather than treating it as zero, but it remains a bias; Wi-Fi can
silently consume most of the 2 ms mapping budget and is not a qualified
competition path.

The runtime filter rejects isolated echoes more than 0.25 ms above its RTT
floor. To recover from a permanent route/link regime change, ten consecutive
valid higher-RTT responses (approximately five seconds at the 500 ms refresh
period) rebase the floor to the lowest RTT in that run. Rebase does not weaken
the safety gate: the new RTT/2 and the complete pre-filter offset correction
are charged to mapping uncertainty, and publication remains stopped if the
result exceeds the configured limit.

`RCL_SYSTEM_TIME` is the Unix epoch supplied by `CLOCK_REALTIME` and must pass
the deployment Chrony qualification. `max_clock_sync_uncertainty_ms` and
`max_capture_age_ms` reject unsafe frames. The old
`ros_latency_compensated`/`network_latency_ms` path remains only as an explicit
legacy fallback; it cannot measure one-way network delay. Do not use
`header_time: ros` for moving cross-sensor calibration: a fixed 5 ms path delay
becomes 2.5 mm of invisible position bias at 0.5 m/s. Do not substitute
`header_time: camera`: it preserves the Motive interval but uses the Motive
host's unrelated high-resolution-clock epoch. The pelvis reference producer
must independently map its own acquisition time into the same ROS epoch.

`max_clock_sync_uncertainty_ms: 2.0` is **not** the complete mocap-to-A3
alignment number. It covers the Motive↔adapter mapping plus the local
system-clock read bracket. The adapter-host NTP error and the A3 UTC/PTP error
are separate terms. For example, if those two deployment gates each allow
10 ms, the conservative layered bound is `10 + 2 + 10 = 22 ms`, not 2 ms.

If an older Motive does not answer NatNet echo, `camera_utc` fails startup by
design. An operator may explicitly launch with `header_time:=ros` only for a
diagnostic or application that does not require moving cross-sensor alignment;
receipt time is not a degraded-but-equivalent form of acquisition time.

This implements the same timing principle as OptiTrack's documented
`NatNetClient::SecondsSinceHostTimestamp(CameraMidExposureTimestamp)`; the
open-source backend implements the echo wire protocol directly so it also
works on the A3 adapter's aarch64 platform.

### Optional marker-CAD calibration: P1 to an A3 `pelvis_link`

The integrated Foxglove console can use all ten waist markers to compute this
alignment without an independent pelvis tracker. Its `Calibration` button calls
`/hope/calibrate`: while the authoritative Runner remains in fresh `PD_STAND`,
the Laptop recomputes the fixed `P1 -> pelvis_link` registration, composes the
current stationary `world -> P1` pose into a `world -> pelvis_link` audit
snapshot, and atomically stores both in `calibration/p1_to_pelvis.json`. The
separate `Refresh x_hit` button calls `/hope/refresh_x_hit`; it does not rerun
the marker calibration.

When an approved setup procedure calls for this transform, first put the MDU
Runner in settled PD_STAND, then run `p1_marker_cad_calibrator` manually on
`/optitrack/rigid_body_markers`.
Motive supplies
the P1-local ModelDef marker centres plus live labeled-marker samples; the tool
registers those centres to the A3 v2 hip-shell CAD (`f1`–`f5`, `b1`–`b5`) and
checks every selected marker's live residual and definition stability.

The non-collinear named 3-D marker layout observes all six degrees of the fixed
`P1 → pelvis_link` transform even while the robot is stationary in PD_STAND.
An approved result atomically replaces `calibration/p1_to_pelvis.json`,
relative to the external computer's HOPE repository root (for example,
`/home/user/HOPE/calibration/p1_to_pelvis.json`). A failed fit cannot install a
current-run calibration. Run the calculation only after PD_STAND has already
been established by the approved robot procedure:

```bash
ros2 run hope_bringup p1_marker_cad_calibrator \
  --topic /optitrack/rigid_body_markers \
  --asset-name P1 \
  --marker-names f1,f2,f3,f4,f5,b1,b2,b3,b4,b5 \
  --minimum-frames 200 \
  --capture-duration 4 \
  --stationary-prepare \
  --attest-installed-layout \
  --allow-nominal-only-markers \
  --output calibration/p1_to_pelvis.json
```

After the replacement, the computer-side `hope_base_pose_flat_relay` reads the
canonical `p1_to_pelvis` object from that JSON and composes it with the live
`world → P1` pose. The additional `world_to_pelvis_snapshot` object records
the stationary calibration instant for audit only; it is not published as a
static transform after the robot moves. The computer publishes
`/a3/base_pose_flat` for policy localization and the unshifted diagnostic
`/a3/mocap/pelvis_pose`. It does not recalculate while the robot is playing.
The robot consumes `/a3/base_pose_flat`; it never stores, reads, or receives
the JSON. The SHA-derived PREPARE receipt gate described by the imported
adapter is not part of the native Runner admission contract.

#### Legacy independent pose-pair method

The following older method is retained for a genuinely independent external
6-DOF reference or a simulation test. Do this only in a
**setup/calibration session**. The calibrator does not require
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

No checked-in real-robot node publishes `/a3/calibration/pelvis_pose`. It is an
input to this legacy tool, not the output of the ten-marker calculation. Never
feed `/a3/mocap/pelvis_pose` or any other P1-derived result into it, because
that would make the measurement circular.

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

The production marker/CAD route described at the top of this section supersedes
the old checked-in P1 transform in `hope_world_frame.yaml`. The runtime relay
now reads the approved JSON directly. Use exactly one route per calibration
receipt — marker/CAD registration or the independent pose-pair method — never
stack both corrections (see [interfaces/frames.md](interfaces/frames.md)).

## Bringup

### Preflight (before launch, no ROS required)

```bash
ros2 run hope_bringup natnet_preflight.py --hostname <MOTIVE_PC_IP>
# or directly: ./hope_ws/src/hope_bringup/scripts/natnet_preflight.py --hostname <MOTIVE_PC_IP>
```

`natnet_preflight.py` speaks the NatNet command protocol itself and separates
the failure modes that all look identical from the ROS side (every HOPE topic
silent): Motive unreachable / streaming disabled, Motive ignoring the
model-definition request (see the adapter driver's PIN.md patch #9 — on
Motive 3.1 / NatNet 4.1 this used to hang the driver's constructor before it
created any publisher), NatNet echo clock synchronization unavailable, and
frames not reaching this host (wrong interface, multicast routed out a VPN
tunnel, firewall). It reports the minimum echo RTT / midpoint uncertainty,
verifies the `Ball` and `P1` assets in the model definition, and gates on the
measured frame rate (`--min-hz`, default 250). Exit code 0 means the bridge
should come up.

### Launch

Start the raw adapter from its own workspace:

```bash
source NatNet2ROS2/install/setup.bash
ros2 launch motion_capture_tracking natnet2ros2.launch.py \
  hostname:=<MOTIVE_PC_IP>
```

Then start the independently built HOPE relay and planner. Source the adapter
first so this host has `NamedPoseArray` type support:

```bash
source NatNet2ROS2/install/setup.bash
source hope_ws/install/setup.bash
ros2 launch hope_bringup hope_bringup.launch.py mocap_backend:=optitrack
```

`hostname` is a REQUIRED argument with no default — venue values are passed
explicitly to the adapter, never baked in. Driver/timestamp config:
[`hope_optitrack.yaml`](../NatNet2ROS2/src/motion_capture_tracking/config/hope_optitrack.yaml).
Relay config (name → topic mapping, scale):
[`config/optitrack_relay.yaml`](../hope_ws/src/hope_bringup/config/optitrack_relay.yaml).

### Verify

```bash
ros2 topic echo --once /optitrack/poses   # raw driver frames (names must match config)
ros2 topic echo --once /poses             # HOPE contract (ball at index 0)
ros2 run hope_bringup mocap_rate_probe.py --topic /P1/pose --min-hz 150
```

`mocap_rate_probe.py` is a one-shot pass/fail rate gate (NatNet is UDP — unlike
the VRPN TCP port there is nothing to `connect()` to before launch, so mocap
liveness can only be proven by data). NatNet2ROS2 receives every source frame
but caps its filtered ROS pose output at 200 Hz by default. Change the adapter launch
argument `output_rate_hz:=<Hz>`, or use `0.0` for every valid source frame.
Counting published messages can read below that cap under receive-side drops;
that is normal for a best-effort sensor stream.

Because `/optitrack/poses` carries empty heartbeats, its message rate proves
NatNet/adapter/timestamp-path liveness, not competition-body readiness. Inspect
the array contents or gate on downstream `/P1/pose` as shown above. The managed
`run_rally_v10_hdu.sh` startup uses `/P1/pose`, so empty heartbeats cannot pass
its robot-body qualification.

## No-hardware smoke test

`fake_optitrack_publisher` replaces Motive + driver with a synthetic
`/optitrack/poses` feed (alternating serves; the ball entry is omitted between
serves, exercising the relay's `/poses` gating):

```bash
ros2 run hope_bringup fake_optitrack_publisher
ros2 launch hope_bringup optitrack_mct_relay.launch.py   # relay under test
ros2 topic echo /poses
```

A driver-level no-hardware test also exists: launch `natnet2ros2.launch.py`
with `mocap_type:=mock header_time:=ros`; this runs the real driver code with a
static mock backend instead of NatNet. Caveat: mock streams only bodies defined in a
`rigid_bodies` block, which the shipped config deliberately has none of (the
ball is a Motive rigid-body asset, not a tracker body) — with the default
config mock emits empty frames. For bench work just use
`fake_optitrack_publisher` above, or add a throwaway `rigid_bodies` entry
locally (mock uses only each body's `initial_position`).

For bag replay, record `/optitrack/poses` at a live session
(`ros2 bag record /optitrack/poses`) and replay it against
`optitrack_mct_relay.launch.py`; the raw adapter stays stopped during replay.

## Source/output rate and the planner's `fit_window`

The camera rate and ROS output rate are now intentionally distinct. OptiTrack
rigs commonly capture at 360 Hz, while NatNet2ROS2 receives every frame and
publishes at a configurable maximum of 200 Hz by default. The planner's
velocity fit uses `fit_window` **received samples**, so couple it to the ROS
output rate, not the Motive camera rate: retain about 100 ms with
`round(31 × output_rate / 300)`. At the 200 Hz default use `fit_window: 21`; if
downsampling is disabled on a 360 Hz source, use 37. Configure it in
[`hope_planner.yaml`](../hope_ws/src/hope_planner/config/hope_planner.yaml), or
pass `planner_fit_window:=<samples>` to `hope_bringup.launch.py`. That launch
selects 21 for either adapter's default 200 Hz output.
Read the camera rate from Motive and the output rate from adapter configuration;
`ros2 topic hz` is only a receive-side verification of the latter.

## Multi-machine DDS (laptop bridge topology)

A common venue topology puts the NatNet driver on a laptop that bridges the
Motive LAN to the robot's network. Where DDS multicast discovery does not work
(venue Wi-Fi, segmented LANs), wrap each side's command with
`with_fastdds_unicast.sh` to use explicit unicast peers:

```bash
# Laptop (runs the independent raw adapter; peers with the robot host):
./hope_ws/src/hope_bringup/scripts/with_fastdds_unicast.sh --peer <ROBOT_HOST_IP> -- \
  ros2 launch motion_capture_tracking natnet2ros2.launch.py \
    hostname:=<MOTIVE_PC_IP>

# Robot host (sources the interface package, then runs HOPE relay + planner):
./hope_ws/src/hope_bringup/scripts/with_fastdds_unicast.sh --peer <LAPTOP_IP> -- \
  ros2 launch hope_bringup hope_bringup.launch.py mocap_backend:=optitrack ...
```

The wrapper generates a Fast DDS profile (unicast-only transport, interface
whitelist derived from the route to each peer) and sets
`ROS_STATIC_PEERS`/`RMW_IMPLEMENTATION` for the wrapped command only.

## Troubleshooting

| Symptom | Cause / fix |
|---|---|
| Driver starts but 0 Hz on `/optitrack/poses` | Wrong `hostname`, firewall on UDP 1510/1511, not on the Motive LAN, or all frames are failing the configured timestamp gates. Inspect the adapter log, `ping` the Motive PC, then run `natnet_preflight.py --hostname <MOTIVE_PC_IP>` to pinpoint the failing stage. |
| `/optitrack/poses` exists with `Publisher count: 0`, nothing logged | Pre-patch-#9 driver hung in its constructor: Motive 3.1 / NatNet 4.1 silently drops payload-less model-definition requests. Fixed in the pinned adapter driver (type-mask request + bounded handshake — it now retries and then exits with an error instead of hanging); `natnet_preflight.py` reports this Motive behavior explicitly. |
| `/optitrack/poses` continues near 200 Hz with `poses: []` | NatNet transport and timestamp gates are live, but no valid exact-name `Ball`, `P1`, or `P2` asset is present in the selected frames. Check Motive tracking/streaming and case-sensitive asset names. The relay intentionally emits no downstream pose or TF. |
| Objects stream but nothing relayed | Motive asset names don't match the exact adapter allowlist and `optitrack_relay.yaml` (`Ball`/`P1`/`P2`, case-sensitive), or only empty heartbeats are arriving. Check `ros2 topic echo --once /optitrack/poses`. |
| Rigid bodies stream with empty names | Fixed by vendored patch #6 (self-heals in ~1–2 s); if persistent, restart the bridge. |
| `/P1/pose` positions in the hundreds | Millimetre feed → `position_scale:=0.001`. |
| `/poses` pauses while `/P1/pose` keeps updating | By design: the ball left the volume / lost tracking; the relay never re-emits a stale ball (protects the planner's velocity fit). |
| `optitrack_mct_relay` exits with an import error | `motion_capture_tracking_interfaces` from NatNet2ROS2 is not installed/sourced on the HOPE host. Source that workspace (or install the interface package), or use the VRPN backend. |
| Planner predictions lag/noisy after changing output rate | Scale `fit_window` with the NatNet2ROS2 ROS output rate (see above). |
