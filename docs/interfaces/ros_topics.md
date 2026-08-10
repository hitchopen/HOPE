# ROS topics

The live control path is a short chain: mocap ball poses in, one racket command
out. The authoritative planner contract is
[PLANNER_INTERFACE.md](../PLANNER_INTERFACE.md). The raw VRPN client and HOPE
planner/relay are independent workspaces; build and launch them separately
(`use_fake_ball:=true` remains available for a mocap-less smoke test).

```text
/vrpn_mocap/<tracker>/pose_id_<N>      geometry_msgs/PoseStamped   (independent VRPN2ROS2)
        |  pose_to_posearray (hope_bringup)
        v
/poses                                 geometry_msgs/PoseArray     (Ball at index 0; P1/P2 optional)
        |  hope_planner  (or hope_planner_cpp)
        v
/racket/command                        hope_msgs/RacketCommand      (tooling/gates)
/racket/command_flat                   std_msgs/Float64MultiArray   (schema-tagged hardware wire)
        |  + /a3/base_pose_flat (hope_base_pose_flat_relay, from /P1/pose)
        v
a3_pingpong C++ runner --planner  ->  50 Hz policy control loop (body-drive iceoryx)
```

The chain above shows the default **VRPN backend**. With
`mocap_backend:=optitrack` the first two hops are replaced by the independently
built NatNet2ROS2 adapter + HOPE relay — the `/poses` hop and everything below it are identical
(see [the OptiTrack backend](#optitrack-backend) below and
[docs/OPTITRACK.md](../OPTITRACK.md)):

```text
/optitrack/poses                       motion_capture_tracking_interfaces/NamedPoseArray
        |  optitrack_mct_relay (hope_bringup)
        v
/poses (+ /ball/point, /{P1,P2}/pose, TF)
```

## Topics

| Topic | Type | From → to | QoS |
|-------|------|-----------|-----|
| `/vrpn_mocap/<tracker>/pose_id_<N>` | `geometry_msgs/PoseStamped` | vrpn_mocap client → `pose_to_posearray` | sensor-data (best-effort, volatile) |
| `/poses` | `geometry_msgs/PoseArray` | `pose_to_posearray` (or `fake_ball_publisher`) → `hope_planner` | best-effort, volatile, keep-last 1 |
| `/racket/command` | `hope_msgs/RacketCommand` | `hope_planner` → gates/tooling/MuJoCo closed loop | reliable, volatile, keep-last 10 |
| `/racket/command_flat` | `std_msgs/Float64MultiArray` (schema 2, 19 doubles) | planner → C++ runner `--planner` | reliable, volatile |
| `/a3/base_pose_flat` | `std_msgs/Float64MultiArray` (schema 2, 16 doubles) | `hope_base_pose_flat_relay` (from `/P1/pose`) → C++ runner | reliable, volatile |
| `/serve/ball_state_flat` | `std_msgs/Float64MultiArray` (≥11 doubles) | serve tooling → C++ runner | reliable, volatile |
| `/ball/pose` | `geometry_msgs/PoseStamped` | relay (valid Ball rigid-body quaternion only) → planner spin shadow (diagnostics) | best-effort, volatile |
| `/planner/diagnostics` | (see `hope_planner_cpp`) | C++ planner → operators/audit | keep-last 1 |

Flat layouts are pinned in
[`pp_planner_input.hpp`](../../a3_deploy/a3_deploy_example/src/a3/a3_deploy_onnx_ref/include/a3_pingpong/pp_planner_input.hpp)
and summarized in [PLANNER_INTERFACE.md](../PLANNER_INTERFACE.md#wire-contract): the
runner subscribes core `std_msgs` flats so the aarch64 cross-build needs no custom
message typesupport.

Notes per hop:

- **`/vrpn_mocap/<tracker>/pose_id_<N>`** — the independent
  [`VRPN2ROS2`](../../VRPN2ROS2/README.md) client publishes one
  `PoseStamped` topic per tracker. The bundled `client.launch.yaml` forces
  `multi_sensor: true`, which suffixes topics with `_id_<N>` — a tracker named
  `Ball` publishes `/vrpn_mocap/Ball/pose_id_0` (the default
  `ball_pose_topic` launch argument). The deployment config enables
  `use_vrpn_timestamps` and rejects source stamps outside a strict age/future
  bound against the adapter host's NTP-disciplined system clock. Thus ROS
  preserves the VRPN server report `timeval` rather than receipt time. VRPN
  validates every report before limiting each output topic/sensor to 200 Hz by
  default (`output_rate_hz:=0.0` disables the cap). VRPN
  does not prove which camera event a proprietary server associates with that
  value; exposure-time provenance remains a vendor/hardware acceptance item.
- **`/poses`** — `pose_to_posearray` caches the latest pose from each configured
  input topic and, whenever the trigger topic (the ball, `trigger_index` 0)
  updates, publishes a `PoseArray` whose slot *i* is input topic *i*'s latest
  pose (including its quaternion), trigger stamp passed through unmodified. The
  input order is `Ball` first (default bringup aggregates only the ball), then `P1`, `P2`
  when marker-cluster poses are aggregated; `Table` is never streamed in competition. The planner reads the ball at
  `ball_pose_index` (default 0). With `use_fake_ball:=true`,
  `fake_ball_publisher` publishes this form directly.
- **`/racket/command` + `/racket/command_flat`** — the planner feeds every mocap
  sample to its estimator but solves at bounded rate; topic names and tuning
  live in
  [`hope_ws/src/hope_planner/config/hope_planner.yaml`](../../hope_ws/src/hope_planner/config/hope_planner.yaml).
  The C++ runner's `--planner` mode subscribes the **flat** topics with matching
  reliable QoS and hands the newest command to the 50 Hz control loop; the rich
  `RacketCommand` stream feeds gates, tooling, and the MuJoCo closed loop.
  Revisions for the same flight freeze once the runner engages the swing.

## VRPN backend

```bash
# Terminal 1: raw VRPN client
cd VRPN2ROS2
colcon build --symlink-install
source install/setup.bash
ros2 launch vrpn_mocap client.launch.yaml \
  server:=<CHINGMU_SERVER_IP> port:=3883

# Terminal 2: HOPE adapter + planner
source VRPN2ROS2/install/setup.bash
source hope_ws/install/setup.bash
ros2 launch hope_bringup hope_bringup.launch.py \
  mocap_backend:=vrpn \
  ball_pose_topic:=/vrpn_mocap/Ball/pose_id_0
```

Run the timestamp acceptance probe from the VRPN workspace README before using
the stream for trajectory estimation.

## OptiTrack backend

```bash
source NatNet2ROS2/install/setup.bash
ros2 launch motion_capture_tracking natnet2ros2.launch.py \
  hostname:=<MOTIVE_PC_IP>

# In the HOPE terminal, source both installs for NamedPoseArray type support.
source NatNet2ROS2/install/setup.bash
source hope_ws/install/setup.bash
ros2 launch hope_bringup hope_bringup.launch.py mocap_backend:=optitrack
```

The two workspaces build and launch independently while keeping the `/poses`
contract byte-identical. Operational guide:
[docs/OPTITRACK.md](../OPTITRACK.md).

| Topic | Type | From → to | QoS |
|-------|------|-----------|-----|
| `/optitrack/poses` | `motion_capture_tracking_interfaces/NamedPoseArray` | independent NatNet2ROS2 driver → `optitrack_mct_relay` | sensor-data (best-effort, volatile, keep-last 1) |
| `/poses` | `geometry_msgs/PoseArray` | `optitrack_mct_relay` → `hope_planner` | best-effort, volatile, keep-last 1 |
| `/ball/point` | `geometry_msgs/PointStamped` | `optitrack_mct_relay` → (debug / downstream consumers) | best-effort, volatile, keep-last 1 |
| `/P1/pose`, `/P2/pose` | `geometry_msgs/PoseStamped` | `optitrack_mct_relay` → (debug / downstream consumers) | best-effort, volatile, keep-last 1 |

Notes:

- **`/optitrack/poses`** — at most one selected message per output period,
  carrying only the available exact-name Motive rigid bodies `Ball`, `P1`, and
  `P2`, in that order. Missing bodies are silently omitted. A selected valid
  source frame containing none is an empty-array heartbeat: continued empty
  messages mean the NatNet/adapter path is alive but the competition assets
  are absent, invalid, or misnamed; a stopped topic means transport, timestamp
  gating, or process failure. `Ball` is a strict
  6-DOF rigid body per the HOPE spec. `Table`, marker coordinates, skeletons,
  arbitrary Motive assets, raw TF, and every other raw ROS output are blocked
  by the adapter; see
  [`hope_optitrack.yaml`](../../NatNet2ROS2/src/motion_capture_tracking/config/hope_optitrack.yaml).
  ⚠ deliberately namespaced away from the bare `/poses` name by
  `natnet2ros2.launch.py`: same basename, DIFFERENT message type than the
  HOPE contract — an unremapped driver breaks the planner with a DDS type
  mismatch. The relay is the only `world → Ball/P1/P2` TF authority. The
  supplied driver uses `camera_utc`:
  NatNet echo synchronization maps `CameraMidExposureTimestamp` from Motive QPC
  into the adapter's monotonic clock, then into its Chrony-disciplined ROS
  system-time/Unix epoch. Neither bare receipt-time `ros` nor the unrelated
  Motive `camera` epoch is suitable for moving cross-sensor calibration.
- **`/poses`** — published by the relay ONLY on frames that contain the ball
  entry (an occluded ball is omitted by the driver), so a stale ball position
  is never re-emitted at rigid-body timestamps — the same
  ball-triggered publishing the VRPN path gets from `pose_to_posearray`.
  Competition order is `["ball", "P1", "P2"]` (ball first, matching the
  planner's default `ball_pose_index: 0`); absent objects are skipped.
  The source allowlist means even an accidentally active Motive Table asset
  creates no `/table/pose`, Table TF, or `/poses` entry.
- **Rates** — Both adapters validate every source report before reducing ROS
  traffic. NatNet2ROS2 caps its strict `Ball`/`P1`/`P2` named-pose array at
  200 Hz by default; VRPN2ROS2 independently caps each topic/sensor at 200 Hz.
  The planner's `fit_window` is coupled to the ROS input rate
  (`round(31 × rate / 300)`, ≥ ~100 ms of samples — 200 Hz → 21); see
  [docs/OPTITRACK.md](../OPTITRACK.md). Measured `ros2 topic hz` can read below
  the configured cap under receive-side drops; that is normal for a
  best-effort sensor stream.

## `hope_msgs/RacketCommand`

Full definition:
[`hope_ws/src/hope_msgs/msg/RacketCommand.msg`](../../hope_ws/src/hope_msgs/msg/RacketCommand.msg).
All fields are in the world frame (metres, seconds).

| Field | Type | Meaning |
|-------|------|---------|
| `header` | `std_msgs/Header` | Stamp + world frame id. |
| `position` | `geometry_msgs/Point` | Target racket position at the strike (m). |
| `velocity` | `geometry_msgs/Vector3` | Target racket velocity at the strike (m/s). |
| `normal` | `geometry_msgs/Vector3` | Desired racket face normal at contact. |
| `strike_time` | `float64` | Absolute strike wall time (s). |
| `time_to_strike` | `float64` | Seconds until the strike. |
| `ball_velocity_outgoing` | `geometry_msgs/Vector3` | Predicted outgoing ball velocity. |
| `valid` | `bool` | Command is currently actionable. |
| `clears_net` | `bool` | Predicted outgoing shot clears the net. |
| `bypasses_net_posts` | `bool` | Predicted shot passes outside the net posts. |
| `predicted_bounces` | `int32` | Incoming-trajectory bounce count. |

Flight/revision identity and `swing_sign` travel on the schema-2
`/racket/command_flat` wire (see
[PLANNER_INTERFACE.md](../PLANNER_INTERFACE.md#wire-contract)).

## QoS convention

- **Mocap side** (`/vrpn_mocap/...`, `/poses`): best-effort, volatile —
  high-rate sensor data where the latest sample is all that matters.
- **Command side** (`/racket/command`): reliable, volatile, keep-last depth 10 —
  a control setpoint that must not be dropped; the runner's subscription matches
  this profile.
