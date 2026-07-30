# ROS topics

The live control path is a short chain: mocap ball poses in, one racket command
out. The authoritative planner contract is
[PLANNER_INTERFACE.md](../PLANNER_INTERFACE.md); bring-up is
`cd hope_ws && colcon build && source install/setup.bash`, then
`ros2 launch hope_bringup hope_bringup.launch.py` (`use_fake_ball:=true` for a
mocap-less smoke test).

```text
/vrpn_mocap/<tracker>/pose_id_<N>      geometry_msgs/PoseStamped   (vendored vrpn_mocap client)
        |  pose_to_posearray (hope_bringup)
        v
/poses                                 geometry_msgs/PoseArray     (Ball at index 0; P1/P2 optional)
        |  hope_planner
        v
/racket/command                        hope_msgs/RacketCommand
        |  python -m a3_deploy_onnx_ref_pingpong --planner
        v
50 Hz policy control loop
```

The chain above shows the default **VRPN backend**. With
`mocap_backend:=optitrack` the first two hops are replaced by the vendored
NatNet driver + relay — the `/poses` hop and everything below it are identical
(see [the OptiTrack backend](#optitrack-backend) below and
[docs/OPTITRACK.md](../OPTITRACK.md)):

```text
/optitrack/poses                       motion_capture_tracking_interfaces/NamedPoseArray
        |  optitrack_mct_relay (hope_bringup)
        v
/poses (+ /ball/point, /{P1,P2}/pose, TF; Table is setup-only opt-in)
```

## Topics

| Topic | Type | From → to | QoS |
|-------|------|-----------|-----|
| `/vrpn_mocap/<tracker>/pose_id_<N>` | `geometry_msgs/PoseStamped` | vrpn_mocap client → `pose_to_posearray` | sensor-data (best-effort, volatile) |
| `/poses` | `geometry_msgs/PoseArray` | `pose_to_posearray` (or `fake_ball_publisher`) → `hope_planner` | best-effort, volatile, keep-last 1 |
| `/racket/command` | `hope_msgs/RacketCommand` | `hope_planner` → runner `--planner` | reliable, volatile, keep-last 10 |

Notes per hop:

- **`/vrpn_mocap/<tracker>/pose_id_<N>`** — the vendored
  [`vrpn_mocap`](../../hope_ws/src/vrpn_mocap/README.md) client publishes one
  `PoseStamped` topic per tracker. The bundled `client.launch.yaml` forces
  `multi_sensor: true`, which suffixes topics with `_id_<N>` — a tracker named
  `ball` publishes `/vrpn_mocap/ball/pose_id_0` (the default `ball_pose_topic`
  launch argument). Header stamps are ROS-side **receipt time** unless the
  driver's `use_vrpn_timestamps` parameter is enabled, so network jitter is
  present in the stamps the planner's velocity fit consumes.
- **`/poses`** — `pose_to_posearray` caches the latest pose from each configured
  input topic and, whenever the trigger topic (the ball, `trigger_index` 0)
  updates, publishes a `PoseArray` whose slot *i* is input topic *i*'s latest
  pose (including its quaternion), trigger stamp passed through unmodified. The
  input order is `Ball` first (default bringup aggregates only the ball), then `P1`, `P2`
  when marker-cluster poses are aggregated; `Table` is never streamed in competition. The planner reads the ball at
  `ball_pose_index` (default 0). With `use_fake_ball:=true`,
  `fake_ball_publisher` publishes this form directly.
- **`/racket/command`** — the planner feeds every mocap sample to its estimator
  but solves at most every `solve_period_s` (≤ 50 Hz); topic names and tuning
  live in
  [`hope_ws/src/hope_planner/config/hope_planner.yaml`](../../hope_ws/src/hope_planner/config/hope_planner.yaml).
  The reference runner's `--planner` mode subscribes with the matching reliable
  QoS and hands the newest command to the 50 Hz control loop.

## OptiTrack backend

```bash
ros2 launch hope_bringup hope_bringup.launch.py \
  mocap_backend:=optitrack \
  mocap_server:=<MOTIVE_PC_IP> \
  mocap_network_latency_ms:=<MEASURED_ONE_WAY_MS>
```

(Or use the standalone `optitrack_hope_bridge.launch.py`.) This swaps the mocap source
while keeping the `/poses` contract byte-identical. Operational guide:
[docs/OPTITRACK.md](../OPTITRACK.md).

| Topic | Type | From → to | QoS |
|-------|------|-----------|-----|
| `/optitrack/poses` | `motion_capture_tracking_interfaces/NamedPoseArray` | vendored `motion_capture_tracking` driver → `optitrack_mct_relay` | sensor-data (best-effort, volatile, keep-last 1) |
| `/poses` | `geometry_msgs/PoseArray` | `optitrack_mct_relay` → `hope_planner` | best-effort, volatile, keep-last 1 |
| `/ball/point` | `geometry_msgs/PointStamped` | `optitrack_mct_relay` → (debug / downstream consumers) | best-effort, volatile, keep-last 1 |
| `/P1/pose`, `/P2/pose` | `geometry_msgs/PoseStamped` | `optitrack_mct_relay` → (debug / downstream consumers) | best-effort, volatile, keep-last 1 |
| `/table/pose` | `geometry_msgs/PoseStamped` | setup/recording only when `publish_table:=true`; no publisher is created by the competition-default relay | best-effort, volatile, keep-last 1 |

Notes:

- **`/optitrack/poses`** — ONE message per camera frame carrying every tracked
  object by name (Motive rigid-body assets verbatim: `Ball` — a strict 6-DOF
  rigid body per the HOPE spec — plus `P1`/`P2`; a `Table` asset appears only
  in setup/calibration sessions and is never streamed in competition, see
  [`optitrack_mct.yaml`](../../hope_ws/src/hope_bringup/config/optitrack_mct.yaml)).
  ⚠ deliberately remapped AWAY from the bare `/poses` name by
  `optitrack_hope_bridge.launch.py`: same name, DIFFERENT message type than the
  HOPE contract — an unremapped driver breaks the planner with a DDS type
  mismatch. The driver's raw TF is likewise remapped to `/optitrack/tf` /
  `/optitrack/tf_static` so the relay stays the only
  `world → Ball/P1/P2` TF authority (and setup-only `Table` only when explicitly
  enabled); `/optitrack/pointCloud` carries the
  unlabeled-marker cloud (diagnostics only — the ball is a rigid-body asset,
  never reconstructed from the cloud). The supplied driver uses
  `ros_latency_compensated` timestamps: local ROS receipt time minus NatNet's
  Camera/Motive latency and the measured `mocap_network_latency_ms`; neither
  bare receipt-time `ros` nor the unrelated Motive `camera` epoch is suitable
  for moving cross-sensor calibration.
- **`/poses`** — published by the relay ONLY on frames that contain the ball
  entry (an occluded ball is omitted by the driver), so a stale ball position
  is never re-emitted at rigid-body timestamps — the same
  ball-triggered publishing the VRPN path gets from `pose_to_posearray`.
  Competition order is `["ball", "P1", "P2"]` (ball first, matching the
  planner's default `ball_pose_index: 0`); absent objects are skipped.
  `publish_table` defaults to `false`, so even an accidentally active Motive
  Table asset creates no `/table/pose` publisher, Table TF, or `/poses` entry.
- **Rates** — OptiTrack rigs commonly stream 360 Hz (vs the 300 Hz VRPN
  default). The planner's `fit_window` is coupled to the rate
  (`round(31 × rate / 300)`, ≥ ~100 ms of samples — 360 Hz → 37); see
  [docs/OPTITRACK.md](../OPTITRACK.md). Measured `ros2 topic hz` can read
  below the camera rate under receive-side drops; that is normal for a
  best-effort sensor stream.

## `hope_msgs/RacketCommand`

Full definition:
[`hope_ws/src/hope_msgs/msg/RacketCommand.msg`](../../hope_ws/src/hope_msgs/msg/RacketCommand.msg).
All fields are in the world frame (metres, seconds).

| Field | Type | Meaning |
|-------|------|---------|
| `header` | `std_msgs/Header` | Stamp + world frame id. |
| `task_id` | `uint64` | New unique id per incoming ball. |
| `task_revision` | `uint32` | Increments as the pre-strike plan for the *same* ball is refined. |
| `FOREHAND` / `BACKHAND` | `int8` constants | `1` / `-1`. |
| `swing_side` | `int8` | Chosen once per task and locked for the whole strike. |
| `position` | `geometry_msgs/Point` | Target racket position at the strike (m). |
| `velocity` | `geometry_msgs/Vector3` | Target racket velocity at the strike (m/s). |
| `time_to_strike` | `float64` | Seconds until the strike. |

There is intentionally no `valid`/`reason`/failure field — if the incoming data
is insufficient, the planner simply has not published yet.

## QoS convention

- **Mocap side** (`/vrpn_mocap/...`, `/poses`): best-effort, volatile —
  high-rate sensor data where the latest sample is all that matters.
- **Command side** (`/racket/command`): reliable, volatile, keep-last depth 10 —
  a control setpoint that must not be dropped; the runner's subscription matches
  this profile.
