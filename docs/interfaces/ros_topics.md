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
/poses                                 geometry_msgs/PoseArray     (ball at index 0)
        |  hope_planner
        v
/racket/command                        hope_msgs/RacketCommand
        |  python -m a3_deploy_onnx_ref_pingpong --planner
        v
50 Hz policy control loop
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
  pose, trigger stamp passed through unmodified. The planner reads the ball at
  `ball_pose_index` (default 0). With `use_fake_ball:=true`,
  `fake_ball_publisher` publishes this form directly.
- **`/racket/command`** — the planner feeds every mocap sample to its estimator
  but solves at most every `solve_period_s` (≤ 50 Hz); topic names and tuning
  live in
  [`hope_ws/src/hope_planner/config/hope_planner.yaml`](../../hope_ws/src/hope_planner/config/hope_planner.yaml).
  The reference runner's `--planner` mode subscribes with the matching reliable
  QoS and hands the newest command to the 50 Hz control loop.

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
