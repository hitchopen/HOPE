# vrpn_mocap (HOPE)

VRPN client for ROS 2 that publishes ChingMu/VRPN motion-capture poses. This
package originates from the ChingMu VRPN ROS 2 plugin and is vendored into the
independent `VRPN2ROS2` workspace for the HOPE motion-capture pipeline.

Tested target: Linux + ROS 2 Jazzy.

## Build

Use the Ubuntu 24.04/ROS 2 Jazzy `hope` Distrobox documented in the workspace
[`README`](../../README.md). From the `VRPN2ROS2` workspace root inside that
container:

```
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## Run

```
ros2 launch vrpn_mocap client.launch.yaml server:=<MOCAP_SERVER_IP> port:=3883
```

`client.launch.yaml` arguments:

- `server` -- VRPN server IP/hostname (default: `localhost`)
- `port` -- VRPN server port (default: `3883`)
- `update_freq` -- VRPN client polling frequency (default: `500.0` Hz)
- `output_rate_hz` -- maximum ROS output rate per topic/sensor (default:
  `200.0` Hz; `0.0` publishes every accepted report)

`config/client.yaml` parameters:

- `update_freq` (double) -- VRPN socket/mainloop polling frequency (default:
  `500.`; keep it at or above the measured server stream rate)
- `output_rate_hz` (double) -- maximum pose, velocity, and acceleration
  publication rate per sensor (default: `200.0`; `0.0` disables limiting)
- `refresh_freq` (double) -- frequency of dynamically adding newly tracked objects (default: `1.`)
- `sensor_data_qos` (bool) -- use best-effort QoS for the VRPN data stream; set to `false` for the reliable system-default QoS (default: `true`)
- `multi_sensor` (bool) -- set to `true` if more than one sensor (frame) reports on the same object (default: `false`)
- `use_vrpn_timestamps` (bool) -- preserve the server-provided VRPN `timeval`
  instead of ROS receipt time (default: `true`; strict mode fails closed)
- `validate_vrpn_timestamps` (bool) -- compare that source timestamp with the
  adapter's absolute system clock and fail closed (default: `true`)
- `max_vrpn_timestamp_age_ms` / `max_vrpn_future_skew_ms` -- accepted source
  time bounds (defaults: 100 ms / 5 ms; 100 ms is bring-up-only and must be
  tightened from measured venue latency before competition)
- `min_age_monitor_window_ms` / `min_age_monitor_warmup_samples` -- window and
  warmup for the runtime minimum-age proxy (defaults: 5000 ms / 100 samples)
- `max_vrpn_min_age_shift_ms` -- maximum change in
  `min(receipt_time - source_stamp)` from the runtime reference (default: 5 ms)
- `validate_expected_vrpn_min_age` / `expected_vrpn_min_age_ms` /
  `max_expected_vrpn_min_age_error_ms` -- optional fail-closed comparison with
  a minimum age commissioned during a known-good run (defaults: disabled /
  0 ms / 5 ms)

`update_freq` does not change the CMTracker stream rate. A VRPN connection
mainloop drains the UDP/TCP reports currently available in the socket queue,
so polling below the source rate does not intentionally decimate the stream.
It does, however, publish reports in polling bursts: a 100 Hz timer adds an
ideal 0–10 ms wait before callback dispatch, whereas the checked-in 500 Hz
setting reduces that term to approximately 0–2 ms. OS scheduling and overload
can add more, and UDP socket overflow can still lose reports. Measure the
observed output rate and age distribution rather than assuming losslessness.

`output_rate_hz` is the separate ROS/DDS traffic control. Every received report
still passes timestamp and minimum-age validation; only accepted publications
are limited. Published messages retain the original VRPN server timestamp.
Sensor indices outside 0–255 are rejected before any limiter or publisher
vector is resized; normal HOPE rigid-body topics use sensor index 0.

## Inspect the data stream

```
ros2 topic list
ros2 topic echo /vrpn_mocap/<tracker>/pose_id_<N> --once
```

Raw topics live under the `/vrpn_mocap` namespace: `pose` per tracker with
`multi_sensor: false`, or `pose_id_<N>` per sensor with `multi_sensor: true`
(the bundled `client.launch.yaml` forces the latter, so a tracker named `Ball`
publishes `/vrpn_mocap/Ball/pose_id_0`). The HOPE planner consumes a
`geometry_msgs/PoseArray` with the ball at `ball_pose_index`; map or relay your
tracker's ball pose onto that topic.

## Timestamp validation

VRPN forwards the server's timestamp without cross-host clock correction. Both
hosts must use the same NTP/PTP epoch. Run `vrpn_timestamp_probe.py` on the
adapter host as documented in the workspace-level
[`../../README.md`](../../README.md). Its default Chrony gate plus the stream
checks prove local NTP health and absolute epoch/age consistency; verify the
server host's NTP health separately. The runtime sliding minimum detects
offset/delay changes, while the optional commissioned minimum is required to
catch a static offset already present at startup. Neither mechanism
synchronizes the clocks or separates clock offset from one-way delay. No
packet-only test proves proprietary camera-exposure provenance.

## Runtime

Launch this client independently. `hope_bringup` starts only the downstream
`pose_to_posearray` adapter and planner.

## License

See [LICENSE](LICENSE).
