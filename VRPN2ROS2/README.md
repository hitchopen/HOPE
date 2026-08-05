# VRPN2ROS2

`VRPN2ROS2` is the standalone VRPN-to-ROS 2 adapter workspace. It connects to
a VRPN server such as ChingMu CMTracker/MCServer and publishes one
`geometry_msgs/PoseStamped` topic per tracker/sensor. It does not contain the
HOPE planner or the `/poses` aggregator.

## Build

```bash
cd VRPN2ROS2
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## Launch

```bash
source VRPN2ROS2/install/setup.bash
ros2 launch vrpn_mocap client.launch.yaml \
  server:=<CHINGMU_SERVER_IP> port:=3883
```

With `multi_sensor: true`, a tracker named `Ball` normally publishes
`/vrpn_mocap/Ball/pose_id_0`.

## ROS 2 NTP timestamp estimation and validation

The checked-in deployment config enables `use_vrpn_timestamps: true`. The ROS
header therefore preserves the VRPN packet's server-supplied `timeval` at
microsecond resolution instead of using adapter receipt time.

Unlike NatNet's QPC mapping, this path has no protocol exchange from which the
adapter can estimate a true server-to-adapter clock offset. Timestamp handling
is therefore:

1. CMTracker/MCServer assigns the VRPN tracker report a `timeval` containing
   seconds and microseconds in the server host's wall-clock domain.
2. VRPN forwards that value without clock conversion. The adapter validates
   its structure, converts it to nanoseconds, and constructs an
   `RCL_SYSTEM_TIME` ROS stamp in the Unix epoch.
3. At receipt, the adapter reads its own Chrony-disciplined
   `RCL_SYSTEM_TIME` and evaluates
   `age = adapter_system_time - server_stamp`. Stale/future limits and a
   sliding `min(age)` monitor reject implausible timestamps or a changed
   server-clock/network regime.
4. An optional minimum-age value commissioned during a known-good wired run
   detects a static server offset already present when the adapter starts.

### Client polling latency

The client must call the VRPN connection mainloop to move received reports from
the OS socket queue into ROS callbacks. VRPN processes all reports currently
available during that call, so a polling rate below the server stream rate does
not deliberately downsample the stream. It still delays callback dispatch
until the next poll, and UDP buffer overflow or sustained CPU overload can lose
reports.

The checked-in competition-oriented default is `update_freq: 500.0`, above the
typical 300–360 Hz mocap stream. Ignoring scheduler jitter, this bounds the
polling wait to about 2 ms instead of the 10 ms introduced by the former 100 Hz
setting. Keep `update_freq` at least as high as the measured server rate and
retain headroom; it can be overridden at launch with `update_freq:=<HZ>`.

The source `header.stamp` remains the server report time regardless of polling
frequency, but the driver's receipt-side age, the subscriber probe's p95 age,
and real planner availability all include this socket/polling wait. Therefore
the competition age budget must include server processing, network transit,
VRPN polling, ROS callback/publication, and downstream DDS scheduling before
tightening a 20–30 ms gate.

The resulting header can share an absolute epoch with the humanoid only when
the CMTracker/MCServer host, adapter computer, and robot computer are all
disciplined to the same approved NTP source. VRPN validation detects many
failures but does not itself synchronize those clocks or separate server clock
offset from one-way network delay. For Agibot A3, the robot-side Chrony and
internal PTP implementation is documented in
[`../agibot/ntp_sync/README.md`](../agibot/ntp_sync/README.md).

VRPN does **not** synchronize the server and client clocks. The CMTracker/
MCServer host, adapter host, robot host, and other measurement producers must
be disciplined to the same NTP/PTP absolute epoch. The driver compares every
source stamp with the adapter host's `RCL_SYSTEM_TIME` and drops timestamps
that are more than 100 ms old, more than 5 ms in the future, structurally
invalid, or not representable as ROS nanoseconds. Receipt-time fallback also
uses `RCL_SYSTEM_TIME`, so enabling `/use_sim_time` cannot move fallback
headers into a simulated epoch.

Validation is not synchronization. In addition to the absolute window, the
adapter continuously tracks
`min(adapter_receipt_system_time - server_stamp)` over a monotonic-time sliding
window. This is a minimum-one-way-delay-plus-clock-offset proxy. A change larger
than `max_vrpn_min_age_shift_ms` is logged and, with strict validation enabled,
frames are dropped until the proxy returns to the trusted regime. It catches a
clock or link regime change during a run, but it cannot identify which term
changed.

There is one unavoidable single-ended-observation limit: a static server clock
offset already present when the adapter starts becomes its runtime reference.
For competition, record the adapter log's “minimum-age monitor established
runtime reference” value over multiple known-good wired runs after verifying
NTP on both hosts. Set the stable value as `expected_vrpn_min_age_ms`, then
enable `validate_expected_vrpn_min_age`. Use the probe's `age_ms_min` only as a
cross-check: subscriber-side DDS/scheduling delay is included there but not in
the driver's receipt-time proxy. The commissioned check is what makes a
startup-static 30–40 ms server offset fail closed. It remains an acceptance
baseline, not a replacement for monitoring the CMTracker host's NTP service.

Run the live acceptance probe **on the adapter host** after launch. By default
it also requires `chronyc tracking` to report a normal source, a valid stratum,
and no more than 1 ms of local system-clock correction:

```bash
ros2 run vrpn_mocap vrpn_timestamp_probe.py \
  --topic /vrpn_mocap/Ball/pose_id_0 \
  --samples 1500 --min-hz 250 \
  --max-p95-age-ms 20 --max-age-ms 100 --max-future-ms 5 \
  --max-ntp-offset-ms 1
```

A passing probe proves that the ROS headers are monotonic and share the
adapter's locally validated NTP/Unix epoch within the configured age bounds.
The age contains server clock error, CMTracker processing, network transit,
VRPN polling/socket queueing, adapter callback/publication, and probe-side DDS
scheduling; it bounds their combination rather than identifying each component.
Verify the CMTracker/MCServer host's own NTP health separately.
The checked-in 100 ms per-sample driver bound is a bring-up safety ceiling, not
an approved competition value. Before competition, measure the wired venue,
then reduce `max_vrpn_timestamp_age_ms` to just above the observed high
percentile/worst credible transport age plus a documented margin. A 20–30 ms
bound is a reasonable candidate only if the measurement supports it. The probe
example's 20 ms p95 gate does not by itself detect all clock offsets; use the
runtime minimum-age monitor and commissioned expected minimum as well.

The relevant runtime parameters are:

- `min_age_monitor_window_ms` and `min_age_monitor_warmup_samples`: sliding
  window and self-reference warmup;
- `max_vrpn_min_age_shift_ms`: allowed change from the runtime reference;
- `validate_expected_vrpn_min_age`, `expected_vrpn_min_age_ms`, and
  `max_expected_vrpn_min_age_error_ms`: optional commissioned startup-static
  offset gate.

The probe does **not** prove that proprietary CMTracker software associates the
VRPN report timestamp with camera exposure or exposure midpoint. That final
provenance claim requires one of the following:

1. a CMTracker/SDK specification identifying the VRPN timestamp as the camera
   exposure timestamp; or
2. a hardware-trigger/frame-log comparison between the camera frame timestamp
   and `/vrpn_mocap/.../pose_id_0`.

Without that evidence the precise statement is “NTP-aligned VRPN server report
time,” not “proven source-camera exposure time.” A NatNet
`CameraMidExposureTimestamp` and a VRPN server report timestamp therefore refer
to different events even when both are expressed in Unix time; the unknown
camera-exposure-to-report processing interval must remain in the planner's
latency/error budget.

`--skip-ntp-check` exists only for hosts that do not use Chrony. If used, attach
equivalent OS/PTP clock-health evidence to the acceptance record; timestamp age
alone is not an NTP-health proof.

## Connect to HOPE

Build `hope_ws` independently. Start this adapter first, then source both
overlays in the HOPE terminal:

```bash
source VRPN2ROS2/install/setup.bash
source hope_ws/install/setup.bash
ros2 launch hope_bringup hope_bringup.launch.py \
  mocap_backend:=vrpn \
  ball_pose_topic:=/vrpn_mocap/Ball/pose_id_0
```

The HOPE `pose_to_posearray` node passes the trigger `PoseStamped` header
through unchanged.
