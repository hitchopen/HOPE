# HOPE A3 Foxglove Operator Interface

> **Formal model_21800 operator stack.** This branch combines the monitoring,
> OptiTrack/marker, pelvis TF and E-stop work from `Catrunaround/HOPE:nightly_built`
> with the native Runner contract. The imported 8-double TTY adapter and
> `/hope/control/*` services are retained as source history/compatibility assets
> but are not exposed by the fleet bridge and must not be enabled for the native
> Runner. Use the opt-in `8766` bridge, the services under `/hope/runner/*`,
> and `layouts/model21800_console.json`; the exact mapping and deployment
> boundary are in `docs/operations/foxglove_runner_integration.md`.
> The copy/paste deployment procedure is
> `docs/operations/foxglove_first_hardware_test.md`.

Rules and procedures for monitoring **any** Agibot A3 unit from a laptop on
the same subnet: NTP offset, aggregate CPU utilization, ROS-message timestamp
latency, vendor process and pelvis-TF readiness. The operator layouts contain
no URDF or robot TF-tree rendering; only the sanitized Pelvis link is shown.
The fleet endpoint exposes one audited action:
assert E-stop. In the integrated Runner deployment the E-stop requests the vendor
software latch and independently requests native Runner PASSIVE, but cannot
release either latch. All other Runner actions use the separate attended
control endpoint documented in `foxglove/README.md`. Motive/NatNet conversion,
ten-marker calibration, the calibration JSON, and
HOPE base-pose reconstruction all stay on the external computer. The A3 never
connects to or probes the Motive host and never stores, reads, or receives the
JSON; it consumes the computer's `/a3/base_pose_flat` output.

On the attended `8766` console, `/hope/calibrate` and
`/hope/refresh_x_hit` are deliberately separate. Calibration atomically
replaces the Laptop JSON with the fixed `P1 -> pelvis_link` result plus a
stationary `world -> pelvis_link` audit snapshot, then waits for the matching
live base receipt. Refresh x_hit touches only the Planner request/status files.

This document uses the current fixed3 site as its staged network example. The
operator must verify both distinct Wi-Fi addresses whenever the robot, Laptop,
or venue changes:

1. Shell sessions use the HDU address in `A3_HOST`.
2. `fastdds_bridge_profile.xml` uses UDPv4 without shared memory but does not
   pin a local IPv4 address. Runner supplies validated peers at startup.
3. `/etc/hope-foxglove/network.env` supplies the Laptop peer; the current
   runbook uses `172.23.20.46`.
4. The Foxglove connection dialog takes the HDU address as
   `ws://<HDU-IP>:8765` or, for the attended console, port `8766`.

> **Nothing in this folder is installed on a robot by itself.** The `a3/`
> directory contains staged deployment assets; a robot changes only when an
> operator runs the per-robot installation procedure below.

## Rules

- **R1 — One bridge per robot, on the robot.** The Foxglove bridge runs on the
  robot and serves a plain-TCP WebSocket over Wi-Fi. Separately, the external
  computer is an explicit FastDDS peer: it owns the calibration service and
  publishes `/a3/base_pose_flat`. Foxglove client-topic publishing remains
  disabled; it is not the base-pose transport.
- **R2 — Least-privilege control.** Client publishing, parameters, and remote
  assets remain disabled. The fleet service allowlist contains exactly one
  `std_srvs/Trigger` endpoint: the assert-only vendor E-stop proxy. The monitor advertises the E-stop proxy
  only while the live vendor `HalEmergencyService/SetEmergencyCommand` RPC is
  matched. It can only encode `software_emergency_stop=true`; it contains no
  reset/release path.
- **R3 — Additive only.** Two dedicated fleet-monitoring HDU systemd units; no
  vendor runner file is modified. The Runner observer, command proxy and 8766
  bridge are separately staged opt-in units. Disabling the units removes their runtime effect; the documented
  removal procedure separately cleans installed and build artifacts.
- **R4 — Bandwidth is opt-in.** The topic whitelist excludes H.265 camera
  and lidar streams by default. Add them per-robot, deliberately.
- **R5 — Verify per unit before trusting.** Fleet defaults below were
  verified live on one A3 (2026-08-04). Robots on different vendor software
  versions must pass the per-unit verification checklist before the layout
  is trusted.
- **R6 — Mocap and optional marker-CAD calibration stay computer-side.** The
  external computer receives NatNet from Motive and publishes
  `/optitrack/rigid_body_markers`. If an approved setup procedure runs the
  optional P1 marker-CAD calibration, only that computer writes
  `calibration/p1_to_pelvis.json`. The receipt includes the fixed extrinsic and
  a stationary world-pelvis audit snapshot; the latter is not a static runtime
  TF. Its base-pose relay reads the fixed extrinsic and
  publishes `/a3/base_pose_flat`; the A3 does not consume the marker stream or
  JSON and never attempts to reach the mocap network.
- **R7 — PM state is not TF readiness.** `/hope/vendor/agibot_pm_active`
  reports the HDU's local systemd unit literally. On split HDU/MDU deployments,
  the HDU unit can be active while MDU-owned joint/TF/localization processes are
  absent. `/hope/vendor/tf_ready` is the authoritative layout gate for the 3D
  pelvis indicator.

## Fleet defaults (verify per unit)

| Item | Fleet default | Verify with |
|---|---|---|
| ROS 2 distribution | Jazzy at `/opt/ros/jazzy` | `ls /opt/ros` |
| DDS domain | `ROS_DOMAIN_ID=232` | `grep -r ROS_DOMAIN_ID /opt/agibot/entry/env/` |
| DDS interfaces | HDU-MDU internal link plus the unit's Wi-Fi interface for the explicit computer peer | staged FastDDS profile and vendor `ros_dds_configuration.xml` |
| TF topics | `/tf`, `/tf_static` (`tf2_msgs/TFMessage`) | `ros2 topic type /tf` |
| Odometry | `/agivslam/localization/odometry` (`nav_msgs/Odometry`) | `ros2 topic list` |
| chrony | installed per `agibot/ntp_sync` | `chronyc tracking` |
| Software E-stop service | live `HalEmergencyService/SetEmergencyCommand`, conditionally wrapped by `/hope/safety/trigger_estop` | `/hope/safety/estop_ready`; do not test by firing it |

```text
A3 unit                                          External computer
  AimRT apps ⇄ iceoryx (local SHM)
            ⇄ ROS2/FastDDS domain 232  ──┐
  hope_monitor.py (health, E-stop, scene) ──┤
  fleet foxglove_bridge ─────────────────┴── ws://<robot-ip>:8765 ──► monitoring / E-stop
  Runner observer + command proxy ◄──► native Runner request/state
  attended control bridge ─────────────── ws://<robot-ip>:8766 ──► A3 Console

Motive ── NatNet ──► adapter ── ten-marker calibration ──► calibration/p1_to_pelvis.json
```

## Folder contents

```text
foxglove/
|-- README.md                        this rulebook
|-- layouts/
|   `-- a3_monitor.json              Foxglove layout (import on the laptop)
|-- assets/
|   `-- hope_ping_pong_table.urdf    static table visualization asset
|-- laptop/                          Laptop marker and local-asset services
|-- helpers/
|   `-- hope-lifecycle               fixed three-machine lifecycle helper
`-- a3/                              staged robot-side assets (NOT installed)
    |-- build_foxglove_bridge.sh     pinned ROS 2 bridge build vs /opt/ros/jazzy
    |-- bridge_params.yaml           read-only fleet endpoint on 8765
    |-- bridge_params_control.yaml   attended control endpoint on 8766
    |-- fastdds_bridge_profile.xml   UDPv4-only, address-agnostic host profile
    |-- network.env.example          one-place Laptop static-peer configuration
    |-- hope_monitor.py              health, scene and assert-only E-stop monitor
    |-- hope_observer.py             normalized Runner/operator state publisher
    |-- hope_command_proxy.py        fixed Runner and calibration service proxy
    |-- hope_lifecycle_supervisor.py fixed process lifecycle authority
    |-- hope-foxglove-bridge.service read-only fleet bridge
    |-- hope-foxglove-control-bridge.service
    |-- hope-monitor.service
    |-- hope-observer.service
    |-- hope-command-proxy.service
    |-- hope-lifecycle-supervisor.service
    |-- hope-runner-adapter.service  legacy TTY adapter; not installed for Runner
    |-- hope_runner_adapter.py       legacy source; not a Runner authority
    |-- hope_model21800_runner.sh    legacy MDU helper; not installed for Runner
    `-- patches/                     pinned A3 ament-index compatibility patches
```

## Per-robot installation (operator action; changes that robot)

Set the target once per session; every command below uses it:

```bash
export A3_HOST=<robot-ip-or-hostname>
```

### 1. Stage and build foxglove_bridge (once per robot)

> **Confirmed Stage 1 installation (2026-08-06).** One A3 running the vendor
> Debian 12 / ROS 2 Jazzy image was staged, built, and smoke-tested successfully.
> The installed bridge started as Foxglove Bridge `3.4.3`, listened on
> `0.0.0.0:8765`, and shut down cleanly on `SIGINT`. The build used
> `FOXGLOVE_BRIDGE_REMOTE_ACCESS=OFF`. This is a unit-specific confirmation,
> not a fleet-wide compatibility claim.

> **Revalidation status.** The compatibility work from the confirmed unit is
> now captured in `build_foxglove_bridge.sh` and its checked-in `patches/`.
> Patch application and resulting source hashes are tested locally against both
> pinned upstream commits. The revised one-command path still requires the
> normal per-unit verification below before it is trusted on another robot; it
> is not an unattended fleet rollout mechanism.

No compatible Jazzy apt repository is assumed on the vendor Debian 12 image.
The script is intended to build the official ROS 2 implementation from the
pinned Foxglove `ros-v3.4.3` release
(`05f27efc7e535d9c30c6b0cb4f6aa89de7243870`) with remote access disabled. The
robot must already carry `g++`, `cmake`, `colcon`, and the Jazzy development
tree:

```bash
ssh "agi@$A3_HOST" 'mkdir -p ~/foxglove_a3'
scp -r foxglove/a3/. "agi@$A3_HOST:~/foxglove_a3/"
ssh "agi@$A3_HOST" 'sudo apt-get install --no-install-recommends -y \
  git ca-certificates libssl-dev zlib1g-dev rapidjson-dev && \
  bash ~/foxglove_a3/build_foxglove_bridge.sh'
```

Result on the robot:
`~/hope_foxglove_ws/foxglove-sdk/ros/install/foxglove_bridge/`.
The build stops if either existing source checkout does not match its pinned
commit. It never silently replaces or updates a checkout, accepts only the two
expected compatibility modifications, and rejects untracked or additional
changes.

The confirmed installation has these identities:

| Item | Confirmed value |
|---|---|
| Foxglove source | `ros-v3.4.3` at `05f27efc7e535d9c30c6b0cb4f6aa89de7243870` |
| `rosx_introspection` source dependency | `3.1.1` at `ab747a0d3970d3297a5652b82e7645ab1d11feb9` |
| Foxglove compatibility patch | SHA-256 `1c6f40f6af4fe0186f196f65fb04c4d79b585ae16df448f16c72e09887d58828` |
| `rosx_introspection` compatibility patch | SHA-256 `bd6541d663b57505cc083b7d67aa5593c6297928adee04ed72e64ae35f6e4da5` |
| Patched Foxglove source file | SHA-256 `ff879cd712a4d167169c5d229a0f67e2c112f42b65ecb62404d6dc51cb44a8f1` |
| Patched `rosx_introspection` source file | SHA-256 `c3100994dea0fdc6dc2b87614ed35b1582d95819cd3f714a062910da64e36d16` |
| Installed bridge | `~/hope_foxglove_ws/foxglove-sdk/ros/install/foxglove_bridge/` |
| Bridge executable SHA-256 | `c9aeed8fb5ae95d06927ad1ef9e87b0a5f95c7ec7611ef90f6120f42c80411a1` |
| CMake remote-access setting | `FOXGLOVE_BRIDGE_REMOTE_ACCESS:BOOL=OFF` |
| Smoke test | Server started on port `8765`; timed `SIGINT` shutdown was clean |
| Stage 2 state | Installed and verified separately below |

Verify the installed result without starting a persistent service:

```bash
ssh "agi@$A3_HOST" '
  source /opt/ros/jazzy/setup.bash
  source ~/hope_foxglove_ws/foxglove-sdk/ros/install/setup.bash
  ros2 pkg prefix foxglove_bridge
  ros2 pkg executables foxglove_bridge
  sha256sum \
    ~/hope_foxglove_ws/foxglove-sdk/ros/install/foxglove_bridge/lib/foxglove_bridge/foxglove_bridge
  grep "^FOXGLOVE_BRIDGE_REMOTE_ACCESS:BOOL=" \
    ~/hope_foxglove_ws/foxglove-sdk/ros/build/foxglove_bridge/CMakeCache.txt
'
```

#### Captured vendor-image compatibility

The A3 vendor Jazzy tree supplies `ament_index_cpp` `1.8.1`, which does not
contain `ament_index_cpp/version.h`. The pinned Foxglove Bridge and
`rosx_introspection` sources otherwise select newer APIs using that header.
`build_foxglove_bridge.sh` now closes all three gaps found during the first
installation:

1. It sources `/opt/ros/jazzy/setup.bash` before enabling Bash nounset mode.
2. It fetches `rosx_introspection` at the exact confirmed commit and builds it
   in the same colcon invocation as Foxglove Bridge.
3. It checksum-verifies and applies narrow patches that unconditionally select
   the `ament_index_cpp` 1.8-compatible share-directory and resource APIs. It
   then verifies the complete patched source-file hashes before building.

The first unit's executable SHA-256 above remains the confirmed reference. Do
not treat a binary mismatch on another compiler/sysroot as sufficient proof of
a functional difference; verify source identities, CMake settings, startup,
shutdown, topics, and service restrictions using this checklist.

### 2. Install the bridge and monitor services

The calibration JSON and base-pose relay are computer-side components. Do not
create a calibration directory or install a JSON-reading world service on the
robot.

```bash
ssh "agi@$A3_HOST"
sudo install -D -o root -g root -m 0644 ~/foxglove_a3/bridge_params.yaml \
  /etc/hope-foxglove/bridge_params.yaml
sudo install -D -o root -g root -m 0644 ~/foxglove_a3/fastdds_bridge_profile.xml \
  /etc/hope-foxglove/fastdds_bridge_profile.xml
sudo install -D -o root -g root -m 0644 ~/foxglove_a3/network.env.example \
  /etc/hope-foxglove/network.env
# Edit this one file and set ROS_STATIC_PEERS to the Laptop Wi-Fi address.
# The current Runner fixed3 runbook uses 172.23.20.46; re-check it after a venue
# or network change.
sudoedit /etc/hope-foxglove/network.env
sudo install -D -o root -g root -m 0755 ~/foxglove_a3/hope_monitor.py \
  /usr/local/bin/hope_monitor.py
sudo install -D -o root -g root -m 0644 ~/foxglove_a3/hope_monitor_core.py \
  /usr/local/lib/hope-foxglove/hope_monitor_core.py
sudo install -D -o root -g root -m 0644 ~/foxglove_a3/hope-foxglove-bridge.service \
  /etc/systemd/system/hope-foxglove-bridge.service
sudo install -D -o root -g root -m 0644 ~/foxglove_a3/hope-monitor.service \
  /etc/systemd/system/hope-monitor.service

# If this unit's TF tree differs from the fleet defaults, edit only the
# pelvis_frame and reference_frame values in the installed monitor unit.
# No Motive/mocap IP is configured on the A3.

sudo systemctl daemon-reload
sudo systemctl enable --now hope-monitor.service hope-foxglove-bridge.service
systemctl status hope-monitor hope-foxglove-bridge --no-pager
```

> **Confirmed Stage 2 installation (2026-08-06).** On the same A3, both units
> were installed, enabled, and observed active with zero restarts. The bridge
> listened on `0.0.0.0:8765`, and a TCP connection from the external laptop
> succeeded. The NTP gate published `true`. The monitor unit contained no
> Motive IP or mocap-host parameter; laptop-side NatNet handling was confirmed
> as the intended architecture.

That confirmation describes the installed monitoring baseline before the
E-stop-proxy and timestamp-latency revision in this branch. Redeploy the staged
`bridge_params.yaml`, both fleet service units, and the monitor files
to activate the revised layout. The vendor E-stop request encoding was checked
against the installed A3 protobuf schemas; **no live E-stop was fired as part
of development or verification.**

The verification also found that the HDU `agibot_pm.service` was active while
the MDU vendor manager had been intentionally stopped for a custom application.
The vendor joint-state, localization, TF, and emergency-service endpoints were
therefore absent from a direct live ROS graph query. Consequently
`odom -> pelvis_link` was unavailable, and the E-stop proxy could not safely be
exposed. This is why the revised UI has
separate **HDU agibot_pm**, **LIVE TF READY**, and **E-STOP BACKEND READY**
indicators. A stale `ros2cli` daemon can retain old service names; the monitor
uses live client matching rather than trusting that cache.

Runtime removal disables the added processes and removes their installed
configuration. It does not claim to reverse apt packages that may have existed
before this procedure:

```bash
sudo systemctl disable --now hope-foxglove-bridge hope-monitor
sudo rm -f /etc/systemd/system/hope-foxglove-bridge.service \
  /etc/systemd/system/hope-monitor.service \
  /usr/local/bin/hope_monitor.py \
  /usr/local/lib/hope-foxglove/hope_monitor_core.py
sudo rmdir --ignore-fail-on-non-empty /usr/local/lib/hope-foxglove
sudo rm -rf /etc/hope-foxglove
sudo systemctl daemon-reload
```

After verifying that the following dedicated paths contain only files created
by this procedure, the operator may also remove staged/build artifacts:

```bash
rm -rf /home/agi/hope_foxglove_ws /home/agi/foxglove_a3
```

### 3. Per-unit verification checklist

```bash
ssh "agi@$A3_HOST" \
  'systemctl is-active hope-monitor hope-foxglove-bridge'
```

Then from the laptop, after connecting (next section): `/tf` frames appear in
the 3D panel when the vendor TF publisher is active;
`/hope/ntp/offset_ms` plots live values; the time indicator is green;
`/hope/clock/message_latency_ms` plots the A3 ROS clock minus the pelvis-IMU
header timestamp; `/hope/system/cpu_load_percent` plots aggregate A3 CPU
utilization from 0% to 100%; `/hope/vendor/agibot_pm_active` reports the local
HDU unit; and `/hope/vendor/tf_ready` becomes green only when the configured
live TF path exists. `/hope/safety/estop_ready` is true while at least one of
the vendor or native Runner emergency paths is callable, so the assert button
is not lost when managed operation intentionally stops `agibot_pm`.
`/hope/safety/estop_full_ready` is true only when both paths are live; a
one-path state is visibly `PARTIAL ONLY`. Native Runner acknowledgments and role/serve state are published by
the Runner observer on `/hope/runner/**` when the attended layer is installed.
The same observer converts `/poses[0]` into a 4 cm orange
`visualization_msgs/Marker` on `/hope/ball/marker`; its 0.2 s lifetime removes
the ball promptly when tracking is lost.
Camera topics, arbitrary publish controls, parameters, and all other services
remain unavailable.

On the A3, verify discovery without invoking the safety action:

```bash
ros2 topic echo /hope/vendor/agibot_pm_active --once
ros2 topic echo /hope/vendor/tf_ready --once
ros2 topic echo /hope/clock/message_latency_ms --once
ros2 topic echo /hope/system/cpu_load_percent --once
ros2 topic echo /hope/safety/estop_ready --once
ros2 topic echo /hope/safety/estop_full_ready --once
ros2 topic info /hope/pelvis/marker
ros2 topic info /hope/ball/marker
# Run the next line only after estop_ready reports true:
ros2 service type /hope/safety/trigger_estop
```

Do **not** invoke this service as a deployment smoke test. E-stop is a real
safety action.

> **Confirmed Stage 3 connection (2026-08-06).** Foxglove Desktop `2.58.0` on
> the external laptop completed a `foxglove.sdk.v1` WebSocket upgrade and held
> an established connection to the A3. The bridge registered the client and
> created subscriptions for `/tf` and `/tf_static`. Both robot services stayed
> active with zero restarts. That installed baseline exposed no client
> publishing, services, parameters, camera, or lidar topics; the current branch
> intentionally changes only the service portion by exposing the E-stop-only
> proxy. The local layout file remains a GUI import action; Foxglove deep links
> can select a remotely saved `layoutId` but cannot import this repository's
> local JSON file.

On that verification run, upstream vendor joint-state, TF, and localization
publishers were inactive. Therefore the connection baseline passed, while the
3D robot pose, joint freshness, and pelvis-pose data checks correctly remained
unavailable.

## Laptop setup — the robot IP is typed into the Foxglove UI

1. Install [Foxglove Desktop](https://foxglove.dev/download).
2. **Open connection → Foxglove WebSocket**, and type the robot's address
   as text in the URL field:

   ```text
   ws://<robot-ip>:8765
   ```

   This is the only place the laptop needs the robot's IP. To switch to a
   different A3, open a new connection with that robot's IP — the layout is
   fleet-generic and needs no editing.
3. Layout menu → **Import from file** → `foxglove/layouts/a3_monitor.json`.

Foxglove Desktop uses one active data source per window. This A3 layout connects
to `ws://<robot-ip>:8765`; to view laptop-side NatNet/ROS 2 mocap data at the
same time, open a second Foxglove window and connect that window to the laptop
mocap data source.

## Pelvis and ball in 3D

The operator layouts contain no URDF and do not expose
`/hope/robot_description`, `/joint_states`, raw `/tf`, or raw `/tf_static`
through the Foxglove bridges. `/hope/pelvis/tf` contains exactly one sanitized
`world -> pelvis_link` transform. The 3D panel shows only a `world` grid, that
Pelvis link and marker, and the ball marker.
`hope_monitor.py` validates the configured `world -> pelvis_link` pose, publishes
the structured/text status and standard visualization markers, and broadcasts
only the root transform needed to place the pelvis status in the HOPE world.
The Pelvis point and text markers expire after 0.5 seconds without a fresh
authoritative pose/TF, so the panel never presents a stale world position as live.

The live ball is a separate `/hope/ball/marker` topic in the same 3D panel.
It is visible only while `/poses` contains a finite pose at index 0; absence of
the ball is therefore represented by the marker expiring, not by retaining a
stale last position.

The grid follows the repository's HOPE world convention:

- `world=(0,0,0)` is the near-side left corner of the **playing surface** from
  P1's view;
- +X points along the table toward P2, so the table spans `x=[0, 2.740]` m;
- +Y points left from P1, so the table spans `y=[-1.525, 0]` m;
- +Z points up, the playing surface is `z=0`, and the floor is `z=-0.760` m.

The visual includes a red/green/blue +X/+Y/+Z triad at the origin. It is an
illustration only: no collision or planning geometry is provided.

The Pelvis indicator appears on the world grid only when the authoritative mocap
pose or TF provides a connected, fresh `world -> pelvis_link` path. Until then,
**LIVE TF READY** stays false and the 3D viewer intentionally shows only the
grid without a Pelvis label. Do not add an identity transform unless calibration
actually establishes that identity.

**Pelvis position and rotation in text:** `hope_monitor.py` looks up the
transform `reference_frame -> pelvis_frame` (layout defaults `world ->
pelvis_link`, both settable in `hope-monitor.service`) and publishes:

- `/hope/pelvis/pose` (`geometry_msgs/PoseStamped`) — structured values for
  optional inspection.
- `/hope/pelvis/text` (`std_msgs/String`) — one human-readable line:
  position in meters, quaternion, and roll/pitch/yaw in degrees. If the TF
  lookup fails, this topic carries the error text instead, so a wrong frame
  name is visible directly in the UI.

The structured pose is published only for a fresh authoritative mocap pose or
a fresh TF fallback. Consumers must still use `/hope/vendor/tf_ready` as the
validity bit because ROS topics retain the last pose a subscriber received.
The default freshness limit is 0.5 seconds and is configurable as
`tf_stale_after_s`.

During the managed model21800 session the authoritative root pose is
`/a3/mocap/pelvis_pose`, composed on the Laptop from the approved marker
calibration. The monitor validates its `world` frame and timestamp, republishes
it as `/hope/pelvis/pose`, and broadcasts `world -> pelvis_link`. Existing live
vendor TF remains the fallback when the authoritative mocap pose is absent.

To discover a unit's actual frame names, open the 3D panel's frame list, or
run on the robot: `ros2 topic echo /tf_static --once | grep frame_id`.

## CPU-load semantics

The CPU plot is the A3's aggregate Linux CPU utilization, sampled every second
from the first `cpu` row in `/proc/stat`:

```text
cpu_load_percent = 100 * (delta_total - delta(idle + iowait)) / delta_total
```

The first sample establishes the baseline, so the numeric topic begins on the
second poll. `/hope/system/cpu_load_percent` is clamped to the physical range
0%–100%, and the Foxglove Y-axis is fixed to the same range. This is not Linux
load average and it is not a sum of per-core percentages: 100% means the A3's
CPUs were collectively busy throughout the sampling interval. Sustained values
near 100% are evidence of compute saturation, but temperature throttling,
memory pressure, and scheduling latency require separate diagnostics. The
sampling interval is configurable as `cpu_publish_period_s` in
`hope-monitor.service`.

At the same one-second cadence the monitor reads the CPU tick counters in each
readable `/proc/<pid>/stat` and publishes the largest delta on
`/hope/system/cpu_top_process`. The text reports both `core=...%` (the familiar
one-core `top` scale) and `system=...%` (share of total machine CPU time). This
is diagnostic attribution; it does not change Runner, TF, joint, or latency
rates automatically.

## Timestamp latency and process-state semantics

The latency plot is not a ping test. `hope_monitor.py` subscribes to the A3's
`/ros2/body_drive/pelvis_imu/data` (`sensor_msgs/Imu`) using sensor-data QoS and
computes, at message arrival:

```text
latency_ms = (A3 ROS system clock now - message.header.stamp) / 1e6
```

It republishes the latest finite value at a bounded 20 Hz and marks the source
stale after 0.5 s. A negative or implausibly large value is useful evidence of
an epoch/clock problem; it is not clamped away. Change
`message_latency_topic` only to another `sensor_msgs/Imu` topic unless the node
is extended to support a different message type.

The green/red `agibot_pm` tile is the local **HDU** systemd state. It is not a
claim that the MDU vendor manager or TF publisher is ready. The adjacent
`LIVE TF READY` tile checks the configured `world -> pelvis_link` lookup and is
the display gate that matters for the combined robot/table scene. The 3D panel
follows `world`, not `pelvis_link`, so the table and HOPE axes remain usable
while the vendor tree is absent; this also avoids treating a missing
`pelvis_link` as the panel's required coordinate frame.

## Legacy TTY adapter reference (not used by native Runner integration)

The following section documents the colleague branch's imported 8-double
adapter for source provenance only. Do not install or enable it alongside the
native 19-double Runner interface, and do not expose its `/hope/control/*`
services on either integrated bridge.

### Prepare calibration and policy gate

`/hope/control/enter_prepare` starts a new initialization. The internal adapter
starts the unchanged `/agibot/a3_deploy_model21800/run_a3.sh` in a managed TTY
when needed and sends its existing `s` key. It never passes `--auto-start` and
refuses to start while vendor `motion_control` or another policy runner is
present. After the stock runner log reports `mode=pd_stand pd=N` with `N>150`
and no halt/fault evidence, the monitor invokes the
external computer's calibration service. That computer records the ten
installed waist markers (`f1`…`f5`, `b1`…`b5`) from
`/optitrack/rigid_body_markers` and fits their live 3-D geometry to the A3
pelvis CAD coordinates to obtain the fixed `P1 -> pelvis_link` transform.

Enable the computer adapter's `publish_p1_markers:=true` for each
initialization. Every PREPARE recomputes the transform, even if the previous
run's file is present, and an accepted fit atomically replaces the computer's
repository-relative `calibration/p1_to_pelvis.json` (for example,
`/home/user/HOPE/calibration/p1_to_pelvis.json`). The policy never triggers
another fit while playing; Foxglove does not transport or regenerate the
marker stream.

After the replacement, the computer-side base-pose relay only reads that JSON
for the rest of the run. It combines the stored transform with live mocap and
publishes `/a3/base_pose_flat` for the policy plus the unshifted reconstructed
world pelvis pose on `/a3/mocap/pelvis_pose` for diagnostics. The robot receives
the final `/a3/base_pose_flat` stream; it never stores, reads, or receives the
JSON. The policy-entry service remains locked until `/a3/base_pose_flat`
carries the exact SHA-derived calibration ID of the current PREPARE receipt.

`/a3/calibration/pelvis_pose` is intentionally not reused by this flow. It is
the independent `world -> pelvis_link` input of the older two-PoseStamped
calibration tool, and no node in this repository publishes it. Treating the
10-marker result as that independent input would make the calibration circular.

The four runner controls are:

- `/hope/control/enter_prepare`: request PD_STAND, then generate this
  initialization's marker calibration and replace the previous JSON.
- `/hope/control/enter_policy`: send the stock runner's existing `m` key only
  after its PD_STAND gate and the monitor's current-calibration/base-pose gate
  pass. The unchanged runner does not read the calibration JSON.
- `/hope/control/exit_policy`: return from policy to PD_STAND without replacing
  the current session calibration.
- `/hope/control/enter_passive`: send the stock runner's existing `p` key and
  cancel any pending prepare generation so it cannot later publish a
  calibration. PASSIVE is not E-stop.

## Foxglove E-stop button

The red button calls `/hope/safety/trigger_estop` with `{}`. The monitor
immediately latches out its legacy command path, requests the configured native
Runner `emergency_passive` service when available, and independently adds a
fresh timestamp and trace ID, encodes the vendor
`aimdk.protocol.EmergencyCommandReq`, and calls the live A3
`HalEmergencyService/SetEmergencyCommand` endpoint with only
`software_emergency_stop=true`. The HOPE proxy is dynamically advertised while
either that vendor endpoint or the native Runner emergency-PASSIVE endpoint is
matched and the local latch is clear. `/hope/safety/estop_ready` means at least
one explicit stop path is callable; `/hope/safety/estop_full_ready` is true only
when both independent paths are callable. A one-path state is always shown as
`PARTIAL ONLY` and requires the physical E-stop.
The independent managed Runner emergency service is attempted whenever it is
available, but its absence never suppresses the primary vendor stop; that case
returns an explicit `PARTIAL E-STOP` result.

Success requires both independent paths: the managed runner must be confirmed
stopped, AimRT's `RosRpcWrapper.code` must be zero, and the protobuf
`EmergencyCommandRsp.header.code` inside the returned `data` must decode to
zero. A nonzero application code, unexpected serialization,
missing header, or malformed payload is reported as failure; the vendor
message is included when present. The service callback does not hold the
readiness lock while waiting up to 2 seconds for the vendor RPC, so NTP, CPU,
TF, and other monitor timers continue publishing during the call. A concurrent
second E-stop request is rejected while the first remains in progress. The
Runner emergency action bypasses the ordinary operator-action lock, so a long
calibration cannot prevent the independent command-source removal request.
Before either backend call completes, the monitor stores an assert-only latch
at `/var/lib/hope-monitor/estop-latched`; `/hope/safety/estop_latched` remains
true across monitor restarts and the console follows this authoritative state
instead of a browser-local flag. Even when both requests succeed, this
repository has no independent actuator-state
feedback, so the response explicitly requires checking the physical E-stop.

- The button is a real safety action, not a test widget.
- The fleet bridge allowlists only the assert-only E-stop service; the attended
  bridge has a separate fixed Runner allowlist. Arbitrary services, parameters,
  and client-published topics remain blocked.
- There is deliberately no remote reset/release button. Inspect the robot and
  use Agibot's approved local recovery procedure after an E-stop. Only after
  the physical and vendor emergency state has been safely reset may an HDU
  operator remove the fixed latch file and restart `hope-monitor.service`.
- If the vendor wrapper package or emergency service is absent, the proxy is
  removed and the Foxglove control is unavailable. If the vendor endpoint drops
  during a call or its response cannot be validated, the callback fails closed
  and reports that success is unconfirmed; inspect the physical robot and its
  emergency state before any further action.

After the physical inspection and vendor emergency reset are complete, the
local operator must verify native Runner state and follow the approved local
recovery procedure. This recovery is intentionally unavailable through
Foxglove.

## What the layout shows

- **3D (left):** Pelvis link/world-pose marker and live ball on a HOPE `world`
  grid; no URDF, non-Pelvis robot links, or joint rendering.
- **NTP offset (upper right):** A3 system-clock offset and root dispersion in
  milliseconds.
- **ROS timestamp latency (upper right):** A3 ROS clock minus the pelvis-IMU
  message header timestamp, in milliseconds.
- **CPU utilization (upper right):** aggregate A3 CPU busy percentage over
  time, displayed on a fixed 0%–100% scale, plus the process with the largest
  per-sample CPU delta.
- **Status tiles:** HDU `agibot_pm`, connected TF readiness, runner readiness,
  calibration readiness, E-stop backend, NTP gate, and timestamp freshness use
  green/red/orange backgrounds.
- **A3 EMERGENCY STOP:** backed by the assert-only proxy described above and
  enabled when the live primary vendor backend exists and no prior latch is
  active; the detail text reports whether the Runner path is also ready.
- The integrated A3 Console on port 8766 provides the fixed native Runner actions
  documented in `docs/operations/foxglove_runner_integration.md`; the fleet
  layout itself has no legacy TTY Runner controls.

## Notes and limits

- **Security:** the fleet WebSocket has no authentication and carries the real
  E-stop service. The attended 8766 endpoint carries the fixed native Runner
  actions. Serve either only on the trusted lab subnet; do not
  port-forward it, and do not leave an untrusted Foxglove client connected.
- **Mocap network:** Motive access, NatNet reception, and mocap diagnostics are
  computer-side responsibilities. The A3 monitor has no mocap host parameter
  or network probe. It requests one computer-side calibration after settled
  PD_STAND and consumes the resulting `/a3/base_pose_flat`, not marker samples
  or JSON. Do not route the Motive network through A3.
- **Battery:** the BMS topic is protobuf-wrapped
  (`ros2_plugin_proto/msg/RosMsgWrapper`) and needs a decoder before
  Foxglove can display fields — planned extension, not in this layout.
- **Clock context:** `/hope/ntp/utc_qualified` uses the same Leap Normal plus
  selected-source rule as `timesync.sh --preflight`.
  `/hope/ntp/gate_pass` additionally applies the documented offset and skew
  limits (see
  [agibot/ntp_sync/README.md](../agibot/ntp_sync/README.md)). The monitor never
  modifies clock state. Robot-state mutations occur only through the explicitly
  allowlisted Trigger services on their respective endpoints.
