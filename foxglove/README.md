# A3 Foxglove Monitoring — Fleet Rulebook

Rules and procedures for monitoring **any** Agibot A3 unit from a laptop on
the same subnet: NTP offset, aggregate CPU utilization, ROS-message timestamp
latency, vendor process and TF readiness, and the live A3 URDF beside a
HOPE-aligned table asset. The layout also has one narrowly scoped button that
can **assert** the vendor software E-stop but cannot release it. Motive/NatNet
conversion runs on the external laptop; the A3 neither connects to nor probes
the mocap host.

This document is fleet-generic. **No robot Wi-Fi address is hardcoded in this
folder.** The operator supplies the robot's address in exactly two places. The
separate `10.42.10.10` address in the DDS profile is the fleet-standard internal
HDU interface and must be verified per unit:

1. Shell sessions use the `A3_HOST` variable (`export A3_HOST=<robot-ip>`).
2. The Foxglove UI takes the robot IP as text in its connection dialog:
   `ws://<robot-ip>:8765`.

> **Nothing in this folder is installed on a robot by itself.** The `a3/`
> directory contains staged deployment assets; a robot changes only when an
> operator runs the per-robot installation procedure below.

## Rules

- **R1 — One bridge per robot, on the robot.** The vendor FastDDS profile
  binds DDS to the internal HDU interface only (fleet-standard address
  `10.42.10.10` on the HDU-MDU wire). DDS is invisible on Wi-Fi by design;
  never try to join the ROS 2 graph from a laptop. The bridge runs on the
  robot and serves a WebSocket, which is plain TCP and reachable on Wi-Fi.
- **R2 — Least-privilege control.** Client publishing, parameters, and remote
  assets remain disabled. The sole service allowlist entry is
  `/hope/safety/trigger_estop` (`std_srvs/Trigger`). The monitor advertises that
  proxy only while the live vendor `HalEmergencyService/SetEmergencyCommand`
  RPC is matched. It can only encode `software_emergency_stop=true`; it
  contains no reset/release path.
- **R3 — Additive only.** Two dedicated systemd units; no vendor file is
  modified. Disabling the units removes their runtime effect; the documented
  removal procedure separately cleans installed and build artifacts.
- **R4 — Bandwidth is opt-in.** The topic whitelist excludes H.265 camera
  and lidar streams by default. Add them per-robot, deliberately.
- **R5 — Verify per unit before trusting.** Fleet defaults below were
  verified live on one A3 (2026-08-04). Robots on different vendor software
  versions must pass the per-unit verification checklist before the layout
  is trusted.
- **R6 — Mocap stays laptop-side.** The external laptop receives NatNet from
  Motive and converts it to ROS 2 for Foxglove. The A3 services do not require
  the Motive IP and do not attempt to reach the mocap network.
- **R7 — PM state is not TF readiness.** `/hope/vendor/agibot_pm_active`
  reports the HDU's local systemd unit literally. On split HDU/MDU deployments,
  the HDU unit can be active while MDU-owned joint/TF/localization processes are
  absent. `/hope/vendor/tf_ready` is the authoritative layout gate for the 3D
  robot.

## Fleet defaults (verify per unit)

| Item | Fleet default | Verify with |
|---|---|---|
| ROS 2 distribution | Jazzy at `/opt/ros/jazzy` | `ls /opt/ros` |
| DDS domain | `ROS_DOMAIN_ID=232` | `grep -r ROS_DOMAIN_ID /opt/agibot/entry/env/` |
| DDS interface whitelist | `10.42.10.10` only | vendor `ros_dds_configuration.xml` |
| TF topics | `/tf`, `/tf_static` (`tf2_msgs/TFMessage`) | `ros2 topic type /tf` |
| Odometry | `/agivslam/localization/odometry` (`nav_msgs/Odometry`) | `ros2 topic list` |
| Joint states | `/motion/control/{leg,arm,hand,neck,waist}_joint_state` | `ros2 topic list` |
| URDF selection | model ID in `/agibot/data/info/model`; model package under `/opt/agibot/share/robot_model/models/<lowercase-model>/` | see URDF section |
| chrony | installed per `agibot/ntp_sync` | `chronyc tracking` |
| Software E-stop service | live `HalEmergencyService/SetEmergencyCommand`, conditionally wrapped by `/hope/safety/trigger_estop` | `/hope/safety/estop_ready`; do not test by firing it |

```text
A3 unit                                          Laptop (same subnet)
  AimRT apps ⇄ iceoryx (local SHM)
            ⇄ ROS2/FastDDS domain 232  ──┐
  hope_monitor.py (NTP, CPU, timestamp, ─┤
    PM/TF status, E-stop-only proxy)     │
  foxglove_bridge ───────────────────────┴── ws://<robot-ip>:8765 ──► Foxglove Desktop

Motive ── NatNet ──► laptop ROS 2 adapter ──────────────────────────► Foxglove Desktop
```

## Folder contents

```text
foxglove/
|-- README.md                        this rulebook
|-- layouts/
|   `-- a3_monitor.json              Foxglove layout (import on the laptop)
|-- assets/
|   `-- hope_ping_pong_table.urdf    visualization-only HOPE-world table
`-- a3/                              staged robot-side assets (NOT installed)
    |-- build_foxglove_bridge.sh     pinned ROS 2 bridge build vs /opt/ros/jazzy
    |-- bridge_params.yaml           port, address, topic whitelist
    |-- fastdds_bridge_profile.xml   UDPv4-only profile matching the vendor whitelist
    |-- hope-foxglove-bridge.service systemd unit (bridge)
    |-- hope_monitor.py              ROS monitor plus E-stop-only proxy
    |-- hope_monitor_core.py         ROS-free probe parsing and health rules
    |-- hope-monitor.service         systemd unit (monitor node)
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

```bash
ssh "agi@$A3_HOST"
sudo install -D -o root -g root -m 0644 ~/foxglove_a3/bridge_params.yaml \
  /etc/hope-foxglove/bridge_params.yaml
sudo install -D -o root -g root -m 0644 ~/foxglove_a3/fastdds_bridge_profile.xml \
  /etc/hope-foxglove/fastdds_bridge_profile.xml
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
sudo systemctl enable --now hope-foxglove-bridge.service hope-monitor.service
systemctl status hope-foxglove-bridge hope-monitor --no-pager
```

> **Confirmed Stage 2 installation (2026-08-06).** On the same A3, both units
> were installed, enabled, and observed active with zero restarts. The bridge
> listened on `0.0.0.0:8765`, and a TCP connection from the external laptop
> succeeded. The NTP gate published `true`. The monitor unit contained no
> Motive IP or mocap-host parameter; laptop-side NatNet handling was confirmed
> as the intended architecture.

That confirmation describes the installed monitoring baseline before the
E-stop-proxy and timestamp-latency revision in this branch. Redeploy the staged
`bridge_params.yaml`, both service units, and both `hope_monitor*.py` files to
activate the revised layout. The vendor E-stop request encoding was checked
against the installed A3 protobuf schemas; **no live E-stop was fired as part
of development or verification.**

The verification also found that the HDU `agibot_pm.service` was active while
the MDU vendor manager had been intentionally stopped for a custom application.
The vendor joint-state, localization, TF, and emergency-service endpoints were
therefore absent from a direct live ROS graph query. Consequently
`/hope/joints/fresh` was `false`, `odom -> pelvis_link` was unavailable, and the
E-stop proxy could not safely be exposed. This is why the revised UI has
separate **HDU agibot_pm**, **LIVE TF READY**, and **E-STOP BACKEND READY**
indicators. A stale `ros2cli` daemon can retain old service names; the monitor
uses live client matching rather than trusting that cache.

Runtime removal disables the added processes and removes their installed
configuration. It does not claim to reverse apt packages that may have existed
before this procedure:

```bash
sudo systemctl disable --now hope-foxglove-bridge hope-monitor
sudo rm -f /etc/systemd/system/hope-foxglove-bridge.service \
  /etc/systemd/system/hope-monitor.service /usr/local/bin/hope_monitor.py \
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
ssh "agi@$A3_HOST" 'systemctl is-active hope-foxglove-bridge hope-monitor'
```

Then from the laptop, after connecting (next section): `/tf` frames appear in
the 3D panel when the vendor TF publisher is active;
`/hope/ntp/offset_ms` plots live values; the time indicator is green;
`/hope/clock/message_latency_ms` plots the A3 ROS clock minus the pelvis-IMU
header timestamp; `/hope/system/cpu_load_percent` plots aggregate A3 CPU
utilization from 0% to 100%; `/hope/vendor/agibot_pm_active` reports the local
HDU unit; and `/hope/vendor/tf_ready` becomes green only when the configured
live TF path exists. `/hope/safety/estop_ready` becomes green only when the
vendor emergency RPC is live; only then is `/hope/safety/trigger_estop`
advertised. Camera topics, arbitrary publish controls, parameters, and all
other services remain unavailable.

On the A3, verify discovery without invoking the safety action:

```bash
ros2 topic echo /hope/vendor/agibot_pm_active --once
ros2 topic echo /hope/vendor/tf_ready --once
ros2 topic echo /hope/clock/message_latency_ms --once
ros2 topic echo /hope/system/cpu_load_percent --once
ros2 topic echo /hope/safety/estop_ready --once
ros2 topic info /hope/robot_description
# Run the next line only after estop_ready reports true:
ros2 service type /hope/safety/trigger_estop
```

Do **not** use `ros2 service call` as a smoke test: a successful call asserts
the real vendor software E-stop.

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

## Robot and HOPE-world table in 3D

The table environment is independent of the robot state and is always enabled.
The A3 layer is sourced from `/hope/robot_description`, not directly from a URL.
`hope_monitor.py` publishes the selected A3 URDF on that topic only while the
configured `world -> ... -> pelvis_link` transform exists and is no more than
0.5 seconds old. When the lookup fails or becomes stale, it immediately
publishes an empty description, which removes the robot while leaving the table
and HOPE axes visible. The description publisher uses reliable,
transient-local, depth-1 QoS, so a newly connected bridge/client can receive
the current visible-or-hidden state immediately instead of waiting for the
periodic refresh.

This explicit gate is necessary. Foxglove's built-in transform-controlled URDF
layer can fall back to the URDF's internal joint tree when data-source frames
are incomplete; merely enabling a URL layer does **not** guarantee that a
disconnected robot stays hidden. See Foxglove's
[URDF custom-layer documentation](https://docs.foxglove.dev/docs/visualization/panels/3d#urdf-custom-layer).

Each robot still supplies its own visual mesh files. The installed A3 selects
its model using `/agibot/data/info/model`. Copy that complete model directory
to the matching lowercase directory on the laptop (read-only on the robot):

```bash
A3_MODEL="$(ssh "agi@$A3_HOST" \
  'tr "[:upper:]" "[:lower:]" < /agibot/data/info/model')"
mkdir -p foxglove/urdf
scp -r \
  "agi@$A3_HOST:/opt/agibot/share/robot_model/models/$A3_MODEL" \
  foxglove/urdf/
```

For the verified unit, `/agibot/data/info/model` contains `A3_P1D0`, so this
selects `/opt/agibot/share/robot_model/models/a3_p1d0/urdf/model.urdf`. Its mesh
references are relative paths such as `../meshes/pelvis_link.STL`. The previous
`/proc/<run_agibot-pid>/environ` procedure is not reliable on this image: the
launcher exists, but those URDF variables were not present in its live process
environment.

Serve the whole `foxglove/` folder from the laptop. This one server supplies
both the copied per-unit meshes and the checked-in table asset:

```bash
cd foxglove && python3 -m http.server 8000
```

No A3 layer URL needs to be edited in Foxglove. The monitor reads the selected
robot-side URDF and rewrites its relative mesh references to the corresponding
`http://localhost:8000/urdf/<model>/...` locations before publishing the gated
description. Here, `localhost` is resolved by Foxglove Desktop on the laptop.
`package://<package>/...` references are mapped to
`http://localhost:8000/urdf/<package>/...`; the copied directory name must
therefore match the lowercase ROS package name. Uppercase package names are
rejected so a layout cannot work accidentally on macOS and then 404 on a
case-sensitive Linux laptop. Relative paths such as the A3's
`../meshes/pelvis_link.STL` remain supported, but the resolved URL must stay
inside `robot_asset_root_url`. Local absolute paths and relative escapes are
rejected rather than exposed. The URL root is configurable in
`hope-monitor.service`.

The checked-in **Standard ping-pong table (HOPE world)** layer remains a
separate, always-on URL:
`http://localhost:8000/assets/hope_ping_pong_table.urdf`.

The table illustration uses regulation dimensions (2.740 m × 1.525 m, playing
surface 0.760 m above the floor, net height 0.1525 m) and the repository's HOPE
world convention:

- `world=(0,0,0)` is the near-side left corner of the **playing surface** from
  P1's view;
- +X points along the table toward P2, so the table spans `x=[0, 2.740]` m;
- +Y points left from P1, so the table spans `y=[-1.525, 0]` m;
- +Z points up, the playing surface is `z=0`, and the floor is `z=-0.760` m.

The visual includes a red/green/blue +X/+Y/+Z triad at the origin. It is an
illustration only: no collision or planning geometry is provided.

The robot appears beside the table only when TF provides a connected, fresh
`world -> ... -> pelvis_link` path. `agibot_pm` can restore the vendor joint and
localization tree, but a vendor tree rooted at `odom` still needs a measured
`world -> odom` alignment before it is a HOPE-world scene. Until that measured
connection exists, **LIVE TF READY** stays false and the 3D viewer intentionally
shows the table without the robot. Do not add an identity transform unless the
table-origin calibration actually establishes that identity.

**Pelvis position and rotation in text:** `hope_monitor.py` looks up the
transform `reference_frame -> pelvis_frame` (layout defaults `world ->
pelvis_link`, both settable in `hope-monitor.service`) and publishes:

- `/hope/pelvis/pose` (`geometry_msgs/PoseStamped`) — structured values for
  optional inspection.
- `/hope/pelvis/text` (`std_msgs/String`) — one human-readable line:
  position in meters, quaternion, and roll/pitch/yaw in degrees. If the TF
  lookup fails, this topic carries the error text instead, so a wrong frame
  name is visible directly in the UI.

The structured pose is published only for fresh TF samples. Consumers must
still use `/hope/vendor/tf_ready` as the validity bit because ROS topics retain
the last pose a subscriber received. The default freshness limit is 0.5 seconds
and is configurable as `tf_stale_after_s`.

The monitor caches the five vendor joint groups but publishes the merged
`/joint_states` at a bounded 20 Hz. It publishes only while every group is
fresh, preserves the oldest contributing source timestamp, and exposes group
health on `/hope/joints/fresh` and `/hope/joints/text`. The rate and freshness
window are configurable in `hope-monitor.service`.

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

## Foxglove E-stop button

The red button calls `/hope/safety/trigger_estop` with `{}`. The monitor adds a
fresh timestamp and trace ID, encodes the vendor
`aimdk.protocol.EmergencyCommandReq`, and calls the live A3
`HalEmergencyService/SetEmergencyCommand` endpoint with only
`software_emergency_stop=true`. The HOPE proxy is dynamically advertised only
while the vendor endpoint is matched; otherwise Foxglove shows the service as
unavailable and the adjacent backend tile is red.

Success requires both layers to pass: AimRT's `RosRpcWrapper.code` must be zero,
and the protobuf `EmergencyCommandRsp.header.code` inside the returned `data`
must decode to zero. A nonzero application code, unexpected serialization,
missing header, or malformed payload is reported as failure; the vendor
message is included when present. The service callback does not hold the
readiness lock while waiting up to 2 seconds for the vendor RPC, so NTP, CPU,
TF, and other monitor timers continue publishing during the call. A concurrent
second E-stop request is rejected while the first remains in progress.

- The button is a real safety action, not a test widget.
- The bridge allowlists this one `std_srvs/Trigger` service only; arbitrary
  services, parameters, and client-published topics remain blocked.
- There is deliberately no remote reset/release button. Inspect the robot and
  use Agibot's approved local recovery procedure after an E-stop.
- If the vendor wrapper package or emergency service is absent, the proxy is
  removed and the Foxglove control is unavailable. If the vendor endpoint drops
  during a call or its response cannot be validated, the callback fails closed
  and reports that success is unconfirmed; inspect the physical robot and its
  emergency state before any further action.

## What the layout shows

- **3D (left):** live A3 URDF and TF plus the standard table illustration in
  HOPE `world`. The A3 appears only when a connected live TF path exists.
- **NTP offset (upper right):** A3 system-clock offset and root dispersion in
  milliseconds.
- **ROS timestamp latency (upper right):** A3 ROS clock minus the pelvis-IMU
  message header timestamp, in milliseconds.
- **CPU utilization (upper right):** aggregate A3 CPU busy percentage over
  time, displayed on a fixed 0%–100% scale.
- **Status tiles:** HDU `agibot_pm`, connected TF readiness, E-stop backend,
  NTP gate, and timestamp freshness use green/red/orange backgrounds.
- **A3 EMERGENCY STOP:** the sole command control, backed by the assert-only
  proxy described above and enabled only when its live vendor backend exists.

## Notes and limits

- **Security:** the WebSocket has no authentication and now carries a real
  E-stop-only service. Serve it only on the trusted lab subnet; do not
  port-forward it, and do not leave an untrusted Foxglove client connected.
- **Mocap network:** Motive access, NatNet reception, and mocap diagnostics are
  laptop-side responsibilities. The A3 monitor has no mocap host parameter,
  probe, or `/hope/mocap/*` publishers. Do not route the Motive network through
  A3.
- **Battery:** the BMS topic is protobuf-wrapped
  (`ros2_plugin_proto/msg/RosMsgWrapper`) and needs a decoder before
  Foxglove can display fields — planned extension, not in this layout.
- **Clock context:** `/hope/ntp/utc_qualified` uses the same Leap Normal plus
  selected-source rule as `timesync.sh --preflight`.
  `/hope/ntp/gate_pass` additionally applies the documented offset and skew
  limits (see
  [agibot/ntp_sync/README.md](../agibot/ntp_sync/README.md)). The monitor never
  modifies clock state; its only robot-state mutation is the explicit E-stop
  Trigger service.
