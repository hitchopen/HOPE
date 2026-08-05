# A3 Foxglove Monitoring — Fleet Rulebook

Rules and procedures for monitoring **any** Agibot A3 unit from a laptop on
the same subnet: TF tree and URDF model in 3D, a live text readout of the
`pelvis_link` position and rotation, real-time NTP sync quality, and
real-time network latency between the robot and the mocap (Motive) host.

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
- **R2 — Read-only.** The bridge does not advertise the `clientPublish`,
  parameter, service, or asset capabilities. Its client, service, and parameter
  allowlists use a match-nothing expression. The layout contains no publish
  controls. The monitor publishes diagnostics and a visualization-only merged
  `/joint_states`; it publishes no robot command topic.
- **R3 — Additive only.** Two dedicated systemd units; no vendor file is
  modified. Disabling the units removes their runtime effect; the documented
  removal procedure separately cleans installed and build artifacts.
- **R4 — Bandwidth is opt-in.** The topic whitelist excludes H.265 camera
  and lidar streams by default. Add them per-robot, deliberately.
- **R5 — Verify per unit before trusting.** Fleet defaults below were
  verified live on one A3 (2026-08-04). Robots on different vendor software
  versions must pass the per-unit verification checklist before the layout
  is trusted.

## Fleet defaults (verify per unit)

| Item | Fleet default | Verify with |
|---|---|---|
| ROS 2 distribution | Jazzy at `/opt/ros/jazzy` | `ls /opt/ros` |
| DDS domain | `ROS_DOMAIN_ID=232` | `grep -r ROS_DOMAIN_ID /opt/agibot/entry/env/` |
| DDS interface whitelist | `10.42.10.10` only | vendor `ros_dds_configuration.xml` |
| TF topics | `/tf`, `/tf_static` (`tf2_msgs/TFMessage`) | `ros2 topic type /tf` |
| Odometry | `/agivslam/localization/odometry` (`nav_msgs/Odometry`) | `ros2 topic list` |
| Joint states | `/motion/control/{leg,arm,hand,neck,waist}_joint_state` | `ros2 topic list` |
| URDF location | exported in vendor process env (`AGIBOT_ROBOT_BODY_URDF_*`) | see URDF section |
| chrony | installed per `agibot/ntp_sync` | `chronyc tracking` |

```text
A3 unit                                          Laptop (same subnet)
  AimRT apps ⇄ iceoryx (local SHM)
            ⇄ ROS2/FastDDS domain 232  ──┐
  hope_monitor.py (NTP, mocap RTT,   ────┤
    /joint_states, pelvis pose text)     │
  foxglove_bridge ───────────────────────┴── ws://<robot-ip>:8765 ──► Foxglove Desktop
```

## Folder contents

```text
foxglove/
|-- README.md                        this rulebook
|-- layouts/
|   `-- a3_monitor.json              Foxglove layout (import on the laptop)
`-- a3/                              staged robot-side assets (NOT installed)
    |-- build_foxglove_bridge.sh     pinned ROS 2 bridge build vs /opt/ros/jazzy
    |-- bridge_params.yaml           port, address, topic whitelist
    |-- fastdds_bridge_profile.xml   UDPv4-only profile matching the vendor whitelist
    |-- hope-foxglove-bridge.service systemd unit (bridge)
    |-- hope_monitor.py              ROS node for NTP / RTT / joints / pelvis pose
    |-- hope_monitor_core.py         ROS-free probe parsing and health rules
    `-- hope-monitor.service         systemd unit (monitor node)
```

## Per-robot installation (operator action; changes that robot)

Set the target once per session; every command below uses it:

```bash
export A3_HOST=<robot-ip-or-hostname>
```

### 1. Stage and build foxglove_bridge (once per robot)

No compatible Jazzy apt repository is assumed on the vendor Debian 12 image.
The script builds the official ROS 2 implementation from the pinned Foxglove
`ros-v3.4.3` release (`05f27efc7e535d9c30c6b0cb4f6aa89de7243870`) with remote
access disabled. The robot must already carry `g++`, `cmake`, `colcon`, and the
Jazzy development tree:

```bash
ssh "agi@$A3_HOST" 'mkdir -p ~/foxglove_a3'
scp -r foxglove/a3/. "agi@$A3_HOST:~/foxglove_a3/"
ssh "agi@$A3_HOST" 'sudo apt-get install --no-install-recommends -y \
  git ca-certificates libssl-dev zlib1g-dev && \
  bash ~/foxglove_a3/build_foxglove_bridge.sh'
```

Result on the robot:
`~/hope_foxglove_ws/foxglove-sdk/ros/install/foxglove_bridge/`.
The build stops if an existing source checkout does not match the pinned
commit; it never silently replaces or updates a checkout.

### 2. Install the two services

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

# Per-robot parameters live in ONE file: the monitor unit.
# Set the mocap host, and the pelvis/reference frame names if this unit's
# TF tree differs from the fleet default:
sudo sed -i 's/REPLACE_WITH_MOCAP_HOST/<motive-host-ip>/' \
  /etc/systemd/system/hope-monitor.service

sudo systemctl daemon-reload
sudo systemctl enable --now hope-foxglove-bridge.service hope-monitor.service
systemctl status hope-foxglove-bridge hope-monitor --no-pager
```

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
the 3D panel; `/hope/ntp/offset_ms` and `/hope/mocap/rtt_ms` plot live values;
the time and joint indicators are green; `/hope/pelvis/text` shows a pose, not
a TF error message; and camera topics and publish controls are absent. A red
joint indicator means at least one configured vendor group is missing or older
than 0.5 seconds, so the last displayed robot pose must not be treated as live.

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

## Robot model (URDF) and the pelvis readout

Each robot carries its own URDF; the vendor exports its location in the
process environment. Fetch it from the target unit (read-only):

```bash
ssh "agi@$A3_HOST" \
  'sudo cat /proc/$(pgrep -f run_agibot | head -1)/environ | tr "\0" "\n" | grep URDF'
scp -r "agi@$A3_HOST:<URDF_DIR>" ./foxglove/urdf/    # per-unit; not committed
```

Serve it to Foxglove from the laptop:

```bash
cd foxglove/urdf && python3 -m http.server 8000
```

In the 3D panel's **A3 URDF layer**, replace the placeholder with the URDF's
actual path relative to `foxglove/urdf`, for example
`http://localhost:8000/a3_description/urdf/model.urdf`. If the URDF uses
`package://a3_description/...` mesh paths, make an HTTP-specific copy and
replace that exact package prefix while preserving the package directory:

```bash
cp foxglove/urdf/a3_description/urdf/model.urdf \
  foxglove/urdf/a3_description/urdf/model.http.urdf
sed -i '' \
  's#package://a3_description/#http://localhost:8000/a3_description/#g' \
  foxglove/urdf/a3_description/urdf/model.http.urdf
```

Then point the layer at `model.http.urdf`. Substitute the unit's real package
and file names; do not use a wildcard replacement that discards the package
directory.

**Pelvis position and rotation in text:** `hope_monitor.py` looks up the
transform `reference_frame -> pelvis_frame` (fleet defaults `odom ->
pelvis_link`, both settable in `hope-monitor.service`) and publishes:

- `/hope/pelvis/pose` (`geometry_msgs/PoseStamped`) — structured values,
  shown numerically in the layout's **Pelvis pose** panel.
- `/hope/pelvis/text` (`std_msgs/String`) — one human-readable line:
  position in meters, quaternion, and roll/pitch/yaw in degrees. If the TF
  lookup fails, this topic carries the error text instead, so a wrong frame
  name is visible directly in the UI.

The monitor caches the five vendor joint groups but publishes the merged
`/joint_states` at a bounded 20 Hz. It publishes only while every group is
fresh, preserves the oldest contributing source timestamp, and exposes group
health on `/hope/joints/fresh` and `/hope/joints/text`. The rate and freshness
window are configurable in `hope-monitor.service`.

To discover a unit's actual frame names, open the 3D panel's frame list, or
run on the robot: `ros2 topic echo /tf_static --once | grep frame_id`.

## What the layout shows

- **3D (left):** TF tree with axes and labels, URDF model layer, SLAM
  odometry arrows. Follow-mode tracks the robot root frame.
- **Pelvis pose (under the 3D view):** live numeric position/orientation of
  `pelvis_link`, plus the formatted text line from `/hope/pelvis/text`.
- **NTP sync (top right):** `offset_ms` + `root_dispersion_ms` and
  `skew_ppm` plots from chrony. **MOCAP TIME READY** requires Leap Normal, a
  currently selected `^*` chrony source, |offset| ≤ 10 ms, and skew ≤ 5 ppm.
- **Mocap link (bottom right):** round-trip network latency from the robot
  to the Motive host (1 Hz ICMP probe) and a reachability indicator. This is
  *network* latency only; end-to-end capture latency belongs to the
  wired-relay design in [docs/OPTITRACK.md](../docs/OPTITRACK.md) and the
  [clock plan](../docs/HOPE_A3_Clock_Synchronization_Improvement_Plan.pdf).
- **Joint freshness (bottom right):** green only when all five configured
  vendor joint-state groups have arrived within the freshness window.

## Notes and limits

- **Security:** the WebSocket has no authentication. Serve it only on the
  trusted lab subnet; do not port-forward it.
- **Windows firewall:** the Motive host must answer ICMP echo (allow "File
  and Printer Sharing (Echo Request)"), or the RTT plot shows
  `reachable=false`.
- **Battery:** the BMS topic is protobuf-wrapped
  (`ros2_plugin_proto/msg/RosMsgWrapper`) and needs a decoder before
  Foxglove can display fields — planned extension, not in this layout.
- **Clock context:** `/hope/ntp/utc_qualified` uses the same Leap Normal plus
  selected-source rule as `timesync.sh --preflight`.
  `/hope/ntp/gate_pass` additionally applies the documented offset and skew
  limits (see
  [agibot/ntp_sync/README.md](../agibot/ntp_sync/README.md)). The monitor
  never modifies clock state.
