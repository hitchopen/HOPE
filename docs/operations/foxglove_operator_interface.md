# Foxglove Runner operator interface

This is the handoff contract for the single robot controlled by our Runner.
Foxglove is an operator UI and calls only fixed services. Runner remains the
authority for robot mode, command state, local role, and serve-controller
state. No interface here connects to or controls the opponent robot.

## Display interfaces

| Operator item | ROS interface | Producer and meaning |
|---|---|---|
| NTP world-clock offset | `/hope/ntp/offset_ms` (`Float64`), `/hope/ntp/text` (`String`) | HDU `chronyc tracking`; signed system-clock offset in ms |
| ROS 2 message latency | `/hope/clock/message_latency_ms` (`Float64`), `/hope/clock/message_text` (`String`) | HDU ROS clock minus pelvis-IMU header stamp in ms |
| `agibot_pm` | `/hope/vendor/agibot_pm_active` (`Bool`) | local HDU systemd unit state |
| HDU active | `/hope/system/hdu_active` (`Bool`) | `hope_monitor` probing the HDU observer systemd unit |
| MDU active | `/hope/system/mdu_active` (`Bool`) | freshness of the MDU Runner authoritative state heartbeat |
| P1 live marker count | `/hope/mocap/p1_marker_count` (`UInt32`), `p1_marker_text` (`String`), `p1_marker_fresh` and `p1_markers_complete` (`Bool`) | Laptop `RigidBodyMarkerArray`; unique physical live samples only, bounded 0–10 |
| Pelvis world XYZ | `/hope/pelvis/pose` (`PoseStamped`), `/hope/pelvis/text` (`String`) | fresh authoritative mocap pose, with live `world -> pelvis_link` TF fallback |
| Pelvis point and label in 3D | `/hope/pelvis/marker` (`visualization_msgs/Marker`) | world-position sphere plus XYZ/RPY text; no `foxglove_msgs` dependency |
| Pelvis link in 3D | `/hope/pelvis/tf` (`tf2_msgs/TFMessage`) | sanitized single `world -> pelvis_link` transform; no other robot frame |
| Ball in 3D | `/hope/ball/marker` (`visualization_msgs/Marker`) | 4 cm sphere from `/poses[0]`, expiring after 0.2 s without tracking |
| Runner/role/serve state | `/hope/runner/**`, `/hope/opponent/**` | our Runner confirmed; opponent expected role is inferred and never confirmed |
| CPU load graph | `/hope/system/cpu_load_percent` (`Float64`), `/hope/system/cpu_top_process` (`String`) | aggregate `/proc/stat` busy percentage plus largest per-process CPU delta |

The marker count does not count Motive model-definition entries or predicted
positions. A marker must have `has_live_sample=true`, not be occluded, be
point-cloud solved, and have a finite position. Stale Laptop input publishes
`0/10 | NO FRESH LAPTOP DATA`, with `p1_marker_fresh=false` so a real live zero
is distinguishable from no data.

The operator layout contains one URL-loaded URDF for the static ping-pong
table, but no robot description or joint-state visualization. Its dynamic 3D
view is limited to the ball marker, the Pelvis world-pose marker, and the
sanitized `world -> pelvis_link`. The WebSocket bridges do not forward raw
`/tf` or `/tf_static`.

`hdu_active` is emitted by the independent fleet monitor, so an observer crash
can produce a real red/false state. If the entire HDU or its network connection
dies, the WebSocket dies too, so Foxglove shows `NO DATA / WS DISCONNECTED`; it
is impossible for the dead HDU to publish a final false.
`mdu_active` means the MDU Runner state is fresh, not merely that the MDU Linux
host answers ping.

## Button interfaces

All calls use `std_srvs/Trigger`; request payload editing is disabled.

| Button | Service | Exact effect |
|---|---|---|
| Calibration | `/hope/calibrate` | recompute ten-marker `P1 -> pelvis_link`, persist a stationary world-pelvis snapshot, and wait for the matching live base receipt |
| Refresh x_hit | `/hope/refresh_x_hit` | refresh only the current Planner x_hit request/status contract |
| Stand | `/hope/runner/enter_pd_stand` | same Runner transition as keyboard `s` |
| Ready | `/hope/runner/enter_motion` | same Runner transition as keyboard `m`; the UI enables it only after PD_STAND and a fresh HDU-observed Pelvis base, while Runner semantics remain unchanged |
| Ready to Serve | `/hope/runner/ready_to_serve` | starts existing serve pre-position; operator waits for `AWAIT_BALL_ON_PALM` |
| Serve | `/hope/runner/serve` | accepted only at `AWAIT_BALL_ON_PALM`; confirms ball on palm and commits the existing serve |
| Runner Passive | `/hope/runner/emergency_passive` | same zero-gain transition as keyboard `p`; robot loses active support |
| E-Stop | `/hope/safety/trigger_estop` | assert primary vendor stop plus independent managed Runner stop when available; persistent authoritative latch and no Foxglove reset path |
| Our role: Server/Receiver | `/hope/runner/set_server`, `/hope/runner/set_receiver` | changes only our local Runner role in PASSIVE/PD_STAND |

The command proxy creates an exact request ID and waits for a matching Runner
acknowledgement. Foxglove never publishes the internal flat request topic and
never changes displayed mode optimistically.

Calibration is accepted only for a fresh session-matching Runner in
`PD_STAND`. The existing Laptop algorithm fits the fixed `P1 -> pelvis_link`
extrinsic from ten physical markers and CAD, while the base relay composes it
with live `world -> P1`. The service waits until the new receipt ID is visible
in a valid schema-2 base packet before reporting success. Calibration does not
create an x_hit request. The operator uses the separate `Refresh x_hit` button
when desired.

## Laptop marker publisher

Install the Laptop files from `foxglove/laptop/` into the same ROS 2
environment that receives `/optitrack/rigid_body_markers`. The supplied unit
uses ROS domain 232; adjust deployment paths for the Laptop environment, then
start it only after the mocap bridge is available. The publisher has no
service, process-management, or control API.

The supported lifecycle runs ROS Jazzy inside the `hope` Distrobox. Install the
three marker files below into the operator account; the fixed lifecycle helper
starts the node inside the same Distrobox/Fast DDS context as the OptiTrack
bridge and stops it with that managed session. Use the complete commands in
`docs/operations/foxglove_first_hardware_test.md`. On a new Laptop, install
Distrobox and create the ROS-equipped `hope` environment through
[`DISTROBOX_SETUP.md`](../DISTROBOX_SETUP.md) first.

```bash
install -D -m 0755 \
  foxglove/laptop/hope_marker_monitor.py \
  "$HOME/.local/share/hope-foxglove/hope_marker_monitor.py"
install -D -m 0644 \
  foxglove/laptop/hope_marker_monitor_core.py \
  "$HOME/.local/share/hope-foxglove/hope_marker_monitor_core.py"
install -D -m 0644 \
  foxglove/laptop/marker_monitor.yaml \
  "$HOME/.local/share/hope-foxglove/marker_monitor.yaml"
```

## Foxglove connection

Build and install `foxglove/extensions/hope-a3-console`, use the opt-in control
endpoint `ws://<HDU-IP>:8766`, and import
`foxglove/layouts/model21800_console.json`. The custom panel implements the
new desktop design and includes all display and button interfaces above. The
endpoint still has no client topic publish, parameter mutation, wildcard
service, shell, argv, PID, or arbitrary signal capability.

The static table and rewritten robot mesh URLs use the existing Laptop asset
server. Start it from the repository before opening the 3D panel:

```bash
cd foxglove && python3 -m http.server 8000
```

The fixed-lifecycle runbook instead installs
`foxglove/laptop/hope-foxglove-assets.service` as a user service, so this
manual command is not required after the one-time setup.
