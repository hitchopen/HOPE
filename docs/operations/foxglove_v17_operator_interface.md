# Foxglove V17 operator interface

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
| HDU active | `/hope/v17/system/hdu_active` (`Bool`) | `hope_monitor` probing the HDU observer systemd unit |
| MDU active | `/hope/v17/system/mdu_active` (`Bool`) | freshness of the MDU Runner authoritative state heartbeat |
| P1 live marker count | `/hope/v17/mocap/p1_marker_count` (`UInt32`), `p1_marker_text` (`String`), `p1_marker_fresh` and `p1_markers_complete` (`Bool`) | Laptop `RigidBodyMarkerArray`; unique physical live samples only, bounded 0–10 |
| Pelvis world XYZ | `/hope/pelvis/pose` (`PoseStamped`), `/hope/pelvis/text` (`String`) | fresh `world -> pelvis_link` vendor TF |
| Pelvis label in 3D | `/hope/pelvis/scene` (`foxglove_msgs/SceneUpdate`) | frame-locked XYZ/RPY text attached to `pelvis_link` |
| Standard table | static `hope_ping_pong_table.urdf` layer | fixed HOPE-world table geometry |
| Robot URDF and TF tree | `/hope/robot_description`, `/tf`, `/tf_static`, `/joint_states` | TF-gated A3 URDF driven by vendor live transforms/joints |
| Runner/role/serve state | `/hope/v17/runner/**`, `/hope/v17/opponent/**` | our Runner confirmed; opponent expected role is inferred and never confirmed |
| CPU load graph | `/hope/system/cpu_load_percent` (`Float64`) | aggregate `/proc/stat` busy percentage, 0–100 |

The marker count does not count Motive model-definition entries or predicted
positions. A marker must have `has_live_sample=true`, not be occluded, be
point-cloud solved, and have a finite position. Stale Laptop input publishes
`0/10 | NO FRESH LAPTOP DATA`, with `p1_marker_fresh=false` so a real live zero
is distinguishable from no data.

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
| Calibration | `/hope/v17/refresh_x_hit` | existing Planner request/status x_hit refresh |
| Stand | `/hope/v17/runner/enter_pd_stand` | same Runner transition as keyboard `s` |
| Ready | `/hope/v17/runner/enter_motion` | same Runner transition as keyboard `m`; no new hidden gate |
| Ready to Serve | `/hope/v17/runner/ready_to_serve` | starts existing serve pre-position; operator waits for `AWAIT_BALL_ON_PALM` |
| Serve | `/hope/v17/runner/serve` | accepted only at `AWAIT_BALL_ON_PALM`; confirms ball on palm and commits the existing serve |
| Runner Passive | `/hope/v17/runner/emergency_passive` | same zero-gain transition as keyboard `p`; robot loses active support |
| E-Stop | `/hope/safety/trigger_estop` | assert-only vendor software E-stop; no Foxglove reset path |
| Our role: Server/Receiver | `/hope/v17/runner/set_server`, `/hope/v17/runner/set_receiver` | changes only our local Runner role in PASSIVE/PD_STAND |

The command proxy creates an exact request ID and waits for a matching Runner
acknowledgement. Foxglove never publishes the internal flat request topic and
never changes displayed mode optimistically.

## Laptop marker publisher

Install the Laptop files from `foxglove/v17/laptop/` into the same ROS 2
environment that receives `/optitrack/rigid_body_markers`. The supplied unit
uses ROS domain 232; adjust deployment paths for the Laptop environment, then
start it only after the mocap bridge is available. The publisher has no
service, process-management, or control API.

```bash
sudo install -D -m 0755 \
  foxglove/v17/laptop/hope_v17_marker_monitor.py \
  /usr/local/bin/hope_v17_marker_monitor.py
sudo install -D -m 0644 \
  foxglove/v17/laptop/hope_v17_marker_monitor_core.py \
  /usr/local/lib/hope-v17-foxglove/hope_v17_marker_monitor_core.py
sudo install -D -m 0644 \
  foxglove/v17/laptop/v17_marker_monitor.yaml \
  /etc/hope-foxglove/v17_marker_monitor.yaml
sudo install -D -m 0644 \
  foxglove/v17/laptop/hope-v17-marker-monitor.service \
  /etc/systemd/system/hope-v17-marker-monitor.service
sudo systemctl daemon-reload
sudo systemctl enable --now hope-v17-marker-monitor.service
```

## Foxglove connection

Build and install `foxglove/extensions/hope-a3-console`, use the opt-in control
endpoint `ws://<HDU-IP>:8766`, and import
`foxglove/layouts/v17_model21800_console.json`. The custom panel implements the
new desktop design and includes all display and button interfaces above. The
endpoint still has no client topic publish, parameter mutation, wildcard
service, shell, argv, PID, or arbitrary signal capability.

The static table and rewritten robot mesh URLs use the existing Laptop asset
server. Start it from the repository before opening the 3D panel:

```bash
cd foxglove && python3 -m http.server 8000
```
