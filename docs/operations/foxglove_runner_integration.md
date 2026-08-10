# Foxglove + native Runner integration

This integration combines three sources without conflating their authority:

- `Catrunaround/HOPE:nightly_built` commit `942f1e79` supplies the fleet
  monitor, OptiTrack physical-marker messages, P1 calibration support,
  TF/URDF display, CPU/NTP/latency telemetry, and vendor assert-only E-stop.
- `runner_interface` commit `d0fe3d40` supplies the authoritative 19-double
  Runner state, fixed request/ack transport, local role, and serve state.
- `Foxglove+Desktop+UI+improvements/design_handoff_a3_console` supplies the
  visual design implemented by `foxglove/extensions/hope-a3-console`.

## Name mapping

The colleague branch's 8-double `/hope/runner/mode_state` and
`/hope/control/*` services belong to its external SSH/tmux adapter. They are
not used by the integrated UI. The browser calls the native Runner endpoints:

| Console action | Integrated service | Runner authority |
|---|---|---|
| Our role: Server | `/hope/v17/runner/set_server` | `local_role=SERVER` |
| Our role: Receiver | `/hope/v17/runner/set_receiver` | `local_role=RECEIVER` |
| Stand | `/hope/v17/runner/enter_pd_stand` | same action queue as `s` |
| Calibration | `/hope/v17/refresh_x_hit` | Planner request/status file contract |
| Ready | `/hope/v17/runner/enter_motion` | same action queue as `m` |
| Ready to Serve | `/hope/v17/runner/ready_to_serve` | real serve controller `Start()` |
| Serve | `/hope/v17/runner/serve` | accepted only at `AWAIT_BALL_ON_PALM` |
| Runner Passive | `/hope/v17/runner/emergency_passive` | same zero-gain action as `p` |
| E-Stop | `/hope/safety/trigger_estop` | assert-only vendor stop plus native Runner passive request |

The recommended receiver order is **Stand, wait for physical stability,
Calibration, then Ready**. The UI shows that order, but Calibration remains
telemetry and does not introduce a hidden Runner admission gate. The server
flow is **set Server, Stand, Ready to Serve, wait for
`AWAIT_BALL_ON_PALM`, then Serve**.

## One Foxglove data source

The console uses the explicitly started control bridge:

```text
ws://<HDU-IP>:8766
```

That bridge carries the colleague monitor topics and the V17 observer topics,
but exposes only the nine fixed Trigger services listed above. Client topic
publishing, parameter mutation, wildcard services, shell, argv, path, PID and
signal inputs remain disabled. Port `8765` stays the fleet monitoring endpoint
with only assert-only E-stop.

All HDU Foxglove/observer units read the same optional
`/etc/hope-foxglove/network.env`. Set `ROS_STATIC_PEERS` there to the Laptop
Wi-Fi address used by the hardware runbook. For the current fixed3 site this is
`172.23.20.46`; do not reuse the colleague snapshot's stale
`172.23.21.67` value after a network change.

The UI topic mapping is:

| UI item | Topic |
|---|---|
| NTP offset/dispersion/gate | `/hope/ntp/{offset_ms,root_dispersion_ms,gate_pass}` |
| ROS timestamp | `/hope/clock/{message_latency_ms,message_fresh}` |
| CPU | `/hope/system/cpu_load_percent` |
| agibot_pm / TF / E-stop backend | `/hope/vendor/agibot_pm_active`, `/hope/vendor/tf_ready`, `/hope/safety/estop_ready` |
| HDU / MDU | `/hope/v17/system/hdu_active`, `/hope/v17/system/mdu_active` |
| physical P1 markers | `/hope/v17/mocap/p1_marker_count`, `/hope/v17/mocap/p1_marker_fresh` |
| Runner/role/serve | `/hope/v17/runner/**` |
| in-scene pelvis text | `/hope/pelvis/scene` (`foxglove_msgs/SceneUpdate`) |

## Desktop install

Build and install the extension on the Laptop:

```bash
cd foxglove/extensions/hope-a3-console
npm install
npm run lint
npm run package
```

Install the generated `.foxe` in Foxglove Desktop, connect to port `8766`,
start the local asset server from `foxglove/`, and import
`foxglove/layouts/v17_model21800_console.json`.

## Lifecycle finding

The inspected colleague commit does **not** implement the complete hardware
runbook lifecycle. Its `hope_model21800_runner.sh` can start an older fixed
`run_a3.sh` in tmux and send `s/m/p`, but it does not start the Laptop bridge,
HDU base relay/Planner, MDU HAL, stop/restore MDU `agibot_pm`, or collect the
three-machine evidence bundle. Those operations therefore remain outside this
Runner/UI adapter until a separately reviewed fixed lifecycle supervisor is
added. The UI never claims those processes were started merely because the
WebSocket or monitor is alive.

## Verification boundary

All repository tests and the extension build are offline checks. Do not invoke
E-stop or powered Runner actions as installation smoke tests. A current AArch64
package and an attended Laptop-HDU-MDU request/ack trial are still required
before treating the console as field-qualified.
