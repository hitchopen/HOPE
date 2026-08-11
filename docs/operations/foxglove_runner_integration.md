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
| Our role: Server | `/hope/runner/set_server` | `local_role=SERVER` |
| Our role: Receiver | `/hope/runner/set_receiver` | `local_role=RECEIVER` |
| Stand | `/hope/runner/enter_pd_stand` | same action queue as `s` |
| Calibration | `/hope/calibrate` | ten-marker `P1 -> pelvis_link`, persisted world-pelvis audit snapshot, then fresh matching base receipt |
| Refresh x_hit | `/hope/refresh_x_hit` | Planner request/status file contract only |
| Ready | `/hope/runner/enter_motion` | same action queue as `m` |
| Ready to Serve | `/hope/runner/ready_to_serve` | real serve controller `Start()` |
| Serve | `/hope/runner/serve` | accepted only at `AWAIT_BALL_ON_PALM` |
| Runner Passive | `/hope/runner/emergency_passive` | same zero-gain action as `p` |
| E-Stop | `/hope/safety/trigger_estop` | assert-only vendor stop plus native Runner passive request |

The recommended receiver order is **Stand, wait for physical stability,
Calibration, Refresh x_hit, then Ready**. Calibration requires the current
managed Runner to remain in fresh `PD_STAND`, replaces the approved marker-to-pelvis receipt,
stores a stationary world-pelvis audit snapshot, and waits for that exact
receipt in the live base stream. The separate x_hit button talks only to the
Planner file contract. Neither operation introduces a hidden Runner MOTION
gate. The server flow is **set Server, Stand, Ready to Serve, wait for
`AWAIT_BALL_ON_PALM`, then Serve**.

That flow is conditional on `serve_capability=AVAILABLE`. The current
model_21800 artifact declares `rally_v14`, while the checked-in serve controller
accepts only its qualified `rally_v17` artifact. Lifecycle startup therefore
preserves the formal fixed3 command without `--serve`; the two serve buttons
remain disabled for the current package instead of bypassing that artifact
check.

## One Foxglove data source

The console uses the restricted control bridge. It is explicitly started for a
legacy attended trial, or kept as an always-available control plane when the
fixed lifecycle supervisor is installed:

```text
ws://<HDU-IP>:8766
```

That bridge carries the colleague monitor topics and the Runner observer topics.
It exposes the fixed Runner/calibration/x_hit/E-stop Trigger services listed above and,
when the optional lifecycle supervisor is installed, three exact lifecycle
services. The lifecycle configuration service uses the standard
`SetParameters` message shape but is a dedicated validator for four complete
IPv4 strings; generic parameter services and the bridge parameter capability
remain disabled. Client topic publishing, wildcard services, shell, argv, path,
PID and signal inputs remain disabled. Port `8765` stays the fleet monitoring
endpoint with only assert-only E-stop.

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
| CPU | `/hope/system/cpu_load_percent`, `/hope/system/cpu_top_process` |
| agibot_pm / TF / E-stop backend | `/hope/vendor/agibot_pm_active`, `/hope/vendor/tf_ready`, `/hope/safety/{estop_ready,estop_latched,estop_text}` |
| HDU / MDU | `/hope/system/hdu_active`, `/hope/system/mdu_active` |
| physical P1 markers | `/hope/mocap/p1_marker_count`, `/hope/mocap/p1_marker_fresh` |
| Runner/role/serve | `/hope/runner/**` |
| Pelvis point and in-scene text | `/hope/pelvis/marker` (`visualization_msgs/Marker`) |

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
`foxglove/layouts/model21800_console.json`.

## Lifecycle supervisor

The colleague commit did not contain the complete runbook lifecycle. This
branch therefore adds a separate HDU-resident fixed lifecycle supervisor rather
than reusing its legacy TTY adapter. The custom console can confirm four
validated private IPv4 addresses, start STEP 0/1/2A/2B/4/5, show per-step
progress, then perform the reviewed reverse stop/restore/collection sequence.

The lifecycle surface exposes no shell, argv, path, PID, signal, generic
parameter service or browser publisher. Long-lived processes are launched only
through the root-owned checked-in helper installed at
`/usr/local/libexec/hope-lifecycle`. See
[`foxglove_lifecycle.md`](foxglove_lifecycle.md) for installation,
SSH-key/sudo prerequisites, state semantics and the live verification boundary.

## Verification boundary

All repository tests and the extension build are offline checks. Do not invoke
E-stop or powered Runner actions as installation smoke tests. A current AArch64
package and an attended Laptop-HDU-MDU request/ack trial are still required
before treating the console as field-qualified.
