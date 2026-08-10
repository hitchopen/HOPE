# model_21800 Foxglove trial layer

This directory is separate from the fleet-generic `foxglove/a3/` baseline.
Phase 1 is read-only: it adds no bridge capability and no remotely callable
application service. ROS 2 Jazzy's automatic type-description endpoint is not
matched by the bridge allowlist. See
[`docs/operations/foxglove_v17_model21800_plan.md`](../../docs/operations/foxglove_v17_model21800_plan.md)
for the command migration and safety plan.
The implemented single-robot boundary and frozen wire are documented in
[`docs/operations/foxglove_v17_local_runner_control_contract.md`](../../docs/operations/foxglove_v17_local_runner_control_contract.md).

## Files

```text
foxglove/v17/
|-- README.md
|-- a3/
    |-- bridge_params_v17_control.yaml
    |-- hope-foxglove-v17-control-bridge.service
    |-- hope-v17-command-proxy.service
    |-- hope_v17_observer.py       ROS node; publishes /hope/v17/**
    |-- hope_v17_observer_core.py  ROS-free schema/status parsers
    |-- hope_v17_runner_control_core.py  fixed Runner request/state codec
    |-- hope_v17_command_proxy.py  seven Runner actions + RefreshXHit
    |-- hope_v17_command_core.py   atomic request/status file contract
    `-- hope-v17-observer.service  optional HDU systemd unit
`-- laptop/
    |-- hope_v17_marker_monitor.py       live physical P1 marker count
    |-- hope_v17_marker_monitor_core.py  ROS-free counting rules
    |-- v17_marker_monitor.yaml
    `-- hope-v17-marker-monitor.service  optional Laptop systemd template
```

The observer reads:

- `/a3/base_pose_flat` (16-double schema-2);
- `/racket/command_flat` (19-double schema-2);
- `/poses` callback timing;
- `/hope/v17/runner/state_flat` (19-double schema-1, local Runner authority);
- `/tmp/hope_model21800_session_id`;
- `/tmp/hope_real/<session>/hdu/{current_planner_attempt,x_hit.status,...}`;
- the selected Planner PID's `/proc/<pid>/cmdline`.

It writes no audit/runtime files and sends no signals. It publishes only
standard ROS messages under `/hope/v17/**`, already matched by the fleet
bridge's existing read-only topic allowlist.

## Optional HDU install

Run only after the fleet Foxglove bridge/profile is already installed and the
unit's internal HDU interface has been verified. These commands change the
target robot; they are not run automatically by this repository.

```bash
export A3_HOST=<robot-ip-or-hostname>

ssh "agi@$A3_HOST" 'mkdir -p ~/foxglove_v17_a3'
scp -r foxglove/v17/a3/. "agi@$A3_HOST:~/foxglove_v17_a3/"
ssh -tt "agi@$A3_HOST"

sudo install -D -o root -g root -m 0755 \
  ~/foxglove_v17_a3/hope_v17_observer.py \
  /usr/local/bin/hope_v17_observer.py
sudo install -D -o root -g root -m 0644 \
  ~/foxglove_v17_a3/hope_v17_observer_core.py \
  /usr/local/lib/hope-v17-foxglove/hope_v17_observer_core.py
sudo install -D -o root -g root -m 0644 \
  ~/foxglove_v17_a3/hope_v17_runner_control_core.py \
  /usr/local/lib/hope-v17-foxglove/hope_v17_runner_control_core.py
sudo install -D -o root -g root -m 0644 \
  ~/foxglove_v17_a3/hope-v17-observer.service \
  /etc/systemd/system/hope-v17-observer.service
sudo systemctl daemon-reload
sudo systemctl enable --now hope-v17-observer.service
systemctl status hope-v17-observer.service --no-pager
```

No `bridge_params.yaml` replacement is required for Phase 1. Import
`foxglove/layouts/v17_model21800_observer.json` in Foxglove Desktop while
connected to `ws://<robot-ip>:8765`.

Read-only verification on the HDU:

```bash
ros2 topic echo /hope/v17/observer_alive --once
ros2 topic echo /hope/v17/session/text --once
ros2 topic echo /hope/v17/base/summary --once
ros2 topic echo /hope/v17/command/summary --once
ros2 topic echo /hope/v17/x_hit/status --once
ros2 topic echo /hope/v17/runner/summary --once
ros2 topic echo /hope/v17/opponent/expected_role --once
ros2 topic echo /hope/v17/opponent/role_confirmed --once
```

An absent ball, session, Planner attempt, or x_hit status is shown as telemetry;
it does not stop Planner output or prevent the operator from entering MOTION.

## Optional attended local-Runner control endpoint

The command proxy exposes exactly seven local-Runner actions plus the Planner's
existing `RefreshXHit` action:

- `/hope/v17/runner/set_server` and `set_receiver` update only our Runner's
  role context;
- `/hope/v17/runner/enter_pd_stand`, `enter_motion`, and
  `emergency_passive` use the same Runner transition logic as `s`, `m`, and
  `p`;
- `/hope/v17/runner/ready_to_serve` starts the existing serve pre-position,
  and `/hope/v17/runner/serve` confirms the ball only at the real
  `AWAIT_BALL_ON_PALM` state;
- `/hope/v17/refresh_x_hit` keeps using the Planner's existing
  `x_hit.request`/`x_hit.status` audit contract.

It accepts no shell, argv, path, PID, signal, arbitrary role/mode, browser
topic publishing, or parameter changes. `RefreshXHit` does not gate or change
Runner mode: the operating order remains **stable PD_STAND first, then refresh
x_hit, then the operator decides whether to enter MOTION**. `local_role` does
not gate MOTION, and assigning SERVER alone does not start a serve.

The control endpoint is a second bridge on port `8766`; the fleet bridge on
`8765` and its E-stop-only allowlist stay unchanged. Stage the same folder as
above, then install but do not enable the opt-in units:

```bash
sudo install -D -o root -g root -m 0755 \
  ~/foxglove_v17_a3/hope_v17_command_proxy.py \
  /usr/local/bin/hope_v17_command_proxy.py
sudo install -D -o root -g root -m 0644 \
  ~/foxglove_v17_a3/hope_v17_command_core.py \
  /usr/local/lib/hope-v17-foxglove/hope_v17_command_core.py
sudo install -D -o root -g root -m 0644 \
  ~/foxglove_v17_a3/hope_v17_runner_control_core.py \
  /usr/local/lib/hope-v17-foxglove/hope_v17_runner_control_core.py
sudo install -D -o root -g root -m 0644 \
  ~/foxglove_v17_a3/bridge_params_v17_control.yaml \
  /etc/hope-foxglove/v17_control_bridge_params.yaml
sudo install -D -o root -g root -m 0644 \
  ~/foxglove_v17_a3/hope-v17-command-proxy.service \
  /etc/systemd/system/hope-v17-command-proxy.service
sudo install -D -o root -g root -m 0644 \
  ~/foxglove_v17_a3/hope-foxglove-v17-control-bridge.service \
  /etc/systemd/system/hope-foxglove-v17-control-bridge.service
sudo systemctl daemon-reload
```

For an attended trial, explicitly open the control endpoint:

```bash
sudo systemctl start \
  hope-v17-command-proxy.service \
  hope-foxglove-v17-control-bridge.service
systemctl status \
  hope-v17-command-proxy.service \
  hope-foxglove-v17-control-bridge.service \
  --no-pager
```

Connect a Foxglove window to `ws://<robot-ip>:8766` and import
`foxglove/layouts/v17_model21800_control_phase2.json`. The panel labels local
role as Runner-confirmed. The opponent expected role is always derived from
that local role, its source is `INFERRED_FROM_LOCAL_ROLE`, and
`/hope/v17/opponent/role_confirmed` is always false. The two red actions are
different safety paths: Runner PASSIVE loses active support, while vendor
E-stop remains the real assert-only vendor path and must not be used as a
smoke test.

The same layout also consumes the fleet monitor's NTP/ROS latency, agibot_pm,
Pelvis TF/URDF, and CPU topics. The Laptop marker node publishes the live P1
physical-marker count as standard `/hope/v17/mocap/**` messages; see
[`docs/operations/foxglove_v17_operator_interface.md`](../../docs/operations/foxglove_v17_operator_interface.md)
for the complete 16-item interface and precise semantics.

Close the opt-in endpoint after the attended trial:

```bash
sudo systemctl stop \
  hope-foxglove-v17-control-bridge.service \
  hope-v17-command-proxy.service
```

To remove only this optional layer:

```bash
sudo systemctl disable --now hope-v17-observer.service
sudo systemctl disable --now \
  hope-foxglove-v17-control-bridge.service \
  hope-v17-command-proxy.service
sudo unlink /etc/systemd/system/hope-v17-observer.service
sudo unlink /etc/systemd/system/hope-v17-command-proxy.service
sudo unlink /etc/systemd/system/hope-foxglove-v17-control-bridge.service
sudo unlink /usr/local/bin/hope_v17_observer.py
sudo unlink /usr/local/bin/hope_v17_command_proxy.py
sudo unlink /usr/local/lib/hope-v17-foxglove/hope_v17_observer_core.py
sudo unlink /usr/local/lib/hope-v17-foxglove/hope_v17_command_core.py
sudo unlink /usr/local/lib/hope-v17-foxglove/hope_v17_runner_control_core.py
sudo unlink /etc/hope-foxglove/v17_control_bridge_params.yaml
sudo rmdir --ignore-fail-on-non-empty /usr/local/lib/hope-v17-foxglove
sudo systemctl daemon-reload
```
