# Foxglove Runner local Runner control contract

Status: implementation contract for the first single-robot control slice.

This interface controls only the Runner on our robot. It neither connects to
nor controls the opponent robot. The Runner remains the sole authority for its
mode, local role, command fault, command-publishing state, and serve-controller
state.

## Authority boundary

- Runner-confirmed: `run_mode`, `local_role`, role epoch, command publishing,
  command fault, serve capability/state, action result/reason, boot id, and
  state sequence.
- HDU Planner-confirmed: `RefreshXHit` result.
- Vendor safety chain-confirmed: assert-only software E-stop.
- Foxglove-derived only: opponent expected role. Its source is always
  `INFERRED_FROM_LOCAL_ROLE` and `role_confirmed` is always `false`.

The derivation is `SERVER -> RECEIVER`, `RECEIVER -> SERVER`, and
`UNASSIGNED -> UNKNOWN`. No claim about the opponent Runner's real state is
made.

## Internal transport

The browser cannot publish either internal topic. The HDU command proxy and
MDU Runner exchange fixed `std_msgs/Float64MultiArray` schemas over the
existing AimRT ROS 2 backend:

- request: `/hope/runner/control_request_flat`;
- state: `/hope/runner/state_flat`.

Request schema 1 contains exactly four doubles:

```text
[schema=1, request_id, action_code, reserved=0]
```

The seven externally accepted action codes are (wire code 6 remains the
keyboard-only `ENTER_SHADOW` and is rejected on remote requests):

| Code | Action |
|---:|---|
| 1 | `SET_SERVER` |
| 2 | `SET_RECEIVER` |
| 3 | `ENTER_PD_STAND` |
| 4 | `ENTER_MOTION` |
| 5 | `EMERGENCY_PASSIVE` |
| 7 | `READY_TO_SERVE` |
| 8 | `SERVE` |

State schema 1 contains exactly 19 doubles:

```text
schema, boot_id, state_sequence, run_mode,
command_publishing, policy_native, command_fault_latched,
local_role, role_epoch, role_change_allowed,
role_last_result, role_last_reason,
serve_capability, serve_state,
last_action_id, last_action, last_action_result, last_action_reason,
session_fingerprint
```

`session_fingerprint` only associates this compact state with the existing
human-readable session id. The observer publishes `runner/session_matches` and
does not invent a session id when it does not match.

`serve_state` is the numeric value of the real `PpServeController` enum. It is
`-1` only when no serve controller is loaded; Foxglove does not synthesize
serve phases.

## Browser-facing services

The separate bridge on port 8766 allowlists only:

```text
/hope/runner/set_server
/hope/runner/set_receiver
/hope/runner/enter_pd_stand
/hope/runner/enter_motion
/hope/runner/emergency_passive
/hope/runner/ready_to_serve
/hope/runner/serve
/hope/calibrate
/hope/refresh_x_hit
/hope/safety/trigger_estop
```

All services use `std_srvs/Trigger`. The proxy generates a request id, sends
one fixed action, then waits for a Runner state whose `last_action_id` matches.
It never optimistically changes displayed state.

Client topic publishing, parameter access, wildcard services, shell, argv,
paths, PIDs, signals, arbitrary role strings, remote Runner quit, and generic
serve payloads are not exposed.

## Transition rules

Role changes are allowed only in `PASSIVE` or `PD_STAND` and while the Runner
command fault is clear. A repeated assignment is `ALREADY_SET` and does not
advance `role_epoch`. Role changes never change mode, refresh x_hit, start
Planner/HAL, start SERVE, or alter `q_des`.

`ENTER_PD_STAND`, `ENTER_MOTION`, and `EMERGENCY_PASSIVE` share the same Runner
transition logic as keyboard `s`, `m`, and `p`. If SERVE owns `q_des`, stand
requests reuse the existing phase-aware abort and return `ACCEPTED_PENDING`;
motion requests are rejected; emergency passive remains the zero-gain escape.

`ENTER_MOTION` deliberately retains the current `m` semantics. It adds no role,
x_hit, Planner READY, localization, or clock gate.

`READY_TO_SERVE` and `SERVE` reuse the existing two-stage keyboard `v`
contract. `READY_TO_SERVE` calls `PpServeController::Start()` only when a serve
controller is loaded, the Runner command fault is clear, no serve is active,
and arm/leg/ankle gain scales resolve to exactly 1.0. It changes the Runner to
`SERVE` and begins the existing pre-position sequence. `SERVE` calls
`ConfirmBallOnPalm()` only when the authoritative controller state is exactly
`AWAIT_BALL_ON_PALM`; all other phases are rejected. No new trajectory or
serve state machine is introduced.

## Operator workflows

Receiver:

```text
set_receiver -> enter_pd_stand -> operator confirms stable stand
-> calibrate (new world-to-pelvis JSON receipt)
-> refresh_x_hit (separate Planner operation) -> enter_motion
-> enter_pd_stand for normal stop
```

Server:

```text
set_server -> local_role=SERVER -> enter_pd_stand
-> ready_to_serve -> wait until serve_state=AWAIT_BALL_ON_PALM
-> operator places ball on rigid palm -> serve
```

The role assignment alone does not start a serve and still does not control the
opponent. Normal abort remains `enter_pd_stand`, which uses the existing
phase-aware serve abort; emergency paths remain separate.

Emergency paths:

- `emergency_passive` is Runner `p`: zero gains and loss of active support;
- `trigger_estop` is the independent assert-only vendor safety path.

Runner/HAL process exit remains a lifecycle-supervisor or attended runbook
operation; there is no remote `quit_runner` service.

## `nightly_built` verification boundary (2026-08-11)

- `python3 -m unittest discover -s foxglove/tests -p 'test_*.py'`: 54 tests
  passed.
- The x86 Runner and test binary built successfully; all 11
  `PpRunnerControl` tests passed.
- The Rockchip build with AimRT and ROS 2 enabled succeeded. The AArch64
  Runner SHA-256 is
  `b6d0cb132e97656aad8fcb4e5179cb0b95ab34afca072c42267f368739400418`.
- On the MDU, that binary started in `--dry-run --start shadow`, registered
  both fixed Runner topics, received all six body-state inputs, and kept
  command publishing disabled. The HDU read the authoritative 19-double
  state over ROS domain 232 and observed SHADOW, `command_publishing=false`,
  `policy_native=true`, and no command fault.
- No powered mode/action was tested. A repeated HDU reboot interrupted the
  earlier five-action request/ack field test, and the two new serve services
  have only offline state-machine coverage. All seven services still require
  an attended end-to-end action test. The previous Rockchip binary/hash and
  MDU dry-run evidence predate the two serve actions and are not evidence for
  this expanded binary.
