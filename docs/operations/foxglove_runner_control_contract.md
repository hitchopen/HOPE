# Foxglove Runner local Runner control contract

This document defines the robot-local Runner schema-2 control contract.
Startup ELink checks are separate from diagnostic-only runtime observations.
Local tests do not constitute production deployment or hardware safety acceptance.

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

## ELink observations and terminal serve recovery

ELink runtime health is diagnostic-only. The command proxy, keyboard, native
Runner and Planner do not require telemetry contracts, clock qualification or
an ELink lease to execute normal actions. Existing Runner role, mode,
command-fault, joint-limit and phase-aware recovery checks remain authoritative.
Standalone, simulation, periodic swings and same-flight target streaming retain
their existing behavior. Managed startup still uses its existing PASSIVE mode.

Managed startup separately checks passive process/listener state and known
current-session failure logs; a failed startup check is retryable. Runtime
diagnostic failures do not change modes or create permanent motion lockouts.
See [ELink startup checks and runtime diagnostics](foxglove_lifecycle.md#elink-startup-checks-and-runtime-diagnostics)
for commissioning, BMS diagnostics and the limits of this mitigation.

Terminal serve FAULT/ABORTED recovery is separate from normal COMPLETE handoff.
An operator Stand request in a terminal serve state reports
`SERVE_RECOVERY_PENDING` (reason 34); it does not bypass sample validation on
the action worker. The command thread accepts PD_STAND recovery only after the
serve controller validates the current sample, preserving the existing entry
blend (default 0.5 seconds) and the controller's fault record. Persistent invalid
samples cannot advance recovery, a concurrent PASSIVE request is not undone,
and recovery never resumes MOTION. Transition results are checked and terminal
events are edge-reported rather than logged on every control tick. This does
not add an ELink or command-fault veto to the existing Stand recovery action.
Command continuity tests are not hardware qualification of the recovery posture.

## Internal transport

The browser cannot publish either internal topic. The HDU command proxy and
MDU Runner exchange fixed `std_msgs/Float64MultiArray` schemas over the
existing AimRT ROS 2 backend:

- request: `/hope/runner/control_request_flat`;
- state: `/hope/runner/state_flat`.

Request schema 2 contains exactly four doubles:

```text
[schema=2, request_id, action_code, reserved=0]
```

The current proxy action codes are (wire code 6 remains keyboard-only
`ENTER_SHADOW`; code 11 is a retired loading-zone confirmation):

| Code | Action |
|---:|---|
| 1 | `SET_SERVER` |
| 2 | `SET_RECEIVER` |
| 3 | `ENTER_PD_STAND` |
| 4 | `ENTER_MOTION` |
| 5 | `EMERGENCY_PASSIVE` |
| 7 | `PREPARE_SERVE` |
| 8 | `CONFIRM_BALL_LOADED` (compatibility action; current gripper workflow may reject it) |
| 9 | `READY_TO_SERVE` |
| 10 | `OPEN_GRIPPER` |
| 12 | `CONFIRM_GRIP_SECURE` |
| 13 | `ENTER_TELEOP` |

State schema 2 contains exactly 21 doubles:

```text
schema, boot_id, state_sequence, run_mode,
command_publishing, policy_native, command_fault_latched,
local_role, role_epoch, role_change_allowed,
role_last_result, role_last_reason,
serve_capability, serve_state, gripper_state, serve_cleanup_required,
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

The separate bridge on port 8766 allowlists fixed services; its Runner-related
actions include:

```text
/hope/runner/set_server
/hope/runner/set_receiver
/hope/runner/enter_pd_stand
/hope/runner/enter_motion
/hope/runner/emergency_passive
/hope/runner/enter_teleop
/hope/runner/prepare_serve
/hope/runner/confirm_ball_loaded
/hope/runner/confirm_grip_secure
/hope/runner/ready_to_serve
/hope/runner/open_gripper
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
requests use only a qualified phase-aware abort and may return
`ACCEPTED_PENDING`; they are not a blanket stand override. During active serve
phases motion requests are rejected. Successful serve completion retains the
existing automatic handoff to MOTION; no extra Continue Receive action is required.
Emergency passive remains the explicit zero-gain escape, not an automatic
health-loss response or a safe weight-bearing stop.

`ENTER_MOTION` retains the existing role/mode/controller checks, without a new
ELink gate. It does not itself recompute x_hit or validate a
future ball prediction.

The current gripper workflow uses `PREPARE_SERVE`, `CONFIRM_GRIP_SECURE` and
`READY_TO_SERVE`; each is checked against authoritative serve/gripper state,
role, cleanup and gain constraints in `PpRunnerControl`. ELink diagnostics do
not replace those checks, invent gripper feedback or alter the serve trajectory.

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
set_server -> enter_pd_stand -> prepare_serve
-> follow authoritative loading/gripper state -> confirm_grip_secure
-> ready_to_serve
```

The role assignment alone does not start a serve and still does not control the
opponent. Normal abort remains `enter_pd_stand`, which uses the existing
phase-aware serve abort; emergency paths remain separate.

Emergency paths:

- `emergency_passive` is Runner `p`: zero gains and loss of active support;
- `trigger_estop` is the independent assert-only vendor safety path.

Runner/HAL process exit remains a lifecycle-supervisor or attended runbook
operation; there is no remote `quit_runner` service.

## Deployment acceptance boundary

Schema-1/19-double dry-run observations and hashes from earlier binaries do not
qualify this schema-2/21-double contract. Offline state-machine and command
continuity tests establish software behavior, not powered recovery safety.

Before accepting a deployed package, verify its exact hash and schema, all
published action/reason codes, request/ack matching, managed PASSIVE startup,
and the intended Stand/Serve/Ready workflows. Terminal FAULT/ABORTED recovery
needs a separately approved attended hardware test with physical support and
an operator at the E-stop; do not use an unqualified fault-injection test during
normal competition operation. Do not infer full hardware acceptance from an
installation smoke test or from this document.
