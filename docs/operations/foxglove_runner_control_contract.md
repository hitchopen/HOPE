# Foxglove and Runner serve025 control contract

Status: implemented and offline-tested; no powered-robot or physical-ball
qualification is claimed.

This contract controls one local A3 Runner. Foxglove is an attended UI, not a
second controller: the same Runner process and the same command publisher own
PD_STAND, the Jeremy serve025 motion, and the rally policy. No endpoint here
connects to or controls the opponent robot.

## Authority boundary

- Runner-confirmed: run mode, local role, role epoch, command publication,
  command fault, serve phase, gripper command phase, cleanup requirement, and
  request acknowledgement.
- Human-confirmed: the ball is physically placed, the hand is outside the
  loading zone, the fingers physically retain the ball, and the area is clear
  before an attended cleanup OPEN.
- Gripper bridge-confirmed: only that an allowlisted command was passed to the
  ROS publisher. Its receipt explicitly reports `physical_confirmation=0` and
  `state_observable=0`; it is never evidence of finger motion or ball state.
- HDU Planner-confirmed: `Refresh x_hit` result.
- Foxglove-derived only: the expected opponent role. `SERVER -> RECEIVER`,
  `RECEIVER -> SERVER`, and `UNASSIGNED -> UNKNOWN`; it is always labelled as
  inference rather than opponent telemetry.

## Frozen wire schema

The browser cannot publish either internal topic. The HDU command proxy and
MDU Runner exchange `std_msgs/Float64MultiArray` on:

- `/hope/runner/control_request_flat`;
- `/hope/runner/state_flat`.

Request schema 2 has exactly four doubles:

```text
[schema=2, request_id, action_code, reserved=0]
```

Twelve actions are exposed remotely. Frozen code 11 remains decode-only:

| Code | Action | Meaning |
|---:|---|---|
| 1 | `SET_SERVER` | assign this Runner as the server |
| 2 | `SET_RECEIVER` | assign this Runner as the receiver |
| 3 | `ENTER_PD_STAND` | enter stand, or request a phase-aware serve abort |
| 4 | `ENTER_MOTION` | receiver-only external policy entry |
| 5 | `EMERGENCY_PASSIVE` | immediate zero-gain Runner escape |
| 7 | `PREPARE_SERVE` | rate-limited ankle-pitch preload plus bounded forward-tilt feedback while automatically running OPEN -> GRAB and raising the serving hand |
| 8 | `CONFIRM_BALL_LOADED` | legacy wire-compatible no-op after PREPARE_SERVE |
| 9 | `READY_TO_SERVE` | play Jeremy's serve025 frame 0 onward |
| 10 | `OPEN_GRIPPER` | attended retained-ball cleanup after abort |
| 11 | `CONFIRM_LOADING_ZONE_CLEAR` | removed; frozen wire code returns `ALREADY_SET/LOADING_ZONE_CLEAR_REMOVED` |
| 12 | `CONFIRM_GRIP_SECURE` | legacy wire-compatible no-op |
| 13 | `ENTER_TELEOP` | Xbox HumanLike teleop from PD_STAND |
| 14 | `ENTER_DOCK` | Reposition: HumanLike walks to the dock.yaml serve pose |
| 15 | `ENTER_WALK_STAND` | HumanLike stand at zero velocity, from PD_STAND |

Wire code 6 remains local keyboard-only `ENTER_SHADOW` and is rejected from a
remote request.

State schema 2 has exactly 21 doubles:

```text
schema, boot_id, state_sequence, run_mode,
command_publishing, policy_native, command_fault_latched,
local_role, role_epoch, role_change_allowed,
role_last_result, role_last_reason,
serve_capability, serve_state, gripper_state, serve_cleanup_required,
last_action_id, last_action, last_action_result, last_action_reason,
session_fingerprint
```

`serve_state` is the real controller enum, not a UI reconstruction:

```text
IDLE -> PREPARING_STAND -> TRANSITION_TO_LOAD -> WAIT_READY_TO_SERVE
     -> PLAYING_PRE_RELEASE -> RELEASE_PENDING -> STRIKE
     -> FOLLOW_THROUGH -> RECOVERY -> HANDOFF_READY -> COMPLETE
```

Abort/fault states are `ABORT_RETURN`, `ABORTED`, `FAULT`, and
`CLEANUP_OPENING`; numeric state 13 is reserved only for old schema decoding.
Gripper states are `UNKNOWN`, `OPENING`,
`OPEN`, `CLOSING`, `GRABBED`, `RELEASING`, `RELEASED`, and `FAULT`.

## Browser services

The restricted bridge on port 8766 exposes these exact Runner Trigger
services:

```text
/hope/runner/set_server
/hope/runner/set_receiver
/hope/runner/enter_pd_stand
/hope/runner/enter_motion
/hope/runner/emergency_passive
/hope/runner/prepare_serve
/hope/runner/confirm_ball_loaded
/hope/runner/confirm_grip_secure
/hope/runner/ready_to_serve
/hope/runner/open_gripper
/hope/runner/enter_teleop
/hope/runner/enter_walk_stand
/hope/runner/enter_dock
```

Calibration, Planner refresh, vendor E-stop, and fixed lifecycle services are
separate allowlisted operations. There is no generic browser publisher,
parameter mutation, shell, argv, path, PID, signal, arbitrary gripper payload,
Runner quit, or opponent-control service.

The proxy assigns a request ID and waits for authoritative Runner state with a
matching `last_action_id`. Foxglove never changes displayed state
optimistically.

## Walk stand (the operator Stand)

`run_mode` 8 is `WALK_STAND`: the Runner's HumanLike locomotion policy (the
Teleop policy, `--teleop-policy-dir`) holds the robot at zero velocity. With
HumanLike loaded it replaces the stiff PD stand for the operator:

- `MOTION` -> `ENTER_PD_STAND` also enters `WALK_STAND` directly, with a
  2.5 s HumanLike leg/arm handover (`PpTeleopEntry::kFromReceiveSeconds`)
  instead of 0.5 s. In MuJoCo (deployed 14200best + HumanLike) the 0.5 s
  handover from the receive ready pose lunged ~15 cm forward at 0.3 m/s;
  2.5 s keeps it at 2-5 cm and 0.05 m/s, like PD_STAND -> HumanLike
  (`scripts/validate_receive_to_humanlike_mujoco.py`).
- `ENTER_PD_STAND` from `PASSIVE`, `PD_STAND`, `SHADOW` or
  `REFERENCE_PLAYBACK` enters `WALK_STAND` directly (0.5 s handover); the HumanLike entry
  blend raises the leg gains from the measured pose in 0.5 s. A hoisted or
  tilted robot is therefore driven by HumanLike: lower it first. With a
  latched command fault, or without HumanLike, Stand is the old `PD_STAND`.
  `ENTER_WALK_STAND` (code 15) still works from `PD_STAND`.
- Field sessions run with `--no-fall-guard` (hope-lifecycle): there is no
  automatic fall response in any mode; operators press PASSIVE. If a Runner
  is started without that flag, the HumanLike fall guard now switches to
  PASSIVE (zero gains) instead of latching a command fault.
- `ENTER_PD_STAND` in `TELEOP` or `DOCK` settles the feet and lands in
  `WALK_STAND`; in `WALK_STAND` it is `ALREADY_SET`. `PD_STAND` reached from
  `MOTION` (Stand, mocap or operator-link recovery) or from a completed serve
  hands back to `WALK_STAND` after the stand blend plus 0.5 s and a quiet
  body. A serve abort/fault recovery stays in the stiff `PD_STAND`.
- `ENTER_MOTION` / `PREPARE_SERVE` from `WALK_STAND` are staged:
  `ACCEPTED_PENDING` (`MODE_CHANGED` / `SERVE_PREPARE_REQUESTED`), HumanLike
  settles, the Runner passes through `PD_STAND`, and after the stand blend
  the command thread re-enqueues the action as a local request with the
  normal admission. A second request while one settles, role changes and
  Reposition are rejected until then; Stand or PASSIVE cancels it.
- Role changes, `ENTER_TELEOP` and `OPEN_GRIPPER` are accepted in
  `WALK_STAND` as in `PD_STAND`; shadow is rejected.
- `EMERGENCY_PASSIVE` is immediate, as in every mode.
- Telemetry is the Teleop stream `/hope/runner/teleop_state_flat` (phase 1
  entering, 2 active, 3 stopping); the Runner keyboard key is `w`.
- Without HumanLike the old behaviour is unchanged (Stand = `PD_STAND`).

## Reposition (auto-dock)

`run_mode` 7 is `DOCK`. The Runner loads it with `--auto-dock --dock-config
dock/dock.yaml` (the lifecycle adds both when the slot has `dock/dock.yaml`
and `teleop_humanlike/`). `PpDockController` is upstream's controller
(Pablo-Davila/HOPE `feat/reposition`); it steers HumanLike to the pelvis pose
in `dock.yaml` using `/a3/base_pose_flat`, so it needs OptiTrack mode.

- `ENTER_DOCK` is accepted from `PD_STAND`, an idle `WALK_STAND` or `TELEOP` with no
  fault, no active serve, a fresh schema-2 base pose with the V17 flags
  (`DOCK_MOCAP_UNAVAILABLE`), and the pelvis within `max_start_distance_m`
  of the target (`DOCK_START_TOO_FAR`). Without `--auto-dock` it is
  `REJECTED_DOCK/DOCK_UNAVAILABLE`.
- From `PD_STAND` it plays the 2 s HumanLike entry blend first; from
  `WALK_STAND` or `TELEOP` it starts walking at once (no policy reset).
- When the walk ends on its own (`DONE`, or `FAILED` on timeout, stale or
  invalid mocap, world-frame change, too many attempts) the Runner waits for
  quiet feet and stays on HumanLike in `WALK_STAND`.
- `ENTER_PD_STAND` during `DOCK` is `ACCEPTED_PENDING/DOCK_STOP_REQUESTED`:
  the walk stops and the Runner settles into `WALK_STAND`.
- Telemetry: `/hope/runner/dock_state_flat`, relayed by the HDU transport
  gateway to `/hope/runner/dock_state_hdu_flat`. Schema 1, 15 doubles:
  schema, boot, phase (0 idle, 1 entry, 2 align, 3 settle, 4 verify, 5 done,
  6 failed), attempt, error x/y/yaw, distance, elapsed s, result (0 none,
  1 running, 2 done, 3 failed), reason (0 none, 1 timeout, 2 mocap stale,
  3 mocap invalid, 4 wrong frame, 5 start too far, 6 attempts exhausted,
  7 stopped), target x/y/yaw, pose age.
- Dock state schema 2 (24 doubles) appends: tuning request seq, tuning result
  (1 applied+saved, 2 invalid, 3 busy, 4 applied not saved), done tolerance m,
  done tolerance rad, forward, backward, lateral m/s, yaw rad/s, max start m.
- Tuning: the Console publishes `/hope/runner/dock_config_hdu_flat` (the only
  other client topic besides operator echo); the HDU gateway forwards it to
  `/hope/runner/dock_config_flat`. Schema 1, 13 doubles: 1, Runner boot,
  request seq (must increase), target x, y, yaw rad, done tolerance m, rad,
  forward, backward, lateral m/s, yaw rad/s, max start m. Envelope: target
  within 5 m, tolerance 2-30 cm / 0.5-20 deg, forward 0.05-0.40, backward and
  lateral 0.05-0.25 m/s, yaw 5-60 deg/s, start 0.1-3 m. Rejected while DOCK.
  An applied request is saved atomically to `dock/dock.yaml` (the first
  original is kept as `dock.yaml.orig`).
- First-hardware `dock.yaml` (2026-11-17): target (-0.50, -0.7625, 0) in
  `table_p1_to_p2_v1` (P1 side only), done within 0.10 m / 5 deg, speeds
  0.20 m/s forward, 0.10 m/s back and sideways, 0.30 rad/s yaw, start cap
  1.5 m, 45 s timeout.

## Admission and role logic

Role change is accepted only in PASSIVE or PD_STAND, with no command fault, no
active serve, and no retained-ball cleanup requirement. Role selection itself
does not move the robot, start the gripper, refresh x_hit, or enter policy.

Receiver flow:

```text
SET_RECEIVER -> ENTER_PD_STAND -> calibrate V2/V3 -> Refresh x_hit
-> ENTER_MOTION
```

External `ENTER_MOTION` is receiver-only and requires PD_STAND. It deliberately
keeps the existing Runner `m` semantics; calibration/base/x_hit freshness are
visible UI preflight signals, not a new policy-output gate in the Runner.

Server flow:

```text
SET_SERVER -> ENTER_PD_STAND -> calibrate V2/V3 -> Refresh x_hit
-> PREPARE_SERVE -> strong stand + automatic OPEN -> GRAB + raise hand
-> WAIT_READY_TO_SERVE -> human places ball in the closed claw and clears area
-> READY_TO_SERVE -> serve motion -> recovery -> internal MOTION handoff
```

`PREPARE_SERVE` additionally requires a loaded serve stack, the exact
model_21800 policy bytes (`rally_v14` version 1 / `rally_final_v2`),
`--planner --policy-native`, all arm/leg/ankle gain scales
equal to 1.0 and no serve fault. The controller applies a deterministic
25-tick strong-stand hold before raising; actual-q readiness and gripper
telemetry are diagnostic only and cannot strand the next step.

The server cannot use external `ENTER_MOTION`. Only completion of the exact
serve timeline, return to canonical stand, gain handoff, and a fresh 25-tick
settle can switch that same Runner internally to MOTION. An aborted serve
returns to PD_STAND instead.

## Motion and gripper binding

The replacement imports Jeremy's final manual-load artifacts byte-for-byte:

- `a3p_op3_serve025_manual_timeline_v1.csv`: 468 ARM14 frames at 100 Hz,
  SHA-256 `4286384b8d6f8d109db8ed9c1d82bd6325387b8a56d9ba5b068517c1ca410bfe`;
- `a3p_op3_serve025_pdstand_to_ready_v1.csv`: 251 samples at 50 Hz, resampled
  to 501 inclusive 100 Hz ticks,
  SHA-256 `7078b2ec4ff1afd5cf17793d25798cdfc218bf376a59d5769c0cb5a8ea5f6c4d`;
- canonical `grip.py`, SHA-256
  `bf837895ec010ee4b512f82cc286b3bab520b00dea051837b5016b5b15c32e19`.

The content loader rejects a wrong hash, schema, header, joint order, frame
count, timestamp, phase/event schedule, endpoint, or limit. The play events are
fixed: RELEASE command frame 46, detach target frame 50, strike frame 69,
nominal contact frame 78, recovery begins at frame 167, and complete is frame
467. The non-arm joints remain at the Runner's canonical stand target.

The packaged launcher starts one persistent gripper publisher and private
owner-only Unix socket. Startup sends no hand command. OPEN and GRAB publish
five copies at 100 ms spacing; RELEASE acknowledges after the first publish so
the 100 Hz timeline can enforce its pre-strike receipt deadline, then sends the
remaining redundant copies. A receipt remains command-publication evidence
only.

When `--serve` is active, the same Runner command callback runs at 100 Hz for
the frozen serve schedule. Policy/reference computation remains logically 50
Hz and its command is held for the alternating callback; the 110-D input and
31-D pure-policy output contract is unchanged.

## Abort, cleanup, and emergency boundaries

- Stand/abort before play enters the controlled return directly; there is no
  Loading-zone-clear action or gate.
- Abort after GRAB but before confirmed RELEASE returns to stand without
  opening the claw. `serve_cleanup_required=true` blocks role and MOTION
  changes. After a person is ready to catch/remove the ball, explicit
  `OPEN_GRIPPER` performs cleanup; it may drop the ball.
- Abort at or after strike finishes the bounded recovery but does not hand off
  to policy.
- `EMERGENCY_PASSIVE` remains immediate zero-gain loss of active support. If a
  close/release may have occurred, the software conservatively preserves the
  cleanup boundary. Inspect the physical claw and use the attended runbook
  before restart.
- A controller/bridge/tracking/timing fault is restart-and-inspect territory.
  Restarting the bridge cannot infer whether a ball is retained, so the
  operator must physically resolve that uncertainty first.

Runner/HAL process exit stays under the lifecycle supervisor or attended
runbook. Foxglove exposes no remote quit or automatic retry.

## Verification boundary (2026-08-20)

- 16 Foxglove Python tests with 286 subtests passed.
- 12 gripper bridge protocol/security tests passed.
- The x86 Runner and test target compiled. All 15 focused
  `PpRunnerControl`, `PpServe025Assets`, `PpServePolicyIdentity`,
  `PpServeController`, and `PpSha256` tests passed, including the exact
  model_21800 identity gate, full serve/handoff, and both abort/cleanup paths.
- The full 279-test binary reported 271 pass, 3 fixture skips, and 5 unrelated
  environment failures: four AimRT-off asynchronous backend/driver tests and
  one missing legacy FK fixture.
- Both the package script and its generated Runner wrapper pass `bash -n`.
- The Rockchip package built successfully; the Runner is AArch64 and the exact
  ten-file package receipt binds the binary, wrapper, configs, model_21800
  ONNX/deploy YAML, both serve CSVs, gripper bridge, and `grip.py`.
- The extension passed `npm ci`, ESLint, a Foxglove production build, and
  `.foxe` packaging under a temporary fixed Node 22.14.0 toolchain; npm
  reported zero vulnerabilities.
- No MDU upload/request/ack trial, gripper motion, powered stand, physical ball,
  strike, or serve-to-policy handoff was run. Field qualification remains
  `NOT_PROVEN`.

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
