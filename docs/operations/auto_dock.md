# Auto-dock to the serve pose

Auto-dock walks the A3 to the serve pelvis pose with the Runner's existing HumanLike policy. Motion capture supplies the pelvis pose. The operator presses Reposition. The robot returns to Stand when the dock finishes or aborts. Start to Serve stays a separate button.

The target, gains, and limits live in `a3_deploy/a3_deploy_example/assets/a3_runtime/dock/dock.yaml`. The shipped target is pelvis `(x, y, yaw) = (-0.50, -0.7625, 0)` in the `table_p1_to_p2_v1` frame: 0.50 m behind the near end line, on the table centerline, facing +x toward P2. The command is cut inside the capture band. Done is the wider tolerance in that file, because HumanLike keeps stepping for about a second after the command returns to zero. A larger error reopens the walk.

## Operating envelope

A start is accepted from any heading when the horizontal distance to the target is at most 3.0 m. The MuJoCo acceptance box is ±2 m around the target with a full turn: x in [-2.50, 1.50], y in [-2.7625, 1.2375], yaw in [-180, 180] deg. Draws with pelvis x ≥ 0, or a stand pose that intersects the table, are discarded so a trial starts on the floor. The far corner of that box is about 2.83 m from the target, inside the 3.0 m cap.

The walk has 45 s. Mocap older than 150 ms, an invalid sample, or a world-frame change aborts the dock. There is no separate start-yaw limit.

## Phases

`ENTRY` is the existing 2 s stand blend. Velocity is ignored while that blend is active. Then `ALIGN`, `SETTLE` (zero command until the gait and the body are quiet), and `VERIFY`. Outside the approach radius, and while the final heading is still outside the yaw-priority band, the walk faces the target point and then turns to the final yaw near the pose. A sample inside the capture band starts the settle. Verify accepts a fresh sample inside the done tolerance and otherwise returns to `ALIGN`, up to `max_attempts` failures. `DONE` and `FAILED` both take the quiet exit back to `PD_STAND`.

The same header-only `PpDockController` runs in the Runner and in `scripts/sim_auto_dock_mujoco.py`. The simulator does not reimplement the control law.

## Lifecycle

Auto-dock is a lifecycle setting, `off` or `on`, default `off`. Confirm it while the lifecycle is `STOPPED`. Off is today's Kernel start: no laptop mocap, no base relay, no Planner. On keeps laptop mocap and `BASE_RELAY` during Kernel start, still skips `PLANNER`, and passes `--auto-dock --dock-config dock/dock.yaml` to the Runner. The package copy of that file is `dock/dock.yaml`.

Reposition is shown only when Auto-dock is on. It is enabled from fresh `PD_STAND` with `/hope/base/fresh` true and no serve in progress. Stand aborts the dock and waits for the feet.

## Failure reasons

| Reason | Meaning |
| --- | --- |
| `DOCK_UNAVAILABLE` | `--auto-dock` or the walking policy is not loaded |
| `DOCK_MOCAP_UNAVAILABLE` | No fresh authoritative base pose |
| `DOCK_START_TOO_FAR` | Pelvis is farther than `max_start_distance_m` |
| `DOCK_STOP_REQUESTED` | Stand was pressed during the dock |
| `timeout` | The 45 s budget elapsed |
| `mocap_stale` / `mocap_invalid` / `wrong_frame` | The pose stream failed the dock's own check |
| `attempts_exhausted` | Verify failed too many times |
| `stopped` | An operator stop was already latched |

Dock telemetry is `/hope/runner/dock_state_flat` on the robot and `/hope/runner/dock_state_hdu_flat` on the console. Schema 1 has 15 doubles: schema, boot, phase, attempt, error x/y/yaw, distance, elapsed, result, reason, target x/y/yaw, pose age. It is not part of the 21-field Runner state.

## Simulation

Commands and how to read the JSON are in the `test-auto-dock` skill. A headless run reports the initial, target, and final pose, the time, attempts, fall, and the result. It exits non-zero on any fall or when the success rate is below `--min-success` (default 0.95). The 50-trial check at the default box, with 10 ms latency and 1 mm noise, finished every trial without a fall. The median time was 22 s and the 95th percentile was 35 s. At the controller's done sample the 95th percentile error was 11 cm and 5.1 deg. The stand exit after that moves the pelvis again; that shift is the `final` pose, not the dock tolerance.

## Hardware

Use an attended robot with the physical E-stop in reach. Do not move the robot until that step is requested. Start at about 10 cm and 5 deg, then widen toward the simulated envelope. After Start to Serve, log the pelvis pose at `WAIT_READY_TO_SERVE`. If the stand-to-serve blend shifts the pelvis, change the target in `dock.yaml` and redeploy that file. Simulation acceptance is not a hardware qualification.
