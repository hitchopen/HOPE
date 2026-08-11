# Changelog

## 1.2.4

- Keep Ready as the native keyboard `m` action, but disable its UI button until
  the Runner is in PD_STAND and the HDU observes a fresh calibrated Pelvis base.
- Allow Refresh x_hit enough time for the Planner file acknowledgement under
  load.

## 1.2.3

- Replace `STOP & COLLECT` with the explicit `KILL ALL & COLLECT` managed
  lifecycle action. It no longer requires a fresh PASSIVE/PD_STAND Runner
  state and warns that active support may be lost immediately.
- Keep the kill boundary limited to the lifecycle's fixed tmux sessions,
  restore `agibot_pm`, and collect session logs; no wildcard process kill is
  exposed to Foxglove.

## 1.2.2

- Keep the E-stop assert service and red panel button available independently
  of readiness telemetry, and allow an existing latch to reassert both stop
  paths without adding a Foxglove reset path.
- Preserve the panel's state/order button gates without changing the
  model_21800 Runner behavior used by the hardware-trial procedure.
- Add the Laptop flight-packet data adapter required by the HDU Planner so a
  `MOTION` Runner can receive real incoming-ball commands without changing
  the downstream `build_1` estimator, bounce, target or schema-2 algorithms.

## 1.2.1

- Keep the assert-only E-stop button actionable when managed operation removes
  the vendor service but the native Runner emergency-PASSIVE path is live.
- Publish and display full dual-path readiness separately, so Runner-only or
  vendor-only operation remains explicitly marked partial and requires the
  physical E-stop.

## 1.2.0

- Split Calibration and Planner `x_hit` refresh into two explicit buttons and
  services. Calibration no longer creates an x_hit request.
- Persist an audited stationary `world -> pelvis_link` snapshot inside the
  calibration JSON while retaining live `world -> P1 -> pelvis_link` runtime
  localization.
- Keep Runner sequence state and role controls inside a responsive bordered
  header so the row no longer overflows the card.

## 1.1.0

- Make Calibration one fail-closed operation: recalculate the ten-marker
  `P1 -> pelvis_link` receipt, wait for a fresh matching `world -> pelvis_link`
  base packet, then refresh Planner `x_hit`.
- Show the process with the largest CPU delta beside aggregate HDU CPU load.
- Preserve vendor E-stop availability when the managed Runner stop path is
  degraded, run both paths when possible, and display the persistent
  authoritative latch instead of optimistic UI state.

## 1.0.0

- Promote the single-robot Runner console to its stable interface names.
- Use the formal `/hope/runner/*`, `/hope/lifecycle/*`, and monitoring namespaces.
- Remove development-version naming from layouts, services, and deployment paths.

## 0.2.0

- Add four editable, confirmed IPv4 fields for the three-machine lifecycle.
- Add fixed STEP 0/1/2A/2B/4/5 startup progress and session state.
- Add explicit stop, `agibot_pm` restore and evidence-collection feedback.

## 0.1.0

- Add the HOPE A3 single-robot operator console.
- Bind controls directly to the native Runner and x_hit services.
- Display NTP, ROS latency, process, marker, role, serve and CPU state.
