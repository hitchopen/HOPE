# Changelog

## 1.9.17

- Register the three 14200 variants with matched serve CSV selection and
  preserve installed-bundle verification before selection or startup.

- Remove operator UI/computer challenge echoes and heartbeat admission; managed
  play retains the independent mocap freshness guard.

- Listen at document capture so shortcuts survive action-button focus loss and
  work from other Foxglove panels. With multiple visible Consoles, target only
  the last Console selected by click/focus. Ignore hidden panels and dialogs.
- Verify the complete production bundle in Chromium with real pointer/key input:
  both Serve stages, Passive, and PLAYING/MOTION to Init after focus loss.

## 1.9.16 (local shortcut restoration)

- Rename Stand to Init and bind I → Init, R → Ready, S → the currently enabled
  Serve stage, and simultaneous Left + Right → Runner Passive.
- Remove both Serve confirmation dialogs while retaining click-time state and
  identity checks, local ball intents and synchronous duplicate suppression.
- Suppress editing, IME and held-key repeats, including across Serve stages;
  clear held keys on focus loss, window blur and visibility changes.

## 1.9.13

- Accept qualified incoming observations without invented pre-net history.
- Preserve the logical ball state across missing observations; show the pause
  reason and reject status that claims forwarding while observations are missing.
- Pair with the packetizer update that clears old fits/retries on observation
  loss and recovers from a fresh ballistic arc without another Ready/Serve press.
- A held ego serve can start a fresh observed flight with or without Ready;
  local Stop remains latched until explicit Ready/Serve.
- This release contains the ball-state refinements only; field-only telemetry,
  policy selection and network QoS changes are not included.

## 1.9.3

- Display the operator-local C++ five-state classifier, forwarding eligibility
  and separate forecast advisories through a local HTTP endpoint. No state
  topic is sent to A3; panel rendering does not control ball filtering.
- Initialize from local button intent only: Ready → OPPONENT_SERVE, Prepare /
  Ready to Serve → SERVE, Stand/Stop/Passive/E-stop → INVALID until another local
  Ready/Serve. Existing robot requests and their results remain independent.
- Remove Runner mode, serve phase, boot and lifecycle feedback coupling. Keep
  observing and forwarding eligible returns regardless of Runner state.
- Retain visible ego flights; a confirmed hold blocks automatic restart from
  SERVE. Explicit local Ready may directly initialize a new OPPONENT_SERVE.
  Observed in-table incoming returns start OPPONENT_RALLY.
- Reject foreign web origins server-side. Desktop uses a private local pairing
  file because its null origin cannot uniquely identify it. No key is sent over
  ROS, returned by HTTP, or stored in shared layouts.
- Scope local intents to the current producer and revision; reject stale or
  reordered requests and report local failures separately from robot requests.
- Pair with the laptop packetizer's advisory-only forecasts, measured 3D rally
  speed rule (below 0.10 m/s for 100 ms after a 40 ms window), and qualified
  source-clock recovery. Serve states are exempt from the new speed rule.
- Keep pelvis/base traffic independent. Support Ubuntu and the macOS operator
  container; no A3 software or bridge allowlist update is required.
- Reject missing or stale local status without matching robot lifecycle state. Cached JSON heartbeats
  avoid rebuilding unchanged status on every packetizer timer tick.

## 1.8.18

- Verify P1/P2 availability directly from fresh raw player poses before rotation.
- Accept Foxglove-normalized `sec/nsec` timestamps as well as raw ROS 2 `sec/nanosec`, so valid player poses enable side selection and START while stale/replayed poses remain blocked.
- Gray out unavailable ends and lock the assigned side until deployment stops.
- Repeat the position check in the HDU START path; preserve offline Kernel serving.
- Pair with the native 1 s mocap-loss Stand/serve-abort protection and retain model17400 selection.
- Display the persistent mocap-stop reason separately from action results; refused transitions cannot report APPLIED.
- Match HDU START evidence, explicit-peer supervisor discovery and clean retry rejection; cap raw end-position telemetry at 30 Hz.
- Mark 20 ms pelvis holds without inventing sensor freshness or delaying the 1 s rescue.
- Add independent, session-bound challenge echoes: 1 s Console / 1 s operator-relay recovery in normal play; preserve offline serve and Teleop.
- Keep 20 ms HELD telemetry separate from mode admission/serve completion, expose refused completion reasons, and run L3 pending expiry before the held-pose guard.
- Decode UInt64 boot/flight IDs from numbers or exact bigints; test decoded safety status and swing labels end to end.
- Requires matching updated mocap relays, HDU modules and rebuilt MDU Runner.

## 1.8.15

- Add field gripper presets, independent NatNet/VRPN selection and V2/V4 pants settings.
- Keep source, endpoint, units and pelvis/world confirmations explicit in the Mac capture command.
- Match the combined schema-6 backend, which preserves schema-5 venue configuration and Mac/Linux selection.
- Rebuild the installer; no robot services or motion start when installing the panel.

## 1.8.14

- Add an explicit Linux/Mac operator-computer selector alongside the network configuration.
- Subscribe to the supervisor's confirmed operator profile and submit it with the current Laptop IP.
- Require a recognized profile before configuration confirmation and lock the selector during an active lifecycle.
- Rebuild the distributed installer from the matching public source. Requires the matching profile-aware HDU supervisor.

## 1.8.13

- Accept the current named Motive body without a pinned asset hash or fixed layout.
- Display physical visibility against the live ModelDef count, including the 17-station shell.
- Use the tested `UCB_P1` asset and retain separate pelvis, clock and x_hit status.
- Include distributed session recording and the public-layout integration.

## 1.8.12

- Name the supplied 12-sticker Motive rigid body `UCB_P1` directly and update its
  pinned asset hash and companion metadata.
- Show canonical `UCB_P1` / `UCB_P2` names in the asset card and table-side controls.

## 1.8.11

- Replace Venue Calibration, JSON transfer and Cali V2/V3 controls with live status for the imported pelvis-aligned 12-sticker Motive asset.
- Show physical marker visibility out of 12; retain separate live pelvis, table-side, clock calibration and x_hit controls.
- Requires the matching Laptop asset-verification relay and launch update; the extension alone does not change localization.

## 1.8.10

- Allow Teleop requests after Xbox reconnects using fresh Runner Stand and neutral controller input; delayed Teleop display telemetry does not lock the button. Runner retains all admission checks.
- Y requires a new press in Stand and never queues locomotion through serve recovery.

## 1.8.9

- Restore Xbox face-button mode changes after reconnects.
- Align Kernel Serve completion with the slow arm return to Stand.
- Include updated Runner lifecycle and startup validation behavior.

## 1.8.8

- Publish the installer built from the public Xbox/Kernel Mode console source.
- Include 24-sticker calibration, legacy V2/V3 controls, fixed Xbox status layout
  and software E-stop reset.
- Include the extension license and bundled font notices.

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
