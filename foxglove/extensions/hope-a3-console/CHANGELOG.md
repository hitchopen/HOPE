# Changelog

## 2.2.0-beta.10

Needs the matching HDU supervisor and `hope-lifecycle` helper (automatic vendor repair).

- STOP repairs the vendor stack by itself when it never becomes ready (`VENDOR_SM_NOT_READY` /
  `VENDOR_SM_FAILED`): the status line reads 「正在自动修复厂商系统」; if that does not succeed the result says
  to reboot the HDU and MDU together (`PAIRED_REBOOT_REQUIRED`).

## 2.2.0-beta.9

- Keyboard shortcuts labelled: the key badges (I / R / S / ← →) stay on the state-graph buttons at all times
  (dimmed while that action is blocked, smaller in narrow panels) and a one-line legend sits under the graph
  ("快捷键 · I 站立 · R 接球 · S 发球（当前阶段）· 同时按 ← → PASSIVE"; full rules on hover). Behaviour unchanged.

## 2.2.0-beta.8

- Policy dropdown shows only 14200 Best. 19000 GRU, 26500 and 400 stay in the catalog with `hidden: true`
  (not deleted); a robot that reports one of them still sees it named and flagged.

## 2.2.0-beta.7

- Recording row: one short description ("记录机器人、策略、Planner 和动捕数据"; Kernel: robot and policy only).

## 2.2.0-beta.6

Needs the matching HDU `hope_time_calibration.py`, `hope_mocap_recovery.py` (deployed together).

- **Time calibration no longer depends on mocap.** It only coordinates HDU/MDU clocks, so it is admitted when
  every HDU service runs the saved configuration (`hdu_applied`); an unplugged Motive no longer disables it.
- **"动捕连接" row in the Start card**, under time calibration, with its own **重新连接动捕** button. It shows
  live poses / no poses (with the saved Motive or VRPN address) / reconnecting / last failure; the hover
  text explains why the button is disabled (system running, cooldown, another operation, mocap 无).
  Reconnecting restarts only the laptop receiver and never creates a configuration revision.
- **Plain START reason when only mocap is missing** (normal play): "正常比赛需要动捕，现在动捕没连上…" instead of
  "请保存配置…"; the config row points at 重新连接动捕.
- **Recording moved into the Start card** (row under 动捕连接): same Off/On switch, state, notes, output path
  and error as before; the diagnostics fold now holds only the activity log ("更多：操作记录").
- Robot side: operator reconnect now serves Motive as well as VRPN (automatic recovery stays VRPN-only),
  with a 10 s cooldown after a manual attempt (automatic keeps 60 s) and a stable `manual_blocked_code`.

## 2.2.0-beta.5

Needs the matching HDU `hope_config_runtime.py` and supervisor (deployed together).

- **Kernel START no longer depends on the laptop mocap receiver.** A Kernel session never contacts the
  operator computer, yet START required the laptop Motive/VRPN receiver to be running, so with Motive
  unplugged a Kernel START was impossible ("确认完没有办法 start"). The robot now publishes `hdu_applied`
  (every HDU service runs the saved configuration) and admits a Kernel START on it; normal (OptiTrack) START
  still needs the complete audit, and a START admitted as Kernel re-checks the complete audit if the mode
  changed before the session is selected. The Console shows "✓ 配置已生效 · Kernel 不用动捕" in that case.

## 2.2.0-beta.4

Needs the matching HDU supervisor and `hope-lifecycle` helper (deployed together).

- **Confirm configuration = one revision.** Choices and typed addresses no longer save themselves; they stay
  an edit ("N 项修改未确认") until 确认配置. Each click previously saved a revision and restarted the
  laptop mocap receiver (r41–r45 in a few clicks).
- **Confirming unchanged values keeps the revision.** The robot replies `CONFIG_UNCHANGED`, only re-checks
  the runtime and restarts a missing mocap receiver; the Console no longer waits for a revision that never
  comes ("确认中…" / "已保存，等待生效" forever).
- **No revision numbers on screen.** The config row and the stopped START line no longer show "配置 rNN"
  (still in the row's hover text).
- **Plain reason when mocap is down.** When every robot service matches and only the laptop receiver is
  missing, the row says "动捕没连上（Motive <ip>）· 没接动捕就把「系统」选「无」…" instead of the raw audit.
- Robot side: `reconnect-laptop` waits (≤14 s) for the stopped receiver's in-container processes to exit
  before starting the new one, so back-to-back confirms no longer fail with `UNMANAGED_BRIDGE_PRESENT`
  (which left no receiver at all); reconnect timeouts 25 s → 40 s to fit the wait.

## 2.2.0-beta.3

- **Mocap & hardware card layout.** Same controls and styling; every row now aligned to the 4-column grid.
  Mocap stays at the top; **Network** and **Devices** get small headings with a divider. "Use detected
  connection addresses" fills the space next to the MDU address, with a one-line hint below (full
  explanation on hover). Operator PC and Gripper share one row. Previously MDU sat alone in its row, which
  shifted every following row.

## 2.2.0-beta.1

Merged `origin/codex/model400-foxglove-20261118` (Console 1.9.20-recovery → 1.9.23-model400, ported into the 2.x UI).
Needs the matching HDU supervisor / config runtime, laptop helper and Planner; do not install alone.

- **Robot identity and network migration (their scheme).**
  - The robot address is the one bound to the live robot identity (machine_id / boot_id), not a typed IP.
  - "Confirm connection" keeps the current window when it is the same robot; another robot is an explicit
    direct connection.
  - Mocap & hardware has the HDU address field and **Use observed connection**, which fills the HDU / laptop
    pair the robot sees on 8766. Confirm config then sends `connection_identity`, and the robot checks it
    (live connection, registered operator machine).
  - The connection editor shows the saved address and the robot identity.
  - The 2.1 "Sync addresses" banner and the laptop `hope-robot-link` reporter are removed.
- **Pelvis gate.** Ready, Start to Serve and Serve fail closed (render, keyboard and dispatch) unless the pelvis is
  fresh from this Runner (base consumer and pelvis monitor agree, or the Runner's bounded hold). MOTION is not
  shown as reached without it.
- **P1 / P2 select the pants body** (UCB_P1 / UCB_P2), not a table end. Either can be chosen while stopped; the
  receiver finds the physical end. VRPN keeps only the world-frame confirmation (config schema 8).
- **Return flight time** in Hit tuning (`/hope/planner/set_flight_time`, persisted by the Planner).
- **model400** policy (serve + receive).
- Live / replay telemetry text moved into the catalogs (the long-failing i18n test passes); our Walk stand /
  Reposition labels and "pose from mocap" state kept.

## 2.1.0-beta.26

- **Teleop panel after Y.** While the Runner is in TELEOP, the teleop bar becomes a panel:
  - Entering → Active → Leaving steps.
  - One large instruction for the current step: release LT and center the sticks; hold LT to move; release
    LT to stop; leaving.
  - Live LT state, forward / sideways / turn stick bars with the Runner's commanded velocity.
  - A key map with the pressed key highlighted. A and X are flagged because they leave Teleop (serve / receive);
    B and Y have no effect; LB+RB is the E-stop.
  - Pressing A or X turns the panel amber with a warning.
  - Runner input age, the pad's last button feedback and Stop & stand.
- Outside Teleop the bar shows "Y pressed; requesting Teleop…" while the request is pending and the pad's
  last button feedback (e.g. why Y was ignored).

## 2.1.0-beta.25

- System health → More → Recording has its own Off / On switch (the same command as the header REC pill).
  Before, that line only showed the status and could not be clicked. While the system is stopped, a hint explains
  that On waits until Start system, which also turns recording on automatically.

## 2.1.0-beta.24

- Ball model offers only PPQ002 and PPQ004; Berkeley is removed from the selector (the robot still accepts it, so
  a robot that reports Berkeley shows neither option selected).

## 2.1.0-beta.23

- System health → More no longer shows the E-stop backend line (the header E-stop chip and the E-stop button
  tooltip carry it). More now holds recording and the activity log.

## 2.1.0-beta.22

- **Reset software E-stop** is a permanent button next to the E-stop in the header (amber while the stop is
  latched). System health → More keeps the E-stop backend status line without its own reset button; the latched
  banner is unchanged.

## 2.1.0-beta.21

- Local control pairing ("本机控制授权（连接设置）") moved from System health → More to the top of Setup, as a
  fold showing whether the local ball service is live.

## 2.1.0-beta.20

- **Header status merged by subject (13 chips → 7 while stopped, 5 while running):**
  - System = idle + stopped
  - Time = NTP gate + message timestamps + time calibration
  - Configuration (unchanged)
  - Mocap = markers / pose + our table side seen (yellow when the side is not seen)
  - Field = field status + ball model verification
  - E-stop and TF (unchanged)
  - Swing input, now shown only while the system runs
  A merged chip turns red/yellow when any part fails, lists every part in its tooltip and opens the first failing
  part. Display only; START gating (startBlockReason) is unchanged.

## 2.1.0-beta.19

- Setup order: This session above Mocap & hardware (the configuration bar stays pinned at the bottom).

## 2.1.0-beta.18

- The beta.17 SuperMimic-style theme is removed (operator: the original look is better); the Console is visually
  identical to beta.16 again.

## 2.1.0-beta.17

- **Dark theme in the supermimic.github.io style** (Apple dark mode): near-black band, #1c1c1e cards, grey
  wells and tracks, #f5f5f7 text, one blue (#0a84ff), pill buttons (primary blue, secondary grey, STOP red),
  segmented controls and tabs with a raised grey thumb, borderless input wells, frosted dark header and footer,
  system green / orange / red for status; the E-stop stays solid red. Markup is unchanged.

## 2.1.0-beta.16

- Setup no longer has the System card (status "Stopped · config rN" is in the header and on Play's Start card).
  Its occasional notices (locked while running, field-control error) appear above the pinned configuration bar.

## 2.1.0-beta.15

- The configuration row (status, robot reason, Confirm config) is pinned to the bottom of Setup, so Confirm is
  reachable while editing Mocap & hardware or This session at any scroll position. The System card keeps the
  lifecycle status.

## 2.1.0-beta.14

- **Start checks join the header health chips.** Idle, stopped, time calibration, configuration, field status,
  ball model and the table-side check sit in the same row as clock, latency, TF, E-stop, mocap and swing input,
  in the same chip style. A failing check opens where it is fixed. Hidden while the system is running.
- **Setup → Confirm config → Play.** Setup's System card keeps the lifecycle status and the permanent configuration
  row; after Confirm config is accepted the Console switches to Play.
- **Play starts with a "Start" card:** lifecycle status, time calibration, START / STOP and the lifecycle result,
  above System health. "Runner offline" points to it ("Go to start").

## 2.1.0-beta.13

- **Configuration is a permanent row** of the System card, directly under time calibration: a dot, "Configuration
  · config rN", the state text ("Config revision N applied", "Not applied yet · <robot reason>", "Saved, waiting
  for the robot", "N unconfirmed edits") and an always-present **Confirm config** button. The button is yellow
  when there is something to confirm and grey otherwise, so the row never appears or disappears. The separate
  "config revision applied/pending" result line is gone (the row shows it).

## 2.1.0-beta.12

- Setup order: System, then Mocap & hardware, then This session.

## 2.1.0-beta.11

- "Configuration not applied" no longer flashes. When the robot's runtime audit flaps (seen 2026-11-17: the
  laptop Motive node exits every ~12 s because Motive 169.254.10.221 is unreachable, so each audit briefly reports
  `missing_processes: motive`), the Console holds a robot-reported "not applied" for 15 s. The bar, the config
  check chip and START therefore show one steady state, and the bar shows the robot's reason. Missing or stale
  data does not start the hold, so connecting does not wait 15 s.

## 2.1.0-beta.10

- Setup order: Mocap & hardware, then System, then This session.

## 2.1.0-beta.9

- The beta.8 HUD theme is removed (operator: harder to read than the original); the Console uses the original
  2.0 look again. The beta.8 layout and functions stay: three Setup cards with every original control, time
  calibration as a row of the System card, and paired rows in Mocap & hardware.

## 2.1.0-beta.8

- **Every original Setup control is back, reorganized into three numbered sections** (01 System, 02 This
  session, 03 Mocap & hardware). The beta.7 device line/dialog and the clock-only control are gone.
  - **01 System:** the original lifecycle card (all start-check chips, the "configuration not applied" bar with
    Confirm config, START / STOP, result line) with **time calibration as its own row** under the status line
    (state, step, result, Run calibration).
  - **02 This session:** run mode and ball model keep their subtitles; the ball status line is always shown;
    serve CSV above the policy picker.
  - **03 Mocap & hardware** is always visible. Pairs share a row (Laptop IP | MDU, operator computer | gripper)
    and the mocap connection and asset lines are one line. The original auto-save behaviour is restored.
- **HUD theme** across the whole Console: glass panels with a lit corner, a faint grid backdrop, a glowing
  accent for selection and status, the Space Grotesk display face for titles, tabs and buttons, gradient
  START / STOP / Confirm buttons and a stronger E-stop. Markup is unchanged apart from class hooks.

## 2.1.0-beta.7

- **Setup v3.** Three blocks: the lifecycle card, This session, and one quiet **Devices** line.
  - Time calibration is no longer a card. The lifecycle status line shows the clock state (dot + "Clock
    calibrated / not calibrated / failed / calibrating") with a **Calibrate** button on the right; details are in
    the tooltip. The START check "time calibration" jumps there.
  - Start checks show only what fails; when everything passes, one line reads "✓ All N start checks pass".
  - The yellow "configuration not applied" bar became an inline note plus **Confirm config** in the checks row.
  - **Devices** (mocap source and address, Laptop, MDU, operator computer, gripper) is one line with
    **Edit** (**View** while running). Editing happens in a dialog with **Save** / **Cancel** (Esc). Fields in
    the dialog never save on blur, Enter or click; only Save writes the robot configuration. Table side still
    saves on click.

## 2.1.0-beta.6

- **Policy picker scales with the catalog.** The policy row is one line showing the current model (name,
  serve/receive tags, bundle or note, loaded state). Clicking it opens the list below: one row per model, at most
  about five rows tall and then scrolling. Models this robot cannot load sink to the bottom, and a filter box
  appears once there are more than six models. Selecting a model closes the list; Esc also closes it. Adding a
  model is still one `POLICY_ROWS` entry and never widens or lengthens Setup.

## 2.1.0-beta.5

- **Setup redesign.** Seven cards became four:
  - Time calibration is one line (state · step; the result text only when it failed or is running).
  - The lifecycle card is unchanged.
  - **This session** holds what changes per session, in order: run mode, **serve CSV (above the policy)**,
    policy (a three-way segmented control with serve/receive tags and the 14200 Best bundle), our table side,
    ball model (its status line only when it is not ready). Kernel mode shows IMU localization instead of the
    ball model.
  - **Mocap & hardware** is a fold, closed by default. Its summary shows the mocap source and address, Laptop,
    MDU, operator computer and gripper, with a dot for the mocap connection. Inside: mocap system and addresses
    (VRPN units, pants, attestation and reconnect when VRPN), one combined mocap status line, Laptop / MDU
    addresses, operator computer, gripper and the manual startup command.
- Segment subtitles (mode, mocap source, pants, ball stars, gripper values) moved to tooltips.

## 2.1.0-beta.4

- **Sync robot addresses after a network change.** A new loopback reporter on the operator computer,
  `hope-robot-link` (`foxglove/laptop/install_robot_link.sh`, http://127.0.0.1:8771/robot-link), reads the
  kernel TCP table and reports which robot (HDU) this Foxglove is connected to on 8766 and which local address
  the connection uses. When the robot's stored HDU / Laptop addresses differ, the header shows both pairs and a
  **Sync addresses** button (only while STOPPED and idle). It writes exactly those two fields from the live
  connection through the existing apply_config; typed HDU addresses are still never written to a robot.
- The live link also completes a robot switch whose stored address is from an earlier network, and the header
  shows the live robot address instead of the stale stored one.
- System health "More" no longer shows the config runtime block (and its pending tag).

## 2.1.0-beta.3

- System health no longer turns red just because the MDU Runner is stopped while the lifecycle is not RUNNING;
  the Runner pill is grey then. A stopped agibot_pm / HDU, a stopped Runner while RUNNING, or a recording
  ERROR still turn the card red.

## 2.1.0-beta.2

- System health sits at the top of Play, above the play control (state graph) card.
- Time calibration sits at the top of Setup, above the lifecycle card (state / config revision / START).

## 2.1.0-beta.1

- **Health tab merged into Play.** The Console has two tabs, Setup and Play (Kernel debug). The bottom of
  Play is a **System health** dropdown, styled like Hit tuning: the summary always shows clock offset, message
  latency, CPU with a 120 s mini chart and processes ok/total, and its dot turns red when a process is down or
  recording fails (amber when the config is not applied or the E-stop is latched). Expanding it shows what the
  Health tab showed: NTP / latency / CPU tiles, the CPU chart and top process, process pills, E-stop backend
  with software reset, config runtime, recording output, ball pairing, the activity log and the version.
- Header health chips open Play and expand System health. The Play tab carries the red badge when a chip fails.
- **Time calibration moved to Setup** (below Hardware); the START check "time calibration" jumps there.
- The duplicate Runner-safety card was dropped: the Play state card already shows the mocap safety line and the
  Runner summary.

## 2.0.0-beta.17

- Walk stand and the Reposition target are one dropdown, styled like Hit tuning. The summary shows ▸ 步态站立,
  the current state, target x / y and the Reposition button (Stop & stand while walking). Expanding it shows the
  walk-stand and Reposition status lines and the target x / y fields with Apply / Revert. The separate 走位目标
  dropdown is gone.

## 2.0.0-beta.16

- Walk stand card: the Reposition target is a collapsible dropdown styled like Hit tuning (▸ 走位目标 · x · y).
  Expanding it shows target x / y with Apply / Revert; every other limit keeps the Runner's current value.

## 2.0.0-beta.15

- Walk stand card: only an **Adjust Reposition target** button is shown. Clicking it opens target x / y with
  Apply / Revert; every other Reposition limit keeps the Runner's current value.

## 2.0.0-beta.14

- Reposition limits are built directly into the Walk stand card (no title, no dropdown).
- START no longer drops the Console: the laptop helper (`hope-lifecycle`, laptop copy) restarts the Host bridge
  on ws://127.0.0.1:8767 only when its robot peer (HDU IP, laptop IP, code root) changes. Before, every START's
  mocap reconnect restarted it, so the Console disconnected, flickered and lost clicks until it reconnected.

## 2.0.0-beta.13

- Reposition tuning is now a dropdown inside the Walk stand card instead of a separate card.
- Reposition (node and button) starts at once, without a confirmation dialog.
- Dock telemetry freshness is 2 s, so Wi-Fi bursts no longer toggle Reposition between enabled and disabled.

## 2.0.0-beta.12

- **Init ends in the walking stand.** PASSIVE -> Stand still lifts the robot with the stiff PD stand, then the
  Runner switches to WALK_STAND by itself after about 2 s once the body is quiet. The Stand node reads
  `Init → walk stand`; the Walk stand card shows the current mode instead of "Not standing".
- **Reposition tuning card** (Play tab, below Walk stand): target x / y / yaw, done tolerance (cm, deg), forward /
  backward / sideways speed, turn rate and max start distance. Apply publishes one request on
  `/hope/runner/dock_config_hdu_flat`; the Runner validates it against a fixed envelope, applies it to the next
  Reposition (never during a walk) and saves `dock/dock.yaml` (first original kept as `dock.yaml.orig`). The
  card shows the Runner's reply: applied/saved, out of range, busy, or not saved. Needs Runner dock state
  schema 2.

## 2.0.0-beta.11

- **Stand is the walking stand** (Runner `WALK_STAND`). PASSIVE -> Stand is still the stiff Init (hoist-safe);
  Stand again, Teleop/Reposition stop, end of Receive and a completed Kernel serve all land in WALK_STAND.
  Ready and Start-to-Serve are enabled from WALK_STAND: the Runner settles the feet, passes through the stiff
  PD stand for about a second, then starts them. The Stand node reads `WALK_STAND · HumanLike`, or
  `PD stand · click for walk stand` in the stiff stand. The separate Walk stand enter/exit buttons are removed.
- **Reposition from Teleop**: the node and button are enabled in TELEOP too (no policy reset).
- Teleop (Y / button) and Open gripper are allowed from WALK_STAND.

## 2.0.0-beta.10

- **Reposition** is now a node in the Play state graph, below-left of Stand with an arrow from Stand (normal play
  only; Kernel has no mocap base pose). Grey with a short reason (Runner offline, 未加载走位, 先站立, 等待动捕,
  发球中) until it can run; click asks for confirmation. While walking it is the current node and shows phase and
  remaining distance; the Stand node reads `DOCK · 点击中止` and stops the walk.

## 2.0.0-beta.9

- The **Reposition** button is always shown on the Walk stand card. It is greyed out, with the reason on the line
  above it, until the Runner publishes dock telemetry (system started with an auto-dock slot) and the robot is
  in Stand or Walk stand with a fresh base pose. beta.8 hid the button entirely before START.

## 2.0.0-beta.8

- New **Walk stand** card on the Play tab (above Teleop). It switches the stand from the stiff PD stand to the
  Runner's HumanLike walking policy at zero velocity (`/hope/runner/enter_walk_stand`, Runner action 15, mode
  `WALK_STAND` = 8). Available only from a fresh, healthy PD stand with HumanLike loaded, and only after a
  confirmation that the robot is on the ground with the hoist slack. **Back to PD stand** (or Stand) settles the
  feet and returns to the stiff stand. Ready and Serve stay unavailable in walk stand; the stand node shows
  `WALK_STAND · click for PD stand`.
- **Reposition** on the same card (`/hope/runner/enter_dock`, action 14, mode `DOCK` = 7): HumanLike walks the
  robot to the serve pose from upstream auto-dock (P1, 0.5 m behind the end line, centerline). Shown only when
  the Runner publishes dock telemetry; enabled from Stand or Walk stand with a fresh mocap base pose, after a
  confirmation. A status line shows phase, Δx/Δy/Δyaw, attempt, elapsed time and result. **Stop & stand** aborts
  the walk; a finished walk stays in Walk stand.
- Includes the live-telemetry pose fallback from the parallel session: robot XYZ / R-P-Y show from the mocap-only
  `base_available` / `base_age_ms` fields when Runner joints are not fresh.

## 2.0.0-beta.7

- New separate policy option **26500** (`model26500`, policy SHA256 `379c2abc…`), listed below 19000 GRU. Existing
  bundles are unchanged. Feed-forward receiver on the same field Runner as 14200 Best, installed as its own MDU slot
  and HDU Planner profile. Receive only (no matched serve CSV) and OptiTrack only; Kernel mode refuses it.

## 2.0.0-beta.6

- New separate policy option **19000 GRU** (`model19000`), listed below 14200 Best; 14200 Best and every installed
  bundle are unchanged. It is the explicit-GRU receiver from the 2026-11-07 integration review (policy SHA256
  `2b2a029b…`), installed as its own MDU slot and HDU Planner profile. Receive only: it has no matched serve CSV,
  so the serve capability is shown off, and it requires OptiTrack (Kernel mode refuses it, as the backend's
  `MOCAP_MODELS` does). The "another policy is selected" warning now covers only models not offered in the list.

## 2.0.0-beta.5

- START and STOP sit side by side at the bottom of the 启动系统 card: START on the left, STOP on the right, both
  full size in every state. One rule table (`src/lifecycleButtons.ts`, tested state by state) decides them:
  - stopped: START live (grey while a start check fails), STOP grey "nothing is running";
  - starting: START striped 启动中…, STOP grey (the backend rejects a kill while busy; use the E-stop);
  - running: START grey ▶ 运行中, STOP live red ■ 停止;
  - stopping: STOP striped 停止中…, START grey until it finishes;
  - failed: 重试启动 and ■ 停止 (cleanup only) both live;
  - stale lifecycle state: both grey.
  STOP is live exactly where the backend accepts kill_all_and_collect (RUNNING/FAILED, not busy). Every grey
  button shows its reason as a tooltip; greyed buttons are neutral grey rather than faded colour. Same services
  and confirmation dialogs as before. The small stop button in the card header is removed.
- The lifecycle result line no longer shows a raw `CONFIG_SAVED_PENDING_RUNTIME_REVISION_<n>` after the robot has
  applied that revision (the backend never rewrites it). Config results are read against the live
  `/hope/lifecycle/config/status`: "✓ 配置第 n 版已生效" once applied, "配置第 n 版已保存，等待机器人生效…" until then.
  `CONFIG_LOADED` / `CONFIG_NOT_CONFIRMED` are hidden; every other result stays verbatim.

## 2.0.0-beta.4

- Kernel mode also runs 14200 Best. The robot backend rule (`hope_policy_models.MOCAP_MODELS`) now exempts
  `model14200best` as well as `model14600`; every other model still requires OptiTrack. The Kernel row for 14600
  is gone and switching between Normal play and Kernel is a single `set_mode` (no policy swap, no multi-board
  verification round trips).
- Configuration choices save themselves: segmented choices, table side, operator computer and VRPN
  confirmations call the existing confirm-configuration service after a 0.4 s debounce; IP fields save on blur or
  Enter. The same conditions as the 确认配置 button apply; when they are not met the edit stays pending (and the
  manual button stays available) and saves once they clear.

## 2.0.0-beta.3

- The 14200 Best bundle is identified from the policy catalog's own serve CSV hash, so the
  installed version shows as soon as the model is available, not only after it is selected.

## 2.0.0-beta.2

- Policy list offers only 14200 Best (`model14200best`, bundles published on
  `origin/codex/ball-models-field-runtime-20261016`). The selected row names the installed
  bundle (`best-v1` or `earlier-staged`) by matching the reported serve CSV / policy hash
  against `model14200best/manifest.json`; an unknown hash is shown as not identified.
- If the robot reports another selected model, the policy card says so and offers 14200 Best.
- Kernel mode shows its only legal policy, 14600 (backend `MOCAP_MODELS` rule). The Kernel
  button selects 14600 before `set_mode KERNEL`; Normal play switches mode first and then
  selects 14200 Best when coming from 14600. Each step is the existing field command under its
  existing gate, and a rejected step stops the sequence.
- `fieldCommand` now reports success to its caller; its notices are unchanged.

## 2.0.0-beta.1 (Console 2.0 redesign)

Presentation and layout rewrite of the HOPE A3 Console, based on field runtime
259c249e3. Service names, requests, timeouts, confirmation prompts, click-time
identity/state checks, keyboard shortcuts and every enable/disable condition
are unchanged except for the three behavior changes listed first.

Behavior changes:

- **Automatic role switch.** Receive (Ready) while our role is SERVER calls the
  existing `set_receiver` first; Raise & load (Start to Serve) while RECEIVER
  calls `set_server` first. This is only offered when the Runner reports
  `role_change_allowed`. The Console then waits up to 2 s for a newly received
  `local_role` equal to the requested role, re-checks the unchanged 1.9 action
  gate against fresh state, and only then sends Ready or the unchanged immediate
  Start to Serve path. A rejected/failed/timed-out role change, any other robot
  command, a robot/session/boot change, a pending robot switch or a latched
  E-stop in the meantime sends nothing and leaves a notice. Because keyboard
  shortcuts press the same buttons, R and S follow this rule as well (matching
  the controller's X and A).
- **PASSIVE is a single click** with no confirmation and no long press (the
  1.9 behavior, kept deliberately; the mock's long-press was not adopted).
- **Read-only teleop bar.** The Console no longer offers Enter Teleop; Teleop
  is entered with Y on the Xbox controller. The bar shows controller status,
  the Runner's commanded velocity and the existing Teleop entry reason. Exit
  through the Stand node; the 1.9 "Stop and return to Init" button (same gate)
  is shown in the bar only while in TELEOP.

Layout:

- Sticky header: robot IP/:8766 with Switch (the existing connect/pending/cancel
  flow), language, E-stop, status pills (mode · lifecycle, Runner, policy ·
  role, Record), six health chips (details in tooltips; click opens Health),
  latched E-stop banner with the software reset, and Setup / Play / Health tabs.
  Kernel mode adds a violet top border and a violet "Kernel debug" tab.
- Setup: Start card with one check chip per existing Start condition
  (`startChecks`, proven equal to `startBlockReason`), unconfirmed-edit count with
  the existing Confirm config button, full-width Start / Retry / Stop & collect;
  segmented controls for run mode, mocap source, units, pants, side, ball model
  and gripper; policy rows with capability tags; serve CSV with a timeline from
  validated segment metadata. Kernel shows an IMU-odometry localization card.
- Play: clickable Runner state graph (PASSIVE, Stand, Receive, Raise & load,
  Serve; Kernel variant with the automatic return to Stand), Open gripper, the
  unchanged hit-speed / plane-mode / y-filter controls in a collapsible card,
  and the teleop bar. The ball-state line moved out of the Console (Live
  Telemetry); ball pairing stays under Health.
- Health: NTP/latency/CPU, CPU history, processes, time calibration, E-stop
  backend, Runner safety, configuration runtime audit, recording, ball pairing
  and a notice history. Long explanations moved into tooltips.
- All tabs stay mounted (inactive ones hidden), so keyboard shortcuts, drafts
  and the y-filter preview behave exactly as in 1.9 regardless of the open tab.
- Confirmations use a modal (Esc cancels, Enter confirms; focus starts on Cancel).
- Live Telemetry adds an event strip: Planner target sent, ball crossed the hit
  plane, strike snapshot with |actual − target| in mm. With panel config
  `ownsPreview: true` it also applies the Console's local y-filter preview to
  its sibling 3D panel (as HOPE Live Status does).
- New layout `foxglove/layouts/console_v2.json`: 3D over Live Telemetry on the
  left, Console on the right.

## 1.9.17

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

- Add the independent 13800 Astra policy and matched CSV selection; retain the original 13800 entry.

## 1.9.11

- Make the live ball state, Runner mode and forwarding state visible as compact
  14 px status badges while preserving the existing XYZ/RPY and freeze values.
- Show observation loss independently from the retained logical ball state and
  show armed serve/receive intent without hiding forwarding status.
- Read Runner mode and its own freshness from `/hope/live/telemetry`; stale or
  absent Runner feedback displays unknown rather than inferring mode from the ball.
- This panel update is display-only and sends no robot control requests.

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

## 1.8.16

- Add runtime strike-plane comparison: fixed, current position per ball, and
  experimental posture-stable updates. START does not lock the selector.
- Confirm selections from fresh Planner diagnostics and distinguish the next
  selection from the last published flight/reference/plane.
- Keep active targets frozen; dynamic modes require the upgraded shared-HOME
  Planner/Runner contract. The posture gate does not infer foot support.

## 1.8.15

- Add an explicit stopped-only mocap selector: None, Motive / NatNet, or
  QingTong / CMTracker / VRPN, with separate addresses and VRPN port/units.
- Show the selected system's setup and a copyable Laptop startup command;
  retain separate Runner startup and the V2/V4 pants and gripper selections.
- Require matching schema-5 supervisor and Laptop runtime. Old configurations
  preserve existing fields and default to no mocap until explicitly selected.
- Display VRPN as pose-only with live marker count unavailable, retaining the
  explicit pelvis/world confirmations before a valid base packet is emitted.

## 1.8.14

- Add a stopped-only V2 / V4 marker-pants selector and atomic VRPN configuration.
- Supply pelvis-origin CMTracker ground-truth definitions for V2 (10 points) and V4 (17 stickers).
- Show VRPN pose-only verification limits; retain the existing gripper profiles.

## 1.8.13

- Add a stopped-only gripper selector: legacy close/open 800/2000 or the
  physically confirmed 2026-10-15 gripper 2410/4000. Persist the choice for the
  next start and use it for manual commands and native timed serve release.

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

## Public 1.8.9 / build_6 runtime sync

- Include the main release installer; local development calibration UI is retained.
- Restore face-button mode changes after Xbox reconnects.
- Kernel Serve lowers the arms slowly into Stand; default CSV is v12.
- Sync lifecycle startup validation and document simulation limits.

## 1.7.4

- Remove the Diagnostics heading while keeping all diagnostic cards visible in place.

## 1.7.3

- Put operating mode, Recording, venue calibration and Pure Serve CSV above System Lifecycle.
- Keep audit chips and the last UI request first, and Diagnostics visible below System Lifecycle.

## 1.7.2

- Always show Diagnostics below System Lifecycle, with no collapse control.

## 1.7.1

- Put System Lifecycle and Time Calibration below the source / E-stop header, audit chips and last UI request.
- Place the collapsed Diagnostics section directly below System Lifecycle.
- Adapt layout to the Foxglove panel width, including narrow panes in a wide window.
- Keep cards at their content height and wrap long button labels, statuses and filenames without overlap.

## 1.7.0

- Persist venue calibration and show Load, Export and explicit Recalibrate controls.
- Add Pure Serve with validated local CSV upload and the existing two-step serve; finish in Stand.
- Add independent Record ON/OFF, default OFF, with actual MCAP process/storage state.
- Keep normal play controls and show diagnostics in a separate disclosure. Requires matching Laptop, HDU and MDU backend updates.

## 1.6.8

- Remove OptiTrack from `START SYSTEM` lifecycle admission and managed cleanup;
  the optional Laptop mocap stack can run independently.
- Allow the optional Motive address field to remain empty for Runner-only
  sessions.
- Allow a standing Receiver to invoke the native Runner `ENTER_MOTION` action
  without fresh base/calibration data. Marker, calibration, base and ball data
  remain visible as optional telemetry.
- Keep the existing Runner freshness/fault, role/mode, HAL input readiness and
  E-stop paths unchanged; this release does not change Runner safety thresholds.

## 1.6.7

- Make the receive sequence explicit: a visible Ball marker is not a swing
  command, and a new complete incoming flight must arrive after Runner reports
  `MOTION`.
- Show the existing HDU Planner command flight/countdown as a `SWING INPUT`
  audit chip, so a visible Ball can no longer be mistaken for a live command.
- Clarify that the server receive handoff becomes active only after the full31
  CSV completes.

## 1.6.6

- Update the server controls for the Build4 serve025 single full31 timeline:
  smooth PD_STAND-to-frame-0 preparation, human load/clear, then one complete
  CSV play followed by the same Runner's automatic MOTION/receive-policy
  handoff.
- Remove stale Spin001 and return-to-PD_STAND wording.

## 1.6.5

- Replaced the retired Serve025 UI wording with Jeremy's exact Spin001
  full31 ENTRY/TIMED_SERVE/RECOVERY workflow at 50 Hz.
- Kept server launch free of Cali, Refresh x_hit, base freshness, gripper and
  tracking gates; the same Runner still owns stand, serve and policy handoff.

## 1.6.4

- Remove the stale Pelvis-base gate from the server raise-hand path.
- Keep serve tracking error as telemetry only; it no longer faults the
  deterministic PD motion or drops immediately back to `PD_STAND`.
- Require lifecycle Runner launch through the packaged serve wrapper so its
  best-effort gripper bridge starts with the same accepted action.
- Remove the unavailable `aimdk.protocol_pb2` runtime dependency from the
  bridge while preserving the vendor `HandCommandChannel` protobuf wire format.

## 1.6.3

- Allow an attended server to enter `Serve · Raise hand` directly from a fresh
  standing session without running Cali V2/V3 or refreshing x_hit first.
- Keep calibration and x_hit requirements unchanged for receiver policy entry.

## 1.6.2

- Display and audit the P1/P2 side selected in the lifecycle configuration on
  marker and Cali V2/V3 controls.
- Publish side-neutral marker telemetry while retaining the legacy `p1_*`
  topics for staged-update compatibility.

## 1.6.1

- Fix the merged gripper action so `Serve · Raise hand` immediately publishes
  `GRAB` without a preceding OPEN or inferred-state gate, in parallel with
  strong stand and arm raise. No separate Ball loaded action is restored.
- Remove the unused Loading zone clear action and service from the console.

## 1.6.0

- Merge ball load into `Serve · Raise hand`: the Runner applies the stronger
  serve stance, raises the hand, sends one best-effort GRAB, and advances
  directly to `Ready to Serve` without a tracking-error or gripper gate.
- Strengthen ankle pitch/roll hold and increase strike-arm velocity
  feed-forward while preserving the content-bound positions and release frame.
- Remove the separate Ball loaded button so the attended flow is exactly
  `Raise hand`, load the ball, clear the area, then `Ready to Serve`.

## 1.5.1

- Make gripper availability, command acknowledgement, and physical-grip
  confirmation telemetry-only; none can block raising the hand or serving.
- Reduce the attended serve flow to `Raise hand`, `Ball loaded`, and
  `Ready to Serve`; OPEN, GRAB, and RELEASE remain best-effort commands.
- Keep the strict command-gap guard during the timed serve while allowing the
  slower preparation trajectory to tolerate normal scheduling jitter.

## 1.5.0

- Replace the build_4 palm serve with Jeremy's content-bound manual-load
  serve025 sequence while keeping stand, serve, and policy under one native
  Runner and command publisher.
- Add explicit local Server/Receiver selection plus attended `Raise hand`,
  `Ball loaded`, `Grip secure`, `Ready to Serve`, abort-clear, and retained-ball
  cleanup controls. Gripper receipts remain publication-only telemetry.
- Require the separate physical-grip confirmation before `Ready to Serve`
  starts frame 0, and keep successful serve recovery as the server's only
  internal path to policy.

## 1.4.0

- Split the ordinary world-to-pelvis action into explicit `Cali V2` and
  `Cali V3` buttons. V2 preserves the legacy pants marker table; V3 uses the
  ten-point V03 carrier table and records its source hashes in each receipt.
- Add a stopped-lifecycle `P1 · DIRECT` / `P2 · ROTATE 180°` table-side
  selector. P2 selects the Motive UCB_P2 rigid body and rotates ball, base, and
  orientation together about the table centre before Planner/Runner use.
- Keep `TIME CALIBRATION` unchanged and visually separate.

## 1.3.2

- Keep the 10 ms selected-source wall-clock offset as the time-calibration
  hard gate, while showing chrony frequency Skew as audit-only telemetry.
- Request an `8/8` chrony burst after the supervised hard-step and bound the
  normal post-step wait to two minutes instead of stranding robot services on
  a transient 5 ppm frequency-uncertainty estimate.
- Recycle only the completed coordinator process after a system clock step so
  Foxglove status publishers return with a fresh DDS participant; persisted
  status prevents the hard-step from being repeated.
- Show Skew explicitly beside the NTP offset so it cannot be confused with
  ROS 2 message latency.

## 1.3.1

- Recover a previously managed, same-boot MDU `STOPPED` receipt automatically
  before time-calibration preflight; never start an arbitrary stopped baseline.
- Surface structured MDU preflight and recovery reasons in the console instead
  of reducing stdout-only helper failures to `exit code 1`.
- Require MDU PTP and `agibot_pm` to be active before the hard-step transaction.

## 1.3.0

- Add an attended `TIME CALIBRATION` action for the fixed runbook 10.4
  stop, single hard-step, and dependency-ordered restore sequence.
- Require a fresh failing NTP gate, a stopped lifecycle, and confirmed network
  configuration; refuse a second hard-step in the same maintenance cycle.
- Share an exclusive hardware-operation interlock with `START SYSTEM` so a
  second window or direct service client cannot race lifecycle startup.
- Show persistent calibration progress after the expected temporary 8766
  disconnect. Failures keep robot vendor/PTP services stopped while restoring
  chrony and the Foxglove control plane for diagnosis.

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
  calibration JSON while retaining live `world -> UCB_P1 -> pelvis_link` runtime
  localization.
- Keep Runner sequence state and role controls inside a responsive bordered
  header so the row no longer overflows the card.

## 1.1.0

- Make Calibration one fail-closed operation: recalculate the ten-marker
  `UCB_P1 -> pelvis_link` receipt, wait for a fresh matching `world -> pelvis_link`
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
