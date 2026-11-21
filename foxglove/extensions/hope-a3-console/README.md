# HOPE A3 Console

PR #90 local integration adds raw P1/P2 end verification at START, a 20 ms marked
pelvis hold with a 1 s mocap-loss recovery deadline, and independent Console
(1 s) / operator-relay (1 s) challenge echoes. Use matching Runner, operator
relays, consolidated HDU operations/gateway and the locally rebuilt Console.
See [the safety contract and qualification limits](../../../docs/operations/foxglove_lifecycle.md#player-assignment-and-mocap-loss).
This source integration does not establish installed runtime or hardware qualification.


Console 1.8.15 adds **MOCAP SYSTEM**: None, Motive / NatNet, or QingTong /
CMTracker / VRPN. Confirm the source while stopped, then use its displayed
Laptop setup command. VRPN has its own IP, port, units and V2/V4 pants selector;
Motive keeps its live ModelDef workflow. See
[mocap setup](../../../docs/operations/foxglove_mocap_setup.md) for paired deployment,
asset/world confirmation and source switching. `START SYSTEM` still manages
Runner startup separately.

The **Ready** card includes the PPP3–PPP4 y filter. Inputs are ordered **left
boundary (larger y)**, then **right boundary (smaller y)**, in metres. Checking
the box immediately shows two orange bars and a faint green lane in the existing
left 3D panel; editing moves them and unchecking hides them. This is a local
preview built from an inline URDF layer through Foxglove's layout API, requiring
neither HDU, Planner, ROS topics, nor the asset server. Existing camera, table,
ball and pelvis settings are preserved. The preview draft is saved in the layout.
The old `/hope/planner/y_filter_markers` layer is hidden to avoid overlapping
live and draft ranges. No topic needs to be manually enabled.

**Apply to Planner** remains separate: Planner validates and saves the entire
setting atomically, and only fresh diagnostics confirm the applied range. Apply
is disabled while disconnected. The local preview never claims the robot's
filter is active, and an unapplied restored draft is not overwritten by telemetry.
Defaults are left y = -0.5543593239924284, right y = -1.25 m (operator update
2026-10-16). A saved edited flag distinguishes drafts from applied values:
fresh Planner receipts update the latter, while genuine pending edits remain
protected. The first local-preview version's automatically saved old default
(-1.38959) migrates to -1.25; explicitly edited values are preserved.

Foxglove extension implementing the attended operator console against the
native model_21800 Runner contract.

The panel uses one opt-in data source, `ws://<HDU-IP>:8766`. It publishes only session-bound liveness echoes on
`/hope/runner/operator_echo_hdu_flat` and cannot access generic parameters. Buttons call only the explicit
services listed in `foxglove/a3/bridge_params_control.yaml`. The
lifecycle configuration call uses a `SetParameters`-shaped request only so the
named network, mocap, pants, gripper and P1/P2 settings can be transported; the
dedicated server rejects every other name/type and accepts confirmation only
while stopped.

```bash
npm install
npm run lint
npm run package
```

Install the generated `.foxe` through Foxglove Desktop's Extensions screen,
then import `foxglove/layouts/model21800_console.json`.

Keyboard shortcuts work throughout the Foxglove window, including after an
action button disables itself or focus moves to another panel:
**I → Init** (the former Stand), **R → Ready**,
**S → the currently available Serve stage**, and **hold ← + → together
→ Runner Passive**, in either order. They click the corresponding enabled
buttons, including the same local ball intent and Runner service request.
Both Start to Serve and Serve send immediately, without a confirmation dialog.
S selects Start to Serve in IDLE/COMPLETE/ABORTED, or Serve in WAIT_READY_TO_SERVE;
it does nothing between stages. Release S and press again for the next stage.
The buttons retain their role, mode, freshness and identity checks.
Init still calls the existing PD Stand action; Ready requires a fresh, usable
Runner in PD Stand with the receiver role.
Runner Passive retains its zero-gain behavior and removes active support.
Typing in inputs, IME composition, Ctrl/Alt/Meta combinations and open
confirmation dialogs suppress shortcuts. Held keys do not repeat commands;
release both arrows before another Passive chord. Changing focus, clicking or
leaving the window clears held keys. If multiple visible Consoles are open,
click the Console to control first; a key is sent to only that Console.

Before start, the operator chooses `UCB_P1 · DIRECT` or `UCB_P2 · ROTATE 180°` and
confirms configuration. P2 tracks Motive rigid body UCB_P2 and rotates every ball
and rigid-body pose about the table centre, so both end robots consume the same
policy convention. The selection is locked while a lifecycle is active.

The no-OptiTrack receiver test sequence shown by the UI is Init, then Ready.
`START SYSTEM` does not launch or wait for the Laptop OptiTrack stack, and
the optional Motive address may be left empty. `Ready` does not require a fresh
base packet. The native Runner still owns the
role/mode transition and all of its fault checks. With OptiTrack absent there
is no live ball/base stream, so this proves Runner entry and attended motion
behavior rather than incoming-ball performance.

The Console uses the current pelvis-aligned Motive asset. The former Venue
Calibration card, Load/Export JSON and Cali V2/V3 buttons are removed. Opening
the panel starts no processes.

| Control | State | Effect |
| --- | --- | --- |
| Motive Live Asset | ASSET ACCEPTED / CHECK MOTIVE ASSET / WAITING FOR MOTIVE; separate live pelvis state | Read-only current body status |
| Marker audit | Visible / defined markers, e.g. 15/17 | Definition count follows the streamed ModelDef |
| Serve CSV | Select/upload while stopped | Existing Runner CSV validation and serve controls |
| OptiTrack / Normal play | Select while stopped; start OptiTrack separately | Uses the imported asset's nominal pelvis frame |
| Recording | OFF / STARTING / ON / STOPPING | Explicit MCAP start/stop |

For the 17-sticker shell, import
[A3_v3_17_stickers.motive](../../../agibot/mocap_sticker_shell_v3/motive_asset/A3_v3_17_stickers.motive).
Its internal NodeName, AssetName, JointName and all constraint joint references
are `UCB_P1`. Keep its pivot/axes unchanged. Disable any old body with the same
canonical name, select P1 in the Console, set Motive Streaming Up Axis to Z,
and enable Rigid Bodies. Enable Labeled Markers for the physical visibility
audit. Then use the existing OptiTrack start control.

The default launch and Laptop lifecycle helper use `localization_source:=motive_live`.
This mode reads no local Motive file and imposes no fixed SHA-256, marker count
or marker layout. It accepts the selected `UCB_P1`/`UCB_P2` body with a valid
live ID and structurally valid ModelDef, assuming the imported origin/axes
already describe `pelvis_link`. The marker monitor's `expected_count: 0`
follows the live definition; it never caps a 17-marker body at 12.

`ASSET ACCEPTED` means the current body definition was accepted, not that its
geometry was compared with a reference. The status text identifies live versus
pinned mode. Stale data, malformed definitions, source timestamps, world receipt,
pose jumps and invalid quaternions retain their existing checks. The identity
extrinsic, table-side transform and single policy Z offset preserve the schema-2
base packet. Its provenance ID is not an asset-file checksum.

This mode assumes pelvis alignment; it does not measure the physical mounting
or verify the Motive table origin. Set the table/world origin in Motive as before.
`TIME CALIBRATION` remains a separate operation. Runner captures shared HOME
on policy entry; no manual x_hit refresh is needed. No
`calibration/ucb_robot_to_pelvis.json` is read, created or overwritten.

The pinned 17-marker check is opt-in via `localization_source:=motive_asset`;
legacy measured V2/V3 calibration is opt-in via
`localization_source:=calibration start_calibration:=true`.

The 17-sticker file was copied from the user-supplied USB file, byte-identical
before renaming to the asset at upstream commit
[`10b1bd710`](https://github.com/hitchopen/HOPE/commit/10b1bd710c074cf9851a3ca2f63494dc23a39e06).
Only the 21 exact body-name/reference attributes were renamed from `P1` to
`UCB_P1`; marker coordinates, pivot, axes, IDs and other settings are unchanged.
The USB original is backed up as `A3_v3_17_stickers.motive.original-P1.bak`.
The older 12-sticker asset and original exports under `motive_asset/source/`
remain available. These offline preparation checks do not establish successful
Motive import or live NatNet tracking.


Pure Serve uploads accept the existing full31 serve025 CSV schema: 468 frames,
100 Hz, 31 SDK joints and the existing event/limit/velocity contract. The MDU
runs the same C++ parser through `--validate-serve-timeline`, which exits before
backend or gripper initialization. Successful uploads are stored by SHA-256;
invalid uploads preserve the previous selection. The hash is checked before
Run and again on MDU launch. Only Pure Serve uses the uploaded selection;
normal play continues to use the package's built-in serve. The Runner's
`--serve-only` prevents policy/receiver/reference modes even for keyboard or
other clients. Its final CSV frame returns through the existing Stand entry
blend. Normal serve still hands off to native receive policy.

In this build_6 checkout, the HDU's `DistributedRecorder` coordinates the
session-owned Laptop, HDU and MDU capture workers. Recording starts OFF after
supervisor boot and automatically starts after a successful START SYSTEM;
Record ON/OFF controls MCAP, Runner trace/obs, Planner and mocap/flight CSVs,
including the MDU body-drive/IMU recorder. Kernel mode only requires its MDU
worker. This supersedes the earlier MCAP-only recording description.
The UI displays actual STARTING/ON/STOPPING/OFF/ERROR and segment identity.
Closing the panel does not cancel recording; use OFF and wait for OFF.
OFF does not stop Runner/HAL, while lifecycle KILL requests recording closure
as part of stopping the session. Routine diagnostic logs remain active.
See [three-machine recording](../../../docs/operations/foxglove_lifecycle.md)
and the [field runbook](../../../docs/operations/deploy_and_gate3.md) for capture
contents, collection and the distinction between exporting and stopping.

The field API is `/hope/field/command`, a dedicated `SetParameters`-shaped
service accepting one string parameter named `request` with a fixed JSON
operation. Status is `/hope/field/status`. Content is sent as JSON on SSH stdin
to fixed laptop/MDU helper endpoints; clients cannot choose remote commands or
destination paths. Persistent selections are in
`/var/lib/hope-lifecycle/field/selection.json`; MDU CSVs are in
`/agibot/hope-field-assets/`. The writable MCAP directory is
`/agibot/data/bag/hope_pingpong_telemetry/`.

For the 12-sticker update, rebuild Laptop `hope_bringup`, update the staged
Laptop marker monitor and runtime helper, and install Console 1.8.12. Update
the HDU supervisor to stop polling legacy venue files. This change does not
require replacing the Runner or its policy. Broader deployment changes still
follow the wired runbook. The shared HDU Python
modules include `hope_field_assets`, `hope_field_operations`, `p1_calibration`
and `base_pose_contract`; the MDU needs `hope_field_assets`. Installing only the
extension does not upgrade the remote Runner.

The gripper indication is intentionally command-publication telemetry, not a
finger or ball sensor. The operator loads only after the raised hand has
stopped, clears the loading/swing area before play, and has a person ready
before an aborted retained-ball OPEN.

The CPU card reports aggregate HDU load and the process with the largest CPU
delta. E-stop is assert-only: the panel follows the persistent HDU latch and
does not expose a reset operation.

The system-lifecycle card is backed by the separately installed HDU supervisor
documented in `docs/operations/foxglove_lifecycle.md`. `START SYSTEM`
replaces runbook STEP 0/1/2A/2B/4/5 and leaves the Runner in PASSIVE; it does
not replace physical robot support or access to the hardware E-stop.

`TIME CALIBRATION` is a separate attended maintenance action for runbook 10.4.
It is enabled only with confirmed network configuration, a stopped lifecycle,
and a fresh NTP gate (passing or failing). The fixed root coordinator stops MDU consumers
before HDU consumers, performs at most one hard-step per attended request,
restores in dependency order, and publishes persistent status after the expected
temporary 8766 disconnect. Before preflight it may recover only an exact,
same-boot MDU `STOPPED` receipt created by the managed helper; arbitrary stopped
services remain fail-closed. Structured helper reasons are shown directly in
the panel. It shares an exclusive operation interlock with `START SYSTEM`; the
panel exposes no clock parameters or shell input. The NTP hard gate is selected
source + Leap Normal + at most 10 ms wall-clock offset. Chrony Skew is displayed
as frequency-uncertainty audit telemetry and is deliberately separate from the
ROS 2 message-latency card.

## Public release

The earlier reviewed main installer is
[hopeopen.hope-a3-console-1.8.9.foxe](hopeopen.hope-a3-console-1.8.9.foxe).
It includes main's 24-sticker calibration interface. For the new 12-sticker
asset workflow, use the locally built
[Console 1.8.12](hopeopen.hope-a3-console-1.8.12.foxe) described above.
See [current Runner/serve behavior](../../../docs/operations/runtime_xbox.md)
for Kernel completion, v12 timing and release limitations.

V1.8.14 adds the V2 / V4 pants selector for VRPN. See
[VRPN setup](../../../docs/operations/vrpn_pants_profiles.md) for matching CMTracker definitions.

## Ball physics choice (2026-10-16 review branch)

The Ball physics model selector offers **Berkeley original (2026-10-10)**,
**local one-star PPQ002**, and **local three-star PPQ004 unified (2026-10-20)** while the lifecycle is STOPPED. It saves matching
14-parameter overlays on HDU and Laptop, with rollback on partial failure.
Restart the separately managed Laptop mocap/packetizer, then START SYSTEM.
Actual ROS parameters are checked before HAL/Runner start; the console separates
the saved choice from the current session's verification. PPQ003 is an archived
fit of the same local ball and is deliberately not labelled Berkeley.

Deploy the matching supervisor, field operations, `hope_ball_models.py`, lifecycle
helpers and updated bringup launch files with this extension. The matching field
backend was deployed on 2026-10-21. `/hope/field/status.ball_model_status` is refreshed
every 15 seconds from both machines' effective configuration and all 14 live ROS
parameters. Directly installed configurations are identified even without a saved
overlay selection. The dropdown uses this verified configured model, while the
status distinguishes CONFIGURED, LOADED, MISMATCH and UNAVAILABLE and shows each
node separately. Missing or older-than-45-second verification is shown as waiting,
never as a current model. A stopped node is not labelled loaded. Normal START
checks both configurations and rejects any running process with different physics;
Kernel operation retains its independent admission path.

### Runtime plane selector (1.8.16)

Ready controls now include **击球平面 · 下一球**. On an upgraded Planner,
START does not lock this selector: fixed/session HOME, per-ball current position,
or posture-stable updates apply only to a new ball. Current commands remain
frozen. The panel reads back requested mode, last actual flight, reference X,
world plane X and the offset exported by the loaded policy. Unknown/stale Planner
state disables editing. No Runner restart, rearm, or HOME recapture is requested.

The selection resets to the Planner startup default on restart. The experimental
stability gate uses 200 ms of waiting/low XY speed/low angular speed/low tilt;
it has no foot-support measurement. Runner/Planner must both support schema-5
per-flight references; installing this UI alone does not upgrade them.

### 击球力度 scale

打球区域 Ready 附近新增「击球力度 scale」，默认 `1`。START 后可以输入非负倍率并点击
「应用力度」；面板等待 `/planner/diagnostics` 回读确认，显示当前设置及最近已发送目标使用的倍率。
`0.8` 表示球拍目标速度向量乘以 `0.8`，`1.2` 表示乘以 `1.2`，`0` 表示零目标速度。
服务 `/hope/planner/set_hit_speed_scale` 使用 `SetParametersAtomically`，只接受一个
`scale(double)`。控制桥 :8766 需要加载新增的精确服务白名单。

修改作用于后续新发布的目标；one-shot 模式下已提交的球及其重复数据不重新计算。
倍率不会修改目标位置、法向、击球时间、Runner 模式或发球 CSV；Kernel 模式不经过 Planner，故禁用此项。
Planner 启用 `hit_speed_scale_state_file` 后，应用时先保存，再确认；重启恢复保存值。没有保存文件时使用启动默认值 `1`。面板读取 `hit_speed_scale_persistent`，旧后端不会被误标成已保存。
这是 Stage 3 规划后的球拍速度倍率，原规划的出球速度、落点和过网预测仍对应未缩放解，不能视为倍率后的落点保证。


### 回球飞行时间

力度旁的「回球飞行时间（秒）」可在 START 后输入并应用。它控制击球后到目标落点的
期望飞行时间，不是来球剩余时间或击球触发延迟。服务 `/hope/planner/set_flight_time`
使用 `SetParametersAtomically`，只接受一个正的有限 `flight_time_s(double)`，不得超过
Planner 回读的预测时域上限。Kernel 模式、旧后端或状态过期时不可操作。

未应用前沿用 `delta_t_flight`（硬件默认 0.50 秒）及现有正反手覆盖值；应用后统一覆盖
正反手，只影响后续新目标，已提交球及其重复包不重新规划。面板显示当前正反手时间、
最近已发布目标的设定时间和求解预测时间。求解器仍可采用原有备用飞行时间，所以设定
不保证等于预测；预测对应力度缩放前的物理解，也不是实测落地时间。

配置 `flight_time_state_file` 后先原子保存再确认，重启恢复；硬件配置使用
`/var/lib/hope-lifecycle/planner_flight_time.state`。写入失败保留原值，没有保存配置时
面板明确提示。启用此功能需要更新 Planner、控制桥服务白名单和 Console；单独更新
面板不能让旧 Planner 支持热调节，不需要为每次调参重启。

### 连接目标与配置保存

顶部机器人 IP / CONFIRM 连接通过 Foxglove 官方 deep link 打开目标 `:8766` 数据源，
不调用旧机器人的配置、停止或启动服务。原面板等待切换时锁定操作，可明确取消。
新机器人回读后重新加载它的配置，Y 草稿按机器人地址隔离。该入口不改变机器人网卡 IP。

停止状态下 CONFIRM CONFIG 原子保存动捕、裤子、夹爪等选择；动捕相关字段变更时，
受管重连 Laptop 接收器。接收器归属于另一台机器人或无法核实归属时拒绝停止它。
保存与连接失败分别报告；只有新鲜位姿和标记点才显示“动捕数据已连接”。
配置保存在机器人 config.json；球模型选择在 field/selection.json，参数在两端 ball_physics.json；
力度和 Y 范围分别在机器人 planner_hit_speed_scale.state / planner_y_filter.state。
这些都是运行配置，不会因打开面板而被源码默认值覆盖。

Manual TIME CALIBRATION is available even when NTP already passes, and can be
requested again after a successful COMPLETE. Each request requires fresh STOPPED
lifecycle evidence, confirmed configuration and operator confirmation. Failed or
interrupted calibration remains locked for recovery; a successful clock status
does not bypass stopped-consumer checks or final clock qualification.

## Operator ball-state gate (PR #119, Console 1.9.3)

The operator-side C++ packetizer classifies normalized P1/P2 ball tracks and
filters both ball channels. Local Ready arms OPPONENT_SERVE; local Serve arms
SERVE; local Stand/Stop holds the gate closed until explicit Ready/Serve.
Runner feedback does not control this classifier. Pelvis/base traffic remains
independent. Forecasts are advisory and cannot reject a measured trajectory.
A held ego serve requires explicit local Ready before a fresh opponent serve.

Install the matching packetizer, bringup launch files and Console 1.9.3.
Foxglove Desktop requires one-time pairing with the runtime's private
`hope-ball-controls.key`; status is at `http://127.0.0.1:8768/ball-state`.
See [state rules, installation, tests and rollback](../../../docs/operations/laptop_ball_states.md).
This operator-only integration preserves the field scheduling and physics
settings and changes no HDU/MDU service. It cannot recall a previously sent target.
