# `model_21800` 真机流程迁移到 Foxglove 的实施计划

最后核对：2026-08-11

目标是把 [`RUN_ON_AGIBOT.md`](../RUN_ON_AGIBOT.md) 描述的公开 Runner 流程，以及现场私有
runbook 中对应的操作顺序，逐步变成可观察、可审计、最终可由 Foxglove 操作的接口。
Foxglove 只承担 UI 和受限的 ROS 服务调用；它不成为通用远程 shell，也不改变
`model_21800` 的控制合同。

## 1. 已审计的上游实现

当前实现目标是 `hitchopen/HOPE` 的
`nightly_built@d28fee07e82d4326e952be66e90b63bff472c182`。其中的 Foxglove 基线来自：

- `8fea916bb910fa54b5c719667bc7d773b44846d0`：初始 Foxglove viewer；
- `ee19eb650e087cc11aaf140cb248841668392544`：2026-08-07 的增强版，也是本次基线。

`ee19eb6` 已提供并可直接复用：

| 组件 | 结论 | 用法 |
| --- | --- | --- |
| pinned Foxglove Bridge build + 两个兼容 patch | 直接复用 | A3 vendor Jazzy / `ament_index_cpp 1.8` 构建底座 |
| robot-side bridge systemd unit | 直接复用 | HDU 上的 `ws://<robot-ip>:8765` |
| Fast DDS internal-interface profile | 复用，但每台确认 `10.42.10.10` | 让 bridge 看见 HDU/MDU 内部 ROS 图 |
| `hope_monitor.py` / pure core | 直接复用 | NTP、CPU、ROS stamp latency、TF、joint、URDF、PM 状态 |
| HOPE table URDF + A3 layout | 直接复用 | 3D 场景和基础状态面板 |
| assert-only E-stop proxy | 直接复用 | 唯一的 fleet 级写操作；没有远程解除路径 |
| pure-Python invariant tests | 直接复用 | 锁定 topic/service allowlist 和急停方向 |

它不能直接承担 runbook 命令：fleet 配置明确设置
`client_topic_whitelist: ["(?!)"]`、空 parameter allowlist，并且只放行
`/hope/safety/trigger_estop`。Foxglove WebSocket 也没有认证。扩大为任意 topic、任意 service
或 shell 字符串，会把同一网段上的 UI 变成无认证机器人终端，因此不采用。

## 2. 双层结构

```text
Foxglove Desktop
        │ ws://robot:8765
        ▼
fleet layer（保持老板基线不变）
  bridge + monitor + assert-only E-stop
        │ /hope/.* read-only telemetry
        ├──────────────────────────────┐
        ▼                              ▼
V17 observer（只读）             V17 command proxy（attended control）
  schema-2/base/ball/target       固定 action ID 的显式 ROS services
  session/x_hit/Runner state      不接收 shell、argv、path 或任意 signal
                                       │
                              我方 MDU native Runner API

Laptop-only Motive/build/upload/log jobs
  └─ 保留 Laptop agent / 第二个 Foxglove data-source window；A3 不访问 Motive
```

fleet layer 始终保持通用、可回退和急停-only。V17 layer 是单独安装的 trial profile；第一阶段
只发布 `/hope/v17/**`，正好被现有 bridge 的 `/hope/.*` topic allowlist 覆盖，不需要放宽任何
写权限。

Foxglove Desktop 每个窗口只有一个 active data source。机器人内部状态接
`ws://<robot-ip>:8765`；Laptop NatNet/maintenance agent 若需要同时显示，先使用第二个窗口，
后续再评估是否值得做受限 gateway。A3 端不配置 Motive IP。

## 3. runbook 命令覆盖矩阵

“进入 Foxglove”表示把一个固定动作封装成有名称、固定参数和结构化返回值的 API，不表示把
原 shell 文本粘进 UI 执行。

| runbook 区段 | 目标机 | Foxglove 形态 | 阶段 | 当前状态 |
| --- | --- | --- | --- | --- |
| Pre-flight A：Laptop mocap build | Laptop Distrobox | maintenance job + build log/status | P4 | CLI 保留 |
| Pre-flight B：同步/构建 Planner | Laptop + HDU | versioned deploy job；固定源码目录 | P4 | CLI 保留 |
| Pre-flight C：Rockchip build/upload/hash | Laptop + MDU | versioned deploy job；无任意路径输入 | P4 | CLI 保留 |
| Pre-flight D：网络/NatNet | Laptop/HDU | reachability、MODELDEF、rate topics | P1/P4 | P1 先显示 ROS 收包结果 |
| D1：clock 状态 | HDU/MDU | NTP/PTP topics + plots | P1/P2 | fleet NTP 已有；跨机 PTP 待补 |
| STEP 0：统一 session | 三机 | `CreateSession` / `ResumeSession` | P2 | 设计中 |
| STEP 1：Laptop bridge | Laptop | `StartMocap` / `StopMocap` + log tail | P2 | 第二 data source |
| STEP 2A：base relay | HDU | fixed systemd lifecycle services | P2 | P1 已观测 schema-2 |
| STEP 2B：Planner | HDU | fixed systemd lifecycle services | P2 | P1 已观测 PID/attempt/target |
| STEP 2C：刷新 `x_hit` | HDU | `RefreshXHit`，沿用 request/status 文件合同 | P2 | nightly_built 仓库实现已完成，现场待验 |
| STEP 3：SHADOW | MDU | fixed runner start/stop，强制既有 `--dry-run` argv | P2 | 待实现 |
| STEP 4：HAL | MDU | explicit HAL lifecycle API | P3 | TTY/现场命令保留 |
| STEP 5：runner + `s/m/p` | MDU | native fixed actions；不注入伪 TTY 按键，不开放远程 `q` | P3 | nightly_built MDU dry-run/state 链路已验；有输出 bench 待验 |
| STEP 6：停止/恢复/收日志 | 三机 | ordered stop + restore + evidence manifest | P2/P3 | CLI 保留 |
| 更换控制 Wi-Fi | 本机网络栈 | 不经当前无认证 WS | 不迁移 | 本地现场操作 |
| 故障对照与 topic echo | 各机 | status topics、plots、raw-message panels | P1/P2 | 已开始 |

build、rsync、apt、Wi-Fi 等维护动作不放到当前 robot WebSocket。它们以后若需要按钮化，必须由
Laptop 侧的受限 agent 执行固定 job，并与 robot runtime control profile 分开；不能增加
`RunCommand(string)` 之类接口。

## 4. 分阶段实施

### P0：引入 fleet baseline（本轮已完成）

- 原样引入 `foxglove/` 的 bridge、monitor、layout、assets、patches 和 tests；
- 保持 `service_whitelist` 只有 assert-only E-stop；
- 忽略每台机器人复制到 Laptop 的 `foxglove/urdf/`；
- 上游 23 个 pure-Python tests 必须全通过。

### P1：V17 只读 observer（nightly_built 仓库实现已完成，待部署）

`foxglove/v17/a3/hope_v17_observer.py`：

- 解码 16-double `/a3/base_pose_flat` schema-2；
- 解码 19-double `/racket/command_flat` schema-2；
- 分开显示 observer 本机 monotonic receipt age 与 HDU ROS source-stamp age；
- 显示 `/poses` 本机 callback rate/receipt freshness；
- 读取当前 session、Planner attempt/PID、`x_hit.status`，但绝不写文件或发 signal；
- 解码我方 Runner 的 19-double 固定状态，发布 mode、local role、boot/sequence、fault、
  publishing 和真实 serve-controller state；
- 仅由 `local_role` 推导 opponent expected role，并永久发布 `role_confirmed=false`；
- 只发布标准消息到 `/hope/v17/**`，没有应用层 ROS service、parameter mutation 或 client
  publisher。observer 关闭 parameter services；Jazzy 自动生成的 type-description service 不在
  bridge allowlist 中，因此不能从 Foxglove 调用。

配套 layout 是 `foxglove/layouts/v17_model21800_observer.json`。这些状态只用于 operator 和日志
判断，不接 Planner release、runner `m` 或任何新 pass receipt。

P1 离线验收：

1. schema-2 正常、explicit-invalid、malformed packet 单测；
2. session/attempt/PID/x_hit parser 单测；
3. layout 中不存在 `ServiceCall!` 或 `Publish!`；
4. fleet bridge allowlist 不变；
5. HDU 部署后只读验证 topics，并确认 Foxglove 看不到 observer 的自动 type-description
   service；不触发急停。

### P2：非电机输出生命周期（本轮已开始）

新增显式 supervisor，不开放任意参数：

- session create/resume；
- Laptop mocap、HDU base relay、Planner、MDU SHADOW 的 start/stop/status；
- `RefreshXHit` 复用已有 request/status 文件；
- evidence manifest 和日志收集状态。

动作返回“已接受/失败 + 当前状态 + audit ID”，但不形成控制 gate。现场顺序仍是先进入
PD_STAND、机器人站稳、再刷新 `x_hit`；刷新成功后才由操作员决定是否进入 MOTION。不会新增
READY、stable-frame、confidence、station、source-age 或平衡门控。

每个进程改为独立 systemd unit 或等价的固定 argv wrapper。supervisor 只能选择枚举 action，
不能接收 shell、argv、环境变量、文件路径、PID 或 signal 编号。

P2 最初实现的固定 action：

- `/hope/v17/refresh_x_hit` (`std_srvs/Trigger`)；
- 后端只从固定 session/attempt/PID 文件解析当前 C++ Planner；
- request 先完整写入同目录临时文件，再用不覆盖现有 request 的原子 hard-link 发布；
- 只接受与本次 request ID 匹配的 `x_hit.status`；超时返回“未确认”，不会删除或覆盖请求；
- 单独的 control bridge 使用端口 `8766`；P2 初始切片只放行 assert-only E-stop 和这一个
  action，P3 再逐项加入固定本机 Runner service；
- `8765` fleet bridge 的配置和进程保持不变。

这条服务不会读取 runner mode，也不会自行执行 `s` 或 `m`；PD_STAND 稳定后的现场顺序仍由
操作员负责。剩余 session/process lifecycle 继续按本节设计实现。

### P3：我方 Runner 角色和模式切换（nightly_built 仓库实现已完成，待验收）

Runner 增加 native control endpoint，并让键盘和 API 共用同一动作队列与转换逻辑：

- `SET_SERVER` / `SET_RECEIVER` 只维护我方 `local_role`；仅在 PASSIVE/PD_STAND 且无
  command fault 时允许，角色变化不触发任何 `q_des`、Planner、x_hit、MOTION 或 SERVE；
- `PD_STAND` 等价于键 `s`；
- `MOTION` 等价于键 `m`；
- `PASSIVE` 等价于键 `p`，UI 必须明确“会失去支撑力”；
- `READY_TO_SERVE` / `SERVE` 复用键 `v` 的两阶段发球合同；后者只在真实
  `AWAIT_BALL_ON_PALM` 状态接受；
- 第一版不开放远程 `QUIT`；Runner/HAL 退出仍由现场生命周期流程负责；
- assert-only vendor E-stop 继续是独立的紧急路径，绝不增加远程 reset。

内部 request/state 使用固定 `Float64MultiArray` schema；浏览器无权直接 publish。端口 8766
只精确 allowlist 七个 Runner `Trigger` service、`RefreshXHit` 和 assert-only vendor E-stop。
完整冻结合同见
[`foxglove_v17_local_runner_control_contract.md`](foxglove_v17_local_runner_control_contract.md)。

不使用 PTY 按键注入，因为它无法可靠证明请求、当前 mode、结果和 audit identity 来自同一条
控制路径。P3 先完成 x86/unit/MuJoCo 和无电机 bench，再做有人支撑、实体急停可用的硬件验收。

P3 不修改 ONNX、`deploy.yaml`、110-D observation、31-D action、schema-2、policy-native q_des、
动态边界 frozen-target 或 post-swing 纯策略合同。

### P4：维护面

把 Pre-flight A/B/C、部署 hash 和收集/回放封装为 Laptop fixed jobs。Foxglove 只显示 job ID、
进度、退出码和 artifact hash；job definition 仍在代码仓库中，UI 不能提供任意命令文本。

## 5. 安全和回退边界

- 当前 WebSocket 无认证，只能位于可信实验室网段，不做端口转发；
- fleet config 永久保留急停-only；V17 control config 以后单独 opt-in；
- 不提供软件急停远程解除；
- 不让 A3 访问 Motive 网段；
- P1 telemetry 不是 readiness gate，也不是接触、落点或真机平衡证明；
- 每个新增 unit 都必须可单独 disable/remove，不修改 vendor unit 文件；
- 任一 V17 layer 异常时，可停用新增 unit 并回到当前 runbook 的 SSH/TTY 流程。

## 6. 当前交付边界

本轮完成 P0/P1、P2 的 `RefreshXHit`，以及 P3 的本机角色、Runner 状态、`s/m/p` 和两阶段发球固定接口，
并已迁移到公开 `a3_deploy/` 目录。`build_2` 上曾做过的构建或硬件结果没有作为本分支证据。

`nightly_built@d28fee07` 的独立验证结果：

- Foxglove Python invariant tests：54/54 通过；
- x86 Runner 与 `run_tests` 编译通过；`PpRunnerControl` 测试 11/11 通过；
- 全量 C++ 为 263 通过、3 跳过、5 个既有环境型失败：4 个来自该 x86 配置关闭 AimRT，1 个
  来自未公开的 FK fixture；
- Rockchip/AimRT/ROS 2 交叉构建通过，AArch64 Runner SHA-256 为
  `b6d0cb132e97656aad8fcb4e5179cb0b95ab34afca072c42267f368739400418`；
- MDU 在 `--dry-run --start shadow` 下完成 AimRT 初始化，六类 body state 均 ready，日志确认
  `publish_enabled=false`；HDU 在 domain 232 实际读取到 19-double Runner state，内容为 SHADOW、
  `command_publishing=0`、`policy_native=1`、无 command fault；测试后无残留 Runner，
  `agibot_pm=active`。

本次没有完成现场 service request -> Runner acknowledgement：准备继续请求测试时 HDU 再次在约
10 分钟窗口重启，SSH jump 中断。因此 repository wire、codec 和状态发布已有离线/MDU 证据，
但原五个 browser-facing service 的现场动作验收仍为 pending；新增两阶段发球服务当前只有
离线状态机验证。

当前没有宣称 live `RefreshXHit`、厂商 E-stop、有电机 PD_STAND/MOTION、Ready-to-Serve、Serve
或接触验收通过。Rockchip/AimRT/ROS 2 和 MDU 证据属于扩展前的五动作版本；本次新增发球动作
尚未重新交叉构建或部署到 MDU。
