# 新 A3 机器人：首次安装与部署验收

2026-10-31 已结合新 P1 机器人在线安装、双板重启及 START 排障修订。
本页给出部署与验收流程；具体机器是否通过，仍以本次 boot/session 的实测记录为准。

## 1. 先确定部署范围

用户说明“新机器人”“换了 HDU/MDU”或“重刷 BSP”后，“部署”必须包含系统基线、
应用安装和实际验收，不能默认沿用上一台机器的 NTP、PTP、SSH、依赖、服务、配置或标定。
已有机器人也只有在本机基线有当前证据时，才走增量程序/策略更新。
单独上传 policy、构建 ARM 包、安装 Console 或启动到 PASSIVE，只能报告对应阶段完成。

先记录本次物理机器人/板卡身份、主机名、架构、boot ID、实际网口/MAC/路由、SSH 指纹、
源码版本与本地差异、所选模型和目标安装槽位。旧手册的 IP、模型、hash 和成功快照只属于
原设备。保留现场配置并备份待替换文件，不要覆盖整棵有未提交修改的工作树。

采用 [首次安装手册](foxglove_first_hardware_test.md)的逐机命令；其历史模型、地址、
分立 observer/proxy 示例必须先按[当前节点结构](node_architecture.md)和
[匹配组件升级](foxglove_lifecycle.md#upgrade-the-supervisor-console-and-container-together)核对。
全新 HDU 的 operations 初始化见 [安装控制服务](foxglove_lifecycle.md#install-the-hdu-supervisor)。
原生 C++ operations 的激活流程属于 [PR #102](https://github.com/hitchopen/HOPE/pull/102)，
仅在同时采用该版本时使用其部署工具；当前 main 的 Python 服务继续遵循本库安装流程。
PR #102 的 `install_native_operations.sh` 和 `hope_operations_deploy.py stage` 只准备 operations
候选，不负责安装全机时钟、依赖和网络，也不自动激活服务。

## 2. 按顺序完成，逐项保留证据

| 阶段 | 必须安装/核对 | 验收与操作入口 |
| --- | --- | --- |
| 身份、网络、SSH | Laptop↔HDU、HDU↔MDU、HDU→Laptop 的实际路由和免交互 SSH；现场 `network.env`、DDS peers/interface、Console 地址一致 | 首次手册 §0、§3；有线场景使用 `foxglove/laptop/check_wired_connection.py`。四向 SSH/WebSocket 成功仅证明连接 |
| Laptop 环境 | HOST 工具、ROS `hope` 环境、匹配消息/球包器/动捕桥、lifecycle helper、布局/Console、资产服务；使用 Xbox 时安装并验证输入服务 | [环境安装](../DISTROBOX_SETUP.md)、首次手册 §2、§8–9；核对实际安装路径和版本 |
| HDU 时钟基线 | chrony、bootstrap、区域上游源、vendor/PTP 所有权、启动顺序、当前 Laptop 地址的 NTP ACL | [时钟安装](clock_baseline_installation.md) §2–4；`HDU_CLOCK_BASELINE_OK` 之外仍需下文在线同步证据 |
| Laptop 时间同步 | chrony 实际加载当前 HDU source，旧 HDU source 已审查 | [时钟安装](clock_baseline_installation.md) §5；`LAPTOP_CURRENT_HDU_NTP_OK`，当前 HDU 是选中的源 |
| MDU 系统与 PTP | 匹配 BSP/HAL、硬件 timestamp、PTP units/worker 所有权、CPU/affinity、helper 和最小 sudoers | 首次手册 §7.2；`MDU_CLOCK_BASELINE_OK`、`MDU_FRESH_IMAGE_PREFLIGHT_OK`；核对 PTP `SLAVE`、`gmPresent=true` 和持续更新的同步观测 |
| HDU 控制面 | 匹配的 Planner、native base relay/gateway/IMU、monitor、operations、8765/8766 bridge、校时 coordinator/core、ELink 模块 | 首次手册 §6 与[当前 prerequisites](foxglove_lifecycle.md#one-time-prerequisites)；检查有效 unit/drop-in、实际 PID/可执行文件与参数，而非仅 `enabled` |
| MDU 应用 | 完整 ARM Runner 包及依赖、ONNX/deploy、runtime/CPU 配置、policy catalog/selection、所需 Serve/teleop/夹爪资产 | 首次手册 §7 与[部署](deploy_and_gate3.md)；活动槽位、实际文件 SHA、启动参数相符；receive-only 不要求不存在的 Serve 能力 |
| 动捕电脑与数据链 | 实际使用的 Windows/Motive 或 QingTong/VRPN 时间源、输出、P1/P2/单位/坐标、机器人对应资产 | [动捕设置](foxglove_mocap_setup.md)；时间与帧计数均验收。旧机器标定不得直接移植 |
| 受管启动 | 基线检查通过后，经既有 lifecycle 启动；核对唯一控制 owner、选定策略、实际进程、session、Runner 新鲜 PASSIVE/tick/IMU | [生命周期](foxglove_lifecycle.md)；PASSIVE 是运行中的 Runner，不是 STOPPED，也不证明动作或击球通过 |
| 持久性 | 现场允许的受控重启后，复查同机身份、网络、unit/drop-in、同步和控制面 | 首次装机未做重启验证时明确写“持久性未验证”；不能由一次手动启动推断开机即正常 |

安装或校时涉及停止厂商/时钟服务时，按对应手册进入有人值守、物理支撑、实体急停
可触达且无 Runner/Policy 的维护状态。不能因为请求“部署”就跳过现场状态核对。
日常 START 不负责装包，也不自动 hard-step。

## 3. NTP/PTP 必须验收到哪一层

此仓库首次安装手册的授时方向是：

```text
已选定的外部 NTP 上游 → HDU chrony / CLOCK_REALTIME
                         ├─ NTP → Laptop
                         └─ phc2sys → eth_hdu PHC → PTP → MDU PHC → MDU CLOCK_REALTIME
动捕电脑：单独记录并验证其时间源、与此链路的偏差及输出帧时间语义
```

Laptop 为 HDU 提供网络共享不代表 Laptop 是 HDU 的 NTP 上游；不能同时配置
HDU→Laptop 和 Laptop→HDU 互相授时。离线时内部 PTP 可以工作，但外部 UTC
可能不合格，不能据此宣布跨机器时间已同步。

以下命令分别在 **HDU 原生 shell** 和 **Laptop HOST** 只读执行并保存输出：

```bash
hostname -s
cat /proc/sys/kernel/random/boot_id
systemctl is-active chrony.service
chronyc -n sources -v
chronyc tracking
chronyc waitsync 1 0.010 0 1
```

HDU 要有实际选中源 `^*`、`Leap status: Normal`，绝对系统 correction ≤10 ms；
Laptop 的 `^*` 应指向本次 HDU。检查有效配置加载了哪些 sources 和 ACL，而不只是
配置文件存在。监控默认 `ntp_max_offset_ms=10.0`、`ntp_max_skew_ppm=0.0`；
Skew 是频率估计不确定度，默认仅审计，不能当作当前毫秒偏差。记录现场实际参数，
不要放宽阈值或使用 local reference 伪造通过。

MDU 按首次手册 §7.2、§10.3 检查 `agibot-mdu-ptp4l.service` 和
`agibot-mdu-phc2sys.service`、唯一 worker、`pmc` 数据和日志。`SLAVE`、
`gmPresent=true` 是必要的拓扑证据，还需核对同步偏差和跨板消息时间持续正常。
HDU 的 NTP 灯不涵盖 Laptop、MDU 和 Windows 的全部时间验收。

使用 Windows 动捕时，在该电脑只读记录 `w32tm /query /source`、
`w32tm /query /status`，并测量它相对已确认时间源的偏差；服务 running 或服务器 TCP
可达都不够。分别统计原始动捕、坐标转换、relay、球包器/Planner 的连续帧与原始时间。
不能用接收时间替换源时间来掩盖负龄、跳变或陈旧。未启用动捕的模式写明“不适用”，
同时把接球输入验收标为未完成。

需要修正偏差时，先区分“时钟基线未安装”和“基线已安装但未同步”：前者走首次
手册 §6.2.1/§7.2，后者按
[受控校时](foxglove_lifecycle.md#pre-operation-clock-correction)排查源/网络并维护。
Time Calibration 不会安装缺失的 NTP/PTP。当前 Console 允许 NTP 新鲜且通过或失败
时显式发起校时，但还必须满足 STOPPED、idle、配置确认和 coordinator 就绪；
PASSIVE 不满足停止要求。完成后要有 `COMPLETE`、`SERVICES_RESTORED`、新鲜
NTP gate，并重做三机检查；失败/中断不能靠重复点击跳过恢复。

### 新 BSP 的 `timesync_guard.py`：必须先分支

2026-10-31 新机现场发现，厂商 `run_agibot.sh` 会调用 `system/timesync_guard.py`：
只在 MDU uptime 小于 180 秒时启动 HDU PTP master。MDU 已运行较久，或 MDU 的
uptime 服务不可达时，单独重启 HDU `agibot_pm` 不会恢复 PTP。检查**实际 launcher**，
不能只检查 `timesync.sh` 含有 ptp4l/phc2sys 就认定旧安装器适用。

- 普通 `--install` 现在会在修改服务前拒绝这种 BSP；禁止删除 guard 或伪造 MDU uptime。
- 此 BSP 首次安装使用 `install_hdu_clock_baseline.sh --install-for-paired-reboot PACKAGE_ROOT`。
  先按有人值守维护流程持有硬件操作锁、停止 HOPE 会话和两板时间消费者，并备份服务状态；
  HDU PM 在进入安装器时仍须 active，由安装器负责停下。MDU 必须已停止厂商服务、HAL、
  Runner 和 PTP；安装器通过已信任的 HDU→MDU SSH 再次只读验证，失败则不安装。
- 该模式保留原厂 launcher/guard，只安装固定资产并留下 HDU vendor 服务 stopped，返回
  `HDU_CLOCK_BASELINE_STAGED_PENDING_PAIRED_REBOOT`；这不是完成标志。当前原始安装器
  选择 China profile，其他区域必须先准备并核对对应的区域安装流程，不能沿用 China。
- 两板消费者停止期间，按既有 bootstrap 流程校准 HDU UTC、恢复 continuous chrony 并
  验证 10 ms gate；然后协调重启两块板，使新启动的 HDU guard 观察到真实的新 MDU boot。
  重启前核对原来启用的服务及现场配置，重启后重新读两板 boot ID、服务、哈希、PTP 和 UTC。
- RTC 分辨率可能使重启后仍有几百毫秒 correction；重启前合格不能代替重启后验收。
  等 continuous chrony 收敛并再次确认 ≤10 ms。此次有人值守维护曾临时使用连续 slew，
  结束后恢复固定 `maxslewrate 100` 与配置 hash；此临时维护调整不属于日常运行配置。
- 未安装匹配适配器时，Time Calibration 在任何 MDU recovery/stop 之前返回
  `GUARDED_BSP_REQUIRES_PAIRED_REBOOT`，保持当前服务运行。完成下述按钮兼容 setup 后，
  受控校时可用一次性维护凭据恢复 PTP；正常开机仍执行原厂 guard。不能删除保护或伪造 uptime。

### HOPE 开机一次性校时 setup

HOPE 场景要求正常整机开机后自动完成校时。通用 `agibot/ntp_sync` 基线为了保留原厂
离线行为，默认禁用 bootstrap；只安装该基线会把 RTC 的数百毫秒误差留给
`maxslewrate 100` 慢慢消除。NTP 黄灯此时表示 correction 尚未达标，不能解释为
“同步成功但没有显示”。Time Calibration 被 guard 拒绝时，按钮没有执行任何一次性校时。

在已完成时钟基线安装的 HDU 上，额外执行以下 HOPE setup。脚本和同目录的
`30-hope-clock-boot-ordering.conf` 必须一起同步；先检查实际区域上游源能访问。

```bash
cd ~/HOPE
sudo bash foxglove/a3/configure_hdu_boot_clock.sh --enable-next-boot
```

该脚本验证固定 bootstrap/helper/config 身份、备份原状态，补上 bootstrap 先于 HOPE
控制面/DDS 的顺序并 enable；它不会启动 bootstrap、改当前时钟、停止服务或重启机器人。
输出 `HDU_BOOT_CLOCK_STAGED_NEXT_BOOT` 只表示下次开机配置完成。重新安装通用基线后
必须重新执行此步，因为通用安装器会恢复 disabled。

随后在已支撑、急停可触达、无 Runner/策略且持有硬件维护锁的条件下，协调两板重启。
正常顺序是：网络 → 一次性 bootstrap → continuous chrony → 厂商 guard/PTP 与 HOPE。
原厂 guard 继续检查真实 MDU uptime；不能伪造 uptime 或在热运行中强行调用 timesync.sh。
单板重启、MDU 启动过慢或 uptime 服务未就绪时，仍可能拒绝分发，必须检查日志。
初始 NTP 不可达时，bootstrap 有界失败后保留原厂离线启动与 continuous chrony；
外部时间 gate 仍须通过，不能承诺断网也能立即开始跨机器动捕任务。

重启后在 HDU 检查：

```bash
cat /proc/sys/kernel/random/boot_id
systemctl is-enabled agibot-clock-bootstrap.service
systemctl show agibot-clock-bootstrap.service -p Result -p ActiveEnterTimestampMonotonic
journalctl -b -u agibot-clock-bootstrap.service --no-pager
test -e /run/agibot-time/bootstrap-qualified
chronyc tracking
chronyc waitsync 1 0.010 0 1
systemctl show agibot_pm.service hope-lifecycle-supervisor.service -p ActiveEnterTimestampMonotonic
```

bootstrap 应为 enabled、Result=success，完成时间早于厂商 PM 和 HOPE；当前 correction
≤10 ms，Console `/hope/ntp/gate_pass` 为新鲜 true。另在 MDU 读回新 boot ID、PTP
`SLAVE` / `gmPresent=true`、持续更新的 master offset 与 phc2sys system offset，
按上文核对 Laptop/动捕电脑。一次性的 bootstrap 完成后不再调整时钟；continuous chrony、
HDU/MDU 的 PTP workers 和机器人所需的 PM/HAL 保持运行。`timesync_guard.py` 自身检查完就退出。

2026-10-31 本次新机验证：重启前 HDU correction 约 331 ms；启用上述顺序后，12:56:46
bootstrap 自动修正 -477.378 ms（本次 RTC 重载后的误差），随后 `bootstrap-qualified`、
`rtc_updated=yes`。ROS 实测 NTP gate=true、lifecycle=STOPPED；MDU 后续进入 SLAVE、
gmPresent=true，单次 master offset=-65 ns，phc2sys offset=-110 ns。HDU/MDU boot ID 均变化。
这些是该次启动的时钟验收，不代表策略/动作或整场 demo 已验证。按钮的 guarded-BSP
适配已具备维护凭据路径，见下节；该次实机验证仅覆盖开机校时，
按钮端到端验收须在停止 demo 后另行记录。

### Guarded BSP 的时间同步按钮 setup

开机校时和按钮热校时是两个独立验收项。按[按钮适配安装与验收](guarded_clock_calibration.md)
安装匹配的 coordinator core 和固定 BSP adapter。它保留原厂 guard 的完整副本，普通启动
仍执行该副本；只在 STOPPED 校时、HDU UTC 已合格、MDU 消费者仍停止且维护锁有效时，
使用 60 秒有效、绑定本次 boot/operation/coordinator 的一次性凭据恢复原厂 PTP。
恢复有确认回执之后才恢复 MDU。未完成真实按钮验收时不得报告“所有 Foxglove 功能均可用”。

## 4. 先让实际安装与 START 使用同一组组件

2026-10-31 的新机排障先后发现：CSV 校验仍调用不存在的旧 MDU 槽位、HDU helper
强制要求旧 overlay 目录、MDU 混装旧 ELink scheduler、logs 权限缺失及 canonical
夹爪资产不匹配。NTP 通过不代表这些安装条件已满足。按下面顺序读回和预检，再 START。

### 4.1 MDU 所选 bundle 与 CSV validator

在 MDU 上只读核对（以实际安装 catalog 为准）：

```bash
cat /agibot/hope-policy-models/selected.json
python3 /usr/local/lib/hope-foxglove/hope_policy_models.py \
  --root /agibot/hope-policy-models --field deployment
```

HDU 的 `/var/lib/hope-lifecycle/policy-models/selected.json` 必须选中同一模型，
两端 `verify_policy_model` 的 policy/deploy SHA、receive-only、ball model 和匹配 CSV
一致。`foxglove/helpers/hope-lifecycle` 的 `field-assets-mdu` 必须先调用
`resolve_mdu_deployment`；它与正常启动使用同一个 catalog resolver 和 library path。
不要创建指向当前模型的旧目录软链接来掩盖 helper 缺陷。

在 HDU 读取 field selection 的 CSV SHA，然后在 MDU 通过真实 helper 验证；以下
变量必须填刚读出的 SHA，不能照抄历史模型值：

```bash
# HDU
cat /var/lib/hope-lifecycle/field/selection.json
# MDU；receive-only 且没有 CSV 时不执行本项
HOPE_SELECTED_CSV_SHA='<本次 selection 的 csv.sha256>'
printf '{"op":"verify_csv","sha256":"%s"}\n' "$HOPE_SELECTED_CSV_SHA" | \
  /usr/local/libexec/hope-lifecycle field-assets-mdu
```

退出码必须为 0，输出 SHA、segments 中 policy/deploy SHA 必须匹配所选模型。
本步骤只调用原生 validator，不启动 HAL 或 Runner。

### 4.2 HDU overlay 必须显式对应实际安装

旧机使用 `~/hope_ws/install_model21800_fix`，本次新机完整包使用
`/home/agi/hope_ws/install`。helper 新增 `HOPE_HDU_OVERLAY`，默认仍保留旧机路径。
先在 HDU 验证实际包，而后在 **agi 的** `~/.config/hope-foxglove/lifecycle.env`
合并一行配置；备份已有文件，不覆盖其他现场设置：

```bash
# HDU，本例路径只适用于确实安装到这里的机器
source /opt/ros/jazzy/setup.bash
source /home/agi/hope_ws/install/local_setup.bash
ros2 pkg prefix hope_msgs
ros2 pkg prefix hope_planner_cpp
ros2 interface show hope_msgs/msg/BallFlightPacket
# 确认后，在 lifecycle.env 中唯一设置：
# HOPE_HDU_OVERLAY=/home/agi/hope_ws/install
```

`preflight-hdu` 与实际 Planner 启动都读取该路径，且继续要求两个包的 prefix 严格
匹配所选 overlay；目录存在不能替代接口、可执行文件和实际加载位置检查。

### 4.3 ELink scheduler 必须匹配 helper 与厂商 BSP

不能把旧 `install_mdu_elink_scheduler_p1_field.sh` 与新版 lifecycle 混装。
新版 helper 需要 `check-hardware`、`check-privileges`、直接可执行的隔离 Python helper、
以及受限 sudoers。默认 scheduler 是 T3；不能直接安装到 P1。

本次 P1 BSP 的专用入口是
`foxglove/a3/install_mdu_elink_scheduler_p1_20261030.sh`。仅支持以下固定二进制对：

| 文件 | SHA-256 |
| --- | --- |
| `/opt/agibot/bin/aimrt_main` | `61a3581eb5a50b7d9338035644e81ed0c70ad98c9cceccf51833249d89ad6cfc` |
| `/opt/agibot/bin/libhal_elink_pkg.so` | `cff7ec66b551b45864154b13297cedc301e605cc7f7a19d586c6c852e5431c1c` |

只读审计依据：此库 `ELinkStackInit` 在 `0x33b544` 创建入口 `0x33b470`，
循环调用 `ELinkMainLoop`；`ELinkUDPClientInit` 在 `0x33bd48`、`0x33bd60`
依次创建入口 `0x33b8e0`（MessageQueueRecv/UDPWrite）和 `0x33b750`（UDPRead）。
`ELinkThreadCreate` 经默认 pthread 属性创建线程，wrapper 不改名。现场 factory
进程有且仅有三个非主线程名为 `hal_elink`，创建时间顺序一致。厂商 manager 启动时
继承 CPU 2–3；受管 shell 的 CPU 是 0、1、7，因此受管 scheduler 仍严格验证该
起始状态，再对三个线程应用 main/send/receive = RR 23/20/24、CPU 1，并保留回滚。
其他 BSP 必须重新审计，不能把刚读到的任意 hash 写进白名单。

复制以下三个文件到 MDU，保留相对目录：

```text
foxglove/a3/install_mdu_elink_scheduler.sh
foxglove/a3/install_mdu_elink_scheduler_p1_20261030.sh
foxglove/helpers/hope_elink_scheduler_p1_20261030.py
```

在 MDU 运行（`cd` 到刚复制的 `foxglove` 目录）：

```bash
python3 helpers/hope_elink_scheduler_p1_20261030.py check-hardware \
  --lifecycle /usr/local/libexec/hope-lifecycle
sudo bash a3/install_mdu_elink_scheduler_p1_20261030.sh
sudo -n /usr/local/libexec/hope_elink_scheduler.py check-privileges
```

要求 `MDU_ELINK_SCHEDULER_INSTALLED` 与 `ELINK_SCHEDULER_PRIVILEGES_OK`。
安装器先检查硬件，再备份、安装 helper/规则并以 agi 验证权限；不重启 HAL。
如果报告 `Expected root-owned install path without group/other write access`，先
`stat` 检查报告路径。本次 BSP 的 `/usr`、`/usr/local`、`/etc` 是 root:root 0775，
已收紧为 0755；不要取消检查或对整棵系统目录递归改权限。

### 4.4 Runner runtime 目录必须可写

用 root 解包后，二进制/策略保持只读，但 Runner wrapper 的 `mkdir -p logs`
需要可写 runtime 目录。本次在 HAL 已就绪后遇到 `mkdir: cannot create directory
'logs': Permission denied`；这不是模型推理或 NTP 故障。

MDU 上，先用 catalog resolver 取得 `HOPE_SELECTED_DEPLOY`，仅建立 logs 目录：

```bash
HOPE_SELECTED_DEPLOY="$(python3 /usr/local/lib/hope-foxglove/hope_policy_models.py \
  --root /agibot/hope-policy-models --field deployment)"
sudo install -d -o agi -g agi -m 0750 "$HOPE_SELECTED_DEPLOY/logs"
test -w "$HOPE_SELECTED_DEPLOY/logs"
```

其他已安装模型也需要对应目录，不能只修当前模型后声称所有切换都可用。
新 helper 在停止 HAL 前检查此条件，失败返回 `RUNNER_LOG_DIRECTORY_NOT_WRITABLE`。
不要将整套 policy/binary `chmod 777` 或用 root 跑 Runner。

### 4.5 夹爪资产必须满足 canonical 与 native wire 双重校验

包里的 `tools/grip.py` 不能替换成只有 PRESETS 的简化文件。bridge 绑定原件 SHA
`bf837895ec010ee4b512f82cc286b3bab520b00dea051837b5016b5b15c32e19`，
随后解析预设、topic 和 `gripper_profiles.json`。本次错误为
`canonical_grip_sha256_mismatch`；恢复仓库
`agi/agibot_a3_Ultra_aimdk-dev3.1/grip.py` 的匹配原件后，保留当前新夹爪 profile，
更新部署 catalog 中**这个文件**的真实 SHA，policy/deploy/CSV 不变。
发布包应在生成 catalog 前修正资产；已安装机器先备份资产和 catalog，再成对替换。
不能只改 bridge 的预期 hash 来接受简化文件。

MDU 上 source ROS 与所选 bundle 的 library path 后，分别只读输出：

```bash
source /opt/ros/jazzy/setup.bash
export LD_LIBRARY_PATH="$HOPE_SELECTED_DEPLOY:$HOPE_SELECTED_DEPLOY/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
python3 "$HOPE_SELECTED_DEPLOY/tools/a3p_gripper_bridge.py" \
  --grip-source "$HOPE_SELECTED_DEPLOY/tools/grip.py" \
  --gripper-profile new_20261015 --print-release-wire
"$HOPE_SELECTED_DEPLOY/a3_deploy_onnx_ref_pingpong" \
  --inspect-serve-release-wire --gripper-profile new_20261015 \
  --gripper-profiles "$HOPE_SELECTED_DEPLOY/tools/gripper_profiles.json"
```

两行编码必须完全相同；profile 使用本机确认值。本步骤不建 ROS 节点、不发送命令。
新 `preflight-mdu` 已包含同样检查，失败返回 `GRIPPER_WIRE_CONTRACT_INVALID`，
不再等到 HAL 已切换才暴露。编码一致不等于实体夹爪动作或抓球已验收。

还必须核对 ROS imports 与实际 ELink 配置，不能只验证两个编码：

```bash
# MDU，只读；沿用上段 source ROS 和 HOPE_SELECTED_DEPLOY
export PYTHONPATH="/opt/agibot/share/ros2_package/aimrt_protocol_ros2_package/lib/python3.11/site-packages:/opt/ros/jazzy/lib/python3.11/site-packages${PYTHONPATH:+:$PYTHONPATH}"
export LD_LIBRARY_PATH="/opt/agibot/share/ros2_package/aimrt_protocol_ros2_package/lib:/opt/ros/jazzy/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
python3 "$HOPE_SELECTED_DEPLOY/tools/a3p_gripper_bridge.py" \
  --grip-source "$HOPE_SELECTED_DEPLOY/tools/grip.py" \
  --gripper-profile new_20261015 --check-contract
```

要求 `contract/import check PASS; no ROS node or command`。上面路径适用于本次
Debian/Python 3.11 BSP；其他 BSP 应核对实际包路径，不能混装 Python ABI。

本次 P1 使用通用 `hal_elink_a3_cfg.yaml`，受管 helper 从原厂配置生成 session 文件，
只将 hand command backend 扩为 `[iceoryx, ros2]`，并启用 `topic.upper.hand.enable`。
核对 `mdu/system/gripper_config_receipt.txt` 的原厂/会话 SHA，以及**实际进程参数**指向
该 session 配置。旧 `install_mdu_elink_gripper_backend.sh` 修改的是 Ultra 配置，不能
拿它给这台通用 P1 “补装”；更不能改完未加载的文件就声称运行配置已更新。

启动日志还必须有 `gripper_profile=new_20261015 wire_match=1` 和
`serve025 bridge ready; startup sent no gripper command`。本次新夹爪 profile 的 release
位置为 1500、force=1、velocity=100；这些是已选软件参数，不是本次物理夹爪验收结果。
原件契约见[夹爪绑定](foxglove_runner_control_contract.md#motion-and-gripper-binding)，
其中旧模型/CSV 的历史常量不替代当前所选模型的 catalog。

### 4.6 完整预检、START 与完成判据

三端 helper 的通用参数应由当前 `/var/lib/hope-lifecycle/config.json` 和
`field/selection.json` 取得，含 session、当前三机 IP、动捕 IP、P1/P2、mode、CSV SHA、
gripper profile。安装者先调用 `preflight-laptop`、`preflight-hdu`、`preflight-mdu`，
三端分别返回 `LAPTOP_READY / HDU_READY / MDU_READY`；不要把缺失依赖留到用户点击
START 才发现。保留完整 stderr，`INVALID_ARGUMENTS` 或最终一个 reason 不能替代根因。

随后通过 Console START（或同一 `/hope/lifecycle/start` Trigger 服务）启动。
`success=true / start accepted` 仅表示请求受理。必须持续观察 lifecycle state/step、
Runner alive/mode/session_matches/summary，并核对本次实际 PID、可执行文件、model、
policy/config SHA。完成判据是 lifecycle 结束忙状态、运行态成立且 Runner 新鲜 PASSIVE、
session 一致；不要发送 Stand、Policy、Prepare 或 Serve 来证明安装成功。

失败时查 HDU `journalctl -u hope-lifecycle-supervisor` 的同一 session 错误，以及
该 session 的三端 `/tmp/hope_real`/`real_logs` 日志。修复后按受管 recovery/cleanup
重试；不要手动拉起另一套 HAL/Runner。只有真实走完上述启动链路，才可记录
“受管 START → PASSIVE 通过”；基线、动作和击球资格继续分项记录。

## 5. Foxglove 剩余提示怎样解释

| 提示 | 需要查的证据 | 不能直接推出的结论 |
| --- | --- | --- |
| `NTP · AUDIT` 红/未知 | HDU chrony source、offset、monitor 原始 probe 的新鲜度、DDS/bridge 与已安装版本 | 红灯不一定缺安装；未知也不能当通过；单个绿灯不证明三机和动捕电脑都同步 |
| `TIMESTAMP · AUDIT` 未通过 | 原始 IMU stamp、接收时间、跨板 PTP、native IMU 发布和 topic 新鲜度 | 不能只重装 UI，也不能仅凭 HDU NTP 绿灯排除时钟问题 |
| `TF · AUDIT`、asset/marker 提示 | 当前模式是否使用动捕、实际 frame/坐标、刚体和所选资产、连续输入 | VRPN 的 `LIVE MARKER COUNT N/A` 是协议能力限制，不等于机器人没装好 |
| `E-STOP BACKEND · AUDIT` 部分可用/未知 | latch 状态、Runner/vendor 两条软件停止路径、实际后端和新鲜回执 | 不等于实体急停已验证；不能只看按钮存在 |
| `SWING INPUT · AUDIT` 等待 | Runner 模式、Planner 输入/输出、flight/command 新鲜度 | PASSIVE/STOPPED 时没有挥拍命令是预期；看到球不等于 Planner 已生成可用命令 |
| 校时协调器无数据/按钮禁用 | 服务、matching helper/core、sudoers、bridge allowlist、配置、STOPPED 与维护锁 | 不能通过放开按钮或手动 step 绕过安装/维护前提 |

这些是诊断证据；本清单定义部署验收要求，并不声称所有项目已经实现为自动 START
拦截条件。需要对照本次模式解释每条提示，不能要求把所有灰灯都强行变绿。

## 6. 每次交付必须写清楚

按本次机器保存下列记录，每项使用 `PASS / FAIL / NOT_PROVEN / N/A（原因）`，
附时间、机器/boot/session 身份和实际输出/日志路径：

- 系统基线：网络/SSH、依赖、HDU NTP、Laptop NTP、MDU PTP、服务持久性。
- 应用安装：源码及本地差异、活动槽位、binary/config/policy SHA、matching helper/Console。
- 当前运行：实际 PID/argv、lifecycle、PASSIVE、tick/关节/IMU、Foxglove 数据新鲜度。
- 按模式验收：动捕电脑时间、raw→转换→relay→Planner、资产/坐标、需要的 ELink/夹爪/手柄。
- 未完成项、失败原因和下一步；Gate3、真实动作、发球与击球单独报告。

只要必需基线仍是 FAIL 或 NOT_PROVEN，就报告“应用已上传/启动，但新机部署验收未完成”，
列出缺项，不能笼统称“部署完成”。历史机器人上的成功记录不能填进本次验收表。

## 7. 2026-10-31 新机启动实测记录

本次实例：HDU `192.168.200.202`，MDU `10.42.10.12`，Laptop `192.168.100.250`，
所选 `model12000`、P2、VRPN、新夹爪 profile。地址和模型仅为本次记录，不是新机默认值。

- 成功 session：`model21800_20261030T172144Z`；session 前缀是兼容命名，实际模型由
  catalog、Runner policy 路径和 SHA 确认，不能从前缀推断为 model21800。
- 01:25（上海时间）连续约 30 秒读回：lifecycle `RUNNING`、busy=false、Runner
  `PASSIVE`、session_matches=true，state sequence 从 180 增至 248；Runner 日志
  100 Hz tick 持续增长，halts=0、command_fault_latched=0。
- 同窗口 HDU NTP offset 约 -0.323 至 -0.318 ms，gate=true；MDU PTP 为
  `SLAVE`、gmPresent=true，采样 master_offset=50 ns。
- 当前新机的三个 ELink transport 线程已实际核验 CPU 1、RR 23/20/24；canonical
  夹爪资产、profile wire 与启动 bridge 就绪均已核对，启动没有发送手部命令。
- 该记录证明受管 START 到 PASSIVE 已走通，不证明实体急停、夹爪动作、发球、击球
  或完整动捕时间验收。后续操作者切换 PD_STAND/Serve 的回执另行记录，不能覆盖启动证据。
