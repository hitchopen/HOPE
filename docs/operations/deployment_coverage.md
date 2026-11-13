# 全机部署覆盖核对：换机器人时不能只验模型包

2026-11-13 修订。入口：[首次装机](new_robot_commissioning.md)。

## 11 月 10 日旧 P1 复盘

当时 model14200best 的 MDU 88 个、HDU 1 个 catalog 文件全部匹配，仍查出：
ELink 调度加固未迁移、两个 bridge 缺少显示线程补丁、Laptop 沿用旧 HDU NTP
地址、HDU 缺少当前 Laptop ACL、bootstrap 未启用且缺启动排序、operations/worker
与恢复链版本落后、分角色录制及检查缓存未完整安装。模型 catalog 的覆盖范围并非全机。

初次装机文档当时已要求检查时钟、CPU、控制面与重启持久性。因此“文档没更新”只能
解释一部分：文档仍把 native operations 写成 PR 阶段，且未逐项列出显示线程、缓存、
分角色录制；执行时也没有完成统一的逐机覆盖核对。不能据此证明某次遗漏完全由某份
文档造成。另一个明确断点是：最终旧 P1 的 scheduler/lifecycle 适配留在部署工作目录，
回执明确记录“没有提交或推送”。随后仅核对 Git 分支相等，不能发现这些目录外修改。

11 月 10 日回执记录补装后的一次 30 秒 PASSIVE、实际 ELink RR 23/20/24 CPU1、
Runner CPU4/5/6，以及重启后授时链检查；Console 安装文件匹配，但窗口重新加载未观察。
这是历史部署证据，不是当前机器人状态，也不是站立/发球/击球资格证明。
回顾依据为该次任务的 `outputs/deployment-audit.{txt,json}`、
`outputs/deployment-alignment.{txt,json}` 和冻结的 `work/alignment/source`；
它们属于 2026-11-10/new-chat-3 本地记录，未当作当前机器配置导入。

旧 P1 最终两个 helper 的精确差异已保存在
[历史重建记录](../../foxglove/deployment_profiles/old_p1_20261110/README.md)。
其中也有已经被后续代码取代的逻辑；保留溯源不等于批准再次安装。

## 每次开始前建立清单

全机/换机任务以 [coverage.json](../../foxglove/deployment_profiles/coverage.json) 为最低范围：
Laptop 环境、Console、时钟、网络和录制；HDU 时钟/开机顺序、operations、lifecycle、
Planner、transport、DDS、显示线程、缓存和录制；MDU BSP/调度、PTP、模型包、受管
运行和录制；另外显式决定动捕、附加模型和输入设备是否适用。新增部署组件必须同时
更新此清单、安装/回滚入口和核对方法。清单是最低范围，现场新增组件还需补入，不能
声称它能自动发现任意未登记改动。

`hope_operations_deploy.py` 的 manifest 只证明 operations bundle；policy catalog
只证明它登记的资产。两者都不能替代这份覆盖清单。

在源码工作树之外创建本次回执目录：

```bash
mkdir -p /path/to/audit/evidence
python3 foxglove/a3/deployment_coverage.py template > /path/to/audit/receipt.json
python3 foxglove/a3/deployment_coverage.py check /path/to/audit/receipt.json --level installed
```

模板默认所有项 `not_checked`，检查预期失败。准备阶段填写本次 release 的完整 Git
commit、开始时间、各主机 machine ID/当前 boot ID，以及每项 `source_paths`。
`source_paths` 必须在该 commit 中存在，应列明实际源码、构建配方、补丁、安装器和
硬件 profile。厂商二进制用版本化的 hash/profile 配方溯源；不要提交厂商二进制或凭据。
临时目录里的现场修复必须先提取到独立分支并记录可重建配方；未入库就不能填成完成。
仅可选项能设置 `applicable=false`，且必须解释原因。核心项不能静默排除。

## 分层收集和审核证据

每项的三个 stage 含 `expected`、`observed`、`status`、采样时间、machine/boot ID
和 `evidence: [{"path": "evidence/xxx.txt", "sha256": "完整 SHA256"}]`。
附件路径相对回执目录。先确定 expected，再采集 observed；禁止把 observed 直接复制
成 expected 以消除差异。附件应为原始只读输出或实测回执，人工逐项审核其是否支持断言。

- `installed`：真实安装路径、精确文件 SHA、可用的回滚记录。预期来自本次构建或已确认
  的厂商/profile 清单，观测来自目标主机读回。每个 artifact 都要列出，不能只写“匹配”。
- `running`：还要核对实际加载的可执行文件/库、PID 与启动时间、参数、配置、session、
  线程调度和对应功能观测。文件相同或 unit enabled 不证明进程已加载；Console 包已
  安装不证明窗口已加载。当前 STOPPED 时运行项仍未验证，不为填表擅自 START。
- `persistent`：还要有受控重启前后不同的 boot ID、有效 unit/drop-in 和重启后再次采样。
  模板中的 `before_boot_id` 是同一 machine 的重启前标识。重启后必须更新所有阶段的
  观测，使它们都绑定本次 boot；沿用旧 boot 的“通过”将失败。

每个 stage 的必填事实键见模板；可补充预期键，观测必须完全一致。结构化事实可用
字典或列表，例如 `artifact_hashes` 填 `{绝对安装路径: SHA256}`，
`thread_nice` 填两个 bridge 的线程/PID/nice 读回；偏差范围需人工审阅采样后记录
阈值判定和原始附件。时间使用带时区 ISO 8601；审核窗口最长 24 小时，过期需重采。
不适用的某个细项也必须在 expected/observed 中写明理由，由审核人核查，不能留空。

```bash
python3 foxglove/a3/deployment_coverage.py check /path/to/audit/receipt.json --level running
python3 foxglove/a3/deployment_coverage.py check /path/to/audit/receipt.json --level persistent
```

工具离线运行，不 SSH、不安装、不启停服务、不校时、不触碰机器人。它检查缺项、
版本化来源、预期/观测一致性、附件 hash、主机/boot 绑定和时间窗口；它**不能自动解释
附件是否真实支持人工填写的断言**，也不是经过认证的部署安装器或物理验收器。
退出码 1 / `INCOMPLETE` 时报告缺项；退出码 0 / `EVIDENCE_COVERAGE_COMPLETE`
只表示所选层级证据覆盖完整，仍须人工审阅。不得据此输出无范围的“全部部署完成”。

## 交付与回写

每次交付列清：已推送 commit/分支，目标机器和 boot，安装、运行、持久性各层通过项与
未通过项，现场例外、回滚位置。动作资格单独列出。只有安装证据就报告“安装层完成”；
没有重启就明确“持久性未验证”。用户要求 push 时同时核对仓库工作树、部署 staging
目录和现场回执里的例外，不再只搜索当前仓库的未提交文件。

现场操作继续遵守首次装机和生命周期手册的授权/支撑/急停流程；填表不是操作授权。
