# 2026-10-21 分批 PR 交付

针对 2026-10-20 的剩余功能，两个仓库均已推送并创建 PR。本次未执行合并、部署、重启或机器人操作。核对时发现 HOPE #66 已被协作者 learnwithallen 合并；HOPE #67 已将软件提交对齐最新 main。现场分支与正在进行的本地修改保留。

| 批次 | 范围 | nohope | HOPE | PR 基线 |
| --- | --- | --- | --- | --- |
| A | PPQ004 三星球模型参数、完整 C3D/CSV/NPZ 轨迹、拟合结果与说明；纯数据 | [#20](https://github.com/dongc1/nohope/pull/20) | [#66](https://github.com/hitchopen/HOPE/pull/66) | main |
| B | 球选择、运行时加载/核验、Console、离线拟合与复现工具 | [#21](https://github.com/dongc1/nohope/pull/21) | [#67](https://github.com/hitchopen/HOPE/pull/67) | nohope: A；HOPE: main（A 已合并） |
| C | 连续轨迹拟合、完整帧流、405 ms 截止时间与回放 | [#22](https://github.com/dongc1/nohope/pull/22) | [#68](https://github.com/hitchopen/HOPE/pull/68) | main |
| D | Runner HOME 捕获、epoch/坐标校验、schema 4/5、传输 | [#23](https://github.com/dongc1/nohope/pull/23) | [#69](https://github.com/hitchopen/HOPE/pull/69) | C |
| E | Planner 共享 HOME、下一球平面模式、Console 与配套启动配置 | [#24](https://github.com/dongc1/nohope/pull/24) | [#70](https://github.com/hitchopen/HOPE/pull/70) | D |

A 的最终差异只含数据；拟合脚本亦归 B。nohope #20 已整理为单笔纯数据提交；HOPE #66 在历史整理前已被协作者 squash 合并，已核验 main 的合并树与纯数据分支完全相同。A 共 41 个数据/参数/说明文件，9 个无损压缩包按原始及压缩 SHA 校验。保留三份完整原始录制与所有轨迹行。A 不改变默认运行时球模型。

审阅/合并顺序为 A → B，以及 C → D → E，两条链可分别进行。
上游采用 squash merge 时，先将下游分支 rebase 到更新的 main，再 retarget PR；不要仅修改目标分支而带入已审过的提交。B 与 E 的预合并树无冲突，组合 Console 构建及 21 项测试通过。

D 默认需显式开启；E 将受管 normal-play Runner 与 model21800 Planner 成对切换到 shared HOME，平面默认 fixed。实际安装须同时更新 Runner、AimRT 配置/relay、Planner、helper 与 YAML。Kernel 仍使用独立参数列表。参数和源码检查不是物理验证。

验证：球模型 48 项 Python 测试及 73 子测试、4 项隔离 ROS 参数测试；9 包校验及 24 条因果留出重算。405 ms 的 17 个 CTest 目标与 11 项 relay 测试、27 条归档回放通过。Runner 的 16 项协议测试、原生 ONNX 离线探针及 ROS/AimRT 语法检查通过。Planner HOME 的 19 个 CTest 目标经完整运行及修复测试夹具后定向复测通过；辅助脚本 33 项测试及 520 子测试通过。

PPQ004 接触留出只有 3 次，分速收益跨时间不稳定；不采用自动分速参数。不把离线预测、消息回执或 PASSIVE 状态视为实打/Gate3 通过。
