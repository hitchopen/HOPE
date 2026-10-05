# Gate3 两文件入口

在仓库根目录执行：

```bash
./run_gate3.sh --policy-dir /absolute/path/to/policy
```

模型目录只需要：

```text
policy/
  params/deploy.yaml
  exported/policy.onnx
```

也可以直接给出两个文件，名称和所在目录不限：

```bash
./run_gate3.sh --deploy /path/to/deploy.yaml --onnx /path/to/model.onnx
```

入口将两文件原样复制到本次结果目录，Runner 和 Gate3 使用同一个副本。
测试阶段不读取训练 checkpoint、env.pkl 或训练工程；导出阶段仍需要导出器要求的训练资料。
主机没有 ROS Jazzy 时，入口自动进入已安装的 `hope` Distrobox；容器内直接执行。

## 环境配置一次

准备带 `--inspect-policy` 的 x86 Runner 包、仪器化 MuJoCo install 和已经构建的 hope_ws。
Runner 必须包含 ROS 消息与 AimRT backend。检查模式使用真实 ONNX/deploy 加载器，并在通信、gripper、机器人 backend 初始化之前退出。

```bash
./run_gate3.sh --configure \
  --runner-dist /absolute/path/to/runner_package \
  --sim-install /absolute/path/to/mujoco/install \
  --hope-ws /absolute/path/to/hope_ws
```

路径保存到 `a3_deploy/a3_deploy_example/config/gate3/runtime.local.yaml`，由 Git 忽略。
没有本地配置时使用同目录 `runtime.yaml` 的常规安装路径。换模型不用重新配置环境或重新编译。
迁移到另一台机器时，重新配置该机器的安装路径。`--runtime /path/to/runtime.yaml` 可以选择另一套已安装环境。

## 常用操作

```bash
# 原 fixed-HOME 场景：26 球，无球间 reset，默认无 GUI
./run_gate3.sh --policy-dir /path/to/policy

# 同一个场景延长到 100 球，显示 GUI
./run_gate3.sh --policy-dir /path/to/policy --balls 100 --viewer

# 只用真实 Runner 校验两文件并保存解析结果，不启动仿真和通信
./run_gate3.sh --policy-dir /path/to/policy --check

# 使用自定义场景文件并指定新的结果目录
./run_gate3.sh --policy-dir /path/to/policy \
  --scenario /path/to/my-scene.yaml --output /path/to/new-result-directory
```

`--headless` 显式选择默认的无 GUI 模式；`--dry-run` 等价于 `--check`。
输出目录必须是新目录，以免覆盖旧结果。模型和输出目录支持空格。

## 场景与策略分别配置

场景在 `a3_deploy/a3_deploy_example/config/gate3/scenarios/`：

- `fixed-home.yaml`：26 球默认场景，保留原发球序列与评估阈值。
- `fixed-home-100.yaml`：相同序列延长到 100 球。

复制任一文件即可修改 `clean_serves`、`rapid_cycle`、`timing` 和 `metrics`。
发球行是球台表面坐标下的 `[x,y,z,vx,vy,vz]`，不包含人为指定的正反手。
Planner 从实际球轨迹和 session HOME 决定正反手。恢复半径、速度、持续时间、yaw 和脚位阈值由场景指定。
当前 fixed_home 评估族需要六个 clean 问题、总球数为不小于 26 的偶数；rapid 循环自动扩展，每个球均保留完整飞行。

所有球均保留从发出到下降穿过地面高度的完整物理飞行。`flight_s` 只控制发球槽交接时刻，
不会删除旧球：旧球的位置、速度和接触计数转入独立 MuJoCo 球体，继续运动并记录落台。
Planner 仍只接收当前来球；`/sim/gate3/physical_ball_state` 和结果中的
`pp_ball_trajectories.csv` 记录全部球的实测轨迹。球槽满时等待空位，运行退出前等待已发出的球落地。
任何缺失终点或被截断的轨迹都记为不完整，rapid/torture 场景也不再豁免。

这是固定场景的 simulator-truth 测试，默认不要求实机 P1 校准。它不会自动复放训练题库，
也不代表训练分布、实机校准或实机部署认证。定制的场景文件、解析后的序列和阈值会随结果保存；不同场景的分数不能视为同一个 benchmark。

## 模型兼容边界

当前统一场景入口支持以下 schema-2 执行协议：

| 运行 ABI | 观测 | 执行方式 |
| --- | --- | --- |
| `ball_clock_110_v1` | 110 维 | ball-clock、V14 观测与动作协议 |
| `small_station_112_v1` | 112 维 | head-vxy、executed-action 反馈、V12 动作解码 |
| `small_station_324_v1` | 324 维 | current114 + past3×70、成功发送目标反馈、V12 动作解码 |

在这些协议下，recipe 名称、版本、reward、初始化来源和训练 build 名称属于记录信息，
不再选择 Runner、Planner 或报告实现。同一协议的新 checkpoint、新 reward、新训练名称只需重新导出两文件。
ONNX 与 deploy.yaml 中的来源记录仍须配对，文件 SHA、关节顺序、维度、PD、动作缩放、限位、时钟与历史语义仍由真实 Runner 校验。
这里没有把观测维度相同当成语义相同；新增时钟、历史、动作或 Planner 协议需要实现相应 ABI 一次。

公开版原 `pp_gate3_hitter_pingpong.sh` 默认仍是 model_21800 的 12 球 qualification，支持 `--preflight-only`。
新两文件入口显式选择 fixed-home 场景；未支持的 ABI 会被拒绝，不会改写模型 recipe，也不依赖未发布的训练题库。

## 结果与进程

默认结果路径是 `a3_deploy/a3_deploy_example/gate3/runs/<timestamp>/`：

- `run.json`：运行状态、退出码、环境路径、两文件/Runner/场景 SHA、解析后的场景。
- `policy_inspection.json`：原生 Runner 识别出的 ABI 与来源记录。
- `policy/`、`scene.yaml`：本次实际测试的两文件和场景快照。
- `console.log`、`pp_*.log/csv/json`：启动、Runner、Planner、物理球、plant 与报告证据。

`--check` 成功只表示加载与配置检查成功。实际测试退出码仍沿用 Gate3 判定；模型跌倒或恢复指标未过会失败。
新入口只清理本次创建的进程组。旧后端仍共用 `/tmp/pp_*`、ROS/iceoryx 通道，因此同一环境一次运行一场；
已有 Runner/Planner/仿真正在使用通道时会保留原进程并报告占用。Ctrl-C 会结束本次测试并归档已有日志。

收尾先停止新发球，并等待已发球的自然终止记录。`pp_gate3_drain_report.json`
保存未完成球号、模拟器/记录器存活状态及失败原因；`run.json` 和最终认证结果会纳入它。
持续更新的飞行没有总时长上限。若生产进程退出，默认留 2 秒接收最后的缓冲记录；
若某个待结束球连续 10 秒没有遥测进展，则以证据不完整判定失败并清理本次进程。
只刷新报告文件的时间戳不算进展，也不会把缺失的落地记录补成成功。
直接运行旧脚本时可用 `PP_DRAIN_DEAD_GRACE_S` / `PP_DRAIN_STALE_S` 调整这两个间隔。
统一入口在 drain 已失败但引擎仍未退出时再留 10 秒清理，之后终止本次记录的进程组并归档。


## 开发验证

```bash
python3 -m pytest -q \
  a3_deploy/a3_deploy_example/scripts/tests/test_run_gate3.py \
  a3_deploy/a3_deploy_example/scripts/tests/test_gate3_continuous_flights.py
```

实际模型回归测试 `test_gate3_native_policy.py` 通过环境变量 `GATE3_TEST_DIST`、
`GATE3_TEST_POLICY112`、`GATE3_TEST_POLICY324` 指向安装包和两个导出目录，在 hope 容器运行。
它验证原始两文件、只改训练来源的两文件、哈希错配、未知时钟与未知动作协议，全部使用真实原生加载器。

2026-09-14 本机验证：针对性 Python/实际模型检查 35 项通过，C++ runtime 回归 54 项通过。
从 model5460 的两文件副本只修改训练来源信息后，全部 ONNX initializer 保持相同；新入口完成
26/26 engage、26/26 complete、零跌倒，并正确生成 112 维报告。原 Gate3 行为判定为 FAIL，
没有因启动流程改造而改变恢复、脚位、姿态或碰撞判定。
证据在 `a3_deploy/a3_deploy_example/gate3/unified_validation_20260914/full run/`。
带空格路径、独立两文件/324 维检查以及 SIGTERM 中断后归档和进程清理也已验证。

扩大运行现有 `scripts/tests` 时为 66 passed、10 skipped、1 failed：失败项引用仓库 HEAD 中已不存在的
`pp_gate3_rally_v10.sh` 等历史包装脚本；该项未通过，不属于本次新入口测试。
