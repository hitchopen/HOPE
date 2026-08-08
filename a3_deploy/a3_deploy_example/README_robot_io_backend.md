# RobotIOBackend 架构与策略适配指南

这份文档面向需要复用 A3 body-drive 通信层的团队。它只解释机器人 I/O backend 如何把六路状态 topic 整理成统一的 `RobotState`，以及外部运控策略如何生成 `RobotCommand` 并交给 backend 发布；策略模型、reference motion、tokenizer 和上层业务逻辑可以独立替换。

## 一句话概览

`RobotIOBackend` 是策略和机器人通信之间的窄接口：

```text
body-drive state topics
  -> A3AimRT subscribers
  -> per-topic converters
  -> six ring buffers
  -> A3SyncLoop
  -> robot_io::RobotState
  -> custom policy/controller
  -> robot_io::RobotCommand
  -> A3AimRT publishers
  -> body-drive command topics
```

策略侧只需要关心两件事：

1. 从 backend 的 state callback 缓存最新 `robot_io::RobotState`。
2. 按 backend joint layout 填好 `robot_io::RobotCommand`，调用 `SendCommand()`。

AimRT、ROS 2、iceoryx、六路 state topic、topic 内 joint 顺序、同步组帧和 command topic 拆分都封装在 backend 内部。

## 核心文件

| 文件 | 作用 |
| --- | --- |
| `src/a3/a3_deploy_onnx_ref/include/robot_io/robot_io_backend.hpp` | 策略和通信层之间的统一接口，定义 `RobotState`、`RobotCommand`、`RobotIOBackend`。 |
| `src/a3/a3_deploy_onnx_ref/include/robot_io/a3_aimrt_backend.hpp` | A3 AimRT backend 声明和 backend config key。 |
| `src/a3/a3_deploy_onnx_ref/src/robot_io/a3_aimrt_backend.cpp` | AimRT 初始化、topic 注册、state/command pub-sub 绑定。 |
| `src/a3/a3_deploy_onnx_ref/include/a3_sync/a3_sync_types.hpp` | 六路 sample 类型和同步默认参数。 |
| `src/a3/a3_deploy_onnx_ref/include/a3_sync/a3_sync_loop.hpp` | 六路输入同步成一帧 `RobotState`。 |
| `src/a3/a3_deploy_onnx_ref/include/robot_io/a3_layout_extra.hpp` | A3 31-DOF backend layout 与 29-DOF policy view 映射。 |
| `src/a3/a3_deploy_onnx_ref/include/a3_deploy/a3_policy_driver.hpp` | 自带 50 Hz policy driver，可作为固定频率控制循环参考。 |
| `src/a3/a3_deploy_onnx_ref/config/a3_runtime_config.yaml` | runtime YAML，包含 backend、policy driver、模型和 motion 默认配置。 |
| `src/a3/a3_deploy_onnx_ref/config/a3_aimrt_config.ros2.yaml` | ROS 2 transport 参考配置。 |
| `src/a3/a3_deploy_onnx_ref/config/a3_aimrt_config.iceoryx.yaml` | iceoryx transport 参考配置。 |

## 自包含策略资产

仓库内带了 reference deploy 所需的默认 runtime assets：

```text
assets/a3_runtime/models/model_step_098000_a3.onnx
assets/a3_runtime/models/model_step_026000_smpl.onnx
assets/a3_runtime/models/model_step_098000_a3_fast.onnx
assets/a3_runtime/rknn_models/*.rknn
assets/a3_runtime/motions/*.csv
assets/a3_runtime/teleop_motions/*.csv
```

默认 `a3_runtime_config.yaml` 使用仓库相对路径。只复用通信层时，可以忽略这些模型和 motion，写自己的 executable 接 `RobotIOBackend`。

## RobotIOBackend 接口

策略侧建议只依赖这个接口，不直接依赖 AimRT、ROS 2 或具体 topic：

```cpp
class RobotIOBackend {
 public:
  virtual bool Init(const std::string& config) = 0;
  virtual bool Start() = 0;
  virtual void Stop() = 0;

  virtual const JointLayout& GetLayout() const = 0;

  using StateCallback = std::function<void(const RobotState&)>;
  virtual void RegisterStateCallback(StateCallback cb) = 0;

  virtual bool SendCommand(const RobotCommand& cmd) = 0;

  virtual std::string Name() const = 0;
  virtual double StateRateHz() const = 0;
};
```

典型生命周期：

```text
CreateBackend("a3")
  -> Init("cfg_file_path=...,sync_mode=min_skew_pair,...")
  -> RegisterStateCallback(...)
  -> Start()
  -> policy thread reads cached RobotState
  -> SendCommand(RobotCommand)
  -> Stop()
```

使用约束：

- `RegisterStateCallback()` 建议在 `Start()` 前注册。
- callback 运行在 backend/sync 线程，不要在 callback 里做模型推理或耗时控制计算。
- `RobotCommand` 的五个向量长度都必须等于 `backend.GetLayout().dof()`。
- `publish_enabled=false`、`--dry-run`、`--probe` 可用于避免注册/发布 command topic。

## RobotState

`RobotState` 是 backend 输出给策略的统一状态：

```cpp
struct RobotState {
  int64_t timestamp_ns = 0;
  int64_t tick = 0;

  int64_t state_data_ready_ns = 0;
  int64_t state_sync_ready_ns = 0;

  bool sync_complete = true;
  bool sync_aligned = true;
  int64_t sync_skew_ns = 0;

  Eigen::VectorXd q;
  Eigen::VectorXd dq;
  Eigen::VectorXd tau_est;

  Eigen::Vector4d imu_quat_wxyz;
  Eigen::Vector3d imu_gyro;
  Eigen::Vector3d imu_accel;

  bool has_secondary_imu = false;
  Eigen::Vector4d sec_imu_quat_wxyz;
  Eigen::Vector3d sec_imu_gyro;
  Eigen::Vector3d sec_imu_accel;
};
```

关键字段：

| 字段 | 含义 |
| --- | --- |
| `timestamp_ns` | 同步后的逻辑状态时间。当前 A3 backend 使用 system clock 体系，便于和 ROS/AimRT header stamp 对齐。 |
| `tick` | backend 输出帧递增序号，可用于丢帧检测。 |
| `state_data_ready_ns` | 参与本帧组帧的原始样本中，最晚到达本机的时间。 |
| `state_sync_ready_ns` | backend 完成 `RobotState` 组装的时间。 |
| `sync_complete` | 六路输入是否都为本帧贡献了可用样本。 |
| `sync_aligned` | 本帧选中的状态样本是否满足同步 skew 阈值。 |
| `sync_skew_ns` | 当前同步模式下用于判定 aligned 的 skew。 |

策略可以严格要求 `sync_complete && sync_aligned` 后再输出 command，也可以像自带 `A3PolicyDriver` 一样交给 watchdog 做短时容忍。

## RobotCommand

`RobotCommand` 是策略发给 backend 的统一 command：

```cpp
struct RobotCommand {
  Eigen::VectorXd q_des;
  Eigen::VectorXd dq_des;
  Eigen::VectorXd tau_ff;
  Eigen::VectorXd kp;
  Eigen::VectorXd kd;
};
```

所有 vector 长度必须等于 `backend.GetLayout().dof()`。A3 backend 的布局是 31 DOF，因此五个 vector 都应为 31。backend 会把一个 31-DOF command 拆成四路 body-drive command topic：

```text
/body_drive/waist_joint_command
/body_drive/leg_joint_command
/body_drive/arm_joint_command
/body_drive/neck_joint_command
```

## A3 数据布局

A3 backend 暴露 31-DOF layout：

```text
[0..2]   waist: waist_yaw, waist_roll, waist_pitch
[3..4]   neck: head_yaw, head_pitch
[5..11]  left arm
[12..18] right arm
[19..24] left leg
[25..30] right leg
```

默认 reference policy 使用 29-DOF policy view，跳过 neck/head：

```text
[0..2]   waist
[3..9]   left arm
[10..16] right arm
[17..22] left leg
[23..28] right leg
```

如果策略输出 29 DOF，建议复用 `kA3PolicyToSdkIdx`、`ExtractPolicyView()` 和 `ExpandToBackend()`，不要手写散乱下标。`ExpandToBackend()` 默认把 neck/head 补成 `q_des=0`、`kp=40.0`、`kd=2.0`，其余 `dq_des` 和 `tau_ff` 置零。

## A3AimrtBackend 做了什么

`A3AimrtBackend` 是当前 A3 实机通信 backend。它负责：

1. 初始化 AimRT runtime。
2. 根据 AimRT YAML 选择 ROS 2 或 iceoryx transport。
3. 订阅六路状态输入：

```text
/body_drive/waist_joint_state
/body_drive/leg_joint_state
/body_drive/arm_joint_state
/body_drive/neck_joint_state
/body_drive/pelvis_imu/data
/body_drive/torso_imu/data
```

4. 把每路 ROS message 转成内部 sample，写入对应 ring buffer。
5. 通过 `A3SyncLoop` 固定频率输出同步后的 `RobotState`。
6. 在 `SendCommand()` 中把 `RobotCommand` 拆成四路 command topic 发布。

运行包通常用环境变量选择 transport：

```bash
A3_TRANSPORT=iceoryx ./run_a3_probe.sh
A3_TRANSPORT=ros2 ./run_a3_probe.sh
```

约定默认值：

| 平台 | 推荐 transport |
| --- | --- |
| Rockchip/MDU | `iceoryx` |
| Thor/ADU | `ros2` |
| x86_64 仿真/回放 | 按仿真包或回放环境选择 |

## 默认同步策略

当前主程序默认使用 `min_skew_pair` 同步策略，不是简单取每路 latest，也不是默认插值路径。

默认值来源分三层：

| 来源 | 默认/当前值 |
| --- | --- |
| `a3_deploy/main.cpp` | `backend.sync_mode` 省略时为 `min_skew_pair`。 |
| `a3_deploy/main.cpp` | `backend.sync_hz` 省略时为 `policy_driver.policy_hz * 2`，当前默认 `50 * 2 = 100 Hz`。 |
| `a3_sync::SyncConfig` | `auto_phase=true`、fallback `phase_ms=1.5`、`sync_ready_after_input_ms=0.2`、`max_sample_age_ms=50`、`max_backtrack=200`、`group_pair_search_depth=8`。 |
| `A3AimrtBackend` | `sync_release_margin_ms` 内置默认 `0.5`，当前 runtime YAML 覆盖为 `0.25`。 |
| `a3_runtime_config.yaml` | 当前覆盖 `max_group_internal_skew_ms=0.05`、`max_group_pair_skew_ms=1.0`。 |

`min_skew_pair` 的工作方式：

1. 对 waist/leg/arm/neck 组成 joint group：以候选 waist stamp 为锚点，在每个 ring buffer 中找最接近且内部 skew 不超过 `max_group_internal_skew_ms` 的 leg/arm/neck 样本。
2. 对 pelvis IMU/torso IMU 组成 IMU group：同样要求两路 IMU 内部 skew 不超过 `max_group_internal_skew_ms`。
3. 在候选 joint group 和 IMU group 中选一对最新、低 skew 的组合；pair skew 不超过 `max_group_pair_skew_ms` 时，本帧 `sync_aligned=true`。
4. `state_data_ready_ns` 取本帧样本中最晚到达本机的时间，`state_sync_ready_ns` 取完成组帧时间，用于 latency 统计和 policy phase-align。
5. 启动时如果六路 topic ready，backend 会根据实际到包时间自动校准 sync phase，让同步线程在输入到齐后短暂等待再释放 `RobotState`。

当前推荐的 backend config 片段：

```yaml
backend:
  aimrt_cfg_path: src/a3/a3_deploy_onnx_ref/config/a3_aimrt_config.yaml
  dry_run: false

  # sync_mode 省略时默认为 min_skew_pair
  # sync_hz 省略时默认为 policy_driver.policy_hz * 2
  sync_release_margin_ms: 0.25
  max_group_internal_skew_ms: 0.05
  max_group_pair_skew_ms: 1.0
```

可用高级 key：

| key | 默认/当前 | 说明 |
| --- | --- | --- |
| `sync_mode` | `min_skew_pair` | 可选 `min_skew_pair`、`header_interp`、`latest_frame`。 |
| `sync_hz` | `policy_hz * 2` | 主程序默认值；当前为 `100.0`。 |
| `auto_phase` | `true` | `min_skew_pair` 下启动时自动校准 sync phase。 |
| `phase_ms` | `1.5` fallback | 显式设置会关闭 auto phase。 |
| `sync_ready_after_input_ms` | `0.2` | `min_skew_pair` 每个 tick 释放前等待输入到齐的时间。 |
| `sync_release_margin_ms` | 当前 YAML `0.25` | auto phase 目标：最新输入到达后再等待多久释放。 |
| `max_group_internal_skew_ms` | 当前 YAML `0.05` | joint group 内、IMU group 内的最大 stamp 差。 |
| `max_group_pair_skew_ms` | 当前 YAML `1.0` | joint group 与 IMU group 之间的最大 stamp 差。 |
| `group_pair_search_depth` | `8` | 为 pair selection 保留的候选 group 数。 |
| `max_sample_age_ms` | `50.0` | 以本机接收时间判定样本新鲜度，超时视为 stale。 |
| `max_backtrack` | `200` | ring buffer 回看深度。 |
| `align_delay_ms` | `2.0` | `header_interp`/`latest_frame` 路径使用；`min_skew_pair` 使用 `sync_ready_after_input_ms`。 |
| `max_skew_ms` | `3.0` | `header_interp` 路径的六路 header skew 阈值；默认主路径主要看 group skew。 |

## 适配自己的策略

如果策略不是本仓库自带的 ONNX/reference-motion 逻辑，推荐新建一个 executable，只复用 `RobotIOBackend`。

最小结构如下：

```cpp
#include "robot_io/robot_io_backend.hpp"

#include <Eigen/Core>
#include <chrono>
#include <memory>
#include <mutex>
#include <thread>

int main() {
  auto backend = robot_io::CreateBackend("a3");
  if (!backend) return 1;

  const std::string backend_cfg =
      "cfg_file_path=src/a3/a3_deploy_onnx_ref/config/a3_aimrt_config.ros2.yaml,"
      "sync_mode=min_skew_pair,"
      "sync_hz=100,"
      "sync_ready_after_input_ms=0.2,"
      "sync_release_margin_ms=0.25,"
      "max_group_internal_skew_ms=0.05,"
      "max_group_pair_skew_ms=1.0,"
      "max_sample_age_ms=50.0,"
      "publish_enabled=true";

  if (!backend->Init(backend_cfg)) return 2;

  std::mutex state_mtx;
  std::shared_ptr<robot_io::RobotState> latest_state;

  backend->RegisterStateCallback([&](const robot_io::RobotState& state) {
    auto copy = std::make_shared<robot_io::RobotState>(state);
    std::lock_guard<std::mutex> lock(state_mtx);
    latest_state = std::move(copy);
  });

  if (!backend->Start()) return 3;

  const int dof = backend->GetLayout().dof();

  while (true) {
    const auto wake =
        std::chrono::steady_clock::now() + std::chrono::milliseconds(20);

    std::shared_ptr<robot_io::RobotState> state;
    {
      std::lock_guard<std::mutex> lock(state_mtx);
      state = latest_state;
    }

    if (state && state->sync_complete && state->sync_aligned) {
      robot_io::RobotCommand cmd;
      cmd.q_des = Eigen::VectorXd::Zero(dof);
      cmd.dq_des = Eigen::VectorXd::Zero(dof);
      cmd.tau_ff = Eigen::VectorXd::Zero(dof);
      cmd.kp = Eigen::VectorXd::Zero(dof);
      cmd.kd = Eigen::VectorXd::Zero(dof);

      // TODO: call your controller here and fill cmd in backend layout order.
      backend->SendCommand(cmd);
    }

    std::this_thread::sleep_until(wake);
  }
}
```

真实控制中建议额外加入：

- state 超时保护。
- 连续 `sync_aligned=false` 保护。
- policy inference 超时保护。
- command 限幅、增益限幅和 safe-halt。
- 策略频率、backend `StateRateHz()` 和下游控制频率的一致性检查。

## 是否复用 A3PolicyDriver

有两条常见路线。

路线 A：只复用 `RobotIOBackend`，自己写 policy loop。适合已有完整策略框架的团队，你只需要管理策略频率、state cache、inference、action smoothing、watchdog 和 command gains。

路线 B：复用 `A3PolicyDriver`。它已经提供固定频率 RT loop、最新 state 缓存、watchdog、safe-halt，以及 `PolicyFn` 和 `CommandFn` 两种回调接口。

`PolicyFn` 输入是 `RobotState`，输出 29-DOF `q_des`：

```cpp
using PolicyFn = std::function<void(
    std::uint64_t tick_idx,
    const robot_io::RobotState& state,
    std::array<double, 29>& q_des_29_out)>;
```

`CommandFn` 更通用，直接输出完整 `RobotCommand`：

```cpp
using CommandFn = std::function<bool(
    std::uint64_t tick_idx,
    const robot_io::RobotState& state,
    robot_io::RobotCommand& command_out)>;
```

如果策略输出不是本仓库 reference policy 的 29-DOF 格式，优先使用 `CommandFn`。

## Ping-pong runner 额外约束

当前仓库里的 `a3_deploy_onnx_ref_pingpong` 属于上面的“路线 A”变体：
它复用了 `RobotIOBackend` 和 `A3PolicyDriver`，但 180-D 观测、31-DOF
ONNX、`level 0/1`、reference clock 和 localization mode 都在前端自己实现。

对这条路径，现场最容易误解的边界是：

- backend 只负责 joint/IMU 状态同步与 body-drive command 发布，不负责世界位姿估计。
- `perfect_tracking` 只是把 base/torso 世界位置用参考轨迹占位；`oracle` 只给仿真。
- 当前 HOPE mocap/VRPN 话题还没有直接接进 `a3_deploy_onnx_ref_pingpong`，所以“现场有 mocap”
  不等于这条 deploy 路径已经在用 mocap。
- MuJoCo 和真机走的是同一条 `A3AimrtBackend` `/body_drive/*` 接口，所以应先在
  `scripts/run_sim.sh` + `run_a3_pingpong.sh --dry-run/--reference-playback/shadow`
  这条 shared-interface rehearsal 上过关，再去做真机地面支撑排查。只有 source sim
  支持 `scripts/run_oracle.sh` / `scripts/reset_sim.sh`；standalone 只覆盖 A/B 模式。
- 真机 bring-up 不应直接从 ONNX swing 开始，先跑 `--reference-playback` 验证同一条
  scatter/gain/topic 路径，再跑 `--dry-run` / `--probe` / `shadow`。

## 诊断和无输出测试

部署包里有两个常用入口：

```bash
./run_a3.sh --dry-run
./run_a3_probe.sh
```

区别：

| 命令 | 行为 |
| --- | --- |
| `--dry-run` | 只启动 backend/sync，不加载策略，不跑 inference，不发布 command。 |
| `run_a3_probe.sh` | 接收/sync 正常后跑 inference latency probe，但强制不发布 command。 |

probe 日志里比较有用的字段：

| 字段 | 含义 |
| --- | --- |
| `state_transport_apparent_ms(min/avg/max)` | 本机收到 state 的时间减原始 header timestamp；跨机器时受时钟同步影响。 |
| `state_ready_ms(min/avg/max)` | 原始样本到齐后，到同步 state ready 的延迟。 |
| `state_header_skew_ms(min/avg/max)` | `header_interp` 路径的六路 header timestamp 最大差。 |
| `group_pair_skew_ms(min/avg/max)` | `min_skew_pair` 路径中 joint group 与 IMU group 的 stamp 差。 |
| `resample_samples(interp/hold)` | `header_interp` 路径的插值/hold 统计。 |
| `infer_ms(min/avg/max)` | policy inference 延迟。 |

这些诊断应先用于确认通信和同步质量，再打开自己的 command 输出。

## ROS 2 CLI message overlay

如果在目标机上执行 `ros2 topic hz /body_drive/arm_joint_state` 报 message type invalid，通常是当前 shell 没有 source 到 `joint_msgs` 的 Python message package。部署包会带一个 overlay：

```bash
source ./setup_ros2_msgs.bash
ros2 topic hz /body_drive/arm_joint_state
```

这只影响 ROS 2 CLI。C++ backend 运行时主要依赖已打包的 type-support `.so`。

## 移植到不同场景

### 同样是 A3 body-drive topic，只换策略

保持 `A3AimrtBackend` 不动，写自己的 policy executable 或接入 `A3PolicyDriver::CommandFn`。

需要确认：

- 六路 state topic 名称不变。
- command topic 名称不变。
- message 类型仍是 `joint_msgs/msg/JointState`、`joint_msgs/msg/JointCommand`、`sensor_msgs/msg/Imu`。
- command 的 31-DOF 顺序匹配 `MakeA3Layout31()`。

### Topic 名称变了，但 message 和 DOF 没变

当前 topic 名称在 `A3AimrtBackend::RegisterPubSub_()` 里写死。要支持可配置 topic，建议新增 backend config keys，例如：

```text
waist_state_topic=/xxx/waist_joint_state
leg_state_topic=/xxx/leg_joint_state
...
```

然后在 `RegisterPubSub_()` 里用成员变量替换硬编码 topic。AimRT YAML 也要同步允许这些 topic 走对应 transport。

### DOF 或 joint 顺序变了

不要在策略里硬凑下标。推荐新增一套 layout 和 mapping：

- 新增 `MakeYourRobotLayout()`。
- 定义每个 topic 的 start/count 和 topic 内 joint name/order。
- 修改 subscriber converter，把 topic message 重排到统一 layout。
- 修改 publisher，把统一 `RobotCommand` 拆回各 topic。

如果机器人拓扑差异很大，建议实现一个新的 `RobotIOBackend`，不要强行复用 `A3AimrtBackend`。

## 常见坑

1. `RobotCommand` 五个 vector 长度必须等于 `backend.GetLayout().dof()`。
2. A3 backend 边界是 31 DOF；29 DOF 只是一部分策略使用的 policy view。
3. callback 线程里不要直接跑大模型推理，避免堵住 sync/transport。
4. 跨板运行时先确认时钟同步，否则 header-based latency 只能看作 apparent latency。
5. `--probe` 和 `publish_enabled=false` 适合只测收包和 inference，因为不会输出关节 command。
6. 真机输出 command 前一定要有 watchdog 或 safe-halt 路径，不能只依赖策略永远正常返回。
7. 如果使用 ROS 2 transport，确认 `ROS_DOMAIN_ID`、发现范围、静态 peers 和目标机一致。

## 推荐对接步骤

1. 先跑 `./run_a3.sh --dry-run`，确认六路 topic ready。
2. 再跑 `./run_a3_probe.sh`，确认同步和 latency 指标正常，且 command publisher disabled。
3. 写一个最小策略 executable，只打印 state tick 和 layout，不发 command。
4. 发零增益或安全站立 command，确认 topic 和下游接收链路。
5. 接入自己的策略输出，先低 kp/kd、小幅动作、短时间运行。
6. 最后再调策略频率、线程绑核、ORT/TensorRT/RKNN 或其他 inference backend。

按这个边界接入时，策略作者可以把 `RobotIOBackend` 当成稳定的机器人 I/O 适配层：它负责把真实通信世界整理成统一 state，也负责把统一 command 拆回底层 topic。
