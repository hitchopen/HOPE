# A3 Deploy Example

本仓库是 A3 实机部署代码的独立示例仓库，包含 C++ 部署主程序、A3 通信 backend、状态同步、策略运行、打包脚本、runtime assets 和交叉编译依赖。目标读者是需要复用 A3 部署链路、或把自己的运控策略接入 A3 body-drive 接口的团队。

实机运行前请先完成仿真、台架、安全绳、急停和低增益验证。建议先运行 `--dry-run` 或 `run_a3_probe.sh`，确认状态接收和推理延迟稳定后再发布关节命令。

## 目录结构

```text
a3_deploy_example/
  CMakeLists.txt
  README.md
  README_robot_io_backend.md
  cmake/
  docker/
  scripts/
    build_a3_deploy_pkg.sh
    export_rockchip_sysroot.sh
    export_thor_sysroot.sh
    convert_a3_onnx_to_rknn.py
  src/
    a3/a3_deploy_onnx_ref/
      config/
      include/
      src/
      scripts/
      tests/
  assets/
    a3_runtime/
      models/
      motions/
      remote_motions/
      teleop_motions/
      rknn_models/
  mujoco_sim_standalone/
    run.sh
    bin/
    models/
  thirdparty/
    joint_msgs/
    onnxruntime/
    rknn_runtime/
    rockchip_sysroot/
    thor_sysroot/
    unitree_sdk2/
  dist/
```

关键模块：

| 路径 | 作用 |
| --- | --- |
| `src/a3/a3_deploy_onnx_ref/include/robot_io/` | `RobotIOBackend` 契约、A3 31 DOF layout、backend 工厂。 |
| `src/a3/a3_deploy_onnx_ref/src/robot_io/` | A3 AimRT backend、publisher/subscriber 绑定、command 拆分。 |
| `src/a3/a3_deploy_onnx_ref/include/a3_sync/` | 六路输入同步、ring buffer、同步统计。 |
| `src/a3/a3_deploy_onnx_ref/include/a3_deploy/` | policy driver、obs builder、动作扩展、watchdog、safe halt。 |
| `src/a3/a3_deploy_onnx_ref/config/a3_runtime_config.yaml` | 默认 runtime config，已改为本仓库相对路径。 |
| `scripts/build_a3_deploy_pkg.sh` | 生成 `dist/a3_deploy_x86_64`、`dist/a3_deploy_rockchip`、`dist/a3_deploy_thor`。 |
| `assets/a3_runtime/` | 默认 ONNX、RKNN、motion CSV 和 teleop fallback CSV。 |
| `mujoco_sim_standalone/` | 仿照真机 `hal_ethercat` body-drive I/O 接口实现的 MuJoCo 仿真引擎，用于上机前闭环验证。 |

部署主链路不依赖 URDF 或 mesh。`a3_body_drive_debug_convert.py` 的 3D Foxglove 可视化如果需要机器人模型，请由使用方通过 `--asset-dir /path/to/urdf/a3` 显式传入，不随部署包默认携带。`mujoco_sim_standalone/` 内的模型资源只服务于本地 MuJoCo 仿真。

## 仿真与真机接口关系

`mujoco_sim_standalone/` 不是替代部署代码的另一套控制程序，而是仿照真机 `hal_ethercat` 接口实现的本地仿真引擎。它接收和真机一致的 body-drive command topic，并发布和真机一致的 joint state / IMU topic，让部署程序在上机前先验证完整 I/O 闭环。

```text
真机链路:
  deploy program
    -> /body_drive/*_joint_command
    -> hal_ethercat
    -> robot system
    -> /body_drive/*_joint_state + /body_drive/{pelvis_imu,torso_imu}/data
    -> deploy program

仿真链路:
  deploy program
    -> /body_drive/*_joint_command
    -> mujoco_sim_standalone
    -> MuJoCo physics
    -> /body_drive/*_joint_state + /body_drive/{pelvis_imu,torso_imu}/data
    -> deploy program
```

因此本地仿真验证的目标不是证明策略在真实机器人上一定安全，而是先确认部署包、AimRT transport、`RobotIOBackend` 同步、策略推理、手动状态机和 command 发布链路可以在与真机同形的接口上跑通。通过后再进入 Rockchip/MDU 上机验证。

## 运行链路

```text
/body_drive/waist_joint_state
/body_drive/leg_joint_state
/body_drive/arm_joint_state
/body_drive/neck_joint_state
/body_drive/pelvis_imu/data
/body_drive/torso_imu/data
        |
        v
A3AimrtBackend subscribers
        |
        v
per-topic converters -> six ring buffers
        |
        v
A3SyncLoop
        |
        v
robot_io::RobotState
        |
        v
policy loop or custom controller
        |
        v
robot_io::RobotCommand
        |
        v
A3AimrtBackend publishers
        |
        v
/body_drive/waist_joint_command
/body_drive/leg_joint_command
/body_drive/arm_joint_command
/body_drive/neck_joint_command
```

运行脚本用 `A3_TRANSPORT` 选择 transport：

```bash
A3_TRANSPORT=iceoryx ./run_a3_probe.sh
A3_TRANSPORT=ros2 ./run_a3_probe.sh
```

推荐默认：

| 平台 | transport |
| --- | --- |
| Rockchip/MDU | `iceoryx` |
| Thor/ADU | `ros2` |
| x86_64 开发机 | 按仿真或回放环境选择 |

## 平台与网络约定

A3 真机部署涉及开发机、HDU 和 MDU。网络角色建议按下面理解，具体编译、传输和运行命令见后文“验证流程”。

```text
开发机 / 编译机
  |
  | ssh / rsync，通过 HDU Wi-Fi IP 作为 JumpHost
  v
HDU
  |
  | 机器人内部网络
  v
MDU: 10.42.10.12
  |
  | 本机 iceoryx transport + hal_ethercat
  v
body-drive state / command topics
```

约定：

| 角色 | 默认/说明 |
| --- | --- |
| 开发机 | 编译 `dist/a3_deploy_rockchip/`，通过 SSH/rsync 传包。 |
| HDU | 作为跳板机；`<hdu_wifi_ip>` 由现场 Wi-Fi 网络分配。 |
| MDU | 默认机器人内网地址 `10.42.10.12`，Rockchip 包在 MDU 上运行。 |
| transport | Rockchip/MDU 默认 `iceoryx`。 |
| 运行目录 | 建议 `/agibot/a3_deploy`，团队可按现场规范替换。 |

MDU 上的 `hal_ethercat` 负责关节和 IMU 信息收发；部署程序通过 body-drive topic 读取状态并发布 command。Rockchip/MDU 通常使用 `taskset -c 4-7` 绑定 RK3588 性能核，以减少推理延迟抖动。

## RobotIOBackend 基础说明

策略侧建议只依赖 `RobotIOBackend`，不要直接依赖 AimRT、ROS 2、iceoryx 或具体 topic。完整接口、字段语义、同步算法和自定义策略伪代码见 [README_robot_io_backend.md](README_robot_io_backend.md)。

基础生命周期：

```text
CreateBackend("a3")
  -> Init("cfg_file_path=...,sync_mode=min_skew_pair,...")
  -> RegisterStateCallback(...)
  -> Start()
  -> policy thread reads cached RobotState
  -> SendCommand(RobotCommand)
  -> Stop()
```

最重要的边界约束：

- `RegisterStateCallback()` 建议在 `Start()` 前注册。
- callback 运行在 backend/sync 线程，只缓存最新 state，不要在 callback 里推理。
- `RobotCommand::{q_des,dq_des,tau_ff,kp,kd}` 长度都必须等于 `backend.GetLayout().dof()`。
- A3 backend 边界是 31 DOF；默认 reference policy 使用跳过 neck/head 的 29 DOF policy view。
- `publish_enabled=false`、`--dry-run`、`--probe` 都可用于避免发布关节 command。

31 DOF backend layout：

```text
[0..2]   waist: waist_yaw, waist_roll, waist_pitch
[3..4]   neck: head_yaw, head_pitch
[5..11]  left arm
[12..18] right arm
[19..24] left leg
[25..30] right leg
```

如果策略输出 29 DOF，建议复用 `ExtractPolicyView()`、`kA3PolicyToSdkIdx` 和 `ExpandToBackend()` 做映射与 neck/head 补齐。

## 默认同步策略

当前 backend 默认同步策略是 `min_skew_pair`。主程序在 `backend.sync_mode` 省略时使用 `min_skew_pair`，在 `backend.sync_hz` 省略时使用 `policy_driver.policy_hz * 2`，因此当前默认是 policy `50 Hz`、sync `100 Hz`。

`min_skew_pair` 会先分别组成 joint group 和 IMU group，再选择最新、低 skew 的一对 group 输出 `RobotState`；`sync_aligned` 主要由 `max_group_pair_skew_ms` 判定。启动时若六路 topic ready，backend 会自动校准 sync phase，让同步线程在输入到齐后短暂等待再释放 state。更细的算法和参数含义见 [README_robot_io_backend.md](README_robot_io_backend.md#默认同步策略)。

当前 runtime YAML 只显式覆盖了几个同步参数：

```yaml
backend:
  sync_release_margin_ms: 0.25
  max_group_internal_skew_ms: 0.05
  max_group_pair_skew_ms: 1.0
```

常用接入检查：

- 确认策略输出是 29 DOF 还是 31 DOF。
- 确认关节顺序，并写 mapping 单元测试。
- 实机前先设置 `publish_enabled=false` 或运行 `--probe`。
- 加入状态超时、连续 unaligned、推理超时和异常输出 watchdog。
- 首次实机按 `PASSIVE -> PD_STAND -> POLICY` 推进。

## 常用参数

主程序参数：

| 参数 | 默认/建议 | 说明 |
| --- | --- | --- |
| `--runtime-cfg PATH` | 必填 | 主运行 YAML。 |
| `--aimrt-cfg PATH` | config 中的 `backend.aimrt_cfg_path` | 覆盖 AimRT YAML。 |
| `--dry-run` | `false` | 只启动 backend，不加载策略、不发布命令。 |
| `--probe` | `false` | 接收/同步 + 推理延迟测试，不发布命令。 |
| `--probe-source a3|smpl|both` | `both` | probe 时选择推理源。 |
| `--auto-start` | `false` | 仿真调试可用；实机建议 manual。 |
| `--frame-log-interval N` | config 值 | 每 N 个 driver frame 打印进度。 |

环境变量：

| 变量 | 默认/建议 | 说明 |
| --- | --- | --- |
| `A3_TRANSPORT` | Rockchip: `iceoryx`，Thor: `ros2` | `run_a3.sh` 选择 transport YAML。 |
| `A3_SOURCE_ROBOT_ENV` | arm 包默认 `1` | 是否 source `/agibot/software/v0/entry/env/env.sh`。 |
| `A3_ROBOT_ENV` | `/agibot/software/v0/entry/env/env.sh` | 机器人环境脚本路径。 |
| `A3_LATENCY_LOG` | `compact` | 可设为 `verbose`。 |
| `A3_PROBE_SOURCE` | `both` | `run_a3_probe.sh` 默认 probe source。 |
| `A3_FRAME_LOG_INTERVAL` | `50` | `run_a3_probe.sh` 默认日志间隔。 |

关键 YAML 默认值：

| key | 默认值 | 说明 |
| --- | --- | --- |
| `onnx.mode` | `monolithic` | 单图策略。 |
| `onnx.model_path` | `assets/a3_runtime/models/model_step_098000_a3.onnx` | 默认 x86 ORT CPU policy。 |
| `onnx.backend` | `ort_cpu` | 可选 `ort_cpu`、`rknn`、`trt`。 |
| `onnx.rknn_core_mask` | `auto` | RKNN NPU core 选择。 |
| `onnx.intra_op_num_threads` | `1` | ORT intra-op threads。 |
| `onnx.inter_op_num_threads` | `1` | ORT inter-op threads。 |
| `reference_motion.motion_dir` | `assets/a3_runtime/motions` | 默认 CSV motion 目录。 |
| `reference_motion.source_fps` | `30.0` | 输入 motion 帧率。 |
| `reference_motion.csv_frame_stride` | `4` | CSV 采样步长。 |
| `reference_motion.target_fps` | `50.0` | 策略时间线。 |
| `reference_motion.future_frame_skip` | `5` | tokenizer future window 步长。 |
| `reference_motion.on_end` | `hold_last` | clip 结束后保持最后帧。 |
| `teleop.delay_ms` | `940.0` | 普通 teleop 延迟。 |
| `teleop.fast_delay_ms` | `220.0` | A3-fast teleop 延迟。 |
| `teleop.stale_warn_ms` | `250.0` | teleop stale warning。 |
| `teleop.max_frames` | `512` | teleop buffer 长度。 |
| `policy_driver.policy_hz` | `50.0` | 策略频率。 |
| `policy_driver.warmup_ticks` | `150` | auto-start warmup。 |
| `policy_driver.pd_stand_ticks` | `150` | manual PD_STAND ramp。 |
| `policy_driver.watchdog.max_frame_age_ms` | `50.0` | 状态最大年龄。 |
| `policy_driver.watchdog.max_unaligned_frames` | `10` | 连续 unaligned 容忍帧。 |
| `backend.sync_mode` | `min_skew_pair` | 默认同步模式。 |
| `backend.sync_hz` | `policy_hz * 2`，当前 `100.0` | backend 状态输出频率。 |
| `backend.auto_phase` | `true` | `min_skew_pair` 启动时自动校准 sync phase。 |
| `backend.sync_ready_after_input_ms` | `0.2` | `min_skew_pair` tick 释放前等待输入到齐的时间。 |
| `backend.sync_release_margin_ms` | `0.25` | 当前 YAML 覆盖值；auto phase 目标等待时间。 |
| `backend.max_group_internal_skew_ms` | `0.05` | 当前 YAML 覆盖值；joint/IMU 单组内部 skew 阈值。 |
| `backend.max_group_pair_skew_ms` | `1.0` | 当前 YAML 覆盖值；joint group 与 IMU group skew 阈值。 |
| `backend.group_pair_search_depth` | `8` | group pair 候选搜索深度。 |
| `backend.max_sample_age_ms` | `50.0` | 样本最大年龄。 |
| `backend.max_skew_ms` | `3.0` | `header_interp` 路径的六路 header skew 阈值。 |

## 编译依赖

通用依赖：

- Linux x86_64 或 aarch64。
- CMake 3.14+。
- C++20 编译器。
- Eigen3、yaml-cpp、msgpack、ZeroMQ/cppzmq、protobuf、zlib。
- Python 3，建议安装 `python3-yaml`、`python3-numpy`、`python3-empy`、`python3-lark`、`python3-protobuf`、`python3-catkin-pkg`。
- ROS 2 Humble 或 Jazzy。
- AimRT 1.6.x，由 CMake FetchContent 获取。
- ONNX Runtime、RKNN Runtime 或 TensorRT/CUDA，按目标 backend 选择。
- Docker，用于 x86_64 主机交叉编译 arm 包。

Ubuntu/Debian 示例：

```bash
sudo apt-get update
sudo apt-get install -y \
  build-essential cmake git pkg-config patch \
  libeigen3-dev libyaml-cpp-dev libmsgpack-dev \
  libzmq3-dev cppzmq-dev protobuf-compiler libprotobuf-dev zlib1g-dev \
  python3-yaml python3-numpy python3-empy python3-lark \
  python3-protobuf python3-catkin-pkg
```

MuJoCo standalone 运行依赖：

`mujoco_sim_standalone/` 是 x86_64 Linux 本地仿真包，包内已带 MuJoCo、AimRT 插件、仿真模型和自定义消息库，但不打包系统 ROS 2 和基础系统库。推荐在 Ubuntu 22.04 + ROS 2 Humble 环境运行：

```bash
sudo apt-get update
sudo apt-get install -y \
  libstdc++6 libgcc-s1 libc6 libx11-6 libxcb1 libxau6 libxdmcp6 \
  libbsd0 libmd0 libfmt8 libspdlog1 \
  ros-humble-ros-base ros-humble-rclcpp \
  ros-humble-sensor-msgs ros-humble-tf2-msgs ros-humble-statistics-msgs
```

如果机器已经安装 `ros-humble-desktop` 或完整 ROS 2 Humble，上面的 ROS 依赖通常已经满足。`mujoco_sim_standalone/env.sh` 会自动 source `/opt/ros/humble/setup.bash`，并设置包内 `LD_LIBRARY_PATH` 与 message overlay。图形显示需要可用的本地桌面、X11 转发，或等价的虚拟显示环境。

编译前 source ROS 2：

```bash
source /opt/ros/jazzy/setup.bash
# or
source /opt/ros/humble/setup.bash
```

## 编译与打包

进入仓库根目录。后续本地命令除特别说明外，都默认从仓库根目录执行：

```bash
cd a3_deploy_example
```

x86_64 包：

```bash
source /opt/ros/jazzy/setup.bash
bash scripts/build_a3_deploy_pkg.sh --arch x86_64 --jobs 20
```

Rockchip 包：

```bash
bash scripts/build_a3_deploy_pkg.sh --arch rockchip --jobs 20
```

Thor 包：

```bash
bash scripts/build_a3_deploy_pkg.sh --arch thor --jobs 20
```

输出目录：

```text
dist/a3_deploy_x86_64/
dist/a3_deploy_rockchip/
dist/a3_deploy_thor/
```

常用打包参数：

| 参数 | 说明 |
| --- | --- |
| `--arch x86_64|rockchip|thor` | 目标平台。 |
| `--runtime-cfg PATH` | 指定 runtime YAML，打包时会复制模型、motion 和 AimRT config。 |
| `--smpl-zmq-host HOST` | 覆盖 packaged `smpl_zmq.host`。 |
| `--jobs N` | 并行编译任务数，默认建议 `20`。 |

## 交叉编译

Rockchip 和 Thor 交叉编译使用 x86_64 Docker builder + aarch64 sysroot：

```text
x86_64 host
  -> Docker builder image
  -> aarch64-linux-gnu toolchain
  -> thirdparty/*_sysroot/*.tar.gz
  -> CMake Release build
  -> dist/a3_deploy_<arch>/
```

sysroot tarball 已放在：

```text
thirdparty/rockchip_sysroot/rockchip-1.0-aarch64-sysroot.tar.gz
thirdparty/thor_sysroot/thor-1.0-aarch64-sysroot.tar.gz
```

如目标镜像更新，可重新导出：

```bash
bash scripts/export_rockchip_sysroot.sh
bash scripts/export_thor_sysroot.sh
```

sysroot 应包含：

```text
opt/ros/jazzy
usr/include
usr/share/eigen3
usr/lib/aarch64-linux-gnu
```

Rockchip 包会启用 RKNN Runtime，并把 packaged config 的 `onnx.backend` 改为 `rknn`。Thor 包会启用 TensorRT/CUDA，并把 packaged config 的 `onnx.backend` 改为 `trt`。

## 验证流程

### 本地 MuJoCo 仿真验证

本地仿真建议先验证通信、同步和推理延迟，再进入手动状态机。仓库内已包含解压后的 MuJoCo standalone 包，路径为 `mujoco_sim_standalone/`；它模拟真机 `hal_ethercat` 一侧的 body-drive 接口，所以 deploy 进程仍然通过同一组 state / command topic 闭环运行。

先编译 x86_64 deploy 包：

```bash
source /opt/ros/jazzy/setup.bash
bash scripts/build_a3_deploy_pkg.sh --arch x86_64 --jobs 20
```

终端 A 启动 MuJoCo sim：

```bash
cd mujoco_sim_standalone
./run.sh a3_t2d0_cfg.yaml
```

可用配置位于 `mujoco_sim_standalone/bin/cfg/`。如果不传配置名，`./run.sh` 会列出菜单并等待选择；请按目标机器人型号选择对应 cfg。

终端 B 先做只收包 dry-run：

```bash
cd dist/a3_deploy_x86_64
A3_SOURCE_ROBOT_ENV=0 A3_TRANSPORT=iceoryx ./run_a3.sh --dry-run
```

确认六路状态 topic ready 后，再跑 probe。probe 会跑推理和延迟统计，但不会发布 command topic：

```bash
cd dist/a3_deploy_x86_64
A3_SOURCE_ROBOT_ENV=0 A3_TRANSPORT=iceoryx ./run_a3_probe.sh
```

probe 稳定后启动正式手动流程：

```bash
cd dist/a3_deploy_x86_64
A3_SOURCE_ROBOT_ENV=0 A3_TRANSPORT=iceoryx ./run_a3.sh
```

仿真中进入动作前需要先让机器人站稳：在 deploy 终端按 `s` 进入 `PD_STAND`，等待约 3 秒后，在 MuJoCo 界面点击 `load-key`，确认机器人稳定站立，再回到 deploy 终端按 `m` 进入 `MOTION`。

仓库提供的 MuJoCo sim 默认使用 `iceoryx` transport。仿真阶段推荐先确认 `sync_complete`、`sync_aligned` 稳定，`infer_ms` 低于控制周期，再开始动作播放。

### Rockchip/MDU 上机验证

先在开发机编译 Rockchip 包：

```bash
bash scripts/build_a3_deploy_pkg.sh --arch rockchip --jobs 20
```

通过 HDU 跳板把包同步到 MDU。`<hdu_wifi_ip>` 替换成现场 HDU Wi-Fi 地址：

```bash
ssh -J agi@<hdu_wifi_ip> agi@10.42.10.12 \
  'mkdir -p /agibot/a3_deploy'

rsync -azP \
  -e "ssh -J agi@<hdu_wifi_ip>" \
  dist/a3_deploy_rockchip/ \
  agi@10.42.10.12:/agibot/a3_deploy/
```

源路径 `dist/a3_deploy_rockchip/` 末尾的 `/` 表示把包内文件同步到 `/agibot/a3_deploy/`。如果去掉末尾 `/`，目标目录下会多一层 `a3_deploy_rockchip`。

MDU 终端 A 关闭系统服务并手动启动 EtherCAT：

```bash
ssh -J agi@<hdu_wifi_ip> agi@10.42.10.12

sudo systemctl stop agibot_pm
source /agibot/software/v0/entry/env/env.sh
cd /agibot/software/v0
bash scripts/hal_ethercat/start_hal_ethercat.sh
```

等待 `hal_ethercat` 启动完成后，MDU 终端 B 进入部署目录。可选检查二进制架构；如果现场需要用 ROS 2 CLI 查看 topic，再 source message overlay：

```bash
ssh -J agi@<hdu_wifi_ip> agi@10.42.10.12

cd /agibot/a3_deploy
file ./a3_deploy_onnx_ref
source ./setup_ros2_msgs.bash
ros2 topic hz /body_drive/arm_joint_state
```

再按下面顺序验证。`taskset -c 4-7` 将策略进程绑定到 RK3588 性能核，提高推理延迟稳定性：

```bash
cd /agibot/a3_deploy

# 只启动 backend/sync，不加载策略、不发布 command。
taskset -c 4-7 ./run_a3.sh --dry-run

# 接收/sync 正常后跑 inference latency probe，不发布 command。
A3_LATENCY_LOG=verbose taskset -c 4-7 ./run_a3_probe.sh

# probe 稳定后再正式启动。
taskset -c 4-7 ./run_a3.sh
```

上机时不要使用 `--auto-start`。启动正式程序后，按手动状态机从 `PASSIVE` 逐步推进。

### 手动状态机

常用按键：

| 按键 | 行为 |
| --- | --- |
| `p` | PASSIVE。 |
| `s` | PD_STAND。 |
| `m` | MOTION。 |
| `t` | TELEOP。 |
| `1` | A3 tokenizer policy source。 |
| `2` | SMPL source，需要 SMPL model/ZMQ。 |
| `3` | A3-fast source，需要 A3-fast model。 |
| 方向键 | remote motion clips。 |
| `q` | 退出。 |

仿真推荐顺序：`s` 进入 `PD_STAND`，等待约 3 秒，在 MuJoCo 界面点击 `load-key` 让机器人站稳，再按 `m` 进入 `MOTION`，最后按 `r` 或空格播放当前动作。上机推荐顺序相同，但不需要点击 MuJoCo `load-key`；在 `PD_STAND` 稳定前不要进入 `MOTION` 或 `TELEOP`，首次验证只做短时间、小幅度动作。

## 安全与调试

安全原则：

- 先用 `--dry-run` 或 `run_a3_probe.sh` 做无命令输出验证，再进入手动状态机。
- 上机时不要使用 `--auto-start`，从 `PASSIVE` 手动推进到 `PD_STAND`。
- `PD_STAND` 稳定前不要进入 `MOTION` 或 `TELEOP`。
- 首次策略验证只做短时间、小幅度动作，保留急停和安全绳。

重点观察：

- waist、leg、arm、neck、pelvis IMU、torso IMU 是否 ready。
- `sync_complete`、`sync_aligned` 是否稳定。
- header skew、group skew、sample age 是否低于阈值。
- state-to-action latency 是否低于控制周期。
- watchdog 是否误触发或设置过松。
- command vector 长度、关节顺序和 PD gains 是否符合目标机器人。
