# macOS 非训练开发与部署环境

适用于 Apple Silicon、Docker Desktop 和本分支 `agi/a3_deploy_example` 目录。
Mac 本机运行 Foxglove 和离线 Python；ROS、原生 Runner、Gate3 以及机器人交叉编译在 Linux 容器内执行。
本地构建入口不连接机器人，也不启动 HAL 或实际运动；现场 Foxglove 操作入口见后文。

构建入口保留 `build_6` 的原目录，不移动源码或复制同名文件。若在 `main` 的已有
`a3_deploy/` 布局中使用，同一入口会根据 `CMakeLists.txt` 选择现有源码目录；旧路径
只剩构建产物时不会被误选。两种源码同时存在时优先使用 `build_6` 的 `agi/` 路径。
本文命令和 2026-10-17 的现场记录以 `build_6` 布局为准；`main` 下对应 Runner/simulator
路径前缀为 `a3_deploy/`。记录中的设备地址、二进制哈希和测试结果属于当次部署。

## 已安装工具与本地文件

- Xcode/Clang、Homebrew、Docker Desktop、Node/npm、Foxglove。
- Homebrew Bash、coreutils、findutils、GNU sed/tar、rsync、wget、CMake、Ninja、pkgconf。
- `~/.local/bin/git-lfs`：官方 macOS ARM64 发行包，下载时校验 SHA-256。
- 仓库根目录 `.venv-tools`：uv 管理的 Python 3.12，用于 NumPy/SciPy、绘图、ONNX CPU 推理、C3D、MCAP、独立 MuJoCo。
- `artifacts/macos_setup/`：安装日志、版本清单、依赖与策略来源校验记录。它是本机生成目录，不提交 Git。
- Agi 原始依赖已复制到 `agi/a3_deploy_example/thirdparty/`；现有源码未被厂商示例覆盖。
- Foxglove Console 1.8.12 安装在 `~/.foxglove-studio/extensions/`。

本次还恢复了原 CMake/头文件引用但 checkout 缺失的五个源码文件：
`serve_hal_clock.hpp`、`serve_release_message.hpp`、`serve_hal_observer.cpp`、
`serve_hal_loader_probe.cpp`、`serve_hand_channel.proto`。它们逐字节取自 `origin/main` 的 `0c97fe61c`，
来源和 SHA-256 记录于 `restored-sources.json`；`.gitignore` 已允许它们被版本管理，避免新机器再次缺失。

macOS 27 上 Homebrew 默认安装曾报版本识别错误；本次使用 `brew install --force-bottle` 成功安装上述工具。
不需要安装 Isaac、训练用 PyTorch、NVIDIA 驱动或 `grasping`。

本机 x86 Runner 编译曾在 oneTBB 的 LTO 阶段触发 GCC 13 `return_token` 内部错误。
Mac 入口设置 `A3_TBB_ENABLE_IPO=OFF`，仿真 CMake 也关闭 oneTBB IPO；其他构建入口保留原默认设置。
这是构建选项调整，不修改策略、控制逻辑或模型文件。

原打包脚本还无条件要求三个已删除的 RallyV10 启动脚本。本次将该要求限定为
`model_10500_v10.onnx` 的旧式模型包；带 `policy_dir` 的包使用通用 `run_a3_pingpong.sh`。
旧式 V10 包仍执行原有脚本完整性检查，当前 `model_21800` 包不使用 V10 启动入口。

## 已验证的本地结果

- ARM64 和 x86_64 ROS 工作区均完成全部六个启用包的构建。
- ARM64 C++ 测试 119 项通过，Python Planner 测试 134 项通过。
- ARM64 本地合成来球运行 25 秒，观测到 2,870 条 `/poses` 和 39 个 `/ball/flight_packet`；
  fake ball、packetizer、Planner、Foxglove Bridge 均存活。此项验证启动和消息链路，不代表击球性能通过。
- Mac 到容器 `ws://127.0.0.1:8765` 的 WebSocket 握手成功；安装的 Bridge 3.6.0 使用 `foxglove.sdk.v1`。
- 两套 main 导出模型均通过 macOS ONNX 检查及 CPU 推理，输出为有限值。
- 本机 Python MuJoCo 可加载最小模型并完成 CPU 步进。
- Rockchip 完整包通过原脚本自检，40 个 ELF 文件全部为 AArch64；ARM Runner strict loader 可加载两套模型。
  在本地 Ubuntu ARM64 容器中，Runner 动态库均可解析；这不替代目标 MDU 的 ABI/现场验证。
- C++ MuJoCo 3.1.6/AimRT 1.6.0 已编译并安装；Xvfb 下运行 18 秒收到 1,135 条
  `/sim/a3/pelvis_pose`，仿真进程持续存活并完成清理。
- x86_64 完整包构建命令退出码 0，通过原打包自检；39 个 ELF 文件全部为 x86_64。
  两套策略在 x86 strict loader 中均加载成功，ABI 识别结果与 ARM 一致。
- 统一 Gate3 使用 `main-checkpoints` 完成 `--check`，输出 `POLICY_CHECK_PASS`。

26 球 `fixed-home` 闭环场景也已执行结束，耗时约 163.46 秒，记录 0 次摔倒。
**验收未通过：301 项通过、28 项失败，退出码 1，`certification_pass: false`。**
失败项包括站位 Y 误差、击球前底盘速度、一次 `sync_miss`、快速球完成身份/时序检查。
这次运行验证了本地仿真、ROS、Planner、Runner 和报告生成链路；模型与时序性能仍需单独分析。
没有为通过测试修改模型或降低验收阈值。
完整日志、模型快照与报告位于 `artifacts/macos_setup/gate3-main-checkpoints-run2/`，
重点查看 `run.json`、`pp_rally_report.json` 和 `console.log`。

各项日志和机器可读结果见 `artifacts/macos_setup/`。

## 日常入口

所有命令从仓库根目录运行。`python3` 只负责调用 Docker；ROS 使用容器内 `/usr/bin/python3`。

```bash
# 创建/更新 Linux 环境（首次需要下载软件包）
python3 scripts/macos/hope.py image --arch arm64
python3 scripts/macos/hope.py image --arch amd64

# ARM64：日常 Planner、消息、动捕工具构建和测试
python3 scripts/macos/hope.py build-ros --arch arm64 --jobs 2
# 切换不同目录布局后，第一次构建清除缓存中的旧 include/source 路径
python3 scripts/macos/hope.py build-ros --arch arm64 --clean-cache --jobs 2
python3 scripts/macos/hope.py test-ros --arch arm64
python3 scripts/macos/hope.py shell --arch arm64

# x86：完整原生部署程序和 Gate3 使用的 ROS 工作区
python3 scripts/macos/hope.py build-ros --arch amd64 --no-tests --jobs 2
python3 scripts/macos/hope.py build-runner --arch amd64 --build-only --jobs 2
python3 scripts/macos/hope.py build-sim --arch amd64 --jobs 2
```

Docker 内源码路径为 `/work`。ROS 输出分别位于 Docker 卷
`hope-macos-arm64`、`hope-macos-amd64` 中的 `/opt/hope/ros/`。
旧的 `hope_ws/build`、`install`、`log` 软链接没有被删除，以上入口不使用它们。
`build-ros` 在当前 checkout 存在 `NatNet2ROS2/src` 时一并构建该动捕工作区；
`build_6` 保留原有 `hope_ws/src` 来源。`--clean-cache` 仅用于 `build-ros`。
两个架构不可混用构建缓存或安装产物。x86 在 Apple Silicon 上通过模拟执行，编译和运行比原生 ARM 慢。
本机 Docker VM 的 `/dev/shm` tmpfs 不支持 POSIX ACL，Iceoryx RouDi 因此无法启动。
入口将 `/dev/shm` 映射到每次运行独立的匿名 Linux 数据卷，保留 ACL 检查；`--rm` 会清理该卷。
它由 Docker VM 的磁盘与页缓存承载，未共享宿主机 IPC，也不用于证明实时性能。

`shell` 自动加载 ROS 以及已构建工作区。执行单条命令可使用：

```bash
python3 scripts/macos/hope.py exec --arch arm64 -- ros2 pkg list
```

## 选择策略与打包

从 `origin/main` 恢复的文件分别保存在：

- `artifacts/macos_setup/policies/main-checkpoints/`：`continuous_rally_sim2real_v3`，110 维输入。
- `artifacts/macos_setup/policies/main-model21800/`：`rally_v14`，110 维输入。
- `artifacts/macos_setup/teleop_humanlike/`：策略、速度编码器和 HumanLike 配置。

`main-assets.json` 记录来源提交、原路径和 SHA-256。目录名不是兼容性证明；以当前 C++ Runner 的 `--inspect-policy` 结果为准。
不要将两套策略的 `deploy.yaml` 和 `policy.onnx` 混配。

实际 ARM Runner strict loader 已验证两套模型均可加载：`main-model21800` 的 ABI 为 `legacy`，
`main-checkpoints` 的 ABI 为 `ball_clock_110_v1`，动作维度均为 31。
新版统一 Gate3 支持后者；`model21800` 需要对应的历史验证入口，不能直接套用下面的统一 Gate3 命令。
本次示例部署包携带 `main-model21800`；Gate3 通过显式 `--policy-dir` 使用独立的 checkpoints 模型副本。

```bash
python3 scripts/macos/hope.py build-runner --arch amd64 --jobs 2 \
  --policy-dir artifacts/macos_setup/policies/main-model21800 \
  --teleop-policy-dir artifacts/macos_setup/teleop_humanlike

python3 scripts/macos/hope.py inspect-policy --arch amd64 \
  --policy-dir artifacts/macos_setup/policies/main-model21800

python3 scripts/macos/hope.py build-rockchip --jobs 2 \
  --policy-dir artifacts/macos_setup/policies/main-model21800 \
  --teleop-policy-dir artifacts/macos_setup/teleop_humanlike
```

完整包输出到 `agi/a3_deploy_example/dist/a3_deploy_x86_64` 或 `dist/a3_deploy_rockchip`。
原有打包脚本会重建对应输出目录；需要保留旧包时先保存到独立版本目录。
`--build-only` 仅编译，不能作为完整部署包。
`build-rockchip` 使用目标 sysroot 和 amd64→aarch64 交叉编译器；不会启动机器人。

Thor sysroot 已恢复；`build-thor` 使用厂商 DRIVE OS 基础镜像，需能访问 `registry.agibot.com`。
它不属于 MDU/Rockchip 构建的依赖，不能用 Rockchip 镜像代替。
本次两次 HTTPS 连接均超时，且 Agi 目录中只有 sysroot、没有离线 DRIVE OS 镜像，
因此可选的 Thor 构建未执行。**Planner、x86 Runner、MuJoCo/Gate3、Rockchip/MDU 包均不依赖 Thor，
当前已完成的构建链路不受影响。** 以后只有选择 Thor 目标时，才需要厂商仓库网络或对应离线镜像。

## Gate3

构建 x86 ROS、完整 Runner 包以及 simulator install 后，在 amd64 环境中配置一次：

```bash
python3 scripts/macos/hope.py exec --arch amd64 -- bash /work/run_gate3.sh \
  --configure \
  --runner-dist /work/agi/a3_deploy_example/dist/a3_deploy_x86_64 \
  --sim-install /opt/hope/sim/install \
  --hope-ws /opt/hope/ros

python3 scripts/macos/hope.py exec --arch amd64 -- bash /work/run_gate3.sh \
  --policy-dir /work/artifacts/macos_setup/policies/main-checkpoints --check

# 完整本地场景：Xvfb 为厂商 GLFW 提供虚拟显示，不需要 macOS X11
python3 scripts/macos/hope.py exec --arch amd64 --virtual-display -- \
  env ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ROS_DOMAIN_ID=92 \
  bash /work/run_gate3.sh \
  --policy-dir /work/artifacts/macos_setup/policies/main-checkpoints \
  --scenario fixed-home --headless
```

`--check` 只检查模型和准备运行文件。完整场景需另行用 `--scenario fixed-home --headless` 执行，
并检查返回码和报告，不能以环境构建或模型加载替代 Gate3 通过。
厂商 C++ 仿真器即使设置 `MUJOCO_GL=egl` 仍需 GLFW；因此完整场景需加 `--virtual-display`，
使用 Xvfb 和 Mesa 软件 OpenGL。此方式不在 Mac 上显示窗口，也不证明实时性能。
容器没有配置 macOS 图形显示转发；原生 Python MuJoCo viewer 与该 C++ Gate3 是不同环境。

## Mac 离线工具与 Foxglove

```bash
source .venv-tools/bin/activate
python -c 'import numpy, scipy, onnxruntime, mujoco; print(onnxruntime.get_available_providers())'
```

Python MuJoCo 的 passive viewer 在 macOS 上通过 `.venv-tools/bin/mjpython` 启动。
该环境的具体包版本保存在 `artifacts/macos_setup/python-tools.lock.txt`；不替代 C++ ONNX Runtime 1.19.2。

Foxglove 导入 `foxglove/layouts/model21800_console.json`，现场使用 `ws://<HDU-IP>:8766`。
`8765` 为监视入口。安装扩展和布局不启动机器人。

本机查看合成来球和 Planner 话题，可在同一个容器 shell 中运行：

```bash
python3 scripts/macos/hope.py shell --arch arm64 --publish-port 8765
# 以下命令在容器 shell 中运行
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export ROS_DOMAIN_ID=90
ros2 launch hope_bringup hope_bringup.launch.py use_fake_ball:=true &
ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765
```

Mac Foxglove 连接 `ws://127.0.0.1:8765`。此入口提供本地话题查看，未启动现场 lifecycle/command proxy。
端口只绑定 Mac 的 localhost；同一端口不能同时发布给两个容器。退出容器后，容器内进程会结束。

## 现场连接的边界

本地构建不证明 HDU/MDU 系统 ABI、动捕或时钟已就绪。
HDU Planner 仍需在目标匹配的环境中构建/验证；本地 Ubuntu ARM64 产物不能仅凭架构相同直接认定适配厂商系统。

当前 Laptop 现场脚本依赖 Linux systemd、chrony，以及 Xbox 的 `/dev/input`/evdev。
默认构建容器没有接入这些物理设备，也没有按场地配置 NatNet、DDS peer 或时钟服务。
若 Mac 要同时取代现场 Linux 数据采集机，需单独完成网卡、设备透传和服务适配验证；
Mac 作为 SSH/rsync/Foxglove 操作端则不依赖这些本机 ROS 服务。

### Mac 与 Linux 共用 Foxglove

2026-10-17 已在实际 HDU/MDU 上验证 Mac 操作端。当前地址保存在本机
`~/.config/hope-macos/operator.json`，SSH 私钥单独保存在该目录，不进入仓库。
桌面 `HOPE Foxglove.command` 会启动 Mac 后端并打开已保存的 `HOPE A3 · Mac + Linux` 布局。
它不会点击 START、切换 Runner 模式或发送关节/夹爪命令。
入口链接使用 [Foxglove 官方深链接协议](https://docs.foxglove.dev/docs/visualization/shareable-links)。

两台电脑可以同时连接 HDU 的 `:8766` 查看同一个 supervisor。日志/会话 Laptop 在每次
会话中只选一台；切换时必须保持 STOPPED。两个 UI 不代表两套独立的机器人控制权。

```bash
# Mac 后端与反向通道；登录后也会由 LaunchAgent 自动恢复
python3 scripts/macos/operator.py up
python3 scripts/macos/operator.py status

# 同时选择本次会话的 operator_profile 和 Laptop IP；保留其余现场配置
python3 scripts/macos/operator.py select-mac
python3 scripts/macos/operator.py select-linux

# 更新本机 operator.json 的 Mac 地址后，重新绑定 HDU 路由和 DDS peer
python3 scripts/macos/operator.py bind

# 先选回 Linux，再关闭 Mac 后端；up 可以重新启用
python3 scripts/macos/operator.py down
```

Mac 后端镜像为 `hope-operator:jazzy-arm64`，容器为 `hope-macos-operator`。
重建此镜像使用 `docker build --platform linux/arm64 -t hope-operator:jazzy-arm64 -f scripts/macos/Dockerfile.operator scripts/macos`。
该容器复用已经构建的 ARM64 ROS 工作区，保留 Linux 的 `/proc`、tmux 和 MCAP 运行环境。
Linux 原有 Distrobox 路径保持默认；容器显式设置 `HOPE_LAPTOP_RUNTIME=direct`。
登录账户在兼容容器内为 `dongc1`，不要求 macOS 创建同名账户或打开系统远程登录。

操作端身份使用固定的 `operator_profile=mac` 或 `linux`，不绑定 Wi-Fi IP 或物理
网卡 MAC 地址。Foxglove 的 **Operator computer** 选择和 `select-mac` / `select-linux`
会将 profile 与当前 Laptop IP 一起确认。换 IP 后 profile 保持不变；IP 仍用于 DDS
和会话元数据，需要更新为当前地址。Mac 的 SSH 目的地固定为 `hope-mac-operator`，
由 `bind` 指向 HDU localhost 上的反向隧道，Linux 的 SSH 目的地则使用当前 Laptop IP。

HDU supervisor 按 profile 选择账户和 helper，所有 Laptop 启动、回收、录制及资源调用
都走同一选择逻辑：

| Profile | 账户配置 | helper 配置及默认值 |
| --- | --- | --- |
| `linux` | `HOPE_LAPTOP_USER`，默认 `operator` | `HOPE_LAPTOP_LIFECYCLE_HELPER`，默认 `/usr/local/libexec/hope-lifecycle` |
| `mac` | `HOPE_MAC_USER`，默认容器账户 `dongc1` | `HOPE_MAC_LIFECYCLE_HELPER`，默认 `/home/dongc1/.local/libexec/hope-lifecycle` |

这些设置在 HDU 的 `/etc/hope-foxglove/network.env` 中独立保存。使用仓库提供的 Mac
容器时无需修改 Mac 默认值，也无需将 Linux 的全局设置改成 Mac 账户。仅修改 SSH
config 的 `User` 不能覆盖 supervisor 显式指定的账户。

升级已有部署时，同时安装新的 `hope_lifecycle_core.py`、`hope_lifecycle_supervisor.py`
并在 STOPPED 状态重启 supervisor，然后构建/安装新版 Console，运行一次 `bind` 和
`select-mac` 或 `select-linux`。旧 schema 1/2 配置保留现有账户行为，迁移为 `linux`
profile；旧客户端只提交原有五个字段时，会保留当前 profile。应通过新版入口明确选择
一次操作端。`bind` 会移除旧的 Mac IP 匹配，只保留稳定别名，避免旧 IP 被 Linux
复用后仍误接 Mac 隧道。

SSH 只在 Mac localhost 发布，并经 SSH 反向转发到 HDU localhost。HDU 的专用 Mac host
配置使用已有 HDU 公钥认证，只允许固定的 Laptop helper 子命令，拒绝任意远程 shell、
PTY 和端口转发。原 Linux host 路由与账户保持可用。HDU 的 network.env 和 SSH config
修改前均保留了备份。换 Wi-Fi 后需要检查 operator.json 的地址，并执行 bind 和 select-mac。
HDU SSH 复用套接字位于 `/var/lib/hope-lifecycle/mac-ssh-%C`。该目录只允许服务用户访问，
也在 supervisor 的可写路径内；不能放在 `~/.ssh`，因为服务使用 `ProtectHome=read-only`。
已在 supervisor 的实际 mount namespace 内执行 Mac preflight，验证冷连接成功。

Mac 登录后由两个用户 LaunchAgent 维护：

- `org.hope.foxglove-assets`：localhost:8000 球桌 URDF 服务。
- `org.hope.macos-operator`：每 30 秒检查本地容器与反向 SSH 通道。`down` 会留下停用标记，
  后台检查不会重新启动它；`up` 清除停用标记。

日志写入本仓库的 `real_logs/`；容器 home/状态分别持久化于
`hope-macos-operator-home`、`hope-macos-operator-state` 卷。
Mac 运行时的标定目录单独使用 `artifacts/macos_setup/operator-calibration/`，首次为空，
不会把 checkout 自带的旧机器人标定当作当前机器人的有效标定。

本次现场证据在 `artifacts/macos_setup/robot-104/`：

- Foxglove WebSocket 公布 110 个话题、21 个服务，10 秒实际收到 414 条选定状态消息。
- Mac Laptop、HDU、MDU 的静态 preflight 均通过；Mac 的 prepare/stop-capture helper 也成功。
- Mac → HDU → MDU 与 HDU → Mac 容器的 SSH 均验证；任意 shell 命令被容器拒绝。
- Mac/Linux 两种 Laptop 配置切换均被 supervisor 确认；本次 Linux 主机离线，未重复验收其运行环境。
- Linux 容器内 25 项 lifecycle/field/record-switch 测试通过。
- 在隔离 ROS domain 90 中，真实 MCAP 录制开启、写入 20 条标记明确的测试消息、关闭成功，
  回读文件确认 20 条消息。这是本地录制链路测试，不是真机动捕数据。

**仍需区别三条链路：** Mac Foxglove 的 TCP/WebSocket 和 SSH 日志后端已验证；
Mac Docker 到现场 DDS 的直接订阅测试未收到话题，不能据此宣称原生 DDS/NatNet 已打通。
现有 Linux 采集端可以继续承担 OptiTrack；若把 Motive 网线接到 Mac，需要单独验证
网卡、NatNet 流、容器 UDP/DDS 转发和时钟。手柄也尚未接入 Mac 容器。
时间校准、站立、接球、发球不能用以上预检代替。

### 本次 PASSIVE 启动与现场阻塞

用户确认机器人可靠支撑且有人值守后，于 2026-10-18 04:00:36 UTC 提交 START。
使用机器人原有 `/agibot/a3_deploy_model14600_recordfix_f0932ed1` 包，没有上传本机示例策略。
会话为 `model21800_20261018T040036Z`；这个历史会话前缀不代表实际模型版本。
六路 HAL 输入均报告 `ready=yes`，MDU Runner 启动并持续上报 `PASSIVE`、约 500 Hz、
`command_fault_latched=0`，没有发送 Stand、Ready、Serve、遥操或夹爪动作。

约 04:02 UTC HDU 重启，MDU 没有重启，HAL/Runner 保持 PASSIVE。
HDU 的启动记录显示此前也约每 11 分钟重启一次，现场确认没有人工重启操作。
已从 `/var/lib/systemd/pstore/dmesg-ramoops-0` 找到本次崩溃的持久化证据：
`[636.478670] Kernel panic - not syncing: RCU Stall`，内核为厂商 `6.1.118 PREEMPT_RT`。
停滞任务 `ktimers/6` 的栈涉及 `rt_spin_lock`、`__hrtimer_run_queues`；
另一 CPU 的栈涉及 `pl330_irq_handler`。此前还有 SPI busy 与 RT throttling 记录。
这确认了内核 panic 是重启的直接原因，但尚不能仅凭栈确定具体驱动缺陷或修复补丁。
重启丢失了 HDU 的 Planner 和 supervisor 会话内存，因此**完整 START 验收未通过**。
已按原会话恢复 Runner transport，收到新鲜的 `alive=1 mode=PASSIVE session_matches=1 fault=0`。
通过已有的 managed-stack preflight 保护使 supervisor 进入 `FAILED / MANAGED_RECOVERY_REQUIRED`，
不会把重启后的默认 STOPPED 误认为机器人全部停止。Mac 日志状态恢复为 OFF、无错误。

当前须保持现场支撑并先修复 HDU 内核崩溃，再走正常恢复/重新启动流程。
未改内核、关闭 panic/watchdog 或调整实时保护。匹配的厂商内核/固件修复版本尚未提供。
不得仅凭可连接 Foxglove 就进入站立或击球。HDU NTP 世界时钟偏差仍约 -6.2 秒，尚未执行时间校准。
证据见 `robot-104/passive-live.json`、`runner-passive-evidence.log`、
`hdu-kernel-panic.txt`、`reboot-diagnostic.log`、`reboot-syslog.log` 与 `reboot-vendor.log`。
`hdu-reboot-diagnostics.zip` 包含可供厂商检查的崩溃记录和简要说明，未包含 SSH 密钥。

后续现场重启复验（2026-10-17 PDT）：约 21:12 HDU/MDU 都重启，HOPE 回到 STOPPED，
Runner 未运行。本轮只读观察期间，Foxglove 连续 60 秒收到 839 条状态消息；约 21:23
HDU 再次失联，随后 boot ID 改变。新 pstore 明确记录
`[692.480028] Kernel panic - not syncing: RCU Stall`，即开机约 11 分 32 秒后复发。
因此本次重启未解决故障。SPI busy 仅记录，不单独作为阻塞依据；验收失败依据是实际重启和新的 panic。
最新状态以 `post-recurrence-live.json` 为准，原 PASSIVE 记录是重启前的验证结果。

Docker Desktop 使用 Linux 虚拟机及容器网络，现场 DDS/NatNet 需要按实际网络验证；
相关平台行为见 [Docker 官方网络说明](https://docs.docker.com/desktop/features/networking/)。

### 2026-10-17：合入新的 START 后台清理流程

已将 HOPE 上游 `58108499672dcf47bf76d4ef9f7b65f1b652bad2` cherry-pick 到本机
`build_6`，本地提交为 `f1c93c16f14124c0fd360903470c3cc0511e2116`。
原有 Mac direct runtime、Linux distrobox 路径与机器人模型路径均保留。
Linux 操作端容器内的 `test_catch_background.py` 与 `test_lifecycle_core.py`
共 27 项通过，helper 的 Bash 语法检查通过。

HDU 已安装新的 `hope_catch_background.py`，并将本次 commit 的精确增量应用到
已部署的 `hope-lifecycle` 和 `hope_lifecycle_supervisor.py`，保留它们其他现场差异。
HDU 原文件备份位于 `/var/backups/hope-foxglove/581084996-20261018T043022Z`。
仅重启了 lifecycle supervisor 以加载 180 秒预检超时，没有提交 START。
安装后文件 SHA-256 与本地准备文件一致；完整记录在
`artifacts/macos_setup/foxglove-581084996/`。

**首次部署发现的启动权限兼容问题（已按下节修复）：** 机器人 supervisor 的
`NoNewPrivileges=true` 生效，进程 `/proc/<pid>/status` 中 `NoNewPrivs=1`。
新清理脚本在厂商 UI 活跃时调用 `sudo -n systemctl stop agibot_ui.service`；
在相同的 no-new-privileges 标志下，sudo 明确返回无法提权，退出码为 1。
检查时 `agibot_ui.service` 为 active，因此在原服务配置下，新 START 预检会因
`UNRELATED_BACKGROUND_NOT_STOPPED` 被阻止。即使 SSH 登录用户有免密 sudo，
也不能据此认定 supervisor 内部可以执行 sudo。
首次部署时保留了该服务限制，随后按用户要求改为下节的上游 sudo 方案。
完整启动尚未验收；此更新是否缓解此前的 RCU Stall 重启也尚未验证。

### 当前配置：使用 main 的原版启动流程

按用户要求，2026-10-17 21:41 PDT 已撤销临时 polkit 方案。
本地及 HDU 的 `hope_catch_background.py` 均与 HOPE/main
`58108499672dcf47bf76d4ef9f7b65f1b652bad2` 完全一致，继续使用上游的
`sudo -n systemctl stop agibot_ui.service`。本地测试与 README 的临时改动已撤销，
新增的 polkit 规则及其专项测试已移除。Mac/Linux 支持和当前机器人模型配置保留。

上游的清理脚本需要 sudo，而其 service 模板仍含 `NoNewPrivileges=true`。
为让原版脚本在这台 HDU 上直接执行，已安装仅作用于 lifecycle supervisor 的配置：
`/etc/systemd/system/hope-lifecycle-supervisor.service.d/70-upstream-startup-sudo.conf`：

```ini
[Service]
NoNewPrivileges=false
```

服务已重新加载并重启，实际进程确认 `NoNewPrivs=0`；使用既有的 agi sudo 策略，
没有增加新的 sudoers 规则。此前 `/etc/polkit-1/rules.d/49-hope-background-cleanup.rules`
已移除。其他服务配置保持原样。

验证结果：

- 原版后台清理及生命周期测试共 27 项通过，helper Bash 语法检查通过。
- 在实际服务挂载命名空间中，以 agi 执行上游的完整 sudo 停 UI 命令，返回码 0，
  无标准错误输出。测试时 UI 已处于 inactive，没有为测试重新启动它。
- KERNEL、OPTITRACK 两种 HDU 预检均返回 COMPLETE，清理回执均为 PASS。
- 录制、进程管理器、Foxglove Bridge、chrony 等非目标进程 PID 保持不变。

本轮仅执行启动预检，没有发送 START 或关节动作；此前 RCU Stall 的长期稳定性
不由这次预检结果证明。完整工作区仍是 build_6 加原有 Mac 适配，未切换整个项目分支。
HDU 备份：`/var/backups/hope-foxglove/main-startup-20261018T044112Z`。
当前验证记录：`artifacts/macos_setup/main-startup-alignment/`。
此前 polkit 方案的证据保留在 `foxglove-cleanup-permissions/`，仅作历史记录。

### 2026-10-17 22:05 PDT：5 秒 PD Stand 与 Planner 位姿延迟检查更新

> 2026-10-18 源码回退：已撤销本节中的 Planner 位姿年龄 gate 移除，恢复
> `target_turn_max_pose_age_s: 0.2` 及接收年龄、源时间戳年龄两项检查。
> 5 秒 PD Stand 和 Mac/Linux 操作端支持继续保留。下方哈希与运行结果是
> 2026-10-17 的部署记录；本次源码回退尚未替换机器人上的 Planner。

按现场反馈，`a3_deploy_onnx_ref_pingpong` 的 PD Stand 使用独立的
`--pd-stand-blend-sec` 参数，默认 **5.0 秒**。从 PASSIVE 进入时，以实测关节位置、
速度和零增益作为起点，在 5 秒内用五次平滑曲线过渡到原站姿及其 Kp/Kd；
入口速度的延续在 0.25 秒内消退。策略动作的原有过渡时间独立保留。
此修改调整的是目标位置、速度及增益的命令过渡；实体机器人的实际站起过程仍须现场测试。

Planner 删除了身体位姿接收时间与源时间超过 200 ms 时拒绝击球的两处判断：
`target_turn_base_stale`、`target_turn_base_source_stale`。
对应的 `target_turn_max_pose_age_s` 参数已移除。位姿年龄继续记录到审计数据；
无效/缺失位姿、目标几何和已经错过的击球时刻仍按各自逻辑处理。

源码先在本地 `build_6` 修改，再通过本机 Docker 的 Debian 12 / ARM64 交叉环境构建。
本次修改尚未提交 Git。9 项命令过渡测试、12 项 Planner ROS 集成测试全部通过；
延迟测试同时覆盖 Pose 和 Flat 输入。HDU 上的隔离 Planner 启动检查通过。

部署结果：

- MDU 活动链接 `/agibot/a3_deploy_build4_serve025` 指向
  `/agibot/a3_deploy_stand5s_98724f730cc4`；Runner SHA-256 前缀 `98724f730cc4`。
- HDU 活动 Planner 位于 `install_model21800_fix/hope_planner_cpp`；
  SHA-256 前缀 `6bfa6c18acf47`。整个 Planner 源码包及活动配置均与本地载荷逐文件校验一致。
- 通过正常 lifecycle 停止后重新启动，会话 `model21800_20261018T050406Z`。
  状态 `RUNNING / PASSIVE`、`alive=1`、`session_matches=1`、`fault=0`，HAL 六组反馈 ready。
  实际 `/proc/22568/exe` 的哈希与本地新 Runner 一致，启动日志确认 `pose_gain_blend_sec=5.000`。
- Mac 到 `ws://10.44.84.104:8766` 的 Foxglove 连接检查通过：116 个通道，
  对生命周期、Runner 和 field 状态订阅在 10 秒内收到 141 条消息。
- 保留现场 KERNEL 模式、现有模型 14600（ONNX 哈希前缀 `a987240610bb`）和发球 CSV。
  KERNEL 不启动 Planner；在 OPTITRACK 模式启动时会使用更新后的 Planner。
  本次没有发送 PD_STAND、Motion、Serve 或夹爪动作。

原 MDU 完整运行包仍在 `/agibot/a3_deploy_model14600_recordfix_f0932ed1`；
HDU 原 Planner 源码、二进制及配置备份在
`/home/agi/hope_updates/stand5s-98724f730cc4/backup`。
本地源码载荷、文件哈希、构建和测试日志、实际运行证据统一保存在
`artifacts/macos_setup/stand-and-planner/`。后续构建无需在机器人上手改源码。

### Mac/Linux 兼容性复核

两端共用机器人上已部署的 Runner 和 Planner；5 秒 Stand 与位姿延迟检查的修改
不依赖 Foxglove 操作端的操作系统。Mac 通过 Docker 提供 Linux ROS 环境，Linux
继续使用现有 Distrobox `hope` 环境。`HOPE_LAPTOP_RUNTIME` 默认值仍是 `distrobox`，
仅 Mac operator 容器明确设置为 `direct`。

复核中修正了 Mac SSH dispatcher 的日志回收动作名：实际 supervisor/helper 使用
`collect`，原 dispatcher 错写为 `collect-logs`。源码、正在运行的 Mac 容器及
`hope-operator:jazzy-arm64` 镜像均已同步。HDU 通过实际 SSH 通道调用 Mac 的
`preflight-laptop` 成功；对独立空测试会话调用 `collect` 成功返回
`NO_REMOTE_SESSION_LOGS`，证明请求已完整走通 Mac、HDU 和 MDU 的检查链路。

验证包括 44 项操作端测试、7 项新增跨平台专项测试，以及 x86_64 Linux 环境中的
12 项 Planner ROS 集成测试和 9 项 Stand 过渡测试，均通过。
运行路径测试覆盖 Linux 默认/显式 Distrobox、Mac 容器 direct 和包含空格的路径。
测试及构建证据位于 `artifacts/macos_setup/cross-platform/`。

随后通过用户提供的 Tailscale 地址 `100.109.128.56` 连接 Linux 实机，确认它的
当前局域网地址是 `10.10.10.11`。原 Laptop 配置指向训练 checkout
`/home/dongc1/workspace/Hope_v11`，该目录没有 ROS install；旧的独立 HOPE checkout
也没有当前 C++ Planner。因此本轮新建独立部署目录：

`/home/dongc1/workspace/HOPE_deploy_stand5s_98724f73`

该目录包含本机同版本的非训练源码、动捕标记资源、完整 x86_64 Runner 包和当前机器人
策略的验证副本。原训练 checkout 和已有运行目录保留。Linux `hope` Distrobox 已补齐
`tmux`、Python `mcap`/`rosbags` 等缺失运行依赖，并原生构建全部六个启用的 ROS 包。
Linux 实机上的 12 项 Planner、9 项 Stand、51 项操作端测试全部通过；Runner strict
loader 成功加载与机器人一致的 `ball_clock_110_v1` 策略。Foxglove 扩展为 1.8.12，
本机资产服务返回 HTTP 200。

Linux 的 `~/.config/hope-foxglove/lifecycle.env` 已指向新目录，保留 Distrobox 路径。
用户级 lifecycle、录制和 marker helpers 已更新；原配置与文件备份在新目录
`operator-before/`。安装后的 Laptop 预检返回 `LAPTOP_LOG_COLLECTOR_READY`。
源码及文件校验记录见新目录 `SOURCE_SHA256SUMS`，构建和测试记录见 `linux-build.log`。

**现场网络尚未验收：** Linux 当前网络无法访问机器人 `10.44.84.104`，用户确认电脑
不在现场，暂时无法切换网络。本轮没有用 Tailscale 地址替换现场 DDS/Laptop 地址。
以后把 Linux 接到机器人网络，在 STOPPED 状态将 Foxglove 的 `laptop_wifi_ip`
设置成那时的 Linux 局域网地址，再进行正常 START。若从 Mac 使用 `select-linux`，
先把 `~/.config/hope-macos/operator.json` 的 `linux_ip` 更新为同一个地址，再执行
`bind` 和 `select-linux`。原地址 `10.44.165.132` 不能当作当前已验证地址使用。
