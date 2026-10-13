# Deployment and native Gate 3 command sequence

Runtime workflow ported from the October 11–13 updates and adapted to the
public `main` layout and package builder. The installation guide is
[DISTROBOX_SETUP.md](../DISTROBOX_SETUP.md). Commands below build/package or
inspect locally; simulation and hardware operations are identified separately.
The native deployment project in this checkout is `a3_deploy/a3_deploy_example/`.

## 0. 怎样才算成功

本文前半部分保留构建和部署命令，后半部分是现场操作顺序。
球参数的标定项目、当前系数与消费者见 [球物理校准](ball_physics_calibration.md)；
USB 手柄安装和按键见 [Runner / Xbox](runtime_xbox.md)。
本次文档更新只核对本地代码，没有连接机器人、启停服务或执行仿真。

| 阶段 | 必须看到的证据 | 此时可以说什么 |
| --- | --- | --- |
| 电脑端构建/模型加载 | 指定 ONNX 与 deploy.yaml 完整且同一次导出；x86 包构建成功；实际 Runner `--inspect-policy` 成功；需要的 ROS 消息/Planner 和 MuJoCo 已安装 | 电脑端包可加载；还没有闭环通过结论 |
| 电脑端 Gate3 | 相同模型、指定场景的完整运行成功退出；`run.json.certification_pass=true`；物理球、机器人、Planner、完整收尾/归档证据满足场景要求 | 这一版本/场景的仿真通过 |
| 机器人部署/启动 | 目标机上的 ARM 程序、策略与配置 SHA 符合本次版本；实际 PID 指向该版本；生命周期最终 `RUNNING`，Runner 新鲜反馈 `PASSIVE`，tick 与关节/IMU 接收在更新 | 实机部署启动成功；未证明能接球 |
| 实机输入就绪 | 动捕、世界/pelvis 坐标、时钟、球包、HDU→MDU 传输及手柄按需通过；Record ON 后三机同段确认 | 可以开始相应的有人值守测试 |
| 实机击球验收 | 预先约定的快/慢球、连续多球协议；实际接触、合法回球、回位及摔倒/故障逐球记录，达到事先写明的目标 | 只对本次策略/场地/测试覆盖范围确认效果 |

不能用单个“PASS”概括全部阶段：`--check`、SSH 连通、Foxglove 有画面、START 请求被接受、零摔倒或第一球打中，均不足以替代下一阶段。
未测试的阶段写 `NOT_PROVEN`。如果 Gate3 未通过却进行诊断性实机测试，记录为实验测试，不写成正式验收通过。

## 1. Select the exact two-file policy

From the repository root on the Laptop host:

```bash
export HOPE_ROOT="$(git rev-parse --show-toplevel)"
export HOPE_POLICY_DIR=/absolute/path/to/policy
test -s "$HOPE_POLICY_DIR/params/deploy.yaml"
test -s "$HOPE_POLICY_DIR/exported/policy.onnx"
```

Both files come from the same export. For the bundled public policy, use
`a3_deploy/a3_deploy_example/models/model_21800/policy`. Export instructions for
other policies remain in [the public quickstart](../../QUICKSTART_A3_ISAAC.md). A training checkpoint alone is not a
deployable package. Keep the exported metadata and motion identity; a new
policy does not require editing C++ or changing the observation recipe by hand.

Use a fresh worktree when retaining an existing generated package: the builder
recreates `dist/a3_deploy_ARCH/`. The following commands are not installation
or execution instructions for an already running robot.

## 2. Build the x86 Runner package inside hope

Run from the initialized host terminal above:

```bash
distrobox enter hope -- env \
  HOPE_ROOT="$HOPE_ROOT" HOPE_POLICY_DIR="$HOPE_POLICY_DIR" bash -lc '
  set -eo pipefail
  source /opt/ros/jazzy/setup.bash
  cd "$HOPE_ROOT/a3_deploy/a3_deploy_example"
  source setup_a3_env.sh
  bash scripts/build_a3_deploy_pkg.sh \
    --arch x86_64 \
    --runtime-cfg src/a3/a3_deploy_onnx_ref/config/a3_runtime_config.pingpong.hitter_pingpong.yaml \
    --policy-dir "$HOPE_POLICY_DIR" --jobs 8

  dist="$PWD/dist/a3_deploy_x86_64"
  test -x "$dist/run_a3_pingpong.sh"
  LD_LIBRARY_PATH="$dist:/opt/ros/jazzy/lib" \
    "$dist/a3_deploy_onnx_ref_pingpong" \
    --inspect-policy --policy-dir "$dist/policy"
'
```

`--inspect-policy` loads the native policy contract and exits before transport
or robot backend initialization. This is a model-loading check, not a Gate 3
execution. Full packaging stages `policy/`, runtime/AimRT configs, libraries,
messages and available serving assets. `--build-only` skips runtime asset
staging and does not replace this package step.

The public builder includes the bundled HumanLike locomotion assets by default.
Use `--teleop-policy-dir /absolute/path/to/humanlike` to override them, or
`--without-teleop` to omit locomotion explicitly. An override must contain
`humanlike.yaml`, `policy.onnx` and `lin_vel_encoder.onnx`. `--stage-only` stages
an already compiled package; it does not rebuild changed binaries.

## 3. Build the intended ROS workspace and simulator install

From the host, build the message/bringup/C++ Planner dependency closure:

```bash
distrobox enter hope -- env HOPE_ROOT="$HOPE_ROOT" bash -lc '
  set -eo pipefail
  source /opt/ros/jazzy/setup.bash
  cd "$HOPE_ROOT/hope_ws"
  rosdep install --from-paths src --ignore-src --rosdistro jazzy -r -y
  colcon build --symlink-install \
    --base-paths src ../NatNet2ROS2/src \
    --packages-up-to hope_msgs hope_bringup hope_planner_cpp motion_capture_tracking \
    --cmake-args -DBUILD_TESTING=OFF \
      -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3
  source install/local_setup.bash
  /usr/bin/python3 -c "from motion_capture_tracking_interfaces.msg import NamedPoseArrayV2"
  test -x install/hope_planner_cpp/lib/hope_planner_cpp/hope_planner_cpp_node
'
```

Build and install the current native MuJoCo source into its own ignored build
directory. Installing Python `mujoco` does not perform this step.

```bash
distrobox enter hope -- env HOPE_ROOT="$HOPE_ROOT" bash -lc '
  set -eo pipefail
  source /opt/ros/jazzy/setup.bash
  sim="$HOPE_ROOT/a3_deploy/A3_MuJoCo_Sim/aimrt_mujoco_sim"
  sim_build="$sim/cmake-build-gate3-current"
  cmake -S "$sim" -B "$sim_build" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX="$sim_build/install" \
    -DAIMRT_MUJOCO_SIM_INSTALL=ON \
    -DAIMRT_MUJOCO_SIM_BUILD_WITH_ROS2=ON \
    -DAIMRT_MUJOCO_SIM_BUILD_TESTS=OFF \
    -DAIMRT_MUJOCO_SIM_BUILD_EXAMPLES=OFF \
    -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3
  cmake --build "$sim_build" --target install --parallel 8
  test -x "$sim_build/install/bin/aimrt_main"
'
```

First builds fetch native dependencies unless the matching cache is already
present. Rebuilding source without installing does not update the runtime
selected by Gate 3. A historical `build_v11/install` or a Humble standalone
bundle is not proof that the new source was installed.

## 4. Configure paths, inspect, then run native Gate 3

From the host terminal with `HOPE_ROOT` and `HOPE_POLICY_DIR` set:

```bash
cd "$HOPE_ROOT"
./run_gate3.sh --configure \
  --runner-dist "$HOPE_ROOT/a3_deploy/a3_deploy_example/dist/a3_deploy_x86_64" \
  --sim-install "$HOPE_ROOT/a3_deploy/A3_MuJoCo_Sim/aimrt_mujoco_sim/cmake-build-gate3-current/install" \
  --hope-ws "$HOPE_ROOT/hope_ws"
./run_gate3.sh --policy-dir "$HOPE_POLICY_DIR" --check
```

The unified entry accepts `ball_clock_110_v1`, `small_station_112_v1` and
`small_station_324_v1`; the native inspection receipt determines compatibility.
Other ABIs need their own supported entry. `--configure` first requires the
installed Runner/wrapper, simulator executable and workspace overlay to exist.
It saves only machine-local paths in ignored `config/gate3/runtime.local.yaml`.
After moving a worktree, configure the paths again. Relative paths *inside*
runtime YAML resolve against the repository root; an explicit `--runtime` file
path resolves from the invocation directory.

`--check` creates a fresh result directory with the copied policy, scenario,
inspection and `run.json` in `prepared` status. It starts no simulation or
transport and has no certification verdict. It is not a read-only filesystem
operation. `--dry-run` is its alias.

The following command starts the physical-ball simulation:

```bash
./run_gate3.sh --policy-dir "$HOPE_POLICY_DIR" --scenario fixed-home --headless
```

The default is 26 balls. To run 100 balls with the viewer:

```bash
./run_gate3.sh --policy-dir "$HOPE_POLICY_DIR" --balls 100 --viewer
```

For a custom scene and explicit new output directory:

```bash
./run_gate3.sh --deploy /absolute/path/to/deploy.yaml \
  --onnx /absolute/path/to/policy.onnx \
  --scenario /absolute/path/to/scene.yaml \
  --output /absolute/path/to/new_gate3_result
```

The fixed-home family requires six clean serves and an even total of at least
26. If the scene supplies per-ball timing lists, their length must match the
selected ball count. Use `--viewer` or `--headless`, not both. Do not mix
`--policy-dir` with `--deploy`/`--onnx`. `--objective`, `--qualification` and
`--preflight-only` are not options of `run_gate3.sh`.

The unified entry clears inherited `PP_*` and `A3_*` run knobs. Select paths
with its CLI/runtime YAML and thresholds with scene YAML. It serializes its
shared `/tmp`/iceoryx backend and rejects an active competing runtime. Ctrl-C
cleans this run's recorded process groups and archives the available evidence.
See [gate3_two_file_runner.md](gate3_two_file_runner.md) for flight draining.

## 5. Read the saved result

The console prints the new result directory, normally under
`a3_deploy/a3_deploy_example/gate3/runs/`. In a new terminal, set `HOPE_GATE3_RESULT`
to that exact directory and read the retained JSON:

```bash
export HOPE_GATE3_RESULT=/absolute/path/to/gate3_result
python3 - "$HOPE_GATE3_RESULT" <<'PY'
import json
import sys
from pathlib import Path

root = Path(sys.argv[1])
for name in (
    "run.json", "pp_rally_report.json", "pp_physical_ball_report.json",
    "pp_mujoco_plant_report.json", "pp_planner_cpp_report.json",
    "pp_gate3_drain_report.json",
):
    path = root / name
    print(f"\n{name}")
    print(json.dumps(json.loads(path.read_text()), indent=2)
          if path.exists() else "MISSING")
PY
```

The full run requires a successful exit and `run.json.certification_pass=true`,
including the final rally, flight-drain and archive checks. A `prepared` check,
missing report, contact count, zero falls or completed video is not that result.
Read physical-ball, plant and Planner failures separately; the result applies
to the saved model, scenario and runtime only. Simulation certification does
not establish real-hardware qualification.

## 6. Cross-build and stage a hardware package

Cross-building uses Docker on the Laptop host, not the Isaac environment.
Provision the target's Rockchip sysroot through the documented exporter only
when it is missing; do not assume a fresh clone contains the archive.

```bash
cd "$HOPE_ROOT/a3_deploy/a3_deploy_example"
test -f thirdparty/rockchip_sysroot/rockchip-1.0-aarch64-sysroot.tar.gz
bash scripts/build_a3_deploy_pkg.sh --arch rockchip \
  --runtime-cfg src/a3/a3_deploy_onnx_ref/config/a3_runtime_config.pingpong.hitter_pingpong.yaml \
  --policy-dir "$HOPE_POLICY_DIR" --jobs 8
```

`--arch rockchip` selects the target toolchain; it does not convert the selected
ONNX into an RKNN policy. Model backend and policy contract still come from the
selected runtime/export. `scripts/export_rockchip_sysroot.sh --help` documents
the vendor image and output options. Thor uses its own matching sysroot.

To stage the complete package into a **new** immutable slot, substitute the
current SSH endpoints and a new slot name in a host terminal:

```bash
export HOPE_HDU_SSH=agi@HDU_ADDRESS
export HOPE_MDU_SSH=agi@MDU_ADDRESS
export HOPE_DEPLOY_SLOT=/agibot/a3_deploy_SELECTED_RELEASE
ssh -J "$HOPE_HDU_SSH" "$HOPE_MDU_SSH" \
  "test ! -e '$HOPE_DEPLOY_SLOT' && mkdir -p '$HOPE_DEPLOY_SLOT'"
rsync -azP -e "ssh -J $HOPE_HDU_SSH" \
  "$HOPE_ROOT/a3_deploy/a3_deploy_example/dist/a3_deploy_rockchip/" \
  "$HOPE_MDU_SSH:$HOPE_DEPLOY_SLOT/"
```

These are operator commands; the documentation update did not connect to a
robot. Staging does not select the slot or start Runner. Continue with the
field procedure below. The [first-deployment guide](foxglove_first_hardware_test.md)
retains installation and historical recovery details; its old Cali V2/V3 and
manual recording steps are not the default current workflow.

### 三机分别需要部署什么

| 位置 | 必须包含 | 验证依据 |
| --- | --- | --- |
| Laptop | `hope` ROS 环境、动捕/坐标桥、球包器、相应消息定义、Foxglove Console 与布局、可选 Xbox 输入服务 | 实际安装目录/程序 SHA、动捕与球包 ROS 数据、手柄预览 |
| HDU | ARM C++ Planner、匹配消息/配置、base 与 Runner transport relay、lifecycle/field/recording 服务、8766 控制桥 | 实际进程与参数、Planner 审计、Runner 状态回传 |
| MDU | 完整 Rockchip Runner 包、策略及元数据、运行配置/库、HAL/夹爪配套、需要时的 `teleop_humanlike` 和原始记录组件 | 本机加载检查、启动日志、实际 Runner tick/状态、策略/二进制 SHA |

只上传 ONNX、只安装 `.foxe` 或只替换 Laptop Planner，都不算三机部署完成。
HDU Planner 与 MDU Runner 是两个独立产物；x86 ELF 不能复制去冒充 ARM 程序。

选择新 MDU slot 前保存旧目标与 SHA，机器人可靠支撑并进入 PASSIVE，再由既有 lifecycle 停止会话。
在 **MDU helper 实际读取的** `~/.config/hope-foxglove/lifecycle.env` 中维护 `HOPE_MDU_DEPLOY`，
或按已安装的固定部署链接选择新 slot；只改 Laptop 同名文件不等于 MDU 已切换。
同时按本次版本更新 HDU Planner 和 Laptop 球包器；不要覆盖正在运行的二进制。
启动后核对 `/proc/<PID>/exe`、进程参数、策略 SHA 和运行时参数，再记录采用结果。
回退也经过相同的受支撑/PASSIVE→停止→旧 slot→PASSIVE 流程，保留失败会话数据。

## 7. 跑之前逐项看什么

| 检查 | 具体看什么 | 正常判据 / 不正常时怎么办 |
| --- | --- | --- |
| 场地与机器人 | 防倒支撑、足下和挥拍空间、夹爪/球、实体急停、伺服故障 | 有人值守；急停状态和本体故障已按现场流程处理。软件 Reset 不能释放硬件急停 |
| 版本 | ONNX、deploy.yaml、Runner、HDU Planner、球包器、MuJoCo 安装和配置 SHA | 与本次选定版本对应；`model21800` 文件名/session 前缀不代表实际策略就是 model21800 |
| 控制归属 | 当前 Runner、HAL、厂商控制服务、残留仿真 | 身体命令只有一个 owner。使用 lifecycle 管理，不再叠加另一套手动 Runner/厂商控制器 |
| 网络 | Laptop↔HDU，HDU↔MDU，HDU→Laptop 的免交互 SSH；Motive 到 Laptop 的独立网口/路由 | 每条实际使用的链路可用。Motive link-local 地址不能填进 HDU Wi-Fi 栏 |
| Foxglove | `ws://<HDU-IP>:8766`；Console、Runner/lifecycle 新鲜状态与服务 | 8766 控制连接成功且状态更新时间在推进；8765 是只读端，页面静态显示不是连接证据 |
| 时钟 | Laptop/HDU/MDU 的同步状态、源时间/接收时间、源年龄是否异常 | 没有负龄/大跳变或持续陈旧；不把 ROS 接收时间替换成相机曝光时间来掩盖延迟 |
| 坐标 | 近端左桌角原点、桌面 z=0、+X 向对方、+Z 向上、P1/P2、机器人胸前方向 | 转换后的球/机器人同时符合规范，机器人正对球桌时 heading 正确；进入 READY 前确认 |
| 动捕 | `Ball`（兼容 `ball`）及选定 `UCB_P1/UCB_P2`，rigid body 有效与时间推进 | 不只是能 ping Motive；需要实际 NatNet ModelDef 与连续帧 |
| 标定 | Motive Asset 状态、有效 pelvis/world receipt、独立 `x_hit` | `ASSET MATCHED` 只证明模型定义；还要确认安装/世界坐标与实际击球平面 |
| 参数 | 两节点实际 ROS 参数、Gate3 的球物理回执、控制模式/转向设置 | 与 [球物理校准](ball_physics_calibration.md) 的本次选定值一致，不能只看 YAML 文件 |
| 手柄 | Laptop 设备、输入服务、HDU 地址、Console Controller/Input/Sticks | 新鲜 `LIVE`、设备唯一、LT 已松开、摇杆零值；具体流程见下方链接 |
| 记录 | 空间、Record 实际状态、session/segment、三机记录器 | 按 ON 后等 `ON` 再试球；`ERROR` 不算录制成功，保留错误及已有文件 |

本仓库 [世界坐标配置](../../hope_ws/src/hope_bringup/config/hope_world_frame.yaml)
当前保留的是“原始 Motive +X 向左、+Z 向上、近端左角桌面原点”的 **+90° yaw** 转换。
如果新工程已经把 +X 设置为向对方，不可再沿用这次 +90°；需要重新确定并记录变换。
驱动先统一 Motive 世界坐标，P2 再绕桌心旋转 180°；球与机器人姿态必须一起变换。
policy 的桌面→地面高度偏移 `+0.76 m` 只应用一次。

当前默认使用导入的 pelvis 对齐 12-sticker Motive asset，Console 1.8.12 没有旧 Venue JSON / Cali V2/V3 按钮。
见 [当前 asset 流程](../../foxglove/extensions/hope-a3-console/README.md)。
不要把“全部 12 片可见”作为 ModelDef 匹配的同义词，也不要用 nominal asset match 代替实际安装正确性。

### 只读检查命令示例（由操作者需要时执行）

在 Laptop 的仓库根目录填写本次地址，不照搬以前测试的 IP：

```bash
export HOPE_ROOT="$PWD"
export HOPE_LAPTOP_IP=LAPTOP_WIFI_IP
export HOPE_HDU_IP=HDU_WIFI_IP
export HOPE_MDU_IP=MDU_INTERNAL_IP
export HOPE_MOTIVE_IP=MOTIVE_PC_IP
export HOPE_TABLE_SIDE=P1
export HOPE_OPERATOR_USER="$USER"

ip route get "$HOPE_HDU_IP"
ip route get "$HOPE_MOTIVE_IP"
ssh -o BatchMode=yes -o ConnectTimeout=5 "agi@$HOPE_HDU_IP" hostname
ssh -o BatchMode=yes -o ConnectTimeout=5 -J "agi@$HOPE_HDU_IP" "agi@$HOPE_MDU_IP" hostname
ssh -o BatchMode=yes "agi@$HOPE_HDU_IP" \
  "ssh -o BatchMode=yes '$HOPE_OPERATOR_USER@$HOPE_LAPTOP_IP' true"
```

Motive 开流后、启动正式桥接前，可以从 Laptop 做 NatNet 探测：

```bash
python3 hope_ws/src/hope_bringup/scripts/natnet_preflight.py \
  --hostname "$HOPE_MOTIVE_IP" --table-side "$HOPE_TABLE_SIDE"
```

成功需要 route、`NAT_SERVERINFO`、`MODELDEF` 中正确对象与 `FRAMEOFDATA`，并满足该探测器报告的频率/丢帧检查。
默认 `--min-hz=250` 是该工具的预检标准；它不是某次实测帧率，也不是球包器 `expected_mocap_hz` 的同义项。
先处理首个失败层，不用反复 START 代替网络检查。

ROS 查看需在已加载 Jazzy、`hope_ws/install/local_setup.bash` 的 `hope` 环境，使用与现场一致的 domain 232 / Fast DDS peer 配置：

```bash
ros2 param dump /hope_planner
ros2 param dump /hope_ball_flight_packetizer
ros2 topic echo --once /hope/lifecycle/state
ros2 topic echo --once /hope/runner/mode
ros2 topic echo --once /hope/planner/hit_plane
```

这些命令读取当前状态，不启动运动。球静止时没有 `/ball/flight_packet` 是正常现象：球包只在有效来球提交时产生。
日常看 Console 的真实状态/回执即可，不要求每次重跑所有构建与仿真测试。
需要 Time Calibration 时，只在受支撑且会话停止后用既有流程操作；它会调整时钟并暂时断开 Foxglove，不能在挥拍过程中执行。

## 8. 实机从 START 到连续接球

1. **STOPPED 时选定模式与配置。** 正常来球选 OptiTrack / Normal Play，填写地址、P1/P2 并 `CONFIRM CONFIG`；需要发球时先选正确 CSV。Kernel 是无动捕 actor/模式测试，不能证明真实来球链路。
2. **START SYSTEM。** 请求返回 accepted 只是开始。等待最终 `RUNNING`、`busy=false`、Runner 新鲜 `PASSIVE`，检查相应 session/实际程序。失败按显示的 PREFLIGHT/PLANNER/HAL/RUNNER 层定位。
3. **正常接球单独启动 Laptop OptiTrack。** 当前 supervisor 明确不自动启动/等待它，因此 RUNNING/PASSIVE 并不证明有球或定位。已运行桥接时不重复启动；新启动使用本次 session ID。
4. **Stand。** 从 PASSIVE 点击 Stand，等真实 `PD_STAND` 与稳定站姿；确认 heading、base、实际关节/IMU 正常。
5. **确认击球平面。** 需要更新时在 Stand 点击 `Refresh x_hit`，看 Planner acknowledged 与 `/hope/planner/hit_plane`；移动机器人后重新核对。它不负责纠正世界轴，也不是 Time Calibration。
6. **Record ON。** 等实际 `ON`，保存 session 和 segment。Normal Play 的三机采集应是同一段。
7. **RECEIVER → Ready。** 点击角色和 Ready，等实际 `MOTION`/接球等待状态。Xbox X 可以发同一序列；不能因为按钮未锁定就认定动捕/坐标已验收。
8. **先单球，再连续多球。** 记录发出、收到球包、有效命令、实际碰球、合法回球、完成回位及下一球是否接续。分别覆盖快球/慢球及第一球/第二球以后。
9. **结束。** 回 Stand 并确认，再 Record OFF 等 `OFF`。如要关停或换版本，先可靠支撑再 PASSIVE，执行 KILL + COLLECT，等结束后再关机。

第 3 步的 Laptop helper 示例会启动动捕和球包进程，不会启动身体运动。
在安装了当前可执行 helper、`HOPE_ROOT`/用户名本地配置正确且没有现存桥接时使用：

```bash
export HOPE_SESSION_ID=SESSION_ID_SHOWN_IN_CONSOLE
/usr/local/libexec/hope-lifecycle start-laptop \
  "$HOPE_SESSION_ID" "$HOPE_LAPTOP_IP" "$HOPE_HDU_IP" "$HOPE_MDU_IP" \
  "$HOPE_MOTIVE_IP" "$HOPE_TABLE_SIDE" OPTITRACK DEFAULT
```

如果本机安装使用 `HOPE_LAPTOP_LIFECYCLE_HELPER` 自定义路径，应使用那一份已部署 helper。
当前 KILL + COLLECT 不负责停止独立 Laptop 动捕桥；重新开会话要核对其 session/球包 session，必要时在适当的停止状态单独 stop-laptop/start-laptop。

**Stand、PASSIVE、Record OFF、KILL 与 E-stop 不等价。** Record OFF 只关数据，Stand 是有控制的站立转换；
PASSIVE/KILL/急停可能撤去支撑。KILL 技术上允许直接终止会话，现场仍须先保证机器人有支撑。
异常急停优先使用可达的实体急停；Xbox LB+RB 是依赖电脑/网络的锁存软件急停。

### 记录成功与数据导出

当前运行端由分布式记录器控制 Laptop mocap/球包、HDU Planner/ROS 和 MDU body-drive/双 IMU/Runner obs/trace。
Record 默认为 OFF；每次 ON 创建新的同名分段；OFF 不停策略/HAL。
录制成功要看最终状态和实际文件关闭情况，不能只看按钮文字 `Record OFF · Stop`。
切换失败为 ERROR 时保留已有文件与错误，不覆盖旧段来假装成功。

机器人 HDU/MDU 的原始持久数据在 `/agibot/data/bag/hope_pingpong_telemetry/sim2real_sessions/<session>/<role>`；
Laptop 本机数据及正常收集后的三机副本在 `real_logs/<session>/`。记录内容/缺口说明见 [三机录制](foxglove_lifecycle.md)。
只要求导出时，仅复制现有三机目录并打包到新目录，不自动调用 KILL、OFF、重启或分析脚本。
`hope_sim2real_export.py` 会做审计/转换，应只在确实要分析/转换时使用。

### 常见现象与应看的层

| 现象 | 优先核对 |
| --- | --- |
| 能 ping，Foxglove 连不上 | 是否连 HDU 的 8766、控制 bridge 与 WebSocket；不能用 MDU 地址替代 |
| RUNNING 但看不到球/不能接球 | 单独的 Laptop 动捕是否启动，Ball/UCB 名称、源时间、完整坐标与球包路径 |
| READY 马上转向 | 世界旋转是否重复应用、P1/P2、pelvis +X/安装、heading；先回 Stand，不用改阻力掩盖方向错误 |
| 第一球行、后续球不行 | 新 flight/session、outgoing→incoming 分段、恢复完成、时钟/延迟和执行跟踪；不能只看首球 |
| Planner 拒绝 | 审计具体 reason：无效/陈旧 base、过期击球时刻、`dead_ball`、模型解；不要一律归为 40° 限制 |
| 手柄亮灯但没反应 | Laptop evdev 权限/唯一设备、输入服务实际 HDU peer、Console 新鲜输入、按键重新松开再按 |
| TELEOP 不进入或不走 | 先实际 Stand、HumanLike 包已加载、设备新鲜且摇杆居中/LT 松开；ACTIVE 后重新 LT 使能 |
| Record OFF 出 ERROR | 记录器具体 role/segment、文件关闭/写盘错误；OFF 失败不是 Planner 故障 |

当前基础配置 `max_target_turn_deg=180` 表示原先 40° 拒绝已关闭；这只约束到击球点的夹角，不负责 READY 的初始朝向校正。

## Documentation verification boundary

The 2026-10-11 review checked the source CLI options, five command-argument
examples and the 26/100-ball scene composition in each checkout, plus Markdown
links/anchors and shell syntax. The documented build/install, full simulation
and hardware-staging commands were not executed during this review. They do
not carry a new build, Gate 3 or hardware verdict. The 2026-10-13 extension
additionally checked ball-physics consumers, lifecycle/recording behavior and
Xbox source mappings; its examples remain operator instructions, not actions
performed by the documentation update.
