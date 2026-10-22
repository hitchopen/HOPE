# Runner, Xbox and field operation

The public runtime uses **model_21800**, including its original ONNX, deployment
parameters and 110-input/31-action contract. The training and checkpoint workflow
remains described in [MODEL_21800](../MODEL_21800.md). Runtime features do not
require a newer checkpoint. No unpublished checkpoints, training configurations,
experiment results or robot-specific calibration receipts are distributed here.

For current x86/Rockchip packaging, installed simulator paths, Gate 3 execution
and retained reports, follow [deploy_and_gate3.md](deploy_and_gate3.md), checked
against this branch on 2026-10-11. Pass the selected policy/runtime explicitly;
this branch does not use public main's package defaults.

## 手柄接哪里、怎样确认已经接通

本节按 2026-10-13 仓库源码核对，没有连接手柄或机器人做新现场测试。
当前链路是：**Xbox → Linux Laptop 的 evdev → HDU relay → MDU Runner**。
手柄接运行 `hope-xbox-input.service` 的 Laptop，不是运行 Motive 的另一台电脑，也不是直接插到 HDU/MDU。
Foxglove 显示输入和发送模式请求；真正的身体指令始终只有 Runner 生成。

1. 用有数据能力的 USB 线把 Xbox Series S|X 接到 Laptop，保持 LT、LB/RB 和 A/B/X/Y 松开、摇杆回中。
2. 当前 [设备发现代码](../../foxglove/laptop/hope_xbox_preview.py) 只自动选择 VID:PID **045e:0b12**，并要求 X/Y/RX/Z 轴。
   只连接一个匹配手柄；多个匹配设备会报 `MULTIPLE_XBOX_DEVICES`，不会猜测使用哪一个。
3. USB 无线接收器、蓝牙或其他手柄可能呈现不同 PID/轴映射，当前实现没有通用自动适配；配对亮灯不能证明可用。
   已有经过验证的无线配置才继续使用，否则按 USB 数据线流程。不要仅改设备名字就认为按键一致。
4. 操作者账号必须能读取对应 `/dev/input/event*`，`hope` 容器内同一用户也要能读。event/js 编号会变化，不写死 `event0`。
5. 验证本地读取，再启动唯一的输入服务，最后在 Foxglove 看完整链路。

### 一次性环境安装（仅在没有安装时执行）

在 Laptop 仓库根目录执行。依赖放在 `hope` 环境，独立 venv，不使用训练 Python：

```bash
export HOPE_ROOT="$PWD"
distrobox enter hope -- bash -lc '
  set -e
  source /opt/ros/jazzy/setup.bash
  xbox_env="$HOME/.local/share/hope-xbox/venv"
  test -x "$xbox_env/bin/python" || /usr/bin/python3 -m venv --system-site-packages "$xbox_env"
  "$xbox_env/bin/python" -m pip install evdev
'
```

如果 `venv` 模块缺失，在 `hope` 容器内安装其系统 Python 对应的 venv 包再重试。
读取权限先看 `ls -l /dev/input/by-id/` 和对应 event 设备的权限/ACL；不要用 `sudo` 启动整套 Runner 来绕过权限。
如果本机缺少当前登录桌面用户的设备授权，可为这一个 USB 型号安装 udev 规则，再拔插手柄：

```bash
# Laptop 宿主机；一次性设备授权，不启动运动。
sudo tee /etc/udev/rules.d/70-hope-xbox.rules >/dev/null <<'RULE'
SUBSYSTEM=="input", KERNEL=="event*", ATTRS{idVendor}=="045e", ATTRS{idProduct}=="0b12", TAG+="uaccess"
RULE
sudo udevadm control --reload-rules
```

`uaccess` 面向本机活动桌面会话。远程/无桌面安装要由设备管理员给操作者配置对应权限；不要对所有输入设备设置全局可写。

下面的预览 **不加 `--ros` 或 `--control`**，只读手柄并打印 JSON，不请求机器人模式：

```bash
distrobox enter hope -- env HOPE_ROOT="$HOPE_ROOT" bash -lc '
  "$HOME/.local/share/hope-xbox/venv/bin/python" \
    "$HOPE_ROOT/foxglove/laptop/hope_xbox_preview.py" --seconds 10
'
```

应看到 `connected: true`、正确 device 名称，摇杆居中时 velocity 为零，LT 和按钮随操作正确变化。
`NO_XBOX_DEVICE` 也可能是权限导致设备打不开；先排权限，再核对 VID/PID/轴，不用启动机器人试探。

### 输入服务与 HDU 地址

已有服务只需确认状态和现场地址；没有安装时才复制仓库 unit：

```bash
install -D -m 0644 foxglove/laptop/hope-xbox-input.service \
  "$HOME/.config/systemd/user/hope-xbox-input.service"
systemctl --user edit hope-xbox-input.service
```

在编辑器填入以下 override，替换 `HDU_WIFI_IP` 与仓库路径。
将地址放进容器内的 `env`，避免依赖某次终端临时 export：

```ini
[Service]
ExecStart=
ExecStart=/usr/bin/distrobox enter hope -- /usr/bin/env HOPE_XBOX_HDU_IP=HDU_WIFI_IP /bin/bash %h/workspace/HOPE/foxglove/laptop/run_xbox_input.sh
```

`run_xbox_input.sh` 的源码默认 peer 仍是 `10.42.20.10`；它不会自动读取 Console 已填写的新 HDU 地址。
服务用 ROS domain 232 和 Fast DDS 单播，必须改为本次可达的 HDU 接口地址。
unit 默认仓库目录也是 `%h/workspace/HOPE`；移动仓库后一起改。

以下步骤会启用手柄控制请求。在有人值守、机器人受支撑且 PASSIVE/停止、手柄松开并回中的状态执行：

```bash
systemctl --user daemon-reload
systemctl --user enable --now hope-xbox-input.service
# 已运行的服务改过地址/路径时，在上述状态下重启这个输入服务：
systemctl --user restart hope-xbox-input.service
systemctl --user status hope-xbox-input.service --no-pager
journalctl --user -u hope-xbox-input.service -n 40 --no-pager
```

不要同时手动另起 `--ros --control`；服务具有单例锁。服务 active 仅证明进程存在。
完整成功还要在 Console 看到：Controller 正确、Input **LIVE**、LT **RELEASED**、Sticks 零值；
Runner 状态新鲜，模式请求的回执与实际状态一致。
没有新鲜 Runner boot 时输入程序不会伪造有效控制包；电脑预览成功也不能替代 Runner/HAL 验证。

## 现场用手柄的顺序

1. 按 [部署现场流程](deploy_and_gate3.md) 启动到 PASSIVE，检查方向、定位和实体急停；先点击 Stand 并等实际 PD_STAND。
2. 接球：选 Normal Play、确认动捕与球包链路、Record ON 后，按 X 或点 RECEIVER → Ready。X 不是只读连接测试，会请求接球策略进入 MOTION。
3. 遥操：Stand 时按 Y / Enter Teleop，等 ACTIVE；摇杆居中，松开 LT 后重新按住 LT，再小幅推杆。仅加载接球模型不够，还需已部署 HumanLike actor、encoder 和 humanlike.yaml。
4. 松开 LT 停止速度输入；如要退出遥操，点 Stand 并等实际 PD_STAND。减速/收脚可能需要时间，不要把 LT 松开当成立即撤销所有控制。
5. 重连手柄或 Runner 重启后，所有模式键先松开再重新按，LT 重新使能；不会自动恢复上次运动。

下面的 A/B/X/Y 是有动作的模式键，普通按键一次只按一个且松开 LT；LB+RB 为急停组合。
不要在没有支撑的站立机器人上用急停作为“手柄是否接通”的测试。软件急停依赖电脑/网络，实体急停保持可达。

## Controls and ownership

One Runner owns every body command, including Stand, Serve, receive and optional
HumanLike locomotion. Foxglove and the controller request state changes; they do
not write motor commands. The gripper bridge runs asynchronously and does not
own body motion.

| 输入 | 操作 |
| --- | --- |
| A | 选择 SERVER，准备 **Start to Serve**，抬起装球手；按实际状态完成 Stand/角色/准备序列 |
| B | 播放选定发球；Kernel 缓慢回 **Stand**，Normal Play 进入 **Ready / MOTION** |
| X | 选择 RECEIVER，按实际状态完成 Stand/角色/**Ready / MOTION** 序列 |
| Y | 只从已经到达的 **Stand** 进入 **Teleop**；其他模式不排队，回 Stand 后重新按 |
| 按住 LT + 左摇杆 | 机器人身体坐标中的前后/横向速度 |
| 按住 LT + 右摇杆 | 转向 |
| 松开 LT / 失去新鲜输入 | 目标速度归零并减速；退出 Teleop 等待收脚稳定，不等于立即 PASSIVE |
| 同时 LB + RB | 触发锁存软件急停；松开按键不会自动复位 |

输入按 50 Hz 采样；Runner 遥操 watchdog 为 200 ms。
当前脚本默认速度上限：前进 0.30 m/s、后退 0.20 m/s、横移 0.15 m/s、转向 0.40 rad/s。
这是输入限幅，不是已经证明安全的实机速度范围，也不是必须推到的测试幅度。

A/B/X/Y do not require LT. After entering Teleop, wait for ACTIVE, center the
sticks, release LT and hold it again. The input latch prevents a held stick from
starting locomotion on entry. Xbox status has a fixed display area and does not
resize the panel whenever an input packet becomes stale.

**Reset Software E-stop** stays visible, including after an Xbox stop. It clears
the software latch; it neither releases a hardware/vendor emergency stop nor
starts motion. Hardware recovery remains an operator action.

## Normal Play and Kernel Mode

| | Normal Play | Kernel Mode |
| --- | --- | --- |
| Connection | Laptop, HDU, MDU; Motive stream for localization | Laptop, HDU, MDU; OptiTrack optional |
| Receive localization | Calibrated, timestamped world pelvis pose | Local frame established from IMU heading; stance-foot odometry |
| Incoming ball | Planner's live trajectory and receive command | No live-ball gate when testing mode entry |
| A/B/X/Y and transition controller | Shared | Shared |
| Interpretation | Full receive workflow with valid field geometry | Local actor/transition testing; local odometry is not a venue calibration |

Kernel Mode permits X to enter the selected receive actor without an OptiTrack
calibration. Serve completion instead lowers the arms slowly and returns to Stand. It must not publish a fabricated
world calibration receipt or make Normal Play consume its local pose as mocap.
Both pelvis and torso IMU acquisition remain part of the hardware state contract.

A receive command requires an incoming-flight packet in Normal Play. Seeing a
ball in Foxglove does not by itself schedule a swing. Kernel Mode has no real
ball trajectory when OptiTrack is absent.

## Serve loop and transitions

Stand/Teleop-to-Serve transitions carry position, velocity, feed-forward torque and
PD gains together. The last delivered command seeds the transition. Loading-arm
motion uses bounded pitch support; entry does not add an operator settlement gate.
In Kernel mode, the selected CSV's validated final stance sets the matching Stand width.

In Kernel mode the sequence is **Start to Serve → Serve → slow arm lowering → Stand**.
Normal Play retains the receive-policy handoff. Repeated A requests prepare the next
serve. Y never queues a Stand-to-Teleop sequence during a serve or another mode.
Reconnecting a controller reopens the device; held face buttons require release
and a new press. Neither reconnecting nor finishing a serve selects locomotion.

The Console enables **Enter Teleop** from fresh Stand and connected, neutral Xbox
input with LT released. Its separate Teleop phase/input-age display can reconnect
independently and does not lock the request button. Runner still validates the
loaded policy, current Stand mode and neutral input within its 200 ms watchdog
when processing every request; rejection is shown in the request receipt.

The default serve is:

```text
a3_deploy/a3_deploy_example/assets/a3_runtime/serve/motions/a3p_op3_serve025_smooth_center_v14.csv
```

This is a named SDK 31-joint CSV with 468 frames at 100 Hz. The default handoff
starts after the final frame (467). Kernel mode uses a 2.5-second full-command return, then measured
settling into Stand. Normal Play uses a one-second return and the existing receive
blend. `--serve-handoff-frame` and `--serve-return-sec` remain explicit options.
`--serve-only` retains complete CSV playback for isolated tests.

**v14 is the attended Kernel-mode serve clip.** Select Kernel before starting it.
Normal Play still requires a CSV whose final pose matches its nominal Stand; the
wide-stance clip fails that preflight. The wide-stance Normal Play/receive loop is
not qualified: a separate simulation stalled during settling, and a trial with a
longer return fell. Those trials are not included in the passing Kernel results.

The v14 clip uses a continuous right-arm spline while preserving the other joints
and release events. Its 70 primary MuJoCo cases (0–80 ms release delay, tested
ball offsets and hold durations) passed; three additional cases at 85 ms failed.
This is not a calibrated hardware hit-rate claim. v13 is rejected due to observed
hardware jitter; historical CSVs remain available for comparison.
The packaged Rockchip profile sends the release command at frame 47; physical jaw
opening and ball detachment are not confirmed by the software publish receipt.

The [gripper URDF](../../a3_deploy/a3_deploy_example/assets/a3_runtime/robots/A3PingPong-with-gripper/README.md)
includes relative mesh paths and documents missing collision-mesh fallbacks.

The field panel can upload a replacement CSV. The MDU validates its complete
schema with the actual Runner before storing it by SHA-256. Load the desired
file while stopped; the next Runner session uses that file. Calibration JSONs
are similarly archived and identified by hash.

## Public package

Use the environment setup in [RUN_ON_AGIBOT](../RUN_ON_AGIBOT.md). Build from
the HOPE deployment directory, not the separate vendor example:

```bash
export HOPE_ROOT="$(git rev-parse --show-toplevel)"
cd "$HOPE_ROOT/a3_deploy/a3_deploy_example"
source /opt/ros/jazzy/setup.bash
bash scripts/build_a3_deploy_pkg.sh --arch x86_64
# MDU package, after provisioning the documented Rockchip sysroot:
bash scripts/build_a3_deploy_pkg.sh --arch rockchip
```

Both commands select `models/model_21800/policy` and the ping-pong runtime YAML
by default. `--policy-dir` is an explicit operator override. Model and parameters
are copied together to `policy/`; packaged paths are relative. Do not substitute
a development checkpoint when building a public release.

The public runtime includes the matching HumanLike inference bundle at
`assets/a3_runtime/teleop_humanlike/`. The builder includes its `humanlike.yaml`,
`policy.onnx` and `lin_vel_encoder.onnx` by default under `teleop_humanlike/`.
The lifecycle discovers this directory and passes it to Runner, so the standard
public package has the files required for **Enter Teleop**. Runtime readiness
and operator input requirements still apply.

Use `--teleop-policy-dir /absolute/path/to/humanlike` to select another compatible
bundle. `--without-teleop` deliberately builds a Stand/Serve/receive package
without learned locomotion. These vendor-origin inference assets are separate
from the unchanged HOPE model_21800 receive policy and private HOPE training.

The gripper bridge includes declarative E-link command presets in
`scripts/serve_gripper_presets.py`; it does not import a private vendor `grip.py`.
The current lifecycle hardware profile is **A3_T3D0** with E-link hand control.
Other A3 variants require their own checked hardware configuration. Missing
gripper transport is reported separately from arm control.

The lifecycle default MDU slot is `/agibot/a3_deploy_model21800`.
`HOPE_MDU_DEPLOY` in the robot account's
`~/.config/hope-foxglove/lifecycle.env` selects another immutable slot. Copy the
complete package and retain the previous slot before switching. This document
and the package builder do not start a robot automatically.

## Laptop and Foxglove update

Follow the SSH keys, HDU services and network setup in
[the first hardware deployment guide](foxglove_first_hardware_test.md). Replace
site addresses and usernames with your own. Keep `HOPE_ROOT` in
`~/.config/hope-foxglove/lifecycle.env`; no workstation home path is required.

Build the public standalone NatNet packages together with the HOPE workspace:

```bash
cd "$HOPE_ROOT/hope_ws"
source /opt/ros/jazzy/setup.bash
colcon build --base-paths src ../NatNet2ROS2/src \
  --packages-up-to motion_capture_tracking hope_bringup hope_planner_cpp \
  --symlink-install --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/local_setup.bash
```

The optional AvatarPro path uses `../VRPN2ROS2/src` as an additional base path.
There is one copy of each driver package; do not copy NatNet packages into
`hope_ws/src`.

Update both helpers on the Laptop and the lifecycle helper on each robot host:

```bash
sudo install -Dm755 "$HOPE_ROOT/foxglove/helpers/hope-lifecycle" \
  /usr/local/libexec/hope-lifecycle
sudo install -Dm755 "$HOPE_ROOT/foxglove/helpers/hope-laptop-container-runtime" \
  /usr/local/libexec/hope-laptop-container-runtime
install -d "$HOME/.local/share/hope-foxglove"
install -m755 "$HOPE_ROOT/foxglove/laptop/hope_marker_monitor.py" \
  "$HOME/.local/share/hope-foxglove/"
install -m644 "$HOPE_ROOT/foxglove/laptop/hope_marker_monitor_core.py" \
  "$HOPE_ROOT/foxglove/laptop/marker_monitor.yaml" \
  "$HOME/.local/share/hope-foxglove/"
```

For the HDU control-plane update, install all `foxglove/a3/hope_*.py` modules
into `/usr/local/bin` together, including field assets, field operations, time
calibration. Runner transport and IMU clock telemetry now use the native C++ binaries
from `foxglove/a3/native`; install them using the native-build commands in
[the Foxglove installation guide](../../foxglove/README.md#per-robot-installation-operator-action-changes-that-robot).
Also install `hope-imu-telemetry.service` alongside the updated monitor unit and
keep both bridge YAMLs at `num_threads: 2`. The observer suppresses unchanged
display-status messages with a bounded heartbeat; Runner liveness, mode and fault
status retain their existing rate. Install `hope_field_assets.py` additionally on the MDU at
`/usr/local/lib/hope-foxglove/hope_field_assets.py`. The Laptop helper reads it
and the calibration validators directly from `HOPE_ROOT`.
Update the fixed command/monitor/lifecycle service files and bridge YAMLs as a
set, then reload systemd while the lifecycle is stopped. The telemetry recorder
also needs `hope-pingpong-telemetry-record` and its matching service file.
Do not mix an old proxy or transport relay with the new console.

Xbox input runs in the Laptop ROS environment:

```bash
python3 -m venv --system-site-packages "$HOME/.local/share/hope-xbox/venv"
"$HOME/.local/share/hope-xbox/venv/bin/pip" install evdev
export HOPE_XBOX_HDU_IP='<hdu-ip>'
bash "$HOPE_ROOT/foxglove/laptop/run_xbox_input.sh"
```

For automatic startup, install `foxglove/laptop/hope-xbox-input.service` as a user
service. It reads the same `lifecycle.env`; set `HOPE_XBOX_HDU_IP` in a service
drop-in. The gamepad must be visible inside the `hope` container. `LT`, axes and
A/B/X/Y are read locally, so a Foxglove browser gamepad preview is not required.

Install the ready-made console on the Laptop:

1. Download [hopeopen.hope-a3-console-1.8.14.foxe](../../foxglove/extensions/hope-a3-console/hopeopen.hope-a3-console-1.8.14.foxe)
   from this checkout. On GitHub, use the file's download button.
2. Open the `.foxe` in Foxglove Desktop's Extensions screen to install it.
3. Import `foxglove/layouts/model21800_console.json` and connect to the HDU control
   bridge on port 8766. Confirm the field addresses, mode and P1/P2 selection.

No Node.js build is required to install the committed `.foxe`. To rebuild it:

```bash
cd "$HOPE_ROOT/foxglove/extensions/hope-a3-console"
npm ci
npm run package
```

When publishing a rebuilt console, commit its installer with the matching source
and update download links and the exact `.foxe` filename allowed by the extension's
`.gitignore`. Keep `node_modules/`, `dist/` and superseded installers out of Git.

## Calibration and clocks

The supplied shell has **17 retained stations** and the imported body is named
`UCB_P1` (or `UCB_P2` for the other table side). The default `motive_live` relay
accepts that body's current ModelDef without binding to a file hash or fixed
marker layout. It assumes the imported pivot and axes already describe the
pelvis. The console shows live asset acceptance and counts physical markers
against the current ModelDef size. The imported geometry and the venue's
Motive-to-world transform must use the same frame convention.

Legacy measured calibration remains an explicit launch option, rather than a
startup requirement. Existing V2/V3 or old sticker receipts must not be reused
for a different body/layout. See the [Motive setup](../../agibot/mocap_sticker_shell_v3/README.md).

The standalone NatNet adapter keeps its public Ball/P1/P2 output defaults and
200 Hz output limit. HOPE bringup explicitly aliases `P1`/`P2` to
`UCB_P1`/`UCB_P2` and `ball` to `Ball`, retaining exact canonical names when both
exist. Set `HOPE_MOTIVE_INTERFACE_IP` in Laptop `lifecycle.env` to the wired NIC
address. NatNet's transport is independent of Fast DDS transport.

P2 applies the table-side transform consistently to the ball, robot and saved
calibration. Saved JSON must identify the selected side and rigid body. Changing
field mode or table side is a stopped-session operation.

Keep main's [supervised clock package](../../agibot/ntp_sync/README.md).
HDU Chrony and the dedicated HDU/MDU PTP services retain clock ownership.
**TIME CALIBRATION** is an explicit stopped-session operation: it coordinates
NTP/MDU synchronization and restores clock services. Starting a normal session
does not silently run time calibration. Clock diagnostics remain visible.

## Reproducible transition checks

The native simulation probe links the production controllers without HAL or DDS:

```bash
cd "$HOPE_ROOT/a3_deploy/a3_deploy_example"
bash scripts/build_runner_transition_probe.sh /tmp/hope-transition.so
python scripts/validate_serve_loop_mujoco.py \
  --library /tmp/hope-transition.so \
  --policy-dir models/model_21800/policy \
  --output /tmp/model21800-serve-loop.json
```

Use a Python environment with MuJoCo and NumPy. The probe includes both IMUs,
actual command-delivery feedback and the production frame-110/one-second return
settings. It checks command continuity and floating-base stability separately;
it sends no actuator commands and does not qualify real-ball performance.
See [public runtime validation](runtime_public_validation.md) for the results
and outstanding qualification limits of this update.

## Supplied AimDK reference bundle

The A3 Ultra AimDK 3.1 protocols, examples, wheel and AArch64 protocol libraries
are available in [the consolidated SDK directory](../../agi/agibot_a3_Ultra_aimdk-dev3.1/HOPE_INTEGRATION.md).
Main's Runner and declarative gripper presets retain their existing build paths.
