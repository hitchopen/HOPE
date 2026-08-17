# Foxglove Runner 首次部署与现场测试手册

本文用于把当前 HOPE checkout 部署到一台 Laptop、一个 HDU 和一个 MDU，
并完成第一次有人值守测试。完成“一次性部署”后，正常
session 不再手工执行旧 runbook 的 STEP 0、STEP 1、STEP 2A/2B、STEP 4
和 STEP 5；这些步骤由 Foxglove 的 `START SYSTEM` 固定执行。

Foxglove 不是远程终端。它只能确认四个 IP、启动/停止固定流程，以及调用
明确列出的 Runner/Planner 服务。Runner 仍然是 PASSIVE、PD_STAND、MOTION、
角色和 Serve 状态的唯一权威。

如果现场已经完成过本手册的旧版 Foxglove/lifecycle 部署，现在只需要增加
`TIME CALIBRATION`，不要重复构建 Planner、Rockchip Runner 或重新上传 policy。
增量升级路径是：第 0 节设置变量 → 第 1 节构建当前 `.foxe` → 第 9.2 节安装 UI
→ 第 9.2.1 节部署 MDU/HDU 增量后端 → 第 9.3 节重新连接 → 第 10 节 preflight。
全新机器人或尚未部署 8766/lifecycle 的现场仍按第 0–9 节首次部署路径执行，并按
9.2.1 的说明跳过增量重复安装。

## 0. 设置现场参数并确认执行环境

先在 **Laptop HOST** 的仓库根目录执行下面的 block。把四个尖括号占位符换成
现场真实地址；本文后续 Laptop 命令均复用这些变量：

```bash
cd "$(git rev-parse --show-toplevel)"
export HOPE_ROOT="$PWD"
export LAPTOP_USER="${USER}"
export ROBOT_USER="${ROBOT_USER:-agi}"
export LAPTOP_IP=<laptop-wifi-ip>
export HDU_IP=<hdu-wifi-ip>
export MDU_IP=<mdu-internal-ip>
export MOTIVE_IP=<motive-ip>
export ROS_DOMAIN_ID=232
export DOWNLOAD_DIR="${XDG_DOWNLOAD_DIR:-$HOME/Downloads}"

[[ "$LAPTOP_USER" =~ ^[a-z_][a-z0-9_-]{0,31}$ ]]
[[ "$ROBOT_USER" =~ ^[a-z_][a-z0-9_-]{0,31}$ ]]

printf 'HOPE_ROOT=%s\nLAPTOP_USER=%s\nROBOT_USER=%s\n' \
  "$HOPE_ROOT" "$LAPTOP_USER" "$ROBOT_USER"
printf 'Laptop=%s HDU=%s MDU=%s Motive=%s\n' \
  "$LAPTOP_IP" "$HDU_IP" "$MDU_IP" "$MOTIVE_IP"
```

不要把真实地址、密码或私钥提交到 Git。重新打开终端后，需要重新执行这个
参数 block。

本文有四种执行环境。不要把命令贴错机器：

| 标记 | 如何进入 | 是否使用 distrobox |
| --- | --- | --- |
| `Laptop HOST` | Laptop 当前操作员的普通终端 | 否 |
| `Laptop hope Distrobox` | 从 Laptop 执行 `distrobox enter hope` | 是 |
| `HDU` | `ssh -tt "${ROBOT_USER}@${HDU_IP}"` | 否，使用 HDU 原生 Jazzy |
| `MDU` | `ssh -tt -J "${ROBOT_USER}@${HDU_IP}" "${ROBOT_USER}@${MDU_IP}"` | 否，使用 MDU vendor 环境 |

需要确认的连接参数：

```text
Laptop Wi-Fi IP   $LAPTOP_IP
HDU Wi-Fi IP      $HDU_IP
MDU internal IP   $MDU_IP
Motive IP         $MOTIVE_IP
ROS_DOMAIN_ID     232
Foxglove control  ws://$HDU_IP:8766
```

Laptop 正式运行目录：

```text
$HOPE_ROOT
    Foxglove、Runner、Laptop OptiTrack workspace、Calibration JSON、
    real_logs、部署文件和本文全部从这里取。不要混用其他旧工作区。
```

本文中的默认 IP 可以改。部署时 SSH 命令必须使用当时真实地址；每次运行时
四个地址在 Foxglove 输入框中填写并确认。修改 HDU IP 后，还必须把 Foxglove
连接重新打开到新的 `ws://<HDU-IP>:8766`。

> **Laptop 环境边界：** 这台 Laptop HOST 没有 `/opt/ros/jazzy`，也没有
> `ros2` 命令。凡是本文要求在 Laptop 上执行 `source /opt/ros/jazzy/setup.bash`
> 或 `ros2 ...` 的地方，都必须先进入 `hope` distrobox。HDU 上的 ROS 命令则在
> HDU 原生 shell 执行，不进 distrobox。

> 安全要求：第一次必须有人扶机器人，实体急停必须随时可触达。不要用
> `E-STOP`、`Runner Passive` 或 `START SYSTEM` 做普通安装 smoke test。
> `Runner Passive` 等价于原来的 `p`，机器人会失去主动支撑。

---

## 1. Laptop HOST：核对当前分支并构建 UI

以下完整 block 在 **Laptop HOST，不进 distrobox** 执行：

```bash
cd "$HOPE_ROOT"

git branch --show-current
git status --short

test -f foxglove/extensions/hope-a3-console/package.json
test -f foxglove/layouts/model21800_console.json
```

部署前记录当前 commit。`git status` 非空时先确认改动来源；不要为了测试执行
`git reset` 或盲目切换分支。

当前 Laptop Host 不要求预装 Node.js。第一次使用或 UI 源码改变后，
用隔离的 Node 22 容器构建当前扩展：

```bash
cd "$HOPE_ROOT/foxglove/extensions/hope-a3-console"

if ! command -v podman >/dev/null; then
  sudo apt-get update
  sudo apt-get install -y podman
fi

podman run --rm --userns keep-id \
  -v "$PWD:/workspace" \
  -w /workspace \
  docker.io/library/node:22-bookworm-slim \
  bash -lc 'npm ci && npm run lint && npx tsc --noEmit && npm run package'

UI_VERSION="$(python3 -c 'import json; print(json.load(open("package.json"))["version"])')"
FOXE="$PWD/hopeopen.hope-a3-console-$UI_VERSION.foxe"
test -f "$FOXE"
ls -lh "$FOXE"
```

首次会拉取 Node 22 容器镜像。`.foxe` 是本地生成物，不随源码提交；安装时只使用
上面 `$FOXE` 指向的当前版本。当前面板包含独立 Calibration/Refresh x_hit、
可重复 assert 的 E-stop、不依赖 Runner 模式的 `KILL ALL & COLLECT`，以及仅在
NTP gate 不合格且 lifecycle 已停止时启用的 `TIME CALIBRATION`。

---

## 2. Laptop HOST：准备 SSH、tmux、rsync 和 hope Distrobox

### 2.1 安装/启用本机基础工具

新机器如果还没有 Distrobox，先完成
[`docs/DISTROBOX_SETUP.md`](../DISTROBOX_SETUP.md) 中的 HOST 安装，并按其
`hope` 小节完成 ROS 2 Jazzy 容器。不要只创建一个没有 ROS 的空容器。

在 **Laptop HOST** 执行：

```bash
cd "$HOPE_ROOT"

sudo apt-get update
sudo apt-get install -y \
  openssh-client openssh-server rsync tmux curl netcat-openbsd podman
sudo systemctl enable --now ssh.service

if ! command -v distrobox >/dev/null 2>&1; then
  if ! sudo apt-get install -y distrobox; then
    curl -s \
      https://raw.githubusercontent.com/89luca89/distrobox/main/install |
      sh -s -- --prefix "$HOME/.local"
    grep -qxF 'export PATH="$HOME/.local/bin:$PATH"' "$HOME/.bashrc" ||
      echo 'export PATH="$HOME/.local/bin:$PATH"' >> "$HOME/.bashrc"
    export PATH="$HOME/.local/bin:$PATH"
  fi
fi

command -v distrobox >/dev/null
distrobox --version
distrobox list
```

确认列表中存在名为 `hope` 的容器；如果不存在，停止本 runbook，先完成上面链接的
`hope` 一次性环境构建。

### 2.2 在 hope Distrobox 中构建 Laptop OptiTrack workspace

下面仍从 **Laptop HOST** 直接整段执行；`distrobox enter ... -- bash -lc`
后面的内容是在 `hope` 容器里执行，不要再手工进入第二次：

```bash
distrobox enter hope -- bash -lc '
  set -eo pipefail
  source /opt/ros/jazzy/setup.bash
  cd "$HOPE_ROOT/hope_ws"
  /usr/bin/colcon build \
    --base-paths src ../NatNet2ROS2/src \
    --symlink-install \
    --packages-up-to hope_bringup hope_planner_cpp \
    --cmake-clean-cache \
    --cmake-args \
      -DBUILD_TESTING=OFF \
      -DPython3_EXECUTABLE=/usr/bin/python3
  source install/local_setup.bash
  set -u
  test -x install/hope_bringup/lib/hope_bringup/with_fastdds_unicast.sh
  test -x install/hope_bringup/lib/hope_bringup/p1_marker_cad_calibration_server
  test -x install/motion_capture_tracking/lib/motion_capture_tracking/motion_capture_tracking_node
  test -x install/hope_planner_cpp/lib/hope_planner_cpp/hope_ball_flight_packetizer
  ros2 interface show hope_msgs/msg/BallFlightPacket >/dev/null
  ros2 pkg prefix motion_capture_tracking_interfaces
'
```

以后没有修改 `HOPE_OPEN/hope_ws` 的 OptiTrack/bringup/flight-packet 源码时，
不需要每次重编。Foxglove 启动链额外运行 Laptop 数据适配器：它从原生 `/poses`
生成 `/ball/flight_packet`，但不修改后续已检入的 estimator、bounce、target 或
schema-2 算法。

### 2.3 安装 Laptop lifecycle helper 和 marker 文件

在 **Laptop HOST** 执行：

```bash
cd "$HOPE_ROOT"

sudo install -D -o root -g root -m 0755 \
  foxglove/helpers/hope-lifecycle \
  /usr/local/libexec/hope-lifecycle

install -d -m 0700 "$HOME/.config/hope-foxglove"
printf 'HOPE_ROOT=%q\nHOPE_LAPTOP_USER=%q\nHOPE_ROBOT_USER=%q\n' \
  "$HOPE_ROOT" "$LAPTOP_USER" "$ROBOT_USER" \
  > "$HOME/.config/hope-foxglove/lifecycle.env"
chmod 0600 "$HOME/.config/hope-foxglove/lifecycle.env"

install -D -m 0755 \
  foxglove/laptop/hope_marker_monitor.py \
  "$HOME/.local/share/hope-foxglove/hope_marker_monitor.py"
install -D -m 0644 \
  foxglove/laptop/hope_marker_monitor_core.py \
  "$HOME/.local/share/hope-foxglove/hope_marker_monitor_core.py"
install -D -m 0644 \
  foxglove/laptop/marker_monitor.yaml \
  "$HOME/.local/share/hope-foxglove/marker_monitor.yaml"

test -x /usr/local/libexec/hope-lifecycle
test -r "$HOME/.local/share/hope-foxglove/hope_marker_monitor.py"
```

Marker publisher 不在 Laptop Host 直接 source ROS。它由 lifecycle 的
`OPTITRACK` 步骤和 bridge 一起进入 `hope` distrobox，并使用 UI 确认的 HDU IP。
因此正常运行时不需要另开 marker 终端。

### 2.4 安装本地球桌 asset 服务

正式 Layout 只保留一个 `foxglove.Urdf` layer，用于静态标准球桌；它不读取机器人
URDF。8765/8766 仍不转发原始 `/tf` 或 `/tf_static`，`hope_monitor` 只向
`/hope/pelvis/tf` 发布一条经过净化的 `world -> pelvis_link`，因此 Foxglove 中会有
球桌和 Pelvis，但不会重新出现机器人的其他 link。

在 **Laptop HOST** 执行：

```bash
cd "$HOPE_ROOT"

install -d -m 0755 "$HOME/.local/share/hope-foxglove"
cp -a foxglove/assets "$HOME/.local/share/hope-foxglove/"
install -D -m 0644 \
  foxglove/laptop/hope-foxglove-assets.service \
  "$HOME/.config/systemd/user/hope-foxglove-assets.service"

systemctl --user daemon-reload
systemctl --user enable --now hope-foxglove-assets.service

systemctl --user is-active hope-foxglove-assets.service
curl -fsS \
  http://127.0.0.1:8000/assets/hope_ping_pong_table.urdf \
  >/dev/null
```

---

## 3. 一次性配置四个方向的 SSH key

后台一键启动使用 `BatchMode=yes`，不能输入密码。只在本节的
`ssh-copy-id` 提示中通过安全渠道输入现场密码；不要把密码写进仓库、脚本、
Foxglove、ROS 消息或命令历史。HDU 到 Laptop 的 key 由 Laptop 本地安装，
不需要、也不应猜测 Laptop 登录密码。

### 3.1 Laptop 到 HDU/MDU

在 **Laptop HOST** 执行：

```bash
install -d -m 0700 "$HOME/.ssh"

if [[ ! -f "$HOME/.ssh/id_ed25519" ]]; then
  ssh-keygen -t ed25519 -N '' -f "$HOME/.ssh/id_ed25519"
fi

ssh-copy-id "${ROBOT_USER}@${HDU_IP}"
ssh-copy-id -o "ProxyJump=${ROBOT_USER}@${HDU_IP}" "${ROBOT_USER}@${MDU_IP}"

ssh -o BatchMode=yes "${ROBOT_USER}@${HDU_IP}" true
ssh -o BatchMode=yes -J "${ROBOT_USER}@${HDU_IP}" "${ROBOT_USER}@${MDU_IP}" true
```

第一次连接如询问 host key，先核对目标地址，然后输入 `yes`。两个最后的验证命令
必须直接返回，不能再出现 password 提示。

### 3.2 HDU 到 Laptop/MDU

先在 **Laptop HOST** 登录 HDU：

```bash
ssh -tt "${ROBOT_USER}@${HDU_IP}"
```

看到 HDU 的 `agi@...` 提示符后，在 **HDU 原生 shell，不进 distrobox** 执行：

```bash
install -d -m 0700 "$HOME/.ssh"

if [[ ! -f "$HOME/.ssh/id_ed25519" ]]; then
  ssh-keygen -t ed25519 -N '' -f "$HOME/.ssh/id_ed25519"
fi

ssh-copy-id <robot-user>@<mdu-internal-ip>

ssh -o BatchMode=yes <robot-user>@<mdu-internal-ip> true

exit
```

回到 **Laptop HOST** 后，从 HDU 读取公钥并本地写入
`$HOME/.ssh/authorized_keys`。这段命令是幂等的，重复执行不会
追加重复 key：

```bash
HDU_PUBLIC_KEY="$(
  ssh -o BatchMode=yes "${ROBOT_USER}@${HDU_IP}" \
    'cat "$HOME/.ssh/id_ed25519.pub"'
)"

case "$HDU_PUBLIC_KEY" in
  ssh-ed25519\ *)
    ;;
  *)
    echo "ERROR: HDU returned an invalid public key" >&2
    unset HDU_PUBLIC_KEY
    exit 1
    ;;
esac

install -d -m 0700 "$HOME/.ssh"
touch "$HOME/.ssh/authorized_keys"
chmod 0600 "$HOME/.ssh/authorized_keys"

if ! grep -qxF "$HDU_PUBLIC_KEY" "$HOME/.ssh/authorized_keys"; then
  printf '%s\n' "$HDU_PUBLIC_KEY" \
    >> "$HOME/.ssh/authorized_keys"
fi

unset HDU_PUBLIC_KEY
```

仍在 **Laptop HOST**，做四方向最终验证：

```bash
ssh -o BatchMode=yes "${ROBOT_USER}@${HDU_IP}" true
ssh -o BatchMode=yes -J "${ROBOT_USER}@${HDU_IP}" "${ROBOT_USER}@${MDU_IP}" true
ssh "${ROBOT_USER}@${HDU_IP}" \
  ssh -o BatchMode=yes "${LAPTOP_USER}@${LAPTOP_IP}" true &&
  echo HDU_TO_LAPTOP_OK
ssh "${ROBOT_USER}@${HDU_IP}" \
  ssh -o BatchMode=yes "${ROBOT_USER}@${MDU_IP}" true &&
  echo HDU_TO_MDU_OK

echo "four SSH directions: OK"
```

预期明确看到 `HDU_TO_LAPTOP_OK`、`HDU_TO_MDU_OK` 和
`four SSH directions: OK`。MDU 如果曾显示 `All keys were skipped because they
already exist`，表示该 key 已存在，不要使用 `ssh-copy-id -f`。

---

## 4. Laptop HOST：构建当前 Rockchip Runner package

本节在 **Laptop HOST，不进 distrobox** 执行。构建脚本使用 Docker 生成
Rockchip/AArch64 package；不要在 HDU 或 MDU 编译。

```bash
cd $HOPE_ROOT/agibot/code_deployment/a3_deploy_example

BUILD_LOG=/tmp/hope_open_model21800_rockchip_build.log
VENDOR_PAYLOAD=/absolute/path/to/licensed/a3_deploy_example

test -f "$VENDOR_PAYLOAD/thirdparty/rockchip_sysroot/rockchip-1.0-aarch64-sysroot.tar.gz"

A3_VENDOR_PAYLOAD_ROOT="$VENDOR_PAYLOAD" \
  bash scripts/build_a3_deploy_pkg.sh \
  --arch rockchip \
  --policy-dir "$HOPE_ROOT/a3_deploy/a3_deploy_example/models/model_21800/policy" \
  --jobs 4 \
  2>&1 | tee "$BUILD_LOG"

BUILD_RC=${PIPESTATUS[0]}
printf 'ROCKCHIP_BUILD_EXIT_CODE=%s\n' "$BUILD_RC"
test "$BUILD_RC" -eq 0
```

构建成功后继续在 **同一个 Laptop HOST 终端** 核对：

```bash
cd $HOPE_ROOT/agibot/code_deployment/a3_deploy_example

DIST=$HOPE_ROOT/agibot/code_deployment/a3_deploy_example/dist/a3_deploy_rockchip
HASH_FILE=/tmp/hope_open_model21800_candidate_sha256.txt

sha256sum \
  "$DIST/a3_deploy_onnx_ref_pingpong" \
  "$DIST/run_a3_pingpong.sh" \
  "$DIST/config/a3_aimrt_config.pingpong_ros2body.yaml" \
  "$DIST/config/a3_runtime_config.pingpong.hitter_pingpong.yaml" \
  "$DIST/policy/exported/policy.onnx" \
  "$DIST/policy/params/deploy.yaml" \
  | tee "$HASH_FILE"

file "$DIST/a3_deploy_onnx_ref_pingpong"
strings "$DIST/a3_deploy_onnx_ref_pingpong" | \
  grep -E '/hope/runner/(control_request_flat|state_flat)|SET_SERVER|SET_RECEIVER'

echo "candidate hashes: $HASH_FILE"
```

这里 Runner/接口源码来自当前 checkout 的 `a3_deploy`。受许可证约束的 vendor
runtime payload 和 Rockchip sysroot 不在公开仓库中；`VENDOR_PAYLOAD` 必须指向
操作者合法取得的本地 payload，不能把它提交或上传到 PR。

必须能看到两个 `/hope/runner/*_flat` topic 和
`SET_SERVER/SET_RECEIVER`。不要把 ping-pong 的精简 runtime YAML 传给
`--runtime-cfg`：该参数属于 package 中的通用 A3 runner；ping-pong runner 会由打包
脚本单独 stage `a3_runtime_config.pingpong.hitter_pingpong.yaml`。
`a3_aimrt_config.pingpong_ros2body.yaml` 也必须存在：它让 body-drive 保持在 MDU
本机 iceoryx，同时只把 `/hope/runner/state_flat` 和
`/hope/runner/control_request_flat` 接到
ROS 2。缺少或部署旧版该文件时，Runner
虽然会启动，但 Foxglove 会一直显示 `NO FRESH RUNNER STATE`，所有 Runner 按钮和
Runner 动作按钮都会被权威状态检查锁住；`KILL ALL & COLLECT` 不依赖这条状态，
仍可清理 lifecycle 已经创建的固定 session。
当前 policy metadata 仍为 `rally_v14`，所以角色切换可以测试，但
`Ready to Serve`/`Serve` 仍会由 Runner 报告为不可用。

---

## 5. Laptop HOST：把 Foxglove 文件 stage 到 HDU

在 **Laptop HOST** 执行：

```bash
cd $HOPE_ROOT

ssh "${ROBOT_USER}@${HDU_IP}" \
  'mkdir -p "$HOME/foxglove_a3"'

rsync -azP \
  --exclude '__pycache__/' \
  --exclude '*.pyc' \
  foxglove/a3/ \
  "${ROBOT_USER}@${HDU_IP}:~/foxglove_a3/"

scp foxglove/helpers/hope-lifecycle \
  "${ROBOT_USER}@${HDU_IP}:/tmp/hope-lifecycle"

sed \
  -e "s/^HOPE_LAPTOP_USER=.*/HOPE_LAPTOP_USER=$LAPTOP_USER/" \
  -e "s/^HOPE_ROBOT_USER=.*/HOPE_ROBOT_USER=$ROBOT_USER/" \
  foxglove/a3/network.env.example \
  > /tmp/hope-foxglove-network.env

scp /tmp/hope-foxglove-network.env \
  "${ROBOT_USER}@${HDU_IP}:/tmp/hope-foxglove-network.env"
```

同样在 **Laptop HOST，不进 distrobox**，把当前硬件运行 workspace 的源码
同步到 HDU。这里不上传、不删除 HDU 的 build/install/log：

```bash
cd "$HOPE_ROOT"

rsync -azP \
  --exclude 'build*' \
  --exclude 'install*' \
  --exclude 'log*' \
  --exclude '__pycache__/' \
  --exclude '*.pyc' \
  --exclude '.pytest_cache/' \
  hope_ws/ \
  "${ROBOT_USER}@${HDU_IP}:~/hope_ws/"
```

---

## 6. HDU：构建 bridge 并安装七个常驻服务

### 6.1 进入 HDU

在 **Laptop HOST** 执行：

```bash
ssh -tt "${ROBOT_USER}@${HDU_IP}"
```

以下小节全部在新的 **HDU 原生 shell，不进 distrobox** 执行。

### 6.2 HDU 原生 Jazzy：构建 `hope_msgs` + C++ Planner overlay

在 **HDU** 执行；不要进入 distrobox，也不要全量删除原来的 install：

```bash
(
set -eo pipefail

source /opt/ros/jazzy/setup.bash
cd $HOME/hope_ws
test -f $HOME/hope_ws/src/hope_msgs/msg/BallFlightPacket.msg

if [[ -f $HOME/hope_ws/install/local_setup.bash ]]; then
  source $HOME/hope_ws/install/local_setup.bash
fi

colcon --log-base log_model21800_fix build \
  --build-base build_model21800_fix \
  --install-base install_model21800_fix \
  --symlink-install \
  --packages-up-to hope_planner_cpp \
  --cmake-clean-cache \
  --cmake-args \
    -DBUILD_TESTING=OFF \
    -DCMAKE_BUILD_TYPE=Release \
    -DPython3_EXECUTABLE=/usr/bin/python3

source /opt/ros/jazzy/setup.bash
source $HOME/hope_ws/install/local_setup.bash
source $HOME/hope_ws/install_model21800_fix/local_setup.bash

test "$(ros2 pkg prefix hope_msgs)" = \
  $HOME/hope_ws/install_model21800_fix/hope_msgs
ros2 interface show hope_msgs/msg/BallFlightPacket >/dev/null

ros2 pkg prefix hope_planner_cpp
test "$(ros2 pkg prefix hope_planner_cpp)" = \
  $HOME/hope_ws/install_model21800_fix/hope_planner_cpp
test -x \
  $HOME/hope_ws/install_model21800_fix/hope_planner_cpp/lib/hope_planner_cpp/hope_planner_cpp_node
test -x \
  $HOME/hope_ws/install_model21800_fix/hope_planner_cpp/lib/hope_planner_cpp/hope_ball_flight_packetizer
test -x $HOME/hope_ws/src/hope_bringup/scripts/with_fastdds_unicast.sh
echo "HDU_PLANNER_OVERLAY_OK"
)
```

`hope_msgs` 和 `hope_planner_cpp` 的 prefix 都必须来自
`install_model21800_fix`，并且 `BallFlightPacket` interface 与 Planner executable
都必须存在。核心 Planner 行为由当前 checkout 中已审阅并检入的实现决定；本集成
只负责把既有 Planner 接到 Foxglove 启动链，不从私有仓库动态下载代码。
不要只看 `ros2 pkg prefix hope_planner_cpp` 的输出：构建失败时，旧 overlay
的残留目录仍可能返回这个 prefix。以后 Planner 和 `hope_msgs` 源码都没有
变化时，可以跳过同步和本小节构建。

### 6.3 首次缺少 foxglove_bridge 时构建

```bash
cd $HOME/foxglove_a3

if ! command -v tmux >/dev/null || ! command -v rsync >/dev/null; then
  sudo apt-get update
  sudo apt-get install -y tmux rsync
fi

if [[ ! -x $HOME/hope_foxglove_ws/foxglove-sdk/ros/install/foxglove_bridge/lib/foxglove_bridge/foxglove_bridge ]]; then
  sudo apt-get update
  sudo apt-get install --no-install-recommends -y \
    git ca-certificates libssl-dev zlib1g-dev rapidjson-dev tmux rsync
  bash $HOME/foxglove_a3/build_foxglove_bridge.sh
else
  echo "foxglove_bridge already built"
fi

source /opt/ros/jazzy/setup.bash
source $HOME/hope_foxglove_ws/foxglove-sdk/ros/install/setup.bash
ros2 pkg prefix foxglove_bridge
ros2 pkg executables foxglove_bridge
```

### 6.4 安装 fleet monitor、Runner control plane 和 lifecycle supervisor

仍在 **HDU** 执行整段：

```bash
cd $HOME

sudo install -D -o root -g root -m 0644 \
  $HOME/foxglove_a3/bridge_params.yaml \
  /etc/hope-foxglove/bridge_params.yaml
sudo install -D -o root -g root -m 0644 \
  $HOME/foxglove_a3/fastdds_bridge_profile.xml \
  /etc/hope-foxglove/fastdds_bridge_profile.xml
sudo install -D -o root -g root -m 0644 \
  /tmp/hope-foxglove-network.env \
  /etc/hope-foxglove/network.env

sudo install -D -o root -g root -m 0755 \
  $HOME/foxglove_a3/hope_monitor.py \
  /usr/local/bin/hope_monitor.py
sudo install -D -o root -g root -m 0644 \
  $HOME/foxglove_a3/hope_monitor_core.py \
  /usr/local/lib/hope-foxglove/hope_monitor_core.py
sudo install -D -o root -g root -m 0644 \
  $HOME/foxglove_a3/hope-monitor.service \
  /etc/systemd/system/hope-monitor.service
sudo install -D -o root -g root -m 0644 \
  $HOME/foxglove_a3/hope-foxglove-bridge.service \
  /etc/systemd/system/hope-foxglove-bridge.service

sudo install -D -o root -g root -m 0755 \
  $HOME/foxglove_a3/hope_observer.py \
  /usr/local/bin/hope_observer.py
sudo install -D -o root -g root -m 0755 \
  $HOME/foxglove_a3/hope_command_proxy.py \
  /usr/local/bin/hope_command_proxy.py
sudo install -D -o root -g root -m 0755 \
  $HOME/foxglove_a3/hope_lifecycle_supervisor.py \
  /usr/local/bin/hope_lifecycle_supervisor.py
sudo install -D -o root -g root -m 0755 \
  $HOME/foxglove_a3/hope_time_calibration.py \
  /usr/local/bin/hope_time_calibration.py

sudo install -D -o root -g root -m 0644 \
  $HOME/foxglove_a3/hope_observer_core.py \
  /usr/local/lib/hope-foxglove/hope_observer_core.py
sudo install -D -o root -g root -m 0644 \
  $HOME/foxglove_a3/hope_command_core.py \
  /usr/local/lib/hope-foxglove/hope_command_core.py
sudo install -D -o root -g root -m 0644 \
  $HOME/foxglove_a3/hope_runner_control_core.py \
  /usr/local/lib/hope-foxglove/hope_runner_control_core.py
sudo install -D -o root -g root -m 0644 \
  $HOME/foxglove_a3/hope_lifecycle_core.py \
  /usr/local/lib/hope-foxglove/hope_lifecycle_core.py
sudo install -D -o root -g root -m 0644 \
  $HOME/foxglove_a3/hope_time_calibration_core.py \
  /usr/local/lib/hope-foxglove/hope_time_calibration_core.py

sudo install -D -o root -g root -m 0644 \
  $HOME/foxglove_a3/bridge_params_control.yaml \
  /etc/hope-foxglove/control_bridge_params.yaml
sudo install -D -o root -g root -m 0644 \
  $HOME/foxglove_a3/hope-observer.service \
  /etc/systemd/system/hope-observer.service
sudo install -D -o root -g root -m 0644 \
  $HOME/foxglove_a3/hope-command-proxy.service \
  /etc/systemd/system/hope-command-proxy.service
sudo install -D -o root -g root -m 0644 \
  $HOME/foxglove_a3/hope-foxglove-control-bridge.service \
  /etc/systemd/system/hope-foxglove-control-bridge.service
sudo install -D -o root -g root -m 0644 \
  $HOME/foxglove_a3/hope-lifecycle-supervisor.service \
  /etc/systemd/system/hope-lifecycle-supervisor.service
sudo install -D -o root -g root -m 0644 \
  $HOME/foxglove_a3/hope-time-calibration.service \
  /etc/systemd/system/hope-time-calibration.service

sudo install -D -o root -g root -m 0755 \
  /tmp/hope-lifecycle \
  /usr/local/libexec/hope-lifecycle
sudo install -D -o root -g root -m 0755 \
  $HOME/foxglove_a3/hope_base_pose_transport_relay.py \
  /usr/local/libexec/hope-base-pose-transport-relay

sudo systemctl daemon-reload
sudo systemctl enable --now \
  hope-monitor.service \
  hope-foxglove-bridge.service \
  hope-observer.service \
  hope-command-proxy.service \
  hope-foxglove-control-bridge.service \
  hope-lifecycle-supervisor.service \
  hope-time-calibration.service
```

`/etc/hope-foxglove/network.env` 默认不写死旧 Laptop 地址。Lifecycle 启动的
Laptop bridge、marker publisher、Laptop base relay、HDU base transport relay 和
HDU Planner 会使用 UI 已确认的地址构造固定 Fast DDS peer 列表。Laptop 发布
`/a3/base_pose_laptop_flat`；双网口 HDU 只改 topic 名并逐包转发到
`/a3/base_pose_flat`，让 MDU Runner 收到既有 schema-2 payload。
`with_fastdds_unicast.sh` 会把这些 peer 同时写入 Fast DDS XML 的
`initialPeersList`；只设置 `ROS_STATIC_PEERS` 在 HDU/MDU 这套 Fast DDS 上不足以
完成跨网卡发现。不要换回没有 `initialPeersList` 的旧 wrapper，否则 Ready 以后
Runner 会再次出现 `NO FRESH authoritative mocap base pose`。
BASE_RELAY 启动阶段只确认受管 relay 进程已存活，不等待 ROS topic、Calibration
service、marker 或第一帧 Pelvis 数据，因此这些数据不会把系统启动链卡成 `FAILED`。
实时 Pelvis 是否新鲜只在面板中显示，并只约束 Foxglove 的 Ready 按钮。

### 6.5 HDU 只读验证

仍在 **HDU** 执行；本节不会启动 HAL、Planner 或 Runner：

```bash
systemctl is-enabled \
  hope-monitor.service \
  hope-foxglove-bridge.service \
  hope-observer.service \
  hope-command-proxy.service \
  hope-foxglove-control-bridge.service \
  hope-lifecycle-supervisor.service \
  hope-time-calibration.service

systemctl is-active \
  hope-monitor.service \
  hope-foxglove-bridge.service \
  hope-observer.service \
  hope-command-proxy.service \
  hope-foxglove-control-bridge.service \
  hope-lifecycle-supervisor.service \
  hope-time-calibration.service

ss -lnt | grep -E ':(8765|8766)[[:space:]]'

source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=232
export ROS_LOCALHOST_ONLY=0

ros2 topic echo /hope/lifecycle/summary --once
ros2 service type /hope/lifecycle/apply_config
ros2 service type /hope/lifecycle/start
ros2 service type /hope/lifecycle/kill_all_and_collect
ros2 service type /hope/lifecycle/time_calibration
ros2 service type /hope/calibrate
ros2 service type /hope/refresh_x_hit
test -x /usr/local/libexec/hope-base-pose-transport-relay
timeout 10s ros2 topic echo /hope/system/cpu_top_process --once
timeout 10s ros2 topic echo /hope/safety/estop_ready --once
timeout 10s ros2 topic echo /hope/safety/estop_full_ready --once
timeout 10s ros2 topic echo /hope/safety/estop_text --once
timeout 10s ros2 topic echo /hope/safety/estop_latched --once

exit
```

预期 lifecycle summary 在第一次确认 UI 配置前显示 `STOPPED` 和
`config_revision=0`。不要从命令行调用 `start` 服务。

---

## 7. MDU：安装 helper、sudoers 和新的 Runner package

### 7.1 把 helper stage 到 MDU

在 **Laptop HOST** 执行：

```bash
cd $HOPE_ROOT

scp -o "ProxyJump=${ROBOT_USER}@${HDU_IP}" \
  foxglove/helpers/hope-lifecycle \
  "${ROBOT_USER}@${MDU_IP}:/tmp/hope-lifecycle"
```

### 7.2 进入 MDU 并配置最小 sudo 权限

在 **Laptop HOST** 执行登录命令：

```bash
ssh -tt -J "${ROBOT_USER}@${HDU_IP}" "${ROBOT_USER}@${MDU_IP}"
```

看到 MDU 的 `agi@...` 提示符后，在 **MDU 原生 shell，不进 distrobox** 执行：

```bash
cd $HOME

test "$(command -v systemctl)" = /usr/bin/systemctl

if ! command -v tmux >/dev/null ||
   ! command -v rsync >/dev/null ||
   ! command -v flock >/dev/null; then
  sudo apt-get update
  sudo apt-get install -y tmux rsync util-linux
fi

command -v tmux
command -v rsync
test "$(command -v flock)" = /usr/bin/flock

sudo install -D -o root -g root -m 0755 \
  /tmp/hope-lifecycle \
  /usr/local/libexec/hope-lifecycle

ROBOT_LOGIN="$(id -un)"
printf '%s ALL=(root) NOPASSWD: %s\n' \
  "$ROBOT_LOGIN" '/usr/bin/systemctl stop agibot_pm.service' \
  "$ROBOT_LOGIN" '/usr/bin/systemctl start agibot_pm.service' \
  "$ROBOT_LOGIN" '/usr/local/libexec/hope-lifecycle time-calibration-preflight-mdu' \
  "$ROBOT_LOGIN" '/usr/local/libexec/hope-lifecycle time-calibration-stop-mdu' \
  "$ROBOT_LOGIN" '/usr/local/libexec/hope-lifecycle time-calibration-restore-mdu' \
  | sudo tee /etc/sudoers.d/hope-lifecycle >/dev/null

sudo chmod 0440 /etc/sudoers.d/hope-lifecycle
sudo visudo -cf /etc/sudoers.d/hope-lifecycle

install -d -m 0700 "$HOME/.config/hope-foxglove"
printf 'HOPE_ROBOT_USER=%q\n' "$ROBOT_LOGIN" \
  > "$HOME/.config/hope-foxglove/lifecycle.env"
chmod 0600 "$HOME/.config/hope-foxglove/lifecycle.env"

systemctl is-active agibot_pm.service
test -x /usr/local/libexec/hope-lifecycle

exit
```

最后一条 `is-active` 只读取状态，不会启停服务。Lifecycle 正常启动前要求
`agibot_pm.service` 为 active。

### 7.3 确认旧 Runner 未运行并备份现有 package

从 **Laptop HOST** 再次进入 MDU：

```bash
ssh -tt -J "${ROBOT_USER}@${HDU_IP}" "${ROBOT_USER}@${MDU_IP}"
```

在 **MDU** 执行：

```bash
if pgrep -u "$(id -un)" -f '[a]3_deploy_onnx_ref_pingpong' >/dev/null; then
  echo 'STOP: an existing Runner is active; do not overwrite its package.' >&2
  exit 1
fi

cd /agibot

if [[ -d /agibot/a3_deploy_model21800 ]]; then
  BACKUP_DIR="/agibot/a3_deploy_model21800_backup_$(date -u +%Y%m%dT%H%M%SZ)"
  cp -a --reflink=auto /agibot/a3_deploy_model21800 "$BACKUP_DIR"
  echo "backup=$BACKUP_DIR"
fi

exit
```

### 7.4 从 Laptop 上传当前 package

在 **Laptop HOST，不进 distrobox** 执行：

```bash
cd "$HOPE_ROOT/agibot/code_deployment/a3_deploy_example"

rsync -azP -e "ssh -J ${ROBOT_USER}@${HDU_IP}" \
  dist/a3_deploy_rockchip/ \
  "${ROBOT_USER}@${MDU_IP}:/agibot/a3_deploy_model21800/"
```

不使用 `--delete`，不会删除目标目录里额外的现场文件。

### 7.5 MDU 核对上传结果

在 **Laptop HOST** 执行下面一个只读 block：

```bash
ssh -J ${ROBOT_USER}@${HDU_IP} ${ROBOT_USER}@${MDU_IP} '
  set -e
  cd /agibot/a3_deploy_model21800
  sha256sum \
    a3_deploy_onnx_ref_pingpong \
    run_a3_pingpong.sh \
    config/a3_aimrt_config.pingpong_ros2body.yaml \
    config/a3_runtime_config.pingpong.hitter_pingpong.yaml \
    policy/exported/policy.onnx \
    policy/params/deploy.yaml
  file a3_deploy_onnx_ref_pingpong
  test -x run_a3_pingpong.sh
  strings a3_deploy_onnx_ref_pingpong | \
    grep -E "/hope/runner/(control_request_flat|state_flat)|SET_SERVER|SET_RECEIVER"
  systemctl is-active agibot_pm.service
'
```

把这里的六个 hash 与 Laptop 的
`/tmp/hope_open_model21800_candidate_sha256.txt` 对照。第一行 Runner binary
以及其余五个文件都应一致。特别确认
`config/a3_aimrt_config.pingpong_ros2body.yaml` 的 hash 一致；否则不要 START。

---

## 8. Laptop HOST：确认只保留球桌和 Pelvis link

在 **Laptop HOST** 执行：

```bash
cd $HOPE_ROOT

if rg -n 'robot_description|joint_states|urdf-a3' \
  foxglove/layouts/model21800_console.json; then
  echo "STOP: formal Layout still contains robot rendering" >&2
  exit 1
fi

test "$(rg -c 'foxglove\.Urdf' \
  foxglove/layouts/model21800_console.json)" -eq 1
grep -F 'hope_ping_pong_table.urdf' \
  foxglove/layouts/model21800_console.json

if rg -n '\^/tf\$|\^/tf_static\$' \
  foxglove/a3/bridge_params_control.yaml; then
  echo "STOP: control bridge still exposes the TF tree" >&2
  exit 1
fi

grep -F 'pelvis/(pose|text|marker|tf)' \
  foxglove/a3/bridge_params_control.yaml
systemctl --user is-active hope-foxglove-assets.service
echo "Table plus Pelvis-only transform surface: OK"
```

只有在 `world -> pelvis_link` 新鲜时才显示 Pelvis 世界位姿标签，应同时查看
`LIVE TF READY`。球 marker 仍作为独立功能保留，但不会产生机器人 link。

---

## 9. Foxglove Desktop：安装或升级 HOPE UI

这一节在 **Laptop 桌面**操作。Foxglove Desktop 是独立应用，不是
浏览器页面、终端或 distrobox。

UI 源码和设计均在本仓库的 `foxglove/extensions/hope-a3-console`。现场不需要
额外的设计目录或私有下载内容；实际可安装产物是第 1 节本地生成的 `.foxe`。

### 9.1 首次使用：安装并启动 Foxglove Desktop

先在 **Laptop HOST，不进 distrobox** 检查：

```bash
command -v foxglove-studio || true
dpkg --print-architecture
```

如果第一条没有输出，表示尚未安装。Laptop 架构应输出 `amd64`；
在 Chrome 打开 Foxglove 官方下载页：

```text
https://foxglove.dev/download
```

选择 **Linux x64**（不要选 Linux arm64）并把 `.deb` 下载到
`$DOWNLOAD_DIR`。下载完后，在 **Laptop HOST** 使用明确的 AMD64
文件名执行；不要使用 `foxglove-studio-*.deb` 通配符，因为目录里如果还留着
ARM64 包，`apt` 可能选错架构：

```bash
cd $DOWNLOAD_DIR

test "$(dpkg --print-architecture)" = amd64
test "$(dpkg-deb -f foxglove-studio-latest-linux-amd64.deb Architecture)" = amd64

sudo apt install ./foxglove-studio-latest-linux-amd64.deb
command -v foxglove-studio
```

如果下载目录里已经有 `foxglove-studio-latest-linux-arm64.deb`，无需用它，
也不要把它传给 `apt`；它是给 ARM64 主机使用的，不适用于这台 AMD64 Laptop。

如果启动时报告 `The SUID sandbox helper binary was found, but is not
configured correctly`，说明运行的是手工解包到用户目录的 Electron 程序，而不是
上面由 `apt` 正式安装的版本。正式现场安装应重新执行上面的 AMD64 `apt install`
命令；不要添加 `--no-sandbox`，因为它会关闭整个 Chromium 沙箱。

安装后可以用两种方式打开：

- 按键盘 `Super` 键（Windows 图标），搜索 **Foxglove**，点击 Foxglove 图标；
- 或在 Laptop 终端执行 `foxglove-studio`。

首次启动按提示登录 Foxglove 账号。安装本地 `.foxe` 需要账号具有
Developer seat；如果命令面板没有本地 extension 安装项，先检查账号 seat。

### 9.2 在 Foxglove Desktop 中安装 HOPE UI

1. 启动 Foxglove Desktop 并进入 Visualization 界面。
2. 按 `Ctrl+K` 打开 command palette。
3. 输入并选择 `Install local extension…`。如果旧版界面没有该命令，
   也可以把 `.foxe` 文件从文件管理器拖入已打开的 Foxglove Visualization 页面。
4. 先在 **Laptop HOST** 打印当前安装包的绝对路径：

   ```bash
   cd "$HOPE_ROOT/foxglove/extensions/hope-a3-console"
   UI_VERSION="$(python3 -c 'import json; print(json.load(open("package.json"))["version"])')"
   FOXE="$PWD/hopeopen.hope-a3-console-$UI_VERSION.foxe"
   test -f "$FOXE"
   printf '%s\n' "$FOXE"
   ```

   然后在 Foxglove 中选择该命令打印的文件。文件选择器不会展开 `$HOPE_ROOT`
   或 `$FOXE` 变量。

5. 如果 Installed Extensions 中存在多个 `HOPE A3 Console` 版本，卸载或禁用
   旧版，只保留 `$UI_VERSION` 对应的当前包。

6. 安装或重新启用当前版本后，使用 Foxglove 菜单中的 **Quit**（或 `Ctrl+Q`）
   完全退出 Desktop。只关闭 Visualization 窗口不一定会退出 Electron 主进程。
7. 在 **Laptop HOST** 确认进程已退出，然后重新启动：

   ```bash
   while pgrep -f '^/opt/Foxglove/foxglove-studio($| )' >/dev/null; do
     echo "Waiting for Foxglove Desktop to quit..."
     sleep 1
   done

   foxglove-studio
   ```

8. 重新进入 Visualization，打开 **Add panel** 并搜索 `HOPE A3 Console`。只有搜索
   结果中已经出现这个 panel，才继续 9.3 导入 Layout。

如果 Add panel 中没有 `HOPE A3 Console`，先不要导入 Layout。在 **Laptop HOST**
执行下面的只读检查：

```bash
SRC="$HOPE_ROOT/foxglove/extensions/hope-a3-console"
UI_VERSION="$(python3 -c 'import json, sys; print(json.load(open(sys.argv[1]))["version"])' "$SRC/package.json")"
EXT="$HOME/.foxglove-studio/extensions/hopeopen.hope-a3-console-$UI_VERSION"

test -r "$EXT/package.json"
test -r "$EXT/dist/extension.js"
grep -q '"displayName": "HOPE A3 Console"' "$EXT/package.json"
cmp "$SRC/package.json" "$EXT/package.json"
cmp "$SRC/dist/extension.js" "$EXT/dist/extension.js"
echo "HOPE extension files: OK"
```

五条检查通过但 Add panel 仍没有该名称时，在 **Extensions** 中把当前版本禁用后
重新启用，再完整 Quit/reopen 一次。仍失败则按 `Ctrl+Shift+I` 打开 Developer Tools，
保留 Console 中第一条 extension activation error；这时是 Desktop 扩展加载错误，
不是 HDU、ROS 或 8766 错误。

#### 9.2.1 已部署旧版现场：安装 Time Calibration 增量后端

本小节只用于已经能通过 `ws://<HDU-IP>:8766` 使用旧版 HOPE A3 Console、且
HDU lifecycle supervisor 与 MDU helper 已经部署的现场。全新部署已经在第 6.4 和
7.2 节安装了相同文件，应跳过本小节。仅增加 Time Calibration 不需要执行第 4、6.2、
6.3、7.3、7.4 或 7.5 节，也不需要重建或上传 Planner、Runner、ONNX 和 policy。

开始前确认机器人已经物理支撑、实体急停可触达、lifecycle 为 `STOPPED`，并且没有
Policy、Runner、Planner 或受管硬件 session 正在运行。部署期间不要点击
`START SYSTEM`。在增量后端安装完成前，新 UI 中 Time Calibration 状态显示
`STALE` 或按钮保持禁用是正常现象。

先在 **Laptop HOST，不进 distrobox** stage 当前文件：

```bash
cd "$HOPE_ROOT"

test -n "${ROBOT_USER:-}"
test -n "${HDU_IP:-}"
test -n "${MDU_IP:-}"

ssh "${ROBOT_USER}@${HDU_IP}" \
  'mkdir -p "$HOME/foxglove_a3"'

rsync -azP \
  --exclude '__pycache__/' \
  --exclude '*.pyc' \
  foxglove/a3/ \
  "${ROBOT_USER}@${HDU_IP}:~/foxglove_a3/"

scp foxglove/helpers/hope-lifecycle \
  "${ROBOT_USER}@${HDU_IP}:/tmp/hope-lifecycle"

scp -o "ProxyJump=${ROBOT_USER}@${HDU_IP}" \
  foxglove/helpers/hope-lifecycle \
  "${ROBOT_USER}@${MDU_IP}:/tmp/hope-lifecycle"
```

先从 **Laptop HOST** 登录 MDU：

```bash
ssh -tt -J "${ROBOT_USER}@${HDU_IP}" "${ROBOT_USER}@${MDU_IP}"
```

在 **MDU 原生 shell，不进 distrobox** 更新 helper 和最小 sudoers：

```bash
if ! command -v flock >/dev/null; then
  sudo apt-get update
  sudo apt-get install -y util-linux
fi

test "$(command -v flock)" = /usr/bin/flock

sudo install -D -o root -g root -m 0755 \
  /tmp/hope-lifecycle \
  /usr/local/libexec/hope-lifecycle

ROBOT_LOGIN="$(id -un)"
printf '%s ALL=(root) NOPASSWD: %s\n' \
  "$ROBOT_LOGIN" '/usr/bin/systemctl stop agibot_pm.service' \
  "$ROBOT_LOGIN" '/usr/bin/systemctl start agibot_pm.service' \
  "$ROBOT_LOGIN" '/usr/local/libexec/hope-lifecycle time-calibration-preflight-mdu' \
  "$ROBOT_LOGIN" '/usr/local/libexec/hope-lifecycle time-calibration-stop-mdu' \
  "$ROBOT_LOGIN" '/usr/local/libexec/hope-lifecycle time-calibration-restore-mdu' \
  | sudo tee /etc/sudoers.d/hope-lifecycle >/dev/null

sudo chmod 0440 /etc/sudoers.d/hope-lifecycle
sudo visudo -cf /etc/sudoers.d/hope-lifecycle

sudo -n /usr/local/libexec/hope-lifecycle \
  time-calibration-preflight-mdu

exit
```

最后一条是只读 preflight，不会停止服务；预期包含：

```text
HOPE_LIFECYCLE_V1 step=TIME_CALIBRATION state=COMPLETE reason=MDU_PREFLIGHT_READY
```

如果它报告 `RUNNER_PRESENT`、`NON_VENDOR_HAL_PRESENT` 或 PTP service 不 active，
不要继续 HDU 安装或为了变绿而执行通用 `pkill`；先查清当前运行状态。

再从 **Laptop HOST** 登录 HDU：

```bash
ssh -tt "${ROBOT_USER}@${HDU_IP}"
```

在 **HDU 原生 shell，不进 distrobox** 安装 Time Calibration、共享 lifecycle
interlock 和 8766 allowlist 的增量文件：

```bash
sudo install -D -o root -g root -m 0755 \
  "$HOME/foxglove_a3/hope_lifecycle_supervisor.py" \
  /usr/local/bin/hope_lifecycle_supervisor.py
sudo install -D -o root -g root -m 0755 \
  "$HOME/foxglove_a3/hope_time_calibration.py" \
  /usr/local/bin/hope_time_calibration.py

sudo install -D -o root -g root -m 0644 \
  "$HOME/foxglove_a3/hope_lifecycle_core.py" \
  /usr/local/lib/hope-foxglove/hope_lifecycle_core.py
sudo install -D -o root -g root -m 0644 \
  "$HOME/foxglove_a3/hope_time_calibration_core.py" \
  /usr/local/lib/hope-foxglove/hope_time_calibration_core.py

sudo install -D -o root -g root -m 0755 \
  /tmp/hope-lifecycle \
  /usr/local/libexec/hope-lifecycle

sudo install -D -o root -g root -m 0644 \
  "$HOME/foxglove_a3/bridge_params_control.yaml" \
  /etc/hope-foxglove/control_bridge_params.yaml
sudo install -D -o root -g root -m 0644 \
  "$HOME/foxglove_a3/hope-lifecycle-supervisor.service" \
  /etc/systemd/system/hope-lifecycle-supervisor.service
sudo install -D -o root -g root -m 0644 \
  "$HOME/foxglove_a3/hope-time-calibration.service" \
  /etc/systemd/system/hope-time-calibration.service
sudo install -D -o root -g root -m 0644 \
  "$HOME/foxglove_a3/hope-foxglove-control-bridge.service" \
  /etc/systemd/system/hope-foxglove-control-bridge.service

sudo systemctl daemon-reload
sudo systemctl enable hope-time-calibration.service
sudo systemctl restart hope-lifecycle-supervisor.service
sudo systemctl restart hope-time-calibration.service
sudo systemctl restart hope-foxglove-control-bridge.service
```

重启 control bridge 时现有 8766 连接会短暂断开。仍在 **HDU** 做只读验证：

```bash
systemctl is-active \
  hope-lifecycle-supervisor.service \
  hope-time-calibration.service \
  hope-foxglove-control-bridge.service

systemctl is-enabled hope-time-calibration.service
ss -lnt | grep -E ':8766[[:space:]]'

source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=232
export ROS_LOCALHOST_ONLY=0

ros2 service type /hope/lifecycle/time_calibration
timeout 10s ros2 topic echo \
  /hope/lifecycle/time_calibration/state --once

exit
```

三个 service 应全部为 `active`，time calibration service 应为 `enabled`，8766
应重新监听，service type 必须是 `std_srvs/srv/Trigger`。第一次安装且尚未执行过校时
时，状态应为 `IDLE`；协调器状态会持久化，所以重复部署时也可能保留之前的
`COMPLETE`、`REJECTED` 或 `FAILED_SAFE_STOP`，不要通过删除状态文件绕过它。
service 启动失败时在 HDU 查看：

```bash
journalctl -u hope-time-calibration.service -b --no-pager
```

### 9.3 连接 HDU 并导入 Layout

开始本节前，必须已经在 9.2 的 **Add panel** 中看到 `HOPE A3 Console`。Layout
只保存 panel 类型和布局，不会替你安装或激活扩展。

当前 Foxglove Desktop 2.59 保存的本地扩展 panel type 是：

```text
hope-a3-console.HOPE A3 Console!operator
```

仓库中的正式 Layout 已使用这个完整类型。不要把它简写成旧格式
`HOPE A3 Console!operator`，否则即使扩展已经加载，Foxglove 仍会显示 Unknown。

1. 从 Foxglove dashboard 或左侧菜单点击 **Open connection**。
2. 选择 **Foxglove WebSocket**。
3. 先在 Laptop HOST 执行 `printf 'ws://%s:8766\n' "$HDU_IP"`，把打印出的完整
   URL 输入连接框并点击 **Open**。不要把下面的 shell 变量名原样填进 GUI：

   ```text
   ws://<HDU-IP>:8766
   ```

4. 先在 Laptop HOST 执行
   `realpath "$HOPE_ROOT/foxglove/layouts/model21800_console.json"`，再在顶部工具栏
   打开 **Layouts** 菜单，选择 **Import from file…** 并选择打印出的文件。文件
   选择器不会展开下面的 shell 变量：

   ```text
   <HOPE_ROOT>/foxglove/layouts/model21800_console.json
   ```

5. 如果出现 `Unknown panel type: HOPE A3 Console`，不要修改 8766：
   - 如果 Add panel 中没有 `HOPE A3 Console [local]`，扩展尚未 activation，回到
     9.2 完全退出并重启；
   - 如果 Add panel 中已经有 `HOPE A3 Console [local]`，说明导入的是使用旧 panel
     type 的 Layout。确认当前仓库 JSON 中含有
     `hope-a3-console.HOPE A3 Console!operator`，然后重新导入正式 Layout。
6. 如果 panel 类型已经识别但内容暂时空白，重载当前窗口一次。不要同时使用旧的
   `8765` layout 作为这个窗口的数据源；新 console 只使用 `8766`。

如果 `Open connection` 报 connection refused，先在 Laptop 检查
`nc -vz ${HDU_IP} 8766`；这表示 HDU control bridge 未监听，不是 Layout 导入问题。
同一版本的布局只需导入一次。Foxglove Desktop 保存的是导入时的副本；仓库中的
`model21800_console.json` 更新后（例如增加球 Marker 或移除机器人 URDF），必须
重新 **Import from file…**，不能只重连 WebSocket。
仅执行 9.2.1 的 Time Calibration 增量升级时，本次没有修改正式 Layout；已有窗口
只需确认当前扩展已升级、重新连接 8766，不需要重复导入 Layout。

---

## 10. 每次现场测试开始前：按顺序完成 preflight

每次 session 都按下面的顺序执行。除非 HDU 时钟不合格，正常路径的 10.1、10.2、
10.3 和 10.5 都是只读检查：

```text
10.1 Laptop HOST 网络和本地服务
  -> 10.2 HDU 控制面和 UTC/NTP
       -> HDU 时钟不合格：跳到 10.4，恢复后从 10.1 重新检查
       -> HDU 时钟合格：继续 10.3 MDU/PTP
  -> 10.5 Laptop hope Distrobox 中检查 ROS 发现
  -> 第 11 节连接 Foxglove 并 START SYSTEM
```

这里的时钟判定是现场 runbook 的准入条件，不是 Runner/lifecycle 新增的自动运动
gate。Foxglove 不会、也不应在机器人运行时自动 step 系统时钟。

### 10.1 Laptop HOST 网络和本地服务

在 **Laptop HOST** 执行：

```bash
cd $HOPE_ROOT

ping -c 3 ${MOTIVE_IP}
ping -c 3 ${HDU_IP}
ip route get ${MOTIVE_IP}
ip route get ${HDU_IP}

ssh -o BatchMode=yes ${ROBOT_USER}@${HDU_IP} true
ssh -o BatchMode=yes -J ${ROBOT_USER}@${HDU_IP} ${ROBOT_USER}@${MDU_IP} true

test -x /usr/local/libexec/hope-lifecycle
test -d $HOPE_ROOT/hope_ws/install
test -r $HOME/.local/share/hope-foxglove/hope_marker_monitor.py

if command -v chronyc >/dev/null; then
  chronyc tracking || true
  chronyc -n sources -v || true
else
  timedatectl show -p NTPSynchronized -p TimeUSec || true
fi

echo "Laptop preflight: OK"
```

只要求 Laptop 能访问 Motive。HDU 不能直接访问 Motive 是当前双网卡拓扑的正常
结果。

### 10.2 HDU 常驻控制面

在 **Laptop HOST** 执行下面的远程只读 block：

```bash
ssh ${ROBOT_USER}@${HDU_IP} '
  set -e
  systemctl is-active \
    hope-monitor.service \
    hope-foxglove-bridge.service \
    hope-observer.service \
    hope-command-proxy.service \
    hope-foxglove-control-bridge.service \
    hope-lifecycle-supervisor.service \
    hope-time-calibration.service
  test -x /usr/local/libexec/hope-lifecycle
  test -x /usr/local/libexec/hope-base-pose-transport-relay
  test -d $HOME/hope_ws/install
  test -d $HOME/hope_ws/install_model21800_fix
  ss -lnt | grep -E ":8766[[:space:]]"
  test "$(pgrep -x ptp4l | wc -l)" -eq 1
  test "$(pgrep -x phc2sys | wc -l)" -eq 1
  pgrep -a -x ptp4l
  pgrep -a -x phc2sys
  chronyc tracking
'
```

检查 `chronyc tracking`：`Leap status` 必须是 `Normal`，`System time` 的绝对值
必须不大于 `0.010 seconds`（10 ms），`Skew` 必须不大于 `5 ppm`。三项任一不满足，
不要继续 10.3 或启动机器人，执行 10.4 的受控校时流程。现场已验证的正常示例为
`System time 0.000065644 seconds fast`、`Skew 0.904 ppm`。

### 10.3 MDU 初始状态

在 **Laptop HOST** 执行下面的远程只读 block：

```bash
ssh -J ${ROBOT_USER}@${HDU_IP} ${ROBOT_USER}@${MDU_IP} '
  set -e
  systemctl is-active agibot_pm.service
  test -x /usr/local/libexec/hope-lifecycle
  test -x /agibot/a3_deploy_model21800/run_a3_pingpong.sh
  command -v tmux
  if pgrep -u agi -f "[a]3_deploy_onnx_ref_pingpong"; then
    echo "STOP: unmanaged Runner is already active" >&2
    exit 1
  fi
  while IFS= read -r hal_pid; do
    [[ -n "$hal_pid" ]] || continue
    if ! grep -Eq \
      "^[^:]*:[^:]*:/system[.]slice/agibot_pm[.]service(/.*)?$" \
      "/proc/$hal_pid/cgroup"; then
      echo "STOP: HAL pid $hal_pid is outside agibot_pm.service" >&2
      exit 1
    fi
    echo "Expected vendor HAL in agibot_pm.service: pid $hal_pid"
  done < <(pgrep -f "[a]imrt_main_hal" || true)
  systemctl is-active \
    agibot-mdu-ptp4l.service \
    agibot-mdu-phc2sys.service
  test "$(pgrep -x ptp4l | wc -l)" -eq 1
  test "$(pgrep -x phc2sys | wc -l)" -eq 1
  pgrep -a -x ptp4l
  pgrep -a -x phc2sys
  echo "MDU preflight: OK"
'
```

如果发现旧的 unmanaged Runner、HAL、Planner、bridge 或固定 tmux session，先查明
来源。不要为了让 preflight 变绿而执行通用 `pkill`。

MDU 开机后的厂家 `agibot_pm.service` 本身会启动一个 `aimrt_main_hal`。只要该
HAL 的 cgroup 属于 `/system.slice/agibot_pm.service`，它就是 STEP 4 将正常停止的
厂家 HAL，不是 unmanaged 冲突。Lifecycle 会先停止整个 `agibot_pm.service`，确认
厂家 HAL 已退出，再启动固定 `hope-hal` tmux session；只有位于该 systemd cgroup
之外的 HAL 才会使 preflight fail closed。

### 10.4 仅当 10.2 时钟不合格：受控 hard-step 与服务恢复

这一节会修改系统时钟并停止机器人相关服务，只能在机器人已经物理支撑、没有站立、
没有运行 Policy、没有 Runner session 时执行。实体急停必须可触达。正常时钟已经合格
时跳过本节；同一次维护中不要重复 hard-step。

恢复顺序必须保持为：先停止下游 MDU，再停止 HDU；校准 HDU UTC 后，先恢复 HDU
上游时钟，再恢复 MDU PTP，最后恢复 Foxglove 控制面。

正常现场操作不再复制粘贴 10.4.1–10.4.6 的 shell block。保持 lifecycle 为
`STOPPED`，在 HOPE A3 Console 中确认四个现场 IP（只点 `CONFIRM CONFIG`，不要点
`START SYSTEM`）。当面板上的 `NTP · AUDIT` 为失败且数据新鲜时，
`TIME CALIBRATION` 按钮才会启用。点击后再次确认机器人已物理支撑、Policy/Runner
没有运行且实体急停可触达。

按钮状态按下面解释：

- `NTP · AUDIT` 已合格时按钮禁用是正常行为，不需要 hard-step；
- Time Calibration 卡片显示 `STALE` 时，先检查第 9.2.1 节的 coordinator 和 8766
  allowlist 是否已经部署并 active；
- NTP 数据新鲜且失败、但按钮仍禁用时，检查 lifecycle 是否为 `STOPPED`、四个 IP
  是否已 `CONFIRM CONFIG`、输入框是否又被修改，以及 coordinator 是否 busy/locked。

预期过程为：

```text
RUNNING · HANDOFF
→ RUNNING · PREFLIGHT
→ RUNNING · MDU_STOP
→ RUNNING · HDU_STOP
→ Foxglove :8766 暂时断开
→ HARD_STEP（本次维护周期最多一次）
→ HDU_RESTORE
→ MDU_RESTORE
→ CONTROL_RESTORE
→ Foxglove :8766 恢复
→ COMPLETE · CLOCK_QUALIFIED ... SERVICES_RESTORED
```

最长的 `chronyc waitsync` 仍可能等待 20 分钟；断开期间不要再次点击、不要手工启动
任何 vendor、PTP、Runner 或 Foxglove service。连接恢复后只有状态为 `COMPLETE`、
NTP gate 变绿，并重新完成 10.1、10.2、10.3，才能继续。协调器会持久化进度，并拒绝
同一维护周期中的第二次 hard-step。一次完整 lifecycle 进入 `RUNNING` 后才自动为下一
次维护重新解锁；HDU 重启也会开始新周期。它和 `START SYSTEM` 在整个事务期间共同持有同一
个硬件操作 interlock；另一个 Foxglove 窗口或直接 service call 不能并发启动 Runner。

如果显示 `REJECTED`，表示尚未改动机器状态，按 result 修正 preflight 后可重新请求。
如果显示 `FAILED_SAFE_STOP`，机器人相关 HDU/MDU vendor 与 PTP 服务保持停止；协调器
只保证尽力恢复 `chrony.service` 和 Foxglove 控制面用于诊断。此时不要再次校时或启动
系统，保留实体支撑并按下面的手工 block 检查/恢复。10.4.1–10.4.6 因而保留为故障
恢复和实现审计参考，不再是每次测试的正常操作路径。`FAILED_SAFE_STOP` 后本次 boot
不要再次执行 10.4.3；先根据 service journal 和 `chronyc tracking` 查清失败点，再由
现场负责人决定只恢复 10.4.4–10.4.6，还是关机后重新开始维护。

协调器把每个固定命令的 return code 和截断后的 stdout/stderr 写入本机 journal。故障时
在 **HDU** 读取，不要从 Foxglove 增加通用日志/命令接口：

```bash
journalctl -u hope-time-calibration.service -b --no-pager
```

#### 10.4.1 先停止 MDU 时间消费者

在 **Laptop HOST** 执行。命令只停止 MDU 上“已加载且当前 active”的 vendor unit，
因此不会再因为这台 MDU 没有 `agibot_ui.service` 或 `agibot_top.service` 而在
`set -e` 下提前退出：

```bash
ssh -J ${ROBOT_USER}@${HDU_IP} ${ROBOT_USER}@${MDU_IP} '
  set -e

  RESTORE=/tmp/hope-clock-active-mdu
  : > "$RESTORE"

  for SERVICE in \
    agibot_roudi.service \
    agibot_top.service \
    agibot_ui.service \
    agibot_pm.service; do
    if systemctl cat "$SERVICE" >/dev/null 2>&1 &&
       systemctl is-active --quiet "$SERVICE"; then
      echo "$SERVICE" >> "$RESTORE"
    fi
  done

  mapfile -t ACTIVE_SERVICES < "$RESTORE"
  if ((${#ACTIVE_SERVICES[@]})); then
    sudo systemctl stop "${ACTIVE_SERVICES[@]}"
  fi

  sudo systemctl stop \
    agibot-mdu-phc2sys.service \
    agibot-mdu-ptp4l.service

  ! pgrep -u agi -f "[a]3_deploy_onnx_ref_pingpong"
  ! pgrep -f "[a]imrt_main_hal"
  ! systemctl is-active --quiet agibot-mdu-ptp4l.service
  ! systemctl is-active --quiet agibot-mdu-phc2sys.service

  echo "MDU clock consumers stopped"
'
```

#### 10.4.2 再停止 HDU 控制面、vendor 服务和 chrony

仍在 **Laptop HOST** 执行：

```bash
ssh ${ROBOT_USER}@${HDU_IP} '
  set -e

  VENDOR_RESTORE=/tmp/hope-clock-active-hdu-vendor
  FOXGLOVE_RESTORE=/tmp/hope-clock-active-hdu-foxglove
  : > "$VENDOR_RESTORE"
  : > "$FOXGLOVE_RESTORE"

  for SERVICE in \
    agibot_roudi.service \
    agibot_top.service \
    agibot_ui.service \
    agibot_pm.service; do
    if systemctl cat "$SERVICE" >/dev/null 2>&1 &&
       systemctl is-active --quiet "$SERVICE"; then
      echo "$SERVICE" >> "$VENDOR_RESTORE"
    fi
  done

  for SERVICE in \
    hope-monitor.service \
    hope-foxglove-bridge.service \
    hope-observer.service \
    hope-command-proxy.service \
    hope-foxglove-control-bridge.service \
    hope-lifecycle-supervisor.service; do
    if systemctl cat "$SERVICE" >/dev/null 2>&1 &&
       systemctl is-active --quiet "$SERVICE"; then
      echo "$SERVICE" >> "$FOXGLOVE_RESTORE"
    fi
  done

  mapfile -t ACTIVE_SERVICES < "$FOXGLOVE_RESTORE"
  if ((${#ACTIVE_SERVICES[@]})); then
    sudo systemctl stop "${ACTIVE_SERVICES[@]}"
  fi

  mapfile -t ACTIVE_SERVICES < "$VENDOR_RESTORE"
  if ((${#ACTIVE_SERVICES[@]})); then
    sudo systemctl stop "${ACTIVE_SERVICES[@]}"
  fi

  for ATTEMPT in {1..20}; do
    if ! pgrep -x ptp4l >/dev/null &&
       ! pgrep -x phc2sys >/dev/null; then
      break
    fi
    sleep 0.5
  done

  if pgrep -a -x ptp4l || pgrep -a -x phc2sys; then
    echo "STOP: HDU PTP worker did not exit" >&2
    exit 1
  fi

  sudo systemctl stop chrony.service
  echo "HDU clock consumers stopped"
'
```

#### 10.4.3 对 HDU UTC 执行一次 hard-step

仍在 **Laptop HOST** 执行。`waitsync` 最长等待 20 分钟；多数情况下会更快返回：

```bash
ssh -tt ${ROBOT_USER}@${HDU_IP} '
  set -e

  sudo systemctl reset-failed agibot-clock-bootstrap.service
  sudo systemctl restart agibot-clock-bootstrap.service
  sudo test -e /run/agibot-time/bootstrap-qualified

  sudo systemctl start chrony.service
  chronyc waitsync 600 0.010 5 2
  chronyc tracking

  echo "HDU UTC hard-step complete"
'
```

只有输出重新满足 `Leap status Normal`、绝对 `System time <= 0.010 seconds`、
`Skew <= 5 ppm` 才能继续。如果命令失败，保持机器人服务停止，只启动
`chrony.service` 并保存 `systemctl status agibot-clock-bootstrap.service` 与
`chronyc tracking` 输出；不要在服务运行后重试 hard-step。

失败时只恢复 chrony 并收集诊断，随后停止本次机器人启动：

```bash
ssh ${ROBOT_USER}@${HDU_IP} '
  sudo systemctl start chrony.service
  systemctl status agibot-clock-bootstrap.service --no-pager || true
  chronyc tracking || true
'
```

#### 10.4.4 先恢复 HDU vendor 时钟链

当前这台 HDU 的 `ptp4l/phc2sys` 仍由 vendor `agibot_pm.service` 启动，所以先恢复
维护前 active 的 HDU vendor unit，再等待两个 PTP worker：

```bash
ssh ${ROBOT_USER}@${HDU_IP} '
  set -e

  while IFS= read -r SERVICE; do
    [[ -n "$SERVICE" ]] && sudo systemctl start "$SERVICE"
  done < /tmp/hope-clock-active-hdu-vendor

  for ATTEMPT in {1..90}; do
    if pgrep -x ptp4l >/dev/null &&
       pgrep -x phc2sys >/dev/null; then
      break
    fi
    sleep 1
  done

  pgrep -a -x ptp4l
  pgrep -a -x phc2sys

  while IFS= read -r SERVICE; do
    [[ -n "$SERVICE" ]] && systemctl is-active "$SERVICE"
  done < /tmp/hope-clock-active-hdu-vendor
'
```

仓库中的长期正式方案是由独立 systemd unit 监管 HDU 和 MDU 的四个 PTP worker，
见 `agibot/ntp_sync/README.md`。在完成那套部署前，不要把其中的安装步骤混入本次
现场启动；本节按当前机器上已经验证的 vendor-owned HDU PTP 拓扑恢复。

#### 10.4.5 恢复 MDU PTP，再恢复 MDU vendor 服务

在 **Laptop HOST** 执行：

```bash
ssh -J ${ROBOT_USER}@${HDU_IP} ${ROBOT_USER}@${MDU_IP} '
  set -e

  sudo systemctl start \
    agibot-mdu-ptp4l.service \
    agibot-mdu-phc2sys.service

  systemctl is-active \
    agibot-mdu-ptp4l.service \
    agibot-mdu-phc2sys.service

  sleep 5

  while IFS= read -r SERVICE; do
    [[ -n "$SERVICE" ]] && sudo systemctl start "$SERVICE"
  done < /tmp/hope-clock-active-mdu

  while IFS= read -r SERVICE; do
    [[ -n "$SERVICE" ]] && systemctl is-active "$SERVICE"
  done < /tmp/hope-clock-active-mdu
'
```

#### 10.4.6 最后恢复 HDU Foxglove 控制面

在 **Laptop HOST** 执行：

```bash
ssh ${ROBOT_USER}@${HDU_IP} '
  set -e

  sudo systemctl reset-failed \
    hope-monitor.service \
    hope-foxglove-bridge.service \
    hope-observer.service \
    hope-command-proxy.service \
    hope-foxglove-control-bridge.service \
    hope-lifecycle-supervisor.service \
    hope-time-calibration.service

  while IFS= read -r SERVICE; do
    [[ -n "$SERVICE" ]] && sudo systemctl start "$SERVICE"
  done < /tmp/hope-clock-active-hdu-foxglove

  systemctl is-active \
    hope-monitor.service \
    hope-foxglove-bridge.service \
    hope-observer.service \
    hope-command-proxy.service \
    hope-foxglove-control-bridge.service \
    hope-lifecycle-supervisor.service \
    hope-time-calibration.service

  ss -lnt | grep -E ":8766[[:space:]]"
  chronyc tracking
'
```

现在从 10.1 开始重新执行 10.1、10.2、10.3。`/tmp/hope-clock-active-*` 记录的是
维护开始前的 active 状态；如果任一机器在中途重启，不能继续使用旧记录，必须重新
确认服务状态。

### 10.5 Laptop hope Distrobox：ROS 发现与 3D 数据 smoke test

这一节从 **Laptop HOST** 进入 `hope` distrobox。不要在 Laptop HOST 直接执行
`source /opt/ros/jazzy/setup.bash`；HOST 上没有这个路径，也没有 `ros2`。

先在 **Laptop HOST** 执行：

```bash
distrobox enter hope
```

进入后，在 **Laptop hope Distrobox** 执行：

```bash
source /opt/ros/jazzy/setup.bash
source $HOPE_ROOT/hope_ws/install/local_setup.bash

DDS=$HOPE_ROOT/hope_ws/install/hope_bringup/lib/hope_bringup/with_fastdds_unicast.sh

"$DDS" --domain-id 232 --peer ${HDU_IP} -- bash -lc '
  set -e
  ros2 daemon stop >/dev/null 2>&1 || true
  trap "ros2 daemon stop >/dev/null 2>&1 || true" EXIT

  ros2 topic info /hope/ball/marker
  ros2 topic info /hope/pelvis/marker
  ros2 topic info /hope/pelvis/tf
  ros2 topic info /ball/flight_packet -v
  timeout 20s ros2 topic echo /hope/vendor/tf_ready --once
  timeout 20s ros2 topic echo /hope/pelvis/text --once
'
```

预期 `/hope/pelvis/marker`、`/hope/pelvis/tf` 和 `/hope/ball/marker` 的
`Publisher count` 至少为 1，这证明
Laptop 通过固定 Fast DDS unicast 配置发现了 HDU。`START SYSTEM` 之前 Laptop base relay 尚未启动时，
`tf_ready` 为 `false`、pelvis 报 `world` 不存在是允许的；这不能证明 3D TF 已就绪。

完成后退出 distrobox：

```bash
exit
```

第 11 节 `START SYSTEM` 到达 `RUNNING · RUNNER` 后，再执行一次 10.5。此时
`/ball/flight_packet` 必须显示 Laptop 的 `hope_ball_flight_packetizer` publisher；
没有球时没有 packet 消息是正常的，但 publisher count 不能为 0。Pelvis 3D
标签要求 `tf_ready` 为 `true`，且 `/hope/pelvis/text` 给出有限的世界坐标；球进入
Motive 捕捉范围时，`/hope/ball/marker` 才会产生消息。若仍为 false，按 13.3 诊断，
不要伪造 identity TF。

Pelvis 点和文字 Marker 的 lifetime 为 0.5 秒。权威 pelvis pose/TF 变旧后 Marker
主动消失，并由 `/hope/pelvis/text` 报告 stale 原因；这比保留旧世界坐标更安全。

---

## 11. 正常测试：从这里开始只操作 Foxglove

只有 10.1、10.2、10.3 和 10.5 已按顺序完成，并且没有待处理的 10.4 时钟异常，
才能进入本节。

### 11.1 确认四个输入框

先在 **Laptop HOST** 执行下面的命令并保留输出：

```bash
printf 'Laptop Wi-Fi   %s\nHDU Wi-Fi      %s\nMDU internal   %s\nMotive         %s\n' \
  "$LAPTOP_IP" "$HDU_IP" "$MDU_IP" "$MOTIVE_IP"
```

在 HOPE A3 Console 中填写命令打印出的四个地址。不要把 `${...}` 变量名原样
粘贴到 GUI：

```text
Laptop Wi-Fi   <laptop-wifi-ip>
HDU Wi-Fi      <hdu-wifi-ip>
MDU internal   <mdu-internal-ip>
Motive         <motive-ip>
```

确认系统状态为 `STOPPED`，然后点击 `CONFIRM CONFIG`。预期：

```text
config revision >= 1
CONFIG_CONFIRMED_REVISION_<n>
```

输入框在运行中仍可编辑，但只有 `STOPPED` 或 `CONFIG_ERROR` 才能确认。运行中修改
文字不会改变当前 session。

### 11.2 点击 START SYSTEM

确认现场人员已经扶住机器人且实体急停可触达，然后点击：

```text
START SYSTEM
```

不要同时手工启动旧 runbook 的任何 STEP。UI 应依次显示：

```text
STARTING · PREFLIGHT
STARTING · SESSION
STARTING · OPTITRACK
STARTING · BASE_RELAY
STARTING · PLANNER
STARTING · HAL
STARTING · RUNNER
STARTING · RUNNER_VERIFY
RUNNING · RUNNER
START_COMPLETE_RUNNER_PASSIVE
```

这个过程自动完成：

```text
创建统一 session
→ Laptop hope Distrobox 中启动 OptiTrack bridge、marker publisher 和 flight packetizer
→ 在 Laptop 同一进程树中启动 world frames、校准服务和 authoritative base relay
→ HDU 启动 C++ Planner
→ MDU 停止 agibot_pm 并启动 EtherCAT HAL
→ MDU 启动同一个 model_21800 Runner，初始为 PASSIVE
```

`RUNNER_VERIFY` 不是策略挥拍 gate，也不是额外人工步骤；它只防止生命周期把未启动
的 Runner 报成 RUNNING。Supervisor 最多等待 15 秒，要求 HDU 收到
与本次 session 匹配、1.5 秒内新鲜且模式为 `PASSIVE` 的 Runner 权威状态，之后才
允许显示 `RUNNING`。若 ROS 2 transport/YAML 配错，它会进入 `FAILED`，此时可在
物理支撑机器人后直接使用 `KILL ALL & COLLECT` 清理，而不会产生“进程启动了但按钮
全锁、KILL 又被拒绝”的假 RUNNING。

### 11.3 Runner 按钮顺序

Runner 到达 `PASSIVE` 后再操作：

1. 按 `Stand`，等待 Runner 确认 `PD_STAND` 且机器人站稳。
2. 选择本机角色：`Server` 或 `Receiver`。这里只改变我方 Runner，不控制对方。
3. 按 `Calibration`，保持机器人不动，等待十个 P1 marker 完成重新拟合。Laptop
   JSON 会保存稳定的 `P1 -> pelvis_link` 标定量和本次静止采样派生出的
   `world -> pelvis_link` audit snapshot；面板还会等待带新 calibration receipt 的
   新鲜 base packet。这个按钮不再刷新 `x_hit`。
4. 单独按 `Refresh x_hit`，等待当前 Planner 的 request/status ack。它不会重新标定
   `world -> pelvis_link`。
5. 需要进入策略时按 `Ready`；它等价于原键盘 `m`，Runner 最终决定是否进入
   `MOTION`。

现有算法支持这条链：Motive 给出实时 `world -> P1`，十 marker/CAD 刚体配准重新
计算固定的 `P1 -> pelvis_link`；Laptop base relay 每秒热加载获批 JSON，并组合出
实时 `world -> pelvis_link`。JSON 里的 `world_to_pelvis_snapshot` 只记录 Calibration
那一刻的静止位姿，不能作为机器人运动后的静态 TF；这里也不会重新定义球台的
`world` 原点。只有新 JSON 的 SHA-derived calibration ID 已出现在有效 schema-2
base packet 中，Calibration 才会完成。`Refresh x_hit` 有自己独立的成功/失败反馈。

面板继续保留按钮 gate：例如 `Calibration`/`Refresh x_hit` 需要新鲜的
`PD_STAND`，角色切换要等待 Runner 明确允许，`Serve` 需要相应 capability 和状态。
这些 gate 只约束操作员的按钮顺序，Runner 仍负责接受或拒绝请求。Foxglove 启动的
Runner 与 `run_v17_r1_fixed3_hardware_trial.md` STEP 5 使用相同命令参数和相同二进制，
不会另行改写其 Planner engage、挥拍或恢复行为；这里也不再添加 Foxglove 专属的
Runner 内部 gate。`Calibration` 的 `PD_STAND`/静止要求属于十 marker 采样算法的执行
前提，不授权 `Ready`。NTP、timestamp、TF、marker 和 E-Stop backend 行在 UI 中均
明确标为 `AUDIT`。

当前 package 是 `rally_v14`：

- `Server/Receiver` 状态接口可以测试；
- `Ready to Serve` 和 `Serve` 应显示不可用；
- 不要修改 UI 或服务来绕过 Runner 的 serve capability 检查。

### 11.4 E-Stop 和 Runner Passive 的区别

- `E-STOP`：按钮始终保持红色且可点击，backend ready/full-ready 只是 audit，不能
  屏蔽 assert。每次点击都会同时尝试 vendor emergency RPC 和 Runner emergency
  PASSIVE；任一条确认会返回 `ACCEPTED · PARTIAL E-STOP`，但 UI 不会把单路径结果
  冒充 dual-path 完整确认。首次点击立即写入
  `/var/lib/hope-monitor/estop-latched`；已闩锁后按钮仍可再次点击并重新 assert 两条
  路径。Foxglove 没有解除路径。不要为了测试按钮而在真机上触发。
- `Runner Passive`：等价于键盘 `p`，切到零增益 PASSIVE，机器人会失去主动
  支撑。它不是 vendor E-stop。
- 实体急停始终是现场主要安全手段，不能由 Foxglove 替代。

只有在机器人已物理支撑、厂商/实体 E-Stop 已按批准流程复位、现场负责人确认可以
恢复之后，才在 **HDU 原生 shell** 清除 Foxglove 软件 latch：

```bash
sudo systemctl stop hope-monitor.service
sudo test -f /var/lib/hope-monitor/estop-latched
sudo rm -- /var/lib/hope-monitor/estop-latched
sudo systemctl start hope-monitor.service
systemctl is-active hope-monitor.service
```

这条命令只清除 HOPE UI 的持久 latch，不会替你复位实体急停或厂商 emergency
状态。若 vendor 状态尚未安全复位，不得执行。

### 11.5 Kill All 和收日志

`KILL ALL & COLLECT` 不要求 Runner 先进入 PASSIVE/PD_STAND；它可从受管
`RUNNING` 或 `FAILED` session 直接执行。因为 Runner/HAL 被终止时机器人可能立即
失去主动支撑，点击前必须由现场人员物理支撑机器人，并确保实体 E-stop 随时可触达。
若当前情况允许，仍建议先按 `Stand`，但它不再是 lifecycle 的准入 gate。

1. 现场人员物理支撑机器人。
2. 确认实体 E-stop 随时可触达。
3. 点击 `KILL ALL & COLLECT`，并确认危险提示。
4. 等到以下最终结果：

```text
STOPPED · IDLE
KILL_COMPLETE_AGIBOT_PM_RESTORED_LOGS_COLLECTED
```

如果 managed recovery 指向的 session 还没有创建远端日志目录，KILL 清理成功后的
正常结果是：

```text
STOPPED · IDLE
KILL_COMPLETE_AGIBOT_PM_RESTORED_NO_REMOTE_SESSION_LOGS
```

这不是日志丢失：它明确表示本次 Runner 从未启动，因此没有远端 session 日志可收。
Laptop 会在对应目录写入 `collection_status.txt`。只有 SSH 不可达、实际 rsync 失败
或停止步骤失败时，状态才会保留为 `FAILED`。

终止顺序为：

```text
Runner
→ HAL
→ 恢复 MDU agibot_pm
→ Planner
→ Laptop world/calibration/base relay
→ Laptop OptiTrack/marker
→ rsync HDU/MDU 日志到 Laptop
```

这里的 `ALL` 只指 lifecycle 以固定名称创建的 Runner、HAL、Planner、base relay、
world/calibration 和 Laptop OptiTrack/marker/packetizer tmux sessions，不会执行
`pkill`/`killall`，也不会终止机器上任意其他进程。如果启动后的权威状态验证失败，
lifecycle 会进入 `FAILED`，此时同一个按钮仍可清理已经创建的受管 session。

### 11.6 为什么 KILL 后不能 START，以及如何恢复

`START SYSTEM` 只在 lifecycle 为 `STOPPED`（或首次配置错误状态）时启用。
`FAILED` 表示仍需要执行一次受管清理，所以 Stand、Calibration、Refresh x_hit、Ready 也不会因此
变得可用；这些按钮必须等同一个受管 Runner 启动并发布新鲜状态后才解锁。

恢复顺序：

1. 先确认机器人已被物理支撑。
2. 在 `FAILED` 状态点击一次 `KILL ALL & COLLECT`。
3. 等待 `STOPPED · IDLE`。若本次尚未建立 session，接受上面的
   `...NO_REMOTE_SESSION_LOGS` 结果。
4. 核对 `agibot_pm` 为绿色，再点击 `START SYSTEM`。
5. 只有达到 `RUNNING · RUNNER` 且 Runner 为新鲜 `PASSIVE` 后，才依次使用
   Stand、角色、Calibration、Refresh x_hit、Ready。

若第二步仍返回 `KILL_FAILED`，不要反复点 START。先在 **Laptop HOST** 执行：

```bash
ssh ${ROBOT_USER}@${HDU_IP} '
  systemctl status hope-lifecycle-supervisor.service --no-pager -l
  journalctl -u hope-lifecycle-supervisor.service -n 200 --no-pager
'
```

新版 supervisor 会把原始 START/KILL helper 错误写进 journal，避免后续收日志错误
覆盖最初原因。

如果界面在某个 START 步骤停留超过该步骤超时，先看 `busy`：新版 supervisor
会把内部异常转换成 `START_INTERNAL_ERROR`、清除 `busy` 并进入 `FAILED`，从而
允许 `KILL ALL & COLLECT` 清理已经启动的前置组件。不要绕过 lifecycle 手动补启动
后续 Runner；尤其在 HAL 已启动但持续报告 EtherCAT fault 时，必须先清理并恢复
`agibot_pm`。

`STARTING · HAL` 通常是整个 START 最慢的一步，因为它必须等待 MDU 的
`agibot_pm.service` 完整停止；现场可能需要 40–60 秒，supervisor 为它保留 120 秒。
在 `busy=true` 且未超过 120 秒时继续等待，不要重复点击。正常完成后会依次显示
`HAL_RUNNING`、`STARTING · RUNNER` 和 `START_COMPLETE_RUNNER_PASSIVE`。

---

## 12. 测试后核对日志

在 **Laptop HOST** 执行：

```bash
cd $HOPE_ROOT

SESSION_ID="$(cat real_logs/.active_session_id)"
LOG_ROOT="$HOPE_ROOT/real_logs/$SESSION_ID"

echo "SESSION_ID=$SESSION_ID"
find "$LOG_ROOT" -maxdepth 4 -type f | sort

test -f "$LOG_ROOT/laptop/laptop_bridge.log"
test -f "$LOG_ROOT/laptop/marker_monitor.log"
test -f "$LOG_ROOT/hdu/planner.log"
test -f "$LOG_ROOT/mdu/real/current_attempt"
test -f $HOPE_ROOT/calibration/p1_to_pelvis.json

ssh -J ${ROBOT_USER}@${HDU_IP} ${ROBOT_USER}@${MDU_IP} \
  'systemctl is-active agibot_pm.service'
```

Planner 每次启动写独立的 `planner_attempt_###`，Runner 每次启动写独立的
`attempt_###`，不会覆盖前一次 trace/obs。

---

## 13. 失败时的诊断命令

### 13.1 Foxglove 无法连接 8766

在 **Laptop HOST** 执行：

```bash
ping -c 3 ${HDU_IP}
nc -vz ${HDU_IP} 8766

ssh ${ROBOT_USER}@${HDU_IP} \
  'systemctl status hope-foxglove-control-bridge.service \
    hope-command-proxy.service \
    hope-lifecycle-supervisor.service --no-pager'
```

### 13.2 Lifecycle 进入 FAILED

在 **Laptop HOST** 执行以下只读命令：

```bash
ssh ${ROBOT_USER}@${HDU_IP} '
  journalctl -u hope-lifecycle-supervisor.service -n 200 --no-pager
  tmux ls 2>&1 || true
  cat /tmp/hope_model21800_session_id 2>/dev/null || true
'

ssh -J ${ROBOT_USER}@${HDU_IP} ${ROBOT_USER}@${MDU_IP} '
  tmux ls 2>&1 || true
  systemctl is-active agibot_pm.service || true
  pgrep -a -f "[a]imrt_main_hal|[a]3_deploy_onnx_ref_pingpong" || true
'

tmux ls 2>&1 || true
```

如果是部分启动后 `FAILED`，并且 UI 提供 managed recovery，先支撑机器人，再用
`KILL ALL & COLLECT` 清理这个受管 session。若错误明确写着 `UNMANAGED_*`，不要让
lifecycle 杀掉它；先查明旧进程是谁启动的。

### 13.3 Pelvis 或球的 3D 显示异常

Pelvis 标签不显示时，优先从 Laptop 按 10.5 进入 `hope` distrobox 检查。
不要在 **Laptop HOST** 直接 source `/opt/ros/jazzy/setup.bash`。也可以登录 **HDU**，
在 HDU 原生 shell 做下面的只读检查：

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=232

ros2 topic echo /hope/vendor/tf_ready --once
ros2 topic echo /hope/pelvis/text --once
ros2 topic info /hope/pelvis/marker
ros2 topic info /hope/pelvis/tf
```

正式 Layout 只含静态球桌 URDF，不含 `/hope/robot_description` 或 `/joint_states`；
8766 不转发原始 `/tf`、`/tf_static`，只接收 `/hope/pelvis/tf` 中的
`world -> pelvis_link`。球桌不显示时先检查 2.4 的 Laptop asset 服务；若看到
Pelvis 以外的机器人 link/frame，说明 Foxglove 还保留旧 Layout 副本或旧 8766
bridge 进程：重新部署并重启 control bridge，再按 9.3 重新导入正式 Layout，不能
只重连旧窗口。

球不显示时，先把球放入 Motive 捕捉范围，再在 **HDU 原生 shell** 执行：

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=232

ros2 topic info /poses
timeout 10s ros2 topic echo /hope/ball/marker --once
```

收到的 Marker 应为 `type: 2`（sphere）、`scale` 三轴均为 `0.04`，位置来自
`/poses[0]`。Marker lifetime 为 0.2 秒；球丢失或移出捕捉范围后主动消失是正确行为。
如果 `/poses` 没有新消息，应先修复 Laptop OptiTrack bridge/Motive 可见性，不是 3D
Layout 的问题。

不要为了让 Pelvis 标签出现而伪造 `world -> odom` identity TF。

### 13.4 Ready 已是 MOTION，但喂球后仍不挥拍

先区分 Runner mode 与 Planner command：`Ready → MOTION` 只让策略进入等待来球状态；
实际挥拍还需要 Laptop 对当前入射球生成一个 `/ball/flight_packet`，HDU Planner 再
发布 `/racket/command_flat`。NTP、TF、marker freshness、heading、station readiness、
target support 和 nominal late cutoff 在 policy-native 现场路径中只做 audit，不会挡住
一个有限且 strike time 仍在未来的 Planner command。

在 **Laptop HOST** 执行；这条命令自己进入 `hope` distrobox：

```bash
distrobox enter hope -- bash -lc '
  set -eo pipefail
  source /opt/ros/jazzy/setup.bash
  source $HOPE_ROOT/hope_ws/install/local_setup.bash
  DDS=$HOPE_ROOT/hope_ws/install/hope_bringup/lib/hope_bringup/with_fastdds_unicast.sh
  "$DDS" --domain-id 232 --peer ${HDU_IP} -- bash -lc "
    ros2 node list | grep -E \"/hope_ball_flight_packetizer|/optitrack_mct_relay\"
    ros2 topic info /poses -v
    ros2 topic info /ball/flight_packet -v
  "
'
```

再查看本次 Laptop 日志：

```bash
cd $HOPE_ROOT
SESSION_ID="$(cat real_logs/.active_session_id)"
grep -E 'Flight Packet producer started|ball.*acquired|ball.*lost|ERROR|FATAL' \
  "real_logs/$SESSION_ID/laptop/laptop_bridge.log" | tail -n 100
tail -n 50 "real_logs/$SESSION_ID/laptop/flight_packets.csv" 2>/dev/null || true
```

在 **HDU 原生 shell** 查看 Planner 是否收到 packet 并输出 command：

```bash
source /opt/ros/jazzy/setup.bash
source $HOME/hope_ws/install/local_setup.bash
source $HOME/hope_ws/install_model21800_fix/local_setup.bash
export ROS_DOMAIN_ID=232

ros2 topic info /ball/flight_packet -v
ros2 topic info /racket/command_flat -v

SESSION_ID="$(cat /tmp/hope_model21800_session_id)"
tail -n 120 "/tmp/hope_real/$SESSION_ID/hdu/planner.log"
```

判读顺序：

- packet publisher 为 0：Laptop overlay/helper 仍是旧版，重新执行 2.2、2.3、5、6.2；
- publisher 为 1 但 `flight_packets.csv` 没有数据：检查 Motive 球名 `Ball`、`/poses[0]`
  和实际喂球是否形成由对侧过网的完整 incoming flight；
- Planner 日志 `packets=0/0`：检查 Fast DDS peer/Domain 232；
- `packets` 增加但 `/racket/command_flat` 仍无发布：保留 Planner 日志与 CSV，这才是
  Planner 算法失败，不能靠删除 Runner gate 修复；
- `/racket/command_flat` 已发布而 Runner 仍 `PLANNER: no_command`：检查 HDU→MDU
  transport 和 Runner package/YAML。

### 13.5 Marker 显示 0/10 或 NO FRESH DATA

正常启动后，Laptop marker 日志位于：

```bash
cd $HOPE_ROOT
SESSION_ID="$(cat real_logs/.active_session_id)"
tail -n 100 "real_logs/$SESSION_ID/laptop/marker_monitor.log"
```

同时确认 Motive 中 P1 名称正确、marker 实际可见，并检查 Laptop bridge 日志。Marker
计数只统计 P1 的真实、非遮挡、point-cloud-solved 样本，不统计 Motive model definition
中的理论 marker。

### 13.6 CPU 持续接近 100%

面板中的百分比是 HDU 所有 CPU 的 aggregate busy time；例如 `96%` 表示采样窗口内
整机确实接近饱和，不是把多核百分比错误相加。`TOP CPU PROCESS` 显示同一秒内 CPU
tick 增量最大的进程，并同时给出：

- `core=...%`：类似 `top` 的单核尺度，一个核跑满约为 100%；
- `system=...%`：该进程占整台机器全部 CPU 时间的比例。

新监控只每秒扫描一次 `/proc/<pid>/stat`，不会先入为主降低 Runner、TF 或 ROS 发布
频率。部署后先用面板归因；也可在 **HDU 原生 shell** 做只读交叉检查：

```bash
top -b -n 3 -d 1 -o %CPU | head -n 40
ps -eo pid,comm,pcpu,pmem --sort=-pcpu | head -n 20
journalctl -u hope-monitor.service -n 100 --no-pager
```

如果最高进程每次都不同而 aggregate 持续高，再检查 IRQ、I/O wait、温度和频率；
在拿到进程证据前不要盲目降低 Runner 控制频率。

---

## 14. 当前验证边界

本仓库的单元测试、shell 解析、systemd 校验和 UI 打包只能证明接口、配置限制和
固定命令构造。第一次三机部署仍必须完成一次有人值守 rehearsal，重点检查：

- 四方向 SSH key；
- MDU sudoers；
- 三台机器的 `tmux`；
- HDU 8766 服务发现；
- MDU Runner package/hash；
- PASSIVE、PD_STAND、MOTION 的真实反馈；
- Calibration 的新 JSON/receipt/world-pelvis base packet，以及独立 x_hit ack；
- CPU top-process 归因是否与 HDU `top` 一致；
- E-Stop 只做接口/ready/latch 的只读核对，不为测试而实际触发；
- `KILL ALL & COLLECT` 后 `agibot_pm` 恢复及日志完整性。

详细设计与安全边界见：

- `docs/operations/foxglove_lifecycle.md`
- `docs/operations/foxglove_operator_interface.md`
- `docs/operations/foxglove_runner_integration.md`
