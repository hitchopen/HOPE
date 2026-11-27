# Mac 独立现场机

本版本从 HOPE `reposition@95b05239` 移植。Apple Silicon Mac 上运行 Foxglove Desktop，同一台 Mac 里的 ARM64 Ubuntu 24.04 虚拟机承担 VRPN、ROS 2 Jazzy、三视图和 MCAP 录制。给虚拟机的现场网卡开启有线桥接，分配一个独立的场地 LAN 地址。HDU 必须能把 DDS UDP 数据发回这个地址。准备好虚拟机内的 Docker Engine、Git、Python 3 和 OpenSSH，先运行 `docker info` 确认可用。Mac 和虚拟机各需要本版本的文件。

在 Mac 和虚拟机分别取得发布的源码。已有现场 checkout 可保留在原目录。

```sh
git clone --depth 1 --branch codex/mac-field-takeover-20261127 https://github.com/hitchopen/HOPE.git HOPE-mac
cd HOPE-mac
```

### 1. 在 Mac 安装界面

在 Foxglove Desktop 打开 `foxglove/extensions/hope-a3-console/hopeopen.hope-a3-console-2.2.0-beta.10.field.14.foxe`，重新加载窗口。在 Layouts 菜单选择 **Import from file**，导入 `foxglove/layouts/field_three_view.json`。布局包含现场三视图、实时遥测和 Console。目标机器人地址由本次场地连接确定。

在 Mac 用户的 SSH 配置中把虚拟机命名为 `hope-field`，或用 `HOPE_FIELD_GUEST` 指定自己的 SSH 别名。先单独运行 `ssh hope-field` 核对虚拟机主机指纹并确认可以登录普通用户。桌面端转发使用这条 SSH 连接。HDU 的受限 helper 通道单独建立。

### 2. 在虚拟机内构建并配置

在本版本源码目录执行：

```sh
python3 scripts/macos/hope.py build-operator --arch arm64 --jobs 2
install -d -m 700 ~/.config/hope-macos
test -e ~/.config/hope-macos/operator.json || cp scripts/macos/operator.example.json ~/.config/hope-macos/operator.json
```

编辑 `operator.json`，加入 `"network_mode": "field-vm"`。将 `root` 设为虚拟机内本次构建的源码绝对路径。`mac_ip` 填虚拟机访问 HDU 的有线源地址，可以用 `ip -4 route get <HDU-IP>` 查看。`hdu_ip`、`mdu_ip` 和用于切回的 `linux_ip` 均填写本场地实际值。`laptop_user` 和 `laptop_helper` 对应 HDU supervisor 的 Mac profile，默认是 `operator` 和 `/home/operator/.local/libexec/hope-lifecycle`。

在虚拟机的 `~/.config/hope-macos/` 放置已获授权的 `client_key`（权限 0600）、经核验的 HDU/MDU `known_hosts`，以及 HDU supervisor 对外登录所用的公钥 `hdu.pub`。配置目录、凭据和机器人标定保留在现场机。虚拟机通过现场时钟源同步时间，并检查到动捕服务器和 HDU 的有线路由。

本版本 operator 镜像使用 contract 5。升级已有容器时，先在 STOPPED 窗口切回可用的 Linux profile，按原有 Mac 部署指南关闭旧容器。保留 home、state 和 ROS build 三个卷。全新的虚拟机可直接执行下一步。

### 3. 登记并切换

首次接管前，HDU 的 `hope_connection_identity.py` 和 `hope_lifecycle_supervisor.py` 需要使用本次 Mac 身份校验。由现场负责人根据当前安装目录在 STOPPED 维护窗口更新这两份配套源码并重载对应 supervisor。已经安装相同源码的 HDU 可以直接使用。Mac 构建和分支发布仅提供源码。原 Linux 机器的身份登记保留，用于切回。

1. 在虚拟机运行 `python3 scripts/macos/operator.py up`。确认容器完成初始化后，在 STOPPED 状态运行 `python3 scripts/macos/operator.py bind`。这一步登记 Mac helper 的 SSH 主机密钥与反向隧道，并更新 HDU 的 DDS peer。所需的 HDU 管理员权限与 Mac 用户权限独立。
2. 在 Mac 的源码目录运行 `bash scripts/macos/open_field.command commission <HDU-IP>`，保持这个终端和 Foxglove 数据源打开。此时有一条从虚拟机现场地址到 HDU 8766 的真实 Foxglove 连接。关闭其他电脑对 HDU 8766 的控制连接，保证目标地址可以唯一核对。
3. 在虚拟机运行 `python3 scripts/macos/operator.py select-mac`，随后运行 `python3 scripts/macos/operator.py status`。核对 profile、Laptop IP、青瞳 VRPN 地址/端口、单位、裤子和场地侧，并在 Console 查看配置回读。修改 IP 时，HDU 会检查机器人、启动标识、配置版本和已登记的目标机。需要返还给 Linux 时，先在注册的 Linux 机器上打开到 HDU 8766 的唯一直接连接，在 STOPPED 状态执行 `select-linux`。
4. 关闭首次登记用的 Foxglove 连接，在其终端按 Ctrl-C。在 Mac 运行 `bash scripts/macos/open_field.command view`，选择三视图布局。这个终端负责把虚拟机 loopback 上的 8767（Foxglove）、8768（球状态）和 8769（3D 模型）转发到 Mac。三视图使用已安装模型资源。容器跟随当前 Mac 的现场配置启动显示进程，首次收到匹配的配置前 8767 尚未开始监听。

本地球状态首次配对时，等 packetizer 在容器内生成控制密钥，再按 [本地球状态配对步骤](laptop_ball_states.md) 在虚拟机内导出权限为 0600 的文件。通过已核验的虚拟机 SSH 连接把这个文件复制到 Mac 的用户私有目录，在 Console 的 **Pair local ball controls** 中选择它。

### 4. 验收采集和录制

在 Mac 上使用当前场地的青瞳 VRPN 配置。先检查虚拟机内收到球和对应 P1/P2 的骨盆样本且时间戳递增，再核对 HDU 在 ROS domain 232 收到相同数据流。确认时钟偏差满足场地要求，Console 显示配置已应用，三视图可加载模型并持续刷新。

在获准的现场会话内使用现有 Record 开关录一小段，再关闭、等待文件收尾并 Collect。虚拟机源码目录的 `real_logs/<session>/laptop/` 保存本机录制。用 `ros2 bag info <ros_raw目录>` 核对 MCAP 格式、球、骨盆、消息数和时长，在 Foxglove 打开同一段 MCAP 回放，核对消息时间和机器人/球的变化。完整会话同时核对 HDU、MDU 和 Laptop 的同一 session。Mac 可通过 SSH/SFTP 取回这些文件。

`up` 报路由不一致时，检查虚拟机桥接的物理网卡和 `mac_ip`。配置回读失败时，检查容器的 `docker logs`、三视图的监听进程、主机密钥以及 HDU 返回的具体原因。原生 Docker Desktop 的默认 NAT 模式仍用于原有的构建和辅助操作。
