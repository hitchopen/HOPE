# Foxglove 动捕系统选择

Lifecycle schema 6 提供三个选项：无动捕、Motive（NatNet）、
青瞳 / CMTracker（VRPN）。后两者输出相同的 HOPE topics，但每次只能启动一套。
地址、单位、资产导入不同，不能靠看到 `/poses` 自动判断来源。

## 操作

1. 停止原 Laptop 动捕。经本页命令启动的两套系统沿用 `hope-optitrack` tmux
   会话；执行 `tmux send-keys -t hope-optitrack C-c`，确认进程退出。
   手工启动的独立 launch 则在原终端 Ctrl-C。Runner 必须处于 lifecycle STOPPED
   才能确认新配置。
2. 在 **MOCAP SYSTEM · 动捕系统** 选择系统。填写对应地址；青瞳还需核对
   VRPN 端口（默认 3883）和服务器输出单位（毫米或米）。选择 P1/P2；
   青瞳另选 V2/V4 裤子。Motive 使用实时 ModelDef，不套用 VRPN 裤子 JSON。
3. 按 **CONFIRM CONFIG**。界面只按已确认配置生成启动命令；编辑中的草稿不
   生成命令。夹爪和裤子配置与动捕系统独立保存。
4. 在所选 Laptop 环境的仓库根目录运行显示的命令。Linux 使用 HOST，Mac 使用已配置的 operator container。它调用
   `foxglove/helpers/hope-lifecycle start-laptop`，Linux 在 hope distrobox 内、Mac 在 direct runtime 中启动所选链路。
   检测到已有 Motive/VRPN 进程时拒绝再启动第二套。只改变选项不会切换正在
   运行的进程；更换 setup 后需停止原链路并重新生成、执行命令。
5. `START SYSTEM` 仍启动 Runner/HAL/Planner 流程；动捕保持单独启动。
   `KILL ALL + COLLECT` 不负责关闭这个独立动捕会话。日志写入
   `real_logs/<session>/laptop/laptop_bridge.log`。若 Runner 已有 session，界面复用
   它；否则启动命令生成独立的时间戳 session。

选择 **无动捕** 用于 Runner-only 测试。该选项不启动任何接收端，不提供实时球
或 pelvis 数据；切换到无动捕前仍须按第 1 步停止旧动捕。

## Motive / NatNet

- 导入 pelvis 对齐的 `.motive` 资产，保持其原点和轴向。
- 刚体命名 `UCB_P1` / `UCB_P2`，球名 `Ball`；Streaming Up Axis 为 Z。
- 开启 NatNet Rigid Bodies；Labeled Markers 用于实时可见点数审计。
- 启动现有 `optitrack_hope_bridge.launch.py` / `motive_live` 链路与 marker
  monitor。原有 Motive 场地配置继续适用，不能用于推断青瞳场地坐标。
- 原有 NatNet unicast/multicast、端口和网卡设置仍按该链路的配置进行；
  Foxglove 的协议选项不修改 Motive 软件。

## 青瞳 / CMTracker / VRPN

- 按实物选择 V2（10 点）或 V4（17 点贴纸），导入
  `hope_ws/src/hope_bringup/config/pants_profiles/a3_<v2或v4>_<P1或P2>_ground_truth.json`。
  详细坐标约定见 [裤子与 CMTracker 设置](vrpn_pants_profiles.md)。
- 名称仍为 `UCB_P1` / `UCB_P2` 与 `Ball`。保留 pelvis_link 原点及
  X前/Y左/Z上轴向，核对服务器输出单位。毫米流乘 0.001，米流乘 1.0。
- 启动 `avatar_pro_hope_bridge.launch.py`，不启动 NatNet 或其 marker monitor。
  VRPN 不提供在线贴纸点数或 ModelDef，界面显示 `POSE ONLY` / `N/A`。
- 接收端核对 Foxglove 原子快照中的系统、地址、端口和单位。选择错误、参数
  不匹配或快照超过 3 秒未更新时停止转发，等待修正并重启。
- 默认不确认资产原点或世界坐标，base 输出 valid=0，flight packetizer 关闭。
  完成实际导入和场地检查后，可在 Laptop 的
  `~/.config/hope-foxglove/lifecycle.env` 中设置以下两项，再重新启动动捕：

  ```bash
  HOPE_VRPN_PELVIS_ASSET_CONFIRMED=true
  HOPE_VRPN_WORLD_FRAME_CONFIRMED=true
  ```

  前者确认当前所选资产保留 pelvis 原点/轴向；后者确认 VRPN 世界为右手系、
  Z向上、桌面 z=0、桌长 x=[0,2.74]、桌宽 y=[-1.525,0]。
  世界确认还会开启 flight packetizer。换资产或场地后先恢复 false 并重新检查。
  这两项是操作者确认，不是程序测得的校准结果。

## 更新与旧配置

需配套更新 Console、HDU `hope_lifecycle_core.py` / `hope_lifecycle_supervisor.py`、
Laptop `hope-lifecycle` / `hope-laptop-container-runtime` 及重建 `hope_bringup`。
VRPN 需要 `vrpn_mocap`；完整球数据链还需 `hope_planner_cpp` / `hope_msgs`。

schema 1–4 保留网络地址、桌边、夹爪和已存在的裤子配置；新增动捕系统默认
**无动捕**，要求操作者明确选择。旧 `motive_ip` 曾被用来填 VRPN 地址，不能据此
自动推断协议或搬入 VRPN 字段。首次 CONFIRM CONFIG 原子写入 schema 6 并增加
revision。只替换扩展而未更新 supervisor 会导致新配置请求被旧服务拒绝。


主线 schema 3/4 依靠 operator_profile 区分于现场 schema 3/4，保留 Mac/Linux 与夹爪；现场 schema 4/5 保留裤子和已有动捕选项。旧五、六、七字段客户端更新网络时保留新选项。不会自动安装、启用或重启服务。

可选的只读 mocap-config-transport 服务用于 HDU 网卡更换后重建 DDS participant；它沿用 /etc/hope-foxglove/network.env 和 fastdds_bridge_profile.xml，不固定现场 IP。它只转发原始配置消息，不伪造新鲜心跳。按需安装，正常网络不需要此服务。
