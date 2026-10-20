# V2 / V4 裤子与 CMTracker VRPN（2026-10-15）

V4 是用户确认的 17 点贴纸裤子（前 7、后 10），原始 CAD 目录仍叫
`agibot/mocap_sticker_shell_v3`。它不同于旧的十球 V3 标记布局。

## 文件与原点

可导入的 CMTracker JSON 放在
`hope_ws/src/hope_bringup/config/pants_profiles/`：

| 裤子 | P1 侧机器人 | P2 侧机器人 |
| --- | --- | --- |
| V2，10 点 | `a3_v2_P1_ground_truth.json` | `a3_v2_P2_ground_truth.json` |
| V4，17 点贴纸 | `a3_v4_P1_ground_truth.json` | `a3_v4_P2_ground_truth.json` |

这些文件沿用桌面 `a3.json` 的 CMTracker JSON 结构，`markerPose` 单位为
**毫米**，数值直接来自 pelvis_link 下的 CAD ground truth：不减点集中心，
不添加旧刚体偏移。原点是 URDF 的 `pelvis_link`，X 向前、Y 向左、Z 向上。
V4 使用贴纸外表面的光学中心（厚度 0.20 mm），不是安装座或反光球球心。
`profiles.json` 记录点序、来源、数量和每份导入文件的 SHA-256。

新文件的数据检查已通过；尚未做 CMTracker 导入再导出的回读验证。
导入后须确认软件没有自动重新居中或旋转刚体，且实际贴纸安装对应这套 CAD。
桌面旧文件不能仅重命名后当作 pelvis 对齐文件使用。

## 刚体命名

大小写必须一致：本方机器人 `UCB_P1` 或 `UCB_P2`，球 `Ball`。
单机器人不需要同时建两个机器人刚体。`PPT` 是可选球桌刚体；球拍没有必需名称。
导入文件内已经使用 `UCB_P1` / `UCB_P2`。不要同时导入同侧的 V2 和 V4。

## Foxglove 切换

Console 动捕选项中，先选择 **MOCAP SYSTEM → 青瞳 / CMTracker · VRPN**，
填写独立的 VRPN 地址、端口和单位，再选择裤子。完整启动及升级步骤见
[Foxglove 动捕 setup](foxglove_mocap_setup.md)。

在停止状态选择 **MARKER PANTS · 动捕裤子 → V2 / V4 → CONFIRM CONFIG**。
同时在 CMTracker 导入匹配的文件。切换选项不会远程修改 CMTracker。
旧配置升级默认 V2，保留已有网络、桌边和夹爪选择。

Supervisor 发布原子配置 `/hope/lifecycle/config/mocap`，包括裤子、桌边和
revision；VRPN 在处理下一条数据前同时应用这些字段，并清空旧缓存。
配置超过 3 秒收不到时停止转发，防止断线后继续使用未知选择。
VRPN 只提供位姿，没有 ModelDef，状态会明确写出 `live geometry NOT verified`，
不会伪造贴纸数、实测安装校准或在线匹配成功。
Motive / NatNet 保留原 live asset 行为；此选择器接入的是 VRPN 路径。

## 运行

在已有 ROS Jazzy / hope 工作区环境中启动：

```bash
ros2 launch hope_bringup avatar_pro_hope_bridge.launch.py \
  server:=<CMTracker_IP> position_scale:=0.001 follow_lifecycle_config:=true
```

生产环境沿用 Laptop → HDU 的 DDS 配置：domain 232、已确认的 HDU peer。
可用已安装的 `with_fastdds_unicast.sh --domain-id 232 --peer <HDU_IP> --`
包裹上述 `ros2 launch`。`server` 是 CMTracker 电脑，和 HDU peer 不是同一地址。
正常运行只能保留一个动捕输入栈，避免同时启动 NatNet 和 VRPN 的同名输出。

没有 HDU / Foxglove 时，可独立验证：

```bash
ros2 launch hope_bringup avatar_pro_hope_bridge.launch.py \
  server:=<CMTracker_IP> follow_lifecycle_config:=false pants_profile:=v4 table_side:=P1
```

默认可以观察 `/UCB_P1/pose`、`/UCB_P2/pose`、`/ball/point` 和 `/poses`。
`position_scale:=0.001` 适用于毫米流；服务器输出米时改为 `1.0`。
VRPN client 保留并验证服务器 report 的源时间戳；CMTracker 主机必须与接收端
处于同一 NTP 时间基准，不能把 Laptop 到达时间当作相机采样时间。

**完成导入原点/轴向检查和现场球桌坐标检查后**，再加
`pelvis_asset_confirmed:=true world_frame_confirmed:=true`，才输出有效的
schema-2 `/a3/base_pose_laptop_flat`。缺少确认时输出 valid=0；旧 JSON 不会被加载。
`world_frame_confirmed` 表示 VRPN 已输出右手系、Z 向上、桌面 z=0、
桌长 x=[0,2.74]、桌宽 y=[-1.525,0] 的标准坐标。
不要沿用 Motive 的 +90° 现场变换来猜测 VRPN 坐标。
这两个参数是操作者对名义坐标约定的确认，不是实测安装校准证明。

P2 模式对球和所有刚体统一执行 `(x,y,z) → (2.74-x,-1.525-y,z)`，
姿态同步绕 Z 转 180°。`/a3/mocap/pelvis_pose` 保留真实骨盆高度；只有
schema-2 的策略坐标加 0.76 m。HDU 后续按既有链路改写时间戳，适配现有 Runner。
陈旧/重复时间戳、非法姿态、异常位姿跳变和旧配置回放均被拒绝。

`start_flight_packetizer` 默认跟随 `world_frame_confirmed`：确认世界坐标后自动
启动 `/poses → /ball/flight_packet` 打包器。完整运行可显式设
`start_flight_packetizer:=true debug_session_id:=<当前会话ID>`；仅检查定位时才显式
设 `start_flight_packetizer:=false`。Planner 使用 flight-packet 模式时，Foxglove
能显示 `/poses` 并不代表已有击球输入，必须检查 `/ball/flight_packet` 发布者和
Planner 的 packet 计数。此 launch 不启动 Runner；打包器恢复后，已有 Planner
可重新输出击球目标，调试时应保持 Runner 为 PASSIVE。
`start_world` 仅保留旧命令兼容性，不再加载 Motive 场地参数；新的骨盆适配器由
`start_base_pose` 控制，默认开启。
