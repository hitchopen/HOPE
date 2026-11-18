# V2 / V4 裤子与 CMTracker VRPN（2026-10-27）

V4 对应同一个 17 孔 v3 壳体（前 7、后 10 个安装位），现在 P1/P2 各安装
12 张贴纸，使用不同布局。CAD 目录仍叫 `agibot/mocap_sticker_shell_v3`，
打印件不变；它不同于旧的十球 V3 标记布局。

## 文件与原点

导入 [`P1.json`](../../agibot/mocap_sticker_shell_v3/vrpn_asset/P1.json)
和 [`P2.json`](../../agibot/mocap_sticker_shell_v3/vrpn_asset/P2.json)，内部名称为
`UCB_P1` / `UCB_P2`，ID 为 1 / 2。每份文件与其更新后的 V4 运行时文件
逐字节一致。请按[安装 PDF](../../agibot/mocap_sticker_shell_v3/output/pdf/A3_marker_shell_v3_instructions.pdf)
贴满所选 12 点，另五点保持不反光。P1 是指定点位；P2 通过 CAD 筛选计算。
两种布局必然共用至少 7 点，因此仅剩共用点可见时仍可能混淆，不能保证任意
视角或遮挡下正确识别。P2 独有点都在背面，必须实测背面覆盖和双机器人重新捕获。
[`vrpn_asset/README.md`](../../agibot/mocap_sticker_shell_v3/vrpn_asset/README.md)
提供导入、骨盆原点、轴向和单位检查步骤；不要导入 `asset_manifest.json`。

运行时配置位于 `hope_ws/src/hope_bringup/config/pants_profiles/`。
V4 的目录、两份文件及读取代码必须一同更新；V2 保持原样：

| 裤子版本 | P1 裤子刚体 | P2 裤子刚体 |
| --- | --- | --- |
| V2，10 点 | `a3_v2_P1_ground_truth.json` | `a3_v2_P2_ground_truth.json` |
| V4，17 孔壳体、每侧 12 点 | `a3_v4_P1_ground_truth.json` | `a3_v4_P2_ground_truth.json` |

这些文件沿用桌面 `a3.json` 的 CMTracker JSON 结构，`markerPose` 单位为
**毫米**，数值直接来自 pelvis_link 下的 CAD ground truth：不减点集中心，
不添加旧刚体偏移。原点是 URDF 的 `pelvis_link`，X 向前、Y 向左、Z 向上。
V4 使用贴纸外表面的光学中心（厚度 0.20 mm），不是安装座或反光球球心。
`profiles.json` 按 P1/P2 分别记录 V4 点序、坐标及 SHA-256。旧的相同 17 点
V4 配置不再匹配这两种安装。对应的
[Motive P1/P2 文件](../../agibot/mocap_sticker_shell_v3/motive_asset/README.md)
使用相同的 12 点子集和 CAD 坐标，但存储为 Motive 原生 Y-up、米单位。
旧 17 点 Motive 文件仅保留为历史证据，不能用于这两种安装。

原文件 `/home/user/Desktop/a3.json` 保持不变。它定义 `SJTU_a3`，10 点均值为
零，是旧 V2 点集中心。自动对应到 V2 CAD 后 RMS 为 1.7105 mm；拟合得到
pelvis 原点在该刚体坐标中约为 `(6.667, -0.319, 148.870)` mm。
这仅用于诊断原文件，**没有作为运行时校准写入**。

新文件的数据检查已通过；尚未做 CMTracker 导入再导出的回读验证。
导入后须确认软件没有自动重新居中或旋转刚体，且实际贴纸安装对应这套 CAD。
桌面旧文件不能仅重命名后当作 pelvis 对齐文件使用。

## 刚体命名

大小写必须一致：本方机器人 `UCB_P1` 或 `UCB_P2`，球 `Ball`。
单机器人不需要同时建两个机器人刚体。`PPT` 是可选球桌刚体；球拍没有必需名称。
两份发布文件已分别命名 `UCB_P1` / `UCB_P2`，确认 ID 与项目其他刚体不冲突。
不要重新居中、旋转或镜像标记点。不要同时导入同侧的 V2 和 V4。

## Foxglove 切换

Console 1.8.15 起，先选择 **MOCAP SYSTEM → 青瞳 / CMTracker · VRPN**，
填写独立的 VRPN 地址、端口和单位，再选择裤子。完整启动及升级步骤见
[Foxglove 动捕 setup](foxglove_mocap_setup.md)。

在停止状态选择 **MARKER PANTS · 动捕裤子 → V2 / V4 → CONFIRM CONFIG**。
同时在 CMTracker 导入匹配的文件。切换选项不会远程修改 CMTracker。
旧配置升级默认 V2，保留已有网络、裤子标识和夹爪选择。

P1/P2 仅标识两套裤子贴纸布局和 UCB_P1/UCB_P2 刚体，与机器人站位无关。
旧字段名 `table_side` 为兼容安装保留，只选择裤子。接收器根据所选刚体的新鲜原始
位置自动锁定实际桌端，与 P1/P2 裤子名称无关。对面桌端统一应用
`(x,y,z) → (2.74-x,-1.525-y,z)`，姿态左乘绕 Z 轴 180° 的旋转；
机器人、球、球姿态及 Planner 输入使用同一变换。例如 `x=3.431 m, yaw=170°`
变为 `x=-0.691 m, yaw=-10°`，固定击球平面继续按 canonical HOME 加模型偏移。

VRPN observation schema 3 明确携带 `coordinate_frame=hope_table_canonical` 和
`physical_table_side`，base adapter 按实际桌端计算世界坐标 receipt，拒绝旧 schema
及同一配置 revision 内的桌端改变。裤子资产哈希仍按裤子名称校验。
`/hope/mocap/players/*` 保持原始 `hope_table_global`，供选择裤子和显示全场站位。
没有新鲜所选刚体时不猜桌端、不转发策略用球流；桌中心 ±0.25 m 内不建立桌端。
桌端一旦锁定，本场不自动翻转；发现跨端则停止该坐标链路的有效输出，必须停止并
开始新会话，重新建立 HOME。更新接收器和 base adapter 后也必须开始新会话，
禁止将旧 HOME、旧 flight 与新坐标混用。

Supervisor 发布原子配置 `/hope/lifecycle/config/mocap`，包括裤子版本、裤子标识和
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

生产环境沿用 Laptop → HDU 的 DDS 配置：domain 232、HDU peer 10.42.20.10。
可用已安装的 `with_fastdds_unicast.sh --domain-id 232 --peer 10.42.20.10 --`
包裹上述 `ros2 launch`。`server` 是 CMTracker 电脑，和 HDU peer 不是同一地址。
正常运行只能保留一个动捕输入栈，避免同时启动 NatNet 和 VRPN 的同名输出。

没有 HDU / Foxglove 时，可独立验证：

```bash
ros2 launch hope_bringup avatar_pro_hope_bridge.launch.py \
  server:=<CMTracker_IP> follow_lifecycle_config:=false pants_profile:=v4 table_side:=P1
```

默认可以观察 `/UCB_P1/pose`、`/UCB_P2/pose`、`/ball/point` 和 `/poses`。
`position_scale:=0.001` 适用于毫米流；服务器输出米时改为 `1.0`。
源代码的 VRPN client 使用 Laptop 接收时间作 PoseStamped 时间戳。

**完成现场球桌坐标检查后**，再加
`world_frame_confirmed:=true`，才输出有效的
schema-2 `/a3/base_pose_laptop_flat`。缺少确认时输出 valid=0；旧 JSON 不会被加载。
`world_frame_confirmed` 表示 VRPN 已输出右手系、Z 向上、桌面 z=0、
桌长 x=[0,2.74]、桌宽 y=[-1.525,0] 的标准坐标。
不要沿用 Motive 的 +90° 现场变换来猜测 VRPN 坐标。
此参数确认球桌世界坐标；资产选择、版本与哈希由接收链路自动校验，不再设置骨盆原点／轴向确认开关。

P1 / P2 仅选择裤子刚体；策略坐标根据实际桌端自动旋转并在本场锁定。`/a3/mocap/pelvis_pose` 保留真实骨盆高度；只有
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


## 2026-10-15 V2 现场定位修复（历史记录）

下述双确认环境变量是历史做法。当前只在 Console 保存世界坐标确认；schema 8 删除骨盆确认字段，旧 schema 7 迁移时保留世界确认并忽略已删除字段。

本次现场使用 `/home/user/Desktop/1015_measured/UCB_P1.json`（10 点实测 V2，
已变换到 pelvis），不是桌面根目录的 17 点 V4 文件。操作员确认 CMTracker
导入保留原点/轴向，并确认世界系为桌面 z=0、X 从 P1 到 P2、Y 向左、Z 向上后，
在 Laptop `~/.config/hope-foxglove/lifecycle.env` 持久化：

```bash
HOPE_VRPN_PELVIS_ASSET_CONFIRMED=true
HOPE_VRPN_WORLD_FRAME_CONFIRMED=true
```

这两个确认仅适用于已经核验的资产和现场坐标。重新建立刚体或世界坐标后需重新核验。
实测点已包含裤子到 pelvis 的对齐，不再额外叠加 149 mm；只有策略 base wire
加一次 0.76 m 的桌面到地面高度，`/a3/mocap/pelvis_pose` 保持桌面坐标。

按操作员要求，撤销此次新增的定位无效 MOTION 拒绝 gate：定位状态只保留
既有告警，不新增拒绝进入 MOTION 或自动退回 PD_STAND 的行为。
已确认的 V2 资产、世界坐标配置和动捕有效输出修复继续保留。

`refresh_x_hit` 仍为独立 Planner 操作，应在机器人就位且 Planner/base 有效后执行；
它不负责 pelvis 校准，也不能修复无效的 base packet。此次修复没有启动 Runner、
Planner 或实机 MOTION，没有调整时钟。
