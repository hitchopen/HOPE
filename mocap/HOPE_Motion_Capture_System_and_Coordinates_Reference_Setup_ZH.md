# HOPE 乒乓球竞技场动作捕捉系统参考设计

**v0.7** — 2026-07-21

> **场地参考文档。** 本文档记录参考场地配置与仓库内已实现的传输路径。
> 现行的坐标系与话题契约以 [`mocap/README.md`](README.md) 为准；仓库内实际使用的路径为
> OptiTrack/NatNet 的 `motion_capture_tracking` + `optitrack_mct_relay`，以及供青瞳使用的
> vendored `vrpn_mocap`。索引见
> [`REFERENCE_DOCS.md`](../REFERENCE_DOCS.md)。

---

## 1  兼容的动作捕捉系统

本参考设计文档构建了一套兼容多种主流动作捕捉系统的参考方案，重点覆盖 **OptiTrack**、**Vicon** 与 **青瞳视觉（CHINGMU）** 三大常用品牌，并预期可进一步兼容 `motion_capture_tracking` 库所支持的其他基于标记点（marker）的动捕品牌，包括 Qualisys、NOKOV、VRPN、FZMotion 以及 Motion Analysis。各品牌的相机硬件与厂商软件不尽相同——例如 OptiTrack 配合 Motive 与 NatNet 协议、Vicon 配合 Vicon Tracker、青瞳配合 CMTracker/CMAvatar 并支持 VRPN、TrackD、DTrack、OpenVR 及原生 LiveStream 等协议，且各家均提供 C/C++、Python、ROS 等 SDK——但本设计将它们统一到同一套 ROS 2 REP 103 坐标系与 `/poses` + `/tf` 话题接口之下。比赛期间，场地流式传输具名 **6 自由度刚体** `Ball`、`P1` 与 `P2`；规划器要求 `/poses` 的索引 0 为 `Ball`，默认 VRPN bringup 只聚合该球位姿。第四个资产 `Table` 仅用于场地搭建/标定，且仅出现在训练数据录制中——比赛期间**不**跟踪、也**不**上报。仓库内两条路径按传输协议区分：**OptiTrack/Motive 使用 NatNet**，经 `motion_capture_tracking` 与 `optitrack_mct_relay`；**青瞳使用 VRPN**，经 `vrpn_mocap` 与 `pose_to_posearray`。两者最终生成同一 HOPE 规划器契约；细节见第 6 节。

对于 HOPE 参考设计，最低规格为：

- 至少 **8 台**相机，布置以覆盖整个球台体积，并在每位选手一侧留出 1.5 m 余量
- 相机帧率 **≥ 300 Hz**（可胜任球速超过 5 m/s 的竞技性球体跟踪）
- 在跟踪体积内达到亚毫米级的重建精度
- 小球**必须**提供稳定的刚体建模与追踪——即厂商认可的 `Ball` 刚体资产（第 5 节），并已验证高速跟踪、遮挡后重捕获与 ID 稳定性。单点/未标记标记点的球体跟踪不满足本参考设计。

---

## 2  环境标记点与坐标系的设置

为避免标定误差及平台潜在移动，最直接的做法是将动捕系统原点直接锚定在 `Table` 刚体上（旧版笔记称为 PPT）。然而，一个常见的混淆点在于：OptiTrack 的默认坐标系（Y 轴向上）与 ROS 2（Z 轴向上，REP 103）以及 Vicon（Z 轴向上）均不相同。**在本参考设计中，我们采用 ROS 2 REP 103 约定作为标准世界坐标系。**

### 2.1  标准世界坐标系（ROS 2 REP 103）

世界坐标系原点设置在 **球台台面近端左角**（从选手一 P1 的视角看）：

| 轴 | 方向 | 在台面上的范围 |
|------|-----------|------------------------|
| **X** | 向前——沿球台长度方向朝向选手二（P2） | 0 → +2.74 m |
| **Y** | 向左——沿球台宽度方向，从 P1 视角 | 0 → −1.525 m |
| **Z** | 向上——竖直方向 | 0 = 台面 |

该约定与配套文档《HOPE 7DOF 球拍基于模型的规划器参考设计》中所使用的坐标系**完全一致**，从而确保所有球体轨迹预测、球拍目标计算以及 ROS 2 话题消息共享同一套一致的坐标系。

该坐标系中的关键地标：

| 地标 | X (m) | Y (m) | Z (m) |
|----------|-------|-------|-------|
| 原点（P1 近端左角） | 0.0 | 0.0 | 0.0 |
| 球网中心线 | 1.37 | −0.7625 | 0.0 |
| P1 半场中心 | 0.685 | −0.7625 | 0.0 |
| P2 半场中心 | 2.055 | −0.7625 | 0.0 |
| 原点正下方地面 | 0.0 | 0.0 | −0.76 |
| 虚拟击球平面（规划器） | x = x_hit ≈ 0.0 | — | — |

台面占据区域为：`x ∈ [0, 2.74]`、`y ∈ [−1.525, 0]`、`z = 0`。

### 2.2  修正 OptiTrack 的默认坐标系

OptiTrack Motive 默认采用 **Y 轴向上** 的坐标系，这与 ROS 2 的 Z 轴向上约定不兼容。修正方法：

1. 在 Motive 中，导航至 **Edit → Settings → Streaming**（或打开 Data Streaming 面板）。
2. 在 **Advanced Network Options** 下，将 **Up Axis** 由 “Y Axis” 改为 **“Z Axis”**。该设置是仓库内 OptiTrack **NatNet** 路径（第 6.2 节）的必需条件，使输出直接符合 ROS 2 REP 103。比赛前仍须在经测量的球台地标处验证（第 6.5 节）：若源坐标系出现反向、偏移或其他不符合约定的情况，应在 HOPE relay 上游加入**完整位姿**变换，绝不可逐分量修改 pitch/yaw/roll。
3. 调整标定地面（ground plane）的朝向，使标定方块（calibration square）的长边对齐期望的 X 轴方向（朝向 P2）。这在标定杆（wand）流程中设定了世界坐标系的朝向。

Vicon Tracker 默认为 Z 轴向上，通常无需进行轴向修正。但应在地面标定过程中确认 X 轴沿球台长度方向指向 P2。

对于 **青瞳（Chingmu）CMTracker**，世界坐标系由地面标定步骤中 L 型架/标定方块的摆放位置确定，而向上轴（up axis）可在流式/导出设置中配置。请将向上轴设为 **Z**，使流式数据匹配 ROS 2 REP 103 约定，并摆放标定方块使其长边沿球台长度方向指向 P2。若某一特定的 CMTracker 安装版本只能以 Y 轴向上或其他非 REP-103 坐标系进行流式传输，**请勿**尝试围绕该坐标系重新标定——而应在第 6.4 节所述的 ROS 2 桥接节点中应用固定的轴向转换。

### 2.3  球台刚体定义（`Table`；旧称 `PPT`）

将反光标记点或回射贴片（至少 10 mm × 10 mm）贴附在球台的**外框**上。这些标记点共同构成一个刚体，在 Motive 或 CMTracker 中定义为资产 **`Table`**。旧版场地笔记可能将同一资产称为 `PPT`（Ping-Pong Table）；`Table` 是搭建场次与训练数据录制中的规范资产名。**`Table` 资产仅是搭建/标定工具——比赛期间不流式传输、不上报。**

放置要求：

- 在球台框架外缘以**非对称**配置贴附**至少 4 个标记点**。
- 将标记点放置在大多数相机位置可见、且在比赛过程中不会被选手、球网或球体遮挡之处。
- **不要将标记点放置在击球台面上**——它们会干扰球体的弹跳动力学，并可能降低刚体识别的可靠性。

`Table` 刚体的枢轴点（pivot point）必须设置在 **台面近端左角**（即原点），并使刚体局部坐标系与上文定义的世界坐标轴对齐。标定完成后，当球台静止且对齐正确时，`Table` 刚体应报告单位位姿（位置 ≈ [0, 0, 0]，姿态 ≈ [0, 0, 0, 1]）。

`Table` 刚体具有两个用途：

1. **原点锚定**——为所有其他被跟踪物体定义世界坐标系原点。
2. **场次间移动核验**——在搭建或核验场次中，`Table` 位姿偏离单位位姿即表明球台被碰撞或移位，需要重新标定。比赛期间球台被视为静态的、经测量的世界原点：不存在实时 `Table` 流，怀疑球台移位时应重新执行核验，而非依赖运行时话题。

---

## 3  被跟踪物体分类

比赛期间，动作捕捉系统流式传输具名刚体 **`Ball`、`P1` 与 `P2`**。`Table` 资产仅用于搭建/标定并存在于训练数据录制中（第 2.3 节）。球拍（paddle）在任何时候都明确**不被**任何方式跟踪。

### 3.1  球拍排除策略——球拍不由动作捕捉系统跟踪

**动作捕捉系统不得跟踪乒乓球拍（paddle）。** 不应在球拍上放置或贴附任何反光标记点或跟踪资产。这是一项与 HOPE 竞赛设计相一致的、刻意的架构性决策：

**理由：**

1. **正运动学推断。** 人形机器人必须依据自身的本体感受状态（关节编码器读数加上所申报的 URDF 根坐标系位姿；Unitree G1 为 `pelvis`，Agibot A3 为 `pelvis_link`），通过其手臂运动链的正运动学来推断球拍的 6 自由度位姿（位置与姿态）。动捕跟踪 P1/P2 标记点簇坐标系，再通过已标定的静态变换将其映射到机器人根坐标系。这考验机器人内部身体模型的精度，而这是任何现实世界操作任务的核心能力。

2. **末端执行器无外部传感。** 在本架构中，全身控制器（WBC）从规划器接收期望的球拍状态 `(p_intercept, v_racket, n_racket, t_strike)`，并使用其 RL 策略驱动 7 自由度手臂达到该状态。控制器从不接收来自动捕系统的实测球拍位姿。球拍的实际位置是机器人关节构型的涌现属性，而非外部测量量。

3. **竞赛公平性。** 外部跟踪球拍会提供绕过机器人控制挑战的闭环反馈。HOPE 竞赛要求每支队伍的人形机器人通过自身的运动学模型来展示自主的球拍控制。

4. **实际可靠性。** 在快速挥动的球拍上（手臂速度超过 3 m/s），标记点会遭受严重遮挡、运动模糊及离心脱落。将球拍排除在跟踪之外消除了一个脆弱的传感环节。

**执行：** 在竞赛布置过程中，裁判将核实球拍、机器人手部，以及超出机器人躯干/骨盆上最后一个被跟踪刚体标记点的腕部连杆上，均无回射材料。

**交叉引用：** 配套文档《HOPE 7DOF 球拍基于模型的规划器参考设计》（第 0.1 节）记录了规划器在无任何球拍位姿反馈的情况下输出期望球拍状态。配套文档《HOPE WBC 仿真训练参考设计》（第 2.8 节——球拍安装运动学）记录了从所申报的机器人根坐标系经 7 自由度手臂到 3D 打印固定球拍支架的完整正运动学（FK）链，包括确保仿真模型与物理支架匹配的 `T_mount` 标定流程。

### 3.2  被跟踪物体汇总

| 物体 ID | 资产类型 | 跟踪对象 | 标记点 | 跟踪模式 |
|-----------|-----------|-----------------|---------|---------------|
| **Table** | 刚体（仅搭建/标定——**比赛期间不流式传输**；位姿仅出现在训练数据中） | 乒乓球台框架与世界原点 | 球台外框上 ≥ 4 个非对称 | 厂商 6 自由度 |
| **P1** | 刚体（厂商跟踪） | 选手一标记点簇坐标系；静态标定至其申报的机器人根坐标系 | 躯干/骨盆板上 ≥ 4 个非对称 | 厂商 6 自由度 |
| **P2** | 刚体（厂商跟踪） | 选手二标记点簇坐标系；静态标定至其申报的机器人根坐标系 | 躯干/骨盆板上 ≥ 4 个非对称 | 厂商 6 自由度 |
| **Ball（球）** | 刚体（厂商跟踪） | 乒乓球球心位姿 | 厂商认可的刚体图案/标记点星座 | 厂商 6 自由度 |

比赛过程中，跟踪体积内不应出现未登记的回射图案。应为每个刚体使用唯一的非对称特征和稳定的资产名称，避免厂商解算器混淆资产身份。

---

## 4  人形机器人根坐标系标记点的设置

在本参考设计中，人形机器人通过**从其申报的 URDF 根坐标系出发**经手臂运动链的正运动学来推断球拍的 6 自由度位姿。动捕系统提供 P1/P2 标记点簇位姿，再通过已标定的静态变换映射到该根坐标系（Unitree G1 为 `pelvis`，Agibot A3 为 `pelvis_link`）。

### 4.1  机器人根坐标系约定——一般原则

人形机器人 URDF 根坐标系的名称和位置没有统一标准；平台可能使用 `base_link`、`pelvis`、`pelvis_link` 或其他名称。该约定因制造商、URDF 编写选择以及机器人预期的控制架构而异。业界常见以下三种模式：

**模式 A——骨盆根（双足运动最常见）。** 所申报的根坐标系位于骨盆连杆，即髋部板中心；腿部运动链由此向下分支、躯干链由此向上分支。这是 RL 训练运动控制器的常见选择，因为骨盆在行走中是最稳定的参考——它是全身动力学中的浮动基坐标系。Unitree G1 的 URDF 根连杆确切名称为 `pelvis`；Agibot A3 为 `pelvis_link`。

**模式 B——躯干/胸部根。** 一些平台将所申报的根坐标系置于上躯干或胸部，即腰关节之上。这在双足运动中较不常见（骨盆动力学更稳定），但可能出现在以操作为主的配置中——此时手臂是主要关注点，而腿部被视为移动底盘子系统。

**模式 C——腰关节根。** 一种折中方案，所申报的根坐标系位于腰关节本身——即腿部与躯干的交界处。在许多简单设计中，这与骨盆原点共位（模式 A）。在具有多自由度腰部关节的机器人中，腰关节位于骨盆之上，将其选作根坐标系会把根置于两个子系统之间。

**对于 HOPE 竞赛，关键要求是：**

> 所申报的机器人根坐标系必须锚定通往持拍手的正运动学链。规划器在世界坐标系中输出期望球拍状态；机器人的 WBC 必须计算从该根坐标系到球拍的手臂关节轨迹。

完整空间链为：`world → P1/P2（实时动捕）→ 所申报的机器人根坐标系（标定静态 TF）→ 腰关节 → 肩 → 肘 → 腕 → 拍尖（关节编码器）`。根坐标系与球拍之间的每个关节都必须配备编码器，且其读数可供机器人控制软件使用。

### 4.2  Unitree G1

下文以 Unitree G1 为一个机器人专属集成示例；所有参赛人形机器人均遵循相同的报名与坐标系要求。

| 属性 | 取值 |
|----------|-------|
| 所申报的 URDF 根连杆 | **`pelvis`** |
| 所申报的 URDF 根坐标系位置 | **骨盆**——腰部下躯干中心，大致位于两条髋偏航（hip yaw）关节轴的交点处 |
| 模式 | A（骨盆根） |
| 站立时骨盆高度 | 离地约 0.78 m（在 HOPE 坐标系中 z ≈ +0.02 m） |
| 机器人总高 | 1.27–1.32 m |
| 重量 | 含电池约 35 kg |
| 总自由度 | 23（基础版）至 43（带灵巧手的 EDU 版） |
| 手臂自由度 | 每臂 7 |
| 腰部自由度 | 1（偏航） |
| URDF 来源 | `github.com/unitreerobotics/unitree_ros` → `robots/g1_description` |
| 中间件 | 原生支持 ROS 2 |

运动学树由骨盆分支：

```
pelvis（所申报的 URDF 根坐标系）
├── left_hip_yaw_joint  → 左腿 (6 DOF)
├── right_hip_yaw_joint → 右腿 (6 DOF)
└── waist_yaw_joint     → 躯干 → 肩 → 肘 → 腕 (每臂 7 DOF)
```

**标记点放置：** 在固定于骨盆外壳的刚性板上贴附 4 标记点非对称簇。在 Motive 中将刚体枢轴点设置为骨盆原点（髋部板中心）。若标记点位于外壳表面，则标定一个数厘米的 Z 向静态 TF 偏移。

### 4.3  Agibot 远征 A3（Expedition A3）

远征 A3 是智元（Agibot）的运动型人形机器人。

| 属性 | 取值 |
|----------|-------|
| 所申报的 URDF 根连杆 | **`pelvis_link`**——已由 A3 URDF 确认 |
| 站立高度 | 全尺寸（约 1.75 m，据视频估计） |
| 重量 | 未公开披露 |
| 总自由度 | 未公开披露；描述为“高度拟人的全身自由度” |
| 手臂自由度 | 未公开披露（据 Agibot 平台谱系预期每臂 7 自由度） |
| 腰部自由度 | **多自由度柔性腰**——一项关键的区别性特征，专为镜像人体活动范围而设计，可实现复杂全身动作所需的旋转与摆动 |
| URDF 来源 | 仓库路径 `agibot/URDF/a3_t2d5/urdf/model.urdf` 与 `agibot/URDF/A3T2.5-URDF-std-pingpang/urdf/URDF-JOINT-LINK.urdf` |
| 中间件 | **AimRT**（Agibot 原生 C++20 运行时）；支持 ROS 2 协议桥接 |

**关键考量：**

1. **柔性腰部的影响。** A3 根连杆 `pelvis_link` 位于柔性腰关节之下，因此腰部关节仍包含在通往球拍的正运动学链中。这对乒乓球所需的躯干旋转、重心转移与触及范围十分重要。

2. **运动链核验。** 使用 A3 的队伍应依据部署真机核验所用 URDF 版本、站立时 `pelvis_link` 高度，以及从 `pelvis_link` 到持拍手的完整关节链。

3. **中间件桥接。** A3 原生运行于 AimRT 而非 ROS 2。AimRT 支持将 ROS 2 作为其多种通信协议之一（另有 HTTP、gRPC、MQTT 与 Zenoh）。对于 HOPE 架构，有两种集成方式：
   - **方式 1（推荐）：** 将 HOPE 规划器作为 ROS 2 节点运行；把 `RacketCommand` 话题桥接到 AimRT，由 A3 的原生 WBC 消费。由 `world → P1 → pelvis_link` 得到的 `pelvis_link` 位姿经 ROS 2 → AimRT 流动。
   - **方式 2：** 直接在 AimRT 内运行规划器，通过 AimRT 的 ROS 2 协议支持订阅动捕数据。

4. **A3 P1 配准与当前标记点状态。** v2 CAD 表与物理 `a3_hip_marker_shell_p1_mocap_balls_0702.x_t` 外壳定义了全部十个标记点球心 `f1`–`f5`、`b1`–`b5`；真机动捕实验已确认十个点均可见。完整十点集合在 `pelvis_link` 中的名义质心为 `[-0.0024, 0, -0.1490] m`，应以全部十个标记点创建 Motive P1 资产。在 P1 局部轴已对齐到 `pelvis_link`（`+X` 向前、`+Y` 向左、`+Z` 向上）后，质心到骨盆的 CAD 核验枢轴平移为 `[+0.0024, 0, +0.1490] m`（`[+2.4, 0, +149.0] mm`）。仅凭质心无法确定姿态。一次性启动标定时，使用 `p1_pelvis_calibrator` 将动捕 `/P1/pose`（`world → P1`）与独立产生、按采集时刻同步且完整 6 自由度的 `world → pelvis_link` `PoseStamped` 对比；该工具不使用 `Table`，因为共同的 Table 变换会在每个样本中精确抵消。当前 A3 真机桥接仅提供骨盆 IMU、没有骨盆绝对平移，因此真机集成必须先由外部测量系统或状态估计器提供独立的 `/a3/calibration/pelvis_pose`，之后才能执行此流程。采集时应同时平移并旋转骨盆；低 RMS 只能说明一致性，工具会另行检查运动激励与时间覆盖，避免静止样本通过。把已安装标记板完整的 6 自由度 `P1 → pelvis_link` 修正保存为 JSON。正常启动时由 `p1_pelvis_tf_publisher` 将该 JSON 发布为静态 TF；其实测结果优先于 CAD 名义值。也可以选择把修正吸收到 Motive 的 P1 枢轴定义中，但不得同时再发布 ROS 2 静态 TF。工具和验收步骤见 [`docs/OPTITRACK.md`](../docs/OPTITRACK.md#calibrating-p1-to-an-a3-pelvis_link)。

### 4.4  竞赛报名要求

每支队伍必须在 HOPE 竞赛报名时声明以下信息。这些信息用于核实动捕系统、规划器与 WBC 是否已针对其特定的人形机器人平台正确集成。

| 项目 | 描述 | 示例（Agibot A3） |
|------|-------------|---------------------|
| **机器人型号** | 制造商与型号标识 | Agibot 远征 A3 |
| **所申报的 URDF 根连杆** | 控制器使用的根连杆确切名称 | `pelvis_link` |
| **根坐标系物理位置** | 描述根坐标系原点在物理机器人上的位置 | 髋部板中心，位于髋偏航轴交点 |
| **根坐标系模式** | 采用哪种约定（第 4.1 节的 A/B/C） | 模式 A（骨盆根） |
| **站立时根坐标系高度** | 标称姿态下根坐标系原点离地高度 | 各队实测值 |
| **动捕到根坐标系静态变换** | 从 P1/P2 标记点簇坐标系到所申报 URDF 根坐标系的完整 6 自由度变换 | 标定值；A3 为 `P1 → pelvis_link` |
| **手臂自由度数** | 从所申报根坐标系到拍柄的驱动关节数，含腰部 | 依平台而定 |
| **中间件** | ROS 2 原生、带 ROS 2 桥接的 AimRT，或其他 | 带 ROS 2 桥接的 AimRT |
| **URDF 可获取性** | 公开 URL 或“在 NDA 下提供给组织方” | 仓库 `agibot/URDF/` 路径 |

标定后的动捕到 URDF 偏移保存为 JSON，并在启动时发布。A3 的最终链路为
`world → P1 → pelvis_link`：

```bash
ros2 run hope_bringup p1_pelvis_tf_publisher \
  --calibration-file calibration/p1_to_pelvis.json
```

### 4.5  机器人已知信息 vs. 动捕提供信息

| 信息 | 来源 | 使用方 |
|-------------|--------|---------|
| 采集频率下的球体 6 自由度位姿：位置 `[x, y, z]` + 四元数 `[qx, qy, qz, qw]` | 动捕 → ROS 2 话题 | 规划器使用位置（阶段 1–3）；姿态保留用于校验及未来含旋转估计 |
| 人形机器人根坐标系 6 自由度位姿（A3 为 `pelvis_link`） | 实时 P1/P2 位姿与标定静态变换组合 | WBC（阶段 4）用于根位置指令 |
| `Table`（旧称 PPT）刚体位姿 | 仅搭建/标定场次与训练数据录制——**无比赛流** | 场地标定（世界原点核验） |
| 球拍 6 自由度位姿 | 由关节编码器 + 所申报机器人根坐标系经**正运动学** | WBC 内部状态；**非**来自动捕 |
| 球拍期望状态 | 规划器输出（阶段 3） | WBC（阶段 4）作为跟踪目标 |

---

## 5  球体刚体跟踪配置

OptiTrack Motive 与青瞳 CMTracker 现在均将乒乓球解算为具名的**刚体资产**。测量状态为完整位姿：平移 `(x, y, z)` 与姿态。资产枢轴必须与球体几何中心重合；若厂商工具无法直接做到，应记录并应用固定的“资产坐标系到球心”变换。

### 5.1  球体准备与资产定义

- 使用厂商认可的刚体球处理方案和标记图案/星座。**经验证的处理方式：在标准乒乓球上加贴回射标记点（marker dots），在 OptiTrack 与青瞳上均可实现稳定的 6 自由度刚体跟踪。** 不得根据已淘汰的单点方案自行推断可用的标记数量或布局。
- 尽量减小对球体质量、质心、直径、表面摩擦和空气动力学的影响，并按竞赛规则验证处理后的球。
- 图案应非对称，并在整个相机空间内与 `Table` 和机器人图案保持可区分。
- 在 Motive 或 CMTracker 中设置稳定的刚体资产名称和 ID（建议逻辑名称：`Ball`）。话题名和发送方名称区分大小写。
- 将刚体枢轴设置在几何球心并记录局部坐标轴。正式采集前验证高速跟踪、遮挡后重捕获以及 ID 稳定性。

> **旧版球体处理方式不兼容。** 本文档早期版本（≤ v0.4，单标记点跟踪）推荐*整体回射涂覆*的球体。该建议现已**反转**：均匀涂覆的球面没有可区分的标记点星座，无法被识别为刚体。刚体跟踪需要带图案的处理方式。请勿复用为已淘汰的单点方案准备的球。

### 5.2  位姿表示

操作人员可将状态查看为 `(x, y, z, pitch, yaw, roll)`，但 ROS 2 不应把欧拉角作为标准姿态传输格式。VRPN tracker 报告以四元数传输姿态；ROS 2 在 `geometry_msgs/Pose.orientation` 中按 `(x, y, z, w)` 存储：

```text
geometry_msgs/Pose
  position:    x, y, z
  orientation: x=qx, y=qy, z=qz, w=qw
```

应对每个四元数归一化，并拒绝 NaN、零范数或过期位姿。如需为显示或分析转换为欧拉角，必须注明坐标系、手性与旋转顺序。Motive 的官方约定以 X 表示 Pitch、Y 表示 Yaw、Z 表示 Roll，并采用右手局部轴；不得假定其他厂商的欧拉角显示使用相同约定。

### 5.3  姿态、角速度与旋转

当前 HOPE 规划器仍是**无旋转**规划器：它消费 `(x, y, z)` 并建模平移阻力，但不使用球体姿态或马格努斯力。ROS 2 桥接仍完整保留所测四元数，使录包保持 6 自由度，并供未来估计器使用。

刚体四元数本身并不是角速度。含旋转的扩展必须处理四元数正负号连续性、使用源时间戳对旋转求导、滤除噪声，并确认刚体标记图案与球壳机械锁定。若标记载体相对球壳滑动，报告的姿态就不等于真实球体旋转。

### 5.4  验收检查

每次场地采集前：

1. 将 Ball 资产放在经测量的球台地标处，确认报告的枢轴就是球心。
2. 按已知姿态转动处理后的球，确认四元数已归一化，且显示的 pitch/yaw/roll 沿预期轴变化。
3. 在完整空间内发球、落台和击球；测量丢帧、延迟、重捕获行为及资产 ID 互换。
4. 用 `ros2 topic echo` 确认 ROS 2 的 `frame_id`、源时间戳、单位及完整的位置与姿态字段。
5. 在采集元数据中记录厂商资产名称/ID、球体标记方案、枢轴变换、局部轴定义及软件版本。

---

## 6  向 ROS 2 流式传输刚体位姿

厂商软件负责相机重建与刚体解算。ROS 2 桥接接收厂商原生数据流，并将每个已解算位姿映射为标准 ROS 消息。位置单位为米；姿态自厂商流至 ROS 2 全程保持四元数。

### 6.1  已确认的传输路径

规划器契约统一，但底层协议有意区分：**OptiTrack 使用 NatNet；青瞳使用 VRPN。** 两条路径都保留厂商解算的坐标与四元数，最终在 `/poses` 发布 `geometry_msgs/PoseArray`，且索引 0 为 `Ball`。

```text
OptiTrack 相机 → Motive → NatNet UDP → motion_capture_tracking_node
                                      → /optitrack/poses (NamedPoseArray)
                                      → optitrack_mct_relay → /poses

青瞳相机 → CMTracker/MCServer → VRPN → vrpn_mocap
                                     → /vrpn_mocap/<sender>/pose_id_<N> (PoseStamped)
                                     → pose_to_posearray → /poses
```

| 系统 | 厂商载荷 | ROS 2 桥接 | ROS 2 结果 |
|------|----------|------------|------------|
| **OptiTrack / Motive** | `Ball`、`P1`、`P2` 的 NatNet 刚体帧：资产名、位置向量与四元数 | vendored `motion_capture_tracking`（命名空间 `/optitrack`），再经 `optitrack_mct_relay` | `/optitrack/poses` 为 `NamedPoseArray`（`header` 加 `{string name, geometry_msgs/Pose pose}` 条目）；relay 产生 HOPE `/poses`、`/ball/point`、可选的 `/P1/pose` 与 `/P2/pose`，以及 TF |
| **青瞳** | 具名 CMTracker 刚体的 VRPN tracker 报告：发送方名称、传感器索引、位置向量与四元数 | vendored `vrpn_mocap`，再经 `pose_to_posearray` | `/vrpn_mocap/<sender>/pose_id_<sensor_id>`，类型 `geometry_msgs/PoseStamped`；适配器将完整位姿复制到 `/poses` |

`(pitch, yaw, roll)` 只是操作界面的表示方式。两条路径均以四元数传输并保留姿态。`PoseArray` 没有每个位姿的名称字段，因此 `Ball` 必须保持在首位；对象名保留在 OptiTrack 原始流或场次元数据中。OptiTrack 原始话题被刻意放在命名空间下：若将 `NamedPoseArray` 发布到裸 `/poses`，会与规划器的 `geometry_msgs/PoseArray` 发生 DDS 类型冲突。

### 6.2  OptiTrack / NatNet 路径

在 Motive 中将比赛资产 `Ball`、`P1`、`P2` 定义为刚体；`Ball` 枢轴位于球心，`P1`/`P2` 保持为标记点簇刚体坐标系，并分别标定至各机器人申报的 URDF 根坐标系。`Table` 资产仅在独立的搭建/标定或训练数据录制场次中使用（第 2.3 节）；比赛流式传输前应停用或省略。

Motive 预期设置：

| 设置 | 所需取值 | 备注 |
|------|----------|------|
| NatNet | ✅ 启用 | 仓库内 OptiTrack 后端的必需项；此路径不使用 Motive 的 VRPN Streaming Engine |
| Up Axis | **Z** | 与 HOPE ROS 2 REP 103 世界坐标系一致；比赛前须在地标处验证 |
| 传输方式 | 优先 Unicast | 客户端连接到 Motive PC；NatNet 与服务器协商流细节 |
| 命令端口 | 通常为 UDP 1510 | vendored 驱动使用 NatNet 命令通道并从 Motive 获得数据端口信息；防火墙须允许协商后的 UDP 数据流量 |
| 刚体 | 比赛为 `Ball`、`P1`、`P2`；`Table` 仅限标定 | 名称区分大小写，并原样传递至 relay |
| 球 | 名为 `Ball` 的 6-DOF 刚体资产 | 枢轴设在几何球心；跟踪丢失时 `Ball` 条目消失，relay 暂停 `/poses`，不会重发旧球位姿 |

以 Motive PC 地址启动完整规划器路径：

```bash
ros2 launch hope_bringup hope_bringup.launch.py \
  mocap_backend:=optitrack \
  mocap_server:=MOTIVE_PC_IP \
  mocap_network_latency_ms:=MEASURED_ONE_WAY_MS
```

链路为 `Motive → motion_capture_tracking_node → /optitrack/poses → optitrack_mct_relay → /poses`。驱动每个相机帧发布一个 `NamedPoseArray`；relay 按 `config/optitrack_relay.yaml` 映射名称，仅在另行配置时缩放位置，并保留四元数。默认驱动使用 `topics.header_time: ros_latency_compensated`：从 ROS 接收时刻减去 NatNet 的 Camera/Motive 延迟和实测单向网络/主机接收延迟，把曝光时刻映射到本地 ROS 时间域。不得使用仅表示到达时刻的 `ros`，也不得把 Motive 主机独立时钟域的 `camera` 直接与 ROS 时间混用。旧版 Motive VRPN 的 3883 端口不属于此连接。构建、启动和诊断细节见 [`docs/OPTITRACK.md`](../docs/OPTITRACK.md)。

### 6.3  青瞳 / VRPN 路径

在 CMTracker/MCServer 中将球定义为刚体，并分配稳定的 VRPN 发送方名称（如 `Ball`）。球不再作为共享发送方下的未标记标记点处理。将流式向上轴设置为 **Z**（第 2.2 节），即无需软件坐标转换。在 ROS 2 Jazzy 主机上运行 vendored 原生 ROS 2 VRPN 客户端，指向青瞳服务器：

```bash
ros2 launch vrpn_mocap client.launch.yaml server:=CHINGMU_SERVER_IP port:=3883
```

配合参数文件：

```yaml
/vrpn_mocap_client:
  ros__parameters:
    server: "CHINGMU_SERVER_IP"   # CMTracker / MCServer PC
    port: 3883                    # VRPN 端口
    frame_id: "world"
    multi_sensor: true            # 每个刚体传感器索引一个话题
    use_vrpn_timestamps: false    # 仅当服务器与 ROS 主机时钟已同步时设为 true
    update_freq: 100.0
    refresh_freq: 1.0
```

客户端自动发现 VRPN tracker 发送方。其位姿回调将 `vrpn_TRACKERCB.pos[0:3]` 直接写入 `PoseStamped.pose.position`，将 `quat[0:4]` 直接写入 `pose.orientation.{x,y,z,w}`。当 `multi_sensor: true` 时，典型单传感器刚体显示为：

```text
/vrpn_mocap/P1/pose_id_0     geometry_msgs/PoseStamped
/vrpn_mocap/P2/pose_id_0     geometry_msgs/PoseStamped
/vrpn_mocap/Ball/pose_id_0   geometry_msgs/PoseStamped
```

实际名称与大小写由 CMTracker 决定且区分大小写。若 CMTracker 分配的传感器索引不是 0，应使用实际发布的索引，不要在桥接层重写。`multi_sensor: true` 是安全默认值，可避免同一发送方暴露多个传感器时的冲突。

配置 `hope_bringup/pose_to_posearray` 时将 Ball 话题放在首位，以保持规划器默认的 `ball_pose_index: 0`。适配器发布 `/poses` 但不产生 `/tf`；如部署还需要命名变换，请另加 `tf2_ros` broadcaster。

### 6.4  坐标与姿态转换

两家厂商的输出都必须以第 2.1 节的标准 REP 103 Z 轴向上坐标系到达。将 Motive 的 NatNet 流和 CMTracker 的 VRPN 流配置为 Z-up（第 2.2 节），并在比赛前用地标验证。仓库内适配器不执行厂商专属的轴向旋转。若场地无法输出标准坐标系，应在 HOPE relay 上游加入一次明确的变换，并变换**完整位姿**，而不仅仅是三个位置分量：

```text
p_HOPE = R_HOPE_FROM_MOCAP · p_mocap + t_HOPE_FROM_MOCAP
R_HOPE_BODY = R_HOPE_FROM_MOCAP · R_mocap_body
```

对不符合约定的右手系 Y-up 源，平移可映射为 `x_HOPE=x_mocap`、`y_HOPE=-z_mocap`、`z_HOPE=y_mocap`。姿态须用 `tf2` 或四元数/矩阵组合应用同一固定旋转。逐分量修改 pitch/yaw/roll 不是有效的一般位姿变换。

在信任任何转换之前，请在经测量的球台地标处核实源坐标系的手性与轴向。镜像的源坐标系需要按安装情况专门校正；不要根据厂商名称猜测。

### 6.5  ROS 2 验证清单

```bash
# OptiTrack / NatNet
ros2 topic echo --once /optitrack/poses

# 青瞳 / VRPN
ros2 topic echo --once /vrpn_mocap/Ball/pose_id_0

# 任一后端：规划器输入
ros2 topic echo --once /poses
```

确认以下各项：

- `Ball`（以及流式传输时的 `P1`/`P2`）是彼此独立、稳定的刚体；遮挡后不发生资产 ID 互换。确认比赛期间**没有** `Table` 话题在流式传输。
- `position` 单位为米，`orientation` 有限且为单位长度，Ball 枢轴位于几何球心。
- 消息 `frame_id` 与轴向匹配 HOPE 世界坐标系。**每个厂商、每次安装在比赛前都必须做地标验证**：将 `Ball` 资产放在经测量的地标处（如球网中心线 `x = 1.37, y = −0.7625, z = 0.02`），确认流式坐标与第 2.1 节一致。这可发现错误的 Up Axis、原点偏移、镜像坐标系或意外的重复变换。
- 遮挡应产生**丢帧**，而非冻结或全零位姿。NatNet 路径中，缺少 `Ball` 条目会使 relay 暂停 `/poses`；消费者不得用旧球位姿填充。
- 仓库默认 NatNet 配置使用 `ros_latency_compensated` 与实测 `mocap_network_latency_ms`；不得使用仅表示接收时刻的 `ros`，也不得把 Motive 的 `camera` 时钟域直接与 ROS 混用。VRPN 厂商时间戳必须经过已验证的 NTP/PTP 或等效时钟映射。
- `/poses` 索引顺序与规划器配置一致。当前无旋转规划器读取 Ball 位置，完整四元数仍保留在消息与录包中。

---

## 7  与 HOPE 规划器的集成

配套规划器文档（《HOPE 7DOF 球拍基于模型的规划器参考设计》）消费由动捕桥接发布的球体位姿（当前使用位置）并产生球拍目标指令。整个系统的数据流为：

```
动作捕捉系统 (360 Hz)                                   人形机器人 (本体感受)
  │                                                      │
  ├── Ball 6 自由度刚体位姿 ──▶ HOPE 规划器             │
  │      （规划器当前使用 xyz）     阶段 1–3            │
  │                                       ▼               │
  └── P1 标记点簇 6 自由度 ─▶ 机器人根 TF ─▶ WBC（阶段 4）◀── RacketCommand
                                          │              (p_intercept,
                                          │               v_racket,
                                          ▼               n_racket,
                                    关节指令               t_strike)
                                    （随平台而异）
                                          │
                                          ▼
                                    球拍位姿
                                    （由机器人根坐标系 + 关节编码器
                                     经 FK 推断，
                                     非由动捕测量）
```

规划器完全在第 2.1 节定义的 HOPE 标准世界坐标系中运行。OptiTrack/NatNet 与青瞳/VRPN 路径在完成配置或第 6.4 节的转换后，交付完整刚体位姿；当前规划器使用 Ball 的平移，四元数仍保留在 ROS 2 消息与录包中。

---

## 8  小结

HOPE 动作捕捉参考系统定义四个具名刚体资产；比赛期间仅流式传输 `Ball`、`P1` 与 `P2`：

1. **`Ball`**——乒乓球的 6 自由度刚体位姿；ROS 2 接收位置与四元数，当前规划器使用位置。
2. **`Table`**——仅用于搭建/标定的资产，锚定世界坐标系原点（旧版笔记称 `PPT`）；其位姿出现在训练数据录制中，但比赛期间**不**流式传输。
3. **`P1`**——选手一人形机器人的标记点簇刚体。
4. **`P2`**——选手二人形机器人的标记点簇刚体。

每支队伍在报名时申报其机器人专用 URDF 根坐标系，并提供从 P1/P2 到该根坐标系的标定静态变换（第 4 节）。A3 的映射为 `P1 → pelvis_link`。

**球拍从不由动作捕捉系统跟踪。** 每台人形机器人必须通过关节编码器与所申报的机器人根坐标系位姿，经正运动学推断自身的球拍位姿。这是根本性的传感架构：外部感知（球体轨迹）馈入基于模型的规划器，而内部本体感受（关节状态 + 机器人根位姿）驱动定位球拍的全身控制器。完整的从机器人根坐标系经 7 自由度手臂到 3D 打印球拍支架的正运动学链，参见配套文档《HOPE WBC 仿真训练参考设计》（第 2.8 节）。

---

## 参考文献

- Su, Z., Zhang, B., Rahmanian, N., Gao, Y., Liao, Q., Regan, C., Sreenath, K., & Sastry, S. S. (2025). HITTER: A HumanoId Table TEnnis Robot via Hierarchical Planning and Learning. *arXiv:2508.21043v2*.
- HITTER 项目主页：https://humanoid-table-tennis.github.io/
- motion_capture_tracking（已 vendored 进仓库，作为受支持的 OptiTrack/NatNet 后端——`hope_ws/src/motion_capture_tracking/`，精确 pin 与本地补丁见其 PIN.md；发布 `NamedPoseArray`，由 `optitrack_mct_relay` 桥接到 `/poses` 契约；上游：https://github.com/IMRCLab/motion_capture_tracking）
- OptiTrack Motive VRPN Streaming Engine（仅限刚体；默认端口 3883）：https://docs.optitrack.com/motive-ui-panes/settings/settings-streaming
- VRPN 协议：https://github.com/vrpn/vrpn
- OptiTrack Motive 流式设置：https://docs.optitrack.com/v3.0/motive-ui-panes/settings/settings-streaming
- 青瞳视觉（CHINGMU）动作捕捉：https://www.chingmu.com/ （英文：https://en.chingmu.com/）——VRPN/LiveStream 流式传输，C/C++/C#/Python/ROS SDK
- ChingMuVrpnRos（青瞳官方 ROS/VRPN 参考驱动）：https://github.com/ChingMuVisionTech/ChingMuVrpnRos
- vrpn_mocap（ROS 2 VRPN 客户端）：https://index.ros.org/p/vrpn_mocap/
- Agibot X1 训练代码（Agibot 运动学约定参考）：https://github.com/AgibotTech/agibot_x1_train
- 配套文档：《HOPE 7DOF 球拍基于模型的规划器参考设计，v0.1》
- 配套文档：《HOPE WBC 仿真训练参考设计，v0.5》
- 配套文档：《HOPE 硬件部署参考设计，v0.1》
