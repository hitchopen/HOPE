# 球物理模型：校准项目、数据与采用判据

适用：本仓库 `hope_planner_cpp`、实机 Laptop/HDU 和原生 MuJoCo Gate3。
2026-10-13 按仓库代码核对；本文更新没有连接机器人或重新拟合。
部署操作见 [部署与 Gate3](deploy_and_gate3.md)，手柄见 [Runner / Xbox](runtime_xbox.md)。

## 1. 先区分物理、测量和控制

球拍碰不到球不一定是球的阻力错了。按下面顺序校准，保留各自结果，不用一个系数吸收所有误差。

| 层 | 需要确定的量 | 如何取得证据 |
| --- | --- | --- |
| 测量坐标 | 长度单位、世界原点/轴、桌面高度、球刚体 pivot 到球心的偏移、四元数方向 | 桌角/已知距离、球静置与旋转、已知旋转方向；球心接触高度应是桌面加球半径 |
| 时钟与链路 | 相机源时间、接收时间、跨机时钟偏差、丢帧/乱序、球包延迟 | 原始 MCAP 时间戳、clock samples、球包收发审计；不能直接把不同钟域的时间相减 |
| 球的飞行 | 阻力、Magnus、每球初速度与旋转、可能的旋转衰减 | 无接触的完整自由飞行弧，覆盖快/慢球及不同旋转 |
| 球桌接触 | 垂直反弹、切向冲量、滑动摩擦上限、碰后旋转 | 同一次落台前后的连续轨迹与姿态；隔开接触附近的混合窗口 |
| 球拍接触 | 拍面恢复系数随法向速度变化、摩擦、出球旋转、有效碰撞面 | 入/出球轨迹、拍面位置/法向/速度、实际接触事件；只看入射球无法拟合 |
| 机器人执行 | pelvis 到拍面 FK、击球平面、关节零位、指令到实测的延迟与跟踪误差 | base、实测 q/dq、最终 q_des/dq_des/tau/kp/kd、IMU、Runner/Planner 事件 |

机器人执行误差属于 MuJoCo 机器人模型/策略跟踪的校准，不能靠修改球阻力来补偿。
接球位置预测与击球后的回球落点也要分别评分。

## 2. 当前模型与需要校准的参数

来球估计/预测使用的自由飞行模型为：

```text
dp/dt = v
dv/dt = g - drag_k * |v| * v + magnus_k * (omega × v)
```

位置用 m，时间用 s，速度用 m/s，`omega` 是世界坐标中的 rad/s；rev/s 要乘 `2π`。
`drag_k` 的单位为 1/m，`magnus_k` 在这一加速度公式中按弧度无量纲约定解释。
二者已经是加速度系数，不再额外乘一次质量的倒数。

桌面碰撞使用球底部滑移速度与有上限的切向冲量：

```text
slip = (vx - r*omega_y, vy + r*omega_x, 0)
Jt_over_m = min(table_tangential_gain * |slip|,
               table_friction_cap_mu * (1 + restitution_v) * |vz|)
delta_v_xy = -Jt_over_m * slip / |slip|       # |slip|=0 时为 0
vz_after = -restitution_v * vz_before
delta_omega = (delta_v_y, -delta_v_x, 0) / ((2/3)*r)
```

实现在 [ball_dynamics.hpp](../../hope_ws/src/hope_planner_cpp/include/hope_planner_cpp/ball_dynamics.hpp)。
只拟合速度变化而不看旋转，会把摩擦与入射旋转混在一起。

| 项目 / 参数 | 当前参考值 | 校准方法与限制 |
| --- | --- | --- |
| `drag_k` | 0.12861635416914574 | 多速度自由飞行弧，联合拟合初始状态；检查误差是否随速度变化 |
| `magnus_k` | 0.00379260504480258 | 带可信姿态的不同旋转轴/转速飞行；旋转比例错会直接污染该系数 |
| `restitution_v` | 0.9313775047019153 | 同一落台事件前后法向速度；分法向速度区间检查 |
| `table_tangential_gain` | 0.38744443855065164 | 用入射滑移、碰前后切向速度与旋转拟合 |
| `table_friction_cap_mu` | 0.16706204822993975 | 需要覆盖摩擦上限触发与未触发的事件；单一角度无法充分区分增益与上限 |
| `restitution_h` | 0.6125555614493483 | `1-table_tangential_gain` 的兼容值，只在零入射旋转且未触发摩擦上限时成立；不单独当作材料常数拟合 |
| 球半径 `radius` | 0.020 m，固定先验 | 实物测量；不要把球刚体 pivot/桌面原点误差拟合成半径 |
| 重力 / 惯性系数 | 9.81 m/s² / 2/3，固定先验 | 当前程序中的先验；贴标后的质量分布变化需独立测量/辨识 |
| 质量 | 本次未辨识 | 称量同一只贴标球并记录 ball ID；不能从上述加速度系数单独恢复质量 |
| 旋转衰减 | 未辨识，保持零先验 | 较长无遮挡姿态序列、分转速观察；置信区间包含零不能称为“测得衰减为零” |
| 球拍法向/切向接触 | 本次未重新拟合 | 按实际球拍、正反面、击打部位、相对速度采样；不要直接使用球桌摩擦替代 |

这些是 2026-10-10 多批次同型号贴标球的拟合/采用值，来源与限制在
[采用回执](../../hope_ws/src/hope_planner_cpp/config/ball_physics_real_20261010.receipt.json)。
换过球但换球时刻未完整记录，不能称为某一只球的精确常数。换球、重新贴标、换桌面/球拍或重新定义刚体后，应补采对照数据。

### 每次来球的旋转不是一个固定配置值

球姿态四元数用于估计当次 `omega`，需要处理四元数符号等价和遮挡后的跳变。
单反光点只能提供位置，不能提供可辨识旋转。姿态无效时可以有位置预测，但不能把该球标为“旋转测量有效”。

以下是估计器/预测开关，不是材料参数：

| 配置 | 当前值 | 含义 |
| --- | --- | --- |
| `future_contact_spin_enabled` / `future_contact_magnus_enabled` | true / true | 使用旋转接触及 Magnus 预测；仅打开 shadow 日志不代表控制使用旋转 |
| `future_contact_spin_omega_scale` | 1.0 | 采用测得角速度；不是所有球都以 1 rad/s 转动 |
| `future_contact_spin_min_coherence` | 0.4 | 旋转估计的一致性阈值 |
| `spin_window_s` / `spin_min_span_s` | 0.18 / 0.05 s | 因果估计窗口与最短跨度 |
| `spin_max_gap_s` / `spin_max_rev_s` | 0.05 s / 20 rev/s | 当前采样有效性边界；窗口内大缺口或更快旋转需要单独验证 |
| `post_net_commit_delay_s` | 0.05 s | 提交基准，不是球的飞行参数或固定执行延迟 |

当前球包器启用了自适应提交：策略准备预算 0.35 s、传输预算 0.10 s、最长过网等待 0.50 s、碰后观察跨度 0.08 s。
见 [球包配置](../../hope_ws/src/hope_planner_cpp/config/model21800_flight_packetizer.yaml)。
预算不保证 Wi-Fi 延迟小于 100 ms；不能给每球统一减掉 50 ms 来“校准物理”。

### 当前尚未统一成完整出球旋转求解的部分

[Stage 3 球拍目标求解](../../hope_ws/src/hope_planner_cpp/src/racket_target_planner.cpp)
的出球飞行积分目前只用重力和阻力，没有把出球旋转/Magnus 联合纳入落点逆解。
法向拍速采用 `restitution_exp_g1=0.759`、`restitution_exp_g2=-0.0441` 的速度相关模型；
`restitution_racket=0.654` 是该迭代的初值，不是最终对所有击球固定使用的恢复系数。

[Gate3 球拍碰撞](../../a3_deploy/A3_MuJoCo_Sim/aimrt_mujoco_sim/src/module/mujoco_sim_module/common/gate3_ball_contact_model.h)
仍使用原有球拍切向系数 `0.52 / 0.0`、摩擦上限 `0.5` 和法向系数 `0.759 / -0.0441`。
它与 Stage 3 也存在模型及恢复系数下限差异（0.05 与 0.2）。这些没有被 10 月 10 日球桌/自由飞行拟合重新验证。
所以“Planner / Gate3 来球与球桌模型一致”不能扩展成“球拍接触和回球落点已全部标定”。

## 3. 采集一轮足够用于校准的数据

1. 记录 ball ID、型号、贴标方式/更换时间，桌面、球拍正反面与安装方式，以及策略/程序/配置 SHA。
2. 保留原始 C3D 或 MCAP，包含 Ball 位置和姿态、源时间与接收时间、有效标记；保留桌面与机器人坐标变换。
3. 用不同速度、弧线和旋转覆盖现场来球，尤其保留慢球、低球、遮挡、拒绝和未命中样本。
4. 桌面碰撞采到两侧自由飞行段；拟合时剔除接触附近混合窗口，不用跨碰撞的一段多项式估计速度。
5. 球拍碰撞额外记录拍面运动与接触时刻。实机比赛/正常运行以 pelvis + 实测关节 FK 求拍面；额外拍面动捕只用于专门内部标定，不能混入正常运行结果。
6. Foxglove **Record ON** 等实际 `ON` 后试球，完成后 **Record OFF** 等实际 `OFF`，保留三机相同 `record_<UTC>_<id>` 分段。

用于 MuJoCo 机器人跟踪辨识还需 Runner obs/actions、最终控制目标、关节 state/command、双 IMU、base、Planner 球包/命令和 clock samples。
`ERROR` 或缺失日志要标明，不能把不完整的一段当作完整拟合证据。

## 4. 拟合、留出验证和采用顺序

1. 先验证单位、球心、坐标、时间和姿态连续性；记录遮挡/跳变，不对大缺口悄悄插值。
2. 按自由飞行、桌面接触、球拍接触分段。飞行用轨迹积分拟合，避免对噪声位置二次差分来拟合加速度。
3. 先拟合重力固定下的阻力与 Magnus，再拟合桌面冲量和旋转变化；球拍使用单独数据。
4. 按批次/球 ID 留出，不能把同一条轨迹的相邻帧随机拆为训练和测试。报告批次差异、适用速度/转速范围和不确定度。
5. 因果回放只给提交时刻以前的数据。真值使用实际观察到的击球平面穿越/接触/落点，不用同一模型生成的终点作真值。
6. 同时报告时间、Y/Z、速度、旋转的中位数/P90、有效覆盖率、每类拒绝数及新增拒绝球。旧/新模型比较注明各自分母。
7. 使用同一模型和场景跑完整 Gate3，再做有现场人员的实机连续多球测试，分别记录第一球与后续球、快球与慢球、接触/合法回球/回位/摔倒。

2026-10-10 的历史采用回执：75 球开发回放从 75 个有效输出变成 73 个，另有两条低球被 `dead_ball` 拒绝；有效输出误差下降不能单独证明命中率提高。
当时 Gate3 26 球、14 次有效击球/合法落点、5 次回位、0 次摔倒，正式验证未通过。
该回执不是 2026-10-13 的现场状态或新测试结论。

旧 [球物理拟合工具](https://github.com/dongc1/nohope/tree/build_6/hope_training/ball_physics_fit) 使用另一套场地数据及桌心原点。
不能把旧工具的桌心坐标、球质量、旋转边界或 YAML 原样当成本次近端桌角坐标下的部署值。

## 5. 参数如何到达 Planner 与 Gate3

| 消费者 | 配置/入口 | 采用后需要核对 |
| --- | --- | --- |
| Laptop 球包器 | [model21800_flight_packetizer.yaml](../../hope_ws/src/hope_planner_cpp/config/model21800_flight_packetizer.yaml) → `hope_ball_flight_packetizer` | 实际启动 YAML/覆盖项、旋转配置、当前 session、与 HDU 相同物理参数 |
| HDU Planner | [model21800_hardware.yaml](../../hope_ws/src/hope_planner_cpp/config/model21800_hardware.yaml) → `hope_planner_cpp_node` → 估计/预测 | 已部署 ARM 可执行文件、ROS 参数、Stage-2 模式和审计中的系数 |
| Gate3 C++ Planner / 球包器 | [pp_ball_physics.py](../../a3_deploy/a3_deploy_example/scripts/pp_ball_physics.py) 解析基础 YAML 与选定 overlay；[pp_gate3_rally.sh](../../a3_deploy/a3_deploy_example/scripts/pp_gate3_rally.sh) 传参 | 本次结果的 `pp_planner_debug.csv.ball_physics.json` 与实际审计 |
| Gate3 MuJoCo | 同一解析器 → `A3_GATE3_BALL_*` → 已安装 MuJoCo 插件 | 阻力、Magnus、桌面增益/上限/反弹一致；实际姿态/世界角速度有输出 |

[ball_physics_real_20261010.yaml](../../hope_ws/src/hope_planner_cpp/config/ball_physics_real_20261010.yaml)
提供双节点参数快照；文件存在不会自动覆盖活动 YAML。后加载的配置或 CLI 参数也可能覆盖基础值。
`model21800_*` 是沿用的配置文件名，不能据此认定实际模型编号；实际策略身份以 `policy.onnx`、`deploy.yaml` 和 SHA 为准。

只更新 `configs/ball_physics_venue.yaml`、Isaac YAML 或 MuJoCo XML 的接触参数，不会自动更新这四个消费者。
MuJoCo 的 soft-contact 阻尼也不是这里的恢复系数；实际桌面边沿事件会执行代码中的冲量模型。

只在电脑上查看“将采用什么值”（不会启动机器人）：

```bash
cd /absolute/path/to/HOPE
distrobox enter hope -- /usr/bin/python3 \
  a3_deploy/a3_deploy_example/scripts/pp_ball_physics.py \
  --params-file hope_ws/src/hope_planner_cpp/config/model21800_hardware.yaml
```

物理参数更新需同时准备两节点配置和匹配程序、Gate3 安装产物及前后 SHA。
机器人可靠支撑并进入 PASSIVE 后，按部署流程停会话、切换版本、重启到 PASSIVE；这些参数主要在启动时读取，改源码/本地 YAML 本身不构成实机采用。
验收时在正确的 ROS 232 / Fast DDS 环境查询两节点 `ros2 param dump`，对照解析回执和实际 PID 可执行文件；不要把旧进程的参数查询结果用于新版本证明。

## 6. 一份校准结果最少交付什么

- 原始数据路径、批次/球身份、单位/坐标/时钟说明与数据缺口。
- 参数值、固定先验、未辨识项、拟合方法与适用范围。
- 按批次留出结果和因果回放结果，包含全部拒绝/丢球分母。
- 四个消费者的实际配置、二进制/安装目录、SHA 和采用回执。
- Gate3 正式结果、实机连续多球结果及回退版本；未做实测写 `NOT_PROVEN`。

本文只规定要校准和验证的内容，不自动修改训练参数、策略、机器人或录制状态。
