# model_15200 → AGI RobotIOBackend 对齐 + MuJoCo 仿真验证 · 完整 Hands-on 手册

> Current workflow checked on 2026-10-11: use
> [deploy_and_gate3.md](../../docs/operations/deploy_and_gate3.md) for complete
> packaging, current native simulator installation, policy checks, Gate 3
> execution/report reading and immutable-slot staging. Use
> [runtime_xbox.md](../../docs/operations/runtime_xbox.md) and the
> [first-deployment guide](../../docs/operations/foxglove_first_hardware_test.md)
> for managed hardware operation. The dated model-specific examples below are
> historical text, including their old paths, cloud aliases, manual HAL startup,
> reset/perfect-tracking commands and claimed results. They are not current
> launch instructions or results from this documentation review.

> **Historical HitterPingPong path (2026-08-03):** follow
> [HITTER_PINGPONG_UNITREE_DEPLOY.md](HITTER_PINGPONG_UNITREE_DEPLOY.md).  It
> requires a complete `hope` Distrobox relink/install, then the fixed-racket
> contact test, and permits full Gate3 only after the outgoing-ball gate passes.
> The model_15200 instructions below are retained as historical material.

> **状态更新（2026-07-03）**：当前上线契约是 **175 维 deploy-parity**（`model_p4_deployparity`；最新导出谱系为 explicitpd_ft 微调 `model_25700`）。C++ runner 会按 ONNX 输入维度自动识别 175/180（`pp_onnx_policy.hpp`）。本手册中 `model_15200` / 180 维的细节仅适用于旧版（legacy 180-D）谱系。换 checkpoint 的流程见 `PINGPONG_NEW_CHECKPOINT_TUTORIAL.md`。

> 对应 AGI 两份文档：`README_robot_io_backend.md`（Step 1 ONNX 对齐）+ `README.md`（Step 2/3 编译 & MuJoCo 验证）。
> 状态（2026-06-30 复核）：对齐 + x86_64 + rockchip 编译**都已完成并验证**。本手册是“怎么跑”，不是“从头做”。

---

## 0. 先读：背景 + 关键结论（直接回应 AGI 那句话）

AGI 说的三件事 → 我们的应对：

| AGI | 含义 / 我们怎么做 |
|---|---|
| 别改我们的 simulation | 不碰他们 sim 的 MJCF / PD。只在我们这侧对齐 ONNX + 跑 runner。 |
| simulation 可能不准 | 他们 sim 用**显式 Euler PD**（kd 当电机力矩）；我们的策略是 Isaac **隐式 PD** 训练的。所以 free-base 在他们 sim 里可能 ~0.1s 发散——这是 **sim 保真度**问题，不是移植 bug。 |
| 机器人上是隐式 PD | 真机 body-drive 后端是 PD-in-backend（隐式），与训练一致 → 真机行为 ≈ 我们隐式 MuJoCo 的稳定结果。 |

**因此本次 MuJoCo 验证的目标 = 验证 I/O 契约**（obs → jointmap → decode → command → sync → transport 全链路在和真机同形的接口上跑通），**不是**在他们 sim 里证明策略绝对稳定。要“干净看挥拍”就用 **hoist（固定基座）** 变体；真正的稳定性以真机 / 隐式 MuJoCo 为准。

**环境前提（所有命令都在 distrobox `hope` 里跑）：**
```text
distrobox enter hope          # ROS2 Jazzy + g++13 + zmq headers 都在这个 box
```
- box 的 `.bashrc` 坏了（指向不存在的 `/opt/ros/lyrical`）→ **每个新 shell 都要手动** `source /opt/ros/jazzy/setup.bash`。
- 主机和 box 都**没有 ROS2 Humble**。AGI 原文档的 `mujoco_sim_standalone` 需要 Humble → 见 Step 3 路径 B。

---

## Step 1 — ONNX 对齐验证（README_robot_io_backend.md）

**对齐早已在 C++ 前端做完**（`src/a3/a3_deploy_onnx_ref/include/a3_pingpong/pp_*.hpp` + `src/a3/a3_deploy_onnx_ref/src/a3_deploy/a3_pingpong_main.cpp`；AGI 原 `a3_deploy_onnx_ref` 不动）。前端干的事正是 README_robot_io_backend.md 的 31-DOF `RobotCommand` 契约：

- model_15200：输入 `obs[1,180]` + `time_step[1,1]`，输出 `actions[1,31]`（+ 参考 body/joint 侧输出）。
- 解码：`q_des = default_joint_pos + action * action_scale`（Isaac 顺序），`kp/kd` 取自 onnx metadata。
- 按**关节名**把 31-DOF Isaac 输出 scatter 到 31-DOF backend 槽位（`MakeA3Layout31`：waist3, neck2, Larm7, Rarm7, Lleg6, Rleg6）。
- **neck 被动**：槽位 [3,4] 覆盖成 `q=0, kp=40, kd=2`（AGI `kA3Head*`），模型的 neck 输出丢弃。

### 复核命令（off-robot，不需要机器人）—— 已实测全绿

```text
distrobox enter hope -- bash -lc '
cd /home/dongc1/workspace/HOPE/a3_deploy/a3_deploy_example/src/a3/a3_deploy_onnx_ref
ORT=/home/dongc1/workspace/HOPE/a3_deploy/a3_deploy_example/thirdparty/onnxruntime/onnxruntime-linux-x64-1.19.2
MODEL=/home/dongc1/workspace/HOPE/a3_deploy/a3_deploy_example/dist/a3_deploy_x86_64/models/model_15200.onnx
OUT=/tmp/pp_gates; mkdir -p $OUT
# (a) bijection + default_q scatter
g++ -std=c++17 -O2 -I include -I /usr/include/eigen3 -I $ORT/include \
  include/a3_pingpong/test/pp_jointmap_test.cpp -L $ORT/lib -lonnxruntime -Wl,-rpath,$ORT/lib -o $OUT/jm && $OUT/jm $MODEL | tail -2
# (b) 完整 CommandFn（neck 被动 / gains / |action| 有界 / swing sweep）
g++ -std=c++17 -O2 -I include -I /usr/include/eigen3 -I $ORT/include \
  include/a3_pingpong/test/pp_policy_test.cpp -L $ORT/lib -lonnxruntime -Wl,-rpath,$ORT/lib -o $OUT/pol && $OUT/pol $MODEL | tail -3
# (c) 180-D obs builder vs Python golden（eigen-only）
g++ -std=c++17 -O2 -I include -I /usr/include/eigen3 \
  include/a3_pingpong/test/pp_parity_test.cpp -o $OUT/par && $OUT/par include/a3_pingpong/test/golden.txt | tail -3
'
```

**期望输出（实测）：** `JOINTMAP PASS` / `POLICY CALLBACK PASS`（max|action|≈14，fails=0）/ `PARITY PASS`（obs 误差 1.1e-16，clock 8/8，strike_frames C++=(34,147)==py）。

### Dump ONNX metadata（确认对齐字段）

```text
distrobox enter hope -- bash -lc '
python3 - <<PY
import onnxruntime as ort
s=ort.InferenceSession("/home/dongc1/workspace/HOPE/a3_deploy/a3_deploy_example/dist/a3_deploy_x86_64/models/model_15200.onnx")
md=s.get_modelmeta().custom_metadata_map
for k in ["joint_names","default_joint_pos","action_scale","joint_stiffness","joint_damping","body_names"]:
    print(k, "->", md.get(k,"<MISSING>")[:120])
print("in :", [(i.name,i.shape) for i in s.get_inputs()])
print("out:", [(o.name,o.shape) for o in s.get_outputs()])
PY
'
```
已确认：31 个 `joint_names`（Isaac interleaved），`joint_stiffness` = 80/80/85/120/120/50/…，`joint_damping` = 3/3/3/4/4/2/…，14 个 `body_names`。

> **结论：Step 1 已完成，无需再做对齐工作。** 上面三个 gate 是“想再确认时”的复核手段。

---

## Step 2 — 编译 x86_64 包（README.md「编译与打包」）

x86_64 包 + pingpong binary **已编好并 staged**：`dist/a3_deploy_x86_64/a3_deploy_onnx_ref_pingpong`（8.2 MB, x86-64 ELF）+ `models/model_15200.onnx` + `config/a3_runtime_config.pingpong.yaml` + `config/a3_aimrt_config.pingpong_iceoryx.yaml`。**跑仿真不需要重编。**

### 仅当你改了前端代码才重编（在 box 内）

```text
distrobox enter hope -- bash -lc '
cd /home/dongc1/workspace/HOPE/a3_deploy/a3_deploy_example
source /opt/ros/jazzy/setup.bash
bash scripts/build_a3_deploy_pkg.sh --arch x86_64 --jobs $(nproc)
'
```
打包脚本会自动构建 `a3_deploy_onnx_ref_pingpong` 目标。box 里有 zmq dev headers，**无需** host 那套 `thirdparty/zmq_shim` 垫片。

> ⚠️ **坑：打包会 `rm -rf dist/a3_deploy_x86_64` 再重建**，而 `model_15200.onnx` 和两个 `*.pingpong*` cfg **只存在于 dist、不在 src 树**。重编后要把它们拷回：
```text
# 重编后补回（从备份或上一份 dist 拷）
cp model_15200.onnx                         dist/a3_deploy_x86_64/models/
cp a3_runtime_config.pingpong.yaml          dist/a3_deploy_x86_64/config/
cp a3_aimrt_config.pingpong_iceoryx.yaml    dist/a3_deploy_x86_64/config/
```
（重编前先 `cp` 这三个文件到别处备份。）

---

## Step 3 — MuJoCo 仿真验证（核心）

两个 sim 都是 AGI 的，都通过同一套 `/body_drive/*` **iceoryx** 接口和 runner 闭环。runner 与 sim 用 iceoryx 共享内存通信（与 ROS 版本无关），所以 Jazzy runner 能驱动任一 sim。

### ✅ 路径 A（推荐，现在就能跑）：`A3_MuJoCo_Sim` 的 `a3_pingpong` sim

这就是 `SIM_FIDELITY_NOTE_FOR_AGI.md` 里讨论的那个 AGI MuJoCo sim：**带球拍的 a3_pingpong 机型**、Jazzy 编好、之前已和我们 runner 闭过 50Hz 环。开**两个 box 终端**。

#### 终端 1 — 起 sim

**(推荐) HOIST 固定基座**（看干净挥拍，不受 connect-gap 倾倒 / 显式-PD 发散影响）：
```text
distrobox enter hope
cd /home/dongc1/workspace/HOPE/a3_deploy/A3_MuJoCo_Sim/aimrt_mujoco_sim/build/install/bin
source /opt/ros/jazzy/setup.bash
for p in ros2_plugin_proto aimrt_msgs joint_msgs mujoco_sim_msgs; do source ../share/$p/local_setup.bash; done
export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:$(pwd):$(pwd)/../lib"
pkill -9 -x aimrt_main 2>/dev/null; pkill -9 -x iox-roudi 2>/dev/null; rm -f /dev/shm/iox*
setsid ./iox-roudi >/tmp/iox-roudi.log 2>&1 </dev/null & sleep 1
./aimrt_main --cfg_file_path=./cfg/a3_pingpong_hoist_cfg.yaml     # 带 viewer；无显示器加前缀 MUJOCO_GL=egl
```

**(可选) FREE-BASE 浮动基座**（看完整闭环；脚本自带 roudi + overlay，只 hardcode 非 hoist cfg）：
```text
distrobox enter hope
cd /home/dongc1/workspace/HOPE/a3_deploy/A3_MuJoCo_Sim/aimrt_mujoco_sim/build/install/bin
source /opt/ros/jazzy/setup.bash
pkill -9 -x aimrt_main 2>/dev/null; pkill -9 -x iox-roudi 2>/dev/null; rm -f /dev/shm/iox*
./start_a3_pingpong_iceoryx.sh                                    # viewer；无显示器：MUJOCO_GL=egl ./start_a3_pingpong_iceoryx.sh
```

#### 终端 2 — 起 runner（staged）

```text
distrobox enter hope
cd /home/dongc1/workspace/HOPE/a3_deploy/a3_deploy_example/dist/a3_deploy_x86_64
source /opt/ros/jazzy/setup.bash
export LD_LIBRARY_PATH=".:${LD_LIBRARY_PATH}"

# HOIST sim 配套（推荐先跑这个）：
./a3_deploy_onnx_ref_pingpong \
  --runtime-cfg config/a3_runtime_config.pingpong.yaml \
  --start passive --level 1 --legs-passive
```

**键位（运行时按键切状态）：**

| 键 | 作用 |
|---|---|
| `p` | PASSIVE（限位、零增益） |
| `s` | PD_STAND（保持 nominal 站姿） |
| `h` | SHADOW（算策略但**不发** command） |
| `m` | MOTION（算 + 发 command） |
| `0` / `1` | 挥拍等级 0=hold-windup / 1=forehand |
| `[` / `]` | gain_scale −/＋ 0.1 |
| `,` / `.` | swing_speed −/＋ 0.1（动作太快 actuator 跟不上时放慢） |
| `q` | 退出 |

**推荐验证顺序：** `p`（确认 passive）→ `h`（SHADOW：看 rate≈50Hz、`proj_grav≈[0,0,-1]`、ts 在 clip 上 sweep、`|act|` 有界，机器人不动）→ `s`（PD_STAND：站稳）→ `m`（MOTION 开始发 command）→ `0` 先 hold → `1` forehand。盯 neck/wrist 是否抖。

**常用 flag（runner）：**

| flag | 默认 | 说明 |
|---|---|---|
| `--start passive\|pd_stand\|shadow\|motion` | passive | 初始模式 |
| `--level 0\|1` | 1 | 挥拍等级 |
| `--gain-scale F` | 1.0 | PD 增益整体缩放（先试 0.4~0.8 更稳） |
| `--legs-passive` | off | **hoist 用**：腿固定 hold，不做平衡 |
| `--swing-speed F` | 1.0 | 动作时间拉伸（0.7=慢 30%） |
| `--perfect-tracking` / `--oracle-pelvis` | off | **free-base 用**：世界位姿 obs 用 sim 真值（默认 `fabricated` 是假的 nominal） |
| `--base-estimator` / `--official-stand` | off | **仅地面站立**，hoist **不要**用（会很响/失稳） |
| `--no-publish` / `--dry-run` | off | 只算不发（等同 SHADOW） |
| `--warmup-sec S` | 0 | 先 PD_STAND S 秒再自动切 `--start` 模式（非交互安全启动） |
| `--obs-csv` / `--trace-csv PATH` | "" | 逐 tick 记录 obs / 每关节 des·q·qd·kp·kd（仅诊断） |

> free-base 跑法：去掉 `--legs-passive`、加 `--perfect-tracking`，但**预期它可能在 connect gap 倾倒 / 因显式 PD 发散**——这是 AGI sim 保真度问题（见 Step 0），不是 bug。要看挥拍稳定性请用 hoist。

#### 每次 run 之间清理 iceoryx（两个终端都退出后）
```text
pkill -9 -x aimrt_main; pkill -9 -x iox-roudi; rm -f /dev/shm/iox*
```

---

### 🔶 路径 B（AGI 文档原本指的 `mujoco_sim_standalone`，本机被 Humble 卡住）

README.md「本地 MuJoCo 仿真验证」用的是 `mujoco_sim_standalone/`，但它 `env.sh` 要 `source /opt/ros/humble`，且自带的 `libaimrt_ros2_plugin.so` 是 **Humble ABI**（在 Jazzy 下 `undefined symbol rclcpp::QOSEventHandlerBase`）。本机**没有 Humble** → **当前跑不了**。两种解法：

**B-1 建一个 Humble distrobox（最贴近 README.md）：**
```text
distrobox create --name humble-mujoco --image ubuntu:22.04 --pull
distrobox enter humble-mujoco -- bash -lc '
  sudo apt-get update && sudo apt-get install -y curl gnupg lsb-release software-properties-common
  sudo add-apt-repository -y universe
  sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
  echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" | sudo tee /etc/apt/sources.list.d/ros2.list
  sudo apt-get update && sudo apt-get install -y \
    ros-humble-ros-base ros-humble-rclcpp ros-humble-sensor-msgs ros-humble-tf2-msgs ros-humble-statistics-msgs \
    libfmt8 libspdlog1 libx11-6 libxcb1
'
```
然后按 README.md 跑（终端 A sim / 终端 B runner）：
```text
# 终端 A（humble box）
distrobox enter humble-mujoco
cd /home/dongc1/workspace/HOPE/a3_deploy/a3_deploy_example/mujoco_sim_standalone
./run.sh a3_t2d5_cfg.yaml          # a3_t2d5 = 我们机型；GUI 里之后点 load-key

# 终端 B（hope/jazzy box，跑 pingpong runner；t2d5 默认 ros2 transport）
distrobox enter hope
cd /home/dongc1/workspace/HOPE/a3_deploy/a3_deploy_example/dist/a3_deploy_x86_64
source /opt/ros/jazzy/setup.bash; export LD_LIBRARY_PATH=".:${LD_LIBRARY_PATH}"
# 注意：standalone 的 pingpong runner cfg 走 iceoryx，而 a3_t2d5_cfg 走 ros2 → transport 不匹配。
# 因此路径 B 下，要么改用 iceoryx-only 的 sim cfg，要么给 runner 配 ros2 aimrt cfg。
```
> **坑：** `mujoco_sim_standalone` 里唯一的 iceoryx-only cfg 是 `a3_t2d0_iceoryx_cfg.yaml`，那是 **t2d0 机型**（不带 pingpong 球拍、DOF/顺序可能不符）。`a3_t2d5_cfg.yaml`（我们机型）走 ros2。所以**对 pingpong 任务，路径 A 的 `a3_pingpong` sim 才是对的机型**——这也是推荐 A 的原因。

---

## 验收标准（什么算“验证通过”）

| 维度 | 通过判据 |
|---|---|
| 通信/同步 | runner `rate≈50Hz`，`sync_complete`/`sync_aligned` 稳定，`infer_ms` < 20ms，`halts≈0` |
| obs 正确 | `proj_grav≈[0,0,-1]`，`base_quat≈[1,0,0,0]`，`time_step` 在 clip 上 sweep（forehand 0→~95），racket_target 合理 |
| 动作 | `|action|` 有界且随挥拍相位起伏（不是单调发散）；PD_STAND 站稳；MOTION level1 能看出 forehand；neck 不抖、wrist 稳 |
| 链路 | command 被 sim 收到（关节动）；neck 被动（q=0, kp=40, kd=2） |
| 预期内的“非通过” | **free-base 在 AGI sim 里 connect-gap 倾倒 / 显式-PD 发散** = sim 保真度，不计入 I/O 验证失败；以 **hoist** 结果为准 |

---

## Step 4 — 交叉编译 rockchip + 部署真机（仿真通过后的下一关）

rockchip pingpong binary **已编好并 staged**：`dist/a3_deploy_rockchip/a3_deploy_onnx_ref_pingpong`（7.1 MB, aarch64 ELF）。sysroot 已就位：`thirdparty/rockchip_sysroot/rockchip-1.0-aarch64-sysroot.tar.gz`。

重编（在 box 内，Docker + aarch64 工具链 + sysroot）：
```text
distrobox enter hope -- bash -lc '
cd /home/dongc1/workspace/HOPE/a3_deploy/a3_deploy_example
bash scripts/build_a3_deploy_pkg.sh --arch rockchip --jobs $(nproc)
'
# 同样记得把 model_15200.onnx + 两个 pingpong cfg 拷回 dist/a3_deploy_rockchip/{models,config}
```

部署到 MDU（`<hdu_wifi_ip>` 换成现场 HDU Wi-Fi 地址）——**仿真通过后**，HOISTED、低增益、急停在手：
```text
ssh -J agi@<hdu_wifi_ip> agi@10.42.10.12 'mkdir -p /agibot/a3_deploy'
rsync -azP -e "ssh -J agi@<hdu_wifi_ip>" dist/a3_deploy_rockchip/ agi@10.42.10.12:/agibot/a3_deploy/
# 在 MDU 上：
#   source /opt/ros/jazzy/setup.bash
#   cd /agibot/a3_deploy
#   ./a3_deploy_onnx_ref_pingpong --runtime-cfg config/a3_runtime_config.pingpong.yaml \
#       --start passive --legs-passive --gain-scale 0.4
#   然后键 p→s→h→m→0→1
```

---

## 速查：每次 run 的 5 步

1. `distrobox enter hope` ×2 个终端
2. 两个终端都 `source /opt/ros/jazzy/setup.bash`
3. 终端1 起 sim（hoist cfg），终端2 起 runner（`--legs-passive`）
4. runner 里走 `p→h→s→m→0→1`，对照「验收标准」
5. 收工：`pkill -9 -x aimrt_main; pkill -9 -x iox-roudi; rm -f /dev/shm/iox*`
