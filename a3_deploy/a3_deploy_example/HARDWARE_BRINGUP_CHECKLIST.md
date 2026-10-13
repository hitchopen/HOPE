# A3 真机硬件链路验证 Checklist（自带策略 / sim-to-real 地基）

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

目标：在不依赖你自己 BeyondMimic policy 的前提下，用 Agibot 自带策略把
`a3_deploy_onnx_ref` 跑到**真机 A3** 上，验证 step 18 的全部硬件链路：
网络 → `hal_ethercat` → 六路状态同步 → 推理延迟 → `PD_STAND` → 动作 → 急停。

为什么先做这个：你的 `hope_forehand_policy.onnx` 和这个部署程序接口不兼容
（两输入七输出 / 31-DOF vs 单输入单输出 / 29-DOF view，obs 维度 A3 是 `[1,1570]`），
直接指过去会输出垃圾动作（危险）。所以先用**自带 rknn 策略**把真机链路打通，
再回去补 obs/action 适配桥 + sim-to-sim。本文件不碰你的训练产物。

> 自带策略跑的是通用 A3 动作，不是乒乓挥拍。这一步证明的是 harness/硬件，不是球技。

---

## 0. 现场要先填的占位符

| 占位符 | 含义 | 怎么拿 |
| --- | --- | --- |
| `<HDU_WIFI_IP>` | HDU 跳板机的现场 Wi-Fi IP | 现场网络分配；`ping` 通即可 |
| `<MDU_IP>` | 机器人内网计算单元地址 | 文档默认 `10.42.10.12`，以现场为准 |
| 计算单元类型 | Rockchip/MDU（默认）还是 Thor/ADU | 看你现场是哪台；本文按 **Rockchip/MDU + iceoryx** |

Thor/ADU 的话：下面所有 `rockchip` 换成 `thor`，transport 换 `ros2`。

---

## 1. 物理预检（上电前 / 通电后，缺一不可）

- [ ] 机器人挂安全绳 / 吊架，脚可离地或在低功率支撑下。
- [ ] 确认硬件急停按钮位置，**先手动按一次再松开**，确认回路正常。
- [ ] 手臂活动半径 + 1.5m 内无人无障碍物。
- [ ] 开发机 ↔ HDU ↔ MDU 网络/网线就位，`ping <HDU_WIFI_IP>` 通。
- [ ] 软件急停（`q` 退出 / `p` 进 PASSIVE / Ctrl+C）操作者已经知道在哪敲。

---

## 2. 开发机：交叉编译 Rockchip 部署包

需要 Docker（交叉编译走 Docker builder，sysroot 已随仓库带好）。

> 说明：本 checklist 全部是 **Linux 命令**（`taskset`、`systemctl` 本就是 Linux 专有，
> macOS 上没有）。下面第一行的 `find ... -delete` 也是 Linux 命令——它删的是当初从
> 飞书/macOS 拷进仓库的 `._*` 垃圾文件；**现在仓库里已经是 0 个**，所以这行空跑无害，
> 留着是防止以后再拷入时编译被 glob 带坏。

```text
cd ~/workspace/HOPE/a3_deploy/a3_deploy_example
find . -name '._*' -type f -delete                      # Linux 命令：删 ._* 垃圾文件（现已 0 个，空跑无害）
bash scripts/build_a3_deploy_pkg.sh --arch rockchip --jobs 20
ls -la dist/a3_deploy_rockchip/                          # 应看到 a3_deploy_onnx_ref + run_a3.sh + assets
```

打包会自动把 `onnx.backend` 切成 `rknn`，并带上 `model_step_098000_a3.rknn`（已确认存在）。

---

## 3. 开发机：经 HDU 跳板把包同步到 MDU

```text
cd ~/workspace/HOPE/a3_deploy/a3_deploy_example
ssh -J agi@<HDU_WIFI_IP> agi@<MDU_IP> 'mkdir -p /agibot/a3_deploy'
rsync -azP -e "ssh -J agi@<HDU_WIFI_IP>" \
  dist/a3_deploy_rockchip/ \
  agi@<MDU_IP>:/agibot/a3_deploy/
```

> 源路径末尾的 `/` 不能去掉——去掉会在目标下多套一层 `a3_deploy_rockchip/`。

---

## 4. MDU 终端 A：停系统服务，手动启 EtherCAT

```text
ssh -J agi@<HDU_WIFI_IP> agi@<MDU_IP>
sudo systemctl stop agibot_pm
source /agibot/software/v0/entry/env/env.sh
cd /agibot/software/v0
bash scripts/hal_ethercat/start_hal_ethercat.sh
```

等 `hal_ethercat` 起来后**保持这个终端开着**（它在收发关节/IMU）。

---

## 5. MDU 终端 B：确认状态 topic 在发

```text
ssh -J agi@<HDU_WIFI_IP> agi@<MDU_IP>
cd /agibot/a3_deploy
file ./a3_deploy_onnx_ref            # 确认是 aarch64
source ./setup_ros2_msgs.bash
ros2 topic hz /body_drive/arm_joint_state
```

> ⚠️ **MDU（iceoryx）上 `ros2 topic hz` 永远看不到这些 topic**——它们走 iceoryx 共享内存，
> 不在 DDS graph 上，`ros2` CLI 只看 DDS。所以这里报 `does not appear to be published yet`
> **是正常的，不代表 ethercat 没起**。`setup_ros2_msgs.bash` 只负责 source ROS + 设 ament/lib 路径，
> 不能把 iceoryx 桥进 DDS。**MDU 上真正的「六路 topic 在不在」判据是第 6 步的 `--dry-run`**
> （它走部署程序自带的 iceoryx 订阅）。`ros2 topic hz` 只在 Thor/ADU（ros2 transport）上才有意义。

> 🩹 **`setup_ros2_msgs.bash: 没有那个文件或目录`**：老的 rockchip 包漏带了这个文件
> （build 脚本已修，新包/重新 rsync 会自带）。现场如果还缺，直接在机器上原地补：
> ```text
> cd /agibot/a3_deploy
> cat > setup_ros2_msgs.bash <<'EOF'
> #!/usr/bin/env bash
> SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
> if [[ -z "${ROS_DISTRO:-}" ]]; then
>   for d in jazzy humble; do
>     [[ -f /opt/ros/$d/setup.bash ]] && { set +u; source /opt/ros/$d/setup.bash; set -u 2>/dev/null; break; }
>   done
> elif [[ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
>   set +u; source "/opt/ros/${ROS_DISTRO}/setup.bash"; set -u 2>/dev/null
> fi
> export AMENT_PREFIX_PATH="${SCRIPT_DIR}:${AMENT_PREFIX_PATH:-}"
> export LD_LIBRARY_PATH="${SCRIPT_DIR}:${LD_LIBRARY_PATH:-}"
> EOF
> source ./setup_ros2_msgs.bash
> ```

**结论：MDU 上别卡在第 5 步**——`source` 完（哪怕缺文件就用上面补）直接进第 6 步，用 `--dry-run`
看六路 ready 才是判据。看不到 dry-run 的 ready → 回终端 A 确认 ethercat 起来了。

---

## 6. dry-run → probe → run（三段递进，绝不跳）

`taskset -c 4-7` 把进程绑到 RK3588 性能核，降低推理延迟抖动。
**上机不要用 `--auto-start`。**

```text
cd /agibot/a3_deploy

# 6a) 只收状态、不加载策略、不发命令。
#     看：waist/leg/arm/neck/pelvis_imu/torso_imu 六路 ready，sync_complete / sync_aligned 稳定。
taskset -c 4-7 ./run_a3.sh --dry-run

# 6b) 跑推理 + 延迟统计，仍不发命令。
#     看：infer_ms 低于控制周期（50Hz → 20ms），header/group skew、sample age 都在阈值内。
A3_LATENCY_LOG=verbose taskset -c 4-7 ./run_a3_probe.sh

# 6c) 正式手动状态机（开始会发关节命令）。
taskset -c 4-7 ./run_a3.sh
```

> Rockchip arm 包默认会 source 机器人 env（`A3_SOURCE_ROBOT_ENV=1`）。如果它报
> `required robot env not found`，先在本终端 `source /agibot/software/v0/entry/env/env.sh` 再跑。

---

## 7. 手动状态机（在 **deploy 终端** 里敲键，别敲错窗口）

| 按键 | 行为 |
| --- | --- |
| `p` | PASSIVE（被动，最安全的回退） |
| `s` | PD_STAND（站立，**等约 3 秒确认稳定**） |
| `m` | MOTION（PD_STAND 没稳之前**绝不**进） |
| `r` / 空格 | 播放当前动作 |
| `q` | 退出 |

推进顺序：`p` → `s`（稳）→ `m` → `r`。
**首次只做短时间、小幅度动作，全程握住急停。** 任何异常立刻按硬件急停 / `p` / Ctrl+C。

---

## 8. 通过标准（verification gate）

- [ ] dry-run：六路 topic ready，`sync_complete` / `sync_aligned` 稳定。
- [ ] probe：`infer_ms` < 20ms，skew / sample age 在阈值内，watchdog 不误触发。
- [ ] run：`s` 后机器人稳定站立；`m` + `r` 后能跑完一小段动作不摔。
- [ ] 急停：硬件急停能在 200ms 内停住上肢和步态（competition 要求；用 120fps 录像或日志量）。
- [ ] 关节状态 name / order 与目标机一致，command 向量长度正确。

跑通这 8 条 = step 18 的真机/网络/同步/PD_STAND/急停全链路已验证。
下一步才是：写 obs/action 适配桥 → 过 16.2 sim-to-sim → 用你自己的 policy 重复 6–8。

---

## 9. 排错速查

| 现象 | 处理 |
| --- | --- |
| `RouDi not found` / `Timeout registering at RouDi` | iceoryx RouDi 没起。终端 A 的 ethercat / sim 必须先跑；清残留 `pkill -f iox-roudi; pkill -f aimrt_main` |
| dry-run `complete=0 ... not all 6 topics ready` | 没收到状态：ethercat 没真正在发，回终端 A 看 |
| 按 `s`/`m` 没反应、显示 `ignored` | 你敲到别的窗口了；焦点切回 **deploy 终端**（它读自己终端的 stdin） |
| 进了 `mode=motion` 但 `playing=0 hold=1` | 动作没播放，按 `r` 或空格 |
| `required robot env not found` | 本终端先 `source /agibot/software/v0/entry/env/env.sh` |
| `file ./a3_deploy_onnx_ref` 显示 x86-64 | 传错包了，必须是 `dist/a3_deploy_rockchip/`（aarch64） |
| `setup_ros2_msgs.bash: 没有那个文件或目录` | 老 rockchip 包漏带（build 脚本已修）。按第 5 步的 🩹 块原地补一份，或重新打包/rsync |
| `ros2 topic hz` 报 `does not appear to be published yet` | MDU 正常现象：topic 走 iceoryx 不在 DDS graph，`ros2` CLI 看不到。**别管它**，用第 6 步 `--dry-run` 判六路 ready |
| 推理延迟抖动大 | 确认用了 `taskset -c 4-7`，并且没有 `--auto-start` |

参考来源：[a3_deploy_example/README.md](README.md) “Rockchip/MDU 上机验证” + “手动状态机” + “安全与调试”；
[../../docs/operations/deploy_and_gate3.md](../../docs/operations/deploy_and_gate3.md) step 16.2 / step 18。
