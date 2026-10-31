# Guarded BSP：开机校时与时间同步按钮

开机校时负责消除 RTC 重载误差；Console 的 Time Calibration 按钮负责在 STOPPED 状态
重新执行一次校准。两者都必须通过真实 NTP gate，不能用服务 running 或按钮请求已接受
代替完成。普通 START、Stand、Serve 不执行 hard-step。

## 安装边界

此适配器只支持 `timesync_guard.py` SHA-256
`8850128968c84cadf31f920fa0b3ce76d2bb4c6aa93aa56b5bcaf5f987f278da`，以及现有固定
`agibot/ntp_sync` timesync 基线。未知 BSP 或厂商更新应先审查，安装器在修改前拒绝。
不要为了安装成功修改 hash。适配器不改变 PTP 方向、参数或所有权。

完整 setup 顺序：安装 NTP/PTP 基线 → 选择可用的区域 NTP 源 →
[启用 HOPE 开机校时](new_robot_commissioning.md#hope-开机一次性校时-setup) →
安装下述按钮适配 → 分别验收正常整机开机和停止状态的按钮校时。

在 HDU 使用与实际控制面匹配的 HOPE 源码。停止 demo，机器人有物理支撑、实体急停
可触达，Console 必须是新鲜 STOPPED / idle；PASSIVE 不符合要求。备份实际安装的
coordinator/core/unit 和厂商 guard，确认没有其他 lifecycle 操作持有硬件锁。
原生 C++ operations 与 Python operations 的 core 有各自的进程验证逻辑；不能从旧分支
单独覆盖 core，必须保留本机已匹配的控制面支持。

```bash
cd ~/HOPE
# --install 只安装适配器，记录 /var/backups/hope-guarded-clock-*，不重启服务。
sudo python3 foxglove/a3/hope_guarded_clock.py --install
sudo /usr/local/libexec/hope-guarded-clock --check
# 应输出 GUARDED_CLOCK_ADAPTER_V1。
# 按现有匹配版本的部署流程安装 hope_time_calibration_core.py，并备份被替换模块；
# 停止状态下重启 coordinator 使它加载新 core，不要手动执行 --arm 或 --launch。
sudo systemctl restart hope-time-calibration.service
```

`timesync_guard.py` 变成固定入口，原厂完整内容保存在同目录的
`timesync_guard.vendor.py`，并在每次使用前校验固定 hash。没有维护凭据的正常启动，
仍执行该原厂副本：MDU 必须是真实新启动，uptime 服务未知/不可达仍不能绕过。
脚本安装成功不代表 core 已部署，也不代表按钮已验收。

## 按钮执行过程

1. 保留原有新鲜 STOPPED、idle、NTP 数据、配置确认和全局硬件锁检查。
2. 停止并核实两板的时间消费者，执行一次 HDU bootstrap，等待 correction ≤10 ms。
3. coordinator 在 `HDU_RESTORE` 状态产生 root-only 一次性凭据。凭据绑定本次 HDU
   boot ID、操作 ID、coordinator PID 与启动时间，使用 monotonic 时钟限制为 60 秒。
4. 厂商 PM 恢复时，适配器原子消耗凭据，再核实 coordinator/锁仍有效、HDU 时间合格、
   本机无 PTP worker，以及 MDU 的当前 boot 停止凭据与实际 HAL/PTP/Runner 停止状态。
   检查通过后才在原厂 PM cgroup 内运行原 timesync.sh；不伪造 MDU uptime。
5. coordinator 读到本次恢复回执并确认 HDU PTP workers 后，才恢复 MDU 和 Foxglove。
   完成/失败均撤销凭据；失败按原有 fail-safe 停住机器人服务。

失效、过期、跨 boot、PID 重用、运行中消费者或身份不明不会降级为无保护恢复。
适配器没有任意命令入口。普通启动仍有 BSP guard；平时 chrony、PTP、PM/HAL 仍运行。

## 必须保存的验收证据

- 正常整机开机：新的两板 boot ID，bootstrap success 与 `bootstrap-qualified`，其完成
  时间先于 PM/HOPE 服务；新鲜 NTP gate=true、correction ≤10 ms；MDU `SLAVE`、
  `gmPresent=true` 和持续更新的 PTP/phc2sys offset。单板异常重启仍受原厂 guard 限制。
- 手动按钮：从新鲜 STOPPED 点击，记录请求操作 ID、`HARD_STEP`、
  `HOPE_GUARDED_CLOCK_RESTORED`、最终 `COMPLETE / CLOCK_QUALIFIED / SERVICES_RESTORED`。
  确认两板 boot ID 不变、PTP 拓扑恢复、8766 自动重连，coordinator DDS 回收后仍可发现。
- 同一 boot 再点击一次：必须生成新的维护操作并重新完成；不存在凭据重放。
- RUNNING / Stand / Serve 不允许按钮校时。失败时保持停机，不重复点击越过恢复要求。
- Laptop 与动捕电脑分别核对自己的时钟和帧时间。上述证据不证明物理动作或全部
  Foxglove 功能均已验收；动作验收使用对应功能的既有流程。

## 回滚

停止状态、无活动校时操作时，使用安装器打印的备份目录恢复原厂 guard 与旧 helper，
并恢复本机 core 的部署备份。原厂 guard 的已校验完整副本也保留在
`/opt/agibot/entry/system/timesync_guard.vendor.py`。回滚后未适配的 guarded BSP
重新返回 `GUARDED_BSP_REQUIRES_PAIRED_REBOOT`；不得保留旧凭据强行热恢复。
开机校时配置独立：其备份位于 `/var/backups/hope-boot-clock-*`，按备份恢复原 drop-in
和 enabled/disabled 状态，再 daemon-reload。所有回滚与再次启动均按受控维护流程执行。

## 验证状态

2026-10-31 已实测开机自动校时、ROS NTP gate=true、MDU PTP 恢复和 STOPPED 状态。
本次 demo 开始后暂停了新的机器人修改。按钮适配当前通过离线回归，实机安装、两次
按钮校时与恢复验收仍待 demo 停止后完成；此项未完成前 PR 保持 draft。
