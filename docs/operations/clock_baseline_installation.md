# HDU clock baseline installation and current Laptop NTP access

Use this procedure with the [new-robot commissioning checklist](new_robot_commissioning.md).
Set `REPO_ROOT` to the current HOPE checkout and `HDU_IP` / `LAPTOP_WIFI_IP` to the
verified current addresses. Historical addresses below are examples, never defaults.
For a guarded BSP, follow the paired-reboot branch in the commissioning checklist
before invoking any installer; ordinary `--install` deliberately rejects it.

### 6.2.1 新机器人：验证 HDU 时钟基线与 Laptop NTP 访问

> **新 BSP 分支：** 先读实际 `/opt/agibot/entry/run_agibot.sh`。若调用
> `system/timesync_guard.py`，不要执行下文普通热安装/恢复路径；按
> [guarded BSP 首次安装](new_robot_commissioning.md#新-bsp-的-timesync_guardpy必须先分支)
> 使用明确的双板重启模式。普通安装器与手动校时会提前拒绝不兼容的热恢复。

旧机器人已经做过的 chrony/bootstrap 改造不会随第 5 节源码同步到新 HDU。当前
Time Calibration coordinator 明确依赖“`agibot_pm.service` 拥有 HDU 的
`ptp4l/phc2sys`”这套已验证拓扑；不能拿另一套独立 HDU PTP unit 临时替代。仍在
**HDU 干净 shell** 执行：

```bash
(
set -eo pipefail

for TOOL in chronyc pgrep sha256sum cmp grep; do
  command -v "$TOOL" >/dev/null || {
    echo "STOP: HDU clock baseline is missing tool: $TOOL" >&2
    exit 1
  }
done

for FILE in \
  /opt/agibot/entry/system/timesync.sh \
  /etc/chrony/chrony.conf \
  /etc/chrony/chrony-bootstrap.conf \
  /usr/local/sbin/agibot-clock-bootstrap \
  /etc/systemd/system/agibot-clock-bootstrap.service \
  /etc/systemd/system/chrony.service.d/agibot-controls.conf \
  /etc/systemd/system/agibot_pm.service.d/20-clock-ordering.conf; do
  test -r "$FILE" || {
    echo "STOP: approved HDU clock baseline file is missing: $FILE; use 6.2.1.1" >&2
    exit 1
  }
done

test "$(sha256sum /opt/agibot/entry/system/timesync.sh | awk '{print $1}')" = \
  80c35a34fef2f7b7abd845e09c7c3770dd9a863ed68e9caa71f6f4161afccc5b

systemctl cat chrony.service >/dev/null
systemctl cat agibot-clock-bootstrap.service >/dev/null
systemctl is-enabled --quiet chrony.service
systemctl is-active --quiet chrony.service
BOOTSTRAP_STATE="$(systemctl is-enabled agibot-clock-bootstrap.service 2>/dev/null || true)"
test "$BOOTSTRAP_STATE" = disabled

mapfile -t HDU_PTP4L_PIDS < <(pgrep -x ptp4l)
mapfile -t HDU_PHC2SYS_PIDS < <(pgrep -x phc2sys)
test "${#HDU_PTP4L_PIDS[@]}" -eq 1
test "${#HDU_PHC2SYS_PIDS[@]}" -eq 1

for PID in "${HDU_PTP4L_PIDS[@]}" "${HDU_PHC2SYS_PIDS[@]}"; do
  grep -Eq '^[^:]*:[^:]*:/system[.]slice/agibot_pm[.]service(/.*)?$' \
    "/proc/$PID/cgroup" || {
      echo "STOP: HDU clock pid $PID is not owned by agibot_pm.service" >&2
      exit 1
    }
done

pgrep -a -x ptp4l | \
  grep -F -- '-i eth_hdu -2 -E -m --tx_timestamp_timeout 100'
pgrep -a -x phc2sys | \
  grep -F -- '-s CLOCK_REALTIME -c eth_hdu'

DROPIN_SOURCE="$HOME/hope_ws/src/hope_bringup/config/chrony-hope-laptop-server.conf"
DROPIN_TARGET=/etc/systemd/system/chrony.service.d/30-hope-laptop-server.conf
grep -Fx \
  'ExecStartPost=/usr/bin/chronyc allow 10.42.0.1/32' \
  "$DROPIN_SOURCE"

if ! sudo cmp -s "$DROPIN_SOURCE" "$DROPIN_TARGET"; then
  BACKUP_DIR="/var/backups/hope-clock-laptop-ntp-$(date -u +%Y%m%dT%H%M%SZ)"
  sudo install -d -m 0755 "$BACKUP_DIR"
  if sudo test -e "$DROPIN_TARGET"; then
    sudo cp -a "$DROPIN_TARGET" "$BACKUP_DIR/"
  fi
  sudo install -D -o root -g root -m 0644 \
    "$DROPIN_SOURCE" "$DROPIN_TARGET"
  sudo systemctl daemon-reload
  sudo systemctl restart chrony.service
fi

systemctl is-active --quiet chrony.service
sudo chronyc accheck 10.42.0.1 | grep -F '208 Access allowed'
chronyc -n sources -v
chronyc tracking
sudo /opt/agibot/entry/system/timesync.sh --runtime-status || true

echo "HDU_CLOCK_BASELINE_OK"
)
```

必须看到 `HDU_CLOCK_BASELINE_OK`。`chronyc tracking` 此时允许显示 offset 尚未进入
10 ms；第 10.2/10.4 节负责资格判定和受控校时。但 core 文件、固定 `timesync.sh`
hash、PTP 进程数量/命令/cgroup、chrony active 或 Laptop `accheck` 任一不符时，必须
在安装 HOPE 服务前停止。

新 HDU 缺少任一 core 文件时，按下面 6.2.1.1 安装仓库固定的完整 package；不要从旧
PDF 片段、MDU unit 或其他分支拼装。第 10.4 节仍只是已安装基线上的维护动作，不是
新机器人安装器。

#### 6.2.1.1 缺少 HDU clock core：安装固定、可回滚的基线 package

本路径只处理 6.2.1 明确报告的 core 文件缺失或固定 `timesync.sh` hash 不符。当前 package
的 `MANIFEST.sha256` 为
`92fa52faef8a89608be2c90bad3a4250803c624c61d87d96f127b3ed3fdc2168`；HDU
`timesync.sh` 为 `80c35a34fef2f7b7abd845e09c7c3770dd9a863ed68e9caa71f6f4161afccc5b`，
并保留 vendor-owned Layer-2/E2E/domain-0 PTP，只加入已验证的
`--tx_timestamp_timeout 100`。不要换用含 `agibot-hdu-ptp4l.service` 的另一版 package，
否则会把 HDU PTP 所有权从 `agibot_pm.service` 改掉。
China profile 仍要求至少两个独立阿里云样本；其 `maxdelay` 为 `0.50 s`，用于覆盖
当前 venue Wi-Fi 实测约 `0.37–0.41 s` 的 NTP RTT。它不会替代第 10.4 节的
`Leap status Normal` 与绝对 offset `≤10 ms` 最终硬门槛。

先确认当前旧检查已经返回 `bash-5.2$`。不要继续 6.3；在 **HDU 干净 shell** 执行：

```bash
exit
```

看到 Laptop 的 `dongc1@...` 提示符后，在 **Laptop HOST** 执行下面整个 stage block：

```bash
(
set -euo pipefail

cd /home/dongc1/workspace/Hope_v11
CLOCK_PACKAGE=$PWD/agibot/ntp_sync
CLOCK_INSTALLER=$PWD/foxglove/a3/install_hdu_clock_baseline.sh

test "$(sha256sum "$CLOCK_PACKAGE/MANIFEST.sha256" | awk '{print $1}')" = \
  92fa52faef8a89608be2c90bad3a4250803c624c61d87d96f127b3ed3fdc2168
test "$(sha256sum "$CLOCK_INSTALLER" | awk '{print $1}')" = \
  419a103d3557ffd0172bf826d7a707a5d826daa6605fc973d9707f6fb842c534
"$CLOCK_INSTALLER" --verify-package-only "$CLOCK_PACKAGE"

ssh agi@10.42.0.73 \
  'install -d -m 0755 "$HOME/hope_clock_baseline"'
rsync -azP \
  "$CLOCK_PACKAGE/" \
  agi@10.42.0.73:~/hope_clock_baseline/
scp "$CLOCK_INSTALLER" \
  agi@10.42.0.73:/tmp/hope-install-hdu-clock-baseline
ssh agi@10.42.0.73 \
  'chmod 0755 /tmp/hope-install-hdu-clock-baseline &&
   /tmp/hope-install-hdu-clock-baseline \
     --verify-package-only "$HOME/hope_clock_baseline"'

echo "HDU_CLOCK_PACKAGE_STAGED_OK"
)
```

必须先看到 Laptop 本地和 HDU 各一次 `HDU_CLOCK_PACKAGE_VERIFY_OK`，最后看到
`HDU_CLOCK_PACKAGE_STAGED_OK`。完成后保持在 **Laptop HOST** 提示符，不需要先进入 HDU。

如果提示符是 `dongc1@...`，直接运行裸命令
`sudo /tmp/hope-install-hdu-clock-baseline ...` 会得到 `command not found`，因为该 `/tmp`
路径位于 HDU，不在 Laptop；这个错误不表示 HDU 上的 stage 失败，也没有开始安装。此时从
上面的 stage block 重新执行是安全的。

下一步会停止并恢复当前 active 的 `agibot_roudi`、`agibot_top`、`agibot_ui` 和
`agibot_pm`，同时短暂停止 chrony/PTP。执行前必须由现场人员物理支撑机器人，禁用运动和
外部命令，并让实体急停保持可触达。满足这些条件后，仍在 **Laptop HOST** 执行下面完整的
SSH 命令；不要只复制其中的 `sudo /tmp/...` 部分：

```bash
ssh -tt "agi@10.42.0.73" \
  'sudo /tmp/hope-install-hdu-clock-baseline --install "$HOME/hope_clock_baseline"'
```

安装器会再次校验完整 package 和 HDU 身份，显示危险说明，并要求手工输入
`INSTALL_HDU_CLOCK_BASELINE`。它在 `/var/backups/hope-hdu-clock-baseline-<UTC>` 保存所有
被替换文件和原 active-service 状态；安装或复启动任一步失败时会自动恢复并打印
`HDU_CLOCK_BASELINE_ROLLED_BACK`。如果任何文件或服务没有完整恢复，安装器会改为打印
`HDU_CLOCK_BASELINE_ROLLBACK_INCOMPLETE`；看到它必须保持机器人受支撑且不继续部署，并保留
完整终端输出。依赖包如果刚由 apt 安装则保留，不会在 rollback 中卸载。

如果旧安装器在复制文件前报告
`agibot-clock-bootstrap.service: Command /usr/local/sbin/agibot-clock-bootstrap is not executable`
并随后打印 `HDU_CLOCK_BASELINE_ROLLED_BACK`，不要直接重跑 HDU `/tmp` 中的旧副本。该版本把
unit 验证放在了绝对路径可执行文件安装之前；先回到 Laptop，重新执行上面的整个 stage
block，并确认安装器 hash、两次 `HDU_CLOCK_PACKAGE_VERIFY_OK` 和
`HDU_CLOCK_PACKAGE_STAGED_OK`，再重新进入有人值守的安装步骤。

只有看到 `HDU_CLOCK_BASELINE_INSTALLED_OK`，才按 6.1 进入 **HDU 干净 shell**，并逐字
重跑 6.2.1 的完整验证 block。重跑负责安装当前 Laptop `10.42.0.1` 的 chrony access
drop-in；最终仍必须看到 `HDU_CLOCK_BASELINE_OK`，然后才可进入 6.3/6.3.1。


### 6.6 Laptop HOST：把 chrony source 切到当前新 HDU

第 6.2.1 节只允许 HDU 向当前 Laptop 提供 NTP；Laptop 端还必须从旧机器人地址切到
当前 `10.42.0.73`。退出 HDU、回到 **Laptop HOST** 后执行：

```bash
(
set -euo pipefail

cd /home/dongc1/workspace/Hope_v11
test "$(grep -Ec '^server 10[.]42[.]0[.]73([[:space:]]|$)' \
  hope_ws/src/hope_bringup/config/hope-hdu.sources)" -eq 1

sudo install -D -o root -g root -m 0644 \
  hope_ws/src/hope_bringup/config/hope-hdu.sources \
  /etc/chrony/sources.d/hope-hdu.sources
sudo systemctl restart chrony.service
sudo chronyc burst 8/8 10.42.0.73

chronyc waitsync 120 0.010 0 1
chronyc -n sources -v | \
  grep -E '^\^\*[[:space:]]+10[.]42[.]0[.]73([[:space:]]|$)'
chronyc tracking

echo "LAPTOP_CURRENT_HDU_NTP_OK"
)
```

这里不自动 `makestep` Laptop 时钟。120 次等待后仍未选中当前 HDU 或 offset 仍大于
10 ms 时停止，先检查 HDU chrony、路由和防火墙；不要继续使用旧 HDU 的 NTP sample，
也不要把机器人第 10.4 节的 hard-step 命令复制到 Laptop。只有看到
`LAPTOP_CURRENT_HDU_NTP_OK` 才继续 MDU 部署。

---
