# Host 显示与按角色录制

## 数据所有权

动捕、完整球轨迹、FK 和场景几何在 Laptop/Host 上处理。Foxglove 使用 Host
本地端点 `ws://127.0.0.1:8767`，由 `hope-host-bridge.service` 连接
`laptop-runtime.env` 指定的当前 HDU。`/hope/live/*` 从 Host ROS 直接进入
Host bridge，不再经过 HDU WebSocket 返回 Host。控制服务、参数和客户端发布
白名单从 `bridge_params_control.yaml` 生成；只开放原有控制入口和操作员心跳。
本地端点只监听 loopback；它不会自行发送心跳、控制命令或启动机器人。

原 HDU 8766 控制入口保留作直连回退。HDU 两座 bridge 的白名单排除
`/hope/live/*`，防止旧客户端重新建立场景绕行。切换回 HDU 后控制/状态可用，
Host live scene 在该数据源不可见。切换连接要在非 MOTION 状态进行，之后验证
新的真实 Console 心跳和实际 Runner 状态，不能用测试程序代发心跳掩盖问题。

## Host 安装与验收

要求已有 `hope` distrobox、ROS Jazzy `foxglove_bridge`、PyYAML 和 matching
`hope_ws/install`，并有本次机器的 `~/.config/hope-foxglove/laptop-runtime.env`。
在仓库根运行：

```bash
bash foxglove/laptop/install_host_bridge.sh
systemctl --user status hope-host-bridge.service
```

在 Foxglove 打开 `ws://127.0.0.1:8767`，或使用链接：
`foxglove://open?ds=foxglove-websocket&ds.url=ws%3A%2F%2F127.0.0.1%3A8767`。
沿用现有布局。机器人切换按钮仍是 HDU 直连入口；更换机器人后，先确认
Laptop runtime env 的 HDU 地址，再重启 Host bridge 并重新核对显示的机器人身份。
不把 localhost 连接成功当作已经连对机器人。

核对实时 scene/TF（目标 20 Hz，TF 仅在有新姿态时发布）、telemetry（5 Hz）、Runner mode、NTP 和操作员 challenge；
使用真实 Console 检查 echo，检查服务广告是否保留原有 allowlist。ROS graph 中
`/hope/live/*` 的订阅者应只有 Host bridge，不应出现 HDU bridge。
Host bridge 采用正常调度优先级，保证服务与心跳；live scene 仍为 nice +10。
库存 Host bridge 不包含 HDU 的 display-thread patch，不宣称其线程单独降优先级。
Host live scene 默认每 50 ms 更新球轨迹和姿态显示；全速模式每次 timer 回调均发布，
避免再次比较墙钟间隔时因微小抖动跳帧。此频率只影响 Host 显示发布，
不会提高动捕采样、HDU 控制或录制频率。修改显示频率后仅重启
`systemctl --user restart hope-live-scene.service`，无需重启 Host bridge 或机器人。

## 录制分工

`hope_sim2real_capture.py` 为 Laptop 与 HDU 使用不同的 topic 列表：

- Laptop：保留现有回放所需 topics，并补录 `/poses/planner_full`、
  `/poses/operator_raw`、`/ball/operator_raw_point`，保留门控前的完整球观测。
- HDU：不再订阅 P1/P2 原始姿态、原始球位置、markers 和原始 OptiTrack 数据。
  保留 `/ball/flight_packet`、`/a3/base_pose_laptop_flat`、`/a3/base_pose_flat`、
  `/racket/command_flat`、Runner 状态和诊断；补录 `/serve/ball_state_flat`。
  base relay 两侧和 flight packet 两端的记录是有意保留的时序证据。
- MDU：原 body recorder、Runner trace 和 policy obs/action 不变。

完整原始观测须从 Host 导出，不能只拿 HDU 包当作完整动捕数据。
修改 helper 不会改变已经运行的 recorder。通过既有 Record 开关关闭旧段、
确认三端 OFF 后再开启新段；不要 kill wrapper，因为它可能拥有 Planner/Runner。
录制轮换会有可辨认的分段间隙；核对三端同一新 segment、MCAP 正常关闭及
recorder argv/实际订阅后才报告生效。保留旧段，不删除任何历史记录。

## 厂商脸屏、语音与相机

`hope-monitor` 是时钟、健康与软件急停代理，不是脸屏。脸屏是
`agibot_ui.service` / `flutter-pi`。经使用者明确要求，可独立执行：

```bash
sudo systemctl disable --now agibot_ui.service
```

语音交流通过厂商 `AgentControlService/GetVoiceEnable` 核对，必要时用同一服务
`SetVoiceEnable` 的 `enable_voice:false` 关闭，并核对
`/agibot/data/var/agent/speech_config.json` 的持久值。不要以 kill 整个 `agent` 或
`hal_audio` 代替关闭交流功能；它们还参与启动状态、提示音和厂商控制链。
`enable_voice=false` 证明交流功能关闭，不等于所有云连接或整个 Agent CPU 消失。

双目相机由 `hal_hdu_camera` 管理：通过 EmAppService/StopApp 停止并检查 IDLE、
无 camera 进程。若要求重启后仍关闭，备份本机实际 `APM_RUN_CFG`，只从
`default_apps` 删除 `hal_hdu_camera`，保留 app 定义和其他应用，不为此重启
process manager。既有 HOPE catch preflight 继续检查相机关闭。

## 回退与验收边界

Host bridge 可独立 `systemctl --user disable --now hope-host-bridge.service`；
先让真实 Console 在非 MOTION 窗口切回已确认的 HDU 8766。需要旧 live 绕行时，
恢复备份的两份 HDU 白名单并在同一受控窗口重启两座 bridge。
录制 helper 回退也在下一次受管录制分段生效。脸屏可 `enable --now` 恢复；
相机启动清单按本次备份回退；语音按原来的特性值恢复。

CPU 采用同长度窗口、单核 100% 的进程增量统计。禁止将整个进程的 CPU 变化都
归因于一个 topic；减负不证明定位 P99、policy 跟踪或物理击球已经改善。
是否跨重启生效须单独验证，不能从 enable/disable 和配置文件推断重启验收通过。

## HOPE source integration

The optional scene files are included because they were previously local-only.
Geometry comes from the maintained `a3_deploy/a3_deploy_example/assets/a3_runtime/robots/A3PingPong-with-gripper`
asset. This is display FK, not physical mount calibration. Check the model against the installed robot.
Install NumPy and SciPy in the `hope` distrobox. After installing the Host bridge,
set `HOPE_LIVE_MDU_HOST` in `~/.config/hope-foxglove/host-bridge.env` to the SSH
host for the **selected** MDU (with its existing approved ProxyJump, if needed).
This explicit selection replaces the field-only hardcoded SSH alias.
Install `foxglove/laptop/hope-live-scene.service` to `~/.config/systemd/user/`,
run `systemctl --user daemon-reload`, then enable/start it in a stopped test window.
Select `/hope/live/scene`, `/hope/live/robot` and `/hope/live/tf` in standard Foxglove
3D panels; custom field-only panels are not required. The robot remains hidden
when the selected Runner has no current session-owned trace samples.

These helpers use main's existing unicast interface policy; they do not install
field-only routing/QoS experiments. Preserve full-rate mocap acquisition; 20 Hz
is solely the scene publication target, while text telemetry remains 5 Hz.
Timing topics are recorded on the Host if the optional VRPN timing feature is installed.
