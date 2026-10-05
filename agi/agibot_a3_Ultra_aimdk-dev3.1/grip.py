#!/usr/bin/env python3
"""夹爪一条命令抓/放工具。

用法（先 source env_hand.sh）:
  python3 grip.py grab                  # 抓乒乓球（预设 pos）
  python3 grip.py release               # 张开放球（预设 pos）
  python3 grip.py --method 1 --pos 800  # 自定义 pos（0~4096），标定用
  python3 grip.py grab --side right     # 只动右爪

发送一次命令后默认监听 2 秒状态反馈并解码打印（0x12=锁定成功 / 0x14=工件脱落...），
用它判断球有没有真的抓住。
"""

import argparse
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from ros2_plugin_proto.msg import RosMsgWrapper

from aimdk.protocol_pb2 import HandCommandChannel, HandStateChannel

CMD_TOPIC = "/body_drive/hand_joint_command/pb_3Aaimdk_2Eprotocol_2EHandCommandChannel"
STATE_TOPIC = "/body_drive/hand_joint_state/pb_3Aaimdk_2Eprotocol_2EHandStateChannel"

# ── 两个预设（08-11 现场重标定定稿，勿随意改）──────────────
# pos 范围 0~4096（厂家确认，其他为无效值）。实测锚点：100≈闭合到底。
# grab=800：乒乓球夹住；release=2000：开口放球。（08-06 旧值 1650/3000 作废）
# 发球周期：grab → 塞球 → (发球动作) → release。force/vel 是 0~100。
PRESETS = {
    "grab":    dict(method=1, pos=800, force=20, vel=40),
    "release": dict(method=1, pos=2000, force=20, vel=60),
}
# ──────────────────────────────────────────────────────────

STATE_NAMES = {
    0x11: "正在锁定",
    0x12: "锁定成功(已夹住)",
    0x13: "锁定失败",
    0x14: "工件脱落(没夹住/掉了)",
    0x21: "正在释放",
    0x22: "释放成功(已张开)",
    0x23: "释放失败(夹爪堵塞)",
}

QOS = QoSProfile(
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
)


def clamp_percent(value, name):
    v = int(value)
    if not 0 <= v <= 100:
        raise SystemExit(f"{name} 必须在 0~100，收到 {value}")
    return v


def clamp_pos(value):
    v = int(value)
    if not 0 <= v <= 4096:
        raise SystemExit(f"pos 必须在 0~4096（厂家规定，其他为无效值），收到 {value}")
    return v


class GripNode(Node):
    def __init__(self, claw_cfg, side, watch_sec):
        super().__init__("grip_oneshot")
        self.claw_cfg = claw_cfg
        self.side = side
        self.watch_sec = watch_sec
        self.last_state = None

        self.pub = self.create_publisher(RosMsgWrapper, CMD_TOPIC, QOS)
        if watch_sec > 0:
            self.sub = self.create_subscription(
                RosMsgWrapper, STATE_TOPIC, self.on_state, QOS
            )
        self.seq = 0

    def fill_claw(self, single_hand_cmd):
        claw = single_hand_cmd.agi_claw_cmd
        claw.cmd = self.claw_cfg.get("cmd", 0)
        claw.pos = self.claw_cfg["pos"]
        claw.force = self.claw_cfg["force"]
        claw.clamp_method = self.claw_cfg["method"]
        claw.vel = self.claw_cfg["vel"]

    def fill_idle(self, single_hand_cmd):
        # 非指挥侧也必须携带 agi_claw_cmd 占位（全 0 = idle）：
        # 实测缺一侧时整条命令疑似被丢弃，厂家样例也永远双侧齐发
        single_hand_cmd.agi_claw_cmd.SetInParent()

    def send_once(self):
        ch = HandCommandChannel()
        now = time.time()
        ch.header.seq = self.seq
        ch.header.timestamp.seconds = int(now)
        ch.header.timestamp.nanos = int((now % 1) * 1e9)
        ch.header.timestamp.ms_since_epoch = int(now * 1000)
        ch.header.control_source = 1

        if self.side in ("left", "both"):
            self.fill_claw(ch.data.left)
        else:
            self.fill_idle(ch.data.left)
        if self.side in ("right", "both"):
            self.fill_claw(ch.data.right)
        else:
            self.fill_idle(ch.data.right)

        raw = ch.SerializeToString()
        msg = RosMsgWrapper()
        msg.serialization_type = "pb"
        msg.context = ["aimdk.protocol.HandCommandChannel"]
        msg.data = [bytes([b]) for b in raw]
        self.pub.publish(msg)
        self.seq += 1

    def on_state(self, msg):
        if msg.serialization_type != "pb":
            return
        st = HandStateChannel()
        st.ParseFromString(b"".join(msg.data))
        self.last_state = st

    def describe(self, tag, single):
        s = single.agi_claw_state
        if s.state == 0:
            # 08-06 实测：本机固件 state/pos/temp 恒为 0，不上报（已问厂家待回复）
            return f"{tag}: 固件未上报状态（恒 0），请目视确认"
        name = STATE_NAMES.get(s.state, f"未知(0x{s.state:x})")
        return f"{tag}: {name}, pos={s.pos}, {s.temperature}°C"


def run_burst(node, cfg, side, repeat, watch):
    node.claw_cfg = cfg
    node.side = side
    for _ in range(max(1, repeat)):
        node.send_once()
        t = time.time() + 0.1
        while time.time() < t:
            rclpy.spin_once(node, timeout_sec=0.05)
    node.get_logger().info(f"已发送×{max(1, repeat)} side={side} {cfg}")
    if watch > 0:
        end = time.time() + watch
        while time.time() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.last_state is None:
            print("⚠️  没收到状态反馈（elink 在跑吗？）")
        else:
            d = node.last_state.data
            if side in ("left", "both"):
                print(node.describe("左爪", d.left))
            if side in ("right", "both"):
                print(node.describe("右爪", d.right))


def main():
    parser = argparse.ArgumentParser(description="夹爪一次性抓/放命令")
    parser.add_argument("preset", nargs="?", choices=sorted(PRESETS),
                        help="grab=抓球 / release=放球；不给则须用 --method/--pos 自定义")
    # 本机只有左手装爪，默认只指挥 left（右侧自动带全零 idle 占位保证消息结构完整）
    parser.add_argument("--side", choices=["left", "right", "both"], default="left")
    parser.add_argument("--pos", type=int, help="开口位置 0~4096")
    parser.add_argument("--force", type=int, help="夹持力 0~100%%")
    parser.add_argument("--vel", type=int, help="速度 0~100%%")
    parser.add_argument("--method", type=int, choices=[0, 1, 2],
                        help="0=idle 1=夹持 2=释放")
    parser.add_argument("--cmd", type=int, default=0,
                        help="夹爪配置指令字段，默认 0（语义未公开，实验用）")
    parser.add_argument("--watch", type=float, default=2.0,
                        help="发送后监听状态反馈的秒数，0=不监听")
    parser.add_argument("--repeat", type=int, default=5,
                        help="同一命令连发次数（对冲 BEST_EFFORT 丢包），默认 5")
    parser.add_argument("--interactive", "-i", action="store_true",
                        help="交互模式：节点常驻，连续敲命令（标定用，无发现竞态）")
    args = parser.parse_args()

    cfg = dict(PRESETS[args.preset]) if args.preset else dict(method=None, pos=None,
                                                             force=20, vel=40)
    for key in ("pos", "force", "vel", "method"):
        override = getattr(args, key)
        if override is not None:
            cfg[key] = override
    have_cmd = cfg["method"] is not None and cfg["pos"] is not None
    if not have_cmd and not args.interactive:
        raise SystemExit("没选预设时必须给 --method 和 --pos（或用 -i 进交互模式）")
    if have_cmd:
        cfg["pos"] = clamp_pos(cfg["pos"])
        cfg["force"] = clamp_percent(cfg["force"], "force")
        cfg["vel"] = clamp_percent(cfg["vel"], "vel")
    cfg["cmd"] = max(0, int(args.cmd))

    rclpy.init()
    node = GripNode(cfg, args.side, args.watch)
    try:
        # 08-06 修复：固定等 0.6s 不可靠——DDS 握手偶尔更慢，整串命令发进空气，
        # 表现为"同一命令时灵时不灵"。改为等到发布端真正匹配上订阅者再发。
        deadline = time.time() + 5.0
        while node.pub.get_subscription_count() == 0 and time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.pub.get_subscription_count() == 0:
            print("⚠️  5 秒没匹配到订阅者（elink 在跑吗？），命令大概率发不出去")
        else:
            t = time.time() + 0.2   # 匹配后留 0.2s 余量等 DataReader 完全就绪
            while time.time() < t:
                rclpy.spin_once(node, timeout_sec=0.05)

        if args.interactive:
            print("交互模式：grab / release / <pos>(m1 到该值) / m2 <pos> / q 退出")
            while True:
                try:
                    line = input("grip> ").strip().lower()
                except EOFError:
                    break
                if line in ("q", "quit", "exit"):
                    break
                if line in PRESETS:
                    c = dict(PRESETS[line])
                elif line.startswith("m2 ") and line.split()[1].isdigit():
                    c = dict(method=2, pos=int(line.split()[1]), force=20, vel=40)
                elif line.isdigit():
                    c = dict(method=1, pos=int(line), force=20, vel=40)
                else:
                    print("可用：grab / release / 1500 / m2 1500 / q")
                    continue
                if not 0 <= c["pos"] <= 4096:
                    print("pos 必须 0~4096")
                    continue
                c["cmd"] = args.cmd
                run_burst(node, c, args.side, args.repeat, 0)
        else:
            run_burst(node, cfg, args.side, args.repeat, args.watch)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
