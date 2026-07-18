# Copyright (c) 2026 Intelligent Racing Inc. (dba Hitch Interactive)
# SPDX-License-Identifier: Apache-2.0
"""CLI entrypoint: run the reference runner against the MuJoCo sim.

    python -m a3_deploy_onnx_ref_pingpong --view --realtime

By default it loads the shipped runtime config, drives the in-process MuJoCo
bridge, and feeds an example (planner-less) command stream so the policy visibly
swings. Point ``--onnx`` at your exported ``hope_pingpong.onnx``.

Command feeds (mutually exclusive):
  * default      -- the built-in ExampleCommandFeed (planner-less demo);
  * ``--planner``-- subscribe the live planner's ``hope_msgs/RacketCommand`` on
                    ``--command-topic`` (needs a sourced ROS 2 env + built
                    hope_msgs; see ros_command_source.py). This is the documented
                    full planner -> runner control path;
  * ``--idle``   -- no commands at all (robot just holds its stand).
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from .config import RuntimeConfig
from .racket_command import ExampleCommandFeed, QueueRacketCommandSource
from .ros_command_source import DEFAULT_COMMAND_TOPIC
from .runner import PingPongReferenceRunner

_DEFAULT_CONFIG = Path(__file__).resolve().parents[2] / "config" / "hope_pingpong_runtime.yaml"


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="a3_deploy_onnx_ref_pingpong", description=__doc__)
    p.add_argument("--config", type=Path, default=_DEFAULT_CONFIG,
                   help="runtime YAML (default: the shipped config)")
    p.add_argument("--backend", choices=["mujoco", "aimrt"], default="mujoco",
                   help="mujoco = in-process (default); aimrt = live-sim seam")
    p.add_argument("--onnx", type=Path, default=None, help="override the ONNX policy path")
    p.add_argument("--model-xml", type=Path, default=None,
                   help="override the a3_pingpong MJCF path (mujoco backend)")
    p.add_argument("--view", action="store_true", help="launch the MuJoCo passive viewer")
    p.add_argument("--realtime", action="store_true", help="pace the loop at wall-clock 50 Hz")
    p.add_argument("--duration", type=float, default=None, help="run for N seconds")
    p.add_argument("--max-ticks", type=int, default=None, help="run for N control ticks")
    feed = p.add_mutually_exclusive_group()
    feed.add_argument("--planner", action="store_true",
                      help="consume the live planner's hope_msgs/RacketCommand via ROS 2 "
                           "(the full planner -> runner path; needs rclpy + built hope_msgs)")
    feed.add_argument("--idle", action="store_true",
                      help="no command feed (robot just holds a stand)")
    p.add_argument("--command-topic", default=DEFAULT_COMMAND_TOPIC,
                   help=f"planner command topic for --planner (default: {DEFAULT_COMMAND_TOPIC})")
    return p


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)

    cfg = RuntimeConfig.load(args.config)
    if args.onnx is not None:
        cfg.onnx_path = args.onnx.resolve()
    if args.model_xml is not None:
        cfg.model_xml_path = args.model_xml.resolve()

    if not Path(cfg.onnx_path).is_file():
        print(
            f"[ref] ONNX policy not found: {cfg.onnx_path}\n"
            f"      Export one with the training package "
            f"(hope_pingpong.onnx) and pass --onnx, or set policy.onnx_path.",
            file=sys.stderr,
        )
        return 2

    # Sim bridge.
    if args.backend == "mujoco":
        from .sim_bridge import MujocoDirectBridge

        bridge = MujocoDirectBridge(
            cfg.model_xml_path, control_dt=cfg.control_dt, launch_viewer=args.view
        )
    else:
        from .sim_bridge import AimrtSimBridge

        bridge = AimrtSimBridge()  # documented seam: raises with wiring guidance

    # Command source.
    if args.planner:
        from .ros_command_source import RosRacketCommandSource

        source = RosRacketCommandSource(topic=args.command_topic)
        print(f"[ref] consuming planner commands on {args.command_topic}", file=sys.stderr)
    elif args.idle:
        source = QueueRacketCommandSource()
    else:
        source = ExampleCommandFeed(dt=cfg.control_dt)

    max_ticks = args.max_ticks
    if args.duration is not None:
        max_ticks = int(round(args.duration / cfg.control_dt))

    runner = PingPongReferenceRunner(cfg, bridge, source)
    try:
        runner.run(max_ticks=max_ticks, realtime=args.realtime)
    finally:
        close = getattr(source, "close", None)
        if close is not None:
            close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
