#!/usr/bin/env python3
"""Record MuJoCo contact/landing evidence for every Gate3 shot."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path

import rclpy
from mujoco_sim_msgs.msg import Gate3BallState
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from pp_gate3_core import PhysicalEvidenceAccumulator


class Gate3BallEvidence(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("gate3_ball_evidence")
        self._output = Path(args.output)
        self._accumulator = PhysicalEvidenceAccumulator(
            range(1, args.expected_shots + 1),
            min_samples=args.min_samples,
            max_sample_gap_s=args.max_sample_gap_s,
        )
        self.create_subscription(
            Gate3BallState, "/sim/gate3/ball_state", self._on_state, 10
        )
        # The conductor consumes this file before process teardown.  Periodic
        # atomic snapshots make the final parked-shot evidence available
        # without coupling either process to a private shutdown signal.
        self.create_timer(0.5, lambda: self.write_report(log=False))

    @staticmethod
    def _stamp_ns(msg: Gate3BallState) -> int:
        return int(msg.header.stamp.sec) * 1_000_000_000 + int(
            msg.header.stamp.nanosec
        )

    def _on_state(self, msg: Gate3BallState) -> None:
        self._accumulator.ingest(
            stamp_ns=self._stamp_ns(msg),
            shot_id=msg.shot_id,
            active=msg.active,
            position=(msg.position.x, msg.position.y, msg.position.z),
            velocity=(
                msg.linear_velocity.x,
                msg.linear_velocity.y,
                msg.linear_velocity.z,
            ),
            racket_contact_count=msg.racket_contact_count,
            table_contact_count=msg.table_contact_count,
            net_contact_count=msg.net_contact_count,
            racket_normal_force_n=msg.racket_normal_force_n,
        )

    def write_report(self, *, log: bool = True) -> None:
        report = self._accumulator.report()
        self._output.parent.mkdir(parents=True, exist_ok=True)
        tmp = self._output.with_suffix(self._output.suffix + ".tmp")
        tmp.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
        tmp.replace(self._output)
        if log:
            self.get_logger().info(
                "Gate3 physical evidence: measured=%s contacts=%s landings=%s json=%s"
                % (
                    report["physical_contact_measured"],
                    report["physical_contact_pass"],
                    report["landing_pass"],
                    self._output,
                )
            )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--expected-shots", type=int, default=int(os.environ.get("PP_SERVES", "12"))
    )
    parser.add_argument(
        "--output",
        default=os.environ.get(
            "PP_PHYSICAL_EVIDENCE_JSON", "/tmp/pp_physical_ball_report.json"
        ),
    )
    parser.add_argument("--min-samples", type=int, default=20)
    parser.add_argument("--max-sample-gap-s", type=float, default=0.050)
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)
    node = Gate3BallEvidence(args)
    try:
        rclpy.spin(node)
        return 0
    except (KeyboardInterrupt, ExternalShutdownException):
        return 130
    except Exception:
        # Jazzy can surface SIGTERM as an RCLError from a wait-set after the
        # context has already been invalidated instead of raising
        # ExternalShutdownException. Preserve real runtime failures, but keep
        # an expected Gate3 teardown quiet and deterministic.
        if rclpy.ok():
            raise
        return 130
    finally:
        node.write_report(log=rclpy.ok())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
