# Copyright (c) 2026 Intelligent Racing Inc. (dba Hitch Interactive)
# SPDX-License-Identifier: Apache-2.0
"""ROS 2 planner bridge: ``hope_msgs/RacketCommand`` -> :class:`RacketCommandSource`.

This is the documented planner-to-runner control path. :class:`RosRacketCommandSource`
subscribes to the planner's command topic (default ``/racket/command``), converts each
``hope_msgs/RacketCommand`` message into the runner's :class:`RacketCommand` dataclass,
and hands it to the 50 Hz control loop through the same thread-safe latest-value mailbox
the other sources use. Select it with ``python -m a3_deploy_onnx_ref_pingpong --planner``.

Requirements (only when the bridge is actually constructed — the module itself imports
without ROS):

  * a sourced ROS 2 environment with ``rclpy``;
  * the built ``hope_msgs`` package on the overlay, e.g.::

        cd hope_ws && colcon build --packages-up-to hope_msgs
        source install/setup.bash

``racket_command_from_msg`` is a pure conversion function (duck-typed, no ROS imports)
so the field mapping is unit-testable without a ROS installation.
"""

from __future__ import annotations

import threading

from .racket_command import QueueRacketCommandSource, RacketCommand, RacketCommandSource

DEFAULT_COMMAND_TOPIC = "/racket/command"


def racket_command_from_msg(msg) -> RacketCommand:
    """Convert a ``hope_msgs/RacketCommand`` message into the runner dataclass.

    Duck-typed on the message fields so it works with the real ROS message class or
    any stand-in exposing the same attributes.
    """
    return RacketCommand(
        task_id=int(msg.task_id),
        task_revision=int(msg.task_revision),
        swing_side=int(msg.swing_side),
        position=(float(msg.position.x), float(msg.position.y), float(msg.position.z)),
        velocity=(float(msg.velocity.x), float(msg.velocity.y), float(msg.velocity.z)),
        time_to_strike=float(msg.time_to_strike),
    )


class RosRacketCommandSource(RacketCommandSource):
    """rclpy-backed command source: subscribes the planner topic on a background thread.

    Owns a private rclpy context + single-threaded executor so it composes with (and
    never tears down) any other rclpy usage in the process. ``poll()`` is the runner-side
    contract: it returns the newest converted command (or ``None`` before the first one).
    Call :meth:`close` when done.
    """

    def __init__(
        self,
        topic: str = DEFAULT_COMMAND_TOPIC,
        node_name: str = "a3_deploy_onnx_ref_pingpong",
    ) -> None:
        try:
            import rclpy
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.qos import (
                DurabilityPolicy,
                HistoryPolicy,
                QoSProfile,
                ReliabilityPolicy,
            )
        except ImportError as exc:  # pragma: no cover - environment-dependent
            raise RuntimeError(
                "--planner needs a sourced ROS 2 environment (rclpy not importable). "
                "Source your ROS 2 setup and the hope_ws overlay first."
            ) from exc
        try:
            from hope_msgs.msg import RacketCommand as RacketCommandMsg
        except ImportError as exc:  # pragma: no cover - environment-dependent
            raise RuntimeError(
                "--planner needs the hope_msgs package: build it with "
                "`cd hope_ws && colcon build --packages-up-to hope_msgs` and "
                "`source install/setup.bash`."
            ) from exc

        self._queue = QueueRacketCommandSource()
        self._rclpy = rclpy
        self._context = rclpy.Context()
        rclpy.init(context=self._context, args=None)
        self._node = rclpy.create_node(node_name, context=self._context)
        # Match the planner publisher QoS (reliable / volatile / keep-last).
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._sub = self._node.create_subscription(
            RacketCommandMsg, topic, self._on_msg, qos
        )
        self._executor = SingleThreadedExecutor(context=self._context)
        self._executor.add_node(self._node)
        self._spin_thread = threading.Thread(
            target=self._spin, name="racket-command-sub", daemon=True
        )
        self._closed = False
        self._spin_thread.start()

    def _spin(self) -> None:
        try:
            while not self._closed and self._context.ok():
                self._executor.spin_once(timeout_sec=0.1)
        except Exception:  # executor shut down under us during close()
            if not self._closed:
                raise

    def _on_msg(self, msg) -> None:
        self._queue.submit(racket_command_from_msg(msg))

    # -- RacketCommandSource -------------------------------------------------
    def poll(self) -> RacketCommand | None:
        return self._queue.poll()

    def has_any(self) -> bool:
        return self._queue.has_any()

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        self._executor.shutdown(timeout_sec=1.0)
        self._node.destroy_node()
        try:
            self._rclpy.shutdown(context=self._context)
        except RuntimeError:
            pass  # context already shut down
        self._spin_thread.join(timeout=2.0)
