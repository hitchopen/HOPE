# Copyright (c) 2026 Intelligent Racing Inc. (dba Hitch Interactive)
# SPDX-License-Identifier: Apache-2.0
"""ROS 2 planner bridge: ``/racket/command_flat`` -> :class:`RacketCommandSource`.

This is the documented planner-to-runner control path. The live planner publishes
its racket command as a ``std_msgs/Float64MultiArray`` on ``/racket/command_flat``
(reliable QoS). Using the core ``std_msgs`` type instead of a custom message means
this bridge needs NO ``hope_msgs`` build and no rosidl typesupport overlay — a
sourced ROS 2 environment with ``rclpy`` is the only requirement (and only when the
bridge is actually constructed; the module itself imports without ROS).

Flat wire layouts (element [0] is a schema tag; both schemas share the same head):

    schema 1 (>= 11 doubles, legacy):
      [0]=schema(1)  [1]=valid(0/1)  [2]=swing_sign(+1 forehand / -1 backhand)
      [3..5]=pos_w(x,y,z)  [6..8]=vel_w(x,y,z)
      [9]=time_to_strike(s)  [10]=strike_time(s, informational)
      trailing elements (e.g. [11]=frame_code) are ignored here.

    schema 2 (19 doubles, revisioned):
      same head as schema 1, plus
      [11]=frame_code  [12]=producer_sec  [13]=producer_nsec  [14]=command_seq
      [15]=flight_id  [16]=revision_id  [17]=estimator_sample_count
      [18]=estimator_span_s
      Only [15]/[16] are consumed (ball identity / pre-strike refinement); the
      other extras are transport/audit fields this reference deliberately ignores.

``parse_flat_racket_command`` is the pure conversion function (list of floats ->
:class:`RacketCommand` or ``None``), unit-testable without a ROS installation.
Packets with ``valid == 0``, an unknown schema tag, a short array, or non-finite
required fields return ``None`` (skipped — the mailbox keeps the previous command).

Ball identity: schema 2 carries it explicitly (``flight_id``/``revision_id`` map
onto the runner's ``task_id``/``task_revision``). Schema 1 carries none, so
:class:`RosRacketCommandSource` synthesizes it with the same rule the producers
use: a valid command more than ``SCHEMA1_NEW_FLIGHT_GAP_S`` after the previous
valid one opens a new task, otherwise it is a revision of the active task.
"""

from __future__ import annotations

import math
import threading
import time
from typing import Sequence

from .racket_command import QueueRacketCommandSource, RacketCommand, RacketCommandSource

DEFAULT_COMMAND_TOPIC = "/racket/command_flat"

# Minimum silence between valid schema-1 commands that opens a new ball (task_id).
# Mirrors the 250 ms flight rule of both shipped planner publishers.
SCHEMA1_NEW_FLIGHT_GAP_S: float = 0.25

_SCHEMA1_MIN_LEN = 11
_SCHEMA2_LEN = 19


def _finite(values: Sequence[float]) -> bool:
    return all(math.isfinite(float(v)) for v in values)


def parse_flat_racket_command(values: Sequence[float]) -> RacketCommand | None:
    """Decode one ``/racket/command_flat`` array into a :class:`RacketCommand`.

    Pure function (no ROS imports, no state): accepts schema 1 (>= 11 doubles) or
    schema 2 (19 doubles), ignores the extra transport/audit fields, and returns
    ``None`` for anything that must be skipped (``valid == 0``, short array,
    unknown schema tag, non-finite required fields).

    Schema-1 arrays carry no ball identity, so the returned command has
    ``task_id == 0`` / ``task_revision == 0``; the stateful subscriber assigns the
    synthesized identity (see :class:`RosRacketCommandSource`).
    """
    a = [float(v) for v in values]
    if len(a) < 2 or not _finite(a[:2]):
        return None
    schema = a[0]
    if schema == 1.0:
        if len(a) < _SCHEMA1_MIN_LEN:
            return None
        head = a[1:_SCHEMA1_MIN_LEN]
        task_id = 0
        task_revision = 0
    elif schema == 2.0:
        if len(a) < _SCHEMA2_LEN:
            return None
        head = a[1:_SCHEMA1_MIN_LEN]
        if not _finite(a[_SCHEMA1_MIN_LEN:_SCHEMA2_LEN]):
            return None
        task_id = int(a[15])        # flight_id
        task_revision = int(a[16])  # revision_id
    else:
        return None

    if not _finite(head):
        return None
    if head[0] != 1.0:  # valid flag: 0 (or anything else) -> skip
        return None

    return RacketCommand(
        task_id=task_id,
        task_revision=task_revision,
        swing_sign=1 if head[1] >= 0.0 else -1,
        position=(head[2], head[3], head[4]),
        velocity=(head[5], head[6], head[7]),
        time_to_strike=head[8],
    )


class RosRacketCommandSource(RacketCommandSource):
    """rclpy-backed command source: subscribes the flat planner topic on a background thread.

    Owns a private rclpy context + single-threaded executor so it composes with (and
    never tears down) any other rclpy usage in the process. ``poll()`` is the runner-side
    contract: it returns the newest decoded command (or ``None`` before the first one).
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
            from std_msgs.msg import Float64MultiArray
        except ImportError as exc:  # pragma: no cover - environment-dependent
            raise RuntimeError(
                "--planner needs a sourced ROS 2 environment (rclpy + std_msgs not "
                "importable). No hope_msgs build is required — the planner command "
                "arrives as a std_msgs/Float64MultiArray on the flat topic."
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
            Float64MultiArray, topic, self._on_msg, qos
        )
        # Schema-1 identity synthesis state (schema 2 carries its own identity).
        self._s1_task_id = 0
        self._s1_revision = 0
        self._s1_last_valid_mono: float | None = None
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
        cmd = parse_flat_racket_command(msg.data)
        if cmd is None:
            return
        if cmd.task_id == 0:
            # Schema-1 stream: assign a synthesized ball identity (new task after a
            # SCHEMA1_NEW_FLIGHT_GAP_S silence, else a revision of the active one).
            now = time.monotonic()
            if (
                self._s1_last_valid_mono is None
                or now - self._s1_last_valid_mono > SCHEMA1_NEW_FLIGHT_GAP_S
            ):
                self._s1_task_id += 1
                self._s1_revision = 0
            else:
                self._s1_revision += 1
            self._s1_last_valid_mono = now
            cmd.task_id = self._s1_task_id
            cmd.task_revision = self._s1_revision
        self._queue.submit(cmd)

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
