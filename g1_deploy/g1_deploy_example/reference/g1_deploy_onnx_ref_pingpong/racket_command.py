# Copyright (c) 2026 Intelligent Racing Inc. (dba Hitch Interactive)
# SPDX-License-Identifier: Apache-2.0
"""RacketCommand: the strike goal the planner streams to the runner.

Mirrors the public ``RacketCommand.msg`` (one field group, nothing else):

    uint64 task_id
    uint32 task_revision
    int8   swing_side        # FOREHAND = +1, BACKHAND = -1
    Point  position          # target racket position, world frame, m
    Vector3 velocity         # target racket velocity, world frame, m/s
    float64 time_to_strike   # seconds

Each incoming ball gets a new ``task_id``. Pre-strike trajectory refinements
arrive as an increasing ``task_revision`` under the same ``task_id``. ``swing_side``
is chosen once per ``task_id`` and is locked for that task.

A ``RacketCommandSource`` is the seam between the runner and whatever produces the
command. In a real deployment that is a ROS2 subscriber writing into
``QueueRacketCommandSource``. ``ExampleCommandFeed`` is a self-contained stand-in
so the runner is runnable against the sim WITHOUT a live planner; it is a
demonstration feed only, NOT part of the deploy contract and NOT a scripted swing
(the swing trajectory is always produced by the learned policy).
"""

from __future__ import annotations

import threading
from abc import ABC, abstractmethod
from dataclasses import dataclass, field

import numpy as np

FOREHAND: int = 1
BACKHAND: int = -1


@dataclass
class RacketCommand:
    task_id: int
    task_revision: int
    swing_side: int                       # +1 forehand / -1 backhand
    position: np.ndarray                   # (3,) world m
    velocity: np.ndarray                   # (3,) world m/s
    time_to_strike: float                  # s

    def __post_init__(self) -> None:
        self.position = np.asarray(self.position, dtype=np.float64).reshape(3)
        self.velocity = np.asarray(self.velocity, dtype=np.float64).reshape(3)
        self.swing_side = FOREHAND if self.swing_side >= 0 else BACKHAND


class RacketCommandSource(ABC):
    """Latest-command mailbox. ``poll`` returns the newest command or ``None``."""

    @abstractmethod
    def poll(self) -> RacketCommand | None:
        ...


class QueueRacketCommandSource(RacketCommandSource):
    """Thread-safe latest-value mailbox.

    Integration seam for a live planner: a ROS2 (or other) subscriber thread calls
    ``submit(...)`` on each ``RacketCommand.msg``; the 50 Hz runner thread calls
    ``poll()``. Only the newest command is retained.
    """

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._latest: RacketCommand | None = None
        self._seen = False

    def submit(self, cmd: RacketCommand) -> None:
        with self._lock:
            self._latest = cmd
            self._seen = True

    def poll(self) -> RacketCommand | None:
        with self._lock:
            return self._latest

    def has_any(self) -> bool:
        with self._lock:
            return self._seen


@dataclass
class ExampleCommandFeed(RacketCommandSource):
    """Planner-less demonstration feed for the reference sim.

    Emits one new ``task_id`` every ``period_s`` seconds, alternating forehand and
    backhand so all four adjacent side transitions are exercised over a session.
    Within each ball it streams increasing ``task_revision`` with a decreasing
    ``time_to_strike`` across the approach window, imitating how a real planner
    refines the intercept before contact. The targets are fixed example reach
    points in front of the robot; a real deployment replaces this entirely with the
    planner's ``RacketCommand`` stream.
    """

    dt: float = 0.02
    period_s: float = 4.0
    lead_time_s: float = 1.2           # time_to_strike when a ball first appears
    forehand_pos: tuple = (0.55, -0.35, 0.85)
    backhand_pos: tuple = (0.55, 0.25, 1.00)
    forehand_vel: tuple = (1.5, 1.3, 0.6)
    backhand_vel: tuple = (2.0, -0.7, 0.4)
    _t: float = field(default=0.0, init=False)
    _task_id: int = field(default=0, init=False)
    _revision: int = field(default=0, init=False)
    _issued_for_cycle: int = field(default=-1, init=False)
    _side: int = field(default=FOREHAND, init=False)
    _pos: np.ndarray = field(default=None, init=False)
    _vel: np.ndarray = field(default=None, init=False)
    _latest: RacketCommand | None = field(default=None, init=False)

    def poll(self) -> RacketCommand | None:
        cycle = int(self._t // self.period_s)
        elapsed = self._t - cycle * self.period_s     # time since this ball appeared

        if cycle != self._issued_for_cycle:
            # A fresh ball: new task_id, reset revision, choose side/target.
            self._issued_for_cycle = cycle
            self._task_id += 1
            self._revision = 0
            self._side = FOREHAND if (self._task_id % 2 == 1) else BACKHAND
            if self._side == FOREHAND:
                self._pos = np.array(self.forehand_pos, dtype=np.float64)
                self._vel = np.array(self.forehand_vel, dtype=np.float64)
            else:
                self._pos = np.array(self.backhand_pos, dtype=np.float64)
                self._vel = np.array(self.backhand_vel, dtype=np.float64)
        else:
            self._revision += 1

        tts = max(self.lead_time_s - elapsed, 0.0)     # counts down to contact
        self._latest = RacketCommand(
            task_id=self._task_id,
            task_revision=self._revision,
            swing_side=self._side,
            position=self._pos,
            velocity=self._vel,
            time_to_strike=tts,
        )
        self._t += self.dt
        return self._latest
