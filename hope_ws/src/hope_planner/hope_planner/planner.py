"""Top-level HOPE planner pipeline combining Stages 1-3.

Call .update() with each ball position at the motion capture frame rate;
read .racket_command for the latest desired racket state.

See HOPE_7DOF_Racket_Model_based_Planner_Reference_Setup.md, Section 6.
"""

from dataclasses import dataclass
import time
from typing import Optional

import numpy as np

from .ball_state_estimator import BallStateEstimator
from .ball_kalman_estimator import BallKalmanEstimator
from .ball_trajectory_predictor import BallTrajectoryPredictor, StrikeTarget
from .constants import BallPhysics, PlannerConfig, TableParams
from .racket_target_planner import RacketCommand, RacketTargetPlanner


@dataclass
class PlannerAudit:
    """Per-solve diagnostics that never participate in command decisions."""

    reason: str = "not_run"
    stage1_ms: float = 0.0
    stage2_ms: float = 0.0
    stage3_ms: float = 0.0
    solve_total_ms: float = 0.0


class HOPEPlanner:
    """Top-level planner combining Stages 1-3."""

    def __init__(
        self,
        physics: Optional[BallPhysics] = None,
        config: Optional[PlannerConfig] = None,
        table: Optional[TableParams] = None,
    ):
        self.physics = physics or BallPhysics()
        self.config = config or PlannerConfig()
        self.table = table or TableParams()

        self.estimator = (
            BallKalmanEstimator(self.config, self.physics, self.table)
            if self.config.use_kalman
            else BallStateEstimator(
                self.config,
                horizontal_poly_order=self.config.horizontal_poly_order,
            )
        )
        self.predictor = BallTrajectoryPredictor(self.physics, self.config, self.table)
        self.target_planner = RacketTargetPlanner(self.physics, self.config, self.table)

        self._latest_command: Optional[RacketCommand] = None
        self._latest_strike: Optional[StrikeTarget] = None
        self._latest_t: Optional[float] = None
        self._latest_est: Optional[tuple] = None  # (p_est, v_est, t_est) of the last solve
        self._audit = PlannerAudit()

    def update(self, t: float, p_ball: np.ndarray) -> Optional[RacketCommand]:
        """Process a new ball position measurement.

        Parameters
        ----------
        t : float
            Timestamp in seconds (monotonic).
        p_ball : np.ndarray, shape (3,)
            Ball position [x, y, z] in HOPE canonical frame.

        Returns
        -------
        RacketCommand or None
        """
        solve_start_ns = time.perf_counter_ns()
        self._audit = PlannerAudit(reason="stage1_running")
        active_stage = "stage1"
        try:
            stage_start_ns = time.perf_counter_ns()
            self.estimator.push(t, p_ball)

            if not self.estimator.ready:
                self._latest_command = None
                self._latest_strike = None
                self._latest_est = None
                self._audit.reason = "estimator_not_ready"
                return None

            p_est, v_est, t_est = self.estimator.estimate()
            self._audit.stage1_ms = self._elapsed_ms(stage_start_ns)
            self._latest_t = t_est

            # Only predict if the ball is moving toward P1 (v_x < 0).
            if v_est[0] >= 0:
                self._latest_command = None
                self._latest_strike = None
                self._latest_est = None
                self._audit.reason = "not_incoming"
                return None
            self._latest_est = (p_est, v_est, t_est)

            active_stage = "stage2"
            stage_start_ns = time.perf_counter_ns()
            strike = self.predictor.predict(p_est, v_est, t_est)
            self._audit.stage2_ms = self._elapsed_ms(stage_start_ns)
            self._latest_strike = strike

            active_stage = "stage3"
            stage_start_ns = time.perf_counter_ns()
            command = self.target_planner.plan(strike)
            self._audit.stage3_ms = self._elapsed_ms(stage_start_ns)
            self._latest_command = command
            if not strike.valid:
                self._audit.reason = self.predictor.last_reason
            else:
                self._audit.reason = "command_valid" if command.valid else "stage3_invalid"
            return command
        except Exception:
            self._audit.reason = f"{active_stage}_exception"
            raise
        finally:
            # When Stage 1 returns before estimate(), retain the complete Stage
            # 1 duration. This assignment is telemetry only.
            if active_stage == "stage1" and self._audit.stage1_ms == 0.0:
                self._audit.stage1_ms = self._elapsed_ms(solve_start_ns)
            self._audit.solve_total_ms = self._elapsed_ms(solve_start_ns)

    def repredict_at_plane(self, x_hit: float) -> Optional[RacketCommand]:
        """Re-run Stages 2+3 on the LATEST estimator state at a different hit plane.

        Per-side hit planes (2026-07-13, v13 facefix): the caller first solves at
        the forehand plane (the farther one — the incoming ball crosses it first),
        picks the swing side from that intercept, and when the backhand is selected
        re-predicts here at the backhand plane. Mutates ``config.x_hit`` — callers
        that keep a fixed plane must re-assign it before the next solve (node.py
        sets it at the top of every solve). No new estimator sample is pushed, so
        ``time_to_strike`` stays consistent with the re-predicted strike.
        """
        if self._latest_est is None:
            return self._latest_command
        solve_start_ns = time.perf_counter_ns()
        self.config.x_hit = float(x_hit)
        p_est, v_est, t_est = self._latest_est
        stage_start_ns = time.perf_counter_ns()
        strike = self.predictor.predict(p_est, v_est, t_est)
        self._audit.stage2_ms += self._elapsed_ms(stage_start_ns)
        self._latest_strike = strike
        stage_start_ns = time.perf_counter_ns()
        self._latest_command = self.target_planner.plan(strike)
        self._audit.stage3_ms += self._elapsed_ms(stage_start_ns)
        self._audit.solve_total_ms += self._elapsed_ms(solve_start_ns)
        if not strike.valid:
            self._audit.reason = self.predictor.last_reason
        else:
            self._audit.reason = (
                "command_valid" if self._latest_command.valid else "stage3_invalid"
            )
        return self._latest_command

    def replan_latest(self) -> Optional[RacketCommand]:
        """Re-run Stage 3 only, on the latest Stage-2 prediction.

        For callers that mutate aim params (config.target_land / delta_t_flight)
        AFTER seeing where the current ball actually arrives (per-side aim): the
        estimator/predictor state is untouched, so time_to_strike stays consistent.
        No-op (returns the existing command) when no prediction is available.
        """
        if self._latest_strike is None:
            return self._latest_command
        solve_start_ns = time.perf_counter_ns()
        self._latest_command = self.target_planner.plan(self._latest_strike)
        elapsed_ms = self._elapsed_ms(solve_start_ns)
        self._audit.stage3_ms += elapsed_ms
        self._audit.solve_total_ms += elapsed_ms
        self._audit.reason = (
            "command_valid" if self._latest_command.valid else "stage3_invalid"
        )
        return self._latest_command

    @staticmethod
    def _elapsed_ms(start_ns: int) -> float:
        return (time.perf_counter_ns() - start_ns) * 1.0e-6

    @property
    def audit(self) -> PlannerAudit:
        """Latest per-solve timings/reason; read-only command-path telemetry."""
        return self._audit

    @property
    def racket_command(self) -> Optional[RacketCommand]:
        return self._latest_command

    @property
    def strike_target(self) -> Optional[StrikeTarget]:
        """Latest Stage-2 prediction (ball state at the hitting plane).

        Exposed for the strike-spec diagnostics path (node.py), which needs
        the predicted ball state AT the strike, not just the racket command.
        """
        return self._latest_strike

    @property
    def time_to_strike(self) -> Optional[float]:
        """Seconds remaining until the predicted strike (positive, decreasing).

        NOTE: this returns time *remaining* (t_strike - latest sample time),
        not the absolute strike time. The reference doc skeleton returned the
        absolute time; that is corrected here so the value is positive and
        decreases as the ball approaches, per the verification gate.
        """
        if self._latest_command is None or not self._latest_command.valid:
            return None
        if self._latest_strike is None or self._latest_t is None:
            return None
        return self._latest_strike.t_strike - self._latest_t
