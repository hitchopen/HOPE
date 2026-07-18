"""Top-level HOPE planner pipeline.

Call :meth:`HOPEPlanner.update` with each ball position at the motion-capture
sample rate; it estimates the ball state, predicts the no-spin trajectory to
the fixed hitting plane, and returns the desired racket command (or None when
there is no usable strike yet).
"""

from typing import Optional

import numpy as np

from .ball_state_estimator import BallStateEstimator
from .ball_trajectory_predictor import BallTrajectoryPredictor, StrikeTarget
from .constants import BallPhysics, PlannerConfig, TableParams
from .racket_target_planner import RacketCommand, RacketTargetPlanner


class HOPEPlanner:
    """Ball estimation -> trajectory prediction -> racket target planning."""

    def __init__(
        self,
        physics: Optional[BallPhysics] = None,
        config: Optional[PlannerConfig] = None,
        table: Optional[TableParams] = None,
    ):
        self.physics = physics or BallPhysics()
        self.config = config or PlannerConfig()
        self.table = table or TableParams()

        self.estimator = BallStateEstimator(self.config)
        self.predictor = BallTrajectoryPredictor(self.physics, self.config, self.table)
        self.target_planner = RacketTargetPlanner(self.physics, self.config, self.table)

        self._latest_command: Optional[RacketCommand] = None
        self._latest_strike: Optional[StrikeTarget] = None
        self._latest_t: Optional[float] = None
        self._incoming: Optional[bool] = None

    def update(self, t: float, p_ball: np.ndarray) -> Optional[RacketCommand]:
        """Process a new ball position measurement.

        Returns a :class:`RacketCommand` when the ball is incoming and predicted
        to cross the hitting plane, otherwise None.
        """
        self.estimator.push(t, p_ball)

        if not self.estimator.ready:
            self._latest_command = None
            self._latest_strike = None
            self._incoming = None
            return None

        p_est, v_est, t_est = self.estimator.estimate()
        self._latest_t = t_est
        self._incoming = bool(v_est[0] < 0.0)

        # Only plan for a ball moving toward the robot (vx < 0).
        if v_est[0] >= 0:
            self._latest_command = None
            self._latest_strike = None
            return None

        strike = self.predictor.predict(p_est, v_est, t_est)
        if not strike.valid:
            self._latest_command = None
            self._latest_strike = None
            return None

        self._latest_strike = strike
        self._latest_command = self.target_planner.plan(strike)
        return self._latest_command

    @property
    def racket_command(self) -> Optional[RacketCommand]:
        return self._latest_command

    @property
    def ball_incoming(self) -> Optional[bool]:
        """True/False once a velocity is available (vx < 0 = toward the robot); else None."""
        return self._incoming

    @property
    def strike_target(self) -> Optional[StrikeTarget]:
        """Latest predicted ball state at the hitting plane (or None)."""
        return self._latest_strike

    @property
    def time_to_strike(self) -> Optional[float]:
        """Seconds remaining until the predicted strike (positive, decreasing)."""
        if self._latest_strike is None or self._latest_t is None:
            return None
        return self._latest_strike.t_strike - self._latest_t
