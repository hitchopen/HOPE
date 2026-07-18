"""Racket target planning.

Given the predicted ball state at the hitting plane, compute the desired racket
velocity (and face normal) to return the ball toward a fixed opponent-half
landing target, using the same no-spin drag + gravity flight model as the
trajectory predictor.
"""

from dataclasses import dataclass
from typing import Tuple

import numpy as np

from .ball_trajectory_predictor import StrikeTarget
from .constants import BallPhysics, PlannerConfig, TableParams


@dataclass
class RacketCommand:
    """Desired racket state at strike time.

    The ROS message publishes only ``p_intercept`` (target position),
    ``v_racket`` (target velocity), and the derived time-to-strike; ``n_racket``
    is an orientation hint for downstream inverse kinematics (see
    :func:`quaternion_utils.normal_to_quaternion`) and is not part of the wire
    message.
    """

    p_intercept: np.ndarray   # desired racket centre position at interception
    v_racket: np.ndarray      # desired racket velocity vector [vx, vy, vz]
    n_racket: np.ndarray      # desired racket face normal (unit vector)
    t_strike: float           # predicted absolute time of the strike
    num_bounces: int = 0      # table bounces predicted before the strike


class RacketTargetPlanner:
    """Compute the desired racket velocity and face orientation for a return."""

    def __init__(self, physics: BallPhysics, config: PlannerConfig, table: TableParams):
        self.physics = physics
        self.config = config
        self.table = table

    def _flight_acceleration(self, v: np.ndarray) -> np.ndarray:
        """Free-flight acceleration: quadratic drag + gravity."""
        speed = np.linalg.norm(v)
        return -self.physics.k * speed * v + self.physics.g

    def _integrate_free_flight(
        self, p0: np.ndarray, v0: np.ndarray, duration: float,
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Integrate ball free flight for ``duration`` with the drag + gravity model."""
        p = p0.astype(float).copy()
        v = v0.astype(float).copy()
        remaining = max(float(duration), 0.0)
        dt = self.config.dt_integrate
        while remaining > 1e-12:
            h = min(dt, remaining)
            a = self._flight_acceleration(v)
            p = p + v * h + 0.5 * a * h ** 2
            v = v + a * h
            remaining -= h
        return p, v

    def _compute_outgoing_velocity(
        self, p_strike: np.ndarray, p_land: np.ndarray, delta_t: float,
    ) -> np.ndarray:
        """Solve the outgoing velocity that lands at ``p_land`` after ``delta_t``.

        Seeds with the drag-free analytic solution, then refines with a
        finite-difference Newton solve on the drag + gravity flight.
        """
        if delta_t <= 0.0:
            raise ValueError("delta_t must be positive")

        v = (p_land - p_strike) / delta_t - 0.5 * self.physics.g * delta_t
        if self.physics.k == 0.0:
            return v

        for _ in range(12):
            p_end, _ = self._integrate_free_flight(p_strike, v, delta_t)
            residual = p_end - p_land
            if np.linalg.norm(residual) < 1e-5:
                break

            jac = np.zeros((3, 3))
            for axis in range(3):
                eps = 1e-4 * max(1.0, abs(v[axis]))
                v_eps = v.copy()
                v_eps[axis] += eps
                p_eps, _ = self._integrate_free_flight(p_strike, v_eps, delta_t)
                jac[:, axis] = (p_eps - p_end) / eps
            try:
                step = np.linalg.solve(jac, residual)
            except np.linalg.LinAlgError:
                step = np.linalg.lstsq(jac, residual, rcond=None)[0]
            v = v - step
            if not np.all(np.isfinite(v)):
                raise FloatingPointError("outgoing velocity solve diverged")
        return v

    @staticmethod
    def _opponent_facing_normal(candidate: np.ndarray) -> np.ndarray:
        """Return a unit racket normal with a positive forward (+x) component."""
        n = np.asarray(candidate, dtype=float)
        norm = np.linalg.norm(n)
        if norm < 1e-9 or not np.isfinite(norm):
            return np.array([1.0, 0.0, 0.0])

        n = n / norm
        if n[0] < 0.0:
            n = -n
        if n[0] <= 1e-6:
            n = n + np.array([1.0, 0.0, 0.0])
            n = n / np.linalg.norm(n)
        return n

    def _compute_racket_velocity(
        self, v_incoming: np.ndarray, v_outgoing: np.ndarray, C_r: float,
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Desired racket velocity and face normal from the impact model.

        Along the face normal n, restitution gives
        v_out_n - v_r_n = -C_r (v_in_n - v_r_n), so the required racket normal
        speed is v_r_n = (v_out_n + C_r v_in_n) / (1 + C_r). The racket velocity
        is taken along n (no tangential drive in this simplified model).
        """
        delta_v = v_outgoing - v_incoming
        if np.linalg.norm(delta_v) < 1e-6:
            return np.zeros(3), np.array([1.0, 0.0, 0.0])

        n = self._opponent_facing_normal(delta_v)
        v_o_n = np.dot(v_outgoing, n)
        v_i_n = np.dot(v_incoming, n)
        v_r_n = (v_o_n + C_r * v_i_n) / (1.0 + C_r)
        return v_r_n * n, n

    def plan(self, strike: StrikeTarget) -> RacketCommand:
        """Compute the racket target for a valid predicted strike.

        Callers pass only valid strikes (see :class:`HOPEPlanner`).
        """
        p_strike = strike.p_ball
        v_incoming = strike.v_ball
        p_land = self.config.target_land.copy()

        v_outgoing = self._compute_outgoing_velocity(
            p_strike, p_land, self.config.delta_t_flight
        )
        v_racket, n_racket = self._compute_racket_velocity(
            v_incoming, v_outgoing, self.config.C_r
        )
        return RacketCommand(
            p_intercept=p_strike, v_racket=v_racket, n_racket=n_racket,
            t_strike=strike.t_strike, num_bounces=strike.num_bounces,
        )
