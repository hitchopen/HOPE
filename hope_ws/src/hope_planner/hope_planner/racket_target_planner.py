"""Stage 3 - Racket target planning.

Given the predicted ball state at the hitting plane (Stage 2), compute the
desired racket velocity and face orientation to return the ball to the
opponent's half center, with a net-clearance check and flight-time fallback.

See HOPE_7DOF_Racket_Model_based_Planner_Reference_Setup.md, Section 5.
"""

from dataclasses import dataclass
from typing import Tuple

import numpy as np

from .ball_trajectory_predictor import StrikeTarget
from .constants import BallPhysics, PlannerConfig, TableParams


@dataclass
class RacketCommand:
    """Output of Stage 3: desired racket state at strike time.

    This is the planner's output to the whole-body controller. The racket's
    actual pose is inferred by the humanoid via forward kinematics - it is
    never measured by the motion capture system.
    """

    p_intercept: np.ndarray   # desired racket center position at interception
    v_racket: np.ndarray      # desired racket velocity vector [vx, vy, vz]
    n_racket: np.ndarray      # desired racket face normal (unit vector)
    t_strike: float           # predicted time of strike
    v_ball_outgoing: np.ndarray  # expected outgoing ball velocity
    target_land: np.ndarray   # intended landing point
    clears_net: bool          # True if return trajectory clears the net
    bypasses_net_posts: bool  # True if ball passes outside net Y extent
    valid: bool               # True if all computations succeeded
    num_bounces: int = 0      # bounces predicted before the strike (from Stage 2)


class RacketTargetPlanner:
    """Compute desired racket velocity and orientation for a valid return.

    Implements HITTER Section III-C: racket-ball interaction model.
    """

    def __init__(self, physics: BallPhysics, config: PlannerConfig, table: TableParams):
        self.physics = physics
        self.config = config
        self.table = table

    def _compute_outgoing_velocity(
        self, p_strike: np.ndarray, p_land: np.ndarray, delta_t: float,
    ) -> np.ndarray:
        """Solve initial outgoing velocity with the same drag+gravity model used by Stage 2."""
        if delta_t <= 0.0:
            raise ValueError("delta_t must be positive")

        v = (p_land - p_strike) / delta_t - 0.5 * self.physics.g * delta_t
        if self.physics.k == 0.0:
            return v

        # Shooting solve: finite-difference Newton on final position after free
        # flight. Integrate the baseline and all three finite-difference
        # perturbations in one Python loop. Each row still uses the exact same
        # 1 ms drag/gravity update; only Python loop overhead is shared.
        for _ in range(12):
            eps = 1e-4 * np.maximum(1.0, np.abs(v))
            v_batch = np.repeat(v[None, :], 4, axis=0)
            v_batch[1:, :] += np.diag(eps)
            p_ends, _ = self._integrate_free_flight_batch(
                p_strike, v_batch, delta_t
            )
            p_end = p_ends[0]
            residual = p_end - p_land
            if np.linalg.norm(residual) < 1e-5:
                break

            jac = (p_ends[1:] - p_end).T / eps[None, :]
            try:
                step = np.linalg.solve(jac, residual)
            except np.linalg.LinAlgError:
                step = np.linalg.lstsq(jac, residual, rcond=None)[0]
            v = v - step
            if not np.all(np.isfinite(v)):
                raise FloatingPointError("outgoing velocity solve diverged")
        return v

    def _flight_acceleration(self, v: np.ndarray) -> np.ndarray:
        """Compute free-flight acceleration: quadratic drag plus gravity."""
        speed = np.linalg.norm(v)
        return -self.physics.k * speed * v + self.physics.g

    def _integrate_free_flight(
        self, p0: np.ndarray, v0: np.ndarray, duration: float,
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Integrate ball free flight with the Stage 2 drag+gravity model."""
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

    def _integrate_free_flight_batch(
        self, p0: np.ndarray, v0: np.ndarray, duration: float,
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Integrate N independent free flights in one shared Python loop."""
        velocities = np.asarray(v0, dtype=float)
        if velocities.ndim != 2 or velocities.shape[1] != 3:
            raise ValueError("v0 must have shape (N, 3)")
        p = np.repeat(np.asarray(p0, dtype=float)[None, :], len(velocities), axis=0)
        v = velocities.copy()
        remaining = max(float(duration), 0.0)
        dt = self.config.dt_integrate
        gravity = self.physics.g[None, :]
        while remaining > 1e-12:
            h = min(dt, remaining)
            speed = np.linalg.norm(v, axis=1, keepdims=True)
            a = -self.physics.k * speed * v + gravity
            p = p + v * h + 0.5 * a * h ** 2
            v = v + a * h
            remaining -= h
        return p, v

    @staticmethod
    def _opponent_facing_normal(candidate: np.ndarray) -> np.ndarray:
        """Return a unit racket normal with positive HOPE-frame X component."""
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
        """Compute desired racket velocity and face normal from impact model.

        Restitution is velocity-dependent (venue fit, F4 KILL of constant e):
        e(u_n) = g1 * exp(g2 * |u_n|) with u_n = v_i_n - v_r_n, the normal
        approach speed in the racket frame. Since u_n depends on the answer,
        solve by fixed point from the constant seed C_r (converges in 2-3
        iterations; g2*u_n is small). Falls back to constant C_r when the
        exponential parameters are absent (older configs).
        """
        delta_v = v_outgoing - v_incoming
        delta_v_norm = np.linalg.norm(delta_v)

        if delta_v_norm < 1e-6:
            n = np.array([1.0, 0.0, 0.0])
            return np.zeros(3), n

        n = self._opponent_facing_normal(delta_v)
        v_o_n = np.dot(v_outgoing, n)
        v_i_n = np.dot(v_incoming, n)

        g1 = getattr(self.config, "e_exp_g1", None)
        g2 = getattr(self.config, "e_exp_g2", None)
        e = C_r
        v_r_n = (v_o_n + e * v_i_n) / (1.0 + e)
        if g1 is not None and g2 is not None:
            for _ in range(3):
                u_n = abs(v_i_n - v_r_n)
                e = float(np.clip(g1 * np.exp(g2 * u_n), 0.2, 0.95))
                v_r_n = (v_o_n + e * v_i_n) / (1.0 + e)

        return v_r_n * n, n

    def _check_net_clearance(
        self, p_strike: np.ndarray, v_outgoing: np.ndarray, margin: float = 0.03,
    ) -> Tuple[bool, bool]:
        """Check net clearance under the drag+gravity free-flight model."""
        x_net = self.table.net_x
        z_net = self.table.net_height

        if v_outgoing[0] <= 0:
            return False, False

        p_net = self._free_flight_position_at_x(p_strike, v_outgoing, x_net)
        if p_net is None:
            return False, False
        y_at_net = p_net[1]
        z_at_net = p_net[2]

        y_net_min = -self.table.width - self.table.net_overhang
        y_net_max = self.table.net_overhang

        bypasses_posts = (y_at_net < y_net_min) or (y_at_net > y_net_max)
        if bypasses_posts:
            return False, True

        return z_at_net > (z_net + margin), False

    def _free_flight_position_at_x(
        self, p0: np.ndarray, v0: np.ndarray, x_target: float,
    ) -> np.ndarray | None:
        """Integrate free flight until x crosses ``x_target`` and return interpolated position."""
        p = p0.astype(float).copy()
        v = v0.astype(float).copy()
        if np.isclose(p[0], x_target, atol=1e-9):
            return p

        direction = np.sign(x_target - p[0])
        if direction == 0.0 or direction * v[0] <= 0.0:
            return None

        dt = self.config.dt_integrate
        max_steps = int(self.config.max_predict_time / dt)
        for _ in range(max_steps):
            a = self._flight_acceleration(v)
            p_next = p + v * dt + 0.5 * a * dt ** 2
            v_next = v + a * dt
            crossed = (p[0] - x_target) * (p_next[0] - x_target) <= 0.0
            if crossed:
                dx = p_next[0] - p[0]
                alpha = 0.0 if abs(dx) < 1e-12 else (x_target - p[0]) / dx
                alpha = np.clip(alpha, 0.0, 1.0)
                return p + alpha * (p_next - p)
            p, v = p_next, v_next
        return None

    def plan(self, strike: StrikeTarget) -> RacketCommand:
        """Compute racket target state for a valid return."""
        if not strike.valid:
            return RacketCommand(
                p_intercept=strike.p_ball, v_racket=np.zeros(3),
                n_racket=np.array([1.0, 0.0, 0.0]), t_strike=strike.t_strike,
                v_ball_outgoing=np.zeros(3), target_land=self.config.target_land.copy(),
                clears_net=False, bypasses_net_posts=False, valid=False,
                num_bounces=strike.num_bounces,
            )

        p_strike = strike.p_ball
        v_incoming = strike.v_ball
        p_land = self.config.target_land.copy()

        v_outgoing = self._compute_outgoing_velocity(
            p_strike, p_land, self.config.delta_t_flight
        )
        v_racket, n_racket = self._compute_racket_velocity(
            v_incoming, v_outgoing, self.config.C_r
        )
        clears, bypasses = self._check_net_clearance(p_strike, v_outgoing)

        # Auto-adjust flight time if net clearance fails
        if not clears:
            for dt_adj in [0.4, 0.6, 0.35, 0.7, 0.3]:
                v_out_adj = self._compute_outgoing_velocity(p_strike, p_land, dt_adj)
                clears_adj, bypasses_adj = self._check_net_clearance(p_strike, v_out_adj)
                if clears_adj:
                    v_outgoing = v_out_adj
                    v_racket, n_racket = self._compute_racket_velocity(
                        v_incoming, v_outgoing, self.config.C_r
                    )
                    clears, bypasses = True, bypasses_adj
                    break

        return RacketCommand(
            p_intercept=p_strike, v_racket=v_racket, n_racket=n_racket,
            t_strike=strike.t_strike, v_ball_outgoing=v_outgoing,
            target_land=p_land, clears_net=clears,
            bypasses_net_posts=bypasses, valid=True,
            num_bounces=strike.num_bounces,
        )
