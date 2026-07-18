"""Ball trajectory prediction (no spin).

Forward-integrates the ball trajectory with explicit Euler using a simple
flight model (quadratic drag + gravity) and a diagonal table-bounce model, and
returns the predicted ball state at the fixed virtual hitting plane.

Table contact follows the shared ``configs/ball_physics.yaml`` convention: the
ball CENTROID contacts the surface at z = ball radius (0.02 m for a 40 mm ball),
and the bounce event is interpolated to that plane within the crossing step.
"""

from dataclasses import dataclass

import numpy as np

from .constants import BallPhysics, PlannerConfig, TableParams


@dataclass
class StrikeTarget:
    """Predicted ball state at the hitting plane."""

    p_ball: np.ndarray        # predicted ball position at strike [x, y, z]
    v_ball: np.ndarray        # predicted ball velocity at strike [vx, vy, vz]
    t_strike: float           # absolute time of the strike
    num_bounces: int          # number of table bounces before the strike
    valid: bool               # True if a usable plane crossing was found


class BallTrajectoryPredictor:
    """Forward-integrate the ball trajectory and find the hitting-plane crossing."""

    def __init__(self, physics: BallPhysics, config: PlannerConfig, table: TableParams):
        self.physics = physics
        self.config = config
        self.table = table

    def _is_on_table(self, p: np.ndarray) -> bool:
        """True if the ball could contact the table surface (bounds + ball radius)."""
        r = self.physics.radius
        y_hi = self.table.y_max
        return (
            -r <= p[0] <= self.table.length + r
            and y_hi - self.table.width - r <= p[1] <= y_hi + r
        )

    def _flight_acceleration(self, v: np.ndarray) -> np.ndarray:
        """Flight acceleration: a = -k |v| v + g."""
        speed = np.linalg.norm(v)
        return -self.physics.k * speed * v + self.physics.g

    def _apply_bounce(self, v: np.ndarray) -> np.ndarray:
        """Diagonal table bounce: v+ = diag(C_h, C_h, -C_v) @ v-."""
        C = np.diag([self.physics.C_h, self.physics.C_h, -self.physics.C_v])
        return C @ v

    def predict(self, p0: np.ndarray, v0: np.ndarray, t0: float) -> StrikeTarget:
        """Forward-integrate from (p0, v0, t0) and find the hitting-plane crossing.

        Returns a :class:`StrikeTarget`. ``valid`` is False if the ball never
        crosses ``config.x_hit`` while moving toward the robot within the
        prediction horizon, or if the only crossing is a dead ball skimming the
        table on the way down.
        """
        dt = self.config.dt_integrate
        max_steps = int(self.config.max_predict_time / dt)
        x_hit = self.config.x_hit
        # Contact plane for the ball CENTROID: the ball touches the table when its
        # centre reaches z = ball radius (configs/ball_physics.yaml convention),
        # not when the centre reaches the table surface z = 0.
        contact_z = self.physics.radius

        p = p0.copy()
        v = v0.copy()
        t = t0
        bounces = 0

        # Track the most recent bounce so a plane crossing that happens in the
        # same step as a bounce interpolates along the post-bounce arc.
        p_bounce = p.copy()
        v_post = v.copy()
        remaining_dt = dt

        for _step in range(max_steps):
            p_prev_x = p[0]

            a = self._flight_acceleration(v)
            v_new = v + a * dt
            p_new = p + v * dt + 0.5 * a * dt ** 2
            t += dt
            bounce_this_step = False

            # --- Bounce detection (centroid contact at z = ball radius, interpolated) ---
            if p_new[2] < contact_z and v_new[2] < 0.0:
                if self._is_on_table(p_new):
                    dz = p[2] - p_new[2]
                    frac = (p[2] - contact_z) / dz if dz > 1e-9 else 0.5
                    frac = np.clip(frac, 0.0, 1.0)

                    p_bounce = p + frac * (p_new - p)
                    p_bounce[2] = contact_z
                    v_at_bounce = v + a * (frac * dt)
                    v_post = self._apply_bounce(v_at_bounce)

                    remaining_dt = (1.0 - frac) * dt
                    a_post = self._flight_acceleration(v_post)
                    p_new = p_bounce + v_post * remaining_dt + 0.5 * a_post * remaining_dt ** 2
                    v_new = v_post + a_post * remaining_dt
                    bounces += 1
                    bounce_this_step = True
                else:
                    p_new[2] = max(p_new[2], contact_z)

            # --- Hitting-plane crossing detection ---
            if p_prev_x > x_hit and p_new[0] <= x_hit and v_new[0] < 0:
                if bounce_this_step:
                    dx_arc = p_bounce[0] - p_new[0]
                    frac_cross = (p_bounce[0] - x_hit) / dx_arc if abs(dx_arc) > 1e-9 else 0.5
                    frac_cross = np.clip(frac_cross, 0.0, 1.0)
                    p_cross = p_bounce + frac_cross * (p_new - p_bounce)
                    v_cross = v_post + frac_cross * (v_new - v_post)
                    t_cross = (t - remaining_dt) + frac_cross * remaining_dt
                else:
                    dx_step = p[0] - p_new[0]
                    frac_cross = (p[0] - x_hit) / dx_step if abs(dx_step) > 1e-9 else 0.5
                    frac_cross = np.clip(frac_cross, 0.0, 1.0)
                    p_cross = p + frac_cross * (p_new - p)
                    v_cross = v + frac_cross * (v_new - v)
                    t_cross = t - dt + frac_cross * dt

                p_cross[0] = x_hit

                # A crossing at table-skim height with the ball still falling
                # means no bounce was modelled (off-table, centroid clamped at the
                # contact height z = ball radius): the ball is effectively dead, so
                # it is not a usable strike. The margin keeps the threshold strictly
                # above the clamp height.
                dead_ball = p_cross[2] < contact_z + 0.03 and v_cross[2] < 0.0
                return StrikeTarget(
                    p_ball=p_cross, v_ball=v_cross,
                    t_strike=t_cross, num_bounces=bounces, valid=not dead_ball,
                )

            p = p_new
            v = v_new

        return StrikeTarget(
            p_ball=p, v_ball=v, t_strike=t, num_bounces=bounces, valid=False,
        )
