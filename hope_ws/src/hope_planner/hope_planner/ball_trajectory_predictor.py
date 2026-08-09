"""Stage 2 - Ball trajectory prediction.

Forward-integrate the ball trajectory with explicit Euler at 1 kHz using a
hybrid flight (quadratic drag + gravity) / bounce (diagonal restitution)
model, and return the predicted ball state at the virtual hitting plane.

See HOPE_7DOF_Racket_Model_based_Planner_Reference_Setup.md, Section 4.
"""

from dataclasses import dataclass
import math
from typing import Optional, Tuple

import numpy as np

from .constants import BallPhysics, PlannerConfig, TableParams


@dataclass
class StrikeTarget:
    """Output of Stage 2: predicted ball state at the hitting plane."""

    p_ball: np.ndarray        # predicted ball position at strike [x, y, z]
    v_ball: np.ndarray        # predicted ball velocity at strike [vx, vy, vz]
    t_strike: float           # absolute time of strike
    num_bounces: int          # number of table bounces before strike
    valid: bool               # True if a valid crossing was found


class BallTrajectoryPredictor:
    """Forward-integrate ball trajectory and find the hitting-plane crossing.

    Uses explicit Euler integration with the hybrid flight/bounce model
    from HITTER Section III-B.
    """

    def __init__(self, physics: BallPhysics, config: PlannerConfig, table: TableParams):
        self.physics = physics
        self.config = config
        self.table = table
        # Diagnostics only: the top-level planner copies this into its audit
        # record. It is not consumed by the prediction or command path.
        self.last_reason = "not_run"

    def _is_on_table(self, p: np.ndarray) -> bool:
        """Check if the ball could contact the table surface.

        Bounds are expanded by ball radius to handle edge contacts.
        """
        r = self.physics.radius
        y_hi = self.table.y_max
        return (
            -r <= p[0] <= self.table.length + r
            and y_hi - self.table.width - r <= p[1] <= y_hi + r
        )

    def _prediction_horizon_s(
        self, p0: np.ndarray, v0: np.ndarray
    ) -> float:
        """Return the Stage-2 integration horizon for one incoming estimate.

        The extension is a compute budget, not a validity or release gate. It
        never shortens the retained venue horizon and the normal crossing/dead
        ball rules still decide whether a physical solution exists.
        """
        base = max(0.0, float(self.config.max_predict_time))
        if not self.config.adaptive_predict_horizon:
            return base
        cap = max(base, float(self.config.max_predict_time_cap))
        distance_x = float(p0[0] - self.config.x_hit)
        incoming_speed_x = -float(v0[0])
        if distance_x <= 0.0 or incoming_speed_x <= 1.0e-6:
            return base
        # x/vx establishes that this is a forward-searchable incoming state,
        # but it is only an optimistic lower bound: quadratic drag and a table
        # bounce can push the real crossing past that estimate. Keep the normal
        # base budget and allow the integrator to continue to the configured
        # cap only when no crossing was found during those first base seconds.
        # Valid trajectories inside the base horizon therefore perform the
        # same number of integration steps as before.
        return cap

    def _flight_acceleration(self, v: np.ndarray, omega: Optional[np.ndarray] = None) -> np.ndarray:
        """Flight acceleration: a = -k|v|v + g [+ k_m (omega x v)].

        omega is the ball spin (rad/s); when omega is None or zero the Magnus
        term contributes exactly 0.0 so legacy (spin-blind) behavior is
        bit-identical. k_m is the venue Magnus coefficient
        (configs/ball_physics_venue.yaml flight.k_m).
        """
        speed = np.linalg.norm(v)
        a = -self.physics.k * speed * v + self.physics.g
        if omega is not None:
            a = a + self.config.k_m * np.cross(omega, v)
        return a

    def _apply_bounce(self, v: np.ndarray) -> np.ndarray:
        """Apply table bounce restitution: v+ = diag(C_h, C_h, -C_v) @ v-"""
        C = np.diag([self.physics.C_h, self.physics.C_h, -self.physics.C_v])
        return C @ v

    def _apply_bounce_nakashima(
        self, v: np.ndarray, w: np.ndarray
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Spin-coupled table bounce (Ace/Nakashima impulse model).

        Contact-point tangential velocity (contact at -r*ez from center):
            v_T = [vx - r*wy, vy + r*wx, 0]
        A friction impulse -alpha*m*v_T acts at the contact point, with
        alpha = mu (1 + e_n) |vz| / |v_T| while sliding persists, capped at
        the ROLLING value alpha = 2/5 (hollow-sphere inertia I = (2/3) m r^2)
        at which v_T+ = (1 - 5/2 alpha) v_T reaches exactly zero. The spin
        rows follow from the impulse torque delta_w = -(3/2r) ez x delta_v:
            w+_x = wx - (3 alpha / 2r) v_Ty
            w+_y = wy + (3 alpha / 2r) v_Tx
            w+_z = wz
        e_n is the venue-fit table restitution (BallPhysics.C_v = contact.
        table.e_eff); mu is an UNFITTED Ace prior (constants.mu_table) — the
        venue tangential refit was degenerate (F5 in the fit report).
        """
        r = self.physics.radius
        e_n = self.physics.C_v
        mu = self.config.mu_table

        v_T = np.array([v[0] - r * w[1], v[1] + r * w[0], 0.0])
        v_T_norm = np.linalg.norm(v_T)

        if v_T_norm < 1e-9:
            alpha = 0.0
        else:
            alpha_slide = mu * (1.0 + e_n) * abs(v[2]) / v_T_norm
            nu_s = 1.0 - 2.5 * alpha_slide
            alpha = alpha_slide if nu_s > 0.0 else 0.4  # rolling cap = 2/5

        v_out = np.array([
            v[0] - alpha * v_T[0],
            v[1] - alpha * v_T[1],
            -e_n * v[2],
        ])
        gain = 3.0 * alpha / (2.0 * r)
        w_out = np.array([
            w[0] - gain * v_T[1],
            w[1] + gain * v_T[0],
            w[2],
        ])
        return v_out, w_out

    def integrate_to_table_plane(
        self,
        p0: np.ndarray,
        v0: np.ndarray,
        omega0: Optional[np.ndarray] = None,
    ) -> Optional[Tuple[np.ndarray, float]]:
        """First DOWNWARD table-plane (z = 0) crossing of a post-strike flight.

        Shares _flight_acceleration (drag + gravity + Magnus) and the Euler
        scheme with predict() so the strike-spec planner's landing forward
        model is the same physics as the incoming-ball prediction — no second
        integrator to keep in sync. Spin is constant in flight (venue
        measurement: spin decay consistent with zero, fit report §11.2).

        Returns (landing_xy, flight_time_s) or None if the ball never crosses
        z = 0 from above within config.max_predict_time.
        """
        dt = self.config.dt_integrate
        max_steps = int(self._prediction_horizon_s(p0, v0) / dt)
        p = np.asarray(p0, dtype=float).copy()
        v = np.asarray(v0, dtype=float).copy()
        omega = np.zeros(3) if omega0 is None else np.asarray(omega0, dtype=float)

        t = 0.0
        for _ in range(max_steps):
            a = self._flight_acceleration(v, omega)
            p_new = p + v * dt + 0.5 * a * dt ** 2
            v_new = v + a * dt
            t += dt
            if p[2] > 0.0 and p_new[2] <= 0.0:
                dz = p[2] - p_new[2]
                frac = p[2] / dz if dz > 1e-12 else 0.5
                frac = float(np.clip(frac, 0.0, 1.0))
                p_land = p + frac * (p_new - p)
                return p_land[:2].copy(), (t - dt) + frac * dt
            p, v = p_new, v_new
        return None

    def _predict_diagonal_no_spin(
        self,
        p0: np.ndarray,
        v0: np.ndarray,
        t0: float,
    ) -> StrikeTarget:
        """Scalar fast path for the production diagonal/no-spin predictor.

        This is the same 1 kHz Euler/bounce/crossing arithmetic as ``predict``
        below. Keeping three scalar floats avoids thousands of tiny NumPy array
        allocations on the HDU ARM CPU. The vector path remains loadable by
        passing an explicit omega (including zeros) and serves as the parity
        reference in tests and offline replay.
        """
        dt = self.config.dt_integrate
        half_dt_sq = 0.5 * dt * dt
        max_steps = int(self._prediction_horizon_s(p0, v0) / dt)
        x_hit = self.config.x_hit
        drag = self.physics.k
        gx, gy, gz = (float(value) for value in self.physics.g)
        c_h = self.physics.C_h
        c_v = self.physics.C_v
        radius = self.physics.radius
        table_y_hi = self.table.y_max
        table_y_lo = table_y_hi - self.table.width

        px, py, pz = (float(value) for value in p0)
        vx, vy, vz = (float(value) for value in v0)
        t = float(t0)
        bounces = 0

        for _step in range(max_steps):
            previous_x = px
            speed = math.sqrt(vx * vx + vy * vy + vz * vz)
            ax = -drag * speed * vx + gx
            ay = -drag * speed * vy + gy
            az = -drag * speed * vz + gz

            vx_new = vx + ax * dt
            vy_new = vy + ay * dt
            vz_new = vz + az * dt
            px_new = px + vx * dt + ax * half_dt_sq
            py_new = py + vy * dt + ay * half_dt_sq
            pz_new = pz + vz * dt + az * half_dt_sq
            t += dt
            bounce_this_step = False

            if pz_new < 0.0 and vz_new < 0.0:
                on_table = (
                    -radius <= px_new <= self.table.length + radius
                    and table_y_lo - radius <= py_new <= table_y_hi + radius
                )
                if on_table:
                    dz = pz - pz_new
                    bounce_fraction = pz / dz if dz > 1.0e-9 else 0.5
                    bounce_fraction = min(max(bounce_fraction, 0.0), 1.0)
                    px_bounce = px + bounce_fraction * (px_new - px)
                    py_bounce = py + bounce_fraction * (py_new - py)

                    bounce_dt = bounce_fraction * dt
                    vx_at_bounce = vx + ax * bounce_dt
                    vy_at_bounce = vy + ay * bounce_dt
                    vz_at_bounce = vz + az * bounce_dt
                    vx_post = c_h * vx_at_bounce
                    vy_post = c_h * vy_at_bounce
                    vz_post = -c_v * vz_at_bounce

                    remaining_dt = (1.0 - bounce_fraction) * dt
                    speed_post = math.sqrt(
                        vx_post * vx_post
                        + vy_post * vy_post
                        + vz_post * vz_post
                    )
                    ax_post = -drag * speed_post * vx_post + gx
                    ay_post = -drag * speed_post * vy_post + gy
                    az_post = -drag * speed_post * vz_post + gz
                    half_remaining_sq = 0.5 * remaining_dt * remaining_dt
                    px_new = (
                        px_bounce
                        + vx_post * remaining_dt
                        + ax_post * half_remaining_sq
                    )
                    py_new = (
                        py_bounce
                        + vy_post * remaining_dt
                        + ay_post * half_remaining_sq
                    )
                    pz_new = (
                        vz_post * remaining_dt
                        + az_post * half_remaining_sq
                    )
                    vx_new = vx_post + ax_post * remaining_dt
                    vy_new = vy_post + ay_post * remaining_dt
                    vz_new = vz_post + az_post * remaining_dt
                    bounces += 1
                    bounce_this_step = True
                else:
                    pz_new = max(pz_new, 0.0)

            if previous_x > x_hit and px_new <= x_hit and vx_new < 0.0:
                if bounce_this_step:
                    dx_arc = px_bounce - px_new
                    crossing_fraction = (
                        (px_bounce - x_hit) / dx_arc
                        if abs(dx_arc) > 1.0e-9 else 0.5
                    )
                    crossing_fraction = min(max(crossing_fraction, 0.0), 1.0)
                    py_cross = py_bounce + crossing_fraction * (py_new - py_bounce)
                    pz_cross = crossing_fraction * pz_new
                    vx_cross = vx_post + crossing_fraction * (vx_new - vx_post)
                    vy_cross = vy_post + crossing_fraction * (vy_new - vy_post)
                    vz_cross = vz_post + crossing_fraction * (vz_new - vz_post)
                    t_cross = (
                        t - remaining_dt + crossing_fraction * remaining_dt
                    )
                else:
                    dx_step = px - px_new
                    crossing_fraction = (
                        (px - x_hit) / dx_step
                        if abs(dx_step) > 1.0e-9 else 0.5
                    )
                    crossing_fraction = min(max(crossing_fraction, 0.0), 1.0)
                    py_cross = py + crossing_fraction * (py_new - py)
                    pz_cross = pz + crossing_fraction * (pz_new - pz)
                    vx_cross = vx + crossing_fraction * (vx_new - vx)
                    vy_cross = vy + crossing_fraction * (vy_new - vy)
                    vz_cross = vz + crossing_fraction * (vz_new - vz)
                    t_cross = t - dt + crossing_fraction * dt

                dead_ball = pz_cross < 0.05 and vz_cross < 0.0
                self.last_reason = "dead_ball" if dead_ball else "prediction_valid"
                return StrikeTarget(
                    p_ball=np.array([x_hit, py_cross, pz_cross]),
                    v_ball=np.array([vx_cross, vy_cross, vz_cross]),
                    t_strike=t_cross,
                    num_bounces=bounces,
                    valid=not dead_ball,
                )

            px, py, pz = px_new, py_new, pz_new
            vx, vy, vz = vx_new, vy_new, vz_new

        if p0[0] <= x_hit:
            self.last_reason = "no_hit_plane_crossing"
        elif px > x_hit:
            self.last_reason = "prediction_horizon_exceeded"
        else:
            self.last_reason = "prediction_invalid"
        return StrikeTarget(
            p_ball=np.array([px, py, pz]),
            v_ball=np.array([vx, vy, vz]),
            t_strike=t,
            num_bounces=bounces,
            valid=False,
        )

    def predict(
        self,
        p0: np.ndarray,
        v0: np.ndarray,
        t0: float,
        omega0: Optional[np.ndarray] = None,
    ) -> StrikeTarget:
        """Forward-integrate and find the hitting-plane crossing.

        Parameters
        ----------
        p0 : Current ball position in HOPE frame.
        v0 : Current ball velocity in HOPE frame.
        t0 : Current timestamp (s).
        omega0 : Optional ball spin (rad/s) for the Magnus term and (with
            config.bounce_model == "nakashima") the spin-coupled bounce.
            Default None -> zero spin -> identical to the legacy prediction.

        Returns
        -------
        StrikeTarget with predicted ball state at the virtual hitting plane.
        """
        self.last_reason = "prediction_running"
        if omega0 is None and self.config.bounce_model == "diagonal":
            return self._predict_diagonal_no_spin(p0, v0, t0)
        dt = self.config.dt_integrate
        max_steps = int(self._prediction_horizon_s(p0, v0) / dt)
        x_hit = self.config.x_hit

        p = p0.copy()
        v = v0.copy()
        # Spin is carried as a constant during flight (no spin-decay model)
        # and updated at bounces when the nakashima map is active.
        omega = np.zeros(3) if omega0 is None else np.asarray(omega0, dtype=float).copy()
        use_nakashima = self.config.bounce_model == "nakashima"
        t = t0
        bounces = 0

        # Track the most recent bounce state so a hit-plane crossing that
        # happens in the same step as a bounce interpolates on a continuous arc.
        p_bounce = p.copy()
        v_post = v.copy()
        remaining_dt = dt

        for _step in range(max_steps):
            p_prev_x = p[0]

            # --- Euler integration step ---
            a = self._flight_acceleration(v, omega)
            v_new = v + a * dt
            p_new = p + v * dt + 0.5 * a * dt ** 2
            t += dt
            bounce_this_step = False

            # --- Bounce detection ---
            if p_new[2] < 0.0 and v_new[2] < 0.0:
                if self._is_on_table(p_new):
                    # Sub-step interpolation to find exact bounce time
                    dz = p[2] - p_new[2]
                    frac = p[2] / dz if dz > 1e-9 else 0.5
                    frac = np.clip(frac, 0.0, 1.0)

                    p_bounce = p + frac * (p_new - p)
                    p_bounce[2] = 0.0
                    v_at_bounce = v + a * (frac * dt)

                    if use_nakashima:
                        v_post, omega = self._apply_bounce_nakashima(v_at_bounce, omega)
                    else:
                        v_post = self._apply_bounce(v_at_bounce)

                    # Continue from bounce with second-order correction
                    remaining_dt = (1.0 - frac) * dt
                    a_post = self._flight_acceleration(v_post, omega)
                    p_new = p_bounce + v_post * remaining_dt + 0.5 * a_post * remaining_dt ** 2
                    v_new = v_post + a_post * remaining_dt
                    bounces += 1
                    bounce_this_step = True
                else:
                    p_new[2] = max(p_new[2], 0.0)

            # --- Hitting plane crossing detection ---
            if p_prev_x > x_hit and p_new[0] <= x_hit and v_new[0] < 0:
                if bounce_this_step:
                    # Use post-bounce arc for interpolation
                    dx_arc = p_bounce[0] - p_new[0]
                    if abs(dx_arc) > 1e-9:
                        frac_cross = (p_bounce[0] - x_hit) / dx_arc
                    else:
                        frac_cross = 0.5
                    frac_cross = np.clip(frac_cross, 0.0, 1.0)
                    p_cross = p_bounce + frac_cross * (p_new - p_bounce)
                    v_cross = v_post + frac_cross * (v_new - v_post)
                    t_cross = (t - remaining_dt) + frac_cross * remaining_dt
                else:
                    dx_step = p[0] - p_new[0]
                    if abs(dx_step) > 1e-9:
                        frac_cross = (p[0] - x_hit) / dx_step
                    else:
                        frac_cross = 0.5
                    frac_cross = np.clip(frac_cross, 0.0, 1.0)
                    p_cross = p + frac_cross * (p_new - p)
                    v_cross = v + frac_cross * (v_new - v)
                    t_cross = t - dt + frac_cross * dt

                p_cross[0] = x_hit

                # DEAD-BALL GUARD (2026-07-05): a crossing at table-skim height with the
                # ball still falling means the prediction never modeled a bounce (off-table
                # per _is_on_table, z clamped to 0 while vz kept integrating) — publishing
                # it as valid=True produced garbage strike plans (z=0.00, vz=-7 m/s) that
                # the runner's gate had to reject every time. A dead ball is NOT a plan.
                dead_ball = p_cross[2] < 0.05 and v_cross[2] < 0.0
                self.last_reason = "dead_ball" if dead_ball else "prediction_valid"
                return StrikeTarget(
                    p_ball=p_cross, v_ball=v_cross,
                    t_strike=t_cross, num_bounces=bounces, valid=not dead_ball,
                )

            p = p_new
            v = v_new

        if p0[0] <= x_hit:
            self.last_reason = "no_hit_plane_crossing"
        elif p[0] > x_hit:
            self.last_reason = "prediction_horizon_exceeded"
        else:
            self.last_reason = "prediction_invalid"
        return StrikeTarget(
            p_ball=p, v_ball=v, t_strike=t,
            num_bounces=bounces, valid=False,
        )
