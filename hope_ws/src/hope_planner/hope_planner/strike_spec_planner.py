"""Strike-specification inverse planner.

At play time the robot KNOWS its own racket face angle and normal/tangential
velocity via forward kinematics — those are the CONTROL variables (franco,
2026-07-04). This module therefore inverts the full contact + flight chain:
given the predicted ball state at the strike and a desired landing point, it
solves for the racket FACE NORMAL and the racket velocity DECOMPOSED along
that normal (v_n) and in the face plane (v_t), and reports the landing
sensitivities of each control variable — the control-precision budget the
whole-body controller must meet.

Forward model (no physics duplicated, both halves are the shared venue fit):
  ball_contact.predict_paddle_contact  (spin-impulse paddle contact, e(u_n))
  -> BallTrajectoryPredictor.integrate_to_table_plane  (drag + gravity +
     Magnus flight using omega_plus, Euler @ config.dt_integrate).

Solver: damped Gauss-Newton / Levenberg-Marquardt (numpy only) over
(tilt_pitch, tilt_yaw, v_n, v_t_x, v_t_y), seeded by the mirror law
(racket_target_planner-style normal along v_out - v_in) plus the restitution
speed heuristic. Tilts are expressed in DEGREES relative to that mirror-law
seed normal so a spec reads as "open the face 3 deg more than the naive
mirror" — human-readable and zero-centered for the optimizer.
"""

from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np

from .ball_contact import predict_paddle_contact
from .ball_trajectory_predictor import BallTrajectoryPredictor
from .constants import BallPhysics, PlannerConfig, TableParams

_Z_HAT = np.array([0.0, 0.0, 1.0])


@dataclass
class StrikeSpec:
    """The strike as control variables + landing sensitivities.

    All vectors are HOPE world frame. Tilts are relative to the mirror-law
    initial normal (see module docstring); the face-plane basis for v_t_vec
    is (b1, b2) = (horizontal-in-face, face-up), returned in face_basis.
    """

    # --- control variables (what forward kinematics can verify at contact) ---
    n: np.ndarray             # racket face normal, unit, world frame
    tilt_pitch_deg: float     # elevation offset vs mirror-law normal (deg, + = open face up)
    tilt_yaw_deg: float       # azimuth offset vs mirror-law normal (deg, + = steer toward +y)
    v_n_signed: float         # racket speed along n (m/s, + = driving into the ball)
    v_t_vec: np.ndarray       # (2,) tangential racket velocity in (b1, b2) face basis (m/s)
    v_r: np.ndarray           # full 3D racket contact-point velocity (m/s)
    face_basis: np.ndarray    # (2, 3) rows b1, b2 spanning the face plane

    # --- predicted outcome ---
    v_plus: np.ndarray        # outgoing ball velocity (m/s)
    omega_plus: np.ndarray    # outgoing ball spin (rad/s)
    landing_xy: np.ndarray    # (2,) predicted first table-plane crossing (m)
    landing_time: float       # flight time strike -> landing (s)

    # --- landing sensitivities (the control-precision budget) ---
    d_landing_d_pitch: np.ndarray   # (2,) m per DEGREE of pitch tilt
    d_landing_d_yaw: np.ndarray     # (2,) m per DEGREE of yaw tilt
    d_landing_d_v_n: np.ndarray     # (2,) m per m/s of normal speed
    d_landing_d_v_t: np.ndarray     # (2,) m per m/s of |v_t| (along current v_t dir)

    # --- solver diagnostics ---
    residual_m: float         # final ||landing - target|| (m)
    iterations: int           # LM iterations used


class StrikeSpecPlanner:
    """Invert contact + flight: landing target -> racket control variables."""

    # LM tuning: converge at 5 mm within 30 iterations (task spec); the speed
    # regularization keeps the 5-var / 2-residual system well-posed and picks
    # the least-effort racket motion among the landing-equivalent solutions.
    MAX_ITER = 30
    TOL_M = 0.005
    W_SPEED = 0.03            # sqrt of the speed regularization weight (~1e-3)

    def __init__(
        self,
        physics: Optional[BallPhysics] = None,
        config: Optional[PlannerConfig] = None,
        table: Optional[TableParams] = None,
    ):
        self.physics = physics or BallPhysics()
        self.config = config or PlannerConfig()
        self.table = table or TableParams()
        # Reuse the Stage-2 integrator for the post-strike flight so the
        # landing model cannot drift from the incoming-ball prediction.
        self.predictor = BallTrajectoryPredictor(self.physics, self.config, self.table)

    # ------------------------------------------------------------------ #
    # geometry helpers
    # ------------------------------------------------------------------ #

    @staticmethod
    def _face_basis(n: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """Orthonormal (b1, b2) spanning the face plane of unit normal n.

        b1 = z x n normalized (horizontal in the face), b2 = n x b1 (face-up,
        positive z for any non-vertical n). Falls back to the x axis when the
        face points straight up/down (never a real return, but keep it total).
        """
        b1 = np.cross(_Z_HAT, n)
        norm = np.linalg.norm(b1)
        if norm < 1e-9:
            b1 = np.cross(np.array([1.0, 0.0, 0.0]), n)
            norm = np.linalg.norm(b1)
        b1 = b1 / norm
        b2 = np.cross(n, b1)
        return b1, b2

    @staticmethod
    def _normal_from_tilt(phi0: float, theta0: float, pitch_deg: float, yaw_deg: float) -> np.ndarray:
        """Unit normal at spherical offset (pitch = elevation, yaw = azimuth).

        Spherical parameterization keeps the two tilt angles independent and
        directly human-readable as degrees of face rotation.
        """
        theta = theta0 + np.deg2rad(pitch_deg)
        phi = phi0 + np.deg2rad(yaw_deg)
        ct = np.cos(theta)
        return np.array([ct * np.cos(phi), ct * np.sin(phi), np.sin(theta)])

    # ------------------------------------------------------------------ #
    # forward model: control variables -> landing
    # ------------------------------------------------------------------ #

    def _forward(
        self,
        q: np.ndarray,
        phi0: float,
        theta0: float,
        p_strike: np.ndarray,
        v_ball: np.ndarray,
        omega_ball: np.ndarray,
    ) -> Optional[dict]:
        """Evaluate contact + flight for q = (pitch, yaw, v_n, v_t_x, v_t_y)."""
        n = self._normal_from_tilt(phi0, theta0, q[0], q[1])
        b1, b2 = self._face_basis(n)
        v_r = q[2] * n + q[3] * b1 + q[4] * b2
        v_plus, omega_plus = predict_paddle_contact(
            v_ball, v_r, n, omega_ball, self.physics, self.config
        )
        landing = self.predictor.integrate_to_table_plane(p_strike, v_plus, omega_plus)
        if landing is None:
            return None
        landing_xy, t_land = landing
        return {
            "n": n, "b1": b1, "b2": b2, "v_r": v_r,
            "v_plus": v_plus, "omega_plus": omega_plus,
            "landing_xy": landing_xy, "t_land": t_land,
        }

    def _residual(self, fwd: dict, q: np.ndarray, target_xy: np.ndarray) -> np.ndarray:
        """Landing error + small racket-speed regularization (see W_SPEED)."""
        return np.concatenate([
            fwd["landing_xy"] - target_xy,
            self.W_SPEED * q[2:5],
        ])

    # ------------------------------------------------------------------ #
    # initial guess: mirror law + restitution speed heuristic
    # ------------------------------------------------------------------ #

    def _initial_guess(
        self, p_strike: np.ndarray, v_ball: np.ndarray, target_xy: np.ndarray,
    ) -> Tuple[float, float, float]:
        """(phi0, theta0, v_n0): mirror-law normal + normal-speed seed.

        Same construction as racket_target_planner._compute_racket_velocity:
        vacuum-ballistic outgoing velocity over delta_t_flight, normal along
        v_out - v_in (opponent-facing), v_n from the e(u_n) fixed point. The
        LM solver then absorbs drag/Magnus/tangential-grab, which the mirror
        law ignores.
        """
        T = self.config.delta_t_flight
        p_land = np.array([target_xy[0], target_xy[1], 0.0])
        v_out = (p_land - p_strike) / T - 0.5 * self.physics.g * T

        delta_v = v_out - v_ball
        norm = np.linalg.norm(delta_v)
        n0 = delta_v / norm if norm > 1e-9 else np.array([1.0, 0.0, 0.0])
        if n0[0] < 0.0:
            n0 = -n0  # opponent-facing (+x) convention, as in Stage 3

        v_o_n = float(np.dot(v_out, n0))
        v_i_n = float(np.dot(v_ball, n0))
        e = self.config.C_r
        v_n0 = (v_o_n + e * v_i_n) / (1.0 + e)
        for _ in range(3):  # fixed point on e(u_n), converges in 2-3 iterations
            u_n = abs(v_i_n - v_n0)
            e = float(np.clip(
                self.config.e_exp_g1 * np.exp(self.config.e_exp_g2 * u_n), 0.05, 0.95))
            v_n0 = (v_o_n + e * v_i_n) / (1.0 + e)

        phi0 = float(np.arctan2(n0[1], n0[0]))
        theta0 = float(np.arcsin(np.clip(n0[2], -1.0, 1.0)))
        return phi0, theta0, v_n0

    # ------------------------------------------------------------------ #
    # solver
    # ------------------------------------------------------------------ #

    def solve(
        self,
        p_ball: np.ndarray,
        v_ball: np.ndarray,
        omega_ball: Optional[np.ndarray],
        landing_target_xy: np.ndarray,
        racket_speed_budget: float,
    ) -> Optional[StrikeSpec]:
        """Solve for the strike spec that lands the ball at the target.

        Parameters
        ----------
        p_ball, v_ball, omega_ball : ball state AT the strike (world frame);
            omega_ball None means spin-blind (zeros), matching the legacy path.
        landing_target_xy : (2,) desired first-bounce point on the table plane.
        racket_speed_budget : max allowed |v_r| (m/s); specs beyond it -> None.

        Returns StrikeSpec, or None when LM does not reach 5 mm in 30
        iterations or the solution exceeds the speed budget.
        """
        p_ball = np.asarray(p_ball, dtype=float)
        v_ball = np.asarray(v_ball, dtype=float)
        omega_ball = (
            np.zeros(3) if omega_ball is None else np.asarray(omega_ball, dtype=float)
        )
        target_xy = np.asarray(landing_target_xy, dtype=float)[:2]

        phi0, theta0, v_n0 = self._initial_guess(p_ball, v_ball, target_xy)
        q = np.array([0.0, 0.0, v_n0, 0.0, 0.0])

        fwd = self._forward(q, phi0, theta0, p_ball, v_ball, omega_ball)
        if fwd is None:
            return None
        r = self._residual(fwd, q, target_xy)
        cost = float(r @ r)

        # Jacobian steps sized to each variable's natural scale (deg / m/s).
        h = np.array([0.2, 0.2, 0.02, 0.02, 0.02])
        lam = 1e-3
        iterations = 0

        for _ in range(self.MAX_ITER):
            iterations += 1
            if np.linalg.norm(fwd["landing_xy"] - target_xy) < self.TOL_M:
                break

            # Forward-difference Jacobian (5 extra rollouts/iter; central
            # differences are saved for the reported sensitivities below).
            J = np.zeros((r.size, 5))
            failed = False
            for j in range(5):
                q_j = q.copy()
                q_j[j] += h[j]
                fwd_j = self._forward(q_j, phi0, theta0, p_ball, v_ball, omega_ball)
                if fwd_j is None:
                    failed = True
                    break
                J[:, j] = (self._residual(fwd_j, q_j, target_xy) - r) / h[j]
            if failed:
                lam *= 10.0  # retreat toward gradient descent near the model's edge
                if lam > 1e6:
                    return None
                continue

            JtJ = J.T @ J
            g = J.T @ r
            # Marquardt scaling + small absolute floor for rank-deficient J.
            damp = np.diag(np.diag(JtJ)) + 1e-9 * np.eye(5)
            accepted = False
            for _try in range(6):
                try:
                    dq = np.linalg.solve(JtJ + lam * damp, -g)
                except np.linalg.LinAlgError:
                    lam *= 10.0
                    continue
                q_new = q + dq
                fwd_new = self._forward(q_new, phi0, theta0, p_ball, v_ball, omega_ball)
                if fwd_new is not None:
                    r_new = self._residual(fwd_new, q_new, target_xy)
                    cost_new = float(r_new @ r_new)
                    if cost_new < cost:
                        q, fwd, r, cost = q_new, fwd_new, r_new, cost_new
                        lam = max(lam * 0.3, 1e-8)
                        accepted = True
                        break
                lam *= 10.0
            if not accepted and lam > 1e6:
                return None

        residual_m = float(np.linalg.norm(fwd["landing_xy"] - target_xy))
        if residual_m >= self.TOL_M:
            return None
        v_r = fwd["v_r"]
        if float(np.linalg.norm(v_r)) > racket_speed_budget + 1e-9:
            return None

        return self._build_spec(
            q, fwd, phi0, theta0, p_ball, v_ball, omega_ball, residual_m, iterations
        )

    # ------------------------------------------------------------------ #
    # sensitivities + spec assembly
    # ------------------------------------------------------------------ #

    def _landing_of(self, q, phi0, theta0, p_ball, v_ball, omega_ball) -> Optional[np.ndarray]:
        fwd = self._forward(q, phi0, theta0, p_ball, v_ball, omega_ball)
        return None if fwd is None else fwd["landing_xy"]

    def _central_diff(
        self, q, direction, step, phi0, theta0, p_ball, v_ball, omega_ball,
    ) -> np.ndarray:
        """d landing_xy / d (q along direction), central differences through
        the FULL contact + flight chain; one-sided fallback at model edges."""
        lp = self._landing_of(q + step * direction, phi0, theta0, p_ball, v_ball, omega_ball)
        lm = self._landing_of(q - step * direction, phi0, theta0, p_ball, v_ball, omega_ball)
        if lp is not None and lm is not None:
            return (lp - lm) / (2.0 * step)
        l0 = self._landing_of(q, phi0, theta0, p_ball, v_ball, omega_ball)
        if lp is not None and l0 is not None:
            return (lp - l0) / step
        if lm is not None and l0 is not None:
            return (l0 - lm) / step
        return np.array([np.nan, np.nan])

    def _build_spec(
        self, q, fwd, phi0, theta0, p_ball, v_ball, omega_ball, residual_m, iterations,
    ) -> StrikeSpec:
        args = (phi0, theta0, p_ball, v_ball, omega_ball)
        eye = np.eye(5)
        # Steps: 0.5 deg / 0.1 m/s — large enough to clear the Euler-grid
        # landing quantization (~0.5 mm), small vs the reported slopes.
        d_pitch = self._central_diff(q, eye[0], 0.5, *args)
        d_yaw = self._central_diff(q, eye[1], 0.5, *args)
        d_v_n = self._central_diff(q, eye[2], 0.1, *args)

        # |v_t| direction: along the solved tangential velocity; when the
        # solution has ~zero v_t, probe the face-up direction b2 (the loft /
        # topspin channel — the tangential axis a controller actually uses).
        v_t = q[3:5]
        v_t_norm = float(np.linalg.norm(v_t))
        if v_t_norm > 1e-6:
            dir_t = np.concatenate([np.zeros(3), v_t / v_t_norm])
        else:
            dir_t = np.array([0.0, 0.0, 0.0, 0.0, 1.0])
        d_v_t = self._central_diff(q, dir_t, 0.1, *args)

        return StrikeSpec(
            n=fwd["n"],
            tilt_pitch_deg=float(q[0]),
            tilt_yaw_deg=float(q[1]),
            v_n_signed=float(q[2]),
            v_t_vec=q[3:5].copy(),
            v_r=fwd["v_r"],
            face_basis=np.vstack([fwd["b1"], fwd["b2"]]),
            v_plus=fwd["v_plus"],
            omega_plus=fwd["omega_plus"],
            landing_xy=fwd["landing_xy"],
            landing_time=float(fwd["t_land"]),
            d_landing_d_pitch=d_pitch,
            d_landing_d_yaw=d_yaw,
            d_landing_d_v_n=d_v_n,
            d_landing_d_v_t=d_v_t,
            residual_m=residual_m,
            iterations=iterations,
        )
