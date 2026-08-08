"""Stage 1 (upgrade) - EKF ball state estimation with colored-noise bias state.

Drop-in interface-compatible with BallStateEstimator (push / ready /
bounce_detected / estimate) but keeps a full covariance and NEVER clears its
buffer at a bounce, so an estimate is available one frame after impact instead
of ~6 frames (the polyfit estimator must refill its window).

WHY a bias state: the venue rig noise is NOT white — 1.9 mm white plus a
5.2 mm-marginal AR(1) component with ~60 ms correlation time (see
configs/ball_physics_venue.yaml `capture.position_noise`). A filter that
models only white noise either trusts the colored wander (velocity bias) or
over-smooths. We therefore estimate a 9-state
    x = [p(3), v(3), b(3)]
where b is the per-axis AR(1)/Ornstein-Uhlenbeck measurement bias and the
measurement is z = p + b + white. The flight model (drag + gravity + optional
Magnus with exogenous omega) is the same one the predictor integrates, so
low-frequency measurement wander that is inconsistent with ballistic dynamics
is attributed to b, not to p/v.

Bounce handling uses the same table-bounded physical crossing as Stage 2, then
maps the state and covariance through v+ = diag(C_h, C_h, -C_v) v- instead of
clearing history. Measurement residuals are robust-clipped while their raw
chi-square and clip count remain observable telemetry.
"""

from typing import Optional, Tuple

import numpy as np

from .constants import BallPhysics, PlannerConfig, TableParams


def _skew(w: np.ndarray) -> np.ndarray:
    """Skew-symmetric matrix so that _skew(w) @ v == np.cross(w, v)."""
    return np.array([
        [0.0, -w[2], w[1]],
        [w[2], 0.0, -w[0]],
        [-w[1], w[0], 0.0],
    ])


class BallKalmanEstimator:
    """9-state EKF: ball position/velocity + AR(1) colored measurement bias.

    Process model (continuous, discretized per-step with the actual dt):
        p_dot = v
        v_dot = g - k_d |v| v + k_m (omega x v)   [omega: exogenous, set_omega]
        b     : Ornstein-Uhlenbeck, tau = ar1_tau_s
    Measurement: z = p + b + n,  H = [I 0 I],  R = sigma_white^2 I.

    Large innovation residuals are clipped (and counted); the covariance
    update uses the Joseph form so P stays PSD through robustification,
    bounces, and measurement gaps.
    """

    def __init__(
        self,
        config: PlannerConfig,
        physics: Optional[BallPhysics] = None,
        table: Optional[TableParams] = None,
    ):
        self.config = config
        self.physics = physics or BallPhysics()
        self.table = table or TableParams()

        self._omega = np.zeros(3)   # exogenous spin input for the Magnus term
        self._x = np.zeros(9)       # [p, v, b]
        self._P = np.eye(9)
        self._t: Optional[float] = None
        self._t_first: Optional[float] = None
        self._n_samples = 0
        self._n_updates = 0
        self._n_rejected = 0
        self._n_track_restarts = 0
        self._last_restart_reason = "initial"
        self._last_innovation_chi2 = float("nan")
        self._bounce_detected = False

        # Constant measurement model.
        self._H = np.zeros((3, 9))
        self._H[:, 0:3] = np.eye(3)
        self._H[:, 6:9] = np.eye(3)
        self._R = (config.sigma_white_m ** 2) * np.eye(3)

    # ------------------------------------------------------------------
    # Interface shared with BallStateEstimator
    # ------------------------------------------------------------------

    def reset(self) -> None:
        """Forget all state (full re-init on the next push)."""
        self._t = None
        self._t_first = None
        self._n_samples = 0
        self._n_updates = 0
        self._n_rejected = 0
        self._n_track_restarts = 0
        self._last_restart_reason = "manual"
        self._last_innovation_chi2 = float("nan")
        self._bounce_detected = False

    @property
    def bounce_detected(self) -> bool:
        """True if the most recent push() detected a table bounce."""
        return self._bounce_detected

    @property
    def ready(self) -> bool:
        """True once enough updates ran for the velocity to have converged.

        Six samples establish the initial velocity without imposing the legacy
        100 ms polynomial span. Unlike the legacy estimator this never goes
        back to False at a physical bounce.
        """
        return self._n_updates >= 6

    @property
    def rejected_count(self) -> int:
        """Number of innovations robust-clipped at the configured chi2 limit."""
        return self._n_rejected

    @property
    def sample_count(self) -> int:
        """Number of source measurements ingested in the current track."""
        return self._n_samples

    @property
    def sample_span_s(self) -> float:
        """Elapsed source time represented by the current physical state."""
        if self._t is None or self._t_first is None:
            return 0.0
        return max(0.0, float(self._t - self._t_first))

    @property
    def last_innovation_chi2(self) -> float:
        """Raw latest innovation chi-square, for audit only."""
        return self._last_innovation_chi2

    @property
    def track_restart_count(self) -> int:
        """Number of source-gap flight-state restarts since manual reset."""
        return self._n_track_restarts

    @property
    def last_restart_reason(self) -> str:
        """Latest flight-state restart reason, for audit only."""
        return self._last_restart_reason

    @property
    def position_std(self) -> np.ndarray:
        """Per-axis filtered position standard deviation, for audit only."""
        return np.sqrt(np.maximum(0.0, np.diag(self._P)[0:3]))

    @property
    def velocity_std(self) -> np.ndarray:
        """Per-axis filtered velocity standard deviation, for audit only."""
        return np.sqrt(np.maximum(0.0, np.diag(self._P)[3:6]))

    def set_omega(self, w: np.ndarray) -> None:
        """Set the exogenous ball spin (rad/s) used by the Magnus term."""
        self._omega = np.asarray(w, dtype=float).copy()

    def push(self, t: float, p: np.ndarray) -> None:
        """Ingest one position measurement (seconds, HOPE frame)."""
        z = np.asarray(p, dtype=float)
        if not np.isfinite(t) or z.shape != (3,) or not np.all(np.isfinite(z)):
            raise ValueError("ball measurement must be finite (t, xyz)")
        if self._t is not None:
            dt_from_last = float(t - self._t)
            max_track_gap = max(0.0, float(self.config.estimator_track_gap_s))
            if dt_from_last <= 0.0 or (
                max_track_gap > 0.0 and dt_from_last > max_track_gap
            ):
                self._restart_track(
                    "nonmonotonic_stamp" if dt_from_last <= 0.0 else "source_gap"
                )
        self._n_samples += 1
        self._bounce_detected = False

        if self._t is None:
            self._initialize(t, z)
            return

        dt = t - self._t
        if dt > 1e-9:
            self._predict_interval(dt)
        self._t = t

        self._measurement_update(z)

    def _restart_track(self, reason: str) -> None:
        """Drop one disconnected flight while retaining session audit counts."""
        self._t = None
        self._t_first = None
        self._n_samples = 0
        self._n_updates = 0
        self._last_innovation_chi2 = float("nan")
        self._bounce_detected = False
        self._n_track_restarts += 1
        self._last_restart_reason = reason

    def estimate(self) -> Tuple[np.ndarray, np.ndarray, float]:
        """Filtered (p, v, t) at the latest sample time — legacy tuple form."""
        if not self.ready:
            raise RuntimeError(
                "Need >= 6 measurement updates, have {}".format(self._n_updates))
        return self._x[0:3].copy(), self._x[3:6].copy(), self._t

    # ------------------------------------------------------------------
    # Extended interface
    # ------------------------------------------------------------------

    def estimate_full(self) -> Tuple[np.ndarray, np.ndarray]:
        """Full filtered state: (mean [p, v, b] (9,), covariance (9, 9))."""
        if not self.ready:
            raise RuntimeError(
                "Need >= 6 measurement updates, have {}".format(self._n_updates))
        return self._x.copy(), self._P.copy()

    def predict_to(self, t: float) -> Tuple[np.ndarray, np.ndarray]:
        """Mean flight prediction (p, v) at time t, without mutating the filter.

        Uses the same drag + gravity + Magnus model as the process step,
        sub-stepped at config.dt_integrate. Does NOT model future bounces —
        use BallTrajectoryPredictor for the full hybrid rollout.
        """
        if self._t is None:
            raise RuntimeError("No measurements yet")
        p = self._x[0:3].copy()
        v = self._x[3:6].copy()
        remaining = t - self._t
        h = self.config.dt_integrate
        while remaining > 1e-12:
            step = min(h, remaining)
            a = self._flight_accel(v)
            p = p + v * step + 0.5 * a * step ** 2
            v = v + a * step
            remaining -= step
        return p, v

    # ------------------------------------------------------------------
    # Internals
    # ------------------------------------------------------------------

    def _initialize(self, t: float, z: np.ndarray) -> None:
        self._t = t
        self._t_first = t
        self._x = np.zeros(9)
        self._x[0:3] = z
        # Position starts at the (biased) measurement; b absorbs the colored
        # offset as evidence accumulates. Velocity is unknown -> wide prior
        # covering venue ball speeds (validity envelope 1-7 m/s).
        marg2 = self.config.sigma_white_m ** 2 + self.config.sigma_ar1_m ** 2
        self._P = np.zeros((9, 9))
        self._P[0:3, 0:3] = marg2 * np.eye(3)
        self._P[3:6, 3:6] = (10.0 ** 2) * np.eye(3)
        self._P[6:9, 6:9] = (self.config.sigma_ar1_m ** 2) * np.eye(3)
        self._n_updates = 1

    def _is_on_table(self, p: np.ndarray) -> bool:
        radius = self.physics.radius
        return (
            -radius <= p[0] <= self.table.length + radius
            and self.table.y_max - self.table.width - radius
            <= p[1]
            <= self.table.y_max + radius
        )

    def _flight_accel(self, v: np.ndarray) -> np.ndarray:
        speed = np.linalg.norm(v)
        return (self.physics.g - self.physics.k * speed * v
                + self.config.k_m * np.cross(self._omega, v))

    def _predict_interval(self, dt: float) -> None:
        """Propagate over an arbitrary source gap using the Stage-2 step size."""
        remaining = float(dt)
        max_step = max(float(self.config.dt_integrate), 1.0e-6)
        while remaining > 1.0e-12:
            step = min(max_step, remaining)
            self._predict_step(step)
            remaining -= step

    def _predict_step(self, dt: float) -> None:
        """One Euler-linearized hybrid flight/bounce prediction step."""
        p = self._x[0:3]
        v = self._x[3:6]
        b = self._x[6:9]

        a = self._flight_accel(v)
        rho = np.exp(-dt / self.config.ar1_tau_s)

        # Mean propagation (2nd-order kinematic step, matches the predictor).
        self._x = np.concatenate([
            p + v * dt + 0.5 * a * dt ** 2,
            v + a * dt,
            rho * b,
        ])

        # Jacobian of v_dot wrt v: drag term + Magnus (constant in v).
        speed = np.linalg.norm(v)
        if speed > 1e-9:
            A = -self.physics.k * (speed * np.eye(3) + np.outer(v, v) / speed)
        else:
            A = np.zeros((3, 3))
        A = A + self.config.k_m * _skew(self._omega)

        F = np.eye(9)
        F[0:3, 3:6] = dt * np.eye(3)
        F[3:6, 3:6] = np.eye(3) + dt * A
        F[6:9, 6:9] = rho * np.eye(3)

        # Process noise: white-accel PSD q on (p, v); exact OU variance on b
        # so the bias marginal stays sigma_ar1 regardless of dt.
        q = self.config.q_accel_psd
        Q = np.zeros((9, 9))
        Q[0:3, 0:3] = (q * dt ** 3 / 3.0) * np.eye(3)
        Q[0:3, 3:6] = (q * dt ** 2 / 2.0) * np.eye(3)
        Q[3:6, 0:3] = Q[0:3, 3:6]
        Q[3:6, 3:6] = (q * dt) * np.eye(3)
        Q[6:9, 6:9] = (self.config.sigma_ar1_m ** 2) * (1.0 - rho ** 2) * np.eye(3)

        self._P = F @ self._P @ F.T + Q
        self._P = 0.5 * (self._P + self._P.T)

        # Use the same physical crossing condition as Stage 2.  The old raw
        # three-frame local-minimum detector fired repeatedly on millimetre
        # noise as high as 0.20 m and flipped velocity long before contact.
        # A 1 ms hybrid predict reaches the table itself and therefore needs
        # neither a noisy height threshold nor a post-bounce buffer clear.
        if self._x[2] < 0.0 and self._x[5] < 0.0:
            if self._is_on_table(self._x[0:3]):
                self._apply_bounce_transform()
                self._bounce_detected = True
            else:
                self._x[2] = 0.0

    def _apply_bounce_transform(self) -> None:
        """Map the state through the table bounce WITHOUT clearing anything.

        v+ = diag(C_h, C_h, -C_v) v- (venue values from BallPhysics). The 1 ms
        hybrid prediction can end slightly below the table, so that small
        penetration is reflected with the same restitution. The full
        covariance is transformed before adding bounce-model uncertainty.
        """
        vz = self._x[5]
        if vz < 0.0:
            C = np.diag([self.physics.C_h, self.physics.C_h, -self.physics.C_v])
            J = np.eye(9)
            J[3:6, 3:6] = C
            self._x[3:6] = C @ self._x[3:6]
            if self._x[2] < 0.0:
                penetration = -self._x[2]
                self._x[2] = self.physics.C_v * penetration
                J[2, 2] = -self.physics.C_v
            else:
                penetration = 0.0
            self._P = J @ self._P @ J.T
            # Discrete crossing-position uncertainty (z only): tunneled depth
            # plus the retained contact tolerance band.
            self._P[2, 2] += penetration ** 2 + self.config.bounce_z_tol ** 2

        sig_t = self.config.bounce_sigma_t
        infl = np.diag([sig_t ** 2, sig_t ** 2, (0.02 * abs(vz)) ** 2])
        self._P[3:6, 3:6] += infl
        self._P = 0.5 * (self._P + self._P.T)

    def _measurement_update(self, z: np.ndarray) -> None:
        H = self._H
        y = z - H @ self._x
        S = H @ self._P @ H.T + self._R
        S_inv = np.linalg.inv(S)

        chi2 = float(y @ S_inv @ y)
        self._last_innovation_chi2 = chi2
        if chi2 > self.config.chi2_gate:
            # Robustify instead of dropping the whole measurement.  The raw
            # chi2 and clip count are audit telemetry only; no confidence
            # threshold participates in command publication or runner release.
            self._n_rejected += 1
            y *= np.sqrt(self.config.chi2_gate / max(chi2, 1.0e-12))

        K = self._P @ H.T @ S_inv
        self._x = self._x + K @ y
        # Joseph form: PSD-preserving even with a suboptimal/gated K.
        I_KH = np.eye(9) - K @ H
        self._P = I_KH @ self._P @ I_KH.T + K @ self._R @ K.T
        self._P = 0.5 * (self._P + self._P.T)
        self._n_updates += 1
