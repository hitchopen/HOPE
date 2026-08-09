"""Physical constants and tuning parameters for the HOPE planner.

Values follow the HOPE canonical world frame (origin at P1 near-side left
corner, X toward P2, Y left, Z up) and ITTF regulation table dimensions.
See HOPE_7DOF_Racket_Model_based_Planner_Reference_Setup.md, Section 2.
"""

from dataclasses import dataclass, field

import numpy as np


@dataclass
class TableParams:
    """ITTF regulation table dimensions in the HOPE canonical frame."""

    length: float = 2.74          # m, along X
    width: float = 1.525          # m, along -Y
    height: float = 0.76          # m, table surface above floor
    # Table Y span = [y_max - width, y_max]. Default 0.0 = the ARENA convention
    # (origin at the P1 near-side LEFT corner, table entirely at y<=0). The sim
    # closed-loop harness centers the table on the robot (robot at y=0) and MUST
    # override this (hope_planner.sim.yaml table_y_max: 0.7825) — a hardcoded
    # arena band made every backhand serve off-table for the predictor: no bounce
    # modeled, floor-slide garbage plans, zero backhand engages (2026-07-05).
    y_max: float = 0.0            # m, table's +Y edge
    net_x: float = 1.37           # m, net position along X
    net_height: float = 0.1525    # m, net height above table surface
    net_overhang: float = 0.15    # m, net extends past each table edge in Y


@dataclass
class BallPhysics:
    """Aerodynamic and restitution parameters.

    Defaults are the 2026-07-03 venue fit on the MATCH ball (retro-reflective
    coated, 3.4 g) — single source of truth: configs/ball_physics_venue.yaml
    + docs/ball_physics_fit_report.md. calibrate_ball_physics can refit from
    recorded trajectories; its estimator is cruder than the yaml pipeline, so
    prefer the yaml values unless the venue changes.
    """

    k: float = 0.1261            # 1/m — QUADRATIC drag accel coefficient (a = -k|v|v).
                                 # Venue fit (C_d ~ 0.57 coated ball). The old default 0.5
                                 # (mislabeled "s/m") over-dragged 4x.
    C_h: float = 0.64            # horizontal (tangential) bounce retention. No-spin equivalent
                                 # of the grip tangential block: v_t+ = (1 - a_t) v_t with
                                 # a_t = 0.369 (101-bounce M-matrix 0.641). NOTE this diagonal
                                 # model cannot represent spin<->velocity coupling at the bounce;
                                 # incoming topspin makes the real outgoing v_t larger.
    C_v: float = 0.9215          # vertical restitution e_n, venue table fit (v_n 1.0-4.5 m/s,
                                 # forensics-corrected; old table read 0.908).
    g: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0, -9.81]))
    radius: float = 0.02         # ball radius, 40 mm diameter
    mass: float = 0.0034         # 3.4 g — the coated MATCH ball (clean ITTF ball is 2.7 g)


@dataclass
class PlannerConfig:
    """Planner tuning parameters."""

    # State estimation
    poly_order: int = 2           # polynomial fit order
    fit_window: int = 31          # number of position samples for velocity fit
                                  # (~103 ms at 300 Hz; venue noise-floor MC recommends >=100 ms —
                                  # the rig noise is ~1.9 mm white + AR(1) rho~0.94 colored)
    fit_window_min_span_s: float = 0.0   # optional minimum elapsed time before estimate is ready
    fit_window_max_span_s: float = 0.15  # cap by elapsed time as well as sample count; prevents
                                         # a callback-rate drop from stretching 31 samples across
                                         # a large fraction of a flight
    # Optional X/Y fit order. None keeps the legacy all-axis ``poly_order``
    # behavior. ``1`` selects a linear horizontal fit while Z remains
    # quadratic; this is exposed for causal field-log comparison and is never
    # used as a validity/release gate.
    horizontal_poly_order: int | None = None
    mocap_hz: float = 300.0       # documentation constant (not consumed anywhere): ChingMu/VRPN
                                  # rig streams 300 Hz, OptiTrack rigs commonly 360. The actual
                                  # rate lives in the mocap bridge; the rate-coupled knob is
                                  # fit_window above (ROS param since 2026-07-18; keep the
                                  # window >= 100 ms: round(31 * rate / 300)).

    # Trajectory prediction
    dt_integrate: float = 0.001   # integration time step (s)
    max_predict_time: float = 2.0  # max forward prediction horizon (s)
    adaptive_predict_horizon: bool = False  # let incoming flights search past base if needed; never shorten
    max_predict_time_cap: float = 3.0  # absolute cap for the adaptive extension (s)
    bounce_z_tol: float = 0.005   # z threshold for bounce detection (m) — LEGACY point-ball /
                                  # bottom-of-ball data only (sim harness). Real mocap tracks the
                                  # ball CENTER whose minimum height at contact is the RADIUS
                                  # (0.02 m > tol), so this condition alone NEVER fires on venue
                                  # data (audit 2026-07-07); the center-geometry local-minimum
                                  # detector below is the one that fires there.
    bounce_center_z_max: float = 0.20  # center-geometry bounce detector: a local z-MINIMUM below
                                  # this height (radius 0.02 plus margin for BEST_EFFORT sample
                                  # loss / occlusion re-lock) counts as a table contact and clears the
                                  # estimator buffer. A rare false clear only costs ~20 ms of fit
                                  # warm-up; fitting ACROSS a real bounce corrupts the velocity
                                  # for the whole 103 ms window.

    # Racket planning
    x_hit: float = 0.0            # virtual hitting plane X coordinate (m)
    target_land: np.ndarray = field(
        default_factory=lambda: np.array([2.055, -0.7625, 0.0])
    )                             # center of opponent's half
    delta_t_flight: float = 0.5   # desired post-strike flight time (s)
    C_r: float = 0.654            # ball-racket normal restitution — FIRST real racket fit
                                  # (150 strikes, venue 2026-07-03). Used as the constant
                                  # fallback / fixed-point seed; the planner prefers the
                                  # velocity-dependent form below (F4: e falls with |u_n|).
    e_exp_g1: float = 0.759       # e(u_n) = g1 * exp(g2 * |u_n|), valid u_n 1.4-7.2 m/s
    e_exp_g2: float = -0.0441     # (u_n = normal approach speed in the racket frame)
    racket_radius: float = 0.075  # 7.5 cm paddle radius

    # --- Ball estimation / prediction upgrade (flag-gated; defaults keep legacy behavior) ---
    use_kalman: bool = False      # select the physics EKF as the active Stage-1 estimator;
                                  # false preserves the legacy polynomial estimator
    bounce_model: str = "diagonal"  # "diagonal" (legacy) | "nakashima" spin-coupled table bounce
    sigma_white_m: float = 0.0019   # capture.position_noise.white_mm, configs/ball_physics_venue.yaml
    sigma_ar1_m: float = 0.0052     # capture.position_noise.ar1_marginal_mm, configs/ball_physics_venue.yaml
    ar1_tau_s: float = 0.060        # capture.position_noise.ar1_tau_ms (~rho 0.946 @300 Hz), configs/ball_physics_venue.yaml
    q_accel_psd: float = 0.1        # m^2/s^3 unmodeled-accel PSD; conservative prior, NOT a venue fit
    chi2_gate: float = 16.3         # 3-dof chi-square @ ~0.999 innovation gate (measurement outliers)
    estimator_track_gap_s: float = 0.10  # longer gaps start a new physical-flight state
    bounce_sigma_t: float = 0.2     # m/s post-bounce tangential vel std (grip refit degenerate, F5 in fit report)
    k_m: float = 0.00444            # Magnus accel coeff, flight.k_m, configs/ball_physics_venue.yaml
    mu_table: float = 0.25          # table friction, Ace prior — UNFITTED (venue F5 tangential refit degenerate)

    # --- Paddle spin-impulse contact (ball_contact.py port of the training-side
    # virtual_ball.predict_paddle_contact). Values MIRROR configs/
    # ball_physics_venue.yaml contact.paddle / ball — do NOT read the yaml at
    # runtime; if the venue refits, update both places (same rule as k/C_v above).
    paddle_a_t: float = 0.52        # tangential gain, velocity-channel fit (CI [0.46, 0.61]);
                                    # only a_t + b_t*cos(theta) identified (contacts near-normal)
    paddle_b_t: float = 0.0         # bootstrap CI spans 0, AIC prefers b_t = 0
    paddle_mu: float = 0.5          # friction cap mu_safety — never binds on venue data
                                    # (impulse-ratio p90 = 0.27); keeps grazing hits sane
    ball_inertia_coeff: float = 2.0 / 3.0  # hollow sphere I = c m R^2, ball.inertia_coeff
                                    # (e_exp_g1/e_exp_g2 above are the same paddle fit — reused)
