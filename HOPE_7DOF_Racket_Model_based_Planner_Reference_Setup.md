# HOPE 7DOF Racket Model-based Planner Reference Setup

v0.1 — 2026-03-19

## Overview

This document provides a reference implementation of the model-based planner (Stages 1–3) for computing the desired 7-DOF racket state — interception position, velocity, and face orientation — for a humanoid ping-pong player. The algorithms are adapted from the HITTER framework (Su et al., arXiv:2508.21043v2, 2025) and operate within the HOPE canonical world frame defined in the companion document *HOPE Motion Capture System Reference Setup for Ping-Pong Arena*.

The planner consumes ball position data from the HOPE motion capture system and produces a `RacketCommand` message specifying where and how the humanoid's paddle must arrive to return the ball. Per the HOPE racket exclusion policy, the paddle's actual pose is **never measured by the motion capture system**; the motion capture system tracks only three categories of objects — the table origin frame (PPT), humanoid base_links (P1, P2), and the ping-pong ball. Each humanoid must achieve the commanded racket state through its own forward kinematics from `base_link` + joint encoders and whole-body controller. See companion *HOPE Motion Capture System Reference Setup* (Section 3.1 — Racket Exclusion Policy) and *HOPE WBC Simulation Training Reference Setup* (Section 2.8 — Racket Mount Kinematics).

---

## 0  Prologue — Relationship to the HITTER Paper

### 0.1  Scope

The HITTER paper describes a four-stage hierarchical framework. This document reimplements **Stages 1–3 only** — the model-based planner — providing the complete algorithmic pipeline from ball position observations to the desired racket state `(p̂_intercept, v̂_racket, n̂_racket, t_strike)`.

Stage 4, the whole-body controller (WBC), is a PPO-trained reinforcement learning policy whose weights are not publicly available. Each HOPE team is expected to provide their own WBC or IK-based controller to execute the planner's racket commands.

**HITTER's source code has not been publicly released.** The project page (humanoid-table-tennis.github.io) and the arXiv paper contain no code repository. All code in this document is a clean-room reimplementation derived solely from the equations in Sections III-A through III-C of the paper.

### 0.2  Coordinate Frame Adaptation

The HITTER paper does not explicitly publish its world frame origin. Based on the virtual hitting plane at `x = −1.37 m` and the use of OptiTrack with Z-up streaming, the paper's origin is inferred to be at the **table center**. The HOPE reference frame places the origin at the **near-side left corner of the table surface**, as defined in the HOPE Motion Capture System Reference Setup.

| | HITTER (inferred) | HOPE canonical frame |
|---|---|---|
| **Origin** | Table center | P1 near-side left corner, table surface |
| **X** | Toward opponent | Toward P2 (same direction) |
| **Y** | Left (player's perspective) | Left (same direction) |
| **Z** | Up | Up (same direction) |
| **Virtual hit plane** | x = −1.37 m | x = x_hit ≈ 0.0 m |
| **Opponent half center** | x ≈ +0.685, y ≈ 0 | x = 2.055, y = −0.7625 |

The axes are co-aligned; the transform is a pure translation:

```
x_HOPE = x_HITTER + 1.37
y_HOPE = y_HITTER − 0.7625
z_HOPE = z_HITTER
```

**Note**: If HITTER's actual (unpublished) implementation uses a different Y-axis convention, this translation must be revisited. The HOPE frame follows ROS 2 REP 103 (Y-left, ENU).

### 0.3  Differences Between This Implementation and the Published Paper

The following table documents every known point where this implementation departs from or extends the paper's description. These are not bugs — they are deliberate design choices made to fill in details the paper leaves unspecified, or to add defensive checks appropriate for a reference implementation.

| Aspect | HITTER paper (arXiv:2508.21043v2) | This implementation | Rationale |
|--------|-----------------------------------|---------------------|-----------|
| **World frame origin** | Table center (inferred) | P1 near-side left corner | Aligned with HOPE Motion Capture Reference Setup; avoids negative coordinates for on-table positions |
| **Integration method** | "step-by-step time integration" (method not specified) | Explicit Euler with dt = 1 ms | Simplest integrator consistent with "step-by-step"; adequate accuracy at 1 kHz for ~7.5 cm racket tolerance |
| **Bounce event timing** | Not specified | Linear sub-step interpolation within the crossing timestep | Improves temporal accuracy when z crosses zero mid-step |
| **Bounce detection in state estimator** | "Upon detecting a table bounce, the buffer is cleared" (detection method not specified) | Three-sample z-height pattern: z descends through threshold then rises | Paper does not specify how bounces are detected in the 360 Hz stream; three-sample pattern avoids premature triggering on the ball's descent before actual contact |
| **Table bounds for bounce** | Not discussed | Ball center bounds expanded by ball radius (±20 mm) | Handles edge bounces where the ball center is slightly outside the table edge but the ball surface contacts the table |
| **Net clearance validation** | Not described in the planner | Added: checks both height clearance and Y-axis net extent; auto-adjusts flight time if clearance fails | The paper may rely on the WBC to handle net failures implicitly; this implementation adds an explicit safety check |
| **Racket orientation output** | WBC receives `v_racket` as a 3D vector; no quaternion | Added `normal_to_quaternion()` utility with optional roll constraint | Convenience for IK-based controllers that need a full orientation quaternion rather than just the velocity vector |
| **Drag calibration procedure** | "15 recorded ball trajectories" fitted offline (procedure not detailed) | Provided as `calibrate_ball_physics()` with least-squares fitting | Clean-room implementation of the fitting procedure described qualitatively in the paper |
| **Post-bounce kinematics** | Not specified at sub-step level | Includes quadratic position correction (½a·dt²) in the remaining sub-step after a bounce | Ensures consistent second-order accuracy across bounce boundaries |
| **Hit-plane crossing interpolation** | Not specified | Interpolation uses post-bounce state if a bounce occurred in the same timestep, ensuring interpolation spans a continuous flight arc | Prevents physically meaningless blending of pre-bounce and post-bounce velocities in rare edge cases |
| **Smoothed position estimate** | "least-squares fit of a second-order polynomial" | Polynomial value at t=0 (latest timestamp) returned as smoothed position, which may differ from raw measurement by ~2 mm | By design — polynomial denoising improves velocity accuracy at the cost of a small position offset |

### 0.4  What Is Not Implemented

The following aspects are present in the HITTER paper but outside the scope of this planner reference:

- **Spin dynamics** — The paper explicitly neglects spin. Magnus force is absent from both the paper and this implementation.
- **Strategic target selection** — The paper always aims for the opponent's half-center. No game-state-dependent target variation.
- **Whole-body controller (Stage 4)** — Requires trained RL policy weights from Isaac Lab. Each HOPE team provides their own.
- **Forehand/backhand selection** — The paper determines swing type by the sign of the ball's Y position relative to the robot. This is a WBC decision, not a planner output.
- **Racket pose measurement** — Per the HOPE racket exclusion policy, the paddle is never tracked by the motion capture system. The motion capture system provides exactly three categories of tracking: the table origin frame (PPT), each humanoid's `base_link` (P1/P2), and the ball. The planner outputs a desired racket state; achieving it is the robot's responsibility via forward kinematics from `base_link` + joint encoders. See companion *HOPE Motion Capture System Reference Setup* (Section 3.1) for the exclusion policy and enforcement, and *HOPE WBC Simulation Training Reference Setup* (Section 2.8) for the fixed wrist mount kinematics that make this work.

---

## 1  HOPE Canonical World Frame

This section restates the frame convention from the HOPE Motion Capture System Reference Setup for completeness. Both documents use the same coordinate system.

| Axis | Direction | Range on table surface |
|------|-----------|------------------------|
| **X** | Toward P2 (opponent), along table length | 0 → +2.74 m |
| **Y** | Left (from P1's perspective), along table width | 0 → −1.525 m |
| **Z** | Up (vertical) | 0 = table surface |

Key landmarks:

| Landmark | X (m) | Y (m) | Z (m) |
|----------|-------|-------|-------|
| Origin (P1 near-side left corner) | 0.0 | 0.0 | 0.0 |
| Net center line | 1.37 | −0.7625 | 0.0 |
| P1 half center | 0.685 | −0.7625 | 0.0 |
| P2 half center | 2.055 | −0.7625 | 0.0 |
| Floor below origin | 0.0 | 0.0 | −0.76 |
| Virtual hitting plane (planner) | x_hit ≈ 0.0 | — | — |
| Robot nominal standing position | ≈ −0.5 | ≈ −0.7625 | −0.76 (floor) |

---

## 2  Physical Constants and Calibration Parameters

```python
import numpy as np
from dataclasses import dataclass, field

@dataclass
class TableParams:
    """ITTF regulation table dimensions in the HOPE canonical frame."""
    length: float = 2.74          # m, along X
    width: float = 1.525          # m, along −Y
    height: float = 0.76          # m, table surface above floor
    net_x: float = 1.37           # m, net position along X
    net_height: float = 0.1525    # m, net height above table surface
    net_overhang: float = 0.15    # m, net extends past each table edge in Y

@dataclass
class BallPhysics:
    """Aerodynamic and restitution parameters.

    Calibrated from recorded ball trajectories by fitting observed
    accelerations and bounce velocity ratios. The HITTER paper uses
    15 trajectories; more data improves robustness.
    """
    k: float = 0.5               # aerodynamic drag coefficient (s/m)
                                  # typical range 0.3–0.8 for 40 mm ball
    C_h: float = 0.75            # horizontal restitution coefficient
    C_v: float = 0.85            # vertical restitution coefficient (magnitude)
    g: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0, -9.81]))
    radius: float = 0.02         # ball radius, 40 mm diameter
    mass: float = 0.0027         # 2.7 g

@dataclass
class PlannerConfig:
    """Planner tuning parameters."""
    # State estimation
    poly_order: int = 2           # polynomial fit order
    fit_window: int = 31          # number of position samples for velocity fit
    mocap_hz: float = 360.0       # motion capture frame rate

    # Trajectory prediction
    dt_integrate: float = 0.001   # integration time step (s)
    max_predict_time: float = 2.0 # max forward prediction horizon (s)
    bounce_z_tol: float = 0.005   # z threshold for bounce detection (m)

    # Racket planning
    x_hit: float = 0.0            # virtual hitting plane X coordinate (m)
    target_land: np.ndarray = field(
        default_factory=lambda: np.array([2.055, -0.7625, 0.0])
    )                             # center of opponent's half
    delta_t_flight: float = 0.5   # desired post-strike flight time (s)
    C_r: float = 0.88             # ball-racket coefficient of restitution
    racket_radius: float = 0.075  # 7.5 cm paddle radius
```

### 2.1  Calibrating k, C_h, C_v from Recorded Trajectories

Collect at least 15 ball trajectories (ball tossed onto the table, bouncing and flying) with the motion capture system recording at 360 Hz. For each trajectory, segment into flight arcs and bounce events by detecting z ≈ 0 crossings.

```python
def calibrate_ball_physics(
    trajectories: list[np.ndarray],
    timestamps: list[np.ndarray],
    g: np.ndarray = np.array([0.0, 0.0, -9.81]),
) -> BallPhysics:
    """Calibrate k, C_h, C_v from recorded ball trajectories.

    Parameters
    ----------
    trajectories : list of (N, 3) arrays
        Each array is a sequence of ball positions [x, y, z] in the HOPE frame.
    timestamps : list of (N,) arrays
        Corresponding timestamps in seconds.

    Returns
    -------
    BallPhysics with fitted parameters.
    """
    drag_samples = []
    h_restitution = []
    v_restitution = []

    for pos, t in zip(trajectories, timestamps):
        dt = np.diff(t)
        vel = np.diff(pos, axis=0) / dt[:, None]
        t_vel = 0.5 * (t[:-1] + t[1:])
        dt_vel = np.diff(t_vel)
        acc = np.diff(vel, axis=0) / dt_vel[:, None]
        vel_mid = 0.5 * (vel[:-1] + vel[1:])
        z_mid = 0.5 * (pos[:-2, 2] + pos[2:, 2])

        # Flight samples: z well above table
        flight_mask = z_mid > 0.03
        if np.any(flight_mask):
            a_flight = acc[flight_mask]
            v_flight = vel_mid[flight_mask]
            a_minus_g = a_flight - g[None, :]
            drag_samples.extend(
                zip(
                    np.linalg.norm(a_minus_g, axis=1),
                    np.linalg.norm(v_flight, axis=1) ** 2,
                )
            )

        # Bounce detection: three-sample pattern (descend → contact → rise)
        z = pos[:, 2]
        for i in range(1, len(z) - 1):
            if z[i - 1] > 0.01 and z[i] < 0.005 and z[i + 1] > 0.01:
                idx_pre = max(0, i - 2)
                idx_post = min(len(vel) - 1, i + 1)
                if idx_pre < len(vel) and idx_post < len(vel):
                    v_pre = vel[idx_pre]
                    v_post = vel[idx_post]
                    v_h_pre = np.sqrt(v_pre[0] ** 2 + v_pre[1] ** 2)
                    v_h_post = np.sqrt(v_post[0] ** 2 + v_post[1] ** 2)
                    if v_h_pre > 0.1:
                        h_restitution.append(v_h_post / v_h_pre)
                    if abs(v_pre[2]) > 0.1:
                        v_restitution.append(abs(v_post[2]) / abs(v_pre[2]))

    # Fit drag: |a - g| = k * |v|^2
    if drag_samples:
        a_mag, v2 = zip(*drag_samples)
        a_mag, v2 = np.array(a_mag), np.array(v2)
        k = float(np.dot(a_mag, v2) / np.dot(v2, v2))
    else:
        k = 0.5

    C_h = float(np.median(h_restitution)) if h_restitution else 0.75
    C_v = float(np.median(v_restitution)) if v_restitution else 0.85

    return BallPhysics(k=k, C_h=C_h, C_v=C_v)
```

---

## 3  Stage 1 — Ball State Estimation

The motion capture system delivers ball center position `p(t) = [p_x, p_y, p_z]` at 360 Hz via `motion_capture_tracking`. Velocity is not directly observed. Following HITTER, a 2nd-order polynomial is fit to the most recent N = 31 samples and differentiated analytically.

### 3.1  Theory

For each axis independently, fit:

```
p(t) = a₂ t² + a₁ t + a₀
```

via least-squares over the 31-sample window (~86 ms at 360 Hz). The velocity estimate is:

```
ṗ(t) = 2 a₂ t + a₁
```

The buffer is cleared on each detected table bounce to prevent the polynomial from fitting across the velocity discontinuity.

**Note on smoothing**: The polynomial value at the latest timestamp is the smoothed position estimate, which may differ from the raw measurement by up to ~2 mm. This is by design and improves velocity accuracy.

### 3.2  Implementation

```python
class BallStateEstimator:
    """Estimate ball position and velocity from motion capture position stream.

    Maintains a sliding window of recent position measurements and
    performs a least-squares polynomial fit to extract smoothed
    position and velocity estimates.

    Bounce detection uses a three-sample pattern (descend → contact → rise)
    to identify the actual table impact event and clear the buffer.
    """

    def __init__(self, config: PlannerConfig):
        self.config = config
        self.t_buffer: list[float] = []
        self.p_buffer: list[np.ndarray] = []

        # Bounce detection: three-sample z-height ring buffer.
        # Initialized to None to suppress false triggers before
        # enough measurements are collected.
        self._z_hist: list[float | None] = [None, None, None]
        self._bounce_detected: bool = False

    def reset(self):
        """Clear the estimation buffer (call on bounce detection)."""
        self.t_buffer.clear()
        self.p_buffer.clear()

    def push(self, t: float, p: np.ndarray) -> None:
        """Add a new position measurement.

        Parameters
        ----------
        t : float
            Timestamp in seconds (monotonic, e.g. from ROS clock).
        p : np.ndarray, shape (3,)
            Ball position [x, y, z] in the HOPE canonical frame.
        """
        # Update z history ring buffer
        self._z_hist[0] = self._z_hist[1]
        self._z_hist[1] = self._z_hist[2]
        self._z_hist[2] = p[2]

        # Bounce detection: three-sample pattern
        # z_prev_prev was above table, z_prev dipped to contact,
        # z_curr is rising again — actual bounce event
        self._bounce_detected = False
        z_pp, z_p, z_c = self._z_hist
        tol = self.config.bounce_z_tol
        if z_pp is not None and z_p is not None and z_c is not None:
            if z_pp > tol and z_p <= tol and z_c > tol:
                self._bounce_detected = True
                self.reset()

        self.t_buffer.append(t)
        self.p_buffer.append(p.copy())

        if len(self.t_buffer) > self.config.fit_window:
            self.t_buffer.pop(0)
            self.p_buffer.pop(0)

    @property
    def bounce_detected(self) -> bool:
        """True if the most recent push() detected a table bounce."""
        return self._bounce_detected

    @property
    def ready(self) -> bool:
        """True if enough samples exist for a reliable fit."""
        return len(self.t_buffer) >= 6

    def estimate(self) -> tuple[np.ndarray, np.ndarray, float]:
        """Compute smoothed ball position and velocity at the latest timestamp.

        Returns
        -------
        p_est : np.ndarray, shape (3,)
            Smoothed position estimate [x, y, z] in HOPE frame.
        v_est : np.ndarray, shape (3,)
            Velocity estimate [vx, vy, vz] in HOPE frame (m/s).
        t_est : float
            Timestamp of the estimate (latest sample time).
        """
        if not self.ready:
            raise RuntimeError(
                f"Need >= 6 samples, have {len(self.t_buffer)}"
            )

        t_arr = np.array(self.t_buffer)
        p_arr = np.array(self.p_buffer)

        # Normalize time to improve numerical conditioning
        t_ref = t_arr[-1]
        t_norm = t_arr - t_ref

        p_est = np.zeros(3)
        v_est = np.zeros(3)

        for axis in range(3):
            # np.polyfit returns [a2, a1, a0] (descending order)
            coeffs = np.polyfit(t_norm, p_arr[:, axis], deg=self.config.poly_order)
            p_est[axis] = coeffs[-1]   # a0 at t_norm = 0
            v_est[axis] = coeffs[-2]   # a1 at t_norm = 0

        return p_est, v_est, t_ref
```

---

## 4  Stage 2 — Ball Trajectory Prediction

Given the current ball state `(p, v)` from Stage 1, forward-integrate the trajectory using:

- **Flight dynamics**: quadratic aerodynamic drag plus gravity
- **Bounce dynamics**: diagonal restitution matrix at `z = 0` contacts

The output is the predicted ball position and velocity at the moment it crosses the virtual hitting plane at `x = x_hit`.

### 4.1  Flight Dynamics

```
a = −k |v| v + g         where g = [0, 0, −9.81] m/s²
```

Integrated with explicit Euler at `dt = 0.001 s` (1 kHz).

### 4.2  Bounce Model

```
v⁺ = C v⁻    where C = diag(C_h, C_h, −C_v)
```

Position z is clamped to zero at the bounce instant. Only bounces within the table surface bounds (expanded by ball radius for edge contacts) are applied.

### 4.3  Implementation

```python
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

    def _is_on_table(self, p: np.ndarray) -> bool:
        """Check if the ball could contact the table surface.

        Bounds are expanded by ball radius to handle edge contacts.
        """
        r = self.physics.radius
        return (
            -r <= p[0] <= self.table.length + r
            and -self.table.width - r <= p[1] <= r
        )

    def _flight_acceleration(self, v: np.ndarray) -> np.ndarray:
        """Compute ball acceleration during free flight: a = −k|v|v + g"""
        speed = np.linalg.norm(v)
        return -self.physics.k * speed * v + self.physics.g

    def _apply_bounce(self, v: np.ndarray) -> np.ndarray:
        """Apply table bounce restitution: v⁺ = diag(C_h, C_h, −C_v) @ v⁻"""
        C = np.diag([self.physics.C_h, self.physics.C_h, -self.physics.C_v])
        return C @ v

    def predict(
        self, p0: np.ndarray, v0: np.ndarray, t0: float
    ) -> StrikeTarget:
        """Forward-integrate and find the hitting-plane crossing.

        Parameters
        ----------
        p0 : Current ball position in HOPE frame.
        v0 : Current ball velocity in HOPE frame.
        t0 : Current timestamp (s).

        Returns
        -------
        StrikeTarget with predicted ball state at the virtual hitting plane.
        """
        dt = self.config.dt_integrate
        max_steps = int(self.config.max_predict_time / dt)
        x_hit = self.config.x_hit

        p = p0.copy()
        v = v0.copy()
        t = t0
        bounces = 0
        bounce_this_step = False

        for step in range(max_steps):
            p_prev_x = p[0]

            # --- Euler integration step ---
            a = self._flight_acceleration(v)
            v_new = v + a * dt
            p_new = p + v * dt + 0.5 * a * dt**2
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

                    v_post = self._apply_bounce(v_at_bounce)

                    # Continue from bounce with second-order correction
                    remaining_dt = (1.0 - frac) * dt
                    a_post = self._flight_acceleration(v_post)
                    p_new = p_bounce + v_post * remaining_dt + 0.5 * a_post * remaining_dt**2
                    v_new = v_post + a_post * remaining_dt
                    bounces += 1
                    bounce_this_step = True
                else:
                    p_new[2] = max(p_new[2], 0.0)

            # --- Hitting plane crossing detection ---
            if p_prev_x > x_hit and p_new[0] <= x_hit and v_new[0] < 0:
                # Interpolate within a continuous flight arc only
                if bounce_this_step:
                    # Use post-bounce arc for interpolation
                    dx_arc = p_new[0] - p_bounce[0]
                    if abs(dx_arc) > 1e-9:
                        frac_cross = (p_bounce[0] - x_hit) / (p_bounce[0] - p_new[0])
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

                return StrikeTarget(
                    p_ball=p_cross, v_ball=v_cross,
                    t_strike=t_cross, num_bounces=bounces, valid=True,
                )

            p = p_new
            v = v_new

        return StrikeTarget(
            p_ball=p, v_ball=v, t_strike=t,
            num_bounces=bounces, valid=False,
        )
```

---

## 5  Stage 3 — Racket Target Planning

Given the predicted ball state at the hitting plane from Stage 2, compute the desired racket velocity and orientation to return the ball to a target landing point on the opponent's side.

### 5.1  Desired Outgoing Ball Velocity

```
v_o = (p̂_land − p̂_intercept) / Δt  +  ½ g Δt
```

This is a simplified ballistic calculation neglecting drag during the return flight.

### 5.2  Desired Racket Velocity and Orientation

```
û = (v_o − v_i) / |v_o − v_i|              (racket face normal)
v̂_racket = [(v_o · û) + C_r (v_i · û)] / (1 + C_r) × û
```

### 5.3  Net Clearance Validation

The return trajectory must clear the net at `x = 1.37 m`, `z_net = 0.1525 m`, and the ball's Y at the net must fall within the net's lateral extent (table width + 0.15 m overhang on each side).

### 5.4  Implementation

```python
@dataclass
class RacketCommand:
    """Output of Stage 3: desired racket state at strike time.

    This is the planner's output to the whole-body controller.
    The racket's actual pose is inferred by the humanoid via forward
    kinematics — it is never measured by the motion capture system.
    """
    p_intercept: np.ndarray   # desired racket center position at interception
                              # (= predicted ball position at hitting plane)
    v_racket: np.ndarray      # desired racket velocity vector [vx, vy, vz]
    n_racket: np.ndarray      # desired racket face normal (unit vector)
    t_strike: float           # predicted time of strike
    v_ball_outgoing: np.ndarray  # expected outgoing ball velocity
    target_land: np.ndarray   # intended landing point
    clears_net: bool          # True if return trajectory clears the net
    bypasses_net_posts: bool  # True if ball passes outside net Y extent
    valid: bool               # True if all computations succeeded


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
        """v_o = (p_land − p_strike) / Δt + ½ g Δt"""
        return (p_land - p_strike) / delta_t + 0.5 * self.physics.g * delta_t

    def _compute_racket_velocity(
        self, v_incoming: np.ndarray, v_outgoing: np.ndarray, C_r: float,
    ) -> tuple[np.ndarray, np.ndarray]:
        """Compute desired racket velocity and face normal from impact model."""
        delta_v = v_outgoing - v_incoming
        delta_v_norm = np.linalg.norm(delta_v)

        if delta_v_norm < 1e-6:
            n = -v_incoming / max(np.linalg.norm(v_incoming), 1e-6)
            return np.zeros(3), n

        u_hat = delta_v / delta_v_norm
        v_o_n = np.dot(v_outgoing, u_hat)
        v_i_n = np.dot(v_incoming, u_hat)
        v_r_n = (v_o_n + C_r * v_i_n) / (1.0 + C_r)

        return v_r_n * u_hat, u_hat

    def _check_net_clearance(
        self, p_strike: np.ndarray, v_outgoing: np.ndarray, margin: float = 0.03,
    ) -> tuple[bool, bool]:
        """Check height clearance and Y-axis net extent."""
        x_net = self.table.net_x
        z_net = self.table.net_height

        dx = x_net - p_strike[0]
        if v_outgoing[0] <= 0:
            return False, False

        t_net = dx / v_outgoing[0]
        if t_net < 0:
            return False, False

        z_at_net = p_strike[2] + v_outgoing[2] * t_net + 0.5 * self.physics.g[2] * t_net**2
        y_at_net = p_strike[1] + v_outgoing[1] * t_net

        y_net_min = -self.table.width - self.table.net_overhang
        y_net_max = self.table.net_overhang

        bypasses_posts = (y_at_net < y_net_min) or (y_at_net > y_net_max)
        if bypasses_posts:
            return False, True

        return z_at_net > (z_net + margin), False

    def plan(self, strike: StrikeTarget) -> RacketCommand:
        """Compute racket target state for a valid return."""
        if not strike.valid:
            return RacketCommand(
                p_intercept=strike.p_ball, v_racket=np.zeros(3),
                n_racket=np.array([1.0, 0.0, 0.0]), t_strike=strike.t_strike,
                v_ball_outgoing=np.zeros(3), target_land=self.config.target_land,
                clears_net=False, bypasses_net_posts=False, valid=False,
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
        )
```

---

## 6  Complete Planner Pipeline

```python
class HOPEPlanner:
    """Top-level planner combining Stages 1–3.

    Call .update() with each ball position at the motion capture frame rate.
    Retrieve .racket_command for the latest desired racket state.
    """

    def __init__(
        self,
        physics: BallPhysics | None = None,
        config: PlannerConfig | None = None,
        table: TableParams | None = None,
    ):
        self.physics = physics or BallPhysics()
        self.config = config or PlannerConfig()
        self.table = table or TableParams()

        self.estimator = BallStateEstimator(self.config)
        self.predictor = BallTrajectoryPredictor(self.physics, self.config, self.table)
        self.target_planner = RacketTargetPlanner(self.physics, self.config, self.table)

        self._latest_command: RacketCommand | None = None
        self._latest_strike: StrikeTarget | None = None

    def update(self, t: float, p_ball: np.ndarray) -> RacketCommand | None:
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
        self.estimator.push(t, p_ball)

        if not self.estimator.ready:
            return None

        p_est, v_est, t_est = self.estimator.estimate()

        # Only predict if ball is moving toward P1 (v_x < 0)
        if v_est[0] >= 0:
            self._latest_command = None
            return None

        strike = self.predictor.predict(p_est, v_est, t_est)
        self._latest_strike = strike

        command = self.target_planner.plan(strike)
        self._latest_command = command
        return command

    @property
    def racket_command(self) -> RacketCommand | None:
        return self._latest_command

    @property
    def time_to_strike(self) -> float | None:
        if self._latest_command is None or not self._latest_command.valid:
            return None
        if self._latest_strike is None:
            return None
        return self._latest_strike.t_strike
```

---

## 7  ROS 2 Node Integration

The planner runs as a ROS 2 node subscribing to `motion_capture_tracking` output (see HOPE Motion Capture System Reference Setup, Section 6).

### 7.1  Topics

| Topic | Type | Direction | Rate |
|-------|------|-----------|------|
| `/poses` | `geometry_msgs/PoseArray` | Subscribe | 360 Hz |
| `/racket/command` | Custom `RacketCommand.msg` | Publish | 360 Hz |
| `/planner/diagnostics` | `diagnostic_msgs/DiagnosticArray` | Publish | 10 Hz |

### 7.2  Custom Message Definition

```
# msg/RacketCommand.msg
std_msgs/Header header
geometry_msgs/Point position            # interception position (HOPE frame)
geometry_msgs/Vector3 velocity          # desired racket velocity (m/s)
geometry_msgs/Vector3 normal            # racket face normal (unit vector)
float64 strike_time                     # predicted strike time (s)
float64 time_to_strike                  # seconds remaining
geometry_msgs/Vector3 ball_velocity_outgoing
bool valid
bool clears_net
bool bypasses_net_posts
int32 predicted_bounces
```

### 7.3  Node Skeleton

```python
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseArray, PoseStamped
import numpy as np


class HOPEPlannerNode(Node):
    """ROS 2 node wrapping the HOPE 7DOF racket planner.

    Subscribes to ball position from motion_capture_tracking and
    publishes desired racket state for the whole-body controller.

    Per HOPE rules, the racket pose is never measured by motion capture.
    The humanoid must achieve the commanded racket state through its
    own forward kinematics.
    """

    def __init__(self):
        super().__init__("hope_planner")

        self.declare_parameter("ball_rigid_body_name", "pingpong_ball")
        self.declare_parameter("x_hit", 0.0)
        self.declare_parameter("target_land_x", 2.055)
        self.declare_parameter("target_land_y", -0.7625)
        self.declare_parameter("delta_t_flight", 0.5)
        self.declare_parameter("drag_k", 0.5)
        self.declare_parameter("restitution_h", 0.75)
        self.declare_parameter("restitution_v", 0.85)
        self.declare_parameter("restitution_racket", 0.88)

        config = PlannerConfig(
            x_hit=self.get_parameter("x_hit").value,
            target_land=np.array([
                self.get_parameter("target_land_x").value,
                self.get_parameter("target_land_y").value,
                0.0,
            ]),
            delta_t_flight=self.get_parameter("delta_t_flight").value,
            C_r=self.get_parameter("restitution_racket").value,
        )
        physics = BallPhysics(
            k=self.get_parameter("drag_k").value,
            C_h=self.get_parameter("restitution_h").value,
            C_v=self.get_parameter("restitution_v").value,
        )

        self.planner = HOPEPlanner(physics=physics, config=config)

        mocap_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=1,
        )
        self.create_subscription(PoseArray, "/poses", self._poses_cb, mocap_qos)
        self.cmd_pub = self.create_publisher(PoseStamped, "/racket/command", mocap_qos)

        self.get_logger().info(
            f"HOPE planner started — x_hit={config.x_hit:.2f}, "
            f"target={config.target_land}"
        )

    def _poses_cb(self, msg: PoseArray):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if len(msg.poses) == 0:
            return

        # TODO: filter by ball_rigid_body_name from motion_capture_tracking
        pose = msg.poses[0]
        p_ball = np.array([pose.position.x, pose.position.y, pose.position.z])

        cmd = self.planner.update(t, p_ball)
        if cmd is not None and cmd.valid:
            out = PoseStamped()
            out.header = msg.header
            out.header.frame_id = "world"
            out.pose.position.x = float(cmd.p_intercept[0])
            out.pose.position.y = float(cmd.p_intercept[1])
            out.pose.position.z = float(cmd.p_intercept[2])
            q = normal_to_quaternion(cmd.n_racket, constrain_up=True)
            out.pose.orientation.x = float(q[0])
            out.pose.orientation.y = float(q[1])
            out.pose.orientation.z = float(q[2])
            out.pose.orientation.w = float(q[3])
            self.cmd_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = HOPEPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

### 7.4  Launch Configuration

```yaml
# config/hope_planner.yaml
hope_planner:
  ros__parameters:
    ball_rigid_body_name: "pingpong_ball"
    x_hit: 0.0
    target_land_x: 2.055
    target_land_y: -0.7625
    delta_t_flight: 0.5
    drag_k: 0.5
    restitution_h: 0.75
    restitution_v: 0.85
    restitution_racket: 0.88
```

---

## 8  Racket Normal to Quaternion Conversion

The face normal **n̂** from Stage 3 defines the paddle tilt (2 DOF) but not the roll about the face axis. The `constrain_up` option resolves this by aligning the paddle handle with world −Z (handle pointing downward).

```python
def normal_to_quaternion(
    n_racket: np.ndarray,
    constrain_up: bool = False,
) -> np.ndarray:
    """Convert racket face normal to orientation quaternion [x, y, z, w]."""
    n = n_racket / np.linalg.norm(n_racket)
    ref = np.array([1.0, 0.0, 0.0])

    axis = np.cross(ref, n)
    sin_angle = np.linalg.norm(axis)
    cos_angle = np.dot(ref, n)

    if sin_angle < 1e-8:
        q = np.array([0.0, 0.0, 0.0, 1.0]) if cos_angle > 0 else np.array([0.0, 1.0, 0.0, 0.0])
    else:
        axis = axis / sin_angle
        half_angle = np.arctan2(sin_angle, cos_angle) / 2.0
        qw = np.cos(half_angle)
        qxyz = axis * np.sin(half_angle)
        q = np.array([qxyz[0], qxyz[1], qxyz[2], qw])

    if not constrain_up:
        return q

    # Constrain roll: align paddle handle (local Y) with world −Z
    def quat_to_matrix(q):
        x, y, z, w = q
        return np.array([
            [1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)],
            [2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)],
            [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)],
        ])

    def quat_multiply(q1, q2):
        x1,y1,z1,w1 = q1; x2,y2,z2,w2 = q2
        return np.array([
            w1*x2+x1*w2+y1*z2-z1*y2, w1*y2-x1*z2+y1*w2+z1*x2,
            w1*z2+x1*y2-y1*x2+z1*w2, w1*w2-x1*x2-y1*y2-z1*z2,
        ])

    R = quat_to_matrix(q)
    current_y = R @ np.array([0.0, 1.0, 0.0])
    down = np.array([0.0, 0.0, -1.0])
    desired_y = down - np.dot(down, n) * n
    desired_y_norm = np.linalg.norm(desired_y)
    if desired_y_norm < 1e-6:
        return q
    desired_y = desired_y / desired_y_norm

    cos_roll = np.clip(np.dot(current_y, desired_y), -1.0, 1.0)
    sin_roll = np.dot(np.cross(current_y, desired_y), n)
    roll_half = np.arctan2(sin_roll, cos_roll) / 2.0

    q_roll = np.array([
        n[0]*np.sin(roll_half), n[1]*np.sin(roll_half),
        n[2]*np.sin(roll_half), np.cos(roll_half),
    ])
    return quat_multiply(q_roll, q)
```

---

## 9  Summary of Data Flow

```
Motion Capture System (360 Hz)                         Humanoid (proprioceptive)
  │                                                      │
  ├── Ball [x,y,z] (single marker) ──▶ HOPE Planner     │
  │                                     Stages 1–3       │
  ├── PPT 6-DOF ──▶ origin validation     │              │
  │                                        ▼              │
  └── P1 base_link 6-DOF ──────────▶ WBC (Stage 4) ◀── RacketCommand
                                           │              (p_intercept,
                                           │               v_racket,
                                           ▼               n_racket,
                                     Joint commands        t_strike)
                                     (29 DOF, 50 Hz)
                                           │
                                           ▼
                                     Paddle pose
                                     (inferred via FK,
                                      NOT from mocap)
```

---

## 10  Known Limitations

1. **No spin model** — Spin-induced Magnus force is neglected, consistent with the HITTER paper. This is the largest source of prediction error for balls with heavy topspin or sidespin.

2. **Drag-free return trajectory** — Stage 3 uses pure ballistic motion (no drag) for the outgoing velocity calculation. For high-speed returns, the actual landing point will undershoot slightly.

3. **Fixed virtual hitting plane** — The hitting plane is at constant x. A more advanced planner would optimize the interception point over the robot's reachable workspace.

4. **Planar restitution model** — The diagonal bounce matrix assumes no spin transfer or tangential friction.

5. **Single landing target** — The planner always aims for the opponent's half-center. Strategic target variation is outside the scope of this reference.

6. **No source code cross-validation** — HITTER has not released source code. Implementation choices not specified in the paper (integrator, bounce handling, timing) may differ from the authors' actual system.

---

## References

- Su, Z., Zhang, B., Rahmanian, N., Gao, Y., Liao, Q., Regan, C., Sreenath, K., & Sastry, S. S. (2025). HITTER: A HumanoId Table TEnnis Robot via Hierarchical Planning and Learning. *arXiv:2508.21043v2*.
- HITTER project page: https://humanoid-table-tennis.github.io/
- motion_capture_tracking: https://github.com/IMRCLab/motion_capture_tracking
- Companion document: *HOPE Motion Capture System Reference Setup for Ping-Pong Arena, v0.2*
