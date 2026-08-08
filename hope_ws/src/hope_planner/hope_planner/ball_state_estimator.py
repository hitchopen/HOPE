"""Stage 1 - Ball state estimation.

Fits a 2nd-order polynomial to the most recent N position samples and
differentiates analytically to obtain a smoothed position and velocity.
The buffer is cleared on each detected table bounce so the polynomial
never fits across the velocity discontinuity.

See HOPE_7DOF_Racket_Model_based_Planner_Reference_Setup.md, Section 3.
"""

from typing import List, Optional, Tuple

import numpy as np

from .constants import PlannerConfig


class BallStateEstimator:
    """Estimate ball position and velocity from a position stream.

    Maintains a sliding window of recent position measurements and performs
    a least-squares polynomial fit to extract smoothed position and velocity.

    Bounce detection uses a three-sample pattern (descend -> contact -> rise)
    to identify the actual table impact event and clear the buffer.
    """

    def __init__(
        self,
        config: PlannerConfig,
        *,
        horizontal_poly_order: int | None = None,
    ):
        self.config = config
        self.horizontal_poly_order = (
            None
            if horizontal_poly_order is None
            else int(horizontal_poly_order)
        )
        if (
            self.horizontal_poly_order is not None
            and self.horizontal_poly_order not in (1, 2)
        ):
            raise ValueError("horizontal_poly_order must be 1 or 2")
        self.t_buffer: List[float] = []
        self.p_buffer: List[np.ndarray] = []

        # Bounce detection: three-sample z-height ring buffer.
        # Initialized to None to suppress false triggers before
        # enough measurements are collected.
        self._z_hist: List[Optional[float]] = [None, None, None]
        self._bounce_detected: bool = False

    def reset(self) -> None:
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

        # Bounce detection: three-sample pattern, two geometries (paper §IV-A: clear the buffer
        # on bounce so the polyfit never spans the velocity discontinuity).
        #   (a) LEGACY point-ball dip: z_prev at/below bounce_z_tol between two above-tol samples
        #       (sim harness feeds ideal point-ball z that reaches ~0 at contact).
        #   (b) CENTER geometry (real mocap, audit 2026-07-07): the rigid-body CENTER never goes
        #       below the ball RADIUS (0.02 m >> bounce_z_tol), so detect a LOCAL Z-MINIMUM below
        #       bounce_center_z_max — descending then rising with the dip in the contact band.
        self._bounce_detected = False
        z_pp, z_p, z_c = self._z_hist
        tol = self.config.bounce_z_tol
        center_max = getattr(self.config, "bounce_center_z_max", 0.05)
        if z_pp is not None and z_p is not None and z_c is not None:
            legacy_dip = z_pp > tol and z_p <= tol and z_c > tol
            center_min = z_p <= center_max and z_pp > z_p and z_c > z_p
            if legacy_dip or center_min:
                self._bounce_detected = True
                self.reset()

        self.t_buffer.append(t)
        self.p_buffer.append(p.copy())

        if len(self.t_buffer) > self.config.fit_window:
            self.t_buffer.pop(0)
            self.p_buffer.pop(0)

        # The mocap callback rate is not guaranteed to match the sensor rate: a BEST_EFFORT,
        # depth-1 subscription deliberately drops samples while the Python solve is busy. A
        # sample-count-only window can therefore stretch from the intended ~100 ms to multiple
        # seconds and fit one polynomial across a bounce. Keep the recipe's sample cap, but also
        # bound its elapsed time. At a severe rate drop this leaves <6 samples and the estimator
        # fails closed until a fresh, compact window is available.
        max_span = float(getattr(self.config, "fit_window_max_span_s", 0.0))
        if max_span > 0.0:
            cutoff = t - max_span
            while len(self.t_buffer) > 1 and self.t_buffer[0] < cutoff:
                self.t_buffer.pop(0)
                self.p_buffer.pop(0)

    @property
    def bounce_detected(self) -> bool:
        """True if the most recent push() detected a table bounce."""
        return self._bounce_detected

    @property
    def sample_count(self) -> int:
        """Number of samples in the active polynomial window."""
        return len(self.t_buffer)

    @property
    def sample_span_s(self) -> float:
        """Elapsed time represented by the active polynomial window."""
        if len(self.t_buffer) < 2:
            return 0.0
        return max(0.0, float(self.t_buffer[-1] - self.t_buffer[0]))

    @property
    def ready(self) -> bool:
        """True if enough samples exist for a reliable fit."""
        if len(self.t_buffer) < 6:
            return False
        min_span = float(getattr(self.config, "fit_window_min_span_s", 0.0))
        return min_span <= 0.0 or self.t_buffer[-1] - self.t_buffer[0] >= min_span

    def estimate(self) -> Tuple[np.ndarray, np.ndarray, float]:
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
            raise RuntimeError(f"Need >= 6 samples, have {len(self.t_buffer)}")

        t_arr = np.array(self.t_buffer)
        p_arr = np.array(self.p_buffer)

        # Normalize time to improve numerical conditioning
        t_ref = t_arr[-1]
        t_norm = t_arr - t_ref

        p_est = np.zeros(3)
        v_est = np.zeros(3)

        for axis in range(3):
            # For the deterministic serve selector, X/Y are fitted linearly:
            # over a compact ~100 ms flight window their physical curvature
            # is negligible, while a quadratic endpoint derivative strongly
            # amplifies millimetre-scale position noise.  The rally estimator
            # retains the legacy configured order unless its caller opts in.
            order = (
                self.horizontal_poly_order
                if axis < 2 and self.horizontal_poly_order is not None
                else self.config.poly_order
            )
            # Coefficients are descending; the final two are a1, a0 for
            # either the linear or quadratic case.
            coeffs = np.polyfit(t_norm, p_arr[:, axis], deg=order)
            p_est[axis] = coeffs[-1]   # a0 at t_norm = 0
            v_est[axis] = coeffs[-2]   # a1 at t_norm = 0

        return p_est, v_est, t_ref
