"""Ball spin estimation from mocap quaternions.

The venue rig publishes a ball orientation quaternion whose per-event spin
noise floor is ~2 rev/s (configs/ball_physics_venue.yaml
`capture.spin_noise.floor_rev_s`), while venue-median amateur spin sits right
AT that floor. Feeding a noise-floor omega into the Magnus term would be worse
than assuming zero spin, so this estimator is deliberately gated: it returns
an omega only when the windowed estimate clears gate_rev_s (default 3 rev/s,
above the floor) and None otherwise — callers then fall back to omega = 0,
which reproduces the legacy no-Magnus behavior exactly.

Angular velocity is recovered by chaining per-frame incremental rotation
vectors over a sliding window rather than differencing the window endpoints:
at 15 rev/s a 100 ms endpoint difference spans 1.5 revolutions and aliases,
whereas each 300 Hz frame step is only ~0.05 rev and stays unambiguous. The
quaternion algebra mirrors quaternion_utils (which is xyzw; the mocap spin
channel is wxyz, so the helpers here are wxyz).
"""

from typing import List, Optional, Tuple

import numpy as np

TWO_PI = 2.0 * np.pi


def _quat_multiply_wxyz(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    """Hamilton product, [w, x, y, z] convention (mirror of quaternion_utils)."""
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ])


def _quat_conjugate_wxyz(q: np.ndarray) -> np.ndarray:
    return np.array([q[0], -q[1], -q[2], -q[3]])


def _rotation_vector_wxyz(q: np.ndarray) -> np.ndarray:
    """Axis * angle (rad) of a unit quaternion, shortest arc."""
    q = q / np.linalg.norm(q)
    if q[0] < 0.0:  # double cover: keep angle in [0, pi]
        q = -q
    vec = q[1:4]
    sin_half = np.linalg.norm(vec)
    if sin_half < 1e-12:
        return np.zeros(3)
    angle = 2.0 * np.arctan2(sin_half, q[0])
    return (angle / sin_half) * vec


class SpinFromQuats:
    """Windowed finite-difference spin from (t, quaternion wxyz) samples."""

    def __init__(
        self,
        window_s: float = 0.10,
        gate_rev_s: float = 3.0,
        max_gap_s: float = 0.05,
        max_rev_s: float = 20.0,
    ):
        self.window_s = float(window_s)
        self.gate_rad_s = float(gate_rev_s) * TWO_PI
        self.max_gap_s = float(max_gap_s)
        self.max_rad_s = float(max_rev_s) * TWO_PI
        if (
            self.window_s <= 0.0
            or self.gate_rad_s < 0.0
            or self.max_gap_s <= 0.0
            or self.max_rad_s <= self.gate_rad_s
        ):
            raise ValueError("invalid spin estimator window/gate/gap/range")
        # Buffer of (t, world-frame incremental rotation vector since the
        # previous sample). The first sample carries a zero increment.
        self._buffer: List[Tuple[float, np.ndarray]] = []
        self._last_q: Optional[np.ndarray] = None
        self._last_t: Optional[float] = None

    def reset(self) -> None:
        self._buffer.clear()
        self._last_q = None
        self._last_t = None

    def push(self, t: float, quat_wxyz: np.ndarray) -> Optional[np.ndarray]:
        """Ingest one orientation sample; return omega (rad/s) or None.

        None means "no spin estimate above the noise gate" — callers should
        use omega = 0 (legacy behavior), NOT hold the last value.
        """
        q = np.asarray(quat_wxyz, dtype=float)
        q_norm = float(np.linalg.norm(q))
        if (
            q.shape != (4,)
            or not np.isfinite(t)
            or not np.all(np.isfinite(q))
            or q_norm < 1.0e-9
        ):
            self.reset()
            return None
        q = q / q_norm

        if self._last_q is None:
            self._last_q = q
            self._last_t = t
            self._buffer.append((t, np.zeros(3)))
            return None

        if t <= self._last_t:
            return self.omega(t)  # duplicate/out-of-order stamp: ignore sample
        dt = t - self._last_t
        if dt > self.max_gap_s:
            # Occlusion/reacquisition: never bridge the unknown rotation gap.
            self.reset()
            self._last_q = q
            self._last_t = t
            self._buffer.append((t, np.zeros(3)))
            return None

        # World-frame delta rotation: q_k = q_rel * q_{k-1}.
        q_prev = self._last_q
        if float(np.dot(q, q_prev)) < 0.0:  # sign continuity (double cover)
            q = -q
        q_rel = _quat_multiply_wxyz(q, _quat_conjugate_wxyz(q_prev))
        rotation_vector = _rotation_vector_wxyz(q_rel)
        if np.linalg.norm(rotation_vector) / dt > self.max_rad_s:
            # A Motive rigid-body relock/marker permutation can look like an
            # impossible one-frame spin impulse. Start a new window.
            self.reset()
            self._last_q = q
            self._last_t = t
            self._buffer.append((t, np.zeros(3)))
            return None
        self._buffer.append((t, rotation_vector))
        self._last_q = q
        self._last_t = t

        # Trim to the window (keep one sample at/just before the window edge
        # so the time span is at least window_s once enough data exists).
        t_min = t - self.window_s
        while len(self._buffer) > 2 and self._buffer[1][0] <= t_min:
            self._buffer.pop(0)

        return self.omega(t)

    def omega(self, now: Optional[float] = None) -> Optional[np.ndarray]:
        """Latest gated spin estimate (rad/s, world frame), or None."""
        if (
            now is not None
            and self._last_t is not None
            and now - self._last_t > self.max_gap_s
        ):
            return None
        if len(self._buffer) < 3:
            return None
        span = self._buffer[-1][0] - self._buffer[0][0]
        if span < 0.5 * self.window_s:
            return None  # baseline too short for a meaningful estimate
        total = np.zeros(3)
        for _, dr in self._buffer[1:]:
            total += dr
        w = total / span
        if np.linalg.norm(w) <= self.gate_rad_s:
            return None
        return w

    @property
    def last_timestamp(self) -> Optional[float]:
        return self._last_t
