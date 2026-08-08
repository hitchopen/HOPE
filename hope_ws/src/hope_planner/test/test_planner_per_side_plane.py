"""Per-side hit planes (2026-07-13, v13 facefix): HOPEPlanner.repredict_at_plane.

The v13 clips strike on DIFFERENT station-relative planes (fh x 0.65 / bh x 0.50).
node.py solves at the forehand plane, selects the side from that intercept, and
re-predicts at the backhand plane when the backhand is selected. This test drives
the planner with an exact fake_ball_publisher flight (300 Hz semi-implicit Euler,
a = -k|v|v + g, floor bounce 0.85) from the G3 v13 serve sweep and checks the
re-predict lands on the bh plane at the v13 receipt contact height.
"""

import numpy as np
import pytest

from hope_planner.planner import HOPEPlanner
from hope_planner.constants import BallPhysics, PlannerConfig, TableParams

X_HIT_FH = 0.98          # G3: station 0.33 + v13 forehand plane 0.65
X_HIT_BH_DELTA = -0.15   # v13 backhand plane 0.50
BH_CONTACT_Z = 1.050     # v13 receipt measured_contact z (backhand)


def _drive_planner(planner, vy, vz):
    """Feed one fake serve at 300 Hz; return the last command."""
    dt = 1.0 / 300.0
    p = np.array([3.2, 0.12, 0.5])
    v = np.array([-1.4, vy, vz])
    g = np.array([0.0, 0.0, -9.81])
    t, cmd = 0.0, None
    while p[0] > 1.2:
        a = -0.05 * np.linalg.norm(v) * v + g
        v = v + a * dt
        p = p + v * dt
        t += dt
        if p[2] <= 0.0 and v[2] < 0.0:
            p[2] = 0.001
            v[0] *= 0.85
            v[1] *= 0.85
            v[2] = -0.85 * v[2]
        cmd = planner.update(t, p.copy())
    return cmd


@pytest.fixture()
def planner():
    cfg = PlannerConfig(
        x_hit=X_HIT_FH, target_land=np.array([2.2, -0.5, 0.0]), delta_t_flight=0.40
    )
    return HOPEPlanner(
        physics=BallPhysics(k=0.05, C_h=0.85, C_v=0.85),
        config=cfg,
        table=TableParams(y_max=0.7825),
    )


def test_bh_repredict_lands_on_bh_plane(planner):
    # G3 v13 serve 4 (backhand: vy -0.0532, vz 6.467 — solved for z 1.050 @ x 0.83).
    cmd = _drive_planner(planner, -0.0532, 6.467)
    assert cmd is not None and planner.strike_target is not None
    assert planner.strike_target.valid
    # Side decision at the fh plane (robot/station y = 0): rel_y > split -0.25 -> backhand.
    assert float(planner.strike_target.p_ball[1]) - 0.0 > -0.25
    tts_fh = planner.time_to_strike

    cmd2 = planner.repredict_at_plane(X_HIT_FH + X_HIT_BH_DELTA)
    assert cmd2 is not None and cmd2.valid
    assert cmd2.p_intercept[0] == pytest.approx(X_HIT_FH + X_HIT_BH_DELTA, abs=1e-9)
    # The bh plane is NEARER the robot -> crossed later -> tts grows.
    assert planner.time_to_strike > tts_fh
    # Arrival height = the v13 receipt bh contact the sweep was solved for.
    assert cmd2.p_intercept[2] == pytest.approx(BH_CONTACT_Z, abs=0.03)


def test_repredict_without_estimate_is_a_noop(planner):
    assert planner.repredict_at_plane(0.83) is None  # no samples yet -> latest command (None)


def test_forehand_stays_on_fh_plane(planner):
    # G3 v13 serve 1 (forehand: vy -0.3532, vz 5.813 — solved for z 0.982 @ x 0.98).
    cmd = _drive_planner(planner, -0.3532, 5.813)
    assert cmd is not None and cmd.valid
    assert float(planner.strike_target.p_ball[1]) - 0.0 < -0.25  # decisively forehand
    assert cmd.p_intercept[0] == pytest.approx(X_HIT_FH, abs=1e-9)
    assert cmd.p_intercept[2] == pytest.approx(0.982, abs=0.03)
