"""Shared core for the no-spin ball-physics fit.

Load a canonical ball trajectory, split it into free-flight arcs and contacts,
and fit the flight drag with an RK4 shooting fit. Everything is SI (metres,
seconds) in the world frame documented in ``configs/ball_physics.yaml``
(+x forward, +y left, +z up; z = 0 at the table surface).

Ball state is position only in the data (t, x, y, z); velocities are always
recovered by fitting, never by raw finite differences.
"""
import numpy as np
from scipy.signal import savgol_filter
from scipy.optimize import least_squares

R_BALL = 0.020          # m, ball radius (contact geometry)
G_NOM = 9.81            # m/s^2
SG_WIN, SG_POLY = 9, 2  # Savitzky-Golay smoothing window / order for velocities


# ------------------------------------------------------------------ loading --

def load_trajectory(path):
    """Load a canonical ``t,x,y,z`` CSV.

    Returns dict(t (n,), pos (n,3), rate, name). ``rate`` is the median sample
    rate (Hz). The file is one contiguous capture segment in SI units.
    """
    import os
    raw = np.genfromtxt(path, delimiter=",", names=True)
    t = np.asarray(raw["t"], float)
    pos = np.column_stack([raw["x"], raw["y"], raw["z"]]).astype(float)
    order = np.argsort(t)
    t, pos = t[order], pos[order]
    dt = np.median(np.diff(t)) if len(t) > 1 else 0.0
    return dict(t=t, pos=pos, rate=(1.0 / dt if dt > 0 else 0.0),
                name=os.path.splitext(os.path.basename(path))[0])


# ------------------------------------------------------- runs / velocities --

def contiguous_runs(t, rate, min_len=15, max_gap_factor=1.5):
    """Index ranges [a, b] inclusive with no time gap larger than
    ``max_gap_factor / rate`` (occlusion gaps split the capture into runs)."""
    if rate <= 0 or len(t) < min_len:
        return [(0, len(t) - 1)] if len(t) >= min_len else []
    gap = max_gap_factor / rate
    runs, s = [], 0
    for i in range(1, len(t)):
        if t[i] - t[i - 1] > gap:
            runs.append((s, i - 1)); s = i
    runs.append((s, len(t) - 1))
    return [(a, b) for a, b in runs if b - a + 1 >= min_len]


def smooth_vel(t, pos):
    """Smoothed positions and velocities (Savitzky-Golay). Falls back to a plain
    gradient on windows shorter than the filter."""
    if len(t) < SG_WIN:
        return pos.copy(), np.gradient(pos, t, axis=0)
    dt = np.median(np.diff(t))
    sp = savgol_filter(pos, SG_WIN, SG_POLY, axis=0)
    sv = savgol_filter(pos, SG_WIN, SG_POLY, deriv=1, axis=0) / dt
    return sp, sv


# ---------------------------------------------------------------- contacts --

def detect_contacts(t, pos, vel, rate, table_z=0.0, acc_thr=40.0, surf_band=0.045):
    """Contacts inside one run. Returns sorted [(idx, kind)] with kind in
    {'table', 'hit'}:
      * 'table' -- a local z-minimum with a vertical-velocity reversal while the
        ball centre is near the surface (table_z + R_BALL);
      * 'hit'   -- an acceleration spike away from the table (racket strike).

    The 'hit' test is a multiple-of-g acceleration threshold rather than a raw
    velocity delta, so it is robust to the capture rate (in free flight the
    acceleration stays ~g; an impact spikes far above it). ``acc_thr`` may need
    tuning for very noisy or very slow captures.
    """
    n = len(t)
    events = []
    zc = table_z + R_BALL
    # table bounces: z-minimum with a vertical-velocity reversal near the surface
    for i in range(2, n - 2):
        if (pos[i, 2] <= pos[i - 1, 2] and pos[i, 2] <= pos[i + 1, 2]
                and vel[i - 1, 2] < -0.25 and vel[i + 1, 2] > 0.25
                and abs(pos[i, 2] - zc) < surf_band):
            events.append((i, "table"))
    # racket strikes: local acceleration spike, away from the table and the
    # run boundaries (where finite differences are one-sided / unreliable)
    amag = np.linalg.norm(np.gradient(vel, t, axis=0), axis=1)
    for i in range(3, n - 3):
        if pos[i, 2] < zc + surf_band:
            continue                       # near the table -> handled as a bounce
        if amag[i] > acc_thr and amag[i] >= amag[i - 1] and amag[i] >= amag[i + 1]:
            events.append((i, "hit"))
    # merge neighbours (<= 5 frames apart); a 'table' label wins over a 'hit'
    events.sort()
    merged = []
    for idx, kind in events:
        if merged and idx - merged[-1][0] <= 5:
            if kind == "table":
                merged[-1] = (merged[-1][0], "table")
            continue
        merged.append((idx, kind))
    return merged


def extract_arcs(traj, min_frames=12, table_z=0.0):
    """Free-flight arcs: contiguous runs cut at contacts, with short exclusion
    windows around each contact. Returns a list of dicts with t, pos, and the
    adjacent contact kinds."""
    t_all, pos_all, rate = traj["t"], traj["pos"], traj["rate"]
    arcs = []
    for a, b in contiguous_runs(t_all, rate):
        sl = slice(a, b + 1)
        t, pos = t_all[sl], pos_all[sl]
        sp, sv = smooth_vel(t, pos)
        contacts = detect_contacts(t, sp, sv, rate, table_z)
        cuts = [0] + [i for i, _ in contacts] + [len(t)]
        kinds = [None] + [k for _, k in contacts] + [None]
        for j in range(len(cuts) - 1):
            i0, i1 = cuts[j], cuts[j + 1]
            pre_kind, post_kind = kinds[j], kinds[j + 1]
            lo = i0 + (2 if pre_kind == "table" else 3 if pre_kind == "hit" else 0)
            hi = i1 - (2 if post_kind == "table" else 3 if post_kind == "hit" else 0)
            if hi - lo < min_frames:
                continue
            arcs.append(dict(
                name=traj["name"], i0=a + lo, i1=a + hi,
                t=t[lo:hi], pos=pos[lo:hi],
                pre_contact=pre_kind, post_contact=post_kind))
    return arcs


# ----------------------------------------------------------- arc parabola --

def arc_parabola(arc):
    """No-drag parabola fit. Returns g (from the z channel), rms (m) and v0."""
    t = arc["t"] - arc["t"][0]
    P = arc["pos"]
    A = np.vstack([np.ones_like(t), t, t * t]).T
    cx, *_ = np.linalg.lstsq(A, P[:, 0], rcond=None)
    cy, *_ = np.linalg.lstsq(A, P[:, 1], rcond=None)
    cz, *_ = np.linalg.lstsq(A, P[:, 2], rcond=None)
    res = P - A @ np.column_stack([cx, cy, cz])
    return dict(g=-2 * cz[2], rms=float(np.sqrt((res ** 2).sum(1).mean())),
                v0=np.array([cx[1], cy[1], cz[1]]))


def arc_is_ballistic(arc, par=None, max_rms=0.020, g_lo=6.0, g_hi=13.0,
                     min_dur=0.15, max_dur=1.5):
    """A free-flight arc is 'ballistic' if a plain parabola fits it well and its
    apparent gravity is physically sensible."""
    par = par or arc_parabola(arc)
    dur = arc["t"][-1] - arc["t"][0]
    return bool(min_dur <= dur <= max_dur and par["rms"] < max_rms
                and g_lo <= par["g"] <= g_hi)


# --------------------------------------------------- RK4 drag shooting fit --

def rk4_flight(p0, v0, kd, g_vec, dt, n_steps):
    """Vectorised RK4 for quadratic-drag flight:  a = g - kd |v| v.
    p0, v0 are (N, 3); returns positions (N, n_steps+1, 3)."""
    N = p0.shape[0]
    P = np.empty((N, n_steps + 1, 3))
    p, v = p0.copy(), v0.copy()
    P[:, 0] = p

    # A ping-pong ball never exceeds a few tens of m/s. Clipping the speed keeps
    # the integrator finite if the optimiser probes an unphysical k_d, so a bad
    # trial yields a large-but-finite residual instead of an overflow.
    VMAX = 100.0

    def acc(v):
        s = np.clip(np.linalg.norm(v, axis=1, keepdims=True), 0.0, VMAX)
        return g_vec - kd * s * v

    for k in range(n_steps):
        a1 = acc(v)
        v2 = v + 0.5 * dt * a1; a2 = acc(v2)
        v3 = v + 0.5 * dt * a2; a3 = acc(v3)
        v4 = v + dt * a3; a4 = acc(v4)
        p = p + (dt / 6.0) * (v + 2 * v2 + 2 * v3 + v4)
        v = np.clip(v + (dt / 6.0) * (a1 + 2 * a2 + 2 * a3 + a4), -VMAX, VMAX)
        P[:, k + 1] = p
    return P


def _shoot_residuals(arcs, per_arc, kd, g_vec):
    """Predicted - observed positions across all arcs (one padded RK4; assumes a
    common dt). per_arc is (N, 6) of (p0, v0)."""
    N = len(arcs)
    lens = np.array([len(a["t"]) for a in arcs])
    dt = float(np.median(np.diff(arcs[0]["t"])))
    P = rk4_flight(per_arc[:, :3], per_arc[:, 3:], kd, g_vec, dt, int(lens.max()) - 1)
    res = [(P[i, :lens[i]] - arcs[i]["pos"]).ravel() for i in range(N)]
    return np.nan_to_num(np.concatenate(res), nan=1e3, posinf=1e3, neginf=-1e3)


def fit_arcs_global(arcs, fit=("kd",), kd0=0.12, g0=None, huber_delta=0.003):
    """Joint drag shooting fit. Per-arc (p0, v0) are always free; ``kd`` (and
    optionally ``g``) are shared across all arcs. ``g`` is frozen to
    (0, 0, -9.81) unless 'g' is in ``fit``. Returns the fitted values and the
    per-arc position RMS (m)."""
    N = len(arcs)
    per0 = np.array([np.concatenate([a["pos"][0], arc_parabola(a)["v0"]]) for a in arcs])
    g_fixed = np.array([0.0, 0.0, -G_NOM]) if g0 is None else np.asarray(g0, float)

    def unpack(x):
        per = x[:6 * N].reshape(N, 6)
        i = 6 * N
        kd = x[i] if "kd" in fit else kd0
        i += "kd" in fit
        g = x[i:i + 3] if "g" in fit else g_fixed
        return per, kd, g

    def resid(x):
        per, kd, g = unpack(x)
        return _shoot_residuals(arcs, per, kd, g)

    x0 = [per0.ravel()]
    if "kd" in fit:
        x0.append([kd0])
    if "g" in fit:
        x0.append(g_fixed)
    x0 = np.concatenate(x0)

    if N == 1:
        sol = least_squares(resid, x0, loss="huber", f_scale=huber_delta,
                            x_scale="jac", max_nfev=200,
                            ftol=1e-6, xtol=1e-6, gtol=1e-6)
    else:
        from scipy.sparse import lil_matrix
        n_res = sum(3 * len(a["t"]) for a in arcs)
        S = lil_matrix((n_res, len(x0)), dtype=np.uint8)
        r0 = 0
        for i, a in enumerate(arcs):
            nr = 3 * len(a["t"])
            S[r0:r0 + nr, 6 * i:6 * i + 6] = 1
            S[r0:r0 + nr, 6 * N:] = 1
            r0 += nr
        sol = least_squares(resid, x0, jac_sparsity=S, loss="huber",
                            f_scale=huber_delta, x_scale="jac", max_nfev=150,
                            tr_solver="lsmr", ftol=1e-6, xtol=1e-6, gtol=1e-6)

    per, kd, g = unpack(sol.x)
    r = _shoot_residuals(arcs, per, kd, g)
    rms_per, r0 = [], 0
    for a in arcs:
        nr = 3 * len(a["t"])
        rms_per.append(float(np.sqrt((r[r0:r0 + nr].reshape(-1, 3) ** 2).sum(1).mean())))
        r0 += nr
    return dict(per_arc=per, kd=float(kd), g_vec=np.asarray(g, float),
                rms_per_arc=np.array(rms_per), cost=float(sol.cost),
                success=bool(sol.success))
