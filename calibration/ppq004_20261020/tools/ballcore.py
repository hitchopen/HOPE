"""
Shared core for the venue ball-physics fit: take loading, run/arc segmentation,
spin from quaternions, RK4 shooting fits.

All SI (m, s, rad). Table frame: X = length, Y = width, Z = up, origin = table center.
"""
import os, glob, json
import numpy as np
import paths
from scipy.signal import savgol_filter, medfilt
from scipy.spatial.transform import Rotation as Rot
from scipy.optimize import least_squares

R_BALL = 0.020
G_NOM = 9.81
SG_WIN, SG_POLY = 9, 2

EXTRACTED = paths.EXTRACTED

TAKES = {  # canonical ascii name -> npz stem
    "shangxuan": "shangxuan_000",     # topspin strikes
    "xiaxuan": "xiaxuan_000",         # backspin strikes
    "cexuan": "cexuan_000",           # sidespin strikes
    "tantiao": "TANTIAO_000",         # bounce trials
    "kuaisu": "快速",                  # fast strikes
    "zhengchang": "ZHENGCHANG_000",   # normal rally
    "dianqiu_nospin": "buzhuandianqiu_000",  # no-spin juggling
    "dianqiu_spin": "dainqiu_000",    # spin juggling
    "gaoqiu": "GAO_QIU_000",          # high lobs
}


def load_take(name):
    d = dict(np.load(os.path.join(EXTRACTED, f"{TAKES[name]}.npz"), allow_pickle=False))
    d["name"] = name
    return d


def tracked_runs(t, present, rate, min_len=15):
    """Contiguous tracked runs [a,b] inclusive (indices into the take arrays)."""
    runs, s = [], None
    for i, p in enumerate(present):
        if p and s is None:
            s = i
        if not p and s is not None:
            runs.append((s, i - 1)); s = None
    if s is not None:
        runs.append((s, len(present) - 1))
    return [(a, b) for a, b in runs if b - a + 1 >= min_len]


def smooth_vel(t, pos):
    n = len(t)
    dt = np.median(np.diff(t))
    if n < SG_WIN:
        return pos.copy(), np.gradient(pos, t, axis=0)
    sp = savgol_filter(pos, SG_WIN, SG_POLY, axis=0)
    sv = savgol_filter(pos, SG_WIN, SG_POLY, deriv=1, axis=0) / dt
    return sp, sv


def detect_contacts(t, pos, vel, rate, table_z=0.0, acc_thr=60.0, surf_band=0.045):
    """Contact frames inside one tracked run.
    Returns sorted list of (idx, kind) with kind in {'table','hit'}.
    table: local z-min with vz reversal, ball-center near table_z + R.
    hit  : |dv| between +-2-frame windows > 1 m/s in <10ms away from the table."""
    n = len(t)
    dt = 1.0 / rate
    events = []
    zc = table_z + R_BALL
    for i in range(2, n - 2):
        if (pos[i, 2] <= pos[i - 1, 2] and pos[i, 2] <= pos[i + 1, 2]
                and vel[i - 1, 2] < -0.25 and vel[i + 1, 2] > 0.25
                and abs(pos[i, 2] - zc) < surf_band):
            events.append((i, "table"))
    acc = np.gradient(vel, dt, axis=0)
    amag = np.linalg.norm(acc + np.array([0, 0, 0]), axis=1)
    for i in range(3, n - 3):
        if pos[i, 2] < zc + surf_band:
            continue
        dv = vel[i + 2] - vel[i - 2]
        if np.linalg.norm(dv) > 1.0 and amag[i] > acc_thr:
            events.append((i, "hit"))
    # merge neighbors (<= 5 frames apart), keep the max-accel frame
    events.sort()
    merged = []
    for idx, kind in events:
        if merged and idx - merged[-1][0] <= 5:
            if kind == "table":  # table label wins (more specific)
                merged[-1] = (merged[-1][0], "table")
            continue
        merged.append((idx, kind))
    return merged


def extract_arcs(take, min_frames=20, table_z=0.0):
    """Free-flight arcs: tracked runs cut at contacts, exclusion windows applied.
    Returns list of dicts with t, pos (table frame), quat, i0 (take index), contacts."""
    t_all = take["t"]
    pos_all = take["ball_pos_t_m"].astype(float)
    quat_all = take["ball_quat_xyzw"].astype(float)
    present = take["ball_present"].astype(bool)
    rate = float(take["rate"])
    arcs = []
    for a, b in tracked_runs(t_all, present, rate):
        sl = slice(a, b + 1)
        t, pos, quat = t_all[sl], pos_all[sl], quat_all[sl]
        sp, sv = smooth_vel(t, pos)
        contacts = detect_contacts(t, sp, sv, rate, table_z)
        cuts = [0] + [i for i, _ in contacts] + [len(t)]
        kinds = [None] + [k for _, k in contacts] + [None]
        for j in range(len(cuts) - 1):
            i0, i1 = cuts[j], cuts[j + 1]
            # exclusion windows around contacts (frames)
            pre_kind, post_kind = kinds[j], kinds[j + 1]
            lo = i0 + (3 if pre_kind == "table" else 5 if pre_kind == "hit" else 0)
            hi = i1 - (3 if post_kind == "table" else 5 if post_kind == "hit" else 0)
            if hi - lo < min_frames:
                continue
            arcs.append(dict(
                take=take["name"], i0=a + lo, i1=a + hi,
                t=t[lo:hi], pos=pos[lo:hi], quat=quat[lo:hi],
                pre_contact=pre_kind, post_contact=post_kind,
                contact_pre_idx=a + i0 if pre_kind else None,
                contact_post_idx=a + i1 if post_kind else None))
    return arcs


def arc_parabola(arc):
    """No-drag parabola fit. Returns g (z channel), rms (m), v0."""
    t = arc["t"] - arc["t"][0]
    P = arc["pos"]
    A = np.vstack([np.ones_like(t), t, t * t]).T
    cz, *_ = np.linalg.lstsq(A, P[:, 2], rcond=None)
    cx, *_ = np.linalg.lstsq(A, P[:, 0], rcond=None)
    cy, *_ = np.linalg.lstsq(A, P[:, 1], rcond=None)
    res = P - A @ np.column_stack([cx, cy, cz])
    return dict(g=-2 * cz[2], rms=float(np.sqrt((res ** 2).sum(1).mean())),
                v0=np.array([cx[1], cy[1], cz[1]]),
                a_xy=2 * np.array([cx[2], cy[2]]))


def arc_is_ballistic(arc, par=None, max_rms=0.012, g_lo=7.0, g_hi=13.0,
                     min_dur=0.15, max_dur=1.5):
    par = par or arc_parabola(arc)
    dur = arc["t"][-1] - arc["t"][0]
    return (min_dur <= dur <= max_dur and par["rms"] < max_rms
            and g_lo <= par["g"] <= g_hi)


def spin_from_quats(quat, rate, reliable_deg=90.0, R_table=None):
    """Angular velocity (rad/s) from consecutive orientations. Quats are Kabsch
    template->WORLD, so w is a WORLD vector; pass R_table (cols = table axes in
    world) to get w expressed in the TABLE frame (matching the position arrays).
    Returns w (n-1,3), deg/frame (n-1), robust (per-component median) over
    reliable frames."""
    ok = np.isfinite(quat).all(1)
    w = np.full((len(quat) - 1, 3), np.nan)
    ang = np.full(len(quat) - 1, np.nan)
    idx = np.where(ok[:-1] & ok[1:])[0]
    if len(idx):
        Ra = Rot.from_quat(quat[idx])
        Rb = Rot.from_quat(quat[idx + 1])
        rv = (Rb * Ra.inv()).as_rotvec()
        w[idx] = rv * rate
        ang[idx] = np.degrees(np.linalg.norm(rv, axis=1))
    if R_table is not None:
        w = w @ R_table            # row-vector form of R_table.T @ w
    rel = ang < reliable_deg
    if rel.sum() >= 5:
        w_rob = np.nanmedian(w[rel], axis=0)
    else:
        w_rob = np.full(3, np.nan)
    return w, ang, w_rob


# ---------------- RK4 shooting (vectorized across arcs) ----------------

def rk4_flight(p0, v0, w, kd, km, g_vec, dt, n_steps):
    """Vectorized RK4: p0,v0,w (N,3); returns positions (N, n_steps+1, 3).
    a = g - kd|v|v + km (w x v); w constant."""
    N = p0.shape[0]
    P = np.empty((N, n_steps + 1, 3))
    p, v = p0.copy(), v0.copy()
    P[:, 0] = p

    def acc(v):
        s = np.linalg.norm(v, axis=1, keepdims=True)
        return g_vec - kd * s * v + km * np.cross(w, v)

    for k in range(n_steps):
        a1 = acc(v)
        v2 = v + 0.5 * dt * a1; a2 = acc(v2)
        v3 = v + 0.5 * dt * a2; a3 = acc(v3)
        v4 = v + dt * a3; a4 = acc(v4)
        p = p + (dt / 6.0) * (v + 2 * v2 + 2 * v3 + v4)
        v = v + (dt / 6.0) * (a1 + 2 * a2 + 2 * a3 + a4)
        P[:, k + 1] = p
    return P


def shoot_residuals(arcs, per_arc, kd, km, g_vec, w_per_arc=None):
    """Residual vector: predicted - observed positions for all arcs, computed with
    ONE vectorized RK4 over all arcs (padded to the longest; assumes common dt).
    per_arc: (N,6) p0,v0. w_per_arc: (N,3) or None (zero spin)."""
    N = len(arcs)
    lens = np.array([len(a["t"]) for a in arcs])
    dt = float(np.median(np.diff(arcs[0]["t"])))
    w = w_per_arc if w_per_arc is not None else np.zeros((N, 3))
    P = rk4_flight(per_arc[:, :3], per_arc[:, 3:], w, kd, km, g_vec,
                   dt, int(lens.max()) - 1)
    res = [(P[i, :lens[i]] - arcs[i]["pos"]).ravel() for i in range(N)]
    # diverged integrations (wild trial params) -> large finite residual, not NaN
    return np.nan_to_num(np.concatenate(res), nan=1e3, posinf=1e3, neginf=-1e3)


def fit_arcs_global(arcs, fit=("kd",), kd0=0.12, km0=0.0042, g0=None,
                    w_per_arc=None, huber_delta=0.002, verbose=False):
    """Joint shooting fit: per-arc (p0,v0) always free; global params per `fit`
    subset of {kd, km, g}. g frozen to [0,0,-9.81] unless 'g' in fit.
    Returns dict with fitted values and per-arc rms."""
    N = len(arcs)
    per0 = np.array([np.concatenate([a["pos"][0], arc_parabola(a)["v0"]]) for a in arcs])
    g_fixed = np.array([0.0, 0.0, -G_NOM]) if g0 is None else np.asarray(g0, float)

    def unpack(x):
        per = x[:6 * N].reshape(N, 6)
        i = 6 * N
        kd = x[i] if "kd" in fit else kd0
        i += "kd" in fit
        km = x[i] if "km" in fit else km0
        i += "km" in fit
        g = x[i:i + 3] if "g" in fit else g_fixed
        return per, kd, km, g

    def resid(x):
        per, kd, km, g = unpack(x)
        r = shoot_residuals(arcs, per, kd, km, g, w_per_arc)
        # Huber via soft_l1 handled by least_squares loss; return raw
        return r

    x0 = [per0.ravel()]
    if "kd" in fit: x0.append([kd0])
    if "km" in fit: x0.append([km0])
    if "g" in fit: x0.append(g_fixed)
    x0 = np.concatenate(x0)

    if N == 1:
        # dense path: 7-10 params, sparse machinery is pure overhead here
        sol = least_squares(resid, x0, loss="huber", f_scale=huber_delta,
                            x_scale="jac", max_nfev=200,
                            ftol=1e-6, xtol=1e-6, gtol=1e-6,
                            verbose=2 if verbose else 0)
    else:
        # sparsity: per-arc params only hit their own residuals
        from scipy.sparse import lil_matrix
        n_res = sum(3 * len(a["t"]) for a in arcs)
        n_par = len(x0)
        S = lil_matrix((n_res, n_par), dtype=np.uint8)
        r0 = 0
        for i, a in enumerate(arcs):
            nr = 3 * len(a["t"])
            S[r0:r0 + nr, 6 * i:6 * i + 6] = 1
            S[r0:r0 + nr, 6 * N:] = 1
            r0 += nr
        sol = least_squares(resid, x0, jac_sparsity=S, loss="huber", f_scale=huber_delta,
                            x_scale="jac", max_nfev=150, tr_solver="lsmr",
                            ftol=1e-6, xtol=1e-6, gtol=1e-6,
                            verbose=2 if verbose else 0)
    per, kd, km, g = unpack(sol.x)
    r = shoot_residuals(arcs, per, kd, km, g, w_per_arc)
    rms_per = []
    r0 = 0
    for a in arcs:
        nr = 3 * len(a["t"])
        rr = r[r0:r0 + nr].reshape(-1, 3)
        rms_per.append(float(np.sqrt((rr ** 2).sum(1).mean())))
        r0 += nr
    return dict(per_arc=per, kd=float(kd), km=float(km), g_vec=np.asarray(g, float),
                rms_per_arc=np.array(rms_per), cost=float(sol.cost),
                success=bool(sol.success))
