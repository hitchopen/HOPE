"""
Stage 1 — segmentation: flights, table bounces, racket strikes, with pre/post
contact states estimated by windowed shooting fits (never finite-difference).

Outputs (analysis/segments/):
  flights.json  : one row per ballistic arc (t0, dur, v0, spin vector, take, quality)
  bounces.json  : per table bounce: v-, v+, w-, w+, e_n, contact z, incidence
  strikes.json  : per racket strike: ball v-/w-, ball v+/w+, racket pose/vel/normal
                  at contact (occlusion-bridged), which paddle, u_n, u_t
Conventions: table frame (X length, Y width, Z up), SI. Spin = rad/s world(table) frame.
"""
import os, sys, json
import numpy as np
import paths
from scipy.spatial.transform import Rotation as Rot

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from ballcore import (TAKES, load_take, extract_arcs, arc_parabola, arc_is_ballistic,
                      spin_from_quats, R_BALL, G_NOM, smooth_vel, tracked_runs)

OUT = paths.SEGMENTS
KD_NOMINAL = 0.12          # only used inside short-window state estimation
KM_NOMINAL = 0.0042
SURFACE_Z = None           # set from qa_stage0.json at runtime


def rk4_window(p0, v0, w, kd, km, g, dt, n):
    """Single-trajectory RK4, returns positions (n+1,3) and velocities."""
    P = np.empty((n + 1, 3)); V = np.empty((n + 1, 3))
    p, v = p0.astype(float).copy(), v0.astype(float).copy()
    P[0], V[0] = p, v

    def acc(v):
        return g - kd * np.linalg.norm(v) * v + km * np.cross(w, v)

    for k in range(n):
        a1 = acc(v)
        v2 = v + 0.5 * dt * a1; a2 = acc(v2)
        v3 = v + 0.5 * dt * a2; a3 = acc(v3)
        v4 = v + dt * a3; a4 = acc(v4)
        p = p + (dt / 6.0) * (v + 2 * v2 + 2 * v3 + v4)
        v = v + (dt / 6.0) * (a1 + 2 * a2 + 2 * a3 + a4)
        P[k + 1], V[k + 1] = p, v
    return P, V


def window_fit(t, pos, w, kd=KD_NOMINAL, km=KM_NOMINAL):
    """Shooting fit of (p0,v0) at t[0] on a short window; w = spin (rad/s), fixed.
    Returns dict(p0, v0, t0, rms, w, kd, km)."""
    from scipy.optimize import least_squares
    g = np.array([0, 0, -G_NOM])
    dt = np.median(np.diff(t))
    n = len(t) - 1
    v0g = np.polyfit(t - t[0], pos, 1)[0]

    def resid(x):
        P, _ = rk4_window(x[:3], x[3:], w, kd, km, g, dt, n)
        return (P - pos).ravel()

    sol = least_squares(resid, np.concatenate([pos[0], v0g]), method="lm", max_nfev=200)
    rms = float(np.sqrt(((sol.fun.reshape(-1, 3)) ** 2).sum(1).mean()))
    return dict(p0=sol.x[:3], v0=sol.x[3:], t0=float(t[0]), rms=rms, w=w, kd=kd, km=km)


def prop_state(f, t_eval):
    """Position+velocity at t_eval from a window_fit result (extrapolates both ways)."""
    g = np.array([0, 0, -G_NOM])
    T = t_eval - f["t0"]
    m = max(int(abs(T) / 0.0017), 2)
    P, V = rk4_window(f["p0"], f["v0"], f["w"], f["kd"], f["km"], g, T / m, m)
    return P[-1], V[-1]


def window_state(t, pos, w, t_eval, kd=KD_NOMINAL, km=KM_NOMINAL):
    """Back-compat: fit + evaluate in one call."""
    f = window_fit(t, pos, w, kd, km)
    p, v = prop_state(f, t_eval)
    return p, v, f["rms"]


def paddle_state_at(take, pn, t_c, rate, R_table=None):
    """Racket pose/velocity/normal at t_c, bridged over the +-30 ms occlusion window:
    constant-acceleration fit on centroid over [-120,-15]ms, constant angular velocity
    on quats over the same window; NEVER uses post-contact frames."""
    t = take["t"]
    pos = take[f"{pn}_pos_t_m"].astype(float)
    quat = take[f"{pn}_quat_xyzw"].astype(float)
    nrm = take[f"{pn}_normal_t"].astype(float)
    ok = take[f"{pn}_present"].astype(bool) & np.isfinite(pos[:, 0])
    m = ok & (t >= t_c - 0.120) & (t <= t_c - 0.015)
    if m.sum() < 12:
        return None
    tw = t[m] - t_c                      # negative times
    # quadratic (const-accel) position fit
    A = np.vstack([np.ones_like(tw), tw, 0.5 * tw ** 2]).T
    coef, res_, *_ = np.linalg.lstsq(A, pos[m], rcond=None)
    p_c, v_c, a_c = coef[0], coef[1], coef[2]
    fit_rms = float(np.sqrt(((pos[m] - A @ coef) ** 2).sum(1).mean()))
    # angular velocity from quats in window (relative rotvec linear fit)
    qs = quat[m]
    Rm = Rot.from_quat(qs)
    rv = (Rm[1:] * Rm[:-1].inv()).as_rotvec() / np.diff(tw)[:, None]
    w_pad = np.median(rv, axis=0) @ R_table    # world -> table frame
    # face normal at t_c: rotate window-末 normal forward by w_pad * (t_c - t_last)
    i_last = np.where(m)[0][-1]
    dt_ex = t_c - t[i_last]
    R_ex = Rot.from_rotvec(w_pad * dt_ex)
    n_c = R_ex.apply(nrm[i_last])
    return dict(p=p_c, v=v_c, a=a_c, w=w_pad, n=n_c / np.linalg.norm(n_c),
                fit_rms=fit_rms, n_frames=int(m.sum()), dt_extrap=float(dt_ex))


def main():
    global SURFACE_Z
    qa = json.load(open(os.path.join(paths.ANALYSIS, "qa_stage0.json")))
    surf_vals = [v["surface_z"] for v in qa["surface_calibration"].values()
                 if v.get("surface_z") is not None]
    SURFACE_Z = float(np.median(surf_vals))
    print(f"surface z (table frame, from bounce minima): {SURFACE_Z*1000:.1f} mm")

    os.makedirs(OUT, exist_ok=True)
    flights, bounces, strikes = [], [], []

    for name in TAKES:
        take = load_take(name)
        rate = float(take["rate"])
        Rt = take["table_R"]
        arcs = extract_arcs(take, table_z=SURFACE_Z)
        t_all = take["t"]
        pos_all = take["ball_pos_t_m"].astype(float)
        quat_all = take["ball_quat_xyzw"].astype(float)

        # ---- flights ----
        for ai, a in enumerate(arcs):
            par = arc_parabola(a)
            w, ang, w_rob = spin_from_quats(a["quat"], rate, R_table=Rt)
            ok_spin = bool(np.isfinite(w_rob).all()) and (np.nanmax(ang) < 90 if np.isfinite(ang).any() else False)
            flights.append(dict(
                take=name, arc=ai, i0=int(a["i0"]), i1=int(a["i1"]),
                t0=float(a["t"][0]), dur=float(a["t"][-1] - a["t"][0]),
                n=len(a["t"]),
                v0=[float(x) for x in par["v0"]], speed0=float(np.linalg.norm(par["v0"])),
                g_parab=float(par["g"]), parab_rms_mm=float(par["rms"] * 1e3),
                spin_rads=[float(x) for x in w_rob] if ok_spin else None,
                spin_revs=float(np.linalg.norm(w_rob) / 2 / np.pi) if ok_spin else None,
                degframe_max=float(np.nanmax(ang)) if np.isfinite(ang).any() else None,
                ballistic=bool(arc_is_ballistic(a, par)),
                pre=a["pre_contact"], post=a["post_contact"]))

        # ---- pair consecutive PAIRABLE arcs across gaps (contacts are usually
        # occluded, so the contact falls BETWEEN tracked runs; never require the
        # two arcs to share an in-run cut index) ----
        def pairable(a):
            par = a["par"] if "par" in a else arc_parabola(a)
            dur = a["t"][-1] - a["t"][0]
            return dur >= 0.10 and par["rms"] < 0.020 and 5.0 <= par["g"] <= 16.0

        parcs = [a for a in arcs if pairable(a)]
        for j in range(len(parcs) - 1):
            a, b = parcs[j], parcs[j + 1]
            gap = b["t"][0] - a["t"][-1]
            if not (-0.005 <= gap <= 0.40):
                continue
            wa = slice(max(0, len(a["t"]) - 35), len(a["t"]))
            wb = slice(0, min(35, len(b["t"])))
            _, _, w_pre = spin_from_quats(a["quat"][wa], rate, R_table=Rt)
            _, _, w_post = spin_from_quats(b["quat"][wb], rate, R_table=Rt)
            w_pre_f = w_pre if np.isfinite(w_pre).all() else np.zeros(3)
            w_post_f = w_post if np.isfinite(w_post).all() else np.zeros(3)

            # meeting point of the two extrapolations = contact time
            grid = np.arange(a["t"][-1] - 0.010, b["t"][0] + 0.0101, 0.001)
            if len(grid) < 2:
                continue
            try:
                fa = window_fit(a["t"][wa], a["pos"][wa], w_pre_f)
                fb = window_fit(b["t"][wb], b["pos"][wb], w_post_f)
            except Exception:
                continue
            rms_m, rms_p = fa["rms"], fb["rms"]
            Pa = np.array([prop_state(fa, tg)[0] for tg in grid])
            Pb = np.array([prop_state(fb, tg)[0] for tg in grid])
            dmin_i = int(np.argmin(np.linalg.norm(Pa - Pb, axis=1)))
            meet_mm = float(np.linalg.norm(Pa[dmin_i] - Pb[dmin_i]) * 1e3)
            t_c = float(grid[dmin_i])
            if meet_mm > 60.0:
                continue
            p_m, v_m = prop_state(fa, t_c)
            p_p, v_p = prop_state(fb, t_c)
            dv = np.linalg.norm(v_p - v_m)
            if dv < 0.8:
                continue                          # occlusion gap in one flight, not a contact

            near_table = p_m[2] < SURFACE_Z + R_BALL + 0.060
            if near_table and v_m[2] < -0.2 and v_p[2] > 0.05:
                # t_c stays at the two-sided meeting point. Do NOT refine to the
                # nominal contact height: the bounce-minima surface calibration sits
                # ~9 mm above the true pre/post intersection, and refining to it made
                # t_c early -> gravity back-extrapolation inflated e by ~0.17/vn^2
                # (the e>1 tail and the spurious F3 low-vn slope; see forensics
                # g_e_gt1_dissection).
                t_c2 = t_c
                e_n = -v_p[2] / v_m[2]
                ut = np.linalg.norm((v_m + np.cross(w_pre_f, -R_BALL * np.array([0, 0, 1.0])))[:2])
                bounces.append(dict(
                    take=name, t_c=float(t_c2), z_c=float(p_m[2]),
                    p_c=[float(x) for x in p_m],
                    v_in=[float(x) for x in v_m], v_out=[float(x) for x in v_p],
                    w_in=[float(x) for x in w_pre_f], w_out=[float(x) for x in w_post_f],
                    spin_in_ok=bool(np.isfinite(w_pre).all()),
                    spin_out_ok=bool(np.isfinite(w_post).all()),
                    e_n=float(e_n), vn_in=float(-v_m[2]),
                    ut_over_un=float(ut / max(-v_m[2], 1e-6)),
                    meet_mm=meet_mm,
                    fit_rms_mm=float(max(rms_m, rms_p) * 1e3),
                    gap_ms=float(gap * 1e3)))
                continue

            # candidate racket strike: use extrapolated paddle states, pick closer
            best = None
            for pn in ("p1", "p2"):
                if f"{pn}_pos_t_m" not in take:
                    continue
                pad_c = paddle_state_at(take, pn, t_c, rate, R_table=Rt)
                if pad_c is None:
                    continue
                dmin = float(np.linalg.norm(pad_c["p"] - p_m))
                if best is None or dmin < best[1]:
                    best = (pn, dmin, pad_c)
            if best is None or best[1] > 0.30:
                continue
            pn, dist, pad = best
            n = pad["n"]
            if np.dot(v_m - pad["v"], n) > 0:
                n = -n
            cp = p_m - R_BALL * n            # contact point on ball toward face
            v_r = pad["v"] + np.cross(pad["w"], cp - pad["p"])
            u = v_m + np.cross(w_pre_f, -R_BALL * n) - v_r
            u_n = float(np.dot(u, n))
            u_t = float(np.linalg.norm(u - u_n * n))
            strikes.append(dict(
                take=name, t_c=float(t_c), paddle=pn, dist_m=float(dist),
                ball_p=[float(x) for x in p_m],
                v_in=[float(x) for x in v_m], v_out=[float(x) for x in v_p],
                w_in=[float(x) for x in w_pre_f], w_out=[float(x) for x in w_post_f],
                spin_in_ok=bool(np.isfinite(w_pre).all()),
                spin_out_ok=bool(np.isfinite(w_post).all()),
                pad_p=[float(x) for x in pad["p"]], pad_v=[float(x) for x in pad["v"]],
                pad_w=[float(x) for x in pad["w"]], pad_n=[float(x) for x in n],
                pad_fit_rms_mm=float(pad["fit_rms"] * 1e3),
                pad_dt_extrap_ms=float(pad["dt_extrap"] * 1e3),
                u_n=abs(u_n), u_t=u_t, meet_mm=meet_mm,
                fit_rms_mm=float(max(rms_m, rms_p) * 1e3),
                gap_ms=float(gap * 1e3)))
        print(f"{name}: arcs={len(arcs)} flights_tot={len(flights)} bounces_tot={len(bounces)} strikes_tot={len(strikes)}")

    json.dump(flights, open(os.path.join(OUT, "flights.json"), "w"), indent=1)
    json.dump(bounces, open(os.path.join(OUT, "bounces.json"), "w"), indent=1)
    json.dump(strikes, open(os.path.join(OUT, "strikes.json"), "w"), indent=1)
    json.dump(dict(surface_z=SURFACE_Z), open(os.path.join(OUT, "meta.json"), "w"), indent=1)
    print(f"\nTOTALS: {len(flights)} flights, {len(bounces)} bounces, {len(strikes)} strikes")
    print(f"-> {OUT}/")


if __name__ == "__main__":
    main()
