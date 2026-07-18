"""Stage 2 -- ordered no-spin fits (each stage freezes the ones before it):

  1. drag k_d           <- ballistic flight arcs (joint RK4 shooting; g frozen)
                           + a per-arc k_d for the F1 speed check
  2. table e_n          <- table bounces (constant + linear e(v_n) with a
                           bootstrap CI for the F3 check)
  3. table tangential   <- table bounces, e_n frozen (a_t, b_t, mu)
  4. paddle contact     <- racket strikes that carry paddle state (e_n, a_t,
                           b_t, mu) + per-strike e vs |u_n| for the F4 check

Reads analysis/segments/{flights,bounces,strikes}.json (from stage1) and rebuilds
the flight arcs from the trajectory CSVs for the drag fit. Writes
analysis/fits/stage2_fits.json. Stages with too little data degrade to a note.

The paddle stage needs strikes with racket contact-point velocity + face normal
(provide a paddle sidecar to stage1); ball-only capture fits flight + table.

    python stage2_fits.py
"""
import json
import os

import numpy as np
from scipy.optimize import least_squares

import paths
from ballcore import (load_trajectory, extract_arcs, arc_parabola, arc_is_ballistic,
                      fit_arcs_global, R_BALL)
from contact_model import predict_contact

OUT = paths.FITS
SEG = paths.SEGMENTS


def rebuild_ballistic_arcs():
    arcs = []
    for path in paths.trajectory_files():
        traj = load_trajectory(path)
        for a in extract_arcs(traj):
            a["par"] = arc_parabola(a)
            if arc_is_ballistic(a, a["par"]):
                arcs.append(a)
    return arcs


def fit_kd(arcs, min_dur=0.2, max_arcs=200):
    sel = [a for a in arcs if (a["t"][-1] - a["t"][0]) >= min_dur]
    sel.sort(key=lambda a: -(a["t"][-1] - a["t"][0]))
    sel = sel[:max_arcs]
    if not sel:
        return dict(note="no ballistic arc >= min_dur; need longer flight arcs")
    fit = fit_arcs_global(sel, fit=("kd",))
    per = []
    for a in sel:
        f1 = fit_arcs_global([a], fit=("kd",))
        per.append(dict(name=a["name"], speed=float(np.linalg.norm(a["par"]["v0"])),
                        kd=float(f1["kd"]), rms_mm=float(f1["rms_per_arc"][0] * 1e3),
                        dur=float(a["t"][-1] - a["t"][0])))
    return dict(kd=float(fit["kd"]), n_arcs=len(sel),
                rms_med_mm=float(np.median(fit["rms_per_arc"]) * 1e3), per_arc=per)


def fit_table_e(bounces, rms_max_mm=8.0):
    sel = [b for b in bounces if 0.0 < b["e_n"] < 1.8 and b["fit_rms_mm"] < rms_max_mm]
    if len(sel) < 3:
        return dict(note=f"only {len(sel)} clean bounces; need more for e_n",
                    n=len(sel),
                    e_const=(float(np.median([b["e_n"] for b in sel])) if sel else None),
                    per_bounce=[dict(name=b["name"], e=b["e_n"], vn=b["vn_in"],
                                     ut_over_un=b["ut_over_un"]) for b in sel])
    e = np.array([b["e_n"] for b in sel])
    vn = np.array([b["vn_in"] for b in sel])
    A = np.vstack([np.ones_like(vn), vn]).T
    coef, *_ = np.linalg.lstsq(A, e, rcond=None)
    rng = np.random.default_rng(0)
    boots = np.array([np.linalg.lstsq(A[i], e[i], rcond=None)[0]
                      for i in (rng.integers(0, len(e), len(e)) for _ in range(400))])
    return dict(n=len(sel), e_const=float(np.median(e)),
                e_mad=float(np.median(np.abs(e - np.median(e)))),
                linear_a=float(coef[0]), linear_b=float(-coef[1]),
                slope_ci=[float(np.percentile(-boots[:, 1], q)) for q in (2.5, 97.5)],
                vn_range=[float(vn.min()), float(vn.max())],
                per_bounce=[dict(name=b["name"], e=b["e_n"], vn=b["vn_in"],
                                 ut_over_un=b["ut_over_un"]) for b in sel])


def _fit_contact(rows, e_fixed=None, fit_e=True):
    """LS over contact events matching predicted -> measured outgoing velocity.
    rows: dicts with v_in, v_out, v_r, n (all length-3)."""
    V_in = np.array([r["v_in"] for r in rows])
    V_out = np.array([r["v_out"] for r in rows])
    V_r = np.array([r["v_r"] for r in rows])
    N = np.array([r["n"] for r in rows])

    def resid(x):
        e, a_t, b_t, mu = (x if fit_e else (e_fixed, *x))
        out = predict_contact(V_in, V_r, N, e, a_t, b_t, abs(mu))
        return (out["v_plus"] - V_out).ravel()

    x0 = [0.7, 0.4, 0.0, 1.0] if fit_e else [0.4, 0.0, 1.0]
    sol = least_squares(resid, x0, loss="huber", f_scale=0.3, max_nfev=2000)
    x = sol.x
    dv = (resid(x)).reshape(-1, 3)
    out = dict(a_t=float(x[-3]), b_t=float(x[-2]), mu=float(abs(x[-1])),
               dv_rms=float(np.sqrt((dv ** 2).sum(1).mean())), n=len(rows))
    if fit_e:
        out["e_eff"] = float(x[0])
    return out


def fit_table_tangential(bounces, e_const, rms_max_mm=8.0):
    rows = [dict(v_in=b["v_in"], v_out=b["v_out"], v_r=[0, 0, 0], n=[0, 0, 1.0])
            for b in bounces if b["fit_rms_mm"] < rms_max_mm]
    if len(rows) < 5:
        return dict(note=f"only {len(rows)} bounces; need >=5 for tangential")
    return _fit_contact(rows, e_fixed=e_const, fit_e=False)


def fit_paddle(strikes, rms_max_mm=12.0):
    usable = [s for s in strikes if s.get("pad_n") and s.get("pad_v")
              and s["fit_rms_mm"] < rms_max_mm]
    if len(usable) < 10:
        return dict(note=("need >=10 strikes with paddle state (racket contact-point "
                          "velocity + face normal) to fit the paddle contact; "
                          f"have {len(usable)}. Provide a paddle sidecar to stage1."),
                    n_usable=len(usable))
    rows = [dict(v_in=s["v_in"], v_out=s["v_out"], v_r=s["pad_v"], n=s["pad_n"])
            for s in usable]
    fit = _fit_contact(rows, fit_e=True)
    # per-strike effective e vs |u_n| (racket frame) for F4
    per = []
    for s in usable:
        v_in = np.array(s["v_in"]); v_out = np.array(s["v_out"])
        vr = np.array(s["pad_v"]); n = np.array(s["pad_n"], float)
        if np.dot(v_in - vr, n) > 0:
            n = -n
        un_in = float(np.dot(v_in - vr, n))
        un_out = float(np.dot(v_out - vr, n))
        if abs(un_in) < 0.3:
            continue
        u_t = float(np.linalg.norm((v_in - vr) - un_in * n))
        per.append(dict(name=s["name"], u_n=abs(un_in), e=-un_out / un_in, u_t=u_t))
    extra = {}
    if per:
        un = np.array([p["u_n"] for p in per])
        ee = np.array([p["e"] for p in per])
        good = (ee > -0.2) & (ee < 1.5)
        un, ee = un[good], ee[good]
        if good.sum() >= 3:
            lin = np.linalg.lstsq(np.vstack([np.ones_like(un), un]).T, ee, rcond=None)[0]
            pos = ee > 0.05
            ex = np.polyfit(un[pos], np.log(ee[pos]), 1) if pos.sum() > 5 else [np.nan, np.nan]
            extra = dict(e_lin_a=float(lin[0]), e_lin_b=float(-lin[1]),
                         e_exp_g1=float(np.exp(ex[1])), e_exp_g2=float(ex[0]),
                         n_e=int(good.sum()))
    return dict(**fit, per_strike=per, **extra)


def main():
    os.makedirs(OUT, exist_ok=True)
    bounces = json.load(open(os.path.join(SEG, "bounces.json")))
    strikes = json.load(open(os.path.join(SEG, "strikes.json")))

    print("rebuilding flight arcs...")
    arcs = rebuild_ballistic_arcs()
    print(f"  {len(arcs)} ballistic arcs")

    kd = fit_kd(arcs)
    print(f"[1] k_d: {json.dumps({k: v for k, v in kd.items() if k != 'per_arc'})}")

    te = fit_table_e(bounces)
    print(f"[2] table e_n: {json.dumps({k: v for k, v in te.items() if k != 'per_bounce'})}")

    tt = (fit_table_tangential(bounces, te["e_const"])
          if te.get("e_const") is not None else dict(note="no e_const"))
    print(f"[3] table tangential: {json.dumps(tt)}")

    pd_ = fit_paddle(strikes)
    print(f"[4] paddle: {json.dumps({k: v for k, v in pd_.items() if k != 'per_strike'})}")

    result = dict(kd=kd, table_e=te, table_tangential=tt, paddle=pd_)
    path = os.path.join(OUT, "stage2_fits.json")
    json.dump(result, open(path, "w"), indent=1)
    print(f"-> {path}")


if __name__ == "__main__":
    main()
