"""
Stage 2 — ordered fits (each stage freezes everything before it):

  1. k_d  <- low-spin ballistic arcs (joint shooting; g frozen 9.81)
             + per-arc k_hat_d for the F1 speed bins
  2. k_m  <- sidespin-channel arcs, kd frozen (joint) + per-arc k_hat_m vs SR (F2)
  3. table e_eff <- bounces: constant + linear e(v_n) with bootstrap CI (F3)
  4. table a_t, b_t, mu <- bounces with trusted spin (e frozen); residual vs
     |u_t|/|u_n| for the slip probe (F5)
  5. paddle e_eff, a_t, b_t, mu <- strikes (everything else frozen); per-strike
     e vs |u_n| (F4); constant vs linear vs Ace-exponential forms

--split {all,train,test}: stratified 70/30 by trial (arc/bounce/strike) using a
fixed hash, so Stage 4 can fit on train and score on test.
Writes analysis/fits/stage2_fits.json (+ per-item tables for the F battery).
"""
import os, sys, json, argparse, hashlib
import numpy as np
import paths
from scipy.optimize import least_squares

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from ballcore import (TAKES, load_take, extract_arcs, arc_parabola, arc_is_ballistic,
                      spin_from_quats, fit_arcs_global, R_BALL, G_NOM)
from contact_model import predict_contact

ANA = paths.ANALYSIS
OUT = os.path.join(ANA, "fits")
BALL_MASS_TAPED = 0.0034   # kg, coated ball (memory: 3.4 g vs clean 2.70 g)
BALL_MASS_CLEAN = 0.0027


def in_split(key, split):
    if split == "all":
        return True
    h = int(hashlib.md5(key.encode()).hexdigest(), 16) % 100
    return (h < 70) if split == "train" else (h >= 70)


def rebuild_arcs(surface_z):
    """Arc objects (with pos/quat) keyed the same way as flights.json rows.
    Cached to a pickle (keyed by surface_z) because the rebuild costs minutes."""
    import pickle
    cache = os.path.join(OUT, f"arcs_cache_{surface_z:.4f}.pkl")
    if os.path.exists(cache):
        with open(cache, "rb") as f:
            return pickle.load(f)
    arcs_by_key = {}
    for name in TAKES:
        take = load_take(name)
        Rt = take["table_R"]
        rate = float(take["rate"])
        for ai, a in enumerate(extract_arcs(take, table_z=surface_z)):
            a["par"] = arc_parabola(a)
            w, ang, w_rob = spin_from_quats(a["quat"], rate, R_table=Rt)
            a["w_rob"] = w_rob
            a["degframe_max"] = float(np.nanmax(ang)) if np.isfinite(ang).any() else None
            a["spin_ok"] = bool(np.isfinite(w_rob).all()) and (a["degframe_max"] or 999) < 90
            arcs_by_key[(name, ai)] = a
        print(f"  arcs: {name} done", flush=True)
    os.makedirs(OUT, exist_ok=True)
    with open(cache, "wb") as f:
        pickle.dump(arcs_by_key, f)
    return arcs_by_key


def _single_kd(a):
    f1 = fit_arcs_global([a], fit=("kd",))
    return dict(take=a["take"], speed=float(np.linalg.norm(a["par"]["v0"])),
                kd=f1["kd"], rms_mm=float(f1["rms_per_arc"][0] * 1e3),
                dur=float(a["t"][-1] - a["t"][0]))


def _single_km(args):
    a, row_base, kd = args
    f1 = fit_arcs_global([a], fit=("km",), kd0=kd, w_per_arc=a["w_rob"][None])
    return dict(row_base, km=f1["km"], rms_mm=float(f1["rms_per_arc"][0] * 1e3),
                sidespin=bool(row_base["axis_z"] > 0.6))


def _pmap(fn, items, procs=8):
    if not items:
        return []
    from multiprocessing import get_context
    with get_context("fork").Pool(procs) as p:
        return p.map(fn, items, chunksize=1)


def fit_kd(arcs, split):
    sel = [a for (n, i), a in arcs.items()
           if arc_is_ballistic(a, a["par"]) and a["spin_ok"]
           and np.linalg.norm(a["w_rob"]) / (2 * np.pi) < 3.0
           and (a["t"][-1] - a["t"][0]) >= 0.2
           and in_split(f"{n}:{i}", split)]
    sel.sort(key=lambda a: -(a["t"][-1] - a["t"][0]))
    sel = sel[:100]
    print(f"  kd: joint fit over {len(sel)} arcs...", flush=True)
    fit = fit_arcs_global(sel, fit=("kd",))
    print(f"  kd joint = {fit['kd']:.4f}; per-arc fits...", flush=True)
    rows = _pmap(_single_kd, sel)
    return dict(kd=fit["kd"], n_arcs=len(sel),
                rms_med_mm=float(np.median(fit["rms_per_arc"]) * 1e3),
                loo_std=None,
                per_arc=rows)


def fit_km(arcs, kd, split):
    """Sidespin channel: spin axis mostly vertical (|w_z| dominant), spin >= 4 rev/s."""
    sel, w_list = [], []
    per_all = []
    for (n, i), a in arcs.items():
        if not (arc_is_ballistic(a, a["par"]) and a["spin_ok"]):
            continue
        w = a["w_rob"]
        rev = np.linalg.norm(w) / (2 * np.pi)
        if rev < 4.0 or (a["t"][-1] - a["t"][0]) < 0.2:
            continue
        axis = w / (np.linalg.norm(w) + 1e-9)
        v0 = a["par"]["v0"]
        sr = R_BALL * np.linalg.norm(w) / max(np.linalg.norm(v0), 0.1)
        row_base = dict(take=n, arc=i, rev=float(rev), sr=float(sr),
                        axis_z=float(abs(axis[2])),
                        speed=float(np.linalg.norm(v0)))
        per_all.append((a, row_base))
        if abs(axis[2]) > 0.6 and in_split(f"{n}:{i}", split):
            sel.append(a); w_list.append(w)
    fit = None
    if len(sel) >= 5:
        print(f"  km: joint sidespin fit over {len(sel)} arcs...", flush=True)
        fit = fit_arcs_global(sel, fit=("km",), kd0=kd, w_per_arc=np.array(w_list))
    # per-arc km for the F2 SR bins (all spin arcs, any axis; sidespin flagged)
    print(f"  km per-arc fits: {len(per_all)}...", flush=True)
    rows = _pmap(_single_km, [(a, row, kd) for a, row in per_all])
    return dict(km=fit["km"] if fit else None,
                n_sidespin=len(sel),
                rms_med_mm=float(np.median(fit["rms_per_arc"]) * 1e3) if fit else None,
                per_arc=rows)


def fit_table_e(bounces, split):
    # No censoring on e itself (indefensible on the fitted quantity; the e>1 tail
    # was a t_c bias, fixed in stage1 — residual tail is ~4% symmetric noise).
    # Keep only quality gates + a physicality bound far outside the noise tail.
    sel = [b for b in bounces if 0.0 < b["e_n"] < 1.8 and b["fit_rms_mm"] < 5.0
           and in_split(f'{b["take"]}:{b["t_c"]:.3f}', split)]
    e = np.array([b["e_n"] for b in sel])
    vn = np.array([b["vn_in"] for b in sel])
    const = float(np.median(e))
    A = np.vstack([np.ones_like(vn), vn]).T
    coef, *_ = np.linalg.lstsq(A, e, rcond=None)
    boots = []
    rng = np.random.default_rng(0)
    for _ in range(400):
        idx = rng.integers(0, len(e), len(e))
        c, *_ = np.linalg.lstsq(A[idx], e[idx], rcond=None)
        boots.append(c)
    boots = np.array(boots)
    return dict(n=len(sel), e_const=const,
                e_mad=float(np.median(np.abs(e - const))),
                linear_a=float(coef[0]), linear_b=float(-coef[1]),
                slope_ci=[float(np.percentile(-boots[:, 1], q)) for q in (2.5, 97.5)],
                vn_range=[float(vn.min()), float(vn.max())] if len(vn) else None,
                per_bounce=[dict(take=b["take"], t_c=b["t_c"], e=b["e_n"], vn=b["vn_in"],
                                 ut_over_un=b["ut_over_un"]) for b in sel])


C_W_SCALE = 0.0133  # (2/3)*R: converts a rad/s spin residual to m/s-equivalent


def _fit_contact_params(rows, e_fixed=None, w_scale=C_W_SCALE, x0=None, fit_e=True):
    """Shared LS over contact events. rows: dicts with v_in,w_in,v_r,n,v_out,w_out.
    Residual = [dv_pred - dv_meas (3), w_scale*(dw_pred - dw_meas) (3)] per event."""
    V_in = np.array([r["v_in"] for r in rows])
    W_in = np.array([r["w_in"] for r in rows])
    V_out = np.array([r["v_out"] for r in rows])
    W_out = np.array([r["w_out"] for r in rows])
    V_r = np.array([r["v_r"] for r in rows])
    N = np.array([r["n"] for r in rows])
    w_ok = np.array([r.get("w_ok", True) for r in rows])

    def resid(x):
        if fit_e:
            e, a_t, b_t, mu = x
        else:
            a_t, b_t, mu = x
            e = e_fixed
        out = predict_contact(V_in, V_r, N, W_in, e, a_t, b_t, abs(mu))
        rv = (out["v_plus"] - V_out)
        rw = (out["omega_plus"] - W_out) * w_scale
        rw[~w_ok] = 0.0
        return np.concatenate([rv.ravel(), rw.ravel()])

    if x0 is None:
        x0 = [0.9, 0.4, 0.0, 1.0] if fit_e else [0.4, 0.0, 1.0]
    sol = least_squares(resid, x0, loss="huber", f_scale=0.3, max_nfev=2000)
    x = sol.x
    res = resid(x)
    n = len(rows)
    rv = res[:3 * n].reshape(-1, 3)
    rw = res[3 * n:].reshape(-1, 3) / w_scale
    out = dict(a_t=float(x[-3]), b_t=float(x[-2]), mu=float(abs(x[-1])),
               dv_rms=float(np.sqrt((rv ** 2).sum(1).mean())),
               dw_rms_revs=float(np.sqrt((rw[w_ok] ** 2).sum(1).mean()) / (2 * np.pi)))
    if fit_e:
        out["e_eff"] = float(x[0])
    return out


def fit_table_tangential(bounces, e_const, split):
    rows = []
    for b in bounces:
        if not (b["spin_in_ok"] and b["spin_out_ok"]):
            continue
        if b["fit_rms_mm"] > 5.0 or not in_split(f'{b["take"]}:{b["t_c"]:.3f}', split):
            continue
        rows.append(dict(v_in=b["v_in"], w_in=b["w_in"], v_out=b["v_out"], w_out=b["w_out"],
                         v_r=[0, 0, 0], n=[0, 0, 1.0], take=b["take"], t_c=b["t_c"],
                         ut_over_un=b["ut_over_un"]))
    if len(rows) < 10:
        return dict(error=f"only {len(rows)} spin-complete bounces")
    fit = _fit_contact_params(rows, e_fixed=e_const, fit_e=False)
    # residuals vs ratio for F5
    out = predict_contact(np.array([r["v_in"] for r in rows]), np.zeros(3),
                          np.array([0, 0, 1.0]), np.array([r["w_in"] for r in rows]),
                          e_const, fit["a_t"], fit["b_t"], fit["mu"])
    dw_res = (out["omega_plus"] - np.array([r["w_out"] for r in rows]))
    dv_res = (out["v_plus"] - np.array([r["v_out"] for r in rows]))
    per = [dict(take=r["take"], t_c=r["t_c"], ratio=float(r["ut_over_un"]),
                dw_revs=float(np.linalg.norm(dw_res[i]) / (2 * np.pi)),
                dvt=float(np.linalg.norm(dv_res[i][:2])),
                cap_binds=bool(out["cap_binds"][i]))
           for i, r in enumerate(rows)]
    return dict(**fit, n=len(rows), per_bounce=per)


def fit_paddle(strikes, split):
    rows = []
    for s in strikes:
        if not s["spin_in_ok"]:
            continue
        if s["fit_rms_mm"] > 8.0 or s["pad_fit_rms_mm"] > 15.0:
            continue
        if not in_split(f'{s["take"]}:{s["t_c"]:.3f}', split):
            continue
        cp = np.array(s["ball_p"]) - R_BALL * np.array(s["pad_n"])
        v_r = np.array(s["pad_v"]) + np.cross(np.array(s["pad_w"]), cp - np.array(s["pad_p"]))
        rows.append(dict(v_in=s["v_in"], w_in=s["w_in"], v_out=s["v_out"],
                         w_out=s["w_out"], v_r=v_r, n=s["pad_n"],
                         w_ok=s["spin_out_ok"], take=s["take"], t_c=s["t_c"],
                         u_n=s["u_n"], u_t=s["u_t"], pad_w=s["pad_w"]))
    if len(rows) < 10:
        return dict(error=f"only {len(rows)} usable strikes")
    fit = _fit_contact_params(rows, fit_e=True)
    # per-strike effective e for F4: e_i = -(u_out . n)/(u_in . n) in racket frame
    per = []
    for r in rows:
        n = np.asarray(r["n"], float)
        v_in = np.asarray(r["v_in"], float); v_out = np.asarray(r["v_out"], float)
        vr = np.asarray(r["v_r"], float)
        if np.dot(v_in - vr, n) > 0:
            n = -n
        u_in = v_in + np.cross(r["w_in"], -R_BALL * n) - vr
        u_out = v_out + np.cross(r["w_out"], -R_BALL * n) - vr
        un_in = float(np.dot(u_in, n)); un_out = float(np.dot(u_out, n))
        if abs(un_in) < 0.3:
            continue
        per.append(dict(take=r["take"], t_c=r["t_c"], u_n=abs(un_in),
                        e=-un_out / un_in, u_t=float(r["u_t"]),
                        pad_w_mag=float(np.linalg.norm(r["pad_w"]))))
    # e(u_n) linear + exponential fits
    if per:
        un = np.array([p["u_n"] for p in per]); ee = np.array([p["e"] for p in per])
        good = (ee > -0.2) & (ee < 1.5)
        un, ee = un[good], ee[good]
        A = np.vstack([np.ones_like(un), un]).T
        lin, *_ = np.linalg.lstsq(A, ee, rcond=None)
        # exp fit via log on positive e
        pos = ee > 0.05
        ex = np.polyfit(un[pos], np.log(ee[pos]), 1) if pos.sum() > 5 else [np.nan, np.nan]
        extra = dict(e_lin_a=float(lin[0]), e_lin_b=float(-lin[1]),
                     e_exp_g1=float(np.exp(ex[1])), e_exp_g2=float(ex[0]),
                     n_e=int(good.sum()))
    else:
        extra = {}
    return dict(**fit, n=len(rows), per_strike=per, **extra)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--split", default="all", choices=["all", "train", "test"])
    ap.add_argument("--tag", default=None)
    args = ap.parse_args()
    tag = args.tag or args.split

    os.makedirs(OUT, exist_ok=True)
    meta = json.load(open(os.path.join(ANA, "segments", "meta.json")))
    bounces = json.load(open(os.path.join(ANA, "segments", "bounces.json")))
    strikes = json.load(open(os.path.join(ANA, "segments", "strikes.json")))

    arcs = rebuild_arcs(meta["surface_z"])
    print(f"arcs rebuilt: {len(arcs)}")

    kd_fit = fit_kd(arcs, args.split)
    print(f"[1] kd = {kd_fit['kd']:.4f} (n={kd_fit['n_arcs']}, "
          f"rms {kd_fit['rms_med_mm']:.1f} mm)")

    km_fit = fit_km(arcs, kd_fit["kd"], args.split)
    print(f"[2] km = {km_fit['km']} (n_sidespin={km_fit['n_sidespin']})")

    te_fit = fit_table_e(bounces, args.split)
    print(f"[3] table e = {te_fit['e_const']:.4f} (n={te_fit['n']}), "
          f"slope {te_fit['linear_b']:.4f}/m/s CI {te_fit['slope_ci']}")

    tt_fit = fit_table_tangential(bounces, te_fit["e_const"], args.split)
    print(f"[4] table tangential: {json.dumps({k: v for k, v in tt_fit.items() if k != 'per_bounce'})}")

    pd_fit = fit_paddle(strikes, args.split)
    print(f"[5] paddle: {json.dumps({k: v for k, v in pd_fit.items() if k != 'per_strike'})}")

    result = dict(split=args.split, surface_z=meta["surface_z"],
                  ball_mass_taped=BALL_MASS_TAPED, ball_mass_clean=BALL_MASS_CLEAN,
                  kd=kd_fit, km=km_fit, table_e=te_fit, table_tangential=tt_fit,
                  paddle=pd_fit)
    path = os.path.join(OUT, f"stage2_fits_{tag}.json")
    json.dump(result, open(path, "w"), indent=1)
    print(f"-> {path}")


if __name__ == "__main__":
    main()
