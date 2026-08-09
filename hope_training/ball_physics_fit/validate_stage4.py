"""
Stage 4 — held-out validation.

  1. flight  : state from first 60 ms -> position error at +0.4 s
  2. bounce  : predicted vs measured outgoing state (table contact params)
  3. strike->landing : paddle model on measured racket state -> flight -> first
     table crossing, scored against the OBSERVED first-bounce contact point p_c
     from bounces.json (continuity-gated pairing). The old reconstructed label
     (integrate measured post-strike state through the same flight model) is
     also reported as `landing_reconstructed_label` for comparison — it is NOT
     the headline number because it shares the flight model with the prediction.

Parameter sources:
  --fits stage2_fits_train.json            (default: the train-split refit)
  --yaml configs/ball_physics_venue.yaml   validates THE SHIPPED CONSTANTS
         (table block incl. the retained v0 grip params; paddle e via
          --paddle-e const|exp|lin, default exp = the yaml recommendation)

Writes analysis/fits/stage4_validation.json (or --out).
"""
import os, sys, json, argparse
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import paths
from ballcore import (TAKES, load_take, extract_arcs, arc_parabola, arc_is_ballistic,
                      spin_from_quats, R_BALL, G_NOM)
from contact_model import predict_contact
from stage2_fits import in_split
from stage1_segments import rk4_window

ANA = paths.ANALYSIS
TABLE_HALF_L, TABLE_HALF_W = 1.370, 0.7625


def load_yaml_constants(path):
    """Minimal indentation-aware parser for ball_physics_venue.yaml (no pyyaml dep)."""
    vals, stack = {}, []
    for raw in open(path):
        line = raw.split("#", 1)[0].rstrip()
        if not line.strip():
            continue
        indent = len(line) - len(line.lstrip())
        key, _, val = line.strip().partition(":")
        val = val.strip()
        while stack and stack[-1][0] >= indent:
            stack.pop()
        dotted = ".".join([s[1] for s in stack] + [key])
        if val == "":
            stack.append((indent, key))
        else:
            try:
                vals[dotted] = float(val)
            except ValueError:
                vals[dotted] = val.strip('"')
    return vals


def flight_state_from_window(t, pos, w, kd, km):
    from scipy.optimize import least_squares
    g = np.array([0, 0, -G_NOM])
    dt = np.median(np.diff(t))
    v0g = np.polyfit(t - t[0], pos, 1)[0]

    def resid(x):
        P, _ = rk4_window(x[:3], x[3:], w, kd, km, g, dt, len(t) - 1)
        return (P - pos).ravel()

    sol = least_squares(resid, np.concatenate([pos[0], v0g]), method="lm", max_nfev=200)
    return sol.x[:3], sol.x[3:]


def integrate_to_table(p0, v0, w, kd, km, surface_z, t_max=2.0, dt=1e-3):
    """RK4 until ball-center crosses surface_z + R while descending: (p, v, t)."""
    g = np.array([0, 0, -G_NOM])
    zc = surface_z + R_BALL
    p, v = np.asarray(p0, float).copy(), np.asarray(v0, float).copy()
    w = np.asarray(w, float)
    t = 0.0

    def acc(v):
        return g - kd * np.linalg.norm(v) * v + km * np.cross(w, v)

    while t < t_max:
        a1 = acc(v); v2 = v + 0.5 * dt * a1; a2 = acc(v2)
        v3 = v + 0.5 * dt * a2; a3 = acc(v3)
        v4 = v + dt * a3; a4 = acc(v4)
        p_new = p + (dt / 6.0) * (v + 2 * v2 + 2 * v3 + v4)
        v_new = v + (dt / 6.0) * (a1 + 2 * a2 + 2 * a3 + a4)
        if p[2] > zc >= p_new[2] and v_new[2] < 0:
            f = (p[2] - zc) / max(p[2] - p_new[2], 1e-9)
            return p + f * (p_new - p), v_new, t + f * dt
        p, v, t = p_new, v_new, t + dt
    return None, None, None


def paddle_outgoing(s, pd):
    """Model-predicted outgoing (v,w) for a strike record under paddle params pd.
    pd: dict with a_t,b_t,mu and mode const (e_eff) | exp (g1,g2) | lin (a,b)."""
    n = np.array(s["pad_n"], float)
    cp = np.array(s["ball_p"]) - R_BALL * n
    v_r = np.array(s["pad_v"]) + np.cross(np.array(s["pad_w"]), cp - np.array(s["pad_p"]))
    u_n = s["u_n"]
    if pd.get("mode") == "exp" and pd.get("g1") is not None:
        e = pd["g1"] * np.exp(pd["g2"] * u_n)
    elif pd.get("mode") == "lin" and pd.get("a") is not None:
        e = max(pd["a"] - pd["b"] * u_n, 0.05)
    else:
        e = pd["e_eff"]
    out = predict_contact(np.array(s["v_in"])[None], v_r[None], n[None],
                          np.array(s["w_in"])[None], e, pd["a_t"], pd["b_t"], pd["mu"])
    return out["v_plus"][0], out["omega_plus"][0]


def pair_strike_bounce(strikes, bounces, kd, km, surface_z,
                       cont_gate_m=0.30, dt_gate_s=0.08):
    """Continuity-gated pairing: strike -> first observed bounce of the SAME ball
    path. Integrates the MEASURED post-strike state to the surface; the landing
    must agree with the bounce's recorded p_c in position and time.
    Returns list of dicts (strike, bounce, flight_only_err, cont_dt)."""
    by_take = {}
    for b in bounces:
        by_take.setdefault(b["take"], []).append(b)
    pairs = []
    for s in strikes:
        cands = [b for b in by_take.get(s["take"], [])
                 if 0.02 < b["t_c"] - s["t_c"] < 2.0]
        if not cands:
            continue
        b0 = min(cands, key=lambda b: b["t_c"])
        if "p_c" not in b0:
            continue
        w_meas = np.array(s["w_out"], float) if s["spin_out_ok"] else np.zeros(3)
        pL, _, tL = integrate_to_table(s["ball_p"], s["v_out"], w_meas, kd, km, surface_z)
        if pL is None:
            continue
        cont = float(np.linalg.norm(pL[:2] - np.array(b0["p_c"])[:2]))
        cont_dt = float(abs((s["t_c"] + tL) - b0["t_c"]))
        if cont > cont_gate_m or cont_dt > dt_gate_s:
            continue
        pairs.append(dict(strike=s, bounce=b0, flight_only_err=cont, cont_dt=cont_dt))
    return pairs


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--fits", default=os.path.join(ANA, "fits", "stage2_fits_train.json"))
    ap.add_argument("--yaml", default=None,
                    help="validate the shipped constants from this yaml instead of --fits")
    ap.add_argument("--paddle-e", default="exp", choices=["const", "exp", "lin"],
                    help="paddle restitution form when --yaml is given")
    ap.add_argument("--split", default="test", choices=["all", "train", "test"])
    ap.add_argument("--out", default=os.path.join(ANA, "fits", "stage4_validation.json"))
    args = ap.parse_args()

    if args.yaml:
        Y = load_yaml_constants(args.yaml)
        kd, km = Y["flight.k_d"], Y["flight.k_m"]
        table = dict(e=Y["contact.table.e_eff"], a_t=Y["contact.table.a_t"],
                     b_t=Y["contact.table.b_t"], mu=Y["contact.table.mu_safety"])
        pd_ = dict(mode=args.paddle_e, e_eff=Y["contact.paddle.e_eff"],
                   a_t=Y["contact.paddle.a_t"], b_t=Y["contact.paddle.b_t"],
                   mu=Y["contact.paddle.mu_safety"],
                   g1=Y.get("contact.paddle.e_exp_g1"), g2=Y.get("contact.paddle.e_exp_g2"),
                   a=Y.get("contact.paddle.e_lin_a"), b=Y.get("contact.paddle.e_lin_b"))
        src = f"yaml:{args.yaml} (paddle e={args.paddle_e})"
    else:
        F = json.load(open(args.fits))
        kd, km = F["kd"]["kd"], F["km"]["km"]
        tt = F["table_tangential"]
        table = dict(e=F["table_e"]["e_const"], a_t=tt.get("a_t", 0.369),
                     b_t=tt.get("b_t", 0.0), mu=tt.get("mu", 2.0))
        p = F["paddle"]
        pd_ = dict(mode="const", e_eff=p.get("e_eff", 0.65), a_t=p.get("a_t", 0.4),
                   b_t=p.get("b_t", 0.0), mu=p.get("mu", 2.5))
        src = f"fits:{args.fits}"

    meta = json.load(open(os.path.join(ANA, "segments", "meta.json")))
    surface_z = meta["surface_z"]
    bounces = json.load(open(os.path.join(ANA, "segments", "bounces.json")))
    strikes = json.load(open(os.path.join(ANA, "segments", "strikes.json")))

    # ---------- 1. flight propagation ----------
    errs = []
    for name in TAKES:
        take = load_take(name)
        Rt = take["table_R"]; rate = float(take["rate"])
        for ai, a in enumerate(extract_arcs(take, table_z=surface_z)):
            if not arc_is_ballistic(a):
                continue
            if not in_split(f"{name}:{ai}", args.split):
                continue
            if a["t"][-1] - a["t"][0] < 0.46:
                continue
            n60 = max(int(0.060 * rate), 6)
            w, ang, w_rob = spin_from_quats(a["quat"][:n60 + 1], rate, R_table=Rt)
            wv = w_rob if np.isfinite(w_rob).all() else np.zeros(3)
            p0, v0 = flight_state_from_window(a["t"][:n60], a["pos"][:n60], wv, kd, km)
            i_t = int(round(0.4 * rate))
            P, _ = rk4_window(p0, v0, wv, kd, km, np.array([0, 0, -G_NOM]), 1.0 / rate, i_t)
            if i_t < len(a["pos"]):
                errs.append(float(np.linalg.norm(P[-1] - a["pos"][i_t])))
    flight_rep = dict(n=len(errs),
                      median_mm=float(np.median(errs) * 1e3) if errs else None,
                      p95_mm=float(np.percentile(errs, 95) * 1e3) if errs else None,
                      target="median<=25mm, p95<=60mm")

    # ---------- 2. bounce prediction ----------
    sp_err, dir_err, w_err = [], [], []
    for b in bounces:
        if not in_split(f'{b["take"]}:{b["t_c"]:.3f}', args.split):
            continue
        if b["fit_rms_mm"] > 5.0 or not b["spin_in_ok"]:
            continue
        out = predict_contact(np.array(b["v_in"])[None], np.zeros(3), np.array([0, 0, 1.0]),
                              np.array(b["w_in"])[None], table["e"], table["a_t"],
                              table["b_t"], table["mu"])
        vp = out["v_plus"][0]; wp = out["omega_plus"][0]
        vm = np.array(b["v_out"])
        sp_err.append(abs(np.linalg.norm(vp) - np.linalg.norm(vm)) / np.linalg.norm(vm))
        cosang = np.dot(vp, vm) / (np.linalg.norm(vp) * np.linalg.norm(vm))
        dir_err.append(float(np.degrees(np.arccos(np.clip(cosang, -1, 1)))))
        if b["spin_out_ok"]:
            wm = np.array(b["w_out"])
            if np.linalg.norm(wm) > 6:
                w_err.append(float(np.linalg.norm(wp - wm) / np.linalg.norm(wm)))
    bounce_rep = dict(n=len(sp_err),
                      speed_err_med_pct=float(np.median(sp_err) * 100) if sp_err else None,
                      dir_err_med_deg=float(np.median(dir_err)) if dir_err else None,
                      spin_err_med_pct=float(np.median(w_err) * 100) if w_err else None,
                      target="speed<=5%, dir<=3deg, spin<=15%")

    # ---------- 3. strike -> landing vs OBSERVED bounce p_c ----------
    pairs = pair_strike_bounce(strikes, bounces, kd, km, surface_z)
    err_obs, err_recon, on_off_ok, details = [], [], [], []
    for pr in pairs:
        s, b0 = pr["strike"], pr["bounce"]
        if not in_split(f'{s["take"]}:{s["t_c"]:.3f}', args.split):
            continue
        if s["fit_rms_mm"] > 8.0 or s["pad_fit_rms_mm"] > 15.0 or not s["spin_in_ok"]:
            continue
        vp, wp = paddle_outgoing(s, pd_)
        pL_mod, _, _ = integrate_to_table(s["ball_p"], vp, wp, kd, km, surface_z)
        if pL_mod is None:
            continue
        gt = np.array(b0["p_c"])[:2]
        err_obs.append(float(np.linalg.norm(pL_mod[:2] - gt)))
        # legacy reconstructed label (shares the flight model with the prediction)
        w_meas = np.array(s["w_out"], float) if s["spin_out_ok"] else np.zeros(3)
        pL_rec, _, _ = integrate_to_table(s["ball_p"], s["v_out"], w_meas, kd, km, surface_z)
        if pL_rec is not None:
            err_recon.append(float(np.linalg.norm(pL_mod[:2] - pL_rec[:2])))
        on_gt = bool(abs(gt[0]) < TABLE_HALF_L and abs(gt[1]) < TABLE_HALF_W)
        on_mod = bool(abs(pL_mod[0]) < TABLE_HALF_L and abs(pL_mod[1]) < TABLE_HALF_W)
        on_off_ok.append(on_gt == on_mod)
        details.append(dict(take=s["take"], t_c=s["t_c"], err_m=err_obs[-1],
                            flight_only_err_m=pr["flight_only_err"],
                            on_gt=on_gt, on_mod=on_mod))
    land_rep = dict(
        n_pairs_clean=len(pairs), n_scored=len(err_obs),
        median_m=float(np.median(err_obs)) if err_obs else None,
        p90_m=float(np.percentile(err_obs, 90)) if err_obs else None,
        onoff_acc_pct=float(np.mean(on_off_ok) * 100) if on_off_ok else None,
        flight_only_median_m=float(np.median([p["flight_only_err"] for p in pairs])) if pairs else None,
        target="median<=0.10m, p90<=0.25m, onoff>=90%",
        ground_truth="observed bounce p_c (continuity-gated)")
    recon_rep = dict(n=len(err_recon),
                     median_m=float(np.median(err_recon)) if err_recon else None,
                     p90_m=float(np.percentile(err_recon, 90)) if err_recon else None,
                     note="legacy label: measured out-state integrated through the same "
                          "flight model — optimistic, kept for comparison only")

    rep = dict(params_source=src, split=args.split,
               constants=dict(kd=kd, km=km, table=table, paddle=pd_),
               flight=flight_rep, bounce=bounce_rep,
               landing=land_rep, landing_reconstructed_label=recon_rep,
               landing_details=details)
    json.dump(rep, open(args.out, "w"), indent=1)
    print(json.dumps({k: v for k, v in rep.items() if k != "landing_details"}, indent=1))


if __name__ == "__main__":
    main()
