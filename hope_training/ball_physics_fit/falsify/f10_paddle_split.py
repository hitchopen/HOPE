#!/usr/bin/env python
"""F10: paddle-identity / face / blade-position splits of the racket contact model.

Answers, on the venue strikes (strikes.json from stage1): 1) do the two rackets
('p1'/'p2') need separate coefficients?  2) do the two faces of each racket differ
(正/反面)?  3) does contact position on the blade (|r_t|) shift e (sweet spot)?

Per-strike quantities reuse the stage2 fit_paddle conventions EXACTLY (racket-frame
relative contact velocity, normal re-flipped against v_r, e_i = -u_n_out/u_n_in,
gates fit_rms<=8 / pad_fit_rms<=15 mm). a_i = |dv_t|/|u_t| (velocity channel;
strikes with |u_t| < 0.5 m/s excluded from a_i).

FACE-IDENTITY SUBTLETY: stage1 stores pad_n AFTER flipping it toward the incoming
ball, and ball_p sits ~R along +pad_n off the blade whichever face was struck, so
sign(dot(ball_p - pad_p, pad_n)) is degenerate BY CONSTRUCTION (could only split if
the body origin sat > R off the blade plane; tested anyway). When degenerate, fall
back to the take npz: the raw {pn}_normal_t channel is rigidly tied to ONE physical
face -> face = sign(dot(analysis n, raw normal extrapolated to t_c with pad_w — the
exact stage1 construction)). npz access is LAZY; the active path is recorded in the
JSON ("face_source"). Verdicts: DIFFERENT / SAME-within-noise / UNDERPOWERED
(thresholds in JSON; UNDERPOWERED also covers "inconclusive").
"""
import os
import sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import paths
import json
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from scipy import stats

R_BALL = 0.020
SHIPPED_G1, SHIPPED_G2 = 0.759, -0.0441      # configs/ball_physics_venue.yaml paddle e(u_n)
SHIPPED_AT = 0.52
GATE_FIT_RMS_MM, GATE_PAD_RMS_MM = 8.0, 15.0  # stage2 fit_paddle gates, verbatim
E_GATE_LO, E_GATE_HI = -0.2, 1.5
UN_MIN, UT_MIN_FOR_A = 0.3, 0.5
ENVELOPE_UN = (1.4, 7.2)                      # paddle validity envelope (venue fit)
MATERIAL_E, BAND_E = 0.03, 0.05               # material / noise band on median e
MATERIAL_A, BAND_A = 0.05, 0.10
P_DIFF, P_SAME = 0.01, 0.05
N_MIN_GROUP = 15
N_BOOT, N_PERM = 2000, 3000

STRIKES = os.path.join(paths.SEGMENTS, "strikes.json")
OUTDIR = paths.FALSIFICATION
rng = np.random.default_rng(10)
# Okabe-Ito (CVD-safe); identity also carried by marker shape in the face panel
COL = {"p1": "#0072B2", "p2": "#E69F00", "n+": "#009E73", "n-": "#CC79A7"}


def require(path, what):
    """paths.require_oracle-style LOUD failure: never skip silently."""
    if not os.path.isfile(path):
        raise FileNotFoundError(
            f"{what} not found at {path!r}. Set BALLFIT_DATA_ROOT to the venue "
            "recording folder (holding analysis/segments/ from stage1_segments.py). "
            "Refusing to skip: paddle/face splits must run on recorded strikes.")


def shipped_e(un):
    return SHIPPED_G1 * np.exp(SHIPPED_G2 * np.asarray(un, float))


def build_rows(strikes):
    """Per-strike e/a table; stage2 fit_paddle formulas verbatim."""
    rows, n_gate, n_egate = [], 0, 0
    for s in strikes:
        if (not s["spin_in_ok"] or s["fit_rms_mm"] > GATE_FIT_RMS_MM
                or s["pad_fit_rms_mm"] > GATE_PAD_RMS_MM):
            n_gate += 1
            continue
        n = np.asarray(s["pad_n"], float)
        pad_p = np.asarray(s["pad_p"], float)
        ball_p = np.asarray(s["ball_p"], float)
        cp = ball_p - R_BALL * n
        v_r = np.asarray(s["pad_v"], float) + np.cross(
            np.asarray(s["pad_w"], float), cp - pad_p)
        v_in, v_out = np.asarray(s["v_in"], float), np.asarray(s["v_out"], float)
        if np.dot(v_in - v_r, n) > 0:        # stage2 re-flip (vs v_r, not pad_v)
            n = -n
        u_in = v_in + np.cross(s["w_in"], -R_BALL * n) - v_r
        u_out = v_out + np.cross(s["w_out"], -R_BALL * n) - v_r
        un_in, un_out = float(np.dot(u_in, n)), float(np.dot(u_out, n))
        if abs(un_in) < UN_MIN:
            n_gate += 1
            continue
        e = -un_out / un_in
        if not (E_GATE_LO < e < E_GATE_HI):
            n_egate += 1
            continue
        u_t = float(np.linalg.norm(u_in - un_in * n))
        dv = v_out - v_in
        dv_t = dv - np.dot(dv, n) * n
        a = float(np.linalg.norm(dv_t) / u_t) if u_t >= UT_MIN_FOR_A else np.nan
        r_vec = ball_p - pad_p
        d_n = float(np.dot(r_vec, n))
        rows.append(dict(take=s["take"], t_c=float(s["t_c"]), paddle=s["paddle"],
                         u_n=abs(un_in), u_t=u_t, e=e, a=a, d_n=d_n,
                         r_t=float(np.linalg.norm(r_vec - d_n * n)),
                         n_used=n.tolist(), pad_w=list(map(float, s["pad_w"]))))
    return rows, n_gate, n_egate


def resolve_faces(rows):
    """Face per strike. Primary: sign(d_n) IF bimodal per paddle (it is degenerate
    by construction unless the body origin sits off the blade). Else npz fallback."""
    primary_ok = True
    for p in ("p1", "p2"):
        d = np.array([r["d_n"] for r in rows if r["paddle"] == p])
        if len(d) and min((d > 0).mean(), (d < 0).mean()) < 0.10:
            primary_ok = False
    if primary_ok and rows:
        for r in rows:
            r["face"] = "n+" if r["d_n"] > 0 else "n-"
        return "strikes_json:sign(dot(ball_p-pad_p,n))", 0

    # fallback: body-face channel from the take npz (LAZY import: oracle only here)
    from ballcore import load_take
    from scipy.spatial.transform import Rotation as Rot
    cache, n_unresolved = {}, 0
    for r in rows:
        if r["take"] not in cache:
            try:
                cache[r["take"]] = load_take(r["take"])
            except (FileNotFoundError, OSError) as exc:
                raise FileNotFoundError(
                    f"F10 face split needs analysis/extracted npz for take "
                    f"{r['take']!r} (BALLFIT_DATA_ROOT={paths.DATA_ROOT}); the "
                    "stored pad_n is approach-flipped so face identity is only in "
                    "the paddle body channel. Refusing to skip.") from exc
        take, pn, t_c = cache[r["take"]], r["paddle"], r["t_c"]
        t = take["t"]
        nrm = take[f"{pn}_normal_t"].astype(float)
        ok = (take[f"{pn}_present"].astype(bool) & np.isfinite(nrm[:, 0])
              & (t >= t_c - 0.120) & (t <= t_c - 0.015))     # stage1 window
        if not ok.any():
            r["face"] = None
            n_unresolved += 1
            continue
        i_last = int(np.where(ok)[0][-1])
        n_raw = Rot.from_rotvec(np.asarray(r["pad_w"]) * (t_c - t[i_last])
                                ).apply(nrm[i_last])         # stage1 extrapolation
        c = float(np.dot(r["n_used"], n_raw) / (np.linalg.norm(n_raw) + 1e-12))
        if abs(c) < 0.5:
            r["face"] = None
            n_unresolved += 1
        else:
            r["face"] = "n+" if c > 0 else "n-"
    return "npz_body_normal:sign(dot(n_used,raw {pn}_normal_t at t_c))", n_unresolved


def exp_fit(un, e):
    """Ace-form e = g1*exp(g2*u_n), log-linear LSQ on e>0.05 (stage2 convention)."""
    un, e = np.asarray(un, float), np.asarray(e, float)
    m = e > 0.05
    if m.sum() < 8:
        return dict(g1=None, g2=None, n=int(m.sum()))
    c = np.polyfit(un[m], np.log(e[m]), 1)
    return dict(g1=float(np.exp(c[1])), g2=float(c[0]), n=int(m.sum()))


def boot_dmed(xa, xb, B=N_BOOT):
    d = [np.median(rng.choice(xa, len(xa))) - np.median(rng.choice(xb, len(xb)))
         for _ in range(B)]
    return [float(np.percentile(d, 2.5)), float(np.percentile(d, 97.5))]


def strat_perm_p(x, is_a, takes, B=N_PERM):
    """Median-difference permutation WITHIN take: guards the take/player<->paddle
    confound (takes containing one group only contribute nothing, correctly)."""
    obs = np.median(x[is_a]) - np.median(x[~is_a])
    lab, cnt = is_a.copy(), 0
    tk = {t: np.where(takes == t)[0] for t in np.unique(takes)}
    for _ in range(B):
        for idx in tk.values():
            lab[idx] = lab[idx][rng.permutation(len(idx))]
        if abs(np.median(x[lab]) - np.median(x[~lab])) >= abs(obs) - 1e-15:
            cnt += 1
    return float((cnt + 1) / (B + 1))


def two_group(D, mask_a, mask_b, la, lb):
    """e (residual vs shipped exp, i.e. u_n-matched) + a comparison, with verdicts."""
    na, nb = int(mask_a.sum()), int(mask_b.sum())
    out = dict(groups={la: dict(n=na), lb: dict(n=nb)})
    if min(na, nb) < N_MIN_GROUP:
        out["verdict_e"] = out["verdict_a"] = "UNDERPOWERED"
        out["reason"] = f"min group n = {min(na, nb)} < {N_MIN_GROUP}"
        return out
    un_ref = float(np.median(D["un"][mask_a | mask_b]))
    for m, l in ((mask_a, la), (mask_b, lb)):
        g = out["groups"][l]
        g["u_n_median"] = float(np.median(D["un"][m]))
        g["exp_fit"] = exp_fit(D["un"][m], D["e"][m])
        g["e_resid_median"] = float(np.median(D["res"][m]))
        g["e_at_matched_un"] = float(shipped_e(un_ref) + g["e_resid_median"])
        aa = D["a"][m]
        aa = aa[np.isfinite(aa)]
        g["n_a"], g["a_median"] = int(len(aa)), (float(np.median(aa)) if len(aa) else None)
    ga, gb = out["groups"][la], out["groups"][lb]
    d_med = ga["e_resid_median"] - gb["e_resid_median"]
    mw = stats.mannwhitneyu(D["res"][mask_a], D["res"][mask_b])
    sub = mask_a | mask_b
    out["e"] = dict(
        un_ref=un_ref, delta_median_resid=float(d_med),
        delta_ci95=boot_dmed(D["res"][mask_a], D["res"][mask_b]),
        mw_p=float(mw.pvalue),
        strat_perm_p_within_take=strat_perm_p(D["res"][sub], mask_a[sub],
                                              D["take"][sub]))
    # slope-form difference: bootstrap CI on g2_a - g2_b over the common u_n band
    dg2 = []
    ua, ea = D["un"][mask_a], D["e"][mask_a]
    ub, eb = D["un"][mask_b], D["e"][mask_b]
    for _ in range(600):
        ia, ib = rng.integers(0, na, na), rng.integers(0, nb, nb)
        fa, fb = exp_fit(ua[ia], ea[ia]), exp_fit(ub[ib], eb[ib])
        if fa["g2"] is not None and fb["g2"] is not None:
            dg2.append(fa["g2"] - fb["g2"])
    span = float(min(np.percentile(ua, 90), np.percentile(ub, 90))
                 - max(np.percentile(ua, 10), np.percentile(ub, 10)))
    if len(dg2) > 200 and ga["exp_fit"]["g2"] is not None and gb["exp_fit"]["g2"] is not None:
        ci = [float(np.percentile(dg2, 2.5)), float(np.percentile(dg2, 97.5))]
        pt = ga["exp_fit"]["g2"] - gb["exp_fit"]["g2"]
        out["dg2"] = dict(point=float(pt), ci95=ci, common_un_span=max(span, 0.0),
                          slope_form_de=float(abs(pt) * max(span, 0.0)
                                              * shipped_e(un_ref)))
    # verdict on e
    ci = out["e"]["delta_ci95"]
    dg2v = out.get("dg2")
    slope_diff = (dg2v and (dg2v["ci95"][0] * dg2v["ci95"][1] > 0)
                  and dg2v["slope_form_de"] > MATERIAL_E)
    if mw.pvalue < P_DIFF and abs(d_med) > MATERIAL_E:
        out["verdict_e"] = "DIFFERENT"
        out["reason_e"] = (f"median e-resid differs by {d_med:+.3f} "
                           f"(MW p={mw.pvalue:.1e}, within-take perm "
                           f"p={out['e']['strat_perm_p_within_take']:.4f})")
    elif slope_diff:
        out["verdict_e"] = "DIFFERENT"
        out["reason_e"] = (f"slope-form: dg2={dg2v['point']:+.4f} CI {dg2v['ci95']} "
                           f"excludes 0, de~{dg2v['slope_form_de']:.3f} over common band")
    elif mw.pvalue >= P_SAME and -BAND_E <= ci[0] and ci[1] <= BAND_E:
        out["verdict_e"] = "SAME-within-noise"
        out["reason_e"] = f"d_med={d_med:+.3f}, CI {ci} inside +-{BAND_E}"
    else:
        out["verdict_e"] = "UNDERPOWERED"
        out["reason_e"] = f"inconclusive: d_med={d_med:+.3f}, CI {ci}, MW p={mw.pvalue:.3f}"
    # verdict on a
    xa, xb = D["a"][mask_a], D["a"][mask_b]
    xa, xb = xa[np.isfinite(xa)], xb[np.isfinite(xb)]
    if min(len(xa), len(xb)) < N_MIN_GROUP:
        out["verdict_a"] = "UNDERPOWERED"
        out["a"] = dict(n=[int(len(xa)), int(len(xb))])
    else:
        mwa = stats.mannwhitneyu(xa, xb)
        da = float(np.median(xa) - np.median(xb))
        cia = boot_dmed(xa, xb)
        out["a"] = dict(delta_median=da, delta_ci95=cia, mw_p=float(mwa.pvalue))
        if mwa.pvalue < P_DIFF and abs(da) > MATERIAL_A:
            out["verdict_a"] = "DIFFERENT"
        elif mwa.pvalue >= P_SAME and -BAND_A <= cia[0] and cia[1] <= BAND_A:
            out["verdict_a"] = "SAME-within-noise"
        else:
            out["verdict_a"] = "UNDERPOWERED"
    return out


def blade_split(D):
    """e-residual vs in-plane offset |r_t| (from the RIGID-BODY ORIGIN, not the
    blade center — unknown constant offset, so only the trend is meaningful)."""
    rt, res = D["rt"], D["res"]
    n = len(rt)
    out = dict(n=n, rt_mm=dict(median=float(np.median(rt) * 1e3),
                               p10=float(np.percentile(rt, 10) * 1e3),
                               p90=float(np.percentile(rt, 90) * 1e3)))
    if n < 2 * N_MIN_GROUP:
        out["verdict_e"] = "UNDERPOWERED"
        out["reason"] = f"n={n} < {2 * N_MIN_GROUP}"
        return out
    ts = stats.theilslopes(res, rt)
    span = float(np.percentile(rt, 90) - np.percentile(rt, 10))
    perm = np.empty(N_PERM)
    for i in range(N_PERM):
        perm[i] = stats.theilslopes(res, rt[rng.permutation(n)])[0]
    p_perm = float((np.sum(np.abs(perm) >= abs(ts[0])) + 1) / (N_PERM + 1))
    eff = [stats.theilslopes(res[i], rt[i])[0] * span
           for i in (rng.integers(0, n, n) for _ in range(400))]
    ci = [float(np.percentile(eff, 2.5)), float(np.percentile(eff, 97.5))]
    # data-driven tercile bins (edges reported in mm)
    q = np.percentile(rt, [100 / 3, 200 / 3])
    bins = [rt < q[0], (rt >= q[0]) & (rt < q[1]), rt >= q[1]]
    out["bins_mm"] = [float(x * 1e3) for x in q]
    out["per_bin"] = [dict(n=int(b.sum()), e_resid_median=float(np.median(res[b])),
                           a_median=float(np.nanmedian(D["a"][b])))
                      for b in bins]
    mw = stats.mannwhitneyu(res[bins[0]], res[bins[2]])
    out["theil_sen"] = dict(slope_per_m=float(ts[0]), perm_p=p_perm,
                            effect_across_p10_p90=float(ts[0] * span),
                            effect_ci95=ci, span_m=span)
    out["mw_inner_vs_outer_tercile_p"] = float(mw.pvalue)
    if p_perm < P_DIFF and abs(ts[0] * span) > MATERIAL_E:
        out["verdict_e"] = "DIFFERENT"
    elif p_perm >= P_SAME and -BAND_E <= ci[0] and ci[1] <= BAND_E:
        out["verdict_e"] = "SAME-within-noise"
    else:
        out["verdict_e"] = "UNDERPOWERED"
    return out


def main():
    require(STRIKES, "strikes.json (stage1 output)")
    strikes = json.load(open(STRIKES))
    if not strikes:
        raise RuntimeError(f"strikes.json at {STRIKES!r} is empty; refusing to skip.")
    rows, n_gate, n_egate = build_rows(strikes)
    if len(rows) < 2 * N_MIN_GROUP:
        raise RuntimeError(f"only {len(rows)} gated strikes (< {2 * N_MIN_GROUP}); "
                           "not enough data for any split. Refusing to skip.")
    face_source, n_face_unresolved = resolve_faces(rows)

    D = dict(take=np.array([r["take"] for r in rows]),
             pad=np.array([r["paddle"] for r in rows]),
             face=np.array([r["face"] or "?" for r in rows]),
             un=np.array([r["u_n"] for r in rows]),
             e=np.array([r["e"] for r in rows]),
             a=np.array([r["a"] for r in rows]),
             rt=np.array([r["r_t"] for r in rows]))
    D["res"] = D["e"] - shipped_e(D["un"])

    splits = {"paddle": two_group(D, D["pad"] == "p1", D["pad"] == "p2", "p1", "p2")}
    for p in ("p1", "p2"):
        splits[f"face_{p}"] = two_group(
            D, (D["pad"] == p) & (D["face"] == "n+"),
            (D["pad"] == p) & (D["face"] == "n-"), f"{p}:n+", f"{p}:n-")
    splits["blade_position"] = blade_split(D)

    crosstab = {t: {p: int(((D["take"] == t) & (D["pad"] == p)).sum())
                    for p in ("p1", "p2")} for t in np.unique(D["take"])}
    n_extrap = int(np.sum((D["un"] < ENVELOPE_UN[0]) | (D["un"] > ENVELOPE_UN[1])))

    out = dict(
        test="F10: per-paddle / per-face / blade-position splits of the paddle model",
        face_source=face_source, n_face_unresolved=n_face_unresolved,
        n_strikes_total=len(strikes), n_gated_out=n_gate, n_e_outliers=n_egate,
        n_used=len(rows),
        thresholds=dict(material_e=MATERIAL_E, band_e=BAND_E, material_a=MATERIAL_A,
                        band_a=BAND_A, p_different=P_DIFF, p_same=P_SAME,
                        n_min_group=N_MIN_GROUP),
        shipped_model=dict(g1=SHIPPED_G1, g2=SHIPPED_G2, a_t=SHIPPED_AT),
        splits=splits, take_paddle_crosstab=crosstab,
        n_outside_un_envelope=n_extrap,
        caveats=[
            "|r_t| is measured from the paddle RIGID-BODY ORIGIN (pad_p), not the "
            "blade center: absolute offsets carry an unknown constant bias; only "
            "the e-resid-vs-|r_t| trend is meaningful",
            "paddle identity is partially confounded with take/player; the "
            "within-take stratified permutation p is reported next to MW",
            f"{n_extrap} strikes lie outside the fitted u_n envelope "
            f"{ENVELOPE_UN} m/s (extrapolation of the shipped reference curve)",
            "face labels n+/n- are relative to the mocap body normal channel, not "
            "to rubber color; pairing to physical faces needs a bench note",
        ],
        plot=f"{OUTDIR}/F10_paddle_split.png")

    os.makedirs(OUTDIR, exist_ok=True)
    with open(f"{OUTDIR}/F10_verdict.json", "w") as f:
        json.dump(out, f, indent=2)

    # ---- plot (2x2) ----
    fig, axes = plt.subplots(2, 2, figsize=(13.5, 10))
    xx = np.linspace(D["un"].min(), D["un"].max(), 120)
    ax = axes[0, 0]
    for p in ("p1", "p2"):
        m = D["pad"] == p
        ax.scatter(D["un"][m], D["e"][m], s=20, alpha=0.6, color=COL[p],
                   label=f"{p} (n={m.sum()})")
        gf = splits["paddle"]["groups"].get(p, {}).get("exp_fit")
        if gf and gf.get("g2") is not None:
            ax.plot(xx, gf["g1"] * np.exp(gf["g2"] * xx), color=COL[p], lw=2,
                    label=f"{p}: {gf['g1']:.3f}·e^({gf['g2']:+.4f}u)")
    ax.plot(xx, shipped_e(xx), "k--", lw=1.5, label="shipped 0.759·e^(-0.0441u)")
    ax.set_xlabel("|u_n| (m/s)"); ax.set_ylabel("per-strike e")
    ax.set_title(f"F10 by paddle -> {splits['paddle']['verdict_e']}")
    ax.legend(fontsize=8); ax.grid(alpha=0.3)

    ax = axes[0, 1]
    for p, mk in (("p1", "o"), ("p2", "^")):
        for fc in ("n+", "n-"):
            m = (D["pad"] == p) & (D["face"] == fc)
            if m.any():
                ax.scatter(D["un"][m], D["e"][m], s=22, alpha=0.65, marker=mk,
                           color=COL[fc], label=f"{p}:{fc} (n={m.sum()})")
    ax.plot(xx, shipped_e(xx), "k--", lw=1.5)
    ax.set_xlabel("|u_n| (m/s)"); ax.set_ylabel("per-strike e")
    ax.set_title(f"by face  (p1: {splits['face_p1']['verdict_e']}, "
                 f"p2: {splits['face_p2']['verdict_e']})")
    ax.legend(fontsize=8); ax.grid(alpha=0.3)

    ax = axes[1, 0]
    bl = splits["blade_position"]
    for p in ("p1", "p2"):
        m = D["pad"] == p
        ax.scatter(D["rt"][m] * 1e3, D["res"][m], s=20, alpha=0.6, color=COL[p], label=p)
    if "theil_sen" in bl:
        rr = np.linspace(D["rt"].min(), D["rt"].max(), 50)
        med = np.median(D["res"] - bl["theil_sen"]["slope_per_m"] * D["rt"])
        ax.plot(rr * 1e3, med + bl["theil_sen"]["slope_per_m"] * rr, "k-", lw=2,
                label=(f"TS {bl['theil_sen']['slope_per_m']:+.3f}/m, "
                       f"perm p={bl['theil_sen']['perm_p']:.3f}"))
    ax.axhline(0, color="gray", lw=0.8, ls="-.")
    ax.set_xlabel("|r_t| in-plane offset from body origin (mm)")
    ax.set_ylabel("e - shipped e(u_n)")
    ax.set_title(f"blade position -> {bl['verdict_e']}")
    ax.legend(fontsize=8); ax.grid(alpha=0.3)

    ax = axes[1, 1]
    grp_defs = [("p1", D["pad"] == "p1"), ("p2", D["pad"] == "p2")] + [
        (f"{p}:{fc}", (D["pad"] == p) & (D["face"] == fc))
        for p in ("p1", "p2") for fc in ("n+", "n-")]
    data = [D["a"][m][np.isfinite(D["a"][m])] for _, m in grp_defs]
    keep = [i for i, d in enumerate(data) if len(d) >= 5]
    if keep:
        bp = ax.boxplot([data[i] for i in keep], showfliers=False, patch_artist=True)
        ax.set_xticks(range(1, len(keep) + 1))          # mpl>=3.7 safe (no tick_labels)
        ax.set_xticklabels([grp_defs[i][0] for i in keep], fontsize=8)
        for box, i in zip(bp["boxes"], keep):
            key = grp_defs[i][0].split(":")[-1]
            box.set_facecolor(COL.get(key, "#888888")); box.set_alpha(0.55)
    ax.axhline(SHIPPED_AT, color="k", ls="--", lw=1.2, label=f"shipped a_t={SHIPPED_AT}")
    ax.set_ylabel("a_i = |dv_t| / |u_t|"); ax.set_title("tangential ratio by group")
    ax.legend(fontsize=8); ax.grid(alpha=0.3)

    fig.tight_layout()
    fig.savefig(out["plot"], dpi=140)

    # ---- compact stdout table ----
    print(f"F10  n_used={len(rows)} (gated {n_gate}, e-outliers {n_egate})  "
          f"face_source={face_source}  unresolved_faces={n_face_unresolved}")
    hdr = f"{'group':<8}{'n':>5}{'med u_n':>9}{'g1':>8}{'g2':>9}{'e@un*':>8}{'d_res':>8}{'med a':>8}"
    print(hdr); print("-" * len(hdr))
    nan = float("nan")
    for sp in ("paddle", "face_p1", "face_p2"):
        for l, g in splits[sp]["groups"].items():
            gf = g.get("exp_fit") or {}
            v = [g["n"], g.get("u_n_median", nan), gf.get("g1"), gf.get("g2"),
                 g.get("e_at_matched_un", nan), g.get("e_resid_median", nan),
                 g.get("a_median")]
            v = [nan if x is None else x for x in v]
            print(f"{l:<8}{v[0]:>5}{v[1]:>9.2f}{v[2]:>8.3f}{v[3]:>9.4f}"
                  f"{v[4]:>8.3f}{v[5]:>8.3f}{v[6]:>8.3f}")
    print()
    for sp in ("paddle", "face_p1", "face_p2"):
        s = splits[sp]
        print(f"{sp:<15} e: {s['verdict_e']:<18} ({s.get('reason_e', s.get('reason', ''))})")
        print(f"{'':<15} a: {s['verdict_a']}")
    bl = splits["blade_position"]
    ts_txt = (f"slope {bl['theil_sen']['slope_per_m']:+.3f}/m perm p="
              f"{bl['theil_sen']['perm_p']:.3f}" if "theil_sen" in bl else bl.get("reason", ""))
    print(f"{'blade_position':<15} e: {bl['verdict_e']:<18} ({ts_txt})")
    print(f"-> {OUTDIR}/F10_verdict.json ; {out['plot']}")


if __name__ == "__main__":
    main()
