#!/usr/bin/env python
"""F4 ADVERSARIAL VERIFICATION: re-derive paddle restitution e vs u_n from
segments/strikes.json (NOT from fits.paddle.per_strike) and try to break the
original KILL verdict (e falls with u_n).

Independent derivation per strike:
  n oriented against approach (u_in . n < 0)
  cp   = ball_p - R n
  v_r  = pad_v + pad_w x (cp - pad_p)
  u_in = v_in + w_in x (-R n) - v_r
  u_out= v_out + w_out x (-R n) - v_r
  e    = -(u_out.n)/(u_in.n),  u_n = -(u_in.n)

Own gates (justified in JSON), own estimators, and three break tests:
 (a) per-take & per-paddle slopes + leave-one-take-out
 (b) juggling (dianqiu_*) vs strike takes: between- vs within-group decomposition
 (c) tracking-quality confounds (|w_out|, fit_rms, gap_ms, meet_mm) vs e and u_n
Extra estimator immune to ratio-noise: u_out_n = a*u_in_n + b*u_in_n^2
(curvature b != 0 <=> velocity-dependent e).
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
from scipy.optimize import least_squares

R = 0.020
STRIKES = os.path.join(paths.SEGMENTS, "strikes.json")
FITS = os.path.join(paths.FITS, "stage2_fits_all.json")
OUTDIR = paths.FALSIFICATION
rng = np.random.default_rng(7)

strikes = json.load(open(STRIKES))
fits = json.load(open(FITS))

rows = []
for r in strikes:
    n = np.array(r["pad_n"], float)
    n = n / np.linalg.norm(n)
    ball_p = np.array(r["ball_p"], float)
    pad_p = np.array(r["pad_p"], float)
    pad_v = np.array(r["pad_v"], float)
    pad_w = np.array(r["pad_w"], float)
    v_in = np.array(r["v_in"], float)
    v_out = np.array(r["v_out"], float)
    w_in = np.array(r["w_in"], float)
    w_out = np.array(r["w_out"], float)

    def rel(nv):
        cp = ball_p - R * nv
        v_r = pad_v + np.cross(pad_w, cp - pad_p)
        u_i = v_in + np.cross(w_in, -R * nv) - v_r
        u_o = v_out + np.cross(w_out, -R * nv) - v_r
        return u_i, u_o

    u_i, u_o = rel(n)
    if np.dot(u_i, n) > 0:  # orient n against approach
        n = -n
        u_i, u_o = rel(n)
    uin_n = float(np.dot(u_i, n))          # < 0
    uout_n = float(np.dot(u_o, n))
    e = -uout_n / uin_n
    u_t_vec = u_i - uin_n * n
    rows.append(dict(
        take=r["take"], paddle=r["paddle"], t_c=r["t_c"],
        u_n=-uin_n, uout_n=uout_n, e=e, u_t=float(np.linalg.norm(u_t_vec)),
        w_out_mag=float(np.linalg.norm(w_out)), w_in_mag=float(np.linalg.norm(w_in)),
        pad_w_mag=float(np.linalg.norm(pad_w)),
        meet_mm=r["meet_mm"], fit_rms_mm=r["fit_rms_mm"],
        pad_fit_rms_mm=r["pad_fit_rms_mm"], gap_ms=r["gap_ms"],
        dist_m=r["dist_m"], u_n_theirs=r["u_n"]))

A = {k: np.array([rr[k] for rr in rows]) for k in rows[0]}
n_total = len(rows)

# cross-check my u_n against the pipeline's stored u_n (derivation agreement)
du = A["u_n"] - A["u_n_theirs"]
derivation_match = dict(median_abs_diff=float(np.median(np.abs(du))),
                        max_abs_diff=float(np.max(np.abs(du))))

# match my e to fits.paddle.per_strike e by (take, t_c)
fit_e = {}
for r in fits["paddle"]["per_strike"]:
    fit_e[(r["take"], round(r["t_c"], 4))] = r["e"]
matched, ediff = 0, []
for rr in rows:
    k = (rr["take"], round(rr["t_c"], 4))
    if k in fit_e:
        matched += 1
        ediff.append(rr["e"] - fit_e[k])
ediff = np.array(ediff)
e_match = dict(n_matched=matched,
               median_abs_diff=float(np.median(np.abs(ediff))) if matched else None,
               max_abs_diff=float(np.max(np.abs(ediff))) if matched else None)

# ---------------- MY quality gates (physics/quality based, not e-based) -------
# g1: u_n >= 1.2 m/s  -> e = ratio of noisy velocities; sigma_v ~0.15-0.3 m/s at
#     contact extrapolation, so u_n < 1.2 gives e noise > ~0.25 (unusable).
# g2: meet_mm <= 40   -> ball-paddle surface mismatch at contact; > 40 mm means
#     wrong pairing or heavy extrapolation of one body.
# g3: pad_fit_rms_mm <= 2.0 -> paddle rigid-body solve trustworthy (n from it).
# g4: gap_ms <= 250   -> velocities are extrapolated across the contact gap;
#     beyond ~250 ms the polynomial extrapolation dominates the estimate.
# NO gate on e itself (gating the dependent variable can sculpt the trend);
# instead report the few wild e values and run a sensitivity check.
g = ((A["u_n"] >= 1.2) & (A["meet_mm"] <= 40.0)
     & (A["pad_fit_rms_mm"] <= 2.0) & (A["gap_ms"] <= 250.0))
gate_counts = dict(
    total=n_total,
    fail_u_n=int(np.sum(A["u_n"] < 1.2)),
    fail_meet=int(np.sum(A["meet_mm"] > 40.0)),
    fail_padfit=int(np.sum(A["pad_fit_rms_mm"] > 2.0)),
    fail_gap=int(np.sum(A["gap_ms"] > 250.0)),
    kept=int(np.sum(g)))

un, e, tk, pd_ = A["u_n"][g], A["e"][g], A["take"][g], A["paddle"][g]
uon = A["uout_n"][g]
wout = A["w_out_mag"][g]; frms = A["fit_rms_mm"][g]; gap = A["gap_ms"][g]
meet = A["meet_mm"][g]; pwm = A["pad_w_mag"][g]
n_used = len(e)
wild = int(np.sum((e < 0) | (e > 1.3)))

# ---------------- estimators ----------------
ts = stats.theilslopes(e, un, 0.95)
ols = stats.linregress(un, e)


def huber_lin(x, y):
    r0 = least_squares(lambda p: p[0] + p[1] * x - y,
                       x0=[np.median(y), 0.0], loss="huber",
                       f_scale=max(np.std(y), 0.05))
    return r0.x  # intercept, slope


hb_i, hb_s = huber_lin(un, e)

# WLS with weights ~ u_n^2 (e noise ~ sigma/u_n -> var ~ 1/u_n^2)
w = un ** 2
W = np.diag(w)
X = np.column_stack([np.ones_like(un), un])
beta = np.linalg.solve(X.T @ W @ X, X.T @ W @ e)
res_w = e - X @ beta
s2 = float(np.sum(w * res_w ** 2) / (n_used - 2))
cov = s2 * np.linalg.inv(X.T @ W @ X)
wls_slope, wls_se = float(beta[1]), float(np.sqrt(cov[1, 1]))

# ratio-noise-immune: u_out_n = a*u_in_n + b*u_in_n^2  (u_in_n = -u_n here,
# use magnitudes: uon = a*un + b*un^2, a=e at 0, de/dun = b)
Q = np.column_stack([un, un ** 2])
ab, *_ = np.linalg.lstsq(Q, uon, rcond=None)
res_q = uon - Q @ ab
covq = float(np.sum(res_q ** 2) / (n_used - 2)) * np.linalg.inv(Q.T @ Q)
quad_a, quad_b = float(ab[0]), float(ab[1])
quad_b_se = float(np.sqrt(covq[1, 1]))

# cluster (take) bootstrap for pooled TS + WLS slopes
uts = np.unique(tk)
B = 4000
bs_ts, bs_wls = [], []
for _ in range(B):
    pick = rng.choice(uts, len(uts), replace=True)
    xs, ys = [], []
    for t in pick:
        m = tk == t
        xs.append(un[m]); ys.append(e[m])
    x = np.concatenate(xs); y = np.concatenate(ys)
    if np.ptp(x) < 1.5:
        continue
    bs_ts.append(stats.theilslopes(y, x)[0])
    ww = x ** 2
    Xb = np.column_stack([np.ones_like(x), x])
    try:
        bb = np.linalg.solve(Xb.T @ (ww[:, None] * Xb), Xb.T @ (ww * y))
        bs_wls.append(bb[1])
    except np.linalg.LinAlgError:
        pass
ts_cl_ci = np.percentile(bs_ts, [2.5, 97.5]).tolist()
wls_cl_ci = np.percentile(bs_wls, [2.5, 97.5]).tolist()

# ---------------- (a) per-take, per-paddle, leave-one-take-out ---------------
per_take = {}
for t in uts:
    m = tk == t
    if m.sum() >= 6 and np.ptp(un[m]) >= 0.8:
        o = stats.linregress(un[m], e[m])
        tss = stats.theilslopes(e[m], un[m])
        per_take[t] = dict(n=int(m.sum()), un_lo=float(un[m].min()),
                           un_hi=float(un[m].max()),
                           ols_slope=float(o.slope), ols_p=float(o.pvalue),
                           ts_slope=float(tss[0]),
                           med_e=float(np.median(e[m])))
per_paddle = {}
for p in ["p1", "p2"]:
    m = pd_ == p
    o = stats.linregress(un[m], e[m])
    tss = stats.theilslopes(e[m], un[m])
    per_paddle[p] = dict(n=int(m.sum()), ols_slope=float(o.slope),
                         ols_p=float(o.pvalue), ts_slope=float(tss[0]),
                         un_range=[float(un[m].min()), float(un[m].max())],
                         med_e=float(np.median(e[m])))
loto = {}
for t in uts:
    m = tk != t
    loto[t] = dict(ts=float(stats.theilslopes(e[m], un[m])[0]),
                   ols=float(stats.linregress(un[m], e[m]).slope), n=int(m.sum()))

# within-take (fixed-effects) slope
und = un.copy(); ed = e.copy()
for t in uts:
    m = tk == t
    und[m] -= un[m].mean(); ed[m] -= e[m].mean()
fe_ols = stats.linregress(und, ed)
fe_ts = stats.theilslopes(ed, und)

# ---------------- (b) juggling vs strike takes -------------------------------
is_jug = np.array([t.startswith("dianqiu") for t in tk])
grp = {}
for name, m in [("juggling", is_jug), ("strike", ~is_jug)]:
    o = stats.linregress(un[m], e[m])
    tss = stats.theilslopes(e[m], un[m])
    grp[name] = dict(n=int(m.sum()), un_range=[float(un[m].min()), float(un[m].max())],
                     med_e=float(np.median(e[m])), ols_slope=float(o.slope),
                     ols_p=float(o.pvalue), ts_slope=float(tss[0]))
# take-level: does the pooled slope survive using only take medians?
tmed_u = np.array([np.median(un[tk == t]) for t in uts])
tmed_e = np.array([np.median(e[tk == t]) for t in uts])
tl = stats.linregress(tmed_u, tmed_e)
# overlap test: strike takes only, u_n >= 3 (removes the juggling cluster)
m_ov = (~is_jug) & (un >= 3.0)
ov = (stats.linregress(un[m_ov], e[m_ov]) if m_ov.sum() > 10 else None)

# ---------------- (c) quality confounds --------------------------------------
def pcorr(x, y, z):
    rx = x - np.polyval(np.polyfit(z, x, 1), z)
    ry = y - np.polyval(np.polyfit(z, y, 1), z)
    return stats.pearsonr(rx, ry)

conf = {}
for nm, q in [("w_out_mag", wout), ("fit_rms_mm", frms), ("gap_ms", gap),
              ("meet_mm", meet), ("pad_w_mag", pwm)]:
    conf[nm] = dict(
        r_e_q=list(map(float, stats.pearsonr(e, q))),
        r_un_q=list(map(float, stats.pearsonr(un, q))),
        r_e_un_given_q=list(map(float, pcorr(e, un, q))))
# spin quality proxy: w_out > 75 rev/s (=471 rad/s) untrusted per notes
hi_w = wout > 75 * 2 * np.pi
conf["n_wout_above_75revs"] = int(hi_w.sum())

# sensitivity: also apply the original e-range gate on top of mine
m_sens = (e > -0.2) & (e < 1.5)
sens_ts = float(stats.theilslopes(e[m_sens], un[m_sens])[0])
sens_ols = float(stats.linregress(un[m_sens], e[m_sens]).slope)
# tight-quality subset (strictest gates)
m_tight = (meet <= 20) & (gap <= 120) & (frms <= 6)
tight = dict(n=int(m_tight.sum()),
             ts=float(stats.theilslopes(e[m_tight], un[m_tight])[0])
             if m_tight.sum() > 15 else None,
             ols_slope=float(stats.linregress(un[m_tight], e[m_tight]).slope)
             if m_tight.sum() > 15 else None,
             ols_p=float(stats.linregress(un[m_tight], e[m_tight]).pvalue)
             if m_tight.sum() > 15 else None)

# group medians test (like original)
lo_m, hi_m = e[un < 4.5], e[un >= 5.0]
mw = stats.mannwhitneyu(lo_m, hi_m, alternative="greater") if len(hi_m) >= 5 else None

# ---------------- ensemble & verdict ------------------------------------------
d6 = dict(theil_sen=ts[0] * 6, ols=ols.slope * 6, huber=hb_s * 6,
          wls_un2=wls_slope * 6, quad_b=quad_b * 6,
          within_take_ols=fe_ols.slope * 6, within_take_ts=fe_ts[0] * 6)
med6 = float(np.median(list(d6.values())))

# my agreement call
neg_all = all(v < 0 for v in d6.values())
wls_sig = wls_slope + 1.96 * wls_se < 0
quad_sig = quad_b + 1.96 * quad_b_se < 0
cl_excl0 = ts_cl_ci[1] < 0 or wls_cl_ci[1] < 0
survives_take = (grp["strike"]["ols_slope"] < 0) and all(
    v["ts"] < 0 for v in loto.values())

# ---------------- plot ---------------------------------------------------------
fig, axes = plt.subplots(1, 3, figsize=(17, 5.4))
ax = axes[0]
for m, lab, c in [(is_jug, "juggling (dianqiu_*)", "tab:orange"),
                  (~is_jug, "strike takes", "tab:blue")]:
    ax.scatter(un[m], e[m], s=20, alpha=0.65, c=c, label=f"{lab} n={m.sum()}")
xx = np.linspace(un.min(), un.max(), 100)
ax.plot(xx, ts[1] + ts[0] * xx, "r-", lw=2, label=f"TS {ts[0]:+.4f}/m/s")
ax.plot(xx, beta[0] + beta[1] * xx, "k--", lw=2, label=f"WLS(u_n^2) {wls_slope:+.4f}")
ax.plot(xx, (quad_a + quad_b * xx), "g:", lw=2,
        label=f"e(u)=a+b·u from quad: b={quad_b:+.4f}")
ax.set_xlabel("u_n (m/s)"); ax.set_ylabel("e (my derivation)")
ax.set_title(f"F4 adversarial re-derivation n={n_used}/{n_total}\n"
             f"median Δe(2-8)={med6:+.3f}")
ax.legend(fontsize=7); ax.grid(alpha=0.3)

ax = axes[1]
ax.scatter(un, uon, s=18, alpha=0.6, c=np.where(is_jug, "tab:orange", "tab:blue"))
ax.plot(xx, quad_a * xx + quad_b * xx ** 2, "g-", lw=2,
        label=f"u_out={quad_a:.3f}u{quad_b:+.4f}u² (b se {quad_b_se:.4f})")
ax.plot(xx, np.median(e) * xx, "k--", lw=1, label=f"const e={np.median(e):.3f}")
ax.set_xlabel("u_in normal (m/s)"); ax.set_ylabel("u_out normal (m/s)")
ax.set_title("ratio-noise-immune fit: curvature ⇔ e(u_n)")
ax.legend(fontsize=8); ax.grid(alpha=0.3)

ax = axes[2]
names = list(per_take.keys())
sl = [per_take[t]["ols_slope"] for t in names]
ax.barh(range(len(names)), sl, color=["tab:orange" if t.startswith("dianqiu")
                                      else "tab:blue" for t in names])
ax.set_yticks(range(len(names)))
ax.set_yticklabels([f"{t} (n={per_take[t]['n']})" for t in names], fontsize=8)
ax.axvline(0, color="k", lw=1)
ax.axvline(-0.018, color="r", ls="--", lw=1, label="lit -0.018/m/s")
ax.set_xlabel("per-take OLS slope de/du_n")
ax.set_title(f"per-take slopes | p1 {per_paddle['p1']['ols_slope']:+.4f}, "
             f"p2 {per_paddle['p2']['ols_slope']:+.4f}\n"
             f"strike-only(u_n≥3) slope "
             f"{ov.slope:+.4f} (p={ov.pvalue:.3f})" if ov else "per-take slopes")
ax.legend(fontsize=8); ax.grid(alpha=0.3, axis="x")

fig.tight_layout()
fig.savefig(f"{OUTDIR}/F4_adv.png", dpi=140)

out = dict(
    test="F4 adversarial: re-derived e vs u_n from strikes.json",
    derivation_match_u_n=derivation_match,
    e_match_vs_fits=e_match,
    gates=gate_counts, n_used=n_used, wild_e_kept=wild,
    pooled=dict(theil_sen=dict(slope=float(ts[0]), ci=[float(ts[2]), float(ts[3])],
                               cluster_ci=ts_cl_ci),
                ols=dict(slope=float(ols.slope), p=float(ols.pvalue),
                         se=float(ols.stderr)),
                huber=float(hb_s),
                wls_un2=dict(slope=wls_slope, se=wls_se, cluster_ci=wls_cl_ci),
                quad_uout=dict(a=quad_a, b=quad_b, b_se=quad_b_se)),
    within_take=dict(ols=float(fe_ols.slope), ols_p=float(fe_ols.pvalue),
                     ts=float(fe_ts[0])),
    delta_e_2to8=d6 | dict(median=med6),
    break_a_per_take=per_take, break_a_per_paddle=per_paddle,
    break_a_leave_one_take_out=loto,
    break_b_groups=grp,
    break_b_take_level=dict(slope=float(tl.slope), p=float(tl.pvalue),
                            n_takes=len(uts)),
    break_b_strike_only_un_ge3=(dict(slope=float(ov.slope), p=float(ov.pvalue),
                                     n=int(m_ov.sum())) if ov else None),
    break_c_confounds=conf,
    sensitivity=dict(with_original_e_gate=dict(ts=sens_ts, ols=sens_ols,
                                               n=int(m_sens.sum())),
                     tight_quality=tight),
    group_test=(dict(med_lo=float(np.median(lo_m)), med_hi=float(np.median(hi_m)),
                     n_lo=int(len(lo_m)), n_hi=int(len(hi_m)),
                     p=float(mw.pvalue)) if mw else None),
    flags=dict(all_estimators_negative=bool(neg_all), wls_sig=bool(wls_sig),
               quad_sig=bool(quad_sig), cluster_ci_excludes_zero=bool(cl_excl0),
               survives_take_structure=bool(survives_take)),
    plot=f"{OUTDIR}/F4_adv.png",
)
with open(f"{OUTDIR}/F4_adv_verdict.json", "w") as f:
    json.dump(out, f, indent=2, default=float)
print(json.dumps(out, indent=2, default=float))
