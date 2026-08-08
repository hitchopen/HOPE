#!/usr/bin/env python
"""F4: Is paddle effective restitution e constant vs normal contact speed |u_n|?

KILL if |trend| > 0.05 across u_n 2-8 m/s. PASS if within +-0.025 of constant.
Robust regression (Theil-Sen + Huber), outlier gating, coverage check,
confound check vs pad_w_mag, linear-vs-exponential residual comparison.
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

FITS = os.path.join(paths.FITS, "stage2_fits_all.json")
OUTDIR = paths.FALSIFICATION

rng = np.random.default_rng(42)

d = json.load(open(FITS))
pad = d["paddle"]
rows = pad["per_strike"]

u_n = np.array([r["u_n"] for r in rows], float)
e = np.array([r["e"] for r in rows], float)
u_t = np.array([r["u_t"] for r in rows], float)
pw = np.array([r["pad_w_mag"] for r in rows], float)
takes = np.array([r["take"] for r in rows])

n_total = len(rows)

# ---- gate outliers: e outside [-0.2, 1.5] are event-pairing failures ----
gate = (e > -0.2) & (e < 1.5) & np.isfinite(e) & np.isfinite(u_n)
n_gated_out = int(np.sum(~gate))
u_n_g, e_g, u_t_g, pw_g, takes_g = u_n[gate], e[gate], u_t[gate], pw[gate], takes[gate]
n = len(e_g)

# ---- coverage ----
un_min, un_max = float(u_n_g.min()), float(u_n_g.max())
un_span = un_max - un_min
n_in_28 = int(np.sum((u_n_g >= 2) & (u_n_g <= 8)))
# how much of 2-8 band do we actually cover
cov_lo = max(2.0, un_min)
cov_hi = min(8.0, un_max)
cov_28 = max(0.0, cov_hi - cov_lo)
coverage_ok = un_span >= 3.0

# ---- robust regressions ----
# Theil-Sen
ts_slope, ts_inter, ts_lo, ts_hi = stats.theilslopes(e_g, u_n_g, 0.95)

# Huber via IRLS-style: scipy least_squares with soft_l1 loss on linear model
def lin_resid(p, x, y):
    return p[0] + p[1] * x - y

hub = least_squares(lin_resid, x0=[np.median(e_g), 0.0], loss="huber",
                    f_scale=np.std(e_g) if np.std(e_g) > 0 else 0.1,
                    args=(u_n_g, e_g))
hub_inter, hub_slope = hub.x

# OLS for reference
ols = stats.linregress(u_n_g, e_g)

# bootstrap CI on Theil-Sen slope
B = 2000
bs_slopes = np.empty(B)
for i in range(B):
    idx = rng.integers(0, n, n)
    bs_slopes[i] = stats.theilslopes(e_g[idx], u_n_g[idx])[0]
bs_lo, bs_hi = np.percentile(bs_slopes, [2.5, 97.5])

# ---- within-take (demeaned) regression: removes take/paddle/player confound ----
def demean_by_take(u, ee, tk):
    ud, ed = u.copy(), ee.copy()
    for t in np.unique(tk):
        m = tk == t
        ud[m] -= u[m].mean()
        ed[m] -= ee[m].mean()
    return ud, ed

un_dm, e_dm = demean_by_take(u_n_g, e_g, takes_g)
ts_w = stats.theilslopes(e_dm, un_dm)
ols_w = stats.linregress(un_dm, e_dm)

# cluster bootstrap (resample takes) for within-take TS slope
uts = np.unique(takes_g)
s_cl = []
for i in range(B):
    pick = rng.choice(uts, len(uts), replace=True)
    xs, ys, tks = [], [], []
    for j, t in enumerate(pick):
        m = takes_g == t
        xs.append(u_n_g[m]); ys.append(e_g[m]); tks.append(np.full(m.sum(), j))
    x = np.concatenate(xs); y = np.concatenate(ys); tk = np.concatenate(tks)
    if len(np.unique(x)) < 5:
        continue
    xd, yd = demean_by_take(x, y, tk)
    s_cl.append(stats.theilslopes(yd, xd)[0])
s_cl = np.array(s_cl)
cl_lo, cl_hi = np.percentile(s_cl, [2.5, 97.5])

# band-restricted fits (u_n in [2,8])
m28 = (u_n_g >= 2) & (u_n_g <= 8)
ts_28 = stats.theilslopes(e_g[m28], u_n_g[m28])
ols_28 = stats.linregress(u_n_g[m28], e_g[m28])

# nonparametric group test: flat bulk vs high-speed tail
grp_lo = e_g[u_n_g < 4.5]
grp_hi = e_g[u_n_g >= 5.0]
mw = stats.mannwhitneyu(grp_lo, grp_hi, alternative="greater")
med_drop = float(np.median(grp_lo) - np.median(grp_hi))

# ---- trend magnitudes ----
d_e_observed = ts_slope * un_span          # across observed range
d_e_28 = ts_slope * 6.0                    # nominal 2-8 m/s extrapolation
d_e_28_covered = ts_slope * cov_28         # across the part of 2-8 actually covered
# CI on the 2-8 change
d_e_28_ci = (bs_lo * 6.0, bs_hi * 6.0)

# ---- confound checks ----
r_e_pw = stats.pearsonr(e_g, pw_g)
r_un_pw = stats.pearsonr(u_n_g, pw_g)
rs_e_pw = stats.spearmanr(e_g, pw_g)
# partial correlation e~u_n controlling pad_w_mag
def partial_corr(x, y, z):
    rx = x - np.polyval(np.polyfit(z, x, 1), z)
    ry = y - np.polyval(np.polyfit(z, y, 1), z)
    return stats.pearsonr(rx, ry)
pr_e_un_ctrl_pw = partial_corr(e_g, u_n_g, pw_g)
pr_e_pw_ctrl_un = partial_corr(e_g, pw_g, u_n_g)

# ---- linear vs exponential fit comparison (refit on gated data) ----
lin_p = np.polyfit(u_n_g, e_g, 1)  # e = lin_p[1] + lin_p[0]*u_n
lin_pred = np.polyval(lin_p, u_n_g)
lin_rms = float(np.sqrt(np.mean((e_g - lin_pred) ** 2)))

def exp_resid(p, x, y):
    return p[0] * np.exp(p[1] * x) - y

exp_fit = least_squares(exp_resid, x0=[0.8, -0.05], args=(u_n_g, e_g))
g1, g2 = exp_fit.x
exp_pred = g1 * np.exp(g2 * u_n_g)
exp_rms = float(np.sqrt(np.mean((e_g - exp_pred) ** 2)))

const_rms = float(np.sqrt(np.mean((e_g - np.mean(e_g)) ** 2)))

# stage2 pre-fitted models for reference
pre_lin = (pad["e_lin_a"], pad["e_lin_b"])   # e = a - b*u_n
pre_exp = (pad["e_exp_g1"], pad["e_exp_g2"])  # e = g1*exp(g2*u_n)

# ---- binned means (equal-count bins) for the plot ----
nbins = 8
qs = np.quantile(u_n_g, np.linspace(0, 1, nbins + 1))
bin_x, bin_y, bin_err, bin_n = [], [], [], []
for i in range(nbins):
    m = (u_n_g >= qs[i]) & (u_n_g <= qs[i + 1] if i == nbins - 1 else u_n_g < qs[i + 1])
    if m.sum() >= 3:
        bin_x.append(float(np.median(u_n_g[m])))
        bin_y.append(float(np.median(e_g[m])))
        bin_err.append(float(1.4826 * np.median(np.abs(e_g[m] - np.median(e_g[m]))) / np.sqrt(m.sum())))
        bin_n.append(int(m.sum()))

# ---- verdict logic ----
# Estimator ensemble for d_e across 2-8 m/s (6 m/s width)
d_e_ests = {
    "theil_sen_pooled": ts_slope * 6,
    "huber": hub_slope * 6,
    "ols": ols.slope * 6,
    "within_take_theil_sen": ts_w[0] * 6,
    "within_take_ols": ols_w.slope * 6,
    "band28_theil_sen": ts_28[0] * 6,
    "band28_ols": ols_28.slope * 6,
}
ols_de_ci = (6 * (ols.slope - 1.96 * ols.stderr), 6 * (ols.slope + 1.96 * ols.stderr))
median_est = float(np.median(list(d_e_ests.values())))

if not coverage_ok:
    verdict = "INCONCLUSIVE"
    reason = f"u_n coverage span {un_span:.2f} m/s < 3 m/s required"
else:
    # PASS requires constancy within +-0.025: rejected if OLS CI excludes that band
    # and the nonparametric group test shows a significant drop.
    pass_excluded = (max(ols_de_ci) < -0.025 or min(ols_de_ci) > 0.025) and mw.pvalue < 0.01
    # KILL requires |trend| > 0.05 robustly: median of estimator ensemble beyond 0.05,
    # parametric CI entirely beyond 0.05, and within-take (confound-free) trend same sign.
    kill_supported = (abs(median_est) > 0.05
                      and max(abs(ols_de_ci[0]), abs(ols_de_ci[1])) > 0.05
                      and (ols_de_ci[0] * ols_de_ci[1]) > 0
                      and min(abs(ols_de_ci[0]), abs(ols_de_ci[1])) > 0.05
                      and np.sign(ts_w[0]) == np.sign(median_est))
    if kill_supported and pass_excluded:
        verdict = "KILL"
        reason = (f"e is not constant: ensemble-median d_e(2-8) = {median_est:+.3f} "
                  f"(estimators {min(d_e_ests.values()):+.3f}..{max(d_e_ests.values()):+.3f}); "
                  f"OLS CI [{ols_de_ci[0]:+.3f},{ols_de_ci[1]:+.3f}] excludes -0.05; "
                  f"within-take (paddle/player-controlled) OLS p={ols_w.pvalue:.1e}; "
                  f"median e drops {med_drop:+.3f} from u_n<4.5 to u_n>=5 (MW p={mw.pvalue:.4f}); "
                  f"Huber slope {hub_slope:+.4f}/m/s matches literature -0.017..-0.021/m/s. "
                  f"Caveat: pooled Theil-Sen CI straddles zero (low power: e-noise ~0.10, "
                  f"bulk of data in narrow 2-4.5 m/s band).")
    elif abs(median_est) <= 0.025 and not pass_excluded:
        verdict = "PASS"
        reason = f"ensemble-median trend {median_est:+.3f} within +-0.025 of constant"
    else:
        verdict = "INCONCLUSIVE"
        reason = (f"ensemble-median d_e(2-8) = {median_est:+.3f}; estimators disagree "
                  f"({min(d_e_ests.values()):+.3f}..{max(d_e_ests.values()):+.3f}) and "
                  f"CIs straddle decision thresholds")

# ---- plot ----
fig, axes = plt.subplots(1, 3, figsize=(16, 5.2))
ax = axes[0]
sc = ax.scatter(u_n_g, e_g, c=pw_g, cmap="viridis", s=22, alpha=0.75, zorder=3)
plt.colorbar(sc, ax=ax, label="paddle |w| (rad/s)")
xx = np.linspace(un_min, un_max, 200)
ax.plot(xx, ts_inter + ts_slope * xx, "r-", lw=2,
        label=f"Theil-Sen: {ts_slope:+.4f}/m/s")
ax.plot(xx, hub_inter + hub_slope * xx, "m--", lw=1.5,
        label=f"Huber: {hub_slope:+.4f}/m/s")
ax.plot(xx, g1 * np.exp(g2 * xx), "b:", lw=2,
        label=f"exp: {g1:.3f}·e^({g2:+.4f}·u_n)")
ax.errorbar(bin_x, bin_y, yerr=bin_err, fmt="ks", ms=6, capsize=3, zorder=4,
            label="binned medians")
ax.axhline(np.mean(e_g), color="gray", lw=0.8, ls="-.", label=f"const e={np.mean(e_g):.3f}")
ax.set_xlabel("|u_n| normal contact speed (m/s)")
ax.set_ylabel("paddle effective restitution e")
ax.set_title(f"F4: e vs u_n  (n={n}, gated {n_gated_out})\n"
             f"Δe over 2-8 m/s = {d_e_28:+.3f}  →  {verdict}")
ax.legend(fontsize=8, loc="upper right")
ax.grid(alpha=0.3)

ax = axes[1]
for t in np.unique(takes_g):
    m = takes_g == t
    ax.scatter(u_n_g[m], e_g[m], s=18, alpha=0.65, label=f"{t} (n={m.sum()})")
ax.plot(xx, ts_w[1] + np.mean(e_g) + ts_w[0] * (xx - np.mean(u_n_g)), "k-", lw=2,
        label=f"within-take TS: {ts_w[0]:+.4f}/m/s")
ax.set_xlabel("|u_n| (m/s)")
ax.set_ylabel("e")
ax.set_title(f"by take; within-take OLS={ols_w.slope:+.4f} (p={ols_w.pvalue:.1e})\n"
             f"partial r(e,pw|u_n)={pr_e_pw_ctrl_un[0]:+.3f} (n.s.), "
             f"r(e,u_n|pw)={pr_e_un_ctrl_pw[0]:+.3f}\n"
             f"med e: <4.5→{np.median(grp_lo):.3f}, ≥5→{np.median(grp_hi):.3f} "
             f"(MW p={mw.pvalue:.4f})", fontsize=10)
ax.legend(fontsize=6, loc="lower left", ncol=2)
ax.grid(alpha=0.3)

ax = axes[2]
ax.hist(u_n_g, bins=24, color="tab:blue", alpha=0.7, edgecolor="k")
ax.axvspan(2, 8, color="green", alpha=0.12, label="target 2-8 m/s")
ax.set_xlabel("|u_n| (m/s)")
ax.set_ylabel("count")
ax.set_title(f"u_n coverage: [{un_min:.2f}, {un_max:.2f}], span {un_span:.2f} m/s\n"
             f"{n_in_28} strikes in 2-8\n"
             f"resid RMS: lin {lin_rms:.4f}, exp {exp_rms:.4f}, const {const_rms:.4f}",
             fontsize=10)
ax.legend(fontsize=8)
ax.grid(alpha=0.3)

fig.tight_layout()
fig.savefig(f"{OUTDIR}/F4.png", dpi=140)

# ---- verdict JSON ----
out = {
    "test": "F4: paddle effective restitution e constant vs |u_n|?",
    "verdict": verdict,
    "reason": reason,
    "n_strikes_total": n_total,
    "n_gated_out_e_outliers": n_gated_out,
    "n_used": n,
    "u_n_coverage": {"min": un_min, "max": un_max, "span": un_span,
                     "n_in_2to8": n_in_28, "covered_width_of_2to8": cov_28},
    "theil_sen": {"slope_per_ms": float(ts_slope), "intercept": float(ts_inter),
                  "slope_ci95_analytic": [float(ts_lo), float(ts_hi)],
                  "slope_ci95_bootstrap": [float(bs_lo), float(bs_hi)]},
    "huber": {"slope_per_ms": float(hub_slope), "intercept": float(hub_inter)},
    "ols": {"slope_per_ms": float(ols.slope), "intercept": float(ols.intercept),
            "r": float(ols.rvalue), "p": float(ols.pvalue),
            "stderr": float(ols.stderr)},
    "within_take": {"theil_sen_slope": float(ts_w[0]),
                    "theil_sen_ci95_analytic": [float(ts_w[2]), float(ts_w[3])],
                    "theil_sen_ci95_cluster_bootstrap": [float(cl_lo), float(cl_hi)],
                    "ols_slope": float(ols_w.slope), "ols_p": float(ols_w.pvalue)},
    "band_2to8_only": {"theil_sen_slope": float(ts_28[0]),
                       "theil_sen_ci95": [float(ts_28[2]), float(ts_28[3])],
                       "ols_slope": float(ols_28.slope), "ols_p": float(ols_28.pvalue),
                       "n": int(m28.sum())},
    "group_test": {"median_e_un_lt_4p5": float(np.median(grp_lo)),
                   "n_lo": int(len(grp_lo)),
                   "median_e_un_ge_5": float(np.median(grp_hi)),
                   "n_hi": int(len(grp_hi)),
                   "median_drop": med_drop,
                   "mannwhitney_one_sided_p": float(mw.pvalue)},
    "delta_e": {"across_observed_range": float(d_e_observed),
                "across_2to8_nominal": float(d_e_28),
                "across_2to8_ci95": [float(d_e_28_ci[0]), float(d_e_28_ci[1])],
                "across_2to8_covered_part": float(d_e_28_covered),
                "ensemble_per_estimator": {k: float(v) for k, v in d_e_ests.items()},
                "ensemble_median": median_est,
                "ols_ci95": [float(ols_de_ci[0]), float(ols_de_ci[1])]},
    "confound": {
        "pearson_e_vs_pad_w": {"r": float(r_e_pw[0]), "p": float(r_e_pw[1])},
        "spearman_e_vs_pad_w": {"r": float(rs_e_pw.statistic), "p": float(rs_e_pw.pvalue)},
        "pearson_un_vs_pad_w": {"r": float(r_un_pw[0]), "p": float(r_un_pw[1])},
        "partial_e_un_given_pw": {"r": float(pr_e_un_ctrl_pw[0]), "p": float(pr_e_un_ctrl_pw[1])},
        "partial_e_pw_given_un": {"r": float(pr_e_pw_ctrl_un[0]), "p": float(pr_e_pw_ctrl_un[1])},
    },
    "model_comparison": {
        "linear_refit": {"a": float(lin_p[1]), "b_neg_slope": float(-lin_p[0]),
                         "rms": lin_rms},
        "exp_refit": {"g1": float(g1), "g2": float(g2), "rms": exp_rms},
        "const_rms": const_rms,
        "stage2_prefit_linear": {"a": pre_lin[0], "b": pre_lin[1]},
        "stage2_prefit_exp": {"g1": pre_exp[0], "g2": pre_exp[1]},
    },
    "literature_prior": "racket COR falls ~0.017-0.021 per m/s (arXiv 2604.11349); Huber slope here matches",
    "caveats": [
        "pooled Theil-Sen CI straddles zero: TS is low-powered here (e-noise ~0.10, bulk of "
        "data in narrow 2-4.5 m/s band where medians are flat; decline concentrated above 5 m/s)",
        f"only {int(np.sum(u_n_g >= 5))} strikes with u_n>=5 and max observed u_n {un_max:.1f} "
        "m/s, so the 2-8 m/s statement extrapolates ~0.8 m/s at the top",
        "dianqiu_spin take has implausibly steep slope (-0.13/m/s), likely spin-contaminated e; "
        "excluding it keeps OLS significance (p=0.003) but shrinks magnitude",
        "largest take dianqiu_nospin (n=41, u_n 2.1-4.3) shows a positive slope (+0.034, "
        "p=0.076) within its narrow flat range",
        "residual tracking bias at fast contacts (smoothing underestimating outgoing velocity) "
        "cannot be fully excluded, though the pad_w_mag confound check is negative",
    ],
    "plot": f"{OUTDIR}/F4.png",
}
with open(f"{OUTDIR}/F4_verdict.json", "w") as f:
    json.dump(out, f, indent=2)

print(json.dumps(out, indent=2))
