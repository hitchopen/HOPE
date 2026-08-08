"""
F5 — slip-regime probe of the grip-only tangential law at the table bounce.

Part A (as specified): bin fits.table_tangential.per_bounce residuals by
ratio = |u_t|/|u_n|; PASS if RMS dw < 3 rev/s in every bin; KILL if ~0 below
ratio 1 but growing beyond 1.5.

Part B (diagnostics): the delivered fit is degenerate (a_t+b_t*cos(th) < 0
everywhere -> s clipped to 0), so Part A residuals are raw measured changes,
not model misfit.  We therefore also run a velocity-channel-only test
(positions trusted ~9 mm; quat spin reads LOW): fit
    grip-only     : s = max(a + b cos th, 0) * |u_t|
    grip + cap    : s = clip((a + b cos th)|u_t|, 0, mu (1+e)|u_n|)
    pure Coulomb  : s = mu (1+e) |u_n|
to the measured tangential velocity change dv_t, and compare per-ratio-bin
residuals.  A real slip branch shows up as the capped model beating grip-only
at high ratio with a sensible mu.
"""
import os
import sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import paths
import json
import numpy as np
from scipy import stats
from scipy.optimize import least_squares
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

ANA = paths.ANALYSIS
FAL = ANA + "/falsification"
R = 0.020
CR = (2.0 / 3.0) * R

fits = json.load(open(ANA + "/fits/stage2_fits_all.json"))
tt = fits["table_tangential"]
e_const = fits["table_e"]["e_const"]
pb = tt["per_bounce"]
a_t, b_t, mu_fit = tt["a_t"], tt["b_t"], tt["mu"]

bounces = json.load(open(ANA + "/segments/bounces.json"))
bmap = {(b["take"], round(b["t_c"], 6)): b for b in bounces}

rows = []
for r in pb:
    b = bmap.get((r["take"], round(r["t_c"], 6)))
    assert b is not None, r
    rows.append({**r, **b})
n_all = len(rows)

ratio = np.array([r["ratio"] for r in rows])
dw = np.array([r["dw_revs"] for r in rows])
dvt_res = np.array([r["dvt"] for r in rows])
cap_binds = np.array([r["cap_binds"] for r in rows])

# ---- degeneracy check of the delivered fit --------------------------------
V_in = np.array([r["v_in"] for r in rows])
V_out = np.array([r["v_out"] for r in rows])
W_in = np.array([r["w_in"] for r in rows])
W_out = np.array([r["w_out"] for r in rows])
n_hat = np.array([0.0, 0.0, 1.0])
# n oriented against approach (vz_in < 0 so n stays +z for all)
u = V_in + np.cross(W_in, -R * n_hat[None, :])
u_n = u @ n_hat                      # signed, negative incoming
u_t_vec = u - u_n[:, None] * n_hat
u_t = np.linalg.norm(u_t_vec, axis=1)
cos_th = np.abs(u_n) / np.hypot(u_t, u_n)
grip_coef = a_t + b_t * cos_th
s_delivered = np.clip(grip_coef * u_t, 0, mu_fit * (1 + e_const) * np.abs(u_n))
degenerate = bool((s_delivered == 0).all())
max_grip_coef = float(grip_coef.max())

# measured tangential impulse (per unit mass) from positions only
dv_meas = V_out - V_in
dvt_meas_vec = dv_meas - (dv_meas @ n_hat)[:, None] * n_hat
s_meas = np.linalg.norm(dvt_meas_vec, axis=1)
unit_t = u_t_vec / (u_t[:, None] + 1e-12)
# component of measured tangential dv along -u_t (physically expected direction)
s_meas_signed = -(dvt_meas_vec * unit_t).sum(1)
align = s_meas_signed / (s_meas + 1e-12)

# spin channel vs velocity channel consistency: dw implied by dv_t
dw_implied_revs = s_meas / CR / (2 * np.pi)
dw_meas_revs = np.linalg.norm(W_out - W_in, axis=1) / (2 * np.pi)
chan_ratio = np.median(dw_meas_revs / (dw_implied_revs + 1e-12))

# ---- Part A: binned residuals as specified --------------------------------
edges = [0.0, 0.5, 1.0, 1.5, 2.5, np.inf]
labels = ["[0,0.5)", "[0.5,1)", "[1,1.5)", "[1.5,2.5)", ">=2.5"]
rng = np.random.default_rng(0)

def rms(x):
    return float(np.sqrt(np.mean(np.square(x)))) if len(x) else float("nan")

def boot_rms_ci(x, n=4000):
    if len(x) < 2:
        return [float("nan")] * 2
    bs = [rms(x[rng.integers(0, len(x), len(x))]) for _ in range(n)]
    return [float(np.percentile(bs, q)) for q in (2.5, 97.5)]

bins = []
for lo, hi, lab in zip(edges[:-1], edges[1:], labels):
    m = (ratio >= lo) & (ratio < hi)
    bins.append(dict(bin=lab, n=int(m.sum()),
                     rms_dw_revs=rms(dw[m]), rms_dw_ci=boot_rms_ci(dw[m]),
                     med_dw_revs=float(np.median(dw[m])) if m.sum() else None,
                     rms_dvt=rms(dvt_res[m])))

noise_mask = ratio < 0.2
noise_floor = rms(dw[noise_mask])           # near-zero slip -> true dw ~ 0
noise_ci = boot_rms_ci(dw[noise_mask])

sp_dw = stats.spearmanr(ratio, dw)
sp_dvt = stats.spearmanr(ratio, dvt_res)
lo_m, hi_m = ratio < 1.0, ratio >= 1.5
mw = stats.mannwhitneyu(dw[hi_m], dw[lo_m], alternative="greater")
# confounder: does dw scale with spin magnitude instead of ratio?
w_in_revs = np.linalg.norm(W_in, axis=1) / (2 * np.pi)
sp_dw_w = stats.spearmanr(w_in_revs, dw)

# ---- Part B: velocity-channel model comparison ----------------------------
def fit_model(kind, mask=None, x0=None):
    m = np.ones(n_all, bool) if mask is None else mask
    ut_, un_, ct_ = u_t[m], np.abs(u_n[m]), cos_th[m]
    tgt = dvt_meas_vec[m]
    utv = unit_t[m]

    def s_of(x):
        if kind == "grip":
            a, b = x
            return np.clip((a + b * ct_) * ut_, 0, None)
        if kind == "grip_cap":
            a, b, mu = x
            return np.clip((a + b * ct_) * ut_, 0, abs(mu) * (1 + e_const) * un_)
        if kind == "coulomb":
            (mu,) = x
            return np.minimum(abs(mu) * (1 + e_const) * un_, ut_)  # can't reverse slip

    def resid(x):
        pred = -s_of(x)[:, None] * utv
        return (pred - tgt).ravel()

    x0 = x0 or dict(grip=[0.3, 0.1], grip_cap=[0.3, 0.1, 0.3], coulomb=[0.3])[kind]
    sol = least_squares(resid, x0, loss="huber", f_scale=0.3, max_nfev=4000)
    res = resid(sol.x).reshape(-1, 3)
    per_ev = np.linalg.norm(res, axis=1)
    full = np.full(n_all, np.nan)
    full[m] = per_ev
    s_full = np.full(n_all, np.nan)
    s_full[m] = s_of(sol.x)
    return dict(kind=kind, x=[float(v) for v in sol.x], rms=rms(per_ev),
                per_ev=full, s=s_full, n=int(m.sum()))

mA = fit_model("grip")
mB = fit_model("grip_cap")
mC = fit_model("coulomb")
capB = (mB["x"][0] + mB["x"][1] * cos_th) * u_t > abs(mB["x"][2]) * (1 + e_const) * np.abs(u_n)

def bin_rms_of(per_ev):
    out = []
    for lo, hi in zip(edges[:-1], edges[1:]):
        m = (ratio >= lo) & (ratio < hi) & np.isfinite(per_ev)
        out.append(rms(per_ev[m]))
    return out

velbin = dict(grip=bin_rms_of(mA["per_ev"]), grip_cap=bin_rms_of(mB["per_ev"]),
              coulomb=bin_rms_of(mC["per_ev"]), null_s0=bin_rms_of(s_meas))

# high-ratio refit per F5 instruction (both channels, mu free, e frozen)
def refit_full(mask, x0=(0.4, 0.0, 0.3)):
    m = mask
    def resid(x):
        a, b, mu = x
        s = np.clip((a + b * cos_th[m]) * u_t[m], 0,
                    abs(mu) * (1 + e_const) * np.abs(u_n[m]))
        dv_t_pred = -s[:, None] * unit_t[m]
        dw_pred = -(1.0 / CR) * np.cross(np.broadcast_to(n_hat, dv_t_pred.shape), dv_t_pred)
        rv = dv_t_pred - dvt_meas_vec[m]
        rw = (dw_pred - (W_out[m] - W_in[m])) * CR
        return np.concatenate([rv.ravel(), rw.ravel()])
    sol = least_squares(resid, x0, loss="huber", f_scale=0.3, max_nfev=4000)
    return [float(v) for v in sol.x]

hi10 = ratio >= 1.0
refit_hi = refit_full(hi10)

# ---- verdict logic ---------------------------------------------------------
pass_all_bins = all(b["rms_dw_revs"] < 3.0 for b in bins if b["n"] > 0)
low_bins_near_zero = all(b["rms_dw_revs"] < 1.0 for b in bins[:2])
n_hi = int((ratio >= 1.5).sum())

verdict = "INCONCLUSIVE"
reasons = []
if pass_all_bins:
    verdict = "PASS"
if degenerate:
    reasons.append("delivered grip fit is degenerate: a_t+b_t*cos(th) <= "
                   f"{max_grip_coef:.1e} < 0 for all 107 bounces -> s==0; "
                   "'residuals' are raw measured changes, cap engaged 0/107, "
                   "mu=1.0 is the untouched initializer")
if noise_floor >= 2.0:
    reasons.append(f"spin-channel noise floor (RMS dw at ratio<0.2, n={int(noise_mask.sum())}) "
                   f"= {noise_floor:.2f} rev/s [{noise_ci[0]:.2f},{noise_ci[1]:.2f}] — "
                   "the 3 rev/s PASS bar is within noise of a perfect model")
reasons.append(f"coverage: only {n_hi}/107 bounces with ratio>=1.5, 1 with ratio>=2.5")

out = dict(
    test="F5 grip-only tangential law in slip regime (table bounce)",
    verdict=verdict,
    delivered_fit=dict(a_t=a_t, b_t=b_t, mu=mu_fit, degenerate_s_zero=degenerate,
                       max_grip_coefficient=max_grip_coef,
                       cap_binds_count=int(cap_binds.sum())),
    ratio_coverage=dict(n_total=n_all, n_ge_1=int((ratio >= 1).sum()),
                        n_ge_1p5=n_hi, n_ge_2p5=int((ratio >= 2.5).sum()),
                        ratio_max=float(ratio.max())),
    bins=bins,
    pass_criterion_rms_lt_3_all_bins=pass_all_bins,
    kill_pattern_low_bins_near_zero=low_bins_near_zero,
    noise_floor_dw_revs=dict(value=noise_floor, ci95=noise_ci,
                             definition="RMS dw at ratio<0.2 where true tangential impulse ~ 0"),
    trends=dict(spearman_dw_vs_ratio=[float(sp_dw.statistic), float(sp_dw.pvalue)],
                spearman_dvt_vs_ratio=[float(sp_dvt.statistic), float(sp_dvt.pvalue)],
                spearman_dw_vs_win=[float(sp_dw_w.statistic), float(sp_dw_w.pvalue)],
                mannwhitney_hi_gt_lo_p=float(mw.pvalue)),
    spin_channel_consistency=dict(
        median_dw_measured_over_dw_implied_by_dvt=float(chan_ratio),
        note="quat spin change reads ~this fraction of the value implied by the "
             "position-derived tangential impulse (dw = dv_t/(cR)); spin reads LOW"),
    velocity_channel_models=dict(
        grip=dict(params=mA["x"], rms=mA["rms"]),
        grip_cap=dict(params=mB["x"], rms=mB["rms"], cap_binds=int(np.nansum(capB))),
        coulomb=dict(params=mC["x"], rms=mC["rms"]),
        bin_rms=velbin, bin_edges=edges[:-1] + [None],
        alignment_median=float(np.median(align))),
    high_ratio_refit_mu_free=dict(subset="ratio>=1.0", n=int(hi10.sum()),
                                  a_t=refit_hi[0], b_t=refit_hi[1], mu=abs(refit_hi[2])),
    reasons=reasons,
)

with open(FAL + "/F5_verdict.json", "w") as f:
    json.dump(out, f, indent=1)
print(json.dumps({k: v for k, v in out.items() if k not in ("bins",)}, indent=1)[:3500])
for b in bins:
    print(b)

# ---- plot ------------------------------------------------------------------
fig, axes = plt.subplots(1, 3, figsize=(15, 4.6))
centers = [0.25, 0.75, 1.25, 2.0, 2.6]

ax = axes[0]
ax.scatter(ratio, dw, s=14, c="tab:blue", alpha=0.6)
ax.errorbar(centers, [b["rms_dw_revs"] for b in bins],
            yerr=[[b["rms_dw_revs"] - b["rms_dw_ci"][0] if b["n"] > 1 else 0 for b in bins],
                  [b["rms_dw_ci"][1] - b["rms_dw_revs"] if b["n"] > 1 else 0 for b in bins]],
            fmt="s-", color="k", capsize=3, label="bin RMS (95% CI)")
ax.axhline(3.0, color="r", ls="--", lw=1, label="PASS bar 3 rev/s")
ax.axhspan(0, noise_floor, color="gray", alpha=0.25, label=f"noise floor {noise_floor:.1f} rev/s")
for c, b in zip(centers, bins):
    ax.annotate(f"n={b['n']}", (c, 0.2), ha="center", fontsize=8)
ax.set_xlabel("|u_t| / |u_n|"); ax.set_ylabel("|dw residual| (rev/s)")
ax.set_title("A: spin residual vs ratio\n(delivered fit is s=0 -> residual = raw |Δω|)")
ax.legend(fontsize=8); ax.set_ylim(0, 14)

ax = axes[1]
ax.scatter(ratio, s_meas, s=14, c="tab:orange", alpha=0.6, label="|Δv_t| measured")
rr = np.linspace(0.01, 2.8, 200)
ct = 1 / np.sqrt(1 + rr**2)
un_med = np.median(np.abs(u_n))
ut_line = rr * un_med
gA = np.clip((mA["x"][0] + mA["x"][1] * ct) * ut_line, 0, None)
gC = np.minimum(abs(mC["x"][0]) * (1 + e_const) * un_med, ut_line)
ax.plot(rr, gA, "b-", lw=1.5, label=f"grip fit a={mA['x'][0]:.2f} b={mA['x'][1]:.2f}")
ax.plot(rr, gC, "g--", lw=1.5, label=f"Coulomb cap mu={abs(mC['x'][0]):.2f}")
ax.set_xlabel("|u_t| / |u_n|"); ax.set_ylabel("tangential impulse |Δv_t| (m/s)")
ax.set_title("B: velocity-channel impulse vs ratio\n(curves at median |u_n|)")
ax.legend(fontsize=8)

ax = axes[2]
w = 0.2
for i, (k, col) in enumerate([("null_s0", "0.6"), ("grip", "tab:blue"),
                              ("grip_cap", "tab:green"), ("coulomb", "tab:red")]):
    ax.bar(np.arange(5) + (i - 1.5) * w, velbin[k], w, color=col, label=k)
ax.set_xticks(range(5)); ax.set_xticklabels(labels, fontsize=8)
ax.set_ylabel("RMS |Δv_t| residual (m/s)"); ax.set_xlabel("ratio bin")
ax.set_title("C: velocity-channel model comparison by bin")
ax.legend(fontsize=8)

fig.suptitle(f"F5 slip-regime probe — verdict {verdict} "
             f"(n={n_all}, ratio>=1.5: n={n_hi})", fontsize=11)
fig.tight_layout(rect=[0, 0, 1, 0.94])
fig.savefig(FAL + "/F5.png", dpi=140)
print("saved", FAL + "/F5.png")
