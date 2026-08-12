import os
import sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import paths
import os
"""F3: is table normal restitution e_n constant over impact speed vn?

Data: stage2_fits_all.json -> table_e.per_bounce (n=92, vn 0.96-4.50 m/s).
KILL if |de/dvn| > 0.01 /m/s with CI support, or visible break >5 m/s.
PASS if flat within +-0.025 over covered vn. Else INCONCLUSIVE.
"""
import json
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

FITS = os.path.join(paths.FITS, "stage2_fits_all.json")
OUTDIR = paths.FALSIFICATION

d = json.load(open(FITS))
te = d["table_e"]
pb = te["per_bounce"]
vn = np.array([b["vn"] for b in pb])
e = np.array([b["e"] for b in pb])
ut = np.array([b["ut_over_un"] for b in pb])
takes = np.array([b["take"] for b in pb])

rng = np.random.default_rng(42)


def ols_slope(x, y):
    A = np.vstack([np.ones_like(x), x]).T
    c, *_ = np.linalg.lstsq(A, y, rcond=None)
    return c  # [intercept, slope]


def boot_slope(x, y, n=4000, fn=None):
    out = []
    for _ in range(n):
        i = rng.integers(0, len(x), len(x))
        if len(np.unique(x[i])) < 3:
            continue
        out.append(fn(x[i], y[i]))
    return np.percentile(out, [2.5, 50, 97.5])


def theil_sen(x, y):
    n = len(x)
    sl = []
    for i in range(n):
        dx = x[i + 1:] - x[i]
        dy = y[i + 1:] - y[i]
        ok = np.abs(dx) > 1e-9
        sl.extend((dy[ok] / dx[ok]).tolist())
    return float(np.median(sl))


def ols_s(x, y):
    return ols_slope(x, y)[1]


results = {}

# --- full-sample fits -------------------------------------------------------
c_ols = ols_slope(vn, e)
ci_ols = boot_slope(vn, e, fn=ols_s)
ts = theil_sen(vn, e)
ci_ts = boot_slope(vn, e, fn=theil_sen)
results["full"] = dict(n=len(e), ols_slope=float(c_ols[1]), ols_intercept=float(c_ols[0]),
                       ols_ci=[float(ci_ols[0]), float(ci_ols[2])],
                       theil_sen=ts, ts_ci=[float(ci_ts[0]), float(ci_ts[2])])

# --- robustness subsets -----------------------------------------------------
subsets = {
    "vn>=1.5": vn >= 1.5,
    "ut/un<1.0": ut < 1.0,
    "vn>=1.5 & ut/un<1.0": (vn >= 1.5) & (ut < 1.0),
    "e<=1 (drop unphysical)": e <= 1.0,
}
for name, m in subsets.items():
    if m.sum() < 10:
        continue
    ci = boot_slope(vn[m], e[m], fn=ols_s)
    results[name] = dict(n=int(m.sum()), ols_slope=float(ols_s(vn[m], e[m])),
                         ols_ci=[float(ci[0]), float(ci[2])],
                         theil_sen=theil_sen(vn[m], e[m]))

# --- binned medians with bootstrap CI --------------------------------------
edges = np.array([0.9, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 4.6])
bins = []
for lo, hi in zip(edges[:-1], edges[1:]):
    m = (vn >= lo) & (vn < hi)
    if m.sum() == 0:
        continue
    meds = [np.median(e[m][rng.integers(0, m.sum(), m.sum())]) for _ in range(3000)]
    bins.append(dict(lo=float(lo), hi=float(hi), n=int(m.sum()),
                     vn_med=float(np.median(vn[m])), e_med=float(np.median(e[m])),
                     ci=[float(np.percentile(meds, 2.5)), float(np.percentile(meds, 97.5))]))
results["bins"] = bins

# spread of binned medians (only bins with n>=8)
good = [b for b in bins if b["n"] >= 8]
med_vals = [b["e_med"] for b in good]
results["bin_spread"] = dict(n_good_bins=len(good), min=float(min(med_vals)),
                             max=float(max(med_vals)),
                             half_range=float((max(med_vals) - min(med_vals)) / 2))

# --- noise diagnostics ------------------------------------------------------
# e>1 unphysical rows: where do they live in vn?
m_gt1 = e > 1.0
results["unphysical_e_gt1"] = dict(
    n=int(m_gt1.sum()), vn_values=[float(v) for v in sorted(vn[m_gt1])],
    frac_below_vn2=float(np.mean(vn[m_gt1] < 2.0)) if m_gt1.any() else None)

# spread vs vn: MAD in low vs high half
lo_half = vn < np.median(vn)
results["spread_by_half"] = dict(
    mad_low_vn=float(np.median(np.abs(e[lo_half] - np.median(e[lo_half])))),
    mad_high_vn=float(np.median(np.abs(e[~lo_half] - np.median(e[~lo_half])))))

# per-take medians (confound check)
per_take = {}
for t in np.unique(takes):
    m = takes == t
    per_take[str(t)] = dict(n=int(m.sum()), vn_med=float(np.median(vn[m])),
                            e_med=float(np.median(e[m])))
results["per_take"] = per_take

# within-take (centered) slope: removes between-take confounds
vn_c = vn.copy().astype(float)
e_c = e.copy().astype(float)
for t in np.unique(takes):
    m = takes == t
    vn_c[m] -= vn[m].mean()
    e_c[m] -= e[m].mean()
ci_w = boot_slope(vn_c, e_c, fn=ols_s)
results["within_take"] = dict(ols_slope=float(ols_s(vn_c, e_c)),
                              ols_ci=[float(ci_w[0]), float(ci_w[2])])

# --- old-rig band comparison ------------------------------------------------
m_band = (vn >= 2.5) & (vn <= 3.6)
results["band_2p5_3p6"] = dict(n=int(m_band.sum()),
                               e_med=float(np.median(e[m_band])) if m_band.any() else None,
                               old_rig=0.908)

results["coverage"] = dict(vn_min=float(vn.min()), vn_max=float(vn.max()),
                           requested=[0.5, 6.0],
                           n_above_5=int((vn > 5).sum()), n_below_1=int((vn < 1).sum()))
results["prefit"] = dict(linear_a=te["linear_a"], linear_b=te["linear_b"],
                         slope_ci=te["slope_ci"], e_const=te["e_const"], e_mad=te["e_mad"])

print(json.dumps(results, indent=1))

# --- plot -------------------------------------------------------------------
fig, axes = plt.subplots(1, 2, figsize=(13, 5.2))
ax = axes[0]
sc = ax.scatter(vn, e, c=ut, cmap="viridis", s=28, alpha=0.8, edgecolors="k",
                linewidths=0.3, vmax=2.0)
plt.colorbar(sc, ax=ax, label="|u_t|/u_n at contact")
bx = [b["vn_med"] for b in bins]
by = [b["e_med"] for b in bins]
blo = [b["e_med"] - b["ci"][0] for b in bins]
bhi = [b["ci"][1] - b["e_med"] for b in bins]
ax.errorbar(bx, by, yerr=[blo, bhi], fmt="s-", color="crimson", ms=7, lw=2,
            capsize=4, label="binned median (95% CI)", zorder=5)
xx = np.linspace(0.9, 4.6, 50)
ax.plot(xx, c_ols[0] + c_ols[1] * xx, "b--", lw=1.5,
        label=f"OLS slope {c_ols[1]:+.4f}/m/s\nCI [{ci_ols[0]:+.4f}, {ci_ols[2]:+.4f}]")
ax.axhline(te["e_const"], color="gray", ls=":", label=f"median e = {te['e_const']:.3f}")
ax.fill_between([0.9, 4.6], te["e_const"] - 0.025, te["e_const"] + 0.025,
                color="gray", alpha=0.15, label="PASS band ±0.025")
ax.axhline(1.0, color="k", lw=0.6)
ax.plot([2.5, 3.6], [0.908, 0.908], color="darkorange", lw=3, alpha=0.8,
        label="old rig 0.908 (2.5–3.6)")
ax.set_xlabel("v_n impact (m/s)")
ax.set_ylabel("e_n = v_n,out / v_n,in")
ax.set_title(f"F3: table restitution vs impact speed (n={len(e)})\n"
             "coverage 0.96–4.50 m/s (requested 0.5–6: no data > 4.5)")
ax.legend(fontsize=8, loc="lower left")
ax.set_ylim(0.55, 1.15)

ax = axes[1]
labels, slopes, los, his = [], [], [], []
for name in ["full", "vn>=1.5", "ut/un<1.0", "vn>=1.5 & ut/un<1.0",
             "e<=1 (drop unphysical)", "within_take"]:
    r = results[name]
    labels.append(f"{name} (n={r.get('n', len(e))})")
    slopes.append(r["ols_slope"])
    los.append(r["ols_slope"] - r["ols_ci"][0])
    his.append(r["ols_ci"][1] - r["ols_slope"])
ypos = np.arange(len(labels))[::-1]
ax.errorbar(slopes, ypos, xerr=[los, his], fmt="o", color="navy", capsize=4)
ax.axvline(0, color="k", lw=0.8)
ax.axvspan(-0.01, 0.01, color="green", alpha=0.12, label="|slope| < 0.01 (PASS zone)")
ax.set_yticks(ypos)
ax.set_yticklabels(labels, fontsize=8)
ax.set_xlabel("de/dvn (per m/s), OLS with 95% bootstrap CI")
ax.set_title("Slope robustness across subsets")
ax.legend(fontsize=8)
fig.tight_layout()
fig.savefig(f"{OUTDIR}/F3.png", dpi=140)
print(f"saved {OUTDIR}/F3.png")

json.dump(results, open(f"{OUTDIR}/F3_diag.json", "w"), indent=1)
