#!/usr/bin/env python
"""F1: is k_d constant over launch speed 2-8 m/s?

Gate failed fits (kd<=0.02, kd>0.35, rms_mm>25). Bin per-arc kd by launch
speed, compare bin medians to the global fit, and test for a monotonic trend
(Spearman on gated arcs + Theil-Sen slope). KILL if significant monotonic
trend or any bin median off global by >20%; PASS if all bins within 10%
and no trend; else INCONCLUSIVE.
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

FITS = os.path.join(paths.FITS, "stage2_fits_all.json")
OUTDIR = paths.FALSIFICATION

d = json.load(open(FITS))
kd_global = d["kd"]["kd"]
pa = d["kd"]["per_arc"]

speed = np.array([r["speed"] for r in pa])
kd = np.array([r["kd"] for r in pa])
rms = np.array([r["rms_mm"] for r in pa])
take = np.array([r["take"] for r in pa])

# --- gating -----------------------------------------------------------------
gate = (kd > 0.02) & (kd <= 0.35) & (rms <= 25.0)
n_total, n_gated_out = len(pa), int((~gate).sum())

# test window: 2-8 m/s (per spec); report actual coverage
in_win = gate & (speed >= 2.0) & (speed <= 8.0)
s, k = speed[in_win], kd[in_win]
n_used = int(in_win.sum())
cov_lo, cov_hi = float(s.min()), float(s.max())

med_gated = float(np.median(k))  # median of gated arcs in window (secondary ref)

# --- bins -------------------------------------------------------------------
# 1 m/s bins 2-5, then a single 5-8 bin (only a handful of arcs above 5)
edges = [2.0, 3.0, 4.0, 5.0, 8.0]
bins = []
for lo, hi in zip(edges[:-1], edges[1:]):
    m = (s >= lo) & (s < hi)
    if m.sum() == 0:
        bins.append(dict(lo=lo, hi=hi, n=0))
        continue
    kb = k[m]
    med = float(np.median(kb))
    mad = float(stats.median_abs_deviation(kb, scale="normal"))
    bins.append(dict(
        lo=lo, hi=hi, n=int(m.sum()),
        median=med, mad=mad,
        dev_vs_global_pct=100.0 * (med - kd_global) / kd_global,
        dev_vs_medgated_pct=100.0 * (med - med_gated) / med_gated,
        # rough SE of the median for context
        se_med=float(1.2533 * mad / np.sqrt(m.sum())),
    ))

# --- trend tests ------------------------------------------------------------
rho, p_rho = stats.spearmanr(s, k)
ts = stats.theilslopes(k, s, 0.95)  # slope, intercept, lo, hi
# implied relative change in kd across the observed window, per Theil-Sen
rel_change_pct = 100.0 * ts.slope * (cov_hi - cov_lo) / kd_global
rel_change_lo = 100.0 * ts.low_slope * (cov_hi - cov_lo) / kd_global
rel_change_hi = 100.0 * ts.high_slope * (cov_hi - cov_lo) / kd_global

# bin-median monotonicity (only bins with n>=5)
solid = [b for b in bins if b["n"] >= 5]
meds = [b["median"] for b in solid]
monotone_bins = all(np.diff(meds) > 0) or all(np.diff(meds) < 0)

# --- verdict logic ----------------------------------------------------------
devs = [abs(b["dev_vs_global_pct"]) for b in solid]
max_dev = max(devs)
worst = max(solid, key=lambda b: abs(b["dev_vs_global_pct"]))

sig_trend = (p_rho < 0.05) and (abs(rel_change_pct) > 20.0) and monotone_bins

if sig_trend or max_dev > 20.0:
    verdict = "KILL"
elif max_dev <= 10.0 and not (p_rho < 0.05 and abs(rel_change_pct) > 10.0):
    verdict = "PASS"
else:
    verdict = "INCONCLUSIVE"

# --- plot -------------------------------------------------------------------
fig, ax = plt.subplots(figsize=(9, 6))
out_m = ~gate
ax.scatter(speed[out_m], np.clip(kd[out_m], 0, 0.45), marker="x", s=30,
           c="0.75", label=f"gated out (n={n_gated_out})", zorder=1)
lowv = gate & (speed < 2.0)
ax.scatter(speed[lowv], kd[lowv], s=25, c="tab:orange", alpha=0.6,
           label=f"<2 m/s, excluded (n={int(lowv.sum())})", zorder=2)
ax.scatter(s, k, s=30, c="tab:blue", alpha=0.65, label=f"used arcs (n={n_used})", zorder=3)

for b in bins:
    if b["n"] == 0:
        continue
    xc = 0.5 * (b["lo"] + min(b["hi"], 7.0))
    ax.errorbar(xc, b["median"], yerr=b["mad"], fmt="s", ms=9, c="crimson",
                capsize=5, lw=2, zorder=5)
    ax.annotate(f"n={b['n']}\n{b['dev_vs_global_pct']:+.0f}%",
                (xc, b["median"]), textcoords="offset points", xytext=(8, 10),
                fontsize=9, color="crimson")

ax.axhline(kd_global, color="k", lw=1.5, label=f"global fit kd={kd_global:.3f}")
ax.axhspan(kd_global * 0.9, kd_global * 1.1, color="green", alpha=0.10, label="±10% (PASS band)")
ax.axhspan(kd_global * 0.8, kd_global * 1.2, color="yellow", alpha=0.10, label="±20% (KILL beyond)")
xs = np.linspace(cov_lo, cov_hi, 10)
ax.plot(xs, ts.intercept + ts.slope * xs, "b--", lw=1,
        label=f"Theil-Sen: {rel_change_pct:+.0f}% over window")

ax.set_xlabel("launch speed (m/s)")
ax.set_ylabel("per-arc k_d")
ax.set_ylim(0, 0.47)
ax.set_title(f"F1: k_d vs launch speed — {verdict}\n"
             f"Spearman rho={rho:.2f} (p={p_rho:.3f}); coverage {cov_lo:.1f}-{cov_hi:.1f} m/s "
             f"(none above {cov_hi:.1f}); max bin dev {max_dev:.0f}%")
ax.legend(fontsize=8, loc="upper right")
ax.grid(alpha=0.3)
fig.tight_layout()
fig.savefig(f"{OUTDIR}/F1.png", dpi=130)

# --- robustness: ungated bin medians + gating fraction per bin ---------------
extra_bins = []
for lo, hi in zip(edges[:-1], edges[1:]):
    m_all = (speed >= lo) & (speed < hi)
    bad = m_all & ~gate
    med_u = float(np.median(kd[m_all])) if m_all.sum() else float("nan")
    extra_bins.append(dict(
        lo=lo, hi=hi, n_all=int(m_all.sum()), n_gated_out=int(bad.sum()),
        gated_out_kd_high=int((bad & (kd > 0.35)).sum()),
        gated_out_kd_low=int((bad & (kd <= 0.02)).sum()),
        ungated_median=round(med_u, 4),
        ungated_dev_vs_global_pct=round(100.0 * (med_u - kd_global) / kd_global, 1),
    ))

# --- verdict json -----------------------------------------------------------
res = {
    "test": "F1: k_d constant over launch speed 2-8 m/s",
    "verdict": verdict,
    "confidence": "medium",
    "caveats": [
        "Speed coverage is 2.0-6.8 m/s only: no arcs in 6.8-8 m/s, and just 6 arcs above 5 m/s (single 5-8 bin), so constancy at the top of the requested range rests on thin data.",
        "2-3 m/s bin is weakly identifiable per-arc as expected: MAD 0.098 (~78% of global kd) and 42% of its arcs were gated out (10 hit the low bound, 8 the high bound - roughly symmetric, so the median is not obviously directionally biased). This bin is 'consistent with constant kd' rather than a strong constraint; the 3-5 m/s region carries the evidential weight.",
        "Gating is not driving the verdict: ungated bin medians are -1.4/+11.8/-4.7/+0.9% vs global, still non-monotonic and within +-20%.",
        "Theil-Sen point slope implies -18.8% over the window, but its 95% CI is [-79%, +37%] (dominated by low-speed scatter) and Spearman p=0.52; no detectable trend, but a mild (<~15%) drift within 2-7 m/s cannot be excluded with this scatter.",
        "Position noise ~9 mm rms on this venue dataset inflates per-arc kd scatter relative to the old rig; medians/MAD used throughout as specified.",
    ],
    "artifacts": [
        f"{OUTDIR}/F1.png",
        f"{OUTDIR}/F1_verdict.json",
        "hope_training/ball_physics_fit/falsify/f1_kd_over_speed.py",
    ],
    "statistic": (f"max bin-median deviation from global kd = {max_dev:.1f}% "
                  f"(worst bin {worst['lo']:.0f}-{worst['hi']:.0f} m/s, "
                  f"{worst['dev_vs_global_pct']:+.1f}%); Spearman rho={rho:.2f}, p={p_rho:.3f}"),
    "key_numbers": {
        "kd_global_fit": round(kd_global, 4),
        "kd_median_gated_2to8": round(med_gated, 4),
        "n_total_arcs": n_total,
        "n_gated_out": n_gated_out,
        "n_below_2ms_excluded": int(lowv.sum()),
        "n_used_2to8": n_used,
        "speed_coverage_ms": [round(cov_lo, 2), round(cov_hi, 2)],
        "spearman_rho": round(float(rho), 3),
        "spearman_p": round(float(p_rho), 4),
        "theilsen_rel_change_over_window_pct": round(rel_change_pct, 1),
        "theilsen_rel_change_95ci_pct": [round(rel_change_lo, 1), round(rel_change_hi, 1)],
        "bin_medians_monotonic": bool(monotone_bins),
        "bins": [
            {k2: (round(v, 4) if isinstance(v, float) else v) for k2, v in b.items()}
            for b in bins
        ],
        "robustness_ungated_bins": extra_bins,
    },
}
with open(f"{OUTDIR}/F1_verdict.json", "w") as f:
    json.dump(res, f, indent=2)
print(json.dumps(res, indent=2))
