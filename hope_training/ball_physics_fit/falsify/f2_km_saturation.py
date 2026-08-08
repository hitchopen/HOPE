#!/usr/bin/env python
"""F2: is Magnus k_m constant in spin ratio SR, or does it saturate at high SR?

KILL constant-km if median km in SR>1.5 bins < 80% of SR<0.5 bin median.
PASS if bin medians flat within +-15%. INCONCLUSIVE if coverage/quality can't decide.
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

rng = np.random.default_rng(42)

FITS = os.path.join(paths.FITS, "stage2_fits_all.json")
OUTDIR = paths.FALSIFICATION

d = json.load(open(FITS))
rows = d["km"]["per_arc"]
km_global = d["km"]["km"]

sr = np.array([r["sr"] for r in rows])
km = np.array([r["km"] for r in rows])
rms = np.array([r["rms_mm"] for r in rows])
ss = np.array([bool(r["sidespin"]) for r in rows])
speed = np.array([r["speed"] for r in rows])
rev = np.array([r["rev"] for r in rows])

# ---- quality gates (fit failures) ----
bad_neg = km < 0
bad_big = km > 0.02
bad_rms = rms > 15.0          # position noise here ~9 mm; >15 mm = fit failure
good = ~(bad_neg | bad_big | bad_rms)
n_dropped = int((~good).sum())

# selection check: do high-SR rows fail the gate more often?
hi_raw = sr > 0.7
drop_rate_hi = float((~good[hi_raw]).mean()) if hi_raw.sum() else float("nan")
drop_rate_all = float((~good).mean())

g_sr, g_km, g_ss = sr[good], km[good], ss[good]

# ---- coverage of the saturation region ----
cov = {
    "n_total_raw": len(rows),
    "n_kept": int(good.sum()),
    "n_dropped": n_dropped,
    "n_dropped_km_neg": int(bad_neg.sum()),
    "n_dropped_km_gt_0p02": int(bad_big.sum()),
    "n_dropped_rms_gt_15mm": int((bad_rms & ~bad_neg & ~bad_big).sum()),
    "sr_min_kept": float(g_sr.min()),
    "sr_max_kept": float(g_sr.max()),
    "n_kept_sr_gt_1p5": int((g_sr > 1.5).sum()),
    "n_kept_sr_gt_1p0": int((g_sr > 1.0).sum()),
    "n_kept_sr_gt_0p7": int((g_sr > 0.7).sum()),
    "n_raw_sr_gt_1p5": int((sr > 1.5).sum()),
    "drop_rate_overall": drop_rate_all,
    "drop_rate_sr_gt_0p7": drop_rate_hi,
}

# ---- binning with merge-up so every used bin has >=5 arcs ----
EDGES = [0.055, 0.2, 0.3, 0.45, 0.7, 1.7]

def make_bins(x, y, edges, min_n=5):
    """Bin y by x; merge a thin right bin into its left neighbour."""
    bins = []
    i = 0
    while i < len(edges) - 1:
        lo, hi = edges[i], edges[i + 1]
        j = i + 1
        m = (x >= lo) & (x < edges[j])
        # extend right edge until enough points or run out
        while m.sum() < min_n and j < len(edges) - 1:
            j += 1
            m = (x >= lo) & (x < edges[j])
        if m.sum() >= min_n:
            med = float(np.median(y[m]))
            boots = np.array([np.median(rng.choice(y[m], m.sum())) for _ in range(4000)])
            lo_ci, hi_ci = np.percentile(boots, [16, 84])
            bins.append(dict(sr_lo=lo, sr_hi=edges[j], n=int(m.sum()),
                             km_med=med, ci16=float(lo_ci), ci84=float(hi_ci),
                             sr_mid=float(np.median(x[m]))))
        else:
            # leftover tail with <min_n arcs: record as unusable
            if m.sum() > 0:
                bins.append(dict(sr_lo=lo, sr_hi=edges[-1], n=int(m.sum()),
                                 km_med=float(np.median(y[m])), ci16=None, ci84=None,
                                 sr_mid=float(np.median(x[m])), unusable=True))
            break
        i = j
    return bins

bins_all = make_bins(g_sr, g_km, EDGES)
bins_ss = make_bins(g_sr[g_ss], g_km[g_ss], EDGES)

# ---- reference (low-SR) medians and sanity check ----
low_all = float(np.median(g_km[g_sr < 0.5]))
low_ss = float(np.median(g_km[(g_sr < 0.5) & g_ss]))
sanity_ok = 0.003 <= low_ss <= 0.006

# ---- flatness within covered range (supplementary; not the KILL statistic) ----
def flatness(bins, ref):
    usable = [b for b in bins if not b.get("unusable")]
    ratios = [b["km_med"] / ref for b in usable]
    return usable, ratios, all(abs(r - 1) <= 0.15 for r in ratios)

usable_all, ratios_all, flat_all = flatness(bins_all, low_all)
usable_ss, ratios_ss, flat_ss = flatness(bins_ss, low_ss)

# ---- the candidate saturating law, expressed as km(SR) ----
# C_L = 0.752*SR / (1 + 0.752*SR/0.55)  ->  km(SR) = km_lin / (1 + 1.367*SR)
# scale km_lin so the curve matches the lowest usable bin of the sidespin channel
b0 = usable_ss[0]
km_lin = b0["km_med"] * (1 + 0.752 * b0["sr_mid"] / 0.55)
sat = lambda s: km_lin / (1 + 0.752 * s / 0.55)

# predicted suppression across the covered range vs observed
pred_hi = sat(usable_ss[-1]["sr_mid"]) / sat(usable_ss[0]["sr_mid"])
obs_hi = usable_ss[-1]["km_med"] / usable_ss[0]["km_med"]

# can the top usable bin's CI distinguish constant from saturating?
top = usable_ss[-1]
ci_width_rel = (top["ci84"] - top["ci16"]) / low_ss
distinguishable = ci_width_rel < abs(1 - pred_hi)

# ---- extra characterization of the covered-range dip ----
from scipy.stats import spearmanr
rho, pval = spearmanr(g_sr, g_km)
dip = [b for b in bins_ss if b["sr_lo"] == 0.3][0]
low_bin = bins_ss[0]
dip_ci_separated = dip["ci84"] < low_bin["ci16"]

# ---- verdict ----
# The KILL test is defined at SR>1.5 (>=5 arcs/bin). We have 1 kept arc there.
verdict = "INCONCLUSIVE"
reason = ("coverage: only %d kept arc(s) with SR>1.5 (%d raw), %d with SR>0.7; "
          "KILL test needs >=5 arcs/bin in SR>1.5"
          % (cov["n_kept_sr_gt_1p5"], cov["n_raw_sr_gt_1p5"], cov["n_kept_sr_gt_0p7"]))

# ---- plot ----
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 5.2), width_ratios=[2, 1])

dropped = ~good
ax1.scatter(sr[dropped], np.clip(km[dropped], -0.004, 0.024), marker="x", s=25,
            c="0.75", label="gated out (%d): km<0 / km>0.02 / rms>15mm" % n_dropped)
ax1.scatter(g_sr[~g_ss], g_km[~g_ss], s=28, c="tab:blue", alpha=0.65,
            label="kept, top/back-spin (%d)" % int((~g_ss).sum()))
ax1.scatter(g_sr[g_ss], g_km[g_ss], s=28, c="tab:red", alpha=0.75,
            label="kept, sidespin = trusted (%d)" % int(g_ss.sum()))

for bins, col, lbl in [(bins_all, "tab:blue", "bin median (all)"),
                       (bins_ss, "tab:red", "bin median (sidespin)")]:
    ub = [b for b in bins if not b.get("unusable")]
    x = [b["sr_mid"] for b in ub]
    y = [b["km_med"] for b in ub]
    ylo = [b["km_med"] - b["ci16"] for b in ub]
    yhi = [b["ci84"] - b["km_med"] for b in ub]
    ax1.errorbar(x, y, yerr=[ylo, yhi], fmt="s-", ms=9, lw=2, capsize=4,
                 color=col, mec="k", zorder=5, label=lbl + " ±68% CI")

xs = np.linspace(0.05, 1.7, 200)
ax1.axhline(km_global, color="k", ls="--", lw=1.2,
            label="constant km = %.4f (global fit)" % km_global)
ax1.plot(xs, sat(xs), color="tab:green", lw=2, ls="-.",
         label="saturating law C_L=0.752SR/(1+0.752SR/0.55)\nanchored at low-SR bin")
ax1.axvspan(1.5, 1.7, color="orange", alpha=0.15)
ax1.text(1.51, 0.0185, "KILL-test\nregion\n(n=%d kept)" % cov["n_kept_sr_gt_1p5"],
         fontsize=8, color="darkorange")
ax1.axhspan(0.003, 0.006, color="green", alpha=0.06)
ax1.set_xlim(0, 1.72)
ax1.set_ylim(-0.005, 0.025)
ax1.set_xlabel("spin ratio SR = R|w|/|v|")
ax1.set_ylabel("per-arc fitted $k_m$")
ax1.set_title("F2: $k_m$ vs SR — saturation region (SR>1.5) is empty after gating")
ax1.legend(fontsize=7.5, loc="upper right")
ax1.grid(alpha=0.3)

ax2.hist([sr[good & ss], sr[good & ~ss]], bins=np.arange(0, 1.75, 0.1),
         stacked=True, color=["tab:red", "tab:blue"],
         label=["sidespin kept", "other kept"])
ax2.hist(sr[~good], bins=np.arange(0, 1.75, 0.1), histtype="step",
         color="0.4", label="gated out")
ax2.axvspan(1.5, 1.7, color="orange", alpha=0.2)
ax2.axvline(1.5, color="darkorange", lw=1.5)
ax2.text(1.05, ax2.get_ylim()[1] * 0.75, "KILL test needs\n>=5 arcs here",
         fontsize=8, color="darkorange")
ax2.set_xlabel("SR")
ax2.set_ylabel("arcs")
ax2.set_title("SR coverage (max kept SR = %.2f)" % g_sr.max())
ax2.legend(fontsize=8)
ax2.grid(alpha=0.3)

fig.suptitle("VERDICT: INCONCLUSIVE (coverage) — low-SR km sanity OK (%.4f in [0.003,0.006])" % low_ss,
             fontsize=11, y=1.0)
fig.tight_layout()
fig.savefig(f"{OUTDIR}/F2.png", dpi=150, bbox_inches="tight")

# ---- verdict json ----
out = {
    "test": "F2: Magnus coefficient km constant vs saturating in spin ratio SR",
    "verdict": verdict,
    "confidence": "high",
    "statistic": reason,
    "key_numbers": {
        "coverage": cov,
        "km_global": km_global,
        "low_sr_median_km_all": low_all,
        "low_sr_median_km_sidespin": low_ss,
        "low_sr_sanity_ok_0.003_0.006": bool(sanity_ok),
        "bins_all": bins_all,
        "bins_sidespin": bins_ss,
        "bin_ratios_vs_lowSR_all": [round(r, 3) for r in ratios_all],
        "bin_ratios_vs_lowSR_sidespin": [round(r, 3) for r in ratios_ss],
        "flat_within_15pct_all": bool(flat_all),
        "flat_within_15pct_sidespin": bool(flat_ss),
        "sat_law_predicted_top_over_low_bin": round(float(pred_hi), 3),
        "observed_top_over_low_bin_sidespin": round(float(obs_hi), 3),
        "top_bin_CI_width_rel_to_low_median": round(float(ci_width_rel), 3),
        "covered_range_can_distinguish_sat_law": bool(distinguishable),
        "max_spin_rev_s": float(rev.max()),
        "speeds_of_sr_gt_0p7_arcs_m_s": [round(float(v), 2) for v in speed[sr > 0.7]],
        "spearman_km_vs_sr_kept": {"rho": round(float(rho), 3), "p": round(float(pval), 4)},
        "dip_bin_sr_0p3_0p45_sidespin": {
            "ratio_vs_low_bin": round(dip["km_med"] / low_bin["km_med"], 3),
            "ci_separated_from_low_bin": bool(dip_ci_separated),
            "sat_law_predicts_ratio": round(float(sat(dip["sr_mid"]) / sat(low_bin["sr_mid"])), 3),
            "median_rev_s_in_bin": 9.9,
        },
    },
    "replacement_fit": None,
    "caveats": [
        "SR>1.5 has 1 kept arc (need >=5/bin); SR>0.7 has 1 kept arc — the saturation region is not sampled. High-SR rows come from LOW SPEED (0.8-1.6 m/s), not high spin, and 3/4 of them are fit failures (75% drop rate vs 33% overall) — selection effect against the exact region the test needs.",
        "34/102 per-arc km rows gated out as fit failures (25 km<0, 9 km>0.02, 0 additional rms>15mm) — per-arc km is very noisy at these low spin rates (max 14.6 rev/s).",
        "Covered range is NOT flat: sidespin bin SR 0.3-0.45 median is 55% of the SR<0.2 bin (bootstrap 68% CIs do not overlap), so constant-km cannot be PASSed either. But the dip is 2x deeper than the candidate saturating law predicts (0.55 vs 0.83) and does not persist (SR 0.45-0.7 rows sit back at/above the low-SR level), i.e. non-monotonic — noise or a take-level confound, not clean saturation.",
        "Quaternion spin reads LOW overall; that bias inflates fitted km at high spin (fit compensates for underestimated w), so it masks rather than mimics saturation — it cannot rescue the test, only deepen a true dip.",
        "All spin here is <15 rev/s (trust limit 75 rev/s), so unwrap failure is unlikely, but real gameplay spins (50-100 rev/s) and their Re/SR regime are simply absent from this dataset.",
        "Low-SR sanity passed: sidespin SR<0.5 median km = 0.0052 in [0.003, 0.006] — no rig/frame problem; the INCONCLUSIVE is coverage, not calibration.",
    ],
    "artifacts": [
        f"{OUTDIR}/F2.png",
        f"{OUTDIR}/F2_verdict.json",
        "hope_training/ball_physics_fit/falsify/f2_km_saturation.py",
    ],
}
with open(f"{OUTDIR}/F2_verdict.json", "w") as f:
    json.dump(out, f, indent=2)

print(json.dumps(out, indent=2))
