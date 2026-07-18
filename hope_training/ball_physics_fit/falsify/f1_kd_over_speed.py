#!/usr/bin/env python
"""F1: is the drag coefficient k_d constant over launch speed?

A quadratic-drag model assumes a single k_d at all speeds. Bin the per-arc k_d
by launch speed, compare bin medians to the global fit, and test for a monotonic
trend. Verdict:
  * KILL         -- a significant monotonic trend, or a bin median off the global
                    fit by more than 20%;
  * PASS         -- all bins within 10% and no trend;
  * INCONCLUSIVE -- otherwise (usually too little / too noisy data).

Reads analysis/fits/stage2_fits.json (stage2 -> kd.per_arc). Writes an F1
verdict JSON (and a PNG if matplotlib is available).
"""
import json
import os
import sys

import numpy as np
from scipy import stats

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import paths

FITS = os.path.join(paths.FITS, "stage2_fits.json")
OUTDIR = paths.FALSIFICATION


def main():
    d = json.load(open(FITS))
    kd = d.get("kd", {})
    pa = kd.get("per_arc")
    if not pa or "kd" not in kd:
        print("F1: no per-arc k_d in stage2 output (need more ballistic arcs).")
        return
    kd_global = kd["kd"]
    speed = np.array([r["speed"] for r in pa])
    kdv = np.array([r["kd"] for r in pa])
    rms = np.array([r["rms_mm"] for r in pa])

    keep = (kdv > 0.02) & (kdv <= 0.35) & (rms <= 25.0)
    s, k = speed[keep], kdv[keep]
    if len(k) < 8:
        print(f"F1: only {len(k)} usable arcs after gating; need more for a verdict.")
        _write(dict(test="F1: k_d constant over launch speed", verdict="INCONCLUSIVE",
                    reason=f"only {len(k)} usable arcs", n_used=int(len(k))))
        return

    edges = [1.0, 2.0, 3.0, 4.0, 5.0, 99.0]
    bins = []
    for lo, hi in zip(edges[:-1], edges[1:]):
        m = (s >= lo) & (s < hi)
        if m.sum() == 0:
            continue
        med = float(np.median(k[m]))
        bins.append(dict(lo=lo, hi=hi, n=int(m.sum()), median=med,
                         dev_vs_global_pct=100.0 * (med - kd_global) / kd_global))

    rho, p_rho = stats.spearmanr(s, k)
    ts = stats.theilslopes(k, s, 0.95)
    span = float(s.max() - s.min())
    rel_change_pct = 100.0 * ts.slope * span / kd_global

    solid = [b for b in bins if b["n"] >= 5]
    max_dev = max((abs(b["dev_vs_global_pct"]) for b in solid), default=0.0)
    meds = [b["median"] for b in solid]
    monotone = len(meds) >= 3 and (all(np.diff(meds) > 0) or all(np.diff(meds) < 0))
    sig_trend = (p_rho < 0.05) and (abs(rel_change_pct) > 20.0) and monotone

    if sig_trend or max_dev > 20.0:
        verdict = "KILL"
    elif max_dev <= 10.0 and not (p_rho < 0.05 and abs(rel_change_pct) > 10.0):
        verdict = "PASS"
    else:
        verdict = "INCONCLUSIVE"

    res = dict(
        test="F1: k_d constant over launch speed",
        verdict=verdict,
        kd_global_fit=round(float(kd_global), 4),
        n_used=int(len(k)),
        speed_coverage=[round(float(s.min()), 2), round(float(s.max()), 2)],
        spearman_rho=round(float(rho), 3), spearman_p=round(float(p_rho), 4),
        theilsen_rel_change_over_window_pct=round(float(rel_change_pct), 1),
        max_bin_dev_pct=round(float(max_dev), 1),
        bins=[{k2: (round(v, 4) if isinstance(v, float) else v)
               for k2, v in b.items()} for b in bins])
    _write(res)
    _plot(s, k, kd_global, bins, verdict, rho, p_rho)
    print(json.dumps(res, indent=2))


def _write(res):
    os.makedirs(OUTDIR, exist_ok=True)
    json.dump(res, open(os.path.join(OUTDIR, "F1_verdict.json"), "w"), indent=2)


def _plot(s, k, kd_global, bins, verdict, rho, p_rho):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception:
        return
    fig, ax = plt.subplots(figsize=(9, 6))
    ax.scatter(s, k, s=30, c="tab:blue", alpha=0.6, label=f"per-arc k_d (n={len(k)})")
    for b in bins:
        xc = 0.5 * (b["lo"] + min(b["hi"], s.max()))
        ax.plot(xc, b["median"], "s", ms=9, c="crimson")
        ax.annotate(f"n={b['n']}\n{b['dev_vs_global_pct']:+.0f}%", (xc, b["median"]),
                    textcoords="offset points", xytext=(8, 8), fontsize=9, color="crimson")
    ax.axhline(kd_global, color="k", lw=1.5, label=f"global fit k_d={kd_global:.3f}")
    ax.axhspan(kd_global * 0.9, kd_global * 1.1, color="green", alpha=0.10, label="±10%")
    ax.set_xlabel("launch speed (m/s)"); ax.set_ylabel("per-arc k_d")
    ax.set_title(f"F1: k_d vs launch speed — {verdict}  (Spearman rho={rho:.2f}, p={p_rho:.3f})")
    ax.legend(fontsize=8); ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(OUTDIR, "F1.png"), dpi=130)


if __name__ == "__main__":
    main()
