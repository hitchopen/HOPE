#!/usr/bin/env python
"""F3: is table normal restitution e_n constant over impact speed v_n?

Fit the slope de/dv_n across the table bounces and test it against a flat model.
Verdict:
  * KILL         -- |slope| > 0.01 /m/s with a bootstrap CI that excludes zero;
  * PASS         -- flat within +-0.025 of the constant over the covered range;
  * INCONCLUSIVE -- otherwise.

A within-file (demeaned) slope is also reported so a per-capture confound does
not masquerade as a speed trend. Reads analysis/fits/stage2_fits.json
(stage2 -> table_e.per_bounce). Writes an F3 diagnostics JSON (+ PNG if possible).
"""
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import paths

FITS = os.path.join(paths.FITS, "stage2_fits.json")
OUTDIR = paths.FALSIFICATION
rng = np.random.default_rng(42)


def ols_slope(x, y):
    return np.linalg.lstsq(np.vstack([np.ones_like(x), x]).T, y, rcond=None)[0]


def boot_ci(x, y, n=2000):
    out = []
    for _ in range(n):
        i = rng.integers(0, len(x), len(x))
        if len(np.unique(x[i])) < 3:
            continue
        out.append(ols_slope(x[i], y[i])[1])
    return [float(np.percentile(out, 2.5)), float(np.percentile(out, 97.5))] if out else [None, None]


def main():
    d = json.load(open(FITS))
    pb = d.get("table_e", {}).get("per_bounce")
    if not pb or len(pb) < 10:
        print(f"F3: {0 if not pb else len(pb)} bounces; need >=10 across a range of v_n.")
        _write(dict(test="F3: table e_n constant over v_n", verdict="INCONCLUSIVE",
                    n=(0 if not pb else len(pb)), reason="too few bounces"))
        return

    vn = np.array([b["vn"] for b in pb])
    e = np.array([b["e"] for b in pb])
    names = np.array([b.get("name", "?") for b in pb])

    coef = ols_slope(vn, e)
    ci = boot_ci(vn, e)
    # within-file demeaned slope (removes per-capture offsets)
    vn_c, e_c = vn.astype(float).copy(), e.astype(float).copy()
    for nm in np.unique(names):
        m = names == nm
        vn_c[m] -= vn[m].mean(); e_c[m] -= e[m].mean()
    slope_within = float(ols_slope(vn_c, e_c)[1])

    edges = np.arange(np.floor(vn.min() * 2) / 2, np.ceil(vn.max() * 2) / 2 + 0.5, 0.5)
    bins = []
    for lo, hi in zip(edges[:-1], edges[1:]):
        m = (vn >= lo) & (vn < hi)
        if m.sum() >= 3:
            bins.append(dict(lo=float(lo), hi=float(hi), n=int(m.sum()),
                             vn_med=float(np.median(vn[m])), e_med=float(np.median(e[m]))))

    slope = float(coef[1])
    ci_excl_zero = ci[0] is not None and (ci[0] * ci[1] > 0)
    if abs(slope) > 0.01 and ci_excl_zero:
        verdict = "KILL"
    elif abs(slope) <= 0.025 / max(vn.max() - vn.min(), 1e-6) and not ci_excl_zero:
        verdict = "PASS"
    else:
        verdict = "INCONCLUSIVE"

    res = dict(
        test="F3: table e_n constant over v_n", verdict=verdict, n=len(e),
        e_const=float(np.median(e)),
        ols_slope_per_ms=slope, ols_intercept=float(coef[0]), ols_slope_ci95=ci,
        within_file_slope_per_ms=slope_within,
        vn_coverage=[float(vn.min()), float(vn.max())], bins=bins)
    _write(res)
    _plot(vn, e, coef, bins, verdict)
    print(json.dumps(res, indent=2))


def _write(res):
    os.makedirs(OUTDIR, exist_ok=True)
    json.dump(res, open(os.path.join(OUTDIR, "F3_diag.json"), "w"), indent=2)


def _plot(vn, e, coef, bins, verdict):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception:
        return
    fig, ax = plt.subplots(figsize=(8, 5.5))
    ax.scatter(vn, e, s=26, alpha=0.7, c="tab:blue")
    if bins:
        ax.plot([b["vn_med"] for b in bins], [b["e_med"] for b in bins],
                "s-", color="crimson", ms=7, label="binned median")
    xx = np.linspace(vn.min(), vn.max(), 40)
    ax.plot(xx, coef[0] + coef[1] * xx, "b--", label=f"OLS slope {coef[1]:+.4f}/m/s")
    ax.axhline(float(np.median(e)), color="gray", ls=":", label=f"median e={np.median(e):.3f}")
    ax.set_xlabel("v_n impact (m/s)"); ax.set_ylabel("e_n = v_n,out / v_n,in")
    ax.set_title(f"F3: table restitution vs impact speed — {verdict}")
    ax.legend(fontsize=8); ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(OUTDIR, "F3.png"), dpi=140)


if __name__ == "__main__":
    main()
