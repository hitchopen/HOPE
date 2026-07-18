#!/usr/bin/env python
"""F4: is paddle effective restitution e constant over normal contact speed |u_n|?

Fit the slope de/du_n across racket strikes (robust Theil-Sen + OLS), with a
within-file (demeaned) slope so a per-capture offset does not look like a speed
trend, and compare constant / linear / exponential restitution forms by residual
RMS. Verdict:
  * KILL         -- |de| over the covered range > 0.05 with a CI excluding it;
  * PASS         -- constant within +-0.025;
  * INCONCLUSIVE -- otherwise (thin coverage or CIs straddling the thresholds).

Needs strikes fit with paddle state, so it reads analysis/fits/stage2_fits.json
(stage2 -> paddle.per_strike); ball-only capture cannot fit the paddle contact.
Writes an F4 verdict JSON (+ PNG if matplotlib is available).
"""
import json
import os
import sys

import numpy as np
from scipy import stats
from scipy.optimize import least_squares

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import paths

FITS = os.path.join(paths.FITS, "stage2_fits.json")
OUTDIR = paths.FALSIFICATION
rng = np.random.default_rng(42)


def main():
    d = json.load(open(FITS))
    pad = d.get("paddle", {})
    rows = pad.get("per_strike")
    if not rows or len(rows) < 10:
        n = 0 if not rows else len(rows)
        print(f"F4: {n} paddle strikes; need >=10 (with racket tracking) for a verdict.")
        _write(dict(test="F4: paddle e constant over |u_n|", verdict="INCONCLUSIVE",
                    n=n, reason="too few strikes with paddle state"))
        return

    u_n = np.array([r["u_n"] for r in rows], float)
    e = np.array([r["e"] for r in rows], float)
    names = np.array([r.get("name", "?") for r in rows])
    keep = (e > -0.2) & (e < 1.5) & np.isfinite(e) & np.isfinite(u_n)
    u_n, e, names = u_n[keep], e[keep], names[keep]
    n = len(e)
    span = float(u_n.max() - u_n.min())

    ts = stats.theilslopes(e, u_n, 0.95)
    ols = stats.linregress(u_n, e)
    # bootstrap CI on the change across the covered range
    d_range = []
    for _ in range(2000):
        i = rng.integers(0, n, n)
        d_range.append(stats.theilslopes(e[i], u_n[i])[0] * span)
    ci = [float(np.percentile(d_range, 2.5)), float(np.percentile(d_range, 97.5))]
    # within-file demeaned slope
    un_c, e_c = u_n.copy(), e.copy()
    for nm in np.unique(names):
        m = names == nm
        un_c[m] -= u_n[m].mean(); e_c[m] -= e[m].mean()
    slope_within = float(stats.theilslopes(e_c, un_c)[0]) if len(np.unique(un_c)) > 2 else None

    # constant / linear / exponential residual RMS
    const_rms = float(np.sqrt(np.mean((e - e.mean()) ** 2)))
    lin = np.polyfit(u_n, e, 1)
    lin_rms = float(np.sqrt(np.mean((e - np.polyval(lin, u_n)) ** 2)))
    exp = least_squares(lambda p: p[0] * np.exp(p[1] * u_n) - e, [e.mean(), -0.04])
    exp_rms = float(np.sqrt(np.mean((e - exp.x[0] * np.exp(exp.x[1] * u_n)) ** 2)))

    d_range_ts = ts.slope * span
    ci_excl = (ci[0] * ci[1] > 0)
    if abs(d_range_ts) > 0.05 and ci_excl and abs(min(ci, key=abs)) > 0.05:
        verdict = "KILL"
    elif abs(d_range_ts) <= 0.025 and not ci_excl:
        verdict = "PASS"
    else:
        verdict = "INCONCLUSIVE"

    res = dict(
        test="F4: paddle e constant over |u_n|", verdict=verdict, n_used=n,
        u_n_coverage=[float(u_n.min()), float(u_n.max())], covered_span=span,
        theil_sen_slope_per_ms=float(ts.slope), ols_slope_per_ms=float(ols.slope),
        ols_p=float(ols.pvalue), within_file_slope_per_ms=slope_within,
        delta_e_over_covered_range=float(d_range_ts), delta_e_ci95=ci,
        residual_rms=dict(const=const_rms, linear=lin_rms, exp=exp_rms),
        linear_refit=dict(a=float(lin[1]), b=float(-lin[0])),
        exp_refit=dict(g1=float(exp.x[0]), g2=float(exp.x[1])))
    _write(res)
    _plot(u_n, e, ts, exp.x, verdict, span)
    print(json.dumps(res, indent=2))


def _write(res):
    os.makedirs(OUTDIR, exist_ok=True)
    json.dump(res, open(os.path.join(OUTDIR, "F4_verdict.json"), "w"), indent=2)


def _plot(u_n, e, ts, exp_x, verdict, span):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception:
        return
    fig, ax = plt.subplots(figsize=(8, 5.5))
    ax.scatter(u_n, e, s=24, alpha=0.7, c="tab:blue")
    xx = np.linspace(u_n.min(), u_n.max(), 60)
    ax.plot(xx, ts.intercept + ts.slope * xx, "r-", label=f"Theil-Sen {ts.slope:+.4f}/m/s")
    ax.plot(xx, exp_x[0] * np.exp(exp_x[1] * xx), "b:", label="exp fit")
    ax.axhline(float(np.mean(e)), color="gray", ls="-.", label=f"const e={np.mean(e):.3f}")
    ax.set_xlabel("|u_n| normal contact speed (m/s)"); ax.set_ylabel("paddle restitution e")
    ax.set_title(f"F4: paddle e vs |u_n| — {verdict}  (Δe over range {ts.slope*span:+.3f})")
    ax.legend(fontsize=8); ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(OUTDIR, "F4.png"), dpi=140)


if __name__ == "__main__":
    main()
