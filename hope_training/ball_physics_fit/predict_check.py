"""Forward flight-prediction accuracy check (no spin).

The deploy-relevant question: given a short early view of a ball in flight, how
accurately does the fitted drag model predict where it goes next? For every
ballistic arc, fit the flight model on the first ``--window-ms`` and integrate
it forward over the rest of the arc, then report the position error binned by
prediction horizon (time since the fit window).

This uses ONLY the ball's own trajectory and the flight (drag + gravity) model.
There is no paddle model, no spin, and no landing-target substitute -- it
measures pure flight-prediction accuracy.

Usage:
    python predict_check.py [--window-ms 100] [--min-dur 0.30] [--kd 0.1261]
Writes analysis/fits/predict_check.json.
"""
import argparse
import json
import os

import numpy as np

import paths
from ballcore import load_trajectory, extract_arcs, arc_parabola, arc_is_ballistic
from stage1_segments import window_fit, prop_state

BINS = ((0.0, 0.1, "0-100ms"), (0.1, 0.2, "100-200ms"), (0.2, 0.3, "200-300ms"),
        (0.3, 0.5, "300-500ms"), (0.5, np.inf, ">500ms"))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--window-ms", type=float, default=100.0)
    ap.add_argument("--min-dur", type=float, default=0.30)
    ap.add_argument("--kd", type=float, default=0.1261)
    args = ap.parse_args()
    W = args.window_ms * 1e-3

    pool = {lbl: [] for _, _, lbl in BINS}     # planar errors (m) per horizon bin
    rows = []
    for path in paths.trajectory_files():
        traj = load_trajectory(path)
        for a in extract_arcs(traj):
            t, pos = a["t"], a["pos"]
            if (t[-1] - t[0]) < args.min_dur or not arc_is_ballistic(a, arc_parabola(a)):
                continue
            mf = t <= t[0] + W
            if mf.sum() < 4:
                continue
            f = window_fit(t[mf], pos[mf], args.kd)
            if f["rms"] > 0.025:
                continue
            future = np.where(t > t[0] + W)[0]
            errs = []
            for i in future:
                p_pred, _ = prop_state(f, float(t[i]))
                e = float(np.linalg.norm(p_pred[:2] - pos[i, :2]))
                horizon = float(t[i] - (t[0] + W))
                errs.append((horizon, e))
                for lo, hi, lbl in BINS:
                    if lo <= horizon < hi:
                        pool[lbl].append(e)
            if errs:
                rows.append(dict(name=a["name"], dur=float(t[-1] - t[0]),
                                 window_rms_mm=float(f["rms"] * 1e3),
                                 max_horizon_ms=float(errs[-1][0] * 1e3),
                                 max_err_mm=float(errs[-1][1] * 1e3)))

    def q(v):
        v = np.asarray(v, float)
        return (dict(n=int(len(v)), median_mm=float(np.median(v)) * 1e3,
                     p90_mm=float(np.percentile(v, 90)) * 1e3) if len(v) else dict(n=0))

    if not rows:
        print("no ballistic arcs long enough for the forward-prediction check")
        return

    rep = dict(n_arcs=len(rows), window_ms=args.window_ms, kd=args.kd,
               error_by_horizon={lbl: q(pool[lbl]) for _, _, lbl in BINS}, rows=rows)
    os.makedirs(paths.FITS, exist_ok=True)
    out = os.path.join(paths.FITS, "predict_check.json")
    json.dump(rep, open(out, "w"), indent=1)
    print(json.dumps({k: v for k, v in rep.items() if k != "rows"}, indent=1))
    print(f"-> {out}")


if __name__ == "__main__":
    main()
