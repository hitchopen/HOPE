"""Flight-model self-consistency check (no spin).

For every ballistic arc: fit the drag flight model on the FRONT window and
predict the BACK of the arc, and vice versa. The size of the disagreement is
compared against a matched Monte-Carlo noise floor (the same estimator applied
to a noise-free trajectory with synthetic capture noise added):

  * observed gap ~= floor -> the error is measurement noise through the window
    fit (shrinks with longer windows / more samples);
  * observed gap >> floor -> flight-model error (wrong k_d, unmodelled aero).

Usage:
    python flight_selfcheck.py [--window-ms 120] [--min-dur 0.35]
                               [--kd 0.1261] [--noise-mm 9] [--mc 100]
Writes analysis/fits/flight_selfcheck.json (+ .png if matplotlib is present).
"""
import argparse
import json
import os

import numpy as np

import paths
from ballcore import load_trajectory, extract_arcs, arc_parabola, arc_is_ballistic
from stage1_segments import window_fit, prop_state


def both_ways(t, pos, kd, W):
    """Front->back and back->front position gaps for one arc (reference = the
    opposite window's fit, so both sides are denoised the same way)."""
    t0, t1 = t[0], t[-1]
    mf, mb = t <= t0 + W, t >= t1 - W
    if mf.sum() < 4 or mb.sum() < 4:
        return None
    ff = window_fit(t[mf], pos[mf], kd)
    fb = window_fit(t[mb], pos[mb], kd)
    pf, _ = prop_state(ff, t1)
    pb_ref, _ = prop_state(fb, t1)
    pb, _ = prop_state(fb, t0)
    pf_ref, _ = prop_state(ff, t0)
    return dict(fwd=float(np.linalg.norm(pf - pb_ref)),
                bwd=float(np.linalg.norm(pb - pf_ref)),
                rms_f=ff["rms"], rms_b=fb["rms"])


def mc_floor(dur, speed, kd, W, noise_m, mc, rng):
    """Median front/back gap for a synthetic noise-free arc of this duration and
    launch speed, with white capture noise added, run through the same fit."""
    n = max(int(dur * 200), 20)
    t = np.linspace(0, dur, n)
    from ballcore import rk4_flight, G_NOM
    v0 = np.array([speed / np.sqrt(1.25), 0.0, speed * 0.5 / np.sqrt(1.25)])
    P = rk4_flight(np.zeros((1, 3)), v0[None], kd, np.array([0, 0, -G_NOM]),
                   t[1] - t[0], n - 1)[0]
    fw, bw = [], []
    for _ in range(mc):
        r = both_ways(t, P + rng.normal(0, noise_m, P.shape), kd, W)
        if r:
            fw.append(r["fwd"]); bw.append(r["bwd"])
    return (float(np.median(fw)) if fw else None,
            float(np.median(bw)) if bw else None)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--window-ms", type=float, default=120.0)
    ap.add_argument("--min-dur", type=float, default=0.35)
    ap.add_argument("--kd", type=float, default=0.1261)
    ap.add_argument("--noise-mm", type=float, default=9.0,
                    help="assumed per-axis capture noise for the MC floor")
    ap.add_argument("--mc", type=int, default=100)
    args = ap.parse_args()
    W = args.window_ms * 1e-3

    rows = []
    for path in paths.trajectory_files():
        traj = load_trajectory(path)
        for a in extract_arcs(traj):
            par = arc_parabola(a)
            if (a["t"][-1] - a["t"][0]) < args.min_dur or not arc_is_ballistic(a, par):
                continue
            r = both_ways(a["t"], a["pos"], args.kd, W)
            if r is None or max(r["rms_f"], r["rms_b"]) > 0.025:
                continue
            rows.append(dict(name=a["name"], dur=float(a["t"][-1] - a["t"][0]),
                             horizon=float(a["t"][-1] - a["t"][0] - W),
                             speed=float(np.linalg.norm(par["v0"])), **r))

    if not rows:
        print("no ballistic arcs long enough for the self-check "
              "(need arcs >= min-dur with clean window fits)")
        return

    rng = np.random.default_rng(0)
    fwd = np.array([r["fwd"] for r in rows])
    bwd = np.array([r["bwd"] for r in rows])
    reps = rows[:: max(1, len(rows) // 5)][:5]
    floor_fw, floor_bw = [], []
    for r in reps:
        ff, bb = mc_floor(r["dur"], r["speed"], args.kd, W,
                          args.noise_mm * 1e-3, args.mc, rng)
        if ff:
            floor_fw.append(ff); floor_bw.append(bb)
    floor = dict(fwd_mm=(float(np.median(floor_fw)) * 1e3 if floor_fw else None),
                 bwd_mm=(float(np.median(floor_bw)) * 1e3 if floor_bw else None))

    rep = dict(n_arcs=len(rows), window_ms=args.window_ms, kd=args.kd,
               noise_mm=args.noise_mm,
               fwd_gap_mm=dict(median=float(np.median(fwd)) * 1e3,
                               p90=float(np.percentile(fwd, 90)) * 1e3),
               bwd_gap_mm=dict(median=float(np.median(bwd)) * 1e3,
                               p90=float(np.percentile(bwd, 90)) * 1e3),
               mc_noise_floor=floor, rows=rows)
    os.makedirs(paths.FITS, exist_ok=True)
    out = os.path.join(paths.FITS, "flight_selfcheck.json")
    json.dump(rep, open(out, "w"), indent=1)
    print(json.dumps({k: v for k, v in rep.items() if k != "rows"}, indent=1))
    print(f"-> {out}")


if __name__ == "__main__":
    main()
