"""
Flight-model self-consistency check: predict the BACK of each ballistic arc from
its FRONT window (forward) and the FRONT from its BACK window (backward), on real
tracked arcs. Decomposes the disagreement into measurement-noise variance (via a
matched Monte-Carlo noise floor using the forensics noise model) and model-form
error (the excess over that floor).

Reading the output:
  - observed ~= floor        -> the error is VARIANCE (mocap noise through the
                                window fit); longer windows shrink it.
  - observed >> floor        -> model-form/bias error (wrong k_d/k_m for that arc,
                                spin decay, unmodeled aero) — NOT fixable by
                                averaging; grows with horizon.
  - backward > forward       -> expected slightly (drag damps velocity-error
                                growth forward, amplifies it backward, ~e^{kd|v|T});
                                a LARGE asymmetry beyond the floor's own asymmetry
                                points at speed-dependent model error.

Usage: python flight_selfcheck.py [--yaml ../../configs/ball_physics_venue.yaml]
                                  [--window-ms 100] [--min-dur 0.35] [--mc 100]
Writes analysis/fits/flight_selfcheck.json + flight_selfcheck.png.
"""
import os, sys, json, argparse
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import paths
from ballcore import (TAKES, load_take, extract_arcs, arc_parabola, arc_is_ballistic,
                      spin_from_quats)
from stage1_segments import window_fit, rk4_window, G_NOM

ANA = paths.ANALYSIS
# forensics noise model (report §9.3): per-axis white + AR(1) colored, 300 Hz
NOISE_WHITE = 0.0019
NOISE_AR_MARG = 0.0052
NOISE_AR_RHO = 0.946


def prop(f, t_eval):
    """Position+velocity at t_eval from a window_fit result (either direction)."""
    g = np.array([0, 0, -G_NOM])
    T = t_eval - f["t0"]
    n = max(int(abs(T) / 0.0017), 2)
    P, V = rk4_window(f["p0"], f["v0"], f["w"], f["kd"], f["km"], g, T / n, n)
    return P[-1], V[-1]


def gapfree(t, rate):
    return len(t) >= 15 and np.max(np.diff(t)) < 1.5 / rate


def colored_noise(n, rng):
    w = rng.normal(0, NOISE_WHITE, (n, 3))
    innov = NOISE_AR_MARG * np.sqrt(1 - NOISE_AR_RHO ** 2)
    ar = np.empty((n, 3))
    ar[0] = rng.normal(0, NOISE_AR_MARG, 3)
    for i in range(1, n):
        ar[i] = NOISE_AR_RHO * ar[i - 1] + rng.normal(0, innov, 3)
    return w + ar


def both_ways(t, pos, w_f, w_b, kd, km, W):
    """Fit front/back windows of one arc; return fwd/bwd position+velocity gaps.
    Reference for each direction = the OPPOSITE window's fit evaluated at the same
    time (denoised); raw-endpoint distances are reported alongside."""
    t0, t1 = t[0], t[-1]
    mf = t <= t0 + W
    mb = t >= t1 - W
    ff = window_fit(t[mf], pos[mf], w_f, kd=kd, km=km)
    fb = window_fit(t[mb], pos[mb], w_b, kd=kd, km=km)
    pf, vf = prop(ff, t1)             # forward: front fit -> arc end
    pb_ref, vb_ref = prop(fb, t1)
    pb, vb = prop(fb, t0)             # backward: back fit -> arc start
    pf_ref, vf_ref = prop(ff, t0)
    return dict(
        fwd_pos=float(np.linalg.norm(pf - pb_ref)),
        bwd_pos=float(np.linalg.norm(pb - pf_ref)),
        fwd_pos_raw=float(np.linalg.norm(pf - pos[-1])),
        bwd_pos_raw=float(np.linalg.norm(pb - pos[0])),
        fwd_vel=float(np.linalg.norm(vf - vb_ref)),
        bwd_vel=float(np.linalg.norm(vb - vf_ref)),
        rms_f_mm=ff["rms"] * 1e3, rms_b_mm=fb["rms"] * 1e3)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--yaml", default=None)
    ap.add_argument("--window-ms", type=float, default=100.0)
    ap.add_argument("--min-dur", type=float, default=0.35)
    ap.add_argument("--mc", type=int, default=100, help="MC draws per floor arc")
    ap.add_argument("--out", default=os.path.join(ANA, "fits", "flight_selfcheck.json"))
    args = ap.parse_args()
    W = args.window_ms * 1e-3

    kd, km = 0.1261, 0.00444
    if args.yaml:
        from validate_stage4 import load_yaml_constants
        Y = load_yaml_constants(args.yaml)
        kd, km = Y["flight.k_d"], Y["flight.k_m"]

    meta_p = os.path.join(ANA, "segments", "meta.json")
    qa_p = os.path.join(ANA, "qa_stage0.json")
    if os.path.exists(meta_p):
        surface_z = json.load(open(meta_p))["surface_z"]
    elif os.path.exists(qa_p):
        qa = json.load(open(qa_p))
        surface_z = float(np.median([v["surface_z"] for v in
                                     qa["surface_calibration"].values()
                                     if v.get("surface_z") is not None]))
    else:
        raise FileNotFoundError(
            f"neither {meta_p} nor {qa_p} found — run stage1/qa first (refusing to guess surface_z)")

    rows = []
    for name in TAKES:
        try:
            take = load_take(name)
        except FileNotFoundError:
            print(f"{name}: npz missing, skipped")
            continue
        rate = float(take["rate"])
        Rt = take["table_R"]
        quat_all = take["ball_quat_xyzw"].astype(float)
        for a in extract_arcs(take, table_z=surface_z):
            t, pos = a["t"], a["pos"]
            dur = t[-1] - t[0]
            if dur < args.min_dur or not arc_is_ballistic(a, arc_parabola(a)):
                continue
            mf = t <= t[0] + W
            mb = t >= t[-1] - W
            if not (gapfree(t[mf], rate) and gapfree(t[mb], rate)):
                continue
            iq = np.searchsorted(take["t"], t)      # arc times -> global frame indices
            _, _, w_f = spin_from_quats(quat_all[iq[mf]], rate, R_table=Rt)
            _, _, w_b = spin_from_quats(quat_all[iq[mb]], rate, R_table=Rt)
            w_f = w_f if np.isfinite(w_f).all() else np.zeros(3)
            w_b = w_b if np.isfinite(w_b).all() else np.zeros(3)
            r = both_ways(t, pos, w_f, w_b, kd, km, W)
            if r["rms_f_mm"] > 12 or r["rms_b_mm"] > 12:   # bad window fit
                continue
            speed = float(np.linalg.norm(window_fit(t[mf], pos[mf], w_f, kd, km)["v0"]))
            rows.append(dict(take=name, t0=float(t[0]), dur=float(dur),
                             horizon=float(dur - W), speed=speed,
                             spin_revs=float(np.linalg.norm(w_f) / 2 / np.pi), **r))
        print(f"{name}: arcs kept so far {len(rows)}")

    if not rows:
        raise RuntimeError("no usable arcs — check data root / gates")

    # ---- matched MC noise floor: same estimator, same geometry, exact-model truth
    rng = np.random.default_rng(20260703)
    bins = [(0.15, 0.25), (0.25, 0.35), (0.35, 0.5), (0.5, 2.0)]
    floors = {}
    for lo, hi in bins:
        cand = [r for r in rows if lo <= r["horizon"] < hi]
        if not cand:
            continue
        reps = cand[:: max(1, len(cand) // 5)][:5]
        fw, bw = [], []
        for r in reps:
            n = int(round((r["horizon"] + W) * 300)) + 1
            t = np.arange(n) / 300.0
            v0 = np.array([r["speed"], 0, 2.0])
            v0 = v0 / np.linalg.norm(v0) * r["speed"]
            w = np.array([0, 0, r["spin_revs"] * 2 * np.pi])
            g = np.array([0, 0, -G_NOM])
            P, _ = rk4_window(np.zeros(3), v0, w, kd, km, g, t[-1] / (n - 1), n - 1)
            for _ in range(args.mc):
                obs = P + colored_noise(n, rng)
                rr = both_ways(t, obs, w, w, kd, km, W)
                fw.append(rr["fwd_pos"]); bw.append(rr["bwd_pos"])
        floors[f"{lo}-{hi}"] = dict(fwd=float(np.median(fw)), bwd=float(np.median(bw)))

    def agg(key, lo, hi):
        v = np.array([r[key] for r in rows if lo <= r["horizon"] < hi])
        return (float(np.median(v)), float(np.percentile(v, 90)), len(v)) if len(v) \
            else (None, None, 0)

    print(f"\n=== flight self-consistency (window {args.window_ms:.0f} ms, "
          f"{len(rows)} arcs, kd={kd}, km={km}) ===")
    print(f"{'horizon':>10s} {'n':>4s} | {'FWD med/p90 mm':>18s} {'floor':>6s} | "
          f"{'BWD med/p90 mm':>18s} {'floor':>6s} | excess_fwd")
    summary = {}
    for lo, hi in bins:
        m, p, n = agg("fwd_pos", lo, hi)
        mb, pb, _ = agg("bwd_pos", lo, hi)
        if not n:
            continue
        fl = floors.get(f"{lo}-{hi}", {})
        exc = np.sqrt(max(m ** 2 - fl.get("fwd", 0) ** 2, 0)) if fl else None
        summary[f"{lo}-{hi}"] = dict(n=n, fwd_med=m, fwd_p90=p, bwd_med=mb, bwd_p90=pb,
                                     floor=fl, excess_fwd=exc)
        print(f"{lo:.2f}-{hi:.2f}s {n:4d} | {m*1e3:8.1f}/{p*1e3:6.1f} mm "
              f"{fl.get('fwd', float('nan'))*1e3:6.1f} | {mb*1e3:8.1f}/{pb*1e3:6.1f} mm "
              f"{fl.get('bwd', float('nan'))*1e3:6.1f} | "
              f"{'' if exc is None else f'{exc*1e3:.1f} mm'}")
    ratio = np.array([r["bwd_pos"] / r["fwd_pos"] for r in rows if r["fwd_pos"] > 1e-6])
    fv = np.median([r["fwd_vel"] for r in rows])
    bv = np.median([r["bwd_vel"] for r in rows])
    print(f"\npaired BWD/FWD ratio median {np.median(ratio):.2f} "
          f"(IQR {np.percentile(ratio,25):.2f}-{np.percentile(ratio,75):.2f})")
    print(f"velocity gap median: fwd {fv:.3f} m/s, bwd {bv:.3f} m/s")

    os.makedirs(os.path.dirname(args.out) or ".", exist_ok=True)
    json.dump(dict(window_ms=args.window_ms, kd=kd, km=km, n_arcs=len(rows),
                   summary=summary, bwd_fwd_ratio_med=float(np.median(ratio)),
                   vel_gap_fwd=fv, vel_gap_bwd=bv,
                   noise_model=dict(white=NOISE_WHITE, ar_marg=NOISE_AR_MARG,
                                    ar_rho=NOISE_AR_RHO),
                   rows=rows), open(args.out, "w"), indent=1)

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(1, 2, figsize=(11, 4.5))
    h = [r["horizon"] for r in rows]
    ax[0].scatter(h, [r["fwd_pos"] * 1e3 for r in rows], s=10, label="forward", alpha=.6)
    ax[0].scatter(h, [r["bwd_pos"] * 1e3 for r in rows], s=10, label="backward", alpha=.6)
    for k, fl in floors.items():
        lo, hi = map(float, k.split("-"))
        ax[0].hlines(fl["fwd"] * 1e3, lo, hi, colors="k", ls=":", lw=1)
    ax[0].set_xlabel("horizon (s)"); ax[0].set_ylabel("position gap (mm)")
    ax[0].set_title("front->back / back->front self-consistency (dots) vs noise floor (dotted)")
    ax[0].legend(); ax[0].set_ylim(0, np.percentile([r["bwd_pos"] for r in rows], 97) * 1e3)
    ax[1].hist(ratio, bins=30)
    ax[1].axvline(1.0, color="k", ls=":")
    ax[1].set_xlabel("BWD/FWD paired ratio"); ax[1].set_title("direction asymmetry")
    fig.tight_layout()
    fig.savefig(args.out.replace(".json", ".png"), dpi=110)
    print(f"-> {args.out}")


if __name__ == "__main__":
    main()
