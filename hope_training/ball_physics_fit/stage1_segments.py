"""Stage 1 -- segmentation.

Read every canonical trajectory CSV under the data root and split each into:
  flights.json  -- one row per ballistic free-flight arc (t0, dur, v0, speed);
  bounces.json  -- one row per table bounce (v_in, v_out at contact, e_n, v_n);
  strikes.json  -- one row per racket strike (ball v_in / v_out; and the paddle
                   contact-point velocity + face normal IF a paddle sidecar is
                   present -- required later to fit the paddle contact).

Contact-time velocities are recovered with short windowed drag shooting fits
either side of the contact (never raw finite differences). No spin is used.

Paddle sidecar (optional): a file ``<name>.paddle.csv`` next to the ball CSV
with a ``t,x,y,z`` header (racket-face centre) and optionally ``nx,ny,nz``
(face-normal unit vector). Its contact-point velocity feeds the paddle fit.

    python stage1_segments.py            # uses BALLFIT_DATA_ROOT / sample_data
"""
import json
import os

import numpy as np
from scipy.optimize import least_squares

import paths
from ballcore import (R_BALL, G_NOM, load_trajectory, contiguous_runs, smooth_vel,
                      detect_contacts, extract_arcs, arc_parabola, arc_is_ballistic,
                      rk4_flight)

OUT = paths.SEGMENTS
KD_NOMINAL = 0.12          # nominal drag for short-window state estimates only;
                           # contact velocities are insensitive to it (drag is
                           # negligible over a ~6-frame window). Stage 2 fits k_d.
WIN = 7                    # frames used either side of a contact
EXCL = 2                   # frames skipped right next to the contact


def window_fit(t, pos, kd=KD_NOMINAL):
    """Drag shooting fit of (p0, v0) at t[0] over a short window."""
    g = np.array([0.0, 0.0, -G_NOM])
    dt = float(np.median(np.diff(t)))
    n = len(t) - 1
    v0g = np.polyfit(t - t[0], pos, 1)[0]

    def resid(x):
        P = rk4_flight(x[None, :3], x[None, 3:], kd, g, dt, n)[0]
        return (P - pos).ravel()

    sol = least_squares(resid, np.concatenate([pos[0], v0g]), method="lm", max_nfev=200)
    rms = float(np.sqrt((sol.fun.reshape(-1, 3) ** 2).sum(1).mean()))
    return dict(p0=sol.x[:3], v0=sol.x[3:], t0=float(t[0]), rms=rms, kd=kd)


def prop_state(f, t_eval):
    """Position + velocity at t_eval from a window_fit result (extrapolates)."""
    g = np.array([0.0, 0.0, -G_NOM])
    T = t_eval - f["t0"]
    m = max(int(abs(T) / 0.002), 2)
    P = rk4_flight(f["p0"][None], f["v0"][None], f["kd"], g, T / m, m)[0]
    # velocity by one more small step difference
    P2 = rk4_flight(f["p0"][None], f["v0"][None], f["kd"], g, T / m, m + 1)[0]
    v = (P2[-1] - P2[-2]) / (T / m)
    return P[-1], v


def contact_states(t, pos, idx):
    """Fit windows before/after a contact at sample ``idx`` and evaluate the ball
    velocity at the contact time. Returns (v_in, v_out, p_contact, rms) or None."""
    n = len(t)
    lo0, lo1 = max(0, idx - EXCL - WIN), idx - EXCL
    hi0, hi1 = idx + EXCL, min(n, idx + EXCL + WIN)
    if lo1 - lo0 < 4 or hi1 - hi0 < 4:
        return None
    t_c = float(t[idx])
    fa = window_fit(t[lo0:lo1], pos[lo0:lo1])
    fb = window_fit(t[hi0:hi1], pos[hi0:hi1])
    p_in, v_in = prop_state(fa, t_c)
    _, v_out = prop_state(fb, t_c)
    return dict(v_in=v_in, v_out=v_out, p_c=p_in,
                rms=float(max(fa["rms"], fb["rms"])))


def load_paddle(ball_path):
    """Optional paddle sidecar loader -> dict(t, pos, normal) or None."""
    side = os.path.splitext(ball_path)[0] + ".paddle.csv"
    if not os.path.exists(side):
        return None
    raw = np.genfromtxt(side, delimiter=",", names=True)
    t = np.asarray(raw["t"], float)
    pos = np.column_stack([raw["x"], raw["y"], raw["z"]]).astype(float)
    normal = None
    if all(c in raw.dtype.names for c in ("nx", "ny", "nz")):
        normal = np.column_stack([raw["nx"], raw["ny"], raw["nz"]]).astype(float)
    order = np.argsort(t)
    return dict(t=t[order], pos=pos[order],
                normal=(normal[order] if normal is not None else None))


def paddle_state_at(pad, t_c):
    """Racket contact-point velocity and face normal at t_c (const-velocity fit on
    a short window). Returns dict(v, n) or None."""
    t, pos = pad["t"], pad["pos"]
    m = (t >= t_c - 0.05) & (t <= t_c + 0.05)
    if m.sum() < 4:
        return None
    tw = t[m] - t_c
    v = np.polyfit(tw, pos[m], 1)[0]           # d(pos)/dt
    n = None
    if pad["normal"] is not None:
        n = np.nanmean(pad["normal"][m], axis=0)
        nn = np.linalg.norm(n)
        n = n / nn if nn > 1e-9 else None
    return dict(v=v, n=n)


def main():
    os.makedirs(OUT, exist_ok=True)
    flights, bounces, strikes = [], [], []

    for path in paths.trajectory_files():
        traj = load_trajectory(path)
        name = traj["name"]

        for ai, a in enumerate(extract_arcs(traj)):
            par = arc_parabola(a)
            f = window_fit(a["t"], a["pos"]) if len(a["t"]) >= 5 else None
            v0 = f["v0"] if f else par["v0"]
            flights.append(dict(
                name=name, arc=ai, i0=int(a["i0"]), i1=int(a["i1"]),
                t0=float(a["t"][0]), dur=float(a["t"][-1] - a["t"][0]), n=len(a["t"]),
                v0=[float(x) for x in v0], speed0=float(np.linalg.norm(v0)),
                g_parab=float(par["g"]), parab_rms_mm=float(par["rms"] * 1e3),
                ballistic=bool(arc_is_ballistic(a, par)),
                pre=a["pre_contact"], post=a["post_contact"]))

        pad = load_paddle(path)
        for a, b in contiguous_runs(traj["t"], traj["rate"]):
            t, pos = traj["t"][a:b + 1], traj["pos"][a:b + 1]
            sp, sv = smooth_vel(t, pos)
            for idx, kind in detect_contacts(t, sp, sv, traj["rate"]):
                st = contact_states(t, pos, idx)
                if st is None or st["rms"] > 0.030:
                    continue
                v_in, v_out = st["v_in"], st["v_out"]
                if kind == "table":
                    if v_in[2] >= 0 or v_out[2] <= 0:
                        continue
                    ut = float(np.linalg.norm(v_in[:2]))
                    bounces.append(dict(
                        name=name, t_c=float(t[idx]),
                        p_c=[float(x) for x in st["p_c"]],
                        v_in=[float(x) for x in v_in], v_out=[float(x) for x in v_out],
                        e_n=float(-v_out[2] / v_in[2]), vn_in=float(-v_in[2]),
                        ut_over_un=float(ut / max(-v_in[2], 1e-6)),
                        fit_rms_mm=float(st["rms"] * 1e3)))
                else:  # racket strike
                    row = dict(
                        name=name, t_c=float(t[idx]),
                        ball_p=[float(x) for x in st["p_c"]],
                        v_in=[float(x) for x in v_in], v_out=[float(x) for x in v_out],
                        fit_rms_mm=float(st["rms"] * 1e3),
                        pad_v=None, pad_n=None)
                    if pad is not None:
                        ps = paddle_state_at(pad, float(t[idx]))
                        if ps is not None and ps["n"] is not None:
                            n = ps["n"]
                            if np.dot(np.array(v_in) - ps["v"], n) > 0:
                                n = -n
                            row["pad_v"] = [float(x) for x in ps["v"]]
                            row["pad_n"] = [float(x) for x in n]
                    strikes.append(row)

        print(f"{name}: flights={len(flights)} bounces={len(bounces)} strikes={len(strikes)}")

    json.dump(flights, open(os.path.join(OUT, "flights.json"), "w"), indent=1)
    json.dump(bounces, open(os.path.join(OUT, "bounces.json"), "w"), indent=1)
    json.dump(strikes, open(os.path.join(OUT, "strikes.json"), "w"), indent=1)
    n_pad = sum(1 for s in strikes if s["pad_n"] is not None)
    print(f"\nTOTAL: {len(flights)} flights, {len(bounces)} bounces, "
          f"{len(strikes)} strikes ({n_pad} with paddle state)")
    print(f"-> {OUT}/")


if __name__ == "__main__":
    main()
