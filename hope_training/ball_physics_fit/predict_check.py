"""
Two-horizon landing-prediction check — the deploy-style accuracy probe.

For every racket strike that has an OBSERVED first bounce (continuity-gated
pairing, ground truth = the bounce contact point p_c from bounces.json):

  H0  at-contact, through-paddle : measured pre-contact ball + racket state ->
      paddle contact model -> flight model -> predicted landing.
      (what the planner/reward will do the instant the ball leaves the racket)
  H1  at-contact, measured-out   : measured post-contact ball state -> flight
      model -> landing. Isolates flight-model + state noise from paddle-model error.
  H2  near-landing (~100 ms lead): ball state re-fit on a short window ending
      ~30 ms before touchdown -> flight model -> landing.
      (the late-refinement prediction; should be the tightest)

Reports per-horizon planar landing error + timing error, per-take breakdown,
and writes analysis/fits/predict_check.json + predict_check.png.

Usage: python predict_check.py [--yaml configs/ball_physics_venue.yaml]
                               [--paddle-e exp|const|lin] [--split all]
                               [--full-state] [--magnus const|sat]

--full-state: FULL-STATE trajectory validation (not just the landing point):
position error along the whole predicted arc vs horizon, velocity + spin at
checkpoints, net-plane (x=0) crossing state. Writes predict_fullstate.json/.png.
"""
import os, sys, json, argparse
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import paths
from ballcore import TAKES, load_take, spin_from_quats, R_BALL, G_NOM
from stage1_segments import window_fit, prop_state
from stage2_fits import in_split
from validate_stage4 import (load_yaml_constants, integrate_to_table, paddle_outgoing,
                             pair_strike_bounce, ANA)


def h2_state(take, Rt, t_b, rate, kd, km, lead_end_s=0.017, lead_start_s=0.150):
    """Ball state shortly before touchdown: shooting fit on the window
    [t_b - lead_start, t_b - lead_end] (ends before the contact exclusion zone)."""
    t = take["t"]
    pos = take["ball_pos_t_m"].astype(float)
    quat = take["ball_quat_xyzw"].astype(float)
    ok = take["ball_present"].astype(bool)
    m = ok & (t >= t_b - lead_start_s) & (t <= t_b - lead_end_s)
    if m.sum() < 15:
        return None
    _, _, w_rob = spin_from_quats(quat[m], rate, R_table=Rt)
    w = w_rob if np.isfinite(w_rob).all() else np.zeros(3)
    f = window_fit(t[m], pos[m], w, kd=kd, km=km)
    t_end = float(t[m][-1])
    return dict(fit=f, w=w, t_end=t_end, lead_s=float(t_b - t_end))


def terminal_window_gt(take, Rt, t_strike, rate, kd, km, surface_z,
                       max_extrap_s=0.20, max_horizon_s=2.0):
    """Landing ground truth for strikes with NO recorded bounce: take the LAST
    tracked descending samples of the post-strike ball path and extrapolate the
    short remaining distance to the surface (same machinery as H2; the bounce
    extractor misses ~2/3 of first bounces because the post-bounce arc is
    occluded/short, but the incoming arc's terminal window is usually there).
    Returns dict(p_land(3), t_land, lead_s) or None."""
    t = take["t"]
    pos = take["ball_pos_t_m"].astype(float)
    quat = take["ball_quat_xyzw"].astype(float)
    ok = take["ball_present"].astype(bool) & np.isfinite(pos[:, 0])
    zc = surface_z + R_BALL
    lo = np.searchsorted(t, t_strike + 0.03)
    hi = np.searchsorted(t, t_strike + max_horizon_s)
    idx = np.where(ok[lo:hi])[0] + lo
    if len(idx) < 20:
        return None
    # i_end = the FIRST approach to the surface after the strike (z within 30 mm of
    # the contact height). If the track never gets that low, the first bounce is not
    # observable here — reject rather than risk latching onto a later arc.
    near = idx[pos[idx, 2] < zc + 0.030]
    if not len(near):
        return None
    i_end = near[0]
    # terminal fit window: last tracked frames strictly before i_end
    win = idx[(idx < i_end) & (t[idx] >= t[i_end] - 0.160)]
    if len(win) < 15:
        return None
    _, _, w_rob = spin_from_quats(quat[win], rate, R_table=Rt)
    w = w_rob if np.isfinite(w_rob).all() else np.zeros(3)
    f = window_fit(t[win], pos[win], w, kd=kd, km=km)
    v_end = np.polyfit(t[win] - t[win][0], pos[win], 1)[0]
    if v_end[2] > -0.3:                  # must be descending toward the table
        return None
    pL, _, tL = integrate_to_table(f["p0"], f["v0"], w, kd, km, surface_z, t_max=0.6)
    if pL is None:
        return None
    lead = float(f["t0"] + tL - t[win][-1])
    if lead > max_extrap_s:              # only trust SHORT extrapolations
        return None
    return dict(p_land=pL, t_land=float(f["t0"] + tL), lead_s=lead)


# ------------- full-state trajectory validation (--full-state) -------------
# Horizons: h0 = through-paddle model; h1 = measured out-state; h1q = h1
# velocity but spin re-estimated from the FIRST 100 ms of flight quats
# (deploy-realistic: the strike w_out quat channel reads ~0.22x -> junk).

FS_BINS = ((0.000, 0.100, "0-100ms"), (0.100, 0.200, "100-200ms"),
           (0.200, 0.300, "200-300ms"), (0.300, 0.450, "300-450ms"),
           (0.450, np.inf, ">450ms"))
FS_CKPTS = (0.25, 0.50, 0.75)          # fractions of the measured arc duration
FS_HORIZONS = ("h0", "h1", "h1q")
FS_SPIN_FLOOR = 3.0    # rad/s; below it spin axis / relative magnitude are undefined
FS_SPIN_NOISE = 12.6   # rad/s (= 2 rev/s) venue quat-spin noise floor per event


def km_eff_sat(km, w, v, cl_slope=0.87, cl_max=0.55, sr_ref=0.05):
    """Saturating-Magnus effective coefficient:
    C_L(SR) = cl_slope*SR / (1 + cl_slope*SR/cl_max), SR = |w| R / |v|;
    k_m_eff(SR) = km * [C_L(SR)/SR] / [C_L(sr_ref)/sr_ref], normalized so it
    EQUALS km in the low-SR limit (SR'=0.05) — identical to the constant model
    inside the validated envelope, saturating beyond it."""
    sr = max(np.linalg.norm(w) * R_BALL / max(np.linalg.norm(v), 1e-9), 1e-6)
    cl = lambda s: cl_slope * s / (1.0 + cl_slope * s / cl_max)
    return km * (cl(sr) / sr) / (cl(sr_ref) / sr_ref)


def dense_flight(p0, v0, w, kd, km, surface_z, t_max=2.0, dt=1e-3, magnus="const"):
    """RK4 flight storing EVERY dt step until touchdown (center crosses
    surface_z + R descending). Returns dict(T, P, V, t_land) with the
    interpolated touchdown appended, or None if no touchdown within t_max."""
    g = np.array([0, 0, -G_NOM])
    zc = surface_z + R_BALL
    p, v = np.asarray(p0, float).copy(), np.asarray(v0, float).copy()
    w = np.asarray(w, float)
    T, P, V = [0.0], [p.copy()], [v.copy()]

    def acc(v_):
        k = km_eff_sat(km, w, v_) if magnus == "sat" else km
        return g - kd * np.linalg.norm(v_) * v_ + k * np.cross(w, v_)

    t = 0.0
    while t < t_max:
        a1 = acc(v); v2 = v + 0.5 * dt * a1; a2 = acc(v2)
        v3 = v + 0.5 * dt * a2; a3 = acc(v3)
        v4 = v + dt * a3; a4 = acc(v4)
        p_new = p + (dt / 6.0) * (v + 2 * v2 + 2 * v3 + v4)
        v_new = v + (dt / 6.0) * (a1 + 2 * a2 + 2 * a3 + a4)
        if p[2] > zc >= p_new[2] and v_new[2] < 0:
            f = (p[2] - zc) / max(p[2] - p_new[2], 1e-9)
            T.append(t + f * dt); P.append(p + f * (p_new - p)); V.append(v_new)
            return dict(T=np.array(T), P=np.array(P), V=np.array(V), t_land=T[-1])
        p, v, t = p_new, v_new, t + dt
        T.append(t); P.append(p.copy()); V.append(v.copy())
    return None


def fs_measured_arc(take, t_strike, t_land, rate):
    """Tracked post-strike frames, stage-1 contact exclusions applied
    (5 frames after the hit, 3 before the table contact)."""
    t = take["t"]
    pos = take["ball_pos_t_m"].astype(float)
    ok = take["ball_present"].astype(bool) & np.isfinite(pos[:, 0])
    m = ok & (t >= t_strike + 5.0 / rate) & (t <= t_land - 3.0 / rate)
    if m.sum() < 20:
        return None
    return dict(t=t[m], pos=pos[m], quat=take["ball_quat_xyzw"].astype(float)[m])


def fs_meas_vel(arc, tc, rate, Rt, kd, km, half=0.030):
    """Measured velocity at tc: 60 ms windowed SHOOTING fit centered there
    (never finite-difference), evaluated at tc."""
    m = (arc["t"] >= tc - half) & (arc["t"] <= tc + half)
    if m.sum() < 12:
        return None
    _, _, w_rob = spin_from_quats(arc["quat"][m], rate, R_table=Rt)
    w = w_rob if np.isfinite(w_rob).all() else np.zeros(3)
    f = window_fit(arc["t"][m], arc["pos"][m], w, kd=kd, km=km)
    return prop_state(f, tc)[1]


def fs_meas_spin(arc, tc, rate, Rt, half=0.050):
    """Measured spin at tc: robust spin_from_quats over a ~100 ms window."""
    m = (arc["t"] >= tc - half) & (arc["t"] <= tc + half)
    if m.sum() < 12:
        return None
    _, _, w_rob = spin_from_quats(arc["quat"][m], rate, R_table=Rt)
    return w_rob if np.isfinite(w_rob).all() else None


def fs_x0_crossing(t, pos, max_gap_s=0.010):
    """First net-plane (x=0) crossing: (t, y, z) linearly interpolated between
    the bracketing samples, or None. Skips occlusion gaps > max_gap_s."""
    x = pos[:, 0]
    for i in range(len(t) - 1):
        if t[i + 1] - t[i] > max_gap_s:
            continue
        if x[i] == 0.0 or x[i] * x[i + 1] < 0.0:
            f = x[i] / (x[i] - x[i + 1]) if x[i] != x[i + 1] else 0.0
            q = pos[i] + f * (pos[i + 1] - pos[i])
            return float(t[i] + f * (t[i + 1] - t[i])), float(q[1]), float(q[2])
    return None


def fs_envelope(v0, w0):
    """Validity-envelope flags for an initial state (speed 1-7, spin<=15 rev/s, SR<=0.5)."""
    sp = float(np.linalg.norm(v0))
    rev = float(np.linalg.norm(w0)) / (2 * np.pi)
    sr = float(np.linalg.norm(w0)) * R_BALL / max(sp, 1e-9)
    flags = []
    if not (1.0 <= sp <= 7.0): flags.append(f"speed {sp:.1f} m/s")
    if rev > 15.0: flags.append(f"spin {rev:.1f} rev/s")
    if sr > 0.5: flags.append(f"SR {sr:.2f}")
    return flags


def run_full_state(gt_records, takes_cache, args, kd, km, pd_, surface_z, src):
    pos_pool = {h: [] for h in FS_HORIZONS}   # rows of (t_rel, planar_m, d3_m)
    vel_pool = {h: {c: dict(dv=[], deg=[]) for c in FS_CKPTS} for h in FS_HORIZONS}
    spin_pool = {h: {c: dict(vec=[], axis=[], rel=[]) for c in FS_CKPTS} for h in FS_HORIZONS}
    net_pool = {h: dict(dz=[], dy=[], dt=[]) for h in FS_HORIZONS}
    rows = []
    n_extrap = {h: 0 for h in FS_HORIZONS}

    for rec in gt_records:
        s = rec["strike"]
        if not in_split(f'{s["take"]}:{s["t_c"]:.3f}', args.split):
            continue
        if s["fit_rms_mm"] > 8.0 or s["pad_fit_rms_mm"] > 15.0 or not s["spin_in_ok"]:
            continue
        if s["take"] not in takes_cache:
            takes_cache[s["take"]] = load_take(s["take"])
        take = takes_cache[s["take"]]
        Rt, rate = take["table_R"], float(take["rate"])
        arc = fs_measured_arc(take, s["t_c"], rec["t_land"], rate)
        if arc is None:
            continue
        t0a, t1a = float(arc["t"][0]), float(arc["t"][-1])

        vp, wp = paddle_outgoing(s, pd_)
        w_h1 = np.array(s["w_out"], float) if s["spin_out_ok"] else np.zeros(3)
        w_q = fs_meas_spin(arc, t0a + 0.050, rate, Rt)   # first 100 ms flight quats
        horizons = dict(h0=(vp, wp), h1=(np.array(s["v_out"], float), w_h1))
        if w_q is not None:
            horizons["h1q"] = (np.array(s["v_out"], float), w_q)

        row = dict(take=s["take"], t_strike=s["t_c"], t_land=rec["t_land"],
                   gt_source=rec["gt_source"], n_frames=int(len(arc["t"])),
                   arc_dur_ms=float((t1a - t0a) * 1e3))
        cross_meas = fs_x0_crossing(arc["t"], arc["pos"])
        for h, (v0, w0) in horizons.items():
            flags = fs_envelope(v0, w0)
            if flags:
                n_extrap[h] += 1
            D = dense_flight(np.array(s["ball_p"], float), v0, w0, kd, km,
                             surface_z, magnus=args.magnus)
            if D is None:
                row[h] = dict(extrap=flags, fail="no touchdown < 2 s")
                continue
            H = dict(extrap=flags, t_land_pred_ms=float(D["t_land"] * 1e3),
                     omega_pred_rads=[float(x) for x in w0])
            # position error at every tracked frame of the post-strike arc
            rel = arc["t"] - s["t_c"]
            sel = rel <= D["t_land"]
            pred = np.column_stack([np.interp(rel[sel], D["T"], D["P"][:, i])
                                    for i in range(3)])
            d = pred - arc["pos"][sel]
            planar = np.linalg.norm(d[:, :2], axis=1)
            d3 = np.linalg.norm(d, axis=1)
            rels = rel[sel]
            pos_pool[h].append(np.column_stack([rels, planar, d3]))
            H["pos_bins"] = {}
            for lo, hi, lbl in FS_BINS:
                m = (rels >= lo) & (rels < hi)
                if m.any():
                    H["pos_bins"][lbl] = dict(
                        n=int(m.sum()),
                        med_planar_mm=float(np.median(planar[m]) * 1e3),
                        med_3d_mm=float(np.median(d3[m]) * 1e3))
            # velocity + spin at checkpoints
            H["vel"], H["spin"] = {}, {}
            for c in FS_CKPTS:
                tc = t0a + c * (t1a - t0a)
                if tc - s["t_c"] > D["t_land"]:
                    continue
                v_meas = fs_meas_vel(arc, tc, rate, Rt, kd, km)
                if v_meas is not None:
                    v_pred = np.array([np.interp(tc - s["t_c"], D["T"], D["V"][:, i])
                                       for i in range(3)])
                    dv = float(np.linalg.norm(v_pred - v_meas))
                    cosang = np.dot(v_pred, v_meas) / max(
                        np.linalg.norm(v_pred) * np.linalg.norm(v_meas), 1e-9)
                    deg = float(np.degrees(np.arccos(np.clip(cosang, -1, 1))))
                    vel_pool[h][c]["dv"].append(dv)
                    vel_pool[h][c]["deg"].append(deg)
                    H["vel"][f"{int(c * 100)}%"] = dict(dv_mps=dv, dir_deg=deg)
                w_meas = fs_meas_spin(arc, tc, rate, Rt)
                if w_meas is not None:
                    vec = float(np.linalg.norm(w0 - w_meas))
                    n_meas, n_pred = np.linalg.norm(w_meas), np.linalg.norm(w0)
                    axis = (float(np.degrees(np.arccos(np.clip(
                        np.dot(w0, w_meas) / (n_pred * n_meas), -1, 1))))
                        if min(n_meas, n_pred) > FS_SPIN_FLOOR else None)
                    rel_mag = float((n_pred - n_meas) / n_meas) if n_meas > FS_SPIN_FLOOR else None
                    spin_pool[h][c]["vec"].append(vec)
                    if axis is not None:
                        spin_pool[h][c]["axis"].append(axis)
                    if rel_mag is not None:
                        spin_pool[h][c]["rel"].append(rel_mag)
                    H["spin"][f"{int(c * 100)}%"] = dict(
                        vec_err_rads=vec, axis_deg=axis, mag_rel=rel_mag)
            # net-plane x=0 crossing
            if cross_meas is not None:
                cp = fs_x0_crossing(D["T"], D["P"], max_gap_s=1.0)
                if cp is not None:
                    dtm = float((s["t_c"] + cp[0]) - cross_meas[0]) * 1e3
                    dy = float(cp[1] - cross_meas[1]) * 1e3
                    dz = float(cp[2] - cross_meas[2]) * 1e3
                    net_pool[h]["dz"].append(dz)
                    net_pool[h]["dy"].append(dy)
                    net_pool[h]["dt"].append(dtm)
                    H["net"] = dict(dz_mm=dz, dy_mm=dy, dt_ms=dtm)
            row[h] = H
        rows.append(row)

    def q(v):
        v = np.asarray([x for x in v if x is not None and np.isfinite(x)], float)
        if not len(v):
            return dict(n=0)
        return dict(n=int(len(v)), median=float(np.median(v)),
                    p90=float(np.percentile(v, 90)))

    pos_agg = {}
    for h in FS_HORIZONS:
        A = np.vstack(pos_pool[h]) if pos_pool[h] else np.zeros((0, 3))
        pos_agg[h] = {lbl: dict(planar_mm=q(A[(A[:, 0] >= lo) & (A[:, 0] < hi), 1] * 1e3),
                                err3d_mm=q(A[(A[:, 0] >= lo) & (A[:, 0] < hi), 2] * 1e3))
                      for lo, hi, lbl in FS_BINS}
    ck = lambda c: f"{int(c * 100)}%"
    vel_agg = {h: {ck(c): dict(dv_mps=q(vel_pool[h][c]["dv"]),
                               dir_deg=q(vel_pool[h][c]["deg"])) for c in FS_CKPTS}
               for h in FS_HORIZONS}
    spin_agg = {h: {ck(c): dict(vec_err_rads=q(spin_pool[h][c]["vec"]),
                                axis_deg=q(spin_pool[h][c]["axis"]),
                                mag_rel=q(spin_pool[h][c]["rel"])) for c in FS_CKPTS}
                for h in FS_HORIZONS}
    net_agg = {h: dict(dz_mm=q(net_pool[h]["dz"]), dy_mm=q(net_pool[h]["dy"]),
                       dt_ms=q(net_pool[h]["dt"])) for h in FS_HORIZONS}

    rep = dict(
        mode="full_state", params_source=src, split=args.split, magnus=args.magnus,
        n_strikes=len(rows),
        n_by_gt_source={k: sum(1 for r in rows if r["gt_source"] == k)
                        for k in ("observed_bounce", "terminal_window")},
        n_extrapolation_flagged=n_extrap,
        notes=dict(
            horizons="h0=through-paddle model; h1=measured out-state; h1q=h1 velocity "
                     "with spin re-estimated from the first 100 ms of flight quats "
                     "(deploy-realistic: strike w_out quat channel reads ~0.22x = junk)",
            spin_noise_floor_rads=FS_SPIN_NOISE,
            spin_floor_for_ratios_rads=FS_SPIN_FLOOR,
            magnus_sat_normalization="k_m_eff(SR)=k_m*[C_L(SR)/SR]/[C_L(0.05)/0.05], "
                                     "C_L(SR)=0.87*SR/(1+0.87*SR/0.55); equals k_m "
                                     "in the low-SR limit"),
        position_bins=pos_agg, velocity_ckpts=vel_agg, spin_ckpts=spin_agg,
        net_crossing=net_agg, rows=rows)
    out = os.path.join(os.path.dirname(args.out), "predict_fullstate.json")
    os.makedirs(os.path.dirname(out) or ".", exist_ok=True)
    json.dump(rep, open(out, "w"), indent=1)

    # ---- plot ----
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    HC = dict(h0=("tab:red", "H0 through-paddle"), h1=("tab:orange", "H1 measured-out"),
              h1q=("tab:blue", "H1q spin from flight quats"))
    fig, ax = plt.subplots(2, 2, figsize=(13, 9))
    centers = [(lo + min(hi, 0.6)) / 2 * 1e3 for lo, hi, _ in FS_BINS]
    for h in FS_HORIZONS:
        med = [pos_agg[h][lbl]["planar_mm"].get("median", np.nan) for _, _, lbl in FS_BINS]
        p90 = [pos_agg[h][lbl]["planar_mm"].get("p90", np.nan) for _, _, lbl in FS_BINS]
        c, lb = HC[h]
        ax[0, 0].plot(centers, med, "o-", color=c, label=f"{lb} (median)")
        ax[0, 0].plot(centers, p90, "o--", color=c, alpha=0.5, label=f"{lb} (p90)")
    ax[0, 0].set_xlabel("time since strike (ms)")
    ax[0, 0].set_ylabel("planar position error (mm)")
    ax[0, 0].set_title("position error vs horizon")
    ax[0, 0].legend(fontsize=7); ax[0, 0].grid(alpha=0.3)

    def box(axx, pool, key, ylabel, title):
        data, poss, colors = [], [], []
        for j, c in enumerate(FS_CKPTS):
            for k, h in enumerate(FS_HORIZONS):
                v = pool[h][c][key]
                if v:
                    data.append(v); poss.append(j + (k - 1) * 0.22)
                    colors.append(HC[h][0])
        if data:
            bp = axx.boxplot(data, positions=poss, widths=0.18, patch_artist=True,
                             showfliers=False, medianprops=dict(color="k"))
            for b, cc in zip(bp["boxes"], colors):
                b.set_facecolor(cc); b.set_alpha(0.6)
        axx.set_xticks(range(len(FS_CKPTS)))
        axx.set_xticklabels([f"{int(c * 100)}% of arc" for c in FS_CKPTS])
        axx.set_ylabel(ylabel); axx.set_title(title); axx.grid(alpha=0.3, axis="y")

    box(ax[0, 1], vel_pool, "dv", "|dv| (m/s)", "velocity error at checkpoints")
    box(ax[1, 0], vel_pool, "deg", "direction error (deg)", "velocity direction error")
    box(ax[1, 1], spin_pool, "vec", "|dw| (rad/s)", "spin vector error at checkpoints")
    ax[1, 1].axhline(FS_SPIN_NOISE, color="k", ls=":", lw=1)
    ax[1, 1].text(0.02, FS_SPIN_NOISE, " 2 rev/s quat-spin noise floor",
                  fontsize=7, va="bottom")
    from matplotlib.patches import Patch
    ax[0, 1].legend(handles=[Patch(facecolor=HC[h][0], alpha=0.6, label=HC[h][1])
                             for h in FS_HORIZONS], fontsize=7)
    fig.tight_layout()
    png = out.replace(".json", ".png")
    fig.savefig(png, dpi=110)
    print(json.dumps({k: v for k, v in rep.items() if k != "rows"}, indent=1))
    print(f"-> {out}\n-> {png}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--yaml", default=None)
    ap.add_argument("--paddle-e", default="exp", choices=["const", "exp", "lin"])
    ap.add_argument("--fits", default=os.path.join(ANA, "fits", "stage2_fits_all.json"))
    ap.add_argument("--split", default="all", choices=["all", "train", "test"])
    ap.add_argument("--out", default=os.path.join(ANA, "fits", "predict_check.json"))
    ap.add_argument("--full-state", action="store_true",
                    help="full-state trajectory validation (position/velocity/spin "
                         "along the arc + net crossing) -> predict_fullstate.json")
    ap.add_argument("--magnus", default="const", choices=["const", "sat"],
                    help="--full-state only: Magnus form (sat = saturating C_L, "
                         "normalized to k_m at low SR)")
    args = ap.parse_args()

    if args.yaml:
        Y = load_yaml_constants(args.yaml)
        kd, km = Y["flight.k_d"], Y["flight.k_m"]
        pd_ = dict(mode=args.paddle_e, e_eff=Y["contact.paddle.e_eff"],
                   a_t=Y["contact.paddle.a_t"], b_t=Y["contact.paddle.b_t"],
                   mu=Y["contact.paddle.mu_safety"],
                   g1=Y.get("contact.paddle.e_exp_g1"), g2=Y.get("contact.paddle.e_exp_g2"),
                   a=Y.get("contact.paddle.e_lin_a"), b=Y.get("contact.paddle.e_lin_b"))
        src = f"yaml (paddle e={args.paddle_e})"
    else:
        F = json.load(open(args.fits))
        kd, km = F["kd"]["kd"], F["km"]["km"]
        p = F["paddle"]
        pd_ = dict(mode="const", e_eff=p["e_eff"], a_t=p["a_t"], b_t=p["b_t"], mu=p["mu"])
        src = f"fits:{os.path.basename(args.fits)}"

    meta = json.load(open(os.path.join(ANA, "segments", "meta.json")))
    surface_z = meta["surface_z"]
    bounces = json.load(open(os.path.join(ANA, "segments", "bounces.json")))
    strikes = json.load(open(os.path.join(ANA, "segments", "strikes.json")))

    pairs = pair_strike_bounce(strikes, bounces, kd, km, surface_z)
    paired_keys = {(pr["strike"]["take"], pr["strike"]["t_c"]) for pr in pairs}
    takes_cache = {}

    # ground-truth records: observed bounce p_c where paired, else terminal-window
    # extrapolation of the post-strike arc (recovers the ~2/3 of first bounces the
    # bounce extractor misses; lead <= 200 ms so it is H2-grade, mm-cm accurate)
    gt_records = [dict(strike=pr["strike"],
                       gt_xy=np.array(pr["bounce"]["p_c"])[:2],
                       t_land=pr["bounce"]["t_c"], gt_source="observed_bounce")
                  for pr in pairs]
    events_by_take = {}
    for s in strikes:
        events_by_take.setdefault(s["take"], []).append(s["t_c"])
    for b in bounces:
        events_by_take.setdefault(b["take"], []).append(b["t_c"])
    for s in strikes:
        if (s["take"], s["t_c"]) in paired_keys:
            continue
        if s["take"] not in takes_cache:
            takes_cache[s["take"]] = load_take(s["take"])
        take = takes_cache[s["take"]]
        gt = terminal_window_gt(take, take["table_R"], s["t_c"], float(take["rate"]),
                                kd, km, surface_z)
        if gt is None:
            continue
        # identity gates: the GT must belong to THIS strike's ballistic segment.
        # (a) no other recorded contact between strike and touchdown
        if any(s["t_c"] + 0.01 < te < gt["t_land"] - 0.01
               for te in events_by_take.get(s["take"], [])):
            continue
        # (b) loose continuity with the measured out-state (same gates family as
        #     pair_strike_bounce; identity check, not an accuracy tune)
        w_meas = np.array(s["w_out"], float) if s["spin_out_ok"] else np.zeros(3)
        pL1, _, tL1 = integrate_to_table(s["ball_p"], s["v_out"], w_meas, kd, km, surface_z)
        if pL1 is None:
            continue
        if (np.linalg.norm(pL1[:2] - gt["p_land"][:2]) > 0.40
                or abs((s["t_c"] + tL1) - gt["t_land"]) > 0.12):
            continue
        gt_records.append(dict(strike=s, gt_xy=gt["p_land"][:2],
                               t_land=gt["t_land"], gt_source="terminal_window"))

    if args.full_state:
        run_full_state(gt_records, takes_cache, args, kd, km, pd_, surface_z, src)
        return

    rows = []
    for rec in gt_records:
        s = rec["strike"]
        if not in_split(f'{s["take"]}:{s["t_c"]:.3f}', args.split):
            continue
        if s["fit_rms_mm"] > 8.0 or s["pad_fit_rms_mm"] > 15.0 or not s["spin_in_ok"]:
            continue
        gt_xy = rec["gt_xy"]
        t_land = rec["t_land"]
        row = dict(take=s["take"], t_strike=s["t_c"], t_land=t_land,
                   u_n=s["u_n"], gt=[float(g) for g in gt_xy],
                   gt_source=rec["gt_source"])

        # H0: through-paddle
        vp, wp = paddle_outgoing(s, pd_)
        pL, _, tL = integrate_to_table(s["ball_p"], vp, wp, kd, km, surface_z)
        if pL is not None:
            row["h0_err_m"] = float(np.linalg.norm(pL[:2] - gt_xy))
            row["h0_dt_ms"] = float(((s["t_c"] + tL) - t_land) * 1e3)

        # H1: measured out-state
        w_meas = np.array(s["w_out"], float) if s["spin_out_ok"] else np.zeros(3)
        pL1, _, tL1 = integrate_to_table(s["ball_p"], s["v_out"], w_meas, kd, km, surface_z)
        if pL1 is not None:
            row["h1_err_m"] = float(np.linalg.norm(pL1[:2] - gt_xy))
            row["h1_dt_ms"] = float(((s["t_c"] + tL1) - t_land) * 1e3)

        # H2: near-landing refinement
        if s["take"] not in takes_cache:
            takes_cache[s["take"]] = load_take(s["take"])
        take = takes_cache[s["take"]]
        st = h2_state(take, take["table_R"], t_land, float(take["rate"]), kd, km)
        if st is not None:
            f = st["fit"]
            pL2, _, tL2 = integrate_to_table(f["p0"], f["v0"], st["w"], kd, km, surface_z)
            if pL2 is not None:
                row["h2_err_m"] = float(np.linalg.norm(pL2[:2] - gt_xy))
                row["h2_dt_ms"] = float(((f["t0"] + tL2) - t_land) * 1e3)
                row["h2_lead_ms"] = float(st["lead_s"] * 1e3)
        rows.append(row)

    def stats(key):
        v = np.array([r[key] for r in rows if key in r])
        if not len(v):
            return dict(n=0)
        return dict(n=int(len(v)), median=float(np.median(v)),
                    p90=float(np.percentile(v, 90)), mean=float(v.mean()))

    rep = dict(params_source=src, split=args.split,
               n_pairs=len(rows),
               n_by_gt_source={k: sum(1 for r in rows if r["gt_source"] == k)
                               for k in ("observed_bounce", "terminal_window")},
               h0_through_paddle=stats("h0_err_m"),
               h1_measured_out=stats("h1_err_m"),
               h2_near_landing=stats("h2_err_m"),
               h0_timing_ms=stats("h0_dt_ms"), h1_timing_ms=stats("h1_dt_ms"),
               h2_timing_ms=stats("h2_dt_ms"),
               per_take={tk: dict(
                   n=len([r for r in rows if r["take"] == tk]),
                   h0=stats_take(rows, tk, "h0_err_m"),
                   h1=stats_take(rows, tk, "h1_err_m"),
                   h2=stats_take(rows, tk, "h2_err_m"))
                   for tk in sorted({r["take"] for r in rows})},
               rows=rows)
    json.dump(rep, open(args.out, "w"), indent=1)

    # ---- plot ----
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(1, 3, figsize=(16, 5))
    for key, lbl, c in (("h0_err_m", "H0 at-contact (through paddle)", "tab:red"),
                        ("h1_err_m", "H1 at-contact (measured out)", "tab:orange"),
                        ("h2_err_m", "H2 near-landing (~100ms lead)", "tab:green")):
        v = np.sort([r[key] for r in rows if key in r])
        if len(v):
            ax[0].plot(v, np.arange(1, len(v) + 1) / len(v), label=f"{lbl} (med {np.median(v)*100:.0f} cm)", color=c)
    ax[0].axvline(0.10, color="k", ls=":", lw=1, label="0.10 m target")
    ax[0].set_xlabel("planar landing error (m)"); ax[0].set_ylabel("CDF")
    ax[0].set_xlim(0, 0.8); ax[0].legend(fontsize=8); ax[0].set_title("prediction error by horizon")
    for r in rows:
        if "h0_err_m" in r:
            ax[1].plot([r["gt"][0]], [r["gt"][1]], "k.", ms=4)
    ax[1].add_patch(plt.Rectangle((-1.37, -0.7625), 2.74, 1.525, fill=False, color="b", lw=1))
    ax[1].set_title("observed landings (table frame)"); ax[1].set_aspect("equal")
    ax[1].set_xlabel("X length (m)"); ax[1].set_ylabel("Y width (m)")
    un = [r["u_n"] for r in rows if "h0_err_m" in r]
    e0 = [r["h0_err_m"] for r in rows if "h0_err_m" in r]
    ax[2].scatter(un, e0, s=14, c="tab:red")
    ax[2].set_xlabel("|u_n| at racket (m/s)"); ax[2].set_ylabel("H0 landing error (m)")
    ax[2].set_title("through-paddle error vs contact speed")
    fig.tight_layout()
    png = args.out.replace(".json", ".png")
    fig.savefig(png, dpi=110)
    print(json.dumps({k: v for k, v in rep.items() if k not in ("rows", "per_take")}, indent=1))
    print("per-take:", json.dumps(rep["per_take"], indent=1))
    print(f"-> {args.out}\n-> {png}")


def stats_take(rows, tk, key):
    v = [r[key] for r in rows if r["take"] == tk and key in r]
    return dict(n=len(v), median=float(np.median(v)) if v else None)


if __name__ == "__main__":
    main()
