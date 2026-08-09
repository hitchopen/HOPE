"""
F8 - Does racket angular velocity during the ~1-2 ms dwell leave a signature the
instantaneous contact model misses?

Per gated strike, predict v_out with the fitted paddle contact model (exactly the
stage-2 convention: cp = ball_p - R*pad_n with pad_n as stored -- stage 1 already
oriented it against approach -- v_r = pad_v + pad_w x (cp - pad_p)).  Target:
outgoing-DIRECTION residual angle (deg) between predicted and measured v_out,
regressed against |pad_w| (rad/s).

KILL if significantly correlated AND implied effect > 2 deg across the observed
|pad_w| range; PASS if uncorrelated / < 2 deg.

Secondary (mechanistic) check: rotate the paddle normal by pad_w * tau (tau = 1.5 ms
dwell) and see whether the measured residual VECTOR projects onto that predicted
dwell-signature direction (signed test, far more specific than the unsigned angle).
Confound check: residual angle vs |pad_w| * dt_extrap (paddle-state extrapolation
error grows the same way a dwell signature would).
"""
import os, sys, json
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from scipy import stats

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import paths
from contact_model import predict_contact, R_BALL

ANA = paths.ANALYSIS
OUTDIR = os.path.join(ANA, "falsification")
os.makedirs(OUTDIR, exist_ok=True)
RNG = np.random.default_rng(8)
N_BOOT = 5000
TAU_DWELL = 1.5e-3  # s, nominal dwell


def rot_matrix(axis, ang):
    """Rodrigues; axis (3,) unit, ang scalar."""
    K = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
    return np.eye(3) + np.sin(ang) * K + (1 - np.cos(ang)) * (K @ K)


def predict_rows(rows, pd, n_rot_tau=None):
    """Model-predicted v_out for each strike. If n_rot_tau is set, rotate pad_n
    (and recompute cp/v_r) by pad_w * tau about the pad_w axis first."""
    V_pred = np.zeros((len(rows), 3))
    for i, s in enumerate(rows):
        n = np.asarray(s["pad_n"], float)
        pad_p = np.asarray(s["pad_p"], float)
        pad_v = np.asarray(s["pad_v"], float)
        pad_w = np.asarray(s["pad_w"], float)
        ball_p = np.asarray(s["ball_p"], float)
        if n_rot_tau is not None:
            wmag = np.linalg.norm(pad_w)
            if wmag > 1e-9:
                n = rot_matrix(pad_w / wmag, wmag * n_rot_tau) @ n
        cp = ball_p - R_BALL * n
        v_r = pad_v + np.cross(pad_w, cp - pad_p)
        out = predict_contact(np.asarray(s["v_in"], float), v_r, n,
                              np.asarray(s["w_in"], float),
                              pd["e_eff"], pd["a_t"], pd["b_t"], pd["mu"])
        V_pred[i] = out["v_plus"][0]
    return V_pred


def angle_deg(A, B):
    ca = (A * B).sum(1) / (np.linalg.norm(A, axis=1) * np.linalg.norm(B, axis=1) + 1e-12)
    return np.degrees(np.arccos(np.clip(ca, -1, 1)))


def spearman_partial(y, x, covs):
    """Spearman of y vs x after rank-regressing both on covariates."""
    ry, rx = stats.rankdata(y), stats.rankdata(x)
    C = np.column_stack([np.ones_like(ry)] + [stats.rankdata(c) for c in covs])
    ry_res = ry - C @ np.linalg.lstsq(C, ry, rcond=None)[0]
    rx_res = rx - C @ np.linalg.lstsq(C, rx, rcond=None)[0]
    return stats.pearsonr(rx_res, ry_res)


def boot_ci(fn, n, rng, n_boot=N_BOOT):
    vals = np.empty(n_boot)
    for b in range(n_boot):
        idx = rng.integers(0, n, n)
        vals[b] = fn(idx)
    return np.percentile(vals, [2.5, 97.5]), vals


def main():
    strikes = json.load(open(os.path.join(ANA, "segments", "strikes.json")))
    fits = json.load(open(os.path.join(ANA, "fits", "stage2_fits_all.json")))
    pd = fits["paddle"]

    rows = [s for s in strikes if s["spin_in_ok"]
            and s["fit_rms_mm"] <= 8.0 and s["pad_fit_rms_mm"] <= 15.0]
    n = len(rows)

    V_meas = np.array([s["v_out"] for s in rows])
    V_pred = predict_rows(rows, pd)
    theta = angle_deg(V_pred, V_meas)                      # deg, target
    x = np.array([np.linalg.norm(s["pad_w"]) for s in rows])   # rad/s
    u_n = np.array([s["u_n"] for s in rows])
    u_t = np.array([s["u_t"] for s in rows])
    dt_ex = np.array([s["pad_dt_extrap_ms"] for s in rows])
    speed_out = np.linalg.norm(V_meas, axis=1)

    # ---- primary: Spearman + slope with bootstrap ----
    rho, p_rho = stats.spearmanr(x, theta)
    slope_ols, icpt_ols = np.polyfit(x, theta, 1)
    ts = stats.theilslopes(theta, x)
    x_range_full = x.max() - x.min()
    x_range_robust = np.percentile(x, 97.5) - np.percentile(x, 2.5)

    def slope_fn(idx):
        if len(np.unique(x[idx])) < 3:
            return 0.0
        return np.polyfit(x[idx], theta[idx], 1)[0]
    (slope_lo, slope_hi), slope_boot = boot_ci(slope_fn, n, RNG)

    def rho_fn(idx):
        return stats.spearmanr(x[idx], theta[idx])[0]
    (rho_lo, rho_hi), _ = boot_ci(rho_fn, n, RNG)

    eff_full = slope_ols * x_range_full          # deg across full observed range
    eff_full_ci = (slope_lo * x_range_full, slope_hi * x_range_full)
    eff_ts_full = ts[0] * x_range_full
    eff_ts_ci = (ts[2] * x_range_full, ts[3] * x_range_full)

    # ---- confounder checks ----
    r_part_un, p_part_un = spearman_partial(theta, x, [u_n])
    r_part_all, p_part_all = spearman_partial(theta, x, [u_n, u_t, speed_out])
    rho_prox, p_prox = stats.spearmanr(x * dt_ex, theta)  # extrapolation-error proxy

    # e(u_n) linear sensitivity (e = a - b*u_n, stage-2 convention)
    pd_lin = dict(pd)
    e_lin = np.clip(pd["e_lin_a"] - pd["e_lin_b"] * u_n, 0.05, 1.5)
    V_pred_lin = np.zeros_like(V_pred)
    for i, s in enumerate(rows):
        nvec = np.asarray(s["pad_n"], float)
        cp = np.asarray(s["ball_p"], float) - R_BALL * nvec
        v_r = np.asarray(s["pad_v"], float) + np.cross(np.asarray(s["pad_w"], float),
                                                       cp - np.asarray(s["pad_p"], float))
        out = predict_contact(np.asarray(s["v_in"], float), v_r, nvec,
                              np.asarray(s["w_in"], float),
                              float(e_lin[i]), pd["a_t"], pd["b_t"], pd["mu"])
        V_pred_lin[i] = out["v_plus"][0]
    theta_lin = angle_deg(V_pred_lin, V_meas)
    rho_lin, p_rho_lin = stats.spearmanr(x, theta_lin)
    slope_lin = np.polyfit(x, theta_lin, 1)[0]

    # confound path documentation
    rho_x_un = stats.spearmanr(x, u_n)[0]
    rho_th_un = stats.spearmanr(theta, u_n)[0]

    # ---- secondary mechanistic: signed projection on dwell-rotation signature ----
    V_pred_rot = predict_rows(rows, pd, n_rot_tau=TAU_DWELL)
    sig = V_pred_rot - V_pred                    # predicted dwell signature (m/s)
    dv = V_meas - V_pred                          # measured residual vector
    sig_mag = np.linalg.norm(sig, axis=1)
    ok = sig_mag > 1e-6
    # least-squares scale: dv ~ k * sig  (k ~ 0.5-1 expected if dwell real)
    def k_fn(idx):
        s_, d_ = sig[idx][ok[idx]], dv[idx][ok[idx]]
        den = (s_ * s_).sum()
        return (d_ * s_).sum() / den if den > 0 else 0.0
    k_hat = k_fn(np.arange(n))
    (k_lo, k_hi), _ = boot_ci(k_fn, n, RNG)
    # angular size of the full dwell signature at max |pad_w| (physical ceiling)
    sig_angle = angle_deg(V_pred_rot, V_pred)     # deg per strike for tau=1.5 ms
    sig_angle_max = float(np.max(sig_angle))
    sig_angle_med = float(np.median(sig_angle))

    # robustness of k: by take, and excluding the top-5 |pad_w| strikes
    k_by_take = {}
    for tk in sorted({s["take"] for s in rows}):
        idx = np.array([i for i, s in enumerate(rows) if s["take"] == tk])
        if len(idx) >= 8:
            k_by_take[tk] = round(float(k_fn(idx)), 1)
    keep = np.argsort(x)[:-5]
    k_trim = float(k_fn(keep))

    # ---- verdict ----
    # Raw unsigned correlation exists but must survive swing-intensity confound
    # control to count; and the dwell-specific signature has a physical ceiling
    # (sig_angle_max) that the kill threshold compares against.  The signed k
    # separates dwell (k ~ 0.5-1) from extrapolation error (k ~ dt_extrap/tau ~ 10+).
    confound_survives = (p_part_all < 0.05 and r_part_all > 0)
    dwell_scale = (k_lo <= 1.5)          # CI reaches down to dwell-sized rotation
    big_effect = abs(eff_full) > 2.0 and np.sign(eff_full_ci[0]) == np.sign(eff_full_ci[1])
    if (p_rho < 0.05) and big_effect and confound_survives and dwell_scale:
        verdict = "KILL"
    elif (not confound_survives) and sig_angle_max < 2.0:
        verdict = "PASS"
    else:
        verdict = "INCONCLUSIVE"

    # ---- plot ----
    fig, axes = plt.subplots(1, 3, figsize=(15, 4.6))
    ax = axes[0]
    ax.scatter(x, theta, s=18, c="tab:blue", alpha=0.6, edgecolors="none")
    xs = np.linspace(x.min(), x.max(), 50)
    ax.plot(xs, icpt_ols + slope_ols * xs, "r-", lw=2,
            label=f"OLS {slope_ols:+.2f} deg/(rad/s)")
    ax.plot(xs, ts[1] + ts[0] * xs, "g--", lw=2,
            label=f"Theil-Sen {ts[0]:+.2f}")
    bins = np.percentile(x, [0, 20, 40, 60, 80, 100])
    bc = 0.5 * (bins[:-1] + bins[1:])
    bmed = [np.median(theta[(x >= bins[i]) & (x <= bins[i + 1])]) for i in range(5)]
    ax.plot(bc, bmed, "ks-", ms=7, lw=1.5, label="quintile medians")
    ax.set_xlabel("|pad_w| (rad/s)"); ax.set_ylabel("direction residual (deg)")
    ax.set_title(f"residual angle vs |pad_w|  (n={n})\n"
                 f"Spearman rho={rho:.2f} p={p_rho:.3f}; range effect "
                 f"{eff_full:+.1f} deg [{eff_full_ci[0]:+.1f},{eff_full_ci[1]:+.1f}]",
                 fontsize=10)
    ax.legend(fontsize=8); ax.grid(alpha=0.3)

    ax = axes[1]
    ax.scatter(x * dt_ex, theta, s=18, c="tab:orange", alpha=0.6, edgecolors="none")
    ax.set_xlabel("|pad_w| * dt_extrap (rad/s * ms)")
    ax.set_ylabel("direction residual (deg)")
    ax.set_title(f"confound control\nextrap proxy rho={rho_prox:.2f} p={p_prox:.3f}\n"
                 f"partial rho(|pad_w| | u_n,u_t,|v_out|)={r_part_all:.2f} p={p_part_all:.2f}",
                 fontsize=10)
    ax.grid(alpha=0.3)

    ax = axes[2]
    ax.scatter(sig_mag, (dv * sig).sum(1) / (sig_mag + 1e-12), s=18,
               c="tab:green", alpha=0.6, edgecolors="none")
    ss = np.linspace(0, sig_mag.max(), 20)
    ax.plot(ss, k_hat * ss, "r-", lw=2, label=f"k={k_hat:.2f} [{k_lo:.2f},{k_hi:.2f}]")
    ax.plot(ss, 1.0 * ss, "k:", lw=1.5, label="k=1 (full dwell rotation)")
    ax.axhline(0, color="gray", lw=0.8)
    ax.set_xlabel(f"|dwell signature| (m/s), tau={TAU_DWELL*1e3:.1f} ms")
    ax.set_ylabel("residual projection onto signature (m/s)")
    ax.set_title(f"mechanistic: dv . sig_hat vs |sig|\n"
                 f"dwell-ceiling angle med {sig_angle_med:.2f} / max {sig_angle_max:.2f} deg\n"
                 f"k={k_hat:.0f} = {k_hat*TAU_DWELL*1e3:.0f} ms rotation (dt_extrap 15-18 ms)",
                 fontsize=10)
    ax.legend(fontsize=8); ax.grid(alpha=0.3)

    fig.suptitle(f"F8 pad_w dwell signature -- verdict: {verdict}", fontsize=13, y=1.04)
    fig.tight_layout()
    fig.savefig(os.path.join(OUTDIR, "F8.png"), dpi=140, bbox_inches="tight")

    out = dict(
        test="F8 racket angular velocity dwell signature",
        verdict=verdict,
        n=n,
        gates=dict(fit_rms_mm=8.0, pad_fit_rms_mm=15.0, spin_in_ok=True),
        pad_w_range_rad_s=[float(x.min()), float(x.max())],
        pad_w_range_robust=float(x_range_robust),
        residual_angle_deg=dict(median=float(np.median(theta)),
                                p90=float(np.percentile(theta, 90))),
        spearman=dict(rho=float(rho), p=float(p_rho),
                      rho_ci=[float(rho_lo), float(rho_hi)]),
        ols_slope_deg_per_rad_s=dict(est=float(slope_ols),
                                     ci=[float(slope_lo), float(slope_hi)]),
        theil_sen_slope=dict(est=float(ts[0]), ci=[float(ts[2]), float(ts[3])]),
        implied_effect_deg_full_range=dict(est=float(eff_full),
                                           ci=[float(eff_full_ci[0]), float(eff_full_ci[1])]),
        implied_effect_deg_theil_sen=dict(est=float(eff_ts_full),
                                          ci=[float(eff_ts_ci[0]), float(eff_ts_ci[1])]),
        partial_spearman_given_un=dict(rho=float(r_part_un), p=float(p_part_un)),
        partial_spearman_given_un_ut_vout=dict(rho=float(r_part_all), p=float(p_part_all)),
        confound_path=dict(rho_padw_vs_un=float(rho_x_un),
                           rho_theta_vs_un=float(rho_th_un)),
        extrap_proxy_spearman=dict(rho=float(rho_prox), p=float(p_prox)),
        e_lin_sensitivity=dict(rho=float(rho_lin), p=float(p_rho_lin),
                               slope=float(slope_lin)),
        dwell_signature=dict(tau_ms=TAU_DWELL * 1e3,
                             k_scale=float(k_hat), k_ci=[float(k_lo), float(k_hi)],
                             k_excl_top5_padw=float(k_trim), k_by_take=k_by_take,
                             equiv_extra_rotation_ms=float(k_hat * TAU_DWELL * 1e3),
                             angle_med_deg=sig_angle_med, angle_max_deg=sig_angle_max),
        pad_dt_extrap_ms=[float(dt_ex.min()), float(dt_ex.max())],
    )
    with open(os.path.join(OUTDIR, "F8_verdict.json"), "w") as f:
        json.dump(out, f, indent=1)
    print(json.dumps(out, indent=1))


if __name__ == "__main__":
    main()
