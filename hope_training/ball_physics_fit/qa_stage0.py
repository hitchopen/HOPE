"""
Stage 0 QA for the venue dataset (doc section 4, Stage 0).

Checks:
  0. effective sampling: gap structure already in manifests; here per-run dt uniformity
  1. units/frame: table extents (done in extraction), surface-z calibration from bounce minima
  2. gravity: joint shooting fit (per-arc p0,v0 + global g_vec + global k_d) on low-spin arcs;
     require |g| = 9.81 +- 0.05 and tilt < 0.2 deg (vs table normal and vs world z)
  3. spin channel sanity: per-arc spin from quats; aliasing margin; in-flight constancy
  4. ball-center wobble: residual oscillation at spin frequency; global delta fit
Writes qa_stage0.json + prints verdict.
"""
import os, sys, json
import numpy as np
import paths
from scipy.spatial.transform import Rotation as Rot

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from ballcore import (TAKES, load_take, extract_arcs, arc_parabola, arc_is_ballistic,
                      spin_from_quats, fit_arcs_global, R_BALL, G_NOM)

OUT = paths.ANALYSIS


def main():
    report = {}
    takes = {n: load_take(n) for n in TAKES}

    # ---- surface-z calibration from bounce minima (all takes) ----
    surf = {}
    all_arcs = {}
    for n, tk in takes.items():
        arcs = extract_arcs(tk)
        all_arcs[n] = arcs
        zmins = []
        pos = tk["ball_pos_t_m"]
        # bounce minima: use contact indices labeled 'table' -- approximate via arc ends
        for a in arcs:
            if a["pre_contact"] == "table":
                i = a["contact_pre_idx"]
                zwin = pos[max(i - 4, 0):i + 5, 2]
                zwin = zwin[np.isfinite(zwin)]
                if len(zwin):
                    zmins.append(float(np.min(zwin)))
        surf[n] = dict(n_bounce=len(zmins),
                       zmin_med=float(np.median(zmins)) if zmins else None,
                       surface_z=float(np.median(zmins)) - R_BALL if zmins else None)
    report["surface_calibration"] = surf

    # ---- arc inventory + spin per arc ----
    inv = {}
    for n, arcs in all_arcs.items():
        rate = float(takes[n]["rate"])
        rows = []
        for a in arcs:
            par = arc_parabola(a)
            w, ang, w_rob = spin_from_quats(a["quat"], rate)
            spd = float(np.linalg.norm(par["v0"]))
            rows.append(dict(dur=float(a["t"][-1] - a["t"][0]), g=par["g"], rms=par["rms"],
                             speed=spd, spin_revs=float(np.linalg.norm(w_rob) / (2 * np.pi))
                             if np.isfinite(w_rob).all() else None,
                             ang_max=float(np.nanmax(ang)) if np.isfinite(ang).any() else None,
                             ballistic=bool(arc_is_ballistic(a, par))))
            a["par"], a["w_rob"] = par, w_rob
        inv[n] = dict(n_arcs=len(rows),
                      n_ballistic=int(sum(r["ballistic"] for r in rows)),
                      dur_total=round(sum(r["dur"] for r in rows), 1))
    report["arc_inventory"] = inv

    # ---- gravity fit on low-spin ballistic arcs ----
    # candidates: no-spin juggling + bounce trials + normal, spin < 3 rev/s, dur >= 0.2 s
    cands = []
    for n in ("dianqiu_nospin", "tantiao", "gaoqiu", "zhengchang"):
        for a in all_arcs[n]:
            if not a.get("par") or not arc_is_ballistic(a, a["par"]):
                continue
            dur = a["t"][-1] - a["t"][0]
            spin = np.linalg.norm(a["w_rob"]) / (2 * np.pi) if np.isfinite(a["w_rob"]).all() else 99
            if dur >= 0.20 and spin < 3.0:
                cands.append(a)
    cands.sort(key=lambda a: -(a["t"][-1] - a["t"][0]))
    cands = cands[:60]
    report["gravity_n_arcs"] = len(cands)
    if len(cands) >= 3:
        fit = fit_arcs_global(cands, fit=("kd", "g"))
        g = fit["g_vec"]
        gmag = float(np.linalg.norm(g))
        tilt_table = float(np.degrees(np.arccos(np.clip(-g[2] / gmag, -1, 1))))
        # tilt vs WORLD z: rotate g (table frame) back to world
        Rt = takes["dianqiu_nospin"]["table_R"]
        g_w = Rt @ g
        tilt_world = float(np.degrees(np.arccos(np.clip(-g_w[2] / gmag, -1, 1))))
        report["gravity"] = dict(
            g_mag=gmag, kd_lowspin=fit["kd"],
            tilt_vs_table_normal_deg=tilt_table, tilt_vs_world_z_deg=tilt_world,
            g_vec_table=[float(x) for x in g],
            rms_med_mm=float(np.median(fit["rms_per_arc"]) * 1e3),
            rms_p90_mm=float(np.percentile(fit["rms_per_arc"], 90) * 1e3),
            PASS_mag=bool(abs(gmag - G_NOM) <= 0.05),
            PASS_tilt=bool(min(tilt_table, tilt_world) <= 0.2))
    else:
        report["gravity"] = dict(error="not enough low-spin arcs")

    # ---- spin sanity on spin takes ----
    spin_rep = {}
    for n in ("cexuan", "shangxuan", "xiaxuan", "dianqiu_spin"):
        vals, angmaxs, decays = [], [], []
        for a in all_arcs[n]:
            if not arc_is_ballistic(a, a.get("par") or arc_parabola(a)):
                continue
            rate = float(takes[n]["rate"])
            w, ang, w_rob = spin_from_quats(a["quat"], rate)
            if not np.isfinite(w_rob).all():
                continue
            rev = np.linalg.norm(w_rob) / (2 * np.pi)
            vals.append(rev)
            angmaxs.append(np.nanmax(ang))
            # in-flight constancy: robust |w| first half vs second half (F7 preview)
            m = len(a["quat"]) // 2
            if m >= 10 and (a["t"][-1] - a["t"][0]) >= 0.4:
                _, _, w1 = spin_from_quats(a["quat"][:m], rate)
                _, _, w2 = spin_from_quats(a["quat"][m:], rate)
                if np.isfinite(w1).all() and np.isfinite(w2).all() and np.linalg.norm(w1) > 6:
                    decays.append(float(np.linalg.norm(w2) / np.linalg.norm(w1)))
        spin_rep[n] = dict(
            n=len(vals),
            rev_med=float(np.median(vals)) if vals else None,
            rev_p90=float(np.percentile(vals, 90)) if vals else None,
            rev_max=float(np.max(vals)) if vals else None,
            degframe_max=float(np.max(angmaxs)) if angmaxs else None,
            halfarc_spin_ratio_med=float(np.median(decays)) if decays else None,
            n_decay_arcs=len(decays))
    report["spin_sanity"] = spin_rep

    # ---- wobble / center-offset check on high-spin arcs ----
    # pick sidespin arcs with spin > 10 rev/s, look at shooting-fit residual RMS
    # vs spin, and fit a global body-frame delta correction.
    hi = []
    for n in ("cexuan", "shangxuan", "xiaxuan", "dianqiu_spin"):
        for a in all_arcs[n]:
            if not arc_is_ballistic(a, a.get("par") or arc_parabola(a)):
                continue
            if not np.isfinite(a["w_rob"]).all():
                continue
            if np.linalg.norm(a["w_rob"]) / (2 * np.pi) > 10 and (a["t"][-1] - a["t"][0]) > 0.25:
                hi.append(a)
    hi.sort(key=lambda a: -np.linalg.norm(a["w_rob"]))
    hi = hi[:30]
    if len(hi) >= 5:
        w_arr = np.array([a["w_rob"] for a in hi])
        base = fit_arcs_global(hi, fit=("kd", "km"), w_per_arc=w_arr)
        # delta fit: pos_corrected = pos_obs + R(t)(delta); linearized joint refit
        from scipy.optimize import least_squares as lsq
        Rmats = [Rot.from_quat(a["quat"]).as_matrix() for a in hi]

        def resid_delta(x):
            delta = x[:3]
            res = []
            for i, a in enumerate(hi):
                corr = a["pos"] + np.einsum("kij,j->ki", Rmats[i], delta)
                aa = dict(a, pos=corr)
                par = arc_parabola(aa)
                res.append(par["rms"])
            return np.array(res)

        sol = lsq(resid_delta, np.zeros(3), diff_step=1e-3)
        delta = sol.x[:3]
        rms0 = resid_delta(np.zeros(3))
        rms1 = resid_delta(delta)
        report["wobble"] = dict(
            n_arcs=len(hi),
            shootfit_rms_med_mm=float(np.median(base["rms_per_arc"]) * 1e3),
            delta_mm=[float(x * 1e3) for x in delta],
            delta_norm_mm=float(np.linalg.norm(delta) * 1e3),
            parab_rms_before_mm=float(np.median(rms0) * 1e3),
            parab_rms_after_mm=float(np.median(rms1) * 1e3))
    else:
        report["wobble"] = dict(error=f"only {len(hi)} high-spin arcs")

    with open(os.path.join(OUT, "qa_stage0.json"), "w") as f:
        json.dump(report, f, indent=2, ensure_ascii=False)
    print(json.dumps(report, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
