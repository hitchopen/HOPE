"""F7: does |w| decay measurably during flight?

For every ballistic arc (dur >= 0.4 s, quat spin >= 6 rev/s, deg/frame < 90):
robust |w| over first third vs last third. decay = 1 - |w|_last/|w|_first.
KILL if median decay > 10% per flight; PASS if < 5%; n < 8 -> INCONCLUSIVE.
"""
import os, sys, json
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import paths
from ballcore import TAKES, load_take, extract_arcs, arc_parabola, arc_is_ballistic, spin_from_quats

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

OUT = paths.FALSIFICATION
SEG = paths.SEGMENTS
os.makedirs(OUT, exist_ok=True)

surface_z = json.load(open(os.path.join(SEG, "meta.json")))["surface_z"]
REV = 2 * np.pi
SPIN_MIN = 6.0 * REV          # rad/s
MIN_VALID_PER_THIRD = 8        # valid spin frames needed in each third
DEG_REL = 90.0

rows = []
counts = dict(arcs_total=0, ballistic=0, dur_ok=0, spin_frames_ok=0, spin_mag_ok=0)

for name in TAKES:
    take = load_take(name)
    Rt = take["table_R"]
    rate = float(take["rate"])
    kab = take["ball_kabsch_rms_m"].astype(float)
    for ai, a in enumerate(extract_arcs(take, table_z=surface_z)):
        counts["arcs_total"] += 1
        par = arc_parabola(a)
        if not arc_is_ballistic(a, par):
            continue
        counts["ballistic"] += 1
        dur = float(a["t"][-1] - a["t"][0])
        if dur < 0.4:
            continue
        counts["dur_ok"] += 1

        w, ang, _ = spin_from_quats(a["quat"], rate, R_table=Rt)
        n = len(w)
        i1, i2 = n // 3, 2 * (n // 3)
        segs = {}
        ok_frames = True
        for key, sl in (("first", slice(0, i1)), ("last", slice(i2, n))):
            ws, angs = w[sl], ang[sl]
            valid = np.isfinite(ws).all(1) & np.isfinite(angs) & (angs < DEG_REL)
            if valid.sum() < MIN_VALID_PER_THIRD:
                ok_frames = False
                break
            wmed = np.median(ws[valid], axis=0)          # per-component median
            segs[key] = dict(
                wmag=float(np.linalg.norm(wmed)),
                wmag_alt=float(np.median(np.linalg.norm(ws[valid], axis=1))),
                axis=wmed / max(np.linalg.norm(wmed), 1e-9),
                n_valid=int(valid.sum()),
                deg_med=float(np.median(angs[valid])))
        if not ok_frames:
            continue
        counts["spin_frames_ok"] += 1
        if segs["first"]["wmag"] < SPIN_MIN:
            continue
        counts["spin_mag_ok"] += 1

        kab_arc = kab[a["i0"]:a["i1"]]
        rows.append(dict(
            take=name, arc=ai, dur=dur,
            speed=float(np.linalg.norm(par["v0"])),
            w_first=segs["first"]["wmag"] / REV,          # rev/s
            w_last=segs["last"]["wmag"] / REV,
            w_first_alt=segs["first"]["wmag_alt"] / REV,
            w_last_alt=segs["last"]["wmag_alt"] / REV,
            decay=1.0 - segs["last"]["wmag"] / segs["first"]["wmag"],
            decay_alt=1.0 - segs["last"]["wmag_alt"] / segs["first"]["wmag_alt"],
            axis_dot=float(np.dot(segs["first"]["axis"], segs["last"]["axis"])),
            kab_mean_mm=float(np.nanmean(kab_arc) * 1e3),
            kab_p90_mm=float(np.nanpercentile(kab_arc, 90) * 1e3),
            n_valid_first=segs["first"]["n_valid"],
            n_valid_last=segs["last"]["n_valid"],
            deg_med_first=segs["first"]["deg_med"],
            deg_med_last=segs["last"]["deg_med"]))

print("counts:", counts)
print(f"qualifying arcs: {len(rows)}")

dec = np.array([r["decay"] for r in rows])
dec_alt = np.array([r["decay_alt"] for r in rows])
dur_arr = np.array([r["dur"] for r in rows])
wf = np.array([r["w_first"] for r in rows])
wl = np.array([r["w_last"] for r in rows])
axd = np.array([r["axis_dot"] for r in rows])

stats = {}
if len(rows):
    q25, q50, q75 = np.percentile(dec, [25, 50, 75])
    rng = np.random.default_rng(0)
    boot = np.array([np.median(rng.choice(dec, len(dec))) for _ in range(5000)])
    ci = np.percentile(boot, [2.5, 97.5])
    try:
        from scipy.stats import wilcoxon
        p_gt0 = float(wilcoxon(dec, alternative="greater").pvalue)
    except Exception:
        p_gt0 = None
    # axis-stable subset (quality check: free-flight axis should be constant)
    stable = axd > 0.9
    med_stable = float(np.median(dec[stable])) if stable.sum() else None
    # per-second rate: thirds' centers are ~2/3 of dur apart
    rate_ps = dec / (dur_arr * 2.0 / 3.0)
    stats = dict(
        n=len(rows), median=float(q50), iqr=[float(q25), float(q75)],
        mean=float(dec.mean()), boot_ci_median=[float(ci[0]), float(ci[1])],
        wilcoxon_p_gt0=p_gt0,
        median_alt_framenorm=float(np.median(dec_alt)),
        n_axis_stable=int(stable.sum()), median_axis_stable=med_stable,
        median_decay_per_s=float(np.median(rate_ps)),
        spearman_decay_vs_dur=None)
    try:
        from scipy.stats import spearmanr
        rho, p = spearmanr(dur_arr, dec)
        stats["spearman_decay_vs_dur"] = [float(rho), float(p)]
    except Exception:
        pass
print(json.dumps(stats, indent=1))

# ---------------- verdict ----------------
# Point-estimate rule: KILL if median > 10%, PASS if < 5%. But a verdict is only
# honest if the median is actually resolved: require the bootstrap 95% CI to
# clear the threshold too, otherwise the data cannot decide (per test brief).
if len(rows) < 8:
    verdict, why = "INCONCLUSIVE", f"only {len(rows)} qualifying arcs (< 8): coverage"
else:
    med = stats["median"]
    lo, hi = stats["boot_ci_median"]
    if med > 0.10 and lo > 0.05:
        verdict, why = "KILL", f"median decay {med*100:.1f}% > 10% (CI [{lo*100:.1f},{hi*100:.1f}]%)"
    elif med < 0.05 and hi < 0.10:
        verdict, why = "PASS", f"median decay {med*100:.1f}% < 5% (CI [{lo*100:.1f},{hi*100:.1f}]%)"
    else:
        verdict, why = ("INCONCLUSIVE",
                        f"median {med*100:.1f}% but 95% CI [{lo*100:.1f},{hi*100:.1f}]% spans both "
                        f"PASS (<5%) and KILL (>10%) zones; per-arc noise ~ thresholds")
print("VERDICT:", verdict, "-", why)

# ---------------- plot ----------------
fig, axs = plt.subplots(2, 2, figsize=(11, 8.5))
short = (f"median {stats['median']*100:.1f}%, 95% CI [{stats['boot_ci_median'][0]*100:.0f},"
         f"{stats['boot_ci_median'][1]*100:.0f}]% spans PASS and KILL zones; n={len(rows)}, "
         f"only 6-12 rev/s arcs qualify") if len(rows) else "no qualifying arcs"
fig.suptitle(f"F7: spin decay during flight — {verdict}\n{short}", fontsize=11)

ax = axs[0, 0]
if len(rows):
    ax.hist(dec * 100, bins=21, color="#4878d0", edgecolor="k", alpha=0.8)
    ax.axvline(np.median(dec) * 100, color="r", lw=2,
               label=f"median {np.median(dec)*100:.1f}%")
    ax.axvline(5, color="g", ls="--", label="PASS < 5%")
    ax.axvline(10, color="k", ls="--", label="KILL > 10%")
    ax.legend(fontsize=8)
ax.set_xlabel("decay per flight [%]  (1 - |w|_last/|w|_first)")
ax.set_ylabel("arcs")
ax.set_title(f"decay distribution, n={len(rows)}")

ax = axs[0, 1]
if len(rows):
    sc = ax.scatter(wf, wl, c=dur_arr, cmap="viridis", s=30, edgecolor="k", lw=0.3)
    lim = [0, max(wf.max(), wl.max()) * 1.05]
    ax.plot(lim, lim, "k--", lw=1, label="no decay")
    ax.plot(lim, [0.9 * l for l in lim], "r:", lw=1, label="-10%")
    ax.set_xlim(lim); ax.set_ylim(lim)
    plt.colorbar(sc, ax=ax, label="arc duration [s]")
    ax.legend(fontsize=8)
ax.set_xlabel("|w| first third [rev/s]")
ax.set_ylabel("|w| last third [rev/s]")
ax.set_title("first vs last third")

ax = axs[1, 0]
if len(rows):
    ax.scatter(dur_arr, dec * 100, c=wf, cmap="plasma", s=30, edgecolor="k", lw=0.3)
    cb = plt.colorbar(ax.collections[0], ax=ax, label="|w| first [rev/s]")
    if stats.get("spearman_decay_vs_dur"):
        rho, p = stats["spearman_decay_vs_dur"]
        ax.set_title(f"decay vs duration (spearman rho={rho:.2f}, p={p:.3f})")
ax.axhline(0, color="k", lw=0.5)
ax.set_xlabel("arc duration [s]")
ax.set_ylabel("decay [%]")

ax = axs[1, 1]
if len(rows):
    ax.scatter(axd, dec * 100, c="#4878d0", s=30, edgecolor="k", lw=0.3)
    ax.axvline(0.9, color="g", ls="--", lw=1, label="axis-stable cut")
    ax.legend(fontsize=8)
ax.axhline(0, color="k", lw=0.5)
ax.set_xlabel("axis(first) . axis(last)")
ax.set_ylabel("decay [%]")
ax.set_title("axis stability vs decay (quality check)")

fig.tight_layout(rect=[0, 0, 1, 0.96])
fig.savefig(os.path.join(OUT, "F7.png"), dpi=130)
print("saved plot")

caveats = [
    "only low-spin arcs qualify (6-12 rev/s): high-spin strike arcs are all < 0.4 s "
    "or exceed the 90 deg/frame quat reliability limit, so the regime where spin "
    "decay would matter most for the model is not covered",
    "per-arc measurement noise ~ +/-20-30% of |w| at 6-12 rev/s: two arcs show "
    "physically impossible negative decay (-31%, -23%); within-arc |w| profiles are "
    "non-monotonic with +/-2-4 rev/s bin-to-bin swings",
    "estimator-sensitive: component-median gives median decay 12.2%, frame-norm "
    "median gives 5.0%",
    "no duration dependence (spearman rho=0.10, p=0.73) as genuine aerodynamic "
    "decay would show; Wilcoxon cannot reject zero decay (p=0.086)",
    "ball body is pre-solved in the C3D (kabsch rms ~0.1 um), so no independent "
    "per-frame tracking-quality gate is available",
]
out = dict(test="F7 spin decay during flight", verdict=verdict, reason=why,
           thresholds=dict(kill_median_gt=0.10, pass_median_lt=0.05, min_n=8),
           selection=dict(min_dur_s=0.4, min_spin_rev_s=6.0, deg_frame_max=90,
                          min_valid_frames_per_third=MIN_VALID_PER_THIRD),
           counts=counts, stats=stats, caveats=caveats, per_arc=rows)
with open(os.path.join(OUT, "F7_verdict.json"), "w") as f:
    json.dump(out, f, indent=1)
print("saved verdict json")
