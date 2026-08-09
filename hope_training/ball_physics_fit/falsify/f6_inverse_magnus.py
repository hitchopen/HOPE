#!/usr/bin/env python
"""F6: inverse-Magnus sign test.

Does the lateral acceleration ever flip sign against w x v?
Model in ballcore: a = g - kd|v|v + km (w x v)  ->  fitted km < 0 on a
well-conditioned arc == lateral accel opposed to w x v (inverse Magnus).

Method: gate per-arc km fits (fit-failure gates as in F1/F2: rms cap,
spin-trust cap), bin arcs into 2D quartile cells over (Re, SR), flag any
cell with n>=4 whose MEDIAN km is negative; check the worst corner
(highest Re x lowest SR). Add sign-test p-values and a conditioning
check (predicted Magnus deflection vs the ~9 mm position noise).
"""
import os
import sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import paths
import json
import os

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

FITS = os.path.join(paths.FITS, "stage2_fits_all.json")
FLIGHTS = os.path.join(paths.SEGMENTS, "flights.json")
OUTDIR = paths.FALSIFICATION

R_BALL = 0.020
NU_SCALE = 0.04 / 1.5e-5     # Re = |v| * 0.04 / 1.5e-5
RMS_GATE_MM = 15.0           # fit-failure gate (F1/F2 style); noise floor ~9 mm
REV_TRUST = 75.0             # quaternion spin trusted only below this
NOISE_MM = 9.0               # venue position rms

fits = json.load(open(FITS))
flights = json.load(open(FLIGHTS))
km_glob = fits["km"]["km"]
rows = fits["km"]["per_arc"]

fl_by_key = {(f["take"], f["arc"]): f for f in flights}

# ---------- assemble arrays ----------
recs = []
for r in rows:
    f = fl_by_key.get((r["take"], r["arc"]))
    dur = f["dur"] if f else np.nan
    if f:
        w = np.array(f["spin_rads"], float)
        v0 = np.array(f["v0"], float)
        wxv = np.linalg.norm(np.cross(w, v0))
    else:
        wxv = np.nan
    # predicted lateral deflection at the GLOBAL km (outcome-independent
    # conditioning metric): 0.5 * a_M * dur^2
    defl_mm = 0.5 * km_glob * wxv * dur ** 2 * 1e3
    recs.append(dict(r, dur=dur, wxv=wxv, defl_mm=defl_mm,
                     Re=r["speed"] * NU_SCALE))

n_all = len(recs)
finite = [r for r in recs if np.isfinite(r["km"])]
gated = [r for r in finite
         if r["rms_mm"] <= RMS_GATE_MM and r["rev"] <= REV_TRUST]
n_gated = len(gated)

km = np.array([r["km"] for r in gated])
Re = np.array([r["Re"] for r in gated])
SR = np.array([r["sr"] for r in gated])
defl = np.array([r["defl_mm"] for r in gated])
side = np.array([r["sidespin"] for r in gated], bool)

# ---------- 2D quartile cells over (Re, SR) ----------
re_edges = np.quantile(Re, [0, .25, .5, .75, 1.0])
sr_edges = np.quantile(SR, [0, .25, .5, .75, 1.0])
re_edges[-1] += 1e-9
sr_edges[-1] += 1e-9


def sign_test_p(x):
    """One-sided binomial P(#neg >= observed | p=.5): is the median < 0?"""
    from math import comb
    n = len(x)
    k = int((np.asarray(x) < 0).sum())
    return sum(comb(n, i) for i in range(k, n + 1)) / 2 ** n


def cell_stats(kmv, Rev, SRv, tag):
    cells = []
    for i in range(4):
        for j in range(4):
            m = ((Rev >= re_edges[i]) & (Rev < re_edges[i + 1]) &
                 (SRv >= sr_edges[j]) & (SRv < sr_edges[j + 1]))
            n = int(m.sum())
            if n == 0:
                continue
            med = float(np.median(kmv[m]))
            cells.append(dict(re_bin=i, sr_bin=j, n=n, median_km=med,
                              frac_neg=float((kmv[m] < 0).mean()),
                              sign_p=float(sign_test_p(kmv[m])),
                              flagged=bool(n >= 4 and med < 0), subset=tag))
    return cells


cells_all = cell_stats(km, Re, SR, "all")
cells_side = cell_stats(km[side], Re[side], SR[side], "sidespin")
flagged = [c for c in cells_all if c["flagged"]]
flagged_side = [c for c in cells_side if c["flagged"]]

# worst corner: highest populated Re bin x lowest populated SR bin (n>=4)
pop = [c for c in cells_all if c["n"] >= 4]
worst = None
if pop:
    worst = sorted(pop, key=lambda c: (-c["re_bin"], c["sr_bin"]))[0]

# conditioning: same cells but only arcs whose predicted Magnus deflection
# at the global km exceeds the position noise (sign actually resolvable)
cond = np.isfinite(defl) & (defl >= NOISE_MM)
cells_cond = cell_stats(km[cond], Re[cond], SR[cond], "conditioned")
flagged_cond = [c for c in cells_cond if c["flagged"]]

# bootstrap CI of the median for every flagged cell
rng = np.random.default_rng(0)


def boot_ci(x, nb=5000):
    meds = np.median(rng.choice(x, (nb, len(x))), axis=1)
    return [float(np.quantile(meds, .025)), float(np.quantile(meds, .975))]


for c in flagged + flagged_cond + flagged_side:
    src = dict(all=(km, Re, SR), conditioned=(km[cond], Re[cond], SR[cond]),
               sidespin=(km[side], Re[side], SR[side]))[c["subset"]]
    kv, rv, sv = src
    m = ((rv >= re_edges[c["re_bin"]]) & (rv < re_edges[c["re_bin"] + 1]) &
         (sv >= sr_edges[c["sr_bin"]]) & (sv < sr_edges[c["sr_bin"] + 1]))
    c["median_ci95"] = boot_ci(kv[m])

# ---------- plot ----------
fig, axes = plt.subplots(1, 3, figsize=(16, 5))

for ax, cells, tag in [(axes[0], cells_all, f"all gated arcs (n={n_gated})"),
                       (axes[1], cells_cond,
                        f"conditioned: pred. Magnus defl >= {NOISE_MM:.0f} mm "
                        f"(n={int(cond.sum())})")]:
    grid = np.full((4, 4), np.nan)
    for c in cells:
        grid[c["sr_bin"], c["re_bin"]] = c["median_km"]
    vmax = np.nanmax(np.abs(grid)) or 1e-3
    im = ax.imshow(grid, origin="lower", cmap="RdBu", vmin=-vmax, vmax=vmax,
                   aspect="auto")
    for c in cells:
        star = " *" if c["flagged"] else ""
        ax.text(c["re_bin"], c["sr_bin"],
                f"{c['median_km']:.4f}\nn={c['n']}{star}",
                ha="center", va="center", fontsize=8,
                color="k" if c["n"] >= 4 else "gray")
    ax.set_xticks(range(4), [f"{re_edges[i]/1e3:.1f}-{re_edges[i+1]/1e3:.1f}"
                             for i in range(4)], fontsize=8)
    ax.set_yticks(range(4), [f"{sr_edges[i]:.2f}-{sr_edges[i+1]:.2f}"
                             for i in range(4)], fontsize=8)
    ax.set_xlabel("Re (x1000, quartile bins)")
    ax.set_ylabel("SR = R|w|/|v| (quartile bins)")
    ax.set_title(f"median per-arc km, {tag}\n(* = n>=4 & median<0)",
                 fontsize=10)
    fig.colorbar(im, ax=ax, shrink=0.8)

ax = axes[2]
sc = ax.scatter(Re / 1e3, km, c=SR, cmap="viridis", s=28,
                edgecolors=np.where(km < 0, "r", "none"), linewidths=1.2)
ax.axhline(0, color="k", lw=0.8)
ax.axhline(km_glob, color="tab:blue", lw=0.8, ls="--",
           label=f"global km = {km_glob:.4f}")
ax.axvline(1e5 / 1e3, color="r", ls=":", lw=1,
           label="inverse-Magnus regime Re~1e5 (off scale)")
ax.set_xlim(0, max(Re / 1e3) * 1.08)
ax.set_xlabel("Re (x1000)")
ax.set_ylabel("per-arc km")
ax.set_ylim(np.percentile(km, 1) - 0.02, np.percentile(km, 99) + 0.02)
ax.set_title("per-arc km vs Re (red edge = km<0)", fontsize=10)
ax.legend(fontsize=8, loc="upper right")
fig.colorbar(sc, ax=ax, shrink=0.8, label="SR")

fig.suptitle("F6: inverse-Magnus sign test  --  does lateral accel ever "
             "oppose w x v?  (a = g - kd|v|v + km w x v)", fontsize=12)
fig.tight_layout(rect=[0, 0, 1, 0.94])
os.makedirs(OUTDIR, exist_ok=True)
fig.savefig(os.path.join(OUTDIR, "F6.png"), dpi=130)

# ---------- verdict ----------
sig_flagged = [c for c in flagged if c["sign_p"] < 0.05]
sig_flagged_cond = [c for c in flagged_cond if c["sign_p"] < 0.05]

report = dict(
    n_arcs_total=n_all, n_gated=n_gated, n_conditioned=int(cond.sum()),
    gates=dict(rms_mm_max=RMS_GATE_MM, rev_max=REV_TRUST,
               n_dropped_rms=n_all - n_gated),
    km_global=km_glob,
    re_range=[float(Re.min()), float(Re.max())],
    sr_range=[float(SR.min()), float(SR.max())],
    frac_negative_arcs=float((km < 0).mean()),
    re_edges=[float(x) for x in re_edges],
    sr_edges=[float(x) for x in sr_edges],
    cells_all=cells_all, cells_conditioned=cells_cond,
    cells_sidespin=cells_side,
    flagged_all=flagged, flagged_conditioned=flagged_cond,
    flagged_sidespin=flagged_side,
    sig_flagged_all=sig_flagged, sig_flagged_conditioned=sig_flagged_cond,
    worst_corner_cell=worst,
)
with open(os.path.join(OUTDIR, "F6_report.json"), "w") as fh:
    json.dump(report, fh, indent=1)

print(json.dumps({k: v for k, v in report.items()
                  if not k.startswith("cells")}, indent=1))
print("\nflagged (all):", json.dumps(flagged, indent=1))
print("flagged (conditioned):", json.dumps(flagged_cond, indent=1))
print("flagged (sidespin):", json.dumps(flagged_side, indent=1))
print("worst corner:", json.dumps(worst, indent=1))
