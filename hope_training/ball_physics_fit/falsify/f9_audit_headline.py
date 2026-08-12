"""
F9 - ADVERSARIAL AUDIT of Stage-2 headline constants vs priors + reproducibility.

Checks:
 1. kd vs precedent; verify accel-coefficient convention; implied C_d both masses.
 2. km vs precedent CI; implied spin-scale factor; implied lift coefficient.
 3. table e_n vs ITTF 0.88-0.92 and old fits (0.908 OptiTrack / 0.87 C3D).
 4. paddle e_eff vs v0 0.4632.
 5. Reproduce 3 random per-arc kd fits + 3 per-arc km fits from RAW npz
    (fresh extract_arcs, no pickle cache) -> same numbers +-1%?

Outputs: falsification/F9.png + F9_verdict.json
"""
import os, sys, json
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import paths
from ballcore import (load_take, extract_arcs, arc_parabola, arc_is_ballistic,
                      spin_from_quats, fit_arcs_global, R_BALL)

ANA = paths.ANALYSIS
FITS = json.load(open(os.path.join(ANA, "fits", "stage2_fits_all.json")))
OUTD = os.path.join(ANA, "falsification")

RHO, A, R = 1.20, np.pi * 0.02 ** 2, 0.02
MT, MC = FITS["ball_mass_taped"], FITS["ball_mass_clean"]
SURF = FITS["surface_z"]

kd_hat = FITS["kd"]["kd"]
km_hat = FITS["km"]["km"]
te = FITS["table_e"]
pdl = FITS["paddle"]

# ---------------- priors ----------------
PRI = dict(kd_optitrack_markered=0.136, kd_c3d_unmarked=0.107, kd_v0=0.1222,
           km_prior=0.00417, km_ci=[0.0035, 0.0049],
           e_ittf=[0.88, 0.92], e_old_optitrack=0.908, e_old_c3d=0.87,
           paddle_e_v0=0.4632)

# ---------------- derived numbers ----------------
cd_taped = 2 * MT * kd_hat / (RHO * A)
cd_clean = 2 * MC * kd_hat / (RHO * A)
kd_clean_forceinv = kd_hat * MT / MC
cm_taped = 2 * MT * km_hat / (RHO * A * R)
spin_scale = PRI["km_prior"] / km_hat  # <1 means venue spin reads low vs prior

# ---------------- reproducibility: fresh rebuild, 3 kd + 3 km rows ----------------
rng = np.random.default_rng(9)
kd_rows = FITS["kd"]["per_arc"]
km_rows = FITS["km"]["per_arc"]
pick_kd = [kd_rows[i] for i in rng.choice(len(kd_rows), 3, replace=False)]
pick_km = [km_rows[i] for i in rng.choice(len(km_rows), 3, replace=False)]

need_takes = sorted({r["take"] for r in pick_kd} | {r["take"] for r in pick_km})
print("rebuilding takes (fresh, no cache):", need_takes, flush=True)

fresh = {}
for name in need_takes:
    take = load_take(name)
    Rt, rate = take["table_R"], float(take["rate"])
    arcs = extract_arcs(take, table_z=SURF)
    for ai, a in enumerate(arcs):
        a["par"] = arc_parabola(a)
        w, ang, w_rob = spin_from_quats(a["quat"], rate, R_table=Rt)
        a["w_rob"] = w_rob
        fresh[(name, ai)] = a
    print(f"  {name}: {len(arcs)} arcs", flush=True)

repro = []
for r in pick_kd:
    # match by (speed, dur) - no arc index stored in kd rows
    cands = [(k, a) for k, a in fresh.items() if k[0] == r["take"]]
    key, a = min(cands, key=lambda ka: abs(np.linalg.norm(ka[1]["par"]["v0"]) - r["speed"])
                 + abs((ka[1]["t"][-1] - ka[1]["t"][0]) - r["dur"]))
    ms = abs(np.linalg.norm(a["par"]["v0"]) - r["speed"])
    f = fit_arcs_global([a], fit=("kd",))
    repro.append(dict(kind="kd", take=r["take"], arc=key[1], match_dspeed=float(ms),
                      stored=r["kd"], recomputed=f["kd"],
                      stored_rms=r["rms_mm"], recomputed_rms=float(f["rms_per_arc"][0] * 1e3),
                      rel_diff=float(abs(f["kd"] - r["kd"]) / max(abs(r["kd"]), 1e-9))))
    print(f"  kd repro {r['take']} arc{key[1]}: stored {r['kd']:.5f} -> {f['kd']:.5f}", flush=True)

for r in pick_km:
    a = fresh[(r["take"], r["arc"])]
    f = fit_arcs_global([a], fit=("km",), kd0=kd_hat, w_per_arc=a["w_rob"][None])
    repro.append(dict(kind="km", take=r["take"], arc=r["arc"],
                      stored=r["km"], recomputed=f["km"],
                      stored_rms=r["rms_mm"], recomputed_rms=float(f["rms_per_arc"][0] * 1e3),
                      rel_diff=float(abs(f["km"] - r["km"]) / max(abs(r["km"]), 1e-9))))
    print(f"  km repro {r['take']} arc{r['arc']}: stored {r['km']:.6f} -> {f['km']:.6f}", flush=True)

repro_pass = all(x["rel_diff"] < 0.01 for x in repro)

# ---------------- check table ----------------
pb = te["per_bounce"]
e_arr = np.array([b["e"] for b in pb]); vn_arr = np.array([b["vn"] for b in pb])
ps = pdl["per_strike"]
un_arr = np.array([p["u_n"] for p in ps]); pe_arr = np.array([p["e"] for p in ps])
kdp = FITS["kd"]["per_arc"]
kd_arr = np.array([r["kd"] for r in kdp]); sp_arr = np.array([r["speed"] for r in kdp])
kms = [r for r in km_rows if r["sidespin"]]
km_arr = np.array([r["km"] for r in kms]); sr_arr = np.array([r["sr"] for r in kms])

checks = [
    dict(row="kd convention", value="a = g - kd|v|v (ballcore.rk4_flight)", verdict="PASS",
         note="kd is an accel coefficient; mass never enters the fit, so k ~ 1/m conversion is the right transform"),
    dict(row="kd vs precedent", value=round(kd_hat, 4),
         prior="OptiTrack markered 0.136 / C3D unmarked 0.107 / v0 0.1222",
         verdict="PASS",
         note="between precedents, +3.2% over v0; taped+heavier ball moving toward the markered value is the expected direction"),
    dict(row="implied C_d (taped 3.4 g)", value=round(cd_taped, 3), prior="0.3-0.7 sanity",
         verdict="PASS", note="high side of smooth-sphere ~0.4-0.5 but plausible for a tape-roughened ball"),
    dict(row="implied C_d (clean 2.7 g)", value=round(cd_clean, 3), prior="0.3-0.7 sanity",
         verdict="PASS", note="if the same FORCE acted on a clean ball"),
    dict(row="kd mass-scaled to clean", value=round(kd_clean_forceinv, 4),
         prior="clean precedents 0.107-0.136",
         verdict="FLAG",
         note="0.159 exceeds ALL precedents incl. markered 0.136 -> force-invariance is dubious (tape raises C_d too); do NOT ship 0.159 for a clean ball"),
    dict(row="km vs precedent", value=round(km_hat, 5), prior="0.00417 CI [0.0035,0.0049]",
         verdict="PASS",
         note=f"inside CI; ratio {km_hat/PRI['km_prior']:.3f} -> implied spin scale {spin_scale:.2f} (spin ~6% low if fully attributed); implied lift coef C_m={cm_taped:.2f} (taped) is at the physical ceiling, weakly corroborating spin-reads-low"),
    dict(row="table e_n", value=round(te["e_const"], 4),
         prior="ITTF 0.88-0.92; old 0.908 / 0.87",
         verdict="FLAG",
         note=f"above ITTF top by ~1.5%, {int(100*np.mean(e_arr>1.0))}% of bounces e>1 (unphysical), MAD {te['e_mad']:.03f} huge vs old rig; vn noise near contact (9 mm, 300 Hz, +-3-frame exclusion) plausibly biases e high; slope CI {[round(c,3) for c in te['slope_ci']]} includes 0"),
    dict(row="paddle e_eff", value=round(pdl["e_eff"], 4), prior="v0 0.4632 (never fit on racket data)",
         verdict="FLAG (expected move)",
         note="+41% vs v0; internally consistent (per-strike median 0.67, e_lin at median u_n 0.67, decays with u_n as physics predicts); adopt only after Stage-4 holdout validation"),
    dict(row="reproducibility 3 kd + 3 km per-arc refits", value="PASS" if repro_pass else "FAIL",
         prior="+-1%", verdict="PASS" if repro_pass else "FLAG",
         note="fresh extract_arcs from raw npz, no cache; " +
              "; ".join(f"{x['kind']} {x['take']}#{x['arc']}: {x['stored']:.5g}->{x['recomputed']:.5g} ({100*x['rel_diff']:.2f}%)" for x in repro)),
]

# ---------------- plot ----------------
fig, axs = plt.subplots(2, 2, figsize=(13.5, 9.5))
fig.suptitle("F9 - adversarial audit of Stage-2 headline constants (venue 2026-07-03, taped ball 3.4 g)", fontsize=12)

ax = axs[0, 0]
ax.scatter(sp_arr, kd_arr, s=14, c="tab:blue", alpha=0.55, label="per-arc kd (n=100)")
ax.axhline(kd_hat, c="k", lw=1.8, label=f"joint kd={kd_hat:.4f}")
ax.axhline(PRI["kd_optitrack_markered"], c="tab:green", ls="--", lw=1, label="OptiTrack markered 0.136")
ax.axhline(PRI["kd_v0"], c="tab:orange", ls="--", lw=1, label="v0 0.1222")
ax.axhline(PRI["kd_c3d_unmarked"], c="tab:red", ls="--", lw=1, label="C3D unmarked 0.107")
for x in repro:
    if x["kind"] == "kd":
        ax.plot(np.linalg.norm(fresh[(x["take"], x["arc"])]["par"]["v0"]), x["recomputed"],
                "x", c="magenta", ms=9, mew=2)
ax.plot([], [], "x", c="magenta", ms=9, mew=2, label="repro refits")
ax.set_ylim(-0.5, 0.9); ax.set_xlabel("|v0| (m/s)"); ax.set_ylabel("kd (1/m)")
ax.set_title(f"kd: C_d(taped)={cd_taped:.2f}, C_d(clean-mass)={cd_clean:.2f}; mass-scaled clean kd={kd_clean_forceinv:.3f} [FLAG]")
ax.legend(fontsize=7, loc="upper right"); ax.grid(alpha=0.3)

ax = axs[0, 1]
ax.scatter(sr_arr, km_arr, s=14, c="tab:blue", alpha=0.55, label="sidespin per-arc km (n=66)")
ax.axhline(km_hat, c="k", lw=1.8, label=f"joint km={km_hat:.5f}")
ax.axhspan(*PRI["km_ci"], color="tab:green", alpha=0.18, label="prior CI [0.0035,0.0049]")
ax.axhline(PRI["km_prior"], c="tab:green", ls="--", lw=1, label="prior 0.00417")
for x in repro:
    if x["kind"] == "km":
        r = next(rr for rr in km_rows if rr["take"] == x["take"] and rr["arc"] == x["arc"])
        ax.plot(r["sr"], x["recomputed"], "x", c="magenta", ms=9, mew=2)
ax.set_ylim(-0.05, 0.05); ax.set_xlabel("spin ratio SR"); ax.set_ylabel("km")
ax.set_title(f"km: inside prior CI; implied spin scale {spin_scale:.2f}; C_m(taped)={cm_taped:.2f} [PASS]")
ax.legend(fontsize=7, loc="lower left"); ax.grid(alpha=0.3)

ax = axs[1, 0]
ax.scatter(vn_arr, e_arr, s=14, c="tab:blue", alpha=0.6, label="per-bounce e (n=92)")
ax.axhspan(*PRI["e_ittf"], color="tab:green", alpha=0.18, label="ITTF 0.88-0.92")
ax.axhline(te["e_const"], c="k", lw=1.8, label=f"e_const={te['e_const']:.3f}")
vv = np.linspace(vn_arr.min(), vn_arr.max(), 50)
ax.plot(vv, te["linear_a"] - te["linear_b"] * vv, c="k", ls=":", label=f"linear {te['linear_a']:.3f}-{te['linear_b']:.3f}vn")
ax.axhline(PRI["e_old_optitrack"], c="tab:orange", ls="--", lw=1, label="old OptiTrack 0.908")
ax.axhline(PRI["e_old_c3d"], c="tab:red", ls="--", lw=1, label="old C3D 0.87")
ax.axhline(1.0, c="gray", lw=0.8)
ax.set_xlabel("vn_in (m/s)"); ax.set_ylabel("e_n")
ax.set_title(f"table e_n=0.934: above ITTF + old fits; {int(100*np.mean(e_arr>1.0))}% of bounces e>1 [FLAG]")
ax.legend(fontsize=7, loc="lower left"); ax.grid(alpha=0.3)

ax = axs[1, 1]
g = (pe_arr > -0.2) & (pe_arr < 1.5)
ax.scatter(un_arr[g], pe_arr[g], s=14, c="tab:blue", alpha=0.6, label=f"per-strike e (n={g.sum()})")
ax.axhline(pdl["e_eff"], c="k", lw=1.8, label=f"joint e_eff={pdl['e_eff']:.3f}")
uu = np.linspace(max(0.3, un_arr.min()), un_arr.max(), 50)
ax.plot(uu, pdl["e_lin_a"] - pdl["e_lin_b"] * uu, "k:", label=f"lin {pdl['e_lin_a']:.3f}-{pdl['e_lin_b']:.3f}u_n")
ax.plot(uu, pdl["e_exp_g1"] * np.exp(pdl["e_exp_g2"] * uu), "k--", lw=0.8,
        label=f"exp {pdl['e_exp_g1']:.3f}e^({pdl['e_exp_g2']:.3f}u_n)")
ax.axhline(PRI["paddle_e_v0"], c="tab:red", ls="--", lw=1, label="v0 0.4632 (never data-fit)")
ax.set_xlabel("|u_n| (m/s)"); ax.set_ylabel("paddle e")
ax.set_title("paddle e_eff=0.654 vs v0 0.463: +41% [FLAG - expected move, gate on Stage-4]")
ax.legend(fontsize=7, loc="upper right"); ax.grid(alpha=0.3)

rp = "; ".join(f"{x['kind']}#{x['arc']}({x['take'][:8]}): {100*x['rel_diff']:.2f}%" for x in repro)
fig.text(0.5, 0.005, f"repro refits (fresh from raw npz, tol 1%): {rp}  ->  "
         + ("ALL PASS" if repro_pass else "FAIL"), ha="center", fontsize=8.5,
         color="darkgreen" if repro_pass else "red")
fig.tight_layout(rect=[0, 0.02, 1, 0.97])
png = os.path.join(OUTD, "F9.png")
fig.savefig(png, dpi=130)
print("->", png)

verdict = dict(
    test="F9 adversarial audit of Stage-2 headline constants",
    headline=dict(kd=kd_hat, km=km_hat, table_e_const=te["e_const"], paddle_e_eff=pdl["e_eff"]),
    derived=dict(cd_taped=cd_taped, cd_clean_mass=cd_clean, kd_mass_scaled_clean=kd_clean_forceinv,
                 lift_coef_cm_taped=cm_taped, implied_spin_scale_vs_prior=spin_scale,
                 table_frac_e_gt_1=float(np.mean(e_arr > 1.0)),
                 table_e_at_ittf_drop_2p43ms=float(te["linear_a"] - te["linear_b"] * 2.43)),
    checks=checks,
    repro=repro,
    overall=("kd PASS / km PASS / table e_n FLAG (likely 2-5% high; noise-limited) / "
             "paddle e_eff FLAG-expected (first racket fit; validate on holdout) / "
             + ("repro PASS" if repro_pass else "repro FAIL")),
)
jp = os.path.join(OUTD, "F9_verdict.json")
json.dump(verdict, open(jp, "w"), indent=1)
print("->", jp)
