"""Causal speed-stratified validation; no ROS or deployment actions.

Split and model family fixed before scores: train files 1+2, test file 3.
Bin edges use training-prefix speed tertiles only. Test speed and spin use
the first 150 ms only. Three constant-parameter models test whether separate
operating-speed calibrations are justified; not an instantaneous drag law.
"""
import json
import os
from pathlib import Path

import numpy as np

from ballcore import spin_from_quats
from refit_c3d_recording import fit, causal, summary, kernel, dump
from refit_ppq_multibounce import position_contacts
from refit_table_contact import fit_contact, predict, evaluate

OUT = Path(os.environ.get("PPQ004_FIT_DIR", str(Path(__file__).resolve().parent.parent / "fit")))


def paired(rows, key_a, key_b, seed=1020):
    """Paired 10 s block bootstrap; descriptive within one recording."""
    blocks = sorted({int(r["t0"] // 10) for r in rows})
    rng = np.random.default_rng(seed)
    delta = []
    for _ in range(2000):
        sampled = rng.choice(blocks, len(blocks), replace=True)
        rr = [r for b in sampled for r in rows if int(r["t0"] // 10) == b]
        delta.append(np.median([r[key_b] for r in rr]) -
                     np.median([r[key_a] for r in rr]))
    return {"blocks": len(blocks), "difference_median": float(
        np.median([r[key_b] for r in rows]) - np.median([r[key_a] for r in rows])),
        "block_bootstrap_95": np.percentile(delta, [2.5, 97.5]) if len(blocks) >= 3 else None}


def main():
    arcs = []
    offset = 0
    for part in (1, 2, 3):
        d = dict(np.load(OUT / f"part{part}/raw_extracted.npz"))
        aa, _, _ = position_contacts(d, part, offset, OUT)
        arcs += aa
        offset += len(d["t"]) / 300
    integrate = kernel(OUT)
    baseline = json.loads((OUT / "baseline_ppq002.json").read_text())
    basepar = [baseline["drag_k"], baseline["magnus_k"]]
    trainpar = json.loads((OUT / "training_fit.json").read_text())["params"]
    eligible = [a for a in arcs if len(a["t"]) >= 61]
    for a in eligible:
        b = dict(a, t=a["t"][:46], pos=a["pos"][:46], quat=a["quat"][:46])
        _, _, b["w"] = spin_from_quats(b["quat"], 300)
        ff = fit([b], integrate, basepar, False)
        assert ff["success"]
        a["causal_speed"] = float(np.linalg.norm(ff["states"][0, 3:]))
    train = [a for a in eligible if a["part"] != 3]
    test = [a for a in eligible if a["part"] == 3]
    # Fair comparison: the unified and speed-specific models use identical
    # training arcs; short arcs excluded from causal scoring are excluded here.
    matched_fit = fit(train, integrate, trainpar)
    assert matched_fit["success"]
    trainpar = matched_fit["params"]
    edges = np.quantile([a["causal_speed"] for a in train], [1/3, 2/3])
    group = lambda a: int(np.searchsorted(edges, a["causal_speed"], side="right"))
    models = []
    for k in range(3):
        aa = [a for a in train if group(a) == k]
        assert len(aa) >= 15
        ff = fit(aa, integrate, trainpar)
        assert ff["success"]
        models.append(ff["params"])
        print("SPEED MODEL", k, "n", len(aa), "params", ff["params"], flush=True)
    unified = {r["id"]: r for r in causal(test, integrate, trainpar)}
    old = {r["id"]: r for r in causal(test, integrate, basepar)}
    by_speed = {}
    per_bin = []
    for k in range(3):
        aa = [a for a in test if group(a) == k]
        rr = causal(aa, integrate, models[k])
        by_speed.update({r["id"]: r for r in rr})
        per_bin.append({"bin": k, "train_n": sum(group(a) == k for a in train),
                        "test_n": len(aa), "parameters": models[k],
                        "ppq002": summary([old[a["id"]] for a in aa]),
                        "unified": summary([unified[a["id"]] for a in aa]),
                        "speed_specific": summary(rr),
                        "observed_prefix_speed_range_mps": [min(a["causal_speed"] for a in aa),
                                                          max(a["causal_speed"] for a in aa)] if aa else None})
    rows = [{"id": a["id"], "t0": float(a["t"][0]), "speed": a["causal_speed"],
             "bin": group(a), "unified": unified[a["id"]]["endpoint_mm"],
             "speed_specific": by_speed[a["id"]]["endpoint_mm"]} for a in test]
    comparison = paired(rows, "unified", "speed_specific")
    us = summary(list(unified.values()))
    ss = summary(list(by_speed.values()))
    ci = comparison["block_bootstrap_95"]
    justified = (all(r["test_n"] >= 8 for r in per_bin) and ci is not None and ci[1] < 0
                 and ss["endpoint_mm"]["median"] < .95 * us["endpoint_mm"]["median"]
                 and ss["endpoint_mm"]["p90"] <= us["endpoint_mm"]["p90"])
    result = {"definition": __doc__, "train_ids": [a["id"] for a in train],
              "test_ids": [a["id"] for a in test], "training_speed_tertiles_mps": edges,
              "matched_unified_training_parameters": trainpar,
              "per_bin": per_bin, "unified": us, "speed_specific": ss,
              "paired_endpoint": comparison, "rows": rows,
              "flight_speed_adaptation_supported": bool(justified),
              "adoption_rule": "At least 8 test arcs per bin, >5% median improvement, "
              "no P90 degradation, and upper paired block interval <0"}
    dump(OUT / "speed_validation.json", result)
    print("FLIGHT SPEED RESULT", json.dumps(result, default=lambda x: x.tolist()), flush=True)

    contact = json.loads((OUT / "spin_contacts_strict_holdout.json").read_text())["accepted"]
    tr = [r for r in contact if r["part"] != 3]
    te = [r for r in contact if r["part"] == 3]
    uniform, _ = fit_contact(tr)
    cr = {}
    for axis in ("total_speed", "normal_speed"):
        speed = lambda r: float(np.linalg.norm(r["vin"])) if axis == "total_speed" else abs(r["vin"][2])
        ce = np.quantile([speed(r) for r in tr], [1/3, 2/3])
        cg = lambda r: int(np.searchsorted(ce, speed(r), side="right"))
        bins = []
        predicted_v, predicted_w = [], []
        for k in range(3):
            train_rows = [r for r in tr if cg(r) == k]
            test_rows = [r for r in te if cg(r) == k]
            par, info = fit_contact(train_rows)
            assert info["success"]
            bins.append({"bin": k, "train_n": len(train_rows), "test_n": len(test_rows),
                         "params": par, "fit": info,
                         "unified": evaluate(test_rows, uniform) if test_rows else None,
                         "speed_specific": evaluate(test_rows, par) if test_rows else None})
            if test_rows:
                vv, ww, _ = predict(par, np.array([r["vin"] for r in test_rows]),
                                    np.array([r["win"] for r in test_rows]))
                predicted_v.extend(np.linalg.norm(vv-np.array([r["vout"] for r in test_rows]), axis=1))
                predicted_w.extend(np.linalg.norm(ww-np.array([r["wout"] for r in test_rows]), axis=1))
        cr[axis] = {"training_tertiles_mps": ce, "bins": bins,
                    "speed_specific_velocity_median_p90": np.percentile(predicted_v, [50, 90]),
                    "speed_specific_spin_median_p90": np.percentile(predicted_w, [50, 90])}
    dump(OUT / "contact_speed_validation.json", {
        "unified": evaluate(te, uniform), "train_n": len(tr), "test_n": len(te),
        "analyses": cr, "speed_adaptation_supported": False,
        "limitation": "Only file 3 contacts are held out. Fewer than 8 per speed bin "
        "cannot establish a speed law. Contact velocities are reconstructed, not direct truth."})


if __name__ == "__main__":
    main()
