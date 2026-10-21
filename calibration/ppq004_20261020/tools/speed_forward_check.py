"""Second temporal check: train only file 1, validate on file 2.

This is a follow-up robustness check, not a new blind holdout: file 2 has
already contributed to the main fit. No coefficients are deployed from here.
"""
import json
import numpy as np
from validate_speed import OUT, paired
from refit_ppq_multibounce import position_contacts
from refit_c3d_recording import kernel, fit, causal, summary, dump
from ballcore import spin_from_quats

arcs = []
offset = 0
for part in (1, 2):
    d = dict(np.load(OUT/f"part{part}/raw_extracted.npz"))
    aa, _, _ = position_contacts(d, part, offset, OUT)
    arcs.extend(a for a in aa if len(a["t"]) >= 61)
    offset += len(d["t"])/300
integrate = kernel(OUT)
baseline = json.loads((OUT/"baseline_ppq002.json").read_text())
base = [baseline["drag_k"], baseline["magnus_k"]]
for a in arcs:
    b = dict(a, t=a["t"][:46], pos=a["pos"][:46], quat=a["quat"][:46])
    _, _, b["w"] = spin_from_quats(b["quat"], 300)
    f = fit([b], integrate, base, False)
    assert f["success"]
    a["speed"] = float(np.linalg.norm(f["states"][0, 3:]))
train = [a for a in arcs if a["part"] == 1]
test = [a for a in arcs if a["part"] == 2]
edge = np.quantile([a["speed"] for a in train], [1/3, 2/3])
group = lambda a: int(np.searchsorted(edge, a["speed"], side="right"))
uniform = fit(train, integrate, base)
assert uniform["success"]
u = {r["id"]: r for r in causal(test, integrate, uniform["params"])}
split = {}
bins = []
for k in range(3):
    tr = [a for a in train if group(a) == k]
    te = [a for a in test if group(a) == k]
    f = fit(tr, integrate, uniform["params"])
    assert f["success"]
    rr = causal(te, integrate, f["params"])
    split.update({r["id"]: r for r in rr})
    bins.append({"bin": k, "train_n": len(tr), "test_n": len(te), "params": f["params"],
                 "unified": summary([u[a["id"]] for a in te]), "split": summary(rr)})
rows = [{"id": a["id"], "t0": float(a["t"][0]), "speed": a["speed"],
         "unified": u[a["id"]]["endpoint_mm"], "split": split[a["id"]]["endpoint_mm"]} for a in test]
result = {"definition": __doc__, "train_n": len(train), "test_n": len(test),
          "speed_edges": edge, "unified_parameters": uniform["params"], "bins": bins,
          "unified": summary(list(u.values())), "split": summary(list(split.values())),
          "paired": paired(rows, "unified", "split"), "rows": rows}
dump(OUT/"speed_forward_check.json", result)
print({k:v for k,v in result.items() if k not in ("rows", "bins")}, flush=True)
