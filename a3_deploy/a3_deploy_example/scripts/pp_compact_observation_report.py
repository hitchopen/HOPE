"""Audit the compact324 CSV without interpreting absolute sent targets as actions.

Physical contact, falls, recovery and the 26-flight verdict belong to the
existing conductor/plant reports. This report checks the actor observation ABI.
"""
import csv
import json
from pathlib import Path
import numpy as np


def report(obs_csv, conductor_json, dimension=324):
    with open(obs_csv) as stream:
        reader = csv.DictReader(stream)
        columns = [f"obs_{i}" for i in range(dimension)]
        if not all(name in reader.fieldnames for name in columns):
            raise ValueError("compact324 CSV does not contain all declared observation columns")
        raw = list(reader)
    observations = np.array([[float(row[key]) for key in columns] for row in raw]).reshape(-1, dimension)
    finite = bool(len(raw) and np.isfinite(observations).all())
    zero_reach = bool(len(raw) and np.all(np.abs(observations[:, 110:112]) < 1e-6))
    # Adjacent past frames must advance in order. A duplicate runner snapshot
    # or a policy-mode entry (all history slots repeat) is explicitly handled.
    history_rows = history_errors = 0
    previous = None
    for row in observations if dimension == 324 else []:
        history = row[114:].reshape(3, 70)
        reset = np.allclose(history, history[0], atol=3e-5, rtol=0.)
        duplicate = previous is not None and np.array_equal(row, previous)
        if previous is not None and not reset and not duplicate:
            history_rows += 1
            old = previous[114:].reshape(3, 70)
            if not np.allclose(history[1:], old[:2], atol=3e-5, rtol=0.):
                history_errors += 1
        previous = row
    checks = {"finite_observations": finite, "disabled_reach_is_zero": zero_reach}
    if dimension == 324:
        checks["history_advances_in_order"] = history_rows > 0 and history_errors == 0
    result = {"report_mode": "compact324" if dimension == 324 else "small_station112", "scope": "observation ABI only; physical verdict is separate",
              "rows": len(raw), "history_rows": history_rows, "history_errors": history_errors,
              "checks": checks, "pass": all(checks.values())}
    path = Path(conductor_json or obs_csv).with_suffix(".compact_observation.json" if dimension == 324 else ".small_station_observation.json")
    path.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))
    return 0 if result["pass"] else 1
