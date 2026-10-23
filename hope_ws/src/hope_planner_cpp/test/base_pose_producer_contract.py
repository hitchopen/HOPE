#!/usr/bin/env python3
"""Actual Python base producer -> production C++ decoder and turn guard."""

import argparse
import importlib.util
import json
from pathlib import Path
import subprocess
import time
import numpy as np


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--probe", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    args = p.parse_args()
    producer = Path(__file__).parents[2] / "hope_bringup/scripts/base_pose_contract.py"
    spec = importlib.util.spec_from_file_location("actual_base_producer", producer)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    rows = []
    expected = []
    start = time.time_ns()
    for degrees in [0.0, 30.0, -30.0, 50.0]:
        for sign in [1.0, -1.0]:
            q = (
                np.array(
                    [
                        np.cos(np.deg2rad(degrees) / 2),
                        0.0,
                        0.0,
                        np.sin(np.deg2rad(degrees) / 2),
                    ]
                )
                * sign
            )
            ns = start + len(rows)
            wire = module.pose_to_base_flat(
                [-0.43, -0.7, 0.0],
                q,
                [0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
                0.76,
                sequence=len(rows) + 1,
                source_sec=ns // 10**9,
                source_nsec=ns % 10**9,
                tracking_quality=1,
                flags=63,
                calibration_id=1,
                world_frame_id=1,
            )
            rows.append(wire + [0.15, -0.7, 0.35])
            expected.append((degrees, q))
    csv = "\n".join(",".join(map(str, row)) for row in rows) + "\n"
    result = subprocess.run(
        [str(args.probe)], input=csv, text=True, capture_output=True, check=True
    )
    decoded = list(map(json.loads, result.stdout.splitlines()))
    assert len(decoded) == len(rows)
    for wire, (degrees, q), got in zip(rows, expected, decoded):
        assert got["valid"]
        assert np.allclose(got["q"], q, rtol=0, atol=1e-12)
        assert abs(got["yaw_deg"] - degrees) < 1e-10
        assert got["allowed"] == (abs(degrees) <= 40)
        assert np.allclose(
            got["forward"],
            [np.cos(np.deg2rad(degrees)), np.sin(np.deg2rad(degrees)), 0.0],
            atol=1e-12,
            rtol=0,
        )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.with_suffix(".csv").write_text(csv)
    report = {
        "status": "PASS",
        "producer": str(producer),
        "cases": len(rows),
        "yaws": [0, 30, -30, 50],
        "q_sign_aliases": True,
        "decoded": decoded,
        "physical_registration": False,
    }
    args.output.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report))


if __name__ == "__main__":
    main()
