"""
Stage 3 — run the full F1–F8 falsification battery (+ the two adversarial
verifiers and the constants audit) and collect the verdicts.

Each test is a standalone script in falsify/ that writes
<FALSIFICATION>/F*_verdict.json + a PNG. This driver runs them in sequence
(each is minutes-scale) and prints a summary table at the end.

Usage:  python stage3_falsify.py [--only F2,F4]
"""
import argparse
import glob
import json
import os
import subprocess
import sys

import paths

HERE = os.path.dirname(os.path.abspath(__file__))
BATTERY = [
    ("F2", "falsify/f2_km_saturation.py"),
    ("F4", "falsify/f4_paddle_e_vs_un.py"),
    ("F3", "falsify/f3_table_e_vs_vn.py"),
    ("F5", "falsify/f5_slip_regime.py"),
    ("F1", "falsify/f1_kd_over_speed.py"),
    ("F6", "falsify/f6_inverse_magnus.py"),
    ("F7", "falsify/f7_spin_decay.py"),
    ("F8", "falsify/f8_padw_dwell.py"),
    ("F2v", "falsify/f2_verify_polyaccel.py"),   # adversarial re-derivation of F2
    ("F4v", "falsify/f4v_adversarial.py"),       # adversarial re-derivation of F4
    ("F9", "falsify/f9_audit_headline.py"),      # constants audit vs priors
]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--only", default=None, help="comma-separated subset, e.g. F2,F4")
    args = ap.parse_args()
    only = set(args.only.split(",")) if args.only else None

    os.makedirs(paths.FALSIFICATION, exist_ok=True)
    failures = []
    for key, script in BATTERY:
        if only and key not in only:
            continue
        print(f"=== {key}: {script}", flush=True)
        r = subprocess.run([sys.executable, os.path.join(HERE, script)],
                           cwd=HERE, capture_output=True, text=True)
        if r.returncode != 0:
            failures.append(key)
            print(r.stdout[-2000:])
            print(r.stderr[-2000:])

    print("\n===== VERDICT SUMMARY =====")
    for fn in sorted(glob.glob(os.path.join(paths.FALSIFICATION, "F*_verdict.json"))):
        try:
            d = json.load(open(fn))
        except Exception as e:
            print(f"{os.path.basename(fn)}: unreadable ({e})")
            continue
        print(f"{os.path.basename(fn):28s} {d.get('verdict', '?'):14s} "
              f"{(d.get('statistic') or d.get('reason') or '')[:110]}")
    if failures:
        print(f"\nFAILED SCRIPTS: {failures}")
        sys.exit(1)


if __name__ == "__main__":
    main()
