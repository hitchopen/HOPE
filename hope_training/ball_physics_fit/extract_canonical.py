"""Raw capture -> canonical ball-trajectory CSVs.

Takes a raw motion-capture CSV of the ball centroid and emits one canonical
``t,x,y,z`` CSV per contiguous capture segment, in SI units and the world frame
used everywhere else (+x forward, +y left, +z up; z = 0 at the table surface).

The raw input is a CSV with a ``t,x,y,z`` header where:
  * ``t`` may be absolute (epoch) seconds -- it is rebased to start at 0;
  * ``x,y,z`` may be in millimetres -- auto-detected and scaled to metres
    (override with --scale).

The capture is split into segments wherever the sample interval jumps (tracking
gaps / separate rallies); each segment long enough to be useful is written out.
Optionally shift z so the table surface sits at z = 0 (--z-offset).

Usage:
    python extract_canonical.py RAW.csv OUT_DIR
        [--scale 0.001] [--z-offset 0.0] [--gap-s 0.10] [--min-rows 15]
"""
import argparse
import os

import numpy as np


def load_raw(path):
    raw = np.genfromtxt(path, delimiter=",", names=True)
    t = np.asarray(raw["t"], float)
    pos = np.column_stack([raw["x"], raw["y"], raw["z"]]).astype(float)
    order = np.argsort(t)
    return t[order], pos[order]


def guess_scale(pos):
    """1e-3 if the coordinates look like millimetres, else 1.0."""
    med = float(np.nanmedian(np.abs(pos)))
    return 1e-3 if med > 20.0 else 1.0


def split_segments(t, gap_s, min_rows):
    """Index ranges [a, b] with no internal time gap larger than ``gap_s``."""
    segs, s = [], 0
    for i in range(1, len(t)):
        if t[i] - t[i - 1] > gap_s:
            segs.append((s, i - 1)); s = i
    segs.append((s, len(t) - 1))
    return [(a, b) for a, b in segs if b - a + 1 >= min_rows]


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("raw_csv")
    ap.add_argument("out_dir")
    ap.add_argument("--scale", type=float, default=None,
                    help="metres per raw unit (default: auto — 0.001 if mm-like)")
    ap.add_argument("--z-offset", type=float, default=0.0,
                    help="metres subtracted from z so the table surface is z=0")
    ap.add_argument("--gap-s", type=float, default=0.10,
                    help="time gap (s) that starts a new segment")
    ap.add_argument("--min-rows", type=int, default=15,
                    help="drop segments shorter than this")
    args = ap.parse_args()

    t, pos = load_raw(args.raw_csv)
    scale = args.scale if args.scale is not None else guess_scale(pos)
    pos = pos * scale
    pos[:, 2] -= args.z_offset
    t = t - t[0]

    os.makedirs(args.out_dir, exist_ok=True)
    stem = os.path.splitext(os.path.basename(args.raw_csv))[0]
    segs = split_segments(t, args.gap_s, args.min_rows)
    if not segs:
        raise SystemExit(f"no segment >= {args.min_rows} rows in {args.raw_csv}")

    written = []
    for k, (a, b) in enumerate(segs):
        ts = t[a:b + 1] - t[a]              # each segment starts at 0
        seg = np.column_stack([ts, pos[a:b + 1]])
        out = os.path.join(args.out_dir, f"{stem}_seg{k:02d}.csv")
        np.savetxt(out, seg, delimiter=",", header="t,x,y,z", comments="",
                   fmt="%.9f")
        written.append((out, b - a + 1, float(ts[-1])))

    print(f"scale={scale} m/unit, z_offset={args.z_offset} m, "
          f"{len(written)} segment(s):")
    for out, n, dur in written:
        print(f"  {os.path.basename(out)}: {n} rows, {dur:.2f} s")


if __name__ == "__main__":
    main()
