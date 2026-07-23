"""Extract Motive rigid-body ``A`` ball trajectories.

This is a raw Motive CSV/ZIP front-end for the canonical HOPE ball trajectory
format used by the rest of ``ball_physics_fit``:

    t,x,y,z

Output coordinates are SI metres. For planner validation, the default
``--frame-preset hope-planner`` writes the HOPE play frame used by
``hope_planner.constants``:

    planner x = raw Motive X
    planner y = raw Motive Z - 1525 mm
    planner z = raw Motive Y

That places the table at ``x in [0, length]``, ``y in [-width, 0]`` and the
table surface at ``z ~= 0`` for this capture. The raw origin can be shifted
before remapping with ``--origin-raw-mm X Y Z``.

Examples:

    python extract_motive_a_segments.py /path/拍1动捕数据.zip /path/拍2动捕数据.zip \
        --out-dir data/motive_a_planner_frame

    # Reproduce the originally requested axis shuffle for comparison only:
    python extract_motive_a_segments.py raw.zip --frame-preset requested-remap \
        --out-dir data/motive_a_requested
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import zipfile
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Iterator, Sequence

import numpy as np


AXIS_INDEX = {"x": 0, "y": 1, "z": 2}

FRAME_PRESETS = {
    "hope-planner": {
        "axis_map": "x,z,y",
        "origin_raw_mm": (0.0, 0.0, 1525.0),
        "description": "HOPE planner frame: x=rawX, y=rawZ-table_width, z=rawY",
    },
    "requested-remap": {
        "axis_map": "z,x,y",
        "origin_raw_mm": (0.0, 0.0, 0.0),
        "description": "original requested shuffle: x=rawZ, y=rawX, z=rawY",
    },
}


@dataclass(frozen=True)
class Capture:
    source: str
    entry: str
    frame_rate: float
    units: str
    t: np.ndarray
    raw_pos: np.ndarray


def _decode_lines(handle: Iterable[bytes]) -> Iterator[str]:
    for line in handle:
        yield line.decode("utf-8-sig", errors="replace")


def _metadata_value(row: Sequence[str], key: str, default: str = "") -> str:
    for i in range(0, len(row) - 1, 2):
        if row[i] == key:
            return row[i + 1]
    return default


def _find_a_position_column(
    types: Sequence[str],
    names: Sequence[str],
    categories: Sequence[str],
    axes: Sequence[str],
) -> int:
    """Return the first column of rigid-body A Position X,Y,Z."""
    for i in range(2, len(axes) - 2):
        if (
            types[i] == "Rigid Body"
            and names[i] == "A"
            and categories[i] == "Position"
            and axes[i : i + 3] == ["X", "Y", "Z"]
        ):
            return i
    raise ValueError("could not find rigid-body A Position X,Y,Z columns")


def _load_motive_csv(source: str, entry: str, lines: Iterable[str]) -> Capture:
    reader = csv.reader(lines)
    meta = next(reader)
    # Motive exports an empty spacer row before the multi-row header.
    spacer = next(reader)
    while spacer:
        spacer = next(reader)

    types = next(reader)
    names = next(reader)
    _ids = next(reader)
    categories = next(reader)
    axes = next(reader)
    pos_col = _find_a_position_column(types, names, categories, axes)

    ts: list[float] = []
    pts: list[list[float]] = []
    for row in reader:
        if len(row) <= pos_col + 2:
            continue
        try:
            t = float(row[1])
            p = [float(row[pos_col]), float(row[pos_col + 1]), float(row[pos_col + 2])]
        except (TypeError, ValueError):
            continue
        if math.isfinite(t) and all(math.isfinite(v) for v in p):
            ts.append(t)
            pts.append(p)

    if not ts:
        raise ValueError("no valid rigid-body A positions")

    order = np.argsort(ts)
    return Capture(
        source=source,
        entry=entry,
        frame_rate=float(_metadata_value(meta, "Export Frame Rate", "0") or 0.0),
        units=_metadata_value(meta, "Length Units", ""),
        t=np.asarray(ts, dtype=float)[order],
        raw_pos=np.asarray(pts, dtype=float)[order],
    )


def _iter_input_captures(paths: Sequence[Path]) -> Iterator[Capture]:
    for path in paths:
        if path.suffix.lower() == ".zip":
            with zipfile.ZipFile(path) as zf:
                for entry in sorted(n for n in zf.namelist() if n.lower().endswith(".csv")):
                    with zf.open(entry) as handle:
                        yield _load_motive_csv(str(path), entry, _decode_lines(handle))
        else:
            with path.open("r", encoding="utf-8-sig", errors="replace", newline="") as handle:
                yield _load_motive_csv(str(path), path.name, handle)


def _unit_scale(units: str, override: float | None) -> float:
    if override is not None:
        return override
    normalized = units.strip().lower()
    if normalized.startswith("milli"):
        return 1e-3
    if normalized in {"meter", "meters", "metre", "metres", "m"}:
        return 1.0
    # Motive files in this dataset say Millimeters. This fallback keeps the
    # script usable on header-light exports.
    return 1e-3


def _parse_axis_map(spec: str) -> tuple[int, int, int]:
    labels = [p.strip().lower() for p in spec.split(",")]
    if len(labels) != 3 or any(label not in AXIS_INDEX for label in labels):
        raise argparse.ArgumentTypeError("--axis-map must look like z,x,y")
    if len(set(labels)) != 3:
        raise argparse.ArgumentTypeError("--axis-map must use each raw axis once")
    return tuple(AXIS_INDEX[label] for label in labels)


def _resolve_transform(args: argparse.Namespace) -> tuple[str, tuple[int, int, int], np.ndarray]:
    preset = FRAME_PRESETS[args.frame_preset]
    axis_map_spec = args.axis_map if args.axis_map is not None else preset["axis_map"]
    origin_raw_mm = args.origin_raw_mm if args.origin_raw_mm is not None else preset["origin_raw_mm"]
    return axis_map_spec, _parse_axis_map(axis_map_spec), np.asarray(origin_raw_mm, dtype=float)


def _split_gap_runs(t: np.ndarray, gap_s: float, min_rows: int) -> list[tuple[int, int]]:
    if len(t) < min_rows:
        return []
    runs: list[tuple[int, int]] = []
    start = 0
    for i in range(1, len(t)):
        if t[i] - t[i - 1] > gap_s:
            runs.append((start, i))
            start = i
    runs.append((start, len(t)))
    return [(a, b) for a, b in runs if b - a >= min_rows]


def _contact_segments(
    stem: str,
    t: np.ndarray,
    pos: np.ndarray,
    min_rows: int,
) -> list[dict]:
    """Use ballcore contact detection to split a capture into free-flight arcs."""
    from ballcore import extract_arcs

    dt = np.median(np.diff(t)) if len(t) > 1 else 0.0
    traj = {
        "t": t,
        "pos": pos,
        "rate": 1.0 / dt if dt > 0 else 0.0,
        "name": stem,
    }
    out = []
    for arc in extract_arcs(traj, min_frames=min_rows, table_z=0.0):
        out.append(
            {
                "i0": int(arc["i0"]),
                "i1": int(arc["i1"]),
                "t": arc["t"],
                "pos": arc["pos"],
                "pre_contact": arc["pre_contact"],
                "post_contact": arc["post_contact"],
            }
        )
    return out


def _safe_stem(capture: Capture) -> str:
    return Path(capture.entry).stem.replace(" ", "_")


def _write_segment(
    out_dir: Path,
    stem: str,
    seg_idx: int,
    t_abs: np.ndarray,
    pos: np.ndarray,
    metadata: dict,
) -> dict:
    t = t_abs - t_abs[0]
    data = np.column_stack([t, pos])
    filename = f"{stem}_seg{seg_idx:03d}.csv"
    path = out_dir / filename
    np.savetxt(path, data, delimiter=",", header="t,x,y,z", comments="", fmt="%.9f")

    row = {
        **metadata,
        "file": filename,
        "rows": int(len(t)),
        "duration_s": float(t[-1]) if len(t) else 0.0,
        "t_start_s": float(t_abs[0]),
        "t_end_s": float(t_abs[-1]),
        "min_xyz_m": [float(v) for v in np.min(pos, axis=0)],
        "max_xyz_m": [float(v) for v in np.max(pos, axis=0)],
        "median_xyz_m": [float(v) for v in np.median(pos, axis=0)],
    }
    return row


def extract(args: argparse.Namespace) -> list[dict]:
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    axis_map_spec, axis_map, raw_origin = _resolve_transform(args)

    manifest = []
    for capture in _iter_input_captures([Path(p) for p in args.inputs]):
        scale = _unit_scale(capture.units, args.scale)
        raw_shifted = capture.raw_pos - raw_origin
        pos = raw_shifted[:, axis_map] * scale

        # Rebase file time before segmentation; per-segment files are rebased
        # again to start at zero.
        t = capture.t - capture.t[0]
        stem = _safe_stem(capture)
        base_meta = {
            "source": capture.source,
            "entry": capture.entry,
            "frame_rate_hz": capture.frame_rate,
            "units": capture.units,
            "scale_m_per_unit": scale,
            "frame_preset": args.frame_preset,
            "frame_preset_description": FRAME_PRESETS[args.frame_preset]["description"],
            "axis_map_output_xyz_from_raw": axis_map_spec,
            "origin_raw_mm": [float(v) for v in raw_origin],
            "split_mode": args.split_mode,
        }

        if args.split_mode == "gap":
            segments = [
                {
                    "i0": a,
                    "i1": b,
                    "t": t[a:b],
                    "pos": pos[a:b],
                    "pre_contact": None,
                    "post_contact": None,
                }
                for a, b in _split_gap_runs(t, args.gap_s, args.min_rows)
            ]
        else:
            segments = _contact_segments(stem, t, pos, args.min_rows)

        written = 0
        for seg_idx, seg in enumerate(segments):
            if len(seg["t"]) < args.min_rows:
                continue
            row = _write_segment(
                out_dir,
                stem,
                seg_idx,
                seg["t"],
                seg["pos"],
                {
                    **base_meta,
                    "source_i0": int(seg["i0"]),
                    "source_i1": int(seg["i1"]),
                    "pre_contact": seg["pre_contact"],
                    "post_contact": seg["post_contact"],
                },
            )
            manifest.append(row)
            written += 1
        print(f"{capture.entry}: {len(capture.t)} A samples -> {written} segment(s)")

    manifest_path = out_dir / "manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=1, ensure_ascii=False), encoding="utf-8")
    print(f"\nWrote {len(manifest)} segment(s) -> {out_dir}")
    print(f"Manifest: {manifest_path}")
    return manifest


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("inputs", nargs="+", help="Motive CSV or ZIP files")
    parser.add_argument("--out-dir", required=True, help="directory for canonical segment CSVs")
    parser.add_argument(
        "--frame-preset",
        choices=sorted(FRAME_PRESETS),
        default="hope-planner",
        help=(
            "coordinate preset. Default hope-planner produces the HOPE planner play frame; "
            "requested-remap reproduces the original raw z,x,y shuffle for comparison"
        ),
    )
    parser.add_argument(
        "--axis-map",
        default=None,
        help=(
            "override output x,y,z source raw axes, comma-separated, e.g. x,z,y. "
            "Defaults come from --frame-preset"
        ),
    )
    parser.add_argument(
        "--origin-raw-mm",
        nargs=3,
        type=float,
        default=None,
        metavar=("X", "Y", "Z"),
        help="override raw Motive origin offset in millimetres, subtracted before axis remap",
    )
    parser.add_argument("--scale", type=float, default=None, help="metres per raw unit")
    parser.add_argument(
        "--split-mode",
        choices=("contact", "gap"),
        default="contact",
        help="contact: free-flight arcs split at detected contacts; gap: contiguous tracked runs",
    )
    parser.add_argument("--gap-s", type=float, default=0.05, help="gap threshold for split-mode=gap")
    parser.add_argument("--min-rows", type=int, default=20, help="drop shorter segments")
    return parser


def main() -> None:
    extract(build_arg_parser().parse_args())


if __name__ == "__main__":
    main()
