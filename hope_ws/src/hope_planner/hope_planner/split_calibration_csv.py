"""Split a mixed calibration CSV into per-trajectory HOPE calibration CSVs.

This helper is for salvage situations where multiple tosses were recorded into
one file, the position unit is not yet meters, and the table-height zero point
is offset. It:

1. loads one `t,x,y,z` CSV,
2. infers whether the position unit is millimeters,
3. splits the recording on large time gaps, and
4. writes one normalized CSV per chunk with time relative to the chunk start
   and z shifted so the local low-z band is near the table plane.
"""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import numpy as np


@dataclass(frozen=True)
class ChunkSummary:
    chunk_id: int
    rows: int
    duration_s: float
    scale_to_m: float
    z_offset_m: float
    x_min_m: float
    x_max_m: float
    y_min_m: float
    y_max_m: float
    z_min_m: float
    z_max_m: float


def _load_csv(path: Path) -> list[tuple[float, float, float, float]]:
    rows: list[tuple[float, float, float, float]] = []
    with path.open(newline="") as fh:
        reader = csv.reader(fh)
        for row in reader:
            if not row or row[0].strip().lower() in ("t", "time"):
                continue
            rows.append((float(row[0]), float(row[1]), float(row[2]), float(row[3])))
    if not rows:
        raise RuntimeError(f"No numeric rows found in {path}")
    return rows


def _infer_scale(rows: Iterable[tuple[float, float, float, float]]) -> float:
    coords = [abs(v) for _, x, y, z in rows for v in (x, y, z)]
    median_abs = float(np.median(coords))
    return 0.001 if median_abs > 10.0 else 1.0


def _split_rows(
    rows: list[tuple[float, float, float, float]],
    gap_seconds: float,
) -> list[list[tuple[float, float, float, float]]]:
    if len(rows) == 1:
        return [rows]

    chunks: list[list[tuple[float, float, float, float]]] = []
    start = 0
    for i in range(len(rows) - 1):
        if rows[i + 1][0] - rows[i][0] > gap_seconds:
            chunks.append(rows[start : i + 1])
            start = i + 1
    chunks.append(rows[start:])
    return chunks


def _normalize_chunk(
    rows: list[tuple[float, float, float, float]],
    scale_to_m: float,
) -> tuple[np.ndarray, np.ndarray, float]:
    raw = np.asarray(rows, dtype=float)
    t = raw[:, 0]
    pos = raw[:, 1:] * scale_to_m

    z_sorted = np.sort(pos[:, 2])
    low_count = max(3, len(z_sorted) // 10)
    z_offset_m = float(np.mean(z_sorted[:low_count]))
    pos[:, 2] -= z_offset_m

    return t - t[0], pos, z_offset_m


def _summarize_chunk(
    chunk_id: int,
    times: np.ndarray,
    pos: np.ndarray,
    scale_to_m: float,
    z_offset_m: float,
) -> ChunkSummary:
    return ChunkSummary(
        chunk_id=chunk_id,
        rows=len(times),
        duration_s=float(times[-1]) if len(times) > 1 else 0.0,
        scale_to_m=scale_to_m,
        z_offset_m=z_offset_m,
        x_min_m=float(np.min(pos[:, 0])),
        x_max_m=float(np.max(pos[:, 0])),
        y_min_m=float(np.min(pos[:, 1])),
        y_max_m=float(np.max(pos[:, 1])),
        z_min_m=float(np.min(pos[:, 2])),
        z_max_m=float(np.max(pos[:, 2])),
    )


def _write_chunk_csv(path: Path, times: np.ndarray, pos: np.ndarray) -> None:
    with path.open("w", newline="") as fh:
        writer = csv.writer(fh)
        writer.writerow(["t", "x", "y", "z"])
        for t, (x, y, z) in zip(times, pos):
            writer.writerow([f"{t:.9f}", f"{x:.9f}", f"{y:.9f}", f"{z:.9f}"])


def _parse_selected_ids(text: str | None) -> set[int] | None:
    if not text:
        return None
    out: set[int] = set()
    for part in text.split(","):
        stripped = part.strip()
        if not stripped:
            continue
        out.add(int(stripped))
    return out


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(
        description="Split one mixed t,x,y,z calibration CSV into per-trajectory CSVs."
    )
    parser.add_argument("--input", required=True, help="Input CSV path.")
    parser.add_argument("--output-dir", required=True, help="Directory for split CSVs.")
    parser.add_argument(
        "--gap-seconds",
        type=float,
        default=1.0,
        help="Start a new chunk when the timestamp gap exceeds this value.",
    )
    parser.add_argument(
        "--select",
        default="",
        help="Optional comma-separated chunk ids to export. Default: export every chunk.",
    )
    parser.add_argument(
        "--prefix",
        default="traj",
        help="Output file prefix. Example: traj -> traj_chunk02.csv",
    )
    args = parser.parse_args(argv)

    input_path = Path(args.input)
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    rows = _load_csv(input_path)
    scale_to_m = _infer_scale(rows)
    selected_ids = _parse_selected_ids(args.select)

    manifest_rows: list[ChunkSummary] = []
    for chunk_id, chunk in enumerate(_split_rows(rows, args.gap_seconds), start=1):
        times, pos, z_offset_m = _normalize_chunk(chunk, scale_to_m)
        summary = _summarize_chunk(chunk_id, times, pos, scale_to_m, z_offset_m)
        manifest_rows.append(summary)

        if selected_ids is not None and chunk_id not in selected_ids:
            continue

        filename = f"{args.prefix}_chunk{chunk_id:02d}.csv"
        _write_chunk_csv(output_dir / filename, times, pos)

    manifest_path = output_dir / "manifest.csv"
    with manifest_path.open("w", newline="") as fh:
        writer = csv.writer(fh)
        writer.writerow([
            "chunk_id",
            "rows",
            "duration_s",
            "scale_to_m",
            "z_offset_m",
            "x_min_m",
            "x_max_m",
            "y_min_m",
            "y_max_m",
            "z_min_m",
            "z_max_m",
        ])
        for item in manifest_rows:
            writer.writerow([
                item.chunk_id,
                item.rows,
                f"{item.duration_s:.6f}",
                f"{item.scale_to_m:.6f}",
                f"{item.z_offset_m:.6f}",
                f"{item.x_min_m:.6f}",
                f"{item.x_max_m:.6f}",
                f"{item.y_min_m:.6f}",
                f"{item.y_max_m:.6f}",
                f"{item.z_min_m:.6f}",
                f"{item.z_max_m:.6f}",
            ])

    written = len(manifest_rows) if selected_ids is None else len(selected_ids)
    print(f"Input rows: {len(rows)}")
    print(f"Inferred scale_to_m: {scale_to_m}")
    print(f"Detected chunks: {len(manifest_rows)}")
    print(f"Wrote chunk CSVs: {written}")
    print(f"Manifest: {manifest_path}")


if __name__ == "__main__":
    main()
