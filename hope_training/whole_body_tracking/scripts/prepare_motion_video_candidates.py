"""Download candidate table-tennis videos and generate review contact sheets.

This is the first, low-cost stage of the motion-data pipeline. It intentionally stops before
GVHMR/GMR retargeting so a human can reject clips with occluded feet, multiple people, bad crops,
or unclear contact timing.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import re
import shutil
import subprocess
import tempfile
from pathlib import Path
from typing import Any

from PIL import Image, ImageDraw


VIDEO_EXTS = {".mp4", ".mkv", ".webm", ".flv", ".mov", ".avi"}


def repo_root() -> Path:
    here = Path(__file__).resolve()
    for parent in here.parents:
        if (parent / "hope_training").is_dir():
            return parent
    raise RuntimeError(f"could not find repo root from {here}")


def sanitize_slug(value: str) -> str:
    slug = re.sub(r"[^A-Za-z0-9_.-]+", "_", value.strip())
    slug = slug.strip("._-")
    if not slug:
        raise ValueError(f"invalid empty slug from {value!r}")
    return slug


def run_logged(cmd: list[str], log_path: Path, cwd: Path | None = None) -> int:
    log_path.parent.mkdir(parents=True, exist_ok=True)
    with log_path.open("w", encoding="utf-8") as log:
        log.write("$ " + " ".join(cmd) + "\n\n")
        proc = subprocess.run(cmd, cwd=cwd, stdout=log, stderr=subprocess.STDOUT, text=True)
    return proc.returncode


def read_candidates(path: Path) -> list[dict[str, str]]:
    with path.open("r", encoding="utf-8", newline="") as fh:
        return list(csv.DictReader(fh, delimiter="\t"))


def find_downloaded_video(raw_dir: Path, slug: str) -> Path | None:
    matches = []
    for p in raw_dir.glob(f"{slug}.*"):
        if p.suffix.lower() in VIDEO_EXTS and not p.name.endswith(".part"):
            matches.append(p)
    if not matches:
        return None
    return sorted(matches, key=lambda p: p.stat().st_size, reverse=True)[0]


def download_video(row: dict[str, str], raw_dir: Path, log_dir: Path, force: bool) -> tuple[str, Path | None]:
    slug = sanitize_slug(row["slug"])
    existing = find_downloaded_video(raw_dir, slug)
    if existing is not None and not force:
        return "exists", existing

    for p in raw_dir.glob(f"{slug}.*"):
        if p.suffix.lower() in VIDEO_EXTS or p.name.endswith(".part") or p.name.endswith(".info.json"):
            p.unlink()

    out_tpl = str(raw_dir / f"{slug}.%(ext)s")
    cmd = [
        "yt-dlp",
        "--no-playlist",
        "--merge-output-format",
        "mp4",
        "-f",
        "bv*[height<=720]+ba/b[height<=720]/best",
        "--write-info-json",
        "-o",
        out_tpl,
        row["url"],
    ]
    rc = run_logged(cmd, log_dir / f"{slug}.download.log")
    video = find_downloaded_video(raw_dir, slug)
    if rc != 0 or video is None:
        return "download_failed", video
    return "downloaded", video


def probe_video(video: Path) -> dict[str, Any]:
    cmd = [
        "ffprobe",
        "-v",
        "error",
        "-select_streams",
        "v:0",
        "-show_entries",
        "stream=width,height,r_frame_rate,avg_frame_rate,duration,nb_frames",
        "-of",
        "json",
        str(video),
    ]
    raw = subprocess.check_output(cmd, text=True)
    stream = json.loads(raw)["streams"][0]

    def rate(value: str) -> float:
        if not value or "/" not in value:
            return 0.0
        num, den = value.split("/", 1)
        den_f = float(den)
        return float(num) / den_f if den_f else 0.0

    fps = rate(stream.get("avg_frame_rate") or stream.get("r_frame_rate") or "0/1")
    duration = float(stream.get("duration") or 0.0)
    if duration <= 0.0 and stream.get("nb_frames") and fps > 0.0:
        duration = float(stream["nb_frames"]) / fps
    return {
        "width": int(stream.get("width") or 0),
        "height": int(stream.get("height") or 0),
        "fps": fps,
        "duration_s": duration,
        "nb_frames": stream.get("nb_frames") or "",
    }


def extract_frame(video: Path, t: float, out: Path, width: int) -> bool:
    cmd = [
        "ffmpeg",
        "-hide_banner",
        "-loglevel",
        "error",
        "-ss",
        f"{t:.3f}",
        "-i",
        str(video),
        "-frames:v",
        "1",
        "-vf",
        f"scale={width}:-1",
        "-y",
        str(out),
    ]
    return subprocess.run(cmd).returncode == 0 and out.is_file()


def make_contact_sheet(video: Path, sheet: Path, duration_s: float, frames: int, cols: int, cell_width: int) -> bool:
    if duration_s <= 0.0:
        return False
    rows = int(math.ceil(frames / cols))
    times = [(duration_s * (i + 0.5) / frames) for i in range(frames)]

    with tempfile.TemporaryDirectory(prefix="hope_sheet_") as tmp_s:
        tmp = Path(tmp_s)
        frame_paths: list[Path] = []
        for i, t in enumerate(times):
            out = tmp / f"{i:03d}.jpg"
            if extract_frame(video, t, out, cell_width):
                frame_paths.append(out)
        if not frame_paths:
            return False

        images = [Image.open(p).convert("RGB") for p in frame_paths]
        cell_h = max(img.height for img in images) + 22
        canvas = Image.new("RGB", (cols * cell_width, rows * cell_h), "white")
        draw = ImageDraw.Draw(canvas)
        for i, img in enumerate(images):
            x = (i % cols) * cell_width
            y = (i // cols) * cell_h
            canvas.paste(img, (x, y))
            draw.rectangle((x, y + img.height, x + cell_width, y + cell_h), fill="white")
            draw.text((x + 4, y + img.height + 4), f"{times[i]:.2f}s", fill="black")
        sheet.parent.mkdir(parents=True, exist_ok=True)
        canvas.save(sheet, quality=92)
    return True


def write_manifest(path: Path, rows: list[dict[str, Any]]) -> None:
    fields = [
        "slug",
        "status",
        "priority",
        "action",
        "title",
        "uploader",
        "url",
        "video_path",
        "sheet_path",
        "duration_s",
        "fps",
        "width",
        "height",
        "notes",
        "manual_decision",
        "clip_start_s",
        "clip_end_s",
        "strike_frame",
        "review_notes",
    ]
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as fh:
        writer = csv.DictWriter(fh, fieldnames=fields, delimiter="\t")
        writer.writeheader()
        for row in rows:
            writer.writerow({key: row.get(key, "") for key in fields})


def parse_args() -> argparse.Namespace:
    root = repo_root()
    default_run_dir = root / "data" / "motion_pipeline" / "backhand_hunt_20260721"
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--candidates", type=Path, default=default_run_dir / "candidates.tsv")
    parser.add_argument("--run-dir", type=Path, default=default_run_dir)
    parser.add_argument("--force-download", action="store_true")
    parser.add_argument("--sheet-frames", type=int, default=24)
    parser.add_argument("--sheet-cols", type=int, default=6)
    parser.add_argument("--cell-width", type=int, default=320)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    candidates = read_candidates(args.candidates)
    run_dir = args.run_dir.resolve()
    raw_dir = run_dir / "raw"
    sheet_dir = run_dir / "sheets"
    log_dir = run_dir / "logs"
    metadata_dir = run_dir / "metadata"
    for d in (raw_dir, sheet_dir, log_dir, metadata_dir):
        d.mkdir(parents=True, exist_ok=True)

    if shutil.which("yt-dlp") is None:
        raise RuntimeError("yt-dlp not found")
    if shutil.which("ffmpeg") is None or shutil.which("ffprobe") is None:
        raise RuntimeError("ffmpeg/ffprobe not found")

    review_rows: list[dict[str, Any]] = []
    for row in candidates:
        slug = sanitize_slug(row["slug"])
        status, video = download_video(row, raw_dir, log_dir, args.force_download)
        out_row: dict[str, Any] = {
            "slug": slug,
            "status": status,
            "priority": row.get("priority", ""),
            "action": row.get("action", ""),
            "title": row.get("title", ""),
            "uploader": row.get("uploader", ""),
            "url": row.get("url", ""),
            "notes": row.get("notes", ""),
        }
        if video is not None:
            out_row["video_path"] = str(video)
            try:
                probe = probe_video(video)
                out_row.update(
                    duration_s=f"{probe['duration_s']:.3f}",
                    fps=f"{probe['fps']:.3f}",
                    width=probe["width"],
                    height=probe["height"],
                )
                sheet = sheet_dir / f"{slug}_sheet.jpg"
                if make_contact_sheet(
                    video,
                    sheet,
                    probe["duration_s"],
                    args.sheet_frames,
                    args.sheet_cols,
                    args.cell_width,
                ):
                    out_row["sheet_path"] = str(sheet)
                else:
                    out_row["status"] = f"{status}+sheet_failed"
            except Exception as exc:
                out_row["status"] = f"{status}+probe_or_sheet_failed"
                (log_dir / f"{slug}.probe_sheet.err").write_text(str(exc), encoding="utf-8")
        review_rows.append(out_row)
        print(f"{slug}\t{out_row['status']}\t{out_row.get('duration_s', '')}\t{out_row.get('sheet_path', '')}")

    manifest = metadata_dir / "review_manifest.tsv"
    write_manifest(manifest, review_rows)
    print(f"review_manifest={manifest}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
