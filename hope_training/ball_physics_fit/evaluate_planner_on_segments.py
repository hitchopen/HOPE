"""Offline HOPE planner accuracy check on canonical ball trajectory segments.

The script replays ``t,x,y,z`` CSV segments into the pure-Python HOPE planner and
compares each predicted hitting-plane crossing with the measured next crossing
of the same ``x_hit`` plane.

It reports prediction error by remaining time-to-crossing. This checks the
deployed estimator + no-spin predictor using real mocap samples, without ROS 2.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[2]
PLANNER_SRC = REPO_ROOT / "hope_ws" / "src" / "hope_planner"
if str(PLANNER_SRC) not in sys.path:
    sys.path.insert(0, str(PLANNER_SRC))

from hope_planner.constants import (  # noqa: E402
    PlannerConfig,
    load_ball_physics,
    load_paddle_params,
    load_table_params,
)
from hope_planner.planner import HOPEPlanner  # noqa: E402


BINS = (
    (0.00, 0.05, "0-50ms"),
    (0.05, 0.10, "50-100ms"),
    (0.10, 0.20, "100-200ms"),
    (0.20, 0.30, "200-300ms"),
    (0.30, 0.50, "300-500ms"),
    (0.50, np.inf, ">500ms"),
)


def _load_yaml(path: Path) -> dict[str, Any]:
    if not path.is_file():
        return {}
    try:
        import yaml
    except ImportError:
        return {}
    with path.open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle)
    return data if isinstance(data, dict) else {}


def _planner_params(path: Path) -> dict[str, Any]:
    data = _load_yaml(path)
    params = data.get("hope_planner", {}).get("ros__parameters", {})
    return params if isinstance(params, dict) else {}


def _make_config(args: argparse.Namespace) -> tuple[Any, PlannerConfig, Any]:
    physics_path = Path(args.physics_path) if args.physics_path else REPO_ROOT / "configs" / "ball_physics.yaml"
    params = _planner_params(Path(args.planner_yaml))

    physics = load_ball_physics(str(physics_path))
    for param_name, attr_name in (
        ("drag_k", "k"),
        ("table_c_h", "C_h"),
        ("table_C_h", "C_h"),
        ("table_c_v", "C_v"),
        ("table_C_v", "C_v"),
    ):
        if params.get(param_name) is not None:
            value = float(params[param_name])
            if value >= 0.0:
                setattr(physics, attr_name, value)
    table_y_max = args.table_y_max
    if table_y_max is None and params.get("table_y_max") is not None:
        table_y_max = float(params["table_y_max"])
    table = load_table_params(str(physics_path), y_max=table_y_max)

    cfg = PlannerConfig()
    for attr in (
        "x_hit",
        "delta_t_flight",
        "max_predict_time",
        "dt_integrate",
        "fit_window",
        "bounce_z_tol",
        "bounce_center_z_max",
    ):
        if params.get(attr) is not None:
            setattr(cfg, attr, type(getattr(cfg, attr))(params[attr]))
    if params.get("target_land_x") is not None and params.get("target_land_y") is not None:
        cfg.target_land = np.array(
            [float(params["target_land_x"]), float(params["target_land_y"]), physics.radius],
            dtype=float,
        )
    if args.x_hit is not None:
        cfg.x_hit = float(args.x_hit)
    if args.fit_window is not None:
        cfg.fit_window = int(args.fit_window)
    for key, value in load_paddle_params(str(physics_path)).items():
        setattr(cfg, key, value)
    return physics, cfg, table


def _load_segment(path: Path) -> tuple[np.ndarray, np.ndarray]:
    raw = np.genfromtxt(path, delimiter=",", names=True)
    t = np.asarray(raw["t"], dtype=float)
    pos = np.column_stack([raw["x"], raw["y"], raw["z"]]).astype(float)
    good = np.isfinite(t) & np.isfinite(pos).all(axis=1)
    t, pos = t[good], pos[good]
    order = np.argsort(t)
    return t[order], pos[order]


def _segment_files(path: Path) -> list[Path]:
    if path.is_file():
        return [path]
    return sorted(p for p in path.glob("*.csv") if p.name != "manifest.csv")


def _manifest_metadata(path: Path) -> dict[str, Any]:
    manifest_path = path.parent / "manifest.json" if path.is_file() else path / "manifest.json"
    if not manifest_path.is_file():
        return {}
    try:
        rows = json.loads(manifest_path.read_text(encoding="utf-8"))
    except json.JSONDecodeError:
        return {"manifest_path": str(manifest_path), "error": "manifest is not valid JSON"}
    if not isinstance(rows, list):
        return {"manifest_path": str(manifest_path), "error": "manifest is not a row list"}

    def unique_values(key: str) -> list[Any]:
        values = []
        for row in rows:
            if isinstance(row, dict) and key in row and row[key] not in values:
                values.append(row[key])
        return values

    return {
        "manifest_path": str(manifest_path),
        "num_manifest_segments": len(rows),
        "frame_preset": unique_values("frame_preset"),
        "axis_map_output_xyz_from_raw": unique_values("axis_map_output_xyz_from_raw"),
        "origin_raw_mm": unique_values("origin_raw_mm"),
    }


def _crossings(t: np.ndarray, pos: np.ndarray, x_hit: float) -> list[dict[str, Any]]:
    events: list[dict[str, Any]] = []
    for i in range(1, len(t)):
        x0 = pos[i - 1, 0]
        x1 = pos[i, 0]
        if x0 > x_hit and x1 <= x_hit:
            denom = x1 - x0
            alpha = (x_hit - x0) / denom if abs(denom) > 1e-12 else 0.5
            alpha = float(np.clip(alpha, 0.0, 1.0))
            p = pos[i - 1] + alpha * (pos[i] - pos[i - 1])
            p[0] = x_hit
            events.append(
                {
                    "index": len(events),
                    "sample_i": i,
                    "t": float(t[i - 1] + alpha * (t[i] - t[i - 1])),
                    "p": p,
                }
            )
    return events


def _next_crossing(events: list[dict[str, Any]], t_now: float, min_horizon: float) -> dict[str, Any] | None:
    for event in events:
        if event["t"] - t_now >= min_horizon:
            return event
    return None


def _near_surface_points(t: np.ndarray, pos: np.ndarray, z_max: float = 0.08) -> np.ndarray:
    if len(pos) < 5:
        return np.empty((0, 3))
    z = pos[:, 2]
    keep = []
    for i in range(2, len(pos) - 2):
        if z[i] <= z_max and z[i] <= z[i - 1] and z[i] <= z[i + 1]:
            keep.append(i)
    return pos[keep] if keep else np.empty((0, 3))


def _percentiles(values: list[float], scale: float = 1e3, suffix: str = "mm") -> dict[str, float | int]:
    if not values:
        return {"n": 0}
    arr = np.asarray(values, dtype=float)
    return {
        "n": int(len(arr)),
        f"median_{suffix}": float(np.median(arr) * scale),
        f"p90_{suffix}": float(np.percentile(arr, 90) * scale),
        f"mean_{suffix}": float(np.mean(arr) * scale),
        f"max_{suffix}": float(np.max(arr) * scale),
    }


def _summarize_predictions(rows: list[dict[str, Any]]) -> dict[str, Any]:
    by_bin: dict[str, dict[str, Any]] = {}
    for lo, hi, label in BINS:
        selected = [r for r in rows if lo <= r["horizon_s"] < hi]
        by_bin[label] = {
            "yz_error": _percentiles([r["err_yz_m"] for r in selected]),
            "y_error": _percentiles([abs(r["err_y_m"]) for r in selected]),
            "z_error": _percentiles([abs(r["err_z_m"]) for r in selected]),
            "time_error": _percentiles([abs(r["err_t_s"]) for r in selected], scale=1e3, suffix="ms"),
        }
    return by_bin


def _diagnose_geometry(
    files: list[Path],
    table: Any,
    physics: Any,
    max_files: int | None = None,
) -> dict[str, Any]:
    mins = []
    maxs = []
    rates = []
    near_surface = []
    for path in files[:max_files]:
        t, pos = _load_segment(path)
        if len(t) < 2:
            continue
        mins.append(np.min(pos, axis=0))
        maxs.append(np.max(pos, axis=0))
        rates.append(1.0 / float(np.median(np.diff(t))))
        near = _near_surface_points(t, pos, z_max=physics.radius + 0.06)
        if len(near):
            near_surface.append(near)
    if not mins:
        return {}

    pmin = np.min(np.vstack(mins), axis=0)
    pmax = np.max(np.vstack(maxs), axis=0)
    table_y_min = table.y_max - table.width
    table_y_max = table.y_max
    diag = {
        "xyz_min_m": [float(v) for v in pmin],
        "xyz_max_m": [float(v) for v in pmax],
        "median_rate_hz": float(np.median(rates)) if rates else 0.0,
        "table_x_range_m": [0.0, float(table.length)],
        "table_y_range_m": [float(table_y_min), float(table_y_max)],
    }
    if near_surface:
        ns = np.vstack(near_surface)
        inside = (
            (ns[:, 0] >= -physics.radius)
            & (ns[:, 0] <= table.length + physics.radius)
            & (ns[:, 1] >= table_y_min - physics.radius)
            & (ns[:, 1] <= table_y_max + physics.radius)
        )
        diag["near_surface_points"] = int(len(ns))
        diag["near_surface_inside_table_fraction"] = float(np.mean(inside))
        diag["near_surface_xyz_min_m"] = [float(v) for v in np.min(ns, axis=0)]
        diag["near_surface_xyz_max_m"] = [float(v) for v in np.max(ns, axis=0)]
    return diag


def _planner_frame_issue(geometry: dict[str, Any]) -> str | None:
    if not geometry.get("near_surface_points", 0):
        return None
    inside = float(geometry.get("near_surface_inside_table_fraction", 1.0))
    if inside >= 0.5:
        return None
    return (
        "Trajectory coordinates do not look like the HOPE planner frame: most "
        "near-table ball-center minima are outside table bounds. For the current "
        "Motive captures, extract planner-validation data with "
        "`--frame-preset hope-planner` (x=rawX, y=rawZ-1.525, z=rawY)."
    )


def evaluate(args: argparse.Namespace) -> dict[str, Any]:
    physics, cfg, table = _make_config(args)
    files = _segment_files(Path(args.segments))
    if args.limit:
        files = files[: args.limit]
    if not files:
        raise SystemExit(f"no segment CSVs found under {args.segments}")

    prediction_rows: list[dict[str, Any]] = []
    segment_rows: list[dict[str, Any]] = []
    total_crossings = 0
    total_valid_strike_predictions = 0

    for path in files:
        t, pos = _load_segment(path)
        if len(t) < max(cfg.fit_window, 6):
            continue
        crossings = _crossings(t, pos, cfg.x_hit)
        total_crossings += len(crossings)
        if not crossings:
            segment_rows.append({"file": path.name, "rows": int(len(t)), "crossings": 0, "predictions": 0})
            continue

        planner = HOPEPlanner(physics=physics, config=cfg, table=table)
        before = len(prediction_rows)
        next_eval_t = -np.inf
        for i, (ti, pi) in enumerate(zip(t, pos)):
            ti = float(ti)
            planner.estimator.push(ti, pi)
            if args.eval_period_s > 0.0 and ti < next_eval_t:
                continue
            next_eval_t = ti + args.eval_period_s
            if not planner.estimator.ready:
                continue

            p_est, v_est, t_est = planner.estimator.estimate()
            if v_est[0] >= 0.0:
                continue
            strike = planner.predictor.predict(p_est, v_est, t_est)
            if not strike.valid:
                continue

            total_valid_strike_predictions += 1
            actual = _next_crossing(crossings, ti, args.min_horizon_ms * 1e-3)
            if actual is None:
                continue
            horizon = float(actual["t"] - ti)
            if horizon > cfg.max_predict_time:
                continue
            err = strike.p_ball - actual["p"]
            prediction_rows.append(
                {
                    "file": path.name,
                    "sample_i": int(i),
                    "crossing_index": int(actual["index"]),
                    "t_sample_s": ti,
                    "horizon_s": horizon,
                    "predicted_horizon_s": float(strike.t_strike - ti),
                    "err_t_s": float(strike.t_strike - actual["t"]),
                    "err_x_m": float(err[0]),
                    "err_y_m": float(err[1]),
                    "err_z_m": float(err[2]),
                    "err_yz_m": float(np.linalg.norm(err[1:3])),
                    "predicted_p": [float(v) for v in strike.p_ball],
                    "actual_p": [float(v) for v in actual["p"]],
                    "num_bounces": int(strike.num_bounces),
                }
            )
        segment_rows.append(
            {
                "file": path.name,
                "rows": int(len(t)),
                "crossings": int(len(crossings)),
                "predictions": int(len(prediction_rows) - before),
            }
        )

    geometry = _diagnose_geometry(files, table, physics)
    manifest_metadata = _manifest_metadata(Path(args.segments))
    fit_window_s = cfg.fit_window / geometry.get("median_rate_hz", 1.0) if geometry else 0.0
    notes = []
    if geometry and fit_window_s < 0.095:
        notes.append(
            f"fit_window={cfg.fit_window} is only {fit_window_s * 1e3:.1f} ms at "
            f"{geometry['median_rate_hz']:.1f} Hz; this dataset validated best near "
            "67 samples for 50-500 ms prediction."
        )
    frame_issue = _planner_frame_issue(geometry)
    if frame_issue:
        notes.append(frame_issue)
    if total_crossings and not prediction_rows:
        notes.append("Measured x_hit crossings exist, but planner produced no comparable valid predictions.")

    result = {
        "segments": str(Path(args.segments).resolve()),
        "num_files": len(files),
        "num_crossings": int(total_crossings),
        "num_valid_strike_predictions": int(total_valid_strike_predictions),
        "num_compared_predictions": int(len(prediction_rows)),
        "config": {
            "x_hit": float(cfg.x_hit),
            "fit_window": int(cfg.fit_window),
            "dt_integrate": float(cfg.dt_integrate),
            "max_predict_time": float(cfg.max_predict_time),
            "bounce_z_tol": float(cfg.bounce_z_tol),
            "bounce_center_z_max": float(cfg.bounce_center_z_max),
            "drag_k": float(physics.k),
            "table_C_h": float(physics.C_h),
            "table_C_v": float(physics.C_v),
            "table_y_max": float(table.y_max),
            "eval_period_s": float(args.eval_period_s),
        },
        "geometry": geometry,
        "segments_manifest": manifest_metadata,
        "planner_frame_geometry_ok": frame_issue is None,
        "notes": notes,
        "error_by_horizon": _summarize_predictions(prediction_rows),
        "segments_summary": segment_rows,
        "predictions": prediction_rows,
    }

    out_path = Path(args.out_json)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(json.dumps(result, indent=1), encoding="utf-8")

    printable = {k: v for k, v in result.items() if k not in {"segments_summary", "predictions"}}
    print(json.dumps(printable, indent=1))
    print(f"-> {out_path}")
    if frame_issue and not args.allow_bad_geometry:
        raise SystemExit(
            "bad planner-frame geometry; use --allow-bad-geometry only for intentional diagnostics"
        )
    return result


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("segments", help="directory of canonical t,x,y,z segment CSVs, or one CSV")
    parser.add_argument(
        "--planner-yaml",
        default=str(REPO_ROOT / "hope_ws" / "src" / "hope_planner" / "config" / "hope_planner.yaml"),
    )
    parser.add_argument("--physics-path", default=str(REPO_ROOT / "configs" / "ball_physics.yaml"))
    parser.add_argument("--out-json", default=str(REPO_ROOT / "analysis" / "planner_eval.json"))
    parser.add_argument("--x-hit", type=float, default=None)
    parser.add_argument("--fit-window", type=int, default=None)
    parser.add_argument("--table-y-max", type=float, default=None)
    parser.add_argument("--min-horizon-ms", type=float, default=20.0)
    parser.add_argument(
        "--allow-bad-geometry",
        action="store_true",
        help="do not fail when near-table points fall outside planner table bounds",
    )
    parser.add_argument(
        "--eval-period-s",
        type=float,
        default=0.02,
        help="run prediction at this period while still feeding every sample to the estimator; 0 evaluates every sample",
    )
    parser.add_argument("--limit", type=int, default=None, help="limit number of input CSVs")
    return parser


def main() -> None:
    evaluate(build_arg_parser().parse_args())


if __name__ == "__main__":
    main()
