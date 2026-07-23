"""Continuous planner-physics optimization on real ball trajectories.

This script treats the no-spin planner physics terms as a constrained
optimization problem:

    minimize  median(||e_yz||) + alpha * p90(||e_yz||) + invalid penalties

where each error compares the planner-predicted hitting-plane crossing with the
next measured crossing from held-out Motive trajectories. The estimator settings
come from ``hope_planner.yaml`` by default; only the continuous physics terms are
optimized:

* ``drag_k``: quadratic drag acceleration coefficient;
* ``table_C_h``: table tangential velocity retention;
* ``table_C_v``: table normal restitution.

Unlike the coarse grid search, this script splits files into train/validation
sets so a candidate must be checked on trajectories it did not optimize against.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any

import numpy as np


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from evaluate_planner_on_segments import (  # noqa: E402
    REPO_ROOT,
    _diagnose_geometry,
    _make_config,
    _manifest_metadata,
    _planner_frame_issue,
    _segment_files,
)
from optimize_planner_params_on_segments import (  # noqa: E402
    _clone_physics,
    _collect_cases,
    _evaluate_cases,
    _stratified_sample,
)

PLANNER_SRC = REPO_ROOT / "hope_ws" / "src" / "hope_planner"
if str(PLANNER_SRC) not in sys.path:
    sys.path.insert(0, str(PLANNER_SRC))

from hope_planner.constants import load_ball_physics  # noqa: E402


def _parse_bounds(text: str) -> list[tuple[float, float]]:
    bounds = []
    for part in text.split(","):
        lo_hi = [float(v.strip()) for v in part.split(":") if v.strip()]
        if len(lo_hi) != 2 or lo_hi[0] > lo_hi[1]:
            raise argparse.ArgumentTypeError(
                "--bounds must look like k_lo:k_hi,ch_lo:ch_hi,cv_lo:cv_hi"
            )
        bounds.append((lo_hi[0], lo_hi[1]))
    if len(bounds) != 3:
        raise argparse.ArgumentTypeError(
            "--bounds must provide exactly three ranges: drag_k,table_C_h,table_C_v"
        )
    return bounds


def _candidate_dict(x: np.ndarray, metrics: dict[str, Any] | None = None) -> dict[str, Any]:
    row = {
        "drag_k": float(x[0]),
        "table_C_h": float(x[1]),
        "table_C_v": float(x[2]),
    }
    if metrics is not None:
        row.update(
            {
                "objective": float(metrics["objective"]),
                "valid_rate": float(metrics["valid_rate"]),
                "num_valid": int(metrics["num_valid"]),
                "num_cases": int(metrics["num_cases"]),
                "summary": metrics["summary"],
            }
        )
    return row


def _evaluate_x(x: np.ndarray, cases: list[Any], base_physics: Any, cfg: Any, table: Any, args: argparse.Namespace):
    physics = _clone_physics(base_physics, x[0], x[1], x[2])
    metrics = _evaluate_cases(cases, physics, cfg, table, invalid_penalty_m=args.invalid_penalty_m)
    return metrics


def _objective(
    x: np.ndarray,
    cases: list[Any],
    base_physics: Any,
    cfg: Any,
    table: Any,
    args: argparse.Namespace,
) -> float:
    metrics = _evaluate_x(x, cases, base_physics, cfg, table, args)
    return float(metrics["objective"])


def _split_files(files: list[Path], train_fraction: float, seed: int) -> tuple[list[Path], list[Path]]:
    rng = np.random.default_rng(seed)
    order = np.arange(len(files))
    rng.shuffle(order)
    n_train = int(round(len(files) * train_fraction))
    n_train = min(max(n_train, 1), max(len(files) - 1, 1))
    train = [files[i] for i in sorted(order[:n_train])]
    val = [files[i] for i in sorted(order[n_train:])]
    return train, val


def _initial_points(bounds: list[tuple[float, float]], base_physics: Any, args: argparse.Namespace) -> list[np.ndarray]:
    rng = np.random.default_rng(args.seed)
    points = [
        np.array([base_physics.k, base_physics.C_h, base_physics.C_v], dtype=float),
        np.array([0.1261, 0.631, 0.9215], dtype=float),
        np.array([0.1261, 0.70, 0.9215], dtype=float),
        np.array([0.15, 0.75, 0.95], dtype=float),
        np.array([0.18, 0.80, 0.95], dtype=float),
    ]
    lows = np.array([b[0] for b in bounds], dtype=float)
    highs = np.array([b[1] for b in bounds], dtype=float)
    for _ in range(args.random_candidates):
        points.append(rng.uniform(lows, highs))
    clipped = []
    seen = set()
    for point in points:
        x = np.clip(point, lows, highs)
        key = tuple(np.round(x, 6))
        if key not in seen:
            clipped.append(x)
            seen.add(key)
    return clipped


def _minimize_from_starts(
    starts: list[np.ndarray],
    bounds: list[tuple[float, float]],
    train_cases: list[Any],
    base_physics: Any,
    cfg: Any,
    table: Any,
    args: argparse.Namespace,
) -> list[dict[str, Any]]:
    try:
        from scipy.optimize import minimize
    except ImportError:
        return []

    out = []
    for i, x0 in enumerate(starts[: args.num_starts], start=1):
        print(f"continuous start {i}/{min(args.num_starts, len(starts))}: {x0.tolist()}")
        result = minimize(
            _objective,
            x0,
            args=(train_cases, base_physics, cfg, table, args),
            method="Powell",
            bounds=bounds,
            options={"maxfev": args.max_evals_per_start, "xtol": 1e-3, "ftol": 1e-3, "disp": False},
        )
        metrics = _evaluate_x(np.asarray(result.x, dtype=float), train_cases, base_physics, cfg, table, args)
        out.append(
            {
                "source": "powell",
                "success": bool(result.success),
                "message": str(result.message),
                "nfev": int(result.nfev),
                **_candidate_dict(np.asarray(result.x, dtype=float), metrics),
            }
        )
        print(f"  -> objective={metrics['objective'] * 1e3:.1f} mm, x={result.x}")
    return out


def optimize(args: argparse.Namespace) -> dict[str, Any]:
    planner_physics, cfg, table = _make_config(args)
    shared_physics = load_ball_physics(args.physics_path)
    base_physics = planner_physics if args.physics_source == "planner-yaml" else shared_physics

    files = _segment_files(Path(args.segments))
    if args.limit:
        files = files[: args.limit]
    if not files:
        raise SystemExit(f"no segment CSVs found under {args.segments}")
    geometry = _diagnose_geometry(files, table, shared_physics)
    frame_issue = _planner_frame_issue(geometry)
    if frame_issue and not args.allow_bad_geometry:
        raise SystemExit(
            f"{frame_issue} Use --allow-bad-geometry only for intentional diagnostics."
        )

    if args.bounce_center_z_max is not None:
        cfg.bounce_center_z_max = float(args.bounce_center_z_max)
    if args.fit_window is not None:
        cfg.fit_window = int(args.fit_window)

    train_files, val_files = _split_files(files, args.train_fraction, args.seed)
    min_h = args.min_horizon_ms * 1e-3
    max_h = args.max_horizon_ms * 1e-3
    cfg.max_predict_time = max(float(cfg.max_predict_time), max_h)
    train_cases_full = _collect_cases(train_files, cfg, args.eval_period_s, min_h, max_h)
    val_cases = _collect_cases(val_files, cfg, args.eval_period_s, min_h, max_h)
    train_cases = _stratified_sample(train_cases_full, args.train_sample_cases, args.seed)
    if not train_cases or not val_cases:
        raise SystemExit(
            f"not enough cases after split: train={len(train_cases)}, val={len(val_cases)}"
        )

    bounds = _parse_bounds(args.bounds)
    starts = _initial_points(bounds, base_physics, args)
    seed_rows = []
    for point in starts:
        metrics = _evaluate_x(point, train_cases, base_physics, cfg, table, args)
        seed_rows.append({"source": "seed", **_candidate_dict(point, metrics)})
    seed_rows.sort(key=lambda r: r["objective"])
    print("seed top:")
    for row in seed_rows[: min(8, len(seed_rows))]:
        print(
            f"  obj={row['objective'] * 1e3:.1f} mm, "
            f"k={row['drag_k']:.4f}, C_h={row['table_C_h']:.3f}, C_v={row['table_C_v']:.3f}"
        )

    powell_rows = _minimize_from_starts(
        [np.array([r["drag_k"], r["table_C_h"], r["table_C_v"]], dtype=float) for r in seed_rows],
        bounds,
        train_cases,
        base_physics,
        cfg,
        table,
        args,
    )
    candidate_rows = seed_rows + powell_rows
    candidate_rows.sort(key=lambda r: r["objective"])

    final_rows = []
    seen = set()
    for row in candidate_rows:
        x = np.array([row["drag_k"], row["table_C_h"], row["table_C_v"]], dtype=float)
        key = tuple(np.round(x, 5))
        if key in seen:
            continue
        seen.add(key)
        train_full_metrics = _evaluate_x(x, train_cases_full, base_physics, cfg, table, args)
        val_metrics = _evaluate_x(x, val_cases, base_physics, cfg, table, args)
        final_rows.append(
            {
                **_candidate_dict(x),
                "train_sample_objective": float(row["objective"]),
                "train_full": train_full_metrics,
                "validation": val_metrics,
            }
        )
        print(
            f"final candidate {len(final_rows)}: train={train_full_metrics['objective'] * 1e3:.1f} mm, "
            f"val={val_metrics['objective'] * 1e3:.1f} mm, x={x.tolist()}"
        )
        if len(final_rows) >= args.keep_top:
            break

    final_rows.sort(key=lambda r: r["validation"]["objective"])
    result = {
        "segments": str(Path(args.segments).resolve()),
        "planner_yaml": str(Path(args.planner_yaml).resolve()),
        "physics_path": str(Path(args.physics_path).resolve()),
        "physics_source": args.physics_source,
        "geometry": geometry,
        "segments_manifest": _manifest_metadata(Path(args.segments)),
        "planner_frame_geometry_ok": frame_issue is None,
        "config": {
            "fit_window": int(cfg.fit_window),
            "bounce_center_z_max": float(cfg.bounce_center_z_max),
            "x_hit": float(cfg.x_hit),
            "eval_period_s": float(args.eval_period_s),
            "horizon_range_ms": [float(args.min_horizon_ms), float(args.max_horizon_ms)],
        },
        "bounds": bounds,
        "train_files": [str(p) for p in train_files],
        "validation_files": [str(p) for p in val_files],
        "num_train_cases_full": int(len(train_cases_full)),
        "num_train_cases_used": int(len(train_cases)),
        "num_validation_cases": int(len(val_cases)),
        "seed_rows": seed_rows,
        "best_by_validation": final_rows[0] if final_rows else None,
        "final_candidates": final_rows,
    }
    out_path = Path(args.out_json)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(json.dumps(result, indent=1), encoding="utf-8")

    printable = {
        "config": result["config"],
        "num_train_cases_used": result["num_train_cases_used"],
        "num_validation_cases": result["num_validation_cases"],
        "best_by_validation": {
            k: result["best_by_validation"][k]
            for k in ("drag_k", "table_C_h", "table_C_v")
        }
        if result["best_by_validation"]
        else None,
        "best_train_objective_mm": result["best_by_validation"]["train_full"]["objective"] * 1e3
        if result["best_by_validation"]
        else None,
        "best_validation_objective_mm": result["best_by_validation"]["validation"]["objective"] * 1e3
        if result["best_by_validation"]
        else None,
    }
    print(json.dumps(printable, indent=1))
    print(f"-> {out_path}")
    return result


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("segments", help="directory of canonical planner-frame t,x,y,z segment CSVs")
    parser.add_argument(
        "--planner-yaml",
        default=str(REPO_ROOT / "hope_ws" / "src" / "hope_planner" / "config" / "hope_planner.yaml"),
    )
    parser.add_argument("--physics-path", default=str(REPO_ROOT / "configs" / "ball_physics.yaml"))
    parser.add_argument("--out-json", default=str(REPO_ROOT / "analysis" / "planner_physics_continuous.json"))
    parser.add_argument(
        "--physics-source",
        choices=("shared", "planner-yaml"),
        default="shared",
        help="initial/base physics source; optimization itself may move anywhere within --bounds",
    )
    parser.add_argument("--x-hit", type=float, default=None)
    parser.add_argument("--table-y-max", type=float, default=None)
    parser.add_argument("--fit-window", type=int, default=None)
    parser.add_argument("--bounce-center-z-max", type=float, default=None)
    parser.add_argument("--bounds", default="0.05:0.30,0.45:0.95,0.75:1.00")
    parser.add_argument("--min-horizon-ms", type=float, default=50.0)
    parser.add_argument("--max-horizon-ms", type=float, default=500.0)
    parser.add_argument("--eval-period-s", type=float, default=0.02)
    parser.add_argument("--invalid-penalty-m", type=float, default=1.0)
    parser.add_argument("--train-fraction", type=float, default=0.7)
    parser.add_argument("--train-sample-cases", type=int, default=900)
    parser.add_argument("--random-candidates", type=int, default=12)
    parser.add_argument("--num-starts", type=int, default=4)
    parser.add_argument("--max-evals-per-start", type=int, default=45)
    parser.add_argument("--keep-top", type=int, default=8)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--limit", type=int, default=None)
    parser.add_argument(
        "--allow-bad-geometry",
        action="store_true",
        help="do not fail when near-table points fall outside planner table bounds",
    )
    return parser


def main() -> None:
    optimize(build_arg_parser().parse_args())


if __name__ == "__main__":
    main()
