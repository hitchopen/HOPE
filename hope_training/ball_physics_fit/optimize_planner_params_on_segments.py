"""Search HOPE planner prediction parameters on real ball trajectories.

This script uses the same planner components as the ROS node:

* every mocap sample is pushed into ``BallStateEstimator``;
* trajectory predictions are evaluated at a node-like solve period;
* ``BallTrajectoryPredictor`` predicts the next crossing of ``x_hit``;
* prediction errors are measured against the measured next crossing.

It only tunes parameters that the ball-position data can identify:
``fit_window``, centre-bounce detection height, flight drag ``k``, and
table-bounce ``C_h``/``C_v``. Racket contact and landing-target parameters
require tracked racket state and outgoing ball targets, so they are intentionally
out of scope here.
"""

from __future__ import annotations

import argparse
import json
import sys
from dataclasses import dataclass
from itertools import product
from pathlib import Path
from typing import Any

import numpy as np


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from evaluate_planner_on_segments import (  # noqa: E402
    REPO_ROOT,
    _crossings,
    _diagnose_geometry,
    _load_segment,
    _make_config,
    _manifest_metadata,
    _next_crossing,
    _planner_frame_issue,
    _segment_files,
    _summarize_predictions,
)

PLANNER_SRC = REPO_ROOT / "hope_ws" / "src" / "hope_planner"
if str(PLANNER_SRC) not in sys.path:
    sys.path.insert(0, str(PLANNER_SRC))

from hope_planner.ball_state_estimator import BallStateEstimator  # noqa: E402
from hope_planner.ball_trajectory_predictor import BallTrajectoryPredictor  # noqa: E402
from hope_planner.constants import BallPhysics  # noqa: E402


@dataclass(frozen=True)
class Case:
    file: str
    sample_i: int
    p_est: np.ndarray
    v_est: np.ndarray
    t_est: float
    actual_t: float
    actual_p: np.ndarray
    horizon_s: float


def _parse_float_list(text: str) -> list[float]:
    return [float(x.strip()) for x in text.split(",") if x.strip()]


def _parse_int_list(text: str) -> list[int]:
    return [int(x.strip()) for x in text.split(",") if x.strip()]


def _clone_physics(base: Any, k: float, c_h: float, c_v: float) -> BallPhysics:
    return BallPhysics(
        k=float(k),
        C_h=float(c_h),
        C_v=float(c_v),
        g=np.asarray(base.g, dtype=float).copy(),
        radius=float(base.radius),
        mass=float(base.mass),
    )


def _collect_cases(
    files: list[Path],
    cfg: Any,
    eval_period_s: float,
    min_horizon_s: float,
    max_horizon_s: float,
) -> list[Case]:
    cases: list[Case] = []
    for path in files:
        t, pos = _load_segment(path)
        if len(t) < max(cfg.fit_window, 6):
            continue
        crossings = _crossings(t, pos, cfg.x_hit)
        if not crossings:
            continue

        est = BallStateEstimator(cfg)
        next_eval_t = -np.inf
        for i, (ti, pi) in enumerate(zip(t, pos)):
            ti = float(ti)
            est.push(ti, pi)
            if eval_period_s > 0.0 and ti < next_eval_t:
                continue
            next_eval_t = ti + eval_period_s
            if not est.ready:
                continue
            p_est, v_est, t_est = est.estimate()
            if v_est[0] >= 0.0:
                continue
            actual = _next_crossing(crossings, ti, min_horizon_s)
            if actual is None:
                continue
            horizon = float(actual["t"] - ti)
            if horizon > max_horizon_s:
                continue
            cases.append(
                Case(
                    file=path.name,
                    sample_i=int(i),
                    p_est=p_est,
                    v_est=v_est,
                    t_est=float(t_est),
                    actual_t=float(actual["t"]),
                    actual_p=np.asarray(actual["p"], dtype=float),
                    horizon_s=horizon,
                )
            )
    return cases


def _stratified_sample(cases: list[Case], max_cases: int, seed: int) -> list[Case]:
    if max_cases <= 0 or len(cases) <= max_cases:
        return cases
    rng = np.random.default_rng(seed)
    horizons = np.asarray([c.horizon_s for c in cases])
    edges = np.array([0.0, 0.1, 0.2, 0.3, 0.5, np.inf])
    chosen: list[int] = []
    per_bin = max(1, max_cases // (len(edges) - 1))
    for lo, hi in zip(edges[:-1], edges[1:]):
        idx = np.flatnonzero((horizons >= lo) & (horizons < hi))
        if len(idx) <= per_bin:
            chosen.extend(idx.tolist())
        else:
            chosen.extend(rng.choice(idx, size=per_bin, replace=False).tolist())
    if len(chosen) < max_cases:
        rest = np.setdiff1d(np.arange(len(cases)), np.asarray(chosen, dtype=int), assume_unique=False)
        add = min(max_cases - len(chosen), len(rest))
        if add:
            chosen.extend(rng.choice(rest, size=add, replace=False).tolist())
    chosen = sorted(set(chosen))[:max_cases]
    return [cases[i] for i in chosen]


def _evaluate_cases(
    cases: list[Case],
    physics: BallPhysics,
    cfg: Any,
    table: Any,
    invalid_penalty_m: float,
) -> dict[str, Any]:
    predictor = BallTrajectoryPredictor(physics, cfg, table)
    rows = []
    invalid = 0
    for case in cases:
        strike = predictor.predict(case.p_est, case.v_est, case.t_est)
        if not strike.valid:
            invalid += 1
            continue
        err = strike.p_ball - case.actual_p
        rows.append(
            {
                "file": case.file,
                "sample_i": case.sample_i,
                "horizon_s": case.horizon_s,
                "predicted_horizon_s": float(strike.t_strike - case.t_est),
                "err_t_s": float(strike.t_strike - case.actual_t),
                "err_x_m": float(err[0]),
                "err_y_m": float(err[1]),
                "err_z_m": float(err[2]),
                "err_yz_m": float(np.linalg.norm(err[1:3])),
                "predicted_p": [float(v) for v in strike.p_ball],
                "actual_p": [float(v) for v in case.actual_p],
                "num_bounces": int(strike.num_bounces),
            }
        )
    valid_errors = [r["err_yz_m"] for r in rows]
    objective_values = valid_errors + [invalid_penalty_m] * invalid
    if objective_values:
        obj = float(np.median(objective_values) + 0.25 * np.percentile(objective_values, 90))
    else:
        obj = invalid_penalty_m * 2.0
    valid_rate = len(rows) / max(len(cases), 1)
    return {
        "objective": obj,
        "valid_rate": float(valid_rate),
        "invalid": int(invalid),
        "num_cases": int(len(cases)),
        "num_valid": int(len(rows)),
        "summary": _summarize_predictions(rows),
        "rows": rows,
    }


def optimize(args: argparse.Namespace) -> dict[str, Any]:
    base_physics, base_cfg, table = _make_config(args)
    files = _segment_files(Path(args.segments))
    if args.limit:
        files = files[: args.limit]
    if not files:
        raise SystemExit(f"no segment CSVs found under {args.segments}")
    geometry = _diagnose_geometry(files, table, base_physics)
    manifest_metadata = _manifest_metadata(Path(args.segments))
    frame_issue = _planner_frame_issue(geometry)
    if frame_issue and not args.allow_bad_geometry:
        raise SystemExit(
            f"{frame_issue} Use --allow-bad-geometry only for intentional diagnostics."
        )

    min_h = args.min_horizon_ms * 1e-3
    max_h = args.max_horizon_ms * 1e-3
    fit_windows = _parse_int_list(args.fit_windows)
    bounce_center_z_values = _parse_float_list(args.bounce_center_z_max_values)
    k_values = _parse_float_list(args.drag_k_values)
    ch_values = _parse_float_list(args.table_ch_values)
    cv_values = _parse_float_list(args.table_cv_values)

    fit_window_results = []
    cases_by_key: dict[tuple[int, float], list[Case]] = {}
    for fw, bounce_center_z_max in product(fit_windows, bounce_center_z_values):
        cfg = base_cfg
        cfg.fit_window = int(fw)
        cfg.bounce_center_z_max = float(bounce_center_z_max)
        cfg.max_predict_time = max(float(cfg.max_predict_time), max_h)
        cases = _collect_cases(files, cfg, args.eval_period_s, min_h, max_h)
        cases_eval = _stratified_sample(cases, args.max_cases, args.seed)
        cases_by_key[(fw, bounce_center_z_max)] = cases_eval
        current = _evaluate_cases(
            cases_eval, base_physics, cfg, table, invalid_penalty_m=args.invalid_penalty_m
        )
        fit_window_results.append(
            {
                "fit_window": int(fw),
                "bounce_center_z_max": float(bounce_center_z_max),
                "full_cases": int(len(cases)),
                "sampled_cases": int(len(cases_eval)),
                "objective": current["objective"],
                "valid_rate": current["valid_rate"],
                "baseline_summary": current["summary"],
            }
        )
        print(
            f"fit_window={fw}, bounce_center_z_max={bounce_center_z_max:.3f}: "
            f"cases={len(cases_eval)}/{len(cases)}, "
            f"baseline objective={current['objective'] * 1e3:.1f} mm"
        )

    best_estimator_row = min(fit_window_results, key=lambda r: r["objective"])
    best_fw = int(best_estimator_row["fit_window"])
    best_bounce_center_z_max = float(best_estimator_row["bounce_center_z_max"])
    best_cfg = base_cfg
    best_cfg.fit_window = best_fw
    best_cfg.bounce_center_z_max = best_bounce_center_z_max
    best_cfg.max_predict_time = max(float(best_cfg.max_predict_time), max_h)
    search_cases = cases_by_key[(best_fw, best_bounce_center_z_max)]

    grid_results = []
    total = len(k_values) * len(ch_values) * len(cv_values)
    for idx, (k, c_h, c_v) in enumerate(product(k_values, ch_values, cv_values), start=1):
        physics = _clone_physics(base_physics, k, c_h, c_v)
        metrics = _evaluate_cases(
            search_cases, physics, best_cfg, table, invalid_penalty_m=args.invalid_penalty_m
        )
        grid_results.append(
            {
                "drag_k": float(k),
                "table_C_h": float(c_h),
                "table_C_v": float(c_v),
                "objective": metrics["objective"],
                "valid_rate": metrics["valid_rate"],
                "num_valid": metrics["num_valid"],
                "num_cases": metrics["num_cases"],
                "summary": metrics["summary"],
            }
        )
        if idx == 1 or idx == total or idx % max(1, total // 10) == 0:
            print(f"physics grid {idx}/{total}: current objective={metrics['objective'] * 1e3:.1f} mm")

    grid_results.sort(key=lambda r: r["objective"])
    best_physics_row = grid_results[0]
    best_physics = _clone_physics(
        base_physics,
        best_physics_row["drag_k"],
        best_physics_row["table_C_h"],
        best_physics_row["table_C_v"],
    )
    final_metrics = _evaluate_cases(
        cases_by_key[(best_fw, best_bounce_center_z_max)],
        best_physics,
        best_cfg,
        table,
        invalid_penalty_m=args.invalid_penalty_m,
    )
    baseline_metrics = _evaluate_cases(
        cases_by_key[(best_fw, best_bounce_center_z_max)],
        base_physics,
        best_cfg,
        table,
        invalid_penalty_m=args.invalid_penalty_m,
    )

    result = {
        "segments": str(Path(args.segments).resolve()),
        "num_files": int(len(files)),
        "horizon_range_ms": [float(args.min_horizon_ms), float(args.max_horizon_ms)],
        "eval_period_s": float(args.eval_period_s),
        "max_cases": int(args.max_cases),
        "geometry": geometry,
        "segments_manifest": manifest_metadata,
        "planner_frame_geometry_ok": frame_issue is None,
        "baseline": {
            "fit_window": int(best_fw),
            "bounce_center_z_max": float(best_bounce_center_z_max),
            "drag_k": float(base_physics.k),
            "table_C_h": float(base_physics.C_h),
            "table_C_v": float(base_physics.C_v),
            "objective": baseline_metrics["objective"],
            "valid_rate": baseline_metrics["valid_rate"],
            "summary": baseline_metrics["summary"],
        },
        "best": {
            "fit_window": int(best_fw),
            "bounce_center_z_max": float(best_bounce_center_z_max),
            "drag_k": float(best_physics_row["drag_k"]),
            "table_C_h": float(best_physics_row["table_C_h"]),
            "table_C_v": float(best_physics_row["table_C_v"]),
            "objective": float(final_metrics["objective"]),
            "valid_rate": float(final_metrics["valid_rate"]),
            "summary": final_metrics["summary"],
        },
        "fit_window_results": fit_window_results,
        "top_grid_results": grid_results[: args.keep_top],
    }
    out_path = Path(args.out_json)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(json.dumps(result, indent=1), encoding="utf-8")

    printable = {
        "baseline": result["baseline"],
        "best": result["best"],
        "fit_window_results": fit_window_results,
        "top_grid_results": grid_results[: min(5, args.keep_top)],
    }
    print(json.dumps(printable, indent=1))
    print(f"-> {out_path}")
    return result


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("segments", help="directory of canonical t,x,y,z segment CSVs")
    parser.add_argument(
        "--planner-yaml",
        default=str(REPO_ROOT / "hope_ws" / "src" / "hope_planner" / "config" / "hope_planner.yaml"),
    )
    parser.add_argument("--physics-path", default=str(REPO_ROOT / "configs" / "ball_physics.yaml"))
    parser.add_argument("--out-json", default=str(REPO_ROOT / "analysis" / "planner_param_search.json"))
    parser.add_argument("--x-hit", type=float, default=None)
    parser.add_argument("--table-y-max", type=float, default=None)
    parser.add_argument("--fit-window", type=int, default=None, help="ignored; use --fit-windows")
    parser.add_argument("--fit-windows", default="25,31,37,43,49")
    parser.add_argument("--bounce-center-z-max-values", default="0.05")
    parser.add_argument("--drag-k-values", default="0.00,0.05,0.10,0.1261,0.16,0.20,0.25,0.32")
    parser.add_argument("--table-ch-values", default="0.45,0.55,0.631,0.70,0.80,0.90,1.00")
    parser.add_argument("--table-cv-values", default="0.75,0.85,0.9215,1.00,1.08")
    parser.add_argument("--min-horizon-ms", type=float, default=50.0)
    parser.add_argument("--max-horizon-ms", type=float, default=500.0)
    parser.add_argument("--eval-period-s", type=float, default=0.02)
    parser.add_argument("--invalid-penalty-m", type=float, default=1.0)
    parser.add_argument(
        "--allow-bad-geometry",
        action="store_true",
        help="do not fail when near-table points fall outside planner table bounds",
    )
    parser.add_argument("--max-cases", type=int, default=700)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--keep-top", type=int, default=20)
    parser.add_argument("--limit", type=int, default=None)
    return parser


def main() -> None:
    optimize(build_arg_parser().parse_args())


if __name__ == "__main__":
    main()
