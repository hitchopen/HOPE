#!/usr/bin/env python3
"""Host-only envelope audit for the Gate3 HITTER planner sweep.

This reuses the production Stage-2/Stage-3 physics, but never starts ROS, Isaac,
MuJoCo, or the policy.  It answers the narrow pre-training question: can one
per-side ``(target_land_x, target_land_y, delta_t_flight)`` tuple map every Gate3 serve into the
policy's commanded racket box while retaining a physically legal return?

The serve list, aim, split, and intercept geometry are read from a versioned
Gate3 wrapper so the audit cannot silently validate the internal engine's stale
fallback literals instead of the deployed profile.
"""

from __future__ import annotations

import argparse
import json
import re
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[3]
SCRIPT_DIR = Path(__file__).resolve().parent
PLANNER_SRC = ROOT / "hope_ws/src/hope_planner"
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))
if str(PLANNER_SRC) not in sys.path:
    sys.path.insert(0, str(PLANNER_SRC))

from hope_planner.ball_trajectory_predictor import BallTrajectoryPredictor  # noqa: E402
from hope_planner.constants import BallPhysics, PlannerConfig, TableParams  # noqa: E402
from hope_planner.racket_target_planner import RacketTargetPlanner  # noqa: E402
from pp_gate3_core import parse_serves_list  # noqa: E402


DEFAULT_GATE3 = (
    ROOT
    / "a3_deploy/a3_deploy_example/scripts/pp_gate3_hitter_pingpong.sh"
)
DEFAULT_RECEIPT = (
    ROOT
    / "hope_training/whole_body_tracking/analysis/"
    "hitter_rally_final_v3_v7_approved_receipt.json"
)


@dataclass(frozen=True)
class ClipBox:
    reach_y: tuple[float, float]
    z: tuple[float, float]
    velocity: np.ndarray  # shape (3, 2)


PROVEN_FINAL_V2 = {
    "forehand": ClipBox(
        reach_y=(-0.40, -0.40),
        z=(0.67, 0.97),
        velocity=np.array([[1.05, 2.05], [0.96, 1.96], [0.31, 1.11]]),
    ),
    "backhand": ClipBox(
        reach_y=(0.20, 0.20),
        z=(0.88, 1.18),
        velocity=np.array([[1.61, 2.61], [-1.21, -0.21], [0.00, 0.71]]),
    ),
}

RALLY_V8 = {
    "forehand": ClipBox(
        reach_y=(-0.48, -0.40),
        z=(0.85, 1.30),
        velocity=np.array([[1.24, 2.24], [-0.31, 0.69], [0.66, 1.66]]),
    ),
    "backhand": ClipBox(
        reach_y=(-0.13, -0.05),
        z=(0.85, 1.30),
        velocity=np.array([[1.60, 2.60], [-0.66, 0.34], [0.00, 0.54]]),
    ),
}

RALLY_V10 = {
    "forehand": ClipBox(
        reach_y=(-0.48, -0.40),
        z=(0.85, 1.30),
        velocity=np.array([[1.57, 2.55], [0.10, 0.52], [0.41, 1.35]]),
    ),
    "backhand": ClipBox(
        reach_y=(-0.13, -0.05),
        z=(0.85, 1.30),
        velocity=np.array([[1.55, 2.52], [-0.18, 0.29], [0.40, 1.32]]),
    ),
}
RALLY_V10_UNION = {
    "forehand": np.array([[1.24, 2.60], [-0.31, 0.69], [0.40, 1.66]]),
    "backhand": np.array([[1.50, 2.60], [-0.66, 0.40], [0.00, 1.35]]),
}

# Gate3 begins observing the incoming ball only after it is already above the
# opponent's regulation half.  This keeps the certification timing honest:
# the old synthetic sweep started 0.96 m beyond the far edge and supplied
# roughly 2.58 s of warning, which could hide a policy that cannot finish its
# step/recovery before the next real ball.  V1 and V8 exercise 0.917--1.183 s;
# retain a small fixed margin for future physically equivalent serve tuning.
GATE3_VISIBLE_LEAD_RANGE_S = (0.85, 1.25)


def _numbers(text: str) -> list[float]:
    return [float(x) for x in re.findall(r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)", text)]


def load_exported_number(path: Path, name: str, cast=float):
    """Read one literal exported wrapper value used as a versioned Gate3 contract."""
    text = path.read_text()
    match = re.search(
        rf"^export\s+{re.escape(name)}=(?:['\"])?([-+.\d]+)(?:['\"])?\s*$",
        text,
        flags=re.MULTILINE,
    )
    if match is None:
        raise RuntimeError(f"could not find literal export {name} in {path}")
    return cast(match.group(1))


def load_harness(path: Path, serves_override: str | None = None) -> tuple[
    np.ndarray, dict[str, tuple[float, float]], dict[str, float]
]:
    text = path.read_text()
    # Accept both the original literal and the operational env-override form:
    #   serves:='[1,2,...]'
    #   serves:='${PP_SERVES_LIST:-[1,2,...]}'
    if serves_override is not None:
        specs = parse_serves_list(serves_override)
        flat = [
            value
            for spec in specs
            for value in (*spec.position, *spec.velocity)
        ]
    else:
        serve_match = re.search(r"export\s+PP_SERVES_LIST=['\"]?\[([^]]+)\]", text)
        if serve_match is None:
            serve_match = re.search(
                r"-p\s+serves:='(?:\$\{PP_SERVES_LIST:-)?\[([^]]+)\](?:\})?'",
                text,
                re.DOTALL,
            )
        if serve_match is None:
            raise RuntimeError(
                f"could not find physical serves in {path}; pass --serves-list"
            )
        flat = _numbers(serve_match.group(1))
        if len(flat) == 0 or len(flat) % 6:
            raise RuntimeError(
                f"Gate3 serve list must contain 6*N numbers, got {len(flat)}"
            )

    def parameter_default(parameter: str, env_name: str) -> float:
        match = re.search(
            rf"{parameter}:=(?:\$\{{{env_name}:-)?([-+.\d]+)(?:\}})?",
            text,
        )
        if match is not None:
            return float(match.group(1))
        env_match = re.search(
            rf"(?:^|[ \t]){env_name}=(?:['\"])?(?:\$\{{{env_name}:-)?([-+.\d]+)",
            text,
            flags=re.MULTILINE,
        )
        if env_match is None:
            raise RuntimeError(f"could not find {parameter}/{env_name} in {path}")
        return float(env_match.group(1))

    fh_y = parameter_default("target_land_y_fh", "PP_LAND_Y_FH")
    bh_y = parameter_default("target_land_y_bh", "PP_LAND_Y_BH")
    fh_dt = parameter_default("delta_t_flight_fh", "PP_DTF_FH")
    bh_dt = parameter_default("delta_t_flight_bh", "PP_DTF_BH")
    aims = {
        "forehand": (fh_y, fh_dt),
        "backhand": (bh_y, bh_dt),
    }
    geometry = {
        "target_land_x": parameter_default("target_land_x", "PP_LAND_X"),
        "x_hit": parameter_default("x_hit", "PP_XHIT"),
        "x_hit_bh_delta": parameter_default("x_hit_bh_delta", "PP_XHIT_BH_DELTA"),
        "split_y": parameter_default("swing_side_split_y", "PP_SPLIT_Y"),
        "split_hyst": parameter_default("swing_side_hysteresis_y", "PP_SPLIT_HYST"),
        "demo": bool(re.search(r"^export\s+PP_EXTRA_ARGS=.*--demo", text, re.MULTILINE)),
        "policy_z_offset": 0.760,
        "station_y_anchor": -0.7625,
    }
    return np.asarray(flat, dtype=float).reshape(-1, 6), aims, geometry


def load_receipt(path: Path) -> dict[str, ClipBox]:
    recipe = json.loads(path.read_text())["recipe"]["clips"]
    result: dict[str, ClipBox] = {}
    for side in ("forehand", "backhand"):
        pos = recipe[side]["position_box"]
        vel = recipe[side]["velocity_box"]
        result[side] = ClipBox(
            reach_y=(float(pos["y"][0]), float(pos["y"][1])),
            z=(float(pos["z"][0]), float(pos["z"][1])),
            velocity=np.asarray([vel["x"], vel["y"], vel["z"]], dtype=float),
        )
    return result


def _axis_margin(values: np.ndarray, box: np.ndarray) -> np.ndarray:
    return np.minimum(values - box[:, 0], box[:, 1] - values)


def audit(
    serves: np.ndarray,
    boxes: dict[str, ClipBox],
    aims: dict[str, tuple[float, float]],
    geometry: dict[str, float],
    demo_velocity_boxes: dict[str, np.ndarray] | None = None,
    *,
    station_limit: float = 0.35,
    runner_station_margin: float = 0.05,
    runner_velocity_margin: float = 0.30,
) -> dict:
    physics = BallPhysics(k=0.1261, C_h=0.64, C_v=0.9215)
    table = TableParams(y_max=0.0)
    rows = []
    previous_station = geometry["station_y_anchor"]
    last_side = 0
    transitions = []
    signed_transitions = []
    for index, serve in enumerate(serves):
        fh_config = PlannerConfig(x_hit=geometry["x_hit"], max_predict_time=2.6)
        fh_predictor = BallTrajectoryPredictor(physics, fh_config, table)
        fh_strike = fh_predictor.predict(serve[:3], serve[3:], 0.0)
        rel_y = float(fh_strike.p_ball[1] - previous_station)
        split = geometry["split_y"]
        hyst = geometry["split_hyst"]
        if last_side > 0:
            side = "backhand" if rel_y > split + hyst else "forehand"
        elif last_side < 0:
            side = "forehand" if rel_y < split - hyst else "backhand"
        else:
            side = "forehand" if rel_y < split else "backhand"
        side_sign = 1 if side == "forehand" else -1
        config = PlannerConfig(
            x_hit=geometry["x_hit"] + (geometry["x_hit_bh_delta"] if side_sign < 0 else 0.0),
            max_predict_time=2.6,
        )
        predictor = BallTrajectoryPredictor(physics, config, table)
        strike = fh_strike if side_sign > 0 or geometry["x_hit_bh_delta"] == 0.0 else predictor.predict(
            serve[:3], serve[3:], 0.0
        )
        target_planner = RacketTargetPlanner(physics, config, table)
        box = boxes[side]
        land_y, flight_time = aims[side]
        target_land = np.array([geometry["target_land_x"], land_y, 0.0])
        v_out = target_planner._compute_outgoing_velocity(
            strike.p_ball, target_land, flight_time
        )
        v_racket, _ = target_planner._compute_racket_velocity(
            strike.v_ball, v_out, config.C_r
        )
        end, _ = target_planner._integrate_free_flight(
            strike.p_ball, v_out, flight_time
        )
        at_net = target_planner._free_flight_position_at_x(
            strike.p_ball, v_out, table.net_x
        )

        reach_lo, reach_hi = box.reach_y
        reach_at_held_station = float(strike.p_ball[1] - previous_station)
        if reach_lo <= reach_at_held_station <= reach_hi:
            station_y = previous_station
        elif reach_at_held_station > reach_hi:
            station_y = float(strike.p_ball[1] - reach_hi)
        else:
            station_y = float(strike.p_ball[1] - reach_lo)
        signed_transition = station_y - previous_station
        transition = abs(signed_transition)
        if index:
            transitions.append(transition)
            signed_transitions.append(signed_transition)
        previous_station = station_y
        last_side = side_sign

        margin = _axis_margin(v_racket, box.velocity)
        command_velocity = v_racket
        if geometry["demo"]:
            command_box = (
                demo_velocity_boxes[side] if demo_velocity_boxes is not None else box.velocity
            )
            command_velocity = np.mean(command_box, axis=1)
        command_margin = _axis_margin(command_velocity, box.velocity)
        planned_net_ok = bool(
            at_net is not None
            and table.y_max - table.width - table.net_overhang
            <= float(at_net[1])
            <= table.y_max + table.net_overhang
            and float(at_net[2]) > table.net_height + 0.03
        )
        planned_landing_ok = bool(
            table.net_x < target_land[0] < table.length
            and table.y_max - table.width <= land_y <= table.y_max
            and np.linalg.norm(end - target_land) <= 5e-4
            and planned_net_ok
        )
        visible_lead_s = float(strike.t_strike)
        initial_state_in_arena = bool(
            table.net_x <= float(serve[0]) <= table.length
            and table.y_max - table.width <= float(serve[1]) <= table.y_max
            and float(serve[2]) > 0.0
            and float(serve[3]) < 0.0
        )
        visible_lead_ok = bool(
            GATE3_VISIBLE_LEAD_RANGE_S[0]
            <= visible_lead_s
            <= GATE3_VISIBLE_LEAD_RANGE_S[1]
        )
        strike_policy_z = float(strike.p_ball[2] + geometry["policy_z_offset"])
        rows.append(
            {
                "serve": index + 1,
                "side": side,
                "initial_ball_state": serve.tolist(),
                "initial_state_in_regulation_arena": initial_state_in_arena,
                "visible_lead_time_s": visible_lead_s,
                "visible_lead_time_ok": visible_lead_ok,
                "strike_position": strike.p_ball.tolist(),
                "strike_velocity": strike.v_ball.tolist(),
                "racket_velocity": v_racket.tolist(),
                "raw_velocity_margin": margin.tolist(),
                "raw_velocity_ok": bool(np.all(margin >= 0.0)),
                "runner_velocity_ok": bool(np.all(margin >= -runner_velocity_margin)),
                "command_velocity": command_velocity.tolist(),
                "command_velocity_margin": command_margin.tolist(),
                "command_velocity_ok": bool(np.all(command_margin >= 0.0)),
                "strike_position_policy": [
                    float(strike.p_ball[0]),
                    float(strike.p_ball[1]),
                    strike_policy_z,
                ],
                "incoming_bounces": int(strike.num_bounces),
                "incoming_scenario_ok": bool(
                    strike.valid
                    and strike.num_bounces == 1
                    and initial_state_in_arena
                    and visible_lead_ok
                ),
                "raw_z_ok": bool(box.z[0] <= strike_policy_z <= box.z[1]),
                "runner_z_ok": bool(
                    box.z[0] - 0.05 <= strike_policy_z <= box.z[1] + 0.05
                ),
                "station_y": station_y,
                "station_offset_y": station_y - geometry["station_y_anchor"],
                "station_strict_ok": (
                    abs(station_y - geometry["station_y_anchor"]) <= station_limit
                ),
                "station_runner_ok": (
                    abs(station_y - geometry["station_y_anchor"])
                    <= station_limit + runner_station_margin
                ),
                "landing_error": float(np.linalg.norm(end - target_land)),
                "net_position": None if at_net is None else at_net.tolist(),
                # This is Stage-3's ballistic prediction from the requested outgoing ball state.
                # It is not a measured racket-ball collision or an observed post-contact landing.
                "planned_landing_ok": planned_landing_ok,
            }
        )

    by_side = {}
    for side in ("forehand", "backhand"):
        side_rows = [row for row in rows if row["side"] == side]
        velocities = np.asarray([row["racket_velocity"] for row in side_rows])
        margins = np.asarray([row["raw_velocity_margin"] for row in side_rows])
        by_side[side] = {
            "aim": {
                "target_land_x": geometry["target_land_x"],
                "target_land_y": aims[side][0],
                "delta_t_flight": aims[side][1],
            },
            "velocity_min": velocities.min(axis=0).tolist(),
            "velocity_max": velocities.max(axis=0).tolist(),
            "raw_axis_margin_min": margins.min(axis=0).tolist(),
            "raw_velocity_coverage": sum(row["raw_velocity_ok"] for row in side_rows),
            "runner_velocity_coverage": sum(
                row["runner_velocity_ok"] for row in side_rows
            ),
            "command_velocity_coverage": sum(
                row["command_velocity_ok"] for row in side_rows
            ),
            "raw_z_coverage": sum(row["raw_z_ok"] for row in side_rows),
            "runner_z_coverage": sum(row["runner_z_ok"] for row in side_rows),
            "planner_ballistic_landing_coverage": sum(
                row["planned_landing_ok"] for row in side_rows
            ),
        }

    report = {
        "rows": rows,
        "by_side": by_side,
        "station_strict_coverage": sum(row["station_strict_ok"] for row in rows),
        "station_runner_coverage": sum(row["station_runner_ok"] for row in rows),
        "station_transitions_after_first": transitions,
        "station_signed_transitions_after_first": signed_transitions,
        "demo_substitution_active": bool(geometry["demo"]),
        "physical_contact_measured": False,
        "landing_measured": False,
        "visible_lead_time_range_s": list(GATE3_VISIBLE_LEAD_RANGE_S),
    }
    report["raw_planner_velocity_pass"] = bool(all(row["raw_velocity_ok"] for row in rows))
    report["runner_margin_velocity_pass"] = bool(all(row["runner_velocity_ok"] for row in rows))
    report["command_velocity_pass"] = bool(all(row["command_velocity_ok"] for row in rows))
    report["planner_ballistic_return_pass"] = bool(
        all(row["planned_landing_ok"] for row in rows))
    report["station_input_pass"] = bool(all(row["station_strict_ok"] for row in rows))
    report["incoming_physical_scenario_pass"] = bool(
        all(row["incoming_scenario_ok"] for row in rows)
    )
    report["initial_state_arena_pass"] = bool(
        all(row["initial_state_in_regulation_arena"] for row in rows)
    )
    report["visible_lead_time_pass"] = bool(
        all(row["visible_lead_time_ok"] for row in rows)
    )
    report["autonomous_side_coverage_pass"] = bool(
        {row["side"] for row in rows} == {"forehand", "backhand"}
    )
    geometry_pass = bool(
        all(row["raw_z_ok"] for row in rows)
        and report["planner_ballistic_return_pass"]
        and report["station_input_pass"]
        and report["incoming_physical_scenario_pass"]
        and report["autonomous_side_coverage_pass"])
    report["runner_harness_pass"] = bool(report["command_velocity_pass"] and geometry_pass)
    report["planner_contract_pass"] = bool(
        report["raw_planner_velocity_pass"] and geometry_pass and not geometry["demo"])
    # Backward-compatible process verdict: the caller chooses which named layer is required.
    report["pass"] = report["runner_harness_pass"]
    return report


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--gate3-script", type=Path, default=DEFAULT_GATE3)
    parser.add_argument(
        "--serves-list",
        help="Exact side-neutral physical 6*N scenario list (table-surface frame).",
    )
    parser.add_argument(
        "--contract", choices=("proven_final_v2", "receipt", "rally_v8", "rally_v10", "rally_v11", "rally_v12", "rally_v13", "rally_v14", "rally_v15", "rally_v17", "rally_v17_r10"),
        default="rally_v10",
    )
    parser.add_argument("--receipt", type=Path, default=DEFAULT_RECEIPT)
    parser.add_argument("--land-x", type=float)
    parser.add_argument("--fh-land-y", type=float)
    parser.add_argument("--fh-dtf", type=float)
    parser.add_argument("--bh-land-y", type=float)
    parser.add_argument("--bh-dtf", type=float)
    parser.add_argument("--json-out", type=Path)
    parser.add_argument(
        "--verdict", choices=("runner_harness", "planner_contract"),
        default="runner_harness",
        help="Select the named evidence layer used for the process exit code.",
    )
    args = parser.parse_args()

    serves, aims, geometry = load_harness(args.gate3_script, args.serves_list)
    if args.land_x is not None:
        geometry["target_land_x"] = args.land_x
    if args.fh_land_y is not None:
        aims["forehand"] = (args.fh_land_y, aims["forehand"][1])
    if args.fh_dtf is not None:
        aims["forehand"] = (aims["forehand"][0], args.fh_dtf)
    if args.bh_land_y is not None:
        aims["backhand"] = (args.bh_land_y, aims["backhand"][1])
    if args.bh_dtf is not None:
        aims["backhand"] = (aims["backhand"][0], args.bh_dtf)
    boxes = (
        PROVEN_FINAL_V2 if args.contract == "proven_final_v2"
        else (
            RALLY_V8 if args.contract == "rally_v8"
            else (
                RALLY_V10
                if args.contract in (
                    "rally_v10", "rally_v11", "rally_v12",
                    "rally_v13", "rally_v14", "rally_v15", "rally_v17",
                    "rally_v17_r10",
                )
                else load_receipt(args.receipt)
            )
        )
    )
    report = audit(
        serves, boxes, aims, geometry,
        RALLY_V10_UNION if args.contract in ("rally_v10", "rally_v11", "rally_v12", "rally_v13", "rally_v14", "rally_v15", "rally_v17", "rally_v17_r10") else None,
    )
    if args.contract in (
        "rally_v11", "rally_v12", "rally_v13", "rally_v14", "rally_v17"
    ):
        positive_main = [
            value for value in report["station_signed_transitions_after_first"]
            if 0.19 - 1e-3 <= value <= 0.24 + 1e-3
        ]
        report["positive_main_transitions"] = positive_main
        report["positive_main_coverage_ok"] = len(positive_main) >= 2
        report["runner_harness_pass"] = (
            report["runner_harness_pass"] and report["positive_main_coverage_ok"])
        report["planner_contract_pass"] = (
            report["planner_contract_pass"] and report["positive_main_coverage_ok"])
    elif args.contract in ("rally_v8", "rally_v15"):
        step_lo = load_exported_number(args.gate3_script, "PP_STATION_STEP_LO")
        step_hi = load_exported_number(args.gate3_script, "PP_STATION_STEP_HI")
        min_steps = load_exported_number(
            args.gate3_script, "PP_MIN_STATION_TRANSITIONS", int
        )
        finite_steps = [
            value for value in report["station_signed_transitions_after_first"]
            if step_lo - 1e-3 <= abs(value) <= step_hi + 1e-3
        ]
        directions_ok = any(value > 0.0 for value in finite_steps) and any(
            value < 0.0 for value in finite_steps
        )
        report["finite_step_range_m"] = [step_lo, step_hi]
        report["finite_step_min_required"] = min_steps
        report["finite_step_transitions"] = finite_steps
        report["finite_step_directions_ok"] = directions_ok
        report["finite_step_coverage_ok"] = len(finite_steps) >= min_steps and directions_ok
        report["runner_harness_pass"] = (
            report["runner_harness_pass"] and report["finite_step_coverage_ok"])
        report["planner_contract_pass"] = (
            report["planner_contract_pass"] and report["finite_step_coverage_ok"])
    elif args.contract == "rally_v17_r10":
        # R10 freezes the station at MOTION entry.  The offline station solve is
        # diagnostic only; every random draw must already be reachable from that
        # anchor (allowing <=1 cm numerical/trajectory-fit residue), so no moving
        # station curriculum can leak back into the Gate3 scenario.
        offsets = [abs(row["station_offset_y"]) for row in report["rows"]]
        report["fixed_station_max_abs_offset_m"] = max(offsets, default=float("inf"))
        report["fixed_station_coverage_ok"] = bool(
            offsets and all(value <= 0.010 + 1.0e-9 for value in offsets)
        )
        report["runner_harness_pass"] = bool(
            report["runner_harness_pass"] and report["fixed_station_coverage_ok"]
        )
        report["planner_contract_pass"] = bool(
            report["planner_contract_pass"] and report["fixed_station_coverage_ok"]
        )
    report["contract"] = args.contract
    report["gate3_script"] = str(args.gate3_script)

    print(f"contract={args.contract} serves={len(serves)}")
    for side in ("forehand", "backhand"):
        item = report["by_side"][side]
        print(
            f"{side}: aim_x={item['aim']['target_land_x']:.4f} "
            f"aim_y={item['aim']['target_land_y']:+.4f} "
            f"dt={item['aim']['delta_t_flight']:.4f} "
            f"vmin={np.round(item['velocity_min'], 4).tolist()} "
            f"vmax={np.round(item['velocity_max'], 4).tolist()}"
        )
        print(
            f"  raw_vel={item['raw_velocity_coverage']}/{sum(row['side'] == side for row in report['rows'])} "
            f"worst_axis_margin={np.round(item['raw_axis_margin_min'], 4).tolist()} "
            f"runner_vel={item['runner_velocity_coverage']}/{sum(row['side'] == side for row in report['rows'])} "
            f"raw_z={item['raw_z_coverage']}/{sum(row['side'] == side for row in report['rows'])} "
            f"planned_land={item['planner_ballistic_landing_coverage']}/{sum(row['side'] == side for row in report['rows'])}"
        )
        print(
            f"  commanded_vel={item['command_velocity_coverage']}/"
            f"{sum(row['side'] == side for row in report['rows'])} "
            f"mode={'demo-union-center' if geometry['demo'] else 'planner-raw'}"
        )
    print(
        f"station strict={report['station_strict_coverage']}/{len(serves)} "
        f"runner={report['station_runner_coverage']}/{len(serves)} "
        "transitions="
        f"{np.round(report['station_transitions_after_first'], 4).tolist()}"
    )
    lead_times = [row["visible_lead_time_s"] for row in report["rows"]]
    print(
        "incoming visible lead="
        f"[{min(lead_times):.3f},{max(lead_times):.3f}] s "
        f"required=[{GATE3_VISIBLE_LEAD_RANGE_S[0]:.2f},"
        f"{GATE3_VISIBLE_LEAD_RANGE_S[1]:.2f}] s "
        f"arena={'PASS' if report['initial_state_arena_pass'] else 'FAIL'} "
        f"timing={'PASS' if report['visible_lead_time_pass'] else 'FAIL'}"
    )
    if args.contract in (
        "rally_v11", "rally_v12", "rally_v13", "rally_v14", "rally_v17"
    ):
        print(
            "positive-main [+0.19,+0.24] m="
            f"{len(report['positive_main_transitions'])} "
            f"values={np.round(report['positive_main_transitions'], 4).tolist()}"
        )
    elif args.contract in ("rally_v8", "rally_v15"):
        print(
            f"finite station steps {report['finite_step_range_m']} m="
            f"{len(report['finite_step_transitions'])}/{report['finite_step_min_required']} "
            f"directions={'PASS' if report['finite_step_directions_ok'] else 'FAIL'} "
            f"values={np.round(report['finite_step_transitions'], 4).tolist()}"
        )
    elif args.contract == "rally_v17_r10":
        print(
            "fixed session station max implied offset="
            f"{report['fixed_station_max_abs_offset_m']:.4f} m "
            f"({'PASS' if report['fixed_station_coverage_ok'] else 'FAIL'})"
        )
    verdict_key = f"{args.verdict}_pass"
    report["selected_verdict"] = args.verdict
    report["pass"] = bool(report[verdict_key])
    print(
        "evidence: "
        f"runner_harness={'PASS' if report['runner_harness_pass'] else 'FAIL'} "
        f"planner_contract={'PASS' if report['planner_contract_pass'] else 'FAIL'} "
        "physical_contact=NOT_MEASURED landing=NOT_MEASURED"
    )
    print(f"AUDIT[{args.verdict}] {'PASS' if report['pass'] else 'FAIL'}")
    if args.json_out is not None:
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        args.json_out.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
        print(f"json={args.json_out}")
    return 0 if report["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
