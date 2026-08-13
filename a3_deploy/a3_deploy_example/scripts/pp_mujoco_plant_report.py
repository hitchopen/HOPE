#!/usr/bin/env python3
"""Summarize the synchronized Gate3 MuJoCo plant and runner command traces."""

from __future__ import annotations

import argparse
import csv
import json
import math
import re
import statistics
from pathlib import Path
from typing import NamedTuple


class SmoothnessLimits(NamedTuple):
    """Fail-closed limits for the no-ball policy-native MOTION hold."""

    min_idle_s: float = 15.0
    trim_s: float = 1.0
    max_qdes_step_peak_rad: float = 0.08
    max_qdes_step_rms_rad: float = 0.005
    max_qdes_reversals_hz: float = 8.0
    reversal_step_epsilon_rad: float = 0.001
    reversal_min_step_rms_rad: float = 0.001
    max_tracking_error_rms_rad: float = 0.15
    max_qd_rms_radps: float = 0.50
    max_ctrl_step_rms_ratio: float = 0.03
    max_ctrl_step_peak_ratio: float = 0.20
    max_ctrl_saturation_fraction: float = 0.0
    ctrl_reversal_step_epsilon_ratio: float = 0.005


def _f(row: dict[str, str], name: str, default: float = 0.0) -> float:
    try:
        return float(row.get(name, default))
    except (TypeError, ValueError):
        return default


def _percentile(values: list[float], q: float) -> float:
    if not values:
        return float("nan")
    ordered = sorted(values)
    pos = (len(ordered) - 1) * q
    lo = int(math.floor(pos))
    hi = int(math.ceil(pos))
    if lo == hi:
        return ordered[lo]
    return ordered[lo] * (hi - pos) + ordered[hi] * (pos - lo)


def _rms(values: list[float]) -> float:
    if not values:
        return 0.0
    return math.sqrt(sum(value * value for value in values) / len(values))


def _reversal_rate(steps: list[float], duration_s: float, epsilon: float) -> float:
    signs = [1 if step > 0.0 else -1 for step in steps if abs(step) >= epsilon]
    reversals = sum(a != b for a, b in zip(signs, signs[1:]))
    return reversals / duration_s if duration_s > 0.0 else 0.0


def _worst(by_joint: dict[str, dict[str, float]], metric: str) -> dict[str, float | str]:
    joint, values = max(by_joint.items(), key=lambda item: item[1][metric])
    return {"joint": joint, "value": values[metric]}


def _first_fake_serve_wall_ns(path: Path) -> int:
    pattern = re.compile(r"\[(\d+)(?:\.(\d+))?\].*\bserve\s+1:")
    with path.open(errors="replace") as f:
        for line in f:
            match = pattern.search(line)
            if match:
                fraction = (match.group(2) or "").ljust(9, "0")[:9]
                return int(match.group(1)) * 1_000_000_000 + int(fraction or "0")
    raise ValueError(f"ball log has no timestamped serve 1 marker: {path}")


def _tilt_deg(row: dict[str, str], prefix: str) -> float:
    w, x = _f(row, f"{prefix}_qw", 1.0), _f(row, f"{prefix}_qx")
    y, z = _f(row, f"{prefix}_qy"), _f(row, f"{prefix}_qz")
    sinr = 2.0 * (w * x + y * z)
    cosr = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr, cosr)
    sinp = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(sinp)
    return math.degrees(max(abs(roll), abs(pitch)))


def summarize_plant(path: Path, wall_start_ns: int | None = None,
                    wall_end_ns: int | None = None) -> tuple[dict, list[str]]:
    with path.open(newline="") as f:
        reader = csv.DictReader(f)
        fields = reader.fieldnames or []
        required = {
            "sim_time", "wall_time_ns", "base_z", "base_qw", "base_qx", "base_qy",
            "base_qz", "racket_vx", "racket_vy", "racket_vz", "ctrl_sat_count",
            "max_ctrl_ratio", "left_foot_normal_force", "right_foot_normal_force",
        }
        missing = sorted(required - set(fields))
        if missing:
            raise ValueError(f"plant CSV missing columns: {missing}")

        rows = 0
        total_rows = 0
        first_time = last_time = first_wall = last_wall = None
        pd_modes: set[str] = set()
        integrators: set[int] = set()
        reset_seq_max = 0
        base_z: list[float] = []
        base_tilt: list[float] = []
        base_xy_speed: list[float] = []
        racket_speed: list[float] = []
        left_force: list[float] = []
        right_force: list[float] = []
        foot_slip: list[float] = []
        ctrl_ratio: list[float] = []
        ctrl_sat_rows = 0
        joint_ratio_peak: dict[str, float] = {
            name[len("ctrl_ratio_"):]: 0.0 for name in fields if name.startswith("ctrl_ratio_")
        }
        for row in reader:
            wall = int(_f(row, "wall_time_ns"))
            total_rows += 1
            if wall_start_ns is not None and wall < wall_start_ns:
                continue
            if wall_end_ns is not None and wall > wall_end_ns:
                continue
            rows += 1
            sim_time = _f(row, "sim_time")
            first_time = sim_time if first_time is None else first_time
            last_time = sim_time
            first_wall = wall if first_wall is None else first_wall
            last_wall = wall
            reset_seq_max = max(reset_seq_max, int(_f(row, "reset_seq")))
            pd_modes.add(row.get("pd_mode", "unknown"))
            integrators.add(int(_f(row, "integrator", -1)))
            base_z.append(_f(row, "base_z"))
            base_tilt.append(_tilt_deg(row, "base"))
            base_xy_speed.append(math.hypot(_f(row, "base_vx"), _f(row, "base_vy")))
            racket_speed.append(math.sqrt(sum(_f(row, f"racket_v{a}") ** 2 for a in "xyz")))
            lf, rf = _f(row, "left_foot_normal_force"), _f(row, "right_foot_normal_force")
            left_force.append(lf)
            right_force.append(rf)
            for side, force in (("left", lf), ("right", rf)):
                if force > 10.0:
                    foot_slip.append(math.hypot(_f(row, f"{side}_foot_vx"),
                                                _f(row, f"{side}_foot_vy")))
            ratio = _f(row, "max_ctrl_ratio")
            ctrl_ratio.append(ratio)
            ctrl_sat_rows += int(_f(row, "ctrl_sat_count") > 0.0)
            for joint in joint_ratio_peak:
                joint_ratio_peak[joint] = max(joint_ratio_peak[joint],
                                              _f(row, f"ctrl_ratio_{joint}"))

    if rows < 2:
        raise ValueError(f"plant CSV has only {rows} data row(s)")
    top_ctrl = sorted(joint_ratio_peak.items(), key=lambda kv: kv[1], reverse=True)[:8]
    summary = {
        "rows": rows,
        "total_rows_in_csv": total_rows,
        "wall_filter_start_ns": wall_start_ns,
        "wall_filter_end_ns": wall_end_ns,
        "pd_modes": sorted(pd_modes),
        "integrators": sorted(integrators),
        "reset_count": reset_seq_max,
        "sim_time_first_s": first_time,
        "sim_time_last_s": last_time,
        "wall_time_first_ns": first_wall,
        "wall_time_last_ns": last_wall,
        "base_z_min_m": min(base_z),
        "base_z_max_m": max(base_z),
        "base_tilt_peak_deg": max(base_tilt),
        "base_xy_speed_p95_mps": _percentile(base_xy_speed, 0.95),
        "base_xy_speed_peak_mps": max(base_xy_speed),
        "racket_speed_p95_mps": _percentile(racket_speed, 0.95),
        "racket_speed_peak_mps": max(racket_speed),
        "left_foot_contact_fraction": sum(v > 10.0 for v in left_force) / rows,
        "right_foot_contact_fraction": sum(v > 10.0 for v in right_force) / rows,
        "both_feet_contact_fraction": sum(l > 10.0 and r > 10.0
                                            for l, r in zip(left_force, right_force)) / rows,
        "contact_foot_slip_p95_mps": _percentile(foot_slip, 0.95),
        "contact_foot_slip_peak_mps": max(foot_slip, default=float("nan")),
        "ctrl_saturation_row_fraction": ctrl_sat_rows / rows,
        "ctrl_ratio_p95": _percentile(ctrl_ratio, 0.95),
        "ctrl_ratio_peak": max(ctrl_ratio),
        "top_ctrl_ratio_by_joint": dict(top_ctrl),
    }
    return summary, fields


def summarize_runner(path: Path) -> dict:
    with path.open(newline="") as f:
        reader = csv.DictReader(f)
        fields = reader.fieldnames or []
        if "wall_time_ns" not in fields:
            raise ValueError("runner trace missing wall_time_ns")
        joints = [name[len("des_"):] for name in fields if name.startswith("des_")]
        if not joints:
            raise ValueError("runner trace has no des_* columns")
        rows = 0
        motion_rows = 0
        walls: list[int] = []
        motion_walls: list[int] = []
        swing_walls: list[int] = []
        err_sq = {joint: 0.0 for joint in joints}
        err_peak = {joint: 0.0 for joint in joints}
        qd_peak = {joint: 0.0 for joint in joints}
        for row in reader:
            rows += 1
            walls.append(int(_f(row, "wall_time_ns")))
            mode = int(_f(row, "mode"))
            if mode == 3:
                motion_rows += 1
                motion_walls.append(walls[-1])
                if int(_f(row, "level")) == 1:
                    swing_walls.append(walls[-1])
            for joint in joints:
                err = abs(_f(row, f"des_{joint}") - _f(row, f"q_{joint}"))
                err_sq[joint] += err * err
                err_peak[joint] = max(err_peak[joint], err)
                qd_peak[joint] = max(qd_peak[joint], abs(_f(row, f"qd_{joint}")))
    if rows < 2:
        raise ValueError(f"runner trace has only {rows} data row(s)")
    dt_ms = [(b - a) / 1e6 for a, b in zip(walls, walls[1:]) if b > a]
    top_err = sorted(err_peak.items(), key=lambda kv: kv[1], reverse=True)[:8]
    top_qd = sorted(qd_peak.items(), key=lambda kv: kv[1], reverse=True)[:8]
    return {
        "rows": rows,
        "motion_rows": motion_rows,
        "wall_time_first_ns": walls[0],
        "wall_time_last_ns": walls[-1],
        "motion_wall_time_first_ns": motion_walls[0] if motion_walls else None,
        "motion_wall_time_last_ns": motion_walls[-1] if motion_walls else None,
        "swing_wall_time_first_ns": swing_walls[0] if swing_walls else None,
        "swing_wall_time_last_ns": swing_walls[-1] if swing_walls else None,
        "command_dt_median_ms": statistics.median(dt_ms),
        "command_dt_p95_ms": _percentile(dt_ms, 0.95),
        "tracking_error_rms_max_joint_rad": max(math.sqrt(v / rows) for v in err_sq.values()),
        "top_tracking_error_peak_by_joint_rad": dict(top_err),
        "top_qd_peak_by_joint_radps": dict(top_qd),
    }


def _load_runner_control_samples(path: Path) -> tuple[list[str], list[dict]]:
    with path.open(newline="") as f:
        reader = csv.DictReader(f)
        fields = reader.fieldnames or []
        joints = [name[len("des_"):] for name in fields if name.startswith("des_")]
        if not joints:
            raise ValueError("runner trace has no des_* columns")
        required = {"wall_time_ns", "mode", "level"}
        required.update(f"q_{joint}" for joint in joints)
        required.update(f"qd_{joint}" for joint in joints)
        missing = sorted(required - set(fields))
        if missing:
            raise ValueError(f"runner trace missing control columns: {missing}")
        samples = []
        for row in reader:
            samples.append({
                "wall_time_ns": int(_f(row, "wall_time_ns")),
                "mode": int(_f(row, "mode")),
                "level": int(_f(row, "level")),
                "des": [_f(row, f"des_{joint}") for joint in joints],
                "q": [_f(row, f"q_{joint}") for joint in joints],
                "qd": [_f(row, f"qd_{joint}") for joint in joints],
            })
    if len(samples) < 2:
        raise ValueError(f"runner trace has only {len(samples)} data row(s)")
    return joints, samples


def _runner_window_metrics(samples: list[dict], joints: list[str],
                           reversal_epsilon: float) -> dict:
    duration_s = ((samples[-1]["wall_time_ns"] - samples[0]["wall_time_ns"]) / 1e9
                  if len(samples) >= 2 else 0.0)
    by_joint = {}
    for index, joint in enumerate(joints):
        desired = [sample["des"][index] for sample in samples]
        steps = [b - a for a, b in zip(desired, desired[1:])]
        tracking_errors = [sample["des"][index] - sample["q"][index]
                           for sample in samples]
        velocities = [sample["qd"][index] for sample in samples]
        by_joint[joint] = {
            "qdes_step_peak_rad": max((abs(step) for step in steps), default=0.0),
            "qdes_step_rms_rad": _rms(steps),
            "qdes_reversals_hz": _reversal_rate(
                steps, duration_s, reversal_epsilon),
            "tracking_error_rms_rad": _rms(tracking_errors),
            "qd_rms_radps": _rms(velocities),
        }
    return {
        "rows": len(samples),
        "duration_s": duration_s,
        "wall_time_first_ns": samples[0]["wall_time_ns"] if samples else None,
        "wall_time_last_ns": samples[-1]["wall_time_ns"] if samples else None,
        "by_joint": by_joint,
        "worst": {
            metric: _worst(by_joint, metric)
            for metric in (
                "qdes_step_peak_rad",
                "qdes_step_rms_rad",
                "qdes_reversals_hz",
                "tracking_error_rms_rad",
                "qd_rms_radps",
            )
        } if by_joint else {},
    }


def _plant_effort_metrics(path: Path, wall_start_ns: int,
                          wall_end_ns: int, reversal_epsilon: float) -> dict:
    with path.open(newline="") as f:
        reader = csv.DictReader(f)
        fields = reader.fieldnames or []
        joints = [name[len("ctrl_ratio_"):] for name in fields
                  if name.startswith("ctrl_ratio_")
                  and f"ctrl_{name[len('ctrl_ratio_') :]}" in fields]
        if not joints:
            raise ValueError("plant CSV has no matched ctrl_*/ctrl_ratio_* columns")
        signed_ratios = {joint: [] for joint in joints}
        walls = []
        saturated_rows = 0
        for row in reader:
            wall = int(_f(row, "wall_time_ns"))
            if wall < wall_start_ns or wall > wall_end_ns:
                continue
            walls.append(wall)
            saturated_rows += int(_f(row, "ctrl_sat_count") > 0.0)
            for joint in joints:
                ctrl = _f(row, f"ctrl_{joint}")
                ratio = _f(row, f"ctrl_ratio_{joint}")
                signed_ratios[joint].append(math.copysign(ratio, ctrl) if ctrl else 0.0)
    if len(walls) < 2:
        raise ValueError(f"plant CSV has only {len(walls)} row(s) in steady idle window")
    duration_s = (walls[-1] - walls[0]) / 1e9
    by_joint = {}
    for joint, values in signed_ratios.items():
        steps = [b - a for a, b in zip(values, values[1:])]
        by_joint[joint] = {
            "ctrl_step_peak_ratio": max((abs(step) for step in steps), default=0.0),
            "ctrl_step_rms_ratio": _rms(steps),
            "ctrl_reversals_hz": _reversal_rate(steps, duration_s, reversal_epsilon),
        }
    return {
        "rows": len(walls),
        "duration_s": duration_s,
        "wall_time_first_ns": walls[0],
        "wall_time_last_ns": walls[-1],
        "ctrl_saturation_row_fraction": saturated_rows / len(walls),
        "by_joint": by_joint,
        "worst": {
            metric: _worst(by_joint, metric)
            for metric in (
                "ctrl_step_peak_ratio", "ctrl_step_rms_ratio", "ctrl_reversals_hz")
        },
    }


def evaluate_idle_smoothness(plant_path: Path, runner_path: Path,
                             limits: SmoothnessLimits | None = None,
                             ball_log_path: Path | None = None) -> dict:
    """Measure policy control chatter after `m` and before the first fake-ball serve."""
    limits = limits or SmoothnessLimits()
    joints, samples = _load_runner_control_samples(runner_path)
    failures: list[str] = []

    motion_index = next((i for i, sample in enumerate(samples)
                         if sample["mode"] == 3), None)
    if motion_index is None:
        return {
            "pass": False,
            "failures": ["runner trace has no MOTION (mode=3) rows"],
            "limits": limits._asdict(),
        }
    swing_index = next((i for i in range(motion_index, len(samples))
                        if samples[i]["mode"] == 3 and samples[i]["level"] == 1), None)
    first_ball_wall_ns = None
    if ball_log_path is not None:
        try:
            first_ball_wall_ns = _first_fake_serve_wall_ns(ball_log_path)
        except (OSError, ValueError) as exc:
            failures.append(f"no-ball boundary unavailable: {exc}")
    boundary_indices = [index for index in (swing_index,) if index is not None]
    if first_ball_wall_ns is not None:
        boundary_indices.append(next(
            (i for i in range(motion_index, len(samples))
             if samples[i]["wall_time_ns"] >= first_ball_wall_ns), len(samples)))
    stop_index = min(boundary_indices, default=len(samples))
    idle = [sample for sample in samples[motion_index:stop_index]
            if sample["mode"] == 3 and sample["level"] == 0]
    if len(idle) < 2:
        return {
            "pass": False,
            "failures": [f"runner trace has only {len(idle)} no-ball MOTION row(s)"],
            "limits": limits._asdict(),
        }

    idle_duration_s = (idle[-1]["wall_time_ns"] - idle[0]["wall_time_ns"]) / 1e9
    if idle_duration_s < limits.min_idle_s:
        failures.append(
            f"no-ball MOTION duration {idle_duration_s:.3f}s < {limits.min_idle_s:.3f}s")

    # Include the last STAND command so a discontinuity caused by pressing `m` is visible.
    entry = idle
    if motion_index > 0:
        entry = [samples[motion_index - 1], *idle]
    entry_metrics = _runner_window_metrics(
        entry, joints, limits.reversal_step_epsilon_rad)

    steady_start_ns = idle[0]["wall_time_ns"] + int(limits.trim_s * 1e9)
    steady = [sample for sample in idle if sample["wall_time_ns"] >= steady_start_ns]
    if len(steady) < 2:
        failures.append(
            f"only {len(steady)} runner row(s) remain after {limits.trim_s:.3f}s trim")
        steady = idle[-2:]
    steady_metrics = _runner_window_metrics(
        steady, joints, limits.reversal_step_epsilon_rad)

    stand = [sample for sample in samples[:motion_index]
             if sample["mode"] == 1
             and sample["wall_time_ns"] >= idle[0]["wall_time_ns"] - 5_000_000_000]
    stand_reference = (_runner_window_metrics(
        stand, joints, limits.reversal_step_epsilon_rad) if len(stand) >= 2 else None)

    entry_peak = entry_metrics["worst"]["qdes_step_peak_rad"]
    if entry_peak["value"] > limits.max_qdes_step_peak_rad:
        failures.append(
            "m-entry q_des step peak "
            f"{entry_peak['value']:.6f}rad ({entry_peak['joint']}) > "
            f"{limits.max_qdes_step_peak_rad:.6f}rad")

    step_rms = steady_metrics["worst"]["qdes_step_rms_rad"]
    if step_rms["value"] > limits.max_qdes_step_rms_rad:
        failures.append(
            "steady q_des step RMS "
            f"{step_rms['value']:.6f}rad ({step_rms['joint']}) > "
            f"{limits.max_qdes_step_rms_rad:.6f}rad")

    for joint, metrics in steady_metrics["by_joint"].items():
        if (metrics["qdes_step_rms_rad"] >= limits.reversal_min_step_rms_rad
                and metrics["qdes_reversals_hz"] > limits.max_qdes_reversals_hz):
            failures.append(
                f"steady q_des reversals {metrics['qdes_reversals_hz']:.3f}Hz "
                f"({joint}) > {limits.max_qdes_reversals_hz:.3f}Hz")
            break

    tracking = steady_metrics["worst"]["tracking_error_rms_rad"]
    if tracking["value"] > limits.max_tracking_error_rms_rad:
        failures.append(
            "steady tracking-error RMS "
            f"{tracking['value']:.6f}rad ({tracking['joint']}) > "
            f"{limits.max_tracking_error_rms_rad:.6f}rad")

    qd = steady_metrics["worst"]["qd_rms_radps"]
    if qd["value"] > limits.max_qd_rms_radps:
        failures.append(
            f"steady qd RMS {qd['value']:.6f}rad/s ({qd['joint']}) > "
            f"{limits.max_qd_rms_radps:.6f}rad/s")

    plant_effort = None
    try:
        plant_effort = _plant_effort_metrics(
            plant_path, steady[0]["wall_time_ns"], idle[-1]["wall_time_ns"],
            limits.ctrl_reversal_step_epsilon_ratio)
    except (OSError, ValueError) as exc:
        failures.append(f"steady idle actuator-effort evidence unavailable: {exc}")
    if plant_effort is not None:
        ctrl_rms = plant_effort["worst"]["ctrl_step_rms_ratio"]
        if ctrl_rms["value"] > limits.max_ctrl_step_rms_ratio:
            failures.append(
                "steady actuator-effort step RMS "
                f"{ctrl_rms['value']:.6f} ({ctrl_rms['joint']}) > "
                f"{limits.max_ctrl_step_rms_ratio:.6f} of torque limit")
        ctrl_peak = plant_effort["worst"]["ctrl_step_peak_ratio"]
        if ctrl_peak["value"] > limits.max_ctrl_step_peak_ratio:
            failures.append(
                "steady actuator-effort step peak "
                f"{ctrl_peak['value']:.6f} ({ctrl_peak['joint']}) > "
                f"{limits.max_ctrl_step_peak_ratio:.6f} of torque limit")
        saturation = plant_effort["ctrl_saturation_row_fraction"]
        if saturation > limits.max_ctrl_saturation_fraction:
            failures.append(
                f"steady actuator saturation fraction {saturation:.6f} > "
                f"{limits.max_ctrl_saturation_fraction:.6f}")

    return {
        "pass": not failures,
        "failures": failures,
        "definition": ("mode=3, level=0, before the first fake-ball serve"
                       if first_ball_wall_ns is not None
                       else "mode=3, level=0, before first mode=3 level=1 swing"),
        "limits": limits._asdict(),
        "idle_window": {
            "rows": len(idle),
            "duration_s": idle_duration_s,
            "wall_time_first_ns": idle[0]["wall_time_ns"],
            "wall_time_last_ns": idle[-1]["wall_time_ns"],
            "first_fake_serve_wall_time_ns": first_ball_wall_ns,
            "first_swing_wall_time_ns": (
                samples[swing_index]["wall_time_ns"] if swing_index is not None else None),
        },
        "m_entry": entry_metrics,
        "steady": steady_metrics,
        "stand_reference": stand_reference,
        "plant_effort_steady": plant_effort,
    }


def build_report(plant_path: Path, runner_path: Path,
                 smoothness_limits: SmoothnessLimits | None = None,
                 ball_log_path: Path | None = None) -> dict:
    runner = summarize_runner(runner_path)
    plant, _ = summarize_plant(plant_path)
    motion_start = runner["motion_wall_time_first_ns"]
    motion_end = runner["motion_wall_time_last_ns"]
    plant_motion = None
    if motion_start is not None and motion_end is not None:
        plant_motion, _ = summarize_plant(plant_path, motion_start, motion_end)
    overlap_ns = max(
        0,
        min(plant["wall_time_last_ns"], runner["wall_time_last_ns"])
        - max(plant["wall_time_first_ns"], runner["wall_time_first_ns"]),
    )
    return {
        "plant": plant,
        "plant_motion": plant_motion,
        "runner": runner,
        "wall_time_overlap_s": overlap_ns / 1e9,
        "idle_smoothness": evaluate_idle_smoothness(
            plant_path, runner_path, smoothness_limits, ball_log_path),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--plant", type=Path, default=Path("/tmp/pp_mujoco_plant.csv"))
    parser.add_argument("--runner-trace", type=Path, default=Path("/tmp/pp_runner_trace.csv"))
    parser.add_argument("--json", type=Path, default=Path("/tmp/pp_mujoco_plant_report.json"))
    parser.add_argument("--ball-log", type=Path)
    parser.add_argument("--require-idle-smoothness", action="store_true")
    parser.add_argument("--min-idle-s", type=float, default=15.0)
    parser.add_argument("--idle-trim-s", type=float, default=1.0)
    parser.add_argument("--max-qdes-step-peak-rad", type=float, default=0.08)
    parser.add_argument("--max-qdes-step-rms-rad", type=float, default=0.005)
    parser.add_argument("--max-qdes-reversals-hz", type=float, default=8.0)
    parser.add_argument("--qdes-reversal-step-epsilon-rad", type=float, default=0.001)
    parser.add_argument("--qdes-reversal-min-step-rms-rad", type=float, default=0.001)
    parser.add_argument("--max-tracking-error-rms-rad", type=float, default=0.15)
    parser.add_argument("--max-qd-rms-radps", type=float, default=0.50)
    parser.add_argument("--max-ctrl-step-rms-ratio", type=float, default=0.03)
    parser.add_argument("--max-ctrl-step-peak-ratio", type=float, default=0.20)
    parser.add_argument("--max-ctrl-saturation-fraction", type=float, default=0.0)
    parser.add_argument("--ctrl-reversal-step-epsilon-ratio", type=float, default=0.005)
    args = parser.parse_args()
    limits = SmoothnessLimits(
        min_idle_s=args.min_idle_s,
        trim_s=args.idle_trim_s,
        max_qdes_step_peak_rad=args.max_qdes_step_peak_rad,
        max_qdes_step_rms_rad=args.max_qdes_step_rms_rad,
        max_qdes_reversals_hz=args.max_qdes_reversals_hz,
        reversal_step_epsilon_rad=args.qdes_reversal_step_epsilon_rad,
        reversal_min_step_rms_rad=args.qdes_reversal_min_step_rms_rad,
        max_tracking_error_rms_rad=args.max_tracking_error_rms_rad,
        max_qd_rms_radps=args.max_qd_rms_radps,
        max_ctrl_step_rms_ratio=args.max_ctrl_step_rms_ratio,
        max_ctrl_step_peak_ratio=args.max_ctrl_step_peak_ratio,
        max_ctrl_saturation_fraction=args.max_ctrl_saturation_fraction,
        ctrl_reversal_step_epsilon_ratio=args.ctrl_reversal_step_epsilon_ratio,
    )
    try:
        report = build_report(args.plant, args.runner_trace, limits, args.ball_log)
    except (OSError, ValueError) as exc:
        print(f"[plant-report] FAIL: {exc}")
        return 2
    args.json.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
    p, r = report["plant"], report["runner"]
    pm = report["plant_motion"] or p
    idle = report["idle_smoothness"]
    verdict = "PASS" if idle["pass"] else "FAIL"
    idle_window = idle.get("idle_window", {})
    print(f"[plant-report] idle smoothness {verdict}: no-ball MOTION "
          f"duration={idle_window.get('duration_s', 0.0):.2f}s")
    if idle.get("steady"):
        steady = idle["steady"]["worst"]
        entry = idle["m_entry"]["worst"]
        effort = idle.get("plant_effort_steady")
        print("[plant-report] idle q_des entry_peak="
              f"{entry['qdes_step_peak_rad']['value']:.6f}rad; steady step_rms="
              f"{steady['qdes_step_rms_rad']['value']:.6f}rad reversals="
              f"{steady['qdes_reversals_hz']['value']:.2f}Hz tracking_rms="
              f"{steady['tracking_error_rms_rad']['value']:.6f}rad qd_rms="
              f"{steady['qd_rms_radps']['value']:.6f}rad/s")
        if effort:
            ctrl = effort["worst"]
            print("[plant-report] idle actuator-effort step_rms/peak="
                  f"{ctrl['ctrl_step_rms_ratio']['value']:.6f}/"
                  f"{ctrl['ctrl_step_peak_ratio']['value']:.6f} of torque limit; "
                  f"reversals={ctrl['ctrl_reversals_hz']['value']:.2f}Hz")
    for failure in idle["failures"]:
        print(f"[plant-report] idle FAIL: {failure}")
    print("[plant-report] synchronized MuJoCo/runner evidence captured")
    print(f"[plant-report] rows plant/runner={p['rows']}/{r['rows']} "
          f"overlap={report['wall_time_overlap_s']:.2f}s resets={p['reset_count']} "
          f"pd={p['pd_modes']} integrator={p['integrators']}")
    print(f"[plant-report] MOTION rows={pm['rows']} base z="
          f"[{pm['base_z_min_m']:.3f},{pm['base_z_max_m']:.3f}]m "
          f"tilt_peak={pm['base_tilt_peak_deg']:.1f}deg xy_v_p95/peak="
          f"{pm['base_xy_speed_p95_mps']:.3f}/{pm['base_xy_speed_peak_mps']:.3f}m/s")
    print(f"[plant-report] MOTION racket_v_p95/peak={pm['racket_speed_p95_mps']:.3f}/"
          f"{pm['racket_speed_peak_mps']:.3f}m/s both_feet={pm['both_feet_contact_fraction']:.3f} "
          f"slip_p95={pm['contact_foot_slip_p95_mps']:.3f}m/s")
    print(f"[plant-report] MOTION ctrl_sat_rows={pm['ctrl_saturation_row_fraction']:.4f} "
          f"ctrl_ratio_p95/peak={pm['ctrl_ratio_p95']:.3f}/{pm['ctrl_ratio_peak']:.3f} "
          f"cmd_dt_p95={r['command_dt_p95_ms']:.2f}ms")
    print(f"[plant-report] full-run startup/reset envelope: z="
          f"[{p['base_z_min_m']:.3f},{p['base_z_max_m']:.3f}]m "
          f"tilt_peak={p['base_tilt_peak_deg']:.1f}deg")
    print(f"[plant-report] JSON: {args.json}")
    if args.require_idle_smoothness and not idle["pass"]:
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
