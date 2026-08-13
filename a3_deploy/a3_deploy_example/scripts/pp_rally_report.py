#!/usr/bin/env python3
"""Deep-dive analyzer for the Gate-3 rally rehearsal actor-observation CSV.

Reads the runner's --obs-csv capture (and optionally the conductor's report JSON)
and prints a per-swing table + health checks, so a failed/odd rally can be
debugged LOCALLY from numbers instead of re-running with a viewer.

110-D obs layout (pp_obs_builder.hpp build_obs_110 / LogFirstTick blks110):
  [0:3]    base_ang_vel        [3:34]   joint_pos_rel     [34:65]  joint_vel
  [65:96]  last_action         [96:99]  projected_gravity [99:101] base_forward_xy
  [101:103] base_target_delta_xy (world station - base)
  [103:106] racket_target_rel_base (world)   [106:109] racket_target_vel_w
  [109]    time_to_strike

V15 keeps the position-receipt localization state and appends the finite-gait
command that is generated identically in training and in the runner:
  [110:112] filtered base_xy_velocity from table-relative mocap positions
  [112]     normalized local receipt age
  [113]     desired lateral velocity
  [114:116] left/right gait clocks
  [116]     locomotion mode (+1 STEP, 0 STAND, -1 strike/recovery)
  [117]     upper-body intervention indicator (always 0 on deployment)

Swing segmentation: at idle the runner pins tts at the selected clip's windup
maximum; during a swing tts DECREASES every tick down to the clip-end clamp.
A swing = a maximal run of strictly-decreasing tts spanning >0.5 s.

Usage: python3 pp_rally_report.py /tmp/pp_obs.csv [/tmp/pp_rally_report.json]
                                   [--mode legacy|rally_final_v3|rally_v8|rally_v9|rally_v10|rally_v11|rally_v12|rally_v13|rally_v14|rally_v15|rally_v17|rally_v17_r10|auto]
"""
import argparse
import csv
import json
import math
from pathlib import Path
import re
import sys

B = {"ang_vel": (0, 3), "jpos": (3, 34), "jvel": (34, 65), "act": (65, 96),
     "grav": (96, 99), "fwd": (99, 101), "dstation": (101, 103),
     "rkt_rel": (103, 106), "rkt_vel": (106, 109), "tts": 109,
     "base_vel": (110, 112), "loc_age": 112, "gait_vy": 113,
     "gait_clock": (114, 116), "locomotion_mode": 116, "upper_intervention": 117}
CONTROL_DT = 0.02  # runner policy rate = 50 Hz; CSV `ts` is motion frame, not wall seconds
JOINT_NAMES = (
    "left_hip_pitch_joint", "right_hip_pitch_joint", "waist_yaw_joint",
    "left_hip_roll_joint", "right_hip_roll_joint", "waist_roll_joint",
    "left_hip_yaw_joint", "right_hip_yaw_joint", "waist_pitch_joint",
    "left_knee_joint", "right_knee_joint", "head_yaw_joint",
    "left_shoulder_pitch_joint", "right_shoulder_pitch_joint",
    "left_ankle_pitch_joint", "right_ankle_pitch_joint", "head_pitch_joint",
    "left_shoulder_roll_joint", "right_shoulder_roll_joint",
    "left_ankle_roll_joint", "right_ankle_roll_joint",
    "left_shoulder_yaw_joint", "right_shoulder_yaw_joint", "left_elbow_joint",
    "right_elbow_joint", "left_wrist_roll_joint", "right_wrist_roll_joint",
    "left_wrist_pitch_joint", "right_wrist_pitch_joint", "left_wrist_yaw_joint",
    "right_wrist_yaw_joint",
)

# Keep the historical report windows bit-for-bit as ``legacy``.  FinalV3 deliberately
# lengthens the station-settle and recovery terms; its deploy report must measure the
# same windows as the task instead of silently certifying only the shorter V2 slice.
WINDOWS = {
    "legacy": {
        "pre": (0.12, 0.45),
        "post": (0.20, 1.00),
        "ready_heading": (0.45, 1.40),
        "post_heading": (0.35, 1.80),
    },
    "rally_final_v3": {
        "pre": (0.12, 1.10),
        "post": (0.20, 1.55),
        "ready_heading": (0.45, 1.10),
        "post_heading": (0.20, 1.55),
    },
    # RallyV8 (v13 FACEFIX clips, 2026-07-13): windup fh 0.82 s / bh 0.96 s pins the hold tts
    # (pre t_hi = the MIN windup, so the fh/bh pre-strike means stay comparable; ready_heading
    # t_hi = the MAX windup so the check spans the LATER-arming bh too). The post-swing x-drift
    # terms close at t_hi 1.2 -> mirror them in the windows.
    # A window shorter than the windup would sample the swing itself as "pre-strike" and the
    # base-speed HARD checks would fail a healthy policy.
    "rally_v8": {
        "pre": (0.12, 0.82),
        "post": (0.20, 1.20),
        "ready_heading": (0.45, 0.96),
        "post_heading": (0.20, 1.20),
    },
    "rally_v9": {
        "pre": (0.12, 0.96),
        "post": (0.20, 1.20),
        "ready_heading": (0.45, 1.00),
        "post_heading": (0.20, 1.20),
    },
    "rally_v10": {
        "pre": (0.12, 1.10),
        "post": (0.20, 1.20),
        "ready_heading": (0.45, 1.00),
        "post_heading": (0.20, 1.20),
    },
    "rally_v11": {
        "pre": (0.12, 1.10),
        "post": (0.20, 1.20),
        "ready_heading": (0.45, 1.00),
        # V11 extends the heading reward and deterministic telemetry through 1.55 s.
        "post_heading": (0.20, 1.55),
    },
    "rally_v12": {
        "pre": (0.12, 1.10),
        "post": (0.20, 1.20),
        "ready_heading": (0.45, 1.00),
        "post_heading": (0.20, 1.55),
    },
    "rally_v13": {
        "pre": (0.12, 1.10),
        # V13's explicit post-swing x-lock remains live through 1.55 s.
        "post": (0.10, 1.55),
        "ready_heading": (0.45, 1.00),
        "post_heading": (0.20, 1.55),
    },
    "rally_v14": {
        "pre": (0.12, 1.10),
        "post": (0.10, 1.55),
        "ready_heading": (0.45, 1.00),
        "post_heading": (0.20, 1.55),
    },
    "rally_v15": {
        "pre": (0.12, 1.10),
        "post": (0.10, 1.55),
        "ready_heading": (0.45, 1.00),
        "post_heading": (0.20, 1.55),
    },
    # V17 restores the V11 action/deploy contract and extends recovery supervision through
    # tts=-1.55 s. Score the full trained recovery interval rather than V11's shorter
    # generic post-speed slice.
    "rally_v17": {
        "pre": (0.12, 1.10),
        "post": (0.10, 1.55),
        "ready_heading": (0.45, 1.00),
        "post_heading": (0.20, 1.55),
    },
    "rally_v17_r10": {
        "pre": (0.12, 1.10),
        "post": (0.10, 1.55),
        "ready_heading": (0.45, 1.00),
        "post_heading": (0.20, 1.55),
    },
}


def contact_window_indices(tts, start, end, strike_tick, half_width=0.03):
    """Return contact-window ticks, falling back to the closest-to-strike sample.

    Runner CSVs are normally sampled at 50 Hz, but a sparse capture can skip the closed
    ``|tts| <= half_width`` window entirely.  The already-selected minimum-|tts| strike tick is
    the only defensible fallback; returning an empty list would turn the elbow percentile into a
    NaN and falsely fail an otherwise readable swing.
    """
    indices = [tick for tick in range(start, end + 1) if abs(tts[tick]) <= half_width]
    return indices if indices else [strike_tick]


def _discover_mode_from_runner(runner_log, runner_cwd):
    """Return the report mode proven by the runner/ONNX, or ``None``.

    A new FinalV3 runner emits an exact recipe marker after it has validated the
    model.  The metadata fallback keeps existing V2 packages auto-detectable: the
    model path comes from the runner itself, never from its filename.
    """
    if not runner_log:
        return None
    try:
        log_text = Path(runner_log).read_text(encoding="utf-8", errors="replace")
    except OSError:
        return None
    if "[pp] hitter_pure training_recipe=rally_final_v3" in log_text:
        return "rally_final_v3"
    if "[pp] hitter_pure training_recipe=rally_v8" in log_text:
        return "rally_v8"
    if "[pp] hitter_pure training_recipe=rally_v9" in log_text:
        return "rally_v9"
    runtime_v2_proven = "[pp] hitter_pure runtime_contract=rally_final_v2" in log_text
    if "[pp] hitter_pure training_recipe=rally_v10" in log_text:
        return "rally_v10" if runtime_v2_proven else None
    if "[pp] hitter_pure training_recipe=rally_v11" in log_text:
        return "rally_v11" if runtime_v2_proven else None
    if "[pp] hitter_pure training_recipe=rally_v12" in log_text:
        return "rally_v12" if runtime_v2_proven else None
    if "[pp] hitter_pure training_recipe=rally_v13" in log_text:
        return "rally_v13" if runtime_v2_proven else None
    if "[pp] hitter_pure training_recipe=rally_v14" in log_text:
        return "rally_v14" if runtime_v2_proven else None
    if (
        "[v17-r10-gate3] PROFILE ACCEPTED" in log_text
        and "[pp] hitter_pure training_recipe=rally_v17" in log_text
        and "[pp] hitter_pure runtime_contract="
        "rally_v17_fixed_station_ball_clock_v1" in log_text
    ):
        return "rally_v17_r10"
    if "[pp] hitter_pure training_recipe=rally_v17" in log_text:
        return "rally_v17" if runtime_v2_proven else None
    runtime_v15_proven = "[pp] hitter_pure runtime_contract=rally_v15" in log_text
    if "[pp] hitter_pure training_recipe=rally_v15" in log_text:
        return "rally_v15" if runtime_v15_proven else None

    model_matches = re.findall(
        r"^\[pingpong\] A3AimrtBackend initialised; model=(\S+)(?:\s+.*)?$",
        log_text,
        flags=re.MULTILINE,
    )
    unique_models = list(dict.fromkeys(model_matches))
    if len(unique_models) != 1:
        return None
    model_path = Path(unique_models[0])
    if not model_path.is_absolute():
        if not runner_cwd:
            return None
        model_path = Path(runner_cwd) / model_path
    try:
        import onnx
        model = onnx.load(str(model_path), load_external_data=False)
    # Discovery is advisory for an explicit mode and mandatory for auto.  Any ONNX
    # parser/provider failure therefore becomes ``None`` and is handled fail-closed
    # by ``resolve_mode`` instead of leaking a backend-specific traceback.
    except Exception:
        return None
    metadata = {entry.key: entry.value for entry in model.metadata_props}
    recipe = metadata.get("hitter_pure_training_recipe", "").strip()
    version = metadata.get("hitter_pure_training_recipe_version", "").strip()
    known = {
        ("legacy_station_step", "0"): "legacy",
        ("rally_final_v1", "1"): "legacy",
        ("rally_final_v2", "2"): "legacy",
        ("rally_final_v3", "3"): "rally_final_v3",
        ("rally_v8", "4"): "rally_v8",
        ("rally_v9", "5"): "rally_v9",
        ("rally_v10", "7"): "rally_v10",
        ("rally_v11", "1"): "rally_v11",
        ("rally_v12", "1"): "rally_v12",
        ("rally_v13", "1"): "rally_v13",
        ("rally_v14", "1"): "rally_v14",
        ("rally_v17", "4"): "rally_v17",
        ("rally_v15", "3"): "rally_v15",
        ("rally_v15", "4"): "rally_v15",
        # "5" was the retired v4 full-span decode (no checkpoint survives — NOT accepted);
        # "6" is the shipping v5 default-anchored decode.
        ("rally_v15", "6"): "rally_v15",
    }
    # Do not silently classify a future recipe as legacy: its phase windows may
    # differ just as V3 differs from V2.
    discovered = known.get((recipe, version))
    if discovered in (
        "rally_v10", "rally_v11", "rally_v12", "rally_v13", "rally_v14",
        "rally_v17",
    ):
        # An ONNX recipe label alone cannot prove that the running binary implements component
        # velocity gating. Only the v2-capable loader emits this marker after validation.
        if metadata.get("hitter_pure_runtime_contract") != "rally_final_v2":
            return None
        if not runtime_v2_proven:
            return None
        if discovered in (
            "rally_v11", "rally_v12", "rally_v13", "rally_v14", "rally_v17"
        ) and metadata.get("hitter_pure_deployment_status") != \
                "gate3_candidate":
            return None
    if discovered == "rally_v15":
        if metadata.get("hitter_pure_runtime_contract") != "rally_v15":
            return None
        if not runtime_v15_proven:
            return None
        if metadata.get("hitter_pure_deployment_status") != "gate3_candidate":
            return None
    return discovered


def discover_planes_from_runner(runner_log):
    """Per-SIDE trained plane x from the runner's metadata-geometry banner, or ``None``.

    The runner prints the per-clip target centers it resolved from the loaded ONNX
    pos boxes ("[pp] 110 hitter_pure: target centers from ONNX boxes: fh pos=(...)
    bh pos=(...)").  The reach-x expectation must follow the MODEL, not the report
    generation — hardcoded pairs went stale every clip generation (v11 0.64/0.64,
    v12 0.69/0.69, v13 0.65/0.50 PER-SIDE), which is exactly the "report reach
    expectation" deploy TODO from the v13 facefix migration.
    """
    if not runner_log:
        return None
    try:
        log_text = Path(runner_log).read_text(encoding="utf-8", errors="replace")
    except OSError:
        return None
    matches = re.findall(
        r"target centers from ONNX boxes:\s*"
        r"fh pos=\(([+-]?[0-9.]+),[^)]*\).*?bh pos=\(([+-]?[0-9.]+),",
        log_text,
    )
    if not matches:
        return None
    fh, bh = matches[-1]
    return {"fh": float(fh), "bh": float(bh)}


def discover_finite_gait_from_runner(runner_log):
    """Return the V15 gait envelope proven by the validated runner banner.

    The runner emits this banner only after the ONNX loader has checked the metadata stamped
    from the task YAML.  Reading it here keeps Gate3 tied to that contract instead of copying
    another velocity limit into the report script.
    """
    if not runner_log:
        return None
    try:
        log_text = Path(runner_log).read_text(encoding="utf-8", errors="replace")
    except OSError:
        return None
    matches = re.findall(
        r"\[pp\] V15 finite gait from ONNX/YAML: "
        r"freq=([0-9.]+) Hz duty=([0-9.]+) deadband=([0-9.]+) m "
        r"step=([0-9.]+) m cycles<=([0-9]+) \|vy\|<=([0-9.]+) m/s; "
        r"intervention deploy value=([+-]?[0-9.]+)",
        log_text,
    )
    unique = list(dict.fromkeys(matches))
    if len(unique) != 1:
        return None
    frequency, duty, deadband, step, cycles, velocity_max, intervention = unique[0]
    contract = {
        "frequency_hz": float(frequency),
        "duty_factor": float(duty),
        "move_deadband": float(deadband),
        "step_distance": float(step),
        "max_cycles": int(cycles),
        "velocity_max": float(velocity_max),
        "deploy_intervention": float(intervention),
    }
    if not (
        contract["frequency_hz"] > 0.0
        and 0.0 < contract["duty_factor"] < 1.0
        and contract["move_deadband"] >= 0.0
        and contract["step_distance"] > 0.0
        and contract["max_cycles"] >= 1
        and contract["velocity_max"] > 0.0
        and contract["deploy_intervention"] == 0.0
    ):
        return None
    return contract


def discover_joint_defaults_from_runner(runner_log, runner_cwd=None):
    """Absolute joint defaults proved by the runner banner or its exact loaded ONNX."""
    if not runner_log:
        return None
    try:
        log_text = Path(runner_log).read_text(encoding="utf-8", errors="replace")
    except OSError:
        return None
    model_matches = re.findall(
        r"^\[pingpong\] A3AimrtBackend initialised; model=(\S+)(?:\s+.*)?$",
        log_text,
        flags=re.MULTILINE,
    )
    unique_models = list(dict.fromkeys(model_matches))
    if len(unique_models) != 1:
        return None
    elbow_matches = re.findall(
        r"^\[pp\] hitter_pure joint_default right_elbow_joint=([+-]?[0-9.]+)\s*$",
        log_text,
        flags=re.MULTILINE,
    )
    unique_elbows = list(dict.fromkeys(elbow_matches))
    if len(unique_elbows) == 1:
        return {"right_elbow_joint": float(unique_elbows[0])}
    if len(unique_elbows) > 1:
        return None
    model_path = Path(unique_models[0])
    if not model_path.is_absolute():
        if not runner_cwd:
            return None
        model_path = Path(runner_cwd) / model_path
    try:
        import onnx
        model = onnx.load(str(model_path), load_external_data=False)
        metadata = {entry.key: entry.value for entry in model.metadata_props}
        names = metadata["joint_names"].split(",")
        defaults = [float(value) for value in metadata["default_joint_pos"].split(",")]
    except Exception:
        return None
    if len(names) != len(defaults) or set(names) != set(JOINT_NAMES):
        return None
    return dict(zip(names, defaults))


def runner_clamp_stats(runner_log):
    """Read final q_des safe/hard-limit telemetry instead of raw-action magnitude."""
    if not runner_log:
        return None
    try:
        log_text = Path(runner_log).read_text(encoding="utf-8", errors="replace")
    except OSError:
        return None
    samples = [
        int(value) for value in re.findall(r"\[status\].*?\bclamp=(\d+)", log_text)
    ]
    safe_samples = [
        int(value) for value in re.findall(r"\[status\].*?\bsafe=(\d+)", log_text)
    ]
    audit_only_samples = [
        int(value)
        for value in re.findall(r"\[status\].*?\bqdes_audit_only=(\d+)", log_text)
    ]
    warning_count = log_text.count("[pp WARN] q_des clamped to joint limits")
    warned = warning_count > 0
    if not samples and not warned:
        return None
    joint_summary = {}
    audit_lines = re.findall(r"^\[clamp-audit\].*$", log_text, flags=re.MULTILINE)
    if audit_lines:
        for name, hits, ticks, max_viol in re.findall(
            r"\b([A-Za-z0-9_]+_joint)=(\d+)/(\d+)/([0-9.eE+-]+)", audit_lines[-1]
        ):
            joint_summary[name] = {
                "hits": int(hits),
                "ticks": int(ticks),
                "max_viol": float(max_viol),
            }
    return {
        "samples": len(samples),
        "nonzero_samples": sum(value > 0 for value in samples),
        "peak": max(samples, default=0),
        "safe_samples": len(safe_samples),
        "safe_nonzero_samples": sum(value > 0 for value in safe_samples),
        "safe_peak": max(safe_samples, default=0),
        "audit_only": bool(audit_only_samples and all(audit_only_samples)),
        "warned": warned,
        "warning_count": warning_count,
        "joints": joint_summary,
    }


def qdes_projector_trace_stats(trace_path):
    """Read per-policy-tick V15 projector telemetry from the final command trace."""
    if not trace_path:
        return None
    try:
        with open(trace_path, newline="", encoding="utf-8") as stream:
            reader = csv.DictReader(stream)
            required = {
                "mode", "qdes_projector_active", "qdes_projector_rate",
                "qdes_projector_tracking", "qdes_projector_torque",
                "qdes_projector_infeasible", "qdes_projector_max_norm_debt",
            }
            if reader.fieldnames is None or not required.issubset(reader.fieldnames):
                return {"error": "required qdes projector columns are missing"}
            motion = [row for row in reader if row.get("mode") == "3"]
    except (OSError, csv.Error) as exc:
        return {"error": str(exc)}
    if not motion:
        return {"error": "no MOTION rows in runner trace"}
    try:
        counts = {
            name: [int(row[f"qdes_projector_{name}"]) for row in motion]
            for name in ("active", "rate", "tracking", "torque", "infeasible")
        }
        debts = [float(row["qdes_projector_max_norm_debt"]) for row in motion]
    except (KeyError, TypeError, ValueError) as exc:
        return {"error": f"malformed projector telemetry: {exc}"}
    if any(value < 0 or value > 31 for values in counts.values() for value in values) or any(
            not math.isfinite(value) or value < 0.0 for value in debts):
        return {"error": "out-of-range/non-finite projector telemetry"}
    n = len(motion)
    return {
        "rows": n,
        "joint_fractions": {
            name: sum(values) / (31.0 * n) for name, values in counts.items()
        },
        "affected_tick_fractions": {
            name: sum(value > 0 for value in values) / n for name, values in counts.items()
        },
        "infeasible_peak": max(counts["infeasible"], default=0),
        "infeasible_ticks": sum(value > 0 for value in counts["infeasible"]),
        "max_norm_debt": max(debts, default=0.0),
    }


def resolve_mode(requested, runner_log=None, runner_cwd=None):
    """Resolve/verify the metric recipe; auto mode is intentionally fail-closed."""
    discovered = _discover_mode_from_runner(runner_log, runner_cwd)
    if requested == "auto":
        if discovered is None:
            raise ValueError(
                "cannot prove report recipe from runner log/loaded ONNX; set "
                "--mode legacy, rally_final_v3, rally_v8, rally_v9, rally_v10, rally_v11, "
                "rally_v12, rally_v13, rally_v14, rally_v15, rally_v17 or "
                "rally_v17_r10 explicitly"
            )
        return discovered
    if requested in (
        "rally_v10", "rally_v11", "rally_v12", "rally_v13", "rally_v14",
        "rally_v17",
    ) and discovered != requested:
        raise ValueError(
            f"{requested} report requires a runner-validated rally_final_v2 capability marker"
        )
    if requested == "rally_v17_r10" and discovered != requested:
        raise ValueError(
            "rally_v17_r10 report requires the isolated runner-validated "
            "fixed-station ball-clock runtime marker"
        )
    if requested == "rally_v15" and discovered != requested:
        raise ValueError(
            "rally_v15 report requires a runner-validated paired rally_v15 runtime marker"
        )
    if discovered is not None and discovered != requested:
        raise ValueError(
            f"requested report mode {requested!r} contradicts loaded model mode "
            f"{discovered!r}"
        )
    return requested


def engaged_sides_from_report(report):
    """Extract runner-selected sides in chronological engage order.

    The actor observation intentionally has no side channel.  FinalV3 velocity
    boxes may cross ``vy=0``, so the target velocity is not a side oracle.  The
    conductor's runner engage event is the authoritative deploy-side decision.
    """
    errors = []
    rows = report.get("rows")
    if not isinstance(rows, list):
        return [], ["conductor report has no rows list"]
    sides = []
    previous_serve = None
    for index, row in enumerate(rows):
        if not isinstance(row, dict):
            errors.append(f"row {index} is not an object")
            continue
        serve = row.get("serve")
        if not isinstance(serve, int):
            errors.append(f"row {index} has invalid serve index {serve!r}")
        elif previous_serve is not None and serve <= previous_serve:
            errors.append("conductor rows are not in strictly increasing serve order")
        if isinstance(serve, int):
            previous_serve = serve
        engages = row.get("engages")
        if engages is None:  # schema v1 compatibility
            engaged = row.get("engaged")
            engages = [] if engaged is None else [engaged]
        if not isinstance(engages, list):
            errors.append(f"serve {serve!r} engages is not a list")
            continue
        for engaged in engages:
            if not isinstance(engaged, dict):
                errors.append(f"serve {serve!r} engaged event is not an object")
                continue
            side = engaged.get("side")
            if side not in ("forehand", "backhand"):
                errors.append(f"serve {serve!r} has invalid engaged side {side!r}")
                continue
            sides.append("fh" if side == "forehand" else "bh")
    expected = report.get("total_engage_events", report.get("engaged"))
    if not isinstance(expected, int) or expected != len(sides):
        errors.append(
            f"summary engage events={expected!r} but rows contain {len(sides)} valid sides"
        )
    return sides, errors


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("obs_csv", nargs="?", default="/tmp/pp_obs.csv")
    parser.add_argument("conductor_json", nargs="?")
    parser.add_argument(
        "--mode", choices=("legacy", "rally_final_v3", "rally_v8", "rally_v9", "rally_v10", "rally_v11", "rally_v12", "rally_v13", "rally_v14", "rally_v15", "rally_v17", "rally_v17_r10", "auto"), default="auto",
        help="metric windows; auto proves the recipe from --runner-log/loaded ONNX. Default is "
             "AUTO and fail-closed (2026-07-12 audit): the old 'legacy' default silently scored a "
             "rally_v8/v11 trace with the legacy reach 0.51 instead of 0.64 on any bare invocation "
             "without --mode. Pass --runner-log so auto can prove the recipe.",
    )
    parser.add_argument("--runner-log", help="runner log used for fail-closed auto detection")
    parser.add_argument(
        "--runner-cwd", help="working directory used to resolve a relative ONNX path from the log"
    )
    parser.add_argument(
        "--runner-trace", default="/tmp/pp_runner_trace.csv",
        help="runner command trace; mandatory projector evidence for rally_v15",
    )
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    path = args.obs_csv
    try:
        report_mode = resolve_mode(args.mode, args.runner_log, args.runner_cwd)
    except ValueError as exc:
        print(f"FAIL: {exc}")
        return 2
    windows = WINDOWS[report_mode]
    onnx_planes = discover_planes_from_runner(args.runner_log)
    joint_defaults = discover_joint_defaults_from_runner(args.runner_log, args.runner_cwd)
    finite_gait = discover_finite_gait_from_runner(args.runner_log)
    if onnx_planes is not None:
        print(f"reach-x expectation from the loaded ONNX (runner banner): "
              f"fh {onnx_planes['fh']:.2f} / bh {onnx_planes['bh']:.2f}")
    conductor_report = None
    if args.conductor_json:
        try:
            with open(args.conductor_json) as f:
                conductor_report = json.load(f)
        except (OSError, json.JSONDecodeError) as exc:
            print(f"FAIL: cannot read conductor report {args.conductor_json}: {exc}")
            return 2
    rows = []
    expected_n_obs = 118 if report_mode == "rally_v15" else 110
    with open(path) as f:
        rd = csv.reader(f)
        hdr = next(rd)
        n_obs = sum(1 for c in hdr if c.startswith("obs_"))
        if n_obs != expected_n_obs:
            print(f"FAIL: obs CSV has {n_obs} obs columns, not the {expected_n_obs}-D contract")
            sys.exit(1)
        o0 = hdr.index("obs_0")
        for r in rd:
            if len(r) < o0 + expected_n_obs:
                continue
            rows.append({
                "tick": int(r[0]), "ts": float(r[1]), "mode": r[2],
                "sync_miss": int(r[hdr.index("sync_miss")]),
                "o": [float(v) for v in r[o0:o0 + expected_n_obs]],
            })
    if not rows:
        print("FAIL: empty obs CSV")
        sys.exit(1)

    def seg(o, k):
        lo, hi = B[k]
        return o[lo:hi]

    # ---- global health ----
    bad = ok = 0
    checks = []
    nan_ticks = sum(1 for r in rows if any(math.isnan(v) or math.isinf(v) for v in r["o"]))
    checks.append(("no NaN/Inf in obs", nan_ticks == 0, f"{nan_ticks} ticks affected"))
    sm = rows[-1]["sync_miss"]
    checks.append(("sync_miss == 0", sm == 0, f"final sync_miss={sm}"))
    if report_mode in (
        "rally_v10", "rally_v11", "rally_v12", "rally_v13", "rally_v14",
        "rally_v15", "rally_v17", "rally_v17_r10",
    ):
        checks.append((
            "loaded policy bundle exposes joint defaults for V10--V17 elbow gate",
            joint_defaults is not None,
            "available" if joint_defaults is not None else "missing/unreadable",
        ))
        fixed_plane_ok = (
            onnx_planes is not None
            and abs(onnx_planes.get("fh", float("nan")) - 0.58) <= 1.0e-6
            and abs(onnx_planes.get("bh", float("nan")) - 0.58) <= 1.0e-6
        )
        checks.append((
            "loaded policy bundle uses one shared x=0.58 m plane",
            fixed_plane_ok,
            "missing runner geometry banner" if onnx_planes is None else
            f"fh={onnx_planes['fh']:.3f} bh={onnx_planes['bh']:.3f}",
        ))
    if report_mode == "rally_v15":
        checks.append((
            "loaded runner proves the V15 finite-gait ONNX/YAML contract",
            finite_gait is not None,
            "missing/ambiguous runner gait banner" if finite_gait is None else
            f"freq={finite_gait['frequency_hz']:.2f}Hz duty={finite_gait['duty_factor']:.2f} "
            f"step={finite_gait['step_distance']:.2f}m cycles<={finite_gait['max_cycles']} "
            f"|vy|<={finite_gait['velocity_max']:.2f}m/s",
        ))
        loc_ages = [row["o"][B["loc_age"]] for row in rows]
        loc_age_ok = all(math.isfinite(value) and 0.0 <= value < 1.0 for value in loc_ages)
        checks.append((
            "position-receipt localization stays fresh",
            loc_age_ok,
            f"max normalized age={max(loc_ages, default=float('nan')):.3f}",
        ))
        gait_vy = [row["o"][B["gait_vy"]] for row in rows]
        gait_clocks = [value for row in rows for value in seg(row["o"], "gait_clock")]
        gait_modes = [row["o"][B["locomotion_mode"]] for row in rows]
        interventions = [row["o"][B["upper_intervention"]] for row in rows]
        velocity_max = finite_gait["velocity_max"] if finite_gait is not None else -1.0
        deploy_intervention = (
            finite_gait["deploy_intervention"] if finite_gait is not None else float("nan")
        )
        checks.append((
            "V15 finite gait command remains inside the YAML envelope",
            finite_gait is not None
            and all(math.isfinite(v) and abs(v) <= velocity_max + 1e-6 for v in gait_vy)
            and all(math.isfinite(v) and abs(v) <= 1.0 + 1e-6 for v in gait_clocks)
            and all(any(abs(v - allowed) <= 1e-6 for allowed in (-1.0, 0.0, 1.0))
                    for v in gait_modes),
            f"max|vy|={max((abs(v) for v in gait_vy), default=float('nan')):.3f} "
            f"max|clock|={max((abs(v) for v in gait_clocks), default=float('nan')):.3f} "
            f"modes={sorted(set(round(v, 3) for v in gait_modes))}",
        ))
        checks.append((
            "training-only upper intervention is disabled on deploy",
            finite_gait is not None
            and all(abs(value - deploy_intervention) <= 1e-9 for value in interventions),
            f"max|indicator|={max((abs(v) for v in interventions), default=float('nan')):.3g}",
        ))
    gz_bad = sum(1 for r in rows if seg(r["o"], "grav")[2] > -0.7)
    checks.append(("upright (grav_z<-0.7) except transients", gz_bad < 0.02 * len(rows),
                   f"{gz_bad}/{len(rows)} ticks tilted"))
    dmax = max(math.hypot(*seg(r["o"], "dstation")) for r in rows)
    checks.append(("|station delta| <= 0.45 (Final step + readiness bound)", dmax <= 0.45,
                   f"max |dstation|={dmax:.3f} m"))
    action_peaks = [
        max(abs(seg(r["o"], "act")[index]) for r in rows) for index in range(31)
    ]
    action_top = sorted(range(31), key=action_peaks.__getitem__, reverse=True)[:5]
    clamp_stats = runner_clamp_stats(args.runner_log)
    if clamp_stats is not None:
        clamp_ok = (
            not clamp_stats["warned"]
            and clamp_stats["nonzero_samples"] == 0
            and clamp_stats["safe_nonzero_samples"] == 0
        )
        clamp_joints = ",".join(
            f"{name}:{stats['hits']}/{stats['ticks']}@{stats['max_viol']:.4f}rad"
            for name, stats in sorted(clamp_stats["joints"].items())
        )
        checks.append(("policy q_des stays inside safe and hard joint limits", clamp_ok,
                       f"status_nonzero={clamp_stats['nonzero_samples']}/"
                       f"{clamp_stats['samples']} peak={clamp_stats['peak']} "
                       f"safe_nonzero={clamp_stats['safe_nonzero_samples']}/"
                       f"{clamp_stats['safe_samples']} safe_peak={clamp_stats['safe_peak']} "
                       f"audit_only={clamp_stats['audit_only']} "
                       f"async_warning_count={clamp_stats['warning_count']} "
                       f"joints={clamp_joints or 'unavailable'}"))
    projector_stats = qdes_projector_trace_stats(args.runner_trace) if report_mode == "rally_v15" else None
    if report_mode == "rally_v15":
        if projector_stats is None or "error" in projector_stats:
            checks.append((
                "V15 runner trace contains usable qdes projector evidence",
                False,
                "missing" if projector_stats is None else projector_stats["error"],
            ))
        else:
            jf = projector_stats["joint_fractions"]
            tf = projector_stats["affected_tick_fractions"]
            checks.append((
                "qdes projector constraints remain feasible on every MOTION tick",
                projector_stats["infeasible_ticks"] == 0,
                f"infeasible_ticks={projector_stats['infeasible_ticks']}/"
                f"{projector_stats['rows']} peak_joints={projector_stats['infeasible_peak']}; "
                f"active/rate/tracking/torque joint_frac="
                f"{jf['active']:.4f}/{jf['rate']:.4f}/{jf['tracking']:.4f}/{jf['torque']:.4f}; "
                f"affected_tick_frac={tf['active']:.4f}/{tf['rate']:.4f}/"
                f"{tf['tracking']:.4f}/{tf['torque']:.4f}; "
                f"max_norm_debt={projector_stats['max_norm_debt']:.4f}",
            ))
    jvmax = max(max(abs(v) for v in seg(r["o"], "jvel")) for r in rows)
    checks.append(("|joint_vel| < 25 rad/s", jvmax < 25.0, f"max |joint_vel|={jvmax:.1f}"))

    # ---- swing segmentation from the tts channel ----
    tts = [r["o"][B["tts"]] for r in rows]
    swings = []
    i, n = 1, len(rows)
    while i < n:
        if tts[i] < tts[i - 1] - 1e-6:          # decreasing -> in a swing
            j = i
            while j + 1 < n and tts[j + 1] < tts[j] - 1e-6:
                j += 1
            if (rows[j]["tick"] - rows[i - 1]["tick"]) * CONTROL_DT > 0.5:
                swings.append((i - 1, j))
            i = j + 1
        else:
            i += 1
    checks.append(("at least one complete swing segmented", len(swings) > 0,
                   f"swings={len(swings)}"))

    # V9 removed the z-only wrist termination because valid right-hand racket reaches caused
    # false terminations. V10 replaces that blunt guard with a direct left-wrist joint debt.
    # Keep this deploy-visible check on V9 too so the regression cannot be hidden again.
    if report_mode in (
        "rally_v9", "rally_v10", "rally_v11", "rally_v12", "rally_v13",
        "rally_v14", "rally_v15", "rally_v17", "rally_v17_r10",
    ):
        swing_rows = {index for a, b in swings for index in range(a, b + 1)}
        idle_rows = [row for index, row in enumerate(rows) if index not in swing_rows]
        # V15 (2026-07-24): the v13-facefix clips' READY wrist pose sits ~0.32 rad from the zero
        # boot pose (left wrist roll -0.324), and V15's hold supervision anchors the wrists to
        # exactly that clip frame — so the one-time boot->ready transition alone consumes ~0.33
        # of a 0.35 budget and a compliant policy fails the gate by construction.  Budget the
        # anchor offset + noise explicitly for v15; the gate still catches genuine idle creep
        # (v13 failure was 0.364 vs the then-applicable 0.35 with a ~zero anchor).
        idle_wrist_budget = 0.45 if report_mode == "rally_v15" else 0.35
        for joint_name in (
            "left_wrist_roll_joint", "left_wrist_pitch_joint", "left_wrist_yaw_joint"
        ):
            joint_index = JOINT_NAMES.index(joint_name)
            values = [seg(row["o"], "jpos")[joint_index] for row in idle_rows]
            joint_range = max(values) - min(values) if values else float("nan")
            checks.append((
                f"idle {joint_name} range <= {idle_wrist_budget:.2f} rad",
                math.isfinite(joint_range) and joint_range <= idle_wrist_budget,
                f"range={joint_range:.3f} rad over {len(values)} ticks",
            ))

    swing_sides = None
    if conductor_report is not None:
        runner_sides, side_errors = engaged_sides_from_report(conductor_report)
        side_mapping_ok = not side_errors and len(runner_sides) == len(swings)
        side_detail = "; ".join(side_errors) if side_errors else (
            f"segmented={len(swings)} runner_engages={len(runner_sides)}"
        )
        if not side_errors and len(runner_sides) != len(swings):
            side_detail = (
                f"segmented={len(swings)} runner_engages={len(runner_sides)}; "
                "one-to-one chronological mapping is impossible"
            )
        checks.append(("runner engage sides map one-to-one to obs swings",
                       side_mapping_ok, side_detail))
        if side_mapping_ok:
            swing_sides = runner_sides

    print(f"== {path}: {len(rows)} ticks, {len(swings)} swings segmented; "
          f"mode={report_mode} ==")
    print("   raw last_action peaks (diagnostic, not a safety threshold): " + ", ".join(
        f"{JOINT_NAMES[index]}={action_peaks[index]:.2f}" for index in action_top
    ))
    if report_mode == "rally_v8":
        print("   NOTE: the runner holds both head joints physically, but RallyV8 feeds their "
              "raw action slots back to the actor; use q_des clamp telemetry for safety.")
    elif report_mode in (
        "rally_v9", "rally_v10", "rally_v11", "rally_v12", "rally_v13",
        "rally_v14", "rally_v17", "rally_v17_r10",
    ):
        print(f"   NOTE: {report_mode.replace('_', ' ').title().replace(' ', '')} holds both head joints and feeds the applied zero head actions "
              "back to the actor; raw head outputs are penalized but not executed.")
    elif report_mode == "rally_v15":
        print("   NOTE: RallyV15 holds both head joints and feeds normalized actually executed "
              "q_des (after the stateful projector) back to the actor.")
    print("   windows: pre=(%.2f,%.2f] post=(-%.2f,-%.2f) "
          "ready_heading=(%.2f,%.2f) post_heading=(-%.2f,-%.2f)" % (
              windows["pre"][0], windows["pre"][1],
              windows["post"][1], windows["post"][0],
              windows["ready_heading"][0], windows["ready_heading"][1],
              windows["post_heading"][1], windows["post_heading"][0]))
    print("swing  t_start  tts0 side  target_rel_base(xyz)      tgt_vel(xyz)"
          "       |dstation xy| heading  pre_v/p90 post_v/p90 peak|angvel|")
    pre_speeds, post_speeds, pre_p90s, post_p90s = [], [], [], []
    station_y_errors, heading_errors = [], []
    ready_heading_means, ready_heading_maxes, ready_yaw_rates = [], [], []
    post_heading_means, post_heading_maxes, post_yaw_rates = [], [], []
    right_elbow_p90s = []

    def heading_deg_at(t):
        fwd = seg(rows[t]["o"], "fwd")
        return abs(math.degrees(math.atan2(fwd[1], fwd[0])))

    def base_speed_from_station_delta(t):
        if report_mode == "rally_v15":
            return math.hypot(*seg(rows[t]["o"], "base_vel"))
        if t <= 0:
            return float("nan")
        dt = (rows[t]["tick"] - rows[t - 1]["tick"]) * CONTROL_DT
        if dt <= 1e-6:
            return float("nan")
        d0, d1 = seg(rows[t - 1]["o"], "dstation"), seg(rows[t]["o"], "dstation")
        return math.hypot(d1[0] - d0[0], d1[1] - d0[1]) / dt

    def percentile(values, q):
        if not values:
            return float("nan")
        values = sorted(values)
        x = (len(values) - 1) * q
        lo, hi = int(math.floor(x)), int(math.ceil(x))
        return values[lo] if lo == hi else values[lo] + (values[hi] - values[lo]) * (x - lo)

    def mean_or_nan(values):
        return sum(values) / len(values) if values else float("nan")

    for k, (a, b) in enumerate(swings, 1):
        tts0 = tts[a]
        # strike tick = tts closest to 0 inside the swing
        st = min(range(a, b + 1), key=lambda t: abs(tts[t]))
        o = rows[st]["o"]
        rr = seg(o, "rkt_rel")
        rv = seg(o, "rkt_vel")
        # Do not infer side from target velocity: FinalV3 boxes may cross vy=0.
        # With conductor JSON this is the runner's chronological engage decision;
        # without it the actor trace cannot identify the clip side.
        side = swing_sides[k - 1] if swing_sides is not None else "??"
        dstation = seg(o, "dstation")
        ds = math.hypot(*dstation)
        dx, dy = abs(dstation[0]), abs(dstation[1])
        heading = heading_deg_at(st)
        pre_lo, pre_hi = windows["pre"]
        post_lo, post_hi = windows["post"]
        ready_lo, ready_hi = windows["ready_heading"]
        post_heading_lo, post_heading_hi = windows["post_heading"]
        pre = [base_speed_from_station_delta(t) for t in range(a + 1, b + 1)
               if pre_lo < tts[t] <= pre_hi]
        post = [base_speed_from_station_delta(t) for t in range(a + 1, b + 1)
                if -post_hi < tts[t] < -post_lo]
        pre = [v for v in pre if math.isfinite(v)]
        post = [v for v in post if math.isfinite(v)]
        ready_idx = [t for t in range(a, b + 1) if ready_lo < tts[t] < ready_hi]
        post_heading_idx = [
            t for t in range(a, b + 1)
            if -post_heading_hi < tts[t] < -post_heading_lo
        ]
        ready_h = [heading_deg_at(t) for t in ready_idx]
        post_h = [heading_deg_at(t) for t in post_heading_idx]
        ready_wz = [abs(seg(rows[t]["o"], "ang_vel")[2]) for t in ready_idx]
        post_wz = [abs(seg(rows[t]["o"], "ang_vel")[2]) for t in post_heading_idx]
        ready_h_mean = mean_or_nan(ready_h)
        post_h_mean = mean_or_nan(post_h)
        ready_h_max = max(ready_h) if ready_h else float("nan")
        post_h_max = max(post_h) if post_h else float("nan")
        ready_wz_mean = sum(ready_wz) / len(ready_wz) if ready_wz else float("nan")
        post_wz_mean = sum(post_wz) / len(post_wz) if post_wz else float("nan")
        elbow_p90 = float("nan")
        if report_mode in (
            "rally_v10", "rally_v11", "rally_v12", "rally_v13", "rally_v14",
            "rally_v15", "rally_v17", "rally_v17_r10",
        ) and joint_defaults is not None:
            elbow_index = JOINT_NAMES.index("right_elbow_joint")
            contact_idx = contact_window_indices(tts, a, b, st)
            elbow_values = [
                seg(rows[t]["o"], "jpos")[elbow_index]
                + joint_defaults["right_elbow_joint"]
                for t in contact_idx
            ]
            elbow_p90 = percentile(elbow_values, 0.90)
            if math.isfinite(elbow_p90):
                right_elbow_p90s.append(elbow_p90)
        pre_v = sum(pre) / len(pre) if pre else float("nan")
        post_v = sum(post) / len(post) if post else float("nan")
        pre_p90 = percentile(pre, 0.90)
        post_p90 = percentile(post, 0.90)
        if math.isfinite(pre_v): pre_speeds.append(pre_v)
        if math.isfinite(post_v): post_speeds.append(post_v)
        if math.isfinite(pre_p90): pre_p90s.append(pre_p90)
        if math.isfinite(post_p90): post_p90s.append(post_p90)
        station_y_errors.append(dy)
        heading_errors.append(heading)
        ready_heading_means.append(ready_h_mean)
        ready_heading_maxes.append(ready_h_max)
        ready_yaw_rates.append(ready_wz_mean)
        post_heading_means.append(post_h_mean)
        post_heading_maxes.append(post_h_max)
        post_yaw_rates.append(post_wz_mean)
        pav = max(math.sqrt(sum(v * v for v in seg(rows[t]["o"], "ang_vel")))
                  for t in range(a, b + 1))
        print(f"  {k:2d}  {rows[a]['tick'] * CONTROL_DT:7.1f}  {tts0:4.2f} {side}   "
              f"({rr[0]:+.3f},{rr[1]:+.3f},{rr[2]:+.3f})  "
              f"({rv[0]:+.2f},{rv[1]:+.2f},{rv[2]:+.2f})  {dx:5.3f}/{dy:5.3f} "
              f"{heading:6.1f} {pre_v:5.3f}/{pre_p90:5.3f} "
              f"{post_v:5.3f}/{post_p90:5.3f} {pav:6.2f}")
        print(f"      heading ready mean/max={ready_h_mean:.1f}/{ready_h_max:.1f} deg "
              f"post mean/max={post_h_mean:.1f}/{post_h_max:.1f} deg "
              f"ready/post |wz|={ready_wz_mean:.3f}/{post_wz_mean:.3f} rad/s")
        if report_mode in (
            "rally_v10", "rally_v11", "rally_v12", "rally_v13", "rally_v14",
            "rally_v15", "rally_v17", "rally_v17_r10",
        ) and joint_defaults is not None:
            print(f"      contact right-elbow absolute q p90={elbow_p90:.3f} rad")
        # Contract checks: the actor target should stay on the fixed plane and station should be kept.
        # This CSV does not contain actual racket FK; physical tracking is gated in Isaac/full MuJoCo.
        reach_x = rr[0] - dstation[0]
        # Expected reach = the loaded MODEL's per-side plane (runner metadata banner,
        # see discover_planes_from_runner). V10 is a strict shared 0.58 m plane; V8/V9
        # retain their historical per-side geometry. Mode constants are legacy-log fallbacks.
        if onnx_planes is not None and side in onnx_planes:
            expected_reach_x = onnx_planes[side]
        elif onnx_planes:
            # Side unknown (swing_sides missing / "??"): the model's planes are still authoritative.
            # v13's two planes are 0.15 m apart vs the +/-0.01 tolerance, so the NEAREST one is
            # unambiguous — and an off-plane strike still fails. A mode constant here would reject
            # a correct v13 export (both planes differ from the v12-era 0.69).
            expected_reach_x = min(onnx_planes.values(), key=lambda p: abs(reach_x - p))
        elif report_mode in (
            "rally_v8", "rally_v9", "rally_v10", "rally_v11", "rally_v12",
            "rally_v13", "rally_v14", "rally_v15", "rally_v17", "rally_v17_r10",
        ):
            expected_reach_x = (
                0.58 if report_mode in (
                    "rally_v10", "rally_v11", "rally_v12", "rally_v13",
                    "rally_v14", "rally_v15", "rally_v17", "rally_v17_r10",
                )
                else (0.65 if side == "fh" else 0.50)
            )
        elif report_mode == "rally_final_v3":
            expected_reach_x = 0.70
        else:
            expected_reach_x = 0.51
        reach_x_lo, reach_x_hi = expected_reach_x - 0.01, expected_reach_x + 0.01
        checks.append((
            f"swing {k}: baked target reach-x in [{reach_x_lo:.2f},{reach_x_hi:.2f}]",
            reach_x_lo <= reach_x <= reach_x_hi,
            f"{reach_x:+.3f}",
        ))
        station_x_limit = (
            0.10 if report_mode == "rally_v17_r10" else
            (0.03 if report_mode in (
                "rally_v10", "rally_v11", "rally_v12", "rally_v13",
                "rally_v14", "rally_v15", "rally_v17",
            ) else 0.10)
        )
        checks.append((f"swing {k}: |station_x error| at strike <= {station_x_limit:.2f}",
                       dx <= station_x_limit, f"{dx:.3f}"))
        checks.append((f"swing {k}: |station_y error| at strike <= 0.10",
                       dy <= 0.10, f"{dy:.3f}"))
        checks.append((f"swing {k}: heading at strike <= 15 deg",
                       heading <= 15.0, f"{heading:.1f}"))
        checks.append((f"swing {k}: ready heading mean/max <= 10/15 deg",
                       math.isfinite(ready_h_mean) and math.isfinite(ready_h_max)
                       and ready_h_mean <= 10.0 and ready_h_max <= 15.0,
                       f"{ready_h_mean:.1f}/{ready_h_max:.1f}"))
        checks.append((f"swing {k}: post heading mean/max <= 10/15 deg",
                       math.isfinite(post_h_mean) and math.isfinite(post_h_max)
                       and post_h_mean <= 10.0 and post_h_max <= 15.0,
                       f"{post_h_mean:.1f}/{post_h_max:.1f}"))
        checks.append((f"swing {k}: ready/post |yaw-rate| <= 0.20 rad/s",
                       math.isfinite(ready_wz_mean) and math.isfinite(post_wz_mean)
                       and ready_wz_mean <= 0.20 and post_wz_mean <= 0.20,
                       f"{ready_wz_mean:.3f}/{post_wz_mean:.3f}"))
        checks.append((f"swing {k}: pre-strike base speed <= 0.20 m/s",
                       math.isfinite(pre_v) and pre_v <= 0.20, f"{pre_v:.3f}"))
        checks.append((f"swing {k}: post-strike base speed <= 0.25 m/s",
                       math.isfinite(post_v) and post_v <= 0.25, f"{post_v:.3f}"))
        checks.append((f"swing {k}: pre-strike base-speed p90 <= 0.30 m/s",
                       math.isfinite(pre_p90) and pre_p90 <= 0.30, f"{pre_p90:.3f}"))
        checks.append((f"swing {k}: post-strike base-speed p90 <= 0.35 m/s",
                       math.isfinite(post_p90) and post_p90 <= 0.35, f"{post_p90:.3f}"))
        if report_mode in (
            "rally_v10", "rally_v11", "rally_v12", "rally_v13", "rally_v14",
            "rally_v15", "rally_v17", "rally_v17_r10",
        ) and joint_defaults is not None:
            checks.append((
                f"swing {k}: contact right-elbow q p90 <= 1.35 rad",
                math.isfinite(elbow_p90) and elbow_p90 <= 1.35,
                f"{elbow_p90:.3f}",
            ))

    speed_source = ("V15 filtered position-receipt velocity" if report_mode == "rally_v15"
                    else "station-delta finite difference at the 50 Hz control tick")
    print(f"\n== rally phase metrics ({speed_source}) ==")
    print(f"  station_y_error@strike mean={mean_or_nan(station_y_errors):.3f} m")
    print(f"  heading_error@strike mean={mean_or_nan(heading_errors):.1f} deg")
    print(f"  ready_heading mean/max={mean_or_nan(ready_heading_means):.1f}/"
          f"{max(ready_heading_maxes, default=float('nan')):.1f} deg")
    print(f"  post_heading mean/max={mean_or_nan(post_heading_means):.1f}/"
          f"{max(post_heading_maxes, default=float('nan')):.1f} deg")
    print(f"  ready/post |yaw-rate| mean={mean_or_nan(ready_yaw_rates):.3f}/"
          f"{mean_or_nan(post_yaw_rates):.3f} rad/s")
    print(f"  pre_strike_base_speed mean={mean_or_nan(pre_speeds):.3f} m/s")
    print(f"  post_strike_base_speed mean={mean_or_nan(post_speeds):.3f} m/s")
    print(f"  pre/post base_speed p90 mean={mean_or_nan(pre_p90s):.3f}/"
          f"{mean_or_nan(post_p90s):.3f} m/s")
    if report_mode in (
        "rally_v10", "rally_v11", "rally_v12", "rally_v13", "rally_v14",
        "rally_v15", "rally_v17", "rally_v17_r10",
    ) and joint_defaults is not None:
        print(f"  contact right-elbow q p90 mean/max={mean_or_nan(right_elbow_p90s):.3f}/"
              f"{max(right_elbow_p90s, default=float('nan')):.3f} rad")
    print(f"  NOTE: {expected_n_obs}-D obs CSV has no torso pose, foot body velocity, or left-arm FK; "
          "nor actual racket FK. Those tracking/safety gates come from Isaac and "
          "mujoco_eval_onnx.py full-state metrics.")

    print("\n== checks ==")
    for name, passed, detail in checks:
        print(f"  {'PASS' if passed else 'FAIL'}  {name}  ({detail})")
        ok, bad = ok + passed, bad + (not passed)

    if conductor_report is not None:
        rep = conductor_report
        proxy_completed = int(rep.get("completed_recovered_proxy", rep.get("returned", 0)))
        proxy_rate = rep.get("proxy_rate", rep.get("return_rate"))
        print(f"\n== conductor report: serves={rep['serves']} proxy_ok={proxy_completed} "
              f"falls={rep['falls']} drift={rep['station_drift_m']}m "
              f"-> {'PASS' if rep['pass'] else 'FAIL'} ==")
        expected_swings = proxy_completed
        trace_coverage = len(swings) >= expected_swings
        checks.append(("obs trace covers every proxy-completed swing", trace_coverage,
                       f"segmented={len(swings)} proxy_ok={expected_swings}"))
        print(f"  {'PASS' if trace_coverage else 'FAIL'}  "
              "obs trace covers every proxy-completed swing "
              f"(segmented={len(swings)} proxy_ok={expected_swings})")
        ok, bad = ok + trace_coverage, bad + (not trace_coverage)
        checks.append(("conductor PASS", bool(rep["pass"]),
                       f"proxy_rate={proxy_rate} station_coverage="
                       f"{rep.get('station_transition_coverage_ok')}"))
        # Add the late check immediately; the common loop above has already run.
        passed = bool(rep["pass"])
        print(f"  {'PASS' if passed else 'FAIL'}  conductor PASS")
        ok, bad = ok + passed, bad + (not passed)
    print(f"\n{'PASS' if bad == 0 else 'FAIL'}: {ok} checks passed, {bad} failed")
    return 0 if bad == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
