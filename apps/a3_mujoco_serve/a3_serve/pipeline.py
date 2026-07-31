"""End-to-end MuJoCo search, DLS IK, replay, and A3 CSV export."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import numpy as np

from .config import load_config
from .constants import (
    HIGH_LEVEL_ARM_HI,
    HIGH_LEVEL_ARM_LO,
    JOINT_NAMES,
    VALIDATED_FRAME_COUNT,
)
from .csvio import MotionCsv, write_generation_manifest, write_motion_csv
from .mujoco_scene import A3ServeScene, BallReplay
from .physics import search_legal_serve
from .trajectory import build_cartesian_schedule, solve_dls_trajectory


def _json_vector(value: np.ndarray | None) -> list[float] | None:
    if value is None:
        return None
    return [float(item) for item in value]


def _replay_json(replay: BallReplay) -> dict[str, Any]:
    return {
        "racket_contact": replay.racket_contact,
        "net_contact": replay.net_contact,
        "net_clearance_m": replay.net_clearance_m,
        "first_bounce_table": _json_vector(replay.first_bounce_table),
        "second_bounce_table": _json_vector(replay.second_bounce_table),
        "bounces_inside_table": replay.bounces_inside_table,
        "post_contact_velocity_world": _json_vector(
            replay.post_contact_velocity_world
        ),
    }


def _full_replay_legal(replay: BallReplay, physics: dict) -> bool:
    if (
        not replay.racket_contact
        or replay.net_contact
        or not replay.bounces_inside_table
        or replay.first_bounce_table is None
        or replay.second_bounce_table is None
        or replay.net_clearance_m is None
    ):
        return False
    minimum_clearance = float(physics["net"]["height"]) + float(
        physics["ball"]["radius"]
    )
    return bool(
        replay.first_bounce_table[0] < replay.net_x_table
        and replay.second_bounce_table[0] > replay.net_x_table
        and replay.net_clearance_m >= minimum_clearance
    )


def _simulation_reference(template_joints: np.ndarray, ready_frame: int) -> np.ndarray:
    """Match the high-level deployment ownership boundary in MuJoCo.

    Waist, neck and legs are not commanded by the app, so they remain at the
    READY reference during generation.  Both arm groups retain explicit source
    samples because those are the 14 positions sent by the runtime.
    """

    output = np.asarray(template_joints, dtype=np.float64).copy()
    vendor_owned = list(range(0, 5)) + list(range(19, len(JOINT_NAMES)))
    output[:, vendor_owned] = output[ready_frame, vendor_owned]
    return output


def generate(config_path: str | Path, output_directory: str | Path) -> dict[str, Any]:
    config = load_config(config_path)
    config_path = Path(config["_config_path"])
    output_directory = Path(output_directory).expanduser().resolve()
    output_directory.mkdir(parents=True, exist_ok=True)

    template = MotionCsv.load(config["source"]["template_csv"])
    timing = config["timing"]
    physics = config["physics"]
    planner = config["planner"]
    scene = A3ServeScene(config["model"]["xml"], physics, config["model"])
    reference = _simulation_reference(template.joint_radians(), timing["ready_frame"])

    scene.reset(reference[0])
    initial_position, initial_rotation = scene.racket_pose()
    scene.reset(reference[timing["ready_frame"]])
    ready_position, ready_rotation = scene.racket_pose()

    candidate = search_legal_serve(
        scene,
        reference[timing["ready_frame"]],
        ready_position,
        planner,
        physics,
    )
    schedule = build_cartesian_schedule(
        frame_count=timing["frame_count"],
        ready_frame=timing["ready_frame"],
        stroke_start_frame=timing["stroke_start_frame"],
        strike_frame=timing["strike_frame"],
        follow_end_frame=timing["follow_end_frame"],
        return_end_frame=timing["return_end_frame"],
        initial_position=initial_position,
        initial_rotation=initial_rotation,
        ready_position=ready_position,
        ready_rotation=ready_rotation,
        candidate=candidate,
        normal_axis=scene.racket_normal_axis,
    )
    ik = solve_dls_trajectory(
        scene,
        reference,
        schedule,
        config["ik"],
        source_hz=float(timing["source_hz"]),
    )
    if ik.joint_trajectory.shape != (VALIDATED_FRAME_COUNT, len(JOINT_NAMES)):
        raise RuntimeError("internal IK trajectory shape violates the runtime contract")
    arm = ik.joint_trajectory[:, 5:19]
    active_mask = np.ones(14, dtype=bool)
    active_mask[4] = False  # validated runtime holds left_wrist_roll at entry
    if np.any(arm[:, active_mask] < HIGH_LEVEL_ARM_LO[active_mask] - 1.0e-12) or np.any(
        arm[:, active_mask] > HIGH_LEVEL_ARM_HI[active_mask] + 1.0e-12
    ):
        raise RuntimeError("generated active arm trajectory exceeds high-level A3 limits")
    if ik.max_joint_speed_rad_s > float(config["safety"]["max_source_joint_speed_rad_s"]):
        raise RuntimeError(
            "generated right-arm speed exceeds configured source limit: "
            f"{ik.max_joint_speed_rad_s:.6f} rad/s"
        )

    collisions = scene.collision_scan(
        ik.joint_trajectory, stride=int(config["safety"]["collision_scan_stride"])
    )
    if collisions and bool(config["safety"]["fail_on_self_collision"]):
        first = collisions[0]
        raise RuntimeError(
            f"MuJoCo self-collision at frame {first['frame']}: {first['geoms']}"
        )

    preimpact = float(planner["preimpact_seconds"])
    acceleration = np.array([0.0, 0.0, -float(physics["gravity"])])
    impact_velocity = np.asarray(planner["incoming_ball_velocity_world"], dtype=np.float64)
    initial_ball_velocity = impact_velocity - acceleration * preimpact
    initial_ball_position = (
        candidate.ball_contact_position_world
        - impact_velocity * preimpact
        + 0.5 * acceleration * preimpact * preimpact
    )
    stroke_trajectory = ik.joint_trajectory[timing["stroke_start_frame"]:]
    replay = scene.simulate_joint_replay(
        stroke_trajectory,
        initial_ball_position,
        initial_ball_velocity,
        source_hz=float(timing["source_hz"]),
        duration_s=float(planner["flight_duration_s"]),
    )
    replay_legal = _full_replay_legal(replay, physics)
    if bool(config["safety"]["require_full_replay_legal"]) and not replay_legal:
        raise RuntimeError(
            "generated joint replay did not produce a legal MuJoCo serve: "
            + json.dumps(_replay_json(replay), sort_keys=True)
        )

    output_csv = write_motion_csv(
        output_directory / "serve_policy.csv",
        template.with_right_arm(ik.right_arm_trajectory),
    )
    result = {
        "environment": {
            "mujoco_version": str(scene.mj.__version__),
            "model_xml": str(scene.model_xml),
        },
        "search": candidate.json(),
        "ik": {
            "solver": "damped_least_squares",
            "max_position_error_m": ik.max_position_error_m,
            "max_rotation_error_rad": ik.max_rotation_error_rad,
            "max_joint_step_rad": ik.max_joint_step_rad,
            "max_joint_speed_rad_s": ik.max_joint_speed_rad_s,
            "unconverged_frame_count": len(ik.unconverged_frames),
        },
        "collision_scan": {
            "sample_stride": int(config["safety"]["collision_scan_stride"]),
            "contact_count": len(collisions),
            "contacts": collisions[:100],
        },
        "full_joint_replay": {**_replay_json(replay), "legal": replay_legal},
        "ball_initial_state_at_stroke_start": {
            "position_world": _json_vector(initial_ball_position),
            "velocity_world": _json_vector(initial_ball_velocity),
        },
    }
    manifest = write_generation_manifest(
        output_directory / "serve_vendor_arm_manifest.json",
        motion_path=output_csv,
        template_path=template.path,
        model_path=config["model"]["xml"],
        physics_reference_path=physics["source_reference"],
        config_path=config_path,
        result=result,
    )
    report = output_directory / "validation_report.json"
    report.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return {
        "motion_csv": str(output_csv),
        "manifest": str(manifest),
        "validation_report": str(report),
        "result": result,
    }
