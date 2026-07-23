# Copyright (c) 2025, Intelligent Racing Inc. (dba Hitch Interactive).
# SPDX-License-Identifier: Apache-2.0
"""MuJoCo sim-to-sim evaluation of the exported HOPE ONNX policy.

This drives the exported ``hope_pingpong.onnx`` (single layout: observation[1, 111]
-> raw_action[1, 31], no observation normalization) through the SAME 111-D
observation builder, ActionAdapter, swing lifecycle and RacketCommand that the
clean-room reference deploy runner uses, so this is a faithful test of the deploy
contract rather than a second, divergent implementation.

``success_rate`` is measured from an ACTUAL simulated ball. A real ball (free joint,
sphere) is served from the opponent side toward the robot; it flies under gravity +
no-spin aerodynamic drag and really bounces off the racket, table and net inside
MuJoCo. A return succeeds when ALL of the following are observed on the real
trajectory:

  * **contact**   -- an actual MuJoCo contact between the ball geom and the racket
                     collision geom (or the ball passes within the contact radius of
                     the racket site while the racket is moving toward it);
  * **net clear** -- after contact the ball's x crosses the net plane with its centre
                     above the net top; and
  * **opponent-half first bounce** -- the ball's first downward crossing of the table
                     surface (z reaches ball_radius descending) lands on the opponent
                     half, inside the table bounds.

    success_rate = successful_return_tasks / incoming_balls_that_entered_a_strike_task

There is NO analytic predicted-landing substitute: every event above is read off the
real simulated ball. Forehand, backhand and all serves merge into the one number.
Emits only ``{"success_rate": <float>}`` to stdout (and optionally --json-out); a
one-line human summary goes to stderr.

Two evaluation modes (``--eval-mode``):

  * ``continuous`` (default) — deploy-faithful continuous rally: the robot, policy
    ``last_action``, lifecycle and fixed station are initialized ONCE; between serves
    the policy keeps running through its follow-through/recovery with no state reset
    and no teleport, exactly like the deploy runner. The serve side follows the
    pattern FH, FH, BH, BH, ... so all four adjacent transitions (FH->FH, FH->BH,
    BH->BH, BH->FH) are exercised; the transitions actually seen are reported on
    stderr. A fall keeps affecting later serves — that is the honest continuous
    measurement.
  * ``independent`` — independent-strike evaluation: every serve resets the robot to
    its stand with a fresh lifecycle/last_action/station. This measures isolated
    swings only; it does NOT validate between-swing transitions.

Requires ``mujoco``, ``onnxruntime``, ``pyyaml`` and ``numpy`` plus the shipped
reference deploy package (``a3_deploy/a3_deploy_example``) and the
``a3_pingpong`` MJCF. Runs OUTSIDE Isaac.
"""

from __future__ import annotations

import argparse
import csv
import json
import os
import pathlib
import sys

import numpy as np


_ISAAC_A3_CANONICAL_TO_ARTICULATION = np.array(
    [2, 5, 8, 11, 16, 12, 17, 21, 23, 25, 27, 29, 13, 18, 22, 24,
     26, 28, 30, 0, 3, 6, 9, 14, 19, 1, 4, 7, 10, 15, 20],
    dtype=np.int64,
)
_ISAAC_A3_ARTICULATION_TO_CANONICAL = np.empty_like(_ISAAC_A3_CANONICAL_TO_ARTICULATION)
for _canonical_i, _articulation_i in enumerate(_ISAAC_A3_CANONICAL_TO_ARTICULATION):
    _ISAAC_A3_ARTICULATION_TO_CANONICAL[_articulation_i] = _canonical_i


def _obs_for_policy_joint_order(obs: np.ndarray, last_action_canonical: np.ndarray, mode: str) -> np.ndarray:
    """Return the observation layout expected by the ONNX policy for MuJoCo diagnostics.

    Current exported policies should be canonical. Older checkpoints may have been
    trained before the Isaac articulation order was gated; for those, only MuJoCo
    eval can opt into the legacy articulation-order slices without changing the
    training code or the public canonical deploy contract.
    """
    if mode != "isaac-articulation-obs-action":
        return obs
    out = np.asarray(obs, dtype=np.float32).copy()
    art_to_can = _ISAAC_A3_ARTICULATION_TO_CANONICAL
    out[3:34] = obs[3:34][art_to_can]
    out[34:65] = obs[34:65][art_to_can]
    out[65:96] = np.asarray(last_action_canonical, dtype=np.float32)[art_to_can]
    return out


def _raw_action_to_canonical(raw_action: np.ndarray, mode: str) -> np.ndarray:
    """Map policy raw-action columns into the canonical ActionAdapter order."""
    raw = np.asarray(raw_action, dtype=np.float64).reshape(31)
    if mode in ("isaac-articulation-action", "isaac-articulation-obs-action"):
        return raw[_ISAAC_A3_CANONICAL_TO_ARTICULATION].copy()
    return raw.copy()


def _repo_root() -> pathlib.Path:
    here = pathlib.Path(__file__).resolve()
    for parent in here.parents:
        if (parent / "a3_deploy").is_dir() and (parent / "hope_training").is_dir():
            return parent
    return here.parents[3]


def _reference_pkg_dir(repo_root: pathlib.Path) -> pathlib.Path:
    return repo_root / "a3_deploy" / "a3_deploy_example" / "reference"


def _default_runtime_config(repo_root: pathlib.Path) -> pathlib.Path:
    return repo_root / "a3_deploy" / "a3_deploy_example" / "config" / "hope_pingpong_runtime.yaml"


def _load_success_metric(repo_root: pathlib.Path):
    """Import the shared ``success_metric`` module by file path (pure NumPy).

    Importing it as ``whole_body_tracking.utils.success_metric`` would execute the
    training package ``__init__`` (which registers Isaac Lab gym tasks and needs
    gymnasium); this eval only needs the metric's SuccessRate accumulator, the
    TableGeometry.on_opponent_half definition, and the physics-config loader, so we
    load the single file standalone.
    """
    import importlib.util

    path = (
        repo_root / "hope_training" / "whole_body_tracking" / "source" / "whole_body_tracking"
        / "whole_body_tracking" / "utils" / "success_metric.py"
    )
    spec = importlib.util.spec_from_file_location("hope_success_metric", path)
    module = importlib.util.module_from_spec(spec)
    # Register before exec so dataclass introspection (sys.modules[__module__]) works.
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _float_or_none(value):
    if value is None or value == "":
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _load_serve_manifest(path: str | None) -> list[dict]:
    """Load training motion-manifest racket target boxes for MuJoCo serve sampling.

    The target boxes are treated as robot/MuJoCo-world racket intercept positions.
    This is intentionally closer to the current Isaac training distribution than the
    script's neutral example serve distribution.
    """
    if not path:
        return []
    rows: list[dict] = []
    with open(path, "r", encoding="utf-8", newline="") as fh:
        for row in csv.DictReader(fh, delimiter="\t"):
            ranges = []
            ok = True
            for axis in ("x", "y", "z"):
                lo = _float_or_none(row.get(f"racket_pos_{axis}_lo"))
                hi = _float_or_none(row.get(f"racket_pos_{axis}_hi"))
                if lo is None or hi is None:
                    ok = False
                    break
                ranges.append((min(lo, hi), max(lo, hi)))
            if not ok:
                continue
            side = _float_or_none(row.get("swing_side"))
            rows.append(
                {
                    "side": 1.0 if side is None or side >= 0.0 else -1.0,
                    "ranges": tuple(ranges),
                    "source": row.get("output") or row.get("source") or "",
                }
            )
    if not rows:
        raise ValueError(f"serve manifest has no complete racket_pos boxes: {path}")
    return rows


def _new_eval_counts() -> dict:
    return {
        "attempts": 0,
        "successes": 0,
        "contact": 0,
        "real_contact": 0,
        "proximity_contact": 0,
        "net_clear": 0,
        "first_bounce": 0,
        "opponent_bounce": 0,
        "miss_no_contact": 0,
        "miss_contact_no_net": 0,
        "miss_net_no_bounce": 0,
        "miss_wrong_bounce": 0,
        "miss_fell": 0,
        "miss_timeout": 0,
        "nonfinite_action_ticks": 0,
        "min_ball_racket_distance": float("inf"),
        "max_abs_raw_action": 0.0,
        "max_abs_applied_action": 0.0,
        "max_abs_q_des": 0.0,
    }


def _update_max(counts: dict, key: str, value: float) -> None:
    if np.isfinite(value):
        counts[key] = max(float(counts[key]), float(value))


def _finalize_counts(counts: dict) -> dict:
    attempts = int(counts["attempts"])
    out = dict(counts)
    if not np.isfinite(out["min_ball_racket_distance"]):
        out["min_ball_racket_distance"] = None
    for key in (
        "successes",
        "contact",
        "real_contact",
        "proximity_contact",
        "net_clear",
        "first_bounce",
        "opponent_bounce",
        "miss_no_contact",
        "miss_contact_no_net",
        "miss_net_no_bounce",
        "miss_wrong_bounce",
        "miss_fell",
        "miss_timeout",
    ):
        out[f"{key}_rate"] = (float(counts[key]) / attempts) if attempts > 0 else None
    return out


def _norm(v) -> float:
    return float(np.linalg.norm(np.asarray(v, dtype=np.float64)))


def _unit_or_none(v):
    arr = np.asarray(v, dtype=np.float64)
    n = float(np.linalg.norm(arr))
    if n < 1e-12:
        return None
    return arr / n


def _angle_deg(a, b):
    a_u = _unit_or_none(a)
    b_u = _unit_or_none(b)
    if a_u is None or b_u is None:
        return None
    return float(np.degrees(np.arccos(np.clip(float(np.dot(a_u, b_u)), -1.0, 1.0))))


def _xyz(row: dict, prefix: str, value) -> None:
    if value is None:
        row[f"{prefix}_x"] = ""
        row[f"{prefix}_y"] = ""
        row[f"{prefix}_z"] = ""
        return
    arr = np.asarray(value, dtype=np.float64).reshape(3)
    row[f"{prefix}_x"] = float(arr[0])
    row[f"{prefix}_y"] = float(arr[1])
    row[f"{prefix}_z"] = float(arr[2])


def _outgoing_flight_diag(start_pos_table: np.ndarray, ball_vel_w: np.ndarray, physics, table) -> dict:
    """Predict net crossing and first bounce from a post-contact ball state.

    This is only a diagnostic readout. The public MuJoCo eval result still uses the
    simulated ball events observed later in the same trial.
    """
    p = np.asarray(start_pos_table, dtype=np.float64).copy()
    v = np.asarray(ball_vel_w, dtype=np.float64).copy()
    dt = 0.002
    surface_z = physics.ball_radius
    out = {
        "pred_net_cross": 0,
        "pred_net_z": "",
        "pred_net_clear": 0,
        "pred_bounce_x": "",
        "pred_bounce_y": "",
        "pred_opponent_bounce": 0,
    }
    for _ in range(int(3.0 / dt)):
        a = physics.acceleration(v)
        p_new = p + v * dt + 0.5 * a * dt * dt
        v_new = v + a * dt
        if not out["pred_net_cross"] and p[0] < table.net_x <= p_new[0]:
            dx = p_new[0] - p[0]
            frac = (table.net_x - p[0]) / dx if abs(dx) > 1e-12 else 0.5
            net_z = float(p[2] + frac * (p_new[2] - p[2]))
            out["pred_net_cross"] = 1
            out["pred_net_z"] = net_z
            out["pred_net_clear"] = int(net_z > (table.net_height + physics.ball_radius))
        if p[2] > surface_z >= p_new[2] and v[2] < 0.0:
            dz = p_new[2] - p[2]
            frac = (surface_z - p[2]) / dz if abs(dz) > 1e-12 else 0.5
            land_x = float(p[0] + frac * (p_new[0] - p[0]))
            land_y = float(p[1] + frac * (p_new[1] - p[1]))
            out["pred_bounce_x"] = land_x
            out["pred_bounce_y"] = land_y
            out["pred_opponent_bounce"] = int(table.on_opponent_half(land_x, land_y))
            break
        p, v = p_new, v_new
    return out


def _choose_racket_normal(racket_xmat: np.ndarray, to_ball: np.ndarray) -> np.ndarray:
    """Pick the racket geom axis most aligned with the ball-side direction.

    The collision mesh/site convention is model-specific, so diagnostics avoid
    assuming a named local face-normal axis. The chosen normal is always oriented
    from the racket toward the ball at contact.
    """
    axes = [racket_xmat[:, i].copy() for i in range(3)]
    to_ball_u = _unit_or_none(to_ball)
    if to_ball_u is None:
        return axes[0]
    axis = max(axes, key=lambda a: abs(float(np.dot(a, to_ball_u))))
    if float(np.dot(axis, to_ball_u)) < 0.0:
        axis = -axis
    return axis


def _new_contact_diag_row(trial: int, side_name: str, strike_pt, serve_pos, serve_vel) -> dict:
    row = {
        "trial": int(trial),
        "side": side_name,
        "contact_kind": "none",
        "contact_tick": "",
        "contact_substep": "",
        "contact_time_offset_s": "",
        "contact_dist": "",
        "phase_at_contact": "",
        "success": 0,
        "miss_reason": "",
        "net_clear": 0,
        "first_bounce": 0,
        "first_bounce_x": "",
        "first_bounce_y": "",
        "opponent_bounce": 0,
        "fallen": 0,
        "timed_out": 0,
        "min_ball_racket_distance": "",
        "base_z_at_contact": "",
        "base_z_end": "",
        "time_to_strike_cmd": "",
        "racket_target_speed": "",
        "racket_speed_pre": "",
        "racket_speed_post": "",
        "ball_speed_pre": "",
        "ball_speed_post": "",
        "ball_speed_delta": "",
        "racket_vel_target_error": "",
        "racket_vel_target_angle_deg": "",
        "ball_post_vs_target_vel_error": "",
        "ball_post_vs_target_vel_angle_deg": "",
        "closing_speed_pre": "",
        "outgoing_normal_speed_post": "",
        "raw_action_max_at_contact": "",
        "applied_action_max_at_contact": "",
        "q_des_max_at_contact": "",
    }
    for prefix, value in (
        ("strike_sample", strike_pt),
        ("serve_pos", serve_pos),
        ("serve_vel", serve_vel),
        ("contact_pos", None),
        ("mujoco_contact_normal", None),
        ("racket_normal", None),
        ("target_pos", None),
        ("target_vel", None),
        ("ball_pre_pos", None),
        ("ball_pre_vel", None),
        ("ball_post_pos", None),
        ("ball_post_vel", None),
        ("racket_pre_pos", None),
        ("racket_pre_vel", None),
        ("racket_post_pos", None),
        ("racket_post_vel", None),
    ):
        _xyz(row, prefix, value)
    row.update(
        {
            "pred_net_cross": 0,
            "pred_net_z": "",
            "pred_net_clear": 0,
            "pred_bounce_x": "",
            "pred_bounce_y": "",
            "pred_opponent_bounce": 0,
        }
    )
    return row


def _contact_diag_fields() -> list[str]:
    base = [
        "trial",
        "side",
        "contact_kind",
        "contact_tick",
        "contact_substep",
        "contact_time_offset_s",
        "contact_dist",
        "phase_at_contact",
        "success",
        "miss_reason",
        "net_clear",
        "first_bounce",
        "first_bounce_x",
        "first_bounce_y",
        "opponent_bounce",
        "fallen",
        "timed_out",
        "min_ball_racket_distance",
        "base_z_at_contact",
        "base_z_end",
    ]
    xyz_prefixes = [
        "strike_sample",
        "serve_pos",
        "serve_vel",
        "contact_pos",
        "mujoco_contact_normal",
        "racket_normal",
        "target_pos",
        "target_vel",
        "ball_pre_pos",
        "ball_pre_vel",
        "ball_post_pos",
        "ball_post_vel",
        "racket_pre_pos",
        "racket_pre_vel",
        "racket_post_pos",
        "racket_post_vel",
    ]
    xyz_fields = [f"{prefix}_{axis}" for prefix in xyz_prefixes for axis in ("x", "y", "z")]
    scalars = [
        "time_to_strike_cmd",
        "racket_target_speed",
        "racket_speed_pre",
        "racket_speed_post",
        "ball_speed_pre",
        "ball_speed_post",
        "ball_speed_delta",
        "racket_vel_target_error",
        "racket_vel_target_angle_deg",
        "ball_post_vs_target_vel_error",
        "ball_post_vs_target_vel_angle_deg",
        "closing_speed_pre",
        "outgoing_normal_speed_post",
        "pred_net_cross",
        "pred_net_z",
        "pred_net_clear",
        "pred_bounce_x",
        "pred_bounce_y",
        "pred_opponent_bounce",
        "raw_action_max_at_contact",
        "applied_action_max_at_contact",
        "q_des_max_at_contact",
    ]
    return base + xyz_fields + scalars


class _MujocoVideoRecorder:
    def __init__(self, scene, path: str, *, width: int, height: int, fps: int) -> None:
        import imageio.v2 as imageio
        import mujoco

        pathlib.Path(path).resolve().parent.mkdir(parents=True, exist_ok=True)
        self._renderer = mujoco.Renderer(scene.model, height=height, width=width)
        self._writer = imageio.get_writer(path, fps=fps)
        self._camera = mujoco.MjvCamera()
        self._camera.type = mujoco.mjtCamera.mjCAMERA_FREE
        self._camera.lookat[:] = [scene.near_edge_x + 1.15, 0.0, scene.table_height + 0.35]
        self._camera.distance = 3.3
        self._camera.azimuth = -140.0
        self._camera.elevation = -18.0

    def capture(self, scene) -> None:
        self._renderer.update_scene(scene.data, camera=self._camera)
        self._writer.append_data(self._renderer.render())

    def close(self) -> None:
        self._writer.close()
        self._renderer.close()


# ---------------------------------------------------------------------------------------------------
# Example serve distribution (planner-less). These are NEUTRAL example incoming balls that arrive in
# front of the robot with a return toward the opponent half; they are NOT the training sampling boxes.
# Replace them (or feed your own planner's RacketCommand stream) for a deployment-faithful evaluation.
# All positions/velocities below are in the MuJoCo world (robot) frame, metres / m·s^-1.
# ---------------------------------------------------------------------------------------------------
def _rollout_drag_position(origin: np.ndarray, velocity: np.ndarray, flight_t: float, physics, dt: float = 0.002):
    """Forward-integrate no-spin drag flight to ``flight_t`` with the eval physics."""
    p = np.asarray(origin, dtype=np.float64).copy()
    v = np.asarray(velocity, dtype=np.float64).copy()
    remaining = float(flight_t)
    while remaining > 1e-12:
        h = min(dt, remaining)
        a = physics.acceleration(v)
        p = p + v * h + 0.5 * a * h * h
        v = v + a * h
        remaining -= h
    return p, v


def _solve_drag_serve_velocity(origin: np.ndarray, target: np.ndarray, flight_t: float, physics) -> np.ndarray:
    """Shoot an initial velocity whose drag trajectory reaches ``target`` at ``flight_t``.

    The eval predictor and MuJoCo scene both include quadratic drag. The old
    no-drag ballistic serve made the ball reach the strike plane too late/low,
    sometimes producing racket targets metres below the table. This small Newton
    shooting loop keeps the sampled serve and the predictor on the same physics.
    """
    accel = np.array([0.0, 0.0, -physics.gravity])
    v = (target - origin) / flight_t - 0.5 * accel * flight_t
    eps = 1e-3
    for _ in range(8):
        p, _ = _rollout_drag_position(origin, v, flight_t, physics)
        err = p - target
        if float(np.linalg.norm(err)) < 1e-4:
            break
        jac = np.zeros((3, 3), dtype=np.float64)
        for j in range(3):
            dv = np.zeros(3, dtype=np.float64)
            dv[j] = eps
            p_eps, _ = _rollout_drag_position(origin, v + dv, flight_t, physics)
            jac[:, j] = (p_eps - p) / eps
        try:
            step = np.linalg.solve(jac, err)
        except np.linalg.LinAlgError:
            step = np.linalg.lstsq(jac, err, rcond=None)[0]
        v = v - step
        speed = float(np.linalg.norm(v))
        if speed > physics.velocity_clip:
            v *= physics.velocity_clip / speed
    return v


def _sample_serve(rng, side, scene, physics, args, serve_manifest=None):
    """Sample (strike_point, serve_origin, serve_velocity) in the MuJoCo world frame.

    A strike point in the robot's reachable zone is chosen (forehand to the robot's
    right, backhand to its left), a serve origin is placed on the opponent side, and
    the serve velocity is solved with the same no-spin drag model used by the
    predictor/evaluator so the ball actually reaches the sampled strike point.
    """
    if serve_manifest:
        candidates = [r for r in serve_manifest if (r["side"] >= 0.0) == (side >= 0.0)]
        if not candidates:
            candidates = serve_manifest
        row = candidates[int(rng.integers(0, len(candidates)))]
        strike_pt = np.array([rng.uniform(lo, hi) for lo, hi in row["ranges"]], dtype=np.float64)
        y = float(strike_pt[1])
    else:
        strike_x = scene.near_edge_x + args.strike_plane_x  # MuJoCo x of the fixed strike plane
        if side >= 0:  # FOREHAND -> robot's right = -y in the MuJoCo/robot frame
            y = rng.uniform(-0.45, -0.12)
        else:          # BACKHAND -> robot's left = +y
            y = rng.uniform(0.05, 0.38)
        z = scene.table_height + rng.uniform(0.10, 0.22)
        strike_pt = np.array([strike_x, y, z], dtype=np.float64)

    origin = np.array(
        [
            scene.near_edge_x + rng.uniform(1.9, 2.4),
            y + rng.uniform(-0.15, 0.15),
            scene.table_height + rng.uniform(0.15, 0.35),
        ],
        dtype=np.float64,
    )
    flight_t = rng.uniform(0.75, 1.0)
    serve_vel = _solve_drag_serve_velocity(origin, strike_pt, flight_t, physics)
    return strike_pt, origin, serve_vel


def _predict_command(
    ball_pos,
    ball_vel,
    side,
    scene,
    physics,
    args,
    task_id,
    revision,
    RacketCommand,
    *,
    strike_x_w: float | None = None,
):
    """Lightweight inline no-spin intercept predictor -> a RacketCommand.

    Forward-integrates the true ball with the shared no-spin model (gravity +
    quadratic drag) until it crosses the fixed strike plane, producing the racket
    target position (where the ball will be), the time-to-strike, and a target racket
    velocity aimed at the fixed opponent-half landing point. ``swing_side`` is the
    side locked for this task. This mirrors the planner's job but reads the ball
    directly (the evaluator is the ground truth, so no mocap emulation is needed).
    """
    strike_x = float(strike_x_w) if strike_x_w is not None else scene.near_edge_x + args.strike_plane_x
    p = np.asarray(ball_pos, dtype=np.float64).copy()
    v = np.asarray(ball_vel, dtype=np.float64).copy()
    dt = 0.002
    t = 0.0
    y_cross, z_cross, tts = float(p[1]), float(p[2]), 0.0
    for _ in range(int(2.0 / dt)):
        a = physics.acceleration(v)
        p_new = p + v * dt + 0.5 * a * dt * dt
        v_new = v + a * dt
        if p[0] >= strike_x > p_new[0]:  # ball moving -x through the strike plane
            dx = p_new[0] - p[0]
            frac = (strike_x - p[0]) / dx if abs(dx) > 1e-12 else 0.5
            y_cross = float(p[1] + frac * (p_new[1] - p[1]))
            z_cross = float(p[2] + frac * (p_new[2] - p[2]))
            tts = t + frac * dt
            break
        p, v = p_new, v_new
        t += dt

    target_pos = np.array([strike_x, y_cross, z_cross], dtype=np.float64)

    # Target racket velocity: the ballistic velocity that would send the ball from the
    # strike point to the fixed opponent-half landing target in a fixed time.
    landing_table = np.array(
        [(scene.net_x_table + scene.length) / 2.0, -scene.width / 2.0, 0.0], dtype=np.float64
    )
    landing_mujoco = landing_table + scene.offset
    t_out = 0.6
    accel = np.array([0.0, 0.0, -physics.gravity])
    target_vel = (landing_mujoco - target_pos) / t_out - 0.5 * accel * t_out

    return RacketCommand(
        task_id=task_id,
        task_revision=revision,
        swing_side=side,
        position=target_pos,
        velocity=target_vel,
        time_to_strike=float(tts),
    )


def run_eval(args) -> dict:
    repo_root = _repo_root()
    if args.record_video:
        os.environ.setdefault("MUJOCO_GL", args.mujoco_gl)

    # Make the reference deploy package importable (shared 111-D obs / ActionAdapter /
    # lifecycle / RacketCommand / ONNX wrapper).
    ref_dir = pathlib.Path(args.reference_dir) if args.reference_dir else _reference_pkg_dir(repo_root)
    sys.path.insert(0, str(ref_dir))
    from a3_deploy_onnx_ref_pingpong.config import RuntimeConfig
    from a3_deploy_onnx_ref_pingpong.joint_order import HEAD_INDICES, JOINT_NAMES
    from a3_deploy_onnx_ref_pingpong.lifecycle import SwingLifecycle
    from a3_deploy_onnx_ref_pingpong.observation import build_observation
    from a3_deploy_onnx_ref_pingpong.onnx_policy import OnnxPolicy
    from a3_deploy_onnx_ref_pingpong.racket_command import (
        BACKHAND,
        FOREHAND,
        QueueRacketCommandSource,
        RacketCommand,
    )

    # Shared success metric + physics config (pure NumPy; the DEFINITION only -- fed
    # real simulated events here, never an analytic predicted-landing rollout). Loaded
    # directly by file path so importing it does NOT drag in the Isaac Lab training
    # package (its __init__ registers gym tasks that need gymnasium/Isaac).
    metric = _load_success_metric(repo_root)
    BallPhysics = metric.BallPhysics
    SuccessRate = metric.SuccessRate
    TableGeometry = metric.TableGeometry
    load_ball_physics_config = metric.load_ball_physics_config

    # The MuJoCo scene builder lives next to this script.
    sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
    from mujoco_pingpong_scene import PingPongRealPhysicsScene

    runtime_cfg = RuntimeConfig.load(args.runtime_config or _default_runtime_config(repo_root))
    onnx_path = args.onnx or str(runtime_cfg.onnx_path)
    robot_xml = args.model_xml or str(runtime_cfg.model_xml_path)

    ball_cfg = load_ball_physics_config()
    physics = BallPhysics.from_config(ball_cfg)
    table = TableGeometry.from_config(ball_cfg)
    accumulator = SuccessRate()
    serve_manifest = _load_serve_manifest(args.serve_manifest)

    policy = OnnxPolicy(onnx_path)
    scene = PingPongRealPhysicsScene(
        robot_xml,
        ball_cfg,
        JOINT_NAMES,
        control_dt=runtime_cfg.control_dt,
        near_edge_x=args.near_edge_x,
        launch_viewer=args.view,
    )
    recorder = (
        _MujocoVideoRecorder(
            scene,
            args.record_video,
            width=args.video_width,
            height=args.video_height,
            fps=args.video_fps,
        )
        if args.record_video
        else None
    )

    default_q = runtime_cfg.action_adapter.default_q.copy()
    kp = runtime_cfg.sim_kp.copy() * float(args.kp_scale)
    kd = runtime_cfg.sim_kd.copy() * float(args.kd_scale)
    head_idx = list(HEAD_INDICES)
    net_clear_z = table.net_height + physics.ball_radius
    contact_radius = args.contact_radius
    dt = runtime_cfg.control_dt
    max_ticks = max(1, int(round(args.max_trial_seconds / dt)))
    rng = np.random.default_rng(args.seed)
    continuous = args.eval_mode == "continuous"
    counts = _new_eval_counts()
    by_side = {"forehand": _new_eval_counts(), "backhand": _new_eval_counts()}
    trace_fh = None
    trace_writer = None
    if args.trace_csv:
        trace_path = pathlib.Path(args.trace_csv)
        trace_path.parent.mkdir(parents=True, exist_ok=True)
        trace_fh = trace_path.open("w", encoding="utf-8", newline="")
        trace_fields = [
            "trial",
            "tick",
            "side",
            "phase",
            "base_x",
            "base_y",
            "base_z",
            "ball_x",
            "ball_y",
            "ball_z",
            "racket_x",
            "racket_y",
            "racket_z",
            "ball_racket_distance",
            "target_x",
            "target_y",
            "target_z",
            "time_to_strike",
            "contacted",
            "real_contact",
            "proximity_contact",
            "net_clear",
            "first_bounce",
            "fallen",
            "ncon",
            "floor_contact_count",
            "floor_contact_min_dist",
            "max_abs_raw_action",
            "max_abs_applied_action",
            "max_abs_q_des",
        ]
        trace_writer = csv.DictWriter(trace_fh, fieldnames=trace_fields)
        trace_writer.writeheader()
    contact_diag_fh = None
    contact_diag_writer = None
    if args.contact_diag_csv:
        contact_diag_path = pathlib.Path(args.contact_diag_csv)
        contact_diag_path.parent.mkdir(parents=True, exist_ok=True)
        contact_diag_fh = contact_diag_path.open("w", encoding="utf-8", newline="")
        contact_diag_writer = csv.DictWriter(contact_diag_fh, fieldnames=_contact_diag_fields())
        contact_diag_writer.writeheader()

    def _floor_contact_summary():
        vals = []
        for ci in range(scene.data.ncon):
            con = scene.data.contact[ci]
            g1 = scene._mj.mj_id2name(scene.model, scene._mj.mjtObj.mjOBJ_GEOM, int(con.geom1)) or ""
            g2 = scene._mj.mj_id2name(scene.model, scene._mj.mjtObj.mjOBJ_GEOM, int(con.geom2)) or ""
            if g1 == "floor" or g2 == "floor":
                vals.append(float(con.dist))
        return len(vals), (min(vals) if vals else None)

    def _record_contact_diag(
        row: dict,
        *,
        kind: str,
        tick: int,
        phase: str,
        events,
        cmd,
        ball_pre_pos,
        ball_pre_vel,
        racket_pre_pos,
        racket_pre_vel,
        racket_post_pos,
        racket_post_vel,
        racket_geom_xmat,
        ball_post_pos,
        ball_post_vel,
        base_at_contact,
        diag: dict,
    ) -> None:
        real = kind == "real"
        contact_ball_pre_pos = events.contact_ball_pos_pre_w if real and events.contact_ball_pos_pre_w is not None else ball_pre_pos
        contact_ball_pre_vel = events.contact_ball_vel_pre_w if real and events.contact_ball_vel_pre_w is not None else ball_pre_vel
        contact_ball_post_pos = ball_post_pos
        contact_ball_post_vel = ball_post_vel
        contact_pos = events.contact_pos_w if real and events.contact_pos_w is not None else contact_ball_post_pos

        to_ball = np.asarray(contact_ball_pre_pos, dtype=np.float64) - np.asarray(racket_post_pos, dtype=np.float64)
        racket_normal = _choose_racket_normal(np.asarray(racket_geom_xmat, dtype=np.float64), to_ball)
        mujoco_normal = events.contact_normal_w if real and events.contact_normal_w is not None else None
        if mujoco_normal is not None:
            mujoco_normal = np.asarray(mujoco_normal, dtype=np.float64).copy()
            to_ball_u = _unit_or_none(to_ball)
            if to_ball_u is not None and float(np.dot(mujoco_normal, to_ball_u)) < 0.0:
                mujoco_normal = -mujoco_normal

        target_vel = np.asarray(cmd.velocity, dtype=np.float64)
        racket_pre_vel = np.asarray(racket_pre_vel, dtype=np.float64)
        racket_post_vel = np.asarray(racket_post_vel, dtype=np.float64)
        contact_ball_pre_vel = np.asarray(contact_ball_pre_vel, dtype=np.float64)
        contact_ball_post_vel = np.asarray(contact_ball_post_vel, dtype=np.float64)
        rel_pre = contact_ball_pre_vel - racket_pre_vel
        rel_post = contact_ball_post_vel - racket_post_vel
        prediction = _outgoing_flight_diag(scene.to_table(contact_ball_post_pos), contact_ball_post_vel, physics, table)

        row["contact_kind"] = kind
        row["contact_tick"] = int(tick)
        row["contact_substep"] = "" if events.contact_substep is None else int(events.contact_substep)
        row["contact_time_offset_s"] = "" if events.contact_time_offset_s is None else float(events.contact_time_offset_s)
        row["contact_dist"] = "" if events.contact_dist is None else float(events.contact_dist)
        row["phase_at_contact"] = phase
        row["base_z_at_contact"] = float(np.asarray(base_at_contact, dtype=np.float64)[2])
        row["time_to_strike_cmd"] = float(cmd.time_to_strike)
        row["racket_target_speed"] = _norm(target_vel)
        row["racket_speed_pre"] = _norm(racket_pre_vel)
        row["racket_speed_post"] = _norm(racket_post_vel)
        row["ball_speed_pre"] = _norm(contact_ball_pre_vel)
        row["ball_speed_post"] = _norm(contact_ball_post_vel)
        row["ball_speed_delta"] = _norm(contact_ball_post_vel) - _norm(contact_ball_pre_vel)
        row["racket_vel_target_error"] = _norm(racket_post_vel - target_vel)
        angle = _angle_deg(racket_post_vel, target_vel)
        row["racket_vel_target_angle_deg"] = "" if angle is None else angle
        row["ball_post_vs_target_vel_error"] = _norm(contact_ball_post_vel - target_vel)
        ball_angle = _angle_deg(contact_ball_post_vel, target_vel)
        row["ball_post_vs_target_vel_angle_deg"] = "" if ball_angle is None else ball_angle
        row["closing_speed_pre"] = float(-np.dot(rel_pre, racket_normal))
        row["outgoing_normal_speed_post"] = float(np.dot(rel_post, racket_normal))
        row["raw_action_max_at_contact"] = float(diag["max_abs_raw_action"])
        row["applied_action_max_at_contact"] = float(diag["max_abs_applied_action"])
        row["q_des_max_at_contact"] = float(diag["max_abs_q_des"])
        row.update(prediction)

        for prefix, value in (
            ("contact_pos", contact_pos),
            ("mujoco_contact_normal", mujoco_normal),
            ("racket_normal", racket_normal),
            ("target_pos", cmd.position),
            ("target_vel", target_vel),
            ("ball_pre_pos", contact_ball_pre_pos),
            ("ball_pre_vel", contact_ball_pre_vel),
            ("ball_post_pos", contact_ball_post_pos),
            ("ball_post_vel", contact_ball_post_vel),
            ("racket_pre_pos", racket_pre_pos),
            ("racket_pre_vel", racket_pre_vel),
            ("racket_post_pos", racket_post_pos),
            ("racket_post_vel", racket_post_vel),
        ):
            _xyz(row, prefix, value)

    def _policy_tick(lifecycle, source, last_action, fixed_station_xy):
        """One 50 Hz policy step (identical to the deploy runner's tick).

        Returns (events, applied_action) — the caller keeps ``applied_action`` as
        the next tick's ``last_action``.
        """
        state = scene.read_robot_state()
        target = lifecycle.update(source.poll(), state)
        obs = build_observation(state, target, last_action, default_q, fixed_station_xy)
        policy_obs = _obs_for_policy_joint_order(obs, last_action, args.policy_joint_order)
        raw_policy_action = policy.infer(policy_obs)
        raw_action = _raw_action_to_canonical(raw_policy_action, args.policy_joint_order)
        # Applied action = raw with the passive head columns zeroed, matching both
        # the deploy runner and training's zeroed last_action feedback.
        applied_action = np.asarray(raw_action, dtype=np.float64).copy()
        if runtime_cfg.passive_neck:
            applied_action[head_idx] = 0.0
        q_des = runtime_cfg.action_adapter.decode(applied_action)
        if runtime_cfg.passive_neck:
            q_des[head_idx] = default_q[head_idx]
        scene.write_targets(q_des, kp, kd)
        diag = {
            "finite": bool(np.all(np.isfinite(raw_policy_action)) and np.all(np.isfinite(raw_action)) and np.all(np.isfinite(applied_action)) and np.all(np.isfinite(q_des))),
            "max_abs_raw_action": float(np.nanmax(np.abs(raw_policy_action))),
            "max_abs_applied_action": float(np.nanmax(np.abs(applied_action))),
            "max_abs_q_des": float(np.nanmax(np.abs(q_des))),
        }
        return scene.step(), applied_action, diag

    def _park_ball():
        """Drop the ball out of play (past the far edge) between rally serves."""
        scene.set_ball(
            [scene.near_edge_x + scene.length + 1.0, 0.0, scene.table_height + 0.5],
            [0.0, 0.0, 0.0],
        )

    # Continuous mode: ONE initialization for the whole session — robot state,
    # last_action, lifecycle and the fixed station all persist across serves
    # (matching the deploy runner). Independent mode re-creates them per serve.
    scene.reset_stand()
    lifecycle = SwingLifecycle(runtime_cfg.lifecycle)
    source = QueueRacketCommandSource()
    last_action = np.zeros(31, dtype=np.float64)
    fixed_station_xy = scene.base_pos_w()[:2].copy()
    max_rest_ticks = max(1, int(round(args.max_rest_seconds / dt)))
    transitions_seen: set = set()
    prev_side = None
    task_counter = 0

    for trial in range(args.num_serves):
        # Side pattern FH, FH, BH, BH, ... exercises all four adjacent side
        # transitions (FH->FH, FH->BH, BH->BH, BH->FH) across the session.
        if args.side_mode == "forehand":
            side = FOREHAND
        elif args.side_mode == "backhand":
            side = BACKHAND
        else:
            side = FOREHAND if (trial % 4) < 2 else BACKHAND
        if prev_side is not None:
            transitions_seen.add((prev_side, side))
        prev_side = side
        strike_pt, serve_pos, serve_vel = _sample_serve(rng, side, scene, physics, args, serve_manifest)

        if not continuous:
            # Independent-strike evaluation: fresh robot + policy state per serve.
            scene.reset_stand()
            lifecycle = SwingLifecycle(runtime_cfg.lifecycle)
            source = QueueRacketCommandSource()
            last_action = np.zeros(31, dtype=np.float64)
            fixed_station_xy = scene.base_pos_w()[:2].copy()
        elif trial > 0:
            # Continuous rally: keep the policy running (no reset, no teleport)
            # through follow-through/recovery until it is ready for the next ball.
            _park_ball()
            for _ in range(max_rest_ticks):
                _events, last_action, _diag = _policy_tick(
                    lifecycle, source, last_action, fixed_station_xy
                )
                if recorder is not None and trial < args.record_serves:
                    recorder.capture(scene)
                if lifecycle.phase.value == "ready":
                    break

        scene.set_ball(serve_pos, serve_vel)
        if recorder is not None and trial < args.record_serves:
            recorder.capture(scene)
        task_counter += 1
        task_id = task_counter
        revision = 0
        contacted = False
        real_contacted = False
        proximity_contacted = False
        net_clear = False
        first_bounce = None
        fell = False
        timed_out = True
        min_distance = float("inf")
        trial_nonfinite_ticks = 0
        trial_max_raw = 0.0
        trial_max_applied = 0.0
        trial_max_q_des = 0.0
        side_name = "forehand" if side >= 0 else "backhand"
        contact_diag_row = _new_contact_diag_row(trial, side_name, strike_pt, serve_pos, serve_vel)

        for _tick in range(max_ticks):
            ball_pos, ball_vel = scene.ball_state()
            cmd = _predict_command(
                ball_pos,
                ball_vel,
                side,
                scene,
                physics,
                args,
                task_id,
                revision,
                RacketCommand,
                strike_x_w=float(strike_pt[0]) if serve_manifest else None,
            )
            revision += 1
            source.submit(cmd)

            racket_pre_pos, racket_pre_vel, _racket_site_xmat_pre = scene.racket_site_pose()
            events, last_action, diag = _policy_tick(lifecycle, source, last_action, fixed_station_xy)
            if recorder is not None and trial < args.record_serves:
                recorder.capture(scene)
            if not diag["finite"]:
                trial_nonfinite_ticks += 1
            trial_max_raw = max(trial_max_raw, diag["max_abs_raw_action"])
            trial_max_applied = max(trial_max_applied, diag["max_abs_applied_action"])
            trial_max_q_des = max(trial_max_q_des, diag["max_abs_q_des"])

            r_pos, r_vel, _racket_site_xmat = scene.racket_site_pose()
            _racket_geom_pos, racket_geom_xmat = scene.racket_geom_pose()
            b_pos, b_vel = scene.ball_state()
            delta = b_pos - r_pos
            distance = float(np.linalg.norm(delta))
            min_distance = min(min_distance, distance)

            # --- contact: real MuJoCo ball<->racket contact, or the allowed proximity
            #     fallback (ball within contact_radius of the racket site with the
            #     racket moving toward it). ---
            new_contact_kind = None
            if events.ball_racket_contact:
                real_contacted = True
                if not contacted:
                    contacted = True
                new_contact_kind = "real"
            elif not contacted and np.linalg.norm(delta) <= contact_radius and float(np.dot(r_vel, delta)) > 0.0:
                contacted = True
                proximity_contacted = True
                new_contact_kind = "proximity"
            if new_contact_kind == "real" or (
                new_contact_kind == "proximity" and contact_diag_row["contact_kind"] == "none"
            ):
                _record_contact_diag(
                    contact_diag_row,
                    kind=new_contact_kind,
                    tick=_tick,
                    phase=lifecycle.phase.value,
                    events=events,
                    cmd=cmd,
                    ball_pre_pos=ball_pos,
                    ball_pre_vel=ball_vel,
                    racket_pre_pos=racket_pre_pos,
                    racket_pre_vel=racket_pre_vel,
                    racket_post_pos=r_pos,
                    racket_post_vel=r_vel,
                    racket_geom_xmat=racket_geom_xmat,
                    ball_post_pos=b_pos,
                    ball_post_vel=b_vel,
                    base_at_contact=scene.base_pos_w(),
                    diag=diag,
                )

            # --- after contact, watch the REAL outgoing ball for net clearance and its
            #     first bounce (both read off the simulated trajectory). ---
            if contacted and first_bounce is None:
                if not net_clear:
                    for z_cross, x_sign in events.net_crossings:
                        if x_sign > 0.0 and z_cross > net_clear_z:
                            net_clear = True
                            break
                for x_t, y_t, z_sign in events.surface_crossings:
                    if z_sign < 0.0:
                        first_bounce = (x_t, y_t)
                        break

            if first_bounce is not None:
                timed_out = False
                break
            if scene.base_fallen():
                fell = True
            if trace_writer is not None and trial < args.trace_serves:
                floor_count, floor_min = _floor_contact_summary()
                base = scene.base_pos_w()
                trace_writer.writerow(
                    {
                        "trial": trial,
                        "tick": _tick,
                        "side": side_name,
                        "phase": lifecycle.phase.value,
                        "base_x": float(base[0]),
                        "base_y": float(base[1]),
                        "base_z": float(base[2]),
                        "ball_x": float(b_pos[0]),
                        "ball_y": float(b_pos[1]),
                        "ball_z": float(b_pos[2]),
                        "racket_x": float(r_pos[0]),
                        "racket_y": float(r_pos[1]),
                        "racket_z": float(r_pos[2]),
                        "ball_racket_distance": float(distance),
                        "target_x": float(cmd.position[0]),
                        "target_y": float(cmd.position[1]),
                        "target_z": float(cmd.position[2]),
                        "time_to_strike": float(cmd.time_to_strike),
                        "contacted": int(contacted),
                        "real_contact": int(real_contacted),
                        "proximity_contact": int(proximity_contacted),
                        "net_clear": int(net_clear),
                        "first_bounce": int(first_bounce is not None),
                        "fallen": int(fell),
                        "ncon": int(scene.data.ncon),
                        "floor_contact_count": int(floor_count),
                        "floor_contact_min_dist": "" if floor_min is None else float(floor_min),
                        "max_abs_raw_action": float(diag["max_abs_raw_action"]),
                        "max_abs_applied_action": float(diag["max_abs_applied_action"]),
                        "max_abs_q_des": float(diag["max_abs_q_des"]),
                    }
                )
            # Incoming ball that flew past the robot without ever being contacted is a
            # definite miss (it can no longer produce an opponent-half return): stop the
            # trial early instead of waiting out the timeout.
            if not contacted and scene.ball_state()[0][0] < scene.near_edge_x - 0.8:
                timed_out = False
                break

        opponent = bool(first_bounce is not None and table.on_opponent_half(first_bounce[0], first_bounce[1]))
        success = bool(
            contacted
            and net_clear
            and first_bounce is not None
            and opponent
        )
        if success:
            miss_reason = "success"
        elif not contacted:
            miss_reason = "no_contact"
        elif not net_clear:
            miss_reason = "contact_no_net"
        elif first_bounce is None:
            miss_reason = "net_no_bounce"
        elif not opponent:
            miss_reason = "wrong_bounce"
        else:
            miss_reason = "unknown"
        if fell and not success:
            miss_reason = f"{miss_reason}+fell"
        if timed_out and not success:
            miss_reason = f"{miss_reason}+timeout"
        contact_diag_row["success"] = int(success)
        contact_diag_row["miss_reason"] = miss_reason
        contact_diag_row["net_clear"] = int(net_clear)
        contact_diag_row["first_bounce"] = int(first_bounce is not None)
        contact_diag_row["first_bounce_x"] = "" if first_bounce is None else float(first_bounce[0])
        contact_diag_row["first_bounce_y"] = "" if first_bounce is None else float(first_bounce[1])
        contact_diag_row["opponent_bounce"] = int(opponent)
        contact_diag_row["fallen"] = int(fell)
        contact_diag_row["timed_out"] = int(timed_out)
        contact_diag_row["min_ball_racket_distance"] = float(min_distance)
        contact_diag_row["base_z_end"] = float(scene.base_pos_w()[2])
        if contact_diag_writer is not None:
            contact_diag_writer.writerow(contact_diag_row)
        accumulator.add_bool(success)
        for bucket in (counts, by_side[side_name]):
            bucket["attempts"] += 1
            bucket["successes"] += int(success)
            bucket["contact"] += int(contacted)
            bucket["real_contact"] += int(real_contacted)
            bucket["proximity_contact"] += int(proximity_contacted)
            bucket["net_clear"] += int(net_clear)
            bucket["first_bounce"] += int(first_bounce is not None)
            bucket["opponent_bounce"] += int(opponent)
            bucket["nonfinite_action_ticks"] += int(trial_nonfinite_ticks)
            if not contacted:
                bucket["miss_no_contact"] += 1
            elif not net_clear:
                bucket["miss_contact_no_net"] += 1
            elif first_bounce is None:
                bucket["miss_net_no_bounce"] += 1
            elif not opponent:
                bucket["miss_wrong_bounce"] += 1
            if fell:
                bucket["miss_fell"] += 1
            if timed_out:
                bucket["miss_timeout"] += 1
            bucket["min_ball_racket_distance"] = min(bucket["min_ball_racket_distance"], min_distance)
            _update_max(bucket, "max_abs_raw_action", trial_max_raw)
            _update_max(bucket, "max_abs_applied_action", trial_max_applied)
            _update_max(bucket, "max_abs_q_des", trial_max_q_des)

    if recorder is not None:
        recorder.close()
    if trace_fh is not None:
        trace_fh.close()
    if contact_diag_fh is not None:
        contact_diag_fh.close()
    scene.close()
    if continuous:
        _names = {FOREHAND: "FH", BACKHAND: "BH"}
        seen = sorted(f"{_names[a]}->{_names[b]}" for a, b in transitions_seen)
        all_four = len(transitions_seen) == 4
        print(
            f"[mujoco_eval] adjacent side transitions exercised: {', '.join(seen) or 'none'}"
            + ("" if all_four else "  (increase --num-serves >= 5 to cover all four)"),
            file=sys.stderr,
        )
    print(
        f"[mujoco_eval] mode={args.eval_mode} serves={accumulator.attempts} "
        f"returns={accumulator.successes} success_rate={accumulator.value:.4f} "
        f"contact_rate={counts['contact'] / max(1, counts['attempts']):.4f} "
        f"net_clear_rate={counts['net_clear'] / max(1, counts['attempts']):.4f} "
        f"opponent_bounce_rate={counts['opponent_bounce'] / max(1, counts['attempts']):.4f}",
        file=sys.stderr,
    )
    result = accumulator.as_dict()
    if args.detailed:
        result.update(
            {
                "counts": _finalize_counts(counts),
                "by_side": {name: _finalize_counts(v) for name, v in by_side.items()},
                "eval_mode": args.eval_mode,
                "side_mode": args.side_mode,
                "policy_joint_order": args.policy_joint_order,
                "kp_scale": float(args.kp_scale),
                "kd_scale": float(args.kd_scale),
                "serve_manifest": str(args.serve_manifest) if args.serve_manifest else None,
                "contact_diag_csv": str(args.contact_diag_csv) if args.contact_diag_csv else None,
            }
        )
    return result


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--onnx", default=None, help="Exported hope_pingpong.onnx (default: from runtime config).")
    parser.add_argument("--model-xml", default=None, help="a3_pingpong MJCF (default: from runtime config).")
    parser.add_argument("--runtime-config", default=None, help="hope_pingpong_runtime.yaml (default: shipped).")
    parser.add_argument("--reference-dir", default=None, help="Dir containing the reference deploy package.")
    parser.add_argument("--num-serves", type=int, default=50, help="Number of served balls (denominator).")
    parser.add_argument(
        "--side-mode",
        choices=["mixed", "forehand", "backhand"],
        default="mixed",
        help="Serve side schedule. mixed keeps FH,FH,BH,BH; forehand/backhand force one side.",
    )
    parser.add_argument(
        "--serve-manifest",
        default=None,
        help="Optional TSV motion manifest; sample strike points from its racket_pos target boxes.",
    )
    parser.add_argument("--detailed", action="store_true", help="Emit detailed stage rates and diagnostics.")
    parser.add_argument(
        "--policy-joint-order",
        choices=["canonical", "isaac-articulation-action", "isaac-articulation-obs-action"],
        default="canonical",
        help="ONNX policy column semantics for MuJoCo diagnostics. canonical is the public contract; "
             "isaac-articulation-action treats only raw_action as the old Isaac articulation order; "
             "isaac-articulation-obs-action also reorders the 31-D joint_pos/joint_vel/last_action obs slices.",
    )
    parser.add_argument(
        "--eval-mode", choices=["continuous", "independent"], default="continuous",
        help="continuous (default): one uninterrupted rally session — robot/policy state persist "
             "across serves and all four adjacent side transitions are exercised. "
             "independent: reset the robot per serve (isolated-swing evaluation only).",
    )
    parser.add_argument(
        "--max-rest-seconds", type=float, default=2.0,
        help="continuous mode: max seconds the policy gets between serves to finish "
             "follow-through/recovery before the next ball is served.",
    )
    parser.add_argument(
        "--max-trial-seconds", type=float, default=3.0, help="Max simulated seconds per serve before scoring."
    )
    parser.add_argument("--contact-radius", type=float, default=0.10, help="Racket-site proximity contact fallback (m).")
    parser.add_argument("--kp-scale", type=float, default=1.0, help="Scale MuJoCo bridge PD stiffness gains.")
    parser.add_argument("--kd-scale", type=float, default=1.0, help="Scale MuJoCo bridge PD damping gains.")
    parser.add_argument(
        "--near-edge-x", type=float, default=0.30,
        help="MuJoCo x of the table's near edge (sets the robot-to-table placement).",
    )
    parser.add_argument(
        "--strike-plane-x", type=float, default=0.0,
        help="Strike plane, table-frame x (0 = the near table edge); the fixed intercept plane.",
    )
    parser.add_argument("--seed", type=int, default=0, help="RNG seed for the example serve distribution.")
    parser.add_argument("--view", action="store_true", help="Launch the MuJoCo passive viewer (debug only).")
    parser.add_argument("--record-video", default=None, help="Write an MP4 of the first eval serves.")
    parser.add_argument("--record-serves", type=int, default=5, help="Number of initial serves to record.")
    parser.add_argument("--trace-csv", default=None, help="Write per-tick MuJoCo eval diagnostics to CSV.")
    parser.add_argument("--trace-serves", type=int, default=5, help="Number of initial serves to trace.")
    parser.add_argument(
        "--contact-diag-csv",
        default=None,
        help="Write one row per serve with contact-instant ball/racket/outgoing-flight diagnostics.",
    )
    parser.add_argument("--video-width", type=int, default=1280)
    parser.add_argument("--video-height", type=int, default=720)
    parser.add_argument("--video-fps", type=int, default=50)
    parser.add_argument("--mujoco-gl", default="egl", help="MUJOCO_GL value used when recording video.")
    parser.add_argument("--json-out", default=None, help="Also write {'success_rate': ...} to this file.")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    result = run_eval(args)
    print(json.dumps(result))
    if args.json_out:
        with open(args.json_out, "w", encoding="utf-8") as f:
            json.dump(result, f)
            f.write("\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
