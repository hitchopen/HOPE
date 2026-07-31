"""MuJoCo-backed legal-serve search and paddle impact inversion."""

from __future__ import annotations

import itertools
import math
from dataclasses import asdict, dataclass
from typing import TYPE_CHECKING, Any

import numpy as np

from .math3d import normalize

if TYPE_CHECKING:
    from .mujoco_scene import A3ServeScene, BallReplay


@dataclass(frozen=True)
class ServeCandidate:
    outgoing_velocity_world: np.ndarray
    racket_velocity_world: np.ndarray
    racket_normal_world: np.ndarray
    racket_contact_position_world: np.ndarray
    ball_contact_position_world: np.ndarray
    first_bounce_table: np.ndarray | None
    second_bounce_table: np.ndarray | None
    net_clearance_m: float | None
    legal: bool
    score: float

    def json(self) -> dict[str, Any]:
        payload = asdict(self)
        for key, value in list(payload.items()):
            if isinstance(value, np.ndarray):
                payload[key] = [float(item) for item in value]
            elif isinstance(value, np.generic):
                payload[key] = value.item()
        return payload


def invert_normal_impact(
    incoming_velocity: np.ndarray,
    outgoing_velocity: np.ndarray,
    restitution: float,
) -> tuple[np.ndarray, np.ndarray]:
    """Return minimum-norm racket velocity and face normal for a no-spin hit.

    For a moving plane, ``v_out = v_in - (1+e) ((v_in-u)·n) n``.
    The desired velocity change fixes ``n``; choosing only the required normal
    component gives the minimum racket speed compatible with that change.
    """

    incoming = np.asarray(incoming_velocity, dtype=np.float64)
    outgoing = np.asarray(outgoing_velocity, dtype=np.float64)
    delta = outgoing - incoming
    normal = normalize(delta)
    required_normal_speed = float(np.dot(incoming, normal)) + float(
        np.linalg.norm(delta)
    ) / (1.0 + float(restitution))
    return normal * required_normal_speed, normal


def _candidate_score(
    replay: "BallReplay",
    first_target: np.ndarray,
    second_target: np.ndarray,
    net_height: float,
    ball_radius: float,
) -> tuple[float, bool]:
    if replay.first_bounce_table is None:
        return 1.0e6, False
    score = 4.0 * float(np.linalg.norm(replay.first_bounce_table[:2] - first_target[:2]))
    legal = replay.first_bounce_table[0] < replay.net_x_table
    if replay.second_bounce_table is None:
        score += 100.0
        legal = False
    else:
        score += float(np.linalg.norm(replay.second_bounce_table[:2] - second_target[:2]))
        legal = legal and replay.second_bounce_table[0] > replay.net_x_table
    if replay.net_clearance_m is None:
        score += 50.0
        legal = False
    else:
        minimum = net_height + ball_radius
        if replay.net_clearance_m < minimum:
            score += 50.0 + 20.0 * (minimum - replay.net_clearance_m)
            legal = False
        else:
            score += 0.2 * abs(replay.net_clearance_m - (minimum + 0.04))
    if replay.net_contact:
        score += 100.0
        legal = False
    if not replay.bounces_inside_table:
        score += 100.0
        legal = False
    return score, legal


def search_legal_serve(
    scene: "A3ServeScene",
    fixed_joint_positions: np.ndarray,
    ready_racket_position: np.ndarray,
    planner: dict[str, Any],
    physics: dict[str, Any],
) -> ServeCandidate:
    """Grid-search outgoing ball state using MuJoCo drag, gravity, table and net."""

    incoming = np.asarray(planner["incoming_ball_velocity_world"], dtype=np.float64)
    first_target = np.asarray(planner["first_bounce_target_table"], dtype=np.float64)
    second_target = np.asarray(planner["second_bounce_target_table"], dtype=np.float64)
    restitution = float(physics["contact"]["paddle"]["restitution"])
    preimpact = float(planner["preimpact_seconds"])
    gap = float(physics["ball"]["radius"]) + float(planner["racket_contact_gap_m"])

    best: ServeCandidate | None = None
    for speed, elevation_deg, azimuth_deg in itertools.product(
        planner["speed_m_s"],
        planner["elevation_deg"],
        planner["azimuth_deg"],
    ):
        elevation = math.radians(elevation_deg)
        azimuth = math.radians(azimuth_deg)
        outgoing = float(speed) * np.array(
            [
                math.cos(elevation) * math.cos(azimuth),
                math.cos(elevation) * math.sin(azimuth),
                math.sin(elevation),
            ]
        )
        racket_velocity, normal = invert_normal_impact(incoming, outgoing, restitution)
        if float(np.linalg.norm(racket_velocity)) > float(planner["max_racket_speed_m_s"]):
            continue
        # The robot is held at READY before the stroke.  A constant-acceleration
        # ramp from zero to the required impact velocity travels 1/2*u*T.
        racket_contact = (
            np.asarray(ready_racket_position)
            + 0.5 * racket_velocity * preimpact
        )
        ball_contact = racket_contact + normal * gap
        replay = scene.simulate_outgoing_ball(
            fixed_joint_positions,
            ball_contact,
            outgoing,
            duration_s=float(planner["flight_duration_s"]),
        )
        score, legal = _candidate_score(
            replay,
            first_target,
            second_target,
            float(physics["net"]["height"]),
            float(physics["ball"]["radius"]),
        )
        candidate = ServeCandidate(
            outgoing_velocity_world=outgoing,
            racket_velocity_world=racket_velocity,
            racket_normal_world=normal,
            racket_contact_position_world=racket_contact,
            ball_contact_position_world=ball_contact,
            first_bounce_table=replay.first_bounce_table,
            second_bounce_table=replay.second_bounce_table,
            net_clearance_m=replay.net_clearance_m,
            legal=legal,
            score=score,
        )
        if best is None or (candidate.legal, -candidate.score) > (best.legal, -best.score):
            best = candidate
    if best is None:
        raise RuntimeError("serve search produced no candidate within the racket-speed limit")
    if bool(planner.get("require_legal", True)) and not best.legal:
        raise RuntimeError(
            "MuJoCo search found no legal two-bounce serve; best score="
            f"{best.score:.6f}"
        )
    return best
