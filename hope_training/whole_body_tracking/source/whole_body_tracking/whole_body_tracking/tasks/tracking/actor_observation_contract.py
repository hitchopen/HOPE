"""The single HOPE actor-observation contract.

One contract, ``hope_pingpong`` (111 dims), pinned so training, ONNX export, the reference runner and
the planner all agree on the exact term order, dimensions and deploy sources. This module is pure
Python (no Isaac/torch imports) so it can be imported and unit-tested anywhere.

Layout (assembly order is fixed; slices are contiguous):

    [0:3]      base_ang_vel            pelvis angular velocity, body frame (IMU)
    [3:34]     joint_pos               q - default_q, joint order = canonical order (encoders)
    [34:65]    joint_vel               joint velocities (encoders)
    [65:96]    last_action             previous tick's applied raw action (31)
    [96:99]    projected_gravity       gravity direction in the base frame (IMU)
    [99:101]   base_forward_xy         base forward unit vector e_base,x, world XY
    [101:103]  fixed_station_error_xy  fixed startup station XY minus current base XY, world (m)
    [103:106]  racket_target_rel_base  target racket position minus base position, world (m)
    [106:109]  racket_target_vel_w     desired racket velocity, world (m/s)
    [109:110]  time_to_strike          remaining time to the strike (s)
    [110:111]  swing_side              forehand +1 / backhand -1 (locked per task)

Total = 3+31+31+31+3+2+2+3+3+1+1 = 111.

The 62-D reference-motion joint stream and other privileged signals may feed the CRITIC (value
function) during training, but they NEVER enter this actor/deployed observation.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable


@dataclass(frozen=True)
class ActorObservationTerm:
    name: str
    dim: int
    deploy_source: str
    description: str


@dataclass(frozen=True)
class ActorObservationContract:
    name: str
    total_dim: int
    terms: tuple[ActorObservationTerm, ...]

    @property
    def layout(self) -> tuple[tuple[str, int], ...]:
        return tuple((term.name, term.dim) for term in self.terms)

    def slices(self) -> dict[str, tuple[int, int]]:
        """Return ``{term_name: (start, stop)}`` for every term."""
        out: dict[str, tuple[int, int]] = {}
        offset = 0
        for term in self.terms:
            out[term.name] = (offset, offset + term.dim)
            offset += term.dim
        return out


HOPE_PINGPONG = ActorObservationContract(
    name="hope_pingpong",
    total_dim=111,
    terms=(
        ActorObservationTerm("base_ang_vel", 3, "imu", "pelvis angular velocity in the body frame"),
        ActorObservationTerm("joint_pos", 31, "encoders", "joint position offset from default (q - default_q)"),
        ActorObservationTerm("joint_vel", 31, "encoders", "joint velocities"),
        ActorObservationTerm("last_action", 31, "runtime_state", "previous tick's applied raw action"),
        ActorObservationTerm("projected_gravity", 3, "imu", "gravity direction in the base frame"),
        ActorObservationTerm("base_forward_xy", 2, "imu", "base forward unit vector e_base,x, world XY"),
        ActorObservationTerm(
            "fixed_station_error_xy",
            2,
            "runtime_state",
            "fixed startup station XY minus current base XY, world frame",
        ),
        ActorObservationTerm(
            "racket_target_rel_base", 3, "planner", "target racket position minus base position, world frame"
        ),
        ActorObservationTerm("racket_target_vel_w", 3, "planner", "desired racket velocity, world frame"),
        ActorObservationTerm("time_to_strike", 1, "planner", "time remaining until the strike"),
        ActorObservationTerm("swing_side", 1, "planner", "forehand (+1) / backhand (-1), locked per task"),
    ),
)

# One contract only; keyed for lookup convenience.
CONTRACTS = {HOPE_PINGPONG.name: HOPE_PINGPONG}


def resolve_actor_observation_contract(name: str | None) -> ActorObservationContract | None:
    """Resolve a contract by name. ``None``/empty returns ``None``; unknown names raise."""
    if name is None:
        return None
    key = str(name).strip()
    if not key:
        return None
    if key not in CONTRACTS:
        known = ", ".join(sorted(CONTRACTS))
        raise ValueError(f"Unknown actor observation contract '{name}'. Known values: {known}")
    return CONTRACTS[key]


def format_layout(layout: Iterable[tuple[str, int]]) -> str:
    return ", ".join(f"{name}:{dim}" for name, dim in layout)


def policy_layout_from_env(env) -> tuple[tuple[str, int], ...]:
    """Read the live (term_name, dim) layout of the environment's ``policy`` observation group."""
    observation_manager = env.observation_manager
    names = list(observation_manager.active_terms["policy"])
    dims = observation_manager.group_obs_term_dim["policy"]
    flat_dims = []
    for dim in dims:
        if isinstance(dim, (tuple, list)):
            flat_dims.append(int(dim[0]))
        else:
            flat_dims.append(int(dim))
    return tuple(zip(names, flat_dims))


def total_policy_dim_from_env(env) -> int:
    total = env.observation_manager.group_obs_dim["policy"]
    if isinstance(total, (tuple, list)):
        return int(total[0])
    return int(total)


def validate_actor_observation_contract(
    env, expected: str | ActorObservationContract = HOPE_PINGPONG
) -> ActorObservationContract:
    """Assert the env's ``policy`` group matches ``expected`` exactly (order, dims, total)."""
    contract = expected if isinstance(expected, ActorObservationContract) else resolve_actor_observation_contract(expected)
    if contract is None:
        raise ValueError("Expected actor observation contract must not be empty.")
    layout = policy_layout_from_env(env)
    total_dim = total_policy_dim_from_env(env)
    if layout != contract.layout or total_dim != contract.total_dim:
        raise ValueError(
            "Actor observation contract mismatch.\n"
            f"Expected {contract.name} ({contract.total_dim}D): {format_layout(contract.layout)}\n"
            f"Actual   ({total_dim}D): {format_layout(layout)}"
        )
    return contract
