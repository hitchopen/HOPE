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
    obs_mode: str
    total_dim: int
    terms: tuple[ActorObservationTerm, ...]
    legacy_names: tuple[str, ...] = ()

    @property
    def layout(self) -> tuple[tuple[str, int], ...]:
        return tuple((term.name, term.dim) for term in self.terms)


FULL = ActorObservationContract(
    name="full",
    obs_mode="full",
    total_dim=180,
    terms=(
        ActorObservationTerm("command", 62, "reference_clip", "reference joint positions and velocities"),
        ActorObservationTerm(
            "motion_anchor_pos_b",
            3,
            "sim_or_localizer",
            "reference torso position error; requires world base position",
        ),
        ActorObservationTerm(
            "motion_anchor_ori_b",
            6,
            "imu_plus_reference_clip",
            "reference torso orientation error",
        ),
        ActorObservationTerm("base_ang_vel", 3, "imu", "pelvis angular velocity in the body frame"),
        ActorObservationTerm("joint_pos", 31, "encoders", "joint position offset from default"),
        ActorObservationTerm("joint_vel", 31, "encoders", "joint velocities"),
        ActorObservationTerm("actions", 31, "runtime_state", "previous policy action"),
        ActorObservationTerm("projected_gravity", 3, "imu", "gravity direction in the base frame"),
        ActorObservationTerm(
            "base_target_pos_b",
            2,
            "planner_plus_localizer",
            "desired base XY target relative to the current base",
        ),
        ActorObservationTerm(
            "racket_target_pos_b",
            3,
            "planner_plus_localizer",
            "desired racket position relative to the current base",
        ),
        ActorObservationTerm("racket_target_vel_w", 3, "planner", "desired racket velocity in the world frame"),
        ActorObservationTerm("time_to_strike", 1, "reference_clock", "time remaining until strike"),
        ActorObservationTerm("swing_type", 1, "reference_clip", "forehand (+1) or backhand (-1) flag"),
    ),
)

DEPLOY_PARITY = ActorObservationContract(
    name="deploy_parity",
    obs_mode="deploy_parity",
    total_dim=175,
    legacy_names=("real_sensor_only",),
    terms=(
        ActorObservationTerm("command", 62, "reference_clip", "reference joint positions and velocities"),
        ActorObservationTerm(
            "motion_anchor_ori_b",
            6,
            "imu_plus_reference_clip",
            "reference torso orientation error",
        ),
        ActorObservationTerm("base_ang_vel", 3, "imu", "pelvis angular velocity in the body frame"),
        ActorObservationTerm("joint_pos", 31, "encoders", "joint position offset from default"),
        ActorObservationTerm("joint_vel", 31, "encoders", "joint velocities"),
        ActorObservationTerm("actions", 31, "runtime_state", "previous policy action"),
        ActorObservationTerm("projected_gravity", 3, "imu", "gravity direction in the base frame"),
        ActorObservationTerm(
            "racket_target_pos_b",
            3,
            "planner_plus_racket_fk",
            "desired racket position relative to the current racket FK; no world base position",
        ),
        ActorObservationTerm("racket_target_vel_w", 3, "planner", "desired racket velocity in the world frame"),
        ActorObservationTerm("time_to_strike", 1, "reference_clock", "time remaining until strike"),
        ActorObservationTerm("swing_type", 1, "reference_clip", "forehand (+1) or backhand (-1) flag"),
    ),
)

HITTER_FOOTWORK = ActorObservationContract(
    name="hitter_footwork",
    obs_mode="hitter_footwork",
    total_dim=177,
    terms=(
        ActorObservationTerm("command", 62, "reference_clip", "reference joint positions and velocities"),
        ActorObservationTerm(
            "motion_anchor_ori_b",
            6,
            "imu_plus_reference_clip",
            "reference torso orientation error",
        ),
        ActorObservationTerm("base_ang_vel", 3, "imu", "pelvis angular velocity in the body frame"),
        ActorObservationTerm("joint_pos", 31, "encoders", "joint position offset from default"),
        ActorObservationTerm("joint_vel", 31, "encoders", "joint velocities"),
        ActorObservationTerm("actions", 31, "runtime_state", "previous policy action"),
        ActorObservationTerm("projected_gravity", 3, "imu", "gravity direction in the base frame"),
        ActorObservationTerm(
            "base_target_pos_b",
            2,
            "planner_plus_mocap",
            "desired base XY station relative to the current base (yaw-heading frame); mocap base "
            "position at deploy — relative Δ only, Δ=0 graceful fallback on mocap dropout",
        ),
        ActorObservationTerm(
            "racket_target_pos_b",
            3,
            "planner_plus_racket_fk",
            "desired racket position relative to the current racket FK; no world base position",
        ),
        ActorObservationTerm("racket_target_vel_w", 3, "planner", "desired racket velocity in the world frame"),
        ActorObservationTerm("time_to_strike", 1, "reference_clock", "time remaining until strike"),
        ActorObservationTerm("swing_type", 1, "reference_clip", "forehand (+1) or backhand (-1) flag"),
    ),
)

HITTER_PURE = ActorObservationContract(
    name="hitter_pure",
    obs_mode="hitter_pure",
    total_dim=110,
    terms=(
        # HITTER (arXiv:2508.21043) Table I EXACT structure, sized for the A3's 31 joints:
        # proprioception + goal ONLY. The 62-D reference joint stream is CRITIC-ONLY in the paper
        # (and here); swing_type is not observed anywhere (deploy infers fh/bh outside the policy,
        # §V-B-3). Target vectors are WORLD-frame with the explicit base forward vector e_base,x.
        ActorObservationTerm("base_ang_vel", 3, "imu", "pelvis angular velocity in the body frame"),
        ActorObservationTerm("joint_pos", 31, "encoders", "joint position offset from default"),
        ActorObservationTerm("joint_vel", 31, "encoders", "joint velocities"),
        ActorObservationTerm("actions", 31, "runtime_state", "previous policy action"),
        ActorObservationTerm("projected_gravity", 3, "imu", "gravity direction in the base frame"),
        ActorObservationTerm(
            "base_forward_xy", 2, "imu_yaw_aligned", "base forward unit vector e_base,x (world xy)"
        ),
        ActorObservationTerm(
            "base_target_delta_xy",
            2,
            "planner_plus_mocap",
            "target base position minus current base position, world xy (Δ=0 on mocap dropout)",
        ),
        ActorObservationTerm(
            "racket_target_rel_base",
            3,
            "planner_plus_mocap",
            "target racket position relative to the base, world frame",
        ),
        ActorObservationTerm("racket_target_vel_w", 3, "planner", "desired racket velocity in the world frame"),
        ActorObservationTerm("time_to_strike", 1, "reference_clock", "time remaining until strike"),
    ),
)

HITTER_PURE_V15 = ActorObservationContract(
    name="hitter_pure_v15",
    obs_mode="hitter_pure_v15",
    total_dim=118,
    terms=(
        ActorObservationTerm("base_ang_vel", 3, "imu", "pelvis angular velocity in the body frame"),
        ActorObservationTerm("joint_pos", 31, "encoders", "joint position offset from default"),
        ActorObservationTerm("joint_vel", 31, "encoders", "joint velocities"),
        ActorObservationTerm(
            "actions",
            31,
            "runtime_executed_qdes",
            "normalized q_des actually published by the previous policy tick",
        ),
        ActorObservationTerm("projected_gravity", 3, "imu", "gravity direction in the base frame"),
        ActorObservationTerm(
            "base_forward_xy", 2, "imu_yaw_aligned", "base forward unit vector e_base,x (world xy)"
        ),
        ActorObservationTerm(
            "base_target_delta_xy",
            2,
            "planner_plus_mocap",
            "target base position minus held mocap base position, world xy",
        ),
        ActorObservationTerm(
            "racket_target_rel_base",
            3,
            "planner_plus_mocap",
            "target racket position relative to the held mocap base position, world frame",
        ),
        ActorObservationTerm("racket_target_vel_w", 3, "planner", "desired racket velocity in the world frame"),
        ActorObservationTerm("time_to_strike", 1, "reference_clock", "time remaining until strike"),
        ActorObservationTerm(
            "base_velocity_xy",
            2,
            "mocap_position_estimator",
            "filtered world-frame XY velocity differentiated from received base positions",
        ),
        ActorObservationTerm(
            "localization_age",
            1,
            "runtime_receive_clock",
            "local base-position receipt age normalized by the configured stale threshold",
        ),
        ActorObservationTerm(
            "desired_lateral_velocity",
            1,
            "runtime_finite_gait_scheduler",
            "world-Y velocity generated once when a new HITTER station is accepted",
        ),
        ActorObservationTerm(
            "gait_clock",
            2,
            "runtime_finite_gait_scheduler",
            "left/right HUGWBC sine clocks; zero in stand and strike modes",
        ),
        ActorObservationTerm(
            "locomotion_mode",
            1,
            "runtime_finite_gait_scheduler",
            "+1 finite STEP, 0 hold/STAND, -1 racket swing or recovery",
        ),
        ActorObservationTerm(
            "upper_intervention",
            1,
            "runtime_constant_zero",
            "HUGWBC training intervention flag; always zero on real hardware",
        ),
    ),
)

CONTRACTS = {
    FULL.name: FULL,
    DEPLOY_PARITY.name: DEPLOY_PARITY,
    HITTER_FOOTWORK.name: HITTER_FOOTWORK,
    HITTER_PURE.name: HITTER_PURE,
    HITTER_PURE_V15.name: HITTER_PURE_V15,
    FULL.obs_mode: FULL,
    DEPLOY_PARITY.obs_mode: DEPLOY_PARITY,
    HITTER_FOOTWORK.obs_mode: HITTER_FOOTWORK,
    HITTER_PURE.obs_mode: HITTER_PURE,
    HITTER_PURE_V15.obs_mode: HITTER_PURE_V15,
    **{alias: DEPLOY_PARITY for alias in DEPLOY_PARITY.legacy_names},
}


def resolve_actor_observation_contract(name: str | None) -> ActorObservationContract | None:
    if name is None:
        return None
    key = str(name).strip()
    if not key:
        return None
    if key not in CONTRACTS:
        known = ", ".join(sorted(CONTRACTS))
        raise ValueError(f"Unknown actor observation contract '{name}'. Known values: {known}")
    return CONTRACTS[key]


def policy_layout_from_env(env) -> tuple[tuple[str, int], ...]:
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


def infer_actor_observation_contract(env) -> ActorObservationContract | None:
    layout = policy_layout_from_env(env)
    total_dim = total_policy_dim_from_env(env)
    for contract in (FULL, DEPLOY_PARITY, HITTER_FOOTWORK, HITTER_PURE, HITTER_PURE_V15):
        if layout == contract.layout and total_dim == contract.total_dim:
            return contract
    return None


def validate_actor_observation_contract(env, expected: str | ActorObservationContract) -> ActorObservationContract:
    contract = (
        expected
        if isinstance(expected, ActorObservationContract)
        else resolve_actor_observation_contract(expected)
    )
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


def removed_terms_vs_full(contract: str | ActorObservationContract) -> tuple[ActorObservationTerm, ...]:
    resolved = contract if isinstance(contract, ActorObservationContract) else resolve_actor_observation_contract(contract)
    if resolved is None:
        return ()
    removed = []
    active = {term.name for term in resolved.terms}
    for term in FULL.terms:
        if term.name not in active:
            removed.append(term)
    return tuple(removed)


def format_layout(layout: Iterable[tuple[str, int]]) -> str:
    return ", ".join(f"{name}:{dim}" for name, dim in layout)
