"""Agibot A3 — the single HOPE whole-body task.

One environment config, :class:`HOPEPingPongEnvCfg`, wiring:

* motion imitation (:class:`MotionCommand`) over a forehand + backhand clip pair (clip 0 / clip 1),
  ``wrap_teleport=False`` so the policy physically transitions between swings (continuous rally);
* the ping-pong goal (:class:`RacketTargetCommand`): sampled racket target pos/vel + time-to-strike +
  swing side, a fixed startup station, and a no-spin outgoing-ball evaluation for the return rewards;
* the 111-D actor observation (``hope_pingpong`` contract) and a privileged critic that adds
  the 62-D reference joint stream, reference errors, and the actual racket FK state (value function only);
* the eleven reward terms with illustrative example weights;
* the clamped joint-position residual action (passive head);
* physical-fall / time-out terminations and light domain randomization.

Control runs at 50 Hz. The motion clips default to the placeholder examples under
``hope_training/motions/preprocessed`` — replace them with your own retargeted clips.
"""

from __future__ import annotations

import os

from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.utils import configclass
from isaaclab.utils.noise import AdditiveUniformNoiseCfg as Unoise

import whole_body_tracking.tasks.tracking.mdp as mdp
from whole_body_tracking.robots.agibot_a3 import (
    A3_ANCHOR_BODY,
    A3_FEET_BODIES,
    A3_TRACKED_BODIES,
    A3_UPPER_TRACKED,
    AGIBOT_A3_CFG,
    AGIBOT_A3_PASSIVE_HEAD_JOINT_NAMES,
)
from whole_body_tracking.tasks.tracking.tracking_env_cfg import MySceneCfg
from whole_body_tracking.utils.action_adapter_config import load_action_adapter_config


def _find_motion_clip(name: str) -> str:
    """Locate a placeholder clip under ``hope_training/motions/preprocessed`` (walk up from here)."""
    d = os.path.dirname(os.path.abspath(__file__))
    for _ in range(14):
        cand = os.path.join(d, "hope_training", "motions", "preprocessed", name)
        if os.path.exists(cand):
            return cand
        parent = os.path.dirname(d)
        if parent == d:
            break
        d = parent
    # Fall back to a relative path; the task YAML can override motion_file explicitly.
    return os.path.join("hope_training", "motions", "preprocessed", name)


FOREHAND_CLIP = _find_motion_clip("hope_forehand.npz")
BACKHAND_CLIP = _find_motion_clip("hope_backhand.npz")


@configclass
class CommandsCfg:
    """Motion imitation + racket target commands."""

    motion = mdp.MotionCommandCfg(
        asset_name="robot",
        resampling_time_range=(1.0e9, 1.0e9),
        debug_vis=False,
        anchor_body_name=A3_ANCHOR_BODY,
        body_names=A3_TRACKED_BODIES,
        motion_file=[FOREHAND_CLIP, BACKHAND_CLIP],  # clip 0 = forehand, clip 1 = backhand
        wrap_teleport=False,
        stand_start_prob=0.25,
        stand_start_min_hold=25,
        hold_steps_range=(0, 100),
        pose_range={"x": (-0.05, 0.05), "y": (-0.05, 0.05), "z": (-0.01, 0.01),
                    "roll": (-0.1, 0.1), "pitch": (-0.1, 0.1), "yaw": (-0.2, 0.2)},
        velocity_range={"x": (-0.5, 0.5), "y": (-0.5, 0.5), "z": (-0.2, 0.2),
                        "roll": (-0.52, 0.52), "pitch": (-0.52, 0.52), "yaw": (-0.78, 0.78)},
        joint_position_range=(-0.1, 0.1),
    )

    racket_target = mdp.RacketTargetCommandCfg(
        asset_name="robot",
        motion_command_name="motion",
        debug_vis=False,
        mount_normal_axis=1,          # racket-local +Y blade face
        mount_normal_sign_per_clip=(1.0, -1.0),  # forehand/backhand strike with opposite faces
        strike_phase_per_clip=(0.5, 0.5),         # example: placeholder clips strike mid-clip
        strike_window_s=0.12,
        # STATION-RELATIVE racket target boxes (x forward reach, y swing-side band, z absolute height).
        # Example values — tune to your own clips' natural strike points.
        racket_pos_range_per_clip=(
            ((0.45, 0.55), (-0.55, -0.15), (0.70, 1.00)),  # forehand (paddle on the -y side)
            ((0.45, 0.55), (0.15, 0.55), (0.85, 1.15)),    # backhand (+y side)
        ),
        racket_vel_range_per_clip=(
            ((1.0, 2.0), (0.5, 1.5), (0.2, 1.0)),    # forehand
            ((1.5, 2.5), (-1.5, -0.5), (0.0, 0.7)),  # backhand
        ),
        feet_body_names=tuple(A3_FEET_BODIES),
    )


@configclass
class ActionsCfg:
    """31-D clamped joint-position residual action (passive head)."""

    joint_pos = mdp.ClampedJointPositionActionCfg(
        asset_name="robot",
        joint_names=[".*"],
        use_default_offset=True,
        passive_joint_names=AGIBOT_A3_PASSIVE_HEAD_JOINT_NAMES,
    )


@configclass
class ObservationsCfg:
    """111-D actor observation + privileged critic."""

    @configclass
    class PolicyCfg(ObsGroup):
        # Order is fixed — it is the hope_pingpong observation contract.
        base_ang_vel = ObsTerm(func=mdp.base_ang_vel, noise=Unoise(n_min=-0.2, n_max=0.2))
        joint_pos = ObsTerm(func=mdp.joint_pos_rel, noise=Unoise(n_min=-0.01, n_max=0.01))
        joint_vel = ObsTerm(func=mdp.joint_vel_rel, noise=Unoise(n_min=-0.5, n_max=0.5))
        last_action = ObsTerm(func=mdp.applied_last_action, params={"action_name": "joint_pos"})
        projected_gravity = ObsTerm(func=mdp.projected_gravity, noise=Unoise(n_min=-0.05, n_max=0.05))
        base_forward_xy = ObsTerm(
            func=mdp.base_forward_xy, params={"command_name": "racket_target"}, noise=Unoise(n_min=-0.02, n_max=0.02)
        )
        fixed_station_error_xy = ObsTerm(
            func=mdp.fixed_station_error_xy, params={"command_name": "racket_target"}, noise=Unoise(n_min=-0.03, n_max=0.03)
        )
        racket_target_rel_base = ObsTerm(
            func=mdp.racket_target_rel_base, params={"command_name": "racket_target"}, noise=Unoise(n_min=-0.02, n_max=0.02)
        )
        racket_target_vel_w = ObsTerm(func=mdp.racket_target_vel_w, params={"command_name": "racket_target"})
        time_to_strike = ObsTerm(func=mdp.time_to_strike, params={"command_name": "racket_target"})
        swing_side = ObsTerm(func=mdp.swing_side, params={"command_name": "racket_target"})

        def __post_init__(self):
            self.enable_corruption = True
            self.concatenate_terms = True

    @configclass
    class CriticCfg(ObsGroup):
        # Actor terms (noise-free) ...
        base_ang_vel = ObsTerm(func=mdp.base_ang_vel)
        joint_pos = ObsTerm(func=mdp.joint_pos_rel)
        joint_vel = ObsTerm(func=mdp.joint_vel_rel)
        last_action = ObsTerm(func=mdp.applied_last_action, params={"action_name": "joint_pos"})
        projected_gravity = ObsTerm(func=mdp.projected_gravity)
        base_forward_xy = ObsTerm(func=mdp.base_forward_xy, params={"command_name": "racket_target"})
        fixed_station_error_xy = ObsTerm(func=mdp.fixed_station_error_xy, params={"command_name": "racket_target"})
        racket_target_rel_base = ObsTerm(func=mdp.racket_target_rel_base, params={"command_name": "racket_target"})
        racket_target_vel_w = ObsTerm(func=mdp.racket_target_vel_w, params={"command_name": "racket_target"})
        time_to_strike = ObsTerm(func=mdp.time_to_strike, params={"command_name": "racket_target"})
        swing_side = ObsTerm(func=mdp.swing_side, params={"command_name": "racket_target"})
        # ... plus privileged (sim-only) signals for the value function.
        base_lin_vel = ObsTerm(func=mdp.base_lin_vel)
        motion_command = ObsTerm(func=mdp.generated_commands, params={"command_name": "motion"})  # 62-D ref stream
        motion_anchor_pos_b = ObsTerm(func=mdp.motion_anchor_pos_b, params={"command_name": "motion"})
        motion_anchor_ori_b = ObsTerm(func=mdp.motion_anchor_ori_b, params={"command_name": "motion"})
        robot_body_pos_b = ObsTerm(func=mdp.robot_body_pos_b, params={"command_name": "motion"})
        robot_body_ori_b = ObsTerm(func=mdp.robot_body_ori_b, params={"command_name": "motion"})
        racket_pos_b = ObsTerm(func=mdp.racket_pos_b, params={"command_name": "racket_target"})
        racket_lin_vel_w = ObsTerm(func=mdp.racket_lin_vel_w, params={"command_name": "racket_target"})
        racket_normal_w = ObsTerm(func=mdp.racket_normal_w, params={"command_name": "racket_target"})
        racket_target_normal_w = ObsTerm(func=mdp.racket_target_normal_w, params={"command_name": "racket_target"})
        episode_time_left = ObsTerm(func=mdp.episode_time_left)

        def __post_init__(self):
            self.concatenate_terms = True

    policy: PolicyCfg = PolicyCfg()
    critic: CriticCfg = CriticCfg()


@configclass
class RewardsCfg:
    """The eleven reward terms. Weights/stds are illustrative examples — tune them."""

    # 1. upright / balance
    upright = RewTerm(func=mdp.flat_orientation_l2, weight=-1.0)
    # 2. forehand/backhand sample imitation (upper body, swing-gated)
    imitation = RewTerm(
        func=mdp.sample_imitation,
        weight=1.0,
        params={"command_name": "motion", "std_pos": 0.3, "std_ori": 0.4, "body_names": A3_UPPER_TRACKED},
    )
    # 3. racket position (strike window)
    racket_position = RewTerm(
        func=mdp.racket_position, weight=4.0, params={"command_name": "racket_target", "std": 0.12}
    )
    # 4. racket velocity (strike window)
    racket_velocity = RewTerm(
        func=mdp.racket_velocity, weight=2.0, params={"command_name": "racket_target", "std": 0.6}
    )
    # 5. simplified blade direction (strike window)
    blade_direction = RewTerm(
        func=mdp.racket_blade_direction, weight=1.0, params={"command_name": "racket_target", "std": 0.3}
    )
    # 6. actual ball contact (one-shot at strike)
    ball_contact = RewTerm(func=mdp.ball_contact, weight=2.0, params={"command_name": "racket_target"})
    # 7. net crossing (one-shot at strike)
    net_cross = RewTerm(func=mdp.ball_net_cross, weight=2.0, params={"command_name": "racket_target"})
    # 8. opponent-half first bounce (one-shot at strike)
    opponent_bounce = RewTerm(func=mdp.ball_opponent_bounce, weight=4.0, params={"command_name": "racket_target"})
    # 9. in-place follow-through / recovery
    follow_through_recovery = RewTerm(
        func=mdp.follow_through_recovery,
        weight=1.0,
        params={"command_name": "racket_target", "std": 0.5, "station_std": 0.3},
    )
    # 10. action smoothness
    action_rate = RewTerm(func=mdp.action_rate_l2, weight=-0.1)
    # 11. joint-limit regularization
    joint_limit = RewTerm(
        func=mdp.joint_pos_limits, weight=-10.0, params={"asset_cfg": SceneEntityCfg("robot", joint_names=[".*"])}
    )


@configclass
class TerminationsCfg:
    """Time-out and physical-fall resets (ordinary env lifecycle, not a gate)."""

    time_out = DoneTerm(func=mdp.time_out, time_out=True)
    base_tilted = DoneTerm(func=mdp.base_tilted, params={"threshold": 0.8})
    base_too_low = DoneTerm(func=mdp.base_too_low, params={"min_height": 0.5})


@configclass
class EventCfg:
    """Light domain randomization for sim-to-real robustness."""

    physics_material = EventTerm(
        func=mdp.randomize_rigid_body_material,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=".*"),
            "static_friction_range": (0.3, 1.6),
            "dynamic_friction_range": (0.3, 1.2),
            "restitution_range": (0.0, 0.5),
            "num_buckets": 64,
        },
    )
    base_com = EventTerm(
        func=mdp.randomize_rigid_body_com,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=A3_ANCHOR_BODY),
            "com_range": {"x": (-0.025, 0.025), "y": (-0.05, 0.05), "z": (-0.05, 0.05)},
        },
    )
    link_mass = EventTerm(
        func=mdp.randomize_rigid_body_mass,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=".*"),
            "mass_distribution_params": (0.9, 1.1),
            "operation": "scale",
            "distribution": "uniform",
            "recompute_inertia": True,
        },
    )
    joint_default_pos = EventTerm(
        func=mdp.randomize_joint_default_pos,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", joint_names=[".*"]),
            "pos_distribution_params": (-0.01, 0.01),
            "operation": "add",
        },
    )
    pd_gains = EventTerm(
        func=mdp.randomize_actuator_gains,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("robot", joint_names=[".*"]),
            "stiffness_distribution_params": (0.9, 1.1),
            "damping_distribution_params": (0.9, 1.1),
            "operation": "scale",
            "distribution": "log_uniform",
        },
    )


@configclass
class HOPEPingPongEnvCfg(ManagerBasedRLEnvCfg):
    """The single public HOPE task (gym id ``HOPE-PingPong-AgibotA3-v0``)."""

    scene: MySceneCfg = MySceneCfg(num_envs=4096, env_spacing=2.5)
    observations: ObservationsCfg = ObservationsCfg()
    actions: ActionsCfg = ActionsCfg()
    commands: CommandsCfg = CommandsCfg()
    rewards: RewardsCfg = RewardsCfg()
    terminations: TerminationsCfg = TerminationsCfg()
    events: EventCfg = EventCfg()

    def __post_init__(self):
        # 50 Hz control (decimation 4 over a 200 Hz physics step).
        self.decimation = 4
        self.episode_length_s = 10.0
        self.sim.dt = 0.005
        self.sim.render_interval = self.decimation
        self.sim.physics_material = self.scene.terrain.physics_material
        self.sim.physx.gpu_max_rigid_patch_count = 10 * 2**15

        # Robot + shared action adapter. default_q / action_scale / joint clamp all come from the
        # ONE shared config the deploy runner reads (action_adapter.yaml), so the same raw action
        # produces the same joint targets — and the same joint_pos observation — in training and
        # deployment (see tests/test_action_adapter_parity.py).
        adapter = load_action_adapter_config()
        self.scene.robot = AGIBOT_A3_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
        self.scene.robot.init_state.joint_pos = adapter.default_q_by_name()
        self.actions.joint_pos.scale = adapter.action_scale_by_name()
        self.actions.joint_pos.position_clamp = adapter.position_clamp_by_name()

        self.viewer.eye = (1.5, 1.5, 1.5)
        self.viewer.origin_type = "asset_root"
        self.viewer.asset_name = "robot"
