"""Manager-based Isaac Lab environment for a table-tennis match scene.

This builds the scene in the world frame (see :mod:`.geometry`): floor, table, net (+ posts), a dynamic
ball, and an (abstract) humanoid robot. PhysX integrates gravity and resolves all rigid-body contacts
(ball<->table / net / floor / racket); the missing aerodynamic drag is added per physics substep by
:class:`.table_tennis_env.TableTennisEnv` using :mod:`.ball` (no-spin quadratic drag).

The configuration is intentionally **modular**:

* :mod:`.geometry` owns every dimension / landmark / material constant (all read from
  ``configs/ball_physics.yaml``).
* the ``build_*`` helpers below each construct one scene asset, so the ball model, table geometry, and
  decorations can be swapped independently (e.g. replace the box table with a USD table asset).
* the robot is left ``MISSING`` here and filled by a robot-specific subclass
  (:mod:`.config.agibot_a3.table_tennis_env_cfg`), so the same scene/MDP works for any humanoid.
* observations / rewards / events / terminations are standard Isaac Lab managers.
"""

from __future__ import annotations

import os
from dataclasses import MISSING

import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg, AssetBaseCfg, RigidObjectCfg
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.utils import configclass
from isaaclab.utils.noise import AdditiveUniformNoiseCfg as Unoise

from . import geometry
from . import mdp
from .ball import BallAerodynamicsCfg
from .geometry import BounceMaterials, OutOfBoundsBox, ServeConfig

##
# Scene asset builders (modular — one helper per prim, driven by geometry constants).
##

# Visual-only realistic table+net mesh (third-party, MIT-licensed; see
# table_usd/LICENSE-PACE-ICRA2026-MIT.txt). We point at the *base* layer, which carries pure geometry +
# materials and NO PhysX colliders, so it is overlaid for looks only — bounce physics still comes
# entirely from the cuboid colliders below. The USD lives in the version-controlled ``table_usd/`` dir
# and is resolved relative to this module so the task stays importable wherever the repo is checked out.
_TABLE_USD_PATH = os.path.join(
    os.path.dirname(__file__),
    "table_usd",
    "table",
    "configuration",
    "ping_pong_table_urdf_base.usd",
)


def _surface_material(restitution: float, static_friction: float, dynamic_friction: float) -> sim_utils.RigidBodyMaterialCfg:
    """PhysX contact material with multiplicative combine, so ball<->surface restitution is the product
    of the two materials' coefficients (see :class:`geometry.BounceMaterials`)."""
    return sim_utils.RigidBodyMaterialCfg(
        static_friction=static_friction,
        dynamic_friction=dynamic_friction,
        restitution=restitution,
        friction_combine_mode="multiply",
        restitution_combine_mode="multiply",
    )


def build_floor_cfg(mats: BounceMaterials) -> AssetBaseCfg:
    """Global static floor at world z = -TABLE_HEIGHT. Shared by every environment."""
    return AssetBaseCfg(
        prim_path="/World/floor",
        init_state=AssetBaseCfg.InitialStateCfg(pos=(0.0, 0.0, geometry.FLOOR_Z - 0.05)),
        spawn=sim_utils.CuboidCfg(
            size=(100.0, 100.0, 0.1),
            collision_props=sim_utils.CollisionPropertiesCfg(collision_enabled=True),
            physics_material=_surface_material(
                mats.floor_restitution, mats.floor_static_friction, mats.floor_dynamic_friction
            ),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.18, 0.20, 0.22), roughness=0.9),
        ),
    )


def build_table_top_cfg(mats: BounceMaterials, visible: bool = True) -> AssetBaseCfg:
    """The blue table top. Its top face sits exactly at world z = 0 (the table surface).

    ``visible=False`` keeps the collider but hides the box geometry, so a realistic USD mesh can be
    overlaid on top without z-fighting against the plain cuboid."""
    return AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Table",
        init_state=AssetBaseCfg.InitialStateCfg(pos=geometry.table_top_center()),
        spawn=sim_utils.CuboidCfg(
            size=geometry.table_top_size(),
            visible=visible,
            collision_props=sim_utils.CollisionPropertiesCfg(collision_enabled=True),
            physics_material=_surface_material(
                mats.table_restitution, mats.table_static_friction, mats.table_dynamic_friction
            ),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.0, 0.32, 0.55), roughness=0.5),  # ITTF blue
        ),
    )


def build_net_cfg(mats: BounceMaterials, visible: bool = True) -> AssetBaseCfg:
    """Thin collidable net slab across the table at x = NET_X, spanning the table width + overhang.

    ``visible=False`` keeps the collider but hides the slab so the USD net mesh can stand in for it."""
    return AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Net",
        init_state=AssetBaseCfg.InitialStateCfg(pos=geometry.net_center()),
        spawn=sim_utils.CuboidCfg(
            size=geometry.net_size(),
            visible=visible,
            collision_props=sim_utils.CollisionPropertiesCfg(collision_enabled=True),
            physics_material=_surface_material(
                mats.net_restitution, mats.net_static_friction, mats.net_dynamic_friction
            ),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.05, 0.05, 0.08), opacity=0.55),
        ),
    )


def build_ball_cfg(mats: BounceMaterials) -> RigidObjectCfg:
    """The dynamic 40 mm ball. PhysX handles gravity + contacts; no-spin drag is added per substep by
    the environment. ``linear_damping`` is 0 so PhysX does not double-count drag when it is enabled."""
    return RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/Ball",
        # Default spawn over the P2 half; the serve-reset event overrides this on every reset.
        init_state=RigidObjectCfg.InitialStateCfg(pos=(geometry.P2_HALF_CENTER[0], geometry.P2_HALF_CENTER[1], 0.35)),
        spawn=sim_utils.SphereCfg(
            radius=geometry.BALL_RADIUS,
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                disable_gravity=False,
                linear_damping=0.0,
                angular_damping=0.0,
                max_linear_velocity=1000.0,  # m/s
                max_depenetration_velocity=10.0,
                solver_position_iteration_count=16,
                solver_velocity_iteration_count=4,
            ),
            mass_props=sim_utils.MassPropertiesCfg(mass=geometry.BALL_MASS),
            collision_props=sim_utils.CollisionPropertiesCfg(collision_enabled=True),
            physics_material=_surface_material(
                mats.ball_restitution, mats.ball_static_friction, mats.ball_dynamic_friction
            ),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.95, 0.55, 0.05), roughness=0.6),  # orange ITTF
        ),
    )


def build_net_post_cfg(prim_path: str, y: float, visible: bool = True) -> AssetBaseCfg:
    """Visual-only net post (no collision) at one end of the net. ``visible=False`` hides it (the USD
    mesh already models the posts)."""
    post_h = geometry.NET_HEIGHT + 0.02
    return AssetBaseCfg(
        prim_path=prim_path,
        init_state=AssetBaseCfg.InitialStateCfg(pos=(geometry.NET_X, y, post_h / 2.0)),
        spawn=sim_utils.CuboidCfg(
            size=(0.02, 0.02, post_h),
            visible=visible,
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.85, 0.85, 0.85)),
        ),
    )


def build_center_line_cfg(visible: bool = True) -> AssetBaseCfg:
    """Visual-only white center line running along the table length. ``visible=False`` hides it (the USD
    mesh already paints the lines)."""
    return AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/CenterLine",
        init_state=AssetBaseCfg.InitialStateCfg(
            pos=(geometry.TABLE_LENGTH / 2.0, -geometry.TABLE_WIDTH / 2.0, geometry.LINE_THICKNESS / 2.0)
        ),
        spawn=sim_utils.CuboidCfg(
            size=(geometry.TABLE_LENGTH, geometry.LINE_WIDTH, geometry.LINE_THICKNESS),
            visible=visible,
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.95, 0.95, 0.95)),
        ),
    )


def build_table_usd_visual_cfg() -> AssetBaseCfg:
    """Realistic table+net+posts USD mesh, overlaid as a **visual only** prim on the invisible cuboid
    colliders. No physics is spawned from it (the base USD layer carries no PhysX colliders, and we pass
    no collision/rigid props), so PhysX ignores it and the cuboids remain the single source of bounce
    physics.

    Frame alignment: the USD models the floor at its local z = 0, the playing surface at z = TABLE_HEIGHT
    and the net top above it, centered horizontally at its local (x, y) = (0, 0). Translating its local
    origin to the world floor point directly under the table center —
    ``(TABLE_LENGTH/2, -TABLE_WIDTH/2, FLOOR_Z)`` — lands the mesh's playing surface exactly on world
    z = 0 and its net plane on x = NET_X, matching the cuboids. Orientation is identity (both frames are
    Z-up with table length along X).

    NOTE: the USD mesh is slightly wider than the ITTF/cuboid table, so the visual table is a touch wider
    than its collider — purely cosmetic, bounces follow the cuboids. This asset is visual sugar; for
    large-scale headless training you may drop it (set ``scene.table_visual = None``) to save memory."""
    return AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/TableVisual",
        init_state=AssetBaseCfg.InitialStateCfg(
            pos=(geometry.TABLE_CENTER[0], geometry.TABLE_CENTER[1], geometry.FLOOR_Z),
            rot=(1.0, 0.0, 0.0, 0.0),
        ),
        spawn=sim_utils.UsdFileCfg(usd_path=_TABLE_USD_PATH),
    )


##
# Scene definition.
##

# Shared contact-material parameters (scalars only; each builder constructs its own material object).
_MATS = BounceMaterials()


@configclass
class TableTennisSceneCfg(InteractiveSceneCfg):
    """Full table-tennis court in the world frame. Robot is filled by a robot-specific subclass.

    Each environment is an independent court whose local origin coincides with the world origin (the
    near-side left corner of the table surface, at table-surface height). The floor is global; the
    table, net, lines, ball and robot are cloned per environment.
    """

    # Static world. The table / net / posts / center line are kept as **invisible cuboid colliders**
    # (visible=False) so they still own all bounce physics, while the realistic USD mesh (table_visual)
    # is overlaid for looks. The floor stays visible (the USD models only the table, not the ground).
    floor: AssetBaseCfg = build_floor_cfg(_MATS)
    table: AssetBaseCfg = build_table_top_cfg(_MATS, visible=False)
    net: AssetBaseCfg = build_net_cfg(_MATS, visible=False)
    net_post_left: AssetBaseCfg = build_net_post_cfg("{ENV_REGEX_NS}/NetPostLeft", 0.0 + geometry.NET_OVERHANG, visible=False)
    net_post_right: AssetBaseCfg = build_net_post_cfg(
        "{ENV_REGEX_NS}/NetPostRight", -geometry.TABLE_WIDTH - geometry.NET_OVERHANG, visible=False
    )
    center_line: AssetBaseCfg = build_center_line_cfg(visible=False)

    # Visual-only realistic table+net mesh overlaid on the invisible colliders above.
    table_visual: AssetBaseCfg = build_table_usd_visual_cfg()

    # Dynamic ball.
    ball: RigidObjectCfg = build_ball_cfg(_MATS)

    # Robot — filled per robot (see config/<robot>/table_tennis_env_cfg.py).
    robot: ArticulationCfg = MISSING

    # Lights.
    light = AssetBaseCfg(
        prim_path="/World/light",
        spawn=sim_utils.DistantLightCfg(color=(0.9, 0.9, 0.9), intensity=3000.0),
    )
    sky_light = AssetBaseCfg(
        prim_path="/World/skyLight",
        spawn=sim_utils.DomeLightCfg(color=(0.2, 0.2, 0.2), intensity=1000.0),
    )


##
# MDP — commands (none), actions, observations, events, rewards, terminations.
##


@configclass
class CommandsCfg:
    """No command terms: the "task" (incoming ball) is part of the scene, observed directly."""

    pass


@configclass
class ActionsCfg:
    """Joint-position control of the robot. ``scale`` is set per robot (see robot subclass)."""

    joint_pos = mdp.JointPositionActionCfg(asset_name="robot", joint_names=[".*"], use_default_offset=True)


@configclass
class ObservationsCfg:
    """Robot proprioception + ball state. A reasonable starting point for a returner policy.

    The ball terms are scene/critic signals (privileged), not the deployed actor observation.
    """

    @configclass
    class PolicyCfg(ObsGroup):
        # Robot proprioception.
        base_lin_vel = ObsTerm(func=mdp.base_lin_vel, noise=Unoise(n_min=-0.1, n_max=0.1))
        base_ang_vel = ObsTerm(func=mdp.base_ang_vel, noise=Unoise(n_min=-0.2, n_max=0.2))
        projected_gravity = ObsTerm(func=mdp.projected_gravity, noise=Unoise(n_min=-0.05, n_max=0.05))
        joint_pos = ObsTerm(func=mdp.joint_pos_rel, noise=Unoise(n_min=-0.01, n_max=0.01))
        joint_vel = ObsTerm(func=mdp.joint_vel_rel, noise=Unoise(n_min=-0.5, n_max=0.5))
        actions = ObsTerm(func=mdp.last_action)
        # Ball state, expressed in the robot base frame.
        ball_pos_b = ObsTerm(func=mdp.ball_position_b, noise=Unoise(n_min=-0.01, n_max=0.01))
        ball_vel_b = ObsTerm(func=mdp.ball_velocity_b, noise=Unoise(n_min=-0.05, n_max=0.05))

        def __post_init__(self):
            self.enable_corruption = True
            self.concatenate_terms = True

    @configclass
    class CriticCfg(ObsGroup):
        # Privileged (noise-free) mirror for an asymmetric actor-critic.
        base_lin_vel = ObsTerm(func=mdp.base_lin_vel)
        base_ang_vel = ObsTerm(func=mdp.base_ang_vel)
        projected_gravity = ObsTerm(func=mdp.projected_gravity)
        joint_pos = ObsTerm(func=mdp.joint_pos_rel)
        joint_vel = ObsTerm(func=mdp.joint_vel_rel)
        actions = ObsTerm(func=mdp.last_action)
        ball_pos_b = ObsTerm(func=mdp.ball_position_b)
        ball_vel_b = ObsTerm(func=mdp.ball_velocity_b)

        def __post_init__(self):
            self.enable_corruption = False
            self.concatenate_terms = True

    policy: PolicyCfg = PolicyCfg()
    critic: CriticCfg = CriticCfg()


@configclass
class EventCfg:
    """Reset the robot to its standing pose and (re)serve the ball each episode."""

    reset_robot = EventTerm(
        func=mdp.reset_root_state_uniform,
        mode="reset",
        params={
            "pose_range": {"x": (-0.05, 0.05), "y": (-0.05, 0.05), "yaw": (-0.1, 0.1)},
            "velocity_range": {},
            "asset_cfg": SceneEntityCfg("robot"),
        },
    )
    reset_robot_joints = EventTerm(
        func=mdp.reset_joints_by_scale,
        mode="reset",
        params={
            "position_range": (1.0, 1.0),
            "velocity_range": (0.0, 0.0),
            "asset_cfg": SceneEntityCfg("robot"),
        },
    )
    serve_ball = EventTerm(
        func=mdp.reset_ball_serve,
        mode="reset",
        params={"serve_cfg": ServeConfig(), "asset_cfg": SceneEntityCfg("ball")},
    )


@configclass
class RewardsCfg:
    """Example rewards for a returner policy. Extend with more match objectives (racket-to-ball
    tracking, net crossing, ...) as the policy is developed."""

    alive = RewTerm(func=mdp.is_alive, weight=1.0)
    action_rate = RewTerm(func=mdp.action_rate_l2, weight=-0.01)
    # Small bonus while the ball is in flight above the table surface.
    ball_in_play = RewTerm(func=mdp.ball_above_surface, weight=0.05)
    # Bonus for the ball bouncing on the opponent (P2) half near the surface — the return objective.
    ball_opponent_half = RewTerm(func=mdp.ball_bounce_opponent_half, weight=1.0)


@configclass
class TerminationsCfg:
    """End the episode on timeout, when the ball leaves the (generous) play volume, or if the robot falls."""

    time_out = DoneTerm(func=mdp.time_out, time_out=True)
    ball_out_of_bounds = DoneTerm(
        func=mdp.ball_out_of_bounds,
        params={"bounds": OutOfBoundsBox().as_dict(), "asset_cfg": SceneEntityCfg("ball")},
    )
    robot_fell = DoneTerm(
        func=mdp.robot_base_too_low,
        params={"minimum_height": -0.1, "asset_cfg": SceneEntityCfg("robot")},
    )


@configclass
class CurriculumCfg:
    """No curriculum terms (yet)."""

    pass


##
# Environment configuration.
##


@configclass
class TableTennisEnvCfg(ManagerBasedRLEnvCfg):
    """Robot-agnostic table-tennis environment. Subclass and set ``scene.robot`` + action scale."""

    # Scene — env_spacing > table length so adjacent courts never overlap.
    scene: TableTennisSceneCfg = TableTennisSceneCfg(num_envs=1, env_spacing=6.0)

    # MDP.
    observations: ObservationsCfg = ObservationsCfg()
    actions: ActionsCfg = ActionsCfg()
    commands: CommandsCfg = CommandsCfg()
    rewards: RewardsCfg = RewardsCfg()
    terminations: TerminationsCfg = TerminationsCfg()
    events: EventCfg = EventCfg()
    curriculum: CurriculumCfg = CurriculumCfg()

    # No-spin ball drag (applied per physics substep by TableTennisEnv). Coefficients come from
    # configs/ball_physics.yaml. Set enabled=False to fly on PhysX gravity + contacts alone.
    ball_aerodynamics: BallAerodynamicsCfg = BallAerodynamicsCfg.from_physics_config(enabled=True)

    def __post_init__(self):
        # General.
        self.decimation = 4
        self.episode_length_s = 8.0
        # Simulation — a high physics rate + PhysX CCD keep the fast, light ball from tunnelling through
        # the thin racket blade / net, including at struck-return speeds where dt alone is not enough;
        # the drag callback runs at the physics rate. CCD is a scene-level PhysX flag.
        self.sim.dt = 0.0025  # 400 Hz physics
        self.sim.render_interval = self.decimation
        self.sim.gravity = (0.0, 0.0, -geometry.GRAVITY)
        self.sim.physx.enable_ccd = True
        self.sim.physx.gpu_max_rigid_patch_count = 10 * 2**15
        # Default contact material (only used by colliders that do not set their own).
        self.sim.physics_material = sim_utils.RigidBodyMaterialCfg(
            static_friction=0.8, dynamic_friction=0.8, restitution=0.0
        )
        # Viewer — fixed world view looking at the net from the P1 corner.
        self.viewer.origin_type = "world"
        self.viewer.eye = (-1.5, -3.5, 1.8)
        self.viewer.lookat = (geometry.NET_X, -geometry.TABLE_WIDTH / 2.0, 0.1)
