"""Agibot A3 — HOPE ping-pong WBC (BeyondMimic + HITTER racket-target tracking).

This is the step-13 environment. It extends the A3 motion-tracking baseline
(:class:`AgibotA3FlatEnvCfg`) with the HITTER racket objective:

* a :class:`RacketTargetCommand` that samples the desired racket state (position/velocity/normal)
  and desired base XY each swing, and computes the actual racket state by FK through ``T_mount``;
* HOPE actor observations (desired racket pos rel-base, desired racket vel/normal world,
  time-to-strike, desired base XY rel-base) plus projected gravity, with privileged actual racket
  state on the critic;
* HITTER goal rewards (base-position before strike; racket pos/vel/normal in a window around strike),
  on top of the BeyondMimic imitation reward and the regularization reward;
* extended domain randomization for sim-to-real.

Default usage trains one unified forehand+backhand policy by passing two reference clips
(``registry_name`` + ``registry_name_2``). The swing-type observation is present on the actor so
one policy can condition on which clip/target family it is currently imitating.
"""

from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.utils import configclass
from isaaclab.utils.noise import AdditiveUniformNoiseCfg as Unoise

import whole_body_tracking.tasks.tracking.mdp as mdp
from whole_body_tracking.robots.agibot_a3 import A3_FEET_BODIES, A3_HAND_BODIES, A3_UPPER_TRACKED
from whole_body_tracking.tasks.tracking.config.agibot_a3.flat_env_cfg import AgibotA3FlatEnvCfg
from whole_body_tracking.tasks.tracking.tracking_env_cfg import (
    ActionsCfg,
    CommandsCfg,
    EventCfg,
    ObservationsCfg,
    RewardsCfg,
    TerminationsCfg,
)

# FinalV3 motion supplies proximal swing style.  Wrist POSITION is retained as a reach scaffold:
# the V7 contact x/y/z now lies inside the venue command box, and the failed fresh run otherwise
# started 0.5--1.0 m outside the narrow racket-position Gaussian with no usable gradient. Wrist
# ORIENTATION and VELOCITY remain excluded because the planner owns the terminal face/twist and
# those V7 quantities intentionally differ from the real return command.
A3_UPPER_STYLE_TRACKED = [
    body_name for body_name in A3_UPPER_TRACKED if body_name != "right_wrist_yaw_Link"
]
A3_UPPER_STYLE_POSITION_TRACKED = list(A3_UPPER_TRACKED)


def attach_physical_ball_scene(env_cfg) -> None:
    """Attach the cat_stable physical-ball/table truth instrument for low-env audit.

    The ball collider is disabled: the fitted code-driven table and racket contacts remain the
    sole collision authority.  These assets are read only by telemetry and never by policy
    observations, rewards or target generation. Formal HitterPingPong training never calls this
    helper; only the bounded low-env evaluation and verification entrypoints call it in Build.
    """

    if getattr(env_cfg.scene, "pb_ball", None) is not None:
        return

    import yaml as _yaml

    import isaaclab.sim as sim_utils
    from isaaclab.assets import AssetBaseCfg, RigidObjectCfg

    from whole_body_tracking.tasks.table_tennis import geometry as tt_geom
    from whole_body_tracking.tasks.table_tennis import table_tennis_env_cfg as tt_cfg
    from whole_body_tracking.tasks.tracking.mdp.virtual_ball import default_venue_yaml_path

    with open(default_venue_yaml_path(), "r") as fh:
        ball_raw = _yaml.safe_load(fh)["ball"]
    ball_r = float(ball_raw["radius"])
    ball_m = float(ball_raw["mass"])

    racket_cfg = env_cfg.commands.racket_target
    near_x = float(racket_cfg.vb_table_near_x)
    surface_z = float(racket_cfg.vb_table_surface_z)
    materials = tt_geom.BounceMaterials()

    env_cfg.scene.pb_ball = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/PhysicalBall",
        init_state=RigidObjectCfg.InitialStateCfg(pos=(0.0, 0.0, -10.0)),
        spawn=sim_utils.SphereCfg(
            radius=ball_r,
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                disable_gravity=False,
                linear_damping=0.0,
                angular_damping=0.0,
                max_linear_velocity=1000.0,
                max_angular_velocity=1.0e5,
                max_depenetration_velocity=10.0,
                enable_gyroscopic_forces=False,
            ),
            mass_props=sim_utils.MassPropertiesCfg(mass=ball_m),
            collision_props=sim_utils.CollisionPropertiesCfg(collision_enabled=False),
            physics_material=tt_cfg._surface_material(
                0.0,
                materials.ball_static_friction,
                materials.ball_dynamic_friction,
            ),
            visual_material=sim_utils.PreviewSurfaceCfg(
                diffuse_color=(0.95, 0.95, 0.95), roughness=0.4
            ),
        ),
    )
    # No physical table collider is created. Table bounce is integrated by PhysicalBallManager;
    # adding a PhysX slab here would create a second contact authority and could push the robot.
    env_cfg.scene.pb_table_visual = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/PhysicalTableVisual",
        init_state=AssetBaseCfg.InitialStateCfg(
            pos=(
                near_x + tt_geom.TABLE_LENGTH / 2.0,
                0.0,
                surface_z - tt_geom.TABLE_HEIGHT,
            ),
            rot=(1.0, 0.0, 0.0, 0.0),
        ),
        spawn=sim_utils.UsdFileCfg(usd_path=tt_cfg._TABLE_USD_PATH),
    )

##
# Commands: motion (imitation) + racket target.
##


@configclass
class HOPECommandsCfg(CommandsCfg):
    racket_target = mdp.RacketTargetCommandCfg(
        asset_name="robot",
        motion_command_name="motion",
        debug_vis=False,
        # Paddle face normal = racket-local +Y (blade is thin along Y; +Y is the red/hitting face).
        # Confirmed from the std-pingpang URDF + blade STL in reimplement.md Step 11 (the cfg default
        # of axis 2/+Z was a placeholder guess). sign=+1 -> red (forehand) face; use -1 for the
        # black face if you train a backhand-only policy.
        # NOTE: cfg/task/HOPEPingPong.yaml also sets mount_normal_axis and (via train.py) overrides
        # this for the Hydra path — keep the two in sync.
        mount_normal_axis=1,
        mount_normal_sign=1.0,
    )


##
# Observations: HITTER actor (desired targets only) + privileged critic (actual racket state).
##


@configclass
class HOPEObservationsCfg(ObservationsCfg):
    @configclass
    class HOPEPolicyCfg(ObservationsCfg.PolicyCfg):
        # Deployment alignment with HITTER (arXiv:2508.21043, Table — actor obs): world-frame base LINEAR
        # velocity is a CRITIC-ONLY (privileged) observation there, because a humanoid's floating-base
        # linear velocity is not cleanly measurable on hardware (it needs a fragile IMU+leg-odometry state
        # estimator). The BeyondMimic base PolicyCfg feeds it to the actor; remove it here so the actor
        # never depends on a quantity it cannot reliably get at deploy. base_ang_vel / projected_gravity
        # (both from the IMU) and joint pos/vel stay. The critic (HOPECriticCfg) keeps base_lin_vel.
        base_lin_vel = None
        # Appended after the BeyondMimic proprioceptive + motion terms.
        projected_gravity = ObsTerm(func=mdp.projected_gravity, noise=Unoise(n_min=-0.05, n_max=0.05))
        base_target_pos_b = ObsTerm(func=mdp.base_target_pos_b, params={"command_name": "racket_target"})
        racket_target_pos_b = ObsTerm(
            func=mdp.racket_target_pos_b,
            params={"command_name": "racket_target"},
            noise=Unoise(n_min=-0.02, n_max=0.02),
        )
        racket_target_vel_w = ObsTerm(func=mdp.racket_target_vel_w, params={"command_name": "racket_target"})
        # HITTER (arXiv:2508.21043, Table I): the racket NORMAL/orientation is NOT an actor observation —
        # it is a reward target only. The actor sees only desired racket pos (rel base) + desired racket
        # vel (world) + time-to-strike + desired base pos (rel base). The critic keeps the normal (below).
        time_to_strike = ObsTerm(func=mdp.time_to_strike, params={"command_name": "racket_target"})
        # Unified HITTER policy (forehand+backhand in one policy): the actor must know which swing it is
        # doing (forehand +1 / backhand -1), since the swing type selects the imitated clip and the target
        # region. (For a single-swing-type policy this is constant and can be removed.)
        swing_type = ObsTerm(func=mdp.swing_type, params={"command_name": "racket_target"})

    @configclass
    class HOPECriticCfg(ObservationsCfg.PrivilegedCfg):
        base_target_pos_b = ObsTerm(func=mdp.base_target_pos_b, params={"command_name": "racket_target"})
        racket_target_pos_b = ObsTerm(func=mdp.racket_target_pos_b, params={"command_name": "racket_target"})
        # A1: the CRITIC keeps the TRUE live target velocity even when the actor's view is
        # delayed/jittered (task.racket.target_delay_steps / target_jitter_*): the asymmetric critic
        # is privileged/sim-side. Identical value to mdp.racket_target_vel_w when the A1 knobs are off.
        racket_target_vel_w = ObsTerm(func=mdp.racket_target_vel_w_live, params={"command_name": "racket_target"})
        racket_target_normal_w = ObsTerm(func=mdp.racket_target_normal_w, params={"command_name": "racket_target"})
        time_to_strike = ObsTerm(func=mdp.time_to_strike, params={"command_name": "racket_target"})
        # actual racket state (FK) — privileged, never available on hardware
        racket_pos_b = ObsTerm(func=mdp.racket_pos_b, params={"command_name": "racket_target"})
        racket_lin_vel_w = ObsTerm(func=mdp.racket_lin_vel_w, params={"command_name": "racket_target"})
        racket_normal_w = ObsTerm(func=mdp.racket_normal_w, params={"command_name": "racket_target"})
        episode_time_left = ObsTerm(func=mdp.episode_time_left)

    policy: HOPEPolicyCfg = HOPEPolicyCfg()
    critic: HOPECriticCfg = HOPECriticCfg()


##
# Rewards: imitation (inherited) + goal (racket/base) + regularization.
# Weights are HOPE tuning choices (HITTER does not publish reward weights/kernels).
##


@configclass
class HOPERewardsCfg(RewardsCfg):
    # r_goal — racket state tracking, active only in the ±strike_window around the strike.
    # std values are set to the step-14 acceptance tolerances so reward ≈ exp(-1) at the threshold;
    # tune from here (reimplement.md §13.7 item 7). HITTER does not publish reward weights/kernels.
    racket_position = RewTerm(
        func=mdp.racket_position_tracking_exp,
        weight=4.0,
        params={"command_name": "racket_target", "std": 0.075},  # target < 7.5 cm
    )
    racket_velocity = RewTerm(
        func=mdp.racket_velocity_tracking_exp,
        weight=2.0,
        params={"command_name": "racket_target", "std": 0.5},  # target < 0.5 m/s
    )
    racket_normal = RewTerm(
        func=mdp.racket_normal_tracking_exp,
        weight=2.0,
        params={"command_name": "racket_target", "std": 0.262},  # radians, target < 15 deg
    )
    # r_goal — base repositioning, active only before the strike.
    base_position = RewTerm(
        func=mdp.base_position_tracking_exp,
        weight=1.0,
        params={"command_name": "racket_target", "std": 0.3},
    )
    # r_regularization — pre-strike foot-slip penalty (stability). Penalizes horizontal foot speed while
    # the foot is in contact, gated by pre_strike ONLY (the strike swing is untouched). Default weight is
    # overridden by cfg/task/HOPEPingPong.yaml `pre_strike_foot_slip_weight`.
    pre_strike_foot_slip = RewTerm(
        func=mdp.pre_strike_foot_slip,
        weight=-0.2,
        params={"command_name": "racket_target"},
    )
    # r_regularization — energy / torque smoothness (action_rate_l2 already inherited).
    joint_torques = RewTerm(func=mdp.joint_torques_l2, weight=-1.0e-5)


##
# Domain randomization (HITTER + standard sim-to-real reconstruction).
#
# HITTER publishes no DR table; it states PD gains are FIXED. The mass/friction/push/observation-noise
# terms below are standard BeyondMimic practice; PD-gain and motor-strength randomization are added for
# sim-to-real robustness and can be disabled to match HITTER exactly.
#
# Already provided by the base EventCfg: friction (physics_material, startup), CoM (startup),
# joint default pos (startup), external pushes (push_robot, interval). Observation noise comes from the
# per-term Unoise + enable_corruption on the policy observation group.
##


@configclass
class HOPEEventCfg(EventCfg):
    # HITTER alignment: no external push. HITTER's prose DR is mass/friction/restitution + perception
    # noise/delays only — there is no random shove. Keep friction (physics_material) and CoM (base_com)
    # from the base EventCfg; disable the base interval push.
    push_robot = None

    # link mass randomization (±10%) — HITTER prose randomizes link mass.
    randomize_link_mass = EventTerm(
        func=mdp.randomize_rigid_body_mass,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=".*"),
            "mass_distribution_params": (0.85, 1.15),
            "operation": "scale",
            "distribution": "uniform",
            "recompute_inertia": True,
        },
    )
    # PD gain / motor strength randomization (±20%). NOTE: HITTER keeps PD fixed; this is a
    # sim-to-real robustness choice. Set to None to disable.
    randomize_pd_gains = EventTerm(
        func=mdp.randomize_actuator_gains,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("robot", joint_names=[".*"]),
            "stiffness_distribution_params": (0.8, 1.2),
            "damping_distribution_params": (0.8, 1.2),
            "operation": "scale",
            "distribution": "log_uniform",
        },
    )


##
# Environment configuration.
##


##
# deploy-parity variant — deploy-honest observation (no fabricated base pose).
#
# WHY: the `full` actor obs above depends on the robot's true world base pose through three terms
# (motion_anchor_pos_b, base_target_pos_b, racket_target_pos_b). The mocap streams the base pose at
# 300 Hz during play, but that link is not bridged into the deploy front-end, so those terms are
# fabricated at deploy (anchor_pos_b := 0, base_pos := nominal) -> the deployed policy
# sees a DIFFERENT observation distribution than training and the legs cannot balance. Making the
# actor base-position-free is a deliberate robustness choice (no mocap/VRPN dependency). AGI's reference
# policy transfers because its observation is real-sensor-only (IMU orientation + proprioception, no
# world base position). This variant copies that recipe for the HOPE actor. The privileged CRITIC
# group is unchanged (it may use base pose in sim — it is never deployed). The `full` cfgs above are
# untouched (kept for comparison / the old path).
##


@configclass
class HOPEObservationsDeployParityCfg(HOPEObservationsCfg):
    """Actor obs with every world-frame BASE-POSITION dependency removed (180 -> 175):

    * REMOVED  ``motion_anchor_pos_b`` (3)  — reference torso *position* error needs the world base pose.
    * REMOVED  ``base_target_pos_b``   (2)  — base-repositioning target needs the world base pose.
    * REFRAMED ``racket_target_pos_b`` (3)  — now ``target - current_racket`` (FK), base pose cancels.
    * KEPT     ``motion_anchor_ori_b`` (6, orientation-only / IMU), command, base_ang_vel, joint pos/vel,
               last action, projected_gravity, racket_target_vel_w, time_to_strike, swing_type.

    Every kept/reframed term is computable on hardware from IMU + joint encoders + the planner target.
    """

    @configclass
    class HOPEPolicyDeployParityCfg(HOPEObservationsCfg.HOPEPolicyCfg):
        # --- remove base-position-dependent terms (fabricated on hardware) ---
        motion_anchor_pos_b = None  # inherited from ObservationsCfg.PolicyCfg; needs world base position
        base_target_pos_b = None  # base-repositioning target; needs world base position
        # --- reframe racket target to be relative to the current racket (FK); no world base position ---
        racket_target_pos_b = ObsTerm(
            func=mdp.racket_target_pos_rel_b,
            params={"command_name": "racket_target"},
            noise=Unoise(n_min=-0.02, n_max=0.02),
        )

    @configclass
    class HOPECriticDeployParityCfg(HOPEObservationsCfg.HOPECriticCfg):
        # Vestigial in the base-free deploy-parity task: the base target is never consumed by any reward
        # or actor obs and (base_couple_blend=0) is pure spawn+jitter noise — conditioning the value
        # function on it only adds variance. Removing it changes the CRITIC input dim (2026-07-03), so
        # every pre-change checkpoint fails a FULL strict load — train.py resume stays a loud error on
        # purpose; play.py (export) and eval_deterministic.py fall back to an actor-only tolerant load
        # (utils/ckpt_compat.py). The exported ACTOR / 175-D contract is untouched.
        base_target_pos_b = None

    policy: HOPEPolicyDeployParityCfg = HOPEPolicyDeployParityCfg()
    critic: HOPECriticDeployParityCfg = HOPECriticDeployParityCfg()


@configclass
class HOPEDeployParityRewardsCfg(HOPERewardsCfg):
    """FOOTWORK-TO-STRIKE reward — BASE-FREE. No base-position / base-target / base-arrival reward: the
    legs move because reducing the racket->target distance (``racket_progress``) takes whole-body motion.
    The feet are FREE to step/shift — only BAD foot behaviour is penalized (slip / drag / violent / unstable
    at the strike), never "both feet planted". Lower-body imitation is DROPPED (legs free to reach varied
    targets); upper-body + racket imitation is kept for swing style. All weights are STARTING POINTS — the
    footwork weights live here (not the task YAML), so tune them in this class. (Obs is the base-free
    deploy-parity layout from HOPEObservationsDeployParityCfg.)"""

    # --- BASE-FREE corrections: remove every base-position-dependent reward ---
    base_position = None  # inherited HITTER base-repositioning reward -> REMOVED (it needs a base target)
    motion_global_anchor_pos = None  # reference base-POSITION tracking -> REMOVED (it pins the base)

    # --- racket task: keep the additive pos/vel/normal (inherited, wide gradient) + a MULTIPLICATIVE
    #     success bonus that fires only when pos AND vel AND normal are all good at once (tight acceptance). ---
    racket_strike_success = RewTerm(
        func=mdp.racket_strike_success, weight=5.0,
        params={"command_name": "racket_target", "std_pos": 0.075, "std_vel": 0.5, "std_normal": 0.262},
    )
    # --- the BASE-FREE MOVEMENT DRIVER: dense pre-strike reward for closing the racket->target distance.
    #     Telescopes to weight * (distance reduced over the approach) -> the whole body moves to the target. ---
    racket_progress = RewTerm(func=mdp.racket_progress, weight=10.0, params={"command_name": "racket_target"})

    # --- upper-body-only imitation (legs DECOUPLED so footwork is free to adapt to the target) ---
    # swing-only since 2026-07-05: during hold the body refs (frozen crouch frame) fought
    # the stand joint reference -> splayed-feet crouch-stand; see hope_rewards wrappers.
    # Foot discipline (2026-07-05): hip yaw/roll + ankle roll held to the reference
    # footwork (hold-aware). Penalty; tune in [-0.5,-0.1] if it taxes the lunge.
    foot_orientation = RewTerm(func=mdp.foot_orientation_discipline, weight=-0.3,
        params={"command_name": "motion",
                "asset_cfg": SceneEntityCfg("robot", joint_names=[".*_hip_yaw_joint", ".*_hip_roll_joint", ".*_ankle_roll_joint"])})
    motion_body_pos = RewTerm(func=mdp.motion_body_pos_swing_only, weight=1.0,
        params={"command_name": "motion", "std": 0.3, "body_names": A3_UPPER_TRACKED})
    motion_body_ori = RewTerm(func=mdp.motion_body_ori_swing_only, weight=1.0,
        params={"command_name": "motion", "std": 0.4, "body_names": A3_UPPER_TRACKED})
    motion_body_lin_vel = RewTerm(func=mdp.motion_global_body_linear_velocity_error_exp, weight=1.0,
        params={"command_name": "motion", "std": 1.0, "body_names": A3_UPPER_TRACKED})
    motion_body_ang_vel = RewTerm(func=mdp.motion_global_body_angular_velocity_error_exp, weight=1.0,
        params={"command_name": "motion", "std": 3.14, "body_names": A3_UPPER_TRACKED})

    # --- footwork PENALTIES (the feet may step; punish only bad behaviour, NEVER reward "always planted") ---
    foot_slip_sq = RewTerm(func=mdp.foot_slip_sq, weight=-1.0, params={"command_name": "racket_target"})
    foot_velocity = RewTerm(func=mdp.foot_velocity, weight=-0.05, params={"command_name": "racket_target"})
    foot_drag = RewTerm(func=mdp.foot_drag, weight=-0.5, params={"command_name": "racket_target"})
    arm_overreach = RewTerm(func=mdp.arm_overreach, weight=-0.5, params={"command_name": "racket_target"})
    # Anti twist-instead-of-step (pre-strike): penalize |waist_yaw|+|waist_roll| deviation from neutral so
    # the policy cannot face a lateral target by twisting the torso with planted feet — it must STEP.
    # Weight is CLI-tunable via task.rewards.prestrike_waist_twist_weight. Raise if the torso still twists
    # (waist_twist_prestrike stays high / legs stay frozen); lower if it flattens the swing.
    prestrike_waist_twist = RewTerm(
        func=mdp.prestrike_waist_twist, weight=-1.0, params={"command_name": "racket_target"})

    # --- between-swing recovery: POSITIVE ready-stance reward during the pre-swing HOLD --------------
    # (2026-07-03 audit alignment) HITTER's recovery signal is positive-and-causal ("prepare for the next
    # target"), not a pile of penalties. During the hold the imitation reward already pulls the UPPER body
    # to the windup pose, but the legs/base had zero positive signal. hold_ready = exp(-(|v|^2+|w|^2)/std^2)
    # * feet_contact_frac, gated to motion.in_hold AND to target-within-reach (racket_target_distance <
    # reach): near targets -> stand ready pays; far targets -> the term is SILENT so it never out-earns
    # racket_progress for stepping (without the reach gate, planted stillness beats stepping ~1.5/step and
    # teaches freeze-then-rush). The swing itself is untouched (zero outside the hold). CLI-tunable via
    # task.rewards.hold_ready_weight / hold_ready_std / hold_ready_reach.
    hold_ready = RewTerm(
        func=mdp.hold_ready, weight=2.0,
        params={"command_name": "racket_target", "std": 0.5, "reach": 0.65})

    # --- P2.4 PACE-style smooth deceleration (G08, flag-gated, DEFAULT OFF) --------------------------
    # Pseudo base-velocity command proportional to the remaining PLANAR racket->target error:
    # v_des = clamp(v_gain*dist_xy, 0, v_max); reward = exp(-(|v_base_xy| - v_des)^2/std^2), gated to
    # pre_strike. Far target -> pays for moving at v_max (cooperates with racket_progress); at arrival
    # v_des -> 0 -> pays for a CALM base, killing the reactive rush-then-slam toward far targets.
    # REWARD-side only — the frozen 175-D actor obs contract is untouched. weight 0.0 = OFF (IsaacLab's
    # RewardManager skips zero-weight terms); enable per-experiment via task.rewards.base_decel_weight
    # (suggested trial 1.0). CLI/yaml-tunable: base_decel_weight / _v_gain / _v_max / _std.
    # Watch metric: base_speed_xy_prestrike (should taper near targets instead of staying hot).
    base_decel = RewTerm(
        func=mdp.base_decel_tracking, weight=0.0,
        params={"command_name": "racket_target", "v_gain": 2.0, "v_max": 1.6, "std": 0.4})

    # --- strike-window stability: be planted + upright + still AT the hit (gated to the strike window) ---
    strike_upright = RewTerm(func=mdp.strike_proj_grav_xy, weight=-2.0, params={"command_name": "racket_target"})
    strike_ang_vel = RewTerm(func=mdp.strike_base_ang_vel, weight=-0.5, params={"command_name": "racket_target"})
    strike_foot_vel = RewTerm(func=mdp.strike_foot_velocity, weight=-0.5, params={"command_name": "racket_target"})
    strike_vbob = RewTerm(func=mdp.strike_vertical_bob, weight=-1.0, params={"command_name": "racket_target"})

    # --- SIM2REAL FINE-TUNE (2026-07-02): survive AGI's EXPLICIT clipped-PD MuJoCo. ------------------
    # CHANGE 2 — torque-saturation penalty: penalize the mean over-limit fraction of the COMPUTED (pre-clip)
    # effort over the arm + waist joints so the policy stops demanding torque the explicit motor cannot
    # deliver (the elbow was at ~6.7x its 24 Nm limit in the failing trace). Modest weight to protect the
    # strike. CLI-tunable via task.rewards.arm_torque_saturation_weight. Watch metric: arm_torque_sat_frac.
    arm_torque_saturation = RewTerm(
        func=mdp.arm_torque_saturation, weight=-0.5, params={"command_name": "racket_target"})
    # CHANGE 3 — balance shaping (POSITION-based): penalize forward base/torso TILT (proj_grav_xy) DURING
    # the approach (pre_strike), so the CoM stays over the support base THROUGH the swing (strike_upright
    # covers the strike window). NOT an angular-velocity penalty (those are gameable / anti-swing).
    # CLI-tunable via task.rewards.prestrike_upright_weight.
    prestrike_upright = RewTerm(
        func=mdp.prestrike_proj_grav_xy, weight=-1.0, params={"command_name": "racket_target"})

    # --- always-on balance + safety regularizers (kept) ---
    upright = RewTerm(func=mdp.flat_orientation_l2, weight=-1.0)  # base tilt
    base_ang_vel_xy = RewTerm(func=mdp.ang_vel_xy_l2, weight=-0.05)  # roll/pitch rate
    base_lin_vel_z = RewTerm(func=mdp.lin_vel_z_l2, weight=-0.5)  # vertical bob
    joint_vel = RewTerm(func=mdp.joint_vel_l2, weight=-1.0e-4)  # joint-velocity smoothness
    # (inherited & kept: racket_position/velocity/normal, pre_strike_foot_slip, action_rate_l2,
    #  joint_torques, joint_limit, undesired_contacts, motion_global_anchor_ori.)


@configclass
class HOPEDeployParityTerminationsCfg(TerminationsCfg):
    """Inherited reference-relative terminations + ABSOLUTE balance terminations, so a real fall/sink
    ends the episode regardless of the reference clip (the actual deploy failure mode)."""

    base_fell_tilt = DoneTerm(func=mdp.bad_orientation, params={"limit_angle": 0.7})  # ~40 deg, absolute
    base_too_low = DoneTerm(func=mdp.root_height_below_minimum, params={"minimum_height": 0.5})


@configclass
class HOPEPingPongAgibotA3EnvCfg(AgibotA3FlatEnvCfg):
    obs_mode: str = "full"  # descriptive; the deploy-parity variant is HOPEPingPongDeployParityAgibotA3EnvCfg
    commands: HOPECommandsCfg = HOPECommandsCfg()
    observations: HOPEObservationsCfg = HOPEObservationsCfg()
    rewards: HOPERewardsCfg = HOPERewardsCfg()
    events: HOPEEventCfg = HOPEEventCfg()

    def __post_init__(self):
        # AgibotA3FlatEnvCfg sets the robot, action scale, motion anchor/body names, and the A3
        # contact/termination/CoM body names (all valid for the inherited HOPE* cfg subclasses).
        super().__post_init__()
        # Multi-swing ping-pong must learn physical recovery between clips. Reset-time RSI remains active,
        # but clip wrap never teleports the robot back to the next reference start state
        # (MotionCommandCfg.wrap_teleport already defaults to False; kept explicit here).
        self.commands.motion.wrap_teleport = False


@configclass
class HOPEPingPongDeployParityAgibotA3EnvCfg(HOPEPingPongAgibotA3EnvCfg):
    """Deploy-parity variant: deploy-honest actor observation (no fabricated base pose) plus
    absolute balance rewards/terminations. The ``full`` HOPEPingPongAgibotA3EnvCfg is left intact."""

    obs_mode: str = "deploy_parity"
    observations: HOPEObservationsDeployParityCfg = HOPEObservationsDeployParityCfg()
    rewards: HOPEDeployParityRewardsCfg = HOPEDeployParityRewardsCfg()
    terminations: HOPEDeployParityTerminationsCfg = HOPEDeployParityTerminationsCfg()


@configclass
class HOPEPingPongRealSensorAgibotA3EnvCfg(HOPEPingPongDeployParityAgibotA3EnvCfg):
    """Backward-compatible alias for the deploy-parity variant.

    Older docs and scripts still refer to this env as ``real_sensor_only`` / ``RealSensor``.
    The actor contract is the same deploy-parity 175-D layout.
    """


##
# Tier-1 virtual-ball variant (rewardDesign.md) — REWARD-ONLY on top of deploy-parity.
#
# The observation is the UNCHANGED deploy-parity 175-D actor contract (sim-to-real alignment is
# frozen; the virtual ball is never observed — it exists only inside the reward). Per swing the
# command term samples a virtual incoming ball that arrives at the racket target at strike time;
# at the exact-strike frame the achieved racket FK state is pushed through the venue-fitted paddle
# contact model + a coarse landing rollout, and the one-shot virtual_* terms below score the
# predicted shot (net clearance / landing accuracy / outgoing topspin).
##


@configclass
class HOPEVirtualBallRewardsCfg(HOPEDeployParityRewardsCfg):
    """DeployParity reward stack + Tier-1 virtual-ball outcome terms.

    Weights follow rewardDesign.md: landing 30 / pass_net 20 / spin 5 (start of the 5->10 ramp),
    ordered clear-net-first below landing per the PACE/v0 precedent. racket_velocity/racket_normal
    drop 2.0 -> 0.5: the contact model now scores the whole (velocity, normal, timing) manifold
    directly, so vector-matching the commanded velocity becomes shaping, not the task. The approach
    gradient (racket_position 4.0, racket_progress 10.0, racket_strike_success 5.0) is kept — the
    virtual terms are zero until the paddle reaches the 9.5 cm capture gate at the strike frame.
    """

    virtual_pass_net = RewTerm(
        func=mdp.virtual_pass_net, weight=20.0, params={"command_name": "racket_target"})
    virtual_landing = RewTerm(
        func=mdp.virtual_landing, weight=30.0, params={"command_name": "racket_target"})
    virtual_spin = RewTerm(
        func=mdp.virtual_spin, weight=5.0, params={"command_name": "racket_target"})

    racket_velocity = RewTerm(
        func=mdp.racket_velocity_tracking_exp,
        weight=0.5,
        params={"command_name": "racket_target", "std": 0.5},
    )
    racket_normal = RewTerm(
        func=mdp.racket_normal_tracking_exp,
        weight=0.5,
        params={"command_name": "racket_target", "std": 0.262},
    )


@configclass
class HOPEPingPongVirtualBallAgibotA3EnvCfg(HOPEPingPongDeployParityAgibotA3EnvCfg):
    """Deploy-parity env + Tier-1 virtual-ball rewards. Obs/terminations/DR inherited untouched."""

    obs_mode: str = "deploy_parity"
    rewards: HOPEVirtualBallRewardsCfg = HOPEVirtualBallRewardsCfg()

    def __post_init__(self):
        super().__post_init__()
        # Reward-only feature switch: enables the per-swing virtual-ball sampler and the at-strike
        # contact + coarse-landing evaluation in RacketTargetCommand (vb_* cfg fields hold the
        # venue-fit sampling boxes / gates; tune there, not here).
        self.commands.racket_target.virtual_ball = True
        # CLIMB-PHASE shaping width (2026-07-03): the E-champion warm start crosses the net plane
        # ~0.3-0.5 m BELOW the target height; at the v0 default sigma 0.10 the height kernel is
        # exp(-(0.5/0.1)^2) ~ 0 there — no gradient, and vb_warmE14k3 paid zero virtual reward for
        # 2.5k iters. 0.25 keeps a usable gradient down to the current operating band. Tighten
        # back toward 0.10 once virtual_net_clear_rate is healthy (>0.3 or so).
        self.commands.racket_target.vb_net_sigma = 0.25
        # CLIMB-PHASE landing kernel width (2026-07-04): landings start ~1.9 m short of the target
        # (exp(-(1.9/0.3)^2) = 0 — the v0 sigma has no reach); 1.0 pays 0.03 at the current band
        # and grows monotonically toward the target = dense "hit deeper" gradient (the kernel is
        # also ungated from net clearance during the climb — see hope_rewards.virtual_landing).
        # Tighten back toward 0.3 together with re-gating once the net terms carry the signal.
        self.commands.racket_target.vb_landing_sigma = 1.0


##
# HITTER-footwork variant (arXiv:2508.21043 §V-B-1 "Separate Commands for Base and Racket") —
# deploy-parity base + the base-position command channel restored (2026-07-05).
#
# WHY: the BASE-FREE deploy-parity policy self-selects walk-and-strike footwork toward deep
# world-frame racket targets ("chasing a point forward"); it cannot be commanded to a station.
# HITTER instead (a) commands the base to a world XY station, (b) fixes the striking plane
# RELATIVE to the robot (0.4 m in front on their G1; our analog = each clip's reference
# base→racket strike offset), sampling only the racket target's y/z spread, and (c) activates
# base tracking only PRE-STRIKE (mdp.base_position_tracking_exp is already gated that way).
#
# SIM2REAL CONTRACT (177-D actor = 175-D deploy-parity + base_target_pos_b(2) restored at its
# original slot between projected_gravity and racket_target_pos_b):
#   * base_target_pos_b is a RELATIVE Δxy in the yaw-heading frame — computable on hardware from
#     the mocap base position (300 Hz, position-only; hope-mocap-spec) + IMU yaw-align-at-engage.
#     No absolute world coordinates enter the obs; mocap dropout → feed Δ=0, which degrades
#     gracefully to "already at station" (today's BASE-FREE behavior).
#   * A1 target latency/jitter does NOT yet degrade the base channel (the racket channel does);
#     the base station demand is O(10 cm), obs Unoise covers mocap noise. Revisit if hardware
#     shows base-channel transport lag matters.
#   * The C++ runner (pp_policy.hpp build_obs_175) and mujoco_eval_onnx are 175-D and need the
#     177-D layout + a planner base-target input before this variant can deploy — verify with
#     scripts/verify_realsensor.py layout print after any obs change.
##


@configclass
class HOPEObservationsHitterCfg(HOPEObservationsDeployParityCfg):
    """Deploy-parity actor obs + the HITTER base-position command channel (175 -> 177)."""

    @configclass
    class HOPEPolicyHitterCfg(HOPEObservationsDeployParityCfg.HOPEPolicyDeployParityCfg):
        # Restore the base-repositioning target (Δxy, yaw-heading frame). Overriding the parent's
        # `= None` puts the term back at its ORIGINAL declaration slot (configclass inheritance
        # preserves attribute order): between projected_gravity and racket_target_pos_b.
        base_target_pos_b = ObsTerm(
            func=mdp.base_target_pos_b,
            params={"command_name": "racket_target"},
            noise=Unoise(n_min=-0.03, n_max=0.03),  # ~mocap base-position noise at 300 Hz
        )

    @configclass
    class HOPECriticHitterCfg(HOPEObservationsDeployParityCfg.HOPECriticDeployParityCfg):
        # The critic conditions on the station too now that a reward consumes it.
        base_target_pos_b = ObsTerm(func=mdp.base_target_pos_b, params={"command_name": "racket_target"})

    policy: HOPEPolicyHitterCfg = HOPEPolicyHitterCfg()
    critic: HOPECriticHitterCfg = HOPECriticHitterCfg()


@configclass
class HOPEHitterRewardsCfg(HOPEDeployParityRewardsCfg):
    """Deploy-parity rewards + the HITTER base-repositioning goal reward restored.

    mdp.base_position_tracking_exp is PRE-STRIKE gated in the reward function itself (HITTER:
    "the base position tracking reward is activated only before the strike"). std 0.3 m matches
    the original HITTER-alignment tuning (HOPERewardsCfg.base_position).
    """

    base_position = RewTerm(
        func=mdp.base_position_tracking_exp,
        weight=1.0,
        params={"command_name": "racket_target", "std": 0.3},
    )


@configclass
class HOPEPingPongHitterAgibotA3EnvCfg(HOPEPingPongDeployParityAgibotA3EnvCfg):
    """Deploy-parity env + HITTER separate base/racket commands (obs 177-D; NOT deploy-compatible
    with the 175-D C++ runner until the runner/planner grow the base channel)."""

    obs_mode: str = "hitter_footwork"
    observations: HOPEObservationsHitterCfg = HOPEObservationsHitterCfg()
    rewards: HOPEHitterRewardsCfg = HOPEHitterRewardsCfg()

    def __post_init__(self):
        super().__post_init__()
        # HITTER coupling: base station derived from the racket target at the clip's reference
        # reach — the striking plane is fixed relative to the COMMANDED base; the box x-span moves
        # the station, not the reach depth. Jitter ranges (base_target_*_range) train deliberate
        # station offsets (y-reach diversity); the yaml preset owns their spans.
        self.commands.racket_target.base_couple_mode = "reference_reach"


##
# HITTER-PURE variant (2026-07-07) — faithful reproduction of the paper's MDP, replacing the
# accumulated HOPE machinery. Decision context: model_17400 (177-D hitter_footwork) deploys and
# stands on hardware but swings on ~1/10 served balls and misses — the trained distribution is
# clip-centered and narrow, the actor carries the 62-D reference stream (paper: CRITIC-only,
# Table I), and the face-normal target was locked to the reference clip (paper §IV-C: the racket
# plane is PERPENDICULAR TO ITS VELOCITY at impact). This variant re-aligns all three.
#
# vs the paper (arXiv:2508.21043), EXACT alignment:
#   * Actor obs = Table I structure sized for the A3 (110-D): ang vel, gravity, e_base,x,
#     Δbase target (world xy), racket target rel base (world), racket target vel (world),
#     time-to-strike, q/q̇/a_last. NO reference joints, NO swing_type, NO anchor terms.
#   * Separate commands (§V-B-1): base station sampled INDEPENDENTLY (paper Fig. 4: up to
#     ±0.75-0.8 m, 1 cm arrival in <0.8 s); racket target on a plane FIXED relative to the
#     commanded station (their 0.4 m on the G1; our A3 analog = the clips' blade reach 0.70 m),
#     only y/z sampled, per-swing-type non-overlapping regions.
#   * Normal target = velocity direction (§IV-C impact model) — the policy must LEARN the wrist
#     orientation (initial error 18-110°; expected to learn slowly, do NOT "fix" it by moving
#     the target back to the reference normal — that is how legal returns became 0%).
#   * Reward = dense upper-body imitation + sparse goal (racket pos/vel/normal in the strike
#     window; base position pre-strike only) + generic regularization. NO hold_ready, NO foot/
#     stability shaping, NO HER replay, NO base_decel (paper has none of them).
#   * 10 s episodes, multiple swings, swing type + targets resampled per swing, no hold phase.
#
# Deliberate departures (kept, with reasons):
#   * stand_start_prob 0.25 + no-teleport wraps (deploy-honest entry/transition; paper does not
#     document its reset scheme).
#   * DR keeps PD ±15% / link mass ±15% (sim2real; paper fixes PD).
#   * Tuned kernel widths from the 0625-0706 lineage (paper publishes no weights/stds).
#
# Deploy contract: 110-D `hitter_pure` — needs a NEW C++ obs builder + a planner that streams
# (station, racket target, vel, tts) CONTINUOUSLY (no engage-lock). See actor_observation_contract.
##


@configclass
class HOPEObservationsHitterPureCfg(HOPEObservationsCfg):
    """HITTER Table-I actor (110-D, world-frame targets + e_base,x); critic unchanged (privileged)."""

    @configclass
    class HOPEPolicyHitterPureCfg(ObservationsCfg.PolicyCfg):
        # --- remove every non-Table-I term from the BeyondMimic base actor ---
        command = None  # 62-D reference joint stream: CRITIC-ONLY in HITTER (Table I)
        motion_anchor_pos_b = None  # needs world base position; not in Table I
        motion_anchor_ori_b = None  # reference-coupled orientation error; Table I uses e_base,x instead
        base_lin_vel = None  # critic-only in HITTER (not measurable on hardware)
        # --- Table I goal terms (appended after the inherited proprioception) ---
        projected_gravity = ObsTerm(func=mdp.projected_gravity, noise=Unoise(n_min=-0.05, n_max=0.05))
        base_forward_xy = ObsTerm(
            func=mdp.base_forward_xy,
            params={"command_name": "racket_target"},
            noise=Unoise(n_min=-0.02, n_max=0.02),
        )
        base_target_delta_xy = ObsTerm(
            func=mdp.base_target_delta_xy,
            params={"command_name": "racket_target"},
            noise=Unoise(n_min=-0.03, n_max=0.03),  # ~mocap base-position noise
        )
        racket_target_rel_base = ObsTerm(
            func=mdp.racket_target_rel_base,
            params={"command_name": "racket_target"},
            noise=Unoise(n_min=-0.02, n_max=0.02),
        )
        racket_target_vel_w = ObsTerm(func=mdp.racket_target_vel_w, params={"command_name": "racket_target"})
        time_to_strike = ObsTerm(func=mdp.time_to_strike, params={"command_name": "racket_target"})

    @configclass
    class HOPECriticHitterPureCfg(HOPEObservationsCfg.HOPECriticCfg):
        # Table I checkmarks EVERY actor term in the critic column too — make the critic a strict
        # actor superset (audit 2026-07-07): the inherited critic lacked projected_gravity and the
        # world-frame goal view (it only had the yaw-heading-frame legacy accessors). Live,
        # noise-free variants; the privileged heading-frame/FK extras above are kept.
        projected_gravity = ObsTerm(func=mdp.projected_gravity)
        base_forward_xy = ObsTerm(func=mdp.base_forward_xy, params={"command_name": "racket_target"})
        base_target_delta_xy = ObsTerm(
            func=mdp.base_target_delta_xy, params={"command_name": "racket_target"}
        )
        racket_target_rel_base = ObsTerm(
            func=mdp.racket_target_rel_base, params={"command_name": "racket_target"}
        )

    policy: HOPEPolicyHitterPureCfg = HOPEPolicyHitterPureCfg()
    critic: HOPECriticHitterPureCfg = HOPECriticHitterPureCfg()
    # critic = HOPECriticCfg (reference joints, body poses T_B, base lin vel, time-left, live
    # targets + actual racket FK state) + the actor's world-frame goal terms above — a strict
    # superset of both the paper's critic and the actor. Privileged sim-side only, never deployed.


@configclass
class HOPEHitterPureRewardsCfg(RewardsCfg):
    """HITTER §V-B-2 faithful reward stack: r = w_i·r_imitation + w_g·r_goal + w_r·r_regularization.

    * r_imitation — dense, UPPER-BODY reference only (paper §V-A: B = bodies above the pelvis);
      the base is steered by the GOAL terms, not imitation (motion_global_anchor_pos removed).
    * r_goal — sparse, relatively high weights (paper): racket pos/vel/NORMAL tracking in the
      strike window; base position tracking PRE-STRIKE only (gated inside the fn).
    * r_regularization — generic energy/smoothness/safety only. NO hold_ready / foot shaping /
      waist twist / strike-window stability / torque saturation — the paper has none of them.

    Weights/stds are HOPE tuning (the paper publishes neither); the task YAML owns the numbers.
    """

    # --- imitation: upper-body only, swing-gated (hold refs are ready-stand; legs decoupled) ---
    motion_global_anchor_pos = None  # base position is a GOAL (base_position), not imitation
    motion_body_pos = RewTerm(func=mdp.motion_body_pos_swing_only, weight=1.0,
        params={"command_name": "motion", "std": 0.3, "body_names": A3_UPPER_TRACKED})
    motion_body_ori = RewTerm(func=mdp.motion_body_ori_swing_only, weight=1.0,
        params={"command_name": "motion", "std": 0.4, "body_names": A3_UPPER_TRACKED})
    motion_body_lin_vel = RewTerm(func=mdp.motion_global_body_linear_velocity_error_exp, weight=1.0,
        params={"command_name": "motion", "std": 1.0, "body_names": A3_UPPER_TRACKED})
    motion_body_ang_vel = RewTerm(func=mdp.motion_global_body_angular_velocity_error_exp, weight=1.0,
        params={"command_name": "motion", "std": 3.14, "body_names": A3_UPPER_TRACKED})

    # --- goal (sparse; strike-window / pre-strike gating lives inside the reward fns) ---
    racket_position = RewTerm(func=mdp.racket_position_tracking_exp, weight=14.0,
        params={"command_name": "racket_target", "std": 0.15})
    racket_velocity = RewTerm(func=mdp.racket_velocity_tracking_exp, weight=14.0,
        params={"command_name": "racket_target", "std": 0.6})
    racket_normal = RewTerm(func=mdp.racket_normal_tracking_exp, weight=5.0,
        params={"command_name": "racket_target", "std": 0.30})
    base_position = RewTerm(func=mdp.base_position_tracking_exp, weight=2.0,
        params={"command_name": "racket_target", "std": 0.20})
    racket_strike_success = RewTerm(func=mdp.racket_strike_success, weight=5.0,
        params={"command_name": "racket_target", "std_pos": 0.075, "std_vel": 0.5, "std_normal": 0.262})
    # OFF by default (not in the paper). Declared as a fallback shaping knob: if from-scratch
    # exploration cannot find the strike window over the wide station box, re-enable via
    # task.rewards (racket_progress telescopes to distance-reduced; weight 0.0 = skipped).
    racket_progress = RewTerm(func=mdp.racket_progress, weight=0.0, params={"command_name": "racket_target"})
    # OFF by default (weight 0.0 = SKIPPED → plain HitterPure stays paper-faithful/byte-identical).
    # PACE-style base→STATION deceleration (2026-07-09): pays for MOVING when far from the commanded
    # y-station and for a CALM base once arrived → the robot reaches the station EARLY and SETTLES there
    # BEFORE the strike (the pre-strike approach gap the near-strike lower_body_plant_imitation does not
    # cover; base_position's std-0.20 kernel is gradient-dead at fresh ±0.40 stations). Keyed to
    # base→station (NOT racket→target — see hope_rewards.base_station_settle). The V2 variant enables it.
    base_station_settle = RewTerm(func=mdp.base_station_settle, weight=0.0,
        params={"command_name": "racket_target", "v_gain": 2.0, "v_max": 1.2, "std": 0.4})

    # --- regularization (generic only) ---
    joint_torques = RewTerm(func=mdp.joint_torques_l2, weight=-1.0e-5)
    joint_vel = RewTerm(func=mdp.joint_vel_l2, weight=-1.0e-4)
    upright = RewTerm(func=mdp.flat_orientation_l2, weight=-1.0)
    base_ang_vel_xy = RewTerm(func=mdp.ang_vel_xy_l2, weight=-0.05)
    base_lin_vel_z = RewTerm(func=mdp.lin_vel_z_l2, weight=-0.5)
    # (inherited & kept: motion_global_anchor_ori 0.5, action_rate_l2 -0.1, joint_limit -10,
    #  undesired_contacts -0.1.)

    # --- continuous-rally recovery terms (2026-07-07) — weight 0.0 = SKIPPED (RewardManager drops
    # zero-weight terms), so plain HitterPure stays byte-identical / paper-faithful. The Rally
    # variant (HOPEPingPongHitterPureRallyAgibotA3EnvCfg + its YAML) enables them:
    #   post_strike_brake — positive braking kernel through the follow-through ((~pre_strike) &
    #     (~strike_window)); arrests the walk-and-strike lunge momentum (deploy P7 drift fall).
    #   hold_ready — the 177-proven settle term (stillness x planted feet), in_hold-gated with the
    #     STATION reach gate (std 1.5 / reach 0.20 / "station": the YAML-proven numbers — the code
    #     defaults std 0.5/reach "racket" are dead/arm-gameable, see HOPEPingPongHitter.yaml notes).
    post_strike_brake = RewTerm(func=mdp.post_strike_brake, weight=0.0,
        params={"command_name": "racket_target", "std": 0.5})
    hold_ready = RewTerm(func=mdp.hold_ready, weight=0.0,
        params={"command_name": "racket_target", "std": 1.5, "reach": 0.20, "reach_mode": "station"})
    #   hold_heading — heading restoration during holds (2026-07-08 rally-gate finding: deploy
    #     follow-throughs leave the robot 30-55° off the strike heading; nothing restored it and no
    #     trained state was ever yawed, so the runner gates engages on `PLANNER: yawed` and waits for
    #     an operator re-stand). exp(-yaw²/std²)·in_hold, yaw vs world +x. Pair with
    #     commands.motion.stand_start_yaw_range (the data source). See hope_rewards.hold_heading.
    hold_heading = RewTerm(func=mdp.hold_heading, weight=0.0,
        params={"command_name": "racket_target", "std": 0.6})
    #   lower_body_plant_imitation — demo lower-body pose tracked ONLY near the strike, tts-ramped
    #     (2026-07-09): cures the in-swing leg flail / base lunge ("下半身一直在飘") by settling the legs
    #     to the demo's planted stance at contact, while the footwork approach + holds stay FREE (the
    #     ramp is ~0 there — so y-footwork is not suppressed). SMALL weight; tracks leg JOINT angles
    #     (relative pose), NOT global base xy. See hope_rewards.lower_body_plant_imitation.
    lower_body_plant_imitation = RewTerm(func=mdp.lower_body_plant_imitation, weight=0.0,
        params={"command_name": "racket_target", "std": 0.5, "tts_std": 0.25,
                "asset_cfg": SceneEntityCfg("robot", joint_names=[
                    ".*_hip_pitch_joint", ".*_hip_roll_joint", ".*_hip_yaw_joint",
                    ".*_knee_joint", ".*_ankle_pitch_joint", ".*_ankle_roll_joint"])})
    # Foot discipline (declared 2026-07-07 after the pigeon-toe diagnosis): with lower-body
    # imitation absent (paper §V-A: B = above pelvis) the hip-yaw DOF are reward-free and the
    # policy toe-ins HARD while stepping — obs-CSV quantification on model_12200 in the AGI sim:
    # hip_yaw deviation p95 ±0.94 rad, max 1.69 (reference envelope ±0.41; ankle/hip_roll clean;
    # standing clean — it is ONLY the moving gait). Same pathology foot_orientation_discipline
    # was built for on 2026-07-05 (177 ran it at -0.3). Enable on a 12200 resume via
    # task.rewards.foot_orientation_weight=-0.3; gate: single-swing det >= 0.99 AND g25 oracle
    # 10/10 must both hold, then re-run the obs-CSV diag (hip_yaw p95 back inside ~0.41).
    foot_orientation = RewTerm(func=mdp.foot_orientation_discipline, weight=0.0,
        params={"command_name": "motion",
                "asset_cfg": SceneEntityCfg("robot", joint_names=[".*_hip_yaw_joint", ".*_hip_roll_joint", ".*_ankle_roll_joint"])})

    # --- POST-SWING RECOVERY (2026-07-09) — the RIGHT fix for the AGI-plant forward x-drift (replaces
    # the reverted demo leg-POSE plant, which re-imported the forward step-in and regressed AGI G3;
    # memory hope-legplant-regresses-agi). The POST-strike complement to base_station_settle (pre-strike):
    # kill the drift in the follow-through so the base enters the hold already settled on the x-station.
    # All follow-through-gated (_post_strike_window) and x/leg-only ⇒ lateral y-footwork stays FREE.
    # Weight 0.0 = OFF (RewardManager drops zero-weight terms; plain HitterPure stays byte-identical);
    # the Rally variant enables them. ⚠ POSITIVE follow-through income is the GAE channel that made
    # post_strike_brake precision-toxic — gate on single-swing G1 det FIRST (see hope_rewards header).
    #   x_settle — x-only station pull exp(-(base_x-station_x)²/std²), tight std 0.08 (the ~1cm x-lock).
    #   vx_quiet — world-vx quieting exp(-(base_vx)²/std²); vy FREE (post-strike y-steps untaxed).
    #   leg_quiet — -mean(qd_leg²) penalty (legs only, VELOCITY not demo pose = no forward-step-in bias).
    post_strike_x_settle = RewTerm(func=mdp.post_strike_x_settle, weight=0.0,
        params={"command_name": "racket_target", "std": 0.08, "t_hi": 1.2})
    post_strike_vx_quiet = RewTerm(func=mdp.post_strike_vx_quiet, weight=0.0,
        params={"command_name": "racket_target", "std": 0.15, "t_hi": 1.0})
    post_strike_leg_quiet = RewTerm(func=mdp.post_strike_leg_quiet, weight=0.0,
        params={"command_name": "racket_target", "t_hi": 1.2,
                "asset_cfg": SceneEntityCfg("robot", joint_names=[
                    ".*_hip_pitch_joint", ".*_hip_roll_joint", ".*_hip_yaw_joint",
                    ".*_knee_joint", ".*_ankle_pitch_joint", ".*_ankle_roll_joint"])})

    # BACKHAND racket↔left-hand SELF-COLLISION barrier (2026-07-09): Isaac trains with
    # enabled_self_collisions=False so the bh windup sweeps the paddle THROUGH the left hand for free;
    # AGI MuJoCo has self-collision → it HITS (Isaac-hides / AGI-reveals). Reward-side hinge on the
    # racket-center ↔ left-wrist distance, backhand-clip-only, 0 when clear. Weight 0.0 = OFF (V5
    # enables it). See hope_rewards.backhand_left_hand_clearance + memory hope-xdrift-v4-v5.
    backhand_left_hand_clearance = RewTerm(func=mdp.backhand_left_hand_clearance, weight=0.0,
        params={"command_name": "racket_target", "margin": 0.15, "left_body_name": "left_wrist_yaw_Link"})


@configclass
class HOPEPingPongHitterPureAgibotA3EnvCfg(HOPEPingPongAgibotA3EnvCfg):
    """Faithful HITTER MDP on the A3 (110-D hitter_pure actor contract). Code defaults below MIRROR
    cfg/task/HOPEPingPongHitterPure.yaml so eval/verify scripts that bypass train.py see the same
    task — keep the two in sync (the YAML wins at train time)."""

    obs_mode: str = "hitter_pure"
    observations: HOPEObservationsHitterPureCfg = HOPEObservationsHitterPureCfg()
    rewards: HOPEHitterPureRewardsCfg = HOPEHitterPureRewardsCfg()
    terminations: HOPEDeployParityTerminationsCfg = HOPEDeployParityTerminationsCfg()

    def __post_init__(self):
        super().__post_init__()
        # HITTER episode: 10 s, multiple swings, no hold phase (consecutive strikes train the
        # between-swing recovery; deploy idle is the runner's static-stand handoff, not a policy
        # state). stand_start_prob keeps the deploy entry in-distribution (min-hold 25 = 0.5 s to
        # settle stand -> windup); post-swing buffer starts OFF (not in the paper).
        self.episode_length_s = 10.0
        self.commands.motion.hold_steps_range = (0, 0)
        self.commands.motion.post_swing_start_prob = 0.0

        # Mirror the YAML's DR exactly (audit 2026-07-07): the inherited HOPEEventCfg default is
        # ±20%, but the pre-approved sim2real departure is ±15% — without this, every script that
        # bypasses train.py's override layer (verify/eval/export) ran a different DR distribution.
        self.events.randomize_pd_gains.params["stiffness_distribution_params"] = (0.85, 1.15)
        self.events.randomize_pd_gains.params["damping_distribution_params"] = (0.85, 1.15)

        C = self.commands.racket_target
        C.target_mode = "hitter_pure"
        C.normal_mode = "velocity"  # §IV-C: racket plane ⊥ velocity at impact (LEARNED, not ref-locked)
        # Unified fh+bh policy strikes with OPPOSITE paddle faces (forehand=red/+Y, backhand=black/−Y). Score
        # each swing's real striking face so the backhand's velocity-normal target is REACHABLE — a single +1
        # sign pins the backhand normal ~137° off velocity (fh normal 4°, bh 137° observed 2026-07-07),
        # holding backhand composite at 0 even though its pos/vel already pass.
        C.mount_normal_sign_per_clip = (1.0, -1.0)
        C.strike_phase_per_clip = (0.47, 0.333)  # blade-speed-peak re-plane (hopex clips, 2026-07-02)
        # Scalar mirror of the YAML (2026-07-09 audit): the per-clip tuple wins on two-clip runs, but a
        # SINGLE-clip load (bypass-path probes/forensics) falls back to this scalar — the cfg default
        # 0.46 would shift the strike frame by ~1 frame vs the trained 0.47 and corrupt strike metrics.
        C.strike_phase = 0.47
        C.strike_window_s = 0.12
        C.clean_reference_strike_velocity = True
        C.achieved_target_mix_prob = 0.0  # no HER in the paper
        # Independent STATION box (world xy around the env origin; paper Fig. 4 goes to ±0.75-0.8 m —
        # start at ±0.40 = the proven trained band, widen on resume once arrival is established).
        # X-PLANE LOCKED (2026-07-08): station x FIXED at spawn — mirrors HOPEPingPongHitterPure.yaml.
        C.base_target_x_range = (0.0, 0.0)
        C.base_target_y_range = (-0.40, 0.40)
        # STATION-RELATIVE racket boxes: x = the FIXED striking plane (blade reach of both clips
        # ≈ 0.70 m in front of the commanded station), y = per-swing non-overlapping bands centered
        # on each clip's natural lateral reach (fh −0.409 / bh +0.185), z = absolute height bands
        # centered on each clip's blade strike height (fh 0.82 / bh 1.03), half-width 0.15.
        # STRIKING-PLANE x FIX (2026-07-08): x 0.70 -> 0.51 (mirrors HOPEPingPongHitterPure.yaml).
        # 0.70 was 0.16-0.22 m TOO FAR vs the demo racket (pingpang_red_Link rel-station world x =
        # 0.484 fh / 0.542 bh), which forced the forward lunge/lean. 0.51 = demo midpoint so the
        # racket reaches the plane with the base AT the locked station.
        C.racket_pos_range_per_clip = (
            ((0.51, 0.51), (-0.65, -0.15), (0.67, 0.97)),  # forehand
            ((0.51, 0.51), (-0.05, 0.45), (0.88, 1.18)),   # backhand
        )
        # Blade-replaned per-clip velocity boxes (world frame, 2026-07-02 lineage).
        C.racket_vel_range_per_clip = (
            ((1.05, 2.05), (0.96, 1.96), (0.31, 1.11)),    # forehand
            ((1.61, 2.61), (-1.21, -0.21), (0.00, 0.71)),  # backhand
        )


@configclass
class HOPEPingPongHitterPureV2AgibotA3EnvCfg(HOPEPingPongHitterPureAgibotA3EnvCfg):
    """HITTER-PURE V2 — fix the PRE-STRIKE APPROACH: move to the y-station EARLY and SETTLE there before
    striking (2026-07-09). Everything structural (110-D hitter_pure actor contract, station/plane/velocity
    boxes, DR, terminations, NO hold machinery) is byte-identical to plain HitterPure, so a HitterPure
    checkpoint warm-resumes STRICTLY. Two reward-side departures, both PRE-STRIKE base shaping:

    1. base_station_settle (base→station deceleration) — teaches the robot to move to the y-station EARLY
       and SETTLE there BEFORE the strike, instead of the last-moment rush that arrives hot and lunges.
       "Move fast when far, calm when at the station" (see hope_rewards.base_station_settle); the linear
       v_des gradient is alive across the whole ±0.40 band, unlike base_position's dead far edge. It never
       pays for freezing far away, so the y-footwork is DIRECTED, not suppressed.

    2. base_position_std 0.20 → 0.35 — a live station pull across the WHOLE ±0.40 band (at std 0.20 the
       kernel is gradient-dead at a fresh 0.40 m station, ``exp(-4)≈0.018``, so far y-stations exert
       almost no early pull). Trades some fine-arrival precision for the early-move gradient — acceptable
       until we chase the paper's 1 cm arrival; pairs with base_station_settle.

    ⚠ NO lower_body_plant_imitation. The demo leg-POSE plant was REMOVED (2026-07-09 A/B, memory
    hope-legplant-regresses-agi): tracking the demo's leg-joint pose at strike re-imports the demo's
    forward step-INTO-the-ball; Isaac hides it (G1 passes) but the AGI explicit-PD plant amplifies it into
    accumulating station drift → topple (model_22200_legplant fell AGI G3 ×2; the no-plant control
    model_21500_pldamp = 0 falls/12). It is FUNDAMENTALLY at odds with the x-lock intent; not a tuning
    problem. If the in-swing flail ever needs quieting, penalize leg-joint VELOCITY near strike (no
    forward-step-in bias), NOT the pose. base_station_settle above is the "RIGHT direction" (reduces drift).

    NOTE (moving-gait toe-in): the SEPARATE "移动时脚姿势不正常" pigeon-toe pathology (hip_yaw p95 0.94 rad)
    is still NOT addressed (its fix is ``foot_orientation_weight=-0.3``, 177-proven — enable on a resume).

    GATE ON AGI G3 (pp_gate3_rally.sh — FALLS, not just Isaac G1): the plant negative result showed G1
    hides the deploy drift. base_station_settle should HELP G3, but verify. Code defaults MIRROR
    cfg/task/HOPEPingPongHitterPureV2.yaml (the YAML wins at train time) — keep the two in sync (Edit BOTH)."""

    def __post_init__(self):
        super().__post_init__()
        # (1) base→station deceleration: move to the y-station early + settle before the strike.
        self.rewards.base_station_settle.weight = 1.0
        # (2) widen the base-position kernel so the far ±0.40 stations are not gradient-dead.
        self.rewards.base_position.params["std"] = 0.35
        # NO plant: lower_body_plant_imitation stays 0.0 (inert) — demo leg-POSE tracking regresses AGI
        # G3 (drift → topple); see the class docstring + memory hope-legplant-regresses-agi.


@configclass
class HOPEPingPongHitterPureV2HoldAgibotA3EnvCfg(HOPEPingPongHitterPureV2AgibotA3EnvCfg):
    """V2 + a TRAINED "arrive → hold/settle → strike" rhythm (2026-07-09). V2's reward shaping only
    biases the base to arrive EARLY/CALM inside a fixed, clip-driven pre-strike window ("don't rush at
    the last moment") — it cannot make the swing WAIT for arrival, because the strike clock is the demo
    clip phase (hope_commands: ``time_to_strike = (strike_step − motion.time_steps)·dt``), not an arrival
    condition. This variant adds the ONLY machinery that makes the wait a real TRAINED state: a HOLD at
    every clip wrap (+ reset) that freezes ``tts`` POSITIVE at the windup while the base settles at the
    NEXT station, then arms the swing. That frozen-positive-tts hold is exactly the deploy runner's
    idle/wait clamp (the swing clock is generated robot-side at deploy — hope_commands note), so the
    policy learns to behave during the runner's "wait" instead of only generalizing to it.

    Structure per cycle: swing → wrap (resample station/target/clip) → HOLD 0.5-2.5 s (base moves to the
    new station via base_station_settle + base_position, and SETTLES via hold_ready) → windup → swing.
    (Inherits V2's approach shaping = base_station_settle + widened base_position std; NO plant — the demo
    leg-POSE plant was dropped, see the V2 docstring / memory hope-legplant-regresses-agi.)

    Departures from V2 (all reuse existing machinery — obs/critic/boxes/DR/terminations UNCHANGED, so a
    V2 or HitterPure checkpoint warm-resumes STRICTLY; the hold is a new STATE distribution the warm
    start must adapt to, expect a value-refit dip):
      * motion.hold_steps_range (0,0) → (25,125): the 0.5-2.5 s wait window @50 Hz (inside the deploy
        envelope; deliberately not longer — holds pay no goal income and dilute the strike batch).
      * hold_ready weight 0 → 1.0 (std 1.5 / reach 0.30 / station, include_ang_vel=False): settle-at-
        station income DURING the hold (planted feet × LINEAR stillness, in_hold-gated, pays only within
        0.30 m of the station). include_ang_vel=False drops the yaw-rate penalty so it does NOT fight the
        heading turn (see hold_heading below) — the fix for the RallyV3 conflict, letting this task settle
        AND re-square (RallyV3 had to pick one).
      * ARRIVAL-GATED RELEASE (racket.hold_until_settled): the hold EXTENDS past its [25,125] countdown
        while the base is not yet at the station (err>0.12 m) or still moving (>0.20 m/s), capped +100
        steps — so the swing arms only AFTER arrival ("到位才放行"), not on a fixed timer (the timer alone
        can release far/slow/disturbed swings before settling).
      * HEADING RECOVERY: hold_heading 1.0 (std 0.6) + yawed stand starts (±0.6) — re-square during the
        hold when entering yawed (deploy follow-through leaves 30-55°). Coexists with hold_ready via the
        include_ang_vel=False split above.
      * STREAMING PLANNER: midswing_resample_prob 0.05 — the target is refined mid-approach (paper Fig. 3),
        training "planner keeps commanding until contact" rather than one-target-per-swing.
      * WALKING POSTURE: foot_orientation -0.3 (177-proven pigeon-toe fix) — quiets the moving-gait toe-in
        (hip_yaw/roll + ankle_roll toward the reference footwork). ⚠ lower-body reward → gate on AGI G3.
      * post_strike_brake stays 0.0 — GAE precision-killer (0.994→0.866 on model_18000/rally2).

    ⚠ This is a LOT of simultaneous change vs the reward-only V2 (arrival-gate + heading + streaming +
    posture). Ablate any one via CLI (task.racket.hold_until_settled=false, task.rewards.hold_heading_weight=0,
    task.racket.midswing_resample_prob=0, task.rewards.foot_orientation_weight=0) — gate G1 precision FIRST,
    then AGI G3. Code defaults MIRROR cfg/task/HOPEPingPongHitterPureV2Hold.yaml — edit BOTH."""

    def __post_init__(self):
        super().__post_init__()  # V2: base_station_settle + widened base_position std (NO plant)
        self.episode_length_s = 10.0
        # Base recovery/settle hold at every wrap + reset (the MINIMUM hold; arrival-gate extends it).
        self.commands.motion.hold_steps_range = (25, 125)
        # (#2) ARRIVAL-GATED RELEASE: keep the hold open until the base has settled at the station.
        self.commands.racket_target.hold_until_settled = True
        # release also requires the base to be SQUARED (|yaw| < 0.30 rad ≈ 17°) — else the arrival-gate
        # could arm the swing while still yawed (position-settled but not turned back). Bites because
        # heading recovery is on below; hold_heading squares up during the (now extended) hold.
        self.commands.racket_target.hold_settle_yaw_thresh = 0.30
        # (hold rhythm) settle-at-station income; include_ang_vel=False so it does not fight the yaw turn.
        self.rewards.hold_ready.weight = 1.0
        self.rewards.hold_ready.params["reach"] = 0.30
        self.rewards.hold_ready.params["include_ang_vel"] = False
        # (#6) HEADING RECOVERY: re-square during the hold + yawed stand starts as the data source.
        self.rewards.hold_heading.weight = 1.0
        self.commands.motion.stand_start_yaw_range = (-0.6, 0.6)
        # (#4) STREAMING PLANNER: refine the target mid-approach (paper Fig. 3).
        self.commands.racket_target.midswing_resample_prob = 0.05
        # (#5) WALKING POSTURE: quiet the moving-gait toe-in (gate on AGI G3 — lower-body change).
        # hold_gate=True ZEROES it during the hold so it does NOT tax the hip/ankle rotation hold_heading
        # needs to re-square (foot_orientation_discipline penalizes deviation from the SQUARE stand ref in
        # the hold; without the gate it fights heading recovery — the RallyV3 fix). Swing-phase toe-in kept.
        self.rewards.foot_orientation.weight = -0.3
        self.rewards.foot_orientation.params["hold_gate"] = True
        # post_strike_brake stays OFF (0.0) — GAE precision-killer.
        self.rewards.post_strike_brake.weight = 0.0


@configclass
class HOPEPingPongHitterPureRallyAgibotA3EnvCfg(HOPEPingPongHitterPureAgibotA3EnvCfg):
    """CONTINUOUS-RALLY variant of HitterPure (2026-07-07). ⚠ POST-MORTEM: the Gate-2.5 P7 fall
    this task targeted turned out to be the C++ runner's Δ=0-idle artifact, NOT a training gap —
    fixed deploy-side (pp_policy.hpp idle-anchor; model_12200 = ORACLE 10/10 with ZERO
    retraining). The first run of this task (model_18000, weights 1.0) traded single-swing strike
    0.994→0.866 and still failed deploy (P4b) — archived, not deployed. KEPT default-off as
    tooling for future genuine multi-swing robustness (e.g. station widening toward ±0.75 m);
    see the YAML header post-mortem for the lessons (brake ≤0.3, hold_ready reach 0.30, gate any
    candidate on single-swing det ≥0.95 FIRST).

    Mechanics (all existing machinery, unchanged facts): same 110-D contract/boxes/DR/
    terminations as Pure (strict warm-resume works); swing -> follow-through BRAKE -> 0.5-2.5 s
    HOLD (settle at the NEXT station — the wrap resamples target+station+clip BEFORE the hold)
    -> windup -> swing, 3-4 swings per 16 s episode, fh/bh 50/50 per wrap. Hold obs: tts frozen
    POSITIVE at the windup value (== the runner's idle clamp), ready-stand reference, imitation
    auto-zeroed (*_swing_only), base_position live toward the new station. Code defaults MIRROR
    cfg/task/HOPEPingPongHitterPureRally.yaml — edit BOTH."""

    def __post_init__(self):
        super().__post_init__()
        # 10 s (stage-2 x-lock recipe): 16 s dilutes the strike batch (v2 lesson); the warm-start
        # already has the strike, so keep episodes short and let the ~2 held cycles teach recovery.
        # THE structural change: a real recovery window at EVERY wrap (and reset). 25-125 steps =
        # 0.5-2.5 s @50 Hz — inside the deploy envelope (runner hold_recover_s 2.5 s policy-active,
        # scripted P7 holds ~4.5 s). Deliberately NOT longer: holds pay no goal income, so hold
        # steps dilute strike-gradient sample efficiency.
        self.commands.motion.hold_steps_range = (25, 125)
        # Recovery income (weights mirror cfg/task/HOPEPingPongHitterPureRallyV3.yaml — edit BOTH;
        # the YAML wins at train time). STAGE-2 x-LOCK recipe (2026-07-09, warm-start from the
        # model_18400 striker; STAGE 1 of the yaw curriculum):
        #   brake 0.0 — post_strike_brake is GAE-toxic (collapsed single-swing composite 0.994→0.866
        #     on model_18000 and again on rally2 run 10-44-58). It was there to arrest forward
        #     follow-through drift; the x-LOCK (base_target_x [0,0] + base_position) now does that
        #     natively (deploy drift 0.008 m/swing), so brake stays OFF and precision is preserved.
        #   hold_ready 0.0 — its kernel rewards ZERO angular velocity, directly fighting the
        #     re-squaring turn hold_heading teaches; recovery is hold_heading's job alone.
        #   hold_heading 1.0 (std 0.6) — the validated heading-recovery income (model_15500 -> G3
        #     0 rescues); paired with yawed stand starts as the data source.
        #   stand_start_yaw yaw CURRICULUM: ±0.35 (STAGE 1, passed G1 @model_19600 0.988) -> ±0.6
        #     (STAGE 2, covers the deploy 29-55° residual) -> ±0.9. Widen one step per resume, only
        #     after G1 re-passes (flat ±0.9 from iter 0 was v2's OOD hit).
        #   foot_orientation -0.8 + hold_gate — fix the toe-in on the y-steps, but ZEROED during
        #     the hold so it does not tax the hip-yaw re-squaring hold_heading is teaching.
        self.episode_length_s = 10.0
        self.rewards.post_strike_brake.weight = 0.0
        self.rewards.hold_ready.weight = 0.0
        self.rewards.hold_heading.weight = 1.0
        self.commands.motion.stand_start_yaw_range = (-0.6, 0.6)
        # YAML-mirror (2026-07-09 audit): the V3/V4/V5 YAMLs all train with min-hold 50 (yawed stand
        # starts get >=1.0 s to re-square before the clip arms) but the MotionCommandCfg default is 25 —
        # bypass-path scripts (verify_*, probes) built from the registry would otherwise run a different
        # stand-entry distribution than the checkpoints trained on.
        self.commands.motion.stand_start_min_hold = 50
        self.rewards.foot_orientation.weight = -0.8
        self.rewards.foot_orientation.params["hold_gate"] = True
        # DISABLED 2026-07-09 (weight 0). NEGATIVE RESULT: at 0.5 this passed G1 (model_22200 det
        # composite 0.983) but FELL in AGI G3 within 6 serves ×2 (drift 1.4-2.0 m) vs control
        # model_21500_pldamp 0 falls / 12 (drift 0.11 m). Demo leg-POSE imitation re-imports the
        # demo's forward step-into-the-ball, fighting the x-lock; the AGI explicit-PD plant amplifies
        # it into an accumulating station drift → topple. For flail-quieting use a leg-joint VELOCITY
        # penalty near strike, NOT this pose tracking. See memory hope-legplant-regresses-agi.
        self.rewards.lower_body_plant_imitation.weight = 0.0
        # POST-SWING x-DRIFT recovery (post_strike_x_settle / vx_quiet / leg_quiet) is a SEPARATE
        # single-variable experiment — it lives in HOPEPingPongHitterPureRallyV4 (the subclass below +
        # its YAML), NOT here. V3 stays the CLEAN HEADING recipe that produced model_21500, so a V3 ckpt
        # is unambiguous. The 3 RewTerms keep their weight-0 base default in this recipe.


@configclass
class HOPEPingPongHitterPureRallyV4AgibotA3EnvCfg(HOPEPingPongHitterPureRallyAgibotA3EnvCfg):
    """V4 = the V3 rally HEADING recipe + ONLY the 3 POST-SWING x-DRIFT recovery terms (2026-07-09).

    Split out from V3 for a CLEAN single-variable test (user request): warm-resume model_21500_pldamp
    (which the V3 recipe already produced — heading recovery validated, AGI G3 0 falls) and add the
    x-drift terms as the SOLE new gradient, so any G1 change is unambiguously theirs. Inherits V3's
    heading machinery UNCHANGED (hold_heading 1.0, yaw ±0.6, foot_orientation −0.8 hold-gated, brake 0,
    leg-POSE plant 0) — that is NOT re-tested, only MAINTAINED so the warm-start behavior is preserved;
    the x-drift weights are the only values that differ from the 21500 recipe.

    The 3 terms (hope_rewards.py POST-SWING RECOVERY — follow-through-gated, x/leg-only ⇒ y-footwork FREE):
    post_strike_x_settle 2.0 (x-only station pull) + post_strike_vx_quiet 1.0 (world-vx quieting, vy free)
    + post_strike_leg_quiet −0.02 (leg-VELOCITY, not demo pose). ⚠ POSITIVE follow-through income = the
    GAE channel that made post_strike_brake precision-toxic — GATE single-swing G1 det FIRST every ~200
    iters; if <0.95 lower vx_quiet first, then x_settle, or push t_hi later. The drift is an AGI-plant
    effect Isaac under-shows → decide on AGI G3 (pp_gate3_rally.sh), not G1. Code defaults MIRROR
    cfg/task/HOPEPingPongHitterPureRallyV4.yaml (edit BOTH; the YAML wins at train time)."""

    def __post_init__(self):
        super().__post_init__()  # V3 heading recipe (x-drift RewTerms at their weight-0 base default)
        self.rewards.post_strike_x_settle.weight = 2.0
        self.rewards.post_strike_vx_quiet.weight = 1.0
        self.rewards.post_strike_leg_quiet.weight = -0.02


@configclass
class HOPEPingPongHitterPureRallyV5AgibotA3EnvCfg(HOPEPingPongHitterPureRallyV4AgibotA3EnvCfg):
    """V5 = THE FINAL SYNTHESIS (2026-07-09): the V3/V4 rally recipe (heading recovery + x-drift trio —
    the only lineage that has passed AGI G3, model_21500_pldamp 0 falls/12) MERGED with V2Hold's
    "arrive → settle → strike" rhythm. One task covering the whole HITTER demo loop: y-only footwork to
    the station, x locked to the fixed plane, settle BEFORE the swing arms, strike on pos+vel+normal,
    recover to a stable hold and repeat.

    Inherits from V4 (= V3 + x_settle 2.0 / vx_quiet 1.0 / leg_quiet −0.02): strike block, x-lock,
    hold_heading 1.0, yaw ±0.6 stage-2 curriculum, foot_orientation −0.8 hold-gated, brake 0, leg-POSE
    plant 0 (all UNCHANGED — the 21500 recipe). Adds V2Hold's four settle features, each phase-gated to
    its own phase (no new cross-phase GAE channel) and CLI-ablatable:
      * base_station_settle 1.0 — pre-strike directional decel toward the station (V2's approach fix;
        base_position_std stays 0.20, NOT V2's 0.35 — settle covers the far field, 0.35 is the
        177-lineage lazy-optimum precedent, and 0.20 is zero warm-start delta).
      * hold_ready 1.0 (reach 0.30 / station / include_ang_vel=False) — in-hold LINEAR-stillness settle
        income; the ang-vel split is what lets it coexist with hold_heading (V3 had to drop it).
      * ARRIVAL-GATED RELEASE (hold_until_settled) — the hold extends until AT station + calm + squared
        (yaw_thresh 0.30, bites because hold_heading is on); swing arms only AFTER arrival.
      * midswing_resample 0.05 — streaming-planner target refinement (deploy-honest, paper Fig. 3).

    Warm-resume model_21500_pldamp STRICTLY (identical 110-D contract/boxes/DR/terminations; the hold
    already existed in its training, so the new state dist is only the arrival extension). Gate ladder:
    G1 det precision every ~200 iters (tripwire; ablate vx_quiet → x_settle → t_hi → midswing on
    failure) → G2 rhythm (base_pos_error_pre_strike + base_speed_at_strike + hold_extra_steps) → G2.5
    recovery proxy → AGI G3 pp_gate3_rally.sh (DECISIVE) → G4 footwork spread (lower hold_ready weight/
    reach if frozen). Code defaults MIRROR cfg/task/HOPEPingPongHitterPureRallyV5.yaml (the YAML wins
    at train time; edit BOTH)."""

    def __post_init__(self):
        super().__post_init__()  # V4 = V3 heading recipe + x-drift trio (21500 lineage values)
        # (pre-strike) V2's approach shaping: move to the y-station early, settle before the strike.
        self.rewards.base_station_settle.weight = 1.0
        # (in-hold) settle-at-station income; include_ang_vel=False so it does not fight hold_heading's
        # re-squaring turn (the V3 conflict). reach 0.30 = stillness pays only near the station.
        self.rewards.hold_ready.weight = 1.0
        self.rewards.hold_ready.params["reach"] = 0.30
        self.rewards.hold_ready.params["include_ang_vel"] = False
        # (in-hold) ARRIVAL-GATED RELEASE: hold extends until at-station + calm + squared (到位才放行).
        self.commands.racket_target.hold_until_settled = True
        self.commands.racket_target.hold_settle_yaw_thresh = 0.30
        # Cap 50 (1 s), NOT the cfg default 100: the extension makes hold length policy-CONTROLLABLE
        # while ~3-5 weight/step of in-hold income stays live — the anti-farming invariant the brake
        # post-mortem established is "positive-income windows must not be policy-stretchable". Settled
        # states out-earn unsettled ones (~5 vs ~3 per step), so the farm gradient is weak, but the cap
        # bounds it structurally; expected [25,125] countdown + 50 covers a full-band station flip.
        # Tripwire: a RISING hold_extra_steps trend in wandb = the policy is farming the extension —
        # ablate with task.racket.hold_until_settled=false.
        self.commands.racket_target.hold_settle_max_extra_steps = 50
        # (any swing) streaming planner: refine the target mid-approach (deploy-honest).
        self.commands.racket_target.midswing_resample_prob = 0.05
        # BACKHAND self-collision barrier: keep the paddle clear of the left hand on the bh windup
        # (Isaac has self-collision OFF so it clips for free; AGI MuJoCo hits — user saw it on model_23300).
        # Hinge on racket↔left-wrist dist, bh-clip-only, 0 when clear. Mirrors HOPEPingPongHitterPureRallyV5.yaml.
        self.rewards.backhand_left_hand_clearance.weight = -2.0


@configclass
class HOPEPingPongHitterPureRallyV6AgibotA3EnvCfg(HOPEPingPongHitterPureRallyV4AgibotA3EnvCfg):
    """CLEAN REBUILD from the striker model_18400 (2026-07-09). = V4 (x-drift trio + the V3 heading
    recipe) + backhand_left_hand_clearance, with the yaw curriculum RESTARTED at ±0.35 — and DELIBERATELY
    NONE of V5's hold-rhythm stack (hold_ready include_ang_vel=False, base_station_settle, arrival-gated
    hold, midswing), which regressed AGI G3 (fell serve 12, idle base-wobble 0.69 vs V4 0.11 — the
    include_ang_vel=False removed angular stillness so the base spun in the hold; memory hope-xdrift-v4-v5).

    WHY 18400 (not the 21500 lineage): 18400 is the uncontaminated x-lock striker (G1 0.998) — no heading
    curriculum, no hold machinery, no cruft baked into the weights. Warm-resuming it re-trains on the
    current pldamp-folded plant (better AGI alignment) and re-learns heading cleanly from ±0.35 (18400
    never saw yaw, so ±0.6-from-scratch would be the v2 OOD collapse — the curriculum MUST restart at
    ±0.35; widen ±0.35→±0.6→±0.9 one step per resume, only after G1 re-passes).

    So V6 carries exactly the VALIDATED set — x-lock, x-drift containment (V4 = first G3 PASS), heading
    recovery (the 21500 lineage cleared G3), backhand-clearance (the new self-collision fix) — and nothing
    that failed. Warm-resume logs/rsl_rl/agibot_a3_hope_hitter_pure/2026-07-08_23-03-18/model_18400.pt.
    Code defaults MIRROR cfg/task/HOPEPingPongHitterPureRallyV6.yaml (edit BOTH; the YAML wins at train)."""

    def __post_init__(self):
        super().__post_init__()  # V4: x-drift trio + V3 heading recipe (hold_heading 1.0, yaw ±0.6, foot_orient)
        # yaw CURRICULUM restarted at STAGE 1 (±0.35): 18400 never saw yaw, so ±0.6 from scratch = OOD (v2's error).
        self.commands.motion.stand_start_yaw_range = (-0.35, 0.35)
        # backhand racket↔left-hand self-collision barrier (the one fix from the V5 attempt worth keeping).
        self.rewards.backhand_left_hand_clearance.weight = -2.0
        # (2026-07-09, user request) MOVE-TO-y-STATION-then-strike: base_station_settle = the directional
        # PACE term (v_des = unit(station−base)·clamp(v_gain·dist)) that drives crisp y-footwork to the
        # commanded station and settles there before the strike — base_position at std 0.20 is
        # gradient-dead at a fresh ±0.40 y-station, so it cannot give this alone. base_station_settle was
        # in the V5 stack but was NOT the V5 failure cause (that was hold_ready-noangvel + arrival-hold);
        # it is pre_strike/linear/directional = benign. Widen base_position std 0.20→0.35 so the far
        # y-stations are not gradient-dead for the position pull either.
        self.rewards.base_station_settle.weight = 1.0
        self.rewards.base_position.params["std"] = 0.35


@configclass
class HOPEPingPongHitterPureRallyV8TerminationsCfg(HOPEDeployParityTerminationsCfg):
    """V8 split of the z-deviation guard (2026-07-13, G1 forensics): the base ee_body_pos
    (|body_z − reference_z| > threshold on ankles+wrists, 0.25 m) was fighting the TASK'S OWN
    z target band — [0.85,1.30] demands up to 0.32 m of wrist-z deviation from the v13 demo
    contact (0.982), so the guard silently truncated ~15% of swings (the high/low balls) and
    read as pre_strike_fall 0.15 with ZERO physical falls (A/B at identical policy: threshold
    0.25→0.45 ⇒ pre_fall 0.154→0.013, base_roll 0.3°, post_strike_fall 0.0000).
    Split: ankles KEEP 0.25 (the real trip/fall guard — demo & stepping feet stay ≤0.10 m);
    wrists get 0.45 (task z band 0.32 + tracking margin 0.13). The flat-cfg __post_init__
    patcher re-points ee_body_pos at FEET+HANDS, so the V8 __post_init__ re-narrows it to
    FEET after super()."""

    ee_wrist_pos = DoneTerm(
        func=mdp.bad_motion_body_pos_z_only,
        params={"command_name": "motion", "threshold": 0.45, "body_names": list(A3_HAND_BODIES)},
    )


@configclass
class HOPEPingPongHitterPureRallyV8RewardsCfg(HOPEHitterPureRewardsCfg):
    """V8 rewards = the Pure/V4 declaration set + the FinalV2-proven ankle q_des rail debt.

    rally_ankle_qdes_saturation penalizes PRE-CLAMP ankle-roll position targets beyond
    ±safe_abs∩hard-limits — the commands that produce the visible ankle-roll (崴脚) artifact
    while stepping. Surgical (raw action, two joints, Huber) and phase-windowed; complements
    foot_orientation (achieved-posture discipline) without touching the swing."""

    rally_ankle_qdes_saturation = RewTerm(
        func=mdp.rally_ankle_qdes_saturation_penalty, weight=-0.20,
        params={
            "command_name": "racket_target", "action_name": "joint_pos",
            "asset_cfg": SceneEntityCfg(
                "robot", joint_names=[".*_ankle_roll_joint"], preserve_order=True,
            ),
            "safe_abs": 0.20, "std": 0.10, "t_pre": 1.40, "t_post": 1.80,
        },
    )

    # Fresh-bootstrap far-field normal gradient (2026-07-11, run 19-04-23 evidence): the v7
    # forehand demo swings +Y cross-court while the center-return target normal (= velocity
    # direction) is +X-dominant — the achieved face starts ~95 deg from target, which is 5.5
    # sigma outside the positive Gaussian (std 0.30 rad) => exactly ZERO gradient; fh normal
    # error sat PINNED at 90-99 deg for 500 iters while pos/vel learned fine. Same failure and
    # same fix as the FinalV3 fresh-run repair (validated: normal 66 deg -> 14.6/12.9 deg).
    # Strike-window smooth-L1 on clamp(angle - margin, 0): live gradient at ANY angle, exactly
    # zero once within the 5.7 deg margin => does not distort the converged optimum.
    racket_normal_alignment_debt = RewTerm(
        func=mdp.racket_normal_alignment_debt,
        weight=-2.0,
        params={"command_name": "racket_target", "margin": 0.10, "std": 0.35},
    )

    # Upper-body imitation reverted to the FULL A3_UPPER_TRACKED set (2026-07-13, back to the
    # 5bfacde7 state): the 0712 wrist exclusion treated a CLIP artifact — Step 9 (GVHMR)
    # mis-estimates the racket-wrist forearm roll (video face-forward, SMPL face-up ~72°), so
    # wrist imitation fought the flat-hit normal target and the shared mean-then-exp kernel
    # read 0.0000 all run. Root fix is clip-side (face-aligned wrist re-solve → v13); with a
    # face-correct clip the wrist MUST imitate again or face style goes unguided.

    # HOLD POSTURE VACUUM fix (2026-07-13 viewer finding): during holds every posture term is
    # gated off (Cartesian imitation swing_only, foot_orientation hold-gated) and the torque
    # regularizer then actively prefers the hanging twisted arm. Upper body ONLY — legs
    # deliberately excluded (clip ready is a deep leg crouch, receipt leg_ready_default_rms
    # 0.53 rad; a leg target would refight the implicit stand attractors = the historical
    # splayed-feet bug). Target = the current clip's frame-0 ready pose, so hold == swing
    # entry and stand-starts learn raise-to-ready.
    hold_upper_pose_imitation = RewTerm(
        func=mdp.hold_upper_pose_imitation, weight=1.0,
        params={
            "command_name": "motion", "std": 0.5,
            "asset_cfg": SceneEntityCfg(
                "robot",
                joint_names=["waist_.*", ".*_shoulder_.*", ".*_elbow_joint", ".*_wrist_.*"],
            ),
        },
    )
    # Stance rail — PARKED AT WEIGHT 0 pending its own ablation (user 2026-07-13; the
    # post_strike_brake_weight-0.0 idiom): plumbing stays wired, training unaffected, the
    # ablation is a one-number YAML flip. Designed weight −0.5; calibration in the YAML.
    stance_width = RewTerm(
        func=mdp.stance_width_band, weight=0.0,
        params={
            "lo": 0.25, "hi": 0.35, "std": 0.05,
            "asset_cfg": SceneEntityCfg("robot", body_names=list(A3_FEET_BODIES)),
        },
    )


@configclass
class HOPEPingPongHitterPureRallyV8AgibotA3EnvCfg(HOPEPingPongHitterPureRallyV4AgibotA3EnvCfg):
    """V8 = the Pure→V3→V4 staged champion recipe MERGED into ONE fresh-trainable task, on the
    approved v12 clips (2026-07-12). The staged lineage (HitterPure fresh → RallyV3 heading resume →
    RallyV4 x-drift resume → model_23300) is the only recipe that passed AGI G3 (0 falls / 12), but
    it cannot be reproduced on new clips without replaying three manual warm-resume stages. V8 bakes
    the whole ladder into a single run and fixes the documented staged-training gaps:

      1. v13 FACEFIX GEOMETRY (sha-pinned receipt hitter_rally_v13_facefix_receipt.json): the
         v12 videos' retargets with the Step-10.5 wrist re-solve (GVHMR wrist-roll artifact —
         the structural cause of bh normal_pass≡0 across five clip generations — corrected;
         face-vel 8.9°/15.6°). Phases 0.38679245/0.44444444 (frames 41/107, 48/109; windups
         0.82/0.96 s), PER-CLIP planes fh 0.65 / bh 0.50, ARM-REACH y bands fh [−0.48, −0.40]
         / bh [−0.13, −0.05] about the receipt contacts (−0.441/−0.082; disjoint around the
         planner split −0.25), z [0.85, 1.30], FACE-CONE velocity boxes (the velocity box IS
         the face-normal target — centered on the FIXED faces), mount signs (1,−1). The band
         is what makes "stay if reachable" POSSIBLE: nearby balls are returned by arm extension
         from the CURRENT stance; only balls outside it warrant a station step.
      2. MOSTLY-STATIONARY STATION MIXTURE (user intent 2026-07-11: movement only when it buys
         return quality, most balls taken in place): 50% same station / 20% 3-12 cm adjust /
         30% 12-35 cm real step (also closes the old 12-20 cm sampling hole), explicit planner
         side. base_position pays for being AT the commanded station — a "same" command earns
         it by standing still, so movement is command-selected, never rewarded per se.
      3. LONG-HOLD TAIL (hold_long_prob 0.15 → U[150,300] steps): deploy active-idle windows run
         4-7 s; the rallyv2-generation G3 blocker was instability in exactly those windows that
         the base [25,125] hold range never covers.
      4. YAW RAMP (stand_start_yaw_ramp_steps): the ±0.6 heading-recovery band ramps in from 0
         over ~4000 iters — a fresh run starting at the full band never converges its heading
         (V6 regression), and the manual fix was staged resumes.
      5. hold_ready 1.0 with include_ang_vel=True + heading_gate 0.25: the idle-feet prescription
         from the V4/V5 A/B — V4 (no hold_ready) still had intermittent foot convulsions, V5's
         include_ang_vel=False let the base spin 0.69 rad/s in holds; the near-square gate pays
         FULL stillness (linear+angular+feet) only after the hold_heading turn is done, so the
         two incomes never fight. Hold clocks stay exogenous (hold_until_settled=False) so the
         positive in-hold income is un-farmable (the brake post-mortem invariant).
      6. NO backhand clearance terms: the v7 backhand's measured blade↔left-hand minimum is
         0.574 m (receipt) — the hopex-era barrier is unnecessary.
      7. Fresh-only (train.py guard): warm-resume entropy/value-refit illusions were a recurring
         diagnosis sink, and a fresh run inherits the pldamp-folded plant alignment the old
         champions predate.
      8. FOOT DISCIPLINE while stepping (user 2026-07-11: no ankle-rolling/崴脚 during moves):
         rally_ankle_qdes_saturation −0.20 (FinalV2-proven raw pre-clamp ankle-roll q_des
         debt — the ankle-rail commands ARE the visible ankle-roll artifact) on top of the
         inherited foot_orientation −0.8 (swing-phase ankle/hip posture vs reference) and
         hold_ready's planted-feet stillness income.

    Inherits V4 unchanged: strike block 14/14/5 + base_position 2.0/0.20, x-lock station,
    hold_heading 1.0/std 0.6, foot_orientation −0.8 hold-gated, x-drift trio 2.0/1.0/−0.02,
    brake 0, plant 0, DeployParity terminations, DR ±15%. Gate ladder: G1 det (eval_deterministic
    task=HOPEPingPongHitterPureRallyV8, composite ≥0.95 / bh ≥0.93 / pre_fall ≤2%) → G2.5 →
    AGI G3 pp_gate3_rally.sh (DECISIVE — Isaac hides the plant effects). Code defaults MIRROR
    cfg/task/HOPEPingPongHitterPureRallyV8.yaml (the YAML wins at train time; edit BOTH)."""

    rewards: HOPEPingPongHitterPureRallyV8RewardsCfg = HOPEPingPongHitterPureRallyV8RewardsCfg()
    terminations: HOPEPingPongHitterPureRallyV8TerminationsCfg = HOPEPingPongHitterPureRallyV8TerminationsCfg()

    def __post_init__(self):
        super().__post_init__()  # V4 = V3 heading recipe + x-drift trio
        # The flat-cfg patcher re-points ee_body_pos at FEET+HANDS (case-fixed); narrow it back
        # to FEET-only here — the wrists moved to the dedicated 0.45 ee_wrist_pos guard (see
        # HOPEPingPongHitterPureRallyV8TerminationsCfg: the 0.25 wrist-z guard fought the
        # task's own z band and truncated ~15% of swings as fake "pre-strike falls").
        self.terminations.ee_body_pos.params["body_names"] = list(A3_FEET_BODIES)
        C = self.commands.racket_target
        M = self.commands.motion
        # --- v13 FACEFIX geometry (receipt hitter_rally_v13_facefix_receipt.json): Step-10.5
        # wrist re-solve fixed the GVHMR face-up artifact (face-vel 8.9/15.6 deg). PER-CLIP
        # planes fh 0.65 / bh 0.50 (the fix moved the contacts to 0.630/0.525), ARM-REACH y
        # bands ±0.04 about the receipt contacts (fh −0.441 / bh −0.082, split −0.25),
        # z [0.85,1.30] (contacts 0.982/1.050); FACE-CONE vel boxes (below); windups
        # 0.82/0.96 s (deploy engage gate 0.9*0.82 = 0.738 s — runner prefix-skip TODO) ---
        C.strike_phase = 0.38679245
        C.strike_phase_per_clip = (0.38679245, 0.44444444)
        C.racket_pos_range_per_clip = (
            ((0.65, 0.65), (-0.48, -0.40), (0.85, 1.30)),
            ((0.50, 0.50), (-0.13, -0.05), (0.85, 1.30)),
        )
        # THE VELOCITY BOX IS THE FACE-NORMAL TARGET (hitter_pure + normal_mode=velocity ->
        # `normal = vel.clone()`), so it must sit inside the striking face's REACHABLE cone.
        # v13 FACE-CONE boxes: centers = 2.1 m/s along the FIXED strike face normals
        # (fh (0.83,0.09,0.55); bh (1.0,-0.08,0.02) with z floored at 0 for the deploy runner's
        # z_lo>=0 gate, ~8 deg off-face), ±0.50 half-widths; demo clean velocities
        # (1.77,0.25,1.75)/(2.35,-0.10,-0.57) in/near box. Imitation and normal target agree
        # BY CONSTRUCTION now that the face itself is fixed clip-side.
        C.racket_vel_range_per_clip = (
            ((1.24, 2.24), (-0.31, 0.69), (0.66, 1.66)),
            ((1.60, 2.60), (-0.66, 0.34), (0.00, 0.54)),
        )
        C.base_target_y_range = (-0.35, 0.35)
        # v12 true striking faces: fh RED(+Y), bh BLACK(−Y) — back to the hopex-lineage signs
        # (v11 was the inverted generation).
        C.mount_normal_sign_per_clip = (1.0, -1.0)
        # --- MOSTLY-STATIONARY station mixture (wrap steps relative to the previous station):
        # 50% stay / 20% 3-12 cm adjust / 30% 12-35 cm real step (no 12-20 cm hole) ---
        C.station_y_step_range = (0.12, 0.35)
        C.station_y_same_prob = 0.50
        C.station_y_small_step_prob = 0.20
        C.station_y_small_step_range = (0.03, 0.12)
        C.station_side_explicit = True
        C.ready_monitor_step_range = (0.0, 0.35)
        # --- single-run curriculum + deploy-idle coverage ---
        M.stand_start_yaw_ramp_steps = 96000  # ≈4000 iters × 24 steps/iter to full ±0.6
        M.hold_long_prob = 0.15
        M.hold_long_steps_range = (150, 300)  # 3-6 s @50 Hz, the deploy active-idle band
        # --- idle stillness: full kernel, paid only near-square (see class docstring #5) ---
        self.rewards.hold_ready.weight = 1.0
        self.rewards.hold_ready.params["reach"] = 0.30
        self.rewards.hold_ready.params["include_ang_vel"] = True
        self.rewards.hold_ready.params["heading_gate"] = 0.25
        # Exogenous hold clock only (V5's arrival-gated extension regressed AGI G3).
        C.hold_until_settled = False
        # Mirror the YAML exactly (the inherited HitterPure default is -1.0e-5; the whole V4
        # lineage trained at -3.0e-5 via the YAML — close the bypass-script gap here).
        self.rewards.joint_torques.weight = -3.0e-5


##
# HITTER-PURE + PRE-STRIKE STABILITY (2026-07-10) — a MINIMAL single-swing stability fix on TOP of plain
# HitterPure. NOT a rally/hold/arrival/V5/V6 task: everything structural is byte-identical to HitterPure
# (110-D hitter_pure actor contract, independent station sampling, fixed striking PLANE, strike phase /
# window, per-clip velocity/normal boxes, DR, terminations, x-LOCK base_target_x [0,0], base_position_std
# 0.20 — NOT widened to 0.35, NO hold_steps, NO base_station_settle / post_strike_brake / lower_body_plant
# / midswing). A HitterPure checkpoint warm-resumes STRICTLY (obs/critic/boxes unchanged).
#
# Problem targeted: on a SINGLE swing the base is still translating or turning when the swing arms, so the
# robot enters the hit carrying base xy velocity / yaw-rate → foot slip + odd compensation. Four small,
# narrowly-gated reward additions (all deploy-honest — IMU-measurable base state only, NO actor obs change):
#   1. backhand_left_hand_clearance −2.0 (margin 0.15) — the bh windup must not sweep the paddle through
#      the left hand (Isaac self-collision OFF hides it; AGI MuJoCo reveals it). Backhand-clip-only, 0 when
#      clear. (Already declared inert in HOPEHitterPureRewardsCfg; enabled here.)
#   2. pre_strike_base_vel_quiet −0.05 (std 0.20) — quiet base planar velocity pre-strike; gate
#      (t_min<tts<t_max) & ~strike_window ⇒ effective 0.12–0.30 s (strike window itself excluded).
#   3. pre_strike_base_angvel_quiet −0.05 (std 0.30) — quiet base YAW-RATE, same effective window.
#   4. strike_heading +0.5 (std 0.35, window 0.15 s) — face square (heading→+x) AT the strike.
# Terms 2–3 are BOUNDED gentle nudges (1−exp kernels) that can never out-vote the racket strike terms;
# term 4 is a small positive facing reward only around contact. See hope_rewards.py for the full rationale.
# Code defaults MIRROR cfg/task/HOPEPingPongHitterPurePreStrikeStable.yaml (the YAML wins at train time) —
# edit BOTH.
##


@configclass
class HOPEHitterPurePreStrikeStableRewardsCfg(HOPEHitterPureRewardsCfg):
    """HitterPure reward stack + the three NEW pre-strike / strike stability terms (backhand clearance is
    inherited from HOPEHitterPureRewardsCfg and enabled in the env __post_init__)."""

    # Quiet the base BEFORE the swing arms (short pre-strike window; NOT a full-episode velocity penalty).
    pre_strike_base_vel_quiet = RewTerm(
        func=mdp.pre_strike_base_vel_quiet, weight=-0.05,
        params={"command_name": "racket_target", "std": 0.20, "t_min": 0.05, "t_max": 0.30})
    pre_strike_base_angvel_quiet = RewTerm(
        func=mdp.pre_strike_base_angvel_quiet, weight=-0.05,
        params={"command_name": "racket_target", "std": 0.30, "t_min": 0.05, "t_max": 0.30})
    # Face square at the moment of contact (short window around the strike; NOT a full-episode heading lock).
    strike_heading = RewTerm(
        func=mdp.strike_heading, weight=0.5,
        params={"command_name": "racket_target", "std": 0.35, "window_s": 0.15})


@configclass
class HOPEPingPongHitterPurePreStrikeStableAgibotA3EnvCfg(HOPEPingPongHitterPureAgibotA3EnvCfg):
    """Plain HitterPure + the minimal pre-strike stability stack. Structural config (obs/boxes/DR/
    terminations/x-lock/base_position_std 0.20/no-hold) is inherited UNCHANGED from HitterPure."""

    rewards: HOPEHitterPurePreStrikeStableRewardsCfg = HOPEHitterPurePreStrikeStableRewardsCfg()

    def __post_init__(self):
        super().__post_init__()  # HitterPure structural defaults (10 s, no hold, x-lock, boxes, DR)
        # Enable the backhand racket↔left-hand self-collision barrier (inert at 0.0 in the base stack).
        self.rewards.backhand_left_hand_clearance.weight = -2.0
        self.rewards.backhand_left_hand_clearance.params["margin"] = 0.15


@configclass
class HOPEHitterPureRallyFinalRewardsCfg(HOPEHitterPureRewardsCfg):
    """Clean HitterPure strike stack plus the minimal RallyFinal stability/safety corrections.

    No term depends on a hold state, a ball state, or lower-body demo pose.  All penalties are
    bounded and phase-gated so the early lateral approach remains learnable.
    """

    strike_x_drift = RewTerm(
        func=mdp.strike_x_drift_penalty, weight=-0.75,
        params={"command_name": "racket_target", "margin": 0.04, "std": 0.08,
                "t_pre": 0.45, "t_post": 1.00})
    strike_x_velocity = RewTerm(
        func=mdp.strike_x_velocity_penalty, weight=-0.20,
        params={"command_name": "racket_target", "margin": 0.05, "std": 0.20,
                "t_pre": 0.45, "t_post": 1.00})
    pre_strike_station_settle = RewTerm(
        func=mdp.pre_strike_station_settle, weight=0.80,
        params={"command_name": "racket_target", "v_gain": 2.0, "v_max": 0.8,
                "std": 0.35, "t_max": 1.0})
    post_swing_base_quiet = RewTerm(
        func=mdp.post_swing_base_quiet, weight=-0.25,
        params={"command_name": "racket_target", "std": 0.25, "t_lo": 0.20, "t_hi": 1.00})
    post_swing_leg_quiet = RewTerm(
        func=mdp.post_swing_leg_quiet, weight=-0.08,
        params={"command_name": "racket_target", "std": 0.8, "t_lo": 0.20, "t_hi": 1.00,
                "asset_cfg": SceneEntityCfg("robot", joint_names=[
                    ".*_hip_pitch_joint", ".*_hip_roll_joint", ".*_hip_yaw_joint",
                    ".*_knee_joint", ".*_ankle_pitch_joint", ".*_ankle_roll_joint"])})
    settle_foot_slip = RewTerm(
        func=mdp.settle_foot_slip_penalty, weight=-0.10,
        params={"command_name": "racket_target",
                "asset_cfg": SceneEntityCfg(
                    "robot", body_names=A3_FEET_BODIES, preserve_order=True),
                "sensor_cfg": SceneEntityCfg(
                    "contact_forces", body_names=A3_FEET_BODIES, preserve_order=True),
                "std": 0.12, "station_reach": 0.15, "force_threshold": 10.0,
                "pre_t_max": 0.45, "strike_t_post": 0.10,
                "post_t_lo": 0.20, "post_t_hi": 1.00})
    backhand_left_arm_clearance = RewTerm(
        func=mdp.backhand_left_arm_clearance, weight=-1.50,
        params={"command_name": "racket_target", "hand_margin": 0.15,
                "forearm_margin": 0.12, "t_pre": 0.70, "t_post": 0.20,
                "hand_body_name": "left_wrist_yaw_Link", "elbow_body_name": "left_elbow_Link",
                "forearm_end_body_name": "left_wrist_roll_Link"})
    strike_front_facing = RewTerm(
        func=mdp.strike_front_facing, weight=0.50,
        params={"command_name": "racket_target", "base_std": 0.35, "torso_std": 0.30,
                "torso_rate_std": 1.00, "t_pre": 0.25, "t_post": 0.20,
                "torso_body_name": "torso_Link"})
    # Square-pelvis recovery in the two states the runner actually holds: frozen ready (tts=1.30)
    # and the complete post-swing tail.  This is not a hold reward or heading curriculum.
    rally_heading_debt = RewTerm(
        func=mdp.rally_heading_debt, weight=-0.10,
        params={"command_name": "racket_target",
                "yaw_margin": 0.12, "yaw_std": 0.25,
                "rate_margin": 0.15, "rate_std": 0.50, "heading_blend": 0.70,
                "ready_t_lo": 0.45, "ready_t_hi": 1.40,
                "post_t_lo": 0.35, "post_t_hi": 1.80})


@configclass
class HOPEPingPongHitterPureRallyFinalAgibotA3EnvCfg(HOPEPingPongHitterPureAgibotA3EnvCfg):
    """Final rally task rebuilt directly from clean HitterPure/model_18400.

    This class intentionally does NOT inherit Rally V4/V5/V6/V7: no hold rhythm, hold income,
    arrival-gated clock, midswing resampling, heading curriculum, foot-orientation stack, or
    lower-body plant imitation enters either the state distribution or reward.
    """

    rewards: HOPEHitterPureRallyFinalRewardsCfg = HOPEHitterPureRallyFinalRewardsCfg()

    def __post_init__(self):
        super().__post_init__()
        # Explicit clean timing/distribution contract (runner owns idle/rest rhythm).
        self.episode_length_s = 10.0
        self.commands.motion.hold_steps_range = (0, 0)
        self.commands.motion.stand_start_min_hold = 25
        self.commands.motion.stand_start_yaw_range = (0.0, 0.0)
        self.commands.motion.post_swing_start_prob = 0.0
        self.commands.motion.clip_switch_prob = 0.0

        C = self.commands.racket_target
        C.hold_until_settled = False
        C.midswing_resample_prob = 0.0
        C.base_target_x_range = (0.0, 0.0)
        C.base_target_y_range = (-0.35, 0.35)
        # v12 striking faces (receipt hitter_rally_v12_approved_receipt.json): back to the
        # hopex-lineage convention fh RED(+Y) / bh BLACK(−Y) — the v11 inversion is RETIRED.
        # Matches this task's own YAML ([1.0, -1.0]); the stale (-1.0, 1.0) default mis-scored
        # paddle faces in bypass-path evals/exports that skip train.py's YAML overlay.
        C.mount_normal_sign_per_clip = (1.0, -1.0)
        C.station_y_step_range = (0.20, 0.35)
        # Fixed clip-specific reach makes the runner inverse exact:
        # station_y = target_y - reach_y[clip].  All y diversity now comes from station footwork.
        C.racket_pos_range_per_clip = (
            ((0.51, 0.51), (-0.40, -0.40), (0.67, 0.97)),
            ((0.51, 0.51), (0.20, 0.20), (0.88, 1.18)),
        )
        C.achieved_target_mix_prob = 0.0
        C.adaptive_sigma = False
        # Preserve the model_18400 x-lock precision kernel; widening to 0.35 previously regressed G3.
        self.rewards.base_position.params["std"] = 0.20
        # Mirror the Final YAML regularizers for scripts that instantiate the Gym EnvCfg directly
        # without train.py's Hydra override translation.
        self.rewards.joint_torques.weight = -3.0e-5
        self.rewards.action_rate_l2.weight = -0.10
        self.rewards.joint_limit.weight = -10.0
        self.rewards.undesired_contacts.weight = -0.10
        # Explicitly pin every excluded/failed shared term off for bypass-path eval clarity.
        self.rewards.base_station_settle.weight = 0.0
        self.rewards.post_strike_brake.weight = 0.0
        self.rewards.hold_ready.weight = 0.0
        self.rewards.hold_heading.weight = 0.0
        self.rewards.lower_body_plant_imitation.weight = 0.0
        self.rewards.foot_orientation.weight = 0.0
        self.rewards.post_strike_x_settle.weight = 0.0
        self.rewards.post_strike_vx_quiet.weight = 0.0
        self.rewards.post_strike_leg_quiet.weight = 0.0
        self.rewards.backhand_left_hand_clearance.weight = 0.0


@configclass
class HOPEHitterPureRallyFinalV2RewardsCfg(HOPEHitterPureRallyFinalRewardsCfg):
    """RallyFinal plus deploy-ready foot discipline for the externally timed ready clamp.

    These are phase-gated regularizers: orientation debt is bounded, while ankle q_des uses a robust
    Huber debt so gross rail requests retain gradient. They do not reinstate the legacy global
    ``foot_orientation`` term or any lower-body pose imitation.
    """

    rally_foot_orientation = RewTerm(
        func=mdp.rally_foot_orientation_discipline, weight=-0.20,
        params={
            "command_name": "racket_target", "motion_command_name": "motion",
            "asset_cfg": SceneEntityCfg(
                "robot",
                # Hip-roll and ankle-roll clip references reach/exceed their hard limits and must
                # not be imitation goals. Hip yaw remains legal and supplies the anti-toe-in debt.
                joint_names=[".*_hip_yaw_joint"],
                preserve_order=True,
            ),
            "margin": 0.12, "std": 0.50, "t_pre": 1.40, "t_post": 1.80,
        },
    )
    rally_ankle_qdes_saturation = RewTerm(
        func=mdp.rally_ankle_qdes_saturation_penalty, weight=-0.20,
        params={
            "command_name": "racket_target", "action_name": "joint_pos",
            "asset_cfg": SceneEntityCfg(
                "robot", joint_names=[".*_ankle_roll_joint"], preserve_order=True,
            ),
            "safe_abs": 0.20, "std": 0.10, "t_pre": 1.40, "t_post": 1.80,
        },
    )


@configclass
class HOPEPingPongHitterPureRallyFinalV2AgibotA3EnvCfg(
    HOPEPingPongHitterPureRallyFinalAgibotA3EnvCfg
):
    """Causal RallyFinal revision matching the runner's fixed ready-clamp state distribution.

    The clock hold is sampled externally for every reset and wrap and expires after a fixed random
    duration.  It is never extended based on policy state, so there is no arrival-gated hold income
    or hold-farming path.  The runner remains the owner of the real readiness release decision.
    """

    rewards: HOPEHitterPureRallyFinalV2RewardsCfg = HOPEHitterPureRallyFinalV2RewardsCfg()

    def __post_init__(self):
        super().__post_init__()
        # 0.8--1.2 s at 50 Hz, sampled on EVERY reset and intra-episode wrap by MotionCommand.
        # The seeded initial 25--50 tick trial released only 72.1% of deterministic transitions and
        # degraded monotonically with step length; 40 ticks clears the measured 0.72--0.74 s p95
        # plus far-step/dwell margin without making the release policy-controlled.
        self.commands.motion.hold_steps_range = (40, 60)
        # Behaviourally redundant with the U[40,60] lower bound, but keeps bypass-path EnvCfg
        # inspection honest instead of advertising the inherited 25-tick stand minimum.
        self.commands.motion.stand_start_min_hold = 40
        self.commands.racket_target.hold_until_settled = False

        # Runner freezes forehand ready at tts=1.30.  V1 stopped all movement/settle shaping at
        # tts<=1.00 (or 0.45), making that repeated deploy state out of distribution.
        self.rewards.strike_x_drift.params["t_pre"] = 1.40
        self.rewards.strike_x_drift.params["t_post"] = 1.80
        self.rewards.strike_x_velocity.params["t_pre"] = 1.40
        self.rewards.strike_x_velocity.params["t_post"] = 1.80
        self.rewards.pre_strike_station_settle.params["t_max"] = 1.40
        self.rewards.post_swing_base_quiet.params["t_hi"] = 1.80
        self.rewards.post_swing_leg_quiet.params["t_hi"] = 1.80
        self.rewards.settle_foot_slip.params["pre_t_max"] = 1.40
        self.rewards.settle_foot_slip.params["post_t_hi"] = 1.80
        self.rewards.rally_heading_debt.weight = -0.25
        self.rewards.rally_heading_debt.params["post_t_lo"] = 0.20

        # Explicitly keep every policy-stretchable / failed shared hold and plant term disabled.
        self.rewards.hold_ready.weight = 0.0
        self.rewards.hold_heading.weight = 0.0
        self.rewards.lower_body_plant_imitation.weight = 0.0
        self.rewards.foot_orientation.weight = 0.0


# FinalV3 keeps the 110-D wire shape but makes the two neck channels deploy-honest: their
# q_des stays at the articulation default and the actor/critic see zero in those two applied-
# last-action slots, exactly like the A3 runner.
@configclass
class HOPEHitterPureRallyFinalV3ActionsCfg(ActionsCfg):
    joint_pos = mdp.ClampedJointPositionActionCfg(
        asset_name="robot",
        joint_names=[".*"],
        use_default_offset=True,
        passive_joint_names=mdp.A3_PASSIVE_HEAD_JOINT_NAMES,
    )


@configclass
class HOPEObservationsHitterPureRallyFinalV3Cfg(HOPEObservationsHitterPureCfg):
    @configclass
    class PolicyCfg(HOPEObservationsHitterPureCfg.HOPEPolicyHitterPureCfg):
        actions = ObsTerm(func=mdp.applied_last_action, params={"action_name": "joint_pos"})

    @configclass
    class CriticCfg(HOPEObservationsHitterPureCfg.HOPECriticHitterPureCfg):
        actions = ObsTerm(func=mdp.applied_last_action, params={"action_name": "joint_pos"})

    policy: PolicyCfg = PolicyCfg()
    critic: CriticCfg = CriticCfg()


@configclass
class HOPEHitterPureRallyFinalV3TerminationsCfg(HOPEDeployParityTerminationsCfg):
    """Only physical fall guards; reference deviation must be recoverable, not reset away."""

    anchor_pos = None
    anchor_ori = None
    ee_body_pos = None


@configclass
class HOPEHitterPureRallyFinalV3RewardsCfg(HOPEHitterPureRallyFinalV2RewardsCfg):
    # V7 compatibility: the hold is the station-movement interval, so imitation must not charge
    # the 0.4--0.7 m/s lateral step or pull toward V7's tilted frame-0 torso.  After release the
    # arm-velocity prior is measured relative to the live torso anchor, removing V7's 10--12 cm
    # common-mode root drop while retaining the demonstrated arm swing.
    motion_global_anchor_ori = RewTerm(
        func=mdp.motion_global_anchor_ori_windup_only,
        weight=0.5,
        params={
            "command_name": "motion",
            "racket_command_name": "racket_target",
            "std": 0.4,
            "min_time_to_strike": 0.25,
        },
    )
    motion_body_pos = RewTerm(
        func=mdp.motion_body_pos_swing_only,
        weight=1.0,
        params={
            "command_name": "motion",
            "std": 0.3,
            "body_names": A3_UPPER_STYLE_POSITION_TRACKED,
        },
    )
    motion_body_ori = RewTerm(
        func=mdp.motion_body_ori_swing_only,
        weight=1.0,
        params={"command_name": "motion", "std": 0.4, "body_names": A3_UPPER_STYLE_TRACKED},
    )
    motion_body_lin_vel = RewTerm(
        func=mdp.motion_body_lin_vel_anchor_relative_swing_only,
        weight=1.0,
        params={"command_name": "motion", "std": 1.0, "body_names": A3_UPPER_STYLE_TRACKED},
    )
    motion_body_ang_vel = RewTerm(
        func=mdp.motion_body_ang_vel_anchor_relative_swing_only,
        weight=1.0,
        params={"command_name": "motion", "std": 3.14, "body_names": A3_UPPER_STYLE_TRACKED},
    )

    # V7 starts near the 1.0684 m deploy stand but ends 10--12 cm lower.  This zero-income,
    # root-z-only debt prevents that reference artifact from becoming the repeated rally stance
    # without importing any demonstrated leg pose or constraining commanded lateral motion.
    rally_ready_root_height_debt = RewTerm(
        func=mdp.rally_ready_root_height_debt,
        weight=-0.20,
        params={
            "command_name": "racket_target",
            "motion_command_name": "motion",
            "min_height": 1.02,
            "std": 0.05,
            "ready_t_lo": -0.10,
            "ready_t_hi": 1.10,
            "post_t_lo": 0.20,
            "post_t_hi": 1.55,
        },
    )

    # The head outputs are intentionally unused, but still have network output neurons.  Charge
    # their raw values so they converge toward zero instead of becoming an unobserved action rail.
    passive_head_raw_action = RewTerm(
        func=mdp.passive_head_raw_action_penalty,
        weight=-0.05,
        params={
            "action_name": "joint_pos",
            "std": 1.0,
            "loss_type": "huber",
            "huber_delta": 1.0,
        },
    )

    # Failed-run repair: the precise positive Gaussian is gradient-dead at the observed
    # 66-degree forehand normal error. This strike-only smooth-L1 debt is zero once aligned.
    racket_normal_alignment_debt = RewTerm(
        func=mdp.racket_normal_alignment_debt,
        weight=-2.0,
        params={"command_name": "racket_target", "margin": 0.10, "std": 0.35},
    )

    # Fresh V3 started 0.5--1.0 m outside the 0.15-m positive Gaussian, so its nominal weight 14
    # supplied effectively zero position gradient. This strike-only Huber tail reaches the basin;
    # the V7 wrist-position prior above provides dense phase/style guidance between strikes.
    racket_position_alignment_debt = RewTerm(
        func=mdp.racket_position_alignment_debt,
        weight=-2.0,
        params={"command_name": "racket_target", "margin": 0.075, "std": 0.35},
    )

    # Charge only absolute q_des requests discarded by the deploy-faithful soft clamp. Legal
    # lateral steps and arm targets cost zero; one exploding joint cannot hide in a 29-DOF mean.
    rally_joint_qdes_saturation = RewTerm(
        func=mdp.rally_joint_qdes_saturation_penalty,
        weight=-0.20,
        params={
            "command_name": "racket_target",
            "action_name": "joint_pos",
            "std": 0.20,
            "max_blend": 0.25,
        },
    )

    # Preserve the first 0.10 s of follow-through, then prevent the slow-tip shortcut that made
    # 84% of forehands terminate during recovery despite apparently quiet translational metrics.
    post_swing_tilt_debt = RewTerm(
        func=mdp.post_swing_tilt_debt,
        weight=-0.50,
        params={
            "command_name": "racket_target",
            "margin": 0.10,
            "std": 0.20,
            "t_lo": 0.10,
            "t_hi": 1.55,
        },
    )


@configclass
class HOPEPingPongHitterPureRallyFinalV3AgibotA3EnvCfg(
    HOPEPingPongHitterPureRallyFinalV2AgibotA3EnvCfg
):
    """Continuous rally task for a common-ready, ready->strike->recover motion pair.

    This class deliberately keeps the successful FinalV2 strike/x/front-facing stack, but
    removes its old-video left-arm clearance term and four structural mismatches: every-wrap
    hard steps, saturating settle tails, active neck outputs, and reference-relative recovery
    termination.
    """

    actions: HOPEHitterPureRallyFinalV3ActionsCfg = HOPEHitterPureRallyFinalV3ActionsCfg()
    observations: HOPEObservationsHitterPureRallyFinalV3Cfg = (
        HOPEObservationsHitterPureRallyFinalV3Cfg()
    )
    rewards: HOPEHitterPureRallyFinalV3RewardsCfg = HOPEHitterPureRallyFinalV3RewardsCfg()
    terminations: HOPEHitterPureRallyFinalV3TerminationsCfg = (
        HOPEHitterPureRallyFinalV3TerminationsCfg()
    )

    def __post_init__(self):
        super().__post_init__()

        # V3-only removal (user decision, 2026-07-11): the approved V7 backhand keeps at least
        # 0.57 m blade-to-left-arm clearance, so the surgical guard for the old close-hands clip
        # must not shape this training task.  Keep FinalV1/V2 behavior frozen for reproducibility.
        self.rewards.backhand_left_arm_clearance = None
        self.rewards.backhand_left_hand_clearance = None

        # Seven or more physical transitions per episode with a ~2.1 s cyclic clip.  The
        # externally sampled ready clamp is not policy-controlled and pays no positive hold
        # income; it only exposes the same station-tracking state that the runner owns.
        self.episode_length_s = 20.0
        self.commands.motion.hold_steps_range = (40, 60)  # 0.8--1.2 s at 50 Hz
        self.commands.motion.stand_start_min_hold = 40
        self.commands.motion.stand_start_prob = 0.50
        self.commands.motion.stand_start_yaw_range = (0.0, 0.0)
        self.commands.motion.post_swing_start_prob = 0.0
        self.commands.motion.clip_switch_prob = 0.0

        C = self.commands.racket_target
        C.hold_until_settled = False
        C.midswing_resample_prob = 0.0
        C.base_target_x_range = (0.0, 0.0)
        C.base_target_y_range = (-0.35, 0.35)
        # v12 striking faces (receipt hitter_rally_v12_approved_receipt.json): back to the
        # hopex-lineage convention fh RED(+Y) / bh BLACK(−Y) — the v11 inversion is RETIRED.
        # Matches this task's own YAML ([1.0, -1.0]); the stale (-1.0, 1.0) default mis-scored
        # paddle faces in bypass-path evals/exports that skip train.py's YAML overlay.
        C.mount_normal_sign_per_clip = (1.0, -1.0)
        C.station_y_step_range = (0.20, 0.35)
        C.station_y_same_prob = 0.15
        C.station_y_small_step_prob = 0.25
        C.station_y_small_step_range = (0.03, 0.12)
        C.station_side_explicit = True
        C.ready_monitor_step_range = (0.0, 0.35)
        C.ready_monitor_x_thresh = 0.10
        C.ready_monitor_y_thresh = 0.10
        C.ready_monitor_speed_thresh = 0.20
        C.ready_monitor_dwell_s = 0.12
        C.ready_monitor_heading_thresh_rad = 0.261799  # 15 deg, inside runner's 20 deg gate
        C.metrics_pre_settle_t_max = 1.10
        C.metrics_post_settle_t_lo = 0.20
        C.metrics_post_settle_t_hi = 1.55
        C.metrics_clearance_t_pre = 0.70
        C.metrics_clearance_t_post = 0.20
        C.metrics_ready_heading_t_lo = 0.45
        C.metrics_ready_heading_t_hi = 1.10
        C.metrics_post_heading_t_lo = 0.20
        C.metrics_post_heading_t_hi = 1.55

        # User-approved V7 timing and station reach (explicit video contact frames FH=38, BH=37
        # over 104 frames). Rounded x/y centers match measured A3 blade reach. Height and velocity
        # cover the venue-fitted real-contact distribution and HITTER center-return inverse, not the
        # synthetic Gate3 side-line aims. Incoming height is independent of forehand/backhand.
        C.strike_phase = 0.36893204
        C.strike_phase_per_clip = (0.36893204, 0.35922330)
        C.racket_pos_range_per_clip = (
            ((0.70, 0.70), (-0.04, -0.04), (0.98, 1.26)),
            ((0.70, 0.70), (0.10, 0.10), (0.98, 1.26)),
        )
        C.racket_vel_range_per_clip = (
            ((0.25, 1.35), (0.00, 0.35), (0.10, 1.00)),
            ((0.25, 1.35), (-0.15, 0.12), (0.10, 1.00)),
        )

        # Non-saturating, zero-inside-deadband debts.  Correct early lateral motion and a
        # quiet arrived state both cost zero; there is no positive settle/hold income to farm.
        self.rewards.strike_x_drift.params["huber_tail"] = True
        self.rewards.strike_x_drift.params["t_pre"] = 1.10
        self.rewards.strike_x_drift.params["t_post"] = 1.55
        self.rewards.strike_x_velocity.params["huber_tail"] = True
        self.rewards.strike_x_velocity.params["t_pre"] = 1.10
        self.rewards.strike_x_velocity.params["t_post"] = 1.55

        self.rewards.pre_strike_station_settle.weight = -0.25
        self.rewards.pre_strike_station_settle.params.update(
            {"std": 0.30, "t_max": 1.10, "t_min": 0.12,
             "velocity_margin": 0.05, "debt_huber": True}
        )
        self.rewards.post_swing_base_quiet.params.update(
            {"margin": 0.08, "huber_tail": True, "t_hi": 1.55}
        )
        self.rewards.post_swing_leg_quiet.params.update(
            {"margin": 0.30, "huber_tail": True, "t_hi": 1.55}
        )
        self.rewards.settle_foot_slip.weight = -0.04
        self.rewards.settle_foot_slip.params.update(
            {"margin": 0.03, "huber_tail": True,
             "pre_t_max": 1.10, "post_t_hi": 1.55}
        )
        self.rewards.rally_heading_debt.params.update(
            {"huber_tail": True, "ready_t_lo": 0.45, "ready_t_hi": 1.10,
             "post_t_lo": 0.20, "post_t_hi": 1.55}
        )
        self.rewards.rally_foot_orientation.params.update(
            {"t_pre": 1.10, "t_post": 1.55, "default_during_hold": True}
        )
        self.rewards.rally_ankle_qdes_saturation.weight = -0.30
        self.rewards.rally_ankle_qdes_saturation.params.update(
            {"t_pre": 1.10, "t_post": 1.55}
        )
        # Keep the failed stacks explicitly absent.
        self.rewards.hold_ready.weight = 0.0
        self.rewards.hold_heading.weight = 0.0
        self.rewards.lower_body_plant_imitation.weight = 0.0
        self.rewards.foot_orientation.weight = 0.0


@configclass
class HOPEPingPongHitterPureRallyV9RewardsCfg(HOPEPingPongHitterPureRallyV8RewardsCfg):
    """V8 strike recipe plus debts for the failures measured on model_8500 in AGI Gate 3."""

    # Gate3 showed a +7 cm base-x offset accumulated during windup. Charge x error and only
    # outward vx; corrective backward motion remains free. The companion stand-start offset
    # below makes 3--10 cm forward errors an explicit training state.
    windup_x_recovery = RewTerm(
        func=mdp.windup_x_recovery_debt,
        weight=-0.40,
        params={
            "command_name": "racket_target",
            "x_margin": 0.025,
            "x_std": 0.05,
            "vx_margin": 0.05,
            "vx_std": 0.20,
            "position_blend": 0.60,
            "t_hi": 0.96,
        },
    )

    # Match the Gate3 pre-strike p90 failure directly. This desired-velocity debt permits the
    # commanded y approach while far away and converges to quiet at the station.
    pre_strike_station_settle = RewTerm(
        func=mdp.pre_strike_station_settle,
        weight=-0.25,
        params={
            "command_name": "racket_target",
            "v_gain": 2.0,
            "v_max": 0.8,
            "std": 0.30,
            "t_min": 0.12,
            "t_max": 0.96,
            "velocity_margin": 0.05,
            "debt_huber": True,
        },
    )

    # Gate3 observed 10--14 degree recovery heading and 0.29--0.47 rad/s yaw rate. The kernel
    # charges outward yaw rate but leaves a corrective turn free until the heading is recovered.
    rally_heading_debt = RewTerm(
        func=mdp.rally_heading_debt,
        weight=-0.30,
        params={
            "command_name": "racket_target",
            "yaw_margin": 0.12,
            "yaw_std": 0.25,
            "rate_margin": 0.15,
            "rate_std": 0.50,
            "heading_blend": 0.50,
            "ready_t_lo": 0.45,
            "ready_t_hi": 1.00,
            "post_t_lo": 0.20,
            "post_t_hi": 1.20,
            "huber_tail": True,
        },
    )

    # The runner physically holds the head while model_8500 rails its unused raw head outputs.
    passive_head_raw_action = RewTerm(
        func=mdp.passive_head_raw_action_penalty,
        weight=-0.05,
        params={
            "action_name": "joint_pos",
            "std": 1.0,
            "loss_type": "huber",
            "huber_delta": 1.0,
        },
    )

    # The corrected report measured nonzero runner clamp telemetry on 30/111 status samples.
    # Charge every active joint's discarded pre-clamp q_des request, not an arbitrary raw-action cap.
    rally_joint_qdes_saturation = RewTerm(
        func=mdp.rally_joint_qdes_saturation_penalty,
        weight=-0.20,
        params={
            "command_name": "racket_target",
            "action_name": "joint_pos",
            "std": 0.20,
            "max_blend": 0.25,
        },
    )


@configclass
class HOPEPingPongHitterPureRallyV9TerminationsCfg(
    HOPEPingPongHitterPureRallyV8TerminationsCfg
):
    """Keep physical fall/foot guards; wrist reference error must remain recoverable.

    A direct per-term probe on model_8500 measured ee_wrist_pos on 32.27% of deterministic
    termination events, and a fresh-policy smoke test produced 1.67-step episodes with the wrist
    guard accounting for virtually every reset. The task itself requests wrist/racket heights away
    from the reference clip, so reference-z deviation is not a physical failure condition.
    """

    ee_wrist_pos = None


@configclass
class HOPEPingPongHitterPureRallyV9AgibotA3EnvCfg(
    HOPEPingPongHitterPureRallyV8AgibotA3EnvCfg
):
    """Causal RallyV8 repair for AGI x-recovery, pre-strike speed, yaw and clamp failures.

    V8 remains frozen. V9 keeps its v13 motion/velocity/station contract and adds deploy-observed
    state coverage plus zero-inside-deadband debts. The 110-D wire shape is unchanged; head actions
    are held at the same defaults as the C++ runner and applied-last-action feedback reflects that.
    """

    actions: HOPEHitterPureRallyFinalV3ActionsCfg = HOPEHitterPureRallyFinalV3ActionsCfg()
    observations: HOPEObservationsHitterPureRallyFinalV3Cfg = (
        HOPEObservationsHitterPureRallyFinalV3Cfg()
    )
    rewards: HOPEPingPongHitterPureRallyV9RewardsCfg = (
        HOPEPingPongHitterPureRallyV9RewardsCfg()
    )
    terminations: HOPEPingPongHitterPureRallyV9TerminationsCfg = (
        HOPEPingPongHitterPureRallyV9TerminationsCfg()
    )

    def __post_init__(self):
        super().__post_init__()
        M = self.commands.motion
        C = self.commands.racket_target

        # A quarter of episode resets already use DEFAULT STAND in V8. Offset those starts
        # 3--10 cm forward while leaving the commanded station fixed, so recovery is trained.
        M.stand_start_x_range = (0.03, 0.10)

        # Remove the degenerate reach-x box. Together with the forward-start distribution this
        # places the 0.55--0.59 m deployed forehand reach inside training support while retaining
        # the v13 receipt nominal as the box centre.
        C.racket_pos_range_per_clip = (
            ((0.60, 0.70), (-0.48, -0.40), (0.85, 1.30)),
            ((0.45, 0.55), (-0.13, -0.05), (0.85, 1.30)),
        )

        # Align Isaac telemetry windows with the Gate3 report instead of comparing different
        # phases and then treating the disagreement as model ranking signal.
        C.metrics_pre_settle_t_max = 0.96
        C.metrics_post_settle_t_lo = 0.20
        C.metrics_post_settle_t_hi = 1.20
        C.metrics_ready_heading_t_lo = 0.45
        C.metrics_ready_heading_t_hi = 1.00
        C.metrics_post_heading_t_lo = 0.20
        C.metrics_post_heading_t_hi = 1.20


@configclass
class HOPEPingPongHitterPureRallyV10RewardsCfg(HOPEPingPongHitterPureRallyV9RewardsCfg):
    """RallyV9 strike contract with the Gate3 model_10000 recovery repairs."""

    # V9's 4 cm strike-x deadband made the measured +3.5 cm lunge free. Keep the position
    # debt through contact, but do not strengthen the separate velocity term: legitimate strike
    # weight transfer must remain available.
    strike_x_drift = RewTerm(
        func=mdp.strike_x_drift_penalty,
        weight=-0.75,
        params={
            "command_name": "racket_target",
            "margin": 0.02,
            "std": 0.05,
            "t_pre": 1.10,
            "t_post": 1.55,
            "huber_tail": True,
        },
    )

    # V9 used a strict ``tts < 0.96`` gate while the backhand hold is pinned at exactly 0.96 s.
    # Move both approach debts beyond the longest windup so they are active during that hold.
    windup_x_recovery = RewTerm(
        func=mdp.windup_x_recovery_debt,
        weight=-0.40,
        params={
            "command_name": "racket_target",
            "x_margin": 0.02,
            "x_std": 0.05,
            "vx_margin": 0.05,
            "vx_std": 0.20,
            "position_blend": 0.60,
            "t_hi": 1.10,
        },
    )
    pre_strike_station_settle = RewTerm(
        func=mdp.pre_strike_station_settle,
        weight=-0.25,
        params={
            "command_name": "racket_target",
            "v_gain": 2.0,
            "v_max": 0.8,
            "std": 0.30,
            "t_min": 0.12,
            "t_max": 1.10,
            "velocity_margin": 0.05,
            "debt_huber": True,
        },
    )

    # Keep the V9 heading deadband/window, but make angular velocity follow a bounded first-order
    # target. This closes the "fast corrective turn is free" loophole while retaining recovery.
    rally_heading_debt = RewTerm(
        func=mdp.rally_heading_settle_debt,
        weight=-0.30,
        params={
            "command_name": "racket_target",
            "yaw_margin": 0.12,
            "yaw_std": 0.25,
            "rate_margin": 0.05,
            "rate_std": 0.25,
            "heading_blend": 0.35,
            "yaw_rate_gain": 2.0,
            "yaw_rate_max": 0.15,
            "ready_t_lo": 0.45,
            "ready_t_hi": 1.00,
            "post_t_lo": 0.20,
            "post_t_hi": 1.20,
            "huber_tail": True,
        },
    )

    # Directly constrain the non-racket wrist in joint space. The old z-only termination remains
    # disabled because it cannot observe wrist rotation and valid right-hand reaches trigger it.
    left_wrist_reference_debt = RewTerm(
        func=mdp.left_wrist_reference_debt,
        weight=-0.20,
        params={
            "command_name": "racket_target",
            "motion_command_name": "motion",
            "margin": 0.10,
            "std": 0.25,
            "max_blend": 0.50,
            "asset_cfg": SceneEntityCfg(
                "robot",
                joint_names=[
                    "left_wrist_roll_joint",
                    "left_wrist_pitch_joint",
                    "left_wrist_yaw_joint",
                ],
                preserve_order=True,
            ),
        },
    )

    # The A3 elbow extends as q increases and approaches a straight arm near 1.57 rad. Charge
    # only the over-extension tail around contact; do not lock the task wrist/arm to the clip.
    right_elbow_extension_debt = RewTerm(
        func=mdp.right_elbow_extension_debt,
        weight=-0.20,
        params={
            "command_name": "racket_target",
            "extension_start": 1.30,
            "std": 0.15,
            "t_pre": 0.25,
            "t_post": 0.10,
            "asset_cfg": SceneEntityCfg(
                "robot", joint_names=["right_elbow_joint"], preserve_order=True
            ),
        },
    )

    # V9 reduced clamp telemetry from 30/111 to 8/111 status samples, but one waist joint could
    # still hide in the 29-DOF mean. Increase only the max component; legal q_des remains free.
    rally_joint_qdes_saturation = RewTerm(
        func=mdp.rally_joint_qdes_saturation_penalty,
        weight=-0.20,
        params={
            "command_name": "racket_target",
            "action_name": "joint_pos",
            "std": 0.20,
            "max_blend": 0.50,
        },
    )


@configclass
class HOPEPingPongHitterPureRallyV10TerminationsCfg(
    HOPEPingPongHitterPureRallyV9TerminationsCfg
):
    """V10 retains V9's feet/physical guards and keeps the z-only wrist guard disabled."""


@configclass
class HOPEPingPongHitterPureRallyV10AgibotA3EnvCfg(
    HOPEPingPongHitterPureRallyV9AgibotA3EnvCfg
):
    """RallyV10: left-wrist discipline plus smooth station/heading recovery."""

    rewards: HOPEPingPongHitterPureRallyV10RewardsCfg = (
        HOPEPingPongHitterPureRallyV10RewardsCfg()
    )
    terminations: HOPEPingPongHitterPureRallyV10TerminationsCfg = (
        HOPEPingPongHitterPureRallyV10TerminationsCfg()
    )

    def __post_init__(self):
        super().__post_init__()
        # V10 restores the strict single-plane/y-only contract. 0.58 m rounds the approved v13
        # receipt contact midpoint; using it for both clips makes deploy station_x
        # independent of side instead of commanding an untrained forehand/backhand x step.
        self.commands.racket_target.racket_pos_range_per_clip = (
            ((0.58, 0.58), (-0.48, -0.40), (0.85, 1.30)),
            ((0.58, 0.58), (-0.13, -0.05), (0.85, 1.30)),
        )
        # Expand from the demonstrated v13 face-cone core to the central-90% live-planner
        # envelope. The ramp keeps a fresh policy on the feasible core while it bootstraps.
        self.commands.racket_target.racket_vel_start_range_per_clip = (
            ((1.24, 2.24), (-0.31, 0.69), (0.66, 1.66)),
            ((1.60, 2.60), (-0.66, 0.34), (0.00, 0.54)),
        )
        self.commands.racket_target.racket_vel_planner_range_per_clip = (
            ((1.57, 2.55), (0.10, 0.52), (0.41, 1.35)),
            ((1.55, 2.52), (-0.18, 0.29), (0.40, 1.32)),
        )
        self.commands.racket_target.racket_vel_range_per_clip = (
            ((1.24, 2.60), (-0.31, 0.69), (0.40, 1.66)),
            ((1.50, 2.60), (-0.66, 0.40), (0.00, 1.35)),
        )
        self.commands.racket_target.racket_vel_planner_mix_prob = 0.75
        self.commands.racket_target.racket_vel_range_ramp_steps = 96000
        # Keep deterministic telemetry and the deploy report on the exact V10 approach window.
        self.commands.racket_target.metrics_pre_settle_t_max = 1.10


@configclass
class HOPEPingPongHitterPureRallyV11RewardsCfg(HOPEPingPongHitterPureRallyV10RewardsCfg):
    """V10 strike contract plus phase-aware lower-body stance and settling supervision.

    Full lower-body clip imitation remains off: it previously imported the retargeted crouch and
    forward step into the ready stance.  These V11 terms activate only after station arrival and
    heading recovery, use the safe default only for six foot-orientation joints, and otherwise
    constrain geometry/velocity rather than prescribing a complete leg pose.
    """

    ready_deadline = RewTerm(
        func=mdp.rally_ready_deadline_debt,
        weight=-0.35,
        params={
            "command_name": "racket_target",
            "motion_command_name": "motion",
            "x_margin": 0.10,
            "y_margin": 0.10,
            "position_std": 0.10,
            "speed_margin": 0.20,
            "speed_std": 0.20,
            "speed_blend": 0.60,
            "final_window_s": 0.12,
            "target_step_class": 3,
        },
    )

    ready_stance_width = RewTerm(
        func=mdp.rally_ready_stance_width_debt,
        weight=-0.30,
        params={
            "command_name": "racket_target",
            "motion_command_name": "motion",
            "asset_cfg": SceneEntityCfg(
                "robot", body_names=list(A3_FEET_BODIES), preserve_order=True
            ),
            "lo": 0.25,
            "hi": 0.35,
            "std": 0.05,
            "station_reach": 0.10,
            "heading_gate": 0.15,
            "speed_gate": 0.20,
        },
    )
    ready_foot_alignment = RewTerm(
        func=mdp.rally_ready_foot_alignment_debt,
        weight=-0.15,
        params={
            "command_name": "racket_target",
            "motion_command_name": "motion",
            "asset_cfg": SceneEntityCfg(
                "robot",
                joint_names=[
                    ".*_hip_yaw_joint",
                    ".*_hip_roll_joint",
                    ".*_ankle_roll_joint",
                ],
                preserve_order=True,
            ),
            "margin": 0.12,
            "std": 0.25,
            "max_blend": 0.50,
            "station_reach": 0.10,
            "heading_gate": 0.15,
            "speed_gate": 0.20,
        },
    )
    ready_leg_settle = RewTerm(
        func=mdp.rally_ready_leg_settle_debt,
        weight=-0.05,
        params={
            "command_name": "racket_target",
            "motion_command_name": "motion",
            "asset_cfg": SceneEntityCfg(
                "robot",
                joint_names=[
                    ".*_hip_pitch_joint",
                    ".*_hip_roll_joint",
                    ".*_hip_yaw_joint",
                    ".*_knee_joint",
                    ".*_ankle_pitch_joint",
                    ".*_ankle_roll_joint",
                ],
                preserve_order=True,
            ),
            "margin": 0.30,
            "std": 0.80,
            "station_reach": 0.10,
            "heading_gate": 0.15,
        },
    )

    # V10 still sent ankle-roll requests outside the safe envelope on a material fraction of
    # training states. Strengthen the existing pre-clamp debt without adding an action projection.
    rally_ankle_qdes_saturation = RewTerm(
        func=mdp.rally_ankle_qdes_saturation_penalty,
        weight=-0.30,
        params={
            "command_name": "racket_target",
            "action_name": "joint_pos",
            "asset_cfg": SceneEntityCfg(
                "robot", joint_names=[".*_ankle_roll_joint"], preserve_order=True
            ),
            "safe_abs": 0.20,
            "std": 0.10,
            "t_pre": 1.40,
            "t_post": 1.80,
        },
    )

    # V10's recovery-leg speed rose late in training. Keep the same pose-free, post-strike-only
    # channel and double its deliberately small weight; approach steps and contact remain untaxed.
    post_strike_leg_quiet = RewTerm(
        func=mdp.post_strike_leg_quiet,
        weight=-0.04,
        params={
            "command_name": "racket_target",
            "t_hi": 1.20,
            "asset_cfg": SceneEntityCfg(
                "robot",
                joint_names=[
                    ".*_hip_pitch_joint",
                    ".*_hip_roll_joint",
                    ".*_hip_yaw_joint",
                    ".*_knee_joint",
                    ".*_ankle_pitch_joint",
                    ".*_ankle_roll_joint",
                ],
            ),
        },
    )


@configclass
class HOPEPingPongHitterPureRallyV11TerminationsCfg(
    HOPEPingPongHitterPureRallyV10TerminationsCfg
):
    """V11 adds no hard readiness, pose, or release gate."""


@configclass
class HOPEPingPongHitterPureRallyV11AgibotA3EnvCfg(
    HOPEPingPongHitterPureRallyV10AgibotA3EnvCfg
):
    """Fresh-train V11: V10 questions and strike mechanics with a recoverable ready stance."""

    rewards: HOPEPingPongHitterPureRallyV11RewardsCfg = (
        HOPEPingPongHitterPureRallyV11RewardsCfg()
    )
    terminations: HOPEPingPongHitterPureRallyV11TerminationsCfg = (
        HOPEPingPongHitterPureRallyV11TerminationsCfg()
    )

    def __post_init__(self):
        super().__post_init__()
        M = self.commands.motion
        C = self.commands.racket_target

        # model_10500 repeatedly completed the +y 19--24 cm station change one serve late.
        # Most ordinary holds now match the runner's roughly one-second movement budget; the
        # inherited 15% long-hold tail remains for active-idle stability coverage.
        M.hold_steps_range = (45, 60)  # 0.9--1.2 s at 50 Hz
        M.stand_start_min_hold = 45
        C.station_y_positive_main_prob = 0.80
        C.station_y_positive_main_step_range = (0.19, 0.24)

        # The existing desired-velocity profile remains the early move->brake teacher. The new
        # ready_deadline debt supervises the last 0.12 s without blocking swing release.
        self.rewards.pre_strike_station_settle.weight = -0.35

        # Fresh Gate3: idle left-wrist ranges 0.47--0.55 rad, post-heading max 16.4 deg, and the
        # waist-pitch request remained the worst clamp. Tighten the existing soft debts; do not add
        # action projection, pose termination, or a train-time READY admission gate.
        self.rewards.left_wrist_reference_debt.weight = -0.30
        self.rewards.left_wrist_reference_debt.params.update(
            {"margin": 0.08, "max_blend": 0.75}
        )
        self.rewards.rally_heading_debt.weight = -0.40
        self.rewards.rally_heading_debt.params.update(
            {"yaw_margin": 0.10, "heading_blend": 0.50, "post_t_hi": 1.55}
        )
        # Keep deterministic evaluation and the Gate3 report on the same post-heading window as
        # the V11 reward.  Without this, W&B/eval stopped at the inherited V9/V10 1.20 s window
        # while the policy continued paying heading debt through 1.55 s.
        C.metrics_post_heading_t_hi = 1.55
        self.rewards.rally_joint_qdes_saturation.weight = -0.25
        self.rewards.rally_joint_qdes_saturation.params["max_blend"] = 1.0


@configclass
class HOPEPingPongHitterPureRallyV12RewardsCfg(
    HOPEPingPongHitterPureRallyV11RewardsCfg
):
    """V11 plus the three residual strict-Gate3 margin repairs.

    These debts are deliberately additive and narrow: V11's broad station/recovery, wrist
    reference and whole-body pre-clamp terms remain intact.  V12 only exposes the exact channels
    that model_11800 still failed in MuJoCo: contact x margin, hold-wrist overshoot and waist-only
    q_des saturation hidden by the whole-body maximum.
    """

    strike_x_gate_margin = RewTerm(
        func=mdp.rally_strike_x_margin_debt,
        weight=-1.00,
        params={
            "command_name": "racket_target",
            "margin": 0.015,
            "std": 0.020,
            "half_window_s": 0.040,
        },
    )

    idle_left_wrist_debt = RewTerm(
        func=mdp.rally_idle_left_wrist_debt,
        weight=-0.40,
        params={
            "command_name": "racket_target",
            "motion_command_name": "motion",
            "position_margin": 0.020,
            "position_std": 0.120,
            "velocity_margin": 0.050,
            "velocity_std": 0.250,
            "velocity_blend": 0.250,
            "max_blend": 0.750,
            "asset_cfg": SceneEntityCfg(
                "robot",
                joint_names=[
                    "left_wrist_roll_joint",
                    "left_wrist_pitch_joint",
                    "left_wrist_yaw_joint",
                ],
                preserve_order=True,
            ),
        },
    )

    waist_qdes_saturation = RewTerm(
        func=mdp.rally_waist_qdes_saturation_penalty,
        weight=-0.50,
        params={
            "command_name": "racket_target",
            "action_name": "joint_pos",
            "std": 0.100,
            "max_blend": 1.000,
            "asset_cfg": SceneEntityCfg(
                "robot",
                joint_names=["waist_roll_joint", "waist_pitch_joint"],
                preserve_order=True,
            ),
        },
    )


@configclass
class HOPEPingPongHitterPureRallyV12TerminationsCfg(
    HOPEPingPongHitterPureRallyV11TerminationsCfg
):
    """V12 remains soft-supervised and adds no termination or release gate."""


@configclass
class HOPEPingPongHitterPureRallyV12AgibotA3EnvCfg(
    HOPEPingPongHitterPureRallyV11AgibotA3EnvCfg
):
    """Fresh-train V12 with the V11 state/question distribution unchanged."""

    rewards: HOPEPingPongHitterPureRallyV12RewardsCfg = (
        HOPEPingPongHitterPureRallyV12RewardsCfg()
    )
    terminations: HOPEPingPongHitterPureRallyV12TerminationsCfg = (
        HOPEPingPongHitterPureRallyV12TerminationsCfg()
    )


@configclass
class HOPEPingPongHitterPureRallyV13RewardsCfg(
    HOPEPingPongHitterPureRallyV12RewardsCfg
):
    """V12 strike surface plus explicit post-swing balance and recovery supervision."""

    # Replace the V11 single-maximum term. The worst four normalized channels all receive gradient;
    # hard/safe limits are read from the official A3 asset for all 31 action columns.
    rally_joint_qdes_saturation = RewTerm(
        func=mdp.rally_all_joint_qdes_barrier,
        weight=-0.45,
        params={
            "command_name": "racket_target",
            "action_name": "joint_pos",
            "safe_margin_fraction": 0.05,
            "std_fraction": 0.03,
            "topk": 4,
            "topk_blend": 0.75,
        },
    )
    # The all-joint barrier supersedes V12's waist-only patch; keeping both would double-charge
    # waist motion while ankle/shoulder channels receive only one term.
    waist_qdes_saturation = None

    # The V12 Gate3 elbow failures were all forehand. Leave backhand geometry untouched and start
    # the forehand barrier below the 1.35 rad report limit to create selection margin.
    right_elbow_extension_debt = RewTerm(
        func=mdp.right_elbow_extension_debt,
        weight=-0.35,
        params={
            "command_name": "racket_target",
            "extension_start": 1.25,
            "std": 0.12,
            "t_pre": 0.30,
            "t_post": 0.12,
            "forehand_only": True,
            "asset_cfg": SceneEntityCfg(
                "robot", joint_names=["right_elbow_joint"], preserve_order=True
            ),
        },
    )

    # Contact-x misses were backhand-only. Preserve V12's term and reduce its forehand share instead
    # of globally increasing a pressure that can drive the forehand elbow straight.
    strike_x_gate_margin = RewTerm(
        func=mdp.rally_strike_x_margin_debt,
        weight=-1.00,
        params={
            "command_name": "racket_target",
            "margin": 0.015,
            "std": 0.020,
            "half_window_s": 0.040,
            "forehand_scale": 0.50,
            "backhand_scale": 1.25,
        },
    )

    post_swing_xlock = RewTerm(
        func=mdp.rally_post_swing_xlock_debt,
        weight=-0.35,
        params={
            "command_name": "racket_target",
            "margin": 0.030,
            "std": 0.040,
            "t_lo": 0.10,
            "t_hi": 1.55,
            "forehand_scale": 1.00,
            "backhand_scale": 1.25,
        },
    )

    # Backhand owns all measured post-heading failures. Keep the same first-order recovery law and
    # windows, changing only its side multiplier.
    rally_heading_debt = RewTerm(
        func=mdp.rally_heading_settle_debt,
        weight=-0.40,
        params={
            "command_name": "racket_target",
            "yaw_margin": 0.10,
            "yaw_std": 0.25,
            "rate_margin": 0.05,
            "rate_std": 0.25,
            "heading_blend": 0.50,
            "yaw_rate_gain": 2.0,
            "yaw_rate_max": 0.15,
            "ready_t_lo": 0.45,
            "ready_t_hi": 1.00,
            "post_t_lo": 0.20,
            "post_t_hi": 1.55,
            "huber_tail": True,
            "forehand_scale": 1.00,
            "backhand_scale": 1.50,
        },
    )

    # V11 supervised only the +y main subclass (3). V12 serve 5 exposed the unsupervised small-step
    # path, so V13 applies the same fixed-clock final-window debt to every nonzero transition class.
    ready_deadline = RewTerm(
        func=mdp.rally_ready_deadline_debt,
        weight=-0.35,
        params={
            "command_name": "racket_target",
            "motion_command_name": "motion",
            "x_margin": 0.10,
            "y_margin": 0.10,
            "position_std": 0.10,
            "speed_margin": 0.20,
            "speed_std": 0.20,
            "speed_blend": 0.60,
            "final_window_s": 0.12,
            "target_step_class": 3,
            "target_step_classes": (1, 2, 3),
        },
    )

    # Preserve a 0.10 s tilt and 0.20 s dynamic follow-through grace, then directly supervise the
    # physical channels implicated by the V10 hardware fall. These are zero-income debts: they do
    # not compete with the 14/14/5 strike rewards when the robot is already calm and upright.
    post_swing_base_quiet = RewTerm(
        func=mdp.post_swing_base_quiet,
        weight=-0.25,
        params={
            "command_name": "racket_target",
            "std": 0.25,
            "margin": 0.08,
            "t_lo": 0.20,
            "t_hi": 1.55,
            "huber_tail": True,
        },
    )

    post_swing_leg_quiet = RewTerm(
        func=mdp.post_swing_leg_quiet,
        weight=-0.08,
        params={
            "command_name": "racket_target",
            "std": 0.80,
            "margin": 0.30,
            "t_lo": 0.20,
            "t_hi": 1.55,
            "huber_tail": True,
            "asset_cfg": SceneEntityCfg(
                "robot",
                joint_names=[
                    ".*_hip_pitch_joint",
                    ".*_hip_roll_joint",
                    ".*_hip_yaw_joint",
                    ".*_knee_joint",
                    ".*_ankle_pitch_joint",
                    ".*_ankle_roll_joint",
                ],
                preserve_order=True,
            ),
        },
    )

    settle_foot_slip = RewTerm(
        func=mdp.settle_foot_slip_penalty,
        weight=-0.10,
        params={
            "command_name": "racket_target",
            "asset_cfg": SceneEntityCfg(
                "robot", body_names=A3_FEET_BODIES, preserve_order=True
            ),
            "sensor_cfg": SceneEntityCfg(
                "contact_forces", body_names=A3_FEET_BODIES, preserve_order=True
            ),
            "std": 0.12,
            "margin": 0.03,
            "station_reach": 0.15,
            "force_threshold": 10.0,
            "pre_t_max": 1.10,
            "strike_t_post": 0.10,
            "post_t_lo": 0.20,
            "post_t_hi": 1.55,
            "huber_tail": True,
        },
    )

    rally_ready_root_height_debt = RewTerm(
        func=mdp.rally_ready_root_height_debt,
        weight=-0.20,
        params={
            "command_name": "racket_target",
            "motion_command_name": "motion",
            "min_height": 0.98,
            "std": 0.05,
            "ready_t_lo": -0.10,
            "ready_t_hi": 1.10,
            "post_t_lo": 0.20,
            "post_t_hi": 1.55,
        },
    )

    post_swing_tilt_debt = RewTerm(
        func=mdp.post_swing_tilt_debt,
        weight=-0.50,
        params={
            "command_name": "racket_target",
            "margin": 0.10,
            "std": 0.20,
            "t_lo": 0.10,
            "t_hi": 1.55,
        },
    )


@configclass
class HOPEPingPongHitterPureRallyV13TerminationsCfg(
    HOPEPingPongHitterPureRallyV12TerminationsCfg
):
    """V13 remains soft-supervised; no train-time READY gate or new termination."""


@configclass
class HOPEPingPongHitterPureRallyV13AgibotA3EnvCfg(
    HOPEPingPongHitterPureRallyV12AgibotA3EnvCfg
):
    """Fresh-train V13 with the V12 hit curriculum plus post-swing recovery-state coverage."""

    rewards: HOPEPingPongHitterPureRallyV13RewardsCfg = (
        HOPEPingPongHitterPureRallyV13RewardsCfg()
    )
    terminations: HOPEPingPongHitterPureRallyV13TerminationsCfg = (
        HOPEPingPongHitterPureRallyV13TerminationsCfg()
    )

    def __post_init__(self):
        super().__post_init__()
        # Keep stand starts and the planner/core strike curriculum unchanged.  Reassign one quarter
        # of true-reset RSI starts to the policy's own completed follow-through states so recovery
        # is learned from the states that actually lead to a second-swing hardware fall.
        self.commands.motion.post_swing_start_prob = 0.25
        self.commands.motion.post_swing_buffer_size = 8192
        self.commands.motion.post_swing_min_fill = 256
        self.commands.motion.post_swing_min_hold = 45


# Generic deploy-honest successor used by RallyV15.  The class contains only structural wiring;
# all numerical action/localization/reward choices remain in the selected task YAML.
@configclass
class HOPEHitterPureBoundedQdesActionsCfg(ActionsCfg):
    joint_pos = mdp.BoundedJointPositionActionCfg(
        asset_name="robot",
        joint_names=[".*"],
        use_default_offset=True,
        passive_joint_names=mdp.A3_PASSIVE_HEAD_JOINT_NAMES,
    )


@configclass
class HOPEHitterPureV11SafeActionsCfg(ActionsCfg):
    """Exact V11 affine/soft-clamp action with post-physics safety audit support."""

    joint_pos = mdp.V11SafeClampedJointPositionActionCfg(
        asset_name="robot",
        joint_names=[".*"],
        use_default_offset=True,
        passive_joint_names=mdp.A3_PASSIVE_HEAD_JOINT_NAMES,
    )


@configclass
class HOPEHitterPureBoundedQdesTerminationsCfg(
    HOPEDeployParityTerminationsCfg
):
    # Post-physics plant-state safety hook. A hard-limit excursion latches and
    # snapshots the first physical fault, then terminates only the affected
    # vectorized environments. Numeric/q_des contract faults remain global
    # fail-fast paths in the action term.
    actual_q_hard_limit_audit = DoneTerm(
        func=mdp.actual_q_hard_limit_audit,
        params={"action_name": "joint_pos"},
    )


@configclass
class HOPEObservationsHitterPureV15Cfg(HOPEObservationsHitterPureRallyFinalV3Cfg):
    """118-D deployable HITTER + HUGWBC locomotion command contract."""

    @configclass
    class PolicyCfg(HOPEObservationsHitterPureRallyFinalV3Cfg.PolicyCfg):
        actions = ObsTerm(
            func=mdp.executed_qdes_feedback, params={"action_name": "joint_pos"}
        )
        base_target_delta_xy = ObsTerm(
            func=mdp.base_target_delta_xy_mocap,
            params={"command_name": "racket_target"},
        )
        racket_target_rel_base = ObsTerm(
            func=mdp.racket_target_rel_base_mocap,
            params={"command_name": "racket_target"},
        )
        base_velocity_xy = ObsTerm(
            func=mdp.base_velocity_xy_mocap,
            params={"command_name": "racket_target"},
        )
        localization_age = ObsTerm(
            func=mdp.base_localization_age,
            params={"command_name": "racket_target"},
        )
        desired_lateral_velocity = ObsTerm(
            func=mdp.desired_lateral_velocity,
            params={"command_name": "racket_target"},
        )
        gait_clock = ObsTerm(
            func=mdp.gait_clock,
            params={"command_name": "racket_target"},
        )
        locomotion_mode = ObsTerm(
            func=mdp.locomotion_mode,
            params={"command_name": "racket_target"},
        )
        upper_intervention = ObsTerm(
            func=mdp.upper_intervention_indicator,
            params={"action_name": "joint_pos"},
        )

    @configclass
    class CriticCfg(HOPEObservationsHitterPureRallyFinalV3Cfg.CriticCfg):
        actions = ObsTerm(
            func=mdp.executed_qdes_feedback, params={"action_name": "joint_pos"}
        )
        base_velocity_xy = ObsTerm(
            func=mdp.base_velocity_xy_mocap,
            params={"command_name": "racket_target"},
        )
        localization_age = ObsTerm(
            func=mdp.base_localization_age,
            params={"command_name": "racket_target"},
        )
        desired_lateral_velocity = ObsTerm(
            func=mdp.desired_lateral_velocity,
            params={"command_name": "racket_target"},
        )
        gait_clock = ObsTerm(
            func=mdp.gait_clock,
            params={"command_name": "racket_target"},
        )
        locomotion_mode = ObsTerm(
            func=mdp.locomotion_mode,
            params={"command_name": "racket_target"},
        )
        upper_intervention = ObsTerm(
            func=mdp.upper_intervention_indicator,
            params={"action_name": "joint_pos"},
        )

    policy: PolicyCfg = PolicyCfg()
    critic: CriticCfg = CriticCfg()


@configclass
class HOPEObservationsHitterPureV17Cfg(
    HOPEObservationsHitterPureRallyFinalV3Cfg
):
    """110-D V17 actor with deploy-faithful action feedback and mocap base pose."""

    @configclass
    class PolicyCfg(
        HOPEObservationsHitterPureRallyFinalV3Cfg.PolicyCfg
    ):
        # Preserve the exact 110-D order and dimensions.  The parent keeps the
        # two passive-head last-action slots at zero, matching the C++ runner;
        # the terms below change only base-pose provenance to deploy-matched mocap.
        projected_gravity = ObsTerm(
            func=mdp.projected_gravity_mocap,
            params={"command_name": "racket_target"},
        )
        base_forward_xy = ObsTerm(
            func=mdp.base_forward_xy_mocap,
            params={"command_name": "racket_target"},
        )
        base_target_delta_xy = ObsTerm(
            func=mdp.base_target_delta_xy_mocap,
            params={"command_name": "racket_target"},
        )
        racket_target_rel_base = ObsTerm(
            func=mdp.racket_target_rel_base_mocap,
            params={"command_name": "racket_target"},
        )

    policy: PolicyCfg = PolicyCfg()


@configclass
class HitterPingPongObservationsCfg(
    HOPEObservationsHitterPureRallyFinalV3Cfg
):
    """V14's exact 110-D actor layout with only base-pose provenance changed to mocap.

    Joint/proprioception ordering and the two passive-head action-history slots are unchanged.
    Base angular velocity remains the inherited IMU/gyro term.  Only the four terms that require
    world base position or orientation read the synchronized full-pose mocap state.
    """

    @configclass
    class PolicyCfg(HOPEObservationsHitterPureRallyFinalV3Cfg.PolicyCfg):
        projected_gravity = ObsTerm(
            func=mdp.projected_gravity_mocap,
            params={"command_name": "racket_target"},
            noise=Unoise(n_min=-0.05, n_max=0.05),
        )
        base_forward_xy = ObsTerm(
            func=mdp.base_forward_xy_mocap,
            params={"command_name": "racket_target"},
            noise=Unoise(n_min=-0.02, n_max=0.02),
        )
        base_target_delta_xy = ObsTerm(
            func=mdp.base_target_delta_xy_mocap,
            params={"command_name": "racket_target"},
            noise=Unoise(n_min=-0.03, n_max=0.03),
        )
        racket_target_rel_base = ObsTerm(
            func=mdp.racket_target_rel_base_mocap,
            params={"command_name": "racket_target"},
            noise=Unoise(n_min=-0.02, n_max=0.02),
        )

    policy: PolicyCfg = PolicyCfg()


@configclass
class HitterPingPongRewardsCfg(
    HOPEPingPongHitterPureRallyV13RewardsCfg
):
    """V14 reward surface plus Unitree's standard actual-joint acceleration cost."""

    joint_acc = RewTerm(func=mdp.joint_acc_l2, weight=-2.5e-7)


@configclass
class HitterPingPongTerminationsCfg(
    HOPEPingPongHitterPureRallyV13TerminationsCfg
):
    """V14 terminations plus mocap stale fail-closed and non-terminating actual-q telemetry.

    Actual-q hard-limit excursions remain latched telemetry and checkpoint NO-GO evidence; they
    are intentionally not a PPO episode termination, matching the Unitree training structure.
    """

    # TerminationManager is the post-physics/pre-reset hook.  This term samples and snapshots the
    # terminal plant state but always returns False, so simultaneous fall/stale/timeout resets
    # cannot erase actual-q evidence and actual-q itself does not shorten the episode.
    actual_q_hard_limit_telemetry = DoneTerm(
        func=mdp.actual_q_hard_limit_telemetry,
        params={"action_name": "joint_pos"},
    )
    base_mocap_stale = DoneTerm(
        func=mdp.base_mocap_stale,
        params={"command_name": "racket_target"},
    )


@configclass
class HitterPingPongAgibotA3EnvCfg(
    HOPEPingPongHitterPureRallyV13AgibotA3EnvCfg
):
    """Standalone Build task: V14 strike contract plus fixed recovery/plant deltas."""

    observations: HitterPingPongObservationsCfg = HitterPingPongObservationsCfg()
    actions: HOPEHitterPureV11SafeActionsCfg = HOPEHitterPureV11SafeActionsCfg()
    rewards: HitterPingPongRewardsCfg = HitterPingPongRewardsCfg()
    terminations: HitterPingPongTerminationsCfg = HitterPingPongTerminationsCfg()

    def __post_init__(self):
        super().__post_init__()

        # Formal Build batch. Explicit parse_env_cfg(..., num_envs=...) still overrides this
        # for bounded smoke/evaluation runs.
        self.scene.num_envs = 6144

        # Exact V14 numerical delta over V13.  The standalone task YAML repeats these values as
        # the launch source of truth; keeping them here also makes direct Gym construction honest.
        self.commands.motion.post_swing_start_prob = 0.35
        self.commands.motion.fixed_clip_env_fraction_per_clip = 0.125
        self.commands.motion.post_swing_replay_contract = "legacy_state_v1"
        self.commands.motion.post_swing_buffer_size = 8192
        self.commands.motion.post_swing_min_fill = 256
        self.commands.motion.post_swing_capture_phase_bins = 1
        self.commands.motion.post_swing_capture_severity_bins = 1
        self.commands.motion.post_swing_risk_edge_capture = False
        self.commands.motion.post_swing_failure_adaptive = False
        self.commands.motion.post_swing_min_fill_per_bucket = 1
        self.commands.motion.post_swing_curriculum_scaled = False
        # V14 recovery is available from the first genuine follow-through.  The
        # strike gate below is reserved for mocap corruption and cannot suppress
        # post-swing capture or reset sampling.
        self.commands.motion.post_swing_ability_gate_enabled = False
        self.commands.motion.post_swing_replay_ramp_probabilities = ()
        self.commands.racket_target.post_strike_capture_delays_s = ()
        self.actions.joint_pos.actual_q_hard_tolerance_rad = 0.002
        self.actions.joint_pos.actual_q_hard_audit_mode = "telemetry"
        self.actions.joint_pos.qdes_delay_min_steps = 0
        self.actions.joint_pos.qdes_delay_max_steps = 0
        self.actions.joint_pos.qdes_delay_nominal_fraction = 1.0
        self.rewards.rally_joint_qdes_saturation.weight = -0.65
        self.rewards.rally_joint_qdes_saturation.params.update(
            safe_margin_fraction=0.08, topk_blend=0.90
        )
        self.rewards.right_elbow_extension_debt.weight = -0.50
        self.rewards.right_elbow_extension_debt.params["t_post"] = 0.20
        self.rewards.strike_x_gate_margin.params["forehand_scale"] = 0.75
        self.rewards.post_swing_xlock.weight = -0.45
        self.rewards.rally_heading_debt.weight = -0.50
        self.rewards.rally_heading_debt.params["forehand_scale"] = 1.25
        self.rewards.left_wrist_reference_debt.weight = -0.40
        self.rewards.post_swing_base_quiet.weight = -0.35
        self.rewards.rally_ready_root_height_debt.weight = -0.35
        self.rewards.post_swing_tilt_debt.weight = -0.70
        # Restore the complete V14 reward timing surface.  Build adds joint_acc as
        # telemetry-aligned regularization, but does not delay recovery rewards until
        # after the unstable part of follow-through.
        self.rewards.strike_x_drift.params["t_post"] = 1.55
        self.rewards.post_strike_leg_quiet.weight = -0.04
        self.rewards.post_swing_xlock.params["t_lo"] = 0.10
        self.rewards.rally_heading_debt.params["post_t_lo"] = 0.20
        self.rewards.post_swing_base_quiet.params["t_lo"] = 0.20
        self.rewards.post_swing_leg_quiet.params["t_lo"] = 0.20

        racket = self.commands.racket_target
        racket.ability_curriculum_mode = "one_way_strike_gate_v1"
        racket.ability_min_exact_samples_per_side = 50.0
        racket.ability_min_completion_per_side = 0.55
        racket.ability_min_position_pass_per_side = 0.15
        racket.ability_min_composite = 0.03
        racket.ability_max_post_fall = 0.10
        racket.ability_gate_dwell_steps = 250
        racket.base_mocap_robustness_ramp_steps = 8000

        # A3 Parkour-style log-uniform Kp/Kd randomization, with a fixed 25% nominal
        # actuator cohort.  Kd randomizes only the deploy message term; passive damping
        # remains fixed and the V14 affine action scale remains nominal.
        self.events.randomize_pd_gains = EventTerm(
            func=mdp.randomize_a3_message_pd_gains,
            mode="startup",
            params={
                "asset_cfg": SceneEntityCfg("robot", joint_names=[".*"]),
                "alpha_range": (0.85, 1.15),
                "beta_range": (0.85, 1.15),
                "nominal_fraction": 0.25,
            },
        )

        # Preserve the exact V14 material ranges, adding only cat_stable's correctness invariant:
        # sampled dynamic friction is clamped not to exceed sampled static friction.
        material = self.events.physics_material.params
        material.update(
            asset_cfg=SceneEntityCfg(
                "robot", body_names=list(A3_FEET_BODIES), preserve_order=True
            ),
            static_friction_range=(0.3, 1.6),
            dynamic_friction_range=(0.3, 1.2),
            restitution_range=(0.0, 0.5),
            num_buckets=64,
            make_consistent=True,
        )

        # Formal training remains V14-style virtual-ball-only.  The cat_stable rigid-body truth
        # instrument is attached explicitly by the bounded low-env audit entrypoint; keeping it
        # out here avoids allocating one rigid ball and table visual per training environment.
        racket.virtual_ball = True
        racket.physical_ball = False
        racket.physical_ball_impulse = False
        racket.physical_ball_substep = 1


@configclass
class HOPEHitterPureBoundedQdesRewardsCfg(HOPEHitterPureRewardsCfg):
    """Clean HITTER strike objective on a HUGWBC lower-body foundation.

    This intentionally does not inherit the RallyV9--V14 symptom-specific reward stack.  The
    station is converted to one finite gait command; velocity, gait contact, posture and the
    paper's upper-body intervention curriculum train locomotion as a coherent subsystem.
    """

    # Replace HITTER's always-on pre-strike position servo with a finite STEP/terminal score.
    base_position = None
    # Lower-body balance is commanded directly; do not also pull the pelvis toward a demo anchor.
    motion_global_anchor_ori = None
    upright = None
    # V15 smooths the final projected q_des below.  The inherited IsaacLab term reads raw actor
    # outputs and cannot see projector-induced command motion, so keeping both would charge two
    # different quantities under the same name.
    action_rate_l2 = None

    # HITTER imitation remains a clean-group upper-body teacher.  In HUGWBC's intervention group
    # those shoulder/elbow q_des values are externally replaced, so the corresponding imitation
    # rewards are masked instead of asking the actor to optimize actions it does not execute.
    # Anchor-relative velocity tracking also avoids importing clip-root drift into the locomotion
    # controller.  These are structural ownership rules; the task YAML remains the source for all
    # active numerical weights and intervention parameters.
    motion_body_pos = RewTerm(
        func=mdp.motion_body_pos_swing_only,
        weight=1.0,
        params={
            "command_name": "motion",
            "std": 0.3,
            "body_names": A3_UPPER_TRACKED,
            "intervention_action_name": "joint_pos",
            "racket_command_name": None,
            "strike_free_pre_s": 0.0,
            "follow_through_free_s": 0.0,
        },
    )
    motion_body_ori = RewTerm(
        func=mdp.motion_body_ori_swing_only,
        weight=1.0,
        params={
            "command_name": "motion",
            "std": 0.4,
            "body_names": A3_UPPER_TRACKED,
            "intervention_action_name": "joint_pos",
            "racket_command_name": None,
            "strike_free_pre_s": 0.0,
            "follow_through_free_s": 0.0,
        },
    )
    motion_body_lin_vel = RewTerm(
        func=mdp.motion_body_lin_vel_anchor_relative_swing_only,
        weight=1.0,
        params={
            "command_name": "motion",
            "std": 1.0,
            "body_names": A3_UPPER_TRACKED,
            "intervention_action_name": "joint_pos",
            "racket_command_name": None,
            "strike_free_pre_s": 0.0,
            "follow_through_free_s": 0.0,
        },
    )
    motion_body_ang_vel = RewTerm(
        func=mdp.motion_body_ang_vel_anchor_relative_swing_only,
        weight=1.0,
        params={
            "command_name": "motion",
            "std": 3.14,
            "body_names": A3_UPPER_TRACKED,
            "intervention_action_name": "joint_pos",
            "racket_command_name": None,
            "strike_free_pre_s": 0.0,
            "follow_through_free_s": 0.0,
        },
    )

    # A Gaussian orientation kernel is the precise near-target objective, but its gradient
    # vanishes when a fresh policy exposes the opposite paddle face.  Keep the structural term
    # generic/inert here; the selected task YAML owns its weight, margin and scale.
    racket_normal_alignment_debt = RewTerm(
        func=mdp.racket_normal_alignment_debt,
        weight=0.0,
        params={"command_name": "racket_target", "margin": 0.10, "std": 0.35},
    )
    # V15 does not register the old full-window moving-trajectory debt. Precision outside the
    # 7.5 cm hard boundary is charged only against the static strike point in a three-tick local
    # window. The task YAML owns the active weight, margin, Huber scale and window.
    racket_position_alignment_debt = None
    racket_exact_position_debt = RewTerm(
        func=mdp.racket_exact_position_debt,
        weight=0.0,
        params={
            "command_name": "racket_target",
            "margin": 0.075,
            "huber_scale": 0.025,
            "window_s": 0.02,
        },
    )
    # Early-to-contact velocity bridge.  The task YAML owns the active negative weight and
    # Huber scales; structural defaults stay inert for every other task.
    racket_preimpact_velocity_debt = RewTerm(
        func=mdp.racket_preimpact_velocity_huber_debt,
        weight=0.0,
        params={
            "command_name": "racket_target",
            "preimpact_s": 0.30,
            "margin": 0.50,
            "huber_scale": 0.50,
        },
    )

    hugwbc_lateral_velocity = RewTerm(
        func=mdp.hugwbc_lateral_velocity_tracking_exp,
        weight=0.0,
        params={"command_name": "racket_target", "std": 1.0},
    )
    hugwbc_yaw_rate = RewTerm(
        func=mdp.hugwbc_yaw_rate_tracking_exp,
        weight=0.0,
        params={"command_name": "racket_target", "std": 1.0},
    )
    hugwbc_finite_station = RewTerm(
        func=mdp.hugwbc_finite_station_tracking_exp,
        weight=0.0,
        params={
            "command_name": "racket_target",
            "std": 1.0,
            "stand_deadband": 1.0,
        },
    )
    # Soft final-hold READY supervision. This term is deliberately a debt and never controls the
    # fixed external release clock. V15's selected task YAML owns the active weight, transition
    # classes, and whether all strict runner-equivalent READY channels are supervised.
    ready_deadline = RewTerm(
        func=mdp.rally_ready_deadline_debt,
        weight=0.0,
        params={
            "command_name": "racket_target",
            "motion_command_name": "motion",
            "x_margin": 0.10,
            "y_margin": 0.10,
            "position_std": 0.10,
            "speed_margin": 0.20,
            "speed_std": 0.20,
            "speed_blend": 0.60,
            "final_window_s": 0.12,
            "target_step_class": 3,
            "target_step_classes": (1, 2, 3),
            "match_strict_ready": False,
        },
    )
    # V13/V14-proven recovery pressure, added structurally here but numerically owned by V15 YAML.
    # It is zero inside a 3 cm x deadband and only active after impact, so it does not pay
    # micro-stepping or compete with the finite world-Y station transition.
    post_swing_xlock = RewTerm(
        func=mdp.rally_post_swing_xlock_debt,
        weight=0.0,
        params={
            "command_name": "racket_target",
            "margin": 0.030,
            "std": 0.040,
            "t_lo": 0.10,
            "t_hi": 1.55,
            "forehand_scale": 1.0,
            "backhand_scale": 1.0,
        },
    )
    # Recovery-only pelvis tilt tail. The selected task YAML owns its active weight/window.
    post_swing_tilt_debt = RewTerm(
        func=mdp.post_swing_tilt_debt,
        weight=0.0,
        params={
            "command_name": "racket_target",
            "margin": 0.10,
            "std": 0.20,
            "t_lo": 0.10,
            "t_hi": 1.55,
        },
    )
    # Recovery-only WORLD-HEADING tail (2026-07-25). x and tilt already had a post-swing debt;
    # yaw did not, and hold_heading is gated to in_hold, so swing-injected yaw was untaxed and
    # compounded until it closed the ready gate. Same weight=0.0 default: the task YAML owns it.
    post_swing_heading_debt = RewTerm(
        func=mdp.post_swing_heading_debt,
        weight=0.0,
        params={
            "command_name": "racket_target",
            "margin": 0.087,
            "std": 0.25,
            "t_lo": 0.10,
            "t_hi": 1.55,
        },
    )
    hugwbc_contact_force = RewTerm(
        func=mdp.hugwbc_contact_force_tracking,
        weight=0.0,
        params={
            "command_name": "racket_target",
            "sensor_cfg": SceneEntityCfg(
                "contact_forces", body_names=A3_FEET_BODIES, preserve_order=True
            ),
            "force_sigma": 1.0,
        },
    )
    hugwbc_contact_velocity = RewTerm(
        func=mdp.hugwbc_contact_velocity_tracking,
        weight=0.0,
        params={
            "command_name": "racket_target",
            "asset_cfg": SceneEntityCfg(
                "robot", body_names=A3_FEET_BODIES, preserve_order=True
            ),
            "velocity_sigma": 1.0,
        },
    )
    hugwbc_standing = RewTerm(
        func=mdp.hugwbc_standing_double_contact,
        weight=0.0,
        params={
            "command_name": "racket_target",
            "sensor_cfg": SceneEntityCfg(
                "contact_forces", body_names=A3_FEET_BODIES, preserve_order=True
            ),
            "force_threshold": 1.0,
        },
    )
    hugwbc_standing_air = RewTerm(
        func=mdp.hugwbc_standing_air,
        weight=0.0,
        params={
            "command_name": "racket_target",
            "sensor_cfg": SceneEntityCfg(
                "contact_forces", body_names=A3_FEET_BODIES, preserve_order=True
            ),
            "force_threshold": 1.0,
        },
    )
    hugwbc_orientation = RewTerm(
        func=mdp.hugwbc_orientation_control,
        weight=0.0,
        params={"command_name": "racket_target", "swing_scale": 0.0},
    )
    hugwbc_waist = RewTerm(
        func=mdp.hugwbc_waist_yaw_roll_control,
        weight=0.0,
        params={
            "command_name": "racket_target",
            "asset_cfg": SceneEntityCfg(
                "robot",
                joint_names=["waist_yaw_joint", "waist_roll_joint"],
                preserve_order=True,
            ),
            # Default keeps the historical hold-only mask; the task YAML opts into the
            # swing-supervised mask via rewards.hugwbc_waist_swing_supervised.
            "swing_supervised": False,
        },
    )
    hugwbc_base_height = RewTerm(
        func=mdp.hugwbc_base_height_control,
        weight=0.0,
        params={
            "command_name": "racket_target",
            "target_height": 1.0,
            "stand_scale": 1.0,
        },
    )
    hugwbc_stand_still = RewTerm(
        func=mdp.hugwbc_stand_still_foot_placement,
        weight=0.0,
        params={
            "command_name": "racket_target",
            "asset_cfg": SceneEntityCfg(
                "robot", body_names=A3_FEET_BODIES, preserve_order=True
            ),
            "stance_width": 1.0,
        },
    )
    # Swing-foot lift shaping for the finite STEP (released HUGWBC feet_clearance; the
    # 2026-07-23 audit found the port omitted it entirely).  Inert here; YAML owns the numbers.
    hugwbc_feet_clearance = RewTerm(
        func=mdp.hugwbc_feet_clearance,
        weight=0.0,
        params={
            "command_name": "racket_target",
            "asset_cfg": SceneEntityCfg(
                "robot", body_names=A3_FEET_BODIES, preserve_order=True
            ),
            "target_height": 0.08,
        },
    )
    # Joint-space racket-arm anchor through the swing (audit: the Cartesian 7-body-mean imitation
    # cannot stop a contorted elbow/wrist that still reaches the racket target).  Exempt through
    # the contact window so it cannot pin the arm to the clip's strike point against off-manifold
    # sampled targets (2026-07-23 aiming-budget fix).
    swing_arm_imitation = RewTerm(
        func=mdp.swing_arm_joint_imitation,
        weight=0.0,
        params={
            "command_name": "motion",
            "action_name": "joint_pos",
            "std": 1.0,
            "joint_names": (),
            "racket_command_name": "racket_target",
            "strike_free_pre_s": 0.12,
            "follow_through_free_s": 0.30,
        },
    )
    # Non-saturating early-swing teacher for the active racket arm.  It releases before the
    # final target-owned acceleration window and therefore does not pin contact to the clip.
    swing_arm_huber_debt = RewTerm(
        func=mdp.swing_arm_joint_huber_debt,
        weight=0.0,
        params={
            "command_name": "motion",
            "action_name": "joint_pos",
            "joint_names": (),
            "racket_command_name": "racket_target",
            "release_pre_s": 0.30,
            "margin": 0.10,
            "huber_scale": 0.35,
        },
    )
    # v5 decode-saturation debt: projection debt is blind to rail-pinned postures because the v5
    # nominal is inside the safe range by construction (audit finding).
    qdes_saturation_debt = RewTerm(
        func=mdp.qdes_saturation_debt,
        weight=0.0,
        params={
            "command_name": "racket_target",
            "action_name": "joint_pos",
            "free_fraction": 0.90,
            "topk": 1,
            "topk_blend": 0.0,
            "upper_joint_names": (),
            "strike_free_pre_s": 0.12,
            "follow_through_free_s": 0.30,
        },
    )

    passive_head_raw_action = RewTerm(
        func=mdp.passive_head_raw_action_penalty,
        weight=0.0,
        params={
            "action_name": "joint_pos",
            "std": 1.0,
            "loss_type": "huber",
            "huber_delta": 1.0,
        },
    )
    hold_upper_joint_deviation = RewTerm(
        func=mdp.hold_upper_joint_deviation,
        weight=0.0,
        params={
            "command_name": "motion",
            "action_name": "joint_pos",
            "joint_names": (),
        },
    )
    executed_qdes_rate = RewTerm(
        func=mdp.executed_qdes_difference_l2,
        weight=0.0,
        params={
            "command_name": "racket_target",
            "action_name": "joint_pos",
            "order": 1,
            "upper_joint_names": (),
            "strike_free_pre_s": 0.12,
            "follow_through_free_s": 0.30,
        },
    )
    executed_qdes_second_difference = RewTerm(
        func=mdp.executed_qdes_difference_l2,
        weight=0.0,
        params={
            "command_name": "racket_target",
            "action_name": "joint_pos",
            "order": 2,
            "upper_joint_names": (),
            "strike_free_pre_s": 0.12,
            "follow_through_free_s": 0.30,
        },
    )
    joint_acc = RewTerm(
        func=mdp.joint_acc_l2,
        weight=0.0,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=[".*"])},
    )
    qdes_projection_debt = RewTerm(
        func=mdp.qdes_projection_debt,
        # Inert structural defaults; RallyV15 YAML owns the active weight/top-k recipe.
        weight=0.0,
        params={
            "command_name": "racket_target",
            "action_name": "joint_pos",
            "topk": 1,
            "topk_blend": 0.0,
        },
    )


@configclass
class HOPEPingPongHitterPureBoundedQdesAgibotA3EnvCfg(
    HOPEPingPongHitterPureAgibotA3EnvCfg
):
    """Paper-backed V15: HITTER strike task plus a HUGWBC lower-body foundation."""

    obs_mode: str = "hitter_pure_v15"
    actions: HOPEHitterPureBoundedQdesActionsCfg = HOPEHitterPureBoundedQdesActionsCfg()
    observations: HOPEObservationsHitterPureV15Cfg = HOPEObservationsHitterPureV15Cfg()
    rewards: HOPEHitterPureBoundedQdesRewardsCfg = HOPEHitterPureBoundedQdesRewardsCfg()
    terminations: HOPEHitterPureBoundedQdesTerminationsCfg = (
        HOPEHitterPureBoundedQdesTerminationsCfg()
    )

    def __post_init__(self):
        # RallyV8's parent initializer still narrows ee_body_pos to the feet. Let that complete
        # against the inherited term first, then remove all reference-deviation guards. Assigning
        # a fall-only TerminationsCfg as a class field would make ee_body_pos None too early.
        super().__post_init__()
        self.terminations.anchor_pos = None
        self.terminations.anchor_ori = None
        self.terminations.ee_body_pos = None


@configclass
class HOPEPingPongHitterPureRallyV16RewardsCfg(
    HOPEPingPongHitterPureRallyV11RewardsCfg
):
    """The complete V11 reward surface, with no V12--V15 additions."""


@configclass
class HOPEPingPongHitterPureRallyV16TerminationsCfg(
    HOPEPingPongHitterPureRallyV11TerminationsCfg
):
    """V11 physical guards plus a post-physics actual-q hard-limit audit."""

    actual_q_hard_limit_audit = DoneTerm(
        func=mdp.actual_q_hard_limit_audit,
        params={"action_name": "joint_pos"},
    )


@configclass
class HOPEPingPongHitterPureRallyV16AgibotA3EnvCfg(
    HOPEPingPongHitterPureRallyV11AgibotA3EnvCfg
):
    """Direct V11 child with an audited, behavior-identical final q_des clamp."""

    obs_mode: str = "hitter_pure"
    actions: HOPEHitterPureV11SafeActionsCfg = HOPEHitterPureV11SafeActionsCfg()
    rewards: HOPEPingPongHitterPureRallyV16RewardsCfg = (
        HOPEPingPongHitterPureRallyV16RewardsCfg()
    )
    terminations: HOPEPingPongHitterPureRallyV16TerminationsCfg = (
        HOPEPingPongHitterPureRallyV16TerminationsCfg()
    )


@configclass
class HOPEPingPongHitterPureRallyV17RewardsCfg(
    HOPEPingPongHitterPureRallyV16RewardsCfg
):
    """The exact V11 reward surface plus Unitree's standard joint-acc cost."""

    joint_acc = RewTerm(
        func=mdp.joint_acc_l2,
        weight=-2.5e-7,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=[".*"])},
    )


@configclass
class HOPEPingPongHitterPureRallyV17TerminationsCfg(
    HOPEPingPongHitterPureRallyV16TerminationsCfg
):
    """The unchanged V11/V16 physical and actual-q safety guards."""


@configclass
class HOPEPingPongHitterPureRallyV17AgibotA3EnvCfg(
    HOPEPingPongHitterPureRallyV16AgibotA3EnvCfg
):
    """RallyV11 behavior plus the YAML-pinned r12 tuple bank and hardware repairs."""

    obs_mode: str = "hitter_pure"
    actions: HOPEHitterPureV11SafeActionsCfg = HOPEHitterPureV11SafeActionsCfg()
    observations: HOPEObservationsHitterPureV17Cfg = (
        HOPEObservationsHitterPureV17Cfg()
    )
    rewards: HOPEPingPongHitterPureRallyV17RewardsCfg = (
        HOPEPingPongHitterPureRallyV17RewardsCfg()
    )
    terminations: HOPEPingPongHitterPureRallyV17TerminationsCfg = (
        HOPEPingPongHitterPureRallyV17TerminationsCfg()
    )

@configclass
class HOPEHitterPureRallyFinalV2PlusRewardsCfg(HOPEHitterPureRallyFinalV2RewardsCfg):
    """V2 reward surface with no train-time cost for deployment readiness."""

    # A fresh policy must experience the swing before readiness can be learned.  Keep the optional
    # diagnostic term structurally absent from this task rather than assigning it zero weight.
    arm_deadline_miss_penalty = None


@configclass
class HOPEHitterPureRallyFinalV2PlusTerminationsCfg(HOPEDeployParityTerminationsCfg):
    """V2 physical/reference guards; deployment readiness is telemetry, never termination."""

    arm_deadline_miss = None


@configclass
class HOPEPingPongHitterPureRallyFinalV2PlusAgibotA3EnvCfg(
    HOPEPingPongHitterPureRallyFinalV2AgibotA3EnvCfg
):
    """V2 command executor with optional lateral stationing and READY telemetry.

    The planner chooses an explicit station and racket target; the policy executes them.  A target
    already inside the trained per-side reach envelope can keep the previous station, while larger
    lateral residuals exercise V2's footwork.  The fixed training clamp always releases into the
    swing, matching HITTER's requirement that fresh trajectories see the imitation/strike window.
    READY remains observable in metrics; the deployment runner owns latest-release rejection.
    There is no policy-controlled hold extension.

    Actions, 110-D observations, V2 imitation/stability rewards and reference-aware termination
    guards stay intact.  The obsolete old-video clearance terms remain removed.
    """

    rewards: HOPEHitterPureRallyFinalV2PlusRewardsCfg = HOPEHitterPureRallyFinalV2PlusRewardsCfg()
    terminations: HOPEHitterPureRallyFinalV2PlusTerminationsCfg = (
        HOPEHitterPureRallyFinalV2PlusTerminationsCfg()
    )

    def __post_init__(self):
        super().__post_init__()

        # Surgical deletion requested after replacing the close-hands backhand video.
        self.rewards.backhand_left_arm_clearance = None
        self.rewards.backhand_left_hand_clearance = None

        C = self.commands.racket_target
        # Capability coverage, not a command to move every ball: 30% same station, 35% small
        # 3--20 cm correction, 35% V2 main 20--35 cm step.  The touching ranges remove the old
        # 12--20 cm support hole while retaining V2's proven main-step maximum.
        C.station_y_same_prob = 0.30
        C.station_y_small_step_prob = 0.35
        C.station_y_small_step_range = (0.03, 0.20)
        C.station_side_explicit = True
        C.ready_monitor_step_range = (0.0, 0.35)

        # Telemetry at the fixed U[40,60] clamp.  Position comes from mocap-equivalent base
        # localization; heading/rate/tilt are IMU; joint speed is proprioception.  Never use this
        # as a fresh-training termination: the 2026-07-11 run produced zero swing samples in 498
        # iterations when every rare survivor had to pass the joint READY conjunction first.
        C.arm_deadline_gate = False
        C.ready_monitor_x_thresh = 0.10
        C.ready_monitor_y_thresh = 0.10
        C.ready_monitor_speed_thresh = 0.20
        C.ready_monitor_dwell_s = 0.12
        C.ready_monitor_heading_thresh_rad = 0.2617993878  # 15 deg
        C.ready_monitor_yaw_rate_thresh = 0.35
        C.ready_monitor_tilt_thresh = 0.14  # projected-gravity xy ~= sin(8 deg)
        C.ready_monitor_joint_speed_thresh = 0.80  # all-joint RMS rad/s in the static ready pose

        # User-approved V7 contact frames FH=38, BH=37 over 104 frames.  Geometry is the A3/V7
        # station inverse and venue floor-z contact band; velocity is the deploy-honest planner's
        # centre-return envelope.  The conservative +/-4 cm reach-y bootstrap around each V7 nominal
        # lets the planner keep the station for small residuals.  The FH/BH bands stay disjoint with
        # 6 cm raw gap (2 cm after the actor's +/-2 cm target noise), preserving the 110-D no-side-bit
        # observation contract.  Base x remains locked; forward walking is never sampled.
        C.strike_phase = 0.36893204
        C.strike_phase_per_clip = (0.36893204, 0.35922330)
        C.racket_pos_range_per_clip = (
            ((0.70, 0.70), (-0.08, 0.00), (0.98, 1.26)),
            ((0.70, 0.70), (0.06, 0.14), (0.98, 1.26)),
        )
        C.racket_vel_range_per_clip = (
            ((0.25, 1.35), (0.00, 0.35), (0.10, 1.00)),
            ((0.25, 1.35), (-0.15, 0.12), (0.10, 1.00)),
        )
