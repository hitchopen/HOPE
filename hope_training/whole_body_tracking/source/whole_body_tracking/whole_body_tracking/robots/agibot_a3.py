"""Agibot A3 robot configuration for the HOPE whole-body task.

31 controllable DOF (hands excluded): waist yaw/roll/pitch (3), neck yaw/pitch (2), each arm 7
(shoulder pitch/roll/yaw, elbow, wrist roll/pitch/yaw), each leg 6 (hip pitch/roll/yaw, knee,
ankle pitch/roll). The right wrist carries the paddle.

Nothing here touches the filesystem at import time: :class:`ArticulationCfg` only stores the asset
path string, so the task registers and imports fine *without* the asset present. The path is only
resolved when an environment is actually instantiated for training. Supply your own A3 URDF/USD
under :data:`~whole_body_tracking.assets.ASSET_DIR` (see that package's README).

The actuator gains, effort/velocity limits, armature and the standing pose below are EXAMPLE values
that produce a reasonable starting point — transcribe your robot's real values before training a
model you intend to deploy.
"""

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets.articulation import ArticulationCfg

from whole_body_tracking.assets import ASSET_DIR

##
# Asset path — supply your own Agibot A3 asset here (the public repo ships assets/ empty).
##
AGIBOT_A3_ASSET_ROOT = f"{ASSET_DIR}/agibot_a3"
AGIBOT_A3_URDF_PATH = f"{AGIBOT_A3_ASSET_ROOT}/urdf/model.urdf"

##
# Canonical joint order (index 0..30). Training, ONNX export, the reference runner and the planner
# MUST all use this identical order. Head yaw/pitch (idx 3, 4) are held at their default at deploy
# (passive neck) but still occupy action columns.
##
AGIBOT_A3_JOINT_NAMES = [
    "waist_yaw_joint",
    "waist_roll_joint",
    "waist_pitch_joint",
    "head_yaw_joint",
    "head_pitch_joint",
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "left_wrist_roll_joint",
    "left_wrist_pitch_joint",
    "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint",
    "right_wrist_roll_joint",
    "right_wrist_pitch_joint",
    "right_wrist_yaw_joint",
    "left_hip_pitch_joint",
    "left_hip_roll_joint",
    "left_hip_yaw_joint",
    "left_knee_joint",
    "left_ankle_pitch_joint",
    "left_ankle_roll_joint",
    "right_hip_pitch_joint",
    "right_hip_roll_joint",
    "right_hip_yaw_joint",
    "right_knee_joint",
    "right_ankle_pitch_joint",
    "right_ankle_roll_joint",
]

# The two passive-at-deploy neck joints (held at default, still occupy action columns).
AGIBOT_A3_PASSIVE_HEAD_JOINT_NAMES = ("head_yaw_joint", "head_pitch_joint")

##
# Body / link names. The root is ``pelvis_link`` (lowercase); other bodies use ``_Link``. These are
# the asset's real link names — do not normalize the casing.
##
A3_ROOT_BODY = "pelvis_link"
A3_ANCHOR_BODY = "torso_Link"

# Bodies tracked by the motion-imitation command (pelvis, legs, torso, both arms).
A3_TRACKED_BODIES = [
    "pelvis_link",
    "left_hip_roll_Link",
    "left_knee_Link",
    "left_ankle_roll_Link",
    "right_hip_roll_Link",
    "right_knee_Link",
    "right_ankle_roll_Link",
    "torso_Link",
    "left_shoulder_roll_Link",
    "left_elbow_Link",
    "left_wrist_yaw_Link",
    "right_shoulder_roll_Link",
    "right_elbow_Link",
    "right_wrist_yaw_Link",
]

# Upper-body subset (torso + both arms). Used by the swing-imitation reward: the legs are excluded so
# the lower body is free to step/shift toward different racket targets instead of copying the clip's
# fixed leg motion, while upper-body imitation still supplies the swing style.
A3_UPPER_TRACKED = [
    "torso_Link",
    "left_shoulder_roll_Link",
    "left_elbow_Link",
    "left_wrist_yaw_Link",
    "right_shoulder_roll_Link",
    "right_elbow_Link",
    "right_wrist_yaw_Link",
]

# Feet + hands; used for contact/termination exclusions.
A3_FEET_BODIES = ["left_ankle_roll_Link", "right_ankle_roll_Link"]
A3_HAND_BODIES = ["left_wrist_yaw_Link", "right_wrist_yaw_Link"]

##
# Racket mount (right arm). The paddle center is reached from the last actuated wrist link by a fixed
# offset in the wrist frame. If your asset keeps a dedicated racket body, the command uses it directly;
# otherwise it falls back to (wrist pose) * (mount offset). The blade face normal convention lives in
# the motion YAML sidecar.
##
A3_WRIST_BODY = "right_wrist_yaw_Link"          # last actuated link of the paddle arm
A3_RACKET_BODY = "pingpang_red_Link"            # racket-center body (coincident with pingbang_ball_Link)
# Offset wrist_yaw -> racket center, in the wrist_yaw local frame (meters). From the URDF
# pingbang_ball_joint origin; right_hand_pingpang_joint is xyz=0 rpy=0 so this equals the
# offset from right_wrist_yaw_Link directly.
A3_MOUNT_OFFSET = (0.210211399202899, 0.0320784994676765, 0.0320358706296689)


##
# Articulation configuration.
#
# Effort/velocity limits come from joints.txt. PD gains (stiffness=Kp,
# damping=Kd) and armature values follow the A3 starter reference values. The
# shared action adapter uses the old HOPE per-joint scale
# 0.25 * effort_limit / stiffness.
##
AGIBOT_A3_CFG = ArticulationCfg(
    spawn=sim_utils.UrdfFileCfg(
        fix_base=False,
        replace_cylinders_with_capsules=True,
        asset_path=AGIBOT_A3_URDF_PATH,
        activate_contact_sensors=True,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            retain_accelerations=False,
            linear_damping=0.0,
            angular_damping=0.0,
            max_linear_velocity=1000.0,
            max_angular_velocity=1000.0,
            max_depenetration_velocity=1.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False,
            solver_position_iteration_count=8,
            solver_velocity_iteration_count=4,
        ),
        joint_drive=sim_utils.UrdfConverterCfg.JointDriveCfg(
            gains=sim_utils.UrdfConverterCfg.JointDriveCfg.PDGainsCfg(stiffness=0, damping=0)
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        # Old HOPE standing pose used both as the reset pose and as the action offset
        # (use_default_offset=True). Pelvis Z 1.0684 m is the A3 starter stand height
        # for this leg pose; waist, neck, shoulder_yaw, and wrists stay at 0.
        # NOTE: HOPEPingPongEnvCfg OVERRIDES this dict with the exact per-joint default_q from the
        # SHARED deploy config (a3_deploy/a3_deploy_example/config/action_adapter.yaml) — that
        # file is the single source of truth for the default pose; the values here are a matching
        # fallback for uses outside the task cfg. Edit the shared YAML, not this dict.
        pos=(0.0, 0.0, 1.0684),
        joint_pos={
            ".*_hip_pitch_joint": -0.1311,
            ".*_knee_joint": 0.2468,
            ".*_ankle_pitch_joint": -0.1204,
            "left_hip_roll_joint": 0.0056,
            "right_hip_roll_joint": -0.0056,
            "left_hip_yaw_joint": -0.0348,
            "right_hip_yaw_joint": 0.0348,
            "left_ankle_roll_joint": -0.0078,
            "right_ankle_roll_joint": 0.0078,
            ".*_shoulder_pitch_joint": 0.3,
            "left_shoulder_roll_joint": 0.12,
            "right_shoulder_roll_joint": -0.12,
            ".*_elbow_joint": 0.8,
        },
        joint_vel={".*": 0.0},
    ),
    soft_joint_pos_limit_factor=0.9,
    actuators={
        "legs": ImplicitActuatorCfg(
            joint_names_expr=[".*_hip_yaw_joint", ".*_hip_roll_joint", ".*_hip_pitch_joint", ".*_knee_joint"],
            effort_limit_sim={
                ".*_hip_yaw_joint": 220.0,
                ".*_hip_roll_joint": 220.0,
                ".*_hip_pitch_joint": 220.0,
                ".*_knee_joint": 320.0,
            },
            velocity_limit_sim={
                ".*_hip_yaw_joint": 12.0,
                ".*_hip_roll_joint": 12.0,
                ".*_hip_pitch_joint": 12.0,
                ".*_knee_joint": 14.6,
            },
            stiffness={
                ".*_hip_yaw_joint": 80.0,
                ".*_hip_roll_joint": 120.0,
                ".*_hip_pitch_joint": 80.0,
                ".*_knee_joint": 250.0,
            },
            damping={
                ".*_hip_yaw_joint": 3.0,
                ".*_hip_roll_joint": 4.0,
                ".*_hip_pitch_joint": 3.0,
                ".*_knee_joint": 8.0,
            },
            armature={
                ".*_hip_yaw_joint": 0.06646569891,
                ".*_hip_roll_joint": 0.06646569891,
                ".*_hip_pitch_joint": 0.06646569891,
                ".*_knee_joint": 0.1203404,
            },
        ),
        "feet": ImplicitActuatorCfg(
            joint_names_expr=[".*_ankle_pitch_joint", ".*_ankle_roll_joint"],
            effort_limit_sim={".*_ankle_pitch_joint": 118.2, ".*_ankle_roll_joint": 54.75},
            velocity_limit_sim={".*_ankle_pitch_joint": 10.8, ".*_ankle_roll_joint": 19.3},
            stiffness=50.0,
            damping=2.0,
            armature={".*_ankle_pitch_joint": 0.06444060531, ".*_ankle_roll_joint": 0.02012630058},
        ),
        "waist": ImplicitActuatorCfg(
            joint_names_expr=["waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint"],
            effort_limit_sim={"waist_yaw_joint": 220.0, "waist_roll_joint": 46.0, "waist_pitch_joint": 118.0},
            velocity_limit_sim={"waist_yaw_joint": 12.0, "waist_roll_joint": 22.7, "waist_pitch_joint": 9.2},
            stiffness={"waist_yaw_joint": 85.0, "waist_roll_joint": 50.0, "waist_pitch_joint": 50.0},
            damping={"waist_yaw_joint": 3.0, "waist_roll_joint": 2.0, "waist_pitch_joint": 2.0},
            armature={
                "waist_yaw_joint": 0.06646569891,
                "waist_roll_joint": 0.01462087613,
                "waist_pitch_joint": 0.08820859156,
            },
        ),
        "head": ImplicitActuatorCfg(
            joint_names_expr=["head_yaw_joint", "head_pitch_joint"],
            effort_limit_sim=6.0,
            velocity_limit_sim=12.7,
            stiffness=40.0,
            damping=2.0,
            armature={"head_yaw_joint": 0.0008100893338, "head_pitch_joint": 0.0008100893338},
        ),
        "arms": ImplicitActuatorCfg(
            joint_names_expr=[
                ".*_shoulder_pitch_joint",
                ".*_shoulder_roll_joint",
                ".*_shoulder_yaw_joint",
                ".*_elbow_joint",
                ".*_wrist_roll_joint",
                ".*_wrist_pitch_joint",
                ".*_wrist_yaw_joint",
            ],
            effort_limit_sim={
                ".*_shoulder_pitch_joint": 60.0,
                ".*_shoulder_roll_joint": 60.0,
                ".*_shoulder_yaw_joint": 24.0,
                ".*_elbow_joint": 24.0,
                ".*_wrist_roll_joint": 24.0,
                ".*_wrist_pitch_joint": 6.0,
                ".*_wrist_yaw_joint": 6.0,
            },
            velocity_limit_sim={
                ".*_shoulder_pitch_joint": 13.6,
                ".*_shoulder_roll_joint": 13.6,
                ".*_shoulder_yaw_joint": 15.7,
                ".*_elbow_joint": 15.7,
                ".*_wrist_roll_joint": 15.7,
                ".*_wrist_pitch_joint": 12.7,
                ".*_wrist_yaw_joint": 12.7,
            },
            stiffness={
                ".*_shoulder_pitch_joint": 40.0,
                ".*_shoulder_roll_joint": 40.0,
                ".*_shoulder_yaw_joint": 30.0,
                ".*_elbow_joint": 30.0,
                ".*_wrist_roll_joint": 30.0,
                ".*_wrist_pitch_joint": 20.0,
                ".*_wrist_yaw_joint": 20.0,
            },
            damping={
                ".*_shoulder_pitch_joint": 3.0,
                ".*_shoulder_roll_joint": 3.0,
                ".*_shoulder_yaw_joint": 2.0,
                ".*_elbow_joint": 2.0,
                ".*_wrist_roll_joint": 2.0,
                ".*_wrist_pitch_joint": 2.0,
                ".*_wrist_yaw_joint": 2.0,
            },
            armature={
                ".*_shoulder_pitch_joint": 0.01208336871,
                ".*_shoulder_roll_joint": 0.01208336871,
                ".*_shoulder_yaw_joint": 0.004967351303,
                ".*_elbow_joint": 0.004967351303,
                ".*_wrist_roll_joint": 0.004967351303,
                ".*_wrist_pitch_joint": 0.0008100893338,
                ".*_wrist_yaw_joint": 0.0008100893338,
            },
        ),
    },
)


# The action scale / default pose / joint clamp for the joint-position residual action
# (q_des = default_q + action * scale) are NOT defined here: they load from the ONE shared
# adapter config, a3_deploy/a3_deploy_example/config/action_adapter.yaml, via
# whole_body_tracking.utils.action_adapter_config — the same file the deploy runner reads.
