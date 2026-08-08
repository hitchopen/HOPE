"""Agibot (Zhiyuan) Expedition A3 — ping-pong configuration for BeyondMimic / HOPE WBC.

This file is written against the OFFICIAL Agibot A3 ping-pong assets shipped in the HOPE repo
(the URDF ``agibot/URDF/A3T2.5-URDF-std-pingpang/`` and the MuJoCo MJCF
``a3_deploy/A3_MuJoCo_Sim/.../a3_pingpong/a3_pingpong.xml`` — both are Agibot-provided, not stand-ins).
Names, link inertials (urdf mass.txt), effort/velocity limits (joints.txt), joint armature, and the
standing pose are all taken verbatim from those files. The PD control gains (Kp/Kd = stiffness/damping
below) and the standing pose are the OFFICIAL Agibot values transcribed from the A3 deploy package
(``a3_deploy_onnx_ref/include/a3_policy_parameters.hpp`` — ``a3_kps`` / ``a3_kds`` / ``a3_default_angles``,
"a direct transcription of a3.py"). Head 40/2 is the deploy neck/head default (ExpandToBackend); the
2-DOF neck is not in the 29-DOF policy view.

Nothing here touches the filesystem at import time: ``ArticulationCfg`` only stores the asset
path string, so the A3 task registers and imports fine *without* the asset present. The path is
only resolved when an environment is actually instantiated for training.

A3 active DOF (31, excluding hands): waist yaw/roll/pitch (3), neck yaw/pitch (2),
each arm 7 (shoulder pitch/roll/yaw, elbow, wrist roll/pitch/yaw), each leg 6
(hip pitch/roll/yaw, knee, ankle pitch/roll). The right arm holds the paddle.
"""

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets.articulation import ArticulationCfg

from whole_body_tracking.assets import ASSET_DIR

##
# Asset path — the OFFICIAL Agibot A3 ping-pong URDF (copied from agibot/URDF/A3T2.5-URDF-std-pingpang/
# into assets/agibot_a3/, reimplement.md step 12). Isaac Lab spawns the URDF directly (as the G1
# config does with ``UrdfFileCfg``); for the MuJoCo/mjlab path use the official a3_pingpong.xml MJCF.
##
AGIBOT_A3_ASSET_ROOT = f"{ASSET_DIR}/agibot_a3"
AGIBOT_A3_URDF_PATH = f"{AGIBOT_A3_ASSET_ROOT}/urdf/model.urdf"  # official Agibot A3 ping-pong URDF

##
# Body / joint name constants (real names from the A3 ping-pong URDF). The rest of the HOPE
# code imports these so there is a single source of truth when the asset is swapped.
##
# NOTE the mixed casing — it is INTENTIONAL and matches the A3 URDF exactly: the root is
# "pelvis_link" (lowercase) while every other body uses "_Link" (capital L). MotionCommand does an
# exact-string lookup, so do not "normalize" these. Re-verify against the validated asset's link
# table when it arrives.
A3_ROOT_BODY = "pelvis_link"
A3_ANCHOR_BODY = "torso_Link"

# Bodies tracked by the BeyondMimic motion command (mirror of the G1 14-body set).
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

# Feet + hands; used for contact/termination exclusions.
A3_FEET_BODIES = ["left_ankle_roll_Link", "right_ankle_roll_Link"]
A3_HAND_BODIES = ["left_wrist_yaw_Link", "right_wrist_yaw_Link"]

# UPPER-body tracked bodies (torso + both arms) — the subset of A3_TRACKED_BODIES used by the
# footwork variant's imitation reward. The legs (pelvis/hip/knee/ankle) are intentionally EXCLUDED so
# the lower body is free to step/shift to reach different racket targets instead of copying the clip's
# fixed leg motion. Upper-body imitation still gives the swing its style.
A3_UPPER_TRACKED = [
    "torso_Link",
    "left_shoulder_roll_Link",
    "left_elbow_Link",
    "left_wrist_yaw_Link",
    "right_shoulder_roll_Link",
    "right_elbow_Link",
    "right_wrist_yaw_Link",
]

# Joint order for reading the retargeted-motion CSV in scripts/csv_to_npz.py. This is the order of
# the *DOF columns* in the A3 retargeted CSV (columns 7: after base pos/quat), i.e. the order your
# GMR retargeting outputs — NOT the simulation articulation order (the npz stores joint_pos in the
# articulation order automatically). The default below follows the A3 controller_joint_names.yaml
# (agibot/URDF/.../config/joint_names_*.yaml). IMPORTANT: if your GMR A3 retargeting emits a different
# column order, reorder this list to match it, or the npz joints will be scrambled.
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

# Racket mount (right arm). See whole_body_tracking/tasks/tracking/mdp/hope_commands.py.
A3_WRIST_BODY = "right_wrist_yaw_Link"          # last actuated link of the paddle arm
A3_RACKET_BODY = "pingpang_red_Link"            # racket-center body (coincident with pingbang_ball_Link)
# Offset wrist_yaw -> racket center, in the wrist_yaw local frame (meters). From the URDF
# pingbang_ball_joint origin; right_hand_pingpang_joint is xyz=0 rpy=0 so this equals the
# offset from right_wrist_yaw_Link directly.
A3_MOUNT_OFFSET = (0.210211399202899, 0.0320784994676765, 0.0320358706296689)


##
# Effort/velocity limits + joint armature are real (joints.txt + a3_pingpong.xml MJCF). The PD control
# gains (stiffness=Kp, damping=Kd) are the OFFICIAL Agibot values — transcribed from the A3 deploy
# package (a3_deploy_onnx_ref/include/a3_policy_parameters.hpp: a3_kps / a3_kds, "a direct transcription
# of a3.py"; the deploy sends these via ExpandToBackend). With the real effort limits, the resulting
# action scale 0.25*effort/stiffness EXACTLY matches the deploy's a3_action_scale, so training and
# deployment stay consistent (target = action*action_scale + default_angle). Head 40/2 = deploy default.
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
            # Self-collision is OFF: in the official URDF the merged wrist body carries 4 overlapping
            # collision meshes (wrist + hand_pingpang + red/black blades, all coincident) with thin blade
            # hulls, which corrupts PhysX at sim start ("free(): corrupted unsorted chunks" -> Aborted).
            # WBC imitation does not need self-collision. NOTE: the official MJCF a3_pingpong.xml already
            # ships a clean collision setup (convex hulls + primitive racket/hand geoms + adjacent-body
            # <contact><exclude> list) — port that into the URDF to re-enable; it is NOT an Agibot blocker.
            enabled_self_collisions=False, solver_position_iteration_count=8, solver_velocity_iteration_count=4
        ),
        joint_drive=sim_utils.UrdfConverterCfg.JointDriveCfg(
            gains=sim_utils.UrdfConverterCfg.JointDriveCfg.PDGainsCfg(stiffness=0, damping=0)
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        # Standing pose = a3.py ``default_angles`` (OFFICIAL Agibot deploy config,
        # a3_policy_parameters.hpp). This is used BOTH as the reset pose AND the action offset
        # (use_default_offset=True), so it must match the deploy action decoder exactly. Pelvis Z
        # 1.0684 m is the A3 MuJoCo stand-keyframe height for this (near-identical) leg pose; waist,
        # neck, shoulder_yaw and the wrists stay at 0.
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
            # arms — paddle-ready stance (right arm holds the racket)
            ".*_shoulder_pitch_joint": 0.3,
            "left_shoulder_roll_joint": 0.12,
            "right_shoulder_roll_joint": -0.12,
            ".*_elbow_joint": 0.8,
        },
        joint_vel={".*": 0.0},
    ),
    soft_joint_pos_limit_factor=0.9,
    # ACTUATOR MODEL (Phase A, 2026-07-02): ALL groups are ImplicitActuatorCfg. AGI officially trains
    # with IsaacLab implicit PD and the real robot's backend is close to implicit PD; the earlier
    # IdealPDActuatorCfg (explicit) round was built on a falsified premise (implicit training already
    # clamps torque — effort_limit_sim is written into the PhysX drive max force; the "elbow 6.7x24Nm"
    # figure was the PRE-clip computed_effort) and IdealPD@200Hz added discrete-overshoot dynamics
    # (wrist kd*dt/I ~ 1.3-2.5) that empirically DEGRADED the backhand. Do not reintroduce IdealPD.
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
            stiffness={  # a3.py / deploy a3_kps
                ".*_hip_yaw_joint": 80.0,
                ".*_hip_roll_joint": 120.0,
                ".*_hip_pitch_joint": 80.0,
                ".*_knee_joint": 250.0,
            },
            damping={  # kd_msg (a3.py / deploy a3_kds) + MJCF passive damping — see friction note
                ".*_hip_yaw_joint": 4.0,    # 3.0 + 1.0
                ".*_hip_roll_joint": 5.0,   # 4.0 + 1.0
                ".*_hip_pitch_joint": 4.0,  # 3.0 + 1.0
                ".*_knee_joint": 10.0,      # 8.0 + 2.0
            },
            armature={  # MJCF a3_pingpong.xml
                ".*_hip_yaw_joint": 0.06646569891,
                ".*_hip_roll_joint": 0.06646569891,
                ".*_hip_pitch_joint": 0.06646569891,
                ".*_knee_joint": 0.1203404,
            },
            # STATIC JOINT FRICTION (MJCF a3_pingpong.xml frictionloss) — 2026-07-05 sim2sim fix.
            # AGI's MuJoCo (their "≈ real hardware" gate) models 1.2-2.4 Nm of stiction per leg
            # joint; Isaac trained with ZERO. At quasi-static hold the policy's small corrective
            # torques (kp~60 x 0.05-0.2 rad ≈ 2-12 Nm) are mostly eaten by stiction -> commanded
            # corrections barely move the plant (Gate 2.5 measured trk 9-32%) -> the policy
            # (trained on a frictionless plant) winds up and tips the robot in 3-5 s. Big swing
            # motions are barely affected (feedforward-dominated, trk 80-200%), which is why
            # Gate 2 / Gate 3 pass while the bare hold fails.
            # PASSIVE VISCOUS DAMPING (2026-07-09 plant-alignment fix): every `damping` value in
            # this actuators dict is now  kd_msg + MJCF passive viscous damping  (a3_pingpong.xml
            # per-joint `damping`, 0.5-2.0 Nm·s/rad). Rationale: the deploy plants (AGI MuJoCo AND
            # the real drivetrain) apply the runner's message kd PLUS their own passive damping;
            # Isaac modeled only the message kd, so the policy trained on an under-damped plant —
            # measured on model_20200 via the 110-D mujoco_eval_onnx A/B: adding --keep-passive
            # alone took in-swing base-x excursion 0.066→0.084 mean / 0.121→0.464 max and racket
            # vel err 0.176→0.315, reproducing ~90% of the AGI-plant follow-through amplification
            # (explicit-Euler PD adds only the small remainder). Folding the passive term into the
            # implicit drive kd is exact for our command contract (dq_des ≡ 0, so kd·(0−qd) IS
            # viscous damping). THE EXPORT CONTRACT IS GUARDED: utils/exporter.py bakes the
            # DEPLOY message kd from A3_DEPLOY_JOINT_KD_PATTERNS below (NOT these raised training
            # values) — the plant adds its own passive damping, so baking the raised kd would
            # double-damp on deploy. Keep the three tables in sync: damping here ==
            # A3_DEPLOY_JOINT_KD_PATTERNS + A3_PASSIVE_JOINT_DAMPING_PATTERNS.
            friction={
                ".*_hip_yaw_joint": 1.1971,
                ".*_hip_roll_joint": 1.1971,
                ".*_hip_pitch_joint": 1.1971,
                ".*_knee_joint": 2.4276,
            },
        ),
        "feet": ImplicitActuatorCfg(
            joint_names_expr=[".*_ankle_pitch_joint", ".*_ankle_roll_joint"],
            effort_limit_sim={".*_ankle_pitch_joint": 118.2, ".*_ankle_roll_joint": 54.75},
            velocity_limit_sim={".*_ankle_pitch_joint": 10.8, ".*_ankle_roll_joint": 19.3},
            stiffness=50.0,  # a3.py / deploy a3_kps (ankle)
            damping=4.0,     # kd_msg 2.0 + MJCF passive 2.0 (ankle) — see legs friction note
            armature={".*_ankle_pitch_joint": 0.06444060531, ".*_ankle_roll_joint": 0.02012630058},
            friction={".*_ankle_pitch_joint": 1.4, ".*_ankle_roll_joint": 0.778},  # MJCF frictionloss
        ),
        # EXPLICIT PD (sim2real) — see the "feet" group note. effort_limit MUST be set (explicit-cfg
        "waist": ImplicitActuatorCfg(
            joint_names_expr=["waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint"],
            effort_limit_sim={"waist_yaw_joint": 220.0, "waist_roll_joint": 46.0, "waist_pitch_joint": 118.0},
            velocity_limit_sim={"waist_yaw_joint": 12.0, "waist_roll_joint": 22.7, "waist_pitch_joint": 9.2},
            stiffness={"waist_yaw_joint": 85.0, "waist_roll_joint": 50.0, "waist_pitch_joint": 50.0},  # a3_kps
            damping={"waist_yaw_joint": 4.0, "waist_roll_joint": 2.5, "waist_pitch_joint": 2.8},  # kd_msg (3/2/2) + MJCF passive (1.0/0.5/0.8)
            armature={"waist_yaw_joint": 0.06646569891, "waist_roll_joint": 0.01462087613, "waist_pitch_joint": 0.08820859156},
            friction={"waist_yaw_joint": 1.1971, "waist_roll_joint": 0.69223, "waist_pitch_joint": 1.7},  # MJCF frictionloss
        ),
        "head": ImplicitActuatorCfg(
            joint_names_expr=["head_yaw_joint", "head_pitch_joint"],
            effort_limit_sim=6.0,
            velocity_limit_sim=12.7,
            # neck/head kp=40, kd=2 from the deploy default (ExpandToBackend, A3 deploy example.md)
            stiffness=40.0,
            damping=3.0,  # kd_msg 2.0 + MJCF passive 1.0 (deploy overrides neck anyway)
            armature={"head_yaw_joint": 0.0008100893338, "head_pitch_joint": 0.0008100893338},
            friction=0.1,  # MJCF frictionloss
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
            stiffness={  # a3.py / deploy a3_kps
                ".*_shoulder_pitch_joint": 40.0,
                ".*_shoulder_roll_joint": 40.0,
                ".*_shoulder_yaw_joint": 30.0,
                ".*_elbow_joint": 30.0,
                ".*_wrist_roll_joint": 30.0,
                ".*_wrist_pitch_joint": 20.0,
                ".*_wrist_yaw_joint": 20.0,
            },
            damping={  # kd_msg (a3.py / deploy a3_kds) + MJCF passive damping — see legs friction note
                ".*_shoulder_pitch_joint": 4.5,  # 3.0 + 1.5
                ".*_shoulder_roll_joint": 4.5,   # 3.0 + 1.5
                ".*_shoulder_yaw_joint": 3.0,    # 2.0 + 1.0
                ".*_elbow_joint": 3.0,           # 2.0 + 1.0
                ".*_wrist_roll_joint": 3.0,      # 2.0 + 1.0
                ".*_wrist_pitch_joint": 3.0,     # 2.0 + 1.0
                ".*_wrist_yaw_joint": 3.0,       # 2.0 + 1.0
            },
            armature={  # MJCF a3_pingpong.xml
                ".*_shoulder_pitch_joint": 0.01208336871,
                ".*_shoulder_roll_joint": 0.01208336871,
                ".*_shoulder_yaw_joint": 0.004967351303,
                ".*_elbow_joint": 0.004967351303,
                ".*_wrist_roll_joint": 0.004967351303,
                ".*_wrist_pitch_joint": 0.0008100893338,
                ".*_wrist_yaw_joint": 0.0008100893338,
            },
            friction={  # MJCF a3_pingpong.xml frictionloss
                ".*_shoulder_pitch_joint": 0.6293,
                ".*_shoulder_roll_joint": 0.6293,
                ".*_shoulder_yaw_joint": 0.41197,
                ".*_elbow_joint": 0.41197,
                ".*_wrist_roll_joint": 0.41197,
                ".*_wrist_pitch_joint": 0.1,
                ".*_wrist_yaw_joint": 0.1,
            },
        ),
    },
)


# ================================================================================================ #
# DEPLOY kd CONTRACT (2026-07-09 plant-alignment fix — see the legs `friction` note above).
# The actuator `damping` values in A3_CFG are TRAINING-plant totals (message kd + MJCF passive
# viscous damping). The runner must SEND only the message kd (the plant adds its own passive
# damping), so utils/exporter.py bakes ONNX joint_damping from this table instead of
# data.default_joint_damping. The invariant  training_damping == deploy_kd + passive  is asserted
# PER JOINT at export time (utils/exporter.py resolves both tables via a3_deploy_joint_kd +
# a3_passive_joint_damping and raises on any mismatch; 2026-07-09 audit — this line previously
# CLAIMED a tests/exporter assertion that did not exist, and the exporter fell back SILENTLY to
# baking the raised training kd = double damping on the robot). Retune the three tables TOGETHER:
# A3_CFG actuator damping / A3_DEPLOY_JOINT_KD_PATTERNS / A3_PASSIVE_JOINT_DAMPING_PATTERNS.
# ================================================================================================ #
A3_DEPLOY_JOINT_KD_PATTERNS: list[tuple[str, float]] = [
    # (regex fullmatch pattern, message kd)  — a3.py / deploy a3_kds, the values every prior
    # generation baked and the AGI backend/hardware expects on the wire.
    (r".*_hip_yaw_joint", 3.0),
    (r".*_hip_roll_joint", 4.0),
    (r".*_hip_pitch_joint", 3.0),
    (r".*_knee_joint", 8.0),
    (r".*_ankle_pitch_joint", 2.0),
    (r".*_ankle_roll_joint", 2.0),
    (r"waist_yaw_joint", 3.0),
    (r"waist_roll_joint", 2.0),
    (r"waist_pitch_joint", 2.0),
    (r"head_yaw_joint", 2.0),
    (r"head_pitch_joint", 2.0),
    (r".*_shoulder_pitch_joint", 3.0),
    (r".*_shoulder_roll_joint", 3.0),
    (r".*_shoulder_yaw_joint", 2.0),
    (r".*_elbow_joint", 2.0),
    (r".*_wrist_roll_joint", 2.0),
    (r".*_wrist_pitch_joint", 2.0),
    (r".*_wrist_yaw_joint", 2.0),
]

A3_PASSIVE_JOINT_DAMPING_PATTERNS: list[tuple[str, float]] = [
    # (regex fullmatch pattern, MJCF a3_pingpong.xml per-joint passive viscous damping, Nm·s/rad)
    (r".*_hip_(yaw|roll|pitch)_joint", 1.0),
    (r".*_knee_joint", 2.0),
    (r".*_ankle_(pitch|roll)_joint", 2.0),
    (r"waist_yaw_joint", 1.0),
    (r"waist_roll_joint", 0.5),
    (r"waist_pitch_joint", 0.8),
    (r"head_(yaw|pitch)_joint", 1.0),
    (r".*_shoulder_(pitch|roll)_joint", 1.5),
    (r".*_shoulder_yaw_joint", 1.0),
    (r".*_elbow_joint", 1.0),
    (r".*_wrist_(roll|pitch|yaw)_joint", 1.0),
]


# The A3-ONLY joints that positively identify the robot: the Unitree G1's 29 joint names otherwise
# fullmatch every suffix pattern above (hips/knee/ankle/waist/shoulder/elbow/wrist — 2026-07-09 audit:
# a G1 export silently received A3 wire kds), so absence-of-mismatch is NOT an identity test.
_A3_MARKER_JOINTS = ("head_yaw_joint", "head_pitch_joint")


def a3_deploy_joint_kd(joint_names: list[str]) -> list[float] | None:
    """Resolve the DEPLOY message kd for each joint name via A3_DEPLOY_JOINT_KD_PATTERNS.

    Returns the per-joint kd list in the given order, or None ONLY when the name set is positively
    NOT the A3 (e.g. the Unitree G1: 29 joints, no head joints — its names otherwise fullmatch every
    suffix pattern, 2026-07-09 audit, so absence-of-mismatch is not an identity test). Everything
    ambiguous RAISES instead of falling back: the old silent None → callers bake
    data.default_joint_damping = the passive-INCLUSIVE training totals → the plant adds its own
    passive damping on top = double damping (knee 10 vs wire 8, ankles 2x). Ambiguous means an asset
    re-take that renamed or added joints — that must fail the export loudly, not de-tune the robot:
      * BOTH _A3_MARKER_JOINTS present → it IS the A3 → any unmatched name raises.
      * exactly ONE marker present → half-renamed head (e.g. head→neck re-take) → raises.
      * NO markers but the set is A3-SHAPED (>=30 joints, <=2 unmatched — the G1 has 29) → a fully
        head-renamed A3 → raises.
      * anything else → None (genuinely a different robot; caller keeps its own damping)."""
    import re

    out: list[float] = []
    unmatched: list[str] = []
    for name in joint_names:
        for pat, kd in A3_DEPLOY_JOINT_KD_PATTERNS:
            if re.fullmatch(pat, name):
                out.append(kd)
                break
        else:
            unmatched.append(name)
    n_markers = sum(m in joint_names for m in _A3_MARKER_JOINTS)
    a3_shaped = len(joint_names) >= 30 and len(unmatched) <= 2
    if n_markers == 0 and not a3_shaped:
        return None  # positively not the A3 (e.g. G1) — caller keeps the robot's own damping
    if unmatched or n_markers != 2:
        raise ValueError(
            "a3_deploy_joint_kd: robot looks like the A3 "
            f"({n_markers}/2 marker joints, {len(joint_names)} joints) but the deploy-kd contract "
            f"cannot resolve it — unmatched joint names: {unmatched or '(none; head markers missing)'}. "
            "Update the three kd tables in robots/agibot_a3.py TOGETHER (A3_CFG damping / "
            "A3_DEPLOY_JOINT_KD_PATTERNS / A3_PASSIVE_JOINT_DAMPING_PATTERNS) — silently baking the "
            "passive-inclusive training kd would DOUBLE-damp the robot on deploy."
        )
    return out


def a3_passive_joint_damping(joint_names: list[str]) -> list[float]:
    """Per-joint MJCF passive viscous damping via A3_PASSIVE_JOINT_DAMPING_PATTERNS (the plant-side
    half of the deploy-kd contract; 2026-07-09 audit turned this table from documentation into the
    export-time invariant  training_damping == deploy_kd + passive). Call only after a3_deploy_joint_kd
    positively identified the A3; raises on any unmatched name (same fail-loud contract)."""
    import re

    out: list[float] = []
    unmatched: list[str] = []
    for name in joint_names:
        for pat, b in A3_PASSIVE_JOINT_DAMPING_PATTERNS:
            if re.fullmatch(pat, name):
                out.append(b)
                break
        else:
            unmatched.append(name)
    if unmatched:
        raise ValueError(
            f"a3_passive_joint_damping: unmatched A3 joint names {unmatched} — update "
            "A3_PASSIVE_JOINT_DAMPING_PATTERNS (and the deploy/training tables) together."
        )
    return out


# Per-joint action scale, computed like the G1 config: 0.25 * effort_limit / stiffness.
AGIBOT_A3_ACTION_SCALE: dict[str, float] = {}
for _act in AGIBOT_A3_CFG.actuators.values():
    _eff = _act.effort_limit_sim
    _stiff = _act.stiffness
    _names = _act.joint_names_expr
    if not isinstance(_eff, dict):
        _eff = {n: _eff for n in _names}
    if not isinstance(_stiff, dict):
        _stiff = {n: _stiff for n in _names}
    for _n in _names:
        if _n in _eff and _n in _stiff and _stiff[_n]:
            AGIBOT_A3_ACTION_SCALE[_n] = 0.25 * _eff[_n] / _stiff[_n]
