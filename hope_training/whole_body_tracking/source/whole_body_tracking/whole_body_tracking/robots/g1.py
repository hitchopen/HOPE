"""Unitree G1 robot configuration for the HOPE whole-body task.

29 controllable DOF (no head): waist yaw/roll/pitch (3), each arm 7 (shoulder pitch/roll/yaw,
elbow, wrist roll/pitch/yaw), each leg 6 (hip pitch/roll/yaw, knee, ankle pitch/roll). The right
wrist carries the paddle. This is structurally the Agibot A3 minus its two passive neck joints, so
the HOPE task machinery is shared; the G1 twin simply omits the head (``passive_joint_names=()``).

Unlike ``agibot_a3.py`` (which spawns from URDF), the G1 binds a **pre-converted USD** — the one
generated from ``g1_with_racket_adapter_short_ball_throwing.urdf``. Override the asset location with
the ``HOPE_G1_USD_PATH`` env var; the default points at the user's TTRL asset tree (note the space
in the directory name, which is part of the real path).

The actuator gains / effort-velocity limits / armature below are the **real G1 motor-derived**
values (armature-based, natural frequency 10 Hz, damping ratio ζ=2 — overdamped and stable),
transcribed from ``legged_lab/assets/unitree_g1/g1.py`` in the TTRL repo. They are the sim-to-real
starting point; retune if your hardware differs.
"""

import os

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets.articulation import ArticulationCfg

##
# Asset path — the already-converted G1 racket USD. The default is the user's TTRL asset; copy the
# whole ``g1_with_racket_adapter_short _ball_throwing/`` directory (it references configuration/*.usd
# by relative path) if you relocate it, or set HOPE_G1_USD_PATH. NOTE the space before "_ball".
##
_DEFAULT_G1_USD_PATH = (
    "/home/TTRL-ICRA2026/legged_lab/assets/unitree_description/urdf/g1/"
    "g1_with_racket_adapter_short _ball_throwing/g1_with_racket_adapter_short _ball_throwing.usd"
)
G1_USD_PATH = os.environ.get("HOPE_G1_USD_PATH", _DEFAULT_G1_USD_PATH)

##
# Canonical joint order (index 0..28). Training, ONNX export, the reference runner and the planner
# MUST all use this identical order. This is the Isaac-native articulation enumeration for the G1
# USD (breadth-first by tree depth, left/right interleaved with the waist chain), so the export gate
# passes with NO column remap. If the built articulation enumerates differently, the gate prints the
# real order — paste it here (and into ``config/joint_order_unitree_g1.yaml``) in one place.
##
G1_JOINT_NAMES = [
    "left_hip_pitch_joint",
    "right_hip_pitch_joint",
    "waist_yaw_joint",
    "left_hip_roll_joint",
    "right_hip_roll_joint",
    "waist_roll_joint",
    "left_hip_yaw_joint",
    "right_hip_yaw_joint",
    "waist_pitch_joint",
    "left_knee_joint",
    "right_knee_joint",
    "left_shoulder_pitch_joint",
    "right_shoulder_pitch_joint",
    "left_ankle_pitch_joint",
    "right_ankle_pitch_joint",
    "left_shoulder_roll_joint",
    "right_shoulder_roll_joint",
    "left_ankle_roll_joint",
    "right_ankle_roll_joint",
    "left_shoulder_yaw_joint",
    "right_shoulder_yaw_joint",
    "left_elbow_joint",
    "right_elbow_joint",
    "left_wrist_roll_joint",
    "right_wrist_roll_joint",
    "left_wrist_pitch_joint",
    "right_wrist_pitch_joint",
    "left_wrist_yaw_joint",
    "right_wrist_yaw_joint",
]

# G1 has no neck — there are NO passive-at-deploy joints. Kept for symmetry with the A3 cfg so the
# shared HOPE action term (``passive_joint_names``) is fed an explicit empty tuple.
G1_PASSIVE_HEAD_JOINT_NAMES: tuple[str, ...] = ()

##
# Body / link names. G1 links are lowercase ``_link`` (A3 used ``_Link``); the root is ``pelvis``
# (A3 used ``pelvis_link``). The torso is a real moving body (waist_pitch is revolute).
##
G1_ROOT_BODY = "pelvis"
G1_ANCHOR_BODY = "torso_link"

# Bodies tracked by the motion-imitation command (pelvis, legs, torso, both arms) — 14 bodies.
G1_TRACKED_BODIES = [
    "pelvis",
    "left_hip_roll_link",
    "left_knee_link",
    "left_ankle_roll_link",
    "right_hip_roll_link",
    "right_knee_link",
    "right_ankle_roll_link",
    "torso_link",
    "left_shoulder_roll_link",
    "left_elbow_link",
    "left_wrist_yaw_link",
    "right_shoulder_roll_link",
    "right_elbow_link",
    "right_wrist_yaw_link",
]

# Upper-body subset (torso + both arms) for the swing-imitation reward (legs free to step/shift).
G1_UPPER_TRACKED = [
    "torso_link",
    "left_shoulder_roll_link",
    "left_elbow_link",
    "left_wrist_yaw_link",
    "right_shoulder_roll_link",
    "right_elbow_link",
    "right_wrist_yaw_link",
]

# Feet + hands; used for contact/termination exclusions.
G1_FEET_BODIES = ["left_ankle_roll_link", "right_ankle_roll_link"]
G1_HAND_BODIES = ["left_wrist_yaw_link", "right_wrist_yaw_link"]

##
# Racket mount (right arm). The paddle is fixed to the last actuated wrist link. If the USD was
# imported with fixed joints MERGED, ``table_tennis_racket_link`` is not a separate body and the
# racket-target command falls back to (wrist pose) * (mount offset); if it survives import, the
# command uses that body directly. Either way we name both so the FK auto-detects.
#
# The URDF mounts ``table_tennis_racket_joint`` on ``right_wrist_yaw_link`` at
# xyz=(0.1485, 0, 0), rpy=(1.5708, 0, 0). The offset below reaches toward the paddle CENTER and the
# quaternion is that rpy; both are SEED values — verify/tune in the Isaac (or MuJoCo) viewer. The
# URDF itself marks the mount pose as provisional.
##
G1_WRIST_BODY = "right_wrist_yaw_link"          # last actuated link of the paddle arm
G1_RACKET_BODY = "table_tennis_racket_link"     # racket body if it survives USD import (else fallback)
G1_MOUNT_OFFSET = (0.24, 0.0, 0.0)              # wrist_yaw -> paddle center, wrist frame (SEED — tune)
G1_MOUNT_QUAT = (0.70710678, 0.70710678, 0.0, 0.0)  # rpy (1.5708, 0, 0): +90 deg about X (SEED — tune)
G1_MOUNT_NORMAL_AXIS = 1                        # racket-local blade-face axis (verify after mount tuning)


##
# Real G1 motor-derived actuator constants (from TTRL legged_lab/assets/unitree_g1/g1.py).
# K = armature * wn^2 ; D = 2 * zeta * armature * wn ; wn = 2*pi*10 Hz, zeta = 2 (overdamped).
##
_ARMATURE_5020 = 0.003609725     # shoulder p/r/y, elbow, wrist roll, ankle (single), waist roll/pitch
_ARMATURE_7520_14 = 0.010177520  # hip yaw/pitch, waist yaw
_ARMATURE_7520_22 = 0.025101925  # hip roll, knee
_ARMATURE_4010 = 0.00425         # wrist pitch/yaw

_WN = 10.0 * 2.0 * 3.1415926535  # ~62.83 rad/s
_ZETA = 2.0
_STIFFNESS_5020 = _ARMATURE_5020 * _WN**2       # ~14.25
_STIFFNESS_7520_14 = _ARMATURE_7520_14 * _WN**2  # ~40.18
_STIFFNESS_7520_22 = _ARMATURE_7520_22 * _WN**2  # ~99.10
_STIFFNESS_4010 = _ARMATURE_4010 * _WN**2        # ~16.78
_DAMPING_5020 = 2.0 * _ZETA * _ARMATURE_5020 * _WN       # ~0.91
_DAMPING_7520_14 = 2.0 * _ZETA * _ARMATURE_7520_14 * _WN  # ~2.56
_DAMPING_7520_22 = 2.0 * _ZETA * _ARMATURE_7520_22 * _WN  # ~6.31
_DAMPING_4010 = 2.0 * _ZETA * _ARMATURE_4010 * _WN        # ~1.07


##
# Articulation configuration. Spawns the pre-converted USD; drive gains come from the actuator
# groups below. The neutral standing pose and the residual-action offset are OVERRIDDEN by the task
# cfg from the shared deploy config (g1_deploy/g1_deploy_example/config/action_adapter.yaml) — that
# file is the single source of truth for default_q; the dict here is a matching fallback.
##
G1_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=G1_USD_PATH,
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
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        # EXAMPLE neutral standing pose (slight hip/knee bend, arms in a ready posture, waist 0).
        # HOPEPingPongEnvCfg OVERRIDES this with default_q from the shared action_adapter.yaml.
        pos=(0.0, 0.0, 0.79),
        joint_pos={
            ".*_hip_pitch_joint": -0.15,
            ".*_knee_joint": 0.30,
            ".*_ankle_pitch_joint": -0.15,
            "left_shoulder_pitch_joint": 0.20,
            "left_shoulder_roll_joint": 0.30,
            "left_elbow_joint": 0.50,
            "right_shoulder_pitch_joint": 0.0,
            "right_shoulder_roll_joint": -0.10,
            "right_elbow_joint": 0.50,
        },
        joint_vel={".*": 0.0},
    ),
    soft_joint_pos_limit_factor=0.9,
    actuators={
        "legs": ImplicitActuatorCfg(
            joint_names_expr=[".*_hip_pitch_joint", ".*_hip_roll_joint", ".*_hip_yaw_joint", ".*_knee_joint"],
            effort_limit_sim={
                ".*_hip_pitch_joint": 88.0,
                ".*_hip_roll_joint": 139.0,
                ".*_hip_yaw_joint": 88.0,
                ".*_knee_joint": 139.0,
            },
            velocity_limit_sim={
                ".*_hip_pitch_joint": 32.0,
                ".*_hip_roll_joint": 20.0,
                ".*_hip_yaw_joint": 32.0,
                ".*_knee_joint": 20.0,
            },
            stiffness={
                ".*_hip_pitch_joint": _STIFFNESS_7520_14,
                ".*_hip_roll_joint": _STIFFNESS_7520_22,
                ".*_hip_yaw_joint": _STIFFNESS_7520_14,
                ".*_knee_joint": _STIFFNESS_7520_22,
            },
            damping={
                ".*_hip_pitch_joint": _DAMPING_7520_14,
                ".*_hip_roll_joint": _DAMPING_7520_22,
                ".*_hip_yaw_joint": _DAMPING_7520_14,
                ".*_knee_joint": _DAMPING_7520_22,
            },
            armature={
                ".*_hip_pitch_joint": _ARMATURE_7520_14,
                ".*_hip_roll_joint": _ARMATURE_7520_22,
                ".*_hip_yaw_joint": _ARMATURE_7520_14,
                ".*_knee_joint": _ARMATURE_7520_22,
            },
        ),
        "feet": ImplicitActuatorCfg(
            joint_names_expr=[".*_ankle_pitch_joint", ".*_ankle_roll_joint"],
            effort_limit_sim={".*_ankle_pitch_joint": 50.0, ".*_ankle_roll_joint": 50.0},
            velocity_limit_sim={".*_ankle_pitch_joint": 37.0, ".*_ankle_roll_joint": 37.0},
            stiffness=2.0 * _STIFFNESS_5020,   # dual 5020 -> ~28.50
            damping=2.0 * _DAMPING_5020,       # ~1.81
            armature=2.0 * _ARMATURE_5020,     # ~0.007219
        ),
        "waist": ImplicitActuatorCfg(
            joint_names_expr=["waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint"],
            effort_limit_sim={"waist_yaw_joint": 88.0, "waist_roll_joint": 50.0, "waist_pitch_joint": 50.0},
            velocity_limit_sim={"waist_yaw_joint": 32.0, "waist_roll_joint": 37.0, "waist_pitch_joint": 37.0},
            stiffness={
                "waist_yaw_joint": _STIFFNESS_7520_14,       # ~40.18
                "waist_roll_joint": 2.0 * _STIFFNESS_5020,   # ~28.50
                "waist_pitch_joint": 2.0 * _STIFFNESS_5020,
            },
            damping={
                "waist_yaw_joint": _DAMPING_7520_14,         # ~2.56
                "waist_roll_joint": 2.0 * _DAMPING_5020,     # ~1.81
                "waist_pitch_joint": 2.0 * _DAMPING_5020,
            },
            armature={
                "waist_yaw_joint": _ARMATURE_7520_14,
                "waist_roll_joint": 2.0 * _ARMATURE_5020,
                "waist_pitch_joint": 2.0 * _ARMATURE_5020,
            },
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
                ".*_shoulder_pitch_joint": 25.0,
                ".*_shoulder_roll_joint": 25.0,
                ".*_shoulder_yaw_joint": 25.0,
                ".*_elbow_joint": 25.0,
                ".*_wrist_roll_joint": 25.0,
                ".*_wrist_pitch_joint": 5.0,
                ".*_wrist_yaw_joint": 5.0,
            },
            velocity_limit_sim={
                ".*_shoulder_pitch_joint": 37.0,
                ".*_shoulder_roll_joint": 37.0,
                ".*_shoulder_yaw_joint": 37.0,
                ".*_elbow_joint": 37.0,
                ".*_wrist_roll_joint": 37.0,
                ".*_wrist_pitch_joint": 22.0,
                ".*_wrist_yaw_joint": 22.0,
            },
            stiffness={
                ".*_shoulder_pitch_joint": _STIFFNESS_5020,
                ".*_shoulder_roll_joint": _STIFFNESS_5020,
                ".*_shoulder_yaw_joint": _STIFFNESS_5020,
                ".*_elbow_joint": _STIFFNESS_5020,
                ".*_wrist_roll_joint": _STIFFNESS_5020,    # ~14.25
                ".*_wrist_pitch_joint": _STIFFNESS_4010,   # ~16.78
                ".*_wrist_yaw_joint": _STIFFNESS_4010,
            },
            damping={
                ".*_shoulder_pitch_joint": _DAMPING_5020,
                ".*_shoulder_roll_joint": _DAMPING_5020,
                ".*_shoulder_yaw_joint": _DAMPING_5020,
                ".*_elbow_joint": _DAMPING_5020,
                ".*_wrist_roll_joint": _DAMPING_5020,      # ~0.91
                ".*_wrist_pitch_joint": _DAMPING_4010,     # ~1.07
                ".*_wrist_yaw_joint": _DAMPING_4010,
            },
            armature={
                ".*_shoulder_pitch_joint": _ARMATURE_5020,
                ".*_shoulder_roll_joint": _ARMATURE_5020,
                ".*_shoulder_yaw_joint": _ARMATURE_5020,
                ".*_elbow_joint": _ARMATURE_5020,
                ".*_wrist_roll_joint": _ARMATURE_5020,
                ".*_wrist_pitch_joint": _ARMATURE_4010,
                ".*_wrist_yaw_joint": _ARMATURE_4010,
            },
        ),
    },
)


# The action scale / default pose / joint clamp for the joint-position residual action
# (q_des = default_q + action * scale) load from the ONE shared adapter config,
# g1_deploy/g1_deploy_example/config/action_adapter.yaml, via
# whole_body_tracking.utils.action_adapter_config — the same file the G1 deploy runner reads.
