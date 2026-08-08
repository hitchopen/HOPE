"""Agibot A3 — plain BeyondMimic motion tracking (baseline, no racket target).

This mirrors ``config/g1/flat_env_cfg.py`` for the A3. Train this first to confirm the A3 model,
joint order, PD gains, and reference-motion replay before adding the HOPE racket objective
(see ``hope_env_cfg.py``).
"""

from isaaclab.managers import SceneEntityCfg
from isaaclab.utils import configclass

from whole_body_tracking.robots.agibot_a3 import (
    A3_ANCHOR_BODY,
    A3_FEET_BODIES,
    A3_HAND_BODIES,
    A3_TRACKED_BODIES,
    AGIBOT_A3_ACTION_SCALE,
    AGIBOT_A3_CFG,
)
from whole_body_tracking.tasks.tracking.tracking_env_cfg import TrackingEnvCfg

# Contact bodies that are allowed to touch (feet) or that carry the paddle (wrist + racket links);
# the undesired-contact penalty excludes them via this negative-lookahead regex.
A3_CONTACT_EXCLUDE_REGEX = (
    r"^(?!left_ankle_roll_Link$)(?!right_ankle_roll_Link$)"
    r"(?!left_wrist_yaw_Link$)(?!right_wrist_yaw_Link$)"
    r"(?!right_hand_pingpang_Link$)(?!pingpang_red_Link$)(?!pingpang_black_Link$)(?!pingbang_ball_Link$).+$"
)


@configclass
class AgibotA3FlatEnvCfg(TrackingEnvCfg):
    def __post_init__(self):
        super().__post_init__()

        self.scene.robot = AGIBOT_A3_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
        self.actions.joint_pos.scale = AGIBOT_A3_ACTION_SCALE

        # Motion command: anchor + tracked bodies for the A3.
        self.commands.motion.anchor_body_name = A3_ANCHOR_BODY
        self.commands.motion.body_names = A3_TRACKED_BODIES

        # Retarget G1-specific body names in the base config to A3 names.
        self.events.base_com.params["asset_cfg"] = SceneEntityCfg("robot", body_names=A3_ANCHOR_BODY)
        self.rewards.undesired_contacts.params["sensor_cfg"] = SceneEntityCfg(
            "contact_forces", body_names=[A3_CONTACT_EXCLUDE_REGEX]
        )
        # Some deploy-honest tasks deliberately remove reference-relative termination and keep
        # only physical tilt/height falls. Respect that opt-out during subclass __post_init__.
        if self.terminations.ee_body_pos is not None:
            self.terminations.ee_body_pos.params["body_names"] = A3_FEET_BODIES + A3_HAND_BODIES
