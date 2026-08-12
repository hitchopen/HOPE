"""Agibot A3 specialization of the table-tennis environment.

Drops the official Agibot A3 ping-pong articulation into the HOPE table-tennis scene, standing on the
P1 side and facing P2, and wires up the A3 per-joint action scale. Everything else (scene, ball
aerodynamics, observations, rewards, events, terminations) is inherited from
:class:`~whole_body_tracking.tasks.table_tennis.table_tennis_env_cfg.TableTennisEnvCfg`.
"""

from __future__ import annotations

import copy

from isaaclab.utils import configclass

from whole_body_tracking.robots.agibot_a3 import AGIBOT_A3_ACTION_SCALE, AGIBOT_A3_CFG
from whole_body_tracking.tasks.table_tennis import geometry
from whole_body_tracking.tasks.table_tennis.table_tennis_env_cfg import TableTennisEnvCfg

# Pelvis height above the floor in the A3 standing keyframe (= AGIBOT_A3_CFG init z).
A3_STAND_PELVIS_HEIGHT: float = float(AGIBOT_A3_CFG.init_state.pos[2])


@configclass
class AgibotA3TableTennisEnvCfg(TableTennisEnvCfg):
    def __post_init__(self):
        super().__post_init__()

        # Deep-copy so we never mutate the shared global AGIBOT_A3_CFG (its init_state is reused
        # by the WBC tracking configs).
        robot = copy.deepcopy(AGIBOT_A3_CFG)
        robot.prim_path = "{ENV_REGEX_NS}/Robot"
        # Stand at the P1 side, on the floor (HOPE z = -0.76), facing +X toward P2.
        robot.init_state.pos = (
            geometry.P1_STAND_X,
            geometry.P1_STAND_Y,
            geometry.FLOOR_Z + A3_STAND_PELVIS_HEIGHT,
        )
        # Identity orientation = facing +X (toward P2). If the A3 URDF forward axis turns out to be
        # -X, set this to (0.0, 0.0, 0.0, 1.0) (180 deg about Z).
        robot.init_state.rot = (1.0, 0.0, 0.0, 0.0)
        self.scene.robot = robot

        # Per-joint action scale (0.25 * effort / stiffness), matching the A3 deploy decoder.
        self.actions.joint_pos.scale = AGIBOT_A3_ACTION_SCALE
