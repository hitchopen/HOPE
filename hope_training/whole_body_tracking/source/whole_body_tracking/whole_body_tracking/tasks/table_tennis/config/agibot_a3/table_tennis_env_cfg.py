"""Agibot A3 specialization of the table-tennis environment.

Drops the Agibot A3 articulation into the table-tennis scene, standing on the P1 side and facing P2, and
wires up the A3 per-joint action scale. Everything else (scene, ball drag, observations, rewards, events,
terminations) is inherited from
:class:`~whole_body_tracking.tasks.table_tennis.table_tennis_env_cfg.TableTennisEnvCfg`.
"""

from __future__ import annotations

import copy

from isaaclab.utils import configclass

from whole_body_tracking.robots.agibot_a3 import AGIBOT_A3_CFG
from whole_body_tracking.tasks.table_tennis import geometry
from whole_body_tracking.tasks.table_tennis.table_tennis_env_cfg import TableTennisEnvCfg
from whole_body_tracking.utils.action_adapter_config import load_action_adapter_config

# Pelvis height above the floor in the A3 standing keyframe (= AGIBOT_A3_CFG init z).
A3_STAND_PELVIS_HEIGHT: float = float(AGIBOT_A3_CFG.init_state.pos[2])


@configclass
class AgibotA3TableTennisEnvCfg(TableTennisEnvCfg):
    def __post_init__(self):
        super().__post_init__()

        # Deep-copy so we never mutate the shared global AGIBOT_A3_CFG.
        robot = copy.deepcopy(AGIBOT_A3_CFG)
        robot.prim_path = "{ENV_REGEX_NS}/Robot"
        # Stand at the P1 side, on the floor (world z = -TABLE_HEIGHT), facing +X toward P2.
        robot.init_state.pos = (
            geometry.P1_STAND_X,
            geometry.P1_STAND_Y,
            geometry.FLOOR_Z + A3_STAND_PELVIS_HEIGHT,
        )
        # Identity orientation = facing +X (toward P2). If the A3 URDF forward axis is -X, set this to
        # (0.0, 0.0, 0.0, 1.0) (180 deg about Z).
        robot.init_state.rot = (1.0, 0.0, 0.0, 0.0)
        self.scene.robot = robot

        # Shared action adapter (same file the deploy runner reads): action scale + default pose.
        adapter = load_action_adapter_config()
        robot.init_state.joint_pos = adapter.default_q_by_name()
        self.actions.joint_pos.scale = adapter.action_scale_by_name()
