"""Unitree G1 specialization of the table-tennis environment (parallel to the A3 twin).

Drops the G1 articulation into the table-tennis scene, standing on the P1 side and facing P2, wires
up the shared G1 action adapter (scale + default pose), and sets the measured paddle contact
material on the racket link. Everything else (scene, ball drag, observations, rewards, events,
terminations) is inherited from
:class:`~whole_body_tracking.tasks.table_tennis.table_tennis_env_cfg.TableTennisEnvCfg`.
"""

from __future__ import annotations

import copy

from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.utils import configclass

import whole_body_tracking.tasks.table_tennis.mdp as mdp
from whole_body_tracking.robots.g1 import G1_CFG, G1_RACKET_BODY, G1_WRIST_BODY
from whole_body_tracking.tasks.table_tennis import geometry
from whole_body_tracking.tasks.table_tennis.geometry import BounceMaterials
from whole_body_tracking.tasks.table_tennis.table_tennis_env_cfg import TableTennisEnvCfg
from whole_body_tracking.utils.action_adapter_config import load_g1_action_adapter_config

# Pelvis height above the floor in the G1 standing keyframe (= G1_CFG init z).
G1_STAND_PELVIS_HEIGHT: float = float(G1_CFG.init_state.pos[2])


@configclass
class G1TableTennisEnvCfg(TableTennisEnvCfg):
    def __post_init__(self):
        super().__post_init__()

        # Deep-copy so we never mutate the shared global G1_CFG.
        robot = copy.deepcopy(G1_CFG)
        robot.prim_path = "{ENV_REGEX_NS}/Robot"
        # Stand at the P1 side, on the floor (world z = -TABLE_HEIGHT), facing +X toward P2.
        robot.init_state.pos = (
            geometry.P1_STAND_X,
            geometry.P1_STAND_Y,
            geometry.FLOOR_Z + G1_STAND_PELVIS_HEIGHT,
        )
        # Identity orientation = facing +X (toward P2). If the G1 URDF forward axis is -X, set this to
        # (0.0, 0.0, 0.0, 1.0) (180 deg about Z).
        robot.init_state.rot = (1.0, 0.0, 0.0, 0.0)
        self.scene.robot = robot

        # Shared G1 action adapter (same file the deploy runner reads): action scale + default pose.
        adapter = load_g1_action_adapter_config()
        robot.init_state.joint_pos = adapter.default_q_by_name()
        self.actions.joint_pos.scale = adapter.action_scale_by_name()

        # Racket-face contact material. The alternation resolves to whichever body exists: the racket
        # body when the USD keeps it, else the merged wrist body carrying the blade shapes (same
        # fallback the racket-FK code in hope_commands uses).
        mats = BounceMaterials()
        self.events.racket_material = EventTerm(
            func=mdp.randomize_rigid_body_material,
            mode="startup",
            params={
                "asset_cfg": SceneEntityCfg("robot", body_names=f"({G1_RACKET_BODY}|{G1_WRIST_BODY})"),
                "static_friction_range": (mats.paddle_static_friction, mats.paddle_static_friction),
                "dynamic_friction_range": (mats.paddle_dynamic_friction, mats.paddle_dynamic_friction),
                "restitution_range": (mats.paddle_restitution, mats.paddle_restitution),
                "num_buckets": 1,
            },
        )
