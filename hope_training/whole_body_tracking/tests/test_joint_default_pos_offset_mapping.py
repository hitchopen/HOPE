"""Regression test for joint-default randomization offset scattering.

The event receives articulation joint ids, but the HOPE action term uses the
deploy-canonical action order. Updating ``action_term._offset`` with articulation
ids permutes q_des when Isaac imports the A3 joints in a different order.
"""

from __future__ import annotations

import importlib.util
import os
import sys
import types
from types import SimpleNamespace

import torch

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_EVENTS = os.path.join(
    _ROOT,
    "source",
    "whole_body_tracking",
    "whole_body_tracking",
    "tasks",
    "tracking",
    "mdp",
    "events.py",
)


def _install_stubs() -> None:
    if "isaaclab" not in sys.modules:
        sys.modules["isaaclab"] = types.ModuleType("isaaclab")
    assets = types.ModuleType("isaaclab.assets")
    assets.Articulation = object
    sys.modules["isaaclab.assets"] = assets
    managers = types.ModuleType("isaaclab.managers")
    managers.SceneEntityCfg = object
    sys.modules["isaaclab.managers"] = managers
    utils = types.ModuleType("isaaclab.utils")
    math_utils = types.ModuleType("isaaclab.utils.math")
    sys.modules["isaaclab.utils"] = utils
    sys.modules["isaaclab.utils.math"] = math_utils

    mdp_events = types.ModuleType("isaaclab.envs.mdp.events")

    def _randomize_prop_by_op(prop, params, env_ids, joint_ids, operation, distribution):
        assert operation == "add"
        out = prop.clone()
        delta = float(params[0])
        if isinstance(joint_ids, slice):
            out[env_ids] += delta
        else:
            out[env_ids[:, None], joint_ids] += delta
        return out

    mdp_events._randomize_prop_by_op = _randomize_prop_by_op
    sys.modules["isaaclab.envs"] = types.ModuleType("isaaclab.envs")
    sys.modules["isaaclab.envs.mdp"] = types.ModuleType("isaaclab.envs.mdp")
    sys.modules["isaaclab.envs.mdp.events"] = mdp_events


def _load_events_module():
    _install_stubs()
    spec = importlib.util.spec_from_file_location("hope_tracking_events_for_test", _EVENTS)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


class _FakeScene(SimpleNamespace):
    def __getitem__(self, name):
        return getattr(self, name)


def test_joint_default_pos_updates_action_offset_by_action_column_order():
    events = _load_events_module()

    # Articulation order differs from action/canonical order.
    asset_joint_names = ["hip", "waist", "knee"]
    action_joint_names = ["waist", "knee", "hip"]
    default_joint_pos = torch.tensor(
        [
            [10.0, 20.0, 30.0],
            [11.0, 21.0, 31.0],
        ]
    )
    asset = SimpleNamespace(
        device="cpu",
        data=SimpleNamespace(
            joint_names=asset_joint_names,
            default_joint_pos=default_joint_pos.clone(),
        ),
    )
    action_term = SimpleNamespace(
        _joint_names=action_joint_names,
        _offset=torch.zeros(2, 3),
    )
    env = SimpleNamespace(
        scene=_FakeScene(num_envs=2, robot=asset),
        action_manager=SimpleNamespace(get_term=lambda name: action_term),
    )
    asset_cfg = SimpleNamespace(name="robot", joint_ids=[1, 2, 0])

    events.randomize_joint_default_pos(
        env,
        env_ids=torch.tensor([0, 1]),
        asset_cfg=asset_cfg,
        pos_distribution_params=(1.0, 1.0),
        operation="add",
        distribution="uniform",
    )

    # Asset columns are still articulation order.
    assert torch.allclose(asset.data.default_joint_pos, default_joint_pos + 1.0)
    # Action columns are action/canonical order: waist, knee, hip.
    assert torch.allclose(
        action_term._offset,
        torch.tensor(
            [
                [21.0, 31.0, 11.0],
                [22.0, 32.0, 12.0],
            ]
        ),
    )
