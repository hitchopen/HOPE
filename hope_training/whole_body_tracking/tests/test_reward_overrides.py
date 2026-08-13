"""Host-only regression coverage for the complete HOPEPingPong reward recipe."""

from __future__ import annotations

import importlib.util
from pathlib import Path
from types import SimpleNamespace

import yaml


_ROOT = Path(__file__).resolve().parents[1]
_MODULE = (
    _ROOT
    / "source"
    / "whole_body_tracking"
    / "whole_body_tracking"
    / "utils"
    / "task_reward_overrides.py"
)
_SPEC = importlib.util.spec_from_file_location("hope_task_reward_overrides", _MODULE)
reward_overrides = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(reward_overrides)


def _recipe() -> dict:
    task = yaml.safe_load((_ROOT / "cfg" / "task" / "HOPEPingPong.yaml").read_text())
    return task["rewards"]


def _fake_rewards():
    params_by_term: dict[str, set[str]] = {}

    def add(term: str, param: str) -> None:
        params_by_term.setdefault(term, set()).add(param)

    for term in reward_overrides.WEIGHT_SPECS.values():
        params_by_term.setdefault(term, set())
    for term in reward_overrides.STD_SPECS.values():
        add(term, "std")
    for specs in (
        reward_overrides.FLOAT_PARAM_SPECS,
        reward_overrides.BOOL_PARAM_SPECS,
        reward_overrides.INT_PARAM_SPECS,
    ):
        for term, param in specs.values():
            add(term, param)
    for targets in reward_overrides.MULTI_FLOAT_PARAM_SPECS.values():
        for term, param in targets:
            add(term, param)
    for term, param, _allowed in reward_overrides.STRING_PARAM_SPECS.values():
        add(term, param)
    for term, param in reward_overrides.TUPLE_INT_PARAM_SPECS.values():
        add(term, param)

    add("rally_joint_qdes_saturation", "max_blend")
    add("waist_qdes_saturation", "std")
    add("waist_qdes_saturation", "max_blend")
    return SimpleNamespace(
        **{
            term: SimpleNamespace(
                weight=0.0, params={param: object() for param in sorted(params)}
            )
            for term, params in params_by_term.items()
        }
    )


def test_every_yaml_reward_key_has_an_explicit_mapping() -> None:
    assert set(_recipe()) == set(reward_overrides.SUPPORTED_REWARD_KEYS)


def test_complete_recipe_applies_to_registered_reward_terms() -> None:
    rewards = _fake_rewards()
    applied: list[str] = []
    reward_overrides.apply_reward_overrides(rewards, _recipe(), applied)

    assert rewards.racket_position.weight == 14.0
    assert rewards.racket_position.params["std"] == 0.15
    assert rewards.action_rate_l2.weight == -0.1
    assert rewards.joint_acc.weight == -2.5e-7
    assert rewards.rally_joint_qdes_saturation.weight == -0.65
    assert rewards.rally_joint_qdes_saturation.params["topk"] == 4
    assert rewards.rally_joint_qdes_saturation.params["topk_blend"] == 0.9
    assert "max_blend" not in rewards.rally_joint_qdes_saturation.params
    assert rewards.ready_deadline.params["target_step_classes"] == (1, 2, 3)
    assert rewards.ready_stance_width.params["station_reach"] == 0.1
    assert rewards.ready_foot_alignment.params["heading_gate"] == 0.15
    assert rewards.ready_leg_settle.params["station_reach"] == 0.1
    assert len(applied) >= sum(value is not None for value in _recipe().values())


def test_unknown_reward_key_fails_loudly() -> None:
    try:
        reward_overrides.apply_reward_overrides(
            _fake_rewards(), {"racket_position_weigth": 14.0}, []
        )
    except KeyError as error:
        assert "racket_position_weigth" in str(error)
    else:
        raise AssertionError("a misspelled reward key must not be silently ignored")


def test_missing_registered_term_fails_loudly() -> None:
    rewards = _fake_rewards()
    del rewards.racket_position
    try:
        reward_overrides.apply_reward_overrides(
            rewards, {"racket_position_weight": 14.0}, []
        )
    except AttributeError as error:
        assert "not registered" in str(error)
    else:
        raise AssertionError("a YAML reward without an EnvCfg term must fail")


def test_partial_override_does_not_remove_unmentioned_parameters() -> None:
    rewards = _fake_rewards()
    marker = rewards.rally_joint_qdes_saturation.params["max_blend"]
    reward_overrides.apply_reward_overrides(
        rewards, {"racket_position_weight": 12.0}, []
    )
    assert rewards.racket_position.weight == 12.0
    assert rewards.rally_joint_qdes_saturation.params["max_blend"] is marker
