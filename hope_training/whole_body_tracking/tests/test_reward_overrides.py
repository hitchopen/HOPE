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
    for targets in reward_overrides.STRING_LIST_PARAM_SPECS.values():
        for term, param in targets:
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
    for path in (_ROOT / "cfg/task").glob("*.yaml"):
        keys = set(yaml.safe_load(path.read_text()).get("rewards", {}))
        assert keys <= set(reward_overrides.SUPPORTED_REWARD_KEYS), (path.name, keys - set(reward_overrides.SUPPORTED_REWARD_KEYS))


def test_v11_recipe_reaches_registered_parameters():
    rewards = _fake_rewards()
    for name in ("HOPEPingPongHoldReference", "HOPEPingPongCompleteHold"):
        config = yaml.safe_load((_ROOT / f"cfg/task/{name}.yaml").read_text())["rewards"]
        reward_overrides.apply_reward_overrides(rewards, config, [])
    assert rewards.motion_global_anchor_ori.params['exclude_recovery'] is True
    assert rewards.full_body_hold.params['completion_blend'] == .5
    assert rewards.full_body_hold.params['completion_dwell_s'] == .3


def test_v10_recipe_reaches_registered_parameters():
    config = yaml.safe_load((_ROOT / "cfg/task/HOPEPingPongBalancedHold.yaml").read_text())["rewards"]
    rewards = _fake_rewards()
    reward_overrides.apply_reward_overrides(rewards, config, [])
    assert rewards.full_body_hold.params['positive_kernel'] is True
    assert rewards.full_body_hold.params['cost_scale'] == 1.8
    assert rewards.full_body_quiet.params['state_independent'] is False
    assert rewards.full_body_quiet.params['pose_independent'] is True
    assert rewards.full_body_quiet.params['start_steps'] == 384000
    assert rewards.full_body_quiet.weight == -.03


def test_stationary_recipe_reaches_registered_parameters():
    config = yaml.safe_load((_ROOT / "cfg/task/HOPEPingPongStationaryRally.yaml").read_text())["rewards"]
    rewards = _fake_rewards()
    reward_overrides.apply_reward_overrides(rewards, config, [])
    assert rewards.full_body_hold.weight == 1.7
    assert rewards.full_body_hold.params['pose_tail_weight'] == .15
    assert rewards.pre_strike_tilt.weight == -.5
    assert rewards.base_ang_vel_xy_rally.weight == -.3
    assert rewards.base_ang_vel_xy_rally.params['strike_exclusion_s'] == .12
    assert rewards.dynamic_support.weight == -2.5
    assert rewards.foot_friction_cone.weight == -.15


def test_v7_ready_recipe_reaches_registered_terms():
    config = yaml.safe_load((_ROOT / "cfg/task/HOPEPingPongReadyRecovery.yaml").read_text())["rewards"]
    rewards = _fake_rewards()
    reward_overrides.apply_reward_overrides(rewards, config, [])
    assert rewards.actual_ready_legs.weight == -.4
    assert rewards.station_recovery.params["position_margin"] == .03
    assert rewards.loaded_ankle_quiet.params["start_steps"] == 448000
    assert rewards.actual_task_pose.params["torso_std"] == .06


def test_quiet_posture_recipe_is_wired_to_reward_parameters() -> None:
    quiet = yaml.safe_load((_ROOT / "cfg/task/HOPEPingPongQuiet.yaml").read_text())["rewards"]
    rewards = _fake_rewards()
    reward_overrides.apply_reward_overrides(rewards, quiet, [])
    assert rewards.ready_torso_posture.weight == -0.4
    for key, value in quiet.items():
        if key.startswith("ready_torso_posture_") and not key.endswith("_weight"):
            assert rewards.ready_torso_posture.params[key.removeprefix("ready_torso_posture_")] == value


def test_complete_recipe_applies_to_registered_reward_terms() -> None:
    rewards = _fake_rewards()
    applied: list[str] = []
    reward_overrides.apply_reward_overrides(rewards, _recipe(), applied)

    assert rewards.racket_position.weight == 14.0
    assert rewards.racket_position.params["std"] == 0.15
    assert rewards.action_rate_l2.weight == 0.0
    assert rewards.executed_qdes_rate.weight == -0.1
    assert rewards.executed_qdes_rate.params["hold_multiplier"] == 4.0
    assert rewards.settled_leg_command.weight == 0.0
    assert rewards.settled_waist_pitch_tracking.weight == 0.0
    for term in (rewards.settled_leg_command, rewards.settled_waist_pitch_tracking):
        assert term.params["start_steps"] == 448000
        assert term.params["ramp_steps"] == 128000
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


def test_task_reference_recipe_replaces_bias_penalties_and_clip_specific_hold():
    recipe = _recipe()
    for name in ("HOPEPingPongQuiet", "HOPEPingPongTaskReference"):
        recipe.update(yaml.safe_load((_ROOT / f"cfg/task/{name}.yaml").read_text())["rewards"])
    rewards = _fake_rewards()
    reward_overrides.apply_reward_overrides(rewards, recipe, [])
    assert rewards.hip_tracking.weight == rewards.settled_waist_pitch_tracking.weight == 0
    assert rewards.ready_torso_posture.weight == 0
    assert rewards.actual_task_pose.weight == -.4
    assert rewards.actual_joint_limit.weight == -1.
    assert rewards.actual_joint_limit.params['hold_recover_multiplier'] == 3.
    assert rewards.actual_joint_limit.params['waist_margin_rad'] > rewards.actual_joint_limit.params['margin_rad']
    assert rewards.hold_upper_pose_imitation.params['fixed_task_reference'] is True
    assert rewards.actual_task_pose.params['start_steps'] == 7000*64
    assert rewards.actual_task_pose.params['ramp_steps'] == (9000-7000)*64


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
