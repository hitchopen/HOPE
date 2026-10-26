"""Position supervision must reward a real contact, including cold-start misses."""
import ast
from collections import defaultdict
import sys
from types import SimpleNamespace as NS

import pytest
import torch

from stationary_test_support import PKG, recipe, load


tracking = load('strike_position_kernel', PKG/'utils/task_tracking.py')
overrides = load('strike_position_overrides', PKG/'utils/task_reward_overrides.py')
source = PKG/'tasks/tracking/mdp/hope_rewards.py'
names = {'_cmd', '_position_guidance_gate', '_moving_position_kernel',
         'racket_position_tracking_exp', 'racket_exact_position_debt'}
scope = {'torch': torch, 'ManagerBasedRLEnv': object, 'RacketTargetCommand': object}
exec(compile(ast.Module(body=[n for n in ast.parse(source.read_text()).body
                             if getattr(n, 'name', None) in names], type_ignores=[]), str(source), 'exec'), scope)
debt = scope['racket_exact_position_debt']


def fixture(tts):
    n = len(tts)
    cmd = NS(time_to_strike=torch.tensor(tts), racket_pos_w=torch.zeros(n, 3),
             racket_target_pos_w=torch.zeros(n, 3), racket_target_vel_w=torch.zeros(n, 3),
             cfg=NS(position_guidance_window_s=0., position_guidance_temporal_scale=1.,
                    strike_window_s=.12, debug_reward_logging=False),
             metrics=defaultdict(lambda: torch.zeros(n)))
    cmd.strike_window = cmd.time_to_strike.abs() <= .12
    cmd.racket_pos_w[:, 2] = -.115
    return NS(command_manager=NS(get_term=lambda _: cmd)), cmd


def active_terms():
    cfg_source = PKG/'tasks/tracking/config/agibot_a3/hope_env_cfg.py'
    cls = next(n for n in ast.parse(cfg_source.read_text()).body
               if isinstance(n, ast.ClassDef) and n.name == 'HitterPingPongRewardsCfg')
    term = next(n for n in cls.body if isinstance(n, ast.Assign)
                and any(getattr(t, 'id', '') == 'racket_exact_position_debt' for t in n.targets))
    value = eval(compile(ast.Expression(term.value), str(cfg_source), 'eval'),
                 {'RewTerm': lambda **kwargs: NS(**kwargs), 'mdp': NS(racket_exact_position_debt=debt)})
    assert value.weight == 0.0  # legacy recipes stay inert
    rewards = NS(racket_exact_position_debt=value,
                 racket_position=NS(weight=14., params={'std': .15, 'tail_blend': 0.}))
    config = recipe()['task']['rewards']
    overrides.apply_reward_overrides(rewards, {k:v for k,v in config.items()
        if k.startswith(('racket_exact_position_debt_', 'racket_position_'))}, [])
    return rewards


def test_cold_start_tail_preserves_peak_and_gives_far_misses_a_score_signal():
    error = torch.tensor([0., .075, .15, .30, .40, .60], requires_grad=True)
    old = tracking.tracking_kernel(error.square(), .15)
    new = tracking.tracking_kernel(error.square(), .15, .20)
    assert old[-1] < 2e-7 and new[-1] > .01
    assert new[0] == old[0] == 1 and new.le(1).all()
    assert (new[1:] < new[:-1]).all()
    grad, = torch.autograd.grad(new.sum(), error)
    assert grad[1:].lt(0).all() and grad[-1].abs() > .03
    torch.testing.assert_close(tracking.tracking_kernel(error.square(), .15, 0), old)


def test_actual_contact_debt_is_static_and_only_on_the_exact_metric_frame():
    terms = active_terms()
    env, cmd = fixture([.02, .01, 0., -.01, -.02])
    value = terms.racket_exact_position_debt.func(env, **terms.racket_exact_position_debt.params)
    mask = cmd.time_to_strike.abs() <= .5*.02+1e-6
    assert torch.equal(value.gt(0), mask)
    cmd.racket_target_vel_w.fill_(8.)  # cannot move the exact-contact point
    torch.testing.assert_close(debt(env, **terms.racket_exact_position_debt.params), value)
    cmd.racket_pos_w[:, 2] = -.05
    assert debt(env, **terms.racket_exact_position_debt.params).eq(0).all()


def test_far_miss_correction_keeps_learning_signal_and_sides_share_threshold():
    env, cmd = fixture([0., 0.])
    cmd.swing_sign = torch.tensor([1., -1.])
    cmd.racket_pos_w[:, 2] = -.6
    cmd.racket_pos_w.requires_grad_()
    value = debt(env, 'racket_target', window_s=.01)
    torch.testing.assert_close(value[0], value[1])
    reward = -2.*value
    gradient, = torch.autograd.grad(reward.sum(), cmd.racket_pos_w)
    assert gradient[:, 2].gt(0).all()  # raising the low racket must improve reward
    assert gradient[:, 2].gt(70).all()


def instrumentation(cmd, terms):
    path = PKG/'tasks/tracking/mdp/hope_commands.py'
    cls = next(n for n in ast.parse(path.read_text()).body
               if isinstance(n, ast.ClassDef) and n.name == 'RacketTargetCommand')
    method = next(n for n in cls.body if isinstance(n, ast.FunctionDef)
                  and n.name == '_update_strike_local_reward_instrumentation')
    namespace = {'torch': torch}
    exec(compile(ast.Module(body=[method], type_ignores=[]), str(path), 'exec'), namespace)
    cmd._env = NS(step_dt=.02, reward_manager=NS(get_term_cfg=lambda key: getattr(terms, key)))
    cmd.device = 'cpu'
    cmd._strike_audit_size = 48
    n = len(cmd.time_to_strike)
    cmd._strike_audit_context_id = torch.tensor([0, 24])[:n]
    for key in ['_position_guidance_prev_active', '_exact_position_debt_prev_active']:
        setattr(cmd, key, torch.zeros(n, dtype=torch.bool))
    for key in ['_position_guidance_event_acc', '_exact_position_debt_event_acc']:
        setattr(cmd, key, torch.zeros(n))
    for key in ['_strike_local_position_event_count', '_strike_local_position_event_sum',
                '_strike_local_exact_debt_event_count', '_strike_local_exact_debt_event_sum']:
        setattr(cmd, key, torch.zeros(48))
    return lambda: namespace['_update_strike_local_reward_instrumentation'](cmd)


def test_legacy_strike_window_logs_real_debt_and_blended_position_returns(monkeypatch):
    monkeypatch.setitem(sys.modules, 'whole_body_tracking.utils.task_tracking', tracking)
    terms = active_terms()
    env, cmd = fixture([0., 0.])
    update = instrumentation(cmd, terms)
    update()
    raw = debt(env, **terms.racket_exact_position_debt.params)
    torch.testing.assert_close(cmd.metrics['exact_position_debt_raw'], raw)
    torch.testing.assert_close(cmd.metrics['exact_position_debt_weighted'], terms.racket_exact_position_debt.weight*raw)
    assert cmd.metrics['exact_position_debt_active_forehand'].eq(1).all()
    assert cmd.metrics['exact_position_debt_active_backhand'].eq(1).all()
    expected_pos = terms.racket_position.weight*.02*tracking.tracking_kernel(
        torch.full((2,), .115**2), terms.racket_position.params['std'],
        terms.racket_position.params['tail_blend'])
    torch.testing.assert_close(cmd._position_guidance_event_acc, expected_pos)
    cmd.time_to_strike.fill_(-.02)
    update()
    torch.testing.assert_close(cmd.metrics['exact_position_debt_event_return'], .02*terms.racket_exact_position_debt.weight*raw)


def test_inactive_legacy_recipe_does_not_emit_new_reward_telemetry():
    terms = active_terms()
    terms.racket_exact_position_debt.weight = 0.
    _, cmd = fixture([0., 0.])
    instrumentation(cmd, terms)()
    assert not cmd.metrics


def test_new_reward_identity_keeps_balanced_algorithm_guard_and_same_question_bank():
    cfg = recipe()['task']
    assert cfg['build_contract']['reward_revision'] == 'actual_complete_hold_v11_stationary_reach_v4'
    assert cfg['racket']['strike_success_pos_thresh'] == .075
    assert cfg['racket']['venue_tuple_bank_sha256'] == '02619e747333980a6f1d48aa3a9c7896e15727cf95cd3f51eb387d7687c4b419'
    guard = load('strike_position_balanced', PKG/'utils/balanced_hold_contract.py')
    with pytest.raises(ValueError, match='direct temporal actor refinement'):
        guard.validate_balanced_hold_algorithm(cfg['build_contract']['reward_revision'], {'settled_aux_enabled':True})
