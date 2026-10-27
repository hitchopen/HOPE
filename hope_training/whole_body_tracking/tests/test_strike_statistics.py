"""Same-tick EMA increments and host-transfer regression coverage."""
from __future__ import annotations

import ast
from collections import defaultdict
import importlib.util
import math
from pathlib import Path
import textwrap
from types import SimpleNamespace

import pytest
import torch


_ROOT = Path(__file__).resolve().parents[1] / "source/whole_body_tracking/whole_body_tracking/tasks/tracking/mdp"
_SPEC = importlib.util.spec_from_file_location("hope_strike_statistics", _ROOT / "strike_statistics.py")
stats = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(stats)


def _inputs(dtype=torch.float32, empty=False):
    exact = torch.tensor([1, 0, 1, 1, 0, 1], dtype=torch.bool)
    if empty:
        exact.zero_()
    position = torch.tensor([.03, .06, .04, .08, .10, .15], dtype=dtype)
    velocity = torch.tensor([.1, .2, .8, .2, .9, .3], dtype=dtype)
    normal = torch.tensor([1., 3., 20., 30., 6., 7.], dtype=dtype)
    pos = exact & (position < .075)
    vel = exact & (velocity < .5)
    nrm = exact & (normal < 15.)
    return dict(
        exact_strike=exact, pass_position=pos, pass_velocity=vel, pass_normal=nrm,
        pass_composite=pos & vel & nrm, position_error=position, velocity_error=velocity,
        normal_error_deg=normal, in_window=torch.tensor([1, 1, 1, 0, 0, 1], dtype=torch.bool),
        clip_ids=torch.tensor([2, 7, 2, 7, 2, 7]), clip_keys=(2, 7),
        venue_selected=torch.tensor([1, 0, 0, 1, 0, 0], dtype=torch.bool), cohort_names=("tuple", "free"),
    )


@pytest.mark.parametrize("dtype", [torch.float32, torch.float64])
@pytest.mark.parametrize("empty", [False, True])
def test_all_43_increments_equal_original_reductions(dtype, empty):
    data = _inputs(dtype, empty)
    actual = stats.collect_exact_strike_statistics(**data)
    exact = data["exact_strike"]
    expected = {}
    scopes = [("global", exact)] + [("clip", exact & (data["clip_ids"] == clip), clip) for clip in data["clip_keys"]]
    for scope in scopes:
        kind, selected, *clip = scope
        prefix = (kind, *clip)
        expected[prefix + ("n",)] = float(selected.sum())
        for name, key in (("pos", "pass_position"), ("vel", "pass_velocity"), ("normal", "pass_normal"), ("comp", "pass_composite")):
            expected[prefix + (name,)] = float((data[key] & selected).sum())
        mask = selected if kind == "global" else selected.float()
        for name, key in (("pos_err", "position_error"), ("vel_err", "velocity_error")):
            expected[prefix + (name,)] = float((data[key] * mask).sum())
        if kind == "clip":
            expected[prefix + ("normal_err",)] = float((data["normal_error_deg"] * mask).sum())
            for cohort in data["cohort_names"]:
                subset = selected & (data["venue_selected"] if cohort == "tuple" else ~data["venue_selected"])
                cohort_prefix = ("cohort", cohort, clip[0])
                expected[cohort_prefix + ("n",)] = float(subset.sum())
                for name, key in (("pos", "pass_position"), ("vel", "pass_velocity"), ("comp", "pass_composite")):
                    expected[cohort_prefix + (name,)] = float((data[key] & subset).sum())
    expected[("global", "5cm")] = float(((data["position_error"] < .05) & exact).sum())
    expected[("global", "10cm")] = float(((data["position_error"] < .10) & exact).sum())
    expected[("global", "window_n")] = float(data["in_window"].sum())
    expected[("global", "window_normal_err")] = float((data["normal_error_deg"] * (math.pi / 180.) * data["in_window"]).sum())
    assert len(actual) == 43
    assert actual == expected
    assert all(type(value) is float for value in actual.values())


def test_single_clip_skips_breakdowns_and_has_one_host_transfer(monkeypatch):
    data = _inputs()
    data["clip_ids"] = None
    calls = []
    original = torch.Tensor.cpu
    def capture(value, *args, **kwargs):
        calls.append(tuple(value.shape))
        return original(value, *args, **kwargs)
    monkeypatch.setattr(torch.Tensor, "cpu", capture)
    result = stats.collect_exact_strike_statistics(**data)
    assert len(result) == 11 and calls == [(11,)]
    calls.clear()
    stats.collect_exact_strike_statistics(**_inputs())
    assert calls == [(43,)]


def test_production_block_keeps_ema_gate_and_virtual_physical_order():
    source = (_ROOT / "hope_commands.py").read_text()
    marker = source.index("        # Pack read-only scalar reductions once;")
    start = source.rfind("        decay = float(self.cfg.exact_success_decay)", 0, marker)
    end = source.index("        # Per-axis position error AT the exact strike frame", marker)
    block = ast.parse(textwrap.dedent(source[start:end]))
    command = SimpleNamespace()
    # Every inherited scalar starts nonzero so unchanged decay/update order is observable.
    for node in ast.walk(block):
        if isinstance(node, ast.Attribute) and isinstance(node.value, ast.Name) and node.value.id == "self":
            setattr(command, node.attr, .7)
        if isinstance(node, ast.Subscript) and isinstance(node.value, ast.Attribute) and isinstance(node.value.value, ast.Name) and node.value.value.id == "self":
            setattr(command, node.value.attr, defaultdict(lambda: .7))
    # A second pass avoids root Attribute nodes overwriting dictionaries above.
    for node in ast.walk(block):
        if isinstance(node, ast.Subscript) and isinstance(node.value, ast.Attribute) and isinstance(node.value.value, ast.Name) and node.value.value.id == "self":
            setattr(command, node.value.attr, defaultdict(lambda: .7))
    command.metrics = defaultdict(lambda: torch.zeros(6))
    command.cfg = SimpleNamespace(exact_success_decay=.9, exact_success_min_count=.5, virtual_ball=True, venue_tuple_unconditional_outcomes=True, achieved_target_mix_prob=0.)
    command._clip_names = {2: "fh", 7: "bh"}
    command._cohort_names = ("tuple", "free")
    command._exact_pos_signed_sum_c = defaultdict(lambda: torch.full((3,), .7))
    command._exact_vel_signed_sum_c = defaultdict(lambda: torch.full((3,), .7))
    command._exact_normal_dot_sum_c = defaultdict(lambda: torch.tensor(.7))
    data = _inputs()
    command._venue_tuple_selected = data["venue_selected"]
    command._motion = lambda: SimpleNamespace(_multiseg=True, clip_id=data["clip_ids"])
    snapshots = []
    def snapshot(name):
        snapshots.append((name, command._exact_n_acc, command._exact_pos_err_sum, command._exact_n_acc_c[2]))
    command._vb_evaluate = lambda *_: snapshot("virtual")
    command._physical = SimpleNamespace(update=lambda *_: snapshot("physical"))
    local = dict(self=command, torch=torch, math=math, collect_exact_strike_statistics=stats.collect_exact_strike_statistics,
                 exact_strike=data["exact_strike"], pass_comp=data["pass_composite"], pass_pos=data["pass_position"],
                 pass_vel=data["pass_velocity"], pass_normal=data["pass_normal"], pos_err=data["position_error"],
                 vel_err=data["velocity_error"], normal_err_deg=data["normal_error_deg"], in_win=data["in_window"],
                 racket_pos_err_vec=torch.ones(6, 3), racket_vel_err_vec=torch.ones(6, 3), cos_ang=torch.ones(6))
    rng = torch.get_rng_state()
    exec(compile(block, str(_ROOT / "hope_commands.py"), "exec"), local)
    assert snapshots == [("virtual", 4.63, .7, .7), ("physical", 4.63, .7, .7)]
    assert command._exact_n_acc_c[2] == 2.63
    assert command._cohort_exact_acc[("tuple", 2)] == 1.63
    # Global rates consume the new exact count; per-clip completion retains its preexisting
    # location before the per-clip exact EMA update (do not silently "fix" that lag here).
    torch.testing.assert_close(command.metrics["swing_completion_rate_fh"], torch.ones(6))
    torch.testing.assert_close(command.metrics["strike_pos_pass_exact_fh"], torch.ones(6))
    assert type(command._exact_n_acc) is float
    torch.testing.assert_close(torch.get_rng_state(), rng)


def test_bank_core_and_wide_remain_separate_when_every_question_is_a_tuple():
    data = _inputs()
    data['venue_selected'] = torch.ones(6, dtype=torch.bool)
    data['cohort_names'] = ('core', 'tuple')
    data['reach_wide_selected'] = torch.tensor([0, 0, 1, 0, 1, 1], dtype=torch.bool)
    result = stats.collect_exact_strike_statistics(**data)
    for clip in data['clip_keys']:
        assert result[('cohort', 'core', clip, 'n')] == 0
        assert sum(result[('reach', c, clip, 'n')] for c in ('core', 'wide')) == result[('clip', clip, 'n')]
        for cohort, wide in (('core', False), ('wide', True)):
            mask = data['exact_strike'] & (data['clip_ids'] == clip) & (data['reach_wide_selected'] == wide)
            assert result[('reach', cohort, clip, 'n')] == float(mask.sum())
            assert result[('reach', cohort, clip, 'pos')] == float((data['pass_position'] & mask).sum())
            assert result[('reach', cohort, clip, 'pos_err')] == float((data['position_error'] * mask).sum())


def _reach_update():
    tree = ast.parse((_ROOT / 'hope_commands.py').read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'RacketTargetCommand')
    method = next(n for n in cls.body if getattr(n, 'name', None) == '_update_reach_accuracy')
    scope = {'torch': torch}
    exec(compile(ast.Module(body=[method], type_ignores=[]), str(_ROOT / 'hope_commands.py'), 'exec'), scope)
    return scope['_update_reach_accuracy']


def test_fixed_mixture_score_does_not_decay_when_question_mix_changes():
    update = _reach_update()
    for core_n, wide_n in ((90, 10), (10, 90)):
        cmd = SimpleNamespace(cfg=SimpleNamespace(exact_success_min_count=1),
                              num_envs=3, device='cpu', _clip_names={0: 'forehand'},
                              metrics={}, _reach_accuracy_acc=defaultdict(float))
        result = {}
        for cohort, n, passes in (('core', core_n, core_n), ('wide', wide_n, 0)):
            for channel, value in (('n', n), ('pos', passes), ('pos_err', n * .05)):
                result[('reach', cohort, 0, channel)] = float(value)
        rng = torch.get_rng_state()
        update(cmd, result, .9)
        torch.testing.assert_close(cmd.metrics['reach_balanced_pos_pass_forehand'], torch.full((3,), .5))
        assert cmd.metrics['reach_balanced_valid_forehand'].eq(1).all()
        # Log buffers can be reset without erasing the source counters.
        for value in cmd.metrics.values(): value.zero_()
        update(cmd, dict.fromkeys(result, 0.), .9)
        assert cmd.metrics['reach_core_pos_pass_forehand'].eq(1).all()
        torch.testing.assert_close(torch.get_rng_state(), rng)


def test_fixed_mixture_score_marks_absent_wide_evidence_invalid():
    cmd = SimpleNamespace(cfg=SimpleNamespace(exact_success_min_count=1),
                          num_envs=1, device='cpu', _clip_names={0: 'backhand'},
                          metrics={}, _reach_accuracy_acc=defaultdict(float))
    result = {('reach', c, 0, k): 0. for c in ('core', 'wide') for k in ('n', 'pos', 'pos_err')}
    result[('reach', 'core', 0, 'n')] = result[('reach', 'core', 0, 'pos')] = 10.
    _reach_update()(cmd, result, .9)
    assert cmd.metrics['reach_core_pos_pass_backhand'].item() == 1
    assert cmd.metrics['reach_wide_valid_backhand'].item() == 0
    assert cmd.metrics['reach_balanced_valid_backhand'].item() == 0


@pytest.mark.parametrize('std', [.075, .15, .22])
def test_position_sigma_telemetry_reports_the_live_reward_width(std):
    tree = ast.parse((_ROOT / 'hope_commands.py').read_text())
    node = next(n for n in ast.walk(tree) if isinstance(n, ast.Assign)
                and any(isinstance(t, ast.Subscript) and 'position_sigma_current' in ast.unparse(t)
                        for t in n.targets))
    cmd = SimpleNamespace(metrics={'position_sigma_current': torch.zeros(2)},
                          _adaptive_sigma_pos=.20,
                          _env=SimpleNamespace(reward_manager=SimpleNamespace(
                              get_term_cfg=lambda _: SimpleNamespace(params={'std': std}))))
    exec(compile(ast.Module(body=[node], type_ignores=[]), 'sigma_telemetry', 'exec'), {'self': cmd})
    torch.testing.assert_close(cmd.metrics['position_sigma_current'], torch.full((2,), std))


def test_reach_accuracy_metrics_are_uploaded_under_the_active_wandb_profile():
    path = _ROOT.parents[2] / 'utils/wandb_logging.py'
    spec = importlib.util.spec_from_file_location('reach_wandb_logging', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    for side in ('forehand', 'backhand'):
        for cohort in ('core', 'wide', 'balanced'):
            for metric in ('valid', 'pos_pass'):
                assert module.wandb_metric_allowed(f'Live/racket_target/reach_{cohort}_{metric}_{side}', 'hitter_pingpong_v1')
