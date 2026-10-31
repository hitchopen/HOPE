import importlib.util
from pathlib import Path

spec = importlib.util.spec_from_file_location('planner_compat', Path(__file__).parents[1] / 'scripts' / 'pp_gate3_planner_compat.py')
mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)
check = mod.planner_version_independent_errors


def test_adaptive_commit_and_new_calibration_are_accepted():
    assert check('post_net_one_shot_physical_spin_magnus', 1.0, 1.327334, 0.38744443855065164) == []


def test_legacy_fixed_delay_remains_accepted():
    assert check('post_net_one_shot_effective_bounce_zero_spin', 1.0, 1.05, 0.369) == []


def test_spin_conditioned_bounce_is_accepted():
    assert check('post_net_one_shot_spin_conditioned_bounce', 1.0, 1.2, 0.4) == []


def test_invalid_evidence_still_rejected():
    assert check('unknown', 1.0, 1.3, 0.4)
    assert check('post_net_one_shot_physical_spin_magnus', 1.0, 0.99, 0.4)
    assert check('post_net_one_shot_physical_spin_magnus', 1.0, 1.3, 1.1)
    assert check('post_net_one_shot_physical_spin_magnus', 1.0, float('nan'), 0.4)
