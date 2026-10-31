"""Run the transaction regressions against the separately pinned P1 BSP."""
import importlib.util
import pytest
from foxglove.tests import test_elink_scheduler as regression

spec = importlib.util.spec_from_file_location(
    'scheduler_p1', regression.ROOT / 'helpers/hope_elink_scheduler_p1_20261030.py')
policy = importlib.util.module_from_spec(spec)
spec.loader.exec_module(policy)


@pytest.mark.parametrize('case', [
    'test_apply_and_restore_preserve_original_across_repeated_apply',
    'test_priority_failure_restores_affinity_and_previously_changed_threads',
    'test_reused_tid_is_never_mutated',
    'test_concurrent_policy_change_is_preserved',
    'test_readback_mismatch_rolls_back',
    'test_stale_receipt_rejected',
    'test_unknown_model_or_build_fails_before_process_selection',
])
def test_p1_transaction_regression(monkeypatch, tmp_path, case):
    monkeypatch.setattr(regression, 'policy', policy)
    getattr(regression, case)(monkeypatch, tmp_path)


def test_p1_profile_rejects_t3_helper(tmp_path):
    source = (regression.ROOT / 'helpers/hope-lifecycle').read_text()
    path = tmp_path / 'hope-lifecycle'
    path.write_text(source)
    with pytest.raises(RuntimeError, match='hardware profile differs'):
        policy.validate_lifecycle_profile(path)
    path.write_text(source.replace('A3_T3D0', 'A3_P1D0')
                    .replace('hal_ethercat_a3_t3d0_cfg.yaml', 'hal_ethercat_a3_p1d0_cfg.yaml'))
    policy.validate_lifecycle_profile(path)
