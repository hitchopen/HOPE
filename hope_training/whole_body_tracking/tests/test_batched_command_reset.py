"""Compare production batched reset with IsaacLab's sequential reset semantics."""
from __future__ import annotations

import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace

import pytest
import torch


_PATH = Path(__file__).resolve().parents[1] / "source/whole_body_tracking/whole_body_tracking/tasks/tracking/command_reset.py"
_SPEC = importlib.util.spec_from_file_location("hope_command_reset", _PATH)
reset = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(reset)


def _term(metrics):
    term = SimpleNamespace(metrics=metrics, command_counter=torch.arange(4) + 1, calls=[])
    def resample(ids):
        term.calls.append((term.command_counter.clone(), {k: v.clone() for k, v in metrics.items()}))
        # Resampling uses randomness and mutates reset rows; batching must not reorder this.
        term.command_counter[ids] = torch.randint(10, 20, term.command_counter[ids].shape)
    term._resample = resample
    return term


def _upstream(term, env_ids):
    ids = slice(None) if env_ids is None else env_ids
    extras = {}
    for key, value in term.metrics.items():
        extras[key] = torch.mean(value[ids]).item()
        value[ids] = 0.0
    term.command_counter[ids] = 0
    term._resample(ids)
    return extras


def _assert_same_metrics(actual, expected):
    assert list(actual) == list(expected)
    for key in actual:
        torch.testing.assert_close(actual[key], expected[key], rtol=0, atol=0, equal_nan=True)


@pytest.mark.parametrize("env_ids", [None, [1, 3], torch.tensor([3, 1]), torch.tensor([], dtype=torch.long)])
def test_reset_means_rows_counter_resample_and_rng_match_upstream(env_ids):
    def metrics():
        return {
            "position": torch.arange(4, dtype=torch.float32),
            "double": torch.tensor([1.0000000001, 2.0000000002, 3.0000000003, 4.0000000004], dtype=torch.float64),
            "matrix": torch.arange(12, dtype=torch.float32).reshape(3, 4).t(),
            "nonfinite": torch.tensor([2.0, float("nan"), 4.0, float("inf")]),
        }
    expected, actual = _term(metrics()), _term(metrics())
    torch.manual_seed(123)
    wanted = _upstream(expected, env_ids)
    expected_rng = torch.get_rng_state()
    torch.manual_seed(123)
    got = reset.batched_command_term_reset(actual, env_ids)
    assert list(got) == list(wanted)
    for key in got:
        assert got[key] == wanted[key] or (math.isnan(got[key]) and math.isnan(wanted[key]))
    _assert_same_metrics(actual.metrics, expected.metrics)
    torch.testing.assert_close(actual.command_counter, expected.command_counter)
    torch.testing.assert_close(actual.calls[0][0], expected.calls[0][0])
    _assert_same_metrics(actual.calls[0][1], expected.calls[0][1])
    torch.testing.assert_close(torch.get_rng_state(), expected_rng)


def test_nonfinite_rows_are_really_zeroed_not_multiplied_by_zero():
    term = _term({"risk": torch.tensor([1.0, float("nan"), float("inf"), -float("inf")])})
    reset.batched_command_term_reset(term, [1, 2, 3])
    torch.testing.assert_close(term.metrics["risk"], torch.tensor([1.0, 0.0, 0.0, 0.0]))


def test_aliased_metrics_preserve_upstreams_sequential_clear_order():
    def metrics():
        value = torch.arange(4, dtype=torch.float32)
        return {"first": value, "alias": value.view(4)}
    expected, actual = _term(metrics()), _term(metrics())
    assert reset.batched_command_term_reset(actual, [1, 3]) == _upstream(expected, [1, 3])
    assert actual.metrics["first"].data_ptr() == actual.metrics["alias"].data_ptr()


def test_many_metrics_use_one_host_transfer_per_group(monkeypatch):
    term = _term({str(i): torch.ones(4) * i for i in range(128)})
    transfers = []
    original_cpu = torch.Tensor.cpu
    def tracked_cpu(value, *args, **kwargs):
        transfers.append(tuple(value.shape))
        return original_cpu(value, *args, **kwargs)
    monkeypatch.setattr(torch.Tensor, "cpu", tracked_cpu)
    reset.batched_command_term_reset(term, [1, 3])
    assert transfers == [(128,)]


@pytest.mark.parametrize("env_ids", [None, [1, 3], torch.tensor([], dtype=torch.long)])
def test_interleaved_overlapping_views_and_shapes_keep_sequential_semantics(env_ids):
    def metrics():
        a = torch.arange(16, dtype=torch.float32).reshape(4, 4)
        b = a.clone() + 10
        # Includes differing shapes and overlapping views; the final means depend on
        # earlier clears, even when those clears belong to another shape group.
        return {"a": a[:, 0], "a_again": a[:, 0], "b": b[:, :2],
                "a_matrix": a[:, :2], "b_again": b[:, 1], "a_last": a[:, 1]}
    expected, actual = _term(metrics()), _term(metrics())
    wanted = _upstream(expected, env_ids)
    got = reset.batched_command_term_reset(actual, env_ids)
    assert list(got) == list(wanted)
    for key in got:
        assert got[key] == wanted[key] or (math.isnan(got[key]) and math.isnan(wanted[key]))
    _assert_same_metrics(actual.metrics, expected.metrics)


def test_alias_does_not_force_independent_metrics_to_scalar_host_transfers(monkeypatch):
    metrics = {str(i): torch.ones(4) * i for i in range(128)}
    metrics["alias"] = metrics["5"].view(4)
    transfers = []
    original_cpu = torch.Tensor.cpu
    def tracked_cpu(value, *args, **kwargs):
        transfers.append(tuple(value.shape))
        return original_cpu(value, *args, **kwargs)
    def forbidden_item(*args, **kwargs):
        raise AssertionError("per-metric scalar GPU synchronization")
    monkeypatch.setattr(torch.Tensor, "cpu", tracked_cpu)
    monkeypatch.setattr(torch.Tensor, "item", forbidden_item)
    got = reset.batched_command_term_reset(_term(metrics), [1, 3])
    assert transfers == [(128,), (1,)]
    assert got["5"] == 5.0 and got["alias"] == 0.0
