"""Fixed-station acceleration must preserve cohorts and the random stream."""
import pytest
import torch
from stationary_test_support import robust


@pytest.mark.parametrize('device', ['cpu', 'cuda'])
@pytest.mark.parametrize('probability', [0., .35, .85, 1.])
def test_fast_sampling_matches_general_path_and_next_random_draw(device, probability):
    if device == 'cuda' and not torch.cuda.is_available():
        pytest.skip('CUDA unavailable')
    bank = {'station_xy': torch.zeros(10007, 2, device=device),
            'source_kind': (torch.arange(10007, device=device) % 3 != 0).float()}
    pools = tuple(torch.where(bank['source_kind'] == c)[0] for c in (0, 1))
    previous = torch.zeros(257, 2, device=device)
    remaining = torch.ones(257, device=device)
    args = dict(speed=.6, brake_s=.2, wide_probability=probability)
    torch.manual_seed(712)
    # A zero station fits every bounded candidate: this is the unchanged
    # general sampler's exact row-order and RNG oracle for the fast path.
    expected = robust.sample_rows(bank, previous, remaining, **args)
    expected_next = torch.rand(20, device=device)
    torch.manual_seed(712)
    actual = robust.sample_rows(bank, previous, remaining, unbounded=True,
                                cohort_rows=pools, **args)
    actual_next = torch.rand(20, device=device)
    for a, b in zip(actual, expected):
        torch.testing.assert_close(a, b, rtol=0, atol=0)
    torch.testing.assert_close(actual_next, expected_next, rtol=0, atol=0)


def test_fast_sampling_keeps_missing_cohort_error_and_empty_batch():
    bank = {'station_xy': torch.zeros(3, 2), 'source_kind': torch.zeros(3)}
    args = dict(speed=.6, brake_s=.2, wide_probability=1., unbounded=True)
    with pytest.raises(ValueError, match='lacks cohort 1'):
        robust.sample_rows(bank, torch.zeros(1, 2), torch.ones(1), **args)
    result = robust.sample_rows(bank, torch.zeros(0, 2), torch.ones(0), **args)
    assert all(value.numel() == 0 for value in result)
