import pytest

from hope_planner.x_hit_freeze import select_stable_base_x


def test_selects_median_from_recent_stable_samples():
    result = select_stable_base_x(
        [(9.0, 99.0), (9.6, 0.101), (9.8, 0.099), (9.9, 0.100)],
        now_s=10.0,
        window_s=0.5,
        max_age_s=0.2,
        min_samples=3,
        max_span_m=0.01,
    )

    assert result.x_m == pytest.approx(0.100)
    assert result.samples == 3
    assert result.span_m == pytest.approx(0.002)
    assert result.newest_age_s == pytest.approx(0.1)


@pytest.mark.parametrize(
    ("samples", "message"),
    [
        ([(9.9, 0.0)], "need at least 3"),
        ([(9.6, 0.0), (9.7, 0.0), (9.79, 0.0)], "stale"),
        ([(9.8, 0.0), (9.9, 0.02), (9.95, 0.0)], "not settled"),
        ([(9.8, 0.0), (9.9, float("nan")), (9.95, 0.0)], "non-finite"),
    ],
)
def test_rejects_unsafe_sample_windows(samples, message):
    with pytest.raises(ValueError, match=message):
        select_stable_base_x(
            samples,
            now_s=10.0,
            window_s=0.5,
            max_age_s=0.2,
            min_samples=3,
            max_span_m=0.01,
        )
