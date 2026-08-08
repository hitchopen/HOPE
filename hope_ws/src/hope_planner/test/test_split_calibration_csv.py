import numpy as np

from hope_planner.split_calibration_csv import _infer_scale, _normalize_chunk, _split_rows


def test_infer_scale_detects_millimeters():
    rows = [
        (0.0, 1000.0, -500.0, -680.0),
        (0.01, 1200.0, -450.0, -640.0),
    ]
    assert _infer_scale(rows) == 0.001


def test_split_rows_breaks_on_large_time_gap():
    rows = [
        (0.0, 0.0, 0.0, 0.0),
        (0.02, 1.0, 0.0, 0.1),
        (2.50, 2.0, 0.0, 0.2),
    ]
    chunks = _split_rows(rows, gap_seconds=1.0)
    assert len(chunks) == 2
    assert len(chunks[0]) == 2
    assert len(chunks[1]) == 1


def test_normalize_chunk_rezeros_time_and_local_table_height():
    rows = [
        (10.0, 0.0, 0.0, -680.0),
        (10.1, 100.0, 0.0, -680.0),
        (10.2, 200.0, 0.0, 120.0),
        (10.3, 300.0, 0.0, -675.0),
    ]
    times, pos, z_offset = _normalize_chunk(rows, scale_to_m=0.001)
    assert np.isclose(times[0], 0.0)
    assert np.isclose(times[-1], 0.3)
    assert z_offset < -0.60
    assert float(np.min(pos[:, 2])) > -0.05
