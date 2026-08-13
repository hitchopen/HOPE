"""Host regression tests for both supported motion-body schemas."""

import importlib.util
from pathlib import Path

import numpy as np


_ROOT = Path(__file__).resolve().parents[1]
_MODULE = (
    _ROOT
    / "source"
    / "whole_body_tracking"
    / "whole_body_tracking"
    / "utils"
    / "motion_schema.py"
)
_SPEC = importlib.util.spec_from_file_location("hope_motion_schema", _MODULE)
motion_schema = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(motion_schema)
select_motion_bodies = motion_schema.select_motion_bodies


def test_tracked_body_only_array_is_already_in_command_order() -> None:
    array = np.arange(2 * 3 * 4).reshape(2, 3, 4)
    selected = select_motion_bodies(array, [1, 7, 11], "clip.npz", "body_quat_w")
    assert selected is array


def test_full_articulation_array_is_selected_by_live_indexes() -> None:
    array = np.arange(2 * 12 * 3).reshape(2, 12, 3)
    selected = select_motion_bodies(
        array, [1, 7, 11], "clip.npz", "body_pos_w", articulation_body_count=12
    )
    np.testing.assert_array_equal(selected, array[:, [1, 7, 11]])


def test_ambiguous_short_array_fails_before_cuda() -> None:
    array = np.zeros((2, 6, 3), dtype=np.float32)
    try:
        select_motion_bodies(
            array, [1, 4, 5], "clip.npz", "body_pos_w", articulation_body_count=12
        )
    except ValueError as error:
        message = str(error)
        assert "stores 6 bodies" in message
        assert "tracked bodies" in message
        assert "complete articulation-body arrays" in message
    else:
        raise AssertionError("an invalid motion-body schema must fail before CUDA")
