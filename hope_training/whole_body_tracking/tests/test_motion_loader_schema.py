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


def test_motion_loader_accepts_cuda_body_indexes(tmp_path) -> None:
    """Exercise the actual loader without starting Kit or importing task modules."""
    import ast
    import os
    from collections.abc import Sequence

    import pytest

    torch = pytest.importorskip("torch")
    if not torch.cuda.is_available():
        pytest.skip("CUDA is required to reproduce GPU articulation indexes")

    path = _MODULE.parents[1] / "tasks" / "tracking" / "mdp" / "commands.py"
    tree = ast.parse(path.read_text())
    loader = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == "MotionLoader")
    namespace = dict(np=np, torch=torch, os=os, Sequence=Sequence, select_motion_bodies=select_motion_bodies)
    exec(compile(ast.Module(body=[loader], type_ignores=[]), str(path), "exec"), namespace)
    indexes = torch.tensor([1, 7, 11], device="cuda")
    for body_count in (3, 12):
        fields = {
            name: np.arange(2 * body_count * width, dtype=np.float32).reshape(2, body_count, width)
            for name, width in (("body_pos_w", 3), ("body_quat_w", 4),
                                ("body_lin_vel_w", 3), ("body_ang_vel_w", 3))
        }
        clip = tmp_path / f"clip_{body_count}.npz"
        np.savez(clip, fps=50, joint_pos=np.zeros((2, 31)), joint_vel=np.zeros((2, 31)), **fields)
        motion = namespace["MotionLoader"]([str(clip), str(clip)], indexes,
                                          device="cuda", articulation_body_count=12)
        for name, values in fields.items():
            expected = values if body_count == 3 else values[:, [1, 7, 11]]
            actual = getattr(motion, name)
            assert actual.is_cuda
            np.testing.assert_array_equal(actual.cpu().numpy(), np.concatenate([expected, expected]))
        assert motion.seg_start.tolist() == [0, 2]
        assert motion.seg_len.tolist() == [2, 2]
