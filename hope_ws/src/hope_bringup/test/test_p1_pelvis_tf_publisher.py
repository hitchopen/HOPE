"""Non-ROS tests for loading the saved P1-to-pelvis calibration."""

import importlib.machinery
import importlib.util
import json
import pathlib
import sys

import pytest


_SCRIPT = pathlib.Path(__file__).resolve().parents[1] / "scripts" / "p1_pelvis_tf_publisher"


def _load_module():
    loader = importlib.machinery.SourceFileLoader("p1_pelvis_tf_publisher", str(_SCRIPT))
    spec = importlib.util.spec_from_loader(loader.name, loader)
    assert spec is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    loader.exec_module(module)
    return module


def test_loads_and_normalizes_p1_to_pelvis_transform(tmp_path):
    module = _load_module()
    path = tmp_path / "p1_to_pelvis.json"
    path.write_text(
        json.dumps(
            {
                "p1_to_pelvis": {
                    "parent_frame": "P1",
                    "child_frame": "pelvis_link",
                    "translation_m": [0.0024, 0.0, 0.1490],
                    "quaternion_xyzw": [0.0, 0.0, 0.0, 2.0],
                }
            }
        ),
        encoding="utf-8",
    )

    parent, child, translation, quaternion = module.load_calibration(path)

    assert parent == "P1"
    assert child == "pelvis_link"
    assert translation == pytest.approx([0.0024, 0.0, 0.1490])
    assert quaternion == pytest.approx([0.0, 0.0, 0.0, 1.0])


def test_rejects_zero_norm_quaternion(tmp_path):
    module = _load_module()
    path = tmp_path / "bad.json"
    path.write_text(
        json.dumps(
            {
                "p1_to_pelvis": {
                    "parent_frame": "P1",
                    "child_frame": "pelvis_link",
                    "translation_m": [0.0, 0.0, 0.0],
                    "quaternion_xyzw": [0.0, 0.0, 0.0, 0.0],
                }
            }
        ),
        encoding="utf-8",
    )

    with pytest.raises(ValueError, match="non-zero norm"):
        module.load_calibration(path)
