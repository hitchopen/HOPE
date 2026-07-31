from __future__ import annotations

import importlib.util
import json
from pathlib import Path

import numpy as np
import pytest

from a3_serve.config import default_config_path, load_config
from a3_serve.constants import (
    DEFAULT_MODEL_XML,
    RIGHT_ARM_JOINTS,
    VALIDATED_FRAME_COUNT,
)
from a3_serve.csvio import MotionCsv, sha256_file


APP_ROOT = Path(__file__).resolve().parents[1]


def test_default_config_resolves_the_shared_official_assets() -> None:
    config = load_config(default_config_path())
    assert Path(config["model"]["xml"]) == DEFAULT_MODEL_XML
    assert Path(config["model"]["xml"]).is_file()
    assert Path(config["physics"]["source_reference"]).is_file()
    assert Path(config["source"]["template_csv"]).is_file()
    assert config["model"]["racket_normal_axis"] == 1
    assert config["timing"]["frame_count"] == VALIDATED_FRAME_COUNT


def test_validated_csv_is_a_complete_all_joint_template() -> None:
    motion = MotionCsv.load(APP_ROOT / "assets/validated/serve_policy.csv")
    joints = motion.joint_radians()
    assert joints.shape == (VALIDATED_FRAME_COUNT, 31)
    right = motion.arm_radians()[:, 7:]
    copied = motion.with_right_arm(right)
    assert copied.shape == motion.values.shape
    assert np.allclose(copied, motion.values)
    assert len(RIGHT_ARM_JOINTS) == 7


def test_validated_manifest_binds_csv_and_demo_reference() -> None:
    manifest = json.loads(
        (APP_ROOT / "assets/validated/serve_vendor_arm.json").read_text(
            encoding="utf-8"
        )
    )
    csv_path = APP_ROOT / "assets/validated/serve_policy.csv"
    video_path = APP_ROOT / "assets/validated/pr18_a3_serve_demo.mp4"
    assert manifest["source"]["sha256"] == sha256_file(csv_path)
    assert (
        manifest["reference_sources"]["demo_video"]["sha256"]
        == sha256_file(video_path)
    )
    assert manifest["evidence"]["hardware_status"] == (
        "executable_and_safe_on_Agibot_A3"
    )


@pytest.mark.skipif(
    importlib.util.find_spec("mujoco") is None,
    reason="MuJoCo is an optional integration-test dependency",
)
def test_official_model_contains_the_required_racket_contract() -> None:
    from a3_serve.mujoco_scene import A3ServeScene

    config = load_config(default_config_path())
    scene = A3ServeScene(config["model"]["xml"], config["physics"], config["model"])
    assert scene.racket_site_id >= 0
    assert scene.racket_geom_id >= 0
    assert scene.model.nq > 31
    assert scene.model.nv > 31
