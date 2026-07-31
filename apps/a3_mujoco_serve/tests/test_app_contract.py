from __future__ import annotations

import importlib.util
import json
from pathlib import Path
from copy import deepcopy

import numpy as np
import pytest

from a3_serve.config import ConfigError, default_config_path, load_config
from a3_serve.constants import (
    DEFAULT_MODEL_XML,
    RIGHT_ARM_JOINTS,
    VALIDATED_FRAME_COUNT,
)
from a3_serve.csvio import MotionCsv, sha256_file
from a3_serve.pipeline import generate
from a3_serve.qualification import PROFILE_NAME, qualify


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


def test_validated_csv_passes_fixed_profile_and_is_approved() -> None:
    result = qualify(
        APP_ROOT / "assets/validated/serve_policy.csv",
        APP_ROOT / "assets/validated/serve_vendor_arm.json",
        APP_ROOT / "config/approved_motions.json",
    )
    assert result["status"] == "approved"
    assert result["approval_id"] == "a3-pr18-serve-policy"
    assert result["safety_profile"] == PROFILE_NAME
    assert result["safety_checks"] == "pass"
    assert result["motion"]["sha256"] == (
        "2a7de3f1c97a300069899c139c9eb96e94fd61d3419701d5e44ef37b2bf6641d"
    )
    assert result["metrics"]["max_source_stroke_speed_rad_s"] < 5.2
    assert result["metrics"]["max_command_velocity_limit_ratio"] < 0.5


def test_safe_unregistered_motion_is_candidate_not_implicitly_approved(
    tmp_path: Path,
) -> None:
    source = APP_ROOT / "assets/validated/serve_policy.csv"
    candidate = tmp_path / "candidate.csv"
    payload = bytearray(source.read_bytes())
    # Change a root translation digit. Root values are not sent by this
    # arm-only runtime, but the byte change gives the candidate a new identity.
    first_newline = payload.index(b"\n")
    data_offset = first_newline + 1
    payload[data_offset] = ord("1") if payload[data_offset] != ord("1") else ord("2")
    candidate.write_bytes(payload)
    manifest_path = tmp_path / "manifest.json"
    manifest = json.loads(
        (APP_ROOT / "assets/validated/serve_vendor_arm.json").read_text(
            encoding="utf-8"
        )
    )
    manifest["source"]["sha256"] = sha256_file(candidate)
    manifest_path.write_text(json.dumps(manifest), encoding="utf-8")

    result = qualify(
        candidate,
        manifest_path,
        APP_ROOT / "config/approved_motions.json",
    )
    assert result["safety_checks"] == "pass"
    assert result["status"] == "candidate"


def test_config_cannot_relax_fixed_safety_or_break_timing(tmp_path: Path) -> None:
    raw = json.loads(default_config_path().read_text(encoding="utf-8"))
    config_root = default_config_path().parent
    raw["model"]["xml"] = str((config_root / raw["model"]["xml"]).resolve())
    raw["source"]["template_csv"] = str(
        (config_root / raw["source"]["template_csv"]).resolve()
    )
    raw["physics"]["source_reference"] = str(
        (config_root / raw["physics"]["source_reference"]).resolve()
    )
    relaxed = deepcopy(raw)
    relaxed["safety"]["max_source_joint_speed_rad_s"] = 5.200001
    relaxed_path = tmp_path / "relaxed.json"
    relaxed_path.write_text(json.dumps(relaxed), encoding="utf-8")
    with pytest.raises(ConfigError, match="may not exceed"):
        load_config(relaxed_path)

    bad_timing = deepcopy(raw)
    bad_timing["timing"]["return_end_frame"] = bad_timing["timing"][
        "follow_end_frame"
    ]
    bad_timing_path = tmp_path / "bad_timing.json"
    bad_timing_path.write_text(json.dumps(bad_timing), encoding="utf-8")
    with pytest.raises(ConfigError, match="strictly ordered"):
        load_config(bad_timing_path)


def test_generator_refuses_validated_asset_directory() -> None:
    with pytest.raises(RuntimeError, match="may not overwrite"):
        generate(
            default_config_path(),
            APP_ROOT / "assets/validated/attempted-overwrite",
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
