"""The public filenames must contain the complete Build motion pair."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import yaml


_REPO = Path(__file__).resolve().parents[3]
_MOTIONS = _REPO / "hope_training" / "motions" / "preprocessed"
_EXPECTED = {
    "hope_forehand": {
        "frames": 107,
        "strike_frame": 41,
    },
    "hope_backhand": {
        "frames": 109,
        "strike_frame": 48,
    },
}
_ARRAY_SHAPES = {
    "joint_pos": (31,),
    "joint_vel": (31,),
    "body_pos_w": (32, 3),
    "body_quat_w": (32, 4),
    "body_lin_vel_w": (32, 3),
    "body_ang_vel_w": (32, 3),
}
_ISAAC_JOINT_ORDER = (
    "left_hip_pitch_joint", "right_hip_pitch_joint", "waist_yaw_joint",
    "left_hip_roll_joint", "right_hip_roll_joint", "waist_roll_joint",
    "left_hip_yaw_joint", "right_hip_yaw_joint", "waist_pitch_joint",
    "left_knee_joint", "right_knee_joint", "head_yaw_joint",
    "left_shoulder_pitch_joint", "right_shoulder_pitch_joint",
    "left_ankle_pitch_joint", "right_ankle_pitch_joint", "head_pitch_joint",
    "left_shoulder_roll_joint", "right_shoulder_roll_joint",
    "left_ankle_roll_joint", "right_ankle_roll_joint",
    "left_shoulder_yaw_joint", "right_shoulder_yaw_joint",
    "left_elbow_joint", "right_elbow_joint", "left_wrist_roll_joint",
    "right_wrist_roll_joint", "left_wrist_pitch_joint", "right_wrist_pitch_joint",
    "left_wrist_yaw_joint", "right_wrist_yaw_joint",
)


def test_public_motion_names_carry_the_complete_build_pair() -> None:
    for stem, expected in _EXPECTED.items():
        path = _MOTIONS / f"{stem}.npz"
        assert path.is_file()
        with np.load(path, allow_pickle=False) as data:
            assert set(data.files) == {"fps", *_ARRAY_SHAPES}
            assert int(np.asarray(data["fps"]).reshape(-1)[0]) == 50
            for key, tail in _ARRAY_SHAPES.items():
                assert data[key].shape == (expected["frames"], *tail)
                assert data[key].dtype == np.float32
                assert np.isfinite(data[key]).all()


def test_motion_sidecars_match_the_published_binary_pair() -> None:
    for stem, expected in _EXPECTED.items():
        sidecar = yaml.safe_load((_MOTIONS / f"{stem}.yaml").read_text())
        assert sidecar["name"] == stem
        assert sidecar["fps"] == 50
        assert sidecar["frame_count"] == expected["frames"]
        assert sidecar["strike_frame"] == expected["strike_frame"]
        assert sidecar["body_schema"] == "complete_articulation_v1"
        assert sidecar["license"] == "Apache-2.0"
        assert sidecar["provenance"]
        assert sidecar["blade_normal_sign"] in (-1.0, 1.0)
        assert isinstance(sidecar["blade_normal_sign"], (int, float))
        assert tuple(sidecar["joint_order"]) == _ISAAC_JOINT_ORDER
