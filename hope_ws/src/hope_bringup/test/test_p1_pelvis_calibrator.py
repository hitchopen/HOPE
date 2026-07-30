"""Math-level tests for the P1-to-pelvis calibration utility.

These intentionally do not require a sourced ROS 2 installation: live
two-topic synchronization is exercised during the setup-session procedure.
"""

import importlib.machinery
import importlib.util
import math
import pathlib
import sys

import pytest


_SCRIPT = pathlib.Path(__file__).resolve().parents[1] / "scripts" / "p1_pelvis_calibration_impl.py"


def _load_module():
    loader = importlib.machinery.SourceFileLoader("p1_pelvis_calibration_impl", str(_SCRIPT))
    spec = importlib.util.spec_from_loader(loader.name, loader)
    assert spec is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    loader.exec_module(module)
    return module


def _yaw_quaternion(degrees):
    half = math.radians(degrees) / 2.0
    return (0.0, 0.0, math.sin(half), math.cos(half))


def test_recovers_fixed_p1_to_pelvis_transform_with_outlier():
    module = _load_module()
    correction = module.Transform((0.0024, 0.0, 0.1490), _yaw_quaternion(4.0))
    samples = []
    for index in range(20):
        raw = module.Transform(
            (0.2 + 0.01 * index, -0.4 + 0.005 * index, 0.8),
            _yaw_quaternion(-20.0 + index),
        )
        target_pelvis = module.compose(raw, correction)
        samples.append(module.compose(module.inverse(raw), target_pelvis))
    samples.append(module.Transform((0.4, 0.3, -0.1), _yaw_quaternion(75.0)))

    estimate, accepted, statistics, accepted_indices = module.estimate_fixed_transform(
        samples, translation_outlier_m=0.015, rotation_outlier_rad=math.radians(5.0)
    )

    assert len(accepted) == 20
    assert accepted_indices == list(range(20))
    assert estimate.translation == pytest.approx(correction.translation, abs=1e-12)
    assert abs(module.rotation_angle_rad(estimate.quaternion, correction.quaternion)) < 1e-12
    assert statistics["translation_rms_m"] < 1e-12
    assert statistics["rotation_rms_deg"] < 1e-10


def test_report_uses_local_pelvis_translation_after_pivot_axis_rotation():
    module = _load_module()
    # In a 90-degree yaw correction, +X of the raw frame is -Y after the new
    # pelvis axes are installed.  The report must express the pivot move in
    # those post-rotation local axes for Motive's Translation Offset fields.
    correction = module.Transform((0.1, 0.0, 0.0), _yaw_quaternion(90.0))
    document = module.result_document(
        correction,
        {},
        {
            "minimum_translation_span_m": 0.2,
            "minimum_rotation_span_deg": 15.0,
            "minimum_duration_s": 2.0,
        },
        {"absolute_rms_ms": 0.2, "absolute_max_ms": 0.4},
        10,
        10,
        "world",
        "/P1/pose",
        "/a3/calibration/pelvis_pose",
        "P1",
        "pelvis_link",
    )
    local_delta = document["motive_pivot_registration"]["pivot_delta_mm_in_pelvis_axes_after_rotation"]
    assert local_delta == pytest.approx([0.0, -100.0, 0.0], abs=1e-9)
    assert document["p1_to_pelvis"]["translation_m"] == pytest.approx([0.1, 0.0, 0.0])
    assert document["p1_to_pelvis"]["parent_frame"] == "P1"
    assert document["p1_to_pelvis"]["child_frame"] == "pelvis_link"
    assert (
        document["calibration"]["pelvis_reference_independence"]["status"]
        == "operator_precondition_not_observable_from_pose_messages"
    )
    assert (
        document["calibration"]["timestamp_provenance"]["status"]
        == "operator_precondition_not_inferable_from_header_stamp"
    )
    assert document["calibration"]["pelvis_reference_topic"] == "/a3/calibration/pelvis_pose"
    assert document["calibration"]["transform_model"] == "constant_rigid_attachment_P1_to_pelvis_link"
    cad = document["cad_cross_check"]
    assert cad["current_shell_marker_names"] == list(module.MARKER_NAMES)
    assert cad["selected_marker_names"] == list(module.MARKER_NAMES)
    assert cad["marker_centroid_in_pelvis_link_m"] == pytest.approx([-0.0024, 0.0, -0.1490])
    assert cad["expected_pivot_delta_mm_if_axes_aligned"] == pytest.approx([2.4, 0.0, 149.0])


def test_relative_pose_transform_uses_one_common_reference_frame():
    module = _load_module()
    world_to_p1 = module.Transform((0.5, -0.2, 0.8), _yaw_quaternion(30.0))
    expected = module.Transform((0.0024, 0.0, 0.1490), _yaw_quaternion(-4.0))
    world_to_pelvis = module.compose(world_to_p1, expected)

    class _Position:
        pass

    class _Orientation:
        pass

    class _Pose:
        pass

    def as_pose(transform):
        pose = _Pose()
        pose.position = _Position()
        pose.orientation = _Orientation()
        pose.position.x, pose.position.y, pose.position.z = transform.translation
        (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ) = transform.quaternion
        return pose

    result = module.relative_pose_transform(as_pose(world_to_p1), as_pose(world_to_pelvis))
    assert result.translation == pytest.approx(expected.translation, abs=1e-12)
    assert module.rotation_angle_rad(result.quaternion, expected.quaternion) < 1e-12


def test_rotation_medoid_seed_rejects_a_stale_burst():
    module = _load_module()
    valid = _yaw_quaternion(5.0)
    stale = _yaw_quaternion(80.0)
    seed = module.quaternion_medoid([valid] * 12 + [stale] * 8)

    assert module.rotation_angle_rad(seed, valid) < 1e-12


def test_estimator_rejects_a_multi_sample_stale_burst():
    module = _load_module()
    valid = module.Transform((0.0024, 0.0, 0.1490), _yaw_quaternion(5.0))
    stale = module.Transform((0.08, -0.04, 0.10), _yaw_quaternion(80.0))

    estimate, accepted, _, accepted_indices = module.estimate_fixed_transform(
        [valid] * 12 + [stale] * 8,
        translation_outlier_m=0.015,
        rotation_outlier_rad=math.radians(5.0),
    )

    assert len(accepted) == 12
    assert accepted_indices == list(range(12))
    assert estimate.translation == pytest.approx(valid.translation, abs=1e-12)
    assert module.rotation_angle_rad(estimate.quaternion, valid.quaternion) < 1e-12


def test_trajectory_statistics_exposes_stationary_capture():
    module = _load_module()
    fixed = module.Transform((0.0, 0.0, 0.0), _yaw_quaternion(0.0))
    statistics = module.trajectory_statistics(
        [fixed, fixed, fixed],
        [1_000_000_000, 1_500_000_000, 2_000_000_000],
    )

    assert statistics["translation_span_m"] == pytest.approx(0.0)
    assert statistics["rotation_span_deg"] == pytest.approx(0.0)
    assert statistics["duration_s"] == pytest.approx(1.0)
    assert statistics["average_rate_hz"] == pytest.approx(2.0)
    assert statistics["unique_timestamps"] == 3


def test_cad_cross_check_uses_all_ten_verified_visible_markers():
    module = _load_module()

    assert module.marker_centroid(module.MARKER_NAMES) == pytest.approx((-0.0024, 0.0, -0.1490))
    assert module.marker_centroid(module.CURRENT_SHELL_MARKER_NAMES) == pytest.approx(
        (-0.0024, 0.0, -0.1490)
    )
    assert module.CURRENT_SHELL_MARKER_NAMES == module.MARKER_NAMES


def test_cad_cross_check_is_invariant_to_marker_stream_order():
    module = _load_module()
    shuffled = ("b4", "f3", "b2", "f5", "f1", "b5", "f4", "b3", "f2", "b1")

    assert module.marker_centroid(shuffled) == pytest.approx(
        module.marker_centroid(module.CURRENT_SHELL_MARKER_NAMES)
    )
