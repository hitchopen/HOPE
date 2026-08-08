"""Dependency-free tests for P1 marker-to-CAD rigid registration."""

import importlib.util
import math
import random
import sys
import unittest
from pathlib import Path


SCRIPT = (
    Path(__file__).resolve().parents[1]
    / "scripts"
    / "p1_marker_cad_registration_impl.py"
)
SPEC = importlib.util.spec_from_file_location("p1_marker_cad_registration", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


class P1MarkerCadRegistrationTest(unittest.TestCase):
    def setUp(self):
        self.transform = MODULE.Transform(
            (0.031, -0.012, 0.158),
            MODULE.normalize_quaternion((0.14, -0.21, 0.33, 0.90)),
        )

    def model_markers(self, names, *, shuffle=False, noise_m=0.0):
        randomizer = random.Random(124)
        ordered = list(names)
        if shuffle:
            randomizer.shuffle(ordered)
        result = []
        for index, name in enumerate(ordered):
            position = MODULE.transform_point(
                self.transform, MODULE.CAD_MARKERS_PELVIS_M[name]
            )
            if noise_m:
                position = tuple(
                    value + randomizer.gauss(0.0, noise_m)
                    for value in position
                )
            result.append(
                MODULE.ModelMarker(
                    member_id=20 + index,
                    name="",
                    position=position,
                )
            )
        return result

    def assert_transform_close(self, actual, expected, tolerance=1.0e-9):
        for left, right in zip(actual.translation, expected.translation):
            self.assertAlmostEqual(left, right, delta=tolerance)
        alignment = abs(
            sum(
                left * right
                for left, right in zip(actual.quaternion, expected.quaternion)
            )
        )
        self.assertAlmostEqual(alignment, 1.0, delta=tolerance)

    def test_horn_registration_recovers_known_transform(self):
        source = [
            (1.0, 0.0, 0.0),
            (0.0, 1.0, 0.0),
            (-1.0, -1.0, 0.0),
            (0.0, 0.0, 1.0),
        ]
        target = [
            MODULE.transform_point(self.transform, point) for point in source
        ]
        result = MODULE.rigid_registration(source, target)
        self.assertLess(result.rms_m, 1.0e-12)
        self.assert_transform_close(result.transform, self.transform)

    def test_geometry_inference_recovers_shuffled_realized_eight(self):
        markers = self.model_markers(
            MODULE.CURRENT_SHELL_MARKERS, shuffle=True, noise_m=0.0005
        )
        result = MODULE.resolve_correspondence(
            markers, MODULE.CURRENT_SHELL_MARKERS
        )
        self.assertEqual(
            set(result.mapping.values()), set(MODULE.CURRENT_SHELL_MARKERS)
        )
        self.assertLess(result.registration.rms_m, 0.0015)
        self.assertIsNotNone(result.margin_m)
        self.assertGreater(result.margin_m, 0.0015)
        self.assert_transform_close(
            result.registration.transform, self.transform, tolerance=0.0015
        )

    def test_geometry_inference_recovers_complete_ten(self):
        markers = self.model_markers(
            MODULE.MARKER_NAMES, shuffle=True, noise_m=0.0004
        )
        result = MODULE.resolve_correspondence(markers, MODULE.MARKER_NAMES)
        self.assertEqual(set(result.mapping.values()), set(MODULE.MARKER_NAMES))
        self.assertLess(result.registration.rms_m, 0.0015)
        self.assertGreater(result.margin_m, 0.0015)

    def test_explicit_mapping_must_cover_every_member(self):
        markers = self.model_markers(MODULE.CURRENT_SHELL_MARKERS)
        with self.assertRaisesRegex(ValueError, "cover every"):
            MODULE.resolve_correspondence(
                markers,
                MODULE.CURRENT_SHELL_MARKERS,
                {markers[0].member_id: "f2"},
            )

    def test_live_multi_heading_capture_can_pass_all_gates(self):
        markers = self.model_markers(MODULE.CURRENT_SHELL_MARKERS)
        capture = MODULE.Capture(
            rigid_body_name="P1",
            rigid_body_id=7,
            frame_id="world",
            markers=markers,
            frames_received=80,
        )
        for marker in markers:
            capture.live_errors_m[marker.member_id] = [0.0007] * 40
            capture.live_residuals_m[marker.member_id] = [0.0004] * 40
        capture.frames_with_physical_samples = 40
        capture.poses = [
            MODULE.Transform(
                (0.0, 0.0, 0.0),
                (
                    0.0,
                    0.0,
                    math.sin(math.radians(angle) / 2.0),
                    math.cos(math.radians(angle) / 2.0),
                ),
            )
            for angle in (0.0, 6.0, 12.0, 18.0)
        ]
        document, blockers = MODULE.analyze_capture(
            capture,
            MODULE.CURRENT_SHELL_MARKERS,
            {},
            max_registration_rms_m=0.003,
            max_registration_max_m=0.006,
            max_pairwise_rms_m=0.003,
            minimum_mapping_margin_m=0.0015,
            minimum_live_samples_per_marker=30,
            max_live_rms_m=0.004,
            max_live_max_m=0.005,
            minimum_rotation_span_deg=10.0,
            operator_attested_installed_layout=True,
            allow_nominal_only_markers=False,
        )
        self.assertEqual(blockers, [])
        self.assertTrue(document["approved"])
        candidate = document["hope_world_frame_yaml_candidate"]
        self.assertTrue(candidate["calibrated"])
        self.assert_transform_close(
            MODULE.Transform(
                tuple(candidate["xyz_m"]),
                (
                    candidate["quaternion_wxyz"][1],
                    candidate["quaternion_wxyz"][2],
                    candidate["quaternion_wxyz"][3],
                    candidate["quaternion_wxyz"][0],
                ),
            ),
            self.transform,
        )

    def test_ten_marker_nominal_points_require_explicit_confirmation(self):
        markers = self.model_markers(MODULE.MARKER_NAMES)
        capture = MODULE.Capture("P1", 1, "world", markers)
        for marker in markers:
            capture.live_errors_m[marker.member_id] = [0.0] * 30
            capture.live_residuals_m[marker.member_id] = [0.0] * 30
        capture.poses = [
            MODULE.Transform((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
            MODULE.Transform(
                (0.0, 0.0, 0.0),
                (0.0, 0.0, math.sin(0.1), math.cos(0.1)),
            ),
        ]
        _, blockers = MODULE.analyze_capture(
            capture,
            MODULE.MARKER_NAMES,
            {},
            max_registration_rms_m=0.003,
            max_registration_max_m=0.006,
            max_pairwise_rms_m=0.003,
            minimum_mapping_margin_m=0.0015,
            minimum_live_samples_per_marker=30,
            max_live_rms_m=0.004,
            max_live_max_m=0.005,
            minimum_rotation_span_deg=10.0,
            operator_attested_installed_layout=True,
            allow_nominal_only_markers=False,
        )
        self.assertTrue(any("nominal-only" in blocker for blocker in blockers))

    def test_single_live_outlier_fails_maximum_error_gate(self):
        markers = self.model_markers(MODULE.CURRENT_SHELL_MARKERS)
        capture = MODULE.Capture("P1", 1, "world", markers)
        for marker in markers:
            capture.live_errors_m[marker.member_id] = [0.0007] * 40
            capture.live_residuals_m[marker.member_id] = [0.0004] * 40
        capture.live_errors_m[markers[0].member_id][-1] = 0.015
        capture.poses = [
            MODULE.Transform((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
            MODULE.Transform(
                (0.0, 0.0, 0.0),
                (0.0, 0.0, math.sin(0.1), math.cos(0.1)),
            ),
        ]

        document, blockers = MODULE.analyze_capture(
            capture,
            MODULE.CURRENT_SHELL_MARKERS,
            {},
            max_registration_rms_m=0.003,
            max_registration_max_m=0.006,
            max_pairwise_rms_m=0.003,
            minimum_mapping_margin_m=0.0015,
            minimum_live_samples_per_marker=30,
            max_live_rms_m=0.004,
            max_live_max_m=0.005,
            minimum_rotation_span_deg=10.0,
            operator_attested_installed_layout=True,
            allow_nominal_only_markers=False,
        )

        self.assertFalse(document["approved"])
        self.assertTrue(
            any("live marker-to-ModelDef max" in blocker for blocker in blockers)
        )


if __name__ == "__main__":
    unittest.main()
