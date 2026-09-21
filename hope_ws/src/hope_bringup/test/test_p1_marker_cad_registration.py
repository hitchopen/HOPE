"""Dependency-free tests for UCB robot marker-to-CAD registration."""

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
sys.path.insert(0, str(SCRIPT.parent))
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

    def model_markers(
        self,
        names,
        *,
        shuffle=False,
        noise_m=0.0,
        cad_markers=None,
    ):
        randomizer = random.Random(124)
        cad_markers = cad_markers or MODULE.CAD_MARKERS_PELVIS_M
        ordered = list(names)
        if shuffle:
            randomizer.shuffle(ordered)
        result = []
        for index, name in enumerate(ordered):
            position = MODULE.transform_point(
                self.transform, cad_markers[name]
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

    def test_v3_coordinates_and_geometry_are_independent_from_v2(self):
        expected_back = {
            "b1": (-0.090, 0.000, -0.130),
            "b2": (-0.090, 0.000, -0.090),
            "b3": (-0.090, -0.055, -0.130),
            "b4": (-0.090, -0.030, -0.170),
            "b5": (-0.090, 0.030, -0.150),
        }
        for name, point in expected_back.items():
            self.assertEqual(MODULE.V3_CAD_MARKERS_PELVIS_M[name], point)
            self.assertNotEqual(
                MODULE.V3_CAD_MARKERS_PELVIS_M[name],
                MODULE.V2_CAD_MARKERS_PELVIS_M[name],
            )

        markers = self.model_markers(
            MODULE.MARKER_NAMES,
            shuffle=True,
            noise_m=0.0004,
            cad_markers=MODULE.V3_CAD_MARKERS_PELVIS_M,
        )
        result = MODULE.resolve_correspondence(
            markers,
            MODULE.MARKER_NAMES,
            cad_markers_pelvis_m=MODULE.V3_CAD_MARKERS_PELVIS_M,
        )
        self.assertEqual(set(result.mapping.values()), set(MODULE.MARKER_NAMES))
        self.assertLess(result.registration.rms_m, 0.0015)

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
            rigid_body_name="UCB_P1",
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
        canonical = document["p1_to_pelvis"]
        self.assertEqual(canonical["parent_frame"], "UCB_P1")
        self.assertEqual(canonical["child_frame"], "pelvis_link")
        self.assertEqual(
            canonical["translation_m"],
            document["p1_to_pelvis_link"]["xyz_m"],
        )
        snapshot = document["world_to_pelvis_snapshot"]
        self.assertEqual(snapshot["parent_frame"], "world")
        self.assertEqual(snapshot["child_frame"], "pelvis_link")
        self.assertEqual(snapshot["sample_count"], len(capture.poses))
        expected_snapshot = MODULE.compose(
            MODULE.representative_transform(capture.poses), self.transform
        )
        self.assert_transform_close(
            MODULE.Transform(
                tuple(snapshot["translation_m"]),
                tuple(snapshot["quaternion_xyzw"]),
            ),
            expected_snapshot,
        )

    def test_stationary_named_ten_marker_prepare_can_pass(self):
        markers = self.model_markers(MODULE.MARKER_NAMES)
        capture = MODULE.Capture("UCB_P1", 26, "world", markers, frames_received=80)
        for marker in markers:
            capture.live_errors_m[marker.member_id] = [0.0007] * 40
            capture.live_residuals_m[marker.member_id] = [0.0004] * 40
        capture.frames_with_physical_samples = 40
        capture.poses = [
            MODULE.Transform((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
            for _ in range(40)
        ]

        document, blockers = MODULE.analyze_capture(
            capture,
            MODULE.MARKER_NAMES,
            {},
            max_registration_rms_m=0.003,
            max_registration_max_m=0.006,
            max_pairwise_rms_m=0.005,
            minimum_mapping_margin_m=0.0015,
            minimum_live_samples_per_marker=30,
            max_live_rms_m=0.004,
            max_live_max_m=0.005,
            minimum_rotation_span_deg=0.0,
            operator_attested_installed_layout=True,
            allow_nominal_only_markers=True,
        )

        self.assertEqual(blockers, [])
        self.assertTrue(document["approved"])
        self.assertEqual(
            document["method"]["trajectory_validation"],
            "stationary_named_marker_geometry",
        )

    def test_new_v2_pants_uses_only_the_explicit_3_25_mm_rms_gate(self):
        # Motive UCB_P1 ModelDef captured from five independent windows on the
        # replacement V2 pants. The geometry is stable: 3.0818968 mm
        # registration RMS, 3.6609441 mm max, and 4.3907482 mm pairwise RMS.
        positions = (
            (0.07662788033485413, -0.06277170777320862, 0.0024434730876237154),
            (0.09559246152639389, -0.01452559232711792, 0.010213525965809822),
            (0.07411497086286545, -0.04314061254262924, -0.037915267050266266),
            (-0.09059888869524002, -0.041058026254177094, 0.028383569791913033),
            (0.08405787497758865, 0.016179122030735016, -0.03946467861533165),
            (0.09324447065591812, 0.03628412261605263, -0.00035171935451216996),
            (-0.09196184575557709, -0.016972942277789116, -0.022446567192673683),
            (-0.07394591718912125, 0.06866469234228134, 0.025611473247408867),
            (-0.08450483530759811, 0.014738926663994789, 0.05757268890738487),
            (-0.08262129127979279, 0.04260258376598358, -0.024058081209659576),
        )
        markers = [
            MODULE.ModelMarker(member_id=index, name="", position=position)
            for index, position in enumerate(positions)
        ]
        capture = MODULE.Capture("UCB_P1", 33, "world", markers, frames_received=80)
        for marker in markers:
            capture.live_errors_m[marker.member_id] = [0.00042] * 40
            capture.live_residuals_m[marker.member_id] = [0.00036] * 40
        capture.frames_with_physical_samples = 40
        capture.poses = [
            MODULE.Transform((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
            for _ in range(40)
        ]
        common = dict(
            max_registration_max_m=0.006,
            max_pairwise_rms_m=0.005,
            minimum_mapping_margin_m=0.0015,
            minimum_live_samples_per_marker=30,
            max_live_rms_m=0.004,
            max_live_max_m=0.008,
            minimum_rotation_span_deg=0.0,
            operator_attested_installed_layout=True,
            allow_nominal_only_markers=True,
        )

        _, strict_blockers = MODULE.analyze_capture(
            capture,
            MODULE.MARKER_NAMES,
            {},
            max_registration_rms_m=0.003,
            **common,
        )
        document, replacement_pants_blockers = MODULE.analyze_capture(
            capture,
            MODULE.MARKER_NAMES,
            {},
            max_registration_rms_m=0.00325,
            **common,
        )

        self.assertEqual(
            strict_blockers,
            ["CAD registration RMS 3.08 mm exceeds 3.00 mm"],
        )
        self.assertEqual(replacement_pants_blockers, [])
        self.assertTrue(document["approved"])
        self.assertAlmostEqual(
            document["quality"]["registration_rms_m"],
            0.003081896775690046,
        )
        self.assertEqual(
            document["quality"]["gates"]["max_registration_rms_m"],
            0.00325,
        )
        self.assertEqual(
            document["quality"]["gates"]["max_pairwise_rms_m"],
            0.005,
        )
        self.assertEqual(
            document["quality"]["gates"]["max_registration_max_m"],
            0.006,
        )

    def test_v3_p2_snapshot_uses_same_table_rotation_as_runtime(self):
        markers = self.model_markers(
            MODULE.MARKER_NAMES,
            cad_markers=MODULE.V3_CAD_MARKERS_PELVIS_M,
        )
        capture = MODULE.Capture("UCB_P2", 27, "world", markers, frames_received=80)
        for marker in markers:
            capture.live_errors_m[marker.member_id] = [0.0007] * 40
            capture.live_residuals_m[marker.member_id] = [0.0004] * 40
        capture.frames_with_physical_samples = 40
        capture.poses = [
            MODULE.Transform((3.0, -0.5, 0.2), (0.0, 0.0, 0.0, 1.0))
            for _ in range(40)
        ]

        document, blockers = MODULE.analyze_capture(
            capture,
            MODULE.MARKER_NAMES,
            {},
            max_registration_rms_m=0.003,
            max_registration_max_m=0.006,
            max_pairwise_rms_m=0.005,
            minimum_mapping_margin_m=0.0015,
            minimum_live_samples_per_marker=30,
            max_live_rms_m=0.004,
            max_live_max_m=0.005,
            minimum_rotation_span_deg=0.0,
            operator_attested_installed_layout=True,
            allow_nominal_only_markers=False,
            layout=MODULE.V3_MARKER_LAYOUT,
            table_side="P2",
        )

        self.assertEqual(blockers, [])
        self.assertEqual(document["cad"]["marker_layout"], "v3")
        self.assertEqual(document["p1_to_pelvis"]["parent_frame"], "UCB_P2")
        self.assertEqual(document["runtime_world"]["table_side"], "P2")
        local_marker = MODULE.Transform(
            (-0.26, -1.025, 0.2), (0.0, 0.0, 1.0, 0.0)
        )
        expected_snapshot = MODULE.compose(local_marker, self.transform)
        snapshot = document["world_to_pelvis_snapshot"]
        self.assert_transform_close(
            MODULE.Transform(
                tuple(snapshot["translation_m"]),
                tuple(snapshot["quaternion_xyzw"]),
            ),
            expected_snapshot,
        )

    def test_p1_p2_v2_v3_profile_matrix_is_supported(self):
        for table_side in ("P1", "P2"):
            for layout in (MODULE.V2_MARKER_LAYOUT, MODULE.V3_MARKER_LAYOUT):
                with self.subTest(table_side=table_side, layout=layout.layout_id):
                    markers = self.model_markers(
                        layout.realized_marker_names,
                        cad_markers=layout.markers_pelvis_m,
                    )
                    capture = MODULE.Capture(
                        f"UCB_{table_side}", 27, "world", markers, frames_received=20
                    )
                    for marker in markers:
                        capture.live_errors_m[marker.member_id] = [0.0002] * 5
                        capture.live_residuals_m[marker.member_id] = [0.0001] * 5
                    capture.frames_with_physical_samples = 5
                    capture.poses = [
                        MODULE.Transform(
                            (0.25, -0.50, 0.20),
                            (0.0, 0.0, 0.0, 1.0),
                        )
                        for _ in range(5)
                    ]

                    document, blockers = MODULE.analyze_capture(
                        capture,
                        layout.realized_marker_names,
                        {},
                        max_registration_rms_m=0.00325,
                        max_registration_max_m=0.006,
                        max_pairwise_rms_m=0.005,
                        minimum_mapping_margin_m=0.0,
                        minimum_live_samples_per_marker=5,
                        max_live_rms_m=0.004,
                        max_live_max_m=0.008,
                        minimum_rotation_span_deg=0.0,
                        operator_attested_installed_layout=True,
                        allow_nominal_only_markers=False,
                        layout=layout,
                        table_side=table_side,
                    )

                    self.assertEqual(blockers, [])
                    self.assertTrue(document["approved"])
                    self.assertEqual(
                        document["cad"]["marker_layout"], layout.layout_id
                    )
                    self.assertEqual(
                        document["p1_to_pelvis"]["parent_frame"],
                        f"UCB_{table_side}",
                    )
                    self.assertEqual(
                        document["runtime_world"]["table_side"], table_side
                    )

    def test_v3_coordinate_table_matches_the_v03_handoff(self):
        self.assertEqual(
            MODULE.V3_CAD_MARKERS_PELVIS_M,
            {
                "f1": (0.090, 0.000, -0.130),
                "f2": (0.080, 0.050, -0.140),
                "f3": (0.080, -0.050, -0.140),
                "f4": (0.078, -0.030, -0.180),
                "f5": (0.078, 0.030, -0.180),
                "b1": (-0.090, 0.000, -0.130),
                "b2": (-0.090, 0.000, -0.090),
                "b3": (-0.090, -0.055, -0.130),
                "b4": (-0.090, -0.030, -0.170),
                "b5": (-0.090, 0.030, -0.150),
            },
        )
        self.assertEqual(
            dict(MODULE.V3_MARKER_LAYOUT.source_files),
            {
                "0000019398_胯部靶球座-V3-多实体.STEP": (
                    "341c0d089147e50db3ac35a8fa580dcc9609fcb4fdf18d5b3eac9d954d7aaf49"
                ),
                "img_v3_0214n_5dcd9947-eae9-446f-b2f0-0a57dc4da20g.jpg": (
                    "7a3b49f29dac81d695e091dbf5650f0cb91171bdc2233c49d3cf3fa90ea9b059"
                ),
                "img_v3_0214n_a312ac8e-7ae5-4d09-95f5-27506ef4f01g.jpg": (
                    "e6a0efae2a9f65d1739488d2126b6f02a9abcb73a36dfe4e4a566041246dd6c6"
                ),
            },
        )

    def test_capture_waits_until_every_marker_has_physical_samples(self):
        markers = self.model_markers(MODULE.MARKER_NAMES)
        capture = MODULE.Capture("UCB_P1", 26, "world", markers, frames_received=800)
        for marker in markers:
            capture.live_errors_m[marker.member_id] = [0.0007] * 30
        self.assertTrue(MODULE.physical_marker_samples_ready(capture, 30))

        capture.live_errors_m[markers[6].member_id] = []
        self.assertFalse(MODULE.physical_marker_samples_ready(capture, 30))

        capture.live_errors_m[markers[6].member_id] = [0.0007] * 29
        self.assertFalse(MODULE.physical_marker_samples_ready(capture, 30))

        capture.live_errors_m[markers[6].member_id].append(0.0007)
        self.assertTrue(MODULE.physical_marker_samples_ready(capture, 30))

    def test_atomic_write_replaces_complete_receipt(self):
        import tempfile

        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "ucb_robot_to_pelvis.json"
            path.write_bytes(b"old\n")
            MODULE._write_bytes_atomic(path, b'{"approved": true}\n')
            self.assertEqual(path.read_bytes(), b'{"approved": true}\n')
            self.assertEqual(list(path.parent.glob(".*.tmp")), [])

    def test_ten_marker_nominal_points_require_explicit_confirmation(self):
        markers = self.model_markers(MODULE.MARKER_NAMES)
        capture = MODULE.Capture("UCB_P1", 1, "world", markers)
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
        capture = MODULE.Capture("UCB_P1", 1, "world", markers)
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

    def test_damaged_b4_exception_needs_explicit_eight_mm_live_max(self):
        markers = self.model_markers(MODULE.MARKER_NAMES)
        capture = MODULE.Capture("UCB_P1", 26, "world", markers, frames_received=80)
        for marker in markers:
            capture.live_errors_m[marker.member_id] = [0.0008] * 40
            capture.live_residuals_m[marker.member_id] = [0.0004] * 40
        damaged_b4 = next(
            marker for marker in markers
            if MODULE.MARKER_NAMES[marker.member_id - 20] == "b4"
        )
        capture.live_errors_m[damaged_b4.member_id] = [0.0062] * 40
        capture.poses = [
            MODULE.Transform((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
            for _ in range(40)
        ]

        common = dict(
            max_registration_rms_m=0.003,
            max_registration_max_m=0.006,
            max_pairwise_rms_m=0.005,
            minimum_mapping_margin_m=0.0015,
            minimum_live_samples_per_marker=30,
            max_live_rms_m=0.004,
            minimum_rotation_span_deg=0.0,
            operator_attested_installed_layout=True,
            allow_nominal_only_markers=True,
        )
        _, strict_blockers = MODULE.analyze_capture(
            capture,
            MODULE.MARKER_NAMES,
            {},
            max_live_max_m=0.005,
            **common,
        )
        document, relaxed_blockers = MODULE.analyze_capture(
            capture,
            MODULE.MARKER_NAMES,
            {},
            max_live_max_m=0.008,
            **common,
        )

        self.assertTrue(
            any("live marker-to-ModelDef max" in item for item in strict_blockers)
        )
        self.assertEqual(relaxed_blockers, [])
        self.assertTrue(document["approved"])


if __name__ == "__main__":
    unittest.main()
