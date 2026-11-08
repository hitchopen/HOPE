"""Reject evidence that could admit a different or unsafe model/CSV pairing."""
import copy
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import unittest

SCRIPTS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SCRIPTS))
spec = importlib.util.spec_from_file_location("serve_bundle", SCRIPTS / "prepare_serve_policy_bundle.py")
bundle = importlib.util.module_from_spec(spec)
spec.loader.exec_module(bundle)


class EvidenceTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.policy = self.root / "policy"
        self.csv = self.root / "v23-17400.csv"
        paths = {"policy_sha256": self.policy / "exported/policy.onnx",
                 "deploy_sha256": self.policy / "params/deploy.yaml",
                 "csv_sha256": self.csv}
        self.identity = {}
        for key, path in paths.items():
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(key.encode())
            self.identity[key] = hashlib.sha256(path.read_bytes()).hexdigest()
        self.recipe = dict(self.identity)
        case = {"status": "PASS", "clock": "simulation", "cycles": [0, 1, 2],
                "transition_torque_ratio": 0.9,
                "boundaries": [{"q_dq_tau_kp_kd": [0.0] * 5}]}
        self.evidence = {**self.identity, "cases": {name: copy.deepcopy(case) for name in
            ("normal", "kernel", "normal_yaw", "pitch_plus", "pitch_minus")}}

    def validate(self):
        return bundle.validate_evidence(self.recipe, self.evidence, self.policy, self.csv)

    def test_matching_evidence(self):
        self.assertEqual(self.validate(), self.identity)

    def test_recipe_still_resolves_after_bundle_moves(self):
        source = self.root / 'source.csv'
        source.write_bytes(b'original motion')
        recipe = self.root / 'recipe.json'
        recipe.write_text(json.dumps({'source_csv': str(source),
                                     'source_sha256': bundle.digest(source)}))
        output = self.root / 'bundle'
        output.mkdir()
        bundle.copy_portable_recipe(recipe, output / recipe.name)
        source.unlink()
        moved = self.root / 'moved'
        output.rename(moved)
        copied = json.loads((moved / recipe.name).read_text())
        self.assertEqual(bundle.digest(moved / copied['source_csv']), copied['source_sha256'])

    def test_installed_static_files_readable_without_losing_executable(self):
        static = self.root / 'private.yaml'
        binary = self.root / 'runner'
        static.write_text('static')
        binary.write_text('executable')
        static.chmod(0o600)
        binary.chmod(0o700)
        bundle.normalize_static_permissions(self.root)
        self.assertEqual(static.stat().st_mode & 0o777, 0o644)
        self.assertEqual(binary.stat().st_mode & 0o777, 0o755)

    def test_csv_changed_after_simulation(self):
        self.csv.write_bytes(b"different adaptation")
        with self.assertRaisesRegex(ValueError, "evidence does not match"):
            self.validate()

    def test_recipe_from_another_model(self):
        self.recipe["policy_sha256"] = "0" * 64
        with self.assertRaisesRegex(ValueError, "recipe does not match"):
            self.validate()

    def test_nonfinite_or_discontinuous_handoff(self):
        for value in (float("nan"), float("inf"), 0.01):
            with self.subTest(value=value):
                self.evidence["cases"]["normal"]["boundaries"][0]["q_dq_tau_kp_kd"][2] = value
                with self.assertRaisesRegex(ValueError, "continuity"):
                    self.validate()

    def test_overlimit_transition(self):
        self.evidence["cases"]["kernel"]["transition_torque_ratio"] = 1.01
        with self.assertRaisesRegex(ValueError, "torque limits"):
            self.validate()

    def test_wall_clock_or_incomplete_repetition(self):
        self.evidence["cases"]["normal"]["clock"] = "wall"
        with self.assertRaisesRegex(ValueError, "simulation case"):
            self.validate()
        self.evidence["cases"]["normal"]["clock"] = "simulation"
        self.evidence["cases"]["normal"]["cycles"] = [0]
        with self.assertRaisesRegex(ValueError, "repeated-cycle"):
            self.validate()
