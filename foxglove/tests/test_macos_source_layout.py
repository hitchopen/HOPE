"""Select existing deployment sources without moving the checkout's files."""
import importlib.util
from pathlib import Path
import tempfile
import unittest


ENTRY = Path(__file__).resolve().parents[2] / "scripts/macos/hope.py"
SPEC = importlib.util.spec_from_file_location("hope_macos_entry", ENTRY)
entry = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(entry)


class MacSourceLayoutTests(unittest.TestCase):
    def test_original_build6_directory_wins_when_both_layouts_exist(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            for parent in ("agi", "a3_deploy"):
                source = root / parent / "a3_deploy_example"
                source.mkdir(parents=True)
                (source / "CMakeLists.txt").touch()
            self.assertEqual(entry.source_directory(root, "a3_deploy_example", "CMakeLists.txt"),
                             root / "agi/a3_deploy_example")

    def test_old_build_outputs_do_not_hide_main_sources(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            (root / "agi/a3_deploy_example/build").mkdir(parents=True)
            source = root / "a3_deploy/a3_deploy_example"
            source.mkdir(parents=True)
            (source / "CMakeLists.txt").touch()
            self.assertEqual(entry.source_directory(root, "a3_deploy_example", "CMakeLists.txt"),
                             source)

    def test_simulator_uses_its_existing_directory_in_each_layout(self):
        for parent in ("agi", "a3_deploy"):
            with self.subTest(parent=parent), tempfile.TemporaryDirectory() as temporary:
                root = Path(temporary)
                source = root / parent / "A3_MuJoCo_Sim/aimrt_mujoco_sim"
                source.mkdir(parents=True)
                (source / "CMakeLists.txt").touch()
                self.assertEqual(entry.source_directory(root, "A3_MuJoCo_Sim/aimrt_mujoco_sim", "CMakeLists.txt"),
                                 source)

    def test_missing_sources_fail_without_creating_another_directory(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            with self.assertRaises(FileNotFoundError):
                entry.source_directory(root, "a3_deploy_example", "CMakeLists.txt")
            self.assertEqual(list(root.iterdir()), [])


if __name__ == "__main__":
    unittest.main()
