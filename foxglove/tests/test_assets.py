import json
from pathlib import Path
import unittest


FOXGLOVE_DIR = Path(__file__).resolve().parents[1]
REPO_ROOT = FOXGLOVE_DIR.parent


class AssetInvariantTests(unittest.TestCase):
    def test_layout_is_a3_rooted_and_has_no_publish_controls(self):
        layout = json.loads((FOXGLOVE_DIR / "layouts/a3_monitor.json").read_text())
        panel = layout["configById"]["3D!a3tf"]
        self.assertEqual(panel["followTf"], "pelvis_link")
        self.assertNotIn("publish", panel)
        self.assertEqual(
            layout["configById"]["Indicator!utc"]["path"],
            "/hope/ntp/gate_pass.data",
        )
        self.assertIn("Indicator!joints", layout["configById"])

    def test_bridge_does_not_advertise_write_or_asset_capabilities(self):
        params = (FOXGLOVE_DIR / "a3/bridge_params.yaml").read_text()
        self.assertNotIn("- clientPublish", params)
        self.assertNotIn("- assets", params)
        self.assertNotIn('^/motion/control/.*_joint_state$', params)
        self.assertIn('client_topic_whitelist: ["(?!)"]', params)

    def test_build_uses_pinned_ros2_source(self):
        build = (FOXGLOVE_DIR / "a3/build_foxglove_bridge.sh").read_text()
        self.assertIn("https://github.com/foxglove/foxglove-sdk.git", build)
        self.assertIn("ros-v3.4.3", build)
        self.assertIn("05f27efc7e535d9c30c6b0cb4f6aa89de7243870", build)
        self.assertNotIn("foxglove/ros-foxglove-bridge", build)

        service = (FOXGLOVE_DIR / "a3/hope-foxglove-bridge.service").read_text()
        self.assertIn("/hope_foxglove_ws/foxglove-sdk/ros/install/setup.bash", service)

    def test_downloaded_urdf_directory_is_ignored(self):
        ignore = (REPO_ROOT / ".gitignore").read_text().splitlines()
        self.assertIn("/foxglove/urdf/", ignore)


if __name__ == "__main__":
    unittest.main()
