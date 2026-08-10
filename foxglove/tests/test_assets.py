import hashlib
import json
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET


FOXGLOVE_DIR = Path(__file__).resolve().parents[1]
REPO_ROOT = FOXGLOVE_DIR.parent


class AssetInvariantTests(unittest.TestCase):
    def test_layout_is_a3_rooted_and_uses_the_console_extension(self):
        layout = json.loads((FOXGLOVE_DIR / "layouts/a3_monitor.json").read_text())
        panel = layout["configById"]["3D!a3tf"]
        # Keep the scene usable while vendor TF is absent; the table itself
        # provides `world`, and A3 appears only after world->pelvis is connected.
        self.assertEqual(panel["followTf"], "world")
        robot_layer = panel["layers"]["urdf-a3"]
        self.assertEqual(robot_layer["sourceType"], "topic")
        self.assertEqual(robot_layer["topic"], "/hope/robot_description")
        self.assertEqual(robot_layer["controlMode"], "transforms")
        self.assertNotIn("url", robot_layer)
        self.assertNotIn("publish", panel)
        extension_panel = "hope-a3-console.hope-a3-console!a3console"
        self.assertEqual(
            set(layout["configById"]),
            {"3D!a3tf", extension_panel},
        )
        self.assertEqual(layout["layout"]["first"], "3D!a3tf")
        self.assertEqual(layout["layout"]["second"], extension_panel)
        self.assertEqual(layout["layout"]["splitPercentage"], 60)
        self.assertNotIn("Publish!", json.dumps(layout["configById"]))

    def test_console_reuses_existing_topics_and_only_estop_control(self):
        extension = FOXGLOVE_DIR / "extensions/hope-a3-console"
        package = json.loads((extension / "package.json").read_text())
        self.assertEqual(package["name"], "hope-a3-console")
        self.assertEqual(package["publisher"], "hitchopen")
        source = (extension / "src/HopeA3Console.tsx").read_text()
        expected_topics = {
            "/hope/ntp/offset_ms",
            "/hope/ntp/root_dispersion_ms",
            "/hope/ntp/gate_pass",
            "/hope/clock/message_latency_ms",
            "/hope/clock/message_fresh",
            "/hope/system/cpu_load_percent",
            "/hope/vendor/agibot_pm_active",
            "/hope/vendor/tf_ready",
            "/hope/safety/estop_ready",
        }
        for topic in expected_topics:
            self.assertIn(f'"{topic}"', source)
        self.assertIn('const ESTOP_SERVICE = "/hope/safety/trigger_estop"', source)
        self.assertIn("context.callService(ESTOP_SERVICE, {})", source)
        self.assertNotIn("/hope/sequence/", source)
        self.assertNotIn("/hope/vendor/hdu_active", source)
        self.assertNotIn("/hope/vendor/mdu_active", source)
        self.assertNotIn("foxglove.SceneUpdate", source)

    def test_table_asset_uses_hope_world_geometry(self):
        asset = FOXGLOVE_DIR / "assets/hope_ping_pong_table.urdf"
        root = ET.parse(asset).getroot()
        self.assertEqual(root.tag, "robot")
        links = {element.attrib["name"] for element in root.findall("link")}
        self.assertIn("world", links)
        self.assertIn("hope_tabletop", links)

        joints = {element.attrib["name"]: element for element in root.findall("joint")}
        tabletop_origin = joints["world_to_hope_tabletop"].find("origin")
        self.assertEqual(tabletop_origin.attrib["xyz"], "1.370 -0.7625 -0.015")
        table_layer = json.loads(
            (FOXGLOVE_DIR / "layouts/a3_monitor.json").read_text()
        )["configById"]["3D!a3tf"]["layers"]["urdf-table"]
        self.assertEqual(
            table_layer["url"],
            "http://localhost:8000/assets/hope_ping_pong_table.urdf",
        )

    def test_bridge_exposes_only_estop_service_and_no_client_publish(self):
        params = (FOXGLOVE_DIR / "a3/bridge_params.yaml").read_text()
        self.assertNotIn("- clientPublish", params)
        self.assertNotIn("- assets", params)
        self.assertNotIn('^/motion/control/.*_joint_state$', params)
        self.assertIn('client_topic_whitelist: ["(?!)"]', params)
        self.assertIn('service_whitelist: ["^/hope/safety/trigger_estop$"]', params)
        self.assertIn("      - services", params)
        self.assertNotIn("HalEmergencyService", params)

    def test_build_uses_pinned_ros2_source(self):
        build = (FOXGLOVE_DIR / "a3/build_foxglove_bridge.sh").read_text()
        self.assertIn("https://github.com/foxglove/foxglove-sdk.git", build)
        self.assertIn("ros-v3.4.3", build)
        self.assertIn("05f27efc7e535d9c30c6b0cb4f6aa89de7243870", build)
        self.assertIn(
            "https://github.com/facontidavide/rosx_introspection.git", build
        )
        self.assertIn("ab747a0d3970d3297a5652b82e7645ab1d11feb9", build)
        self.assertIn("--packages-up-to foxglove_bridge", build)
        self.assertIn("apply_verified_patch", build)
        build_lines = build.splitlines()
        self.assertLess(
            build_lines.index("source /opt/ros/jazzy/setup.bash"),
            build_lines.index("set -u"),
        )
        self.assertNotIn("foxglove/ros-foxglove-bridge", build)

        patches = FOXGLOVE_DIR / "a3/patches"
        expected_patch_hashes = {
            "foxglove_bridge-ament-index-1.8.patch": (
                "1c6f40f6af4fe0186f196f65fb04c4d79b585ae16df448f16c72e09887d58828"
            ),
            "rosx_introspection-ament-index-1.8.patch": (
                "bd6541d663b57505cc083b7d67aa5593c6297928adee04ed72e64ae35f6e4da5"
            ),
        }
        for filename, expected_hash in expected_patch_hashes.items():
            payload = (patches / filename).read_bytes()
            self.assertEqual(hashlib.sha256(payload).hexdigest(), expected_hash)
            self.assertIn(expected_hash, build)

        service = (FOXGLOVE_DIR / "a3/hope-foxglove-bridge.service").read_text()
        self.assertIn("/hope_foxglove_ws/foxglove-sdk/ros/install/setup.bash", service)

    def test_a3_monitor_does_not_require_motive_access(self):
        service = (FOXGLOVE_DIR / "a3/hope-monitor.service").read_text()
        self.assertNotIn("mocap_host", service)
        self.assertNotIn("REPLACE_WITH_MOCAP_HOST", service)

        monitor = (FOXGLOVE_DIR / "a3/hope_monitor.py").read_text()
        self.assertNotIn('declare_parameter("mocap_host"', monitor)
        self.assertNotIn('"/hope/mocap/', monitor)
        self.assertIn('"/hope/robot_description"', monitor)
        self.assertIn("DurabilityPolicy.TRANSIENT_LOCAL", monitor)
        self.assertIn('declare_parameter("tf_stale_after_s", 0.5)', monitor)
        self.assertIn('"/hope/system/cpu_load_percent"', monitor)
        self.assertIn('Path("/proc/stat")', monitor)
        self.assertIn(
            'payload = self._robot_urdf_xml if show_robot else ""', monitor
        )

        unit = (FOXGLOVE_DIR / "a3/hope-monitor.service").read_text()
        self.assertIn("robot_model_info_path:=/agibot/data/info/model", unit)
        self.assertIn(
            "robot_model_root:=/opt/agibot/share/robot_model/models", unit
        )
        self.assertIn(
            "robot_asset_root_url:=http://localhost:8000/urdf", unit
        )
        self.assertIn("cpu_publish_period_s:=1.0", unit)

    def test_estop_proxy_is_assert_only(self):
        monitor = (FOXGLOVE_DIR / "a3/hope_monitor.py").read_text()
        self.assertIn("software E-stop asserted", monitor)
        self.assertIn("build_software_estop_request", monitor)
        self.assertIn("decode_software_estop_response", monitor)
        self.assertIn("client.service_is_ready()", monitor)
        self.assertIn("self.destroy_service(self._estop_proxy_service)", monitor)
        self.assertIn("lock is not held across vendor I/O", monitor)
        self.assertNotIn("software_emergency_stop = False", monitor)
        self.assertNotIn("clear_estop", monitor.lower())

    def test_downloaded_urdf_directory_is_ignored(self):
        ignore = (REPO_ROOT / ".gitignore").read_text().splitlines()
        self.assertIn("/foxglove/urdf/", ignore)


if __name__ == "__main__":
    unittest.main()
