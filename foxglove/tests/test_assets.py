import hashlib
import json
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

import yaml


FOXGLOVE_DIR = Path(__file__).resolve().parents[1]
REPO_ROOT = FOXGLOVE_DIR.parent


class AssetInvariantTests(unittest.TestCase):
    def test_callservice_layout_alias_matches_canonical_layout(self):
        canonical = (FOXGLOVE_DIR / "layouts/a3_monitor.json").read_bytes()
        compatibility = (
            FOXGLOVE_DIR / "layouts/a3_monitor_callservice.json"
        ).read_bytes()
        self.assertEqual(compatibility, canonical)

    def test_legacy_layout_is_a3_rooted_and_has_monitoring_panels(self):
        layout = json.loads((FOXGLOVE_DIR / "layouts/a3_monitor.json").read_text())
        panel = layout["configById"]["3D!a3tf"]
        # Keep the scene rooted in `world` without exposing the robot TF tree.
        # The only URDF layer is the static table asset served by the Laptop.
        self.assertEqual(panel["followTf"], "world")
        table = panel["layers"]["urdf-table"]
        self.assertEqual(table["layerId"], "foxglove.Urdf")
        self.assertEqual(
            table["url"],
            "http://localhost:8000/assets/hope_ping_pong_table.urdf",
        )
        self.assertEqual(
            [layer["layerId"] for layer in panel["layers"].values()].count(
                "foxglove.Urdf"
            ),
            1,
        )
        self.assertTrue(panel["scene"]["transforms"]["showLabel"])
        self.assertEqual(panel["transforms"], {})
        self.assertNotIn("/hope/robot_description", json.dumps(panel))
        self.assertNotIn("/joint_states", json.dumps(panel))
        self.assertTrue(panel["topics"]["/hope/ball/marker"]["visible"])
        self.assertNotIn("publish", panel)
        self.assertEqual(
            layout["configById"]["Indicator!ntp"]["path"],
            "/hope/ntp/gate_pass.data",
        )
        self.assertEqual(
            layout["configById"]["Indicator!agibotpm"]["path"],
            "/hope/vendor/agibot_pm_active.data",
        )
        self.assertEqual(
            layout["configById"]["Indicator!tfready"]["path"],
            "/hope/vendor/tf_ready.data",
        )
        self.assertEqual(
            layout["configById"]["Indicator!estopready"]["path"],
            "/hope/safety/estop_ready.data",
        )
        self.assertEqual(
            layout["configById"]["Plot!messagelatency"]["paths"][0]["value"],
            "/hope/clock/message_latency_ms.data",
        )
        cpu_plot = layout["configById"]["Plot!cpuload"]
        self.assertEqual(
            cpu_plot["paths"][0]["value"],
            "/hope/system/cpu_load_percent.data",
        )
        self.assertEqual(cpu_plot["minYValue"], 0)
        self.assertEqual(cpu_plot["maxYValue"], 100)
        self.assertEqual(cpu_plot["yAxisLabel"], "CPU load (%)")
        self.assertIn('"Plot!cpuload"', json.dumps(layout["layout"]))
        self.assertEqual(
            layout["configById"]["CallService!estop"]["serviceName"],
            "/hope/safety/trigger_estop",
        )
        self.assertFalse(layout["configById"]["CallService!estop"]["editingMode"])
        self.assertNotIn("Publish!", json.dumps(layout["configById"]))

    def test_table_asset_has_hope_world_geometry_and_is_used_by_layout(self):
        asset = FOXGLOVE_DIR / "assets/hope_ping_pong_table.urdf"
        root = ET.parse(asset).getroot()
        self.assertEqual(root.tag, "robot")
        links = {element.attrib["name"] for element in root.findall("link")}
        self.assertIn("world", links)
        self.assertIn("hope_tabletop", links)

        joints = {element.attrib["name"]: element for element in root.findall("joint")}
        tabletop_origin = joints["world_to_hope_tabletop"].find("origin")
        self.assertEqual(tabletop_origin.attrib["xyz"], "1.370 -0.7625 -0.015")
        layout = json.loads((FOXGLOVE_DIR / "layouts/a3_monitor.json").read_text())
        table = layout["configById"]["3D!a3tf"]["layers"]["urdf-table"]
        self.assertEqual(table["layerId"], "foxglove.Urdf")
        self.assertIn("hope_ping_pong_table.urdf", table["url"])

    def test_fleet_bridge_exposes_only_assert_estop_and_no_client_publish(self):
        params = (FOXGLOVE_DIR / "a3/bridge_params.yaml").read_text()
        self.assertNotIn("- clientPublish", params)
        self.assertNotIn("- assets", params)
        self.assertNotIn('^/motion/control/.*_joint_state$', params)
        self.assertNotIn('"^/tf$"', params)
        self.assertNotIn('"^/tf_static$"', params)
        self.assertIn('client_topic_whitelist: ["(?!)"]', params)
        self.assertIn('- "^/hope/safety/trigger_estop$"', params)
        parsed = yaml.safe_load(params)["/**"]["ros__parameters"]
        self.assertEqual(
            parsed["service_whitelist"],
            ["^/hope/safety/trigger_estop$"],
        )
        self.assertNotIn("/hope/control/", params)
        self.assertNotIn("/hope/runner/emergency_stop", params)
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
        self.assertIn("from tf2_msgs.msg import TFMessage", monitor)
        self.assertIn('TFMessage, "/hope/pelvis/tf", 10', monitor)
        self.assertIn("TFMessage(transforms=[root_tf])", monitor)
        self.assertNotIn('declare_parameter("mocap_host"', monitor)
        self.assertNotIn('"/hope/mocap/', monitor)
        self.assertNotIn('"/hope/robot_description"', monitor)
        self.assertNotIn("JointState", monitor)
        self.assertNotIn("load_robot_urdf_for_foxglove", monitor)
        self.assertIn('declare_parameter("tf_stale_after_s", 0.5)', monitor)
        self.assertIn('"/hope/system/cpu_load_percent"', monitor)
        self.assertIn('"/hope/ntp/text"', monitor)
        self.assertIn('Path("/proc/stat")', monitor)

        unit = (FOXGLOVE_DIR / "a3/hope-monitor.service").read_text()
        self.assertNotIn("robot_model_info_path", unit)
        self.assertNotIn("robot_model_root", unit)
        self.assertNotIn("robot_asset_root_url", unit)
        self.assertIn("cpu_publish_period_s:=1.0", unit)

    def test_estop_proxy_is_assert_only(self):
        monitor = (FOXGLOVE_DIR / "a3/hope_monitor.py").read_text()
        self.assertIn("combine_estop_results", monitor)
        unit = (FOXGLOVE_DIR / "a3/hope-monitor.service").read_text()
        self.assertIn(
            "runner_estop_service:=/hope/runner/emergency_passive", unit
        )
        self.assertIn("build_software_estop_request", monitor)
        self.assertIn("decode_software_estop_response", monitor)
        self.assertIn("client.service_is_ready()", monitor)
        self.assertIn('"/hope/safety/estop_full_ready"', monitor)
        self.assertIn("estop_backend_status", monitor)
        self.assertIn("self._estop_proxy_service = self.create_service(", monitor)
        self.assertNotIn("self.destroy_service(self._estop_proxy_service)", monitor)
        self.assertIn("E-STOP REASSERTED", monitor)
        self.assertIn("Do not hold this lock across vendor I/O", monitor)
        self.assertNotIn("software_emergency_stop = False", monitor)
        self.assertNotIn("clear_estop", monitor.lower())

    def test_pelvis_3d_label_uses_standard_visualization_markers(self):
        monitor = (FOXGLOVE_DIR / "a3/hope_monitor.py").read_text()
        unit = (FOXGLOVE_DIR / "a3/hope-monitor.service").read_text()
        self.assertIn("from visualization_msgs.msg import Marker", monitor)
        self.assertIn('"/hope/pelvis/marker"', monitor)
        self.assertIn("Marker.SPHERE", monitor)
        self.assertIn("Marker.TEXT_VIEW_FACING", monitor)
        self.assertIn('"/a3/mocap/pelvis_pose"', monitor)
        self.assertIn("TransformBroadcaster", monitor)
        self.assertIn("point.header = pose.header", monitor)
        self.assertIn("point.frame_locked = True", monitor)
        self.assertIn("label.frame_locked = True", monitor)
        self.assertNotIn("foxglove_msgs", monitor)
        self.assertFalse(
            (FOXGLOVE_DIR / "a3/hope-robot-state-publisher.service").exists()
        )
        self.assertIn(
            "/hope_foxglove_ws/foxglove-sdk/ros/install/setup.bash", unit
        )

    def test_runner_adapter_unit_does_not_restart_or_broaden_process_kills(self):
        unit = (FOXGLOVE_DIR / "a3/hope-runner-adapter.service").read_text()
        self.assertIn("Restart=no", unit)
        self.assertIn("KillMode=control-group", unit)
        self.assertIn("TimeoutStopSec=3", unit)
        self.assertIn("/usr/local/bin/hope_runner_adapter.py", unit)
        helper = (FOXGLOVE_DIR / "a3/hope_model21800_runner.sh").read_text()
        self.assertNotIn("pkill", helper)
        self.assertNotIn("killall", helper)
        self.assertIn('kill -KILL "${exact_pid}"', helper)

    def test_downloaded_urdf_directory_is_ignored(self):
        ignore = (REPO_ROOT / ".gitignore").read_text().splitlines()
        self.assertIn("/foxglove/urdf/", ignore)


if __name__ == "__main__":
    unittest.main()
