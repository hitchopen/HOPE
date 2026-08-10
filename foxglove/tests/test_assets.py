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

    def test_layout_is_a3_rooted_and_has_five_audited_controls(self):
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
        expected_services = {
            "CallService!estop": "/hope/safety/trigger_estop",
            "CallService!prepare": "/hope/control/enter_prepare",
            "CallService!policy": "/hope/control/enter_policy",
            "CallService!exitpolicy": "/hope/control/exit_policy",
            "CallService!passive": "/hope/control/enter_passive",
        }
        for panel_id, service_name in expected_services.items():
            self.assertEqual(
                layout["configById"][panel_id]["serviceName"], service_name
            )
            self.assertFalse(layout["configById"][panel_id]["editingMode"])
        self.assertNotIn("Publish!", json.dumps(layout["configById"]))

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

    def test_bridge_exposes_only_five_audited_services_and_no_client_publish(self):
        params = (FOXGLOVE_DIR / "a3/bridge_params.yaml").read_text()
        self.assertNotIn("- clientPublish", params)
        self.assertNotIn("- assets", params)
        self.assertNotIn('^/motion/control/.*_joint_state$', params)
        self.assertIn('client_topic_whitelist: ["(?!)"]', params)
        for service in (
            "/hope/safety/trigger_estop",
            "/hope/control/enter_prepare",
            "/hope/control/enter_policy",
            "/hope/control/exit_policy",
            "/hope/control/enter_passive",
        ):
            self.assertIn(f'- "^{service}$"', params)
        parsed = yaml.safe_load(params)["/**"]["ros__parameters"]
        self.assertEqual(
            parsed["service_whitelist"],
            [
                "^/hope/safety/trigger_estop$",
                "^/hope/control/enter_prepare$",
                "^/hope/control/enter_policy$",
                "^/hope/control/exit_policy$",
                "^/hope/control/enter_passive$",
            ],
        )
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
        self.assertIn("combine_estop_results", monitor)
        self.assertIn('"/hope/runner/emergency_stop"', monitor)
        self.assertIn("build_software_estop_request", monitor)
        self.assertIn("decode_software_estop_response", monitor)
        self.assertIn("client.service_is_ready()", monitor)
        self.assertIn("self.destroy_service(self._estop_proxy_service)", monitor)
        self.assertIn("lock is not held across vendor I/O", monitor)
        self.assertNotIn("software_emergency_stop = False", monitor)
        self.assertNotIn("clear_estop", monitor.lower())

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
