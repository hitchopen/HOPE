import json
from pathlib import Path
import unittest


FOXGLOVE_DIR = Path(__file__).resolve().parents[1]


class V17AssetInvariantTests(unittest.TestCase):
    def test_observer_layout_is_read_only(self):
        layout = json.loads(
            (FOXGLOVE_DIR / "layouts/v17_model21800_observer.json").read_text()
        )
        encoded = json.dumps(layout["configById"])
        self.assertNotIn("ServiceCall!", encoded)
        self.assertNotIn("Publish!", encoded)
        self.assertEqual(
            layout["configById"]["Indicator!basefresh"]["path"],
            "/hope/v17/base/fresh.data",
        )
        self.assertEqual(
            layout["configById"]["Plot!countdown"]["paths"][0]["value"],
            "/hope/v17/command/hdu_wall_countdown_s.data",
        )

    def test_observer_has_no_control_or_process_mutation_api(self):
        observer = (
            FOXGLOVE_DIR / "v17/a3/hope_v17_observer.py"
        ).read_text()
        self.assertIn("start_parameter_services=False", observer)
        self.assertNotIn("create_service(", observer)
        self.assertNotIn("subprocess", observer)
        self.assertNotIn("os.kill", observer)
        self.assertNotIn("write_text(", observer)
        self.assertIn('"/a3/base_pose_flat"', observer)
        self.assertIn('"/racket/command_flat"', observer)
        self.assertIn('"/poses"', observer)
        self.assertIn('"/hope/v17/runner/state_flat"', observer)
        self.assertIn('"/hope/v17/opponent/role_confirmed"', observer)
        self.assertIn('"/hope/v17/opponent/summary"', observer)
        self.assertIn("source=INFERRED_FROM_LOCAL_ROLE confirmed=0", observer)
        self.assertIn("source=RUNNER_CONFIRMED", observer)
        self.assertIn("self._bool_message(False)", observer)

    def test_phase_one_reuses_existing_read_only_bridge_allowlist(self):
        bridge = (FOXGLOVE_DIR / "a3/bridge_params.yaml").read_text()
        self.assertIn('"^/hope/.*"', bridge)
        self.assertIn('client_topic_whitelist: ["(?!)"]', bridge)
        self.assertIn(
            'service_whitelist: ["^/hope/safety/trigger_estop$"]', bridge
        )
        service = (
            FOXGLOVE_DIR / "v17/a3/hope-v17-observer.service"
        ).read_text()
        self.assertIn(
            "FASTRTPS_DEFAULT_PROFILES_FILE=/etc/hope-foxglove/fastdds_bridge_profile.xml",
            service,
        )

    def test_phase_two_control_bridge_is_separate_and_exactly_allowlisted(self):
        params = (
            FOXGLOVE_DIR / "v17/a3/bridge_params_v17_control.yaml"
        ).read_text()
        self.assertIn("port: 8766", params)
        self.assertIn('"^/hope/safety/trigger_estop$"', params)
        self.assertIn('"^/hope/v17/refresh_x_hit$"', params)
        for action in (
            "set_server",
            "set_receiver",
            "enter_pd_stand",
            "enter_motion",
            "emergency_passive",
            "ready_to_serve",
            "serve",
        ):
            self.assertIn(f'"^/hope/v17/runner/{action}$"', params)
        self.assertNotIn('"^/hope/.*"', params)
        self.assertIn('param_whitelist: ["(?!)"]', params)
        self.assertIn('client_topic_whitelist: ["(?!)"]', params)

        fleet_params = (FOXGLOVE_DIR / "a3/bridge_params.yaml").read_text()
        self.assertNotIn("refresh_x_hit", fleet_params)
        self.assertIn("port: 8765", fleet_params)

    def test_control_layout_has_only_fixed_local_actions_x_hit_and_estop(self):
        layout = json.loads(
            (
                FOXGLOVE_DIR
                / "layouts/v17_model21800_control_phase2.json"
            ).read_text()
        )
        panels = layout["configById"]
        service_names = sorted(
            panel["serviceName"]
            for panel_id, panel in panels.items()
            if panel_id.startswith("ServiceCall!")
        )
        self.assertEqual(
            service_names,
            [
                "/hope/safety/trigger_estop",
                "/hope/v17/refresh_x_hit",
                "/hope/v17/runner/emergency_passive",
                "/hope/v17/runner/enter_motion",
                "/hope/v17/runner/enter_pd_stand",
                "/hope/v17/runner/ready_to_serve",
                "/hope/v17/runner/serve",
                "/hope/v17/runner/set_receiver",
                "/hope/v17/runner/set_server",
            ],
        )
        self.assertIn("PD_STAND", panels["ServiceCall!calibration"]["buttonTooltip"])
        self.assertIn(
            "loses support",
            panels["ServiceCall!passive"]["buttonText"].lower(),
        )
        self.assertIn("does not control the opponent", panels["ServiceCall!setserver"]["buttonTooltip"])
        self.assertEqual(
            panels["RawMessages!opponent"]["topicPath"],
            "/hope/v17/opponent/summary",
        )
        self.assertEqual(
            panels["RawMessages!ntp"]["topicPath"], "/hope/ntp/text"
        )
        self.assertEqual(
            panels["RawMessages!latency"]["topicPath"],
            "/hope/clock/message_text",
        )
        self.assertEqual(
            panels["RawMessages!markers"]["topicPath"],
            "/hope/v17/mocap/p1_marker_text",
        )
        self.assertEqual(
            panels["RawMessages!pelvis"]["topicPath"], "/hope/pelvis/text"
        )
        self.assertEqual(
            panels["Plot!cpuload"]["paths"][0]["value"],
            "/hope/system/cpu_load_percent.data",
        )
        scene = panels["3D!operator"]
        self.assertEqual(
            scene["layers"]["urdf-a3"]["topic"], "/hope/robot_description"
        )
        self.assertIn(
            "hope_ping_pong_table.urdf",
            scene["layers"]["urdf-table"]["url"],
        )
        self.assertNotIn("Publish!", json.dumps(panels))

    def test_laptop_marker_interface_uses_real_physical_samples(self):
        marker_node = (
            FOXGLOVE_DIR / "v17/laptop/hope_v17_marker_monitor.py"
        ).read_text()
        marker_core = (
            FOXGLOVE_DIR / "v17/laptop/hope_v17_marker_monitor_core.py"
        ).read_text()
        self.assertIn("RigidBodyMarkerArray", marker_node)
        self.assertIn('"/optitrack/rigid_body_markers"', marker_node)
        self.assertIn('"/hope/v17/mocap/p1_marker_count"', marker_node)
        self.assertIn("has_live_sample", marker_core)
        self.assertIn("params & 0x01", marker_core)
        self.assertIn("params & 0x02", marker_core)
        self.assertNotIn("create_service(", marker_node)

    def test_command_proxy_accepts_no_generic_execution_input(self):
        proxy = (
            FOXGLOVE_DIR / "v17/a3/hope_v17_command_proxy.py"
        ).read_text()
        self.assertIn("start_parameter_services=False", proxy)
        self.assertIn("self.create_service(", proxy)
        self.assertIn('"/hope/v17/refresh_x_hit"', proxy)
        self.assertIn('"/hope/v17/runner/control_request_flat"', proxy)
        for action in (
            "set_server",
            "set_receiver",
            "enter_pd_stand",
            "enter_motion",
            "emergency_passive",
            "ready_to_serve",
            "serve",
        ):
            self.assertIn(f'("{action}",', proxy)
        self.assertNotIn("subprocess", proxy)
        self.assertNotIn("os.kill", proxy)
        self.assertNotIn("Popen", proxy)
        self.assertNotIn("shell=True", proxy)


if __name__ == "__main__":
    unittest.main()
