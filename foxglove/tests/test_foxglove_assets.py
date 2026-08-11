import json
from pathlib import Path
import subprocess
import unittest


FOXGLOVE_DIR = Path(__file__).resolve().parents[1]
REPO_DIR = FOXGLOVE_DIR.parent


class FoxgloveAssetInvariantTests(unittest.TestCase):
    def test_formal_names_are_shared_by_runner_and_foxglove(self):
        legacy_version = "v" + str(17)
        aimrt_config = (
            REPO_DIR
            / "a3_deploy/a3_deploy_example/src/a3/a3_deploy_onnx_ref/config"
            / "a3_aimrt_config.pingpong_ros2body.yaml"
        ).read_text()
        aimrt_header = (
            REPO_DIR
            / "a3_deploy/a3_deploy_example/src/a3/a3_deploy_onnx_ref/include"
            / "robot_io/a3_aimrt_backend.hpp"
        ).read_text()
        aimrt_source = (
            REPO_DIR
            / "a3_deploy/a3_deploy_example/src/a3/a3_deploy_onnx_ref/src"
            / "robot_io/a3_aimrt_backend.cpp"
        ).read_text()
        for content in (aimrt_config, aimrt_header):
            self.assertIn("/hope/runner/control_request_flat", content)
            self.assertIn("/hope/runner/state_flat", content)
            self.assertNotIn("/hope/runner/joint_states", content)
            self.assertNotIn(f"/hope/{legacy_version}", content)
        self.assertNotIn("sensor_msgs::msg::JointState", aimrt_source)
        self.assertNotIn("runner_joint_state_publish_fn_(state)", aimrt_source)

        for path in FOXGLOVE_DIR.rglob("*"):
            if "node_modules" in path.parts:
                continue
            self.assertNotIn(legacy_version, path.name.lower())

        forbidden = (
            f"/hope/{legacy_version}",
            f"hope_{legacy_version}",
            f"hope-{legacy_version}",
            f"foxglove/{legacy_version}",
            f"{legacy_version}_model",
        )
        sources = [
            path
            for path in FOXGLOVE_DIR.rglob("*")
            if path.is_file()
            and "node_modules" not in path.parts
            and path.suffix in {".md", ".py", ".yaml", ".xml", ".service", ".tsx", ".css", ".json"}
        ]
        sources.extend(Path(REPO_DIR / "docs/operations").glob("foxglove*.md"))
        for path in sources:
            content = path.read_text(errors="replace").lower()
            for token in forbidden:
                with self.subTest(path=path, token=token):
                    self.assertNotIn(token, content)

    def test_observer_layout_is_read_only(self):
        layout = json.loads(
            (FOXGLOVE_DIR / "layouts/model21800_observer.json").read_text()
        )
        encoded = json.dumps(layout["configById"])
        self.assertNotIn("ServiceCall!", encoded)
        self.assertNotIn("Publish!", encoded)
        self.assertEqual(
            layout["configById"]["Indicator!basefresh"]["path"],
            "/hope/base/fresh.data",
        )
        self.assertEqual(
            layout["configById"]["Plot!countdown"]["paths"][0]["value"],
            "/hope/command/hdu_wall_countdown_s.data",
        )

    def test_observer_has_no_control_or_process_mutation_api(self):
        observer = (
            FOXGLOVE_DIR / "a3/hope_observer.py"
        ).read_text()
        self.assertIn("start_parameter_services=False", observer)
        self.assertNotIn("create_service(", observer)
        self.assertNotIn("subprocess", observer)
        self.assertNotIn("os.kill", observer)
        self.assertNotIn("write_text(", observer)
        self.assertIn('"/a3/base_pose_flat"', observer)
        self.assertIn('"/racket/command_flat"', observer)
        self.assertIn('"/poses"', observer)
        self.assertIn('"/hope/ball/marker"', observer)
        self.assertIn("Marker.SPHERE", observer)
        self.assertIn("marker.lifetime.nanosec = 200_000_000", observer)
        self.assertIn('"/hope/runner/state_flat"', observer)
        self.assertIn('"/hope/opponent/role_confirmed"', observer)
        self.assertIn('"/hope/opponent/summary"', observer)
        self.assertIn("source=INFERRED_FROM_LOCAL_ROLE confirmed=0", observer)
        self.assertIn("source=RUNNER_CONFIRMED", observer)
        self.assertIn("self._bool_message(False)", observer)

    def test_observer_reuses_existing_read_only_bridge_allowlist(self):
        bridge = (FOXGLOVE_DIR / "a3/bridge_params.yaml").read_text()
        self.assertIn('"^/hope/.*"', bridge)
        self.assertIn('client_topic_whitelist: ["(?!)"]', bridge)
        self.assertIn('service_whitelist:', bridge)
        self.assertIn('- "^/hope/safety/trigger_estop$"', bridge)
        self.assertNotIn('/hope/control/', bridge)
        service = (
            FOXGLOVE_DIR / "a3/hope-observer.service"
        ).read_text()
        self.assertIn(
            "FASTRTPS_DEFAULT_PROFILES_FILE=/etc/hope-foxglove/fastdds_bridge_profile.xml",
            service,
        )

    def test_control_bridge_is_separate_and_exactly_allowlisted(self):
        params = (
            FOXGLOVE_DIR / "a3/bridge_params_control.yaml"
        ).read_text()
        self.assertIn("port: 8766", params)
        self.assertIn('"^/hope/(observer_alive|session/.*', params)
        self.assertIn('"^/hope/safety/trigger_estop$"', params)
        self.assertIn('"^/hope/calibrate$"', params)
        self.assertIn('"^/hope/refresh_x_hit$"', params)
        for action in (
            "set_server",
            "set_receiver",
            "enter_pd_stand",
            "enter_motion",
            "emergency_passive",
            "ready_to_serve",
            "serve",
        ):
            self.assertIn(f'"^/hope/runner/{action}$"', params)
        for service in (
            "apply_config",
            "start",
            "kill_all_and_collect",
        ):
            self.assertIn(f'"^/hope/lifecycle/{service}$"', params)
        self.assertNotIn('"^/hope/.*"', params)
        self.assertIn("root_dispersion_ms", params)
        self.assertIn("message_fresh", params)
        self.assertIn("tf_ready", params)
        self.assertIn("pelvis/(pose|text|marker|tf)", params)
        self.assertNotIn('"^/tf$"', params)
        self.assertNotIn('"^/tf_static$"', params)
        self.assertIn('param_whitelist: ["(?!)"]', params)
        self.assertIn('client_topic_whitelist: ["(?!)"]', params)

        fleet_params = (FOXGLOVE_DIR / "a3/bridge_params.yaml").read_text()
        self.assertNotIn("refresh_x_hit", fleet_params)
        self.assertNotIn('"^/tf$"', fleet_params)
        self.assertNotIn('"^/tf_static$"', fleet_params)
        self.assertIn("port: 8765", fleet_params)

    def test_hdu_units_share_one_configurable_laptop_peer_file(self):
        units = (
            "a3/hope-monitor.service",
            "a3/hope-foxglove-bridge.service",
            "a3/hope-observer.service",
            "a3/hope-command-proxy.service",
            "a3/hope-foxglove-control-bridge.service",
        )
        for relative in units:
            with self.subTest(unit=relative):
                content = (FOXGLOVE_DIR / relative).read_text()
                self.assertIn(
                    "EnvironmentFile=-/etc/hope-foxglove/network.env",
                    content,
                )
                self.assertNotIn("172.23.21.67", content)
        example = (FOXGLOVE_DIR / "a3/network.env.example").read_text()
        self.assertIn("ROS_STATIC_PEERS=", example)
        self.assertNotIn("172.23.21.67", example)
        profile = (FOXGLOVE_DIR / "a3/fastdds_bridge_profile.xml").read_text()
        self.assertNotIn("172.23.20.135", profile)
        self.assertNotIn("10.42.10.10", profile)
        self.assertNotIn("interfaceWhiteList", profile)
        self.assertIn("<type>UDPv4</type>", profile)
        self.assertIn("<useBuiltinTransports>false</useBuiltinTransports>", profile)

    def test_fastdds_wrapper_generates_explicit_initial_peers(self):
        wrapper = (
            REPO_DIR
            / "hope_ws/src/hope_bringup/scripts/with_fastdds_unicast.sh"
        ).read_text()
        self.assertIn("<initialPeersList>", wrapper)
        self.assertIn("<address>${peer}</address>", wrapper)
        self.assertIn("<maxInitialPeersRange>${MAX_INITIAL_PEERS}</maxInitialPeersRange>", wrapper)
        self.assertIn('for peer in "${PEERS[@]}"; do', wrapper)

        runbook = (
            REPO_DIR / "docs/operations/foxglove_first_hardware_test.md"
        ).read_text()
        self.assertIn("initialPeersList", runbook)
        self.assertIn("NO FRESH authoritative mocap base pose", runbook)

    def test_control_layout_has_only_fixed_local_actions_x_hit_and_estop(self):
        layout = json.loads(
            (
                FOXGLOVE_DIR
                / "layouts/model21800_control.json"
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
                "/hope/calibrate",
                "/hope/refresh_x_hit",
                "/hope/runner/emergency_passive",
                "/hope/runner/enter_motion",
                "/hope/runner/enter_pd_stand",
                "/hope/runner/ready_to_serve",
                "/hope/runner/serve",
                "/hope/runner/set_receiver",
                "/hope/runner/set_server",
                "/hope/safety/trigger_estop",
            ],
        )
        self.assertIn("PD_STAND", panels["ServiceCall!calibration"]["buttonTooltip"])
        self.assertIn("world→pelvis", panels["ServiceCall!calibration"]["buttonTooltip"])
        self.assertIn("Does not refresh x_hit", panels["ServiceCall!calibration"]["buttonTooltip"])
        self.assertEqual(panels["ServiceCall!calibration"]["timeoutSeconds"], 40)
        self.assertEqual(
            panels["ServiceCall!refreshxhit"]["serviceName"],
            "/hope/refresh_x_hit",
        )
        self.assertIn(
            "loses support",
            panels["ServiceCall!passive"]["buttonText"].lower(),
        )
        self.assertIn("does not control the opponent", panels["ServiceCall!setserver"]["buttonTooltip"])
        self.assertEqual(
            panels["RawMessages!opponent"]["topicPath"],
            "/hope/opponent/summary",
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
            "/hope/mocap/p1_marker_text",
        )
        self.assertEqual(
            panels["RawMessages!pelvis"]["topicPath"], "/hope/pelvis/text"
        )
        self.assertEqual(
            panels["Plot!cpuload"]["paths"][0]["value"],
            "/hope/system/cpu_load_percent.data",
        )
        scene = panels["3D!operator"]
        table = scene["layers"]["urdf-table"]
        self.assertEqual(table["layerId"], "foxglove.Urdf")
        self.assertIn("hope_ping_pong_table.urdf", table["url"])
        self.assertEqual(
            sum(
                layer["layerId"] == "foxglove.Urdf"
                for layer in scene["layers"].values()
            ),
            1,
        )
        self.assertTrue(scene["scene"]["transforms"]["showLabel"])
        self.assertEqual(scene["transforms"], {})
        self.assertNotIn("/hope/robot_description", json.dumps(scene))
        self.assertNotIn("/joint_states", json.dumps(scene))
        self.assertTrue(scene["topics"]["/hope/ball/marker"]["visible"])
        self.assertNotIn("Publish!", json.dumps(panels))

    def test_laptop_marker_interface_uses_real_physical_samples(self):
        marker_node = (
            FOXGLOVE_DIR / "laptop/hope_marker_monitor.py"
        ).read_text()
        marker_core = (
            FOXGLOVE_DIR / "laptop/hope_marker_monitor_core.py"
        ).read_text()
        self.assertIn("RigidBodyMarkerArray", marker_node)
        self.assertIn('"/optitrack/rigid_body_markers"', marker_node)
        self.assertIn('"/hope/mocap/p1_marker_count"', marker_node)
        self.assertIn("has_live_sample", marker_core)
        self.assertIn("params & 0x01", marker_core)
        self.assertIn("params & 0x02", marker_core)
        self.assertNotIn("create_service(", marker_node)

        lifecycle_helper = (
            FOXGLOVE_DIR / "helpers/hope-lifecycle"
        ).read_text()
        self.assertIn("LAPTOP_SHARE", lifecycle_helper)
        self.assertIn("hope_marker_monitor.py", lifecycle_helper)
        self.assertIn("marker_monitor.log", lifecycle_helper)
        self.assertIn('--peer "$hdu_ip"', lifecycle_helper)
        self.assertIn("set +u\n  source /opt/ros/jazzy/setup.bash", lifecycle_helper)
        self.assertIn("set +u\n  source /agibot/software/v0/entry/env/env.sh", lifecycle_helper)

    def test_laptop_asset_server_is_local_only(self):
        unit = (
            FOXGLOVE_DIR / "laptop/hope-foxglove-assets.service"
        ).read_text()
        self.assertIn(
            "WorkingDirectory=/home/dongc1/workspace/HOPE_OPEN/foxglove",
            unit,
        )
        self.assertIn("python3 -m http.server 8000 --bind 127.0.0.1", unit)

    def test_command_proxy_accepts_no_generic_execution_input(self):
        proxy = (
            FOXGLOVE_DIR / "a3/hope_command_proxy.py"
        ).read_text()
        self.assertIn("start_parameter_services=False", proxy)
        self.assertIn("self.create_service(", proxy)
        self.assertIn('"/hope/calibrate"', proxy)
        self.assertIn('"/a3/calibration/recompute_p1"', proxy)
        self.assertIn('"/a3/base_pose_flat"', proxy)
        self.assertIn('urgent=action_name == "EMERGENCY_PASSIVE"', proxy)
        self.assertIn("NOT CALIBRATED FOR CURRENT RUNNER SESSION", proxy)
        calibration_body = proxy.split("    def _calibrate(", 1)[1].split(
            "    def _refresh_x_hit(", 1
        )[0]
        self.assertNotIn("publish_x_hit_request", calibration_body)
        refresh_body = proxy.split("    def _refresh_x_hit(", 1)[1]
        self.assertIn("publish_x_hit_request", refresh_body)
        self.assertIn('"/hope/runner/control_request_flat"', proxy)
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

    def test_custom_console_maps_directly_to_native_runner_contract(self):
        extension = (
            FOXGLOVE_DIR
            / "extensions/hope-a3-console/src/HopeA3Console.tsx"
        ).read_text()
        package = json.loads(
            (
                FOXGLOVE_DIR / "extensions/hope-a3-console/package.json"
            ).read_text()
        )
        layout = json.loads(
            (FOXGLOVE_DIR / "layouts/model21800_console.json").read_text()
        )
        self.assertEqual(package["displayName"], "HOPE A3 Console")
        for service in (
            "/hope/safety/trigger_estop",
            "/hope/calibrate",
            "/hope/refresh_x_hit",
            "/hope/runner/set_server",
            "/hope/runner/set_receiver",
            "/hope/runner/enter_pd_stand",
            "/hope/runner/enter_motion",
            "/hope/runner/emergency_passive",
            "/hope/runner/ready_to_serve",
            "/hope/runner/serve",
            "/hope/lifecycle/apply_config",
            "/hope/lifecycle/start",
            "/hope/lifecycle/kill_all_and_collect",
        ):
            self.assertIn(service, extension)
        self.assertNotIn("/hope/lifecycle/stop_and_collect", extension)
        self.assertNotIn("/hope/control/enter_", extension)
        self.assertNotIn("context.publish", extension)
        self.assertIn('key === "refreshXHit"', extension)
        self.assertIn('? 7_000', extension)
        self.assertIn("E-STOP ASSERTED", extension)
        self.assertIn("CLICK TO REASSERT", extension)
        self.assertIn("BACKEND UNKNOWN · CLICK STILL ASSERTS", extension)
        self.assertIn("disabled={busy.estop === true}", extension)
        self.assertNotIn(
            "disabled={!estopUsable || busy.estop === true || estopAsserted}",
            extension,
        )
        self.assertIn(
            "disabled={snapshot.roleChangeAllowed !== true || busy.setServer === true}",
            extension,
        )
        self.assertIn(
            "disabled={!runnerUsable || snapshot.standing !== true}", extension
        )
        self.assertIn(
            "disabled={!runnerUsable || snapshot.standing !== true || !baseFresh}",
            extension,
        )
        self.assertIn('baseFresh: "/hope/base/fresh"', extension)
        self.assertIn(
            "disabled={!runnerUsable || !serveAvailable}", extension
        )
        self.assertIn("stale telemetry must never make the panel look reset", extension)
        self.assertIn('"/hope/safety/estop_latched"', extension)
        self.assertIn('"/hope/system/cpu_top_process"', extension)
        self.assertIn("LAST UI REQUEST", extension)
        self.assertIn(
            "hope-a3-console.HOPE A3 Console!operator", layout["configById"]
        )
        self.assertEqual(
            layout["layout"]["second"],
            "hope-a3-console.HOPE A3 Console!operator",
        )
        self.assertNotIn("HOPE A3 Console!operator", layout["configById"])
        self.assertEqual(layout["layout"]["splitPercentage"], 60)
        scene = layout["configById"]["3D!a3tf"]
        table = scene["layers"]["urdf-table"]
        self.assertEqual(table["layerId"], "foxglove.Urdf")
        self.assertIn("hope_ping_pong_table.urdf", table["url"])
        self.assertEqual(
            sum(
                layer["layerId"] == "foxglove.Urdf"
                for layer in scene["layers"].values()
            ),
            1,
        )
        self.assertTrue(scene["scene"]["transforms"]["showLabel"])
        self.assertEqual(scene["transforms"], {})
        self.assertNotIn("/hope/robot_description", json.dumps(scene))
        self.assertNotIn("/joint_states", json.dumps(scene))
        self.assertTrue(scene["topics"]["/hope/pelvis/marker"]["visible"])
        self.assertTrue(scene["topics"]["/hope/ball/marker"]["visible"])

    def test_lifecycle_surface_is_fixed_and_has_no_browser_shell(self):
        supervisor = (
            FOXGLOVE_DIR / "a3/hope_lifecycle_supervisor.py"
        ).read_text()
        core = (
            FOXGLOVE_DIR / "a3/hope_lifecycle_core.py"
        ).read_text()
        helper = (
            FOXGLOVE_DIR / "helpers/hope-lifecycle"
        ).read_text()
        service = (
            FOXGLOVE_DIR / "a3/hope-lifecycle-supervisor.service"
        ).read_text()
        extension = (
            FOXGLOVE_DIR / "extensions/hope-a3-console/src/HopeA3Console.tsx"
        ).read_text()

        self.assertIn("start_parameter_services=False", supervisor)
        self.assertNotIn("shell=True", supervisor)
        self.assertIn('LIFECYCLE_HELPER = "/usr/local/libexec/hope-lifecycle"', supervisor)
        self.assertIn('"/hope/lifecycle/apply_config"', supervisor)
        self.assertIn('"/hope/lifecycle/start"', supervisor)
        self.assertIn('"/hope/lifecycle/kill_all_and_collect"', supervisor)
        self.assertNotIn('"/hope/lifecycle/stop_and_collect"', supervisor)
        self.assertIn('"/hope/runner/mode"', supervisor)
        self.assertIn('"/hope/runner/session_matches"', supervisor)
        self.assertIn("RUNNER_START_VERIFY_TIMEOUT_S = 15.0", supervisor)
        self.assertIn('self._step = "RUNNER_VERIFY"', supervisor)
        self.assertIn(
            "authoritative Runner did not publish fresh PASSIVE state",
            supervisor,
        )
        self.assertIn('"PASSIVE"', supervisor)
        self.assertNotIn(
            "put the authoritative Runner in fresh PASSIVE or PD_STAND",
            supervisor,
        )
        for field in (
            "laptop_wifi_ip",
            "hdu_wifi_ip",
            "mdu_internal_ip",
            "motive_ip",
        ):
            self.assertIn(field, core)
            self.assertIn(field, extension)
        self.assertIn("configuration request must contain all four", core)
        self.assertIn("RFC1918", core)
        self.assertNotIn("eval ", helper)
        self.assertNotIn("sshpass", helper)
        self.assertNotIn("pkill", helper)
        self.assertNotIn("killall", helper)
        self.assertIn("--no-fall-guard", helper)
        self.assertIn("--start passive", helper)
        self.assertIn("ros2 pkg prefix hope_msgs", helper)
        self.assertIn("hope_msgs/msg/BallFlightPacket", helper)
        self.assertIn("HOPE_MSGS_STALE", helper)
        self.assertIn("PLANNER_OVERLAY_STALE", helper)
        self.assertIn("cgroup_path_in_systemd_unit", helper)
        self.assertIn("has_unmanaged_hal", helper)
        self.assertIn("HAL_REMAINS_AFTER_AGIBOT_PM_STOP", helper)
        self.assertIn("NO_REMOTE_SESSION_LOGS", helper)
        self.assertIn("PARTIAL_LOGS_COLLECTED", helper)
        self.assertIn("collection_status.txt", helper)
        self.assertIn("HDU_UNREACHABLE", helper)
        self.assertIn("MDU_UNREACHABLE", helper)
        self.assertNotIn("pgrep -u agi -f '[h]ope_planner_cpp_node'", helper)
        self.assertIn(
            "pgrep -u agi -f '(^|/)hope_planner_cpp_node([[:space:]]|$)'",
            helper,
        )
        self.assertIn('"KILL_COMPLETE_AGIBOT_PM_RESTORED_" + collection_reason', supervisor)
        self.assertIn("Lifecycle start failed for {session_id}", supervisor)
        self.assertIn("Lifecycle kill failed for {session_id}", supervisor)
        self.assertIn('"start-hal", *common), 120.0', supervisor)
        self.assertNotIn("get_logger().exception", supervisor)
        self.assertIn('errors="replace"', supervisor)
        self.assertIn("START_INTERNAL_ERROR", supervisor)
        self.assertIn("KILL_INTERNAL_ERROR", supervisor)
        self.assertIn("never leave hardware lifecycle busy", supervisor)
        self.assertIn("KILL ALL & COLLECT", extension)
        self.assertIn('lifecycleState === "KILLING"', extension)
        self.assertIn("</dev/null >/dev/null 2>&1", helper)
        self.assertIn("start_world:=true start_calibration:=true", helper)
        self.assertIn(
            "install/motion_capture_tracking/lib/motion_capture_tracking/"
            "motion_capture_tracking_node",
            helper,
        )
        self.assertIn(
            "../NatNet2ROS2/src/motion_capture_tracking/src/"
            "motion_capture_tracking_node.cpp",
            helper,
        )
        self.assertIn(
            "readonly HOPE_ROOT=/home/dongc1/workspace/HOPE_OPEN", helper
        )
        self.assertNotIn("/home/dongc1/workspace/Hope_v11", helper)
        self.assertNotIn("src/hope_bringup/config/optitrack_mct.yaml", helper)
        self.assertIn('motive_hostname:="$motive_ip"', helper)
        run_laptop_outer = helper.split("run_laptop() {", 1)[1].split(
            '"$dds_wrap"', 1
        )[0]
        self.assertIn('hope_root="$4"', run_laptop_outer)
        self.assertNotIn('hope_root="$5"', run_laptop_outer)
        self.assertIn("HDU_TRANSPORT_RELAY_RUNNING", helper)
        self.assertIn("/a3/base_pose_laptop_flat", helper)
        self.assertIn("hope-base-pose-transport-relay", helper)
        preflight_mdu = helper.split("preflight_mdu() {", 1)[1].split("\n}", 1)[0]
        self.assertLess(
            preflight_mdu.index("systemctl is-active --quiet agibot_pm.service"),
            preflight_mdu.index("has_unmanaged_hal"),
        )
        start_hal = helper.split("start_hal() {", 1)[1].split("\n}", 1)[0]
        self.assertLess(
            start_hal.index("sudo -n systemctl stop agibot_pm.service"),
            start_hal.index("HAL_REMAINS_AFTER_AGIBOT_PM_STOP"),
        )
        self.assertIn("start_parameter_services=False", supervisor)
        self.assertIn("StateDirectory=hope-lifecycle", service)
        self.assertIn("ROS_LOG_DIR=/var/lib/hope-lifecycle/ros-log", service)
        self.assertIn(
            "ExecStartPre=/usr/bin/install -d -m 0700 /var/lib/hope-lifecycle/ros-log",
            service,
        )

        runbook = (
            REPO_DIR / "docs/operations/foxglove_first_hardware_test.md"
        ).read_text()
        self.assertIn("--packages-up-to hope_planner_cpp", runbook)
        self.assertIn("--cmake-clean-cache", runbook)
        self.assertIn("src/hope_msgs/msg/BallFlightPacket.msg", runbook)
        self.assertIn("ros2 interface show hope_msgs/msg/BallFlightPacket", runbook)
        self.assertIn("HDU_PLANNER_OVERLAY_OK", runbook)
        self.assertNotIn("ssh-copy-id dongc1@172.23.20.46", runbook)
        self.assertIn("HDU_PUBLIC_KEY", runbook)
        self.assertIn("HDU_TO_LAPTOP_OK", runbook)
        self.assertIn("https://foxglove.dev/download", runbook)
        self.assertIn("foxglove-studio-latest-linux-amd64.deb", runbook)
        self.assertNotIn("sudo apt install ./foxglove-studio-*.deb", runbook)
        self.assertIn("Install local extension", runbook)
        self.assertIn("Open connection", runbook)
        self.assertIn("ws://172.23.20.135:8766", runbook)
        self.assertIn("Unknown panel type: HOPE A3 Console", runbook)
        self.assertIn("Add panel", runbook)
        self.assertIn("hopeopen.hope-a3-console-1.2.4", runbook)
        self.assertIn("hope-a3-console.HOPE A3 Console!operator", runbook)
        self.assertIn('cmp "$SRC/dist/extension.js" "$EXT/dist/extension.js"', runbook)
        self.assertIn("agibot-clock-bootstrap.service", runbook)
        self.assertIn("chronyc waitsync 600 0.010 5 2", runbook)
        self.assertIn("hope-clock-active-mdu", runbook)
        self.assertIn('systemctl cat "$SERVICE"', runbook)
        self.assertIn('mapfile -t ACTIVE_SERVICES', runbook)
        self.assertIn("distrobox enter hope", runbook)
        self.assertIn("with_fastdds_unicast.sh", runbook)
        self.assertIn("Laptop HOST 没有 `/opt/ros/jazzy`", runbook)

    def test_lifecycle_vendor_hal_cgroup_matching_is_exact(self):
        helper = FOXGLOVE_DIR / "helpers/hope-lifecycle"
        script = r'''
source "$1"
cgroup_path_in_systemd_unit \
  /system.slice/agibot_pm.service agibot_pm.service
cgroup_path_in_systemd_unit \
  /system.slice/agibot_pm.service/vendor.scope agibot_pm.service
! cgroup_path_in_systemd_unit \
  /system.slice/agibot_pm.service.evil agibot_pm.service
! cgroup_path_in_systemd_unit \
  /user.slice/agibot_pm.service agibot_pm.service
'''
        subprocess.run(
            ["bash", "-c", script, "bash", str(helper)],
            check=True,
            capture_output=True,
            text=True,
        )

    def test_lifecycle_base_start_is_process_based_not_a_data_gate(self):
        helper = (FOXGLOVE_DIR / "helpers/hope-lifecycle").read_text()
        start_base = helper.split("start_base() {", 1)[1].split("\n}\n", 1)[0]
        self.assertNotIn("ros2 topic type", start_base)
        self.assertNotIn("ros2 topic info", start_base)
        self.assertNotIn("ros2 service type", start_base)
        self.assertIn("hope-base-pose-transport-relay", start_base)
        self.assertIn("gates only the Foxglove Ready button", start_base)


if __name__ == "__main__":
    unittest.main()
