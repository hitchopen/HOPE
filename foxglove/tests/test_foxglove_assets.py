import json
from pathlib import Path
import subprocess
import tempfile
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
        self.assertIn('"/hope/runner/state_hdu_flat"', observer)
        self.assertNotIn('"/hope/runner/state_flat"', observer)
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
        self.assertIn('"^/hope/calibrate_v2$"', params)
        self.assertIn('"^/hope/calibrate_v3$"', params)
        self.assertNotIn('"^/hope/calibrate$"', params)
        self.assertIn('"^/hope/refresh_x_hit$"', params)
        for action in (
            "set_server",
            "set_receiver",
            "enter_pd_stand",
            "enter_motion",
            "emergency_passive",
            "prepare_serve",
            "confirm_ball_loaded",
            "ready_to_serve",
            "open_gripper",
        ):
            self.assertIn(f'"^/hope/runner/{action}$"', params)
        self.assertNotIn("confirm_loading_zone_clear", params)
        for service in (
            "apply_config",
            "start",
            "kill_all_and_collect",
            "time_calibration",
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
            "a3/hope-time-calibration.service",
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
        self.assertIn("MDU internal address", example)
        self.assertNotIn("172.23.21.67", example)
        profile = (FOXGLOVE_DIR / "a3/fastdds_bridge_profile.xml").read_text()
        self.assertNotIn("172.23.20.135", profile)
        self.assertNotIn("172.23.21.123", profile)
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

        explicit_peer_units = (
            "a3/hope-command-proxy.service",
            "a3/hope-foxglove-bridge.service",
            "a3/hope-foxglove-control-bridge.service",
        )
        for relative in explicit_peer_units:
            with self.subTest(unit=relative):
                unit = (FOXGLOVE_DIR / relative).read_text()
                self.assertIn("ExecStart=:/bin/bash -lc", unit)
                self.assertIn(
                    'IFS=";" read -r -a hope_static_peers', unit
                )
                self.assertIn(
                    'hope_peer_args+=(--peer "$hope_peer")', unit
                )
                self.assertIn(
                    "/with_fastdds_unicast.sh \\\n"
                    '    --domain-id "$ROS_DOMAIN_ID" '
                    '"${hope_peer_args[@]}" --',
                    unit,
                )

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
                "/hope/calibrate_v2",
                "/hope/calibrate_v3",
                "/hope/refresh_x_hit",
                "/hope/runner/emergency_passive",
                "/hope/runner/enter_motion",
                "/hope/runner/enter_pd_stand",
                "/hope/runner/open_gripper",
                "/hope/runner/prepare_serve",
                "/hope/runner/ready_to_serve",
                "/hope/runner/set_receiver",
                "/hope/runner/set_server",
                "/hope/safety/trigger_estop",
            ],
        )
        for profile in ("v2", "v3"):
            calibration = panels[f"ServiceCall!calibration{profile}"]
            self.assertIn("PD_STAND", calibration["buttonTooltip"])
            self.assertIn("world→pelvis", calibration["buttonTooltip"])
            self.assertIn("Does not refresh x_hit", calibration["buttonTooltip"])
            self.assertEqual(calibration["timeoutSeconds"], 40)
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
            "/hope/mocap/marker_text",
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
        for topic in (
            "/hope/mocap/marker_asset",
            "/hope/mocap/marker_count",
            "/hope/mocap/marker_fresh",
            "/hope/mocap/markers_complete",
            "/hope/mocap/marker_text",
        ):
            self.assertIn(f'"{topic}"', marker_node)
        # Old layouts remain usable during a staged update.
        self.assertNotIn('"/hope/mocap/p1_marker_count"', marker_node)
        self.assertIn("has_live_sample", marker_core)
        self.assertIn("params & 0x01", marker_core)
        self.assertIn("params & 0x02", marker_core)
        self.assertNotIn("create_service(", marker_node)

        lifecycle_helper = (
            FOXGLOVE_DIR / "helpers/hope-lifecycle"
        ).read_text()
        laptop_runtime = (
            FOXGLOVE_DIR / "helpers/hope-laptop-container-runtime"
        ).read_text()
        self.assertIn("LAPTOP_SHARE", lifecycle_helper)
        self.assertIn("hope_marker_monitor.py", laptop_runtime)
        self.assertIn("marker_monitor.log", laptop_runtime)
        self.assertIn('--peer "$HOPE_LIFECYCLE_HDU_IP"', laptop_runtime)
        self.assertIn("set +u\n  source /opt/ros/jazzy/setup.bash", lifecycle_helper)
        self.assertIn("set +u\n  source /agibot/software/v0/entry/env/env.sh", lifecycle_helper)

    def test_laptop_asset_server_is_local_only(self):
        unit = (
            FOXGLOVE_DIR / "laptop/hope-foxglove-assets.service"
        ).read_text()
        self.assertIn(
            "WorkingDirectory=%h/.local/share/hope-foxglove",
            unit,
        )
        self.assertIn("python3 -m http.server 8000 --bind 127.0.0.1", unit)
        self.assertIn("--directory %h/.local/share/hope-foxglove", unit)

    def test_command_proxy_accepts_no_generic_execution_input(self):
        proxy = (
            FOXGLOVE_DIR / "a3/hope_command_proxy.py"
        ).read_text()
        self.assertIn("start_parameter_services=False", proxy)
        self.assertIn("self.create_service(", proxy)
        self.assertIn('f"/hope/calibrate_{profile.lower()}"', proxy)
        self.assertIn('"/a3/calibration/recompute_ucb_robot_v2"', proxy)
        self.assertIn('"/a3/calibration/recompute_ucb_robot_v3"', proxy)
        self.assertIn('for profile in ("STICKERS_V3", "V2", "V3")', proxy)
        self.assertIn('"calibration_service_discovery_timeout_s"', proxy)
        self.assertIn("calibration_client.wait_for_service(", proxy)
        self.assertIn(
            "timeout_sec=self._calibration_service_discovery_timeout_s",
            proxy,
        )
        self.assertIn(
            'self.declare_parameter("calibration_service_discovery_timeout_s", 20.0)',
            proxy,
        )
        self.assertIn(
            'self.declare_parameter("calibration_timeout_s", 45.0)', proxy
        )
        proxy_unit = (FOXGLOVE_DIR / "a3/hope-command-proxy.service").read_text()
        self.assertIn("calibration_service_discovery_timeout_s:=20.0", proxy_unit)
        self.assertIn("calibration_timeout_s:=45.0", proxy_unit)
        self.assertIn('"/a3/base_pose_flat"', proxy)
        self.assertIn("ExternalShutdownException", proxy)
        self.assertIn("rclpy.try_shutdown()", proxy)
        self.assertIn('urgent=action_name == "EMERGENCY_PASSIVE"', proxy)
        self.assertNotIn("NOT CALIBRATED FOR CURRENT RUNNER SESSION", proxy)
        self.assertIn("WAITING FOR MATCHING LIVE BASE", proxy)
        calibration_body = proxy.split("    def _calibrate(", 1)[1].split(
            "    def _refresh_x_hit(", 1
        )[0]
        self.assertNotIn("publish_x_hit_request", calibration_body)
        refresh_body = proxy.split("    def _refresh_x_hit(", 1)[1]
        self.assertIn("publish_x_hit_request", refresh_body)
        self.assertIn('"/hope/runner/control_request_hdu_flat"', proxy)
        self.assertIn('"/hope/runner/state_hdu_flat"', proxy)
        for action in (
            "set_server",
            "set_receiver",
            "enter_pd_stand",
            "enter_motion",
            "emergency_passive",
            "prepare_serve",
            "confirm_ball_loaded",
            "ready_to_serve",
            "open_gripper",
        ):
            self.assertIn(f'"{action}"', proxy)
        self.assertNotIn("confirm_loading_zone_clear", proxy)
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
            "/hope/calibrate_v2",
            "/hope/calibrate_v3",
            "/hope/refresh_x_hit",
            "/hope/runner/set_server",
            "/hope/runner/set_receiver",
            "/hope/runner/enter_pd_stand",
            "/hope/runner/enter_motion",
            "/hope/runner/emergency_passive",
            "/hope/runner/prepare_serve",
            "/hope/runner/confirm_ball_loaded",
            "/hope/runner/ready_to_serve",
            "/hope/runner/open_gripper",
            "/hope/lifecycle/apply_config",
            "/hope/lifecycle/start",
            "/hope/lifecycle/kill_all_and_collect",
            "/hope/lifecycle/time_calibration",
        ):
            self.assertIn(service, extension)
        self.assertNotIn("/hope/runner/confirm_loading_zone_clear", extension)
        self.assertNotIn("Loading zone clear", extension)
        self.assertIn('label="Start to Serve · Raise hand"', extension)
        self.assertIn('label="Ready"', extension)
        self.assertNotIn("/hope/lifecycle/stop_and_collect", extension)
        self.assertNotIn("/hope/control/enter_", extension)
        self.assertNotIn("context.publish", extension)
        self.assertIn('key === "refreshXHit"', extension)
        self.assertIn('label={`Cali V2 · ${markerAsset}`}', extension)
        self.assertIn('label={`Cali V3 · ${markerAsset}`}', extension)
        self.assertIn('"P2 · ROTATE 180°"', extension)
        self.assertIn('tableSide: "/hope/lifecycle/config/table_side"', extension)
        self.assertIn('markerAsset: "/hope/mocap/marker_asset"', extension)
        self.assertIn('markers: "/hope/mocap/marker_count"', extension)
        self.assertNotIn('markersLegacy:', extension)
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
        self.assertIn("runnerStanding", extension)
        self.assertIn("post-contact handoff", extension)
        self.assertIn("Reset Software E-stop", extension)
        self.assertNotIn("SERVER returns to PD_STAND after serve", extension)
        self.assertIn('snapshot.localRole !== "RECEIVER"', extension)
        self.assertIn('snapshot.localRole !== "SERVER"', extension)
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

    def test_time_calibration_is_fixed_attended_and_fail_closed(self):
        coordinator = (
            FOXGLOVE_DIR / "a3/hope_time_calibration.py"
        ).read_text()
        core = (
            FOXGLOVE_DIR / "a3/hope_time_calibration_core.py"
        ).read_text()
        helper = (
            FOXGLOVE_DIR / "helpers/hope-lifecycle"
        ).read_text()
        unit = (
            FOXGLOVE_DIR / "a3/hope-time-calibration.service"
        ).read_text()
        burst_unit = (
            FOXGLOVE_DIR / "a3/hope-chrony-burst.service"
        ).read_text()
        lifecycle_unit = (
            FOXGLOVE_DIR / "a3/hope-lifecycle-supervisor.service"
        ).read_text()
        lifecycle_supervisor = (
            FOXGLOVE_DIR / "a3/hope_lifecycle_supervisor.py"
        ).read_text()
        bridge = (
            FOXGLOVE_DIR / "a3/bridge_params_control.yaml"
        ).read_text()
        extension = (
            FOXGLOVE_DIR / "extensions/hope-a3-console/src/HopeA3Console.tsx"
        ).read_text()
        runbook = (
            REPO_DIR / "docs/operations/foxglove_first_hardware_test.md"
        ).read_text()

        self.assertIn('"/hope/lifecycle/time_calibration"', coordinator)
        self.assertIn("start_parameter_services=False", coordinator)
        self.assertIn('lifecycle_state != "STOPPED"', coordinator)
        self.assertIn("fresh failing NTP gate", coordinator)
        self.assertIn("one hard-step was already attempted", coordinator)
        self.assertIn("REARMED_AFTER_COMPLETED_LIFECYCLE", coordinator)
        self.assertIn("POST_STEP_DDS_RECYCLE", coordinator)
        self.assertIn("raise SystemExit(75)", coordinator)
        self.assertIn("self._status_publishers = {", coordinator)
        self.assertNotIn("self._publishers = {", coordinator)
        self.assertNotIn("shell=True", coordinator)
        self.assertNotIn("shell=True", core)
        self.assertIn('"time-calibration-stop-mdu"', core)
        self.assertIn('"time-calibration-preflight-mdu"', core)
        self.assertIn('"time-calibration-recover-mdu"', core)
        self.assertIn('"time-calibration-restore-mdu"', core)
        self.assertIn(
            'CHRONY_BURST_UNIT = "hope-chrony-burst.service"', core
        )
        self.assertIn("CHRONY_BURST_UNIT_MISSING", core)
        self.assertIn('label="chrony burst"', core)
        self.assertIn('[CHRONYC, "waitsync", "120", "0.010", "0", "1"]', core)
        self.assertIn("robot services stopped", core)
        self.assertIn("time_calibration_require_root", helper)
        self.assertIn("MDU_BASELINE_RECEIPT_MISSING", helper)
        self.assertIn("MDU_BASELINE_ALREADY_READY", helper)
        self.assertIn("INVALID_MDU_RESTORE_RECEIPT", helper)
        time_stop = helper.split("time_calibration_stop_mdu()", 1)[1].split(
            "time_calibration_restore_mdu()", 1
        )[0]
        self.assertIn('/usr/bin/systemctl stop "$service"', time_stop)
        ordered_stop = time_stop.split("# Stop dependants", 1)[1]
        self.assertLess(
            ordered_stop.index("agibot_pm.service"),
            ordered_stop.index("agibot_roudi.service"),
        )
        self.assertNotIn("eval ", helper)
        self.assertIn("User=root", unit)
        self.assertIn("NoNewPrivileges=true", unit)
        self.assertIn("ProtectSystem=strict", unit)
        self.assertIn("hope-lifecycle-supervisor.service", unit)
        self.assertIn("User=_chrony", burst_unit)
        self.assertIn("Group=_chrony", burst_unit)
        self.assertIn(
            "ExecStart=/usr/bin/chronyc -h /run/chrony/chronyd.sock burst 8/8",
            burst_unit,
        )
        self.assertIn("ReadWritePaths=/run/chrony", burst_unit)
        self.assertIn("RestrictAddressFamilies=AF_UNIX", burst_unit)
        self.assertNotIn("[Install]", burst_unit)
        self.assertIn("hardware-operation.lock", lifecycle_unit)
        self.assertIn("try_acquire_hardware_operation_lock", coordinator)
        self.assertIn("try_acquire_hardware_operation_lock", lifecycle_supervisor)
        self.assertIn("release_hardware_operation_lock", lifecycle_supervisor)
        self.assertIn('"^/hope/lifecycle/time_calibration$"', bridge)
        self.assertIn("TIME CALIBRATION", extension)
        self.assertIn("snapshot.ntpPass === false", extension)
        self.assertIn("lifecycleStopped", extension)
        self.assertIn("FAILED_SAFE_STOP", extension)
        self.assertIn("valid same-boot receipt", extension)
        self.assertIn("RECEIPT-SAFE", extension)

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
        laptop_runtime = (
            FOXGLOVE_DIR / "helpers/hope-laptop-container-runtime"
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
            "table_side",
        ):
            self.assertIn(field, core)
            self.assertIn(field, extension)
        self.assertIn(
            "configuration request must contain three required IPv4 fields",
            core,
        )
        self.assertIn("RFC1918", core)
        for script in (helper, laptop_runtime):
            self.assertNotIn("eval ", script)
            self.assertNotIn("sshpass", script)
            self.assertNotIn("pkill", script)
            self.assertNotIn("killall", script)
        self.assertIn("--no-fall-guard", helper)
        self.assertIn('field_runner_args=(--planner)', helper)
        self.assertIn('"${field_runner_args[@]}" --policy-native --serve', helper)
        self.assertNotIn("--trace-csv", helper)
        self.assertNotIn("--obs-csv", helper)
        runner_block = helper.split("run_runner()", 1)[1].split(
            "start_runner()", 1
        )[0]
        self.assertIn("./run_a3_pingpong.sh", runner_block)
        self.assertNotIn("./a3_deploy_onnx_ref_pingpong \\", runner_block)
        self.assertIn("/agibot/a3_deploy_model21800", helper)
        self.assertIn("RUNNER_EXECUTABLE_MISSING", helper)
        self.assertIn("POLICY_FILE_MISSING", helper)
        self.assertIn("POLICY_CONFIG_MISSING", helper)
        self.assertIn("RUNTIME_CONFIG_MISSING", helper)
        self.assertIn("AIMRT_CONFIG_MISSING", helper)
        self.assertIn("SERVE025_TIMELINE_MISSING", helper)
        self.assertIn("GRIPPER_BRIDGE_MISSING", helper)
        self.assertIn(
            "a3p_op3_serve025_photo_right30_advance20_v12.csv",
            helper,
        )
        self.assertNotIn("SPIN001_ENTRY_SHA256_MISMATCH", helper)
        self.assertNotIn("SPIN001_TIMED_SHA256_MISMATCH", helper)
        self.assertNotIn("SPIN001_RECOVERY_SHA256_MISMATCH", helper)
        self.assertNotIn("SERVE_GRIP_SHA256_MISMATCH", helper)
        self.assertNotIn("SERVE_GRIPPER_ROS2_PROTO_MISSING", helper)
        self.assertNotIn("SERVE_GRIPPER_ROSIDL_PARSER_MISSING", helper)
        self.assertNotIn("SERVE_GRIPPER_ROS_IMPORT_FAILED", helper)
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
        self.assertNotIn("pgrep -u agi", helper)
        self.assertIn(
            "pgrep -u \"$ROBOT_USER\" -f '(^|/)hope_planner_cpp_node([[:space:]]|$)'",
            helper,
        )
        self.assertIn('"KILL_COMPLETE_AGIBOT_PM_RESTORED_" + collection_reason', supervisor)
        self.assertIn("Lifecycle start failed for {session_id}", supervisor)
        self.assertIn("Lifecycle kill failed for {session_id}", supervisor)
        self.assertIn('"start-hal", *common), 120.0', supervisor)
        start_commands = supervisor.split("commands = (", 1)[1].split(
            ")\n        for step, argv, timeout_s in commands:", 1
        )[0]
        self.assertNotIn('"start-laptop"', start_commands)
        self.assertIn('config.motive_ip or "NONE"', supervisor)
        kill_commands = supervisor.split("def kill_all_and_collect(", 1)[1].split(
            "errors: list[str]", 1
        )[0]
        self.assertNotIn('"stop-laptop"', kill_commands)
        self.assertIn('"start-runner-transport", *common', supervisor)
        self.assertLess(
            supervisor.index('"start-runner", *common'),
            supervisor.index('"start-runner-transport", *common'),
        )
        self.assertNotIn("get_logger().exception", supervisor)
        self.assertIn('errors="replace"', supervisor)
        self.assertIn("START_INTERNAL_ERROR", supervisor)
        self.assertIn("KILL_INTERNAL_ERROR", supervisor)
        self.assertIn("never leave hardware lifecycle busy", supervisor)
        self.assertIn("KILL ALL & COLLECT", extension)
        self.assertIn('lifecycleState === "KILLING"', extension)
        self.assertIn("</dev/null >/dev/null 2>&1", helper)
        self.assertIn("hope-laptop-container-runtime", helper)
        self.assertIn("start_world:=true start_calibration:=true", laptop_runtime)
        self.assertNotIn(
            "install/motion_capture_tracking/lib/motion_capture_tracking/"
            "motion_capture_tracking_node",
            helper,
        )
        self.assertIn(
            "src/motion_capture_tracking/src/"
            "motion_capture_tracking_node.cpp",
            laptop_runtime,
        )
        self.assertIn(
            "src/motion_capture_tracking/include/motion_capture_tracking/"
            "rigid_body_marker_association.hpp",
            laptop_runtime,
        )
        self.assertIn('readonly HOPE_ROOT="${HOPE_ROOT:-$HOME/HOPE}"', helper)
        self.assertIn("LAPTOP_CONFIG", helper)
        self.assertNotIn("/home/dongc1", helper)
        self.assertNotIn("src/hope_bringup/config/optitrack_mct.yaml", helper)
        self.assertIn('hostname:="$motive_ip"', laptop_runtime)
        self.assertIn("publish_ucb_markers:=true", laptop_runtime)
        self.assertIn('table_side:="$table_side"', laptop_runtime)
        self.assertIn("P1) tracked_asset=UCB_P1", laptop_runtime)
        self.assertIn("P2) tracked_asset=UCB_P2", laptop_runtime)
        self.assertIn('tracked_marker_frame:="$tracked_asset"', laptop_runtime)
        self.assertIn('tracked_pose_topic:="/$tracked_asset/pose"', laptop_runtime)
        self.assertIn("calibration/ucb_robot_to_pelvis.json", laptop_runtime)
        launch = (
            REPO_DIR
            / "hope_ws/src/hope_bringup/launch/optitrack_hope_bridge.launch.py"
        ).read_text()
        self.assertIn(
            '"topics.rigid_body_markers.asset_name":\n'
            "                        tracked_marker_frame",
            launch,
        )
        mct_config = (
            REPO_DIR / "hope_ws/src/hope_bringup/config/optitrack_mct.yaml"
        ).read_text()
        self.assertIn(
            'rigid_body_name_allowlist: ["Ball", "PPT", "UCB_P1", "UCB_P2"]',
            mct_config,
        )
        self.assertNotIn("THU_", mct_config)
        self.assertIn('default_value="UCB_P1"', launch)
        self.assertIn('default_value="/UCB_P1/pose"', launch)
        self.assertIn('"/a3/calibration/recompute_ucb_robot_v2"', launch)
        self.assertIn('"/a3/calibration/recompute_ucb_robot_v3"', launch)
        self.assertIn(
            '"require_marker_identity": publish_ucb_markers',
            launch,
        )
        world_launch = (
            REPO_DIR / "hope_ws/src/hope_bringup/launch/hope_world.launch.py"
        ).read_text()
        self.assertIn('"require_marker_identity": ParameterValue(', world_launch)
        base_relay = (
            REPO_DIR
            / "hope_ws/src/hope_bringup/scripts/hope_base_pose_flat_relay"
        ).read_text()
        self.assertIn("validate_calibration_source_identity", base_relay)
        # Both profile servers use the same selected UCB_P1/UCB_P2 asset, so
        # all four side/profile combinations remain available.
        self.assertEqual(
            launch.count('"asset_name": tracked_marker_frame'),
            3,
        )
        self.assertIn('"marker_layout": "v2"', launch)
        self.assertIn('"marker_layout": "v3"', launch)
        run_laptop_outer = helper.split("run_laptop() {", 1)[1].split(
            "start_laptop()", 1
        )[0]
        self.assertIn("HOPE_LIFECYCLE_ROOT", run_laptop_outer)
        self.assertIn("hope-laptop-container-runtime", run_laptop_outer)
        self.assertIn('hope_root="$5"', laptop_runtime)
        self.assertNotIn('hope_root="$4"', laptop_runtime)
        self.assertIn('marker_root="$1"', laptop_runtime)
        self.assertIn("HDU_TRANSPORT_RELAY_RUNNING", helper)
        self.assertIn("/a3/base_pose_laptop_flat", laptop_runtime)
        self.assertIn("hope-base-pose-transport-relay", helper)
        self.assertIn("hope-runner-transport-relay", helper)
        self.assertIn("BIDIRECTIONAL_DDS_HEALTHY", helper)
        self.assertIn("BIDIRECTIONAL_DDS_NOT_HEALTHY", helper)
        self.assertIn("readonly MDU_ELINK_SESSION=hope-elink", helper)
        self.assertIn("Enter ElinkKeepAliveLoop", helper)
        self.assertIn("ethercat controller start success.", helper)
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
        self.assertLess(
            start_hal.index('start_tmux "$MDU_ELINK_SESSION"'),
            start_hal.index('start_tmux "$MDU_HAL_SESSION"'),
        )
        self.assertIn("ELINK_REMAINS_AFTER_AGIBOT_PM_STOP", start_hal)
        self.assertIn("SERVO_ENABLE_TIMEOUT", start_hal)
        start_runner = helper.split("start_runner() {", 1)[1].split("\n}", 1)[0]
        self.assertIn("HAL_INPUTS_NOT_READY", start_runner)
        self.assertIn('runner_inputs_ready "$runner_log"', start_runner)
        self.assertNotIn("ready=yes samples=", start_runner)
        start_transport = helper.split("start_runner_transport() {", 1)[1].split(
            "\n}", 1
        )[0]
        self.assertIn("RUNNER TRANSPORT HEALTHY", start_transport)
        self.assertIn("BIDIRECTIONAL_DDS_NOT_HEALTHY", start_transport)
        stop_mdu = helper.split("\nstop_mdu() {", 1)[1].split("\n}", 1)[0]
        self.assertLess(
            stop_mdu.index('stop_tmux "$MDU_HAL_SESSION"'),
            stop_mdu.index('stop_tmux "$MDU_ELINK_SESSION"'),
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

    def test_runner_input_gate_tolerates_interleaved_aimrt_log_fragments(self):
        helper_path = FOXGLOVE_DIR / "helpers/hope-lifecycle"
        interleaved = """[a3_backend] topic readiness after startup gate:
  waist        ready=yes\x1b[0;32m[AimRT initialization log]
 samples=3
  leg          ready=yes samples=4
  arm          ready=yes samples=4
  neck         ready=yes samples=4
  pelvis_imu   ready=yes samples=4
  torso_imu    ready=yes samples=3
[a3_pingpong] serve025 bridge ready; startup sent no gripper command
"""
        with tempfile.TemporaryDirectory() as directory:
            runner_log = Path(directory) / "runner.log"
            runner_log.write_text(interleaved)
            command = 'source "$1"; runner_inputs_ready "$2"'
            complete = subprocess.run(
                ["bash", "-c", command, "bash", str(helper_path), str(runner_log)],
                check=False,
            )
            self.assertEqual(complete.returncode, 0)

            runner_log.write_text(interleaved.replace("torso_imu", "missing_imu"))
            incomplete = subprocess.run(
                ["bash", "-c", command, "bash", str(helper_path), str(runner_log)],
                check=False,
            )
            self.assertNotEqual(incomplete.returncode, 0)

    def test_runner_input_gate_handles_split_name_and_rejects_missing_or_no(self):
        helper_path = FOXGLOVE_DIR / "helpers/hope-lifecycle"
        injected = "\x1b[0;32m[2026-09-27][Info] AimRT initialization\nreport\n\x1b[0m\n"
        normal = "".join(f"  {name} ready=yes samples=6\n" for name in
                         ("waist", "leg", "arm", "neck", "pelvis_imu", "torso_imu"))
        bridge = "serve025 bridge ready; startup sent no gripper command\n"
        split = normal.replace("pelvis_imu ready=yes", "pelvis_imu" + injected + " ready=yes")
        cases = [(split + bridge, 0),
                 (split.replace("pelvis_imu", "missing_imu") + bridge, 1),
                 (split.replace(" ready=yes", " ready=no", 1) + bridge, 1),
                 (split.replace("pelvis_imu" + injected + " ready=yes", "pelvis_imu" + injected + " ready=no") + bridge, 1),
                 (split, 1)]
        with tempfile.TemporaryDirectory() as directory:
            runner_log = Path(directory) / "runner.log"
            for content, expected in cases:
                runner_log.write_text(content)
                result = subprocess.run(["bash", "-c", 'source "$1"; runner_inputs_ready "$2"',
                                         "bash", str(helper_path), str(runner_log)], check=False)
                self.assertEqual(result.returncode, expected)

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

    def test_lifecycle_uses_t3_elink_gripper_profile(self):
        helper = (FOXGLOVE_DIR / "helpers/hope-lifecycle").read_text()
        for field in ("MDU_SUPPORTED_ROBOT_MODEL=A3_T3D0", "hal_elink_a3_cfg.yaml",
                      "hal_ethercat_a3_t3d0_cfg.yaml", "HOPE_GRIPPER_CONFIG_T3_V2",
                      "owner=hal_elink_t3d0", "render_t3_elink_config",
                      "ELINK_HAND_ROS2_SUBSCRIBER_NOT_REGISTERED"):
            self.assertIn(field, helper)

    def test_lifecycle_base_start_is_process_based_not_a_data_gate(self):
        helper = (FOXGLOVE_DIR / "helpers/hope-lifecycle").read_text()
        start_base = helper.split("start_base() {", 1)[1].split("\n}\n", 1)[0]
        self.assertNotIn("ros2 topic type", start_base)
        self.assertNotIn("ros2 topic info", start_base)
        self.assertNotIn("ros2 service type", start_base)
        self.assertIn("hope-base-pose-transport-relay", start_base)
        self.assertIn(
            "never gates lifecycle completion or the Foxglove Ready button",
            start_base,
        )

    def test_optitrack_is_not_a_lifecycle_preflight_gate(self):
        helper = (FOXGLOVE_DIR / "helpers/hope-lifecycle").read_text()
        laptop_preflight = helper.split("preflight_laptop() {", 1)[1].split(
            "\n}\n", 1
        )[0]
        self.assertIn("LAPTOP_LOG_COLLECTOR_READY", laptop_preflight)
        self.assertIn("RSYNC_MISSING", laptop_preflight)
        self.assertNotIn("LAPTOP_NATNET_NODE_MISSING", laptop_preflight)
        self.assertNotIn("LAPTOP_CALIBRATION_SERVICE_MISSING", laptop_preflight)
        self.assertNotIn("UNMANAGED_BRIDGE_PRESENT", laptop_preflight)
        self.assertIn('[[ "$MOTIVE_IP" == NONE ]]', helper)
        self.assertIn("MOTIVE_IP_REQUIRED", helper)
        preflight = helper.split("preflight_hdu() {", 1)[1].split("\n}\n", 1)[0]
        self.assertIn(
            'static_peer_list_contains "$static_peers" "$LAPTOP_WIFI_IP"',
            preflight,
        )
        self.assertIn(
            'static_peer_list_contains "$static_peers" "$MDU_INTERNAL_IP"',
            preflight,
        )
        self.assertIn("HDU_DDS_LAPTOP_PEER_MISSING", preflight)
        self.assertIn("HDU_DDS_MDU_PEER_MISSING", preflight)
        runbook = (
            REPO_DIR / "docs/operations/foxglove_first_hardware_test.md"
        ).read_text()

    def test_runner_transport_is_late_started_bidirectional_and_loop_free(self):
        relay = (FOXGLOVE_DIR / "a3/hope_runner_transport_relay.py").read_text()
        observer = (FOXGLOVE_DIR / "a3/hope_observer.py").read_text()
        proxy = (FOXGLOVE_DIR / "a3/hope_command_proxy.py").read_text()
        self.assertIn('REMOTE_STATE_TOPIC = "/hope/runner/state_flat"', relay)
        self.assertIn('LOCAL_STATE_TOPIC = "/hope/runner/state_hdu_flat"', relay)
        self.assertIn(
            'LOCAL_CONTROL_TOPIC = "/hope/runner/control_request_hdu_flat"',
            relay,
        )
        self.assertIn(
            'REMOTE_CONTROL_TOPIC = "/hope/runner/control_request_flat"',
            relay,
        )
        self.assertIn("RUNNER TRANSPORT HEALTHY", relay)
        self.assertIn("local_state_subscribers >= 2", relay)
        self.assertIn("local_control_publishers >= 1", relay)
        self.assertIn("remote_control_subscribers >= 1", relay)
        self.assertIn('"/hope/runner/state_hdu_flat"', observer)
        self.assertIn('"ball_marker_rate_hz", 30.0', observer)
        self.assertIn("_ball_marker_period_ns", observer)
        observer_service = (
            FOXGLOVE_DIR / "a3/hope-observer.service"
        ).read_text()
        self.assertIn("-p ball_marker_rate_hz:=30.0", observer_service)
        self.assertIn('"/hope/runner/state_hdu_flat"', proxy)
        self.assertIn('"/hope/runner/control_request_hdu_flat"', proxy)
        self.assertNotIn('"/hope/runner/control_request_flat"', proxy)
        fake = (
            FOXGLOVE_DIR / "tests/runner_transport_fake_endpoint.py"
        ).read_text()
        self.assertIn("/hope/diagnostic/", fake)
        self.assertIn("FAKE RUNNER ACK", fake)
        self.assertNotIn("/hope/runner/control_request_flat", fake)
        self.assertNotIn("subprocess", fake)


if __name__ == "__main__":
    unittest.main()
