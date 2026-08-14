"""Static integrity checks for the public model_21800 Gate3 bundle."""

from __future__ import annotations

import hashlib
import os
from pathlib import Path
import re

import yaml


REPO = Path(__file__).resolve().parents[3]
EXAMPLE = REPO / "a3_deploy/a3_deploy_example"
SCRIPTS = EXAMPLE / "scripts"
CHECKPOINT = REPO / "hope_training/whole_body_tracking/checkpoints/model_21800.pt"
POLICY = EXAMPLE / "models/model_21800/policy/exported/policy.onnx"
DEPLOY = EXAMPLE / "models/model_21800/policy/params/deploy.yaml"
RUNNER = EXAMPLE / "src/a3/a3_deploy_onnx_ref"
SIM = REPO / "a3_deploy/A3_MuJoCo_Sim/aimrt_mujoco_sim"
PACKAGER = REPO / "agibot/code_deployment/a3_deploy_example/scripts/build_a3_deploy_pkg.sh"
FIND_ONNXRUNTIME = EXAMPLE / "cmake/Findonnxruntime.cmake"

FORMAL_SCRIPTS = (
    "pp_gate3_hitter_pingpong.sh",
    "pp_gate3_rally.sh",
    "pp_gate3_physical_common.sh",
    "pp_gate3_runner_common.sh",
    "pp_gate3_ball_launcher.py",
    "pp_gate3_ball_evidence.py",
    "pp_gate3_sim_mocap.py",
    "pp_gate3_core.py",
    "pp_gate3_contact_test.sh",
    "pp_rally_conductor.py",
    "pp_rally_report.py",
    "pp_mujoco_plant_report.py",
    "pp_planner_envelope_audit.py",
    "run_sim.sh",
    "reset_sim.sh",
)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def test_model21800_deploy_actor_matches_exported_provenance():
    metadata = yaml.safe_load(DEPLOY.read_text())
    assert metadata["task_plugin"] == "a3_pingpong_hitter_pure"
    assert metadata["contracts"]["actor_observation"]["total_dim"] == 110
    assert len(metadata["joint_sdk_names"]) == 31
    assert metadata["contracts"]["runtime"] == "rally_final_v2"
    assert metadata["contracts"]["training_recipe"] == {
        "name": "rally_v14",
        "version": "1",
    }
    assert _sha256(POLICY) == metadata["provenance"]["policy_sha256"]
    assert re.fullmatch(
        r"[0-9a-f]{64}", metadata["provenance"]["checkpoint_sha256"]
    )


def test_checkpoint_is_present_as_materialized_payload_or_valid_lfs_pointer():
    payload = CHECKPOINT.read_bytes()
    if payload.startswith(b"version https://git-lfs.github.com/spec/v1\n"):
        text = payload.decode("ascii")
        assert re.search(r"^oid sha256:[0-9a-f]{64}$", text, re.MULTILINE)
        size = re.search(r"^size ([0-9]+)$", text, re.MULTILINE)
        assert size is not None and int(size.group(1)) > 1_000_000
    else:
        assert len(payload) > 1_000_000


def test_formal_gate3_script_dependency_closure_is_complete_and_executable():
    for name in FORMAL_SCRIPTS:
        path = SCRIPTS / name
        assert path.is_file(), name
        if path.suffix in {".sh", ".py"} and name != "pp_gate3_core.py":
            assert os.access(path, os.X_OK), name

    entry = (SCRIPTS / "pp_gate3_hitter_pingpong.sh").read_text()
    engine = (SCRIPTS / "pp_gate3_rally.sh").read_text()
    assert "PP_GATE3_PROFILE=rally_v14" in entry
    assert "PP_GATE3_PHASE=qualification" in entry
    assert "gate3_apply_physical_arena_contract" in entry
    assert "PP_SERVES=12" in entry
    assert "--gate3-qdes-audit-only" in entry
    assert "--preflight-only" in entry
    assert "physical MuJoCo" in engine or "MuJoCo ball/table/net/racket" in engine
    assert "/agi/A3_MuJoCo_Sim" not in engine
    assert "p1_marker_cad_registration_20260805_redefined_p1_strict.json" in engine
    assert "p1_calibration_file:=" in engine
    assert "hope_planner_cpp_node" in engine
    assert "hope_ball_flight_packetizer" in engine
    assert "model21800_hardware.yaml" in engine
    assert "model21800_flight_packetizer.yaml" in engine
    assert "flight_packet_input_enabled:=true" in engine
    assert '"$PLANNER_DEBUG_CSV.flight_packets.csv"' in engine
    assert "ros2 run hope_planner hope_planner_node" not in engine
    assert "PYTHONPATH='$WS/src/hope_planner'" not in engine
    assert 'pgrep", "-f", "hope_planner_cpp_node"' in (
        SCRIPTS / "pp_rally_conductor.py"
    ).read_text()
    conductor = (SCRIPTS / "pp_rally_conductor.py").read_text()
    assert '"gate_name": "Gate3"' in conductor
    assert "Gate3CherryPick" not in conductor

    sim_mocap = (SCRIPTS / "pp_gate3_sim_mocap.py").read_text()
    assert "import NamedPose, NamedPoseArrayV2" not in sim_mocap
    assert "NamedPoseArray" in sim_mocap

    ball_launcher = (SCRIPTS / "pp_gate3_ball_launcher.py").read_text()
    assert "action=ENTER_MOTION result=APPLIED" in ball_launcher
    assert "MOTION (PUBLISHING)" in ball_launcher

    contact_test = (SCRIPTS / "pp_gate3_contact_test.sh").read_text()
    assert "pp_scripted_racket_contact_ab.cc" in contact_test
    assert "explicit_contact_energy_pass" in contact_test
    assert "cmake-build-model21800-gate3/install" in contact_test


def test_gate3_runner_and_mujoco_core_are_available():
    policy = (RUNNER / "include/a3_pingpong/pp_policy.hpp").read_text()
    main = (RUNNER / "src/a3_deploy/a3_pingpong_main.cpp").read_text()
    assert "Gate3 continues with the unmodified command" in policy
    assert "actual_q_hard_limit_audit_only(" in policy
    assert "kGate3QdesAuditOnlySupported" in main
    assert 'Has(argc, argv, "--gate3-qdes-audit-only")' in main

    messages = SIM / "src/protocols/mujoco_sim_msgs/msg"
    assert (messages / "Gate3BallCommand.msg").is_file()
    assert (messages / "Gate3BallState.msg").is_file()
    cfg = (SIM / "src/models/bin/cfg/a3_pingpong_iceoryx_cfg.yaml").read_text()
    assert "/sim/gate3/ball_command" in cfg
    assert "/sim/gate3/ball_state" in cfg
    assert (SIM / "src/module/mujoco_sim_module/common/gate3_ball_contact_model.h").is_file()
    cmake = (SIM / "CMakeLists.txt").read_text()
    assert "AIMRT_MUJOCO_SIM_ENABLE_COVERAGE" in cmake
    assert "AIMRT_MUJOCO_SIM_ENABLE_COVERAGE AND" in cmake


def test_model21800_only_packager_checks_runtime_dependency_closure():
    packager = PACKAGER.read_text()
    for marker in (
        "--pingpong-only",
        "verify_pingpong_package",
        "aimrt_plugins_iceoryx_plugin",
        "aimrt_plugins_ros2_plugin",
        "libaimrt_iceoryx_plugin.so",
        "libaimrt_ros2_plugin.so",
        "libonnxruntime.so.1",
        "libjoint_msgs__rosidl_typesupport_cpp.so",
        "provenance.policy_sha256",
    ):
        assert marker in packager

    finder = FIND_ONNXRUNTIME.read_text()
    assert "thirdparty/onnxruntime/onnxruntime-linux-*" in finder
    assert "onnxruntime_ROOT" in finder
