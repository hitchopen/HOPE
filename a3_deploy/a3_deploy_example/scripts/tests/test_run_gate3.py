"""Regression tests for model-independent Gate3 composition."""
import json
import os
from pathlib import Path
import shlex
import shutil
import subprocess
import sys

import pytest
import yaml

SCRIPTS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SCRIPTS))
import run_gate3 as gate3
from pp_policy_artifact import small_station_mode
from pp_rally_report import _discover_mode_from_runner


def scene():
    return yaml.safe_load((gate3.CONFIG / "scenarios/fixed-home.yaml").read_text())


@pytest.mark.parametrize("balls", [None, 26, 100])
def test_public_entry_preserves_qualification_and_explicit_scene(tmp_path, balls):
    scripts = tmp_path / "scripts"
    scripts.mkdir()
    for name in ("pp_gate3_hitter_pingpong.sh", "pp_gate3_physical_common.sh"):
        shutil.copy2(SCRIPTS / name, scripts / name)
    entry = scripts / "pp_gate3_hitter_pingpong.sh"
    entry.write_text(entry.read_text().replace("/tmp/pp_", str(tmp_path / "pp_")))
    # Capture the composed environment without starting ROS, Runner or MuJoCo.
    (scripts / "pp_gate3_rally.sh").write_text(
        "exec python3 -c 'import json, os; print(json.dumps(dict(os.environ)))'\n"
    )
    (scripts / "pp_planner_envelope_audit.py").write_text("pass\n")
    config = tmp_path / "runtime.yaml"
    config.write_text("{}\n")
    sim = tmp_path / "sim/bin"
    sim.mkdir(parents=True)
    (sim / "aimrt_main").write_text("#!/bin/sh\nexit 0\n")
    (sim / "aimrt_main").chmod(0o755)
    env = {k: v for k, v in os.environ.items() if not k.startswith(("PP_", "A3_"))}
    env.update(A3_PINGPONG_RUNTIME_CFG=str(config), PP_SIM_INSTALL=str(sim.parent),
               PP_DIST=str(tmp_path), PP_PHYSICAL_EVIDENCE_JSON=str(tmp_path / "physical.json"))
    if balls is not None:
        resolved = gate3.scene_environment(scene(), balls)
        exports = tmp_path / "scene.env.sh"
        exports.write_text("\n".join(f"export {k}={shlex.quote(v)}" for k, v in resolved.items()))
        env.update(PP_SCENE_EXPORTS=str(exports), PP_SERVES=str(balls))
    result = subprocess.run(["bash", str(scripts / "pp_gate3_hitter_pingpong.sh"),
                             "--preflight-only"], env=env, text=True, capture_output=True, check=True)
    actual = json.loads(result.stdout)
    assert actual["PP_GATE3_PREFLIGHT_ONLY"] == "1"
    assert actual["PP_GATE3_PHASE"] == ("qualification" if balls is None else "fixed_home")
    assert actual["PP_SERVES"] == str(balls or 12)
    assert len(json.loads(actual["PP_SERVES_LIST"])) == (balls or 12) * 6
    assert "--gate3-qdes-audit-only" in actual["PP_EXTRA_ARGS"]
    assert "PP_QUESTION_BANK_PATH" not in actual
    if balls is not None:
        assert actual["PP_FLIGHT_S_LIST"] == resolved["PP_FLIGHT_S_LIST"]
        assert actual["PP_MIN_GLOBAL_LANDINGS"] == actual["PP_MIN_GLOBAL_CONTACTS"]


def test_default_scene_preserves_original_physical_sequence():
    old = subprocess.check_output([
        "bash", "-c", 'source "$1"; gate3_apply_hitter_fixed_home_rapid_1150ms_v1_contract; '
        'printf "%s\\n%s\\n%s\\n" "$PP_SERVES_LIST" "$PP_FLIGHT_S_LIST" "$PP_PAUSE_S_LIST"',
        "bash", str(SCRIPTS / "pp_gate3_physical_common.sh")], text=True)
    resolved = gate3.scene_environment(scene())
    for key, line in zip(("PP_SERVES_LIST", "PP_FLIGHT_S_LIST", "PP_PAUSE_S_LIST"), old.splitlines(), strict=True):
        assert json.loads(resolved[key]) == json.loads(line)


def test_100_balls_one_continuous_session_and_only_one_final_tail():
    env = gate3.scene_environment(scene(), 100)
    flights = json.loads(env["PP_FLIGHT_S_LIST"])
    assert len(json.loads(env["PP_SERVES_LIST"])) == 600
    assert flights == [2.5] * 6 + [1.15] * 93 + [2.5]
    assert env["PP_MIN_RAPID_COMPLETED"] == "94"
    assert env["PP_MIN_COMPLETED_SERVES"] == "100"


@pytest.mark.parametrize("balls", [0, 3, 25, 27, -2])
def test_invalid_ledger_rejected(balls):
    with pytest.raises(ValueError):
        gate3.scene_environment(scene(), balls)


def test_scene_changes_timing_and_recovery_without_code():
    cfg = scene()
    cfg["timing"]["rapid_flight_s"] = 1.3
    cfg["metrics"]["PP_RECOVERY_RADIUS_M"] = .06
    env = gate3.scene_environment(cfg)
    assert json.loads(env["PP_FLIGHT_S_LIST"])[6] == 1.3
    assert env["PP_RECOVERY_RADIUS_M"] == "0.06"


def test_scene_cannot_select_model_or_shell_command():
    cfg = scene()
    cfg["metrics"]["PP_EXTRA_ARGS"] = "--policy-dir another-model"
    with pytest.raises(ValueError, match="invalid scene metric"):
        gate3.scene_environment(cfg)


def test_per_ball_timing_reaches_launcher_and_conductor_without_repeating():
    cfg = scene()
    flights = [round(2.0 + i * .03, 2) for i in range(200)]
    pauses = [.5 if i % 10 == 9 else 0 for i in range(200)]
    cfg["timing"].update(flight_s_list=flights, pause_s_list=pauses)
    env = gate3.scene_environment(cfg, 200)
    assert json.loads(env["PP_FLIGHT_S_LIST"]) == flights
    assert json.loads(env["PP_PAUSE_S_LIST"]) == pauses
    with pytest.raises(ValueError, match="flight_s_list"):
        gate3.scene_environment(cfg, 26)


@pytest.mark.parametrize("name,bad", [("flight_s_list", 0), ("flight_s_list", float("nan")),
                                    ("pause_s_list", -1), ("pause_s_list", True)])
def test_per_ball_timing_rejects_invalid_duration(name, bad):
    cfg = scene()
    cfg["timing"][name] = [2.0] * 26
    cfg["timing"][name][12] = bad
    with pytest.raises(ValueError, match=name):
        gate3.scene_environment(cfg)


@pytest.mark.parametrize("abi,mode", [("small_station_112_v1", "small_station112"), ("small_station_324_v1", "compact324"), ("ball_clock_110_v1", "rally_v14")])
def test_report_uses_native_abi_receipt_not_recipe(tmp_path, abi, mode):
    log = tmp_path / "runner.log"
    log.write_text(f"[pp] validated_policy_abi={abi}\n[pp] hitter_pure training_recipe=new_reward_2027\n")
    assert _discover_mode_from_runner(log, None) == mode


def test_conflicting_runner_abi_receipts_are_not_merged(tmp_path):
    log = tmp_path / "runner.log"
    log.write_text("[pp] validated_policy_abi=small_station_112_v1\n[pp] validated_policy_abi=small_station_324_v1\n")
    assert _discover_mode_from_runner(log, None) is None


def test_routing_does_not_need_training_provenance():
    metadata = {"hitter_pingpong_command_contract": "small_station_external_schedule_precommit_settle_v1",
                "hitter_pingpong_planner_commit_contract": "fresh_schema2_absolute_small_station_v1",
                "hitter_pure_runtime_contract": "rally_final_v2", "qdes_action_contract": "v12_affine_safe_slew_qdes_v1",
                "hitter_pingpong_optional_reach_training_enabled": "false",
                "actor_obs_contract": "hitter_pure_112_headslots_vxy_reach_v1"}
    assert small_station_mode(metadata, {"obs": [1,112]}, {"actions": [1,31]}) == "small_station112"
    metadata["hitter_pingpong_planner_commit_contract"] = "unknown_v2"
    assert small_station_mode(metadata, {"obs": [1,112]}, {"actions": [1,31]}) is None


@pytest.mark.parametrize("recipe,version", [("hitter_small_station_matched_plant_recovery_v4", "4"),
                                          ("hitter_small_station_hold_resume_recovery_v5", "5")])
def test_small_station_recipe_routes_by_unchanged_112_contract(tmp_path, recipe, version):
    metadata = {"hitter_pure_training_recipe": recipe,
                "hitter_pure_training_recipe_version": version,
                "a3_passive_dynamics_contract": "mjcf_absolute_friction_and_separate_viscous_isaac5_v1",
                "hitter_pingpong_command_contract": "small_station_external_schedule_precommit_settle_v1",
                "hitter_pingpong_planner_commit_contract": "fresh_schema2_absolute_small_station_v1",
                "hitter_pure_runtime_contract": "rally_final_v2", "qdes_action_contract": "v12_affine_safe_slew_qdes_v1",
                "hitter_pingpong_optional_reach_training_enabled": "false",
                "actor_obs_contract": "hitter_pure_112_headslots_vxy_reach_v1"}
    assert small_station_mode(metadata, {"obs": [1,112]}, {"actions": [1,31]}) == "small_station112"
    assert small_station_mode(metadata, {"obs": [1,112]}, {"actions": [1,30]}) is None
    log = tmp_path / "runner.log"
    log.write_text("[pp] validated_policy_abi=small_station_112_v1\n"
                   f"[pp] hitter_pure training_recipe={recipe} version={version}\n")
    assert _discover_mode_from_runner(log, None) == "small_station112"
