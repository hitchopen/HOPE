"""Guard the deployed source/launch boundary, including ROS overlay precedence."""
import importlib.util
import os
from pathlib import Path
import subprocess

import pytest
import yaml

SCRIPTS = Path(__file__).resolve().parents[1]
ROOT = SCRIPTS.parents[2]
SPEC = importlib.util.spec_from_file_location("ball_physics", SCRIPTS / "pp_ball_physics.py")
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)
CONFIG = Path(os.environ.get("HOPE_BALL_CONFIG", str(ROOT / "hope_ws/src/hope_planner_cpp/config/model21800_hardware.yaml")))


def test_deployed_coefficients_and_effective_future_contact():
    values = MODULE.resolve([CONFIG])["parameters"]
    assert values["drag_k"] == pytest.approx(0.12861635416914574)
    assert values["restitution_v"] == pytest.approx(0.9313775047019153)
    assert values["restitution_h"] == pytest.approx(0.6125555614493483)
    assert values["post_net_future_bounce_tangential_gain"] == pytest.approx(0.38744443855065164)
    assert values["post_net_future_bounce_restitution_v"] == pytest.approx(0.9313775047019153)
    raw = yaml.safe_load(CONFIG.read_text())["hope_planner"]["ros__parameters"]
    assert raw["table_tangential_gain"] == pytest.approx(0.38744443855065164)
    assert raw["magnus_k"] == pytest.approx(0.00379260504480258)


def test_profile_overlay_and_shell_environment_agree(tmp_path):
    overlay = tmp_path / "profile.yaml"
    overlay.write_text("hope_planner:\n  ros__parameters:\n    drag_k: 0.2\n")
    assert MODULE.resolve([CONFIG, overlay])["parameters"]["drag_k"] == 0.2
    command = 'source "$1"; pp_load_ball_physics "$2" "$3" >/dev/null; printf "%s %s" "$PP_BALL_DRAG_K" "$PP_BALL_RESTITUTION_V"'
    result = subprocess.run(["bash", "-euc", command, "test", str(SCRIPTS / "pp_ball_physics.sh"), str(CONFIG), str(overlay)], capture_output=True, text=True, check=True)
    assert result.stdout == "0.2 0.9313775047019153"


def test_observed_bounce_overlay_is_shared_with_packetizer(tmp_path):
    overlay = tmp_path / "observed.yaml"
    overlay.write_text("hope_planner:\n  ros__parameters:\n    table_tangential_gain: 0.42\n")
    values = MODULE.resolve([CONFIG, overlay])["parameters"]
    assert values["table_tangential_gain"] == 0.42
    assert values["post_net_future_bounce_tangential_gain"] == 0.38744443855065164
    result = subprocess.run(["python3", str(SCRIPTS / "pp_ball_physics.py"),
                             "--params-file", str(CONFIG), "--params-file", str(overlay),
                             "--shell"], capture_output=True, text=True, check=True)
    assert "export PP_BALL_OBSERVED_BOUNCE_GAIN=0.42" in result.stdout


def test_model14600_packetizer_preview_matches_hardware_physics():
    path = CONFIG.with_name("model14600_adaptive_flight_packetizer.yaml")
    preview = yaml.safe_load(path.read_text())["hope_ball_flight_packetizer"]["ros__parameters"]
    for key, value in MODULE.resolve([CONFIG])["parameters"].items():
        assert preview[key] == pytest.approx(value), key
    assert preview["adaptive_policy_lead_s"] == pytest.approx(0.35)


@pytest.mark.parametrize("bad", [".nan", "-.1", "12", "true"])
def test_invalid_coefficient_fails_before_launch(tmp_path, bad):
    overlay = tmp_path / "invalid.yaml"
    overlay.write_text(f"hope_planner:\n  ros__parameters:\n    drag_k: {bad}\n")
    with pytest.raises(ValueError):
        MODULE.resolve([CONFIG, overlay])


def test_simulator_fallbacks_match_adopted_config():
    header = (ROOT / "a3_deploy/A3_MuJoCo_Sim/aimrt_mujoco_sim/src/module/mujoco_sim_module/mujoco_sim_module.h").read_text()
    for key, variable in [("drag_k", "DRAG_K"), ("restitution_h", "RESTITUTION_H"), ("restitution_v", "RESTITUTION_V")]:
        value = MODULE.resolve([CONFIG])["parameters"][key]
        import re
        assert float(re.search(rf"gate3_ball_{key}_ = ([0-9.]+);", header).group(1)) == value


def test_future_contact_overlay_does_not_change_simulated_material(tmp_path):
    overlay = tmp_path / "future.yaml"
    overlay.write_text("hope_planner:\n  ros__parameters:\n    post_net_future_bounce_restitution_v: 0.98\n")
    values = MODULE.resolve([CONFIG, overlay])["parameters"]
    assert values["post_net_future_bounce_restitution_v"] == 0.98
    assert values["restitution_v"] == pytest.approx(0.9313775047019153)


def test_legacy_profile_inherits_vertical_coefficient(tmp_path):
    data = yaml.safe_load(CONFIG.read_text())
    del data["hope_planner"]["ros__parameters"]["post_net_future_bounce_restitution_v"]
    legacy = tmp_path / "legacy.yaml"
    legacy.write_text(yaml.safe_dump(data))
    values = MODULE.resolve([legacy])["parameters"]
    assert values["post_net_future_bounce_restitution_v"] == values["restitution_v"]


def test_shared_source_does_not_hide_prediction_material_mismatch(tmp_path):
    receipt = MODULE.resolve([CONFIG])
    comparison = receipt["prediction_vs_simulated_contact"]
    assert comparison["matched"] is True
    assert comparison["planner_horizontal_retention"] == pytest.approx(0.6125555614493483)
    assert comparison["simulator_horizontal_retention"] == pytest.approx(0.6125555614493483)
    overlay = tmp_path / "mismatch.yaml"
    overlay.write_text("hope_planner:\n  ros__parameters:\n"
                       "    post_net_future_bounce_tangential_gain: 0.2\n")
    mismatched = MODULE.resolve([CONFIG, overlay])
    assert mismatched["prediction_vs_simulated_contact"]["matched"] is False
    assert mismatched["parameters"]["table_tangential_gain"] == receipt["parameters"]["table_tangential_gain"]


def test_experimental_profiles_are_consistent_without_changing_hardware_defaults():
    overlay = yaml.safe_load(CONFIG.with_name(
        "model14600_contact_observation_candidate.yaml").read_text())
    sender = overlay["hope_ball_flight_packetizer"]["ros__parameters"]
    receiver = overlay["hope_planner"]["ros__parameters"]
    relay = overlay["optitrack_mct_relay"]["ros__parameters"]
    assert sender["input_pose_topic"] == relay["planner_poses_topic"]
    assert sender["adaptive_post_bounce_span_s"] == receiver["batch_post_bounce_min_span_s"]
    for key in ("batch_post_bounce_min_span_s", "batch_post_bounce_exclusion_s"):
        assert sender[key] == receiver[key]
    active = yaml.safe_load(CONFIG.with_name(
        "model21800_flight_packetizer.yaml").read_text())["hope_ball_flight_packetizer"]["ros__parameters"]
    assert active["input_pose_topic"] == "/poses"
    assert active.get("batch_post_bounce_min_span_s", 0.0) == 0.0
    diagnostic = CONFIG.with_name("model14600_mujoco_material_match_diagnostic.yaml")
    assert not MODULE.resolve([CONFIG, diagnostic])["prediction_vs_simulated_contact"]["matched"]  # historical diagonal overlay


def test_spin_contact_overlay_reaches_both_nodes_and_gate3_shell():
    path = CONFIG.with_name("model14600_spin_contact.yaml")
    overlay = yaml.safe_load(path.read_text())
    wanted = overlay["hope_planner"]["ros__parameters"]
    assert overlay["hope_ball_flight_packetizer"]["ros__parameters"] == wanted
    resolved = MODULE.resolve([CONFIG, path])
    assert resolved["contact_prediction"] == {k: wanted[k] for k in MODULE.CONTACT_DEFAULTS}
    result = subprocess.run(["python3", str(SCRIPTS / "pp_ball_physics.py"),
                             "--params-file", str(CONFIG), "--params-file", str(path),
                             "--shell"], capture_output=True, text=True, check=True)
    for key, value in wanted.items():
        env = MODULE.KEYS[key][0] if key in MODULE.KEYS else f"PP_{key.upper()}"
        assert f"export {env}={str(value).lower()}" in result.stdout
        launch = (SCRIPTS / "pp_gate3_rally.sh").read_text()
        assert launch.count(f"-p {key}:=${env}") == 2
