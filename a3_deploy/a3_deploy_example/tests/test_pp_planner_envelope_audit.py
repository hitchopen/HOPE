"""Host-only checks for the formal model_21800 planner-envelope preflight."""

import importlib.util
import re
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
SCRIPTS = ROOT / "a3_deploy/a3_deploy_example/scripts"
SCRIPT = SCRIPTS / "pp_planner_envelope_audit.py"
FORMAL_GATE3 = SCRIPTS / "pp_gate3_hitter_pingpong.sh"
PHYSICAL_COMMON = SCRIPTS / "pp_gate3_physical_common.sh"


def _module():
    spec = importlib.util.spec_from_file_location("pp_planner_envelope_audit", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _physical_scenario() -> str:
    match = re.search(
        r"^GATE3_PHYSICAL_SERVES_V1='([^']+)'$",
        PHYSICAL_COMMON.read_text(),
        flags=re.MULTILINE,
    )
    assert match is not None
    return match.group(1)


def test_model21800_physical_scenario_covers_the_rally_v14_envelope():
    audit = _module()
    serves, aims, geometry = audit.load_harness(
        FORMAL_GATE3, _physical_scenario()
    )
    assert len(serves) == 12
    assert aims == {
        "forehand": (-0.7625, 0.50),
        "backhand": (-0.7625, 0.50),
    }
    assert geometry == {
        "x_hit": 0.08,
        "x_hit_bh_delta": 0.0,
        "target_land_x": 2.055,
        "split_y": -0.25,
        "split_hyst": 0.04,
        "demo": False,
        "policy_z_offset": 0.760,
        "station_y_anchor": -0.7625,
    }

    report = audit.audit(serves, audit.RALLY_V10, aims, geometry)
    assert report["runner_harness_pass"] is True
    assert report["planner_contract_pass"] is True
    assert report["demo_substitution_active"] is False
    assert report["station_strict_coverage"] == 12
    assert report["initial_state_arena_pass"] is True
    assert report["visible_lead_time_pass"] is True
    assert {row["side"] for row in report["rows"]} == {
        "forehand",
        "backhand",
    }
    for side in ("forehand", "backhand"):
        assert report["by_side"][side]["raw_velocity_coverage"] == 6
        assert report["by_side"][side]["command_velocity_coverage"] == 6
        assert report["by_side"][side]["raw_z_coverage"] == 6
        assert report["by_side"][side]["planner_ballistic_landing_coverage"] == 6


def test_audit_defaults_to_the_only_public_formal_gate3_entry():
    audit = _module()
    assert audit.DEFAULT_GATE3 == FORMAL_GATE3
    text = FORMAL_GATE3.read_text()
    uncommented = re.sub(r"^#.*$", "", text, flags=re.MULTILINE)
    for marker in (
        "PP_GATE3_PROFILE=rally_v14",
        "PP_GATE3_PHASE=qualification",
        "PP_SERVES=\"${PP_SERVES:-12}\"",
        "PP_MIN_GLOBAL_CONTACTS=11",
        "PP_MIN_GLOBAL_LANDINGS=10",
        "PP_MIN_PROXY_RATE=1.0",
        "--gate3-qdes-audit-only",
        "pp_gate3_rally.sh",
    ):
        assert marker in text
    assert "--demo" not in uncommented
    assert "--side" not in uncommented
