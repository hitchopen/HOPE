"""Host-only structural checks for strict rally-conductor thresholds."""

import ast
from pathlib import Path


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "pp_rally_conductor.py"


def test_xlock_keeps_physical_threshold_and_only_adds_fixed_arithmetic_epsilon():
    source = SCRIPT.read_text()
    tree = ast.parse(source)
    constants = {
        target.id: node.value.value
        for node in tree.body
        if isinstance(node, ast.Assign)
        and len(node.targets) == 1
        and isinstance((target := node.targets[0]), ast.Name)
        and isinstance(node.value, ast.Constant)
    }
    assert constants["XLOCK_COMPARE_EPS_M"] == 1.0e-6
    assert 'os.environ.get("PP_XLOCK_THRESH", "0.05")' in source
    assert "PP_XLOCK_EPS" not in source
    helpers = {
        node.name: node for node in tree.body if isinstance(node, ast.FunctionDef)
    }
    helper = helpers["xlock_within_threshold"]
    tolerance_adds = [
        node
        for node in ast.walk(helper)
        if isinstance(node, ast.BinOp)
        and isinstance(node.op, ast.Add)
        and isinstance(node.left, ast.Name)
        and node.left.id == "XLOCK_THRESH"
        and isinstance(node.right, ast.Name)
        and node.right.id == "XLOCK_COMPARE_EPS_M"
    ]
    assert len(tolerance_adds) == 1
    helper_calls = [
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Name)
        and node.func.id == "xlock_within_threshold"
    ]
    assert len(helper_calls) == 4


def test_direct_pelvis_height_is_a_fail_fast_fall_signal():
    source = SCRIPT.read_text()
    tree = ast.parse(source)
    constants = {
        target.id: node.value.value
        for node in tree.body
        if isinstance(node, ast.Assign)
        and len(node.targets) == 1
        and isinstance((target := node.targets[0]), ast.Name)
        and isinstance(node.value, ast.Constant)
    }
    assert constants["MIN_UPRIGHT_PELVIS_Z_M"] == 0.80
    assert "PHYSICAL FALL: pelvis z=" in source
    assert '"physical_falls": int(physical_fall_detected)' in source
    assert '"runner_fall_guards": int(runner_fall_guard_tripped)' in source


def test_runner_command_safety_latch_is_a_fail_fast_stability_failure():
    source = SCRIPT.read_text()
    assert "SAFETY_LATCH_RE = re.compile(" in source
    assert 'if ev["command_safety_faults"]:' in source
    assert 'and not command_safety_faults' in source
    assert '"command_safety_fault_count": len(command_safety_faults)' in source


def test_conductor_tracks_the_cpp_planner_process_only():
    source = SCRIPT.read_text()
    assert '["pgrep", "-f", "hope_planner_cpp_node"]' in source
    assert '["pgrep", "-f", "hope_planner_node"]' not in source


def test_gate3_has_one_fail_closed_certification_verdict():
    source = SCRIPT.read_text()
    assert 'GATE3_VERDICT != "certification"' in source
    assert '"gate_name": "Gate3"' in source
    assert '"selected_gate_verdict": "certification"' in source
    assert "cherry_pick" not in source
