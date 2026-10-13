"""Recipe wiring regressions, runnable without the simulator."""

import ast
import importlib.util
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / "source/whole_body_tracking/whole_body_tracking"
spec = importlib.util.spec_from_file_location(
    "command_overrides", PACKAGE / "utils/task_command_overrides.py"
)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def _config_fields(filename, classname):
    tree = ast.parse((PACKAGE / "tasks/tracking/mdp" / filename).read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == classname)
    return SimpleNamespace(**{
        n.target.id: None for n in cls.body
        if isinstance(n, ast.AnnAssign) and isinstance(n.target, ast.Name)
    })


def _env():
    return SimpleNamespace(
        commands=SimpleNamespace(
            motion=_config_fields("commands.py", "MotionCommandCfg"),
            racket_target=_config_fields("hope_commands.py", "RacketTargetCommandCfg"),
        ),
        actions=SimpleNamespace(joint_pos=_config_fields(
            "hope_actions.py", "V11SafeClampedJointPositionActionCfg"
        )),
    )


def test_every_published_command_key_reaches_a_real_config_field():
    task = yaml.safe_load((ROOT / "cfg/task/HOPEPingPong.yaml").read_text())
    env = _env()
    applied = []
    module.apply_command_overrides(env, task, applied)
    assert len(applied) == sum(len(task[b]) for b in ("motion", "racket", "action"))
    assert env.commands.motion.hold_steps_range == tuple(task["motion"]["hold_steps_range"])
    assert env.actions.joint_pos.action_contract == task["action"]["contract"]
    assert env.commands.racket_target.racket_pos_range_per_clip[0][1] == tuple(
        task["racket"]["pos_range_per_clip"]["forehand"]["y"]
    )


def test_cli_override_changes_effective_timing_and_rejects_typos():
    env = _env()
    module.apply_command_overrides(env, {"motion": {"hold_steps_range": [2, 4]}}, [])
    assert env.commands.motion.hold_steps_range == (2, 4)
    with pytest.raises(ValueError, match="hold_step_range"):
        module.apply_command_overrides(env, {"motion": {"hold_step_range": [2, 4]}}, [])


def test_clip_mapping_uses_explicit_fh_bh_xyz_order():
    bh = {"z": [5, 6], "y": [3, 4], "x": [1, 2]}
    fh = {"z": [15, 16], "y": [13, 14], "x": [11, 12]}
    assert module._clip_ranges({"backhand": bh, "forehand": fh}) == (
        ((11., 12.), (13., 14.), (15., 16.)), ((1., 2.), (3., 4.), (5., 6.)))
    with pytest.raises(ValueError):
        module._clip_ranges({"forehand": fh})


def test_resume_rejects_legacy_policy_and_old_recipe_but_accepts_new_checkpoint():
    tree = ast.parse((ROOT / "scripts/train.py").read_text())
    fn = next(n for n in tree.body if isinstance(n, ast.FunctionDef)
              and n.name == "_validate_training_checkpoint")
    namespace = {}
    exec(compile(ast.Module(body=[fn], type_ignores=[]), "train.py", "exec"), namespace)
    validate = namespace[fn.name]
    policy = SimpleNamespace(exploration_contract="bounded")
    with pytest.raises(ValueError, match="exploration"):
        validate({}, policy, "repaired")
    state = {"policy_exploration_contract": "bounded"}
    with pytest.raises(ValueError, match="recipe"):
        validate({"hope_exact_resume_state": state}, policy, "repaired")
    state["training_recipe_revision"] = "repaired"
    validate({"hope_exact_resume_state": state}, policy, "repaired")
    with pytest.raises(ValueError, match="reward revision"):
        validate({"hope_exact_resume_state": state}, policy, "repaired", "waist_hip_tracking_v1")
    state["reward_revision"] = "waist_hip_tracking_v1"
    validate({"hope_exact_resume_state": state}, policy, "repaired", "waist_hip_tracking_v1")
