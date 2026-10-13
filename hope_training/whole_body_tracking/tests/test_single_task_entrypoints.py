"""Keep train, play and the direct Gym runner on the single retained recipe."""

import ast
import os
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / "source/whole_body_tracking/whole_body_tracking"


def test_train_and_play_select_the_only_task_and_algorithm():
    hydra = pytest.importorskip("hydra")
    from omegaconf import OmegaConf

    assert {p.stem for p in (ROOT / "cfg/task").glob("*.yaml")} == {"HOPEPingPong"}
    assert {p.stem for p in (ROOT / "cfg/algo").glob("*.yaml")} == {"ppo"}
    with hydra.initialize_config_dir(config_dir=str(ROOT / "cfg"), version_base=None):
        train = hydra.compose(config_name="train")
        play = hydra.compose(config_name="play")
    assert OmegaConf.to_container(train.task) == OmegaConf.to_container(play.task)
    assert OmegaConf.to_container(train.algo) == OmegaConf.to_container(play.algo)
    assert train.task.gym_task == "HOPE-HitterPingPong-AgibotA3-v0"


def test_direct_gym_runner_loads_the_same_standalone_algorithm(monkeypatch):
    # The YAML lookup is pure; load its functions without importing Isaac Sim.
    path = PACKAGE / "utils/ppo_cfg.py"
    tree = ast.parse(path.read_text())
    functions = [node for node in tree.body if isinstance(node, ast.FunctionDef)
                 and node.name in {"find_ppo_yaml", "load_ppo_params"}]
    namespace = {"os": os, "yaml": yaml, "__file__": str(path)}
    exec(compile(ast.Module(body=functions, type_ignores=[]), str(path), "exec"), namespace)
    monkeypatch.delenv("WBT_AGIBOT_A3_PPO_CFG", raising=False)
    selected = ROOT / "cfg/algo/ppo.yaml"
    assert Path(namespace["find_ppo_yaml"]()) == selected
    assert namespace["load_ppo_params"]() == yaml.safe_load(selected.read_text())


def test_only_selected_gym_task_is_registered():
    task_ids = []
    for path in (PACKAGE / "tasks").rglob("*.py"):
        for node in ast.walk(ast.parse(path.read_text())):
            if (isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
                    and isinstance(node.func.value, ast.Name)
                    and node.func.value.id == "gym" and node.func.attr == "register"):
                task_ids.extend(ast.literal_eval(kw.value) for kw in node.keywords if kw.arg == "id")
    assert task_ids == ["HOPE-HitterPingPong-AgibotA3-v0"]
