"""Exercise checkpoint persistence without Isaac, a GPU, or a cloud logger."""
import ast
import hashlib
import json
import os
import pathlib
import pickle
from types import SimpleNamespace

import pytest


RUNNER = pathlib.Path(__file__).resolve().parents[1] / (
    "source/whole_body_tracking/whole_body_tracking/utils/my_on_policy_runner.py"
)


def runner_methods():
    module = ast.parse(RUNNER.read_text())
    cls = next(n for n in module.body if isinstance(n, ast.ClassDef) and n.name == "MotionOnPolicyRunner")
    methods = [n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name in {"save", "_prepare_logging_writer"}]
    cls.body = methods
    namespace = dict(
        pathlib=pathlib, os=os, json=json, hashlib=hashlib,
        OnPolicyRunner=type("LocalWriterBase", (), {
            "_prepare_logging_writer": lambda self: setattr(self, "writer", "local")
        }),
        torch=SimpleNamespace(save=lambda value, path: pathlib.Path(path).write_bytes(pickle.dumps(value))),
    )
    exec(compile(ast.Module(body=[cls], type_ignores=[]), str(RUNNER), "exec"), namespace)
    return namespace["MotionOnPolicyRunner"]


@pytest.mark.parametrize("with_provenance", [False, True])
def test_atomic_local_checkpoint_retains_resume_state(tmp_path, with_provenance):
    runner = runner_methods()()
    runner.cfg = {"max_iterations": 100}
    runner.current_learning_iteration = 16
    runner.tot_timesteps = 1024
    runner.tot_time = 10.0
    runner.log_dir = str(tmp_path)
    runner.alg = SimpleNamespace(
        learning_rate=.001,
        policy=SimpleNamespace(state_dict=lambda: {"weight": [1, 2]}),
        optimizer=SimpleNamespace(state_dict=lambda: {"step": 16}),
    )
    runner._capture_rng_state = lambda: {"rng": "saved"}
    runner._capture_environment_resume_state = lambda: {"step": 32}
    if with_provenance:
        runner.checkpoint_provenance = {"source": "local"}
    checkpoint = tmp_path / "model_16.pt"
    runner.save(str(checkpoint), infos={"local": True})
    saved = pickle.loads(checkpoint.read_bytes())
    assert saved["model_state_dict"] == {"weight": [1, 2]}
    assert saved["optimizer_state_dict"] == {"step": 16}
    resume = saved["hope_exact_resume_state"]
    assert resume["next_learning_iteration"] == 17
    assert resume["environment_resume_state"] == {"step": 32}
    assert resume["rng"] == "saved"
    assert not any("wandb" in key for key in resume)
    assert not list(tmp_path.glob(".*.tmp-*"))
    if with_provenance:
        receipt = json.loads(pathlib.Path(str(checkpoint) + ".final_v3.json").read_text())
        assert receipt["checkpoint_sha256"] == hashlib.sha256(checkpoint.read_bytes()).hexdigest()


def test_writer_is_local_only():
    runner = runner_methods()()
    runner.cfg = {"logger": "tensorboard"}
    runner._prepare_logging_writer()
    assert runner.writer == "local"
    runner.cfg = {"logger": "remote_service"}
    with pytest.raises(ValueError, match="local tensorboard"):
        runner._prepare_logging_writer()
