"""Opt-in integration against real exports and the installed native Runner.

Set GATE3_TEST_DIST and GATE3_TEST_POLICY110/112/324. Fixtures
copy only deploy.yaml/ONNX; no training environment, recipe code or checkpoint.
"""
import hashlib
import os
from pathlib import Path
import shutil
import sys

import pytest
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from run_gate3 import native_inspect


@pytest.fixture(params=[110, 112, 324])
def pair(request, tmp_path):
    source = os.environ.get(f"GATE3_TEST_POLICY{request.param}")
    dist = os.environ.get("GATE3_TEST_DIST")
    if not source or not dist:
        pytest.skip("real policy fixtures and native Runner not configured")
    for rel in ("params/deploy.yaml", "exported/policy.onnx"):
        target = tmp_path / rel
        target.parent.mkdir(exist_ok=True)
        shutil.copy2(Path(source) / rel, target)
    return dist, tmp_path, request.param


def change_metadata(path, changes, *, bind_hash=True, recipe=None):
    import onnx
    model_path = path / "exported/policy.onnx"
    model = onnx.load(model_path)
    metadata = {p.key:p.value for p in model.metadata_props}
    metadata.update(changes)
    onnx.helper.set_model_props(model, metadata)
    onnx.save(model, model_path)
    deploy_path = path / "params/deploy.yaml"
    cfg = yaml.safe_load(deploy_path.read_text())
    if bind_hash:
        cfg["provenance"]["policy_sha256"] = hashlib.sha256(model_path.read_bytes()).hexdigest()
    if recipe:
        cfg["contracts"]["training_recipe"] = {"name":recipe,"version":"99"}
    deploy_path.write_text(yaml.safe_dump(cfg, sort_keys=False))


def test_native_two_file_pair_loads(pair):
    dist, policy, dim = pair
    result = native_inspect(dist, policy)
    assert result["policy_abi"] == ("ball_clock_110_v1" if dim == 110 else f"small_station_{dim}_v1")
    assert result["observation_dim"] == dim


def test_new_training_provenance_does_not_change_runtime(pair):
    dist, policy, dim = pair
    change_metadata(policy, {
        "hitter_pure_training_recipe":"future_reward_only_recipe",
        "hitter_pure_training_recipe_version":"99",
        "hitter_pingpong_build_contract":"future_training_build",
        "hitter_pingpong_reward_contract":"future_reward",
        "hitter_pingpong_training_initialization_contract":"future_initialization",
    }, recipe="future_reward_only_recipe")
    result = native_inspect(dist, policy)
    assert result["policy_abi"] == ("ball_clock_110_v1" if dim == 110 else f"small_station_{dim}_v1")
    assert result["training_recipe"] == "future_reward_only_recipe"


def test_mixed_deploy_onnx_pair_fails_before_transport(pair):
    dist, policy, _ = pair
    change_metadata(policy, {"test_provenance":"changed"}, bind_hash=False)
    with pytest.raises(ValueError, match="policy_sha256"):
        native_inspect(dist, policy)


def test_unknown_clock_is_not_mistaken_for_same_abi(pair):
    dist, policy, dim = pair
    key = "hitter_pingpong_rally_end_contract" if dim == 110 else "hitter_pingpong_wait_clock_contract"
    change_metadata(policy, {key:"unknown_clock_v2"})
    with pytest.raises(ValueError, match="rally end" if dim == 110 else "wait_clock_contract"):
        native_inspect(dist, policy)


def test_unknown_action_semantics_are_rejected(pair):
    dist, policy, _ = pair
    change_metadata(policy, {"qdes_action_contract":"unknown_decoder_v2"})
    with pytest.raises(ValueError, match="qdes_action_contract"):
        native_inspect(dist, policy)
