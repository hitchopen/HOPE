"""Unit tests for the exported policy contract (obs 110 hitter_pure / action 31 / manifest schema).

The authoritative layout lives in the actor observation contract REGISTRY
(``tasks/tracking/actor_observation_contract.py``); the shipped task YAML names its
entry (``actor_obs_contract: hitter_pure``) and the reference deploy package and the
export manifest must agree with it. Modules are loaded by file path so the tests run
without torch / Isaac.

Run:  python tests/test_policy_contract.py   (or pytest)
"""

from __future__ import annotations

import importlib.util
import os
import sys

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_REPO = os.path.dirname(os.path.dirname(_ROOT))
_TRACKING = os.path.join(
    _ROOT, "source", "whole_body_tracking", "whole_body_tracking", "tasks", "tracking"
)
_JOINT_ORDER_YAML = os.path.abspath(
    os.path.join(_ROOT, "..", "config", "joint_order_agibot_a3.yaml")
)
_TASK_YAML = os.path.join(_ROOT, "cfg", "task", "HOPEPingPong.yaml")
_REFERENCE_DIR = os.path.join(_REPO, "a3_deploy", "a3_deploy_example", "reference")


def _load(name: str, path: str):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    spec.loader.exec_module(mod)
    return mod


registry = _load(
    "hope_actor_obs_contract", os.path.join(_TRACKING, "actor_observation_contract.py")
)
export_script = _load("hope_export_onnx_script", os.path.join(_ROOT, "scripts", "export_onnx.py"))

# The exact hitter_pure layout, in order (name, dim). Slices follow contiguously.
_EXPECTED_LAYOUT = (
    ("base_ang_vel", 3),
    ("joint_pos", 31),
    ("joint_vel", 31),
    ("actions", 31),
    ("projected_gravity", 3),
    ("base_forward_xy", 2),
    ("base_target_delta_xy", 2),
    ("racket_target_rel_base", 3),
    ("racket_target_vel_w", 3),
    ("time_to_strike", 1),
)


def test_registry_hitter_pure_dims_and_layout():
    contract = registry.resolve_actor_observation_contract("hitter_pure")
    assert contract is registry.HITTER_PURE
    assert contract.total_dim == 110
    assert contract.layout == _EXPECTED_LAYOUT
    assert sum(dim for _, dim in contract.layout) == 110
    # No swing-side observation slot anywhere in the layout.
    names = [name for name, _ in contract.layout]
    assert "swing_side" not in names and "swing_type" not in names
    # obs_mode aliases resolve to the same entry.
    assert registry.resolve_actor_observation_contract(contract.obs_mode) is contract


def test_registry_slices_cover_110_contiguously():
    contract = registry.HITTER_PURE
    cursor = 0
    for name, dim in contract.layout:
        assert dim > 0, name
        cursor += dim
    assert cursor == contract.total_dim == 110
    # The final term is the strike clock (the layout ends on time_to_strike, not a side flag).
    assert contract.layout[-1] == ("time_to_strike", 1)


def test_task_yaml_declares_hitter_pure():
    import yaml

    with open(_TASK_YAML) as f:
        task = yaml.safe_load(f)
    assert task["actor_obs_contract"] == "hitter_pure"
    assert task["gym_task"] == "HOPE-HitterPingPong-AgibotA3-v0"
    assert int(task["build_contract"]["actor_observation_dim"]) == 110
    assert int(task["build_contract"]["action_dim"]) == 31


def test_reference_package_matches_registry():
    sys.path.insert(0, _REFERENCE_DIR)
    try:
        from a3_deploy_onnx_ref_pingpong.observation import CONTRACT_NAME, OBS_DIM
    finally:
        sys.path.remove(_REFERENCE_DIR)
    contract = registry.HITTER_PURE
    assert CONTRACT_NAME == contract.name == "hitter_pure"
    assert OBS_DIM == contract.total_dim == 110


def test_manifest_schema():
    joint_names = [f"j{i}" for i in range(31)]
    manifest = export_script.build_manifest(registry.HITTER_PURE, joint_names)
    assert manifest["contract_name"] == "hitter_pure"
    assert manifest["obs_dim"] == 110
    assert manifest["action_dim"] == 31
    assert manifest["control_rate_hz"] == 50
    assert manifest["observation_normalization"] == "none"
    assert manifest["action_adapter_config"].endswith("action_adapter.yaml")
    sig = manifest["onnx_signature"]
    assert sig["input"]["shape"] == [1, 110] and sig["output"]["shape"] == [1, 31]
    assert manifest["joint_order"] == joint_names
    # The manifest layout mirrors the registry: contiguous slices summing to 110.
    cursor = 0
    for term, (name, dim) in zip(manifest["observation_layout"], registry.HITTER_PURE.layout):
        assert term["name"] == name and term["dim"] == dim
        assert term["slice"] == [cursor, cursor + dim]
        cursor += dim
    assert cursor == 110
    # No lineage / recipe / metric / wandb fields.
    forbidden = {"recipe", "lineage", "receipt", "wandb", "metrics", "success_rate", "version"}
    assert not (forbidden & set(manifest.keys()))


def test_joint_order_yaml_has_31_unique_joints():
    import yaml

    with open(_JOINT_ORDER_YAML) as f:
        data = yaml.safe_load(f)
    order = data["joint_order"]
    assert len(order) == 31
    assert len(set(order)) == 31
    assert order[0] == "waist_yaw_joint"
    assert order[3] == "head_yaw_joint" and order[4] == "head_pitch_joint"  # passive-at-deploy neck


def main() -> int:
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_") and callable(v)]
    failed = 0
    for fn in tests:
        try:
            fn()
            print(f"[ok] {fn.__name__}")
        except AssertionError as e:
            failed += 1
            print(f"[FAIL] {fn.__name__}: {e}")
    print(f"\n{len(tests) - failed}/{len(tests)} policy-contract tests passed")
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
