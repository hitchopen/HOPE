"""Unit tests for the exported policy contract (obs 111 / action 31 / manifest schema).

Loaded by file path so it runs without torch / Isaac (the exporter imports torch lazily).

Run:  python tests/test_policy_contract.py
"""

from __future__ import annotations

import importlib.util
import os
import sys

_ROOT = os.path.dirname(os.path.dirname(__file__))
_UTILS = os.path.join(_ROOT, "source", "whole_body_tracking", "whole_body_tracking", "utils")
_JOINT_ORDER_YAML = os.path.abspath(
    os.path.join(_ROOT, "..", "config", "joint_order_agibot_a3.yaml")
)


def _load(name: str, filename: str):
    path = os.path.join(_UTILS, filename)
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    spec.loader.exec_module(mod)
    return mod


exporter = _load("hope_exporter", "exporter.py")


def test_contract_dims():
    assert exporter.OBS_DIM == 111
    assert exporter.ACTION_DIM == 31
    assert exporter.CONTROL_RATE_HZ == 50
    assert exporter.CONTRACT_NAME == "hope_pingpong"


def test_observation_layout_covers_111_contiguously():
    layout = exporter.OBSERVATION_LAYOUT
    assert sum(t["dim"] for t in layout) == 111
    cursor = 0
    for term in layout:
        lo, hi = term["slice"]
        assert lo == cursor, f"gap/overlap before {term['name']}"
        assert hi - lo == term["dim"]
        cursor = hi
    assert cursor == 111
    # The trailing swing_side term distinguishes this from the proprioception-only 110-D layout.
    assert layout[-1]["name"] == "swing_side" and layout[-1]["dim"] == 1


def test_manifest_schema():
    joint_names = [f"j{i}" for i in range(31)]
    manifest = exporter.build_manifest(joint_names=joint_names)
    assert manifest["contract_name"] == "hope_pingpong"
    assert manifest["obs_dim"] == 111
    assert manifest["action_dim"] == 31
    assert manifest["control_rate_hz"] == 50
    assert manifest["observation_normalization"] == "none"
    assert manifest["action_adapter_config"].endswith("action_adapter.yaml")
    sig = manifest["onnx_signature"]
    assert sig["input"]["shape"] == [1, 111] and sig["output"]["shape"] == [1, 31]
    assert manifest["joint_order"] == joint_names
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
