"""Export/loader joint-order gate: a permuted ONNX column order must be rejected.

Two layers, both exercised against the CURRENT implementations:

  * EXPORT side — ``scripts/export_onnx.py`` hard-gates the articulation enumeration
    against the canonical joint order (``assert_canonical_joint_order``) and embeds it
    in the ONNX metadata (key ``joint_order``, plus the ``contract`` name).
  * LOADER side — ``OnnxPolicy`` re-validates that metadata at load time through the
    module-level pure validators ``validate_embedded_joint_order`` /
    ``validate_embedded_contract`` (110-D hitter_pure), so a previously exported (or
    foreign) policy with a different column order can never drive the robot with
    silently permuted joints. Models without the metadata keys load unchecked.

The metadata round-trip is checked through real ONNX files (the ``onnx`` package);
the validators are pure functions, so no onnxruntime session is needed on this host.

Run:  pytest tests/test_onnx_joint_order_gate.py
"""

from __future__ import annotations

import importlib.util
import os
import sys

import numpy as np
import pytest

onnx = pytest.importorskip("onnx")  # required: the gate is about ONNX metadata

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_REPO = os.path.dirname(os.path.dirname(_ROOT))
sys.path.insert(0, os.path.join(_REPO, "a3_deploy", "a3_deploy_example", "reference"))

from a3_deploy_onnx_ref_pingpong.joint_order import JOINT_NAMES  # noqa: E402
from a3_deploy_onnx_ref_pingpong import onnx_policy  # noqa: E402
from a3_deploy_onnx_ref_pingpong.observation import CONTRACT_NAME, OBS_DIM  # noqa: E402


def _load_by_path(name: str, path: str):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    spec.loader.exec_module(mod)
    return mod


# The export script is import-light at module level (heavy imports live in main()).
export_script = _load_by_path(
    "hope_export_onnx_script_gate", os.path.join(_ROOT, "scripts", "export_onnx.py")
)


def _tiny_actor(path: str, joint_order: list[str] | None, contract: str | None = None) -> str:
    """Write a minimal obs[1,110] -> raw_action[1,31] actor with optional metadata."""
    from onnx import TensorProto, helper

    W = np.zeros((OBS_DIM, 31), dtype=np.float32)
    graph = helper.make_graph(
        [helper.make_node("MatMul", ["observation", "W"], ["raw_action"])],
        "tiny_actor",
        [helper.make_tensor_value_info("observation", TensorProto.FLOAT, [1, OBS_DIM])],
        [helper.make_tensor_value_info("raw_action", TensorProto.FLOAT, [1, 31])],
        initializer=[helper.make_tensor("W", TensorProto.FLOAT, W.shape, W.flatten())],
    )
    model = helper.make_model(graph, opset_imports=[helper.make_opsetid("", 17)])
    model.ir_version = 8
    if joint_order is not None:
        entry = model.metadata_props.add()
        entry.key = "joint_order"
        entry.value = ",".join(joint_order)
    if contract is not None:
        entry = model.metadata_props.add()
        entry.key = "contract"
        entry.value = contract
    onnx.save(model, path)
    return path


def _metadata_map(path: str) -> dict:
    """Read the custom metadata exactly as the loader sees it."""
    model = onnx.load(path)
    return {p.key: p.value for p in model.metadata_props}


def _validate_like_loader(path: str) -> None:
    """Run the same metadata gates OnnxPolicy.__init__ applies after session setup."""
    meta = _metadata_map(path)
    onnx_policy.validate_embedded_joint_order(meta.get("joint_order", ""))
    onnx_policy.validate_embedded_contract(meta.get("contract", ""))


def test_canonical_joint_order_metadata_accepted(tmp_path):
    path = _tiny_actor(str(tmp_path / "ok.onnx"), list(JOINT_NAMES), CONTRACT_NAME)
    _validate_like_loader(path)  # must not raise


def test_permuted_joint_order_metadata_rejected(tmp_path):
    permuted = list(JOINT_NAMES)
    permuted[0], permuted[-1] = permuted[-1], permuted[0]
    path = _tiny_actor(str(tmp_path / "bad.onnx"), permuted)
    with pytest.raises(ValueError, match="joint_order"):
        _validate_like_loader(path)


def test_wrong_contract_metadata_rejected(tmp_path):
    path = _tiny_actor(str(tmp_path / "old.onnx"), list(JOINT_NAMES), "hope_pingpong")
    with pytest.raises(ValueError, match="hitter_pure"):
        _validate_like_loader(path)


def test_metadata_less_model_accepted(tmp_path):
    path = _tiny_actor(str(tmp_path / "plain.onnx"), None)
    _validate_like_loader(path)  # must not raise


def test_loader_shape_gate_is_110():
    # Trailing observation dim must be 110; the batch dim may be dynamic.
    onnx_policy.OnnxPolicy._validate_shape([1, OBS_DIM], OBS_DIM, "observation input")
    onnx_policy.OnnxPolicy._validate_shape([None, OBS_DIM], OBS_DIM, "observation input")
    with pytest.raises(ValueError, match="111"):
        onnx_policy.OnnxPolicy._validate_shape([1, 111], OBS_DIM, "observation input")
    assert OBS_DIM == 110


def test_export_gate_rejects_permuted_articulation():
    canonical = list(JOINT_NAMES)
    export_script.assert_canonical_joint_order(canonical, canonical)  # must not raise
    permuted = list(JOINT_NAMES)
    permuted[3], permuted[4] = permuted[4], permuted[3]
    with pytest.raises(RuntimeError, match="joint order"):
        export_script.assert_canonical_joint_order(permuted, canonical)


def test_canonical_yaml_order_matches_reference_runner():
    """The exporter's canonical order (joint_order_agibot_a3.yaml) == the loader's order."""
    import yaml

    yaml_path = os.path.abspath(os.path.join(_ROOT, "..", "config", "joint_order_agibot_a3.yaml"))
    with open(yaml_path) as f:
        order = tuple(str(n) for n in yaml.safe_load(f)["joint_order"])
    assert order == tuple(JOINT_NAMES)
