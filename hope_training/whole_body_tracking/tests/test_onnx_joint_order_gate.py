"""The deploy runner must reject an ONNX whose embedded joint order is permuted.

The exporter hard-gates the articulation enumeration against the canonical joint order
and embeds it in the ONNX metadata; ``OnnxPolicy`` re-validates that metadata at load
time so a previously exported (or foreign) policy with a different column order can
never drive the robot with silently permuted joints. Models without the metadata key
(e.g. hand-authored test actors) load unchecked.

Skipped automatically when ``onnx`` / ``onnxruntime`` are unavailable.
"""

from __future__ import annotations

import os
import sys

import numpy as np
import pytest

onnx = pytest.importorskip("onnx")
pytest.importorskip("onnxruntime")

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_REPO = os.path.dirname(os.path.dirname(_ROOT))
sys.path.insert(0, os.path.join(_REPO, "a3_deploy", "a3_deploy_example", "reference"))

from a3_deploy_onnx_ref_pingpong.joint_order import JOINT_NAMES  # noqa: E402
from a3_deploy_onnx_ref_pingpong.onnx_policy import OnnxPolicy  # noqa: E402


def _tiny_actor(path: str, joint_order: list[str] | None) -> str:
    from onnx import TensorProto, helper

    W = np.zeros((111, 31), dtype=np.float32)
    graph = helper.make_graph(
        [helper.make_node("MatMul", ["observation", "W"], ["raw_action"])],
        "tiny_actor",
        [helper.make_tensor_value_info("observation", TensorProto.FLOAT, [1, 111])],
        [helper.make_tensor_value_info("raw_action", TensorProto.FLOAT, [1, 31])],
        initializer=[helper.make_tensor("W", TensorProto.FLOAT, W.shape, W.flatten())],
    )
    model = helper.make_model(graph, opset_imports=[helper.make_opsetid("", 17)])
    model.ir_version = 8
    if joint_order is not None:
        entry = model.metadata_props.add()
        entry.key = "joint_order"
        entry.value = ",".join(joint_order)
    onnx.save(model, path)
    return path


def test_canonical_joint_order_metadata_accepted(tmp_path):
    path = _tiny_actor(str(tmp_path / "ok.onnx"), list(JOINT_NAMES))
    OnnxPolicy(path)  # must not raise


def test_permuted_joint_order_metadata_rejected(tmp_path):
    permuted = list(JOINT_NAMES)
    permuted[0], permuted[-1] = permuted[-1], permuted[0]
    path = _tiny_actor(str(tmp_path / "bad.onnx"), permuted)
    with pytest.raises(ValueError, match="joint_order"):
        OnnxPolicy(path)


def test_metadata_less_model_accepted(tmp_path):
    path = _tiny_actor(str(tmp_path / "plain.onnx"), None)
    OnnxPolicy(path)  # must not raise
