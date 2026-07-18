# Copyright (c) 2026 Intelligent Racing Inc. (dba Hitch Interactive)
# SPDX-License-Identifier: Apache-2.0
"""ONNX actor wrapper.

The exported policy is a single-input, single-output network:

    observation[1, 111]  ->  raw_action[1, 31]

No observation normalization is applied (the observation is raw). The runner zeroes
the passive head columns of ``raw_action`` to form the applied action, which is fed
back as the next tick's ``last_action`` and passed through the ActionAdapter.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np

from .observation import OBS_DIM
from .joint_order import JOINT_NAMES, NUM_JOINTS


class OnnxPolicy:
    def __init__(self, onnx_path: str | Path, providers: list[str] | None = None) -> None:
        import onnxruntime as ort  # imported lazily so the module imports without ORT

        onnx_path = str(onnx_path)
        if not Path(onnx_path).is_file():
            raise FileNotFoundError(f"ONNX policy not found: {onnx_path}")

        so = ort.SessionOptions()
        so.intra_op_num_threads = 1
        so.inter_op_num_threads = 1
        self._sess = ort.InferenceSession(
            onnx_path, sess_options=so, providers=providers or ["CPUExecutionProvider"]
        )

        inputs = self._sess.get_inputs()
        outputs = self._sess.get_outputs()
        if len(inputs) != 1 or len(outputs) != 1:
            raise ValueError("expected a single-input, single-output actor ONNX")
        self._input_name = inputs[0].name
        self._output_name = outputs[0].name
        self._validate_shape(inputs[0].shape, OBS_DIM, "observation input")
        self._validate_shape(outputs[0].shape, NUM_JOINTS, "raw_action output")

        # If the exporter embedded the joint order, it must equal the canonical order
        # this runner assumes for every obs/action column — a mismatch means every
        # joint column would be silently permuted. Models without the metadata key
        # (e.g. hand-authored test actors) are accepted unchecked.
        meta = self._sess.get_modelmeta().custom_metadata_map or {}
        embedded = meta.get("joint_order", "")
        if embedded:
            embedded_names = tuple(embedded.split(","))
            if embedded_names != tuple(JOINT_NAMES):
                raise ValueError(
                    "ONNX metadata joint_order does not match this runner's canonical "
                    f"joint order.\n  onnx:      {list(embedded_names)}\n"
                    f"  canonical: {list(JOINT_NAMES)}\n"
                    "Re-export the policy from an asset whose articulation enumerates "
                    "joints in the canonical order (joint_order_agibot_a3.yaml)."
                )

    @staticmethod
    def _validate_shape(shape, expected_last: int, what: str) -> None:
        # Trailing dim must match; leading (batch) dim may be dynamic/None.
        if shape and isinstance(shape[-1], int) and shape[-1] != expected_last:
            raise ValueError(f"{what} trailing dim {shape[-1]} != expected {expected_last}")

    def infer(self, obs: np.ndarray) -> np.ndarray:
        """Run one forward pass. Returns ``raw_action`` as ``float32`` length 31."""
        x = np.asarray(obs, dtype=np.float32).reshape(1, OBS_DIM)
        out = self._sess.run([self._output_name], {self._input_name: x})[0]
        return np.asarray(out, dtype=np.float32).reshape(NUM_JOINTS)
