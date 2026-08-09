# Copyright (c) 2026 Intelligent Racing Inc. (dba Hitch Interactive)
# SPDX-License-Identifier: Apache-2.0
"""ONNX actor wrapper.

The deployed model_21800 graph has the actor input plus a reference-motion clock:

    obs[1, 110], time_step[1, 1]  ->  actions[1, 31] + reference side outputs

Compact single-input/single-output actors exported from the public training path
remain supported as well.

No observation normalization is applied (the observation is raw). The runner zeroes
the passive head columns of ``raw_action`` to form the applied action, which is fed
back as the next tick's ``last_action`` and passed through the ActionAdapter.

Load-time gates (mirrored by the exporter's manifest):
  * the trailing input/output dims must be OBS_DIM (110) / NUM_JOINTS (31);
  * an embedded ``joint_order`` metadata string must equal the canonical order;
  * an embedded ``contract`` metadata string must equal ``hitter_pure``.
Models without the metadata keys (e.g. hand-authored test actors) load unchecked.
The metadata validators are module-level pure functions so they are unit-testable
without onnxruntime.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np

from .observation import CONTRACT_NAME, OBS_DIM
from .joint_order import JOINT_NAMES, NUM_JOINTS


def validate_embedded_joint_order(embedded: str) -> None:
    """Reject a non-empty embedded ``joint_order`` that mismatches the canonical order.

    If the exporter embedded the joint order, it must equal the canonical order
    this runner assumes for every obs/action column — a mismatch means every joint
    column would be silently permuted. An empty string (metadata absent) passes.
    """
    if not embedded:
        return
    embedded_names = tuple(embedded.split(","))
    if embedded_names != tuple(JOINT_NAMES):
        raise ValueError(
            "ONNX metadata joint_order does not match this runner's canonical "
            f"joint order.\n  onnx:      {list(embedded_names)}\n"
            f"  canonical: {list(JOINT_NAMES)}\n"
            "Re-export the policy from an asset whose articulation enumerates "
            "joints in the canonical order (joint_order_agibot_a3.yaml)."
        )


def validate_embedded_contract(embedded: str) -> None:
    """Reject a non-empty embedded ``contract`` that is not the 110-D hitter_pure.

    An empty string (metadata absent) passes.
    """
    if not embedded:
        return
    if embedded != CONTRACT_NAME:
        raise ValueError(
            f"ONNX metadata contract '{embedded}' does not match this runner's "
            f"observation contract '{CONTRACT_NAME}' ({OBS_DIM}-D). Export the "
            "policy from a task using the hitter_pure actor observation contract."
        )


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

        inputs = {item.name: item for item in self._sess.get_inputs()}
        outputs = {item.name: item for item in self._sess.get_outputs()}
        obs_input = inputs.get("obs") or inputs.get("observation")
        if obs_input is None and len(inputs) == 1:
            obs_input = next(iter(inputs.values()))
        action_output = outputs.get("actions") or outputs.get("raw_action")
        if action_output is None and len(outputs) == 1:
            action_output = next(iter(outputs.values()))
        if obs_input is None or action_output is None:
            raise ValueError(
                "ONNX must expose obs/observation input and actions/raw_action output"
            )
        unknown_inputs = set(inputs) - {obs_input.name, "time_step"}
        if unknown_inputs:
            raise ValueError(f"unsupported ONNX actor inputs: {sorted(unknown_inputs)}")
        self._input_name = obs_input.name
        self._time_step_name = "time_step" if "time_step" in inputs else None
        self._output_name = action_output.name
        self._validate_shape(obs_input.shape, OBS_DIM, "observation input")
        if self._time_step_name is not None:
            self._validate_shape(inputs[self._time_step_name].shape, 1, "time_step input")
        self._validate_shape(action_output.shape, NUM_JOINTS, "raw_action output")

        meta = self._sess.get_modelmeta().custom_metadata_map or {}
        validate_embedded_joint_order(meta.get("joint_order", ""))
        validate_embedded_contract(meta.get("contract", meta.get("actor_obs_contract", "")))

        policy_names = tuple(filter(None, meta.get("joint_names", "").split(",")))
        if policy_names:
            if len(policy_names) != NUM_JOINTS or set(policy_names) != set(JOINT_NAMES):
                raise ValueError("ONNX joint_names must be a permutation of the 31 A3 joints")
            sdk_index = {name: index for index, name in enumerate(JOINT_NAMES)}
            self._policy_to_sdk = np.asarray(
                [sdk_index[name] for name in policy_names], dtype=np.int64
            )
        else:
            self._policy_to_sdk = np.arange(NUM_JOINTS, dtype=np.int64)

        self._clip_lengths = self._parse_pair(meta.get("clip_seg_lengths", ""), int)
        self._strike_phases = self._parse_pair(meta.get("clip_strike_phases", ""), float)
        if self._time_step_name and (self._clip_lengths is None or self._strike_phases is None):
            raise ValueError(
                "time_step ONNX input requires clip_seg_lengths and clip_strike_phases metadata"
            )

    @staticmethod
    def _validate_shape(shape, expected_last: int, what: str) -> None:
        # Trailing dim must match; leading (batch) dim may be dynamic/None.
        if shape and isinstance(shape[-1], int) and shape[-1] != expected_last:
            raise ValueError(f"{what} trailing dim {shape[-1]} != expected {expected_last}")

    @staticmethod
    def _parse_pair(raw: str, cast):
        if not raw:
            return None
        values = tuple(cast(item) for item in raw.split(","))
        return values if len(values) == 2 else None

    @staticmethod
    def _round_positive(value: float) -> int:
        return int(np.floor(float(value) + 0.5))

    def reference_time_step(
        self, time_to_strike: float, swing_sign: float, step_dt: float = 0.02
    ) -> int:
        if self._time_step_name is None:
            return 0
        clip_id = 0 if float(swing_sign) > 0.0 else 1
        length = int(self._clip_lengths[clip_id])
        start = 0 if clip_id == 0 else int(self._clip_lengths[0])
        strike = start + self._round_positive(
            float(self._strike_phases[clip_id]) * (length - 1)
        )
        raw = strike - float(time_to_strike) / max(float(step_dt), 1.0e-6)
        return max(start, min(start + length - 1, self._round_positive(raw)))

    def _observation_to_policy_order(self, obs: np.ndarray) -> np.ndarray:
        x = np.asarray(obs, dtype=np.float32).reshape(OBS_DIM).copy()
        for slc in (slice(3, 34), slice(34, 65), slice(65, 96)):
            sdk_values = x[slc].copy()
            x[slc] = sdk_values[self._policy_to_sdk]
        return x.reshape(1, OBS_DIM)

    def infer(self, obs: np.ndarray, time_step: int = 0) -> np.ndarray:
        """Run one actor pass and return raw action in MuJoCo/SDK joint order."""
        feeds = {self._input_name: self._observation_to_policy_order(obs)}
        if self._time_step_name is not None:
            feeds[self._time_step_name] = np.asarray([[time_step]], dtype=np.float32)
        policy_action = self._sess.run([self._output_name], feeds)[0]
        policy_action = np.asarray(policy_action, dtype=np.float32).reshape(NUM_JOINTS)
        sdk_action = np.empty(NUM_JOINTS, dtype=np.float32)
        sdk_action[self._policy_to_sdk] = policy_action
        return sdk_action

    def infer_target(
        self,
        obs: np.ndarray,
        time_to_strike: float,
        swing_sign: float,
        step_dt: float = 0.02,
    ) -> np.ndarray:
        return self.infer(
            obs,
            time_step=self.reference_time_step(time_to_strike, swing_sign, step_dt),
        )
