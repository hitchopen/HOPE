# Copyright (c) 2025, Intelligent Racing Inc. (dba Hitch Interactive).
# SPDX-License-Identifier: Apache-2.0
"""Export a trained HOPE PingPong actor to a single-output ONNX policy + a JSON manifest.

The exported graph is a plain feed-forward actor:

    observation[1, 111]  ->  raw_action[1, 31]

There is exactly ONE output (the raw action). No reference-motion tensors, no side / status /
debug heads, no multi-layout contract table, and no training lineage. The observation is fed to
the network RAW (no normalization is baked in and none is expected at deploy) unless a non-identity
normalizer is explicitly supplied, in which case it is folded into the graph and the manifest records
it. See ``policy_manifest.json`` and ``docs/POLICY_INTERFACE.md`` for the authoritative contract.
"""

from __future__ import annotations

import json
import os

# NOTE: torch / onnx are imported lazily inside the functions that need them, so this module (and its
# manifest/contract helpers) can be imported and unit-tested without torch installed.

# ---------------------------------------------------------------------------------------------------
# Public contract constants (see docs/POLICY_INTERFACE.md and the observation contract module).
# ---------------------------------------------------------------------------------------------------
CONTRACT_NAME = "hope_pingpong"
OBS_DIM = 111
ACTION_DIM = 31
CONTROL_RATE_HZ = 50
# The ActionAdapter constants (default_q, action_scale, clamp limits) are shipped as an editable
# EXAMPLE config, referenced by path and read by BOTH training and the reference runner. They are not
# baked into the policy.
ACTION_ADAPTER_CONFIG = "a3_deploy/a3_deploy_example/config/action_adapter.yaml"
# Canonical joint-order source (31 DOF), relative to the repo root.
JOINT_ORDER_CONFIG = "hope_training/config/joint_order_agibot_a3.yaml"

# The public 111-D observation layout (docs/POLICY_INTERFACE.md §Observation). Recorded in the
# manifest for documentation; it is fixed by the contract.
OBSERVATION_LAYOUT = [
    {"name": "base_ang_vel", "slice": [0, 3], "dim": 3},
    {"name": "joint_pos", "slice": [3, 34], "dim": 31},
    {"name": "joint_vel", "slice": [34, 65], "dim": 31},
    {"name": "last_action", "slice": [65, 96], "dim": 31},
    {"name": "projected_gravity", "slice": [96, 99], "dim": 3},
    {"name": "base_forward_xy", "slice": [99, 101], "dim": 2},
    {"name": "fixed_station_error_xy", "slice": [101, 103], "dim": 2},
    {"name": "racket_target_rel_base", "slice": [103, 106], "dim": 3},
    {"name": "racket_target_vel_w", "slice": [106, 109], "dim": 3},
    {"name": "time_to_strike", "slice": [109, 110], "dim": 1},
    {"name": "swing_side", "slice": [110, 111], "dim": 1},
]


def _build_exporter_module(actor_critic, normalizer=None):
    """Build the single-output actor wrapper module (torch imported lazily)."""
    import copy

    import torch.nn as nn

    actor = getattr(actor_critic, "actor", None)
    if actor is None:
        raise AttributeError("actor_critic has no `.actor` attribute — expected an rsl_rl ActorCritic.")

    class _OnnxActorExporter(nn.Module):
        """Wraps the trained actor (+ optional input normalizer) as a single-output module."""

        def __init__(self):
            super().__init__()
            # Deep-copy so exporting never mutates the live training module.
            self.actor = copy.deepcopy(actor)
            self.normalizer = copy.deepcopy(normalizer) if normalizer is not None else nn.Identity()

        def forward(self, observation):
            return self.actor(self.normalizer(observation))

        @property
        def in_features(self) -> int:
            return int(self.actor[0].in_features)

        @property
        def out_features(self) -> int:
            return int(self.actor[-1].out_features)

    return _OnnxActorExporter()


def export_policy_as_onnx(
    actor_critic,
    path: str,
    filename: str = "hope_pingpong.onnx",
    normalizer=None,
    verbose: bool = False,
) -> str:
    """Export ``actor_critic`` to ``path/filename`` as a single-output ONNX graph.

    Returns the absolute path of the written ONNX file. Raises ``ValueError`` if the actor does not
    match the public 111 -> 31 contract.
    """
    import torch

    os.makedirs(path, exist_ok=True)
    exporter = _build_exporter_module(actor_critic, normalizer).to("cpu").eval()

    if exporter.in_features != OBS_DIM or exporter.out_features != ACTION_DIM:
        raise ValueError(
            f"actor shape {exporter.in_features} -> {exporter.out_features} does not match the "
            f"public contract {OBS_DIM} -> {ACTION_DIM}. Refusing to export a non-contract policy."
        )

    onnx_path = os.path.join(path, filename)
    dummy_obs = torch.zeros(1, OBS_DIM)
    torch.onnx.export(
        exporter,
        dummy_obs,
        onnx_path,
        export_params=True,
        opset_version=11,
        verbose=verbose,
        input_names=["observation"],
        output_names=["raw_action"],
        dynamic_axes={"observation": {0: "batch"}, "raw_action": {0: "batch"}},
    )
    return os.path.abspath(onnx_path)


def _default_joint_order() -> list[str] | None:
    """Read the canonical 31-DOF joint order from ``joint_order_agibot_a3.yaml`` (walk up), or None."""
    try:
        import yaml
    except ImportError:
        return None
    d = os.path.dirname(os.path.abspath(__file__))
    for _ in range(12):
        cand = os.path.join(d, JOINT_ORDER_CONFIG)
        if os.path.isfile(cand):
            with open(cand) as f:
                data = yaml.safe_load(f) or {}
            order = data.get("joint_order")
            return list(order) if order else None
        parent = os.path.dirname(d)
        if parent == d:
            break
        d = parent
    return None


def build_manifest(
    joint_names: list[str] | None = None,
    observation_normalization: str = "none",
    onnx_filename: str = "hope_pingpong.onnx",
) -> dict:
    """Assemble the ``policy_manifest.json`` dictionary for the exported policy."""
    if joint_names is None:
        joint_names = _default_joint_order()
    manifest = {
        "contract_name": CONTRACT_NAME,
        "onnx_file": onnx_filename,
        "obs_dim": OBS_DIM,
        "action_dim": ACTION_DIM,
        "control_rate_hz": CONTROL_RATE_HZ,
        "observation_normalization": observation_normalization,
        "onnx_signature": {
            "input": {"name": "observation", "shape": [1, OBS_DIM], "dtype": "float32"},
            "output": {"name": "raw_action", "shape": [1, ACTION_DIM], "dtype": "float32"},
        },
        "observation_layout": OBSERVATION_LAYOUT,
        "joint_order_source": JOINT_ORDER_CONFIG,
        "joint_order": list(joint_names) if joint_names else None,
        "action_adapter_config": ACTION_ADAPTER_CONFIG,
    }
    return manifest


def write_policy_manifest(
    path: str,
    joint_names: list[str] | None = None,
    observation_normalization: str = "none",
    onnx_filename: str = "hope_pingpong.onnx",
    manifest_filename: str = "policy_manifest.json",
) -> str:
    """Write ``policy_manifest.json`` next to the ONNX. Returns the absolute manifest path."""
    os.makedirs(path, exist_ok=True)
    manifest = build_manifest(joint_names, observation_normalization, onnx_filename)
    manifest_path = os.path.join(path, manifest_filename)
    with open(manifest_path, "w", encoding="utf-8") as f:
        json.dump(manifest, f, indent=2, sort_keys=False)
        f.write("\n")
    return os.path.abspath(manifest_path)


def attach_onnx_metadata(onnx_path: str, joint_names: list[str] | None = None,
                         observation_normalization: str = "none") -> None:
    """Embed the compact contract fields into the ONNX file's metadata (for the deploy backend).

    Only contract-level fields are embedded (dims, control rate, joint order, contract name, action
    adapter config path). No PD gains, action scale, reward, or lineage data is written.
    """
    import onnx

    if joint_names is None:
        joint_names = _default_joint_order()
    meta = {
        "contract_name": CONTRACT_NAME,
        "obs_dim": str(OBS_DIM),
        "action_dim": str(ACTION_DIM),
        "control_rate_hz": str(CONTROL_RATE_HZ),
        "observation_normalization": observation_normalization,
        "action_adapter_config": ACTION_ADAPTER_CONFIG,
        "joint_order": ",".join(joint_names) if joint_names else "",
    }
    model = onnx.load(onnx_path)
    for key, value in meta.items():
        entry = model.metadata_props.add()
        entry.key = key
        entry.value = value
    onnx.save(model, onnx_path)


def export_policy(
    actor_critic,
    path: str,
    joint_names: list[str] | None = None,
    normalizer=None,
    onnx_filename: str = "hope_pingpong.onnx",
    verbose: bool = False,
) -> tuple[str, str]:
    """Export the ONNX policy + write ``policy_manifest.json`` + embed ONNX metadata.

    Returns ``(onnx_path, manifest_path)``. ``normalizer`` should be left None (raw-observation
    contract); pass one only if you deliberately baked normalization into training.
    """
    import torch.nn as nn

    obs_norm = "none" if (normalizer is None or isinstance(normalizer, nn.Identity)) else "empirical"
    onnx_path = export_policy_as_onnx(actor_critic, path, onnx_filename, normalizer, verbose)
    attach_onnx_metadata(onnx_path, joint_names, obs_norm)
    manifest_path = write_policy_manifest(path, joint_names, obs_norm, onnx_filename)
    return onnx_path, manifest_path
