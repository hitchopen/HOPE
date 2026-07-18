# Copyright (c) 2026 Intelligent Racing Inc. (dba Hitch Interactive)
# SPDX-License-Identifier: Apache-2.0
"""Runtime configuration loader for the reference runner.

Reads ``config/hope_pingpong_runtime.yaml`` (the clean 111-D runtime config) and
resolves the ActionAdapter, the example simulation PD gains, and the lifecycle
timing into ready-to-use arrays. All relative paths in the YAML are resolved
against the YAML file's own directory.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import yaml

from .action_adapter import ActionAdapter
from .joint_order import NUM_JOINTS
from .lifecycle import LifecycleConfig

# Index ranges of the four joint groups (used to expand example PD gains).
_GROUP_RANGES = {
    "waist": range(0, 3),
    "neck": range(3, 5),
    "arm": range(5, 19),
    "leg": range(19, 31),
}


@dataclass
class RuntimeConfig:
    control_hz: float
    onnx_path: Path
    model_xml_path: Path
    action_adapter: ActionAdapter
    sim_kp: np.ndarray
    sim_kd: np.ndarray
    lifecycle: LifecycleConfig
    passive_neck: bool = True
    config_dir: Path = field(default_factory=Path)

    @property
    def control_dt(self) -> float:
        return 1.0 / float(self.control_hz)

    @classmethod
    def load(cls, path: str | Path) -> "RuntimeConfig":
        path = Path(path).resolve()
        cfg_dir = path.parent
        with open(path, "r", encoding="utf-8") as fh:
            doc = yaml.safe_load(fh)

        norm = str(doc.get("observation_normalization", "none")).lower()
        if norm != "none":
            raise ValueError(
                f"observation_normalization must be 'none' (raw obs), got '{norm}'"
            )

        control_hz = float(doc.get("control_hz", 50.0))
        dt = 1.0 / control_hz

        onnx_path = _resolve(cfg_dir, doc["policy"]["onnx_path"])
        model_xml_path = _resolve(cfg_dir, doc["simulation"]["model_xml_path"])
        adapter_path = _resolve(cfg_dir, doc["action_adapter"]["config_path"])
        adapter = ActionAdapter.from_yaml(adapter_path)

        sim_kp, sim_kd = _expand_pd_gains(doc["simulation"]["pd_gains"])

        life_doc = doc.get("lifecycle", {})
        lifecycle = LifecycleConfig(
            dt=dt,
            follow_through_s=float(life_doc.get("follow_through_s", 0.6)),
            recovery_s=float(life_doc.get("recovery_s", 0.8)),
            ready_time_to_strike=float(life_doc.get("ready_time_to_strike", 1.0)),
            ready_reach_x=float(life_doc.get("ready_reach_x", 0.40)),
            ready_reach_y=float(life_doc.get("ready_reach_y", 0.20)),
            ready_reach_z=float(life_doc.get("ready_reach_z", -0.05)),
        )

        return cls(
            control_hz=control_hz,
            onnx_path=onnx_path,
            model_xml_path=model_xml_path,
            action_adapter=adapter,
            sim_kp=sim_kp,
            sim_kd=sim_kd,
            lifecycle=lifecycle,
            passive_neck=bool(doc.get("passive_neck", True)),
            config_dir=cfg_dir,
        )


def _resolve(base: Path, rel: str) -> Path:
    p = Path(rel)
    return p if p.is_absolute() else (base / p).resolve()


def _expand_pd_gains(spec: dict) -> tuple[np.ndarray, np.ndarray]:
    """Expand per-group example gains into length-31 kp/kd arrays."""
    kp = np.zeros(NUM_JOINTS, dtype=np.float64)
    kd = np.zeros(NUM_JOINTS, dtype=np.float64)
    groups = spec.get("groups", {})
    for name, rng in _GROUP_RANGES.items():
        if name not in groups:
            raise ValueError(f"simulation.pd_gains.groups is missing '{name}'")
        g = groups[name]
        for i in rng:
            kp[i] = float(g["kp"])
            kd[i] = float(g["kd"])
    return kp, kd
