"""Deterministic robot-side rough-ground patch for RallyV17.

The authored range remains compatible with the historical ``[0.0, 0.04]``
spelling, but is re-centred about the calibrated floor: the actual collision
surface is ``[-0.02, +0.02]``.  The table side is exactly flat and the initial
standing footprint is also flat, avoiding a reset-time penetration impulse.

Everything above :func:`attach_rough_ground_patch` is NumPy-only so the terrain
contract can be tested and hashed without starting Isaac Sim.
"""

from __future__ import annotations

import hashlib
import math
from typing import Callable, Optional, Tuple

import numpy as np

GENERATOR_VERSION = "v17_correlated_spawn_flat_v1"
HORIZONTAL_SCALE_M = 0.1
VERTICAL_SCALE_M = 0.005
SLOPE_THRESHOLD = 0.75
X_BACK_M = 3.0
X_FORWARD_MARGIN_M = 0.5
Y_HALF_M = 3.0
SAFETY_FLOOR_MARGIN_M = 0.05
MIN_BAND_M = 0.01
MAX_BAND_M = 0.15
SPAWN_FLAT_RADIUS_M = 0.20
SPAWN_BLEND_RADIUS_M = 0.40
TABLE_BLEND_WIDTH_M = 0.30
SMOOTHING_PASSES = 4

ROUGH_PATCH_SCENE_ATTR = "rough_ground_patch"
SAFETY_FLOOR_SCENE_ATTR = "rough_safety_floor"


def zero_mean_half_band_m(height_range) -> float:
    lo, hi = float(height_range[0]), float(height_range[1])
    return (hi - lo) / 2.0


def _table_length_m() -> float:
    value = getattr(_table_length_m, "_value", None)
    if value is None:
        try:
            from whole_body_tracking.tasks.table_tennis import geometry

            value = float(geometry.TABLE_LENGTH)
        except Exception:
            import pathlib
            import re

            path = (
                pathlib.Path(__file__).resolve().parents[1]
                / "table_tennis"
                / "geometry.py"
            )
            match = re.search(
                r"^TABLE_LENGTH:\s*float\s*=\s*([0-9.]+)",
                path.read_text(encoding="utf-8"),
                flags=re.MULTILINE,
            )
            if match is None:
                raise RuntimeError(f"cannot read TABLE_LENGTH from {path}")
            value = float(match.group(1))
        _table_length_m._value = value
    return value


def patch_extents_m(near_x_m: Optional[float]) -> Tuple[float, float, float]:
    if near_x_m is None:
        return (-X_BACK_M, X_BACK_M, Y_HALF_M)
    x_min = float(near_x_m) - X_BACK_M
    x_max = float(near_x_m) + _table_length_m() + X_FORWARD_MARGIN_M
    return (x_min, x_max, Y_HALF_M)


def _validate_height_range(height_range) -> tuple[float, float, int]:
    if not isinstance(height_range, (tuple, list)) or len(height_range) != 2:
        raise ValueError("rough patch height range must be a [lo, hi] pair")
    lo, hi = float(height_range[0]), float(height_range[1])
    if not math.isfinite(lo) or not math.isfinite(hi) or lo < 0.0:
        raise ValueError("rough patch height range must contain finite values with lo >= 0")
    band = hi - lo
    if band < MIN_BAND_M - 1.0e-12:
        raise ValueError(
            f"rough patch band (hi - lo) must be >= {MIN_BAND_M:g} m; got [{lo}, {hi}]"
        )
    if band > MAX_BAND_M + 1.0e-12:
        raise ValueError(
            f"rough patch band (hi - lo) must be <= {MAX_BAND_M:g} m; got [{lo}, {hi}]"
        )
    ratio = zero_mean_half_band_m((lo, hi)) / VERTICAL_SCALE_M
    if abs(ratio - round(ratio)) > 1.0e-6:
        raise ValueError(
            f"rough patch band must be a multiple of {2 * VERTICAL_SCALE_M:g} m"
        )
    levels = int(round(ratio))
    if levels < 1:
        raise ValueError("rough patch quantizes to a flat surface")
    return lo, hi, levels


def _box_smooth(value: np.ndarray) -> np.ndarray:
    padded = np.pad(value, ((1, 1), (1, 1)), mode="edge")
    result = np.zeros_like(value, dtype=np.float64)
    for row in range(3):
        for col in range(3):
            result += padded[row : row + value.shape[0], col : col + value.shape[1]]
    return result / 9.0


def _smoothstep01(value: np.ndarray) -> np.ndarray:
    value = np.clip(value, 0.0, 1.0)
    return value * value * (3.0 - 2.0 * value)


def build_patch_height_field(
    height_range,
    flat_from_x_m: Optional[float],
    x_min_m: float,
    x_max_m: float,
    y_half_m: float,
    rng: np.random.Generator,
) -> np.ndarray:
    """Build a deterministic, correlated, zero-centred integer height field.

    The initial standing footprint has a smooth flat island and the last
    ``TABLE_BLEND_WIDTH_M`` before the table edge tapers continuously to zero.
    This removes the cell-to-cell steps from the old i.i.d. height field while
    preserving a useful +/-2 cm footwork perturbation away from reset.
    """

    _, _, levels = _validate_height_range(height_range)
    num_rows = int(
        math.ceil((float(x_max_m) - float(x_min_m)) / HORIZONTAL_SCALE_M - 1.0e-9)
    ) + 1
    num_cols = int(
        math.ceil(2.0 * float(y_half_m) / HORIZONTAL_SCALE_M - 1.0e-9)
    ) + 1
    if num_rows < 2 or num_cols < 2:
        raise ValueError("rough patch extents are degenerate")

    noise = rng.standard_normal((num_rows, num_cols))
    for _ in range(SMOOTHING_PASSES):
        noise = _box_smooth(noise)
    noise -= float(noise.mean())
    peak = float(np.max(np.abs(noise)))
    if peak <= 1.0e-12:
        raise RuntimeError("rough patch random field unexpectedly collapsed")
    noise /= peak

    x = float(x_min_m) + np.arange(num_rows) * HORIZONTAL_SCALE_M
    y = -float(y_half_m) + np.arange(num_cols) * HORIZONTAL_SCALE_M
    xx = x[:, None]
    yy = y[None, :]

    # Flat reset island: both feet start on the calibrated z=0 plane, then
    # encounter roughness only once the policy actually translates.
    radius = np.sqrt(np.square(xx) + np.square(yy))
    spawn_weight = _smoothstep01(
        (radius - SPAWN_FLAT_RADIUS_M)
        / (SPAWN_BLEND_RADIUS_M - SPAWN_FLAT_RADIUS_M)
    )
    weight = spawn_weight

    if flat_from_x_m is not None:
        table_weight = _smoothstep01(
            (float(flat_from_x_m) - xx) / TABLE_BLEND_WIDTH_M
        )
        weight = weight * table_weight

    continuous = noise * weight
    hf = np.rint(continuous * levels).astype(np.int16)
    np.clip(hf, -levels, levels, out=hf)

    # Re-assert hard geometric invariants after quantization.
    hf[radius <= SPAWN_FLAT_RADIUS_M + 1.0e-9] = 0
    if flat_from_x_m is not None:
        hf[x >= float(flat_from_x_m) - 1.0e-9, :] = 0
    return hf


def height_field_sha256(height_field: np.ndarray) -> str:
    array = np.ascontiguousarray(height_field.astype("<i2", copy=False))
    return hashlib.sha256(array.tobytes(order="C")).hexdigest()


def terrain_contract(height_range, near_x_m: Optional[float], seed: int) -> dict:
    """Return the canonical receipt for the exact collision height field."""

    seed = int(seed)
    if seed < 0:
        raise ValueError("rough patch seed must be a non-negative integer")
    x_min, x_max, y_half = patch_extents_m(near_x_m)
    hf = build_patch_height_field(
        height_range,
        near_x_m,
        x_min,
        x_max,
        y_half,
        np.random.default_rng(seed),
    )
    lo, hi, _ = _validate_height_range(height_range)
    return {
        "generator": GENERATOR_VERSION,
        "seed": seed,
        "authored_height_range_m": [lo, hi],
        "actual_half_band_m": zero_mean_half_band_m((lo, hi)),
        "horizontal_scale_m": HORIZONTAL_SCALE_M,
        "vertical_scale_m": VERTICAL_SCALE_M,
        "smoothing_passes": SMOOTHING_PASSES,
        "spawn_flat_radius_m": SPAWN_FLAT_RADIUS_M,
        "spawn_blend_radius_m": SPAWN_BLEND_RADIUS_M,
        "table_blend_width_m": TABLE_BLEND_WIDTH_M,
        "flat_from_x_m": None if near_x_m is None else float(near_x_m),
        "extents_m": [x_min, x_max, y_half],
        "shape": [int(hf.shape[0]), int(hf.shape[1])],
        "height_field_sha256": height_field_sha256(hf),
    }


def _spawn_rough_ground_patch(
    prim_path, cfg, translation=None, orientation=None
):
    import isaacsim.core.utils.prims as prim_utils
    import trimesh

    from isaaclab.terrains.height_field.utils import (
        convert_height_field_to_mesh,
    )
    from isaaclab.terrains.utils import create_prim_from_mesh

    hf = build_patch_height_field(
        cfg.height_range_m,
        cfg.flat_from_x_m,
        cfg.x_min_m,
        cfg.x_max_m,
        cfg.y_half_m,
        np.random.default_rng(int(cfg.seed)),
    )
    actual_sha = height_field_sha256(hf)
    if actual_sha != cfg.height_field_sha256:
        raise RuntimeError(
            "rough-ground collision receipt mismatch: "
            f"built={actual_sha}, expected={cfg.height_field_sha256}"
        )
    vertices, triangles = convert_height_field_to_mesh(
        hf, HORIZONTAL_SCALE_M, VERTICAL_SCALE_M, SLOPE_THRESHOLD
    )
    mesh = trimesh.Trimesh(vertices=vertices, faces=triangles)
    mesh.apply_translation((cfg.x_min_m, -cfg.y_half_m, 0.0))
    create_prim_from_mesh(
        prim_path,
        mesh,
        translation=translation,
        orientation=orientation,
        physics_material=cfg.physics_material,
        visual_material=cfg.visual_material,
    )
    return prim_utils.get_prim_at_path(prim_path)


def _isaac_spawner_bindings():
    cls = globals().get("RoughGroundPatchSpawnerCfg")
    if cls is not None:
        return cls

    from isaaclab.sim.spawners.spawner_cfg import SpawnerCfg
    from isaaclab.sim.utils import clone as _clone
    from isaaclab.utils import configclass

    wrapper = _clone(_spawn_rough_ground_patch)
    wrapper.__name__ = "spawn_rough_ground_patch"
    wrapper.__qualname__ = "spawn_rough_ground_patch"
    globals()["spawn_rough_ground_patch"] = wrapper

    @configclass
    class RoughGroundPatchSpawnerCfg(SpawnerCfg):
        func: Callable = wrapper
        height_range_m: Tuple[float, float] = (0.0, 0.0)
        flat_from_x_m: Optional[float] = None
        x_min_m: float = 0.0
        x_max_m: float = 0.0
        y_half_m: float = 0.0
        seed: int = 0
        height_field_sha256: str = ""
        generator_version: str = GENERATOR_VERSION
        physics_material: object = None
        visual_material: object = None

    RoughGroundPatchSpawnerCfg.__module__ = __name__
    RoughGroundPatchSpawnerCfg.__qualname__ = "RoughGroundPatchSpawnerCfg"
    globals()["RoughGroundPatchSpawnerCfg"] = RoughGroundPatchSpawnerCfg
    return RoughGroundPatchSpawnerCfg


def __getattr__(name):
    if name in ("RoughGroundPatchSpawnerCfg", "spawn_rough_ground_patch"):
        _isaac_spawner_bindings()
        return globals()[name]
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")


def attach_rough_ground_patch(env_cfg, height_range, seed: int):
    """Replace the plane importer with a receipt-bound cloned rough pad."""

    import isaaclab.sim as sim_utils
    from isaaclab.assets import AssetBaseCfg

    scene = getattr(env_cfg, "scene", None)
    terrain = None if scene is None else getattr(scene, "terrain", None)
    if terrain is None or getattr(terrain, "terrain_type", None) != "plane":
        raise RuntimeError(
            "attach_rough_ground_patch requires a plane TerrainImporter recipe"
        )
    if getattr(scene, ROUGH_PATCH_SCENE_ATTR, None) is not None:
        raise RuntimeError("rough-ground patch is already attached")
    material_src = getattr(terrain, "physics_material", None)
    if material_src is None:
        raise RuntimeError("rough-ground patch requires terrain physics material")

    rt = getattr(getattr(env_cfg, "commands", None), "racket_target", None)
    near_x = (
        float(rt.vb_table_near_x)
        if rt is not None and hasattr(rt, "vb_table_near_x")
        else None
    )
    contract = terrain_contract(height_range, near_x, int(seed))
    x_min, x_max, y_half = contract["extents_m"]
    half_band = float(contract["actual_half_band_m"])

    def _ground_material():
        return sim_utils.RigidBodyMaterialCfg(
            friction_combine_mode=getattr(
                material_src, "friction_combine_mode", "multiply"
            ),
            restitution_combine_mode=getattr(
                material_src, "restitution_combine_mode", "multiply"
            ),
            static_friction=float(material_src.static_friction),
            dynamic_friction=float(material_src.dynamic_friction),
            restitution=float(getattr(material_src, "restitution", 0.0)),
        )

    RoughGroundPatchSpawnerCfg = _isaac_spawner_bindings()
    setattr(
        scene,
        ROUGH_PATCH_SCENE_ATTR,
        AssetBaseCfg(
            prim_path="{ENV_REGEX_NS}/RoughGroundPatch",
            init_state=AssetBaseCfg.InitialStateCfg(pos=(0.0, 0.0, 0.0)),
            spawn=RoughGroundPatchSpawnerCfg(
                height_range_m=tuple(contract["authored_height_range_m"]),
                flat_from_x_m=near_x,
                x_min_m=x_min,
                x_max_m=x_max,
                y_half_m=y_half,
                seed=int(seed),
                height_field_sha256=contract["height_field_sha256"],
                generator_version=GENERATOR_VERSION,
                physics_material=_ground_material(),
                visual_material=sim_utils.PreviewSurfaceCfg(
                    diffuse_color=(0.31, 0.28, 0.25), roughness=0.9
                ),
            ),
        ),
    )

    drop = half_band + SAFETY_FLOOR_MARGIN_M
    num_envs = getattr(scene, "num_envs", None)
    spacing = getattr(scene, "env_spacing", None)
    if (
        isinstance(num_envs, int)
        and num_envs > 0
        and isinstance(spacing, (int, float))
    ):
        extent = (
            math.ceil(math.sqrt(num_envs)) * float(spacing)
            + 2.0 * X_BACK_M
            + 20.0
        )
    else:
        extent = 400.0
    setattr(
        scene,
        SAFETY_FLOOR_SCENE_ATTR,
        AssetBaseCfg(
            prim_path="/World/roughSafetyFloor",
            init_state=AssetBaseCfg.InitialStateCfg(pos=(0.0, 0.0, -drop)),
            spawn=sim_utils.GroundPlaneCfg(
                physics_material=_ground_material(), size=(extent, extent)
            ),
        ),
    )

    scene.terrain = None
    sim = getattr(env_cfg, "sim", None)
    if sim is not None and hasattr(sim, "physics_material"):
        sim.physics_material = _ground_material()
    setattr(env_cfg, "v17_terrain_contract", contract)

    return [
        (
            "scene.rough_ground_patch="
            f"{GENERATOR_VERSION}, seed={seed}, authored={tuple(height_range)}, "
            f"actual=±{half_band:.3f}m, sha256={contract['height_field_sha256']}"
        ),
        (
            "scene.rough_ground_patch.flat="
            f"spawn_r<={SPAWN_FLAT_RADIUS_M:g}m and table_x>={near_x}"
        ),
        f"scene.rough_safety_floor.z=-{drop:.3f}m",
        "scene.terrain=None (GridCloner origins preserved)",
    ]
