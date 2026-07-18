"""Regression tests for the shared table-tennis geometry (ITTF dims / landmarks).

Pure Python (no Isaac). The geometry module uses a package-relative import of its physics-config
loader, so the modules are loaded under a synthetic package to resolve it without importing the full
Isaac Lab extension.

Run:  python tests/test_table_tennis_geometry.py
"""

from __future__ import annotations

import importlib.util
import math
import os
import sys
import types

_PKG_DIR = os.path.join(
    os.path.dirname(os.path.dirname(__file__)),
    "source", "whole_body_tracking", "whole_body_tracking", "tasks", "table_tennis",
)
_PKG = "ttpkg"


def _load_package():
    pkg = types.ModuleType(_PKG)
    pkg.__path__ = [_PKG_DIR]
    sys.modules[_PKG] = pkg
    for name in ("physics_config", "geometry"):
        spec = importlib.util.spec_from_file_location(f"{_PKG}.{name}", os.path.join(_PKG_DIR, f"{name}.py"))
        mod = importlib.util.module_from_spec(spec)
        sys.modules[f"{_PKG}.{name}"] = mod
        spec.loader.exec_module(mod)
    return sys.modules[f"{_PKG}.geometry"]


geometry = _load_package()


def test_table_dimensions_match_ittf():
    assert geometry.TABLE_LENGTH == 2.74
    assert geometry.TABLE_WIDTH == 1.525
    assert geometry.TABLE_HEIGHT == 0.76
    assert geometry.NET_X == 1.37
    assert geometry.NET_HEIGHT == 0.1525
    assert geometry.BALL_RADIUS == 0.02


def test_world_frame_landmarks():
    assert geometry.ORIGIN == (0.0, 0.0, 0.0)
    assert geometry.TABLE_CENTER == (1.37, -0.7625, 0.0)
    assert geometry.NET_CENTER == (1.37, -0.7625, 0.0)
    assert geometry.FLOOR_Z == -0.76


def test_table_top_face_at_surface():
    cz = geometry.table_top_center()[2]
    sz = geometry.table_top_size()[2]
    assert math.isclose(cz + sz / 2.0, 0.0, abs_tol=1e-9)


def test_net_spans_width_plus_overhang():
    _, ny, nz = geometry.net_size()
    assert math.isclose(ny, geometry.TABLE_WIDTH + 2 * geometry.NET_OVERHANG)
    assert math.isclose(geometry.net_center()[0], 1.37)


def test_robot_stands_on_near_half():
    assert geometry.P1_STAND_X < 0.0
    assert math.isclose(geometry.P1_STAND_Y, -geometry.TABLE_WIDTH / 2.0)


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
    print(f"\n{len(tests) - failed}/{len(tests)} geometry tests passed")
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
