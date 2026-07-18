"""Unit tests for the shared no-spin success metric (pure NumPy; no Isaac / torch needed).

Run:  python tests/test_success_metric.py
"""

from __future__ import annotations

import importlib.util
import os
import sys

import numpy as np

_UTILS = os.path.join(
    os.path.dirname(os.path.dirname(__file__)),
    "source", "whole_body_tracking", "whole_body_tracking", "utils",
)


def _load(name: str, filename: str):
    path = os.path.join(_UTILS, filename)
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod  # dataclasses need the module registered to resolve annotations
    spec.loader.exec_module(mod)
    return mod


sm = _load("success_metric", "success_metric.py")


def _phys_table():
    return sm.BallPhysics(), sm.TableGeometry()


def test_geometry_opponent_half():
    _, table = _phys_table()
    # Opponent half is beyond the net (net_x = length/2) up to the far edge, within table width.
    assert table.net_x == table.length / 2.0
    assert table.on_opponent_half(table.net_x + 0.3, -table.width / 2.0)
    assert not table.on_opponent_half(table.net_x - 0.3, -table.width / 2.0)  # our half
    assert not table.on_opponent_half(table.length + 0.5, -table.width / 2.0)  # long


def test_successful_return():
    phys, table = _phys_table()
    # Racket reaches the target and sends the ball forward + up: clears the net, lands opponent half.
    target = np.array([0.10, -0.76, 0.35])
    achieved = target.copy()  # perfect contact
    vel = np.array([4.0, 0.0, 1.2])
    out = sm.evaluate_return(target, achieved, vel, phys, table)
    assert out.contacted and out.net_clear and out.on_opponent and out.success


def test_missed_contact_is_failure():
    phys, table = _phys_table()
    target = np.array([0.10, -0.76, 0.35])
    achieved = target + np.array([0.3, 0.0, 0.0])  # 30 cm away -> no contact (radius 0.10)
    vel = np.array([4.0, 0.0, 1.2])
    out = sm.evaluate_return(target, achieved, vel, phys, table)
    assert not out.contacted and not out.success


def test_short_ball_fails_net_and_bounds():
    phys, table = _phys_table()
    target = np.array([0.10, -0.76, 0.20])
    vel = np.array([1.0, 0.0, 0.2])  # too slow / flat: bounces on our half before the net
    out = sm.evaluate_return(target, target.copy(), vel, phys, table)
    assert out.contacted and not out.net_clear and not out.on_opponent and not out.success


def test_success_rate_accumulation():
    phys, table = _phys_table()
    acc = sm.SuccessRate()
    good = sm.evaluate_return(np.array([0.1, -0.76, 0.35]), np.array([0.1, -0.76, 0.35]),
                              np.array([4.0, 0.0, 1.2]), phys, table)
    bad = sm.evaluate_return(np.array([0.1, -0.76, 0.20]), np.array([0.1, -0.76, 0.20]),
                             np.array([1.0, 0.0, 0.2]), phys, table)
    acc.add(good)
    acc.add(bad)
    assert acc.attempts == 2 and acc.successes == 1
    assert abs(acc.value - 0.5) < 1e-9
    assert acc.as_dict() == {"success_rate": 0.5}


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
    print(f"\n{len(tests) - failed}/{len(tests)} success-metric tests passed")
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
