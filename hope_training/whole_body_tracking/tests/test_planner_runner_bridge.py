"""Planner -> runner control-path tests (the flat /racket/command_flat wire).

The live planner publishes its racket command as a ``std_msgs/Float64MultiArray``
with a schema tag at element [0]; the runner-side bridge decodes it with the pure
module-level function ``parse_flat_racket_command`` (no rclpy, no hope_msgs). These
tests exercise that pure parsing layer:

  * schema-1 (>= 11 doubles) and schema-2 (19 doubles) field mapping;
  * ``valid == 0`` packets are skipped (None);
  * short arrays and unknown schema tags are rejected (None);
  * extra trailing fields are ignored;
  * schema-2 flight/revision ids map onto the runner's task identity.

Run:  python tests/test_planner_runner_bridge.py   (or pytest)
"""

from __future__ import annotations

import math
import os
import sys

import numpy as np

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_REPO = os.path.dirname(os.path.dirname(_ROOT))
_REFERENCE_DIR = os.path.join(_REPO, "a3_deploy", "a3_deploy_example", "reference")
sys.path.insert(0, _REFERENCE_DIR)

from a3_deploy_onnx_ref_pingpong.racket_command import BACKHAND, FOREHAND  # noqa: E402
from a3_deploy_onnx_ref_pingpong.ros_command_source import (  # noqa: E402
    DEFAULT_COMMAND_TOPIC,
    parse_flat_racket_command,
)


def _schema1(valid=1.0, swing=1.0, pos=(0.5, -0.3, 0.9), vel=(1.5, 0.8, 0.4),
             tts=0.42, strike_time=123.4, extra=()):
    return [1.0, valid, swing, *pos, *vel, tts, strike_time, *extra]


def _schema2(valid=1.0, swing=-1.0, pos=(0.58, -0.25, 1.0), vel=(2.0, -0.7, 0.4),
             tts=0.35, strike_time=1.754e9 + 0.35, frame_code=0.0,
             producer_sec=1.754e9, producer_nsec=125000000.0,
             command_seq=1234.0, flight_id=7.0, revision_id=3.0,
             estimator_samples=40.0, estimator_span=0.5):
    return [
        2.0, valid, swing, *pos, *vel, tts, strike_time, frame_code,
        producer_sec, producer_nsec, command_seq, flight_id, revision_id,
        estimator_samples, estimator_span,
    ]


def test_default_topic_is_the_flat_topic():
    assert DEFAULT_COMMAND_TOPIC == "/racket/command_flat"


def test_schema1_field_mapping():
    cmd = parse_flat_racket_command(_schema1(swing=-1.0))
    assert cmd is not None
    assert cmd.swing_sign == BACKHAND
    np.testing.assert_allclose(cmd.position, [0.5, -0.3, 0.9])
    np.testing.assert_allclose(cmd.velocity, [1.5, 0.8, 0.4])
    assert cmd.time_to_strike == 0.42
    # Schema 1 carries no ball identity: the subscriber synthesizes it later.
    assert cmd.task_id == 0 and cmd.task_revision == 0


def test_schema1_side_normalization():
    assert parse_flat_racket_command(_schema1(swing=1.0)).swing_sign == FOREHAND
    assert parse_flat_racket_command(_schema1(swing=-1.0)).swing_sign == BACKHAND


def test_schema1_ignores_trailing_extras():
    # e.g. the optional [11]=frame_code the C++ wire allows.
    cmd = parse_flat_racket_command(_schema1(extra=(0.0,)))
    assert cmd is not None and cmd.time_to_strike == 0.42


def test_schema2_field_mapping_and_identity():
    cmd = parse_flat_racket_command(_schema2())
    assert cmd is not None
    assert cmd.swing_sign == BACKHAND
    np.testing.assert_allclose(cmd.position, [0.58, -0.25, 1.0])
    np.testing.assert_allclose(cmd.velocity, [2.0, -0.7, 0.4])
    assert cmd.time_to_strike == 0.35
    # flight_id / revision_id map onto the runner's task identity.
    assert cmd.task_id == 7
    assert cmd.task_revision == 3


def test_valid_zero_is_skipped():
    assert parse_flat_racket_command(_schema1(valid=0.0)) is None
    assert parse_flat_racket_command(_schema2(valid=0.0)) is None


def test_short_arrays_rejected():
    good = _schema1()
    assert parse_flat_racket_command(good[:10]) is None    # schema-1 needs >= 11
    assert parse_flat_racket_command(_schema2()[:18]) is None  # schema-2 needs 19
    assert parse_flat_racket_command([]) is None
    assert parse_flat_racket_command([1.0]) is None


def test_unknown_schema_rejected():
    bad = _schema1()
    bad[0] = 3.0
    assert parse_flat_racket_command(bad) is None
    bad[0] = 0.0
    assert parse_flat_racket_command(bad) is None


def test_non_finite_required_fields_rejected():
    nan = math.nan
    assert parse_flat_racket_command(_schema1(tts=nan)) is None
    assert parse_flat_racket_command(_schema1(pos=(nan, 0.0, 0.0))) is None
    assert parse_flat_racket_command(_schema2(flight_id=nan)) is None


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
    print(f"\n{len(tests) - failed}/{len(tests)} planner-runner bridge tests passed")
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
