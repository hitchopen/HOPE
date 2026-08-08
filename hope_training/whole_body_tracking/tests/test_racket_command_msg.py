"""ABI test for the RacketCommand ROS message (fields, order, no constants).

Parses ``hope_ws/src/hope_msgs/msg/RacketCommand.msg`` and checks it matches the CURRENT
public contract: a stamped strike command with geometry (position / velocity / normal),
timing (strike_time / time_to_strike), the predicted outgoing ball, and validity /
trajectory-quality flags. There are NO constants and NO task-identity or side fields —
ball identity travels on the flat wire (schema-2 flight/revision ids) and the swing side
is inferred outside the policy.

Run:  python tests/test_racket_command_msg.py   (or pytest)
"""

from __future__ import annotations

import os


def _repo_root() -> str:
    here = os.path.abspath(os.path.dirname(__file__))
    prev = None
    while here != prev:
        if os.path.isdir(os.path.join(here, "hope_ws")) and os.path.isdir(os.path.join(here, "hope_training")):
            return here
        prev, here = here, os.path.dirname(here)
    raise RuntimeError("could not locate the repo root (looked for hope_ws + hope_training)")


_MSG_PATH = os.path.join(_repo_root(), "hope_ws", "src", "hope_msgs", "msg", "RacketCommand.msg")


def _parse_msg(path: str):
    """Return (fields, constants): fields = [(type, name)], constants = {name: value}."""
    fields, constants = [], {}
    with open(path) as f:
        for raw in f:
            line = raw.split("#", 1)[0].strip()
            if not line:
                continue
            type_tok, rest = line.split(None, 1)
            if "=" in rest:  # constant: "TYPE NAME=VALUE"
                name, value = rest.split("=", 1)
                constants[name.strip()] = value.strip()
            else:
                fields.append((type_tok, rest.strip()))
    return fields, constants


def test_fields_exact_order():
    fields, _ = _parse_msg(_MSG_PATH)
    expected = [
        ("std_msgs/Header", "header"),
        ("geometry_msgs/Point", "position"),
        ("geometry_msgs/Vector3", "velocity"),
        ("geometry_msgs/Vector3", "normal"),
        ("float64", "strike_time"),
        ("float64", "time_to_strike"),
        ("geometry_msgs/Vector3", "ball_velocity_outgoing"),
        ("bool", "valid"),
        ("bool", "clears_net"),
        ("bool", "bypasses_net_posts"),
        ("int32", "predicted_bounces"),
    ]
    assert fields == expected, f"field ABI mismatch:\n got {fields}\n want {expected}"


def test_no_constants():
    _, constants = _parse_msg(_MSG_PATH)
    assert constants == {}, f"RacketCommand.msg must define no constants, found {constants}"


def test_removed_fields_absent():
    fields, constants = _parse_msg(_MSG_PATH)
    names = {n for _, n in fields} | set(constants)
    removed = {
        # Retired task-identity / side fields (identity now lives on the flat wire;
        # the side is inferred outside the policy and never observed).
        "task_id", "task_revision", "swing_side", "swing_sign",
        "FOREHAND", "BACKHAND",
        # Never part of the public contract.
        "reason", "failure", "status", "confidence", "diagnostics",
        "schema_version", "version",
    }
    leaked = removed & names
    assert not leaked, f"forbidden fields present: {leaked}"


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
    print(f"\n{len(tests) - failed}/{len(tests)} RacketCommand.msg ABI tests passed")
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
