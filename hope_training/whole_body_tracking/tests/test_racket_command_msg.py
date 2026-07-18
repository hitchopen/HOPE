"""ABI test for the RacketCommand ROS message (fields, order, constants).

Parses ``hope_ws/src/hope_msgs/msg/RacketCommand.msg`` and checks it matches the public contract
(no spin / normal / outgoing-ball / validity / status fields, no version tag).

Run:  python tests/test_racket_command_msg.py
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
        ("uint64", "task_id"),
        ("uint32", "task_revision"),
        ("int8", "swing_side"),
        ("geometry_msgs/Point", "position"),
        ("geometry_msgs/Vector3", "velocity"),
        ("float64", "time_to_strike"),
    ]
    assert fields == expected, f"field ABI mismatch:\n got {fields}\n want {expected}"


def test_swing_side_constants():
    _, constants = _parse_msg(_MSG_PATH)
    assert constants.get("FOREHAND") == "1"
    assert constants.get("BACKHAND") == "-1"


def test_removed_fields_absent():
    fields, constants = _parse_msg(_MSG_PATH)
    names = {n for _, n in fields} | set(constants)
    removed = {
        "normal", "strike_time", "ball_velocity_outgoing", "valid", "clears_net",
        "bypasses_net_posts", "predicted_bounces", "reason", "failure", "status",
        "confidence", "diagnostics", "schema_version", "version",
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
