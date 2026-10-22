"""Check receiver ownership before changing a Laptop mocap connection."""
from pathlib import Path
import shlex
import sys


def read_receipt(path: Path) -> dict[str, str]:
    result = {}
    for line in path.read_text().splitlines():
        if "=" not in line:
            continue
        key, raw = line.split("=", 1)
        values = shlex.split(raw)
        if key in result or len(values) != 1:
            raise ValueError("Ambiguous mocap receiver receipt")
        result[key] = values[0]
    return result


def require_receiver_owner(path: Path, target: str) -> None:
    # Read the helper's receipt as data, never execute its shell assignments.
    if read_receipt(path).get("HOPE_LIFECYCLE_HDU_IP") != target:
        raise ValueError("Existing mocap receiver belongs to another or unknown robot; left untouched")


def retained_confirmations(path: Path, values: list[str]) -> str:
    keys = ["HDU_IP", "MOCAP_SOURCE", "MOTIVE_IP", "VRPN_IP", "VRPN_PORT", "VRPN_UNITS", "PANTS_PROFILE", "TABLE_SIDE"]
    try:
        receipt = read_receipt(path)
    except (OSError, ValueError):
        return "false false"
    if len(values) != len(keys) or any(receipt.get("HOPE_LIFECYCLE_" + key) != value for key, value in zip(keys, values)):
        return "false false"
    return " ".join("true" if receipt.get(key) == "true" else "false" for key in
                    ["HOPE_VRPN_PELVIS_ASSET_CONFIRMED", "HOPE_VRPN_WORLD_FRAME_CONFIRMED"])


if __name__ == "__main__":
    try:
        if sys.argv[1] == "--confirmations":
            print(retained_confirmations(Path(sys.argv[2]), sys.argv[3:]))
        else:
            require_receiver_owner(Path(sys.argv[1]), sys.argv[2])
    except (OSError, ValueError) as error:
        print(str(error), file=sys.stderr)
        raise SystemExit(1)
