#!/usr/bin/env python3
"""Apply the pinned display executor patch without overwriting field changes."""
import hashlib
from pathlib import Path
import subprocess
import sys

FILES = {'ros/src/foxglove_bridge/src/ros2_foxglove_bridge.cpp': ['f359d0b84e3c7ddfbaaf76b06c4b1f8808203a312dc37f7087b900427f8ce3a2', '01c4a8b446fc93f096d63917be37861a61795f4ae6d5dd03dcc087c5b7d98eb2'], 'ros/src/foxglove_bridge/include/foxglove_bridge/ros2_foxglove_bridge.hpp': ['91b2070198c66582b7bddf9677e3d740dab68fc07492ad2af88327094cc1cb05', 'eaa89a5dff0ce1c7ed379c3e93aaf170bac3f9e3095693d7227db20ebf91e1db']}
PATCH_SHA256 = "273babf6b5f8f62e8e8d9b1b052e817807954bc7bf14d52c0016eb5cd1ca80bd"

def main():
    source = Path(sys.argv[1])
    patch = Path(__file__).with_name("patches") / "foxglove_bridge-display-priority.patch"
    if hashlib.sha256(patch.read_bytes()).hexdigest() != PATCH_SHA256:
        raise SystemExit("Display patch checksum mismatch")
    actual = {p: hashlib.sha256((source / p).read_bytes()).hexdigest() for p in FILES}
    if all(actual[p] == FILES[p][1] for p in FILES):
        return
    if not all(actual[p] == FILES[p][0] for p in FILES):
        raise SystemExit("Unexpected bridge display source changes; preserve and inspect them")
    subprocess.run(["git", "-C", str(source), "apply", "--check", str(patch)], check=True)
    subprocess.run(["git", "-C", str(source), "apply", str(patch)], check=True)
    for p, (_, expected) in FILES.items():
        if hashlib.sha256((source / p).read_bytes()).hexdigest() != expected:
            raise SystemExit("Patched display source checksum mismatch: " + p)

if __name__ == "__main__":
    main()
