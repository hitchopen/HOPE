#!/usr/bin/env python3
"""Fixed recording intent/status exchange; cannot start or signal robot control."""
import json
import os
from pathlib import Path
import re
import sys


def state_root():
    root = Path(os.environ.get("HOPE_RECORD_STATE_ROOT", str(Path.home() / ".local/state/hope-sim2real")))
    root.mkdir(parents=True, exist_ok=True)
    boot = Path("/proc/sys/kernel/random/boot_id").read_text().strip()
    identity = root / "boot_id"
    if not identity.exists() or identity.read_text().strip() != boot:
        (root / "recording.request").write_text("")
        identity.write_text(boot)
    return root


def process_identity(pid):
    try:
        return Path(f"/proc/{pid}/stat").read_text().rsplit(")", 1)[1].split()[19]
    except (OSError, IndexError):
        return None


def exchange(request):
    root = state_root()
    if request.get("op") == "set":
        segment = request.get("segment", "")
        if not isinstance(segment, str) or (segment and not re.fullmatch(r"record_[A-Za-z0-9_]{1,88}", segment)):
            raise ValueError("invalid recording segment")
        temp = root / f"request.{os.getpid()}.tmp"
        temp.write_text(segment + "\n")
        temp.replace(root / "recording.request")
    elif request.get("op") != "status":
        raise ValueError("unsupported recording operation")
    workers = []
    for path in (root / "workers").glob("*.json"):
        try:
            worker = json.loads(path.read_text())
            if worker["identity"] != process_identity(worker["pid"]):
                continue
            status = json.loads(Path(worker["status_path"]).read_text())
            workers.append({**worker, **status})
        except (OSError, ValueError, KeyError):
            continue
    return {"requested": (root / "recording.request").read_text().strip(), "workers": workers}


if __name__ == "__main__":
    print(json.dumps(exchange(json.load(sys.stdin))))
