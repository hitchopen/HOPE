"""Read-only legacy ELink records and evidence-based startup log inspection.

Earlier candidates wrote permanent admission latches. Preserve those records as
diagnostic evidence only: no new records, recovery requirement, or command veto.
"""
from __future__ import annotations

import json
from pathlib import Path


class PersistentElinkFault:
    """Compatibility reader for old candidate files; never mutates them."""

    def __init__(self, path: Path):
        self.path = Path(path)
        self.active_path = self.path.with_name(self.path.name + ".active")
        self.reason = ""
        try:
            if self.path.is_symlink():
                raise ValueError("symlink")
            if self.path.exists():
                with self.path.open() as stream:
                    raw = stream.read(4097)
                if len(raw) > 4096:
                    raise ValueError("oversized fault")
                data = json.loads(raw)
                if data.get("schema") != 1 or not isinstance(data.get("reason"), str) or not data["reason"]:
                    raise ValueError("invalid fault")
                self.reason = data["reason"][:256]
            elif self.active_path.exists() or self.active_path.is_symlink():
                # Legacy evidence, not a requirement to clear files or recover.
                self.reason = "ELINK_RECOVERY_REQUIRED_AFTER_SUPERVISOR_RESTART"
        except (OSError, ValueError, TypeError, AttributeError):
            self.reason = "ELINK_FAULT_RECORD_UNREADABLE"

def startup_log_fault(text: str) -> str:
    """A later reset/crash cannot be hidden behind earlier readiness markers."""
    ready = False
    for line in text.splitlines():
        if "Enter ElinkKeepAliveLoop" in line:
            ready = True
        if ready and any(token in line for token in (
            "is not alive", "Invalid session", "DestoryElink",
            "destory all elink node", "SIGSEGV", "Segmentation fault",
        )):
            return "ELINK_FAILURE_AFTER_READY"
    return ""
