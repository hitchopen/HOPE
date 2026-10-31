"""Process discovery for isolated operations tests and CPU measurements only."""
from pathlib import Path


def child_pids(pid, proc_root=Path("/proc")):
    """Return direct children even when CONFIG_CHECKPOINT_RESTORE is disabled.

    Some A3 kernels omit task/children. Their process stat files still contain
    PPIDs; scan process entries, not threads, so each child is counted once.
    Do not silently treat a missing parent or an unreadable children file as an
    empty child list: fault-injection tests must identify the intended worker.
    """
    root = Path(proc_root)
    try:
        children = (root / str(pid) / "task" / str(pid) / "children").read_text()
    except FileNotFoundError:
        if not (root / str(pid) / "stat").exists():
            raise ProcessLookupError(f"parent process {pid} no longer exists") from None
        result = []
        for entry in root.iterdir():
            if not entry.name.isdecimal():
                continue
            try:
                # comm can contain spaces and parentheses; PPID follows the
                # state field after the final closing parenthesis.
                fields = (entry / "stat").read_text().rsplit(")", 1)[1].split()
                if int(fields[1]) == pid:
                    result.append(int(entry.name))
            except (OSError, ValueError, IndexError):
                # A child can exit while /proc is being scanned.
                continue
        return sorted(result)
    return sorted(int(child) for child in children.split())
