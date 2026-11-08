"""Same-boot ownership receipt; evidence for explicit cleanup, never auto-resume.

Online recovery requires fresh authoritative Runner identity. The explicitly
confirmed offline procedure uses the saved session and existing STOP helpers
under the hardware interlock, and archives evidence only after cleanup succeeds.
"""
import json
import os
from pathlib import Path
import tempfile
import fcntl
import subprocess
import uuid

from hope_lifecycle_core import load_config, validate_session_id
from hope_runner_control_core import runner_session_fingerprint


class SessionReceipt:
    def __init__(self, path, boot_id):
        self.path, self.boot_id = Path(path), boot_id

    def record(self, session, mode, config, runner_boot=None):
        if not self.boot_id:
            raise ValueError("host boot identity unavailable")
        validate_session_id(session)
        if mode not in {"OPTITRACK", "KERNEL"}:
            raise ValueError("invalid session mode")
        document = dict(schema=1, host_boot=self.boot_id, session=session, mode=mode,
                        config=config.values(), revision=config.revision, runner_boot=runner_boot)
        fd, temporary = tempfile.mkstemp(prefix=".operations-session-", dir=self.path.parent)
        try:
            with os.fdopen(fd, "w", encoding="utf-8") as stream:
                json.dump(document, stream, sort_keys=True)
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary, self.path)
        finally:
            Path(temporary).unlink(missing_ok=True)

    def load(self, config):
        if not self.path.exists():
            return None
        if self.path.is_symlink() or self.path.stat().st_size > 16384:
            raise ValueError("invalid session receipt file")
        value = json.loads(self.path.read_text(encoding="utf-8"))
        if not isinstance(value, dict) or value.get("schema") != 1:
            raise ValueError("invalid session receipt schema")
        if not self.boot_id or value.get("host_boot") != self.boot_id:
            return None  # A reboot is not evidence that the previous Runner exists.
        validate_session_id(value.get("session"))
        if value.get("config") != config.values() or value.get("revision") != config.revision:
            raise ValueError("session receipt configuration differs; manual inspection required")
        if value.get("mode") not in {"OPTITRACK", "KERNEL"}:
            raise ValueError("invalid session receipt mode")
        boot = value.get("runner_boot")
        if boot is not None and (type(boot) is not int or not 0 < boot <= 1 << 52):
            raise ValueError("invalid Runner boot identity")
        return value

    @staticmethod
    def verify(receipt, snapshot, now_ns):
        state = snapshot.value
        if (state is None or snapshot.decode_error or not 0 <= now_ns-snapshot.received_ns < 1_500_000_000
                or state.session_fingerprint != runner_session_fingerprint(receipt["session"])
                or receipt["runner_boot"] is not None and state.boot_id != receipt["runner_boot"]):
            raise ValueError("RECOVERY_IDENTITY_UNVERIFIED: fresh matching Runner/session required; no cleanup sent")

    def clear(self):
        self.path.unlink(missing_ok=True)


def require_stopped_owners(run=subprocess.run, cgroup_root=Path("/sys/fs/cgroup")):
    """Read-only verification; never stop/mask services or infer absence from errors."""
    for unit in ("hope-lifecycle-supervisor.service", "hope-observer.service", "hope-command-proxy.service"):
        result = run(["systemctl", "show", unit, "--all",
                      "--property=LoadState,ActiveState,MainPID,ControlPID,ControlGroup"],
                     capture_output=True, text=True, check=False, timeout=5)
        state = dict(line.split("=", 1) for line in result.stdout.splitlines() if "=" in line)
        if ((result.returncode and state.get("LoadState") != "not-found")
                or state.get("LoadState") not in {"loaded", "masked", "not-found"}
                or state.get("ActiveState") not in {"inactive", "failed"}
                or state.get("MainPID") != "0" or state.get("ControlPID") != "0"
                or "ControlGroup" not in state):
            raise RuntimeError(f"recovery requires stopped operations owner: {unit}")
        group = state["ControlGroup"]
        if group:
            if not group.startswith("/") or ".." in Path(group).parts:
                raise RuntimeError(f"invalid owner cgroup: {unit}")
            events = cgroup_root / group.lstrip("/") / "cgroup.events"
            if events.exists() and "populated 0" not in events.read_text().splitlines():
                raise RuntimeError(f"owner cgroup still populated: {unit}")


def recover_stopped_session(config_path, expected_session, cleanup, *, confirmed=False,
                            event_callback=lambda _event: None,
                            require_stopped=require_stopped_owners,
                            boot_path=Path("/proc/sys/kernel/random/boot_id")):
    """Attended recovery without DDS/Runner evidence; never merely clears a latch.

    The operator explicitly authorizes the existing STOP sequence (including
    vendor PM restoration, which can enter Manual). Any failure retains the
    receipt, so partial startup/cleanup can be retried without rebooting.
    """
    if not confirmed:
        raise ValueError("confirm supported/stopped robot, operator at wired E-stop, and vendor PM restoration")
    validate_session_id(expected_session)
    config_path = Path(config_path)
    require_stopped()
    lock_path = config_path.parent / "hardware-operation.lock"
    # Never create a substitute lock or run as a second active owner.
    if lock_path.is_symlink():
        raise ValueError("hardware interlock must not be a symlink")
    try:
        lock = lock_path.open("r+")
    except FileNotFoundError:
        raise RuntimeError("existing hardware-operation.lock missing; inspect installation, do not fabricate it") from None
    with lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        require_stopped()
        config = load_config(config_path)
        receipt = SessionReceipt(config_path.parent / "operations_session.json", boot_path.read_text().strip())
        value = receipt.load(config)
        if value is None or value["session"] != expected_session:
            raise ValueError("same-boot saved session does not match --recover-session; no cleanup sent")
        original = receipt.path.read_bytes()
        cleanup(config, expected_session, event_callback, value["mode"])
        # Keep evidence on every failure, including an unexpected concurrent
        # receipt change. Do not erase another owner's/new session's receipt.
        require_stopped()
        if receipt.path.is_symlink() or receipt.path.read_bytes() != original:
            raise RuntimeError("session receipt changed during cleanup; not retired")
        archive = receipt.path.with_name(f"operations_session.recovered-{uuid.uuid4().hex}.json")
        os.replace(receipt.path, archive)
        return archive
