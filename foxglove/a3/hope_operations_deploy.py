#!/usr/bin/env python3
"""Stage-only native operations installation; explicit, stopped activation/rollback.

No command stops, starts, enables, or disables a service. Staging never writes
active binaries, Python modules, units, drop-ins, or systemd's loaded state.
"""
import argparse
from contextlib import contextmanager
import fcntl
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import tempfile

MODULES = (
    "hope_operations_transport.py", "hope_operations_native_state.py", "hope_operations_core.py",
    "hope_operations_runtime.py", "hope_telemetry.py", "hope_command_proxy.py", "hope_command_core.py",
    "hope_lifecycle_supervisor.py", "hope_lifecycle_core.py", "hope_observer.py", "hope_observer_core.py",
    "hope_runner_control_core.py", "hope_display_publishers.py", "hope_field_operations.py",
    "hope_field_assets.py", "hope_distributed_recording.py", "hope_ball_models.py", "hope_policy_models.py",
    "hope_monitor_core.py", "hope_elink_health.py", "hope_elink_fault.py", "hope_elink_clock.py",
    "hope_elink_telemetry.py", "hope_elink_telemetry_ros.py", "hope_player_positions.py",
    "hope_time_calibration_core.py",
    "hope_operations_recovery.py",
)
FILES = {"hope-operations": ("/usr/local/libexec/hope-operations", 0o755),
         "hope_operations_worker.py": ("/usr/local/libexec/hope_operations_worker.py", 0o755),
         **{name: ("/usr/local/lib/hope-foxglove/" + name, 0o644) for name in MODULES},
         "hope-lifecycle-supervisor.service": ("/etc/systemd/system/hope-lifecycle-supervisor.service", 0o644)}
UNITS = ("hope-lifecycle-supervisor.service", "hope-observer.service", "hope-command-proxy.service")


class Deployment:
    # root/run injection is exclusively for filesystem-only regression tests;
    # the CLI always uses / and real systemctl, and requires effective uid 0.
    def __init__(self, root=Path("/"), run=subprocess.run):
        self.root, self.run = Path(root), run

    def path(self, absolute):
        return self.root / absolute.lstrip("/")

    def stage(self, source, binary, unit):
        parent = self.path("/usr/local/lib/hope-operations/staged")
        parent.mkdir(parents=True, exist_ok=True)
        bundle = Path(tempfile.mkdtemp(prefix="candidate-", dir=parent))
        records = {}
        for name, (_, mode) in FILES.items():
            origin = binary if name == "hope-operations" else unit if name.endswith(".service") else source / name
            shutil.copyfile(origin, bundle / name)
            (bundle / name).chmod(mode)
            records[name] = hashlib.sha256((bundle / name).read_bytes()).hexdigest()
        # Written last: a partial/failed stage cannot be activated.
        (bundle / "manifest.json").write_text(json.dumps(records, sort_keys=True) + "\n")
        return bundle

    def validated_directory(self, path, parent):
        path = Path(path)
        if path.is_symlink() or path.resolve().parent != self.path(parent).resolve():
            raise ValueError("not a managed operations deployment directory")
        if not path.is_dir() or path.stat().st_uid != os.geteuid() or path.stat().st_mode & 0o022:
            raise ValueError("deployment directory must be owned by installer and not writable by others")
        return path

    def verify(self, bundle):
        bundle = self.validated_directory(bundle, "/usr/local/lib/hope-operations/staged")
        records = json.loads((bundle / "manifest.json").read_text())
        if set(records) != set(FILES):
            raise ValueError("incomplete or mismatched operations bundle")
        for name, expected in records.items():
            source = bundle / name
            if source.is_symlink() or not source.is_file() or hashlib.sha256(source.read_bytes()).hexdigest() != expected:
                raise ValueError("operations bundle verification failed: " + name)
        return bundle

    @staticmethod
    def check_legacy_layout(unit, state):
        # Disabled real services are permitted by the migration contract, but
        # inactive/enabled is not enough: they would return at the next boot.
        if state.get("UnitFileState") in {"disabled", "masked"}:
            return
        if state.get("UnitFileState") == "masked-runtime":
            raise RuntimeError(
                f"activation refused: {unit} has only a runtime mask, which disappears on reboot. "
                "Inspect and persistently mask/disable the legacy unit in the attended stopped "
                "window, reload systemd, then retry. Activation does not change masks automatically.")
        # Read systemd's effective commands (including drop-ins), not just the
        # repository unit text. A pending reload cannot prove the next boot's
        # layout. Reject shell wrappers, multiple commands and executable hooks.
        hooks = ("ExecCondition", "ExecStartPre", "ExecStartPost", "ExecStop", "ExecStopPost")
        true_stub = (state.get("Type") == "oneshot"
                     and state.get("NeedDaemonReload") == "no"
                     and all(state.get(key) == "" for key in hooks)
                     and re.fullmatch(
                         r"\{\s*path=/usr/bin/true\s*;\s*argv\[\]=/usr/bin/true\s*;[^{}]*\}",
                         state.get("ExecStart", "").strip()) is not None)
        if not true_stub:
            raise RuntimeError(
                f"activation refused: {unit} is a real or unverified split-layout service "
                f"(UnitFileState={state.get('UnitFileState', 'unknown')}). Stopping it is not enough: "
                "it could duplicate the native owner's ROS services/topics on reboot. "
                "In the attended stopped window, inspect its effective unit/drop-ins, then disable "
                "or persistently mask the legacy unit, or install the /usr/bin/true compatibility "
                "stub and reload systemd. "
                "Activation does not disable or rewrite legacy units automatically.")

    def check_stopped(self, *, for_activation=False):
        for unit in UNITS:
            result = self.run(["systemctl", "show", unit, "--all",
                              "--property=LoadState,ActiveState,MainPID,ControlPID,UnitFileState,Type,"
                              "NeedDaemonReload,ExecStart,ExecCondition,ExecStartPre,ExecStartPost,ExecStop,ExecStopPost"],
                              capture_output=True, text=True, check=False)
            state = dict(line.split("=", 1) for line in result.stdout.splitlines() if "=" in line)
            if state.get("LoadState") == "not-found":
                continue  # Current consolidated installs can omit the old shim units.
            if (result.returncode or state.get("LoadState") not in {"loaded", "masked"}
                    or state.get("ActiveState") not in {"inactive", "failed"}
                    or state.get("MainPID") != "0" or state.get("ControlPID") != "0"):
                raise RuntimeError("attended activation requires stopped owner: " + unit)
            if for_activation and unit == UNITS[0] and (
                    state.get("LoadState") == "masked"
                    or state.get("UnitFileState") in {"masked", "masked-runtime"}):
                raise RuntimeError(
                    "activation refused: canonical supervisor is masked; activation will not "
                    "overwrite or remove its mask. Inspect/unmask it explicitly in the attended "
                    "stopped window before retrying.")
            if state.get("LoadState") == "masked" and state.get("NeedDaemonReload") != "no":
                raise RuntimeError("masked owner has unverified pending unit changes; reload systemd: " + unit)
            if for_activation and unit != UNITS[0]:
                self.check_legacy_layout(unit, state)

    @contextmanager
    def stopped(self, confirmed, *, for_activation=False):
        if not confirmed:
            raise ValueError("confirm supported/stopped robot and operator at verified wired E-stop")
        self.check_stopped(for_activation=for_activation)
        # Existing deployment's lock also excludes clock calibration and any
        # other legitimate hardware owner. Do not create/change its ownership.
        lock_path = self.path("/var/lib/hope-lifecycle/hardware-operation.lock")
        try:
            lock = lock_path.open("r+")
        except FileNotFoundError:
            raise RuntimeError(
                f"missing existing hardware interlock: {lock_path}. This activation/rollback tool "
                "migrates an initialized HOPE deployment; the supervisor may never have run. "
                "Complete baseline HOPE supervisor initialization in an attended supported/stopped "
                "maintenance window, stop the owner, then retry. The tool will not create or bypass "
                "the hardware-operation.lock interlock.") from None
        with lock:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            self.check_stopped(for_activation=for_activation)
            yield

    def replace(self, source, destination, mode):
        destination.parent.mkdir(parents=True, exist_ok=True)
        fd, temp = tempfile.mkstemp(prefix=".hope-operations-", dir=destination.parent)
        os.close(fd)
        temp = Path(temp)
        try:
            temp.unlink()
            shutil.copy2(source, temp, follow_symlinks=False)
            if not temp.is_symlink():
                temp.chmod(mode)
            os.replace(temp, destination)
        finally:
            temp.unlink(missing_ok=True)

    def backup(self):
        parent = self.path("/var/backups/hope-operations")
        parent.mkdir(parents=True, exist_ok=True)
        backup = Path(tempfile.mkdtemp(prefix="activation-", dir=parent))
        present = {}
        for name, (absolute, _) in FILES.items():
            target = self.path(absolute)
            if target.exists() or target.is_symlink():
                shutil.copy2(target, backup / name, follow_symlinks=False)
                present[name] = target.lstat().st_mode & 0o777
            else:
                present[name] = None
        (backup / "manifest.json").write_text(json.dumps(present, sort_keys=True) + "\n")
        return backup

    def restore(self, backup):
        records = json.loads((backup / "manifest.json").read_text())
        if set(records) != set(FILES):
            raise ValueError("incomplete operations rollback manifest")
        # Validate the complete backup before restoring anything.
        for name, mode in records.items():
            if mode is not None and not ((backup / name).exists() or (backup / name).is_symlink()):
                raise ValueError("missing operations rollback file: " + name)
        for name, (absolute, _) in FILES.items():
            target = self.path(absolute)
            if records[name] is None:
                target.unlink(missing_ok=True)  # Only exact allowlisted candidate targets absent before activation.
            else:
                self.replace(backup / name, target, records[name])
        self.run(["systemctl", "daemon-reload"], check=True)

    def activate(self, bundle, confirmed=False):
        bundle = self.verify(bundle)
        with self.stopped(confirmed, for_activation=True):
            backup = self.backup()
            print("Rollback bundle: " + str(backup), flush=True)
            try:
                for name, (absolute, mode) in FILES.items():
                    self.replace(bundle / name, self.path(absolute), mode)
                self.run(["systemctl", "daemon-reload"], check=True)
            except Exception:
                self.restore(backup)
                raise
        return backup

    def rollback(self, backup, confirmed=False):
        backup = self.validated_directory(backup, "/var/backups/hope-operations")
        with self.stopped(confirmed):
            self.restore(backup)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="action", required=True)
    stage = sub.add_parser("stage")
    for option in ("source", "binary", "unit"):
        stage.add_argument("--" + option, type=Path, required=True)
    for action in ("activate", "rollback"):
        command = sub.add_parser(action)
        command.add_argument("bundle", type=Path)
        command.add_argument("--confirm-supported-stopped-wired-estop", action="store_true")
    args = parser.parse_args()
    if os.geteuid() != 0:
        parser.error("use sudo; deployment files must be root-owned")
    deployment = Deployment()
    try:
        if args.action == "stage":
            print(deployment.stage(args.source, args.binary, args.unit))
            print("Staged only: active files, systemd state and boot behavior are unchanged.")
        else:
            getattr(deployment, args.action)(args.bundle, args.confirm_supported_stopped_wired_estop)
            print("Files updated with rollback protection; no service was started or enabled.")
    except (OSError, ValueError, RuntimeError, subprocess.CalledProcessError) as error:
        parser.exit(1, f"operations {args.action} failed: {error}\n")


if __name__ == "__main__":
    main()
