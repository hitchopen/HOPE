"""Persistent field selections and an explicitly owned, opt-in MCAP recorder."""
from __future__ import annotations

from datetime import datetime, timezone
import json
import os
from pathlib import Path
import signal
import subprocess
import threading
import time

from hope_field_assets import atomic_write, asset_name, calibration_metadata
from hope_ball_models import PROFILES

MODES = {"OPTITRACK", "KERNEL"}


class ManagedRecorder:
    def __init__(self, root: Path, *, popen=subprocess.Popen):
        self.root = root
        self._popen = popen
        self._process = None
        self._log = None
        self._lock = threading.RLock()
        self._operation_lock = threading.Lock()
        self.state = "OFF"
        self.output = ""
        self.error = ""

    def status(self) -> dict:
        with self._lock:
            if self.state != "STOPPING" and self._process is not None and self._process.poll() is not None:
                code = self._process.returncode
                self._process = None
                if self._log:
                    self._log.close()
                    self._log = None
                self.state = "ERROR"
                self.error = f"recorder exited unexpectedly ({code}); inspect {self.output}.log"
            # ON means storage has actually been opened, not just Popen succeeded.
            if self.state == "STARTING" and Path(self.output).is_dir():
                if any(Path(self.output).glob("*.mcap")):
                    self.state = "ON"
            return dict(state=self.state, output=self.output, error=self.error)

    def start(self) -> dict:
        with self._operation_lock, self._lock:
            self.status()
            if self._process is not None:
                return self.status()
            self.root.mkdir(parents=True, exist_ok=True)
            self.output = str(self.root / datetime.now(timezone.utc).strftime("record_%Y%m%dT%H%M%S_%fZ"))
            self.error = ""
            self._log = open(self.output + ".log", "wb")
            try:
                self._process = self._popen(
                    ["/usr/local/bin/hope-pingpong-telemetry-record"],
                    env={**os.environ, "HOPE_RECORD_OUTPUT": self.output},
                    stdin=subprocess.DEVNULL, stdout=self._log, stderr=subprocess.STDOUT,
                    start_new_session=True,
                )
            except OSError:
                self._log.close()
                self._log = None
                self.state = "ERROR"
                raise
            self.state = "STARTING"
            return self.status()

    def stop(self) -> dict:
        with self._operation_lock:
            with self._lock:
                process = self._process
                self.state = "STOPPING" if process is not None else "OFF"
            if process is not None:
                try:
                    os.killpg(process.pid, signal.SIGINT)
                except ProcessLookupError:
                    pass
                try:
                    process.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    try:
                        os.killpg(process.pid, signal.SIGTERM)
                        process.wait(timeout=2)
                    except subprocess.TimeoutExpired:
                        os.killpg(process.pid, signal.SIGKILL)
                        process.wait(timeout=2)
                    except ProcessLookupError:
                        process.wait(timeout=2)
                    self.error = "recorder required forced stop; check the last MCAP before use"
            with self._lock:
                self._process = None
                if self._log:
                    self._log.close()
                    self._log = None
                self.state = "OFF"
                return self.status()


class FieldOperations:
    def __init__(self, root: Path, exchange, *, recorder=None):
        self.root = root
        self.exchange = exchange
        self.recorder = recorder or ManagedRecorder(Path("/agibot/data/bag/hope_pingpong_telemetry"))
        self._lock = threading.RLock()
        self._data = {"mode": "OPTITRACK", "csv": None, "calibration": None, "ball_model": None}
        self.ball_model_status = {"state": "CHECKING", "configured_model": None, "endpoints": {}}
        self._ball_checked_at = None
        self.calibration_sync = "NOT_CHECKED"
        self.error = ""
        try:
            saved = json.loads((root / "selection.json").read_text())
            if saved.get("mode") == "PURE_SERVE":
                saved["mode"] = "KERNEL"
            if saved.get("mode") not in MODES:
                raise ValueError("invalid saved operating mode")
            if saved.get("ball_model") is not None and saved["ball_model"] not in PROFILES:
                raise ValueError("invalid saved ball model")
            self._data.update({key: saved.get(key) for key in self._data})
        except FileNotFoundError:
            pass
        except (OSError, ValueError, AttributeError) as exc:
            self.error = f"cannot load saved field selections: {exc}"

    def _save(self) -> None:
        atomic_write(self.root / "selection.json", json.dumps(self._data).encode())

    def status(self) -> dict:
        with self._lock:
            ball_status = {**self.ball_model_status, "age_s":
                           None if self._ball_checked_at is None else time.monotonic() - self._ball_checked_at}
            return {**self._data, "ball_model_status": ball_status, "calibration_sync": self.calibration_sync,
                    "error": self.error, "recording": self.recorder.status()}

    def refresh_ball_models(self, config) -> dict:
        endpoints = {}
        for host in ("hdu", "laptop"):
            try:
                endpoints[host] = self.exchange(host, config, {"op": "inspect_ball_model"})
            except (OSError, ValueError, subprocess.SubprocessError) as exc:
                endpoints[host] = {"runtime": "UNAVAILABLE", "error": str(exc)[-400:]}
        models = [e.get("configured_model") for e in endpoints.values()]
        runtimes = [e.get("runtime") for e in endpoints.values()]
        same = models[0] is not None and models[0] == models[1]
        if "UNAVAILABLE" in runtimes:
            state = "UNAVAILABLE"
        elif not same or "MISMATCH" in runtimes:
            state = "MISMATCH"
        elif all(r == "VERIFIED" for r in runtimes):
            state = "LOADED"
        else:
            state = "CONFIGURED"
        with self._lock:
            if self._data["ball_model"] is not None and models != [self._data["ball_model"]] * 2:
                state = "MISMATCH"
            self.ball_model_status = {"state": state, "configured_model": models[0] if same else None,
                                      "endpoints": endpoints}
            self._ball_checked_at = time.monotonic()
        return self.status()

    def refresh(self, config) -> None:
        """Adopt an existing or explicitly recomputed receipt. Never capture or start ROS."""
        try:
            result = self.exchange("laptop", config, {"op": "read_calibration", "table_side": config.table_side})
            with self._lock:
                self._adopt(result, config.table_side)
                self.calibration_sync = "LOADED" if result["calibration"] else "MISSING"
        except (OSError, ValueError, subprocess.SubprocessError) as exc:
            with self._lock:
                self.calibration_sync = "UNAVAILABLE: " + str(exc)[-240:]

    def _adopt(self, result: dict, side: str) -> None:
        metadata = result["calibration"]
        if metadata:
            metadata = calibration_metadata(result["content"], metadata["name"], side)
            archive = self.root / "calibrations" / (metadata["sha256"] + ".json")
            if not archive.exists():
                atomic_write(archive, result["content"].encode())
        if metadata != self._data["calibration"]:
            self._data["calibration"] = metadata
            self._save()

    def command(self, request: dict, config, *, stopped: bool) -> dict:
        operation = request.get("op")
        if operation in {"set_mode", "upload_csv", "use_default_csv", "load_calibration", "set_ball_model"} and not stopped:
            raise ValueError("stop the Runner lifecycle before changing mode, CSV or calibration")
        if operation == "set_ball_model":
            model = request.get("model")
            if model not in PROFILES:
                raise ValueError("ball model must be berkeley, ppq002 or ppq004")
            previous = {host: self.exchange(host, config, {"op": "read_ball_model"})["model"]
                        for host in ("laptop", "hdu")}
            attempted = []
            try:
                for host in ("laptop", "hdu"):
                    attempted.append(host)  # Even a timed-out write may have committed.
                    result = self.exchange(host, config, {"op": "select_ball_model", "model": model})
                    if result.get("model") != model:
                        raise ValueError(f"{host} did not confirm ball model")
                with self._lock:
                    old_model = self._data["ball_model"]
                    self._data["ball_model"] = model
                    try:
                        self._save()
                    except Exception:
                        self._data["ball_model"] = old_model
                        raise
            except Exception as exc:
                failed = []
                for host in reversed(attempted):
                    try:
                        restored = self.exchange(host, config, {"op": "select_ball_model", "model": previous[host]})
                        if restored.get("model") != previous[host]:
                            raise ValueError("rollback was not confirmed")
                    except Exception:
                        failed.append(host)
                if failed:
                    raise ValueError(f"ball model update failed; rollback unconfirmed on {failed}: {exc}") from exc
                raise
        elif operation == "set_mode":
            request = {**request, "mode": "KERNEL" if request.get("mode") == "PURE_SERVE" else request.get("mode")}
            if request.get("mode") not in MODES:
                raise ValueError("mode must be OPTITRACK or KERNEL")
            with self._lock:
                self._data["mode"] = request["mode"]
                self._save()
        elif operation == "upload_csv":
            name = asset_name(request.get("name"), ".csv")
            result = self.exchange("mdu", config, request)
            with self._lock:
                self._data["csv"] = {**result, "name": name}
                self._save()
        elif operation == "use_default_csv":
            with self._lock:
                self._data["csv"] = None
                self._save()
        elif operation == "load_calibration":
            calibration_metadata(request["content"], request["name"], config.table_side)
            result = self.exchange("laptop", config, {**request, "table_side": config.table_side})
            with self._lock:
                self._adopt(result, config.table_side)
                self.calibration_sync = "LOADED"
        elif operation == "export_calibration":
            self.refresh(config)
            with self._lock:
                metadata = self._data["calibration"]
                if not metadata:
                    raise ValueError("no saved calibration to export")
                content = (self.root / "calibrations" / (metadata["sha256"] + ".json")).read_text()
                return {"name": metadata["name"], "content": content}
        elif operation == "refresh_calibration":
            self.refresh(config)
        elif operation == "record_on":
            self.recorder.start()
        elif operation == "record_off":
            self.recorder.stop()
        else:
            raise ValueError("unsupported field operation")
        if operation == "set_ball_model":
            return self.refresh_ball_models(config)
        return self.status()

    def runner_selection(self, config) -> tuple[str, str]:
        with self._lock:
            mode = self._data["mode"]
            csv = self._data["csv"]
        if mode != "KERNEL":
            # Also verify an absent selection: an interrupted/uncertain write
            # must not leave an unnoticed one-sided physics override.
            verified = [self.exchange(host, config, {"op": "verify_ball_model", "model": self._data["ball_model"]})
                        for host in ("laptop", "hdu")]
            models = [result.get("configured_model") for result in verified]
            if models[0] is None or models[0] != models[1]:
                raise ValueError("Planner and packetizer configured ball models do not match")
        digest = "DEFAULT"
        if csv:
            # Recheck existence and bytes before changing any hardware state.
            result = self.exchange("mdu", config, {"op": "verify_csv", "sha256": csv["sha256"]})
            digest = result["sha256"]
        return mode, digest
