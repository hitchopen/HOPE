"""Foxglove's existing Record switch coordinates capture, never robot motion."""
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime, timezone
import threading
import time
import uuid


class DistributedRecorder:
    def __init__(self, exchange, config, roles=lambda: {"laptop", "hdu", "mdu"}):
        self.exchange = exchange
        self.config = config
        self.roles = roles
        self._condition = threading.Condition()
        self._desired = ""
        self._revision = 0
        self._thread = None
        self.state, self.output, self.error = "OFF", "", ""

    def status(self):
        with self._condition:
            return dict(state=self.state, output=self.output, error=self.error)

    def _wake(self):
        if self._thread is None:
            self._thread = threading.Thread(target=self._run, daemon=True, name="hope-record-switch")
            self._thread.start()
        self._condition.notify_all()

    def start(self):
        with self._condition:
            if self._desired:
                return self.status()
            self._desired = datetime.now(timezone.utc).strftime("record_%Y%m%dT%H%M%S_") + uuid.uuid4().hex[:12]
            self.output = self._desired
            self.state, self.error = "STARTING", ""
            self._revision += 1
            self._wake()
        return self.status()

    def stop(self, wait=True):
        with self._condition:
            self._desired = ""
            self.state, self.error = "STOPPING", ""
            self._revision += 1
            self._wake()
            if wait:
                self._condition.wait_for(lambda: self.state in {"OFF", "ERROR"}, timeout=35)
        return self.status()

    def _run(self):
        applied = -1
        with ThreadPoolExecutor(max_workers=3, thread_name_prefix="hope-record-peer") as pool:
            while True:
                with self._condition:
                    revision, desired = self._revision, self._desired
                request = {"op": "set", "segment": desired} if revision != applied else {"op": "status"}
                required = set(self.roles())
                futures = {role: pool.submit(self.exchange, role, self.config(), request)
                           for role in sorted(required)}
                results, errors = {}, []
                for role, future in futures.items():
                    try:
                        results[role] = future.result()
                    except Exception as exc:
                        errors.append(role + ": " + str(exc)[-180:])
                if not errors:
                    applied = revision
                with self._condition:
                    if revision != self._revision:
                        continue
                    self.error = "; ".join(errors)
                    workers = [w for result in results.values() for w in result["workers"]]
                    wrong = any(result["requested"] != desired for result in results.values())
                    if wrong:
                        applied = -1
                    failures = [w for w in workers if w["state"] == "ERROR"]
                    if errors or failures:
                        self.state = "ERROR"
                        self.error = self.error or str([w.get("errors") or w.get("csv") for w in failures])[-400:]
                    elif desired:
                        roles = {w["role"] for w in workers if w["state"] == "ON" and w["segment"] == desired}
                        self.state = "ON" if roles == required and not wrong else "STARTING"
                        if self.state == "STARTING":
                            self.error = "Waiting for capture: " + ", ".join(sorted(required - roles))
                    else:
                        self.state = "OFF" if not wrong and all(w["state"] in {"OFF", "CLOSED"} for w in workers) else "STOPPING"
                    self._condition.notify_all()
                    if self.state == "OFF":
                        self._condition.wait_for(lambda: revision != self._revision)
                    else:
                        self._condition.wait(timeout=1)
