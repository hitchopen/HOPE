"""On-demand native snapshots; sensor traffic does not enter the Python worker.

The native receipt clock is the same host CLOCK_MONOTONIC. Command waits are
registered against a generation, so updates between snapshot and wait cannot
be lost. No polling timer fabricates receipts or replays sensor messages.
"""
from dataclasses import asdict
from pathlib import Path
import threading
import time

from hope_observer_core import BasePacket
from hope_runner_control_core import RunnerState
from hope_operations_core import CachedCall, RosComponent, WireSnapshot, telemetry_callback
from hope_player_positions import PlayerPositions


class NativePlayerPositions(PlayerPositions):
    status_threadsafe = True  # Native owner serializes its own cache accesses.

    def __init__(self, connection):
        self.connection = connection

    def status(self, now):
        # The native owner evaluates at query time so a newer packet arriving
        # during IPC cannot appear to have a negative age relative to `now`.
        return self.connection.query("player_status", now=now)


class NativeCondition(threading.Condition):
    def __init__(self, wire):
        super().__init__()
        self.wire = wire

    def wait(self, timeout=None):
        if not self._is_owned():
            raise RuntimeError("cannot wait on un-acquired lock")
        # Capture under the same condition as the command's snapshot check.
        generation = self.wire.generation
        saved = self._release_save()
        try:
            deadline = time.monotonic() + (3600. if timeout is None else max(0., timeout))
            try:
                sequence, future = self.wire.connection.request(
                    "wire_wait", name=self.wire.name, generation=generation,
                    timeout=3600. if timeout is None else max(0., timeout),
                    _deadline=min(deadline, time.monotonic()+5.))
            except TimeoutError:
                return False
            try:
                return future.result(timeout=max(0., deadline-time.monotonic()))
            except TimeoutError:
                return False  # Preserve the caller's existing absolute deadline.
            finally:
                with self.wire.connection.lock:
                    self.wire.connection.pending.pop(sequence, None)
        finally:
            self._acquire_restore(saved)


class NativeWire:
    def __init__(self, connection, name):
        self.connection, self.name = connection, name
        self.condition = NativeCondition(self)
        self.generation = -1
        self._cached = WireSnapshot()

    def snapshot(self):
        with self.condition:
            result = self.connection.query("wire_snapshot", name=self.name)
            if result["generation"] != self.generation:
                value = result["value"]
                if value is not None:
                    if self.name == "base":
                        value["source_time_ns"] = int(value["source_time_ns"])
                        value["position_xyz"] = tuple(value["position_xyz"])
                        value["quaternion_wxyz"] = tuple(value["quaternion_wxyz"])
                    value = (BasePacket if self.name == "base" else RunnerState)(**value)
                self._cached = WireSnapshot(value, result["received_ns"], result["error"], result["decode_error"])
                self.generation = result["generation"]
            return self._cached


class RuntimeRelay(RosComponent):
    """Keep the existing audited file/cache rules; publish one compact snapshot.

All sensor decoding, numeric calculation and ROS observer output is native.
The Python reference observer remains available for differential tests.
"""
    def __init__(self, node):
        super().__init__(node)
        from hope_observer import ObserverModule
        self._declare_parameter("session_id_path", "/tmp/hope_model21800_session_id")
        self._declare_parameter("runtime_root", "/tmp/hope_real")
        self._declare_parameter("planner_process_fragment", "hope_planner_cpp_node")
        self._session_id_path = Path(self.get_parameter("session_id_path").value)
        self._runtime_root = Path(self.get_parameter("runtime_root").value)
        self._planner_process_fragment = self.get_parameter("planner_process_fragment").value
        max_age = self.get_parameter("runtime_snapshot_max_age_s").value
        self._runtime_cache = CachedCall("hope-observer-files",
            lambda: ObserverModule._read_runtime_snapshot(self),
            ("", "", False, None, "RUNTIME SNAPSHOT PENDING"),
            max_age_s=max_age, refresh_min_interval_s=max_age / 2.)
        self.create_timer(self.get_parameter("publish_period_s").value, self._publish_runtime)

    def _runtime_snapshot(self):
        return self._runtime_cache.poll_snapshot()

    @telemetry_callback
    def _publish_runtime(self):
        sent_ns = time.monotonic_ns()
        snapshot = self._runtime_snapshot()
        session, attempt, planner, xhit, text = snapshot.value
        self.connection.send(dict(op="observer_runtime", transport_metrics=self.connection.diagnostics(), runtime=dict(
            fresh=snapshot.fresh, error=snapshot.error, age_s=snapshot.age_s,
            sent_ns=sent_ns, session_id=session, attempt=attempt,
            planner_alive=planner, x_hit=asdict(xhit) if xhit else None, x_hit_text=text)))

    def _telemetry_failure(self, _callback, exc):
        self.connection.send(dict(op="observer_runtime", runtime=dict(
            fresh=False, telemetry_failed=True, error=f"TELEMETRY FAULT: {type(exc).__name__}: {exc}")))

    def stop_accepting(self):
        self._runtime_cache.stop_refreshing()

    def close(self):
        self._runtime_cache.close()
