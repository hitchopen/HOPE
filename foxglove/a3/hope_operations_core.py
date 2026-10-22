"""Shared state and bounded workers for the single operations ROS node.

The state/worker classes have no ROS dependency, so their concurrency guarantees
can be exercised without DDS or a robot. RosComponent owns no ROS node.
"""
from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor
from collections import Counter
from contextlib import nullcontext
from dataclasses import dataclass
from functools import wraps
from types import MappingProxyType
import threading
import time

from hope_observer_core import DecodeError, decode_base_packet
from hope_runner_control_core import decode_runner_state


@dataclass(frozen=True)
class WireSnapshot:
    value: object = None
    received_ns: int = 0
    error: str = ""
    decode_error: str = ""


class SharedWire:
    def __init__(self, decode, label):
        self.condition = threading.Condition()
        self._decode = decode
        self._label = label
        self._snapshot = WireSnapshot(error=f"NO {label}")

    def update(self, data):
        # Decode inside the lock: two callbacks must not publish an older
        # decode after a newer packet has already notified command waiters.
        with self.condition:
            received = time.monotonic_ns()
            try:
                value = self._decode(data)
                error = "" if getattr(value, "valid", True) else value.reason
            except DecodeError as exc:
                diagnostic = f"MALFORMED {self._label}: {exc}"
                # Match the command proxy's existing last-good semantics:
                # malformed traffic is diagnostic, never a fresh receipt.
                # Telemetry can separately invalidate its displayed packet.
                prior = self._snapshot
                self._snapshot = WireSnapshot(prior.value, prior.received_ns,
                                              diagnostic, diagnostic)
            else:
                # Successfully decoded invalid packets explicitly invalidate
                # their source; only decode failures retain the prior value.
                self._snapshot = WireSnapshot(value, received, error)
            self.condition.notify_all()

    def snapshot(self):
        with self.condition:
            return self._snapshot


def _freeze_field_status(value):
    """Own an immutable copy of the JSON-shaped status shared in this process."""
    if isinstance(value, dict):
        return MappingProxyType({key: _freeze_field_status(item) for key, item in value.items()})
    if isinstance(value, (list, tuple)):
        return tuple(_freeze_field_status(item) for item in value)
    return value


@dataclass(frozen=True)
class FieldStatusSnapshot:
    wire: str
    parsed: object


class OperationsState:
    def __init__(self):
        self.admission = WorkerAdmission()
        self.runner = SharedWire(decode_runner_state, "RUNNER STATE")
        self.base = SharedWire(decode_base_packet, "BASE PACKET")
        self._field_lock = threading.Lock()
        self._field_status = None

    def set_field_status(self, wire, parsed):
        snapshot = FieldStatusSnapshot(wire, _freeze_field_status(parsed))
        with self._field_lock:
            self._field_status = snapshot

    def field_status(self):
        with self._field_lock:
            return self._field_status


class WorkerAdmission:
    """Atomically stop admission only after all state owners are quiescent."""
    def __init__(self):
        self.lock = threading.Lock()
        self._state = "OPEN"

    @property
    def closed(self):
        return self._state != "OPEN"

    def seal_if(self, predicate):
        with self.lock:
            if self._state != "OPEN":
                return False
            if not predicate():
                return False
            self._state = "DRAINING"
            return True

    def seal(self):
        with self.lock:
            if self._state == "OPEN":
                self._state = "DRAINING"

    def close(self):
        with self.lock:
            self._state = "CLOSED"


@dataclass(frozen=True)
class WorkerSubmission:
    future: object = None
    rejection: str = ""


class RpcDrainTracker:
    """Track pending service coroutines and their callback sources."""
    def __init__(self):
        self._lock = threading.Lock()
        self._pending = 0
        self._sources = Counter()

    def enter(self, source):
        with self._lock:
            self._pending += 1
            self._sources[source] += 1

    def leave(self, source):
        with self._lock:
            self._pending -= 1
            self._sources[source] -= 1
            if not self._sources[source]:
                del self._sources[source]

    @property
    def pending(self):
        with self._lock:
            return self._pending

    def pending_sources(self):
        with self._lock:
            return dict(self._sources)


class BoundedWorker:
    """Reject excess work immediately; never build an unbounded request queue."""
    def __init__(self, name, capacity=1, admission=None):
        self._pool = ThreadPoolExecutor(max_workers=capacity, thread_name_prefix=name)
        self._lock = threading.Lock()
        self._capacity = capacity
        self._pending = 0
        self._state = "OPEN"
        self._admission = admission

    @property
    def busy(self):
        with self._lock:
            return self._pending != 0

    def submit(self, callback, *args, **kwargs):
        return self.try_submit(callback, *args, **kwargs).future

    def try_submit(self, callback, *args, **kwargs):
        # Return the reason captured under the admission locks. Reading state
        # after a rejected submit could misreport a shutdown race as BUSY.
        with self._admission.lock if self._admission else nullcontext():
            if self._admission and self._admission.closed:
                return WorkerSubmission(rejection=self._rejection(self._admission._state))
            with self._lock:
                if self._state != "OPEN":
                    return WorkerSubmission(rejection=self._rejection(self._state))
                if self._pending >= self._capacity:
                    return WorkerSubmission(rejection="BUSY: another explicit operator action is already in progress")
                self._pending += 1
                try:
                    future = self._pool.submit(callback, *args, **kwargs)
                except BaseException:
                    self._pending -= 1
                    raise
        future.add_done_callback(self._finished)
        return WorkerSubmission(future=future)

    @staticmethod
    def _rejection(state):
        if state == "DRAINING":
            return "DRAINING: operations is stopping; new requests are not accepted"
        return "CLOSED: operations worker is closed"

    def _finished(self, _future):
        with self._lock:
            self._pending -= 1

    def stop_accepting(self):
        with self._lock:
            if self._state == "OPEN":
                self._state = "DRAINING"

    def close(self):
        self.stop_accepting()
        # Owners close before destroying their publishers/clients. Let already
        # accepted work release its locks and finish using those ROS entities.
        self._pool.shutdown(wait=True, cancel_futures=True)
        with self._lock:
            self._state = "CLOSED"


@dataclass(frozen=True)
class CacheSnapshot:
    value: object
    fresh: bool
    error: str
    age_s: float | None
    pending: bool


class CachedCall:
    """One refresh at a time; consumers only read the latest immutable result."""
    def __init__(self, name, callback, initial, max_age_s=2.0, refresh_min_interval_s=0.0):
        self._worker = BoundedWorker(name)
        self._callback = callback
        self._lock = threading.Lock()
        self._value = initial
        self._initial = initial
        self._updated = 0.0
        self._first_requested = None
        self._max_age_s = max_age_s
        self._refresh_min_interval_s = refresh_min_interval_s
        self._error = ""

    def poll(self):
        snapshot = self.poll_snapshot()
        # Preserve the original fail-closed value interface for callers that
        # cannot separately represent freshness. Rich consumers retain identity
        # through poll_snapshot(), without treating the old value as current.
        if snapshot.age_s is not None and snapshot.age_s > self._max_age_s:
            return self._initial
        return snapshot.value

    def _status_locked(self, now):
        age = now - self._updated if self._updated else None
        error = self._error
        if not error:
            if age is None:
                overdue = (self._first_requested is not None and
                           now - self._first_requested > self._max_age_s)
                error = "initial snapshot refresh overdue" if overdue else "snapshot refresh pending"
            elif age > self._max_age_s:
                error = "snapshot refresh overdue"
        return age, error

    @property
    def error(self):
        with self._lock:
            return self._status_locked(time.monotonic())[1]

    def poll_snapshot(self):
        with self._lock:
            now = time.monotonic()
            if self._first_requested is None:
                self._first_requested = now
            refresh = (not self._updated or bool(self._error) or
                       now - self._updated >= self._refresh_min_interval_s)
        if refresh:
            self._worker.submit(self._refresh)
        pending = self._worker.busy
        with self._lock:
            age, error = self._status_locked(time.monotonic())
            return CacheSnapshot(self._value, age is not None and not error,
                                 error, age, pending)

    def _refresh(self):
        try:
            value = self._callback()
        except Exception as exc:  # preserve the last complete snapshot on IO errors
            with self._lock:
                self._error = f"{type(exc).__name__}: {exc}"
            return
        with self._lock:
            self._value = value
            self._updated = time.monotonic()
            self._error = ""

    def close(self):
        self._worker.close()

    def stop_refreshing(self):
        self._worker.stop_accepting()


def telemetry_callback(callback):
    """Contain observational callback failures without swallowing restart signals.

    Apply only to telemetry entry points, never control or reconnect callbacks.
    """
    @wraps(callback)
    def guarded(self, *args, **kwargs):
        source = callback.__qualname__
        faults = self._node._hope_telemetry_faults
        try:
            result = callback(self, *args, **kwargs)
        except Exception as exc:
            faults.failed(source, exc)
            try:
                self._telemetry_failure(callback.__name__, exc)
            except Exception as invalidation_error:
                faults.failed(source + ".invalidate", invalidation_error)
            return None
        faults.recovered(source)
        return result
    return guarded


class RosComponent:
    """A module using an injected Node, with no Node inheritance or ownership."""
    def __init__(self, node):
        self._node = node
        if not hasattr(node, "_hope_rpc_tracker"):
            node._hope_rpc_tracker = RpcDrainTracker()
        if not hasattr(node, "_hope_telemetry_faults"):
            from hope_telemetry import TelemetryFaults
            node._hope_telemetry_faults = TelemetryFaults(node)

    def _telemetry_failure(self, _callback, _exc):
        pass

    def __getattr__(self, name):
        node = self.__dict__.get("_node")
        if node is None or node is self:
            raise AttributeError(name)
        return getattr(node, name)

    def _declare_parameter(self, name, default):
        # session_id_path/runtime_root/planner_process_fragment are deliberately
        # shared parameters, including their existing CLI override names.
        if not self._node.has_parameter(name):
            self._node.declare_parameter(name, default)
        return self._node.get_parameter(name)

    def _worker_callback(self, callback, worker):
        source = callback.__qualname__
        async def invoke(request, response):
            tracker = self._node._hope_rpc_tracker
            tracker.enter(source)
            try:
                return await dispatch(request, response)
            finally:
                # This happens after await-ready resumes, not when the worker
                # future completes. Executor shutdown then joins the running
                # service task's synchronous send_response before Node teardown.
                tracker.leave(source)

        async def dispatch(request, response):
            from rclpy.task import Future
            count = len(request.parameters) if hasattr(request, "parameters") else 1
            try:
                lane = worker(request) if callable(worker) else worker
                submission = lane.try_submit(callback, request, response)
            except Exception as exc:
                return self._failed_response(response, f"operation rejected: {type(exc).__name__}: {exc}", count)
            if submission.future is None:
                return self._failed_response(response, submission.rejection, count)
            completed = submission.future
            ready = Future(executor=self._node.executor)

            def finish(done):
                try:
                    result = done.result()
                except Exception as exc:  # do not terminate ownership on RPC errors
                    result = self._failed_response(response, f"operation failed: {type(exc).__name__}: {exc}", count)
                if not ready.done():
                    ready.set_result(result)

            completed.add_done_callback(finish)
            return await ready
        return invoke

    @staticmethod
    def _failed_response(response, reason, count=1):
        if hasattr(response, "success"):
            response.success = False
            response.message = reason
        else:
            from rcl_interfaces.msg import SetParametersResult
            response.results = [SetParametersResult(successful=False, reason=reason) for _ in range(count)]
        return response
