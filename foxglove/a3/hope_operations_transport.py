"""Private socket transport for the native operations service's Python worker.

This module creates no ROS context, Node, executor or DDS participant. Generated
message classes are used only as data containers by the existing action code.
The socket is inherited from the C++ parent; there is no listening endpoint.
"""
from __future__ import annotations

from concurrent.futures import Future, ThreadPoolExecutor, InvalidStateError
from collections import OrderedDict
import array
import inspect
import json
import logging
import math
import select
import socket
import struct
import threading
import time
from types import SimpleNamespace
from hope_operations_core import WorkerAdmission, WireSnapshot


MAX_FRAME = 32 * 1024 * 1024  # Existing 9 MiB field requests plus JSON escaping.


def wire(value):
    if isinstance(value, float) and not math.isfinite(value):
        return "NaN" if math.isnan(value) else ("Infinity" if value > 0 else "-Infinity")
    if hasattr(value, "get_fields_and_field_types"):
        return {name: wire(getattr(value, name)) for name in value.get_fields_and_field_types()}
    if isinstance(value, dict):
        return {str(key): wire(item) for key, item in value.items()}
    if isinstance(value, (list, tuple, array.array, bytes, bytearray)):
        return [wire(item) for item in value]
    return value


def namespace(value):
    if isinstance(value, dict):
        return SimpleNamespace(**{key: namespace(item) for key, item in value.items()})
    if isinstance(value, list):
        return [namespace(item) for item in value]
    return value


def ros_type(cls):
    package, kind, *_ = cls.__module__.split(".")
    name = cls.__qualname__.split(".")[0]
    return f"{package}/{kind}/{name}"


def qos_spec(qos):
    if isinstance(qos, int):
        return dict(depth=qos, reliable=True, transient=False)
    return dict(depth=qos.depth, reliable=int(qos.reliability) != 2,
                transient=int(qos.durability) == 1)


class ServiceAdmission(WorkerAdmission):
    """Seal idle restart against both queued and running service callbacks."""
    def __init__(self):
        super().__init__()
        self.pending = 0
        self.generation = 0

    @property
    def sealed(self):
        return self.closed

    def enter(self):
        # Never block the sole IPC reply reader behind an owner lock. A busy
        # reconnect check defers itself below; this lock only protects counters.
        with self.lock:
            if self.closed:
                return False
            self.pending += 1
            self.generation += 1
            return True

    def leave(self):
        with self.lock:
            self.pending -= 1

    def seal_if(self, quiescent):
        # Never hold admission while probing another owner's lock. The socket
        # reader must remain able to deliver that owner's pending IPC reply.
        with self.lock:
            if self.sealed or self.pending:
                return False
            generation = self.generation
        if not quiescent():
            return False
        with self.lock:
            # Even a callback that STARTed async work and already returned
            # invalidates the earlier idle snapshot. Retry next poll instead.
            if self.sealed or self.pending or generation != self.generation:
                return False
            self._state = "DRAINING"
            return True


class WireConnection:
    def __init__(self, stream, sample_stream=None):
        self.stream = stream
        self.sample_stream = sample_stream
        self.write_lock = threading.Lock()
        self.lock = threading.Lock()
        self.sequence = 0
        self.pending = {}
        self.closed = threading.Event()
        self.subscriptions = {}
        self.services = {}
        self.timers = []
        self.next_entity = 0
        # Depth one per registered subscription, as with the original vendor
        # diagnostic DDS readers. Telemetry bursts cannot exhaust the RPC link.
        self.callbacks = OrderedDict()
        self.last_received = {}
        self.metrics = dict(samples_received=0, samples_coalesced=0,
                            dispatch_delay_last_s=0., dispatch_delay_max_s=0.)
        self.native_fault_callback = None
        self.shutdown_requested = threading.Event()
        self.callback_condition = threading.Condition()
        # Field lanes: two lifecycle/field RPCs, two independent recording RPCs,
        # one ordinary command, plus a slot to return their explicit busy reply.
        self.normal_slots = threading.BoundedSemaphore(6)
        self.urgent_slots = threading.BoundedSemaphore(1)
        self.admission = ServiceAdmission()
        self.normal = ThreadPoolExecutor(max_workers=6, thread_name_prefix="hope-operations-action")
        self.urgent = ThreadPoolExecutor(max_workers=1, thread_name_prefix="hope-operations-urgent")
        self.reader = threading.Thread(target=self._read, name="hope-operations-socket", daemon=True)
        self.reader.start()
        self.dispatcher = threading.Thread(target=self._dispatch, name="hope-operations-input", daemon=True)
        self.dispatcher.start()
        self.sample_reader = None
        if sample_stream is not None:
            self.sample_reader = threading.Thread(target=self._read_samples,
                name="hope-operations-diagnostics", daemon=True)
            self.sample_reader.start()

    def entity(self):
        with self.lock:
            self.next_entity += 1
            return self.next_entity

    def send(self, message, *, deadline=None):
        # Includes encoding, lock acquisition and all partial writes. This is
        # a transport budget, not a shorter hardware action/cleanup deadline.
        deadline = time.monotonic() + 5. if deadline is None else deadline
        encoded = json.dumps(wire(message), separators=(",", ":"), allow_nan=False, ensure_ascii=False).encode()
        if len(encoded) > MAX_FRAME:
            raise ValueError("operations IPC frame exceeds 32 MiB")
        if not self.write_lock.acquire(timeout=max(0., deadline-time.monotonic())):
            raise TimeoutError("operations send deadline before dispatch; not replayed")
        sent = 0
        try:
            if self.closed.is_set():
                raise ConnectionError("native operations connection closed")
            frame = memoryview(struct.pack("!I", len(encoded)) + encoded)
            while sent < len(frame):
                remaining = deadline-time.monotonic()
                if remaining <= 0 or not select.select([], [self.stream], [], max(0., remaining))[1]:
                    raise TimeoutError("operations send deadline; action outcome unknown; not replayed")
                try:
                    count = self.stream.send(frame[sent:], socket.MSG_DONTWAIT)
                except BlockingIOError:
                    continue
                if count <= 0:
                    raise ConnectionError("operations socket closed during send")
                sent += count
        except OSError:
            # Once a frame is partially written the stream cannot be reused.
            # With zero bytes sent the individual request can fail safely.
            if sent:
                self.closed.set()
                self.stream.shutdown(socket.SHUT_RDWR)
            raise
        finally:
            self.write_lock.release()

    def request(self, op, *, _deadline=None, **fields):
        with self.lock:
            self.sequence += 1
            seq = self.sequence
            future = Future()
            self.pending[seq] = future
        try:
            self.send(dict(op=op, id=seq, **fields), deadline=_deadline)
        except Exception:
            with self.lock:
                self.pending.pop(seq, None)
            raise
        return seq, future

    def query(self, op, *, _timeout=5., **fields):
        deadline = time.monotonic() + _timeout
        seq, future = self.request(op, _deadline=deadline, **fields)
        try:
            return future.result(timeout=max(0., deadline-time.monotonic()))
        finally:
            with self.lock:
                self.pending.pop(seq, None)

    def _exact(self, size, stream=None):
        stream = self.stream if stream is None else stream
        chunks = bytearray()
        while len(chunks) < size:
            chunk = stream.recv(size - len(chunks))
            if not chunk:
                raise EOFError("native operations socket EOF")
            chunks.extend(chunk)
        return chunks

    def _queue_sample(self, message):
        with self.callback_condition:
            entity = message["entity"]
            if entity in self.subscriptions:
                self.metrics["samples_received"] += 1
                previous = self.callbacks.get(entity)
                if previous is None or message["received"] >= previous["received"]:
                    self.metrics["samples_coalesced"] += int(previous is not None)
                    self.callbacks[entity] = message
                    self.callback_condition.notify()

    def diagnostics(self):
        with self.callback_condition:
            return dict(self.metrics, pending_samples=len(self.callbacks))

    def _read_samples(self):
        try:
            while not self.closed.is_set():
                size, = struct.unpack("!I", self._exact(4, self.sample_stream))
                if not 0 < size <= MAX_FRAME:
                    raise ValueError("invalid diagnostic frame length")
                sample = json.loads(self._exact(size, self.sample_stream))
                if sample.get("op") != "sample":
                    raise ValueError("non-sample on diagnostics channel")
                self._queue_sample(sample)
        except (OSError, EOFError, ValueError, RuntimeError):
            if not self.closed.is_set():
                logging.exception("optional operations diagnostics disconnected; control retained")
        finally:
            try:
                self.sample_stream.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass

    def _read(self):
        try:
            while not self.closed.is_set():
                size, = struct.unpack("!I", self._exact(4))
                if not 0 < size <= MAX_FRAME:
                    raise ValueError("invalid operations IPC frame length")
                message = json.loads(self._exact(size))
                op = message.get("op")
                if op == "reply":
                    with self.lock:
                        future = self.pending.pop(message["id"], None)
                    if future is not None and not future.done():
                        if message.get("error"):
                            future.set_exception(RuntimeError(message["error"]))
                        else:
                            future.set_result(message.get("value"))
                elif op == "call":
                    self._service(message)
                elif op == "sample":
                    # Keep original receipts; replacing a pending sample must
                    # not make old telemetry fresh or block service replies.
                    self._queue_sample(message)
                elif op == "shutdown":
                    self.shutdown_requested.set()
                elif op == "native_fault":
                    if self.native_fault_callback is not None:
                        self.native_fault_callback(message["source"], message["error"])
                else:
                    raise ValueError("unsupported parent operations message")
        except (OSError, EOFError, ValueError, RuntimeError):
            if not self.closed.is_set():
                logging.exception("operations transport stopped")
        finally:
            self.closed.set()
            # Real framing/EOF failures must not leave the parent writing to an
            # open socket with no reader. A telemetry burst never enters here.
            try:
                self.stream.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            with self.lock:
                pending, self.pending = self.pending, {}
            for future in pending.values():
                if not future.done():
                    future.set_exception(ConnectionError("native operations connection lost"))
            with self.callback_condition:
                self.callback_condition.notify_all()

    def _dispatch(self):
        while not self.closed.is_set():
            with self.callback_condition:
                self.callback_condition.wait_for(lambda: self.callbacks or self.closed.is_set())
                if self.closed.is_set():
                    return
                _, item = self.callbacks.popitem(last=False)
            spec = self.subscriptions.get(item["entity"])
            if spec is None:
                continue
            cls, callback, takes_info = spec
            try:
                # A flush racing an urgent edge can reach the socket later.
                # Never let an older cached snapshot overwrite a newer event.
                if item["received"] < self.last_received.get(item["entity"], -math.inf):
                    continue
                self.last_received[item["entity"]] = item["received"]
                delay = max(0., time.monotonic() - item["received"])
                with self.callback_condition:
                    self.metrics["dispatch_delay_last_s"] = delay
                    self.metrics["dispatch_delay_max_s"] = max(self.metrics["dispatch_delay_max_s"], delay)
                if "message" in item:
                    message = namespace(item["message"])
                    if (getattr(cls, "__name__", None) == "PoseStamped"
                            and getattr(cls, "__module__", "").startswith("geometry_msgs.msg.")):
                        # Restore invalid floats as floats, not strings/nulls:
                        # PlayerPositions must reject them under its existing
                        # numeric validation rather than fail the callback.
                        for vector, names in ((message.pose.position, "xyz"),
                                              (message.pose.orientation, "xyzw")):
                            for name in names:
                                setattr(vector, name, float(getattr(vector, name)))
                    message._received_monotonic_s = item["received"]
                    message._receipt_times_ns = item.get("receipt_times_ns", [])
                else:
                    from rclpy.serialization import deserialize_message
                    message = deserialize_message(bytes(item["cdr"]), cls)
                if takes_info:
                    callback(message, SimpleNamespace(
                        publisher_gid=item.get("publisher_gid", []),
                        received_monotonic_s=item["received"],
                        received_wall_ns=item.get("received_wall_ns")))
                else:
                    callback(message)
            except Exception:
                logging.exception("operations input callback failed")

    def _service(self, item):
        spec = self.services.get(item["entity"])
        urgent = spec and spec[2].endswith("/emergency_passive")
        slots = self.urgent_slots if urgent else self.normal_slots
        pool = self.urgent if urgent else self.normal
        if spec is None or not slots.acquire(blocking=False):
            self.send(dict(op="result", id=item["id"], error="operations worker busy or service unavailable"))
            return
        # Admission and the final idle-reconnect check share a lock. A START
        # either enters before sealing (and prevents restart), or is explicitly
        # refused before running. Never lose an accepted hardware operation.
        if not self.admission.enter():
            slots.release()
            self.send(dict(op="result", id=item["id"], error="operations DDS reconnecting; request not accepted or replayed"))
            return
        def invoke():
            try:
                cls, callback, _ = spec
                request = namespace(item.get("request", {}))
                request._native_runner_epoch = item.get("runner_epoch")
                request._native_runner_received = item.get("received")
                response = callback(request, cls.Response())
                self.send(dict(op="result", id=item["id"], response=response))
            except Exception as exc:
                logging.exception("operations service failed")
                try:
                    self.send(dict(op="result", id=item["id"], error=str(exc)))
                except OSError:
                    pass
            finally:
                self.admission.leave()
                slots.release()
        try:
            pool.submit(invoke)
        except Exception:
            self.admission.leave()
            slots.release()
            raise

    def spin(self, stop):
        # Timers preserve their original low rates. They never receive or
        # deserialize sensor traffic and cannot block the IPC response reader.
        while not stop.is_set() and not self.shutdown_requested.is_set() and not self.closed.is_set():
            now = time.monotonic()
            for timer in tuple(self.timers):
                if timer.active and now >= timer.next:
                    timer.next = now + timer.period
                    # Fatal timer errors propagate. Network reconnects use a
                    # separate idle-only, atomically sealed admission check;
                    # the standalone proxy's network watcher is disabled here.
                    timer.callback()
            # Sleep until the next due low-rate job, bounded for shutdown.
            due = [t.next for t in self.timers if t.active]
            stop.wait(min(.1, max(.001, min(due)-time.monotonic())) if due else .1)

    def close(self):
        self.closed.set()
        try:
            self.stream.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        self.stream.close()
        if self.sample_stream is not None:
            try:
                self.sample_stream.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            self.sample_stream.close()
        with self.callback_condition:
            self.callback_condition.notify_all()
        for thread in (self.reader, self.dispatcher, self.sample_reader):
            if thread is not None and thread is not threading.current_thread():
                thread.join(timeout=5.)
        self.normal.shutdown(wait=True, cancel_futures=False)
        self.urgent.shutdown(wait=True, cancel_futures=False)


class WorkerLogger:
    def __init__(self):
        self.logger = logging.getLogger("hope-operations-worker")
        self.last = {}

    def _log(self, level, text, **kwargs):
        period = kwargs.pop("throttle_duration_sec", 0.)
        now = time.monotonic()
        key = level, str(text)
        if period and now - self.last.get(key, -math.inf) < period:
            return
        if len(self.last) > 512:
            self.last.clear()
        self.last[key] = now
        self.logger.log(level, text)

    def debug(self, text, **kwargs):
        self._log(logging.DEBUG, text, **kwargs)

    def info(self, text, **kwargs):
        self._log(logging.INFO, text, **kwargs)

    def warning(self, text, **kwargs):
        self._log(logging.WARNING, text, **kwargs)

    warn = warning

    def error(self, text, **kwargs):
        self._log(logging.ERROR, text, **kwargs)

    def fatal(self, text, **kwargs):
        self._log(logging.CRITICAL, text, **kwargs)


class WorkerNode:
    _native_operations_transport = True

    def __init__(self, connection):
        self.connection = connection
        self.logger = WorkerLogger()
        self.parameters = set()

    def has_parameter(self, name):
        return name in self.parameters

    def declare_parameter(self, name, default):
        value = self.connection.query("parameter", name=name, default=default)
        self.parameters.add(name)
        return SimpleNamespace(value=value)

    def get_parameter(self, name):
        return SimpleNamespace(value=self.connection.query("parameter", name=name))

    def get_logger(self):
        return self.logger

    def configure_native_runner(self, timeout, seed):
        self.connection.query("runner_config", timeout=timeout, seed=seed)

    def native_runner_action(self, action, timeout, epoch, received):
        return self.connection.query("runner_action", action=action, epoch=epoch,
                                     received=received, _timeout=timeout + 5.)

    def get_clock(self):
        # This hardware worker uses the same host SYSTEM_TIME as the parent.
        # Sensor freshness remains original parent CLOCK_MONOTONIC ingress.
        return SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=time.time_ns()))

    def create_publisher(self, cls, topic, qos, **_kwargs):
        entity = self.connection.entity()
        self.connection.query("publisher", entity=entity, type=ros_type(cls), topic=topic, qos=qos_spec(qos))
        def publish(message):
            self.connection.send(dict(op="publish", entity=entity, message=message))
        return SimpleNamespace(publish=publish, msg_type=cls)

    def create_subscription(self, cls, topic, callback, qos, **_kwargs):
        entity = self.connection.entity()
        signature = inspect.signature(callback)
        try:
            signature.bind(object())
            takes_info = False
        except TypeError:
            signature.bind(object(), object())
            takes_info = True
        self.connection.subscriptions[entity] = cls, callback, takes_info
        self.connection.query("subscription", entity=entity, type=ros_type(cls), topic=topic, qos=qos_spec(qos))
        return entity

    def destroy_subscription(self, entity):
        self.connection.subscriptions.pop(entity, None)
        self.connection.query("remove_subscription", entity=entity)

    def create_service(self, cls, topic, callback, **_kwargs):
        entity = self.connection.entity()
        self.connection.services[entity] = cls, callback, topic
        self.connection.query("service", entity=entity, type=ros_type(cls), topic=topic)
        return entity

    def create_client(self, cls, topic, **_kwargs):
        entity = self.connection.entity()
        self.connection.query("client", entity=entity, type=ros_type(cls), topic=topic)
        def wait_for_service(timeout_sec=None):
            deadline = time.monotonic() + (timeout_sec or 0.)
            while True:
                if self.connection.query("client_ready", entity=entity):
                    return True
                if time.monotonic() >= deadline:
                    return False
                time.sleep(.05)
        def call_async(request):
            seq, source = self.connection.request("client_call", entity=entity, request=request)
            target = Future()
            def done(future):
                if target.cancelled():
                    return
                try:
                    result = namespace(future.result())
                except Exception as exc:
                    try:
                        target.set_exception(exc)
                    except InvalidStateError:
                        pass
                else:
                    try:
                        target.set_result(result)
                    except InvalidStateError:
                        pass  # Application timeout raced the downstream reply.
            def cancelled(future):
                if future.cancelled():
                    with self.connection.lock:
                        self.connection.pending.pop(seq, None)
                    source.cancel()
                    try:
                        self.connection.send(dict(op="client_cancel", call_id=seq))
                    except OSError:
                        pass
            target.add_done_callback(cancelled)
            source.add_done_callback(done)
            return target
        return SimpleNamespace(wait_for_service=wait_for_service, call_async=call_async,
                               service_is_ready=lambda: self.connection.query("client_ready", entity=entity))

    def create_timer(self, period, callback, **_kwargs):
        timer = SimpleNamespace(period=period, callback=callback, next=time.monotonic()+period, active=True)
        timer.cancel = lambda: self.destroy_timer(timer)
        self.connection.timers.append(timer)
        return timer

    @staticmethod
    def destroy_timer(timer):
        timer.active = False

    def get_publishers_info_by_topic(self, topic):
        return [namespace(value) for value in self.connection.query("publishers_info", topic=topic)]
