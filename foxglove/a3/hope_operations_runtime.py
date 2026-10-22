"""Graceful ROS shutdown for operations and lifecycle/command compatibility entries."""
import signal
import inspect
import sys
import math
import threading
import time

import rclpy
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.signals import SignalHandlerOptions


def shutdown_executor(executor, *, abandon_responses=False):
    """Join callback threads on both early and current Jazzy executors.

    HDU rclpy 7.1.6 inherits Executor.shutdown: it neither accepts
    wait_for_threads nor joins MultiThreadedExecutor's worker pool. Stop
    admission first, then explicitly join that pool before destroying nodes.
    Inspect the installed base API rather than catching unrelated TypeErrors.
    """
    kwargs = {"timeout_sec": 0.0} if abandon_responses else {}
    if "wait_for_threads" in inspect.signature(MultiThreadedExecutor.shutdown).parameters:
        executor.shutdown(**kwargs, wait_for_threads=True)
    else:
        # The caller has stopped spinning. Early Executor.shutdown destroys
        # guard conditions without waiting for callbacks; join first so a
        # finishing callback cannot touch a guard already marked for deletion.
        executor._executor.shutdown(wait=True)
        executor.shutdown(**kwargs)


def drain_and_destroy(node, executor, *, response_grace_s=0.2, response_timeout_s=5.0, callback_error=None):
    """Drain workers and suspended RPCs while keeping status and DDS live.

    A completed worker does not mean its ROS service response has been sent.
    RPC tracking ends only after the coroutine resumes; executor shutdown then
    joins the synchronous send_response step before destroying the Node.
    The response timeout starts only after worker cleanup completes; it never
    shortens collection/recovery or releases a hardware lock early. It bounds
    suspended RPC draining, not a callback stuck synchronously on an OS thread.
    """
    if not math.isfinite(response_timeout_s) or response_timeout_s <= 0:
        raise ValueError("response_timeout_s must be positive and finite")
    logger = node.get_logger()
    done = threading.Event()

    def report(where, exc):
        logger.error(f"shutdown {where}: {type(exc).__name__}: {exc}")

    try:
        node.stop_accepting()
    except Exception as exc:
        report("admission", exc)

    def close():
        try:
            node.close()
        except Exception as exc:
            report("cleanup", exc)
        finally:
            done.set()

    cleanup = threading.Thread(target=close, name="hope-operations-close")
    cleanup.start()
    abandon_responses = False
    try:
        grace_until = None
        response_deadline = None
        while True:
            # An externally shut-down context cannot complete suspended ROS
            # callbacks. Still join accepted worker cleanup before teardown.
            context_ok = node.context.ok()
            pending = node._hope_rpc_tracker.pending
            cleanup_done = done.is_set()
            now = time.monotonic()
            if cleanup_done and response_deadline is None:
                response_deadline = now + response_timeout_s
            if cleanup_done and context_ok and pending and now >= response_deadline:
                sources = node._hope_rpc_tracker.pending_sources()
                logger.error(
                    f"shutdown response drain timed out after {response_timeout_s:.1f}s "
                    f"following worker cleanup: pending={pending} sources={sources}; "
                    "pending responses are abandoned")
                abandon_responses = True
                break
            if cleanup_done and (pending == 0 or not context_ok):
                if not context_ok:
                    if pending:
                        logger.error("DDS context closed before pending RPC responses could drain")
                        abandon_responses = True
                    break
                if grace_until is None:
                    grace_until = time.monotonic() + response_grace_s
                if time.monotonic() >= grace_until:
                    break
            else:
                grace_until = None
            if context_ok:
                try:
                    executor.spin_once(timeout_sec=0.05)
                except ExternalShutdownException:
                    continue
                except Exception as exc:
                    # A telemetry callback failure must not abandon lifecycle
                    # ownership or strand a command waiting for a DDS receipt.
                    # Early Jazzy can enqueue the same completed Task twice.
                    # Its already-propagating exception is not a new drain fault.
                    if exc is not callback_error:
                        report("callback", exc)
            else:
                done.wait(0.05)
    finally:
        # Keep cleanup joined even if a non-callback interruption escapes.
        cleanup.join()
        try:
            shutdown_executor(executor, abandon_responses=abandon_responses)
        finally:
            node.destroy_node()


def run_node(node_factory, *, num_threads=4):
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    stopping = threading.Event()
    previous = {sig: signal.signal(sig, lambda *_args: stopping.set())
                for sig in (signal.SIGINT, signal.SIGTERM)}
    node = None
    executor = None
    try:
        node = node_factory()
        executor = MultiThreadedExecutor(num_threads=num_threads)
        executor.add_node(node)
        while not stopping.is_set() and node.context.ok():
            executor.spin_once(timeout_sec=0.2)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            if node is not None:
                if executor is not None:
                    drain_and_destroy(node, executor, callback_error=sys.exc_info()[1])
                else:
                    node.destroy_node()
        finally:
            rclpy.try_shutdown()
            for sig, handler in previous.items():
                signal.signal(sig, handler)
