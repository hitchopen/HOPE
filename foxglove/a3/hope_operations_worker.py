#!/usr/bin/env python3
"""Audited action/configuration worker for hope-operations; creates no ROS Node.

The C++ parent owns DDS, high-rate input caches and ROS endpoints. Python keeps
the existing lifecycle, command, calibration and display rules behind one
inherited private socket. No public socket or caller-supplied shell is accepted.
"""
from __future__ import annotations

import argparse
import logging
import signal
import socket
import sys
import threading

from hope_operations_transport import WireConnection, WorkerNode
from hope_operations_core import CachedCall, OperationsState
from hope_monitor_core import read_ipv4_interface_signature


class DdsReconnectRequired(RuntimeError):
    """Rebuild DDS only after the combined owner has atomically become idle."""


class IdleNetworkReconnect:
    def __init__(self, connection, lifecycle, proxy, logger, read_interfaces=read_ipv4_interface_signature):
        self.connection, self.lifecycle, self.proxy = connection, lifecycle, proxy
        self.logger, self.read_interfaces = logger, read_interfaces
        self.baseline = read_interfaces()
        self.status = "READY: native ROS operations, Python orchestration worker"

    def poll(self):
        current = self.read_interfaces()
        status = "READY: native ROS operations, Python orchestration worker"
        if current is not None:
            if self.baseline is None:
                self.baseline = current
            elif current != self.baseline:
                # Count all admitted RPCs, including urgent commands, not just
                # the proxy action lock. Async START/STOP keep lifecycle busy
                # and retain the hardware interlock after their RPC returns.
                if self.connection.admission.seal_if(lambda:
                        self.lifecycle.quiescent() and self.proxy.quiescent()
                        and self._native_idle()):
                    self._status("DDS_RECONNECTING: network interfaces changed; restarting stopped operations")
                    raise DdsReconnectRequired("network interfaces changed; reconnecting stopped operations DDS")
                status = ("DDS_RECONNECT_DEFERRED: network interfaces changed; "
                          "retaining managed session and hardware interlock until stopped")
        self._status(status)

    def _native_idle(self):
        # Native emergency calls bypass Python admission. Query outside its
        # lock, and defer on unknown status; teardown also drains a late race.
        try:
            return self.connection.query("runner_busy") is False
        except (OSError, RuntimeError, TimeoutError):
            return False

    def _status(self, status):
        if status != self.status:
            self.logger.info(status)
            self.connection.send(dict(op="operations_status", status=status))
            self.status = status


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--fd", type=int, required=True)
    parser.add_argument("--samples-fd", type=int)
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format="[operations-worker] %(levelname)s %(message)s")
    stop = threading.Event()
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    connection = WireConnection(socket.socket(fileno=args.fd),
        socket.socket(fileno=args.samples_fd) if args.samples_fd is not None else None)
    shared = OperationsState()
    shared.admission = connection.admission
    components = []
    network = None
    try:
        from hope_command_proxy import CommandProxyModule
        from hope_lifecycle_supervisor import LifecycleModule
        from hope_operations_native_state import NativeWire, NativePlayerPositions, RuntimeRelay
        node = WorkerNode(connection)
        node.native_player_positions = NativePlayerPositions(connection)
        shared.base = NativeWire(connection, "base")
        shared.runner = NativeWire(connection, "runner")
        # The standalone proxy can restart itself on an address change. That
        # watcher must never restart the combined lifecycle/session owner.
        proxy = CommandProxyModule(node, shared_state=shared, watch_interfaces=False)
        components.append(proxy)
        observer = RuntimeRelay(node)
        components.append(observer)
        lifecycle = LifecycleModule(node, shared_state=shared)
        components.append(lifecycle)
        def native_fault(source, error):
            faults = node._hope_telemetry_faults
            if error:
                faults.failed(source, RuntimeError(error))
            else:
                faults.recovered(source)
        connection.native_fault_callback = native_fault
        # Slow OS probes use the same cached worker as the main reference.
        network = CachedCall("hope-network", read_ipv4_interface_signature, None)
        reconnect = IdleNetworkReconnect(connection, lifecycle, proxy, node.get_logger(), network.poll)
        node.create_timer(1.0, reconnect.poll)
        connection.send(dict(op="ready"))
        connection.spin(stop)
    finally:
        errors = []
        shared.admission.seal()
        for timer in connection.timers:
            timer.active = False
        for component in components:
            stop_accepting = getattr(component, "stop_accepting", None)
            if stop_accepting is not None:
                try:
                    stop_accepting()
                except Exception as exc:
                    errors.append(exc)
        # Finish accepted explicit actions; no timed kill or automatic retry.
        connection.normal.shutdown(wait=True, cancel_futures=False)
        connection.urgent.shutdown(wait=True, cancel_futures=False)
        for component in reversed(components):
            try:
                # Field close() drains accepted START/STOP/field workers before
                # releasing the hardware interlock. No new orchestration copy.
                component.close()
            except Exception as exc:
                errors.append(exc)
        if network is not None:
            try:
                network.close()
            except Exception as exc:
                errors.append(exc)
        shared.admission.close()
        if not connection.closed.is_set():
            connection.send(dict(op="drained", success=sys.exc_info()[0] is None and not errors))
        connection.close()
        if errors:
            raise RuntimeError("operations cleanup failed: " + "; ".join(map(str, errors))) from errors[0]


if __name__ == "__main__":
    main()
