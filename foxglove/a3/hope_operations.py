#!/usr/bin/env python3
"""One ROS node for lifecycle ownership, explicit commands and telemetry.

Only this class constructs a Node. Its components share decoded input state;
blocking operations run in bounded workers, never executor callbacks.
"""
from __future__ import annotations

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String

from hope_command_proxy import CommandProxyModule
from hope_lifecycle_supervisor import LifecycleModule
from hope_monitor_core import read_ipv4_interface_signature
from hope_observer import ObserverModule
from hope_operations_core import CachedCall, OperationsState
from hope_operations_runtime import run_node


class DdsReconnectRequired(RuntimeError):
    pass


class HopeOperations(Node):
    def __init__(self):
        super().__init__("hope_operations", start_parameter_services=False)
        self.shared_state = OperationsState()
        self.commands = CommandProxyModule(self, shared_state=self.shared_state,
                                           watch_interfaces=False)
        self.observer = ObserverModule(self, shared_state=self.shared_state)
        self.lifecycle = LifecycleModule(self, shared_state=self.shared_state)
        # Each source progresses independently of telemetry timers while
        # preserving callback order within that wire.
        self._runner_input_group = MutuallyExclusiveCallbackGroup()
        self._base_input_group = MutuallyExclusiveCallbackGroup()
        # One reliable Runner reader satisfies the native gateway's warmup gate
        # and feeds command acknowledgments, telemetry and lifecycle readiness.
        self.create_subscription(Float64MultiArray, "/hope/runner/state_hdu_flat",
                                 self.commands._runner_state_callback, 10,
                                 callback_group=self._runner_input_group)
        # Preserve the command proxy's reliable calibration-receipt contract.
        self.create_subscription(Float64MultiArray, "/a3/base_pose_flat",
                                 self.commands._base_pose_callback, 10,
                                 callback_group=self._base_input_group)
        self._network = CachedCall("hope-network", read_ipv4_interface_signature, None)
        self._dds_interfaces = None
        self._network_status = "READY"
        self._operations_status = self.create_publisher(String, "/hope/operations/status", 10)
        self.create_timer(1.0, self._check_network)
        self._closed = False
        self._stopping = False
        self.get_logger().info("one operations Node: lifecycle, commands, observer; shared Runner/base inputs")

    def _check_network(self):
        if self._stopping:
            self._operations_status.publish(String(data=self._network_status))
            return
        current = self._network.poll()
        status = "READY"
        if current is not None:
            if self._dds_interfaces is None:
                self._dds_interfaces = current
            elif current != self._dds_interfaces:
                # Service admission and this final check use the same lock:
                # no accepted START can slip between STOPPED and restart.
                if self.shared_state.admission.seal_if(
                        lambda: self.lifecycle.quiescent() and self.commands.quiescent()
                        and self._hope_rpc_tracker.pending == 0):
                    self._network_status = "DDS_RECONNECTING: network interfaces changed; restarting stopped operations"
                    try:
                        self._operations_status.publish(String(data=self._network_status))
                    finally:
                        raise DdsReconnectRequired("network interfaces changed; reconnecting stopped operations DDS")
                status = ("DDS_RECONNECT_DEFERRED: network interfaces changed; "
                          "retaining managed session and hardware interlock until stopped")
        if status != self._network_status:
            self.get_logger().warning(status) if status != "READY" else self.get_logger().info(status)
            self._network_status = status
        self._operations_status.publish(String(data=status))

    def stop_accepting(self):
        self._stopping = True
        self._network_status = "DRAINING: finishing accepted operations; new requests are rejected"
        self.shared_state.admission.seal()
        for worker in (self.commands._command_workers, self.commands._urgent_workers):
            worker.stop_accepting()
        self.lifecycle.stop_accepting()
        self.observer._runtime_cache.stop_refreshing()
        self._network.stop_refreshing()
        self._operations_status.publish(String(data=self._network_status))

    def close(self):
        if self._closed:
            return
        self._closed = True
        errors = []
        try:
            self.stop_accepting()
        except Exception as exc:
            errors.append(exc)
        for component in (self.commands, self.lifecycle, self.observer, self._network):
            try:
                component.close()
            except Exception as exc:
                errors.append(exc)
        if errors:
            raise RuntimeError("operations cleanup failed: " + "; ".join(str(exc) for exc in errors)) from errors[0]

    def destroy_node(self):
        try:
            self.close()
        finally:
            self.shared_state.admission.close()
            result = Node.destroy_node(self)
        return result


def main():
    run_node(HopeOperations, num_threads=5)


if __name__ == "__main__":
    main()
