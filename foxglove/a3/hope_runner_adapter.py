#!/usr/bin/env python3
"""ROS adapter for the unchanged model21800 manual TTY runner.

The adapter owns no gains or state transitions. It forwards the existing
sequenced mode contract to the fixed MDU helper, which presses the stock
runner's existing p/s/m keys and acknowledges only new runner log evidence.
"""

from concurrent.futures import Future, ThreadPoolExecutor
from pathlib import Path
import sys
import threading
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger

sys.path.insert(0, str(Path(__file__).resolve().parent))
from hope_runner_adapter_core import (  # noqa: E402
    SshRunnerClient,
    decode_mode_command,
    execute_mode_command,
    unavailable_status,
)


class HopeRunnerAdapter(Node):
    def __init__(self):
        super().__init__("hope_model21800_runner_adapter")
        self._client = SshRunnerClient()
        self._pool = ThreadPoolExecutor(
            max_workers=3, thread_name_prefix="hope-model21800"
        )
        self._lock = threading.Lock()
        self._latest_status = unavailable_status("STARTING")
        self._last_backend_status_monotonic = 0.0
        self._estop_latched = False
        self._status_future: Future | None = None
        self._command_futures: set[Future] = set()
        self._callback_group = ReentrantCallbackGroup()

        self._state_pub = self.create_publisher(
            Float64MultiArray, "/hope/runner/mode_state", 10
        )
        self.create_subscription(
            Float64MultiArray,
            "/hope/runner/mode_command",
            self._on_mode_command,
            10,
            callback_group=self._callback_group,
        )
        self.create_service(
            Trigger,
            "/hope/runner/emergency_stop",
            self._emergency_stop,
            callback_group=self._callback_group,
        )
        self.create_timer(0.2, self._poll_status)
        self.create_timer(0.2, self._publish_state)
        self.get_logger().info(
            "model21800 adapter ready; runner remains STOPPED until PREPARE"
        )

    def _update_status(self, status, *, force=False):
        with self._lock:
            if status.reason in {
                "ESTOP_LATCHED",
                "ESTOP_RUNNER_STOPPED",
                "ALREADY_STOPPED",
            } and status.fault:
                self._estop_latched = True
            if self._estop_latched and not status.fault:
                return
            if force or status.request_seq >= self._latest_status.request_seq:
                self._latest_status = status
                self._last_backend_status_monotonic = time.monotonic()

    def _on_mode_command(self, message):
        with self._lock:
            if self._estop_latched:
                self.get_logger().error(
                    "rejected runner command because E-stop is locally latched"
                )
                return
        try:
            sequence, code = decode_mode_command(list(message.data))
        except (TypeError, ValueError) as exc:
            self.get_logger().error(f"rejected malformed runner command: {exc}")
            return
        future = self._pool.submit(
            execute_mode_command, self._client, sequence, code
        )
        with self._lock:
            self._command_futures.add(future)

    def _reap_command_futures(self):
        with self._lock:
            completed = [future for future in self._command_futures if future.done()]
            for future in completed:
                self._command_futures.remove(future)
        for future in completed:
            try:
                status = future.result()
            except Exception as exc:  # noqa: BLE001 - convert to fail-closed state
                self.get_logger().error(f"runner command failed closed: {exc}")
                status = unavailable_status("COMMAND_TRANSPORT_FAILED")
            self._update_status(status, force=status.reason == "COMMAND_TRANSPORT_FAILED")

    def _poll_status(self):
        self._reap_command_futures()
        with self._lock:
            future = self._status_future
            if future is None:
                self._status_future = self._pool.submit(
                    self._client.invoke, "status"
                )
                return
            if not future.done():
                return
            self._status_future = None
        try:
            status = future.result()
        except Exception as exc:  # noqa: BLE001 - publish UNKNOWN on SSH/helper loss
            self.get_logger().warning(
                f"runner status unavailable: {exc}", throttle_duration_sec=5.0
            )
            status = unavailable_status("STATUS_TRANSPORT_FAILED")
        self._update_status(status, force=status.reason == "STATUS_TRANSPORT_FAILED")

    def _publish_state(self):
        with self._lock:
            status = self._latest_status
            received = self._last_backend_status_monotonic
        if received <= 0.0 or time.monotonic() - received > 1.5:
            status = unavailable_status("STATUS_STALE")
        message = Float64MultiArray()
        message.data = status.wire_data()
        self._state_pub.publish(message)

    def _emergency_stop(self, _request, response):
        """Latch and stop only this adapter's exactly identified runner."""

        with self._lock:
            self._estop_latched = True
        try:
            status = self._client.invoke("stop", timeout_s=2.5)
        except Exception as exc:  # noqa: BLE001 - caller must see partial failure
            status = unavailable_status("STOP_TRANSPORT_FAILED")
            self._update_status(status, force=True)
            response.success = False
            response.message = f"managed runner stop failed: {exc}"
            return response
        self._update_status(status)
        response.success = (
            status.state == "STOPPED"
            and status.reason in {"ALREADY_STOPPED", "ESTOP_RUNNER_STOPPED"}
        )
        if response.success:
            response.message = (
                "managed model21800 runner is stopped and restart is locally latched"
            )
        else:
            response.message = (
                f"managed runner stop is unconfirmed: state={status.state} "
                f"reason={status.reason}"
            )
        return response

    def stop_workers(self):
        self._pool.shutdown(wait=False, cancel_futures=True)


def main():
    rclpy.init()
    node = HopeRunnerAdapter()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.stop_workers()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
