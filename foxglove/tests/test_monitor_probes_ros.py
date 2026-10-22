"""Monitor probe isolation on real rclpy, with external reads replaced."""
from concurrent.futures import Future, ThreadPoolExecutor
import os
from pathlib import Path
import sys
import threading
import time
from types import MethodType, SimpleNamespace
from unittest.mock import Mock, patch

import pytest

pytest.importorskip("rclpy")
pytestmark = pytest.mark.skipif(
    os.environ.get("ROS_LOCALHOST_ONLY") != "1" or os.environ.get("ROS_DOMAIN_ID") != "219",
    reason="requires isolated localhost domain 219",
)
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "a3"))
import rclpy
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Float64MultiArray
import hope_monitor as monitor_module
from hope_monitor_core import NtpProbeResult, ServiceProbeResult, parse_process_stat


def test_failed_cpu_future_is_consumed_before_exception_and_next_poll_retries():
    poisoned = Future()
    poisoned.set_exception(RuntimeError("fixture probe failure"))
    pool = Mock()
    node = SimpleNamespace(_cpu_future=poisoned, _probe_pool=pool, _probe_cpu=Mock())
    poll = MethodType(monitor_module.HopeMonitor._poll_cpu, node)
    with pytest.raises(RuntimeError, match="fixture probe failure"):
        poll()
    assert node._cpu_future is None
    pool.submit.assert_not_called()
    poll()
    pool.submit.assert_called_once_with(node._probe_cpu)
    assert node._cpu_future is pool.submit.return_value


def test_cpu_worker_owns_paired_samples_while_polling_only_collects():
    release, second_read = threading.Event(), threading.Event()
    source_threads, calls = [], []
    original_read = Path.read_text

    def read_stat(path, *args, **kwargs):
        if str(path) == "/proc/stat":
            source_threads.append(threading.get_ident())
            sample = min(len(calls), 2)
            return ("cpu 10 0 10 80 0 0 0 0\n", "cpu 30 0 20 150 0 0 0 0\n",
                    "cpu 50 0 30 220 0 0 0 0\n")[sample]
        return original_read(path, *args, **kwargs)

    def process_stat():
        calls.append(True)
        if len(calls) == 2:
            second_read.set()
            assert release.wait(3)
        ticks = 10 * len(calls)
        return {42: parse_process_stat("42 (planner) R " + " ".join(["0"] * 10 + [str(ticks), "0"]))}

    node = SimpleNamespace(_previous_cpu_times=None, _previous_process_cpu_times={},
                           _read_process_cpu_times=process_stat, _cpu_future=None,
                           pub_cpu_load=Mock(), pub_cpu_text=Mock(), pub_cpu_top_process=Mock(),
                           get_logger=lambda: Mock())
    node._probe_cpu = MethodType(monitor_module.HopeMonitor._probe_cpu, node)
    node._poll_cpu = MethodType(monitor_module.HopeMonitor._poll_cpu, node)
    with ThreadPoolExecutor(max_workers=1) as pool, patch.object(Path, "read_text", read_stat), \
            patch.object(monitor_module.os, "cpu_count", return_value=8):
        node._probe_pool = pool
        try:
            node._poll_cpu()
            node._cpu_future.result(1)
            first = node._previous_cpu_times
            node._poll_cpu()  # publish warmup, then submit the second paired read
            assert second_read.wait(1)
            pending = node._cpu_future
            for _ in range(20):
                node._poll_cpu()
            assert node._cpu_future is pending
            assert len(calls) == 2
            assert node._previous_cpu_times is first
            release.set()
            pending.result(1)
            node._poll_cpu()
            assert node.pub_cpu_load.publish.call_args.args[0].data == pytest.approx(30.0)
            assert "system=10.0%" in node.pub_cpu_top_process.publish.call_args.args[0].data
            assert all(thread_id != threading.get_ident() for thread_id in source_threads)
        finally:
            release.set()


def test_five_stalled_probes_leave_runner_base_and_default_callbacks_live(tmp_path):
    release = threading.Event()
    entered = {name: threading.Event() for name in ("cpu", "interfaces", "ntp", "pm", "hdu")}

    def block(name, result):
        entered[name].set()
        assert release.wait(5)
        return result

    def service(unit):
        return block("pm" if unit == "agibot_pm.service" else "hdu", ServiceProbeResult())

    with patch.object(monitor_module.HopeMonitor, "_probe_cpu",
                      lambda _self: block("cpu", monitor_module.CpuProbeResult())), \
            patch.object(monitor_module, "read_ipv4_interface_signature",
                         lambda: block("interfaces", ((2, "fixture0", "192.0.2.1"),))), \
            patch.object(monitor_module, "probe_ntp", lambda **_kwargs: block("ntp", NtpProbeResult())), \
            patch.object(monitor_module, "probe_systemd_service", service):
        rclpy.init(args=["--ros-args", "-p", "period_s:=0.02", "-p", "cpu_publish_period_s:=0.02",
                        "-p", "enable_tf_fallback:=false", "-p", "native_clock_topics:=true",
                        "-p", f"estop_latch_path:={tmp_path}/estop-latch"])
        node = monitor_module.HopeMonitor()
        probe = rclpy.create_node("monitor_probe_test")
        executor = MultiThreadedExecutor(num_threads=3)
        executor.add_node(node)
        executor.add_node(probe)
        pulses = []
        node.create_timer(.02, lambda: pulses.append(True))
        base = probe.create_publisher(Float64MultiArray, "/a3/base_pose_flat", 10)
        runner = probe.create_publisher(Float64MultiArray, "/hope/runner/mode_state", 10)

        def until(predicate):
            end = time.monotonic() + 3
            while not predicate() and time.monotonic() < end:
                executor.spin_once(timeout_sec=.01)
            assert predicate()

        try:
            until(lambda: all(event.is_set() for event in entered.values()))
            until(lambda: base.get_subscription_count() and runner.get_subscription_count())
            count = len(pulses)
            base.publish(Float64MultiArray(data=[2., 1., 0., 0., 0., 0., 0., 0., 1., 0., 0., 0., 1., 0., 77., 1.]))
            runner.publish(Float64MultiArray(data=[1., 1., 1., 9., 9., 1., 1., 0.]))
            until(lambda: node._base_flat_calibration_id == 77 and node._runner_state is not None
                  and len(pulses) >= count + 3)
            assert node._runner_state["applied_sequence"] == 9
            assert not node._cpu_future.done()
            assert not node._dds_future.done()
        finally:
            release.set()
            executor.shutdown()
            node.stop_workers()
            node._probe_pool.shutdown(wait=True)
            probe.destroy_node()
            node.destroy_node()
            rclpy.try_shutdown()


def test_interface_future_change_reaches_real_executor_as_reconnect(tmp_path):
    owner, source_threads = {}, []
    old = ((2, "fixture0", "192.0.2.1"),)
    new = ((2, "fixture0", "192.0.2.2"),)

    def interfaces():
        source_threads.append(threading.get_ident())
        node = owner.get("node")
        return new if node is not None and node._dds_interfaces == old else old

    with patch.object(monitor_module.HopeMonitor, "_probe_cpu", return_value=monitor_module.CpuProbeResult()), \
            patch.object(monitor_module, "read_ipv4_interface_signature", side_effect=interfaces), \
            patch.object(monitor_module, "probe_ntp", return_value=NtpProbeResult()), \
            patch.object(monitor_module, "probe_systemd_service", return_value=ServiceProbeResult()):
        rclpy.init(args=["--ros-args", "-p", "enable_tf_fallback:=false", "-p", "native_clock_topics:=true",
                        "-p", f"estop_latch_path:={tmp_path}/estop-latch"])
        node = monitor_module.HopeMonitor()
        owner["node"] = node
        executor = MultiThreadedExecutor(num_threads=3)
        executor.add_node(node)
        try:
            deadline = time.monotonic() + 4
            with pytest.raises(RuntimeError, match="Network interfaces changed; reconnecting monitor DDS"):
                while time.monotonic() < deadline:
                    executor.spin_once(timeout_sec=.02)
            assert len(source_threads) >= 2
            assert all(thread_id != threading.get_ident() for thread_id in source_threads)
        finally:
            executor.shutdown()
            node.stop_workers()
            node._probe_pool.shutdown(wait=True)
            node.destroy_node()
            rclpy.try_shutdown()
