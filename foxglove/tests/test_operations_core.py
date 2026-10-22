"""No ROS or hardware: bounded admission, snapshot and shutdown races."""
from pathlib import Path
import sys
import threading
import time
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "a3"))
from hope_operations_core import BoundedWorker, CachedCall, OperationsState, RpcDrainTracker, WorkerAdmission
from hope_observer_core import REQUIRED_BASE_FLAGS


class OperationsCoreTests(unittest.TestCase):
    def test_rpc_tracker_retains_source_counts_until_each_callback_resumes(self):
        tracker = RpcDrainTracker()
        tracker.enter("LifecycleModule._apply_config")
        tracker.enter("LifecycleModule._apply_config")
        tracker.enter("CommandProxyModule._calibrate")
        sources = tracker.pending_sources()
        self.assertEqual(tracker.pending, 3)
        self.assertEqual(sources["LifecycleModule._apply_config"], 2)
        tracker.leave("LifecycleModule._apply_config")
        self.assertEqual(tracker.pending_sources()["LifecycleModule._apply_config"], 1)
        tracker.leave("CommandProxyModule._calibrate")
        tracker.leave("LifecycleModule._apply_config")
        self.assertEqual(tracker.pending, 0)
        self.assertEqual(tracker.pending_sources(), {})
        self.assertEqual(sources["LifecycleModule._apply_config"], 2)

    def test_saturated_command_lane_does_not_queue_or_block_emergency(self):
        release = threading.Event()
        ordinary = BoundedWorker("test-command")
        urgent = BoundedWorker("test-urgent")
        try:
            first = ordinary.submit(release.wait, 3)
            self.assertIsNone(ordinary.submit(lambda: self.fail("queued excess work")))
            self.assertEqual(urgent.submit(lambda: "ack").result(1), "ack")
            self.assertFalse(first.done())
        finally:
            release.set()
            ordinary.close()
            urgent.close()

    def test_rejection_reason_is_atomic_and_distinguishes_busy_drain_and_closed(self):
        admission = WorkerAdmission()
        lane = BoundedWorker("test-reasons", admission=admission)
        release = threading.Event()
        try:
            accepted = lane.try_submit(release.wait, 3)
            busy = lane.try_submit(lambda: None)
            self.assertIsNotNone(accepted.future)
            self.assertTrue(busy.rejection.startswith("BUSY:"))
            admission.seal()
            draining = lane.try_submit(lambda: None)
            self.assertTrue(draining.rejection.startswith("DRAINING:"))
            self.assertTrue(busy.rejection.startswith("BUSY:"))
            release.set()
            lane.close()
            admission.close()
            closed = lane.try_submit(lambda: None)
            self.assertTrue(closed.rejection.startswith("CLOSED:"))
            self.assertTrue(draining.rejection.startswith("DRAINING:"))
        finally:
            release.set()
            lane.close()

    def test_shutdown_seals_admission_and_waits_for_accepted_work(self):
        release, closing = threading.Event(), threading.Event()
        lane = BoundedWorker("test-shutdown")
        accepted = lane.submit(release.wait, 3)
        def close():
            lane.close()
            closing.set()
        thread = threading.Thread(target=close)
        thread.start()
        try:
            self.assertFalse(closing.wait(.05))
            self.assertIsNone(lane.submit(lambda: None))
            release.set()
            self.assertTrue(closing.wait(1))
            self.assertTrue(accepted.done())
        finally:
            release.set()
            thread.join(3)

    def test_restart_check_cannot_admit_start_between_quiescence_and_seal(self):
        admission = WorkerAdmission()
        lane = BoundedWorker("test-admission", admission=admission)
        racing = threading.Event()
        results = []
        def attempt():
            racing.set()
            results.append(lane.submit(lambda: self.fail("START admitted after restart check")))
        thread = threading.Thread(target=attempt)
        def quiescent():
            thread.start()
            self.assertTrue(racing.wait(1))
            return not lane.busy
        try:
            self.assertTrue(admission.seal_if(quiescent))
            thread.join(1)
            self.assertEqual(results, [None])
        finally:
            lane.close()

    def test_busy_restart_check_leaves_service_admission_open(self):
        admission = WorkerAdmission()
        lane = BoundedWorker("test-busy", admission=admission)
        release = threading.Event()
        try:
            running = lane.submit(release.wait, 3)
            self.assertFalse(admission.seal_if(lambda: not lane.busy))
            self.assertFalse(admission.closed)
            release.set()
            running.result(1)
            self.assertEqual(lane.submit(lambda: "next").result(1), "next")
        finally:
            release.set()
            lane.close()

    def test_malformed_shared_runner_retains_last_good_receipt_and_notifies_waiter(self):
        state = OperationsState()
        self.assertEqual(state.base.snapshot().error, "NO BASE PACKET")
        packet = [2., 1234., 8., 1., 1., 1., 0., 2., 1., 1., 1., 1., 0., -1., -1., 0., 99., 2., 1., 1., 555.]
        state.runner.update(packet)
        prior = state.runner.snapshot()
        self.assertEqual(prior.value.run_mode, "PD_STAND")
        ready, notified = threading.Event(), threading.Event()
        def wait():
            with state.runner.condition:
                ready.set()
                state.runner.condition.wait_for(lambda: state.runner.snapshot() != prior, timeout=2)
                notified.set()
        waiter = threading.Thread(target=wait)
        waiter.start()
        self.assertTrue(ready.wait(1))
        state.runner.update([2.])
        self.assertTrue(notified.wait(1))
        waiter.join(1)
        malformed = state.runner.snapshot()
        self.assertIs(malformed.value, prior.value)
        self.assertEqual(malformed.received_ns, prior.received_ns)
        self.assertIn("MALFORMED", malformed.error)
        self.assertEqual(malformed.decode_error, malformed.error)
        self.assertEqual(prior.value.run_mode, "PD_STAND")  # immutable prior read
        state.runner.update(packet)
        self.assertEqual(state.runner.snapshot().error, "")
        self.assertEqual(state.runner.snapshot().decode_error, "")

    def test_base_decode_error_preserves_receipt_but_explicit_invalid_replaces_it(self):
        state = OperationsState()
        packet = [2., 1., 5., 1., 0., 0., 0., .2, 1., 0., 0., 0., 1.,
                  float(REQUIRED_BASE_FLAGS), 77., 1.]
        with patch("hope_operations_core.time.monotonic_ns", return_value=100):
            state.base.update(packet)
        valid = state.base.snapshot()
        self.assertTrue(valid.value.valid)
        self.assertEqual(valid.value.reason, "valid")
        self.assertEqual(valid.error, "")
        with patch("hope_operations_core.time.monotonic_ns", return_value=200):
            state.base.update([2.])
        malformed = state.base.snapshot()
        self.assertIs(malformed.value, valid.value)
        self.assertEqual(malformed.received_ns, 100)
        self.assertTrue(malformed.decode_error.startswith("MALFORMED BASE PACKET:"))
        packet[1] = 0.
        with patch("hope_operations_core.time.monotonic_ns", return_value=300):
            state.base.update(packet)
        invalid = state.base.snapshot()
        self.assertFalse(invalid.value.valid)
        self.assertEqual(invalid.received_ns, 300)
        self.assertEqual(invalid.decode_error, "")
        self.assertEqual(invalid.error, "producer marked base packet invalid")

    def test_blocked_or_failed_file_refresh_expires_old_health(self):
        release = threading.Event()
        cache = CachedCall("test-cache", lambda: "healthy", "unknown", max_age_s=2)
        try:
            # Set a known complete snapshot without any scheduling/timing race.
            with patch("hope_operations_core.time.monotonic", return_value=10):
                cache._refresh()
            cache._callback = lambda: release.wait(3)
            with patch("hope_operations_core.time.monotonic", return_value=13):
                self.assertEqual(cache.poll(), "unknown")
                self.assertEqual(cache.error, "snapshot refresh overdue")
        finally:
            release.set()
            cache.close()

    def test_observer_refresh_floor_keeps_initial_error_and_overdue_semantics(self):
        cache = CachedCall("test-floor", lambda: "healthy", "unknown",
                           max_age_s=2.0, refresh_min_interval_s=1.0)
        try:
            with patch.object(cache._worker, "submit", return_value=None) as submit:
                with patch("hope_operations_core.time.monotonic", return_value=10):
                    self.assertFalse(cache.poll_snapshot().fresh)
                    submit.assert_called_once()
                    cache._refresh()
                submit.reset_mock()
                with patch("hope_operations_core.time.monotonic", return_value=10.99):
                    self.assertTrue(cache.poll_snapshot().fresh)
                    submit.assert_not_called()
                with patch("hope_operations_core.time.monotonic", return_value=11):
                    self.assertTrue(cache.poll_snapshot().fresh)
                    submit.assert_called_once()
                with patch("hope_operations_core.time.monotonic", return_value=12.01):
                    snapshot = cache.poll_snapshot()
                    self.assertFalse(snapshot.fresh)
                    self.assertEqual(snapshot.error, "snapshot refresh overdue")
                    self.assertEqual(snapshot.value, "healthy")
                # Even immediately after the last good read, a failed refresh
                # remains unknown and retries at the next poll.
                cache._error = "OSError: fixture read failed"
                submit.reset_mock()
                with patch("hope_operations_core.time.monotonic", return_value=10.1):
                    self.assertFalse(cache.poll_snapshot().fresh)
                    submit.assert_called_once()
        finally:
            cache.close()


if __name__ == "__main__":
    unittest.main()
