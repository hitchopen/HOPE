"""Same-boot recovery evidence; no SSH, ROS or hardware actions."""
from pathlib import Path
import fcntl
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "a3"))
from hope_operations_recovery import SessionReceipt, recover_stopped_session, require_stopped_owners
from hope_lifecycle_core import LifecycleConfig, save_config_atomic
from hope_operations_core import WireSnapshot
from hope_runner_control_core import runner_session_fingerprint


class RecoveryTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.path = Path(self.temp.name)/"receipt.json"
        self.receipt = SessionReceipt(self.path, "host-boot")
        self.config = LifecycleConfig(revision=1)
        self.session = "model21800_20261029T110000Z"

    def test_same_boot_roundtrip_and_fresh_matching_runner_required(self):
        self.receipt.record(self.session, "OPTITRACK", self.config, 42)
        value = self.receipt.load(self.config)
        state = SimpleNamespace(boot_id=42, session_fingerprint=runner_session_fingerprint(self.session))
        snapshot = WireSnapshot(state, 1_000_000_000)
        SessionReceipt.verify(value, snapshot, 1_100_000_000)
        for bad in (WireSnapshot(state, 0), WireSnapshot(None),
                    WireSnapshot(state, 1_000_000_000, decode_error="MALFORMED"),
                    WireSnapshot(SimpleNamespace(boot_id=43, session_fingerprint=state.session_fingerprint), 1_000_000_000),
                    WireSnapshot(SimpleNamespace(boot_id=42, session_fingerprint=123), 1_000_000_000)):
            with self.assertRaisesRegex(ValueError, "RECOVERY_IDENTITY_UNVERIFIED"):
                SessionReceipt.verify(value, bad, 2_000_000_000)
        self.receipt.clear()
        self.assertIsNone(self.receipt.load(self.config))

    def test_no_adoption_across_host_boots_or_config_changes(self):
        self.assertIsNone(self.receipt.load(self.config))
        self.receipt.record(self.session, "KERNEL", self.config)
        self.assertIsNone(SessionReceipt(self.path, "new-boot").load(self.config))
        with self.assertRaisesRegex(ValueError, "configuration differs"):
            self.receipt.load(LifecycleConfig(revision=2))

    def test_bad_receipt_is_not_cleanup_authority(self):
        self.path.write_text("{}")
        with self.assertRaises(ValueError):self.receipt.load(self.config)
        self.path.unlink()
        self.path.symlink_to(Path(self.temp.name)/"missing")
        # A dangling link has no usable receipt either.
        self.assertIsNone(self.receipt.load(self.config))


class OfflineRecoveryTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.config_path = self.root / "config.json"
        self.config = LifecycleConfig(laptop_wifi_ip="192.168.8.132", hdu_wifi_ip="192.168.8.182",
                                      mdu_internal_ip="10.42.10.12", revision=1)
        save_config_atomic(self.config_path, self.config)
        self.boot = self.root / "boot"
        self.boot.write_text("host-boot")
        self.lock = self.root / "hardware-operation.lock"
        self.lock.touch()
        self.session = "model21800_20261029T110000Z"
        self.receipt = SessionReceipt(self.root / "operations_session.json", "host-boot")
        self.receipt.record(self.session, "OPTITRACK", self.config)
        self.stopped, self.cleanup = Mock(), Mock(return_value="LOGS_COLLECTED")

    def recover(self, **kw):
        return recover_stopped_session(self.config_path, self.session, self.cleanup,
            confirmed=kw.pop("confirmed", True), require_stopped=self.stopped, boot_path=self.boot, **kw)

    def test_missing_runner_and_partial_startup_cleanup_archives_only_after_success(self):
        original = self.receipt.path.read_bytes()
        def cleanup(*_args):
            self.assertTrue(self.receipt.path.exists())
            with self.lock.open("r+") as contender:
                with self.assertRaises(BlockingIOError):
                    fcntl.flock(contender, fcntl.LOCK_EX | fcntl.LOCK_NB)
        self.cleanup.side_effect = cleanup
        archive = self.recover()
        self.assertFalse(self.receipt.path.exists())
        self.assertEqual(archive.read_bytes(), original)
        self.assertEqual(self.stopped.call_count, 3)
        self.assertEqual(self.cleanup.call_args.args[1], self.session)
        self.assertEqual(self.cleanup.call_args.args[3], "OPTITRACK")
        with self.assertRaisesRegex(ValueError, "does not match"):
            self.recover()
        self.assertEqual(self.cleanup.call_count, 1)

    def test_crash_mid_cleanup_keeps_receipt_and_can_be_retried(self):
        self.cleanup.side_effect = RuntimeError("SSH failed midway")
        with self.assertRaisesRegex(RuntimeError, "SSH failed"):
            self.recover()
        self.assertTrue(self.receipt.path.exists())
        self.cleanup.side_effect = None
        self.assertTrue(self.recover().exists())

    def test_unconfirmed_or_active_owner_never_sends_cleanup(self):
        with self.assertRaisesRegex(ValueError, "confirm"):
            self.recover(confirmed=False)
        self.stopped.side_effect = RuntimeError("active owner")
        with self.assertRaisesRegex(RuntimeError, "active owner"):
            self.recover()
        self.cleanup.assert_not_called()
        self.assertTrue(self.receipt.path.exists())

    def test_missing_or_busy_interlock_never_sends_cleanup(self):
        with self.lock.open("r+") as contender:
            fcntl.flock(contender, fcntl.LOCK_EX | fcntl.LOCK_NB)
            with self.assertRaises(BlockingIOError):
                self.recover()
        self.lock.unlink()
        with self.assertRaisesRegex(RuntimeError, "interlock|hardware-operation.lock"):
            self.recover()
        self.assertFalse(self.lock.exists())
        self.cleanup.assert_not_called()

    def test_wrong_session_boot_or_config_cannot_authorize_cleanup(self):
        for change in ("session", "boot", "config", "corrupt"):
            with self.subTest(change=change):
                self.receipt.record(self.session, "OPTITRACK", self.config)
                self.boot.write_text("host-boot")
                if change == "session":
                    self.receipt.record("model21800_20261029T120000Z", "OPTITRACK", self.config)
                elif change == "boot":self.boot.write_text("other-boot")
                elif change == "config":self.receipt.record(self.session, "KERNEL", LifecycleConfig(revision=2))
                else:self.receipt.path.write_text("{}")
                with self.assertRaises(ValueError):self.recover()
                self.assertTrue(self.receipt.path.exists())
        self.cleanup.assert_not_called()

    def test_receipt_change_after_cleanup_is_not_erased(self):
        self.cleanup.side_effect = lambda *_: self.receipt.path.write_text("changed")
        with self.assertRaisesRegex(RuntimeError, "receipt changed"):
            self.recover()
        self.assertEqual(self.receipt.path.read_text(), "changed")

    def test_owner_reappearing_during_cleanup_preserves_receipt(self):
        self.stopped.side_effect = [None, None, RuntimeError("owner returned")]
        with self.assertRaisesRegex(RuntimeError, "owner returned"):
            self.recover()
        self.cleanup.assert_called_once()
        self.assertTrue(self.receipt.path.exists())

    def test_kernel_recovery_keeps_original_mode(self):
        self.receipt.record(self.session, "KERNEL", self.config)
        self.recover()
        self.assertEqual(self.cleanup.call_args.args[3], "KERNEL")

    def test_stopped_owner_probe_rejects_active_unknown_and_remaining_cgroup(self):
        def result(**updates):
            data = dict(LoadState="loaded", ActiveState="inactive", MainPID="0", ControlPID="0", ControlGroup="")
            data.update(updates)
            return SimpleNamespace(returncode=0, stdout="\n".join(f"{k}={v}" for k,v in data.items()))
        run = Mock(return_value=result())
        require_stopped_owners(run, self.root)
        for load in ("not-found", "masked"):
            run.return_value = result(LoadState=load)
            require_stopped_owners(run, self.root)
        run.return_value = result(LoadState="not-found")
        run.return_value.returncode = 1
        require_stopped_owners(run, self.root)  # Complete explicit absent-unit state, not an unreadable query.
        run.return_value = SimpleNamespace(returncode=1, stdout="")
        with self.assertRaises(RuntimeError):require_stopped_owners(run, self.root)
        for update in (dict(ActiveState="active"), dict(MainPID="123"), dict(ControlPID="123"),
                       dict(LoadState="error"), dict(ControlGroup="/../escape")):
            run.return_value = result(**update)
            with self.assertRaises(RuntimeError):require_stopped_owners(run, self.root)
        group = self.root / "system.slice/test.service"
        group.mkdir(parents=True)
        (group / "cgroup.events").write_text("populated 1\nfrozen 0\n")
        run.return_value = result(ControlGroup="/system.slice/test.service")
        with self.assertRaisesRegex(RuntimeError, "populated"):
            require_stopped_owners(run, self.root)
        (group / "cgroup.events").write_text("populated 0\nfrozen 0\n")
        require_stopped_owners(run, self.root)
