"""Kernel-independent worker discovery without ROS or real process signals."""
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from operations_processes import child_pids


class ChildProcessTests(unittest.TestCase):
    def setUp(self):
        temp = tempfile.TemporaryDirectory(prefix="operations-proc-")
        self.addCleanup(temp.cleanup)
        self.root = Path(temp.name)
        self.process(100, 1)

    def process(self, pid, ppid, name="worker"):
        directory = self.root / str(pid)
        directory.mkdir()
        (directory / "stat").write_text(f"{pid} ({name}) S {ppid} 0 0 0\n")

    def test_native_children_file_is_used_when_present(self):
        directory = self.root / "100/task/100"
        directory.mkdir(parents=True)
        (directory / "children").write_text("103 101\n")
        self.assertEqual(child_pids(100, self.root), [101, 103])

    def test_fallback_filters_ppid_and_handles_complex_process_names(self):
        self.process(103, 100, "worker ) with ( spaces")
        self.process(101, 100)
        self.process(104, 101)  # A grandchild must not count as a direct child.
        self.process(105, 1)
        (self.root / "sys").mkdir()
        (self.root / "106").mkdir()  # Process disappeared before stat was read.
        (self.root / "107").mkdir()
        (self.root / "107/stat").write_text("malformed\n")
        self.assertEqual(child_pids(100, self.root), [101, 103])

    def test_parent_without_children_returns_empty(self):
        self.assertEqual(child_pids(100, self.root), [])

    def test_missing_parent_is_not_reported_as_childless(self):
        with self.assertRaises(ProcessLookupError):
            child_pids(999, self.root)

    def test_permission_error_is_not_hidden_by_fallback(self):
        with patch.object(Path, "read_text", side_effect=PermissionError("unreadable")):
            with self.assertRaises(PermissionError):
                child_pids(100, self.root)
