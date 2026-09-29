"""DDS recovery waits for the current RPC and never changes Runner mode."""
import sys
import threading
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
sys.path.insert(0, str(Path(__file__).parents[1] / 'a3'))
try:
    from hope_command_proxy import HopeCommandProxy
except ModuleNotFoundError:
    HopeCommandProxy = None

@unittest.skipIf(HopeCommandProxy is None, 'requires ROS environment')
class CommandProxyNetworkTests(unittest.TestCase):
    def setUp(self):
        self.initial = ((5, 'usb0', '10.42.20.10'),)
        self.node = SimpleNamespace(_dds_interfaces=self.initial, _action_lock=threading.Lock())

    def poll(self, signature):
        with patch('hope_command_proxy.read_ipv4_interface_signature', return_value=signature):
            HopeCommandProxy._poll_dds_interfaces(self.node)

    def test_replug_same_ip_new_interface_restarts_dds(self):
        self.poll(self.initial)
        with self.assertRaisesRegex(RuntimeError, 'reconnecting command proxy DDS'):
            self.poll(((9, 'usb0', '10.42.20.10'),))

    def test_inflight_rpc_finishes_before_restart(self):
        self.node._action_lock.acquire()
        changed = self.initial + ((6, 'eth1', '10.42.10.10'),)
        self.poll(changed)
        self.assertEqual(self.node._dds_interfaces, self.initial)
        self.node._action_lock.release()
        with self.assertRaises(RuntimeError):
            self.poll(changed)

    def test_failed_probe_does_not_restart(self):
        self.poll(None)
        self.assertEqual(self.node._dds_interfaces, self.initial)

if __name__ == '__main__':
    unittest.main()
