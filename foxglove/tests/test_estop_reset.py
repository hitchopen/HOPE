"""Exercise the production reset callback without ROS or emergency backends."""
import ast
from pathlib import Path
import tempfile
import threading
from types import SimpleNamespace
import unittest

ROOT = Path(__file__).resolve().parents[1]
tree = ast.parse((ROOT / 'a3/hope_monitor.py').read_text())
method = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == '_reset_software_estop')
namespace = {'Bool': SimpleNamespace}
exec(compile(ast.Module(body=[method], type_ignores=[]), '<production reset callback>', 'exec'), namespace)
reset = namespace['_reset_software_estop']


class EstopResetTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.path = Path(self.temp.name) / 'latch'
        self.path.write_text('asserted')
        self.messages = []
        self.node = SimpleNamespace(
            _estop_service_lock=threading.Lock(), _control_lock=threading.Lock(),
            _estop_call_in_progress=False, _estop_latch_path=self.path,
            _control_estop_latched=True, _prepare_waiting_for_stand=True,
            _prepare_requires_calibration=True, _prepare_request_sequence=42,
            pub_estop_latched=SimpleNamespace(publish=self.messages.append))

    def test_explicit_reset_clears_persistence_and_pending_prepare_without_motion_backend(self):
        result = reset(self.node, None, SimpleNamespace())
        self.assertTrue(result.success)
        self.assertFalse(self.path.exists())
        self.assertFalse(self.node._control_estop_latched)
        self.assertFalse(self.node._prepare_waiting_for_stand)
        self.assertEqual(self.node._prepare_request_sequence, 0)
        self.assertFalse(self.messages[-1].data)
        self.assertIn('No motion started', result.message)
        # No Runner/vendor client exists on this node: any motion call fails this test.
        self.assertTrue(reset(self.node, None, SimpleNamespace()).success)

    def test_cannot_clear_an_outstanding_stop(self):
        self.node._estop_call_in_progress = True
        self.assertFalse(reset(self.node, None, SimpleNamespace()).success)
        self.assertTrue(self.path.exists())
        self.assertTrue(self.node._control_estop_latched)
        self.assertEqual(self.messages, [])

    def test_failed_unlink_retains_latch(self):
        self.path.unlink()
        self.path.mkdir()
        self.assertFalse(reset(self.node, None, SimpleNamespace()).success)
        self.assertTrue(self.node._control_estop_latched)
        self.assertEqual(self.messages, [])

    def test_only_attended_bridge_exposes_reset(self):
        self.assertIn('^/hope/safety/reset_software_estop$', (ROOT/'a3/bridge_params_control.yaml').read_text())
        self.assertNotIn('reset_software_estop', (ROOT/'a3/bridge_params.yaml').read_text())

if __name__ == '__main__':
    unittest.main()
