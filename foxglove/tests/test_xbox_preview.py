import importlib.util
from pathlib import Path
import unittest
import sys
import io
import json
from types import SimpleNamespace
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).parents[1] / "laptop"))

spec = importlib.util.spec_from_file_location(
    "xbox", Path(__file__).parents[1] / "laptop/hope_xbox_preview.py"
)
xbox = importlib.util.module_from_spec(spec)
spec.loader.exec_module(xbox)


class XboxInputTests(unittest.TestCase):
    def test_idle_stream_is_10hz_and_active_stream_keeps_every_tick(self):
        idle, active = xbox.StreamCadence(), xbox.StreamCadence()
        self.assertEqual(sum(idle.ready(i * .02, (False,)) for i in range(50)), 10)
        self.assertEqual(sum(active.ready(i * .02, (True,), active=True)
                             for i in range(50)), 50)

    def test_disable_disconnect_rearm_and_mode_edges_bypass_idle_gate(self):
        gate = xbox.StreamCadence()
        for i, key in enumerate(((1, True, True, False, 'TELEOP'),
                                 (1, True, False, False, 'TELEOP'),
                                 (1, False, False, False, 'TELEOP'),
                                 (1, True, False, True, 'PD_STAND'),
                                 (2, True, False, True, 'PD_STAND'))):
            self.assertTrue(gate.ready(i * .02, key))
        self.assertFalse(gate.ready(.09, key))

    def test_removed_device_is_reopened_without_replaying_held_buttons(self):
        clock = [0.]
        devices = []
        codes = SimpleNamespace(EV_ABS=3, ABS_X=0, ABS_Y=1, ABS_RX=3, ABS_Z=2, BTN_TL=310, BTN_TR=311)
        class Device:
            name = 'Microsoft Xbox Series S|X Controller'
            info = SimpleNamespace(vendor=0x045e, product=0x0b12)
            def __init__(self, path):
                self.removed = not devices
                self.closed = False
                devices.append(self)
            def capabilities(self):
                return {3: [0, 1, 2, 3]}
            def absinfo(self, code):
                if self.removed:
                    raise OSError('device removed')
                return SimpleNamespace(value=1023 if code == 2 else 0, min=0 if code == 2 else -32768, max=1023 if code == 2 else 32767)
            def active_keys(self):
                return [304]  # Held A must not become an action on reconnect.
            def close(self):
                self.closed = True
        evdev = SimpleNamespace(InputDevice=Device, ecodes=codes, list_devices=lambda: ['/dev/input/event24'])
        args = SimpleNamespace(ros=False, control=False, seconds=.1, deadzone=.12, forward=.3, backward=.2, lateral=.15, yaw=.4)
        output = io.StringIO()
        with patch.dict(sys.modules, evdev=evdev), patch.object(xbox.time, 'monotonic', side_effect=lambda: clock[0]), patch.object(xbox.time, 'sleep', side_effect=lambda duration: clock.__setitem__(0, clock[0]+max(duration,.001))), patch('sys.stdout', output):
            xbox.run(args)
        packets = [json.loads(line) for line in output.getvalue().splitlines()]
        self.assertEqual(len(devices), 2)
        self.assertTrue(all(device.closed for device in devices))
        self.assertFalse(packets[0]['connected'])
        self.assertTrue(packets[1]['connected'])
        self.assertNotEqual(packets[0]['session'], packets[1]['session'])
        self.assertFalse(packets[1]['enabled'])
        self.assertFalse(packets[1]['action_pending'])

    def test_network_replug_rebuilds_transport_even_with_same_interface_name(self):
        recovery = xbox.ConnectionRecovery(10.)
        self.assertIsNone(recovery.reason(11., ('usb0', '5', '10.42.20.1'), 11.))
        self.assertIsNotNone(recovery.reason(12., ('usb0', '9', '10.42.20.1'), 12.))

    def test_missing_safety_stream_recovers_without_enabling_input(self):
        recovery = xbox.ConnectionRecovery(10.)
        self.assertIsNone(recovery.reason(17., None, 0.))
        self.assertIsNotNone(recovery.reason(19., None, 0.))
        self.assertIsNone(recovery.reason(25., None, 24.))
        self.assertIsNotNone(recovery.reason(33., None, 24.))

    def test_reconnect_never_replays_held_enable(self):
        latch = xbox.EnableLatch()
        self.assertFalse(latch.sample(True, True, [0, 0, 0])[0])
        self.assertFalse(latch.sample(True, False, [0, 0, 0])[0])
        self.assertTrue(latch.sample(True, True, [0, 0, 0])[0])
        self.assertTrue(latch.sample(True, True, [0.4, 0, 0])[0])
        self.assertFalse(latch.sample(False, True, [0.4, 0, 0])[0])
        self.assertFalse(latch.sample(True, True, [0, 0, 0])[0])
        self.assertFalse(latch.sample(True, False, [0, 0, 0])[0])
        self.assertTrue(latch.sample(True, True, [0, 0, 0])[0])

    def test_enable_requires_center_and_new_press(self):
        latch = xbox.EnableLatch()
        latch.sample(True, False, [0, 0, 0])
        self.assertFalse(latch.sample(True, True, [0.2, 0, 0])[0])
        self.assertFalse(latch.sample(True, True, [0, 0, 0])[0])
        latch.sample(True, False, [0, 0, 0])
        self.assertTrue(latch.sample(True, True, [0, 0, 0])[0])
        self.assertFalse(latch.sample(True, False, [0.2, 0, 0])[0])

    def test_axis_noise_and_sign(self):
        self.assertEqual(xbox.normalize_axis(400, -32768, 32767, 0.12), 0)
        self.assertEqual(xbox.normalize_axis(-32768, -32768, 32767, 0.12), -1)
        self.assertEqual(xbox.normalize_axis(32767, -32768, 32767, 0.12), 1)

    def test_invalid_input_disarms(self):
        latch = xbox.EnableLatch()
        latch.sample(True, False, [0, 0, 0])
        latch.sample(True, True, [0, 0, 0])
        self.assertFalse(latch.sample(True, True, [float("nan"), 0, 0])[0])
        self.assertFalse(latch.sample(True, True, [0, 0, 0])[0])


if __name__ == "__main__":
    unittest.main()
