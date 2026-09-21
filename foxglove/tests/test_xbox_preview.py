import importlib.util
from pathlib import Path
import unittest
import sys

sys.path.insert(0, str(Path(__file__).parents[1] / "laptop"))

spec = importlib.util.spec_from_file_location(
    "xbox", Path(__file__).parents[1] / "laptop/hope_xbox_preview.py"
)
xbox = importlib.util.module_from_spec(spec)
spec.loader.exec_module(xbox)


class XboxInputTests(unittest.TestCase):
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
