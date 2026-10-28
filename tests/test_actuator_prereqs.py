"""prereqs.py against steps.STEPS: every step is declared, and the steps
that drive measured current come after the current sense is calibrated."""
import os
import sys
import unittest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__),
                                                '..', '..')))
from p4core.actuator import prereqs, steps                               # noqa: E402


class TestPrereqs(unittest.TestCase):
    def test_every_step_is_declared(self):
        self.assertEqual(set(prereqs.DEPS), set(steps.STEP_KEYS))

    def test_closed_loop_steps_follow_the_isense_gate(self):
        gate = max(steps.STEP_KEYS.index('bias'), steps.STEP_KEYS.index('gain'))
        for i, key in enumerate(steps.STEP_KEYS):
            if key not in steps.OPEN_LOOP_STEPS:
                self.assertGreater(i, gate, key)


if __name__ == '__main__':
    unittest.main()
