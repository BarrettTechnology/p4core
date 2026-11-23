"""p4core.actuator.device: the motor thermistor reading, and a blank 0x3026."""

import os
import sys
import unittest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__),
                                                '..', '..')))

from p4core.actuator.device import P4Drive                              # noqa: E402
from p4core.tests.fake import FakeNode                                   # noqa: E402


def drive(therm_c10, config=(3010, 3950, 25, 10000), dsp_c=40):
    n = FakeNode(2)
    n.set('Motor', 'Therm', therm_c10)
    n.set('Amplifier', 'Temperature', dsp_c)
    if config is not None:
        for sub, v in zip((1, 2, 3, 4), config):
            n.values[(0x3026, sub)] = v
    d = P4Drive.__new__(P4Drive)
    d.node = n
    return d


class TestThermistor(unittest.TestCase):
    def test_configured_thermistor_is_read(self):
        d = drive(312)
        self.assertEqual(d.read_winding_temp_c(), (31.2, 'thermistor'))
        self.assertFalse(d.therm_unconfigured)

    def test_blank_config_is_no_thermistor(self):
        # A wiped P4-37: 0x3026 all 0, the K64's 0/0 reads exactly 0.0 C.
        d = drive(0, config=(0, 0, 0, 0))
        self.assertIsNone(d.read_motor_temp_c())
        self.assertTrue(d.therm_unconfigured)
        self.assertEqual(d.read_winding_temp_c(), (40.0, 'DSP'))

    def test_any_zero_term_is_unconfigured(self):
        for cfg in ((0, 3950, 25, 10000), (3010, 0, 25, 10000),
                    (3010, 3950, 25, 0)):
            self.assertIsNone(drive(0, config=cfg).read_motor_temp_c(), cfg)
        # a 0 C base temperature is a real setting, and so is 0.0 C
        self.assertEqual(drive(0, config=(3010, 3950, 0, 32650))
                         .read_motor_temp_c(), 0.0)

    def test_config_is_read_only_for_a_zero_reading(self):
        d = drive(312)
        d.read_motor_temp_c()
        self.assertFalse([k for k in d.node.reads if k[0] == 0x3026])

    def test_unreadable_config_falls_back_to_the_reading(self):
        # Firmware without 0x3026: judged by the reading alone, as before.
        self.assertEqual(drive(312, config=None).read_motor_temp_c(), 31.2)
        self.assertIsNone(drive(-400, config=None).read_motor_temp_c())


if __name__ == '__main__':
    unittest.main()
