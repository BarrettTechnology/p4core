"""calibrate.write_i_cont / rescale_torque_units: on rated-unit firmware
(stm32 ecdc33c) i_cont sets the rated torque every per-mille is of."""

import math
import unittest

import fake  # noqa: F401  (puts p4core on the path)
from fake import FakeNode

from p4core import calibrate  # noqa: E402

GAINS = {(0x2381, 1): 0.0035, (0x2381, 2): 4.25e-5, (0x2381, 3): 0.0,
         (0x2381, 4): float('nan'), (0x6087, 0): 1000}


def node(rated_fw=True, i_cont=5327):
    n = FakeNode(8, dict(GAINS))
    n.values[(0x3011, 8)] = i_cont
    if rated_fw:
        n.values[(0x6075, 0)] = int(round(i_cont / 2 ** 0.5))   # r.m.s. (stage 3)
    return n


class TestWriteICont(unittest.TestCase):

    def test_rated_fw_keeps_the_torque_the_gains_command(self):
        n = node()
        self.assertAlmostEqual(calibrate.write_i_cont(n, 4000), 5327 / 4000.0)
        self.assertEqual(n.values[(0x3011, 8)], 4000)
        self.assertAlmostEqual(n.values[(0x2381, 1)], 0.0035 * 5327 / 4000, places=6)
        self.assertAlmostEqual(n.values[(0x2381, 2)], 4.25e-5 * 5327 / 4000, places=9)
        self.assertEqual(n.values[(0x6087, 0)], round(1000 * 5327 / 4000))

    def test_unset_rows_stay_unset(self):
        n = node()
        calibrate.write_i_cont(n, 4000)
        self.assertEqual(n.values[(0x2381, 3)], 0.0)
        self.assertTrue(math.isnan(n.values[(0x2381, 4)]))

    def test_older_fw_writes_i_cont_only(self):
        n = node(rated_fw=False)
        self.assertEqual(calibrate.write_i_cont(n, 4000), 1.0)
        self.assertEqual(n.values[(0x3011, 8)], 4000)
        self.assertEqual(n.values[(0x2381, 1)], 0.0035)

    def test_same_value_changes_nothing(self):
        n = node()
        self.assertEqual(calibrate.write_i_cont(n, 5327), 1.0)
        self.assertEqual(n.values[(0x2381, 1)], 0.0035)

    def test_set_limits_keep_their_torque_unset_stay(self):
        n = node()
        n.values.update({(0x6072, 0): 3000, (0x6073, 0): 0xFFFF,
                         (0x60E0, 0): 50000, (0x60E1, 0): 0xFFFF})
        calibrate.write_i_cont(n, 4000)
        self.assertEqual(n.values[(0x6072, 0)], round(3000 * 5327 / 4000))
        self.assertEqual(n.values[(0x6073, 0)], 0xFFFF)
        self.assertEqual(n.values[(0x60E0, 0)], 0xFFFE)   # held under unset
        self.assertEqual(n.values[(0x60E1, 0)], 0xFFFF)

    def test_limits_left_out_on_request(self):
        n = node()
        n.values[(0x6072, 0)] = 3000
        calibrate.rescale_torque_units(n, 0.5, limits=False)
        self.assertEqual(n.values[(0x6072, 0)], 3000)


if __name__ == '__main__':
    unittest.main()
