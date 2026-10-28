"""
Tests for winding.py: e_zero, e_polarity and the pole count from a sweep,
and the integer e_zero that centres the drive's truncated electrical cycle.
"""

import os
import sys
import unittest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__),
                                                '..', '..')))

from p4core.actuator import winding                                     # noqa: E402

F16 = winding.F16_CYCLE

def sweep(pp, zero, direction, res=4096, steps=16, travel=1.0, lag=0.0):
    """Sweep samples of a rotor mounted raw = zero + direction * count.

    *travel* scales how far the rotor follows the field (1 = exactly);
    *lag* is the friction lag in counts, against the motion.
    """
    cpe = res / float(pp)
    out = []
    theta = 0
    for d in (1, -1):
        for _ in range(pp * steps):
            theta += d * F16 // steps
            count = travel * theta / F16 * cpe - d * lag
            out.append((d, theta, zero + direction * count))
    return out


class TestSolve(unittest.TestCase):
    def test_zero_and_polarity(self):
        for pp, zero, direction in ((8, 100, 1), (4, 3000, -1), (2, 2047, -1)):
            r = winding.solve(sweep(pp, zero, direction), 4096, pp)
            cpe = 4096 // pp
            self.assertEqual(r['e_polarity'], direction)
            self.assertEqual(r['e_zero'], zero % cpe)
            self.assertAlmostEqual(r['travel_ratio'], 1.0, places=6)
            self.assertAlmostEqual(r['commutation_error_deg'], 0.0, places=6)

    def test_friction_cancels_between_directions(self):
        r = winding.solve(sweep(8, 100, 1, lag=5), 4096, 8)
        self.assertEqual(r['e_zero'], 100)
        self.assertAlmostEqual(abs(r['friction_split_deg']),
                               2 * 5 * 360 / 512.0, places=3)

    def test_wrong_pole_count_rejected(self):
        with self.assertRaises(ValueError):
            winding.solve(sweep(7, 50, 1), 4096, 8)

    def test_stall_rejected(self):
        with self.assertRaises(ValueError):
            winding.solve(sweep(8, 50, 1, travel=0.5), 4096, 8)

    def test_non_dividing_pole_count_is_centred(self):
        # 10 pp on 4096 counts: the drive's cycle is 409, not 409.6.  The
        # physical zero 90.4 needs e_zero 99 for the drive's frame to sit
        # on the rotor's on average (what the twin's coast measures).
        r = winding.solve(sweep(10, 500, 1), 4096, 10)
        self.assertEqual(r['e_zero_physical'], 90)
        self.assertEqual(r['e_zero'], 99)
        self.assertGreater(r['commutation_error_deg'], 2.0)


class TestFirmwareEZero(unittest.TestCase):
    def test_dividing_is_the_rounded_zero(self):
        r = winding.firmware_e_zero(952.0, -1, 4096, 4)
        self.assertEqual(r['e_zero'], 952)
        self.assertAlmostEqual(r['error_peak_deg'], 0.0, places=9)

    def test_seven_pole_pairs(self):
        # Node 127: 585-count cycle, one count lost; the coast put q at 65.2.
        r = winding.firmware_e_zero(63.7, 1, 4096, 7)
        self.assertEqual(r['e_zero'], 65)
        self.assertLess(abs(r['mean_error_deg']), 0.5)

    def test_mean_error_under_a_count(self):
        for pp in (3, 5, 10, 21):
            r = winding.firmware_e_zero(40.3, 1, 4096, pp)
            self.assertLess(abs(r['mean_error_deg']), 360.0 * pp / 4096)


class TestFirmwareEZeroExact(unittest.TestCase):
    """Firmware with 0x3013,8: the angle is pp * (raw - e_zero) mod res."""

    @staticmethod
    def _frame_error_deg(ez, exact, pol, res, pp):
        # The drive's angle minus the true one, at any raw (it is constant).
        raw = 1234
        fw = (pp * pol * (raw - ez)) % res / float(res)
        true = (pp * pol * (raw - exact)) % res / float(res)
        return ((fw - true + 0.5) % 1.0 - 0.5) * 360.0

    def test_finer_than_a_count(self):
        # 7 and 4096 share no factor: some cycle lands within 1/14 count.
        for exact in (63.7, 50.3, 1000.49):
            for pol in (1, -1):
                r = winding.firmware_e_zero(exact, pol, 4096, 7,
                                            exact_angle=True)
                self.assertEqual(r['error_peak_deg'], 0.0)
                err = self._frame_error_deg(r['e_zero'], exact, pol, 4096, 7)
                self.assertAlmostEqual(err, r['mean_error_deg'], places=6)
                self.assertLessEqual(abs(err), 360.0 * 7 / 4096 / 7 / 2 + 1e-9)

    def test_dividing_is_the_rounded_zero_mod_a_cycle(self):
        r = winding.firmware_e_zero(952.0, -1, 4096, 4, exact_angle=True)
        self.assertEqual(r['e_zero'] % 1024, 952)
        self.assertAlmostEqual(r['mean_error_deg'], 0.0, places=9)

    def test_solve_passes_it_through(self):
        cpe = 4096 / 10.0
        samples = []
        for d in (1, -1):
            for k in range(0, 10 * 16):
                t = k * 65536 // 16 if d > 0 else (160 - k) * 65536 // 16
                samples.append((d, t, 500.3 + t / 65536.0 * cpe))
        r = winding.solve(samples, 4096, 10, exact_angle=True)
        self.assertTrue(r['exact_angle'])
        self.assertEqual(r['commutation_error_deg'], 0.0)
        self.assertLess(abs(self._frame_error_deg(r['e_zero'], 500.3, 1,
                                                  4096, 10)), 0.45)


if __name__ == '__main__':
    unittest.main()
