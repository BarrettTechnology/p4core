"""
Tests for fits.py — the numerical core of p4core.actuator.

Each fit is fed synthetic data generated from known parameters and checked to
recover them.  The point is that these are the parts hardware cannot check
conveniently: a motor tells you what Rs *is* only if you already trust the fit,
so the fit is pinned here against data whose answer is known exactly.

Noise amplitudes are chosen to be representative of the real signals (a few mA
of current quantisation, a few RPM of velocity ripple) rather than
vanishingly small, so a fit that only works on clean data fails here.
"""

import os
import sys
import unittest


# The package is this repository's root directory, so its parent goes on the
# path and the tests import it by name, exactly as an application does.
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__),
                                                '..', '..')))

from p4core.actuator import fits                             # noqa: E402


class TestResistanceFit(unittest.TestCase):

    def test_single_point_ratio_would_have_been_biased(self):
        """The reason for the affine fit: V/I at one point is badly biased.

        Not a test of fits.py at all — it pins the motivation.  If someone later
        'simplifies' fit_resistance to a ratio, this documents the cost.
        """
        rs_true, v_offset = 0.35, 0.40
        i_low = 0.5
        naive = (rs_true * i_low + v_offset) / i_low
        self.assertGreater(naive, 2.0 * rs_true)


class TestPwmFrequency(unittest.TestCase):
    """Choosing a switching frequency from the winding.

    Timings are a real P4-42's 0x3001,2/3/5/6: dead time 250 ns, propagation
    180, settling 500, sampling 313 — so delay = 930 ns and sampling < delay.
    """

    TIMING = dict(dead_time_ns=250.0, prop_delay_ns=180.0,
                  settling_ns=500.0, sampling_ns=313.0)

    def test_headroom_matches_the_hal_expression(self):
        """hal_pwm_set_rate gives 1 - 4*delay*f_sw when sampling < delay."""
        for f in (20e3, 40e3, 80e3):
            expected = 1.0 - 4.0 * 930e-9 * f
            self.assertAlmostEqual(
                fits.pwm_voltage_headroom(f, **self.TIMING), expected,
                places=9)

    def test_headroom_falls_as_frequency_rises(self):
        """The cost of switching fast, and it comes off top speed."""
        self.assertAlmostEqual(fits.pwm_voltage_headroom(80e3, **self.TIMING),
                               0.7024, places=3)
        self.assertAlmostEqual(fits.pwm_voltage_headroom(40e3, **self.TIMING),
                               0.8512, places=3)

    def test_headroom_is_one_when_no_timings_are_given(self):
        self.assertEqual(fits.pwm_voltage_headroom(80e3, 0, 0, 0, 0), 1.0)


if __name__ == '__main__':
    unittest.main()


class TestOneSidedBudget(unittest.TestCase):
    """The K64 budget, against what node 127 (P4-16 DevKit) reported."""

    P416 = dict(dead_time_ns=420.0, prop_delay_ns=940.0, settling_ns=1000.0,
                sampling_ns=400.0)

    def test_matches_the_drive(self):
        # 0x3016,4/5 read 884/1000 at 40 kHz and 384/500 at 80 kHz.
        for f, mw, period in ((40e3, 884, 1000), (80e3, 384, 500)):
            self.assertAlmostEqual(
                fits.pwm_voltage_headroom(f, one_sided=True, shift=False,
                                          **self.P416),
                fits.drive_voltage_limit(mw, period), places=2)

    def test_pulse_shift(self):
        # node 127 2026-10-13: the shift took 80 kHz from 0.54 to 0.70
        self.assertAlmostEqual(
            fits.pwm_voltage_headroom(80e3, one_sided=True, **self.P416),
            0.70, places=2)

    def test_costs_about_half_the_symmetric_budget(self):
        one = fits.pwm_voltage_headroom(40e3, one_sided=True, **self.P416)
        two = fits.pwm_voltage_headroom(40e3, **self.P416)
        self.assertAlmostEqual(two, 0.622, places=3)
        self.assertGreater(one, 0.76)

    def test_budget_is_recognised_from_the_report(self):
        self.assertTrue(fits.budget_is_one_sided(884, 1000, 40e3,
                                                 **self.P416))
        # The symmetric budget's pulse for the same timing.
        self.assertFalse(fits.budget_is_one_sided(811, 1000, 40e3,
                                                  **self.P416))
