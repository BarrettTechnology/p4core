"""
Tests for slope.py on a simulated puck: the measured alpha/beta current is
the true rotating current plus an offset a0 + k*|I|, so the slope fit must
recover k and a0, store them in Q4.12, and the fold must write a0/b0.
"""

import math
import os
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.abspath(os.path.join(HERE, '..', '..')))

from p4core.tests.fake import FakeNode                          # noqa: E402
from p4core.cia402 import MODE_IDLE                             # noqa: E402
from p4core.reporter import Reporter                            # noqa: E402

from p4core.actuator import slope                                       # noqa: E402
from p4core.actuator.drivecal import CalError                           # noqa: E402

SAVE = (0x1010, 4)            # Save.Single


class FastReporter(Reporter):
    def __init__(self):
        super().__init__()
        self.notes = []

    def note(self, text):
        self.notes.append(text)

    def sleep(self, seconds):
        self.check_abort()


class SimPuck(FakeNode):
    """ud -> |I| = MA_PER_UD * ud along Theta_e; the sense adds an offset
    (a0 + kA*|I|, b0 + kB*|I|) mA in alpha/beta."""
    MA_PER_UD = 0.2
    I_PEAK = 20000

    def __init__(self, a0=10.0, b0=5.0, ka=0.05, kb=0.025, gain=4096,
                 fw=b'4.4.0', offset_reg=True):
        super().__init__()
        self.a0, self.b0, self.ka, self.kb = a0, b0, ka, kb
        self.values[(0x100A, 0)] = fw
        for name, sub, v in (('Calibration', 'i_peak', self.I_PEAK),
                             ('Calibration', 'i_cal', 1000),
                             ('Alpha', 'Gainfactor', gain),
                             ('Beta', 'Gainfactor', gain),
                             ('Alpha', 'Bias', 32768),
                             ('Beta', 'Bias', 32768),
                             ('Motor', 'ud', 0), ('Motor', 'uq', 0),
                             ('Encoder', 'RawPosition', 1234),
                             ('Amp', 'MaxSettlingTime', 800)):
            self.set(name, sub, v)
        self.values[self.key('Theta_e')] = 0
        self.values[(0x3008, 7)] = 99
        self.values[(0x3009, 7)] = 99
        if offset_reg:
            self.values[(0x3008, 8)] = 7
            self.values[(0x3009, 8)] = 7
        self.hook_read('Motor', 'id', lambda: self._dq()[0])
        self.read_hooks[self.key('CurrentFeedback')] = lambda: self._dq()[1]
        self.hook_read('Alpha', 'Filtered', lambda: self._filtered(0))
        self.hook_read('Beta', 'Filtered', lambda: self._filtered(1))

    def _ab(self):
        i = self.MA_PER_UD * self.get('Motor', 'ud')
        th = self.values[self.key('Theta_e')] / 32768.0 * math.pi
        return (i * math.cos(th) + self.a0 + self.ka * i,
                i * math.sin(th) + self.b0 + self.kb * i, th)

    def _dq(self):
        a, b, th = self._ab()
        i_d = a * math.cos(th) + b * math.sin(th)
        i_q = -a * math.sin(th) + b * math.cos(th)
        return i_d / self.I_PEAK * 1000.0, i_q / self.I_PEAK * 1000.0

    def _filtered(self, ch):
        bias = self.get('Alpha' if ch == 0 else 'Beta', 'Bias')
        return bias + slope.A_SENS * self._ab()[ch]

    def saves(self):
        return [v for k, v in self.writes if k == SAVE]


class FitTests(unittest.TestCase):
    def test_fit_line(self):
        k, c, r2 = slope.fit_line([1, 2, 3], [3, 5, 7])
        self.assertAlmostEqual(k, 2.0)
        self.assertAlmostEqual(c, 1.0)
        self.assertAlmostEqual(r2, 1.0)

    def test_fit_slope_gates_direct_drive(self):
        # |I| frozen across levels, direction all over the place: no effect.
        rows = [(150, 5, 0), (151, 0, 5), (150, -5, 0), (152, 0, -5)]
        fit = slope.fit_slope(rows, 4096, 4096, 1000)
        self.assertFalse(fit['effect_ok'])
        self.assertFalse(fit['store'])
        self.assertTrue(fit['rejected'])

    def test_fit_slope_gates_bad_gain(self):
        rows = [(100 * i, 10 + 0.05 * 100 * i, 5 + 0.025 * 100 * i)
                for i in range(1, 6)]
        self.assertTrue(slope.fit_slope(rows, 4096, 4096, 1000)['store'])
        self.assertFalse(slope.fit_slope(rows, 100, 4096, 1000)['store'])

    def test_fit_slope_zeros_a_noisy_fit(self):
        # Direction fixed (off_b = 0.5*off_a at every level, so effect_ok's
        # angle spread is 0) but the magnitude bounces around with no trend
        # against |I| (R^2 ~0.04): gain/slope/effect all pass, but the fit
        # itself is noise, not a real term -- nodes 3/4 saw this run to run.
        mi = [100, 200, 300, 400, 500]
        off_a = [15, 2, 18, 3, 20]
        rows = [(m, a, 0.5 * a) for m, a in zip(mi, off_a)]
        fit = slope.fit_slope(rows, 4096, 4096, 1000)
        self.assertTrue(fit['gain_ok'])
        self.assertTrue(fit['slope_ok'])
        self.assertTrue(fit['effect_ok'])
        self.assertFalse(fit['r2_ok'])
        self.assertTrue(fit['zeroed'])
        self.assertTrue(fit['store'])          # a pass, with 0 stored
        self.assertEqual(fit['kA'], 0.0)
        self.assertEqual(fit['kB'], 0.0)
        self.assertNotEqual(fit['kA_fit'], 0.0)  # the raw fit kept for the log


class SlopeCalTests(unittest.TestCase):
    def test_recovers_and_stores_slope(self):
        node = SimPuck()
        res = slope.calibrate_current_slope(node, FastReporter(),
                                            force_sdo=True)
        self.assertTrue(res['stored'])
        self.assertIsNone(res['skipped'])
        self.assertFalse(res['pdo'])
        # 10 angles of 6553.6 counts round to integers: not exactly uniform
        self.assertAlmostEqual(res['kA'], 0.05, places=4)
        self.assertAlmostEqual(res['kB'], 0.025, places=4)
        self.assertAlmostEqual(res['a0'], 10.0, places=1)
        self.assertAlmostEqual(res['b0'], 5.0, places=1)
        self.assertEqual(len(res['rows']), len(slope.SLOPE_FRACS))
        self.assertEqual(node.values[(0x3008, 7)], round(0.05 * 4096))
        self.assertEqual(node.values[(0x3009, 7)], round(0.025 * 4096))
        self.assertEqual(res['written'], {(0x3008, 7): 205, (0x3009, 7): 102})
        # cleared first (slope + offset register), then stored, each saved
        self.assertEqual(node.saves().count((0x3008 << 8) | 7), 2)
        self.assertEqual(node.values[(0x3008, 8)], 0)
        self.assertEqual(node.get('Motor', 'ud'), 0)
        self.assertEqual(node.values[node.key('SetModeOfOperation')], MODE_IDLE)

    def test_quick_has_four_levels(self):
        res = slope.calibrate_current_slope(SimPuck(), FastReporter(),
                                            force_sdo=True, quick=True)
        self.assertEqual(len(res['rows']), 4)
        self.assertTrue(res['stored'])

    def test_bad_gain_clears_and_does_not_drive(self):
        node = SimPuck(gain=100)
        res = slope.calibrate_current_slope(node, FastReporter(),
                                            force_sdo=True)
        self.assertEqual(res['skipped'], 'gainfactor')
        self.assertFalse(res['stored'])
        self.assertEqual(node.values[(0x3008, 7)], 0)
        self.assertNotIn((node.key('Motor', 'ud'), 2000), node.writes)

    def test_old_firmware_skips(self):
        node = SimPuck(fw=b'4.3.9')
        res = slope.calibrate_current_slope(node, FastReporter())
        self.assertEqual(res, dict(stored=False, skipped='firmware'))
        self.assertEqual(node.writes, [])

    def test_failure_deenergises_and_raises(self):
        node = SimPuck()
        calls = [0]

        def broken():
            calls[0] += 1
            if calls[0] > 20:
                raise RuntimeError('bus gone')
            return 1234
        node.hook_read('Encoder', 'RawPosition', broken)
        with self.assertRaises(RuntimeError):
            slope.calibrate_current_slope(node, FastReporter(), force_sdo=True)
        self.assertEqual(node.get('Motor', 'ud'), 0)
        self.assertEqual(node.values[node.key('SetModeOfOperation')], MODE_IDLE)
        self.assertEqual(node.values[(0x3008, 7)], 0)    # left cleared


class FoldTests(unittest.TestCase):
    def test_register_path(self):
        node = SimPuck()
        res = slope.fold_baseline_offset(
            node, dict(stored=True, a0=12.4, b0=-3.6), FastReporter())
        self.assertEqual(res['path'], 'register')
        self.assertEqual(node.values[(0x3008, 8)], 12)
        self.assertEqual(node.values[(0x3009, 8)], -4)
        self.assertIn((0x3008 << 8) | 8, node.saves())
        self.assertIn((0x3009 << 8) | 8, node.saves())

    def test_bias_path_without_register(self):
        node = SimPuck(offset_reg=False)
        node.values[(0x3008, 5)] = 1000   # shunt
        node.values[(0x3008, 4)] = 20000  # gain
        res = slope.fold_baseline_offset(
            node, dict(stored=True, a0=10.0, b0=0.0), FastReporter())
        self.assertEqual(res['path'], 'bias')
        self.assertLess(node.get('Alpha', 'Bias'), 32768)
        self.assertEqual(node.get('Beta', 'Bias'), 32768)
        self.assertIn((0x3008 << 8) | 3, node.saves())

    def test_refuses_a_failed_sanity_gate(self):
        node = SimPuck()
        with self.assertRaises(CalError):
            slope.fold_baseline_offset(node, dict(stored=False, a0=1, b0=1))
        with self.assertRaises(CalError):
            slope.fold_baseline_offset(node, dict(
                stored=False, gain_ok=False, slope_ok=True, a0=1, b0=1,
                a0_flat=1, b0_flat=1))
        with self.assertRaises(CalError):
            slope.fold_baseline_offset(node, None)
        self.assertEqual(node.writes, [])

    def test_unstored_slope_folds_the_mean_offset(self):
        node = SimPuck()
        res = slope.fold_baseline_offset(node, dict(
            stored=False, gain_ok=True, slope_ok=True, effect_ok=False,
            kA=0.02, kB=0.0, a0=-60.0, b0=5.0, a0_flat=-20.4, b0_flat=7.6),
            FastReporter())
        self.assertEqual(res['basis'], 'mean')
        self.assertEqual(node.values[(0x3008, 8)], -20)
        self.assertEqual(node.values[(0x3009, 8)], 8)

    def test_zeroed_slope_folds_the_mean_offset(self):
        # kA/kB stored as 0: the intercept would leave kA_fit*|I| behind.
        a0, b0, basis = slope.baseline_of(dict(
            stored=True, zeroed=True, kA=0.0, kB=0.0, a0=-60.0, b0=5.0,
            a0_flat=-20.0, b0_flat=6.0))
        self.assertEqual((a0, b0, basis), (-20.0, 6.0, 'mean'))

    def test_stored_slope_folds_the_intercept(self):
        self.assertEqual(slope.baseline_of(dict(
            stored=True, kA=0.01, kB=0.0, a0=-60.0, b0=5.0, a0_flat=-20.0,
            b0_flat=6.0)), (-60.0, 5.0, 'fit'))

    def test_fit_reports_the_mean_offsets(self):
        rows = [(1000.0, 10.0, -2.0), (2000.0, 14.0, -4.0),
                (3000.0, 18.0, -6.0)]
        fit = slope.fit_slope(rows, 4096, 4096, 3000)
        self.assertAlmostEqual(fit['a0_flat'], 14.0)
        self.assertAlmostEqual(fit['b0_flat'], -4.0)


if __name__ == '__main__':
    unittest.main()
