"""Tests for enctest.py: the linearity sweep against a rotor that follows
the field."""

import os
import sys
import unittest

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
sys.path.insert(0, ROOT)

from p4core.tests.fake import FakeNode                           # noqa: E402
from p4core.calibrate import CalibrationError                    # noqa: E402
from p4core.reporter import Reporter                             # noqa: E402

from p4core.actuator import enctest                                      # noqa: E402
from p4core.actuator.drivecal import CalError                            # noqa: E402


class FastReporter(Reporter):
    def __init__(self, answers=()):
        super().__init__()
        self.notes = []
        self.answers = list(answers)

    def note(self, text):
        self.notes.append(text)

    def sleep(self, seconds):
        self.check_abort()

    def confirm(self, question, default=False):
        return self.answers.pop(0) if self.answers else default


RES = 4096
POLES = 8

def linearity_node(error_cts=0.0, lag_cts=0.0, cog_cts=0.0, error_k=1,
                   ripple_cts=0.0, ripple_k=6):
    """Rotor follows Theta_e exactly; *error_cts* adds an *error_k*-per-rev
    encoder error (amplitude, counts); *lag_cts* a friction lag behind the
    field, *cog_cts* a 6-per-electrical-cycle cogging wobble and
    *ripple_cts* a *ripple_k*-per-rev rotor ripple (not field-locked)."""
    import math
    n = FakeNode()
    for name, sub, v in (('Calibration', 'e_zero', 0),
                         ('Calibration', 'e_polarity', 1),
                         ('EncoderConfig', 'Resolution', RES),
                         ('Calibration', 'poles', POLES),
                         ('Calibration', 'i_cal', 1000),
                         ('Calibration', 'i_peak', 5000),
                         ('Alpha', 'Bias', 2048 * 16),
                         ('Beta', 'Bias', 2048 * 16),
                         ('Alpha', 'Gainfactor', 4096),
                         ('Beta', 'Gainfactor', 4096),
                         ('Motor', 'ud', 0)):
        n.set(name, sub, v)
    n.set('StatusWord', None, 0x37)
    n.set('ControlWord', None, 0)
    n.set('SetModeOfOperation', None, 0)
    n.set('Theta_e', None, 0)
    cts_per_elec = RES * 2.0 / POLES
    st = dict(theta=0, turns=0.0, last=0)

    def on_theta(v):
        u = v % 65536
        frac = u / 65536.0
        prev = st['last']
        d = frac - prev
        if d < -0.5:
            d += 1.0
        st['turns'] += d
        st['last'] = frac
    n.hook_write('Theta_e', None, on_theta)
    # id reaches i_cal as soon as ud is non-zero.
    n.hook_read('Motor', 'id', lambda: 1000 * 1000 // 5000
                if n.get('Motor', 'ud') else 0)

    def true_pos():
        pos = st['turns'] * cts_per_elec
        mech = pos / RES * 2 * math.pi
        if st['turns'] > 0:
            pos += -lag_cts + cog_cts * math.sin(6 * 2 * math.pi * st['turns'])
            pos += ripple_cts * math.sin(ripple_k * mech)
        return pos, mech

    def sensor():
        pos, mech = true_pos()
        return int(round(pos + error_cts * math.sin(error_k * mech))) % RES

    def corrected():
        # The table takes error_cts out; the copy is raw + correction, so
        # just below 0 it reads negative instead of wrapping.
        pos, _ = true_pos()
        v = int(round(pos)) % RES
        return v - RES if v > RES - 3 else v
    n.hook_read('Encoder', 'RawPosition', sensor)
    n.hook_read('Encoder', 'EncPos', corrected)
    return n


class LinearityTest(unittest.TestCase):
    def test_corrected_reads_the_compensated_position(self):
        # 'corrected' sweeps 0x3012,2 (EncPos): the raw sensor carries a
        # 40-count error, the corrected copy has it taken out (and sits
        # past the wrap near 0, as the firmware's can).
        n = linearity_node(error_cts=40.0)
        self.assertFalse(enctest.test_encoder_linearity(
            n, FastReporter())['passed'])
        n = linearity_node(error_cts=40.0)
        res = enctest.test_encoder_linearity(n, FastReporter(),
                                             position='corrected')
        self.assertTrue(res['passed'], res['max_err'])
        self.assertEqual(res['position'], 'corrected')
        self.assertLess(res['rms_err'], 1.0)

    def test_a_fault_mid_sweep_raises_instead_of_reading_as_error(self):
        # An amplifier over-temperature drops the field: the rotor stops
        # and the rest would read as encoder error (node 2, 2026-10-15).
        n = linearity_node()
        n.set('StatusWord', None, 0x38)
        n.set('LastError', None, 0x4210)
        with self.assertRaises(CalError) as cm:
            enctest.test_encoder_linearity(n, FastReporter())
        self.assertIn('0x4210', str(cm.exception))
        self.assertEqual(n.get('SetModeOfOperation'), 0)

    def test_perfect_encoder_passes(self):
        n = linearity_node()
        res = enctest.test_encoder_linearity(n, FastReporter())
        self.assertTrue(res['passed'])
        self.assertEqual(res['pole_pairs'], 4)
        self.assertEqual(len(res['results']), 48 * 4 + 1)
        self.assertLess(res['rms_err'], 1.0)
        self.assertEqual(n.get('SetModeOfOperation'), 0)

    def test_runout_fails(self):
        # 20 counts once per rev = 20/1024*360 = 7 deg elec.
        res = enctest.test_encoder_linearity(linearity_node(20.0), FastReporter())
        self.assertFalse(res['passed'])
        self.assertGreater(abs(res['max_err']), 6)

    def test_lag_and_cogging_are_not_encoder_error(self):
        # a P4-42 at i_cal: friction lag and cogging, both locked to the
        # field, 20+ deg electrical before they are set aside
        res = enctest.test_encoder_linearity(
            linearity_node(lag_cts=20.0, cog_cts=20.0), FastReporter())
        self.assertTrue(res['passed'], res['max_err'])
        self.assertGreater(res['locked_rms'], 5.0)

    def test_gross_runout_is_gross(self):
        # 100 counts once per rev = 35 deg elec at 4 pole pairs
        res = enctest.test_encoder_linearity(linearity_node(100.0), FastReporter())
        self.assertTrue(res['gross'])
        res = enctest.test_encoder_linearity(linearity_node(20.0), FastReporter())
        self.assertFalse(res['gross'])

    def test_rotor_ripple_is_reported_not_judged(self):
        # 4 pole pairs: a 6-per-rev rotor ripple (slot-locked lag, not a
        # multiple of the pole pairs) of 20 counts = 7 deg elec is the
        # rotor's, not the encoder's.
        res = enctest.test_encoder_linearity(
            linearity_node(ripple_cts=20.0, ripple_k=6), FastReporter())
        self.assertTrue(res['passed'], res['max_err'])
        self.assertLess(abs(res['max_err']), 1.0)
        self.assertAlmostEqual(abs(res['ripple_max']), 7.0, delta=1.0)
        self.assertGreater(res['ripple_rms'], 4.0)

    def test_encoder_orders_below_the_pole_pairs_are_judged(self):
        # 3 per rev, under the 4 pole pairs: the encoder's.
        res = enctest.test_encoder_linearity(
            linearity_node(error_cts=20.0, error_k=3), FastReporter())
        self.assertFalse(res['passed'])
        self.assertAlmostEqual(abs(res['max_err']), 7.0, delta=1.0)
        self.assertLess(res['ripple_rms'], 1.0)

    def test_encoder_split(self):
        import math
        n, pp = 96, 4
        e = [3.0 + 2.0 * math.sin(2 * math.pi * i / n)
             + 5.0 * math.cos(2 * math.pi * 7 * i / n) for i in range(n)]
        enc, rip = enctest.encoder_split(e, pp)
        for i in range(n):
            self.assertAlmostEqual(enc[i], 2.0 * math.sin(2 * math.pi * i / n))
            self.assertAlmostEqual(rip[i],
                                   5.0 * math.cos(2 * math.pi * 7 * i / n))

    def test_template_polarity_is_not_encoder_error(self):
        n = linearity_node()
        n.set('Calibration', 'e_polarity', -1)
        res = enctest.test_encoder_linearity(n, FastReporter())
        self.assertTrue(res['passed'])
        self.assertEqual(res['direction'], 1)

    def test_wrong_pole_count_is_flagged(self):
        n = linearity_node()
        n.set('Calibration', 'poles', POLES // 2)
        res = enctest.test_encoder_linearity(n, FastReporter())
        self.assertFalse(res['poles_ok'])
        self.assertFalse(res['passed'])
        self.assertAlmostEqual(res['travel_ratio'], 0.5, delta=0.02)

    def test_uncalibrated_sense_is_refused(self):
        # an erased gain reads 0 A: the ramp would run ud to the top
        n = linearity_node()
        n.set('Beta', 'Gainfactor', 0)
        with self.assertRaisesRegex(Exception, 'not calibrated'):
            enctest.test_encoder_linearity(n, FastReporter())
        self.assertEqual(n.get('Motor', 'ud'), 0)
        self.assertEqual(n.get('SetModeOfOperation'), 0)

    def test_zero_i_cal_is_refused(self):
        # A blank board's winding probe has not run yet (i_cal 0): the sense
        # is calibrated (Bias/Gain), but there is no safe current to inject.
        n = linearity_node()
        n.set('Calibration', 'i_cal', 0)
        with self.assertRaisesRegex(Exception, 'winding probe'):
            enctest.test_encoder_linearity(n, FastReporter())
        self.assertEqual(n.get('Motor', 'ud'), 0)
        self.assertEqual(n.get('SetModeOfOperation'), 0)

    def test_stats(self):
        st = enctest.linearity_stats([(0, 0, 1, 1.0), (1, 1, 1, -6.0)])
        self.assertEqual(st['max_err'], -6.0)
        self.assertEqual(st['max_idx'], 1)
        self.assertFalse(st['passed'])

    def test_one_cycle_keeps_the_encoder_error(self):
        # One pole pair: one sample per field angle, so a field-locked mean
        # would cancel everything.  Only the constant lag comes out.
        spc = 8
        rows = [(0, 0, 1, 0.0)] + [(i, i * 45.0, 1, 2.0 + (6.0 if i == 3 else 0))
                                   for i in range(1, spc + 1)]
        st = enctest.linearity_stats(rows, steps_per_cycle=spc)
        self.assertAlmostEqual(st['max_err'], 6.0 - 6.0 / spc)
        self.assertFalse(st['passed'])

    def test_sdo_error_deenergises(self):
        n = linearity_node()
        n.hook_read('Encoder', 'RawPosition', lambda: 1 / 0)
        with self.assertRaises(ZeroDivisionError):
            enctest.test_encoder_linearity(n, FastReporter())
        self.assertEqual(n.get('Motor', 'ud'), 0)
        self.assertEqual(n.get('SetModeOfOperation'), 0)


class EncoderTestTest(unittest.TestCase):
    def test_is_p4core(self):
        n = FakeNode()
        n.set('SetModeOfOperation', None, 0)
        import itertools
        seq = itertools.count(0, 3)
        n.hook_read('PositionFeedback', None, lambda: next(seq))
        with self.assertRaises(CalibrationError):
            enctest.test_encoder(n, FastReporter(answers=[False]))


if __name__ == '__main__':
    unittest.main()
