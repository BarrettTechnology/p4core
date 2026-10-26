"""ezero.solve(): e_zero from a stepped field sweep over one revolution."""
import math
import os
import sys
import unittest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))
from p4core import ezero  # noqa: E402

RES, PP, STEPS = 4096, 7, 16
CPE = RES / float(PP)


def sweep(e_zero, polarity=1, friction=0.0, ripple=0.0, ratio=1.0, raw0=1000):
    """Synthetic samples: the rotor settles at the field, lagging it by
    `friction` counts, with a 2-per-rev encoder error of `ripple` counts.
    `ratio` scales rotor travel (a wrong pole count or a stall)."""
    out = []
    theta = 0
    step = ezero.F16_CYCLE // STEPS
    for direction in (1, -1):
        for _ in range(PP * STEPS):
            theta += direction * step
            ideal = e_zero + polarity * theta / float(ezero.F16_CYCLE) * CPE * ratio
            mech = ideal - polarity * direction * friction
            raw = mech + ripple * math.sin(2 * 2 * math.pi * mech / RES) + raw0
            out.append((direction, theta, raw))
    return out, raw0


class TestSolve(unittest.TestCase):
    def check(self, want, **kw):
        samples, raw0 = sweep(want, **kw)
        r = ezero.solve(samples, RES, PP)
        got = (r['e_zero_exact'] - raw0) % CPE
        self.assertAlmostEqual(ezero.wrap(got - want % CPE, CPE), 0.0, delta=0.5)
        return r

    def test_exact_zero(self):
        r = self.check(50.0)
        self.assertEqual(r['e_polarity'], 1)
        self.assertAlmostEqual(r['spread_max_deg'], 0.0, places=6)

    def test_friction_cancels_between_directions(self):
        r = self.check(120.0, friction=15.0)
        self.assertAlmostEqual(abs(r['friction_split_deg']), 2 * 15.0 * 360 / CPE, delta=0.5)

    def test_encoder_ripple_averages_out(self):
        # 11 counts of 2-per-rev error is +-7 deg electrical at any single point
        r = self.check(300.0, ripple=11.0)
        self.assertGreater(r['spread_max_deg'], 5.0)

    def test_negative_polarity(self):
        r = self.check(200.0, polarity=-1, friction=5.0, ripple=5.0)
        self.assertEqual(r['e_polarity'], -1)

    def test_zero_near_the_wrap(self):
        self.check(CPE - 2.0, friction=10.0)
        self.check(1.0, friction=10.0)

    def test_stall_is_rejected(self):
        samples, _ = sweep(50.0, ratio=0.05)
        with self.assertRaisesRegex(ezero.EZeroError, 'Stalled'):
            ezero.solve(samples, RES, PP)

    def test_wrong_pole_count_is_rejected(self):
        samples, _ = sweep(50.0, friction=10.0, ripple=11.0)   # a 14-pole motor...
        with self.assertRaisesRegex(ezero.EZeroError, r'suggests 14\.0 poles, not 16'):
            ezero.solve(samples, RES, 8)                         # ...configured as 16

    def test_result_is_an_integer_count_inside_the_cycle(self):
        samples, _ = sweep(CPE - 0.2)
        r = ezero.solve(samples, RES, PP)
        self.assertIsInstance(r['e_zero'], int)
        self.assertTrue(0 <= r['e_zero'] < round(CPE))


if __name__ == '__main__':
    unittest.main()


class _Var:
    def __init__(self, get=None, put=None):
        self._get, self._put = get, put

    @property
    def raw(self):
        return self._get()

    @raw.setter
    def raw(self, value):
        self._put(value)


class _Drive:
    """A rotor that follows the field exactly, whose first Motor.id reading is
    still the previous calibration step's current - what calibrate_all hands
    the encoder sweep when the gain step has only just let go."""

    def __init__(self, e_zero, stale_id=900):
        self.e_zero = e_zero
        self.ud_writes = []
        self.id_reads = 0
        self.stale_id = stale_id
        self.theta = 0          # unwrapped electrical angle, F16 counts
        self.last_wrapped = 0
        cell = lambda v: _Var(lambda: v, lambda _: None)
        sink = cell(0)
        self.sdo = {
            'EncoderConfig': {'Resolution': cell(RES)},
            'Calibration': {'poles': cell(2 * PP), 'i_peak': cell(10000), 'i_cal': cell(1000)},
            'ControlWord': sink, 'SetModeOfOperation': sink,
            'Motor': {'ud': _Var(lambda: self.ud_writes[-1], self.ud_writes.append),
                      'uq': sink, 'id': _Var(self._id, None)},
            'Theta_e': _Var(lambda: self.last_wrapped, self._theta),
            'Encoder': {'RawPosition': _Var(self._raw, None)},
        }

    def _id(self):
        self.id_reads += 1
        if self.id_reads == 1:
            return self.stale_id
        return min(1000, (self.ud_writes[-1] if self.ud_writes else 0) // 20)

    def _theta(self, wrapped):
        delta = (wrapped - self.last_wrapped + 32768) % ezero.F16_CYCLE - 32768
        self.theta += delta
        self.last_wrapped = wrapped

    def _raw(self):
        return int(round(self.e_zero + self.theta / float(ezero.F16_CYCLE) * CPE)) % RES


class TestSweepRamp(unittest.TestCase):
    def test_a_stale_current_reading_does_not_end_the_ramp_at_zero(self):
        """Measured first, the leftover reading ended the ramp with ud never
        written, and the sweep turned a field of zero volts."""
        drive = _Drive(e_zero=250)
        ezero.sweep(drive, sleep=lambda s: None)
        self.assertGreater(max(drive.ud_writes), 0)
        self.assertEqual(drive.ud_writes[-1], 0)              # left de-energised

    def test_ramps_to_the_calibration_current(self):
        drive = _Drive(e_zero=250, stale_id=0)
        r = ezero.sweep(drive, sleep=lambda s: None)
        self.assertEqual(max(drive.ud_writes), 2000)          # 100 pm of 10 A = i_cal
        self.assertEqual(drive.ud_writes[-1], 0)
        self.assertEqual(r['e_polarity'], 1)
        self.assertLess(abs(ezero.wrap(r['e_zero'] - 250, CPE)), 2)


class TestSweepFault(unittest.TestCase):
    def test_a_fault_mid_sweep_is_named_not_read_as_a_stall(self):
        """An amplifier over-temperature (0x4210) drops the field; the rotor
        stops and the travel check would call it a stall."""
        drive = _Drive(e_zero=250, stale_id=0)
        cell = lambda v: _Var(lambda: v, lambda _: None)
        drive.sdo['StatusWord'] = cell(0x0008)
        drive.sdo['LastError'] = cell(0x4210)
        drive.sdo['Amplifier'] = {'Temperature': cell(71)}
        with self.assertRaises(ezero.EZeroError) as cm:
            ezero.sweep(drive, sleep=lambda s: None)
        self.assertIn('0x4210', str(cm.exception))
        self.assertIn('forward', str(cm.exception))
        self.assertIn('71 C', str(cm.exception))
        self.assertEqual(drive.ud_writes[-1], 0)              # left de-energised

    def test_no_fault_bit_sweeps_on(self):
        drive = _Drive(e_zero=250, stale_id=0)
        drive.sdo['StatusWord'] = _Var(lambda: 0x0237, None)
        self.assertEqual(ezero.sweep(drive, sleep=lambda s: None)['e_polarity'], 1)
