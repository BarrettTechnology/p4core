"""
Tests for encoder.py: the runout fit, bin selection and Q8.8 bins (checked
by applying them the way firmware does), the sweep/upload/retest against a
stub drive.
"""

import math
import os
import struct
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__),
                                                '..', '..')))

from p4core.actuator import encoder                                      # noqa: E402
from p4core.actuator.device import SysidError                            # noqa: E402

RES = 4096

def fw_correct(raw, q88, res=RES):
    """pwm.c correct_pos: pos -= [A_s sin(k th) + A_c cos(k th)] / 256."""
    th = 2 * math.pi * (raw % res) / res
    return raw - sum((a_s * math.sin(k * th) + a_c * math.cos(k * th)) / 256.0
                     for k, a_s, a_c in q88)


class Rotor:
    """Rotor on a d-axis field, read by an encoder with error *errors*
    = {k: (amp_ct, phase_rad)}: raw = true + sum amp cos(k phi + phase)."""

    def __init__(self, pp=7, pol=1, errors=None, lag_ct=1.5, offset=1234,
                 noise=0.0, seed=1, cogging=None):
        self.pp, self.pol = pp, pol
        self.errors = errors or {}
        self.cogging = cogging or {}       # {k: (amp_ct, phase)}: the rotor's
        self.lag, self.offset = lag_ct, offset
        self.noise = noise
        self.rng = np.random.default_rng(seed)
        self.theta = 0
        self.last_dir = 1
        self.q88 = []
        self.active = False

    def true(self):
        cpe = RES / self.pp
        p = self.offset + self.pol * self.theta / 65536.0 * cpe \
            - self.lag * self.last_dir
        phi = 2 * math.pi * p / RES
        return p + sum(a * math.cos(k * phi + ph)
                       for k, (a, ph) in self.cogging.items())

    def error(self, p):
        phi = 2 * math.pi * (p % RES) / RES
        return sum(a * math.cos(k * phi + ph) for k, (a, ph) in self.errors.items())

    def raw(self):
        p = self.true()
        r = p + self.error(p) + (self.rng.normal(0, self.noise) if self.noise else 0)
        return int(round(r)) % RES

    def enc(self):
        r = self.raw()
        return int(round(fw_correct(r, self.q88 if self.active else []))) % RES

def sweep_samples(rotor, steps=64):
    out = []
    step = 65536 // steps
    lead = steps // 2
    ref = rotor.raw()
    for d in (1, -1):
        rotor.last_dir = d
        for k in range(lead + rotor.pp * steps):
            rotor.theta += d * step
            raw = rotor.raw()
            ref = encoder._unwrap(raw, ref, RES)
            if k >= lead:
                out.append((d, rotor.theta, ref, raw))
    return out


class TestFit(unittest.TestCase):
    def test_recovers_harmonics(self):
        r = Rotor(errors={1: (4.0, 0.3), 2: (8.0, -1.1), 5: (1.0, 2.0)})
        f = encoder.fit_runout(sweep_samples(r), RES, 7, 1)
        for k, (a, _) in r.errors.items():
            self.assertAlmostEqual(f['amp'][k - 1], a, delta=0.35)
        # The friction lag is in the per-direction constants, not a harmonic.
        self.assertAlmostEqual(f['dc'][0] - f['dc'][1], 2 * 1.5, delta=0.3)

    def test_q88_bins_cancel_the_error(self):
        for pol in (1, -1):
            r = Rotor(pol=pol, errors={1: (4.0, 0.3), 2: (8.0, -1.1)})
            f = encoder.fit_runout(sweep_samples(r), RES, 7, pol)
            q = encoder.bins_q88(f, [1, 2])
            p = np.arange(0, RES, 7, dtype=float)
            before = np.array([r.error(x) for x in p])
            after = np.array([fw_correct(x + r.error(x), q) - x for x in p])
            self.assertLess(np.sqrt(np.mean(after ** 2)),
                            0.1 * np.sqrt(np.mean(before ** 2)), 'pol %d' % pol)

    def test_cogging_makes_no_sidebands(self):
        # Node 4: 42 = 6x7 cogging moves the rotor; the encoder's 11 ct k = 2
        # error, fitted in the raw angle, used to modulate it into 42 +- 2n
        # and kept a 1.4 ct k = 38 bin the encoder did not have.
        r = Rotor(errors={2: (11.0, 1.86)}, cogging={42: (20.0, 0.4),
                                                    14: (4.0, 1.0)})
        f = encoder.fit_runout(sweep_samples(r), RES, 7, 1)
        self.assertAlmostEqual(f['amp'][1], 11.0, delta=0.35)
        for k in (36, 38, 40, 44, 46):
            if k <= len(f['amp']):
                self.assertLess(f['amp'][k - 1], 0.3, 'k = %d' % k)
        self.assertTrue(f['field_locked'][41])
        self.assertAlmostEqual(f['amp'][41], 20.0, delta=1.0)
        self.assertEqual(encoder.select_bins(f, 7, RES, n_bins=2)['k'], [2])

    def test_one_pole_pair(self):
        r = Rotor(pp=1, errors={1: (3.0, 0.0)})
        f = encoder.fit_runout(sweep_samples(r), RES, 1, 1)
        self.assertAlmostEqual(f['amp'][0], 3.0, delta=0.4)

    def test_one_pole_pair_with_drag_is_not_a_lag(self):
        # Node 3 (slotless, 1 pp, gearhead drag): the two directions read
        # different counts, so both together have ~128 distinct readings for
        # 128 samples.  Sized from that the fit was square, the direction
        # constants landed 700 ct off and the peak read 137 deg electrical.
        r = Rotor(pp=1, errors={1: (30.0, 0.4), 2: (20.0, 1.0)}, lag_ct=9.0,
                  noise=3.0, seed=4)
        smp = sweep_samples(r)
        f = encoder.fit_runout(smp, RES, 1, 1)
        self.assertLessEqual(len(f['k']), 31)
        self.assertAlmostEqual(f['dc'][0] - f['dc'][1], 18.0, delta=3.0)
        self.assertLess(f['error_peak_ct'] / f['cpe'] * 360,
                        encoder.SWEEP_LAG_MAX_DEG)
        self.assertAlmostEqual(f['amp'][0], 30.0, delta=3.0)


class TestSelect(unittest.TestCase):
    def fit(self, errors, pp=7):
        r = Rotor(pp=pp, errors=errors, noise=0.2)
        return encoder.fit_runout(sweep_samples(r), RES, pp, 1)

    def test_rules(self):
        f = self.fit({2: (8.0, 0), 6: (3.0, 0), 7: (5.0, 0), 1: (2.0, 0),
                      12: (4.0, 0)})
        sel = encoder.select_bins(f, 7, RES, n_bins=2, skip_k=(12,))
        self.assertEqual(sel['k'], [2, 12][:1] + [1])
        self.assertIn('stator', sel['dropped'][6])
        self.assertIn('electrical', sel['dropped'][7])
        self.assertIn('skip_k', sel['dropped'][12])

    def test_only_n_bins(self):
        f = self.fit({1: (3.0, 0), 2: (8.0, 0), 3: (2.0, 0)})
        sel = encoder.select_bins(f, 7, RES, n_bins=2)
        self.assertEqual(sel['k'], [2, 1])
        self.assertIn('bins', sel['dropped'][3])

    def test_noise_only(self):
        f = self.fit({})
        self.assertEqual(encoder.select_bins(f, 7, RES)['k'], [])

    def test_slope_gate(self):
        # 30 ct at k = 20: slope 30 * 2 pi 20 / 4096 = 0.92 > 0.75; the next
        # one alone fits.  pole_pairs 13 (not 3): keeps k = 20 clear of the
        # cogging-leak window below (pp x 2 = 26, pp x 1 = 13).
        amp = np.zeros(42)
        a = np.zeros(42)
        a[19], a[0] = 30.0, 8.0
        f = {'k': np.arange(1, 43), 'a': a, 'b': np.zeros(42), 'amp': np.abs(a)}
        self.assertAlmostEqual(encoder.table_slope(f, [20], RES),
                               30 * 2 * math.pi * 20 / RES, places=3)
        sel = encoder.select_bins(f, 13, RES)
        self.assertEqual(sel['k'], [1])
        self.assertIn('slope', sel['dropped'][20])

    def test_cogging_leak_window(self):
        # Node 4 (pp 7): a strong 42 = 6x7 cogging line leaks into k = 40.
        f = self.fit({42: (6.0, 0), 40: (5.5, 0), 2: (8.0, 0)})
        sel = encoder.select_bins(f, 7, RES, n_bins=2)
        self.assertIn('leak', sel['dropped'][40])
        self.assertIn('electrical', sel['dropped'][42])
        self.assertNotIn(40, sel['k'])

    def test_cogging_leak_protects_sensor_terms(self):
        # k = 1, 2 are the MA7xx's own dominant terms: never masked, even
        # where the leak window falls right over them.
        f1 = self.fit({1: (3.0, 0)}, pp=3)          # pp x 1 = 3, window [1, 5]
        self.assertNotIn(1, encoder.select_bins(f1, 3, RES, n_bins=2)['dropped'])
        f2 = self.fit({2: (8.0, 0)}, pp=4)          # pp x 1 = 4, window [2, 6]
        self.assertNotIn(2, encoder.select_bins(f2, 4, RES, n_bins=2)['dropped'])

    def test_cogging_leak_is_a_noop_at_one_pole_pair(self):
        # pp = 1: every k is a pole-pair multiple; the existing k <= 2 rule
        # already decides this, so the leak window never runs here.
        f = self.fit({1: (3.0, 0), 2: (8.0, 0), 3: (5.0, 0)}, pp=1)
        sel = encoder.select_bins(f, 1, RES, n_bins=2)
        self.assertEqual(sorted(sel['k']), [1, 2])
        self.assertIn('mechanical', sel['dropped'][3])


class StubSdo:
    def __init__(self, dev):
        self.dev = dev
        self.od = {(0x3027, 0): 9,   # highest sub: 2 bins + fn, zeta
                   (0x3013, 6): 0xA0, (0x3013, 5): -34,
                   (0x3013, 10): 6020}
        self.od.update({(0x3027, s): 0 for s in range(1, 10)})

    def upload(self, idx, sub):
        v = self.od[(idx, sub)]
        return struct.pack('<i', v)

    def download(self, idx, sub, data):
        fmt = 'h' if (idx, sub) in ((0x3013, 5),) or (
            idx == 0x3027 and sub in (2, 4, 5, 7)) else 'H'
        v = struct.unpack('<' + fmt, bytes(data)[:2])[0]
        self.od[(idx, sub)] = v
        if idx == 0x3027:
            self.dev.sync_comp()

    def __getitem__(self, name):
        dev = self.dev

        class V:
            def __init__(self, f):
                self.f = f

            @property
            def raw(self):
                return self.f()
        if name == 'Encoder':
            return {'RawPosition': V(dev.rotor.raw), 'EncPos': V(dev.rotor.enc)}
        if name == 'Calibration':
            return {'e_polarity': V(lambda: dev.rotor.pol)}
        raise KeyError(name)


class StubNode:
    def __init__(self, dev):
        self.sdo = StubSdo(dev)


class StubDev:
    def __init__(self, rotor, sign=1):
        self.rotor = rotor
        self.pole_pairs = rotor.pp
        self.node = StubNode(self)
        self.logs = []
        self.sign = sign       # -1: firmware applies the bins backwards

    def sync_comp(self):
        od = self.node.sdo.od
        self.rotor.active = bool(od[(0x3027, 1)])
        q = []
        for i in range(2):
            k = od[(0x3027, 3 + 3 * i)]
            if k:
                q.append((k, self.sign * od[(0x3027, 2 + 3 * i)],
                          self.sign * od[(0x3027, 4 + 3 * i)]))
        self.rotor.q88 = q

    def refresh_scaling(self):
        return {'enc_res': RES, 'i_cal_ma': 1000, 'i_peak_ma': 5000,
                'u_max_v': self.u_max_v}

    def _log(self, m):
        self.logs.append(m)

    def _begin_d_axis(self, theta_e=0):
        pass

    rs_ohm = None
    u_max_v = None
    ramp_a = None

    def _configured_rs_ohm(self):
        return self.rs_ohm

    def _ramp_ud_to_current(self, a):
        self.ramp_a = a

    def _sleep(self, s):
        pass

    def _check_abort(self):
        pass

    def safe_stop(self):
        pass

    fault = None

    def _raise_if_faulted(self, what):
        if self.fault:
            raise SysidError('{} stopped with the drive in FAULT: {}'.format(
                what, self.fault))

    def _set_theta_e(self, theta):
        self.rotor.last_dir = 1 if theta > self.rotor.theta else -1
        self.rotor.theta = theta


class TestCalibrateRunout(unittest.TestCase):
    def test_upload_and_retest(self):
        dev = StubDev(Rotor(errors={2: (8.0, -1.1), 1: (3.0, 0.4)}, noise=0.2))
        r = encoder.calibrate_runout(dev)
        self.assertTrue(r['passed'], dev.logs)
        self.assertEqual(sorted(k for k, _, _ in r['bins']), [1, 2])
        self.assertEqual(r['filter_model'], (150, 880))
        od = dev.node.sdo.od
        self.assertEqual(od[(0x3027, 1)], 1)
        self.assertEqual((od[(0x3027, 8)], od[(0x3027, 9)]), (150, 880))
        for b in r['retest']['bins']:
            self.assertLess(b['after_ct'], 0.5 * b['before_ct'])

    def test_high_r_sweeps_within_the_voltage(self):
        dev = StubDev(Rotor(errors={2: (8.0, -1.1)}, noise=0.2))
        encoder.calibrate_runout(dev)
        self.assertAlmostEqual(dev.ramp_a, 1.0)          # i_cal
        dev.rs_ohm, dev.u_max_v = 11.25, 13.86           # an 11 ohm P4-16
        encoder.calibrate_runout(dev)
        self.assertAlmostEqual(dev.ramp_a, 0.45 * 13.86 / 11.25)

    def test_wrong_sign_switches_off(self):
        dev = StubDev(Rotor(errors={2: (8.0, -1.1)}, noise=0.2), sign=-1)
        r = encoder.calibrate_runout(dev)
        self.assertFalse(r['passed'])
        self.assertEqual(dev.node.sdo.od[(0x3027, 1)], 0)

    def test_rotor_that_does_not_follow_fails(self):
        # stuck: the field turns, the rotor does not
        dev = StubDev(Rotor(errors={2: (8.0, -1.1)}))
        dev._set_theta_e = lambda theta: None
        dev.node.sdo.od[(0x3027, 1)] = 1
        with self.assertRaises(encoder.EncoderCalError):
            encoder.calibrate_runout(dev)
        self.assertEqual(dev.node.sdo.od[(0x3027, 1)], 1)    # left as it was
        # follows, but lags the field by far more than any runout
        dev = StubDev(Rotor(errors={1: (100.0, 0.0)}))   # 62 deg electrical
        with self.assertRaises(encoder.EncoderCalError):
            encoder.calibrate_runout(dev)

    def test_a_fault_mid_sweep_is_named_not_read_as_a_stall(self):
        # A P4-37 at 3 A and 80 kHz tripped its 70 C amplifier limit: the
        # field dropped and the rotor stopped 0.08 rev in.
        dev = StubDev(Rotor(errors={2: (8.0, -1.1)}))
        dev._set_theta_e = lambda theta: None
        dev.fault = 'error 0x4210 amplifier over-temperature'
        with self.assertRaisesRegex(SysidError, 'runout sweep.*0x4210'):
            encoder.calibrate_runout(dev)

    def test_unknown_filter_leaves_model_off(self):
        dev = StubDev(Rotor(errors={2: (8.0, 0)}, noise=0.2))
        dev.node.sdo.od[(0x3013, 6)] = 0x20
        r = encoder.calibrate_runout(dev)
        self.assertEqual(r['filter_model'], (0, 0))


if __name__ == '__main__':
    unittest.main()
