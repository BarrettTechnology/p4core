"""
Tests for cogging.py: the load-torque fit on synthetic captures (a rotor
with known cogging, its speed rippling as a velocity loop lets it), the
choice of bins, and the 0x302E layout.
"""

import math
import os
import struct
import sys
import unittest
from unittest import mock

import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__),
                                                '..', '..')))

from p4core.actuator import cogging                                      # noqa: E402

RES, CTRL_HZ, KT, J = 4096, 15000.0, 31.0, 7.45e-6
KT_PK = KT * 1e-3 / math.sqrt(2.0)
#: Node 4's lines, mNm (c cos + s sin on the encoder's angle).
LINES = {14: 2.9 * np.exp(1j * 0.3), 12: 0.9 * np.exp(-1.2j),
         28: 1.7 * np.exp(2.0j), 84: 1.9 * np.exp(-0.5j)}

def cfg():
    return dict(enc_res=RES, control_hz=CTRL_HZ, kt=KT, j_kg_m2=J,
                current_bw_hz=234.0, vel_kp=0.003)

def synth(rpms, revs=6, lines=LINES, wander=None, ripple_rad=0.02,
          iq_noise_a=0.01, seed=0):
    """A capture: the angle at the control rate is the speed plus a ripple
    at k14 (ripple_rad, as large as an uncompensated rotor's), the iq what
    makes Kt iq - J alpha the cogging, then 1 kHz frames of it: position
    quantised, iq averaged over each frame.  *wander* = (k, mNm): a line
    whose phase is new every run."""
    rng = np.random.default_rng(seed)
    runs = []
    for rpm in rpms:
        for sgn in (1, -1):
            W = rpm / 60.0 * 2 * np.pi
            n = int((revs + 0.3) * 2 * np.pi / W * CTRL_HZ)
            t = np.arange(n) / CTRL_HZ
            th0 = sgn * W * t
            th = th0 + ripple_rad * np.sin(14 * th0 + 0.7)
            al = np.gradient(np.gradient(th, t), t)
            d = sum((h.real * np.cos(k * th) + h.imag * np.sin(k * th))
                    for k, h in lines.items()) * 1e-3
            if wander:
                ph = rng.uniform(0, 2 * np.pi)
                d = d + wander[1] * 1e-3 * np.cos(wander[0] * th + ph)
            iq = (J * al + d) / KT_PK + rng.normal(0, iq_noise_a, n)
            step = int(CTRL_HZ / 1000)
            idx = np.arange(0, n, step)
            pos = np.round(th[idx] * RES / (2 * np.pi))
            iqf = np.array([iq[a:a + step].mean() for a in idx[:-1]])
            runs.append(dict(rpm=rpm, sgn=sgn, cycles=idx.astype(float),
                             iq=iqf, pos=pos))
    return dict(cfg=cfg(), frame=dict(p6064=0, enc=0, pol=0, epol=1),
                runs=runs)

def fitted(cap, n_bins=4, exclude=()):
    lines = cogging.fit_lines(cogging.revolutions(cap, 96))
    return cogging.choose_bins(lines, KT, n_bins, exclude)


class TestFit(unittest.TestCase):
    def test_recovers_the_lines_under_large_ripple(self):
        bins, _ = fitted(synth((30.0, 60.0)))
        self.assertEqual([b[1] for b in bins], [14, 84, 28, 12])
        for a_s, k, a_c in bins:
            want = LINES[k] / (KT_PK * 1e3) * 1e3     # mA
            self.assertAlmostEqual(a_c, want.real, delta=4 + 0.03 * abs(want))
            self.assertAlmostEqual(a_s, want.imag, delta=4 + 0.03 * abs(want))

    def test_fewer_bins_keeps_the_strongest(self):
        bins, _ = fitted(synth((30.0, 60.0)), n_bins=3)
        self.assertEqual([b[1] for b in bins], [14, 84, 28])

    def test_a_line_that_wanders_run_to_run_is_not_a_bin(self):
        cap = synth((30.0, 60.0), lines={14: LINES[14]}, wander=(5, 2.0),
                    seed=3)
        bins, cands = fitted(cap)
        self.assertEqual([b[1] for b in bins], [14])
        self.assertNotIn(5, [c['k'] for c in cands])

    def test_excluded_orders_are_not_bins(self):
        bins, _ = fitted(synth((30.0, 60.0)), exclude={12})
        self.assertNotIn(12, [b[1] for b in bins])

    def test_noise_alone_gives_no_bins(self):
        bins, _ = fitted(synth((30.0, 60.0), lines={}, ripple_rad=0.0))
        self.assertEqual(bins, [])


class TestRipple(unittest.TestCase):
    def test_locked_ripple_falls_with_the_cogging(self):
        off = cogging.revolutions(synth((30.0,), ripple_rad=0.02), 96)
        on = cogging.revolutions(synth((30.0,), ripple_rad=0.002), 96)
        v0, s0, _ = cogging.ripple(off)
        v1, s1, _ = cogging.ripple(on)
        self.assertGreater(v0 - v1, 3 * math.hypot(s0, s1))
        l0, _ = cogging.line_rpm(off, 14)
        l1, _ = cogging.line_rpm(on, 14)
        self.assertGreater(l0, 5 * l1)

def frames(n=200, step=15, rate=2.0, seed=0):
    """Stream frames (cycle count mod 2^16, iq sum, position) at a steady
    speed, the count starting near its wrap."""
    c = (65000 + step * np.arange(n)) % 65536
    pos = np.round(rate * step * np.arange(n))
    iq = np.cumsum(np.full(n, 100.0 * step)) % 2 ** 32
    return np.column_stack([c, iq, pos])


class TestBadFrames(unittest.TestCase):
    def test_good_frames_are_all_kept(self):
        fr = frames()
        got, bad = cogging.clean_frames(fr, RES)
        self.assertEqual(bad, 0)
        np.testing.assert_array_equal(got, fr)

    def test_a_lost_frame_is_not_bad(self):
        fr = np.delete(frames(), 50, axis=0)
        self.assertEqual(cogging.clean_frames(fr, RES)[1], 0)

    def test_a_duplicate_is_dropped(self):
        fr = frames()
        fr = np.insert(fr, 60, fr[59], axis=0)
        got, bad = cogging.clean_frames(fr, RES)
        self.assertEqual(bad, 1)
        self.assertTrue((np.mod(np.diff(got[:, 0]), 65536) > 0).all())

    def test_a_glitched_count_or_position_is_dropped(self):
        fr = frames()
        fr[70, 0] = (fr[70, 0] + 30000) % 65536
        fr[120, 2] += 3000
        got, bad = cogging.clean_frames(fr, RES)
        self.assertEqual(bad, 2)
        self.assertEqual(len(got), 198)

    def test_a_stall_resyncs_instead_of_dropping_the_rest(self):
        fr = frames()
        fr[100:, 0] = (fr[100:, 0] + 15 * 40) % 65536     # 40 frames late
        fr[100:, 2] += 2.0 * 15 * 40
        got, bad = cogging.clean_frames(fr, RES)
        self.assertEqual(bad, 0)

    def test_a_bad_revolution_is_dropped_from_the_fit(self):
        cap = synth((60.0,))
        r = cap['runs'][0]
        r['pos'] = r['pos'].copy()
        r['pos'][len(r['pos']) // 2] += 100               # one bad position
        revs = cogging.revolutions(cap, 96)
        self.assertEqual(revs[0]['bad_revs'], 1)
        self.assertEqual(revs[1]['bad_revs'], 0)


class TestLeadScan(unittest.TestCase):
    def scan(self, best_us, budget=806.0):
        seen = []

        def measure(lead):
            seen.append(lead)
            return 4.0 + ((lead - best_us) / 500.0) ** 2, 0.1
        return cogging.scan_lead(budget, measure), seen

    def test_node_4_finds_the_hand_lead(self):
        scan, seen = self.scan(1500.0)
        self.assertEqual(cogging.best_lead(scan), 1450)
        self.assertEqual(len(seen), 4)        # 1.8x is inside the scan

    def test_extends_while_the_best_is_at_the_top(self):
        scan, seen = self.scan(2600.0)
        self.assertLessEqual(abs(cogging.best_lead(scan) - 2600), 200)
        self.assertEqual(seen, sorted(seen))
        self.assertGreater(len(seen), 4)

    def test_extends_down_and_stops_at_the_limits(self):
        scan, _ = self.scan(100.0)
        self.assertEqual(scan[0][0], cogging._lead_at(806.0, 0.6))
        scan, _ = self.scan(1e5)
        self.assertEqual(scan[-1][0], cogging._lead_at(806.0, 3.4))


class TestDesign(unittest.TestCase):
    def test_node_4_lead_and_fade(self):
        c = cfg()
        # 568 us encoder + 680 us current loop + a 67 us control cycle
        self.assertAlmostEqual(cogging.lead_budget_us(c, 568.0), 1315, delta=2)
        self.assertEqual(cogging.fade_for(c), 101)
        c['current_bw_hz'] = None
        self.assertEqual(cogging.fade_for(c), 101)    # control_hz / 64

    def test_exclusions(self):
        self.assertEqual(cogging.excluded_ks(7, [2, 36]), {7, 2, 34, 36, 38})
        # one pole pair: k1 is the rotor's own line too, the z test decides
        self.assertEqual(cogging.excluded_ks(1, [2]), {2})

    def test_minimum_line(self):
        self.assertEqual(cogging.min_bin_ma(1.0), 10.0)
        self.assertEqual(cogging.min_bin_ma(0.1), 1.0)
        self.assertAlmostEqual(cogging.min_bin_ma(0.1, 28284), 2.83, places=2)
        self.assertEqual(cogging.min_bin_ma(0.1, 3000), 1.0)


class _Sdo:
    def __init__(self, count, unit=None):
        self.v = {(cogging.OD_COG, 0): struct.pack('<B', count)}
        if unit is not None:
            self.v[(cogging.OD_COG, 16)] = struct.pack('<H', unit)

    def upload(self, index, sub):
        return self.v.get((index, sub), b'\0\0')

    def download(self, index, sub, data):
        self.v[(index, sub)] = bytes(data)


class _Dev:
    def __init__(self, count, unit=None):
        self.node = type('N', (), {})()
        self.node.sdo = _Sdo(count, unit)

    def raw(self, sub):
        return struct.unpack('<h', self.node.sdo.v[(cogging.OD_COG, sub)])[0]


class TestOd(unittest.TestCase):
    def test_layouts(self):
        self.assertEqual(cogging.layout(_Dev(10)), (3, False, False))
        self.assertEqual(cogging.layout(_Dev(11)), (3, True, False))
        self.assertEqual(cogging.layout(_Dev(15)), (4, True, True))
        self.assertEqual(cogging.bin_subs(2), (8, 9, 10))
        self.assertEqual(cogging.bin_subs(3), (12, 13, 14))

    def test_write_and_read_back(self):
        dev = _Dev(15)
        cogging.write_table(dev, bins=[(133, 14, -15), (84, 84, -26)],
                            lead_us=1500, fade_hz=100, active=1)
        t = cogging.read_table(dev)
        self.assertEqual(t['bins'], [(133, 14, -15), (84, 84, -26),
                                     (0, 0, 0), (0, 0, 0)])
        self.assertEqual((t['active'], t['lead_us'], t['fade_hz']),
                         (1, 1500, 100))

    def test_tenth_ma_units(self):
        dev = _Dev(16, unit=1)
        self.assertEqual(cogging.bin_unit_ma(dev), 0.1)
        cogging.write_table(dev, bins=[(3.1, 1, -4.9), (2.04, 2, 1.96)])
        self.assertEqual((dev.raw(2), dev.raw(4)), (31, -49))
        self.assertEqual((dev.raw(5), dev.raw(7)), (20, 20))
        self.assertEqual(cogging.read_table(dev)['bins'][:2],
                         [(3.1, 1, -4.9), (2.0, 2, 2.0)])

    def test_whole_ma_units(self):
        for dev in (_Dev(15), _Dev(16, unit=0)):
            self.assertEqual(cogging.bin_unit_ma(dev), 1.0)
            cogging.write_table(dev, bins=[(3.4, 1, -4.6)])
            self.assertEqual((dev.raw(2), dev.raw(4)), (3, -5))

    def test_bins_round_to_the_unit(self):
        lines = dict(mean=np.array([KT_PK * (3.04 - 4.96j)]),
                     se=np.array([1e-6]))
        bins, _ = cogging.choose_bins(lines, KT, 1, unit_ma=0.1)
        self.assertEqual(bins, [(-5.0, 1, 3.0)])
        bins, _ = cogging.choose_bins(lines, KT, 1, unit_ma=1.0)
        self.assertEqual(bins, [])          # 5.8 mA < 10 units of 1 mA

    def test_too_many_bins_for_the_firmware(self):
        with self.assertRaises(cogging.SysidError):
            cogging.write_table(_Dev(11), bins=[(1, 1, 1)] * 4)


class TestCalibrateNoGain(unittest.TestCase):
    def test_no_bin_helping_keeps_the_old_table_and_passes_on(self):
        """Node 2 (direct drive): the fitted lines did not lower the
        velocity ripple.  That is a finding, not a fault: the drive keeps
        the table it had and the step does not stop the plan."""
        dev = _Dev(16, unit=1)
        cogging.write_table(dev, bins=[(3.0, 84, -2.0)], lead_us=1500,
                            fade_hz=100, active=1)
        before = cogging.read_table(dev)
        bins = [(21.6, 148, -12.1), (-8.6, 94, 17.4)]
        cfg = dict(vel_kp=0.01, kt=30.0, i_peak=9913, control_hz=15000,
                   current_bw_hz=500)
        with mock.patch.object(cogging.steptest, 'read_config', return_value=cfg), \
                mock.patch.object(cogging.steptest, '_sleep'), \
                mock.patch.object(cogging.encoder, 'comp_layout', return_value=(0, None)), \
                mock.patch.object(cogging.encoder, 'read_latency_us', return_value=(500.0, 0)), \
                mock.patch.object(cogging, 'capture', return_value={}), \
                mock.patch.object(cogging, 'kmax_for', return_value=200), \
                mock.patch.object(cogging, 'revolutions', return_value=[]), \
                mock.patch.object(cogging, 'fit_lines', return_value={}), \
                mock.patch.object(cogging, 'choose_bins', return_value=(bins, [])), \
                mock.patch.object(cogging, 'ripple', return_value=(10.0, 0.4, None)), \
                mock.patch.object(cogging, 'judge', return_value=([dict(rpm=30, off=9.7, on=9.8, off_se=0.3, on_se=0.3)], [])):
            rec = cogging.calibrate(dev, 14)
        self.assertFalse(rec['passed'])
        self.assertEqual(rec['bins'], [])
        self.assertIn('no bin lowered', rec['detail'])
        self.assertEqual(cogging.read_table(dev), before)
        self.assertEqual(cogging.summary(rec), rec['detail'])


if __name__ == '__main__':
    unittest.main()
