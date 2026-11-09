"""Tests for actuator/friction.py: the Coulomb feed-forward kept or dropped
on stubbed PP moves, and pp_move's trapezoid; and the Friction calibration
step (steps._r_friction) on a stubbed drag."""

import unittest
from unittest import mock

import fake  # noqa: F401  (puts p4core on the path)

from p4core.actuator import commission, friction, steps, steptest


class TestFrictionFf(unittest.TestCase):
    def _run(self, tc_mnm, pp, regs=None):
        regs = regs if regs is not None else {'pos_velff': 8192,
                                              'vel_tc_ff': 0.0}
        calls = []

        def pp_at(dev, velff, cfg=None, top=None, deg=None, band_deg=None,
                  tail_s=None):
            calls.append(velff)
            return pp(regs)
        with mock.patch.object(steptest, 'read', lambda d, k: regs[k]), \
                mock.patch.object(steptest, 'write',
                                  lambda d, k, v: regs.__setitem__(k, v)), \
                mock.patch.object(friction, 'pp_following', pp_at), \
                mock.patch.object(friction, '_faulted', lambda d: False):
            r = friction.friction_ff(object(), tc_mnm,
                                     cfg={'rated_torque': 600.0,
                                          'enc_res': 4096},
                                     log=lambda *_: None)
        return r, regs, calls

    @staticmethod
    def _pp(ferr, settle, over=0.0):
        return {'ferr_rms_deg': ferr, 'ferr_max_deg': 2 * ferr,
                'over_deg': over, 'settle_s': settle,
                'reps': [{'settle_s': settle}, {'settle_s': settle}]}

    def test_kept_when_it_follows_closer(self):
        r, regs, calls = self._run(5.0, lambda r: self._pp(
            1.0 if not r['vel_tc_ff'] else 0.6, 0.1))
        self.assertTrue(r['kept'])
        self.assertAlmostEqual(r['permille'], 0.8 * 5.0 / 600 * 1000)
        self.assertAlmostEqual(regs['vel_tc_ff'], r['permille'])
        self.assertTrue(all(v == 0.5 for v in calls))   # the drive's velff
        self.assertEqual(len(calls), 2 * friction.FRICTION_FF_REPS)

    def test_dropped_when_it_does_not_help(self):
        r, regs, _ = self._run(5.0, lambda r: self._pp(1.0, 0.1))
        self.assertFalse(r['kept'])
        self.assertEqual(regs['vel_tc_ff'], 0.0)
        self.assertIn('not kept', friction.summary_friction_ff(r))

    def test_dropped_when_it_overshoots(self):
        r, regs, _ = self._run(5.0, lambda r: self._pp(
            1.0 if not r['vel_tc_ff'] else 0.5, 0.1,
            over=0.0 if not r['vel_tc_ff'] else 2.0))
        self.assertFalse(r['kept'])
        self.assertEqual(regs['vel_tc_ff'], 0.0)

    def test_capped(self):
        r, _, _ = self._run(500.0, lambda r: self._pp(
            1.0 if not r['vel_tc_ff'] else 0.5, 0.1))
        self.assertTrue(r['capped'])
        self.assertEqual(r['permille'], friction.FRICTION_FF_MAX_PERMILLE)

    def test_no_tc_writes_it_off(self):
        r, regs, calls = self._run(0.0, lambda r: self._pp(1.0, 0.1),
                                   regs={'pos_velff': 0, 'vel_tc_ff': 3.0})
        self.assertEqual(regs['vel_tc_ff'], 0.0)
        self.assertFalse(calls)
        self.assertIn('ff off', friction.summary_friction_ff(r))


class TestPpMove(unittest.TestCase):
    def test_trapezoid_under_the_caps(self):
        cfg = {'enc_res': 4096, 'rated_torque': 140.0, 'j_kg_m2': 6.7e-6,
               'vel_kp': 0.003, 'pos_kp': 50, 'pos_outmul': 100}
        cts, vmax, acc, move = friction.pp_move(cfg, top_rpm=3000)
        self.assertEqual(cts, 1024)
        self.assertLessEqual(vmax, 0.3 * 3000 * 4096 / 60.0)
        self.assertLessEqual(vmax, friction.VELFF_LAG_FRACTION * cts * 50)
        alpha = 0.3 * 140e-3 / 6.7e-6 * 4096 / (2 * 3.141592653589793)
        self.assertLessEqual(acc, alpha * 1.0001)
        self.assertGreater(move, cts / vmax)


class TestFrictionStep(unittest.TestCase):
    """steps._r_friction: the drag, the velocity gains rescaled to its J,
    then the ff on its Tc.  The drag's physics is p4sysid's test_device."""

    def _run(self, j_was, j_drag, has_ff=True, answer=True):
        regs = {'vel_kp': 0.004, 'vel_ki': 0.2}
        s = mock.Mock(values={}, ctx={'coast': {'j_kg_m2': j_was}},
                      limits=commission.Limits(), log=lambda *_: None)
        self.asked = []
        s.confirm = lambda q, default=False: self.asked.append(
            (q, default)) or answer
        s.dev.node = object()
        writes = []
        s.write = lambda *keys: writes.append(keys)

        def drag(dev, limits, reboot, rep):
            s.values.update(j_kg_m2=j_drag, tc_nm=0.002,
                            b_nm_s_per_rad=1e-6)
        s.stage = lambda fn: fn(s.dev, s.limits, None, s.rep)
        ff = []
        with mock.patch.object(commission, '_stage_drag', drag), \
                mock.patch.object(steptest, 'read', lambda d, k: regs[k]), \
                mock.patch.object(steptest, 'write',
                                  lambda d, k, v: regs.__setitem__(k, v)), \
                mock.patch.object(steptest, 'read_config', lambda d: {}), \
                mock.patch.object(steps.firmware, 'has',
                                  lambda n, f: has_ff), \
                mock.patch.object(steps, 'rated_torque_mnm',
                                  lambda n: 140.0), \
                mock.patch.object(friction, 'friction_ff',
                                  lambda dev, tc, *a, **k: ff.append(tc) or
                                  {'capped': False, 'kept': True}), \
                mock.patch.object(friction, 'summary_friction_ff',
                                  lambda r: 'ff kept'):
            out = steps._r_friction(s)
        return out, regs, writes, ff

    def test_rescales_the_velocity_gains_to_the_drag_j(self):
        out, regs, writes, ff = self._run(5e-6, 6e-6)
        self.assertAlmostEqual(regs['vel_kp'], 0.004 * 1.2)
        self.assertAlmostEqual(regs['vel_ki'], 0.2 * 1.2)
        self.assertEqual(writes, [('j', 'tc', 'b')])
        self.assertEqual(ff, [2.0])               # mNm, after the rescale
        self.assertIn('x 1.200', out)
        self.assertIn('ff kept', out)

    def test_asks_first_with_the_old_and_new_gains(self):
        self._run(5e-6, 6e-6)
        (q, default), = self.asked
        self.assertTrue(q.startswith('Rescale velocity gains?\n'))
        self.assertIn('Kp 0.004 -> 0.0048', q)
        self.assertIn('Ki 0.2 -> 0.24', q)
        self.assertTrue(default)        # a script keeps the old behaviour

    def test_no_keeps_the_gains(self):
        out, regs, _, ff = self._run(5e-6, 6e-6, answer=False)
        self.assertEqual(regs, {'vel_kp': 0.004, 'vel_ki': 0.2})
        self.assertIn('velocity gains kept', out)
        self.assertEqual(ff, [2.0])     # the ff still runs

    def test_a_large_change_is_flagged(self):
        self._run(21e-6, 6e-6)          # x 0.285, node 15's
        q = self.asked[0][0]
        self.assertIn('large change', q)

    def test_same_j_leaves_the_gains(self):
        out, regs, _, _ = self._run(6e-6, 6e-6)
        self.assertEqual(regs, {'vel_kp': 0.004, 'vel_ki': 0.2})
        self.assertNotIn('velocity gains', out)
        self.assertEqual(self.asked, [])

    def test_no_prior_j_leaves_the_gains(self):
        _, regs, _, _ = self._run(None, 6e-6)
        self.assertEqual(regs['vel_kp'], 0.004)

    def test_firmware_without_the_ff_gets_the_drag_alone(self):
        out, _, writes, ff = self._run(6e-6, 6e-6, has_ff=False)
        self.assertEqual(writes, [('j', 'tc', 'b')])
        self.assertEqual(ff, [])
        self.assertIn('no Coulomb ff', out)


if __name__ == '__main__':
    unittest.main()
