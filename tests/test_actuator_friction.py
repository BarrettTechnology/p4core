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
    """steps._r_friction: the drag, then the ff on its Tc, with no loop gain
    touched.  The rescale p4sysid hooks in is p4sysid's test_steps; the
    drag's physics is its test_device."""

    def _run(self, j_was, j_drag, has_ff=True, after_drag=None):
        regs = {'vel_kp': 0.004, 'vel_ki': 0.2}
        s = mock.Mock(values={}, ctx={'coast': {'j_kg_m2': j_was}},
                      limits=commission.Limits(), log=lambda *_: None)
        self.asked = []
        s.confirm = lambda q, default=False: self.asked.append(q) or True
        s.dev.node = object()
        writes = []
        s.write = lambda *keys: writes.append(keys)

        def drag(dev, limits, reboot, rep):
            s.values.update(j_kg_m2=j_drag, tc_nm=0.002,
                            b_nm_s_per_rad=1e-6)
        s.stage = lambda fn: fn(s.dev, s.limits, None, s.rep)
        ff = []
        gains_at_ff = []
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
                                  gains_at_ff.append(dict(regs)) or
                                  {'capped': False, 'kept': True}), \
                mock.patch.object(friction, 'summary_friction_ff',
                                  lambda r: 'ff kept'):
            out = steps._r_friction(s, after_drag) if after_drag else \
                steps._r_friction(s)
        self.gains_at_ff = gains_at_ff
        return out, regs, writes, ff

    def test_a_new_j_leaves_the_velocity_gains(self):
        out, regs, writes, ff = self._run(5e-6, 6e-6)
        self.assertEqual(regs, {'vel_kp': 0.004, 'vel_ki': 0.2})
        self.assertEqual(writes, [('j', 'tc', 'b')])
        self.assertEqual(ff, [2.0])               # mNm
        self.assertEqual(self.asked, [])
        self.assertNotIn('velocity gains', out)
        self.assertIn('ff kept', out)

    def test_after_drag_runs_between_the_drag_and_the_ff(self):
        seen = []

        def hook(s, j_was, j):
            seen.append((j_was, j))
            steptest.write(s.dev, 'vel_kp', 1.0)
            return 'hooked'
        out, regs, _, ff = self._run(5e-6, 6e-6, after_drag=hook)
        self.assertEqual(seen, [(5e-6, 6e-6)])
        self.assertEqual(self.gains_at_ff[0]['vel_kp'], 1.0)
        self.assertIn('; hooked', out)

    def test_firmware_without_the_ff_gets_the_drag_alone(self):
        out, _, writes, ff = self._run(6e-6, 6e-6, has_ff=False)
        self.assertEqual(writes, [('j', 'tc', 'b')])
        self.assertEqual(ff, [])
        self.assertIn('no Coulomb ff', out)

    def test_no_rescale_left_in_p4core(self):
        self.assertFalse(hasattr(steps, '_rescale_for_j'))
        self.assertFalse(hasattr(steps, 'J_RESCALE_MIN'))


if __name__ == '__main__':
    unittest.main()


class TestStageDragGuard(unittest.TestCase):
    """A drag fit with a non-positive J is not a rotor (the WAM4's
    J -4.9e-06 at 0.5/1 A, 2026-11-23): the step fails before J or the
    friction reach the values."""

    def _rep(self):
        rep = mock.Mock()
        rep.values = {'kt_nm_per_a': 0.564, 'ld_h': 1.4e-3, 'lq_h': 2.4e-3,
                      'r_ohm': 1.87}
        rep.ctx = {'s': {'i_max_a': 10.0, 'v_nom': 48.0},
                   'coast': {'j_kg_m2': 1.08e-4},
                   'no_load': {'no_load_ideal_rpm': 995.0},
                   'i_test': 1.5, 'bus': 48.0}
        rep.raw = {}
        return rep

    def _run(self, j):
        dev = mock.Mock()
        dev.estim_drag.return_value = {'j_kg_m2': j, 'a_nm': 0.30,
                                       'b_nm_s_per_rad': 6.7e-3}
        rep = self._rep()
        lim = mock.Mock(max_speed_rpm=1000.0)
        with mock.patch.object(commission, 'drag_current',
                               lambda *a: 0.5), \
                mock.patch.object(commission, 'supply_limited_rpm',
                                  lambda *a: None):
            commission._stage_drag(dev, lim, None, rep)
        return rep

    def test_negative_j_fails_the_step(self):
        with self.assertRaises(commission.SysidError) as cm:
            self._run(-4.9e-6)
        self.assertIn('drag fit gave J', str(cm.exception))

    def test_zero_j_fails_the_step(self):
        with self.assertRaises(commission.SysidError):
            self._run(0.0)

    def test_a_j_off_the_coast_fails_the_step(self):
        """The WAM4 at 250 rad/s^2: both drag runs biased alike, J -34 %."""
        with self.assertRaises(commission.SysidError) as cm:
            self._run(0.66e-4)
        self.assertIn("off the coast's", str(cm.exception))

    def test_a_j_inside_the_agreement_passes(self):
        rep = self._run(1.0e-4)
        self.assertAlmostEqual(rep.values['j_kg_m2'], 1.0e-4)

    def test_the_coast_friction_rate_reaches_the_drag(self):
        dev = mock.Mock()
        dev.estim_drag.return_value = {'j_kg_m2': 1.0e-4, 'a_nm': 0.024,
                                       'b_nm_s_per_rad': 2.5e-4}
        rep = self._rep()
        rep.ctx['coast']['tc_over_j'] = 236.0
        rep.ctx['coast']['b_over_j'] = 3.0
        with mock.patch.object(commission, 'drag_current',
                               lambda *a: 0.5), \
                mock.patch.object(commission, 'supply_limited_rpm',
                                  lambda *a: None):
            commission._stage_drag(dev, mock.Mock(max_speed_rpm=1000.0),
                                   None, rep)
        self.assertEqual(
            dev.estim_drag.call_args.kwargs['friction_accel_rad_s2'], 236.0)
