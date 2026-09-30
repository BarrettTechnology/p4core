"""tuning: pucktuner's gain design, pinned against the formulas it used."""

import math
import os
import struct
import sys
import unittest

sys.path.insert(0, os.path.dirname(__file__))

from fake import FakeNode                                        # noqa: E402

from p4core import tuning                                        # noqa: E402

QUIET = dict(log=lambda *a: None)


def dq(r_ll, l_ll):
    rdq = r_ll / 2 * (1 + 0.0039 * 20)
    ldq = l_ll / 2 * 0.8
    return rdq, ldq, ldq / rdq


class TestCurrentLoop(unittest.TestCase):

    def test_a_slow_motor_at_its_corner(self):
        rdq, ldq, tau = dq(0.2, 180e-6)
        natural = 1 / (tau * 2 * math.pi)
        t = tuning.current_gains(0.2, 180e-6, **QUIET)
        # stability needs 25 x 238 Hz x 4 = 23.8 kHz -> 32 kHz
        self.assertEqual(t.control_rate, 32000 / 4)
        self.assertFalse(t.clamped)
        self.assertAlmostEqual(t.control_bw, natural)
        self.assertAlmostEqual(t.kp, 4 * math.pi * natural * ldq - rdq)
        self.assertAlmostEqual(t.ki, (2 * math.pi * natural) ** 2 * ldq)
        self.assertEqual(t.control_gain, 1.0)

    def test_the_back_off_floors_at_min_gain(self):
        _rdq, _ldq, tau = dq(0.2, 20e-6)
        natural = 1 / (tau * 2 * math.pi)
        t = tuning.current_gains(0.2, 20e-6, **QUIET)
        self.assertEqual(t.control_rate, 100000 / 4)
        self.assertTrue(t.clamped)
        self.assertAlmostEqual(t.control_gain, tuning.MIN_CURRENT_GAIN)
        self.assertAlmostEqual(t.control_bw, 0.6 * natural)

    def test_the_override_skips_the_back_off(self):
        t = tuning.current_gains(0.2, 20e-6, override_backoff=True, **QUIET)
        self.assertEqual(t.control_gain, 1.0)
        self.assertLess(t.margin, tuning.STABILITY_BACKOFF_MARGIN)

    def test_nyquist_wins_over_the_floor(self):
        t = tuning.current_gains(0.2, 5e-6, **QUIET)
        self.assertAlmostEqual(t.control_bw, 0.4 * 25000 / 2)
        self.assertTrue(t.clamped)
        self.assertLess(t.control_gain, tuning.MIN_CURRENT_GAIN)

    def test_a_negative_kp_is_zeroed(self):
        t = tuning.current_gains(10.0, 180e-6, pwm_hz=16000, **QUIET)
        self.assertEqual(t.kp, 0)

    def test_the_user_pwm_wins(self):
        self.assertEqual(tuning.select_pwm(0.2, 20e-6, user_pwm=40000,
                                           **QUIET).pwm_hz, 40000)
        # no R/L: keep a sane stored PWM, else the floor
        self.assertEqual(tuning.select_pwm(0, 0, stored_pwm=60000).pwm_hz, 60000)
        self.assertEqual(tuning.select_pwm(0, 0, stored_pwm=5).pwm_hz, 16000)

    def test_no_r_or_l_is_an_error(self):
        with self.assertRaises(ValueError):
            tuning.current_gains(0, 0, **QUIET)


class TestVelocityLoop(unittest.TestCase):

    def test_auto_bandwidth(self):
        self.assertEqual(tuning.auto_velocity_bandwidth(50)[0], 4.0)
        self.assertEqual(tuning.auto_velocity_bandwidth(200)[0], 16.0)
        self.assertEqual(tuning.auto_velocity_bandwidth(0)[0], 4.0)
        # a rough inner loop gentles it: 16 x 16/25
        self.assertAlmostEqual(tuning.auto_velocity_bandwidth(200, 16)[0], 10.2)
        self.assertEqual(tuning.auto_velocity_bandwidth(10)[0], 1.5)

    def test_gains(self):
        t = tuning.velocity_gains(66, 300, spd_trq_grad=10, vel_filter_hz=50,
                                  **QUIET)
        io = 1e6 * (2 * math.pi / 4096) / 300
        w = 2 * math.pi * 4.0
        self.assertAlmostEqual(t.kp, io * 2 * w * 66e-7)
        self.assertAlmostEqual(t.ki, io * 66e-7 * w * w / 1000)
        user = tuning.velocity_gains(66, 300, spd_trq_grad=10, bandwidth_hz=8,
                                     **QUIET)
        self.assertEqual(user.bandwidth_hz, 8)

    def test_the_gradient_comes_from_the_speeds(self):
        t = tuning.velocity_gains(66, 300, no_load_spd=5000, nominal_spd=4000,
                                  cont_torque=100, **QUIET)
        self.assertEqual(t.spd_trq_grad, 10)

    def test_position_kp(self):
        self.assertEqual(tuning.position_kp(4), 3)
        self.assertEqual(tuning.position_kp(1.5), 1)
        self.assertEqual(tuning.position_kp(None), 2)


def puck(over=None):
    node = FakeNode(values={
        (0x3011, 5): 200, (0x3011, 6): 180, (0x3011, 4): 30,
        (0x3011, 3): 14, (0x6076, 0): 300, (0x3001, 1): 16000,
        (0x3011, 7): 66 * 2 ** 15, (0x3024, 1): 1000, (0x3024, 2): 100,
        (0x3024, 3): 100, (0x3024, 4): 0, (0x3024, 5): 100,
        (0x3024, 6): 0, (0x3024, 7): 0, (0x3024, 8): 0, (0x2100, 1): 50,
    })
    node.values.update(over or {})
    return node


class FakeSession:
    def __init__(self, node):
        self.node, self.calls = node, []

    def add_node(self, node_id):
        return self.node

    def save_all(self, node):
        self.calls.append('save')

    def reset_node(self, node_id):
        self.calls.append('reset')


class TestDrive(unittest.TestCase):

    def test_read_and_design(self):
        p = tuning.read_params(puck())
        self.assertEqual((p['resistance'], p['inductance'], p['j']),
                         (0.2, 180e-6, 66))
        self.assertIsNone(p['bandwidth_hz'])
        cur, vel, kp = tuning.design(p, **QUIET)
        self.assertEqual(cur.control_rate, 8000)
        self.assertEqual(vel.bandwidth_hz, 4.0)
        self.assertEqual(kp, 3)

    def test_apply_writes_gains_and_the_pwm(self):
        node = puck()
        cur, vel, kp = tuning.design(tuning.read_params(node), **QUIET)
        session = FakeSession(node)
        tuning.apply(session, 5, cur, vel, kp)
        down = {(i, s): d for i, s, d in node.downloads}
        self.assertAlmostEqual(struct.unpack('<f', down[(0x2380, 1)])[0],
                               cur.kp, places=6)
        self.assertEqual(down[(0x3024, 2)], bytes([100]))
        self.assertEqual(down[(0x2382, 1)], (3).to_bytes(2, 'little'))
        self.assertEqual(down[(0x3001, 1)], (32000).to_bytes(4, 'little'))
        self.assertEqual(session.calls, ['save', 'reset'])

    def test_an_unchanged_pwm_is_not_rewritten(self):
        node = puck({(0x3001, 1): 32000})
        cur, _vel, _kp = tuning.design(tuning.read_params(node), **QUIET)
        tuning.apply(FakeSession(node), 5, cur, save=False, reboot=False)
        self.assertNotIn((0x3001, 1), [(i, s) for i, s, _d in node.downloads])


if __name__ == '__main__':
    unittest.main()
