"""calibrate.gain_fit(): the Beta gain factor from a rotating current vector."""
import math
import os
import sys
import unittest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))
from p4core.calibrate import GAIN_FIT_STEPS, gain_fit  # noqa: E402


def rotate(a_gain, b_gain, ripple6=0.0, bias=(32000, 32900)):
    """Filtered (Q12.4) readings over one electrical revolution. |I| carries a
    six-fold term (dead time in voltage mode)."""
    out = []
    for k in range(GAIN_FIT_STEPS):
        t = 2 * math.pi * k / GAIN_FIT_STEPS
        i = 1600 * (1 + ripple6 * math.cos(6 * t))
        out.append((t, bias[0] + a_gain * i * math.cos(t), bias[1] + b_gain * i * math.sin(t)))
    return out


class TestGainFit(unittest.TestCase):
    def ratio(self, **kw):
        a, b = gain_fit(rotate(**kw))
        return 4096 * a / b

    def test_matched_channels(self):
        self.assertAlmostEqual(self.ratio(a_gain=1.0, b_gain=1.0), 4096, delta=0.5)

    def test_mismatch_is_the_ratio(self):
        self.assertAlmostEqual(self.ratio(a_gain=1.0, b_gain=1.05), 4096 / 1.05, delta=0.5)

    def test_dead_time_distortion_cancels(self):
        # A 25% six-fold swing in |I| is what the old two-angle method read as
        # a 25% channel mismatch; it has no fundamental.
        self.assertAlmostEqual(self.ratio(a_gain=1.0, b_gain=1.0, ripple6=0.25), 4096, delta=0.5)

    def test_amplitude_is_in_filtered_counts(self):
        a, b = gain_fit(rotate(a_gain=1.0, b_gain=1.0))
        self.assertAlmostEqual(a, 1600, delta=0.01)
        self.assertAlmostEqual(b, 1600, delta=0.01)


if __name__ == '__main__':
    unittest.main()
