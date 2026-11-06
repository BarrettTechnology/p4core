"""bring_iface_up tries the bench FD timing, then default FD timing, then classic."""
import subprocess
import unittest
from unittest import mock

from p4core import can_backend


def _fake_ip(fail_until):
    """An `ip` that rejects the first *fail_until* `type can` settings."""
    calls = []

    def run(cmd, **kw):
        calls.append(cmd)
        if 'type' in cmd and sum('type' in c for c in calls) <= fail_until:
            raise subprocess.CalledProcessError(2, cmd)
        return subprocess.CompletedProcess(cmd, 0)
    return run, calls


class BringIfaceUpTest(unittest.TestCase):

    def _run(self, fail_until, **kw):
        run, calls = _fake_ip(fail_until)
        with mock.patch.object(can_backend.platform, 'system', return_value='Linux'), \
                mock.patch.object(can_backend.subprocess, 'run', side_effect=run):
            ok = can_backend.bring_iface_up('can0', **kw)
        return ok, [c[c.index('type') + 2:] for c in calls if 'type' in c and 'can' in c
                    and 'restart-ms' not in c]

    def test_bench_timing_first(self):
        ok, tries = self._run(0)
        self.assertTrue(ok)
        self.assertEqual(tries, [['bitrate', '1000000', 'dbitrate', '5000000',
                                  'sample-point', '0.687', 'sjw', '50',
                                  'dsample-point', '0.687', 'dsjw', '10', 'fd', 'on']])

    def test_falls_back_to_default_fd_then_classic(self):
        ok, tries = self._run(2)
        self.assertTrue(ok)
        self.assertEqual(tries[1], ['bitrate', '1000000', 'dbitrate', '5000000', 'fd', 'on'])
        self.assertEqual(tries[2], ['bitrate', '1000000'])

    def test_other_rates_skip_the_bench_timing(self):
        ok, tries = self._run(0, bitrate=500_000, data_bitrate=2_000_000)
        self.assertTrue(ok)
        self.assertEqual(tries, [['bitrate', '500000', 'dbitrate', '2000000', 'fd', 'on']])

    def test_nothing_takes(self):
        ok, tries = self._run(3)
        self.assertFalse(ok)
        self.assertEqual(len(tries), 3)


if __name__ == '__main__':
    unittest.main()
