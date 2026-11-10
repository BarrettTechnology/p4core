"""Which 0x1010 save each job uses, on firmware with and without the ranges."""
import unittest
from unittest import mock

from canopen.sdo.exceptions import SdoAbortedError, SdoCommunicationError

from p4core import config_csv, nvsave, ops

SIG = (0x65766173).to_bytes(4, 'little')


class Sdo:
    RESPONSE_TIMEOUT = 0.3

    def __init__(self, ranges, pending_bit=True, pending_reads=2, enabled=False):
        self.ranges = ranges
        self.pending_bit = pending_bit      # firmware shows 0x1010 bit 2
        self.pending_reads = pending_reads  # reads that still show it per save
        self.enabled = enabled              # outputs on: saves refused
        self.pending = 0
        self.saves = []
        self.timeouts = []
        self.polls = []
        self.od = {}

    def upload(self, index, sub):
        if index == 0x1010:
            if sub == 5 and not self.ranges:
                raise SdoAbortedError(0x06020000)
            self.polls.append(sub)
            status = 1
            if self.pending_bit and self.pending:
                self.pending -= 1
                status |= nvsave.SAVE_PENDING
            return status.to_bytes(4, 'little')
        return self.od.get((index, sub), (0x205).to_bytes(4, 'little'))

    def download(self, index, sub, data):
        if index == 0x1010:
            if self.enabled:
                raise SdoAbortedError(0x08000022)
            self.pending = self.pending_reads
            self.saves.append(sub)
            self.timeouts.append(self.RESPONSE_TIMEOUT)
            assert data == SIG
        else:
            self.od[index, sub] = data


def node(ranges, **kw):
    return mock.Mock(sdo=Sdo(ranges, **kw), id=5)


def setUpModule():
    # Polls sleep for real otherwise; the Polling tests use a fake clock.
    global _sleep
    _sleep = mock.patch.object(nvsave.time, 'sleep')
    _sleep.start()


def tearDownModule():
    _sleep.stop()


class Clock:
    """A fake time: sleep() advances it."""

    def __init__(self):
        self.now = 0.0
        self.sleeps = []

    def __call__(self):
        return self.now

    def sleep(self, s):
        self.sleeps.append(s)
        self.now += s


class Saves(unittest.TestCase):
    def test_settings_leave_communication_out(self):
        n = node(True)
        nvsave.save_settings(n)
        self.assertEqual(sorted(n.sdo.saves), [nvsave.APP, nvsave.MFG])

    def test_old_firmware_falls_back_because_its_ranged_saves_store_nothing(self):
        for save in (nvsave.save_settings, nvsave.save_comm, nvsave.save_all):
            n = node(False)
            save(n)
            self.assertEqual(n.sdo.saves, [nvsave.ALL], save.__name__)

    def test_comm_and_all(self):
        n = node(True)
        nvsave.save_comm(n)
        nvsave.save_all(n)
        self.assertEqual(n.sdo.saves, [nvsave.COMM, nvsave.ALL])

    def test_no_answer_is_a_failed_save_not_old_firmware(self):
        n = node(True)
        n.sdo.upload = mock.Mock(side_effect=SdoCommunicationError('no answer'))
        with self.assertRaises(SdoCommunicationError):
            nvsave.save_settings(n)
        self.assertEqual(n.sdo.saves, [])

    def test_the_answer_is_not_cached_across_a_flash(self):
        n = node(True)
        nvsave.save_settings(n)
        n.sdo.ranges = False
        n.sdo.saves.clear()
        nvsave.save_settings(n)
        self.assertEqual(n.sdo.saves, [nvsave.ALL])

    def test_a_save_gets_time_and_puts_the_timeout_back(self):
        n = node(True)
        nvsave.save_settings(n)
        self.assertEqual(n.sdo.timeouts, [nvsave.TIMEOUT_S] * 2)
        self.assertEqual(n.sdo.RESPONSE_TIMEOUT, 0.3)


class Polling(unittest.TestCase):
    def wait(self, n, sub):
        clock = Clock()
        nvsave.wait_committed(n, sub, sleep=clock.sleep, clock=clock)
        return clock

    def test_reads_the_sub_until_the_pending_bit_clears(self):
        n = node(True, pending_reads=3)
        n.sdo.pending = 3
        clock = self.wait(n, nvsave.APP)
        self.assertEqual(n.sdo.polls, [nvsave.APP] * 4)
        self.assertEqual(clock.sleeps, [nvsave.POLL_INTERVAL_S] * 3)

    def test_firmware_without_the_bit_gets_one_read_and_the_fallback_wait(self):
        n = node(True, pending_bit=False)
        n.sdo.pending = 3
        clock = self.wait(n, nvsave.APP)
        self.assertEqual(n.sdo.polls, [nvsave.APP])
        self.assertEqual(clock.sleeps, [nvsave.FALLBACK_WAIT_S])

    def test_an_unreadable_sub_gets_the_fallback_wait(self):
        n = node(True)
        n.sdo.upload = mock.Mock(side_effect=SdoAbortedError(0x06010001))
        clock = self.wait(n, nvsave.ALL)
        self.assertEqual(clock.sleeps, [nvsave.FALLBACK_WAIT_S])

    def test_a_bit_that_never_clears_times_out(self):
        n = node(True)
        n.sdo.pending = 10 ** 6
        with self.assertRaises(nvsave.SaveTimeout) as cm:
            self.wait(n, nvsave.MFG)
        self.assertIn('pending', str(cm.exception))
        self.assertTrue(isinstance(cm.exception, SdoCommunicationError))
        # ~POLL_TIMEOUT_S of polling, not longer
        self.assertLess(len(n.sdo.polls), nvsave.POLL_TIMEOUT_S / nvsave.POLL_INTERVAL_S + 3)

    def test_save_all_gets_the_longer_deadline(self):
        n = node(True)
        n.sdo.pending = 10 ** 6
        clock = Clock()
        with self.assertRaises(nvsave.SaveTimeout):
            nvsave.wait_committed(n, nvsave.ALL, sleep=clock.sleep, clock=clock)
        self.assertGreaterEqual(clock.now, nvsave.POLL_TIMEOUT_ALL_S)

    def test_a_busy_node_is_asked_again_until_the_deadline(self):
        n = node(True)
        answers = [SdoCommunicationError('busy'), (1 | 4).to_bytes(4, 'little'),
                   (1).to_bytes(4, 'little')]
        n.sdo.upload = mock.Mock(side_effect=answers)
        clock = self.wait(n, nvsave.COMM)
        self.assertEqual(clock.sleeps, [nvsave.POLL_INTERVAL_S] * 2)

    def test_a_node_that_stops_answering_times_out(self):
        n = node(True)
        n.sdo.upload = mock.Mock(side_effect=SdoCommunicationError('gone'))
        with self.assertRaises(nvsave.SaveTimeout):
            self.wait(n, nvsave.COMM)

    def test_every_save_polls_its_own_sub(self):
        n = node(True)
        nvsave.save_settings(n)
        self.assertEqual(n.sdo.polls,
                         [nvsave.MFG] + [nvsave.MFG] * 3 + [nvsave.APP] * 3)  # probe, then each save

    def test_the_poll_runs_with_the_long_timeout(self):
        n = node(True)
        seen = []
        upload = n.sdo.upload
        n.sdo.upload = lambda i, s: (seen.append(n.sdo.RESPONSE_TIMEOUT), upload(i, s))[1]
        nvsave.save_comm(n)
        self.assertEqual(seen[1:], [nvsave.TIMEOUT_S] * 3)   # [0]: has_ranges


class Refused(unittest.TestCase):
    def test_a_save_while_enabled_says_to_disable_the_drive(self):
        n = node(True, enabled=True)
        with self.assertRaises(nvsave.SaveRefused) as cm:
            nvsave.save_settings(n)
        self.assertIn('disable the drive before saving', str(cm.exception))
        self.assertIn('node 5', str(cm.exception))
        self.assertIsInstance(cm.exception, SdoAbortedError)
        self.assertEqual(cm.exception.code, 0x08000022)
        self.assertEqual(n.sdo.polls, [nvsave.MFG])     # has_ranges only

    def test_other_aborts_pass_through(self):
        n = node(True)
        n.sdo.download = mock.Mock(side_effect=SdoAbortedError(0x08000020))
        with self.assertRaises(SdoAbortedError) as cm:
            nvsave.save_all(n)
        self.assertNotIsInstance(cm.exception, nvsave.SaveRefused)

    def test_the_cli_prints_the_advice(self):
        from p4core import cli
        self.assertTrue(issubclass(nvsave.SaveRefused, cli.SdoError))


class Jobs(unittest.TestCase):
    def test_a_node_id_change_saves_communication(self):
        n = node(True)
        ok, _ = config_csv.remap_pdo_cob_ids(n, 5, verify=False, log=lambda m: None)
        self.assertTrue(ok)
        self.assertEqual(n.sdo.saves, [nvsave.COMM])

    def test_a_config_load_starts_from_nv_then_stores_every_range(self):
        calls = []
        session = mock.Mock()
        session.reset_node.side_effect = lambda *a, **k: calls.append(('reset', k)) or 'node'
        session.save_all.side_effect = lambda n: calls.append(('save_all', n))
        result = mock.Mock(written=1, errors=[])
        with mock.patch.object(config_csv, 'load', side_effect=lambda *a, **k: (
                calls.append(('load', a[0])), result)[1]):
            ops.load_config(session, 5, 'x.csv')
        self.assertEqual(calls, [('reset', {'reselect': False}), ('load', 'node'),
                                 ('save_all', 'node'), ('reset', {})])

    def test_a_ram_only_load_does_not_reboot(self):
        session = mock.Mock()
        with mock.patch.object(config_csv, 'load', return_value=mock.Mock(written=1, errors=[])):
            ops.load_config(session, 5, 'x.csv', save=False, reboot=False)
        session.reset_node.assert_not_called()
        session.save_all.assert_not_called()


if __name__ == '__main__':
    unittest.main()
