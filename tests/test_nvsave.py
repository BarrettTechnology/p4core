"""Which 0x1010 save each job uses, on firmware with and without the ranges."""
import unittest
from unittest import mock

from canopen.sdo.exceptions import SdoAbortedError, SdoCommunicationError

from p4core import config_csv, nvsave, ops

SIG = (0x65766173).to_bytes(4, 'little')


class Sdo:
    RESPONSE_TIMEOUT = 0.3

    def __init__(self, ranges, pending_bit=True, pending_reads=2, enabled=False,
                 operational=False):
        self.ranges = ranges
        self.pending_bit = pending_bit      # firmware shows 0x1010 bit 2
        self.pending_reads = pending_reads  # reads that still show it per save
        self.enabled = enabled              # outputs on: saves refused
        self.operational = operational      # NMT OPERATIONAL: 5.0.7+ refuses saves
        self.log = []                       # ('nmt', cmd) / ('save', sub) / ('poll', sub)
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
            self.log.append(('poll', sub))
            status = 1
            if self.pending_bit and self.pending:
                self.pending -= 1
                status |= nvsave.SAVE_PENDING
            return status.to_bytes(4, 'little')
        if (index, sub) == (0x6041, 0):
            return (0x0637 if self.enabled else 0x0631).to_bytes(2, 'little')
        return self.od.get((index, sub), (0x205).to_bytes(4, 'little'))

    def download(self, index, sub, data):
        if index == 0x1010:
            if self.enabled or self.operational:
                raise SdoAbortedError(0x08000022)
            self.log.append(('save', sub))
            self.pending = self.pending_reads
            self.saves.append(sub)
            self.timeouts.append(self.RESPONSE_TIMEOUT)
            assert data == SIG
        else:
            self.od[index, sub] = data


def node(ranges, **kw):
    n = mock.Mock(sdo=Sdo(ranges, **kw), id=5)

    def nmt(cmd):
        n.sdo.log.append(('nmt', cmd))
        n.sdo.operational = cmd == nvsave.NMT_START

    n.nmt.send_command.side_effect = nmt
    return n


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


class PreOp(unittest.TestCase):
    """5.0.7+: NV writes in NMT PRE-OPERATIONAL only."""

    def test_an_operational_node_is_saved_in_pre_op_and_started_again(self):
        n = node(True, operational=True)
        nvsave.save_comm(n)
        self.assertEqual(n.sdo.saves, [nvsave.COMM])
        log = [e for e in n.sdo.log if e[0] != 'poll']
        self.assertEqual(log, [('nmt', 0x80), ('save', nvsave.COMM), ('nmt', 0x01)])
        # the commit is waited for before the start
        start = n.sdo.log.index(('nmt', 0x01))
        self.assertEqual(n.sdo.log[start - 1], ('poll', nvsave.COMM))
        self.assertTrue(n.sdo.operational)

    def test_a_node_in_pre_op_gets_no_nmt(self):
        n = node(True)
        nvsave.save_settings(n)
        n.nmt.send_command.assert_not_called()

    def test_enabled_is_refused_without_touching_nmt(self):
        n = node(True, enabled=True, operational=True)
        with self.assertRaises(nvsave.SaveRefused):
            nvsave.save_all(n)
        n.nmt.send_command.assert_not_called()

    def test_without_restore_the_node_stays_in_pre_op(self):
        n = node(True, operational=True)
        nvsave.in_pre_op(n, lambda: n.sdo.download(0x1010, 1, SIG), restore=False)
        self.assertEqual([e for e in n.sdo.log if e[0] == 'nmt'], [('nmt', 0x80)])

    def test_the_node_is_started_again_when_the_retry_fails(self):
        n = node(True, operational=True)
        calls = []

        def action():
            calls.append(1)
            raise SdoAbortedError(0x06090030)

        with self.assertRaises(SdoAbortedError):
            nvsave.in_pre_op(n, lambda: (calls.append(0), n.sdo.download(0x1010, 1, SIG))
                             if not calls else action())
        self.assertEqual([e for e in n.sdo.log if e[0] == 'nmt'],
                         [('nmt', 0x80), ('nmt', 0x01)])

    def test_other_aborts_are_not_retried(self):
        n = node(True)
        n.sdo.download = mock.Mock(side_effect=SdoAbortedError(0x06010002))
        with self.assertRaises(SdoAbortedError):
            nvsave.in_pre_op(n, lambda: n.sdo.download(0x21B0, 0, b'\x05'))
        n.nmt.send_command.assert_not_called()


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
