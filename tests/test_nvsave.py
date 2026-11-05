"""Which 0x1010 save each job uses, on firmware with and without the ranges."""
import unittest
from unittest import mock

from canopen.sdo.exceptions import SdoAbortedError, SdoCommunicationError

from p4core import config_csv, nvsave, ops

SIG = (0x65766173).to_bytes(4, 'little')


class Sdo:
    RESPONSE_TIMEOUT = 0.3

    def __init__(self, ranges):
        self.ranges = ranges
        self.saves = []
        self.timeouts = []
        self.od = {}

    def upload(self, index, sub):
        if (index, sub) == (0x1010, 5):
            if not self.ranges:
                raise SdoAbortedError(0x06020000)
            return b'\x01\0\0\0'
        return self.od.get((index, sub), (0x205).to_bytes(4, 'little'))

    def download(self, index, sub, data):
        if index == 0x1010:
            self.saves.append(sub)
            self.timeouts.append(self.RESPONSE_TIMEOUT)
            assert data == SIG
        else:
            self.od[index, sub] = data


def node(ranges):
    return mock.Mock(sdo=Sdo(ranges), id=5)


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
