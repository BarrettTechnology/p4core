"""nvgate: 0x1010/0x1011 writes on firmware that saves only in NMT PRE-OP (5.0.7)."""
import unittest
from unittest import mock

from canopen.sdo.exceptions import SdoAbortedError

from p4core import nvgate

SIG = (0x65766173).to_bytes(4, 'little')


class Net:
    def __init__(self, sdo):
        self.sdo = sdo
        self.sent = []

    def send_message(self, cob, data):
        self.sent.append((cob, bytes(data)))
        cmd = data[0]
        if cmd == nvgate.NMT_ENTER_PREOP:
            self.sdo.nmt = 'PRE-OP'
        elif cmd == nvgate.NMT_START:
            self.sdo.nmt = 'OPERATIONAL'


class Drive:
    """A 5.0.7 drive's save rule, or an older drive's (gate=False)."""

    def __init__(self, gate=True, nmt='OPERATIONAL', enabled=False, pending_reads=0):
        self.gate, self.nmt, self.enabled = gate, nmt, enabled
        self.pending_reads = pending_reads
        self.rx_cobid = 0x600 + 127
        self.network = Net(self)
        self.saves = []
        self.writes = {}

    # the original SdoClient.download, as gated_download calls it: orig(sdo, ...)
    @staticmethod
    def orig(sdo, index, sub, data, *a, **k):
        if index in nvgate.NV_INDICES:
            if sdo.gate and (sdo.enabled or sdo.nmt != 'PRE-OP'):
                raise SdoAbortedError(nvgate.ABORT_DEVICE_STATE)
            sdo.saves.append((index, sub, sdo.nmt))
        else:
            sdo.writes[index, sub] = data

    def upload(self, index, sub):
        if (index, sub) == (0x6041, 0):
            return (0x0237 if self.enabled else 0x0231).to_bytes(2, 'little')
        if (index, sub) == (0x1010, 1):
            if self.pending_reads:
                self.pending_reads -= 1
                return (5).to_bytes(4, 'little')
            return (1).to_bytes(4, 'little')
        return b'\0\0\0\0'


def save(drive, index=0x1010, sub=1):
    nvgate.gated_download(Drive.orig, drive, index, sub, SIG)


class NvGateTest(unittest.TestCase):

    def setUp(self):
        patcher = mock.patch.object(nvgate.time, 'sleep')
        patcher.start()
        self.addCleanup(patcher.stop)

    def test_older_firmware_saves_at_once(self):
        d = Drive(gate=False)
        save(d)
        self.assertEqual(d.saves, [(0x1010, 1, 'OPERATIONAL')])
        self.assertEqual(d.network.sent, [])          # no NMT traffic at all

    def test_already_in_preop_saves_at_once(self):
        d = Drive(nmt='PRE-OP')
        save(d)
        self.assertEqual(d.saves, [(0x1010, 1, 'PRE-OP')])
        self.assertEqual(d.network.sent, [])

    def test_operational_saves_in_preop_and_returns(self):
        d = Drive(pending_reads=3)
        save(d)
        self.assertEqual(d.saves, [(0x1010, 1, 'PRE-OP')])
        self.assertEqual(d.network.sent, [(0, bytes([0x80, 127])), (0, bytes([0x01, 127]))])
        self.assertEqual(d.nmt, 'OPERATIONAL')
        self.assertEqual(d.pending_reads, 0)          # waited out bit 2

    def test_save_single_and_restore_are_gated_too(self):
        d = Drive()
        nvgate.gated_download(Drive.orig, d, 0x1010, 4, ((0x3008 << 8) | 7).to_bytes(4, 'little'))
        nvgate.gated_download(Drive.orig, d, 0x1011, 1, b'load')
        self.assertEqual([s[:2] for s in d.saves], [(0x1010, 4), (0x1011, 1)])
        self.assertEqual(d.nmt, 'OPERATIONAL')

    def test_outputs_on_is_refused_without_nmt(self):
        d = Drive(enabled=True)
        with self.assertRaises(nvgate.NmtSaveRefused) as cm:
            save(d)
        self.assertEqual(cm.exception.code, nvgate.ABORT_DEVICE_STATE)
        self.assertEqual(d.network.sent, [])          # never switched NMT under an enabled drive
        self.assertEqual(d.saves, [])

    def test_other_aborts_pass_through(self):
        d = Drive()

        def orig(sdo, *a, **k):
            raise SdoAbortedError(0x06090030)
        with self.assertRaises(SdoAbortedError) as cm:
            nvgate.gated_download(orig, d, 0x1010, 1, SIG)
        self.assertEqual(cm.exception.code, 0x06090030)
        self.assertEqual(d.network.sent, [])

    def test_other_indices_untouched(self):
        d = Drive()
        nvgate.gated_download(Drive.orig, d, 0x6040, 0, b'\x06\0')
        self.assertEqual(d.writes, {(0x6040, 0): b'\x06\0'})
        self.assertEqual(d.network.sent, [])

    def test_still_back_in_operational_when_the_retry_fails(self):
        d = Drive()

        def orig(sdo, index, *a, **k):
            raise SdoAbortedError(nvgate.ABORT_DEVICE_STATE)
        with self.assertRaises(SdoAbortedError):
            nvgate.gated_download(orig, d, 0x1010, 1, SIG)
        self.assertEqual(d.network.sent[-1], (0, bytes([0x01, 127])))

    def test_install_is_idempotent(self):
        from canopen.sdo.client import SdoClient
        nvgate.uninstall()
        before = SdoClient.download
        nvgate.install()
        once = SdoClient.download
        nvgate.install()
        self.assertIs(SdoClient.download, once)
        self.assertTrue(once._p4core_nvgate)
        nvgate.uninstall()
        self.assertIs(SdoClient.download, before)
        nvgate.install()                              # leave it as the session left it


if __name__ == '__main__':
    unittest.main()
