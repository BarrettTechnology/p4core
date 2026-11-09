"""can_backend: the CAN link's timing and fault counters from `ip -details
-json`, the /etc files that shadow puck-can-common's, and the bus fault a
calibration reports instead of "the supply browning out"."""

import json
import os
import subprocess
import sys
import tempfile
import unittest
from unittest import mock

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__),
                                                '..', '..')))

from p4core import can_backend                                    # noqa: E402

#: iproute2's JSON for a CANable at the kernel's default timing, after a
#: bus-off (the shape of `ip -details -statistics -json link show can0`).
IP_JSON = [{
    'ifname': 'can0',
    'linkinfo': {
        'info_kind': 'can',
        'info_data': {
            'ctrlmode': ['FD'], 'state': 'BUS-OFF',
            'berr_counter': {'tx': 248, 'rx': 0}, 'restart_ms': 0,
            'bittiming': {'bitrate': 1000000, 'sample_point': '0.750',
                          'tq': 6, 'prop_seg': 59, 'phase_seg1': 60,
                          'phase_seg2': 40, 'sjw': 20},
            'data_bittiming': {'bitrate': 5000000, 'sample_point': '0.750',
                               'sjw': 4}},
        'info_xstats': {'restarts': 0, 'bus_error': 0,
                        'arbitration_lost': 0, 'error_warning': 3,
                        'error_passive': 2, 'bus_off': 1}}}]


def _ip(stdout=None, rc=0):
    return mock.patch.object(
        can_backend.subprocess, 'run',
        return_value=subprocess.CompletedProcess(
            [], rc, stdout=json.dumps(stdout if stdout is not None else IP_JSON),
            stderr=''))


@mock.patch.object(can_backend.platform, 'system', return_value='Linux')
class TestIfaceDetails(unittest.TestCase):
    def test_parses_timing_and_counters(self, _):
        with _ip():
            d = can_backend.iface_details('can0')
        self.assertEqual(d['state'], 'BUS-OFF')
        self.assertEqual((d['bitrate'], d['sample_point'], d['sjw']),
                         (1000000, 0.75, 20))
        self.assertEqual((d['dbitrate'], d['dsjw']), (5000000, 4))
        self.assertEqual((d['bus_off'], d['error_passive']), (1, 2))

    def test_timing_text(self, _):
        with _ip():
            self.assertEqual(
                can_backend.iface_timing_text('can0'),
                'can0: 1000000 bit/s, sample point 0.750, SJW 20; data '
                '5000000 bit/s, sample point 0.750, SJW 4')

    def test_not_a_can_link(self, _):
        with _ip([{'ifname': 'eth0', 'linkinfo': {'info_kind': 'veth'}}]):
            self.assertIsNone(can_backend.iface_details('eth0'))
        with _ip(rc=1):
            self.assertIsNone(can_backend.iface_details('can9'))

    def test_bus_trouble(self, _):
        net = mock.Mock()
        net.bus.channel = 'can0'
        with _ip():
            self.assertEqual(can_backend.bus_trouble(net),
                             'can0 is BUS-OFF, went bus-off 1 time(s), went '
                             'error-passive 2 time(s)')
            # Counted since an earlier reading.
            ok = dict(IP_JSON[0]['linkinfo']['info_data'], state='ERROR-ACTIVE')
            calm = [{'linkinfo': dict(IP_JSON[0]['linkinfo'], info_data=ok)}]
        with _ip(calm):
            self.assertEqual(can_backend.bus_trouble(
                net, {'bus_off': 1, 'error_passive': 2}), '')

    def test_no_channel_no_verdict(self, _):
        net = mock.Mock()
        net.bus.channel = 'PCAN_USBBUS1'
        self.assertEqual(can_backend.bus_trouble(net), '')


@mock.patch.object(can_backend.platform, 'system', return_value='Linux')
class TestShadowedFiles(unittest.TestCase):
    def test_an_etc_copy_that_differs_is_reported(self, _):
        with tempfile.TemporaryDirectory() as d:
            usr, etc, same = (os.path.join(d, n) for n in ('usr', 'etc', 'same'))
            for path, text in ((usr, 'ours'), (etc, 'theirs'), (same, 'ours')):
                with open(path, 'w') as f:
                    f.write(text)
            files = ((usr, etc), (usr, same), (usr, os.path.join(d, 'none')))
            with mock.patch.object(can_backend, 'CAN_UP_FILES', files):
                self.assertEqual(can_backend.shadowed_can_files(), [(etc, usr)])
                msg = can_backend.shadowed_can_files_message()
        self.assertIn(etc, msg)
        self.assertIn('installer', msg)

    def test_none(self, _):
        with mock.patch.object(can_backend, 'CAN_UP_FILES', ()):
            self.assertEqual(can_backend.shadowed_can_files_message(), '')


class TestFaultCause(unittest.TestCase):
    """P4Drive._fault_cause: no error code and a bus-off on the adapter is
    a CAN fault, not "the supply browning out"."""

    def cause(self, last=0, hist=b'\0\0\0\0', trouble=''):
        from p4core.actuator.device import P4Drive
        dev = P4Drive.__new__(P4Drive)
        node = mock.MagicMock()
        node.sdo.__getitem__.return_value.raw = last
        node.sdo.upload.return_value = hist
        dev.node = node
        with mock.patch.object(can_backend, 'bus_trouble',
                               return_value=trouble):
            return dev._fault_cause('under it')

    def test_bus_off_is_not_the_supply(self):
        c = self.cause(trouble='can0 went bus-off 1 time(s)')
        self.assertIn('can0 went bus-off', c)
        self.assertNotIn('supply', c)

    def test_no_code_no_bus_fault_is_still_the_supply(self):
        self.assertIn('supply browning out', self.cause())

    def test_history_supplies_a_lost_code(self):
        self.assertIn('0x8140 CAN bus-off',
                      self.cause(hist=bytes([0x40, 0x81, 0, 0])))


if __name__ == '__main__':
    unittest.main()
