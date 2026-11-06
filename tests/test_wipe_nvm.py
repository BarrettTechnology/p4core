"""ops.wipe_nvm: back up, erase, reload the blank, keep the bus settings."""

import os
import tempfile
import unittest

from p4core import config_csv, firmware, ops
from p4core.reporter import RecordingReporter


class Sdo:
    RESPONSE_TIMEOUT = 0.3

    def __init__(self, node):
        self.node = node

    def upload(self, index, sub):
        if (index, sub) in self.node.absent:
            raise ValueError('abort')
        value = self.node.values.get((index, sub), 0)
        return value if isinstance(value, bytes) else value.to_bytes(4, 'little')

    def download(self, index, sub, data, force_segment=False):
        self.node.writes.append((index, sub, data))
        self.node.values[(index, sub)] = int.from_bytes(data, 'little')


class Node:
    id = 3
    object_dictionary = None

    def __init__(self, **values):
        self.absent = set()
        self.writes = []
        self.values = {(0x1018, 2): 0x164b, (0x1018, 4): 77, (0x21B0, 0): 3,
                       (0x3001, 9): 240}
        self.sdo = Sdo(self)


class Wipe(unittest.TestCase):
    def setUp(self):
        self.node = Node()
        self.reboots = 0
        self.dir = tempfile.TemporaryDirectory()
        self.backup = os.path.join(self.dir.name, 'b', 'n3.csv')
        self.reporter = RecordingReporter()
        self.reporter.sleep = lambda s: None
        self.loaded = []
        self.saved = (firmware.detect, config_csv.template_for,
                      config_csv.build_from_template, config_csv.load,
                      ops._calibrate.read_bus_v)
        firmware.detect = lambda n: firmware.Firmware(
            (5, 0, 5), '5.0.5', frozenset(['nvm_wipe']))
        config_csv.template_for = lambda n: 'P4-16-Blank.csv'

        def build(node, template, out, reporter):
            self.order('backup')
            with open(out, 'w') as f:
                f.write('x')
            return 1
        config_csv.build_from_template = build

        def load(node, path, node_id, reporter, keep=None, force=False):
            self.order('load')
            self.loaded.append(keep)
            result = config_csv.ApplyResult()
            return result
        config_csv.load = load
        ops._calibrate.read_bus_v = lambda n: 24.0
        self.events = []

    def tearDown(self):
        (firmware.detect, config_csv.template_for,
         config_csv.build_from_template, config_csv.load,
         ops._calibrate.read_bus_v) = self.saved
        self.dir.cleanup()

    def order(self, what):
        self.events.append(what)

    def reboot(self):
        self.reboots += 1
        self.order('reboot')
        return self.node

    def wipe(self):
        return ops.wipe_node(self.node, self.reboot, self.reporter, self.backup)

    def test_backs_up_before_erasing_and_reloads_after(self):
        real = self.node.sdo.download

        def download(index, sub, data, force_segment=False):
            self.order((index, sub))
            real(index, sub, data)
        self.node.sdo.download = download
        result = self.wipe()
        self.assertEqual(self.events, ['backup', (0x1011, 3), (0x1011, 5),
                                       'reboot', 'load',
                                       (0x1010, 5), (0x1010, 3), 'reboot'])
        self.assertTrue(result.ok)
        self.assertEqual(result.backup, self.backup)
        self.assertTrue(os.path.exists(self.backup))
        self.assertEqual(self.node.values[(0x1011, 3)], 0x64616F6C)
        self.assertEqual(self.node.values[(0x1011, 5)], 0x64616F6C)
        self.assertIs(self.loaded[0], config_csv.application_row)

    def test_keeps_all_of_identity(self):
        for sub in range(1, 7):
            self.assertIn((0x1018, sub), ops.WIPE_KEEPS)

    def _refuses(self, text):
        with self.assertRaisesRegex(ops.OperationError, text):
            self.wipe()
        self.assertEqual(self.node.writes, [])
        self.assertEqual(self.reboots, 0)
        self.assertFalse(os.path.exists(self.backup))

    def test_refuses_without_the_firmware(self):
        firmware.detect = lambda n: firmware.Firmware((5, 0, 4), '5.0.4', frozenset())
        self._refuses('update')

    def test_refuses_enabled(self):
        self.node.values[(0x6041, 0)] = 0x0027
        self._refuses('enabled')

    def test_refuses_without_a_template(self):
        def none(node):
            raise ValueError('no config template')
        config_csv.template_for = none
        self._refuses('no config template')

    def test_backup_failure_erases_nothing(self):
        def fail(*a):
            raise OSError('disk full')
        config_csv.build_from_template = fail
        with self.assertRaises(OSError):
            self.wipe()
        self.assertEqual(self.node.writes, [])

    def test_a_changed_identity_fails(self):
        def reboot():
            self.node.values[(0x1018, 2)] = 0
            return self.node
        self.reboot = reboot
        self.assertFalse(self.wipe().ok)
        self.assertTrue(any('0x1018,2' in t for t in self.reporter.texts('warn')))

    def test_warns_of_a_bus_off_nominal(self):
        ops._calibrate.read_bus_v = lambda n: 48.0
        self.wipe()
        self.assertTrue(any('0x3001,9' in t for t in self.reporter.texts('warn')))

    def test_no_template_skips_the_load(self):
        result = ops.wipe_node(self.node, self.reboot, self.reporter, self.backup,
                               template=False)
        self.assertEqual(self.loaded, [])
        self.assertEqual(self.reboots, 1)
        self.assertIsNone(result.template)
        self.assertTrue(result.ok)
        self.assertTrue(os.path.exists(self.backup))       # still backed up
        self.assertTrue(any('load the configuration CSV' in t
                            for t in self.reporter.texts('warn')))

    def test_legacy_refused_unless_asked(self):
        firmware.detect = lambda n: firmware.Firmware((4, 3, 5), '4.3.5', frozenset())
        self._refuses('update')

    def test_legacy_writes_defaults_and_backs_up_as_json(self):
        firmware.detect = lambda n: firmware.Firmware((4, 3, 5), '4.3.5', frozenset())
        calls = []
        saved = ops.nvjson.save, ops.nvjson.legacy_wipe

        def save(node, path):
            calls.append(('save', path))
            with open(path, 'w') as f:
                f.write('{}')

        def legacy_wipe(node, reboot, reporter):
            calls.append('wipe')
            return dict(written=98, failed=[], differ=[], node=reboot())
        ops.nvjson.save, ops.nvjson.legacy_wipe = save, legacy_wipe
        try:
            result = ops.wipe_node(self.node, self.reboot, self.reporter,
                                   os.path.join(self.dir.name, 'n3.json'),
                                   legacy=True)
        finally:
            ops.nvjson.save, ops.nvjson.legacy_wipe = saved
        self.assertEqual([c if c == 'wipe' else c[0] for c in calls], ['save', 'wipe'])
        self.assertEqual(self.loaded, [])           # no 5.0 template on 4.3.x
        self.assertNotIn((0x1011, 5), [w[:2] for w in self.node.writes])
        self.assertTrue(result.ok)

    def test_legacy_backup_defaults_to_json(self):
        self.assertTrue(ops.default_wipe_backup(3, '.json').endswith('.json'))


class ApplicationRow(unittest.TestCase):
    def row(self, index, kind='entry'):
        return type('R', (), {'kind': kind, 'index': index})()

    def test_keeps_the_bus_settings(self):
        for index in ('0x1400', '0x1018', '0x1F80', '0x21B0', '0x21B2'):
            self.assertFalse(config_csv.application_row(self.row(index)), index)

    def test_loads_the_rest(self):
        for index in ('0x2000', '0x21B1', '0x3001', '0x6075'):
            self.assertTrue(config_csv.application_row(self.row(index)), index)
        self.assertTrue(config_csv.application_row(self.row(None, 'comment')))


if __name__ == '__main__':
    unittest.main()
