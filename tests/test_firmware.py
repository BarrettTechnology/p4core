"""p4core.firmware: the firmware line, its features and its units."""
import os
import struct
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))
from p4core import config_csv, firmware, tuning  # noqa: E402
from p4core.reporter import Reporter  # noqa: E402
from p4core.tests.fake import FakeNode, current_node, legacy_node  # noqa: E402


class _Notes(Reporter):
    def __init__(self):
        super().__init__()
        self.notes, self.warns = [], []

    def note(self, text):
        self.notes.append(text)

    def warn(self, text):
        self.warns.append(text)


class DetectTest(unittest.TestCase):
    def test_legacy(self):
        fw = firmware.detect(legacy_node())
        self.assertEqual((fw.version, fw.text), ((4, 3, 5), '4.3.5'))
        self.assertTrue(fw.legacy)
        self.assertTrue(fw.has('legacy_polarity'))
        self.assertEqual(fw.missing('capture', 'runout', 'cogging'),
                         ['capture', 'runout', 'cogging'])
        self.assertIn('4.3.x', fw.summary())

    def test_current(self):
        fw = firmware.detect(current_node())
        self.assertEqual(fw.version, (4, 5, 0))
        self.assertFalse(fw.legacy)
        self.assertTrue(fw.has('capture', 'runout', 'cogging', 'lq'))
        self.assertFalse(fw.has('legacy_polarity'))

    def test_cached_until_forget(self):
        node = legacy_node()
        firmware.detect(node)
        n = len(node.reads)
        firmware.has(node, 'capture')
        firmware.legacy(node)
        self.assertEqual(len(node.reads), n)
        node.values[(0x100A, 0)] = b'4.5.0'
        self.assertTrue(firmware.legacy(node))
        firmware.forget(node)
        self.assertFalse(firmware.legacy(node))

    def test_require(self):
        with self.assertRaises(firmware.FirmwareUnsupported) as cm:
            firmware.require(legacy_node(), 'capture', 'runout', what='Auto Tune')
        self.assertIn('Auto Tune needs firmware 4.5.0', str(cm.exception))
        self.assertIn('4.3.5', str(cm.exception))
        self.assertEqual(cm.exception.missing, ('capture', 'runout'))
        firmware.require(current_node(), 'capture', 'runout')

    def test_unreadable_version(self):
        # No 0x100A: the objects decide (a bare test fake is the current line).
        self.assertFalse(firmware.legacy(FakeNode()))
        node = FakeNode(values={(0x3013, 3): b'\x01'})
        self.assertTrue(firmware.legacy(node))

    def test_units(self):
        old, new = legacy_node(), current_node()
        self.assertAlmostEqual(firmware.rt_ohm(old, 28), 0.28)
        self.assertAlmostEqual(firmware.rt_ohm(new, 290), 0.29)
        self.assertAlmostEqual(firmware.lt_h(old, 17), 170e-6)
        self.assertAlmostEqual(firmware.lt_h(new, 98), 98e-6)
        self.assertEqual(firmware.rt_raw(old, 0.28), 28)
        self.assertEqual(firmware.lt_raw(new, 98e-6), 98)


def _tuning_values(node, rt, lt, bw):
    node.values.update({
        (0x3011, 5): rt, (0x3011, 6): lt, (0x3011, 4): 27, (0x3011, 3): 14,
        (0x3011, 7): 0, (0x3011, 8): 5000, (0x3011, 9): 20000,
        (0x6076, 0): 400, (0x3001, 1): 60000, (0x3024, 1): 0, (0x3024, 2): 100,
        (0x3024, 3): 100, (0x3024, 4): bw, (0x3024, 5): 100, (0x3024, 6): 0,
        (0x3024, 7): 0, (0x3024, 8): 0, (0x2100, 1): 0})
    return node


class TuningParamsTest(unittest.TestCase):
    def test_legacy_units(self):
        p = tuning.read_params(_tuning_values(legacy_node(), 28, 17, 100))
        self.assertAlmostEqual(p['resistance'], 0.28)
        self.assertAlmostEqual(p['inductance'], 170e-6)
        self.assertIsNone(p['bandwidth_hz'])    # 100 was a gain factor

    def test_current_units(self):
        p = tuning.read_params(_tuning_values(current_node(), 290, 98, 100))
        self.assertAlmostEqual(p['resistance'], 0.29)
        self.assertAlmostEqual(p['inductance'], 98e-6)
        self.assertAlmostEqual(p['bandwidth_hz'], 10.0)


LEGACY_CSV = """\
Rt,,0x3011,5,UNSIGNED16,28
Lt,,0x3011,6,UNSIGNED16,17
polarity,,0x3013,3,INTEGER8,-1
lag factor,,0x3013,5,INTEGER16,0
velocity gain factor,,0x3024,4,INTEGER8,100
Kt,,0x3011,4,UNSIGNED16,27
"""

CURRENT_CSV = """\
Rt,,0x3011,5,UNSIGNED16,290
Ld,,0x3011,6,UNSIGNED16,98
Lq,,0x3011,12,UNSIGNED16,188
polarity,,0x607E,0,UNSIGNED8,0xC0
"""


class CsvLineTest(unittest.TestCase):
    def rows(self, text):
        with tempfile.NamedTemporaryFile('w', suffix='.csv', delete=False) as f:
            f.write(text)
        try:
            return config_csv.read(f.name)
        finally:
            os.unlink(f.name)

    def test_csv_line(self):
        self.assertEqual(config_csv.csv_line(self.rows(LEGACY_CSV)), '4.3')
        self.assertEqual(config_csv.csv_line(self.rows(CURRENT_CSV)), '4.5')
        self.assertIsNone(config_csv.csv_line(self.rows('Kt,,0x3011,4,UNSIGNED16,27\n')))

    def test_current_csv_refused_on_legacy(self):
        node, rep = legacy_node(), _Notes()
        res = config_csv.apply(node, self.rows(CURRENT_CSV), reporter=rep)
        self.assertFalse(res.ok)
        self.assertEqual(res.written, 0)
        self.assertEqual(node.downloads, [])
        self.assertIn('for firmware 4.5.0', res.errors[0])

    def test_legacy_csv_converted_on_current(self):
        node, rep = current_node(), _Notes()
        node.readonly.add((0x3013, 3))
        res = config_csv.apply(node, self.rows(LEGACY_CSV), reporter=rep)
        written = {(i, s): d for i, s, d in node.downloads}
        self.assertEqual(struct.unpack('<H', written[(0x3011, 5)])[0], 280)
        self.assertEqual(struct.unpack('<H', written[(0x3011, 6)])[0], 170)
        self.assertNotIn((0x3013, 5), written)
        self.assertNotIn((0x3024, 4), written)
        self.assertEqual(written[(0x607E, 0)], b'\xc0')
        self.assertTrue(any('converted' in n for n in rep.notes))

    def test_legacy_csv_unchanged_on_legacy(self):
        node = legacy_node()
        config_csv.apply(node, self.rows(LEGACY_CSV), reporter=_Notes())
        written = {(i, s): d for i, s, d in node.downloads}
        self.assertEqual(struct.unpack('<H', written[(0x3011, 5)])[0], 28)
        self.assertIn((0x3013, 5), written)


if __name__ == '__main__':
    unittest.main()
