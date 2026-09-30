import os
import tempfile
import unittest

import fake  # noqa: F401  (puts p4core on the path)
from fake import FakeNode

from p4core import config_csv, paths
from p4core.reporter import RecordingReporter


def rows(text):
    return config_csv.parse(text.strip().splitlines())


class TestParseValidate(unittest.TestCase):
    def test_kinds(self):
        r = rows('# a comment\n\nName,FILE_INFO,,,STRING,x\n')
        self.assertEqual([x.kind for x in r], ['comment', 'blank', 'entry'])

    def test_codes(self):
        cases = {
            'a,b,c': 1,
            'x,,0x2000,,UNSIGNED8,1': 2,
            'x,BOGUS,,,UNSIGNED8,1': 3,
            'x,FILE_INFO,0x2000,,STRING,1': 4,
            'x,,zz,1,UNSIGNED8,1': 5,
            'x,,0x2000,1,UNSIGNED9,1': 6,
            'x,SETTING_NODE_ID,,,UNSIGNED16,1': 8,
            'x,,0x2000,1,UNSIGNED8,256': 10,
            'x,,0x2000,1,INTEGER8,-129': 10,
        }
        for line, code in cases.items():
            errors, _ = config_csv.validate(rows(line), node_id=5)
            self.assertEqual(errors, [(1, code)], line)

    def test_id_without_node(self):
        errors, _ = config_csv.validate(rows('x,,0x1400,1,UNSIGNED32,0x200 | $ID'))
        self.assertEqual(errors, [(1, 10)])

    def test_unknown_index_is_a_warning(self):
        errors, warnings = config_csv.validate(
            rows('x,,0x5FFF,1,UNSIGNED8,1'), 5, fake.od())
        self.assertEqual((errors, warnings), ([], [(1, 11)]))

    def test_templates_validate(self):
        for name in paths.templates():
            r = config_csv.read(paths.template(name))
            errors, _ = config_csv.validate(r, 5, fake.od())
            self.assertEqual(errors, [], name)

    def test_encode(self):
        self.assertEqual(config_csv.encode('UNSIGNED32', '0x200 | $ID', 5)[1], 0x205)
        self.assertEqual(config_csv.encode('INTEGER16', '-2', None)[0], b'\xfe\xff')
        self.assertEqual(config_csv.encode('REAL32', '1.5')[1], 1.5)
        self.assertEqual(config_csv.encode('UNSIGNED8', '4.0')[1], 4)
        with self.assertRaises(ValueError):
            config_csv.encode('UNSIGNED8', '__import__("os")')


class TestApply(unittest.TestCase):
    def test_downloads_and_settings(self):
        node = FakeNode(5)
        text = '\n'.join([
            'Version,FILE_VERSION,,,STRING,V1',
            'ID,SETTING_NODE_ID,,,UNSIGNED8,9',
            'RPDO1,,0x1400,1,UNSIGNED32,0x200 | $ID',
            'Reset,CONTROL_NMT,,,UNSIGNED16,0x0101',
            'Temp,,0x220A,0,UNSIGNED8,85',
        ])
        rep = RecordingReporter()
        result = config_csv.apply(node, rows(text), reporter=rep)
        self.assertTrue(result.ok)
        self.assertEqual(result.written, 2)
        self.assertEqual(node.downloads[0], (0x1400, 1, (0x209).to_bytes(4, 'little')))
        self.assertEqual(node.values[(0x220A, 0)], 85)
        self.assertTrue(any('skipped' in t for t in rep.texts('note')))
        self.assertEqual(node.sdo.RESPONSE_TIMEOUT, 0.3)

    def test_abort_is_an_error_not_a_stop(self):
        node = FakeNode(5)
        node.readonly.add((0x220A, 0))
        text = 'Temp,,0x220A,0,UNSIGNED8,85\nOV,,0x2384,4,UNSIGNED16,500'
        result = config_csv.apply(node, rows(text))
        self.assertEqual(result.written, 1)
        self.assertEqual(len(result.errors), 1)
        self.assertIn('0x06010002', result.errors[0])

    def test_load_refuses_invalid(self):
        with tempfile.NamedTemporaryFile('w', suffix='.csv', delete=False) as f:
            f.write('x,,0x2000,1,UNSIGNED8,999\n')
        try:
            with self.assertRaises(config_csv.ConfigError):
                config_csv.load(FakeNode(5), f.name)
        finally:
            os.unlink(f.name)


class TestBuildFromTemplate(unittest.TestCase):
    def test_round_trip(self):
        """A built CSV validates and, loaded onto a blank puck, reproduces
        every non-communication value it was built from."""
        template = paths.template('P4-42-Blank.csv')
        src = FakeNode(5)
        entries = [r for r in config_csv.read(template) if r.kind == 'entry'
                   and r.index and not r.index.lower().startswith('0x1')]
        wire = {}
        for n, r in enumerate(entries):
            key = (int(r.index, 0), int(r.subindex, 0))
            if r.datatype == 'REAL32':
                value = str(0.25 * n)
            elif r.datatype.startswith('INTEGER'):
                value = str(-n)
            else:
                value = str(n)
            # What the puck sends: the value in the CSV's own datatype.
            wire[key] = src.values[key] = config_csv.encode(r.datatype, value)[0]
        out = tempfile.mktemp(suffix='.csv')
        try:
            count = config_csv.build_from_template(src, template, out,
                                                   date='2026-09-30')
            self.assertEqual(count, len(entries))
            built = config_csv.read(out)
            self.assertEqual(config_csv.validate(built, 7, fake.od())[0], [])
            self.assertEqual(config_csv.file_version(built), 'V1.0.0 (2026-09-30)')
            self.assertTrue(any('$ID' in r.data for r in built
                                if r.kind == 'entry' and len(r.fields) == 6))
            dst = FakeNode(7)
            result = config_csv.apply(dst, built)
            self.assertTrue(result.ok, result.errors)
            sent = {(i, s): d for i, s, d in dst.downloads}
            for key, data in wire.items():
                self.assertEqual(sent.get(key), data, hex(key[0]))
            self.assertEqual(sent[(0x1400, 1)], (0x207).to_bytes(4, 'little'))
        finally:
            if os.path.exists(out):
                os.unlink(out)

    def test_format(self):
        self.assertEqual(config_csv._format('0x2000', 'UNSIGNED32', 255), '0xff')
        self.assertEqual(config_csv._format('0x6000', 'INTEGER32', -16), '-0x10')
        self.assertEqual(config_csv._format('0x6000', 'UNSIGNED32', 16), '16')
        self.assertEqual(config_csv._format('0x2000', 'UNSIGNED8', 0), '0')


class TestRemap(unittest.TestCase):
    def test_remap_keeps_high_bits_and_saves(self):
        node = FakeNode(9)
        for index, base in config_csv.PDO_COB_BASES.items():
            node.values[(index, 1)] = base | 5 | (0x80000000 if index == 0x1403 else 0)
        ok, results = config_csv.remap_pdo_cob_ids(node, 9, old_id=5, log=lambda s: None)
        self.assertTrue(ok)
        self.assertEqual(node.values[(0x1400, 1)], 0x209)
        self.assertEqual(node.values[(0x1403, 1)], 0x80000509)
        self.assertEqual(node.values[(0x1010, 1)], 0x65766173)

    def test_leaves_cross_linked_pdo(self):
        node = FakeNode(9)
        for index, base in config_csv.PDO_COB_BASES.items():
            node.values[(index, 1)] = base | 5
        node.values[(0x1401, 1)] = 0x300 | 22
        config_csv.remap_pdo_cob_ids(node, 9, old_id=5, save=False, log=lambda s: None)
        self.assertEqual(node.values[(0x1401, 1)], 0x316)


if __name__ == '__main__':
    unittest.main()
