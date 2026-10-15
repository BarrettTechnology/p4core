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

    def test_legacy_polarity_row_goes_to_607e(self):
        """An old CSV's 0x3013,3 row: firmware without it gets 0x607E."""
        for data, want in (('0xFF', 0xC0), ('1', 0x00)):
            node = FakeNode(5)
            node.readonly.add((0x3013, 3))    # refused, as the new firmware does
            rep = RecordingReporter()
            result = config_csv.apply(node, rows(
                'Encoder user polarity,,0x3013,3,UNSIGNED8,' + data), reporter=rep)
            self.assertTrue(result.ok, result.errors)
            self.assertEqual(node.values[(0x607E, 0)], want)
            self.assertTrue(any('0x607E' in t for t in rep.texts('note')))

    def test_legacy_polarity_row_on_legacy_firmware(self):
        node = FakeNode(5)
        result = config_csv.apply(node, rows(
            'Encoder user polarity,,0x3013,3,UNSIGNED8,0xFF'))
        self.assertTrue(result.ok)
        self.assertNotIn((0x607E, 0), node.values)

    def test_legacy_polarity_row_validates(self):
        self.assertEqual(config_csv.validate(
            rows('x,,0x3013,3,UNSIGNED8,1'), 5, fake.od()), ([], []))

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
        src.values[(0x1018, 4)] = (0x5E41).to_bytes(4, 'little')
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
            dst = FakeNode(7)               # the same board, re-addressed
            dst.values[(0x1018, 4)] = (0x5E41).to_bytes(4, 'little')
            result = config_csv.apply(dst, built)
            self.assertTrue(result.ok, result.errors)
            sent = {(i, s): d for i, s, d in dst.downloads}
            for key, data in wire.items():
                self.assertEqual(sent.get(key), data, hex(key[0]))
            self.assertEqual(sent[(0x1400, 1)], (0x207).to_bytes(4, 'little'))
        finally:
            if os.path.exists(out):
                os.unlink(out)

    def test_round_trip_all_models(self):
        """The same property as test_round_trip, for every model template --
        so a row added to only one of the three (e.g. a model-specific
        default) still round-trips, and a template that fails to validate
        against the EDS (a bad datatype or a stray comma) is caught here
        too, not just for P4-42."""
        for name in paths.templates():
            template = paths.template(name)
            src = FakeNode(5)
            src.values[(0x1018, 4)] = (0x5E41).to_bytes(4, 'little')
            entries = [r for r in config_csv.read(template)
                      if r.kind == 'entry' and r.index
                      and not r.index.lower().startswith('0x1')]
            wire = {}
            for n, r in enumerate(entries):
                key = (int(r.index, 0), int(r.subindex, 0))
                if r.datatype == 'REAL32':
                    value = str(0.25 * n)
                elif r.datatype.startswith('INTEGER'):
                    value = str(-n)
                else:
                    value = str(n)
                wire[key] = src.values[key] = config_csv.encode(
                    r.datatype, value)[0]
            out = tempfile.mktemp(suffix='.csv')
            try:
                count = config_csv.build_from_template(src, template, out)
                self.assertEqual(count, len(entries), name)
                built = config_csv.read(out)
                self.assertEqual(config_csv.validate(built, 7, fake.od())[0],
                                 [], name)
                dst = FakeNode(7)
                dst.values[(0x1018, 4)] = (0x5E41).to_bytes(4, 'little')
                result = config_csv.apply(dst, built)
                self.assertTrue(result.ok, (name, result.errors))
                sent = {(i, s): d for i, s, d in dst.downloads}
                for key, data in wire.items():
                    self.assertEqual(sent.get(key), data, (name, hex(key[0])))
            finally:
                if os.path.exists(out):
                    os.unlink(out)

    def test_format(self):
        self.assertEqual(config_csv._format('0x2000', 'UNSIGNED32', 255), '0xff')
        self.assertEqual(config_csv._format('0x6000', 'INTEGER32', -16), '-0x10')
        self.assertEqual(config_csv._format('0x6000', 'UNSIGNED32', 16), '16')
        self.assertEqual(config_csv._format('0x2000', 'UNSIGNED8', 0), '0')


class TestPerBoardRows(unittest.TestCase):
    TEXT = '\n'.join([
        'Temp,,0x220A,0,UNSIGNED8,85',
        'Alpha bias,,0x3008,3,UNSIGNED16,2048',
        'Alpha gain factor,,0x3008,6,UNSIGNED16,0',
        'Electrical zero,,0x3011,1,UNSIGNED16,100',
        'Comp active,,0x3027,1,UNSIGNED16,1',
    ])

    def node(self, serial):
        n = FakeNode(5)
        if serial is not None:
            n.values[(0x1018, 4)] = serial.to_bytes(4, 'little')
        return n

    def written(self, node):
        return {(i, s) for i, s, _ in node.downloads}

    def test_a_csv_with_no_serial_keeps_the_boards_calibration(self):
        # A product CSV made from a blank template: its zero Gainfactor
        # must not land on a calibrated board.
        node = self.node(0x5E41)
        rep = RecordingReporter()
        result = config_csv.apply(node, rows(self.TEXT), reporter=rep)
        self.assertTrue(result.ok)
        self.assertEqual(self.written(node), {(0x220A, 0)})
        self.assertTrue(any('own calibration' in t for t in rep.texts('note')))

    def test_another_boards_export_is_skipped(self):
        text = config_csv.BOARD_SERIAL_TAG + ' 0x1111\n' + self.TEXT
        node = self.node(0x5E41)
        config_csv.apply(node, rows(text))
        self.assertEqual(self.written(node), {(0x220A, 0)})

    def test_the_same_boards_export_restores_everything(self):
        text = config_csv.BOARD_SERIAL_TAG + ' 0x5e41\n' + self.TEXT
        node = self.node(0x5E41)
        result = config_csv.apply(node, rows(text))
        self.assertEqual(result.written, 5)
        self.assertEqual(config_csv.board_serial(rows(text)), 0x5E41)

    def test_an_unprogrammed_serial_matches_nothing(self):
        text = config_csv.BOARD_SERIAL_TAG + ' 0x0\n' + self.TEXT
        node = self.node(0)
        config_csv.apply(node, rows(text))
        self.assertEqual(self.written(node), {(0x220A, 0)})

    def test_board_rows_true_forces_and_false_refuses(self):
        node = self.node(None)
        self.assertEqual(config_csv.apply(node, rows(self.TEXT),
                                          board_rows=True).written, 5)
        text = config_csv.BOARD_SERIAL_TAG + ' 0x5e41\n' + self.TEXT
        node = self.node(0x5E41)
        self.assertEqual(config_csv.apply(node, rows(text),
                                          board_rows=False).written, 1)

    def test_export_records_the_serial(self):
        template = paths.template('P4-42-Blank.csv')
        src = self.node(0x5E41)
        for r in config_csv.read(template):
            if r.kind == 'entry' and r.index and \
                    not r.index.lower().startswith('0x1'):
                src.values[(int(r.index, 0), int(r.subindex, 0))] = \
                    config_csv.encode(r.datatype, '0')[0]
        out = tempfile.mktemp(suffix='.csv')
        try:
            config_csv.build_from_template(src, template, out)
            self.assertEqual(config_csv.board_serial(config_csv.read(out)),
                             0x5E41)
        finally:
            if os.path.exists(out):
                os.unlink(out)


class TestSetEntry(unittest.TestCase):
    def test_update_and_append(self):
        with tempfile.NamedTemporaryFile('w', suffix='.csv', delete=False) as f:
            f.write('# keep me\ni_cal,,0x3011,11,UNSIGNED16,1000\n')
        try:
            self.assertTrue(config_csv.set_entry(f.name, 0x3011, 11, 'UNSIGNED16', 1500))
            self.assertFalse(config_csv.set_entry(f.name, 0x3011, 9, 'UNSIGNED16', 20000, 'i_peak'))
            with open(f.name) as g:
                self.assertEqual(g.read(), '# keep me\ni_cal,,0x3011,11,UNSIGNED16,1500\n'
                                           'i_peak,,0x3011,9,UNSIGNED16,20000\n')
        finally:
            os.unlink(f.name)


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
