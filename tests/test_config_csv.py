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


class TestBoardCalibrationRows(unittest.TestCase):
    TEXT = '\n'.join([
        'Temp,,0x220A,0,UNSIGNED8,85',
        'Alpha bias,,0x3008,3,UNSIGNED16,2048',
        'Alpha gain factor,,0x3008,6,UNSIGNED16,0',
        'Electrical zero,,0x3011,1,UNSIGNED16,100',
        'Comp active,,0x3027,1,UNSIGNED16,1',
    ])

    def written(self, node):
        return {(i, s) for i, s, _ in node.downloads}

    def test_apply_never_writes_board_calibration(self):
        # An older CSV (an export, or a product CSV made from an old blank
        # template with a zero Gainfactor) must not land on a calibrated
        # board, whichever board it came from.
        node = FakeNode(5)
        rep = RecordingReporter()
        result = config_csv.apply(node, rows(
            '# Board serial (0x1018,4): 0x5e41\n' + self.TEXT), reporter=rep)
        self.assertTrue(result.ok)
        self.assertEqual(self.written(node), {(0x220A, 0)})
        self.assertTrue(any('own calibration' in t for t in rep.texts('note')))

    def test_templates_carry_no_board_calibration(self):
        for name in paths.templates():
            keys = {(int(r.index, 0), int(r.subindex, 0))
                    for r in config_csv.read(paths.template(name))
                    if r.kind == 'entry' and r.index and r.subindex}
            self.assertFalse(keys & config_csv.BOARD_CALIBRATION, name)

    def test_export_drops_board_calibration_and_the_serial(self):
        # A template made from an old export still has them: the export
        # leaves them out.
        src = FakeNode(5)
        src.values[(0x220A, 0)] = (85).to_bytes(1, 'little')
        src.values[(0x3008, 6)] = (4096).to_bytes(2, 'little')
        with tempfile.NamedTemporaryFile('w', suffix='.csv',
                                         delete=False) as f:
            f.write('# Board serial (0x1018,4): 0x5e41\n' + self.TEXT + '\n')
        out = tempfile.mktemp(suffix='.csv')
        try:
            config_csv.build_from_template(src, f.name, out)
            text = open(out).read()
            self.assertNotIn('Board serial', text)
            self.assertNotIn('0x3008', text)
            self.assertIn('0x220A', text)
        finally:
            for path in (f.name, out):
                if os.path.exists(path):
                    os.unlink(path)


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


class TestReal32(unittest.TestCase):
    """F32 objects (the gains, Kt) as REAL32 rows, and Kt on older firmware."""
    GAINS = {(0x2380, 1): 0.13842642307281494, (0x2380, 2): 213.53012084960938,
             (0x2381, 1): 0.003231385722756386, (0x2381, 2): 8.982659892353695e-06}

    def test_format_is_the_shortest_float32_round_trip(self):
        import struct
        for v in list(self.GAINS.values()) + [31.27, 1250.0, 0.1, 1e-9]:
            text = config_csv.format_real32(v)
            self.assertLessEqual(len(text.replace('e-', '').replace('.', '')
                                     .lstrip('0')), 9, text)
            self.assertEqual(struct.pack('<f', float(text)), struct.pack('<f', v), text)
        self.assertEqual(config_csv.format_real32(0.13842642307281494), '0.13842642')
        self.assertEqual(config_csv.format_real32(31.27), '31.27')
        self.assertEqual(config_csv.format_real32(0.0), '0')

    def test_export_then_load_is_bit_exact(self):
        import struct
        src, dst = FakeNode(5), FakeNode(6)
        for k, v in self.GAINS.items():
            src.values[k] = v
        src.values[(0x3011, 14)] = 31.27
        with tempfile.NamedTemporaryFile('w', suffix='.csv', delete=False) as f:
            f.write('\n'.join('x,,0x{:04X},{},REAL32,0'.format(*k)
                              for k in list(self.GAINS) + [(0x3011, 14)]) + '\n')
        out = tempfile.mktemp(suffix='.csv')
        try:
            config_csv.build_from_template(src, f.name, out)
            with open(out) as g:
                text = g.read()
            self.assertIn('0x2380,1,REAL32,0.13842642', text)
            self.assertIn('0x3011,14,REAL32,31.27', text)
            self.assertTrue(config_csv.load(dst, out).ok)
            for k in list(self.GAINS) + [(0x3011, 14)]:
                self.assertEqual(struct.pack('<f', dst.values[k]),
                                 struct.pack('<f', src.values[k]), k)
        finally:
            for path in (f.name, out):
                if os.path.exists(path):
                    os.unlink(path)

    def test_old_unsigned32_rows_still_load_the_same_bits(self):
        node = FakeNode(5)
        self.assertTrue(config_csv.apply(node, rows(
            'Kp,,0x2381,1,UNSIGNED32,0x3b53c5a8')).ok)
        self.assertAlmostEqual(node.values[(0x2381, 1)], 0.0032313857, places=9)

    def test_kt_row_on_firmware_without_0x3011_14(self):
        node = FakeNode(5)
        node.readonly.add((0x3011, 14))
        rep = RecordingReporter()
        result = config_csv.apply(node, rows(
            'Motor Kt,,0x3011,14,REAL32,31.6'), reporter=rep)
        self.assertTrue(result.ok, result.errors)
        self.assertEqual(node.values[(0x3011, 4)], 32)
        self.assertTrue(any('0x3011,4' in t for t in rep.texts('note')))

    def test_kt_export_from_firmware_without_0x3011_14(self):
        node = FakeNode(5)
        node.values[(0x3011, 4)] = 31
        with tempfile.NamedTemporaryFile('w', suffix='.csv', delete=False) as f:
            f.write('Motor Kt,,0x3011,14,REAL32,0\n')
        out = tempfile.mktemp(suffix='.csv')
        try:
            config_csv.build_from_template(node, f.name, out)
            with open(out) as g:
                self.assertIn('0x3011,14,REAL32,31', g.read())
        finally:
            for path in (f.name, out):
                if os.path.exists(path):
                    os.unlink(path)

    def test_real32_out_of_range_is_invalid(self):
        errors, _ = config_csv.validate(rows('x,,0x2380,1,REAL32,1e40'), 5)
        self.assertEqual(errors, [(1, 10)])

    def test_templates_type_the_gains_and_kt_real32(self):
        floats = {(0x2380, 1), (0x2380, 2), (0x2381, 1), (0x2381, 2), (0x3011, 14)}
        for name in paths.templates():
            seen = {(int(r.index, 0), int(r.subindex, 0)): r.datatype
                    for r in config_csv.read(paths.template(name))
                    if r.kind == 'entry' and r.index and r.subindex}
            self.assertNotIn((0x3011, 4), seen, name)
            for k in floats:
                self.assertEqual(seen.get(k), 'REAL32', (name, k))


class TestKt(unittest.TestCase):
    def test_float_kt(self):
        from p4core import calibrate
        node = FakeNode(5)
        node.values[(0x3011, 14)] = 31.27
        self.assertTrue(calibrate.has_float_kt(node))
        self.assertAlmostEqual(calibrate.read_kt(node), 31.27, places=5)
        self.assertEqual(calibrate.write_kt(node, 1250.5, save=True), (0x3011, 14))
        self.assertEqual(node.values[(0x3011, 14)], 1250.5)
        self.assertEqual(node.values[(0x1010, 5)] if (0x1010, 5) in node.values
                         else node.get('Save', 'Single'), (0x3011 << 8) | 14)

    def test_old_firmware_kt(self):
        from p4core import calibrate
        node = FakeNode(5)
        node.values[(0x3011, 4)] = 31
        node.readonly.add((0x3011, 14))
        self.assertFalse(calibrate.has_float_kt(node))
        self.assertEqual(calibrate.read_kt(node), 31.0)
        self.assertEqual(calibrate.write_kt(node, 31.6), (0x3011, 4))
        self.assertEqual(node.values[(0x3011, 4)], 32)
        with self.assertRaises(ValueError):
            calibrate.write_kt(node, -1)


if __name__ == '__main__':
    unittest.main()
