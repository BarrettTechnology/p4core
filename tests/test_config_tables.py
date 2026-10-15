"""config_csv.template_for / write_table / read_table."""

import os
import tempfile
import unittest

import fake  # noqa: F401  (puts p4core on the path)
from fake import FakeNode

from p4core import config_csv


class TestTemplateFor(unittest.TestCase):
    def node(self, code):
        n = FakeNode(4)
        n.values[(0x1018, 2)] = code
        return n

    def test_legacy_and_ascii_codes(self):
        self.assertTrue(config_csv.template_for(self.node(5755))
                        .endswith('P4-42-Blank.csv'))
        ascii_p416 = int.from_bytes(b'P416'[::-1], 'little')
        self.assertTrue(config_csv.template_for(self.node(ascii_p416))
                        .endswith('P4-16-Blank.csv'))

    def test_board_variant_template_is_preferred(self):
        # Both P4-37 codes are model P4-37; the 1323 (20 mOhm, 7.49/4.32)
        # has its own template, the 1950 takes the model's.
        self.assertTrue(config_csv.template_for(self.node(1323))
                        .endswith('P4-37-1323-Blank.csv'))
        self.assertTrue(config_csv.template_for(self.node(1950))
                        .endswith('P4-37-Blank.csv'))

    def test_1323_template_carries_its_sense_scale(self):
        from p4core import paths
        rows = {}
        for name in ('P4-37-1323-Blank.csv', 'P4-37-Blank.csv'):
            with open(paths.template(name)) as f:
                rows[name] = {(c[2].strip(), c[3].strip()): c[5].strip()
                              for c in (l.split(',') for l in f)
                              if len(c) > 5 and c[2].strip()}
        v = rows['P4-37-1323-Blank.csv']
        self.assertEqual((v[('0x3008', '4')], v[('0x3008', '5')],
                          v[('0x3009', '4')], v[('0x3009', '5')]),
                         ('7490', '20', '4320', '20'))
        # i_peak under the 11.0 A full scale (1.65 V / 20 mOhm / 7.49).
        self.assertLessEqual(int(v[('0x3011', '9')]), 1.65e6 / 20 / 7.49)
        # Only the sense-scale rows differ from the 1950's.
        diff = {k for k in v if v[k] != rows['P4-37-Blank.csv'].get(k)}
        self.assertEqual(diff, {('0x3008', '4'), ('0x3008', '5'),
                                ('0x3009', '4'), ('0x3009', '5'),
                                ('0x3011', '8'), ('0x3011', '9'),
                                ('0x3001', '8')})

    def test_unknown_model(self):
        with self.assertRaises(ValueError):
            config_csv.template_for(self.node(1234))


class TestTables(unittest.TestCase):
    def test_round_trip(self):
        fd, path = tempfile.mkstemp(suffix='.csv')
        os.close(fd)
        try:
            config_csv.write_table(path, 'gains', [('a', 1.5), ('b', None),
                                                   ('c', 3)],
                                   columns=('Gain', 'Value'))
            with open(path) as f:
                lines = f.read().splitlines()
            self.assertEqual(lines[:2], ['# gains', 'Gain,Value'])
            self.assertEqual(lines[3], 'b,')
            self.assertEqual(config_csv.read_table(path), {'a': 1.5, 'c': 3.0})
            self.assertEqual(config_csv.read_table(path, names=['c']),
                             {'c': 3.0})
        finally:
            os.unlink(path)


if __name__ == '__main__':
    unittest.main()
