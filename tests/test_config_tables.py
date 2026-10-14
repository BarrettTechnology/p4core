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
