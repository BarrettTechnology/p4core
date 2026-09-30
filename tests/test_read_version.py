"""flash.read_version(): 0x100A as a VISIBLE_STRING or the packed U32."""
import os
import sys
import unittest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))
from p4core import flash as flashp4  # noqa: E402


class _Node:
    def __init__(self, raw):
        self.sdo = self
        self._raw = raw

    def upload(self, index, sub):
        assert (index, sub) == (0x100A, 0)
        return self._raw


class ReadVersionTest(unittest.TestCase):
    def test_string_from_application(self):
        self.assertEqual(flashp4.read_version(_Node(b'4.4.0')), '4.4.0')

    def test_string_widest_fields(self):
        self.assertEqual(flashp4.read_version(_Node(b'255.65535.255\0')),
                         '255.65535.255')

    def test_packed_u32_from_flashloader(self):
        packed = (3 << 24) | (0 << 8) | 7
        self.assertEqual(flashp4.read_version(_Node(packed.to_bytes(4, 'little'))),
                         '3.0.7')

    def test_packed_u32_wide_minor(self):
        packed = (4 << 24) | (300 << 8) | 2
        self.assertEqual(flashp4.read_version(_Node(packed.to_bytes(4, 'little'))),
                         '4.300.2')


if __name__ == '__main__':
    unittest.main()
