"""
Tests for pdo_layout.py -- decoding a PDO frame by its written mapping.

The case that matters is the one the firmware used to produce: a 16-bit object
mapped as 32 bits came out as a frame 2 bytes short of the mapping, and a
decoder that read the first 8 bytes of anything long enough, in a fixed
format, either dropped it silently or read the wrong fields.
"""

import os
import struct
import sys
import types
import unittest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__),
                                                '..', '..')))

from p4core.actuator import device                                      # noqa: E402
from p4core.actuator.pdo_layout import PdoLayout                        # noqa: E402


class TestPdoLayout(unittest.TestCase):

    def test_format_follows_the_mapping(self):
        lay = PdoLayout.from_words((0x60710010, 0x607A0020), 'ss')
        self.assertEqual('<hi', lay.format)
        self.assertEqual(6, lay.size)
        self.assertEqual((-5, -100000), lay.decode(struct.pack('<hi', -5, -100000)))

    def test_a_frame_not_the_mapping_length_is_refused_and_counted(self):
        lay = PdoLayout.from_words(device.STREAM_MAPPING, device.STREAM_KINDS, 'stream')
        good = struct.pack('<Hih', 1, -2, 3)
        self.assertEqual((1, -2, 3), lay.decode(good))
        self.assertIsNone(lay.decode(good[:6]))         # 2 bytes short
        self.assertIsNone(lay.decode(good + b'\0'))     # longer is no better
        self.assertEqual(1, lay.frames)
        self.assertEqual(2, lay.bad)
        self.assertIn('9 bytes', lay.problem())
        self.assertIn('mapping is 8 bytes', lay.problem())

    def test_kinds_must_match_the_entries(self):
        with self.assertRaises(ValueError):
            PdoLayout.from_words((0x60710010,), 'ss')
        with self.assertRaises(ValueError):
            PdoLayout.from_words((0x60710018,), 's')    # no 24-bit decode

    def test_from_a_canopen_map_read_back(self):
        var = lambda length, dt: types.SimpleNamespace(
            length=length, index=0x2000, subindex=1,
            od=types.SimpleNamespace(data_type=dt))
        pdo = types.SimpleNamespace(map=[var(16, 0x03), var(32, 0x08),
                                         var(16, 0x06), var(0, 0x07)])
        lay = PdoLayout.from_pdo_map(pdo)
        self.assertEqual('<hfH', lay.format)
        pdo.map.append(var(8, 0x09))                    # a string
        with self.assertRaises(ValueError):
            PdoLayout.from_pdo_map(pdo)


class TestStreamDecode(unittest.TestCase):
    """The stream's frames go through the layout: a short one is not taken,
    and it is reported, not read past."""

    def _stream(self):
        logs = []
        dev = types.SimpleNamespace(node=types.SimpleNamespace(id=4),
                                    _log=logs.append)
        return device._Stream(dev, 1), logs

    def test_short_frame_is_refused_and_named_in_the_error(self):
        s, _ = self._stream()
        s._on_frame(0x484, struct.pack('<Hih', 10, 1000, 0), 0.0)
        s._on_frame(0x484, struct.pack('<Hih', 20, 3000, 0)[:6], 0.001)
        self.assertEqual([(10, 1000, 0, 0.0)], s.frames)
        with self.assertRaises(device.SysidError) as e:
            s.mean_iq_a(0, 2)
        self.assertIn('refused', str(e.exception))


if __name__ == '__main__':
    unittest.main()
