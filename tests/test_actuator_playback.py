"""p4core.actuator.playback: a logger capture read back over TPDO4."""

import os
import struct
import sys
import unittest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__),
                                                '..', '..')))

from p4core.actuator import playback                                   # noqa: E402

WORDS = [0x30100910, 0x60640020]        # iq I16, position I32: 6 bytes


class _Net:
    def __init__(self):
        self.subs = {}

    def subscribe(self, cob, cb):
        self.subs[cob] = cb

    def unsubscribe(self, cob, cb):
        self.subs.pop(cob, None)


class _Nmt:
    state = 'PRE-OPERATIONAL'


class _Node:
    """TPDO4 and 0x3004 as the firmware has them: a playback starts on
    0x3004,1 = 3 and its frames go out once TPDO4 is enabled (here all at
    once, from the enabling write), *stop_after* of them if given."""

    def __init__(self, records, stop_after=None):
        self.network = _Net()
        self.nmt = _Nmt()
        self.records = records
        self.stop_after = stop_after
        self.od = {(0x1803, 1): 0x80000485, (0x1803, 2): 1, (0x1803, 3): 500,
                   (0x1803, 5): 1000, (0x3004, 1): 2, (0x3004, 4): 0}
        self.sdo = self
        self.writes = []

    def upload(self, index, sub):
        return struct.pack('<I', self.od[(index, sub)])

    def download(self, index, sub, data):
        v = int.from_bytes(bytes(data), 'little')
        self.writes.append(((index, sub), v))
        self.od[(index, sub)] = v
        if (index, sub) == (0x1803, 1) and not v & 0x80000000 \
                and self.od[(0x3004, 1)] == 3:
            assert self.od[(0x1803, 2)] == 254 and self.od[(0x1803, 3)] == 0
            assert self.nmt.state == 'OPERATIONAL'
            cb = self.network.subs[v & 0x7FF]
            recs = self.records[:self.stop_after]
            for iq, pos in recs:
                cb(v & 0x7FF, struct.pack('<hi', iq, pos), 0.0)
            if len(recs) == len(self.records):
                self.od[(0x3004, 1)] = 2
            else:
                cb(v & 0x7FF, b'\0' * 3, 0.0)   # a stray frame of another size


class TestPlayback(unittest.TestCase):

    def setUp(self):
        self._first, self._stall = playback.FIRST_S, playback.STALL_S
        playback.FIRST_S = playback.STALL_S = 0.02

    def tearDown(self):
        playback.FIRST_S, playback.STALL_S = self._first, self._stall

    def test_reads_the_records_signed(self):
        recs = [(-5, -70000), (32767, 1 << 30), (-32768, 3)]
        node = _Node(recs)
        self.assertEqual(playback.read(node, WORDS, 3), [list(r) for r in recs])

    def test_puts_tpdo4_and_nmt_back(self):
        node = _Node([(1, 2)])
        playback.read(node, WORDS, 1)
        self.assertEqual(node.od[(0x1803, 1)], 0x80000485)
        self.assertEqual(node.od[(0x1803, 2)], 1)
        self.assertEqual(node.od[(0x1803, 3)], 500)
        self.assertEqual(node.od[(0x1803, 5)], 1000)
        self.assertEqual(node.nmt.state, 'PRE-OPERATIONAL')
        self.assertEqual(node.network.subs, {})

    def test_playback_starts_before_the_pdo_is_enabled(self):
        node = _Node([(1, 2)])
        playback.read(node, WORDS, 1)
        keys = [k for k, _ in node.writes]
        enable = max(i for i, (k, v) in enumerate(node.writes)
                     if k == (0x1803, 1) and not v & 0x80000000)
        self.assertLess(keys.index((0x3004, 1)), enable)
        self.assertLess(keys.index((0x3004, 4)), keys.index((0x3004, 1)))

    def test_frames_that_stop_fall_back_to_sdo(self):
        node = _Node([(1, 2), (3, 4), (5, 6)], stop_after=2)
        logs = []
        self.assertIsNone(playback.read(node, WORDS, 3, log=logs.append))
        self.assertEqual(node.od[(0x3004, 1)], playback.LOG_IDLE)
        self.assertTrue(any('by SDO' in m for m in logs), logs)

    def test_a_virtual_network_is_not_played_back(self):
        node = _Node([(1, 2)])
        node.network.advance = lambda s: None
        self.assertIsNone(playback.read(node, WORDS, 1))
        self.assertEqual(node.writes, [])

    def test_a_record_longer_than_a_frame_is_not_played_back(self):
        node = _Node([(1, 2)])
        words = [0x606B0020, 0x60740010, 0x606C0020]     # 10 bytes
        self.assertIsNone(playback.read(node, words, 1))
        self.assertEqual(node.writes, [])


if __name__ == '__main__':
    unittest.main()
