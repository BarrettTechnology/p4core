"""p4core.nvjson: NV keys from co_dict.c, save/restore order and checks, the
4.3.x wipe."""

import json
import os
import tempfile
import unittest

from p4core import firmware, nvjson
from p4core.reporter import RecordingReporter

CO_DICT = '''
    {0x1017, 0, U16 | RW | NV, FN_HEARTBEAT_TIME, NV_SLOT(5)},   // Heartbeat
    {0x1600, 0, U08 | RW | NV, FN_RPDO_MAPPING, NV_SLOT(21)},
    {0x1600, 1, U32 | RW | NV | X16, FN_MAP_RECORD, NV_SLOT(22)},
    {0x1800, 1, U32 | RW | NV, FN_TPDO_ID, NV_SLOT(89)},
    {0x3001, 1, U32 | RW | NV, parseDefault, 0},    // PWM freq (Hz)
    {0x3027, 2,  I16|RW|NV,  FN_DEFAULT, NV_SLOT(280)},
    {0x6041, 0, U16, FN_DEFAULT, 0},
    {0x3008, 1, U16 | RW, FN_DEFAULT, 0},
    {0x1017, 0, 2000},      // co_nv_blank: not a row
'''


class Sdo:
    RESPONSE_TIMEOUT = 0.3

    def __init__(self, node):
        self.node = node

    def upload(self, index, sub):
        if (index, sub) not in self.node.values:
            raise ValueError('abort')
        return self.node.values[(index, sub)]

    def download(self, index, sub, data, force_segment=False):
        if (index, sub) in self.node.refuse:
            raise ValueError('refused')
        self.node.writes.append((index, sub, bytes(data)))
        self.node.values[(index, sub)] = bytes(data)


class Node:
    id = 4

    def __init__(self, values):
        self.values = dict(values)
        self.writes = []
        self.refuse = set()
        self.sdo = Sdo(self)


def u32(v):
    return v.to_bytes(4, 'little')


class Keys(unittest.TestCase):
    def test_nv_rows_only_and_x16_expanded(self):
        keys = nvjson.nv_keys_from_co_dict(CO_DICT)
        self.assertEqual(keys[:2], [(0x1017, 0), (0x1600, 0)])
        self.assertEqual([k for k in keys if k[0] == 0x1600][1:],
                         [(0x1600, s) for s in range(1, 17)])
        self.assertIn((0x3027, 2), keys)
        self.assertNotIn((0x6041, 0), keys)
        self.assertNotIn((0x3008, 1), keys)
        self.assertEqual(keys.count((0x1017, 0)), 1)

    def test_shipped_lists(self):
        legacy, current = nvjson.nv_keys('legacy'), nvjson.nv_keys('current')
        for keys in (legacy, current):
            self.assertIn((0x3008, 3), keys)       # the bias
            self.assertIn((0x21B0, 0), keys)
            self.assertEqual(len(keys), len(set(keys)))
        self.assertIn((0x3013, 3), legacy)          # 4.3.x polarity
        self.assertIn((0x3011, 14), current)        # float Kt
        self.assertNotIn((0x3011, 14), legacy)

    def test_key_text(self):
        self.assertEqual(nvjson.format_key(0x1A00, 3), '1A00,3')
        self.assertEqual(nvjson.parse_key('1A00,3'), (0x1A00, 3))


class Githash(unittest.TestCase):
    def test_clean_and_dirty(self):
        self.assertEqual(nvjson.read_githash(Node({(0x3003, 0): u32(0x0BE59679)})),
                         'be59679')
        self.assertEqual(nvjson.read_githash(Node({(0x3003, 0): u32(0x8BE59679)})),
                         'be59679-dirty')

    def test_none_on_4_3(self):
        self.assertIsNone(nvjson.read_githash(Node({})))


class Restore(unittest.TestCase):
    def test_order(self):
        entries = {'1800,1': u32(0x184).hex(), '1A00,0': '02', '1A00,1': u32(0x60640020).hex(),
                   '3001,1': u32(80000).hex(), '21B0,0': '0000', '3005,1': '0000'}
        order = nvjson._ordered_writes(entries)
        keys = [k for k, _ in order]
        self.assertNotIn((0x21B0, 0), keys)
        self.assertNotIn((0x3005, 1), keys)
        self.assertEqual(order[0], ((0x1800, 1), u32(0x80000184)))  # invalid first
        self.assertEqual(order[1], ((0x1A00, 0), b'\x00'))
        self.assertEqual(order[-2], ((0x1A00, 0), b'\x02'))          # count late
        self.assertEqual(order[-1], ((0x1800, 1), u32(0x184)))        # real COB-ID last
        self.assertLess(keys.index((0x1A00, 1)), len(keys) - 2)

    def test_cob_id_bit_30_ignored(self):
        self.assertTrue(nvjson._same((0x1800, 1), u32(0x184), u32(0x40000184)))
        self.assertFalse(nvjson._same((0x3001, 1), u32(0x184), u32(0x40000184)))

    def test_mismatch(self):
        ident = dict(firmware='5.0.4', githash='be59679')
        self.assertIsNone(nvjson.mismatch(dict(format=nvjson.FORMAT, **ident), ident))
        self.assertIn('4.3.5', nvjson.mismatch(
            dict(format=nvjson.FORMAT, firmware='4.3.5', githash=None), ident))
        self.assertIn('header', nvjson.mismatch(dict(format=None), ident))


class SaveRestore(unittest.TestCase):
    def setUp(self):
        self.dir = tempfile.TemporaryDirectory()
        self.path = os.path.join(self.dir.name, 'n4.json')
        self.saved = firmware.detect, nvjson.nv_keys, nvjson.nvsave.save_all
        firmware.detect = lambda n, refresh=False: firmware.Firmware(
            (5, 0, 4), '5.0.4', frozenset())
        nvjson.nv_keys = lambda line: [(0x1800, 1), (0x3001, 1), (0x3008, 3), (0x3011, 99)]
        self.saves = []
        nvjson.nvsave.save_all = lambda node: self.saves.append(node.id)

    def tearDown(self):
        firmware.detect, nvjson.nv_keys, nvjson.nvsave.save_all = self.saved
        self.dir.cleanup()

    def node(self):
        return Node({(0x1800, 1): u32(0x40000184), (0x3001, 1): u32(80000),
                     (0x3008, 3): (32000).to_bytes(2, 'little'),
                     (0x3003, 0): u32(0x0BE59679), (0x6041, 0): b'\x31\x02'})

    def test_round_trip(self):
        node = self.node()
        doc = nvjson.save(node, self.path)
        self.assertEqual(doc['githash'], 'be59679')
        self.assertEqual(len(doc['entries']), 3)            # 3011,99 absent
        with open(self.path) as f:
            self.assertEqual(json.load(f)['entries']['3001,1'], u32(80000).hex())
        node.values[(0x3001, 1)] = u32(40000)
        r = nvjson.restore(node, self.path, lambda: node, RecordingReporter())
        self.assertEqual(node.values[(0x3001, 1)], u32(80000))
        self.assertEqual(r['differ'], [])
        self.assertEqual(self.saves, [4])

    def test_refuses_other_firmware(self):
        node = self.node()
        nvjson.save(node, self.path)
        node.values[(0x3003, 0)] = u32(0x0123456)
        with self.assertRaisesRegex(nvjson.NvError, 'refusing'):
            nvjson.restore(node, self.path, lambda: node)
        self.assertEqual(node.writes, [])
        nvjson.restore(node, self.path, lambda: node, force=True)
        self.assertTrue(node.writes)

    def test_refuses_enabled(self):
        node = self.node()
        nvjson.save(node, self.path)
        node.values[(0x6041, 0)] = b'\x37\x02'
        with self.assertRaisesRegex(nvjson.NvError, 'enabled'):
            nvjson.restore(node, self.path, lambda: node)

    def test_flat_nvsnap_file(self):
        with open(self.path, 'w') as f:
            json.dump({'3001,1': u32(1).hex()}, f)
        doc = nvjson.load(self.path)
        self.assertEqual(doc['entries'], {'3001,1': u32(1).hex()})
        self.assertIsNone(doc['format'])


class LegacyWipe(unittest.TestCase):
    def test_defaults_from_the_4_3_5_eds(self):
        d = nvjson.legacy_defaults()
        self.assertTrue(d)
        self.assertTrue(all(i >= 0x2000 for i, _ in d))
        for key in nvjson.WIPE_KEEP:
            self.assertNotIn(key, d)
        self.assertEqual(d[(0x3008, 3)][0], b'\x00\x00')      # bias -> 0

    def test_refuses_5_0(self):
        saved = firmware.detect
        firmware.detect = lambda n, refresh=False: firmware.Firmware(
            (5, 0, 4), '5.0.4', frozenset())
        try:
            with self.assertRaisesRegex(nvjson.NvError, '4.3.x'):
                nvjson.legacy_wipe(Node({}), lambda: None)
        finally:
            firmware.detect = saved

    def test_writes_saves_reboots_verifies(self):
        saved = firmware.detect, nvjson.legacy_defaults, nvjson.nvsave.save_all
        firmware.detect = lambda n, refresh=False: firmware.Firmware(
            (4, 3, 5), '4.3.5', frozenset())
        nvjson.legacy_defaults = lambda eds=None: {
            (0x3008, 3): (b'\x00\x00', 0), (0x3001, 1): (u32(0), 0)}
        events = []
        nvjson.nvsave.save_all = lambda node: events.append('save')
        node = Node({(0x6041, 0): b'\x31\x00', (0x3008, 3): b'\x63\x7d',
                     (0x3001, 1): u32(120000)})

        def reboot():
            events.append('reboot')
            node.values[(0x3001, 1)] = u32(40000)      # 4.3.5's PWM guard
            return node
        try:
            r = nvjson.legacy_wipe(node, reboot, RecordingReporter())
        finally:
            firmware.detect, nvjson.legacy_defaults, nvjson.nvsave.save_all = saved
        self.assertEqual(events, ['save', 'reboot'])
        self.assertEqual(node.values[(0x3008, 3)], b'\x00\x00')
        self.assertEqual(r['written'], 2)
        self.assertEqual(r['differ'], ['3001,1'])
        self.assertIs(r['node'], node)


if __name__ == '__main__':
    unittest.main()
