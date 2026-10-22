"""Manufacturer modes negative (stm32 cf8d11e) vs 12 / 14 on older firmware,
and the 0x6079 bus voltage with its 0x3000,1 fallback."""
import struct
import unittest

from canopen.sdo.exceptions import SdoAbortedError

import fake  # noqa: F401  (puts p4core on the path)

from p4core import calibrate, cia402


class Sdo:
    def __init__(self, rows):
        self.rows = rows
        self.uploads = 0

    def upload(self, index, sub):
        self.uploads += 1
        if (index, sub) not in self.rows:
            raise SdoAbortedError(0x06020000)
        return self.rows[(index, sub)]


class Node:
    def __init__(self, rows):
        self.sdo = Sdo(rows)


NEW = {(0x6502, 0): struct.pack('<I', 0x000307AD)}
OLD = {(0x6502, 0): struct.pack('<I', 0x00002BAD)}


class TestModes(unittest.TestCase):

    def test_new_firmware_takes_negative_modes(self):
        n = Node(NEW)
        self.assertTrue(cia402.negative_modes(n))
        self.assertEqual(cia402.drive_mode(n, cia402.MODE_PHASE_VOLTAGE_ANGLE), -1)
        self.assertEqual(cia402.drive_mode(n, cia402.MODE_CYCLIC_SYNC_IMP), -2)
        self.assertEqual(cia402.drive_mode(n, cia402.MODE_PROFILE_VEL), 3)

    def test_old_firmware_gets_12_and_14(self):
        n = Node(OLD)
        self.assertFalse(cia402.negative_modes(n))
        self.assertEqual(cia402.drive_mode(n, cia402.MODE_PHASE_VOLTAGE_ANGLE), 12)
        self.assertEqual(cia402.drive_mode(n, cia402.MODE_CYCLIC_SYNC_IMP), 14)
        self.assertEqual(cia402.drive_mode(n, cia402.MODE_CYCLIC_SYNC_TRQ), 10)

    def test_unreadable_0x6502_is_old_firmware(self):
        self.assertEqual(cia402.drive_mode(Node({}), cia402.MODE_PHASE_VOLTAGE_ANGLE), 12)

    def test_probe_is_cached(self):
        n = Node(NEW)
        for _ in range(3):
            cia402.drive_mode(n, cia402.MODE_PHASE_VOLTAGE_ANGLE)
        self.assertEqual(n.sdo.uploads, 1)

    def test_bits(self):
        self.assertEqual(cia402.mode_bit(-1), 1 << 16)
        self.assertEqual(cia402.mode_bit(-2), 1 << 17)
        self.assertEqual(cia402.mode_bit(11), 1 << 10)
        self.assertTrue(0x000307AD & cia402.mode_bit(cia402.MODE_CYCLIC_SYNC_TRQ_ANGLE))

    def test_canonical_and_byte(self):
        self.assertEqual(cia402.canonical_mode(255), -1)
        self.assertEqual(cia402.canonical_mode(12), -1)
        self.assertEqual(cia402.canonical_mode(14), -2)
        self.assertEqual(cia402.canonical_mode(-2), -2)
        self.assertEqual(cia402.canonical_mode(9), 9)
        self.assertEqual(cia402.mode_byte(-1), 0xFF)
        self.assertEqual(cia402.mode_byte(12), 12)


class TestBusVoltage(unittest.TestCase):

    def test_6079_in_mv(self):
        n = Node({(0x6079, 0): struct.pack('<I', 41300), (0x3000, 1): struct.pack('<H', 999)})
        self.assertEqual(calibrate.read_bus_mv(n), 41300)
        self.assertAlmostEqual(calibrate.read_bus_v(n), 41.3)

    def test_older_firmware_falls_back_to_0x3000(self):
        n = Node({(0x3000, 1): struct.pack('<H', 413)})
        self.assertEqual(calibrate.read_bus_mv(n), 41300)


if __name__ == '__main__':
    unittest.main()
