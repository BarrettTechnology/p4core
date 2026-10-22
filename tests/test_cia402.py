import unittest

import fake  # noqa: F401  (puts p4core on the path)

from p4core import cia402


class TestErrorCodes(unittest.TestCase):
    def test_describe(self):
        self.assertTrue(cia402.describe_error(0x8110).startswith('0x8110 CAN overrun'))
        self.assertEqual(cia402.describe_error(0x1234), '0x1234')

    def test_supply(self):
        self.assertIn(0x3220, cia402.SUPPLY_ERROR_CODES)
        self.assertNotIn(0x8110, cia402.SUPPLY_ERROR_CODES)
