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


class TestEnable(unittest.TestCase):
    """cia402.enable: a refusal names the fault (review-nov5 P1-3: node 4's
    persisting fault read only as 'Code 0x08000022 ... device state')."""

    def node(self, sw=0x27, code=0, reg=0):
        n = fake.FakeNode()
        n.values[(0x6041, 0)] = sw
        n.values[(0x603F, 0)] = code
        n.values[(0x1001, 0)] = reg
        return n

    def test_a_clean_enable_writes_the_three_words(self):
        n = self.node()
        self.assertEqual(cia402.enable(n, sleep=lambda s: None), 0x27)
        self.assertEqual([v for k, v in n.writes if k == (0x6040, 0)],
                         [0x80, 0x06, 0x0F])

    def test_a_reset_that_clears_the_fault_goes_on(self):
        n = self.node(sw=0x08)
        n.hook_write('ControlWord', None,
                     lambda v: n.values.__setitem__((0x6041, 0), 0x40)
                     if v == 0x80 else None)
        cia402.enable(n, sleep=lambda s: None)

    def test_a_persisting_fault_is_named_and_nothing_more_is_written(self):
        n = self.node(sw=0x0218, code=0x3220, reg=0x05)
        with self.assertRaises(cia402.EnableRefused) as cm:
            cia402.enable(n, 'Current-sense bias', sleep=lambda s: None)
        msg = str(cm.exception)
        self.assertIn('Current-sense bias refused', msg)
        self.assertIn('persists', msg)
        self.assertIn('0x3220 bus under-voltage', msg)
        self.assertIn('error register 0x05 (generic, voltage)', msg)
        self.assertIn('(Fault)', msg)
        self.assertEqual([v for k, v in n.writes if k == (0x6040, 0)], [0x80])

    def test_an_sdo_abort_on_shutdown_is_explained(self):
        from canopen.sdo.exceptions import SdoAbortedError
        n = self.node(sw=0x40, code=0x4210)

        def refuse(v):
            if v == 0x06:
                raise SdoAbortedError(0x08000022)
        n.hook_write('ControlWord', None, refuse)
        with self.assertRaises(cia402.EnableRefused) as cm:
            cia402.enable(n, sleep=lambda s: None)
        msg = str(cm.exception)
        self.assertIn('Shutdown', msg)
        self.assertIn('amplifier over-temperature', msg)
        self.assertIn('0x08000022', msg)

    def test_a_drive_that_does_not_answer_is_not_a_refusal(self):
        n = fake.FakeNode()     # no 0x6041: a fake, or old firmware
        cia402.enable(n, sleep=lambda s: None)

    def test_state_names(self):
        self.assertEqual(cia402.state_name(0x0237), 'Operation Enabled')
        self.assertEqual(cia402.state_name(0x0250), 'Switch On Disabled')
        self.assertEqual(cia402.state_name(0x0238), 'Fault')
