"""p4core.actuator.encfilter: the encoder hardware filter from the product code."""

import os
import struct
import sys
import unittest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__),
                                                '..', '..')))

from p4core.actuator import encfilter, encoder, steps                  # noqa: E402


class _Sdo:
    def __init__(self, od):
        self.od = od
        self.saved = []

    def upload(self, index, sub):
        if (index, sub) not in self.od:
            raise KeyError((index, sub))
        v = self.od[(index, sub)]
        return struct.pack('<B' if (index, sub) in ((0x3013, 6), (0x3013, 12)) else '<I', v)

    def download(self, index, sub, data):
        self.od[(index, sub)] = data[0] if len(data) == 1 else \
            struct.unpack('<I', data)[0]

    def __getitem__(self, name):
        sdo = self

        class _Rec:
            def __getitem__(self, sub):
                class _V:
                    @property
                    def raw(self):
                        return 0

                    @raw.setter
                    def raw(self, v):
                        sdo.saved.append(v)
                return _V()
        return _Rec()


class Node:
    def __init__(self, code, filt, readback=0, chip=None):
        od = {(0x1018, 2): code, (0x3013, 6): filt, (0x3013, 9): readback}
        if chip is not None:
            od[(0x3013, 12)] = chip
        self.sdo = _Sdo(od)

def _save(node, index, sub):
    node.sdo.saved.append((index << 8) | sub)


class TestChoice(unittest.TestCase):
    def test_ma782_is_fw7(self):
        k, why = encfilter.choose('MA782')
        self.assertEqual(k, 0x70)
        self.assertIn('quietest', why)
        s = encfilter.ENCODERS['MA782'][k]
        self.assertLessEqual(encfilter.lag_deg(s), encfilter.LAG_BUDGET_DEG)
        # the next quieter one is over the budget
        self.assertGreater(encfilter.lag_deg(encfilter.ENCODERS['MA782'][0x80]),
                           encfilter.LAG_BUDGET_DEG)

    def test_lower_crossover_allows_more_filter(self):
        self.assertGreater(encfilter.choose('MA782', f_hz=25.0)[0], 0x70)

    def test_ma736_is_fw9(self):
        k, why = encfilter.choose('MA736')
        self.assertEqual(k, 0x90)
        self.assertIn('quietest', why)
        # the factory default is over the budget
        self.assertGreater(encfilter.lag_deg(encfilter.ENCODERS['MA736'][0xA0]),
                           encfilter.LAG_BUDGET_DEG)

    def test_type2_lag(self):
        s = encfilter.ENCODERS['MA736'][0xA0]
        self.assertAlmostEqual(encfilter.lag_deg(s, 1e-3), 0.0, places=3)
        self.assertAlmostEqual(encfilter.lag_deg(s, 100.0), 15.1, delta=0.5)

    def test_ma736_datasheet_matches_fit(self):
        # FW 10, 1024 us: 155 Hz, zeta 1 -- within 4 % of the bench fit
        s = encfilter._type2(1024.0, '')
        self.assertAlmostEqual(s['fn_hz'], 155.4, delta=0.1)

    def test_spread_limit(self):
        self.assertEqual(encfilter.spread_limit('MA782', 0x70, 8), 8)
        self.assertEqual(encfilter.spread_limit('MA782', 0x30, 8), 28)
        self.assertEqual(encfilter.spread_limit('MA736', 0xA0, 8), 8)
        self.assertEqual(encfilter.spread_limit('MA702', 0, 8), 8)
        self.assertEqual(encfilter.spread_limit(None, None, 8), 8)

    def test_tracking_model(self):
        m = encoder.FILTER_MODELS
        self.assertEqual(encfilter.tracking_model('MA736', 0xA0, m), (150, 880))
        self.assertEqual(encfilter.tracking_model('MA736', 0x90, m), (311, 1000))
        self.assertEqual(encfilter.tracking_model('MA736', 0xF0, m), (0, 0))
        self.assertEqual(encfilter.tracking_model('MA702', 0, m), (163, 1000))
        self.assertEqual(encfilter.tracking_model('MA782', 0xA0, m), (0, 0))
        self.assertEqual(encfilter.tracking_model(None, 0xA0, m), (150, 880))
        self.assertEqual(encfilter.tracking_model(None, 0x90, m), (0, 0))


class TestMA735(unittest.TestCase):
    def test_values_and_tau(self):
        self.assertEqual(encfilter.MA735_VALUES,
                         (0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xAA, 0xBB))
        self.assertAlmostEqual(encfilter.ma735_tau_us(0x77), 1024.0)
        self.assertAlmostEqual(encfilter.ma735_tau_us(0xBB), 16384.0)
        for v in (0x00, 0x70, 0x90, 0xA0, 0x22, 0xCC, 0x76, None):
            self.assertFalse(encfilter.is_ma735_value(v))


class TestPlan(unittest.TestCase):
    def test_p416_from_blank(self):
        n = Node(5707, 0x30, readback=0x30)
        p = encfilter.plan(n)
        self.assertEqual((p['model'], p['encoder']), ('P4-16', 'MA782'))
        self.assertEqual((p['current'], p['target'], p['running']),
                         (0x30, 0x70, 0x30))
        self.assertTrue(p['change'])
        encfilter.apply(n, p, _save)
        self.assertEqual(n.sdo.od[(0x3013, 6)], 0x70)
        self.assertEqual(n.sdo.saved, [0x301306])

    def test_p442_factory_default_moves_to_fw9(self):
        p = encfilter.plan(Node(5755, 0xA0))
        self.assertEqual((p['model'], p['encoder']), ('P4-42', 'MA736'))
        self.assertEqual(p['target'], 0x90)
        self.assertTrue(p['change'])
        self.assertIsNone(p['running'])       # STM32: no read-back

    def test_p432_is_ma736(self):
        p = encfilter.plan(Node(5760, 0x90))
        self.assertEqual((p['model'], p['encoder']), ('P4-32', 'MA736'))
        self.assertFalse(p['change'])

    def test_p437_fixed_filter_left_alone(self):
        p = encfilter.plan(Node(1323, 0, readback=0x5C))
        self.assertEqual((p['model'], p['encoder']), ('P4-37', 'MA702'))
        self.assertEqual(p['target'], 0)
        self.assertFalse(p['change'])
        self.assertIsNone(p['running'])       # 0x0E is not its filter
        self.assertIn('fixed', p['reason'])

    def test_p442_ma735_from_the_read_back(self):
        # Same product code; the register read back at boot (0x77, the
        # MA735's default) says MA735.  The MA736 setting in 0x3013,6 was
        # refused by the sensor and moves to the MA735's 512 us.
        p = encfilter.plan(Node(5755, 0x90, readback=0x77))
        self.assertEqual((p['model'], p['encoder']), ('P4-42', 'MA735'))
        self.assertEqual((p['target'], p['running']), (0x66, 0x77))
        self.assertTrue(p['change'])
        self.assertEqual('MA735', encfilter.encoder_of(
            Node(5755, 0x66, readback=0x66)))

    def test_ma736_read_back_stays_ma736(self):
        p = encfilter.plan(Node(5755, 0x90, readback=0x90))
        self.assertEqual(p['encoder'], 'MA736')
        self.assertFalse(p['change'])

    def test_firmware_chip_wins_over_the_read_back(self):
        # stm32 f226b61: 0x3013,12 from the registers' factory defaults.
        # An MA735 with an MA736 value burned in (node 8, 2026-10-21) is
        # still an MA735, and its own window goes back.
        p = encfilter.plan(Node(5755, 0x66, readback=0xA0, chip=2))
        self.assertEqual(p['encoder'], 'MA735')
        self.assertEqual(p['target'], 0x66)
        self.assertEqual('MA736', encfilter.encoder_of(
            Node(5755, 0x90, readback=0x90, chip=1)))
        # 0 = not identified: the read-back decides, as before.
        self.assertEqual('MA735', encfilter.encoder_of(
            Node(5755, 0x77, readback=0x77, chip=0)))

    def test_ma735_value_only_on_an_ma736_board(self):
        # A K64 MA782 read-back never turns into an MA735.
        self.assertEqual('MA782', encfilter.encoder_of(
            Node(5707, 0x70, readback=0x77)))

    def test_unknown_encoder_left_alone(self):
        p = encfilter.plan(Node(1950 + 1, 0))
        self.assertIsNone(p['encoder'])
        self.assertIsNone(p['target'])
        self.assertFalse(p['change'])
        self.assertIn('not known', p['reason'])

    def test_ascii_product_code(self):
        p = encfilter.plan(Node(int.from_bytes(b'P416', 'big'), 0x70))
        self.assertEqual(p['encoder'], 'MA782')


class _Dev:
    def __init__(self, node):
        self.node = node
        self.logs = []

    def _log(self, m):
        self.logs.append(m)


class TestEncoderFilterStep(unittest.TestCase):
    def test_change_saves_and_reboots(self):
        n = Node(5707, 0x30, readback=0x30)
        fresh = Node(5707, 0x70, readback=0x70)
        boots = []

        def reboot():
            boots.append(1)
            return fresh
        s = steps.Session(_Dev(n), reboot=reboot)
        p, note = steps._encoder_filter(s)
        self.assertEqual(boots, [1])
        self.assertIs(s.dev.node, fresh)
        self.assertEqual(note, 'filter 0x70 (was 0x30)')
        self.assertEqual(n.sdo.saved, [0x301306])

    def test_no_reboot_stops(self):
        s = steps.Session(_Dev(Node(5707, 0x30)))
        with self.assertRaisesRegex(steps.StepFailed, 'reboot the drive'):
            steps._encoder_filter(s)

    def test_unsaved_values_refuse_the_reboot(self):
        s = steps.Session(_Dev(Node(5707, 0x30)), reboot=lambda: None)
        s.unsaved = True
        with self.assertRaisesRegex(steps.StepFailed, 'Save first'):
            steps._encoder_filter(s)

    def test_sensor_not_running_it(self):
        s = steps.Session(_Dev(Node(5707, 0x70, readback=0x30)))
        with self.assertRaisesRegex(steps.StepFailed, 'runs filter 0x30'):
            steps._encoder_filter(s)


if __name__ == '__main__':
    unittest.main()
