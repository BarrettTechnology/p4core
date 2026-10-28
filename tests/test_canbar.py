"""canbar: the CAN port bar the GUIs share, and can_backend.port_choices."""

import os
import sys
import unittest
from unittest import mock

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__),
                                                '..', '..')))

from p4core import can_backend                              # noqa: E402

try:
    import wx
    from p4core import canbar
except ImportError:                       # headless install: no wx
    wx = None

PORTS = [{'port': 'can0', 'label': 'can0, USB (CANable), up'},
         {'port': 'can2', 'label': 'can2, PCI (Peak), up'}]


class TestPortChoices(unittest.TestCase):
    def test_windows_lists_the_pcan_channels(self):
        with mock.patch.object(can_backend.platform, 'system',
                               return_value='Windows'):
            ports = can_backend.port_choices()
        self.assertEqual([p['port'] for p in ports],
                         ['PCAN_USBBUS{}'.format(i) for i in range(1, 5)])

    def test_linux_lists_the_socketcan_ports(self):
        with mock.patch.object(can_backend.platform, 'system',
                               return_value='Linux'), \
                mock.patch.object(can_backend, 'list_can_ports',
                                  return_value=PORTS):
            self.assertEqual(can_backend.port_choices(), PORTS)


@unittest.skipIf(wx is None or not os.environ.get('DISPLAY'), 'no wx display')
class TestCanBar(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = wx.App.Get() or wx.App(False)

    def setUp(self):
        self.frame = wx.Frame(None)
        self.bar = canbar.CanBar(self.frame, ports=PORTS)

    def tearDown(self):
        self.frame.Destroy()

    def test_ports_carry_their_names(self):
        self.assertEqual(self.bar.choice_port.GetStringSelection(),
                         PORTS[0]['label'])
        self.assertEqual(self.bar.port(), 'can0')
        self.assertTrue(self.bar.select_port('can2'))
        self.assertEqual(self.bar.port(), 'can2')

    def test_a_port_not_plugged_in_is_added_by_name(self):
        self.assertFalse(self.bar.select_port('can3', add=False))
        self.assertTrue(self.bar.select_port('can3'))
        self.assertEqual((self.bar.port(),
                          self.bar.choice_port.GetStringSelection()),
                         ('can3', 'can3'))

    def test_no_port_until_one_is_picked(self):
        self.assertFalse(self.bar.select_port(None))
        self.assertIsNone(self.bar.port())
        self.assertEqual(self.bar.choice_port.GetCount(), 2)   # still listed

    def test_fill_ports_keeps_a_named_selection(self):
        self.bar.fill_ports(PORTS, select='can2')
        self.assertEqual(self.bar.port(), 'can2')
        self.bar.fill_ports([])
        self.assertIsNone(self.bar.port())

    def test_nodes(self):
        self.assertIsNone(self.bar.selected_id())
        self.assertEqual(self.bar.set_nodes([8, 4, 7]), 4)
        self.assertEqual(self.bar.choice_id.GetItems(), ['4', '7', '8'])
        self.assertEqual(self.bar.set_nodes([8, 4, 7], select=7), 7)
        self.assertEqual((self.bar.selected_id(), self.bar.nodes()),
                         (7, [4, 7, 8]))
        self.assertEqual(self.bar.set_nodes([4], select=7), 4)
        self.assertIsNone(self.bar.set_nodes([]))
        self.assertEqual(self.bar.choice_id.GetItems(), ['None'])
        self.assertEqual(self.bar.nodes(), [])

    def test_bind(self):
        seen = []
        self.bar.bind(on_scan=lambda e: seen.append('scan'),
                      on_select=lambda e: seen.append('select'),
                      on_port=lambda e: seen.append('port'))
        for widget, kind in ((self.bar.btn_scan, wx.wxEVT_BUTTON),
                             (self.bar.choice_id, wx.wxEVT_CHOICE),
                             (self.bar.choice_port, wx.wxEVT_CHOICE)):
            event = wx.CommandEvent(kind, widget.GetId())
            event.SetEventObject(widget)
            widget.GetEventHandler().ProcessEvent(event)
        self.assertEqual(seen, ['scan', 'select', 'port'])

    def test_sizer_holds_the_bar_in_order(self):
        self.assertEqual([it.GetWindow() for it in self.bar.sizer.GetChildren()],
                         [self.bar.label_can_port, self.bar.choice_port,
                          self.bar.btn_scan, self.bar.label_select_id,
                          self.bar.choice_id])


if __name__ == '__main__':
    unittest.main()
