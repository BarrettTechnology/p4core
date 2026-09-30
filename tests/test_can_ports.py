"""
Tests for can_backend's port listing -- SocketCAN interface enumeration from sysfs.

A fake ``/sys/class/net`` tree is built in a temp directory (mirroring the
real symlink layout for PCI and USB CAN adapters) so the parsing/formatting
logic can be exercised without real hardware.
"""
import os
import sys
import shutil
import tempfile
import unittest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))

from p4core import can_backend as can_ports  # noqa: E402


class TestCanPorts(unittest.TestCase):

    def setUp(self):
        self.tmp = tempfile.mkdtemp()
        self._orig_sys_net = can_ports.SYS_NET

        # Real device trees live under <tmp>/devices; <tmp>/net holds the
        # canN interface dirs whose ``device`` symlink points into them.
        self.net = os.path.join(self.tmp, 'net')
        os.makedirs(self.net)
        can_ports.SYS_NET = self.net

    def tearDown(self):
        can_ports.SYS_NET = self._orig_sys_net
        shutil.rmtree(self.tmp, ignore_errors=True)

    # -- helpers -------------------------------------------------------
    def _write(self, path, text):
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, 'w') as fh:
            fh.write(text)

    def _make_iface(self, name, operstate):
        iface = os.path.join(self.net, name)
        os.makedirs(iface)
        self._write(os.path.join(iface, 'operstate'), operstate + '\n')
        return iface

    def _add_pci(self, name, addr, vendor, device, operstate='down'):
        iface = self._make_iface(name, operstate)
        dev = os.path.join(self.tmp, 'devices', 'pci', addr)
        os.makedirs(dev, exist_ok=True)  # can0/can1 may share one PCI device
        self._write(os.path.join(dev, 'vendor'), vendor + '\n')
        self._write(os.path.join(dev, 'device'), device + '\n')
        bus = os.path.join(self.tmp, 'bus', 'pci')
        os.makedirs(bus, exist_ok=True)
        sub = os.path.join(dev, 'subsystem')
        if not os.path.lexists(sub):
            os.symlink(bus, sub)
        os.symlink(dev, os.path.join(iface, 'device'))

    def _add_usb(self, name, usbdev, vendor, product, operstate='up'):
        iface = self._make_iface(name, operstate)
        dev_root = os.path.join(self.tmp, 'devices', 'usb', usbdev)
        intf = os.path.join(dev_root, usbdev + ':1.0')
        os.makedirs(intf)
        self._write(os.path.join(dev_root, 'idVendor'), vendor + '\n')
        self._write(os.path.join(dev_root, 'idProduct'), product + '\n')
        bus = os.path.join(self.tmp, 'bus', 'usb')
        os.makedirs(bus, exist_ok=True)
        os.symlink(bus, os.path.join(intf, 'subsystem'))
        os.symlink(intf, os.path.join(iface, 'device'))

    # -- PCI -----------------------------------------------------------
    def test_pci_alias_peak(self):
        # PEAK PCAN-PCI: vendor 001c -> "Peak" alias replaces vendor:product@bus.
        self._add_pci('can0', '0000:04:01.0', '0x001c', '0x0001', operstate='down')
        info = can_ports.port_info('can0')
        self.assertEqual(info['bus'], 'PCI')
        self.assertEqual(info['vendor'], '001c')
        self.assertEqual(info['alias'], 'Peak')
        self.assertEqual(info['label'], 'can0, PCI (Peak), down')

    def test_pci_no_alias_uses_vendor_product_at_bus(self):
        self._add_pci('can0', '0000:04:01.0', '0x1234', '0x5678', operstate='up')
        info = can_ports.port_info('can0')
        self.assertIsNone(info['alias'])
        self.assertEqual(info['busid'], '04:01.0')          # domain stripped
        self.assertEqual(info['label'], 'can0, PCI (1234:5678 @ 04:01.0), up')

    # -- USB -----------------------------------------------------------
    def test_usb_alias_canable(self):
        self._add_usb('can2', '2-2.2', '1d50', '606f', operstate='down')
        info = can_ports.port_info('can2')
        self.assertEqual(info['alias'], 'CANable')
        self.assertEqual(info['label'], 'can2, USB (CANable), down')

    def test_usb_alias_peak(self):
        self._add_usb('can3', '2-9.7.1', '0c72', '000c', operstate='up')
        info = can_ports.port_info('can3')
        self.assertEqual(info['alias'], 'Peak')
        self.assertEqual(info['label'], 'can3, USB (Peak), up')

    def test_usb_no_alias_uses_vendor_product_at_bus(self):
        self._add_usb('can4', '2-2.4', 'abcd', '1234', operstate='up')
        info = can_ports.port_info('can4')
        self.assertIsNone(info['alias'])
        self.assertEqual(info['busid'], '2-2.4')
        self.assertEqual(info['label'], 'can4, USB (abcd:1234 @ 2-2.4), up')

    def test_uppercase_sysfs_id_is_normalised(self):
        # sysfs is normally lower-case, but be robust if a vendor id is upper.
        self._add_usb('can5', '2-2.5', '1D50', '606F', operstate='up')
        info = can_ports.port_info('can5')
        self.assertEqual(info['alias'], 'CANable')

    # -- enumeration ---------------------------------------------------
    def test_list_sorted_and_complete(self):
        self._add_usb('can2', '2-2.2', '1d50', '606f')
        self._add_pci('can0', '0000:04:01.0', '0x001c', '0x0001')
        self._add_pci('can1', '0000:04:01.0', '0x001c', '0x0001')
        ports = can_ports.list_can_ports()
        self.assertEqual([p['port'] for p in ports], ['can0', 'can1', 'can2'])

    def test_missing_device_is_graceful(self):
        # An interface with no ``device`` symlink (e.g. vcan) must not crash.
        self._make_iface('can9', 'down')
        info = can_ports.port_info('can9')
        self.assertEqual(info['bus'], '?')
        self.assertEqual(info['label'], 'can9, ? (?), down')


if __name__ == '__main__':
    unittest.main()
