"""can_backend.make_network: sends wait for TX queue space."""

import os
import sys
import unittest
from unittest import mock

sys.path.insert(0, os.path.dirname(__file__))

import fake  # noqa: E402,F401  (puts p4core on the path)

from p4core import can_backend                                 # noqa: E402


class Bus:
    def __init__(self):
        self.calls = []

    def send(self, msg, timeout=None):
        self.calls.append(timeout)


class Network:
    """A canopen.Network stand-in: connect() makes the bus."""

    def __init__(self, bus=None):
        self.bus = bus

    def connect(self, **kwargs):
        self.bus = Bus()


class FdNetwork(Network):
    can_fd = False              # p4core.session.P4Network's mark


def make(cls, fd=False):
    with mock.patch.object(can_backend.platform, 'system', return_value='Linux'):
        return can_backend.make_network('can0', fd=fd, network_cls=cls)


class TestTxBackpressure(unittest.TestCase):

    def test_send_gets_a_timeout(self):
        net = make(Network)
        net.bus.send(object())
        self.assertEqual(net.bus.calls, [can_backend.TX_TIMEOUT])

    def test_caller_timeout_kept(self):
        net = make(Network)
        net.bus.send(object(), timeout=2)
        net.bus.send(object(), 3)
        self.assertEqual(net.bus.calls, [2, 3])

    def test_composes_with_fd(self):
        net = make(Network, fd=True)
        msg = mock.Mock()
        net.bus.send(msg)
        self.assertTrue(msg.is_fd)
        self.assertEqual(net.bus.calls, [can_backend.TX_TIMEOUT])

    def test_p4network_not_wrapped(self):
        net = make(FdNetwork)
        net.bus.send(object())
        self.assertEqual(net.bus.calls, [None])


if __name__ == '__main__':
    unittest.main()
