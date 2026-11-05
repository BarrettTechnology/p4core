"""The default PDO mapping, through the real canopen PDO code and EDS."""
import unittest

import canopen

from p4core import paths, pdo_defaults


class Puck:
    """An object dictionary answering the SDO calls; unset objects read 0."""

    def __init__(self, node_id=5):
        self.od = {}
        self.writes = []
        self.net = canopen.Network()
        self.node = canopen.RemoteNode(node_id, paths.data('puck4.eds'))
        self.net.add_node(self.node)
        self.node.sdo.upload = self.upload
        self.node.sdo.download = self.download

    def upload(self, index, sub):
        var = self.node.object_dictionary[index]
        if isinstance(var, (canopen.objectdictionary.ODRecord, canopen.objectdictionary.ODArray)):
            var = var[sub]
        return var.encode_raw(self.od.get((index, sub), 0))

    def download(self, index, sub, data, force_segment=False):
        value = int.from_bytes(data, 'little')
        self.od[index, sub] = value
        self.writes.append((index, sub, value))


class Defaults(unittest.TestCase):
    def setUp(self):
        self.puck = Puck()
        self.node = self.puck.node

    def test_a_blank_puck_gets_the_mapping_the_tools_use(self):
        pdo_defaults.apply_default_pdos(self.node)
        od = self.puck.od
        self.assertEqual([od[0x1600, i] for i in range(4)], [3, 0x60400010, 0x60600008, 0x60710010])
        self.assertEqual([od[0x1601, i] for i in range(3)], [2, 0x60FF0020, 0x607A0020])
        self.assertEqual([od[0x1A00, i] for i in range(4)], [3, 0x60410010, 0x60610008, 0x60640020])
        self.assertEqual([od[0x1A01, i] for i in range(3)], [2, 0x606C0020, 0x60780010])
        self.assertEqual(od[0x1A02, 0], 2)
        self.assertEqual(od[0x1A03, 0], 1)
        for index, cob, typ in ((0x1400, 0x205, 0), (0x1401, 0x305, 0), (0x1800, 0x185, 0),
                                (0x1801, 0x285, 0), (0x1802, 0x385, 10), (0x1803, 0x485, 1)):
            self.assertEqual((od[index, 1], od[index, 2]), (cob, typ), hex(index))

    def test_the_names_the_tools_look_up_resolve(self):
        pdo_defaults.apply_default_pdos(self.node)
        for pdo, names in ((self.node.rpdo[1], ('ControlWord', 'SetModeOfOperation', 'TargetTorque')),
                           (self.node.rpdo[2], ('TargetVelocity', 'TargetPosition')),
                           (self.node.tpdo[1], ('PositionFeedback',)),
                           (self.node.tpdo[2], ('VelocityFeedback', 'CurrentFeedback')),
                           (self.node.tpdo[3], ('Amplifier.Temperature', 'Motor.Therm')),
                           (self.node.tpdo[4], ('Motor.id',))):
            for name in names:
                pdo[name]

    def test_replaces_a_customer_mapping_and_clears_its_timers(self):
        self.puck.od.update({(0x1A00, 0): 1, (0x1A00, 1): 0x60FF0020, (0x1800, 1): 0x195,
                             (0x1800, 2): 255, (0x1800, 3): 50, (0x1800, 5): 20})
        pdo_defaults.apply_default_pdos(self.node)
        od = self.puck.od
        self.assertEqual((od[0x1A00, 0], od[0x1A00, 1]), (3, 0x60410010))
        self.assertEqual((od[0x1800, 1], od[0x1800, 2], od[0x1800, 3], od[0x1800, 5]), (0x185, 0, 0, 0))

    def test_rpdo3_and_4_stay_the_customers(self):
        pdo_defaults.apply_default_pdos(self.node)
        self.assertFalse([w for w in self.puck.writes if w[0] in (0x1402, 0x1403, 0x1602, 0x1603)])

    def test_never_saves_to_nv(self):
        pdo_defaults.apply_default_pdos(self.node)
        self.assertFalse([w for w in self.puck.writes if w[0] == 0x1010])

    def test_callbacks_survive(self):
        seen = []
        self.node.tpdo[1].add_callback(seen.append)
        pdo_defaults.apply_default_pdos(self.node)
        self.assertEqual(self.node.tpdo[1].callbacks, [seen.append])


class SessionApplies(unittest.TestCase):
    def setUp(self):
        from p4core.session import Session
        self.session = Session()
        self.puck = Puck()
        self.session.network = self.puck.net
        self.puck.net.send_message = lambda *args, **kwargs: None

    def mapped(self):
        return self.puck.od.get((0x1A00, 0))

    def test_select_node(self):
        self.session._read_drive_scaling = lambda: None
        self.session.select_node(5)
        self.assertEqual(self.mapped(), 3)

    def test_an_extra_node_after_it_reboots(self):
        self.session.configure_extra_node(5)
        self.assertEqual(self.mapped(), 3)
        self.puck.od.clear()
        self.session.reset_node(5, reselect=False, sleep=lambda s: None)
        self.session.configure_extra_node(5)
        self.assertEqual(self.mapped(), 3)


if __name__ == '__main__':
    unittest.main()
