"""The PDO mapping the Barrett tools work with, set up when they connect.

The config CSV leaves the PDOs to the customer, so a puck's NV may hold any
mapping or none.  The tools write this one instead, in RAM: the next reboot
goes back to what NV holds, so whoever reboots a puck applies it again.

RPDO3 and RPDO4 stay as they are.  nvsave.save_settings leaves this mapping out
of NV; Save.All would store it, so a config load reboots first.
"""

#: PDO number: (COB-ID base, transmission type, [mapped objects]).  Type 0 and
#: type 1 both act on every SYNC on this firmware.
RPDOS = {
    1: (0x200, 0, [(0x6040, 0),     # ControlWord
                   (0x6060, 0),     # SetModeOfOperation
                   (0x6071, 0)]),   # TargetTorque
    2: (0x300, 0, [(0x60FF, 0),     # TargetVelocity
                   (0x607A, 0)]),   # TargetPosition
}
TPDOS = {
    1: (0x180, 0, [(0x6041, 0),     # StatusWord
                   (0x6061, 0),     # ModeOfOperationDisplay
                   (0x6064, 0)]),   # PositionFeedback
    2: (0x280, 0, [(0x606C, 0),     # VelocityFeedback
                   (0x6078, 0)]),   # CurrentFeedback
    3: (0x380, 10, [('Amplifier', 'Temperature'),
                    ('Motor', 'Therm')]),
    4: (0x480, 1, [('Motor', 'id')]),
}


def write_pdo(pdo_map, cob_id, trans_type, objects):
    """Replace one PDO's mapping and enable it (RAM only)."""
    pdo_map.clear()
    for index, sub in objects:
        if pdo_map.add_variable(index, sub) is None:
            raise KeyError('{},{} is not in the EDS'.format(index, sub))
    pdo_map.cob_id = cob_id
    pdo_map.trans_type = trans_type
    pdo_map.enabled = True
    pdo_map.save()


def apply_default_pdos(node):
    """Write the default mapping to *node* and read back the rest.

    An SDO abort is left to the caller: a node without this mapping cannot be
    commanded by the tools."""
    node.tpdo.read()
    node.rpdo.read()
    for number, (base, trans_type, objects) in RPDOS.items():
        write_pdo(node.rpdo[number], base + node.id, trans_type, objects)
    for number, (base, trans_type, objects) in TPDOS.items():
        # read() skips these for a SYNC type, and an old value would be saved.
        node.tpdo[number].inhibit_time = 0
        node.tpdo[number].event_timer = 0
        write_pdo(node.tpdo[number], base + node.id, trans_type, objects)
