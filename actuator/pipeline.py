"""Save.All, the last step of a calibration.

One Save.All (0x1010,1): a burst of Save.Single starved the IWDG on STM32
firmware before 53536c2.
"""

import struct


SAVE_SIGNATURE = 0x65766173          # "save", CiA 301 0x1010
OD_SAVE = (0x1010, 1)


def save_all(dev):
    """Every object to NVM (~0.55 s on the drive)."""
    sdo = dev.node.sdo
    old = getattr(sdo, 'RESPONSE_TIMEOUT', None)
    try:
        if old is not None and old < 2.0:
            sdo.RESPONSE_TIMEOUT = 2.0
        sdo.download(OD_SAVE[0], OD_SAVE[1], struct.pack('<I', SAVE_SIGNATURE))
    finally:
        if old is not None:
            sdo.RESPONSE_TIMEOUT = old
