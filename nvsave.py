"""The 0x1010 saves to NV, and which job uses which.

Each sub stores one range of the object dictionary, so a tool stores what it
changed and leaves the rest of NV alone:

  save_settings   Mfg (0x2000-0x5FFF) + App (0x6000-0x9FFF): calibration, tuning
                  and anything else typed in.  Leaves communication out, so the
                  PDO map the tools run with (pdo_defaults) and a heartbeat they
                  turned off stay in RAM, and the customer's PDOs stay in NV.
  save_comm       Comm (0x1000-0x1FFF): what a node ID change rewrites.
  save_all        All: a config CSV, which holds rows in every range.  Reboot
                  before loading one, so RAM holds only NV and the file.

Older firmware has no Mfg save (0x1010,5), and its App and Comm saves answer
success and store nothing, so there these all fall back to Save.All.
"""

try:
    from canopen.sdo.exceptions import SdoCommunicationError
except ImportError:                     # pragma: no cover - canopen is a dependency
    SdoCommunicationError = Exception

SAVE_SIGNATURE = 0x65766173             # "save", CiA 301 0x1010
ALL, COMM, APP, SINGLE, MFG = 1, 2, 3, 4, 5
TIMEOUT_S = 2.0                         # a Save.All takes ~0.55 s on the drive


def _save(node, sub):
    sdo = node.sdo
    old = getattr(sdo, 'RESPONSE_TIMEOUT', None)
    try:
        if old is not None and old < TIMEOUT_S:
            sdo.RESPONSE_TIMEOUT = TIMEOUT_S
        sdo.download(0x1010, sub, SAVE_SIGNATURE.to_bytes(4, 'little'))
    finally:
        if old is not None:
            sdo.RESPONSE_TIMEOUT = old


def has_ranges(node):
    """True when the ranged saves work.  Asked at every save, not cached: after
    a flash back to older firmware they would answer success and store nothing."""
    try:
        node.sdo.upload(0x1010, MFG)
        return True
    except SdoCommunicationError:       # no answer: not evidence of absence
        raise
    except Exception:                   # abort, or absent from a test fake
        return False


def save_settings(node):
    """Store everything but communication: the drive's settings."""
    if has_ranges(node):
        _save(node, MFG)
        _save(node, APP)
    else:
        _save(node, ALL)


def save_comm(node):
    """Store communication and identity (0x1000-0x1FFF)."""
    _save(node, COMM if has_ranges(node) else ALL)


def save_all(node):
    """Store every NV row: for a config CSV, after a reboot before the load."""
    _save(node, ALL)
