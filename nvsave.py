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

A save is not over when the SDO is acknowledged.  Firmware 5.x acknowledges
once the flash write is queued and commits it from the main loop, with the
motor outputs off; newer builds show the queued write as bit 2 of every
0x1010 sub ("save pending") and refuse a save while the outputs are on
(abort 0x08000022).  So after each save _save() reads 0x1010,<sub> until bit
2 clears.  On firmware without the bit it reads 0 at once, so a first read
without the bit costs one more round-trip and a FALLBACK_WAIT_S wait, which
covers a ranged commit there; a reboot or a power cut right after a save then
no longer loses it.
"""

import time

try:
    from canopen.sdo.exceptions import SdoAbortedError, SdoCommunicationError
except ImportError:                     # pragma: no cover - canopen is a dependency
    SdoAbortedError = SdoCommunicationError = Exception

SAVE_SIGNATURE = 0x65766173             # "save", CiA 301 0x1010
ALL, COMM, APP, SINGLE, MFG = 1, 2, 3, 4, 5
TIMEOUT_S = 2.0                         # a Save.All takes ~0.55 s on the drive

SAVE_PENDING = 0x4                      # 0x1010,<sub> bit 2: queued, not committed
POLL_INTERVAL_S = 0.01
POLL_TIMEOUT_S = 0.5                    # ranged saves
POLL_TIMEOUT_ALL_S = TIMEOUT_S          # Save.All writes every page
FALLBACK_WAIT_S = 0.06                  # firmware without the pending bit
ABORT_DEVICE_STATE = 0x08000022


class SaveRefused(SdoAbortedError if SdoAbortedError is not Exception
                  else RuntimeError):
    """The drive refused a save because its outputs are on (0x08000022).

    Still an SdoAbortedError (code 0x08000022), so callers that handle aborts
    keep working; the message says what to do."""

    def __init__(self, node_id, sub):
        self.code = ABORT_DEVICE_STATE
        self.node_id = node_id
        self.sub = sub

    def __str__(self):
        return ('node {}: save refused (0x1010,{}, abort 0x{:08X}): disable '
                'the drive before saving'.format(self.node_id, self.sub,
                                                 ABORT_DEVICE_STATE))


class SaveTimeout(SdoCommunicationError if SdoCommunicationError is not Exception
                  else RuntimeError):
    """0x1010,<sub> still showed the save pending at the deadline."""


def _save(node, sub):
    sdo = node.sdo
    old = getattr(sdo, 'RESPONSE_TIMEOUT', None)
    try:
        if old is not None and old < TIMEOUT_S:
            sdo.RESPONSE_TIMEOUT = TIMEOUT_S
        try:
            sdo.download(0x1010, sub, SAVE_SIGNATURE.to_bytes(4, 'little'))
        except SdoAbortedError as exc:
            if getattr(exc, 'code', None) == ABORT_DEVICE_STATE:
                raise SaveRefused(getattr(node, 'id', '?'), sub) from exc
            raise
        wait_committed(node, sub)
    finally:
        if old is not None:
            sdo.RESPONSE_TIMEOUT = old


def _pending(raw):
    """Bit 2 of an 0x1010 read; None when the answer is not a U32 (a fake)."""
    try:
        return bool(int.from_bytes(bytes(raw)[:4], 'little') & SAVE_PENDING)
    except Exception:
        return None


def wait_committed(node, sub, timeout=None, sleep=None, clock=None):
    """Return once the save on 0x1010,*sub* is in flash.

    Reads 0x1010,*sub* every POLL_INTERVAL_S until bit 2 clears.  When the
    first read already shows it clear (firmware without the bit, or a commit
    that beat the read) wait FALLBACK_WAIT_S more.  Raises SaveTimeout when the
    bit is still set, or the node stops answering, past the deadline."""
    sleep = sleep or time.sleep
    clock = clock or time.monotonic
    if timeout is None:
        timeout = POLL_TIMEOUT_ALL_S if sub == ALL else POLL_TIMEOUT_S
    deadline = clock() + timeout
    seen = False
    while True:
        try:
            pending = _pending(node.sdo.upload(0x1010, sub))
        except SdoCommunicationError:
            if clock() >= deadline:     # busy committing, then gone
                raise SaveTimeout('node {}: no answer to 0x1010,{} after the '
                                  'save'.format(getattr(node, 'id', '?'), sub))
            sleep(POLL_INTERVAL_S)
            continue
        except Exception:               # an abort: this sub cannot be read
            pending = None
        if not pending:
            if not seen:
                sleep(FALLBACK_WAIT_S)
            return
        seen = True
        if clock() >= deadline:
            raise SaveTimeout('node {}: 0x1010,{} still shows the save pending '
                              'after {:.1f} s'.format(getattr(node, 'id', '?'),
                                                      sub, timeout))
        sleep(POLL_INTERVAL_S)


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
