"""NV saves on firmware 5.0.7+: only in NMT PRE-OPERATIONAL, never with the outputs on.

Firmware 5.0.7 refuses a write to 0x1010 (save) or 0x1011 (restore defaults) with
abort 0x08000022 ("present device state") unless the node is in NMT
PRE-OPERATIONAL and its motor outputs are off (stm32 ``common/parse.c``
``parseNvGate()``; FIRMWARE_UPDATES 5.0.6/5.0.7). On the STM32 a page erase drops
SYNCs for ~22 ms, which in PRE-OP costs no PDO. Older firmware never refuses.

The apps write 0x1010/0x1011 in many places (p4core, p4sysid, the pucktuner GUI,
``node.sdo['Save'][...].raw = ...`` or ``node.sdo.download(0x1010, ...)``). Rather
than changing every one, ``install()`` wraps the one function they all end in,
``canopen.sdo.client.SdoClient.download``. For an index other than 0x1010/0x1011
nothing changes. For a save or restore:

  1. It is sent as before. Older firmware, or a node already in PRE-OP, takes it.
  2. Only if it is refused with 0x08000022:
     - statusword (0x6041) says Operation Enabled -> the outputs are on. Raised
       again unchanged (NmtSaveRefused): the save has to wait for the disable,
       and switching NMT state under an enabled drive is not ours to do.
     - otherwise: NMT Enter PRE-OPERATIONAL (0x80) for this node, send the save
       again, wait while 0x1010,1 reads bit 2 (save pending, STM32 5.0.7), then
       NMT Start (0x01) to put it back in OPERATIONAL. The node was not in
       PRE-OP before (the first try would have passed) and an SDO answered, so
       it was OPERATIONAL: Start restores exactly that.

A save from a tool that already handles PRE-OP itself passes on the first try,
so the two never stack. install() is idempotent and called once at import of
p4core.session and of the pucktuner GUI; ``uninstall()`` is for tests.
"""

import logging
import time

try:
    from canopen.sdo.client import SdoClient
    from canopen.sdo.exceptions import SdoAbortedError
except ImportError:                     # pragma: no cover - canopen is a dependency
    SdoClient = None
    SdoAbortedError = Exception

log = logging.getLogger(__name__)

NV_INDICES = (0x1010, 0x1011)
ABORT_DEVICE_STATE = 0x08000022
NMT_ENTER_PREOP = 0x80
NMT_START = 0x01
SAVE_PENDING_BIT = 0x04                 # 0x1010,1 reads 5 while a save commits (STM32 5.0.7)
PENDING_TIMEOUT_S = 2.0                 # a Save.All commits in ~0.55 s at worst
PREOP_SETTLE_S = 0.02                   # NMT is unconfirmed: let the node act on it


class NmtSaveRefused(SdoAbortedError):
    """A save refused because the drive is Operation Enabled (outputs on)."""


def _operation_enabled(sdo):
    try:
        sw = int.from_bytes(sdo.upload(0x6041, 0), 'little')
    except Exception:
        return False
    return (sw & 0x6F) == 0x27


def _wait_pending(sdo):
    deadline = time.monotonic() + PENDING_TIMEOUT_S
    while time.monotonic() < deadline:
        try:
            if not int.from_bytes(sdo.upload(0x1010, 1), 'little') & SAVE_PENDING_BIT:
                return True
        except Exception:
            return False                # older firmware, or busy: nothing to wait on
        time.sleep(0.01)
    log.warning('NV save still pending after %.1f s', PENDING_TIMEOUT_S)
    return False


def _nmt(sdo, command):
    node_id = sdo.rx_cobid - 0x600
    sdo.network.send_message(0, bytes([command, node_id]))


def gated_download(orig, sdo, index, subindex, data, *args, **kwargs):
    """SdoClient.download with the 5.0.7 save rule (see the module docstring)."""
    if index not in NV_INDICES:
        return orig(sdo, index, subindex, data, *args, **kwargs)
    try:
        return orig(sdo, index, subindex, data, *args, **kwargs)
    except SdoAbortedError as exc:
        if getattr(exc, 'code', None) != ABORT_DEVICE_STATE:
            raise
        if _operation_enabled(sdo):
            raise NmtSaveRefused(ABORT_DEVICE_STATE) from exc
    node_id = sdo.rx_cobid - 0x600
    log.info('node %d: 0x%04X,%d refused outside PRE-OP; saving in PRE-OP',
             node_id, index, subindex)
    _nmt(sdo, NMT_ENTER_PREOP)
    time.sleep(PREOP_SETTLE_S)
    try:
        result = orig(sdo, index, subindex, data, *args, **kwargs)
        if index == 0x1010:
            _wait_pending(sdo)
        return result
    finally:
        _nmt(sdo, NMT_START)


_orig = None


def install():
    """Route every SdoClient.download through gated_download (once)."""
    global _orig
    if SdoClient is None or _orig is not None:
        return
    _orig = SdoClient.download

    def download(self, index, subindex, data, *args, **kwargs):
        return gated_download(_orig, self, index, subindex, data, *args, **kwargs)

    download.__doc__ = _orig.__doc__
    download._p4core_nvgate = True
    SdoClient.download = download


def uninstall():
    global _orig
    if _orig is not None:
        SdoClient.download = _orig
        _orig = None
