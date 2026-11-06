"""cia402.py — CiA-402 drive-profile constants the apps and bench scripts
share (they were copied out of canopen_runner.py).

ControlWord (0x6040) commands and ModesOfOperation (0x6060) values.  The
drive refuses a 0x6060 write outside Operation Enabled (abort 0x08000022):
enable first, then pick the mode.
"""
import struct
import time


# ControlWord commands
CLEAR_FAULT = 0x80   # Fault Reset (rising edge of bit 7)
SHUTDOWN = 0x06      # Shutdown -> Ready to Switch On
OP_ENABLED = 0x0F    # Switch On + Enable Operation (bits 0-3)

# ModesOfOperation
MODE_IDLE = 0
MODE_PROFILE_POS = 1
MODE_VELOCITY = 2
MODE_PROFILE_VEL = 3
MODE_PROFILE_TRQ = 4
MODE_RESERVED = 5
MODE_HOMING = 6
MODE_INTERPOLATED_POS = 7
MODE_CYCLIC_SYNC_POS = 8
MODE_CYCLIC_SYNC_VEL = 9
MODE_CYCLIC_SYNC_TRQ = 10
MODE_CYCLIC_SYNC_TRQ_ANGLE = 11
# Manufacturer-specific modes: 0x6060 is Integer8 and CiA 402 gives them -1..-128
# (stm32 cf8d11e, "DS402 stage 3").  Older firmware numbered them 12 and 14, in the
# range CiA 402 reserves; drive_mode() picks the numbering the drive takes.
MODE_PHASE_VOLTAGE_ANGLE = -1
MODE_CYCLIC_SYNC_IMP = -2
LEGACY_MODES = {MODE_PHASE_VOLTAGE_ANGLE: 12, MODE_CYCLIC_SYNC_IMP: 14}

SUPPORTED_MODES = (0x6502, 0)
#: 0x6502 bit for manufacturer mode -k is 15 + k: -1 (phase voltage) is bit 16.
NEGATIVE_MODES_BIT = 1 << 16


def mode_bit(mode):
    """The 0x6502 bit (as a mask) that advertises *mode*."""
    return 1 << (15 - mode if mode < 0 else mode - 1)


def supported_modes(node):
    """0x6502 as an int, cached on the node object; 0 when it cannot be read
    (a test fake, a flashloader)."""
    cached = getattr(node, '_p4_supported_modes', None)
    if cached is not None:
        return cached
    try:
        v = int.from_bytes(bytes(node.sdo.upload(*SUPPORTED_MODES))[:4], 'little')
    except Exception:   # SdoAbortedError, a fake without upload or the row
        v = 0
    try:
        node._p4_supported_modes = v
    except AttributeError:
        pass
    return v


def negative_modes(node):
    """True when the drive numbers its manufacturer modes negative (0x6502
    bit 16), False for older firmware (12 / 14)."""
    return bool(supported_modes(node) & NEGATIVE_MODES_BIT)


def drive_mode(node, mode):
    """The 0x6060 value for *mode* on this drive: the MODE_* constants are the
    current numbering, and a manufacturer mode maps to its old number (12 /
    14) on firmware that predates it.  Standard modes pass through."""
    if mode in LEGACY_MODES and not negative_modes(node):
        return LEGACY_MODES[mode]
    return mode


def canonical_mode(value):
    """A 0x6060 / 0x6061 value read from any firmware, as a MODE_* constant:
    a raw unsigned byte (255) reads as its signed value (-1), and the old 12 /
    14 as -1 / -2."""
    value = int(value)
    if value > 127:
        value -= 256
    for new, old in LEGACY_MODES.items():
        if value == old:
            return new
    return value


def mode_byte(value):
    """*value* (a drive_mode() result) as the one unsigned byte 0x6060 carries,
    for raw packing ('B', bytes([...])); '<b' packs the signed value as is."""
    return int(value) & 0xFF


def write_mode(node, mode):
    """Write 0x6060 by name with drive_mode(node, mode).  An application EDS
    from before the INTEGER8 change types it UNSIGNED8, which cannot encode
    -1; the same byte as 255 is what the drive reads as -1."""
    value = drive_mode(node, mode)
    try:
        node.sdo['SetModeOfOperation'].raw = value
    except (struct.error, OverflowError, ValueError):
        node.sdo['SetModeOfOperation'].raw = mode_byte(value)
    return value

# Error codes (0x603F / 0x1003 / EMCY) the firmware raises, by 0x1002 bit
# (stm32 app/faults.c codes[] and faultHandle()).
ERROR_CODES = {
    0x6320: 'current sense outside its calibration while regulating',
    0x7385: 'encoder magnet distance',
    0x2310: 'current limit active (i2t)',
    0x3210: 'bus over-voltage',
    0x3220: 'bus under-voltage',
    0x4210: 'amplifier over-temperature',
    0x8110: 'CAN overrun: the drive fell behind the bus (CPU load -- '
            'PWM/control rate too high for it)',
    0x8700: 'SYNC timeout',
    0x8180: 'SYNC timeout (firmware before 5.0.5)',
    0x8140: 'CAN bus-off',
    0x8130: 'heartbeat consumer timeout',
    0x8482: 'velocity tracking fault',
    0x8611: 'position tracking fault (a warning before 5.0.4)',
    0x2390: 'magnet demag (reverse id held too long)',
    0x2391: 'positive id control fault',
    0x2392: 'hard over-current trip',
    0x3280: 'bus over-voltage fast trip',
    # Older firmware (before 5.0.4)
    0x8411: 'velocity tracking warning',
    0x8414: 'velocity tracking fault',
    0x8418: 'velocity limit active',
    0x8614: 'position tracking fault',
    0x7321: 'encoder magnet distance',
    0x2311: 'magnet demag (reverse id held too long)',
    0x2312: 'positive id control fault',
    0x2313: 'hard over-current trip',
    0x2314: 'bus over-voltage fast trip',
}

#: The codes that point at the supply rather than the drive.
SUPPLY_ERROR_CODES = frozenset((0x3210, 0x3220, 0x3280, 0x2314))


def describe_error(code):
    """'0x8110 CAN overrun: ...' -- the code and what it means, if known."""
    text = ERROR_CODES.get(code)
    return '{:#06x}{}'.format(code, ' ' + text if text else '')


# ---------------------------------------------------------------------------
# Enable, with the reason when the drive refuses
# ---------------------------------------------------------------------------
#: StatusWord (0x6041) bits.
SW_FAULT = 0x0008
SW_STATE_MASK = 0x006F
SW_OPERATION_ENABLED = 0x0027
#: How long the fault reset gets to leave FAULT, s (the state machine runs
#: on the 1 kHz tick).
FAULT_RESET_S = 0.05
#: 0x1001 error register bits (CiA 301 7.5.2.2).
ERROR_REGISTER_BITS = ((0x01, 'generic'), (0x02, 'current'),
                       (0x04, 'voltage'), (0x08, 'temperature'),
                       (0x10, 'communication'), (0x20, 'device profile'),
                       (0x80, 'manufacturer'))
_STATES = ((0x4F, 0x00, 'Not Ready to Switch On'),
           (0x4F, 0x40, 'Switch On Disabled'),
           (0x6F, 0x21, 'Ready to Switch On'),
           (0x6F, 0x23, 'Switched On'),
           (0x6F, 0x27, 'Operation Enabled'),
           (0x6F, 0x07, 'Quick Stop Active'),
           (0x4F, 0x0F, 'Fault Reaction Active'),
           (0x4F, 0x08, 'Fault'))


class EnableRefused(RuntimeError):
    """The drive would not reach Operation Enabled; the message says why
    (diagnose())."""


def state_name(sw):
    """The CiA 402 state a StatusWord value reports."""
    for mask, value, name in _STATES:
        if sw & mask == value:
            return name
    return 'unknown state'


def _upload_int(node, index, sub, size):
    try:
        data = bytes(node.sdo.upload(index, sub))
    except Exception:       # noqa: BLE001 -- a diagnosis reads what it can
        return None
    return int.from_bytes(data[:size].ljust(size, b'\0'), 'little')


def statusword(node):
    """0x6041, or None when it cannot be read."""
    return _upload_int(node, 0x6041, 0, 2)


def diagnose(node):
    """What the drive says about its state: the StatusWord and its state,
    the last error (0x603F) by name, the error register (0x1001) and, when
    it differs, the newest 0x1003 entry.  For an error message; never
    raises."""
    parts = []
    sw = statusword(node)
    if sw is not None:
        parts.append('StatusWord {:#06x} ({})'.format(sw, state_name(sw)))
    code = _upload_int(node, 0x603F, 0, 2)
    if code:
        parts.append('error ' + describe_error(code))
    reg = _upload_int(node, 0x1001, 0, 1)
    if reg:
        parts.append('error register {:#04x} ({})'.format(
            reg, ', '.join(n for b, n in ERROR_REGISTER_BITS if reg & b)))
    hist = _upload_int(node, 0x1003, 1, 4)
    if hist and hist & 0xFFFF != code:
        parts.append('newest in 0x1003 ' + describe_error(hist & 0xFFFF))
    return '; '.join(parts) or 'the drive did not answer'


def _refused(node, what, why, exc=None):
    return EnableRefused('{} refused ({}): {}{}'.format(
        what, why, diagnose(node),
        '' if exc is None else ' [{}]'.format(exc)))


def enable(node, what='enable', sleep=time.sleep):
    """Fault reset, Shutdown, Enable Operation over SDO, each refusal
    explained: a fault that the reset cannot clear (its cause is still
    there; the firmware acks 0x80 and stays in FAULT, and the Shutdown
    after it aborts 0x08000022) or one that trips on enable raises
    EnableRefused with diagnose()'s reading, not a bare SDO abort."""
    cw = node.sdo['ControlWord']
    try:
        cw.raw = CLEAR_FAULT
    except Exception as exc:     # noqa: BLE001 -- explained, then raised
        raise _refused(node, what, 'the fault reset', exc) from exc
    sw = statusword(node)
    end = time.monotonic() + FAULT_RESET_S
    while sw is not None and sw & SW_FAULT and time.monotonic() < end:
        sleep(0.005)
        sw = statusword(node)
    if sw is not None and sw & SW_FAULT:
        raise _refused(node, what, 'the fault persists after a reset: '
                       'clear its cause first')
    for word, step in ((SHUTDOWN, 'Shutdown'),
                       (OP_ENABLED, 'Enable Operation')):
        try:
            cw.raw = word
        except Exception as exc:     # noqa: BLE001 -- explained, then raised
            raise _refused(node, what, step, exc) from exc
    sw = statusword(node)
    if sw is not None and sw & SW_FAULT:
        raise _refused(node, what, 'faulted on enable')
    return sw
