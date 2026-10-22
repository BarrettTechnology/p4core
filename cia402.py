"""cia402.py — CiA-402 drive-profile constants the apps and bench scripts
share (they were copied out of canopen_runner.py).

ControlWord (0x6040) commands and ModesOfOperation (0x6060) values.  The
drive refuses a 0x6060 write outside Operation Enabled (abort 0x08000022):
enable first, then pick the mode.
"""

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

# Error codes (0x603F / 0x1003 / EMCY) the firmware raises, by 0x1002 bit
# (stm32 app/faults.c codes[] and faultHandle()).
ERROR_CODES = {
    0x6320: 'current sense outside its calibration while regulating',
    0x7321: 'encoder magnet distance',
    0x2310: 'current limit active (i2t)',
    0x3210: 'bus over-voltage',
    0x3220: 'bus under-voltage',
    0x4210: 'amplifier over-temperature',
    0x8110: 'CAN overrun: the drive fell behind the bus (CPU load -- '
            'PWM/control rate too high for it)',
    0x8180: 'SYNC timeout',
    0x8140: 'CAN bus-off',
    0x8130: 'heartbeat consumer timeout',
    0x8411: 'velocity tracking warning',
    0x8414: 'velocity tracking fault',
    0x8418: 'velocity limit active',
    0x8611: 'position tracking warning',
    0x8614: 'position tracking fault',
    0x2311: 'magnet demag (reverse id held too long)',
    0x2312: 'positive id control fault',
    0x2313: 'hard over-current trip',
    0x2314: 'bus over-voltage fast trip',
}

#: The codes that point at the supply rather than the drive.
SUPPLY_ERROR_CODES = frozenset((0x3210, 0x3220, 0x2314))


def describe_error(code):
    """'0x8110 CAN overrun: ...' -- the code and what it means, if known."""
    text = ERROR_CODES.get(code)
    return '{:#06x}{}'.format(code, ' ' + text if text else '')
