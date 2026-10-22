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
MODE_PHASE_VOLTAGE_ANGLE = 12

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
