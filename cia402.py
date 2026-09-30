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
