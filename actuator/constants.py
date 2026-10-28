"""Firmware facts the actuator calibration agrees on.

One copy, so the calibrations and the drive layer cannot drift apart on a
number the firmware decides.
"""

#: Case slots per control cycle (pwm.h PWM_PATTERNS_PER_CONTROL_CYCLE).  A slot
#: is K PWM periods (0x3001,13), so the current loop, the injection primitive
#: and one capture record per decimate step all run at pwm_hz / (4 K).  The
#: drive reports that rate at 0x3001,14; control_rate_hz() is the fallback.
PWM_PATTERNS_PER_CONTROL_CYCLE = 4

#: Theta_e / 0x3015,4 counts per electrical revolution.
THETA_E_PER_ELEC_REV = 65536
