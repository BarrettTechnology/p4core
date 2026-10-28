"""The drive's voltage budget: what a PWM rate and the current-sense window
leave of the theoretical voltage.  Pure arithmetic, no CANopen.
"""


def kgm2_to_gcm2(j):
    """kg·m² → g·cm².  1 kg·m² = 1e3 g × 1e4 cm² = 1e7 g·cm²."""
    return j * 1e7


#: K64 PDB-to-ADC aperture latency plus the guard hal_pwm_set_rate keeps, in
#: timer ticks (40 MHz): the one-sided budget's fixed overhead (k64/hal.c
#: ISENSE_LAT_CLKS 4 + 2).
ONE_SIDED_GUARD_TICKS = 6
K64_TIMER_HZ = 40e6


def pwm_voltage_headroom(f_sw_hz, dead_time_ns, prop_delay_ns, settling_ns,
                         sampling_ns, one_sided=False, shift=True):
    """Fraction of the theoretical alpha-beta voltage still reachable.

    Every PWM period has to leave the ADC a window clear of any switching edge,
    wide enough for the gate dead time, the gate driver's propagation delay and
    the shunt's settling.  hal_pwm_set_rate reserves it by shrinking the
    maximum pulse.  The STM32 (centre-aligned, sample centred on the valley)
    reserves it on both sides:

        delay      = dead_time + prop_delay + settling
        max_width  = modulo - 2*delay          (when sampling < delay)
                   = modulo - delay - sampling (otherwise)
        v_limit    = (2*max_width - modulo) / modulo

    which for the first case is ``1 - 4*delay*f_sw``.  **This is the cost of
    switching fast, and it is linear.**  On a P4-42 (delay = 930 ns) it is 70 %
    at 80 kHz against 85 % at 40 kHz — and what is lost comes directly off the
    speed the motor can reach before back-EMF meets the available voltage.

    *one_sided* is the K64 budget: the sample follows the widest falling edge
    past the period boundary when it has to, so only the settling after that
    edge and the sample itself are reserved:

        max_width  = modulo - delay - sampling - ONE_SIDED_GUARD_TICKS

    floored at half the period.  For the same timing this roughly halves what
    the window costs (P4-16 at 40 kHz: 0.77 against 0.62).  Then the K64
    shifts the pulses next to the sample (k64/hal.c shift_width): a pulse may
    grow until the unsampled zero vector keeps only 2 dead times + 2 ticks,

        shift_width = (modulo + max_width - 2 dead_time - 2 ticks) / 2

    and the limit is (2 shift_width - modulo) / modulo -- half the window's
    cost again (P4-16 at 80 kHz: 0.54 -> 0.70 on the bench).  *shift* False
    gives the unshifted budget, the one the drive reports at 0x3016,4/5.

    Returns a fraction in [0, 1]; 0 means the reserved window has eaten the
    whole period.
    """
    if f_sw_hz <= 0:
        raise ValueError('PWM frequency must be positive')
    modulo_ns = 1e9 / f_sw_hz
    delay_ns = dead_time_ns + prop_delay_ns + settling_ns
    if one_sided:
        tick_ns = 1e9 / K64_TIMER_HZ
        max_width_ns = max(0.5 * modulo_ns,
                           modulo_ns - delay_ns - sampling_ns
                           - ONE_SIDED_GUARD_TICKS * tick_ns)
        free_gap_ns = 2.0 * dead_time_ns + 2.0 * tick_ns
        if shift:
            max_width_ns = max(max_width_ns,
                               0.5 * (modulo_ns + max_width_ns - free_gap_ns))
    elif sampling_ns < delay_ns:
        max_width_ns = modulo_ns - 2.0 * delay_ns
    else:
        max_width_ns = modulo_ns - delay_ns - sampling_ns
    limit = (2.0 * max_width_ns - modulo_ns) / modulo_ns
    return max(0.0, min(1.0, limit))


def drive_voltage_limit(max_width_ticks, period_ticks, free_gap_ticks=None):
    """The voltage limit the drive itself computed, from 0x3016,4 and ,5.

    Both chips publish their maximum pulse and period in the same units
    (STM32: half-pulse against ARR, K64: pulse against the modulo), so this is
    (2*max_width - period) / period on either.  A K64 shifts its pulses past
    that (pwm_voltage_headroom): pass *free_gap_ticks* (2 dead times + 2) and
    the limit is the shifted width's, as k64/hal.c computes it.
    """
    if period_ticks <= 0:
        raise ValueError('period must be positive')
    width = float(max_width_ticks)
    if free_gap_ticks is not None:
        width = max(width, 0.5 * (period_ticks + width - free_gap_ticks))
    return max(0.0, min(1.0, (2.0 * width - period_ticks)
                        / float(period_ticks)))


def budget_is_one_sided(max_width_ticks, period_ticks, f_sw_hz, dead_time_ns,
                        prop_delay_ns, settling_ns, sampling_ns):
    """Which budget the drive is running, judged from what it reports.

    The host cannot ask which chip it is talking to, but the two budgets give
    different maximum pulses for the same timing.  Returns True when the
    reported limit is nearer the one-sided prediction.
    """
    got = drive_voltage_limit(max_width_ticks, period_ticks)
    args = (f_sw_hz, dead_time_ns, prop_delay_ns, settling_ns, sampling_ns)
    return (abs(got - pwm_voltage_headroom(*args, one_sided=True,
                                          shift=False))
            < abs(got - pwm_voltage_headroom(*args)))
