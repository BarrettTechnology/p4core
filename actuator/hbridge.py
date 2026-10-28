"""The inverter's dead-time knee, as the actuator calibration needs it.

A HOLD's voltage is not R * i near zero current: every half-bridge loses a
dead-time volt-second each PWM period, signed with its phase current, and
the PWM ripple smears that sign out over a knee of width i0.  A d-axis bias
of a few i0 puts every phase past the knee.
"""


#: A d-axis bias this many i0 keeps the two minor phases (each carrying half of
#: it) at tanh(3) = 0.995 of the saturated drop, i.e. past the knee, so a small
#: step or sine on top of it sees the linear winding and not the bridge.
KNEE_BIAS_PER_I0 = 6.0


def knee_bias_a(i0_a):
    """Smallest d-axis bias that keeps every phase past the knee."""
    return KNEE_BIAS_PER_I0 * i0_a
