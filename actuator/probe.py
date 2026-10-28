"""The calibration current's bounds, shared with the winding probe.

The board calibrations hold the drive's i_cal (0x3011,11).  Without a
winding probe to size it, it is capped at CAL_I_MAX_A; SENSE_FLOOR_COUNTS
is the least signal the gain fit wants.
"""


CAL_I_MAX_A = 3.0
#: The sense floor: the gain fit needs 30 ADC counts of fundamental
#: (p4core calibrate_gain); i_cal gives a healthy margin over that, not just
#: twice it -- a P4-16's thermal i_cal was 121-446 mA, ~24 counts, too thin a
#: signal for Gain to fit well.  A motor that is thermally limited below the
#: floor gets the floor, if that warms it by no more than CAL_RISE_MAX_C over
#: the window; the steps that hold the raised current (Gain, Slope) shorten
#: their dwell by dwell_scale (below) to keep the same I^2Rt.
SENSE_FLOOR_COUNTS = 150.0
