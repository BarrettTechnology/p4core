"""The drive's velocity loop, as the actuator calibration needs it.

The velocity PI runs on the 1 kHz timer tick, its output in per-mille of
RatedTorque (0x6076), clamped to the working peak's per-mille on rated-unit
firmware (+-1000 before).
"""


#: The velocity and position loops run on the 1 kHz timer tick.
VEL_LOOP_HZ = 1000.0

#: The velocity loop's one tuning knob, InstaSPIN's delta (SPRUHJ1 11.5):
#: the phase margin is atan(delta) - atan(1/delta) and the PI zero sits
#: delta^2 under the crossover.  Lower is stiffer, higher is softer.
#: InstaSPIN's own spacing (zero at wc / delta) overshoots a reference step
#: by 18 % at delta 4 with a third of the crossover (loopsim, three motors,
#: 2026-10-14); the squared spacing is the bench's: wc / 12 to / 37 stepped
#: with 2-10 %.  delta 4 is 62 deg and wc / 16: 7 % in loopsim, the design
#: the three-node bench tuned with (60 deg, wc / 15).
#:   delta  2.5   3    3.5   4    5    6
#:   PM     46   53    58   62   67   71   deg
#:   step   24   16    12    7    5    4   % overshoot (loopsim, P4-42)
VEL_DELTA = 4.0

#: Crossover ceiling, as a fraction of the loop rate: past ~1/10 the
#: discrete loop is no longer described by the continuous model.
VEL_MAX_FRACTION = 0.1

#: The velocity PI output limit, per-mille of RatedTorque, on firmware before
#: rated units (rated = peak).  Later firmware reaches the working peak's
#: per-mille: steptest.velocity_output_limit() reads it.
VEL_OUTPUT_LIMIT = 1000.0
