"""winding.py — electrical zero, winding order and pole count from one sweep.

The pure half of the e_zero sweep that p4gui's and puckutility's ezero.py
run over SDO (P4Drive.measure_winding is the device half).

A current held on the d axis in phase-voltage-angle mode pulls the rotor to
the field.  Stepping the commanded angle through every electrical cycle of one
mechanical revolution, forward and then back, gives at each settled step

    e_zero = raw - e_polarity * theta   (mod one electrical cycle, in counts)

and three things fall out of the whole sweep:

  * e_polarity, the sign of encoder travel against the field: the winding
    order relative to the encoder.  Two phases swapped flips it; nothing
    else can tell the difference, and nothing else needs to.
  * the pole count checked: one revolution of field per pole pair must move
    the encoder one revolution, both ways.
  * e_zero as the circular mean over the whole revolution both ways, which
    averages the once-per-revolution encoder error and cogging out and
    cancels friction (on a P4-42 single points scatter +-20 deg electrical;
    the mean lands within ~1 deg of the back-EMF zero).
"""

import math

import numpy as np

#: Theta_e units per electrical cycle.
F16_CYCLE = 65536


def unwrap(p, ref, res):
    """Encoder reading *p* moved to within half a revolution of *ref*."""
    while p - ref > res / 2.0:
        p -= res
    while p - ref < -res / 2.0:
        p += res
    return p


def circ_mean(vals, period):
    s = sum(math.sin(2 * math.pi * v / period) for v in vals)
    c = sum(math.cos(2 * math.pi * v / period) for v in vals)
    return (math.atan2(s, c) * period / (2 * math.pi)) % period


def wrap(v, period):
    """*v* folded into [-period/2, period/2)."""
    return (v + period / 2.0) % period - period / 2.0


def solve(samples, res, pole_pairs, ratio_tolerance=0.05, exact_angle=False):
    """e_zero, e_polarity and the travel check from sweep samples.

    *samples* are ``(direction, theta, raw)`` per settled step: direction
    +1/-1, theta the commanded angle unwrapped in F16 units, raw the encoder
    unwrapped in counts.  Raises ValueError if the rotor did not follow one
    revolution per *pole_pairs* electrical cycles within *ratio_tolerance*
    (stalled, or the pole count is wrong), or the two directions disagree.
    *exact_angle* is whether the drive computes the angle exactly for any
    pole count (0x3013,8 = 1), which changes the best integer e_zero
    (firmware_e_zero).
    """
    cpe = res / float(pole_pairs)
    polarity = None
    ratios = {}
    for direction in (1, -1):
        leg = [(t, r) for d, t, r in samples if d == direction]
        name = 'forward' if direction > 0 else 'reverse'
        if len(leg) < 2:
            raise ValueError('too few samples in the {} sweep'.format(name))
        mt = sum(t for t, _ in leg) / float(len(leg))
        mr = sum(r for _, r in leg) / float(len(leg))
        stt = sum((t - mt) ** 2 for t, _ in leg)
        ratio = sum((t - mt) * (r - mr) for t, r in leg) / stt * F16_CYCLE / cpe
        ratios[direction] = ratio
        if not 1.0 - ratio_tolerance < abs(ratio) < 1.0 + ratio_tolerance:
            raise ValueError(
                'the rotor moved {:.2f}x the field in the {} sweep: stalled '
                '(friction above the sweep torque), or the pole count is '
                'wrong -- this travel suggests {:.1f} poles, not {}'.format(
                    abs(ratio), name,
                    2 * pole_pairs / max(abs(ratio), 1e-6), 2 * pole_pairs))
        sign = 1 if ratio > 0 else -1
        if polarity is not None and sign != polarity:
            raise ValueError('the forward and reverse sweeps disagree on the '
                             'electrical polarity')
        polarity = sign

    est = [(d, (r - polarity * t / float(F16_CYCLE) * cpe) % cpe)
           for d, t, r in samples]
    fwd = [e for d, e in est if d > 0]
    rev = [e for d, e in est if d < 0]
    e_zero = circ_mean(fwd + rev, cpe)
    dev = [wrap(e - e_zero, cpe) * 360.0 / cpe for _, e in est]
    fw = firmware_e_zero(e_zero, polarity, res, pole_pairs, exact_angle)
    return dict(
        e_zero=fw['e_zero'],
        e_zero_physical=int(round(e_zero)) % int(round(cpe)),
        e_zero_exact=e_zero,
        commutation_error_deg=fw['error_peak_deg'],
        e_zero_error_deg=fw['mean_error_deg'],
        exact_angle=bool(exact_angle),
        e_polarity=polarity,
        cts_per_elec_cycle=cpe,
        travel_ratio=(abs(ratios[1]) + abs(ratios[-1])) / 2.0,
        friction_split_deg=wrap(circ_mean(fwd, cpe) - circ_mean(rev, cpe),
                                cpe) * 360.0 / cpe,
        spread_rms_deg=math.sqrt(sum(x * x for x in dev) / len(dev)),
        spread_max_deg=max(abs(x) for x in dev),
        samples=len(samples),
    )


def firmware_e_zero(e_zero_exact, polarity, res, pole_pairs,
                    exact_angle=False):
    """The integer e_zero that best centres the drive's frame on the rotor's.

    *e_zero_exact* is solve's physical zero (counts, fractional).  Returns
    ``e_zero``, ``mean_error_deg`` (what is left) and ``error_peak_deg`` (the
    angle's sawtooth over a revolution around that mean).

    Firmware with *exact_angle* (0x3013,8 = 1) takes the electrical position
    as pole_pairs * (raw - e_zero) mod res, so e_zero one electrical cycle
    (res/pole_pairs counts, not whole) further on is the same frame less a
    fraction of a count: of the integers across the revolution one lands
    within 1/(2 pole_pairs) count of the zero, when the pole pairs and the
    resolution share no factor.  No sawtooth.

    Older firmware takes it modulo a cycle truncated to res // pole_pairs
    counts.  Its angle is evaluated at every encoder count against the true
    one, and the e_zero whose mean error over the revolution is least is
    picked; the sawtooth is 0 only when the pole pairs divide the resolution.
    """
    res = int(res)
    pp = int(pole_pairs)
    if exact_angle:
        ez = np.arange(res)
        # The drive's angle less the true one, in electrical cycles.
        err = polarity * pp * (e_zero_exact - ez) / float(res)
        err = (err + 0.5) % 1.0 - 0.5
        best = int(np.argmin(np.abs(err)))
        return {'e_zero': best, 'mean_error_deg': float(err[best]) * 360.0,
                'error_peak_deg': 0.0}
    cpe = res / float(pp)
    cpe_i = res // pp
    raw = np.arange(res)
    true = polarity * (raw - e_zero_exact) / cpe

    def error(ez):
        tm = polarity * (raw - ez)
        fw = ((res + tm) % cpe_i) / float(cpe_i)
        d = (fw - true + 0.5) % 1.0 - 0.5
        m = math.atan2(np.mean(np.sin(2 * np.pi * d)),
                       np.mean(np.cos(2 * np.pi * d))) / (2 * math.pi)
        return m, d

    base = int(round(e_zero_exact))
    best = None
    for ez in range(base - cpe_i // 4, base + cpe_i // 4 + 1):
        m, d = error(ez)
        if best is None or abs(m) < abs(best[1]):
            best = (ez, m, d)
    ez, m, d = best
    spread = (d - m + 0.5) % 1.0 - 0.5
    return {'e_zero': ez % cpe_i,
            'mean_error_deg': m * 360.0,
            'error_peak_deg': float(np.max(np.abs(spread))) * 360.0}
