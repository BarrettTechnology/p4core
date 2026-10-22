"""Electrical-zero (e_zero) calibration: stepped field sweep over one revolution.

Shared by every app through p4core.calibrate (p4gui's copy, with the ramp fix:
step, then measure).

The older methods aligned the rotor at ONE electrical position (theta = 0 from
both sides, or a +-22.5 deg spin-through around it) and took e_zero from there.
Friction cancels between the two directions, but the encoder's nonlinearity and
cogging at that one spot do not: on a P4-42 single points scatter by +-20 deg
electrical about the true zero, and a calibration from one of them left e_zero
21 deg off (confirmed by the back-EMF of a zero-current coast,
tools/bench/enc_latency_coast.py). Both errors are periodic over a revolution.

So this holds the calibration current on the d axis in phase-voltage-angle mode
and steps the commanded angle through every electrical cycle of one mechanical
revolution, forward then back. Each settled point gives

    e_zero = raw - e_polarity * theta   (mod one electrical cycle, in counts)

and the circular mean over the whole revolution, both ways, averages the
periodic errors out and cancels friction. On the P4-42 it lands within ~1 deg
of the back-EMF zero at 1 A or 2 A.

The same sweep gives e_polarity (the sign of raw travel against theta) and
checks the pole count: one revolution of theta must move the encoder one
revolution. The rotor turns one full revolution each way.
"""
import math
import time

MODE_IDLE = 0
MODE_PHASE_VOLTAGE_ANGLE = 12
F16_CYCLE = 65536          # Theta_e units per electrical cycle


class EZeroError(RuntimeError):
    """The sweep did not produce a trustworthy e_zero."""


def unwrap(p, ref, res):
    """Encoder reading p moved to within half a revolution of ref."""
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
    """v folded into [-period/2, period/2)."""
    return (v + period / 2.0) % period - period / 2.0


def solve(samples, res, pole_pairs):
    """e_zero from sweep samples.

    samples: (direction, theta, raw) per settled step, direction +1/-1, theta the
    commanded angle unwrapped in F16 units, raw the encoder unwrapped in counts.
    Raises EZeroError if the rotor did not follow one revolution per revolution
    of theta (stalled, or the pole count is wrong)."""
    cpe = res / float(pole_pairs)
    polarity = None
    for direction in (1, -1):
        leg = [(t, r) for d, t, r in samples if d == direction]
        if len(leg) < 2:
            raise EZeroError('too few samples in the {} sweep'.format('forward' if direction > 0 else 'reverse'))
        # Travel per electrical cycle from a least-squares slope, so a single late settle
        # at either end doesn't move it. One revolution of field is one revolution of rotor.
        mt = sum(t for t, _ in leg) / float(len(leg))
        mr = sum(r for _, r in leg) / float(len(leg))
        stt = sum((t - mt) ** 2 for t, _ in leg)
        ratio = sum((t - mt) * (r - mr) for t, r in leg) / stt * F16_CYCLE / cpe
        if not 0.95 < abs(ratio) < 1.05:
            raise EZeroError(
                'the rotor moved {:.2f}x the field in the {} sweep. Stalled (friction above the '
                'calibration torque), or the pole count is wrong: this travel suggests {:.1f} '
                'poles, not {}.'.format(abs(ratio), 'forward' if direction > 0 else 'reverse',
                                        2 * pole_pairs / max(abs(ratio), 1e-6), 2 * pole_pairs))
        sign = 1 if ratio > 0 else -1
        if polarity is not None and sign != polarity:
            raise EZeroError('the forward and reverse sweeps disagree on the electrical polarity')
        polarity = sign

    est = [(d, (r - polarity * t / float(F16_CYCLE) * cpe) % cpe) for d, t, r in samples]
    fwd = [e for d, e in est if d > 0]
    rev = [e for d, e in est if d < 0]
    e_zero = circ_mean(fwd + rev, cpe)
    dev = [wrap(e - e_zero, cpe) * 360.0 / cpe for _, e in est]
    return dict(
        e_zero=int(round(e_zero)) % int(round(cpe)),
        e_zero_exact=e_zero,
        e_polarity=polarity,
        cts_per_elec_cycle=cpe,
        friction_split_deg=wrap(circ_mean(fwd, cpe) - circ_mean(rev, cpe), cpe) * 360.0 / cpe,
        spread_rms_deg=math.sqrt(sum(x * x for x in dev) / len(dev)),
        spread_max_deg=max(abs(x) for x in dev),
        samples=len(samples),
    )


def sweep(node, steps=16, dwell=0.1, current=None, progress=None, sleep=time.sleep):
    """Run the sweep on a canopen RemoteNode and return solve()'s result.

    Writes nothing to the calibration: the caller stores e_zero / e_polarity.
    Always leaves the drive with ud = 0 in IDLE. steps is per electrical cycle;
    progress(fraction) is called as it goes; sleep lets a GUI stay responsive."""
    sdo = node.sdo
    res = sdo['EncoderConfig']['Resolution'].raw
    poles = sdo['Calibration']['poles'].raw
    if poles < 2 or poles % 2:
        raise EZeroError('Calibration poles (0x3011,3) is {}: load the configuration first'.format(poles))
    pole_pairs = poles // 2
    i_peak = sdo['Calibration']['i_peak'].raw
    if current is None:
        current = sdo['Calibration']['i_cal'].raw
    current = min(current, i_peak)
    if current <= 0 or i_peak <= 0:
        raise EZeroError('i_cal / i_peak (0x3011,11 / 9) not set: load the configuration first')
    from .calibrate import id_scale     # calibrate imports this module
    base_ma, saturated = id_scale(node)

    step = F16_CYCLE // steps
    lead = steps // 2          # half an electrical cycle first each way, so friction has turned
    n = pole_pairs * steps     # one mechanical revolution
    total = 2 * (lead + n)
    samples = []
    try:
        sdo['ControlWord'].raw = 0x80
        sdo['ControlWord'].raw = 0x06
        sdo['ControlWord'].raw = 0x0F
        sdo['SetModeOfOperation'].raw = MODE_PHASE_VOLTAGE_ANGLE
        sdo['Motor']['uq'].raw = 0      # uq == 0 keeps the firmware on the host's angle
        theta = 0
        sdo['Theta_e'].raw = theta
        # Step, then measure: a reading taken before the first step can still
        # carry the previous calibration's current and end the ramp at ud = 0.
        ud = 0
        while ud < 32000:
            ud = min(ud + 100, 32000)
            sdo['Motor']['ud'].raw = ud
            sleep(0.02)
            i_d = sdo['Motor']['id'].raw
            if i_d >= saturated or i_d / 1000.0 * base_ma >= current:
                break
        sleep(0.5)
        ref = sdo['Encoder']['RawPosition'].raw
        done = 0
        for direction in (1, -1):
            for k in range(lead + n):
                theta += direction * step
                sdo['Theta_e'].raw = (theta + 32768) % F16_CYCLE - 32768
                sleep(dwell)
                ref = unwrap(sdo['Encoder']['RawPosition'].raw, ref, res)
                if k >= lead:
                    samples.append((direction, theta, ref))
                done += 1
                if progress:
                    progress(done / float(total))
    finally:
        try:
            sdo['Motor']['ud'].raw = 0
        finally:
            sdo['SetModeOfOperation'].raw = MODE_IDLE
    return solve(samples, res, pole_pairs)
