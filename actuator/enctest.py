"""enctest.py -- encoder checks, UI-free.

  test_encoder            stability at rest (p4core.calibrate.test_encoder)
  test_encoder_linearity  a voltage-angle sweep over one mechanical revolution:
                          encoder travel against the commanded field angle

Each takes a canopen RemoteNode and a p4core Reporter and returns a result
dict; the GUI draws its plots from that dict.
"""


from p4core.calibrate import test_encoder                        # noqa: F401
from p4core.cia402 import (CLEAR_FAULT, SHUTDOWN, OP_ENABLED, MODE_IDLE,
                           MODE_PHASE_VOLTAGE_ANGLE, write_mode)


from . import drivecal
from p4core.calibrate import permille_base_ma

#: Linearity sweep: steps per electrical cycle (7.5 deg) and dwell per step.
LIN_STEPS_PER_CYCLE = 48
LIN_STEP_S = 0.08
#: Engineering pass/fail limit, electrical degrees.
LIN_PASS_DEG = 5.0
#: The sweep's travel must be one revolution to this fraction, or the pole
#: count (not the encoder) is wrong.
LIN_TRAVEL_TOL = 0.1
#: Past LIN_PASS_DEG but under this, the error is runout the Runout step's
#: table corrects (an MA736's 2x/rev read 6.6 deg electrical on a P4-42);
#: past it, commutation loses more than cos(30 deg) of its torque.
LIN_GROSS_DEG = 30.0
#: test_encoder_linearity's position: the sensor, or the sensor through the
#: runout table (the firmware's display copy; the Runout step re-checks it).
LIN_POSITION_SUBS = {'raw': 'RawPosition', 'corrected': 'EncPos'}


# ---------------------------------------------------------------------------
# Linearity
# ---------------------------------------------------------------------------
def _ramp_to_current(node, cal_current, pm_base, reporter):
    """Raise ud until Motor/id reaches *cal_current* (mA) or ud hits 32000."""
    motor_ud = 0
    while (node.sdo['Motor']['id'].raw / 1000.0 * pm_base) < cal_current \
            and motor_ud < 32000:
        reporter.check_abort()
        id_now = node.sdo['Motor']['id'].raw / 1000.0 * pm_base
        if motor_ud > 0 and id_now > 0:
            step = max(100, int((motor_ud * cal_current / id_now - motor_ud) / 4))
        else:
            step = max(100, 32000 // 12)
        motor_ud = min(motor_ud + step, 32000)
        node.sdo['Motor']['ud'].raw = motor_ud
        reporter.sleep(0.05)
    return motor_ud


def field_locked(results, steps_per_cycle):
    """The part of each error that repeats every electrical cycle: the mean
    over the cycles at the same field angle.  The friction lag (a constant)
    and cogging (a P4-42's 6 per cycle read 9.6 deg) are there, and so is any
    encoder error at a multiple of the pole pairs, which a field drag cannot
    tell from them (the Runout step drops those orders too)."""
    sums, counts = {}, {}
    keys = []
    for mech, elec, cyc, err in results:
        k = int(round(elec / 360.0 * steps_per_cycle)) % steps_per_cycle
        keys.append(k)
        sums[k] = sums.get(k, 0.0) + err
        counts[k] = counts.get(k, 0) + 1
    return [sums[k] / counts[k] for k in keys]


def encoder_split(errors, pole_pairs):
    """Split *errors*, one revolution uniformly sampled (no repeat of the
    first point), into (encoder, ripple) by mechanical harmonic.

    An encoder or its magnet can only be wrong slower than once per pole
    pair: its error is a function of the shaft angle, and the field-locked
    part (orders that are multiples of the pole pairs) has already come
    out.  Orders 1..pole_pairs-1 are the encoder's.  Everything faster is
    the rotor itself not sitting where the field puts it -- slot-locked lag,
    which shrinks with current, the cogging's sidebands, sample jitter: on
    node 2 (P4-37, 14 pole pairs, direct drive) single-sample spikes of
    9-13 deg against 2-3 deg of low-order error.  The mean goes with
    neither: an offset is not nonlinearity."""
    import numpy as np
    e = np.asarray(errors, dtype=float)
    spec = np.fft.rfft(e)
    enc = np.zeros_like(spec)
    enc[1:pole_pairs] = spec[1:pole_pairs]
    encoder = np.fft.irfft(enc, len(e))
    ripple = e - e.mean() - encoder
    return [float(x) for x in encoder], [float(x) for x in ripple]


def linearity_stats(results, threshold=LIN_PASS_DEG, steps_per_cycle=None):
    """max_err (signed), max_idx, rms_err and passed over *results*
    [(mech_deg, elec_deg, cycle, error_deg)].  With *steps_per_cycle*, the
    field-locked part (field_locked) is taken out first and reported as
    locked_rms; and over two or more pole pairs swept a whole revolution,
    only the encoder's orders are judged (encoder_split) and the rest is
    reported as ripple_rms / ripple_max, not judged."""
    errors = [r[3] for r in results]
    locked_rms = ripple_rms = ripple_max = None
    if steps_per_cycle and len(results) > steps_per_cycle:
        # The first sample is the rotor at rest, before the field moved it:
        # no friction lag yet, so not on the field-locked curve -- judged
        # as zero (it is where the error is measured from).
        if len(results) - 1 >= 2 * steps_per_cycle:
            locked = field_locked(results[1:], steps_per_cycle)
        else:
            # One cycle (one pole pair): each field angle is seen once, so
            # the field-locked mean IS the error and would cancel it all.
            # Only the friction lag, a constant, can be taken out.
            lag = sum(errors[1:]) / (len(errors) - 1)
            locked = [lag] * (len(errors) - 1)
        errors = [0.0] + [e - l for e, l in zip(errors[1:], locked)]
        locked_rms = (sum(l * l for l in locked) / len(locked)) ** 0.5
        n = len(results) - 1
        pole_pairs = n // steps_per_cycle
        if pole_pairs >= 2 and n == pole_pairs * steps_per_cycle:
            # Samples 1..n: one revolution, the n-th back at the start.
            enc, rip = encoder_split(errors[1:], pole_pairs)
            errors = [enc[-1]] + enc
            ripple_rms = (sum(r * r for r in rip) / len(rip)) ** 0.5
            ripple_max = max(rip, key=abs)
    max_idx = max(range(len(errors)), key=lambda i: abs(errors[i]))
    max_err = errors[max_idx]
    rms_err = (sum(e * e for e in errors) / len(errors)) ** 0.5
    return dict(max_err=max_err, max_idx=max_idx, rms_err=rms_err,
                locked_rms=locked_rms, ripple_rms=ripple_rms,
                ripple_max=ripple_max, passed=abs(max_err) <= threshold,
                gross=abs(max_err) > LIN_GROSS_DEG)


def _fault_code(node):
    try:
        return 'error {:#06x}'.format(node.sdo['LastError'].raw)
    except Exception:
        return 'error code unreadable'


def test_encoder_linearity(node, reporter=None, steps_per_cycle=LIN_STEPS_PER_CYCLE,
                           step_s=LIN_STEP_S, threshold=LIN_PASS_DEG,
                           position='raw'):
    """Drag the rotor one mechanical revolution with the field (voltage-angle
    mode at the calibration current) and compare the encoder's travel with
    the commanded electrical angle.

    The error is against the encoder's own direction of travel (*direction*,
    +1 with the field), so a template's e_polarity does not fail it, and
    *poles_ok* is False when the travel is not one revolution
    (*travel_ratio*): a wrong pole count, not encoder error.

    Returns dict(results=[(mech_deg, elec_deg, cycle, error_deg)],
    pole_pairs, cts_per_elec, e_zero, e_polarity, direction, travel_ratio,
    poles_ok, steps_per_cycle, threshold, position,
    max_err, max_idx, rms_err, locked_rms, ripple_rms, ripple_max, passed,
    gross).  The error judged is the encoder's: what is left after the
    field-locked part (field_locked), slower than once per pole pair
    (encoder_split); the faster rotor ripple is reported, not judged.  Writes nothing persistent; leaves the
    drive in IDLE.  Raises on an SDO error (de-energised).

    *position* 'raw' reads the sensor (0x3012,1); 'corrected' reads it
    with the runout table applied (0x3012,2, same counts, may sit a few
    counts outside 0..Resolution near the wrap)."""
    from p4core import calibrate
    sub = LIN_POSITION_SUBS[position]
    reporter = drivecal.reporter_or_default(reporter)
    reporter.status('Encoder linearity sweep...')
    # The ramp below raises ud until the sense reads the calibration
    # current: an erased gain reads 0 whatever flows, and ud runs to the top.
    ok, problems = calibrate.isense_window(node)
    if not ok:
        raise drivecal.CalError('the current sense is not calibrated ({}): '
                                'run Bias and Gain first'
                                .format('; '.join(problems)))
    try:
        e_zero = node.sdo['Calibration']['e_zero'].raw
        e_polarity = int(node.sdo['Calibration']['e_polarity'].raw)
        enc_resolution = node.sdo['EncoderConfig']['Resolution'].raw
        motor_poles = node.sdo['Calibration']['poles'].raw
        cts_per_elec = enc_resolution * 2.0 / motor_poles
        pole_pairs = motor_poles // 2
        cal_current = node.sdo['Calibration']['i_cal'].raw
        i_peak = node.sdo['Calibration']['i_peak'].raw
        if cal_current > i_peak:
            cal_current = i_peak
        if cal_current <= 0:
            raise drivecal.CalError(
                'the calibration current is not set (i_cal 0 mA): run the '
                'winding probe first')

        reporter.note('Encoder linearity sweep — {} pole pairs  {:.1f} '
                      'cts/elec_cyc  e_zero={}  e_polarity={}'.format(
                          pole_pairs, cts_per_elec, e_zero, e_polarity))

        # PHASE_VOLTAGE_ANGLE, ramp to the calibration current at theta_e = 0.
        node.sdo['ControlWord'].raw = CLEAR_FAULT
        node.sdo['ControlWord'].raw = SHUTDOWN
        node.sdo['ControlWord'].raw = OP_ENABLED
        write_mode(node, MODE_PHASE_VOLTAGE_ANGLE)
        node.sdo['Theta_e'].raw = 0
        reporter.sleep(0.3)
        _ramp_to_current(node, cal_current, permille_base_ma(node), reporter)
        reporter.sleep(0.3)

        # steps_per_cycle steps per electrical cycle x pole_pairs = 1 mech rev.
        n_total = steps_per_cycle * pole_pairs
        enc_prev = node.sdo['Encoder'][sub].raw
        enc_accumulated = 0
        travel = []
        reporter.note('Sweeping {} steps ({} per elec cycle x {} cycles) — '
                      '~{:.0f} s ...'.format(n_total, steps_per_cycle,
                                             pole_pairs, n_total * step_s))
        for step in range(n_total + 1):
            frac_in_cycle = (step % steps_per_cycle) / steps_per_cycle
            total_elec_cycles = step / steps_per_cycle
            theta_e_u = round(frac_in_cycle * 65536) % 65536
            node.sdo['Theta_e'].raw = (theta_e_u if theta_e_u < 32768
                                       else theta_e_u - 65536)
            reporter.sleep(step_s)
            if n_total:
                reporter.progress(100.0 * step / n_total)

            enc = node.sdo['Encoder'][sub].raw
            delta = enc - enc_prev
            if delta > enc_resolution / 2:
                delta -= enc_resolution
            if delta < -enc_resolution / 2:
                delta += enc_resolution
            enc_accumulated += delta
            enc_prev = enc

            travel.append((total_elec_cycles, frac_in_cycle,
                           step // steps_per_cycle + 1, enc_accumulated))
        # A drive that faulted mid-sweep (an amplifier over-temperature
        # after a few back-to-back sweeps at i_cal) drops the field; the
        # rotor stops and the rest of the sweep reads as encoder error.
        sw = node.sdo['StatusWord'].raw
        if sw & 0x08:
            raise drivecal.CalError(
                'the drive faulted during the linearity sweep ({}): {}'.format(
                    _fault_code(node), drivecal.fault_message(node, None)))
    except BaseException:
        drivecal.deenergise(node)
        raise
    node.sdo['SetModeOfOperation'].raw = MODE_IDLE

    # The encoder should have moved direction * cycles * cts_per_elec.  The
    # direction is the travel's own: on a new motor e_polarity is still a
    # template's guess (the Winding step measures it), and a wrong guess is
    # not encoder error.  A travel that is not one revolution means the
    # pole count is wrong, which is not encoder error either.
    total = travel[-1][3] if travel else 0
    expected_total = n_total / steps_per_cycle * cts_per_elec
    direction = 1 if total >= 0 else -1
    ratio = abs(total) / expected_total if expected_total else 0.0
    results = [(cyc / pole_pairs * 360.0, frac * 360.0, n,
                (acc - direction * cyc * cts_per_elec) / cts_per_elec * 360.0)
               for cyc, frac, n, acc in travel]
    out = dict(results=results, pole_pairs=pole_pairs,
               cts_per_elec=cts_per_elec, e_zero=e_zero,
               e_polarity=e_polarity, direction=direction,
               travel_ratio=ratio, steps_per_cycle=steps_per_cycle,
               threshold=threshold, position=position)
    out.update(linearity_stats(results, threshold, steps_per_cycle))
    out['poles_ok'] = abs(ratio - 1.0) <= LIN_TRAVEL_TOL
    if direction != e_polarity:
        reporter.note('Encoder runs {} the field: e_polarity {:+d} is not '
                      'this motor\'s (the Winding step sets it)'.format(
                          'with' if direction > 0 else 'against', e_polarity))
    if not out['poles_ok']:
        out['passed'] = False
        reporter.note('Encoder travel {:.2f} of a revolution for {} pole '
                      'pairs: the pole count is wrong, not the encoder'
                      .format(ratio, pole_pairs))
    worst = results[out['max_idx']]
    reporter.note('Encoder linearity: max {:.2f}° elec at mech={:.1f}° '
                  'elec={:.1f}° (cycle {}), RMS {:.2f}°, {} (±{:.1f}°); '
                  'field-locked lag/cogging {:.2f}° rms set aside{}'.format(
                      out['max_err'], worst[0], worst[1], worst[2],
                      out['rms_err'], 'PASS' if out['passed'] else 'FAIL',
                      threshold, out['locked_rms'] or 0.0,
                      '; rotor ripple (faster than once per pole pair, not '
                      'judged) {:.2f}° rms, {:.2f}° max'.format(
                          out['ripple_rms'], out['ripple_max'])
                      if out['ripple_rms'] is not None else ''))
    return out
