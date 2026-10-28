"""Encoder calibration: the runout table (0x3027).

It comes before the velocity and position loops are tuned: the runout is
read by the velocity loop as speed ripple at k times the shaft rate, and it
caps the gain the loop can take (a P4-16/MA782 at 3000 rpm read +-7 % once
per rev and a Kp above 0.002 chased it).

Runout
------
The rotor is dragged quasi-statically by a d-axis field (mode -1, the same
sweep as P4Drive.measure_winding, at 64 steps an electrical cycle), one
mechanical revolution forward and one back.  Where the field puts the rotor
is known from Theta_e; where the encoder says it is, is RawPosition.  The
difference, fitted as a Fourier series in the raw position with a separate
constant per direction (the friction lag), is the encoder's error.  Where
the rotor really sits is not quite where the field puts it: cogging and
the field's own harmonics pull it at multiples of the pole pairs, locked to
the rotor's true angle.  Those orders are fitted in the field's angle, not
the encoder's, in the same least squares: fitted against the raw angle, the
encoder's own error (11 ct at k = 2 on a P4-42) phase-modulates a cogging
line at n x pole_pairs into sidebands at n x pole_pairs +- k, and node 4's
42 = 6x7 cogging left a 1.4 ct k = 38 bin that the raw angle did not have
(0.12-0.22 ct at 150-2000 rpm; the bin put 1.4 ct into the corrected angle).
The firmware corrects

    pos += sum_k  -[A_s sin(k theta) + A_c cos(k theta)] / 256,
    theta = 2 pi raw / res

with A_s, A_c Q8.8 counts and k in 0x3027 (two bins on current firmware:
each costs ~1 us of the PWM ISR's case 2).  Which bins to keep, the rules
learned on the bench (puckutility calibrate_menu.generate_enc_correction_
table, where they came from):

  * k = pole_pairs - 1 is the sweep's own stator field reaching the sensor,
    not the encoder (grew 0.40 -> 1.6 ct with the drive current on node 127);
  * multiples of the pole pairs are electrical (cogging, the field) -- not
    encoder error; on one pole pair only k <= 2 (eccentricity, ellipticity);
  * above k = 42 the stepped sweep cannot pin the phase;
  * below 4x the spectrum's median (and 0.5 ct) is noise;
  * the table's real slope d(corr)/d(pos) must stay under 0.75, or the
    corrected angle goes non-monotonic and commutation runs away;
  * orders the caller has shown are not encoder error (*skip_k*: P4-42 HFM
    reads 5-6 ct at k = 12 on the sweep but <= 0.3 ct coasting, and a k = 12
    bin put 3-7 ct into the angle);
  * a strong cogging/field-locked harmonic (an exact multiple of the pole
    pairs) smears into its FFT neighbours -- within COGGING_LEAK_WINDOW of
    n x pole_pairs for n in COGGING_LEAK_HARMONICS is dropped too (a P4-42
    HFM, pp 7: the 42 = 6x7 cogging line put a 5.5 ct ghost at k = 40).
    k = 1, 2 (the MA7xx's own dominant terms) are never masked this way.
    n = 1, 2 are exactly what enctest.field_locked's phase-averaging
    already sets aside for the linearity step; 6 and 12 are cogging orders
    seen on more than one motor (stm32 docs/EncoderCoggingCompTiming.md).

The table is the error at rest.  An MA736 (filter setting 0x3013,6 = 0xA0)
tracks with a 150 Hz, zeta 0.88 second-order filter, and firmware applies
that response to the table at speed when 0x3027,8-9 hold it (fn Hz, zeta x
1000); other MA736 settings and the fixed MA702 take the datasheet's
fn = 1 / (2 pi tau), zeta 1, and an MA782 leaves the model off.

After the upload the sweep runs again on the compensated position: each
kept bin must have fallen to half (or under 0.5 ct).  A bin that grew is a
sign error -- compensation is switched off and the run fails.
"""

import struct

import numpy as np

from . import encfilter
from .constants import THETA_E_PER_ELEC_REV
from .device import SysidError

OD_COMP = 0x3027
OD_ENC_FILTER = (0x3013, 6)
OD_LATENCY_US = (0x3013, 5)
OD_LATENCY_PIPE = (0x3013, 10)

#: Sweep: steps per electrical cycle, dwell per step (s).
RUNOUT_STEPS = 64
RUNOUT_DWELL_S = 0.025
#: The sweep's I R as a fraction of U_MAX (commission.TEST_VOLTAGE_FRACTION).
SWEEP_VOLTAGE_FRACTION = 0.45
RETEST_DWELL_S = 0.015
#: The sweep must carry the rotor a revolution each way to within this
#: fraction, and the error must stay under this many electrical degrees, or
#: the rotor did not follow the field (a P4-42 at a template's 0.5 A i_cal
#: lagged 20-28 deg and the fit returned 9e10 ct).
SWEEP_TRAVEL_TOL = 0.1
SWEEP_LAG_MAX_DEG = 45.0

#: Bin selection (see the module docstring).
K_MAX = 42
NOISE_FACTOR = 4.0
MIN_AMPLITUDE_CT = 0.5
TOTAL_SLOPE_MAX = 0.75
#: Geared units: shaft content is not a rotor-encoder error; keep the old
#: per-harmonic cap and budget (from the geared k = 6 runaway).
GEARED_SLOPE_CAP, GEARED_SLOPE_BUDGET = 0.06, 0.12

#: Spectral leakage from a strong cogging/field-locked harmonic (module
#: docstring): drop a candidate within this many bins of n x pole_pairs,
#: for n in COGGING_LEAK_HARMONICS, unless it is one of the protected
#: low-order sensor terms.  n = 4 was considered and dropped: nothing in
#: the datasheets, the firmware docs or the bench notes supports it, on
#: this or any other motor reviewed -- flagged for a second look.
COGGING_LEAK_WINDOW = 2
COGGING_LEAK_HARMONICS = (1, 2, 6, 12)
COGGING_LEAK_PROTECT = (1, 2)

#: Retest: a kept bin must fall to this fraction of itself (or under
#: MIN_AMPLITUDE_CT).
RETEST_FRACTION = 0.5

#: Fitted MA736 tracking-filter model for 0x3027,8-9, by 0x3013,6 setting;
#: other settings and the MA702 come from the datasheet, and an MA782 has
#: none (encfilter.tracking_model).
FILTER_MODELS = {0xA0: (150, 880)}


class EncoderCalError(SysidError):
    pass


# ---------------------------------------------------------------------------
# Runout: pure analysis
# ---------------------------------------------------------------------------
def fit_runout(samples, res, pole_pairs, e_polarity, k_max=None):
    """The encoder's error from a sweep.

    *samples* are (direction, theta_e_total, position_unwrapped, raw_abs):
    the field angle accumulated in Theta_e counts, the encoder's position
    unwrapped (counts), and its raw reading 0..res-1 (where the firmware
    indexes the table).  The correction is where the field put the rotor
    minus where the encoder says, fitted as

        corr = c_dir + sum_k a_k cos(k phi) + b_k sin(k phi),  phi = 2 pi raw / res

    with phi the field's angle (2 pi expected / res) instead for k a
    multiple of the pole pairs (``field_locked``: the rotor's own motion,
    never uploaded).

    Returns ``k`` (1..K), ``a``, ``b``, ``amp`` (counts), ``dc`` (per
    direction), ``residual_ct`` (rms after the fit), ``error_rms_ct`` (rms of
    the correction less its constants), ``cpe`` (counts per electrical cycle).
    """
    d = np.array([s[0] for s in samples], float)
    th = np.array([s[1] for s in samples], float)
    pos = np.array([s[2] for s in samples], float)
    raw = np.array([s[3] for s in samples], float)
    cpe = res / float(pole_pairs)
    expected = e_polarity * th / THETA_E_PER_ELEC_REV * cpe
    corr = expected - pos
    # Orders up to one direction's Nyquist: the two directions sample the
    # same angles (offset by the lag), so their distinct readings together
    # are not twice the resolution.  A one-pole-pair sweep (64 points each
    # way) sized from both had as many columns as samples, and the direction
    # constants came out 700 ct off the data (node 3, 2026-10-26).
    distinct = len(np.unique(np.round(raw).astype(int)))
    per_dir = max(int(np.sum(d > 0)), int(np.sum(d < 0)))
    kk = min(k_max or 64, max(1, min(distinct, per_dir) // 2 - 1))
    phi = 2 * np.pi * raw / res
    # Multiples of the pole pairs are the rotor's (cogging, the field): in
    # the field's angle, so the encoder's error cannot modulate them into
    # sidebands (module docstring).  One pole pair: every k is a multiple,
    # and the rotor's and the encoder's terms cannot be told apart.
    psi = 2 * np.pi * expected / res
    pp = int(pole_pairs)
    field = np.array([pp >= 2 and k % pp == 0 for k in range(1, kk + 1)])
    cols = [(d > 0).astype(float), (d < 0).astype(float)]
    for k in range(1, kk + 1):
        ang = k * (psi if field[k - 1] else phi)
        cols += [np.cos(ang), np.sin(ang)]
    x = np.column_stack(cols)
    # Only the direction constants when a direction is missing.
    keep = [i for i in range(x.shape[1]) if np.any(x[:, i])]
    coef = np.zeros(x.shape[1])
    sol, *_ = np.linalg.lstsq(x[:, keep], corr, rcond=None)
    coef[keep] = sol
    fit = x @ coef
    a, b = coef[2::2], coef[3::2]
    dc_only = corr - x[:, :2] @ coef[:2]
    return {'k': np.arange(1, kk + 1), 'a': a, 'b': b, 'amp': np.hypot(a, b),
            'field_locked': field,
            'dc': (float(coef[0]), float(coef[1])),
            'residual_ct': float(np.sqrt(np.mean((corr - fit) ** 2))),
            'error_rms_ct': float(np.sqrt(np.mean(dc_only ** 2))),
            'error_peak_ct': float(np.max(np.abs(dc_only))),
            'cpe': cpe, 'n': int(len(samples))}


def table_slope(fit, ks, res):
    """The real max |d corr / d pos| of bins *ks* over a revolution."""
    if not ks:
        return 0.0
    p = np.arange(min(res, 2048)) * (res / min(res, 2048.0))
    phi = 2 * np.pi * p / res
    der = np.zeros_like(p, dtype=float)
    for k in ks:
        i = k - 1
        w = 2 * np.pi * k / res
        der += w * (-fit['a'][i] * np.sin(k * phi) + fit['b'][i] * np.cos(k * phi))
    return float(np.max(np.abs(der)))


def _near_cogging(k, pole_pairs):
    """Whether *k* sits within COGGING_LEAK_WINDOW of n x pole_pairs for
    some n in COGGING_LEAK_HARMONICS -- a strong harmonic's FFT leakage
    into a neighbour, not real encoder error (module docstring)."""
    return any(abs(k - pole_pairs * n) <= COGGING_LEAK_WINDOW
              for n in COGGING_LEAK_HARMONICS)


def select_bins(fit, pole_pairs, res, n_bins=2, skip_k=(), geared=False,
                k_max=K_MAX):
    """Which harmonics to upload, largest first, and why each other one
    was not.  Returns {'k': [...], 'dropped': {k: reason}, 'noise_ct',
    'threshold_ct', 'slope'}."""
    amp = fit['amp']
    ks = [int(k) for k in fit['k'][np.argsort(-amp)]]
    noise = float(np.median(amp)) if amp.size else 0.0
    thresh = max(NOISE_FACTOR * noise, MIN_AMPLITUDE_CT)
    dropped = {}
    cand = []
    pp = int(pole_pairs)
    skip = set(int(k) for k in skip_k)
    for k in ks:
        A = float(amp[k - 1])
        if A < thresh:
            continue                        # noise: not even reported
        if k > k_max:
            dropped[k] = 'k > {}: the sweep cannot pin its phase'.format(k_max)
        elif pp >= 2 and k == pp - 1:
            dropped[k] = 'k = pole pairs - 1: the sweep\'s stator field'
        elif k in skip:
            dropped[k] = 'skip_k: shown not to be encoder error'
        elif pp >= 2 and k % pp == 0:
            dropped[k] = 'multiple of the pole pairs: electrical'
        elif pp >= 2 and k not in COGGING_LEAK_PROTECT and _near_cogging(k, pp):
            dropped[k] = ('within {} of n x pole pairs (n in {}): cogging/'
                          'field-locked spectral leakage'.format(
                              COGGING_LEAK_WINDOW, COGGING_LEAK_HARMONICS))
        elif pp == 1 and k > 2:
            dropped[k] = 'one pole pair: k > 2 is mechanical'
        else:
            cand.append(k)
    kept = []
    for k in cand:
        if geared:
            w = 2 * np.pi * k / res
            if amp[k - 1] * w > GEARED_SLOPE_CAP:
                dropped[k] = 'slope over the geared per-harmonic cap'
                continue
            budget = sum(amp[j - 1] * 2 * np.pi * j / res for j in kept)
            if budget + amp[k - 1] * w > GEARED_SLOPE_BUDGET:
                dropped[k] = 'over the geared slope budget'
                continue
        elif table_slope(fit, kept + [k], res) > TOTAL_SLOPE_MAX:
            dropped[k] = 'table slope over {}'.format(TOTAL_SLOPE_MAX)
            continue
        if len(kept) >= n_bins:
            dropped[k] = 'past the {} bins the firmware has'.format(n_bins)
            continue
        kept.append(k)
    return {'k': kept, 'dropped': dropped, 'noise_ct': noise,
            'threshold_ct': thresh, 'slope': table_slope(fit, kept, res)}


def _i16(v):
    return max(-32768, min(32767, int(round(v))))


def bins_q88(fit, ks):
    """[(k, A_s, A_c)] in the firmware's Q8.8, sorted by k (the firmware
    rotates from one bin to the next)."""
    out = []
    for k in sorted(ks):
        a, b = fit['a'][k - 1], fit['b'][k - 1]
        out.append((int(k), _i16(-256.0 * b), _i16(-256.0 * a)))
    return out


# ---------------------------------------------------------------------------
# Runout: the drive
# ---------------------------------------------------------------------------
def _sdo_r(dev, idx, sub, fmt):
    d = bytes(dev.node.sdo.upload(idx, sub))
    n = struct.calcsize(fmt)
    return struct.unpack('<' + fmt, d[:n].ljust(n, b'\0'))[0]


def _sdo_w(dev, idx, sub, fmt, v):
    dev.node.sdo.download(idx, sub, struct.pack('<' + fmt, int(v)))


def comp_layout(dev):
    """(bins, (fn sub, zeta sub) or None) of this firmware's 0x3027; (0,
    None) when it has none."""
    try:
        count = _sdo_r(dev, OD_COMP, 0, 'B')
    except Exception:
        return 0, None
    bins = max(0, (count - 1) // 3)
    filt = (2 + 3 * bins, 3 + 3 * bins) if count >= 4 and (count - 1) % 3 == 2 else None
    return bins, filt


def comp_active(dev):
    return bool(_sdo_r(dev, OD_COMP, 1, 'H'))


def set_comp_active(dev, on):
    _sdo_w(dev, OD_COMP, 1, 'H', 1 if on else 0)


def write_bins(dev, q88, n_bins, filter_subs=None, filter_model=(0, 0)):
    """Write *q88* (bins_q88) into 0x3027, zero the unused bins, the
    filter model, and switch compensation on.  RAM only."""
    set_comp_active(dev, False)
    for i in range(n_bins):
        k, a_s, a_c = q88[i] if i < len(q88) else (0, 0, 0)
        _sdo_w(dev, OD_COMP, 2 + 3 * i, 'h', a_s)
        _sdo_w(dev, OD_COMP, 3 + 3 * i, 'H', k)
        _sdo_w(dev, OD_COMP, 4 + 3 * i, 'h', a_c)
    if filter_subs:
        _sdo_w(dev, OD_COMP, filter_subs[0], 'H', filter_model[0])
        _sdo_w(dev, OD_COMP, filter_subs[1], 'H', filter_model[1])
    set_comp_active(dev, True)
    back = []
    for i in range(len(q88)):
        back.append((_sdo_r(dev, OD_COMP, 3 + 3 * i, 'H'),
                     _sdo_r(dev, OD_COMP, 2 + 3 * i, 'h'),
                     _sdo_r(dev, OD_COMP, 4 + 3 * i, 'h')))
    if back != [tuple(b) for b in q88]:
        raise EncoderCalError('0x3027 read back {} for {}'.format(back, q88))


def _unwrap(value, ref, res):
    d = (value - ref) % res
    if d > res // 2:
        d -= res
    return ref + d


def sweep(dev, pole_pairs, steps=RUNOUT_STEPS, dwell_s=RUNOUT_DWELL_S,
          target_a=None, compensated=False):
    """Drag the rotor one revolution each way on a d-axis field; returns the
    samples fit_runout takes.  *compensated*: the position is the
    firmware's compensated one (EncPos), for the retest."""
    s = dev.refresh_scaling()
    res = s['enc_res']
    if target_a is None:
        target_a = min(s['i_cal_ma'], s['i_peak_ma']) / 1000.0
        # ...and what the winding takes at SWEEP_VOLTAGE_FRACTION of U_MAX:
        # an 11 ohm P4-16's i_cal (0.87 A) is past the ramp's ud cap.
        rs = dev._configured_rs_ohm()
        if rs and s.get('u_max_v'):
            target_a = min(target_a, SWEEP_VOLTAGE_FRACTION * s['u_max_v'] / rs)
    step = THETA_E_PER_ELEC_REV // steps
    lead = steps // 2
    n = int(pole_pairs) * steps
    out = []
    theta = 0

    def read():
        raw = int(dev.node.sdo['Encoder']['RawPosition'].raw) % res
        if not compensated:
            return raw, raw
        enc = int(dev.node.sdo['Encoder']['EncPos'].raw) % res
        return raw, enc

    try:
        dev._begin_d_axis(theta_e=0)
        dev._ramp_ud_to_current(target_a)
        dev._sleep(0.5)
        raw, enc = read()
        ref = enc
        for direction in (1, -1):
            for k in range(lead + n):
                dev._check_abort()
                theta += direction * step
                dev._set_theta_e(theta)
                dev._sleep(dwell_s)
                raw, enc = read()
                ref = _unwrap(enc, ref, res)
                if k >= lead:
                    out.append((direction, theta, ref, raw))
            # A fault mid-sweep (amplifier over-temperature) drops the field:
            # say so, not that the rotor did not follow it.
            dev._raise_if_faulted('the runout sweep')
    finally:
        dev.safe_stop()
    return out


def check_followed(samples, res, pole_pairs, target_a=None):
    """Raise EncoderCalError unless the sweep carried the rotor a revolution
    each way: a fit of a rotor that did not follow is meaningless."""
    at = ' at {:.2f} A'.format(target_a) if target_a else ''
    for direction in (1, -1):
        pos = [p for d, _, p, _ in samples if d == direction]
        if not pos:
            continue
        travel = abs(pos[-1] - pos[0]) / float(res)
        if abs(travel - 1.0) > SWEEP_TRAVEL_TOL:
            raise EncoderCalError(
                'the rotor did not follow the field{}: {:.2f} of a revolution '
                '{} -- raise i_cal (the winding probe) or check for a binding '
                'load'.format(at, travel, 'forward' if direction > 0 else 'back'))


def calibrate_runout(dev, skip_k=(), geared=False, steps=RUNOUT_STEPS,
                     dwell_s=RUNOUT_DWELL_S, target_a=None, retest=True,
                     log=None):
    """Measure the runout, upload the bins, retest.  RAM only: the caller
    saves (0x3027 subs 1 .. last, Save.All).  Returns the report dict:
    ``fit``, ``selection``, ``bins`` (Q8.8), ``filter_model``, ``retest``
    (the compensated sweep's fit and per-bin before/after), ``passed``."""
    log = log or dev._log
    s = dev.refresh_scaling()
    res, pp = s['enc_res'], int(dev.pole_pairs)
    pol = int(dev.node.sdo['Calibration']['e_polarity'].raw) or 1
    n_bins, filt = comp_layout(dev)
    if not n_bins:
        raise EncoderCalError('this firmware has no encoder compensation '
                              '(0x3027)')
    was = comp_active(dev)
    set_comp_active(dev, False)
    try:
        samples = sweep(dev, pp, steps, dwell_s, target_a)
        check_followed(samples, res, pp)
    except Exception:
        set_comp_active(dev, was)
        raise
    fit = fit_runout(samples, res, pp, pol)
    lag_deg = fit['error_peak_ct'] / fit['cpe'] * 360
    if lag_deg > SWEEP_LAG_MAX_DEG:
        set_comp_active(dev, was)
        raise EncoderCalError(
            'the sweep error peaks at {:.0f} deg electrical: the rotor lags '
            'the field, which is not runout -- raise i_cal (the winding '
            'probe) or check for a binding load'.format(lag_deg))
    sel = select_bins(fit, pp, res, n_bins, skip_k, geared)
    q88 = bins_q88(fit, sel['k'])
    log('encoder: error {:.2f} ct rms ({:.2f} peak, {:.1f} deg electrical); '
        'noise {:.2f} ct; bins {} (slope {:.3f}); dropped {}'.format(
            fit['error_rms_ct'], fit['error_peak_ct'],
            fit['error_peak_ct'] / fit['cpe'] * 360, sel['noise_ct'],
            ', '.join('k={} {:.2f} ct'.format(k, fit['amp'][k - 1])
                      for k in sel['k']) or 'none', sel['slope'],
            '; '.join('k={}: {}'.format(k, r) for k, r in sel['dropped'].items())
            or 'none'))
    try:
        hw = _sdo_r(dev, OD_ENC_FILTER[0], OD_ENC_FILTER[1], 'B')
    except Exception:
        hw = None
    model = encfilter.tracking_model(encfilter.encoder_of(dev.node), hw,
                                     FILTER_MODELS)
    report = {'fit': fit, 'selection': sel, 'bins': q88, 'filter_model': model,
              'encoder_filter': hw, 'n_bins_fw': n_bins, 'retest': None,
              'passed': True}
    if not q88:
        log('encoder: nothing above the noise to correct -- compensation off')
        set_comp_active(dev, False)
        return report
    write_bins(dev, q88, n_bins, filt, model)
    if not retest:
        return report
    after = fit_runout(sweep(dev, pp, steps, RETEST_DWELL_S, target_a,
                             compensated=True), res, pp, pol)
    per = []
    ok = True
    for k, _, _ in q88:
        b, a = float(fit['amp'][k - 1]), float(after['amp'][k - 1]) \
            if k <= len(after['amp']) else 0.0
        good = a <= max(RETEST_FRACTION * b, MIN_AMPLITUDE_CT)
        ok &= good
        per.append({'k': k, 'before_ct': b, 'after_ct': a, 'ok': good})
    report['retest'] = {'fit': after, 'bins': per}
    log('encoder: retest {}: {}; error {:.2f} -> {:.2f} ct rms'.format(
        'PASS' if ok else 'FAIL',
        ', '.join('k={} {:.2f} -> {:.2f} ct'.format(p['k'], p['before_ct'],
                                                    p['after_ct']) for p in per),
        fit['error_rms_ct'], after['error_rms_ct']))
    if not ok:
        set_comp_active(dev, False)
        log('encoder: a bin did not fall -- compensation switched off')
    report['passed'] = ok
    return report


# ---------------------------------------------------------------------------
# Latency
# ---------------------------------------------------------------------------
def read_latency_us(dev):
    """(pipeline part, us or None; trim 0x3013,5, us)."""
    trim = _sdo_r(dev, OD_LATENCY_US[0], OD_LATENCY_US[1], 'h')
    try:
        pipe = _sdo_r(dev, OD_LATENCY_PIPE[0], OD_LATENCY_PIPE[1], 'h') / 10.0
    except Exception:
        pipe = None
    return pipe, trim
