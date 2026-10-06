"""calibrate.py — current-sense and encoder calibration, UI-free.

p4gui's configure_tab routines (bias, gain, encoder zero, encoder
stability) without wx: each takes a canopen RemoteNode and a Reporter, stores
what it measured with Save.Single, and returns a result dict.  A result
outside its acceptance band goes to ``reporter.confirm`` — the GUI shows a
dialog, the command line asks on the terminal or answers the default (no)
when it cannot.

Order matters (calibrate_all): bias needs nothing; gain subtracts the
biases; the encoder zero's ramp watches Motor/id, which is downstream of
both, and needs Calibration.poles.  Each step is followed by deenergise(), so
no step starts with current left flowing by the one before.
"""

import math
import time

from . import ezero
from .flash import read_version
from .reporter import Reporter


MODE_IDLE = 0
MODE_PHASE_VOLTAGE_ANGLE = 12

GAIN_FIT_STEPS = 48         # one electrical revolution
GAIN_NOMINAL = 4096
GAIN_TOLERANCE = 0.10
BIAS_MIDPOINT = 2048        # ADC counts
BIAS_TOLERANCE = 0.03
ENCODER_MAX_JITTER = 8      # counts over a second at rest

#: ud step and ceiling for the calibration current ramp, and settle per step.
RAMP_UD_STEP = 100
RAMP_UD_MAX = 32000
RAMP_SETTLE_S = 0.05
#: Per-mille of i_peak at which the exported id is taken as saturated.
RAMP_ID_SATURATED = 1000


class CalibrationError(RuntimeError):
    """A calibration step failed, or the user declined to continue."""


def save_single(node, index, sub):
    node.sdo['Save']['Single'].raw = (index << 8) | sub


def firmware_version(node):
    """0x100A as a (major, minor, patch) tuple, string or packed U32 (see read_version)."""
    return tuple(int(x) for x in read_version(node).split('.')[:3])


def bias_scale(node):
    """Bias counts per ADC count: 16 (Q12.4) on firmware 4.4.0 and later, else 1."""
    return 16 if firmware_version(node) >= (4, 4, 0) else 1


def gain_fit(samples):
    """Fundamental amplitudes (|A|, |B|) of the two current-sense channels.

    samples: (theta, alpha, beta) over whole electrical revolutions at equal
    steps, theta in radians. Alpha ~ bias + A cos(theta), beta ~ bias + B sin(theta);
    the bias, and every harmonic but the first, sum to zero over a revolution."""
    n = float(len(samples))
    ac = sum(a * math.cos(t) for t, a, _ in samples)
    as_ = sum(a * math.sin(t) for t, a, _ in samples)
    bc = sum(b * math.cos(t) for t, _, b in samples)
    bs = sum(b * math.sin(t) for t, _, b in samples)
    return math.hypot(ac, as_) * 2 / n, math.hypot(bc, bs) * 2 / n


def read_values(node):
    """The stored calibration, for display: bias in ADC counts."""
    scale = bias_scale(node)
    return dict(
        alpha_bias=node.sdo['Alpha']['Bias'].raw / scale,
        beta_bias=node.sdo['Beta']['Bias'].raw / scale,
        alpha_gain=node.sdo['Alpha']['Gainfactor'].raw,
        beta_gain=node.sdo['Beta']['Gainfactor'].raw,
        e_zero=node.sdo['Calibration']['e_zero'].raw,
        e_polarity=node.sdo['Calibration']['e_polarity'].raw,
        i_cal=node.sdo['Calibration']['i_cal'].raw,
    )


def _enable(sdo):
    """Clear faults, ready to switch on, operation enabled."""
    sdo['ControlWord'].raw = 0x80
    sdo['ControlWord'].raw = 0x06
    sdo['ControlWord'].raw = 0x0F


def _read_filtered(node, channel, samples=16, sleep=time.sleep):
    """Average of fresh reads of the Q12.4 filtered ADC value."""
    total = 0
    for _ in range(samples):
        total += node.sdo[channel]['Filtered'].raw
        sleep(0.005)
    return total / float(samples)


def calibration_current(node):
    """i_cal, capped at i_peak (both mA)."""
    sdo = node.sdo
    return min(sdo['Calibration']['i_cal'].raw, sdo['Calibration']['i_peak'].raw)


def ramp_to_calibration_current(node, reporter=None):
    """Raise ud until the d-axis current reaches i_cal.  Returns that ud.

    Apply, then measure: measured first, a back-to-back run (calibrate_all)
    sees the previous step's current still decaying, exits at ud = 0 and
    calibrates against a rotor that never moved.
    """
    reporter = reporter or Reporter()
    sdo = node.sdo
    target = calibration_current(node)
    i_peak = sdo['Calibration']['i_peak'].raw
    ud = 0
    while ud < RAMP_UD_MAX:
        ud += RAMP_UD_STEP
        sdo['Motor']['ud'].raw = ud
        reporter.sleep(RAMP_SETTLE_S)
        i_d = sdo['Motor']['id'].raw
        if i_d >= RAMP_ID_SATURATED or i_d / 1000.0 * i_peak >= target:
            break
    return ud


def deenergise(node, settle_s=1.0, sleep=time.sleep):
    """Drop ud/uq to zero, go IDLE, and let Motor/id follow it down.

    The exported current is low-passed at a cutoff from rt/lt; a second
    covers every puck.  Never raises: it is what runs on the way out.
    """
    for write in (lambda: setattr(node.sdo['Motor']['ud'], 'raw', 0),
                  lambda: setattr(node.sdo['Motor']['uq'], 'raw', 0),
                  lambda: setattr(node.sdo['SetModeOfOperation'], 'raw',
                                  MODE_IDLE)):
        try:
            write()
        except Exception:
            pass
    sleep(settle_s)


def _continue_or_raise(reporter, message, default=False):
    if not reporter.confirm(message + '\n\nContinue anyway?', default):
        raise CalibrationError(message)


# ---------------------------------------------------------------------------
# Steps
# ---------------------------------------------------------------------------
def test_encoder(node, reporter=None):
    """Encoder stability at rest: position spread over one second."""
    reporter = reporter or Reporter()
    reporter.status('Testing encoder...')
    node.sdo['SetModeOfOperation'].raw = MODE_IDLE
    reporter.sleep(1)
    end = time.monotonic() + 1
    positions = []
    while time.monotonic() < end:
        reporter.check_abort()
        positions.append(node.sdo['PositionFeedback'].raw)
    spread = max(positions) - min(positions) if positions else 0
    result = dict(spread=spread, limit=ENCODER_MAX_JITTER, ok=spread <= ENCODER_MAX_JITTER)
    reporter.note('Encoder spread at rest: {} counts (limit {})'.format(
        spread, ENCODER_MAX_JITTER))
    if not result['ok']:
        _continue_or_raise(reporter, 'Encoder readings unstable: {} counts '
                           '(limit {}).'.format(spread, ENCODER_MAX_JITTER))
    return result


def calibrate_bias(node, reporter=None):
    """Current-sense bias (Alpha/Beta Bias, 0x3008/0x3009,3), zero current."""
    reporter = reporter or Reporter()
    reporter.status('Calibrating current-sense bias...')
    sdo = node.sdo
    try:
        _enable(sdo)
        sdo['Theta_e'].raw = 0x7FFF
        sdo['Motor']['ud'].raw = 0
        sdo['SetModeOfOperation'].raw = MODE_PHASE_VOLTAGE_ANGLE
        reporter.sleep(1)
        # Filtered is Q12.4.  Firmware 4.4.0 and later keeps Bias in Q12.4
        # too, so it is stored as read; older firmware wants whole counts.
        scale = bias_scale(node)
        for channel in ('Alpha', 'Beta'):
            filt = _read_filtered(node, channel, sleep=reporter.sleep)
            sdo[channel]['Bias'].raw = int(round(filt * scale / 16.0))
        save_single(node, 0x3008, 3)
        save_single(node, 0x3009, 3)
        a_bias = sdo['Alpha']['Bias'].raw / scale
        b_bias = sdo['Beta']['Bias'].raw / scale
    finally:
        try:
            sdo['SetModeOfOperation'].raw = MODE_IDLE
        except Exception:
            pass
    lo = BIAS_MIDPOINT * (1 - BIAS_TOLERANCE)
    hi = BIAS_MIDPOINT * (1 + BIAS_TOLERANCE)
    ok = lo <= a_bias <= hi and lo <= b_bias <= hi
    reporter.note('Bias: alpha {:.1f}, beta {:.1f} counts'.format(a_bias, b_bias))
    if not ok:
        _continue_or_raise(reporter, 'Current-sense bias out of bounds: alpha '
                           '{:.1f}, beta {:.1f} counts (acceptable {:.0f} - {:.0f}).'
                           .format(a_bias, b_bias, lo, hi))
    return dict(alpha_bias=a_bias, beta_bias=b_bias, ok=ok)


def calibrate_gain(node, reporter=None):
    """Beta current-sense gain factor (0x3009,6).

    Drives the calibration current on the d axis in phase-voltage-angle mode
    and rotates it through one electrical revolution in GAIN_FIT_STEPS steps,
    then takes the ratio of the two channels' fundamentals (gain_fit()).  The
    dead-time error is six-fold over a revolution, so it has no fundamental
    and the fit ignores it, along with bias drift and the 2x-electrical term.
    """
    reporter = reporter or Reporter()
    reporter.status('Calibrating current-sense gain...')
    sdo = node.sdo
    orig = (sdo['Alpha']['Gainfactor'].raw, sdo['Beta']['Gainfactor'].raw)

    def restore():
        sdo['Alpha']['Gainfactor'].raw, sdo['Beta']['Gainfactor'].raw = orig

    sdo['Alpha']['Gainfactor'].raw = GAIN_NOMINAL
    sdo['Beta']['Gainfactor'].raw = GAIN_NOMINAL
    target = calibration_current(node)
    i_peak = sdo['Calibration']['i_peak'].raw
    samples, ids = [], []
    try:
        _enable(sdo)
        sdo['SetModeOfOperation'].raw = MODE_PHASE_VOLTAGE_ANGLE
        sdo['Motor']['uq'].raw = 0
        sdo['Theta_e'].raw = 0
        reporter.sleep(1)
        ramp_to_calibration_current(node, reporter)
        reporter.sleep(0.5)
        for k in range(GAIN_FIT_STEPS):
            au = k * 65536 // GAIN_FIT_STEPS
            sdo['Theta_e'].raw = au if au < 32768 else au - 65536
            reporter.sleep(0.12)
            samples.append((2.0 * math.pi * k / GAIN_FIT_STEPS,
                            sdo['Alpha']['Filtered'].raw, sdo['Beta']['Filtered'].raw))
            ids.append(sdo['Motor']['id'].raw / 1000.0 * i_peak)
            reporter.progress(100 * (k + 1) // GAIN_FIT_STEPS)
    except BaseException:
        restore()
        raise
    finally:
        try:
            sdo['Motor']['ud'].raw = 0
        finally:
            sdo['SetModeOfOperation'].raw = MODE_IDLE

    a_amp, b_amp = gain_fit(samples)
    id_med = sorted(ids)[len(ids) // 2]
    # Filtered is Q12.4: 30 ADC counts of fundamental is the least worth a ratio.
    if a_amp < 30 * 16 or b_amp < 30 * 16 or id_med < 0.3 * target:
        restore()
        raise CalibrationError(
            'no measurable rotating current (|A| {:.0f} |B| {:.0f} counts, '
            'id {:.0f} mA); nothing stored'.format(a_amp / 16, b_amp / 16, id_med))
    gainfactor = int(round(GAIN_NOMINAL * a_amp / b_amp))
    lo = round(GAIN_NOMINAL * (1 - GAIN_TOLERANCE))
    hi = round(GAIN_NOMINAL * (1 + GAIN_TOLERANCE))
    ok = lo <= gainfactor <= hi
    # Bounds before the write, so a rejected value never reaches the OD.
    if not ok and not reporter.confirm(
            'Beta gain factor {} is out of bounds ({} - {}). Store it anyway?'
            .format(gainfactor, lo, hi)):
        restore()
        raise CalibrationError('Beta gain factor {} out of bounds; not stored'
                               .format(gainfactor))
    sdo['Beta']['Gainfactor'].raw = gainfactor
    save_single(node, 0x3008, 6)
    save_single(node, 0x3009, 6)
    reporter.note('Beta gain factor {} (id {:.0f} mA)'.format(gainfactor, id_med))
    return dict(beta_gain=gainfactor, id_ma=id_med, ok=ok)


def calibrate_ezero(node, reporter=None):
    """Electrical zero and polarity (ezero.sweep); stores both.

    The rotor turns one mechanical revolution each way.
    """
    reporter = reporter or Reporter()
    reporter.status('Calibrating encoder zero...')
    try:
        r = ezero.sweep(node, progress=lambda f: reporter.progress(100 * f),
                        sleep=reporter.sleep)
    except ezero.EZeroError as exc:
        raise CalibrationError('encoder zero failed, nothing stored: {}'
                               .format(exc)) from exc
    sdo = node.sdo
    previous = sdo['Calibration']['e_zero'].raw
    sdo['Calibration']['e_polarity'].raw = r['e_polarity']
    sdo['Calibration']['e_zero'].raw = r['e_zero']
    save_single(node, 0x3011, 2)    # e_polarity
    save_single(node, 0x3011, 1)    # e_zero
    r['previous'] = previous
    r['shift_deg'] = ezero.wrap(r['e_zero'] - previous, r['cts_per_elec_cycle']) \
        * 360.0 / r['cts_per_elec_cycle']
    reporter.note('Encoder zero {} (was {}, {:+.1f} deg elec), polarity {:+d}, '
                  'spread {:.1f} deg rms'.format(
                      r['e_zero'], previous, r['shift_deg'], r['e_polarity'],
                      r['spread_rms_deg']))
    return r


STEPS = (
    ('encoder-test', test_encoder),
    ('bias', calibrate_bias),
    ('gain', calibrate_gain),
    ('ezero', calibrate_ezero),
)


def calibrate_all(node, reporter=None, steps=None, settle_s=1.0):
    """Run *steps* (names from STEPS, default all) in dependency order.

    Returns {name: result}.  Raises CalibrationError at the first step that
    fails or that the user declines to continue past; the drive is left
    de-energised either way.
    """
    reporter = reporter or Reporter()
    wanted = set(steps) if steps else None
    results = {}
    for name, step in STEPS:
        if wanted is not None and name not in wanted:
            continue
        try:
            results[name] = step(node, reporter)
        finally:
            deenergise(node, settle_s, sleep=reporter.sleep
                       if not reporter.aborted else time.sleep)
    return results
