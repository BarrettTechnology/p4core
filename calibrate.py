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

puckutility's calibrate menu runs the same steps with its own settings:
BIAS_MENU / BIAS_MENU_QUICK, GAIN_MENU and EZERO_QUICK_STEPS (keyword
arguments to the step functions), after clear_slope().
"""

import math
import struct
import time

from canopen.sdo.exceptions import SdoAbortedError

from . import ezero
from .flash import read_version
from .reporter import Reporter


MODE_IDLE = 0
MODE_PHASE_VOLTAGE_ANGLE = 12

GAIN_FIT_STEPS = 48         # one electrical revolution
#: Per-step dwell at full size; a caller that raised i_cal to the current
#: sense floor (p4sysid.probe) passes a smaller dwell_scale to hold the same
#: I^2Rt, down to GAIN_STEP_DWELL_MIN_S (the fit still needs a settled read).
GAIN_STEP_DWELL_S = 0.12
GAIN_STEP_DWELL_MIN_S = 0.03
GAIN_NOMINAL = 4096
BIAS_MIDPOINT = 2048        # ADC counts
#: The firmware's current-sense window (stm32 app/mode_switch.h): outside it,
#: every closed-loop mode and R/L injection is refused, and a running one
#: faults.  Bias +-100 ADC counts about mid-scale, gain factor +-200 about 1.0.
BIAS_WINDOW_COUNTS = 100
GAIN_WINDOW = 200
BIAS_TOLERANCE = BIAS_WINDOW_COUNTS / float(BIAS_MIDPOINT)
GAIN_TOLERANCE = GAIN_WINDOW / float(GAIN_NOMINAL)
ENCODER_MAX_JITTER = 8      # counts over a second at rest

#: ud step and ceiling for the calibration current ramp, and settle per step.
RAMP_UD_STEP = 100
RAMP_UD_MAX = 32000
RAMP_SETTLE_S = 0.05
#: Per-mille of i_peak at which the exported id is taken as saturated on
#: firmware before stm32 ecdc33c (later: id_scale()'s peak per-mille).
RAMP_ID_SATURATED = 1000
#: The proportional ramp (newton=True): per-step ud bounds, first step, settle.
NEWTON_STEP_MIN = 100
NEWTON_STEP_MAX = 2000
NEWTON_FIRST_STEP = RAMP_UD_MAX // 12
NEWTON_SETTLE_S = 0.05

#: puckutility's calibrate menu settings for the steps.  Bias: a shorter
#: fixed settle, then a 100-sample average for sub-count precision, in the
#: firmware's window.
#: Convergence polling was abandoned: the ADC's noise floor exceeds any
#: practical threshold for variance-based settling.  The mean converges much
#: faster than the noise floor; 0.5 s is 5-10x the firmware filter time
#: constant.  QUICK trims it to 0.35 s (still 3.5-7x): this is the 0 A
#: reference every later step subtracts, so it is not trimmed hard.
BIAS_MENU = dict(settle_s=0.5, samples=100, sample_dwell_s=0.0,
                 tolerance=BIAS_TOLERANCE)
BIAS_MENU_QUICK = dict(BIAS_MENU, settle_s=0.35)
#: Gain: the proportional ramp, sampling straight after it.
GAIN_MENU = dict(newton_ramp=True, post_ramp_s=0.0)
#: Encoder zero, quick: 8 steps per electrical cycle instead of 16, about
#: twice as fast, within ~1 deg electrical on a P4-42.
EZERO_QUICK_STEPS = 8

#: Shown with an out-of-bounds bias or gain.
_DEBUG_HINTS = ('Debugging steps:\n'
                '- Ensure the proper configuration file has been loaded\n'
                '- Verify the phase leads are properly connected')


class CalibrationError(RuntimeError):
    """A calibration step failed, or the user declined to continue."""


class CalibrationDeclined(CalibrationError):
    """The user answered no to continuing past an out-of-bounds result."""


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


def isense_window(node):
    """The stored current-sense calibration against the firmware's window.

    Returns (ok, problems): *problems* names each channel's bias or gain
    factor outside it.  Firmware refuses closed-loop current (any mode but
    IDLE and phase-voltage, and every R/L injection) until both channels are
    inside; an erased board's gain factor is 0, which reads 0 A whatever
    flows."""
    sdo = node.sdo
    problems = []
    for ch in ('alpha', 'beta'):
        bias = sdo[ch.capitalize()]['Bias'].raw
        gain = sdo[ch.capitalize()]['Gainfactor'].raw
        # Q12.4 (firmware 4.4.0 on) puts mid-scale at 32768, whole counts
        # (before) at 2048: the value says which without asking the version.
        bias = bias / 16.0 if bias >= 2 * BIAS_MIDPOINT else float(bias)
        if abs(bias - BIAS_MIDPOINT) > BIAS_WINDOW_COUNTS:
            problems.append('{} bias {:.1f} (window {} +-{})'.format(
                ch, bias, BIAS_MIDPOINT, BIAS_WINDOW_COUNTS))
        if abs(gain - GAIN_NOMINAL) > GAIN_WINDOW:
            problems.append('{} gain factor {} (window {} +-{})'.format(
                ch, gain, GAIN_NOMINAL, GAIN_WINDOW))
    return not problems, problems


def _enable(sdo):
    """Clear faults, ready to switch on, operation enabled."""
    sdo['ControlWord'].raw = 0x80
    sdo['ControlWord'].raw = 0x06
    sdo['ControlWord'].raw = 0x0F


def _read_filtered(node, samples=16, dwell_s=0.005, sleep=time.sleep,
                   progress=None):
    """Averages {'Alpha', 'Beta'} of fresh reads of the Q12.4 filtered ADC
    values, the two channels read in turn."""
    total = {'Alpha': 0, 'Beta': 0}
    for i in range(samples):
        if progress:
            progress(i / float(samples))
        for channel in ('Alpha', 'Beta'):
            total[channel] += node.sdo[channel]['Filtered'].raw
        if dwell_s:
            sleep(dwell_s)
    return {k: v / float(samples) for k, v in total.items()}


def clear_slope(node):
    """Switch off the Current Sense Slope (0x3008/0x3009,7) and save.

    It distorts the raw alpha/beta current, so bias, gain and the encoder
    zero must run without it: a stale or garbage slope poisons the gain
    (alpha reads about half, "Beta Gainfactor out of bounds") and the whole
    calibration cascades.  The slope calibration re-measures and re-stores
    it.  Errors are ignored (older firmware has no slope)."""
    try:
        node.sdo[0x3008][7].raw = 0
        node.sdo[0x3009][7].raw = 0
        save_single(node, 0x3008, 7)
        save_single(node, 0x3009, 7)
    except Exception:
        pass


def calibration_current(node):
    """i_cal, capped at i_peak (both mA)."""
    sdo = node.sdo
    return min(sdo['Calibration']['i_cal'].raw, sdo['Calibration']['i_peak'].raw)


def ramp_to_calibration_current(node, reporter=None, newton=False,
                                 progress=None):
    """Raise ud until the d-axis current reaches i_cal.  Returns that ud.

    Apply, then measure: measured first, a back-to-back run (calibrate_all)
    sees the previous step's current still decaying, exits at ud = 0 and
    calibrates against a rotor that never moved.

    *newton* (puckutility's ramp) sizes each step from the current so far,
    a quarter of the way to the ud that would give i_cal, between
    NEWTON_STEP_MIN and NEWTON_STEP_MAX.  The cap matters: a falsely low
    reading used to slam ud to the ceiling in one jump, which drives a small
    motor into overcurrent and pushes the PWM to near-full duty, where the
    bottom-shunt current sense cannot sample.  *progress(fraction of i_cal)*
    is called on every step.
    """
    reporter = reporter or Reporter()
    sdo = node.sdo
    target = calibration_current(node)
    base_ma, saturated = id_scale(node)
    ud = 0
    i_ma = 0.0
    while ud < RAMP_UD_MAX:
        if not newton:
            step = RAMP_UD_STEP
        elif ud > 0 and i_ma > 0:
            step = max(NEWTON_STEP_MIN, min(NEWTON_STEP_MAX,
                       int((ud * target / i_ma - ud) / 4)))
        else:
            step = NEWTON_FIRST_STEP
        ud = min(ud + step, RAMP_UD_MAX)
        sdo['Motor']['ud'].raw = ud
        reporter.sleep(NEWTON_SETTLE_S if newton else RAMP_SETTLE_S)
        i_d = sdo['Motor']['id'].raw
        i_ma = i_d / 1000.0 * base_ma
        if progress and target > 0:
            progress(min(1.0, max(0.0, i_ma / target)))
        if i_d >= saturated or i_ma >= target:
            break
    return ud


def deenergise(node, settle_s=1.0, sleep=time.sleep, clear_fault=False):
    """Drop ud/uq to zero, go IDLE, and let Motor/id follow it down.

    The exported current is low-passed at a cutoff from rt/lt; a second
    covers every puck.  Never raises: it is what runs on the way out.
    *clear_fault* also writes CLEAR_FAULT, ready for the next step.
    """
    writes = [lambda: setattr(node.sdo['Motor']['ud'], 'raw', 0),
              lambda: setattr(node.sdo['Motor']['uq'], 'raw', 0),
              lambda: setattr(node.sdo['SetModeOfOperation'], 'raw',
                              MODE_IDLE)]
    if clear_fault:
        writes.append(lambda: setattr(node.sdo['ControlWord'], 'raw', 0x80))
    for write in writes:
        try:
            write()
        except Exception:
            pass
    if settle_s:
        sleep(settle_s)


def _continue_or_raise(reporter, message, default=False):
    if not reporter.confirm(message + '\n\nContinue anyway?', default):
        raise CalibrationDeclined(message)


# ---------------------------------------------------------------------------
# Steps
# ---------------------------------------------------------------------------
def test_encoder(node, reporter=None, limit=ENCODER_MAX_JITTER):
    """Encoder stability at rest: position spread over one second, against
    *limit* counts (p4sysid.encfilter sets one by sensor and filter)."""
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
    result = dict(spread=spread, limit=limit, ok=spread <= limit)
    reporter.note('Encoder spread at rest: {} counts (limit {})'.format(
        spread, limit))
    if not result['ok']:
        _continue_or_raise(reporter, 'Encoder readings unstable: {} counts '
                           '(limit {}).'.format(spread, limit))
    return result


def calibrate_bias(node, reporter=None, settle_s=1.0, samples=16,
                   sample_dwell_s=0.005, tolerance=BIAS_TOLERANCE):
    """Current-sense bias (Alpha/Beta Bias, 0x3008/0x3009,3), zero current.

    Holds the bridge at ud = uq = 0 in phase-voltage-angle mode for
    *settle_s* (a fixed settle), then averages *samples* reads of both
    filtered channels.  Stores and saves both biases, then checks them
    against 2048 counts +- *tolerance*: outside it, asks whether to go on
    (CalibrationDeclined on no; the values stay stored).  BIAS_MENU and
    BIAS_MENU_QUICK are puckutility's settings.
    """
    reporter = reporter or Reporter()
    reporter.status('Calibrating current-sense bias...')
    sdo = node.sdo
    scale = bias_scale(node)
    previous = (sdo['Alpha']['Bias'].raw / scale, sdo['Beta']['Bias'].raw / scale)
    try:
        _enable(sdo)
        sdo['Theta_e'].raw = 0x7FFF         # stall at the alpha peak (+pi)
        sdo['Motor']['ud'].raw = 0
        sdo['SetModeOfOperation'].raw = MODE_PHASE_VOLTAGE_ANGLE
        # uq after the mode switch: in profile torque the PI overwrites it
        # every ISR; in phase-voltage-angle mode the write sticks.
        sdo['Motor']['uq'].raw = 0
        end = time.monotonic() + settle_s
        while True:
            left = end - time.monotonic()
            reporter.progress(int(55 * (1 - max(0.0, left) / settle_s))
                              if settle_s else 55)
            if left <= 0:
                break
            reporter.sleep(min(0.05, left))
        # Filtered is Q12.4.  Firmware 4.4.0 and later keeps Bias in Q12.4
        # too, so it is stored as read; older firmware wants whole counts.
        filt = _read_filtered(node, samples, sample_dwell_s, reporter.sleep,
                              lambda f: reporter.progress(55 + int(40 * f)))
        for channel in ('Alpha', 'Beta'):
            sdo[channel]['Bias'].raw = int(round(filt[channel] * scale / 16.0))
        save_single(node, 0x3008, 3)
        save_single(node, 0x3009, 3)
        a_bias = sdo['Alpha']['Bias'].raw / scale
        b_bias = sdo['Beta']['Bias'].raw / scale
    finally:
        try:
            sdo['SetModeOfOperation'].raw = MODE_IDLE
        except Exception:
            pass
    reporter.progress(100)
    lo = BIAS_MIDPOINT * (1 - tolerance)
    hi = BIAS_MIDPOINT * (1 + tolerance)
    ok = lo <= a_bias <= hi and lo <= b_bias <= hi
    reporter.note('Bias: alpha {:.3f} (was {:.3f}), beta {:.3f} (was {:.3f}) '
                  'counts, {}-sample average'.format(
                      a_bias, previous[0], b_bias, previous[1], samples))
    if not ok:
        _continue_or_raise(reporter, 'Current-sense bias out of bounds: alpha '
                           '{:.1f}, beta {:.1f} counts (acceptable {:.0f} - {:.0f}; '
                           'the drive refuses closed-loop current outside it).'
                           '\n\n{}'.format(a_bias, b_bias, lo, hi, _DEBUG_HINTS))
    return dict(alpha_bias=a_bias, beta_bias=b_bias, previous_alpha_bias=previous[0],
                previous_beta_bias=previous[1], ok=ok)


def calibrate_gain(node, reporter=None, newton_ramp=False, post_ramp_s=0.5,
                   settle_s=1.0, dwell_scale=1.0):
    """Beta current-sense gain factor (0x3009,6).

    Drives the calibration current on the d axis in phase-voltage-angle mode
    and rotates it through one electrical revolution in GAIN_FIT_STEPS steps,
    then takes the ratio of the two channels' fundamentals (gain_fit()).  The
    dead-time error is six-fold over a revolution, so it has no fundamental
    and the fit ignores it, along with bias drift and the 2x-electrical term.

    Why rotate: the old 2-angle method stalled at +pi (alpha) and -pi/2
    (beta); a current-sense zero that drifts between the two reads (alpha
    drifts thermally at 100 kHz) biases the ratio.  Validated hot at 100 kHz:
    stock cal 6635 out of bounds, this 3886/3878/3889.  One whole revolution
    is enough for the cancellation (sum cos = sum sin = 0); 48 samples keep
    the accuracy, and the rotor moves a third of what three revolutions
    moved.  The bounds are checked before the write, so a rejected value
    never reaches the OD.  GAIN_MENU is puckutility's setting (the
    proportional ramp, no extra settle).  dwell_scale shortens the per-step
    settle and post_ramp_s (not below GAIN_STEP_DWELL_MIN_S) for a current
    raised to the sense floor, to hold the winding's I^2Rt budget.
    """
    reporter = reporter or Reporter()
    reporter.status('Calibrating current-sense gain...')
    sdo = node.sdo
    orig = (sdo['Alpha']['Gainfactor'].raw, sdo['Beta']['Gainfactor'].raw)
    step_dwell_s = max(GAIN_STEP_DWELL_MIN_S, GAIN_STEP_DWELL_S * dwell_scale)
    if dwell_scale < 1.0:
        post_ramp_s = post_ramp_s * dwell_scale if post_ramp_s else post_ramp_s
        reporter.note('Gain: dwell scaled by {:.2f} ({:.0f} ms/step) for the '
                      'raised calibration current'.format(
                          dwell_scale, step_dwell_s * 1000))

    def restore():
        sdo['Alpha']['Gainfactor'].raw, sdo['Beta']['Gainfactor'].raw = orig

    sdo['Alpha']['Gainfactor'].raw = GAIN_NOMINAL
    sdo['Beta']['Gainfactor'].raw = GAIN_NOMINAL
    target = calibration_current(node)
    base_ma = id_scale(node)[0]
    samples, ids = [], []
    try:
        _enable(sdo)
        sdo['SetModeOfOperation'].raw = MODE_PHASE_VOLTAGE_ANGLE
        sdo['Motor']['uq'].raw = 0
        sdo['Theta_e'].raw = 0
        reporter.sleep(settle_s)
        ramp_to_calibration_current(
            node, reporter, newton=newton_ramp,
            progress=lambda f: reporter.progress(3 + int(27 * f)))
        if post_ramp_s:
            reporter.sleep(post_ramp_s)
        for k in range(GAIN_FIT_STEPS):
            au = k * 65536 // GAIN_FIT_STEPS
            sdo['Theta_e'].raw = au if au < 32768 else au - 65536
            reporter.sleep(step_dwell_s)
            samples.append((2.0 * math.pi * k / GAIN_FIT_STEPS,
                            sdo['Alpha']['Filtered'].raw, sdo['Beta']['Filtered'].raw))
            ids.append(sdo['Motor']['id'].raw / 1000.0 * base_ma)
            reporter.progress(30 + 65 * (k + 1) // GAIN_FIT_STEPS)
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
    reporter.note('Rotating gain fit: |A| {:.1f} |B| {:.1f} ({} steps, id ~{:.0f} mA)'
                  .format(a_amp, b_amp, GAIN_FIT_STEPS, id_med))
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
            'Beta gain factor {} is out of bounds ({} - {}; the drive refuses '
            'closed-loop current outside it).\n\n{}\n\n'
            'Store it anyway?'.format(gainfactor, lo, hi, _DEBUG_HINTS)):
        restore()
        raise CalibrationDeclined('Beta gain factor {} out of bounds; not stored'
                                  .format(gainfactor))
    sdo['Beta']['Gainfactor'].raw = gainfactor
    save_single(node, 0x3008, 6)
    save_single(node, 0x3009, 6)
    readback = sdo['Beta']['Gainfactor'].raw
    if readback != gainfactor:
        reporter.warn('Beta gain factor read back {}, written {}'.format(
            readback, gainfactor))
    reporter.progress(100)
    reporter.note('Beta gain factor {} (was {}, a/b {:.5f}, id {:.0f} mA)'.format(
        gainfactor, orig[1], a_amp / b_amp, id_med))
    return dict(beta_gain=gainfactor, previous_beta_gain=orig[1], id_ma=id_med,
                a_amp=a_amp, b_amp=b_amp, ok=ok)


def calibrate_ezero(node, reporter=None, steps=16):
    """Electrical zero and polarity (ezero.sweep); stores both.

    The rotor turns one mechanical revolution each way.  *steps* per
    electrical cycle (EZERO_QUICK_STEPS for the quick calibration).

    Why a revolution sweep: a spin-through at one spot cancels friction but
    not the encoder nonlinearity or cogging there; on a P4-42 single points
    scatter +-20 deg electrical about the true zero, and a one-spot cal left
    e_zero 21 deg off (a zero-current coast's back-EMF measured it; the sweep
    then agreed within ~1 deg).
    """
    reporter = reporter or Reporter()
    reporter.status('Calibrating encoder zero (one revolution each way)...')
    try:
        node.sdo['ControlWord'].raw = 0x80
        r = ezero.sweep(node, steps=steps,
                        progress=lambda f: reporter.progress(95 * f),
                        sleep=reporter.sleep)
    except ezero.EZeroError as exc:
        hint = ''
        try:
            hint = ('; check the configuration (poles, i_cal) and that output '
                    'friction is under the calibration torque ({:.0f} mNm)'
                    .format(node.sdo['Calibration']['i_cal'].raw
                            * read_kt(node) / 1000))
        except Exception:
            pass
        raise CalibrationError('encoder zero failed, nothing stored: {}{}'
                               .format(exc, hint)) from exc
    sdo = node.sdo
    previous = sdo['Calibration']['e_zero'].raw
    r['previous_polarity'] = sdo['Calibration']['e_polarity'].raw
    sdo['Calibration']['e_polarity'].raw = r['e_polarity']
    sdo['Calibration']['e_zero'].raw = r['e_zero']
    save_single(node, 0x3011, 2)    # e_polarity
    save_single(node, 0x3011, 1)    # e_zero
    r['previous'] = previous
    r['shift_deg'] = ezero.wrap(r['e_zero'] - previous, r['cts_per_elec_cycle']) \
        * 360.0 / r['cts_per_elec_cycle']
    reporter.progress(100)
    reporter.note('Encoder zero {} (was {}, {:+.1f} deg elec), polarity {:+d} '
                  '(was {:+d}); friction split {:.1f} deg, points scatter '
                  '+-{:.1f} deg (rms {:.1f})'.format(
                      r['e_zero'], previous, r['shift_deg'], r['e_polarity'],
                      r['previous_polarity'], r['friction_split_deg'],
                      r['spread_max_deg'], r['spread_rms_deg']))
    return r


#: Kt: 0x3011,14 is a REAL32 in mNm/A rms (stm32 c330a51).  Firmware before
#: it has only 0x3011,4, a U16 in whole mNm/A, which newer firmware keeps as
#: a rounded mirror (unsaved; a write to it sets ,14).
KT = (0x3011, 14)
KT_U16 = (0x3011, 4)


def has_float_kt(node):
    """True on firmware with the float Kt, 0x3011,14."""
    try:
        node.sdo.upload(*KT)
        return True
    except SdoAbortedError:
        return False


def read_kt(node):
    """Kt in mNm/A rms, as a float: 0x3011,14, or 0x3011,4 on older firmware."""
    try:
        data = bytes(node.sdo.upload(*KT))
    except SdoAbortedError:
        data = bytes(node.sdo.upload(*KT_U16))
        return float(int.from_bytes(data[:2], 'little'))
    return struct.unpack('<f', data[:4])[0]


def write_kt(node, mnm, save=False):
    """Set Kt (mNm/A rms) where this firmware keeps it, rounded to whole
    mNm/A on older firmware, and save it when *save*.  Returns where."""
    mnm = float(mnm)
    if not (0.0 <= mnm <= 1e6):
        raise ValueError('Kt {} mNm/A is out of range'.format(mnm))
    if has_float_kt(node):
        where, data = KT, struct.pack('<f', mnm)
    else:
        where, data = KT_U16, struct.pack('<H', min(65535, int(round(mnm))))
    node.sdo.download(*where, data)
    if save:
        save_single(node, *where)
    return where


#: CiA 402 units (stm32 ecdc33c, "DS402 stage 2"): every torque and current
#: per-mille (0x6071-0x6074, 0x6077, 0x6078, 0x3010,6, 0x60B2, 0x60E0/1,
#: 0x6087, the velocity and impedance gains) is of the RATED current 0x6075
#: (= i_cont, mA) and the rated torque 0x6076 = Kt x 0x6075 / sqrt2, both
#: read-only and derived.  Older firmware has no 0x6075: its current per-mille
#: is of i_peak (0x3011,9) and its torque per-mille of a writable 0x6076 the
#: host chose.  0x3011,15 is the working peak (min of i_peak, amp.i_max and
#: the sense; stage 1), absent before it.
RATED_CURRENT = (0x6075, 0)
RATED_TORQUE = (0x6076, 0)
WORKING_PEAK = (0x3011, 15)
I_PEAK = (0x3011, 9)
I_CONT = (0x3011, 8)
SQRT2 = 2.0 ** 0.5


def _u(node, index, sub, n):
    return int.from_bytes(bytes(node.sdo.upload(index, sub))[:n], 'little')


def has_rated_units(node):
    """True on firmware whose per-mille are of the rated current (0x6075)."""
    try:
        node.sdo.upload(*RATED_CURRENT)
        return True
    except (SdoAbortedError, KeyError):   # KeyError: absent from a test fake
        return False


def working_peak_ma(node):
    """The peak current the drive works to (mA): 0x3011,15, or i_peak on
    firmware without it."""
    try:
        return _u(node, *WORKING_PEAK, 2)
    except (SdoAbortedError, KeyError):   # KeyError: absent from a test fake
        return _u(node, *I_PEAK, 2)


def permille_base_ma(node):
    """What a current per-mille (0x6078, 0x3010,6, 0x6073) is of, in mA: the
    rated current 0x6075, or i_peak on older firmware.  0 = no scale."""
    try:
        return _u(node, *RATED_CURRENT, 4)
    except (SdoAbortedError, KeyError):   # KeyError: absent from a test fake
        return _u(node, *I_PEAK, 2)


def id_scale(node):
    """(base mA, saturation per-mille) of the exported d-axis current
    Motor/id (0x3010,6).  Rated-unit firmware: per-mille of 0x6075, pinned at
    the working peak's per-mille (3000 for a P4-42 at i_cont 9428); older
    firmware: per-mille of i_peak, pinned at 1000.  Probes that a test fake
    cannot answer (no upload, no row) read as older firmware."""
    try:
        rated = _u(node, *RATED_CURRENT, 4)
    except (SdoAbortedError, KeyError, AttributeError):
        rated = 0
    if not rated:
        return node.sdo['Calibration']['i_peak'].raw, RAMP_ID_SATURATED
    try:
        peak = _u(node, *WORKING_PEAK, 2)
    except (SdoAbortedError, KeyError, AttributeError):
        peak = node.sdo['Calibration']['i_peak'].raw
    return rated, int(1000.0 * peak / rated)


def rated_torque_mnm(node):
    """What a torque per-mille is of, in mNm.  On rated-unit firmware this is
    computed as the firmware does (Kt x 0x6075 / sqrt2, unrounded: 0x6076
    publishes it in whole mNm); older firmware's writable 0x6076 as is."""
    if has_rated_units(node):
        return read_kt(node) * _u(node, *RATED_CURRENT, 4) * 1e-3 / SQRT2
    try:
        return float(node.sdo['RatedTorque'].raw)
    except KeyError:    # a node without the EDS names
        return float(_u(node, *RATED_TORQUE, 4))


#: The user polarity: 0x607E (DS402) holds 0x00, or 0xC0 to invert position
#: and velocity together.  Firmware before stm32 8a366c6 kept a +-1 sign at
#: 0x3013,3 instead (0x607E was there, but unused).
POLARITY = (0x607E, 0)
POLARITY_INVERT = 0xC0
LEGACY_POLARITY = (0x3013, 3)


def has_legacy_polarity(node):
    """True on firmware that still has 0x3013,3 (and ignores 0x607E)."""
    try:
        node.sdo.upload(*LEGACY_POLARITY)
        return True
    except SdoAbortedError:
        return False


def polarity_sign(raw):
    """+1 or -1 from a 0x607E value (bit 7, the position bit, decides)."""
    return -1 if raw & 0x80 else 1


def read_user_polarity(node):
    """The user sign, +1 or -1 (0 on old firmware never configured)."""
    if has_legacy_polarity(node):
        return int.from_bytes(node.sdo.upload(*LEGACY_POLARITY)[:1], 'little',
                              signed=True)
    return polarity_sign(node.sdo.upload(*POLARITY)[0])


def write_user_polarity(node, sign, save=True):
    """Set the user sign (+1 or -1) where this firmware keeps it, and save it."""
    if sign not in (1, -1):
        raise ValueError('user polarity is +1 or -1, not {}'.format(sign))
    where = LEGACY_POLARITY if has_legacy_polarity(node) else POLARITY
    if where == LEGACY_POLARITY:
        node.sdo.download(*where, bytes([sign & 0xFF]))
    else:
        node.sdo.download(*where, bytes([POLARITY_INVERT if sign < 0 else 0]))
    if save:
        save_single(node, *where)
    return where


def set_user_direction(node, reporter=None, threshold=1 / 8.0, poll_s=1.0,
                       timeout_s=None):
    """The user turns the output by hand in the direction they call positive;
    the user polarity (0x607E, or 0x3013,3 on older firmware) is set so that
    reads positive, and saved.

    Starts from +1 (0x607E = 0x00), waits until the position has moved more
    than *threshold* of a revolution, then keeps +1 if it moved up, else
    writes -1 (0xC0).  *timeout_s* None waits until moved or aborted."""
    reporter = reporter or Reporter()
    sdo = node.sdo
    previous = read_user_polarity(node)
    write_user_polarity(node, 1, save=False)
    res = sdo['EncoderConfig']['Resolution'].raw
    start = sdo['PositionFeedback'].raw
    reporter.status('Please turn the motor in the positive (+) direction...')
    t0 = time.monotonic()
    while True:
        reporter.sleep(poll_s)
        moved = sdo['PositionFeedback'].raw - start
        if abs(moved) > res * threshold:
            break
        if timeout_s is not None and time.monotonic() - t0 > timeout_s:
            if previous in (1, -1):
                write_user_polarity(node, previous, save=False)
            raise CalibrationError('the motor did not turn ({} counts); '
                                   'user direction unchanged'.format(moved))
    polarity = 1 if moved > 0 else -1
    where = write_user_polarity(node, polarity)
    reporter.note('User direction: user polarity {:+d} (was {:+d}) in 0x{:04X},{}, '
                  'moved {} counts'.format(polarity, previous, where[0], where[1], moved))
    return dict(user_polarity=polarity, previous=previous, moved=moved)


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
