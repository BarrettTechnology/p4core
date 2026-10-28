"""commission.py -- the session limits and the gates a calibration runs behind.

  Limits             the user's bounds: speed, current, test power, supply
                     current, temperature
  wait_until_cool    hold off until the winding is back near the start
  require_isense     refuse closed-loop current on an uncalibrated sense
  _stage_preflight   bus, winding temperature, the firmware's capture and
                     voltage stream, i_peak, the temperature limit armed on
                     the device
  _stage_test_current  the stationary test current: from R and a power
                     budget, never past the user's limit or half of i_peak
  _stage_winding     e_zero and the winding order over a whole revolution
  provisional_i_cont, cap_i_peak, set_rated_torque
                     a working i_cont / i_peak / RatedTorque before the
                     motor's own are measured
  write_od, drive_units
                     results to the drive's RAM in its units

All results are SI and per phase; drive_units() converts to the drive's.
"""

import math
import struct

from canopen.sdo.exceptions import SdoAbortedError
from p4core.calibrate import has_rated_units, read_kt, write_kt

from . import fits
from .device import SysidError


# ---------------------------------------------------------------------------
# Limits and design rules
# ---------------------------------------------------------------------------
#: How long a step waits for the winding (or the DSP standing in for it)
#: to cool to max_temp_c - TEMP_START_MARGIN_C before it gives up.  The
#: thermal test ends hot on purpose: a GL40 board's DSP read 76 C after it,
#: and came back under 45 C in a few minutes (tau 175 s).  0: do not wait.
COOL_WAIT_S = 1800.0
#: Seconds between temperature reads while waiting.
COOL_POLL_S = 10.0
#: The wait cools this far under the start gate: the DSP reads in whole
#: degrees, and a winding that just made 44 C read 45.0 C at the preflight
#: a moment later (node 3, Resistance).
COOL_HYSTERESIS_C = 1.0


def wait_until_cool(dev, limits, log=None):
    """Wait, up to limits.cool_wait_s, for the winding temperature to fall
    below max_temp_c - TEMP_START_MARGIN_C.  Returns the seconds waited.
    Does nothing without a sensor or a limit; the preflight's check still
    decides."""
    log = log or dev._log
    wait = getattr(limits, 'cool_wait_s', 0) or 0
    if limits.max_temp_c is None or wait <= 0:
        return 0.0
    target = limits.max_temp_c - TEMP_START_MARGIN_C - COOL_HYSTERESIS_C
    t, source = dev.read_winding_temp_c()
    if t is None or t < target:
        return 0.0
    log('cooling: {:.0f} C ({}), waiting for {:.0f} C (up to {:.0f} s)'
        .format(t, source, target, wait))
    waited, last_log = 0.0, 0.0
    while waited < wait:
        dev._check_abort()
        dev._sleep(COOL_POLL_S)
        waited += COOL_POLL_S
        t = dev.read_winding_temp_c()[0]
        if t is None or t < target:
            break
        if waited - last_log >= 60.0:
            log('cooling: {:.0f} C after {:.0f} s'.format(t, waited))
            last_log = waited
    log('cooling: {} C after {:.0f} s'.format(
        'n/a' if t is None else '{:.0f}'.format(t), waited))
    return waited


class Limits:
    """What the user allows.  Every stage stays inside these.

    max_current_a   hard ceiling on any test current (A peak, dq).  None: half
                    of i_peak.
    max_temp_c      the run stops when the winding reaches this.
    max_speed_rpm   the rotor is never spun faster (a mechanical limit: the
                    load, the gearbox, the bearings).
    test_power_w    I^2 R budget that sizes the stationary test current.
    min_current_a   below this the stationary tests cannot beat cogging and
                    friction; a winding whose power budget allows less than
                    this gets a warning, not more current.
    allow_pwm_change  write the chosen PWM frequency and control divisor
                    (0x3001,1/13; saved, rebooted).  On by default:
                    --keep-pwm turns it off.
    supply_a        the bus supply's current limit (a bench's 2 A).  The
                    spins are sized to draw at most SUPPLY_USE of it; None
                    leaves them to the bus-sag cut alone.
    max_pwm_hz      the highest PWM the recommendation may pick (None: the
                    bounds in fits.choose_pwm alone).
    i_peak_a        i_peak to write instead of deriving it from i_cont
                    (i_peak_policy).  None: the model's FIXED_I_PEAK_MA if
                    it has one, else derived; 0: derived even then.
    """

    def __init__(self, max_current_a=None, max_temp_c=65.0,
                 max_speed_rpm=3000.0, test_power_w=10.0, min_current_a=0.3,
                 allow_pwm_change=True, supply_a=None, max_pwm_hz=None,
                 cool_wait_s=COOL_WAIT_S, i_peak_a=None):
        self.max_current_a = max_current_a
        self.max_temp_c = max_temp_c
        self.max_speed_rpm = max_speed_rpm
        self.test_power_w = test_power_w
        self.min_current_a = min_current_a
        self.allow_pwm_change = allow_pwm_change
        self.supply_a = supply_a
        self.max_pwm_hz = max_pwm_hz
        self.cool_wait_s = cool_wait_s
        self.i_peak_a = i_peak_a

    def as_dict(self):
        return dict(vars(self))


#: The winding must be at least this far below max_temp_c to start: a whole
#: run warms a P4-42 by 15-19 C at the sensor (10 W test power), and a run
#: cut off by the temperature limit part way through is wasted.
TEMP_START_MARGIN_C = 20.0
#: Bus within this fraction of NominalBusVoltage, or the gains and U_MAX are
#: scaled for a different supply.
BUS_TOLERANCE = 0.2
#: A stationary test current's R drop, at most this fraction of U_MAX: the
#: ramp stops at device.UD_MEASURED_CAP (0.60), and the copper gains ~16 %
#: by the 65 C limit plus the bridge's drop.  0.77 A (10 W) into an 11.3 ohm
#: P4-16 needed 0.63 of U_MAX: poles, winding and i_cont all stopped short.
TEST_VOLTAGE_FRACTION = 0.45
#: Test-current floors.  The sense: the gain fit's SENSE_FLOOR_COUNTS (60
#: ADC counts).  The bridge: past the dead-time knee (hbridge.knee_bias_a)
#: when the R sweep pinned it; when it did not, I R at TEST_IR_OVER_VDT
#: times the drop, so a straight line through the levels is mostly R.
#: A floor raises the test current only within its caps (i_peak / 2, the
#: session's or the winding's current limit, FLOOR_POWER_FACTOR times the
#: test power); past them the test runs at the cap and says so.
TEST_IR_OVER_VDT = 5.0
FLOOR_POWER_FACTOR = 2.0
#: The drive's commutation sawtooth (res not a multiple of the pole pairs)
#: past this costs 1 - cos = 0.1 % torque and ripple worth mentioning.
COMMUTATION_ERROR_WARN_DEG = 3.0
#: Winding sweep spread (cogging, load) past this is worth a warning.
WINDING_SPREAD_WARN_DEG = 10.0


def speed_limits(ke_v_s_per_rad, v_bus_v, r_ohm=None, lq_h=None,
                 pole_pairs=None, current_a=0.0, headroom=1.0):
    """No-load and loaded top speeds, rpm, from the back-EMF constant.

    U_MAX = V_bus / sqrt(3) is the peak phase voltage space-vector modulation
    reaches; *headroom* (fits.pwm_voltage_headroom) is what the ADC window
    leaves of it.  Loaded, at iq = *current_a* with id = 0:

        ud = -we Lq iq,  uq = R iq + we psi,  |u| = headroom * U_MAX
    """
    u_max = v_bus_v / math.sqrt(3.0)
    rad = 60.0 / (2.0 * math.pi)
    out = {'u_max_v': u_max, 'headroom': headroom,
           'no_load_ideal_rpm': u_max / ke_v_s_per_rad * rad,
           'no_load_rpm': headroom * u_max / ke_v_s_per_rad * rad}
    if current_a and r_ohm and lq_h and pole_pairs:
        pp = pole_pairs
        psi = ke_v_s_per_rad / pp
        u = headroom * u_max
        # (we Lq i)^2 + (R i + we psi)^2 = u^2, a quadratic in we.
        a = (lq_h * current_a) ** 2 + psi ** 2
        b = 2.0 * r_ohm * current_a * psi
        c = (r_ohm * current_a) ** 2 - u ** 2
        disc = b * b - 4 * a * c
        we = (-b + math.sqrt(disc)) / (2 * a) if disc > 0 and c < 0 else 0.0
        out['loaded_rpm'] = we / pp * rad
        out['loaded_current_a'] = current_a
    return out


def test_current(r_ohm, i_peak_a, limits, u_max_v=None):
    """The stationary test current: the I^2 R budget, capped (and by
    TEST_VOLTAGE_FRACTION of *u_max_v* when given)."""
    i = math.sqrt(limits.test_power_w / (1.5 * r_ohm))
    i = min(i, 0.5 * i_peak_a)
    if u_max_v:
        i = min(i, TEST_VOLTAGE_FRACTION * u_max_v / r_ohm)
    if limits.max_current_a:
        i = min(i, limits.max_current_a)
    return i


def drive_units(values):
    """SI per-phase results -> the drive's objects (what commit writes)."""
    od = {}
    if values.get('r_ohm'):
        od['rt'] = int(round(2.0 * values['r_ohm'] * 1e3))       # mohm l-l
    if values.get('ld_h'):
        od['lt'] = int(round(2.0 * values['ld_h'] * 1e6))        # uH l-l
    if values.get('lq_h'):
        od['Lq'] = int(round(2.0 * values['lq_h'] * 1e6))        # uH l-l
    if values.get('kt_nm_per_a'):
        od['kt'] = round(values['kt_nm_per_a'] * 1e3, 3)          # mNm/A rms, REAL32
    if values.get('j_kg_m2'):
        od['j'] = int(round(fits.kgm2_to_gcm2(values['j_kg_m2']) * 32768))
    if values.get('tc_nm') is not None:
        # The drag fit's friction, stored for a later session's friction_ff.
        od['tc'] = round(max(0.0, values['tc_nm']) * 1e3, 4)     # mNm, REAL32
        od['b'] = round(max(0.0, values.get('b_nm_s_per_rad') or 0.0) * 1e3,
                        7)                                      # mNm.s/rad
    if values.get('pole_pairs'):
        od['poles'] = 2 * int(values['pole_pairs'])
    for key in ('e_zero', 'e_polarity'):
        if values.get(key) is not None:
            od[key] = int(values[key])
    if values.get('kp') is not None:
        od['Kp'] = float(values['kp'])
        od['Ki'] = float(values['ki'])
    if values.get('no_load_rpm'):
        od['NoLoadSpeed'] = int(round(min(values['no_load_rpm'], 65535)))
    if values.get('pwm_hz') and values.get('pwm_changed'):
        od['Frequency'] = int(round(values['pwm_hz']))
        # After Frequency: the firmware resolves K against the rate.
        if values.get('pwm_periods'):
            od['Periods'] = int(values['pwm_periods'])
    return od


#: Where each drive_units key lives: (index, sub, SDO path).
OD_MAP = {
    'e_zero':      (0x3011, 1, ('Calibration', 'e_zero')),
    'e_polarity':  (0x3011, 2, ('Calibration', 'e_polarity')),
    'poles':       (0x3011, 3, ('Calibration', 'poles')),
    'kt':          (0x3011, 14, ('Calibration', 'kt_float')),  # write_od: write_kt (,4 on older firmware)
    'rt':          (0x3011, 5, ('Calibration', 'rt')),
    'lt':          (0x3011, 6, ('Calibration', 'lt')),
    'j':           (0x3011, 7, ('Calibration', 'j')),
    'Lq':          (0x3011, 12, ('Calibration', 'Lq')),
    'tc':          (0x3011, 17, (0x3011, 17)),   # write_od: REAL32, optional
    'b':           (0x3011, 18, (0x3011, 18)),   # (stm32 898910f)
    'Kp':          (0x2380, 1, (0x2380, 1)),
    'Ki':          (0x2380, 2, (0x2380, 2)),
    'NoLoadSpeed': (0x3024, 6, ('TuningAppProperties', 'NoLoadSpeed')),
    'Frequency':   (0x3001, 1, ('Amp', 'Frequency')),
    'Periods':     (0x3001, 13, (0x3001, 13)),
}
#: Stored only, on firmware from stm32 898910f: older firmware has no such
#: object, and write_od notes it instead of failing.
OPTIONAL_FLOATS = ('tc', 'b')


#: motion.c before rated units (stm32 ecdc33c): a torque demand of 1000
#: per-mille is iq = 1.414 RatedTorque / kt, and the demand saturates there.
#: RatedTorque = kt i_peak / 1.414 puts that full demand at i_peak, the most
#: the drive will give.  Rated-unit firmware computes 0x6076 itself (Kt
#: i_cont / sqrt2) and the demand runs on to the working peak; the writes
#: below are gated on has_rated_units.
DEMAND_RMS = 1.414


def rated_torque_mnm(kt_mnm_per_a, i_peak_ma):
    """0x6076 (mNm) that puts a full torque demand at *i_peak_ma*."""
    return max(1, int(round(kt_mnm_per_a * i_peak_ma / 1e3 / DEMAND_RMS)))


def set_rated_torque(dev, only_if_zero=False):
    """RatedTorque from the drive's kt and i_peak (rated_torque_mnm).  With
    *only_if_zero*, only a RatedTorque of 0 (a blank drive) is replaced.
    Returns the value written, or None.

    Firmware from stm32 ecdc33c derives 0x6076 itself (Kt x i_cont / sqrt2,
    read-only): nothing is written, and the drive's value is returned (None
    with *only_if_zero*: it is never the user's to replace)."""
    sdo = dev.node.sdo
    if has_rated_units(dev.node):
        return None if only_if_zero else int(sdo['RatedTorque'].raw)
    if only_if_zero and int(sdo['RatedTorque'].raw) > 0:
        return None
    kt = read_kt(dev.node)
    ip = int(sdo['Calibration']['i_peak'].raw)
    if kt <= 0 or ip <= 0:
        return None
    rated = rated_torque_mnm(kt, ip)
    sdo['RatedTorque'].raw = rated
    dev._log('commission: RatedTorque = {} mNm (kt {} mNm/A x i_peak {} mA '
             '/ {})'.format(rated, kt, ip, DEMAND_RMS))
    return rated


def _rated_is_derived(dev, kt_mnm):
    """Is the drive's RatedTorque what set_rated_torque gives for *kt_mnm*
    (mNm/A) and the drive's i_peak?"""
    if not kt_mnm or kt_mnm <= 0:
        return False
    if has_rated_units(dev.node):
        return True     # the drive derives it
    sdo = dev.node.sdo
    try:
        rated = int(sdo['RatedTorque'].raw)
        ip = int(sdo['Calibration']['i_peak'].raw)
    except Exception:
        return False
    # A drive commissioned before Kt was a float set it from whole mNm/A.
    return ip > 0 and min(abs(rated - rated_torque_mnm(k, ip))
                          for k in (kt_mnm, round(kt_mnm))) <= 1


#: i_peak over i_cont, used for the provisional pair below as well as the
#: real one the thermal step writes: the shipped P4 CSVs run 2.0 to 2.8.
I_PEAK_OVER_CONT = 2.5

#: i_peak stays under this fraction of the current sense's full scale: past
#: it the ADC clips and the loop regulates a flat top.  A P4-37 1323 (20 mOhm,
#: x7.49) reads 11.0 A; 2.5 x a GL40's 4.66 A i_cont asked for 11.6 A.
I_PEAK_SENSE_MARGIN = 0.9


def sense_full_scale_ma(dev):
    """The most the current sense reads, mA: half the 3.3 V ADC span over
    the alpha shunt x amplifier gain (0x3008,5 mOhm, 0x3008,4 x1000), the
    firmware's ma_per_ct x 2048 (stm32 app/pwm.c).  Beta's amplifier is
    alpha's over sqrt(3) on (B - C), so it reads the same beta current.
    None when either is unset or unreadable."""
    try:
        gain = int(dev.node.sdo['Alpha']['Gain'].raw)
        shunt = int(dev.node.sdo['Alpha']['Shunt'].raw)
    except Exception:
        return None
    if gain <= 0 or shunt <= 0:
        return None
    return 1.65e9 / (shunt * gain)


def cap_i_peak(dev, i_peak_ma):
    """(*i_peak_ma* capped at I_PEAK_SENSE_MARGIN of the sense's full scale,
    the cap or None)."""
    fs = sense_full_scale_ma(dev)
    if fs is None:
        return i_peak_ma, None
    cap = int(I_PEAK_SENSE_MARGIN * fs)
    return min(i_peak_ma, cap), cap


#: Models whose i_peak is a requirement, not a derivation, mA: i_cont is
#: measured, i_peak is this.  P4-42: the customer rejects anything but 28 A
#: (the shipped template's 28284 mA, 20 A rms; the sense reads 31.2 A).
FIXED_I_PEAK_MA = {'P4-42': 28284}


def model_of(dev):
    """The drive's model name from 0x1018,2, or None."""
    try:
        from p4core import can_backend
        raw = dev.node.sdo.upload(0x1018, 2)
        return can_backend.model_from_product_code(
            int.from_bytes(raw[:4], 'little'))
    except Exception:
        return None


def i_peak_policy(dev, limits, i_cont_ma):
    """(i_peak mA, how, sense cap mA or None) for *i_cont_ma*:
    limits.i_peak_a when given (>0), else the model's FIXED_I_PEAK_MA,
    else I_PEAK_OVER_CONT x i_cont capped under the sense's full scale
    (cap_i_peak).  A required i_peak is written as it is, never capped:
    the caller warns when it is past the sense's full scale (the third
    item, mA, None when unknown)."""
    want = getattr(limits, 'i_peak_a', None)
    fs = sense_full_scale_ma(dev)
    full = None if fs is None else int(fs)
    if want:
        return int(round(want * 1000.0)), '--i-peak', full
    if want is None:
        model = model_of(dev)
        if model in FIXED_I_PEAK_MA:
            return FIXED_I_PEAK_MA[model], 'fixed for the {}'.format(model), full
    i_peak, _ = cap_i_peak(dev, int(round(I_PEAK_OVER_CONT * i_cont_ma)))
    how = '{:g} x i_cont'.format(I_PEAK_OVER_CONT)
    if i_peak < int(round(I_PEAK_OVER_CONT * i_cont_ma)):
        how += ', capped at {:.0%} of the sense'.format(I_PEAK_SENSE_MARGIN)
    return i_peak, how, full


def provisional_i_cont(dev, ctx, limits=None):
    """i_cont/i_peak to work with before the thermal step (now last, right
    before Save) has measured the real ones: the drive's own when it has
    both, else from the winding probe's i_cal.

    The drive's own are what the steps ran on before the thermal step moved
    (a template's or a past session's).  i_cal is far under a real i_cont
    (1.4 A against 5.9 A on a P4-42), and as i_peak it would clip the
    velocity and position steps, so it is only the blank drive's stand-in.
    A wrong i_peak does not leave the loops mistuned: _write_i_cont rescales
    the velocity gains when RatedTorque moves.

    Everything from here to there that sizes a current off i_peak -- the R
    and L sweep caps, the drag/friction excitation, the current/velocity/
    position steps, and RatedTorque (the velocity and position designs'
    torque gain) -- divides or multiplies by it; a blank one (an erased
    drive) would make several of them zero, and a shipped CSV's template is
    not even safe to assume (settling.py: "ten times the real one on a
    P4-16").  i_cal is already this session's trusted ceiling (a real,
    if short, thermal measurement -- Session.motor_limits), so it is reused
    here rather than guessing again; it runs a quick-heat rate, not a
    steady-state one, so it tends to overestimate the steady-state i_cont a
    little, not underestimate it.

    That direction matters for the velocity and position designs
    specifically: their Kp is sized to put the *assumed* RatedTorque's
    plant at a target crossover, so an assumed RatedTorque on the low side
    gives a Kp that runs the real (higher-torque) plant faster than
    intended on the very first try, before the steps' own backoff ever
    gets a say. Overestimating runs it slower instead -- the safe side,
    and the one the backoff (which only ever softens) can equally recover
    from if the first try is too soft to pass.

    Always overwrites (like provisional_current_pi): a wrong template is
    worse than i_cal's own estimate.  The real thermal step overwrites this
    in turn (_write_i_cont); neither the velocity nor the position design
    needs to be redone then, because both are accepted by an empirical
    bench step with its own pass/fail and backoff (tune.tune_velocity,
    tune_position) against the real motor, not by this design's assumed
    RatedTorque -- a wrong assumption changes how many backoff tries it
    takes, not whether the gains it settles on pass.  See RUNBOOK.md,
    the i_cont step.
    """
    cal = dev.node.sdo['Calibration']
    have = int(cal['i_cont'].raw), int(cal['i_peak'].raw)
    if have[0] > 0 and have[1] > 0:
        i_peak, how, _ = i_peak_policy(dev, limits, have[0])
        if not how.startswith('{:g}'.format(I_PEAK_OVER_CONT)) and \
                i_peak != have[1]:
            derived = _rated_is_derived(dev, read_kt(dev.node))
            cal['i_peak'].raw = i_peak
            if derived:
                set_rated_torque(dev)
            dev._log('commission: i_peak {} -> {} mA ({})'.format(
                have[1], i_peak, how))
            have = have[0], i_peak
            ctx['s'] = dev.refresh_scaling()
        set_rated_torque(dev, only_if_zero=True)
        dev._log('commission: provisional i_cont {} mA, i_peak {} mA (the '
                 'drive\'s) -- the thermal step (last, before Save) replaces '
                 'it'.format(*have))
        return have
    i_cal_ma = int(cal['i_cal'].raw)
    i_cont_ma = i_cal_ma or int(round(ctx.get('i_test', 0.0) * 1000.0))
    if i_cont_ma <= 0:
        return None
    i_peak_ma, _, _ = i_peak_policy(dev, limits, i_cont_ma)
    cal['i_cont'].raw = i_cont_ma
    cal['i_peak'].raw = i_peak_ma
    set_rated_torque(dev, only_if_zero=True)
    ctx['s'] = dev.refresh_scaling()
    dev._log('commission: provisional i_cont {} mA, i_peak {} mA (from {}) -- '
             'the thermal step (now last, before Save) replaces it'.format(
                 i_cont_ma, i_peak_ma, 'i_cal' if i_cal_ma else 'the test current'))
    return i_cont_ma, i_peak_ma


def _od_var(node, key):
    a, b = OD_MAP[key][2]
    return node.sdo[a][b]


def write_od(dev, od, keys=None, save=False):
    """Write drive_units values to RAM (and NVM when *save*)."""
    for key in keys or od:
        if key not in od:
            continue
        value = od[key]
        if key == 'kt':
            write_kt(dev.node, value, save=save)
        elif key in OPTIONAL_FLOATS:
            # Raw, not by name: an app's older EDS may not list the object.
            idx, sub, _ = OD_MAP[key]
            try:
                dev.node.sdo.download(idx, sub, struct.pack('<f', value))
            except SdoAbortedError:
                dev._log('commission: this firmware has no 0x{:04X},{} ({}): '
                         'not stored'.format(idx, sub, key))
                continue
            if save:
                dev.node.sdo['Save']['Single'].raw = (idx << 8) | sub
        else:
            _od_var(dev.node, key).raw = value
            if save:
                idx, sub, _ = OD_MAP[key]
                dev.node.sdo['Save']['Single'].raw = (idx << 8) | sub
        dev._log('commission: {} = {}{}'.format(key, od[key],
                                                ' (saved)' if save else ''))


# ---------------------------------------------------------------------------
# The pipeline
# ---------------------------------------------------------------------------
class _Report:
    def __init__(self, dev):
        self.dev = dev
        self.values = {}
        self.checks = []
        self.raw = {}
        self.ctx = {}
        self.stage = None

    def check(self, name, ok, detail, fatal=False):
        severity = 'pass' if ok else ('fail' if fatal else 'warn')
        self.checks.append({'name': name, 'ok': bool(ok),
                            'severity': severity, 'detail': detail})
        self.dev._log('commission: [{}] {}: {}'.format(
            severity.upper(), name, detail))
        if not ok and fatal:
            raise SysidError('{} failed: {}'.format(name, detail))


# Each stage reads what the stages before it left in rep.values (SI results)
# and rep.ctx (working state: the scaling, the test current, the coasts), and
# leaves its own there.  identify_motor runs them all in order; steps.py runs
# one at a time, seeding both from the drive for any stage left out.

def _read_i8(dev, index, sub):
    try:
        return int.from_bytes(dev.node.sdo.upload(index, sub)[:1], 'little',
                              signed=True)
    except Exception:
        return None


def _read_u8(dev, index, sub):
    try:
        return dev.node.sdo.upload(index, sub)[0]
    except Exception:
        return None


def require_isense(dev, rep=None):
    """Stop unless the current sense is calibrated (p4core
    calibrate.isense_window).  Firmware refuses closed-loop current without
    it; this says why before a refused mode switch does.  An erased gain
    factor reads 0 A whatever flows: a HOLD on one browned out the bench
    supply (2026-10-14)."""
    from p4core import calibrate
    ok, problems = calibrate.isense_window(dev.node)
    detail = ('in the window' if ok else '; '.join(problems) +
              ' -- run the Bias and Gain calibrations first')
    if rep is not None:
        rep.check('current sense calibrated', ok, detail, fatal=True)
    elif not ok:
        raise SysidError('current sense not calibrated: ' + detail)


def _stage_preflight(dev, limits, reboot, rep):
    v, c = rep.values, rep.ctx
    rep.stage = 'preflight'
    s = c['s'] = dev.refresh_scaling()
    v['i_peak_a'] = s['i_max_a']
    v['enc_res'] = s['enc_res']
    rep.check('config loaded', s['i_max_a'] > 0 and s['enc_res'] > 0,
              'i_peak {:.0f} mA, encoder {} counts: load the model\'s CSV '
              '(p4core load_config) first -- i_peak sizes every test '
              'current, and a blank one makes them 0 A'.format(
                  s['i_max_a'] * 1e3, s['enc_res']), fatal=True)
    bus = dev.read_bus_v()
    v['bus_v'] = c['bus'] = bus
    rep.check('bus voltage', bus is not None and bus > 0,
              '{} V (nominal {:.1f})'.format(bus, s['v_nom']), fatal=True)
    rep.check('bus near nominal', abs(bus / s['v_nom'] - 1) <= BUS_TOLERANCE,
              '{:.1f} V vs NominalBusVoltage {:.1f} V: U_MAX and the gains '
              'scale with the nominal'.format(bus, s['v_nom']))
    t, source = dev.read_winding_temp_c()
    v['start_temp_c'] = t
    c['temp_source'] = source
    if t is None:
        # No sensor (absent or floating): nothing to stop on and nothing to
        # correct Rs with.  The run is power-limited; say so and carry on.
        raw = getattr(dev, 'therm_implausible_c', None)
        rep.check('winding temperature', False,
                  'no sensor{} -- no over-temperature stop, Rs is reported '
                  'at the winding\'s present temperature, uncorrected'.format(
                      '' if raw is None else ' (reads {:.1f} C)'.format(raw)))
    else:
        if source == 'DSP':
            # The controller is on the motor body: its sensor stands in, a
            # step behind the copper under load.
            dev._log('commission: no thermistor: the DSP ({:.0f} C) stands '
                     'in for the winding'.format(t))
        rep.check('winding temperature',
                  t < limits.max_temp_c - TEMP_START_MARGIN_C,
                  '{:.1f} C{}, limit {:.0f} C; a run needs {:.0f} C of room '
                  '-- let it cool below {:.0f} C'.format(
                      t, ' (DSP)' if source == 'DSP' else '', limits.max_temp_c, TEMP_START_MARGIN_C,
                      limits.max_temp_c - TEMP_START_MARGIN_C), fatal=True)
    # The user polarity is 0x607E (DS402: 0x00, or 0xC0 to invert), which
    # the firmware only takes in those two values.  Firmware before stm32
    # 8a366c6 kept a +-1 sign at 0x3013,3 instead, refused every closed-loop
    # mode (fault 1) while it was not +-1, and ignored 0x607E -- which a CSV
    # round-trip still carries (a P4-16 read 0xAB), and which a 0x607E
    # firmware's CSV load would then refuse.
    upol = _read_i8(dev, 0x3013, 3)
    pol = _read_u8(dev, 0x607E, 0)
    if upol is not None:
        rep.check('user polarity', upol in (-1, 1),
                  '0x3013,3 is {}: the coast, drag and loop steps will be '
                  'refused until it is +1 or -1'.format(upol))
        if pol is not None:
            rep.check('0x607E polarity', pol in (0x00, 0xC0),
                      '0x607E is 0x{:02X} (unused by this firmware, but '
                      'exported to the CSV, and the polarity from stm32 '
                      '8a366c6 on): set it to 0x00, or 0xC0 to match a '
                      '0x3013,3 of -1'.format(pol))
    elif pol is not None:
        rep.check('user polarity', pol in (0x00, 0xC0),
                  '0x607E is 0x{:02X}: 0x00, or 0xC0 to invert'.format(pol))
    require_isense(dev, rep)
    dev._require_capture_support()
    rep.check('voltage stream', dev._has_volt_stream(),
              'firmware streams 0x3004,13-14 (the coast needs it)',
              fatal=True)


def current_floor(r_ohm, rs=None, counts_per_a=None):
    """(A, why): the least test current that measures well, from the R
    sweep *rs* (its knee, drop and counts per amp) and/or *counts_per_a*;
    (0, None) when nothing is known."""
    from . import hbridge, probe
    rs = rs or {}
    cpa = counts_per_a or rs.get('counts_per_a')
    floors = []
    if cpa:
        floors.append((probe.SENSE_FLOOR_COUNTS / cpa, 'sense {:.0f} counts'
                       .format(probe.SENSE_FLOOR_COUNTS)))
    if rs.get('i0_a'):
        floors.append((hbridge.knee_bias_a(rs['i0_a']),
                       'dead-time knee (i0 {:.2f} A)'.format(rs['i0_a'])))
    elif (rs.get('vdt_v') and r_ohm and not rs.get('deadtime_degenerate')
          and (rs.get('fit') or {}).get('se', {}).get('vdt_v', 0.0)
          <= 0.25 * rs['vdt_v']):
        # a drop the sweep could not separate from R is no floor
        floors.append((TEST_IR_OVER_VDT * rs['vdt_v'] / r_ohm,
                       'I R {:.0f} x Vdt {:.2f} V'.format(TEST_IR_OVER_VDT,
                                                          rs['vdt_v'])))
    return max(floors) if floors else (0.0, None)


def floor_test_current(i_a, r_ohm, i_peak_a, limits, floor, u_max_v=None):
    """(A, raised, capped_by): *i_a* raised to *floor* (A, why) within the
    caps; *capped_by* names the cap when the floor is past it."""
    floor_a, _ = floor
    if not floor_a or i_a >= floor_a:
        return i_a, False, None
    caps = [(0.5 * i_peak_a, 'i_peak / 2'),
            (math.sqrt(FLOOR_POWER_FACTOR * limits.test_power_w
                       / (1.5 * r_ohm)), '{:.0f} x test power'.format(
                           FLOOR_POWER_FACTOR))]
    if limits.max_current_a:
        caps.append((limits.max_current_a, 'current limit'))
    if u_max_v:
        caps.append((TEST_VOLTAGE_FRACTION * u_max_v / r_ohm, 'voltage'))
    cap, what = min(caps)
    if floor_a <= cap:
        return floor_a, True, None
    return max(i_a, cap), cap > i_a, what


def _stage_test_current(dev, limits, reboot, rep):
    v, c = rep.values, rep.ctx
    rep.stage = 'test current'
    u_max = c['s'].get('u_max_v')
    i_test = test_current(v['r_ohm'], c['s']['i_max_a'], limits, u_max)
    probe_cpa = (c.get('probe') or {}).get('counts_per_a')
    floor = current_floor(v['r_ohm'], c.get('rs'), probe_cpa)
    if floor[0] and i_test < floor[0]:
        i_new, raised, capped = floor_test_current(
            i_test, v['r_ohm'], c['s']['i_max_a'], limits, floor, u_max)
        rep.check('test current floor', capped is None,
                  '{:.2f} A under the {} floor {:.2f} A: {}'.format(
                      i_test, floor[1], floor[0],
                      'raised to it' if capped is None else
                      '{} at the {} cap ({:.2f} A)'.format(
                          'raised' if raised else 'kept', capped, i_new)))
        i_test = i_new
    v['test_current_a'] = c['i_test'] = i_test
    rep.check('test current', i_test >= limits.min_current_a,
              '{:.2f} A ({:.1f} W in the winding)'.format(
                  i_test, 1.5 * v['r_ohm'] * i_test ** 2))


def _stage_winding(dev, limits, reboot, rep):
    v, c = rep.values, rep.ctx
    rep.stage = 'poles'
    pp = v['pole_pairs']
    wr = dev.measure_winding(pole_pairs=pp, target_a=c['i_test'])
    rep.raw['winding'] = wr
    rep.check('pole count (whole revolution)', True,
              'travel {:.4f} rev per {} electrical cycles'.format(
                  wr['travel_ratio'], pp))
    rep.check('winding sweep spread', wr['spread_rms_deg']
              <= WINDING_SPREAD_WARN_DEG,
              '{:.1f} deg rms: cogging or load pulls the rotor off the '
              'field'.format(wr['spread_rms_deg']))
    v['exact_angle'] = bool(wr.get('exact_angle'))
    if v['exact_angle']:
        rep.check('commutation angle',
                  abs(wr.get('e_zero_error_deg', 0.0))
                  <= COMMUTATION_ERROR_WARN_DEG,
                  'the drive computes it exactly for {} pole pairs; e_zero '
                  '{} is {:+.2f} deg electrical from the zero'.format(
                      pp, wr['e_zero'], wr.get('e_zero_error_deg', 0.0)))
    else:
        rep.check('encoder divides into electrical cycles',
                  wr['commutation_error_deg'] <= COMMUTATION_ERROR_WARN_DEG,
                  '{} counts over {} pole pairs: this firmware truncates the '
                  'cycle to {} counts, so its angle saws +-{:.1f} deg '
                  'electrical over each revolution (e_zero {} centres it; '
                  'the physical zero is {}); firmware with 0x3013,8 is '
                  'exact'.format(v['enc_res'], pp, v['enc_res'] // pp,
                                 wr['commutation_error_deg'], wr['e_zero'],
                                 wr['e_zero_physical']))
    v['commutation_error_deg'] = wr['commutation_error_deg']
    v['e_zero'] = wr['e_zero']
    v['e_polarity'] = wr['e_polarity']
