"""Commissioning as a list of steps that can be run one at a time.

STEPS is the whole job in the order it must be done (RUNBOOK.md says why).
Each step has:

  - a ``present`` reader, which shows what the drive holds now, read back
    from the drive;
  - a ``run`` function, which measures or tunes, writes its result to the
    drive's RAM and returns a one-line verdict.

A Session runs any selection of them, always in STEPS order, and stops at
the first one that fails.  A step whose predecessors were not run this
session takes what it needs from the drive.  For example, L uses the
drive's rt when Resistance was left out, and the drag uses the drive's kt.
A new motor therefore needs every identify step once.  After that, any
single step can be repeated on its own.

Nothing is saved to NVM except by the Save step (one Save.All), and by the
steps that must save to take effect:

  - The board calibrations (bias, gain, settling, slope, offset fold)
    save themselves with Save.Single (p4core.calibrate, slope).
  - Poles, PWM frequency and the encoder filter are latched at boot: they
    are saved, and the drive is rebooted.

tui.py is the screen for this; the steps themselves need no terminal.
"""

import copy
import math

from . import cogging, commission, encoder, loops, pipeline
from .commission import Limits, SysidError, drive_units, write_od
from .drivecal import CalError
from p4core import firmware
from p4core.calibrate import read_friction, read_kt


class StepFailed(SysidError):
    pass


class Step:
    def __init__(self, key, group, title, present, run):
        self.key, self.group, self.title = key, group, title
        self.present, self.run = present, run


# ---------------------------------------------------------------------------
# Reading the drive
# ---------------------------------------------------------------------------
def _cal(dev, name):
    return dev.node.sdo['Calibration'][name].raw


def _p_bias(dev):
    from p4core import calibrate
    v = calibrate.read_values(dev.node)
    return 'alpha {:.1f}  beta {:.1f} ct'.format(v['alpha_bias'], v['beta_bias'])


def _p_gain(dev):
    from p4core import calibrate
    v = calibrate.read_values(dev.node)
    return 'beta {}  (alpha {})'.format(v['beta_gain'], v['alpha_gain'])


def _p_encoder(dev):
    from . import encfilter
    out = '{} ct/rev'.format(int(dev.node.sdo['EncoderConfig']['Resolution'].raw))
    try:
        out += ', filter {:#04x}'.format(encfilter.read_filter(dev.node))
    except Exception:
        pass
    return out


def _upload_i16(dev, index, sub):
    try:
        return int.from_bytes(dev.node.sdo.upload(index, sub)[:2], 'little',
                              signed=True)
    except Exception:
        return None


def _p_slope(dev):
    ka, kb = _upload_i16(dev, 0x3008, 7), _upload_i16(dev, 0x3009, 7)
    if ka is None:
        return 'n/a (firmware)'
    if not ka and not kb:
        return 'off'
    return 'kA {:+.4f}  kB {:+.4f}'.format(ka / 4096.0, kb / 4096.0)


def _p_fold(dev):
    a0, b0 = _upload_i16(dev, 0x3008, 8), _upload_i16(dev, 0x3009, 8)
    if a0 is None:
        return 'n/a (firmware)'
    return 'a0 {:+d}  b0 {:+d} mA'.format(a0, b0)


def _p_winding(dev):
    return 'e_zero {}  polarity {:+d}'.format(int(_cal(dev, 'e_zero')),
                                              int(_cal(dev, 'e_polarity')))


def _p_runout(dev):
    bins, _ = encoder.comp_layout(dev)
    if not bins:
        return 'not in this firmware'
    ks = [encoder._sdo_r(dev, encoder.OD_COMP, 3 + 3 * i, 'H')
          for i in range(bins)]
    ks = [k for k in ks if k]
    return '{}, bins {}'.format('on' if encoder.comp_active(dev) else 'off',
                                ', '.join('k={}'.format(k) for k in ks) or
                                'none')


def _p_cogging(dev):
    n, _, _ = cogging.layout(dev)
    if not n:
        return 'not in this firmware'
    t = cogging.read_table(dev)
    ks = ', '.join('k{}'.format(k) for _, k, _ in t['bins'] if k) or 'no bins'
    txt = '{}, {}'.format('on' if t['active'] else 'off', ks)
    if t['lead_us'] is not None:
        txt += ', lead {} us'.format(t['lead_us'])
    if t['fade_hz'] is not None:
        txt += ', fade {} Hz'.format(t['fade_hz'] or 'off')
    return txt


# ---------------------------------------------------------------------------
# The session: what the steps share
# ---------------------------------------------------------------------------
class _Reporter:
    """p4core's Reporter interface onto the session's log and abort."""

    interactive = False

    def __init__(self, session):
        self.s = session

    def status(self, text):
        self.s.log(text)

    def note(self, text):
        self.s.log(text)

    def warn(self, text):
        self.s.log('WARNING: ' + text)

    def progress(self, pct, text=None):
        if text:
            self.s.log(text)

    def confirm(self, question, default=False):
        return default

    def check_abort(self):
        self.s.dev._check_abort()

    def sleep(self, seconds):
        import time
        end = time.monotonic() + seconds
        while True:
            self.check_abort()
            left = end - time.monotonic()
            if left <= 0:
                return
            time.sleep(min(0.02, left))

    @property
    def aborted(self):
        return False


class Session:
    """One commissioning session on one drive.

    *dev* is a P4Drive (p4sysid's P4Sysid for its steps).  *reboot* restarts the drive (see
    commission.identify_motor); without it, a pole or PWM change stops with
    'reboot and run again'.  *skip_k*, *geared*, *max_vel_hz* and *delta*
    are the runout and velocity options from commission_motor.  *export_path* is
    where Export CSV writes (default: <model>-node<N>-<date>.csv here).
    """

    def __init__(self, dev, limits=None, reboot=None, skip_k=(), geared=False,
                 max_vel_hz=None, delta=None, log=None, export_path=None):
        self.dev = dev
        self.export_path = export_path
        self.limits = limits or Limits()
        self.reboot = reboot
        self.skip_k, self.geared, self.max_vel_hz = tuple(skip_k), geared, max_vel_hz
        self.delta = delta or loops.VEL_DELTA
        self.log = log or dev._log
        self.rep = commission._Report(dev)
        self.results = {}            # key -> {'status', 'detail', 'result'}
        self.unsaved = False
        self.last_export = None
        self._preflight_done = False

    def _quiet_bus(self):
        """Put every other node in PRE-OPERATIONAL, so none sends PDOs.
        Each drive answers SYNC with its TPDOs: at the 2 kHz SYNC of the
        fast-read steps three other drives fill a 1 Mbit/s bus, and this
        node's TPDO and SDO responses (higher COB-IDs) lose arbitration --
        the slope sweep timed out on its first angle with nodes 2/3/4 left
        OPERATIONAL (node 8, 2026-10-22).  One NMT command per other id;
        an absent id costs one frame."""
        node = getattr(self.dev, 'node', None)
        net = getattr(node, 'network', None)
        if net is None:
            return
        try:
            for nid in range(1, 128):
                if nid != node.id:
                    net.send_message(0, bytes([0x80, nid]))
        except Exception as exc:                 # noqa: BLE001 -- best effort
            self.log('could not quiet the other nodes: {}'.format(exc))
            return
        self.log('other nodes set PRE-OPERATIONAL (no PDOs while node {} '
                 'is calibrated)'.format(node.id))

    @property
    def values(self):
        return self.rep.values

    @property
    def ctx(self):
        return self.rep.ctx

    # -- seeding: a value from this session, else the drive's -------------
    def _scaling(self):
        if 's' not in self.ctx:
            self.ctx['s'] = self.dev.refresh_scaling()
            self.values.setdefault('enc_res', self.ctx['s']['enc_res'])
            self.values.setdefault('i_peak_a', self.ctx['s']['i_max_a'])
        return self.ctx['s']

    def provisional_currents(self):
        """commission.provisional_i_cont, once a session: before the first
        step that sizes a current off i_cont/i_peak (Settling's top level,
        Resistance's sweep cap and the test current), not at Current PI --
        a drive with no CSV has i_peak 0 until then."""
        if not self.ctx.get('currents_provisional'):
            commission.provisional_i_cont(self.dev, self.ctx, self.limits)
            self.ctx.pop('s', None)
            self.ctx['currents_provisional'] = True

    def _preflight(self):
        if not self._preflight_done:
            self.provisional_currents()
            # Its temperature gate is fatal, and the first step to need it
            # may be a cold one (PWM) that skipped the wait.
            commission.wait_until_cool(self.dev, self.limits, self.log)
            commission._stage_preflight(self.dev, self.limits, self.reboot,
                                        self.rep)
            self._preflight_done = True

    def motor_limits(self):
        """The session's limits, with the current capped by the winding's
        own bound: the drive's i_cal when the winding probe set it below
        its CAL_I_MAX_A cap (a P4-16's 0.45 A, where the 10 W I^2 R budget
        would hold 0.78 A).  At the cap, the probe found no motor bound."""
        from . import probe
        try:
            i_cal = _cal(self.dev, 'i_cal') / 1000.0
        except Exception:
            return self.limits
        if not 0 < i_cal < probe.CAL_I_MAX_A:
            return self.limits
        if self.limits.max_current_a and self.limits.max_current_a <= i_cal:
            return self.limits
        lim = copy.copy(self.limits)
        lim.max_current_a = i_cal
        return lim

    def need(self, *keys):
        """Make sure each of *keys* is in values/ctx, from the drive if this
        session has not measured it.  A value the drive does not have either
        is a StepFailed naming the step that gives it."""
        v, c = self.values, self.ctx
        s = self._scaling()
        for key in keys:
            if key in v or key in c:
                continue
            if key == 'bus':
                c['bus'] = self.dev.read_bus_v()
            elif key == 'r_ohm':
                rt = int(_cal(self.dev, 'rt'))
                if rt <= 0:
                    raise StepFailed('the drive has no R: run Resistance')
                v['r_ohm'] = firmware.rt_ohm(self.dev.node, rt) / 2.0
            elif key == 'i_test':
                self.need('r_ohm')
                commission._stage_test_current(self.dev, self.motor_limits(),
                                               self.reboot, self.rep)
            elif key in ('ld_h', 'lq_h'):
                lt = int(_cal(self.dev, 'lt'))
                lq = int(_cal(self.dev, 'Lq')) if firmware.has(
                    self.dev.node, 'lq') else 0
                if lt <= 0:
                    raise StepFailed('the drive has no L: run Inductance')
                v['ld_h'] = firmware.lt_h(self.dev.node, lt) / 2.0
                v['lq_h'] = firmware.lt_h(self.dev.node, lq or lt) / 2.0
            elif key == 'pole_pairs':
                if s['poles'] < 2:
                    raise StepFailed('the drive has no pole count: run Poles')
                v['pole_pairs'] = s['poles'] // 2
            elif key in ('e_zero', 'e_polarity', 'exact_angle'):
                v['e_zero'] = int(_cal(self.dev, 'e_zero'))
                v['e_polarity'] = int(_cal(self.dev, 'e_polarity')) or 1
                v['exact_angle'] = bool(self.dev.exact_angle())
            elif key == 'pwm':
                c['pwm'] = s['pwm_freq_hz']
            elif key == 'budget':
                c['budget'] = self.dev.voltage_budget()
                c['one_sided'] = bool(c['budget'] and c['budget']['one_sided'])
            elif key in ('kt_nm_per_a', 'ke_v_s_per_rad', 'psi_wb'):
                kt = read_kt(self.dev.node) / 1000.0
                if kt <= 0:
                    raise StepFailed('the drive has no Kt: run Back-EMF')
                self.need('pole_pairs')
                v['kt_nm_per_a'] = kt
                v['ke_v_s_per_rad'] = kt / (1.5 * math.sqrt(2.0))
                v['psi_wb'] = v['ke_v_s_per_rad'] / v['pole_pairs']
                # Not measured here: a template's kt reads the same.
                c['ke_from_drive'] = True
            elif key == 'j_kg_m2':
                j = float(_cal(self.dev, 'j')) / 32768.0 * 1e-7
                if j <= 0:
                    raise StepFailed('the drive has no J: run Inertia')
                v['j_kg_m2'] = j
            elif key == 'coast':
                # No coast this session: the drag's J prior is the drive's.
                if int(_cal(self.dev, 'j')) > 0:
                    self.need('j_kg_m2')
                c['coast'] = {'j_kg_m2': v.get('j_kg_m2')}
            elif key == 'no_load':
                self.need('ke_v_s_per_rad')
                c['no_load'] = commission.speed_limits(v['ke_v_s_per_rad'],
                                                       s['v_nom'])
            else:
                raise KeyError(key)

    def need_friction(self):
        """The drag's Tc/B: this session's, or the drive's (0x3011,17-18)."""
        v = self.values
        if v.get('tc_nm') is not None:
            return
        f = read_friction(self.dev.node)
        if f is None:
            raise StepFailed('this firmware stores no Tc (0x3011,17, stm32 '
                             '898910f): run Inertia + friction (drag) in this '
                             'session')
        if f[0] <= 0:
            raise StepFailed('the drive has no Tc: run Inertia + friction '
                             '(drag)')
        v['tc_nm'], v['b_nm_s_per_rad'] = f[0] / 1e3, f[1] / 1e3
        self.ctx['friction_from_drive'] = True

    def stage(self, fn, limits=None):
        fn(self.dev, limits or self.limits, self.reboot, self.rep)

    def write(self, *keys):
        write_od(self.dev, drive_units(self.values), keys)
        self.unsaved = True

    # -- running ----------------------------------------------------------
    def run(self, keys, on_update=None):
        """Run the steps in *keys*, in STEPS order, stopping at the first
        failure.  *on_update(key)* is called before and after each step.
        Returns True when every one passed."""
        on_update = on_update or (lambda key: None)
        todo = [st for st in STEPS if st.key in set(keys)]
        self.selected = {st.key for st in todo}
        self.rerun = []
        for st in todo:
            self.results[st.key] = {'status': 'pending', 'detail': ''}
        saved_temp = self.dev.max_temp_c
        self.dev.max_temp_c = self.limits.max_temp_c
        self._preflight_done = False
        self._quiet_bus()
        ok = True
        i = 0
        try:
            while i < len(todo):
                st = todo[i]
                i += 1
                self.results[st.key] = {'status': 'running', 'detail': ''}
                on_update(st.key)
                self.log('=== {} ==='.format(st.title))
                n_checks = len(self.rep.checks)
                why = unavailable(self.dev.node, st.key)
                if why:
                    self.results[st.key] = {'status': 'skipped', 'detail': why}
                    self.log('{}: skipped: {}'.format(st.title, why))
                    on_update(st.key)
                    continue
                try:
                    if st.key not in OPEN_LOOP_STEPS:
                        commission.require_isense(self.dev)
                    if st.key not in COLD_STEPS:
                        commission.wait_until_cool(self.dev, self.limits,
                                                   self.log)
                    detail = st.run(self) or ''
                    warns = [c for c in self.rep.checks[n_checks:]
                             if c['severity'] == 'warn']
                    status = 'warn' if warns else 'pass'
                    if warns:
                        detail = '{}  [warn: {}]'.format(
                            detail, ', '.join(c['name'] for c in warns))
                    self.results[st.key] = {'status': status, 'detail': detail}
                except Exception as exc:      # noqa: BLE001 -- shown, not hidden
                    ok = False
                    self.results[st.key] = {'status': 'fail',
                                            'detail': str(exc)}
                    self.log('{}: FAILED: {}'.format(st.title, exc))
                    try:
                        self.dev.safe_stop()
                    except Exception:
                        pass
                on_update(st.key)
                if ok and self.rerun:
                    # A step changed what earlier ones measured at (the PWM
                    # rate): run those again, next.
                    todo[i:i] = [s for s in STEPS if s.key in self.rerun]
                    self.rerun = []
                if not ok:
                    for rest in todo[i:]:
                        self.results[rest.key] = {'status': 'skipped',
                                                  'detail': 'after a failure'}
                        on_update(rest.key)
                    break
        finally:
            self.dev.max_temp_c = saved_temp
        return ok


# ---------------------------------------------------------------------------
# The steps
# ---------------------------------------------------------------------------
#: The linearity sweep's travel under this fraction of a revolution: the
#: encoder is not reading the rotor.
LIN_DEAD_RATIO = 0.05


def _encoder_filter(s):
    """The hardware filter for this sensor (encfilter): a change is
    saved and the drive rebooted, since firmware writes it only at boot.
    Returns (plan, note)."""
    from p4core.calibrate import save_single
    from . import encfilter
    p = encfilter.plan(s.dev.node)
    if p['target'] is None:
        s.rep.check('Encoder filter', True, p['reason'])
        return p, 'filter {} (encoder not known)'.format(
            encfilter._hex(p['current']))
    detail = '{} {}: {} = {:#04x}, {}; {:.1f} deg at {:.0f} Hz'.format(
        p['model'], p['encoder'], p['label'], p['target'], p['reason'],
        p['lag_deg'], encfilter.CROSSOVER_HZ)
    if p['change']:
        if s.unsaved:
            raise StepFailed('the encoder filter needs a reboot and this '
                             'session has unsaved values: run Save first')
        encfilter.apply(s.dev.node, p, save_single)
        if s.reboot is None:
            raise StepFailed('encoder filter {:#04x} -> {:#04x} saved; it '
                             'takes effect at boot -- reboot the drive and run '
                             'again'.format(p['current'], p['target']))
        s.log('encoder: filter {:#04x} -> {:#04x}, rebooting'.format(
            p['current'], p['target']))
        node = s.reboot()
        if node is not None:
            s.dev.node = node
        s.ctx.pop('s', None)
        s._preflight_done = False
        p['running'] = encfilter.read_running(s.dev.node)
        detail += ' (was {:#04x}; saved, rebooted)'.format(p['current'])
    if p['running'] is not None and p['running'] != p['target']:
        raise StepFailed('the sensor runs filter {:#04x}, not {:#04x} '
                         '(0x3013,9)'.format(p['running'], p['target']))
    s.rep.check('Encoder filter', True, detail)
    return p, 'filter {:#04x}{}'.format(
        p['target'], ' (was {:#04x})'.format(p['current']) if p['change'] else '')


def _r_encoder(s):
    """The hardware filter for the sensor (encfilter: saved and rebooted when
    it changes) and the encoder at rest (p4core test_encoder, against the
    filter's own noise).  It drives no current, so it runs first, on a blank
    board; the linearity sweep is its own step, after Pole pairs."""
    from p4core import firmware
    from p4core.calibrate import ENCODER_MAX_JITTER
    from . import encfilter, enctest
    rep = _Reporter(s)
    if firmware.legacy(s.dev.node):
        # 4.3.x reads neither the sensor's chip nor its running filter
        # (0x3013,9/12): test it at rest against the generic limit.
        r = enctest.test_encoder(s.dev.node, rep, limit=ENCODER_MAX_JITTER)
        return 'spread {} ct (limit {}; filter not checked on firmware {})'.format(
            r['spread'], ENCODER_MAX_JITTER, firmware.detect(s.dev.node).text)
    p, fnote = _encoder_filter(s)
    s.ctx['encoder_filter'] = p
    limit = encfilter.spread_limit(p['encoder'], p['target'],
                                   ENCODER_MAX_JITTER)
    r = enctest.test_encoder(s.dev.node, rep, limit=limit)
    return '{}, spread {} ct (limit {})'.format(fnote, r['spread'], limit)


def _clear_board_corrections(s):
    """When the slope is re-measured this run, clear it and the drive-gated
    offset before bias and gain, as puckutility's Calibrate All does: the
    firmware applies both under drive, so stale ones bias the measurement."""
    if 'slope' not in s.selected or s.ctx.get('board_cleared'):
        return
    if unavailable(s.dev.node, 'slope'):    # 4.3.x: no slope to clear
        return
    from p4core import calibrate
    from . import drivecal
    calibrate.clear_slope(s.dev.node)
    drivecal.clear_offset_reg(s.dev.node)
    s.ctx['board_cleared'] = True


def _current_sense(session, which):
    from p4core import calibrate
    _clear_board_corrections(session)
    rep = _Reporter(session)
    try:
        if which == 'bias':
            r = calibrate.calibrate_bias(session.dev.node, rep)
        else:
            # A probe that raised i_cal to the sense floor shortens Gain's
            # per-step dwell to keep the same I^2Rt (the winding probe's dwell_scale).
            dwell_scale = session.ctx.get('probe', {}).get('dwell_scale', 1.0)
            r = calibrate.calibrate_gain(session.dev.node, rep,
                                         dwell_scale=dwell_scale)
    finally:
        calibrate.deenergise(session.dev.node, sleep=rep.sleep)
    if not r.get('ok'):
        raise StepFailed('{} out of bounds'.format(which))
    if which == 'bias':
        return 'alpha {:.1f}, beta {:.1f} ct (saved)'.format(r['alpha_bias'],
                                                             r['beta_bias'])
    return 'saved'


def _r_bias(s):
    return _current_sense(s, 'bias')


def _r_gain(s):
    return _current_sense(s, 'gain')


def _r_slope(s):
    """Current-proportional current-sense offset, 0x3008,7 / 0x3009,7
    (slope).  One retry over SDO if the PDO read fails.  A fit too
    noisy to trust (poor R^2, or the slope within noise of zero) stores 0
    instead of failing -- that is a pass, with a note."""
    from p4core.reporter import Aborted
    from . import slope
    from .device import SysidAborted
    _clear_board_corrections(s)
    rep = _Reporter(s)
    # A probe that raised i_cal to the sense floor shortens this sweep too
    # (quick, 4 levels) to hold the same I^2Rt (the winding probe's dwell_scale).
    dwell_scale = s.ctx.get('probe', {}).get('dwell_scale', 1.0)
    try:
        r = slope.calibrate_current_slope(s.dev.node, rep,
                                          dwell_scale=dwell_scale)
    except (Aborted, SysidAborted):
        raise
    except Exception as exc:                     # noqa: BLE001 -- retried
        s.log('slope: {}; retrying over SDO'.format(exc))
        r = slope.calibrate_current_slope(s.dev.node, rep, force_sdo=True,
                                          dwell_scale=dwell_scale)
    s.ctx['slope'] = r
    if r.get('skipped'):
        s.rep.check('Slope', False, 'skipped: ' + r['skipped'])
        return 'skipped ({})'.format(r['skipped'])
    if r.get('zeroed'):
        return ('kA {:+.4f}, kB {:+.4f} (saved): fit too noisy to trust '
                '(R^2 alpha {:.2f} beta {:.2f}) -- stored as 0, correction '
                'left off'.format(r['kA'], r['kB'], r['r2A'], r['r2B']))
    if not r['stored']:
        s.rep.check('Slope', False, 'fit not stored, slope left off: '
                    + '; '.join(r.get('rejected') or ()))
        return 'not stored (slope off)'
    return 'kA {:+.4f}, kB {:+.4f} (saved)'.format(r['kA'], r['kB'])


def _r_fold(s):
    """The slope sweep's drive-on baseline into the drive-gated offset
    register (slope.fold_baseline_offset).  Needs the slope sweep
    from this run.  With no slope term in effect (stored as 0, or not stored
    because the offset does not track load) it folds the mean offset; only a
    sweep that failed its gain/magnitude sanity gate is refused."""
    from . import slope
    sl = s.ctx.get('slope')
    if not sl or sl.get('skipped'):
        raise StepFailed('needs the slope sweep from this run: select Slope '
                         'too')
    try:
        r = slope.fold_baseline_offset(s.dev.node, sl, _Reporter(s))
    except CalError as exc:
        raise StepFailed(str(exc))
    return 'a0 {:+.0f}, b0 {:+.0f} mA ({}) -> {} (saved)'.format(
        r['a0'], r['b0'], 'mean offset, no slope term' if r.get('basis') == 'mean'
        else 'fit intercept', r['path'])


def _r_winding_legacy(s):
    """Electrical zero on firmware 4.3.x: p4core's field sweep (one
    revolution each way at i_cal, by SDO, no capture), written to RAM like
    the 4.5.0 step; Save stores it."""
    from p4core import calibrate, ezero
    rep = _Reporter(s)
    node = s.dev.node
    try:
        node.sdo['ControlWord'].raw = 0x80
        r = ezero.sweep(node, progress=lambda f: rep.progress(95 * f),
                        sleep=rep.sleep)
    except ezero.EZeroError as exc:
        raise StepFailed('electrical zero: {}'.format(exc))
    finally:
        calibrate.deenergise(node, sleep=rep.sleep)
    node.sdo['Calibration']['e_polarity'].raw = r['e_polarity']
    node.sdo['Calibration']['e_zero'].raw = r['e_zero']
    s.values['e_zero'], s.values['e_polarity'] = r['e_zero'], r['e_polarity']
    s.unsaved = True
    return 'e_zero {}, polarity {:+d} (field sweep, scatter +-{:.1f} deg)'.format(
        r['e_zero'], r['e_polarity'], r['spread_max_deg'])


def _r_winding(s):
    from p4core import firmware
    if firmware.legacy(s.dev.node):
        return _r_winding_legacy(s)
    s._preflight()
    s.need('i_test', 'pole_pairs')
    s.stage(commission._stage_winding)
    s.write('e_zero', 'e_polarity')
    return 'e_zero {}, polarity {:+d}'.format(s.values['e_zero'],
                                              s.values['e_polarity'])


def _r_runout(s):
    s._scaling()
    r = encoder.calibrate_runout(s.dev, skip_k=s.skip_k, geared=s.geared)
    s.unsaved = True
    s.ctx['runout'] = r
    if not r['passed']:
        # Off is safe: the encoder runs as it did.  On a tiny rotor the sweep
        # is the drag's friction lag, not the encoder (a P4-16: 1.3 deg of
        # stick-slip at 0.55 A, the 'runout' changing from sweep to sweep).
        s.rep.check('runout retest', False,
                    'the compensated retest did not improve ({:.2f} -> {:.2f} '
                    'ct rms): compensation left off.  If the uncompensated '
                    'error does not repeat, friction lag swamps it'.format(
                        r['fit']['error_rms_ct'],
                        r['retest']['fit']['error_rms_ct']))
        return 'compensation off ({:.2f} ct rms, not improved)'.format(
            r['fit']['error_rms_ct'])
    if not r['bins']:
        return '{:.2f} ct rms, nothing worth a bin'.format(
            r['fit']['error_rms_ct'])
    detail = '{:.2f} ct rms; {}'.format(
        r['fit']['error_rms_ct'],
        ', '.join('k={} {:.2f}->{:.2f}'.format(b['k'], b['before_ct'],
                                              b['after_ct'])
                  for b in r['retest']['bins']) if r.get('retest') else
        ', '.join('k={}'.format(k) for k, _, _ in r['bins']))
    # Re-verify linearity on the compensated position with the same sweep
    # and metric as the Linearity step (field-locked lag/cogging set
    # aside, only the encoder's orders judged).  The retest's own peak is not that metric: it keeps what the
    # table leaves out on purpose (skip_k, multiples of the pole pairs), so
    # it reads worse than the linearity step did before any table.
    if r.get('retest'):
        from . import enctest
        lin = enctest.test_encoder_linearity(s.dev.node, _Reporter(s),
                                             position='corrected')
        if lin['travel_ratio'] < LIN_DEAD_RATIO or not lin['poles_ok']:
            # The rotor did not follow the field this time (it once moved
            # 0.27 rev and read -2489 deg): not a linearity figure.
            s.rep.check('Encoder linearity (after runout)', False,
                        'the rotor did not follow the field ({:.2f} of a '
                        'revolution): re-run the Linearity step'.format(
                            lin['travel_ratio']))
            return detail + ', linearity re-check: rotor did not follow'
        s.rep.check('Encoder linearity (after runout)', lin['passed'],
                    '{:.1f} deg past {:.1f} after compensation: check the '
                    'magnet and the encoder mounting'.format(
                        lin['max_err'], lin['threshold']))
        detail += ', encoder linearity after {:.1f} deg'.format(lin['max_err'])
        if lin.get('ripple_rms') is not None:
            detail += ' (rotor ripple {:.1f} deg rms, not judged)'.format(
                lin['ripple_rms'])
    return detail


def _r_cogging(s):
    """Cogging feed-forward (cogging.calibrate): bins from slow spins
    both ways, the lead scanned from the latency budget (1-2.2x, extended
    while the best is at the edge), the fade from the current loop's
    bandwidth, kept only if the A/B shows it.  Bad stream frames are dropped
    before the fit; too many fails the step."""
    if not cogging.layout(s.dev)[0]:
        return 'not in this firmware: skipped'
    s.need('pole_pairs')
    r = cogging.calibrate(s.dev, int(s.values['pole_pairs']), log=s.log)
    s.unsaved = True
    s.ctx['cogging'] = r
    if not r.get('passed'):
        s.rep.check('cogging verify', False, r.get('detail', 'not verified'))
    if r.get('dropped'):
        s.rep.check('cogging bins', False, 'dropped {}: no lower velocity '
                    'line with them'.format(', '.join(
                        'k{}'.format(b[1]) for b in r['dropped'])))
    scan = r.get('lead_scan') or []
    if scan and r.get('lead_us') == max(x[0] for x in scan) and \
            r['lead_us'] >= cogging._lead_at(r['budget_us'],
                                             cogging.LEAD_SCALE_MAX):
        s.rep.check('cogging lead', False, 'best at the scan limit ({} us, '
                    '{:.1f}x the budget): the velocity loop may be too '
                    'soft'.format(r['lead_us'],
                                  r['lead_us'] / r['budget_us']))
    if r.get('worse'):
        s.rep.check('cogging ripple', False, 'worse with it on at {} rpm: '
                    'lower the fade (0x302E,15)'.format(
                        ', '.join('{:.0f}'.format(x) for x in r['worse'])))
    return cogging.summary(r)


def _r_save(s):
    """Save.All; and a reboot when i_cont/i_peak changed, since i2t takes
    them only at boot."""
    pipeline.save_all(s.dev)
    s.unsaved = False
    if not s.ctx.get('i2t_changed'):
        return 'Save.All done'
    if s.reboot is None:
        s.rep.check('i2t', False, 'i_cont/i_peak changed: i2t takes them at '
                    'boot -- reboot the drive')
        return 'Save.All done; reboot for i2t'
    node = s.reboot()
    if node is not None:
        s.dev.node = node
    s.ctx.pop('s', None)
    s.ctx['i2t_changed'] = False
    s._preflight_done = False
    return 'Save.All done, rebooted (i2t takes i_cont/i_peak at boot)'


def _r_export(s):
    """The drive's configuration as a CSV: the shipped blank template for
    its model, filled with what the drive holds (p4core.config_csv), so the
    next config load does not undo the commissioning."""
    import datetime
    import os
    from p4core import config_csv
    template = config_csv.template_for(s.dev.node)
    path = s.export_path or '{}-node{}-{}.csv'.format(
        os.path.basename(template).replace('-Blank.csv', ''), s.dev.node.id,
        datetime.date.today().isoformat())
    _, count = config_csv.export_node(s.dev.node, path, template,
                                      _Reporter(s))
    s.last_export = path
    if s.unsaved:
        s.rep.check('Export', False, 'exported values that are only in RAM: '
                    'run Save too')
    return '{} values -> {}'.format(count, path)


def _p_export(dev):
    return ''


def _p_save(dev):
    return ''


#: The steps that drive no closed-loop current (phase-voltage mode, or
#: nothing), so they may run on an uncalibrated current sense: they are how
#: it gets calibrated.  Every other step needs bias and gain in the
#: firmware's window first (commission.require_isense).
OPEN_LOOP_STEPS = frozenset(('encoder', 'bias', 'probe', 'gain', 'save',
                             'export'))
#: Steps that put no current in the winding: they do not wait to cool.
COLD_STEPS = frozenset(('pwm', 'current_pi', 'save', 'export'))

#: The actuator calibration this package runs, in the order it must run.
#: The motor identification and loop tuning steps are not here: a model's
#: config CSV carries their results, and the tools that measure them add
#: their steps with register().
STEPS = (
    Step('encoder', 'Board', 'Encoder filter + rest test', _p_encoder,
         _r_encoder),
    Step('bias', 'Board', 'Current-sense bias', _p_bias, _r_bias),
    Step('gain', 'Board', 'Current-sense gain', _p_gain, _r_gain),
    Step('slope', 'Board', 'Current-sense slope', _p_slope, _r_slope),
    Step('fold', 'Board', 'Current-sense offset fold', _p_fold, _r_fold),
    Step('winding', 'Identify', 'Electrical zero + winding order', _p_winding,
         _r_winding),
    Step('runout', 'Encoder', 'Encoder runout table', _p_runout, _r_runout),
    Step('cogging', 'Tune', 'Cogging feed-forward', _p_cogging, _r_cogging),
    Step('save', 'Save', 'Save to NV (Save.All)', _p_save, _r_save),
    Step('export', 'Save', 'Export config CSV', _p_export, _r_export),
)
STEP_KEYS = tuple(st.key for st in STEPS)

#: What each step needs of the drive's firmware (p4core.firmware features).
#: Firmware 4.3.x has no logger or injection ('capture'), so it can run
#: only the steps listed with () -- bias and gain as they are, and the
#: encoder test and electrical zero in their 4.3.x form (_r_encoder,
#: _r_winding) -- plus Save.  Session.run skips a step whose needs are
#: unmet, with the reason, instead of failing; unavailable() says why
#: before anything runs, for the apps' menus.
_CAPTURE = ('capture',)
NEEDS = {
    'encoder': (), 'bias': (), 'gain': (), 'winding': (), 'save': (),
    'export': ('current_line',),
    'slope': _CAPTURE + ('isense_slope',),
    'fold': _CAPTURE + ('isense_slope',),
    'runout': _CAPTURE + ('runout',),
    'cogging': _CAPTURE + ('cogging',),
}


def register(steps, order, needs=None):
    """Add *steps* (Step objects) to STEPS, which is then *order* (every
    key, these and the ones already here, in run order).  *needs* adds to
    NEEDS; a step left out of it needs the capture logger.  p4sysid calls
    this for its identification and tuning steps."""
    global STEPS, STEP_KEYS
    by_key = {st.key: st for st in STEPS}
    by_key.update((st.key, st) for st in steps)
    unplaced = set(by_key) - set(order)
    if unplaced:
        raise ValueError('register: no place in the order for {}'.format(
            ', '.join(sorted(unplaced))))
    STEPS = tuple(by_key[k] for k in order if k in by_key)
    STEP_KEYS = tuple(st.key for st in STEPS)
    for st in steps:
        NEEDS.setdefault(st.key, _CAPTURE)
    NEEDS.update(needs or {})


def unavailable(node, key):
    """Why *node*'s firmware cannot run step *key*, or None if it can."""
    from p4core import firmware
    fw = firmware.detect(node)
    missing = fw.missing(*NEEDS.get(key, ()))
    if not missing:
        return None
    return 'needs firmware {} (this drive runs {}; missing {})'.format(
        firmware.RECOMMENDED, fw.text,
        ', '.join(firmware.FEATURES[m].label for m in missing))


def present(dev, step):
    """The step's value as the drive holds it now, or why it cannot say."""
    try:
        return step.present(dev)
    except Exception as exc:      # noqa: BLE001 -- a display, not a check
        return '? {}: {}'.format(type(exc).__name__, exc)
