"""slope.py — the current-sense offset calibrations: slope and baseline fold.

Moved from puckutility's calibrate menu (gui/cal_slope.py keeps only the
dialogs and the plots).  Each function takes a canopen RemoteNode and a p4core
Reporter, drives the motor in voltage-angle mode, and returns a result dict
with everything the caller shows or plots.  On any failure the drive is left
at ud = 0 and IDLE and the exception is re-raised; a refusal the library
decides itself raises drivecal.CalError.

  calibrate_current_slope  the current-proportional alpha/beta offset
                           (0x3008,7 / 0x3009,7, Q4.12), plus the drive-on
                           baseline a0/b0 that fold_baseline_offset applies
  fold_baseline_offset     a0/b0 into the drive-gated offset register
                           (0x3008,8 / 0x3009,8, fw v3+), else the bias

Order: run after a fresh, warm Bias and Gain.  The slope subtracts the stored
Bias as its 0-A reference, so a stale bias becomes a false intercept.
"""

import math
import time

import numpy as np

from p4core.cia402 import (CLEAR_FAULT, MODE_IDLE, MODE_PHASE_VOLTAGE_ANGLE, write_mode,
                           OP_ENABLED, SHUTDOWN)

from . import drivecal
from .drivecal import CalError
from p4core.calibrate import permille_base_ma

#: A valid gainfactor is ~4096 (Q4.12 = 1.0).
GAIN_MIN, GAIN_MAX = 1024, 16384

#: Fit-quality gates: below either, the measured slope is noise, not a real
#: term (nodes 3/4 saw R^2 as low as 0.03, varying run to run).  The fit is
#: still stored -- as 0 -- rather than failing the step.
SLOPE_R2_GATE = 0.5
SLOPE_SIGMA_GATE = 2.0

#: Load levels as fractions of i_cal: thorough and quick.
SLOPE_FRACS = (0.1, 0.2, 0.32, 0.5, 0.7, 0.85, 1.0)
SLOPE_FRACS_QUICK = (0.12, 0.3, 0.6, 1.0)

#: SYNC period (ms) of the PDO fast-read: 0.5 = 2 kHz.  The puck streamed
#: 1 kHz fine; 2 kHz is the next step.  0.25 (4 kHz) is possible but watch CAN
#: bus load / notifier-thread keep-up (frames would drop -> per-sample SDO
#: fallback = slower, not faster).
SLOPE_SYNC_MS = 0.5

UD_CAP = 16000


def _gain_ok(alpha_gf, beta_gf):
    return GAIN_MIN <= alpha_gf <= GAIN_MAX and GAIN_MIN <= beta_gf <= GAIN_MAX


def _imag(node, pm_base):
    """|i| (mA) from 0x3010,6 / 0x6078, per-mille of *pm_base* (permille_base_ma)."""
    i_d = node.sdo['Motor']['id'].raw / 1000.0 * pm_base
    i_q = node.sdo['CurrentFeedback'].raw / 1000.0 * pm_base
    return (i_d * i_d + i_q * i_q) ** 0.5


def _wait_settled(node, reporter, timeout=1.5, poll_s=0.06, need=3):
    """Wait for the rotor to actually STOP after a theta_e step.  If it is
    still moving its back-EMF modulates |I| and swamps the tiny alpha/beta
    offset.  Poll the raw encoder until it holds within 2 counts for *need*
    polls, or give up at *timeout*."""
    p0 = node.sdo['Encoder']['RawPosition'].raw
    t0 = time.monotonic()
    stable = 0
    while time.monotonic() - t0 < timeout:
        reporter.sleep(poll_s)
        p1 = node.sdo['Encoder']['RawPosition'].raw
        if abs(p1 - p0) <= 2:
            stable += 1
            if stable >= need:
                return
        else:
            stable = 0
        p0 = p1


def _energise(node):
    """Voltage-angle mode from IDLE, through a fault clear."""
    node.sdo['SetModeOfOperation'].raw = MODE_IDLE
    node.sdo['ControlWord'].raw = CLEAR_FAULT
    node.sdo['ControlWord'].raw = SHUTDOWN
    node.sdo['ControlWord'].raw = OP_ENABLED
    write_mode(node, MODE_PHASE_VOLTAGE_ANGLE)


def clear_slope(node):
    """Zero and save 0x3008,7 / 0x3009,7 (slope correction off).  Errors are
    ignored: older firmware has no such object."""
    try:
        node.sdo[0x3008][7].raw = 0
        node.sdo[0x3009][7].raw = 0
        drivecal.save_single(node, 0x3008, 7)
        drivecal.save_single(node, 0x3009, 7)
        return True
    except Exception:
        return False


def _theta_grid(n):
    return [int(round(-32768 + i * 65536.0 / n)) for i in range(n)]


def fit_line(x, y):
    """Degree-1 least squares: (slope, intercept, R^2)."""
    xa = np.asarray(x, dtype=float)
    ya = np.asarray(y, dtype=float)
    slope, icpt = np.polyfit(xa, ya, 1)
    yp = slope * xa + icpt
    sr = float(np.sum((ya - yp) ** 2))
    st = float(np.sum((ya - ya.mean()) ** 2))
    r2 = 1.0 - sr / st if st > 0 else 0.0
    return float(slope), float(icpt), r2


def slope_stderr(x, y, slope, icpt):
    """Standard error of the fitted slope: how much noise, not R^2, there is
    in it.  n-2 degrees of freedom; 0 if there are too few points to say."""
    xa = np.asarray(x, dtype=float)
    ya = np.asarray(y, dtype=float)
    n = len(xa)
    if n < 3:
        return 0.0
    resid = ya - (slope * xa + icpt)
    sxx = float(np.sum((xa - xa.mean()) ** 2))
    if sxx <= 0:
        return 0.0
    mse = float(np.sum(resid ** 2)) / (n - 2)
    return (mse / sxx) ** 0.5


def angle_spread_deg(rows):
    """Circular std (deg) of the per-level offset angle: a clean
    fixed-direction slope keeps it low.  *rows*: (|I|, off_a, off_b)."""
    angs = [math.atan2(r[2], r[1]) for r in rows]
    if not angs:
        return 0.0
    mean = math.atan2(sum(math.sin(a) for a in angs),
                      sum(math.cos(a) for a in angs))
    devs = [((math.degrees(a - mean) + 180.0) % 360.0) - 180.0 for a in angs]
    return (sum(d * d for d in devs) / len(devs)) ** 0.5


def fit_slope(rows, alpha_gf, beta_gf, i_cal):
    """The slope model and its store gates from the per-level sweep.

    off_alpha = a0 + kA*|I|,  off_beta = b0 + kB*|I|.  Returns a dict with
    kA, kB, a0, b0, r2A, r2B, slope_mag, direction_deg, angle_spread_deg,
    i_spread, base_mag, base_dir_deg, slope_at_ical, off_at_ical,
    slope_significant, gain_ok, slope_ok, effect_ok, r2_ok, sig_ok,
    quality_ok, zeroed, store, rejected.  When the other gates pass but the
    fit is too noisy to trust (R^2 below SLOPE_R2_GATE on either channel, or
    the slope is within SLOPE_SIGMA_GATE standard errors of zero), kA/kB are
    stored as 0 rather than failing the step (zeroed=True); kA_fit/kB_fit
    keep the raw fit for the log."""
    mi = [r[0] for r in rows]
    kA, a0, r2A = fit_line(mi, [r[1] for r in rows])
    kB, b0, r2B = fit_line(mi, [r[2] for r in rows])
    seA = slope_stderr(mi, [r[1] for r in rows], kA, a0)
    seB = slope_stderr(mi, [r[2] for r in rows], kB, b0)
    kA_fit, kB_fit = kA, kB
    slope_mag = (kA * kA + kB * kB) ** 0.5
    spread = angle_spread_deg(rows)
    # SANITY GATE: never store garbage.  A reset/bad gainfactor (e.g. after a
    # firmware flash) makes that channel read ~0, collapsing the |I| circle so
    # the measured "offset" equals the current -> kA ~= 1.0 with a perfect R^2.
    # Storing that cripples the FOC (subtracts ~100 % of |I|).  Refuse it.
    gain_ok = _gain_ok(alpha_gf, beta_gf)
    slope_ok = slope_mag <= 0.5        # an iSense offset can't be >50 % of I
    # REAL-EFFECT gate (assembly-agnostic, no re-measure: reuses this sweep):
    # a genuine current-proportional slope needs the offset to TRACK load.
    # Direct-drive collapses id/iq, so mean|I| is frozen (identical across
    # levels) and the offset direction is inconsistent (huge angle spread); a
    # geared puck's |I| spreads across the level range with a consistent
    # direction.  (Geared: |I| 187->452, spread 2.8 deg.  Direct-drive: |I|
    # ~149 flat, spread 101 deg.)  Both must hold to store.
    i_vals = mi
    i_spread = max(i_vals) - min(i_vals)
    effect_ok = i_spread > 0.25 * max(i_vals) and spread < 30.0
    rejected = []
    if not gain_ok:
        rejected.append(
            'gainfactor out of range (Alpha={}, Beta={}; ~4096 expected). Run '
            "'Current Sense Gainfactor' / a full calibration FIRST."
            .format(alpha_gf, beta_gf))
    if not slope_ok:
        rejected.append(
            'slope_mag={:.3f} absurd (offset ~= current -> a channel reads ~0,'
            ' almost always a bad gain).'.format(slope_mag))
    if not effect_ok:
        rejected.append(
            'NO REAL EFFECT: offset does not track load (|I| spread {:.0f} mA '
            'over levels, direction spread {:.0f} deg) — degenerate, e.g. '
            'direct-drive id/iq collapse. Not a geared assembly; slope left '
            'OFF.'.format(i_spread, spread))
    # QUALITY GATE: a poor fit (R^2 below SLOPE_R2_GATE on either channel) or
    # a slope within SLOPE_SIGMA_GATE standard errors of zero is noise, not a
    # real term -- nodes 3/4 saw R^2 as low as 0.03, varying run to run.
    # Store 0 instead of the fit; this is a pass, not a rejection.
    r2_ok = r2A >= SLOPE_R2_GATE and r2B >= SLOPE_R2_GATE
    sig_ok = (abs(kA) >= SLOPE_SIGMA_GATE * seA
             or abs(kB) >= SLOPE_SIGMA_GATE * seB)
    quality_ok = r2_ok and sig_ok
    store = gain_ok and slope_ok and effect_ok
    zeroed = store and not quality_ok
    if zeroed:
        kA, kB = 0.0, 0.0
        slope_mag = 0.0
    slope_at_ical = slope_mag * i_cal          # mA the SLOPE term removes
    return dict(
        kA=kA, kB=kB, kA_fit=kA_fit, kB_fit=kB_fit, a0=a0, b0=b0,
        a0_flat=sum(r[1] for r in rows) / len(rows),
        b0_flat=sum(r[2] for r in rows) / len(rows),
        r2A=r2A, r2B=r2B, seA=seA, seB=seB, slope_mag=slope_mag,
        direction_deg=math.degrees(math.atan2(kB, kA)) if slope_mag else 0.0,
        angle_spread_deg=spread, i_spread=i_spread,
        base_mag=(a0 * a0 + b0 * b0) ** 0.5,
        base_dir_deg=math.degrees(math.atan2(b0, a0)),
        slope_at_ical=slope_at_ical,
        off_at_ical=(((a0 + kA * i_cal) ** 2) + ((b0 + kB * i_cal) ** 2)) ** 0.5,
        slope_significant=slope_mag >= 0.005,  # >= 0.5 %/A: a real term
        gain_ok=gain_ok, slope_ok=slope_ok, effect_ok=effect_ok,
        r2_ok=r2_ok, sig_ok=sig_ok, quality_ok=quality_ok, zeroed=zeroed,
        store=store, rejected=rejected)


def calibrate_current_slope(node, reporter=None, force_sdo=False, quick=False,
                            dwell_scale=1.0):
    """Calibrate the current-PROPORTIONAL alpha/beta current-sense offset
    (Current Sense Slope).

    With Bias (0 A) and Gain (counts/mA) already calibrated, the P4-16 still
    shows a residual current-sense offset whose magnitude grows linearly with
    load and points in a fixed direction.  This sweeps a set of load
    currents, measures the current-vector OFFSET (displacement of the |I|
    circle's centre) via an inverse-Park mean at each level, then fits
        off_alpha = a0 + kA*|I|      off_beta = b0 + kB*|I|
    The slope coefficients kA, kB (mA offset per mA current) are the stored
    model; firmware subtracts kA*|I| from ialpha and kB*|I| from ibeta.
    slope_mag = sqrt(kA^2+kB^2) (~0.146 on the P4-16), direction =
    atan2(kB, kA).  Firmware sign-check: after enabling the correction,
    re-run -- the offset-vs-current sweep should collapse toward the residual
    baseline; flip both stored coefficient signs if it grows.

    *force_sdo* skips the PDO fast-read (the retry after a SYNC/SDO comms
    glitch).  *quick* sweeps 4 levels instead of 7.  *dwell_scale* below 1.0
    (a probe that raised i_cal to the current-sense floor: the winding probe)
    forces *quick* on, trading levels for time -- there is no single fixed
    dwell here to scale, as there is in Gain: the sweep's time is mostly
    settle-polling, not a fixed sleep.

    Returns a dict; ``stored`` says whether kA/kB were written and saved (the
    baseline fold must only use a stored fit).  ``skipped`` is None, or
    'firmware' (< 4.4.0: no 0x3008,7) or 'gainfactor' (out of range: the
    slope was cleared and nothing was driven).  After a sweep it also has
    rows [(mean|I|, off_a, off_b) mA], levels, i_cal, alpha_gf, beta_gf,
    pdo, sweep_s, written {(index, sub): value} and fit_slope's keys."""
    reporter = drivecal.reporter_or_default(reporter)
    if dwell_scale < 1.0 and not quick:
        quick = True
        reporter.note('Slope: i_cal is at the sense floor, so this sweep '
                      'runs the quick 4-level sweep to keep the winding\'s '
                      'I^2Rt budget.')
    node_sdo = node.sdo
    # Firmware < 4.4.0 has no slope-correction object (0x3008:7 / 0x3009:7):
    # skip the whole cal rather than drive the motor for a result it can't
    # apply -- same firmware gate as the mag/encoder compensation cal.
    if not drivecal.fw_at_least(node, 4, 4, 0):
        reporter.note('Current Sense Slope cal SKIPPED: needs firmware v4.4.0+ '
                      '(no 0x3008:7/0x3009:7 slope-correction object on older '
                      'firmware).')
        return dict(stored=False, skipped='firmware')
    fp = None
    try:
        pm_base = permille_base_ma(node)    # 0x6078 / 0x3010,6 are per-mille of this
        i_cal = node_sdo['Calibration']['i_cal'].raw
        alpha_gf = node_sdo['Alpha']['Gainfactor'].raw
        beta_gf = node_sdo['Beta']['Gainfactor'].raw
        base = dict(stored=False, skipped=None, i_cal=i_cal,
                    alpha_gf=alpha_gf, beta_gf=beta_gf)
        reporter.note('--- Current Sense Slope calibration (current-'
                      'proportional alpha/beta offset) ---')
        reporter.note('Gainfactors in effect: Alpha={}  Beta={}  (i_cal {} mA)'
                      .format(alpha_gf, beta_gf, i_cal))

        # NOTE: no gear-ratio gate — the REAL-EFFECT gate at store time
        # (fit_slope) handles every assembly with one cal, using the sweep we
        # ALREADY have: a geared puck's offset grows cleanly with load; a
        # direct-drive puck's id/iq collapse, so mean|I| is IDENTICAL across
        # levels and the offset direction is inconsistent -> gated out,
        # nothing stored, correction left OFF (it was cleared below).

        # EARLY ABORT: a valid gainfactor is ~4096 (Q4.12 = 1.0).  A
        # reset/garbage gain -- e.g. the calibrated gain is wiped by a
        # firmware flash and not restored by config -- makes that channel read
        # ~0, so the offset measurement is meaningless AND driving with a
        # stored bad slope is unsafe.  Clear any stored slope (disable the
        # correction) and bail BEFORE driving.
        if not _gain_ok(alpha_gf, beta_gf):
            reporter.warn("ABORT: gainfactor out of range (Alpha={}, Beta={}; "
                          "~4096 expected). Run 'Current Sense Gainfactor' or "
                          'a full calibration FIRST.'.format(alpha_gf, beta_gf))
            if clear_slope(node):
                reporter.note('cleared 0x3008:7 / 0x3009:7 to 0 (slope '
                              'correction disabled until a valid cal).')
            base['skipped'] = 'gainfactor'
            return base

        # Disable any active slope correction BEFORE measuring, so the sweep
        # sees the RAW alpha/beta offset -- not a signal the firmware is
        # already correcting.  Otherwise a re-run measures only the residual
        # and the stored coefficient drifts instead of converging.  This also
        # clears a stale/garbage stored value up front.
        clear_slope(node)
        # v3+: same reason -- the drive-gated offset is applied during THIS
        # cal's own drive and would self-corrupt the measurement / overshoot.
        drivecal.clear_offset_reg(node)

        # This cal is in practice run on geared assemblies (rotor held), so
        # it settles almost instantly — a short poll + 2-stable check is
        # enough; a longer one was pure per-angle overhead (24 angles x 5
        # levels).  If ever reused on a ringy rotor, raise these back.
        def settle():
            _wait_settled(node, reporter, poll_s=0.03, need=2)

        # PDO fast-read: stream id/iq at SYNC rate so we can average MANY
        # more samples/angle in the same wall-clock -> cleaner per-level
        # offset -> tighter, repeatable a0 (which the baseline fold depends
        # on).  FastPDO keeps the drive ENABLED during SYNC (RPDO1 async
        # prime) and captures via a raw callback (hardware-validated).  Set up
        # BEFORE energising so its PRE-OP/remap happens first; SDO fallback if
        # it can't come up.  read() -> [id, iq].
        if force_sdo:
            reporter.note('  Slope on SDO path (FastPDO skipped after a prior '
                          'comms glitch).')
        else:
            try:
                from .fast_pdo import FastPDO
                fp_try = FastPDO(node, sync_period_ms=SLOPE_SYNC_MS)
                fp_try.__enter__()
                if getattr(fp_try, 'ok', False):
                    fp = fp_try
                    reporter.note('  PDO fast-read active (id/iq stream @ {:g} '
                                  'ms SYNC = {:.0f} Hz).'.format(
                                      SLOPE_SYNC_MS, 1000.0 / SLOPE_SYNC_MS))
                else:
                    fp_try.__exit__(None, None, None)
                    reporter.note('  PDO setup failed ({}); using SDO.'.format(
                        getattr(fp_try, 'err', '?')))
            except Exception as exc:
                reporter.note('  PDO unavailable ({}); using SDO.'.format(exc))

        # Energise in voltage-angle mode; align+stop the rotor at theta=0
        # before ramping.
        _energise(node)
        node_sdo['Theta_e'].raw = 0
        # seed to pull the rotor to 0
        drivecal.seed_ud(node, lambda: _imag(node, pm_base), 0.5 * i_cal,
                         reporter.sleep)
        settle()

        # Angles (BIDIRECTIONAL: each read from both approach directions).
        # 8 uniform angles fully cancel the fundamental; with 48 clean PDO
        # samples each, fewer angles hold the offset quality while cutting the
        # settle count (settles dominate the sweep time).  SDO keeps 10 x 12.
        n_ang = 8 if fp else 10
        n_avg = 48 if fp else 12
        t_slope0 = time.monotonic()
        theta = _theta_grid(n_ang)

        def read_angle(th):
            # Settle at th (SYNC OFF), then stream n_avg id/iq (SYNC ON).
            # Theta_e set + settle are SDO — they MUST run SYNC-OFF, else they
            # collide with continuous SYNC (0x05040000/1).  Harmless on a held
            # (geared) rotor where settle returns at once, but FATAL on a
            # DIRECT-DRIVE rotor that polls RawPosition many times while
            # detenting (aborted the slope cal mid-sweep).
            node_sdo['Theta_e'].raw = th
            settle()
            if fp:
                fp.sync_on()                  # SYNC on ONLY for the burst
            sid = siq = 0.0
            for _ in range(n_avg):
                v = fp.read() if fp else None  # coherent [id, iq], else SDO
                if v is not None:
                    sid += v[0]
                    siq += v[1]
                else:
                    sid += node_sdo['Motor']['id'].raw
                    siq += node_sdo['CurrentFeedback'].raw
                    reporter.sleep(0.003)
                reporter.check_abort()
            if fp:
                fp.sync_off()                 # SYNC off before the next SDO
            return (sid / n_avg) / 1000.0 * pm_base, (siq / n_avg) / 1000.0 * pm_base

        def sweep_offset(level_no, n_levels):
            # One full-electrical-cycle sweep at the present ud; rotor stopped
            # each step.  Returns (mean|I|, off_alpha, off_beta) in mA -- the
            # current-vector offset, i.e. the displacement of the |I| circle's
            # centre, via an inverse-Park mean (the true rotating current
            # averages to zero, so the residual inverse-Park mean IS the fixed
            # sense offset).
            # FORWARD pass (theta increasing) then REVERSE pass (decreasing)
            # at the SAME angles: each angle is approached from BOTH
            # directions, so averaging the two cancels the friction/backlash
            # hysteresis (the rotor detents slightly off-angle by approach
            # direction) that wobbled kB/offset-direction run-to-run.
            def pct(k):
                return int(100.0 * (level_no + k / (2.0 * n_ang)) / n_levels)
            fwd = [None] * n_ang
            for k in range(n_ang):
                reporter.progress(pct(k), 'slope offset fwd {}/{}'.format(
                    k + 1, n_ang))
                fwd[k] = read_angle(theta[k])
            rev = [None] * n_ang
            for k in range(n_ang - 1, -1, -1):
                reporter.progress(pct(2 * n_ang - 1 - k), 'slope offset rev {}/{}'
                                  .format(n_ang - k, n_ang))
                rev[k] = read_angle(theta[k])
            idl = [0.5 * (fwd[i][0] + rev[i][0]) for i in range(n_ang)]
            iql = [0.5 * (fwd[i][1] + rev[i][1]) for i in range(n_ang)]
            mg = [(idl[i] * idl[i] + iql[i] * iql[i]) ** 0.5 for i in range(n_ang)]
            r = [theta[i] / 32768.0 * math.pi for i in range(n_ang)]
            mnv = (sum(mg) / n_ang) or 1.0
            oa = sum(idl[i] * math.cos(r[i]) - iql[i] * math.sin(r[i])
                     for i in range(n_ang)) / n_ang
            ob = sum(idl[i] * math.sin(r[i]) + iql[i] * math.cos(r[i])
                     for i in range(n_ang)) / n_ang
            return mnv, oa, ob

        # OFFSET vs CURRENT: sweep load levels; ud is carried across the
        # (increasing) levels so we don't re-ramp from zero each time.  The
        # offset saturates at high current (nonlinear via duty), so the levels
        # are WEIGHTED LOW: extra low-current points anchor the intercept a0
        # that the baseline fold consumes -- three near/below 0.35*i_cal pin
        # |I|=0 instead of extrapolating a long way down.  Sweep 0.1 -> 1.0x
        # i_cal (NOT above it): the fit only needs to cover the operating
        # point, and driving past i_cal just burns I^2*R (a big low-KV
        # winding is watts).
        # QUICK trims the sweep to 4 levels (still weighted LOW: two points
        # <= 0.35*i_cal pin a0, one mid + top define the slope).  A degree-1
        # fit stays well-determined at 4 points and every downstream
        # write/model/gate is unchanged — the ONLY difference is level COUNT.
        # Dropping ~3 levels is where the ~18.5 s sweep's time lives
        # (per-level ramp + settles).  beta R^2 is intrinsically noisy, so we
        # do NOT trim below 4.
        i_top = int(i_cal)   # HARD ceiling: never drive above the op point
        fracs = SLOPE_FRACS_QUICK if quick else SLOPE_FRACS
        levels = sorted(set(max(20, int(i_cal * f)) for f in fracs))
        rows = []
        ud = 500
        for n_lvl, lvl in enumerate(levels):
            node_sdo['Theta_e'].raw = 0
            node_sdo['Motor']['ud'].raw = ud
            settle()
            cur = _imag(node, pm_base) or 0.0
            # stop at the target OR at i_cal (whichever first) -- the i_cal
            # clamp guards the overshoot a geared rotor's back-EMF can
            # inflate, so we never ramp ud past the operating current.
            while cur < lvl and cur < i_top and ud < UD_CAP:
                ud += 300
                node_sdo['Motor']['ud'].raw = ud
                reporter.sleep(0.02)
                cur = _imag(node, pm_base) or 0.0
            settle()
            rows.append(sweep_offset(n_lvl, len(levels)))

        node_sdo['Motor']['ud'].raw = 0
        node_sdo['SetModeOfOperation'].raw = MODE_IDLE
        sweep_s = time.monotonic() - t_slope0
        reporter.note('  (slope sweep took {:.1f} s)'.format(sweep_s))
        pdo = fp is not None
        if fp:
            fp.__exit__(None, None, None)   # stop SYNC + restore callbacks
            fp = None

        fit = fit_slope(rows, alpha_gf, beta_gf, i_cal)
        result = dict(base, rows=rows, levels=levels, pdo=pdo,
                      sweep_s=sweep_s, written={}, **fit)
        _report_slope(reporter, result)
        if fit.get('zeroed'):
            reporter.note(
                '>>> fit too noisy to trust (R^2 alpha={:.2f} beta={:.2f}, '
                'kA={:+.4f}+-{:.4f} kB={:+.4f}+-{:.4f}): storing 0 instead of '
                'the fit -- correction left off, not a failure.'.format(
                    fit['r2A'], fit['r2B'], fit['kA_fit'], fit['seA'],
                    fit['kB_fit'], fit['seB']))

        if not fit['store']:
            reporter.warn('NOT STORING (slope left OFF — it was cleared at '
                          'the start of this cal):')
            for why in fit['rejected']:
                reporter.warn('  ' + why)
            return result
        # STORE (guarded: OD entries may not exist in firmware yet)
        ka_q = int(round(fit['kA'] * 4096))   # Q4.12, signed (I16)
        kb_q = int(round(fit['kB'] * 4096))
        try:
            node_sdo[0x3008][7].raw = ka_q
            node_sdo[0x3009][7].raw = kb_q
            drivecal.save_single(node, 0x3008, 7)
            drivecal.save_single(node, 0x3009, 7)
            result['written'] = {(0x3008, 7): ka_q, (0x3009, 7): kb_q}
            result['stored'] = True
            reporter.note('>>> stored to 0x3008:7 / 0x3009:7 (and saved to '
                          'EEPROM).')
        except Exception as exc:
            reporter.warn('firmware OD entries 0x3008:7/0x3009:7 not present '
                          'yet -- coefficients above; wire them once firmware '
                          'adds them. ({})'.format(exc))
        return result
    except BaseException:
        try:
            if fp:
                fp.__exit__(None, None, None)   # stop SYNC before anything
        except Exception:
            pass
        drivecal.deenergise(node)
        raise


def _report_slope(reporter, r):
    note = reporter.note
    note('--- per-level current-sense offset ---')
    note('   mean|I|(mA)   off_alpha(mA)   off_beta(mA)   angle(deg)')
    for row in r['rows']:
        note('   {:9.1f}     {:10.2f}     {:10.2f}     {:8.1f}'.format(
            row[0], row[1], row[2], math.degrees(math.atan2(row[2], row[1]))))
    # MODEL SUMMARY: off(|I|) = OFFSET + SLOPE·|I|.  Show BOTH terms so it's
    # clear which is doing the work: small motors (P4-16) carry a real
    # current-proportional SLOPE; large motors (P4-37) are ~flat and the fixed
    # drive-on OFFSET (a0/b0) is the whole correction.
    i_cal = r['i_cal']
    note('>>> current-sense offset model:  off(|I|) = OFFSET + SLOPE·|I|')
    note('>>>   SLOPE  (∝ current):    kA={:+.4f} kB={:+.4f}   |k|={:.4f} mA/mA '
         '({:.2f}%/A)   -> {:.1f} mA @{} mA   [{}]'.format(
             r['kA'], r['kB'], r['slope_mag'], r['slope_mag'] * 100.0,
             r['slope_at_ical'], i_cal,
             'SIGNIFICANT' if r['slope_significant']
             else '~flat, negligible on this motor'))
    note('>>>          stored Q4.12: kA*4096={:+d}  kB*4096={:+d}   -> 0x3008:7 '
         '/ 0x3009:7'.format(int(round(r['kA'] * 4096)),
                             int(round(r['kB'] * 4096))))
    note('>>>   OFFSET (fixed drive-on): a0={:+.2f} b0={:+.2f} mA   |{:.1f}| mA '
         '@{:+.0f}°   [{}]'.format(
             r['a0'], r['b0'], r['base_mag'], r['base_dir_deg'],
             'DOMINANT term here'
             if r['base_mag'] >= max(r['slope_at_ical'], 8.0) else 'secondary'))
    note('>>>          -> drive-gated register 0x3008:8 / 0x3009:8 (fw v3+), '
         'else bias fold')
    note('>>>   net offset at {} mA = {:.1f} mA   |   fit R² α={:.3f} β={:.3f}   '
         'dir-spread {:.1f}°'.format(i_cal, r['off_at_ical'], r['r2A'],
                                      r['r2B'], r['angle_spread_deg']))
    if r['angle_spread_deg'] > 20.0:
        reporter.warn('offset direction not consistent ({:.1f}° spread) -- not '
                      'a clean fixed-direction slope.'.format(
                          r['angle_spread_deg']))


def _i16le(v):
    return max(-32768, min(32767, int(round(v)))).to_bytes(2, 'little',
                                                            signed=True)


def baseline_of(slope):
    """(a0, b0, basis) the fold applies for a slope result; basis is 'fit'
    (the intercepts of a stored non-zero slope) or 'mean' (the mean offsets,
    no slope term in effect).  Raises CalError when there is nothing
    trustworthy to fold (see fold_baseline_offset)."""
    slope = slope or {}
    if slope.get('a0') is None or slope.get('b0') is None:
        raise CalError("Baseline fold: no slope result — run 'Current Sense "
                       "Slope' (or a full cal) first. Nothing changed.")
    stored = slope.get('stored')
    if stored and (slope.get('kA') or slope.get('kB')):
        return slope['a0'], slope['b0'], 'fit'
    sane = stored or (slope.get('gain_ok') and slope.get('slope_ok'))
    if not sane:
        raise CalError('Baseline fold: the last Current Sense Slope sweep '
                       'failed its sanity gate (gainfactor / absurd slope) — '
                       'refusing to fold its baseline. Fix the gain cal and '
                       're-run Slope. Nothing changed.')
    if slope.get('a0_flat') is None:          # a result from before a0_flat
        return slope['a0'], slope['b0'], 'fit'
    return slope['a0_flat'], slope['b0_flat'], 'mean'


def fold_baseline_offset(node, slope, reporter=None):
    """Apply the slope cal's fitted drive-on baseline (a0/b0, mA) so it is
    removed under drive.

    *slope* is calibrate_current_slope's result (or any dict with stored, a0,
    b0).  Which baseline:
      * a stored, non-zero slope: the fit's intercepts a0/b0 (the slope term
        removes the rest);
      * no slope term in effect (stored as 0 because the fit was too noisy,
        or not stored because the offset does not track load): the MEAN
        offsets a0_flat/b0_flat -- with kA = kB = 0 in the firmware the mean
        is what is left under drive;
      * the gain or slope-magnitude sanity gate failed (bad gainfactor): the
        sweep itself is garbage -> CalError, nothing changed.  Also CalError
        when there is no slope result.

    TWO PATHS, chosen by firmware capability (probed at run time):
      * v3+ (has 0x3008:8 / 0x3009:8): write a0/b0 to the DRIVE-GATED offset
        register.  The firmware subtracts it ONLY while driving
        (``iAlBe -= offset``), so there is NO idle phantom, Bias stays clean,
        and a later Bias re-cal doesn't cancel it.  The production fix.
      * pre-v3 (no register): LEGACY fold into the always-on iSense Bias.
        This still removes the offset under drive, but Bias is unconditional,
        so it injects a0/b0 as an equal/opposite error at TRUE idle, and a
        later Bias re-cal / full cal OVERWRITES it.  Diagnostic stopgap only;
        revert = re-run Current Sense Bias.

    Returns dict(path='register'|'bias', a0, b0, written={(idx, sub): v});
    the bias path adds old_bias, new_bias (alpha, beta) and dbias."""
    reporter = drivecal.reporter_or_default(reporter)
    a0, b0, basis = baseline_of(slope)
    if basis == 'mean':
        reporter.note('  no slope term in effect: folding the mean drive-on '
                      'offset (slope-0 baseline)')

    # Drive-gated offset register (firmware v3+).  a0/b0 are the measured
    # alpha/beta intercepts in mA; the firmware does `iAlBe -= offset`, so
    # store them as-measured (signed).  Probe the object raw (bypasses the
    # EDS): if present -> use it and SKIP the Bias fold; Bias stays clean.
    try:
        node.sdo.upload(0x3008, 8)   # SDO-aborts (0x06020000) on pre-v3 fw
        has_offset_reg = True
    except Exception:
        has_offset_reg = False
    if has_offset_reg:
        node.sdo.download(0x3008, 8, _i16le(a0))
        node.sdo.download(0x3009, 8, _i16le(b0))
        drivecal.save_single(node, 0x3008, 8)   # persist Alpha offset
        drivecal.save_single(node, 0x3009, 8)   # persist Beta offset
        reporter.note('=== Drive-gated iSense offset: a0={:+.2f} b0={:+.2f} mA '
                      '-> 0x3008:8 / 0x3009:8 ==='.format(a0, b0))
        reporter.note('  Firmware applies it ONLY while driving (no idle '
                      'phantom); saved to EEPROM. Bias left clean.')
        clamp = lambda v: max(-32768, min(32767, int(round(v))))  # noqa: E731
        return dict(path='register', a0=a0, b0=b0, basis=basis,
                    written={(0x3008, 8): clamp(a0), (0x3009, 8): clamp(b0)})

    # Legacy fallback (pre-v3 firmware, no register): fold into the
    # ALWAYS-ON Bias.  ma_per_ct from the Alpha shunt (0x3008:5) and the
    # designed gain (0x3008:4), per the firmware.
    shunt = float(node.sdo[0x3008][5].raw)
    gain = float(node.sdo[0x3008][4].raw)
    ma_per_ct = 3.3 / 4096.0 * 1000.0 / shunt * 1000.0 / gain * 1000.0
    gf_a = float(node.sdo[0x3008][6].raw)   # Gainfactor (Q4.12)
    gf_b = float(node.sdo[0x3009][6].raw)
    # dI/dBias = gainfactor/65536 * ma_per_ct (>0).  To SUBTRACT the +a0
    # offset from the driven reading, dBias = -a0 * 65536/(gainfactor *
    # ma_per_ct).  Per channel; Bias is Q12.4 counts.
    dbias_a = -a0 * 65536.0 / (gf_a * ma_per_ct)
    dbias_b = -b0 * 65536.0 / (gf_b * ma_per_ct)
    old_a = int(node.sdo['Alpha']['Bias'].raw)
    old_b = int(node.sdo['Beta']['Bias'].raw)
    new_a = int(round(old_a + dbias_a))
    new_b = int(round(old_b + dbias_b))
    reporter.note('=== Baseline Offset Fold: a0={:+.2f} b0={:+.2f} mA '
                  '(drive-on residual) ==='.format(a0, b0))
    reporter.note('  Alpha Bias {} -> {} ({:+.0f})   Beta Bias {} -> {} ({:+.0f})'
                  .format(old_a, new_a, dbias_a, old_b, new_b, dbias_b))
    node.sdo['Alpha']['Bias'].raw = new_a
    node.sdo['Beta']['Bias'].raw = new_b
    drivecal.save_single(node, 0x3008, 3)   # persist Alpha iSense
    drivecal.save_single(node, 0x3009, 3)   # persist Beta iSense
    # Over-corrects by a0/b0 at TRUE zero current (Bias is unconditional);
    # the firmware drive-gated offset is the clean fix.
    reporter.note('  applied + saved  (revert = re-run Current Sense Bias).')
    return dict(path='bias', a0=a0, b0=b0, basis=basis, old_bias=(old_a, old_b),
                new_bias=(new_a, new_b), dbias=(dbias_a, dbias_b),
                written={(0x3008, 3): new_a, (0x3009, 3): new_b})
