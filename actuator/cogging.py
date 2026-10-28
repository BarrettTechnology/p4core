"""Cogging / torque-ripple feed-forward (0x302E): fit, lead, fade, verify.

The firmware adds  i_ff = sum [a_s sin(k theta) + a_c cos(k theta)]  mA to the
q reference at the corrected encoder angle theta, read lead_us ahead
(theta + w lead), each bin faded out between fade Hz and twice that (stm32
pwm.h cog_comp_t).  This module finds those numbers for any motor:

  - Bins: the position-locked LOAD torque, d = Kt iq - J alpha, from slow
    velocity-mode spins both ways (the 1 kHz iq/position stream).  It is
    absolute -- the measured iq already holds whatever feed-forward runs --
    so a fit taken with the bins on refines the one taken with them off
    (node 4: converged in 2 passes, spread 0.4 -> 0.06 mNm).  A line is a
    bin only when it is the same in every run (both directions, both
    speeds) to many standard errors: that keeps out the sense offset (k =
    pole pairs, wanders run to run), slow low-order wander (k1-4 on node 4)
    and encoder error left over from 0x3027 (it reads as a load growing with
    speed squared).  The strongest that pass, up to the firmware's bin
    count.
  - Lead: the torque arrives late (encoder latency + the current loop +
    a control cycle, and more the budget does not count), so the bins are
    read at the angle the rotor will be at.  Scanned from that budget to
    2.2x, and on while the best is at the edge.
  - Bad frames: duplicate / out-of-order / jumping stream frames are
    dropped before the fit (clean_frames), and a revolution whose residual
    stands out is dropped after it; too many bad frames fails the capture.
  - Fade: one lead fits only inside the current loop's bandwidth; the bins
    fade out above FADE_PER_CURRENT_BW of it (node 4: 100 Hz at 234 Hz;
    150 Hz left k84 adding ripple at 120 rpm).
  - Verify: ff off vs on, angle-locked velocity ripple per speed (per-rev
    phasors, bootstrap standard error).  A bin that does not reduce its own
    velocity line is dropped.

Node 4 (P4-42, 7 pp, 12 slots), by hand 2026-10-16: k14, k12, k28, k84;
locked ripple at 30 rpm 28 -> 4.3 rpm.
"""

import math
import struct

import numpy as np

from . import encoder, steptest
from .commission import SysidError
from .device import LOG_SRC_IQ_POS, _Stream

OD_COG = 0x302E
SUB_ACTIVE, SUB_LEAD, SUB_FADE, SUB_UNIT = 1, 11, 15, 16
#: 0x302E,16 = 1: the bins are 0.1 mA (4.5.0 ba0c0c7 on); absent or 0,
#: whole mA.
UNIT_TENTH_MA = 1

#: Fit speeds (rpm): slow enough that encoder error (a load growing with
#: speed squared) stays small and every likely bin is far below the fade;
#: two of them so a speed-dependent line shows as run-to-run spread.
FIT_RPMS = (30.0, 60.0)
#: Verify speeds: the fit's, and one where high bins are fading.
VERIFY_RPMS = (30.0, 60.0, 120.0)
#: Whole revolutions captured each way at each speed.
REVS = 8
#: A line is a bin when its run-to-run mean is this many standard errors
#: from zero ...
MIN_Z = 4.0
#: ... and at least this much current: MIN_BIN_UNITS of the firmware's bin
#: unit (10 mA on whole-mA firmware, 1 mA on 0.1 mA), or MIN_BIN_FRAC of
#: i_peak, the larger (a P4-16's lines are 1-6 mA, a P4-42's 50-150).
MIN_BIN_UNITS = 10
MIN_BIN_FRAC = 1e-4
#: Highest harmonic fitted: enough for LCM(slots, 2 pp) on usual motors.
KMAX_PER_POLE_PAIR = 16
KMAX_FLOOR = 96
#: Fade start as a fraction of the current loop's bandwidth.
FADE_PER_CURRENT_BW = 0.43
#: Lead candidates: the latency budget times these.  The budget runs short
#: (node 4: 806 us against a hand optimum of 1500, ~1.9x -- the velocity
#: loop adds lag the budget does not count), so the scan is wide ...
LEAD_SCALES = (1.0, 1.4, 1.8, 2.2)
#: ... and keeps going a step at a time, up to LEAD_SCALE_MAX (or down to
#: LEAD_SCALE_MIN), while the best candidate is the one at the edge.
LEAD_SCALE_STEP = 0.4
LEAD_SCALE_MAX = 3.4
LEAD_SCALE_MIN = 0.6
#: Stream frames: a frame whose control-cycle count does not advance (a
#: duplicate), or advances more than this many times the usual step, or
#: whose position jumps more than POS_JUMP_X times the usual move, is
#: dropped (it once turned a 4 rpm ripple into 715 +- 1523).  More than
#: MAX_BAD_FRAC of them fails the capture.
MAX_GAP_X = 8.0
POS_JUMP_X = 8.0
MAX_BAD_FRAC = 0.02
#: A revolution whose velocity residual is this many times its run's median
#: is dropped from the analysis (a frame glitch the frame check missed).
REV_NOISE_X = 5.0
#: A bin is kept when it lowers its own velocity line by this many sigma.
KEEP_SIGMAS = 2.0
#: The verify fails when the slowest speed improves by less than this many
#: sigma, and warns when any speed is worse by more.
VERIFY_SIGMAS = 3.0
BOOTSTRAP = 200


# ---------------------------------------------------------------------------
# The OD
# ---------------------------------------------------------------------------
def _r(dev, sub, fmt):
    return struct.unpack('<' + fmt, bytes(dev.node.sdo.upload(OD_COG, sub)))[0]


def _w(dev, sub, fmt, value):
    dev.node.sdo.download(OD_COG, sub, struct.pack('<' + fmt, int(value)))


def layout(dev):
    """(bins, has lead, has fade) in this firmware's 0x302E; bins 0 when it
    has none.  Bins 0-2 at ,2-10, the lead at ,11, bin 3 at ,12-14, the fade
    at ,15."""
    try:
        count = _r(dev, 0, 'B')
    except Exception:
        return 0, False, False
    bins = min(3, max(0, (count - 1) // 3)) + (1 if count >= 14 else 0)
    return bins, count >= SUB_LEAD, count >= SUB_FADE


def bin_unit_ma(dev):
    """mA per raw bin count: 0.1 when 0x302E,16 says so, else 1."""
    try:
        if _r(dev, 0, 'B') >= SUB_UNIT and _r(dev, SUB_UNIT, 'H') == UNIT_TENTH_MA:
            return 0.1
    except Exception:
        pass
    return 1.0


def min_bin_ma(unit_ma, i_peak_ma=None):
    return max(MIN_BIN_UNITS * unit_ma, MIN_BIN_FRAC * (i_peak_ma or 0))


def _to_raw(ma, unit_ma):
    return max(-32768, min(32767, int(round(ma / unit_ma))))


def bin_subs(i):
    """(a_s, k, a_c) subs of bin i."""
    base = 2 + 3 * i if i < 3 else 12 + 3 * (i - 3)
    return base, base + 1, base + 2


def read_table(dev):
    """The table, bin amplitudes in mA (whatever the firmware's unit)."""
    n, has_lead, has_fade = layout(dev)
    u = bin_unit_ma(dev)
    bins = []
    for i in range(n):
        s, k, c = bin_subs(i)
        bins.append((round(_r(dev, s, 'h') * u, 1), _r(dev, k, 'H'),
                     round(_r(dev, c, 'h') * u, 1)))
    return dict(active=_r(dev, SUB_ACTIVE, 'H') if n else 0, bins=bins,
                lead_us=_r(dev, SUB_LEAD, 'H') if has_lead else None,
                fade_hz=_r(dev, SUB_FADE, 'H') if has_fade else None)


def write_table(dev, bins=None, lead_us=None, fade_hz=None, active=None):
    """Write what is given; bins (mA) pads with empty ones to the
    firmware's count and rounds to its unit.  Off first and on last, so no
    half-written table runs."""
    n, has_lead, has_fade = layout(dev)
    if active is not None and not active:
        _w(dev, SUB_ACTIVE, 'H', 0)
    if bins is not None:
        if len(bins) > n:
            raise SysidError('{} cogging bins, the firmware has {}'.format(
                len(bins), n))
        u = bin_unit_ma(dev)
        rows = list(bins) + [(0, 0, 0)] * (n - len(bins))
        for i, (a_s, k, a_c) in enumerate(rows):
            s, ks, c = bin_subs(i)
            _w(dev, s, 'h', _to_raw(a_s, u))
            _w(dev, ks, 'H', k)
            _w(dev, c, 'h', _to_raw(a_c, u))
    if lead_us is not None and has_lead:
        _w(dev, SUB_LEAD, 'H', max(0, min(65535, int(round(lead_us)))))
    if fade_hz is not None and has_fade:
        _w(dev, SUB_FADE, 'H', max(0, min(65535, int(round(fade_hz)))))
    if active:
        _w(dev, SUB_ACTIVE, 'H', 1)


# ---------------------------------------------------------------------------
# Capture
# ---------------------------------------------------------------------------
def _frame(dev):
    """The frame the stream's position (0x6064) is turned into the corrected
    encoder angle with: user sign, e_polarity, and one simultaneous-enough
    reading of both."""
    up = lambda i, s, f: struct.unpack(          # noqa: E731
        '<' + f, bytes(dev.node.sdo.upload(i, s)))[0]
    return dict(p6064=up(0x6064, 0, 'i'), enc=up(0x3012, 2, 'h'),
                pol=up(0x607E, 0, 'B'), epol=up(0x3011, 2, 'b'))


def capture(dev, rpms, revs=REVS, cfg=None, log=None):
    """Velocity-mode spins at each speed, both ways, streaming iq and
    position at 1 kHz.  Returns dict(cfg, frame, runs); each run holds
    rpm, sgn, cycles (control cycles, from 0), iq (A, mean over each
    frame) and pos (0x6064 counts, unwrapped)."""
    cfg = cfg or steptest.read_config(dev)
    res = cfg['enc_res']
    out = dict(cfg=cfg, frame=_frame(dev), runs=[])
    for rpm in rpms:
        for sgn in (1, -1):
            with _Stream(dev, LOG_SRC_IQ_POS) as s:
                try:
                    steptest._enable(dev, steptest.MODE_CSV)
                    steptest.write(dev, 'target_velocity',
                                   int(round(sgn * rpm * res / 60.0)))
                    steptest._sleep(dev, min(1.5, 0.3 + 6.0 / rpm))
                    m0 = s.mark()
                    steptest._sleep(dev, revs * 60.0 / rpm + 0.2)
                    m1 = s.mark()
                finally:
                    steptest._stop(dev)
                fr = np.array(s.frames[m0:m1], float)
            if len(fr) < 10:
                raise SysidError('the stream sent {} frames at {:+.0f} rpm'
                                 .format(len(fr), sgn * rpm))
            fr, bad = clean_frames(fr, res)
            if bad > MAX_BAD_FRAC * (len(fr) + bad):
                raise SysidError('{} of {} stream frames at {:+.0f} rpm were '
                                 'bad (duplicate, out of order or a jump): '
                                 'is the bus overloaded?'.format(
                                     bad, len(fr) + bad, sgn * rpm))
            c = np.cumsum(np.r_[0, np.mod(np.diff(fr[:, 0]), 65536)])
            ds = np.mod(np.diff(fr[:, 1]) + 2 ** 31, 2 ** 32) - 2 ** 31
            iq = ds / np.maximum(np.diff(c), 1) / 1000.0
            pos = np.unwrap(fr[:, 2] * (2 * np.pi / 65536)) * 65536 / (2 * np.pi)
            moved = abs(pos[-1] - pos[0]) / res
            if moved < 0.8 * revs:
                raise SysidError('{:+.0f} rpm moved {:.2f} of {} revolutions: '
                                 'is the rotor free and the velocity loop '
                                 'tuned?'.format(sgn * rpm, moved, revs))
            out['runs'].append(dict(rpm=rpm, sgn=sgn, cycles=c, iq=iq, pos=pos,
                                    bad_frames=bad))
            if log:
                log('cogging capture {:+.0f} rpm: {} frames{}, {:.2f} rev'
                    .format(sgn * rpm, len(fr),
                            ' ({} bad dropped)'.format(bad) if bad else '',
                            moved))
            steptest._sleep(dev, 0.3)
    return out


# ---------------------------------------------------------------------------
# Analysis (pure: tested on synthetic and recorded captures)
# ---------------------------------------------------------------------------
def clean_frames(fr, res):
    """Drop the stream frames that cannot be right: (cycle count mod 2^16,
    iq sum, position) rows.  A frame is kept when its cycle count advances
    from the last kept one by 1..MAX_GAP_X times the usual step (a lost
    frame doubles the step and is fine: the iq sum and the count cover the
    gap) and its position moves no more than POS_JUMP_X times the usual
    move for that many cycles, plus a few counts.  A bad frame is compared
    against the last good one, so one glitch costs one frame.  Returns
    (the kept frames, how many were dropped)."""
    fr = np.asarray(fr, float)
    if len(fr) < 3:
        return fr, 0
    dc = np.mod(np.diff(fr[:, 0]), 65536)
    step = float(np.median(dc[dc > 0])) if (dc > 0).any() else 1.0
    dp = np.mod(np.diff(fr[:, 2]) + 2 ** 31, 2 ** 32) - 2 ** 31
    ok = dc > 0
    rate = float(np.median(np.abs(dp[ok]) / dc[ok])) if ok.any() else 0.0
    def follows(a, b):
        n = (b[0] - a[0]) % 65536
        d = (b[2] - a[2] + 2 ** 31) % 2 ** 32 - 2 ** 31
        return 0 < n <= MAX_GAP_X * step and \
            abs(d) <= POS_JUMP_X * rate * n + 4 + res / 1024.0

    keep = [0]
    for i in range(1, len(fr)):
        a, b = fr[keep[-1]], fr[i]
        if follows(a, b):
            keep.append(i)
        elif 0 < (b[0] - a[0]) % 65536 < 32768 and i + 2 < len(fr) and \
                follows(b, fr[i + 1]) and follows(fr[i + 1], fr[i + 2]):
            # A real gap (the stream stalled), not a glitch: the frames go
            # on from here.  The cycle count still spans it.
            keep.append(i)
    return fr[keep], len(fr) - len(keep)


def kmax_for(pole_pairs, cap):
    """Highest k worth fitting, held to a quarter of the slowest frame count
    in a revolution of the capture."""
    k = max(KMAX_FLOOR, KMAX_PER_POLE_PAIR * int(pole_pairs or 0))
    fastest = max(r['rpm'] for r in cap['runs'])
    frames = steptest.SYNC_HZ * 60.0 / fastest     # one stream frame a SYNC
    return int(max(8, min(k, frames // 4)))


def _sign(frame):
    return (-1 if frame['pol'] & 0x80 else 1) * frame['epol']


def revolutions(cap, kmax):
    """Per run, per whole revolution: the velocity (rpm) and load torque
    (mNm, the raw encoder's direction) phasors at k = 1..kmax on the
    corrected angle, h = c_k + j s_k for c cos + s sin.  Returns a list of
    dict(rpm, sgn, V [revs x kmax], L [revs x kmax], noise [revs]).

    The load is d = Kt iq - J alpha in the time domain, alpha the second
    difference of the position at the frame instants, then fitted on the
    angle.  Not -(k W)^2 times the angle's ripple: that assumes the ripple
    is small against the speed, and uncompensated at 30-60 rpm node 4's is
    not (28 rpm at 30): k14 read 1.5-2 mNm for 2.9.  Each phasor is divided
    by what the frames do to it at its frequency: the second difference is
    two frame-long boxcars (sinc^2), the iq beside it a frame-long boxcar
    averaged over two frames (sinc cos)."""
    cfg, f0 = cap['cfg'], cap['frame']
    res, hz = cfg['enc_res'], cfg['control_hz']
    kt_pk = cfg['kt'] * 1e-3 / math.sqrt(2.0)       # Nm per A of iq
    j = cfg['j_kg_m2']
    S = _sign(f0)
    off = f0['enc'] - S * f0['p6064']
    ks = np.arange(1, kmax + 1)
    out = []
    for r in cap['runs']:
        t = r['cycles'] / hz
        th = 2 * np.pi * (off + S * r['pos']) / res      # corrected angle, unwrapped
        dt = np.diff(t)
        w = np.diff(th) / dt                              # rad/s between frames
        tm = 0.5 * (t[1:] + t[:-1])
        alpha = np.diff(w) / np.diff(tm)                  # at frames 1..n-2
        iq = r['iq']
        iq_at = (iq[:-1] * dt[:-1] + iq[1:] * dt[1:]) / (dt[:-1] + dt[1:])
        load = S * iq_at * kt_pk - j * alpha              # Nm, raw direction
        tha = th[1:-1]
        v = w * 60.0 / (2 * np.pi) * S * r['sgn']         # rpm, positive
        thm = 0.5 * (th[1:] + th[:-1])
        frame_s = float(np.median(dt))
        V, L, noise = [], [], []
        sg = np.sign(th[-1] - th[0])
        nrev = int(abs(th[-1] - th[0]) // (2 * np.pi))
        for i in range(nrev):
            lo, hi = th[0] + sg * i * 2 * np.pi, th[0] + sg * (i + 1) * 2 * np.pi
            lo, hi = min(lo, hi), max(lo, hi)
            mv = (thm >= lo) & (thm < hi)
            ma = (tha >= lo) & (tha < hi)
            Av = np.column_stack([np.ones(mv.sum()), tm[mv] - tm[mv].mean()] +
                                 [g(k * thm[mv]) for k in ks
                                  for g in (np.cos, np.sin)])
            Aa = np.column_stack([np.ones(ma.sum())] +
                                 [g(k * tha[ma]) for k in ks
                                  for g in (np.cos, np.sin)])
            cv, *_ = np.linalg.lstsq(Av, v[mv], rcond=None)
            cl, *_ = np.linalg.lstsq(Aa, load[ma], rcond=None)
            cq, *_ = np.linalg.lstsq(Aa, S * iq_at[ma] * kt_pk, rcond=None)
            W = 2 * np.pi / max(t[1:-1][ma][-1] - t[1:-1][ma][0], 1e-9)
            x = ks * W / (2 * np.pi) * frame_s            # cycles per frame
            h_iq = np.sinc(x) * np.cos(np.pi * x)
            h_a = np.sinc(x) ** 2
            Lk = cl[1::2] + 1j * cl[2::2]
            Qk = cq[1::2] + 1j * cq[2::2]
            # load = motor/h_iq - (motor - load)/h_a, each part unfiltered
            V.append(cv[2::2] + 1j * cv[3::2])
            L.append((Qk / h_iq - (Qk - Lk) / h_a) * 1e3)
            noise.append((v[mv] - Av @ cv).std())
        noise = np.array(noise)
        good = np.isfinite(noise)
        if good.any():
            good &= noise <= REV_NOISE_X * np.median(noise[good])
        out.append(dict(rpm=r['rpm'], sgn=r['sgn'], V=np.array(V)[good],
                        L=np.array(L)[good], noise=noise[good],
                        bad_revs=int(len(noise) - np.count_nonzero(good))))
    return out


def fit_lines(revs):
    """Run-to-run statistics of the load torque per k: the mean over runs
    of each run's mean (mNm phasor), its standard error from the spread of
    the run means, and the run means themselves."""
    runs = np.array([r['L'].mean(0) for r in revs if len(r['L'])])
    n = len(runs)
    if n < 2:
        raise SysidError('the cogging fit needs at least two runs')
    mean = runs.mean(0)
    se = np.sqrt((np.abs(runs - mean) ** 2).sum(0) / (n * (n - 1)))
    return dict(mean=mean, se=se, runs=runs)


def choose_bins(lines, kt_mnm_per_a_rms, n_bins, exclude=(), unit_ma=0.1,
                min_ma=None):
    """The strongest stationary lines as firmware bins (a_s, k, a_c) mA,
    rounded to unit_ma, strongest first, and every candidate's figures for
    the report.  min_ma defaults to min_bin_ma(unit_ma)."""
    if min_ma is None:
        min_ma = min_bin_ma(unit_ma)
    kt_pk = kt_mnm_per_a_rms / math.sqrt(2.0)      # mNm per A of iq
    mean, se = lines['mean'], lines['se']
    rows = []
    for i, m in enumerate(mean):
        k = i + 1
        if k in exclude:
            continue
        z = abs(m) / se[i] if se[i] > 0 else float('inf')
        ma = abs(m) / kt_pk * 1e3
        rows.append(dict(k=k, mnm=abs(m), ma=ma, z=z, h=m))
    ok = [r for r in rows if r['z'] >= MIN_Z and r['ma'] >= min_ma]
    ok.sort(key=lambda r: -r['mnm'])
    bins = []
    for r in ok[:n_bins]:
        i_ma = r['h'] / kt_pk * 1e3
        q = lambda x: round(round(x / unit_ma) * unit_ma, 1)  # noqa: E731
        bins.append((q(i_ma.imag), r['k'], q(i_ma.real)))
    return bins, ok


def locked_power(revs, sel=None):
    """Angle-locked velocity power per k (rpm^2): per direction the power
    of the mean phasor less what the noise leaves in a mean, averaged over
    the directions."""
    P = []
    for sg in (1, -1):
        X = [x for r in revs if r['sgn'] == sg for x in r['V']]
        if sel is not None:
            X = [X[i] for i in sel[sg]]
        X = np.array(X)
        if len(X) < 2:
            continue
        P.append(np.abs(X.mean(0)) ** 2 / 2 - np.var(X, 0) / len(X) / 2)
    return np.mean(P, 0)


def ripple(revs, exclude=(), rng=None):
    """(locked velocity ripple rms over the k not excluded, its bootstrap
    standard error, the per-k power)."""
    rng = rng or np.random.default_rng(1)
    p = locked_power(revs)
    keep = np.ones(len(p), bool)
    for k in exclude:
        if 1 <= k <= len(p):
            keep[k - 1] = False
    total = lambda q: math.sqrt(max(float(q[keep].sum()), 0.0))  # noqa: E731
    counts = {sg: sum(len(r['V']) for r in revs if r['sgn'] == sg)
              for sg in (1, -1)}
    boot = []
    for _ in range(BOOTSTRAP):
        sel = {sg: rng.integers(0, n, n) for sg, n in counts.items() if n}
        boot.append(total(locked_power(revs, sel)))
    return total(p), float(np.std(boot)), p


def line_rpm(revs, k):
    """(amplitude rpm, standard error) of the velocity line k, pooled over
    the directions."""
    amps, var = [], []
    for sg in (1, -1):
        X = np.array([x[k - 1] for r in revs if r['sgn'] == sg for x in r['V']])
        if len(X) < 2:
            continue
        amps.append(abs(X.mean()))
        var.append(np.var(X) / len(X))
    return float(np.mean(amps)), float(math.sqrt(np.mean(var) / len(var)))


def excluded_ks(pole_pairs, encoder_ks):
    """Lines that are not cogging: k = pole pairs (the sense offset, which
    wanders run to run and belongs to the bias cal) unless the motor has one
    pole pair -- there k1 is also the rotor's once-per-rev load (a P4-16's
    strongest, 6 mA behind its gearhead) and the fit's run-to-run z test
    is what keeps a wandering offset out -- the slow wander at
    k <= 4 (from the metric only), and the encoder's 0x3027 orders with the
    sidebands its k2 puts on them."""
    ex = set()
    if pole_pairs and int(pole_pairs) > 1:
        ex.add(int(pole_pairs))
    for k in encoder_ks:
        ex.add(k)
        if k > 4:
            ex.update((k - 2, k + 2))
    return ex


def _lead_at(budget, f):
    return int(round(budget * f / 50.0) * 50)


def best_lead(scan):
    return min(scan, key=lambda x: x[1])[0]


def scan_lead(budget, measure, scales=LEAD_SCALES):
    """[(lead us, ripple, se)] over the budget times *scales*, extended a
    LEAD_SCALE_STEP at a time while the best is the highest (lowest) lead
    tried, within LEAD_SCALE_MIN..MAX.  *measure*(lead) -> (ripple, se)."""
    tried = {}

    def at(f):
        cand = _lead_at(budget, f)
        if cand not in tried:
            tried[cand] = measure(cand)
        return cand
    fs = sorted(scales)
    for f in fs:
        at(f)
    lo, hi = fs[0], fs[-1]
    while True:
        scan = sorted((c, v, se) for c, (v, se) in tried.items())
        best = best_lead(scan)
        if best == scan[-1][0] and hi + LEAD_SCALE_STEP <= LEAD_SCALE_MAX + 1e-9:
            hi += LEAD_SCALE_STEP
            at(hi)
        elif best == scan[0][0] and lo - LEAD_SCALE_STEP >= LEAD_SCALE_MIN - 1e-9:
            lo -= LEAD_SCALE_STEP
            at(lo)
        else:
            return scan


def lead_budget_us(cfg, latency_us):
    """Encoder latency + the current loop's lag (1/wc) + a control cycle."""
    bw = cfg.get('current_bw_hz') or cfg['control_hz'] / 64.0
    return latency_us + 1e6 / (2 * math.pi * bw) + 1e6 / cfg['control_hz']


def fade_for(cfg):
    bw = cfg.get('current_bw_hz') or cfg['control_hz'] / 64.0
    return int(round(FADE_PER_CURRENT_BW * bw))


# ---------------------------------------------------------------------------
# The whole job
# ---------------------------------------------------------------------------
def _encoder_ks(dev):
    bins, _ = encoder.comp_layout(dev)
    if not bins or not encoder.comp_active(dev):
        return []
    ks = [encoder._sdo_r(dev, encoder.OD_COMP, 3 + 3 * i, 'H')
          for i in range(bins)]
    return [k for k in ks if k]


def calibrate(dev, pole_pairs, log=None, fit_rpms=FIT_RPMS,
              verify_rpms=VERIFY_RPMS, revs=REVS):
    """Fit, lead, fade and verify the cogging table; leaves it in RAM, on.
    Returns the record for the report; 'passed' False leaves the table the
    drive had before."""
    log = log or (lambda *a: None)
    n_bins, has_lead, has_fade = layout(dev)
    if not n_bins:
        raise SysidError('this firmware has no cogging feed-forward (0x302E)')
    before = read_table(dev)
    unit = bin_unit_ma(dev)
    cfg = steptest.read_config(dev)
    min_ma = min_bin_ma(unit, cfg.get('i_peak'))
    if not cfg['vel_kp']:
        raise SysidError('no velocity gains: tune the velocity loop first')
    enc_ks = _encoder_ks(dev)
    ex_fit = excluded_ks(pole_pairs, enc_ks)
    ex_metric = ex_fit | {1, 2, 3, 4}
    pipe, trim = encoder.read_latency_us(dev)
    latency = (pipe or 0.0) + trim
    fade = fade_for(cfg) if has_fade else None
    budget = lead_budget_us(cfg, latency)
    rec = dict(before=before, fade_hz=fade, budget_us=budget, passed=False)
    try:
        # Pass 1, off; pass 2 with it on at the budget lead.
        write_table(dev, active=0)
        cap = capture(dev, fit_rpms, revs, cfg, log)
        kmax = kmax_for(pole_pairs, cap)
        lines = fit_lines(revolutions(cap, kmax))
        bins, _ = choose_bins(lines, cfg['kt'], n_bins, ex_fit, unit, min_ma)
        if not bins:
            rec.update(bins=[], detail='no stationary line above {:g} mA'
                       .format(min_ma))
            write_table(dev, bins=[], active=0)
            rec['passed'] = True
            return rec
        log('cogging pass 1: ' + _fmt_bins(bins))
        write_table(dev, bins=bins, lead_us=budget, fade_hz=fade, active=1)
        steptest._sleep(dev, 0.3)
        cap = capture(dev, fit_rpms, revs, cfg, log)
        lines = fit_lines(revolutions(cap, kmax))
        bins, cands = choose_bins(lines, cfg['kt'], n_bins, ex_fit, unit,
                                  min_ma)
        log('cogging pass 2: ' + _fmt_bins(bins))
        rec['candidates'] = [(c['k'], round(c['mnm'], 2), round(c['z'], 1))
                             for c in cands]
        write_table(dev, bins=bins)
        # Lead: scanned from the budget up, at the faster fit speed.
        lead = budget
        if has_lead:
            def measure(cand):
                write_table(dev, lead_us=cand)
                c = capture(dev, fit_rpms[-1:], revs, cfg, log)
                v, se, _ = ripple(revolutions(c, kmax), ex_metric)
                log('cogging lead {} us: {:.2f} +- {:.2f} rpm'.format(cand, v, se))
                return v, se
            scan = scan_lead(budget, measure)
            lead = best_lead(scan)
            rec['lead_scan'] = scan
            write_table(dev, lead_us=lead)
        rec['lead_us'] = lead
        # Verify: off vs on.
        write_table(dev, active=0)
        steptest._sleep(dev, 0.3)
        off = revolutions(capture(dev, verify_rpms, revs, cfg, log), kmax)
        write_table(dev, active=1)
        steptest._sleep(dev, 0.3)
        on = revolutions(capture(dev, verify_rpms, revs, cfg, log), kmax)
        verify, keep = judge(off, on, bins, ex_metric)
        rec.update(verify=verify, kept=keep,
                   dropped=[b for b in bins if b not in keep])
        if not keep:
            # Measured, not broken: the fitted lines are not what moves the
            # rotor (node 2, a direct drive: k148/k94/k71 at 0.1-0.2 mNm).
            # The drive keeps the table it had, as for any 'passed' False, and
            # the tuning after it runs (runout's "not improved" is a warn too).
            write_table(dev, bins=before['bins'], lead_us=before['lead_us'],
                        fade_hz=before['fade_hz'], active=before['active'])
            rec.update(bins=[], detail='no bin lowered its own velocity line: '
                       'the table the drive had is kept')
            return rec
        write_table(dev, bins=keep)
        rec['bins'] = keep
        slow = verify[0]
        rec['passed'] = slow['off'] - slow['on'] > VERIFY_SIGMAS * math.hypot(
            slow['off_se'], slow['on_se'])
        rec['worse'] = [v['rpm'] for v in verify
                        if v['on'] - v['off'] > VERIFY_SIGMAS * math.hypot(
                            v['off_se'], v['on_se'])]
        if not rec['passed']:
            raise SysidError('the ripple at {:.0f} rpm did not come down '
                             '({:.2f} -> {:.2f} rpm)'.format(
                                 slow['rpm'], slow['off'], slow['on']))
        return rec
    except BaseException:
        try:
            write_table(dev, bins=before['bins'], lead_us=before['lead_us'],
                        fade_hz=before['fade_hz'], active=before['active'])
        except Exception:     # noqa: BLE001 -- the first error is the story
            pass
        raise


def judge(off, on, bins, exclude):
    """Per speed: locked ripple off and on; and the bins that lowered their
    own velocity line at the slowest speed (or, for a bin faded there,
    wherever it is whole)."""
    verify = []
    for rpm in sorted({r['rpm'] for r in off}):
        a = [r for r in off if r['rpm'] == rpm]
        b = [r for r in on if r['rpm'] == rpm]
        v0, s0, _ = ripple(a, exclude)
        v1, s1, _ = ripple(b, exclude)
        verify.append(dict(rpm=rpm, off=v0, off_se=s0, on=v1, on_se=s1))
    slowest = min(r['rpm'] for r in off)
    a = [r for r in off if r['rpm'] == slowest]
    b = [r for r in on if r['rpm'] == slowest]
    keep = []
    for bn in bins:
        k = bn[1]
        l0, e0 = line_rpm(a, k)
        l1, e1 = line_rpm(b, k)
        if l0 - l1 > KEEP_SIGMAS * math.hypot(e0, e1):
            keep.append(bn)
    return verify, keep


def _fmt_bins(bins):
    return ', '.join('k{} ({}, {})'.format(k, a_s, a_c) for a_s, k, a_c in bins)


def summary(rec):
    if not rec.get('bins'):
        return rec.get('detail', 'no bins')
    v = rec.get('verify') or []
    ripple_txt = ', '.join('{:.0f} rpm {:.1f}->{:.1f}'.format(
        x['rpm'], x['off'], x['on']) for x in v)
    txt = '{}; lead {} us, fade {} Hz; ripple {}'.format(
        _fmt_bins(rec['bins']), rec.get('lead_us'), rec.get('fade_hz'),
        ripple_txt)
    if rec.get('dropped'):
        txt += '; dropped ' + ', '.join('k{}'.format(b[1])
                                        for b in rec['dropped'])
    return txt
