"""
dragfit.py — the drag's host-side fits: J, B and Coulomb friction from a
rotor dragged on a rotating field, and the flux from the cruise voltages.

Moved from p4sysid.estim (which re-exports these).  Units: SI throughout,
per phase, peak amplitudes; Kt per RMS amp, as the drive stores it.
"""

import math

import numpy as np


def kt_from_flux(psi_wb, pole_pairs):
    """NXP's Kt = 1.5 * pp * psi, per peak amp, as the drive's per-RMS figure."""
    return 1.5 * pole_pairs * psi_wb * math.sqrt(2.0)


# ---------------------------------------------------------------------------
# Drag: J, B and Coulomb friction with the rotor on a rotating field
# ---------------------------------------------------------------------------
def local_speed(position_rad, sample_interval_s, half):
    """Speed at every sample from a least-squares line through 2*half+1 samples.

    Returns ``(speed, first)``: the speeds for samples first .. n-first-1.
    A line, not a difference, because a 4096-count encoder sampled at a few
    kHz moves only a count or two per sample at low speed.
    """
    p = np.asarray(position_rad, float)
    k = np.arange(-half, half + 1, dtype=float)
    return np.correlate(p, k / (k @ k) / sample_interval_s, 'valid'), half


def ring_frequency(speed, sample_interval_s, min_hz=2.0):
    """The dominant ringing in a speed record, Hz, and its peak-to-peak.

    For the dragged rotor's magnetic spring, which rings almost undamped at
    sqrt(K/J).  Mean and slope removed, Hann window, zero-padded FFT, and a
    parabola through the peak bin and its neighbours.
    """
    x = np.asarray(speed, float)
    n = len(x)
    t = np.arange(n)
    x = x - np.polyval(np.polyfit(t, x, 1), t)
    pad = 8 * n
    spec = np.abs(np.fft.rfft(x * np.hanning(n), pad))
    freq = np.fft.rfftfreq(pad, sample_interval_s)
    spec[freq < min_hz] = 0.0
    k = int(np.argmax(spec))
    if 0 < k < len(spec) - 1:
        y0, y1, y2 = spec[k - 1], spec[k], spec[k + 1]
        den = y0 - 2 * y1 + y2
        k = k + (0.5 * (y0 - y2) / den if den else 0.0)
    return float(k * freq[1]), float(x.max() - x.min())


def drag_torque(iq_a, i_hold_a, kt_nm_per_a_rms, pole_pairs, ld_h=None,
                lq_h=None):
    """Air-gap torque of a rotor dragged by a d current of *i_hold_a*.

    *iq_a* is the rotor-frame q current (capture source 6) and the field-frame
    loop holds the vector's length at *i_hold_a*, so the rotor-frame d current
    is sqrt(I^2 - iq^2).  That d current is large, which makes the reluctance
    term matter on a salient motor:

        T = 1.5 pp (psi + (Ld - Lq) id) iq,    1.5 pp psi = Kt / sqrt2

    With Ld < Lq it takes away torque: 7 % at 3 A on a P4-42.  Without
    Ld and Lq the PM term alone is returned.
    """
    iq = np.asarray(iq_a, float)
    kt_pk = kt_nm_per_a_rms / math.sqrt(2.0)
    if ld_h is None or lq_h is None:
        return kt_pk * iq
    psi = kt_pk / (1.5 * pole_pairs)
    i_d = np.sqrt(np.maximum(i_hold_a * i_hold_a - iq * iq, 0.0))
    return kt_pk * iq * (1.0 + (ld_h - lq_h) * i_d / psi)


def _cumtrapz(y, dt):
    return np.concatenate(([0.0], np.cumsum(0.5 * (y[1:] + y[:-1]) * dt)))


def _cumtrapz_t(y, t):
    return np.concatenate(([0.0], np.cumsum(0.5 * (y[1:] + y[:-1])
                                            * np.diff(t))))


def stream_window(frames, cycle_s, rad_per_count, torque_of_iq_a):
    """``(t, position_rad, int_torque_nm_s)`` from stream frames.

    *frames* are the stream TPDO's ``(cycles, iq_sum_ma, position)`` in the
    order they arrived: control cycles (U16), the sum of the rotor-frame iq in
    mA over them (I32) and the position in counts (I16), all wrapping.  Each
    difference is the exact integral of iq over an exact number of cycles, so
    the result is exact however irregular the SYNCs were -- the time axis is
    the device's.  *torque_of_iq_a* turns each interval's mean iq (A) into
    torque; the integral of that over the interval is its torque times its
    length.  A frame that repeats its predecessor's cycle count is dropped.
    """
    f = np.asarray(frames, np.int64).reshape(-1, 3)
    dc = np.diff(f[:, 0]) & 0xFFFF
    keep = np.concatenate(([True], dc > 0))
    f = f[keep]
    dc = np.diff(f[:, 0]) & 0xFFFF
    ds = (np.diff(f[:, 1]) + (1 << 31)) % (1 << 32) - (1 << 31)
    dp = (np.diff(f[:, 2]) + (1 << 15)) % (1 << 16) - (1 << 15)
    if dc.size == 0:
        return np.zeros(1), np.zeros(1), np.zeros(1)
    iq = ds / dc.astype(float) / 1000.0
    t = np.concatenate(([0], np.cumsum(dc))) * cycle_s
    th = np.concatenate(([0], np.cumsum(dp))) * rad_per_count
    it = np.concatenate(([0.0], np.cumsum(
        np.asarray(torque_of_iq_a(iq), float) * dc * cycle_s)))
    return t, th, it


def drag_fit(segments, min_speed_rad_s=5.0):
    """J, B and Coulomb friction from captures of a dragged rotor.

    *segments* is a list of ``(position_rad, torque_nm, sample_interval_s)``,
    one per capture: sampled torque, integrated here and handed to
    drag_fit_windows.
    """
    windows = []
    for position_rad, torque_nm, dt in segments:
        p = np.asarray(position_rad, float)
        tq = np.asarray(torque_nm, float)
        windows.append((np.arange(p.size) * dt, p, _cumtrapz(tq, dt)))
    return drag_fit_windows(windows, min_speed_rad_s)


def drag_fit_windows(windows, min_speed_rad_s=5.0):
    """J, B and Coulomb friction from windows of a dragged rotor.

    *windows* is a list of ``(t, position_rad, int_torque_nm_s)``: sample
    times, the position there, and the torque integrated up to there -- from
    the stream exactly (stream_window), from a capture by the trapezoid.  Each
    window turns one way throughout.  The equation of motion
    J th'' = T - B th' - Tc sgn(th') integrated twice from the start of a
    window is, at every sample,

        int int T  =  J th + B int th + Tc+ t^2/2 - Tc- t^2/2 + c0 + c1 t

    with th the position less its first sample, one of the Tc terms present
    by the direction, and c0 + c1 t -- the unknown speed and position the
    window started at, times J and B -- a nuisance pair per window.  One
    least-squares fit over all of them; no speed is ever differentiated out
    of the position, which is what biased B when it was.  Each window is
    weighted by 1/duration^2 so a long one does not drown the rest.  A
    window that comes within *min_speed_rad_s* of standstill is dropped, as
    there friction is whatever stiction makes it rather than Tc.

    Returns a dict: ``j_kg_m2``, ``b_nm_s_per_rad``, ``tc_pos_nm``,
    ``tc_neg_nm`` (None for a direction with no windows), ``segments`` used
    and ``residual_rms_nm_s2``.
    """
    usable = []
    for t, position_rad, it in windows:
        t = np.asarray(t, float)
        p = np.asarray(position_rad, float)
        it = np.asarray(it, float)
        if p.size < 16:
            continue
        # Speed over ~1 ms spans, however the window is sampled.
        span = max(1, int(round(0.001 / max(np.median(np.diff(t)), 1e-9))))
        w = (p[span:] - p[:-span]) / (t[span:] - t[:-span])
        if np.min(np.abs(w)) < min_speed_rad_s or np.ptp(np.sign(w)) != 0:
            continue
        usable.append((t - t[0], p - p[0], it - it[0], w[0] > 0))
    if len(usable) < 2:
        raise ValueError('only {} usable drag windows'.format(len(usable)))
    n = len(usable)
    rows, rhs = [], []
    for k, (tt, p, it, pos) in enumerate(usable):
        x = np.zeros((p.size, 4 + 2 * n))
        x[:, 0] = p
        x[:, 1] = _cumtrapz_t(p, tt)
        x[:, 2 if pos else 3] = (1.0 if pos else -1.0) * 0.5 * tt * tt
        x[:, 4 + 2 * k] = 1.0
        x[:, 5 + 2 * k] = tt
        weight = 1.0 / (tt[-1] * tt[-1])
        rows.append(x * weight)
        rhs.append(_cumtrapz_t(it, tt) * weight)
    x = np.vstack(rows)
    y = np.concatenate(rhs)
    used = [c for c in range(x.shape[1]) if np.any(x[:, c] != 0.0)]
    xs = x[:, used]
    scale = np.sqrt(np.sum(xs * xs, axis=0))
    c, *_ = np.linalg.lstsq(xs / scale, y, rcond=None)
    c = c / scale
    coef = dict(zip(used, c))
    res = y - xs @ c
    return {
        'j_kg_m2': float(coef[0]),
        'b_nm_s_per_rad': float(coef.get(1, 0.0)),
        'tc_pos_nm': float(coef[2]) if 2 in coef else None,
        'tc_neg_nm': float(coef[3]) if 3 in coef else None,
        'segments': n,
        'residual_rms_nm_s2': float(np.sqrt(np.mean(res * res))),
    }



# ---------------------------------------------------------------------------
# Drag: the flux, from the voltage the current loop asks for in the cruises
# ---------------------------------------------------------------------------
def pair_frames(times_a, times_b, tolerance_s):
    """For each time in *times_a*, the index of the nearest in *times_b*, or
    -1 when none is within *tolerance_s*.  Two TPDOs sent at one SYNC arrive
    back to back, a SYNC period from the next pair; nearest is that one."""
    a = np.asarray(times_a, float)
    b = np.asarray(times_b, float)
    out = np.full(a.size, -1, int)
    if a.size == 0 or b.size == 0:
        return out
    order = np.argsort(b, kind='stable')
    bs = b[order]
    k = np.clip(np.searchsorted(bs, a), 1, bs.size - 1) if bs.size > 1 \
        else np.zeros(a.size, int)
    lo = np.where(np.abs(a - bs[k - 1]) <= np.abs(a - bs[k]), k - 1, k) \
        if bs.size > 1 else k
    ok = np.abs(a - bs[lo]) <= tolerance_s
    out[ok] = order[lo[ok]]
    return out


def drag_cruise(frames, volts, start_cycles, cycle_s, rad_per_count,
                counts_per_rev, pole_pairs, u_max_v):
    """The means over the end of one drag cruise, in whole revolutions.

    *frames* are the stream's ``(cycles, iq_sum_ma, position)`` over one step,
    *volts* the voltage stream's ``(ud_sum, uq_sum)`` sent at the same SYNCs
    (None where that frame was lost): each the sum over the control cycles
    of the field-frame request in Q15 of U_MAX.  The cruise is what follows
    *start_cycles* after the first frame; it is cut to the most whole
    mechanical revolutions it holds, else electrical ones, so a ripple that
    repeats with the rotor or the field averages out.  Returns ``omega_e``
    (rad/s, electrical, by the encoder's direction), ``iq_a`` (rotor frame),
    ``ud_v``, ``uq_v``, ``cycles``, or None if the cruise is too short.
    """
    f = np.asarray(frames, np.int64).reshape(-1, 3)
    if f.shape[0] < 3:
        return None
    c = np.concatenate(([0], np.cumsum(np.diff(f[:, 0]) & 0xFFFF)))
    s = np.concatenate(([0], np.cumsum(
        (np.diff(f[:, 1]) + (1 << 31)) % (1 << 32) - (1 << 31))))
    p = np.concatenate(([0], np.cumsum(
        (np.diff(f[:, 2]) + (1 << 15)) % (1 << 16) - (1 << 15))))
    have = np.array([v is not None for v in volts])
    idx = np.nonzero((c >= start_cycles) & have)[0]
    if idx.size < 2:
        return None
    end = idx[-1]
    travel = np.abs(p[end] - p[idx])
    rev = None
    for per in (counts_per_rev, counts_per_rev / float(pole_pairs)):
        n = math.floor(travel[0] / per)
        if n >= 1:
            rev = n * per
            break
    if rev is None:
        return None
    first = idx[np.argmin(np.abs(travel - rev))]
    dc = c[end] - c[first]
    if dc <= 0:
        return None
    u = np.zeros((2, 2), np.int64)
    for k, i in enumerate((first, end)):
        u[k] = volts[i]
    du = (u[1] - u[0] + (1 << 31)) % (1 << 32) - (1 << 31)
    return {'omega_e': pole_pairs * (p[end] - p[first]) * rad_per_count
            / (dc * cycle_s),
            'iq_a': (s[end] - s[first]) / dc / 1000.0,
            'ud_v': du[0] / dc / 32768.0 * u_max_v,
            'uq_v': du[1] / dc / 32768.0 * u_max_v,
            'cycles': int(dc)}


def drag_flux_fit(cruises, delay_s=0.0, lq_minus_ld_h=0.0,
                  max_load_deg=20.0):
    """psi (and Ld) from the voltage the field-frame loop asks for.

    During DRAG the loop holds the current at (I, 0) in the frame of the
    field, which leads the rotor's d axis by the load angle delta,
    sin(delta) = iq_rotor / I.  In that frame, steady at w_e,

        uq = w_e (psi cos(delta) + I (Ld cos^2(delta) + Lq sin^2(delta)))
        ud = R I + Udt + w_e (psi + (Ld - Lq) I cos(delta)) sin(delta)

    so uq holds neither R nor the dead-time drop -- both lie along the
    current, which is d here -- nor the current sensor's gain, but for the
    I L term.  The request is applied *delay_s* after the angle it was
    turned through was current, by when the field has moved on by w_e times
    that; each (ud, uq) is turned back by the angle before use.  Left
    uncorrected, that angle leaks the ud request into uq and reads as
    delay times Udt of extra flux, about 1 %.

    Each cruise is a dict of drag_cruise's plus ``current_a`` (I) and
    ``sign`` (the direction it was commanded).  One least-squares fit over
    all of them for psi and Ld, with an offset per current and direction
    that no speed-proportional term can hide in; the two currents separate
    psi from Ld I.  A cruise whose load angle is past *max_load_deg* is left
    out: toward pull-out cos(delta) carries psi away and every error in the
    measured iq with it.  Returns ``psi_wb``, ``ld_h``, ``delay_from_ud_s`` (the
    delay the ud requests imply), ``polarity`` (+1 when the
    encoder counts the way the field turns), ``residual_v``, ``cruises``
    and the per-group ``offsets_v``.
    """
    limit = math.sin(math.radians(max_load_deg))
    rows = [cr for cr in cruises if cr is not None
            and abs(cr['iq_a']) <= limit * cr['current_a']]
    if len(rows) < 4:
        raise ValueError('only {} drag cruises for the flux'.format(len(rows)))
    currents = sorted({round(cr['current_a'], 6) for cr in rows})
    if len(currents) < 2:
        raise ValueError('the flux fit needs cruises at two currents')
    w = np.array([cr['omega_e'] for cr in rows])
    uq0 = np.array([cr['uq_v'] for cr in rows])
    # The field turns the way the encoder does, or the other; the flux is
    # positive, which decides.
    polarity = 1.0 if np.sum(w * uq0) >= 0 else -1.0
    w = polarity * w
    phi = w * delay_s
    ud = np.array([cr['ud_v'] for cr in rows])
    uq = uq0 * np.cos(phi) - ud * np.sin(phi)
    i = np.array([cr['current_a'] for cr in rows])
    sin_d = np.clip(np.array([cr['iq_a'] for cr in rows]) / i, -1.0, 1.0)
    cos_d = np.sqrt(1.0 - sin_d * sin_d)
    groups = sorted({(round(cr['current_a'], 6), cr['sign']) for cr in rows})
    g = [groups.index((round(cr['current_a'], 6), cr['sign'])) for cr in rows]
    x = np.zeros((len(rows), 2 + len(groups)))
    x[:, 0] = w * cos_d
    x[:, 1] = w * i
    x[np.arange(len(rows)), 2 + np.asarray(g)] = 1.0
    y = uq - w * i * lq_minus_ld_h * sin_d * sin_d
    scale = np.sqrt(np.sum(x * x, axis=0))
    coef, *_ = np.linalg.lstsq(x / scale, y, rcond=None)
    coef = coef / scale
    res = y - x @ coef
    psi, ld = float(coef[0]), float(coef[1])
    # The delay the ud requests put on themselves: turned back by it, they
    # must be R I + Udt, one per current, plus the known speed terms.  A
    # check on *delay_s*, not a substitute: an encoder that trails the
    # current sample tilts delta by w_e times that, which reads here as delay.
    cur = [currents.index(round(cr['current_a'], 6)) for cr in rows]
    xd = np.zeros((len(rows), 1 + len(currents)))
    xd[:, 0] = -w * uq
    xd[np.arange(len(rows)), 1 + np.asarray(cur)] = 1.0
    yd = ud - w * sin_d * (psi - lq_minus_ld_h * i * cos_d)
    sd = np.sqrt(np.sum(xd * xd, axis=0))
    cd, *_ = np.linalg.lstsq(xd / sd, yd, rcond=None)
    return {'psi_wb': psi, 'ld_h': ld,
            'delay_from_ud_s': float(cd[0] / sd[0]),
            'polarity': int(polarity),
            'residual_v': float(np.sqrt(np.mean(res * res))),
            'cruises': len(rows),
            'offsets_v': {'{:.3f} A {:+d}'.format(*grp): float(v)
                          for grp, v in zip(groups, coef[2:])}}
