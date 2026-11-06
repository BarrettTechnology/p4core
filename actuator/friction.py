"""Coulomb friction feed-forward 0x2381,3 (stm32 e1b0a4e), from the drag's Tc.

The firmware adds +-ff (rated torque / 1000) by the sign of the
trajectory's velocity, 0 at rest -- in every mode with a trajectory, CSV
included.  friction_ff() sizes it from Tc (0x3011,17, the drag fit) and
keeps it only when profile-position moves follow closer with it on.

It runs before cogging: the cogging spins are slow CSV holds, and on a
geared or high-friction motor the rotor sticks and slips there without the
ff (cogging's stick-slip check).  The cogging fit is then of what the ff
leaves.

Moves are measured on the drive's logger (steptest._capture); the value
kept is left in RAM, the caller saves.
"""

import math
import struct

import numpy as np

from . import loops, steptest
from .steptest import StepTestError

#: The PP move: +-POS_STEP_DEG, settled into POS_BAND_PCT of it; the most
#: overshoot the ff may add, percent of the move.
POS_STEP_DEG = 90.0
POS_BAND_PCT = 1.0
POS_OVERSHOOT_MAX_PCT = 1.5

#: The PP move pp_following() runs: POS_STEP_DEG at
#: up to VELFF_TOP_FRACTION of the top speed, accelerating on at most
#: VELFF_TORQUE_FRACTION of RatedTorque (J alpha), never in under
#: VELFF_RAMP_MIN_S nor VELFF_RAMP_TAUS velocity-loop time constants -- and
#: no faster than the position loop follows: with velff off it lags v / kp,
#: held to VELFF_LAG_FRACTION of the move (a twin at kp 46 lagged 62 deg of
#: 90 at 900 rpm, and every velff overshot).
VELFF_TOP_FRACTION = 0.3
VELFF_RPM_CAP = 1500.0
VELFF_TORQUE_FRACTION = 0.3
VELFF_RAMP_MIN_S = 0.02
VELFF_RAMP_TAUS = 4.0
VELFF_LAG_FRACTION = 0.25
VELFF_TAIL_S = 0.3
VELFF_SRC_COMMANDED = 0

#: 0x6041 bit 3.
SW_FAULT = 0x0008

#: Coulomb feed-forward 0x2381,3 (stm32 e1b0a4e): +-Tc by the sign of the
#: trajectory's velocity, 0 at rest, in rated torque / 1000.  The drag's Tc
#: times FRICTION_FF_FRACTION: under-compensated, since the drag fit is an
#: average and an over-compensated FF pushes the rotor through the target.
#: A Tc past FRICTION_FF_MAX_PERMILLE of rated is not a friction this drive
#: has (a gearhead's is a few per cent): capped, and the step warns.
#: Kept only if it cuts the PP following error (rms) by FRICTION_FF_MIN_GAIN
#: and adds at most POS_OVERSHOOT_MAX_PCT of the move to the overshoot.
#: Node 4 (P4-42, gearhead, 2026-10-21): Tc 10 cut the following error
#: 35-45 % and made the 0.1 deg settle 100-200 ms slower -- p4sysid's
#: settle ki, found with the ff on, is what takes that back.
FRICTION_FF_FRACTION = 0.8
FRICTION_FF_MAX_PERMILLE = 100.0
FRICTION_FF_MIN_GAIN = 0.1
FRICTION_FF_REPS = 2


def pp_move(cfg, top_rpm=None, deg=POS_STEP_DEG):
    """(counts, vmax counts/s, accel counts/s^2, move s) of the velff test's
    trapezoid."""
    res = cfg['enc_res']
    top = top_rpm or cfg.get('max_velocity') or 3000.0
    vmax = min(VELFF_TOP_FRACTION * top, VELFF_RPM_CAP) * res / 60.0
    cts = int(round(deg / 360.0 * res))
    kp = (cfg.get('pos_kp') or 0) * (cfg.get('pos_outmul') or 100) / 100.0
    if kp:
        vmax = min(vmax, VELFF_LAG_FRACTION * cts * kp)
    ramp = VELFF_RAMP_MIN_S
    wc = loops.velocity_wc_of(cfg.get('vel_kp') or 0, cfg.get('j_kg_m2') or 0,
                              cfg['rated_torque'], res) \
        if cfg.get('vel_kp') and cfg.get('j_kg_m2') else None
    if wc:
        ramp = max(ramp, VELFF_RAMP_TAUS / wc)
    acc = vmax / ramp
    j = cfg.get('j_kg_m2')
    if j:
        alpha = VELFF_TORQUE_FRACTION * cfg['rated_torque'] * 1e-3 / j
        acc = min(acc, alpha * res / (2 * math.pi))
    ta = vmax / acc
    if cts > vmax * ta:
        move = cts / vmax + ta
    else:
        move = 2.0 * math.sqrt(cts / acc)
    return cts, vmax, acc, move


def pp_following(dev, velff, cfg=None, top_rpm=None, deg=POS_STEP_DEG,
                 band_deg=None, tail_s=None):
    """A PP trapezoid each way at *velff* (commanded source): the drive's
    following error (0x6062 - 0x6064) during the move, the overshoot past
    the target after it, and when it settles in POS_BAND_PCT (or
    *band_deg*).  *tail_s* is the least capture after the move.  Returns
    dict(velff, ferr_max_deg, ferr_rms_deg, over_deg, settle_s)."""
    cfg = cfg or steptest.read_config(dev)
    per_deg = cfg['enc_res'] / 360.0
    cts, vmax, acc, move = pp_move(cfg, top_rpm, deg)
    words = steptest.CHANNELS['position']
    # The tail: long enough for the loop's own settle (steptest's position
    # window: kp and the velocity integrator), not just a fixed time.
    window_s = move + max(VELFF_TAIL_S, tail_s or 0.0,
                          steptest.position_window_s(cfg))
    steptest.write(dev, 'velff_src', VELFF_SRC_COMMANDED)
    steptest.write(dev, 'pos_velff', int(round(velff * 16384)))
    steptest.write(dev, 'profile_velocity', vmax)
    steptest.write(dev, 'profile_accel', acc)
    steptest.write(dev, 'profile_decel', acc)
    reps = []
    try:
        steptest._enable(dev, steptest.MODE_PP)
        steptest._sleep(dev, 0.3)
        for sgn in (1, -1):
            p0 = steptest.read(dev, 'position')
            steptest.write(dev, 'target_position', p0 + sgn * cts)
            t, (cmd, act) = steptest._capture(dev, words, window_s,
                                              cfg['control_hz'],
                                              ('controlword', 0x3F), 0.1)
            steptest.write(dev, 'controlword', 0x0F)
            # From the start position read before the move, not the first
            # 0x6062 sample: node 2 logged one stale demand ~10 M counts off
            # (a 916 000 deg "following error").  A demand outside the move
            # is not one the drive followed: those samples are left out.
            rel = (cmd - p0) * sgn
            good = (rel >= -2) & (rel <= cts + 2)
            moving = np.nonzero(good & (rel > 0.5))[0]
            if not moving.size:
                raise StepTestError('the PP move never started')
            k0 = moving[0]
            done = np.nonzero(good & (np.abs(rel - cts) < 1.5))[0]
            k1 = done[done > k0][0] if (done > k0).any() else len(cmd) - 1
            e = np.where(good, (cmd - act) * sgn, 0.0) / per_deg   # lag positive
            past = (act[k1:] - (p0 + sgn * cts)) * sgn / per_deg
            band = band_deg if band_deg else POS_BAND_PCT / 100.0 * deg
            out = np.nonzero(np.abs(past) > band)[0]
            settle = (None if out.size and out[-1] == len(past) - 1 else
                      float(t[k1 + out[-1] + 1] - t[k1]) if out.size else 0.0)
            reps.append({'ferr_max_deg': float(np.max(np.abs(e[k0:k1 + 1]))),
                         'ferr_rms_deg': float(np.sqrt(np.mean(e[k0:k1 + 1] ** 2))),
                         'over_deg': float(max(0.0, past.max())),
                         'settle_s': settle})
            steptest._sleep(dev, 0.1)
    finally:
        steptest._stop(dev)
    settles = [r['settle_s'] for r in reps]
    return {'velff': velff, 'reps': reps,
            'ferr_max_deg': max(r['ferr_max_deg'] for r in reps),
            'ferr_rms_deg': max(r['ferr_rms_deg'] for r in reps),
            'over_deg': max(r['over_deg'] for r in reps),
            'settle_s': None if None in settles else max(settles)}


def _faulted(dev):
    try:
        d = bytes(dev.node.sdo.upload(0x6041, 0))
    except Exception:          # noqa: BLE001 -- no statusword: no verdict
        return False
    return bool(struct.unpack('<H', d[:2].ljust(2, b'\0'))[0] & SW_FAULT)


def _velff_now(dev):
    return steptest.read(dev, 'pos_velff') / 16384.0


def friction_ff(dev, tc_mnm, rated_mnm=None, cfg=None, top_rpm=None,
                log=None, reps=FRICTION_FF_REPS):
    """The Coulomb feed-forward 0x2381,3 from the drag's Tc (mNm): PP moves
    with it off and on, interleaved, at the drive's own velff.  The value
    kept (or 0) is left in RAM.  Returns dict(tc_mnm, permille, capped,
    off, on, kept, value)."""
    log = log or dev._log
    cfg = cfg or steptest.read_config(dev)
    rated = rated_mnm or cfg['rated_torque']
    if not rated:
        raise StepTestError('no RatedTorque: the feed-forward has no unit')
    pm = FRICTION_FF_FRACTION * tc_mnm / float(rated) * 1000.0
    capped = pm > FRICTION_FF_MAX_PERMILLE
    pm = min(pm, FRICTION_FF_MAX_PERMILLE)
    out = {'tc_mnm': tc_mnm, 'rated_mnm': rated, 'permille': pm,
           'capped': capped, 'kept': False, 'value': 0.0}
    if pm <= 0:
        steptest.write(dev, 'vel_tc_ff', 0.0)
        return out
    velff = _velff_now(dev)
    rows = {0.0: [], pm: []}
    try:
        for _ in range(reps):
            for v in (0.0, pm):
                steptest.write(dev, 'vel_tc_ff', v)
                r = pp_following(dev, velff, cfg, top_rpm)
                rows[v].append(r)
                log('refine: Coulomb ff {:.2f}: following error rms {:.3f} '
                    'max {:.3f} deg, over {:.3f} deg, settle {}'.format(
                        v, r['ferr_rms_deg'], r['ferr_max_deg'], r['over_deg'],
                        '-' if r['settle_s'] is None else
                        '{:.0f} ms'.format(r['settle_s'] * 1e3)))
                if _faulted(dev):
                    raise StepTestError('fault with the Coulomb ff at {:.2f}'
                                        .format(v))
    except Exception:
        steptest.write(dev, 'vel_tc_ff', 0.0)
        raise

    def agg(rs):
        st = [r['settle_s'] for r in rs]
        return {'ferr_rms_deg': float(np.median([r['ferr_rms_deg'] for r in rs])),
                'ferr_max_deg': float(np.median([r['ferr_max_deg'] for r in rs])),
                'over_deg': max(r['over_deg'] for r in rs),
                'settle_s': None if None in st else float(np.median(st))}
    off, on = agg(rows[0.0]), agg(rows[pm])
    out.update(off=off, on=on)
    out['kept'] = (on['settle_s'] is not None
                   and on['ferr_rms_deg'] <= (1 - FRICTION_FF_MIN_GAIN) * off['ferr_rms_deg']
                   and on['over_deg'] <= off['over_deg']
                   + POS_OVERSHOOT_MAX_PCT / 100.0 * POS_STEP_DEG)
    out['value'] = pm if out['kept'] else 0.0
    steptest.write(dev, 'vel_tc_ff', out['value'])
    return out


def summary_friction_ff(r):
    if not r['permille']:
        return 'no Coulomb friction measured: ff off'
    off, on = r['off'], r['on']
    txt = ('Tc {:.2f} mNm -> {:.2f}/1000 of {:.0f} mNm{}; following error rms '
           '{:.3f} -> {:.3f} deg, settle {} -> {} ms'.format(
               r['tc_mnm'], r['permille'], r['rated_mnm'],
               ' (capped)' if r['capped'] else '', off['ferr_rms_deg'],
               on['ferr_rms_deg'], _ms(off['settle_s']), _ms(on['settle_s'])))
    return txt + ('' if r['kept'] else '; not kept: ff off')


def _ms(x):
    return '-' if x is None else '{:.0f}'.format(x * 1e3)
