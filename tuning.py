"""tuning.py — pucktuner's controller-gain design, without the GUI.

The current loop is a PI placed at control_gain x the motor's L/R corner,
with the operating PWM chosen so the discrete control rate keeps a margin
over that bandwidth; the velocity loop is a PI at a fixed (capability-scaled)
bandwidth under the velocity filter; the position Kp sits a decade below
the velocity bandwidth, hard-capped.  The numbers and the reasoning behind
every constant are pucktuner's (pucktunerapp.calculate_gains), moved here
so pucktuner's GUI and command line and p4gui's Tune tab share one design.

Units follow the drive's objects: R and L line-to-line in ohm and H, torque
in mNm, J in g.cm^2, speeds in rpm.  Every function logs its reasoning
through *log* (print by default) the way pucktuner always has.

    params = read_params(node)
    cur = current_gains(**current_inputs(params))
    vel = velocity_gains(**velocity_inputs(params, vel_filter_hz=...,
                                          current_margin=cur.margin))
"""

import collections
import math
import struct

#: control_bw = control_gain x natural (L/R) bandwidth; 1.0 is the corner.
DEFAULT_CURRENT_CONTROL_GAIN = 1.0
#: The PWM selector's target: control rate >= this x current-loop bandwidth
#: (bench: ~17x oscillated at voltage saturation, ~34x was stable).
SATURATION_STABILITY_MARGIN = 25.0
#: The softer floor the gain back-off restores once the PWM ceiling is hit.
STABILITY_BACKOFF_MARGIN = 20.0
#: Kp = (2 gain - 1) Rdq at zeta 1 dies at 0.5; the back-off stops here.
MIN_CURRENT_GAIN = 0.6
#: The hardware/firmware control-rate ceiling.
MAX_OPERATING_PWM = 100000
PWM_CANDIDATES = (16000, 32000, 40000, 60000, 80000, MAX_OPERATING_PWM)
PATTERNS_PER_CYCLE = 4
#: Effective L fraction seen by the current loop.
PWM_FACTOR = 0.8
#: Winding temperature assumed for R (copper, 0.39 %/K from nominal).
ACTUAL_TEMP_C = 45
NOMINAL_TEMP_C = 25

#: Velocity loop: a fixed target scaled toward the filter-limited cap.
VEL_BW_TARGET_HZ = 4.0
VEL_BW_FILTER_DIVISOR = 10.0
VEL_BW_SCALE = 0.8
VEL_BW_CEILING = 20.0
VEL_DT = 1.0 / 1000
#: Below this current-loop margin the velocity target is gentled.
ROUGH_MARGIN = 25.0

#: Position Kp: velocity bandwidth / 10, clamped to [1, 3].
POS_CASCADE_RATIO = 10.0
POS_KP_MIN, POS_KP_MAX = 1, 3

CTS2RAD = math.pi * 2 / 4096

#: Where the gains and their design factors live.
CURRENT_KP = (0x2380, 1)
CURRENT_KI = (0x2380, 2)
VELOCITY_KP = (0x2381, 1)
VELOCITY_KI = (0x2381, 2)
POSITION_KP = (0x2382, 1)
CONTROL_GAIN = (0x3024, 2)          # U8, 0.01
ZETA_CURRENT = (0x3024, 3)          # U8, 0.01
VEL_BANDWIDTH = (0x3024, 4)         # U8, 0.1 Hz (0 = auto)
ZETA_VELOCITY = (0x3024, 5)         # U8, 0.01
VEL_FILTER = (0x2100, 1)            # Hz

CurrentTune = collections.namedtuple(
    'CurrentTune', 'kp ki control_bw natural_bw control_gain margin clamped '
    'tau ldq rdq control_rate')
VelocityTune = collections.namedtuple(
    'VelocityTune', 'kp ki bandwidth_hz practical_max_hz io_factor b j_si bj '
    'spd_trq_grad')
PwmChoice = collections.namedtuple('PwmChoice',
                                   'pwm_hz control_rate tau ldq rdq')


def _say(log):
    return log if log is not None else print


# ---------------------------------------------------------------------------
# current loop
# ---------------------------------------------------------------------------
def dq(resistance, inductance, actual_temp=ACTUAL_TEMP_C,
       nominal_temp=NOMINAL_TEMP_C, pwm_factor=PWM_FACTOR):
    """(Rdq thermally adjusted, Ldq, tau) from line-to-line R (ohm), L (H)."""
    rdq = resistance / 2 * (1 + 0.0039 * (actual_temp - nominal_temp))
    ldq = inductance / 2 * pwm_factor
    return rdq, ldq, (ldq / rdq) if rdq else 0.0


def select_pwm(resistance, inductance, control_gain=DEFAULT_CURRENT_CONTROL_GAIN,
               stored_pwm=16000, user_pwm=0, patterns_per_cycle=PATTERNS_PER_CYCLE,
               log=None, **dq_kw):
    """The lowest standard PWM that resolves tau and keeps the control rate
    SATURATION_STABILITY_MARGIN x above the current-loop bandwidth.

    *user_pwm* > 0 always wins; with no valid tau the *stored_pwm* is kept
    when it is in range, else 16 kHz.
    """
    say = _say(log)
    rdq, ldq, tau = dq(resistance, inductance, **dq_kw)
    if user_pwm > 0:
        pwm = user_pwm
    elif tau > 0:
        required_tau = (5 / (4 * tau * 1000) * 1000) * 5
        control_bw = control_gain / (tau * 2 * math.pi)
        required_stab = SATURATION_STABILITY_MARGIN * control_bw * patterns_per_cycle
        required = max(required_tau, required_stab)
        binding = 'stability' if required_stab >= required_tau else 'tau'
        pwm = next((f for f in PWM_CANDIDATES if f >= required), MAX_OPERATING_PWM)
        achieved = ((pwm / patterns_per_cycle) / control_bw) if control_bw else 0.0
        say('Auto PWM selection:')
        say('  tau={:.3f} ms (Ldq/Rdq); current-loop bandwidth={:.0f} Hz '
            '(control_gain {:.2f})'.format(tau * 1000, control_bw, control_gain))
        say('  req(tau: resolve electrical dynamics)      >= {:>6.0f} Hz'.format(required_tau))
        say('  req(stability: control_rate >= {:.0f}x bw)    >= {:>6.0f} Hz'.format(
            SATURATION_STABILITY_MARGIN, required_stab))
        say('  binding = {:<9s} candidates {} -> SELECTED {} Hz'.format(
            binding, list(PWM_CANDIDATES), pwm))
        verdict = ('OK' if achieved >= SATURATION_STABILITY_MARGIN
                   else 'MARGINAL: PWM ceiling reached - lower control_gain for more margin')
        say('  -> control_rate {:.0f} Hz = {:.1f}x bandwidth (target >= {:.0f}x): {}'.format(
            pwm / patterns_per_cycle, achieved, SATURATION_STABILITY_MARGIN, verdict))
    else:
        pwm = stored_pwm if 16000 <= stored_pwm <= MAX_OPERATING_PWM else 16000
    return PwmChoice(pwm, pwm / patterns_per_cycle, tau, ldq, rdq)


def current_gains(resistance, inductance, control_gain=DEFAULT_CURRENT_CONTROL_GAIN,
                  zeta=1.0, pwm_hz=None, user_pwm=0, stored_pwm=16000,
                  override_backoff=False, patterns_per_cycle=PATTERNS_PER_CYCLE,
                  log=None, **dq_kw):
    """The current-loop PI.  Selects the PWM (select_pwm) unless *pwm_hz* is
    given, then applies the Nyquist clamp (40 % of control_rate/2) and the
    stability back-off (unless *override_backoff*).  The returned
    control_gain is the effective one — store it (0x3024:2) with the gains.
    """
    say = _say(log)
    if pwm_hz is None:
        choice = select_pwm(resistance, inductance, control_gain,
                            stored_pwm=stored_pwm, user_pwm=user_pwm,
                            patterns_per_cycle=patterns_per_cycle, log=log, **dq_kw)
    else:
        rdq, ldq, tau = dq(resistance, inductance, **dq_kw)
        choice = PwmChoice(pwm_hz, pwm_hz / patterns_per_cycle, tau, ldq, rdq)
    tau, ldq, rdq, control_rate = choice.tau, choice.ldq, choice.rdq, choice.control_rate
    if tau <= 0:
        raise ValueError('R and L must be positive to design the current loop')

    natural_bw = 1 / (tau * 2 * math.pi)
    control_bw = control_gain * natural_bw
    max_bw = 0.40 * (control_rate / 2.0)
    say('  Current-loop bandwidth: control_gain={:.2f} x natural(L/R) {:.0f} Hz = {:.0f} Hz'
        '  -> Ki is {:.0f}% of the control_gain=1.0 value{}'.format(
            control_gain, natural_bw, control_bw, 100.0 * control_gain ** 2,
            '' if control_gain >= 1.0 else '  [conservative back-off]'))

    clamped = False
    if control_bw > max_bw:
        requested = control_bw
        eff_gain = max_bw / natural_bw
        say('  CLAMP (Nyquist): control_bw {:.0f} Hz exceeds 40% Nyquist {:.0f} Hz '
            '(control_rate {:.0f} Hz) -> clamped to {:.0f} Hz '
            '({:.0f}% reduction; effective control_gain {:.2f}).'.format(
                requested, max_bw, control_rate, max_bw,
                100.0 * (1.0 - max_bw / requested), eff_gain))
        control_bw, control_gain, clamped = max_bw, eff_gain, True

    margin = control_rate / control_bw if control_bw > 0 else 1e9
    if override_backoff:
        if margin < STABILITY_BACKOFF_MARGIN:
            say('  ADVANCED OVERRIDE: stability back-off SKIPPED — control_bw {:.0f} Hz '
                'at {:.1f}x margin (< {:.0f}x target). The loop may ring / limit-cycle '
                'at voltage saturation; lower the gain factor if it does.'.format(
                    control_bw, margin, STABILITY_BACKOFF_MARGIN))
        else:
            say('  ADVANCED OVERRIDE active — control_bw {:.0f} Hz at {:.1f}x margin '
                '(already >= {:.0f}x target).'.format(
                    control_bw, margin, STABILITY_BACKOFF_MARGIN))
    elif margin < STABILITY_BACKOFF_MARGIN:
        bw_target = control_rate / STABILITY_BACKOFF_MARGIN
        floored = (bw_target / natural_bw) < MIN_CURRENT_GAIN
        if floored:
            bw_target = MIN_CURRENT_GAIN * natural_bw
        # The back-off may only lower the bandwidth: never back above Nyquist.
        bw_target = min(bw_target, control_bw)
        eff_gain = bw_target / natural_bw
        if floored:
            say('  (stability back-off FLOORED: 20x margin needs a gain below the Kp>0 '
                'limit — motor L/R too fast for the {:.0f} Hz control rate at the 100 kHz '
                'ceiling; running at gain {:.2f}, best achievable margin ~{:.1f}x).'.format(
                    control_rate, eff_gain, control_rate / bw_target))
        if bw_target < control_bw - 1.0:
            say('  CLAMP (stability margin): control_rate {:.0f} Hz = {:.1f}x bw {:.0f} Hz '
                '< {:.0f}x target -> bandwidth backed off to {:.0f} Hz '
                '(effective control_gain {:.2f}).'.format(
                    control_rate, margin, control_bw, STABILITY_BACKOFF_MARGIN,
                    bw_target, eff_gain))
            control_bw, control_gain, clamped = bw_target, eff_gain, True
        else:
            say('  (stability back-off: Nyquist clamp already bounds bw at {:.0f} Hz '
                '(gain {:.2f}); a deeper back-off would push above 40% Nyquist and '
                'alias, so held here — this motor is at its 100 kHz limit).'.format(
                    control_bw, control_gain))

    kp = 4 * math.pi * control_bw * zeta * ldq - rdq
    ki = (2 * math.pi * control_bw) ** 2 * ldq
    say('  Current gains: control_bw={:.0f} Hz  zeta={:.2f}  Kp={:.5f}  Ki={:.2f}'.format(
        control_bw, zeta, kp, ki))
    if kp < 0:
        say('  CLAMP (Kp<0): Kp_current = {:.6f} negative -- motor L/R too small for this '
            'bandwidth/zeta; clamped to 0. Lower control_gain or raise zeta.'.format(kp))
        kp = 0
    return CurrentTune(kp, ki, control_bw, natural_bw, control_gain,
                       control_rate / control_bw if control_bw > 0 else 999.0,
                       clamped, tau, ldq, rdq, control_rate)


# ---------------------------------------------------------------------------
# velocity and position loops
# ---------------------------------------------------------------------------
def spd_trq_gradient(no_load_spd, nominal_spd, cont_torque):
    """rpm/mNm from the no-load and nominal speeds at continuous torque."""
    return (no_load_spd - nominal_spd) / cont_torque


def _practical_max(vel_filter_hz):
    """The velocity bandwidth cap: a tenth of the loop's Nyquist and of the
    velocity feedback filter (5 Hz when the filter is unknown)."""
    nyquist_hz = 0.5 / VEL_DT
    if vel_filter_hz > 0:
        return min(0.1 * nyquist_hz, vel_filter_hz / VEL_BW_FILTER_DIVISOR)
    return min(0.1 * nyquist_hz, 5.0)


def auto_velocity_bandwidth(vel_filter_hz, current_margin=999.0, log=None):
    """(bandwidth Hz, practical max Hz, target Hz, gentled?).

    A fixed 4 Hz target (Kp then scales with J, not friction), scaled up to
    0.8 of the filter-limited cap on wide-filter motors, gentled when the
    current loop is margin-limited, capped at filter/10 and floored at 1.5 Hz.
    """
    practical_max = _practical_max(vel_filter_hz)
    target = min(max(VEL_BW_TARGET_HZ, VEL_BW_SCALE * practical_max), VEL_BW_CEILING)
    rough = current_margin < ROUGH_MARGIN
    if rough:
        target = target * max(0.4, current_margin / ROUGH_MARGIN)
    bw = max(round(min(target, practical_max), 1), 1.5)
    return bw, practical_max, target, rough


def velocity_gains(j, max_trq, spd_trq_grad=0.0, no_load_spd=0, nominal_spd=0,
                   cont_torque=0, zeta=1.0, bandwidth_hz=None, vel_filter_hz=0,
                   current_margin=999.0, gear_ratio=1.0, log=None):
    """The velocity-loop PI.  *j* in g.cm^2, *max_trq* (RatedTorque) in mNm.

    *bandwidth_hz* None designs it (auto_velocity_bandwidth); a number is
    the user's.  A zero/NaN *spd_trq_grad* is derived from the speeds.
    """
    say = _say(log)
    if not spd_trq_grad or math.isnan(spd_trq_grad):
        say('Calculating spd/trq gradient from velocities...')
        say('nominal = {} no load = {}'.format(nominal_spd, no_load_spd))
        say('max trq = {}'.format(max_trq))
        spd_trq_grad = spd_trq_gradient(no_load_spd, nominal_spd, cont_torque)
    if not max_trq:
        raise ValueError('RatedTorque (max torque) must be set to design the velocity loop')
    io_factor = 1000000 * CTS2RAD / max_trq
    b = 1 / (spd_trq_grad / 60 * 2 * math.pi * 1000)
    j_si = j / 10000000
    bj = b / j_si if j_si else 0.0

    practical_max = _practical_max(vel_filter_hz)
    if bandwidth_hz is None:
        bandwidth_hz, _max, target, rough = auto_velocity_bandwidth(
            vel_filter_hz, current_margin)
        say('  Auto bandwidth: {:.1f} Hz  (target {:.1f} Hz{}, vel-filter {} Hz / {:.0f}'
            ' → cap {:.1f} Hz; bj = {:.2f} Hz diag)'.format(
                bandwidth_hz, target,
                ' — GENTLED, current-loop margin {:.1f}x < {:.0f}x (rough inner loop)'.format(
                    current_margin, ROUGH_MARGIN) if rough else '',
                vel_filter_hz, VEL_BW_FILTER_DIVISOR, practical_max, bj / (2 * math.pi)))
    if bandwidth_hz > practical_max:
        say('Warning: bandwidth {:.1f} Hz exceeds practical max ({:.1f} Hz). May be unstable.'.format(
            bandwidth_hz, practical_max))
    omega_n = 2 * math.pi * bandwidth_hz
    kp = io_factor * 2 * zeta * omega_n * j_si
    ki = io_factor * j_si * omega_n ** 2 * VEL_DT
    say('  bj = {:.3g} Hz  gearRatio = {:.4g}  bandwidth = {:.1f} Hz'
        '  J = {:.0f} g*cm2  Kp = {:.6f}  Ki = {:.6f}'.format(
            bj / (2 * math.pi), gear_ratio, bandwidth_hz, j, kp, ki))
    if kp < 0:
        say('Kp negative: bandwidth {:.1f} Hz is below minimum {:.1f} Hz (bj/2ζ). Clamping to 0.'.format(
            bandwidth_hz, bj / (2 * math.pi * 2 * zeta)))
        kp = 0
    return VelocityTune(kp, ki, bandwidth_hz, practical_max, io_factor, b,
                        j_si, bj, spd_trq_grad)


def position_kp(velocity_bandwidth_hz):
    """A gentle cascade default for 0x2382:1: the velocity bandwidth over
    POS_CASCADE_RATIO in rad/s, clamped to [1, 3] (units unverified; a
    stiff position Kp browns out the supply on high-inertia motors)."""
    try:
        bw = float(velocity_bandwidth_hz)
    except (TypeError, ValueError):
        bw = 0.0
    if bw > 0:
        kp = int(round(2 * math.pi * bw / POS_CASCADE_RATIO))
        return max(POS_KP_MIN, min(POS_KP_MAX, kp))
    return 2


# ---------------------------------------------------------------------------
# the drive
# ---------------------------------------------------------------------------
def _u(node, index, sub):
    return int.from_bytes(node.sdo.upload(index, sub), 'little', signed=False)


def _f(node, index, sub):
    return struct.unpack('<f', node.sdo.upload(index, sub))[0]


def read_params(node):
    """What the design needs, as stored on the puck (pucktuner's import)."""
    cal = node.sdo['Calibration']
    stored_bw = _u(node, *VEL_BANDWIDTH)
    try:
        vel_filter = _u(node, *VEL_FILTER)
    except Exception:
        vel_filter = 0
    return dict(
        resistance=cal['rt'].raw / 1000,           # mOhm -> ohm, l-l
        inductance=cal['lt'].raw / 1000000,        # uH -> H, l-l
        kt=cal['kt'].raw,                          # mNm/A
        poles=cal['poles'].raw,
        max_trq=node.sdo['RatedTorque'].raw,       # mNm
        pwm_hz=_u(node, 0x3001, 1),
        j=_u(node, 0x3011, 7) / 2 ** 15,           # g.cm^2
        spd_trq_grad=_u(node, 0x3024, 1) / 100,    # rpm/mNm
        control_gain=_u(node, *CONTROL_GAIN) / 100,
        zeta_current=_u(node, *ZETA_CURRENT) / 100,
        bandwidth_hz=(stored_bw / 10.0) if stored_bw else None,
        zeta_vel=_u(node, *ZETA_VELOCITY) / 100,
        no_load_spd=_u(node, 0x3024, 6),
        nominal_spd=_u(node, 0x3024, 7),
        cont_torque=_u(node, 0x3024, 8),
        vel_filter_hz=vel_filter,
    )


def read_gains(node):
    return collections.OrderedDict([
        ('CurrentKp', _f(node, *CURRENT_KP)),
        ('CurrentKi', _f(node, *CURRENT_KI)),
        ('VelocityKp', _f(node, *VELOCITY_KP)),
        ('VelocityKi', _f(node, *VELOCITY_KI)),
        ('PositionKp', _u(node, *POSITION_KP)),
    ])


def write_current(node, tune):
    """Kp, Ki and the effective control gain (so a re-import agrees)."""
    node.sdo.download(*CURRENT_KP, struct.pack('<f', tune.kp))
    node.sdo.download(*CURRENT_KI, struct.pack('<f', tune.ki))
    gain = min(255, max(0, int(round(tune.control_gain * 100))))
    node.sdo.download(*CONTROL_GAIN, gain.to_bytes(1, 'little'))


def write_velocity(node, tune):
    node.sdo.download(*VELOCITY_KP, struct.pack('<f', tune.kp))
    node.sdo.download(*VELOCITY_KI, struct.pack('<f', tune.ki))


def write_position(node, kp):
    node.sdo.download(*POSITION_KP, int(kp).to_bytes(2, 'little'))


def design(params, control_gain=None, bandwidth_hz=None, user_pwm=0,
           override_backoff=False, log=None):
    """All three loops from read_params(): (CurrentTune, VelocityTune,
    position Kp).  *control_gain* overrides the stored one.  The velocity
    bandwidth is designed unless *bandwidth_hz* is given: like the GUI's
    Calculate, a stored bandwidth (params['bandwidth_hz']) is not reused."""
    cur = current_gains(
        params['resistance'], params['inductance'],
        control_gain=params['control_gain'] if control_gain is None else control_gain,
        zeta=params['zeta_current'] or 1.0, user_pwm=user_pwm,
        stored_pwm=params['pwm_hz'], override_backoff=override_backoff, log=log)
    vel = velocity_gains(
        params['j'], params['max_trq'], spd_trq_grad=params['spd_trq_grad'],
        no_load_spd=params['no_load_spd'], nominal_spd=params['nominal_spd'],
        cont_torque=params['cont_torque'], zeta=params['zeta_vel'] or 1.0,
        bandwidth_hz=bandwidth_hz, vel_filter_hz=params['vel_filter_hz'],
        current_margin=cur.margin, log=log)
    return cur, vel, position_kp(vel.bandwidth_hz)


def write_pwm(node, hz):
    node.sdo.download(0x3001, 1, int(hz).to_bytes(4, 'little'))


def apply(session, node_id, cur, vel=None, pos_kp=None, reporter=None,
          save=True, reboot=True):
    """Write a design to *node_id*: the current gains and effective control
    gain, the velocity gains and the position Kp when given, and the PWM the
    current loop was designed at when it differs (latched at boot, hence
    the Save.All and reboot)."""
    from .reporter import Reporter
    reporter = reporter if reporter is not None else Reporter()
    node = session.add_node(node_id)
    write_current(node, cur)
    if vel is not None:
        write_velocity(node, vel)
    if pos_kp is not None:
        write_position(node, pos_kp)
    pwm = int(round(cur.control_rate * PATTERNS_PER_CYCLE))
    if _u(node, 0x3001, 1) != pwm:
        reporter.note('PWM -> {} Hz'.format(pwm))
        write_pwm(node, pwm)
        if not reboot:
            reporter.warn('the PWM takes effect at the next boot')
    if save:
        reporter.status('Saving to NVM...')
        session.save_all(node)
    if reboot:
        reporter.status('Rebooting...')
        session.reset_node(node_id)
