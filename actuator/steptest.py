"""The drive's loop and motor parameters by name (OD), over SDO, and the
enable / stop sequence for running it in a CiA 402 mode.

read() and write() convert to and from the OD's own format; read_config()
gathers what the calibrations size their spins from.  _capture() arms the
0x3004 logger on named channels and reads the record back.  Nothing here
writes NVM.
"""

import math
import struct
import time

import numpy as np

from . import loops, playback

MODE_IDLE, MODE_PP, MODE_PT, MODE_CSP, MODE_CSV = 0, 1, 4, 8, 9
SYNC_HZ = 1000

#: (index, sub) -> struct format
OD = {
    'controlword': (0x6040, 0, 'H'),
    'mode': (0x6060, 0, 'b'),
    'target_torque': (0x6071, 0, 'h'),
    'torque_slope': (0x6087, 0, 'I'),
    'target_velocity': (0x60FF, 0, 'i'),
    'target_position': (0x607A, 0, 'i'),
    'position': (0x6064, 0, 'i'),
    'velocity': (0x606C, 0, 'i'),
    'rated_torque': (0x6076, 0, 'I'),
    'max_velocity': (0x6080, 0, 'I'),
    'pwm_hz': (0x3001, 1, 'I'),
    'control_hz': (0x3001, 14, 'I'),
    'enc_res': (0x3013, 1, 'I'),
    'kt': (0x3011, 14, 'f'),          # mNm/A rms (read_config: ,4 on older firmware)
    'rt': (0x3011, 5, 'H'),           # mohm l-l
    'lq': (0x3011, 12, 'H'),          # uH l-l
    'j': (0x3011, 7, 'I'),            # g.cm^2 x 32768
    'i_peak': (0x3011, 9, 'H'),       # mA
    'cur_kp': (0x2380, 1, 'f'),
    'cur_ki': (0x2380, 2, 'f'),
    'vel_kp': (0x2381, 1, 'f'),
    'vel_ki': (0x2381, 2, 'f'),
    'vel_tc_ff': (0x2381, 3, 'f'),    # Coulomb ff, rated/1000 (stm32 e1b0a4e)
    'settle_ki': (0x2381, 4, 'f'),    # settle ki (stm32 cfaea4d), 0 off
    'pos_kp': (0x2382, 1, 'H'),
    'pos_outmul': (0x2382, 4, 'H'),
    'speed_filter_hz': (0x2100, 1, 'H'),
    'vel_input_hz': (0x2100, 2, 'H'),     # velocity command biquad, 0 off
    'vel_output_hz': (0x2100, 3, 'H'),    # torque demand biquad, 0 off
    'pos_velff': (0x2382, 2, 'H'),        # Q2.14: 16384 = 1.0
    'velff_src': (0x2382, 5, 'B'),        # 0 commanded (RAM; never 1 = measured)
    'profile_velocity': (0x6081, 0, 'I'),
    'profile_accel': (0x6083, 0, 'I'),
    'profile_decel': (0x6084, 0, 'I'),
    'following_error': (0x60F4, 0, 'i'),
    'log_state': (0x3004, 1, 'B'),
    'log_records': (0x3004, 2, 'H'),
    'log_decimate': (0x3004, 3, 'H'),
    'log_mode': (0x3004, 15, 'B'),
    'log_cursor': (0x3004, 4, 'H'),
    'log_interval': (0x3004, 7, 'H'),
    'ident_subs': (0x3010, 0, 'B'),   # 13 from the firmware with DSTEP
    'inj_mode': (0x3015, 1, 'B'),
    'inj_current': (0x3015, 2, 'H'),  # mA
    'inj_ms': (0x3015, 8, 'H'),       # the dead-man
}


# ---------------------------------------------------------------------------
# Drive access: raw SDO by index, so no EDS name is needed
# ---------------------------------------------------------------------------
def _fmt(fmt):
    return '<' + fmt


def read(dev, key):
    idx, sub, fmt = OD[key]
    data = bytes(dev.node.sdo.upload(idx, sub))
    size = struct.calcsize(fmt)
    return struct.unpack(_fmt(fmt), data[:size].ljust(size, b'\0'))[0]


def write(dev, key, value):
    idx, sub, fmt = OD[key]
    if fmt not in 'fd':
        value = int(round(value))
    dev.node.sdo.download(idx, sub, struct.pack(_fmt(fmt), value))


def read_config(dev):
    """What the designs and tests need from the drive, in SI where it has
    units: rated torque (mNm), res, J (kg m^2), Kt (Nm/A rms), R and Lq per
    phase, the gains, the speed filter, the rates."""
    from p4core import firmware
    from p4core.calibrate import read_kt, rated_torque_mnm
    firmware.require(dev.node, 'capture', 'lq', what='Step tests and loop tuning')
    c = {k: read(dev, k) for k in (
        'rated_torque', 'enc_res', 'rt', 'lq', 'j', 'i_peak', 'pwm_hz',
        'cur_kp', 'cur_ki', 'vel_kp', 'vel_ki', 'pos_kp', 'pos_outmul',
        'speed_filter_hz', 'max_velocity')}
    try:
        c['control_hz'] = read(dev, 'control_hz') or c['pwm_hz'] / 4.0
    except Exception:
        c['control_hz'] = c['pwm_hz'] / 4.0
    c['kt'] = read_kt(dev.node)
    # Unrounded where the drive derives it (0x6076 publishes whole mNm).
    c['rated_torque'] = rated_torque_mnm(dev.node)
    c['j_kg_m2'] = c['j'] / 32768.0 * 1e-7
    c['kt_nm_per_a'] = c['kt'] * 1e-3
    c['r_ohm'] = c['rt'] * 1e-3 / 2.0
    c['lq_h'] = c['lq'] * 1e-6 / 2.0
    # The current loop's bandwidth as designed (pole-zero: wc = Kp / L).
    c['current_bw_hz'] = (c['cur_kp'] / c['lq_h'] / (2 * math.pi)
                          if c['lq_h'] > 0 and c['cur_kp'] > 0 else None)
    c['vel_output_limit'] = velocity_output_limit(dev.node)
    return c


def velocity_output_limit(node):
    """The velocity PI's output limit, per-mille of RatedTorque: the working
    peak's per-mille (1000 x 0x3011,15 / i_cont on rated-unit firmware, 3000
    on a P4-42; 1000 before, where rated meant peak), under MaxTorque 0x6072
    when that is set lower (65535 = no limit)."""
    from p4core.calibrate import id_scale
    try:
        lim = float(id_scale(node)[1]) or loops.VEL_OUTPUT_LIMIT
    except Exception:
        lim = loops.VEL_OUTPUT_LIMIT
    try:
        mt = int.from_bytes(bytes(node.sdo.upload(0x6072, 0))[:2], 'little')
    except Exception:
        mt = 0
    if 0 < mt < lim:
        lim = float(mt)
    return lim


def _sleep(dev, seconds):
    dev._stream_sleep(seconds)


def _enable(dev, mode):
    from p4core import cia402
    cia402.enable(dev.node, 'Enable')
    write(dev, 'mode', mode)
    _sleep(dev, 0.05)


def _stop(dev):
    for key, v in (('target_torque', 0), ('target_velocity', 0),
                   ('mode', MODE_IDLE), ('controlword', 0x06)):
        try:
            write(dev, key, v)
        except Exception:
            pass


# ---------------------------------------------------------------------------
# The 0x3004 logger: a capture of named channels around a step
# ---------------------------------------------------------------------------
#: Logger memory, bytes (stm32 app/log.h LOG_BUF_BYTES).
LOG_BUF_BYTES = 2048
LOG_IDLE, LOG_COLLECT, LOG_FULL = 0, 1, 2
LOG_MODE_RAW = 0
OD_LOG_RECORD = (0x3004, 5)
TPDO_COMM, TPDO_MAP = 0x1803, 0x1A03    # TPDO4 names the logger's channels

#: Channels per domain: (demand, response), as PDO mapping words.
CHANNELS = {
    'current': (0x60740010, 0x30100910),
    'current_d': (0x30100D10, 0x30100C10),   # id reference, id (mA)
    'velocity': (0x606B0020, 0x606C0020),
    'position': (0x60620020, 0x60640020),
    # measure_loop_delay: the command (to find the step), the velocity PI's
    # torque demand and the speed it feeds back on
    'loop': (0x606B0020, 0x60740010, 0x606C0020),
}


class StepTestError(RuntimeError):
    pass


class _ChannelMap:
    """TPDO4's mapping naming the logger channels, restored afterwards.  Left
    disabled: the logger latches the channels at arm, nothing is sent."""

    def __init__(self, dev, words):
        self.dev, self.words = dev, words

    def _w(self, idx, sub, fmt, v):
        self.dev.node.sdo.download(idx, sub, struct.pack('<' + fmt, v))

    def _r(self, idx, sub, fmt):
        d = bytes(self.dev.node.sdo.upload(idx, sub))
        n = struct.calcsize(fmt)
        return struct.unpack('<' + fmt, d[:n].ljust(n, b'\0'))[0]

    def __enter__(self):
        cob = self._r(TPDO_COMM, 1, 'I')
        n = self._r(TPDO_MAP, 0, 'B')
        self.saved = (cob, n, [self._r(TPDO_MAP, k + 1, 'I')
                               for k in range(max(n, len(self.words)))])
        self._w(TPDO_COMM, 1, 'I', cob | 0x80000000)
        self._w(TPDO_MAP, 0, 'B', 0)
        for k, word in enumerate(self.words):
            self._w(TPDO_MAP, k + 1, 'I', word)
        self._w(TPDO_MAP, 0, 'B', len(self.words))
        return self

    def __exit__(self, *exc):
        cob, n, slots = self.saved
        try:
            self._w(TPDO_COMM, 1, 'I', cob | 0x80000000)
            self._w(TPDO_MAP, 0, 'B', 0)
            for k, word in enumerate(slots):
                self._w(TPDO_MAP, k + 1, 'I', word)
            self._w(TPDO_MAP, 0, 'B', n)
            self._w(TPDO_COMM, 1, 'I', cob)
        except Exception:
            pass


def _record_bytes(words):
    return sum((w & 0xFF) // 8 for w in words)


def _fire(dev, key, value):
    """Expedited SDO download, not waiting for the answer: the arm and the
    step keep their spacing on the wire, which a round trip each (1-3 ms
    through the OS) would not -- and the current step's capture is only a
    few ms long.  The answers are drained afterwards (_drain)."""
    idx, sub, fmt = OD[key]
    if fmt not in 'fd':
        value = int(round(value))
    data = struct.pack(_fmt(fmt), value)
    cmd = 0x23 | ((4 - len(data)) << 2)
    dev.node.network.send_message(
        0x600 + dev.node.id,
        bytes([cmd, idx & 0xFF, idx >> 8, sub]) + data.ljust(4, b'\0'))


def _drain(dev):
    time.sleep(0.05)
    q = getattr(dev.node.sdo, 'responses', None)
    while q is not None and not q.empty():
        q.get_nowait()


def _virtual(dev):
    """A network on virtual time (fake_puck, the twin): no fire-and-forget."""
    return getattr(dev.node.network, 'advance', None) is not None


def _capture(dev, words, window_s, ctl_hz, step=None, pre_fraction=0.2,
             bias=None, then=None):
    """Arm a capture of about *window_s*, write *step* = (key, value)
    pre_fraction of the way in, wait for it to fill and read it back.
    *step* is None for a steady hold (hold_noise): the whole window is then
    left alone, so what comes back is noise, not a transient.  *then* =
    (key, value, after_s) writes again that long after the step: a torque
    pulse, which cannot wait for the readback to end.  Returns (t, [channel
    arrays])."""
    from p4core import firmware
    firmware.require(dev.node, 'capture', what='A step-test capture')
    rec_bytes = _record_bytes(words)
    capacity = LOG_BUF_BYTES // rec_bytes
    decimate = max(1, int(math.ceil(window_s * ctl_hz / capacity)))
    pwm = read(dev, 'pwm_hz')
    with _ChannelMap(dev, words):
        write(dev, 'log_state', LOG_IDLE)
        # Raw: the mode is latched at arm and outlives it, and a statistics
        # run left in place reads back min/max/mean/sd/count as "records".
        write(dev, 'log_mode', LOG_MODE_RAW)
        write(dev, 'log_records', capacity)
        write(dev, 'log_decimate', decimate)
        dt = decimate / float(ctl_hz)
        pre = pre_fraction * capacity * dt
        # *bias* = (key, value, hold_s): set only now, after the capture's
        # own setup, and held just hold_s before the arm -- a q bias on a
        # free rotor spins it up for as long as it is held.
        if bias is not None and _virtual(dev):
            write(dev, bias[0], bias[1])
            _sleep(dev, bias[2])
        elif bias is not None:
            _fire(dev, bias[0], bias[1])
            end = time.perf_counter() + bias[2]
            while time.perf_counter() < end:
                pass
        if _virtual(dev):
            write(dev, 'log_state', LOG_COLLECT)
            if step is not None:
                _sleep(dev, pre)
                write(dev, step[0], step[1])
                if then is not None:
                    _sleep(dev, then[2])
                    write(dev, then[0], then[1])
        else:
            _fire(dev, 'log_state', LOG_COLLECT)
            if step is not None:
                end = time.perf_counter() + pre
                while time.perf_counter() < end:
                    pass
                _fire(dev, step[0], step[1])
                if then is not None:
                    end = time.perf_counter() + then[2]
                    while time.perf_counter() < end:
                        pass
                    _fire(dev, then[0], then[1])
            # Both fired downloads' answers, before the next SDO request
            # takes one of them for its own (a hold with no step too).
            _drain(dev)
        # 0x3004,7: PWM periods per record, latched at arm -- the real interval.
        interval = read(dev, 'log_interval')
        if interval:
            dt = interval / float(pwm)
        deadline = time.monotonic() + 10.0 * capacity * dt + 2.0
        while read(dev, 'log_state') != LOG_FULL:
            dev._check_abort()
            _sleep(dev, 0.02)
            if time.monotonic() > deadline:
                raise StepTestError('the capture never filled: the control '
                                    'loop is not running')
        n = len(words)
        recs = playback.read(dev.node, words, capacity,
                             log=getattr(dev, '_log', None),
                             should_abort=dev._check_abort)
        if recs is not None:
            vals = [v for rec in recs for v in rec]
        else:
            write(dev, 'log_cursor', 0)
            vals = []
            for _ in range(capacity * n):
                d = bytes(dev.node.sdo.upload(*OD_LOG_RECORD))
                vals.append(struct.unpack('<i', d[:4].ljust(4, b'\0'))[0])
    chans = [np.array(vals[k::n], float) for k in range(n)]
    return np.arange(capacity) * dt, chans


#: Velocity and position captures cover this many velocity-integrator time
#: constants (1/wi) as well as the crossover's: a rotor that stalls on
#: friction short of the target sits there until the integrator winds
#: through it, and an overshoot decays at wi.  An EC16 (Tc/J
#: ~2000 rad/s^2) stalled 5-15 % short at 0.14 s and was within 1 % by
#: 0.4 s (wi 5-13 rad/s).
POS_WINDOW_INTEG_TAUS = 6.0


def _integ_window_s(cfg):
    """POS_WINDOW_INTEG_TAUS velocity-integrator time constants, seconds
    (0 without a PI)."""
    vkp, vki = cfg.get('vel_kp'), cfg.get('vel_ki')
    if not (vkp and vki and vkp > 0 and vki > 0):
        return 0.0
    return POS_WINDOW_INTEG_TAUS * vkp / (2.0 * loops.VEL_LOOP_HZ * vki)


def position_window_s(cfg):
    """Capture length for a position step at *cfg*'s gains, seconds."""
    kp = (cfg.get('pos_kp') or 0) * (cfg.get('pos_outmul') or 100) / 100.0
    return min(1.0, max(0.12, 8.0 / (kp or 10.0), _integ_window_s(cfg)))
