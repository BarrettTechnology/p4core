"""The drive's loop and motor parameters by name (OD), over SDO, and the
enable / stop sequence for running it in a CiA 402 mode.

read() and write() convert to and from the OD's own format; read_config()
gathers what the calibrations size their spins from.  Nothing here writes
NVM.
"""

import math
import struct


from . import loops

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
    write(dev, 'controlword', 0x80)
    write(dev, 'controlword', 0x06)
    write(dev, 'controlword', 0x0F)
    write(dev, 'mode', mode)
    _sleep(dev, 0.05)


def _stop(dev):
    for key, v in (('target_torque', 0), ('target_velocity', 0),
                   ('mode', MODE_IDLE), ('controlword', 0x06)):
        try:
            write(dev, key, v)
        except Exception:
            pass
