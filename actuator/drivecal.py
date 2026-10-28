"""drivecal.py — node-level helpers shared by the board and encoder calibrations.

The calibrations moved here from puckutility's calibrate menu (settling,
slope, enctest, enctable) work on a canopen RemoteNode and a p4core Reporter,
not on a P4Drive: they are older than the sequencer and talk SDO directly.
These are the pieces more than one of them needs.
"""

from p4core.calibrate import firmware_version, read_bus_v
from p4core.can_backend import sdo_contention_message
from p4core.cia402 import MODE_IDLE
from p4core.reporter import Reporter

from .device import SysidError


class CalError(SysidError):
    """A board or encoder calibration failed or was declined."""


def reporter_or_default(reporter):
    return reporter if reporter is not None else Reporter()


def fw_at_least(node, major, minor, patch):
    return firmware_version(node) >= (major, minor, patch)


def save_single(node, index, sub):
    node.sdo['Save']['Single'].raw = (index << 8) | sub


def clear_offset_reg(node):
    """Zero the drive-gated iSense offset (0x3008,8 / 0x3009,8) if the
    firmware has it, so a cal measures raw current.

    The firmware applies this offset under any drive, the cal's own included,
    so a stale value corrupts the offset measurement and makes the voltage
    ramp overshoot (the current reads low, ud keeps rising, the real current
    rails, brown-out).  Same reason the slope (0x3008,7) is cleared before
    measuring.  Raw SDO (bypasses the EDS); the probe aborts on pre-v3
    firmware, so this is a no-op there."""
    try:
        node.sdo.upload(0x3008, 8)
    except Exception:
        return
    try:
        zero = (0).to_bytes(2, 'little', signed=True)
        node.sdo.download(0x3008, 8, zero)
        node.sdo.download(0x3009, 8, zero)
        save_single(node, 0x3008, 8)
        save_single(node, 0x3009, 8)
    except Exception:
        pass


def deenergise(node):
    """ud = 0 and IDLE, ignoring errors: the cleanup after any failure."""
    try:
        node.sdo['Motor']['ud'].raw = 0
    except Exception:
        pass
    try:
        node.sdo['SetModeOfOperation'].raw = MODE_IDLE
    except Exception:
        pass


#: seed_ud's first and last ud.  The cap is the fixed seed the cals used to
#: write: 0.85 V at 24 V, 40 mA in a 10 ohm P4-16 but 2 A in a 0.14 ohm P4-42.
SEED_UD_START = 250
SEED_UD_CAP = 2000


def seed_ud(node, imag_ma, target_ma, sleep, cap=SEED_UD_CAP):
    """Raise ud (voltage-angle mode, Theta_e set) from SEED_UD_START until
    *imag_ma*() reaches *target_ma* or ud reaches *cap*: enough current to
    pull the rotor to the field's angle, sized by the winding instead of a
    fixed voltage.  Returns the ud written."""
    ud = SEED_UD_START
    while True:
        node.sdo['Motor']['ud'].raw = ud
        sleep(0.05)
        if ud >= cap or (imag_ma() or 0.0) >= target_ma:
            return ud
        ud = min(cap, int(ud * 1.5))


def fault_message(node, exc):
    """Why a calibration stopped, for the user: bus contention, or the drive's
    StatusWord, temperature and bus voltage after a fault.  De-energises."""
    contention = sdo_contention_message(exc)
    deenergise(node)
    if contention:
        # A protocol-level abort, not a drive fault: more SDO reads would only
        # collide on the same congested bus.
        return contention
    def read(*key):
        try:
            obj = node.sdo
            for k in key:
                obj = obj[k]
            return obj.raw
        except Exception:
            return None
    sw = read('StatusWord')
    if sw is None:
        return 'Calibration fault: {}'.format(exc)
    temp = read('Amplifier', 'Temperature')
    try:
        bus_v = '{:.1f} V'.format(read_bus_v(node))
    except Exception:
        bus_v = None
    min_v = read('Object2384', 'AmplifierMinVoltage')
    max_v = read('Object2384', 'AmplifierMaxVoltage')
    volt = ''
    if bus_v is not None:
        volt = '  BusVoltage={}'.format(bus_v)
        if min_v is not None and max_v is not None:
            volt += '  (min={}, max={})'.format(min_v, max_v)
    msg = ('Drive is still in fault state after CLEAR_FAULT.\n'
           'StatusWord={}  Temp={}{}\n\n').format(
               hex(sw), '{}°C'.format(temp) if temp is not None else 'N/A',
               volt)
    if temp is not None and temp > 60:
        msg += 'Allow the drive to cool and retry calibration.'
    else:
        msg += 'Check the power supply voltage and retry calibration.'
    return msg


# -- 0x3027 encoder compensation, raw SDO ------------------------------------
# sub1 = active, then per bin i: 2+3i A_s (INT16), 3+3i k (UNSIGNED16),
# 4+3i A_c (INT16).  The bin count is whatever the firmware has (v4.4 builds:
# 2), so it is read from sub0.  Newer firmware follows the bins with two
# UNSIGNED16 subs for the encoder's tracking filter (fn Hz, zeta x1000):
# sub0 = 3n+1 means bins only, 3n+3 bins plus filter.  Raw SDO throughout, so
# an EDS with a different sub count cannot get in the way.

def enc_comp_count(node):
    try:
        return int(node.sdo.upload(0x3027, 0)[0])
    except Exception:
        return 0


def enc_comp_bins(node):
    """Number of compensation bins the node implements (0 without 0x3027)."""
    return max(0, (enc_comp_count(node) - 1) // 3)


def enc_filt_subs(node):
    """(fn sub, zeta sub) of the encoder-filter model, or None."""
    hi = enc_comp_count(node)
    if hi >= 4 and (hi - 1) % 3 == 2:
        n = (hi - 1) // 3
        return (2 + 3 * n, 3 + 3 * n)
    return None


def enc_comp_signed(node, sub):
    return 2 <= sub < 2 + 3 * enc_comp_bins(node) and (sub - 2) % 3 != 1


def enc_comp_read(node, sub):
    return int.from_bytes(node.sdo.upload(0x3027, sub)[:2], 'little',
                          signed=enc_comp_signed(node, sub))


def enc_comp_write(node, sub, value):
    node.sdo.download(0x3027, sub, int(value).to_bytes(
        2, 'little', signed=enc_comp_signed(node, sub)))


def enc_comp_save(node, n_bins):
    """Save.Single 0x3027 sub1 .. the last sub of *n_bins*, and the filter
    subs if present."""
    subs = list(range(1, 2 + 3 * n_bins)) + list(enc_filt_subs(node) or ())
    for sub in subs:
        save_single(node, 0x3027, sub)
