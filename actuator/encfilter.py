"""encfilter.py -- the encoder's hardware filter (0x3013,6), chosen from the
product code.

The product code (0x1018,2) names the model, and the model names its
angle sensor.  Firmware writes 0x3013,6 to the sensor's filter-window
register (0x0E) at boot, and only at boot.  So a new value needs Save.Single
and a reboot.  On the K64, a 0 leaves the sensor's own NVM setting alone.

A wider filter window reads quieter at rest but lags the shaft.  The
velocity loop sees that lag at its crossover, and the design caps the
crossover at VEL_MAX_FRACTION of the 1 kHz loop (loops.py, 100 Hz).  The
choice is the quietest setting whose phase lag at that ceiling stays
within LAG_BUDGET_DEG.  The budget is small next to the 30 degrees the
60-degree margin leaves for everything else (the tick delay alone takes
most of it), so the filter never sets the step response.

  MA782 (P4-16)  FW in [7:4], a first-order filter, tau = 2**FW us
                 (datasheet Table 18).  Rest noise by FW from the
                 2026-10-09 bench (notes/p416-commissioning-issues.md).
                 5 degrees at 100 Hz gives FW 7 (128 us, 4.6 deg).
  MA736 (P4-32,  FW in [7:4], H = (1 + 2 tau s) / (1 + tau s)**2 with
  P4-42)         tau = 2**FW us up to FW 12 (datasheet Table 16): a type-2
                 tracking loop, fn = 1 / (2 pi tau), zeta 1.  The factory
                 default is 0xA0 (FW 10, 1024 us); at 0xA0 the model fitted
                 on a P4-42 from coasts to 12k rpm (fn 150 Hz, zeta 0.88)
                 is used instead.  5 degrees at 100 Hz gives FW 9.
  MA735 (P4-42,  the same type-2 filter with an 8-bit FW: 0x33..0xBB in
  from 2026-10)  steps of 0x11 (both nibbles equal), tau = 64 us x
                 2**((FW - 51) / 17), 64 us to 16 ms (datasheet Table 21);
                 default 0x77 (1024 us).  The product code does not tell
                 it from the MA736: the register read back at boot
                 (0x3013,9, stm32 774cdea on) does -- an MA736 never reads
                 equal nibbles.  Its register writes go to the sensor's
                 NVM (1000 cycles), so firmware writes only a changed
                 value.  5 degrees at 100 Hz gives 0x66 (512 us).
  MA702 (P4-37)  the same filter but fixed: no register 0x0E, fCUTOFF 390
                 Hz, tau = 0.38 / fCUTOFF = 974 us.  0x3013,6 is left as
                 it is; the model still goes to 0x3027,8-9.
  others         the encoder is not known: 0x3013,6 is left as it is.
"""

import cmath
import math
import struct

from p4core.can_backend import model_from_product_code

from . import loops

OD_PRODUCT = (0x1018, 2)
OD_FILTER = (0x3013, 6)
#: Read-back at boot: [7:0] is the sensor's register 0x0E as it runs (K64;
#: STM32 from stm32 774cdea, 0 before).
OD_READBACK = (0x3013, 9)
#: The sensor the STM32 identified at boot from its registers' factory
#: defaults (stm32 f226b61): 1 MA736, 2 MA735, 0 not identified (and K64).
#: That firmware also refuses the other chip's 0x3013,6 values.
OD_CHIP = (0x3013, 12)
CHIPS = {1: 'MA736', 2: 'MA735'}

#: Phase lag the filter may add at the velocity crossover ceiling, degrees.
LAG_BUDGET_DEG = 5.0

#: The ceiling the velocity design can reach (loops.design_velocity_pi).
CROSSOVER_HZ = loops.VEL_LOOP_HZ * loops.VEL_MAX_FRACTION

#: The rest-spread limit is this many times the table's range (400 samples)
#: for the chosen setting, and never under p4core's ENCODER_MAX_JITTER.
SPREAD_MARGIN = 2.0

# Known boards (from the hardware owner, 2026-10-14):
#
#   model   0x1018,2   encoder   bench node (can2)
#   P4-16   5707       MA782     3 (EC16)
#   P4-32   5760       MA736     --
#   P4-37   1323       MA702     2 (GL40)
#   P4-42   5755       MA736     4 (BO4523 HFM)
#
#   P4-42   5755       MA735     8 (from 2026-10-21; same product code)
#
# Datasheets: ~/code/stm32/docs/MA702GQ-Z.pdf, MA735GGU.pdf, MA736GGU.pdf,
# MA782GGU-Z.pdf.
MODEL_ENCODER = {'P4-16': 'MA782', 'P4-32': 'MA736', 'P4-37': 'MA702',
                 'P4-42': 'MA736'}


def _ma782_settings():
    # FW: rest range, sd (counts of 0x3012,1, 400 samples) -- bench 2026-10-09
    noise = {9: (2, 0.47), 7: (4, 1.03), 5: (8, 1.77), 3: (14, 3.40),
             1: (25, 4.74)}
    out = {}
    for fw in range(0, 12):
        rng, sd = noise.get(fw, (None, None))
        out[fw << 4] = dict(kind='first-order', tau_us=float(2 ** fw),
                            rest_range_ct=rng, rest_sd_ct=sd,
                            label='FW {} (tau {} us)'.format(fw, 2 ** fw))
    return out


def _type2(tau_us, label):
    return dict(kind='type-2', fn_hz=1e6 / (2 * math.pi * tau_us), zeta=1.0,
                rest_range_ct=None, rest_sd_ct=None, label=label)


def _ma736_settings():
    out = {fw << 4: _type2(float(2 ** fw),
                           'FW {} (tau {} us)'.format(fw, 2 ** fw))
           for fw in range(0, 13)}
    # Fitted on a P4-42 (coasts to 12k rpm); the datasheet gives 155 Hz, 1.
    out[0xA0] = dict(kind='type-2', fn_hz=150.0, zeta=0.88,
                     rest_range_ct=None, rest_sd_ct=None,
                     label='FW 10 (fitted fn 150 Hz, zeta 0.88)')
    return out


#: MA735 register 0x0E values (Table 21): FW 51..187 in steps of 17.
MA735_VALUES = tuple(range(0x33, 0xBC, 0x11))


def ma735_tau_us(value):
    return 64.0 * 2 ** ((value - 0x33) / 17.0)


def is_ma735_value(value):
    """A register 0x0E value only an MA735 reads: both nibbles equal,
    0x33..0xBB.  An MA736's [3:0] are always 0."""
    return value is not None and value in MA735_VALUES


def _ma735_settings():
    return {v: _type2(ma735_tau_us(v), 'FW {} (tau {:.0f} us)'.format(
        v, ma735_tau_us(v))) for v in MA735_VALUES}


ENCODERS = {
    'MA782': _ma782_settings(),
    'MA736': _ma736_settings(),
    'MA735': _ma735_settings(),
}

#: Sensors whose filter is not adjustable: the model, nothing to write.
FIXED = {'MA702': _type2(0.38 / 390.0 * 1e6, 'fixed (fCUTOFF 390 Hz)')}


def response(setting, f_hz):
    """The filter's complex response at *f_hz*."""
    w = 2 * math.pi * f_hz
    s = complex(0.0, w)
    if setting['kind'] == 'first-order':
        return 1.0 / (1.0 + s * setting['tau_us'] * 1e-6)
    wn, z = 2 * math.pi * setting['fn_hz'], setting['zeta']
    return (2 * z * wn * s + wn * wn) / (s * s + 2 * z * wn * s + wn * wn)


def lag_deg(setting, f_hz=CROSSOVER_HZ):
    return -math.degrees(cmath.phase(response(setting, f_hz)))


def _filtering(setting):
    """Larger is quieter: tau, or 1/fn."""
    if setting['kind'] == 'first-order':
        return setting['tau_us']
    return 1e6 / setting['fn_hz']


def encoder_for(product_code):
    """(model, encoder) for a 0x1018,2 product code; encoder None if unknown."""
    model = model_from_product_code(product_code)
    return model, MODEL_ENCODER.get(model)


def choose(encoder, f_hz=CROSSOVER_HZ, budget_deg=LAG_BUDGET_DEG):
    """The setting (0x3013,6 byte) for *encoder*, and why.

    The quietest characterised setting whose lag at *f_hz* is within
    *budget_deg*; when none is, the one with the least lag."""
    table = ENCODERS[encoder]
    order = sorted(table, key=lambda k: _filtering(table[k]), reverse=True)
    for k in order:
        if lag_deg(table[k], f_hz) <= budget_deg:
            reason = ('quietest within {:.0f} deg at {:.0f} Hz'
                      .format(budget_deg, f_hz) if len(table) > 1
                      else 'the only characterised setting')
            return k, reason
    k = min(order, key=lambda k: lag_deg(table[k], f_hz))
    return k, ('the only characterised setting' if len(table) == 1
               else 'least lag; none within {:.0f} deg'.format(budget_deg))


def spread_limit(encoder, value, floor):
    """The rest-spread limit for *value* on *encoder*: SPREAD_MARGIN times
    its bench range, never under *floor*."""
    s = ENCODERS.get(encoder, {}).get(value)
    if not s or s['rest_range_ct'] is None:
        return floor
    return max(floor, int(math.ceil(SPREAD_MARGIN * s['rest_range_ct'])))


def tracking_model(encoder, value, models):
    """0x3027,8-9 (fn Hz, zeta x 1000) for *value*.  *models* is
    encoder.FILTER_MODELS (fitted MA736 settings); other MA736 settings,
    the MA735 and the MA702 come from the datasheet; an MA782 (first-order) has none.  An
    unknown encoder gets the fitted model only."""
    if encoder in FIXED:
        s = FIXED[encoder]
    elif encoder == 'MA736':
        if value in models:
            return models[value]
        s = ENCODERS['MA736'].get(value)
        if s is None:
            return (0, 0)
    elif encoder == 'MA735':
        s = ENCODERS['MA735'].get(value)
        if s is None:
            return (0, 0)
    elif encoder is None:
        return models.get(value, (0, 0))
    else:
        return (0, 0)
    return (int(round(s['fn_hz'])), int(round(s['zeta'] * 1000)))


def read_product_code(node):
    return struct.unpack('<I', bytes(node.sdo.upload(*OD_PRODUCT)[:4]))[0]


def refine_encoder(enc, running):
    """*enc* from the product code, corrected by the register read back at
    boot: a board listed as MA736 whose register reads an MA735 value has
    an MA735 (P4-42s from 2026-10 share the product code)."""
    if enc == 'MA736' and is_ma735_value(running):
        return 'MA735'
    return enc


def read_chip(node):
    """The sensor the firmware identified at boot (0x3013,12), else None
    (not identified, K64, or firmware before stm32 f226b61)."""
    try:
        return CHIPS.get(node.sdo.upload(*OD_CHIP)[0])
    except Exception:
        return None


def encoder_of(node):
    """The node's encoder from its product code, refined by what the firmware
    identified (0x3013,12) or, before that, the boot read-back of register
    0x0E; None if unknown (or no 0x1018,2 to read)."""
    try:
        enc = encoder_for(read_product_code(node))[1]
    except Exception:
        return None
    if enc != 'MA736':
        return enc
    return read_chip(node) or refine_encoder(enc, read_running(node))


def read_filter(node):
    return node.sdo.upload(*OD_FILTER)[0]


def read_running(node):
    """The sensor's register 0x0E as read back at boot, else None (K64
    always; STM32 firmware before stm32 774cdea leaves the word 0).  The
    STM32 reads it after any write, so it is what the sensor took."""
    try:
        raw = bytes(node.sdo.upload(*OD_READBACK)[:4])
    except Exception:
        return None
    return raw[0] if any(raw) else None


def _hex(v):
    return 'n/a' if v is None else '{:#04x}'.format(v)


def plan(node):
    """What the drive has and what it should have.

    Returns dict(product_code, model, encoder, current, running, target,
    reason, lag_deg, label, change).  *target* is None (and *change* False)
    when the encoder is not known, and the current value for a fixed one."""
    try:
        code = read_product_code(node)
    except Exception:
        code = None
    model, enc = encoder_for(code) if code is not None else (None, None)
    try:
        cur = read_filter(node)
    except Exception:
        cur, enc = None, None          # firmware without 0x3013,6
    running = read_running(node)
    if enc == 'MA736':
        enc = read_chip(node) or refine_encoder(enc, running)
    out = dict(product_code=code, model=model, encoder=enc, current=cur,
               running=running, target=None, reason=None,
               lag_deg=None, label=None, change=False)
    if enc is None:
        out['reason'] = '{}: 0x3013,6 left at {}'.format(
            'no 0x3013,6' if cur is None else 'encoder of {} not known'.format(
                model or 'product code {}'.format(code)), _hex(cur))
        return out
    if enc in FIXED:
        # Nothing to write, and register 0x0E is not the filter: no read-back.
        s = FIXED[enc]
        out.update(target=cur, reason='{} has a fixed filter'.format(enc),
                   lag_deg=lag_deg(s), label=s['label'], running=None)
        return out
    target, reason = choose(enc)
    s = ENCODERS[enc][target]
    out.update(target=target, reason=reason, lag_deg=lag_deg(s),
               label=s['label'], change=(cur != target))
    return out


def apply(node, p, save_single):
    """Write *p*'s target to 0x3013,6 and save it (*save_single*(node,
    index, sub)).  It takes effect at the next boot."""
    node.sdo.download(OD_FILTER[0], OD_FILTER[1], struct.pack('<B', p['target']))
    save_single(node, *OD_FILTER)
