"""
device.py — the bus-facing half of the calibrations: P4Drive.

This holds what has to talk to a puck: the raw-unit conversions, the
CiA-402 / mode -1 sequencing, the capture and stream protocols, and the
safety interlocks around driving current into a stationary winding.  p4sysid
builds its identification measurements on it (P4Sysid is a P4Drive).

It does not import a GUI toolkit.  p4gui runs these in a worker thread and
marshals results back; pucktuner runs them on its GUI thread and passes a
*pump* that yields to wx and kicks its watchdog.  Either way the sequencing is
tested against a fake puck instead of a motor.

Raw unit conventions (all verified against the puck4 firmware source, not
guessed — the object dictionary comments in the EDS are in places stale):

  Motor.ud / Motor.uq  (0x3010,4 / 0x3010,5)
      Q15 fraction of U_MAX, where U_MAX = NominalBusVoltage / sqrt(3) is the
      peak line-to-neutral phase voltage available under space-vector
      modulation.  32767 = 1.0.  Firmware rescales the request by
      nominal/live bus so the volts actually applied match the request.

  Motor.id (0x3010,6) / CurrentFeedback (0x6078)
      Per-mille of the rated current, Calibration.i_cont (mA), on rated-unit
      firmware (stm32 ecdc33c on; 0x6075 present), saturating at the working
      peak's per-mille, 1000 x 0x3011,15 / i_cont (3000 on a P4-42); per-mille
      (±1000) of Calibration.i_peak before.  Either way the base is a *peak*
      dq amplitude, not RMS: i_peak of 28284 mA is the peak of 20 A RMS.
      refresh_scaling() puts the base in pm_base_a, the peak in i_max_a.

  Theta_e (0x60EA)
      Q15 fraction of pi electrical.  65536 LSB per electrical revolution.

  Calibration.rt / .lt / .j (0x3011,5 / ,6 / ,7)
      Line-to-line terminal resistance in mohm; line-to-line terminal
      inductance in uH; rotor inertia in g·cm² x 32768 (U32).
      Firmware halves rt/lt to get the per-phase (dq) values, so the Rs and Ls
      measured here — which are dq quantities — are doubled on the way in.

  Motor.Therm (0x3010,3)
      Motor winding temperature from the NTC, in 0.1 °C.  Telemetry only:
      firmware has no motor-overtemperature fault, so nothing on the device
      will stop a thermal test.  This module enforces the limit itself.
"""

import contextlib
import math
import struct
import time

import numpy as np


from p4core.calibrate import (read_kt, has_rated_units, permille_base_ma, working_peak_ma, read_bus_mv,
                               rated_torque_mnm as read_rated_torque_mnm)
from p4core import cia402

from . import dragfit, fits, winding
from .pdo_layout import PdoLayout
from .constants import PWM_PATTERNS_PER_CONTROL_CYCLE, THETA_E_PER_ELEC_REV
#: A winding reading outside this range is no sensor, not a temperature: an
#: absent thermistor reads -273.0 C (P4-16 DevKit), a floating one -38 to
#: -40 C (P4-37 GL40).  Correcting Rs against either inverts or inflates it.
THERM_PLAUSIBLE_C = (-30.0, 200.0)
#: ...and a winding this much colder than the DSP is none either: the
#: controller sits on the motor body.  A floating thermistor drifts up past
#: THERM_PLAUSIBLE_C's floor as the board warms (P4-37 node 2 read -24.8 C
#: with the DSP at 59 C after a 3 A sweep, so the cool-down wait was skipped
#: and the winding sweep tripped 0x4210 at 70 C, 2026-10-26).  A real
#: winding at 20 C under a K64 idling at 52 C is 32 C colder.
THERM_BELOW_DSP_MAX_C = 45.0

#: Purr generator (0x302B).  Sub-index 1 gates a torque overlay that
#: motion_trq_control() sums straight into the torque demand, so it perturbs
#: every measurement that runs in a velocity or torque mode.  Left enabled on a
#: P4-42 (Volume 150, i.e. up to 15 % of a 462 mNm rated torque) it swamps the
#: ~2 mNm of friction torque those measurements are trying to resolve: measured
#: on hardware, the Kt back-EMF fit went from r2 = 0.980 with purring to
#: r2 = 0.99988 with it suspended.  Mode 12 is immune — motion_eval_pva() is
#: empty, so motion_trq_control() never runs — but it is suspended for every
#: measurement regardless, because the cost is one SDO write.
OD_PURR = 'PurrGenerator'
OD_PURR_ENABLE = 'Enable'

#: Nominal bus voltage (0x3001,9), in 0.1 V.  Needed to turn a ud command into
#: volts, since ud is a Q15 fraction of NominalBusVoltage / sqrt(3).
OD_NOMINAL_BUS_V = ('Amp', 'NominalBusVoltage')
#: 1 when the drive's electrical angle is exact for any pole count.
OD_EXACT_ANGLE = (0x3013, 8)

#: Current-sense sample telemetry (0x3016): ,1 the trigger as actually run
#: (post-floor/post-clamp: hal_dbg_ccr4 on the STM32, the PDB count at the
#: ISR on the K64), ,2 the trigger as armed/commanded (pre-floor: wraps if
#: the floor or hal_pwm_set_duty's centred clamp bites, k64/stm32 hal.c), ,3
#: the dead + propagation + settling reservation, ,4 the maximum pulse, ,5
#: the period, ,11 the widest pulse commanded (all timer ticks); ,7 (RW, K64
#: only) moves the trigger by signed ticks, no edge clamp.
OD_ISENSE_DBG = 0x3016

#: CiA-402 mode of operation used by the calibration routines: the drive applies
#: the commanded ud/uq at the commanded Theta_e with the current regulator
#: bypassed.  With uq == 0 the commutation angle comes from Theta_e rather than
#: the encoder, i.e. fully open loop.
MODE_PHASE_VOLTAGE_ANGLE = cia402.MODE_PHASE_VOLTAGE_ANGLE   # -1 (12 on firmware before stm32 cf8d11e)
MODE_IDLE = 0

#: Q15 full scale.  Firmware's FRAC16 saturates at INT16_MAX, so 32767 is 1.0.
FRAC16_ONE = 32767



# ---------------------------------------------------------------------------
# Safety envelope
# ---------------------------------------------------------------------------
# Mode 12 has no protection whatsoever.  Verified against the firmware:
# motion_eval_pva() is empty, so motion_trq_control() never runs and with it
# none of MaxTorque (0x6072), MaxCurrent (0x6073), the torque limits
# (0x60E0/0x60E1) or the i2t integrator; the d/q current regulators are
# bypassed outright; there is no motor-overtemperature fault (0x220A is read by
# nothing); and the timer break input is disabled in hardware, so there is no
# overcurrent trip either.  The only thing firmware enforces is |ud| <= 32767,
# which on a 0.26 ohm winding at U_MAX = 24.2 V is on the order of 90 A.
#
# So the ud ceiling cannot be a fixed Q15 constant — a constant that is safe for
# one motor is a dead short on the next.  The ramp below derives its ceiling
# from the winding resistance it measures as it goes, and this absolute cap is
# only the backstop for a puck whose current sense reads zero.

#: Absolute backstop on ud, as a fraction of Q15 full scale.  Only reached if
#: the measured current stays implausibly low all the way up, which means the
#: current sense is broken rather than that the winding wants more voltage.
UD_ABSOLUTE_CAP = int(0.35 * FRAC16_ONE)
#: The cap once current has been measured.  The ceiling is then the measured
#: winding's own (UD_ESTIMATE_HEADROOM over the ud it needs), so this only has
#: to stay clear of the modulation limit; 0.35 stopped a 10 ohm P4-16 at 0.41
#: of a 0.58 A target.
UD_MEASURED_CAP = int(0.60 * FRAC16_ONE)

#: First blind step of the ramp, in Q15 counts.  Small on purpose: until some
#: current has been measured the winding resistance is unknown, and this is the
#: one step that cannot be sized from a measurement.  ~0.07 V at U_MAX 24 V, so
#: even a 0.05 ohm winding sees only about 1.5 A from it.
UD_PROBE_STEP = 100

#: Bounds on the ramp's proportional steps once resistance is known.
UD_STEP_MIN = 25
UD_STEP_MAX = 1500

#: Abort if measured current exceeds this multiple of the requested target.
#: A winding that is shorted, or a gain calibration that is badly wrong, shows
#: up as current running away from the ramp.
CURRENT_ABORT_FACTOR = 1.6
#: Reads averaged to confirm a ramp reading past the abort threshold.
RAMP_CONFIRM_READS = 4

#: Headroom allowed above the ud that the measured resistance says is needed.
#: The ramp will not command past this multiple of its own estimate, so a
#: current reading that stops responding cannot walk ud up to full scale.
UD_ESTIMATE_HEADROOM = 1.5

#: Largest per-phase resistance the ramp will assume while it is still driving
#: blind.  Once ud passes the voltage that a winding this resistive would need
#: for the target current and no current has appeared, the honest conclusion is
#: that the current sense is reading nothing — not that the winding wants more
#: voltage.  Continuing to ramp on a dead reading is how a motor gets cooked,
#: since nothing downstream limits current in mode -1.  These are sub-ohm
#: motors, so 2 ohm is already generous.
BLIND_MAX_RS_OHM = 2.0
#: The P4-16 is not sub-ohm (~10 ohm/ph), so once Resistance has written
#: Calibration.rt the blind limit is sized from that R instead, plus this
#: much for the bridge's dead-time drop, which eats most of a volt at low
#: current and high PWM (node 3 at 111 kHz saw nothing at 0.71 V).  A dead
#: sense on the configured R then drives barely more than the target.
BLIND_DEADTIME_V = 1.0

#: Current below this fraction of target counts as "no current at all".
NEGLIGIBLE_CURRENT_FRACTION = 0.02

# --- 0x3004, the control-rate capture buffer --------------------------------
#: See measure_ls() for why it has to exist.
OD_CAPTURE_STATE = (0x3004, 1)      # write 1 arms, 0 stops; 2 reads back full
OD_CAPTURE_SOURCE = (0x3004, 6)     # LOG_SRC_*; takes effect at the next arm (packed firmware)
OD_CAPTURE_INTERVAL = (0x3004, 7)   # PWM periods per record, latched at arm
#:   IQ_POS   encoder position (low 16, wraps) with the unfiltered user-frame
#:            iq in mA (high 16) of the same control cycle: torque against
#:            position with no filter between them, for J from a RIPPLE.
LOG_SRC_VBUS_IQ, LOG_SRC_IQ_POS = 4, 5
OD_CAPTURE_SIGNALS_PROBE = (0x3010, 11)


# --- 0x3015, the current-injection primitive --------------------------------
#
# Firmware owns what must be fast or must be safe — a closed-loop current
# regulator, the SVM clamp, the i2t and over-current backstops, and a dead-man
# on how long any of it may run.  It computes no R, no L and no fit; the samples
# go into 0x3004 and the arithmetic happens here.  See app/inject.h.
#
# The protocol is four lines long:
#
#   1. Write the parameters (,2 through ,8).
#   2. Write the mode (,1) LAST.  The mode write is what latches the rest, so
#      the ISR never observes a half-configured excitation.
#   3. ,8 is the dead-man and 0 is *rejected*, not read as "no limit".
#   4. Poll ,9.  TRIPPED means the bound expired — which is distinguishable
#      from a completed excitation, and that is the whole point of it.
#
# Two properties of an active excitation the measurements here rely on:
#
#   * Firmware **forces** theta_e to ,4 rather than deriving it from the
#     encoder, so the measurement frame is a constant this module chose.
#     Nothing has to log theta_e alongside the samples.
#   * Dead-time compensation is gated **off**, so the dead-time drop lands in
#     the intercept of the (V, I) fit and cancels in its slope.  This module
#     must not compensate for it either.
OD_INJECT_MODE = (0x3015, 1)        # write LAST — it latches everything else

#: RIPPLE adds ,2 mA * sin(2 pi ,6 Hz t) to the q current reference of the
#: running current loop — it owns nothing, so the velocity loop and the
#: encoder angle stay live.  Refused unless a current mode is running.
INJ_OFF, INJ_HOLD, INJ_STEP, INJ_SINE, INJ_RIPPLE = 0, 1, 2, 3, 4

# --- The stream (0x3004,9-12) and the drag's RPDO ---------------------------
#
# The drag's data come over PDOs, not the 512-record capture buffer: a TPDO at
# every SYNC carries the stream -- control cycles, the sum of the rotor-frame
# iq over them and the position, all wrapping (app/log.h) -- so the host has
# the exact integral of the torque current between any two frames, on the
# device's clock, for as long as the staircase runs.  And the staircase is
# stepped by an RPDO that writes the target speed and then the mode, which is
# what arms each step, with no SDO round trips in the way.
OD_STREAM_SOURCE = (0x3004, 9)
#: TPDO4 and RPDO4 (0-based 3): borrowed for the measurement, their own
#: configuration put back afterwards.
STREAM_TPDO = 3
#: Mapping words: cycles U16, iq sum I32, position I16 -- 8 bytes, a classic
#: frame -- and the drag's target speed I32, then the mode U8 that arms it.
STREAM_MAPPING = (0x30040A10, 0x30040B20, 0x30040C10)
STREAM_KINDS = 'uss'
#: The SYNC while streaming.  The frames can come at any rate the counters do
#: not wrap within (3.3 s, and 32768 counts); this one resolves a 40 ms ramp.
STREAM_SYNC_HZ = 1000
OD_STREAM_UD_SUM = (0x3004, 13)
#: Control rate in effect, Hz (read-only): pwm / (4 K).
OD_CONTROL_HZ = (0x3001, 14)
#: PWM periods per case slot, K, as set (0 = auto).
OD_PWM_PERIODS = (0x3001, 13)
#: The most PWM periods a case slot the firmware runs (read-only): 4 on both
#: MCUs since stm32 3c6e36a; absent before it (fits.drive_rates falls back).
OD_PWM_PERIODS_MAX = (0x3001, 15)
SW_FAULT = 0x0008

#: How long to wait after each ramp step, as a multiple of the current
#: feedback filter's time constant, bounded by the two limits below.
#:
#: This is a current-limit matter, not a precision one.  The ramp stops when the
#: current it *reads* reaches target, and that reading lags the winding by the
#: filter; waiting only 50 ms against a ~29 ms filter meant a 3.0 A request was
#: still climbing when the ramp stopped and settled at 3.3-3.6 A.  On a bench
#: supply set just above the requested current that is the difference between a
#: measurement and a trip — and the trip sags the bus, which silently defeats
#: firmware's voltage compensation once it hits its 2x cap.
UD_RAMP_SETTLE_TAUS = 4.0
UD_RAMP_SETTLE_MIN_S = 0.05
UD_RAMP_SETTLE_MAX_S = 0.30


#: Longest a settling wait goes without calling *pump*, when there is one.
PUMP_INTERVAL_S = 0.02

class SysidError(RuntimeError):
    """A measurement could not be completed."""


class FitError(SysidError):
    """A fit in fits.py found the data could not support it (its
    ValueError, restated by P4Drive._fit)."""


class SysidAborted(SysidError):
    """A measurement was stopped by its safety interlock or by the user."""


# ---------------------------------------------------------------------------

class _NetworkSync:
    """The SYNC, for a caller that has no owner of it (p4gui passes its
    P4Model, which has the same ``sync_rate`` and ``set_sync_rate``).

    Same rule as P4Model.set_sync_rate: stop before starting, because
    canopen's SyncProducer.start() does not stop the task it replaces, and on
    socketcan that is a kernel timer that goes on sending -- two producers,
    spoiling every drive's clock discipline.
    """

    def __init__(self, network):
        self.network = network
        sync = network.sync
        running = getattr(sync, '_task', None) is not None and sync.period
        self.sync_rate = 1.0 / sync.period if running else 0

    def set_sync_rate(self, rate_hz):
        try:
            self.network.sync.stop()
        except Exception:
            pass
        self.network.sync._task = None
        if rate_hz > 0:
            self.network.sync.start(1.0 / rate_hz)
        self.sync_rate = rate_hz


class _BorrowedPdo:
    """Map one PDO for a measurement, and put its configuration back after.

    *comm* and *mapping* are the PDO's 0x14xx/0x18xx and 0x16xx/0x1Axx
    indices.  The mapping is written the CiA 301 way: the PDO disabled, the
    count to 0, the entries, the count, the transmission type, enabled
    (unless ``enabled`` is false).
    """

    enabled = True

    def __init__(self, dev, comm, mapping, entries, trans_type, default_cob):
        self.dev = dev
        self.comm, self.mapping = comm, mapping
        self.entries, self.trans_type = entries, trans_type
        self.default_cob = default_cob
        self.saved = None
        self.cob = None

    def _w(self, idx, sub, value):
        self.dev.node.sdo[idx][sub].raw = value

    def _r(self, idx, sub):
        return int(self.dev.node.sdo[idx][sub].raw)

    def _configure(self, cob, entries, trans_type, enabled):
        self._w(self.comm, 1, cob | 0x80000000)
        self._w(self.mapping, 0, 0)
        for k, word in enumerate(entries):
            self._w(self.mapping, k + 1, word)
        self._w(self.mapping, 0, len(entries))
        self._w(self.comm, 2, trans_type)
        if enabled:
            self._w(self.comm, 1, cob)

    def __enter__(self):
        cob_id = self._r(self.comm, 1)
        n = self._r(self.mapping, 0)
        # Every slot this will write, the unused ones too: they are left as
        # found, not just ignored by the count.
        self.saved = (cob_id, self._r(self.comm, 2), n,
                      [self._r(self.mapping, k + 1)
                       for k in range(max(n, len(self.entries)))])
        self.cob = (cob_id & 0x7FF) or self.default_cob
        self._configure(self.cob, self.entries, self.trans_type, self.enabled)
        return self

    def __exit__(self, *exc):
        cob_id, trans_type, n, slots = self.saved
        try:
            self._w(self.comm, 1, (cob_id & 0x7FF) | 0x80000000)
            self._w(self.mapping, 0, 0)
            for k, word in enumerate(slots):
                try:
                    self._w(self.mapping, k + 1, word)
                except Exception:
                    # An unused slot reads back as a word the firmware will
                    # not take (0xFFFFFFFF on an RPDO); with the count below
                    # it, what it holds is never used.
                    if k < n:
                        raise
            self._w(self.mapping, 0, n)
            self._w(self.comm, 2, trans_type)
            self._w(self.comm, 1, cob_id)
        except Exception as e:
            self.dev._log('PDO 0x{:04X} not restored: {}'.format(self.comm, e))
        return False


class _Stream(_BorrowedPdo):
    """The stream TPDO, received on a raw subscription (the PdoMap path froze
    in puckutility; see its fast_pdo.py).  ``frames`` gathers
    ``(cycles, iq_sum_ma, position, host_time_s)`` in the order they arrive.
    """

    def __init__(self, dev, source):
        node_id = getattr(dev.node, 'id', 127) or 127
        super().__init__(dev, 0x1800 + STREAM_TPDO, 0x1A00 + STREAM_TPDO,
                         STREAM_MAPPING, 1, 0x180 + 0x100 * STREAM_TPDO + node_id)
        self.source = source
        self.frames = []
        self._sync_was = None
        self.layout = PdoLayout.from_words(STREAM_MAPPING, STREAM_KINDS,
                                           'stream TPDO{}'.format(STREAM_TPDO + 1))

    def _on_frame(self, can_id, data, timestamp):
        v = self.layout.decode(data)
        if v is not None:
            self.frames.append(v + (timestamp,))

    def __enter__(self):
        super().__enter__()
        dev = self.dev
        net = dev.node.network
        net.subscribe(self.cob, self._on_frame)
        nmt = getattr(dev.node, 'nmt', None)
        if nmt is not None:
            nmt.state = 'OPERATIONAL'   # PDOs flow only there
        dev._inject_write(OD_STREAM_SOURCE, self.source)
        sync = dev._sync_owner()
        self._sync_was = sync.sync_rate
        if self._sync_was != STREAM_SYNC_HZ:
            sync.set_sync_rate(STREAM_SYNC_HZ)
        return self

    def __exit__(self, *exc):
        dev = self.dev
        try:
            if self._sync_was != STREAM_SYNC_HZ:
                dev._sync_owner().set_sync_rate(self._sync_was or 0)
        finally:
            try:
                dev._inject_write(OD_STREAM_SOURCE, 0)
            except Exception:
                pass
            try:
                dev.node.network.unsubscribe(self.cob, self._on_frame)
            except Exception:
                pass
            if self.layout.problem():
                dev._log(self.layout.problem())
            super().__exit__(*exc)
        return False

    def mark(self):
        return len(self.frames)

    def mean_iq_a(self, start, stop):
        """Mean iq over frames[start:stop], from the exact integral."""
        f = self.frames[max(start - 1, 0):stop]
        if len(f) < 2:
            raise SysidError(self.layout.problem() or
                             'the stream sent no frames -- is the node '
                             'OPERATIONAL, and is the SYNC reaching it?')
        dc = sum((b[0] - a[0]) & 0xFFFF for a, b in zip(f, f[1:]))
        ds = sum(((b[1] - a[1] + (1 << 31)) % (1 << 32)) - (1 << 31)
                 for a, b in zip(f, f[1:]))
        if dc == 0:
            raise SysidError('the stream\'s cycle counter is not moving')
        return ds / dc / 1000.0


# --- Moved from p4sysid: the drag and what it stands on ---------------------
OD_CAPTURE_RECORDS = (0x3004, 2)
OD_CAPTURE_DECIMATE = (0x3004, 3)
CAPTURE_IDLE, CAPTURE_COLLECT, CAPTURE_FULL = 0, 1, 2
#: What a record holds — LOG_SRC_* in app/log.h.  Both halves of every record
#: are 16 bits, low half first; what they mean is what differs.
#:
#:   DQ       id and iq, per-mille of i_peak — calibrated, Park-transformed
#:            through the *live* commutation angle, and taken after the export
#:            low-pass.
#:   ENC_VEL  encoder position (wraps) and the velocity of the same cycle.
#:   ALBE     alpha and beta in **signed mA**, before the Park transform and
#:            before the export filter.  This is the source identification
#:            uses: nothing in that path has a time constant, so a fit sees the
#:            winding rather than a filter, and because the host knows the angle
#:            it commanded it can redo Park itself — which also makes
#:            |I| = hypot(alpha, beta) available as a free misalignment check.
#:   ADC_RAW  the raw counts behind alpha and beta, for calibration debugging.
LOG_SRC_DQ, LOG_SRC_ENC_VEL, LOG_SRC_ALBE, LOG_SRC_ADC_RAW = 0, 1, 2, 3
#:   IQR_POS  as IQ_POS, but iq Parked with the encoder's angle even while the
#:            drive commutates at a forced one: the torque current of a DRAG,
#:            whose own loop regulates the field's frame instead.
LOG_SRC_IQR_POS = 6
#: Firmware from stm32 0ea2320 on has no packed sources: its logger records
#: OD objects, the *channels*, named by TPDO4's mapping, each in its OD type,
#: and ,5 reads them one channel at a time.  What the packed sources held are
#: objects there (0x3010,7-11, written every control cycle).  Each source maps
#: to the two channels that were its low and high halves, in that order, so
#: one decode serves both kinds of firmware.  The probe is 0x3010,11.
CAPTURE_TPDO = 3
CAPTURE_CHANNELS = {
    LOG_SRC_DQ: (0x30100610, 0x60780010),       # id, iq per-mille (filtered)
    LOG_SRC_ALBE: (0x30100710, 0x30100810),     # alpha, beta mA
    LOG_SRC_IQ_POS: (0x30100B10, 0x30100910),   # position low 16, iq mA
    LOG_SRC_IQR_POS: (0x30100B10, 0x30100A10),  # position low 16, rotor iq mA
}
OD_INJECT_CURRENT = (0x3015, 2)     # HOLD target current, mA
OD_INJECT_THETA = (0x3015, 4)       # forced electrical angle, F16
OD_INJECT_AXIS = (0x3015, 7)        # 0 = d, 1 = q
OD_INJECT_MS = (0x3015, 8)          # dead-man, ms; 0 is rejected
OD_INJECT_STATE = (0x3015, 9)       # INJ_ST_*
OD_INJECT_DRAG_VEL = (0x3015, 15)   # DRAG target speed, cts/s, user frame
OD_INJECT_DRAG_ACC = (0x3015, 16)   # DRAG acceleration, cts/s^2; 0 rejected
OD_INJECT_DRAG_NOW = (0x3015, 17)   # DRAG field speed now, cts/s (read-only)
OD_INJECT_DRAG_DAMP = (0x3015, 18)  # DRAG damping gain, Q8 (see inject.h)
#: DRAG turns a settled d-axis HOLD's field at a ramped speed, with the drive's
#: own current loop in the field's frame; the rotor follows on the magnetic
#: spring.  No velocity loop, so no J or B, holds the speed.  Firmware releases
#: and reports TRIPPED if the rotor falls 60 electrical degrees behind.
INJ_DRAG = 5
(INJ_ST_IDLE, INJ_ST_HOLDING, INJ_ST_STEPPED,
 INJ_ST_INJECTING, INJ_ST_TRIPPED) = 0, 1, 2, 3, 4
#: Largest duration bound firmware accepts — INJ_MAX_MS in app/inject.h.
INJ_MAX_MS = 5000
#: Integrator gain of the HOLD regulator, 2^-INJ_KI_SH per control cycle —
#: INJ_KI_SH in app/inject.h.  Mirrored here only to predict how long a hold
#: takes to settle; see _inject_settle_s.
INJ_KI_SH = 9
#: Axis names, for messages.
INJ_AXIS_D, INJ_AXIS_Q = 0, 1
#: Seconds allowed for a HOLD to settle before anything is measured from it.
#:
#: The loop is a pure integrator with a gain of 2^-9 per control cycle
#: (INJ_KI_SH in app/inject.h), deliberately slow: it only has to place a
#: settled operating point, never to track one.  Working the closed-loop
#: constant out on a P4-42 — U_MAX 24.2 V, Rs 0.30 ohm, i_peak 28.3 A — one F16
#: volt buys 2.9 F16 of current, so the loop constant is 512/2.9 = 179 control
#: cycles, about 36 ms at 5 kHz.  Two hundred milliseconds is five of those.
#:
#: Sized generously rather than tightly on purpose: over-waiting costs a
#: fraction of a second per level, and under-waiting puts the whole
#: measurement on a current that is still moving.  _inject_settle_s scales it
#: by how far the winding is from what the estimate above assumed.
INJ_SETTLE_S = 0.20
#: What a hold's dead-man keeps past its settle, under INJ_MAX_MS: the
#: records a level captures on top (see _inject_settle_s).
INJ_SETTLE_HEADROOM_S = 0.5
#: Extra milliseconds added to every dead-man bound over what the excitation is
#: expected to need.  The bound exists to catch a host that has stopped paying
#: attention, so it wants to be comfortably longer than the plan and nowhere
#: near INJ_MAX_MS.
INJ_DEADMAN_MARGIN_MS = 250
#: Drag (estim_drag).  The higher of its two d currents, as a fraction of
#: i_peak (the other is half of it): the spring's peak torque is 1.5 pp psi I,
#: so the lower one sets how hard the profile may accelerate.  3 A on a P4-42 is 55 mN.m against a ~1e-5 kg.m^2 rotor, and
#: 1.5 I^2 Rs of heat, about 3 W, for the few seconds the staircase takes.
DRAG_CURRENT_FRACTION = 0.1
#: The profile: from DRAG_LOW_RPM to DRAG_TOP_RPM and back in DRAG_STEP_RPM
#: steps, DRAG_REPEATS times, both directions.  Every step up is an
#: acceleration and every step down a deceleration through the same speeds,
#: which is what separates J from friction; the spread of cruise speeds is
#: what separates B from Tc.  The default is one step each way -- a triangle
#: -- because J lives in the ramps: a ramp's inertial impulse J dw has to
#: stand out of the friction impulse of its whole window, and on a P4-42
#: (J 6.7e-6, Tc 3 mN.m) a 200 rpm step's is a tenth of it.  On the bench,
#: 200 rpm steps read J 17-27 % high and moved with the acceleration
#: (7.4/7.3/7.9e-6 at 125/250/500 rad/s^2); 800 rpm ramps, repeated, read
#: 6.64-6.65e-6 at 500 rad/s^2 and 1.4 A, run to run, against 6.68e-6 from
#: id = 0 torque pulses (impulse over the speed gained).
DRAG_LOW_RPM = 200
DRAG_TOP_RPM = 1000
DRAG_STEP_RPM = 800
DRAG_REPEATS = 4
#: Field acceleration, rad/s^2 mechanical: the ceiling.  What the staircase
#: uses comes from the spring it finds.  A ramp of a shifts the load angle by
#: pp a / w_n^2 electrical (J a against a stiffness of J w_n^2, so Kt and J
#: both drop out), and the spring overshoots that by DRAG_OVERSHOOT; so the
#: acceleration is the one that keeps the friction's load angle plus the
#: ramp's, overshoot and all, inside DRAG_LOAD_BUDGET_DEG.  Halved after a
#: pull-out, at most DRAG_RETRIES times.
DRAG_ACCEL_RAD_S2 = 500.0
DRAG_RETRIES = 2
#: Peak load angle, electrical degrees, the acceleration is sized to.  The
#: firmware trips at 60 (INJ_DRAG_TRIP_F16).
DRAG_LOAD_BUDGET_DEG = 40.0
#: The damping ratio 0x3015,18 is set for, from the first arm on.  On its
#: own the spring barely damps at all (the current loop removes the back-EMF
#: current that damps a voltage-mode drag), and on a real motor it is worse
#: than neutral: a P4-42 at 1.4 A, dragged undamped to 200 rpm, rang at its
#: 27 Hz spring frequency with a load angle that grew to +-50 deg and tripped
#: within 0.3 s.  So the drag is never undamped.  The damping acts on the
#: field, not on the torque the fit reads, so it costs the fit nothing, and
#: it is forgiving: on that motor a gain from a quarter of the right one to
#: four times it held the same drag, so the spring frequency it is set from
#: only has to be roughly right.
DRAG_DAMPING_RATIO = 0.5
#: Peak over steady load angle for a ramp's start, at that damping ratio
#: (1 + exp(-pi z / sqrt(1 - z^2)), with a margin).
DRAG_OVERSHOOT = 1.25
#: The spring frequency comes from its stiffness, 1.5 pp psi I cos(load) =
#: Kt_pk pp I cos(load) per mechanical radian, over J -- a prior J, since J is
#: the unknown: the drive's stored Calibration.j, or DRAG_J_PRIOR_KG_M2 when
#: there is none, and for the full-current run the J the half-current run
#: fitted.  It cannot be read off the drag itself: undamped it is unsafe, and
#: damped, the 2x-electrical and 2x-mechanical ripple of a real motor are
#: bigger than what is left of the ring.  The fallback is a heavy rotor, which
#: sizes a gentle acceleration and overdamps a light one; both are safe.
DRAG_J_PRIOR_KG_M2 = 3e-5
#: Stored J outside this range (kg.m^2) is taken for an unset default.
DRAG_J_PLAUSIBLE = (1e-8, 1e-2)
#: The first ramp, 0 to DRAG_LOW_RPM, before the friction is known.
DRAG_START_ACCEL_RAD_S2 = 100.0
#: Cruise recorded at the low speed to find the friction's load angle.
DRAG_LOAD_S = 0.3
#: Cruise captured after each step's ramp, and the settle before the first.
DRAG_CRUISE_S = 0.04
DRAG_SETTLE_S = 0.3
#: Windows that come this close to standstill are left out of the fit.
DRAG_MIN_SPEED_RAD_S = 5.0
#: Cruise after each step, in revolutions: whole ones, so the once- and
#: twice-per-revolution ripple of a real motor (encoder error, load) averages
#: out of the friction instead of landing in it.  Never shorter than
#: DRAG_CRUISE_S.
DRAG_CRUISE_REVS = 2
DRAG_RPDO = 3
DRAG_RPDO_MAPPING = (0x30150F20, 0x30150108)
#: Frames either side of a window, in seconds, that its clock is fitted on.
DRAG_CLOCK_SPAN_S = 0.3
#: TPDO3 (0-based 2), borrowed beside the stream when the firmware has the
#: voltage sums (0x3004,13-14): the field-frame ud and uq the current loop
#: asked for, summed over the same cycles as the stream's, two I32 -- a
#: classic frame -- paired with the stream's frame of the same SYNC by when
#: they arrived.  From them the drag measures the flux, so Kt, with no
#: velocity loop (dragfit.drag_flux_fit).
VOLT_TPDO = 2
VOLT_MAPPING = (0x30040D20, 0x30040E20)
VOLT_KINDS = 'ss'
#: Two frames of one SYNC arrive within this of each other, and a SYNC
#: period from the next pair.
VOLT_PAIR_S = 0.3 / STREAM_SYNC_HZ
#: The spring rings on after each ramp; a cruise's voltage is taken from
#: this long after the ramp ends, over whole revolutions.
DRAG_CRUISE_SETTLE_S = 0.05
#: Commutation latency (0x3013,7 in 0.01 PWM period, 0x3013,5 in us): the
#: request is applied that long after the angle it was turned through.
OD_ENC_LATENCY_CP = (0x3013, 7)
OD_ENC_LATENCY_US = (0x3013, 5)
#: Its pipeline part as firmware computes it from the PWM timing ((5K+1) PWM
#: periods, 0.1 us, read-only); 0x3013,7 is obsolete where this exists.
OD_ENC_LATENCY_PIPE = (0x3013, 10)
#: Kt that sizes the drag's spring when neither the caller nor the drive
#: has one and the flux is to be measured: sizing only.
DRAG_KT_PRIOR_NM_PER_A = 0.02
#: Electrical angle every d-axis measurement is taken at.
#:
#: One angle, unlike the Rs sweep, and it does not have to be more: the Rs
#: sweep averages over thirds of a revolution because the alpha and beta
#: current-sense channels have separate gain calibrations, so how much of their
#: mismatch lands in id depends on where the rotor is — and Rs is read off the
#: *level* of the current, which that mismatch scales directly.  A time
#: constant is read off the *shape* of the rise, and a scale factor on the
#: whole curve leaves the shape alone.
LS_THETA_E = 0
#
# The time base is measured on the device rather than derived from the control
# rate, because the derivation was wrong on the first firmware that shipped the
# buffer and wrong in the one way that does not announce itself.
#
# pwm.c calls logRecord() from two arms of the ISR's pattern switch — case 1,
# left over from the log.c this replaced, and case 3, added with the buffer.
# Both fire once per control cycle, so the buffer takes two records per cycle
# and the second is a copy of the first (*motor.id is only written in case 3).
# A host assuming one record per cycle therefore reads a 25.6 ms capture as
# 51.2 ms and every time constant fitted from it comes out exactly 2x too long.
#
# That is worth measuring around rather than correcting for.  A 2x error here
# is silent: the waveform still looks like a clean exponential, the fit quality
# is untouched, and on this P4-42 the doubled answer (0.258 mH) lands within
# 1 % of the puck's own configured lt — an agreement that reads as confirmation
# and is a coincidence.  Deriving the number a second time from the same source
# would not have caught that; asking the device does.
#
#: Records and length of the timing capture.  The decimation is sized from
#: the control period so the run takes CAPTURE_RATE_SECONDS at any rate: a
#: fixed 2000 was 5 s at 20 kHz control and 50 s at a blank board's 4 kHz.
#: Two seconds is still three orders of magnitude clear of the ~0.7 ms bus
#: round trip that makes timing an undecimated 512-record capture hopeless.
#: CAPTURE_RATE_DECIMATE, when set, overrides the sizing (the tests).
CAPTURE_RATE_RECORDS = 100
CAPTURE_RATE_SECONDS = 2.0
CAPTURE_RATE_DECIMATE = None
#: 0x3004,3 is UNSIGNED16.
CAPTURE_DECIMATE_MAX = 0xFFFF
#: How far the measured interval may sit from a whole number of PWM periods
#: before it is rejected.  The interval is quantised to that grid — the ISR
#: cannot record at anything else — which also discards the SDO polling
#: overhead in the timing loop (about 1 % here, always positive).
CAPTURE_RATE_TOLERANCE = 0.25
class _DragPulledOut(Exception):
    """The drag tripped: the rotor fell 60 electrical degrees behind."""
class _CaptureMap(_BorrowedPdo):
    """TPDO4's mapping naming the logger's channels for one capture.  Left
    disabled, so nothing goes on the bus; the firmware latches the channels at
    arm, and reads them back by them, so it is held from before the arm until
    the capture has been read."""

    enabled = False

    def __init__(self, dev, channels):
        node_id = getattr(dev.node, 'id', 127) or 127
        super().__init__(dev, 0x1800 + CAPTURE_TPDO, 0x1A00 + CAPTURE_TPDO,
                         channels, 1, 0x180 + 0x100 * CAPTURE_TPDO + node_id)
class _VoltStream(_BorrowedPdo):
    """The voltage TPDO beside the stream: ``frames`` gathers
    ``(ud_sum, uq_sum, host_time_s)`` in the order they arrive.  The _Stream
    it is opened with puts the node in OPERATIONAL and runs the SYNC.
    """

    def __init__(self, dev):
        node_id = getattr(dev.node, 'id', 127) or 127
        super().__init__(dev, 0x1800 + VOLT_TPDO, 0x1A00 + VOLT_TPDO,
                         VOLT_MAPPING, 1, 0x180 + 0x100 * VOLT_TPDO + node_id)
        self.frames = []
        self.layout = PdoLayout.from_words(VOLT_MAPPING, VOLT_KINDS,
                                           'voltage TPDO{}'.format(VOLT_TPDO + 1))

    def _on_frame(self, can_id, data, timestamp):
        v = self.layout.decode(data)
        if v is not None:
            self.frames.append(v + (timestamp,))

    def __enter__(self):
        super().__enter__()
        self.dev.node.network.subscribe(self.cob, self._on_frame)
        return self

    def __exit__(self, *exc):
        try:
            self.dev.node.network.unsubscribe(self.cob, self._on_frame)
        except Exception:
            pass
        if self.layout.problem():
            self.dev._log(self.layout.problem())
        super().__exit__(*exc)
        return False
class _DragRpdo(_BorrowedPdo):
    """The RPDO that steps a DRAG: target speed, then the mode that arms it,
    applied on receipt (type 255)."""

    def __init__(self, dev):
        node_id = getattr(dev.node, 'id', 127) or 127
        super().__init__(dev, 0x1400 + DRAG_RPDO, 0x1600 + DRAG_RPDO,
                         DRAG_RPDO_MAPPING, 255, 0x200 + 0x100 * DRAG_RPDO + node_id)

    def go(self, counts_per_s):
        self.dev.node.network.send_message(
            self.cob, struct.pack('<iB', int(counts_per_s), INJ_DRAG))


class P4Drive:
    """One puck, as the calibrations drive it.

    *node* is a canopen RemoteNode.  *log* is called with progress strings.
    *should_abort* is polled between steps; returning True stops the run and
    leaves the drive safe — that is how the GUI's Stop button and window close
    reach into a measurement already in flight.

    *pump*, when given, is called at every abort check and every
    PUMP_INTERVAL_S through a settling wait.  A caller that runs measurements
    on its GUI thread (pucktuner, behind a watchdog) passes one that yields to
    the toolkit and kicks the heartbeat; a caller on a worker thread leaves it
    out and the waits are plain sleeps.
    """

    def __init__(self, node, log=None, should_abort=None, settle_scale=1.0,
                 pump=None, sync=None, detail=None):
        self.node = node
        # Whoever owns the SYNC producer (p4gui: its P4Model); made on first
        # use from node.network otherwise.  See _NetworkSync.
        self._sync = sync
        self._log = log or (lambda msg: None)
        # A sweep's per-point lines: the log unless the caller has a quieter
        # channel (Reporter.detail: the console shows them with --verbose).
        self._detail = detail or self._log
        self._should_abort = should_abort or (lambda: False)
        self._pump = pump
        self.scaling = {}
        # Scales every settling delay.  Tests set it to 0 so the suite does not
        # spend its time waiting on filters that a fake puck does not have; on
        # hardware it stays 1.0 and must, since the delays exist to let the
        # current filter catch up with a step.
        self._settle_scale = settle_scale
        # Previous purr enable state while a measurement holds it off; None
        # when nothing has been suspended, so restoring is idempotent and a
        # measurement that never enabled the drive leaves the setting alone.
        self._purr_restore = None
        #: Set by _cross_check_capture_interval when the device's published
        #: sample interval and a timed capture do not agree.  None until a
        #: capture has been timed, and None afterwards if they agreed.
        self.capture_interval_disagreement = None
        # Timed once, by _capture_record_interval_s, and kept for the session.
        self._record_interval_s = None
        #: The last measure_rs / measure_ls_sine results.  measure_ls and
        #: measure_ls_sine take the dead-time knee from the first.
        self.last_rs = None
        self.last_ldq = None
        #: Winding temperature, deg C, past which _check_temperature stops a
        #: run (None: no limit).  Firmware has no motor over-temperature
        #: fault, so this is the only one there is.
        self.max_temp_c = None

    def _sync_owner(self):
        if self._sync is None:
            self._sync = _NetworkSync(self.node.network)
        return self._sync

    def _stream_sleep(self, seconds):
        """Wait while a stream collects.  Not scaled by settle_scale -- the
        data are the point, not a filter's settling -- and on a network that
        runs on virtual time (fake_puck, the twin) that time is advanced."""
        if seconds <= 0:
            return
        advance = getattr(self.node.network, 'advance', None)
        if advance is not None:
            self._check_abort()
            advance(seconds)
            return
        end = time.monotonic() + seconds
        while True:
            self._check_abort()
            left = end - time.monotonic()
            if left <= 0:
                return
            if self._pump is not None:
                self._pump()
            time.sleep(min(left, PUMP_INTERVAL_S))

    def _sleep(self, seconds):
        delay = seconds * self._settle_scale
        if delay <= 0:
            return
        if self._pump is None:
            time.sleep(delay)
            return
        end = time.monotonic() + delay
        while True:
            self._check_abort()
            left = end - time.monotonic()
            if left <= 0:
                return
            time.sleep(min(left, PUMP_INTERVAL_S))

    @staticmethod
    def _fit(fn, *args, **kwargs):
        """Run a fit from fits.py, restating its ValueError as a FitError.

        The fits raise ValueError for data that cannot support them, which is a
        measurement failure rather than a programming error.  Callers of this
        module should only ever have to catch SysidError.
        """
        try:
            return fn(*args, **kwargs)
        except ValueError as exc:
            raise FitError(str(exc)) from exc

    # ------------------------------------------------------------------
    # Scaling — read once per measurement, then used without touching the bus
    # ------------------------------------------------------------------
    def refresh_scaling(self):
        """Read the constants every conversion below depends on.

        Deliberately not wrapped in try/except.  A missing i_peak or nominal bus
        voltage makes every current and voltage in the run wrong by an unknown
        factor; failing here is far better than reporting a confident Rs derived
        from a fallback constant.
        """
        sdo = self.node.sdo
        s = {
            'i_peak_ma':    float(sdo['Calibration']['i_peak'].raw),
            'i_cal_ma':     float(sdo['Calibration']['i_cal'].raw),
            'i_cont_ma':    float(sdo['Calibration']['i_cont'].raw),
            'poles':        int(sdo['Calibration']['poles'].raw),
            'enc_res':      int(sdo['EncoderConfig']['Resolution'].raw),
            'rated_torque_mnm': float(read_rated_torque_mnm(self.node)),
            'pwm_freq_hz':  float(sdo['Amp']['Frequency'].raw),
            'v_nom':        sdo[OD_NOMINAL_BUS_V[0]][OD_NOMINAL_BUS_V[1]].raw / 10.0,
            # The Kt already on the device, for the caller to compare a fresh
            # measurement against.  Never used as an input to a measurement —
            # a measurement that leans on the value it is meant to replace
            # would agree with a wrong one.
            'kt_device_mnm_per_a': read_kt(self.node),
        }
        if s['i_peak_ma'] <= 0:
            raise SysidError('Calibration.i_peak is 0 — current readings would '
                             'all scale to zero; commission the puck first')
        if s['v_nom'] <= 0:
            raise SysidError('Nominal bus voltage (0x3001,9) is 0 — cannot '
                             'convert a ud command to volts')
        if s['enc_res'] <= 0:
            raise SysidError('Encoder resolution (0x3013,1) is 0')
        # The control rate: the drive's own (0x3001,14) where it reports one;
        # older firmware runs the control cycle at four PWM periods.
        try:
            ctl = float(sdo[OD_CONTROL_HZ[0]][OD_CONTROL_HZ[1]].raw)
        except Exception:
            ctl = 0.0
        s['control_hz'] = ctl if ctl > 0 else (
            s['pwm_freq_hz'] / PWM_PATTERNS_PER_CONTROL_CYCLE)
        try:   # PWM periods per case slot as set (0 = auto, absent = 1)
            s['pwm_periods'] = int(sdo[OD_PWM_PERIODS[0]][OD_PWM_PERIODS[1]].raw) if ctl > 0 else 1
        except Exception:
            s['pwm_periods'] = 1
        s['pwm_periods_max'] = self.pwm_periods_max()
        self._warn_if_winding_unset()

        # U_MAX: peak line-to-neutral phase voltage at Q15 full scale.
        s['u_max_v'] = s['v_nom'] / math.sqrt(3.0)
        # The peak the drive works to (0x3011,15 from stm32 dc5c63b: i_peak
        # under the amplifier and sense limits), the base of its Q15 current
        # references; and what a current per-mille (0x6078, 0x3010,6, 0x6073)
        # is of: the rated current (i_cont 0x3011,8, amplitude; 0x6075 is
        # its rms from stm32 cf8d11e) from stm32 ecdc33c, i_peak before.
        rated_fw = has_rated_units(self.node)
        s['i_max_a'] = ((working_peak_ma(self.node) if rated_fw else 0)
                        or s['i_peak_ma']) / 1000.0
        s['pm_base_a'] = (permille_base_ma(self.node) if rated_fw
                          else s['i_peak_ma']) / 1000.0
        if s['pm_base_a'] <= 0:
            raise SysidError('the rated current (Calibration.i_cont 0x3011,8) is 0 -- '
                             'current per-mille readings would all scale to zero; '
                             'load the template first')
        self.scaling = s
        line = ('scaling: i_peak={:.0f} mA  V_nom={:.1f} V  U_MAX={:.2f} V  '
                'poles={}  enc={} cts/rev  rated={:.0f} mNm  PWM={:.0f} Hz  '
                'control={:.0f} Hz'
                .format(s['i_peak_ma'], s['v_nom'], s['u_max_v'], s['poles'],
                        s['enc_res'], s['rated_torque_mnm'], s['pwm_freq_hz'],
                        s['control_hz']))
        # Every step refreshes it: a line only when something changed.
        if line != getattr(self, '_scaling_line', None):
            self._scaling_line = line
            self._log(line)
        return s

    def pwm_periods_max(self):
        """The most PWM periods a case slot the firmware runs (0x3001,15), or
        None on firmware that does not report it (fits.drive_rates then
        assumes the MCU's old ceiling)."""
        try:
            v = int.from_bytes(self.node.sdo.upload(*OD_PWM_PERIODS_MAX), 'little')
        except Exception:
            return None
        return v or None

    def _warn_if_winding_unset(self):
        """Warn when rt/lt are zero, because that corrupts the current feedback.

        Firmware derives the cutoff of the filters on the exported id/iq from
        the winding time constant:

            tau_winding = lt * 0.01f / rt

        and — unlike poles and the PWM frequency, which are both range-checked
        a few lines earlier — neither rt nor lt is validated.  With both zero
        that is 0.0f/0.0f, i.e. NaN, which propagates through fc into
        biquadInit and leaves the id/iq filters with NaN coefficients.  Every
        measurement here reads those two objects, so if it has happened the
        results are meaningless rather than merely imprecise.

        A warning rather than a refusal: whether it actually bites depends on
        what is in the puck's NVM, which cannot be determined from the bus.
        Writing Rs and Ls back with Save To Device sets rt/lt, so a measurement
        run followed by a save and a reboot is itself the fix.
        """
        try:
            rt = int(self.node.sdo['Calibration']['rt'].raw)
            lt = int(self.node.sdo['Calibration']['lt'].raw)
        except Exception:
            return
        if rt and lt:
            return
        self._log(
            'WARNING: Calibration.rt={} lt={} — firmware divides by rt with no '
            'guard when setting up the id/iq filters, so these being zero can '
            'leave the current feedback filtered by NaN coefficients.  Treat '
            'the current readings with suspicion; saving a measured Rs and Ls '
            'back to the device and rebooting it clears this.'.format(rt, lt))

    # ------------------------------------------------------------------
    # Raw <-> physical conversions
    # ------------------------------------------------------------------
    def ud_to_volts(self, counts):
        """Q15 ud command → peak line-to-neutral volts."""
        return counts / FRAC16_ONE * self.scaling['u_max_v']

    def volts_to_ud(self, volts):
        """Peak line-to-neutral volts → Q15 ud command."""
        return int(round(volts / self.scaling['u_max_v'] * FRAC16_ONE))

    def permille_to_amps(self, permille):
        """Current per-mille (0x6078, 0x3010,6, 0x6073) → amperes (peak dq
        amplitude): of the rated current 0x6075, or i_peak on older firmware."""
        return permille / 1000.0 * self.scaling['pm_base_a']

    @property
    def pole_pairs(self):
        """Pole pairs.  Calibration.poles holds the TOTAL pole count."""
        return max(1, self.scaling['poles'] // 2)

    # ------------------------------------------------------------------
    # Live readings
    # ------------------------------------------------------------------
    def read_id_amps(self):
        return self.permille_to_amps(self.node.sdo['Motor']['id'].raw)

    def read_motor_temp_c(self):
        """Motor winding temperature in °C, or None if unavailable.

        0x3010,3 is the NTC on the winding, published in 0.1 °C.  Returns None
        rather than a fallback: an Rs temperature-corrected against a guessed
        winding temperature is worse than an uncorrected one honestly labelled.
        An implausible reading (THERM_PLAUSIBLE_C) is no sensor and also
        returns None; the raw value is kept in ``therm_implausible_c``.
        """
        try:
            t = self.node.sdo['Motor']['Therm'].raw / 10.0
        except Exception:
            return None
        if not THERM_PLAUSIBLE_C[0] <= t <= THERM_PLAUSIBLE_C[1]:
            self.therm_implausible_c = t
            return None
        dsp = self._read_dsp_temp_c()
        if dsp is not None and t < dsp - THERM_BELOW_DSP_MAX_C:
            self.therm_implausible_c = t
            return None
        return t

    def _read_dsp_temp_c(self):
        """0x3000,2 in degC, or None (absent or implausible)."""
        try:
            t = float(self.node.sdo['Amplifier']['Temperature'].raw)
        except Exception:
            return None
        if not THERM_PLAUSIBLE_C[0] <= t <= THERM_PLAUSIBLE_C[1]:
            return None
        return t

    def read_winding_temp_c(self):
        """(degC, source) for the winding: the thermistor, else the DSP's
        own sensor (0x3000,2), else (None, None).

        The controller is mounted on the motor body, so with no thermistor
        the DSP is a fair proxy for the winding -- in the steady state.  It
        lags the copper by the housing's time constant (a P4-16 rose 19 C by
        copper in 10 s at 2.4 W with the DSP still), so it supervises a slow
        thermal test, not a fast overload.  It reads in whole degrees."""
        t = self.read_motor_temp_c()
        if t is not None:
            return t, 'thermistor'
        t = self._read_dsp_temp_c()
        return (t, 'DSP') if t is not None else (None, None)

    # ------------------------------------------------------------------
    # Drive state
    # ------------------------------------------------------------------
    def _check_abort(self):
        if self._pump is not None:
            self._pump()
        if self._should_abort():
            raise SysidAborted('stopped by request')

    def op_enable(self):
        """Walk the CiA-402 state machine to Operation Enabled over SDO.

        SDO rather than the cyclic RPDO the Test tab uses: these measurements
        write ud/Theta_e/TargetTorque one value at a time and read back between
        writes, so there is no cyclic setpoint stream to keep alive, and RPDO1
        being transmission-type 0 would fight the SDO writes on every SYNC.

        The purr overlay is suspended here rather than in each measurement, so
        that nothing can reach Operation Enabled with it still injecting.
        """
        self._suspend_purr()
        # Fault reset, shutdown, enable operation; a refusal names the fault.
        cia402.enable(self.node, 'Enable')

    def _suspend_purr(self):
        """Hold the purr overlay off, remembering what it was.

        Idempotent: a measurement that enables the drive more than once must
        not overwrite the remembered value with the zero the first call wrote.  A device with no 0x302B (firmware predating the
        purr generator) is left alone rather than treated as an error — every
        measurement here works fine without it.
        """
        if self._purr_restore is not None:
            return
        try:
            was = int(self.node.sdo[OD_PURR][OD_PURR_ENABLE].raw)
        except Exception:
            return
        self._purr_restore = was
        if not was:
            return
        try:
            self.node.sdo[OD_PURR][OD_PURR_ENABLE].raw = 0
        except Exception:
            self._purr_restore = None
            self._log('WARNING: could not suspend the purr overlay (0x302B,1) '
                      '— it injects torque into the demand, so velocity and '
                      'torque-mode results will be noisy')
            return
        self._log('purr overlay suspended for the measurement (was enabled)')

    def _restore_purr(self):
        """Put the purr enable back the way it was found."""
        if self._purr_restore is None:
            return
        was, self._purr_restore = self._purr_restore, None
        if not was:
            return
        try:
            self.node.sdo[OD_PURR][OD_PURR_ENABLE].raw = was
            self._log('purr overlay restored')
        except Exception:
            self._log('WARNING: could not restore the purr overlay (0x302B,1) '
                      '— it is left disabled')

    def set_mode(self, mode):
        cia402.write_mode(self.node, mode)

    def safe_stop(self):
        """Return the drive to a state that cannot drive current.

        Every measurement calls this from a finally block.  Order matters: the
        voltage and torque commands are zeroed *before* the mode changes, so
        there is no window in which a mode transition re-applies a stale
        setpoint.  Each step is independently guarded — a puck that has stopped
        answering must not prevent the remaining steps from being attempted.
        """
        for write in (
            # First of all, and before the voltage commands: an armed injection
            # owns the dq output ahead of every mode, so zeroing ud and idling
            # the mode would leave it driving.  Releasing it is what makes the
            # rest of this sequence mean anything.
            lambda: setattr(self.node.sdo[OD_INJECT_MODE[0]][OD_INJECT_MODE[1]],
                            'raw', INJ_OFF),
            lambda: setattr(self.node.sdo['Motor']['ud'], 'raw', 0),
            lambda: setattr(self.node.sdo['Motor']['uq'], 'raw', 0),
            lambda: setattr(self.node.sdo['TargetTorque'], 'raw', 0),
            lambda: setattr(self.node.sdo['TargetVelocity'], 'raw', 0),
            lambda: setattr(self.node.sdo['SetModeOfOperation'], 'raw', MODE_IDLE),
            lambda: setattr(self.node.sdo['ControlWord'], 'raw', 0x06),
        ):
            try:
                write()
            except Exception:
                pass
        # Last, and outside the loop above: the drive is already safe by this
        # point, and restoring a torque overlay is the one thing here that must
        # not happen while the drive could still act on it.
        self._restore_purr()

    # ------------------------------------------------------------------
    # Open-loop d-axis injection — the primitive Rs, Ls and poles are built on
    # ------------------------------------------------------------------
    def _begin_d_axis(self, theta_e=0x7FFF):
        """Enter mode -1 with zero voltage at a fixed electrical angle.

        The voltages are zeroed before the mode change, not after, so there is
        no instant at which mode -1 is active carrying a stale ud.

        TargetVelocity is set to zero for documentation rather than effect:
        firmware zeroes it on entry to mode -1, and in mode -1 it is
        reinterpreted as a per-cycle electrical-angle increment rather than a
        velocity.  Writes to it are in any case rejected outside the velocity
        modes, so this cannot be relied on to stop an angle drag — the mode
        entry is what does that.
        """
        self.op_enable()
        self.node.sdo['Motor']['ud'].raw = 0
        self.node.sdo['Motor']['uq'].raw = 0        # keeps the angle open-loop
        self.node.sdo['TargetVelocity'].raw = 0
        self.set_mode(MODE_PHASE_VOLTAGE_ANGLE)
        # Through _set_theta_e, not a bare write: Theta_e is INTEGER16 and
        # callers think in electrical-revolution fractions, so two thirds of a
        # revolution (43690) is a perfectly reasonable thing to ask for and an
        # unwrapped write of it is an SDO abort.
        self._set_theta_e(theta_e)
        self._sleep(0.3)                             # let the rotor align

    def _ramp_ud_to_current(self, target_a, ud_start=0):
        """Raise ud until the d-axis current reaches *target_a*.

        Returns the ud command that achieved it.

        This is the only thing protecting the winding — nothing in firmware
        limits current in mode -1 — so the ramp is built around never
        commanding a voltage it has not justified from a measurement:

        * The first step is a small fixed probe, because until current has been
          seen the winding resistance is unknown and no step can be sized.
        * Every later step is extrapolated from the resistance measured so far,
          approaching the target rather than stepping blindly past it.
        * ud is capped at UD_ESTIMATE_HEADROOM times the ud that measured
          resistance says is needed, so a current reading that dies mid-ramp
          cannot let ud walk up to full scale.
        * The current is re-read and checked against the abort threshold before
          each increase, never only after.

        Raises SysidAborted if current runs past CURRENT_ABORT_FACTOR of target.
        """
        ud = ud_start
        abort_a = target_a * CURRENT_ABORT_FACTOR
        ceiling = UD_ABSOLUTE_CAP
        known_rs = self._configured_rs_ohm()
        blind_rs = max(BLIND_MAX_RS_OHM, known_rs or 0.0)
        blind_v = target_a * blind_rs + (
            BLIND_DEADTIME_V if blind_rs > BLIND_MAX_RS_OHM else 0.0)
        blind_limit = min(UD_ABSOLUTE_CAP, self.volts_to_ud(blind_v))
        prev = None                  # (ud, current) of the last measured step
        settle = self._ramp_settle_s()

        while True:
            self._check_abort()
            measured = self.read_id_amps()
            if measured > abort_a:
                # Confirm it: a small target leaves the threshold a few
                # noise widths away, and a rotor snapping onto a new field
                # angle reads a transient.  Node 5's EC16 (0.10 A level,
                # +-20 mA) read 0.18 A once at a ud that holds 0.155 A.
                # ud is held, so real current is still there one settle on.
                self._sleep(settle)
                measured = sum(self.read_id_amps()
                               for _ in range(RAMP_CONFIRM_READS)
                               ) / float(RAMP_CONFIRM_READS)
            if measured > abort_a:
                raise SysidAborted(
                    'd-axis current {:.2f} A ran past the {:.2f} A abort '
                    'threshold at ud={} — stopping'.format(measured, abort_a, ud))
            if measured >= target_a:
                return ud

            if (measured < NEGLIGIBLE_CURRENT_FRACTION * target_a
                    and ud >= blind_limit):
                raise SysidError(
                    'no current detected at ud={} (~{:.2f} V), which would '
                    'already drive {:.2f} A through a {:.1f} ohm winding — the '
                    'current sense is reading nothing, or the motor is not '
                    'connected.  Refusing to raise the voltage any further.'
                    .format(ud, self.ud_to_volts(ud), target_a, blind_rs))

            if ud > 0 and measured > NEGLIGIBLE_CURRENT_FRACTION * target_a:
                # Resistance is now known well enough to size both the next step
                # and the ceiling.  Approach the target in quarters rather than
                # jumping to it, because the exported current is filtered and
                # still lags the last step.
                ud_needed = self._ud_for_current(prev, ud, measured, target_a)
                ceiling = min(UD_MEASURED_CAP,
                              max(ud + UD_STEP_MIN,
                                  int(ud_needed * UD_ESTIMATE_HEADROOM)))
                step = int((ud_needed - ud) / 4)
                if known_rs:
                    # The winding is never steeper than 1/R (the dead-time
                    # knee only flattens it), so a step of the remaining
                    # gap times R cannot pass the target.  ud_needed is
                    # differenced from two reads, and a small target's
                    # sense noise can make it anything: node 5's EC16
                    # (10.3 ohm) jumped UD_STEP_MAX, 0.1 A, past a 0.10 A
                    # level and read 0.17 A over its 0.16 A abort.
                    step = min(step, self.volts_to_ud(
                        (target_a - measured) * known_rs))
            else:
                step = UD_PROBE_STEP

            if ud >= ceiling:
                raise SysidError(
                    'ud reached {} without the d-axis current reaching {:.2f} A '
                    '(last reading {:.2f} A) — check the current-sense '
                    'calibration and the supply current limit'
                    .format(ud, target_a, measured))

            prev = (ud, measured)
            ud = min(ud + max(UD_STEP_MIN, min(UD_STEP_MAX, step)), ceiling)
            self.node.sdo['Motor']['ud'].raw = ud
            self._sleep(settle)

    def _ramp_settle_s(self):
        """Per-step settling time for the ramp, sized from the feedback filter.

        Falls back to the minimum when the filter cannot be predicted (rt/lt
        unset), which is the same case that makes the filter *slowest* — but
        there is nothing to compute a delay from, and the ramp's abort
        threshold still holds regardless.
        """
        tau = self._feedback_filter_tau_s()
        if tau is None:
            return UD_RAMP_SETTLE_MIN_S
        return min(UD_RAMP_SETTLE_MAX_S,
                   max(UD_RAMP_SETTLE_MIN_S, UD_RAMP_SETTLE_TAUS * tau))

    def _ud_for_current(self, prev, ud, measured, target_a):
        """Extrapolate the ud that would produce *target_a*.

        Uses the *incremental* resistance between the last two steps, not the
        chord V/I through the origin.  With an inverter offset V_off the chord
        reads R + V_off/I, which overestimates the resistance badly at low
        current and so overestimates the ud needed; the increment differences
        the offset away and, for a genuinely affine characteristic, lands
        exactly on target.  Measured on a P4-42: the chord form overshot a 3.0 A
        request to 3.5 A, which on a current-limited supply is the difference
        between a measurement and a tripped bench supply.

        Falls back to the chord when there is no usable previous point, or when
        the two points are too close together for their difference to mean
        anything — an overshoot is better than a division by noise.  The
        ramp caps the step it takes from this with the configured R.
        """
        if prev is not None:
            ud_prev, i_prev = prev
            di = measured - i_prev
            if di > NEGLIGIBLE_CURRENT_FRACTION * target_a:
                r_inc = (self.ud_to_volts(ud) - self.ud_to_volts(ud_prev)) / di
                if r_inc > 0:
                    return ud + self.volts_to_ud((target_a - measured) * r_inc)
        return self.volts_to_ud(target_a * self.ud_to_volts(ud) / measured)

    def _inject_write(self, od, value):
        self.node.sdo[od[0]][od[1]].raw = int(value)

    def _inject_read(self, od):
        return int(self.node.sdo[od[0]][od[1]].raw)

    def read_bus_v(self):
        """Measured bus voltage (0x6079, or 0x3000,1 on older firmware), or None."""
        try:
            return read_bus_mv(self.node) / 1000.0
        except Exception:
            return None

    def _configured_rs_ohm(self):
        """Per-phase Rs the puck is already configured with, or None.

        Calibration.rt is the *line-to-line* terminal resistance in mohm and
        firmware halves it to get the dq value, so this halves it too.  Used
        only to size a settling wait and to bound the ramp's steps.
        """
        try:
            rt = int(self.node.sdo['Calibration']['rt'].raw)
        except Exception:
            return None
        return (rt * 0.001) / 2.0 if rt > 0 else None

    # -- capture buffer -------------------------------------------------
    def _require_capture_support(self):
        """Refuse early, and by name, on firmware without the objects this needs.

        0x3004,6 and ,7 rather than merely 0x3004,1: a puck can have the capture
        buffer and still predate the source selector, in which case every record
        it hands back is the filtered dq export and every fit from it is a fit of
        the filter.  0x3015,1 is the excitation itself.
        """
        needs = [(OD_CAPTURE_STATE, 'the control-rate capture buffer'),
                 (OD_CAPTURE_INTERVAL, 'the published capture interval'),
                 (OD_INJECT_MODE, 'the current-injection primitive')]
        if not self._capture_has_channels():
            needs.insert(1, (OD_CAPTURE_SOURCE, 'the capture source selector'))
        for od, what in needs:
            try:
                self.node.sdo[od[0]][od[1]].raw
            except Exception as exc:
                raise SysidError(
                    'this puck has no 0x{:04X},{} ({}), which R and L cannot be '
                    'measured without: SDO polling is far too slow to see a '
                    'current rise, the dq export is filtered too heavily to see '
                    'through, and the excitation itself lives on the device.  '
                    'Update the firmware.  ({})'
                    .format(od[0], od[1], what, exc))

    def _capture_has_channels(self):
        """True when the firmware's logger records channels (OD objects named
        by TPDO4's mapping) rather than packed sources.  Probed once, by raw
        SDO so an EDS that predates the objects does not decide it."""
        has = getattr(self, '_capture_channels_fw', None)
        if has is None:
            try:
                self.node.sdo.upload(*OD_CAPTURE_SIGNALS_PROBE)
                has = True
            except Exception:
                has = False
            self._capture_channels_fw = has
        return has

    def _feedback_filter_fc_hz(self):
        """Cutoff of the firmware low-pass on the exported id, in Hz.

        Nothing measures *through* this any more — captures are taken at
        LOG_SRC_ALBE, which is upstream of it — so what is left of it is one
        job: pacing the ud ramp, which servos on read_id_amps() and therefore
        does see it.  The bypass, the two-candidate scalings and the shadow
        guards that used to surround it went with the measurement they served.

        Mirrors pwm_update_dq_filters() exactly, *including* its unit
        disagreement, because the point is to predict what the device does
        rather than what it was meant to do:

            tau_winding = lt * 0.01 / rt      # treats rt as milliohms
            fc_dq       = 1.7 / (2*pi*5*tau_winding)

        Note the derived form means a *smaller* configured rt makes the filter
        slower, which is why a puck with rt/lt unset is the worst case rather
        than the harmless one.

        Returns None when there is no basis to say — rt/lt unset or unreadable
        — which is also the case in which firmware runs no filter at all.
        """
        try:
            rt = int(self.node.sdo['Calibration']['rt'].raw)
            lt = int(self.node.sdo['Calibration']['lt'].raw)
        except Exception:
            return None
        if rt <= 0 or lt <= 0:
            return None
        fc = 1.7 / (2.0 * math.pi * 5.0 * (lt * 0.01 / rt))   # pwm.c: lt uH, rt mohm
        return fc if fc > 0 else None

    def _feedback_filter_tau_s(self):
        """Settling time constant of the low-pass on the exported id.

        The filter is a *second-order* Butterworth (biquadInit with Q =
        0.7071), not a single pole, and the difference matters: its step
        response decays as exp(-zeta*w0*t), so the time constant is
        1/(2*pi*fc*0.7071) = 0.2251/fc, not the 0.159/fc a single pole would
        give — 42 % longer.  Measured against a P4-42 running the derived
        5.41 Hz cutoff, an equivalent-time capture of the step fitted 40.5 ms
        against the 41.6 ms this predicts, where the single-pole form would
        have said 29.4 ms.

        Returns None when the cutoff is unknown or the filter is bypassed.
        """
        fc = self._feedback_filter_fc_hz()
        if not fc:
            return None
        return 1.0 / (2.0 * math.pi * fc * 0.7071)

    # ------------------------------------------------------------------
    # Winding order and electrical zero
    # ------------------------------------------------------------------
    def measure_winding(self, pole_pairs=None, target_a=None, steps=16,
                        dwell_s=0.1):
        """e_zero, e_polarity and a pole-count check from one field sweep.

        Holds *target_a* on the d axis in mode -1 and steps Theta_e through
        every electrical cycle of one mechanical revolution, forward then
        back (winding.solve says what comes out and why the whole revolution).
        *pole_pairs* defaults to the drive's; a commissioning run passes the
        count measure_pole_pairs found, since the drive's may not be set yet
        -- mode -1 does not use it.  Writes nothing to the drive.
        """
        s = self.refresh_scaling()
        pp = int(pole_pairs or self.pole_pairs)
        res = s['enc_res']
        if target_a is None:
            target_a = min(s['i_cal_ma'], s['i_peak_ma']) / 1000.0
        step = THETA_E_PER_ELEC_REV // steps
        lead = steps // 2          # half a cycle each way first: friction turns
        n = pp * steps             # one mechanical revolution
        samples = []
        theta = 0
        try:
            self._begin_d_axis(theta_e=0)
            self._ramp_ud_to_current(target_a)
            self._sleep(0.5)
            ref = int(self.node.sdo['Encoder']['RawPosition'].raw)
            for direction in (1, -1):
                for k in range(lead + n):
                    self._check_abort()
                    theta += direction * step
                    self._set_theta_e(theta)
                    self._sleep(dwell_s)
                    ref = winding.unwrap(
                        int(self.node.sdo['Encoder']['RawPosition'].raw),
                        ref, res)
                    if k >= lead:
                        samples.append((direction, theta, ref))
                # A drive that faulted mid-sweep (an amplifier over-temperature
                # after back-to-back 3 A sweeps on a P4-37 at 80 kHz) dropped
                # the field: the rest reads as a stall, not as the drive.
                self._raise_if_faulted('the winding sweep')
        finally:
            self.safe_stop()
        r = self._fit(winding.solve, samples, res, pp,
                      exact_angle=self.exact_angle())
        r['pole_pairs'] = pp
        r['current_a'] = target_a
        self._log('winding: e_zero {} counts, e_polarity {:+d}, travel {:.3f} '
                  'rev/rev, spread {:.1f} deg rms ({:.1f} max), friction split '
                  '{:.1f} deg'.format(r['e_zero'], r['e_polarity'],
                                      r['travel_ratio'], r['spread_rms_deg'],
                                      r['spread_max_deg'],
                                      r['friction_split_deg']))
        return r

    def _set_theta_e(self, value):
        """Write Theta_e, wrapped into the signed 16-bit range it expects."""
        self.node.sdo['Theta_e'].raw = ((int(value) + 0x8000) & 0xFFFF) - 0x8000

    def _raise_if_faulted(self, what, cause=None):
        """A SysidError when the drive is in FAULT: *what* stopped because
        the drive did, most likely on its bus."""
        try:
            sw = int(self.node.sdo['StatusWord'].raw)
        except Exception:
            return
        if sw & SW_FAULT:
            bus = self.read_bus_v()
            raise SysidError(
                '{} stopped with the drive in FAULT (StatusWord {:#06x}, bus '
                '{} V): {}{}'.format(
                    what, sw, '?' if bus is None else '{:.1f}'.format(bus),
                    self._fault_cause('under it'),
                    ' ({})'.format(cause) if cause else ''))

    def _fault_cause(self, where):
        """What the drive's last error (0x603F) says stopped it.  The supply
        is blamed only when the code points there (or there is no code):
        node 8's 'brown-out' at 80 kHz (2026-10-21) was 0x8110, a CAN
        overrun from CPU load."""
        try:
            code = int(self.node.sdo['LastError'].raw)
        except Exception:
            code = 0
        if not code:
            return 'no error code; most likely the supply browning out ' + where
        text = cia402.describe_error(code)
        if code in cia402.SUPPLY_ERROR_CODES:
            return 'error {}: the supply, {}'.format(text, where)
        return 'error {}'.format(text)

    def exact_angle(self):
        """True when the firmware's electrical angle is exact for any pole
        count (0x3013,8 = 1); older firmware truncates the cycle to
        res // pole_pairs counts (winding.firmware_e_zero).  Read by a raw
        SDO upload, so an EDS without the subindex does not matter."""
        try:
            return self.node.sdo.upload(*OD_EXACT_ANGLE)[:1] == b'\x01'
        except Exception:
            return False

    def _has_volt_stream(self):
        """True when the firmware streams the voltage request (0x3004,13)."""
        try:
            self._inject_read(OD_STREAM_UD_SUM)
            return True
        except Exception:
            return False

    # ------------------------------------------------------------------
    # Current-sense sample window
    # ------------------------------------------------------------------
    def pwm_timing_ns(self):
        """The drive's 0x3001,2/3/5/6/7 PWM timing in ns, zeros when unreadable."""
        out = {}
        for name, sub in (('dead_time_ns', 2), ('prop_delay_ns', 3),
                          ('settling_ns', 5), ('sampling_ns', 6),
                          ('conversion_ns', 7)):
            try:
                out[name] = float(self.node.sdo[0x3001][sub].raw)
            except Exception:
                out[name] = 0.0
        return out

    def _isense_telemetry(self, reads=1):
        """0x3016 trigger telemetry; the widest pulse and trigger over *reads*.

        *trigger* (,2) is the commanded/armed value, what the K64 trigger
        sweep moves directly; *ccr4* (,1) is the trigger as actually run,
        after the floor and (on an STM32 near max duty) the centred clamp --
        what the STM32 settling sweep has to read back rather than assume.
        """
        dbg = self.node.sdo[OD_ISENSE_DBG]
        tel = {'delay': int(dbg[3].raw), 'max_width': int(dbg[4].raw),
               'period': int(dbg[5].raw),
               'pwm_hz': float(self.node.sdo[0x3001][1].raw)}
        trig, wide, act = [], [], []
        for _ in range(max(1, reads)):
            trig.append(int(dbg[2].raw))
            wide.append(int(dbg[11].raw))
            act.append(int(dbg[1].raw))
        tel.update(trigger=trig[-1], trigger_min=min(trig),
                   trigger_max=max(trig), widest=max(wide), ccr4=act[-1])
        if tel['period'] <= 0 or tel['pwm_hz'] <= 0:
            raise SysidError('the drive publishes no PWM period (0x3016,5)')
        tel['tick_s'] = 1.0 / (tel['period'] * tel['pwm_hz'])
        return tel

    def voltage_budget(self):
        """The drive's own voltage limit and which budget produced it.

        Returns None on firmware without the 0x3016 telemetry.  *one_sided*
        is True for the K64 budget (fits.budget_is_one_sided); a host choosing
        a PWM rate for this drive passes it to fits.choose_rates.
        """
        try:
            tel = self._isense_telemetry()
        except Exception:
            return None
        if tel['max_width'] <= 0:
            return None
        t = self.pwm_timing_ns()
        one_sided = fits.budget_is_one_sided(
            tel['max_width'], tel['period'], tel['pwm_hz'], t['dead_time_ns'],
            t['prop_delay_ns'], t['settling_ns'], t['sampling_ns'])
        # The K64 shifts its pulses past the reported width (k64/hal.c
        # shift_width): the limit it runs is that one.
        gap = (2.0 * t['dead_time_ns'] * 1e-9 / tel['tick_s'] + 2.0
               if one_sided and tel.get('tick_s') else None)
        return {'limit': fits.drive_voltage_limit(tel['max_width'],
                                                  tel['period'], gap),
                'one_sided': one_sided,
                'max_width': tel['max_width'], 'period': tel['period'],
                'pwm_hz': tel['pwm_hz']}

    def _begin_injection(self):
        """Get the bridge switching, then let the injection take it over.

        Mode 12 with both voltages at zero, which is what enables the bridge —
        firmware skips hal_pwm_disable() while an injection is active but
        nothing *enables* it on arming, so arming out of MODE_IDLE would servo a
        current loop into a dead bridge and time out on a dead-man.

        Zero voltage rather than a small one, and no settling wait: mode -1
        here is scaffolding for the excitation, not an excitation itself.  From
        case 4 onwards the injection owns the dq output ahead of the mode, so
        nothing this writes to Motor.ud is ever applied.
        """
        self.op_enable()
        self.node.sdo['Motor']['ud'].raw = 0
        self.node.sdo['Motor']['uq'].raw = 0
        self.set_mode(MODE_PHASE_VOLTAGE_ANGLE)

    def _inject_read_signed(self, od):
        """Read an object firmware declares INTEGER16, whatever the EDS says.

        0x3015,10 is a voltage and goes negative on the q axis.  A stack that
        has the object typed unsigned — an EDS out of step with the firmware,
        which is the normal case for a freshly added subindex — hands back
        65036 for -500 and the fit silently acquires an enormous outlier.
        """
        value = self._inject_read(od) & 0xFFFF
        return value - 0x10000 if value & 0x8000 else value

    def _inject_arm(self, mode, duration_ms, params=None):
        """Stage *params*, then arm *mode*.

        The mode write goes last because the mode write is what latches
        everything else; that ordering is the whole protocol, not a style, and
        firmware relies on it to keep the ISR from ever seeing half an
        excitation.

        The dead-man is written like any other parameter and is checked here
        before the device rejects it, so the message names the bound rather
        than arriving as a bare SDO abort.
        """
        duration_ms = int(round(duration_ms))
        if not 0 < duration_ms <= INJ_MAX_MS:
            raise SysidError(
                'the injection duration bound must be between 1 and {} ms, not '
                '{} — firmware rejects 0 rather than reading it as "no limit"'
                .format(INJ_MAX_MS, duration_ms))

        for od, value in (params or {}).items():
            self._inject_write(od, value)
        self._inject_write(OD_INJECT_MS, duration_ms)
        try:
            self._inject_write(OD_INJECT_MODE, mode)
        except Exception as exc:
            raise SysidError('the device refused injection mode {}: {}'
                             .format(mode, exc))

    def _inject_settle_s(self, rs_ohm=None, floor_s=INJ_SETTLE_S):
        """Seconds to let a HOLD reach its target current.

        The regulator is a pure integrator of gain 2^-INJ_KI_SH per control
        cycle, so its closed-loop constant is 2^INJ_KI_SH / g cycles, where g
        is the F16 current one F16 volt buys:

            g = U_MAX / (Rs * i_peak)

        Five of those.  A *larger* Rs makes g smaller and the loop slower,
        which is why this takes the winding as an argument at all — and why it
        can only ever extend *floor_s*, never shorten it below it: the floor is
        what holds when Rs is not yet known, which is exactly when it is
        measured.  *floor_s* defaults to INJ_SETTLE_S, sized for a winding
        whose own L/R time constant might still be moving current when the
        loop above says it is done; a caller that knows this excitation never
        depends on L settling (the Rs sweep: every bench motor's L/R, 37 us to
        650 us, is two to three orders of magnitude under this loop's own
        constant) may pass a smaller one.
        """
        if not rs_ohm or rs_ohm <= 0:
            return floor_s
        s = self.scaling or self.refresh_scaling()
        i_peak_a = s['i_peak_ma'] / 1000.0
        u_max = s['u_max_v']
        gain = u_max / (rs_ohm * i_peak_a)
        if gain <= 0:
            return floor_s
        tau_s = (2 ** INJ_KI_SH) / gain * self._control_period_s()
        settle = max(floor_s, 5.0 * tau_s)
        # The hold's dead-man is at most INJ_MAX_MS, with the margin and a
        # capture's records on top.  A big winding against a big i_peak (a
        # blank template's 10 A on a 10 ohm P4-16: tau 1 s) runs over it.
        # Three time constants still put the hold within 5 % of its target.
        cap = (INJ_MAX_MS - INJ_DEADMAN_MARGIN_MS) / 1e3 - INJ_SETTLE_HEADROOM_S
        if settle > cap:
            if cap < 3.0 * tau_s:
                raise SysidError(
                    'a HOLD on {:.2f} ohm settles in {:.1f} s (tau {:.2f} s at '
                    'i_peak {:.1f} A), past the {:.0f} ms dead-man: i_peak '
                    '(Calibration.i_peak) is far above this motor\'s'.format(
                        rs_ohm, 5.0 * tau_s, tau_s, i_peak_a, INJ_MAX_MS))
            self._log('hold settle {:.2f} s capped at {:.2f} s ({:.1f} tau): '
                      'i_peak {:.1f} A makes the regulator slow on {:.2f} ohm'
                      .format(settle, cap, cap / tau_s, i_peak_a, rs_ohm))
            settle = cap
        return settle

    def _capture_map(self, source):
        """Name *source*'s channels for the next arm (channel firmware), and
        keep them until _release_capture_map.  A no-op on packed firmware."""
        self._release_capture_map()
        if not self._capture_has_channels():
            return
        if source not in CAPTURE_CHANNELS:
            raise SysidError('capture source {} has no channel equivalent on '
                             'this firmware'.format(source))
        pdo = _CaptureMap(self, CAPTURE_CHANNELS[source])
        pdo.__enter__()
        self._capture_pdo = pdo

    def _release_capture_map(self):
        pdo = getattr(self, '_capture_pdo', None)
        self._capture_pdo = None
        if pdo is not None:
            pdo.__exit__(None, None, None)

    def _pwm_period_s(self):
        """Seconds per PWM period — the grid the ISR can record on."""
        freq = self.scaling.get('pwm_freq_hz') if self.scaling else None
        if not freq:
            freq = float(self.node.sdo['Amp']['Frequency'].raw)
        if freq <= 0:
            raise SysidError('PWM frequency (0x3007,3) is 0 — cannot work out '
                             'the capture buffer sample rate')
        return 1.0 / freq

    def _control_period_s(self):
        """Seconds per control cycle.

        Not the capture buffer's sample interval — see
        _capture_record_interval_s, which measures that instead of deriving it.
        """
        ctl = self.scaling.get('control_hz') if self.scaling else None
        if ctl:
            return 1.0 / ctl
        return PWM_PATTERNS_PER_CONTROL_CYCLE * self._pwm_period_s()

    def _capture_record_interval_s(self):
        """Seconds between capture records.

        Read from 0x3004,7, which is the number of PWM periods the firmware
        latched when the capture was armed, and cross-checked against a
        decimated capture timed on the wall clock.  Both, not either:

        * The published value is the authority.  Deriving the interval from the
          decimation and the control rate was wrong on the first firmware that
          shipped the buffer — pwm.c called logRecord() from two arms of the
          ISR's pattern switch, so it took two records per control cycle and
          every time constant fitted from one came out exactly 2x too long.
          The host cannot see that from the waveform: the trace is still a
          clean exponential, the fit quality is untouched, and on the P4-42 it
          was found on the doubled answer landed within 1 % of the puck's own
          configured lt, which reads as confirmation and is a coincidence.
        * The timed run is kept anyway, because "the device publishes it" is a
          claim about the device, and this whole measurement exists because a
          derivation nobody checked was believed for months.  A disagreement is
          reported and the published value is still used — timing over SDO is
          the less trustworthy of the two, so it gets to raise a hand, not to
          overrule.

        Both come out of one armed capture: arming is what latches 0x3004,7, so
        the timing run publishes the number it is checking.

        Cached — it is a property of the firmware, and two seconds is too long
        to spend more than once per session.
        """
        if getattr(self, '_record_interval_s', None):
            return self._record_interval_s

        pwm = self._pwm_period_s()
        sdo = self.node.sdo
        decimate = CAPTURE_RATE_DECIMATE or min(CAPTURE_DECIMATE_MAX, max(1, int(round(
            CAPTURE_RATE_SECONDS / (CAPTURE_RATE_RECORDS * self._control_period_s())))))
        calls = CAPTURE_RATE_RECORDS * decimate
        expected_s = calls * self._control_period_s()

        # Timed with the bridge switching, as every capture that uses the
        # interval is.  The K64 slows its PWM interrupt 128x in IDLE (low
        # power, k64/hal.c pwm_idle()), so an IDLE timing run measures a rate
        # no measurement ever records at.  Mode 12 at zero volts restores it.
        idle = int(sdo['ReadModeOfOperation'].raw) == MODE_IDLE

        sdo[OD_CAPTURE_STATE[0]][OD_CAPTURE_STATE[1]].raw = CAPTURE_IDLE
        self._capture_map(LOG_SRC_DQ)
        sdo[OD_CAPTURE_RECORDS[0]][OD_CAPTURE_RECORDS[1]].raw = \
            CAPTURE_RATE_RECORDS
        sdo[OD_CAPTURE_DECIMATE[0]][OD_CAPTURE_DECIMATE[1]].raw = decimate
        try:
            if idle:
                self._begin_injection()
            started = time.monotonic()
            sdo[OD_CAPTURE_STATE[0]][OD_CAPTURE_STATE[1]].raw = CAPTURE_COLLECT
            published = int(
                sdo[OD_CAPTURE_INTERVAL[0]][OD_CAPTURE_INTERVAL[1]].raw)
            # Generous against the derived expectation, which is exactly the
            # number under test: anything within a small multiple of it is a
            # rate worth reporting, and beyond that the ISR is not running.
            deadline = started + max(2.0, 5.0 * expected_s)
            state = sdo[OD_CAPTURE_STATE[0]][OD_CAPTURE_STATE[1]]
            while int(state.raw) != CAPTURE_FULL:
                self._check_abort()
                if time.monotonic() > deadline:
                    raise SysidError(
                        'the capture buffer never filled while its sample rate '
                        'was being timed — the drive is not running its '
                        'control loop, which is where records are written')
            elapsed = time.monotonic() - started
        finally:
            if idle:
                self.safe_stop()
            sdo[OD_CAPTURE_STATE[0]][OD_CAPTURE_STATE[1]].raw = CAPTURE_IDLE
            sdo[OD_CAPTURE_DECIMATE[0]][OD_CAPTURE_DECIMATE[1]].raw = 1
            self._release_capture_map()

        if published <= 0:
            raise SysidError(
                'the device published a capture interval of {} PWM periods '
                '(0x3004,7) after a capture was armed.  That object is latched '
                'at arm time, so a zero means this firmware does not maintain '
                'it and no time base can be taken from it.'.format(published))

        # The published figure already includes the decimation the capture was
        # armed with — logArm() latches decimate * the PWM periods in a cycle
        # — so it has to be brought back to a single record before it can be
        # used for the undecimated captures the measurements take, or compared
        # with the timed run below.
        if published % decimate:
            raise SysidError(
                'the device published {} PWM periods per record (0x3004,7) for '
                'a capture decimated by {}, which is not a whole number of '
                'periods per control cycle.  That object is meant to be the '
                'decimation times the periods in a control cycle, so this '
                'firmware is not maintaining it as documented.'
                .format(published, decimate))
        periods_per_record = published // decimate

        self._record_interval_s = periods_per_record * pwm
        self._cross_check_capture_interval(elapsed, calls, pwm,
                                           periods_per_record)

        control = self._control_period_s()
        self._log('capture records every {:.1f} us ({} PWM period{}, {:.2g} '
                  'per {:.0f} us control cycle)'
                  .format(self._record_interval_s * 1e6, periods_per_record,
                          '' if periods_per_record == 1 else 's',
                          control / self._record_interval_s, control * 1e6))
        return self._record_interval_s

    def _cross_check_capture_interval(self, elapsed, calls, pwm, published):
        """Compare the timed interval with the one the device published.

        Timing over SDO carries a positive bias of about 1 % — the polling loop
        — which is far smaller than the half PWM period it would take to reach
        the next point on the grid the ISR can record on, so rounding to that
        grid absorbs it.  Anything left is a real disagreement.
        """
        timed_periods = (elapsed / calls) / pwm
        nearest = max(1, int(round(timed_periods)))
        if abs(timed_periods - nearest) > CAPTURE_RATE_TOLERANCE:
            self._log(
                'capture timing: WARNING — the timed interval works out at '
                '{:.2f} PWM periods per record, which is not a whole number of '
                'them.  {} records decimated by {} took {:.3f} s.  Either '
                'something other than the PWM interrupt is pacing the buffer, '
                'or the bus was slow enough to spoil the timing; the published '
                '{} is being used.'
                .format(timed_periods, CAPTURE_RATE_RECORDS,
                        calls // CAPTURE_RATE_RECORDS, elapsed, published))
            return
        if nearest != published:
            self.capture_interval_disagreement = nearest / float(published)
            self._log(
                'capture timing: WARNING — the device publishes {} PWM periods '
                'per record (0x3004,7) but a timed capture measures {}, a '
                'factor of {:.2f}.  Every time constant fitted from a capture '
                'scales with this, so one of the two is putting a systematic '
                'error straight into L — and 0x3004,7 is derived from the '
                'decimation, so it cannot see how many times the ISR really '
                'calls logRecord per control cycle, which is exactly how this '
                'was wrong by two before.  The published value is being used; '
                'confirm it on a scope before trusting an inductance.'
                .format(published, nearest, nearest / float(published)))

    def rpm_to_counts_per_s(self, rpm):
        """Motor RPM → encoder counts/second."""
        return int(round(rpm / 60.0 * self.scaling['enc_res']))

    def estim_drag(self, kt_nm_per_a=None, ld_h=None, lq_h=None,
                   current_a=None, accel_rad_s2=DRAG_ACCEL_RAD_S2,
                   low_rpm=DRAG_LOW_RPM, top_rpm=DRAG_TOP_RPM,
                   step_rpm=DRAG_STEP_RPM, repeats=DRAG_REPEATS,
                   j_prior_kg_m2=None):
        """J, B and Coulomb friction with no velocity loop in the way.

        The velocity loop's gains are derived from J and B, so a measurement of
        them that needs the loop to hold a speed is circular on a new motor.
        Here the rotor is dragged instead: a d-axis HOLD aligns it, then DRAG
        turns the field through a staircase of speeds in both directions and
        the rotor follows on the magnetic spring.  Only the current loop runs,
        and its gains come from R and L.

        The data come over PDOs: TPDO4 streams, at every SYNC (1 kHz while
        it runs), the control cycles, the sum of the rotor-frame iq over them
        and the position, so the integral of the torque over each step is
        exact however the SYNCs jitter; and RPDO4 steps the drag, target speed
        then mode, with no SDO round trip in the way.  Both PDOs are put back
        as they were found.  Each step's ramp and whole-revolution cruise is a
        window, and dragfit.drag_fit_windows solves the integrated equation of
        motion over each.  Torque is Kt iq corrected for
        reluctance with *ld_h* and *lq_h* (per phase), which matters here
        because the d current is the whole hold current; without them the PM
        term alone is used and J reads high by about (Lq - Ld) id / psi.

        The rotor-frame iq is only as good as the angle it is Parked with, and
        with the whole hold current on d an angle error of e puts I sin(e)
        into it: 50 mA per electrical degree at 3 A.  Two errors are always
        there -- the electrical zero, a whole count of the encoder (0.6 deg on
        a 7-pole-pair, 4096-count motor), and any skew between the encoder
        and current samples, which grows with speed.  The first is a torque
        offset of one sign in both directions, so it cancels in the mean of
        the two Coulomb terms.  The second reads exactly as viscous friction.
        Both scale with I and friction does not, so the staircase runs at
        *current_a* and at half of it, and B is extrapolated to zero current;
        J and Tc are averaged.  The angle errors come back as diagnostics.

        The spring is damped from the first arm, and the damping and the
        acceleration are both sized from its frequency, which needs a J:
        *j_prior_kg_m2*, else the drive's stored one, else DRAG_J_PRIOR_KG_M2.
        It only sizes the excitation -- the result does not depend on it --
        and the full-current run uses the J the half-current run found.

        Returns ``j_kg_m2``, ``b_nm_s_per_rad``, ``a_nm`` (Tc, the mean of the
        two directions), ``zero_error_deg`` and ``skew_us`` (the angle errors
        removed), and the per-current ``runs`` behind them.
        """
        s = self.refresh_scaling()
        try:
            self._inject_read(OD_STREAM_SOURCE)
        except Exception:
            raise SysidError('this firmware has no drag stream (0x3004,9); '
                             'update it to measure by drag')
        volts = self._has_volt_stream()
        kt_drive = s['kt_device_mnm_per_a'] / 1000.0
        # Sizes the spring until the flux is in; the torque's Kt is settled
        # once both runs are.
        kt_size = kt_nm_per_a or kt_drive
        if kt_size <= 0:
            if not volts:
                raise SysidError('the drag fit needs Kt to turn iq into '
                                 'torque, and this firmware cannot measure '
                                 'it (no 0x3004,13)')
            kt_size = DRAG_KT_PRIOR_NM_PER_A
        if ld_h is None or lq_h is None:
            self._log('Drag: no Ld/Lq, so no reluctance correction -- J will '
                      'read high on a salient motor')
        i_hi = current_a or DRAG_CURRENT_FRACTION * s['i_peak_ma'] / 1000.0
        pp = s['poles'] // 2
        kt_pk = kt_size / math.sqrt(2.0)
        rad_per_ct = 2.0 * math.pi / s['enc_res']

        def fit_run(i_hold, windows, kt):
            def torque_of(iq):
                return dragfit.drag_torque(iq, i_hold, kt, pp, ld_h, lq_h)

            integrated = [dragfit.stream_window(w, cyc, rad_per_ct, torque_of)
                          for w, cyc in windows]
            try:
                fit = dragfit.drag_fit_windows(integrated, DRAG_MIN_SPEED_RAD_S)
            except ValueError as exc:
                raise SysidError('the drag captures at {:.2f} A could not be '
                                 'fitted: {}'.format(i_hold, exc))
            if fit['tc_pos_nm'] is None or fit['tc_neg_nm'] is None:
                raise SysidError('the drag fit at {:.2f} A has windows in one '
                                 'direction only'.format(i_hold))
            fit['current_a'] = i_hold
            fit['cycle_s'] = float(np.mean([cyc for _, cyc in windows]))
            return fit
        j_prior = j_prior_kg_m2 or self._stored_inertia_kg_m2()
        if not j_prior:
            j_prior = DRAG_J_PRIOR_KG_M2
            self._log('Drag: no stored J; sizing the drag for a {:.1e} kg.m^2 '
                      'rotor'.format(j_prior))
        accel = accel_rad_s2
        taken = []
        # The low current first: its spring is the weaker, so a pull-out
        # shows there and the halved acceleration serves both runs.
        for i_hold in (0.5 * i_hi, i_hi):
            for attempt in range(DRAG_RETRIES + 1):
                try:
                    try:
                        windows, info, cruises = self._drag_staircase(
                            i_hold, kt_pk, j_prior, accel, low_rpm, top_rpm,
                            step_rpm, repeats)
                    except (SysidError, _DragPulledOut) as exc:
                        # A drive that faulted (a supply browning out) is
                        # not a rotor that pulled out: say which.
                        self._raise_if_faulted('the drag at {:.2f} A'.format(
                            i_hold), exc)
                        raise
                    break
                except _DragPulledOut as exc:
                    if attempt == DRAG_RETRIES:
                        raise SysidError(
                            'the rotor pulled out of step at {:.2f} A and '
                            '{:.0f} rad/s^2 even after {} halvings ({}).  '
                            'Raise the drag current or check for a binding '
                            'load.'.format(i_hold, accel, attempt, exc))
                    self._log('Drag: pulled out at {:.2f} A, {:.0f} rad/s^2 '
                              '({}); halving'.format(i_hold, accel, exc))
                    # Halve what the steps ran at, which the spring may have
                    # already set below the ceiling.
                    ran = getattr(exc, 'accel_rad_s2', None)
                    accel = 0.5 * (ran if ran else accel)
            taken.append((i_hold, windows, info, cruises))
            # Only the next run's sizing needs this J.
            fit = fit_run(i_hold, windows, kt_nm_per_a or kt_size)
            if fit['j_kg_m2'] > 0:
                self._log('Drag: {:.2f} A fits J {:.2e} kg.m^2 (sized for '
                          '{:.2e})'.format(i_hold, fit['j_kg_m2'], j_prior))
                j_prior = fit['j_kg_m2']

        bemf = None
        cruises = [c for t in taken for c in t[3]]
        if cruises:
            delay = self._output_delay_s()
            dl = (lq_h - ld_h) if ld_h is not None and lq_h is not None \
                else 0.0
            try:
                bemf = dragfit.drag_flux_fit(cruises, delay, dl)
            except ValueError as exc:
                self._log('Drag: no flux from the cruises: {}'.format(exc))
            else:
                bemf['delay_s'] = delay
                bemf['kt_nm_per_a'] = dragfit.kt_from_flux(bemf['psi_wb'], pp)
                self._log('Drag: back-EMF over {} cruises: psi = {:.4e} Wb, '
                          'Kt = {:.2f} mNm/A (drive has {:.2f}), Ld = {:.1f} '
                          'uH, residual {:.1f} mV, output delay {:.1f} us'
                          .format(bemf['cruises'], bemf['psi_wb'],
                                  bemf['kt_nm_per_a'] * 1e3, kt_drive * 1e3,
                                  bemf['ld_h'] * 1e6,
                                  bemf['residual_v'] * 1e3, delay * 1e6))
        if kt_nm_per_a:
            kt_source = 'given'
        elif bemf is not None and bemf['kt_nm_per_a'] > 0:
            kt_nm_per_a, kt_source = bemf['kt_nm_per_a'], 'back-EMF'
        elif kt_drive > 0:
            kt_nm_per_a, kt_source = kt_drive, 'drive'
            self._log('Drag: using the drive\'s Kt, {:.2f} mNm/A'
                      .format(kt_drive * 1e3))
        else:
            raise SysidError('the drag fit needs Kt to turn iq into torque, '
                             'and the back-EMF gave none')
        kt_pk = kt_nm_per_a / math.sqrt(2.0)

        runs = []
        for i_hold, windows, info, _ in taken:
            fit = fit_run(i_hold, windows, kt_nm_per_a)
            fit['spring'] = info
            self._log('Drag: {:.2f} A: J = {:.4e} kg.m^2, B = {:.3e} '
                      'N.m.s/rad, Tc = {:+.5f} / {:+.5f} N.m, {} steps'.format(
                          i_hold, fit['j_kg_m2'], fit['b_nm_s_per_rad'],
                          fit['tc_pos_nm'], fit['tc_neg_nm'], fit['segments']))
            runs.append(fit)

        lo, hi = runs
        di = hi['current_a'] - lo['current_a']
        slope = (hi['b_nm_s_per_rad'] - lo['b_nm_s_per_rad']) / di
        b = lo['b_nm_s_per_rad'] - slope * lo['current_a']
        j = float(np.mean([r['j_kg_m2'] for r in runs]))
        a = float(np.mean([0.5 * (r['tc_pos_nm'] + r['tc_neg_nm'])
                           for r in runs]))
        # Offset e = (Tc+ - Tc-)/2 = kt_pk I sin(zero error); skew from the
        # B slope, kt_pk pp tau per amp.
        zero = float(np.mean([math.degrees(math.asin(max(-1.0, min(1.0,
                     0.5 * (r['tc_pos_nm'] - r['tc_neg_nm'])
                     / (kt_pk * r['current_a'])))))
                     for r in runs]))
        skew_us = slope / (kt_pk * pp) * 1e6
        result = {'kt_nm_per_a': kt_nm_per_a, 'kt_source': kt_source,
                  'bemf': bemf, 'j_kg_m2': j,
                  'b_nm_s_per_rad': b, 'a_nm': a,
                  'zero_error_deg': zero, 'skew_us': skew_us,
                  'accel_rad_s2': min(d['accel_rad_s2'] for r in runs
                                      for d in r['spring']),
                  'runs': runs}
        self._log('Drag: J = {:.4e} kg.m^2 ({:.1f} g.cm^2), B = {:.3e} '
                  'N.m.s/rad, Tc = {:.5f} N.m  (removed: electrical zero '
                  '{:+.2f} deg, encoder/current skew {:+.1f} us)'.format(
                      j, fits.kgm2_to_gcm2(j), b, a, zero, skew_us))
        self.last_estim_drag = result
        return result

    def _stored_inertia_kg_m2(self):
        """The drive's Calibration.j in kg.m^2, or None when it has none."""
        try:
            raw = int(self.node.sdo['Calibration']['j'].raw)
        except Exception:
            return None
        # 32768 raw per g.cm^2, 1e-7 kg.m^2 per g.cm^2.
        j = raw / 32768.0 * 1e-7
        # A drive that was never identified holds whatever its defaults
        # were, and a J sized off by orders of magnitude sizes the damping
        # to nothing.  From a watch motor to a large servo is plenty.
        if not DRAG_J_PLAUSIBLE[0] <= j <= DRAG_J_PLAUSIBLE[1]:
            if raw > 0:
                self._log('Drag: the stored J, {:.1e} kg.m^2, is not a '
                          'rotor\'s; ignoring it'.format(j))
            return None
        return j

    def _drag_staircase(self, i_hold_a, kt_pk, j_prior, accel_rad_s2,
                        low_rpm, top_rpm, step_rpm, repeats=1):
        """Run the staircase once, streaming.

        Returns ``(windows, info, cruises)``: windows are ``(frames,
        cycle_s)``, one per step: the stream's raw ``(cycles, iq_sum_ma,
        position)`` frames from the one before the step's command to the end
        of its cruise, and the control cycle there, measured against the
        host's clock (_stream_cycle_s); info has, per direction, the spring's
        ``spring_hz`` (from *j_prior*), the friction's ``load_deg`` and the
        ``accel_rad_s2`` and ``damp_q8`` the steps ran at; cruises are
        dragfit.drag_cruise's means over each step's cruise, with its
        ``current_a`` and ``sign``, when the firmware streams the voltage
        (empty when it does not).
        """
        s = self.scaling
        cpr = s['enc_res']
        pp = s['poles'] // 2
        rad_per_ct = 2.0 * math.pi / cpr
        dt_cycle = self._capture_record_interval_s()
        levels = list(range(int(low_rpm), int(top_rpm) + 1, int(step_rpm)))
        if len(levels) < 2:
            raise SysidError('the drag staircase needs two speeds or more')
        path = (levels[1:] + levels[-2::-1]) * repeats
        spans = []
        info = []

        # Stiffness per mechanical radian at zero load angle, and the
        # damping for the spring it makes with the prior J.  The derivative
        # the firmware damps with has its corner at rate/16.
        k0 = kt_pk * pp * i_hold_a
        corner = 1.0 / (16.0 * dt_cycle)

        def damp_q8(wn):
            return min(0xFFFF, int(round(
                256.0 * 2.0 * DRAG_DAMPING_RATIO * corner / wn)))

        def acc_cts(accel):
            return max(1, int(round(accel / rad_per_ct)))

        damp = damp_q8(math.sqrt(k0 / j_prior))

        def ramp_s(rpm_from, rpm_to, accel):
            return abs(rpm_to - rpm_from) * 2.0 * math.pi / 60.0 / accel

        def check(what):
            state = self._inject_read(OD_INJECT_STATE)
            if state == INJ_ST_TRIPPED:
                raise _DragPulledOut(what)
            if state != INJ_ST_INJECTING:
                raise SysidError('the drag reports state {} {}'.format(
                    state, what))

        with (_VoltStream(self) if self._has_volt_stream()
              else contextlib.nullcontext()) as vstream, \
                _Stream(self, LOG_SRC_IQR_POS) as stream, \
                _DragRpdo(self) as rpdo:
            try:
                self._begin_injection()
                self._inject_arm(INJ_HOLD, INJ_MAX_MS, {
                    OD_INJECT_CURRENT: int(round(i_hold_a * 1000.0)),
                    OD_INJECT_THETA: LS_THETA_E, OD_INJECT_AXIS: INJ_AXIS_D})
                self._stream_sleep(self._inject_settle_s() + DRAG_SETTLE_S)
                for sign in (1, -1):
                    self._check_abort()
                    # The ramps to and from rest by SDO, with no data taken.
                    start = min(accel_rad_s2, DRAG_START_ACCEL_RAD_S2)
                    self._inject_arm(INJ_DRAG, INJ_MAX_MS, {
                        OD_INJECT_DRAG_VEL: self.rpm_to_counts_per_s(
                            sign * levels[0]),
                        OD_INJECT_DRAG_ACC: acc_cts(start),
                        OD_INJECT_DRAG_DAMP: damp})
                    self._stream_sleep(ramp_s(0, levels[0], start)
                                       + DRAG_SETTLE_S)
                    try:
                        check('reaching {} rpm'.format(sign * levels[0]))
                    except _DragPulledOut as exc:
                        exc.accel_rad_s2 = start
                        raise

                    # The friction's load angle, which takes that much out of
                    # the budget and softens the spring by its cosine.
                    m = stream.mark()
                    self._stream_sleep(DRAG_LOAD_S)
                    iq = stream.mean_iq_a(m, stream.mark())
                    load = math.degrees(math.asin(
                        min(1.0, abs(iq) / i_hold_a)))
                    wn = math.sqrt(k0 * math.cos(math.radians(load))
                                   / j_prior)
                    room = math.radians(max(DRAG_LOAD_BUDGET_DEG - load, 1.0))
                    accel = min(accel_rad_s2,
                                room * wn * wn / (DRAG_OVERSHOOT * pp))
                    damp = damp_q8(wn)
                    info.append({'spring_hz': wn / (2.0 * math.pi),
                                 'load_deg': load, 'accel_rad_s2': accel,
                                 'damp_q8': damp})
                    self._log('Drag: {:+.0f} rpm: friction takes {:.1f} deg; '
                              'spring {:.1f} Hz for J {:.2e}; damping {}/256, '
                              'stepping at {:.0f} rad/s^2'.format(
                                  sign * levels[0], load,
                                  wn / (2.0 * math.pi), j_prior, damp, accel))
                    # Staged now, latched by each step's arm.
                    self._inject_write(OD_INJECT_DRAG_ACC, acc_cts(accel))
                    self._inject_write(OD_INJECT_DRAG_DAMP, damp)

                    now = levels[0]
                    for rpm in path:
                        self._check_abort()
                        cruise = max(DRAG_CRUISE_S,
                                     DRAG_CRUISE_REVS * 60.0 / rpm)
                        m = stream.mark()
                        rpdo.go(self.rpm_to_counts_per_s(sign * rpm))
                        ramp = ramp_s(now, rpm, accel)
                        self._stream_sleep(ramp + cruise)
                        try:
                            check('stepping {} -> {} rpm'.format(sign * now,
                                                                 sign * rpm))
                        except _DragPulledOut as exc:
                            exc.accel_rad_s2 = accel
                            raise
                        # The RPDO has no answer; the field speed is the
                        # proof it was taken.
                        field = self._inject_read_signed32(OD_INJECT_DRAG_NOW)
                        want = self.rpm_to_counts_per_s(sign * rpm)
                        if abs(field - want) > 0.02 * abs(want) + 1:
                            raise SysidError(
                                'the drag RPDO for {} rpm was not taken: the '
                                'field turns at {} counts/s, not {}'.format(
                                    sign * rpm, field, want))
                        spans.append((max(m - 1, 0), stream.mark(), sign,
                                      ramp))
                        now = rpm
                    self._inject_arm(INJ_DRAG, INJ_MAX_MS, {
                        OD_INJECT_DRAG_VEL: 0,
                        OD_INJECT_DRAG_ACC: acc_cts(start)})
                    self._stream_sleep(ramp_s(now, 0, start) + DRAG_SETTLE_S)
                    check('returning to rest')
            finally:
                self.safe_stop()
            frames = list(stream.frames)
            vframes = list(vstream.frames) if vstream is not None else None
        # The device's clock is disciplined to the SYNC, which a USB adapter
        # may pace a percent off, and trimmed in steps while it chases it:
        # each window is timed by the clock around it, not the run's mean.
        reach = int(DRAG_CLOCK_SPAN_S * STREAM_SYNC_HZ)
        windows = []
        ppm = []
        for a, b, _, _ in spans:
            cyc = self._stream_cycle_s(frames[max(a - reach, 0):b + reach],
                                       dt_cycle)
            ppm.append((cyc / dt_cycle - 1.0) * 1e6)
            windows.append(([f[:3] for f in frames[a:b]], cyc))
        self._log('Drag: the control cycle ran {:+.0f} to {:+.0f} ppm off '
                  'nominal ({} frames)'.format(min(ppm), max(ppm),
                                               len(frames)))
        cruises = []
        if vframes is not None:
            pair = dragfit.pair_frames(
                [np.nan if f[3] is None else f[3] for f in frames],
                [np.nan if v[2] is None else v[2] for v in vframes],
                VOLT_PAIR_S)
            for (a, b, sign, ramp), (w, cyc) in zip(spans, windows):
                volts = [vframes[j][:2] if j >= 0 else None
                         for j in pair[a:b]]
                cr = dragfit.drag_cruise(
                    w, volts, (ramp + DRAG_CRUISE_SETTLE_S) / cyc, cyc,
                    rad_per_ct, cpr, pp, s['u_max_v'])
                if cr is not None:
                    cr.update(current_a=i_hold_a, sign=sign)
                    cruises.append(cr)
            self._log('Drag: {} of {} voltage frames paired; {} cruises of '
                      '{} usable for the flux'.format(
                          int(np.sum(pair >= 0)), len(vframes), len(cruises),
                          len(spans)))
        return windows, info, cruises

    def _output_delay_s(self):
        """The commutation latency: from the samples a request is computed
        from to the middle of its application.  The pipeline part the drive
        computes (0x3013,10, 0.1 us) plus 0x3013,5 (us); firmware older than
        0x3013,10 has the pipeline part in 0.01 PWM period at 0x3013,7.  0 when
        the drive has none."""
        try:
            us = self._inject_read_signed(OD_ENC_LATENCY_US)
        except Exception:
            return 0.0
        try:
            pipe = self._inject_read_signed(OD_ENC_LATENCY_PIPE)
        except Exception:
            pipe = 0
        if pipe > 0:
            return pipe * 1e-7 + us * 1e-6
        try:
            cp = self._inject_read_signed(OD_ENC_LATENCY_CP)
        except Exception:
            return 0.0
        return cp / 100.0 / self.scaling['pwm_freq_hz'] + us * 1e-6

    def _inject_read_signed32(self, od):
        value = self._inject_read(od) & 0xFFFFFFFF
        return value - (1 << 32) if value & 0x80000000 else value

    def _stream_cycle_s(self, frames, nominal_s):
        """The control cycle, from the stream's cycle count against the
        host's receive times.

        The device's clock is the HSI, trimmed by the clock discipline to the
        SYNC -- which is only as good as whatever paces it: a USB adapter ran
        a nominal 1 kHz at 1.0096 ms on the bench, and the discipline chased
        it by thousands of ppm in a second and a half.  Every time in the fit
        is counted in its cycles, and J scales with the cycle squared.  A fit
        of the host's receive times to the unwrapped count over *frames* is
        good to the timestamp jitter over their span (~30 us a frame, ~15 ppm
        over 0.6 s), so it is used whenever it lands within 3 % of nominal.
        """
        if len(frames) < 100:
            return nominal_s
        f = np.asarray([(c, ts if ts is not None else np.nan)
                        for c, _, _, ts in frames], float)
        cyc = np.concatenate(([0], np.cumsum(
            np.diff(f[:, 0]).astype(np.int64) & 0xFFFF)))
        ts = f[:, 1]
        ok = np.isfinite(ts)
        # Only while the count runs: it stands still between injections.
        moving = np.concatenate(([False], np.diff(cyc) > 0)) & ok
        if moving.sum() < 100 or np.ptp(cyc[moving]) == 0:
            return nominal_s
        slope = float(np.polyfit(cyc[moving].astype(float), ts[moving], 1)[0])
        ppm = (slope / nominal_s - 1.0) * 1e6
        if abs(ppm) > 3e4:
            self._log('Drag: the stream\'s clock reads {:+.0f} ppm off '
                      'nominal; not believed, using nominal'.format(ppm))
            return nominal_s
        return slope
