"""
Tests for the calibration current ramp (p4core.calibrate).

The ramp is what puts current through the winding for the gain and encoder
calibrations, and its old form could decline to do so without saying anything —
leaving calibrate_enczero to save an e_zero that was just wherever the shaft was
resting and an e_polarity of +1 regardless of the truth.  These cover the shape
of the loop rather than the numbers it produces.
"""

import os
import sys
import unittest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))

from p4core import calibrate                                 # noqa: E402
from p4core.reporter import Reporter                         # noqa: E402


class Entry:
    """One SDO entry, backed by a callable or a plain value."""

    def __init__(self, owner, key, value):
        self._owner, self._key, self._value = owner, key, value

    @property
    def raw(self):
        self._owner.reads.append(self._key)
        if callable(self._value):
            return self._value()
        return self._value

    @raw.setter
    def raw(self, value):
        self._owner.writes.append((self._key, value))
        self._owner.on_write(self._key, value)


class Record:
    """An OD entry addressed either as ``sdo[name]`` or ``sdo[name][sub]``.

    Both forms have to work on the same object: the calibration routines use
    ``sdo['SetModeOfOperation'].raw`` and ``sdo['Motor']['ud'].raw`` in the same
    breath.  A fake that only implements __getitem__ swallows every top-level
    write into an instance attribute and reports nothing.
    """

    def __init__(self, owner, name):
        object.__setattr__(self, '_owner', owner)
        object.__setattr__(self, '_name', name)

    def __getitem__(self, sub):
        return Entry(self._owner, (self._name, sub),
                     self._owner.value((self._name, sub)))

    def __getattr__(self, attr):
        if attr != 'raw':
            raise AttributeError(attr)
        return self[None].raw

    def __setattr__(self, attr, value):
        if attr != 'raw':
            raise AttributeError(attr)
        self[None].raw = value


class FakeNode:
    """Just enough of a canopen node for the ramp.

    ``Motor/id`` follows ``Motor/ud`` through a first-order lag with no memory
    of time — one step per read — which is all the ramp's exit condition needs
    to be exercised.  ``id_on_entry`` seeds it, and that is the whole point:
    the bug being tested only appears when current is already flowing.
    """

    I_PEAK_MA = 28284.0
    I_CAL_MA = 3000.0

    def __init__(self, id_on_entry=0.0, ohms_per_ud=0.0022):
        self.ud = 0
        self.id_permille = id_on_entry
        self.ohms_per_ud = ohms_per_ud
        self.reads, self.writes = [], []
        self.sdo = self

    def value(self, key):
        table = {
            ('Calibration', 'i_cal'): self.I_CAL_MA,
            ('Calibration', 'i_peak'): self.I_PEAK_MA,
            ('Motor', 'id'): lambda: self.id_permille,
            ('Motor', 'ud'): lambda: self.ud,
        }
        return table.get(key, 0)

    def on_write(self, key, value):
        if key == ('Motor', 'ud'):
            self.ud = value
            # Settled current for this ud, in per-mille of i_peak.
            amps = value * self.ohms_per_ud
            self.id_permille = amps / (self.I_PEAK_MA / 1000.0) * 1000.0

    def __getitem__(self, name):
        return Record(self, name)


class RatedFakeNode(FakeNode):
    """Firmware from stm32 ecdc33c: Motor/id is per-mille of the rated current
    i_cont and pins at the working peak's per-mille (0x3011,15), not 1000.
    Stage 3 (cf8d11e) publishes 0x6075 r.m.s.; the base is still i_cont."""

    RATED_MA = 9428

    def upload(self, index, sub):
        rows = {(0x6075, 0): int(round(self.RATED_MA / 2 ** 0.5)),
                (0x3011, 8): self.RATED_MA, (0x3011, 15): int(self.I_PEAK_MA)}
        if (index, sub) not in rows:
            raise KeyError((index, sub))
        return rows[(index, sub)].to_bytes(4, 'little')

    def on_write(self, key, value):
        if key == ('Motor', 'ud'):
            self.ud = value
            amps = value * self.ohms_per_ud
            self.id_permille = min(amps / (self.RATED_MA / 1000.0) * 1000.0,
                                   1000.0 * self.I_PEAK_MA / self.RATED_MA)


class NoWait(Reporter):
    def sleep(self, seconds):
        self.check_abort()


def ramp(node):
    return calibrate.ramp_to_calibration_current(node, NoWait())


def gain(node):
    try:
        calibrate.calibrate_gain(node, NoWait())
    except calibrate.CalibrationError:
        pass        # no rotating current in this fake; the exit path is the point


class TestCalibrationRamp(unittest.TestCase):

    def test_applies_a_step_before_it_measures(self):
        """Current already flowing must not let the ramp decline to run.

        This is the failure: run straight after the gain calibration, which
        used to leave ud energised, Motor/id already reads past the target, the
        old loop's exit condition was true on entry, and ud never moved.  The
        calibration then reported success on a rotor that never turned.
        """
        node = FakeNode(id_on_entry=109.0)      # ~3.1 A, past i_cal
        ud = ramp(node)
        self.assertGreater(ud, 0, 'ramp declined to apply any voltage')
        self.assertEqual(node.ud, ud)

    def test_stops_once_the_live_reading_reaches_target(self):
        node = FakeNode()
        ud = ramp(node)
        reached_a = node.id_permille / 1000.0 * node.I_PEAK_MA
        self.assertGreaterEqual(reached_a, node.I_CAL_MA)
        # ...and stopped there, rather than one step past it.
        one_less = (ud - calibrate.RAMP_UD_STEP) * node.ohms_per_ud * 1000
        self.assertLess(one_less, node.I_CAL_MA)

    def test_rereads_the_current_every_pass(self):
        """The exit test must be a live reading, not one taken before the loop."""
        node = FakeNode()
        ramp(node)
        id_reads = [k for k in node.reads if k == ('Motor', 'id')]
        ud_writes = [k for k, _v in node.writes if k == ('Motor', 'ud')]
        self.assertEqual(len(id_reads), len(ud_writes),
                         'one current reading per applied step')

    def test_never_exceeds_the_ud_ceiling(self):
        """A winding that draws nothing must not walk ud up without limit."""
        node = FakeNode(ohms_per_ud=0.0)        # no current, ever
        ud = ramp(node)
        self.assertLessEqual(ud, calibrate.RAMP_UD_MAX)

    def test_stops_on_a_saturated_reading(self):
        """1000 per-mille is full scale; past it the reading means nothing."""
        node = FakeNode(ohms_per_ud=1.0)        # one step saturates
        node.I_CAL_MA = node.I_PEAK_MA          # target it cannot reach cleanly
        ud = ramp(node)
        self.assertEqual(ud, calibrate.RAMP_UD_STEP)

    def test_caps_the_target_at_i_peak(self):
        node = FakeNode()
        node.I_CAL_MA = node.I_PEAK_MA * 10     # nonsense i_cal
        ud = ramp(node)
        reached = node.id_permille / 1000.0 * node.I_PEAK_MA
        self.assertLessEqual(reached, node.I_PEAK_MA * 1.05)
        self.assertLess(ud, calibrate.RAMP_UD_MAX)


class TestRatedUnitFirmware(unittest.TestCase):
    """Per-mille of 0x6075: read as of i_peak, the ramp stopped at a third of i_cal."""

    def test_reaches_i_cal(self):
        node = RatedFakeNode()
        ramp(node)
        reached = node.id_permille / 1000.0 * node.RATED_MA
        self.assertGreaterEqual(reached, node.I_CAL_MA)
        self.assertLess(reached, node.I_CAL_MA * 1.1)

    def test_1000_permille_is_not_saturation(self):
        node = RatedFakeNode()
        node.I_CAL_MA = 1.5 * node.RATED_MA     # 1500 per-mille, under the 3000 peak
        ramp(node)
        self.assertGreater(node.id_permille, 1000)

    def test_stops_at_the_peak_permille(self):
        node = RatedFakeNode(ohms_per_ud=1.0)   # one step pins the reading
        node.I_CAL_MA = node.I_PEAK_MA * 10
        self.assertEqual(ramp(node), calibrate.RAMP_UD_STEP)

    def test_id_scale(self):
        self.assertEqual(calibrate.id_scale(RatedFakeNode()), (9428, 3000))
        self.assertEqual(calibrate.id_scale(FakeNode()), (28284.0, 1000))


class TestGainCalibrationDeEnergises(unittest.TestCase):
    """calibrate_gain must not hand the next step a live winding.

    Leaving the mode of operation does not clear ud, so a routine that only
    writes SetModeOfOperation = 0 leaves the phase voltage reference standing
    and the next selection of mode 12 re-applies it immediately.
    """

    def test_ud_is_zero_on_exit(self):
        node = FakeNode()
        gain(node)
        self.assertEqual(node.ud, 0)

    def test_ud_is_zeroed_before_the_mode_changes(self):
        node = FakeNode()
        gain(node)
        keys = [k for k, _v in node.writes]
        last_ud = len(keys) - 1 - keys[::-1].index(('Motor', 'ud'))
        last_mode = len(keys) - 1 - keys[::-1].index(('SetModeOfOperation', None))
        self.assertLess(last_ud, last_mode,
                        'ud must reach zero before the mode is left')


if __name__ == '__main__':
    unittest.main()
