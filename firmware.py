"""firmware.py -- which application firmware a drive runs, and what it can do.

Two firmware lines are in the field:

  * 4.3.x (4.3.0-4.3.5): the shipping line.  Gains are IEEE-754 bit patterns
    in UNSIGNED32 objects, Rt/Lt are in 0.01 ohm / 0.01 mH (line-to-line),
    the current-sense bias is in ADC counts, polarity is 0x3013,3, 0x3013,5
    is a lag factor and 0x3024,4 a velocity gain factor in 0.01.  There is no
    logger/injection (0x3004/0x3015), so nothing that identifies a motor on
    the host can run, and no runout (0x3027) or cogging (0x302E) table.
  * 4.4.0 and later (the 4.5.0 release): Rt/Lt in mOhm / uH, bias in Q12.4,
    polarity in DS402 0x607E, 0x3013,5 an encoder latency trim in us and
    0x3024,4 the velocity bandwidth in 0.1 Hz.

detect() reads 0x100A once and probes one object per feature, and caches
the answer on the node object, so a GUI can ask firmware.has(node, ...) as
often as it likes.  Call forget() (or detect(refresh=True)) after flashing.

The version decides the units (legacy()).  On 4.3.x each feature is probed
(development builds carried some ahead of the release); every 4.4.0+ build in
the field has them all, so there only the 4.3.x polarity object is probed (a
4.4 development build still kept it).  An unreadable version is treated as
the current line unless the drive has 0x3013,3 and no capture objects.
"""

import collections

try:
    from canopen.sdo.exceptions import SdoAbortedError, SdoCommunicationError
except ImportError:                     # pragma: no cover - canopen is a dependency
    SdoAbortedError = SdoCommunicationError = Exception

from .flash import read_version

#: The first version with the 4.5.0 units (mOhm/uH, Q12.4 bias, 0x607E ...).
CURRENT_LINE = (4, 4, 0)
#: The release users are told to update to.
RECOMMENDED = '4.5.0'

#: name -> (probe object, what it is).  A feature is present when its probe
#: object can be read.
Feature = collections.namedtuple('Feature', 'probe label')
FEATURES = collections.OrderedDict([
    ('capture', Feature((0x3015, 1), 'logger captures and current injection (0x3004/0x3015)')),
    ('channel_logger', Feature((0x3004, 7), 'the channel logger (0x3004,7)')),
    ('volt_stream', Feature((0x3004, 13), 'the voltage stream (0x3004,13)')),
    ('runout', Feature((0x3027, 1), 'encoder runout compensation (0x3027)')),
    ('cogging', Feature((0x302E, 1), 'cogging compensation (0x302E)')),
    ('float_kt', Feature((0x3011, 14), 'the float Kt (0x3011,14)')),
    ('lq', Feature((0x3011, 12), 'Lq (0x3011,12)')),
    ('l_hf', Feature((0x3011, 13), 'the ripple inductance (0x3011,13)')),
    ('friction', Feature((0x3011, 17), 'stored drag friction (0x3011,17-18)')),
    ('friction_ff', Feature((0x2381, 3), 'friction feed-forward and settle ki (0x2381,3-4)')),
    ('rated_units', Feature((0x6075, 0), 'rated-current units (0x6075)')),
    ('isense_telemetry', Feature((0x3016, 1), 'current-sense window telemetry (0x3016)')),
    ('isense_slope', Feature((0x3008, 7), 'current-sense slope and fold (0x3008/9,7-8)')),
    ('pwm_periods', Feature((0x3001, 13), 'PWM periods per slot (0x3001,13)')),
    ('heartbeat_consumer', Feature((0x1016, 1), 'the heartbeat consumer (0x1016)')),
    ('legacy_polarity', Feature((0x3013, 3), 'polarity in 0x3013,3 (4.3.x)')),
    # Derived, not probed: the 4.5.0 units and CSV layout (not legacy).
    ('current_line', Feature(None, 'the 4.5.0 units and configuration layout')),
])


class FirmwareUnsupported(RuntimeError):
    """The drive's firmware lacks what was asked for."""

    def __init__(self, fw, missing, what=None):
        self.firmware = fw
        self.missing = tuple(missing)
        labels = ', '.join(FEATURES[m].label if m in FEATURES else m
                           for m in self.missing)
        RuntimeError.__init__(self, '{} needs firmware {} or later; this drive runs {} '
                              '(missing {})'.format(what or 'This', RECOMMENDED,
                                                    fw.text, labels))


class Firmware(collections.namedtuple('Firmware', 'version text features')):
    """version: (major, minor, patch) or None when 0x100A could not be read;
    text: what 0x100A said ('?' if nothing); features: frozenset of names."""

    @property
    def legacy(self):
        """True on 4.3.x: the old units and objects (see the module notes).
        With an unreadable version, a drive is legacy when it has the 4.3.x
        polarity object and not the capture objects."""
        if self.version is not None:
            return self.version < CURRENT_LINE
        return 'legacy_polarity' in self.features and 'capture' not in self.features

    def has(self, *names):
        return all(n in self.features for n in names)

    def missing(self, *names):
        return [n for n in names if n not in self.features]

    def require(self, *names, what=None):
        missing = self.missing(*names)
        if missing:
            raise FirmwareUnsupported(self, missing, what)

    def summary(self):
        """One line for a status bar or log."""
        if self.legacy:
            return 'firmware {} (4.3.x: identification, tuning captures, runout and ' \
                   'cogging need {})'.format(self.text, RECOMMENDED)
        return 'firmware {}'.format(self.text)


_ATTR = '_p4_firmware'


def _probe(node, index, sub):
    try:
        node.sdo.upload(index, sub)
        return True
    except SdoCommunicationError:         # no answer: not evidence of absence
        raise
    except Exception:                     # abort, or absent from a test fake
        return False


def _version(node):
    try:
        text = read_version(node)
        version = tuple(int(x) for x in text.split('.')[:3])
        if len(version) < 2:
            return None, '?'
        return version, text
    except Exception:                     # no answer, abort, garbage, or a test fake
        return None, '?'


def detect(node, refresh=False):
    """The node's Firmware, read once and cached on the node."""
    fw = getattr(node, _ATTR, None)
    if isinstance(fw, Firmware) and not refresh:
        return fw
    version, text = _version(node)
    if version is not None and version >= CURRENT_LINE:
        probed = ('legacy_polarity',)
    elif version is not None:
        probed = [name for name, f in FEATURES.items() if f.probe is not None]
    else:
        probed = ('legacy_polarity', 'capture')
    features = frozenset(name for name in probed if _probe(node, *FEATURES[name].probe))
    fw = Firmware(version, text, features)
    if not fw.legacy:
        assumed = (name for name, f in FEATURES.items()
                   if f.probe is not None and name != 'legacy_polarity')
        fw = fw._replace(features=features | set(assumed) | {'current_line'})
    try:
        setattr(node, _ATTR, fw)
    except AttributeError:                # a node that takes no attributes
        pass
    return fw


def forget(node):
    """Drop the cached answer (after a flash or a reset into new firmware)."""
    try:
        delattr(node, _ATTR)
    except AttributeError:
        pass


def has(node, *names):
    return detect(node).has(*names)


def legacy(node):
    return detect(node).legacy


def require(node, *names, what=None):
    """Raise FirmwareUnsupported unless the node has every feature in *names*."""
    detect(node).require(*names, what=what)


# --------------------------------------------------------------------------
# Units that changed between the lines.  Both lines keep Rt/Lt line-to-line.

def rt_ohm(node, raw):
    """0x3011,5 -> ohm line-to-line (0.01 ohm on 4.3.x, mOhm after)."""
    return raw / (100.0 if legacy(node) else 1000.0)


def rt_raw(node, ohm):
    return int(round(ohm * (100.0 if legacy(node) else 1000.0)))


def lt_h(node, raw):
    """0x3011,6 -> H line-to-line (0.01 mH on 4.3.x, uH after)."""
    return raw / (1e5 if legacy(node) else 1e6)


def lt_raw(node, henry):
    return int(round(henry * (1e5 if legacy(node) else 1e6)))
