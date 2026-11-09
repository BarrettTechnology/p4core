"""Unified CAN adapter factory with runtime auto-detection.

Single entry point (`make_network`) used by both the GUI and CLI. On Windows
the adapter is chosen automatically: if a CandleLight CANable is connected
(USB VID 0x1D50 / PID 0x606F) we use the in-tree CandlelightBus driver;
otherwise we fall back to the existing PCAN path. On Linux both adapter
types appear as SocketCAN interfaces (peak-pcan / gs_usb kernel drivers),
so the dispatch is moot and we keep the SocketCAN call. macOS keeps the
existing PCAN path.

Detection is cheap (a USB enumeration with libusb-package's bundled
backend), so make_network probes every time it's called. If you plug in a
CandleLight after starting the app, the next CAN operation will pick it up
without a restart.
"""

import platform
import subprocess
import os
import socket
import struct
import time
import canopen


CANDLELIGHT_VID = 0x1D50
CANDLELIGHT_PID = 0x606F

try:
    from .candlelight_bus import CandlelightBus
    _HAS_CANDLELIGHT = True
except ImportError:             # pyusb / libusb not installed
    _HAS_CANDLELIGHT = False


def _candlelight_present():
    """Return True if a CandleLight CANable is enumerated on USB. Returns
    False (rather than raising) for any error so callers can fall back to
    PCAN whenever detection is inconclusive."""
    if not _HAS_CANDLELIGHT:
        return False
    try:
        import usb.core
        try:
            import libusb_package
            backend = libusb_package.get_libusb1_backend()
        except Exception:
            backend = None
        dev = usb.core.find(idVendor=CANDLELIGHT_VID,
                            idProduct=CANDLELIGHT_PID,
                            backend=backend)
        return dev is not None
    except Exception:
        return False


def iface_is_candlelight(can_device):
    """True if the SocketCAN interface `can_device` is backed by a CandleLight /
    CANable adapter (the gs_usb kernel driver).

    On Linux this is read straight from sysfs, so it is exact even when a Peak
    PCAN is also plugged in -- only the gs_usb-backed interface reports True. On
    non-Linux platforms there is no SocketCAN interface to inspect, so fall back
    to the USB presence probe.

    Used to gate the USB-level adapter reset: only a CANable exhibits the
    full-TX-buffer pathology a reset fixes, and only a CANable should be
    USB-reset (a Peak recovers on the next reconnect)."""
    if platform.system() != 'Linux':
        return _candlelight_present()
    try:
        driver = os.path.basename(os.path.realpath(
            '/sys/class/net/{}/device/driver'.format(can_device)))
        return driver == 'gs_usb'
    except Exception:
        return False


def list_can_interfaces():
    """Return the CAN interfaces currently present on the system, best-effort.

    Linux: every /sys/class/net entry whose ARPHRD type is CAN (280) -- i.e. real
    SocketCAN interfaces (canX / slcanX), regardless of up/down state. This is what
    the puck apps connect to (the shared udev/systemd setup brings CANable/PCAN
    adapters up as canX automatically).

    Other platforms: returns [] -- there is no SocketCAN to enumerate, so callers
    fall back to the configured device / PCAN channel selection.
    """
    if platform.system() != 'Linux':
        return []
    names = []
    net = '/sys/class/net'
    try:
        for name in sorted(os.listdir(net)):
            try:
                with open(os.path.join(net, name, 'type')) as f:
                    if f.read().strip() == '280':   # ARPHRD_CAN
                        names.append(name)
            except OSError:
                continue
    except OSError:
        pass
    return names


#: reset_can.sh's 1M/5M timing: sample points and SJW (in tq of a 160 MHz clock).
_FD_TIMING = ('sample-point', '0.687', 'sjw', '50',
              'dsample-point', '0.687', 'dsjw', '10')


def bring_iface_up(can_device, bitrate=1_000_000, data_bitrate=5_000_000):
    """BEST-EFFORT bring-up of a SocketCAN interface via `ip link`, no sudo.

    Mirrors scripts/reset_can.sh (link down -> set bitrate -> txqueuelen -> up).
    The down->up cycle FLUSHES a wedged SocketCAN TX queue, so it doubles as the
    cheap recovery for the 'transmit buffer full' pathology and re-ups a canN
    left DOWN by a USB re-enumeration.

    This NEVER escalates privileges: `ip link` is run directly, so it only takes
    effect when the app already has the rights (root / CAP_NET_ADMIN). In the
    ordinary unprivileged case the calls just fail and we return False quietly --
    the caller then leans on the shipped udev+systemd auto-bring-up
    (61-can-up.rules / can-up@.service) or a replug to re-up the link. We never
    prompt for a password or ask the user to run a command.

    Returns True if the link was brought up, False otherwise (best-effort)."""
    if platform.system() != 'Linux':
        return False

    def _ip(args, check=True):
        return subprocess.run(['ip', 'link'] + args,
                              stdout=subprocess.DEVNULL,
                              stderr=subprocess.DEVNULL,
                              timeout=10, check=check)

    try:
        _ip(['set', can_device, 'down'], check=False)
        # CAN-FD with the bench timing (68.7 % sample points, SJW sized for the
        # CANable's 160 MHz clock; 1M/5M only), then CAN-FD on the kernel's
        # default timing, then classic for adapters without FD.
        fd = ['bitrate', str(bitrate), 'dbitrate', str(data_bitrate), 'fd', 'on']
        tries = [fd, ['bitrate', str(bitrate)]]
        if (bitrate, data_bitrate) == (1_000_000, 5_000_000):
            tries.insert(0, fd[:4] + list(_FD_TIMING) + fd[4:])
        for i, opts in enumerate(tries):
            try:
                _ip(['set', can_device, 'type', 'can'] + opts)
                break
            except subprocess.CalledProcessError:
                if i == len(tries) - 1:
                    raise
        # restart-ms is best-effort: gs_usb / CANable rejects it ("Device
        # doesn't support restart from Bus Off"), which must not fail bring-up.
        _ip(['set', can_device, 'type', 'can', 'restart-ms', '100'], check=False)
        _ip(['set', can_device, 'txqueuelen', '1000'], check=False)
        _ip(['set', can_device, 'up'])
        return True
    except (FileNotFoundError, subprocess.TimeoutExpired,
            subprocess.CalledProcessError):
        # Unprivileged / no `ip` / busy — best-effort, stay silent and let the
        # systemd auto-bring-up or a replug handle it.
        return False


#: The CAN bring-up files puck-can-common/-peak/-canable install under /usr/lib,
#: and the /etc paths that shadow them (an /etc file of the same name wins
#: in udev and systemd).  P4-Checkout and Puck Fader debs, and the old
#: setup-socketcan.sh, put their own copies in /etc.
CAN_UP_FILES = (
    ('/usr/lib/systemd/system/can-up@.service', '/etc/systemd/system/can-up@.service'),
    ('/usr/lib/udev/rules.d/61-can-up.rules', '/etc/udev/rules.d/61-can-up.rules'),
    ('/usr/lib/udev/rules.d/60-can.rules', '/etc/udev/rules.d/60-can.rules'),
    ('/usr/lib/udev/rules.d/90-canable.rules', '/etc/udev/rules.d/90-canable.rules'),
)


def shadowed_can_files():
    """The /etc files that override the installed CAN bring-up (an /etc
    copy beside our /usr/lib one, with different contents): [(etc, usr)].
    On such a machine can0 comes up with another app's timing (the kernel's
    75 % sample point, SJW 20, not the bench's 68.7 %, 50/10), and the
    adapter's error counters climb under load -- a puck went bus-off in a
    cogging capture.  Linux only; [] elsewhere."""
    if platform.system() != 'Linux':
        return []
    out = []
    for usr, etc in CAN_UP_FILES:
        try:
            if not (os.path.exists(usr) and os.path.exists(etc)):
                continue
            with open(usr, 'rb') as a, open(etc, 'rb') as b:
                if a.read() != b.read():
                    out.append((etc, usr))
        except OSError:
            continue
    return out


def shadowed_can_files_message(files=None):
    """A user message for shadowed_can_files(), or '' when there are none."""
    files = shadowed_can_files() if files is None else files
    if not files:
        return ''
    return ('Another program\'s CAN set-up overrides this one:\n\n{}\n\n'
            'CAN interfaces come up with that file\'s timing, not the puck '
            'timing, and the bus can fail under load (adapter error '
            'warnings, CAN bus-off).  Re-run this app\'s installer, which '
            'moves these files aside, or remove them (sudo rm) and replug '
            'the CAN adapter.'.format(
                '\n'.join('  {}  (overrides {})'.format(e, u)
                          for e, u in files)))


def iface_details(can_device):
    """What the kernel says about SocketCAN *can_device*: dict(state,
    bitrate, sample_point, sjw, dbitrate, dsample_point, dsjw, bus_off,
    error_passive, error_warning, restarts, berr_tx, berr_rx), any of them
    None when not reported; None when it cannot be read (not Linux, no
    `ip`, no such interface, not a CAN link).  `ip -details -statistics
    -json` (iproute2 5.5, Ubuntu 20.04, and later)."""
    if platform.system() != 'Linux' or not can_device:
        return None
    import json
    try:
        out = subprocess.run(['ip', '-details', '-statistics', '-json', 'link',
                              'show', 'dev', str(can_device)],
                             capture_output=True, text=True, timeout=5)
        if out.returncode != 0:
            return None
        info = json.loads(out.stdout)[0].get('linkinfo') or {}
    except (OSError, ValueError, IndexError, subprocess.TimeoutExpired):
        return None
    if info.get('info_kind') != 'can':
        return None
    data = info.get('info_data') or {}
    xs = info.get('info_xstats') or {}
    bt = data.get('bittiming') or {}
    dbt = data.get('data_bittiming') or {}
    berr = data.get('berr_counter') or {}

    def num(v, kind=float):
        try:
            return kind(v)
        except (TypeError, ValueError):
            return None
    return dict(
        state=data.get('state'),
        bitrate=num(bt.get('bitrate'), int),
        sample_point=num(bt.get('sample_point')),
        sjw=num(bt.get('sjw'), int),
        dbitrate=num(dbt.get('bitrate'), int),
        dsample_point=num(dbt.get('sample_point')),
        dsjw=num(dbt.get('sjw'), int),
        bus_off=num(xs.get('bus_off'), int),
        error_passive=num(xs.get('error_passive'), int),
        error_warning=num(xs.get('error_warning'), int),
        restarts=num(xs.get('restarts'), int),
        berr_tx=num(berr.get('tx'), int),
        berr_rx=num(berr.get('rx'), int))


def iface_timing_text(can_device):
    """'can0: 1000000 bit/s, sample point 0.687, SJW 50; data 5000000 bit/s,
    sample point 0.687, SJW 10' for the log, or '' when it cannot be read."""
    d = iface_details(can_device)
    if not d or not d['bitrate']:
        return ''

    def part(rate, sp, sjw):
        return '{} bit/s, sample point {}, SJW {}'.format(
            rate, '?' if sp is None else '{:.3f}'.format(sp),
            '?' if sjw is None else sjw)
    txt = '{}: {}'.format(can_device, part(d['bitrate'], d['sample_point'],
                                           d['sjw']))
    if d['dbitrate']:
        txt += '; data ' + part(d['dbitrate'], d['dsample_point'], d['dsjw'])
    return txt


def network_channel(network):
    """The SocketCAN interface name a canopen Network is on, or None."""
    bus = getattr(network, 'bus', None)
    ch = getattr(bus, 'channel', None) or getattr(bus, 'channel_info', None)
    if isinstance(ch, str) and ch.startswith(('can', 'vcan', 'slcan')):
        return ch.split()[0]
    return None


def bus_trouble(network, before=None):
    """The host adapter's view of a CAN fault on *network*'s interface: a
    text such as 'can0 is BUS-OFF' or 'can0 went bus-off 1 time(s), error
    passive 3 time(s)' (counted since *before*, an earlier iface_details()),
    or '' when it saw none or cannot tell."""
    ch = network_channel(network)
    d = iface_details(ch) if ch else None
    if not d:
        return ''
    parts = []
    if d['state'] and d['state'] not in ('ERROR-ACTIVE', 'STOPPED'):
        parts.append('is {}'.format(d['state']))
    b = before or {}
    for key, label in (('bus_off', 'went bus-off'),
                       ('error_passive', 'went error-passive')):
        n = (d[key] or 0) - (b.get(key) or 0)
        if n > 0:
            parts.append('{} {} time(s)'.format(label, n))
    return '{} {}'.format(ch, ', '.join(parts)) if parts else ''


class CanBusUnavailable(RuntimeError):
    """The CAN interface can't be brought up cleanly. `state` is one of:
      'missing' -- interface not present (adapter unplugged / driver not loaded)
      'down'    -- interface exists but is administratively DOWN
      'in_use'  -- another CANopen master is already driving the bus (two masters
                   would collide -> SDO aborts). See `message` for a user string.
    Callers should surface `message` and NOT bring up their own master; re-probe on
    the next scan and proceed once the bus is free."""
    def __init__(self, state, message, counts=None):
        super().__init__(message)
        self.state = state
        self.message = message
        self.counts = counts or {}


def _iface_up(dev):
    """True/False if the SocketCAN interface is administratively UP (IFF_UP), or None
    if that can't be read. CAN links usually report operstate 'unknown' even when up,
    so we read the IFF_UP flag directly rather than operstate."""
    try:
        with open('/sys/class/net/{}/flags'.format(dev)) as f:
            return bool(int(f.read().strip(), 16) & 0x1)   # IFF_UP = 0x1
    except (OSError, ValueError):
        return None


def probe_interface(can_device, listen_s=0.30):
    """Pre-flight a SocketCAN interface BEFORE bringing up our own CANopen master.

    Returns (ready, state, message, counts):
      ready True  -> safe to connect ('ok').
      ready False -> state in {'missing','down','in_use'} with a human `message`.
    Non-Linux / non-SocketCAN paths return (True,'ok','',{}) -- nothing to probe.

    'in_use' is detected by PASSIVELY listening (~`listen_s`) for another master's
    traffic -- SYNC (0x80) or SDO (0x580-0x67F) -- before we transmit anything. On
    SocketCAN the bus is shareable, so two masters don't error on connect; they just
    collide mid-transfer (the SDO aborts we keep catching). Catching it here, up
    front, lets the app pause and wait for the bus to free instead. Fails OPEN: any
    probe error returns ready -> we never falsely block a good bus."""
    if platform.system() != 'Linux':
        return True, 'ok', '', {}          # PCAN / CandleLight: nothing to sniff here
    dev = str(can_device)
    if dev not in list_can_interfaces():
        return (False, 'missing',
                "CAN interface '{}' not found -- adapter unplugged, driver not loaded, or the "
                "interface was never created.".format(dev), {})
    if _iface_up(dev) is False:
        return (False, 'down',
                "CAN interface '{}' is DOWN. Bring it up first, e.g.:\n"
                "    sudo ip link set {} up type can bitrate 1000000".format(dev, dev), {})

    # Passive listen for another master's traffic. Raw AF_CAN receiver -- receive-only,
    # so it can't disturb an innocent bus; multiple listeners are fine on SocketCAN.
    try:
        s = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        s.bind((dev,))
        s.settimeout(0.05)
    except OSError:
        return True, 'ok', '', {}          # can't open a listener -> fail open, let connect try
    sync = sdo = pdo = 0
    t0 = time.time()
    try:
        while time.time() - t0 < listen_s:
            try:
                frame = s.recv(16)         # struct can_frame: id(4) dlc(1) pad(3) data(8)
            except socket.timeout:
                continue
            except OSError:
                break
            if len(frame) < 8:
                continue
            can_id = struct.unpack('<I', frame[:4])[0]
            if can_id & 0x80000000:        # extended (29-bit) frame -> not 11-bit CANopen
                continue
            cob = can_id & 0x7FF
            if cob == 0x80:                        # SYNC -> a master is running a SYNC producer
                sync += 1
            elif 0x580 <= cob <= 0x67F:            # SDO response (0x580+) / request (0x600+)
                sdo += 1
            elif 0x180 <= cob <= 0x57F:            # PDO -- only flows while a SYNC master drives
                pdo += 1
    finally:
        try:
            s.close()
        except OSError:
            pass

    # Only MASTER-originated traffic proves another app is driving the bus. Node-originated frames do
    # NOT: heartbeats/bootup (0x700+), EMCY (0x81-0xFF), AND async/event-driven PDOs (0x180-0x57F) --
    # e.g. a left-on ADC-monitor stream keeps the PUCK emitting PDOs with no master present. Counting
    # PDO as "another master" false-blocked the connect (the puck's own stream). So trip 'in_use' only
    # on SYNC (0x80) or SDO (0x580-0x67F), which cannot occur without a second master. PDO is kept for
    # diagnostics only.
    if sync or sdo:
        seen = ", ".join(p for p in (
            "{} SYNC".format(sync) if sync else "",
            "{} SDO".format(sdo) if sdo else "") if p)
        return (False, 'in_use',
                "CAN interface '{}' is already being driven by another CANopen master ({} in "
                "{:.0f} ms). Another Puck Utility / pucktuner window or a script is likely running "
                "-- close it, then rescan.".format(dev, seen, listen_s * 1000),
                {'sync': sync, 'sdo': sdo, 'pdo': pdo})
    return True, 'ok', '', {'sync': 0, 'sdo': 0, 'pdo': pdo}


def is_tx_buffer_error(exc):
    """True if `exc` is the 'transmit buffer full' CAN error.

    A device on the bus that never ACKs (classically a blank/erased puck) leaves
    frames unacknowledged until SocketCAN's TX buffer fills and python-can raises
    CanOperationError('Transmit buffer full'). This is the only CAN error a
    USB-level adapter reset actually clears -- other CanErrors (bus-off, network
    down, device unplugged) recover on a plain reconnect/rescan, so they must NOT
    trigger a reset."""
    return 'buffer' in str(exc).lower()


def pcan_channel(can_device):
    """Map a CAN port selector to a PCAN channel name. Accepts a bare bus
    index (e.g. '0' -> PCAN_USBBUS1), a SocketCAN-style 'canN' name
    (e.g. 'can0' -> PCAN_USBBUS1, as produced by the GUI port dropdown), or
    a full 'PCAN_USBBUSn' string passed verbatim."""
    text = str(can_device).strip()
    if text.upper().startswith('PCAN_'):
        return text.upper()
    if text.lower().startswith('can') and text[3:].isdigit():
        return 'PCAN_USBBUS' + str(int(text[3:]) + 1)
    try:
        return 'PCAN_USBBUS' + str(int(text) + 1)
    except ValueError:
        raise ValueError(
            f"invalid PCAN device {can_device!r}: expected a bus index "
            f"(e.g. 0 for PCAN_USBBUS1), a 'canN' name, or a PCAN_USBBUSn name")


# SDO abort codes whose appearance almost always means another CANopen master
# is active on the same bus at the same time — a second copy of PuckUtility or
# PuckTuner left open, both apps connected at once, or a tool flooding the bus
# with SYNC/PDO traffic.  When two clients transmit at once their CAN frames
# interleave and corrupt each other's multi-frame SDO sequence, so the puck
# rejects the malformed request with one of these protocol-level aborts.
#
# These are distinct from object-level aborts (e.g. 0x06020000 "object does not
# exist", 0x06010002 "write to a read-only object") which point at the request
# itself, not at bus contention — those are deliberately excluded here.
SDO_CONTENTION_CODES = {
    0x05040001: "client/server command specifier not valid or unknown",
    0x05030000: "toggle bit not alternated",
    0x05040000: "SDO protocol timed out",
    0x05040002: "invalid block size",
    0x05040003: "invalid sequence number",
}


def sdo_contention_message(exc):
    """If `exc` is an SDO abort whose code indicates competing SDO/PDO traffic
    on the bus, return a clear, user-facing explanation; otherwise return None.

    Accepts a canopen SdoAbortedError (anything with an integer ``.code``) or a
    bare integer abort code.  Returning None lets callers fall through to their
    normal generic error message for ordinary, non-contention aborts.
    """
    code = getattr(exc, "code", exc)
    if not isinstance(code, int) or code not in SDO_CONTENTION_CODES:
        return None
    return (
        "SDO abort 0x{:08X} — likely CAN bus contention.\n"
        "Close any other PuckUtility/PuckTuner window using this bus and retry."
    ).format(code)


# CANopen 0x1018:2 (Product Code) -> Puck model name.
#
# Two encodings exist in the field:
#   * Legacy firmware reports a small numeric code (e.g. 5707 -> 'P4-16').
#   * Newer firmware packs the 4-character model tag into the UNSIGNED32 as
#     ASCII, most-significant byte first: 0x50343332 == b'P432' -> 'P4-32'.
# `_ProductCodeModels` resolves both: numeric codes are looked up in the table,
# anything else is decoded as a 4-byte ASCII tag on the fly, so a new variant
# only needs an entry in _ASCII_MODEL_TAGS (or none, if the table below already
# covers it).
_ASCII_MODEL_TAGS = {
    b'P416': 'P4-16',
    b'P432': 'P4-32',
    b'P437': 'P4-37',
    b'P442': 'P4-42',
}


def _model_from_ascii(code):
    """Return the model name if `code` is an ASCII-packed model tag (e.g. the
    int 0x50343332 / 1345598258 -> 'P4-32'), else None."""
    try:
        raw = int(code).to_bytes(4, 'big')
    except (TypeError, ValueError, OverflowError):
        return None
    return _ASCII_MODEL_TAGS.get(raw)


class _ProductCodeModels(dict):
    """Maps a product code to a Puck model name, accepting both legacy numeric
    codes (stored as ordinary dict items) and newer ASCII-packed tags (decoded
    on the fly). Drop-in for the plain dict the call sites used before: `.get`
    and `in` both transparently handle the ASCII form."""

    def get(self, code, default=None):
        if dict.__contains__(self, code):
            return dict.__getitem__(self, code)
        model = _model_from_ascii(code)
        return model if model is not None else default

    def __contains__(self, code):
        return dict.__contains__(self, code) or _model_from_ascii(code) is not None


PRODUCT_CODE_MODELS = _ProductCodeModels({
    5707: 'P4-16',
    1323: 'P4-37',
    1950: 'P4-37',
    5755: 'P4-42',
    5760: 'P4-32',
})


def model_from_product_code(code):
    """Resolve a CANopen product code (0x1018:2) to a Puck model name, or None.

    Single converged entry point shared by puckutility, pucktuner, and
    P4-checkout. Accepts BOTH encodings: legacy small-integer codes (e.g.
    5760 -> 'P4-32') and newer ASCII-packed codes (e.g. 0x50343332 = b'P432'
    -> 'P4-32'). Thin wrapper over PRODUCT_CODE_MODELS so callers can use a
    plain function instead of the dict-with-.get() form."""
    return PRODUCT_CODE_MODELS.get(code)


def make_network(can_device, bitrate=1_000_000, fd=False, data_bitrate=None,
                 network_cls=None):
    """Create and connect a canopen.Network. The adapter is auto-selected:
    CandleLight if present on USB, otherwise PCAN (Windows/macOS) or
    SocketCAN (Linux).

    `can_device` is the channel selector for the PCAN/SocketCAN paths. For
    CandleLight, integer values select which USB instance to open when
    multiple adapters are connected (0 = first found).

    `fd` enables CAN-FD frame transmission; `data_bitrate` is the FD data-phase
    rate (e.g. 5_000_000). For SocketCAN the data bitrate is set on the LINK
    (`ip link ... dbitrate`), so only `fd` is passed to python-can there; the
    CandleLight/PCAN drivers take the data_bitrate directly. Both default off so
    callers that don't opt in stay on classic CAN.
    """
    system = platform.system()
    network_cls = network_cls or canopen.Network

    if system == "Windows" and _candlelight_present():
        try:
            usb_index = int(str(can_device).strip())
        except (TypeError, ValueError):
            usb_index = 0
        bus = CandlelightBus(channel=usb_index, bitrate=bitrate,
                             fd=fd, data_bitrate=data_bitrate)
        network = network_cls(bus=bus)
        network.connect()
        return network

    # PCAN/CandleLight configure the FD data phase in software; SocketCAN reads
    # it from the link, so it only wants fd=True (passing data_bitrate would be
    # an unexpected kwarg). Build the FD kwargs per backend accordingly.
    pcan_fd_kwargs = {'fd': True, 'data_bitrate': data_bitrate} if fd else {}

    network = network_cls()
    if system == "Windows":
        network.connect(bustype='pcan',
                        channel=pcan_channel(can_device),
                        bitrate=bitrate, **pcan_fd_kwargs)
    elif system == "Linux":
        network.connect(bustype='socketcan', channel=can_device,
                        bitrate=bitrate, **({'fd': True} if fd else {}))
    elif system == "Darwin":
        network.connect(bustype='pcan', channel='PCAN_USBBUS1',
                        bitrate=bitrate, **pcan_fd_kwargs)
    else:
        raise RuntimeError(f"unsupported platform for CAN access: {system}")

    # A network that decides FD per frame itself (p4core.session.P4Network,
    # can_fd switchable at run time) must not have every frame forced to FD.
    if fd and not hasattr(network, 'can_fd'):
        _enable_fd_frames(network)
    # P4Network retries a full queue in its own send_message.
    if not hasattr(network, 'can_fd'):
        _tolerate_tx_backpressure(network)
    return network


def _enable_fd_frames(network):
    """Make outgoing canopen frames actual CAN-FD frames.

    python-can transmits a Message as a CLASSIC frame unless ``msg.is_fd`` is
    set -- and canopen builds plain (classic) Messages, so on an FD-enabled
    SocketCAN/PCAN socket the frames still go out 8-byte at the 1 Mbit nominal
    rate. Wrap the bus ``send`` to flag every outgoing message FD + BRS so the
    data phase actually uses the link's FD data bitrate. (The CandleLight driver
    does this inside its own send, so it is not wrapped here.)
    """
    bus = network.bus
    _orig_send = bus.send

    def _send_fd(msg, *args, **kwargs):
        msg.is_fd = True
        msg.bitrate_switch = True
        return _orig_send(msg, *args, **kwargs)

    bus.send = _send_fd


#: How long a send waits for room in the adapter's transmit queue (s).
TX_TIMEOUT = 0.5


def _tolerate_tx_backpressure(network, timeout=TX_TIMEOUT):
    """Give outgoing frames a real send timeout.

    canopen's Network.send_message calls ``bus.send(msg)`` with no timeout,
    so python-can's socketcan backend polls the socket with select(..., 0)
    and the first moment of TX backpressure raises "Transmit buffer full".
    The gs_usb (CANable) TX URB pool is shallow enough that the 127-frame
    burst of ``scanner.search()`` overruns it whenever the previous burst
    has not drained, and the scan then reported "No Pucks Found" with
    pucks on the bus.  Waiting briefly for queue space costs nothing on a
    healthy bus; a bus that never drains (nothing ACKs) still raises, after
    the timeout.

    Wrapped outermost (after _enable_fd_frames), and only a timeout the
    caller did not give is filled in.  The CandleLight driver blocks on its
    own USB write timeout and is not wrapped (make_network returns before).
    """
    bus = network.bus
    _orig_send = bus.send

    def _send_waiting(msg, *args, **kwargs):
        # A positional timeout would collide with the keyword.
        if not args and kwargs.get('timeout') is None:
            kwargs['timeout'] = timeout
        return _orig_send(msg, *args, **kwargs)

    bus.send = _send_waiting


# ---------------------------------------------------------------------------
# SocketCAN port listing with bus / vendor / state (Linux sysfs, no root)
# ---------------------------------------------------------------------------
# From p4gui's can_ports.py.  list_can_interfaces() above gives bare names;
# list_can_ports() gives the labels a port chooser shows, e.g.
# "can0, USB (CANable), up".
import glob as _glob

SYS_NET = '/sys/class/net'

# (bus_label, vendor-id) -> friendly name.  Vendor ids are lower-case hex as
# reported by sysfs (PCI ``vendor`` minus the ``0x``; USB ``idVendor``).
VENDOR_ALIASES = {
    ('PCI', '001c'): 'Peak',      # PEAK-System Technik (PCAN-PCI)
    ('USB', '0c72'): 'Peak',      # PEAK-System Technik (PCAN-USB)
    ('USB', '1d50'): 'CANable',   # OpenMoko / Geschwister Schneider (gs_usb)
}


def _read(path):
    """Return the stripped contents of *path*, or ``None`` if unreadable."""
    try:
        with open(path) as fh:
            return fh.read().strip()
    except OSError:
        return None


def _hexid(value):
    """Normalise a sysfs id (``0x001c`` / ``1D50``) to bare lower-case hex."""
    if value is None:
        return None
    if value.startswith('0x'):
        value = value[2:]
    return value.lower()


def _bus_type(devpath):
    """Bus subsystem name for the device behind *devpath* (e.g. ``usb``)."""
    if not os.path.exists(devpath):
        return None
    return os.path.basename(os.path.realpath(os.path.join(devpath, 'subsystem')))


def _usb_ids(devpath):
    """Return ``(vendor, product, buspath)`` for the USB device behind *devpath*."""
    path = os.path.realpath(devpath)
    while path and path != '/':
        vendor = _read(os.path.join(path, 'idVendor'))
        product = _read(os.path.join(path, 'idProduct'))
        if vendor and product:
            return _hexid(vendor), _hexid(product), os.path.basename(path)
        path = os.path.dirname(path)
    return None, None, None


def _pci_ids(devpath):
    """Return ``(vendor, device, address)`` for the PCI device behind *devpath*."""
    real = os.path.realpath(devpath)
    address = os.path.basename(real)
    domain, sep, rest = address.partition(':')
    if sep:
        address = rest  # strip the 0000: domain
    return (_hexid(_read(os.path.join(real, 'vendor'))),
            _hexid(_read(os.path.join(real, 'device'))),
            address)


def port_info(port):
    """Return an info dict for a single interface name such as ``can0``."""
    devpath = os.path.join(SYS_NET, port, 'device')
    state = _read(os.path.join(SYS_NET, port, 'operstate')) or 'unknown'
    bus = _bus_type(devpath)

    if bus == 'usb':
        bus_label = 'USB'
        vendor, product, busid = _usb_ids(devpath)
    elif bus == 'pci':
        bus_label = 'PCI'
        vendor, product, busid = _pci_ids(devpath)
    else:
        bus_label, vendor, product, busid = (bus or '?').upper(), None, None, None

    alias = VENDOR_ALIASES.get((bus_label, vendor))
    if alias:
        detail = alias
    elif vendor and product and busid:
        detail = '{}:{} @ {}'.format(vendor, product, busid)
    elif vendor and product:
        detail = '{}:{}'.format(vendor, product)
    else:
        detail = '?'

    label = '{}, {} ({}), {}'.format(port, bus_label, detail, state)
    return {
        'port': port,
        'bus': bus_label,
        'vendor': vendor,
        'product': product,
        'busid': busid,
        'alias': alias,
        'detail': detail,
        'label': label,
    }


def list_can_ports():
    """Return info dicts for every ``can*`` interface, sorted by name."""
    names = sorted(
        os.path.basename(p) for p in _glob.glob(os.path.join(SYS_NET, 'can*'))
    )
    return [port_info(name) for name in names]


def port_choices():
    """The CAN ports a GUI's port dropdown offers, as {'port', 'label'} dicts:
    the PCAN channels on Windows, every SocketCAN ``can*`` interface elsewhere
    (labelled "port, bus (id), state").  'port' is what make_network takes."""
    if platform.system() == 'Windows':
        return [{'port': p, 'label': p} for p in
                ('PCAN_USBBUS1', 'PCAN_USBBUS2', 'PCAN_USBBUS3', 'PCAN_USBBUS4')]
    return list_can_ports()
