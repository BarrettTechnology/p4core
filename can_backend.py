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
        # CAN-FD first; fall back to classic 1 Mbit for adapters without FD.
        try:
            _ip(['set', can_device, 'type', 'can', 'bitrate', str(bitrate),
                 'fd', 'on', 'dbitrate', str(data_bitrate)])
        except subprocess.CalledProcessError:
            _ip(['set', can_device, 'type', 'can', 'bitrate', str(bitrate)])
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
