"""
candlelight_bus.py  –  python-can BusABC implementation for the CANable 2.5
                       Candlelight firmware over WinUSB / libusb-1.0.dll

Drop-in replacement for any python-can bus.  Works with canopen directly:

    from candlelight_bus import CandlelightBus
    import canopen

    bus = CandlelightBus(channel=0, bitrate=1_000_000)
    network = canopen.Network(bus=bus)
    network.connect()          # no bustype/channel args needed

For CAN FD – every outgoing frame is automatically sent as FD+BRS:

    bus = CandlelightBus(channel=0, bitrate=1_000_000,
                         fd=True, data_bitrate=5_000_000)

References:
    https://github.com/Elmue/CANable-2.5-firmware-Slcan-and-Candlelight
    SampleApplication C++/Source/Candlelight/{Candlelight.cpp,.h,_def.h}
"""

import logging
import struct
import time as _time
from typing import Optional, Tuple

import can
import usb.core
import usb.util
import usb.backend.libusb1

logger = logging.getLogger(__name__)

# ── USB identifiers ──────────────────────────────────────────────────────────
VID            = 0x1D50   # OpenMoko / candleLight standard VID
PID            = 0x606F   # candleLight standard PID
IFACE_CANDLE   = 0
EP_OUT         = 0x02
EP_IN          = 0x81
EP_TIMEOUT_MS  = 500
BULK_READ_SIZE = 128      # RX_FIFO_BUF_SIZE

# ── USB request codes (eUsbRequest) ──────────────────────────────────────────
REQ_SET_BITTIMING    = 1
REQ_SET_MODE         = 2
REQ_GET_CAPABILITIES = 4
REQ_GET_VERSION      = 5
REQ_SET_BITTIMING_FD = 10
REQ_GET_CAPS_FD      = 11
REQ_GET_LAST_ERROR   = 22

RT_OUT = 0x41   # bmRequestType: Vendor | Interface | Host→Device
RT_IN  = 0xC1   # bmRequestType: Vendor | Interface | Device→Host

# ── Device flags (eDeviceFlags) ──────────────────────────────────────────────
FLAG_LOOPBACK        = 0x00002
FLAG_CAN_FD          = 0x00100
FLAG_BITTIMING_FD    = 0x00400
FLAG_PROTOCOL_ELMUE  = 0x04000
FLAG_DISABLE_TX_ECHO = 0x08000

MODE_RESET = 0
MODE_START = 1

# ── Frame flags (eFrameFlags) ────────────────────────────────────────────────
FRM_FDF = 0x02
FRM_BRS = 0x04
FRM_ESI = 0x08

CAN_FLAG_RTR = 0x40000000
CAN_FLAG_EXT = 0x80000000
CAN_MASK_11  = 0x000007FF
CAN_MASK_29  = 0x1FFFFFFF

# ── Message types (eMessageType) ─────────────────────────────────────────────
MSG_TX_FRAME = 10
MSG_TX_ECHO  = 11
MSG_RX_FRAME = 12
MSG_ERROR    = 13
MSG_STRING   = 14

FBK_SUCCESS = 2

# ── Struct layouts (1-byte packed, little-endian) ─────────────────────────────
ST_BITTIMING = struct.Struct('<IIIII')   # prop seg1 seg2 sjw brp  (20 B)
ST_MODE      = struct.Struct('<II')      # mode flags               (8 B)
ST_CAPS      = struct.Struct('<II8I')    # feature fclk 8×minmax   (40 B)
ST_CAPS_FD   = struct.Struct('<II8I8I')  # feature fclk nom data   (72 B)
ST_VERSION   = struct.Struct('<BBBBII')  # reserved×3 icount sw hw (12 B)
ST_TX_HDR    = struct.Struct('<BBBIB')   # size type flags id mark  (8 B)
ST_RX_HDR    = struct.Struct('<BBBI')    # size type flags id       (7 B)


# ─────────────────────────────────────────────────────────────────────────────
def _calc_bit_timing(fclk: int, bitrate: int, sample_point: float = 0.75):
    """Return (brp, seg1, seg2, sjw) for the target bitrate.

    Equation: baudrate = fclk / brp / (1 + seg1 + seg2)  with prop = 0.
    Searches BRP 1..512, picks solution closest to target baudrate and SP.
    Raises ValueError if no solution exists.
    """
    best, best_err = None, float('inf')
    for brp in range(1, 513):
        total_tq = fclk // (brp * bitrate)
        if total_tq < 3 or total_tq > 257:
            continue
        seg1 = max(1, round(sample_point * total_tq) - 1)
        seg2 = total_tq - 1 - seg1
        if seg2 < 1:
            seg2, seg1 = 1, total_tq - 2
        if seg1 < 1:
            continue
        actual_baud = fclk // (brp * (1 + seg1 + seg2))
        actual_sp   = (1 + seg1) / (1 + seg1 + seg2)
        err = abs(actual_baud - bitrate) / bitrate + abs(actual_sp - sample_point)
        if err < best_err:
            best_err, best = err, (brp, seg1, seg2, min(seg1, seg2))
    if best is None:
        raise ValueError(f"No valid bit timing for {bitrate} bps at fclk={fclk} Hz")
    return best


# ─────────────────────────────────────────────────────────────────────────────
class CandlelightBus(can.BusABC):
    """
    python-can BusABC implementation for CANable 2.5 Candlelight firmware.

    On construction the device is opened, bitrate(s) configured, and the CAN
    bus started.  The instance is immediately ready for send/recv.

    When fd=True every outgoing message is automatically flagged FD+BRS,
    so that canopen (and any other python-can user) gets FD framing without
    any changes to the upper layer.

    canopen usage::

        bus     = CandlelightBus(channel=0, bitrate=1_000_000)
        network = canopen.Network(bus=bus)
        network.connect()   # no bustype/channel – bus is already set

    Parameters
    ----------
    channel : int
        USB device index (0 = first adapter found).
    bitrate : int
        Nominal CAN bitrate in bits/s (default 1 Mbps).
    fd : bool
        Enable CAN FD mode.  All outgoing frames will use FD+BRS.
    data_bitrate : int
        FD data-phase bitrate in bits/s (default 5 Mbps, used only when fd=True).
    sample_point : float
        Nominal sample point fraction 0–1 (default 0.75 = 75%).
    data_sample_point : float
        FD data-phase sample point fraction (default 0.75).
    loopback : bool
        If True the adapter echoes TX frames back as RX (test without a second node).
    vid, pid : int
        USB vendor / product ID (defaults to standard candleLight 0x1D50:0x606F).
    """

    channel_info: str = "Candlelight"

    def __init__(
        self,
        channel: int = 0,
        bitrate: int = 1_000_000,
        fd: bool = False,
        data_bitrate: int = 5_000_000,
        sample_point: float = 0.75,
        data_sample_point: float = 0.75,
        loopback: bool = False,
        vid: int = VID,
        pid: int = PID,
        can_filters=None,
        **kwargs,
    ):
        self._dev  = None
        self._fd   = fd
        self._marker = 0

        # ── Open USB device ───────────────────────────────────────────────────
        # libusb-1.0.dll isn't shipped with Windows. Prefer the copy bundled
        # inside the `libusb-package` pip dependency (works in any Python env
        # without putting a DLL on PATH); fall back to pyusb's default search
        # path so a system-installed libusb still works.
        backend = None
        try:
            import libusb_package
            backend = libusb_package.get_libusb1_backend()
        except Exception:
            backend = None
        if backend is None:
            backend = usb.backend.libusb1.get_backend()
        if backend is None:
            raise can.CanInitializationError(
                "libusb-1.0.dll not found - install libusb-package "
                "(pip install libusb-package) or place libusb-1.0.dll on PATH"
            )

        dev = usb.core.find(idVendor=vid, idProduct=pid, backend=backend)
        if dev is None:
            raise can.CanInitializationError(
                f"Candlelight device {vid:04X}:{pid:04X} not found"
            )

        try:
            dev.set_configuration()
        except usb.core.USBError:
            pass

        usb.util.claim_interface(dev, IFACE_CANDLE)
        self._dev = dev
        self.channel_info = f"Candlelight {vid:04X}:{pid:04X} ch{channel}"

        # ── Reset and read capabilities ───────────────────────────────────────
        self._ctrl_out(REQ_SET_MODE, ST_MODE.pack(MODE_RESET, 0))

        raw = self._ctrl_in(REQ_GET_VERSION, ST_VERSION.size)
        _, _, _, icount, sw_ver, hw_ver = ST_VERSION.unpack(raw)
        logger.info("Candlelight fw sw=0x%08X hw=0x%08X ch=%d", sw_ver, hw_ver, icount)

        raw = self._ctrl_in(REQ_GET_CAPABILITIES, ST_CAPS.size)
        _feat, fclk_nom, *_ = ST_CAPS.unpack(raw)
        logger.info("fclk_nominal = %.1f MHz", fclk_nom / 1e6)

        raw = self._ctrl_in(REQ_GET_CAPS_FD, ST_CAPS_FD.size)
        _feat_fd, fclk_data, *_ = ST_CAPS_FD.unpack(raw)
        logger.info("fclk_data    = %.1f MHz", fclk_data / 1e6)

        # ── Configure bit timing ──────────────────────────────────────────────
        self._apply_bitrate(fclk_nom, bitrate, fd_data=False,
                            sample_point=sample_point)
        if fd:
            self._apply_bitrate(fclk_data, data_bitrate, fd_data=True,
                                sample_point=data_sample_point)

        # ── Start the bus ─────────────────────────────────────────────────────
        flags = FLAG_PROTOCOL_ELMUE | FLAG_DISABLE_TX_ECHO
        if fd:
            flags |= FLAG_CAN_FD | FLAG_BITTIMING_FD
        if loopback:
            flags |= FLAG_LOOPBACK
        self._ctrl_out(REQ_SET_MODE, ST_MODE.pack(MODE_START, flags))

        if fd:
            self._can_protocol = can.CanProtocol.CAN_FD

        # Must be last – sets _is_shutdown = False
        super().__init__(channel=channel, can_filters=can_filters, **kwargs)

    # ── BusABC interface ──────────────────────────────────────────────────────

    def send(self, msg: can.Message, timeout: Optional[float] = None) -> None:
        """Transmit a CAN or CAN FD frame.

        When the bus was opened with fd=True, every message is automatically
        sent as FD+BRS regardless of the message's own is_fd flag.
        """
        arb = msg.arbitration_id
        can_id = arb & (CAN_MASK_29 if msg.is_extended_id else CAN_MASK_11)
        if msg.is_extended_id:
            can_id |= CAN_FLAG_EXT
        if msg.is_remote_frame:
            can_id |= CAN_FLAG_RTR

        # Upgrade to FD+BRS when the bus is in FD mode
        use_fd  = msg.is_fd  or self._fd
        use_brs = msg.bitrate_switch or self._fd

        frame_flags = 0
        if use_fd:
            frame_flags |= FRM_FDF
        if use_brs:
            frame_flags |= FRM_BRS

        marker = self._marker
        self._marker = (self._marker + 1) & 0xFF

        data = bytes(msg.data)
        total = ST_TX_HDR.size + len(data)
        hdr   = ST_TX_HDR.pack(total, MSG_TX_FRAME, frame_flags, can_id, marker)

        ms = round(timeout * 1000) if timeout is not None else EP_TIMEOUT_MS
        try:
            self._dev.write(EP_OUT, hdr + data, timeout=ms)
        except usb.core.USBError as exc:
            raise can.CanOperationError(f"USB write failed: {exc}") from exc

    def _recv_internal(
        self, timeout: Optional[float]
    ) -> Tuple[Optional[can.Message], bool]:
        """Read one USB packet and return a CAN frame if present.

        Non-frame packets (TX echo, firmware strings, errors) are handled
        internally and (None, False) is returned so BusABC.recv() retries.
        """
        ms = EP_TIMEOUT_MS
        if timeout is not None:
            ms = min(round(timeout * 1000), EP_TIMEOUT_MS)
            if ms <= 0:
                return None, False

        try:
            raw = bytes(self._dev.read(EP_IN, BULK_READ_SIZE, timeout=ms))
        except usb.core.USBError as exc:
            # pyusb 1.1+ exposes USBTimeoutError; older 1.0.x only raises USBError
            # with errno 10060 (Windows) or 110 (Linux). Treat both as "no data".
            if getattr(exc, 'errno', None) in (10060, 110):
                return None, False
            if not self._is_shutdown:
                logger.error("USB read error: %s", exc)
            return None, False
        except Exception as exc:
            if not self._is_shutdown:
                logger.error("USB read error: %s", exc)
            return None, False

        if len(raw) < 2:
            return None, False

        msg_type = raw[1]

        if msg_type == MSG_RX_FRAME:
            if len(raw) < ST_RX_HDR.size:
                return None, False
            size, _, flags, can_id = ST_RX_HDR.unpack(raw[:ST_RX_HDR.size])
            data   = raw[ST_RX_HDR.size : size]
            is_ext = bool(can_id & CAN_FLAG_EXT)
            is_rtr = bool(can_id & CAN_FLAG_RTR)
            arb_id = can_id & (CAN_MASK_29 if is_ext else CAN_MASK_11)
            return can.Message(
                timestamp             = _time.time(),
                arbitration_id        = arb_id,
                is_extended_id        = is_ext,
                is_remote_frame       = is_rtr,
                is_fd                 = bool(flags & FRM_FDF),
                bitrate_switch        = bool(flags & FRM_BRS),
                error_state_indicator = bool(flags & FRM_ESI),
                data                  = data,
                is_rx                 = True,
            ), False

        elif msg_type == MSG_STRING:
            if len(raw) > 2:
                logger.info("[FW] %s", raw[2:].decode('ascii', errors='replace').rstrip())

        elif msg_type == MSG_ERROR:
            if len(raw) >= 6:
                err_id = struct.unpack_from('<I', raw, 2)[0]
                # err_id 0x40 = CAN_ERR_ACK (no one ACKing TX), 0x00 = bus idle.
                # Both are spammed continuously when only the canable is on the
                # bus -- log once per distinct code, not per frame.
                if err_id != getattr(self, '_last_err_id', None):
                    logger.warning("CAN error frame: err_id=0x%04X", err_id)
                    self._last_err_id = err_id

        return None, False

    def shutdown(self) -> None:
        if self._is_shutdown:
            return
        if self._dev is not None:
            try:
                self._ctrl_out(REQ_SET_MODE, ST_MODE.pack(MODE_RESET, 0))
            except Exception:
                pass
            try:
                usb.util.release_interface(self._dev, IFACE_CANDLE)
                usb.util.dispose_resources(self._dev)
            except Exception:
                pass
            self._dev = None
        super().shutdown()   # sets _is_shutdown = True

    # ── Internal helpers ──────────────────────────────────────────────────────

    def _ctrl_out(self, request: int, data: bytes) -> None:
        self._dev.ctrl_transfer(RT_OUT, request,
                                wValue=0, wIndex=IFACE_CANDLE,
                                data_or_wLength=data,
                                timeout=EP_TIMEOUT_MS)
        fb = self._get_feedback()
        if fb != FBK_SUCCESS:
            raise can.CanInitializationError(
                f"Firmware rejected request {request}: feedback=0x{fb:02X}"
            )

    def _ctrl_in(self, request: int, length: int) -> bytes:
        return bytes(self._dev.ctrl_transfer(RT_IN, request,
                                             wValue=0, wIndex=IFACE_CANDLE,
                                             data_or_wLength=length,
                                             timeout=EP_TIMEOUT_MS))

    def _get_feedback(self) -> int:
        raw = self._dev.ctrl_transfer(RT_IN, REQ_GET_LAST_ERROR,
                                      wValue=0, wIndex=IFACE_CANDLE,
                                      data_or_wLength=1,
                                      timeout=EP_TIMEOUT_MS)
        return raw[0]

    def _apply_bitrate(self, fclk: int, bitrate: int, fd_data: bool,
                       sample_point: float) -> None:
        brp, seg1, seg2, sjw = _calc_bit_timing(fclk, bitrate, sample_point)
        actual_baud = fclk // (brp * (1 + seg1 + seg2))
        actual_sp   = (1 + seg1) / (1 + seg1 + seg2)
        tag = "FD data" if fd_data else "nominal"
        logger.info("  %s: %.3f Mbps  SP=%.1f%%  brp=%d seg1=%d seg2=%d sjw=%d",
                    tag, actual_baud / 1e6, actual_sp * 100, brp, seg1, seg2, sjw)
        data = ST_BITTIMING.pack(0, seg1, seg2, sjw, brp)
        req  = REQ_SET_BITTIMING_FD if fd_data else REQ_SET_BITTIMING
        self._ctrl_out(req, data)
