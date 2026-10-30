"""Read a full logger capture (0x3004) back over TPDO4 instead of by SDO.

The logger's channels are TPDO4's mapping, and writing 0x3004,1 = 3 plays
the records back in TPDO4's frames, one record per frame, in the same layout
as a live frame (stm32 app/log.c logPlaybackFrame).  An SDO read of 0x3004,5
returns one channel of one record, a round trip each: about 0.65 ms on an
STM32 and 2 ms on a K64 at 100 kHz PWM (node 5, 2026-10-30), so 512
two-channel records took 2.0 s there.  Played back on TPDO4's event timer
(1 ms, inhibit 0) they took 0.52 s; firmware that sends several playback
frames per tick takes less.

The PDO is driven by its event timer, not by SYNC: a host SYNC burst would
also reach clockcal (app/clockcal.c), which learns the producer's period from
SYNC timestamps.  Playback needs the node OPERATIONAL (PDOs flow only there);
the caller's own 1 kHz SYNC, if it runs one, is harmless -- a SYNC-type TPDO4
is switched to event-driven for the readout.

read() returns None, and leaves the capture to be read by SDO, when it cannot
play the capture back: a virtual network (fake_puck, the twin), a record that
does not fit a classic frame, or frames that stop coming.
"""

import struct
import time

TPDO4_COMM, TPDO4_MAP = 0x1803, 0x1A03
OD_LOG_STATE = (0x3004, 1)
OD_LOG_CURSOR = (0x3004, 4)
LOG_IDLE, LOG_PLAYBACK = 0, 3
#: Event timer (ms) while playing back: firmware sends at least one frame
#: per tick.
EVENT_MS = 1
#: Give up when no frame has come for this long (the first one gets more:
#: the arm of the playback is an SDO round trip).
STALL_S = 0.1
FIRST_S = 0.5
#: Largest record played back: a classic CAN frame.
MAX_RECORD_BYTES = 8


def _w(node, idx, sub, fmt, value):
    node.sdo.download(idx, sub, struct.pack('<' + fmt, value))


def _r(node, idx, sub, fmt):
    d = bytes(node.sdo.upload(idx, sub))
    n = struct.calcsize(fmt)
    return struct.unpack('<' + fmt, d[:n].ljust(n, b'\0'))[0]


def _signed(value, bits):
    return value - (1 << bits) if value & (1 << (bits - 1)) else value


def available(node, words):
    """True when *node*'s network can carry a playback of these channels."""
    net = getattr(node, 'network', None)
    if net is None or getattr(net, 'advance', None) is not None:
        return False            # a virtual network: fake_puck, the twin
    if not hasattr(net, 'subscribe'):
        return False
    return sum((w & 0xFF) // 8 for w in words) <= MAX_RECORD_BYTES


def decode(frame, words):
    """One record's channels from a played-back frame, each sign-extended
    from its own width -- what an SDO read of 0x3004,5 gives for a signed
    channel, the only kind the captures map."""
    out, off = [], 0
    for w in words:
        n = (w & 0xFF) // 8
        out.append(_signed(int.from_bytes(frame[off:off + n], 'little'), 8 * n))
        off += n
    return out


def read(node, words, count, log=None, should_abort=None):
    """Play *count* records of a full capture back over TPDO4 and return them
    as lists of channel values (decode), or None to fall back to SDO.

    TPDO4 must already be mapped to *words* and disabled (the capture's own
    channel map, held until the readout is done); it is left that way.
    """
    if not available(node, words):
        return None
    rec_bytes = sum((w & 0xFF) // 8 for w in words)
    net = node.network
    frames = []

    def on_frame(can_id, data, timestamp):
        if len(data) == rec_bytes:      # anything else is not a record
            frames.append(bytes(data))

    cob_id = _r(node, TPDO4_COMM, 1, 'I')
    cob = cob_id & 0x7FF
    saved = (_r(node, TPDO4_COMM, 2, 'B'), _r(node, TPDO4_COMM, 3, 'H'),
             _r(node, TPDO4_COMM, 5, 'H'))
    nmt = getattr(node, 'nmt', None)
    was = getattr(nmt, 'state', None)
    playing = False
    try:
        _w(node, TPDO4_COMM, 1, 'I', cob | 0x80000000)
        _w(node, TPDO4_COMM, 2, 'B', 254)
        _w(node, TPDO4_COMM, 3, 'H', 0)
        _w(node, TPDO4_COMM, 5, 'H', EVENT_MS)
        if nmt is not None and was != 'OPERATIONAL':
            nmt.state = 'OPERATIONAL'
        net.subscribe(cob, on_frame)
        # Playback first, then the PDO: a frame sent before the playback
        # starts would carry the live values.
        _w(node, OD_LOG_CURSOR[0], OD_LOG_CURSOR[1], 'H', 0)
        _w(node, OD_LOG_STATE[0], OD_LOG_STATE[1], 'B', LOG_PLAYBACK)
        playing = True
        _w(node, TPDO4_COMM, 1, 'I', cob)
        seen, last = 0, time.monotonic()
        limit = FIRST_S
        while len(frames) < count:
            if should_abort is not None:
                should_abort()
            time.sleep(0.002)
            now = time.monotonic()
            if len(frames) != seen:
                seen, last, limit = len(frames), now, STALL_S
            elif now - last > limit:
                break
    except Exception as e:
        if log:
            log('capture playback failed ({}): reading it by SDO'.format(e))
        frames = []
    finally:
        try:
            net.unsubscribe(cob, on_frame)
        except Exception:
            pass
        try:
            _w(node, TPDO4_COMM, 1, 'I', cob | 0x80000000)
            _w(node, TPDO4_COMM, 2, 'B', saved[0])
            _w(node, TPDO4_COMM, 3, 'H', saved[1])
            _w(node, TPDO4_COMM, 5, 'H', saved[2])
            if len(frames) < count and playing:
                # Stopped part-way: back to IDLE, which keeps the records
                # for the SDO readout.
                _w(node, OD_LOG_STATE[0], OD_LOG_STATE[1], 'B', LOG_IDLE)
        except Exception:
            pass
        if nmt is not None and was == 'PRE-OPERATIONAL':
            try:
                nmt.state = 'PRE-OPERATIONAL'
            except Exception:
                pass
    if len(frames) < count:
        if log and frames:
            log('capture playback: {} of {} records came back: reading it '
                'by SDO'.format(len(frames), count))
        return None
    return [decode(f, words) for f in frames[:count]]
