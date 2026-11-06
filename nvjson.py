"""nvjson.py -- a drive's NV as a JSON file, back again, and the 4.3.x wipe.

  save(node, path)             every NV row the running firmware has, raw
  restore(node, path, reboot)  write them back, Save.All, reboot, verify
  legacy_wipe(node, reboot)    4.3.x: NV rows from 0x2000 up back to defaults

Which rows are NV comes from the firmware's co_dict.c (an EDS cannot say),
kept per firmware line in data/nv/<line>.txt, one ``IIII,S`` per line:
legacy (4.3.x, from tag v4.3.5) and current (the 5.0 line).  Regenerate with
nv_keys_from_co_dict() when co_dict.c gains NV rows; a key the drive lacks is
skipped, so one list serves every build of its line.

The JSON holds a header (firmware, githash, node, date) and ``entries``:
{"IIII,S": "<hex of the SDO upload>"}.  The flat {key: hex} files of
notes/bench/fwcompare/nvsnap.py load too.

4.3.x has no NV erase (0x1011,3/5 came with the 5.0 line), so legacy_wipe()
writes each NV row of the manufacturer and profile ranges its EDS default
(data/puck4-4.3.5.eds) and saves.  That is what the 5.0 erase does, and its
job in a firmware A/B is that no 5.0 value survives into a 4.3.x run: 5.0
keeps the current-sense bias in Q12.4, which 4.3.x reads as whole counts.
"""

import datetime
import json
import os
import re

from . import firmware, nvsave, paths

FORMAT = 'p4core-nv/1'
GITHASH = (0x3003, 0)
#: Never written back: 0x3005 is direct EEPROM address/data (a write stores a
#: raw word); NetCfg reads 0 on a node whose ID came from elsewhere, and
#: writing that 0 moves the node to ID 0; 0x1018 is identity.
RESTORE_SKIP = frozenset([(0x3005, 1), (0x3005, 2), (0x21B0, 0)] +
                         [(0x1018, s) for s in range(7)])
#: Left alone by legacy_wipe(), as the 5.0 erase leaves them: the node ID and
#: the CAN termination; and the encoder filter, because 4.3.x writes 0x3013,6
#: into the sensor's register 0x0E at every boot (a wiped 0 would be written
#: to the chip at the reboot that follows).
WIPE_KEEP = frozenset([(0x21B0, 0), (0x21B2, 0), (0x3013, 6)])
LEGACY_EDS = 'puck4-4.3.5.eds'
PDO_COMM = range(0x1400, 0x1A00)
PDO_MAP = tuple(range(0x1600, 0x1800)) + tuple(range(0x1A00, 0x1C00))
COB_INVALID = 0x80000000


class NvError(RuntimeError):
    pass


# ---------------------------------------------------------------------------
# Which rows are NV
# ---------------------------------------------------------------------------
_ROW = re.compile(r'\{\s*0x([0-9A-Fa-f]{4})\s*,\s*(\d+)\s*,\s*([^,}]+),')


def nv_keys_from_co_dict(text):
    """[(index, sub), ...] of co_dict.c's NV rows; an X16 row is 16 subs."""
    keys = []
    for m in _ROW.finditer(text):
        flags = set(re.split(r'[\s|]+', m.group(3).strip()))
        if 'NV' not in flags:
            continue
        index, sub = int(m.group(1), 16), int(m.group(2))
        keys += [(index, sub + k) for k in range(16 if 'X16' in flags else 1)]
    return keys


def write_keys(keys, path):
    with open(path, 'w') as f:
        f.writelines('{:04X},{}\n'.format(i, s) for i, s in keys)


def line_of(node):
    return 'legacy' if firmware.legacy(node) else 'current'


def nv_keys(line):
    """The NV keys of firmware *line* ('legacy' or 'current')."""
    with open(paths.data('nv', line + '.txt')) as f:
        return [parse_key(t) for t in f.read().split()]


def parse_key(text):
    index, sub = text.split(',')
    return int(index, 16), int(sub)


def format_key(index, sub):
    return '{:04X},{}'.format(index, sub)


# ---------------------------------------------------------------------------
# Version and githash
# ---------------------------------------------------------------------------
def read_githash(node):
    """'be59679' (+ '-dirty') from 0x3003, or None (4.3.x has no 0x3003)."""
    try:
        raw = int.from_bytes(node.sdo.upload(*GITHASH)[:4], 'little')
    except Exception:
        return None
    return '{:07x}{}'.format(raw & 0x0FFFFFFF,
                             '-dirty' if raw & 0x80000000 else '')


def identity(node):
    """{'firmware': '5.0.4', 'githash': 'be59679' or None}."""
    return dict(firmware=firmware.detect(node).text, githash=read_githash(node))


# ---------------------------------------------------------------------------
# Save
# ---------------------------------------------------------------------------
def snapshot(node, keys=None):
    """{key: hex} of every NV row in *keys* (default: the firmware line's)
    that the drive answers."""
    entries = {}
    for index, sub in keys or nv_keys(line_of(node)):
        try:
            entries[format_key(index, sub)] = node.sdo.upload(index, sub).hex()
        except Exception:
            pass
    return entries


def save(node, path, reporter=None, date=None):
    """Write *node*'s NV rows to *path* as JSON.  Returns the document."""
    if reporter:
        reporter.status('Reading node {}\'s NV...'.format(node.id))
    doc = dict(format=FORMAT, node_id=node.id, line=line_of(node),
               date=(date or datetime.datetime.now()).isoformat(timespec='seconds'),
               **identity(node))
    doc['entries'] = snapshot(node)
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, 'w') as f:
        json.dump(doc, f, indent=1)
        f.write('\n')
    if reporter:
        reporter.note('{} NV value(s) -> {}'.format(len(doc['entries']), path))
    return doc


def load(path):
    """The document at *path*; a flat nvsnap.py file gets an empty header."""
    with open(path) as f:
        doc = json.load(f)
    if 'entries' not in doc:
        doc = dict(format=None, firmware=None, githash=None, entries=doc)
    return doc


# ---------------------------------------------------------------------------
# Restore
# ---------------------------------------------------------------------------
def mismatch(doc, ident):
    """Why *doc* does not belong to firmware *ident*, or None."""
    if doc.get('format') is None:
        return 'the file has no firmware header'
    for field in ('firmware', 'githash'):
        if doc.get(field) != ident[field]:
            return 'the file is from {} {}, the drive runs {} {}'.format(
                doc.get('firmware'), doc.get('githash') or '',
                ident['firmware'], ident['githash'] or '').replace('  ', ' ')
    return None


def _ordered_writes(entries):
    """(key, data) in a safe order: each PDO's COB-ID invalid first, its map
    count 0, the map, everything else, the map counts, the real COB-IDs."""
    keyed = [(parse_key(k), bytes.fromhex(v)) for k, v in entries.items()]
    keyed = [(k, d) for k, d in keyed if k not in RESTORE_SKIP]
    cobs = [(k, d) for k, d in keyed if k[0] in PDO_COMM and k[0] not in PDO_MAP
            and k[1] == 1]
    counts = [(k, d) for k, d in keyed if k[0] in PDO_MAP and k[1] == 0]
    first = [(k, (int.from_bytes(d[:4], 'little') | COB_INVALID).to_bytes(4, 'little'))
             for k, d in cobs]
    first += [(k, b'\x00') for k, _ in counts]
    rest = [(k, d) for k, d in keyed if (k, d) not in cobs and (k, d) not in counts]
    return first + rest + counts + cobs


def write_entries(node, entries, reporter=None):
    """Download *entries* in _ordered_writes() order.  Returns (written,
    [(key, error), ...]): the keys written (some twice: PDO COB-IDs and map
    counts) and those refused.  Read-only NV rows (derived values the
    drive keeps, 0x1200 SDO server COB-IDs) are refused: not an error."""
    written, failed = set(), {}
    for (index, sub), data in _ordered_writes(entries):
        key = format_key(index, sub)
        try:
            node.sdo.download(index, sub, data)
            written.add(key)
        except Exception as exc:
            failed[key] = str(exc)
    return len(written), sorted(failed.items())


def _same(key, want, got):
    """A 5.0 TPDO/RPDO COB-ID reads bit 30 back set whatever was written."""
    if want == got:
        return True
    index, sub = key
    if index in PDO_COMM and index not in PDO_MAP and sub == 1 and len(want) == 4:
        mask = ~0x40000000 & 0xFFFFFFFF
        return (int.from_bytes(want, 'little') & mask) == \
            (int.from_bytes(got[:4], 'little') & mask)
    return False


def verify(node, entries):
    """[key, ...] whose value on *node* differs from *entries* (or fails)."""
    bad = []
    for text, hx in entries.items():
        key = parse_key(text)
        if key in RESTORE_SKIP:
            continue
        try:
            got = node.sdo.upload(*key)
        except Exception:
            bad.append(text)
            continue
        if not _same(key, bytes.fromhex(hx), got):
            bad.append(text)
    return bad


def _require_disabled(node):
    if int.from_bytes(node.sdo.upload(0x6041, 0)[:2], 'little') & 0x0004:
        raise NvError('node {} is enabled: disable it first'.format(node.id))


def restore(node, path, reboot, reporter=None, force=False):
    """Write *path*'s NV back to *node*, Save.All, *reboot()* (which returns
    the node) and read every value back.  Refuses a file from other firmware
    unless *force*.  Returns dict(written, failed, differ, node)."""
    doc = load(path)
    why = mismatch(doc, identity(node))
    if why and not force:
        raise NvError('{}: refusing (--force writes it anyway)'.format(why))
    if why and reporter:
        reporter.warn(why + ': writing it anyway')
    _require_disabled(node)
    if reporter:
        reporter.status('Writing {} NV value(s)...'.format(len(doc['entries'])))
    written, failed = write_entries(node, doc['entries'], reporter)
    for key, err in failed:
        if reporter:
            reporter.note('not written: {} ({})'.format(key, err))
    nvsave.save_all(node)
    if reporter:
        reporter.status('Saved; rebooting...')
    node = reboot()
    firmware.forget(node)
    differ = verify(node, doc['entries'])
    if reporter:
        for key in differ:
            reporter.warn('{} reads back different'.format(key))
        reporter.note('{} written, {} refused, {} differ after the reboot'.format(
            written, len(failed), len(differ)))
    return dict(written=written, failed=failed, differ=differ, node=node)


# ---------------------------------------------------------------------------
# 4.3.x wipe
# ---------------------------------------------------------------------------
def legacy_defaults(eds=None):
    """{(index, sub): (data bytes, default text)} for the legacy NV rows from
    0x2000 up but WIPE_KEEP, from the 4.3.5 EDS."""
    import canopen
    od = canopen.import_od(eds or paths.data(LEGACY_EDS))
    out = {}
    for index, sub in nv_keys('legacy'):
        if index < 0x2000 or (index, sub) in WIPE_KEEP:
            continue
        try:
            obj = od[index]
            var = obj if isinstance(obj, canopen.objectdictionary.ODVariable) \
                else obj[sub]
        except KeyError:
            continue
        value = var.default if var.default is not None else 0
        out[(index, sub)] = (var.encode_raw(value), value)
    return out


def legacy_wipe(node, reboot, reporter=None, eds=None):
    """4.3.x: every NV row from 0x2000 up but WIPE_KEEP to its default, then
    Save.All and *reboot()*.  Communication (0x1000-0x1FFF) stays, as with
    the 5.0 erase.  Returns dict(written, failed, differ, node): the node
    is the one *reboot()* returned.

    The drive cannot be enabled afterwards (Gainfactor 0): load the CSV and
    calibrate next."""
    if not firmware.legacy(node):
        raise NvError('legacy_wipe() is for 4.3.x; this drive runs {}'.format(
            firmware.detect(node).text))
    _require_disabled(node)
    defaults = legacy_defaults(eds)
    if reporter:
        reporter.status('Writing {} NV row(s) to their 4.3.5 defaults...'.format(
            len(defaults)))
    written, failed = 0, []
    for (index, sub), (data, _) in sorted(defaults.items()):
        try:
            node.sdo.download(index, sub, data)
            written += 1
        except Exception as exc:
            failed.append((format_key(index, sub), str(exc)))
    nvsave.save_all(node)
    if reporter:
        reporter.status('Saved; rebooting...')
    node = reboot()
    firmware.forget(node)
    differ = []
    for (index, sub), (data, _) in sorted(defaults.items()):
        try:
            if node.sdo.upload(index, sub) != data:
                differ.append(format_key(index, sub))
        except Exception:
            differ.append(format_key(index, sub))
    if reporter:
        for key, err in failed:
            reporter.note('not written: {} ({})'.format(key, err))
        for key in differ:
            reporter.note('{} is not at its default after the reboot (the '
                          'firmware corrects some at boot: PWM, poles)'.format(key))
        reporter.note('{} written, {} refused, {} not at the default after the '
                      'reboot'.format(written, len(failed), len(differ)))
        reporter.warn('Gainfactor is 0: the drive cannot be enabled until the '
                      'CSV is loaded and bias/gain are calibrated')
    return dict(written=written, failed=failed, differ=differ, node=node)
