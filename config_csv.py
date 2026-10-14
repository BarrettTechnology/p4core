"""config_csv.py — P4 configuration CSVs: load onto a puck, build from one.

The format is the CANopen File Player's (docs/CANopenFilePlayerManual.pdf):
six columns, ``comment, command, index, subindex, datatype, data``.  A row
with an index and subindex is an SDO download; a row with a command is one
of COMMAND below; ``#`` starts a comment line.  ``$ID`` in the data column
is replaced by the node ID, and the data may be an expression
(``0x200 | $ID``).

Replaces canopen_runner.py (all three apps had a copy) and pucktuner's
pandas-based configtest.build_csv:

  parse(lines)                 rows, pure
  validate(rows, node_id, od)  (errors, warnings), pure
  apply(node, rows, ...)       download to a node; returns an ApplyResult
  load(node, path, ...)        parse + validate + apply
  build_from_template(node, template, out)
                               read the puck's values into a copy of a template
  template_for(node)           the shipped blank template for the puck's model
  export_node(node, output)    build_from_template with that template
  set_entry(path, index, sub, datatype, value)
                               change one value in a CSV file in place
  write_table / read_table     the apps' two-column (name, value) exports:
                               gains, PMSM parameters, limits
  remap_pdo_cob_ids(node, new_id, old_id)
                               the PDO part of a node-ID change
"""

import ast
import csv
import datetime
import operator as op
import struct
import time

from canopen.sdo.exceptions import SdoAbortedError, SdoCommunicationError

from .reporter import Reporter


COMMANDS = (
    'FILE_INFO', 'FILE_VERSION', 'SETTING_NODE_ID', 'SETTING_NODE_ID_ADD',
    'SETTING_B2B_TIMEOUT', 'SETTING_RETRIES', 'SETTING_SDO_TIMEOUT',
    'CONTROL_PAUSE', 'CONTROL_WAIT_FOR', 'CONTROL_NMT', 'CONTROL_LSSM',
    'CONTROL_SDO_READ', 'CONTROL_SDO_BUFFER')

#: The datatype each command must carry.
COMMAND_TYPES = {
    'FILE_INFO': 'STRING', 'FILE_VERSION': 'STRING',
    'SETTING_B2B_TIMEOUT': 'UNSIGNED16', 'SETTING_NODE_ID': 'UNSIGNED8',
    'SETTING_NODE_ID_ADD': 'INTEGER8', 'SETTING_RETRIES': 'UNSIGNED8',
    'SETTING_SDO_TIMEOUT': 'UNSIGNED16', 'CONTROL_PAUSE': 'UNSIGNED16',
    'CONTROL_WAIT_FOR': 'UNSIGNED8', 'CONTROL_SDO_READ': 'UNSIGNED8',
    'CONTROL_NMT': 'UNSIGNED16', 'CONTROL_LSSM': 'LSSMASTERRECORD',
}

#: Accepted, but not carried out (as in canopen_runner).  CONTROL_NMT in
#: particular is logged and skipped: the templates' "Reset node" row would
#: otherwise reboot the puck halfway through its own configuration.
UNIMPLEMENTED_COMMANDS = ('CONTROL_WAIT_FOR', 'CONTROL_LSSM',
                          'CONTROL_SDO_READ', 'CONTROL_SDO_BUFFER')

DATATYPES = (
    'UNSIGNED8', 'INTEGER8', 'INTEGER16', 'INTEGER32', 'UNSIGNED16',
    'UNSIGNED24', 'UNSIGNED32', 'UNSIGNED40', 'UNSIGNED48', 'UNSIGNED56',
    'UNSIGNED64', 'REAL32', 'STRING', 'DOMAIN', 'LSSMASTERRECORD')

#: Error and warning codes, with canopen_runner's numbers.
MESSAGES = {
    1: 'Invalid line format - invalid number of tokens (6 expected)',
    2: 'No command specified and either index or subindex is empty',
    3: 'Specified command not recognized',
    4: 'Command specified and either index or subindex is nonempty',
    5: 'Index or subindex not a valid number',
    6: 'Invalid datatype specified',
    8: 'Datatype does not match expected datatype for the given command',
    9: 'No SDO response',
    10: ('Data overflow, data not valid for given datatype, or use of $ID '
         'without specifying a desired ID'),
    11: ('Key not found, specified index/subindex pair not found in EDS '
         'object dictionary'),
    12: 'Datatype not implemented',
    13: 'Command not implemented',
}

#: The node-ID object; a download to it changes the ID under the transfer,
#: so its missing response is expected.
NODE_ID_OBJECT = (0x21B0, 0)


class Row:
    """One CSV line.  ``kind`` is 'comment', 'blank' or 'entry'."""

    __slots__ = ('line', 'kind', 'fields', 'text')

    def __init__(self, line, kind, fields=(), text=''):
        self.line = line
        self.kind = kind
        self.fields = list(fields)
        self.text = text

    @property
    def comment(self):
        return self.fields[0].strip() if self.fields else ''

    @property
    def command(self):
        return self.fields[1].strip()

    @property
    def index(self):
        return self.fields[2].strip()

    @property
    def subindex(self):
        return self.fields[3].strip()

    @property
    def datatype(self):
        return self.fields[4].strip()

    @property
    def data(self):
        return self.fields[5].strip()


def parse(lines):
    """Rows from an iterable of text lines (a file object works)."""
    rows = []
    for number, text in enumerate(lines, 1):
        text = text.rstrip('\r\n')
        if not text.strip():
            rows.append(Row(number, 'blank', text=text))
            continue
        fields = next(csv.reader([text]))
        if not fields or not ''.join(fields).strip():
            rows.append(Row(number, 'blank', text=text))
        elif fields[0].lstrip().startswith('#'):
            rows.append(Row(number, 'comment', fields, text))
        else:
            rows.append(Row(number, 'entry', fields, text))
    return rows


def read(path):
    with open(path, newline='') as f:
        return parse(f)


def file_version(rows):
    """The FILE_VERSION row's data, or None."""
    for row in rows:
        if row.kind == 'entry' and len(row.fields) == 6 \
                and row.command == 'FILE_VERSION':
            return row.data
    return None


# ---------------------------------------------------------------------------
# Values
# ---------------------------------------------------------------------------
_OPERATORS = {
    ast.Add: op.add, ast.Sub: op.sub, ast.Mult: op.mul, ast.Div: op.truediv,
    ast.Pow: op.pow, ast.USub: op.neg, ast.UAdd: op.pos, ast.BitXor: op.xor,
    ast.BitAnd: op.and_, ast.BitOr: op.or_, ast.Invert: op.inv,
    ast.LShift: op.lshift, ast.RShift: op.rshift, ast.Eq: op.eq,
    ast.Lt: op.lt, ast.Gt: op.gt, ast.LtE: op.le, ast.GtE: op.ge,
    ast.NotEq: op.ne}


def _eval(node):
    """Evaluate a numeric expression tree: constants and operators only."""
    if isinstance(node, ast.Constant) and isinstance(node.value, (int, float)):
        return node.value
    if isinstance(node, ast.BinOp):
        return _OPERATORS[type(node.op)](_eval(node.left), _eval(node.right))
    if isinstance(node, ast.UnaryOp):
        return _OPERATORS[type(node.op)](_eval(node.operand))
    if isinstance(node, ast.Compare):
        return 1 if _OPERATORS[type(node.ops[0])](
            _eval(node.left), _eval(node.comparators[0])) else 0
    raise TypeError('not a numeric expression')


def evaluate(expr, node_id=None):
    """The value of a data-column expression, with $ID substituted."""
    if '$ID' in expr:
        if node_id is None:
            raise ValueError('$ID used but no node ID given')
        expr = expr.replace('$ID', str(node_id))
    return _eval(ast.parse(expr, mode='eval').body)


def is_integer_type(datatype):
    return datatype.startswith('UNSIGNED') or datatype.startswith('INTEGER')


def encode(datatype, data, node_id=None):
    """(bytes or str, value) for a data column, or raise ValueError."""
    if not (is_integer_type(datatype) or datatype == 'REAL32'):
        if '$ID' in data:
            if node_id is None:
                raise ValueError('$ID used but no node ID given')
            data = data.replace('$ID', str(node_id))
        return data, None
    try:
        value = evaluate(data, node_id)
    except (SyntaxError, TypeError, KeyError, ZeroDivisionError) as exc:
        raise ValueError('bad value {!r}: {}'.format(data, exc)) from exc
    if datatype == 'REAL32':
        return struct.pack('<f', float(value)), value
    if isinstance(value, float):
        if not value.is_integer():
            raise ValueError('{} is not an integer'.format(data))
        value = int(value)
    signed = datatype.startswith('INTEGER')
    width = int(datatype[len('INTEGER' if signed else 'UNSIGNED'):]) // 8
    try:
        return value.to_bytes(width, 'little', signed=signed), value
    except OverflowError as exc:
        raise ValueError('{} does not fit {}'.format(value, datatype)) from exc


def decode(datatype, raw):
    """A value uploaded from the puck, as a Python number."""
    if datatype == 'REAL32':
        return struct.unpack('<f', raw[:4])[0]
    return int.from_bytes(raw, 'little', signed=datatype.startswith('INTEGER'))


def _int(text):
    try:
        return int(text, 0)
    except (TypeError, ValueError):
        return None


# ---------------------------------------------------------------------------
# Validate
# ---------------------------------------------------------------------------
def validate(rows, node_id=None, od=None):
    """(errors, warnings): lists of (line, code) — MESSAGES has the text.

    *od* (a canopen ObjectDictionary) turns an index missing from it into a
    warning, as canopen_runner did.
    """
    errors, warnings = [], []
    for row in rows:
        if row.kind != 'entry':
            continue
        if len(row.fields) != 6:
            errors.append((row.line, 1))
            continue
        command, index, sub = row.command, row.index, row.subindex
        datatype = row.datatype
        if command == '':
            if index == '' or sub == '':
                errors.append((row.line, 2))
                continue
            a, b = _int(index), _int(sub)
            if a is None or b is None:
                errors.append((row.line, 5))
                continue
            if od is not None:
                try:
                    obj = od[a]
                    if b:
                        obj = obj[b]
                except KeyError:
                    warnings.append((row.line, 11))
        else:
            if index != '' or sub != '':
                errors.append((row.line, 4))
                continue
            if command not in COMMANDS:
                errors.append((row.line, 3))
                continue
        if datatype not in DATATYPES:
            errors.append((row.line, 6))
            continue
        want = COMMAND_TYPES.get(command)
        if want and datatype != want:
            errors.append((row.line, 8))
            continue
        try:
            encode(datatype, row.data, node_id)
        except ValueError:
            errors.append((row.line, 10))
            continue
        if datatype in ('LSSMASTERRECORD', 'DOMAIN'):
            warnings.append((row.line, 12))
        if command in UNIMPLEMENTED_COMMANDS:
            warnings.append((row.line, 13))
    return errors, warnings


def describe(issues):
    """'line N: message' strings for validate()'s lists."""
    return ['line {}: {}'.format(line, MESSAGES.get(code, 'Unknown error'))
            for line, code in issues]


# ---------------------------------------------------------------------------
# Apply
# ---------------------------------------------------------------------------
class ApplyResult:
    def __init__(self):
        self.written = 0
        self.errors = []        # 'line N: ...' strings

    @property
    def ok(self):
        return not self.errors

    def __repr__(self):
        return '<ApplyResult written={} errors={}>'.format(
            self.written, len(self.errors))


def apply(node, rows, node_id=None, reporter=None):
    """Download every entry row to *node*.  Returns an ApplyResult.

    Rows should have been validated; one that does not encode is counted as
    an error and skipped.  SETTING_* rows change the SDO pacing and the ID
    used for $ID, as in the File Player.  The SDO timeout and retries are
    put back afterwards.
    """
    reporter = reporter or Reporter()
    result = ApplyResult()
    if node_id is None:
        node_id = node.id
    b2b_s = 0.001
    saved = (node.sdo.RESPONSE_TIMEOUT, getattr(node.sdo, 'MAX_RETRIES', None))
    entries = [r for r in rows if r.kind == 'entry']
    try:
        for n, row in enumerate(entries):
            reporter.progress(100 * n // max(len(entries), 1))
            reporter.check_abort()
            where = 'line {}'.format(row.line)
            try:
                data, value = encode(row.datatype, row.data, node_id)
            except (ValueError, IndexError) as exc:
                result.errors.append('{}: {}'.format(where, exc))
                continue
            command = row.command
            if command == 'FILE_INFO':
                reporter.note('File information: {}'.format(data))
            elif command == 'FILE_VERSION':
                reporter.note('File version: {}'.format(data))
            elif command == 'SETTING_NODE_ID':
                node_id = value
            elif command == 'SETTING_NODE_ID_ADD':
                node_id += value
            elif command == 'SETTING_B2B_TIMEOUT':
                b2b_s = value / 1000.0
            elif command == 'SETTING_SDO_TIMEOUT':
                node.sdo.RESPONSE_TIMEOUT = value / 1000.0
            elif command == 'SETTING_RETRIES':
                # MAX_RETRIES is total attempts, the setting is retries.
                node.sdo.MAX_RETRIES = value + 1
            elif command == 'CONTROL_PAUSE':
                reporter.sleep(value / 1000.0)
            elif command == 'CONTROL_NMT':
                reporter.note('{}: NMT 0x{:04X} skipped ({})'.format(
                    where, value, row.comment))
            elif command in UNIMPLEMENTED_COMMANDS:
                pass
            else:
                index, sub = int(row.index, 0), int(row.subindex, 0)
                try:
                    node.sdo.download(index, sub, data)
                    result.written += 1
                    if b2b_s:
                        time.sleep(b2b_s)
                except SdoCommunicationError:
                    if (index, sub) == NODE_ID_OBJECT:
                        continue    # the ID changed under the transfer
                    result.errors.append('{}: no SDO response (0x{:04X},{})'
                                         .format(where, index, sub))
                except SdoAbortedError as exc:
                    result.errors.append(
                        '{}: SDO abort 0x{:08X} on 0x{:04X},{} ({})'.format(
                            where, exc.code, index, sub, row.comment))
        reporter.progress(100)
    finally:
        node.sdo.RESPONSE_TIMEOUT = saved[0]
        if saved[1] is not None:
            node.sdo.MAX_RETRIES = saved[1]
    for error in result.errors:
        reporter.warn(error)
    return result


class ConfigError(ValueError):
    """The CSV did not validate."""

    def __init__(self, errors):
        super().__init__('{} invalid line(s):\n  {}'.format(
            len(errors), '\n  '.join(describe(errors))))
        self.errors = errors


def load(node, path, node_id=None, reporter=None, force=False):
    """Validate the CSV at *path* against *node*'s dictionary and apply it.

    Raises ConfigError on invalid lines unless *force*.  Returns an
    ApplyResult.  Saving to NVM and rebooting are the caller's (the Session
    has save_all() and reset_node()).
    """
    reporter = reporter or Reporter()
    rows = read(path)
    if node_id is None:
        node_id = node.id
    errors, warnings = validate(rows, node_id, node.object_dictionary)
    for text in describe(warnings):
        reporter.note('Warning, ' + text)
    if errors and not force:
        raise ConfigError(errors)
    for text in describe(errors):
        reporter.warn('Skipping invalid ' + text)
    return apply(node, rows, node_id, reporter)


# ---------------------------------------------------------------------------
# Build a CSV from a puck
# ---------------------------------------------------------------------------
def _format(index, datatype, value):
    """The data column for a value read from the puck, in the templates'
    style: 32-bit words in the manufacturer (0x2xxx) range and INTEGER32 in
    the profile range as hex, everything else decimal; zero as 0."""
    if datatype == 'REAL32':
        return repr(float(value))
    if value == 0:
        return '0'
    if datatype in ('INTEGER32', 'UNSIGNED32') and (
            index.startswith('0x2') or
            (index.startswith('0x6') and datatype == 'INTEGER32')):
        return '-0x{:x}'.format(-value) if value < 0 else '0x{:x}'.format(value)
    return str(value)


def build_from_template(node, template, output, reporter=None, date=None):
    """Write *output*: *template* with the puck's current values.

    Communication objects (0x1xxx) keep the template's text — they carry the
    ``$ID`` expressions that make a CSV portable between node IDs.  Every
    other object is read from *node* and decoded by its datatype (signed and
    float values come back as such; configtest read everything unsigned, so
    a negative INTEGER32 came out as 0xffffffff, which the loader rejects).
    The Version row is stamped with *date*.  Returns the number of values
    read.
    """
    reporter = reporter or Reporter()
    date = date or datetime.date.today().isoformat()
    with open(template, newline='') as f:
        lines = f.read().splitlines()
    rows = parse(lines)
    out_lines = []
    count = 0
    entries = sum(1 for r in rows if r.kind == 'entry')
    for row in rows:
        if row.kind == 'comment' and 'This is a CANopen' in row.text:
            out_lines.append('# This is a CANopen config template for - '
                             + output)
            continue
        if row.kind != 'entry' or len(row.fields) != 6:
            out_lines.append(row.text)
            continue
        fields = list(row.fields)
        if row.comment.startswith('Version') and row.command == 'FILE_VERSION':
            fields[5] = 'V1.0.0 ({})'.format(date)
        elif row.index and row.subindex and \
                not row.index.lower().startswith('0x1'):
            reporter.check_abort()
            index, sub = int(row.index, 0), int(row.subindex, 0)
            raw = node.sdo.upload(index, sub)
            value = decode(row.datatype, raw)
            fields[5] = _format(row.index, row.datatype, value)
            count += 1
            reporter.progress(100 * count // max(entries, 1))
        buf = []
        csv.writer(_Collect(buf), lineterminator='').writerow(fields)
        out_lines.append(buf[0])
    with open(output, 'w', newline='') as f:
        f.write('\n'.join(out_lines) + '\n')
    reporter.progress(100)
    return count


def template_for(node):
    """The shipped blank template for *node*'s model (0x1018,2), e.g.
    P4-42-Blank.csv.  Raises ValueError when the model is unknown or has no
    template."""
    from . import can_backend, paths
    code = node.sdo.upload(0x1018, 2)
    model = can_backend.model_from_product_code(int.from_bytes(code[:4],
                                                               'little'))
    name = '{}-Blank.csv'.format(model) if model else None
    if name is None or name not in paths.templates():
        raise ValueError('no config template for product code 0x{:08x} '
                         '(model {}); shipped: {}'.format(
                             int.from_bytes(code[:4], 'little'), model,
                             ', '.join(paths.templates())))
    return paths.template(name)


def export_node(node, output, template=None, reporter=None, date=None):
    """Write *output*: *template* (default: template_for(node)) filled with
    the puck's current values.  Returns (template path, values read)."""
    template = template or template_for(node)
    return template, build_from_template(node, template, output, reporter,
                                         date)


def write_table(path, header, rows, columns=('Parameter', 'Value')):
    """A two-column export: a ``# header`` line, the column names, then
    (name, value) rows; None is written as an empty cell."""
    with open(path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow([header if header.startswith('#')
                         else '# ' + header])
        writer.writerow(list(columns))
        for name, value in rows:
            writer.writerow([name, '' if value is None else value])


def read_table(path, names=None, convert=float):
    """{name: convert(value)} from a write_table file, in file order.  With
    *names*, other rows are ignored; empty values are skipped."""
    out = {}
    with open(path, newline='') as f:
        for row in csv.reader(f):
            if len(row) < 2 or not row[0] or row[0].startswith('#'):
                continue
            if names is not None and row[0] not in names:
                continue
            if not row[1].strip():
                continue
            try:
                out[row[0]] = convert(row[1])
            except ValueError:
                continue
    return out


def set_entry(path, index, sub, datatype, value, comment=''):
    """Set the data of the (*index*, *sub*) row in the CSV at *path*, in place.

    Every matching row is updated; if there is none, one is appended.  The
    file is otherwise left exactly as it was.  Returns True if a row was
    updated, False if one was appended.
    """
    with open(path, newline='') as f:
        lines = f.read().splitlines()
    found = False
    for n, row in enumerate(parse(lines)):
        if row.kind != 'entry' or len(row.fields) != 6 or row.command:
            continue
        if _int(row.index) == index and _int(row.subindex) == sub:
            fields = list(row.fields)
            fields[5] = str(value)
            buf = []
            csv.writer(_Collect(buf), lineterminator='').writerow(fields)
            lines[n] = buf[0]
            found = True
    if not found:
        buf = []
        csv.writer(_Collect(buf), lineterminator='').writerow(
            [comment, '', '0x{:04X}'.format(index), str(sub), datatype, str(value)])
        lines.append(buf[0])
    with open(path, 'w', newline='') as f:
        f.write('\n'.join(lines) + '\n')
    return found


class _Collect:
    def __init__(self, buf):
        self.buf = buf

    def write(self, text):
        self.buf.append(text)


# ---------------------------------------------------------------------------
# PDO COB-IDs after a node-ID change
# ---------------------------------------------------------------------------
#: The predefined connection set — the same mapping the config CSV templates
#: as ``<base> | $ID`` (RPDO1-4 @ 0x1400-0x1403:1, TPDO1-4 @ 0x1800-0x1803:1).
PDO_COB_BASES = {
    0x1400: 0x200, 0x1401: 0x300, 0x1402: 0x400, 0x1403: 0x500,   # RPDO1-4
    0x1800: 0x180, 0x1801: 0x280, 0x1802: 0x380, 0x1803: 0x480,   # TPDO1-4
}
_PDO_VALID_BIT = 0x80000000    # bit31: 1 = PDO invalid/disabled
_SAVE_SIGNATURE = 0x65766173   # "save" (CiA-301 0x1010:1)


def remap_pdo_cob_ids(node, new_id, old_id=None, save=True, verify=True,
                      log=print):
    """Rewrite a node's 8 PDO comm-param COB-IDs to match `new_id` -- the standalone
    equivalent of the config CSV's `<base> | $ID` templating, for use right after a
    Set-ID change (this does NOT change the node ID itself).

    Grounded in scripts/pdo_persistence_probe.py findings on P4 firmware:
      * A bare Set-ID (NetCfg + reset) leaves all 8 COB-IDs at `base | old_id` -- the
        firmware does NOT re-derive them from the node id -- so this remap is required.
      * sub1 is writable DIRECTLY while the PDO is valid (no CiA-301 disable/bit31 dance
        needed); a disable->write->enable fallback is kept in case a verify misses on
        other firmware/PDO states.
      * A plain write is RAM-only and reverts on reboot, so an explicit Save
        (0x1010:1 = "save") IS REQUIRED to commit to NV -- hence save=True default.

    Reads each entry and preserves its high bits, so it self-corrects and tolerates any
    PDO layout. Only remaps PDOs currently keyed to `old_id` (when given), leaving any
    deliberately cross-linked PDO untouched.

    Returns (ok, results) where results is a list of (index, target_or_None, readback).
    """
    def _readback(index, base):
        rb = int.from_bytes(node.sdo.upload(index, 1), 'little', signed=False)
        return rb, (rb & 0x7FF) == (base | (new_id & 0x7F))

    results = []
    ok = True
    for index, base in sorted(PDO_COB_BASES.items()):
        try:
            cur = int.from_bytes(node.sdo.upload(index, 1), 'little', signed=False)
        except Exception as e:
            log("  remap {:#06x}: read failed ({})".format(index, e))
            ok = False; results.append((index, None, None)); continue
        if old_id is not None and (cur & 0x7F) != (old_id & 0x7F):
            log("  remap {:#06x}: node field 0x{:02X} != old id 0x{:02X}; leaving alone"
                .format(index, cur & 0x7F, old_id & 0x7F))
            results.append((index, None, cur)); continue
        target = (cur & ~0x7FF) | base | (new_id & 0x7F)   # keep valid/RTR bits, set func+node
        rb = None
        try:
            node.sdo.download(index, 1, target.to_bytes(4, 'little'))   # direct write (confirmed OK)
            if verify:
                rb, good = _readback(index, base)
                if not good:
                    log("  remap {:#06x}: direct verify miss (0x{:03X}); disable->write->enable"
                        .format(index, rb & 0x7FF))
                    node.sdo.download(index, 1, (cur | _PDO_VALID_BIT).to_bytes(4, 'little'))
                    node.sdo.download(index, 1, (target | _PDO_VALID_BIT).to_bytes(4, 'little'))
                    node.sdo.download(index, 1, (target & ~_PDO_VALID_BIT).to_bytes(4, 'little'))
                    rb, good = _readback(index, base)
                    if not good:
                        log("  remap {:#06x}: VERIFY MISMATCH wrote cob 0x{:03X} read 0x{:03X}"
                            .format(index, base | (new_id & 0x7F), rb & 0x7FF))
                        ok = False
        except Exception as e:
            log("  remap {:#06x}: write failed ({})".format(index, e))
            ok = False; results.append((index, target, rb)); continue
        results.append((index, target, rb))

    if save:
        try:
            node.sdo.download(0x1010, 1, _SAVE_SIGNATURE.to_bytes(4, 'little'))   # commit RAM -> NV
        except Exception as e:
            log("  remap save (0x1010:1) FAILED ({}) -- COB-IDs are in RAM only and will "
                "revert on reboot".format(e))
            ok = False
    return ok, results
