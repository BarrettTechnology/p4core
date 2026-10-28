"""Decode a PDO frame by the mapping that was written for it.

The firmware builds a PDO from its mapping records, each entry the size of its
object.  Before stm32 fc554c0 it ignored the record's length field: a 16-bit
object mapped as 32 bits was accepted and the frame came out 2 bytes short,
every entry after it at the wrong offset.  A decoder that took the first 8
bytes of anything 8 or longer, with a fixed struct format, then read garbage
from a 6-byte frame and silently dropped it, or read the wrong fields from a
frame of the length it expected.

A ``PdoLayout`` is built from the mapping words (bit lengths) and each entry's
kind, so the format is the mapping's.  ``decode`` takes only a frame of exactly
that length; anything else is counted, and the owner reports it from its own
thread (``problem()``) -- the decode runs in the CAN notifier thread.
"""

import struct

#: (bits, kind) -> struct char.  kind: 'u' unsigned, 's' signed, 'f' float.
_CHARS = {(8, 'u'): 'B', (8, 's'): 'b', (16, 'u'): 'H', (16, 's'): 'h',
          (32, 'u'): 'I', (32, 's'): 'i', (32, 'f'): 'f'}

#: CiA 301 data type codes (canopen.objectdictionary) -> kind
_DATA_TYPE_KIND = {0x01: 'u', 0x02: 's', 0x03: 's', 0x04: 's', 0x05: 'u',
                   0x06: 'u', 0x07: 'u', 0x08: 'f'}


class PdoLayout:
    """The byte layout of one PDO: ``fields`` is ``[(bits, kind), ...]`` in
    mapping order."""

    def __init__(self, fields, name='PDO'):
        try:
            fmt = '<' + ''.join(_CHARS[(int(b), k)] for b, k in fields)
        except KeyError as e:
            raise ValueError('{}: no decode for a {} entry'.format(name, e))
        self._struct = struct.Struct(fmt)
        self.size = self._struct.size
        self.format = fmt
        self.name = name
        self.frames = 0
        self.bad = 0
        self.bad_len = None

    @classmethod
    def from_words(cls, words, kinds, name='PDO'):
        """From mapping words (index << 16 | sub << 8 | bits) and one kind per
        word."""
        words = list(words)
        if len(kinds) != len(words):
            raise ValueError('{}: {} kinds for {} entries'
                             .format(name, len(kinds), len(words)))
        return cls([(w & 0xFF, k) for w, k in zip(words, kinds)], name)

    @classmethod
    def from_pdo_map(cls, pdo_map, name='PDO'):
        """From a python-canopen PdoMap as read back from the node: each
        variable's length and its OD data type."""
        fields = []
        for v in pdo_map.map:
            if not v.length:
                continue
            kind = _DATA_TYPE_KIND.get(getattr(v.od, 'data_type', None))
            if kind is None:
                raise ValueError('{}: 0x{:04X},{} is not a number'
                                 .format(name, v.index, v.subindex))
            fields.append((v.length, kind))
        return cls(fields, name)

    def decode(self, data):
        """The entries of a frame of exactly ``size`` bytes, else None (and
        the frame is counted as bad)."""
        if len(data) != self.size:
            self.bad += 1
            self.bad_len = len(data)
            return None
        self.frames += 1
        return self._struct.unpack(bytes(data))

    def problem(self):
        """A one-line report of the frames refused, or None."""
        if not self.bad:
            return None
        return ('{}: {} frame(s) of {} bytes refused, the mapping is {} bytes '
                '-- a mapping record whose length is not its object\'s? '
                '(firmware before stm32 fc554c0 accepts one)'
                .format(self.name, self.bad, self.bad_len, self.size))
