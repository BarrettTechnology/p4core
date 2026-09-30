"""A fake P4 node for p4core's tests: named and raw SDO access over the
real puck4.eds, with hooks so a test can make reads depend on writes."""

import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__),
                                                '..', '..')))

import canopen                                                  # noqa: E402
from canopen.sdo.exceptions import SdoAbortedError              # noqa: E402

from p4core import paths                                        # noqa: E402

_OD = None


def od():
    global _OD
    if _OD is None:
        _OD = canopen.import_od(paths.puck4_eds())
    return _OD


class Var:
    def __init__(self, sdo, entry):
        self._sdo = sdo
        self.entry = entry

    @property
    def key(self):
        return (self.entry.index, self.entry.subindex)

    @property
    def raw(self):
        return self._sdo.read(self.key)

    @raw.setter
    def raw(self, value):
        self._sdo.write(self.key, value)

    def __getitem__(self, sub):
        return Var(self._sdo, self.entry[sub])


class FakeSdo:
    RESPONSE_TIMEOUT = 0.3
    MAX_RETRIES = 1

    def __init__(self, node):
        self.node = node

    def __getitem__(self, name):
        return Var(self, self.node.object_dictionary[name])

    def read(self, key):
        self.node.reads.append(key)
        hook = self.node.read_hooks.get(key)
        if hook:
            return hook()
        if key not in self.node.values:
            raise SdoAbortedError(0x06020000)
        return self.node.values[key]

    def write(self, key, value):
        self.node.writes.append((key, value))
        if key in self.node.readonly:
            raise SdoAbortedError(0x06010002)
        self.node.values[key] = value
        hook = self.node.write_hooks.get(key)
        if hook:
            hook(value)

    def _entry(self, index, sub):
        entry = self.node.object_dictionary[index]
        if not isinstance(entry, canopen.objectdictionary.ODVariable):
            entry = entry[sub]
        return entry

    def upload(self, index, sub):
        entry = self._entry(index, sub)
        value = self.read((index, sub))
        if isinstance(value, (bytes, bytearray)):
            return bytes(value)
        return entry.encode_raw(value)

    def download(self, index, sub, data):
        self.node.downloads.append((index, sub, bytes(data) if not isinstance(data, str) else data))
        try:
            entry = self._entry(index, sub)
            value = entry.decode_raw(data) if not isinstance(data, str) else data
        except KeyError:
            value = data
        self.write((index, sub), value)


class FakeNode:
    def __init__(self, node_id=5, values=None):
        self.id = node_id
        self.object_dictionary = od()
        self.values = dict(values or {})
        self.readonly = set()
        self.read_hooks = {}
        self.write_hooks = {}
        self.reads, self.writes, self.downloads = [], [], []
        self.sdo = FakeSdo(self)

    def key(self, name, sub=None):
        entry = self.object_dictionary[name]
        if sub is not None:
            entry = entry[sub]
        return (entry.index, entry.subindex)

    def set(self, name, sub, value):
        self.values[self.key(name, sub)] = value

    def get(self, name, sub=None):
        return self.values.get(self.key(name, sub))

    def hook_read(self, name, sub, fn):
        self.read_hooks[self.key(name, sub)] = fn

    def hook_write(self, name, sub, fn):
        self.write_hooks[self.key(name, sub)] = fn
