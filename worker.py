"""worker.py — firmware and configuration downloads for a GUI's worker
process (puckutility's and pucktuner's).

A GUI runs a flash or a config load in a multiprocessing child so the
window keeps painting, and reads its progress from a queue: ints 0-100, then
"Pass" or "Fail".  These are those children, on p4core's flash and
config_csv (the CLI uses p4core.ops directly).  A config load only writes:
the GUI saves to NVM and reboots the puck itself afterwards.
"""

import os
import traceback

from . import can_backend, config_csv
from . import flash as _flash
from .reporter import Reporter


class QueueReporter(Reporter):
    """Progress as ints on *queue*; text on stdout (the GUI's log)."""

    def __init__(self, queue):
        super().__init__()
        self.queue = queue
        self._last = None

    def progress(self, pct, text=None):
        pct = int(pct)
        if pct != self._last and self.queue is not None:
            self.queue.put(pct)
            self._last = pct
        if text:
            print(text)

    def status(self, text):
        print(text)

    def note(self, text):
        print(text)

    def warn(self, text):
        print('WARNING: ' + text)


def _finish(queue, ok):
    if queue is not None:
        queue.put('Pass' if ok else 'Fail')


def is_golden(path):
    return 'golden' in os.path.basename(path).lower()


def flash_child(can_device, node_id, path, queue=None):
    """Program *path* into *node_id*; "Pass"/"Fail" on *queue*.  Returns the
    FlashResult (None if it never ran)."""
    reporter = QueueReporter(queue)
    result = None
    try:
        if is_golden(path):
            reporter.warn('refusing to flash a GOLDEN image over CAN: ' + path)
            return None
        print('Flashing node {} with {}'.format(node_id, path))
        result = _flash.flash_port(can_device, int(node_id), path, reporter)
        if result != _flash.FlashResult.SUCCESS:
            reporter.warn('flash failed: ' + _flash.describe(result))
        return result
    except Exception:
        traceback.print_exc()
        return result
    finally:
        _finish(queue, result == _flash.FlashResult.SUCCESS)


def config_child(can_device, node_id, path, queue=None, eds=None):
    """Download the config CSV at *path* to *node_id* (RAM only); "Pass" on
    *queue* when every line was written, else "Fail".  Returns the
    ApplyResult (None if the file was refused or the bus failed)."""
    reporter = QueueReporter(queue)
    result = None
    network = None
    try:
        from . import paths
        network = can_backend.make_network(can_device, bitrate=1_000_000)
        node = network.add_node(int(node_id), eds or paths.puck4_eds())
        print('Loading {} into node {}'.format(path, node_id))
        result = config_csv.load(node, path, int(node_id), reporter)
        print('{} value(s) written, {} error(s)'.format(
            result.written, len(result.errors)))
        return result
    except config_csv.ConfigError as exc:
        reporter.warn(str(exc))
        return None
    except Exception:
        traceback.print_exc()
        return None
    finally:
        if network is not None:
            try:
                network.disconnect()
            except Exception:
                pass
        _finish(queue, result is not None and result.ok)
