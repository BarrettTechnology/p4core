"""flash.py — firmware over CAN through the P4 flashloader.

Merged from the three apps' flashp4.py.  From puckutility: the AutoLaunch
race fix and read_flashloader_version().  From p4gui: the bus is always
released, and the EDS is found by path rather than the working directory.

The operations run on a canopen Network the caller already has (a Session's),
so an app no longer disconnects its bus, lets a subprocess open its own, and
rescans afterwards.  flash_port() keeps the old open-a-port-and-flash form
for scripts.

Progress goes to a Reporter.  Results are FlashResult codes; describe() turns
one into a sentence.
"""

import array
import binascii
import enum
import logging
import os
import time

import semver

from . import can_backend, paths
from .reporter import Reporter


class FlashResult(enum.IntEnum):
    """Result codes; the numbers are the old flashp4.flash_result's."""
    SUCCESS = 0
    FILE_NOT_FOUND = 1
    CANDEV_FAILED = 2
    RESET_FAILED = 3
    VERSION_INCOMPATIBLE = 4
    EXTENSION_INVALID = 5
    ERASE_FAILED = 6
    FLASH_FAILED = 7
    CRC_MISMATCH = 8
    AUTH_FAILED = 9


# Old names, for code that compared against flashp4.flash_result.X.
flash_result = FlashResult

DESCRIPTIONS = {
    FlashResult.SUCCESS: 'firmware programmed and launched',
    FlashResult.FILE_NOT_FOUND: 'firmware file not found',
    FlashResult.CANDEV_FAILED: 'could not open the CAN device',
    FlashResult.RESET_FAILED: 'the node did not reboot into its flashloader',
    FlashResult.VERSION_INCOMPATIBLE: 'flashloader older than 2.1.0',
    FlashResult.ERASE_FAILED: 'flash erase failed',
    FlashResult.EXTENSION_INVALID: 'firmware must be a .bin or .ebin file',
    FlashResult.AUTH_FAILED: 'the encrypted image failed authentication',
    FlashResult.FLASH_FAILED: 'programming failed',
    FlashResult.CRC_MISMATCH: 'CRC mismatch after programming',
}


def describe(result):
    return DESCRIPTIONS.get(FlashResult(result), 'unknown result')


class FlashCommand(enum.IntEnum):
    """ProgramCommand values, as the flashloader defines them."""
    NO_OPERATION = 0
    START = 1
    END = 2
    ERASE = 3
    LAUNCH = 4
    RESET = 5


#: Oldest flashloader the protocol below works with.
MIN_FLASHLOADER = '2.1.0'

#: How long to keep retrying AutoLaunch = 0 after the reset (s).
AUTOLAUNCH_WINDOW_S = 0.5


def get_version(vers):
    """Packed uint32 -> "Major.Minor.Patch"."""
    return '{0}.{1}.{2}'.format(
        (vers >> 24) & 0xFF, (vers >> 8) & 0xFFFF, vers & 0xFF)


def read_version(node):
    """0x100A as "Major.Minor.Patch", in whichever form the node publishes it.

    Applications since the DS301 port publish a VISIBLE_STRING; older
    applications and every flashloader publish the packed U32 that
    get_version() decodes.  The two are told apart by length: the U32 is four
    bytes and the shortest string, "0.0.0", is five.  Reads with a raw upload
    so the answer does not depend on which type the EDS declares.
    """
    raw = node.sdo.upload(0x100A, 0)
    if len(raw) == 4:
        return get_version(int.from_bytes(raw, 'little'))
    return raw.rstrip(b'\0').decode('ascii')


def _enter_flashloader(network, can_id):
    """Reset *can_id* into its flashloader and keep it there.

    Race: the flashloader auto-launches the application after StartTimeoutMs
    (0x3410:08, default 100 ms in flashloader.eds).  If AutoLaunch=0 doesn't
    land before that timer expires, the app comes up and silently handles
    ProgramCommand writes by aborting with 0x06010000 ("Unsupported access").
    A pre-emptive settle sleep overshoots the window — so the write goes out
    immediately and is retried quickly while the SDO server warms up.
    """
    if can_id in network:
        del network[can_id]
    node = network.add_node(can_id, paths.flashloader_eds())
    node.nmt.state = 'RESET'                       # Reboot into flashloader
    node.nmt.wait_for_heartbeat(timeout=1)         # CANopen boot-up message
    deadline = time.time() + AUTOLAUNCH_WINDOW_S
    while True:
        try:
            node.sdo['ProgramInfo']['AutoLaunch'].raw = 0  # Stay in flashloader
            return node
        except Exception:
            if time.time() >= deadline:
                raise
            time.sleep(0.005)


def _launch(node):
    """Hand the node back to its application.

    LAUNCH reboots the node out of the flashloader, so it can never ack the
    SDO — the confirmed write always "times out".  That is the intended
    outcome, so the timeout is short and canopen's logger is quieted for it.
    AutoLaunch = 1 first, so a power cycle self-heals if LAUNCH does not take.
    """
    canlog = logging.getLogger('canopen')
    level = canlog.level
    timeout = node.sdo.RESPONSE_TIMEOUT
    canlog.setLevel(logging.CRITICAL)
    node.sdo.RESPONSE_TIMEOUT = 0.25
    try:
        for var, value in ((node.sdo['ProgramInfo']['AutoLaunch'], 1),
                           (node.sdo['ProgramCommand']['Command'],
                            FlashCommand.LAUNCH)):
            try:
                var.raw = value
            except Exception:
                pass
    finally:
        canlog.setLevel(level)
        node.sdo.RESPONSE_TIMEOUT = timeout


def _write_image(node, words, reporter):
    node.sdo['ProgramCommand']['Command'].raw = FlashCommand.START
    total = len(words)
    for i, word in enumerate(words):
        node.sdo['ProgramCommand']['Word'].raw = word  # Exception on FLASH_FAIL
        if i % 100 == 0:
            reporter.progress(100 * i // total)
            reporter.check_abort()
    # END: the flashloader computes the CRC; SDO abort on AUTH_FAIL.
    node.sdo['ProgramCommand']['Command'].raw = FlashCommand.END
    reporter.progress(100)


def flash(network, can_id, file_name, reporter=None):
    """Program *file_name* (.bin or encrypted .ebin) into node *can_id* on
    an already-connected *network*.  Returns a FlashResult.

    The node is left running the new application.  Its object-dictionary
    entry in *network* is the flashloader's afterwards; a Session caller
    should reset_node()/select_node() it again.
    """
    reporter = reporter or Reporter()
    if not os.path.isfile(file_name):
        return FlashResult.FILE_NOT_FOUND
    ext = os.path.splitext(file_name)[1].lower()
    if ext not in ('.bin', '.ebin'):
        reporter.warn('Invalid extension {}, expected .bin or .ebin'.format(ext))
        return FlashResult.EXTENSION_INVALID

    reporter.status('Rebooting node {} into its flashloader...'.format(can_id))
    try:
        node = _enter_flashloader(network, can_id)
    except Exception as exc:
        reporter.warn('Failed to reset node {}: {}'.format(can_id, exc))
        return FlashResult.RESET_FAILED

    version = get_version(node.sdo['MfgSoftwareVersion'].raw)
    reporter.note('Node {}: flashloader {}'.format(can_id, version))
    if semver.Version.parse(version) < semver.Version.parse(MIN_FLASHLOADER):
        reporter.warn('Flashloader >= {} required'.format(MIN_FLASHLOADER))
        return FlashResult.VERSION_INCOMPATIBLE

    node.sdo.RESPONSE_TIMEOUT = 10  # erase and programming take time
    reporter.status('Erasing...')
    try:
        node.sdo['ProgramCommand']['Command'].raw = FlashCommand.ERASE
    except Exception:
        return FlashResult.ERASE_FAILED

    with open(file_name, 'rb') as image:
        if ext == '.ebin':
            for step in ('ProgramAesIv', 'ProgramAesTag'):
                for i, b in enumerate(array.array('B', image.read(16))):
                    node.sdo[step]['{:X}'.format(i + 1)].raw = b
            node.sdo['ProgramInfo']['IsEncrypted'].raw = 1
            write_failure = FlashResult.AUTH_FAILED
        else:
            node.sdo['ProgramInfo']['IsEncrypted'].raw = 0
            write_failure = FlashResult.FLASH_FAILED
        data = image.read()

    reporter.status('Programming {} ({} bytes)...'.format(
        os.path.basename(file_name), len(data)))
    try:
        # The flashloader expects 4-byte words.
        _write_image(node, array.array('I', data), reporter)
    except Exception as exc:
        if exc.__class__.__name__ == 'Aborted':
            raise
        reporter.warn('Programming failed: {}'.format(exc))
        return write_failure

    if ext == '.bin':                   # the CRC covers the whole app area
        size = node.sdo['ProgramInfo']['MaxSize'].raw
        crc = binascii.crc32(data + b'\xFF' * (size - len(data))) & 0xFFFFFFFF
        if crc != node.sdo['ProgramInfo']['CRC32'].raw:
            return FlashResult.CRC_MISMATCH

    reporter.status('Application validated. Launching...')
    node.sdo['ProgramCommand']['Command'].raw = FlashCommand.LAUNCH
    return FlashResult.SUCCESS


def read_flashloader_version(network, can_id, reporter=None):
    """The flashloader's version, without erasing or programming anything.

    The flashloader version is only readable from inside the bootloader —
    while the application runs, 0x100A is the application's — so this enters
    it exactly as flash() does, reads MfgSoftwareVersion and relaunches the
    application.  Returns the version string, or None if the bootloader could
    not be entered or read.
    """
    reporter = reporter or Reporter()
    try:
        node = _enter_flashloader(network, can_id)
    except Exception as exc:
        reporter.warn('Could not enter the flashloader: {}'.format(exc))
        return None
    try:
        return get_version(node.sdo['MfgSoftwareVersion'].raw)
    except Exception as exc:
        reporter.warn('Could not read the flashloader version: {}'.format(exc))
        return None
    finally:
        _launch(node)


def flash_port(can_device, can_id, file_name, reporter=None):
    """Open *can_device*, flash, and close it again.  Returns a FlashResult."""
    if not os.path.isfile(file_name):
        return FlashResult.FILE_NOT_FOUND
    try:
        network = can_backend.make_network(can_device, bitrate=1_000_000)
    except Exception:
        return FlashResult.CANDEV_FAILED
    try:
        return flash(network, can_id, file_name, reporter)
    finally:
        try:
            network.disconnect()
        except Exception:
            pass
