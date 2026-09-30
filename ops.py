"""ops.py — the operations every P4 app offers, as plain functions.

These are the controllers the three apps share: each takes a connected
Session, the node ID(s) it acts on and a Reporter, does the job, and returns
a result.  They print nothing and show nothing; the command line and the GUI
are two ways of calling them.

  scan(session)                          node IDs on the bus
  info(session, node_id, flashloader)    a dict of what the puck is
  flash(session, node_id, path)          firmware; the node is reselected
  load_config(session, node_id, csv)     download, Save.All, reboot
  export_config(session, node_id, template, out)
  set_id(session, old_id, new_id)        NetCfg, reboot, PDO COB-ID remap
  reset(session, node_id)                NMT reset and wait for it
  calibrate(session, node_id, steps)     p4core.calibrate.calibrate_all
"""

import os
import time

from . import calibrate as _calibrate
from . import can_backend, config_csv
from . import flash as _flash
from .reporter import Reporter


class OperationError(RuntimeError):
    """An operation could not be carried out."""


def _reporter(reporter):
    return reporter or Reporter()


def scan(session, reporter=None):
    reporter = _reporter(reporter)
    reporter.status('Scanning the bus...')
    nodes = session.scan()
    reporter.note('Found {} node(s): {}'.format(len(nodes), nodes))
    return nodes


def require_on_bus(session, node_id):
    """Raise OperationError unless the last scan saw *node_id*."""
    if not session.found_ids:
        session.scan()
    if node_id not in session.found_ids:
        raise OperationError('node {} not found on the bus (found {})'.format(
            node_id, session.found_ids))


# ---------------------------------------------------------------------------
# Info
# ---------------------------------------------------------------------------
def info(session, node_id, flashloader=False, reporter=None):
    """What *node_id* is: firmware, model, and a few live readings.

    *flashloader* also reads the flashloader version, which is only readable
    from inside the bootloader: the puck is rebooted into it and relaunched,
    so this is not passive.  Values that cannot be read are None.
    """
    reporter = _reporter(reporter)
    node = session.add_node(node_id)

    def sdo(name, sub=None):
        try:
            var = node.sdo[name] if sub is None else node.sdo[name][sub]
            return var.raw
        except Exception:
            return None

    try:
        firmware = _flash.read_version(node)
    except Exception:
        firmware = None
    product = sdo(0x1018, 2)
    result = dict(
        node_id=node_id,
        firmware=firmware,
        product_code=product,
        model=can_backend.model_from_product_code(product)
        if product is not None else None,
        serial=sdo(0x1018, 4),
        pwm_hz=sdo(0x3001, 1),
        bus_voltage_v=None,
        temperature_c=sdo('Amplifier', 'Temperature'),
        flashloader=None,
    )
    bus = sdo('Amplifier', 'BusVoltage')
    if bus is not None:
        result['bus_voltage_v'] = bus / 10.0
    if flashloader:
        result['flashloader'] = _flash.read_flashloader_version(
            session.network, node_id, reporter)
        _wait_for_application(session, node_id, reporter)
    return result


def format_info(result):
    """info() as indented lines."""
    def show(value, fmt='{}'):
        return 'unknown' if value is None else fmt.format(value)
    lines = ['Node {}'.format(result['node_id']),
             '  Firmware:     {}'.format(show(result['firmware']))]
    if 'flashloader' in result and result['flashloader'] is not None:
        lines.append('  Flashloader:  {}'.format(result['flashloader']))
    lines += [
        '  Model:        {}  (product code {})'.format(
            show(result['model']), show(result['product_code'])),
        '  Serial:       {}'.format(show(result['serial'])),
        '  PWM:          {}'.format(show(
            None if result['pwm_hz'] is None else result['pwm_hz'] / 1000.0,
            '{:.1f} kHz')),
        '  Bus voltage:  {}'.format(show(result['bus_voltage_v'], '{:.1f} V')),
        '  Temperature:  {}'.format(show(result['temperature_c'], '{} C')),
    ]
    return lines


def _wait_for_application(session, node_id, reporter, timeout=None):
    """Poll 0x1000 until the application answers again, with the EDS back."""
    timeout = timeout or session.REBOOT_TIMEOUT_S
    reporter.sleep(session.BOOT_S)
    session._forget(node_id)
    node = session.add_node(node_id)
    deadline = time.monotonic() + timeout
    while True:
        try:
            node.sdo.upload(0x1000, 0)
            return node
        except Exception as exc:
            if time.monotonic() > deadline:
                raise OperationError('node {} did not come back: {}'.format(
                    node_id, exc)) from exc
            reporter.sleep(0.2)


# ---------------------------------------------------------------------------
# Firmware
# ---------------------------------------------------------------------------
def flash(session, node_id, path, reporter=None):
    """Program *path* into *node_id* on the session's bus.

    Refuses GOLDEN images: those are for SWD programming only.  Returns the
    new application version.  Raises OperationError on failure.  When
    *node_id* was the selected node it is selected again.
    """
    reporter = _reporter(reporter)
    if 'golden' in os.path.basename(path).lower():
        raise OperationError('refusing to flash a GOLDEN image over CAN: {}'
                             .format(path))
    reselect = session.node is not None and session.node.id == node_id
    if reselect:
        session.set_sync_rate(0)
    result = _flash.flash(session.network, node_id, path, reporter)
    if result != _flash.FlashResult.SUCCESS:
        _forget_quietly(session, node_id)
        raise OperationError('flash failed: {}'.format(_flash.describe(result)))
    _wait_for_application(session, node_id, reporter)
    if reselect:
        session.select_node(node_id)
    version = session.read_version(node_id)
    reporter.note('Node {} running {}'.format(node_id, version))
    return version


def _forget_quietly(session, node_id):
    try:
        session._forget(node_id)
    except Exception:
        pass


# ---------------------------------------------------------------------------
# Configuration CSVs
# ---------------------------------------------------------------------------
def load_config(session, node_id, path, reporter=None, save=True,
                reboot=True, force=False):
    """Download the config CSV at *path* to *node_id*, then (by default)
    Save.All and reboot, because much of the configuration is latched at
    boot.  Returns config_csv's ApplyResult; raises config_csv.ConfigError
    for an invalid file (unless *force*)."""
    reporter = _reporter(reporter)
    node = session.add_node(node_id)
    reporter.status('Loading {} into node {}...'.format(
        os.path.basename(path), node_id))
    result = config_csv.load(node, path, node_id, reporter, force=force)
    reporter.note('{} value(s) written, {} error(s)'.format(
        result.written, len(result.errors)))
    if save:
        reporter.status('Saving to NVM...')
        session.save_all(node)
    if reboot:
        reporter.status('Rebooting...')
        session.reset_node(node_id)
    return result


def export_config(session, node_id, template, output, reporter=None):
    """Write *output*: *template* filled with *node_id*'s current values."""
    reporter = _reporter(reporter)
    node = session.add_node(node_id)
    reporter.status('Reading node {} into {}...'.format(node_id, output))
    count = config_csv.build_from_template(node, template, output, reporter)
    reporter.note('{} value(s) written to {}'.format(count, output))
    return count


# ---------------------------------------------------------------------------
# Node ID
# ---------------------------------------------------------------------------
def set_id(session, old_id, new_id, reporter=None):
    """Move a puck from *old_id* to *new_id*.

    NetCfg self-persists and takes effect at the next boot, so: write it,
    NMT-reset, wait for the puck on the new ID, then remap the 8 PDO COB-IDs
    (the firmware does not re-derive them) and save them.  Returns True when
    the remap verified.
    """
    reporter = _reporter(reporter)
    if not 1 <= new_id <= 127:
        raise OperationError('node ID must be 1..127, not {}'.format(new_id))
    if new_id == old_id:
        reporter.note('Node {} already has that ID'.format(old_id))
        return True
    require_on_bus(session, old_id)
    if new_id in session.found_ids:
        raise OperationError('node ID {} is already in use'.format(new_id))
    was_selected = session.node is not None and session.node.id == old_id
    if was_selected:
        session.set_sync_rate(0)
    node = session.add_node(old_id)
    reporter.status('Setting node ID {} -> {}...'.format(old_id, new_id))
    node.sdo['NetCfg'].raw = new_id
    session.send_nmt(0x81, old_id)
    _forget_quietly(session, old_id)
    if was_selected:
        session.node = None
    new_node = _wait_for_application(session, new_id, reporter)
    ok, _ = config_csv.remap_pdo_cob_ids(new_node, new_id, old_id=old_id,
                                         save=True, verify=True,
                                         log=reporter.note)
    session.found_ids = sorted(
        set(session.found_ids) - {old_id} | {new_id})
    if ok:
        reporter.note('Node {} is now node {}; PDO COB-IDs remapped and saved'
                      .format(old_id, new_id))
    else:
        reporter.warn('Node ID changed, but the PDO COB-IDs did not all '
                      'verify: load the configuration to repair them')
    if was_selected:
        session.select_node(new_id)
    return ok


def reset(session, node_id, reporter=None):
    reporter = _reporter(reporter)
    reporter.status('Resetting node {}...'.format(node_id))
    return session.reset_node(node_id)


# ---------------------------------------------------------------------------
# Calibration
# ---------------------------------------------------------------------------
def calibrate(session, node_id, steps=None, reporter=None):
    """Current-sense and encoder calibration (p4core.calibrate).  The motor
    turns.  Returns {step: result}."""
    reporter = _reporter(reporter)
    node = session.add_node(node_id)
    return _calibrate.calibrate_all(node, reporter, steps)
