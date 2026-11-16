# p4core

What the P4 puck tools — p4gui, puckutility and pucktuner — share: the CAN
session, flashing, configuration CSVs, calibration, and the scaffolding that
lets every operation run from the command line or from a GUI.

Each app is split model / controller / view:

* **model** — `Session` (and the app's subclass of it): the bus, the
  selected puck, SYNC.  Nothing but the Session starts SYNC.
* **controllers** — plain functions (`p4core.ops`, and each app's own
  `controllers` module) that take a Session, node IDs, options and a
  `Reporter`, and return results.  No printing, no dialogs.
* **shared algorithms** — `tuning` (pucktuner's current / velocity /
  position gain design and PWM selection), `ezero`, `calibrate`.
* **views** — `cli` (argparse subcommands, `ConsoleReporter`) and the wx GUI
  (each app's own `WxReporter`), both calling the same controllers.  The one
  wx module is `canbar`, the CAN Port / Scan Pucks / Select ID bar the
  GUIs share (the `gui` extra); nothing else in p4core imports wx.

## Shared commands

`p4core.cli.add_common_commands` gives every app, and `python -m p4core`:

    scan                         node IDs on the bus
    info [--all] [--flashloader] firmware, model, live readings
    flash FIRMWARE [--all]       .bin / .ebin over CAN (GOLDEN images refused)
    config load CSV [--all]      download, Save.All, reboot
    config export TEMPLATE OUT   a template filled with the puck's values
    config check CSV             validate without a puck
    config templates             the shipped templates
    set-id NEW                   NetCfg, reboot, PDO COB-ID remap
    reset [--all]                NMT reset and wait
    wipe-nvm [--backup CSV]      erase NV but bus settings, reload the blank template
    calibrate [STEP...]          encoder-test, bias, gain, ezero (motor turns)

with `--can PORT` (default `$P4_CAN` or `can0`), `--id N` (default: the only
puck on the bus), `--fd`/`--classic`, `-y`, `-q`, `-v` and `--json`, before or
after the command.  Exit codes: 0 ok, 1 failed, 2 usage, 3 no bus/puck,
130 interrupted.

## NV saves on firmware 5.0.7+ (`nvgate`)

Firmware 5.0.7 stores to NV only in NMT PRE-OPERATIONAL and only with the motor
outputs off: any other write to 0x1010 (save) or 0x1011 (restore) is refused with
SDO abort `0x08000022` (stm32 `common/parse.c` `parseNvGate()`, FIRMWARE_UPDATES
5.0.6/5.0.7). The P4 tools save in many places (Save.All, Save.App/Mfg, Save.Single
after each calibration step), so without this every save from pucktuner fails on
5.0.7 and calibrations stay in RAM only.

`p4core.nvgate.install()` wraps `canopen`'s `SdoClient.download`, which every
save ends in (`node.sdo['Save'][...].raw = ...` included). Other indices are not
touched. A 0x1010/0x1011 write:

1. is sent as is: older firmware, or a node already in PRE-OP, takes it;
2. if refused with `0x08000022` while the drive is not Operation Enabled
   (0x6041): NMT Enter PRE-OP (0x80) for that node, the save again, wait while
   0x1010,1 reads bit 2 (save pending, STM32), NMT Start (0x01) back to OPERATIONAL;
3. if refused while Operation Enabled: raised as `nvgate.NmtSaveRefused` with no
   NMT change. The save has to come after the disable.

Installed once by `p4core.session` (CLI, p4sysid) and the pucktuner GUI; idempotent.
While the save runs the node is in PRE-OP for a few ms (STM32 ~50 ms): PDOs and
SYNC-driven updates pause for that time. Tests: `tests/test_nvgate.py`.

## Layout

The repository root is the package (as in p4sysid): add it to an app as a
submodule named `p4core`, or `pip install` it (package data included).

## Tests

    python3 -m unittest discover -s tests

`tests/fake.py` is a fake node over the real `puck4.eds`; the calibration
tests drive a small simulated puck.
