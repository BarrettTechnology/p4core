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
* **views** — `cli` (argparse subcommands, `ConsoleReporter`) and the wx GUI
  (`WxReporter`), both calling the same controllers.

## Shared commands

`p4core.cli.add_common_commands` gives every app:

    scan                         node IDs on the bus
    info [--all] [--flashloader] firmware, model, live readings
    flash FIRMWARE [--all]       .bin / .ebin over CAN (GOLDEN images refused)
    config load CSV [--all]      download, Save.All, reboot
    config export TEMPLATE OUT   a template filled with the puck's values
    config check CSV             validate without a puck
    config templates             the shipped templates
    set-id NEW                   NetCfg, reboot, PDO COB-ID remap
    reset [--all]                NMT reset and wait
    calibrate [STEP...]          encoder-test, bias, gain, ezero (motor turns)

with `--can PORT` (default `$P4_CAN` or `can0`), `--id N` (default: the only
puck on the bus), `--fd`/`--classic`, `-y`, `-q`, `-v` and `--json`, before or
after the command.  Exit codes: 0 ok, 1 failed, 2 usage, 3 no bus/puck,
130 interrupted.

## Layout

The repository root is the package (as in p4sysid): add it to an app as a
submodule named `p4core`, or `pip install` it (package data included).

## Tests

    python3 -m unittest discover -s tests

`tests/fake.py` is a fake node over the real `puck4.eds`; the calibration
tests drive a small simulated puck.
