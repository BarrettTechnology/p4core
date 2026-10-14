"""p4core — what the P4 apps (p4gui, puckutility, pucktuner) share.

  session      Session: the CAN bus, the selected puck, SYNC ownership
  can_backend  opening a CAN port on Linux / Windows, port discovery
  flash        firmware over CAN through the flashloader
  config_csv   configuration CSVs: validate, load, build from a template
  calibrate    current-sense bias/gain and encoder-zero calibration
  ezero        the encoder-zero sweep
  ops          the shared operations, UI-free (the apps' controllers call these)
  reporter     how an operation reports progress and asks questions
  cia402       ControlWord commands and ModesOfOperation values
  cli          argparse scaffolding and the shared subcommands
  app_settings per-user settings
  paths        package data and per-user directories

Only the modules are imported here: ``import p4core`` does not open a bus
or pull in canopen.  Nothing in p4core imports wx: the wx Reporter and the
splash live in the apps.
"""

__version__ = '0.1.0'
