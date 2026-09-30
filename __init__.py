"""p4core — what the P4 apps (p4gui, puckutility, pucktuner) share.

  session      Session: the CAN bus, the selected puck, SYNC ownership
  can_backend  opening a CAN port on Linux / Windows, port discovery
  flash        firmware over CAN through the flashloader
  config_csv   configuration CSVs: validate, load, build from a template
  calibrate    current-sense bias/gain and encoder-zero calibration
  ezero        the encoder-zero sweep
  ops          the shared operations, UI-free (the apps' controllers call these)
  reporter     how an operation reports progress and asks questions
  cli          argparse scaffolding and the shared subcommands
  wx_reporter  a Reporter for wx frames (imports wx; import it explicitly)
  app_settings per-user settings
  paths        package data and per-user directories

Only the modules are imported here: ``import p4core`` does not open a bus,
import wx or pull in canopen.
"""

__version__ = '0.1.0'
