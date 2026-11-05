"""Saving the settings, after each calibration step that passed (steps.Session).

One save (nvsave.save_settings): a burst of Save.Single starved the IWDG on
STM32 firmware before 53536c2.
"""

from .. import nvsave


def save_settings(dev):
    """The drive's settings to NV; the PDOs and the rest of communication stay
    as NV has them."""
    nvsave.save_settings(dev.node)
