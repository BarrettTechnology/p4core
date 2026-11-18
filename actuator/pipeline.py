"""Saving the settings, after each calibration step that passed (steps.Session).

One save (nvsave.save_settings): a burst of Save.Single starved the IWDG on
STM32 firmware before 53536c2.
"""

from .. import nvsave
from ..calibrate import deenergise


def save_settings(dev):
    """The drive's settings to NV; the PDOs and the rest of communication stay
    as NV has them.  Shut down first: firmware 5.0.6+ refuses a save with the
    outputs on (0x08000022), and a step can end Operation Enabled (the runout
    sweep, as the encoder zero did before ff45e75).  The next step enables
    again."""
    deenergise(dev.node, settle_s=0.2)
    nvsave.save_settings(dev.node)
