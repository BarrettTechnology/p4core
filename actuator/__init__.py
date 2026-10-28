"""p4core.actuator -- the per-actuator calibration, open source.

The steps whose results differ unit to unit and that a model's config CSV
does not carry, on a drive whose model is already identified:

  encoder   the sensor at rest; its chip and hardware filter (encfilter,
            enctest)
  bias, gain
            current-sense offsets and gain (p4core.calibrate)
  slope, fold
            the shunt amplifier's current-proportional offset and its
            drive-on baseline (slope)
  winding   electrical zero and winding order (winding, commission)
  runout    the magnet's runout table, 0x3027 (encoder)
  cogging   the cogging feed-forward bins, 0x302E (cogging)

  plans     the selection the apps offer, make_session() and run()
  steps     the steps and the Session that runs them
  device    P4Drive: the drive as the calibrations use it
  drivecal  helpers the calibrations share: CalError, Save.Single,
            de-energise, fault text

Motor identification and loop tuning are not here: they make the model's
CSV once per batch, and add their steps to steps.STEPS (register) and
prereqs.DEPS when they are installed.
"""
