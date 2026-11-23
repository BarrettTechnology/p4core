# Changelog

All notable changes to p4core are documented in this file. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/). p4core ships as a
wheel next to the apps that need it (the A/B test package carries both); an
app pins the version that has what it uses (puckutility 1.4.3: `>=0.1.2`).

## [Unreleased]

### Changed
- The actuator calibration no longer changes any loop gain. The friction step used to rescale the velocity Kp/Ki to each drag's J (with no prompt under a script), so repeat calibrations of one actuator left it with different gains and different performance. That rescale is now p4sysid's (pucktuner): `steps._r_friction` takes an `after_drag` hook between the drag and the Coulomb feed-forward, and p4sysid's friction step uses it.
- The default actuator calibration (`plans.ACTUATOR`) is now steps 1-5: encoder, bias, gain, slope, electrical zero + winding order. The new `plans.EXTENDED` adds fold, runout, friction and cogging, and `PLANS['extended']` ("Extended Calibration") runs all nine. Fold now runs after winding, so the default steps are numbered 1-5 and the extended steps 6-9. If fold does not run, the drive-gated offset register stays cleared: the slope step clears it.

## [0.1.3] - 2026-11-20

### Changed
- The EDS is firmware 5.0.9's. The NV list adds `0x3017,8` (field weakening on/off) and `0x3017,9` (overmodulation), both saved since 5.0.8.

### Fixed
- A save from a K64 drive left enabled in IDLE is no longer refused. An enabled K64 holds the bridge on in IDLE (the active short), so 5.0.7+ refused the save (0x08000022) and `in_pre_op` gave up because the outputs read as on; the current-sense bias step failed this way. A drive enabled in IDLE is now shut down (controlword 0x06) and saved through PRE-OP. A drive enabled in any other mode is still refused.
- The drag fit no longer rejects a slow cruise as standstill. With the drag stream at one frame per 1 ms SYNC, each speed was a one-frame difference, where a count is 1.5 rad/s, so a 6 rad/s cruise that read 2 counts in one frame fell under the 5 rad/s guard; friction calibration failed with "only 0 usable drag windows" (WAM4 at 5 kHz control). Speeds are now taken over 5 ms.
- The friction step no longer saves a drag fit its own measurements contradict. On the WAM4 (J 1.0e-4 kg.m^2, Tc 24 mN.m) the stiff 1 A drag spring let the steps run at 500 rad/s^2: the fit read J -7.3e-5 and Tc 0.46 N.m and crashed storing the negative J ("Value does not fit in specified type"); at 250 rad/s^2 both runs agreed with each other at J -34 % and Tc x2, and under `-y` that J was saved and the velocity gains rescaled to it. Now:
  - a drag run whose J is not positive, or whose full-current J is more than 30 % off the half-current run's (`DRAG_J_RETRY`), starts the staircase again at half the acceleration, or at half the coast's Tc/J when that is lower (`estim_drag(friction_accel_rad_s2=)`), at most `DRAG_RETRIES` times. The 500 rad/s^2 ceiling stays: the P4-42 and GL40 drags fit within 1 % at it;
  - a drag J that is not positive, or outside `J_AGREEMENT` (20 %) of the coast's, fails the step: J, Tc/B and the velocity gains are left as they are. It used to be a warning.
- A motor thermistor the drive is not configured for is no longer read as 0.0 C. With 0x3026 (divider, beta, R at base) blank the firmware's conversion is 0/0, which the K64 reports as exactly 0.0 C, a plausible temperature; Rs was then referred to 0 C, or (once the DSP ran 45 C warmer) the DSP stood in for a connected thermistor. That reading now counts as no thermistor (`therm_unconfigured`), and the commission start check warns. The P4-37 templates gain the thermistor rows: the board's 3010 ohm pull-up, with the NTC's beta and resistance left 0 for the motor's CSV (WAM4: US Sensor 203JL1A, 3892 / 20000).

## [0.1.2] - 2026-11-13

### Changed
- NV writes go through NMT PRE-OPERATIONAL when the firmware asks for it. 5.0.7+ refuses 0x1010, 0x1011, 0x21B0 and 0x3005,2 writes outside PRE-OP with 0x08000022. On that abort with the outputs off, `nvsave.in_pre_op()` sends the node to PRE-OP, writes again, waits for the commit and starts it again. The saves, the 0x1011 restore in `wipe-nvm`, `set-id`'s NetCfg and a CSV's node-ID row all use it. A node already in PRE-OP, and older firmware, see no NMT traffic.
- The EDS is firmware 5.0.7's.
- `serial --set 0` is allowed: 0 is a serial like any other. Only 0xFFFFFFFF (the erased word, what 4.3.x and 5.0.6+ read when no serial is programmed) shows as "not set" and is refused.

## [0.1.1] - 2026-11-12

### Added
- `set-id --pdo-ids predefined`: also moves the PDO COB-IDs keyed to node 127 (factory configuration) to the puck's ID, keeps bit 31, and saves communication. With no NEW_ID it fixes the current ID's PDOs without a reboot. The CLI lists each change and asks (`-y`). `ops.pdo_id_changes()` and `config_csv.pdo_cob_id_changes()` give the plan read-only.
- `serial [--set S]`: reads the serial number (0x1018,4) or writes it with the firmware's serial key, reads it back and saves it. 0 and 0xFFFFFFFF show as "not set" and are refused.
- CI: `make test` on every push and merge request.

### Changed
- The EDS is firmware 5.0.6's.
- `--id N` asks that node only (one 0x1000 upload) instead of scanning the bus; commands without `--id` or with `--all` still scan.
- `config_csv.remap_pdo_cob_ids` changes a valid PDO's identifier through invalid (0x80000000 | new, then new), which 5.x requires; takes `from_ids`; saves only when something changed.
- Calibration: friction asks before rescaling the velocity gains (old and new shown); cogging is a refinement (a poor result keeps the old table as a warning); a CAN fault during a capture is named as one.

### Fixed
- Saves wait for the commit (0x1010 save-pending bit) so a reboot right after one keeps it; a save refused while the drive is enabled raises `SaveRefused` ("disable the drive before saving"). Save.Single goes through the same path, and the bias calibration saves after returning to IDLE.
- `make_network`: sends wait up to 0.5 s for transmit-queue space, so a CANable's full queue no longer turns a scan into "No Pucks Found".

## [0.1.0] - 2026-11-06

First wheel, shipped in the A/B test package (ab-testing-20261106).
