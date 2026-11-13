# Changelog

All notable changes to p4core are documented in this file. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/). p4core ships as a
wheel next to the apps that need it (the A/B test package carries both); an
app pins the version that has what it uses (puckutility 1.4.3: `>=0.1.2`).

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
