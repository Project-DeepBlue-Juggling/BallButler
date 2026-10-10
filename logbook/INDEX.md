# Logbook Index

All logbook entries, newest first. Maintained by hand — add a row when you add an
entry. See [README.md](README.md) for the format and [TEMPLATE.md](TEMPLATE.md)
to start a new entry.

## Chapters

Narrative syntheses that read a related block of entries top-to-bottom — the
development, the significant developments/insights, and the open problems. A
chapter is a **reading lens, not a source of record**: it links down to the
entries, which stay authoritative. Open the `.html` in a browser.

| Span | Chapter | Covers |
|------|---------|--------|
| 2026-06-10 → 2026-06-18 | [01 · V0 Throw Accuracy](chapters/01-v0-throw-accuracy.html) | The spatial (aim-correction) + temporal (release-latency) campaign — all 6 entries below |

## Entries

| Date | Status | Type | Subsystem | Title | Entry |
|------|--------|------|-----------|-------|-------|
| 2026-10-10 | in-progress | bugfix | firmware, throwing | BB FW 6 hand torque feedforward ×10: the regenerated protocol_config.h took Jugglebot's hand ODrive InputScale::hand_tor = 1000 and CanInterface applies it to BB's hand ODrive S1 (configured 100) — first FW 6 throwing sitting (2026-10-10 14:49): pre-throw kick 350 mm/s from a −3.2 mm rest, retry kicks at the top of the stroke, 3 SPINOUT_DETECTED, strokes 215 vs 236 mm, launch speed −7 %, landings 120 mm short; fix built and flashed 2026-10-10 17:21 (receipt FW 6 -> 7, link check clean); first FW 7 throwing sitting owed (FW 7: BB's own InputScale::bb_hand_* 100/100 and bb_pitch_*, branch bb-fw7-own-hand-scales; S1 SDO scale guard built as FW 8 from the owner's S1 0.6.11-1 table — SDO READ of node 8's input_torque/vel_scale at every hand arm, mismatch zeroes its FF, branch bb-fw8-sdo-scale-guard — guard built, FW 8 flash pending); the session's RE_PIN +2.4° is not applied | [bb-hand-torque-ff-scale](2026-10-10-bb-hand-torque-ff-scale.md) |
| 2026-10-09 | in-progress | feature | firmware, calibration | Stamped 100 Hz yaw: BB FW 6 sends YAW_ESTIMATE 0x7D8 per fresh 150 Hz sample; can-bridge FW 28 (Jugglebot) publishes it as bb_yaw on /bb/axis_estimates; headers regenerated (owner-tree delta: +0x7D8 + YAW_S_OFFSET_MM sign); both flashed (bridge 2026-10-10 00:01); on the wire 0x93 at 99.9 Hz, 500/500 paired, yaw_age median 3.0 ms; live BB calibrations run on the stamped yaw since 2026-10-10 (bb_yaw_source auto); open: the FW 6 build also borrowed Jugglebot's hand torque scale (see 2026-10-10-bb-hand-torque-ff-scale) — FW 7 owed | [bb-stamped-yaw-100hz](2026-10-09-bb-stamped-yaw-100hz.md) |
| 2026-10-09 | resolved | feature | calibration, tooling | `settle_yaw_gauge.py` + `run_item2_sitting.sh`: settle the constellation estimator's yaw-offset gauge from ~40 corrected throws (pre-registered |mean bearing| ≤ 0.15°); reproduces session B's known −0.50° frame error | [yaw-gauge-settle-tooling](2026-10-09-yaw-gauge-settle-tooling.md) |
| 2026-10-09 | tuned | investigation | calibration, throwing | Full 277-throw local calibration after BB reinstall: 275 accepted, uncorrected 54 mm RMS (bias +44/+9 mm, 8% radial gain); the affine validated on 110 corrected throws at 21.0 mm RMS (the predicted floor) with an 8 mm lateral mean traced to a 0.47° pose-calibration yaw difference between sessions — accepted and deployed (Jugglebot `bb-positive-s-affine-2026-10-09`) | [bb-local-calibration-result](2026-10-09-bb-local-calibration-result.md) |
| 2026-10-07 | resolved | bugfix | calibration, tooling | Pilot extraction 0/10 → 10/10: ball carried rigid-body labels, gravity-only model wrong over full arc (drag + ~1° tilt), apex occlusion | [pilot-trajectory-extraction-fix](2026-10-07-pilot-trajectory-extraction-fix.md) |
| 2026-10-06 | tuned | feature | calibration, tooling | Refill-aware analysis and 331-throw synthetic validation | [local-calibration-analysis](2026-10-06-local-calibration-analysis.md) |
| 2026-10-05 | tuned | feature | calibration, tooling | Mirrored-hand simulation and randomized local two-ball calibration capture | [mirrored-hand-local-calibration](2026-10-05-mirrored-hand-local-calibration.md) |
| 2026-09-30 | in-progress | investigation | firmware, throwing | Layer C yaw settle confirm gains a sample-history term (FW 5): 15 samples (100 ms) in-band + 12 deg/s rate replaces the instantaneous 3 deg/s check that a settled axis's own encoder dither could trip; 4 of 5 R5-sitting NOT_SETTLED refusals were dither, not motion; built, not yet flashed | [yaw-settle-history-term](2026-09-30-yaw-settle-history-term.md) |
| 2026-09-28 | in-progress | feature | firmware, tooling | Firmware update over CAN — the Platform receiver ported to BB, with a safety park (yaw e-stop, pitch ≥ 80°, ODrives IDLE); every session ends in a reboot. Flown: BB FW 2 over CAN (1 → 2, 75.8 s, 0 rewinds), then FW 3 (2 → 3, 56.3 s after the host's sector pause 0.5 → 0.12 s); pitch-raise park branch unexercised; afternoon: FW 4 over CAN in 36.2 s (bridge FW 22, pipelined DATA) | [bb-firmware-update-over-can](2026-09-28-bb-firmware-update-over-can.md) |
| 2026-06-23 | tuned | feature | firmware, throwing, tooling | Loud command-outcome channel (CMD_RESULT) + bb/throw ROS2 action — Phase 2; generic firmware→host outcome relay (CAN1→UDP→action); hardware-validated 2026-06-24 (5/5) | [loud-command-outcome-channel-cmd-result](2026-06-23-loud-command-outcome-channel-cmd-result.md) |
| 2026-06-21 | tuned | feature | firmware, throwing | Throw settle-gates — loud + guaranteed-on-aim throws; Phase 1 (A predictive + C fire-time confirm + serial) hardware-validated; Phase 2 (loud-to-host channel) deferred | [throw-settle-gates-loud-on-aim](2026-06-21-throw-settle-gates-loud-on-aim.md) |
| 2026-06-18 | resolved | investigation | throwing, firmware, calibration, timing | Temporal accuracy resolved (≈44 ms → <10 ms): aim-correction (spatial) + measured release-latency offset (temporal); kinematic-ID & feedforward root-fixes ruled out | [temporal-accuracy-resolved-fractured-solution](2026-06-18-temporal-accuracy-resolved-fractured-solution.md) |
| 2026-06-17 | resolved | investigation | throwing, firmware, calibration | Release-lag fix validated on hardware (δ 140→44 ms); residual is a +3° steeper / +10% hot launch | [release-lag-fix-validated-launch-discrepancy](2026-06-17-release-lag-fix-validated-launch-discrepancy.md) |
| 2026-06-12 | resolved | investigation | throwing, timing, firmware | Temporal "warm-up drift" is a clock-sync artifact (bridge time-master re-acquisition slew), not a thrower effect — root-caused + fixed (2026-06-16) | [temporal-warmup-drift](2026-06-12-temporal-warmup-drift.md) |
| 2026-06-11 | resolved | investigation | throwing, timing, firmware | Release-lag root cause confirmed — two decel-zero mechanisms cancel; 2-line fix (deployed + validated 2026-06-17) | [release-lag-firmware-analysis](2026-06-11-release-lag-firmware-analysis.md) |
| 2026-06-10 | resolved | feature | calibration, throwing, timing | Temporal-accuracy test protocol (v1) + catching-cone uplink plumbing | [temporal-accuracy-protocol](2026-06-10-temporal-accuracy-protocol.md) |
| 2026-06-10 | resolved | feature | calibration, throwing, tracking | Throw aim-correction (2D affine) validated on hardware — ~84% error reduction | [throw-aim-correction-validated](2026-06-10-throw-aim-correction-validated.md) |
| 2026-05-24 | superseded | bugfix | firmware | Axis-settled lead-time gate in executeThrow_ (settle-by-release, silent) — superseded by 2026-06-21 settle-gates | [bb-axis-settled-gate](2026-05-24-bb-axis-settled-gate.md) |
