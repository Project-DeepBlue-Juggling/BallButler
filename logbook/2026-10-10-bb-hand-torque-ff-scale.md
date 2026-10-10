---
title: BB FW 6 sends the hand ODrive ten times the planned torque feedforward — the regenerated protocol_config.h took Jugglebot's hand_tor = 1000 for BB's own hand (configured 100); pre-throw kick, 215 mm strokes, spinouts, throws ~7 % slow
type: bugfix
date: 2026-10-10
status: in-progress
phase: "BB accuracy — yaw gauge settlement"
related_entries:
  - 2026-10-09-bb-stamped-yaw-100hz
  - 2026-10-09-bb-local-calibration-result
files_changed:
  - logbook/2026-10-10-bb-hand-torque-ff-scale.md
  - logbook/2026-10-09-bb-stamped-yaw-100hz.md
  - logbook/INDEX.md
external_changes:
  - "none yet — the proposed fix adds bb_hand_vel / bb_hand_tor to Jugglebot config/protocol_config.yaml and regenerates"
subsystem:
  - firmware
  - throwing
tags:
  - accuracy
  - repeatability
---

# BB FW 6 hand torque feedforward ×10

## Summary

The first throwing sitting on BB FW 6 (2026-10-10 14:49, `run_item2_sitting.sh`, session
`20261010T035151_835895Z`) showed a new kick of the hand before every throw and before every reload
retry, three `SPINOUT_DETECTED` disarms on the hand drive, strokes ending at 215 mm instead of 236 mm,
and landings 117 mm short. Cause, confirmed in the code: FW 6 regenerated `protocol_config.h` from
Jugglebot `skill-stack`, whose `InputScale::hand_tor` is Jugglebot's hand ODrive Pro scale (1000 since
Jugglebot `75030f73`, 2026-09-15), and `CanInterface` uses that constant for BB's own hand ODrive S1,
which is configured with `input_torque_scale` 100. Every hand torque feedforward has gone out ×10 since
the flash. Fix proposed (BB's own hand scales, FW 7), not yet built or flashed. Full analysis:
`~/bb_calibration_sessions/hand_jolt_20261010/REPORT.md`.

## Problem

- Owner: the hand jolted from about −3.3 mm to about 0 mm just before each throw, and before the pitch rose
  for a 2nd/3rd reload attempt, never the 1st. First time seen.
- Telemetry (`/bb/axis_estimates` `bb_hand`, 100 Hz): the hand now rests on its bottom stop at −3.21 ± 0.07 mm
  (FW 5: +0.46 mm, 109 of 109 times) and kicks at nominal release − 0.644 s, peak 350 ± 16 mm/s, overshoot
  +1.90 mm, rebound −3.71 mm (40 of 40 throws). The retry kick is at the TOP of the stroke (284 → 288.5 mm at
  284–365 mm/s; FW 5's retry move: 21–57 mm/s). Pitch and yaw do not move at those moments; no ROS command
  arrives then — the kick is frame 0 of the firmware's own throw plan (the 0.1 rev pre-move the planner inserts
  when the start is clamped to ≥ 0, `HandPathPlanner.cpp:40-51, :286`).
- Throws: stroke end 215.5 ± 2.2 mm (FW 5 236.0 ± 0.3), peak hand speed 5–6 % lower, ensemble braking ~−130 m/s²
  (FW 5 ~−55), horizontal launch speed / predicted 0.873 (FW 5 0.943), catch −10 ms vs predicted (FW 5 +27 ms),
  landings −120 ± 30 mm along-range and +29 mm cross-range (FW 5 −1.6 / −8.8 mm).
- `SPINOUT_DETECTED` on bb_hand after throws 18, 30 and 42 (`~/.ros/log/python3_2033231_1791604158337.log:87,130,173`);
  none in any FW 5 log.

## Root Cause

- `ball_butler_main/protocol_config.h:221`: `InputScale::hand_tor = 1000.0f` in FW 6 (`f296441`); FW 5's header
  had 100. The header is generated from Jugglebot `config/protocol_config.yaml` `input_scales`, where `hand_tor`
  is documented as matching `odrive_pro_hand_config.json` — Jugglebot's hand, not BB's.
- `ball_butler_main/CanInterface.h:353-354` sets `kVelScale_ = InputScale::hand_vel`, `kTorScale_ = InputScale::hand_tor`
  and `CanInterface.cpp:234` scales BB's hand `torque_ff` by it. `CanInterface.h:21` still documents the intent:
  "scaled internally by 100.0".
- BB's hand drive (node 8): `Jugglebot-skills/config/ODrive config Files/odrive_s1_bb_hand_config.json`
  `input_torque_scale: 100`, `input_vel_scale: 100`.
- Effect: smooth moves plan 0.0097 N·m → FW 5 sent 1 count (0.010 N·m), FW 6 sends 10 (0.10 N·m); a 3.36 m/s throw
  plans +0.064 / −0.085 N·m → FW 6 asks +0.64 / −0.85 N·m, clipped by the drive's current limit. The home move
  therefore reaches 0 at ~130 mm/s (FW 5 ~30) and, with the streamer dropping to IDLE on the last frame
  (`HandTrajectoryStreamer.h:132`), coasts onto the stop; the planner's pre-move from the stop at ×10 torque is the kick.
- Nothing else in FW 6 touches the hand (BB reads only the `BB*` namespaces of the regenerated headers; the
  `YAW_S_OFFSET_MM` sign is read by no firmware). Bridge FW 28 and the ROS changes since 2026-10-07 do not touch
  the throw or reload command paths. The 0x7D8 yaw frame (lowest CAN priority) cannot make a 10× dynamic change.
- Still owed: a live read of node 8's `axis0.config.can.input_torque_scale` / `input_vel_scale` (odrivetool or SDO).
  100 confirms this chain; 1000 would mean an ODrive/hardware change instead.

## Fix (proposed — pending the owner's go-ahead; nothing built or flashed)

1. BB's own hand scales (velocity 100, torque 100) in `CanInterface.h:353-354`, best as `bb_hand_vel` /
   `bb_hand_tor` keys under `input_scales` in Jugglebot `config/protocol_config.yaml` + regeneration (a local
   constant in `BallButlerConfig.h` also works); fix the comment at `CanInterface.h:21`. The velocity scale has the
   same latent coupling (100 = 100 today by coincidence).
2. Guard at hand arm: read node 8's torque scale over SDO and send zero torque feedforward on a mismatch (the S1
   0.6.11 endpoint id needs adding; only the Pro's, 283, is tabled).
3. FW 7 (and `BB_FW_VERSION_EXPECTED` 7 in Jugglebot); flash over CAN with the live checkout's tool.
4. Optional, separate: hold closed loop in `HandTrajectoryStreamer.h:131-134` until the hand has nearly stopped,
   so the rest position is predictable. Do not loosen the planner clamp to hide the pre-move.

## Verification

- Done (offline): FW 6 sitting bag `2026-10-10_14-49-14` and session `20261010T035151_835895Z` against the FW 5
  sessions `20261009T031319_612936Z` and `20261009T002142_931068Z`; scripts and CSVs in
  `~/bb_calibration_sessions/hand_jolt_20261010/`. Code pointers above checked in both repos.
- Pass criteria for the FW 7 sitting: hand rests at ~+0.46 mm (no pre-move); retry move < ~60 mm/s; stroke ends
  at ~236 mm; no spinouts; catch time back to ~+27 ms vs predicted. Then re-run the 40-throw validation.
- **Consequences for the accuracy work:** the 14:49 session's landings (mean −120 / −15 mm, RMS 128 mm) are this
  bug's; `settle_yaw_gauge.py`'s RE_PIN +2.406° from it is NOT applied (its own rotation-vs-translation fit reads the
  lateral error as a translation: slope −0.3 ± 0.6°, intercept +46 ± 11 mm), the pin stays 0.208°, and the aim
  correction is not refitted from it.

## Open Questions / Follow-ups

- The live node 8 scale read (owner, odrivetool).
- Whether a similar borrowed constant exists elsewhere in BB's use of the regenerated headers (grep `InputScale`,
  `odrive_pro` names in `ball_butler_main/`).
