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
  - ball_butler_main/protocol_config.h
  - ball_butler_main/CanInterface.h
  - ball_butler_main/CanInterface.cpp
  - ball_butler_main/FwUpdate.h
  - logbook/2026-10-10-bb-hand-torque-ff-scale.md
  - logbook/2026-10-09-bb-stamped-yaw-100hz.md
  - logbook/INDEX.md
external_changes:
  - "Jugglebot branch bb-own-input-scales-2026-10-10 (e0156a7d): config/protocol_config.yaml input_scales + bb_hand_vel/tor 100/100, bb_pitch_vel/tor 1000/1000; regenerated config/generated/protocol_config.{h,py}, ros_ws/src/jugglebot/jugglebot/protocol_config.py and the Platform / can-bridge / CatchingCone protocol_config.h copies; teensy_link/rpc_args.py BB_FW_VERSION_EXPECTED 6 -> 7; logbook/2026-10-10-bb-own-input-scales.md + INDEX.md"
commits:
  - aac6a98
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
the flash. Fix built (FW 7, BB's own hand and pitch scales, branch `bb-fw7-own-hand-scales`), not yet flashed. Full analysis:
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

## Fix (built 2026-10-10; NOT flashed — flash and a throwing sitting still owed)

Owner's decision: BB-specific keys in Jugglebot's `input_scales`, one pair per axis with unique limits
("each axis with unique limits should have its own set of keys"); Jugglebot's `hand_*` / `leg_*` untouched.

1. Jugglebot `config/protocol_config.yaml` (branch `bb-own-input-scales-2026-10-10`, `e0156a7d`):
   `bb_hand_vel: 100`, `bb_hand_tor: 100` (node 8, ODrive S1, matches `odrive_s1_bb_hand_config.json` and the
   owner's odrivetool read) and `bb_pitch_vel: 1000`, `bb_pitch_tor: 1000` (node 7, ODrive Micro, matches this
   repo's `bb_pitch_odrive_micro_config.json`). Regenerated with `generate_config.py --no-external`; the header
   was copied into `ball_butler_main/` by hand. Its diff against FW 6 is only the four `InputScale::bb_*` lines.
2. BB axis audit of `ball_butler_main/`: the only scaled feedforward is `CanInterface::sendInputPos`
   (`InputScale::` was used nowhere else; no literal 100/1000 scaling elsewhere).
   - Hand (node 8): `HandTrajectoryStreamer` streams vel/tor FF -> `bb_hand_*` (the fix).
   - Pitch (node 7): `PitchAxis` sends `vel_ff = tor_ff = 0` (TRAP_TRAJ), so its bytes are unchanged; it gets
     `bb_pitch_*` so it can never inherit the hand's. Caveat: the pitch JSON is a 2026-03 copy whose `node_id`
     reads 0 (live drive is 7), so 1000/1000 is NOT confirmed on the live drive. Read it back before ever
     sending pitch feedforward.
   - Yaw: brushed DC motor on an H-bridge (PWM, `YawAxis`), not an ODrive: no keys.
   - `sendInputVel` (hand homing) sends float vel/torque, not scaled ints: no keys.
3. `CanInterface::sendInputPos` selects the pair per node: `pitch_node_id_` -> `bb_pitch_*`, every other node
   (in practice the hand) -> `bb_hand_*`. The header comment at `CanInterface.h:21` now says this.
4. `FwUpdate::FW_VERSION` 6 -> 7; Jugglebot `BB_FW_VERSION_EXPECTED` 6 -> 7 (bridge stays 28).
5. **SDO readback guard NOT added.** It needs the ODrive S1 0.6.11 endpoint id of
   `axis0.config.can.input_torque_scale`, and no authoritative local source has it: neither repo carries an S1
   flat_endpoints file, and of the five firmware ELFs in `~/.cache/odrivetool/firmware/` (each embeds its endpoint
   tree) none matches the S1 0.6.11 ids proven on BB (`get_gpio_states` 700, `commutation_mapper.pos_abs` 488).
   The one with 726 / 283 is the Pro 0.6.11 (agrees with the tabled 283); `pDOE…` has pos_abs 488 but gpio 675,
   so it is some other build. Not guessed. Follow-up below.
6. Not done (separate, optional): hold closed loop in `HandTrajectoryStreamer.h:131-134` until the hand stops.

## Verification

- Build (2026-10-10): `pio run -e teensy40_can` and `-e teensy40`: SUCCESS, `firmware.hex` 460 937 bytes (both
  envs byte-identical, sha256 `0ec1f953…10da`). Disassembly: `sendInputPos`'s literal pool holds only `100.0f`
  (0x42c80000) and `1000.0f` (0x447a0000), chosen by a compare against `pitch_node_id_`. Not flashed.
- Jugglebot (2026-10-10, `JUGGLEBOT_BALLBUTLER_DIR=~/Desktop/BallButler-fw7`): scoped firmware / teensy_link /
  protocol_config tests 1698 passed, 1 skipped; `./run_tests.sh --full` PASS. `test_bb_fw_update_xref.py` against
  this repo's main (FW 6) fails until this branch merges — expected.
- Done (offline): FW 6 sitting bag `2026-10-10_14-49-14` and session `20261010T035151_835895Z` against the FW 5
  sessions `20261009T031319_612936Z` and `20261009T002142_931068Z`; scripts and CSVs in
  `~/bb_calibration_sessions/hand_jolt_20261010/`. Code pointers above checked in both repos.
- Pass criteria for the FW 7 sitting: hand rests at ~+0.46 mm (no pre-move); retry move < ~60 mm/s; stroke ends
  at ~236 mm; no spinouts; catch time back to ~+27 ms vs predicted. Then re-run the 40-throw validation.
- **Consequences for the accuracy work:** the 14:49 session's landings (mean −120 / −15 mm, RMS 128 mm) are this
  bug's; `settle_yaw_gauge.py`'s RE_PIN +2.406° from it is NOT applied (its own rotation-vs-translation fit reads the
  lateral error as a translation: slope −0.3 ± 0.6°, intercept +46 ± 11 mm), the pin stays 0.208°, and the aim
  correction is not refitted from it.

### Flash record (2026-10-10 17:20–17:21 local)

- From the merged `main` (`da3bfbe`), `pio run -e teensy40_can -t upload` in `ball_butler_main/` (the ini's
  `upload_command` → `~/Desktop/Jugglebot-skills/tools/teensy_link_bridge.py --fw-update … --target bb`), ROS launch
  down, BB IDLE (state 1) and visible on the bridge's BB bus at 457 frames/s beforehand. DATA 163 840 B in 25.4 s
  (pipeline depth 4; 26 BAD_SEQ rewinds in the first second, 0 window retries, 0 missing acks — FW 5 → 6 had 0
  rewinds), VERIFY OK crc32 0x1595BCA5, COMMIT OK, receipt **`Ball Butler FW version: 6 -> 7`**; 43 s end to end.
- Post-flash `scripts/bb_link_check.py`: bridge FW 28, BB bus 459 frames/s, BB state 1, `BB_YAW_ESTIMATE` 99.8 Hz with
  401/401 stamps paired; hand still reads −0.097 rev (−3.2 mm) — it stays on the stop until the next reload moves it,
  so the behavioural check (rest at ~+0.46 mm after a reload, no pre-throw kick, stroke to ~236 mm, no spinouts) is
  the owner's first sitting on FW 7.
- Jugglebot side merged and installed (`skill-stack` 456fd788: `bb_hand_*` 100/100, `bb_pitch_*` 1000/1000,
  `BB_FW_VERSION_EXPECTED` 7); `tests/firmware/test_bb_fw_update_xref.py` + `test_udp_protocol_xlang.py` against this
  tree: 49 passed; `generate_config.py --check`: CONFIG FRESH, no external drift.

## Open Questions / Follow-ups

- ~~The live node 8 scale read~~ — owner read `input_torque_scale` = 100 with odrivetool, 2026-10-10.
- ~~A similar borrowed constant elsewhere~~ — none: `InputScale::` had one use (`CanInterface.h:353-354`).
- Owed: flash FW 7 (receipt `FW version: 6 -> 7`), then the FW 7 throwing sitting against the pass criteria above.
- Follow-up: the S1 torque-scale SDO readback at hand arm, once the S1 0.6.11 endpoint id comes from an
  authoritative source (the S1 0.6.11 flat_endpoints.json, or an odrivetool dump of the drive's tree). Zero torque
  FF on a mismatch, keep FF + warn on no response.
- Follow-up: read node 7's `input_vel_scale` / `input_torque_scale` and refresh `bb_pitch_odrive_micro_config.json`
  (node_id 0 in the saved copy).
