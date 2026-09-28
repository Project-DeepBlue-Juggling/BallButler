---
title: Firmware update over CAN — the Platform receiver ported to Ball Butler, with a safety park
type: feature
date: 2026-09-28
status: in-progress
phase: "BB FW 1 / can-bridge FW 21"
related_entries:
  - 2026-06-23-loud-command-outcome-channel-cmd-result
files_changed:
  - ball_butler_main/FwUpdate.h (new)
  - ball_butler_main/FwUpdate.cpp (new)
  - ball_butler_main/CanInterface.cpp
  - ball_butler_main/BallButlerConfig.h
  - ball_butler_main/ball_butler_main.ino
  - ball_butler_main/platformio.ini
  - ball_butler_main/protocol_config.h (generated)
  - ball_butler_main/hardware_config.h (generated)
external_changes:
  - "Jugglebot: config/protocol_config.yaml (BallButlerCanId FW_UPDATE_CMD 0x7D6 / FW_UPDATE_REPLY 0x7D7)"
  - "Jugglebot: config/generate_udp_protocol.py (RPCs BB_FW_BEGIN/DATA/VERIFY/COMMIT/INFO 0x5B-0x5F)"
  - "Jugglebot: ros_ws/src/jugglebot/Teensy_code_canbridge/{platform_relay,can_buses,rpc,canbridge_config} (CAN1 relay + reply routing, FW 20 -> 21)"
  - "Jugglebot: teensy_link/rpc_args.py (PARKING/PARK_FAILED statuses, FW_OP_INFO, BB_FW_VERSION_EXPECTED)"
  - "Jugglebot: tools/teensy_link_bridge.py (--fw-update --target bb)"
  - "Jugglebot: tests/firmware/test_bb_fw_update_xref.py + native/teensy_link tests"
  - "Jugglebot logbook: 2026-09-28-bb-firmware-over-can-relay"
  - "Jugglebot commit: a229edd (mvp-trajectory-bringup)"
subsystem:
  - firmware
  - tooling
tags:
  - safety
  - testing
---

# Firmware update over CAN — the Platform receiver ported to Ball Butler, with a safety park

## Summary

Ball Butler can be flashed over CAN through the can-hub:
`pio run -e teensy40_can -t upload` builds the image and hands it to Jugglebot's
`tools/teensy_link_bridge.py --fw-update --target bb`, which streams it through the
can-bridge's new `BB_FW_*` relay (CAN1 0x7D6/0x7D7) into `ball_butler_main/FwUpdate.cpp`.
The receiver is the Platform Teensy's (Jugglebot, flashed Platform FW 6 over CAN on
2026-09-09), ported nearly verbatim. The one real addition is a **park** before any
flash is touched: yaw e-stopped, pitch raised to ≥ 80° if it is lower, then the pitch
and hand ODrives set IDLE. Every session ends in a reboot. **BB FW 1 and can-bridge
FW 21 are BUILT, NOT FLASHED**; the park is the part only hardware can prove.

## Motivation

The owner wanted BB flashable the way the Platform is, re-using that system. Both
boards sit behind the same can-hub Teensy. BB's USB still works, so this adds a
convenient single flash path and USB stays the recovery path.

## Design

**Re-used unchanged:** the wire contract (BEGIN/DATA/VERIFY/COMMIT, 5 B per DATA
frame, ACK every 16th, BAD_SEQ rewind), staging above the running image up to the
0x601F0000 EEPROM-emulation reserve, per-sector read-back (`FLASH_ERR`), CRC-32 +
identity marker at VERIFY, and the low-to-high interrupts-off copy + `SCB_AIRCR`
reset. Same MCU, same core primitives, same linker symbol.

**The park (owner-agreed 2026-09-28).** A sector erase disables interrupts for up to
~400 ms. BB's yaw PID runs in an IntervalTimer ISR, so during an erase the yaw motor
would hold its last PWM with no control loop. The Platform never had this problem
because all its motors are ODrives. So the first BEGIN accepted in IDLE or ERROR
latches a session and parks:
1. `yawAxis.estop()`: PWM latched at 0, which stays safe while the ISR is blocked.
2. If pitch is below `PITCH_MIN_STOW_DEG` (80°): pitch goes CLOSED_LOOP and moves
   to `PITCH_DEG_HOME` (90°). Owner: below ~70° an IDLE pitch drops.
3. Once pitch is ≥ 80° and its trajectory is done (or it is already IDLE): pitch
   and hand ODrives are set IDLE, re-requested every 200 ms until both heartbeats
   report IDLE.

Until parked, BEGIN answers `PARKING` (detail = pitch in signed centidegrees) and the
host re-sends it. The park times out after 10 s → `PARK_FAILED` and a reboot. Pitch
angle and freshness come from the ODrive encoder estimate on the **monotonic** clock
(`axisPVMonoAgeUs`), not Proprioception's wall-clock stamp, which the time-sync
slews. The TOO_BIG capacity check runs *before* the park, so the robot never parks
for an image that cannot fit.

**Every session ends in a reboot (owner's call).** COMMIT reboots into the new image.
An abandoned session (60 s without a command: a `--verify-only` run, a crashed host,
a failed VERIFY) and a failed park reboot too. BB therefore always comes back through
BOOT with every axis freshly initialised, never from a half-restored park, and the
session is a one-way latch. While it is open, `loop()` freezes the state machine
and streamer (`pitch.loop()` keeps running, since the park needs it), CAN
throw/reload/reset/calibrate commands are dropped, and serial commands are ignored.

**Identity.** `FwUpdate::FW_NAME = "ballbutler-main"`, `FW_VERSION = 1`, and a boot
banner line. `INFO` (opcode 0x05) returns `FW_VERSION`. BB had no firmware version
on any wire before this; it is the receipt that a COMMIT landed.

## Implementation

- `FwUpdate.{h,cpp}`: receiver + park; hooked from `CanInterface::handleRx_` (0x7D6,
  before every other dispatch) and `loop()` (`FwUpdate::tick()`); `attach()` first
  thing in `setup()`, so a frame arriving during the boot waits is safe.
- `BallButlerConfig.h`: `CanIds::FW_UPDATE_CMD/REPLY`.
- `platformio.ini`: `[env:teensy40_can]` (custom upload → the tool with `--target bb`)
  and a "FLASH OVER CAN" header section. The `teensy40` USB env is unchanged.
- The generated headers were regenerated from Jugglebot `mvp-trajectory-bringup`.
  This replaced an uncommitted resync from the `skill-stack` worktree
  (`hand_tor = 1000`, no `TRAJ_CMD`), which that worktree's generator reproduces.
  Against `HEAD` the result is behaviour-neutral for BB: `InputScale::hand_tor`
  stays 100, and none of the other changed namespaces are used by BB.

## Verification

- **Build** (`pio run -e teensy40` and `-e teensy40_can`, 2026-09-28): both SUCCESS.
- **Image** (`teensy_link_bridge.py --fw-update .pio/build/teensy40/firmware.hex
  --dry-run --target bb`, 2026-09-28): 163840 B, crc32 `0x7AE1BDEB`, identity OK;
  `--target platform` refuses it (no `jugglebot-platform` marker).
- **Host + relay** (Jugglebot, 2026-09-28): an end-to-end loopback through the real
  RPC client against a simulated BB receiver (PARKING ×2, a lost DATA frame
  recovered by rewind, VERIFY, COMMIT, INFO 1 → 2), 10/10 runs. Native relay tests
  and a cross-repo xref pin this repo's `FwUpdate.cpp` status table, opcodes,
  `FW_NAME` and `FW_VERSION` to the host's. Details and the full-gate line are in
  the Jugglebot entry.
- **Not yet on hardware.** The park sequence, the heartbeat-confirmed IDLE and the
  real transfer are unexercised.

## Open Questions / Follow-ups

- **Bring-up sitting (operator):** (1) USB-flash can-bridge FW 21; (2) USB-flash BB
  FW 1 (`pio run -e teensy40 -t upload`), the first image with the receiver; (3) with
  the ROS launch down: `--dry-run`, then `--verify-only` (watch the park: pitch rises
  if below 80°, then pitch + hand go IDLE, yaw limp; BB reboots ~60 s after VERIFY);
  (4) bump `FW_VERSION` to 2 (and Jugglebot `BB_FW_VERSION_EXPECTED`) and
  `pio run -e teensy40_can -t upload`. Expect `Ball Butler FW version: 1 -> 2`.
- The common park is from IDLE, where BB already rests pitch IDLE at ≥ 80°, so it
  only idles the hand. A park from ERROR with a faulted pitch ODrive below 80°
  cannot raise pitch; it answers PARK_FAILED and reboots, and USB is the path then.
