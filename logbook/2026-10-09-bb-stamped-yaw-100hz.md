---
title: Stamped 100 Hz yaw — BB FW 6 sends YAW_ESTIMATE (0x7D8) for /bb/axis_estimates
type: feature
date: 2026-10-09
status: in-progress
related_entries:
  - 2026-10-09-bb-local-calibration-result
files_changed:
  - ball_butler_main/CanInterface.cpp
  - ball_butler_main/CanInterface.h
  - ball_butler_main/Proprioception.cpp
  - ball_butler_main/Proprioception.h
  - ball_butler_main/ball_butler_main.ino
  - ball_butler_main/BallButlerConfig.h
  - ball_butler_main/FwUpdate.h
  - ball_butler_main/protocol_config.h
  - ball_butler_main/hardware_config.h
  - logbook/2026-10-09-bb-stamped-yaw-100hz.md
  - logbook/INDEX.md
external_changes:
  # Jugglebot branch bb-stamped-yaw-100hz (Jugglebot logbook/2026-10-09-bb-stamped-yaw-100hz.md).
  - "Jugglebot: config/protocol_config.yaml + config/generate_config.py (BallButlerCanId::YAW_ESTIMATE 0x7D8, YawEstimateEncoding::vel_res_dps 0.1) + regenerated protocol_config.{h,py} copies"
  - "Jugglebot: config/generate_udp_protocol.py + regenerated udp_protocol.{h,py}, docs/teensy-udp-protocol.md (additive UDP BB_YAW_ESTIMATE 0x93, 24 B; PROTOCOL_VERSION stays 9)"
  - "Jugglebot: ros_ws/src/jugglebot/Teensy_code_canbridge/{ball_butler_state.h,ball_butler_state.cpp,can_buses.cpp,telemetry.cpp,canbridge_config.h} (decode 0x7D8, emit BB_YAW_ESTIMATE paired by t_bridge_us; FW 27 -> 28)"
  - "Jugglebot: ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py (pair + publish bb_yaw on /bb/axis_estimates)"
  - "Jugglebot: ros_ws/src/jugglebot/jugglebot/can/ball_butler.py (BallButlerYawEstimate, Python mirror of the CAN frame)"
  - "Jugglebot: teensy_link/{__init__,protocol,rpc_args}.py (exports; EXPECTED_BRIDGE_FW_VERSION 28, BB_FW_VERSION_EXPECTED 6)"
  - "Jugglebot: ros_ws/gui/js/udp-traffic.js, tests/ros/test_teensy_bridge_node_bb_yaw.py, tests/ros/test_ball_butler.py, tests/teensy_link/test_protocol_codec.py, tests/firmware/{test_udp_protocol_xlang,test_bb_fw_update_xref}.py"
subsystem:
  - firmware
  - calibration
tags:
  - mocap
  - testing
---

# Stamped 100 Hz yaw — BB FW 6 sends YAW_ESTIMATE (0x7D8) for /bb/axis_estimates

## Summary

BB's yaw reached the Jetson only in the 10 Hz heartbeat. That value is unstamped and lags mocap by a per-session 78–174 ms (`~/bb_calibration_sessions/bb_placement_reinvestigation_20261009/REPORT.md` §Q1, §Q4), so the pose calibration had to fit a latency every session. BB FW 6 now also sends its yaw on CAN1 once per fresh 150 Hz sample. The can-bridge (Jugglebot, FW 28) forwards it, stamped at sample time, as a third name `bb_yaw` (deg, deg/s) on `/bb/axis_estimates` at 100 Hz. The heartbeat is unchanged. Both firmwares compile; **nothing was flashed**. The status stays `in-progress` until the owner flashes the boards and checks the topic. The design, the protocol discussion and the bridge/host side are in the Jugglebot entry of the same name.

## Motivation

See the Summary. The REPORT ranks stamped yaw in `/bb/axis_estimates` as the single biggest robustness gain for the per-session pose procedure. The lag fit then becomes a check.

## Design

**Pitch/hand already arrive this way.** They are ODrives (nodes 7, 8): the bridge hears their 1 kHz `Get_Encoder_Estimates` directly on CAN1 and snapshots them at 100 Hz. Yaw is BB's own AS5047P read in the 150 Hz `YawAxis` ISR, so BB has to put it on the bus.

**CAN frame `BallButlerCanId::YAW_ESTIMATE` = 0x7D8** (generated from Jugglebot `config/protocol_config.yaml`), 8 bytes LE:

| bytes | type | content |
|---|---|---|
| 0–3 | f32 | `yaw_deg`: the same `Proprioception` yaw the heartbeat reports, **unwrapped and untruncated** (the heartbeat wraps to [0, 360) and truncates to 0.01°) |
| 4–5 | i16 | yaw velocity, counts of `YawEstimateEncoding::vel_res_dps` = 0.1 deg/s (the YawAxis EMA velocity × 360) |
| 6–7 | u16 | sample age at TX, µs: `micros64()` now minus the ISR's sample stamp, saturating |

- **Rate.** One frame per fresh sample (deduplicated on the sample timestamp): ~150 Hz, about 1.7 % of the 1 Mbit bus.
- **Priority.** It is the lowest-priority BB id, so it never outranks ODrive traffic. At worst an in-progress frame delays a higher-priority one by one frame time (~0.1 ms).
- **Failure mode.** `sendRaw` is non-blocking (FlexCAN TX queue 16); a full queue drops the frame.

## Implementation

- **`Proprioception`.** It now also stores the yaw velocity: `setYawPV`/`getYawPV`, and `ProprioceptionData::yaw_vel_rps`. The ISR callback in the `.ino` calls `setYawPV`. Its velocity was previously discarded with the note "store it if ever needed".
- **`CanInterface::maybePublishYawEstimate_()`.** It is called from `loop()` after the heartbeat.
- **`CanIds::YAW_ESTIMATE`.** Added in `BallButlerConfig.h`.
- **`FwUpdate.h FW_VERSION` 5 → 6.** This is the flash receipt; Jugglebot `BB_FW_VERSION_EXPECTED` is 6.
- **Headers regenerated from Jugglebot `skill-stack` + this change, into this worktree only.** The owner's `~/Desktop/BallButler` was read, never written. The reconciliation summary is below.

### Regenerated headers vs `main` and vs the owner's working tree

- **`protocol_config.h`**
  - **vs the owner's uncommitted tree:** only this change: `YAW_ESTIMATE = 0x7D8` and the `YawEstimateEncoding` namespace. The owner's `hand_tor` 1000, `can_input_torque_scale` = 283 and the removed `TRAJ_CMD` are identical here.
  - **vs `main`:** those three owner edits plus this change.
- **`hardware_config.h`**
  - **vs the owner's tree:** exactly one line, `BBGeom::YAW_S_OFFSET_MM` −105.65 → **+105.65**. This is Jugglebot's 2026-10-09 positive-s fix, generated after the owner last regenerated.
  - **vs `main`:** the owner's 94-line platform/trajectory delta plus that line. All of it is Jugglebot-platform constants.
  - **Effect on BB firmware: none.** The baseline compile of these headers with the unmodified BB sources succeeded. No BB source reads `YAW_S_OFFSET_MM`; the Jugglebot entry for that fix says "no firmware reads s". The `TeensyTraj` namespace that `main` still carries is gone in both regenerated copies, and nothing in `ball_butler_main` references `TeensyTraj::`.
- **To reconcile,** the owner can discard their two uncommitted header edits in favour of this branch's copies. The branch versions contain them.

## Verification

- 2026-10-09, `pio run -e teensy40_can` (no `-t upload`) in `ball_butler_main/`: **SUCCESS** (`firmware.hex` built). `arm-none-eabi-nm` shows `CanInterface::maybePublishYawEstimate_()`, `Proprioception::setYawPV` and `getYawPV` in the image. The regenerated headers alone, before the firmware edits, also built: **SUCCESS**.
- The bridge/host gate triple is in the Jugglebot entry. BB firmware has no ROS test gate; the compile is its verification.

## Flashing (owner — NOT executed)

1. **BB first, over CAN, through the current can-bridge FW 27.** Bring ROS launch DOWN; BB must be IDLE/ERROR. From the merged BallButler tree, `ball_butler_main/`: `pio run -e teensy40_can -t upload`. Expect `FW version: 5 -> 6`. The upload command resolves `$PROJECT_DIR/../../Jugglebot/tools/teensy_link_bridge.py`, i.e. the owner's `~/Desktop/Jugglebot` checkout, whose BB FW pin then reads 5 (advisory only). The FW 27 bridge drops 0x7D8 harmlessly, so the stack keeps working.
2. **Then the can-bridge over USB.** This is the Jugglebot entry's recipe. Never use a bare `pio run -t upload` or a loader `-s` with both Teensys attached. Run the waiting loader, then a 134-baud touch of `/dev/serial/by-id/usb-Teensyduino_USB_Serial_19942350-if00` only. The teensy-hub is ttyACM0 and the catching cone is ttyACM1.
3. **Check after the host deploy.**
   - `ros2 topic hz /bb/axis_estimates` reads ~100 Hz.
   - `ros2 topic echo /bb/axis_estimates --once` shows `[bb_pitch, bb_hand, bb_yaw]`, with `bb_yaw` ≈ heartbeat yaw within 0.01°.
   - Two names only means BB is still on FW 5, or dark.

USB recovery stays `pio run -e teensy40` + `/home/jetson/bin/teensy_loader_cli --mcu=TEENSY40 -w -v` with a 134-baud touch of BB's by-id path (serial 15970570).

## Open Questions / Follow-ups

- The hardware check above, then `resolved`.
- Switch the pose-calibration fit (Jugglebot `mocap_node`) from heartbeat yaw to `bb_yaw`, keeping the lag fit as a check. Measure `yaw_age_us` (expected mean ≈ 4 ms, an inference from the rates).
