---
title: Loud command-outcome channel + bb/throw ROS2 action (CMD_RESULT) — Phase 2
type: feature
date: 2026-06-23
status: tuned   # Phase 2 hardware-validated 2026-06-24 (5/5 loud-throw-action tests on BB+can-bridge)
phase: V1 reliability
related_entries:
  - 2026-06-21-throw-settle-gates-loud-on-aim   # Phase 1 (A + C + serial); this is its Phase 2
files_changed:
  - ball_butler_main/BallButlerConfig.h          # CanIds::CMD_RESULT alias
  - ball_butler_main/CanInterface.h              # publishCmdResult() decl
  - ball_butler_main/CanInterface.cpp            # publishCmdResult() — 8-byte frame, heartbeat idiom
  - ball_butler_main/StateMachine.cpp            # emit at every throw terminal point (reject/abort/OK)
  - ball_butler_main/HandTrajectoryStreamer.h    # Layer-C abort emits ABORTED_NOT_SETTLED + axis
  - ball_butler_main/protocol_config.h           # regenerated (CMD_RESULT id + command_types/outcomes)
  - zTesting/throw_testing/validate_loud_throw_action.py   # Phase-2 validation driver (asserts firmware outcomes via the action)
external_changes:
  - "Jugglebot: config/protocol_config.yaml (ball_butler.CMD_RESULT=0x7D5 + command_types/command_outcomes enums)"
  - "Jugglebot: config/generate_config.py (emit the two enums C++/Py; add Teensy_code_canbridge to extra_copies — was stale, a build blocker)"
  - "Jugglebot: config/generate_udp_protocol.py (uplink MsgType.CMD_RESULT=0x87 + CmdResultFrame raw-CAN relay struct)"
  - "Jugglebot: config/generated/* + copies (protocol_config.h/.py, udp_protocol.h/.py regenerated into both repos)"
  - "Jugglebot: controller/teensy_link/{protocol.py,__init__.py} (re-export CmdResultFrame + CMD_RESULT_FRAME_SIZE)"
  - "Jugglebot: Teensy_code_canbridge/{can_buses.h,can_buses.cpp} (CmdResultFrameRec SPSC ring + on_bb_rx CMD_RESULT branch)"
  - "Jugglebot: Teensy_code_canbridge/{telemetry.h,telemetry.cpp,Teensy_code_canbridge.ino} (cmd_result_uplink_step relay from task_telem; [canhealth] drop counter)"
  - "Jugglebot: jugglebot_interfaces/action/BallButlerThrowCmd.action + CMakeLists.txt (new action type)"
  - "Jugglebot: jugglebot/teensy_bridge_node.py (bb/throw ActionServer, MultiThreadedExecutor + ReentrantCallbackGroup, CMD_RESULT sub + RX-thread completion; speed==0 aim/track fast-path; retired bb/send_throw_command)"
  - "Jugglebot: jugglebot/ball_butler_node.py (bb/throw ActionClient; fire-and-forget + async outcome logging)"
  - "Jugglebot: jugglebot/can_node.py (stale BB-services comment refreshed)"
  - "Jugglebot: tests/ros/{conftest.py,test_teensy_bridge_node_bb.py,test_ball_butler_node.py} + tests/teensy_link/test_protocol_codec.py (action infra mocks, service→action migration, CMD_RESULT loopback + round-trip)"
commits:
  - <bb>      # BallButler-side firmware + regenerated protocol_config.h
  - <jb>      # Jugglebot-side protocol + bridge + host + tests
subsystem:
  - firmware
  - throwing
  - tooling
tags:
  - safety
  - throw
  - testing
---

# Loud command-outcome channel + bb/throw ROS2 action (CMD_RESULT) — Phase 2

## Summary

Phase 1 made BB throws *on-aim* (settle gates A + C) but the outcome was
**serial-only** — a firmware reject/abort never reached the host, and
`bb/send_throw_command` "success" only meant *frame queued at the bridge*. Phase
2 adds a **generic, reusable firmware→host outcome channel**: one `CMD_RESULT`
CAN message (0x7D5), relayed CAN1→UDP by the can-bridge, surfaced through a new
`bb/throw` ROS2 **action** whose result carries the firmware's terminal outcome
(OK / a `REJECTED_*` reason / `ABORTED_NOT_SETTLED`). The throw is the first
consumer; reload/calibrate/home can reuse the same mechanism. Code-complete; the
full Jugglebot suite is green (1746 passed, 1 xfailed). Hardware end-to-end
(flash BB + can-bridge, `ros2 action send_goal /bb/throw`) is the pending
operator step.

## Motivation

`StateMachine::requestThrow` rejects synchronously, but `executeThrow_` runs
later in `handleIdle_` *after* the RPC already acked — and the `BB_THROW` RPC
acks at the **can-bridge** (frame queued to CAN1), never at BB. So every
firmware-side reject/abort was invisible to ROS. Making throws "loud" needs a
BB→host frame the bridge explicitly relays, plus a host primitive that *awaits*
it — hence an action, not a service.

## Design

- **One generic `CMD_RESULT` frame** (8 bytes): `byte0=command_type`,
  `byte1=command_outcome`, `bytes2-3=detail0` (int16 LE), `bytes4-5=detail1`
  (int16 LE), `6-7` reserved. Not throw-specific.
- **Outcome encoding (decided at kickoff): outcome-as-extensions.** `command_outcomes`
  is a shared base enum (`OK/REJECTED/ABORTED/TIMEOUT/BUSY`, 0x00–0x0F) plus
  per-command extension values ≥0x20 carrying the *specific* reason
  (`THROW_REJECTED_NO_BALL`=0x20 … `THROW_ABORTED_NOT_SETTLED`=0x29). One field
  to switch on; the host derives category by range. `detail0/1` stay generic
  (for throws: detail0 = binding axis 0=YAW/1=PITCH/2=BOTH/-1=n/a, detail1 = a
  metric — lead shortfall ms / yaw error centideg). (Alternative considered:
  reason-in-detail0; rejected to keep the base enum the single switch surface.)
- **One outcome per throw, at the true terminal point.** Emitted at each
  `requestThrow`/`executeThrow_` reject, at the Layer-C abort, and — crucially —
  `OK` only at **completion** in `handleThrowing_` (not at arm), so an
  A-misprediction that aborts at C produces exactly one `CMD_RESULT`, never a
  premature OK.
- **No correlation token** — throws are serialized (one ball), so the host
  tracks a single outstanding goal per `cmd_type`.
- **Bridge relay mirrors the cone path** — a small SPSC ring filled in
  `on_bb_rx` (CMD_RESULT branch *before* the ODrive-decode fallthrough),
  drained by `cmd_result_uplink_step()` on `task_telem` → `udp_send_stream`.
- **Host: action + RX-thread completion.** `execute_callback` dispatches the
  RPC, then waits on a `threading.Event` (with `throw_time + 5 s` timeout) that
  the RX-thread `_on_cmd_result` sets. `goal_callback` rejects a second
  concurrent goal. The node moved to a `MultiThreadedExecutor` +
  `ReentrantCallbackGroup` so the result-wait never stalls the 100 Hz telemetry
  timers and a second goal can still be rejected promptly.
- **`bb/throw` is throw-OR-aim** (caught during bring-up). The retired service
  carried both throws (speed>0) and aim/track commands (speed=0 →
  `requestTracking`, which emits *no* `CMD_RESULT`). A speed=0 goal awaiting a
  result would hang until timeout (and the calibration-volley keepalive uses it),
  so the action treats speed=0 as a **fire-and-forget aim**: accepted without
  consuming the single-throw slot, dispatched, succeeded immediately. Only
  speed>0 enters the gate-and-await throw path.

## Implementation

Built in dependency order across ~5 components:

1. **Protocol** (`config/`): YAML id + two enum blocks; `generate_config.py`
   emits them (C++ namespaces + Python `IntEnum`) and — the **build-blocker
   fix** — adds `Teensy_code_canbridge/` to `extra_copies` (its
   `protocol_config.h` was hand-copied and stale, so it never had new ids).
   `generate_udp_protocol.py` adds the `CmdResultFrame` relay struct. Regenerated
   into both repos; `controller/teensy_link` re-exports `CmdResultFrame`.
2. **BB firmware**: `CanInterface::publishCmdResult()` (heartbeat frame idiom),
   wired at every throw terminal point; Layer-C abort emits from the streamer
   (it already owns the binding-axis decision).
3. **can-bridge**: `CmdResultFrameRec` ring + relay (verbatim mirror of the cone
   uplink), `cmd_result_fwd_drops` on the `[canhealth]` line.
4. **Host**: `BallButlerThrowCmd.action`; `teensy_bridge_node` action server;
   `ball_butler_node` swapped to an `ActionClient` (fire-and-forget send +
   async result-callback that logs the firmware outcome).
5. **Migration**: retired `bb/send_throw_command` (clean cutover); updated tests
   + conftest action mocks; added a `CMD_RESULT` loopback test and a codec
   round-trip.

## Verification

- **Builds:** BB firmware `pio run` teensy40 **and** teensy41 SUCCESS; can-bridge
  firmware `pio run` (teensy41) SUCCESS; `colcon build jugglebot_interfaces`
  generated `BallButlerThrowCmd` cleanly (Goal/Result fields confirmed).
- **Tests:** full Jugglebot suite **1746 passed, 1 xfailed** (`run_tests.sh`).
  New `tests/ros/test_teensy_bridge_node_bb.py`: goal-rejection-while-outstanding,
  a full **CMD_RESULT loopback** (FakeTeensy relays an OK frame → action
  succeeds), an abort path carrying reason+axis, dispatch-failure abort, a
  **timeout** path (BB silent → action times out, doesn't hang), and the
  speed==0 **aim fast-path** (completes immediately, doesn't await). Codec
  round-trip in `tests/teensy_link/test_protocol_codec.py`.
- **Hardware — VALIDATED 2026-06-24** (both Teensys flashed; nodes rebuilt;
  `validate_loud_throw_action.py` against live BB + can-bridge). **5/5**:
  - T1 ACCEPT (Δ≈0, lead 2.0 s) → `outcome=OK`, threw.
  - T2 REJECT (Δpitch 50°, lead 1.0 s) → `THROW_REJECTED_CANT_MAKE_LEAD`,
    `axis=PITCH`, shortfall 993 ms, no throw.
  - T3 ACCEPT (Δpitch 50°, lead 3.0 s) → `OK`, threw.
  - T4 REJECT (Δyaw 45°, lead 1.0 s) → `CANT_MAKE_LEAD`, `axis=YAW`, shortfall
    443 ms, no throw.
  - T5 concurrent second goal → rejected at the server; goal A still threw `OK`.
  The action result carried the firmware outcome + binding axis + shortfall in
  every case — the whole Phase-2 objective, now assertable end-to-end.
- **Finding — the test, not the firmware, was wrong first time.** An earlier run
  failed T4 (reported `axis=PITCH`, not the expected YAW). The heartbeat trace
  showed pitch sitting at the ~90° stow position post-reload: the open-loop
  track-hold hadn't slewed the slow pitch axis down to the pose, so PITCH
  genuinely *was* binding and the firmware named it correctly. Fix was in the
  driver — gate track-hold on `bb/heartbeat` until both axes are within 2° AND
  BB is in TRACKING (production-faithful) — after which T4 cleanly binds YAW.
  Reinforces the project's "suspect the instrument" rule: the loud channel
  reported real state; the harness's assumption was the bug.

## Open Questions / Follow-ups

1. **No correlation token** is by design (serialized throws) but means a stale
   `CMD_RESULT` arriving during a *new* throw's window could complete the new
   goal with the old outcome. Practically impossible (seconds between throws,
   one frame per throw), but if reload/calibrate/home ever overlap a throw,
   revisit (a 1-byte token fits if a detail field is freed).
2. **Result-wait timeout is `throw_time + 5 s`** — generous; if a high-speed
   throw's completion lags, widen the margin rather than the throw_time field.
3. **Next consumers** (reload/calibrate/home) only need their own `command_type`
   + extension outcomes + a thin action subclass — the relay, ring, executor,
   and RX-completion plumbing are all shared now.
4. **Pitch re-engage-from-stow latency** (observed in validation, not new):
   after a reload, pitch sits at the ~90° stow position and the slow axis took a
   few heartbeat cycles of track-hold to slew back on-aim. Production tracks
   continuously so it doesn't bite, but it is the same gap the dormant
   `PITCH_REENGAGE_RESERVE_S` (Phase-1 follow-up) would cover — size it from a
   measured re-engage latency if idle-pitch throws ever need it.
