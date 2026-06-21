---
title: Throw settle-gates — loud + guaranteed-on-aim throws (A predictive / B loud channel / C fire-time confirm)
type: feature
date: 2026-06-21
status: in-progress
phase: V1 reliability
files_changed:
  - ball_butler_main/StateMachine.cpp
  - ball_butler_main/HandTrajectoryStreamer.h
  - ball_butler_main/PitchAxis.h
  - ball_butler_main/BallButlerConfig.h
external_changes: []   # Phase 2 will touch Jugglebot (protocol, can-bridge, teensy_bridge_node, ball_butler_node)
commits:
  - 352bb44   # Phase 1: A + C + serial logging
  - 24e0906   # carry-forward teensy_reboot + 2026-05-24 entry
subsystem:
  - firmware
  - throwing
tags:
  - safety
  - timing
  - throw
related_entries:
  - 2026-05-24-bb-axis-settled-gate   # superseded original gate (settle-by-release, in executeThrow_, silent)
---

# Throw settle-gates — loud + guaranteed-on-aim throws

## Summary

BB throws had two failure modes: (1) the hand could fire while pitch/yaw were
still slewing, releasing the ball **off-aim**; and (2) a throw could be
**silently dropped** by firmware checks that run *after* `requestThrow` already
returned "queued" to the host — the ball just never fires, no feedback.

A design review reframed bad throws into three categories:

| Category | Example | Handled before this work? |
|----------|---------|---------------------------|
| **Impossible** (infeasible) | out of reach; pitch/yaw/speed/height beyond limits | ✅ loud, host-side (`solve_throw_local`) |
| **Not-ready** | no ball, no time-sync, PV stale | ⚠️ partly (sync checks loud, `executeThrow_` checks silent) |
| **Un-aimable-in-time** | feasible, but scheduled too soon to slew the platform | ❌ nothing |

The fix is three layers: **A** a predictive *settle-before-wind-up* gate
(fail-fast), **C** a fire-time settled confirm via the encoders (the real
guarantee), and **B** a loud outcome channel back to the host (visibility).
**Phase 1 (this entry) lands A + C + serial-only outcomes, firmware-only.**
Phase 2 builds the reusable host-facing loud channel + a ROS2 action.

## Motivation

"Impossible" throws are already rejected *loudly* host-side: `solve_throw_local`
(Jugglebot `can/throw_ballistics.py`) enforces pitch/yaw/speed/height limits and
raises `ValueError` → `ball_butler_node` returns `success=False` with a message.
The remaining gap is the **un-aimable-in-time** case plus the architectural fact
that the firmware's own rejections live in `executeThrow_`, which runs
asynchronously in `handleIdle_` *after* the host was told "queued" — so they are
invisible. "Loud + on-aim" therefore needs **both** a timing gate and a
feedback channel.

## Design

- **Three layers, one rule.** A predicts the axes will be settled **before the
  hand wind-up begins**; C confirms it with the encoders at the last instant
  before any hand motion; B reports the outcome. They compose: A = fail-fast,
  C = guarantee, B = visibility.
- **Why settle-before-wind-up (not -before-release).** It keeps an *accepted*
  throw trustworthy: A and C share the same rule, so C only ever fires on an A
  *misprediction*. Cost is ~0.6 s more required lead — free at the ~2.5 s
  operating leads, and the cleaner physical model (platform locks on target,
  *then* the hand moves).
- **C uses the direct sensor, not a prediction** (pitch `trajectory_done` + yaw
  error/rate) — the project's "suspect the instrument" principle applied at the
  one moment that matters.
- **Architecture finding that scoped Phase 2.** Host↔BB is three-tier:
  host ⇄ **UDP** ⇄ can-bridge Teensy ⇄ **CAN1** ⇄ BB. The `BB_THROW` RPC acks at
  the *bridge* (frame transmitted), never at BB — the root cause of silent
  throws. A new BB→host result frame must be **explicitly relayed** by the
  bridge, so the loud channel spans the CAN protocol + the UDP protocol + BB
  emit + bridge relay + host action (5 components, not 2).

## Implementation (Phase 1)

- **Layer A** — relocated the predictor (from the superseded 2026-05-24 gate,
  commit `2f9d72b`) into `StateMachine::requestThrow` so the reject is
  **synchronous**, and changed the rule to
  `lead ≥ max(pitch_settle, yaw_settle) + WINDUP_DURATION + SCHEDULE_MARGIN`.
  Closed-form trap-traj pitch estimator (asymmetric accel/decel, reads
  `pitch_.getTraj()`) + constant-rate yaw estimator. `PitchAxis::getTraj()`
  accessor added.
- **Layer C** — one-shot confirm in `HandTrajectoryStreamer::tick()` evaluated
  when frame 0 is due: pitch `trajectory_done` (cached heartbeat) AND
  `|yaw_err| < 1.0°` AND `|yaw_rate| < 3.0°/s` (from `YawAxis::readTelemetry()`).
  Not settled → **abort with zero frames sent**; ball retained; `handleThrowing_`
  returns to IDLE (no reload). Reads only cached state → **zero added throw
  latency**.
- **`AxisSettleCfg`** (firmware-local tunables): `YAW_TRAVERSE_DEG_PER_S = 60`,
  `WINDUP_DURATION_S = 0.6`, `YAW_ERR_TOL_DEG = 1.0`, `YAW_RATE_TOL_DPS = 3.0`.
- **B-on-serial** — A prints `[SM] Throw rejected: axes can't settle before
  wind-up …`; C prints `[Gate] Throw ABORTED (axes not settled) …`. (The CAN
  `THROW_RESULT` message + bridge relay + action are Phase 2.)

## Verification

- **Build:** `pio run` passes both `teensy40` / `teensy41`. (Commit `352bb44`.)
- **Hardware (PENDING — operator bench step):** flash → `pio device monitor` on
  BB serial → issue explicit-target throws via the existing
  `bb/send_throw_command` (needs a ball loaded + the stack up for time-sync):
  1. Δyaw=Δpitch=0, normal lead → `[SM] Throw armed`.
  2. Δpitch=+60°, lead=1 s → A reject (`need ≈2115 ms` = 1415 settle + 600
     wind-up + 100 margin, have 1000).
  3. Same, lead=3 s → armed.
  4. Δyaw=+90°, lead=1 s → A reject.
  5. Force not-settled at fire (marginal timing or tightened tol) → C aborts:
     **no wind-up frame sent**, ball retained, `[Gate] Throw ABORTED`, clean
     return to IDLE.
  - Fallback (pre-registered): if A over-rejects throws the hardware would make,
    raise `YAW_TRAVERSE_DEG_PER_S` 60 → 90 and re-flash.

## Open Questions / Follow-ups

- **Phase 2 (design locked, not built): generic command-outcome channel.** The
  throw is the first of several ops (reload, calibrate, home) that want loud
  feedback, so Phase 2 is a *reusable* mechanism, not throw-specific:
  - one generic `CMD_RESULT` CAN message — `cmd_type` + a **shared base outcome
    enum** (OK / REJECTED / ABORTED / TIMEOUT / BUSY) + per-command extensions +
    2 **generic** detail int16s;
  - **one outstanding goal per `cmd_type`** (no correlation token — throws are
    serialized);
  - one bridge relay path (CAN1 → UDP), modelled on the cone-frame relay;
  - a reusable "command action server" base in `teensy_bridge_node` that
    dispatches the RPC, registers the pending goal by `cmd_type`, and completes
    it from the **serial/UDP RX thread** on `CMD_RESULT`;
  - `teensy_bridge` moves to a `MultiThreadedExecutor` + `ReentrantCallbackGroup`
    so result-waits never stall the 100 Hz telemetry;
  - first consumer: `BallButlerThrowCmd.action` **replacing**
    `bb/send_throw_command`; migrate `ball_butler_node`.
- `WINDUP_DURATION_S = 0.6 s` is a fixed figure (`requestThrow` is pre-plan and
  can't read the trajectory's actual wind-up). Revisit if it mispredicts.
- **Phase-1 hardware validation of A + C is still pending** (above).
