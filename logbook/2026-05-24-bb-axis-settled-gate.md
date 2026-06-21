---
title: Axis-settled lead-time gate in executeThrow_ — reject throws that fire before pitch/yaw reach target
type: bugfix
date: 2026-05-24
status: resolved
related_plan: ""
files_changed:
  # Modified
  - ball_butler_main/StateMachine.cpp
  - ball_butler_main/PitchAxis.h
  - ball_butler_main/BallButlerConfig.h
  # New
  - logbook/README.md
  - logbook/INDEX.md
  - logbook/2026-05-24-bb-axis-settled-gate.md
commits:
  - 2f9d72b  # fix(firmware): reject throws before yaw/pitch reach target (axis-settled gate)
subsystem:
  - firmware
  - state-machine
tags:
  - bugfix
  - timing
  - throw
  - kinematics
---

# Axis-settled lead-time gate in `executeThrow_`

> **Superseded (2026-06-21)** by
> [throw-settle-gates-loud-on-aim](2026-06-21-throw-settle-gates-loud-on-aim.md).
> That work keeps this predictor but (a) relocates it into `requestThrow` so the
> reject is *synchronous/loud*, (b) changes the rule from settle-by-**release** to
> settle-before-**wind-up**, and (c) adds a fire-time encoder confirm (Layer C).
> The symptom/diagnosis/kinematics below remain accurate; the *placement and
> acceptance rule* are what changed. Kept for history.

## Symptom

Operator reported on hardware: *"BB tends to throw very early; the pitch and
yaw axes are still finding their positions by the time the hand throws."*
Every throw with a different aim from the previous one released at the wrong
angle — landing direction non-deterministic.

## Diagnosis

[`StateMachine::executeThrow_`](../ball_butler_main/StateMachine.cpp) (in the
pre-fix version) does, in order:

1. Command yaw + pitch to their targets (motors start moving).
2. Plan the hand trajectory.
3. Check lead time against **only** the hand wind-up.
4. Arm the streamer.

There is no check that yaw + pitch will be at their target by the time the
hand fires the ball. The pitch ODrive runs `trap-traj` configured at
`traj_vel_rps = 1.0`, `traj_accel_rps2 = 0.5`, `traj_decel_rps2 = 0.25` (from
`hardware_config.h`). A 60° pitch change is a triangular profile of ~1.42 s
total; a 30° change ~1.00 s; a 10° change ~0.58 s. Hand wind-up only needs
~600 ms, so any throw scheduled with ~1.0 s lead but a meaningful pitch change
passes the existing lead-time check, the hand fires at +600 ms, and the ball
releases while pitch is still mid-traverse. Yaw (PWM-controlled, custom PID)
has no firmware-side time-to-target either, with the same failure mode.

## Fix

Insert a new **step 4b** in `executeThrow_` (between planning and
arming) that predicts each axis's traverse time at queue time and rejects
the throw if the available lead is shorter than
`max(pitch_settle, yaw_settle) + SCHEDULE_MARGIN`.

Two helpers, both anonymous-namespace free functions in `StateMachine.cpp`:

- **`estimatePitchTraverseTimeUs(current_deg, target_deg, traj)`** — closed-form
  trap-traj kinematics with **asymmetric** accel/decel. Computes
  `v_peak = sqrt(2·Δrev / (1/a_acc + 1/a_dec))`; if `v_peak ≤ v_max` returns
  triangular time `v_peak/a_acc + v_peak/a_dec`, otherwise the trapezoidal
  decomposition `t_acc + d_cruise/v_max + t_dec`. Reads the live `Traj` from
  `PitchAxis::getTraj()` (new one-line accessor) so any runtime
  `setTrajLimits()` change is honoured.

- **`estimateYawTraverseTimeUs(current_deg, target_deg)`** — conservative
  constant rate. `t = |Δdeg| / YAW_TRAVERSE_DEG_PER_S`. The rate lives in a
  new `AxisSettleCfg` namespace in `BallButlerConfig.h`; initial value
  **60 deg/s**, tunable.

Defensive: both estimators return `INFINITY` if their inputs are non-positive,
which causes the gate to reject the throw rather than silently accept.

Rejection log mirrors the existing hand-lead reject style:
```
[SM] Throw rejected: yaw/pitch settle needs N ms, have M ms (pitch=… ms, yaw=… ms)
```

## Discussion

**Why predictive estimation and not "wait until trajectory_done"?** The check
runs at *queue time* (`executeThrow_` is called once when a `THROW_CMD`
arrives, then the streamer is armed against an absolute wall-clock fire
time). There's no opportunity to observe actual settling — the firmware
has to *predict* whether the axes will be settled by `pending_throw_wall_us_`.
A wait-loop would change the streamer-arming contract and is out of scope.

**Why asymmetric trap-traj kinematics?** The original prompt's worked
example assumed symmetric accel = decel = 0.5 and arrived at ~1.6 s for 60°.
The actual config has `decel = 0.25` (half of accel), which lengthens the
profile asymmetrically. Using the symmetric formula would underestimate
the move time and let some marginal throws slip through. The asymmetric
formula adds three lines of code and matches the ODrive's behaviour exactly.

**Why a constant rate for yaw, not an empirical step-size map?** Yaw is
PWM-driven through a custom PID loop in `YawAxis.cpp` — no `v_max`/accel
pair to read like the pitch case. Bench-measuring a polynomial would be
more accurate but is out-of-scope work that the operator can choose to
spend later if 60 deg/s proves too conservative. The constraint from the
prompt was *"lean toward safe-rejection"* — a low (slow) rate means
longer estimated time means harder to satisfy means more rejections,
which is the safer failure mode. Operator can tune the constant up if
it rejects throws the hardware would in fact have made.

**Why `AxisSettleCfg` and not `YawDefaults`?** Every constant in
`YawDefaults` is sourced from `BBYaw::` in the auto-generated
`hardware_config.h`. Adding a non-sourced firmware-local constant there
would break that contract. A separate namespace keeps the codegen
boundary clean and signals "this is firmware-local, edit here, no
Jugglebot-side regen needed".

**Why double-applied `SCHEDULE_MARGIN_S` (once in axis gate, once in hand
gate)?** The two gates check independent conditions — *will the hand be
ready in time?* and *will the platform be aimed in time?* — and each
deserves its own margin against jitter / clock skew / unmodelled effects.
The hand gate's margin protects the streamer arming; the axis gate's
margin protects the geometric aim. Sharing the margin would mean the
worse-case axis (typically pitch) consumes the budget that should
protect the hand, or vice versa.

## Verification

**Compile**: Teensy 4.1 firmware via PlatformIO (operator to flash).
The two helpers are `static`-equivalent (anonymous namespace) and only
add a few hundred bytes of flash. No new public API.

**Hardware acceptance criteria** (operator to run; pipeline:
`ros2 launch jugglebot catching_cone_test.launch.py` then issue throws
via the GUI or `bb/send_throw_command`):

1. **No regression** — TRACK to current pose, fire throw at current
   `(yaw, pitch)` with normal lead time. Expected: `[SM] Throw armed: …`
   with no settle-gate rejection (Δyaw=Δpitch=0 → required = margin =
   100 ms, ≪ typical hand-windup lead of ~600 ms).
2. **Settled gate works** — issue throw with `Δpitch = +60°` and
   `throw_wall_us = now + 1000 ms`. Expected:
   `[SM] Throw rejected: yaw/pitch settle needs ~1515 ms, have ~1000 ms`
   (the breakdown shows `pitch≈1415 ms, yaw≈0 ms`).
3. **Same throw with longer lead** — same `Δpitch = +60°` with
   `throw_wall_us = now + 3000 ms`. Expected: `[SM] Throw armed: …`.
4. **Yaw constant-rate gate** — issue throw with `Δyaw = +90°`,
   `Δpitch = 0`, `lead = 1000 ms`. Expected: rejected with
   `yaw≈1500 ms, pitch≈0 ms` (90/60 s = 1.5 s).
5. **Combined max-wins** —
   (a) `Δpitch = 10°, Δyaw = 80°`, lead = 1000 ms → reject; gate log
       shows yaw dominating (`yaw≈1333 ms > pitch≈577 ms`).
   (b) `Δpitch = 60°, Δyaw = 10°`, lead = 1000 ms → reject; gate log
       shows pitch dominating (`pitch≈1415 ms > yaw≈167 ms`).

If criterion 4 or the yaw side of criterion 5 reject throws that the
operator believes the hardware would in fact have made, raise
`AxisSettleCfg::YAW_TRAVERSE_DEG_PER_S` in `BallButlerConfig.h` (try 90
deg/s next) and re-flash. If criterion 2 *accepts* the throw, lower the
constant or revisit the pitch trap-traj config in
`hardware_config.yaml`.

## Related

- Original prompt and design: [plans/active/bb-firmware-axis-settled-check-prompt.md](../../Jugglebot/plans/active/bb-firmware-axis-settled-check-prompt.md) (in the Jugglebot repo).
- Throw-director and live-integration arc on the Jugglebot side: [Jugglebot/logbook/2026-05-23-throw-director-and-cone-live-integration.md](https://github.com/Project-DeepBlue-Juggling/Jugglebot/blob/refactor/logbook/2026-05-23-throw-director-and-cone-live-integration.md) — the host-side workflow that surfaced this firmware bug.
