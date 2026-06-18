---
title: "Release-lag root cause confirmed in firmware — two decel-zero mechanisms cancel each other; fix is two lines in HandPathPlanner"
type: investigation
date: 2026-06-11
status: resolved   # fix deployed + validated 2026-06-17 (release lag now ≈0); see 2026-06-17-release-lag-fix-validated-launch-discrepancy.md. Note: the predicted "~10% hot in vz" flight-stretch was independently confirmed (vz ratio ~1.12).
phase: "V0 throw accuracy — temporal validation"
files_changed: []   # analysis only; no code changed
related:
  - 2026-06-10-temporal-accuracy-protocol.md
subsystem:
  - throwing
  - timing
  - firmware
tags:
  - release-lag
  - hand-trajectory
  - decel-zero
  - velocity-overshoot
---

# Release-lag root cause confirmed in firmware — two decel-zero mechanisms cancel each other

## Context

The 2026-06-10 temporal campaign measured δ = +126.5 ms, decomposed as
**+82.3 ms release lag** + **+51.3 ms flight stretch (release ~10% hot in
vz)** − 6.7 ms ball radius
([2026-06-10-temporal-accuracy-protocol](2026-06-10-temporal-accuracy-protocol.md)).
This entry records the line-level firmware confirmation, a search for prior
records of the problem, and the recommended fix.

## Prior record search (user recalled a May discussion)

No logbook/git/docs record predates 2026-06-10 in either repo. The
recollection matches a **2026-05-23 Claude session in the Jugglebot-bb
project** (transcript `~/.claude/projects/-home-jetson-Desktop-Jugglebot/`
`83618abc-…jsonl`) about the **sim's** hand release: at the 40 Hz sim command
rate the MuJoCo actuator can't track the throw stroke, so release velocity ≠
`throw_speed_mps`. It was "logged" only as a docstring in
`Jugglebot-bb/sim/juggle_demo.py` (`_analytic_release_velocity_world_mms`,
~line 673) — which asserts the problem is **sim-specific** because "on
hardware … the velocity at release IS the analytic throw_speed". The
2026-06-10 cone campaign **falsifies that assertion for BB hardware**
(measured release ~10% hot in vz at 500 Hz streaming). Different mechanism
(timing defect + servo tracking vs sim sample-rate), same blind spot:
commanded release state was trusted, never measured.

## Root cause — line level

The release instant is decel start (hand decelerates > g; ball separates).
The code contains **two independent, individually-correct decel-zero
mechanisms that defeat each other**:

1. `Trajectory.h:110` — `makeThrow()` ends with `shiftTime(out, -t2)`:
   the intrinsic throw trajectory's t=0 is ALREADY at decel start
   (times span [−t2 … t3−t2]).
2. `HandPathPlanner.cpp:179` — `appendTrajectoryRebased()` computes
   `f.t_s = t_cursor + (tr.t[i] − tr.t[0])`, re-zeroing at the **first
   sample** — silently discarding mechanism 1's shift.
3. `HandPathPlanner.cpp:71-72` — `planThrow()` caches
   `last_decel_time_in_throw_ = throwTr.t[i_decel]`. Because of mechanism 1
   this is **≈ 0** (decel start sits at t≈0 in the shifted trajectory),
   not the intended "decel time after stroke start" (= t2).
4. `HandPathPlanner.cpp:255` — `planThrowDecelZero()` subtracts
   `t_zero_global = last_time_to_ready_s_ + last_decel_time_in_throw_`
   ≈ `last_time_to_ready_s_ + 0`: t=0 lands at **stroke start**, not decel
   start. The streaming variant (`HandPathPlanner.cpp:288`) has the
   identical defect (`t_decel_in_throw = throwTr.t[i_decel]` ≈ 0).

Result: `HandTrajectoryStreamer` (fires frame i at
`throw_wall_us + t_s·1e6`) runs the whole stroke **after** `throw_wall_us`;
release lands at `throw_wall_us + t2`, where from `calcThrow()`
(`Trajectory.h:132-152`) with BBTraj constants (stroke 0.28 m, margin 0.02,
vel-hold 5%, inertia ratio 0.747):
`t2 = [2/(IR+1)·accelSt + velHold]/v = 0.273/v` → **83.3 ms at 3.278 m/s**.
Measured: 82.3 ± 14.8 ms. Confirmed.

## The fix (two lines, NOT yet applied)

Make the cached decel time relative to the trajectory's first sample, which
is correct regardless of any upstream shift:

- `HandPathPlanner.cpp:72`:
  `last_decel_time_in_throw_ = throwTr.t[i_decel] - throwTr.t[0];`
- `HandPathPlanner.cpp:288` (streaming variant):
  `const float t_decel_in_throw = throwTr.t[i_decel] - throwTr.t[0];`

Safety: the StateMachine lead-time gate (`StateMachine.cpp:867-878`) reads
`min(t_s)` from the actual frame buffer, so the required lead automatically
grows by ~t2 — commands inside the old margin get rejected loudly, not
mis-scheduled. `shiftTime` in `makeThrow` can stay (harmless; rebase ignores
it), though removing the double-shift confusion is worth a comment.

**Deliberately deferred** until after the phase-X campaign: changing release
timing shifts every landing and invalidates the deployed spatial
aim-correction (fit WITH the late, hot release). Fix + spatial re-fit +
repeat temporal campaign as one before/after experiment.

## Velocity overshoot (+51 ms term) — design observations

- `accelToTorque()` (`Trajectory.h:86`) feeds `tor_ff = a·m·r` with
  `INERTIA_HAND_ONLY = 0.281 kg`: **no ball mass, no gravity term**. During
  the ~42 m/s² stroke the position loop (pos_gain 35, vel_gain 0.007) must
  supply the missing torque → tracking lag → catch-up velocity overshoot at
  the accel→decel corner.
- The profile is **bang-bang accel** (trapezoidal velocity, discontinuous
  acceleration steps) with a vel-hold of only 5%·0.24 m = 12 mm →
  **t_vel = 3.7 ms ≈ 2 samples at 500 Hz** at 3.28 m/s: no settle time
  before release. Whatever overshoot exists at accel-end is what the ball
  leaves with.
- Precedent: the Jugglebot platform hand carries
  `TeensyTraj::LINEAR_GAIN_FACTOR = 1.035` (`hardware_config.h:173`) — a
  3.5% empirical speed fudge. BB's `BBTraj::LINEAR_GAIN_FACTOR = 1.0`
  (`hardware_config.h:248`) — never calibrated.
- Candidate remedies, in increasing effort: (a) measured-release-velocity
  calibration v_actual(v_cmd) inverted in the solver (mirrors the spatial
  affine flow — no firmware physics change); (b) add ball mass + gravity to
  tor_ff and lengthen vel-hold; (c) jerk-limited (S-curve) stroke;
  (d) firmware reports measured PV at decel start back over CAN so
  ThrowAnnouncements carry the TRUE release state.

## Verification

- Defect mechanics verified by direct read of `Trajectory.h`,
  `HandPathPlanner.cpp/.h`, `HandTrajectoryStreamer.h`,
  `StateMachine.cpp:828-894` (this entry's line refs).
- Quantitative: t2 = 0.273/v = 83.3 ms at 3.278 m/s vs measured
  82.3 ± 14.8 ms; phase-X prediction: release-lag component of δ scales as
  0.273/v per position.
- Side wart noted (cosmetic): `appendTrajectoryRebased` gives frames 0 and 1
  the same timestamp (`t_cursor+dt` for i=0; `t_cursor+(t[1]−t[0])=t_cursor+dt`
  for i=1) — two frames fire on one tick at each segment boundary.
