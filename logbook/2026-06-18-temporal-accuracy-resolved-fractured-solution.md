---
title: "Temporal accuracy resolved (≈44 ms late → <10 ms): aim-correction for space + a measured release-latency offset for time; kinematic-ID and feedforward root-fixes ruled out"
type: investigation
date: 2026-06-18
status: resolved
phase: "V0 throw accuracy — temporal validation"
related_entries:
  - 2026-06-17-release-lag-fix-validated-launch-discrepancy
  - 2026-06-12-temporal-warmup-drift
  - 2026-06-11-release-lag-firmware-analysis
  - 2026-06-10-throw-aim-correction-validated
  - 2026-06-10-temporal-accuracy-protocol
files_changed:
  - ball_butler_main/Trajectory.h               # feedforward sweep — reverted to optimal baseline + documented
  - ball_butler_main/StateMachine.cpp           # monotonic PV-freshness gate; firmware lead-comp tried then reverted
  - ball_butler_main/CanInterface.cpp           # monotonic (sync-immune) PV timestamp + axisPVMonoAgeUs()
  - ball_butler_main/CanInterface.h
  - ball_butler_main/ball_butler_main.ino       # per-axis PV-age / ODrive-state diagnostics in `status`
  - ball_butler_main/hardware_config.h          # regenerated
  - zTesting/throw_testing/temporal_accuracy/    # NEW: kinematic-ID + lag-characterization tooling
external_changes:
  - "Jugglebot: ros_ws/src/jugglebot/jugglebot/ball_butler_node.py — release-latency offset (command the throw BB_OP_THROW_RELEASE_LATENCY_MS earlier)"
  - "Jugglebot: ros_ws/src/jugglebot/jugglebot/can/throw_ballistics.py — bb_release_state() unified forward model; predict_throw now launches from the release point (forward/inverse consistency) + round-trip test"
  - "Jugglebot: config/hardware_config.yaml — BB_OP_THROW_RELEASE_LATENCY_MS=44 (replaces the dead landing_time_offset_ms=-120 relic); BB_AXIS_ESTIMATES 0x86 stream"
  - "Jugglebot: ros_ws/src/jugglebot/launch/jugglebot_launch.py — aim_correction defaults true; rosbag + teensy_bridge_node added"
  - "Jugglebot: ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py — BB axis-estimate uplink + startup-race fix"
subsystem:
  - throwing
  - firmware
  - calibration
  - timing
tags:
  - accuracy
  - timing
  - ballistics
  - mocap
---

# Temporal accuracy resolved (≈44 ms late → <10 ms): a fractured spatial+temporal solution

## Summary

BB now throws **to the right place at the right time** across the workspace:
**arrival error ≤ 14 ms, mean/median < 10 ms** (from a stable ~44 ms late). The
solution is deliberately **fractured** — each sub-problem solved by the tool that
works:

- **Spatial:** the existing 2D `aim_correction` affine, which empirically absorbs
  *all* spatial execution errors (yaw, pitch, speed, geometry) in one fit.
- **Temporal:** a single **measured release-latency constant**,
  `BB_OP_THROW_RELEASE_LATENCY_MS = 44 ms`, applied in `ball_butler_node` as
  *command the throw 44 ms earlier*. So a prod request for `(pos, time)` puts the
  ball there, then — not 44 ms late.

The headline correction to the 2026-06-17 entry: the residual δ is **not** a
"+3° steeper / +10% hot launch execution error." That reading was largely a
**measurement artifact** of the ~44 ms release latency contaminating the mocap
launch-velocity estimate, compounded by unreliable geometry fits. The hand
encoder shows the hand throws at **98–101 % of commanded velocity** and releases
at the **designed stroke point** — execution is essentially as-commanded. What
remains is a **constant ~44 ms actuator latency** (hand command → physical
release), confirmed two independent ways (hand encoder + cone) and proven *not*
fixable by the trajectory feedforward.

This closes the six open questions from
[2026-06-17](2026-06-17-release-lag-fix-validated-launch-discrepancy.md) (pitch
calibration sweep, static-vs-dynamic encoder, speed decoupled from mocap, barrel
direction, workspace sweep, offset-vs-root-cause). The pragmatic temporal offset
flagged there as a fallback is what we landed on — after exhausting the root-fix
routes below.

---

## What we tried — the full record (for the next person)

Four substantial threads were pursued. Two were dead ends; documenting them so
they aren't re-run. **All numbers below are copied inline** (the source bags/
session JSONs may be moved or deleted).

### Thread 1 — Kinematic system-ID (fit physical parameters). DEAD END.

**Idea:** replace the empirical `aim_correction` with a physically-correct
kinematic model — fit the BB geometry (release offsets `d, l, s, z`) + per-axis
execution calibration (pitch/yaw/speed scale+offset) + a release-time offset δ,
jointly, to the mocap-measured **release state** (position + velocity) over a
cone-free grid of throws. Tooling: `run_kinematic_id_session.py` (raw yaw/pitch/
speed grid, ceiling-guarded), `extract_kinematic_obs.py` (MCAP → per-throw launch
arc fit, track-A early-mid window), `calibrate_kinematics.py` (`fit_joint`, scipy
least-squares, CAD prior on geometry).

**Result: the mocap joint fit is unreliable here.** Two valid-yaw runs
(yaw 0–80°, pitches 45–75°), same rig, gave *swinging* launch params and
*physically-absurd* geometry:

| param | run A (n=59) | run B (n=66) | nominal/config |
|---|---|---|---|
| pitch_scale | 1.038 ± 0.016 | 1.017 ± 0.015 | 1.0 |
| pitch_off   | −0.03 ± 1.04° | +2.45 ± 0.97° | 0° |
| **yaw_scale** | **0.962 ± 0.009** | **0.954 ± 0.009** | 1.0 |
| yaw_off     | +0.10 ± 0.49° | −0.00 ± 0.43° | 0° |
| **speed_scale** | **1.056 ± 0.017** | **0.982 ± 0.016** | 1.0 |
| speed_off   | −0.259 ± 0.052 | +0.006 ± 0.050 | 0 m/s |
| release δ   | 29.3 ± 2.5 ms | 16.2 ± 2.5 ms | — |
| release_l   | **150 → 285 mm (+135!)** | **150 → 248 mm (+98!)** | 150 mm |
| yaw_s_offset | **−105.65 → +120.8 mm** | **−105.65 → +120.1 mm** | −105.65 mm |
| pos-fit RMS | 15.2 mm | 9.5 mm | — |

`speed_scale` flipped from 1.056 to 0.982 between runs, and `release_l` and
`yaw_s_offset` are off by >100 mm — not credible. Root cause of the unreliability:
the ~44 ms release latency biases the back-extrapolated launch velocity (vz is
evaluated *before* the actual release), and the joint fit's δ absorbs it
**inconsistently** because it trades off against the (also-wrong) geometry. The
**only stable, trustworthy output is `yaw_scale ≈ 0.95–0.96`** (yaw under-rotates
~4–5 %, δ-clean because horizontal velocity is gravity-independent).

**Lesson:** mocap launch-state inference cannot cleanly separate
geometry / launch-execution / release-timing on this rig. The sensorized cone
(direct landing position + catch timestamp) is the ground truth — use it, not
more mocap kinematic-ID.

**Aside (worth keeping):** the `yaw_s_offset` "sign flip" looked alarming but is a
frame/convention difference between the analysis lateral axis and the config
convention; it's moot because `aim_correction` handles all spatial error anyway.

### Thread 2 — The "+3° / +10%" launch discrepancy, reframed. (It was mostly the δ.)

2026-06-17 concluded the residual δ was a real launch-execution error
(vz ~1.12 hot, vh ~0.93 slow, elevation +3.1°). This session's **hand encoder**
data (the BB ODrive now streams 1 kHz pitch/hand estimates via
`BB_AXIS_ESTIMATES`) contradicts that:

- Peak hand velocity = **98–101 % of commanded** (it reaches the throw speed).
- Hand reaches the **full stroke** (237 mm vs `x3` = 240 mm).
- It just gets there **~44 ms late** (next thread).

So the hand executes ~as-commanded. The mocap "+3°/hot" was the latency inflating
vz (evaluated at the commanded time, before the actual release) plus the Thread-1
fit artifacts — not a barrel-angle or spool-radius error. `LINEAR_GAIN_FACTOR`
stays 1.0; no pitch zero-offset correction is warranted.

### Thread 3 — Feedforward: is the ball's inertia missing from the torque FF? DEAD END.

**Idea (plausible):** `INERTIA_RATIO` (0.747) only sizes the trajectory *timing*
(the accel phase is lengthened so accel-with-ball and decel-without-ball need
equal motor torque ⇒ `I_total = I_hand / INERTIA_RATIO`). But `accelToTorque()`
used `INERTIA_HAND_ONLY` for **every** phase — so the accel-phase torque
feedforward was ~25 % low (omits the ball), the position loop makes up the
difference, hence a lag. We swept the accel-phase inertia:

| accel torque FF | encoder release lag | peak vel |
|---|---|---|
| 0 (none) | 65.8 ± 5.0 ms | 101 % |
| **`I_hand` (baseline)** | **43.7 ± 4.9 ms** | 98 % |
| `I_hand / INERTIA_RATIO` (hand+ball) | 56.3 ± 5.0 ms | 98 % |

**The lag has a minimum at the baseline (hand-only) FF — non-monotonic.** Both
*more* and *less* feedforward make it worse; the FF cannot push the lag below
~44 ms. The original hand-only feedforward was already optimal. (`Trajectory.h`
reverted to baseline, with this sweep documented in a code comment so it isn't
re-tried.)

### Thread 4 — Hand trajectory generator: where does the ~44 ms come from?

- **Release point is fixed & correct.** `x2 = 0.594 · totalStroke = 0.594 · 240 =
  142.6 mm` into the stroke (≈ config `release_l = 150 mm`), and the `v_throw`
  cancels in the algebra so it is **speed-independent** — the ball always leaves
  at the same stroke point, as designed.
- **Scheduling is correct.** The streamer dispatches the release frame (`t_s≈0`)
  exactly at the commanded wall time; the main loop runs fast enough that frames
  go out on time.
- **The lag is a constant transport/actuator latency.** Encoder: 43.7 ± 4.9 ms,
  speed-slope **+1.4 ms per m/s ≈ flat**. A constant (speed-independent) delay is
  a transport/processing latency, **not** a bandwidth-tracking lag — so ODrive
  gain tuning won't remove it either. The hand simply reaches the (correct)
  release point ~44 ms after commanded.

(Also fixed along the way, unrelated to the latency: a PV-freshness gate that used
the *wall* clock — corrupted by time-sync steps — was switched to a **monotonic**
timestamp (`CanInterface::axisPVMonoAgeUs`), which had been causing "PV stale"
throw rejections whenever the bridge was connected.)

A firmware **lead-compensation** (schedule the throw 44 ms early) was implemented
and then **reverted** in favor of the cleaner Jetson-side offset below — but it
proved the principle.

---

## Resolution — the fractured solution

### Spatial — `aim_correction` (unchanged)

The 2D affine from [2026-06-10](2026-06-10-throw-aim-correction-validated.md)
pre-distorts the BB-local target so the ball lands right. It empirically subsumes
**every** spatial execution error we chased in Threads 1–2 (the yaw under-rotation,
any pitch/speed/geometry deviation) — which is exactly why re-deriving them from
physics was redundant. Spatial accuracy is good across most of the workspace
(operator-confirmed; misses in the temporal runs were spatially close).

### Temporal — one measured latency constant, applied as "throw earlier"

The temporal error is **constant across the workspace**. From a distance sweep
(`run_temporal_session.py --phase distance-sweep --positions 15 --throws 5
--delay 2.5`, 34 cone-registered catches):

```
arrival error (catch_time − predicted_landing_time):
  mean 44.1 ms,  median 44.4 ms,  std 5.3 ms,  range [34.9, 54.5]
correlation of arrival error with:
  horizontal range (721–1535 mm):  -0.04   (i.e. flat; -0.7 ms across the span)
  throw speed:                     +0.00
  pitch:                           -0.02
  catch height / ToF:              -0.43   (~6 ms across the full 360 mm height span,
                                            within the 5.3 ms throw-to-throw scatter)
```

No dependence on distance/speed/pitch ⇒ a **single fixed offset suffices** (no
position map). Deployed as `BB_OP_THROW_RELEASE_LATENCY_MS = 44.0`, consumed in
`ball_butler_node._throw_at_global_point`:

```python
release_latency_s = hw.BB_OP_THROW_RELEASE_LATENCY_MS / 1000.0
bb_req.throw_time = float(max(0.0, delay_s - release_latency_s)) if throw else 0.0
```

This commands the release `latency` earlier, so the actuator latency cancels and
the ball **arrives at the requested time**. It is **purely temporal** (same
yaw/pitch/speed ⇒ same landing place; only the release *timing* shifts) so it does
not touch `aim_correction`. It is applied at the command boundary, so it is
correct for the **prod `(pos, time)` pipeline** (where `delay = T − tof − now`),
not just the bench delay. The announced landing time is left equal to the true
arrival. Replaces the dead `landing_time_offset_ms = −120` relic of the old
pre-release-lag-fix era.

## Verification

| | before | after offset |
|---|---|---|
| arrival error (mean/median) | ~44 ms late | **< 10 ms** |
| arrival error (max) | ~54 ms | **~14 ms** |
| spatial | good (aim_correction) | unchanged |

Confirmed on a fresh distance sweep. Spatial hit rate unchanged (offset is
timing-only). Firmware unchanged for this fix — Jetson-side only.

## Open Questions / Follow-ups

1. **Cone piezo no-fire on close catches** (carried from
   [2026-06-12](2026-06-12-temporal-warmup-drift.md) #6). ~40–55 % of
   *spatially-close* throws didn't trigger the contact sensor (excluded as
   "misses"). It caps the cone's measurement yield **and** would matter for real
   catching — worth a sensitivity/threshold pass on the cone. **Outstanding.**
2. **`yaw_min = 0°` rejects slightly-negative required yaw** — 3/75 temporal
   throws failed inverse-ballistics on `Yaw … out of BB range [0,185°]`. Limits
   some workspace directions; revisit the yaw zero/range if those targets matter.
   **Outstanding.**
3. **`aim_correction` workspace coverage** — it's a first-order 2D affine; "very
   good across most of the workspace." Confirm/extend (or re-fit) if the far
   corners drift. **Outstanding (low priority).**
4. **The ~44 ms latency source is characterized but not localized** (CAN command
   pipeline vs ODrive command-processing vs estimator latency). It's compensated,
   not eliminated. If we ever want it *gone* (vs offset), that's the dig.
5. **Catch feedforward** (`Trajectory.h buildCatch`) was left hand-only; its own
   inertia phasing (ball acquired mid-stroke) is an untouched, untested potential
   improvement for *catching*.
6. **`predict_throw` consistency refactor** (Jugglebot `throw_ballistics.py`):
   `predict_throw` now launches from the geometric release point via the shared
   `bb_release_state` and round-trips with the solver (test added). Currently
   behavior-neutral (not in the live announce path, which uses the solver's ToF) —
   a latent correctness improvement.
