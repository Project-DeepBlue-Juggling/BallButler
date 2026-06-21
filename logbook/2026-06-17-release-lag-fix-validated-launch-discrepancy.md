---
title: Release-lag fix validated on hardware; residual δ is a +3° steeper / +10% hot launch
type: investigation
date: 2026-06-17
status: resolved   # 2026-06-18 — see resolution note below; 2026-06-21 — amended (launch is ~7-8% slow, not hot)
related_entries:
  - 2026-06-11-release-lag-firmware-analysis
  - 2026-06-12-temporal-warmup-drift
files_changed:
  - ball_butler_main/HandPathPlanner.cpp
  - ball_butler_main/platformio.ini            # restored (had been removed from the repo)
  - ball_butler_main/scripts/fix_teensy_size_glibc.py
  - ball_butler_main/scripts/bin/teensy_size
  - zTesting/throw_testing/temporal_accuracy/analyze_velocity_postfix.py
  - zTesting/throw_testing/temporal_accuracy/analyze_velocity_robust.py
  - zTesting/throw_testing/temporal_accuracy/analyze_velocity_ransac.py
  - zTesting/throw_testing/temporal_accuracy/analyze_launch_velocity.py
external_changes:
  - "Jugglebot: Teensy_code_canbridge/time_base.{cpp,h} + CatchingCone_code (clock step-on-jump + freshness fix; see 2026-06-12 entry) — FLASHED to both Teensys"
subsystem:
  - throwing
  - firmware
  - calibration
tags:
  - accuracy
  - ballistics
  - timing
  - mocap
---

# Release-lag fix validated on hardware; residual δ is a +3° steeper / +10% hot launch

> **RESOLVED 2026-06-18** — see
> [2026-06-18-temporal-accuracy-resolved-fractured-solution](2026-06-18-temporal-accuracy-resolved-fractured-solution.md).
> The "+3° steeper / +10% hot launch execution error" conclusion below did **not**
> hold up: it was largely the ~44 ms release latency biasing the mocap launch-velocity
> estimate (vz evaluated before the actual release) plus unreliable geometry fits. The
> hand encoder shows the hand throws at **98–101 % of commanded velocity** at the
> designed release point — execution is ~as-commanded. The true residual is a
> **constant ~44 ms actuator latency**, fixed via `aim_correction` (spatial) + a
> measured release-latency offset (temporal) → **arrival error < 10 ms**. All six open
> questions below were investigated there (kinematic-ID and feedforward root-fixes were
> dead ends; the pragmatic temporal offset won).

> **AMENDED 2026-06-21** — refining *both* this entry's headline and the 06-18 note above.
> A fresh trajectory + hand-encoder re-analysis (chapter
> [01 · V0 Throw Accuracy](chapters/01-v0-throw-accuracy.html), Figures 4–6) finds the launch
> is **~7–8 % slow at the commanded angle** — *neither* "+3° steeper / +10 % hot" (this entry)
> *nor* exactly "execution as-commanded" (06-18). Two anchor-clean facts settle it: (1) the
> mocap arc's **curvature** fixes horizontal launch speed at **vh ≈ 0.93 × commanded**
> (anchor-free — needs no release-time/position assumption) ⇒ narrower arc ⇒ slower
> horizontally; (2) the **hand encoder** shows the hand peaks at **97.7 % of commanded** (a ball
> cannot leave faster than the hand ⇒ **not hot**) and the **barrel angle = commanded** (⇒ **not
> steeper**). The deficit decomposes as ≈2.3 % servo under-track + ≈5.5 % hand→ball (uncalibrated
> spool radius / release transfer). The "+3°/hot" was an artifact of evaluating mocap **vz 44 ms
> before the ball releases** (gravity not yet applied) — only horizontal speed is anchor-free.
> It is compensated end-to-end (`aim_correction` + the 44 ms offset), so it is a **low-priority**
> open item; the root lever is `BBTraj::LINEAR_GAIN_FACTOR` (still 1.0, never calibrated). Tools:
> `plot_launch_geometry.py`, `plot_speed_chain.py`, `plot_0617_chain.py`.

## Summary

The release-lag scheduling defect identified on 2026-06-11 (`HandPathPlanner`
decel-zero double-cancellation) was fixed and flashed to the BB Teensy. With the
clock-sync artifact also resolved (2026-06-12), the systematic arrival delay
dropped from **~140 ms → ~44 ms** and the **measured release lag is now ≈ 0**.
The remaining **~44 ms is entirely flight stretch**: mocap shows the ball launches
**~3.1° steeper and ~10% faster than commanded**, reproduced across two sessions
(n≈28/30 each) and two independent track sources. This is an *execution* error
(actual launch ≠ commanded `v0`), not an IK/model error — the spatial aim-correction
absorbs the position consequence, leaving the flight-time consequence as δ.

## Background

- **2026-06-11** found the root cause of the ~82 ms release lag: `makeThrow()`
  shifts the throw so decel sits at t≈0, so `throwTr.t[i_decel]` ≈ 0; both
  decel-zero planners cached that ≈0 instead of the decel offset *relative to the
  throw's first sample* (= `t2`), scheduling the stroke to *start* at `throw_time`
  so the ball released `t2 ≈ 83 ms` late. Fix was deferred pending the clock work.
- **2026-06-12** found the apparent "warm-up drift" was a clock-sync measurement
  artifact (bridge re-acquisition slew); fixed with step-on-jump + freshness.

## Fix

`ball_butler_main/HandPathPlanner.cpp` — both decel-zero sites now subtract the
throw's first-sample time:

- line 72 (`planThrow`, feeds the buffer `planThrowDecelZero` used by the live
  throw path `StateMachine.cpp:858`):
  `last_decel_time_in_throw_ = throwTr.t[i_decel] - throwTr.t[0];`
- line 288 (streaming `planThrowDecelZero`):
  `const float t_decel_in_throw = throwTr.t[i_decel] - throwTr.t[0];`

**Spatially neutral.** The change shifts only the frame *timestamps* (decel/release
now lands at `throw_time`); the anchor, launch position, direction, and release
velocity are unchanged. So it removes the temporal δ *without* moving the landing
point — the validated aim-correction stays valid. This was confirmed by the data
(position accuracy unchanged; δ dropped by ~the predicted t2).

Flashing note: `platformio.ini` + `scripts/` had been removed from the repo and
`pio run -t upload` is broken on the Jetson (`teensy_reboot`/`teensy_size` are
GLIBC-2.34-linked). Restored the build files from git `d5b74b9`; flashed via
`/home/jetson/bin/teensy_loader_cli` (T4-capable, GLIBC-2.17) + a 134-baud serial
reboot. BB Teensy is a **Teensy 4.0**.

## Verification

Post-fix sessions (2026-06-17, clean clock on both Teensys):

| metric | 15:13 run (n=29) | 14:51 run (n=28) |
|---|---|---|
| arrival error (session JSON) | 41–53 ms, flat | 29–58 ms* |
| **\|v\| meas/cmd** | 1.105 ± 0.009 | 1.109 ± 0.012 |
| **vz** | 1.124 ± 0.008 (hot) | 1.128 ± 0.011 |
| **vh** | 0.929 ± 0.015 (slow) | 0.934 ± 0.016 |
| **launch elevation** | 71.0° → 74.1° (**+3.1°**) | 71.0° → 74.1° (**+3.1°**) |
| measured release lag | −16 ms (≈0) | −16 ms (≈0) |
| flight error | +62 ms | +64 ms |
| fit g_eff / RMS | 9763 / 2.5 mm | 9780 / 2.4 mm |

*the 14:51 run showed a within-session ~20 ms climb (early thermal/transient); the
15:13 run did not.

Method (`analyze_launch_velocity.py`): velocity is never finite-differenced — it is
the slope of a multi-sample fit with the physical shape imposed (known-g parabola
`z + ½gT² = z0 + vz·T` for vertical; constant-velocity line for horizontal). Two
independent track sources cross-validate:
- **Track A** (announcement-matched KF, `source=ball_butler`) over the early-mid
  window [tt+0.1, tt+0.6] is **measurement-driven there** (g_eff_free = 9802 ± 32
  mm/s² vs true 9806; RMS ≈ 2 mm; velocity ≠ the commanded prior), n≈26/30.
- **Track B** (prior-free re-detection, `source=human_throw`, RANSAC) covers the
  few throws where A diverges early; agrees with A to <0.5%.

Robustness notes: **vh ≈ 0.93 is release-height independent** (horizontal v is
constant in vacuum) — vh < 1 alone proves a steeper-than-commanded launch. \|v\|
and vz are extrapolated to the model release plane and carry a ~5%-per-50 mm `z_rel`
systematic, so read the +10–12% as "clearly hot, ~10%"; the sign and cross-session
reproducibility are certain.

## Discussion — where the launch discrepancy comes from

The ballistic model (`throw_ballistics.py:76-78`) assumes the launch velocity
direction is *exactly* the commanded pitch (`vz = speed·sinθ`, `vh = speed·cosθ`)
and magnitude is *exactly* `speed_mps`. The measured launch is a **scale (~1.10) +
rotation (+3.1°)** of the commanded vector — fully consistent with both vz and vh
(scale·cos(θ+Δ)/cosθ = 0.93; scale·sin(θ+Δ)/sinθ = 1.125). Two largely independent
effects:

- **Angle (+3.1°, robust):** the physical barrel points steeper than the commanded
  pitch. The chain is commanded `pitch_deg` → firmware map `rev = (deg−90)/360`
  (`PitchAxis.h:16`, assumes load encoder 1:1 with the barrel, zeroed at true
  vertical) → ODrive → physical barrel. Suspects, in order: (a) pitch encoder
  **zero offset** (homing sets rev=0 at a not-quite-vertical pose → constant
  offset); (b) the **linear map is an approximation** — if pitch is driven through
  a linkage rather than 1:1, the true barrel angle vs encoder is nonlinear and the
  `/360` map is only right at one calibration point (this would make any single
  hardcoded offset *unstable across the workspace*); (c) **dynamic deflection** —
  mechanical flex downstream of the encoder under throw reaction torque (a static
  spirit-level check would miss this).
- **Speed (~+10%, less certain — has the `z_rel` systematic):** the hand delivers
  more linear speed than commanded. `BBTraj::LINEAR_GAIN_FACTOR = 1.0`,
  `HAND_SPOOL_RADIUS_M = 0.0052493` (`hardware_config.h:247-248`). Either the
  effective spool radius is larger than configured (string wrap) or the hand
  ODrive overshoots the commanded velocity feed-forward at release. *Not* a
  uniform-scale (LINEAR_GAIN) issue — that can't produce an angle error, so a gain
  change is the wrong lever and `FACTOR=1.0` is being kept.

Firmware paths examined: `PitchAxis.{h,cpp}` (linear deg↔rev map, trap-traj
position control; throw lead time is ~2.5 s so a pitch-settle race is ruled out as
the cause of a *constant* offset), `StateMachine::executeThrow_`,
`HandPathPlanner` / `Trajectory.h` (release at decel = peak hand speed),
`hardware_config.h` (geometry/gain). The discrepancy is in the
command→physical-execution mapping, not the IK math.

## Open Questions / Follow-ups

1. **Pitch calibration vs the barrel — multi-angle.** Measure physical barrel angle
   (digital spirit level) against the BB_pitch ODrive reading at *several* angles
   (e.g. 40/55/71/85°). Constant Δ ⇒ zero offset (a single correction is stable);
   Δ growing with angle ⇒ scale/linkage error (needs the correct kinematic map, not
   an offset — directly answers the "stable across the workspace" worry).
2. **Static vs dynamic.** Log the BB_pitch encoder *during* a throw (ODrive
   telemetry now exists); compare to the static reading to catch reaction-torque
   deflection the encoder doesn't see.
3. **Barrel direction vs ball direction.** Mocap markers on the barrel → does the
   ball leave *along* the barrel axis, or is there loft at release?
4. **Speed, decoupled from mocap.** Log hand ODrive peak velocity during a throw;
   ball speed = v_peak·π·D. Separates spool-radius error from servo overshoot,
   independent of the mocap `z_rel` systematic.
5. **Workspace sweep.** Run `analyze_launch_velocity.py` across varied pitch/speed
   targets to map whether the angle/speed errors are constant, proportional, or
   pose-dependent — decides offset vs scale vs full model correction.
6. Prefer fixing the root cause over a hardcoded offset (may be unstable across the
   workspace). A temporal-only `predicted_tof` correction is available as a
   pragmatic fallback to reach ±10 ms without re-fitting aim.
