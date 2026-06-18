---
title: "Temporal-accuracy test protocol (v1) + catching-cone uplink plumbing"
type: feature
date: 2026-06-10
status: resolved   # 2026-06-18 — protocol + cone uplink used for the temporal campaign (δ → <10 ms); see 2026-06-18 entry
phase: "V0 throw accuracy — temporal validation bring-up"
files_changed:
  - zTesting/throw_testing/temporal_accuracy/README.md
  - zTesting/throw_testing/temporal_accuracy/run_temporal_session.py
  - zTesting/throw_testing/temporal_accuracy/analyze_temporal_session.py
external_changes:
  # The cone uplink lives in the Jugglebot repo — full file list + design in
  # Jugglebot logbook/2026-06-10-cone-uplink-phase-10b.md.
  - "Jugglebot: config/generate_udp_protocol.py + regenerated artifacts (new CONE_FRAME=0x85 UDP relay message)"
  - "Jugglebot: ros_ws/src/jugglebot/Teensy_code_canbridge/{can_buses,telemetry,Teensy_code_canbridge.ino} (CAN2→UDP cone relay: SPSC ring + 100 Hz uplink step)"
  - "Jugglebot: ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py (publishes cone/catch_event + cone/heartbeat; can_node cone path retired)"
  - "Jugglebot: tests/ros/test_teensy_bridge_node_cone.py (+ migrated can_node cone tests)"
subsystem:
  - calibration
  - throwing
  - timing
tags:
  - accuracy
  - timing
  - catching-cone
  - piezo
  - testing
---

# Temporal-accuracy test protocol (v1) + catching-cone uplink plumbing

> **RESOLVED 2026-06-18** — the v1 protocol (`run_temporal_session.py` + analysis) and
> the catching-cone uplink are implemented, unit-tested, and were used for the full
> temporal campaign that drove arrival error to **< 10 ms** — see
> [2026-06-18-temporal-accuracy-resolved-fractured-solution](2026-06-18-temporal-accuracy-resolved-fractured-solution.md).

## Summary

With spatial accuracy validated (31 mm mean after the affine correction —
[2026-06-10-throw-aim-correction-validated](2026-06-10-throw-aim-correction-validated.md)),
the next axis is **temporal**: does the ball arrive when the system says it
will? This entry adds the first formal temporal-accuracy protocol and its
tooling, using the sensorized catching cone (piezo contact sensor +
time-synced Teensy) as a mocap-free arrival sensor with µs-grade timestamps.
A plumbing audit found the cone's data **could not reach the Jetson at all**
after the three-bus CAN split — the can-bridge counted and discarded CAN2
frames — so the phase-10b cone uplink was implemented in Jugglebot as the
enabling change.

## Motivation

Prior timing evidence was ad-hoc and QTM-bound: sessions 7–8 (delay sweeps,
never formally written up) and `accuracy_testing/timing_analysis.py` showed
landing-time errors in the ±25–100 ms range, and the spatial campaign
identified release-timing/repeatability as the next accuracy lever. There was
no protocol, no acceptance criterion, and no mocap-free way to measure
arrival time. The cone closes that gap: its piezo ISR latches a wall-clock
timestamp synced to the same 0x7DD time-sync domain that schedules the throw,
so end-to-end arrival error is measurable anywhere, repeatably, at ~µs sensor
resolution against ~1 ms command granularity.

## Design

**Measurand** — end-to-end arrival error per throw:
`t_catch (cone piezo) − landing_time (ThrowAnnouncement)`. Deliberately
integrates the whole chain (command latency, firmware wall-clock scheduling,
release lag, ToF model, cone fall-through). The cone-bottom geometry and
release/trigger lag form a systematic offset δ, estimated from the
repeatability block and removed before judging accuracy; δ's stability
across delays/distances is itself a diagnostic (drift with ToF ⇒ flight
model, not scheduler).

**Protocol phases** (full procedure in
`zTesting/throw_testing/temporal_accuracy/README.md`):
R — repeatability (N≥30, nominal config; calibrates δ);
D — delay sweep (2.0–5.0 s, randomized, fixed cone);
X — distance sweep (2–4 m, ≥3 positions).
**Pass: |mean residual| ≤ 10 ms per group after δ removal.** Misses,
unsynced events, and failed throws are recorded and excluded, never
silently dropped.

**Tooling** — `run_temporal_session.py` (ROS2): preflight gates (cone
READY + time-synced + sync-RMS, BB ball-in-hand), commands
`bb/throw_at_target` per schedule, pairs announcements with catch events
(±1.5 s window around predicted landing), paces on BB's reload heartbeats
(same gating as the spatial calibration session), writes crash-safe session
JSON; `--listen` mode is the bring-up tap test.
`analyze_temporal_session.py` (stdlib-only, Py3.8): per-group stats, δ
estimation/removal, trend slopes vs delay and ToF, pass/fail report,
optional plots.

## Verification

- Jugglebot side: canbridge firmware compiles (`pio run` SUCCESS); full test
  suite 724 passed, including the new loopback cone-path tests (exact
  wall-time reconstruction, unsynced fallback, drain-exactly-once,
  malformed-frame robustness). Details in Jugglebot
  `logbook/2026-06-10-cone-uplink-phase-10b.md`.
- **Bench tap test (2026-06-10 evening): measurement chain VERIFIED.** With
  the new bridge firmware flashed: cone READY/time-synced at 1–30 µs reported
  sync RMS, 6001/6001 heartbeats over 600 s, 18/18 tap events delivered with
  contiguous sequence and 13–43 ms impact→Jetson transit. One event (seq 5,
  1/18) carried a timestamp ~+40 s in the future — root-caused the same day
  to a pre-existing cone-firmware ISR race (`micros64()` false 32-bit wrap;
  fixed + reflashed — Jugglebot
  `logbook/2026-06-10-cone-micros64-false-wrap.md`). The campaign runner's
  ±1.5 s landing match window rejects any recurrence as a miss, so the
  protocol stays robust regardless.
- **First campaign (2026-06-10 night, phases R + D):** repeatability n=30:
  arrival error **+128.2 ms, σ = 8.8 ms**; delay sweep n=27 (2.0–5.0 s):
  per-delay means 121–127 ms, slope vs commanded delay **−0.17 ms/s ≈ 0**
  (wall-clock scheduling chain exonerated). With δ = +126.5 ms removed,
  every group PASSES the ±10 ms criterion — residuals −5.1…+1.6 ms.
- **The +126.5 ms decomposed** (4-way investigation; per-throw data in
  `zTesting/throw_testing/temporal_accuracy/decompose_results.json`).
  ANALYSIS HAZARD found on the way: `/balls` carries TWO tracks per throw —
  the announcement-matched KF (source `ball_butler`) is anchored to the
  announced prior, loses its 50 mm marker gate early (Jugglebot
  `tracking/matcher.py:71`) and silently COASTS through the cone and floor
  (z → −460 mm). A first-pass analysis trusting it attributed +115 ms to
  "cone interior transit" — an artifact. The measurement-driven re-detection
  track (source `human_throw`) is piezo-consistent (ball z at catch_time =
  1025 ± 27 mm = plane 960 + ball radius 35) and is what the final numbers
  use. The corrected decomposition (n=43, identity-closed to the measured
  +126.3 ± 8.2 ms):
  - **+82.3 ± 14.8 ms — release lag.** The ball leaves the hand ~82 ms after
    the announced throw_time — matching the firmware decel-zero defect
    EXACTLY (t2 = 0.273/v = 83.3 ms at 3.278 m/s): `planThrowDecelZero`'s
    intended "release lands ON throw_wall_us" shift is defeated
    (`Trajectory.h` makeThrow already re-zeroes t at decel start, so the
    planner subtracts ~0 and the rebase puts stroke-start at t=0).
    Corroborated independently by `/bb/heartbeat` hand positions (stroke
    entirely after throw_time).
  - **+51.3 ± 13.7 ms — flight longer than predicted because the ball
    releases ~10% HOT in vz** (measured 3.54 m/s vs commanded 3.28; vz
    +10.2%, vh −11.6% → effective pitch ≈ 73.7° vs 70.0°). Vacuum ToF with
    the measured vz reproduces the +51 ms exactly. Likely mechanism:
    under-damped/lagged ODrive tracking of the aggressive 500 Hz hand
    trajectory (≈42 m/s² accel) — possibly the same root as the timing of
    separation. NOT drag (numerically <1 ms for any plausible Cd).
  - **−6.7 ms — "fall-through" is essentially zero**: the piezo fires when
    the ball CENTER is one radius above the rigid-body-origin plane — the
    cup surface sits at the origin plane. The cone is innocent.
  - Announce-vs-encode skew +3–4 ms; everything else ≤ ±2 ms.
  **CAUTION: the deployed spatial aim-correction was fit WITH the displaced,
  hot release — fixing the firmware defect or the velocity overshoot shifts
  landings and requires a spatial re-fit.** Both fixes should also precede
  any catching integration (the announcement's initial_position/velocity
  are currently untruthful, and the `ball_butler` KF track coasts silently
  off-gate — a separate tracking hazard for consumers of /balls).
- **Pending:** phase X (distance sweep) — predicted δ structure: release
  lag scales as 0.273/v, flight error scales with the overshoot fraction ×
  geometry; per-position grouping in the analysis handles it. Decide
  whether to fix the firmware decel-zero defect + re-fit spatial BEFORE the
  next campaign or keep characterizing the as-is system.
