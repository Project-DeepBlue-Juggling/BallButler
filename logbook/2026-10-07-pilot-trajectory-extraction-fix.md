---
title: Pilot trajectory extraction — labelled ball, wrong flight model, apex occlusion
type: bugfix
date: 2026-10-07
status: resolved
related_entries:
  - 2026-10-06-local-calibration-analysis
  - 2026-10-05-mirrored-hand-local-calibration
files_changed:
  - zTesting/throw_testing/accuracy_testing/analyze_local_calibration.py
  - zTesting/throw_testing/accuracy_testing/test_analysis_local_calibration.py
  - zTesting/throw_testing/accuracy_testing/simulated_calibration_validation.json
  - zTesting/throw_testing/accuracy_testing/LOCAL_CALIBRATION.md
  - zTesting/throw_testing/accuracy_testing/run_local_calibration.py
  - zTesting/throw_testing/accuracy_testing/test_local_calibration.py
  - logbook/2026-10-07-pilot-trajectory-extraction-fix.md
  - logbook/INDEX.md
commits:
  - 308497e
  - a24ecda
  - (pre-flight follow-up)
subsystem:
  - calibration
  - tooling
tags:
  - mocap
  - ballistics
  - testing
---

# Pilot trajectory extraction — labelled ball, wrong flight model, apex occlusion

## Problem

The first complete ten-throw hardware pilot
(`~/bb_calibration_sessions/20261007T043306_968166Z`, `--schedule-to-mocap 0.3 -0.6`,
all ten firmware OK) extracted **0/10** throws. Throws 0, 1 and 9 failed with
"no continuous launch-to-catch ballistic fit passed quality gates", and throws 2–8 with
"no launch-associated observations". The 331-throw synthetic campaign had
passed because it assumed unlabelled, fully visible, gravity-only flights with nominal timing.
All analysis was offline; no hardware was commanded.

## Root Cause

Before touching the extractor, I checked each assumption against the recording:

1. **Timing and clocks were fine.** `/mocap_data.stamp` is QTM frame time, from the
   `Jugglebot-skills` checkout's `mocap_node`:
   - Stamp spacing is quantised at 3.33 ms (300 Hz QTM, republished by a 200 Hz timer, so some stamps repeat).
   - Stamp minus receipt time: median −1.7 ms, range ±23 ms; slope 0.99993.
   - `BallButlerThrowCmd.throw_time` is a *relative* delay. The canbridge adds it to its synced wall clock
     (`ball_butler_protocol.h`), and the runner subtracts the 44 ms release latency, so
     `nominal = dispatch + 3 s` is consistent with the code.
   - Firmware OK arrived 38–55 ms after nominal release.
2. **The reader dropped the ball (primary cause).** QTM gave the thrown ball rigid-body labels:
   - `Base - 1` covered the ball at rest in the hand and the entire flight of throws 0, 2 and 4.
   - `Base - 3/4/7` covered parts of other flights.
   - The reader kept only empty-label markers.
3. **The flight model was wrong.** With every marker kept, the old extractor still rejected 10/10:
   - A gravity-only arc leaves smooth residuals: about ±10 mm mid-flight and 30–50 mm at the ends.
   - A free constant-acceleration fit gives about (0, +140, −9 560) mm/s² on every throw, whatever its direction.
   - Quadratic drag plus a free effective-gravity vector fits every throw to 1.3–2.3 mm RMS,
     with g_eff ≈ (+80, +170, −9 740) mm/s², about 1.1° off the frame z axis.
     BB's calibration reports 0.87° axis tilt.
   - A frame tilt and spin (Magnus) effects cannot be separated with this data. The tilt is recorded as an observation, not acted on.
4. **The launch gate assumed the wrong time and position.**
   - The arc passes within 5–11 mm of the predicted release point, but 40–57 ms *before* nominal release.
   - At nominal release the ball is therefore 85–150 mm higher than predicted, which tripped the 150 mm position-at-t=0 gate.
5. **Apex occlusion.** The ball is often unseen from about 0.1–0.3 s until about 0.4–0.47 s.
   Throws 6 and 9 have no ballistic points before roughly 0.37–0.40 s. This defeated
   the "≥10 points in 0–350 ms" seed and the 80 ms whole-track gap limit.

## Fix

Agreed with the owner before implementation. `analyze_local_calibration.py`:

- **Reader:** keeps all markers, whatever their label. Exactly repeated frame stamps are counted once.
- **Seeds:** anchored at the predicted *release position* at a few arc times and free in velocity, so the
  landing is never pulled toward the target.
- **Launch gate:** the arc's closest approach is ≤100 mm from the predicted release position, at an arc time within ±0.10 s.
- **Free-flight gate:** a per-axis constant-acceleration fit within 6 % of g. The 8 mm inlier, 5 mm RMS, 30-frame and 0.25 s gates are unchanged.
- **Occlusion:** earlier gaps up to 0.45 s are bridged. The segment through the catch plane
  must be continuous (≤80 ms gaps) for ≥0.25 s and observed to within 15 ms of the crossing.
- **Measurement:** catch-plane XY comes from a local fit over −150/+40 ms around the crossing, with the track's acceleration held fixed.
- **Rejections:** each one carries the furthest gate reached plus per-gate counts in `extraction.json`.
- **Ambiguity:** checked between distinct tracks (<50 % shared frames).

Pre-registered criterion: local crossings must agree within 3 mm with an independent
drag + tilted-gravity ODE fit, otherwise reject. Result: 0.9–2.1 mm on all ten throws, so it passed.

## Verification

Pilot, using the CLI with `--extract-only --out …/analysis_fixed`. The original `analysis/`, `session.json`
and bag are untouched; the historical `recording_needs_review` flag is retained.
Errors are measured catch-plane XY minus the commanded target, in mocap mm.

| throw | cell | catch XY | error | samples | final segment (s) | max gap (s) | fit / local RMS (mm) | arc offset |
|---|---|---|---|---|---|---|---|---|
| 0 | 41 | (−214.3, −104.3) | (+22.9, −3.7) | 181 | −0.018–0.915 | 0.013 | 2.53 / 0.37 | −48 ms |
| 1 | 131 | (−345.4, 535.8) | (+44.9, −4.2) | 125 | 0.367–0.927 | 0.236 | 2.93 / 0.48 | −49 ms |
| 2 | 92 | (−371.2, 87.6) | (+19.2, −11.8) | 168 | −0.016–0.925 | 0.017 | 3.22 / 0.26 | −57 ms |
| 3 | 51 | (678.3, −84.7) | (+87.4, +15.9) | 133 | 0.439–0.933 | 0.157 | 2.72 / 0.48 | −48 ms |
| 4 | 60 | (155.6, −57.5) | (+55.3, −6.9) | 150 | 0.143–0.936 | 0.187 | 2.58 / 0.53 | −45 ms |
| 5 | 139 | (243.2, 582.8) | (+92.9, +42.8) | 148 | 0.435–0.942 | 0.173 | 2.82 / 0.53 | −40 ms |
| 6 | 78 | (−576.1, 48.2) | (+14.2, −1.2) | 107 | 0.402–0.929 | 0.363 | 2.61 / 0.81 | −50 ms |
| 7 | 64 | (712.3, −37.9) | (+121.4, +12.7) | 134 | 0.456–0.923 | 0.186 | 3.38 / 0.37 | −48 ms |
| 8 | 115 | (447.3, 191.7) | (+56.4, +4.8) | 144 | 0.064–0.924 | 0.107 | 3.18 / 0.41 | −53 ms |
| 9 | 39 | (−568.0, −105.0) | (+22.3, −4.4) | 116 | 0.353–0.923 | 0.290 | 2.39 / 0.47 | −56 ms |

All ten are accepted. The mean miss is (+53.7, +4.4) mm, RMS 65.9 mm, and the X miss grows with target X.
These are uncorrected misses from ten throws, not an accuracy claim. Ten cells are below the
affine's 12-cell minimum by design, so no correction was fitted.

Synthetic campaign, re-run with the new extractor:
- 331/331 throws recovered; extraction vs truth 0.47 mm RMS (was 0.24).
- Correction vs true inverse 0.78 mm RMS.
- Fresh core RMS 24.36 → 10.09 mm.

Tests: 28 pass under the PDJ venv. The new regression tests cover a hardware-shaped flight,
a labelled ball through the MCAP reader, a non-BB arc rejected at the launch gate, and an unobserved crossing rejected.
The hardware-shaped and MCAP tests fail on the previous extractor.

## Open

- **Arc passes the release point about 48 ms early.** One possible explanation is the host's 44 ms latency compensation
  double-counting after the 2026-06 firmware release-lag fix. Another is a release-geometry offset
  along the stroke. Settle it with the direct sensor: the hand position in the bag's `/bb/axis_estimates` against the ball arc.
- **About 1.1° effective-gravity tilt plus drag.** This is unresolved: it could be a frame tilt or spin. The production solver assumes gravity-only flight with frame z vertical.
- **Model coverage.** The simulator still models gravity-only, visible-launch flights. The hardware-shaped
  cases live in unit tests. Edge cells (larger range, higher apex) are untested on hardware;
  watch the per-gate rejection diagnostics in the full campaign.
- **Runtime.** About 1.2 s per throw for synthetic JSONL analysis; about 27 s for the ten-throw pilot, which is dominated by MCAP decoding.

## Follow-up: false "mocap receive gap" stop (second pilot, QTM recording)

Session `20261007T051301_800928Z`, run with a simultaneous QTM recording,
stopped at throw 6: "Mocap receive gap exceeded 100 ms" (0.102 s). The bag shows
no gap. Receipt gaps were ≤13 ms and QTM stamp gaps ≤21 ms on every throw, and the clean
pilot was the same (≤17 ms). Yet the runner reported 78–99 ms on *every*
throw of both sessions. The guard measured spacing between the runner's own
callbacks. Each `session.json` checkpoint costs ~23 ms, growing with the session. Several in a row
around dispatch and result, with a depth-5 subscription, stall the callbacks by
~100 ms. The full campaign would have tripped it regardless of QTM. The QTM
"timestamp discontinuity" warning came at 16:12:54, before the first throw, and is harmless.

Fix (agreed): a deep (100) best-effort subscription, and `StreamMonitor` judges
continuity from QTM source stamps. A >100 ms stamp gap, a backwards step or a zero
stamp rejects **that throw's capture** (`capture_rejected`) and the session
continues. Three consecutive rejections, or mocap stale for >1 s, still stop the session.
The analysis lists rejected or incomplete captures. Replaying both bags' real
stamps: max 13–21 ms, no rejections. The second session's five complete throws all
extract (fit RMS 2.1–3.1 mm, arc offset −49 to −53 ms, repeating the pilot).
31 tests pass (PDJ venv; system 3.8 skips the MCAP test).

## Follow-up: pre-flight review for the full campaign (2026-10-09)

A fresh agent walked the pipeline offline and read-only. It found no definite blocker; the
high-risk items were all fixed here. Agreed with the owner.

- **Wrong solver risk (verified).** `~/.zshrc` sources `~/Desktop/Jugglebot/ros_ws/install`. Both pilots
  imported `Jugglebot-skills`'s `throw_ballistics.py`. The two differ (sha256 3b4695b4… vs
  bbb80fa5…, 279 vs 277 feasible, up to 0.5° pitch, 84 mm/s speed) and would fail
  silently. The runner now prints the solver path and sha. The new `--expect-solver-sha` aborts on a mismatch,
  and `--resume-from` requires the same sha.
- **No resume.** `--resume-from SESSION_JSON` (repeatable) skips entries already
  released and cleanly captured, and re-throws the rest. Plan, s, frame translation and solver must
  match. The analyser pools several sessions, using per-session BB pose and `K:IDX` exclusions.
- **Stray keystrokes.** The refill prompt flushes earlier keystrokes and re-prompts on junk instead of stopping.
- **Duration.** About 10.4 s per throw cycle and a 35 s refill give ~66 min for 277 throws.
  QTM runs as a continuous capture, ended after the runner closes its bag.
- **Lower risk.** The carry-on decision is now a tested `capture_decision()`. The redundant post-dispatch
  checkpoint was removed (each save costs ~55 ms at 277 rows). The analyser skips and counts
  bad-clock frames instead of aborting.

Real pilots pooled through the new CLI: 15 accepted. The stopped throw is listed as
"capture incomplete". Originals are unchanged (md5). 35 tests pass (system 3.8 skips the MCAP test).
Still untested on hardware: the carry-on path and resume. The simulator still uses
gravity-only flights.
