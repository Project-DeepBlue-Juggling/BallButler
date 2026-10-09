---
title: Tooling to settle the constellation estimator's yaw-offset gauge from corrected throws
type: feature
date: 2026-10-09
status: resolved
related_entries:
  - 2026-10-09-bb-local-calibration-result
files_changed:
  - zTesting/throw_testing/accuracy_testing/settle_yaw_gauge.py
  - zTesting/throw_testing/accuracy_testing/test_settle_yaw_gauge.py
  - zTesting/throw_testing/accuracy_testing/run_item2_sitting.sh
  - zTesting/throw_testing/accuracy_testing/LOCAL_CALIBRATION.md
  - logbook/2026-10-09-yaw-gauge-settle-tooling.md
  - logbook/INDEX.md
external_changes:
  - "Jugglebot: branch bb-constellation-yaw-2026-10-09 (mocap pose calibration from the whole sweep with a fitted latency; gauge pinned to session A's frame) — pending merge"
  - "Jugglebot + BallButler: branch bb-stamped-yaw-100hz (BB yaw in the stamped 100 Hz /bb/axis_estimates stream) — pending merge and flash"
subsystem:
  - calibration
  - tooling
tags:
  - accuracy
  - mocap
---

# Tooling to settle the constellation estimator's yaw-offset gauge from corrected throws

## Summary

Two analyses on 2026-10-09 (`~/bb_calibration_sessions/yaw_offset_investigation_20261009/`
and `~/bb_calibration_sessions/bb_placement_reinvestigation_20261009/`) showed that BB's
stored yaw offset came from one anchor marker's angle about the sweep's fitted axis point
(118 mm radius, 0.49° per mm) and jumped 0.47° between two sessions with BB untouched,
while the whole-sweep constellation fit with a fitted heartbeat latency repeats to 0.03°
per sweep. The estimator being implemented in Jugglebot reports its offset in session A's
frame through a gauge pinned from recorded data, uncertain by about ±0.2°. This tooling
settles that pin with throws.

## Motivation

A yaw-offset error e turns every corrected landing's bearing about BB's yaw axis by −e in
the frame the node used; session B measured −0.50 ± 0.05° for a +0.47° error. Forty
corrected throws therefore pin the gauge to about ±0.1° (per-throw lateral scatter ~10 mm
at ~1 m), without a refit of the affine.

## Design

- `settle_yaw_gauge.py <session>/session.json`: from `analysis/extraction.json`, per-throw
  bearing error (landing − desired, in the session's frame), its mean ± SE, the lateral mean,
  a rotation-versus-translation regression as a diagnostic, and the implied frame offset
  `used + mean_bearing`. Pre-registered rule: ≥30 accepted throws; |mean bearing| ≤ 0.15°
  → `FRAME_CONFIRMED`, else `RE_PIN` with the delta to add to `gauge.pinned_yaw_offset_deg`
  in the Jugglebot template resource. Uncorrected sessions are refused (their aim bias
  would be read as a frame error). The affine's RMS checks from `extraction_report.json`
  are reported alongside.
- `run_item2_sitting.sh [N]`: preflight (`--check-only`), the throws (`--limit N`,
  `--apply-correction`, solver `cb09095e`), `--extract-only`, then the settle script.

## Implementation

See `LOCAL_CALIBRATION.md` § "Settling the yaw-offset gauge". The sign convention is
derived in the script's docstring and pinned by a synthetic test.

## Verification

- `/usr/bin/python3 -m pytest test_settle_yaw_gauge.py -q` (2026-10-09): 5 passed, including
  the real-data regression on session B `20261009T031319_612936Z`: mean bearing
  −0.496 ± 0.053°, implied frame offset 0.184° against the 0.208° the affine was fitted in,
  verdict `RE_PIN` with delta −0.496°.
- The full calibration test set: see the commit message.

## Open Questions / Follow-ups

- The sitting itself (merge and build the sweep estimator first; flashing the stamped-yaw
  firmware is optional for it, the estimator fits the latency).
- BB's reported-to-physical yaw relation is not 1:1 (markers turn ~2.3° less than the
  encoder over 0→125°); the affine absorbs it inside the calibrated region. Not corrected.
- Fixed markers on BB's base and in the room (owner, when possible) to separate a BB or
  encoder change from a QTM frame change.
