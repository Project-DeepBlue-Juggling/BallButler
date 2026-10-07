---
date: 2026-10-06
status: tuned
type: feature
subsystem: calibration, tooling
---

# Refill-aware trajectory extraction and simulated calibration validation

The capture runner already marked successful BB flight windows and refill
intervals, but did not yet extract noisy trajectories or fit a replacement
correction. The owner requested analysis ready immediately after the sitting,
tested with +/-2 mm readings, 1% throw-state errors and ground bounces.
Jugglebot will remain connected; no disconnected-platform mode is needed.

Added `analyze_local_calibration.py`, accepting the recorded session plus raw
ROS2 MCAP, or identically structured synthetic observation frames. It excludes
refill/failed/incomplete windows, seeds gravity-constrained RANSAC near BB's
release state, measures the descending catch crossing and rejects ambiguous
or insufficient tracks. There is no target-XY proximity selection. The affine
is fitted commanded-to-measured with equal weight per cell, then inverted in
BB-local coordinates. Whole-cell cross-validation and core-area/repeatability
metrics are reported. Output remains a candidate: production geometry and
the historical affine have not been changed.

`simulate_local_calibration.py` generates all331 throws at200Hz with shuffled
unlabelled markers, static clutter, refill projectiles every9 throws, small
bounces/rolling and disappearance one second after impact. The systematic
mocap XY response is [[1.012,0.008,10],[-0.006,0.990,20]]. Independent uniform
+/-1% velocity-component errors are applied per throw; release position is
unchanged. Every observed coordinate has independent uniform +/-2mm noise.
A synthetic BB pose makes all planned targets reachable. The analysis never
reads truth.json.

## Outcome and verification

All331 simulated throws extracted across143 cells, none rejected. Extraction
error against hidden truth:0.237mm RMS. Recovered command correction vs true
inverse:1.004mm RMS. An independent validator re-solves corrected commands and
tests2860 fresh throws (300 core), each before/after with paired fresh random
state errors. Full-grid RMS:25.33→10.57mm. Core RMS:24.36→10.10mm; mean core bias
(+10.03,+19.70)→(-0.45,-0.26)mm. Remaining scatter is largely injected random
state variation, which an affine cannot cancel.

23 tests pass, including noise/clutter/bounce extraction, competing trajectory
rejection, absent/occluded balls, affine direction/rank and an actual ROS2 MCAP
encode/decode test. Scripts parse as Python3.8. Raw simulation remains outside
the repository; compact results and deterministic regeneration scripts are
committed. See [usage and results](../zTesting/throw_testing/accuracy_testing/LOCAL_CALIBRATION.md).

No hardware was commanded. Real QTM/DDS timing, visibility and corrected-throw
accuracy still require the hardware pilot. The fitted candidate requires the
positive-s geometry and replaces, rather than compounds, the old affine.

## First hardware pilot: UUID serialization fix (2026-10-07)

The first action was dispatched and accepted, then checkpointing failed with
`Object of type uint8 is not JSON serializable`. ROS2 exposes the goal UUID
as a NumPy uint8 array; converting it with `list()` preserved NumPy scalars.
The same invalid row also broke the final checkpoint. Convert each UUID byte
explicitly to a Python int before storing it. A regression test uses an actual
NumPy uint8 array and checks accepted/event/failure JSON persistence.
The interrupted session must not be resumed or treated as a confirmed failed
throw: its first command may have executed. Start a fresh pilot after updating.
