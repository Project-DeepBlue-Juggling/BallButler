---
title: "Mirrored-hand simulation and local two-ball calibration capture"
type: feature
date: 2026-10-05
status: tuned
subsystem: [calibration, tooling]
tags: [accuracy, mocap, ballistics, testing]
---

# Mirrored-hand simulation and local two-ball calibration capture

The user identified that BB's physical hand is left of the pitch plane when
throwing along +Y; the configured negative s puts it on the right. The offline
simulation explains most of the archived uncorrected bias: 211.3 mm predicted
versus 194.6 mm measured mean miss, 42.3 mm mean prediction discrepancy, and
9.2 degrees mean directional difference. It does not explain the whole
affine-corrected residual. Full evidence is in
[the experiment report](../zTesting/throw_testing/accuracy_testing/mirrored_hand_report.md).

Added an offline nonuniform-grid planner and ROS2 session runner in
`zTesting/throw_testing/accuracy_testing/run_local_calibration.py`. The runner
uses production ballistics with explicit positive s and no affine, awaits the
existing `/bb/throw` terminal result, captures raw MCAP and per-goal provenance,
and keeps uncertain/failed throws explicit. Normal Jugglebot configuration is
unchanged. Grid defaults use the Oct 4 sitting-4 logbook's two-ball area;
original launch logs can refine it. Default: 143 targets / 331 throws before
reachability filtering, 50–200 mm spacing, 5 core / 2 outer repeats.

Fixed the comparison artifact's invalid Windows SVG encoding and clipped
bounds; regenerated an equal-scale PNG, inspected visually, plus UTF-8 SVG.
Usage and frame conventions: [local calibration guide](../zTesting/throw_testing/accuracy_testing/LOCAL_CALIBRATION.md).

Verification (2026-10-05): `python -m unittest discover -s
zTesting/throw_testing/accuracy_testing -p test_local_calibration.py -v`:
15 offline tests pass, including production ballistic round trips. No hardware
was commanded. Jetson DDS/action/MCAP integration and actual calibration
accuracy remain to be validated with a pilot session.

Before the hardware sitting, the owner clarified that balls are thrown back
into BB's magazine during calibration. The initial capture-only runner had
no explicit refill segmentation. Added operator-gated refill intervals (default
after every throw), continued ROS servicing during unlimited refill waits,
ground-arrival wait, per-throw analysis windows and a timestamp-association
helper that rejects refill/failed/incomplete/ambiguous intervals. Raw refill
arcs remain in the MCAP for audit; later extraction must use the windows and
not match every ballistic arc by order. The guide now explicitly requires
the normal ROS2 stack running; only its duplicate recorder can be disabled.
