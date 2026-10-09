---
title: Full local calibration after BB reinstall — candidate affine for positive s
type: investigation
date: 2026-10-09
status: tuned
related_entries:
  - 2026-10-07-pilot-trajectory-extraction-fix
  - 2026-10-06-local-calibration-analysis
  - 2026-10-05-mirrored-hand-local-calibration
files_changed:
  - logbook/2026-10-09-bb-local-calibration-result.md
  - logbook/INDEX.md
  - zTesting/throw_testing/accuracy_testing/run_local_calibration.py
  - zTesting/throw_testing/accuracy_testing/analyze_local_calibration.py
  - zTesting/throw_testing/accuracy_testing/test_local_calibration.py
  - zTesting/throw_testing/accuracy_testing/test_analysis_local_calibration.py
  - zTesting/throw_testing/accuracy_testing/LOCAL_CALIBRATION.md
  - zTesting/throw_testing/accuracy_testing/local_validation_plan.json
  - zTesting/throw_testing/accuracy_testing/local_validation_plan.html
external_changes:
  # The production change lives in the Jugglebot repo, branch bb-positive-s-affine-2026-10-09
  # (Jugglebot logbook/2026-10-09-bb-positive-s-and-refitted-aim-correction.md and
  # logbook/2026-10-09-bb-yaw-root-fix.md).
  - "Jugglebot: config/hardware_config.yaml + regenerated artifacts (ball_butler_geometry.yaw_s_offset_mm -105.65 -> +105.65)"
  - "Jugglebot: ros_ws/src/jugglebot/resources/throw_affine_correction.json (the 2026-10-09 refit, validated; the 2026-06-09 matrix archived as throw_affine_correction_2026-06-09_negative_s.json)"
  - "Jugglebot: ros_ws/src/jugglebot/jugglebot/ball_butler_node.py (refuses a matrix fitted for the other sign of s)"
  - "Jugglebot: ros_ws/src/jugglebot/jugglebot/can/throw_ballistics.py + sim/ball_butler/sim.py (yaw solve always takes the positive-range root)"
  - "Jugglebot: sim/juggle_catch.py + sim/juggle_bb_catch.py (the BB-feed sims throw from the measured 2026-10-09 placement)"
  - "Jugglebot: config/generated/admissible_box.yaml (re-swept: hardware_config.py is a gated file)"
subsystem:
  - calibration
  - throwing
tags:
  - accuracy
  - ballistics
  - mocap
---

# Full local calibration after BB reinstall — candidate affine for positive s

## Symptoms

Before this run, BB had to be removed from its shelf, modified and reinstalled. **Only**
session `~/bb_calibration_sessions/20261009T002142_931068Z` is valid for calibration. Every
earlier session, including the 2026-10-07 pilots and the 2026-10-08 partial runs, predates the
reinstall and must not be pooled with it.

## Diagnosis

### Capture

The session is a single, standalone run.

**Setup**
- Status `completed`; no `--resume-from`.
- Solver: `Jugglebot-skills` `throw_ballistics.py` (sha256 `bbb80fa5…`).
- `s = +105.65 mm`; the affine and the feed bias were bypassed.
- Frame translation: `schedule_to_mocap = (0.3, −0.6)`.
- Post-reinstall BB pose: (−975.6, −389.3, 1734.9) mm, yaw offset 0.208°. That is ~1 mm and 0.36° from the 2026-10-07 pose.

**Throws**
- 277/277 released, all on the first attempt: no yaw-NOT_SETTLED retries, no abandoned entries, no rejected captures.
- Duration 57 min with 31 refills. Largest QTM source-stamp gap 38 ms; `recording_needs_review: false`.

**Extraction:** 275/277 accepted.
- The two rejections (throw 74 / cell 90 and throw 77 / cell 116) are `catch_coverage`: the ball was unseen for the last 32–67 ms before the crossing. Both are legitimate.
- Accepted fits:

| metric | range |
|---|---|
| fit RMS | 0.4–3.8 mm |
| local crossing RMS | ≤1.5 mm |
| launch closest approach | ≤22.7 mm |
| acceleration error | ≤428 mm/s² (gate 588) |
| apex occlusion gap | up to 0.417 s (gate 0.45, the closest margin) |

### Results (BB-local frame, 116 cells)

| quantity | value |
|---|---|
| uncorrected per-throw error | mean (+44.3, +9.2) mm, RMS 54.4 mm (core: 47.9 mm, mean (+42.0, +9.8)) |
| forward affine | [[1.0751, 0.0253, −40.80], [0.0186, 1.0153, −16.56]]: gains 1.082 / 1.008, rotation −0.18° |
| correction (desired → command) | [[0.93058, −0.02322, 37.582], [−0.01706, 0.98531, 15.622]] |
| drift over 57 min | quarter means within ±5 mm; linear (−0.2, +3.4) mm. None |
| per-throw scatter (repeats) | σ ≈ 16.2 / 9.8 mm (x / y); 16.5 radial, 9.3 lateral; 2D 19.0 mm (core 20.7) |
| cell-mean noise floor | 13.0 mm RMS |
| held-out cell residual | affine 17.4–17.7; quadratic 17.3; cubic 17.4; mean-offset only 29.5 mm |
| predicted corrected per-throw error (held-out cells) | grid RMS 21.8, median 17.0, p95 38.6 mm; core RMS 21.3 mm |

## Discussion

**What the data shows**
- **The systematic error is affine.** It is stable over the hour, and higher-order models do no better on held-out cells.
- **In the core, the correction should reach the repeatability floor.** The predicted 21.3 mm per throw is close to the 20.7 mm scatter.
- **Some residual remains unexplained.** About 10 mm grid-wide, mostly radial, fits no smooth model. It is either cell-specific or heavier-tailed throw noise. It is small next to the scatter.
- **Repeatability is now the limit, not aim.** The scatter is mostly radial, which points to speed or release variation.

**Not trusted: launch-state comparisons**
- Measured against predicted release velocity reads +10 % speed, +3° elevation, −3.6° azimuth. That is the shape of the earlier "hot launch" artifact.
- It comes from evaluating the arc 49 ms earlier, where v_z is ~480 mm/s higher.
- The azimuth comes from the ~134 mm/s² lateral acceleration. The measured launch direction differs from its own release-to-landing chord by −3.2°, while the landing bearing error is −0.25°.
- Only landings are used.

**Persists after the reinstall:** the arc passes the predicted release point 49 ms early (median; range −69 to −21 ms). The affine absorbs this; the root cause is still open.

## Fix

Candidate: `20261009T002142_931068Z/analysis/correction_candidate.json`. It requires positive s
and replaces the 2026-06-09 affine (do not stack them). Valid for BB-local x 386–1570 mm,
y 195–928 mm, catch z 830 mm, and this mounting of BB only.

The production change (s sign + affine) is prepared separately. It is **not deployed**
until a corrected hardware validation run passes; see Outcome.

### Validation set-up (agreed 2026-10-09; criteria fixed before the run)

- **Runner:** `--apply-correction` maps each desired BB-local target through the candidate and solves the command with s = +105.65.
  - `target_*` stays the desired point; `command_bb_local_mm` records the solved point.
  - Targets outside the fitted region are skipped.
  - The runner refuses a candidate whose s, frame or solver sha differs.
- **Plan:** `local_validation_plan.json` (sha256 `039355da…`, seed 1042).
  - The core is shifted +25/+25 mm, half a grid step, so every target is a position the fit never saw.
  - Each cell is thrown once and the core twice.
  - Offline, against the installed solver and the 2026-10-09 pose: 110 feasible throws over 95 cells, 30 in the core, about 27 min.
  - This replaces "the first 80 throws of a seed-1042 plan" agreed earlier. That would give only 11 core throws, barely above the 10-throw minimum, and would reuse fitted positions.
- **Criteria** (`VALIDATION_CRITERIA`, errors in BB-local mm, measured minus desired):
  - mean within ±6 mm per axis;
  - per-throw RMS ≤26 mm overall and ≤25 mm in the core (predicted 21.8 / 21.3);
  - a verdict needs ≥60 accepted throws, ≥10 core throws and ≥90 % of releases accepted, otherwise INCONCLUSIVE;
  - only PASS justifies deployment.
- **Analysis:** corrected sessions are analysable only with `--extract-only`, so they can never be fitted as uncorrected.

## Outcome

### Validation run (session `20261009T031319_612936Z`, 2026-10-09)

110/110 corrected throws released and accepted (95 cells, 30 core throws, no retries, no
rejections). Errors in BB-local mm, measured minus desired:

| quantity | value | criterion |
|---|---|---|
| mean | (+1.6, −8.2) | within ±6 per axis — **fails on y** |
| per-throw RMS, grid | 21.0 | ≤26 (predicted 21.8) — passes |
| per-throw RMS, core | 20.9 | ≤25 (predicted 21.3) — passes |
| repeat-cell scatter (15 cells) | 14.1 | — |

The verdict against the pre-registered criteria is **FAIL**, on the lateral mean only. The
correction did what the fit predicted: 54 → 21 mm per throw, which is the repeatability floor.

**Where the lateral mean comes from (instrument first).** The two sessions' BB pose
calibrations differ:

| pose field | calibration session | validation session |
|---|---|---|
| position (mm) | (−975.6, −389.3, 1734.9) | (−976.5, −390.6, 1734.3) |
| yaw offset | 0.208° (σ 0.07°) | 0.681° (σ 0.04°) |
| axis tilt | 0.76° | 1.15° |

A +0.47° rotation of the BB-local frame moves a landing at the 0.97 m mean range by −8.0 mm
in local y: the sign and size of the failing mean. Re-expressed in the calibration session's
frame the mean is (−2.9, −1.6) mm and the RMS 19.4 mm. The per-throw regression of y error on
range cannot separate rotation from translation (intercept −5.5 ± 3.9 mm, slope −0.16 ± 0.22°),
so this is consistent with, not proof of, a frame effect; the direct evidence is the pose
calibration itself moving by 0.47° while its own quoted σ is 0.04–0.07°. The yaw offset is the
angle of one anchor marker about the fitted axis point, so an axis-point difference of 1.7 mm
(as seen here) shifts it by 1.7 mm / r: 0.5° if the anchor sits r ≈ 200 mm from the axis. A second-order hint: the radial error has a
slope of −17 ± 7 mm per metre of range, i.e. the fitted 1.082 gain reads ~1.065 in this
session.

**Decision (owner, 2026-10-09): accept the candidate and deploy.** The failing criterion is
an 8 mm lateral mean of the size the pose calibration itself moves between sessions, which no
affine fitted in one session can remove; the RMS criteria pass at the predicted floor. This is
a documented deviation from the pre-registered PASS requirement, not a re-interpretation of
the criteria: the mean check was written to catch a wrong translation column, and the data say
the translation is right in the frame it was fitted in.

**Follow-ups, outside this entry's scope:**
- Per-session yaw-offset repeatability of the BB pose calibration (0.47° here ≈ 8 mm at 1 m).
  Check the anchor-marker radius and the axis-fit residuals across the two sessions' arcs.
- The ~48 ms arc-time offset (unchanged).

### Production change

Prepared and gated in the Jugglebot repo (`external_changes` above): s = +105.65, the matrix
installed with its validation recorded in the resource provenance, the box re-swept, the
BB-feed sims moved to the measured placement, and the yaw-root fix. Deployment steps
(merge, build, check the node's `Aim correction source` line, a handful of throws) are in the
Jugglebot entry.

**Solver sha after the yaw-root fix.** The runner's `--expect-solver-sha bbb80fa5` refers to
the solver before the fix. The fix changes no command inside the calibrated region (every
target there takes the forward root), so `run_local_calibration.py` now judges a solver file
whose sha differs from the candidate's by re-solving the fit session's recorded throws and
demanding identical solutions, instead of refusing on the sha alone.
