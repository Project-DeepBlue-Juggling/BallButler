---
title: "Throw aim-correction (2D affine) validated on hardware — ~84% error reduction"
type: feature
date: 2026-06-10
status: resolved
phase: "V0 throw accuracy — aim-correction bring-up"
files_changed:
  - zTesting/throw_testing/accuracy_testing/fit_affine.py
  - zTesting/throw_testing/accuracy_testing/throw_affine_correction.json
  - zTesting/throw_testing/accuracy_testing/2026-06-10-BB-calibration-32-throws-CORRECTED.json
external_changes:
  # The V0 thrower's ROS2 control stack lives in the Jugglebot repo.
  - "Jugglebot 27dbc06: ros_ws/src/jugglebot/resources/throw_affine_correction.json (deployed matrix; install copy regenerated on build)"
  - "Jugglebot 36a148a: config/protocol_config.yaml + regenerated protocol_config.py (mocap alignment gate relaxed — enabling fix)"
commits:
  - d681115   # fit tooling (Py3.8 fix) + deployed matrix + residual analysis
subsystem:
  - calibration
  - throwing
  - tracking
tags:
  - accuracy
  - mocap
  - ballistics
  - testing
---

# Throw aim-correction (2D affine) validated on hardware — ~84% error reduction

## Summary

The V0 thrower lands balls a systematic ~195 mm from where it is aimed. A 2D
affine correction, fit from one accuracy-calibration session and applied in the
thrower's local frame, cuts that to ~31 mm — an **84 % reduction in mean aim
error**, measured on a fresh 32-throw validation session. The correction landed
within 3 mm of its own out-of-sample prediction (predicted 27.5 mm, measured
30.9 mm per cell), confirming it generalises rather than overfits. The remaining
error is now dominated by throw-to-throw repeatability (~34 mm scatter), not
systematic aim, so a second-stage refit buys little; the aim map is considered
done for now.

## Motivation

The thrower's open-loop pipeline — `global_to_bb_local` → `solve_throw_local`
(inverse ballistics) — assumes an idealised model of the machine (forward
kinematics, release point, launch timing). Real hardware departs from that model
in ways that are stable run-to-run but hard to attribute to any single physical
cause. The net effect is a **systematic landing bias**: across a grid of
commanded targets, balls land consistently offset and slightly sheared from
where they were aimed (mean ~195 mm in BB-local frame).

Rather than chase each physical contributor, the approach is empirical: command a
known grid of throws, measure where the balls actually land with motion capture,
and fit a single correction map that warps the commanded aim point to cancel the
measured bias. This entry records the process, how the correction is applied at
runtime, and the measured improvement.

## Design

### The correction: a 2D affine in BB-local frame

The thrower aims in its own **BB-local frame** (origin at the thrower, x/y in the
ground plane after removing the mocap yaw offset). The correction is a 2×3 affine
applied to the local (x, y) aim point, between the global→local transform and the
inverse-ballistics solve. In `ball_butler_node.py::_throw_at_global_point`:

```
x_l, y_l, z_l = global_to_bb_local(x_g, y_g, z_g, bb_position_mm, yaw_offset_rad)
x_cmd, y_cmd  = _apply_aim_correction(x_l, y_l)     # <-- the correction
sol           = solve_throw_local(x_cmd, y_cmd, z_l)
```

`_apply_aim_correction` is exactly:

```
(a, b, tx), (c, d, ty) = matrix
return a*x_l + b*y_l + tx,  c*x_l + d*y_l + ty
```

Key properties:

- It maps **measured-landing-local → commanded-local**: i.e. it answers "given
  that aiming at P makes the ball land at Q, where should I aim so the ball lands
  at P?" The fit learns the measured→intended map and the node applies it to the
  intended point before solving.
- **z is not corrected.** The affine is purely 2D in the ground plane. Range/height
  bias is absorbed implicitly by the (x, y) warp at the fit plane only — see Open
  Questions for the height-extrapolation caveat.
- The same correction is applied in three places that must stay consistent:
  the named-target throw service, the accuracy-calibration scheduler's
  reachability pre-filter, and the calibration throws themselves. All route
  through `_apply_aim_correction`.

The currently deployed matrix (fit from the 2026-06-09 session, 41 pairs):

```
[ +0.8665  +0.1112  +116.7388 ]
[ -0.1084  +1.0635  -108.4403 ]
[  0        0          1      ]
```

Near-unit scale with a small shear and a ~(117, −108) mm translation — i.e. the
bias was mostly a rigid offset plus a few percent of scale/rotation.

### The calibration loop

The end-to-end process, which this run exercised twice (once to fit, once to
validate):

1. **Localise the thrower.** `bb/calibrate` → `mocap_node` computes the
   thrower's mocap position and yaw offset and latches them to
   `ball_butler_node` (`_bb_position_mm`, `_bb_yaw_offset_rad`). All aiming is
   relative to this pose.
2. **Run an accuracy session.** `bb/start_accuracy_calibration` →
   `ball_butler_node` builds a seeded, **reachability-filtered** grid schedule
   (params `calib_grid_center_mm`, `calib_grid_size_mm`, `calib_grid_divisions`,
   `calib_throws_per_cell`, `calib_random_seed`), writes the schedule + thrower
   pose + whether a correction was loaded to
   `~/bb_calibration_sessions/accuracy_session_<stamp>.json`, then throws the
   shuffled schedule one ball at a time. Grid used: 5×5 over 1000×1000 mm
   centred at (0, 0, 750) mm.
3. **Capture landings.** QTM records each ball's trajectory; the operator exports
   a (cleaned) trajectory JSON to
   `zTesting/throw_testing/accuracy_testing/`.
4. **Fit.** `fit_affine.py <session.json> <qtm_export.json> --out <matrix.json>`:
   - Detects each ball's landing as the first descending crossing of the target
     plane (z = grid-centre z).
   - Pairs landing *i* ↔ scheduled throw *i* **by throw order** (no manual
     cluster-mapping).
   - Transforms both commanded targets and measured landings into BB-local frame
     using the session's recorded thrower pose, groups by grid cell, takes the
     per-cell mean landing, and least-squares fits the 2×3 affine
     measured-local → commanded-local.
   - Writes the matrix in the exact schema `ball_butler_node` consumes
     (`matrix` + `n_pairs` + `provenance`) and prints a before/after error report.
5. **Deploy.** Copy the matrix to the Jugglebot control stack at
   `ros_ws/src/jugglebot/resources/throw_affine_correction.json` **and** the
   install-space copy (`ros_ws/install/.../resources/...`), then launch with
   `apply_aim_correction:=true`.
6. **Validate.** Re-run an accuracy session with the correction live, re-fit, and
   read the new "before" error — that is the residual accuracy of the corrected
   machine.

A subtle but important property of step 6: running `fit_affine.py` on an
**already-corrected** session reports the residual error as its "before" number,
and what a *second* correction stacked on top would achieve as its "after"
number. That is how today's run was scored.

## Implementation — what was run today

- **Fit** (from the 2026-06-09 uncorrected, 42-throw session; 41 valid pairs):
  ```
  python3 fit_affine.py \
    ~/bb_calibration_sessions/accuracy_session_2026-06-09_16-48-27.json \
    2026-06-09-BB-calibration-42-throws.json \
    --out throw_affine_correction.json
  ```
- **Deploy.** Matrix committed to the Jugglebot `resources/` source
  (`27dbc06`); the install copy is regenerated on build (old install matrix
  backed up to `*.bak-2026-05-24`).
- **Enabling fix (mocap).** The validation run wouldn't publish
  `/rigid_body_poses` because the "Base" rigid body sits ~444 mm off the QTM
  origin and failed the tight alignment gate (2.5 mm / 1.0°). The gate only
  un-blocks publishing — it transforms no poses and the thrower calibration does
  not use it — so the thresholds were relaxed to 2000 mm / 180° at the
  `config/protocol_config.yaml` source and regenerated, rather than re-zeroing
  the QTM frame (which would have perturbed the thrower calibration). A
  **deliberately temporary** change (Jugglebot `36a148a`); restore 2.5 / 1.0 if
  the QTM frame is ever re-zeroed.
- **Validation session.** 32 throws (16 reachable cells × 2), seed 42, correction
  live (`aim_correction_source` = the deployed install matrix), exported as
  `2026-06-10-BB-calibration-32-throws-CORRECTED.json`.
- **Score.** Re-ran `fit_affine.py` on the corrected session (output kept as
  `throw_affine_correction_v2_residual.json` for the record — analysis only, not
  deployed).

## Verification — how much it helped

Error is Euclidean distance between commanded target and measured landing, in
BB-local frame.

| Metric (per grid cell) | Uncorrected (2026-06-09) | Corrected (2026-06-10) |
|---|---|---|
| Mean error | **194.6 mm** | **30.9 mm** |
| Max | 224.8 mm | 139.7 mm |
| Std | — | 30.0 mm |

**~84 % reduction in mean aim error.** Per *individual throw* (not cell-averaged)
the corrected run was mean 33.5 mm, median 25.2 mm, std 36.8 mm.

Three findings support trusting this number:

1. **It generalises.** The 2026-06-09 fit *predicted* 27.5 mm residual; the fresh
   2026-06-10 run *measured* 30.9 mm per cell. Landing within 3 mm of an
   out-of-sample prediction means the affine captured a real, stable bias — not
   noise it was fit to. This was a clean same-plane test (both at z = 750 mm).
2. **The residual is now mostly random, not systematic.** Decomposing the
   corrected residual: systematic bias is only ~13.5 mm (mean residual vector
   −8.0, −10.8 mm); per-axis scatter is ~34 mm; within-cell repeatability (two
   balls to the *same* target) runs 5–30 mm. The dominant term is the machine's
   throw-to-throw repeatability, which no aim map can remove.
3. **Refining further is low-value.** A second-stage affine fit on the corrected
   data recovers only 7 % more (30.9 → 28.7 mm per cell), most of it chasing the
   ~13 mm bias that is within run-to-run noise. Not worth a re-deploy.

One genuine outlier inflates the corrected max/std: cell 10 (throws 20 & 23) at
139.7 mm cell-mean, with one throw at 222 mm and the two throws disagreeing by
~165 mm — a bad throw (reload/release/mistrack), not an aim error. Excluding that
single cell, the corrected system sits at ~26 mm mean.

**Conclusion:** the correction is validated and deployed. The next lever for
accuracy is throw repeatability (release timing / reload consistency), not the
aim map.

## Open Questions / Follow-ups

1. **Height extrapolation.** The fit and validation are both at z = 750 mm. The
   affine is 2D, so at the catching-cone height (lower than 750 mm) the
   correction is an extrapolation with no z-bias term. Confirm with a few throws
   *at cone height* before trusting it there.
2. **Repeatability is the new bottleneck.** ~34 mm per-axis scatter is now the
   limiting factor. A separate investigation into release-timing / reload
   consistency would move the needle more than any aim-map change.
