---
title: BB yaw-offset gauge pin settled from landings — RE_PIN by −0.179° (0.208 → 0.029°) on the 23:34 range-weighted sitting; settle_yaw_gauge.py now evaluates the affine criteria itself when the report has no checks
type: investigation
date: 2026-10-10
status: resolved
phase: ""
related_adr: ""
related_entries:
  - 2026-10-09-yaw-gauge-settle-tooling
  - 2026-10-09-bb-local-calibration-result
  - 2026-10-10-bb-hand-torque-ff-scale
files_changed:
  - zTesting/throw_testing/accuracy_testing/settle_yaw_gauge.py
  - zTesting/throw_testing/accuracy_testing/test_settle_yaw_gauge.py
  - logbook/2026-10-10-bb-yaw-gauge-pin-settled.md
  - logbook/INDEX.md
external_changes:
  - "Jugglebot: ros_ws/src/jugglebot/resources/bb_marker_template.json (gauge.pinned_yaw_offset_deg 0.208 -> 0.029, gauge.repinned_on records the evidence; branch bb-yaw-gauge-repin-2026-10-10, not merged or deployed) + the two shipped-template tests and logbook 2026-10-09-bb-constellation-yaw-offset (\"Pin settled\")"
subsystem:
  - calibration
  - tooling
tags:
  - accuracy
  - mocap
---

# BB yaw-offset gauge pin settled from landings

## Summary

Two corrected sittings on 2026-10-10 after the hand-FF fix (FW 7 / FW 8) measured the constellation estimator's yaw-offset gauge against landings with `settle_yaw_gauge.py`. The 23:34 sitting (47 accepted) is the first to meet the pre-registered rule's ≥ 30 throws: mean bearing −0.179 ± 0.099°, outside ±0.15°, so **RE_PIN by −0.179°**: Jugglebot's `gauge.pinned_yaw_offset_deg` 0.208 → **0.029°**. The 19:23 sitting (28 accepted, below the rule) and the pool agree. While reading the 23:34 output, the script's affine line said "OUTSIDE criteria" for an RMS (24.6 mm) inside the 26 mm criterion. The report's validation was INCONCLUSIVE on counts and so had no `checks`. The script now evaluates the criteria itself.

## Symptoms

- 23:34 output: `RE_PIN … add −0.179 deg … confirm the node reports ~0.626 deg`, then `affine per-throw RMS 24.6 mm: OUTSIDE criteria`. The RMS 24.6 mm is inside the 26 mm criterion, so the second line was wrong.

## Diagnosis

**The pin.** Both sittings ran on the base-frame pose path. The 19:23 used offset 0.790° matches heading + κ pooled from the 24 records then held, to 0.0001°.

| Sitting (local) | Session | Plan | Accepted | Node used | Mean bearing ± SE | Lateral ± SE | Mean range | Rotation fit: slope / intercept | Verdict |
|---|---|---|---|---|---|---|---|---|---|
| 2026-10-10 19:23 (FW 7) | `20261010T082544_262394Z` | `local_validation_plan.json`, 40 throws | 28 | 0.790° | −0.278 ± 0.103° | −5.3 ± 2.1 mm | 1149 mm | −0.15 ± 0.47° / −2.3 ± 9.7 mm | INCONCLUSIVE (< 30) |
| 2026-10-10 23:34 (FW 8) | `20261010T123404_756897Z` | `local_validation_plan_range.json` (8 near cells at 600/700 mm, 8 far at 1450/1550 mm, shared bearings 10–38°, 48 throws) | 47 (1 rejected: catch coverage) | 0.804° | **−0.179 ± 0.099°** | −4.2 ± 1.6 mm | 1065 mm | −0.50 ± 0.21° / +5.0 ± 4.2 mm | **RE_PIN −0.179°** |
| pooled (corroboration) | both | — | 75 | (0.804°, first) | −0.216 ± 0.073° | −4.6 ± 1.3 mm | 1096 mm | −0.44 ± 0.19° / +3.7 ± 3.8 mm | (RE_PIN −0.216°) |

How to read the rotation-vs-translation fit at 23:34 (lateral error vs range):
- A pure frame error is a slope only. The slope −0.50 ± 0.21° agrees with the −0.18° rotation within ~1.5σ.
- The intercept +5.0 ± 4.2 mm is consistent with zero.
- The range-weighted plan (near and far cells at shared bearings) was built to separate the two, and it does not reject a pure rotation.
- A residual range-dependent lateral term (slope beyond the bearing mean) is not excluded. It would belong to the affine, not the pin.
- Unlike the 15:00 sitting (intercept +46 ± 11 mm, RE_PIN not applied), nothing here says the bearing rule is reading a translation.

**The affine line.** `analyze_local_calibration.validation_verdict` returns INCONCLUSIVE without a `checks` block when counts are short (47 < 60 accepted, 0 < 10 core). `settle()` read the absent checks as failed.

## Fix

- **Pin (Jugglebot, see external_changes):** 0.208 − 0.179 = 0.029°, applied by the rule per sitting with ≥ 30 throws. The pooled −0.216° is corroboration only. An offline replay of the sitting's own calibration sweep through the real `mocap_node._finalize_calibration` (bag `2026-10-10_23-32-12`, `~/bb_calibration_sessions/yaw_gauge_repin_20261010/`) gives 0.8045° under the old pin, which is the live value. Under the new pin it gives **0.6255°**. Pooled κ moves by exactly −0.179°.
- **`settle_yaw_gauge.py`:** `affine_checks()` returns the report's `checks` when present. Otherwise it evaluates `criteria` against the measured `rms_mm`, `core_rms_mm` (null → not applicable, passes) and `mean_mm`. The output names the numbers and the source, for example `within criteria (RMS 24.6 <= 26, core n/a; computed from criteria (report verdict INCONCLUSIVE))`. It is not a validation verdict, which also needs the counts. When the mean check fails, a second line splits it radial/lateral. The settlement also records `radial_mean_mm` and `affine_checks`.

## Outcome

- The 23:34 sitting now reads: RMS 24.6 mm within criteria (core n/a).
- Mean landing error (−8.1, −8.7) mm is outside the 6 mm per-axis criterion. It is mostly radial (−11.3 ± 2.7 mm short) and only −4.2 mm lateral, so **the re-pin moves only ~3 mm of it**. A short radial bias of ~11 mm is the affine's concern; watch it at the next validation with ≥ 60 throws.
- Tests: `/usr/bin/python3 -m pytest -q test_settle_yaw_gauge.py` → 6 passed (2026-10-11), including the new INCONCLUSIVE-report case.

## Open Questions / Follow-ups

1. **Confirming sweep (owner):** after the Jugglebot branch is merged and deployed, run one sweep with the base markers seen. Do not set `bb_moved`. The node should report ≈ 0.62–0.63°: 0.6255° at the sitting's base heading, 0.6207° at the 23:46 sweep's.
2. The radial −11 mm mean and the slope beyond the bearing are for the next full validation (≥ 60 accepted, core cells), not the pin.
3. `gauge.pin_uncertainty_deg` still reads 0.2° (reported, not used in σ). The landing SE is now ~0.1°; update it with the next template edit if wanted.
