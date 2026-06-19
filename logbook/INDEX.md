# Logbook Index

All logbook entries, newest first. Maintained by hand — add a row when you add an
entry. See [README.md](README.md) for the format and [TEMPLATE.md](TEMPLATE.md)
to start a new entry.

## Chapters

Narrative syntheses that read a related block of entries top-to-bottom — the
development, the significant developments/insights, and the open problems. A
chapter is a **reading lens, not a source of record**: it links down to the
entries, which stay authoritative. Open the `.html` in a browser.

| Span | Chapter | Covers |
|------|---------|--------|
| 2026-06-10 → 2026-06-18 | [01 · V0 Throw Accuracy](chapters/01-v0-throw-accuracy.html) | The spatial (aim-correction) + temporal (release-latency) campaign — all 6 entries below |

## Entries

| Date | Status | Type | Subsystem | Title | Entry |
|------|--------|------|-----------|-------|-------|
| 2026-06-18 | resolved | investigation | throwing, firmware, calibration, timing | Temporal accuracy resolved (≈44 ms → <10 ms): aim-correction (spatial) + measured release-latency offset (temporal); kinematic-ID & feedforward root-fixes ruled out | [temporal-accuracy-resolved-fractured-solution](2026-06-18-temporal-accuracy-resolved-fractured-solution.md) |
| 2026-06-17 | resolved | investigation | throwing, firmware, calibration | Release-lag fix validated on hardware (δ 140→44 ms); residual is a +3° steeper / +10% hot launch | [release-lag-fix-validated-launch-discrepancy](2026-06-17-release-lag-fix-validated-launch-discrepancy.md) |
| 2026-06-12 | resolved | investigation | throwing, timing, firmware | Temporal "warm-up drift" is a clock-sync artifact (bridge time-master re-acquisition slew), not a thrower effect — root-caused + fixed (2026-06-16) | [temporal-warmup-drift](2026-06-12-temporal-warmup-drift.md) |
| 2026-06-11 | resolved | investigation | throwing, timing, firmware | Release-lag root cause confirmed — two decel-zero mechanisms cancel; 2-line fix (deployed + validated 2026-06-17) | [release-lag-firmware-analysis](2026-06-11-release-lag-firmware-analysis.md) |
| 2026-06-10 | resolved | feature | calibration, throwing, timing | Temporal-accuracy test protocol (v1) + catching-cone uplink plumbing | [temporal-accuracy-protocol](2026-06-10-temporal-accuracy-protocol.md) |
| 2026-06-10 | resolved | feature | calibration, throwing, tracking | Throw aim-correction (2D affine) validated on hardware — ~84% error reduction | [throw-aim-correction-validated](2026-06-10-throw-aim-correction-validated.md) |
