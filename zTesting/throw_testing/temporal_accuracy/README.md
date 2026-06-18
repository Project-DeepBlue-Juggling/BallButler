# Temporal accuracy — test protocol (v1)

Validates Ball Butler's **temporal** accuracy: does the ball arrive when the
system says it will? The spatial campaign (../accuracy_testing/) validated
*where* balls land; this protocol validates *when*, using the sensorized
catching cone as a mocap-free, microsecond-grade arrival sensor.

## Measurand

**End-to-end arrival error** per throw:

```
arrival_error = t_catch − t_landing_predicted
```

- `t_landing_predicted` — `landing_time` from the `ThrowAnnouncement`
  published by `ball_butler_node` at command time
  (= now + throw_delay_s + predicted ToF from the inverse-ballistics solver).
- `t_catch` — wall-clock instant the ball strikes the bottom of the catching
  cone, latched in the cone Teensy's piezo ISR with its clock IIR-locked to
  the system-wide 0x7DD time-sync broadcast (µs-level within the Teensy
  domain; the can-bridge anchors that domain to the Jetson's CLOCK_REALTIME).

This is deliberately end-to-end: it integrates every contributor — ROS →
UDP → CAN command latency, firmware wall-clock scheduling, release lag,
ToF-model error, and the cone-entry → piezo fall-through time. Decomposing
release error vs flight error requires QTM alongside
(../accuracy_testing/timing_analysis.py) and is out of scope for v1.

### The systematic offset δ

The protocol treats the total systematic as **δ**, estimated as the mean
arrival error of the repeatability block and subtracted before judging
accuracy. δ's *stability* across delays and distances is itself a result.

Measured composition (2026-06-10 campaign, δ = +126.5 ms — full analysis in
`logbook/2026-06-10-temporal-accuracy-protocol.md` + `decompose_results.json`):
**+82 ms** release lag (firmware decel-zero defect: ball leaves at
throw_time + 0.273/v), **+51 ms** flight stretch (ball releases ~10% hot in
vz), **−7 ms** ball-radius-early piezo contact. Geometry contributes almost
nothing — the cup surface sits at the rigid-body-origin plane, and drag is
<1 ms. Both dominant terms are thrower-side and speed-dependent (expect δ to
drift across a distance sweep), and both are fixable — but fixing them moves
the release point/velocity and **invalidates the deployed spatial
aim-correction** (re-fit required).

## Measurement chain (must be live before any session)

```
piezo ISR → cone Teensy (CAN2, CATCH_EVENT 0x7E0, wall-µs latched at impact)
          → can-bridge Teensy (CONE_FRAME UDP relay, ≤10 ms forwarding)
          → teensy_bridge_node → ROS topics cone/catch_event + cone/heartbeat
```

Forwarding latency does **not** enter the measurement — the timestamp is
latched at the ISR; only the cone's time-sync quality matters. Command
granularity is ~1 ms (16-bit epoch-ms CAN encoding of the throw time), so the
cone out-resolves the thing being measured by ~3 orders of magnitude.

## Preflight (gates; the runner enforces these)

1. Jugglebot stack up: `teensy_bridge_node`, `ball_butler_node`, `mocap_node`
   with the `Catching_Cone` rigid body tracked (mocap is used to *aim*, not
   to measure).
2. `cone/heartbeat`: `connected`, state READY, `time_synced=true`,
   `sync_rms_us ≤ 50` (warn above).
3. `bb/heartbeat`: `connected`, `ball_in_hand=true`, hopper loaded.
4. **Tap test**: `python3 run_temporal_session.py --listen`, tap the cone
   bottom — a catch event must print with a timestamp within ~1 s of "now"
   (the ~30 ms report deferral + UDP hop are normal), sequence incrementing
   per tap, and retriggers within 500 ms suppressed.

## Phases

Run R first (it calibrates δ), then D, then X. One ball type throughout;
record any hardware change in the session label.

| Phase | Purpose | Procedure |
|-------|---------|-----------|
| **R — Repeatability** | Throw-to-throw timing scatter; calibrates δ | N≥30 identical throws, nominal config (`--phase repeatability --throws 30 --delay 2.5`) |
| **D — Delay sweep** | Wall-clock scheduling vs lead time | Fixed cone; delays 2.0–5.0 s, ≥6 throws each, randomized order (`--phase delay-sweep --delays 2.0 2.75 3.5 4.25 5.0 --throws-per 6`) |
| **X — Distance sweep** | Constant vs ToF-proportional error | ONE interactive run: 5 throws to the current cone position, then the runner pauses (press Enter) while you move the cone — vary distance **and height** — repeating for 20 positions (`--phase distance-sweep --positions 20 --throws 5 --delay 2.5`). Gates are re-checked after every move; a failed throw (e.g. QTM lost the moved body) offers retry instead of aborting. q at any prompt ends the session early, keeping all data. |

Throws auto-pace on BB's reload cycle (same heartbeat gating as the spatial
calibration session). A session aborts cleanly if the hopper runs dry; data
written so far is preserved (JSON is rewritten after every throw).

## Data handling

- Sessions land in `sessions/temporal_<phase>[_<label>]_<stamp>.json` —
  per-throw records: commanded delay, full service response (yaw/pitch/
  speed/ToF/targets), the announcement's `throw_time`/`landing_time`, the
  catch event (timestamp, sequence, flags), and `arrival_error_ms`.
  Distance-sweep throws additionally carry `position`/`position_idx`
  (`pos01`…); the analyzer groups per position automatically.
- **Exclusions are recorded, never silently dropped**: `miss` (no catch
  within ±1.5 s of predicted landing — ball missed the cone or sensor failed
  to trigger), `unsynced` (cone lost time-sync — timestamp is host-arrival,
  not impact truth), `throw-failed`. A phase with >20 % misses is invalid —
  fix aim (re-run spatial validation) or cone placement first.
- Rim-graze ambiguity: a wall hit can trigger the piezo early. The
  `retrigger_suppressed` flag marks extra edges in the 30 ms report window;
  treat flagged throws as suspect when they are outliers.
- Resolved anomaly (2026-06-10 bench, 1/18 taps): a cone-firmware ISR race
  (`micros64()` false 32-bit wrap) could put a future-biased timestamp on a
  catch sent within ~1 s of a wrap — root-caused and fixed the same day, see
  Jugglebot `logbook/2026-06-10-cone-micros64-false-wrap.md`. The ±1.5 s
  match window still rejects any recurrence (shows as a `miss` with a catch
  event present); a miss rate dominated by that signature means the cone
  firmware regressed.

## Analysis & pass criteria

```
/usr/bin/python3 analyze_temporal_session.py sessions/temporal_delay-sweep_*.json \
    --offset-from sessions/temporal_repeatability_<stamp>.json --plot
```

With δ from phase R removed, the system **passes** when every group (each
delay bin in D, each position in X, and R itself) shows

```
|mean(arrival_error) − δ| ≤ 10 ms
```

Also report (characterization, no gate in v1): scatter σ per group, the
trend slopes vs commanded delay (scheduling-chain health — should be ~0)
and vs predicted ToF (flight-model health), and the miss/unsynced counts.

Note for the 20×5 distance sweep: at n=5 per position each group mean
carries ~±4 ms standard error (σ ≈ 9 ms from phase R), so treat individual
per-position verdicts as advisory — the primary phase-X signal is the trend
vs predicted ToF (plus the per-group v/ToF context the analyzer prints)
across all ~100 throws.

## Next step (out of scope for v1): timing correction

If a stable bias structure emerges (e.g. δ component scaling with ToF), fit
a correction mirroring the spatial affine flow: estimate offset(+slope vs
ToF) from one campaign, apply it to `predicted_tof_sec`/`landing_time` in
`ball_butler_node` (analogous to `_apply_aim_correction`), then validate on
a fresh campaign against the same ±10 ms criterion. Decomposition into
release vs flight error (QTM + `timing_analysis.py` alongside a cone
session) is the follow-up if the residual resists a simple model.
