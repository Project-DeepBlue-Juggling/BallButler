---
title: "Temporal 'warm-up drift' is a clock-sync measurement artifact (bridge time-master re-acquisition slew), not a thrower effect — root-caused + fixed"
type: investigation
date: 2026-06-12
status: resolved   # 2026-06-18 — clock fix flashed to both Teensys + committed; residual δ now corrected (see 2026-06-18 entry)
phase: "V0 throw accuracy — temporal validation (phase X distance sweep)"
related:
  - 2026-06-10-temporal-accuracy-protocol.md
  - 2026-06-11-release-lag-firmware-analysis.md
files_changed:
  - zTesting/throw_testing/temporal_accuracy/read_bag_sync_traces.py     # new: standalone mcap sync-trace reader
  - zTesting/throw_testing/temporal_accuracy/analyze_release_from_bag.py  # new: mocap release velocity/timing from a bag
external_changes:
  - "Jugglebot: ros_ws/src/jugglebot/jugglebot/mocap_node.py (decoupled /rigid_body_poses publishing from Base alignment — enabling infra for this Base-absent session; uncommitted)"
  - "Jugglebot: Teensy_code_canbridge/{axis_state.h,axis_state.cpp,can_buses.cpp,telemetry.cpp} + jugglebot/teensy_bridge_node.py + launch/jugglebot_launch.py (NEW: forward BB pitch/hand ODrive telemetry CAN1→UDP→/bb/odrive_diag; bridge Teensy reflashed; uncommitted)"
  - "Jugglebot: Teensy_code_canbridge/time_base.{h,cpp} (FIX: step on large re-acquisition error + honest staleness in time_synced(); bridge reflashed 2026-06-16; uncommitted)"
  - "Jugglebot: CatchingCone_code/CatchingCone_code.ino (FIX: step-on-jump in handleSyncFrame + is_time_synced() honest FLAG_TIME_SYNCED; BUILT not flashed — cone not connected; uncommitted)"
subsystem:
  - throwing
  - timing
  - firmware
tags:
  - warm-up
  - release-lag
  - repeatability
  - time-sync
  - testing
  - odrive-telemetry
---

# Temporal 'warm-up drift' is a clock-sync measurement artifact (bridge time-master re-acquisition slew), not a thrower effect

> **RESOLVED 2026-06-16 (root cause + fix below).** The "warm-up drift" is a
> measurement artifact: the can-bridge time master **slews** (instead of stepping)
> when it re-acquires the Jetson clock after a sync gap (teensy_bridge_node down),
> so its 0x7DD broadcast — and every slave that follows it, the cone included —
> converges over ~30 min, masquerading as a thrower warm-up. The throw is
> temporally **stable** throughout (mocap δ = +140.6 ± 10.6 ms). Fix implemented:
> step-on-large-re-acquisition + honest staleness (bridge, flashed) and
> step-on-jump + honest sync flag (cone, built). The earlier "thrower-side timing
> drift" reading (06-12) is **superseded**.
>
> **2026-06-18:** the clock fix is now flashed to both Teensys, and the residual
> constant δ (~44 ms) is itself corrected — see
> [2026-06-18-temporal-accuracy-resolved-fractured-solution](2026-06-18-temporal-accuracy-resolved-fractured-solution.md).
> The cone piezo no-fire on close catches (#6) is carried to that entry's open questions.

## Summary

The phase-X distance sweep (`temporal_distance-sweep_2026-06-12_11-07-44`, 20
cone positions, 106 throws) *looked* like arrival error grows with catch
distance. It does not. Arrival error tracks **session time, not geometry**: it
starts near **0 ms cold** (pos01, +4.7 ms) and climbs asymptotically to a
**+115 ms plateau over the first ~10 min**, then is flat (±4.7 ms) and
**geometry-independent**. The drift is a **real thrower-side warm-up effect** —
the cone timing chain is exonerated by its own sync telemetry (state READY,
`time_synced`, 2 µs jitter from second 0). Mocap ground truth (Tier 0) shows it
is a **release-timing** drift (launch velocity flat). A PM mechanism campaign
then **ruled out** temperature (hand/FET), throw-duty heating, BB power-on, and
ROS/software state: the ramp fires only **once per long idle** and then holds all
day. Two hypotheses remain — **mechanical seating / long-thermal-mass** (operator-
favoured) vs **bridge-Teensy firmware-state** — distinguished by two deferred
tests. **Practical outcome: the warm plateau is 129.5 ± 5.2 ms, so a fixed +130 ms
feed-forward δ meets the ±20 ms goal** with no temperature model.

> The 06-16 run below resolved which hypothesis was right — neither, exactly: it's
> a clock-sync artifact root-caused to the **bridge** (not the cone, not the
> thrower). See RESOLUTION.

## RESOLUTION (2026-06-16) — clock-sync artifact, root cause in the bridge

After ~4 days idle (Jetson + bridge powered continuously; teensy_bridge_node
**down** over the break) a fresh run rammped **−447 → +120 ms over ~35 min** —
bigger and slower than before. That large, clean ramp let mocap lift the signal
clear of the noise that made 06-12 inconclusive:

- **Decomposition** `cone_AE = A(cone_catch − mocap_cross) + B(mocap_cross − predicted)`:
  **A (cone clock) carries the whole ramp** (slope +13.4 ms/min, r=0.88); **B (real
  physical arrival, mocap = independent clock) is flat at +140.6 ± 10.6 ms** (slope
  −0.02). The throw never drifted.
- **Mocap-free confirmation**: `host_receive − cone_catch_time` decayed +623 → +52 ms
  (toward the true forwarding latency) — the cone's timestamps were ~570 ms early at
  the start, slewing to true, while it reported `synced=True` throughout (its
  `sync_rms` tracks *jitter*, not mean offset).

**Root cause (in code), not the cone:** the 0x7DD broadcast is 100 Hz, so the cone's
1/8 IIR would correct 600 ms in <1 s — yet it took 30 min, and the cone's 2 µs
jitter proves it was tightly locked to the broadcast. The slow-converging error was
in the **broadcast**. `Teensy_code_canbridge/time_base.cpp::set_wall_anchor` STEPS
only on the first anchor; afterwards it **slews** at `diff >> TIME_OFFSET_IIR_SHIFT`
(gain 1/16) and re-anchors only every `TIMEOFDAY_RESYNC_MS = 30 s` → ~8-min time
constant. While teensy_bridge_node was down, the bridge's TOD-RPC went unanswered
and its wall clock free-ran ~600 ms (it kept broadcasting that drift; the cone
followed, locked). On restart it **slewed** back over ~30 min. Fits all sessions:
06-12 morning (overnight gap, ~115 ms, ~10 min); my 13:22 flash *rebooted* the
bridge → first anchor *stepped* → 06-12 PM flat; warmupD (seconds-long gap, ~ms) →
flat; 06-16 (4-day gap, ~600 ms) → ~30 min.

**Why ~120 ms (cone, last week) vs ~140 ms (mocap, today):** the cone's absolute
reading is only correct once the *bridge* clock has fully converged (≥30–50 min with
the old slow slew — longer than those sessions ran), so the cone consistently
*under-read*. Mocap +140 ms is convergence-independent = the true δ. The throw delay
did not change (≈ within measurement uncertainty); don't trust the cone's absolute δ
unless the bridge is fully converged (or use mocap).

### Fix (implemented 2026-06-16)

1. **Bridge `set_wall_anchor` (`time_base.cpp/.h`) — primary, FLASHED:** STEP when
   `|anchor − offset| > TIME_STEP_THRESHOLD_US` (20 ms) — boot / re-acquisition after
   a gap; else slew (small drift). Collapses the 30-min ramp to one RPC (<1 s).
2. **Bridge `time_synced()` honest staleness — FLASHED:** returns false once the
   anchor is older than `TIME_ANCHOR_STALE_US` (90 s). Self-gates the 0x7DD broadcast
   (so slaves go unsynced instead of following a drifting master) and triggers the
   existing 500 ms fast-retry → sub-second recovery.
3. **Cone `handleSyncFrame` step-on-jump + honest flag (`CatchingCone_code.ino`) —
   BUILT, not flashed (cone not connected):** step when the broadcast jumps > 20 ms
   (re-lock in 1 frame vs ~0.5 s slew); `FLAG_TIME_SYNCED` now gated on a *fresh*
   broadcast (`is_time_synced()`, 1 s) so a stopped/stale master reads unsynced.

Verified: both firmwares build clean; bridge flashed and healthy (`/teensy/link_status`
`time_synced=1`, `fault_state NONE`). **Pending:** flash the cone when connected;
operator re-acquisition verification (stop teensy_bridge_node ≥2 min, restart, throw
immediately → expect flat at the true δ from throw 1, no ramp); commit.

**Architecture notes (operator Qs):** (a) *No separate RTC needed* — the Jetson clock
was never the problem; the bug is the re-acquisition discipline, and the fundamental
"monotonic vs. fast-correct-a-forward-drift" tension exists regardless of source. The
existing Jetson+bridge chain is fine once stepped on re-acquisition. (b) The synced
field already exists (`FLAG_TIME_SYNCED`); fix #3 makes it *honest* — it won't always
read synced (correctly false during boot / re-acquisition / Jetson-loss), which is the
value. BB/platform Teensys (not connected) will absorb the bridge's step via their
existing slew (~0.5 s transient at startup, harmless); give them the cone-style
step-on-jump later.

## Symptoms

- Operator impression during the run: arrival error "fairly centred ~120 ms,
  but some positions a lot closer." Grouping the analyzer output by position
  (`analyze_temporal_session.py --offset-from` the 2026-06-10 repeatability
  block) showed per-position raw means climbing almost monotonically from
  **pos01 +4.7 ms → pos20 +119 ms**, every group flagged FAIL against the
  06-10 δ = +128 ms.
- Yield: status counts `{ok: 68, miss: 32, throw-failed: 6}` (36 % excluded).
  Operator notes several "misses" were genuine catches where the piezo did not
  fire, and true misses were near-rim. Exclusions are recorded, not dropped,
  and do not affect the trend below (built on all 68 `ok` throws spanning the
  whole session).

## Diagnosis

### It's time, not geometry

Correlating raw `arrival_error_ms` (n = 68) against candidate drivers:

| vs | Pearson r |
|----|-----------|
| time since first throw | **+0.90** |
| position index (= chrono order) | +0.90 |
| predicted ToF | +0.24 |
| throw speed | +0.51 |
| horizontal range BB→cone | +0.60 |
| target height (global z) | −0.27 |

ToF barely moved all session (0.83–0.90 s — too little leverage to test
ToF-proportionality anyway). The range/speed correlations are confounds: the
operator happened to move the cone outward as the session aged. The clincher is
a **matched-geometry pair**: **pos03 and pos20** have near-identical geometry
(range ~920–950 mm, ToF 0.84–0.87 s) but read **~35 ms vs ~119 ms** — an ~85 ms
gap explained purely by *when*, not *where*. After the warm-up
(t > 900 s, pos14–20, n = 24) arrival error is **+114.8 ± 4.7 ms** and flat
against every geometric axis (|r| ≤ 0.17). The full 0 → 115 ms span was the
cold-start ramp swept through in time order.

Shape: steep rise pos01 (+5 ms) → pos09 (+96 ms) over the first ~600 s, then a
plateau ~+110–120 ms through pos20 (~1450 s). Asymptotic, ~10 min to settle.

### Cross-check — the 2026-06-10 fixed-position blocks

Both 06-10 timing blocks were single fixed positions (zero geometry confound):
the repeatability block (n = 30) started **already at +123 ms** and was roughly
flat (**+128.2 ± 8.6 ms**); the delay-sweep (n = 27) **+124.8 ± 6.8 ms**, flat.
They began warm because ~20 min of throwing preceded the 22:25 block. Today
began **cold** (throw #1 ≈ 0 ms). So the +115–128 ms plateau is the genuine
steady-state δ on both days; the 06-10 campaign simply never captured the cold
ramp.

### Cone clock exonerated (sync traces)

Pulled `/cone/heartbeat` + `/qtm_clock_offset_sec` from the session rosbag
(`~/Desktop/rosbags/2026-06-12_11-03-43`, uncompressed mcap) with a standalone
reader (`read_bag_sync_traces.py` — neither `rosbag2_py` nor the `mcap` lib is
installed in Foxy here):

- Cone **state = READY, `time_synced = True` from heartbeat #1 (t = 0)** and for
  all 17 452 heartbeats; no UNSYNCED/ERROR ever.
- `sync_rms_us`: **median 2 µs, p95 4 µs**; per-120 s-bin jitter flat ~10 µs
  early and *improving* to ~4–7 µs late (opposite of a settling lock; `max 255`
  is uint8 saturation on rare spikes).
- `/qtm_clock_offset_sec` drifted only **−14 ms** over the session — and the
  cone catch path doesn't use the QTM clock anyway.

A converged, time-synced IIR lock at 2 µs jitter **cannot** carry a 115 ms
mean-offset error; if the cone were the cause its jitter/state would show
convergence in the first 10 min, and it doesn't. Catch timestamps were valid
from second 0.

## Tier 0 — mocap ground truth (same bag, ball-tracker re-analysis)

The bag's `/balls` carries mocap-**CONFIRMED** Ball-Butler ball tracks (clean
measured parabolas, g≈9810; 67 k confirmed samples). Decoding `/balls` +
`/throw_announcements` straight from the mcap (`analyze_release_from_bag.py`) and
matching to the cone session gives an independent check — two robust results:

- **Velocity does NOT drift.** Actual/commanded **horizontal** launch speed
  (the most directly-measured quantity, from the ascending arc) is flat at
  **~0.98 across the whole session** (r=+0.19). The thrower launches at a
  consistent speed and vector cold→warm. (An apparent |v| drop was an
  apex-height tracking artifact in the *vertical* term only; horizontal is flat.)
- **Not a clock artifact (independent confirmation).** Decomposing
  `cone_ae = (cone_catch − mocap_cross) + (mocap_cross − predicted)` =
  A(clock) + B(physical): **A shows no session trend (r=−0.008)**; the ramp lives
  in the physical term. (Per-throw A/B are noisy because the ball is *caught* at
  the plane, perturbing the descent, but the sum reconstructs the cone ramp and
  the A-trend is robustly flat.)

**⇒ The warm-up is a release-TIMING drift, not kinematic.** Velocity is provably
stable, so a ball that arrives progressively later with an unchanged trajectory
must be **released progressively later** (same velocity vector, shifted in time)
— which also explains the spatially-neutral signature (uniform miss rate across
the session; late *catches*, not misses).

## PM mechanism campaign (2026-06-12) — temp & power-on **ruled out**

To get a temperature covariate, BB pitch/hand ODrive telemetry was plumbed
CAN1→UDP→ROS (`/bb/odrive_diag`; can-bridge firmware + `teensy_bridge_node`,
bridge Teensy reflashed — see external_changes). Verified live: bb_hand FET/motor
temps decode (~24–29 °C); bb_pitch has **no motor thermistor** (reads 0, expected).
Then a fixed-position protocol (`warmup_investigation_protocol.md`): Block A
(cold curve), 16-min throw-pause cooldown, Block C (re-ramp), Block D (fresh BB
power-cycle **and** full ROS restart).

Cross-session arrival error (ms):

| session | n | first-5 | mean | σ | note |
|---|---|---|---|---|---|
| 2026-06-10 repeatability | 30 | 118.4 | 128.2 | 8.6 | warm start (prior throwing) |
| 2026-06-12 distance-sweep | 68 | **12.6** | 85.0 | 33.4 | **first-of-day → ramped 0→118** |
| 2026-06-12 warmupA | 75 | 138.9 | 129.5 | 5.2 | BB powered ~25 min prior |
| 2026-06-12 warmupC (post-pause) | 19 | 128.1 | 133.3 | 5.1 | 16-min pause didn't reset |
| 2026-06-12 warmupD | 8 | 127.7 | 125.3 | 5.2 | **fresh BB + full ROS restart** |

**Only the first-of-day session started near zero and ramped. Everything since —
including a fresh BB power-cycle with all ROS scripts restarted — starts at the
~125–138 ms plateau.** This rules out, as the trigger of the ramp:
- **Hand-motor / FET temperature** — temps moved only ~3 °C and arrival error did
  not track them (r=−0.18 / −0.13, even slightly negative). No temp-FF on these.
- **Throw-induced / duty-cycle heating** — Block A was flat under continuous
  throwing; the 16-min pause did not reset it (warmupC = same plateau).
- **BB power-on** — warmupD's *first* throws are already +127 ms, not 0.
- **Any software / ROS state** — a full restart of all ROS scripts (warmupD) did
  not reset it.

So the ramp is a **once-per-long-idle** event that develops over the first ~10 min
of throwing and then **persists all day**, surviving BB power-cycles, full ROS
restarts, and throw-pauses. The only things *not* reset across the afternoon
tests were the **bridge Teensy firmware state** and the **physical hardware**.

## Cause

**Real, thrower-side, release-timing drift** (velocity unchanged — Tier 0), of
magnitude ~+115–130 ms, that **latches once per long idle** and holds all day.
Mechanism not pinned; two hypotheses survive (operator leans #2):

1. **First-use mechanical seating / long-time-constant thermal mass** *(leading)* —
   a belt/bearing/grease/escapement seats over the first ~10 min of throwing, or a
   thermal mass the motor thermistor doesn't see (gearbox / structure / ODrive
   heatsink) warms; either persists for the rest of the day and is not reset by
   power cycling logic. Predicts only an **overnight / multi-hour idle** reproduces
   the ramp.
2. **Bridge-Teensy firmware-state settling** *(not excluded)* — the only software
   not restarted in any afternoon test was the bridge Teensy (up since the 13:22
   reflash). Could be its time-anchor / 0x7DD broadcast converging after boot.
   Predicts **rebooting the bridge Teensy** (alone) reproduces the ramp.

The temperature hypothesis (this entry's original lead) is **falsified** for the
hand motor/FET sensors.

## Resolution

1. **Cone exonerated; drift accepted as real.** No cone firmware/timing change
   warranted from this.
2. **This session is not a valid phase-X distance result** (geometry confounded
   by warm-up; ToF leverage only 0.83–0.90 s). Retained as the **warm-up
   characterization dataset**, not a constant-vs-ToF verdict.
3. **Protocol change (apply before the next temporal campaign):**
   - **Pre-warm** ≥ 10–15 min / ≥ ~80–100 throws before the δ-calibration
     (repeatability) block, so δ is measured in the plateau regime.
   - **Bracket** every sweep — re-shoot pos01 at the end — to quantify residual
     drift and separate it from geometry.
   - **Widen** distance/height range for phase X to get real ToF leverage.
   - Fold these into `README.md` (phase-X procedure) and optionally a runner
     warm-up gate.
4. **±20 ms target is met by a fixed offset.** Once warm the system is stable at
   **129.5 ± 5.2 ms** (warmupA) across power cycles and a 16-min pause, so a fixed
   **+130 ms feed-forward δ** on `landing_time`/`throw_time` lands inside ±~15 ms —
   no temperature model needed. Operationally: throw a ~10-min warm-up block at the
   start of each day (or keep the rig running), then treat δ as a constant.
5. **Mechanism + any thrower/firmware fix: deferred/open**, to run with the
   already-deferred release-lag fix
   ([2026-06-11](2026-06-11-release-lag-firmware-analysis.md)) + spatial re-fit.
   Two cheap discriminating tests remain (operator out of bench time 2026-06-12):
   **(a) reboot the bridge Teensy alone, throw immediately** → ramp ⇒ hypothesis 2;
   **(b) true overnight/multi-hour-idle cold start, throw immediately** → ramp ⇒
   hypothesis 1 (and rules out 2 if (a) was flat).

## Impact

- The δ = +126.5 ms used in the 06-10 protocol and the 06-11 release-lag
  decomposition is a **warm** δ (06-10 blocks were already warm) — that analysis
  stands **for the warm regime**, but its *term attribution* deserves a second
  look with warm-up state controlled (see Open Questions).
- Any δ (or temporal correction) fitted from a **cold-start** session would be
  badly biased low. Warm-up control is now a precondition for temporal
  calibration.
- If the warm-up also shifts the **spatial** landing point (it should, if
  release dynamics change), the deployed aim-correction affine is itself
  warm-up-dependent — worth a cold-vs-warm spatial spot-check.

## Data artifacts

(all under `zTesting/throw_testing/temporal_accuracy/`)
- Sessions: `sessions/temporal_distance-sweep_2026-06-12_11-07-44.json` (the cold
  ramp); `sessions/temporal_repeatability_warmup{A-cold,C-reramp,D}_*.json` (PM)
- Plots: `..._11-07-44_DIAGNOSTIC.png` (error vs time/distance),
  `..._SYNC_TRACES.png` (cone sync), `..._TIER0_decompose.png` (clock-vs-physical
  + velocity), `..._warmupA-cold_..._WARMUP_TEMP.png` (AE vs hand temp — no collapse)
- Tools: `read_bag_sync_traces.py`, `analyze_release_from_bag.py`
- Protocol: `warmup_investigation_protocol.md`
- Bags (not in repo): `~/Desktop/rosbags/2026-06-12_11-03-43` (distance sweep),
  `..._13-47-58` (warmupA/C + `/bb/odrive_diag`), `..._14-38-35` (warmupD)

## BB ODrive telemetry (built 2026-06-12; instrumentation note)

BB thrower temps were originally **not** telemetered (the can-bridge decoded
ODrive 0x15 only on CAN3 / the Jugglebot platform). On 2026-06-12 this was wired:
`on_bb_rx` now also decodes CAN1 ODrive frames (nodes 7=bb_pitch, 8=bb_hand) into
a separate `bb_axes` cache (`can_buses.cpp::decode_bb_odrive`), emitted as
DIAGNOSTIC frames with `axis_id` 7/8 (`telemetry.cpp::send_bb_diag`, ~1 Hz each) —
**reusing the existing frame, platform path untouched**. `teensy_bridge_node`
republishes them on **`/bb/odrive_diag`** (`Float32MultiArray`: `[pitch_fet,
pitch_motor, pitch_iq, pitch_state, hand_fet, hand_motor, hand_iq, hand_state]`),
recorded by `jugglebot_launch`. Verified live: hand FET/motor temps decode
(~24–29 °C). **bb_pitch has no motor thermistor** (temp reads 0 — expected, per
operator). Bridge Teensy reflashed; all changes uncommitted. Follow-up: bb_pitch
broadcasts heartbeat but not `Get_Temperature` — enable its cyclic temp broadcast
(or add a periodic request) if pitch temp is ever wanted.

## Open Questions / Follow-ups

1. **Mechanism — the two discriminating tests (see Resolution §5).** Velocity,
   cone clock, motor/FET temp, throw-duty heating, BB power-on, and ROS/software
   state are all ruled out (above). Remaining: **(a) bridge-Teensy reboot alone**
   (→ hypothesis 2: bridge firmware-state) vs **(b) overnight-idle cold start**
   (→ hypothesis 1: mechanical seating / long-thermal-mass). Operator leans (1).
2. **Reconcile cold ≈ 0 with the 0.273/v release-lag term**
   ([2026-06-11](2026-06-11-release-lag-firmware-analysis.md)): that scheduling
   defect is kinematic/temperature-independent, yet cold the *total* release
   timing is ≈ 0 — so either a compensating cold early-release, or the achieved
   release *instant* (servo phase / friction / seating), not the firmware
   schedule, dominates the warm δ. Re-derive with warm-up controlled.
3. **bb_pitch temperature**: pitch ODrive broadcasts no `Get_Temperature` (and has
   no motor thermistor) — enable cyclic temp broadcast / FET-only if pitch thermal
   data is wanted; hand temp already flows on `/bb/odrive_diag`.
4. **Day-to-day plateau** (warm δ ~125–133 ms today vs ~126 ms on 06-10): stable
   to a few ms; sets the fixed-δ correction budget (well inside ±20 ms).
5. **Spatial coupling:** Tier 0 says velocity is flat, so the landing *point*
   should be warm-up-stable — confirm the deployed aim-correction is **not**
   warm-up-dependent (consistent with the uniform miss rate).
6. **Piezo no-fire on genuine catches** (part of the 32 misses): sensor
   threshold/sensitivity — separate from timing but caps yield (~36 % excluded).
