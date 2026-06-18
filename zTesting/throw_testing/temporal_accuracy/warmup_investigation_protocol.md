# Warm-up drift — mechanism test protocol

Companion to logbook `2026-06-12-temporal-warmup-drift.md`. Goal: find the
**mechanism** of the cold-start arrival-time drift (0 → +115 ms over ~10 min,
then plateau), or failing that, characterize it well enough to counteract to
**±20 ms**.

## What we already know (don't re-litigate)

- The drift is **real and physical**, not a cone-clock artifact (cone sync solid
  from t=0; the cone↔QTM clock offset shows no session trend, r≈0).
- It tracks **session time, not catch geometry** (warm plateau is flat ±5 ms vs
  distance/height).
- It is a **release-timing** drift, **not kinematic**: mocap shows actual
  horizontal launch speed is flat at ~98 % of commanded all session (r=+0.19);
  the ball is launched progressively *later* with an unchanged velocity vector.

So this protocol asks: **what makes the release instant drift later as the
machine warms** — thermal vs mechanical, and self-heating (from throwing) vs
powered-idle heating.

## Temperature instrumentation — READ FIRST

BB thrower motor/FET temps are **not currently telemetered to the Jetson**. The
can-bridge decodes ODrive temps only on CAN3 (the Jugglebot *platform*); Ball
Butler is on CAN1, where the bridge decodes only the BB heartbeat (no temp
field). Getting BB temps electronically needs a BB-Teensy firmware change (add
temps to the heartbeat / a new diag frame) — deferred (and the BB Teensy isn't
connected right now).

Therefore this protocol uses **wall-clock time** as the warm-up axis (we know
the drift is a clean asymptotic function of time) and **causal toggles** to
separate thermal from mechanical — these don't need a temp sensor. If you have
an **IR thermometer / thermocouple**, add the optional manual-temp track below;
it converts "time" into "temperature" and directly yields a correction variable.

## Fixed setup (all blocks)

- **One cone position, marked, unchanged for the entire campaign** (zero geometry
  confound). Pick a comfortable mid-range spot (~1.0–1.2 m, the cone height used
  on 06-12). Re-verify QTM has `Catching_Cone` + the BB rigid body.
- Stack: `teensy_bridge_launch.py` + `jugglebot_launch.py apply_aim_correction:=true`
  (underscores!). Same ball type throughout.
- **Record a bag each block** (jugglebot_launch already does, default
  `record:=true`) — keep `/balls`, `/throw_announcements`, `/cone/catch_event`,
  `/cone/heartbeat`, `/bb/heartbeat`, `/teensy/robot_state` (platform temps, a
  weak ambient proxy). Note the bag dir per block.
- Throw with `run_temporal_session.py --phase repeatability --throws N --delay 2.5`
  (fixed position = repeatability). Session JSON timestamps every throw
  (`service_call_wall_ns`) so arrival error can be plotted vs time.
- **Log per block:** wall-clock start time, ambient temp, what state the machine
  was in before the block (cold / warm / how long idle), and the session-JSON +
  bag paths.

## Optional manual-temp track

If you have an IR thermometer: at each block start and **every 2 min during
Block A**, read and write down the temperature of (a) the BB **hand/throw motor**
body, (b) its **ODrive/driver** heatsink, (c) ambient. Same spots each time.
This lets us plot `arrival_error` vs measured temperature.

---

## Block A — cold-start warm-up curve  (PRIMARY)

**Precondition:** machine fully **cold** (off ≥ 60 min, or first run of the day).
**Do:** power on, and *immediately* (minimize idle before throwing) run
`--phase repeatability --throws 120 --delay 2.5` at the fixed position. Don't
stop; let it run ~20 min. Take IR readings every 2 min if available.
**Yields:** the canonical curve `arrival_error(t)` → time-constant **τ** and
asymptote. This is the reference every other block compares against, and the
basis for a feedforward correction.

## Block B — idle-powered vs throw-induced heating

**Precondition:** machine fully **cold** again (ideally a separate cold start;
if same day, after a long cooldown — see Block C2).
**Do:** power on the full stack and **wait 15 min without throwing** (motors
energized/holding). Take IR readings during the wait if available. Then run
`--phase repeatability --throws 25 --delay 2.5`.
**Reads out:**
- First few throws already near the **+115 ms plateau** → heating is from
  **powered/holding current** (electronics/idle), not the throw stroke.
- First few throws still near **0 ms** and they then ramp → heating is
  **throw-induced** (stroke duty cycle / mechanical self-heating).

## Block C — cooldown / restart  (thermal vs mechanical)

Run **immediately after Block A** (machine warm, at plateau).
- **C1 short pause:** stop throwing, leave powered, wait **3 min**, then 10 throws.
- **C2 cooldown:** power **off** BB, wait **25–30 min** (let it cool), power on,
  immediately run `--phase repeatability --throws 30`.

**Reads out:**
- C1 stays at plateau, C2 **drops back toward 0 and re-ramps** → **thermal**
  (recovers when cooled); C2's re-ramp gives the **cooling** behaviour and
  confirms it tracks temperature, not throw-count or one-time wear-in.
- Both stay at plateau (no drop even after C2 cooldown) → **mechanical wear-in /
  backlash settling**, not temperature — the drift is a one-time per-power-cycle
  or per-session settling, not thermal.

## Block D — throw-rate dependence  (OPTIONAL)

Repeat Block A from cold but at **~half the throw cadence** (longer gaps between
throws — pace manually or with longer reload waits, ~60 throws over ~20 min).
**Reads out:** if the curve vs **wall-clock time** matches Block A → ambient/soak
heating dominates; if the curve vs **throw count** matches Block A (i.e. slower
in wall-time) → per-stroke self-heating dominates.

---

## Analysis (I'll run these on the bags/sessions)

1. **Block A:** fit `arrival_error(t) = A·(1 − e^(−t/τ))`; report τ, A, warm σ.
   Re-derive release timing & launch velocity from `/balls`
   (`analyze_release_from_bag.py`) to re-confirm velocity-flat / timing-drift on
   this fixed-position data (cleaner than the 06-12 distance sweep).
2. **Block B:** overlay its first-N throws on Block A at matched throw-count →
   idle-power vs throw-induced.
3. **Block C:** does C2 re-ramp (thermal) or hold (mechanical)? Fit the C2
   re-ramp τ; compare to Block A.
4. **(If manual temp):** plot `arrival_error` vs measured motor/driver temp; if
   it collapses to a single curve, temperature is THE variable → feedforward on
   temp.

## Decision tree (what we conclude)

| A ramps? | B starts warm? | C2 re-ramps? | ⇒ Mechanism |
|---|---|---|---|
| yes | no (starts cold) | yes | **Throw-induced thermal** (stroke self-heating) — correct via temp-FF or pre-warm |
| yes | yes (starts warm) | yes | **Powered/idle thermal** (holding current / drive heating) — pre-warm by powering on early |
| yes | either | no (holds warm) | **Mechanical wear-in / backlash** — settles per power cycle; correct via a fixed post-warm-up offset after a settling burst |

## Counteracting to ±20 ms (regardless of mechanism)

- **Pre-warm** to plateau before any real throws → warm σ is already ~5 ms.
  Block A's τ tells you how long (likely ~10 min).
- **Feedforward** the fitted `A·(1 − e^(−t/τ))` (time) — or `offset(T)` if the
  manual-temp track collapses cleanly — onto the commanded `throw_time`.
- **Closed-loop cone-trim:** slow integrator nulling recent cone arrival error
  by advancing `throw_time` — mechanism-agnostic, also absorbs day-to-day δ.
- **Sequencing:** if the deferred 06-11 firmware release-lag fix lands, it shifts
  the warm δ ~82 ms — re-run Block A *after* that fix before fitting a correction.

## Safety / notes

- Same fixed position the whole time — moving it reintroduces the geometry
  confound and ToF leverage problems.
- A >20 % miss block is invalid (fix aim/placement first); misses are excluded
  but logged.
- Don't flash the BB Teensy for this — it's not connected, and no firmware change
  is needed to run these blocks.
