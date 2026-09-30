---
title: Layer C yaw settle confirm gains a sample-history term (FW 5): dither on a settled axis stops tripping the gate
type: investigation
date: 2026-09-30
status: in-progress
phase: "R5 sitting 1 / BB FW 5"
related_entries:
  - 2026-06-21-throw-settle-gates-loud-on-aim
  - 2026-09-28-bb-firmware-update-over-can
files_changed:
  - ball_butler_main/BallButlerConfig.h (YAW_RATE_TOL_DPS 3.0 -> 12.0, new YAW_SETTLED_MIN_SAMPLES = 15, comment rewrite, stale logbook-date label fixed 06-19 -> 06-21)
  - ball_butler_main/YawAxis.h (Telemetry::settled_samples, settled_samples_ member)
  - ball_butler_main/YawAxis.cpp (ISR settle-history counter, begin()/readTelemetry() wiring, hard-limit fault path resets the counter)
  - ball_butler_main/HandTrajectoryStreamer.h (yaw_settled_min_ member + arm() parameter, yaw_ok gains the history term, serial abort line reports settled=N/required)
  - ball_butler_main/StateMachine.cpp (the one arm() call passes YAW_SETTLED_MIN_SAMPLES explicitly)
  - ball_butler_main/FwUpdate.h (FW_VERSION 4 -> 5, receipt comment)
external_changes:
  - "Jugglebot (skill-stack worktree): teensy_link/rpc_args.py (BB_FW_VERSION_EXPECTED 4 -> 5)"
subsystem:
  - firmware
  - throwing
tags:
  - safety
  - throw
  - testing
---

# Layer C yaw settle confirm gains a sample-history term (FW 5)

## Summary

5 of 19 `bb/throw_at_target` dispatches at the R5 sitting (2026-09-30) refused
`THROW_ABORTED_NOT_SETTLED`. Four sat on a flat, sub-0.4 deg plateau and were refused
by encoder dither on an already-settled axis crossing the old instantaneous
`YAW_RATE_TOL_DPS = 3.0` bound, not by real motion. FW 5 adds a 15-sample (100 ms at
150 Hz) in-band error history alongside a widened 12 deg/s rate bound, so a settled
axis's dither can no longer trip the gate while a genuine traverse still does. Built
(2026-09-30, 18:31, SUCCESS in 5.04 s) but not yet flashed. The investigation's own
starting premise, that the fifth refusal (throw #1) was a distinct late-convergence
case the fix must still catch, does not survive a 150 Hz replay of its own trace: see
Discussion.

## Symptoms

R5 sitting 1 (2026-09-30, bag `~/Desktop/rosbags/2026-09-30_16-20-05`) made 19
`bb/throw_at_target` dispatches to Jugglebot: 14 `OK`, 5 `THROW_ABORTED_NOT_SETTLED`.
Four of the five (throws #3, #5, #10, #12) sat on a flat, sub-0.4 deg peak-to-peak
position plateau for the full 1.5 s before the fire-time check, with error at the
check instant in the same +0.26 to +0.48 deg band as more than half of the accepted
throws, and 10 Hz-derived rate magnitudes (1.10 to 3.18 deg/s) overlapping accepted
throws #14 (3.89 deg/s) and #15 (2.33 deg/s). Nothing in the position trace told the
four plateau refusals apart from the accepted throws around them.

The fifth, throw #1, looked different at first: a real, large, monotonic yaw
convergence from -4.27 deg error down to inside tolerance in the final 0.3 to 0.4 s
before the check, read initially as a Layer A lead under-estimate rather than gate
noise, and treated as a case any fix must still be able to refuse.

## Diagnosis

`YawAxis`'s yaw error and rate come from a 150 Hz finite difference of the AS5047P
absolute encoder (16384 counts/rev, so 1 count = 360/16384 = 0.02197 deg), EMA-filtered
at alpha = 0.3. One encoder count of jitter between two consecutive 150 Hz samples
already implies a raw velocity of 0.02197 x 150 = 3.296 deg/s, above the old
`YAW_RATE_TOL_DPS = 3.0`, before any filtering. A single isolated count only pushes the
filtered value to 0.3 x 3.296 = 0.99 deg/s, so a one-count flicker never tripped the
old gate. What trips it is a kick of 4 or more counts in one sample, or 2 counts in
each of two adjacent samples (confirmed by the offline gate model,
`scratchpad/probe_bb_fw5_gate.py`, probe A).

The yaw loop is pure P (Kp = 300, Ki = 0, Kd = 0, `hardware_config.h:389-391`), which
has no mechanism to null a steady-state friction offset, so it parks at whatever small
residual error balances Kp times error against static friction. Below the PWM
deadband, the sigma-delta dither (`YawAxis.cpp:704-725`, `deadzone_compensate()`)
accumulates sub-threshold command magnitude every tick and fires a single
full-magnitude `PWM_MIN` pulse once the accumulator crosses threshold: a real, brief
motor kick, sized right at the old tolerance boundary by construction, not noise in
the instrumentation sense. That is the plateau refusals' mechanism: a settled axis's
own dither crossing an instantaneous rate check that was built to catch a traverse.

This claim is bounded by what the bag can show: the only yaw signal relayed to the
Jetson is `bb_yaw_deg` position at 10 Hz (`/bb/heartbeat`); the firmware's own 150 Hz
`vel_rps` never reaches the host. A 10 Hz first difference cannot resolve a
single-ISR-tick kick, so the dither mechanism is inferred from the code and the
offline probe, not measured live at 150 Hz. The mechanism is still unmeasured on
hardware; see Outcome for what would close that gap.

## Discussion

**Withdrawn premise: throw #1 is not a distinct case.** The investigation started
from the sitting's own framing, that throw #1 was a genuine late-converging throw a
fix must still be able to refuse, separately from the four dither false positives.
Replaying #1's own 10 Hz trace at 150 Hz (probe C) withdraws that. The yaw entered the
+/-1 deg band at t = -0.79 s, about 0.7 s before the recorded check, and from
t = -0.42 s sat flat at error -0.45 to -0.51 deg. At the recorded check instant:
error +0.45 deg, rate 0.58 deg/s, settled_samples = 116. FW 5 passes this throw, and
so would FW 4 on the same smooth trace (FW 4's last refused instant on this replay is
t = -0.798 s). Throw #1's real refusal at the check was the same unseen 150 Hz dither
as the four plateau cases, riding the tail of a convergence that had already finished
roughly 0.7 s earlier, not a late arrival caught in the act.

To refuse #1 as it was actually recorded, N would need to be about 118 samples
(790 ms). That exceeds the 0.6 s wind-up plus the 0.1 s `SCHEDULE_MARGIN_S` that
Layer A reserves (`hardware_config.h:492`, `StateMachine.cpp:768-771`), so it would
refuse throws systematically rather than catch a rare late arrival. Not recommended,
and not done. What FW 5 does still catch of throw #1's shape is any check instant
earlier than about t = -0.705 s, i.e. while it was genuinely converging or within
100 ms of band entry.

**The trade the design carries.** The brief's design (N = 15 at 150 Hz = 100 ms,
`YAW_RATE_TOL_DPS` 3.0 to 12.0) was implemented as specified, with no constant
changed from what was asked. It carries a trade-off worth stating plainly rather than
leaving inside a constant: by the convention of holding the rate for the whole 0.6 s
wind-up, the rate term's worst-case aim-budget ceiling at 1.1 m is 4x FW 4's (see the
table in Fix). Read alone, that overstates the real exposure, because holding 12 deg/s
for the full wind-up after 100 ms inside tolerance means travelling 7.2 deg past the
target against a live P loop, which is a controller failure, not a throw the gate is
meant to admit. The history term bounds what the rate term can actually pass in
practice: for a monotonic approach that stops at its target, FW 5 passes a moving yaw
only while it is still converging, and only at 8 deg/s or slower with error at or
below 0.23 deg (4.4 mm), tightening to 2 deg/s at 0.80 deg or below. FW 4 passed the
2 to 3 deg/s crawls too, at a looser 0.89 to 0.98 deg. Both statements are true at
once: the nominal ceiling is 4x wider, and the history term makes that ceiling
unreachable for a converging yaw. Recording both here rather than only the reassuring
half is the point of this section.

## Fix

The rule is now two terms, both must hold: (a) error inside 1.0 deg
(`YAW_ERR_TOL_DEG`, unchanged) for the last 15 consecutive 150 Hz control samples
(`YAW_SETTLED_MIN_SAMPLES`, new, 100 ms at 150 Hz), AND (b) filtered rate at or below
12 deg/s (`YAW_RATE_TOL_DPS`, was 3.0), AND the old instantaneous error check (also
inside 1.0 deg, now redundant with (a) but kept). No numeric error tolerance changed;
the rate bound widened and a sample-count history term was added alongside it.

Aim budget at 1.1 m, `1100 * tan`, counting the rate as held for the whole
`WINDUP_DURATION_S` = 0.6 s wind-up (a worst case, not an operating figure, see
Discussion):

| Term | FW <= 4 | FW 5 |
|---|---|---|
| error term (1.0 deg) | 19.2 mm | 19.2 mm (unchanged) |
| rate term alone | 3.0 deg/s -> 1.8 deg -> 34.6 mm | 12 deg/s -> 7.2 deg -> 139.0 mm |
| both terms | 2.8 deg -> 53.8 mm | 8.2 deg -> 158.5 mm |

Implementation: the settle history is counted in `YawAxis::controlISR()`
(`YawAxis.cpp:822-831`), right after `last_err_deg_` is set, before the
enable/deadband branch, so a parked-in-tolerance axis keeps counting; it saturates at
`UINT16_MAX` instead of wrapping, and a NaN error fails the `<=` check and resets it
to 0. `readTelemetry()` copies the new `Telemetry::settled_samples` field inside the
existing `noInterrupts()` block (`YawAxis.cpp:588`). `HandTrajectoryStreamer::arm()`
gains a `yaw_settled_min_samples` parameter defaulting to
`AxisSettleCfg::YAW_SETTLED_MIN_SAMPLES` (`HandTrajectoryStreamer.h:45,50`), so a
caller that leaves it out still gets the history term; a default of 0 would have
turned it off silently. `StateMachine.cpp:1013-1015`'s one `arm()` call passes it
explicitly. The serial abort line gains `settled=%u/%u` (`HandTrajectoryStreamer.h:108,112`).
The stale comment citing "logbook 2026-06-19" is fixed to 2026-06-21
(`BallButlerConfig.h:290`); no 2026-06-19 entry ever existed.

Fail-closed side effects, all intended. No yaw pointer leaves `Telemetry`
value-initialised, so `settled_samples = 0` and the throw refuses. The ISR not
running leaves the count at 0 and refuses. A hard-limit fault now resets the counter
and refuses (`YawAxis.cpp:804`); FW 4 read a placeholder `last_err_deg_ = 0` on that
same path and passed the error term, so a faulted yaw used to be able to pass Layer C.

Struct safety was checked before adding the field: `Telemetry` is only ever read by
field name, never serialised and never positionally initialised (grep of
`ball_butler_main/`: `HandTrajectoryStreamer.h:97` and `ball_butler_main.ino:176,465,480`
are the only readers), so the new field cannot desync a wire layout that does not
exist.

Lockstep with Jugglebot, following the FW 2/3/4 receipt convention: `FwUpdate.h:76,80`
sets `FW_VERSION = 5` with a one-line receipt comment; `teensy_link/rpc_args.py:486-494`
(Jugglebot `skill-stack` worktree) pins `BB_FW_VERSION_EXPECTED = 5` the same way.
There is no wire or protocol change: `THROW_ABORTED_NOT_SETTLED` detail0 (binding
axis) and detail1 (yaw error, centidegrees) keep their meanings, and nothing on the
runtime path compares the firmware version against the constant (only
`tests/firmware/test_bb_fw_update_xref.py:111`, tree vs tree, does). The flash receipt
is the fw-update tool's own `FW version: 4 -> 5` line, not the constant and not a
matching hex md5.

## Outcome

**Build** (2026-09-30, 18:31, `cd ~/Desktop/BallButler/ball_butler_main && pio run -e
teensy40_can`, no `-t upload`): `[SUCCESS] Took 5.04 seconds`, 0 warnings, 0 errors.
`.pio/build/teensy40_can/firmware.hex`: 460937 B, md5 `4798642934b60aaa43edee5b985d3aa1`.

**No BB native or unit tests exist** for `YawAxis` or the streamer: `platformio.ini`
has no native env and `ball_butler_main/test` does not exist. Cross-repo tests
(Jugglebot, `skill-stack` worktree, 2026-09-30): `pytest
tests/firmware/test_bb_fw_update_xref.py -v` gave 5 passed in 0.10 s (5 passed in
0.09 s again after the final comment edit); `pytest tests/firmware -q` at 18:33 gave
263 passed, 1 skipped in 12.42 s. The full `./run_tests.sh` gate was not run as part
of this change.

**Offline gate model** (`python3 scratchpad/probe_bb_fw5_gate.py`, output in
`scratchpad/probe_bb_fw5_gate.out`; mirrors the ISR's count quantisation, EMA and
counter, and the streamer's `yaw_ok`, for FW 4 vs FW 5): probe A confirms FW 4
refuses kicks of 4 or more counts and 2+2, FW 5 passes up to 12 counts and 7+7, and
both refuse 13 counts and 8+8. Probe B confirms FW 5 never passes while moving faster
than 8 deg/s, and on fast arrivals it first passes 93 ms after band entry against
FW 4's 67 to 80 ms, up to 26 ms stricter there. Probe C is the throw #1 replay in
Discussion.

**Flash status: NOT YET FLASHED.** Procedure (unchanged from FW 1 through 4): launch
down, BB in IDLE or ERROR, `cd ~/Desktop/BallButler/ball_butler_main` then `pio run -e
teensy40_can -t upload` on its own line. The receipt to look for is the fw-update
tool's `FW version: 4 -> 5` line; a matching hex md5 alone is not a flash receipt.

**What to watch at the next sitting.** Count `THROW_ABORTED_NOT_SETTLED` on
`/bb/throw_outcome` against this sitting's baseline of 5/19 (4 on plateaus, 1 on
throw #1's tail); expect close to 0 on plateaus. For any refusal, the BB serial line
now prints `settled=N/15 rate=R`, naming the binding term: N < 15 means the history
term bound it (a real late arrival), R > 12 means the rate term bound it (a
traverse). detail1 is still yaw error in centidegrees, so the Jugglebot decode needs
no change. The dither mechanism itself is still unmeasured at 150 Hz on real
hardware; if a residual refusal rate persists after the flash, capture
`Telemetry.vel_rps` / `vel_rps_raw` over USB serial on the bench around a throw.

## Open Questions / Follow-ups

1. **Flash and confirm the receipt.** Not yet flashed; flash before the next sitting
   and confirm the fw-update tool's `FW version: 4 -> 5` line.
2. **Layer A follow-up: the 100 ms dwell eats the schedule margin.** The new history
   term's 100 ms dwell inside tolerance is not reserved anywhere in Layer A's lead
   calculation; it eats into the 0.1 s `SCHEDULE_MARGIN_S` Layer A already reserves
   without accounting for it explicitly.
3. **Layer A follow-up: the traverse-rate assumption is roughly 4x optimistic near
   the target.** Throw #1's 17.4 deg move took about 1.2 s to reach the band. Layer A
   assumes `YAW_TRAVERSE_DEG_PER_S = 60`, which predicts 0.29 s. The P plus
   tapered-feedforward approach is roughly 4x slower than Layer A assumes near the
   target; this is a Layer A sizing question, not something this change addresses.
4. **Cross-reference in Jugglebot updated.** `tests/ros/test_skill_node.py:3322` and
   `skill_node.py`'s retry docstring cited `HandTrajectoryStreamer.h:104-110` for the
   abort block; it moved to `:117-122` in this change and both cites were updated on
   the Jugglebot side the same evening (skill-stack worktree).
5. **Mechanism still unmeasured live.** The dither mechanism is inferred from source
   and an offline model built from a 10 Hz bag, not measured at the firmware's own
   150 Hz on hardware. See Outcome for the bench measurement that would close this.
