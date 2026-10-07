# Local two-ball accuracy calibration

`run_local_calibration.py` plans randomized targets, solves each with the
**corrected hand side** (`s = +105.65 mm`), commands the existing `/bb/throw`
action, waits for the firmware's actual outcome, and records raw ROS2 data
to MCAP. It bypasses the old affine and the columns feed-bias correction for
these throws. It does **not** change production constants, the normal BB node,
or any saved correction. Do not juggle concurrently; the runner requires the
orchestrator to remain IDLE. Arrange free flight through the catch plane into
the usual collection area; the script does not move Jugglebot to catch balls.

QTM must stream to the normal mocap node, but **a separate QTM recording and
manual trajectory cleanup are not normally necessary**. Raw markers, BB pose,
commands, timing windows and firmware outcomes are retained for offline
trajectory extraction. A QTM recording is an optional backup if streaming loses
the ball. After capture, `analyze_local_calibration.py` extracts measured flights
and fits a replacement affine from the session JSON and raw MCAP. It does not
deploy that candidate. The new session JSON is not the legacy `fit_affine.py`
input schema. Do not substitute predicted/tracker landing positions for
measured raw trajectories.

## Grid and frame

The default core comes from Jugglebot's
`logbook/2026-10-04-skill-stack-r5-sitting-4.md`: sites near X = ±62.5 mm,
Y = 0, catch Z = 830 mm; the displaced feed request was X = -52.5 mm.
Full original sitting logs are not present in this checkout. `--logs` can
estimate the bounding box from columns `CATCH-AIM` lines in supplied launch
logs, ignoring other skill runs. Inspect the preview: tracked catch outliers
can enlarge that bounding box. Use `--core xmin xmax ymin ymax` to override
it explicitly (omit `--logs` when doing so).

The dense core rounds outward to 50 mm steps, with at least one step on each
side of its centre. Outside it, each successive axis gap grows from 50 to
200 mm, reaching 200 mm at 200 mm beyond the dense core. The grid is the
Cartesian product of those nonuniform X and Y axes. Thus gaps grow along
each axis, not on circular rings. `--padding` bounds the outward extent;
the final point falls inside the bound rather than adding a tiny edge gap.

Defaults: **143 distinct targets, 331 throws before reachability filtering**.
The central 5x3 points get five repeats; other points get two. Repeats run
in randomized blocks, with each eligible target appearing once per block.
Seed 42 reproduces the exact order; use a different seed for independent
validation. At several seconds per throw plus reloading, allow tens of
minutes, or reduce padding/repeats and use a pilot batch first.

XY is in the **schedule frame** used by the columns logs. Z is already global.
At execution, `--schedule-to-mocap DX DY` adds the measured XY frame offset,
matching `skill_node._mocap_aim_point_mm`. Read it from the current sitting's
frame check (`mocap Platform ... (x ..., y ...) from the commanded position`).
It is **not** the observed ball landing bias. There is deliberately no silent
zero default. If the frames coincide, supply `0 0` explicitly. An old session's
offset must not be reused after QTM realignment without checking it.

## On the Jetson

**Keep ROS2 running.** Start the normal Jugglebot stack with QTM streaming and
BB connected/calibrated, leave the orchestrator IDLE, then run this Python
script in a second sourced terminal. The script is a ROS2 client, not a
replacement for the bridge/mocap/orchestrator nodes. For example, after the
usual environment setup, the first terminal runs:

```bash
ros2 launch jugglebot jugglebot_launch.py record:=false
```

`record:=false` disables only the launch's duplicate recorder, **not ROS2**;
the calibration script starts its own MCAP recorder. Do not start a juggling
pattern alongside it. No separate QTM recording is required for the normal
path.

Activate the project virtualenv and source the built Jugglebot ROS workspace,
then work in `BallButler/zTesting/throw_testing/accuracy_testing`.
Start the normal stack, calibrate BB's pose, and load the hopper. This runner
starts its own focused recorder; normal launch recording may stay on, but
`record:=false` avoids duplicate bag I/O. Whether the normal launch enables
the old affine does not affect this script's direct corrected-geometry path.

1. Make and inspect the plan (no ROS or hardware required):

   ```bash
   python run_local_calibration.py plan --out local_calibration_plan.json
   # Alternatively estimate the core from selected recent launch logs:
   python run_local_calibration.py plan --logs /path/to/launch.log --out local_calibration_plan.json
   ```

   Open `local_calibration_plan.html`; hover over a point for its coordinates
   and repeat count. To narrow the campaign, use e.g. `--padding 300`.
   Spacing, taper distance, core, catch height, repeats and seed are CLI options.

2. Perform a live check with **no motion**. Substitute the actual measured
   frame offset for DX and DY:

   ```bash
   python run_local_calibration.py run local_calibration_plan.json \
       --schedule-to-mocap DX DY --check-only
   ```

   This saves the current BB pose, reachable schedule, skipped targets/reasons,
   installed solver hash, and hardware constants under a timestamped folder
   in `~/bb_calibration_sessions`. Check the reachable region before proceeding.

3. Run a small pilot:

   ```bash
   python run_local_calibration.py run local_calibration_plan.json \
       --schedule-to-mocap DX DY --limit 10
   ```

   This selects the first ten **reachable entries in the randomized schedule**,
   so the pilot is not restricted to the central points. For a core-only pilot,
   generate a separate plan with `--padding 0` first. After inspecting that
   batch, run the full plan by omitting `--limit`. Each invocation writes a new
   session; it never overwrites or silently resumes a previous session.

4. For validation, generate a new plan with a different seed (e.g. 1042).
   The present runner always tests corrected geometry **without an affine**;
   validation of a newly fitted affine is a subsequent step, not an option
   silently applied here.

## Capture and failure handling

### Returning thrown balls to the magazine

**Wait for the printed `REFILL` prompt and verify the last BB ball is down.**
Then throw as many balls back into the magazine as needed. When all of those
balls have landed and the flight area is clear, press Enter. The script keeps
ROS and recording alive during the pause, waits one further settling second,
checks BB readiness, and only then sends the next throw. There is no timeout
on the refill pause. `q` + Enter stops the run with partial data retained.

By default this prompt appears before the first throw and **after every nine
throws**, before the next batch (`--refill-every 9`). It waits indefinitely
until you press Enter. **Do not toss refill balls during the automatic batch**,
even after one ball has grounded: wait for `REFILL`. The final batch may contain
fewer than nine throws; the session finishes after its last capture without
another refill prompt. Use `--refill-every` to override the batch size.

Each throw records a distinct `analysis_window_wall_s`, predicted release
position/velocity, and `capture_complete` flag. Refill start/end times are
saved both in `session.json` and as bag events. Downstream extraction uses
`analysis_throw_at(session, timestamp)` to exclude refill intervals and pair
raw samples with a successful, completed BB capture window. Failed/uncertain
throws and ambiguous overlapping windows are not assigned. Thus your refill
arcs can be perfect projectiles—even on the same path—without being paired as
calibration throws, **provided they happen during the prompted refill period**.
Keeping their paths separate remains useful backup; it is not the primary gate.

The minimum wait before `REFILL` covers predicted ground arrival plus 0.5 s,
then `--pause` (default 1 s). `--ground-z` defaults to world Z = 0 mm; set it
to the actual ground height if your QTM world origin differs. This is a timing
estimate, so still check that the ball has actually grounded before refilling.
`--refill-settle` controls the additional delay after Enter; do not press
Enter while a return ball is still airborne.

Raw refill markers stay in the bag for audit, excluded by the recorded windows.
The companion analysis identifies launch-associated outbound arcs, fits a
free-flight model to raw observations and measures the descending catch-plane
crossing (see "Trajectory extraction"). Bounces
below the catch plane cannot enter the fitted arc. Ambiguous trajectories,
missing crossings and long gaps are rejected and reported. Collect and handle
balls during refill pauses. If a refill accidentally overlaps a BB capture,
record the affected throw and use `--exclude` with its session `throw_idx`.

### Session files

Each session contains:

- `session.json`: full plan, explicit schedule-to-mocap translation, BB pose,
  corrected signed s, no-affine/no-feed-bias flags, solver/hash/constants,
  reachable and skipped targets, actual goal parameters/UUID, wall timestamps,
  firmware terminal outcomes, per-throw analysis windows, refill intervals,
  predicted release states, and capture status.
- `events.jsonl`: append-only dispatch/result event copies, also published on
  `/bb/local_calibration/event` and included in the bag.
- `bag/`: MCAP of raw `/mocap_data`, `/rigid_body_poses`, latched
  `/bb/calibration_result`, BB heartbeat/axis estimates, orchestrator state,
  ROS logs, parameter events, and calibration events.
- `recorder.log`, `record_qos.yaml`: recorder diagnostics and explicit QoS.

The runner waits for a fresh connected BB heartbeat, ball loaded, IDLE/TRACKING
BB state, fresh mocap, and IDLE orchestrator before each command. It checks
reachability against the installed solver's normal speed/pitch/yaw/height
limits, retains all skipped entries, and stops if BB calibration changes.
It waits for the terminal firmware result and for the flight capture window
before proceeding. Mocap continuity is judged from **QTM source stamps**
(deep subscription queue), not from when the runner's callbacks happen to
run. Its own checkpoint writes stall it by ~100 ms, which previously stopped a
run with a false "receive gap". A >100 ms stamp gap, a stamp stepping back or
a zero (unsynchronised) stamp during a throw marks that throw
`capture_complete: false` with `capture_rejected` and the session **carries
on**; the analysis lists it as rejected. Three consecutive rejected captures,
or mocap stale for >1 s, still stop the session. This is a stream guard, not
proof of ball visibility; extraction still rejects occluded, ambiguous or
contacted arcs.

### Recording in QTM at the same time

A simultaneous QTM recording is fine as a backup. Starting a QTM capture
restarts QTM's clock (`mocap_node` logs "QTM timestamp discontinuity" and
re-syncs within ~0.5 s), and ending one does the same. So: set QTM's capture
duration longer than the whole session (277 throws at ~10.5 s plus refills
is ~55–60 min; use e.g. 90 min), start the QTM capture, wait a couple of
seconds, then start the runner, and stop the capture only after the runner
reports the bag closed. A restart during a throw rejects that throw only.
Final rosbag metadata is checked for nonempty raw mocap, BB heartbeat,
calibration and event topics before reporting recording success.

Ctrl-C prevents subsequent throws. **A goal already sent cannot be cancelled
by the bridge**; the recorder stays up through its predicted flight window
before closing. Ambiguous goal/result timeouts are marked `unknown_do_not_retry`;
the script never repeats an uncertain throw. On hopper/reload timeout, failure,
or interruption, partial data remains usable and the process returns failure.
Refill and run a fresh session instead of mixing uncertain retries into one
ordered list. The recorder is sent SIGINT to finalize MCAP; an unfinalized or
incomplete recording is flagged for review.

## Analysis immediately after the run

Install offline dependencies once in the analysis Python environment:

```bash
python -m pip install numpy mcap mcap-ros2-support
```

Then, from this directory (ROS2 is not needed for analysis):

```bash
python analyze_local_calibration.py /path/to/session/session.json
```

The default input is the sibling `bag/` directory. Use `--data /path/to/bag`
after copying a session to another machine. Results go into `analysis/`:
`report.md`, `report.json`, `extraction.json` (including rejection reasons),
and `correction_candidate.json`. At least 12 usable target cells with a
well-conditioned 2D spread are required. Partial campaigns may be analysed;
the report retains the session/recording-review status.

For the ten-throw pilot, add `--extract-only`. This writes `extraction.json`
and `extraction_report.json` with accepted/rejected flights and measured misses,
without attempting the affine fit's 12-cell minimum. Check this pilot before
starting the full campaign. A requested recorder SIGINT may return code 2 on
Foxy; this is accepted only when finalized metadata has all required topics.
Older sessions retain their original review flag; extraction can still inspect
them without modifying that historical record.

### Trajectory extraction

The extractor reads **every** marker and ignores QTM labels. On the
2026-10-07 hardware pilot QTM gave the thrown ball rigid-body labels
(`Base - 1/3/4/7`) for part or all of its flight, so labels are not identity
evidence. Exactly repeated frames (the 200 Hz publisher re-sending a 300 Hz
QTM frame) are counted once.

A track is accepted only if all of these hold (constants at the top of
`analyze_local_calibration.py`):

- **Launch association:** the fitted arc passes within 100 mm of BB's
  predicted release position at an arc time within ±0.10 s of nominal release,
  with release velocity within 30%. Seeds anchor at that release position and
  are free in velocity, so landing XY is never pulled toward the target or a
  predicted landing.
- **Free flight:** one constant-acceleration fit (per-axis) with acceleration
  within 6% of g, ≥30 frames over ≥0.25 s, 8 mm inliers and ≤5 mm RMS. A
  gravity-only arc is the wrong model over a full flight: drag plus a ~1°
  effective-gravity tilt leave 30–50 mm residuals at the arc ends.
- **Occlusion:** earlier gaps up to 0.45 s are bridged (the ball is often
  unseen near the apex), but the segment through the catch plane must be
  continuous (gaps ≤80 ms) for ≥0.25 s and observed to within 15 ms of the
  descending crossing. A crossing is never extrapolated further.
- **Measurement:** catch-plane XY comes from a local fit over −150/+40 ms
  around the crossing (≥15 frames, track acceleration held fixed). On the pilot
  it agreed with an independent drag + tilted-gravity ODE fit within 0.9–2.1 mm.
- **Ambiguity:** a distinct track (<50% shared frames) with ≥80% of the support
  and a landing >10 mm away rejects the throw.

Each rejection in `extraction.json` names the furthest gate reached and its
values, with per-gate failure counts under `diagnostics`. Accepted rows report
samples, observed span, final continuous segment, maximum gap, fit and local
RMS, acceleration, launch closest approach and `arc_time_offset_s` (arc time,
relative to nominal release, at which the track passes the predicted release
point). Raw source timestamps are required; frames without them are excluded.
A rejected throw is never assigned another ball or replaced by a prediction.

The fit gives each target cell equal weight, using its mean measured landing.
It fits commanded-to-measured response and inverts it to obtain a correction,
in BB-local XY millimetres. Five-fold validation holds out whole target cells
(including all repeats). The report separates two-ball-core residuals and
within-cell throw scatter. Cross-validation measures model prediction error;
it is not a hardware test of corrected throws.

**Use this candidate only with the corrected positive hand offset.** It replaces
the old affine; do not stack the transforms or the old feed-bias compensation.
Production configuration is unchanged. Confirm performance with a small
corrected hardware pilot before using the fit for juggling.

## Hardware pilot, 2026-10-07

Session `~/bb_calibration_sessions/20261007T043306_968166Z` (10 throws,
`--schedule-to-mocap 0.3 -0.6`). The original extractor rejected all ten; the
revised one accepts all ten (fit RMS 2.4–3.4 mm, local crossing RMS
0.3–0.8 mm, launch closest approach 5–11 mm). Uncorrected misses, measured
XY − target: 14–122 mm, mean (+53.7, +4.4) mm, growing with target X. The arc
passes the predicted release point 40–57 ms **before** nominal release, a
consistent offset not yet explained (see the logbook). Ten throws over ten
cells are below the affine's 12-cell minimum by design; no correction is fitted.

```bash
python analyze_local_calibration.py \
    ~/bb_calibration_sessions/20261007T043306_968166Z/session.json \
    --extract-only --out ~/bb_calibration_sessions/20261007T043306_968166Z/analysis_fixed
```

## Reproduce the simulated campaign

```bash
python simulate_local_calibration.py --out /path/to/new_simulation
python analyze_local_calibration.py /path/to/new_simulation/session.json --data /path/to/new_simulation/observations.jsonl
python validate_simulated_calibration.py /path/to/new_simulation/session.json --out /path/to/new_simulation/validation.json
```

The seeded 200 Hz simulation uses all 331 targets/repeats, independent uniform
noise of +/-2 mm on every coordinate, and independent uniform +/-1% errors
on each release-velocity component per throw. Release position is unchanged.
An injected small affine response supplies a known systematic error. Balls
bounce slightly, roll, and disappear one second after first ground contact.
Static markers and human refill projectile arcs are included; marker order is
shuffled every frame. `truth.json` is separate and never read by the analysis.
The synthetic BB pose makes all planned targets reachable and is not a
recommendation for hardware placement. Real reachability still depends on
your measured pose. This simulation does not cover every reflection, contact,
occlusion or timing error that might occur on hardware.

Seeded validation results are saved in `simulated_calibration_validation.json`
(regenerated 2026-10-07 with the revised extractor).
All 331 flights were recovered; estimated catches differed from hidden truth
by 0.47 mm RMS. The recovered correction differed from the true inverse by
0.78 mm RMS across the grid. On fresh independent throw states, core-area
RMS error fell from 24.36 to 10.09 mm, with mean bias reduced from
(+10.03, +19.70) to (+0.47, +0.07) mm. Across the full grid RMS fell from
25.32 to 10.55 mm. The remaining random 1% launch variation cannot be removed
by a deterministic affine. These are simulated results, not predicted hardware
accuracy. The simulation still assumes gravity-only flight, visible launches
and nominal release timing; the hardware-shaped cases (labels, drag/tilt,
apex occlusion, early arc) are covered by unit tests, not by this campaign. The independent validator re-solves corrected commands through the
production geometry and generates new physical trajectories.

## Offline verification

```bash
python -m unittest discover -s . -p 'test*local_calibration.py' -v
```

Tests cover 50–200 mm spacing, randomized repeat balance, log-derived core,
frame translation, malformed plans, unreachable targets, stale-data gates,
recorded-topic checks, refill-window exclusion, incomplete/ambiguous capture
rejection, a hardware-shaped flight (rigid-body-labelled ball, drag and tilted
gravity, early arc, apex occlusion, repeated frames), non-BB arcs and
unobserved crossings, operator-pause/EOF/quit behavior, waiting through ground arrival,
and actual production inverse/forward round trips with positive s.
Live DDS/action/recorder integration still needs the Jetson pilot.

The comparison plot is now available as `mirrored_hand_comparison.png`.
`render_mirrored_comparison.py` regenerates PNG and UTF-8 SVG using Pillow;
the earlier Windows-encoded SVG was invalid XML and had clipped plot bounds.
