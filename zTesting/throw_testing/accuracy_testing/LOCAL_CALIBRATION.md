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
the ball. This tool collects the calibration dataset; it does not fit or deploy
a replacement affine automatically. The new session JSON is not the legacy
`fit_affine.py` input schema: use the raw MCAP plus session JSON together for
the next analysis. Do not substitute predicted/tracker landing positions for
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

By default this prompt appears before the first throw and between every pair
of throws (`--refill-every 1`). To throw a batch from a full magazine before
pausing, use e.g. `--refill-every 5`, but **do not toss refill balls during
that automatic batch**, even after one ball has grounded: wait for `REFILL`.
The per-throw prompt is the recommended mode for routinely returning balls.

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

This remains a capture routine, not an automatic trajectory fitter. Raw
refill markers stay in the bag for audit, excluded by the recorded windows.
Within a BB window, subsequent analysis must still identify the outbound BB
arc and descending catch-plane crossing, reject bounces/reflections/occlusion,
and retain ambiguous cases for review. Do not use an arc-count/order matcher
over the entire bag. If a refill accidentally overlaps a BB capture, record
the affected throw number and exclude that capture from the fit.

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
before proceeding. A >100 ms observed mocap receive gap stops the session for
review. This is a coarse stream-loss guard, not proof of ball visibility;
offline extraction must still reject occluded, ambiguous or contacted arcs.
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

## Offline verification

```bash
python -m unittest discover -s . -p test_local_calibration.py -v
```

Tests cover 50–200 mm spacing, randomized repeat balance, log-derived core,
frame translation, malformed plans, unreachable targets, stale-data gates,
recorded-topic checks, refill-window exclusion, incomplete/ambiguous capture
rejection, operator-pause/EOF/quit behavior, waiting through ground arrival,
and actual production inverse/forward round trips with positive s.
Live DDS/action/recorder integration still needs the Jetson pilot.

The comparison plot is now available as `mirrored_hand_comparison.png`.
`render_mirrored_comparison.py` regenerates PNG and UTF-8 SVG using Pillow;
the earlier Windows-encoded SVG was invalid XML and had clipped plot bounds.
