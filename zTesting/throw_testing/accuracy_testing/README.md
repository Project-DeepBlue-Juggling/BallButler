# Accuracy calibration

## Current flow (2026-05 onward)

The host-side aiming code lives in Jugglebot's `ball_butler_node.py`, which
owns the Cartesian-target → joint-space IK. To re-fit the affine correction:

1. **Run the session.** With the Jugglebot stack running, call the service:
   ```
   ros2 service call /bb/start_accuracy_calibration std_srvs/srv/Trigger
   ```
   `ball_butler_node` fires 50 throws (5×5 grid × 2 each, randomised order,
   fixed seed logged in the metadata JSON). It writes a metadata JSON to
   `~/bb_calibration_sessions/accuracy_session_<timestamp>.json` containing
   the exact throw schedule, grid params, BB pose, and whether any existing
   affine was applied. The rosbag captures every `ThrowAnnouncement`.

   To cancel mid-session:
   ```
   ros2 service call /bb/cancel_accuracy_calibration std_srvs/srv/Trigger
   ```

2. **Record QTM** in parallel and export ball trajectories to a `.json` file.

3. **Fit the affine.** From this directory:
   ```
   python fit_affine.py \
     ~/bb_calibration_sessions/accuracy_session_<timestamp>.json \
     ~/Desktop/qtm_export.json \
     --out throw_affine_correction.json
   ```
   The script pairs each ball trajectory to its commanded target by throw
   order (no manual cluster-mapping needed), transforms both into BB-local
   frame using the BB pose recorded in the session metadata, fits the 2D
   affine in BB-local frame, and writes `throw_affine_correction.json` plus
   a `.png` visualisation.

4. **Apply.** Copy `throw_affine_correction.json` into
   `Jugglebot/ros_ws/src/jugglebot/resources/` (the location
   `ball_butler_node` searches by default), then launch with the correction
   enabled:
   ```
   ros2 launch jugglebot jugglebot_launch.py \
     apply_aim_correction:=true
   ```
   Or override the file path via the `aim_correction_file` ROS param. The
   node logs whether it loaded a correction on startup.

5. **Verify.** Re-run the calibration session with the correction loaded;
   mean error should drop substantially. Launch with
   `apply_aim_correction:=false` to disable without removing the file.

## Historical sessions (older workflow)

Sessions 1-8 below used the now-archived `ball_butler_volley_testing_node`
and the manual cluster-mapping pipeline in `volley_viz.py`. The new flow
above replaces both.

- session 1 and 2 - 10 throws to 8 x 8 grid of 800 x 800 mm. Session 1 is first 368 throws, session 2 is rest (note that some throws were skipped due to them being unreachable)
- session 3 and 4 - 5 throws to 2 x 2 grid of 200 x 200 mm. Both had correction transformation matrix (from session 1 and 2) applied
- session 5 - 4 throws to 5 x 5 grid of 1000 x 1000 mm. No correction transformation matrix applied (post QTM calibration, and post fixing rotation bug from `global_to_bb_frame`
- session 6 - same as session 5, but after correcting the forward kinematics (with s, d and l offsets)
- session 7 - Timing test. 5 throws to each of 4 'delays'. Min delay = 2.0 sec, max delay = 5.0 sec, with 4 divisions. All aimed at (0, 0, 750) mm
- session 8 - same as 7, but after major refactor. Might fix throw delay?