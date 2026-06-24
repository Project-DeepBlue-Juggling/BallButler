#!/usr/bin/env python3
"""
validate_loud_throw_action.py — drive the Phase-2 loud-throw-action validation.

Unlike the Phase-1 driver (validate_settle_gates.py, which read BB's serial by
eye because /bb/send_throw_command only confirmed *delivery*), this script talks
to the new **/bb/throw ROS2 action** and ASSERTS the firmware outcome
programmatically. The action result carries the firmware's terminal CMD_RESULT —
OK (ball released on-aim), a REJECTED_* reason, or ABORTED_NOT_SETTLED — so each
test passes/fails on its own, no serial-watching required.

It exercises:
  * Layer A (predictive settle-before-wind-up reject) — naming the binding axis,
  * the accept path (arms + throws -> OK at completion),
  * the one-outstanding-throw goal gate (a concurrent second goal is rejected),
  * (optional) the BB-detached timeout (action times out, does NOT hang).

Watching `pio device monitor` on BB serial is still a useful CROSS-CHECK (the
[SM]/[Gate] lines and the CMD_RESULT), but it is no longer the source of truth.

Track-hold (production-faithful): a throw only fires on-aim if pitch is already
engaged (CLOSED_LOOP) at the pose. Single-park + wait lets tracking time out and
pitch drifts to stow, which Layer C then (correctly) aborts. So before each
throw we send a few aim-only goals (speed=0 -> firmware requestTracking; the
action dispatches these fire-and-forget, no CMD_RESULT awaited) to slew pitch
there AND keep it engaged, then throw promptly.

Safety:
  * Armed throws are apex-capped < 0.5 m (speed 2.0 m/s -> apex <= ~0.20 m at any
    pitch). The script refuses anything that exceeds the ceiling.
  * Keep the area in front of BB clear — accept cases release a ball.
  * Reject cases do NOT throw (Layer A rejects before the axes move).

Pre-reqs (confirm yourself): ROS2 up with the bridge + ball_butler nodes running
the rebuilt install; BB `status` shows TimeSync: YES, Ball: YES, Hand homed: YES.

Run (with the ROS overlay sourced, using the python that has rclpy):
    source /opt/ros/foxy/setup.bash
    source ~/Desktop/Jugglebot/ros_ws/install/setup.bash
    python3 validate_loud_throw_action.py            # interactive
    python3 validate_loud_throw_action.py --yes      # no confirm prompt
    python3 validate_loud_throw_action.py --timeout-test   # incl. BB-detached step
"""
import argparse
import math
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from jugglebot_interfaces.action import BallButlerThrowCmd
from jugglebot_interfaces.msg import BallButlerHeartbeat
from jugglebot.protocol_config import BallButlerCommandOutcome as OUT
from jugglebot.protocol_config import BallButlerStates as ST

G = 9.81
CEILING_M = 0.5
THROW_SPEED = 2.0      # m/s — apex <= ~0.20 m at any pitch (within the 0.5 m ceiling)
RELOAD_S = 6.0         # wait after an armed throw, for the reload cycle
TRACK_GAP_S = 0.4      # spacing between aim goals while slewing to the pose
AIM_TOL_DEG = 2.0      # pitch/yaw "settled at pose" tolerance
AIM_TIMEOUT_S = 12.0   # max time to slew+settle to a pose before giving up

AXIS_NAME = {0: 'YAW', 1: 'PITCH', 2: 'BOTH', -1: 'n/a'}


def apex_m(speed_mps, pitch_deg):
    return (speed_mps * math.sin(math.radians(pitch_deg))) ** 2 / (2.0 * G)


def outcome_name(value):
    try:
        return OUT(int(value)).name
    except ValueError:
        return f"UNKNOWN(0x{int(value):02x})"


class ThrowValidator(Node):
    def __init__(self):
        super().__init__('bb_throw_validator')
        self._client = ActionClient(self, BallButlerThrowCmd, '/bb/throw')
        self.results = []   # (label, passed, detail)
        self._hb = None     # latest BallButlerHeartbeat (for settle-gated aim)
        self.create_subscription(BallButlerHeartbeat, 'bb/heartbeat',
                                 self._on_hb, 20)

    def _on_hb(self, msg):
        self._hb = msg

    # ── low-level action plumbing ────────────────────────────────────────────
    def _send(self, yaw_deg, pitch_deg, speed, throw_time):
        """Send one goal. Returns (accepted, result_or_None). Blocks for the
        result (the action completes on the firmware's CMD_RESULT, or — for
        speed=0 aim goals — immediately on dispatch)."""
        goal = BallButlerThrowCmd.Goal()
        goal.yaw_angle_rad = math.radians(yaw_deg)
        goal.pitch_angle_rad = math.radians(pitch_deg)
        goal.throw_speed = float(speed)
        goal.throw_time = float(throw_time)
        goal.suppress_announcement = True

        gf = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, gf, timeout_sec=10.0)
        gh = gf.result()
        if gh is None or not gh.accepted:
            return False, None
        rf = gh.get_result_async()
        # generous: throw_time lead + wind-up/throw/reload-independent completion
        rclpy.spin_until_future_complete(self, rf, timeout_sec=throw_time + 15.0)
        wrapped = rf.result()
        return True, (wrapped.result if wrapped is not None else None)

    def _send_goal_only(self, yaw_deg, pitch_deg, speed, throw_time):
        """Send a goal and return its goal_handle WITHOUT awaiting the result —
        used to hold a throw 'outstanding' for the concurrency test."""
        goal = BallButlerThrowCmd.Goal()
        goal.yaw_angle_rad = math.radians(yaw_deg)
        goal.pitch_angle_rad = math.radians(pitch_deg)
        goal.throw_speed = float(speed)
        goal.throw_time = float(throw_time)
        goal.suppress_announcement = True
        gf = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, gf, timeout_sec=10.0)
        return gf.result()

    # ── high-level helpers ───────────────────────────────────────────────────
    def track_hold(self, yaw_deg, pitch_deg):
        """Production-faithful aim: keep sending aim-only goals (speed=0) while
        polling bb/heartbeat until BOTH axes are within tolerance of the pose AND
        BB is in TRACKING (pitch is the slow axis, so this can take a few
        seconds after a reload leaves it at the ~90 deg stow). Returns True if the
        pose is confirmed settled; False on timeout (the throw will then likely be
        a fair test of whatever axis is actually still moving)."""
        print("\n--- TRACK-HOLD -> yaw=%g deg pitch=%g deg (slew + confirm settled) ---"
              % (yaw_deg, pitch_deg))
        t0 = time.time()
        settled = False
        while time.time() - t0 < AIM_TIMEOUT_S:
            self._send(yaw_deg, pitch_deg, 0.0, 0.0)   # aim goal (fire-and-forget)
            # drain heartbeats for a beat so self._hb is current
            t1 = time.time()
            while time.time() - t1 < TRACK_GAP_S:
                rclpy.spin_once(self, timeout_sec=0.1)
            hb = self._hb
            if hb is None:
                print("  ... no bb/heartbeat yet (is the bridge publishing?)")
                continue
            dp = abs(hb.pitch_deg - pitch_deg)
            dy = abs((hb.yaw_deg - yaw_deg + 180.0) % 360.0 - 180.0)  # wrap-aware
            tracking = int(hb.state) == int(ST.TRACKING)
            print("  aim: pitch=%.1f (d%.1f) yaw=%.1f (d%.1f) state=%d%s"
                  % (hb.pitch_deg, dp, hb.yaw_deg, dy, int(hb.state),
                     " TRACKING" if tracking else ""))
            if tracking and dp <= AIM_TOL_DEG and dy <= AIM_TOL_DEG:
                settled = True
                break
        print("  -> %s" % ("SETTLED on-aim" if settled
                           else "NOT settled within %.0fs (proceeding anyway)" % AIM_TIMEOUT_S))
        return settled

    def record(self, label, passed, detail):
        self.results.append((label, passed, detail))
        print("  [%s] %s" % ("PASS" if passed else "FAIL", detail))

    def expect_accept(self, yaw, pitch, throw_time, label):
        a = apex_m(THROW_SPEED, pitch)
        print("\n=== %s ===" % label)
        if a > CEILING_M:
            self.record(label, False, "SKIPPED — apex %.2f m exceeds %.1f m ceiling"
                        % (a, CEILING_M))
            return
        print("  throw: yaw=%g pitch=%g speed=%.1f lead=%.1fs (apex~%.2f m) — expect OK/threw"
              % (yaw, pitch, THROW_SPEED, throw_time, a))
        accepted, res = self._send(yaw, pitch, THROW_SPEED, throw_time)
        if not accepted:
            self.record(label, False, "goal REJECTED by server (expected accept+throw)")
            return
        if res is None:
            self.record(label, False, "no result before timeout (firmware silent?)")
            return
        passed = bool(res.success) and int(res.outcome) == int(OUT.OK)
        self.record(label, passed, "outcome=%s success=%s msg=%r"
                    % (outcome_name(res.outcome), res.success, res.message))

    def expect_reject(self, yaw, pitch, throw_time, label, want_axis):
        print("\n=== %s ===" % label)
        print("  throw: yaw=%g pitch=%g speed=%.1f lead=%.1fs — expect REJECT (axis %s), NO throw"
              % (yaw, pitch, THROW_SPEED, throw_time, AXIS_NAME.get(want_axis, '?')))
        accepted, res = self._send(yaw, pitch, THROW_SPEED, throw_time)
        if not accepted:
            self.record(label, False, "goal REJECTED at server level (expected accept+firmware reject)")
            return
        if res is None:
            self.record(label, False, "no result before timeout (firmware silent?)")
            return
        is_reject = (not res.success) and int(res.outcome) == int(OUT.THROW_REJECTED_CANT_MAKE_LEAD)
        axis_ok = int(res.detail0) == want_axis
        self.record(label, is_reject and axis_ok,
                    "outcome=%s axis=%s(detail0=%d) shortfall=%dms msg=%r"
                    % (outcome_name(res.outcome), AXIS_NAME.get(int(res.detail0), '?'),
                       int(res.detail0), int(res.detail1), res.message))

    def expect_concurrent_reject(self, yaw, pitch, label):
        """Hold one throw outstanding (generous lead) and fire a second — the
        second must be rejected at the goal level (one outstanding throw)."""
        a = apex_m(THROW_SPEED, pitch)
        print("\n=== %s ===" % label)
        if a > CEILING_M:
            self.record(label, False, "SKIPPED — apex %.2f m exceeds ceiling" % a)
            return
        print("  goal A: a real throw with a 3.0s lead (it WILL throw); goal B fired"
              " immediately — expect B REJECTED by the server.")
        gh_a = self._send_goal_only(yaw, pitch, THROW_SPEED, 3.0)
        if gh_a is None or not gh_a.accepted:
            self.record(label, False, "goal A unexpectedly rejected — cannot test")
            return
        gh_b = self._send_goal_only(yaw, pitch, THROW_SPEED, 3.0)
        b_rejected = (gh_b is None) or (not gh_b.accepted)
        self.record(label, b_rejected,
                    "B accepted=%s (want rejected)" % (None if gh_b is None else gh_b.accepted))
        # Let goal A run to completion (it throws) so the slot frees + bench reloads.
        rf = gh_a.get_result_async()
        rclpy.spin_until_future_complete(self, rf, timeout_sec=20.0)
        wrapped = rf.result()
        if wrapped is not None:
            r = wrapped.result
            print("  goal A completed: outcome=%s success=%s" %
                  (outcome_name(r.outcome), r.success))

    def expect_timeout(self, yaw, pitch, label):
        print("\n=== %s ===" % label)
        print("  Detach BB now (unplug its CAN/power or pull it off the bus). The")
        print("  bridge will queue the frame but BB never answers -> expect TIMEOUT.")
        try:
            input("  Press Enter once BB is detached... ")
        except EOFError:
            pass
        accepted, res = self._send(yaw, pitch, THROW_SPEED, 1.0)
        if not accepted:
            self.record(label, False, "goal rejected at server (unexpected)")
            return
        if res is None:
            self.record(label, False, "no result returned at all (action hung?)")
            return
        passed = (not res.success) and int(res.outcome) == int(OUT.TIMEOUT)
        self.record(label, passed, "outcome=%s success=%s msg=%r"
                    % (outcome_name(res.outcome), res.success, res.message))
        print("  Reconnect BB before continuing.")

    def summary(self):
        print("\n" + "=" * 70)
        npass = sum(1 for _, p, _ in self.results if p)
        for label, p, detail in self.results:
            print("  %-44s %s" % (label, "PASS" if p else "FAIL"))
        print("  ---")
        print("  %d/%d passed" % (npass, len(self.results)))
        print("=" * 70)
        return npass == len(self.results)


def wait(label, secs):
    print("  ... %s (%.0fs)" % (label, secs))
    time.sleep(secs)


def main():
    global THROW_SPEED
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--yes', action='store_true', help='skip the confirmation prompt')
    ap.add_argument('--timeout-test', action='store_true',
                    help='include the BB-detached timeout case (interactive)')
    ap.add_argument('--speed', type=float, default=THROW_SPEED,
                    help='throw speed m/s (default %.1f; raises the apex)' % THROW_SPEED)
    args = ap.parse_args()

    THROW_SPEED = args.speed
    if apex_m(THROW_SPEED, 90.0) > CEILING_M:
        print("Refusing: speed %.2f m/s exceeds the %.1f m apex ceiling at 90 deg."
              % (THROW_SPEED, CEILING_M))
        return 2

    print(__doc__)
    print("Layer-A rule: lead >= max(pitch_settle, yaw_settle) + 0.6 (wind-up) + 0.1 (margin).\n")
    if not args.yes:
        try:
            input("Confirm BB status (TimeSync/Ball/Hand-homed all YES), area clear, "
                  "then press Enter... ")
        except EOFError:
            pass

    rclpy.init()
    node = ThrowValidator()
    try:
        print("Waiting for the /bb/throw action server...")
        if not node._client.wait_for_server(timeout_sec=10.0):
            print("ERROR: /bb/throw action server not available. Is teensy_bridge_node "
                  "running on the rebuilt install?")
            return 3

        # 1) ACCEPT — pitch already on-aim (engaged via track-hold), comfortable lead.
        node.track_hold(0, 45)
        node.expect_accept(0, 45, 2.0, "T1 ACCEPT (delta~0, lead 2.0s)")
        wait("reload", RELOAD_S)

        # 2) REJECT — engaged at 30 deg, +50 deg pitch with too-short lead.
        node.track_hold(0, 30)
        node.expect_reject(0, 80, 1.0, "T2 REJECT pitch (d-pitch 50, lead 1.0s)", want_axis=1)
        wait("no throw -> no reload", 1.5)

        # 3) ACCEPT — same 50 deg pitch move, generous lead.
        node.track_hold(0, 30)
        node.expect_accept(0, 80, 3.0, "T3 ACCEPT (d-pitch 50, lead 3.0s)")
        wait("reload", RELOAD_S)

        # 4) REJECT — yaw move, short lead. (Adjust 45 deg if outside BB yaw limits.)
        node.track_hold(0, 45)
        node.expect_reject(45, 45, 1.0, "T4 REJECT yaw (d-yaw 45, lead 1.0s)", want_axis=0)
        wait("done", 1.5)

        # 5) CONCURRENCY — second goal rejected while one throw is outstanding.
        node.track_hold(0, 45)
        node.expect_concurrent_reject(0, 45, "T5 CONCURRENT goal rejected")
        wait("reload", RELOAD_S)

        # 6) (optional) TIMEOUT — BB detached, action times out instead of hanging.
        if args.timeout_test:
            node.expect_timeout(0, 45, "T6 TIMEOUT (BB detached)")

        ok = node.summary()
        return 0 if ok else 1
    finally:
        # Destroy the action client before the node so its __del__ doesn't race
        # the destroyed node handle (the InvalidHandle traceback on exit).
        try:
            node._client.destroy()
        except Exception:  # noqa: BLE001
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\naborted")
        sys.exit(1)
