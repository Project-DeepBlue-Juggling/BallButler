#!/usr/bin/env python3
"""
validate_settle_gates.py — drive the Phase-1 axis-settle-gate validation.

Sends a short, safe sequence to BB via `/bb/send_throw_command` to exercise
Layer A (predictive settle-before-wind-up gate). Watch `pio device monitor` on
BB's serial for the `[SM]` / `[Gate]` lines — THOSE are the ground truth. The
ROS service response only tells you the command was *delivered* (success=True)
or had bad args (success=False); the accept/reject is decided in BB firmware and
printed on its serial.

IMPORTANT — track-hold, not single-park:
  The first hardware run aborted "good" throws because a single park command +
  a 5 s wait let BB's tracking time out -> pitch dropped to IDLE and drifted to
  the ~90 deg stow/home position before the throw. In production the throw
  director tracks CONTINUOUSLY, so pitch stays engaged on-aim. This script now
  mimics that: track_hold() sends repeated tracking commands (speed=0) so pitch
  reaches the pose AND stays in CLOSED_LOOP, then the throw fires immediately
  (within the tracking timeout) -> pitch is engaged + on-aim, the real test.

Safety:
  * Armed throws are apex-capped < 0.5 m above the launch point (speed 2.0 m/s
    -> apex <= ~0.20 m at any pitch). The script refuses anything that exceeds it.
  * Keep the area in front of BB clear — armed throws release a ball.
  * Reject cases do NOT throw (Layer A rejects before the axes move).

Pre-reqs (confirm yourself): ROS2 up; BB `status` shows TimeSync: YES, Ball: YES,
Hand homed: YES.

Usage:  python3 validate_settle_gates.py     (run with the venv/ROS sourced)
"""
import math
import re
import subprocess
import sys
import time

G = 9.81
CEILING_M = 0.5
SERVICE = "/bb/send_throw_command"
SRV_TYPE = "jugglebot_interfaces/srv/SendBallButlerCommand"

THROW_SPEED = 2.0     # m/s — apex <= ~0.20 m at any pitch (within the 0.5 m ceiling)
RELOAD_S = 6.0        # wait after an armed throw, for the reload cycle
TRACK_CMDS = 3        # tracking commands per track-hold (keeps pitch engaged)
TRACK_GAP_S = 1.5     # spacing between tracking commands (< 5 s tracking timeout)


def apex_m(speed_mps, pitch_deg):
    return (speed_mps * math.sin(math.radians(pitch_deg))) ** 2 / (2.0 * G)


def _call(yaw_deg, pitch_deg, speed, throw_time):
    req = ("{yaw_angle_rad: %.5f, pitch_angle_rad: %.5f, throw_speed: %.3f, "
           "throw_time: %.3f, suppress_announcement: false}" %
           (math.radians(yaw_deg), math.radians(pitch_deg), speed, throw_time))
    try:
        out = subprocess.run(["ros2", "service", "call", SERVICE, SRV_TYPE, req],
                             stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                             universal_newlines=True, timeout=25)
        return out.stdout or ""
    except Exception as e:  # noqa: BLE001
        return "EXC:%s" % e


def track_hold(yaw_deg, pitch_deg):
    """Hold the platform on-aim with repeated tracking (speed=0) commands so
    pitch slews there AND stays in CLOSED_LOOP. The throw must follow promptly."""
    print("\n--- TRACK-HOLD -> yaw=%g deg pitch=%g deg (keeping pitch engaged) ---"
          % (yaw_deg, pitch_deg))
    for i in range(TRACK_CMDS):
        txt = _call(yaw_deg, pitch_deg, 0.0, 0.0)
        ok = "success=True" in txt
        print("  track %d/%d: %s" % (i + 1, TRACK_CMDS, "ok" if ok else "FAILED"))
        if i < TRACK_CMDS - 1:
            time.sleep(TRACK_GAP_S)


def throw(yaw_deg, pitch_deg, throw_time, label, expect):
    a = apex_m(THROW_SPEED, pitch_deg)
    print("\n=== %s ===" % label)
    if a > CEILING_M:
        print("  !! SKIPPED — predicted apex %.2f m exceeds the %.1f m ceiling" % (a, CEILING_M))
        return
    print("  throw: yaw=%g deg pitch=%g deg speed=%.1f m/s lead=%.1fs (apex~%.2f m)"
          % (yaw_deg, pitch_deg, THROW_SPEED, throw_time, a))
    print("  EXPECT on BB serial: %s" % expect)
    txt = _call(yaw_deg, pitch_deg, THROW_SPEED, throw_time)
    ok = "success=True" in txt
    m = re.search(r"message=['\"](.*?)['\"]", txt)
    print("  service: %s%s" % ("delivered" if ok else "NOT delivered",
                               (" — %s" % m.group(1)) if m else ""))
    if not ok and "ERR_BAD_ARGS" in txt:
        print("  (bad args — ranges: pitch 0..1.571 rad, yaw +-pi, speed 0..6.55)")


def wait(label, secs):
    print("  ... %s (%.0fs)" % (label, secs))
    time.sleep(secs)


def main():
    print(__doc__)
    print("Layer-A: lead >= max(pitch_settle, yaw_settle) + 0.6 (wind-up) + 0.1 (margin).\n")
    try:
        input("Start `pio device monitor` on BB, confirm TimeSync: YES, then press Enter... ")
    except EOFError:
        pass

    # 1) ACCEPT — pitch already on-aim (engaged via track-hold), comfortable lead.
    track_hold(0, 45)
    throw(0, 45, 2.0, "TEST 1 — ACCEPT (delta~0, lead 2.0s)",
          "[SM] Throw queue diag: pitch_state=8 (CLOSED_LOOP) ... -> [SM] Throw armed -> BALL THROWN")
    wait("reload", RELOAD_S)

    # 2) REJECT — engaged at 30 deg, command +50 deg pitch with too-short lead.
    track_hold(0, 30)
    throw(0, 80, 1.0, "TEST 2 — REJECT (delta-pitch 50 deg, lead 1.0s)",
          "[SM] Throw rejected: PITCH can't settle before wind-up — need ~1950 ms, have ~1000 ms  (NO throw)")
    wait("no throw -> no reload", 1.5)

    # 3) ACCEPT — same 50 deg pitch move, generous lead.
    track_hold(0, 30)
    throw(0, 80, 3.0, "TEST 3 — ACCEPT (delta-pitch 50 deg, lead 3.0s)",
          "[SM] Throw armed -> slews to 80 deg, BALL THROWN")
    wait("reload", RELOAD_S)

    # 4) REJECT — yaw move, short lead. Adjust 45 deg if outside BB's yaw limits.
    track_hold(0, 45)
    throw(45, 45, 1.0, "TEST 4 — REJECT (delta-yaw 45 deg, lead 1.0s)",
          "[SM] Throw rejected: YAW can't settle before wind-up — need ~1450 ms, have ~1000 ms  (NO throw)")
    wait("done", 1.0)

    print("\n" + "=" * 70)
    print("With track-hold keeping pitch engaged on-aim, TEST 1 should now ARM+THROW")
    print("(pitch_state=8 at queue, settled-confirm OK at fire). 2 & 4 reject by axis,")
    print("3 accepts. If TEST 1 still aborts with pitch_state=8/pitch_done=0, paste the")
    print("[SM] queue diag + [Gate] lines — that points at re-engage latency, not idle.")
    print("=" * 70)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\naborted")
        sys.exit(1)
