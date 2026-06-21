#!/usr/bin/env python3
"""
validate_settle_gates.py — drive the Phase-1 axis-settle-gate validation.

Sends a short, safe sequence of throw / tracking commands to BB via
`/bb/send_throw_command` to exercise Layer A (predictive settle-before-wind-up
gate). Watch `pio device monitor` on BB's serial for the `[SM]` / `[Gate]` lines
— THOSE are the ground truth. The ROS service response only tells you the
command was *delivered* (success=True) or had bad args (success=False); the
actual accept/reject is decided in BB firmware and printed on its serial.

Safety:
  * Every armed throw is capped so its apex stays < 0.5 m above the launch point
    (throw_speed 2.0 m/s -> apex <= ~0.20 m at any pitch). The script refuses any
    throw it computes as exceeding the ceiling.
  * Make sure the area in front of BB is clear — armed throws DO release a ball.
  * Reject cases do NOT throw (Layer A rejects in requestThrow before the axes
    even move), so they leave the platform where it is and are safe to repeat.

Pre-reqs (the script can't check these — confirm yourself):
  * ROS2 network up; `ros2 service list` shows /bb/send_throw_command.
  * BB `status` shows `TimeSync: YES` (else throws reject with
    "time sync not established"), `Ball: YES`, and `Hand homed: YES`.

Usage:  python3 validate_settle_gates.py        (run with the venv/ROS sourced)
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

THROW_SPEED = 2.0   # m/s — apex <= ~0.20 m at any pitch (within the 0.5 m ceiling)
RELOAD_S = 6.0      # wait after an armed throw, for the reload cycle
PARK_SETTLE_S = 5.0 # wait after a tracking/park command, for the platform to settle


def apex_m(speed_mps, pitch_deg):
    """Apex height above the launch point for a given launch speed/elevation."""
    return (speed_mps * math.sin(math.radians(pitch_deg))) ** 2 / (2.0 * G)


def send(yaw_deg, pitch_deg, speed, throw_time, label, expect):
    armed = speed > 0.0
    if armed:
        a = apex_m(speed, pitch_deg)
        if a > CEILING_M:
            print("\n=== %s ===" % label)
            print("  !! SKIPPED — predicted apex %.2f m exceeds the %.1f m ceiling" % (a, CEILING_M))
            return False

    req = ("{yaw_angle_rad: %.5f, pitch_angle_rad: %.5f, throw_speed: %.3f, "
           "throw_time: %.3f, suppress_announcement: false}" %
           (math.radians(yaw_deg), math.radians(pitch_deg), speed, throw_time))

    print("\n=== %s ===" % label)
    if armed:
        print("  cmd: yaw=%g deg  pitch=%g deg  speed=%.1f m/s  lead=%.1fs  (apex~%.2f m)"
              % (yaw_deg, pitch_deg, speed, throw_time, apex_m(speed, pitch_deg)))
    else:
        print("  cmd: yaw=%g deg  pitch=%g deg  (TRACK/park, no throw)" % (yaw_deg, pitch_deg))
    print("  EXPECT on BB serial: %s" % expect)

    try:
        out = subprocess.run(["ros2", "service", "call", SERVICE, SRV_TYPE, req],
                             stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                             universal_newlines=True, timeout=25)
        txt = out.stdout or ""
    except Exception as e:  # noqa: BLE001
        print("  service call FAILED to run: %s" % e)
        return False

    ok = "success=True" in txt
    m = re.search(r"message=['\"](.*?)['\"]", txt)
    msg = m.group(1) if m else ""
    print("  service: %s%s" % ("delivered (success=True)" if ok else "NOT delivered (success=False)",
                               (" — %s" % msg) if msg else ""))
    if not ok and "ERR_BAD_ARGS" in txt:
        print("  (bad args — ranges: pitch 0..1.571 rad, yaw +-pi, speed 0..6.55, lead 0..65.5)")
    return ok


def park(yaw_deg, pitch_deg):
    send(yaw_deg, pitch_deg, 0.0, 0.0,
         "PARK -> yaw=%g deg pitch=%g deg" % (yaw_deg, pitch_deg),
         "platform slews to the pose (tracking; no throw)")
    print("  ... waiting %.0fs for the platform to settle" % PARK_SETTLE_S)
    time.sleep(PARK_SETTLE_S)


def wait(label, secs):
    print("  ... %s (%.0fs)" % (label, secs))
    time.sleep(secs)


def main():
    print(__doc__)
    print("Layer-A requirement: lead >= max(pitch_settle, yaw_settle) + 0.6 (wind-up) + 0.1 (margin).")
    print("  60 deg pitch ~1.41s settle; 50 deg ~1.25s; yaw 45 deg ~0.75s (60 deg/s model).\n")
    try:
        input("Start `pio device monitor` on BB, confirm TimeSync: YES, then press Enter (Ctrl-C aborts)... ")
    except EOFError:
        pass

    # 1) ACCEPT — near-zero delta, comfortable lead.
    park(0, 45)
    send(0, 45, THROW_SPEED, 2.0, "TEST 1 — ACCEPT (delta~0, lead 2.0s)",
         "[SM] Throw armed ...  -> BALL THROWN")
    wait("reload", RELOAD_S)

    # 2) REJECT — big pitch move, short lead. No throw; platform stays at 30 deg.
    park(0, 30)
    send(0, 80, THROW_SPEED, 1.0, "TEST 2 — REJECT (delta-pitch 50 deg, lead 1.0s)",
         "[SM] Throw rejected: axes can't settle before wind-up "
         "— need ~1950 ms, have ~1000 ms   (NO throw)")
    wait("no throw -> no reload needed", 1.5)

    # 3) ACCEPT — same 50 deg pitch move, generous lead (still parked at 30 deg).
    send(0, 80, THROW_SPEED, 3.0, "TEST 3 — ACCEPT (delta-pitch 50 deg, lead 3.0s)",
         "[SM] Throw armed ...  -> slews to 80 deg, BALL THROWN")
    wait("reload", RELOAD_S)

    # 4) REJECT — yaw move, short lead. Adjust the 45 deg target if it exceeds
    #    BB's yaw soft-limits (you'd see a different rejection then).
    park(0, 45)
    send(45, 45, THROW_SPEED, 1.0, "TEST 4 — REJECT (delta-yaw 45 deg, lead 1.0s)",
         "[SM] Throw rejected: ... yaw~750 ms -> need ~1450 ms, have ~1000 ms   (NO throw)")
    wait("done", 1.0)

    print("\n" + "=" * 70)
    print("Compare each test's [SM]/[Gate] serial line against its EXPECT above.")
    print("Layer-C 'pass' is proven implicitly: TEST 1 & 3 actually firing means")
    print("the fire-time settled-confirm let them through. To exercise the C *abort*")
    print("path, tighten AxisSettleCfg::YAW_RATE_TOL_DPS (e.g. 0.1), re-flash, and")
    print("run TEST 1 again -> expect '[Gate] Throw ABORTED', ball retained, no fire.")
    print("=" * 70)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\naborted")
        sys.exit(1)
