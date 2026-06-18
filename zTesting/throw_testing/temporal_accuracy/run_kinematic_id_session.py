#!/usr/bin/env python3
"""
Cone-free kinematic system-ID session.

BB throws a grid of RAW (yaw, pitch, speed) combinations — no target, no IK, no
catching cone — via bb/send_throw_command (SendBallButlerCommand), while QTM/mocap
records the ball arcs. A few repeats per combo for high n. Each throw's commanded
params + wall-time are logged so the offline fit can match every mocap arc to its
combo by time/order (the raw throw service publishes NO announcement, by design).

Why cone-free: we control the (yaw, pitch, speed) grid directly, so we get exactly
the pitch/yaw/speed DIVERSITY the fit needs (the synthetic identifiability test in
calibrate_kinematics.py shows narrow-pitch can't separate offset from scale), with
no aim-correction confound. Each mocap arc gives BOTH the launch velocity (→ pitch/
yaw/speed execution errors) and the back-extrapolated release position (→ geometry).

SAFETY: balls fly with no catcher. Clear the downrange area / use a net. A minimum
pitch and maximum speed are enforced so throws stay lofted and short; override with
--unsafe (you accept the risk).

Procedure (ROS2 env sourced; teensy_bridge_node + mocap running; hopper loaded;
Jugglebot disconnected so the link has full bandwidth incl. /bb/axis_estimates):
  1. START RECORDING FIRST:   ros2 bag record -a -o <bagdir>
  2. python run_kinematic_id_session.py  [--pitches 45,55,65,75] [--yaws 0,20,40,60,80]
        [--peaks 0.18,0.27] [--repeats 2] [--delay 1.8] [--seed 42]
     (--peaks = COMMANDED peak height above launch in m; speed is derived per pitch so
      the ceiling is bounded by construction. The driver REFUSES any combo whose
      worst-case peak — commanded × overshoot-margin² — exceeds --max-height (0.5 m).)
Then analyse:  extract_kinematic_obs.py <bag>.mcap <session>.json  → calibrate_kinematics.py
"""
from __future__ import annotations

import argparse
import json
import math
import threading
import time
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node

from jugglebot_interfaces.msg import BallButlerHeartbeat
from jugglebot_interfaces.srv import SendBallButlerCommand

try:
    from jugglebot.protocol_config import BallButlerStates as _BB
    STATE_THROWING = int(_BB.THROWING)
    STATE_RELOADING = int(_BB.RELOADING)
except Exception:  # noqa: BLE001
    STATE_THROWING = 3
    STATE_RELOADING = 4

MIN_RELOAD_HEARTBEATS = 3
RELOAD_TIMEOUT_S = 30.0
PREFLIGHT_TIMEOUT_S = 10.0

G = 9.806

# ── CEILING SAFETY ────────────────────────────────────────────────────────────
# Peak throw height ABOVE the launch position must NEVER exceed the ceiling
# clearance, or the ball hits the ceiling. We parameterize the grid by COMMANDED
# peak height (speed is derived per pitch), so the ceiling is guaranteed by
# construction, and budget OVERSHOOT_MARGIN on the vertical launch velocity for
# the measured ~+12% release overshoot (actual peak = margin² × commanded peak).
# Peak height h = (v·sinθ)² / 2g; commanded vz = √(2g·h); speed = vz / sinθ.
MAX_HEIGHT_M = 0.5            # ceiling clearance above the launch position
OVERSHOOT_MARGIN = 1.30      # > measured vz overshoot (~1.12); headroom for uncertainty
MIN_SPEED_MPS = 1.8          # skip throws slower than this (poor arc / reload separation)
MAX_SPEED_MPS = 5.0          # absolute clamp
YAW_MIN_DEG = 0.0            # BB physical yaw axis range; outside this the firmware clamps
YAW_MAX_DEG = 185.0          # → corrupts the kinematic fit (see 2026-06-17 −20/−40° run)


def wait_for(predicate, timeout_s, poll_s=0.02):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        v = predicate()
        if v:
            return v
        time.sleep(poll_s)
    return predicate()


class GridNode(Node):
    def __init__(self):
        super().__init__('kinematic_id_session')
        self.lock = threading.Lock()
        self.bb_hb = None
        self.reloading_beats = 0
        self.create_subscription(BallButlerHeartbeat, 'bb/heartbeat', self._on_bb_hb, 20)
        self.throw_client = self.create_client(SendBallButlerCommand, 'bb/send_throw_command')

    def _on_bb_hb(self, msg):
        with self.lock:
            self.bb_hb = msg
            if msg.state == STATE_RELOADING:
                self.reloading_beats += 1


def _floats(s):
    return [float(x) for x in s.split(',') if x.strip() != '']


def peak_height_m(speed, pitch_deg):
    """Commanded peak height above launch (m) for a (speed, pitch)."""
    vz = speed * math.sin(math.radians(pitch_deg))
    return vz * vz / (2.0 * G)


def speed_for_peak(peak_m, pitch_deg):
    """Speed (m/s) that yields commanded peak height `peak_m` at `pitch_deg`."""
    vz = math.sqrt(2.0 * G * peak_m)
    return vz / math.sin(math.radians(pitch_deg))


def build_combos(pitches, yaws, peaks):
    """Grid over pitch × yaw × commanded-peak-height. Speed is DERIVED per
    (pitch, peak) so the ceiling is bounded by `peaks`, not by a speed guess.
    Returns (combos, skipped) where combo = dict(yaw,pitch,speed,peak_cmd,peak_actual)."""
    combos, skipped = [], []
    for p in pitches:
        for peak in peaks:
            spd = speed_for_peak(peak, p)
            peak_actual = peak * OVERSHOOT_MARGIN ** 2
            rec_base = dict(pitch=p, peak_cmd=peak, speed=spd, peak_actual=peak_actual)
            if spd < MIN_SPEED_MPS or spd > MAX_SPEED_MPS:
                skipped.append({**rec_base, 'why': f'speed {spd:.2f} out of [{MIN_SPEED_MPS},{MAX_SPEED_MPS}]'})
                continue
            for y in yaws:
                combos.append(dict(yaw=y, pitch=p, speed=spd,
                                   peak_cmd=peak, peak_actual=peak_actual))
    return combos, skipped


def schedule_from_combos(combos, repeats, seed):
    import random
    rng = random.Random(seed)
    sched = []
    for _ in range(repeats):
        block = list(combos)
        rng.shuffle(block)          # randomise within each repeat → decorrelate drift from the grid
        sched.extend(block)
    return sched


def preflight(n) -> bool:
    print('Preflight: waiting for BB heartbeat (connected + ball in hand) ...')

    def bb_ready():
        with n.lock:
            hb = n.bb_hb
        return hb if (hb and hb.connected and hb.ball_in_hand) else None

    hb = wait_for(bb_ready, PREFLIGHT_TIMEOUT_S)
    if not hb:
        print('FAIL: BB heartbeat absent/disconnected or no ball in hand. '
              'Load the hopper / run `ros2 service call /bb/reload std_srvs/srv/Trigger`.')
        return False
    print('BB ready: connected, ball_in_hand, state=%d' % hb.state)
    if not n.throw_client.wait_for_service(timeout_sec=PREFLIGHT_TIMEOUT_S):
        print('FAIL: bb/send_throw_command service unavailable. Is teensy_bridge_node running?')
        return False
    return True


def wait_reload(n) -> bool:
    with n.lock:
        n.reloading_beats = 0

    def reloaded():
        with n.lock:
            hb = n.bb_hb
            beats = n.reloading_beats
        return bool(hb and hb.connected and beats >= MIN_RELOAD_HEARTBEATS
                    and hb.state not in (STATE_RELOADING, STATE_THROWING)
                    and hb.ball_in_hand)

    if not wait_for(reloaded, RELOAD_TIMEOUT_S, poll_s=0.05):
        print('FAIL: reload did not complete within %.0f s — hopper empty?' % RELOAD_TIMEOUT_S)
        return False
    return True


def throw(n, yaw_deg, pitch_deg, speed, delay):
    req = SendBallButlerCommand.Request()
    req.yaw_angle_rad = math.radians(yaw_deg)
    req.pitch_angle_rad = math.radians(pitch_deg)
    req.throw_speed = float(speed)
    req.throw_time = float(delay)        # RELATIVE delay (s) → firmware schedules now+delay
    req.suppress_announcement = True
    t_call_ns = time.time_ns()
    fut = n.throw_client.call_async(req)
    if not wait_for(lambda: fut.done(), timeout_s=10.0):
        return t_call_ns, False, 'service timeout'
    r = fut.result()
    return t_call_ns, bool(r.success), str(r.message)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--pitches', default='45,55,65,75', help='deg, comma-sep')
    ap.add_argument('--yaws', default='0,20,40,60,80',
                    help='deg, comma-sep; MUST be within BB axis range [0,185] (else clamped)')
    ap.add_argument('--peaks', default='0.18,0.27',
                    help='COMMANDED peak heights above launch (m); speed derived per pitch')
    ap.add_argument('--repeats', type=int, default=2)
    ap.add_argument('--delay', type=float, default=1.8, help='throw lead time (s)')
    ap.add_argument('--seed', type=int, default=42)
    ap.add_argument('--settle', type=float, default=0.4, help='pause after each throw fires (s)')
    ap.add_argument('--max-height', type=float, default=MAX_HEIGHT_M,
                    help='ceiling clearance above launch (m); HARD limit')
    ap.add_argument('--out-dir', default='sessions')
    ap.add_argument('--label', default='')
    args = ap.parse_args()

    pitches, yaws, peaks = _floats(args.pitches), _floats(args.yaws), _floats(args.peaks)
    bad_yaw = [y for y in yaws if not (YAW_MIN_DEG <= y <= YAW_MAX_DEG)]
    if bad_yaw:
        print(f'REFUSING: yaw(s) {bad_yaw} outside BB axis range '
              f'[{YAW_MIN_DEG:.0f},{YAW_MAX_DEG:.0f}]° — the firmware silently clamps these, '
              f'which corrupts the kinematic fit. Use in-range yaws (e.g. 0,20,40,60,80).')
        return 2
    combos, skipped = build_combos(pitches, yaws, peaks)
    for s in skipped:
        print(f'  skip pitch={s["pitch"]:.0f} peak={s["peak_cmd"]:.2f}m: {s["why"]}')

    # HARD CEILING GUARD — refuse if any throw's worst-case (overshoot-budgeted)
    # peak could exceed the ceiling. No override: hitting the ceiling is unacceptable.
    over = [c for c in combos if c['peak_actual'] > args.max_height]
    if over:
        print(f'\nREFUSING: {len(over)} combo(s) could exceed the {args.max_height*1000:.0f} mm '
              f'ceiling (worst-case peak = {OVERSHOOT_MARGIN}²×commanded):')
        for c in sorted(over, key=lambda c: -c['peak_actual'])[:6]:
            print(f'  pitch={c["pitch"]:.0f} peak_cmd={c["peak_cmd"]*1000:.0f}mm '
                  f'speed={c["speed"]:.2f} → worst-case {c["peak_actual"]*1000:.0f}mm')
        print(f'Lower --peaks (max commanded peak = {args.max_height/OVERSHOOT_MARGIN**2*1000:.0f}mm).')
        return 2
    if not combos:
        print('No safe combos in the grid.'); return 2

    sched = schedule_from_combos(combos, args.repeats, args.seed)
    worst_peak = max(c['peak_actual'] for c in combos)
    spd_lo = min(c['speed'] for c in combos); spd_hi = max(c['speed'] for c in combos)
    # worst-case horizontal range to launch height: 2 v² sinθ cosθ / g
    def hrange(c):
        th = math.radians(c['pitch']); return 2 * c['speed'] ** 2 * math.sin(th) * math.cos(th) / G
    worst_range = max(hrange(c) for c in combos)
    est_min = len(sched) * (args.delay + args.settle + 2.0) / 60.0
    print(f'Grid: {len(pitches)}p × {len(yaws)}y × {len(peaks)} peaks = {len(combos)} safe combos '
          f'× {args.repeats} reps = {len(sched)} throws (~{est_min:.0f} min).')
    print(f'  speed range {spd_lo:.2f}–{spd_hi:.2f} m/s; '
          f'worst-case peak {worst_peak*1000:.0f} mm (ceiling {args.max_height*1000:.0f} mm); '
          f'worst-case horizontal range ~{worst_range:.1f} m to launch height.')
    print('\n*** SAFETY: balls fly with NO catcher. Confirm clearance: '
          f'~{worst_range:.1f} m downrange and {args.max_height*1000:.0f} mm overhead. ***')
    print('*** Confirm `ros2 bag record` (incl. /balls and /bb/axis_estimates) is RUNNING. ***')
    try:
        if input('Type "go" to start: ').strip().lower() != 'go':
            print('Aborted.'); return 1
    except EOFError:
        print('No TTY; aborting for safety.'); return 1

    rclpy.init()
    node = GridNode()
    spin = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin.start()

    session = dict(protocol='kinematic-id-v1', label=args.label,
                   timestamp=datetime.now().strftime('%Y-%m-%d_%H-%M-%S'),
                   args=vars(args), delay_s=args.delay, throws=[])
    out_dir = Path(args.out_dir); out_dir.mkdir(exist_ok=True)
    out_path = out_dir / f"kinematic_id_{args.label+'_' if args.label else ''}{session['timestamp']}.json"

    try:
        if not preflight(node):
            return 1
        for i, c in enumerate(sched):
            yaw_deg, pitch_deg, speed = c['yaw'], c['pitch'], c['speed']
            # Runtime ceiling re-check (belt + suspenders): never issue a throw
            # whose worst-case peak exceeds the limit.
            wc_peak = peak_height_m(speed, pitch_deg) * OVERSHOOT_MARGIN ** 2
            if wc_peak > args.max_height:
                print(f'[{i+1}] BLOCKED: worst-case peak {wc_peak*1000:.0f}mm > ceiling — skipped.')
                continue
            if i > 0 and not wait_reload(node):
                break
            t_call_ns, ok, msg = throw(node, yaw_deg, pitch_deg, speed, args.delay)
            rec = dict(idx=i, yaw_deg=yaw_deg, pitch_deg=pitch_deg, speed_mps=speed,
                       peak_cmd_mm=round(c['peak_cmd'] * 1000, 1),
                       peak_worstcase_mm=round(wc_peak * 1000, 1),
                       t_call_wall_ns=t_call_ns,
                       est_throw_wall_ns=t_call_ns + int(args.delay * 1e9),
                       success=ok, message=msg)
            session['throws'].append(rec)
            print(f'[{i+1:3d}/{len(sched)}] yaw={yaw_deg:+.0f} pitch={pitch_deg:.0f} '
                  f'speed={speed:.2f} peak≤{wc_peak*1000:.0f}mm  {"OK" if ok else "FAIL: "+msg}')
            out_path.write_text(json.dumps(session, indent=1))   # checkpoint each throw
            time.sleep(args.settle)
    finally:
        n_ok = sum(1 for t in session['throws'] if t['success'])
        print(f'\nDone: {n_ok}/{len(session["throws"])} throws issued OK.')
        print(f'Session log: {out_path}')
        print('Stop the bag recording, then run extract_kinematic_obs.py <bag> '
              f'{out_path.name}')
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
