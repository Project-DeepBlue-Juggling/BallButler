#!/usr/bin/env python3
"""Temporal-accuracy campaign orchestrator for the sensorized catching cone.

Runs one session of the temporal-accuracy protocol (see README.md in this
directory): commands Ball Butler throws at the catching cone via the
``bb/throw_at_target`` service, pairs each throw's ``ThrowAnnouncement``
(predicted ``landing_time``) with the cone's piezo ``cone/catch_event``
(measured impact wall-time), and records per-throw arrival error to a session
JSON for ``analyze_temporal_session.py``.

Run on the Jetson with the Jugglebot ROS2 stack up (teensy_bridge_node +
ball_butler_node + mocap_node with the Catching_Cone rigid body tracked):

    # Bring-up / tap test — just print cone heartbeats + catch events:
    python3 run_temporal_session.py --listen

    # Phase R — repeatability block (doubles as the offset-calibration block):
    python3 run_temporal_session.py --phase repeatability --throws 30 --delay 2.5

    # Phase D — delay sweep at fixed cone position:
    python3 run_temporal_session.py --phase delay-sweep \
        --delays 2.0 2.75 3.5 4.25 5.0 --throws-per 6

    # Phase X — distance sweep: ONE interactive run covering all positions.
    # Throws --throws balls to the current cone position, then pauses (press
    # Enter) while you move the cone — vary height too — for --positions
    # positions total:
    python3 run_temporal_session.py --phase distance-sweep \
        --positions 20 --throws 5 --delay 2.5

The arrival error recorded here is END-TO-END: it includes the cone-entry →
piezo fall-through time and any release lag, i.e. a systematic offset δ that
the analysis stage estimates and removes (protocol §Pass criteria).

Python 3.8 / ROS2 Foxy. Stdlib + rclpy + jugglebot_interfaces only.
"""

from __future__ import annotations

import argparse
import json
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node

from jugglebot_interfaces.msg import (
    BallButlerHeartbeat,
    CatchEvent,
    CatchingConeHeartbeat,
    ThrowAnnouncement,
)
from jugglebot_interfaces.srv import BallButlerThrow

# BB heartbeat state codes (protocol_config.BallButlerStates). Imported when
# the jugglebot package is on the path (normal when the ROS env is sourced);
# the fallback constants mirror config/protocol_config.yaml.
try:
    from jugglebot.protocol_config import BallButlerStates as _BB

    STATE_THROWING = int(_BB.THROWING)
    STATE_RELOADING = int(_BB.RELOADING)
except Exception:  # noqa: BLE001 — fallback keeps the tool standalone
    STATE_THROWING = 3
    STATE_RELOADING = 4

DEFAULT_TARGET = 'Catching_Cone'
DEFAULT_DELAY_S = 2.5

# Preflight gates (protocol §Preflight).
CONE_SYNC_RMS_MAX_US = 50      # warn above this; the cone reports its own jitter RMS
PREFLIGHT_TIMEOUT_S = 10.0

# Throw pairing/pacing.
MATCH_WINDOW_S = 1.5           # |catch - predicted landing| to accept a pairing
CATCH_GRACE_S = 3.0            # wait past predicted landing before declaring a miss
MIN_RELOAD_HEARTBEATS = 3      # mirrors ball_butler_node calib_min_reload_heartbeats
RELOAD_TIMEOUT_S = 30.0        # hopper likely empty if reload takes this long
ANNOUNCE_WAIT_S = 2.0


def _stamp_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


class TemporalSessionNode(Node):
    """Thin subscription/state holder; orchestration runs on the main thread."""

    def __init__(self):
        super().__init__('temporal_accuracy_session')
        self.lock = threading.Lock()

        self.cone_hb = None              # latest CatchingConeHeartbeat
        self.bb_hb = None                # latest BallButlerHeartbeat
        self.announcements = []          # [(recv_monotonic, ThrowAnnouncement)]
        self.catch_events = []           # [(recv_monotonic, CatchEvent)]
        self.reloading_beats = 0         # RELOADING heartbeats since reset

        self.create_subscription(
            CatchingConeHeartbeat, 'cone/heartbeat', self._on_cone_hb, 10)
        self.create_subscription(
            CatchEvent, 'cone/catch_event', self._on_catch, 10)
        self.create_subscription(
            BallButlerHeartbeat, 'bb/heartbeat', self._on_bb_hb, 20)
        self.create_subscription(
            ThrowAnnouncement, 'throw_announcements', self._on_announcement, 10)

        self.throw_client = self.create_client(BallButlerThrow, 'bb/throw_at_target')

    def _on_cone_hb(self, msg):
        with self.lock:
            self.cone_hb = msg

    def _on_bb_hb(self, msg):
        with self.lock:
            self.bb_hb = msg
            if msg.state == STATE_RELOADING:
                self.reloading_beats += 1

    def _on_catch(self, msg):
        with self.lock:
            self.catch_events.append((time.monotonic(), msg))

    def _on_announcement(self, msg):
        if msg.thrower_name != 'ball_butler':
            return
        with self.lock:
            self.announcements.append((time.monotonic(), msg))


def wait_for(predicate, timeout_s, poll_s=0.02):
    """Poll ``predicate`` until truthy or timeout; returns its final value."""
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        v = predicate()
        if v:
            return v
        time.sleep(poll_s)
    return predicate()


def _ask(prompt_text):
    """Blocking operator prompt; lowercased reply, EOF (piped stdin) → 'q'.
    The rclpy spin thread keeps subscriptions live while we block here."""
    try:
        return input(prompt_text).strip().lower()
    except EOFError:
        return 'q'


class Campaign:
    def __init__(self, node: TemporalSessionNode, out_path: Path, metadata: dict):
        self.node = node
        self.out_path = out_path
        self.session = dict(metadata)
        self.session['throws'] = []

    # ── Preflight ──────────────────────────────────────────────

    def preflight(self) -> bool:
        n = self.node
        print('Preflight: waiting for cone + BB heartbeats ...')

        def cone_ready():
            with n.lock:
                hb = n.cone_hb
            return hb if (hb and hb.connected) else None

        hb = wait_for(cone_ready, PREFLIGHT_TIMEOUT_S)
        if not hb:
            print('FAIL: no connected cone/heartbeat. Is the cone powered and '
                  'on CAN2, and teensy_bridge_node running?')
            return False
        if not hb.time_synced:
            print('FAIL: cone is not time-synced (state=%d). Wait for the '
                  'bridge 0x7DD broadcast to lock, then retry.' % hb.state)
            return False
        if hb.sync_rms_us > CONE_SYNC_RMS_MAX_US:
            print('WARN: cone sync jitter RMS %d us > %d us — timestamps may '
                  'be degraded.' % (hb.sync_rms_us, CONE_SYNC_RMS_MAX_US))
        print('Cone: connected, READY, time_synced, sync_rms=%d us' % hb.sync_rms_us)
        self.session['preflight_cone'] = {
            'state': int(hb.state),
            'sync_rms_us': int(hb.sync_rms_us),
            'time_synced': bool(hb.time_synced),
        }

        def bb_ready():
            with n.lock:
                hb2 = n.bb_hb
            return hb2 if (hb2 and hb2.connected and hb2.ball_in_hand) else None

        hb2 = wait_for(bb_ready, PREFLIGHT_TIMEOUT_S)
        if not hb2:
            print('FAIL: BB heartbeat absent, disconnected, or no ball in '
                  'hand. Load the hopper / run bb/reload, then retry.')
            return False
        print('BB: connected, ball_in_hand, state=%d' % hb2.state)

        if not n.throw_client.wait_for_service(timeout_sec=PREFLIGHT_TIMEOUT_S):
            print('FAIL: bb/throw_at_target service unavailable. Is '
                  'ball_butler_node running?')
            return False
        print('Preflight OK.\n')
        return True

    # ── One throw ──────────────────────────────────────────────

    def execute_throw(self, throw_idx: int, target: str, delay_s: float) -> dict:
        n = self.node
        rec = {'throw_idx': throw_idx, 'commanded_delay_s': delay_s}

        with n.lock:
            ann_mark = len(n.announcements)
            catch_mark = len(n.catch_events)
            n.reloading_beats = 0

        req = BallButlerThrow.Request()
        req.target_name = target
        req.throw_delay_s = float(delay_s)
        call_wall_ns = time.time_ns()
        future = n.throw_client.call_async(req)
        if not wait_for(lambda: future.done(), timeout_s=10.0):
            rec['status'] = 'throw-failed'
            rec['error'] = 'service call timed out'
            return rec
        res = future.result()
        rec['service'] = {
            'success': bool(res.success),
            'message': res.message,
            'yaw_rad': float(res.yaw_rad),
            'pitch_rad': float(res.pitch_rad),
            'throw_speed_mps': float(res.throw_speed_mps),
            'throw_delay_s': float(res.throw_delay_s),
            'predicted_tof_s': float(res.predicted_tof_s),
            'target_global_mm': [res.target_position_global_mm.x,
                                 res.target_position_global_mm.y,
                                 res.target_position_global_mm.z],
            'target_bb_local_mm': [res.target_position_bb_local_mm.x,
                                   res.target_position_bb_local_mm.y,
                                   res.target_position_bb_local_mm.z],
        }
        rec['service_call_wall_ns'] = call_wall_ns
        if not res.success:
            rec['status'] = 'throw-failed'
            rec['error'] = res.message
            return rec

        # Authoritative prediction: the ThrowAnnouncement this throw produced.
        def fresh_announcement():
            with n.lock:
                return n.announcements[ann_mark:] or None

        anns = wait_for(fresh_announcement, ANNOUNCE_WAIT_S)
        if anns:
            ann = anns[-1][1]
            landing_ns = _stamp_ns(ann.landing_time)
            rec['announcement'] = {
                'stamp_ns': _stamp_ns(ann.header.stamp),
                'throw_time_ns': _stamp_ns(ann.throw_time),
                'landing_time_ns': landing_ns,
                'predicted_tof_sec': float(ann.predicted_tof_sec),
                'landing_position_mm': [ann.landing_position.x,
                                        ann.landing_position.y,
                                        ann.landing_position.z],
                'target_id': ann.target_id,
            }
        else:
            # Fallback prediction from the service response (mark it — the
            # announcement path is the protocol's reference).
            landing_ns = call_wall_ns + int(
                (res.throw_delay_s + res.predicted_tof_s) * 1e9)
            rec['announcement'] = None
            print('  WARN: no ThrowAnnouncement seen; using service-derived '
                  'landing time')
        rec['landing_time_ns'] = landing_ns

        # Await the catch: events whose timestamp lands within MATCH_WINDOW_S
        # of the predicted landing. Wait until landing + grace.
        wait_s = max(0.0, (landing_ns - time.time_ns()) / 1e9) + CATCH_GRACE_S

        def matched_catch():
            with n.lock:
                fresh = n.catch_events[catch_mark:]
            hits = [m for _, m in fresh
                    if abs(_stamp_ns(m.catch_time) - landing_ns)
                    <= MATCH_WINDOW_S * 1e9]
            return hits or None

        hits = wait_for(matched_catch, wait_s)
        if not hits:
            rec['status'] = 'miss'
            print('  MISS — no catch event within ±%.1f s of predicted landing'
                  % MATCH_WINDOW_S)
            return rec

        evt = hits[0]
        catch_ns = _stamp_ns(evt.catch_time)
        rec['catch'] = {
            'catch_time_ns': catch_ns,
            'sequence': int(evt.sequence),
            'time_synced': bool(evt.time_synced),
            'retrigger_suppressed': bool(evt.retrigger_suppressed),
            'extra_events_in_window': len(hits) - 1,
        }
        rec['arrival_error_ms'] = (catch_ns - landing_ns) / 1e6
        if not evt.time_synced:
            # Timestamp is host-arrival, not impact truth — unusable for ±10 ms.
            rec['status'] = 'unsynced'
            print('  UNSYNCED catch (seq %d) — excluded from timing stats'
                  % evt.sequence)
        else:
            rec['status'] = 'ok'
            print('  catch seq %d: arrival error %+.1f ms'
                  % (evt.sequence, rec['arrival_error_ms']))
        return rec

    def recheck_gates(self) -> bool:
        """Light preflight re-check after the operator repositions the cone:
        cone heartbeat connected + time-synced, BB connected w/ ball in hand."""
        n = self.node

        def cone_ok():
            with n.lock:
                hb = n.cone_hb
            return hb if (hb and hb.connected and hb.time_synced) else None

        hb = wait_for(cone_ok, PREFLIGHT_TIMEOUT_S)
        if not hb:
            print('FAIL: cone heartbeat not connected/time-synced after the '
                  'move — check CAN2 cabling/power, then rerun.')
            return False
        if hb.sync_rms_us > CONE_SYNC_RMS_MAX_US:
            print('WARN: cone sync jitter RMS %d us > %d us'
                  % (hb.sync_rms_us, CONE_SYNC_RMS_MAX_US))

        def bb_ok():
            with n.lock:
                hb2 = n.bb_hb
            return hb2 if (hb2 and hb2.connected and hb2.ball_in_hand) else None

        if not wait_for(bb_ok, PREFLIGHT_TIMEOUT_S):
            print('FAIL: BB heartbeat absent/disconnected or no ball in hand.')
            return False
        return True

    def wait_reload(self) -> bool:
        """Mirror ball_butler_node's calibration reload pacing: >=N RELOADING
        heartbeats, then a state that is neither RELOADING nor THROWING, then
        ball_in_hand for the next throw."""
        n = self.node

        def reloaded():
            with n.lock:
                hb = n.bb_hb
                beats = n.reloading_beats
            return (hb is not None
                    and beats >= MIN_RELOAD_HEARTBEATS
                    and hb.state not in (STATE_RELOADING, STATE_THROWING)
                    and hb.ball_in_hand)

        if not wait_for(reloaded, RELOAD_TIMEOUT_S, poll_s=0.05):
            print('FAIL: reload did not complete within %.0f s — hopper '
                  'empty? Aborting session (data so far is saved).'
                  % RELOAD_TIMEOUT_S)
            return False
        return True

    # ── Session loop ───────────────────────────────────────────

    def run(self, schedule, target: str) -> None:
        self.session['n_throws_scheduled'] = len(schedule)
        for i, delay_s in enumerate(schedule):
            print('Throw %d/%d (delay %.2f s):' % (i + 1, len(schedule), delay_s))
            rec = self.execute_throw(i, target, delay_s)
            self.session['throws'].append(rec)
            self._save()
            if rec['status'] == 'throw-failed':
                print('Aborting: throw command failed — %s'
                      % rec.get('error', '?'))
                break
            if i + 1 < len(schedule) and not self.wait_reload():
                break
        ok = sum(1 for t in self.session['throws'] if t['status'] == 'ok')
        print('\nSession complete: %d/%d good throws. Data: %s'
              % (ok, len(self.session['throws']), self.out_path))

    def run_positions(self, n_positions: int, throws_per: int,
                      delay_s: float, target: str) -> None:
        """Phase X: ``throws_per`` throws to the current cone position, then a
        pause while the operator moves the cone (Enter to continue, q to
        finish early); repeat for ``n_positions`` positions. Throw records
        carry ``position``/``position_idx`` so the analyzer groups per
        position. JSON is rewritten after every throw — quitting early or
        crashing keeps everything thrown so far."""
        total = n_positions * throws_per
        self.session['n_positions'] = n_positions
        self.session['throws_per_position'] = throws_per
        self.session['n_throws_scheduled'] = total
        idx = 0
        aborted = False
        for p in range(n_positions):
            label = 'pos%02d' % (p + 1)
            if p > 0:
                ans = _ask('\nPosition %d/%d done. Move the cone to position '
                           '%d (vary the height too), check QTM still tracks '
                           '"%s", then press Enter (q = finish early): '
                           % (p, n_positions, p + 1, target))
                if ans in ('q', 'quit'):
                    print('Finishing early: %d/%d positions thrown.'
                          % (p, n_positions))
                    break
                if not self.recheck_gates():
                    break
            print('\n=== Position %d/%d (%s): %d throws, delay %.2f s ==='
                  % (p + 1, n_positions, label, throws_per, delay_s))
            done = 0
            while done < throws_per:
                print('Throw %d/%d (%s throw %d/%d):'
                      % (idx + 1, total, label, done + 1, throws_per))
                rec = self.execute_throw(idx, target, delay_s)
                rec['position'] = label
                rec['position_idx'] = p
                self.session['throws'].append(rec)
                self._save()
                idx += 1
                if rec['status'] == 'throw-failed':
                    # Don't lose a 100-throw session to one failed service
                    # call (typical cause: QTM lost the moved rigid body).
                    print('Throw command failed: %s' % rec.get('error', '?'))
                    if _ask('  Fix the issue, then press Enter to retry '
                            '(q = end session): ') in ('q', 'quit'):
                        aborted = True
                        break
                    if not self.recheck_gates():
                        aborted = True
                        break
                    continue
                done += 1
                is_last = (p == n_positions - 1 and done == throws_per)
                if not is_last and not self.wait_reload():
                    aborted = True
                    break
            if aborted:
                break
        ok = sum(1 for t in self.session['throws'] if t['status'] == 'ok')
        print('\nSession complete: %d/%d good throws. Data: %s'
              % (ok, len(self.session['throws']), self.out_path))

    def _save(self) -> None:
        tmp = self.out_path.with_suffix('.tmp')
        with open(tmp, 'w') as f:
            json.dump(self.session, f, indent=2)
        tmp.replace(self.out_path)


def listen_mode(node: TemporalSessionNode) -> None:
    """Bring-up mode: print cone heartbeat transitions and catch events."""
    print('Listening on cone/heartbeat + cone/catch_event (Ctrl-C to stop) ...')
    last_key = None
    seen = 0
    while True:
        with node.lock:
            hb = node.cone_hb
            events = list(node.catch_events)
        if hb is not None:
            key = (hb.connected, int(hb.state), hb.time_synced,
                   int(hb.sync_rms_us))
            if key != last_key:
                print('[hb] connected=%s state=%d time_synced=%s '
                      'sync_rms=%dus last_seq=%d' %
                      (hb.connected, hb.state, hb.time_synced,
                       hb.sync_rms_us, hb.last_catch_seq))
                last_key = key
        for _, evt in events[seen:]:
            catch_ns = _stamp_ns(evt.catch_time)
            delta_ms = (time.time_ns() - catch_ns) / 1e6
            print('[catch] seq=%d synced=%s retrigger=%s  t=%.6f  '
                  '(%.1f ms before now)' %
                  (evt.sequence, evt.time_synced, evt.retrigger_suppressed,
                   catch_ns / 1e9, delta_ms))
        seen = len(events)
        time.sleep(0.05)


def build_schedule(args) -> list:
    if args.phase == 'delay-sweep':
        import random
        sched = [d for d in args.delays for _ in range(args.throws_per)]
        random.Random(args.seed).shuffle(sched)
        return sched
    return [args.delay] * args.throws


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--listen', action='store_true',
                    help='bring-up mode: print cone heartbeats + catch events')
    ap.add_argument('--phase',
                    choices=['repeatability', 'delay-sweep', 'distance-sweep'])
    ap.add_argument('--target', default=DEFAULT_TARGET,
                    help='QTM rigid-body name (default %(default)s)')
    ap.add_argument('--label', default='',
                    help='free-form block label, e.g. cone distance "2.0m"')
    ap.add_argument('--throws', type=int, default=None,
                    help='throw count: total for repeatability (default 30); '
                         'per position for distance-sweep (default 5)')
    ap.add_argument('--positions', type=int, default=20,
                    help='number of cone positions (distance-sweep); the '
                         'runner pauses between positions while you move '
                         'the cone (default %(default)s)')
    ap.add_argument('--delay', type=float, default=DEFAULT_DELAY_S,
                    help='throw delay s (repeatability / distance-sweep)')
    ap.add_argument('--delays', type=float, nargs='+',
                    default=[2.0, 2.75, 3.5, 4.25, 5.0],
                    help='delay grid for delay-sweep')
    ap.add_argument('--throws-per', type=int, default=6,
                    help='throws per delay (delay-sweep)')
    ap.add_argument('--seed', type=int, default=42,
                    help='shuffle seed (delay-sweep)')
    ap.add_argument('--out-dir', default=str(Path(__file__).parent / 'sessions'))
    args = ap.parse_args(argv)

    if not args.listen and not args.phase:
        ap.error('either --listen or --phase is required')
    if args.throws is None:
        args.throws = 5 if args.phase == 'distance-sweep' else 30

    rclpy.init()
    node = TemporalSessionNode()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        if args.listen:
            listen_mode(node)
            return 0

        out_dir = Path(args.out_dir)
        out_dir.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
        label = ('_' + args.label.replace(' ', '-')) if args.label else ''
        out_path = out_dir / ('temporal_%s%s_%s.json'
                              % (args.phase, label, stamp))

        metadata = {
            'protocol': 'temporal-accuracy-v1',
            'phase': args.phase,
            'label': args.label,
            'target': args.target,
            'timestamp': stamp,
            'args': {k: v for k, v in vars(args).items() if k != 'listen'},
        }
        campaign = Campaign(node, out_path, metadata)
        if not campaign.preflight():
            return 1
        if args.phase == 'distance-sweep':
            campaign.run_positions(args.positions, args.throws, args.delay,
                                   args.target)
        else:
            campaign.run(build_schedule(args), args.target)
        return 0
    except KeyboardInterrupt:
        print('\nInterrupted — partial session data saved.')
        return 130
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
