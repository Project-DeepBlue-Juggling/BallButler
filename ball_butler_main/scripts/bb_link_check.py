#!/usr/bin/env python3
"""Check the can-bridge and Ball Butler on the raw UDP link — no ROS needed.

Prints the bridge's FW_VERSION (BRIDGE_IDENTITY), whether the Ball Butler CAN
bus carries anything (bridge PROFILE wire slot 2 = BB, frames/s), BB's heartbeat
state as the bridge sees it, and — BB FW >= 6 with bridge FW >= 28 — the rate
of the stamped yaw uplink BB_YAW_ESTIMATE (0x93), its yaw_age_us distribution
and its stamp pairing with BB_AXIS_ESTIMATES. Use it right after a flash
(either board) and before launching ROS.

ROS launch must be DOWN (the bridge is a single-owner UDP link). The client
code comes from the checkout the live bridge was flashed from (see
platformio.ini's upload_command): ~/Desktop/Jugglebot-skills.

    ~/Desktop/PDJ_venv/venv/bin/python scripts/bb_link_check.py [--seconds 4]
"""
import argparse
import collections
import dataclasses
import os
import statistics
import sys
import time

JUGGLEBOT = os.environ.get('JUGGLEBOT_DIR', os.path.expanduser('~/Desktop/Jugglebot-skills'))
sys.path.insert(0, JUGGLEBOT)
sys.path.insert(0, os.path.join(JUGGLEBOT, 'tools'))
import teensy_link_bridge as t  # noqa: E402  (the fw-update tool; brings teensy_link with it)
from config.generated import udp_protocol as u  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--seconds', type=float, default=4.0, help='listening window (default 4 s)')
    ap.add_argument('--teensy-ip', default='192.168.42.2')
    args = ap.parse_args()

    client = t.TeensyLinkClient(teensy_addr=(args.teensy_ip, t.p.PORT_STREAM), bind_host='0.0.0.0')
    client.start()
    client.start_heartbeat(hz=float(t.p.HEARTBEAT_HZ), flags=0)
    counts = collections.Counter()
    yaw, axes, hb, prof = [], [], [], []

    def keep(store, cls):
        def cb(mt, seq, payload, addr):
            counts[cls.__name__] += 1
            if len(store) < 5000:
                store.append(cls.unpack(payload))
        return cb

    unsub = []
    try:
        fw = t._await_bridge_fw_version(client, timeout=3.0)
        print(f'bridge FW_VERSION: {fw if fw is not None else "not heard (link down?)"}')
        unsub = [
            client.subscribe(int(t.MsgType.BB_YAW_ESTIMATE), keep(yaw, u.BbYawEstimate)),
            client.subscribe(int(t.MsgType.BB_AXIS_ESTIMATES), keep(axes, u.BbAxisEstimates)),
            client.subscribe(int(t.MsgType.HEARTBEAT_T2J), keep(hb, u.HeartbeatT2J)),
            client.subscribe(int(t.MsgType.PROFILE), keep(prof, u.Profile)),
        ]
        t0 = time.monotonic()
        time.sleep(args.seconds)
        dt = time.monotonic() - t0
    finally:
        for k in unsub:
            k()
        client.stop()

    rate = {k: round(v / dt, 1) for k, v in sorted(counts.items())}
    print(f'rates over {dt:.1f} s (Hz): {rate}')
    if prof:
        p = prof[-1]
        print(f'BB CAN bus (profile slot 2): rx {p.can2_rx}/s tx {p.can2_tx}/s '
              f'(slot 1 jugglebot rx {p.can1_rx}/s, slot 3 cone rx {p.can3_rx}/s)')
    if hb:
        h = hb[-1]
        print(f'bridge heartbeat: link_state {h.link_state} bus1_health {h.bus1_health} '
              f'bus2_health(BB) {h.bus2_health} fault_state {h.fault_state} uptime {h.uptime_ms / 1e3:.0f} s')
        print(f'BB as seen by the bridge: state {h.bb_state} yaw {h.bb_yaw_deg:.2f} deg '
              f'pitch {h.bb_pitch_deg:.2f} deg hand {h.bb_hand_mm:.1f} mm')
    if axes:
        a = axes[-1]
        print(f'BB_AXIS_ESTIMATES last: pitch {a.pitch_pos_rev:.4f} rev hand {a.hand_pos_rev:.4f} rev')
    if yaw:
        ages = [y.yaw_age_us for y in yaw]
        ys = [y.yaw_deg for y in yaw]
        sy = {y.t_bridge_us for y in yaw}
        sa = {a.t_bridge_us for a in axes}
        print(f'BB_YAW_ESTIMATE: yaw {min(ys):.4f}..{max(ys):.4f} deg, last vel {yaw[-1].yaw_vel_dps:.3f} deg/s, '
              f'bb_frames {yaw[0].bb_frames}->{yaw[-1].bb_frames}')
        print(f'  yaw_age_us min/median/p95/max {min(ages)} / {statistics.median(ages):.0f} / '
              f'{sorted(ages)[int(0.95 * (len(ages) - 1))]} / {max(ages)}')
        print(f'  stamps paired with a BB_AXIS_ESTIMATES stamp: {len(sy & sa)} / {len(sy)}')
    else:
        print('no BB_YAW_ESTIMATE frames: BB dark on CAN, BB FW < 6, or bridge FW < 28')
    return 0


if __name__ == '__main__':
    sys.exit(main())
