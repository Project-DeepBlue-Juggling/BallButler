#!/usr/bin/env python3
"""Pull cone time-sync + QTM clock-offset traces from a rosbag2 *mcap* bag.

Standalone pure-Python MCAP reader — no rosbag2_py / mcap libs required (neither
is installed in Foxy on the Jetson). Handles uncompressed and zstd/lz4 chunks;
decodes only the two small fixed-layout CDR messages we need:

  /cone/heartbeat        (CatchingConeHeartbeat) — state, sync_rms_us, time_synced
  /qtm_clock_offset_sec  (std_msgs/Float64)

Used to settle whether the 2026-06-12 temporal warm-up drift was a real
thrower-side effect or a cone-clock settling artifact (it was real — the cone
was READY + time_synced from t=0; see logbook/2026-06-12-temporal-warmup-drift).

    /usr/bin/python3 read_bag_sync_traces.py [BAG.mcap] [SESSION.json]

Python 3.8 compatible.
"""
import mmap, struct, json, statistics, sys

BAG = (sys.argv[1] if len(sys.argv) > 1 else
       '/home/jetson/Desktop/rosbags/2026-06-12_11-03-43/2026-06-12_11-03-43_0.mcap')
SESS = (sys.argv[2] if len(sys.argv) > 2 else
        'sessions/temporal_distance-sweep_2026-06-12_11-07-44.json')

u16 = lambda b, o: struct.unpack_from('<H', b, o)[0]
u32 = lambda b, o: struct.unpack_from('<I', b, o)[0]
u64 = lambda b, o: struct.unpack_from('<Q', b, o)[0]


def iter_records(buf):
    p, L = 0, len(buf)
    while p + 9 <= L:
        op = buf[p]; ln = u64(buf, p + 1); s = p + 9
        if s + ln > L:
            break
        yield op, buf[s:s + ln]
        p = s + ln


def inflate(comp, recs, usize):
    if comp == b'':
        return recs
    if comp == b'zstd':
        import zstandard
        return zstandard.ZstdDecompressor().decompress(bytes(recs), max_output_size=usize)
    if comp == b'lz4':
        import lz4.frame
        return lz4.frame.decompress(bytes(recs))
    raise RuntimeError('unknown compression %r' % comp)


chan = {}
cone = []   # (t_ns, state, sync_rms_us, time_synced)
qtm = []    # (t_ns, offset_s)
WANT = ('/cone/heartbeat', '/qtm_clock_offset_sec')


def handle_channel(c):
    chan[u16(c, 0)] = bytes(c[8:8 + u32(c, 4)]).decode('utf-8', 'replace')


def handle_message(c):
    topic = chan.get(u16(c, 0))
    if topic not in WANT:
        return
    body = c[22:][4:]                          # skip Message header + 4-byte CDR encap
    if topic == '/cone/heartbeat':
        cone.append((u64(c, 6), body[1], body[3], body[12]))
    else:
        qtm.append((u64(c, 6), struct.unpack_from('<d', body, 0)[0]))


def main():
    f = open(BAG, 'rb'); mm = mmap.mmap(f.fileno(), 0, access=mmap.ACCESS_READ)
    assert mm[:8] == b'\x89MCAP0\r\n', 'bad MCAP magic'
    for op, c in iter_records(memoryview(mm)[8:]):
        if op == 4:
            handle_channel(c)
        elif op == 6:                          # Chunk
            clen = u32(c, 28); comp = bytes(c[32:32 + clen]); q = 32 + clen
            rsize = u64(c, q); q += 8
            raw = inflate(comp, c[q:q + rsize], u64(c, 16))
            for iop, ic in iter_records(memoryview(raw)):
                if iop == 4:
                    handle_channel(ic)
                elif iop == 5:
                    handle_message(ic)
        elif op == 5:
            handle_message(c)

    cone.sort(); qtm.sort()
    t0 = cone[0][0] if cone else qtm[0][0]
    rel = lambda ns: (ns - t0) / 1e9
    ST = {0: 'BOOT', 1: 'UNSYNCED', 2: 'READY', 127: 'ERROR'}

    print('cone hb: %d | qtm: %d' % (len(cone), len(qtm)))
    print('\n=== first 8 cone heartbeats ===')
    for ns, st, rms, syn in cone[:8]:
        print('  %5.1fs  %-8s  sync_rms=%3dus  synced=%s'
              % (rel(ns), ST.get(st, st), rms, bool(syn)))

    print('\n=== cone sync per 120 s bin ===')
    bins = {}
    for ns, st, rms, syn in cone:
        bins.setdefault(int(rel(ns) // 120), []).append((st, rms, syn))
    for b in sorted(bins):
        rows = bins[b]
        sts = ','.join(sorted(set(ST.get(r[0], r[0]) for r in rows)))
        rmss = [r[1] for r in rows]
        print('  %4d-%-4ds  n=%-4d  %-8s  sync_rms mean/max %4.1f/%-3d  synced %.0f%%'
              % (b * 120, (b + 1) * 120, len(rows), sts,
                 statistics.mean(rmss), max(rmss),
                 100.0 * sum(r[2] for r in rows) / len(rows)))

    allr = [r[2] for r in cone]
    print('\nwhole session sync_rms_us: median %d  mean %.2f  p95 %d  max %d'
          % (statistics.median(allr), statistics.mean(allr),
             sorted(allr)[int(0.95 * len(allr))], max(allr)))
    print('states seen: %s | always time_synced: %s'
          % (sorted(set(ST.get(r[1], r[1]) for r in cone)), all(r[3] for r in cone)))
    if qtm:
        qs = [o for _, o in qtm]
        print('qtm_clock_offset drift: %.3f ms over session (min/max span %.3f ms)'
              % ((qtm[-1][1] - qtm[0][1]) * 1e3, (max(qs) - min(qs)) * 1e3))

    # 3-panel: arrival error / cone sync_rms / qtm offset on a shared time axis
    try:
        s = json.load(open(SESS))
        ok = [t for t in s['throws'] if t.get('status') == 'ok']
        import matplotlib; matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(3, 1, figsize=(11, 10), sharex=True)
        ax[0].scatter([(t['service_call_wall_ns'] - t0) / 1e9 for t in ok],
                      [t['arrival_error_ms'] for t in ok], s=28, color='tab:blue')
        ax[0].set_ylabel('arrival error (ms)'); ax[0].grid(alpha=0.3)
        ax[0].set_title('Cold-start warm-up arc: arrival error vs cone time-sync vs QTM offset')
        ax[1].plot([rel(r[0]) for r in cone], [r[2] for r in cone], lw=0.6, color='tab:orange')
        ax[1].set_ylabel('cone sync_rms_us'); ax[1].grid(alpha=0.3)
        if qtm:
            ax[2].plot([rel(r[0]) for r in qtm], [r[1] * 1e3 for r in qtm], lw=0.8, color='tab:green')
        ax[2].set_ylabel('qtm offset (ms)'); ax[2].set_xlabel('time since bag start (s)')
        ax[2].grid(alpha=0.3)
        fig.tight_layout()
        out = SESS.replace('.json', '_SYNC_TRACES.png')
        fig.savefig(out, dpi=120)
        print('\nSaved plot:', out)
    except FileNotFoundError:
        print('\n(session JSON not found at %s — skipped plot)' % SESS)


if __name__ == '__main__':
    main()
