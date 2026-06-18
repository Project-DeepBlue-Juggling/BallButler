#!/usr/bin/env python3
"""Warm-up mechanism analysis: arrival_error vs hand-motor temp vs time across
Block A (cold curve) + cooldown + Block C (re-ramp). Pulls /bb/odrive_diag
(hand temps) from the bag and arrival_error from the two session JSONs."""
import mmap, struct, json, math, glob

BAGDIR = '/home/jetson/Desktop/rosbags/2026-06-12_13-47-58'
SESS_A = ('/home/jetson/Desktop/BallButler/zTesting/throw_testing/temporal_accuracy/'
          'sessions/temporal_repeatability_warmupA-cold_2026-06-12_13-48-56.json')
SESS_C = ('/home/jetson/Desktop/BallButler/zTesting/throw_testing/temporal_accuracy/'
          'sessions/temporal_repeatability_warmupC-reramp_2026-06-12_14-19-55.json')
BAG = sorted(glob.glob(BAGDIR + '/*.mcap'))[0]

u16 = lambda b, o: struct.unpack_from('<H', b, o)[0]
u32 = lambda b, o: struct.unpack_from('<I', b, o)[0]
u64 = lambda b, o: struct.unpack_from('<Q', b, o)[0]

def iter_records(buf):
    p, L = 0, len(buf)
    while p + 9 <= L:
        op = buf[p]; ln = u64(buf, p + 1); s = p + 9
        if s + ln > L: break
        yield op, buf[s:s + ln]; p = s + ln

def inflate(comp, recs, usize):
    if comp == b'': return recs
    if comp == b'zstd':
        import zstandard; return zstandard.ZstdDecompressor().decompress(bytes(recs), max_output_size=usize)
    if comp == b'lz4':
        import lz4.frame; return lz4.frame.decompress(bytes(recs))
    raise RuntimeError('comp %r' % comp)

chan = {}
diag = []   # (t_ns, pitch_fet, pitch_motor, pitch_iq, pitch_state, hand_fet, hand_motor, hand_iq, hand_state)
def handle_channel(c): chan[u16(c, 0)] = bytes(c[8:8 + u32(c, 4)]).decode('utf-8', 'replace')
def handle_message(c):
    if chan.get(u16(c, 0)) != '/bb/odrive_diag': return
    log_time = u64(c, 6); body = c[22:][4:]            # skip Message hdr + CDR encap
    # Float32MultiArray: layout{dim[](u32 len=0), data_offset u32}, then data float32[](u32 len, floats)
    n = u32(body, 8)                                    # data array length (after dim_len@0, data_offset@4)
    vals = [struct.unpack_from('<f', body, 12 + 4 * i)[0] for i in range(n)]
    if len(vals) >= 8:
        diag.append((log_time,) + tuple(vals[:8]))

f = open(BAG, 'rb'); mm = mmap.mmap(f.fileno(), 0, access=mmap.ACCESS_READ)
assert mm[:8] == b'\x89MCAP0\r\n'
for op, c in iter_records(memoryview(mm)[8:]):
    if op == 4: handle_channel(c)
    elif op == 6:
        clen = u32(c, 28); comp = bytes(c[32:32 + clen]); q = 32 + clen
        rsize = u64(c, q); q += 8
        for iop, ic in iter_records(memoryview(inflate(comp, c[q:q + rsize], u64(c, 16)))):
            if iop == 4: handle_channel(ic)
            elif iop == 5: handle_message(ic)
    elif op == 5: handle_message(c)

diag.sort()
print("bag: %s" % BAG.split('/')[-1])
print("/bb/odrive_diag samples: %d" % len(diag))
if not diag:
    print("NO /bb/odrive_diag in bag!"); raise SystemExit

t0 = diag[0][0]
hand_motor = [(d[0], d[6]) for d in diag]            # (t_ns, hand_motor_temp)
hand_fet   = [(d[0], d[5]) for d in diag]
hand_iq    = [(d[0], d[7]) for d in diag]
hm = [x[1] for x in hand_motor]
hf = [x[1] for x in hand_fet]
print("hand_motor temp: start %.1f  min %.1f  max %.1f  end %.1f degC"
      % (hm[0], min(hm), max(hm), hm[-1]))
print("hand_fet   temp: start %.1f  min %.1f  max %.1f  end %.1f degC"
      % (hf[0], min(hf), max(hf), hf[-1]))

def interp(series, t):
    if t <= series[0][0]: return series[0][1]
    if t >= series[-1][0]: return series[-1][1]
    lo, hi = 0, len(series) - 1
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if series[mid][0] <= t: lo = mid
        else: hi = mid
    t0_, v0 = series[lo]; t1_, v1 = series[hi]
    return v0 + (v1 - v0) * (t - t0_) / (t1_ - t0_) if t1_ != t0_ else v0

throws = []
for path, phase in ((SESS_A, 'A'), (SESS_C, 'C')):
    s = json.load(open(path))
    for th in s['throws']:
        if th.get('status') != 'ok': continue
        w = th.get('service_call_wall_ns')
        if not w: continue
        throws.append(dict(t=w, ae=th['arrival_error_ms'], phase=phase,
                           hm=interp(hand_motor, w), hf=interp(hand_fet, w),
                           hi=interp(hand_iq, w)))
throws.sort(key=lambda d: d['t'])
tA = throws[0]['t']
for d in throws: d['trel'] = (d['t'] - tA) / 60e9      # minutes since first throw

def pearson(xs, ys):
    n = len(xs); mx = sum(xs) / n; my = sum(ys) / n
    sxy = sum((x - mx) * (y - my) for x, y in zip(xs, ys))
    sxx = sum((x - mx) ** 2 for x in xs); syy = sum((y - my) ** 2 for y in ys)
    return sxy / math.sqrt(sxx * syy) if sxx and syy else float('nan')

A = [d for d in throws if d['phase'] == 'A']
C = [d for d in throws if d['phase'] == 'C']
print("\nBlock A: n=%d  AE %.1f→%.1f ms  hand_motor %.1f→%.1f degC  (%.1f min)"
      % (len(A), A[0]['ae'], A[-1]['ae'], A[0]['hm'], A[-1]['hm'], A[-1]['trel'] - A[0]['trel']))
gap = C[0]['trel'] - A[-1]['trel']
print("Cooldown gap: %.1f min  (hand_motor %.1f → %.1f degC during gap)"
      % (gap, A[-1]['hm'], C[0]['hm']))
print("Block C: n=%d  AE %.1f→%.1f ms  hand_motor %.1f→%.1f degC"
      % (len(C), C[0]['ae'], C[-1]['ae'], C[0]['hm'], C[-1]['hm']))

allae = [d['ae'] for d in throws]
print("\nPearson (all throws):")
print("  arrival_error vs time          r=%+.3f" % pearson([d['trel'] for d in throws], allae))
print("  arrival_error vs hand_motor T  r=%+.3f" % pearson([d['hm'] for d in throws], allae))
print("  arrival_error vs hand_fet T    r=%+.3f" % pearson([d['hf'] for d in throws], allae))
print("  arrival_error vs hand_iq       r=%+.3f" % pearson([d['hi'] for d in throws], allae))
# within-A correlations
print("\nBlock A only: AE vs time r=%+.3f | AE vs hand_motor r=%+.3f"
      % (pearson([d['trel'] for d in A], [d['ae'] for d in A]),
         pearson([d['hm'] for d in A], [d['ae'] for d in A])))

# plot
try:
    import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
    fig, ax = plt.subplots(2, 1, figsize=(11, 9))
    # Panel 1: AE + hand temp vs time
    for ph, col in (('A', 'tab:blue'), ('C', 'tab:red')):
        d = [x for x in throws if x['phase'] == ph]
        ax[0].scatter([x['trel'] for x in d], [x['ae'] for x in d], s=22, color=col, label='AE block ' + ph)
    ax[0].set_ylabel('arrival error (ms)'); ax[0].legend(loc='upper left', fontsize=8); ax[0].grid(alpha=0.3)
    axt = ax[0].twinx()
    tt = [(t - tA) / 60e9 for t, _ in hand_motor]
    axt.plot(tt, hm, color='tab:green', lw=1, alpha=0.7, label='hand_motor T')
    axt.plot([(t - tA) / 60e9 for t, _ in hand_fet], hf, color='tab:orange', lw=1, alpha=0.5, label='hand_fet T')
    axt.set_ylabel('hand temp (degC)'); axt.legend(loc='lower right', fontsize=8)
    ax[0].set_xlabel('minutes since first throw'); ax[0].set_title('Warm-up: arrival error & hand temp vs time')
    # Panel 2: collapse test — AE vs hand_motor temp
    for ph, col, m in (('A', 'tab:blue', 'o'), ('C', 'tab:red', 'x')):
        d = [x for x in throws if x['phase'] == ph]
        ax[1].scatter([x['hm'] for x in d], [x['ae'] for x in d], s=26, color=col, marker=m, label='block ' + ph)
    ax[1].set_xlabel('hand_motor temp (degC)'); ax[1].set_ylabel('arrival error (ms)')
    ax[1].set_title('Collapse test: does arrival error track hand-motor temperature?')
    ax[1].legend(fontsize=8); ax[1].grid(alpha=0.3)
    fig.tight_layout()
    out = SESS_A.replace('.json', '_WARMUP_TEMP.png')
    fig.savefig(out, dpi=120); print("\nSaved plot:", out)
except Exception as e:
    print("plot skipped:", repr(e))
