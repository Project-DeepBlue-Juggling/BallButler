#!/usr/bin/env python3
"""Tier-0 decisive cross-check. Per Ball-Butler throw, decompose the cone
arrival error using the mocap-tracked ball as an independent clock+physics
reference:

    cone_ae = [cone_catch - mocap_cross] + [mocap_cross - predicted_landing]
            =        A (clock offset)     +        B (physical arrival error)

mocap_cross = time the CONFIRMED mocap ball crosses the cone catch plane
(descending). A ramps -> cone(Teensy) vs QTM clock drift (measurement artifact).
B ramps -> the ball really arrives later than predicted (physical throw drift).
Also reports actual/commanded launch speed (kinematic). Pure-Python (no numpy,
no rosbag2/mcap libs — reads the mcap container directly).

    /usr/bin/python3 analyze_release_from_bag.py [BAG.mcap] [SESSION.json]
"""
import mmap, struct, math, json, collections, sys

BAG = (sys.argv[1] if len(sys.argv) > 1 else
       '/home/jetson/Desktop/rosbags/2026-06-12_11-03-43/2026-06-12_11-03-43_0.mcap')
SESS = (sys.argv[2] if len(sys.argv) > 2 else
        'sessions/temporal_distance-sweep_2026-06-12_11-07-44.json')
G = 9810.0
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

class CDR:
    __slots__ = ('b', 'p')
    def __init__(self, data): self.b = data; self.p = 4
    def align(self, n):
        m = (self.p - 4) % n
        if m: self.p += n - m
    def u8(self): v = self.b[self.p]; self.p += 1; return v
    def i32(self): self.align(4); v = struct.unpack_from('<i', self.b, self.p)[0]; self.p += 4; return v
    def u32(self): self.align(4); v = struct.unpack_from('<I', self.b, self.p)[0]; self.p += 4; return v
    def f64(self): self.align(8); v = struct.unpack_from('<d', self.b, self.p)[0]; self.p += 8; return v
    def vec3(self): return (self.f64(), self.f64(), self.f64())
    def time(self): return self.i32(), self.u32()
    def string(self):
        self.align(4); ln = struct.unpack_from('<I', self.b, self.p)[0]; self.p += 4
        s = bytes(self.b[self.p:self.p + ln]); self.p += ln
        return s[:-1].decode('utf-8', 'replace') if ln else ''

tns = lambda t: t[0] * 1_000_000_000 + t[1]

def dec_announcement(data):
    c = CDR(data); c.time(); c.string(); c.string()
    ip = c.vec3(); iv = c.vec3(); c.string(); tt = c.time(); tof = c.f64()
    lp = c.vec3(); c.vec3(); lt = c.time()
    return dict(init_pos=ip, init_vel=iv, throw_time_ns=tns(tt), tof=tof,
                land_pos=lp, land_time_ns=tns(lt))

def dec_ball(c):
    st = c.time(); c.string(); bid = c.u32(); status = c.u8(); tracking = c.u8()
    src = c.string(); c.string(); pos = c.vec3(); vel = c.vec3(); c.vec3(); c.vec3(); c.time()
    return (tns(st), bid, status, tracking, src, pos, vel)

chan = {}; ann = []; balls = collections.defaultdict(list)
WANT = ('/balls', '/throw_announcements')
def handle_channel(c): chan[u16(c, 0)] = bytes(c[8:8 + u32(c, 4)]).decode('utf-8', 'replace')
def handle_message(c):
    topic = chan.get(u16(c, 0))
    if topic not in WANT: return
    data = c[22:]
    if topic == '/throw_announcements':
        ann.append(dec_announcement(data))
    else:
        cd = CDR(data); n = cd.u32()
        for _ in range(n):
            stamp, bid, status, tracking, src, pos, vel = dec_ball(cd)
            if src == 'ball_butler' and status == 1 and tracking == 1:
                balls[bid].append((stamp, pos, vel))

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

sess = json.load(open(SESS))
js = [(th['announcement']['throw_time_ns'], th['catch']['catch_time_ns'],
       th['announcement']['landing_time_ns'], th['arrival_error_ms'])
      for th in sess['throws']
      if th.get('status') == 'ok' and th.get('announcement') and th.get('catch')
      and th['catch'].get('catch_time_ns')]

ann_s = sorted(ann, key=lambda a: a['throw_time_ns'])
ball_s = sorted(balls.items(), key=lambda kv: sorted(kv[1])[0][0])
rows = []
for a, (bid, raw) in zip(ann_s, ball_s):
    s = sorted(raw)
    if len(s) < 10: continue
    plane = a['land_pos'][2]
    i_apex = max(range(len(s)), key=lambda i: s[i][1][2])
    tc = None
    for i in range(i_apex, len(s) - 1):
        z0 = s[i][1][2]; z1 = s[i + 1][1][2]
        if z0 >= plane >= z1 and z0 != z1:
            fr = (z0 - plane) / (z0 - z1)
            tc = s[i][0] + fr * (s[i + 1][0] - s[i][0]); break
    if tc is None: continue
    jm = min(js, key=lambda j: abs(j[0] - a['throw_time_ns'])) if js else None
    catch, cone_ae = (jm[1], jm[3]) if (jm and abs(jm[0] - a['throw_time_ns']) < 5_000_000) else (None, None)
    B = (tc - a['land_time_ns']) / 1e6
    A = (catch - tc) / 1e6 if catch else None
    # launch velocity from the CLEAN ascending arc (away from the catch)
    early = [x for x in s if s[0][0] + 30_000_000 <= x[0] <= s[0][0] + 300_000_000 and x[0] < s[i_apex][0]] or s[:8]
    vx = sorted(x[2][0] for x in early)[len(early) // 2]; vy = sorted(x[2][1] for x in early)[len(early) // 2]
    vh = math.hypot(vx, vy)
    vzL = math.sqrt(2 * G * max(s[i_apex][1][2] - a['init_pos'][2], 1))
    vmag = math.sqrt(vh * vh + vzL * vzL)
    cv = a['init_vel']; cvh = math.hypot(cv[0], cv[1]); cvm = math.sqrt(sum(x * x for x in cv))
    rows.append(dict(tt=a['throw_time_ns'], A=A, B=B, cone_ae=cone_ae,
                     vratio=vmag / cvm, vhratio=vh / cvh if cvh else None))

rows.sort(key=lambda d: d['tt'])
t0 = rows[0]['tt']
for d in rows: d['trel'] = (d['tt'] - t0) / 1e9
tr = [d['trel'] for d in rows]

def pearson(key):
    pairs = [(d['trel'], d[key]) for d in rows if d[key] is not None]
    n = len(pairs)
    if n < 4: return float('nan')
    xs = [p[0] for p in pairs]; ys = [p[1] for p in pairs]
    mx = sum(xs) / n; my = sum(ys) / n
    sxy = sum((x - mx) * (y - my) for x, y in zip(xs, ys))
    sxx = sum((x - mx) ** 2 for x in xs); syy = sum((y - my) ** 2 for y in ys)
    return sxy / math.sqrt(sxx * syy) if sxx and syy else float('nan')

def thirds(key):
    sp = max(tr); g = [[], [], []]
    for d in rows:
        if d[key] is None: continue
        g[0 if d['trel'] < sp / 3 else (1 if d['trel'] < 2 * sp / 3 else 2)].append(d[key])
    return [sum(x) / len(x) if x else float('nan') for x in g]

print("matched throws with mocap crossing: %d" % len(rows))
print("\nPearson vs session time:")
print("  A = cone_catch - mocap_cross  (CLOCK offset)   r=%+.3f" % pearson('A'))
print("  B = mocap_cross - predicted   (PHYSICAL)       r=%+.3f" % pearson('B'))
print("  cone arrival error  (= A+B)                    r=%+.3f" % pearson('cone_ae'))
print("  actual/commanded |v| (kinematic)               r=%+.3f" % pearson('vratio'))
print("  actual/commanded HORIZ speed (cleanest)        r=%+.3f" % pearson('vhratio'))
print("\n                                  early    mid     late")
print("A  cone_catch - mocap_cross ms : %7.1f %7.1f %7.1f" % tuple(thirds('A')))
print("B  mocap_cross - predicted  ms : %7.1f %7.1f %7.1f" % tuple(thirds('B')))
print("   cone arrival error       ms : %7.1f %7.1f %7.1f" % tuple(thirds('cone_ae')))
print("   actual/commanded |v|        : %7.3f %7.3f %7.3f" % tuple(thirds('vratio')))
print("   actual/commanded HORIZ vh   : %7.3f %7.3f %7.3f" % tuple(thirds('vhratio')))

try:
    import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
    fig, ax = plt.subplots(2, 1, figsize=(11, 8), sharex=True)
    def xy(key): return [(d['trel'], d[key]) for d in rows if d[key] is not None]
    for key, col, lab in (('cone_ae', 'tab:red', 'cone arrival error (A+B)'),
                          ('B', 'tab:blue', 'B: mocap_cross - predicted (PHYSICAL)'),
                          ('A', 'tab:orange', 'A: cone_catch - mocap_cross (CLOCK)')):
        pts = xy(key); ax[0].scatter([p[0] for p in pts], [p[1] for p in pts], s=26, color=col, label=lab)
    ax[0].axhline(0, color='k', lw=0.5); ax[0].set_ylabel('ms'); ax[0].legend(fontsize=8); ax[0].grid(alpha=0.3)
    ax[0].set_title('Tier 0 decisive: does the warm-up ramp live in B (physical) or A (cone-vs-QTM clock)?')
    ax[1].scatter(tr, [d['vratio'] for d in rows], s=24, color='tab:green'); ax[1].axhline(1, color='k', lw=0.5)
    ax[1].set_ylabel('actual/commanded |v|'); ax[1].set_xlabel('session time (s)'); ax[1].grid(alpha=0.3)
    fig.tight_layout(); out = SESS.replace('.json', '_TIER0_decompose.png')
    fig.savefig(out, dpi=120); print("\nSaved plot:", out)
except Exception as e:
    print("plot skipped:", repr(e))
