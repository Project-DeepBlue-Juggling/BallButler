#!/usr/bin/env python3
"""Predicted (commanded) vs measured (mocap) launch geometry — the assumption-light
test of the "+3 deg steeper / +10% hot launch" question (chapter 1, comment #3).

The 2026-06-17 reading ("+3 deg / hot") came from velocity *extrapolated to a model
release plane*, which is sensitive to the ~44 ms release latency and to release-height
error. This script instead leans on quantities that are INDEPENDENT of clock/latency
and of the release-height anchor:

  * vh  = horizontal launch speed = slope of (x,y) vs t. Constant in vacuum, so a time
          offset cannot change it. vh/cmd < 1  with  |v| ~ commanded (hand encoder)
          => the launch vector is tilted steeper than commanded.
  * apex height (absolute mocap z) = peak of the measured parabola. A late release
          shifts the whole arc in TIME but not in HEIGHT, so a higher-than-commanded
          apex is a real vz, not a timing artifact.

For each throw it fits the clean early-mid arc (track 'ball_butler', window [tt+0.10,
tt+0.60]) with the physical shape imposed (constant horizontal velocity; known-g
parabola in z), compares to the announcement's commanded v0, and plots measured arcs
against the commanded-launch prediction.

Pure-Python (no numpy) so it runs on /usr/bin/python3 (3.8). Plots need matplotlib.

    /usr/bin/python3 plot_launch_geometry.py [BAG.mcap ...]
"""
import mmap, struct, math, json, collections, sys, os

G = 9810.0                      # mm/s^2  (mocap mm)
A_WIN = (0.10, 0.60)            # s after throw_time: clean, measurement-driven arc
N_MIN = 12
GEFF_OK = (9200.0, 10500.0)
RMS_MAX = 8.0                   # mm
DEFAULT_BAGS = [
    "/home/jetson/Desktop/rosbags/2026-06-17_15-13-42/2026-06-17_15-13-42_0.mcap",
    "/home/jetson/Desktop/rosbags/2026-06-17_14-51-24/2026-06-17_14-51-24_0.mcap",
]
OUTDIR = os.path.join(os.path.dirname(__file__), "sessions")

# ----- mcap container reader (adapted from analyze_release_from_bag.py) -----
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
    def __init__(s, data): s.b = data; s.p = 4
    def align(s, n):
        m = (s.p - 4) % n
        if m: s.p += n - m
    def u8(s): v = s.b[s.p]; s.p += 1; return v
    def i32(s): s.align(4); v = struct.unpack_from('<i', s.b, s.p)[0]; s.p += 4; return v
    def u32(s): s.align(4); v = struct.unpack_from('<I', s.b, s.p)[0]; s.p += 4; return v
    def f64(s): s.align(8); v = struct.unpack_from('<d', s.b, s.p)[0]; s.p += 8; return v
    def vec3(s): return (s.f64(), s.f64(), s.f64())
    def time(s): return s.i32(), s.u32()
    def string(s):
        s.align(4); ln = struct.unpack_from('<I', s.b, s.p)[0]; s.p += 4
        out = bytes(s.b[s.p:s.p + ln]); s.p += ln
        return out[:-1].decode('utf-8', 'replace') if ln else ''

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

def read_bag(path):
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
                    balls[bid].append((stamp, pos))
    f = open(path, 'rb'); mm = mmap.mmap(f.fileno(), 0, access=mmap.ACCESS_READ)
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
    return ann, balls

# ----- least squares (pure python) -----
def linfit(T, Y):
    """slope, intercept of Y ~ a*T + b."""
    n = len(T); st = sum(T); sy = sum(Y)
    stt = sum(t * t for t in T); sty = sum(t * y for t, y in zip(T, Y))
    d = n * stt - st * st
    a = (n * sty - st * sy) / d
    b = (sy - a * st) / n
    return a, b

def fit_throw(ann, samples):
    """Fit measured launch state from the clean early-mid arc. Returns dict or None."""
    tt = ann['throw_time_ns'] / 1e9
    pts = [(st / 1e9, p) for st, p in samples
           if tt + A_WIN[0] <= st / 1e9 <= tt + A_WIN[1]]
    if len(pts) < N_MIN:
        return None
    t0 = sorted(t for t, _ in pts)[len(pts) // 2]
    T = [t - t0 for t, _ in pts]
    X = [p[0] for _, p in pts]; Y = [p[1] for _, p in pts]; Z = [p[2] for _, p in pts]
    vx, x0 = linfit(T, X); vy, y0 = linfit(T, Y)
    # known-g parabola: z + 0.5 g T^2 = z0 + vz*T  -> linear regression
    vz, z0 = linfit(T, [z + 0.5 * G * t * t for t, z in zip(T, Z)])
    # quality
    resid = [z - (z0 + vz * t - 0.5 * G * t * t) for t, z in zip(T, Z)]
    rms = math.sqrt(sum(r * r for r in resid) / len(resid))
    # g_eff via quadratic coeff of z(T): z = c2 T^2 + c1 T + c0, g_eff = -2 c2
    c2 = quad_c2(T, Z)
    geff = -2.0 * c2
    if not (GEFF_OK[0] <= geff <= GEFF_OK[1]) or rms > RMS_MAX:
        return None
    vh = math.hypot(vx, vy)
    apex_meas = z0 + vz * vz / (2 * G)           # absolute mocap apex height (mm)
    iv = ann['init_vel']
    cvh = math.hypot(iv[0], iv[1]); cvz = iv[2]; cmag = math.sqrt(sum(v * v for v in iv))
    apex_cmd = ann['init_pos'][2] + cvz * cvz / (2 * G)
    cmd_el = math.degrees(math.atan2(cvz, cvh))
    # measured elevation at t0 (window); vz here is vz at t0, not at release, but the
    # RATIO test uses vh which is anchor/latency free.
    meas_el = math.degrees(math.atan2(vz, vh))
    # launch instant = throw_time + measured release latency (44 ms). Evaluate the
    # measured fit there to get the launch point + launch vz (the only place 44 ms enters).
    REL_LAT = 0.044
    t_L = tt + REL_LAT
    dtl = t_L - t0
    x_L = x0 + vx * dtl; y_L = y0 + vy * dtl
    z_L = z0 + vz * dtl - 0.5 * G * dtl * dtl
    vz_L = vz - G * dtl
    return dict(tt=tt, t0=t0, vx=vx, vy=vy, vz=vz, z0=z0, x0=x0, y0=y0,
                vh=vh, cvh=cvh, cvz=cvz, cmag=cmag, n=len(pts), rms=rms, geff=geff,
                vh_ratio=vh / cvh if cvh else None,
                apex_meas=apex_meas, apex_cmd=apex_cmd, d_apex=apex_meas - apex_cmd,
                cmd_el=cmd_el, meas_el=meas_el, t_L=t_L, x_L=x_L, y_L=y_L, z_L=z_L, vz_L=vz_L,
                d_relheight=z_L - ann['init_pos'][2],
                vz_ratio=vz_L / cvz if cvz else None,
                vmag_ratio=math.hypot(vh, vz_L) / cmag if cmag else None,
                meas_el_launch=math.degrees(math.atan2(vz_L, vh)),
                apex_above_launch_meas=vz_L * vz_L / (2 * G),
                apex_above_launch_cmd=cvz * cvz / (2 * G),
                init_pos=ann['init_pos'], init_vel=iv,
                samples=[(t - t0, p) for t, p in pts])

def quad_c2(T, Z):
    """Quadratic least-squares coeff c2 of Z ~ c2 T^2 + c1 T + c0 (pure python)."""
    n = len(T)
    s0 = n; s1 = sum(T); s2 = sum(t**2 for t in T); s3 = sum(t**3 for t in T); s4 = sum(t**4 for t in T)
    b0 = sum(Z); b1 = sum(t*z for t, z in zip(T, Z)); b2 = sum(t*t*z for t, z in zip(T, Z))
    # solve 3x3 [[s0,s1,s2],[s1,s2,s3],[s2,s3,s4]] [c0,c1,c2] = [b0,b1,b2]
    M = [[s0, s1, s2, b0], [s1, s2, s3, b1], [s2, s3, s4, b2]]
    for i in range(3):
        piv = M[i][i]
        for j in range(i, 4): M[i][j] /= piv
        for k in range(3):
            if k != i:
                f = M[k][i]
                for j in range(i, 4): M[k][j] -= f * M[i][j]
    return M[2][3]

def match(ann, balls):
    """Greedy nearest match: each announcement -> the ball track whose samples best
    bracket throw_time. Returns list of (ann, samples)."""
    tracks = []
    for bid, raw in balls.items():
        s = sorted(raw)
        if len(s) < N_MIN: continue
        tracks.append((s[0][0] / 1e9, s[-1][0] / 1e9, s))
    tracks.sort()
    used = [False] * len(tracks)
    out = []
    for a in sorted(ann, key=lambda x: x['throw_time_ns']):
        tt = a['throw_time_ns'] / 1e9
        best = None; bj = -1
        for j, (t_lo, t_hi, s) in enumerate(tracks):
            if used[j]: continue
            if t_lo <= tt + 0.30 and t_hi >= tt + 0.20:   # track spans the early arc
                score = sum(1 for st, _ in s if tt + A_WIN[0] <= st / 1e9 <= tt + A_WIN[1])
                if best is None or score > best:
                    best = score; bj = j
        if bj >= 0:
            used[bj] = True; out.append((a, tracks[bj][2]))
    return out

def stats(vals):
    v = [x for x in vals if x is not None and math.isfinite(x)]
    if not v: return None
    m = sum(v) / len(v)
    sd = math.sqrt(sum((x - m) ** 2 for x in v) / (len(v) - 1)) if len(v) > 1 else 0.0
    return dict(n=len(v), mean=m, std=sd, med=sorted(v)[len(v) // 2],
                lo=min(v), hi=max(v))

def analyze(bag):
    ann, balls = read_bag(bag)
    pairs = match(ann, balls)
    fits = [f for f in (fit_throw(a, s) for a, s in pairs) if f]
    return ann, fits

def report(name, ann, fits):
    print("\n############ %s ############" % name)
    print("announcements: %d   clean fitted throws: %d" % (len(ann), len(fits)))
    if not fits: return
    for key, lbl in [('vh_ratio', 'vh  meas/cmd (LATENCY-FREE)'),
                     ('vz_ratio', 'vz  meas/cmd (apex, @44ms)'),
                     ('vmag_ratio', '|v| meas/cmd (speed)'),
                     ('meas_el_launch', 'measured elevation@launch'),
                     ('cmd_el', 'commanded elevation (deg)'),
                     ('apex_above_launch_meas', 'apex above launch, MEAS (mm)'),
                     ('apex_above_launch_cmd', 'apex above launch, CMD (mm)'),
                     ('d_relheight', 'launch height - init_pos.z (mm)'),
                     ('geff', 'g_eff (mm/s^2)'), ('rms', 'fit RMS (mm)')]:
        s = stats([f[key] for f in fits])
        if s:
            print("  %-28s mean=%+9.3f  sd=%6.3f  med=%+9.3f  [%+.2f,%+.2f]"
                  % (lbl, s['mean'], s['std'], s['med'], s['lo'], s['hi']))
    sv = stats([f['vmag_ratio'] for f in fits]); se = stats([f['meas_el_launch'] for f in fits])
    if sv and se:
        print("  => measured launch: %.0f%% of commanded SPEED at %.1f deg (cmd 71.0) "
              "-> uniform speed deficit, not steeper/hot" % (sv['mean'] * 100, se['mean']))

def make_plots(allfits, out_png):
    import matplotlib; matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    flat = [f for _, _, fits in allfits for f in fits]
    # hero = throw with vh_ratio closest to the global median and good coverage
    vhs = sorted(f['vh_ratio'] for f in flat)
    vh_med = vhs[len(vhs) // 2]
    hero = min((f for f in flat if f['n'] >= 25),
               key=lambda f: (abs(f['vh_ratio'] - vh_med), -f['n']))

    fig, (axL, axR) = plt.subplots(1, 2, figsize=(13.5, 5.6),
                                   gridspec_kw=dict(width_ratios=[1.25, 1]))

    # ---- LEFT: throw-plane path overlay for the hero throw ----
    ux = hero['vx'] / hero['vh']; uy = hero['vy'] / hero['vh']
    hx, hz = [], []
    for T, p in hero['samples']:
        hx.append((p[0] - hero['x_L']) * ux + (p[1] - hero['y_L']) * uy)
        hz.append(p[2] - hero['z_L'])
    tof = max(0.9, hero['init_vel'][2] / G * 2 + 0.2)
    ts = [i * tof / 200 for i in range(201)]
    # measured-fit parabola from launch
    mh = [hero['vh'] * t for t in ts]
    mz = [hero['vz_L'] * t - 0.5 * G * t * t for t in ts]
    # commanded-launch parabola from the same launch point/time
    ch = [hero['cvh'] * t for t in ts]
    cz = [hero['cvz'] * t - 0.5 * G * t * t for t in ts]

    axL.scatter(hx, hz, s=20, color='#1f6fb4', zorder=5, label='measured mocap ball')
    axL.plot(mh, mz, '-', color='#0f766e', lw=2.2, label='measured launch (fit)')
    axL.plot(ch, cz, '--', color='#b45309', lw=2.2, label='commanded launch (model v0)')
    # apex markers
    am = hero['apex_above_launch_meas']; ac = hero['apex_above_launch_cmd']
    axL.axhline(am, color='#0f766e', lw=0.8, ls=':'); axL.axhline(ac, color='#b45309', lw=0.8, ls=':')
    axL.annotate('measured apex +%.0f mm' % am, (max(ch) * 0.02, am), color='#0f766e',
                 fontsize=9, va='bottom')
    axL.annotate('commanded apex +%.0f mm' % ac, (max(ch) * 0.40, ac), color='#b45309',
                 fontsize=9, va='top')
    axL.set_xlabel('horizontal distance from launch (mm)')
    axL.set_ylabel('height above launch (mm)')
    axL.set_title('Measured path vs commanded-launch prediction\n'
                  'same angle (%.1f° vs %.1f°), but ~%.0f%% slower → falls short & lower'
                  % (hero['meas_el_launch'], hero['cmd_el'],
                     (1 - hero['vmag_ratio']) * 100), fontsize=11)
    axL.legend(fontsize=9, loc='upper right'); axL.grid(alpha=0.3)
    axL.set_ylim(bottom=min(min(hz), -30))

    # ---- RIGHT: vh AND vz ratios across all throws (both ~0.93 = uniform deficit) ----
    groups = [('vh_ratio', 'vh (horizontal, latency-free)', '#1f6fb4'),
              ('vz_ratio', 'vz (from apex height)', '#0f766e')]
    xticks = []; xlabels = []
    for gi, (key, lbl, col) in enumerate(groups):
        allr = []
        for b, _, fits in allfits:
            allr += [f[key] for f in fits if f[key] is not None]
        xs = [gi + 0.5 * (i / max(1, len(allr) - 1)) - 0.25 for i in range(len(allr))]
        axR.scatter(xs, allr, s=20, color=col, alpha=0.75, label=lbl)
        m = sum(allr) / len(allr)
        axR.plot([gi - 0.32, gi + 0.32], [m, m], color='k', lw=2.2)
        axR.annotate('%.3f' % m, (gi, m), fontsize=10, ha='center', va='bottom', fontweight='bold')
        xticks.append(gi); xlabels.append(lbl.split(' ')[0])
    axR.axhline(1.0, color='#b45309', lw=1.6, ls='--')
    axR.annotate('commanded = 1.00\n("hot" would be > 1)', (1.0, 1.0), xytext=(0.55, 1.02),
                 fontsize=9, color='#b45309')
    axR.set_xticks(xticks); axR.set_xticklabels(xlabels, fontsize=10)
    axR.set_xlim(-0.5, 1.5); axR.set_ylim(0.85, 1.08)
    axR.set_ylabel('measured / commanded')
    axR.set_title('Both components ≈ 0.93× commanded (n=%d throws, 2 sessions)\n'
                  'uniform speed deficit at the commanded angle — NOT steeper, NOT hot'
                  % len(flat), fontsize=11)
    axR.legend(fontsize=8.5, loc='lower right'); axR.grid(alpha=0.3, axis='y')

    fig.tight_layout()
    fig.savefig(out_png, dpi=125)
    print("\nSaved plot ->", out_png)
    print("hero throw: vh_ratio=%.3f n=%d rms=%.1fmm  meas_apex=+%.0f cmd_apex=+%.0f mm above launch"
          % (hero['vh_ratio'], hero['n'], hero['rms'],
             hero['apex_above_launch_meas'], hero['apex_above_launch_cmd']))

def main():
    args = [a for a in sys.argv[1:] if not a.startswith('--')]
    do_plot = '--plot' in sys.argv
    out = next((a.split('=', 1)[1] for a in sys.argv if a.startswith('--out=')),
               os.path.join(os.path.dirname(__file__), '..', '..', '..',
                            'logbook', 'chapters', 'images', 'hot-launch-test.png'))
    bags = args if args else DEFAULT_BAGS
    allfits = []
    for b in bags:
        if not os.path.exists(b):
            print("MISSING", b); continue
        print("Reading %s ..." % os.path.basename(b), flush=True)
        ann, fits = analyze(b)
        report(os.path.basename(b), ann, fits)
        allfits.append((b, ann, fits))
    if do_plot and allfits:
        make_plots(allfits, os.path.abspath(out))
    return allfits

if __name__ == "__main__":
    main()
