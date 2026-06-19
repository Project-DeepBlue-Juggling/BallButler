#!/usr/bin/env python3
"""Where does the ~8% launch-speed deficit come from? Localise it along the chain
   commanded ball speed  C  ->  hand peak linear speed  H  ->  measured ball speed  B
on a SINGLE bag (needs /throw_announcements + /bb/axis_estimates + /balls), so all
three are on the same throws.

  H/C  (hand peak / commanded)  -- from the 1 kHz hand encoder (rev/s x 2*pi*R).
       This is the ODrive tracking its OWN velocity command; H/C<1 = servo under-track.
  B/C  (ball / commanded)       -- from the mocap launch fit (apex + horizontal slope).
  B/H  (ball / hand peak)       -- the hand->ball gap: the ball leaves below the hand's
       peak linear speed (release a touch after the velocity peak, and/or an
       uncalibrated effective spool radius). Not separable here; both are
       launch-side and uniformly correctable.

Pure-Python (no numpy); reuses the decoder + launch fit from plot_launch_geometry.

    /usr/bin/python3 plot_speed_chain.py [BAG.mcap]   (default: 2026-06-18_16-36-14)
"""
import mmap, math, collections, sys, os
import plot_launch_geometry as P

R = 0.0052493                      # HAND_SPOOL_RADIUS_M (hardware_config.h BBTraj)
TWO_PI_R = 2.0 * math.pi * R       # m per hand-rev
STROKE_WIN = (-0.05, 0.35)         # s around throw_time
DEFAULT_BAG = "/home/jetson/Desktop/rosbags/2026-06-18_16-36-14/2026-06-18_16-36-14_0.mcap"
OUT = os.path.join(os.path.dirname(__file__), "..", "..", "..",
                   "logbook", "chapters", "images", "speed-chain.png")


def dec_jointstate(data):
    """sensor_msgs/JointState -> (stamp_ns, {name: (pos,vel)})."""
    c = P.CDR(data)
    st = c.time(); c.string()                      # header.stamp, frame_id
    n = c.u32(); names = [c.string() for _ in range(n)]
    npos = c.u32(); pos = [c.f64() for _ in range(npos)]
    nvel = c.u32(); vel = [c.f64() for _ in range(nvel)]
    d = {}
    for i, nm in enumerate(names):
        d[nm] = (pos[i] if i < len(pos) else None, vel[i] if i < len(vel) else None)
    return P.tns(st), d


def read_bag_full(path):
    chan = {}; ann = []; balls = collections.defaultdict(list); hand = []  # (t_ns, vel_revps)
    WANT = ('/balls', '/throw_announcements', '/bb/axis_estimates')
    def ch(c): chan[P.u16(c, 0)] = bytes(c[8:8 + P.u32(c, 4)]).decode('utf-8', 'replace')
    def msg(c):
        topic = chan.get(P.u16(c, 0))
        if topic not in WANT: return
        data = c[22:]
        if topic == '/throw_announcements':
            ann.append(P.dec_announcement(data))
        elif topic == '/bb/axis_estimates':
            st, d = dec_jointstate(data)
            if 'bb_hand' in d and d['bb_hand'][1] is not None:
                hand.append((st, d['bb_hand'][1]))
        else:
            cd = P.CDR(data); k = cd.u32()
            for _ in range(k):
                stamp, bid, status, tracking, src, pos, vel = P.dec_ball(cd)
                if src == 'ball_butler' and status == 1 and tracking == 1:
                    balls[bid].append((stamp, pos))
    f = open(path, 'rb'); mm = mmap.mmap(f.fileno(), 0, access=mmap.ACCESS_READ)
    assert mm[:8] == b'\x89MCAP0\r\n'
    for op, c in P.iter_records(memoryview(mm)[8:]):
        if op == 4: ch(c)
        elif op == 6:
            cl = P.u32(c, 28); comp = bytes(c[32:32 + cl]); q = 32 + cl
            rs = P.u64(c, q); q += 8
            for io, ic in P.iter_records(memoryview(P.inflate(comp, c[q:q + rs], P.u64(c, 16)))):
                if io == 4: ch(ic)
                elif io == 5: msg(ic)
        elif op == 5: msg(c)
    hand.sort()
    return ann, balls, hand


def trapezoid_peak(T, V):
    """Peak of a trapezoidal speed profile: fit rising + falling legs as lines and
    return their intersection (the true peak the thin flat-top under-samples).
    Mirrors analyze_during_throw.fit_trapezoid_peak; falls back to max(V)."""
    vmax = max(V)
    i_pk = max(range(len(V)), key=lambda i: V[i])
    if i_pk < 1 or i_pk > len(V) - 2:
        return vmax
    up_a, up_b = P.linfit(T[:i_pk + 1], V[:i_pk + 1])   # rising leg
    dn_a, dn_b = P.linfit(T[i_pk:], V[i_pk:])           # falling leg
    if abs(up_a - dn_a) < 1e-6:
        return vmax
    tc = (dn_b - up_b) / (up_a - dn_a)
    vc = up_a * tc + up_b
    if not (T[0] <= tc <= T[-1]) or vc < 0.9 * vmax:
        return vmax
    return vc


def hand_peak_revps(hand, tt):
    w = [(t / 1e9 - tt, v) for t, v in hand if tt + STROKE_WIN[0] <= t / 1e9 <= tt + STROKE_WIN[1]]
    if len(w) < 6: return None
    w.sort()
    return trapezoid_peak([a for a, _ in w], [b for _, b in w])


def main():
    bag = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_BAG
    print("Reading %s ..." % os.path.basename(bag), flush=True)
    ann, balls, hand = read_bag_full(bag)
    pairs = P.match(ann, balls)
    rows = []
    for a, samples in pairs:
        fit = P.fit_throw(a, samples)
        if not fit: continue
        Hpk = hand_peak_revps(hand, a['throw_time_ns'] / 1e9)
        if Hpk is None: continue
        C = fit['cmag'] / 1000.0                       # commanded |v0| m/s
        Hlin = Hpk * TWO_PI_R                           # hand peak linear m/s
        B = math.hypot(fit['vh'], fit['vz_L']) / 1000.0 # ball launch |v| m/s
        rows.append(dict(C=C, H=Hlin, B=B, hc=Hlin / C, bc=B / C, bh=B / Hlin))
    if not rows:
        print("no throws with all three (ann + hand + ball)"); return

    def mean(k): return sum(r[k] for r in rows) / len(rows)
    hc, bc, bh = mean('hc'), mean('bc'), mean('bh')
    print("\nn=%d throws with commanded + hand + ball" % len(rows))
    print("  H/C  hand peak / commanded   = %.3f   (servo under-track %.1f%%)" % (hc, (1 - hc) * 100))
    print("  B/C  ball / commanded        = %.3f   (net launch deficit %.1f%%)" % (bc, (1 - bc) * 100))
    print("  B/H  ball / hand peak        = %.3f   (hand->ball gap %.1f%%)" % (bh, (1 - bh) * 100))

    try:
        import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
    except Exception as e:
        print("plot skipped:", e); return
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13.5, 5.2), gridspec_kw=dict(width_ratios=[1, 1.1]))

    # LEFT: the speed chain as a descending bar (waterfall)
    stages = [('commanded\n(solver v0)', 1.0, '#b45309'),
              ('hand peak\n(encoder)', hc, '#1f6fb4'),
              ('ball launch\n(mocap)', bc, '#0f766e')]
    xs = range(len(stages))
    ax1.bar([s[0] for s in stages], [s[1] for s in stages],
            color=[s[2] for s in stages], width=0.6, zorder=3)
    for i, (lbl, v, _) in enumerate(stages):
        ax1.annotate('%.1f%%' % (v * 100), (i, v), ha='center', va='bottom', fontweight='bold', fontsize=11)
    # gap labels
    ax1.annotate('', xy=(0.5, hc), xytext=(0.5, 1.0),
                 arrowprops=dict(arrowstyle='<->', color='#444'))
    ax1.text(0.56, (1.0 + hc) / 2, 'servo under-track\n%.1f%%' % ((1 - hc) * 100), fontsize=9, va='center')
    ax1.annotate('', xy=(1.5, bc), xytext=(1.5, hc),
                 arrowprops=dict(arrowstyle='<->', color='#444'))
    ax1.text(1.56, (hc + bc) / 2, 'hand→ball gap\n%.1f%%' % ((1 - bh) * 100), fontsize=9, va='center')
    ax1.set_ylim(0.80, 1.03); ax1.set_ylabel('speed / commanded')
    ax1.axhline(1.0, color='#b45309', lw=0.8, ls='--')
    ax1.set_title('Where the launch-speed deficit comes from\n(same %d throws: commanded → hand → ball)' % len(rows),
                  fontsize=11); ax1.grid(alpha=0.3, axis='y')

    # RIGHT: per-throw H/C and B/C distributions
    import random
    rng = random.Random(1)
    for gi, (key, lbl, col) in enumerate([('hc', 'H/C  hand peak / commanded', '#1f6fb4'),
                                          ('bc', 'B/C  ball / commanded', '#0f766e')]):
        vals = [r[key] for r in rows]
        xs = [gi + (rng.random() - 0.5) * 0.45 for _ in vals]
        ax2.scatter(xs, vals, s=20, color=col, alpha=0.75, label=lbl)
        m = sum(vals) / len(vals)
        ax2.plot([gi - 0.3, gi + 0.3], [m, m], color='k', lw=2.2)
        ax2.annotate('%.3f' % m, (gi, m), ha='center', va='bottom', fontweight='bold', fontsize=10)
    ax2.axhline(1.0, color='#b45309', lw=1.4, ls='--')
    ax2.annotate('commanded = 1.00', (0.5, 1.0), xytext=(0.1, 1.01), color='#b45309', fontsize=9)
    ax2.set_xticks([0, 1]); ax2.set_xticklabels(['hand (H/C)', 'ball (B/C)'])
    ax2.set_xlim(-0.5, 1.5); ax2.set_ylim(0.83, 1.05); ax2.set_ylabel('measured / commanded')
    ax2.set_title('Hand nearly reaches command; ball leaves slower\n(angle is correct — this is purely speed)', fontsize=11)
    ax2.legend(fontsize=8.5, loc='lower left'); ax2.grid(alpha=0.3, axis='y')

    fig.tight_layout(); fig.savefig(os.path.abspath(OUT), dpi=125)
    print("Saved ->", os.path.abspath(OUT))


if __name__ == "__main__":
    main()
