#!/usr/bin/env python3
"""2026-06-17-era speed chain + the "hot vs slow" reversal explainer, from
kinematic_obs.json (extract_kinematic_obs.py output: per-throw commanded speed,
hand-encoder peak, and mocap launch velocity for the 06-17 evening kinematic runs).

The 06-17 analysis evaluated the launch velocity AT the commanded throw_time, but the
ball releases ~44 ms later — so its vz is read before gravity has acted, inflating it
into a "hot" launch. Correcting vz to the actual release (subtract g*0.044) flips the
ball from hot to slow — matching the 06-18 picture. The hand encoder is unchanged.
=> the effect did not physically reverse; the analysis did.

Pure-Python + matplotlib.   /usr/bin/python3 plot_0617_chain.py
"""
import json, math, os

G = 9810.0; R = 0.0052493; TWO_PI_R = 2 * math.pi * R; LAT = 0.044
OBS = os.path.join(os.path.dirname(__file__), "kinematic_obs.json")
OUT = os.path.join(os.path.dirname(__file__), "..", "..", "..",
                   "logbook", "chapters", "images", "speed-chain-0617.png")

rows = json.load(open(OBS))
ok = [r for r in rows if r.get('ok') and 9300 < r.get('g_eff', 0) < 10300
      and r.get('rms_mm', 99) < 8 and r.get('n', 0) >= 20]

def C(r):  return r['cmd_speed']                                   # m/s
def H(r):  return r['enc_hand_vpeak_rps'] * TWO_PI_R               # hand peak linear m/s
def Vthrow(r): return math.sqrt(r['vx']**2 + r['vy']**2 + r['vz']**2) / 1000.0          # vz @ throw_time
def Vrel(r):   return math.sqrt(r['vx']**2 + r['vy']**2 + (r['vz'] - G * LAT)**2) / 1000.0  # vz @ release

hc  = [H(r) / C(r) for r in ok]
bt  = [Vthrow(r) / C(r) for r in ok]   # 06-17 method (hot)
br  = [Vrel(r) / C(r) for r in ok]     # release-corrected (slow)
mean = lambda v: sum(v) / len(v)
mhc, mbt, mbr = mean(hc), mean(bt), mean(br)
print("n=%d  H/C=%.3f  |v|/C @throw_time=%.3f (hot)  @release=%.3f (slow)" % (len(ok), mhc, mbt, mbr))

import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
fig, (axL, axR) = plt.subplots(1, 2, figsize=(13.5, 5.2), gridspec_kw=dict(width_ratios=[1, 1.05]))

# LEFT: the true 06-17 chain (release-corrected) — same shape as 06-18
stages = [('commanded\n(setpoint)', 1.0, '#b45309'),
          ('hand peak\n(encoder)', mhc, '#1f6fb4'),
          ('ball launch\n(mocap, @release)', mbr, '#0f766e')]
axL.bar([s[0] for s in stages], [s[1] for s in stages], color=[s[2] for s in stages], width=0.6, zorder=3)
for i, (lbl, v, _) in enumerate(stages):
    axL.annotate('%.1f%%' % (v * 100), (i, v), ha='center', va='bottom', fontweight='bold', fontsize=11)
axL.annotate('', xy=(0.5, mhc), xytext=(0.5, 1.0), arrowprops=dict(arrowstyle='<->', color='#444'))
axL.text(0.56, (1.0 + mhc) / 2, 'servo\n%.1f%%' % ((1 - mhc) * 100), fontsize=9, va='center')
axL.annotate('', xy=(1.5, mbr), xytext=(1.5, mhc), arrowprops=dict(arrowstyle='<->', color='#444'))
axL.text(1.56, (mhc + mbr) / 2, 'hand→ball\n%.1f%%' % ((1 - mbr / mhc) * 100), fontsize=9, va='center')
axL.axhline(1.0, color='#b45309', lw=0.8, ls='--')
axL.set_ylim(0.80, 1.03); axL.set_ylabel('speed / commanded')
axL.set_title('2026-06-17 era: same chain as 06-18\n(hand 97.6%, like 06-18 — unchanged; ball noisier kinematic grid)', fontsize=10.5)
axL.grid(alpha=0.3, axis='y')

# RIGHT: the reversal explainer — ball |v|/C read two ways
import random
rng = random.Random(7)
for gi, (vals, lbl, col) in enumerate([(bt, 'vz @ throw_time\n(the 06-17 method)', '#c2410c'),
                                       (br, 'vz @ actual release\n(+44 ms)', '#0f766e')]):
    xs = [gi + (rng.random() - 0.5) * 0.4 for _ in vals]
    axR.scatter(xs, vals, s=20, color=col, alpha=0.7)
    m = mean(vals)
    axR.plot([gi - 0.3, gi + 0.3], [m, m], color='k', lw=2.4)
    axR.annotate('%.3f' % m, (gi, m), ha='center', va='bottom' if m > 1 else 'top', fontweight='bold', fontsize=11)
axR.axhline(1.0, color='#b45309', lw=1.5, ls='--')
axR.annotate('commanded = 1.00', (1.0, 1.0), xytext=(1.18, 1.005), color='#b45309', fontsize=9, ha='right')
axR.annotate('', xy=(0.78, mbr + 0.01), xytext=(0.22, mbt - 0.01),
             arrowprops=dict(arrowstyle='->', color='#555', lw=1.5, connectionstyle='arc3,rad=-0.25'))
axR.text(0.5, 0.935, 'removing the 44 ms\nflips hot → slow', fontsize=9.5, ha='center', color='#555')
axR.set_xticks([0, 1]); axR.set_xticklabels(['"hot" (+4%)', 'slow (−11%)'], fontsize=10)
axR.set_xlim(-0.5, 1.5); axR.set_ylim(0.80, 1.12); axR.set_ylabel('ball |v| / commanded')
axR.set_title('Why 06-17 read "hot": vz evaluated 44 ms before release\n(same throws, same data — only the evaluation instant differs)', fontsize=11)
axR.grid(alpha=0.3, axis='y')

fig.tight_layout(); fig.savefig(os.path.abspath(OUT), dpi=125)
print("Saved ->", os.path.abspath(OUT))
