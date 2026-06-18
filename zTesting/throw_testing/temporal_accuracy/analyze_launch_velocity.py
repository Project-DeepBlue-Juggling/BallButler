#!/home/jetson/Desktop/PDJ_venv/venv/bin/python3
"""
Definitive launch-velocity analyzer (track A primary + track B fallback).

KEY FINDING (2026-06-17): the announcement-matched KF (source 'ball_butler',
"track A") is MEASUREMENT-DRIVEN over the early-mid flight window [tt+0.10,
tt+0.60], NOT prior-coasting there: it fits the known-g parabola with
g_eff_free ~ 9806 mm/s^2 (truth) at ~1.5 mm RMS, and its velocity (ratio ~1.11)
is NOT the commanded prior (ratio 1.0) — so measurements, not the prior, set it.
It also agrees with the prior-FREE re-detection ("track B", 'human_throw') to
<0.5% on the throws where B exists. A only coasts LATE (after ~0.6 s, through the
floor), so we simply don't fit it there.

=> Use track A early-mid for the bulk of throws (clean, n~26/30); fall back to a
   RANSAC fit of track B for the few throws where A diverges early (those are
   exactly the throws where B spawns). Near-complete, cross-validated coverage.

CAVEATS on magnitude:
  * vh ratio is release-height INDEPENDENT (horizontal v is constant in vacuum) —
    the most robust number. vh < 1 alone proves a steeper-than-commanded launch.
  * |v| and vz are extrapolated to the MODEL release plane
    (z_rel = p0.z + L_RELEASE*sin(pitch)); a ~50 mm z_rel error moves vz ~5%. So
    the SIGN (hot) and cross-session reproducibility are solid; the exact % carries
    a release-height systematic.
No finite differencing — all velocities are slopes of multi-sample physical fits.

    analyze_launch_velocity.py [BAG.mcap ...]   (defaults to the two 2026-06-17 bags)
"""
import json
import math
import random
import sys
from pathlib import Path

import numpy as np

import analyze_velocity_postfix as AV
import decompose_from_bag as D

G = D.G_MMPS2
L_REL = D.L_RELEASE_MM

A_WIN = (0.10, 0.60)          # track-A early-mid measurement-driven window
B_WIN_AFTER = 0.20
Z_HI = 2700.0
N_MIN = 25
GEFF_OK = (9000.0, 10600.0)
RMS_MAX_A = 8.0
RMS_MAX_B = 18.0
BAND_MM = 45.0
ITERS = 400
SEED_SPAN_MIN = 0.15
SPAN_MIN = 0.30

DEFAULT_BAGS = [
    "/home/jetson/Desktop/rosbags/2026-06-17_15-13-42/2026-06-17_15-13-42_0.mcap",
    "/home/jetson/Desktop/rosbags/2026-06-17_14-51-24/2026-06-17_14-51-24_0.mcap",
]


def _geom(ann):
    v0 = np.array(ann["v0"]); sp = float(np.linalg.norm(v0)); sinp = v0[2] / sp
    return dict(sp=sp, sinp=sinp, z_rel=ann["p0"][2] + L_REL * sinp,
               cvh=float(math.hypot(v0[0], v0[1])), cvz=float(v0[2]),
               z_land=ann["pl"][2], tt=ann["throw_time"], lt=ann["landing_time"],
               tof=ann["tof"])


def _finish(g, t0, z0, vz, vx, vy):
    t_rel = D.cross_gfixed(z0, vz, t0, g["z_rel"], ascending=True)
    t_land = D.cross_gfixed(z0, vz, t0, g["z_land"], ascending=False)
    if t_rel is None or t_land is None:
        return None
    vz_rel = vz - G * (t_rel - t0)
    vh = math.hypot(vx, vy)
    vmag = math.sqrt(vx * vx + vy * vy + vz_rel * vz_rel)
    return dict(release_lag_ms=float((t_rel - g["tt"]) * 1e3),
                flight_error_ms=float(((t_land - t_rel) - g["tof"]) * 1e3),
                meas_speed_mmps=float(vmag), ratio=float(vmag / g["sp"]),
                vz_ratio=float(vz_rel / g["cvz"]), vh_ratio=float(vh / g["cvh"]))


def fit_trackA(ann, balls):
    g = _geom(ann)
    m = ((balls["src"] == "ball_butler") & (balls["t"] >= g["tt"] + A_WIN[0]) &
         (balls["t"] <= g["tt"] + A_WIN[1]) &
         (balls["z"] > g["z_land"] - 200) & (balls["z"] < Z_HI))
    if m.sum() < N_MIN:
        return None
    t = balls["t"][m]; x = balls["x"][m]; y = balls["y"][m]; z = balls["z"][m]
    t0 = float(np.median(t)); T = t - t0; inl = np.ones(len(T), bool)
    for _ in range(6):
        z0, vz, _ = D.fit_gfixed(T[inl], z[inl], 0.0)
        r = z + 0.5 * G * T * T - (z0 + vz * T); s = np.std(r[inl])
        k = np.abs(r - np.mean(r[inl])) < 3 * max(s, 3.0)
        if k.sum() == inl.sum() or k.sum() < N_MIN:
            break
        inl = k
    z0, vz, rms = D.fit_gfixed(T[inl], z[inl], 0.0)
    geff = -2 * np.polyfit(T[inl], z[inl], 2)[0]
    if not (GEFF_OK[0] <= geff <= GEFF_OK[1]) or rms > RMS_MAX_A:
        return None
    vx = float(np.polyfit(T[inl], x[inl], 1)[0]); vy = float(np.polyfit(T[inl], y[inl], 1)[0])
    fin = _finish(g, t0, z0, vz, vx, vy)
    if fin is None:
        return None
    fin.update(src="A", n=int(inl.sum()), g_eff=float(geff), rms_mm=float(rms))
    return fin


def fit_trackB(ann, balls, rng):
    g = _geom(ann)
    m = ((balls["src"] == "human_throw") & (balls["t"] >= g["tt"]) &
         (balls["t"] <= g["lt"] + B_WIN_AFTER) &
         (balls["z"] > g["z_land"] - 80) & (balls["z"] < Z_HI))
    if m.sum() < N_MIN:
        return None
    t = balls["t"][m]; x = balls["x"][m]; y = balls["y"][m]; z = balls["z"][m]
    t0 = float(np.median(t)); T = t - t0; N = len(T)
    best = None; bc = 0
    for _ in range(ITERS):
        s = rng.sample(range(N), 3)
        if T[s].max() - T[s].min() < SEED_SPAN_MIN:
            continue
        vx = np.polyfit(T[s], x[s], 1)[0]; vy = np.polyfit(T[s], y[s], 1)[0]
        A = np.vstack([np.ones(3), T[s]]).T
        z0, vz = np.linalg.lstsq(A, z[s] + 0.5 * G * T[s] ** 2, rcond=None)[0]
        d = np.sqrt((vx * T + np.polyfit(T[s], x[s], 1)[1] - x) ** 2 +
                    (vy * T + np.polyfit(T[s], y[s], 1)[1] - y) ** 2 +
                    (z0 + vz * T - 0.5 * G * T * T - z) ** 2)
        c = int((d < BAND_MM).sum())
        if c > bc:
            bc = c; best = d < BAND_MM
    if best is None or bc < N_MIN:
        return None
    inl = best
    z0, vz, rms = D.fit_gfixed(T[inl], z[inl], 0.0)
    geff = -2 * np.polyfit(T[inl], z[inl], 2)[0]
    span = float(T[inl].max() - T[inl].min())
    if not (GEFF_OK[0] <= geff <= GEFF_OK[1]) or rms > RMS_MAX_B or span < SPAN_MIN:
        return None
    vx = float(np.polyfit(T[inl], x[inl], 1)[0]); vy = float(np.polyfit(T[inl], y[inl], 1)[0])
    fin = _finish(g, t0, z0, vz, vx, vy)
    if fin is None:
        return None
    fin.update(src="B", n=int(inl.sum()), g_eff=float(geff), rms_mm=float(rms))
    return fin


def stt(vals):
    v = np.asarray([x for x in vals if x is not None and np.isfinite(x)], float)
    if not len(v):
        return None
    return dict(n=int(len(v)), mean=float(v.mean()),
                std=float(v.std(ddof=1)) if len(v) > 1 else 0.0,
                med=float(np.median(v)), min=float(v.min()), max=float(v.max()))


def analyze(bagpath):
    AV.BAG = Path(bagpath)
    store = D.setup_typestore()
    anns, catches, balls = AV.read_bag_purepy(store)
    rng = random.Random(42)
    rows = []
    for i, a in enumerate(anns):
        rA = fit_trackA(a, balls)
        rB = fit_trackB(a, balls, rng)
        chosen = rA if rA else rB
        rows.append(dict(idx=i, A=rA, B=rB, chosen=chosen))
    return anns, catches, rows


def report(name, anns, rows):
    chosen = [r["chosen"] for r in rows if r["chosen"]]
    both = [(r["A"], r["B"]) for r in rows if r["A"] and r["B"]]
    nA = sum(1 for r in rows if r["chosen"] and r["chosen"]["src"] == "A")
    nB = sum(1 for r in rows if r["chosen"] and r["chosen"]["src"] == "B")
    print(f"\n############ {name} ############")
    print(f"valid throws: {len(chosen)}/{len(rows)}  (track A={nA}, track B fallback={nB})")
    for k, lbl in [("ratio", "|v| meas/cmd"), ("vz_ratio", "vz meas/cmd  "),
                   ("vh_ratio", "vh meas/cmd  "), ("release_lag_ms", "release lag ms"),
                   ("flight_error_ms", "flight err ms ")]:
        s = stt([c.get(k) for c in chosen])
        if s:
            print(f"  {lbl} mean={s['mean']:+8.3f} σ={s['std']:6.3f} "
                  f"med={s['med']:+8.3f} [{s['min']:+7.3f},{s['max']:+7.3f}]")
    gq = stt([c["g_eff"] for c in chosen]); rq = stt([c["rms_mm"] for c in chosen])
    print(f"  [quality] g_eff {gq['mean']:.0f}±{gq['std']:.0f}  RMS {rq['mean']:.1f}±{rq['std']:.1f} mm")
    if both:
        dv = [abs(a["ratio"] - b["ratio"]) for a, b in both]
        print(f"  [A-vs-B cross-check] n={len(both)} throws have both; "
              f"|Δ(|v| ratio)| mean={np.mean(dv):.4f} max={np.max(dv):.4f}")
    # angle
    s_vz = stt([c["vz_ratio"] for c in chosen]); s_vh = stt([c["vh_ratio"] for c in chosen])
    a0 = anns[0]; v0 = np.array(a0["v0"])
    cmd_el = math.degrees(math.atan2(v0[2], math.hypot(v0[0], v0[1])))
    meas_el = math.degrees(math.atan2(s_vz["mean"] * v0[2],
                                      s_vh["mean"] * math.hypot(v0[0], v0[1])))
    print(f"  launch elevation: commanded {cmd_el:.1f}°  measured {meas_el:.1f}°  "
          f"(Δ {meas_el - cmd_el:+.1f}° steeper)")
    return chosen


def main():
    bags = sys.argv[1:] if len(sys.argv) > 1 else DEFAULT_BAGS
    allrows = {}
    for b in bags:
        print(f"Reading {Path(b).name} ...", flush=True)
        anns, catches, rows = analyze(b)
        report(Path(b).name, anns, rows)
        allrows[b] = [r["chosen"] for r in rows if r["chosen"]]
    Path(__file__).with_name("launch_velocity_results.json").write_text(
        json.dumps({k: v for k, v in allrows.items()}, indent=1, default=float))
    print("\nSaved -> launch_velocity_results.json")


if __name__ == "__main__":
    main()
