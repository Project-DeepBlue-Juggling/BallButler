#!/home/jetson/Desktop/PDJ_venv/venv/bin/python3
"""
RANSAC launch-velocity extractor — post-fix session (2026-06-17 14:55).

The pooled 'human_throw' samples for a throw contain THREE populations: the real
flight arc, KF-divergence excursions (z up to +2700 / down to -9000), and the
previous ball still settling near the cone. A plain least-squares fit is wrecked
by the outliers; a sigma-clip seeded from that fit can't recover.

RANSAC fixes this: repeatedly hypothesise a 3D ballistic model from a few
time-spread samples (vx, vy from linear x(t), y(t); z0, vz from the known-g
parabola), score by how many pooled samples fall within BAND mm in 3D, keep the
best-supported model, then refit on its inliers. The real flight is the single
most-supported coherent 3D arc, so it wins regardless of contamination. Still no
finite differencing — velocities are slopes of multi-sample fits.

Quality gates: g_eff (g re-fit FREE) near 9806, residual RMS small, enough
inliers spanning enough time. Deterministic (fixed RNG seed) for reproducibility.

    /home/jetson/Desktop/PDJ_venv/venv/bin/python3 analyze_velocity_ransac.py
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
OUT = Path(__file__).with_name("velocity_ransac_results.json")

WIN_AFTER = 0.20
Z_LO_MARGIN = -80.0
Z_HI = 2700.0
N_MIN = 25
BAND_MM = 45.0            # 3D inlier band
ITERS = 400
SEED_SPAN_MIN = 0.15     # s — seed points must span at least this
GEFF_OK = (9000.0, 10600.0)
RMS_MAX = 18.0
SPAN_MIN = 0.30          # s — inlier time span required


def fit_model(T, X, Y, Z):
    vx = np.polyfit(T, X, 1)[0]; x0 = np.polyfit(T, X, 1)[1]
    vy = np.polyfit(T, Y, 1)[0]; y0 = np.polyfit(T, Y, 1)[1]
    # g-fixed: z + 0.5 g T^2 = z0 + vz T
    A = np.vstack([np.ones_like(T), T]).T
    coef, *_ = np.linalg.lstsq(A, Z + 0.5 * G * T * T, rcond=None)
    z0, vz = coef
    return x0, vx, y0, vy, z0, vz


def predict(m, T):
    x0, vx, y0, vy, z0, vz = m
    return (x0 + vx * T, y0 + vy * T, z0 + vz * T - 0.5 * G * T * T)


def ransac_throw(ann, balls, rng):
    tt, lt = ann["throw_time"], ann["landing_time"]
    tof = ann["tof"]
    p0 = np.array(ann["p0"]); v0 = np.array(ann["v0"])
    z_land = ann["pl"][2]
    speed0 = float(np.linalg.norm(v0)); sinp = v0[2] / speed0
    z_rel = p0[2] + L_REL * sinp
    cmd_vh = float(math.hypot(v0[0], v0[1])); cmd_vz = float(v0[2])

    out = dict(idx=None, arrival_error_ms=None, n_inlier=0, flags=[],
               commanded_speed_mmps=speed0, cmd_vh_mmps=cmd_vh, cmd_vz_mmps=cmd_vz)
    cand = [c for c in catchesG if lt - 0.3 <= c["t"] <= lt + 0.6]
    if cand:
        out["arrival_error_ms"] = (cand[0]["t"] - lt) * 1e3

    m = ((balls["src"] == "human_throw") & (balls["t"] >= tt) &
         (balls["t"] <= lt + WIN_AFTER) &
         (balls["z"] > z_land + Z_LO_MARGIN) & (balls["z"] < Z_HI))
    if m.sum() < N_MIN:
        out["flags"].append("too_few_human_throw"); return out

    t = balls["t"][m]; x = balls["x"][m]; y = balls["y"][m]; z = balls["z"][m]
    t0 = float(np.median(t)); T = t - t0
    N = len(T)

    best_inl = None; best_cnt = 0
    idxs = list(range(N))
    for _ in range(ITERS):
        s = rng.sample(idxs, 3)
        if T[s].max() - T[s].min() < SEED_SPAN_MIN:
            continue
        try:
            mdl = fit_model(T[s], x[s], y[s], z[s])
        except Exception:
            continue
        px, py, pz = predict(mdl, T)
        d = np.sqrt((px - x) ** 2 + (py - y) ** 2 + (pz - z) ** 2)
        inl = d < BAND_MM
        c = int(inl.sum())
        if c > best_cnt:
            best_cnt = c; best_inl = inl

    if best_inl is None or best_cnt < N_MIN:
        out["flags"].append("ransac_no_consensus"); out["n_inlier"] = best_cnt; return out

    # one refit + reselect pass
    mdl = fit_model(T[best_inl], x[best_inl], y[best_inl], z[best_inl])
    px, py, pz = predict(mdl, T)
    d = np.sqrt((px - x) ** 2 + (py - y) ** 2 + (pz - z) ** 2)
    inl = d < BAND_MM
    if inl.sum() < N_MIN:
        out["flags"].append("inliers_collapsed"); out["n_inlier"] = int(inl.sum()); return out

    x0, vx, y0, vy, z0, vz = fit_model(T[inl], x[inl], y[inl], z[inl])
    n = int(inl.sum()); out["n_inlier"] = n
    span = float(T[inl].max() - T[inl].min())
    resid = z[inl] + 0.5 * G * T[inl] ** 2 - (z0 + vz * T[inl])
    rms = float(np.sqrt(np.mean(resid ** 2)))
    cc = np.polyfit(T[inl], z[inl], 2)
    g_eff = -2 * cc[0]
    out.update(rms_mm=rms, g_eff=float(g_eff), span_s=round(span, 3),
               t_span_rel=(round(float(t[inl].min() - tt), 3),
                           round(float(t[inl].max() - tt), 3)))
    if not (GEFF_OK[0] <= g_eff <= GEFF_OK[1]) or rms > RMS_MAX or span < SPAN_MIN:
        out["flags"].append("fit_quality_gate"); return out

    t_rel = D.cross_gfixed(z0, vz, t0, z_rel, ascending=True)
    t_land = D.cross_gfixed(z0, vz, t0, z_land, ascending=False)
    if t_rel is None or t_land is None:
        out["flags"].append("crossing_failed"); return out
    vz_rel = vz - G * (t_rel - t0)
    vh = math.hypot(vx, vy)
    v_meas = math.sqrt(vx * vx + vy * vy + vz_rel * vz_rel)
    out.update(
        release_lag_ms=float((t_rel - tt) * 1e3),
        flight_error_ms=float(((t_land - t_rel) - tof) * 1e3),
        meas_speed_mmps=float(v_meas), meas_vz_rel_mmps=float(vz_rel),
        meas_vh_mmps=float(vh), ratio=float(v_meas / speed0),
        vz_ratio=float(vz_rel / cmd_vz), vh_ratio=float(vh / cmd_vh))
    return out


def stt(vals):
    v = np.asarray([x for x in vals if x is not None and np.isfinite(x)], float)
    if not len(v):
        return None
    return dict(n=int(len(v)), mean=float(v.mean()),
                std=float(v.std(ddof=1)) if len(v) > 1 else 0.0,
                med=float(np.median(v)), min=float(v.min()), max=float(v.max()))


def main():
    global catchesG
    if len(sys.argv) > 1:
        AV.BAG = Path(sys.argv[1])
    store = D.setup_typestore()
    print(f"Reading bag (summary-free): {AV.BAG.name}", flush=True)
    anns, catchesG, balls = AV.read_bag_purepy(store)
    print(f"announcements={len(anns)} catches={len(catchesG)} ball samples={len(balls['t'])}")
    rng = random.Random(42)

    res = []
    for i, a in enumerate(anns):
        r = ransac_throw(a, balls, rng); r["idx"] = i; res.append(r)

    print("\nidx arr_err rel_lag flt_err  v_cmd v_meas ratio  vz_r  vh_r  g_eff rms  n  span flags")
    for r in res:
        def g(k, f="{:7.1f}"):
            return f.format(r[k]) if r.get(k) is not None else "   --- "
        print(f"{r['idx']:3d}{g('arrival_error_ms')}{g('release_lag_ms')}{g('flight_error_ms')}"
              f"{g('commanded_speed_mmps','{:6.0f}')}{g('meas_speed_mmps','{:6.0f}')}"
              f"{g('ratio','{:6.3f}')}{g('vz_ratio','{:6.3f}')}{g('vh_ratio','{:6.3f}')}"
              f"{g('g_eff','{:6.0f}')}{g('rms_mm','{:4.1f}')}{r['n_inlier']:4d}"
              f"{g('span_s','{:5.2f}')}  {','.join(r['flags'])}")

    good = [r for r in res if "ratio" in r and not r["flags"]]
    print(f"\n=== VALID THROWS: {len(good)}/{len(res)} ===")
    for k, lbl in [("ratio", "|v| meas/cmd"), ("vz_ratio", "vz meas/cmd"),
                   ("vh_ratio", "vh meas/cmd"), ("release_lag_ms", "release lag ms"),
                   ("flight_error_ms", "flight error ms"), ("arrival_error_ms", "arrival err ms")]:
        s = stt([r.get(k) for r in good])
        if s:
            print(f"  {lbl:16s} mean={s['mean']:+8.3f} σ={s['std']:6.3f} "
                  f"med={s['med']:+8.3f} [{s['min']:+8.3f},{s['max']:+8.3f}] n={s['n']}")
    for k, lbl in [("g_eff", "g_eff free"), ("rms_mm", "fit RMS mm"),
                   ("n_inlier", "inliers"), ("span_s", "span s")]:
        s = stt([r.get(k) for r in good])
        if s:
            print(f"  [qual] {lbl:12s} mean={s['mean']:.2f} σ={s['std']:.2f}")

    OUT.write_text(json.dumps(dict(n_valid=len(good), throws=res), indent=1, default=float))
    print(f"\nSaved -> {OUT}")


if __name__ == "__main__":
    main()
