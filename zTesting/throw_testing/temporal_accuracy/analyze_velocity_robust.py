#!/home/jetson/Desktop/PDJ_venv/venv/bin/python3
"""
Robust launch-velocity extractor for the post-fix session (2026-06-17 14:55).

Replaces decompose_from_bag's single-longest-fragment track-B selection (brittle
under the new flight timing) with a POOL + SIGMA-CLIP fit:

  1. Pool ALL 'human_throw' samples (measurement-driven; excludes the prior-
     biased 'ball_butler' coast track) inside THIS flight's window
     [throw_time, landing_time + 0.20 s], with a sane z gate.
  2. Iteratively sigma-clip against the known-g parabola
     z + 1/2 g T^2 = z0 + vz T  (linear LSQ, g fixed 9806) until inliers settle.
     The KF re-spawn fragments all lie on the SAME physical parabola, so pooling
     uses every good sample; divergent KF excursions are clipped out.
  3. Horizontal vx,vy = robust linear fit on the inliers (vh constant in vacuum).
  4. Extrapolate vz to the model release plane; report v_meas/v_cmd, plus the
     g-free g_eff and residual RMS as fit-quality gates.

No finite differencing anywhere — every velocity is a slope from a multi-sample
fit with the physical shape imposed.

    /home/jetson/Desktop/PDJ_venv/venv/bin/python3 analyze_velocity_robust.py
"""
import json
import math
from pathlib import Path

import numpy as np

import analyze_velocity_postfix as AV   # read_bag_purepy
import decompose_from_bag as D          # fit_gfixed, fit_gfree, cross_gfixed, constants

G = D.G_MMPS2
L_REL = D.L_RELEASE_MM
OUT = Path(__file__).with_name("velocity_robust_results.json")

WIN_AFTER = 0.20          # s past landing_time to include (descent past cone)
Z_LO_MARGIN = -80.0       # accept z down to z_land + this
Z_HI = 2700.0             # reject KF divergence above this
N_MIN = 25
GEFF_OK = (8800.0, 10800.0)
RMS_MAX = 20.0


def robust_throw(ann, balls):
    tt, lt = ann["throw_time"], ann["landing_time"]
    tof = ann["tof"]
    p0 = np.array(ann["p0"]); v0 = np.array(ann["v0"])
    z_land = ann["pl"][2]; vz_land = ann["vl"][2]
    speed0 = float(np.linalg.norm(v0)); sinp = v0[2] / speed0
    z_rel = p0[2] + L_REL * sinp
    cmd_vh = float(math.hypot(v0[0], v0[1])); cmd_vz = float(v0[2])

    out = dict(idx=None, arrival_error_ms=None, n_inlier=0, flags=[],
               commanded_speed_mmps=speed0, cmd_vh_mmps=cmd_vh, cmd_vz_mmps=cmd_vz)

    # catch (for arrival error sanity)
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
    inl = np.ones(len(T), bool)
    for _ in range(6):
        z0, vz, rms = D.fit_gfixed(T[inl], z[inl], 0.0)  # T already centred
        resid = (z + 0.5 * G * T * T) - (z0 + vz * T)
        s = np.std(resid[inl])
        keep = np.abs(resid - np.mean(resid[inl])) < 3.0 * max(s, 3.0)
        if keep.sum() == inl.sum() and np.array_equal(keep, inl):
            break
        if keep.sum() < N_MIN:
            break
        inl = keep

    n = int(inl.sum())
    out["n_inlier"] = n
    if n < N_MIN:
        out["flags"].append("inliers_collapsed"); return out

    z0, vz, rms = D.fit_gfixed(T[inl], z[inl], 0.0)
    _, _, cfree, _ = D.fit_gfree(T[inl], z[inl], 0.0)
    g_eff = -2 * cfree
    vx = float(np.polyfit(T[inl], x[inl], 1)[0])
    vy = float(np.polyfit(T[inl], y[inl], 1)[0])

    out.update(rms_mm=float(rms), g_eff=float(g_eff),
               t_span_rel=(round(float(t[inl].min() - tt), 3),
                           round(float(t[inl].max() - tt), 3)))
    if not (GEFF_OK[0] <= g_eff <= GEFF_OK[1]) or rms > RMS_MAX:
        out["flags"].append("fit_quality_gate"); return out

    t_rel = D.cross_gfixed(z0, vz, t0, z_rel, ascending=True)
    t_land = D.cross_gfixed(z0, vz, t0, z_land, ascending=False)
    if t_rel is None or t_land is None:
        out["flags"].append("crossing_failed"); return out

    vz_rel = vz - G * (t_rel - t0)
    vh = math.hypot(vx, vy)
    out.update(
        release_lag_ms=float((t_rel - tt) * 1e3),
        flight_error_ms=float(((t_land - t_rel) - tof) * 1e3),
        t_land_rel_landing_ms=float((t_land - lt) * 1e3),
        meas_speed_mmps=float(math.sqrt(vx * vx + vy * vy + vz_rel * vz_rel)),
        meas_vz_rel_mmps=float(vz_rel), meas_vh_mmps=float(vh),
        ratio=float(math.sqrt(vx * vx + vy * vy + vz_rel * vz_rel) / speed0),
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
    store = D.setup_typestore()
    print("Reading bag (summary-free)...", flush=True)
    anns, catchesG, balls = AV.read_bag_purepy(store)
    print(f"announcements={len(anns)} catches={len(catchesG)} ball samples={len(balls['t'])}")

    res = []
    for i, a in enumerate(anns):
        r = robust_throw(a, balls); r["idx"] = i; res.append(r)

    print("\nidx arr_err rel_lag flt_err  v_cmd v_meas ratio  vz_r  vh_r  g_eff rms  n   flags")
    for r in res:
        def g(k, f="{:7.1f}"):
            return f.format(r[k]) if r.get(k) is not None else "   --- "
        print(f"{r['idx']:3d}{g('arrival_error_ms')}{g('release_lag_ms')}{g('flight_error_ms')}"
              f"{g('commanded_speed_mmps','{:6.0f}')}{g('meas_speed_mmps','{:6.0f}')}"
              f"{g('ratio','{:6.3f}')}{g('vz_ratio','{:6.3f}')}{g('vh_ratio','{:6.3f}')}"
              f"{g('g_eff','{:6.0f}')}{g('rms_mm','{:4.1f}')}{r['n_inlier']:4d}  "
              f"{','.join(r['flags'])}")

    good = [r for r in res if "ratio" in r and not r["flags"]]
    print(f"\n=== VALID THROWS: {len(good)}/{len(res)} ===")
    for k, lbl in [("ratio", "|v| meas/cmd"), ("vz_ratio", "vz meas/cmd"),
                   ("vh_ratio", "vh meas/cmd"), ("release_lag_ms", "release lag ms"),
                   ("flight_error_ms", "flight error ms"), ("arrival_error_ms", "arrival err ms")]:
        s = stt([r.get(k) for r in good])
        if s:
            print(f"  {lbl:16s} mean={s['mean']:+8.3f} σ={s['std']:6.3f} "
                  f"med={s['med']:+8.3f} [{s['min']:+8.3f},{s['max']:+8.3f}] n={s['n']}")
    for k, lbl in [("g_eff", "g_eff free"), ("rms_mm", "fit RMS mm"), ("n_inlier", "inliers")]:
        s = stt([r.get(k) for r in good])
        if s:
            print(f"  [qual] {lbl:12s} mean={s['mean']:.1f} σ={s['std']:.1f}")

    OUT.write_text(json.dumps(dict(n_valid=len(good), throws=res), indent=1, default=float))
    print(f"\nSaved -> {OUT}")


if __name__ == "__main__":
    main()
