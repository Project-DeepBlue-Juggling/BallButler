#!/home/jetson/Desktop/PDJ_venv/venv/bin/python3
"""
Empirical decomposition of the +126.5 ms arrival error from the 2026-06-10 bag.

KEY FINDING THAT SHAPES THIS ANALYSIS (see decompose_results.json diagnostics):
/balls contains TWO tracks per throw:
  - track A (source 'ball_butler'): the announcement-matched KF. It is anchored
    to the announced prior (release at throw_time, v = commanded) and loses its
    50 mm marker gate (matcher.py:71) early; it then silently COASTS, sailing
    through the cone and the floor (z < -400 mm). Its timing is prior-biased.
  - track B (source 'human_throw'): re-detection of the SAME ball once its
    markers leave A's gate. It is measurement-driven (spawns mid-descent,
    converges, and decelerates/plateaus at the cone rim exactly when the piezo
    fires). Piezo cross-check: at catch_time track B puts the ball at
    z = 1025 +/- 27 mm = landing plane (960) + ball radius + small KF lag,
    whereas track A claims z = 287 +/- 65 mm (700 mm below the plane).
    /bb/heartbeat hand_pos_mm independently shows the physical hand stroke
    runs ~130-270 ms AFTER throw_time, not before it as the decel-zero plan
    (HandPathPlanner.h:80, t_s=0 at decel start) intends.
==> Track B + piezo is ground truth; the piezo fires AT the landing plane
    (fall-through ~ 0); the arrival error is release lag + flight error.

Measurements per throw (all from track B's converged descent fit, vacuum
g = 9806 mm/s^2 fixed; drag-induced extrapolation bias quantified by
simulation and reported separately):
  (i)   t_release = upward crossing of the MODEL release plane
        z_rel = initial_position.z + L_RELEASE*sin(pitch)  (the ballistic
        model's t=0 point; initial_position is the BB origin and the solver
        release point is l*sin(pitch) above it, throw_ballistics.py:315).
  (ii)  t_land    = downward crossing of landing_position.z.
  (iii) catch     = /cone/catch_event catch_time.

Components:
  release_lag  = t_release - throw_time
  flight_error = (t_land - t_release) - predicted_tof_sec
  fall_through = catch - t_land          (expect ~ -7 ms: piezo contact when
                                          ball CENTER is ~1 ballradius above
                                          the plane; more negative = track-B
                                          KF time lag)
Identity: sum == catch - landing_time == arrival_error exactly (intermediate
times cancel); the split is the information, not the sum.

Lag correction: track B carries a single time lag delta (KF convergence);
delta = fall_through_true(-6.7 ms) - fall_through_meas. Both t_release and
t_land shift by delta, so flight_error is lag-immune and
release_lag_corr = release_lag + (fall_through_meas - fall_through_true).
i.e. release_lag_corr = arrival_error - flight_error - fall_through_true.

Track-A numbers are retained for reference, explicitly labelled prior-biased.

Interpreter: /home/jetson/Desktop/PDJ_venv/venv/bin/python3 (3.8.10) — the
only python on this machine with mcap+rosbags (same stack as
accuracy_testing/extract_targets_from_mcap.py; /usr/bin/python3 lacks mcap).
"""

import json
import math
from pathlib import Path

import numpy as np
from mcap.reader import make_reader
from rosbags.typesys import get_typestore, get_types_from_msg, Stores

# ── Configuration ───────────────────────────────────────────────────────────

BAG = Path("/home/jetson/Desktop/rosbags/2026-06-10_22-24-02/2026-06-10_22-24-02_0.mcap")
MSGDIR = Path("/home/jetson/Desktop/Jugglebot/ros_ws/src/jugglebot_interfaces/msg")
OUT_JSON = Path(__file__).with_name("decompose_results.json")

G_MMPS2 = 9806.0           # hardware_config.py GRAVITY_MPS2 * 1000
L_RELEASE_MM = 150.0       # hardware_config.py BB_GEOM_RELEASE_L_POSITION_MM
BALL_RADIUS_MM = 35.0      # 70 mm ball
FALL_THROUGH_TRUE_MS = None  # computed per-throw: -BALL_RADIUS_MM / |vz_land|

WIN_BEFORE = 0.5
WIN_AFTER = 1.2
SETTLE_B_S = 0.12          # drop this much of track B after spawn (KF convergence)
B_FIT_ZBAND = (150.0, 1400.0)   # fit z in [z_land+150, z_land+1400] (avoid rim plateau)
B_FIT_MAXSPAN_S = 0.55     # cap fit window after spawn
SETTLE_A_S = 0.12
CATCH_WIN = (-0.3, 0.6)

REPEAT_T0 = 1781094300.0   # 22:25 repeatability session start (approx)
SWEEP_T0 = 1781094650.0    # 22:31 delay sweep start (approx)

# drag parameter k = 0.5*rho*Cd*A/m  [1/m]; 70 mm, 80 g ball, Cd=0.5 nominal
K_DRAG_NOMINAL = 0.5 * 1.2 * 0.5 * math.pi * 0.035 ** 2 / 0.080   # = 0.0144 /m


# ── MCAP reading (pattern from accuracy_testing/extract_targets_from_mcap.py) ──


def setup_typestore():
    store = get_typestore(Stores.ROS2_HUMBLE)
    for name in ["BallState", "BallStateArray", "ThrowAnnouncement", "CatchEvent"]:
        text = (MSGDIR / f"{name}.msg").read_text()
        store.register(get_types_from_msg(text, f"jugglebot_interfaces/msg/{name}"))
    return store


def t_of(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def read_bag(store):
    anns, catches = [], []
    bt, bid, bsrc = [], [], []
    bx, by, bz = [], [], []
    with open(BAG, "rb") as f:
        reader = make_reader(f)
        for schema, channel, message in reader.iter_messages(
            topics=["/throw_announcements", "/cone/catch_event", "/balls"]
        ):
            if channel.topic == "/throw_announcements":
                m = store.deserialize_cdr(message.data, "jugglebot_interfaces/msg/ThrowAnnouncement")
                anns.append(dict(
                    stamp=t_of(m.header.stamp),
                    throw_time=t_of(m.throw_time),
                    landing_time=t_of(m.landing_time),
                    tof=m.predicted_tof_sec,
                    p0=(m.initial_position.x, m.initial_position.y, m.initial_position.z),
                    v0=(m.initial_velocity.x, m.initial_velocity.y, m.initial_velocity.z),
                    pl=(m.landing_position.x, m.landing_position.y, m.landing_position.z),
                    vl=(m.landing_velocity.x, m.landing_velocity.y, m.landing_velocity.z),
                    target=m.target_id,
                ))
            elif channel.topic == "/cone/catch_event":
                m = store.deserialize_cdr(message.data, "jugglebot_interfaces/msg/CatchEvent")
                catches.append(dict(t=t_of(m.catch_time), seq=int(m.sequence),
                                    synced=bool(m.time_synced)))
            else:
                m = store.deserialize_cdr(message.data, "jugglebot_interfaces/msg/BallStateArray")
                for b in m.balls:
                    if b.tracking != 1 or b.status != 1:
                        continue  # skip ANNOUNCED placeholders / non-flight
                    bt.append(t_of(b.header.stamp))
                    bid.append(b.id)
                    bsrc.append(b.source)
                    bx.append(b.position.x)
                    by.append(b.position.y)
                    bz.append(b.position.z)
    balls = dict(t=np.asarray(bt), id=np.asarray(bid), src=np.asarray(bsrc, dtype=object),
                 x=np.asarray(bx), y=np.asarray(by), z=np.asarray(bz))
    order = np.argsort(balls["t"], kind="stable")
    for k in balls:
        balls[k] = balls[k][order]
    return anns, catches, balls


# ── Fitting helpers ─────────────────────────────────────────────────────────


def fit_gfixed(t, z, t0):
    """z + 0.5 g (t-t0)^2 = z0 + vz (t-t0): linear LSQ. Returns (z0, vz, rms)."""
    T = t - t0
    y = z + 0.5 * G_MMPS2 * T * T
    A = np.vstack([np.ones_like(T), T]).T
    coef, *_ = np.linalg.lstsq(A, y, rcond=None)
    resid = y - A @ coef
    return coef[0], coef[1], float(np.sqrt(np.mean(resid ** 2)))


def fit_gfree(t, z, t0):
    """z = a + b(t-t0) + c(t-t0)^2. Returns (a, b, c, rms); g_eff = -2c."""
    T = t - t0
    A = np.vstack([np.ones_like(T), T, T * T]).T
    coef, *_ = np.linalg.lstsq(A, z, rcond=None)
    resid = z - A @ coef
    return coef[0], coef[1], coef[2], float(np.sqrt(np.mean(resid ** 2)))


def cross_gfixed(z0, vz, t0, z_target, ascending):
    """Solve z0 + vz T - 0.5 g T^2 = z_target (T = t - t0)."""
    a, b, c = -0.5 * G_MMPS2, vz, z0 - z_target
    disc = b * b - 4 * a * c
    if disc < 0:
        return None
    r = math.sqrt(disc)
    roots = [(-b + r) / (2 * a), (-b - r) / (2 * a)]
    good = [T for T in roots if (vz - G_MMPS2 * T > 0) == ascending]
    if not good:
        return None
    return t0 + min(good, key=abs)


# ── Drag-bias simulation ────────────────────────────────────────────────────


def simulate_drag_bias(v0_mmps, sinp, z_rel, z_land, k_per_m):
    """Simulate a drag flight from the release plane; apply the same
    fit-vacuum-on-descent-band + extrapolate procedure; return biases (ms):
      bias_release: (fit t_release) - (true t_release=0)   [subtract from meas]
      bias_land:    (fit t_land) - (true t_land)
      drag_tof_delta: (true t_land with drag) - (vacuum t_land), i.e. expected
                      flight_error contribution of drag alone at commanded v.
    """
    k = k_per_m / 1000.0  # 1/mm
    cosp = math.sqrt(max(0.0, 1 - sinp * sinp))
    p = np.array([0.0, z_rel])
    v = np.array([v0_mmps * cosp, v0_mmps * sinp])
    dt = 5e-4
    ts, zs = [], []
    t = 0.0
    t_land_true = None
    while t < 3.0:
        s = np.linalg.norm(v)
        a = np.array([0.0, -G_MMPS2]) - k * s * v
        v = v + a * dt
        p = p + v * dt
        t += dt
        ts.append(t)
        zs.append(p[1])
        if v[1] < 0 and p[1] <= z_land and t_land_true is None:
            t_land_true = t
            break
    ts, zs = np.array(ts), np.array(zs)
    desc = (np.diff(zs, prepend=zs[0]) < 0)
    band = desc & (zs > z_land + B_FIT_ZBAND[0]) & (zs < z_land + B_FIT_ZBAND[1])
    tb, zb = ts[band][::10], zs[band][::10]   # ~140 Hz-ish
    t0 = float(np.mean(tb))
    z0, vz, _ = fit_gfixed(tb, zb, t0)
    t_rel_fit = cross_gfixed(z0, vz, t0, z_rel, ascending=True)
    t_land_fit = cross_gfixed(z0, vz, t0, z_land, ascending=False)
    # vacuum t_land from the true release state
    t_land_vac = cross_gfixed(z_rel, v0_mmps * sinp, 0.0, z_land, ascending=False)
    return ((t_rel_fit - 0.0) * 1e3,
            (t_land_fit - t_land_true) * 1e3,
            (t_land_true - t_land_vac) * 1e3)


# ── Per-throw analysis ──────────────────────────────────────────────────────


def analyse_throw(ann, balls, catches):
    tt, lt, tof = ann["throw_time"], ann["landing_time"], ann["tof"]
    p0, v0 = np.array(ann["p0"]), np.array(ann["v0"])
    z_land = ann["pl"][2]
    vz_land = ann["vl"][2]
    speed0 = float(np.linalg.norm(v0))
    sinp = v0[2] / speed0
    z_rel = p0[2] + L_RELEASE_MM * sinp

    res = dict(flags=[], throw_time=tt, landing_time=lt, predicted_tof_s=tof,
               commanded_delay_s=tt - ann["stamp"],
               z_release_plane_mm=z_rel, z_land_plane_mm=z_land,
               commanded_speed_mmps=speed0)

    m = (balls["t"] >= tt - WIN_BEFORE) & (balls["t"] <= lt + WIN_AFTER)
    t, ids, src = balls["t"][m], balls["id"][m], balls["src"][m]
    P = np.vstack([balls["x"][m], balls["y"][m], balls["z"][m]]).T
    if len(t) == 0:
        res["flags"].append("no_ball_samples")
        return res

    # ---- segment classification by id+source ----
    segA = segB = None
    for uid in np.unique(ids):
        sm = ids == uid
        ts, Ps = t[sm], P[sm]
        s0 = src[sm][0]
        if len(ts) < 12:
            continue
        tf = ts[0]
        if s0 == "ball_butler" and tf - tt < 0.3 and segA is None:
            segA = (ts, Ps)
        elif s0 == "human_throw" and 0.25 <= tf - tt <= (lt - tt) + 0.55 \
                and Ps[0, 2] > z_land + 300:
            if segB is None or len(ts) > len(segB[0]):
                segB = (ts, Ps)
    res["has_trackA"] = segA is not None
    res["has_trackB"] = segB is not None

    # ---- catch ----
    cand = [c for c in catches if lt + CATCH_WIN[0] <= c["t"] <= lt + CATCH_WIN[1]]
    res["n_catch_in_window"] = len(cand)
    catch_t = cand[0]["t"] if cand else None
    if catch_t is None:
        res["flags"].append("no_catch_event")
    else:
        res["arrival_error_ms"] = (catch_t - lt) * 1e3

    # ---- PRIMARY: track-B fit (measurement-driven re-detection) ----
    if segB is not None:
        ts, Ps = segB
        keep = (ts >= ts[0] + SETTLE_B_S) & (ts <= ts[0] + B_FIT_MAXSPAN_S) \
            & (Ps[:, 2] > z_land + B_FIT_ZBAND[0]) & (Ps[:, 2] < z_land + B_FIT_ZBAND[1])
        if keep.sum() >= 10:
            tb, Pb = ts[keep], Ps[keep]
            t0 = float(np.mean(tb))
            z0, vz, rms = fit_gfixed(tb, Pb[:, 2], t0)
            _, _, cfree, _ = fit_gfree(tb, Pb[:, 2], t0)
            vx = np.polyfit(tb - t0, Pb[:, 0], 1)[0]
            vy = np.polyfit(tb - t0, Pb[:, 1], 1)[0]
            res["B_fit"] = dict(n=int(keep.sum()),
                                t_span_rel=(round(tb[0] - tt, 3), round(tb[-1] - tt, 3)),
                                rms_mm=rms, g_eff_free=-2 * cfree,
                                spawn_rel_s=round(ts[0] - tt, 3))
            t_rel_B = cross_gfixed(z0, vz, t0, z_rel, ascending=True)
            t_land_B = cross_gfixed(z0, vz, t0, z_land, ascending=False)
            if t_rel_B is not None and t_land_B is not None:
                res["release_lag_B_ms"] = (t_rel_B - tt) * 1e3
                res["flight_error_B_ms"] = ((t_land_B - t_rel_B) - tof) * 1e3
                res["t_land_B_rel_landing_ms"] = (t_land_B - lt) * 1e3
                vz_at_rel = vz - G_MMPS2 * (t_rel_B - t0)
                res["meas_speed_B_mmps"] = float(math.sqrt(vx * vx + vy * vy + vz_at_rel ** 2))
                res["meas_vz_release_B_mmps"] = float(vz_at_rel)
                res["meas_vh_B_mmps"] = float(math.hypot(vx, vy))
                res["cmd_vh_mmps"] = float(math.hypot(v0[0], v0[1]))
                res["cmd_vz_mmps"] = float(v0[2])
                if catch_t is not None:
                    res["fall_through_B_ms"] = (catch_t - t_land_B) * 1e3
                    # piezo-anchored lag correction (see module docstring)
                    ft_true = -BALL_RADIUS_MM / abs(vz_land) * 1e3
                    res["fall_through_true_ms_assumed"] = ft_true
                    res["trackB_lag_ms"] = ft_true - res["fall_through_B_ms"]
                    res["release_lag_corr_ms"] = (res["arrival_error_ms"]
                                                  - res["flight_error_B_ms"] - ft_true)
                    res["sum_B_ms"] = (res["release_lag_B_ms"] + res["flight_error_B_ms"]
                                       + res["fall_through_B_ms"])
                    res["residual_B_ms"] = res["arrival_error_ms"] - res["sum_B_ms"]
            else:
                res["flags"].append("B_crossing_solve_failed")
        else:
            res["flags"].append("B_fit_band_too_sparse")
    else:
        res["flags"].append("no_trackB")

    # ---- REFERENCE: track-A (prior-biased announcement-matched KF) ----
    if segA is not None:
        ts, Ps = segA
        keep = ts >= ts[0] + SETTLE_A_S
        if keep.sum() >= 15:
            ta, za = ts[keep], Ps[keep, 2]
            t0 = float(np.mean(ta))
            a, b, c, rms = fit_gfree(ta, za, t0)
            if c < 0:
                disc = b * b - 4 * c * (a - z_rel)
                if disc >= 0:
                    r = math.sqrt(disc)
                    roots = [(-b + r) / (2 * c), (-b - r) / (2 * c)]
                    good = [T for T in roots if b + 2 * c * T > 0]
                    if good:
                        res["release_lag_A_ms_PRIORBIASED"] = \
                            (t0 + min(good, key=abs) - tt) * 1e3
            res["A_fit"] = dict(n=int(keep.sum()), rms_mm=rms, g_eff_free=-2 * c)
    return res


# ── Aggregation ─────────────────────────────────────────────────────────────


def stats(vals):
    v = np.asarray([x for x in vals if x is not None and np.isfinite(x)])
    if len(v) == 0:
        return None
    return dict(n=int(len(v)), mean=float(np.mean(v)),
                std=float(np.std(v, ddof=1)) if len(v) > 1 else 0.0,
                min=float(np.min(v)), max=float(np.max(v)))


def main():
    store = setup_typestore()
    print("Reading bag (takes ~1 min)...", flush=True)
    anns, catches, balls = read_bag(store)
    print(f"announcements={len(anns)} catches={len(catches)} "
          f"confirmed-flight ball samples={len(balls['t'])}")

    results = []
    for i, ann in enumerate(anns):
        r = analyse_throw(ann, balls, catches)
        r["idx"] = i
        tt = ann["throw_time"]
        r["session"] = ("warmup" if tt < REPEAT_T0 else
                        "repeatability" if tt < SWEEP_T0 else "delay_sweep")
        results.append(r)

    print("\nidx sess  rel_lagB flt_errB fallB  lagB  rel_corr  arr_err  resid "
          "v_cmd  v_measB g_effB relA(bias) flags")
    for r in results:
        def g(k, f="{:8.1f}"):
            return f.format(r[k]) if k in r and r[k] is not None else "    --- "
        print(f"{r['idx']:3d} {r['session'][:5]:5s}"
              f"{g('release_lag_B_ms')}{g('flight_error_B_ms')}{g('fall_through_B_ms')}"
              f"{g('trackB_lag_ms')}{g('release_lag_corr_ms')}{g('arrival_error_ms')}"
              f"{g('residual_B_ms')}"
              f"{g('commanded_speed_mmps', '{:7.0f}')}{g('meas_speed_B_mmps', '{:7.0f}')}"
              f"{('{:7.0f}'.format(r['B_fit']['g_eff_free']) if 'B_fit' in r else '    ---')}"
              f"{g('release_lag_A_ms_PRIORBIASED', '{:8.1f}')}"
              f"  {','.join(r['flags']) if r['flags'] else ''}")

    keys = ["release_lag_B_ms", "flight_error_B_ms", "fall_through_B_ms",
            "trackB_lag_ms", "release_lag_corr_ms", "arrival_error_ms",
            "residual_B_ms", "t_land_B_rel_landing_ms",
            "release_lag_A_ms_PRIORBIASED"]
    agg = {}
    print("\n=== AGGREGATE (valid throws) ===")
    for k in keys:
        st = stats([r.get(k) for r in results])
        agg[k] = st
        if st:
            print(f"{k:28s} n={st['n']:2d} mean={st['mean']:+8.2f} σ={st['std']:6.2f} "
                  f"[{st['min']:+8.2f},{st['max']:+8.2f}]")
    geff = stats([r["B_fit"]["g_eff_free"] for r in results if "B_fit" in r])
    agg["g_eff_free_B"] = geff
    if geff:
        print(f"{'g_eff_free_B (mm/s^2)':28s} n={geff['n']:2d} mean={geff['mean']:+8.1f} "
              f"σ={geff['std']:6.1f}")

    for sess in ("repeatability", "delay_sweep"):
        sub = [r for r in results if r["session"] == sess]
        print(f"\n=== {sess} only ===")
        for k in ["release_lag_corr_ms", "flight_error_B_ms", "arrival_error_ms"]:
            st = stats([r.get(k) for r in sub])
            if st:
                print(f"{k:28s} n={st['n']:2d} mean={st['mean']:+8.2f} σ={st['std']:6.2f}")

    # speed comparison
    pairs = [(r["meas_speed_B_mmps"], r["commanded_speed_mmps"]) for r in results
             if "meas_speed_B_mmps" in r]
    if pairs:
        ratio = np.array([a / b for a, b in pairs])
        print(f"\nspeed at release (track B): meas/cmd mean={np.mean(ratio):.4f} "
              f"σ={np.std(ratio, ddof=1):.4f} (n={len(ratio)})")
        vzr = np.array([r["meas_vz_release_B_mmps"] / r["cmd_vz_mmps"] for r in results
                        if "meas_vz_release_B_mmps" in r])
        vhr = np.array([r["meas_vh_B_mmps"] / r["cmd_vh_mmps"] for r in results
                        if "meas_vh_B_mmps" in r])
        print(f"vz at release: meas/cmd mean={np.mean(vzr):.4f} σ={np.std(vzr, ddof=1):.4f}")
        print(f"vh           : meas/cmd mean={np.mean(vhr):.4f} σ={np.std(vhr, ddof=1):.4f}")
        agg["speed_ratio"] = dict(mean=float(np.mean(ratio)), std=float(np.std(ratio, ddof=1)),
                                  n=int(len(ratio)))

    # drag-bias simulation (nominal throw geometry from first repeatability throw)
    ref = next(r0 for r0, a0 in zip(results, anns) if r0["session"] == "repeatability")
    a0 = anns[ref["idx"]]
    v0 = np.array(a0["v0"])
    speed0 = float(np.linalg.norm(v0))
    sinp = v0[2] / speed0
    print("\n=== drag simulation (procedure bias + expected drag flight error) ===")
    print("k[1/m]  Cd~   bias_release_ms  bias_tland_ms  drag_tof_delta_ms")
    sim_out = {}
    for k_drag, cd in [(0.00865, 0.3), (K_DRAG_NOMINAL, 0.5), (0.0216, 0.75)]:
        br, bl, dd = simulate_drag_bias(speed0, sinp, ref["z_release_plane_mm"],
                                        ref["z_land_plane_mm"], k_drag)
        print(f"{k_drag:.4f}  {cd:.2f}  {br:+12.2f}  {bl:+12.2f}  {dd:+12.2f}")
        sim_out[f"Cd_{cd}"] = dict(k_per_m=k_drag, bias_release_ms=br,
                                   bias_tland_ms=bl, drag_tof_delta_ms=dd)

    out = dict(bag=str(BAG), n_announcements=len(anns), n_catches=len(catches),
               method_notes=__doc__, aggregate=agg, drag_simulation=sim_out,
               throws=results)
    OUT_JSON.write_text(json.dumps(out, indent=1, default=float))
    print(f"\nSaved per-throw results to {OUT_JSON}")


if __name__ == "__main__":
    main()
