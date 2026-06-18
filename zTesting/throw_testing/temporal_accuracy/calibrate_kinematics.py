#!/home/jetson/Desktop/PDJ_venv/venv/bin/python3
"""
BB kinematic system-ID: solve for the small parameter offsets that best explain
the measured launch + landing, replacing the empirical 2D aim-correction with
physically-correct kinematics (which should collapse the temporal δ too).

Two-stage fit (see logbook 2026-06-17 launch-discrepancy):

  STAGE 1 — execution errors from LAUNCH VELOCITY (decoupled & linear).
    The mocap launch vector decouples the problem into three 1-D regressions, so
    each parameter is directly read off a slope/intercept (no optimiser, no
    cross-coupling) — and the slope-vs-intercept split is exactly the
    "constant offset (stable) vs scale/linkage (pose-dependent)" verdict:
      launch elevation φ = atan2(vz, vh)   vs commanded pitch → pitch_scale, pitch_off
      launch azimuth   ψ = atan2(vy, vx)   vs commanded yaw   → yaw_scale,   yaw_off
      launch speed     |v|                 vs commanded speed → speed_scale, speed_off
    Needs DIVERSITY in each commanded axis (range of pitch via heights/distances,
    yaw spread around the workspace, speed range) — quantified by the synthetic
    identifiability test below.

  STAGE 2 — release geometry (d, l, z) from LANDING POSITION (nonlinear LSQ).
    Given Stage-1 launch velocities, the residual landing error pins the release
    offsets. Uses scipy.least_squares.

Run the self-test (no hardware/bag needed) to validate the fitter end-to-end:
    calibrate_kinematics.py --synthetic
It injects known offsets, simulates noisy measurements over a diverse workspace,
and reports recovery + 1σ uncertainties (the identifiability story).

Real use (after the workspace session): build the per-throw observation list from
the bag (commanded yaw/pitch/speed from announcements; measured launch from
analyze_launch_velocity; landing from mocap track-B; encoder pitch/hand from
analyze_during_throw) and call fit_stage1 / fit_stage2.
"""
import argparse
import math

import numpy as np

G_MMPS2 = 9806.0

# Nominal BB geometry — MUST match config/hardware_config.yaml ball_butler_geometry
# (the values throw_ballistics.py actually uses), so the fitted *_off are corrections
# to the deployed config. NOTE on z: throw_ballistics computes release height as
# bb_position.z + l·sin(pitch); pitch_z_offset (17.5) is baked into the calibrated
# bb_position.z, NOT re-added at throw time. So z_off here is a correction to the
# release-z ORIGIN (calibration), and forward() must NOT add a nominal z (see below).
NOMINAL = dict(d_mm=41.0, l_mm=150.0, s_mm=-105.65, z_mm=17.5)


# ── Parameter vector ─────────────────────────────────────────────────────────
# Θ = execution errors + geometry offsets. Defaults = "no error" (identity).
PARAM_NAMES = ["pitch_scale", "pitch_off_deg", "yaw_scale", "yaw_off_deg",
               "speed_scale", "speed_off_mps", "d_off_mm", "l_off_mm", "z_off_mm", "s_off_mm"]


def theta_identity():
    return dict(pitch_scale=1.0, pitch_off_deg=0.0, yaw_scale=1.0, yaw_off_deg=0.0,
                speed_scale=1.0, speed_off_mps=0.0,
                d_off_mm=0.0, l_off_mm=0.0, z_off_mm=0.0, s_off_mm=0.0)


# ── Forward model ────────────────────────────────────────────────────────────
def forward(cmd_yaw, cmd_pitch, cmd_speed, th, bb_xyz, yaw_offset, catch_z):
    """Commanded (yaw,pitch,speed) + Θ → actual launch velocity, release point,
    landing position, tof. Faithful to throw_ballistics conventions (R = A −
    l·cos p + d; release height l·sin p above the pitch-axis z plane; lateral s)."""
    pitch = th["pitch_scale"] * cmd_pitch + math.radians(th["pitch_off_deg"])
    yaw_l = th["yaw_scale"] * cmd_yaw + math.radians(th["yaw_off_deg"])
    speed = th["speed_scale"] * cmd_speed + th["speed_off_mps"]
    thg = yaw_l + yaw_offset                                   # global throw direction
    d = NOMINAL["d_mm"] + th["d_off_mm"]
    l = NOMINAL["l_mm"] + th["l_off_mm"]
    s = NOMINAL["s_mm"] + th["s_off_mm"]

    cp, sp = math.cos(pitch), math.sin(pitch)
    vh = speed * cp * 1000.0
    vz = speed * sp * 1000.0
    vx = vh * math.cos(thg)
    vy = vh * math.sin(thg)

    # release point (global mm): forward along throw dir + lateral s
    fwd = l * cp - d
    ux, uy = math.cos(thg), math.sin(thg)
    lx_hat, ly_hat = -math.sin(thg), math.cos(thg)
    rx = bb_xyz[0] + fwd * ux + s * lx_hat
    ry = bb_xyz[1] + fwd * uy + s * ly_hat
    # pitch_z_offset is baked into bb_xyz[2] (the calibrated origin); z_off_mm is a
    # residual correction to that origin, NOT an additional nominal offset.
    rz = bb_xyz[2] + th["z_off_mm"] + l * sp

    # ballistic to catch_z (descending crossing)
    dz = catch_z - rz
    disc = vz * vz - 2.0 * G_MMPS2 * dz
    if disc < 0:
        return None
    tof = (vz + math.sqrt(disc)) / G_MMPS2
    if tof <= 0:
        return None
    return dict(v=(vx, vy, vz), rel=(rx, ry, rz),
                land=(rx + vx * tof, ry + vy * tof, catch_z), tof=tof)


# ── Stage 1: decoupled linear regressions on launch velocity ─────────────────
def _linfit(x, y):
    """y = m x + b least squares; returns (m, b, m_se, b_se, r2)."""
    x = np.asarray(x, float); y = np.asarray(y, float)
    n = len(x)
    A = np.vstack([x, np.ones_like(x)]).T
    coef, *_ = np.linalg.lstsq(A, y, rcond=None)
    m, b = coef
    resid = y - A @ coef
    dof = max(n - 2, 1)
    s2 = float(resid @ resid) / dof
    cov = s2 * np.linalg.inv(A.T @ A)
    ss_tot = float(((y - y.mean()) ** 2).sum()) or 1.0
    r2 = 1.0 - float(resid @ resid) / ss_tot
    return m, b, math.sqrt(cov[0, 0]), math.sqrt(cov[1, 1]), r2


def fit_stage1(obs, yaw_offset):
    """obs: list of dict(cmd_yaw, cmd_pitch, cmd_speed, vx, vy, vz) [vel in mm/s].
    Returns the execution params with 1σ and the offset-vs-scale verdict."""
    cmd_pitch = [o["cmd_pitch"] for o in obs]
    cmd_yaw = [o["cmd_yaw"] for o in obs]
    cmd_speed = [o["cmd_speed"] for o in obs]
    phi = [math.atan2(o["vz"], math.hypot(o["vx"], o["vy"])) for o in obs]      # elevation
    psi = [math.atan2(o["vy"], o["vx"]) - yaw_offset for o in obs]             # local azimuth
    psi = [(p + math.pi) % (2 * math.pi) - math.pi for p in psi]
    spd = [math.sqrt(o["vx"] ** 2 + o["vy"] ** 2 + o["vz"] ** 2) / 1000.0 for o in obs]

    mP, bP, mPse, bPse, r2P = _linfit(cmd_pitch, phi)        # φ = mP·pitch + bP
    mY, bY, mYse, bYse, r2Y = _linfit(cmd_yaw, psi)
    mS, bS, mSse, bSse, r2S = _linfit(cmd_speed, spd)
    return dict(
        pitch_scale=(mP, mPse), pitch_off_deg=(math.degrees(bP), math.degrees(bPse)), r2_pitch=r2P,
        yaw_scale=(mY, mYse), yaw_off_deg=(math.degrees(bY), math.degrees(bYse)), r2_yaw=r2Y,
        speed_scale=(mS, mSse), speed_off_mps=(bS, bSse), r2_speed=r2S)


# ── Stage 2: geometry from RELEASE position residual (nonlinear LSQ) ───────────
def fit_stage2(obs, th_stage1, bb_xyz, yaw_offset):
    """Fit release-geometry offsets [d, l, z, s] against the mocap-measured
    RELEASE position (rel_x/y/z, back-extrapolated from each arc), holding the
    Stage-1 execution params fixed. Release position constrains geometry far more
    directly than landing — that is the big win of throwing cone-free (the arc is
    measured right from release). `catch_z` is unused here (kept for API parity)."""
    from scipy.optimize import least_squares
    base = theta_identity(); base.update(th_stage1)
    catch_z = bb_xyz[2]  # any plane; release position doesn't depend on it

    def resid(p):
        th = dict(base)
        th["d_off_mm"], th["l_off_mm"], th["z_off_mm"], th["s_off_mm"] = p
        r = []
        for o in obs:
            f = forward(o["cmd_yaw"], o["cmd_pitch"], o["cmd_speed"], th,
                        bb_xyz, yaw_offset, catch_z)
            if f is None:
                r += [1e4, 1e4, 1e4]; continue
            r += [f["rel"][0] - o["rel_x"], f["rel"][1] - o["rel_y"], f["rel"][2] - o["rel_z"]]
        return r

    sol = least_squares(resid, [0.0, 0.0, 0.0, 0.0], method="lm")
    return dict(d_off_mm=sol.x[0], l_off_mm=sol.x[1], z_off_mm=sol.x[2], s_off_mm=sol.x[3],
                cost=float(sol.cost), rms_mm=math.sqrt(2 * sol.cost / (3 * len(obs))))


# ── Joint fit: all execution + geometry params (+ release-time δ) ──────────────
def fit_joint(obs, bb_xyz, yaw_offset, fit_delta=True, sigma_pos=10.0, sigma_vel=50.0,
              geom_prior_mm=40.0):
    """Single joint fit of ALL parameters to BOTH the measured release POSITION and
    VELOCITY — the spatial+temporal-together fit (vs. the sequential stage1→stage2).

    Each throw's measured state at the commanded throw_time is propagated by δ along
    its free-flight parabola before comparison, so δ absorbs residual release-lag and
    any constant command-vs-mocap clock skew (the temporal term). δ is separable from
    speed_off given PITCH diversity: δ shifts vz by g·δ uniformly, whereas speed_off
    shifts it by sin(pitch)·speed_off.

    Residuals are whitened by σ_pos (mm) and σ_vel (mm/s) so the LSQ balances geometry
    (position) against launch direction/speed (velocity). Returns each param with a 1σ
    from the Jacobian, the release-time δ (ms), and the split position/velocity RMS.
    """
    from scipy.optimize import least_squares
    G = G_MMPS2
    names = list(PARAM_NAMES) + (["delta_s"] if fit_delta else [])
    x0 = [theta_identity()[nm] for nm in PARAM_NAMES] + ([0.0] if fit_delta else [])
    idx = {nm: i for i, nm in enumerate(names)}

    def unpack(p):
        th = theta_identity()
        for nm in PARAM_NAMES:
            th[nm] = p[idx[nm]]
        return th, (p[idx["delta_s"]] if fit_delta else 0.0)

    def resid(p):
        th, dlt = unpack(p)
        r = []
        for o in obs:
            f = forward(o["cmd_yaw"], o["cmd_pitch"], o["cmd_speed"], th,
                        bb_xyz, yaw_offset, bb_xyz[2])
            if f is None:
                r += [1e3] * 6
                continue
            # propagate measured (at throw_time) by δ along the parabola → true release
            mvx, mvy, mvz = o["vx"], o["vy"], o["vz"]
            mrx = o["rel_x"] + mvx * dlt
            mry = o["rel_y"] + mvy * dlt
            mrz = o["rel_z"] + mvz * dlt - 0.5 * G * dlt * dlt
            mvz -= G * dlt
            pvx, pvy, pvz = f["v"]
            prx, pry, prz = f["rel"]
            r += [(prx - mrx) / sigma_pos, (pry - mry) / sigma_pos, (prz - mrz) / sigma_pos,
                  (pvx - mvx) / sigma_vel, (pvy - mvy) / sigma_vel, (pvz - mvz) / sigma_vel]
        if geom_prior_mm:        # L2 prior toward CAD/calib geometry — curbs runaway when
            r += [th[k] / geom_prior_mm for k in   # azimuth/pitch diversity is thin
                  ("d_off_mm", "l_off_mm", "z_off_mm", "s_off_mm")]
        return r

    sol = least_squares(resid, x0, method="lm")
    try:                                   # 1σ from whitened Jacobian: cov ≈ (JᵀJ)⁻¹
        se = np.sqrt(np.clip(np.diag(np.linalg.inv(sol.jac.T @ sol.jac)), 0, None))
    except np.linalg.LinAlgError:
        se = np.full(len(sol.x), float("nan"))
    per = np.asarray(resid(sol.x))[:6 * len(obs)].reshape(-1, 6)   # per-throw rows only
    ok = np.abs(per).max(axis=1) < 100.0   # drop forward()=None penalty rows from RMS
    out = {nm: (float(sol.x[idx[nm]]), float(se[idx[nm]])) for nm in names}
    out["delta_ms"] = ((out["delta_s"][0] * 1e3, out["delta_s"][1] * 1e3) if fit_delta else (0.0, 0.0))
    out["pos_rms_mm"] = float(np.sqrt((per[ok, 0:3] ** 2).mean()) * sigma_pos)
    out["vel_rms_mmps"] = float(np.sqrt((per[ok, 3:6] ** 2).mean()) * sigma_vel)
    out["n"] = int(ok.sum())
    return out


# ── Synthetic identifiability test ────────────────────────────────────────────
def synthetic(n=48, seed=42, ang_noise_deg=0.6, speed_noise_pct=1.5, rel_noise_mm=6.0,
              diversity="full", delta_true=0.012):
    rng = np.random.RandomState(seed)
    bb_xyz = (0.0, 0.0, 1000.0); yaw_offset = math.radians(20.0)
    th_true = theta_identity()
    th_true.update(pitch_off_deg=3.1, speed_scale=1.10, yaw_off_deg=0.8,
                   d_off_mm=8.0, l_off_mm=-12.0, z_off_mm=6.0, s_off_mm=5.0)

    # commanded diversity (the experiment design knob)
    if diversity == "full":
        pit = np.radians(rng.uniform(42, 80, n)); yaw = np.radians(rng.uniform(-55, 55, n))
        spd = rng.uniform(2.6, 4.4, n)
    elif diversity == "narrow_pitch":          # ablation: all throws near one pitch
        pit = np.radians(rng.uniform(69, 73, n)); yaw = np.radians(rng.uniform(-55, 55, n))
        spd = rng.uniform(2.6, 4.4, n)
    else:
        raise ValueError(diversity)

    obs = []
    for i in range(n):
        catch_z = bb_xyz[2] + rng.uniform(-400, 200)            # heights spread
        f = forward(yaw[i], pit[i], spd[i], th_true, bb_xyz, yaw_offset, catch_z)
        if f is None:
            continue
        rvx, rvy, rvz = f["v"]; rrx, rry, rrz = f["rel"]      # TRUE release state
        # The extractor measures at the commanded throw_time = release − delta_true;
        # back-propagate the release state by delta_true along the parabola.
        vx, vy, vz = rvx, rvy, rvz + G_MMPS2 * delta_true
        rx0 = rrx - rvx * delta_true
        ry0 = rry - rvy * delta_true
        rz0 = rrz - rvz * delta_true - 0.5 * G_MMPS2 * delta_true ** 2
        # measurement noise: rotate launch vector by angle noise, scale by speed noise
        sp_n = 1.0 + rng.normal(0, speed_noise_pct / 100.0)
        dphi = math.radians(rng.normal(0, ang_noise_deg)); dpsi = math.radians(rng.normal(0, ang_noise_deg))
        speed = math.hypot(math.hypot(vx, vy), vz) * sp_n
        phi = math.atan2(vz, math.hypot(vx, vy)) + dphi
        psi = math.atan2(vy, vx) + dpsi
        vxm = speed * math.cos(phi) * math.cos(psi)
        vym = speed * math.cos(phi) * math.sin(psi)
        vzm = speed * math.sin(phi)
        obs.append(dict(cmd_yaw=yaw[i], cmd_pitch=pit[i], cmd_speed=spd[i],
                        vx=vxm, vy=vym, vz=vzm, catch_z=catch_z,
                        rel_x=rx0 + rng.normal(0, rel_noise_mm),
                        rel_y=ry0 + rng.normal(0, rel_noise_mm),
                        rel_z=rz0 + rng.normal(0, rel_noise_mm)))
    return obs, th_true, bb_xyz, yaw_offset, delta_true


def _report(tag, obs, th_true, bb_xyz, yaw_offset, delta_true=0.0):
    print(f"\n===== {tag}: n={len(obs)} throws =====")
    s1 = fit_stage1(obs, yaw_offset)
    def line(name, est, true, unit):
        v, se = est
        hit = abs(v - true) <= 2 * se + 1e-9
        print(f"  {name:14s} fit={v:+8.3f} ±{se:5.3f} {unit:5s} true={true:+8.3f}  "
              f"{'OK' if hit else 'OUT'}")
    print(" STAGE 1 (launch velocity → execution errors):")
    line("pitch_scale", s1["pitch_scale"], th_true["pitch_scale"], "")
    line("pitch_off", s1["pitch_off_deg"], th_true["pitch_off_deg"], "deg")
    line("yaw_scale", s1["yaw_scale"], th_true["yaw_scale"], "")
    line("yaw_off", s1["yaw_off_deg"], th_true["yaw_off_deg"], "deg")
    line("speed_scale", s1["speed_scale"], th_true["speed_scale"], "")
    line("speed_off", s1["speed_off_mps"], th_true["speed_off_mps"], "m/s")
    print(f"   R²: pitch={s1['r2_pitch']:.4f} yaw={s1['r2_yaw']:.4f} speed={s1['r2_speed']:.4f}")
    th1 = dict(pitch_scale=s1["pitch_scale"][0], pitch_off_deg=s1["pitch_off_deg"][0],
               yaw_scale=s1["yaw_scale"][0], yaw_off_deg=s1["yaw_off_deg"][0],
               speed_scale=s1["speed_scale"][0], speed_off_mps=s1["speed_off_mps"][0])
    s2 = fit_stage2(obs, th1, bb_xyz, yaw_offset)
    print(" STAGE 2 (release-position residual → release geometry):")
    for k in ("d_off_mm", "l_off_mm", "z_off_mm", "s_off_mm"):
        hit = abs(s2[k] - th_true[k]) <= 6.0
        print(f"  {k:14s} fit={s2[k]:+8.2f} mm     true={th_true[k]:+8.2f}  {'OK' if hit else 'OUT'}")
    print(f"   release-fit residual RMS = {s2['rms_mm']:.1f} mm")

    j = fit_joint(obs, bb_xyz, yaw_offset, fit_delta=True)
    print(" JOINT (all params + release δ, fit to position AND velocity):")
    for k in PARAM_NAMES:
        v, se = j[k]; unit = "deg" if k.endswith("_deg") else ("mm" if k.endswith("_mm") else "")
        hit = abs(v - th_true[k]) <= 2 * se + (0.02 if "scale" in k else 1.0)
        print(f"  {k:14s} fit={v:+8.3f} ±{se:6.3f} {unit:3s} true={th_true[k]:+8.3f}  {'OK' if hit else 'OUT'}")
    dms, dse = j["delta_ms"]
    print(f"  {'delta_ms':14s} fit={dms:+8.2f} ±{dse:6.2f} ms  true={delta_true*1000:+8.2f}")
    print(f"   joint RMS: position={j['pos_rms_mm']:.1f} mm  velocity={j['vel_rms_mmps']:.1f} mm/s")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--synthetic", action="store_true", help="run the identifiability self-test")
    args = ap.parse_args()
    if args.synthetic:
        obs, tt, bb, yo, dt = synthetic(diversity="full")
        _report("FULL workspace diversity", obs, tt, bb, yo, dt)
        obs2, tt2, bb2, yo2, dt2 = synthetic(diversity="narrow_pitch")
        _report("ABLATION: narrow pitch (all ~71°)", obs2, tt2, bb2, yo2, dt2)
        print("\nTakeaway: pitch_scale vs pitch_off are only separable with PITCH DIVERSITY; "
              "the narrow-pitch run should blow up pitch_scale's σ (can't tell offset from "
              "scale) — hence the spread-around-the-workspace experiment design.")
    else:
        print("Provide --synthetic, or import fit_stage1/fit_stage2 with real observations.")


if __name__ == "__main__":
    main()
