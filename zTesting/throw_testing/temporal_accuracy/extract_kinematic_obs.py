#!/home/jetson/Desktop/PDJ_venv/venv/bin/python3
"""
Build kinematic-ID observations from a cone-free grid session, then fit.

Reads a run_kinematic_id_session.py session JSON (commanded yaw/pitch/speed +
est_throw_wall_ns per throw) and the matching rosbag (/balls mocap arcs;
/bb/axis_estimates encoder pitch/hand). For each throw it fits the launch arc
(track A early-mid, measurement-driven — see analyze_launch_velocity) and
evaluates the launch state AT the commanded throw_time (the firmware now releases
at throw_time, post release-lag fix), giving:
  launch velocity (vx,vy,vz)  → Stage-1 execution errors (pitch/yaw/speed)
  release position (rel_x/y/z) → Stage-2 geometry
plus encoder pitch/hand for physical attribution.

Stage-1 pitch & speed regressions need NEITHER bb_xyz nor yaw_offset (they read
straight off elevation-vs-pitch and speed-vs-speed) — so the headline result is
available even without calibration. Pass --bb-xyz / --yaw-offset-deg (from BB
calibration) to also fit azimuth and the Stage-2 geometry.

PROVISIONAL: the arc-window/gates below mirror analyze_launch_velocity, which
needed a real-data tune for track segmentation — expect to confirm/adjust on the
first real grid bag (g_eff/RMS/n diagnostics are printed per throw to guide that).

    extract_kinematic_obs.py <bag.mcap> <session.json> [--bb-xyz x,y,z] [--yaw-offset-deg D]
"""
import argparse
import json
import math
import mmap
from pathlib import Path

import numpy as np
from rosbags.typesys import get_typestore, Stores

import analyze_velocity_postfix as AV      # iter_records/inflate/u16/u32/u64
import decompose_from_bag as D             # setup_typestore, fit_gfixed, t_of
import calibrate_kinematics as CK

G = D.G_MMPS2
A_WIN = (0.10, 0.55)      # track-A early-mid measurement-driven window (s after throw_time)
N_MIN = 25
YAW_VALID_DEG = (0.0, 185.0)   # BB physical yaw axis range; commanded yaw outside this is clamped
# Source tag to require, or None to accept any in-window sample. The cone-free
# kinematic session drives raw bb/send_throw_command with NO /throw_announcements,
# so the tracker can't attribute balls to BB and labels them 'human_throw'. The
# per-throw time window (+ the g_eff/RMS/parabola gates) isolate BB's arc on its
# own, so we don't filter by source here.
SRC_FILTER = None
GEFF_OK = (9000.0, 10600.0)
RMS_MAX = 8.0


def read_bag(bag, store, jt_store):
    chan = {}
    bt, bsrc, bx, by, bz = [], [], [], [], []
    et, e_pitch, e_hvel = [], [], []
    BSA = "jugglebot_interfaces/msg/BallStateArray"
    JT = "sensor_msgs/msg/JointState"

    def handle_channel(c):
        chan[AV.u16(c, 0)] = bytes(c[8:8 + AV.u32(c, 4)]).decode("utf-8", "replace")

    def handle_message(c):
        topic = chan.get(AV.u16(c, 0))
        if topic == "/balls":
            m = store.deserialize_cdr(bytes(c[22:]), BSA)
            for b in m.balls:
                if b.tracking != 1 or b.status != 1:
                    continue
                bt.append(D.t_of(b.header.stamp)); bsrc.append(b.source)
                bx.append(b.position.x); by.append(b.position.y); bz.append(b.position.z)
        elif topic == "/bb/axis_estimates":
            m = jt_store.deserialize_cdr(bytes(c[22:]), JT)
            idx = {n: i for i, n in enumerate(m.name)}
            ip, ih = idx.get("bb_pitch"), idx.get("bb_hand")
            if ip is None or ih is None:
                return
            et.append(D.t_of(m.header.stamp))
            e_pitch.append(90.0 + 360.0 * m.position[ip])
            e_hvel.append(m.velocity[ih])

    f = open(bag, "rb"); mm = mmap.mmap(f.fileno(), 0, access=mmap.ACCESS_READ)
    assert mm[:8] == b"\x89MCAP0\r\n", "bad MCAP magic"
    for op, c in AV.iter_records(memoryview(mm)[8:]):
        if op == 4:
            handle_channel(c)
        elif op == 6:
            clen = AV.u32(c, 28); comp = bytes(c[32:32 + clen]); q = 32 + clen
            rs = AV.u64(c, q); q += 8
            raw = AV.inflate(comp, c[q:q + rs], AV.u64(c, 16))
            for iop, ic in AV.iter_records(memoryview(raw)):
                if iop == 4:
                    handle_channel(ic)
                elif iop == 5:
                    handle_message(ic)
        elif op == 5:
            handle_message(c)

    balls = dict(t=np.asarray(bt), src=np.asarray(bsrc, dtype=object),
                 x=np.asarray(bx), y=np.asarray(by), z=np.asarray(bz))
    o = np.argsort(balls["t"], kind="stable") if len(bt) else np.array([], int)
    for k in balls:
        balls[k] = balls[k][o] if len(bt) else balls[k]
    enc = dict(t=np.asarray(et), pitch=np.asarray(e_pitch), hvel=np.asarray(e_hvel))
    eo = np.argsort(enc["t"], kind="stable") if len(et) else np.array([], int)
    for k in enc:
        enc[k] = enc[k][eo] if len(et) else enc[k]
    return balls, enc


def fit_one(throw, balls, enc):
    """Fit one throw's launch arc; evaluate launch state at the commanded throw_time."""
    tt = throw["est_throw_wall_ns"] / 1e9
    out = dict(idx=throw["idx"], ok=False, flags=[])
    src_ok = (np.ones(len(balls["t"]), bool) if SRC_FILTER is None
              else (balls["src"] == SRC_FILTER))
    m = (src_ok & (balls["t"] >= tt + A_WIN[0]) &
         (balls["t"] <= tt + A_WIN[1]) & (balls["z"] > -500) & (balls["z"] < 4000))
    if m.sum() < N_MIN:
        out["flags"].append("too_few_arc_samples"); return out
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
    if not (GEFF_OK[0] <= geff <= GEFF_OK[1]) or rms > RMS_MAX:
        out["flags"].append(f"fit_gate(g={geff:.0f},rms={rms:.1f})"); return out
    mx, bx_ = np.polyfit(T[inl], x[inl], 1)
    my, by_ = np.polyfit(T[inl], y[inl], 1)
    # evaluate launch state at the commanded throw_time (firmware releases at throw_time)
    dT = tt - t0
    out.update(ok=True, g_eff=float(geff), rms_mm=float(rms), n=int(inl.sum()),
               vx=float(mx), vy=float(my), vz=float(vz - G * dT),
               rel_x=float(bx_ + mx * dT), rel_y=float(by_ + my * dT),
               rel_z=float(z0 + vz * dT - 0.5 * G * dT * dT),
               cmd_yaw=math.radians(throw["yaw_deg"]),
               cmd_pitch=math.radians(throw["pitch_deg"]),
               cmd_speed=float(throw["speed_mps"]))
    # encoder pitch/hand during the stroke
    em = (enc["t"] >= tt) & (enc["t"] <= tt + 0.25) if len(enc["t"]) else np.array([], bool)
    if em.any():
        out["enc_pitch_deg"] = float(np.median(enc["pitch"][em]))
        out["enc_hand_vpeak_rps"] = float(np.max(np.abs(enc["hvel"][em])))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag"); ap.add_argument("session")
    ap.add_argument("--bb-xyz", default=None, help="x,y,z mm (BB origin, from calibration)")
    ap.add_argument("--yaw-offset-deg", type=float, default=None)
    args = ap.parse_args()

    sess = json.load(open(args.session))
    throws = sess["throws"]
    store = D.setup_typestore()
    jt = get_typestore(Stores.ROS2_HUMBLE)
    print(f"Reading {Path(args.bag).name} ...", flush=True)
    balls, enc = read_bag(args.bag, store, jt)
    print(f"ball samples={len(balls['t'])}  enc samples={len(enc['t'])}  throws={len(throws)}")

    obs = []
    print("\nidx cmd(y/p/s)        v(x/y/z) mm/s         rel(x/y/z) mm        g_eff rms n  encP encHv flags")
    for th in throws:
        if not th.get("success", True):
            continue
        r = fit_one(th, balls, enc)
        if r["ok"]:
            obs.append(r)
            ep = r.get("enc_pitch_deg", float("nan")); eh = r.get("enc_hand_vpeak_rps", float("nan"))
            print(f"{r['idx']:3d} {th['yaw_deg']:+4.0f}/{th['pitch_deg']:3.0f}/{th['speed_mps']:.1f} "
                  f"{r['vx']:7.0f}/{r['vy']:7.0f}/{r['vz']:7.0f}  "
                  f"{r['rel_x']:6.0f}/{r['rel_y']:6.0f}/{r['rel_z']:6.0f}  "
                  f"{r['g_eff']:5.0f} {r['rms_mm']:4.1f} {r['n']:3d} {ep:5.1f} {eh:5.1f}")
        else:
            print(f"{th['idx']:3d} {th['yaw_deg']:+4.0f}/{th['pitch_deg']:3.0f}/{th['speed_mps']:.1f}"
                  f"  --  {','.join(r['flags'])}")
    # Drop throws whose COMMANDED yaw is outside BB's physical axis range — those
    # are clamped by the firmware, so their azimuth/release are not what was asked
    # (this is what corrupted the 2026-06-17 run's −20/−40° throws).
    in_range = [o for o in obs if YAW_VALID_DEG[0] <= math.degrees(o["cmd_yaw"]) <= YAW_VALID_DEG[1]]
    dropped = len(obs) - len(in_range)
    if dropped:
        print(f"dropped {dropped} throw(s) with commanded yaw outside {YAW_VALID_DEG}° (clamped → invalid)")
    Path("kinematic_obs.json").write_text(json.dumps(obs, indent=1, default=float))

    if len(in_range) < 8:
        print("Too few in-range throws to fit — check the arc window/gates and yaw range.")
        return
    yaw_off = math.radians(args.yaw_offset_deg) if args.yaw_offset_deg is not None else 0.0
    s1 = CK.fit_stage1(in_range, yaw_off)
    print("\n=== STAGE 1 diagnostic (decoupled launch-velocity regressions) ===")
    def pr(name, key, unit):
        v, se = s1[key]; print(f"  {name:14s} {v:+8.3f} ± {se:5.3f} {unit}")
    pr("pitch_scale", "pitch_scale", ""); pr("pitch_off", "pitch_off_deg", "deg")
    pr("speed_scale", "speed_scale", ""); pr("speed_off", "speed_off_mps", "m/s")
    if args.yaw_offset_deg is not None:
        pr("yaw_scale", "yaw_scale", ""); pr("yaw_off", "yaw_off_deg", "deg")
    print(f"   R²: pitch={s1['r2_pitch']:.4f} yaw={s1['r2_yaw']:.4f} speed={s1['r2_speed']:.4f}")
    print("   NOTE: Stage-1 conflates a release-timing δ into pitch_off/speed_off; the")
    print("         joint fit below de-biases it. Use the JOINT result for deployment.")

    if not (args.bb_xyz and args.yaw_offset_deg is not None):
        print("\n(Joint fit skipped: pass --bb-xyz x,y,z and --yaw-offset-deg from this run's BB calibration.)")
        return

    bb = tuple(float(v) for v in args.bb_xyz.split(","))
    j = CK.fit_joint(in_range, bb, yaw_off, fit_delta=True)
    print("\n=== JOINT FIT — physical kinematic parameters (deploy these) ===")
    def jp(name, key, unit=""):
        v, se = j[key]; print(f"  {name:16s} {v:+9.3f} ± {se:6.3f} {unit}")
    print(" Axis calibration (→ inverse-ballistics / throw_ballistics.py):")
    jp("pitch_scale", "pitch_scale"); jp("pitch_off", "pitch_off_deg", "deg")
    jp("yaw_scale", "yaw_scale");     jp("yaw_off", "yaw_off_deg", "deg")
    jp("speed_scale", "speed_scale"); jp("speed_off", "speed_off_mps", "m/s")
    dms, dse = j["delta_ms"]
    print(f"  {'release δ':16s} {dms:+9.2f} ± {dse:6.2f} ms  (residual release-lag / clock skew)")
    print(" Release geometry — implied config = nominal + fitted offset:")
    for key, nk, label in (("d_off_mm", "d_mm", "pitch_d_offset_mm"),
                           ("l_off_mm", "l_mm", "release_l_position_mm"),
                           ("s_off_mm", "s_mm", "yaw_s_offset_mm"),
                           ("z_off_mm", "z_mm", "release-z origin corr.")):
        off, se = j[key]
        base = CK.NOMINAL[nk]
        tail = f"→ {base + off:+8.2f}" if key != "z_off_mm" else "(adjusts calib origin z)"
        print(f"  {label:24s} {base:+8.2f} {off:+7.2f} ± {se:5.2f} mm  {tail}")
    print(f"   joint RMS: position={j['pos_rms_mm']:.1f} mm  velocity={j['vel_rms_mmps']:.1f} mm/s  (n={j['n']})")
    print("   (If l/z σ are large they trade off — pin one from CAD and refit.)")


if __name__ == "__main__":
    main()
