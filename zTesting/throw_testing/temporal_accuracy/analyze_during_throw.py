#!/home/jetson/Desktop/PDJ_venv/venv/bin/python3
"""
During-throw BB pitch + hand diagnostics from /bb/axis_estimates.

Consumes the high-rate BB pitch/hand encoder estimates added to the can-bridge
(MsgType BB_AXIS_ESTIMATES → /bb/axis_estimates, sensor_msgs/JointState:
name=[bb_pitch, bb_hand], position=rev, velocity=rev/s, header.stamp = bridge
wall time). Aligns each throw (from /throw_announcements) and reports:

  PITCH (angle root-cause): commanded pitch vs the encoder's barrel angle during
    the stroke (deg = 90 + 360*pos_rev, PitchAxis.h), plus the deflection range
    over the stroke — a static offset vs a dynamic (reaction-torque) wobble.
    NB: this is the ENCODER's view; if the barrel flexes downstream of the
    encoder, the encoder won't see it (needs the spirit-level / barrel-mocap
    cross-checks). A constant Δ here ⇒ encoder zero/calibration; Δ that tracks
    the commanded pitch across the workspace ⇒ scale/linkage.

  HAND (speed root-cause): commanded peak hand velocity (= v_throw * LINEAR_GAIN,
    LINEAR_GAIN = 1/(2*pi*R) with FACTOR=1.0) vs the encoder's peak velocity,
    fit from the accel/decel ramps (robust to the ~4 ms flat-top vs the 100 Hz
    forward rate). If actual ≈ commanded hand velocity, the hand TRACKS the
    command and the ~10% hot launch is the effective spool radius (mechanical);
    if actual > commanded, it's servo overshoot. Implied ball speed =
    peak_vel_rps * 2*pi*R is z_rel-independent — compare to the mocap launch speed.

Requires a bag recorded AFTER the bridge reflash, with /bb/axis_estimates and the
BB ODrives broadcasting get_encoder_estimate (set to 1 kHz). Reuses the
summary-free pure-Python MCAP reader.

    analyze_during_throw.py [BAG.mcap]
"""
import math
import sys
from pathlib import Path

import numpy as np
from rosbags.typesys import get_typestore, Stores

import analyze_velocity_postfix as AV   # iter_records, inflate, u16/u32/u64
import decompose_from_bag as D          # setup_typestore (jugglebot msgs), t_of

# BB hand geometry (hardware_config.h BBTraj) — keep in sync if the config changes.
HAND_SPOOL_RADIUS_M = 0.0052493
LINEAR_GAIN_FACTOR = 1.0
LINEAR_GAIN_REV_PER_M = LINEAR_GAIN_FACTOR / (2.0 * math.pi * HAND_SPOOL_RADIUS_M)  # rev/m
TWO_PI_R = 2.0 * math.pi * HAND_SPOOL_RADIUS_M                                       # m/rev

DEFAULT_BAG = "/home/jetson/Desktop/rosbags/2026-06-17_15-13-42/2026-06-17_15-13-42_0.mcap"
STROKE_WIN = (-0.05, 0.35)   # s around throw_time to look at the stroke


def read_bag(bag, store, jt_store):
    """Return (anns, est) where est has t[], pitch_deg[], hand_vel_rps[], hand_pos_rev[]."""
    chan = {}
    anns = []
    et, ppos, pvel, hpos, hvel = [], [], [], [], []
    JT = "sensor_msgs/msg/JointState"
    TA = "jugglebot_interfaces/msg/ThrowAnnouncement"

    def handle_channel(c):
        chan[AV.u16(c, 0)] = bytes(c[8:8 + AV.u32(c, 4)]).decode("utf-8", "replace")

    def handle_message(c):
        topic = chan.get(AV.u16(c, 0))
        if topic == "/throw_announcements":
            m = store.deserialize_cdr(bytes(c[22:]), TA)
            anns.append(dict(stamp=D.t_of(m.header.stamp), throw_time=D.t_of(m.throw_time),
                             landing_time=D.t_of(m.landing_time), tof=m.predicted_tof_sec,
                             v0=(m.initial_velocity.x, m.initial_velocity.y, m.initial_velocity.z)))
        elif topic == "/bb/axis_estimates":
            m = jt_store.deserialize_cdr(bytes(c[22:]), JT)
            t = D.t_of(m.header.stamp)
            name = list(m.name); pos = list(m.position); vel = list(m.velocity)
            idx = {n: i for i, n in enumerate(name)}
            ip, ih = idx.get("bb_pitch"), idx.get("bb_hand")
            if ip is None or ih is None:
                return
            et.append(t)
            ppos.append(pos[ip]); pvel.append(vel[ip])
            hpos.append(pos[ih]); hvel.append(vel[ih])

    import mmap
    f = open(bag, "rb"); mm = mmap.mmap(f.fileno(), 0, access=mmap.ACCESS_READ)
    assert mm[:8] == b"\x89MCAP0\r\n", "bad MCAP magic"
    for op, c in AV.iter_records(memoryview(mm)[8:]):
        if op == 4:
            handle_channel(c)
        elif op == 6:
            clen = AV.u32(c, 28); comp = bytes(c[32:32 + clen]); q = 32 + clen
            rsize = AV.u64(c, q); q += 8
            raw = AV.inflate(comp, c[q:q + rsize], AV.u64(c, 16))
            for iop, ic in AV.iter_records(memoryview(raw)):
                if iop == 4:
                    handle_channel(ic)
                elif iop == 5:
                    handle_message(ic)
        elif op == 5:
            handle_message(c)

    t = np.asarray(et)
    order = np.argsort(t, kind="stable") if len(t) else np.array([], int)
    est = dict(t=t[order] if len(t) else t,
               pitch_deg=90.0 + 360.0 * np.asarray(ppos)[order] if len(t) else np.array([]),
               hand_vel_rps=np.asarray(hvel)[order] if len(t) else np.array([]),
               hand_pos_rev=np.asarray(hpos)[order] if len(t) else np.array([]))
    return anns, est


def fit_trapezoid_peak(t, v):
    """Peak of a trapezoidal speed profile: fit the rising and falling legs as
    lines and return their intersection value. Robust to a sparse flat-top.
    Falls back to max(v) if the geometry is degenerate."""
    if len(t) < 6:
        return float(np.max(v)) if len(v) else float("nan"), "max"
    i_pk = int(np.argmax(v))
    up_t, up_v = t[:i_pk + 1], v[:i_pk + 1]
    dn_t, dn_v = t[i_pk:], v[i_pk:]
    if len(up_t) < 2 or len(dn_t) < 2:
        return float(np.max(v)), "max"
    a_up = np.polyfit(up_t, up_v, 1)   # rising leg
    a_dn = np.polyfit(dn_t, dn_v, 1)   # falling leg
    if abs(a_up[0] - a_dn[0]) < 1e-6:
        return float(np.max(v)), "max"
    t_cross = (a_dn[1] - a_up[1]) / (a_up[0] - a_dn[0])
    v_cross = a_up[0] * t_cross + a_up[1]
    # guard: crossing should sit near the data peak
    if not (t[0] <= t_cross <= t[-1]) or v_cross < 0.9 * np.max(v):
        return float(np.max(v)), "max"
    return float(v_cross), "fit"


def main():
    bag = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_BAG
    store = D.setup_typestore()
    jt_store = get_typestore(Stores.ROS2_HUMBLE)   # has sensor_msgs/JointState
    print(f"Reading {Path(bag).name} ...", flush=True)
    anns, est = read_bag(bag, store, jt_store)
    print(f"announcements={len(anns)}  bb_axis_estimate samples={len(est['t'])}")
    if len(est["t"]) == 0:
        print("\nNo /bb/axis_estimates in this bag. Needs: bridge reflashed with the\n"
              "BB_AXIS_ESTIMATES change, BB ODrives broadcasting get_encoder_estimate,\n"
              "and the topic recorded (ros2 bag record -a, or add /bb/axis_estimates).")
        return
    dt = np.median(np.diff(est["t"])) if len(est["t"]) > 1 else float("nan")
    print(f"sample spacing: median {dt*1e3:.2f} ms ({1/dt:.0f} Hz)")

    print("\nidx  cmd_pitch meas_pitch  Δpitch  deflect | cmd_vthr  peak_vrps cmd_vrps "
          "track  impl_ball Δspeed%  (fit)")
    rows = []
    for i, a in enumerate(anns):
        tt = a["throw_time"]
        v0 = np.array(a["v0"]); speed = float(np.linalg.norm(v0)) / 1000.0  # mm/s → m/s
        cmd_pitch = math.degrees(math.atan2(v0[2], math.hypot(v0[0], v0[1])))
        m = (est["t"] >= tt + STROKE_WIN[0]) & (est["t"] <= tt + STROKE_WIN[1])
        if m.sum() < 6:
            print(f"{i:3d}  (too few axis samples in window)")
            continue
        tw = est["t"][m] - tt
        pit = est["pitch_deg"][m]
        hv = est["hand_vel_rps"][m]
        # pitch during the active stroke (after throw_time, before settle)
        sm = (tw >= 0.0) & (tw <= 0.25)
        meas_pitch = float(np.median(pit[sm])) if sm.sum() else float(np.median(pit))
        deflect = float(np.max(pit[sm]) - np.min(pit[sm])) if sm.sum() else float("nan")
        # hand peak
        peak_vrps, how = fit_trapezoid_peak(tw, hv)
        cmd_vrps = speed * LINEAR_GAIN_REV_PER_M
        track = peak_vrps / cmd_vrps if cmd_vrps else float("nan")
        impl_ball = peak_vrps * TWO_PI_R
        dspeed = (impl_ball / speed - 1.0) * 100.0 if speed else float("nan")
        rows.append(dict(i=i, cmd_pitch=cmd_pitch, meas_pitch=meas_pitch,
                         dpitch=meas_pitch - cmd_pitch, deflect=deflect,
                         track=track, impl_ball=impl_ball, dspeed=dspeed))
        print(f"{i:3d}  {cmd_pitch:8.2f} {meas_pitch:9.2f} {meas_pitch-cmd_pitch:+7.2f} "
              f"{deflect:7.2f} | {speed:7.2f}  {peak_vrps:8.1f} {cmd_vrps:8.1f} "
              f"{track:6.3f} {impl_ball:8.2f} {dspeed:+7.1f}  ({how})")

    if rows:
        def agg(k):
            v = np.array([r[k] for r in rows], float); v = v[np.isfinite(v)]
            return (np.mean(v), np.std(v, ddof=1) if len(v) > 1 else 0.0, len(v))
        print("\n=== AGGREGATE ===")
        for k, lbl in [("dpitch", "Δpitch (meas−cmd) deg"), ("deflect", "pitch deflection deg"),
                       ("track", "hand peak / commanded (servo track)"),
                       ("impl_ball", "implied ball speed m/s"),
                       ("dspeed", "implied-ball vs commanded %")]:
            mu, sd, n = agg(k)
            print(f"  {lbl:36s} mean={mu:+8.3f}  σ={sd:6.3f}  n={n}")
        print("\nReading: Δpitch ≈ the encoder-side launch-angle error; compare its sign/size\n"
              "to the mocap +3.1°. If the encoder reads ~commanded (Δpitch≈0) but mocap is\n"
              "+3°, the error is downstream of the encoder (flex/linkage) — not calibration.\n"
              "track≈1.0 ⇒ hand tracks command ⇒ ~10% hot launch is spool radius; track>1 ⇒ overshoot.")


if __name__ == "__main__":
    main()
