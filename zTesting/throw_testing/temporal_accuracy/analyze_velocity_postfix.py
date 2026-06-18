#!/home/jetson/Desktop/PDJ_venv/venv/bin/python3
"""
v_meas vs v_cmd on the POST-RELEASE-LAG-FIX session (2026-06-17 14:55).

Goal: measure the ball's actual launch speed and compare it to the commanded
throw speed, to characterise the residual ~30-58 ms arrival error now that the
release-lag scheduling defect (HandPathPlanner.cpp:72/:288) is fixed.

Robustness (the differentiation-noise worry): velocity is NOT finite-differenced.
We reuse decompose_from_bag's fit_gfixed — a LINEAR least-squares fit of the
whole descent to the KNOWN-g parabola  z + 1/2 g T^2 = z0 + vz T  (g fixed at
9806 mm/s^2), plus a constant-velocity line for the horizontal axes. Both
average over ~30-60 samples; the parabola SHAPE is imposed, so per-sample mocap
noise is suppressed ~sqrt(N) and never amplified. Quality is cross-checked by
re-fitting with g FREE (g_eff should land near 9806 if the fit is clean) and by
the per-fit RMS residual.

We use track B (source 'human_throw': the measurement-driven RE-DETECTION of the
ball mid-flight), NOT track A (source 'ball_butler': anchored to the commanded
prior, so its velocity is circular). See decompose_from_bag.py header.

Bag is read with a summary-free pure-Python MCAP reader (the mcap lib's seeking
reader chokes on this bag's summary), then decoded via the rosbags typestore.

    /home/jetson/Desktop/PDJ_venv/venv/bin/python3 analyze_velocity_postfix.py
"""
import mmap
import struct
import json
from pathlib import Path

import numpy as np

import decompose_from_bag as D   # reuse setup_typestore, analyse_throw, fit_* , t_of

BAG = Path("/home/jetson/Desktop/rosbags/2026-06-17_14-51-24/2026-06-17_14-51-24_0.mcap")
OUT_JSON = Path(__file__).with_name("velocity_postfix_results.json")

u16 = lambda b, o: struct.unpack_from('<H', b, o)[0]
u32 = lambda b, o: struct.unpack_from('<I', b, o)[0]
u64 = lambda b, o: struct.unpack_from('<Q', b, o)[0]

TOPIC_MSG = {
    "/throw_announcements": "jugglebot_interfaces/msg/ThrowAnnouncement",
    "/cone/catch_event":    "jugglebot_interfaces/msg/CatchEvent",
    "/balls":               "jugglebot_interfaces/msg/BallStateArray",
}


def iter_records(buf):
    p, L = 0, len(buf)
    while p + 9 <= L:
        op = buf[p]
        ln = u64(buf, p + 1)
        s = p + 9
        if s + ln > L:
            break
        yield op, buf[s:s + ln]
        p = s + ln


def inflate(comp, recs, usize):
    if comp == b'':
        return recs
    if comp == b'zstd':
        import zstandard
        return zstandard.ZstdDecompressor().decompress(bytes(recs), max_output_size=usize)
    if comp == b'lz4':
        import lz4.frame
        return lz4.frame.decompress(bytes(recs))
    raise RuntimeError('unknown compression %r' % comp)


def read_bag_purepy(store):
    """Summary-free sequential read; returns (anns, catches, balls) exactly like
    decompose_from_bag.read_bag so D.analyse_throw can consume it."""
    chan = {}
    anns, catches = [], []
    bt, bid, bsrc, bx, by, bz = [], [], [], [], [], []

    def handle_channel(c):
        chan[u16(c, 0)] = bytes(c[8:8 + u32(c, 4)]).decode('utf-8', 'replace')

    def handle_message(c):
        topic = chan.get(u16(c, 0))
        mt = TOPIC_MSG.get(topic)
        if mt is None:
            return
        data = bytes(c[22:])                 # msg header is 22 bytes; rest is CDR data
        m = store.deserialize_cdr(data, mt)
        if topic == "/throw_announcements":
            anns.append(dict(
                stamp=D.t_of(m.header.stamp), throw_time=D.t_of(m.throw_time),
                landing_time=D.t_of(m.landing_time), tof=m.predicted_tof_sec,
                p0=(m.initial_position.x, m.initial_position.y, m.initial_position.z),
                v0=(m.initial_velocity.x, m.initial_velocity.y, m.initial_velocity.z),
                pl=(m.landing_position.x, m.landing_position.y, m.landing_position.z),
                vl=(m.landing_velocity.x, m.landing_velocity.y, m.landing_velocity.z),
                target=m.target_id))
        elif topic == "/cone/catch_event":
            catches.append(dict(t=D.t_of(m.catch_time), seq=int(m.sequence),
                                synced=bool(m.time_synced)))
        else:
            for b in m.balls:
                if b.tracking != 1 or b.status != 1:
                    continue
                bt.append(D.t_of(b.header.stamp)); bid.append(b.id)
                bsrc.append(b.source)
                bx.append(b.position.x); by.append(b.position.y); bz.append(b.position.z)

    f = open(BAG, 'rb')
    mm = mmap.mmap(f.fileno(), 0, access=mmap.ACCESS_READ)
    assert mm[:8] == b'\x89MCAP0\r\n', 'bad MCAP magic'
    for op, c in iter_records(memoryview(mm)[8:]):
        if op == 4:
            handle_channel(c)
        elif op == 6:
            clen = u32(c, 28); comp = bytes(c[32:32 + clen]); q = 32 + clen
            rsize = u64(c, q); q += 8
            raw = inflate(comp, c[q:q + rsize], u64(c, 16))
            for iop, ic in iter_records(memoryview(raw)):
                if iop == 4:
                    handle_channel(ic)
                elif iop == 5:
                    handle_message(ic)
        elif op == 5:
            handle_message(c)

    balls = dict(t=np.asarray(bt), id=np.asarray(bid),
                 src=np.asarray(bsrc, dtype=object),
                 x=np.asarray(bx), y=np.asarray(by), z=np.asarray(bz))
    order = np.argsort(balls["t"], kind="stable")
    for k in balls:
        balls[k] = balls[k][order]
    return anns, catches, balls


def st(vals):
    v = np.asarray([x for x in vals if x is not None and np.isfinite(x)], float)
    if len(v) == 0:
        return None
    return dict(n=int(len(v)), mean=float(v.mean()),
                std=float(v.std(ddof=1)) if len(v) > 1 else 0.0,
                min=float(v.min()), max=float(v.max()), median=float(np.median(v)))


def main():
    store = D.setup_typestore()
    print(f"Reading {BAG.name} (summary-free)...", flush=True)
    anns, catches, balls = read_bag_purepy(store)
    print(f"announcements={len(anns)}  catches={len(catches)}  "
          f"confirmed-flight ball samples={len(balls['t'])}")

    results = [D.analyse_throw(a, balls, catches) for a in anns]
    for i, r in enumerate(results):
        r["idx"] = i

    print("\nidx  arr_err rel_lagB flt_errB fall_B   v_cmd  v_measB ratio  "
          "vz_r  vh_r  g_eff  rms  nfit  flags")
    for r in results:
        def g(k, f="{:7.1f}"):
            return f.format(r[k]) if (k in r and r[k] is not None) else "   --- "
        ratio = (r["meas_speed_B_mmps"] / r["commanded_speed_mmps"]
                 if "meas_speed_B_mmps" in r else None)
        vzr = (r["meas_vz_release_B_mmps"] / r["cmd_vz_mmps"]
               if "meas_vz_release_B_mmps" in r else None)
        vhr = (r["meas_vh_B_mmps"] / r["cmd_vh_mmps"]
               if "meas_vh_B_mmps" in r else None)
        bf = r.get("B_fit", {})
        print(f"{r['idx']:3d}{g('arrival_error_ms')}{g('release_lag_B_ms')}"
              f"{g('flight_error_B_ms')}{g('fall_through_B_ms')}"
              f"{g('commanded_speed_mmps','{:7.0f}')}{g('meas_speed_B_mmps','{:7.0f}')}"
              f"{'  '+format(ratio,'.3f') if ratio else '   --- '}"
              f"{'  '+format(vzr,'.3f') if vzr else '   --- '}"
              f"{'  '+format(vhr,'.3f') if vhr else '   --- '}"
              f"{'  '+format(bf['g_eff_free'],'6.0f') if bf else '    ---'}"
              f"{'  '+format(bf['rms_mm'],'4.1f') if bf else '   ---'}"
              f"{'  '+format(bf['n'],'3d') if bf else '  ---'}"
              f"  {','.join(r['flags']) if r['flags'] else ''}")

    n_B = sum(1 for r in results if "meas_speed_B_mmps" in r)
    print(f"\n=== AGGREGATE (n with track-B fit = {n_B} / {len(results)}) ===")
    agg = {}
    for k in ["arrival_error_ms", "release_lag_B_ms", "flight_error_B_ms",
              "fall_through_B_ms", "t_land_B_rel_landing_ms"]:
        s = st([r.get(k) for r in results]); agg[k] = s
        if s:
            print(f"{k:26s} n={s['n']:2d} mean={s['mean']:+8.2f} "
                  f"σ={s['std']:6.2f}  med={s['median']:+8.2f} "
                  f"[{s['min']:+8.2f},{s['max']:+8.2f}]")

    print("\n=== SPEED: measured-at-release / commanded ===")
    ratio = st([r["meas_speed_B_mmps"] / r["commanded_speed_mmps"]
                for r in results if "meas_speed_B_mmps" in r])
    vzr = st([r["meas_vz_release_B_mmps"] / r["cmd_vz_mmps"]
              for r in results if "meas_vz_release_B_mmps" in r])
    vhr = st([r["meas_vh_B_mmps"] / r["cmd_vh_mmps"]
              for r in results if "meas_vh_B_mmps" in r])
    for name, s in [("|v| ratio", ratio), ("vz ratio", vzr), ("vh ratio", vhr)]:
        if s:
            print(f"{name:10s} mean={s['mean']:.4f}  σ={s['std']:.4f}  "
                  f"med={s['median']:.4f}  [{s['min']:.4f},{s['max']:.4f}]  n={s['n']}")
    agg["speed_ratio"] = ratio; agg["vz_ratio"] = vzr; agg["vh_ratio"] = vhr

    print("\n=== FIT-QUALITY DIAGNOSTICS (robustness vs mocap noise) ===")
    geff = st([r["B_fit"]["g_eff_free"] for r in results if "B_fit" in r])
    rms = st([r["B_fit"]["rms_mm"] for r in results if "B_fit" in r])
    nfit = st([r["B_fit"]["n"] for r in results if "B_fit" in r])
    if geff:
        print(f"g_eff_free (g FREE)  mean={geff['mean']:.0f} σ={geff['std']:.0f} mm/s^2 "
              f"(vacuum truth 9806; near it ⇒ clean fit)")
    if rms:
        print(f"fit residual RMS     mean={rms['mean']:.1f} σ={rms['std']:.1f} mm "
              f"(per-sample mocap noise floor)")
    if nfit:
        print(f"samples per fit      mean={nfit['mean']:.0f} "
              f"(noise on vz ~ RMS/(span·sqrt(N)))")
    agg["g_eff_free"] = geff; agg["fit_rms_mm"] = rms; agg["n_samples_fit"] = nfit

    OUT_JSON.write_text(json.dumps(dict(
        bag=str(BAG), n_announcements=len(anns), n_catches=len(catches),
        aggregate=agg, throws=results), indent=1, default=float))
    print(f"\nSaved -> {OUT_JSON}")


if __name__ == "__main__":
    main()
