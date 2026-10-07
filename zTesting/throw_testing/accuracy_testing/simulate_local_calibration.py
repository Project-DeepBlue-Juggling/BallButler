#!/usr/bin/env python3
"""Offline synthetic calibration capture, including unlabelled tracking clutter.

No hardware/ROS required. The known forward affine is applied to intended
mocap XY landing points by perturbing horizontal release velocity, preserving
the production-model release origin. Independent per-throw velocity-component
errors are uniform +/-1%; observation-coordinate errors are uniform +/-2 mm.
This is a rough ballistic validation, not a model of all hardware errors.
"""
import argparse
import importlib.util
import json
import math
import random
import sys
from pathlib import Path

from run_local_calibration import SCHEMA, flight_capture, prepare_schedule, validate_plan, write_json

FORWARD = [[1.012, 0.008, 10.0], [-0.006, 0.990, 20.0]]
GRAVITY = 9806.0


def load_ballistics(checkout):
    sys.path.insert(0, str(Path(checkout) / 'ros_ws/src/jugglebot'))
    path = Path(checkout) / 'ros_ws/src/jugglebot/jugglebot/can/throw_ballistics.py'
    spec = importlib.util.spec_from_file_location('simulation_ballistics', path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def crossing_time(position, velocity, z):
    return (velocity[2] + math.sqrt(velocity[2] ** 2 + 2 * GRAVITY * (position[2] - z))) / GRAVITY


def point_at(position, velocity, t):
    return [position[0] + velocity[0] * t, position[1] + velocity[1] * t,
            position[2] + velocity[2] * t - 0.5 * GRAVITY * t * t]


def bounced_point(position, velocity, t, ground=0.0):
    """One small bounce, then a rolling marker; vanish 1s after first impact."""
    impact = crossing_time(position, velocity, ground)
    if t < 0 or t >= impact + 1.0:
        return None
    if t <= impact:
        return point_at(position, velocity, t)
    p = point_at(position, velocity, impact)
    age = t - impact
    bounce_vz = min(650.0, abs(velocity[2] - GRAVITY * impact) * 0.12)
    # Horizontal speed sharply reduced at impact; exponential rolling drag.
    travel = (1.0 - math.exp(-3.0 * age)) / 3.0
    return [p[0] + 0.16 * velocity[0] * travel,
            p[1] + 0.16 * velocity[1] * travel,
            ground + max(0.0, bounce_vz * age - 0.5 * GRAVITY * age * age)]


def simulate(plan, out, ballistics, seed=20261006, limit=None, rate=200.0,
             noise_mm=2.0, state_error=0.01):
    validate_plan(plan)
    if rate <= 0 or noise_mm < 0 or not 0 <= state_error < 1:
        raise ValueError('Require rate > 0, noise >= 0 and 0 <= state error < 1.')
    out = Path(out)
    out.mkdir(parents=True, exist_ok=False)
    rng = random.Random(seed)
    # Synthetic pose chosen so all 331 default targets pass production limits.
    # This is not a measured pose or a recommendation to move the hardware.
    pose = [-1400.0, -700.0, 1741.18188]
    yaw = -0.3
    feasible, skipped = prepare_schedule(plan, pose, yaw, [0., 0.],
        ballistics.solve_throw_local, ballistics.global_to_bb_local, 105.65)
    selected = feasible[:limit] if limit else feasible
    session = dict(schema=SCHEMA, simulated=True, status='completed', plan=plan,
        schedule_to_mocap_mm=[0., 0.], signed_s_mm=105.65, affine_applied=False,
        columns_feed_bias_applied=False, bb_pose=dict(position_mm=pose, yaw_offset_rad=yaw),
        refill_every=9, ground_z_mm=0., throws=[], refill_intervals=[],
        skipped_unreachable=skipped, selected_throw_indices=[r['throw_idx'] for r in selected],
        observation_file='observations.jsonl')
    truth = dict(seed=seed, forward_affine_mocap_xy=FORWARD,
        state_error_definition='Independent uniform multiplicative velocity-component errors per throw; release position unchanged.',
        state_error_fraction=state_error, observation_noise_uniform_mm=noise_mm,
        rate_hz=rate, gravity_mm_s2=GRAVITY, throws=[])
    clock = 1800000000.0
    static = [[-1100., -550., 1400.], [350., 600., 150.], [50., -350., 0.]]
    frames = 0
    stream = (out / 'observations.jsonl').open('w', encoding='utf-8')

    def emit_segment(start, duration, trajectories):
        nonlocal frames
        for k in range(int(math.ceil(duration * rate))):
            stamp = start + k / rate
            points = list(static)
            for release, position, velocity in trajectories:
                p = bounced_point(position, velocity, stamp - release)
                if p is not None:
                    points.append(p)
            points = [[round(v + rng.uniform(-noise_mm, noise_mm), 4) for v in p] for p in points]
            rng.shuffle(points)  # Never rely on list position or a tracking label.
            stream.write(json.dumps(dict(wall_time_s=stamp, points_mm=points), separators=(',', ':')) + '\n')
            frames += 1

    try:
        for index, entry in enumerate(selected):
            if index % 9 == 0:
                duration = 12.0
                session['refill_intervals'].append(dict(start_wall_s=clock, end_wall_s=clock + duration))
                # Nine human refill arcs, some overlapping: all are explicitly excluded.
                refill = []
                for n in range(9 if index else 2):
                    p = [rng.uniform(-300, 300), rng.uniform(-200, 200), 500.]
                    tof = 0.95
                    v = [(pose[0] - p[0]) / tof, (pose[1] - p[1]) / tof,
                         (pose[2] - p[2]) / tof + 0.5 * GRAVITY * tof]
                    refill.append((clock + n * 0.9, p, v))
                emit_segment(clock, duration, refill)
                clock += duration
            sol = entry['solution']
            r, v = ballistics.bb_release_state(sol['yaw_rad'], sol['pitch_rad'], sol['speed_mps'],
                                               pose, yaw, yaw_s_offset_mm=105.65)
            r, v = list(r), list(v)
            capture = flight_capture(r, v, sol['tof_s'], 0.)
            release = clock + 3.0
            row = dict(entry, predicted_release_position_mm=r, predicted_release_velocity_mm_s=v,
                capture_duration_s=capture, nominal_release_wall_s=release,
                analysis_window_wall_s=[release - 0.25, release + capture],
                dispatch_wall_time_ns=int(clock * 1e9), status='released', capture_complete=True)
            session['throws'].append(row)
            target = row['target_global_mm']
            systematic = [sum(a * b for a, b in zip(line, target[:2] + [1.])) for line in FORWARD]
            actual_v = list(v)
            for axis in range(2):
                actual_v[axis] += (systematic[axis] - target[axis]) / sol['tof_s']
            factors = [rng.uniform(1 - state_error, 1 + state_error) for _ in range(3)]
            actual_v = [a * b for a, b in zip(actual_v, factors)]
            catch_t = crossing_time(r, actual_v, target[2])
            truth['throws'].append(dict(throw_idx=row['throw_idx'], actual_release_position_mm=r,
                actual_release_velocity_mm_s=actual_v, velocity_error_factors=factors,
                actual_catch_mm=point_at(r, actual_v, catch_t), catch_tof_s=catch_t))
            # Include full post-impact disappearance and nominal inter-throw pause.
            duration = max(3.0 + capture + 1.0, 3.0 + crossing_time(r, actual_v, 0.) + 1.01)
            emit_segment(clock, duration, [(release, r, actual_v)])
            clock += duration
    finally:
        stream.close()
    session['feasible_schedule'] = feasible
    session['simulated_frame_count'] = frames
    write_json(out / 'session.json', session)
    write_json(out / 'truth.json', truth)
    return session


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--plan', type=Path, default=Path(__file__).with_name('local_calibration_plan.json'))
    parser.add_argument('--out', type=Path, required=True, help='new output directory')
    parser.add_argument('--jugglebot', type=Path, default=Path(__file__).resolve().parents[4] / 'Jugglebot')
    parser.add_argument('--seed', type=int, default=20261006)
    parser.add_argument('--limit', type=int)
    parser.add_argument('--rate', type=float, default=200.)
    parser.add_argument('--noise-mm', type=float, default=2.)
    parser.add_argument('--state-error', type=float, default=.01)
    args = parser.parse_args()
    if args.limit is not None and args.limit <= 0:
        parser.error('--limit must be positive')
    plan = json.loads(args.plan.read_text(encoding='utf-8'))
    session = simulate(plan, args.out, load_ballistics(args.jugglebot), args.seed,
                       args.limit, args.rate, args.noise_mm, args.state_error)
    print('%d throws, %d frames -> %s' % (len(session['throws']), session['simulated_frame_count'], args.out))


if __name__ == '__main__':
    main()
