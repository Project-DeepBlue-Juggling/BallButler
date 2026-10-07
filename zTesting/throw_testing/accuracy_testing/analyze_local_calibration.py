#!/usr/bin/env python3
"""Extract launch-associated free flights and fit an inverse BB-local XY affine.

Offline: numpy only for JSONL; mcap and mcap-ros2-support for recorded ROS2 bags.
Never reads simulation truth and never installs a correction on hardware.
"""
from __future__ import annotations
import argparse
import bisect
import hashlib
import json
import math
from pathlib import Path
import numpy as np
from run_local_calibration import analysis_throw_at, write_json


def observations(path):
    path = Path(path)
    if path.suffix == '.jsonl':
        with path.open(encoding='utf-8') as stream:
            for line in stream:
                row = json.loads(line)
                yield row['wall_time_s'], np.asarray(row['points_mm'], float).reshape(-1, 3)
        return
    from mcap.reader import make_reader
    from mcap_ros2.decoder import DecoderFactory
    files = sorted(path.glob('*.mcap')) if path.is_dir() else [path]
    if not files:
        raise ValueError('No MCAP files found')
    for file in files:
        with file.open('rb') as stream:
            reader = make_reader(stream, decoder_factories=[DecoderFactory()])
            for _, _, msg, ros in reader.iter_decoded_messages(topics=['/mocap_data']):
                t = ros.stamp.sec + ros.stamp.nanosec * 1e-9
                # Receipt time jitter corrupts ballistics: reject unsynchronised frames.
                if not t:
                    continue
                if abs(t - msg.log_time * 1e-9) > 2:
                    raise ValueError('Mocap/recording clocks differ by >2 s; check clock synchronisation')
                yield t, np.asarray([[m.position.x, m.position.y, m.position.z]
                                     for m in ros.markers if not m.label], float).reshape(-1, 3)


def crossing(coeff, z, gravity):
    disc = coeff[1, 2]**2 + 2 * gravity * (coeff[0, 2] - z)
    if disc <= 0:
        return None
    t = (coeff[1, 2] + math.sqrt(disc)) / gravity
    return t if t > 0 else None


def extract(row, frames, gravity=9806.):
    """Gravity-constrained RANSAC, seeded near release, with no target-XY gate.

    Labels/track IDs are deliberately ignored. Fit only the launch-connected
    arc down through the catch plane. Reject competing plausible trajectories.
    """
    origin = np.asarray(row['predicted_release_position_mm'])
    velocity = np.asarray(row['predicted_release_velocity_mm_s'])
    epoch = row['nominal_release_wall_s']
    z = row['target_global_mm'][2]
    times, points, frame_ids = [], [], []
    for i, (stamp, pts) in enumerate(sorted(frames, key=lambda f: f[0])):
        t = stamp - epoch
        if t < -.05:
            continue
        for p in pts:
            if np.isfinite(p).all() and p[2] > z - 120:
                times.append(t); points.append(p); frame_ids.append(i)
    if len(times) < 30:
        raise ValueError('insufficient airborne observations')
    t = np.asarray(times); p = np.asarray(points); ids = np.asarray(frame_ids)
    q = p.copy(); q[:, 2] += .5 * gravity * t*t
    design = np.c_[np.ones(len(t)), t]
    predicted = origin + t[:, None]*velocity
    early = np.flatnonzero((t >= 0) & (t < .35) & (np.linalg.norm(q-predicted, axis=1) < 200))
    if len(early) < 10:
        raise ValueError('no launch-associated observations')
    rng = np.random.default_rng(int(row['throw_idx']) + 104)
    candidates = []
    def inliers(coeff):
        end = crossing(coeff, z, gravity)
        if end is None:
            return np.zeros(len(t), bool)
        residual = np.linalg.norm(q-design@coeff, axis=1)
        mask = (residual < 8) & (t <= end + .04)
        # Multiple nearby reflections in a frame must not overweight that frame.
        ix = np.flatnonzero(mask)
        ordered = ix[np.argsort(residual[ix])]
        _, first = np.unique(ids[ordered], return_index=True)
        mask[:] = False
        mask[ordered[first]] = True
        return mask
    for _ in range(180):
        a, b = rng.choice(early, 2, replace=False)
        if abs(t[b]-t[a]) < .08:
            continue
        v = (q[b]-q[a])/(t[b]-t[a]); r = q[a]-t[a]*v
        if np.linalg.norm(r-origin) > 150 or np.linalg.norm(v-velocity) > .3*np.linalg.norm(velocity):
            continue
        coeff = np.array([r, v]); mask = inliers(coeff)
        if mask.sum() < 30:
            continue
        for _ in range(3):
            coeff = np.linalg.lstsq(design[mask], q[mask], rcond=None)[0]
            mask = inliers(coeff)
            if mask.sum() < 30:
                break
        if mask.sum() < 30:
            continue
        end = crossing(coeff, z, gravity)
        ts = np.unique(t[mask])
        if ts[-1]-ts[0] < .25 or ts[0] > .2 or np.max(np.diff(ts)) > .08:
            continue
        if not (ts[0] < end and ts[-1] >= end-.015):
            continue
        rms = float(np.sqrt(np.mean(np.sum((q[mask]-design[mask]@coeff)**2, axis=1))))
        if rms > 5:
            continue
        xy = coeff[0, :2]+end*coeff[1, :2]
        candidates.append((int(mask.sum()), rms, xy, end, coeff))
    if not candidates:
        raise ValueError('no continuous launch-to-catch ballistic fit passed quality gates')
    candidates.sort(key=lambda c: (-c[0], c[1]))
    best = candidates[0]
    if any(c[0] >= .8*best[0] and np.linalg.norm(c[2]-best[2]) > 10 for c in candidates[1:]):
        raise ValueError('ambiguous competing launch trajectories')
    return dict(landing_global_mm=[*best[2].tolist(), z], samples=best[0],
                fit_rms_mm=best[1], catch_wall_s=epoch+best[3], release_fit=best[4].tolist())


def stats(errors):
    errors = np.asarray(errors)
    n = np.linalg.norm(errors, axis=1)
    return dict(n=len(n), mean_vector_mm=errors.mean(axis=0).tolist(),
                rms_mm=float(np.sqrt(np.mean(n*n))), median_mm=float(np.median(n)),
                p95_mm=float(np.percentile(n, 95)))


def local(xy, pose):
    angle = pose['yaw_offset_rad']; c, s = math.cos(angle), math.sin(angle)
    return (np.asarray(xy)-np.asarray(pose['position_mm'][:2])) @ np.array([[c, -s], [s, c]])


def fit_forward(command, measured):
    # Fit command -> mean response (noise belongs to response), then invert.
    centre = command.mean(axis=0); scale = command.std(axis=0)
    if np.min(scale) < 20:
        raise ValueError('insufficient 2D target spread')
    design = np.c_[(command-centre)/scale, np.ones(len(command))]
    if np.linalg.matrix_rank(design) < 3 or np.linalg.cond(design) > 100:
        raise ValueError('degenerate target geometry')
    coefficients = np.linalg.lstsq(design, measured, rcond=None)[0]
    matrix = coefficients[:2].T / scale
    offset = coefficients[2]-matrix@centre
    if np.linalg.cond(matrix) > 10 or np.linalg.det(matrix) <= 0:
        raise ValueError('unstable or mirrored affine response')
    inverse = np.linalg.inv(matrix)
    return np.c_[matrix, offset], np.c_[inverse, -inverse@offset]


def analyse(session_path, data_path, out, exclude=()):
    session_path = Path(session_path)
    session = json.loads(session_path.read_text(encoding='utf-8'))
    if session.get('affine_applied') is not False or session.get('signed_s_mm', 0) <= 0:
        raise ValueError('Expected positive-s calibration with affine disabled')
    rows = [r for r in session['throws'] if r.get('capture_complete') and r.get('status') == 'released'
            and r['throw_idx'] not in exclude]
    rows.sort(key=lambda r: r['analysis_window_wall_s'][0])
    starts = [r['analysis_window_wall_s'][0] for r in rows]
    frames = {r['throw_idx']: [] for r in rows}
    for t, pts in observations(data_path):
        j = bisect.bisect_right(starts, t)-1
        if j >= 0 and t < rows[j]['analysis_window_wall_s'][1]:
            idx = analysis_throw_at(session, t)
            if idx is not None:
                frames[idx].append((t, pts))
    accepted, rejected = [], []
    gravity = session.get('hardware_constants', {}).get('GRAVITY_MPS2', 9.806)*1000
    for number, row in enumerate(rows, 1):
        try:
            result = extract(row, frames[row['throw_idx']], gravity)
            accepted.append(dict(row, **result))
        except ValueError as error:
            rejected.append(dict(throw_idx=row['throw_idx'], cell_idx=row['cell_idx'], reason=str(error)))
        if number % 25 == 0 or number == len(rows):
            print('Analysed %d/%d throws: %d accepted, %d rejected' %
                  (number, len(rows), len(accepted), len(rejected)), flush=True)
    out = Path(out); out.mkdir(parents=True, exist_ok=True)
    write_json(out/'extraction.json', dict(accepted=accepted, rejected=rejected))
    groups = sorted(set(r['cell_idx'] for r in accepted))
    if len(groups) < 12:
        raise ValueError('Fewer than 12 usable target cells; extraction.json records rejection reasons')
    pose = session['bb_pose']
    commands = local([r['target_global_mm'][:2] for r in accepted], pose)
    measured = local([r['landing_global_mm'][:2] for r in accepted], pose)
    cell_ids = np.array([r['cell_idx'] for r in accepted])
    c = np.array([commands[cell_ids == g].mean(axis=0) for g in groups])
    m = np.array([measured[cell_ids == g].mean(axis=0) for g in groups])
    forward, inverse = fit_forward(c, m)
    # Spatial held-out cells: no repeats of the validation target enter its fit.
    cv = np.zeros_like(m)
    shuffled = np.random.default_rng(91).permutation(len(groups))
    for test in np.array_split(shuffled, 5):
        train = np.setdiff1d(np.arange(len(groups)), test)
        f, _ = fit_forward(c[train], m[train])
        cv[test] = m[test]-np.c_[c[test], np.ones(len(test))]@f.T
    report = dict(accepted_throws=len(accepted), rejected_throws=rejected, cells=len(groups),
                  operator_excluded_throw_indices=list(exclude),
                  session_status=session.get('status'), recording_needs_review=session.get('recording_needs_review', False),
                  before=stats(measured-commands), cell_mean_before=stats(m-c),
                  held_out_cell_model_residual=stats(cv),
                  within_cell_repeatability=stats(measured-np.array([m[groups.index(g)] for g in cell_ids])),
                  forward_matrix=forward.tolist(), correction_matrix=inverse.tolist(),
                  note='Held-out model residual is not a physical corrected-throw validation. Re-throw a pilot after applying.')
    dense_ids = {r['cell_idx'] for r in session['plan']['cells'] if r.get('dense')}
    core = np.array([g in dense_ids for g in groups])
    if core.any():
        report['core_cell_mean_before'] = stats((m-c)[core])
        report['core_held_out_model_residual'] = stats(cv[core])
    write_json(out/'report.json', report)
    write_json(out/'correction_candidate.json', dict(matrix=inverse.tolist(), n_pairs=len(groups),
        provenance=dict(session_sha256=hashlib.sha256(session_path.read_bytes()).hexdigest(),
                        frame='BB-local XY mm', signed_s_mm=session['signed_s_mm'], bb_pose=pose,
                        target_bounds_bb_local_mm=[c.min(axis=0).tolist(), c.max(axis=0).tolist()],
                        catch_heights_global_mm=sorted(set(r['target_global_mm'][2] for r in accepted)),
                        method='equal-cell forward least squares, inverted; 5-fold held-out cells',
                        requires_corrected_positive_s=True, validated_on_hardware=False)))
    lines = ['# Local calibration analysis', '',
             '%d accepted throws; %d rejected; %d target cells.' % (len(accepted), len(rejected), len(groups)), '',
             'Uncorrected throw RMS: %.2f mm.' % report['before']['rms_mm'],
             'Held-out cell model residual RMS: %.2f mm.' % report['held_out_cell_model_residual']['rms_mm'],
             'Within-cell repeatability RMS: %.2f mm.' % report['within_cell_repeatability']['rms_mm'], '',
             'The candidate maps desired BB-local XY to commanded BB-local XY. It requires positive s and replaces the old affine; do not stack them.', '',
             report['note'], '', 'Inspect extraction.json for rejected throws and report.json for core-area metrics.']
    (out/'report.md').write_text('\n'.join(lines)+'\n', encoding='utf-8')
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('session', type=Path)
    parser.add_argument('--data', type=Path, help='MCAP bag directory/file, or simulated JSONL; default: session sibling bag/')
    parser.add_argument('--out', type=Path)
    parser.add_argument('--exclude', type=int, nargs='*', default=[], help='exclude known contaminated throw_idx values from session.json')
    args = parser.parse_args()
    report = analyse(args.session, args.data or args.session.parent/'bag', args.out or args.session.parent/'analysis', args.exclude)
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
