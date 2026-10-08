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


# Extraction gates. Measured on the 2026-10-07 hardware pilot (see
# LOCAL_CALIBRATION.md, "Trajectory extraction"): a gravity-only arc leaves
# 30-50 mm systematic residuals over a 0.9 s flight (drag plus a ~1 deg
# effective-gravity tilt), the ball is often occluded near the apex, and the
# arc passes BB's predicted release point ~35 ms before nominal release.
INLIER_MM = 8.            # final per-observation residual, free-acceleration model
MAX_RMS_MM = 5.
MIN_SAMPLES = 30
MIN_SPAN_S = .25          # observed flight span, and continuous final segment span
SEGMENT_GAP_S = .08       # max gap inside the continuous segment through the catch plane
MAX_OCCLUSION_S = .45     # max earlier gap (apex occlusion) bridged by the arc fit
CATCH_COVER_S = .015      # last observation at most this long before the crossing
ACCEL_TOLERANCE = .06     # |a - g| as a fraction of g: drag + tilt, not handling
LAUNCH_RADIUS_MM = 100.   # arc closest approach to the predicted release position
LAUNCH_TIME_S = .10       # ...at an arc time within this of nominal release
VELOCITY_TOLERANCE = .3   # release velocity vs predicted, fraction of |v|
LOCAL_BEFORE_S = .15      # local crossing fit window
LOCAL_AFTER_S = .04
LOCAL_MIN_SAMPLES = 15


def observations(path, skipped=None):
    """Yield (source stamp, all marker positions) per frame.

    QTM labels are NOT identity evidence: on the 2026-10-07 pilot the thrown
    ball carried rigid-body labels ('Base - 1/3/4/7') for part or all of its
    flight, so filtering on an empty label discarded it. Every marker is kept;
    the extractor's launch association and ballistic gates do the selection.
    """
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
                # Receipt time jitter corrupts ballistics: never substitute it.
                # Skip (and count) unsynchronised or clock-inconsistent frames;
                # a hole inside a flight then fails that throw's gap gates.
                if not t:
                    reason = 'unstamped'
                elif abs(t - msg.log_time * 1e-9) > 2:
                    reason = 'stamp_vs_record_clock_over_2s'
                else:
                    reason = None
                if reason:
                    if skipped is not None:
                        skipped[reason] = skipped.get(reason, 0) + 1
                    continue
                yield t, np.asarray([[m.position.x, m.position.y, m.position.z]
                                     for m in ros.markers], float).reshape(-1, 3)


class Rejection(ValueError):
    """A specific rejection reason plus per-gate diagnostics for extraction.json."""

    def __init__(self, reason, diagnostics=None):
        super().__init__(reason)
        self.diagnostics = diagnostics or {}


def crossing(coeff, z, gravity):
    """Descending crossing of z for a gravity-only [r, v] arc (kept for callers)."""
    disc = coeff[1, 2]**2 + 2 * gravity * (coeff[0, 2] - z)
    if disc <= 0:
        return None
    t = (coeff[1, 2] + math.sqrt(disc)) / gravity
    return t if t > 0 else None


def _quadratic_crossing(coeff, z):
    """Descending root of r + v t + a t^2/2 = z, for coeff rows [r, v, a]."""
    a, v, r = coeff[2, 2], coeff[1, 2], coeff[0, 2] - z
    if a >= 0:
        return None
    disc = v*v - 2*a*r
    if disc <= 0:
        return None
    t = (-v - math.sqrt(disc)) / a
    return t if t > 0 else None


def _position(coeff, t):
    t = np.asarray(t, float)
    return coeff[0] + t[..., None]*coeff[1] + .5*(t*t)[..., None]*coeff[2]


def _one_per_frame(mask, residual, ids):
    # Multiple nearby reflections in a frame must not overweight that frame.
    ix = np.flatnonzero(mask)
    ordered = ix[np.argsort(residual[ix])]
    _, first = np.unique(ids[ordered], return_index=True)
    out = np.zeros_like(mask)
    out[ordered[first]] = True
    return out


def _segments(ts, gap):
    breaks = np.flatnonzero(np.diff(ts) > gap)
    return np.split(ts, breaks + 1)


def extract(row, frames, gravity=9806.):
    """Measure one BB flight's descending catch-plane crossing from raw markers.

    Labels/track IDs are ignored. A candidate track must (1) extrapolate back
    through BB's predicted release position at about the nominal release time
    (launch association, independent of the target), (2) be a free flight:
    constant-acceleration fit within ACCEL_TOLERANCE of gravity and tight
    residuals, (3) be observed continuously through the catch plane. The
    crossing XY comes from a local fit around the crossing, never a
    prediction. Competing plausible tracks are rejected as ambiguous.
    Raises Rejection (a ValueError) with a specific reason and diagnostics.
    """
    origin = np.asarray(row['predicted_release_position_mm'], float)
    velocity = np.asarray(row['predicted_release_velocity_mm_s'], float)
    epoch = row['nominal_release_wall_s']
    z = row['target_global_mm'][2]
    g_vec = np.array([0., 0., -gravity])
    times, points, frame_ids = [], [], []
    last = None
    for frame, (stamp, pts) in enumerate(sorted(frames, key=lambda f: f[0])):
        if stamp == last:
            continue  # the 200 Hz publisher can repeat a 300 Hz QTM frame
        last = stamp
        t = stamp - epoch
        if t < -.05:
            continue
        for p in np.asarray(pts, float).reshape(-1, 3):
            if np.isfinite(p).all() and p[2] > z - 120:
                times.append(t); points.append(p); frame_ids.append(frame)
    diagnostics = dict(observations=len(times))
    if len(times) < MIN_SAMPLES:
        raise Rejection('insufficient airborne observations (%d above catch plane - 120 mm)' % len(times), diagnostics)
    t = np.asarray(times); p = np.asarray(points); ids = np.asarray(frame_ids)
    q = p.copy(); q[:, 2] += .5*gravity*t*t      # gravity-compensated: linear in t
    lin = np.c_[np.ones(len(t)), t]
    quad = np.c_[lin, .5*t*t]
    gates = ['velocity', 'samples', 'acceleration', 'residual', 'launch',
             'occlusion', 'catch_segment', 'catch_coverage', 'local_fit']
    counts = dict((g, 0) for g in gates)
    furthest = [-1, None]

    def fail(gate, **info):
        counts[gate] += 1
        k = gates.index(gate)
        if k > furthest[0]:
            furthest[0], furthest[1] = k, dict(info, gate=gate)
        elif k == furthest[0] and gate == 'launch' and info.get('closest_mm', 1e9) < furthest[1].get('closest_mm', 1e9):
            furthest[1] = dict(info, gate=gate)
        return None

    def evaluate(coeff):
        """Refine a gravity-constrained [r, v] seed and apply every gate."""
        if np.linalg.norm(coeff[1]-velocity) > VELOCITY_TOLERANCE*np.linalg.norm(velocity):
            return fail('velocity')
        full = np.r_[coeff, g_vec[None]]
        horizon = _quadratic_crossing(full, z)
        if horizon is None:
            return fail('samples')
        mask = None
        # Coarse association on the gravity-only arc, then the free-acceleration
        # model at the tight final threshold.
        for radius, model in ((60., 'lin'), (25., 'lin'), (15., 'quad'), (INLIER_MM, 'quad'), (INLIER_MM, 'quad')):
            residual = np.linalg.norm(p - _position(full, t), axis=1)
            mask = _one_per_frame((residual < radius) & (t <= horizon + LOCAL_AFTER_S) & (t >= -.05), residual, ids)
            if mask.sum() < MIN_SAMPLES:
                return fail('samples', n=int(mask.sum()))
            if model == 'lin':
                c = np.linalg.lstsq(lin[mask], q[mask], rcond=None)[0]
                full = np.r_[c, g_vec[None]]
            else:
                full = np.linalg.lstsq(quad[mask], p[mask], rcond=None)[0]
            horizon = _quadratic_crossing(full, z)
            if horizon is None:
                return fail('samples')
        residual = np.linalg.norm(p - _position(full, t), axis=1)
        mask = _one_per_frame((residual < INLIER_MM) & (t <= horizon + LOCAL_AFTER_S) & (t >= -.05), residual, ids)
        n = int(mask.sum())
        if n < MIN_SAMPLES:
            return fail('samples', n=n)
        full = np.linalg.lstsq(quad[mask], p[mask], rcond=None)[0]
        end = _quadratic_crossing(full, z)
        if end is None:
            return fail('samples')
        accel_error = float(np.linalg.norm(full[2] - g_vec))
        if accel_error > ACCEL_TOLERANCE*gravity:
            return fail('acceleration', accel_error_mm_s2=round(accel_error, 1))
        rms = float(np.sqrt(np.mean(np.sum((p[mask] - quad[mask] @ full)**2, axis=1))))
        if rms > MAX_RMS_MM:
            return fail('residual', rms_mm=round(rms, 2))
        grid = np.linspace(-.2, .2, 801)
        dist = np.linalg.norm(_position(full, grid) - origin, axis=1)
        k = int(np.argmin(dist))
        closest, tau = float(dist[k]), float(grid[k])
        if closest > LAUNCH_RADIUS_MM or abs(tau) > LAUNCH_TIME_S:
            return fail('launch', closest_mm=round(closest, 1), arc_time_offset_s=round(tau, 4))
        if np.linalg.norm(full[1] + tau*full[2] - velocity) > VELOCITY_TOLERANCE*np.linalg.norm(velocity):
            return fail('velocity')
        ts = np.unique(t[mask])
        if ts[-1]-ts[0] < MIN_SPAN_S:
            return fail('samples', span_s=round(float(ts[-1]-ts[0]), 3))
        gaps = np.diff(ts)
        if gaps.max() > MAX_OCCLUSION_S:
            return fail('occlusion', max_gap_s=round(float(gaps.max()), 3))
        final = _segments(ts, SEGMENT_GAP_S)[-1]
        if final[-1]-final[0] < MIN_SPAN_S or not final[0] < end:
            return fail('catch_segment', final_segment_s=[round(float(final[0]), 3), round(float(final[-1]), 3)],
                        catch_time_s=round(end, 3))
        if final[-1] < end - CATCH_COVER_S:
            return fail('catch_coverage', last_observation_s=round(float(final[-1]), 3), catch_time_s=round(end, 3))
        # Local crossing: re-fit position/velocity near the crossing with the
        # track's own acceleration held fixed. A single constant-acceleration
        # arc is ~10 mm off near its ends; locally the model error is < 1 mm.
        near = mask & (t >= end - LOCAL_BEFORE_S) & (t <= end + LOCAL_AFTER_S)
        if near.sum() < LOCAL_MIN_SAMPLES:
            return fail('local_fit', n=int(near.sum()))
        tl = t[near]; corrected = p[near] - .5*(tl*tl)[:, None]*full[2]
        c = np.linalg.lstsq(np.c_[np.ones(len(tl)), tl], corrected, rcond=None)[0]
        local = np.r_[c, full[2][None]]
        local_rms = float(np.sqrt(np.mean(np.sum((p[near] - _position(local, tl))**2, axis=1))))
        local_end = _quadratic_crossing(local, z)
        if local_end is None or local_rms > MAX_RMS_MM or abs(local_end - end) > .02:
            return fail('local_fit', local_rms_mm=round(local_rms, 2))
        xy = _position(local, local_end)[:2]
        return dict(n=n, rms=rms, xy=xy, end=local_end, coeff=full, mask=mask, closest=closest, tau=tau,
                    accel_error=accel_error, ts=ts, final=final, local_rms=local_rms, local_n=int(near.sum()),
                    global_xy=_position(full, end)[:2], max_gap=float(gaps.max()))

    # Seeds: BB's predicted release position as an anchor at a few plausible
    # arc times, through each later observation. Free in velocity, hence in
    # landing XY: no target or predicted-landing proximity is used.
    seeds = np.flatnonzero((t >= .1))
    if len(seeds) > 300:
        seeds = seeds[np.linspace(0, len(seeds)-1, 300).astype(int)]
    candidates, seen = [], set()
    for tau0 in (-.1, -.05, 0., .05):
        anchor = origin + np.array([0., 0., .5*gravity*tau0*tau0])
        for i in seeds:
            v = (q[i] - anchor)/(t[i] - tau0)
            result = evaluate(np.array([anchor - tau0*v, v]))
            if result is None:
                continue
            key = tuple(np.flatnonzero(result['mask'])[::5])
            if key in seen:
                continue
            seen.add(key)
            candidates.append(result)
    diagnostics['gate_failures'] = dict((k, v) for k, v in counts.items() if v)
    if not candidates:
        if furthest[1] is None:
            raise Rejection('no launch-associated ballistic track (no seed reached the gates)', diagnostics)
        diagnostics['best_failure'] = furthest[1]
        detail = ', '.join('%s=%s' % (k, v) for k, v in sorted(furthest[1].items()) if k != 'gate')
        raise Rejection('no launch-to-catch ballistic track passed quality gates; furthest gate reached: %s%s'
                        % (furthest[1]['gate'], ' (%s)' % detail if detail else ''), diagnostics)
    candidates.sort(key=lambda c: (-c['n'], c['rms']))
    best = candidates[0]
    for other in candidates[1:]:
        shared = (best['mask'] & other['mask']).sum() / float(min(best['n'], other['n']))
        if other['n'] >= .8*best['n'] and shared < .5 and np.linalg.norm(other['xy']-best['xy']) > 10:
            diagnostics['competing_landing_xy_mm'] = [best['xy'].tolist(), other['xy'].tolist()]
            raise Rejection('ambiguous competing launch trajectories', diagnostics)
    return dict(landing_global_mm=[*best['xy'].tolist(), z], samples=best['n'],
                fit_rms_mm=best['rms'], local_fit_rms_mm=best['local_rms'], local_samples=best['local_n'],
                catch_wall_s=epoch+best['end'], catch_time_s=best['end'],
                observed_span_s=[float(best['ts'][0]), float(best['ts'][-1])],
                final_segment_s=[float(best['final'][0]), float(best['final'][-1])],
                max_gap_s=best['max_gap'], accel_mm_s2=best['coeff'][2].tolist(),
                accel_error_mm_s2=best['accel_error'],
                launch_closest_mm=best['closest'], arc_time_offset_s=best['tau'],
                global_fit_landing_xy_mm=best['global_xy'].tolist(),
                release_fit=best['coeff'].tolist())


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


POOLED_KEYS = ('plan_sha256', 'solver_sha256', 'signed_s_mm', 'schedule_to_mocap_mm')


def parse_exclusions(values, sessions):
    """`IDX` excludes throw_idx IDX in every session; `K:IDX` only in session K (0-based)."""
    out = set()
    for value in values:
        text = str(value)
        if ':' in text:
            k, idx = text.split(':', 1)
            if not 0 <= int(k) < sessions:
                raise ValueError('--exclude %s: no session %s' % (text, k))
            out.add((int(k), int(idx)))
        else:
            out.update((k, int(text)) for k in range(sessions))
    return out


def load_sessions(paths):
    """Sessions to pool must share plan, solver, signed s and frame translation.

    BB pose may differ between sessions (e.g. recalibrated after a restart):
    each throw is mapped to BB-local XY with its own session's pose.
    """
    sessions = []
    for path in paths:
        session = json.loads(Path(path).read_text(encoding='utf-8'))
        if session.get('affine_applied') is not False or session.get('signed_s_mm', 0) <= 0:
            raise ValueError('Expected positive-s calibration with affine disabled: %s' % path)
        sessions.append(session)
    for key in POOLED_KEYS:
        if len(set(json.dumps(s.get(key)) for s in sessions)) > 1:
            raise ValueError('Sessions differ in %s; they cannot be pooled' % key)
    return sessions


def extract_session(k, session, data_path, exclude, gravity):
    """Extract every released throw of one session (k = its position in the pool)."""
    keep = lambda r: r.get('status') == 'released' and (k, r['throw_idx']) not in exclude
    tag = lambda r: dict(session_index=k, throw_idx=r['throw_idx'], cell_idx=r['cell_idx'])
    rows = [r for r in session['throws'] if keep(r) and r.get('capture_complete')]
    # Released but not cleanly captured (stream gap, QTM restart, stopped
    # session): listed so the report accounts for every BB release.
    rejected = [dict(tag(r), reason='capture incomplete: ' + r.get('capture_rejected',
                     'session ended before capture completed (%s)' % session.get('error', session.get('status'))))
                for r in session['throws'] if keep(r) and not r.get('capture_complete')]
    rows.sort(key=lambda r: r['analysis_window_wall_s'][0])
    starts = [r['analysis_window_wall_s'][0] for r in rows]
    frames = {r['throw_idx']: [] for r in rows}
    skipped = {}
    for t, pts in observations(data_path, skipped):
        j = bisect.bisect_right(starts, t)-1
        if j >= 0 and t < rows[j]['analysis_window_wall_s'][1]:
            idx = analysis_throw_at(session, t)
            if idx in frames:
                frames[idx].append((t, pts))
    accepted = []
    for number, row in enumerate(rows, 1):
        try:
            accepted.append(dict(row, session_index=k, **extract(row, frames[row['throw_idx']], gravity)))
        except ValueError as error:
            rejected.append(dict(tag(row), reason=str(error), diagnostics=getattr(error, 'diagnostics', {})))
        if number % 25 == 0 or number == len(rows):
            print('Session %d: analysed %d/%d throws: %d accepted, %d rejected' %
                  (k, number, len(rows), len(accepted), len(rejected)), flush=True)
    return accepted, rejected, skipped


def analyse(session_paths, data_paths, out, exclude=(), extract_only=False):
    """Extract (and, unless extract_only, fit) one session or a pool of resumed sessions."""
    single = isinstance(session_paths, (str, Path))
    session_paths = [Path(session_paths)] if single else [Path(p) for p in session_paths]
    data_paths = [Path(data_paths)] if isinstance(data_paths, (str, Path)) else [Path(p) for p in data_paths]
    if len(data_paths) != len(session_paths):
        raise ValueError('Need one data path per session')
    sessions = load_sessions(session_paths)
    exclusions = parse_exclusions(exclude, len(sessions))
    accepted, rejected, summaries = [], [], []
    for k, (path, session, data) in enumerate(zip(session_paths, sessions, data_paths)):
        gravity = session.get('hardware_constants', {}).get('GRAVITY_MPS2', 9.806)*1000
        a, r, skipped = extract_session(k, session, data, exclusions, gravity)
        accepted += a; rejected += r
        summaries.append(dict(path=str(path), status=session.get('status'),
                              recording_needs_review=session.get('recording_needs_review', False),
                              accepted=len(a), rejected=len(r), frames_skipped=skipped,
                              abandoned_not_settled=session.get('abandoned_throw_indices', [])))
    status = lambda key: (summaries[0][key] if len(summaries) == 1 else [s[key] for s in summaries])
    common = dict(sessions=summaries, session_status=status('status'),
                  recording_needs_review=status('recording_needs_review'))
    out = Path(out); out.mkdir(parents=True, exist_ok=True)
    write_json(out/'extraction.json', dict(accepted=accepted, rejected=rejected))
    if extract_only:
        report = dict(accepted_throws=len(accepted), rejected_throws=rejected,
                      mode='extraction_only_no_affine', **common)
        if accepted:
            report['before'] = stats(np.asarray([r['landing_global_mm'][:2] for r in accepted]) -
                                     np.asarray([r['target_global_mm'][:2] for r in accepted]))
        write_json(out/'extraction_report.json', report)
        return report
    groups = sorted(set(r['cell_idx'] for r in accepted))
    if len(groups) < 12:
        raise ValueError('Fewer than 12 usable target cells; extraction.json records rejection reasons')
    poses = [s['bb_pose'] for s in sessions]
    commands = np.array([local([r['target_global_mm'][:2]], poses[r['session_index']])[0] for r in accepted])
    measured = np.array([local([r['landing_global_mm'][:2]], poses[r['session_index']])[0] for r in accepted])
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
                  operator_excluded=sorted([list(e) for e in exclusions]), **common)
    report.update(before=stats(measured-commands), cell_mean_before=stats(m-c),
                  held_out_cell_model_residual=stats(cv),
                  within_cell_repeatability=stats(measured-np.array([m[groups.index(g)] for g in cell_ids])),
                  forward_matrix=forward.tolist(), correction_matrix=inverse.tolist(),
                  note='Held-out model residual is not a physical corrected-throw validation. Re-throw a pilot after applying.')
    dense_ids = {r['cell_idx'] for r in sessions[0]['plan']['cells'] if r.get('dense')}
    core = np.array([g in dense_ids for g in groups])
    if core.any():
        report['core_cell_mean_before'] = stats((m-c)[core])
        report['core_held_out_model_residual'] = stats(cv[core])
    write_json(out/'report.json', report)
    hashes = [hashlib.sha256(p.read_bytes()).hexdigest() for p in session_paths]
    write_json(out/'correction_candidate.json', dict(matrix=inverse.tolist(), n_pairs=len(groups),
        provenance=dict(session_sha256=hashes[0] if len(hashes) == 1 else hashes,
                        frame='BB-local XY mm', signed_s_mm=sessions[0]['signed_s_mm'],
                        bb_pose=poses[0] if len(poses) == 1 else poses,
                        solver_sha256=sessions[0].get('solver_sha256'),
                        target_bounds_bb_local_mm=[c.min(axis=0).tolist(), c.max(axis=0).tolist()],
                        catch_heights_global_mm=sorted(set(r['target_global_mm'][2] for r in accepted)),
                        method='equal-cell forward least squares, inverted; 5-fold held-out cells',
                        requires_corrected_positive_s=True, validated_on_hardware=False)))
    lines = ['# Local calibration analysis', '',
             '%d accepted throws; %d rejected; %d target cells; %d session(s).'
             % (len(accepted), len(rejected), len(groups), len(sessions)), '',
             'Uncorrected throw RMS: %.2f mm.' % report['before']['rms_mm'],
             'Held-out cell model residual RMS: %.2f mm.' % report['held_out_cell_model_residual']['rms_mm'],
             'Within-cell repeatability RMS: %.2f mm.' % report['within_cell_repeatability']['rms_mm'], '',
             'The candidate maps desired BB-local XY to commanded BB-local XY. It requires positive s and replaces the old affine; do not stack them.', '',
             report['note'], '', 'Inspect extraction.json for rejected throws and report.json for core-area metrics.']
    (out/'report.md').write_text('\n'.join(lines)+'\n', encoding='utf-8')
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('session', type=Path, nargs='+',
                        help='session.json; several (a session and its --resume-from parts) are pooled')
    parser.add_argument('--data', type=Path, nargs='+',
                        help='MCAP bag directory/file, or simulated JSONL, one per session; default: each session sibling bag/')
    parser.add_argument('--out', type=Path, help='default: <session>/analysis; required when pooling sessions')
    parser.add_argument('--exclude', nargs='*', default=[],
                        help='contaminated throws: IDX (throw_idx in every session) or K:IDX (session K, 0-based)')
    parser.add_argument('--extract-only', action='store_true', help='check pilot trajectories and misses without fitting an affine (no 12-cell minimum)')
    args = parser.parse_args()
    if len(args.session) > 1 and args.out is None:
        parser.error('--out is required when pooling sessions')
    data = args.data or [path.parent/'bag' for path in args.session]
    try:
        report = analyse(args.session, data, args.out or args.session[0].parent/'analysis', args.exclude, args.extract_only)
    except ValueError as error:
        parser.exit(1, str(error)+'\n')
    print(json.dumps(report, indent=2))

if __name__ == '__main__':
    main()
