#!/usr/bin/env python3
"""Settle BB's yaw-offset gauge from a CORRECTED validation session (the "item 2" sitting).

Why: the deployed aim correction (throw_affine_correction.json) was fitted in one
mocap frame, session A's (yaw offset 0.208 deg, 2026-10-09). The constellation
estimator pins its gauge to that frame from recorded data, with about +-0.2 deg
of uncertainty. The throws settle it: an offset error e makes every landing's
bearing about BB's yaw axis read -e in the frame the node used (session B,
2026-10-09: used 0.681 deg, frame 0.208 deg, mean bearing error -0.47 deg).
So the implied frame offset is  used + mean_bearing_error, and the gauge pin
(gauge.pinned_yaw_offset_deg in the Jugglebot resource bb_marker_template.json)
moves by  +mean_bearing_error.

Pre-registered rule (2026-10-09, before the sitting): with at least MIN_ACCEPTED
accepted corrected throws, |mean bearing error| <= BEARING_TOLERANCE_DEG confirms
the frame; otherwise re-pin by the mean. The RMS checks of extraction_report.json
say separately whether the affine itself still holds; a RE_PIN never needs a refit
unless those fail after the re-pin.

Usage (after analyze_local_calibration.py <session>/session.json --extract-only):
    /usr/bin/python3 settle_yaw_gauge.py <session>/session.json [<other session>/session.json ...]
Several sessions pool their accepted throws (each in its own session's BB pose and
yaw-offset gauge; the node's used offset is reported from the first). The settlement
is written next to the first session; the affine RMS note comes from its report.
"""
import argparse
import json
import math
from pathlib import Path

import numpy as np

BEARING_TOLERANCE_DEG = 0.15   # ~2.5 mm at the 1 m typical range
MIN_ACCEPTED = 30


def bb_local_xy(point, pose):
    dx = point[0] - pose['position_mm'][0]
    dy = point[1] - pose['position_mm'][1]
    c, s = math.cos(pose['yaw_offset_rad']), math.sin(pose['yaw_offset_rad'])
    return dx * c + dy * s, -dx * s + dy * c


def wrap_deg(a):
    return (a + 180.0) % 360.0 - 180.0


def bearing_rows(accepted, pose):
    """Per accepted throw: bearing error (landing - desired, about BB's yaw axis,
    degrees, in the frame the node used), lateral and radial error (mm)."""
    rows = []
    for t in accepted:
        xt, yt = t['target_bb_local_mm'][:2]
        xl, yl = bb_local_xy(t['landing_global_mm'], pose)
        rng = math.hypot(xt, yt)
        ux, uy = xt / rng, yt / rng
        ex, ey = xl - xt, yl - yt
        rows.append(dict(throw_idx=t['throw_idx'], range_mm=rng,
                         bearing_deg=wrap_deg(math.degrees(math.atan2(yl, xl) - math.atan2(yt, xt))),
                         lateral_mm=-ex * uy + ey * ux, radial_mm=ex * ux + ey * uy))
    return rows


def settle(session, rows, report=None, tolerance_deg=BEARING_TOLERANCE_DEG, min_accepted=MIN_ACCEPTED):
    if not session.get('correction'):
        raise ValueError('Not a corrected session: the uncorrected aim bias would be read as a frame error')
    used = math.degrees(session['bb_pose']['yaw_offset_rad'])
    out = dict(n=len(rows), used_yaw_offset_deg=used, tolerance_deg=tolerance_deg, min_accepted=min_accepted,
               correction_sha256=session['correction']['sha256'])
    if len(rows) < min_accepted:
        out.update(verdict='INCONCLUSIVE', reason='%d accepted throws; need %d' % (len(rows), min_accepted))
        return out
    n = len(rows)
    b = np.array([r['bearing_deg'] for r in rows])
    lat = np.array([r['lateral_mm'] for r in rows])
    rng = np.array([r['range_mm'] for r in rows])
    mean_b = float(b.mean())
    out.update(bearing_mean_deg=mean_b, bearing_se_deg=float(b.std(ddof=1) / math.sqrt(n)),
               lateral_mean_mm=float(lat.mean()), lateral_se_mm=float(lat.std(ddof=1) / math.sqrt(n)),
               mean_range_mm=float(rng.mean()))
    # Rotation (slope in range) against translation (intercept): a frame error is pure slope.
    A = np.c_[rng, np.ones(n)]
    coef, _, _, _ = np.linalg.lstsq(A, lat, rcond=None)
    resid = lat - A @ coef
    cov = np.linalg.inv(A.T @ A) * float(resid.var(ddof=2))
    out['rotation_fit'] = dict(slope_deg=math.degrees(math.atan(coef[0])), slope_se_deg=math.degrees(math.sqrt(cov[0, 0])),
                               intercept_mm=float(coef[1]), intercept_se_mm=float(math.sqrt(cov[1, 1])))
    out['implied_frame_yaw_offset_deg'] = used + mean_b
    out['gauge_delta_deg'] = mean_b
    if report and report.get('validation'):
        v = report['validation']
        out['affine_rms_mm'] = v.get('rms_mm')
        # An INCONCLUSIVE validation (too few accepted throws) carries no 'checks'.
        checks = v.get('checks') or {}
        out['affine_rms_ok'] = bool(checks.get('rms')) and bool(checks.get('core_rms', True))
        out['affine_verdict'] = v.get('verdict')
    out['verdict'] = 'FRAME_CONFIRMED' if abs(mean_b) <= tolerance_deg else 'RE_PIN'
    return out


def describe(result):
    lines = ['Yaw-offset gauge settlement: %s' % result['verdict']]
    if result['verdict'] == 'INCONCLUSIVE':
        return lines + ['  ' + result['reason']]
    lines += ['  accepted throws %d, mean range %.0f mm' % (result['n'], result['mean_range_mm']),
              '  node used yaw offset %.3f deg' % result['used_yaw_offset_deg'],
              '  mean bearing error %+.3f +- %.3f deg (lateral %+.1f +- %.1f mm)'
              % (result['bearing_mean_deg'], result['bearing_se_deg'], result['lateral_mean_mm'], result['lateral_se_mm']),
              '  rotation vs translation: slope %+.3f +- %.3f deg, intercept %+.1f +- %.1f mm'
              % (result['rotation_fit']['slope_deg'], result['rotation_fit']['slope_se_deg'],
                 result['rotation_fit']['intercept_mm'], result['rotation_fit']['intercept_se_mm']),
              '  implied frame yaw offset %.3f deg (tolerance +-%.2f deg)'
              % (result['implied_frame_yaw_offset_deg'], result['tolerance_deg'])]
    if result['verdict'] == 'RE_PIN':
        lines += ['  ACTION: add %+.3f deg to gauge.pinned_yaw_offset_deg in Jugglebot '
                  'ros_ws/src/jugglebot/resources/bb_marker_template.json, rebuild, re-run the sweep '
                  'and confirm the node reports ~%.3f deg' % (result['gauge_delta_deg'], result['implied_frame_yaw_offset_deg'])]
    else:
        lines += ['  no re-pin needed']
    if 'affine_rms_ok' in result:
        lines += ['  affine per-throw RMS %.1f mm: %s' % (result['affine_rms_mm'], 'within criteria' if result['affine_rms_ok']
                  else 'OUTSIDE criteria: if it stays outside after the re-pin, refit the affine')]
    return lines


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    p.add_argument('session_json', nargs='+',
                   help='session.json of the sitting; more than one pools their accepted throws')
    p.add_argument('--tolerance-deg', type=float, default=BEARING_TOLERANCE_DEG)
    p.add_argument('--min-accepted', type=int, default=MIN_ACCEPTED)
    args = p.parse_args(argv)
    paths = [Path(x) for x in args.session_json]
    session_path = paths[0]
    session = json.loads(session_path.read_text())
    analysis = session_path.parent / 'analysis'
    report_path = analysis / 'extraction_report.json'
    report = json.loads(report_path.read_text()) if report_path.is_file() else None
    rows, pooled = [], []
    for path in paths:
        sess = json.loads(path.read_text())
        acc = json.loads((path.parent / 'analysis' / 'extraction.json').read_text())['accepted']
        rows.extend(bearing_rows(acc, sess['bb_pose']))
        pooled.append(dict(session=str(path.resolve()), accepted=len(acc),
                           used_yaw_offset_deg=math.degrees(sess['bb_pose']['yaw_offset_rad'])))
    result = settle(session, rows, report, args.tolerance_deg, args.min_accepted)
    result['session'] = str(session_path.resolve())
    if len(paths) > 1:
        result['pooled_sessions'] = pooled
    (analysis / 'yaw_gauge_settlement.json').write_text(json.dumps(result, indent=2) + '\n')
    print('\n'.join(describe(result)))
    print('written %s' % (analysis / 'yaw_gauge_settlement.json'))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
