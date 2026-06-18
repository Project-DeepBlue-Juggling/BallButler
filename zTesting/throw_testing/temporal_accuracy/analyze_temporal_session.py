#!/usr/bin/env python3
"""Analyze temporal-accuracy sessions recorded by run_temporal_session.py.

Computes end-to-end arrival-error statistics (cone piezo impact time vs the
ThrowAnnouncement's predicted landing_time), estimates/removes the systematic
offset δ (cone-entry → piezo fall-through + release lag — protocol §Pass
criteria), and evaluates the ±10 ms residual-bias criterion per group.

    # Single session:
    /usr/bin/python3 analyze_temporal_session.py sessions/temporal_repeatability_*.json

    # Sweeps, with δ taken from the repeatability block:
    /usr/bin/python3 analyze_temporal_session.py \
        sessions/temporal_delay-sweep_*.json \
        --offset-from sessions/temporal_repeatability_<stamp>.json

Grouping: delay-sweep sessions group by commanded delay; distance-sweep
sessions group by the per-throw ``position`` label (pos01… — one interactive
run covers all positions; legacy one-position-per-file sessions fall back to
the session label); repeatability is a single group. Pass: |mean residual| <=
tolerance (default 10 ms) in every group. Plots (--plot) need matplotlib; the
text report is dependency-free.

Python 3.8 compatible (run with /usr/bin/python3 on the Jetson).
"""

from __future__ import annotations

import argparse
import json
import statistics
import sys
from pathlib import Path


def load_session(path):
    with open(path) as f:
        s = json.load(f)
    s['_path'] = str(path)
    return s


def good_throws(session):
    return [t for t in session.get('throws', []) if t.get('status') == 'ok']


def throw_counts(session):
    counts = {}
    for t in session.get('throws', []):
        counts[t.get('status', '?')] = counts.get(t.get('status', '?'), 0) + 1
    return counts


def group_key(session, throw):
    phase = session.get('phase', '?')
    if phase == 'delay-sweep':
        return 'delay %.2f s' % throw['commanded_delay_s']
    if phase == 'distance-sweep':
        return 'position %s' % (throw.get('position')
                                or session.get('label') or session['_path'])
    return phase


def linear_fit(xs, ys):
    """Least-squares slope/intercept; returns (slope, intercept) or None."""
    n = len(xs)
    if n < 3 or len(set(xs)) < 2:
        return None
    mx = sum(xs) / n
    my = sum(ys) / n
    sxx = sum((x - mx) ** 2 for x in xs)
    sxy = sum((x - mx) * (y - my) for x, y in zip(xs, ys))
    slope = sxy / sxx
    return slope, my - slope * mx


def group_context(throws):
    """Mean commanded speed + predicted ToF for a group. The δ components
    scale with these (release lag ~0.273/v, flight-stretch ~ToF — see
    README §systematic offset), so print them beside each group to make
    δ drift across positions diagnosable."""
    svc = [t.get('service') for t in throws]
    svc = [s for s in svc if s]
    if not svc:
        return ''
    v = statistics.mean(s['throw_speed_mps'] for s in svc)
    tof = statistics.mean(s['predicted_tof_s'] for s in svc)
    return '  (v %.2f m/s, ToF %.3f s)' % (v, tof)


def stats_line(name, errors, offset_ms, tol_ms):
    n = len(errors)
    mean = statistics.mean(errors)
    sd = statistics.stdev(errors) if n > 1 else 0.0
    resid = mean - offset_ms
    verdict = 'PASS' if abs(resid) <= tol_ms else 'FAIL'
    return ('  %-18s n=%-3d raw mean %+8.2f ms  σ %6.2f ms  '
            'residual %+7.2f ms  [%s]'
            % (name, n, mean, sd, resid, verdict)), resid, verdict


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('sessions', nargs='+', help='session JSON file(s)')
    ap.add_argument('--offset-from', metavar='JSON',
                    help='estimate δ from this session (typically the '
                         'repeatability block)')
    ap.add_argument('--offset', type=float, metavar='MS',
                    help='explicit δ in ms (overrides --offset-from)')
    ap.add_argument('--tolerance-ms', type=float, default=10.0,
                    help='residual-bias pass threshold (default %(default)s)')
    ap.add_argument('--plot', action='store_true',
                    help='save a PNG next to the first session file')
    args = ap.parse_args(argv)

    sessions = [load_session(p) for p in args.sessions]

    # ── Offset δ ───────────────────────────────────────────────
    if args.offset is not None:
        offset_ms = args.offset
        offset_src = 'explicit --offset'
    elif args.offset_from:
        ref = load_session(args.offset_from)
        ref_errs = [t['arrival_error_ms'] for t in good_throws(ref)]
        if len(ref_errs) < 5:
            print('ERROR: offset reference has only %d good throws (<5)'
                  % len(ref_errs))
            return 1
        offset_ms = statistics.mean(ref_errs)
        offset_src = '%s (n=%d)' % (args.offset_from, len(ref_errs))
    else:
        pooled = [t['arrival_error_ms'] for s in sessions for t in good_throws(s)]
        if not pooled:
            print('ERROR: no good throws in the given sessions')
            return 1
        offset_ms = statistics.mean(pooled)
        offset_src = ('pooled self-mean (n=%d) — NOTE: overall residual is 0 '
                      'by construction; only the per-group spread is '
                      'informative' % len(pooled))

    print('Temporal-accuracy analysis (protocol temporal-accuracy-v1)')
    print('Systematic offset δ = %+.2f ms   [from %s]' % (offset_ms, offset_src))
    print('Pass criterion: |mean residual| <= %.1f ms per group\n'
          % args.tolerance_ms)

    # ── Per-session / per-group stats ──────────────────────────
    overall_errs = []
    verdicts = []
    for s in sessions:
        counts = throw_counts(s)
        print('%s  (phase=%s%s)' % (s['_path'], s.get('phase'),
                                    ', label=' + s['label'] if s.get('label') else ''))
        print('  status counts: %s' % counts)
        excluded = sum(v for k, v in counts.items() if k != 'ok')
        if excluded:
            print('  NOTE: %d throw(s) excluded from stats (miss/unsynced/'
                  'failed are listed above, never silently dropped)' % excluded)

        groups = {}
        for t in good_throws(s):
            groups.setdefault(group_key(s, t), []).append(t)
        for name in sorted(groups):
            errs = [t['arrival_error_ms'] for t in groups[name]]
            line, _, verdict = stats_line(name, errs, offset_ms,
                                          args.tolerance_ms)
            print(line + group_context(groups[name]))
            verdicts.append(verdict)
            overall_errs.extend(errs)

        # Trend: arrival error vs commanded delay (scheduling-chain check)
        # and vs predicted ToF (flight-model check).
        ok = good_throws(s)
        for xname, xget in (('commanded delay', lambda t: t['commanded_delay_s']),
                            ('predicted ToF',
                             lambda t: t['service']['predicted_tof_s'])):
            fit = linear_fit([xget(t) for t in ok],
                             [t['arrival_error_ms'] for t in ok])
            if fit:
                print('  trend vs %-15s %+0.2f ms/s' % (xname + ':', fit[0]))
        print()

    if overall_errs:
        n = len(overall_errs)
        mean = statistics.mean(overall_errs)
        sd = statistics.stdev(overall_errs) if n > 1 else 0.0
        print('OVERALL: n=%d  raw mean %+.2f ms  σ %.2f ms  '
              'residual %+.2f ms' % (n, mean, sd, mean - offset_ms))
        print('RESULT: %s' % ('PASS' if all(v == 'PASS' for v in verdicts)
                              else 'FAIL'))

    if args.plot:
        _plot(sessions, offset_ms, Path(args.sessions[0]))
    return 0


def _plot(sessions, offset_ms, first_path):
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        print('matplotlib unavailable — skipping plot')
        return
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))
    # Distance sweeps vary ToF (delay is fixed) — use ToF on the x-axis there.
    xlabel = 'commanded delay (s)'
    for s in sessions:
        ok = good_throws(s)
        if s.get('phase') == 'distance-sweep':
            xs = [t['service']['predicted_tof_s'] for t in ok]
            xlabel = 'predicted ToF (s)'
        else:
            xs = [t['commanded_delay_s'] for t in ok]
        ys = [t['arrival_error_ms'] - offset_ms for t in ok]
        ax1.scatter(xs, ys, label=Path(s['_path']).stem, alpha=0.7)
        ax2.hist(ys, bins=15, alpha=0.5, label=Path(s['_path']).stem)
    ax1.axhline(0, color='k', lw=0.5)
    ax1.set_xlabel(xlabel)
    ax1.set_ylabel('residual arrival error (ms)')
    ax1.legend(fontsize=7)
    ax2.set_xlabel('residual arrival error (ms)')
    fig.suptitle('Temporal accuracy — residuals after δ=%+.2f ms' % offset_ms)
    out = first_path.with_suffix('.png')
    fig.tight_layout()
    fig.savefig(out, dpi=120)
    print('Plot saved: %s' % out)


if __name__ == '__main__':
    sys.exit(main())
