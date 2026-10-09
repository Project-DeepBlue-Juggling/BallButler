import importlib.util
import json
import tempfile
import unittest
from pathlib import Path
import numpy as np
from analyze_local_calibration import Rejection, extract, fit_forward, observations


def hardware_like_flight(origin, velocity, release=-.045, gravity=(80., 170., -9740.), drag=2.7e-5,
                         occluded=(.1, .42), stop=None, seed=3):
    """Frames reproducing the 2026-10-07 pilot, relative to nominal release t=0.

    Measured there: the arc passes BB's predicted release point ~45 ms before
    nominal release; a tilted effective gravity (+80, +170, -9740 mm/s^2) plus
    quadratic drag (k ~ 2.7e-5 /mm) leaves 30-50 mm residuals on a gravity-only
    arc; the ball is invisible near the apex; it rests in the hand before the
    stroke; 300 Hz QTM frames are republished at 200 Hz, so stamps repeat.
    Returns (frames, true descending catch-plane XY at z=830, wall epoch).
    """
    rng = np.random.default_rng(seed)
    gravity = np.asarray(gravity); dt = 1e-4
    pos, vel = np.array(origin, float), np.array(velocity, float)
    track, truth, t = [], None, release
    while pos[2] > 0:
        track.append((t, pos.copy()))
        acc = gravity - drag*np.linalg.norm(vel)*vel
        new = pos + vel*dt + .5*acc*dt*dt
        if truth is None and pos[2] >= 830 > new[2] and vel[2] < 0:
            truth = pos[:2] + (new[:2]-pos[:2])*(pos[2]-830)/(pos[2]-new[2])
        pos, vel, t = new, vel + acc*dt, t + dt
    times = np.array([x[0] for x in track]); path = np.array([x[1] for x in track])
    epoch = 100.
    frames = []
    for k in range(int(round((1.8+.25)*300))):
        stamp = -.25 + k/300.
        points = [[-1000, 0, 1500], [50, 60, 2], [-790, 740, 2300]]
        if stamp < release - .07:
            points.append(np.asarray(origin) - [0, 0, 30])       # ball resting in the hand
        elif stamp >= release and not occluded[0] <= stamp <= occluded[1] and (stop is None or stamp < stop):
            j = np.searchsorted(times, stamp)
            if j < len(times):
                points.append(path[j])
        frame = (epoch+stamp, np.asarray(points, float) + rng.uniform(-1.5, 1.5, (len(points), 3)))
        frames.append(frame)
        if k % 2 == 0:
            frames.append(frame)                                # republished frame, same stamp
    return frames, truth, epoch


class AnalysisTests(unittest.TestCase):
    def setUp(self):
        self.r = np.array([-900., -200., 1800.])
        self.v = np.array([1100., 250., 3000.])
        self.row = dict(throw_idx=1, predicted_release_position_mm=self.r.tolist(),
                        predicted_release_velocity_mm_s=self.v.tolist(),
                        nominal_release_wall_s=100., target_global_mm=[0., 0., 830.])
        self.end = (3000+np.sqrt(3000**2+2*9806*970))/9806

    def frames(self, second=False):
        rng = np.random.default_rng(2)
        frames = []
        for t in np.arange(0, 1.8, .005):
            p = self.r+self.v*t-np.array([0, 0, .5*9806*t*t])
            points = [[-1000, 0, 1500], [50, 60, 2]]
            if p[2] >= 0:
                points.append(p+rng.uniform(-2, 2, 3))
                if second:
                    points.append(p+np.array([50, 0, 0])+rng.uniform(-2, 2, 3))
            else:
                points.append([200, 100, max(0, 25-100*(t-1.1)**2)])
            frames.append((100+t, np.array(points)))
        return frames

    def test_noise_clutter_bounce_and_large_miss(self):
        result = extract(self.row, self.frames())
        expected = self.r[:2]+self.v[:2]*self.end
        np.testing.assert_allclose(result['landing_global_mm'][:2], expected, atol=.6)

    def test_ambiguous_launch_rejected(self):
        with self.assertRaisesRegex(ValueError, 'ambiguous'):
            extract(self.row, self.frames(second=True))

    def test_no_ball_and_occlusion_rejected(self):
        with self.assertRaises(ValueError):
            extract(self.row, [(t, p[:2]) for t, p in self.frames()])
        with self.assertRaises(ValueError):
            extract(self.row, [(t, p) for t, p in self.frames() if t < 100.5])

    def hardware_row(self):
        return dict(self.row, predicted_release_position_mm=[-999.6, -286.8, 1879.9],
                    predicted_release_velocity_mm_s=[864.9, 211.2, 3131.5])

    def test_hardware_pilot_flight_recovered(self):
        # Regression: every gate of the gravity-only, launch-window extractor
        # failed on this shape (2026-10-07 pilot, throws 0-9).
        row = self.hardware_row()
        frames, truth, _ = hardware_like_flight(row['predicted_release_position_mm'],
                                                row['predicted_release_velocity_mm_s'])
        result = extract(row, frames)
        np.testing.assert_allclose(result['landing_global_mm'][:2], truth, atol=1.5)
        self.assertLess(result['fit_rms_mm'], 5)
        self.assertAlmostEqual(result['arc_time_offset_s'], -.045, delta=.01)
        self.assertLess(result['launch_closest_mm'], 10)
        self.assertGreater(result['max_gap_s'], .3)              # bridged apex occlusion

    def test_arc_not_launched_by_bb_rejected(self):
        # A projectile inside the capture window that never passed BB's release
        # point (e.g. a stray refill ball) must not be measured.
        row = self.hardware_row()
        start = np.asarray(row['predicted_release_position_mm']) + [0, 400, -150]
        frames, _, _ = hardware_like_flight(start, row['predicted_release_velocity_mm_s'])
        with self.assertRaises(Rejection) as caught:
            extract(row, frames)
        self.assertIn('launch', str(caught.exception))
        self.assertIn('best_failure', caught.exception.diagnostics)

    def test_unobserved_catch_crossing_rejected_not_extrapolated(self):
        row = self.hardware_row()
        frames, _, _ = hardware_like_flight(row['predicted_release_position_mm'],
                                            row['predicted_release_velocity_mm_s'], stop=.8)
        with self.assertRaisesRegex(Rejection, 'catch_coverage|catch_segment'):
            extract(row, frames)

    def test_rejected_capture_reported_not_analysed(self):
        from analyze_local_calibration import analyse
        row = dict(self.hardware_row(), cell_idx=7, status='released', capture_complete=False,
                   capture_rejected='mocap source-stamp gap 0.150 s exceeded 0.100 s',
                   analysis_window_wall_s=[99.75, 101.5])
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)
            (path/'session.json').write_text(json.dumps(dict(affine_applied=False, signed_s_mm=105.65,
                                                             throws=[row], refill_intervals=[])))
            (path/'o.jsonl').write_text(json.dumps(dict(wall_time_s=100., points_mm=[[0, 0, 900]])) + '\n')
            report = analyse(path/'session.json', path/'o.jsonl', path/'out', extract_only=True)
        self.assertEqual(report['accepted_throws'], 0)
        self.assertIn('source-stamp gap', report['rejected_throws'][0]['reason'])

    def test_resumed_sessions_pool_with_own_rows_and_exclusions(self):
        from analyze_local_calibration import analyse
        row = self.hardware_row()
        frames, truth, _ = hardware_like_flight(row['predicted_release_position_mm'],
                                                row['predicted_release_velocity_mm_s'])
        base = dict(affine_applied=False, signed_s_mm=105.65, plan_sha256='p', solver_sha256='s',
                    schedule_to_mocap_mm=[.3, -.6], refill_intervals=[])
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory); sessions, data = [], []
            for k, shift in enumerate((0., 50.)):              # second part ran later
                throw = dict(row, throw_idx=0, cell_idx=3+k, status='released', capture_complete=True,
                             nominal_release_wall_s=100.+shift, analysis_window_wall_s=[99.75+shift, 101.6+shift])
                (path/('s%d.json' % k)).write_text(json.dumps(dict(base, throws=[throw])))
                with (path/('o%d.jsonl' % k)).open('w') as stream:
                    for t, pts in frames:
                        stream.write(json.dumps(dict(wall_time_s=t+shift, points_mm=pts.tolist()))+'\n')
                sessions.append(path/('s%d.json' % k)); data.append(path/('o%d.jsonl' % k))
            report = analyse(sessions, data, path/'out', extract_only=True)
            self.assertEqual(report['accepted_throws'], 2)
            accepted = json.loads((path/'out'/'extraction.json').read_text())['accepted']
            self.assertEqual(sorted((r['session_index'], r['cell_idx']) for r in accepted), [(0, 3), (1, 4)])
            for r in accepted:
                np.testing.assert_allclose(r['landing_global_mm'][:2], truth, atol=1.5)
            report = analyse(sessions, data, path/'out2', exclude=['1:0'], extract_only=True)
            self.assertEqual(report['accepted_throws'], 1)
            (path/'s1.json').write_text(json.dumps(dict(base, solver_sha256='other', throws=[])))
            with self.assertRaisesRegex(ValueError, 'solver_sha256'):
                analyse(sessions, data, path/'out3', extract_only=True)

    def test_validation_verdict_against_preregistered_criteria(self):
        from analyze_local_calibration import validation_verdict
        rng = np.random.default_rng(4)
        good = rng.normal(0, 14, (80, 2)); core = np.arange(80) < 20
        self.assertEqual(validation_verdict(good, core, 80)['verdict'], 'PASS')
        self.assertEqual(validation_verdict(good + [10, 0], core, 80)['verdict'], 'FAIL')     # residual bias
        self.assertEqual(validation_verdict(good*2.2, core, 80)['verdict'], 'FAIL')           # too much scatter
        self.assertEqual(validation_verdict(good[:50], core[:50], 50)['verdict'], 'INCONCLUSIVE')
        self.assertEqual(validation_verdict(good, core, 100)['verdict'], 'INCONCLUSIVE')      # 20% rejected

    def test_corrected_session_only_checked_extract_only(self):
        from analyze_local_calibration import analyse
        row = dict(self.hardware_row(), cell_idx=0, status='released', capture_complete=True,
                   analysis_window_wall_s=[99.75, 101.6])
        frames, truth, _ = hardware_like_flight(row['predicted_release_position_mm'],
                                                row['predicted_release_velocity_mm_s'])
        session = dict(affine_applied=True, signed_s_mm=105.65, correction=dict(sha256='c', path='x', matrix=[]),
                       bb_pose=dict(position_mm=[0, 0, 0], yaw_offset_rad=0.), refill_intervals=[],
                       plan=dict(cells=[dict(cell_idx=0, dense=True)]), throws=[row])
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)
            (path/'s.json').write_text(json.dumps(session))
            with (path/'o.jsonl').open('w') as stream:
                for t, pts in frames:
                    stream.write(json.dumps(dict(wall_time_s=t, points_mm=pts.tolist()))+'\n')
            with self.assertRaisesRegex(ValueError, 'extract-only'):
                analyse(path/'s.json', path/'o.jsonl', path/'out')
            report = analyse(path/'s.json', path/'o.jsonl', path/'out', extract_only=True)
        self.assertEqual(report['mode'], 'validation_corrected_extraction')
        self.assertEqual(report['validation']['verdict'], 'INCONCLUSIVE')                     # one throw
        np.testing.assert_allclose(report['validation']['mean_mm'],
                                   np.asarray(report['before']['mean_vector_mm']), atol=1e-6)

    def test_forward_inverse_direction_and_rank(self):
        commands = np.array([[x, y] for x in (-300, 0, 300) for y in (-300, 0, 300)])
        matrix = np.array([[1.02, .01, 10], [-.02, .99, 20]])
        measured = np.c_[commands, np.ones(9)]@matrix.T
        forward, inverse = fit_forward(commands, measured)
        np.testing.assert_allclose(forward, matrix, atol=1e-10)
        np.testing.assert_allclose(np.c_[measured, np.ones(9)]@inverse.T, commands, atol=1e-10)
        with self.assertRaises(ValueError):
            fit_forward(commands[:3], measured[:3])

    @unittest.skipUnless(importlib.util.find_spec('mcap_ros2'), 'optional MCAP dependencies absent')
    def test_real_mcap_decoder(self):
        from mcap_ros2.writer import Writer
        schema = '''MocapDataSingle[] markers
bool aligned
builtin_interfaces/Time stamp
================================================================================
MSG: jugglebot_interfaces/MocapDataSingle
string label
geometry_msgs/Point position
================================================================================
MSG: geometry_msgs/Point
float64 x
float64 y
float64 z
================================================================================
MSG: builtin_interfaces/Time
int32 sec
uint32 nanosec
'''
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'sample.mcap'
            with path.open('wb') as stream:
                writer = Writer(stream)
                definition = writer.register_msgdef('jugglebot_interfaces/msg/MocapDataMulti', schema)
                writer.write_message('/mocap_data', definition,
                    dict(markers=[dict(label='', position=dict(x=1., y=2., z=3.)),
                                  # QTM gave the thrown ball rigid-body labels on hardware.
                                  dict(label='Base - 1', position=dict(x=4., y=5., z=6.))],
                         aligned=True, stamp=dict(sec=100, nanosec=0)), log_time=100_000_000_000)
                for stamp in (dict(sec=0, nanosec=0), dict(sec=110, nanosec=0)):
                    writer.write_message('/mocap_data', definition,
                        dict(markers=[], aligned=True, stamp=stamp), log_time=101_000_000_000)
                writer.finish()
            skipped = {}
            frames = list(observations(path, skipped))
            self.assertEqual(len(frames), 1)                    # bad-clock frames skipped, not fatal
            self.assertEqual(skipped, dict(unstamped=1, stamp_vs_record_clock_over_2s=1))
            self.assertEqual(frames[0][0], 100.)
            np.testing.assert_equal(frames[0][1], [[1, 2, 3], [4, 5, 6]])


if __name__ == '__main__':
    unittest.main()
