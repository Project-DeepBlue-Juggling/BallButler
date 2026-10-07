import importlib.util
import json
import tempfile
import unittest
from pathlib import Path
import numpy as np
from analyze_local_calibration import extract, fit_forward, observations


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
                    dict(markers=[dict(label='', position=dict(x=1., y=2., z=3.))],
                         aligned=True, stamp=dict(sec=100, nanosec=0)), log_time=100_000_000_000)
                writer.finish()
            frames = list(observations(path))
            self.assertEqual(frames[0][0], 100.)
            np.testing.assert_equal(frames[0][1], [[1, 2, 3]])


if __name__ == '__main__':
    unittest.main()
