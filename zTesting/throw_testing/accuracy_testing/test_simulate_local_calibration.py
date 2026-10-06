"""Physical boundary checks for the offline synthetic capture generator."""
import unittest

from simulate_local_calibration import GRAVITY, bounced_point, crossing_time, point_at


class SimulationTests(unittest.TestCase):
    def test_descending_crossing(self):
        r, v = [1., 2., 1700.], [300., 200., 2200.]
        t = crossing_time(r, v, 830.)
        self.assertAlmostEqual(point_at(r, v, t)[2], 830.)
        self.assertLess(v[2] - GRAVITY * t, 0.)

    def test_bounce_roll_and_disappearance(self):
        r, v = [0., 0., 1700.], [1000., 0., 2000.]
        impact = crossing_time(r, v, 0.)
        self.assertIsNone(bounced_point(r, v, -0.01))
        self.assertAlmostEqual(bounced_point(r, v, impact)[2], 0., places=8)
        self.assertGreater(bounced_point(r, v, impact + .04)[2], 0.)
        self.assertEqual(bounced_point(r, v, impact + .5)[2], 0.)
        self.assertIsNotNone(bounced_point(r, v, impact + .999))
        self.assertIsNone(bounced_point(r, v, impact + 1.))
        self.assertGreater(bounced_point(r, v, impact + .5)[0], point_at(r, v, impact)[0])


if __name__ == '__main__':
    unittest.main()
