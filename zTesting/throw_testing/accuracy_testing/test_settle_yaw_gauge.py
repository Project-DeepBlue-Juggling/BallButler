import json
import math
import os
import tempfile
import unittest
from pathlib import Path

import settle_yaw_gauge as settle

POSE = dict(position_mm=[-975.0, -389.0, 1735.0], yaw_offset_rad=math.radians(0.681))
SESSION_B = Path('/home/jetson/bb_calibration_sessions/20261009T031319_612936Z/session.json')


def synthetic(offset_error_deg, n=40, corrected=True):
    """Targets on a fan; landings rotated by -offset_error about BB's yaw axis in
    the node's local frame (what a yaw-offset error does), then put in world."""
    session = dict(bb_pose=POSE, correction=dict(sha256='x' * 64) if corrected else None)
    accepted = []
    c, s = math.cos(POSE['yaw_offset_rad']), math.sin(POSE['yaw_offset_rad'])
    for i in range(n):
        rng = 500.0 + 1000.0 * (i % 10) / 9.0
        bearing = math.radians(10.0 + 50.0 * ((i * 7) % 11) / 10.0)
        xt, yt = rng * math.cos(bearing), rng * math.sin(bearing)
        r = math.radians(-offset_error_deg)
        xl, yl = xt * math.cos(r) - yt * math.sin(r), xt * math.sin(r) + yt * math.cos(r)
        gx, gy = POSE['position_mm'][0] + xl * c - yl * s, POSE['position_mm'][1] + xl * s + yl * c
        accepted.append(dict(throw_idx=i, target_bb_local_mm=[xt, yt, -905.0], landing_global_mm=[gx, gy, 830.0]))
    return session, accepted


class SettleTests(unittest.TestCase):
    def test_offset_error_is_recovered_with_the_session_b_sign_convention(self):
        session, accepted = synthetic(+0.47)
        result = settle.settle(session, settle.bearing_rows(accepted, POSE))
        self.assertEqual(result['verdict'], 'RE_PIN')
        self.assertAlmostEqual(result['bearing_mean_deg'], -0.47, places=6)
        self.assertAlmostEqual(result['gauge_delta_deg'], -0.47, places=6)
        self.assertAlmostEqual(result['implied_frame_yaw_offset_deg'], 0.681 - 0.47, places=6)
        self.assertAlmostEqual(result['rotation_fit']['slope_deg'], -0.47, places=3)
        self.assertAlmostEqual(result['rotation_fit']['intercept_mm'], 0.0, places=3)

    def test_small_error_confirms_the_frame_and_few_throws_are_inconclusive(self):
        session, accepted = synthetic(0.05)
        self.assertEqual(settle.settle(session, settle.bearing_rows(accepted, POSE))['verdict'], 'FRAME_CONFIRMED')
        session, accepted = synthetic(0.5, n=12)
        self.assertEqual(settle.settle(session, settle.bearing_rows(accepted, POSE))['verdict'], 'INCONCLUSIVE')

    def test_uncorrected_sessions_are_refused(self):
        session, accepted = synthetic(0.3, corrected=False)
        with self.assertRaises(ValueError):
            settle.settle(session, settle.bearing_rows(accepted, POSE))

    def test_cli_writes_the_settlement_next_to_the_extraction(self):
        session, accepted = synthetic(0.3)
        with tempfile.TemporaryDirectory() as d:
            (Path(d) / 'analysis').mkdir()
            (Path(d) / 'session.json').write_text(json.dumps(session))
            (Path(d) / 'analysis' / 'extraction.json').write_text(json.dumps(dict(accepted=accepted, rejected=[])))
            (Path(d) / 'analysis' / 'extraction_report.json').write_text(json.dumps(dict(validation=dict(
                rms_mm=21.0, checks=dict(rms=True, core_rms=True)))))
            self.assertEqual(settle.main([str(Path(d) / 'session.json')]), 0)
            out = json.loads((Path(d) / 'analysis' / 'yaw_gauge_settlement.json').read_text())
            self.assertEqual(out['verdict'], 'RE_PIN'); self.assertTrue(out['affine_rms_ok'])

    @unittest.skipUnless(SESSION_B.is_file() and (SESSION_B.parent / 'analysis' / 'extraction.json').is_file(),
                         'session B data not on this machine')
    def test_session_b_real_data_recovers_the_known_frame_error(self):
        # Session B (2026-10-09) used 0.681 deg; the affine was fitted in the 0.208 deg frame.
        session = json.loads(SESSION_B.read_text())
        accepted = json.loads((SESSION_B.parent / 'analysis' / 'extraction.json').read_text())['accepted']
        result = settle.settle(session, settle.bearing_rows(accepted, session['bb_pose']))
        self.assertEqual(result['verdict'], 'RE_PIN')
        self.assertAlmostEqual(result['bearing_mean_deg'], -0.47, delta=0.10)
        self.assertAlmostEqual(result['implied_frame_yaw_offset_deg'], 0.208, delta=0.12)


if __name__ == '__main__':
    unittest.main()
