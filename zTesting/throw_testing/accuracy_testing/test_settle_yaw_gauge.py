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

    def test_inconclusive_report_has_its_affine_criteria_evaluated_here(self):
        # 2026-10-10 23:34 sitting: 47 accepted < the report's 60, so its validation
        # is INCONCLUSIVE with no 'checks'; RMS 24.6 is inside 26 and there are no core throws.
        crit = dict(max_abs_mean_mm=6.0, max_rms_mm=26.0, max_core_rms_mm=25.0,
                    min_accepted=60, min_core_accepted=10, min_accepted_fraction=0.9)
        session, accepted = synthetic(0.3)
        rows = settle.bearing_rows(accepted, POSE)
        report = dict(validation=dict(criteria=crit, n=47, n_core=0, mean_mm=[-8.1, -8.7], rms_mm=24.6,
                                      core_rms_mm=None, verdict='INCONCLUSIVE'))
        result = settle.settle(session, rows, report)
        self.assertTrue(result['affine_rms_ok'])
        self.assertEqual(result['affine_checks'], dict(rms=True, core_rms=True, mean=False))
        text = '\n'.join(settle.describe(result))
        self.assertIn('within criteria (RMS 24.6 <= 26, core n/a', text)
        self.assertNotIn('OUTSIDE criteria', text)
        self.assertIn('mean landing error (-8.1, -8.7) mm: OUTSIDE', text)
        # Above the RMS criterion, or a core RMS above its own, it is OUTSIDE.
        report['validation'].update(rms_mm=27.0)
        result = settle.settle(session, rows, report)
        self.assertFalse(result['affine_rms_ok'])
        self.assertIn('OUTSIDE criteria', '\n'.join(settle.describe(result)))
        report['validation'].update(rms_mm=24.6, core_rms_mm=25.5)
        self.assertFalse(settle.settle(session, rows, report)['affine_rms_ok'])
        # The report's own checks win when present; nothing to judge without criteria or RMS.
        report['validation'].update(checks=dict(mean=True, rms=True, core_rms=True))
        result = settle.settle(session, rows, report)
        self.assertTrue(result['affine_rms_ok']); self.assertEqual(result['affine_checks_source'], 'report')
        result = settle.settle(session, rows, dict(validation=dict(verdict='INCONCLUSIVE', rms_mm=None)))
        self.assertIsNone(result['affine_rms_ok'])
        self.assertIn('not evaluated', '\n'.join(settle.describe(result)))

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
