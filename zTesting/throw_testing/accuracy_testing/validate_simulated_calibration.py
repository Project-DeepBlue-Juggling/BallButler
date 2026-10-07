#!/usr/bin/env python3
"""Independent truth check and fresh physical throws for a synthetic calibration.

Only this validation utility reads truth.json. The production analysis does not.
Corrected commands are re-solved through production geometry and new random
velocity errors; validation is not just evaluating a fitted affine algebraically.
"""
import argparse
import json
import math
import random
from pathlib import Path

import numpy as np
from analyze_local_calibration import local, stats
from run_local_calibration import write_json
from simulate_local_calibration import load_ballistics, crossing_time, point_at


def validate(session_path, analysis, jugglebot, out, seed=72104, repeats=20):
    session_path, analysis = Path(session_path), Path(analysis)
    session = json.loads(session_path.read_text(encoding='utf-8'))
    truth = json.loads(session_path.with_name('truth.json').read_text(encoding='utf-8'))
    extraction = json.loads((analysis/'extraction.json').read_text(encoding='utf-8'))
    candidate = json.loads((analysis/'correction_candidate.json').read_text(encoding='utf-8'))
    inverse = np.asarray(candidate['matrix'])
    forward = np.asarray(truth['forward_affine_mocap_xy'])
    true_by_id = {r['throw_idx']: r for r in truth['throws']}
    errors = [np.asarray(r['landing_global_mm'][:2]) - true_by_id[r['throw_idx']]['actual_catch_mm'][:2]
              for r in extraction['accepted']]
    pose = session['bb_pose']; origin = np.asarray(pose['position_mm'])
    yaw = pose['yaw_offset_rad']; c, s = math.cos(yaw), math.sin(yaw)
    rotation = np.array([[c, -s], [s, c]])
    b = load_ballistics(jugglebot)
    rng = random.Random(seed)
    before, after, core_before, core_after, recovery = [], [], [], [], []
    error_fraction = truth['state_error_fraction']
    skipped = []
    for cell in session['plan']['cells']:
        target = np.asarray(cell['target_mm'], dtype=float)
        target[:2] += session['schedule_to_mocap_mm']
        target_local = local([target[:2]], pose)[0]
        corrected_local = inverse @ np.r_[target_local, 1.]
        corrected_world = corrected_local @ rotation.T + origin[:2]
        ideal_corrected = np.linalg.solve(forward[:, :2], target[:2] - forward[:, 2])
        recovery.append(corrected_world - ideal_corrected)
        states = []
        try:
            for command_xy in (target[:2], corrected_world):
                xyz = [*command_xy, target[2]]
                cmd = b.global_to_bb_local(*xyz, bb_position_mm=origin, yaw_offset_rad=yaw)
                sol = b.solve_throw_local(*cmd, yaw_s_offset_mm=session['signed_s_mm'])
                r, v = b.bb_release_state(sol.yaw_rad, sol.pitch_rad, sol.speed_mps, origin,
                                         yaw, yaw_s_offset_mm=session['signed_s_mm'])
                v = np.asarray(v)
                intended = forward @ np.r_[command_xy, 1.]
                v[:2] += (intended - command_xy) / sol.tof_s
                states.append((r, v))
        except ValueError as exc:
            skipped.append(dict(cell_idx=cell['cell_idx'], reason=str(exc)))
            continue
        for _ in range(repeats):
            # Paired new disturbances make comparisons fair; none are reused
            # from the captured calibration campaign.
            factors = np.array([rng.uniform(1-error_fraction, 1+error_fraction) for _ in range(3)])
            pair = []
            for r, v in states:
                actual_v = v * factors
                t = crossing_time(r, actual_v, target[2])
                pair.append(np.asarray(point_at(r, actual_v, t)[:2])-target[:2])
            before.append(pair[0]); after.append(pair[1])
            if cell['dense']:
                core_before.append(pair[0]); core_after.append(pair[1])
    result = dict(simulated=True, simulation_seed=truth['seed'],
        forward_affine_mocap_xy=truth['forward_affine_mocap_xy'],
        fitted_correction_bb_local_xy=candidate['matrix'],
        simulation_pose=pose, observation_noise_uniform_mm=truth['observation_noise_uniform_mm'],
        state_error_fraction=error_fraction, calibration_throws=len(session['throws']),
        accepted_throws=len(extraction['accepted']), rejected_throws=len(extraction['rejected']),
        extraction_vs_true_catch=stats(errors), correction_command_vs_true_inverse=stats(recovery),
        fresh_state_seed=seed, fresh_throws_per_cell=repeats,
        fresh_full_before=stats(before), fresh_full_after=stats(after),
        fresh_core_before=stats(core_before), fresh_core_after=stats(core_after),
        validation_skipped_cells=skipped,
        note='Synthetic validation only. Corrected commands re-solved with production positive-s geometry, systematic velocity distortion and fresh independent +/-1% velocity-component errors.')
    write_json(out, result)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('session', type=Path)
    parser.add_argument('--analysis', type=Path)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--jugglebot', type=Path, default=Path(__file__).resolve().parents[4]/'Jugglebot')
    args = parser.parse_args()
    result = validate(args.session, args.analysis or args.session.parent/'analysis', args.jugglebot, args.out)
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
