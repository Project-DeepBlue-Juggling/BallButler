"""Offline mirrored-hand experiment; no hardware/configuration changes.

Run with Python + numpy; --jugglebot points at the control-stack checkout.
The exact landing shift is independent of pitch, speed, and flight time:
delta_xy = (s_actual-s_model)*(-sin(yaw), cos(yaw)).
"""
import argparse
import json
import math
import sys
import importlib.util
from pathlib import Path
import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--jugglebot', type=Path, default=Path(__file__).resolve().parents[4] / 'Jugglebot')
    args = parser.parse_args()
    sys.path.insert(0, str(args.jugglebot / 'ros_ws/src/jugglebot'))
    spec = importlib.util.spec_from_file_location('offline_ballistics', args.jugglebot / 'ros_ws/src/jugglebot/jugglebot/can/throw_ballistics.py')
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    yaw_solve_thetas, solve_throw_local, bb_release_state = module.yaw_solve_thetas, module.solve_throw_local, module.bb_release_state
    root = Path(__file__).resolve().parent
    original = json.loads((root / 'throw_affine_correction.json').read_text())
    affine = np.array(original['matrix'])[:2]
    s = -105.65

    def shift(xy):
        yaw = yaw_solve_thetas(*xy, s)[2]
        return -2*s*np.array([-math.sin(yaw), math.cos(yaw)])

    def stats(errors):
        norms = np.linalg.norm(errors, axis=1)
        return dict(mean_mm=float(norms.mean()), rms_mm=float(np.sqrt(np.mean(norms**2))),
                    max_mm=float(norms.max()), mean_vector_mm=errors.mean(axis=0).tolist())

    results = {}
    for name, filename, corrected in [('uncorrected', 'throw_affine_correction.json', False),
                                       ('affine_validation', 'throw_affine_correction_v2_residual.json', True)]:
        data = json.loads((root / filename).read_text())['provenance']
        pairs = data['pairs_used']
        target = np.array([p['target_xy_bb_local'] for p in pairs])
        measured = np.array([p['landing_xy_bb_local'] for p in pairs])
        command = np.c_[target, np.ones(len(target))] @ affine.T if corrected else target
        predicted = command + np.array([shift(p) for p in command])
        groups = sorted(set(p['cell_idx'] for p in pairs))
        means = lambda a: np.array([a[[p['cell_idx']==g for p in pairs]].mean(axis=0) for g in groups])
        t,m,p = map(means, (target, measured, predicted))
        results[name] = dict(n_throws=len(pairs), n_cells=len(groups),
            measured_error=stats(m-t), simulated_error=stats(p-t),
            measured_minus_simulated=stats(m-p),
            cell_results=[dict(cell=g,target=tt.tolist(),measured=mm.tolist(),simulated=pp.tolist())
                          for g,tt,mm,pp in zip(groups,t,m,p)])
        if not corrected:
            observed, modelled = m-t, p-t
            cosines = np.sum(observed*modelled,axis=1)/(np.linalg.norm(observed,axis=1)*np.linalg.norm(modelled,axis=1))
            results[name]['mean_direction_difference_deg'] = float(np.degrees(np.arccos(np.clip(cosines,-1,1))).mean())
            ideal_affine = np.linalg.lstsq(np.c_[p,np.ones(len(p))],t,rcond=None)[0].T
            corrected_commands = np.c_[t,np.ones(len(t))] @ ideal_affine.T
            ideal_landings = corrected_commands + np.array([shift(c) for c in corrected_commands])
            results[name]['mirror_only_fitted_affine'] = ideal_affine.tolist()
            results[name]['mirror_only_affine_forward_residual'] = stats(ideal_landings-t)

    # Independently verify the closed-form shift using production 3D ballistics.
    checks = 0
    for xy in ([600,0], [900,200], [1200,500]):
        for z in (-1000,-700):
            sol = solve_throw_local(*xy,z)
            for actual_s in (s,-s):
                r,v = bb_release_state(sol.yaw_rad,sol.pitch_rad,sol.speed_mps,(0,0,0),yaw_s_offset_mm=actual_s)
                landing = np.array(r)+np.array(v)*sol.tof_s
                landing[2] -= .5*9806*sol.tof_s**2
                expected = np.array([*xy,z],float)
                if actual_s != s:
                    expected[:2] += shift(xy)
                assert np.linalg.norm(landing-expected)<1e-5, (landing,expected)
                checks += 1
    results['forward_ballistic_checks'] = checks
    # Illustrative feed locations under the historical pose; NOT today's pose.
    pose = np.array(original['provenance']['bb_mocap_position_mm_at_fit'][:2])
    angle = original['provenance']['bb_yaw_offset_rad_at_fit']
    rotation = np.array([[math.cos(angle),-math.sin(angle)],[math.sin(angle),math.cos(angle)]])
    results['historical_pose_examples'] = []
    for global_xy in ([-50,0],[-40,0],[0,0],[50,0]):
        local = rotation.T @ (np.array(global_xy)-pose)
        command = affine @ np.r_[local,1]
        error = rotation @ (command+shift(command)-local)
        results['historical_pose_examples'].append(dict(target_global_mm=global_xy,affine_mirror_error_global_mm=error.tolist()))
    (root/'mirrored_hand_results.json').write_text(json.dumps(results,indent=2)+'\n')
    try:
        from render_mirrored_comparison import render
        render()
    except ImportError:
        print('Results saved. Install Pillow or run render_mirrored_comparison.py in an environment with Pillow for PNG/SVG plots.')
    print(json.dumps({k:({a:b for a,b in v.items() if a!='cell_results'} if isinstance(v,dict) else v) for k,v in results.items()},indent=2))


if __name__ == '__main__':
    main()
