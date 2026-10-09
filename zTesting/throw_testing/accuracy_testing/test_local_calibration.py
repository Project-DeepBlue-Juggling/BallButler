"""Offline calibration contract tests: python -m unittest discover -s this_dir."""
import argparse
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest
import threading
import time
from types import SimpleNamespace

import run_local_calibration as calibration


def options(**overrides):
    values=dict(core=calibration.DEFAULT_CORE,logs=[],spacing_min=50.,spacing_max=200.,taper=200.,
                padding=500.,z=830.,repeats=2,core_repeats=5,seed=42)
    values.update(overrides)
    return argparse.Namespace(**values)


class PlanTests(unittest.TestCase):
    def test_recorder_sigint_exit_requires_requested_stop_and_valid_metadata(self):
        self.assertTrue(calibration.recorder_shutdown_ok(0, False, True))
        self.assertTrue(calibration.recorder_shutdown_ok(2, True, True))
        for args in [(2, False, True), (2, True, False), (0, True, False),
                     (1, True, True), (None, True, True), (2, True, True, True)]:
            self.assertFalse(calibration.recorder_shutdown_ok(*args))

    def test_ros_uint8_goal_uuid_survives_checkpoint_and_event_json(self):
        import numpy as np
        goal_id = SimpleNamespace(uuid=np.arange(16, dtype=np.uint8))
        row = dict(status='accepted', goal_uuid=calibration.goal_uuid_bytes(goal_id))
        self.assertTrue(all(type(value) is int for value in row['goal_uuid']))
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'session.json'
            calibration.write_json(path, dict(throws=[row]))
            self.assertEqual(json.loads(path.read_text())['throws'][0]['goal_uuid'], list(range(16)))
            # Both the result event and failure/final checkpoints include this row.
            self.assertEqual(json.loads(json.dumps(dict(kind='result', throw=row)))['throw'], row)
            row['status'] = 'unknown_do_not_retry'
            calibration.write_json(path, dict(status='failed', throws=[row]))
            self.assertEqual(json.loads(path.read_text())['throws'][0]['status'], 'unknown_do_not_retry')

    def qtm_stamps(self, seconds=1.5, start=1791350039.):
        # 300 Hz QTM frames, republished by a 200 Hz timer: 3.3/6.7 ms steps, some repeats.
        return [start + int(k*1.5)/300. for k in range(int(seconds*200))]

    def test_runner_stall_is_not_a_stream_gap(self):
        # Regression, 2026-10-07: checkpoint I/O delayed the runner's callbacks
        # by ~100 ms (it reported 0.102 s and stopped) while the bag's QTM
        # stamps never gapped >21 ms. Queued frames keep their source stamps.
        monitor = calibration.StreamMonitor()
        for stamp in self.qtm_stamps():
            monitor.update(stamp)
        self.assertIsNone(monitor.problem())
        self.assertLess(monitor.max_gap, .01)

    def test_real_gap_and_qtm_restart_reject_capture(self):
        stamps = self.qtm_stamps()
        monitor = calibration.StreamMonitor()
        for stamp in stamps[:100] + stamps[130:]:          # 150 ms outage
            monitor.update(stamp)
        self.assertIn('gap', monitor.problem())
        monitor.reset()
        for stamp in stamps[:100] + [s - 2877. for s in stamps[100:]]:  # QTM capture restart
            monitor.update(stamp)
        self.assertIn('stepped back', monitor.problem())
        monitor.reset()
        for stamp in stamps[:50] + [0.] + stamps[51:]:
            monitor.update(stamp)
        self.assertIn('unsynchronised', monitor.problem())
        monitor.reset()
        self.assertIn('no stamped', monitor.problem())

    def test_default_refill_pauses_only_between_nine_throw_batches(self):
        self.assertEqual([i for i in range(29) if calibration.refill_due(i)],[0,9,18,27])
        self.assertEqual([i for i in range(10) if calibration.refill_due(i,3)],[0,3,6,9])

    def test_refill_projectiles_are_excluded_even_on_same_spatial_arc(self):
        session=dict(throws=[dict(throw_idx=7,status='released',capture_complete=True,
                                  analysis_window_wall_s=[100,102])],
                     refill_intervals=[dict(start_wall_s=103,end_wall_s=200)])
        # The same ballistic arc observed during refill has no assignment:
        # trajectory shape/order plays no role in this temporal exclusion.
        for dt in (0,.2,.7,1.5):
            self.assertEqual(calibration.analysis_throw_at(session,100+dt),7)
            self.assertIsNone(calibration.analysis_throw_at(session,110+dt))
        self.assertIsNone(calibration.analysis_throw_at(session,102))
        self.assertIsNone(calibration.analysis_throw_at(session,99.99))

    def test_unknown_incomplete_and_overlapping_captures_not_paired(self):
        row=dict(throw_idx=7,status='released',capture_complete=True,analysis_window_wall_s=[100,102])
        for changes in ({'status':'unknown_do_not_retry'},{'status':'failed'},{'capture_complete':False}):
            self.assertIsNone(calibration.analysis_throw_at({'throws':[dict(row,**changes)]},101))
        self.assertIsNone(calibration.analysis_throw_at({'throws':[row,dict(row,throw_idx=8)]},101))
        self.assertIsNone(calibration.analysis_throw_at({'throws':[row],
            'refill_intervals':[dict(start_wall_s=100,end_wall_s=None)]},101))

    def test_flight_wait_extends_past_ground_not_just_catch_plane(self):
        duration=calibration.flight_capture([0,0,1800],[2000,0,2000],.7,0)
        ground=(2000+math.sqrt(2000**2+2*9806*1800))/9806
        self.assertAlmostEqual(duration,ground+.5)
        self.assertGreater(duration,.7)
        with self.assertRaises(ValueError): calibration.flight_capture([0,0,1800],[0,0,2000],.7,1900)

    def test_refill_input_keeps_ros_pumping_and_waits_for_enter(self):
        entered=threading.Event(); calls=[]
        def read():
            if not entered.wait(2): raise RuntimeError('test timed out')
            return ''
        def pump():
            calls.append(1)
            if len(calls)>=3: entered.set()
            time.sleep(.001)
        calibration.wait_for_operator(pump,read)
        self.assertGreaterEqual(len(calls),3)

    def test_refill_quit_and_eof_do_not_resume(self):
        with self.assertRaises(KeyboardInterrupt):
            calibration.wait_for_operator(lambda:time.sleep(.001),lambda:'q')
        def eof(): raise EOFError()
        with self.assertRaisesRegex(RuntimeError,'input closed'):
            calibration.wait_for_operator(lambda:time.sleep(.001),eof)

    def test_refill_flushes_stray_keys_and_reprompts_on_junk(self):
        # A stray Enter typed earlier must not skip a refill, and junk input
        # must not end a long session: flush first, re-prompt on junk.
        events=[]; answers=iter(['oops',''])
        def read(): events.append('read'); return next(answers)
        calibration.wait_for_operator(lambda:time.sleep(.001),read,
            flush=lambda:events.append('flush'),say=lambda text:events.append('say'))
        self.assertEqual(events,['flush','read','say','flush','read'])

    def test_only_yaw_not_settled_aborts_are_retried(self):
        # 2026-10-08 sessions stopped on THROW_ABORTED_NOT_SETTLED (axis=YAW);
        # the firmware aborts before hand motion and keeps the ball (IDLE with
        # ball_in_hand 0.16 s later in both bags), so those are re-sent.
        retry=calibration.settle_abort_retryable
        self.assertTrue(retry(41,0)); self.assertTrue(retry(41,2))     # YAW, BOTH
        self.assertFalse(retry(41,1))                                    # PITCH only
        for outcome in (0,1,2,3,4,32,33,34,35,36,37,38):
            self.assertFalse(retry(outcome,0))

    def candidate(self, directory, **provenance):
        values=dict(frame='BB-local XY mm',signed_s_mm=105.65,requires_corrected_positive_s=True,
                    target_bounds_bb_local_mm=[[300,100],[1600,1000]],solver_sha256='bbb')
        values.update(provenance)
        path=Path(directory)/'candidate.json'
        path.write_text(json.dumps(dict(matrix=[[.93,-.02,37.6],[-.017,.985,15.6]],provenance=values)))
        return path

    def test_correction_requires_matching_positive_s_and_bb_local_frame(self):
        with tempfile.TemporaryDirectory() as directory:
            loaded=calibration.load_correction(self.candidate(directory),105.65)
            self.assertEqual(len(loaded['sha256']),64)
            for bad,s in ((dict(signed_s_mm=-105.65),105.65),({},-105.65),(dict(frame='mocap'),105.65)):
                with self.assertRaises(ValueError):
                    calibration.load_correction(self.candidate(directory,**bad),s)

    def fit_session(self, directory, solver, n=3):
        throws=[]
        for i in range(n):
            x,y,z=1000.+100*i,400.+50*i,-900.
            sol=solver(x,y,z,yaw_s_offset_mm=105.65)
            throws.append(dict(throw_idx=i,status='released',target_bb_local_mm=[x,y,z],
                               solution=dict(yaw_rad=sol.yaw_rad,pitch_rad=sol.pitch_rad,speed_mps=sol.speed_mps,tof_s=sol.tof_s)))
        throws.append(dict(throw_idx=n,status='aborted',target_bb_local_mm=[1.,1.,1.]))   # never compared
        path=Path(directory)/'session.json'; path.write_text(json.dumps(dict(throws=throws)))
        return hashlib.sha256(path.read_bytes()).hexdigest()

    def test_a_different_solver_file_is_accepted_only_if_it_reproduces_the_fit_solutions(self):
        # The sha guard alone would refuse the 2026-10-09 yaw-root fix, which
        # changes no command in the calibrated region; judge behaviour instead.
        def solver(x,y,z,yaw_s_offset_mm): return SimpleNamespace(yaw_rad=x/1e4,pitch_rad=y/1e4,speed_mps=3.,tof_s=.9)
        def other(x,y,z,yaw_s_offset_mm): return SimpleNamespace(yaw_rad=x/1e4+1e-6,pitch_rad=y/1e4,speed_mps=3.,tof_s=.9)
        with tempfile.TemporaryDirectory() as directory:
            sha=self.fit_session(directory,solver)
            analysis=Path(directory)/'analysis'; analysis.mkdir()
            correction=calibration.load_correction(self.candidate(analysis,session_sha256=sha),105.65)
            self.assertEqual(calibration.solver_reproduces_fit(correction,solver,105.65),3)
            with self.assertRaisesRegex(RuntimeError,'differs'):
                calibration.solver_reproduces_fit(correction,other,105.65)
            stale=calibration.load_correction(self.candidate(analysis,session_sha256='0'*64),105.65)
            with self.assertRaisesRegex(RuntimeError,'changed'):
                calibration.solver_reproduces_fit(stale,solver,105.65)
            with self.assertRaisesRegex(RuntimeError,'missing'):
                calibration.solver_reproduces_fit(calibration.load_correction(self.candidate(directory,session_sha256=sha),105.65),solver,105.65)

    def test_validation_schedule_commands_corrected_point_keeps_desired_target(self):
        with tempfile.TemporaryDirectory() as directory:
            correction=calibration.load_correction(self.candidate(directory),105.65)
        solved=[]
        def solver(x,y,z,yaw_s_offset_mm):
            solved.append((x,y)); return SimpleNamespace(yaw_rad=0.,pitch_rad=1.,speed_mps=3.,tof_s=.9)
        plan=dict(schedule=[dict(throw_idx=0,cell_idx=0,target_mm=[1000.,500.,830.]),
                            dict(throw_idx=1,cell_idx=1,target_mm=[2000.,500.,830.])])
        identity=lambda x,y,z,bb_position_mm,yaw_offset_rad:(x,y,z)
        good,bad=calibration.prepare_schedule(plan,[0,0,0],0.,[0.,0.],solver,identity,105.65,correction)
        self.assertEqual(good[0]['target_bb_local_mm'][:2],[1000.,500.])         # desired point kept
        self.assertAlmostEqual(good[0]['command_bb_local_mm'][0],.93*1000-.02*500+37.6)
        self.assertEqual(solved,[tuple(good[0]['command_bb_local_mm'][:2])])     # solver sees the command
        self.assertIn('outside',bad[0]['reason'])                                 # never extrapolated
        with self.assertRaisesRegex(ValueError,'different aim correction'):
            calibration.completed_throws([dict(plan_sha256='p',signed_s_mm=105.65,schedule_to_mocap_mm=[0.,0.],
                                               affine_applied=False,throws=[])],'p',105.65,[0.,0.],None,correction['sha256'])

    def test_bad_capture_rejects_throw_and_stops_only_when_consecutive(self):
        state=0; outcomes=[]
        for problem in [None,'gap',None,'gap','gap','gap']:
            fields,state,stop=calibration.capture_decision(problem,state)
            outcomes.append((fields['capture_complete'],state,bool(stop)))
            if not fields['capture_complete']: self.assertEqual(fields['capture_rejected'],'gap')
        self.assertEqual(outcomes,[(True,0,False),(False,1,False),(True,0,False),
                                   (False,1,False),(False,2,False),(False,3,True)])

    def test_resume_skips_only_cleanly_captured_releases(self):
        base=dict(plan_sha256='p',signed_s_mm=105.65,schedule_to_mocap_mm=[.3,-.6],
                  solver_sha256='bbb',affine_applied=False)
        rows=[dict(throw_idx=0,status='released',capture_complete=True),
              dict(throw_idx=1,status='released',capture_complete=False,capture_rejected='gap'),
              dict(throw_idx=2,status='unknown_do_not_retry'),
              dict(throw_idx=3,status='failed'),
              dict(throw_idx=4,status='released')]               # session stopped mid-capture
        later=dict(base,throws=[dict(throw_idx=5,status='released',capture_complete=True)])
        done=calibration.completed_throws([dict(base,throws=rows),later],'p',105.65,(.3,-.6),'bbb')
        self.assertEqual(done,{0,5})
        for key,value in (('plan_sha256','other'),('signed_s_mm',-105.65),
                          ('schedule_to_mocap_mm',[0.,0.]),('solver_sha256','3b4'),('affine_applied',True)):
            with self.assertRaisesRegex(ValueError,'Cannot resume'):
                calibration.completed_throws([dict(base,throws=rows,**{key:value})],'p',105.65,(.3,-.6),'bbb')

    def test_spacing_grows_and_is_bounded(self):
        plan=calibration.make_plan(options())
        for name in ('xs_mm','ys_mm'):
            axis=plan['grid'][name]
            gaps=[b-a for a,b in zip(axis,axis[1:])]
            self.assertGreaterEqual(min(gaps),50-1e-5)
            self.assertLessEqual(max(gaps),200+1e-5)
            self.assertAlmostEqual(max(gaps),200)
            outward=gaps[len(gaps)//2:]
            self.assertTrue(all(b>=a-1e-5 for a,b in zip(outward,outward[1:])))
        self.assertEqual(len(plan['cells']),143)
        self.assertEqual(len(plan['schedule']),331)

    def test_shuffle_reproducible_balanced_blocks(self):
        a=calibration.make_plan(options()); b=calibration.make_plan(options()); c=calibration.make_plan(options(seed=43))
        self.assertEqual(a,b); self.assertNotEqual(a['schedule'],c['schedule'])
        for cell in a['cells']:
            self.assertEqual(sum(e['cell_idx']==cell['cell_idx'] for e in a['schedule']),cell['repeats'])
        for repetition in range(5):
            block=[e['cell_idx'] for e in a['schedule'] if e['repeat']==repetition]
            self.assertEqual(len(block),len(set(block)))
        self.assertNotEqual([e['cell_idx'] for e in a['schedule'][:143]],list(range(143)))

    def test_core_logs_only_columns_and_correct_bounds(self):
        with tempfile.TemporaryDirectory() as folder:
            log=Path(folder)/'launch.log'
            log.write_text('hop started: hi\nCATCH-AIM skill 1: source=schedule landing=(999, 0, 830) mm\n'
                           'columns (reload) started: hi\nCATCH-AIM skill 1: source=schedule landing=(-52.5, 0, 830) mm\n'
                           'CATCH-AIM skill 2: source=fit landing=(62.5, 3, 830) mm\n'
                           'self_toss started: hi\nCATCH-AIM skill 1: source=fit landing=(777, 0, 830) mm\n',encoding='utf-8')
            core,points=calibration.core_from_logs([log])
            self.assertEqual(core,[-52.5,62.5,0,3]); self.assertEqual(len(points),2)

    def test_invalid_inputs(self):
        for changes in ({'spacing_min':0},{'taper':0},{'padding':-1},{'z':float('nan')},
                        {'repeats':0},{'core_repeats':1},{'core':[1,-1,0,0]}):
            with self.assertRaises(ValueError): calibration.make_plan(options(**changes))

    def test_bad_or_empty_plan_refused(self):
        plan=calibration.make_plan(options()); calibration.validate_plan(plan)
        plan['schedule'][0]['target_mm'][2]=float('nan')
        with self.assertRaises(ValueError): calibration.validate_plan(plan)
        with self.assertRaises(ValueError): calibration.validate_plan({'schema':calibration.SCHEMA,'frame':'schedule_xy_world_z','schedule':[]})

    def test_persistence_and_preview(self):
        with tempfile.TemporaryDirectory() as folder:
            plan=calibration.make_plan(options()); target=Path(folder)/'plan.json'
            calibration.write_json(target,plan); calibration.preview(plan,target.with_suffix('.html'))
            self.assertEqual(json.loads(target.read_text()),plan)
            self.assertFalse(target.with_suffix('.json.tmp').exists())
            self.assertIn('331 throws',target.with_suffix('.html').read_text())

    def test_unreachable_entries_are_explicitly_preserved(self):
        plan=calibration.make_plan(options())
        def transform(x,y,z,**kwargs): return x,y,z
        def solver(x,y,z,**kwargs): raise ValueError('unreachable')
        good,bad=calibration.prepare_schedule(plan,[0,0,0],0,[10,20],solver,transform,105.65)
        self.assertFalse(good); self.assertEqual(len(bad),len(plan['schedule']))
        self.assertEqual(bad[0]['reason'],'unreachable')

    def test_streams_require_fresh_data_and_idle(self):
        cache=dict(heartbeat=SimpleNamespace(connected=True),hb_at=10,mocap_at=10,state='IDLE',state_at=10)
        self.assertTrue(calibration.streams_ready(cache,10.5))
        self.assertFalse(calibration.streams_ready(cache,11.1))
        for key,value in [('state','ACTIVE:SKILL'),('hb_at',0),('state_at',0),('heartbeat',None)]:
            modified=dict(cache,**{key:value}); self.assertFalse(calibration.streams_ready(modified,10.5))

    def test_empty_or_missing_raw_capture_is_not_success(self):
        names=['/mocap_data','/bb/local_calibration/event','/bb/heartbeat','/bb/calibration_result']
        rows=[dict(topic_metadata=dict(name=n),message_count=4) for n in names]
        metadata=dict(rosbag2_bagfile_information=dict(topics_with_message_count=rows))
        self.assertEqual(calibration.check_recorded_topics(metadata)['/mocap_data'],4)
        rows[0]['message_count']=0
        with self.assertRaisesRegex(ValueError,'mocap_data'): calibration.check_recorded_topics(metadata)
        with self.assertRaises(ValueError): calibration.check_recorded_topics({})

    def test_production_corrected_solver_hits_world_targets(self):
        root=Path(__file__).resolve().parents[4]/'Jugglebot'
        if not root.exists(): self.skipTest('Sibling Jugglebot checkout not available')
        sys.path.insert(0,str(root/'ros_ws/src/jugglebot'))
        spec=importlib.util.spec_from_file_location('calibration_test_ballistics',root/'ros_ws/src/jugglebot/jugglebot/can/throw_ballistics.py')
        module=importlib.util.module_from_spec(spec);sys.modules[spec.name]=module;spec.loader.exec_module(module)
        plan=calibration.make_plan(options())
        position=[-892.50197,-191.93485,1741.18188]; yaw=-.14581998
        good,bad=calibration.prepare_schedule(plan,position,yaw,[12,-8],module.solve_throw_local,module.global_to_bb_local,105.65)
        self.assertTrue(good);self.assertEqual(len(good)+len(bad),331)
        for entry in good:
            sol=entry['solution'];t=sol['tof_s']
            r,v=module.bb_release_state(sol['yaw_rad'],sol['pitch_rad'],sol['speed_mps'],position,yaw,yaw_s_offset_mm=105.65)
            landing=[r[i]+v[i]*t-(.5*9806*t*t if i==2 else 0) for i in range(3)]
            self.assertLess(math.sqrt(sum((a-b)**2 for a,b in zip(landing,entry['target_global_mm']))),1e-5)
            self.assertAlmostEqual(entry['target_global_mm'][0]-entry['target_mm'][0],12)


if __name__=='__main__': unittest.main()
