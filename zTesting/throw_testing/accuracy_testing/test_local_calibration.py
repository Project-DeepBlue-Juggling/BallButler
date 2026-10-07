"""Offline calibration contract tests: python -m unittest discover -s this_dir."""
import argparse
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
